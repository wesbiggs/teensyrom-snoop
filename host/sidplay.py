"""Turn a teensyrom-snoop stream of SID register writes into audio, with reSIDfp (pyresidfp).

  SidRenderer   the emulation: frames in, 16-bit samples out, clocked by C64 time
  LiveClock     where "now" is on the C64's timeline, from frame arrival times (for live play)
  LivePlayer    renderer + clock + a sample FIFO with slow drift correction, for an audio callback

pyresidfp (GPL-2.0-or-later) and numpy are needed at run time; this file only calls them.
"""
from __future__ import annotations

import collections
import threading
import time

import numpy as np

import snoopproto as sp

try:
    from pyresidfp._pyresidfp import SID, ChipModel, SamplingMethod
except ImportError:  # pragma: no cover - reported by the CLI
    SID = None

WARMUP = 1.2            # seconds each emulated SID runs, silent, before the stream starts
SID_REGS = 0x19          # $00-$18 are writable; $19-$1C are read-only (pots, osc3, env3)


class SidRenderer:
    """One reSIDfp per SID base address.  Mono mixes them; stereo puts SID 1 left, SID 2 right
    (more than two: odd ones left, even ones right)."""

    def __init__(self, bases=(0xD400,), sample_rate=48000, model="6581", stereo=False,
                 phi2_hz=sp.PHI2_PAL, filter=True):
        if SID is None:
            raise RuntimeError("needs pyresidfp: pip install pyresidfp numpy")
        self.bases = list(bases)
        self.rate = sample_rate
        self.model = model
        self.stereo = stereo
        self.filter = filter
        self.queue = collections.deque()      # (time, sid index, register, value), ascending time
        self.late = 0                         # writes that arrived after their time was rendered
        self.ignored = 0                      # writes outside every SID's registers
        self._make(phi2_hz)

    def _make(self, phi2_hz):
        self.phi2 = phi2_hz
        cm = ChipModel.MOS8580 if str(self.model) == "8580" else ChipModel.MOS6581
        self.sids = []
        for _ in self.bases:
            s = SID(cm, SamplingMethod.RESAMPLE, float(phi2_hz), float(self.rate))
            s.enable_filter(self.filter)
            s.clock(int(phi2_hz * WARMUP))   # the output's DC offset settles over ~1 s: not a thump at the start
            self.sids.append(s)
        self.cursor = 0                       # C64 cycles rendered
        self.queue.clear()

    # ---- input ------------------------------------------------------------------------------

    def push(self, frame):
        """A decoded frame.  HELLO starts a new timeline (the emulation is reset)."""
        if isinstance(frame, sp.Hello):
            self._make(frame.phi2_hz)
        elif isinstance(frame, sp.Writes):
            for b in frame.blocks:
                for k, v in enumerate(b.data):
                    self.write(b.time, b.addr + k, v)

    def write(self, t, addr, value):
        for i, base in enumerate(self.bases):
            reg = addr - base
            if 0 <= reg < SID_REGS:
                if t < self.cursor:
                    self.late += 1
                    t = self.cursor
                self.queue.append((t, i, reg, value))
                return
        self.ignored += 1

    # ---- output -----------------------------------------------------------------------------

    def _clock(self, cycles, out):
        if cycles <= 0:
            return
        chans = [s.clock(int(cycles)) for s in self.sids]
        n = min(len(c) for c in chans)
        if n:
            out.append([np.asarray(c[:n], dtype=np.int32) for c in chans])
        self.cursor += cycles

    def render_to(self, t_target):
        """Clock every SID up to C64 time t_target, applying the queued writes on their cycle.
        Returns int16 samples: shape (n,) mono or (n, 2) stereo."""
        out = []
        while self.queue and self.queue[0][0] <= t_target:
            t, i, reg, v = self.queue.popleft()
            self._clock(t - self.cursor, out)
            self.sids[i].write(reg, v)
        self._clock(t_target - self.cursor, out)
        return self._mix(out)

    def skip_to(self, t_target):
        """Like render_to, but the samples are thrown away (catching up after a stall)."""
        self.render_to(t_target)

    def _mix(self, parts):
        if not parts:
            return np.zeros((0, 2) if self.stereo else 0, dtype=np.int16)
        chans = [np.concatenate([p[i] for p in parts]) for i in range(len(self.sids))]
        if self.stereo:
            left = [c for i, c in enumerate(chans) if i % 2 == 0]
            right = [c for i, c in enumerate(chans) if i % 2 == 1] or left
            l = sum(left) // len(left)
            r = sum(right) // len(right)
            return np.clip(np.stack([l, r], axis=1), -32768, 32767).astype(np.int16)
        return np.clip(sum(chans) // len(chans), -32768, 32767).astype(np.int16)


def render_stream(data: bytes, **kw):
    """A whole recorded stream (trsnoop.py --raw) to samples, on the C64's own timeline.
    Ends at the last time the stream knows (its last STATUS or write)."""
    dec = sp.Decoder()
    r = SidRenderer(**kw)
    parts = []
    end = 0
    for f in dec.feed(data):
        if isinstance(f, sp.Hello) and parts:
            parts.append(r.render_to(end))      # the run so far, then the new run's timeline
            end = 0
        r.push(f)
        if isinstance(f, sp.Writes) and f.blocks:
            end = max(end, f.blocks[-1].time)
        elif isinstance(f, sp.Status):
            end = max(end, f.time)
        # render as we go so the queue stays short
        if r.queue and r.queue[-1][0] - r.cursor > r.phi2:
            parts.append(r.render_to(r.queue[-1][0] - r.phi2 // 2))
    parts.append(r.render_to(end))
    if not parts:
        return r, np.zeros(0, dtype=np.int16)
    return r, np.concatenate(parts)


class LiveClock:
    """Estimates the C64 time "now" from (C64 time, arrival wall time) pairs.

    Each frame says the TeensyROM had reached at least its time when it was sent, so
    time/phi2 - arrival is at most the true offset between the two clocks: keep the largest
    of the last `window` seconds (least transport delay), extrapolate with the nominal phi2.
    """

    def __init__(self, phi2_hz, window=4.0, now=time.monotonic):
        self.phi2 = phi2_hz
        self.window = window
        self.now = now
        self.marks = collections.deque()      # (wall, offset seconds)

    def seen(self, c64_time, wall=None):
        wall = self.now() if wall is None else wall
        off = c64_time / self.phi2 - wall
        while self.marks and self.marks[-1][1] <= off:     # sliding-window maximum: marks[0] is the best
            self.marks.pop()
        self.marks.append((wall, off))
        while len(self.marks) > 1 and wall - self.marks[0][0] > self.window:
            self.marks.popleft()

    def c64_now(self, wall=None):
        if not self.marks:
            return None
        wall = self.now() if wall is None else wall
        while len(self.marks) > 1 and wall - self.marks[0][0] > self.window:
            self.marks.popleft()
        return int((self.marks[0][1] + wall) * self.phi2)


class LivePlayer:
    """Glue for live play: feed() frames from the network thread, fill() from the audio callback.

    A producer step (pump) renders up to `latency` behind the estimated C64 now into a FIFO;
    the callback takes from it.  The FIFO level is held near `buffer` by dropping or repeating
    one sample at a time (C64 crystal against the sound card's: ~100 ppm, a few samples a second)."""

    AUTO_START = 0.06      # seconds behind the C64 until enough frames have been seen
    AUTO_MIN, AUTO_MAX = 0.015, 0.5
    AUTO_MARGIN = 0.010    # above the measured 99th-percentile frame delay
    AUTO_WINDOW = 6.0      # seconds of frame delays kept
    # How fast the latency may move, seconds per second.  Both must stay below what the drift correction can do
    # (MAX_STEP per callback: 15.6 ms/s at 48 kHz, 256-sample blocks).  Raising the latency pauses the renderer and
    # the FIFO drains: at 50 ms/s it ran dry (underruns, on hardware with Ms. Pac-Man); slower costs a few more late
    # writes while it rises, each applied a few ms off, which is far less audible than a gap.
    AUTO_UP, AUTO_DOWN = 0.012, 0.010
    JUMP_FIFOS = 4         # a jump of "now" by more than this many FIFO targets while playing is skipped, not queued
    MAX_STEP = 4           # samples dropped or repeated per sound card callback at most (4 per 256: 15.6 ms/s at 48 kHz)

    def __init__(self, renderer: SidRenderer, latency=None, buffer=0.03, now=time.monotonic, trim_start=True):
        """latency None: automatic, from how late frames arrive (99th percentile of the last
        AUTO_WINDOW seconds plus AUTO_MARGIN, moved at most AUTO_UP / AUTO_DOWN per second, so the
        sample-level drift correction absorbs the change)."""
        self.r = renderer
        self.clock = LiveClock(renderer.phi2, now=now)
        self.now = now
        self.auto = latency is None
        self.latency = self.AUTO_START if self.auto else latency
        self.delays = collections.deque()   # (wall, seconds a frame's oldest write had waited when it arrived)
        self.last_adjust = None
        self.target = int(buffer * renderer.rate)
        self.trim_start = trim_start        # False: keep the backlog before playing (recording, no sound card)
        self.fifo = collections.deque()
        self.fifo_len = 0
        self.lock = threading.Lock()
        self.underruns = 0
        self.dropped = 0
        self.repeated = 0
        self.started = False
        self.last = np.zeros(2 if renderer.stereo else 1, dtype=np.int16)

    def feed(self, frame):
        arrived = self.now()   # before the work below: HELLO's emulator reset runs 1.2 s of warm-up, which
        with self.lock:        # took ~0.36 s on a Mac, and C64 time 0 marked after it put "now" that far behind
            if isinstance(frame, sp.Hello):
                self.r.push(frame)
                self.clock = LiveClock(frame.phi2_hz, now=self.now)
                self.clock.seen(0, arrived)
                self.fifo.clear()
                self.fifo_len = 0
                self.started = False
                return
            self.r.push(frame)
            oldest = None
            if isinstance(frame, sp.Writes) and frame.blocks:
                self.clock.seen(frame.blocks[-1].time)
                oldest = frame.blocks[0].time
            elif isinstance(frame, sp.Status):
                self.clock.seen(frame.time)
                oldest = frame.time
            if oldest is not None:
                wall = self.now()
                now = self.clock.c64_now(wall)
                self.delays.append((wall, max(0.0, (now - oldest) / self.r.phi2)))
                while self.delays and wall - self.delays[0][0] > self.AUTO_WINDOW:
                    self.delays.popleft()

    def _adjust_latency(self):
        wall = self.now()
        if self.last_adjust is None:
            self.last_adjust = wall
            return
        dt = wall - self.last_adjust
        if dt < 0.1 or len(self.delays) < 20:
            return
        self.last_adjust = wall
        d = sorted(x for _, x in self.delays)
        p99 = d[min(len(d) - 1, int(0.99 * len(d)))]
        target = min(self.AUTO_MAX, max(self.AUTO_MIN, p99 + self.AUTO_MARGIN))
        if target > self.latency:
            self.latency = min(target, self.latency + self.AUTO_UP * dt)
        else:
            self.latency = max(target, self.latency - self.AUTO_DOWN * dt)

    def pump(self):
        """Render what is due.  Call often (every few ms) from a non-audio thread."""
        with self.lock:
            if self.auto:
                self._adjust_latency()
            now = self.clock.c64_now()
            if now is None:
                return
            target = now - int(self.latency * self.r.phi2)
            if target <= self.r.cursor:
                return
            behind = (target - self.r.cursor) / self.r.phi2
            keep = int(self.target / self.r.rate * self.r.phi2)   # the FIFO's target, in C64 cycles
            if self.trim_start and not self.started and target - self.r.cursor > keep:
                # before playing, render no more than the FIFO holds: what is older (the time HELLO's
                # emulator reset took, ~0.36 s on a Mac) would only sit in the FIFO as extra delay
                self.r.skip_to(target - keep)
            elif self.trim_start and self.started and target - self.r.cursor > self.JUMP_FIFOS * keep:
                # "now" jumped ahead while playing (a stall, or HELLO marked late): skip to just the FIFO's
                # target instead of queueing the jump, which the drift correction would take many seconds to drop
                held = int(self.fifo_len / self.r.rate * self.r.phi2)
                self.r.skip_to(target - max(0, keep - held))
            elif behind > 1.0:                # a stall: jump, keep the state right
                self.r.skip_to(target - int(0.2 * self.r.phi2))
            s = self.r.render_to(target)
            if len(s):
                self.fifo.append(s)
                self.fifo_len += len(s)
                if self.fifo_len >= self.target:
                    self.started = True

    def fill(self, n):
        """n samples for the audio callback, shape (n, channels) int16."""
        ch = 2 if self.r.stereo else 1
        out = np.empty((n, ch), dtype=np.int16)
        with self.lock:
            if not self.started:
                out[:] = 0
                return out
            # drift correction: up to MAX_STEP samples per call, more the further the FIFO is from its
            # target, spread evenly through the block.  It must outpace the automatic latency's slew
            # (AUTO_DOWN): at one sample per 256 it removed only 3.9 ms/s against 10, and the excess sat in
            # the FIFO, heard as extra delay the printed latency did not show.
            err = self.fifo_len - self.target
            k = 0
            if abs(err) > self.target // 4:
                k = min(self.MAX_STEP, abs(err) * 8 // max(self.target, 1))   # 2 at a quarter of the target off
            if err > 0 and k:
                got = self._take(n + k)
                if len(got) == n + k:
                    got = np.delete(got, (np.arange(1, k + 1) * (n + k) // (k + 1)), axis=0)
                    self.dropped += k
            elif err < 0 and k:
                got = self._take(n - k)
                if len(got) == n - k:
                    reps = np.ones(n - k, dtype=int)
                    reps[np.arange(1, k + 1) * (n - k) // (k + 1)] += 1
                    got = np.repeat(got, reps, axis=0)
                    self.repeated += k
            else:
                got = self._take(n)
            if len(got) < n:
                self.underruns += 1
                pad = np.repeat(got[-1:] if len(got) else self.last[None, :], n - len(got), axis=0)
                got = np.concatenate([got, pad]) if len(got) else pad
            out[:] = got[:n]
            self.last = out[-1].copy()
        return out

    def drain(self):
        """Everything rendered so far, no drift correction (recording without a sound card)."""
        with self.lock:
            got = self._take(self.fifo_len)
        return got

    def _take(self, n):
        parts = []
        need = n
        while need and self.fifo:
            a = self.fifo[0]
            if len(a) <= need:
                parts.append(a)
                need -= len(a)
                self.fifo.popleft()
            else:
                parts.append(a[:need])
                self.fifo[0] = a[need:]
                need = 0
        got = n - need
        self.fifo_len -= got
        if not parts:
            return np.zeros((0, 2 if self.r.stereo else 1), dtype=np.int16)
        a = np.concatenate(parts)
        return a.reshape(len(a), -1)
