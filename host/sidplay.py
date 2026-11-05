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
                 phi2_hz=sp.PHI2_PAL, filter=True, sampling="resample"):
        if SID is None:
            raise RuntimeError("needs pyresidfp: pip install pyresidfp numpy")
        self.bases = list(bases)
        self.rate = sample_rate
        self.model = model
        self.stereo = stereo
        self.filter = filter
        if sampling not in ("resample", "fast"):
            raise ValueError(f"sampling {sampling!r}: resample or fast")
        self.sampling = sampling              # fast: reSIDfp's DECIMATE (no sinc resampler, a little aliasing)
        self.queue = collections.deque()      # (time, sid index, register, value), ascending time
        self.late = 0                         # writes that arrived after their time was rendered
        self.ignored = 0                      # writes outside every SID's registers
        self._make(phi2_hz)

    def _make(self, phi2_hz):
        self.phi2 = phi2_hz
        cm = ChipModel.MOS8580 if str(self.model) == "8580" else ChipModel.MOS6581
        self.sids = []
        for _ in self.bases:
            method = SamplingMethod.DECIMATE if self.sampling == "fast" else SamplingMethod.RESAMPLE
            s = SID(cm, method, float(phi2_hz), float(self.rate))
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
        """out: one list of samples per SID (plain lists: a numpy array per call costs more
        than the few samples between two writes)."""
        if cycles <= 0:
            return
        chans = [s.clock(int(cycles)) for s in self.sids]
        if len(chans) == 1:
            out[0].extend(chans[0])
        else:
            n = min(len(c) for c in chans)
            for o, c in zip(out, chans):
                o.extend(c[:n] if len(c) > n else c)
        self.cursor += cycles

    def render_to(self, t_target):
        """Clock every SID up to C64 time t_target, applying the queued writes on their cycle.
        Returns int16 samples: shape (n,) mono or (n, 2) stereo."""
        out = [[] for _ in self.sids]
        while self.queue and self.queue[0][0] <= t_target:
            t, i, reg, v = self.queue.popleft()
            self._clock(t - self.cursor, out)
            self.sids[i].write(reg, v)
        self._clock(t_target - self.cursor, out)
        return self._mix(out)

    def skip_to(self, t_target):
        """Like render_to, but the samples are thrown away (catching up after a stall)."""
        self.render_to(t_target)

    def _mix(self, out):
        chans = [np.array(o, dtype=np.int32) for o in out]
        if not len(chans[0]):
            return np.zeros((0, 2) if self.stereo else 0, dtype=np.int16)
        if len(chans) == 1 and not self.stereo:
            return chans[0].astype(np.int16)      # reSIDfp's samples are already in int16 range
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


def bench_stream(seconds=10.0, digi_hz=0, base=0xD400, phi2=sp.PHI2_PAL):
    """A synthetic capture for timing the player without a C64: a tune-like 50 Hz player
    (three voices, ~25 register writes a frame), plus, with digi_hz, a $D418 volume write
    digi_hz times a second (4-bit sample playback: the heaviest common case)."""
    blocks = []
    frame_cycles = phi2 // 50
    for n in range(int(seconds * 50)):
        t = 1000 + n * frame_cycles
        for v in range(3):
            f = 2000 + 300 * v + 37 * (n % 32)
            r = base + 7 * v
            blocks.append((t + 40 * v, r, bytes([f & 0xFF, f >> 8, 0x00, 0x08, 0x41 if n % 8 else 0x40,
                                                 0x09, 0xA9])))
        blocks.append((t + 200, base + 0x15, bytes([0x00, 0x20 + n % 64, 0xF7, 0x1F])))
    if digi_hz:
        step = phi2 / digi_hz
        k = 0
        while k * step < seconds * phi2:
            blocks.append((1500 + int(k * step), base + 0x18, bytes([k % 16])))
            k += 1
    blocks.sort(key=lambda b: b[0])
    end = 1000 + int(seconds * phi2)
    w = sp.frame(sp.hello_payload([(base, base + 0x1C)], max_gap=0))
    seq = 0
    for i in range(0, len(blocks), 20):
        w += sp.frame(sp.writes_payload(seq, blocks[i:i + 20]))
        seq += 1
    w += sp.frame(sp.status_payload(seq, end)) + sp.frame(sp.bye_payload())
    return w


def bench(data: bytes, step_ms=4.0, **kw):
    """Render a stream the way live play does (decode, then render_to every step_ms of C64 time)
    as fast as possible.  Returns (C64 seconds, CPU seconds, emulation-only CPU seconds, writes)."""
    r = SidRenderer(**kw)       # not timed: the first one builds reSIDfp's filter tables (~0.3 s, once)
    t0 = time.process_time()
    frames = list(sp.Decoder().feed(data))
    t_dec = time.process_time() - t0
    end = 0
    writes = 0
    for f in frames:
        if isinstance(f, sp.Writes) and f.blocks:
            end = max(end, f.blocks[-1].time)
            writes += sum(len(b.data) for b in f.blocks)
        elif isinstance(f, sp.Status):
            end = max(end, f.time)
    step = int(r.phi2 * step_ms / 1000)
    t0 = time.process_time()
    i = 0
    t = step
    while t < end + step:      # feed what has "arrived" by C64 time t, render up to it
        while i < len(frames) and not (isinstance(frames[i], sp.Writes) and frames[i].blocks
                                       and frames[i].blocks[0].time > t):
            if not isinstance(frames[i], sp.Hello):
                r.push(frames[i])
            i += 1
        r.render_to(min(t, end))
        t += step
    t_play = time.process_time() - t0 + t_dec
    # the same span with no writes, in one call per step: what reSIDfp itself costs
    e = SidRenderer(**kw)
    t0 = time.process_time()
    c = step
    while c < end + step:
        e.render_to(min(c, end))
        c += step
    t_emu = time.process_time() - t0
    return end / r.phi2, t_play, t_emu, writes


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
    AUTO_UP, AUTO_DOWN = 0.05, 0.010   # how fast the latency may move, seconds per second

    def __init__(self, renderer: SidRenderer, latency=None, buffer=0.03, now=time.monotonic):
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
        self.fifo = collections.deque()
        self.fifo_len = 0
        self.lock = threading.Lock()
        self.underruns = 0
        self.dropped = 0
        self.repeated = 0
        self.started = False
        self.last = np.zeros(2 if renderer.stereo else 1, dtype=np.int16)

    def feed(self, frame):
        with self.lock:
            if isinstance(frame, sp.Hello):
                self.r.push(frame)
                self.clock = LiveClock(frame.phi2_hz, now=self.now)
                self.clock.seen(0)
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
            if behind > 1.0:                  # a stall (or the start): jump, keep the state right
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
            # drift correction: one sample per call at most
            want = n
            if self.fifo_len > self.target * 5 // 4:
                want = n + 1
                self.dropped += 1
            elif self.fifo_len < self.target * 3 // 4:
                want = n - 1
                self.repeated += 1
            got = self._take(want)
            if len(got) < want:
                self.underruns += 1
            if want == n + 1 and len(got) == want:
                got = got[1:]
            if len(got) < n:
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
