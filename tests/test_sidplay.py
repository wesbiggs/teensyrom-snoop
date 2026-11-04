"""host/sidplay.py: the reSIDfp renderer and the live clock (skipped without pyresidfp + numpy)."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
import snoopproto as sp  # noqa: E402

try:
    import numpy as np
    import sidplay
    HAVE = sidplay.SID is not None
except ImportError:
    HAVE = False

PAL = sp.PHI2_PAL
RATE = 48000


def freq_reg(hz, phi2=PAL):
    return round(hz * (1 << 24) / phi2)


def tone_writes(t, base, hz, wave=0x11):
    """(time, addr, bytes) blocks that start a full-volume tone on voice 1."""
    f = freq_reg(hz)
    return [(t, base, bytes([f & 0xFF, f >> 8])),
            (t + 10, base + 5, bytes([0x00, 0xF0])),     # AD, SR
            (t + 20, base + 0x18, b"\x0f"),               # volume 15
            (t + 30, base + 4, bytes([wave]))]            # waveform + gate


def stream(blocks, end, ranges=((0xD400, 0xD41C),)):
    w = sp.frame(sp.hello_payload(list(ranges), max_gap=0))
    seq = 0
    for i in range(0, len(blocks), 20):
        w += sp.frame(sp.writes_payload(seq, blocks[i:i + 20]))
        seq += 1
    w += sp.frame(sp.status_payload(seq, end)) + sp.frame(sp.bye_payload())
    return w


def peak_hz(x, rate=RATE):
    x = x.astype(float) - x.mean()
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    return np.argmax(spec) * rate / len(x)


@unittest.skipUnless(HAVE, "pyresidfp and numpy not installed")
class Render(unittest.TestCase):
    def test_tone_and_its_timeline(self):
        b = tone_writes(1000, 0xD400, 440)
        f2 = freq_reg(880)
        b.append((PAL // 2, 0xD400, bytes([f2 & 0xFF, f2 >> 8])))   # an octave up at 0.5 s
        r, s = sidplay.render_stream(stream(b, PAL), bases=[0xD400], sample_rate=RATE)
        self.assertAlmostEqual(len(s) / RATE, 1.0, delta=0.002)
        self.assertAlmostEqual(peak_hz(s[4800:23000]), 440, delta=3)
        self.assertAlmostEqual(peak_hz(s[25000:47000]), 880, delta=3)
        self.assertEqual((r.late, r.ignored), (0, 0))

    def test_change_lands_on_its_cycle(self):
        """Moving the note's start by d cycles moves the audio by d * rate / phi2 samples."""
        def render(t):
            # noise: no period for the correlation to slip by; the test bit restarts the oscillator,
            # which otherwise runs free from power-on and would not move with the note
            b = tone_writes(t, 0xD400, 3000, wave=0x81)
            b.insert(3, (t + 25, 0xD404, b"\x08"))
            return sidplay.render_stream(stream(b, PAL // 2), bases=[0xD400], sample_rate=RATE)[1].astype(float)
        a = render(20000)
        for d in (1000, 12345, 40001):
            b = render(20000 + d)
            n = 8192
            x, y = a[500:500 + n], b[500:500 + n]
            lags = np.arange(0, 2500)
            corr = [np.dot(x[:n - 2500], y[l:l + n - 2500]) for l in lags]
            # sub-sample: parabola through the peak
            k = int(np.argmax(corr))
            c0, c1, c2 = corr[k - 1], corr[k], corr[k + 1]
            lag = k + 0.5 * (c0 - c2) / (c0 - 2 * c1 + c2)
            # within half a sample (the parabola fit on a noise correlation is no finer than that)
            self.assertAlmostEqual(lag, d * RATE / PAL, delta=0.5)

    def test_two_sids_stereo_and_ignored(self):
        b = tone_writes(1000, 0xD400, 440)
        b.append((5000, 0xD41B, b"\x00"))                 # read-only register: not the SID's
        b.append((5100, 0xD600, b"\x01"))                 # outside both
        r, s = sidplay.render_stream(stream(b, PAL // 2, ((0xD400, 0xD41C), (0xD500, 0xD51C), (0xD600, 0xD600))),
                                     bases=[0xD400, 0xD500], sample_rate=RATE, stereo=True)
        self.assertEqual(s.shape[1], 2)
        self.assertGreater(np.std(s[4800:, 0]), 1000)
        self.assertLess(np.std(s[:, 1]), 50)                     # warmed up: no start-up DC drift either
        self.assertEqual(r.ignored, 2)

    def test_8580_and_ntsc(self):
        b = tone_writes(1000, 0xD400, 440)
        f = freq_reg(440, sp.PHI2_NTSC)
        b[0] = (1000, 0xD400, bytes([f & 0xFF, f >> 8]))
        w = sp.frame(sp.hello_payload([(0xD400, 0xD41C)], phi2_hz=sp.PHI2_NTSC))
        w += sp.frame(sp.writes_payload(0, b)) + sp.frame(sp.status_payload(1, sp.PHI2_NTSC))
        r, s = sidplay.render_stream(w, bases=[0xD400], sample_rate=RATE, model="8580")
        self.assertEqual(r.phi2, sp.PHI2_NTSC)
        self.assertAlmostEqual(len(s) / RATE, 1.0, delta=0.002)
        self.assertAlmostEqual(peak_hz(s[4800:]), 440, delta=3)


class FakeTime:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Clock(unittest.TestCase):
    @unittest.skipUnless(HAVE, "numpy")
    def test_estimate_tracks_with_jittery_arrivals(self):
        ft = FakeTime()
        c = sidplay.LiveClock(PAL, now=ft)
        rng = np.random.default_rng(1)
        start = ft.t
        errs = []
        for k in range(2000):                     # a frame every 5 ms, 1-20 ms transport delay
            ft.t = start + k * 0.005
            sent = ft.t - rng.uniform(0.001, 0.020)
            c.seen(int((sent - start) * PAL))
            if k > 400:
                errs.append(c.c64_now() / PAL - (ft.t - start))
        self.assertLess(max(abs(e) for e in errs), 0.0025)


@unittest.skipUnless(HAVE, "pyresidfp and numpy not installed")
class Live(unittest.TestCase):
    def test_live_player_no_underruns_with_drift(self):
        """Frames arrive late and jittery; the sound card runs 0.5 % fast (far worse than real)."""
        ft = FakeTime()
        r = sidplay.SidRenderer(bases=[0xD400], sample_rate=RATE)
        p = sidplay.LivePlayer(r, latency=0.03, buffer=0.05, now=ft)
        rng = np.random.default_rng(2)
        start = ft.t
        p.feed(sp.Hello(sp.PROTO_VERSION, PAL, 600_000_000, 0, [(0xD400, 0xD41C)]))
        blocks = [sp.Block(t, a, d) for t, a, d in tone_writes(2000, 0xD400, 440)]
        pending = [(start + 0.01, sp.Writes(0, blocks))]
        # a STATUS-like time mark every 10 ms (as frames of a busy tune would give)
        for k in range(1, 600):
            pending.append((start + k * 0.01 + rng.uniform(0.001, 0.015), sp.Status(1, int(k * 0.01 * PAL), 0, 0, 0, 0)))
        pending.sort(key=lambda x: x[0])
        out = []
        dev_rate = RATE * 1.005
        block = 256
        t_next_cb = start + 0.1
        while ft.t < start + 6.0:
            ft.t += 0.001
            while pending and pending[0][0] <= ft.t:
                p.feed(pending.pop(0)[1])
            p.pump()
            while t_next_cb <= ft.t:
                out.append(p.fill(block))
                t_next_cb += block / dev_rate
        s = np.concatenate(out)[:, 0]
        started = next(i for i, v in enumerate(s) if v != 0)
        tail = s[started + RATE:]
        self.assertEqual(p.underruns, 0)
        self.assertLess(p.fifo_len, 3 * p.target)
        self.assertAlmostEqual(peak_hz(tail), 440, delta=3)    # pitch kept: samples repeated, not stretched
        self.assertGreater(p.dropped + p.repeated, 0)       # it did correct the drift
        self.assertLess(p.dropped + p.repeated, 0.006 * 6 * RATE)


@unittest.skipUnless(HAVE, "pyresidfp and numpy not installed")
class AutoLatency(unittest.TestCase):
    def run_link(self, jitter, seconds=12.0, spikes=0.0):
        """Frames every 2 ms of C64 time arrive after 1 ms + jitter (and a rare spike); the sound card
        takes 256 samples at a time. Returns the player and the latency history."""
        ft = FakeTime()
        r = sidplay.SidRenderer(bases=[0xD400], sample_rate=RATE)
        p = sidplay.LivePlayer(r, now=ft)          # automatic latency
        rng = np.random.default_rng(3)
        start = ft.t
        p.feed(sp.Hello(sp.PROTO_VERSION, PAL, 600_000_000, 0, [(0xD400, 0xD41C)]))
        pending = []
        seq = 0
        notes = tone_writes(2000, 0xD400, 440)
        for k in range(1, int(seconds / 0.002)):
            t = int(k * 0.002 * PAL)
            blocks = [sp.Block(t, 0xD401, bytes([k & 0x3F]))]       # a write in every frame
            if k == 1:
                blocks = [sp.Block(bt, a, d) for bt, a, d in notes] + blocks
            delay = 0.001 + rng.uniform(0, jitter)
            if spikes and rng.random() < spikes:
                delay += 0.05
            pending.append((start + k * 0.002 + delay, sp.Writes(seq, blocks)))
            seq += 1
        pending.sort(key=lambda x: x[0])
        history = []
        t_next_cb = start + 0.05
        while ft.t < start + seconds:
            ft.t += 0.001
            while pending and pending[0][0] <= ft.t:
                p.feed(pending.pop(0)[1])
            p.pump()
            while t_next_cb <= ft.t:
                p.fill(256)
                t_next_cb += 256 / RATE
            history.append(p.latency)
        return p, history

    def test_settles_just_above_the_jitter(self):
        p, hist = self.run_link(jitter=0.020)
        self.assertGreater(p.latency, 0.025)                  # 1 ms + up to 20 ms, + 10 ms margin
        self.assertLess(p.latency, 0.040)
        self.assertEqual(p.underruns, 0)
        self.assertLess(p.r.late, 0.01 * 6000)                 # under 1 % of the ~6000 writes

    def test_fast_link_gets_low_latency(self):
        p, hist = self.run_link(jitter=0.002)
        self.assertLess(p.latency, 0.020)
        self.assertEqual(p.underruns, 0)

    def test_rare_spikes_are_ridden_out(self):
        p, hist = self.run_link(jitter=0.005, spikes=0.002)    # 0.2 % of frames 50 ms late
        self.assertLess(p.latency, 0.045)
        self.assertEqual(p.underruns, 0)


if __name__ == "__main__":
    unittest.main()
