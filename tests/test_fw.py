"""Decode the channel bytes the firmware glue wrote in tests/fw_host (make test-fw runs both)."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
import snoopproto as sp  # noqa: E402


def frames(name):
    with open(os.path.join(ROOT, "build", name), "rb") as fh:
        data = fh.read()
    dec = sp.Decoder()
    return dec, list(dec.feed(data)), data


class Glue(unittest.TestCase):
    def test_serial_run(self):
        dec, fr, data = frames("fw_serial.bin")
        self.assertTrue(data.startswith(sp.ACK))
        h = fr[0]
        self.assertIsInstance(h, sp.Hello)
        self.assertEqual((h.phi2_hz, h.cpu_hz, h.max_gap, h.ranges), (sp.PHI2_PAL, 600_000_000, 32, [(0xD400, 0xD41C)]))
        w = sp.writes_of(fr)
        self.assertEqual([(a, v) for _, a, v in w[:25]], [(0xD400 + k, 0x40 + k) for k in range(25)])
        blocks = [b for f in fr if isinstance(f, sp.Writes) for b in f.blocks]
        self.assertEqual((blocks[0].addr, len(blocks[0].data)), (0xD400, 25))   # 3 cycles apart: one block
        self.assertEqual([v for _, a, v in w[25:]], list(range(40)))            # held back while room was 0, not lost
        self.assertEqual([t2 - t1 for (t1, _, _), (t2, _, _) in zip(w[25:], w[26:])], [1] * 39)
        self.assertEqual(dec.lost_frames, 0)
        self.assertIn(dec.bad_frames, (1, 2))   # the two ack tokens in the stream (one chunk when no frame went out between them), skipped
        self.assertEqual(sp.ACK in data[2:], True)                              # the status command's ack
        sts = [f for f in fr if isinstance(f, sp.Status)]
        self.assertGreaterEqual(len(sts), 2)
        probed = [st for st in sts if st.probe_cyc]
        self.assertTrue(probed)                                                 # the CPU probe ran during the run
        for st in probed:   # 6-cycle reads, a 300-cycle interrupt every 50: 49 * 6 free of 600
            self.assertAlmostEqual(st.cpu_free, 294 / 600, delta=0.01)
            self.assertEqual(st.probe_stall, 306)
            self.assertIn(st.probe_worst, range(480, 500))
        self.assertIsInstance(fr[-1], sp.Bye)
        self.assertEqual(fr[-1].reason, 1)                                      # replaced by the TCP start

    def test_tcp_run(self):
        dec, fr, data = frames("fw_tcp.bin")
        self.assertTrue(data.startswith(sp.ACK))
        self.assertEqual(fr[0].ranges, [(0xD418, 0xD418), (0xDE00, 0xDE00)])
        self.assertEqual(fr[0].max_gap, 0)
        w = sp.writes_of(fr)
        self.assertEqual([(a, v) for _, a, v in w],
                         [x for k in range(5) for x in ((0xD418, 0xF0 + k), (0xDE00, k))] + [(0xD418, 0xAA)])
        self.assertEqual([t2 - t1 for (t1, _, _), (t2, _, _) in zip(w[:10], w[1:10])], [1] * 9)
        self.assertNotIsInstance(fr[-1], sp.Bye)    # the channel went away: nothing more sent

    def test_ntsc_and_stop(self):
        dec, fr, data = frames("fw_ntsc.bin")
        self.assertEqual(fr[0].phi2_hz, sp.PHI2_NTSC)
        self.assertEqual([(a, v) for _, a, v in sp.writes_of(fr)], [(0xD418, 0x0F)])
        st = [f for f in fr if isinstance(f, sp.Status)][-1]
        self.assertEqual((st.late, st.late250, st.late1000), (1, 1, 1))
        self.assertIsInstance(fr[-1], sp.Bye)
        self.assertEqual(fr[-1].reason, 0)


class DataPort(unittest.TestCase):
    def test_stream_on_6581(self):
        dec, fr, data = frames("fw_dataport.bin")
        self.assertTrue(data.startswith(sp.ACK))
        self.assertEqual(fr[0].ranges, [(0xD418, 0xD418)])
        self.assertEqual([v for _, a, v in sp.writes_of(fr)], [0x30 + k for k in range(6)] + [0x40 + k for k in range(4)])
        self.assertIn(b"only the snoop command", data)
        self.assertGreaterEqual(len([f for f in fr if isinstance(f, sp.Status)]), 1)
        self.assertEqual(dec.lost_frames, 0)


class Tone(unittest.TestCase):
    """The 1 kHz test tone as the dongle gets it (tests/fw_host section 10: 1000 frames of 192 bytes)."""

    @staticmethod
    def level(x, hz, rate=48000):
        """amplitude of the hz component (Goertzel), in the samples' units"""
        import math
        w = 2 * math.pi * hz / rate
        c = 2 * math.cos(w)
        s1 = s2 = 0.0
        for v in x:
            s1, s2 = v + c * s1 - s2, s1
        return 2 * math.sqrt(max(s1 * s1 + s2 * s2 - c * s1 * s2, 0.0)) / len(x)

    def test_spectrum(self):
        import array
        import math
        with open(os.path.join(ROOT, "build", "fw_tone.raw"), "rb") as fh:
            data = fh.read()
        self.assertEqual(len(data), 192 * 1000)
        a = array.array("h")
        a.frombytes(data)
        if sys.byteorder != "little":
            a.byteswap()
        left = a[0::2]
        self.assertEqual(list(left), list(a[1::2]))
        fund = self.level(left, 1000)
        self.assertAlmostEqual(fund, 8192, delta=8)                     # -12 dBFS
        for hz in (500, 2000, 3000, 5000, 7000, 11000):
            self.assertLess(20 * math.log10(self.level(left, hz) / fund + 1e-12), -80, f"{hz} Hz")


@unittest.skipUnless(os.path.exists(os.path.join(ROOT, "build", "fw_sid.bin")), "no build/fw_sid.bin (make test-sid)")
class SidEmulator(unittest.TestCase):
    def test_status_reports_the_emulator_cost(self):
        dec, fr, data = frames("fw_sid.bin")
        sts = [f for f in fr if isinstance(f, sp.Status)]
        self.assertTrue(sts)
        emulated = sum(st.emu_c64 for st in sts)
        self.assertGreater(emulated, sp.PHI2_PAL // 10)                  # the run, ~0.15 s of C64 time
        self.assertLessEqual(emulated, sts[-1].time)                     # never ahead of the capture
        self.assertGreater(sum(st.emu_cyc for st in sts), 0)
        busy = [st for st in sts if st.emu_c64]
        self.assertTrue(busy[0].emu_text(600_000_000, sp.PHI2_PAL).startswith("reSID"))

    def test_status_reports_the_dongle_buffer(self):
        if not os.path.exists(os.path.join(ROOT, "build", "fw_sid_dongle.bin")):
            self.skipTest("make resid test-sid")
        dec, fr, data = frames("fw_sid_dongle.bin")
        h = [f for f in fr if isinstance(f, sp.Hello)]
        self.assertEqual(h[0].version, 8)
        sts = [f for f in fr if isinstance(f, sp.Status)]
        self.assertGreaterEqual(len(sts), 2)                               # the status command's, then the stop's
        self.assertEqual((sts[0].audio_low, sts[0].audio_under), (0, 1))    # the short packet, during the capture
        self.assertGreater(sts[0].audio_trim, 0)
        self.assertTrue(sts[0].audio_text().startswith("dongle lowest  0.0 ms, underruns 1, trimmed "))
        self.assertEqual((sts[-1].audio_low, sts[-1].audio_under, sts[-1].audio_trim), (sp.AUDIO_NONE, 0, 0))


if __name__ == "__main__":
    unittest.main()
