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
        self.assertEqual(dec.bad_frames, 2)     # the two ack tokens in the stream, skipped
        self.assertEqual(sp.ACK in data[2:], True)                              # the status command's ack
        self.assertGreaterEqual(len([f for f in fr if isinstance(f, sp.Status)]), 2)
        self.assertIsInstance(fr[-1], sp.Bye)
        self.assertEqual(fr[-1].reason, 1)                                      # replaced by the TCP start

    def test_tcp_run(self):
        dec, fr, data = frames("fw_tcp.bin")
        self.assertTrue(data.startswith(sp.ACK))
        self.assertEqual(fr[0].ranges, [(0xD418, 0xD418), (0xDE00, 0xDE00)])
        self.assertEqual(fr[0].max_gap, 0)
        w = sp.writes_of(fr)
        self.assertEqual([(a, v) for _, a, v in w],
                         [x for k in range(5) for x in ((0xD418, 0xF0 + k), (0xDE00, k))])
        self.assertEqual([t2 - t1 for (t1, _, _), (t2, _, _) in zip(w, w[1:])], [1] * 9)
        self.assertNotIsInstance(fr[-1], sp.Bye)    # the channel went away: nothing more sent

    def test_ntsc_and_stop(self):
        dec, fr, data = frames("fw_ntsc.bin")
        self.assertEqual(fr[0].phi2_hz, sp.PHI2_NTSC)
        self.assertEqual([(a, v) for _, a, v in sp.writes_of(fr)], [(0xD418, 0x0F)])
        self.assertIsInstance(fr[-1], sp.Bye)
        self.assertEqual(fr[-1].reason, 0)


if __name__ == "__main__":
    unittest.main()
