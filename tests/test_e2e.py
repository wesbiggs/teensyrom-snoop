"""End to end with real tunes: a PSID's writes (tools/sidtrace.py, py65) through the capture core,
the wire, the decoder and reSIDfp, against rendering the trace directly.

The tunes are the ones the TeensyROM repo ships (Source/Teensy/TRMenuFiles/SIDs/*.sid.h); set
TEENSYROM to a checkout (default ../3p/TeensyROM).  Skipped without it, py65, pyresidfp or numpy.
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "host"), os.path.join(ROOT, "tools")]
import snoopproto as sp  # noqa: E402

TR = os.environ.get("TEENSYROM", os.path.join(ROOT, "..", "3p", "TeensyROM"))
SIDS = os.path.join(TR, "Source", "Teensy", "TRMenuFiles", "SIDs")
TUNES = ["Aces_High", "Echoes", "Switch_625"]

try:
    import numpy as np
    import py65  # noqa: F401
    import sidplay
    import sidtrace
    HAVE = sidplay.SID is not None and os.path.isdir(SIDS)
except ImportError:
    HAVE = False


def snr_db(ref, x):
    n = min(len(ref), len(x))
    ref, x = ref[:n].astype(float), x[:n].astype(float)
    return 10 * np.log10(np.sum(ref ** 2) / max(np.sum((x - ref) ** 2), 1e-9))


@unittest.skipUnless(HAVE, "needs py65, pyresidfp, numpy and the TeensyROM SIDs (TEENSYROM=...)")
class RealTunes(unittest.TestCase):
    def test_tunes(self):
        for t in TUNES:
            with self.subTest(tune=t):
                sid = sidtrace.Psid(sidtrace.load_sid_bytes(os.path.join(SIDS, t + ".sid.h")))
                w, phi2, _ = sidtrace.trace(sid, 6)
                end = w[-1][0] + 1000
                ref = sidplay.SidRenderer(bases=[0xD400])
                for c, a, v in w:
                    ref.write(c, a, v)
                ref = ref.render_to(end)

                wire = sidtrace.capture(w, phi2, 0)
                dec = sp.Decoder()
                fr = list(dec.feed(wire))
                self.assertEqual((dec.bad_frames, dec.lost_frames), (0, 0))
                self.assertEqual(sp.writes_of(fr), w)                    # every write, exact cycle
                self.assertLess(len(wire) / 6, 10_000)                   # bytes per second
                st = [f for f in fr if isinstance(f, sp.Status)][-1]
                self.assertEqual((st.dropped, st.out_dropped, st.writes), (0, 0, len(w)))

                r = sidplay.SidRenderer(bases=[0xD400])
                for f in fr:
                    r.push(f)
                # reSIDfp dithers its output (two renders of the same input differ by a few LSB)
                self.assertGreater(snr_db(ref, r.render_to(end)), 55)


if __name__ == "__main__":
    unittest.main()
