"""core/snoop.c through tests/shim.c (ctypes), decoded with host/snoopproto.py."""
import ctypes
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
import snoopproto as sp  # noqa: E402

LIB = ctypes.CDLL(os.path.join(ROOT, "build", "libsnoop.so"))
U32 = ctypes.c_uint32
LIB.sim_poll.argtypes = [U32, U32]
LIB.sim_poll.restype = U32
LIB.sim_stop.argtypes = [ctypes.c_int, U32, U32]
LIB.sim_bus.argtypes = [ctypes.c_uint16, ctypes.c_uint8, U32]
LIB.sim_take.argtypes = [ctypes.c_char_p, U32]
LIB.sim_take.restype = U32
LIB.sim_avail.restype = U32
LIB.sim_dropped.restype = U32
LIB.sim_crc16.argtypes = [ctypes.c_char_p, U32]
LIB.sim_crc16.restype = ctypes.c_uint16
LIB.sim_cobs.argtypes = [ctypes.c_char_p, U32, ctypes.c_char_p]
LIB.sim_cobs.restype = U32
LIB.sim_start.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_int, ctypes.c_int, U32, U32, U32, U32]

CPU = 600_000_000
PAL = sp.PHI2_PAL
RING = LIB.sim_ring_size()
OUT = LIB.sim_out_size()


def stamp_of(cycle, jitter=0):
    """CPU stamp of C64 cycle n (n = 0 at the start stamp 1000), plus some ISR entry jitter."""
    return (1000 + round(cycle * CPU / PAL) + jitter) & 0xFFFFFFFF


class Sim:
    def __init__(self, ranges, gap=32, flush_us=2000, status_ms=0):
        LIB.sim_init()
        LIB.sim_set(flush_us, status_ms)
        self.us = 0
        self.dec = sp.Decoder()
        self.frames = []
        flat = (ctypes.c_uint16 * (2 * len(ranges)))(*[v for r in ranges for v in r])
        self.rc = LIB.sim_start(flat, len(ranges), gap, PAL, CPU, 0, stamp_of(0))

    def bus(self, addr, data, cycle, jitter=0):
        return LIB.sim_bus(addr, data, stamp_of(cycle, jitter))

    def poll(self, cycle, us_step=100):
        self.us += us_step
        LIB.sim_poll(self.us, stamp_of(cycle))

    def drain(self):
        buf = ctypes.create_string_buffer(OUT)
        out = b""
        while True:
            n = LIB.sim_take(buf, OUT)
            if not n:
                break
            out += buf.raw[:n]
        self.frames += list(self.dec.feed(out))
        return out

    def stop(self, cycle, reason=0):
        self.us += 100
        LIB.sim_stop(reason, self.us, stamp_of(cycle))
        self.drain()

    def writes(self):
        return sp.writes_of(self.frames)

    def of(self, kind):
        return [f for f in self.frames if isinstance(f, kind)]


class Helpers(unittest.TestCase):
    def test_crc_and_cobs_match_python(self):
        rnd = random.Random(1)
        cases = [b"", b"\x00", b"\x00\x00", bytes(range(1, 255)), bytes(range(1, 256)) * 2,
                 bytes(600), b"123456789"]
        cases += [bytes(rnd.choice([0, 1, 0xFF, rnd.randrange(256)]) for _ in range(rnd.randrange(700)))
                  for _ in range(200)]
        self.assertEqual(sp.crc16(b"123456789"), 0x29B1)
        for c in cases:
            self.assertEqual(LIB.sim_crc16(c, len(c)), sp.crc16(c))
            out = ctypes.create_string_buffer(len(c) + len(c) // 254 + 2)
            n = LIB.sim_cobs(c, len(c), out)
            enc = out.raw[:n]
            self.assertNotIn(0, enc)
            self.assertEqual(enc, sp.cobs_encode(c))
            self.assertEqual(sp.cobs_decode(enc), c)

    def test_parse_range(self):
        self.assertEqual(sp.parse_range("D400-D41C"), (0xD400, 0xD41C))
        self.assertEqual(sp.parse_range("$d418"), (0xD418, 0xD418))
        self.assertEqual(sp.parse_range("D400+29"), (0xD400, 0xD41C))
        with self.assertRaises(ValueError):
            sp.parse_range("D41C-D400")

    def test_start_command(self):
        self.assertEqual(sp.start_command([(0xD400, 0xD41C)], 32),
                         bytes([0x64, 0x53, 1, 1, 0x00, 0xD4, 0x1C, 0xD4, 32, 0]))
        self.assertEqual(sp.start_command([(0xD418, 0xD418)], 0, 50)[-2:], bytes([0, 50]))


class Filter(unittest.TestCase):
    def test_only_watched_addresses(self):
        s = Sim([(0xD400, 0xD41C), (0xDE00, 0xDE00)], gap=0)
        hits = 0
        for a in [0xD3FF, 0xD400, 0xD41C, 0xD41D, 0xD500, 0xDE00, 0xDE01, 0x0001, 0xFFFF]:
            hits += s.bus(a, a & 0xFF, 10)
        self.assertEqual(hits, 3)
        s.poll(20)
        s.stop(30)
        self.assertEqual([(a, v) for _, a, v in s.writes()],
                         [(0xD400, 0x00), (0xD41C, 0x1C), (0xDE00, 0x00)])

    def test_hello_and_bye(self):
        s = Sim([(0xD400, 0xD41C), (0xD500, 0xD51C)], gap=7)
        s.stop(0)
        h = s.of(sp.Hello)[0]
        self.assertEqual((h.version, h.phi2_hz, h.cpu_hz, h.max_gap), (sp.PROTO_VERSION, PAL, CPU, 7))
        self.assertEqual(h.ranges, [(0xD400, 0xD41C), (0xD500, 0xD51C)])
        self.assertEqual(s.of(sp.Bye)[0].reason, 0)
        self.assertEqual(len(s.of(sp.Status)), 1)
        self.assertIsInstance(s.frames[0], sp.Hello)
        self.assertIsInstance(s.frames[-1], sp.Bye)

    def test_disarmed_after_stop(self):
        s = Sim([(0xD400, 0xD7FF)])
        s.stop(0)
        self.assertEqual(sum(LIB.sim_page(p) for p in range(256)), 0)
        self.assertEqual(s.bus(0xD400, 1, 5), 0)
        self.assertEqual(LIB.sim_running(), 0)

    def test_bad_starts(self):
        self.assertEqual(Sim([]).rc, -1)
        self.assertEqual(Sim([(0xD41C, 0xD400)]).rc, -2)
        self.assertEqual(Sim([(0x0000, 0x10FF)]).rc, -3)   # 17 pages
        self.assertEqual(Sim([(0x0000, 0x0FFF)]).rc, 0)    # 16 pages
        self.assertEqual(Sim([(0xD400, 0xD400)] * 17).rc, -1)

    def test_refused_start_keeps_the_run(self):
        s = Sim([(0xD400, 0xD41C)])
        flat = (ctypes.c_uint16 * 2)(0x0000, 0xFFFF)
        self.assertEqual(LIB.sim_start(flat, 1, 32, PAL, CPU, 0, stamp_of(0)), -3)
        self.assertEqual(LIB.sim_running(), 1)
        self.assertEqual(s.bus(0xD400, 1, 5), 1)

    def test_replace(self):
        s = Sim([(0xD400, 0xD41C)])
        s.bus(0xD400, 0x11, 5)
        flat = (ctypes.c_uint16 * 2)(0xD418, 0xD418)
        self.assertEqual(LIB.sim_start(flat, 1, 32, PAL, CPU, 50, stamp_of(10)), 0)
        s.drain()
        kinds = [type(f).__name__ for f in s.frames]
        self.assertEqual(kinds, ["Hello", "Writes", "Status", "Bye", "Hello"])
        self.assertEqual(s.of(sp.Bye)[0].reason, 1)
        self.assertEqual(s.bus(0xD400, 1, 20), 0)
        self.assertEqual(s.bus(0xD418, 1, 20), 1)


class Grouping(unittest.TestCase):
    def test_sid_register_dump_is_one_block(self):
        s = Sim([(0xD400, 0xD41C)], gap=32)
        regs = bytes(range(0x40, 0x59))   # 25 registers
        for k, v in enumerate(regs):
            s.bus(0xD400 + k, v, 1000 + 6 * k)
        s.poll(2000)
        s.stop(2000)
        blocks = [b for w in s.of(sp.Writes) for b in w.blocks]
        self.assertEqual(len(blocks), 1)
        self.assertEqual((blocks[0].time, blocks[0].addr, blocks[0].data), (1000, 0xD400, regs))

    def test_splits(self):
        s = Sim([(0xD400, 0xD41C)], gap=10)
        s.bus(0xD400, 1, 100)
        s.bus(0xD401, 2, 110)   # +10: joins
        s.bus(0xD402, 3, 121)   # +11: new block
        s.bus(0xD404, 4, 125)   # not consecutive: new block
        s.bus(0xD404, 5, 129)   # same address again: new block
        s.bus(0xD405, 6, 130)   # joins
        s.stop(200)
        got = [(b.time, b.addr, b.data) for w in s.of(sp.Writes) for b in w.blocks]
        self.assertEqual(got, [(100, 0xD400, b"\x01\x02"), (121, 0xD402, b"\x03"),
                               (125, 0xD404, b"\x04"), (129, 0xD404, b"\x05\x06")])

    def test_gap_zero_times_every_write(self):
        s = Sim([(0xD400, 0xD41C)], gap=0)
        for k in range(10):
            s.bus(0xD400 + k, k, 50 + 4 * k)
        s.stop(200)
        self.assertEqual(s.writes(), [(50 + 4 * k, 0xD400 + k, k) for k in range(10)])

    def test_block_length_cap(self):
        s = Sim([(0x0400, 0x07FF)], gap=255)
        for k in range(300):
            s.bus(0x0400 + k, k & 0xFF, 10 + 4 * k)
            if k % 100 == 99:
                s.poll(10 + 4 * k)
        s.stop(2000)
        blocks = [b for w in s.of(sp.Writes) for b in w.blocks]
        self.assertTrue(all(len(b.data) <= 255 for b in blocks))
        self.assertEqual([(a, v) for _, a, v in s.writes()], [(0x0400 + k, k & 0xFF) for k in range(300)])


class Timing(unittest.TestCase):
    def test_cycles_exact_with_jitter(self):
        s = Sim([(0xD418, 0xD418)], gap=0)
        rnd = random.Random(2)
        t = 0
        expect = []
        for _ in range(400):
            t += rnd.choice([1, 2, 3, 63, 125, 19656, 1000000])
            s.bus(0xD418, t & 0xFF, t, jitter=rnd.randint(-150, 150))
            expect.append((t, 0xD418, t & 0xFF))
            if rnd.random() < 0.3:
                s.poll(t)
                s.drain()
        s.stop(t + 10)
        self.assertEqual(s.writes(), expect)

    def test_long_quiet_spell_crosses_stamp_wrap(self):
        # 2^32 CPU ticks is ~7.2 s; poll every ~1 s with no writes, then write again
        s = Sim([(0xD418, 0xD418)], gap=0)
        s.bus(0xD418, 1, 100)
        cycle = 100
        for _ in range(30):
            cycle += PAL
            s.poll(cycle, us_step=1_000_000)
        s.bus(0xD418, 2, cycle + 5)
        s.stop(cycle + 10)
        self.assertEqual(s.writes(), [(100, 0xD418, 1), (cycle + 5, 0xD418, 2)])

    def test_status_time_moves_without_writes(self):
        s = Sim([(0xD418, 0xD418)], status_ms=1000)
        s.poll(PAL // 2, us_step=500_000)
        s.poll(PAL, us_step=500_000)     # 1 s: STATUS
        s.drain()
        st = s.of(sp.Status)
        self.assertEqual(len(st), 1)
        self.assertEqual(st[0].time, PAL)


class LateIsr(unittest.TestCase):
    def test_late_counts_and_thresholds(self):
        s = Sim([(0xD418, 0xD418)])
        LIB.sim_late_cyc.restype = U32
        period = CPU // PAL
        self.assertEqual(LIB.sim_late_cyc(0), period + CPU // 4_000_000)
        self.assertEqual(LIB.sim_late_cyc(1), period + CPU // 1_000_000)
        for cyc in [period + 20, period + 100, period + 200, period + 400, period + 700, 2 * period + 10, 50 * period]:
            LIB.sim_late(cyc)
        s.stop(PAL)
        st = s.of(sp.Status)[-1]
        # thresholds at 600 MHz: +150 cycles (250 ns), +600 (1 us)
        self.assertEqual((st.late, st.late250, st.late1000), (7, 5, 3))

    def test_reset_by_start(self):
        s = Sim([(0xD418, 0xD418)])
        LIB.sim_late(10 * CPU // PAL)
        s.stop(10)
        s2 = Sim([(0xD418, 0xD418)])
        s2.stop(10)
        self.assertEqual(s2.of(sp.Status)[-1].late, 0)


class CpuProbe(unittest.TestCase):
    def setUp(self):
        LIB.sim_probe.argtypes = [U32] * 6
        LIB.sim_probe.restype = U32

    def test_free_share_worst_and_longest(self):
        s = Sim([(0xD418, 0xD418)])
        # a read every 8 cycles, a 400-cycle interrupt every 26 reads (~608 cycles: one PAL phi2 at 600 MHz)
        free1 = LIB.sim_probe(600_000, 8, 26, 400, 0, 0)
        # no interrupts but one 5000-cycle stall, and gaps under the threshold count as free
        free2 = LIB.sim_probe(600_000, 30, 0, 0, 100, 5000)
        s.poll(1, us_step=1)
        LIB.sim_status()
        s.drain()
        st = s.of(sp.Status)[-1]
        self.assertAlmostEqual(free1 / 600_000, 200 / 608, delta=0.002)   # the interrupted read's own 8 count as taken
        self.assertGreaterEqual(free2, 600_000 - 5000 - 30)
        self.assertEqual(st.probe_free, free1 + free2)
        self.assertGreaterEqual(st.probe_cyc, 1_200_000)
        self.assertAlmostEqual(st.cpu_free, (free1 + free2) / st.probe_cyc)
        self.assertIn(st.probe_worst, range(326, 332))      # the first probe's share, per mille
        self.assertEqual(st.probe_stall, 5030)              # the stall plus the read it delayed
        # the next STATUS starts afresh
        LIB.sim_status()
        s.drain()
        st = s.of(sp.Status)[-1]
        self.assertEqual((st.probe_cyc, st.probe_free, st.probe_worst, st.probe_stall), (0, 0, 1000, 0))
        self.assertIsNone(st.cpu_free)

    def test_version_3_status_still_decodes(self):
        p = sp.status_payload(1, 2, late=(3, 2, 1))[:-14]
        st = sp.parse_payload(p)
        self.assertEqual((st.late, st.late250, st.late1000, st.probe_cyc), (3, 2, 1, 0))


class Limits(unittest.TestCase):
    def test_ring_full_counts_drops(self):
        s = Sim([(0xD418, 0xD418)], gap=0)
        for k in range(RING + 50):
            s.bus(0xD418, k & 0xFF, k * 2)
        self.assertEqual(LIB.sim_dropped(), 51)   # one slot is always empty
        s.stop(10 * RING)
        st = s.of(sp.Status)[-1]
        self.assertEqual((st.dropped, st.writes), (51, RING - 1))
        self.assertEqual(len(s.writes()), RING - 1)

    def test_output_full_drops_whole_frames(self):
        s = Sim([(0xD400, 0xD400)], gap=0, flush_us=0)
        cycle = 0
        for k in range(3000):               # never drained: the FIFO fills
            cycle += 10
            s.bus(0xD400, k & 0xFF, cycle)
            s.poll(cycle)
        s.drain()                           # STATUS/BYE need room: drain before the stop
        s.stop(cycle + 10)
        self.assertEqual(s.dec.bad_frames, 0)
        self.assertGreater(s.dec.lost_frames, 0)
        st = s.of(sp.Status)[-1]
        self.assertEqual(st.out_dropped, s.dec.lost_frames)
        self.assertEqual(st.writes, 3000)

    def test_flush_waits_for_flush_us(self):
        s = Sim([(0xD400, 0xD400)], flush_us=2000)
        s.drain()
        s.bus(0xD400, 1, 10)
        s.poll(20, us_step=500)
        s.drain()
        self.assertEqual(s.of(sp.Writes), [])
        s.poll(30, us_step=1999)       # 1999 us since the poll that took the write
        s.drain()
        self.assertEqual(s.of(sp.Writes), [])
        s.poll(40, us_step=1)
        s.drain()
        self.assertEqual(len(s.of(sp.Writes)), 1)


class Stream(unittest.TestCase):
    def test_fuzz_round_trip_with_junk(self):
        rnd = random.Random(3)
        ranges = [(0xD400, 0xD41C), (0xD500, 0xD51C), (0xDE00, 0xDEFF), (0x0001, 0x0001)]
        gap = 12
        s = Sim(ranges, gap=gap, flush_us=1000, status_ms=50)
        watched = [a for f, l in ranges for a in range(f, l + 1)]
        expect = []
        cycle = 0
        wire = b""
        for _ in range(20000):
            cycle += rnd.choice([3, 4, 6, 8, 12, 13, 40, 900])
            a = rnd.choice(watched) if rnd.random() < 0.7 else (expect[-1][1] + 1 if expect else 0xD400)
            if a > 0xFFFF or not any(f <= a <= l for f, l in ranges):
                a = 0xD400
            v = rnd.randrange(256)
            self.assertEqual(s.bus(a, v, cycle, jitter=rnd.randint(-100, 100)), 1)
            expect.append((cycle, a, v))
            if rnd.random() < 0.05:
                s.poll(cycle, us_step=rnd.randint(10, 3000))
                buf = ctypes.create_string_buffer(OUT)
                n = LIB.sim_take(buf, OUT)
                wire += buf.raw[:n]
                if rnd.random() < 0.2:      # the TR printing text, or a reply token, in the stream
                    wire += rnd.choice([b"Busy!\n", b"\xcc\x64", b"Resetting C64\r\n", bytes([0, 0, 7])])
        s.poll(cycle + 1)
        LIB.sim_stop(0, s.us + 100, stamp_of(cycle + 2))
        buf = ctypes.create_string_buffer(OUT)
        while True:
            n = LIB.sim_take(buf, OUT)
            if not n:
                break
            wire += buf.raw[:n]
        dec = sp.Decoder()
        frames = list(dec.feed(wire))
        self.assertEqual(dec.lost_frames, 0)
        got = sp.writes_of(frames)
        self.assertEqual([(a, v) for _, a, v in got], [(a, v) for _, a, v in expect])
        # a write's time is its block's: at or before the real time, by at most gap per byte before it
        for (bt, _, _), (t, _, _) in zip(got, expect):
            self.assertLessEqual(bt, t)
        k = 0
        for f in frames:
            if isinstance(f, sp.Writes):
                for b in f.blocks:
                    self.assertEqual(b.time, expect[k][0])
                    for j in range(1, len(b.data)):
                        self.assertLessEqual(expect[k + j][0] - expect[k + j - 1][0], gap)
                    k += len(b.data)
        self.assertEqual(k, len(expect))
        self.assertIsInstance(frames[-1], sp.Bye)


if __name__ == "__main__":
    unittest.main()
