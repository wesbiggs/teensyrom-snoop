"""The USB audio driver's portable half (firmware/usbhost/isoaudio.c) through ctypes: UAC1 descriptor parsing
against the owner's dongle, siTD words, the frame plan, the test tone.  make test-audio builds the library."""
import ctypes as C
import math
import os
import random
import struct
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(ROOT, "build", "libisoaudio.so")

OK, NOT_AS, NO_OUT, NO_FORMAT, ASYNC, SMALL = 1, 0, -1, -2, -3, -4


class Stream(C.Structure):
    _fields_ = [("iface", C.c_uint8), ("alt", C.c_uint8), ("ep", C.c_uint8), ("interval", C.c_uint8),
                ("max_packet", C.c_uint16), ("sync", C.c_uint8), ("terminal", C.c_uint8), ("rates", C.c_uint8),
                ("freq_control", C.c_uint8), ("channels", C.c_uint8), ("subframe", C.c_uint8), ("bits", C.c_uint8)]


class Unit(C.Structure):
    _fields_ = [("id", C.c_uint8), ("subtype", C.c_uint8), ("nsrc", C.c_uint8), ("src", C.c_uint8 * 4),
                ("ctl_master", C.c_uint8), ("ctl_ch", C.c_uint8 * 2), ("type", C.c_uint16)]


class Control(C.Structure):
    _fields_ = [("iface", C.c_uint8), ("n", C.c_uint8), ("unit", Unit * 16)]


class Tone(C.Structure):
    _fields_ = [("phase", C.c_uint32), ("step", C.c_uint32), ("amp", C.c_int32)]


def lib():
    L = C.CDLL(LIB)
    L.isoaudio_parse_control.argtypes = [C.c_char_p, C.c_uint32, C.POINTER(Control)]
    L.isoaudio_find_playback.argtypes = [C.c_char_p, C.c_uint32, C.c_uint32, C.POINTER(Stream)]
    L.isoaudio_feature_unit.argtypes = [C.POINTER(Control), C.c_uint8]
    L.isoaudio_feature_unit.restype = C.POINTER(Unit)
    L.isoaudio_fu_channels.argtypes = [C.POINTER(Unit), C.c_uint8, C.c_uint8 * 2]
    L.isoaudio_sitd_out.argtypes = [C.c_uint32 * 7, C.c_uint8, C.c_uint8, C.c_uint8, C.c_uint8, C.c_uint32,
                                    C.c_uint32, C.c_uint32, C.c_int]
    L.isoaudio_splits.argtypes = [C.c_uint32]
    L.isoaudio_splits.restype = C.c_uint32
    L.isoaudio_plan.argtypes = [C.c_uint32, C.POINTER(C.c_uint32), C.c_uint32, C.c_uint32, C.POINTER(C.c_uint32)]
    L.isoaudio_plan.restype = C.c_uint32
    L.isoaudio_tone_init.argtypes = [C.POINTER(Tone), C.c_uint32, C.c_uint32, C.c_int32]
    L.isoaudio_tone_next.argtypes = [C.POINTER(Tone)]
    L.isoaudio_tone_next.restype = C.c_int16
    L.isoaudio_tone_packet.argtypes = [C.POINTER(Tone), C.c_char_p, C.c_uint32]
    return L


# ---- descriptors -------------------------------------------------------------------------------------------

def iface(num, alt, neps, cls=1, sub=2):
    return bytes([9, 4, num, alt, neps, cls, sub, 0, 0])


def as_general(link, fmt=0x0001):
    return bytes([7, 0x24, 1, link, 1]) + struct.pack("<H", fmt)


def fmt_type1(ch, subframe, bits, rates):
    """rates: a list of discrete rates, or ('range', lo, hi)"""
    if isinstance(rates, tuple):
        body = bytes([0]) + rates[1].to_bytes(3, "little") + rates[2].to_bytes(3, "little")
    else:
        body = bytes([len(rates)]) + b"".join(r.to_bytes(3, "little") for r in rates)
    d = bytes([0x24, 2, 1, ch, subframe, bits]) + body
    return bytes([len(d) + 1]) + d


def ep(addr, attrs, maxp, interval=1):
    return bytes([9, 5, addr, attrs]) + struct.pack("<H", maxp) + bytes([interval, 0, 0])


def cs_ep(freq_control=1):
    return bytes([7, 0x25, 1, freq_control, 0, 0, 0])


def config(*parts):
    body = b"".join(parts)
    n = len({(p[2]) for p in parts if len(p) >= 9 and p[1] == 4})
    return bytes([9, 2]) + struct.pack("<H", 9 + len(body)) + bytes([n, 1, 0, 0x80, 50]) + body


def control_iface():
    """The AudioControl interface: a typical UAC1 headset-adapter topology.  The units' IDs are typical values,
    not read from the device (docs/USB-AUDIO.md has only the lsusb summary)."""
    units = (bytes([12, 0x24, 2, 1, 0x01, 0x01, 0, 2, 3, 0, 0, 0])      # IT 1: USB streaming, 2 ch (L, R)
             + bytes([10, 0x24, 6, 2, 1, 1, 0x03, 0x00, 0x00, 0])        # FU 2 <- 1: master mute + volume
             + bytes([9, 0x24, 3, 3, 0x01, 0x03, 0, 2, 0])               # OT 3 <- 2: speaker
             + bytes([12, 0x24, 2, 4, 0x01, 0x02, 0, 1, 0, 0, 0, 0])     # IT 4: microphone, 1 ch
             + bytes([9, 0x24, 6, 5, 4, 1, 0x43, 0x00, 0])               # FU 5 <- 4: mute, volume, AGC
             + bytes([9, 0x24, 3, 6, 0x01, 0x01, 0, 5, 0]))              # OT 6 <- 5: USB streaming
    header = bytes([10, 0x24, 1, 0x00, 0x01]) + struct.pack("<H", 10 + len(units)) + bytes([2, 1, 2])
    return iface(0, 0, 0, sub=1) + header + units


def ab13x():
    """The owner's dongle, 0020:0b21 "AB13X USB Audio" (docs/USB-AUDIO.md, lsusb -v on a Pi): full speed, UAC1.
    Capture: interface 1, endpoint 0x83 IN.  Playback: interface 2, alternate setting 1, endpoint 0x03 OUT,
    isochronous synchronous, bInterval 1, wMaxPacketSize 384, 16-bit stereo PCM at 48000 Hz only.  HID:
    interface 3, two interrupt endpoints."""
    return config(
        control_iface(),
        iface(1, 0, 0), iface(1, 1, 1), as_general(6), fmt_type1(1, 2, 16, [48000]),
        ep(0x83, 0x05, 96), cs_ep(1),
        iface(2, 0, 0), iface(2, 1, 1), as_general(1), fmt_type1(2, 2, 16, [48000]),
        ep(0x03, 0x0D, 384), cs_ep(1),
        iface(3, 0, 2, cls=3, sub=0), bytes([9, 0x21, 0x11, 0x01, 0, 1, 0x22, 0x40, 0]),
        bytes([7, 5, 0x84, 3, 4, 0, 1]), bytes([7, 5, 0x04, 3, 4, 0, 1]))


def interfaces(cfg):
    """(offset, descriptor) of every interface descriptor: what USBHost_t36's claim_drivers() offers drivers"""
    i, out = 9, []
    while i < len(cfg):
        if cfg[i + 1] == 4 and cfg[i] == 9:
            out.append(i)
        i += cfg[i]
    return out


@unittest.skipUnless(os.path.exists(LIB), "build/libisoaudio.so missing (make test-audio)")
class Descriptors(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.L = lib()

    def find(self, cfg, rate=48000):
        """What USBAudioOut::claim() sees, interface by interface: the first AudioStreaming verdict that is OK,
        else the per-interface verdicts."""
        verdicts = []
        for off in interfaces(cfg):
            s = Stream()
            r = self.L.isoaudio_find_playback(cfg[off:], len(cfg) - off, rate, C.byref(s))
            verdicts.append((cfg[off + 2], cfg[off + 3], r))
            if r == OK:
                return s, verdicts
        return None, verdicts

    def test_owners_dongle(self):
        cfg = ab13x()
        s, verdicts = self.find(cfg)
        # the AudioControl interface is not streaming; the capture alternate settings have no OUT endpoint
        self.assertEqual(verdicts, [(0, 0, NOT_AS), (1, 0, NO_OUT), (1, 1, NO_OUT), (2, 0, OK)])
        self.assertEqual((s.iface, s.alt, s.ep, s.max_packet, s.interval), (2, 1, 0x03, 384, 1))
        self.assertEqual((s.sync, s.channels, s.subframe, s.bits, s.rates), (3, 2, 2, 16, 1))
        self.assertEqual(s.terminal, 1)
        self.assertEqual(s.freq_control, 1)    # a control is offered, but one rate only: the driver skips SET_CUR
        # the AudioControl units: the feature unit in the playback path, and which channels to address
        ac = Control()
        self.assertEqual(self.L.isoaudio_parse_control(cfg[9:], len(cfg) - 9, C.byref(ac)), 1)
        self.assertEqual((ac.iface, ac.n), (0, 6))
        fu = self.L.isoaudio_feature_unit(C.byref(ac), s.terminal)
        self.assertTrue(fu)
        self.assertEqual((fu.contents.id, fu.contents.ctl_master), (2, 0x03))
        ch = (C.c_uint8 * 2)()
        self.assertEqual((self.L.isoaudio_fu_channels(fu, 1, ch), ch[0]), (1, 0))   # mute: master
        self.assertEqual((self.L.isoaudio_fu_channels(fu, 2, ch), ch[0]), (1, 0))   # volume: master
        mic = self.L.isoaudio_feature_unit(C.byref(ac), 4)
        self.assertEqual(mic.contents.id, 5)                   # the capture path's own unit, not ours
        # not an AudioControl interface
        self.assertEqual(self.L.isoaudio_parse_control(cfg[9 + len(control_iface()):], 9, C.byref(ac)), 0)

    def playback(self, *alts, link=1):
        """an AudioControl interface and one AudioStreaming interface 1 with alt 0 plus the given alternates"""
        parts = [control_iface(), iface(1, 0, 0)]
        for k, (fmt, endpoint) in enumerate(alts, start=1):
            parts += [iface(1, k, 1), as_general(link), fmt, endpoint, cs_ep(1)]
        return config(*parts)

    def test_formats(self):
        stereo48 = fmt_type1(2, 2, 16, [48000])
        sync = ep(0x01, 0x0D, 192)
        cases = [
            ((fmt_type1(2, 3, 24, [48000]), sync), NO_FORMAT),            # 24-bit only
            ((fmt_type1(1, 2, 16, [48000]), sync), NO_FORMAT),            # mono
            ((fmt_type1(2, 2, 16, [44100]), sync), NO_FORMAT),            # 44.1 kHz only
            ((fmt_type1(2, 2, 16, [44100, 48000]), sync), OK),            # two rates: rates == 2, SET_CUR sent
            ((fmt_type1(2, 2, 16, ("range", 8000, 96000)), sync), OK),    # a continuous range
            ((fmt_type1(2, 2, 16, ("range", 8000, 44100)), sync), NO_FORMAT),
            ((stereo48, ep(0x01, 0x05, 196)), ASYNC),                     # asynchronous: needs feedback
            ((stereo48, ep(0x01, 0x09, 196)), OK),                        # adaptive
            ((stereo48, ep(0x01, 0x01, 196)), OK),                        # no synchronisation type
            ((stereo48, ep(0x01, 0x0D, 100)), SMALL),                     # cannot carry 192 bytes
            ((stereo48, ep(0x81, 0x0D, 192)), NO_OUT),                    # IN
            ((stereo48, ep(0x01, 0x02, 192)), NO_OUT),                    # bulk
        ]
        for alt, want in cases:
            cfg = self.playback(alt)
            s, verdicts = self.find(cfg)
            got = OK if s else verdicts[-1][2]
            self.assertEqual(got, want, f"{alt[0].hex()} {alt[1].hex()}")
        s, _ = self.find(self.playback((fmt_type1(2, 2, 16, [44100, 48000]), sync)))
        self.assertEqual((s.rates, s.freq_control), (2, 1))
        s, _ = self.find(self.playback((fmt_type1(2, 2, 16, ("range", 8000, 96000)), sync)))
        self.assertEqual(s.rates, 0)
        # PCM only (format tag 1)
        cfg = config(control_iface(), iface(1, 0, 0), iface(1, 1, 1), as_general(1, fmt=0x1001), stereo48, sync)
        self.assertEqual(self.find(cfg)[1][-1][2], NO_FORMAT)

    def test_picks_the_matching_alternate(self):
        cfg = self.playback((fmt_type1(2, 3, 24, [48000]), ep(0x02, 0x0D, 288)),
                            (fmt_type1(2, 2, 16, [48000]), ep(0x02, 0x0D, 192)))
        s, _ = self.find(cfg)
        self.assertEqual((s.iface, s.alt, s.max_packet), (1, 2, 192))

    def test_feature_unit_behind_a_mixer_and_per_channel_controls(self):
        units = (bytes([12, 0x24, 2, 1, 0x01, 0x01, 0, 2, 3, 0, 0, 0])
                 + bytes([13, 0x24, 4, 7, 2, 1, 9, 2, 3, 0, 0, 0, 0])                        # mixer 7 <- 1, 9
                 + bytes([10, 0x24, 6, 8, 7, 1, 0x00, 0x03, 0x02, 0])                         # FU 8 <- 7: per channel
                 + bytes([9, 0x24, 3, 3, 0x01, 0x03, 0, 8, 0]))
        header = bytes([9, 0x24, 1, 0x00, 0x01]) + struct.pack("<H", 9 + len(units)) + bytes([1, 1])
        ac_bytes = iface(0, 0, 0, sub=1) + header + units
        ac = Control()
        self.assertEqual(self.L.isoaudio_parse_control(ac_bytes, len(ac_bytes), C.byref(ac)), 1)
        fu = self.L.isoaudio_feature_unit(C.byref(ac), 1)
        self.assertEqual(fu.contents.id, 8)
        ch = (C.c_uint8 * 2)()
        self.assertEqual(self.L.isoaudio_fu_channels(fu, 1, ch), 1)                  # mute on channel 1 only
        self.assertEqual(ch[0], 1)
        self.assertEqual(self.L.isoaudio_fu_channels(fu, 2, ch), 2)                  # volume on 1 and 2
        self.assertEqual(list(ch), [1, 2])
        self.assertFalse(self.L.isoaudio_feature_unit(C.byref(ac), 4))               # nothing downstream of 4

    def test_malformed_descriptors_do_not_crash(self):
        cfg = ab13x()
        rng = random.Random(7)
        offs = interfaces(cfg)
        for _ in range(3000):
            b = bytearray(cfg)
            for _ in range(rng.randint(1, 6)):
                b[rng.randrange(len(b))] = rng.randrange(256)
            b = bytes(b[:rng.randint(9, len(b))])
            off = rng.choice(offs + [9])
            if off >= len(b):
                continue
            s, ac = Stream(), Control()
            r = self.L.isoaudio_find_playback(b[off:], len(b) - off, 48000, C.byref(s))
            self.assertIn(r, (OK, NOT_AS, NO_OUT, NO_FORMAT, ASYNC, SMALL))
            if self.L.isoaudio_parse_control(b[off:], len(b) - off, C.byref(ac)):
                self.assertLessEqual(ac.n, 16)
                self.L.isoaudio_feature_unit(C.byref(ac), 1)


@unittest.skipUnless(os.path.exists(LIB), "build/libisoaudio.so missing (make test-audio)")
class SiTD(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.L = lib()

    def words(self, *args):
        w = (C.c_uint32 * 7)()
        r = self.L.isoaudio_sitd_out(w, *args)
        return r, list(w)

    def test_one_frame_of_48_khz_stereo(self):
        # device 5, endpoint 3, on the root port (the i.MX RT's embedded TT: hub 0, port 0), microframe 0
        r, w = self.words(5, 3, 0, 0, 0, 0x20201000, 192, 1)
        self.assertEqual(r, 0)
        self.assertEqual(w[1], 0x00000305)           # OUT, port 0, hub 0, endpoint 3, address 5
        self.assertEqual(w[2], 0x03)                  # two start-splits (188 + 4 bytes), microframes 0 and 1; no C-mask
        self.assertEqual(w[3], 0x80C00080)            # IOC, 192 bytes, active
        self.assertEqual(w[4], 0x20201000)
        self.assertEqual(w[5], 0x20202000 | 1 << 3 | 2)   # page 1, TP begin, T-count 2
        self.assertEqual(w[6], 1)

    def test_splits_hub_and_shift(self):
        self.assertEqual([self.L.isoaudio_splits(n) for n in (1, 188, 189, 192, 376, 377, 1023)], [1, 1, 2, 2, 2, 3, 6])
        r, w = self.words(9, 1, 4, 3, 2, 0x2020F0C0, 100, 0)   # behind port 3 of hub 4, microframe 2
        self.assertEqual(r, 0)
        self.assertEqual(w[1], 3 << 24 | 4 << 16 | 1 << 8 | 9)
        self.assertEqual(w[2], 0x04)
        self.assertEqual(w[3], 100 << 16 | 0x80)                 # no IOC
        self.assertEqual(w[5], 0x20210000 | 0 << 3 | 1)          # TP all, T-count 1
        self.assertEqual(self.words(5, 3, 0, 0, 4, 0, 192, 1)[0], 0)    # splits in microframes 4, 5
        self.assertEqual(self.words(5, 3, 0, 0, 5, 0, 192, 1)[0], -1)   # 5, 6: too late in the frame
        self.assertEqual(self.words(5, 3, 0, 0, 0, 0, 1024, 1)[0], -1)  # more than a full-speed packet


@unittest.skipUnless(os.path.exists(LIB), "build/libisoaudio.so missing (make test-audio)")
class FramePlan(unittest.TestCase):
    """isoaudio_plan against a model of the controller: slot = frame % 32; a refill comes every so many frames."""
    SIZE, DEPTH = 32, 8

    @classmethod
    def setUpClass(cls):
        cls.L = lib()

    def run_plan(self, gaps, start=5):
        nxt, missed = C.c_uint32((start + 2) % self.SIZE), C.c_uint32()
        queued = {}               # absolute frame -> the refill frame that queued it
        reported = 0
        t = start
        first = start + 2
        for gap in [0] + gaps:
            t += gap
            cur = t % self.SIZE
            n = self.L.isoaudio_plan(cur, C.byref(nxt), self.SIZE, self.DEPTH, C.byref(missed))
            reported += missed.value
            for k in range(n):
                slot = (nxt.value + k) % self.SIZE
                frame = t + (slot - cur) % self.SIZE
                self.assertGreaterEqual(frame, t + 2)              # never the frame the controller may be fetching
                self.assertLessEqual(frame, t + self.DEPTH)        # the ring's depth
                self.assertNotIn(frame, queued)                    # each frame once
                queued[frame] = t
            nxt.value = (nxt.value + n) % self.SIZE
        last = max(queued)
        empty = [f for f in range(first, last + 1) if f not in queued]
        return queued, reported, empty

    def test_steady_refill_never_misses(self):
        rng = random.Random(1)
        for each in (1, 2, 3, 6):     # a refill every frame (each siTD's IOC) or later, up to depth-2 frames late
            queued, reported, empty = self.run_plan([rng.randint(1, each) for _ in range(3000)])
            self.assertEqual(empty, [])
            self.assertEqual(reported, 0)

    def test_late_refills_are_counted_as_missed(self):
        rng = random.Random(2)
        gaps = [rng.choice([1, 1, 1, 2, 5, 9, 12, 20]) for _ in range(3000)]
        queued, reported, empty = self.run_plan(gaps)
        self.assertGreater(len(empty), 100)
        self.assertEqual(reported, len(empty))   # every frame that went out empty, counted


@unittest.skipUnless(os.path.exists(LIB), "build/libisoaudio.so missing (make test-audio)")
class TestTone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.L = lib()

    def tone(self, hz=1000, amp=8192):
        t = Tone()
        self.L.isoaudio_tone_init(C.byref(t), hz, 48000, amp)
        return t

    def packets(self, t, n, frames=48):
        out = []
        for _ in range(n):
            buf = C.create_string_buffer(frames * 4 + 4)
            buf[frames * 4] = b"\xEE"                                   # a guard byte after the packet
            self.L.isoaudio_tone_packet(C.byref(t), buf, frames)
            self.assertEqual(buf.raw[frames * 4], 0xEE)                 # exactly 192 bytes written
            out.append(buf.raw[:frames * 4])
        return out

    def test_1khz_packets(self):
        t = self.tone()
        self.assertEqual(t.step, round(2 ** 32 / 48))
        pk = self.packets(t, 1000)                                      # one second
        self.assertTrue(all(len(p) == 192 for p in pk))
        samples = struct.unpack(f"<{1000 * 96}h", b"".join(pk))
        left, right = samples[0::2], samples[1::2]
        self.assertEqual(left, right)
        ups = sum(1 for a, b in zip(left, left[1:]) if a < 0 <= b)
        self.assertIn(ups, (999, 1000))                                 # 1000 Hz
        self.assertLessEqual(max(left), 8192)
        self.assertGreaterEqual(max(left), 8190)
        self.assertGreaterEqual(min(left), -8192)
        # against the exact sine, sample by sample (phase from the same 32-bit accumulator)
        err = max(abs(v - 8192 * math.sin(2 * math.pi * ((i * t.step) % 2 ** 32) / 2 ** 32)) for i, v in enumerate(left))
        self.assertLess(err, 2.0)
        # 1 kHz at 48 kHz: every 1 ms packet holds exactly one period, so they are (nearly) the same
        self.assertLessEqual(max(abs(a - b) for a, b in zip(left[:48], left[-48:])), 2)

    def test_phase_continuous_across_packets(self):
        for hz, frames in ((1000, 48), (441, 48), (997, 47), (3000, 49)):
            a, b = self.tone(hz), self.tone(hz)
            split = b"".join(self.packets(a, 40, frames))
            whole = self.packets(b, 1, 40 * frames)[0]
            self.assertEqual(split, whole, f"{hz} Hz in packets of {frames}")

    def test_amplitude_and_silence(self):
        t = self.tone(1000, 0)
        self.assertEqual(set(self.packets(t, 3)[0]), {0})
        t = self.tone(1000, 40000)                                       # clamped to full scale
        s = struct.unpack("<96h", self.packets(t, 1)[0])
        self.assertGreaterEqual(max(s), 32760)          # the sample nearest the crest is 4 phase units short
        self.assertGreaterEqual(min(s), -32767)


class Command(unittest.TestCase):
    """64 53 03 mode (protocol 6) from the host side: snoopproto and trsnoop.py --tone."""

    def test_bytes(self):
        import sys
        sys.path.insert(0, os.path.join(ROOT, "host"))
        import snoopproto as sp
        self.assertEqual(sp.PROTO_VERSION, 6)
        self.assertEqual(sp.audio_command(sp.AUDIO_TONE), bytes([0x64, 0x53, 3, 1]))
        self.assertEqual(sp.audio_command(sp.AUDIO_SILENCE), bytes([0x64, 0x53, 3, 0]))
        self.assertEqual(sp.audio_command(sp.AUDIO_REPORT), bytes([0x64, 0x53, 3, 2]))
        with self.assertRaises(ValueError):
            sp.audio_command(3)

    def test_trsnoop_tone(self):
        import contextlib
        import io
        import sys
        sys.path.insert(0, os.path.join(ROOT, "host"))
        import trsnoop

        class Chan:
            def __init__(self, answer):
                self.sent, self.answer = b"", [answer[:5], answer[5:]]   # the answer in two reads

            def write(self, b):
                self.sent += b

            def read(self, n=4096):
                return self.answer.pop(0) if self.answer else b""

        line = b"Snoop audio: tone 1000 Hz; 0020:0B21 interface 2 alt 1 ep 03, streaming; frames 0, errors 0, missed 0"
        ch = Chan(b"\xcc\x64" + line + b"\n")
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertTrue(trsnoop.audio(ch, "on"))
        self.assertEqual(ch.sent, bytes([0x64, 0x53, 3, 1]))
        self.assertEqual(out.getvalue().strip(), line.decode())
        ch = Chan(b"\x7f\x9bSnoop: unknown op\n")                        # firmware before 0.9-sn8
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertFalse(trsnoop.audio(ch, "off"))
        self.assertIn("unknown op", err.getvalue())
        self.assertEqual(ch.sent, bytes([0x64, 0x53, 3, 0]))


if __name__ == "__main__":
    unittest.main()
