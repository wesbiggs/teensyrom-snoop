#!/usr/bin/env python3
"""Run a PSID tune's init and play routines in py65 and record its SID register writes with their
C64 cycle, as the TeensyROM would capture them.  For tests and demos without hardware.

  tools/sidtrace.py TUNE.sid|TUNE.sid.h [--seconds 20] [--song N] [--snoop out.snoop] [--gap 32]

--snoop also runs the trace through the real capture core (build/libsnoop.so, as the ISR and the
main loop would) and writes the wire stream, which host/trsnoop.py --replay and host/trsid.py
--replay read.

Simplifications (it is not a C64): play is called every 19656 cycles (PAL, 50 Hz) or the CIA
timer value from the header's speed bit is ignored; no badlines or sprite DMA, so a write's cycle
is exact within its instruction (py65 counts whole instructions: the write is stamped at the
instruction's start); RSID tunes and PSIDs with play address 0 (they install their own IRQ) are
refused; memory banking ($01) is not emulated (I/O is always visible at $D000-$DFFF).
"""
from __future__ import annotations

import argparse
import ctypes
import os
import re
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
import snoopproto as sp  # noqa: E402

FRAME_PAL = 19656          # 312 lines * 63 cycles
FRAME_NTSC = 17095         # 263 * 65


def load_sid_bytes(path: str) -> bytes:
    data = open(path, "rb").read()
    if path.endswith(".h"):    # a C array, as the TeensyROM repo ships its SIDs
        body = data.decode("latin-1")
        body = body[body.index("{") + 1: body.rindex("}")]
        data = bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{1,2})", body))
    return data


class Psid:
    def __init__(self, data: bytes):
        magic, ver, off, load, init, play, songs, start, speed = struct.unpack(">4sHHHHHHHI", data[:22])
        if magic != b"PSID":
            raise ValueError(f"not a PSID ({magic!r}: RSID needs a real C64 environment)")
        self.name = data[0x16:0x36].split(b"\0")[0].decode("latin-1")
        self.author = data[0x36:0x56].split(b"\0")[0].decode("latin-1")
        payload = data[off:]
        if load == 0:
            load = payload[0] | payload[1] << 8
            payload = payload[2:]
        self.load, self.init, self.play = load, init, play
        self.songs, self.start = songs, start
        self.payload = payload
        flags = struct.unpack(">H", data[0x76:0x78])[0] if ver >= 2 and off >= 0x7C else 0
        self.ntsc = (flags >> 2) & 3 == 2
        self.model = {1: "6581", 2: "8580"}.get((flags >> 4) & 3, "6581")
        if play == 0:
            raise ValueError("play address 0: the tune installs its own interrupt (not supported)")


def trace(sid: Psid, seconds: float, song: int | None = None, max_frame_cycles=None):
    from py65.devices.mpu6502 import MPU
    from py65.memory import ObservableMemory

    mem = ObservableMemory()
    for i, b in enumerate(sid.payload):
        mem[(sid.load + i) & 0xFFFF] = b
    writes = []
    mpu = MPU(memory=mem)
    cur = {"cycle": 0}

    def on_write(addr, value):
        writes.append((cur["cycle"], addr, value))
        return None
    mem.subscribe_to_write(range(0xD400, 0xD420), on_write)

    def call(addr, a=0, budget=200000):
        # JSR addr from a trap at $0000-ish: push a return to $FFF0 (BRK-free loop) and run to it
        mpu.pc = addr
        mpu.a = a
        mpu.sp = 0xFF
        ret = 0xFFEF          # RTS goes to $FFF0
        mem[0x01FF] = ret >> 8
        mem[0x01FE] = ret & 0xFF
        mpu.sp = 0xFD
        start = mpu.processorCycles
        while mpu.pc != 0xFFF0:
            cur["cycle"] = base + (mpu.processorCycles - start)
            mpu.step()
            if mpu.processorCycles - start > budget:
                raise RuntimeError(f"routine at ${addr:04X} ran over {budget} cycles")
        return mpu.processorCycles - start

    frame = FRAME_NTSC if sid.ntsc else FRAME_PAL
    phi2 = sp.PHI2_NTSC if sid.ntsc else sp.PHI2_PAL
    base = 1000
    used = call(sid.init, (song or sid.start) - 1, budget=5_000_000)
    base += used
    base = ((base // frame) + 1) * frame
    end = int(seconds * phi2)
    longest = 0
    while base < end:
        used = call(sid.play)
        longest = max(longest, used)
        base += frame
    return writes, phi2, longest


def capture(writes, phi2, gap, poll_cycles=1000, cpu_hz=600_000_000):
    """The trace through the real core (tests/shim.c), as isrPHI2 + loop() would: returns the wire bytes."""
    lib = ctypes.CDLL(os.path.join(ROOT, "build", "libsnoop.so"))
    u32 = ctypes.c_uint32
    lib.sim_poll.argtypes = [u32, u32]
    lib.sim_stop.argtypes = [ctypes.c_int, u32, u32]
    lib.sim_bus.argtypes = [ctypes.c_uint16, ctypes.c_uint8, u32]
    lib.sim_take.argtypes = [ctypes.c_char_p, u32]
    lib.sim_take.restype = u32
    lib.sim_start.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_int, ctypes.c_int, u32, u32, u32, u32]

    def stamp(c):
        return (round(c * cpu_hz / phi2)) & 0xFFFFFFFF

    def us(c):
        return int(c * 1_000_000 / phi2) & 0xFFFFFFFF

    lib.sim_init()
    flat = (ctypes.c_uint16 * 2)(0xD400, 0xD41C)
    assert lib.sim_start(flat, 1, gap, phi2, cpu_hz, 0, stamp(0)) == 0
    wire = bytearray()
    buf = ctypes.create_string_buffer(65536)
    next_poll = poll_cycles
    hiwater = 0

    def drain():
        while True:
            n = lib.sim_take(buf, 65536)
            if not n:
                return
            wire.extend(buf.raw[:n])

    for c, a, v in writes:
        while c >= next_poll:
            lib.sim_poll(us(next_poll), stamp(next_poll))
            drain()
            next_poll += poll_cycles
        lib.sim_bus(a, v, stamp(c))
    last = writes[-1][0] if writes else 0
    lib.sim_poll(us(last + 1), stamp(last + 1))
    lib.sim_stop(0, us(last + 2), stamp(last + 2))
    drain()
    return bytes(wire)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tune")
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--song", type=int)
    ap.add_argument("--snoop", help="write the captured wire stream here")
    ap.add_argument("--gap", type=int, default=32)
    a = ap.parse_args()
    sid = Psid(load_sid_bytes(a.tune))
    writes, phi2, longest = trace(sid, a.seconds, a.song)
    print(f"# {sid.name} / {sid.author}: {len(writes)} writes in {a.seconds} s "
          f"({len(writes) / a.seconds:.0f}/s), longest play {longest} cycles, {'NTSC' if sid.ntsc else 'PAL'}, {sid.model}")
    if a.snoop:
        wire = capture(writes, phi2, a.gap)
        with open(a.snoop, "wb") as fh:
            fh.write(wire)
        print(f"# {len(wire)} wire bytes ({len(wire) / a.seconds:.0f} B/s) -> {a.snoop}")


if __name__ == "__main__":
    main()
