"""teensyrom-snoop wire protocol: decoder and command builder.  See docs/PROTOCOL.md.

Pure Python 3, no dependencies.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

PROTO_VERSION = 1

F_HELLO, F_WRITES, F_STATUS, F_BYE = 0x01, 0x02, 0x03, 0x04
BYE_REASONS = {0: "stopped", 1: "replaced", 2: "channel"}

# TeensyROM command channel
CMD_LEAD = 0x64
CMD_SNOOP = 0x53            # token 0x6453 ('d' 'S')
OP_STOP, OP_START, OP_STATUS = 0, 1, 2
ACK = bytes([0xCC, 0x64])   # AckToken 0x64CC, sent low byte first
FAIL = bytes([0x7F, 0x9B])  # FailToken 0x9B7F

PHI2_PAL = 985248
PHI2_NTSC = 1022727


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("zero inside COBS frame")
        i += 1
        end = i + code - 1
        if end > n:
            raise ValueError("COBS run past the end")
        out += data[i:end]
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_at, code = 0, 1
    for b in data:
        if b == 0:
            out[code_at] = code
            code_at = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code_at = len(out)
                out.append(0)
                code = 1
    out[code_at] = code
    return bytes(out)


def _varint(buf: bytes, i: int) -> tuple[int, int]:
    v = shift = 0
    while True:
        if i >= len(buf):
            raise ValueError("varint past the end")
        b = buf[i]
        i += 1
        v |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return v, i
        if shift > 63:
            raise ValueError("varint too long")


@dataclass
class Hello:
    version: int
    phi2_hz: int
    cpu_hz: int
    max_gap: int
    ranges: list[tuple[int, int]]


@dataclass
class Block:
    time: int           # C64 cycles since the start of the run, of the block's first write
    addr: int
    data: bytes


@dataclass
class Writes:
    seq: int
    blocks: list[Block]


@dataclass
class Status:
    next_seq: int       # seq the next WRITES frame will carry
    time: int
    dropped: int        # writes lost in the TeensyROM's ISR ring (total this run)
    out_dropped: int    # frames lost because the output buffer was full (total this run)
    ring_hiwater: int   # most ring entries waiting at once since the last STATUS
    writes: int         # writes captured this run


@dataclass
class Bye:
    reason: int

    @property
    def reason_text(self) -> str:
        return BYE_REASONS.get(self.reason, f"unknown {self.reason}")


def parse_payload(p: bytes):
    if not p:
        raise ValueError("empty payload")
    t = p[0]
    if t == F_HELLO:
        version, phi2, cpu, gap, n = struct.unpack_from("<BIIBB", p, 1)
        ranges = [struct.unpack_from("<HH", p, 12 + 4 * k) for k in range(n)]
        if len(p) != 12 + 4 * n:
            raise ValueError("HELLO length")
        return Hello(version, phi2, cpu, gap, ranges)
    if t == F_WRITES:
        seq = struct.unpack_from("<H", p, 1)[0]
        t0, i = _varint(p, 3)
        blocks = []
        prev = t0
        while i < len(p):
            dt, i = _varint(p, i)
            if i + 3 > len(p):
                raise ValueError("block header past the end")
            addr, ln = struct.unpack_from("<HB", p, i)
            i += 3
            if ln == 0 or i + ln > len(p):
                raise ValueError("block data past the end")
            prev += dt
            blocks.append(Block(prev, addr, bytes(p[i:i + ln])))
            i += ln
        return Writes(seq, blocks)
    if t == F_STATUS:
        next_seq = struct.unpack_from("<H", p, 1)[0]
        time, i = _varint(p, 3)
        dropped, out_dropped, hiw, writes = struct.unpack_from("<IIHI", p, i)
        if i + 14 != len(p):
            raise ValueError("STATUS length")
        return Status(next_seq, time, dropped, out_dropped, hiw, writes)
    if t == F_BYE:
        if len(p) != 2:
            raise ValueError("BYE length")
        return Bye(p[1])
    raise ValueError(f"unknown frame type {t:#x}")


@dataclass
class Decoder:
    """Feed it raw channel bytes; it yields parsed frames.

    Anything that is not a good frame (text the TeensyROM prints, a reply token, a frame cut by a
    reconnect) is counted in `junk_bytes` / `bad_frames` and skipped: the 0x00 delimiter resyncs.
    """
    buf: bytearray = field(default_factory=bytearray)
    bad_frames: int = 0
    junk_bytes: int = 0
    next_seq: int | None = None   # None until a HELLO
    lost_frames: int = 0          # WRITES frames the seq numbers say are missing

    def feed(self, data: bytes):
        self.buf += data
        while True:
            z = self.buf.find(0)
            if z < 0:
                if len(self.buf) > 4096:      # no delimiter for that long: not our stream
                    self.junk_bytes += len(self.buf)
                    self.buf.clear()
                return
            chunk = bytes(self.buf[:z])
            del self.buf[:z + 1]
            if not chunk:
                continue
            try:
                raw = cobs_decode(chunk)
                if len(raw) < 3:
                    raise ValueError("short")
                payload, crc = raw[:-2], raw[-2] | raw[-1] << 8
                if crc16(payload) != crc:
                    raise ValueError("crc")
                frame = parse_payload(payload)
            except ValueError:
                self.bad_frames += 1
                self.junk_bytes += len(chunk)
                continue
            if isinstance(frame, Hello):
                self.next_seq = 0
            elif isinstance(frame, (Writes, Status)):
                seq = frame.seq if isinstance(frame, Writes) else frame.next_seq
                if self.next_seq is not None:
                    self.lost_frames += (seq - self.next_seq) & 0xFFFF
                self.next_seq = (seq + 1) & 0xFFFF if isinstance(frame, Writes) else seq
            yield frame


def parse_range(text: str) -> tuple[int, int]:
    """'D400-D41C', 'D418', '$D400-$D7FF', '0xD400+29' -> (first, last), hex."""
    t = text.strip().replace("$", "").replace("0x", "").replace("0X", "")
    if "+" in t:
        a, n = t.split("+", 1)
        first = int(a, 16)
        last = first + int(n, 0) - 1
    elif "-" in t:
        a, b = t.split("-", 1)
        first, last = int(a, 16), int(b, 16)
    else:
        first = last = int(t, 16)
    if not (0 <= first <= last <= 0xFFFF):
        raise ValueError(f"bad range {text!r}")
    return first, last


def start_command(ranges: list[tuple[int, int]], max_gap: int = 0) -> bytes:
    if not 1 <= len(ranges) <= 16:
        raise ValueError("1 to 16 ranges")
    if not 0 <= max_gap <= 255:
        raise ValueError("max_gap 0-255")
    b = bytearray([CMD_LEAD, CMD_SNOOP, OP_START, len(ranges)])
    for first, last in ranges:
        b += struct.pack("<HH", first, last)
    b.append(max_gap)
    b.append(0)  # flags, reserved
    return bytes(b)


def stop_command() -> bytes:
    return bytes([CMD_LEAD, CMD_SNOOP, OP_STOP])


def status_command() -> bytes:
    return bytes([CMD_LEAD, CMD_SNOOP, OP_STATUS])


# ---- encoding (the firmware's side, for tests, fake devices and file conversion) ----------------

def put_varint(v: int) -> bytes:
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        out.append(b | (0x80 if v else 0))
        if not v:
            return bytes(out)


def frame(payload: bytes) -> bytes:
    """One wire frame: 00 COBS(payload + crc16 LE) 00."""
    crc = crc16(payload)
    return b"\x00" + cobs_encode(payload + bytes([crc & 0xFF, crc >> 8])) + b"\x00"


def hello_payload(ranges, phi2_hz=PHI2_PAL, cpu_hz=600_000_000, max_gap=32) -> bytes:
    p = struct.pack("<BBIIBB", F_HELLO, PROTO_VERSION, phi2_hz, cpu_hz, max_gap, len(ranges))
    return p + b"".join(struct.pack("<HH", a, b) for a, b in ranges)


def writes_payload(seq: int, blocks) -> bytes:
    """blocks: (time, addr, data) with ascending times; t0 = the first block's time."""
    p = bytearray(struct.pack("<BH", F_WRITES, seq & 0xFFFF))
    prev = blocks[0][0]
    p += put_varint(prev)
    for t, addr, data in blocks:
        p += put_varint(t - prev) + struct.pack("<HB", addr, len(data)) + bytes(data)
        prev = t
    return bytes(p)


def status_payload(next_seq, time, dropped=0, out_dropped=0, hiwater=0, writes=0) -> bytes:
    return (struct.pack("<BH", F_STATUS, next_seq & 0xFFFF) + put_varint(time)
            + struct.pack("<IIHI", dropped, out_dropped, hiwater, writes))


def bye_payload(reason=0) -> bytes:
    return bytes([F_BYE, reason])


def writes_of(frames) -> list[tuple[int, int, int]]:
    """Expand WRITES frames to (time, addr, value) per byte; times inside a block are the block's."""
    out = []
    for f in frames:
        if isinstance(f, Writes):
            for b in f.blocks:
                for k, v in enumerate(b.data):
                    out.append((b.time, b.addr + k, v))
    return out
