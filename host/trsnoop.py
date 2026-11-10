#!/usr/bin/env python3
"""trsnoop: start a write snoop on a TeensyROM and print or record the stream.

  trsnoop.py --port /dev/ttyACM0 --range D400-D41C
  trsnoop.py --tcp 192.168.1.50 --range D400-D41C --range D500-D51C --raw sid.snoop
  trsnoop.py --replay sid.snoop

Ranges are hex, inclusive: D418, D400-D41C, D400+29.  Ctrl-C sends the stop command.
--raw writes the channel bytes as received (replay them with --replay).
Serial needs pyserial (pip install pyserial); TCP uses snoop's data port 6581 (the TeensyROM's port 2112 with firmware before sn4).
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import snoopproto as sp  # noqa: E402


DATA_PORT, CONTROL_PORT = 6581, 2112


class TcpChan:
    """port None: the snoop data port (6581, firmware 0.9-sn5+; sn4 used 2113), else the TeensyROM's own port (2112)."""
    def __init__(self, host: str, port: int | None = None):
        if port is None:
            try:
                self.s = socket.create_connection((host, DATA_PORT), timeout=5)
                self.port = DATA_PORT
            except ConnectionRefusedError:
                sys.stderr.write(f"# port {DATA_PORT} refused (firmware before 0.9-sn4?): using {CONTROL_PORT}, "
                                 "which other tools then cannot use during the capture\n")
                self.s = socket.create_connection((host, CONTROL_PORT), timeout=5)
                self.port = CONTROL_PORT
        else:
            self.s = socket.create_connection((host, port), timeout=5)
            self.port = port
        self.s.settimeout(0.2)

    def write(self, b: bytes):
        self.s.sendall(b)

    def read(self, n: int = 4096) -> bytes:
        try:
            d = self.s.recv(n)
        except socket.timeout:
            return b""
        if not d:
            raise ConnectionError("TeensyROM closed the connection")
        return d

    def close(self):
        self.s.close()


class SerialChan:
    def __init__(self, port: str):
        try:
            import serial  # type: ignore
        except ImportError:
            sys.exit("serial needs pyserial: pip install pyserial")
        self.s = serial.Serial(port, 115200, timeout=0.2)

    def write(self, b: bytes):
        self.s.write(b)

    def read(self, n: int = 4096) -> bytes:
        return self.s.read(max(1, min(n, self.s.in_waiting or 1)))

    def close(self):
        self.s.close()


def wait_ack(ch, seconds: float = 3.0) -> tuple[bool, bytes]:
    """Read until AckToken or FailToken; returns (ok, bytes after the token)."""
    buf = b""
    end = time.time() + seconds
    while time.time() < end:
        buf += ch.read()
        if sp.ACK in buf:
            return True, buf[buf.index(sp.ACK) + 2:]
        if sp.FAIL in buf:
            time.sleep(0.2)
            buf += ch.read()
            msg = buf[buf.index(sp.FAIL) + 2:].decode("ascii", "replace").strip()
            sys.stderr.write(f"TeensyROM refused: {msg}\n")
            return False, b""
    sys.stderr.write("no answer from the TeensyROM (firmware without teensyrom-snoop?)\n")
    return False, b""


class Printer:
    def __init__(self, quiet: bool):
        self.quiet = quiet
        self.phi2 = sp.PHI2_PAL

    def frame(self, f):
        if isinstance(f, sp.Hello):
            self.phi2 = f.phi2_hz
            rs = " ".join(f"{a:04X}-{b:04X}" for a, b in f.ranges)
            print(f"# hello v{f.version} phi2 {f.phi2_hz} Hz, cpu {f.cpu_hz} Hz, gap {f.max_gap}, ranges {rs}")
        elif isinstance(f, sp.Writes):
            if self.quiet:
                return
            for b in f.blocks:
                secs = b.time / self.phi2
                print(f"{b.time:12d} {secs:10.6f}  {b.addr:04X}: {b.data.hex(' ').upper()}")
        elif isinstance(f, sp.Status):
            print(f"# status t={f.time} writes={f.writes} dropped={f.dropped} out_dropped={f.out_dropped} "
                  f"ring_hiwater={f.ring_hiwater} late_isr={f.late} >250ns={f.late250} >1us={f.late1000}")
        elif isinstance(f, sp.Bye):
            print(f"# bye ({f.reason_text})")
        sys.stdout.flush()


def replay(path: str, quiet: bool):
    dec = sp.Decoder()
    pr = Printer(quiet)
    with open(path, "rb") as fh:
        for f in dec.feed(fh.read()):
            pr.frame(f)
    print(f"# lost frames {dec.lost_frames}, bad frames {dec.bad_frames}, junk bytes {dec.junk_bytes}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="TeensyROM USB serial port")
    g.add_argument("--tcp", help="TeensyROM IP address (snoop data port 6581; see --tcp-port)")
    g.add_argument("--replay", help="decode a file written with --raw")
    ap.add_argument("--tcp-port", type=int, help="default: the snoop data port 6581, or 2112 when the firmware has none")
    ap.add_argument("--range", action="append", default=[], help="hex address or range (repeatable)")
    ap.add_argument("--gap", type=int, default=0, help="C64 cycles a write may follow the previous one and still join its block, sharing its time; 0 (default) = every write exactly timed. For the SID keep 0: see README")
    ap.add_argument("--send-ms", type=int, default=0, help="how often the TeensyROM sends, ms (default: 20 on USB serial, "
                    "at once on TCP). Every USB transfer briefly stops the bus interrupt: do not go below 20 on USB")
    ap.add_argument("--raw", help="also write the received bytes to this file")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--quiet", action="store_true", help="only HELLO/STATUS/BYE lines")
    ap.add_argument("--stop", action="store_true", help="just send the stop command")
    a = ap.parse_args()

    if a.replay:
        replay(a.replay, a.quiet)
        return

    ch = TcpChan(a.tcp, a.tcp_port) if a.tcp else SerialChan(a.port)
    if a.stop:
        ch.write(sp.stop_command())
        ok, _ = wait_ack(ch)
        ch.close()
        sys.exit(0 if ok else 1)

    if not a.range:
        ap.error("at least one --range")
    ranges = [sp.parse_range(r) for r in a.range]
    ch.write(sp.start_command(ranges, a.gap, a.send_ms))
    ok, rest = wait_ack(ch)
    if not ok:
        ch.close()
        sys.exit(1)

    raw = open(a.raw, "wb") if a.raw else None
    dec = sp.Decoder()
    pr = Printer(a.quiet)
    end = time.time() + a.seconds if a.seconds else None
    stopping = False
    try:
        data = rest
        while True:
            if data:
                if raw:
                    raw.write(data)
                for f in dec.feed(data):
                    pr.frame(f)
                    if isinstance(f, sp.Bye) and stopping:
                        raise StopIteration
            if end and time.time() > end and not stopping:
                ch.write(sp.stop_command())
                stopping = True
                end = time.time() + 3
            elif stopping and end and time.time() > end:
                break
            data = ch.read()
    except KeyboardInterrupt:
        ch.write(sp.stop_command())
        t = time.time() + 2
        while time.time() < t:
            d = ch.read()
            if raw and d:
                raw.write(d)
            if any(isinstance(f, sp.Bye) for f in dec.feed(d)):
                break
    except StopIteration:
        pass
    except ConnectionError as e:
        print(f"# {e}")
    finally:
        if raw:
            raw.close()
        ch.close()
    print(f"# lost frames {dec.lost_frames}, bad frames {dec.bad_frames}, junk bytes {dec.junk_bytes}")


if __name__ == "__main__":
    main()
