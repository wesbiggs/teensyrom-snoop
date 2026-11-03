#!/usr/bin/env python3
"""A fake TeensyROM on TCP for trying the clients without hardware: answers the snoop start command
with AckToken, then plays a recorded stream (trsnoop.py --raw / sidtrace.py --snoop) at the C64's
pace (each frame sent when its time comes), and answers stop with Ack + BYE.

  tools/fake_tr.py build/demo/Popcorn.snoop [--port 2113] [--once]
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "host"))
import snoopproto as sp  # noqa: E402


def frames_with_times(data):
    """(time in C64 cycles, raw frame bytes, phi2) per frame of a recording."""
    out = []
    phi2 = sp.PHI2_PAL
    for chunk in data.split(b"\x00"):
        if not chunk:
            continue
        try:
            raw = sp.cobs_decode(chunk)
            f = sp.parse_payload(raw[:-2])
        except ValueError:
            continue
        t = 0
        if isinstance(f, sp.Hello):
            phi2 = f.phi2_hz
        elif isinstance(f, sp.Writes):
            t = f.blocks[-1].time
        elif isinstance(f, sp.Status):
            t = f.time
        elif isinstance(f, sp.Bye):
            continue                  # sent on stop instead
        out.append((t, b"\x00" + chunk + b"\x00", phi2))
    return out


def serve(conn, frames):
    conn.settimeout(5)
    cmd = conn.recv(64)
    if not cmd.startswith(bytes([sp.CMD_LEAD, sp.CMD_SNOOP, sp.OP_START])):
        conn.sendall(sp.FAIL + b"fake: start expected\n")
        return
    conn.sendall(sp.ACK)
    conn.setblocking(False)
    t0 = time.monotonic()
    worst = 0.0
    for t, fr, phi2 in frames:
        while time.monotonic() - t0 < t / phi2:
            try:
                if conn.recv(64)[:3] == bytes([sp.CMD_LEAD, sp.CMD_SNOOP, sp.OP_STOP]):
                    conn.sendall(sp.ACK + sp.frame(sp.bye_payload(0)))
                    print(f"fake: worst send lag {worst * 1000:.1f} ms", file=sys.stderr)
                    return
            except BlockingIOError:
                pass
            time.sleep(0.001)
        worst = max(worst, time.monotonic() - t0 - t / phi2)
        if worst > 0.02 and not getattr(serve, "warned", False):
            serve.warned = True
            print(f"fake: a frame went out {worst * 1000:.0f} ms late (this machine is busy)", file=sys.stderr)
        conn.sendall(fr)
    conn.sendall(sp.frame(sp.bye_payload(0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--port", type=int, default=2113)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()
    frames = frames_with_times(open(a.recording, "rb").read())
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port))
    srv.listen(1)
    print(f"fake TeensyROM on 127.0.0.1:{a.port}, {len(frames)} frames", flush=True)
    while True:
        conn, _ = srv.accept()
        try:
            serve(conn, frames)
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()
        if a.once:
            break


if __name__ == "__main__":
    main()
