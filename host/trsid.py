#!/usr/bin/env python3
"""trsid: hear the SID writes a TeensyROM captures, through reSIDfp.

  trsid.py --tcp 192.168.1.50                      # play live (needs sounddevice)
  trsid.py --port /dev/ttyACM0 --wav live.wav      # play and record
  trsid.py --tcp 192.168.1.50 --no-play --wav x.wav --seconds 30
  trsid.py --replay sid.snoop --wav sid.wav        # a trsnoop.py --raw recording to WAV
  trsid.py --tcp ... --sid D400 --sid D500 --stereo --model 8580

Needs: pip install pyresidfp numpy (and sounddevice for playing).  The capture is started with
the SID ranges (each --sid base, $00-$1C), gap 0, so every register write has its own cycle.
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import snoopproto as sp  # noqa: E402


def write_wav(path, samples, rate, channels):
    with wave.open(path, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(samples.astype("<i2").tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="TeensyROM USB serial port")
    g.add_argument("--tcp", help="TeensyROM IP address (TCP 2112)")
    g.add_argument("--replay", help="a file written by trsnoop.py --raw")
    ap.add_argument("--tcp-port", type=int, help="default: the snoop data port 6581, or 2112 when the firmware has none")
    ap.add_argument("--sid", action="append", default=[], help="SID base address, hex (default D400; repeat for more)")
    ap.add_argument("--stereo", action="store_true", help="SID 1 left, SID 2 right")
    ap.add_argument("--model", choices=["6581", "8580"], default="6581")
    ap.add_argument("--no-filter", action="store_true")
    ap.add_argument("--rate", type=int, default=48000)
    ap.add_argument("--wav", help="write the audio to this WAV file")
    ap.add_argument("--no-play", action="store_true", help="live: record only, no sound card")
    ap.add_argument("--seconds", type=float, default=0, help="live: stop after this long")
    ap.add_argument("--latency", type=float, help="live: seconds behind the C64 (the transport margin), fixed. "
                    "Default: automatic, just above how late frames arrive (99th percentile + 10 ms)")
    ap.add_argument("--send-ms", type=int, default=0, help="how often the TeensyROM sends, ms (default: 20 on USB "
                    "serial, at once on TCP). Every USB transfer briefly stops the bus interrupt: do not go below 20 on USB")
    ap.add_argument("--buffer", type=float, default=0.03, help="live: sound card FIFO target, seconds")
    ap.add_argument("--isr", action="store_true", help="live: print the TeensyROM's late bus-interrupt counts every second "
                    "(firmware 0.9-sn3+): a cartridge game misbehaves when '>250ns' climbs; and the CPU share the "
                    "interrupts leave to the main loop (0.9-sn6+)")
    a = ap.parse_args()

    try:
        import numpy as np  # noqa: F401
        import sidplay
    except ImportError as e:
        sys.exit(f"{e}: pip install pyresidfp numpy")
    if sidplay.SID is None:
        sys.exit("needs pyresidfp: pip install pyresidfp")

    bases = [int(b.replace("$", ""), 16) for b in a.sid] or [0xD400]
    kw = dict(bases=bases, sample_rate=a.rate, model=a.model, stereo=a.stereo, filter=not a.no_filter)
    channels = 2 if a.stereo else 1

    if a.replay:
        with open(a.replay, "rb") as fh:
            r, samples = sidplay.render_stream(fh.read(), **kw)
        secs = len(samples) / a.rate
        print(f"# {secs:.2f} s, {r.ignored} writes outside the SIDs, {r.late} late")
        if a.wav:
            write_wav(a.wav, samples, a.rate, channels)
            print(f"# wrote {a.wav}")
        elif not a.no_play:
            import sounddevice as sd
            sd.play(samples, a.rate, blocking=True)
        return

    # The player and the sound card first: building the renderer runs a 1.2 s emulator warm-up (~0.36 s of real
    # time on a Mac).  Done after the start command, HELLO (which comes with the ack) waited that long before it was
    # fed, C64 time 0 was marked that late, and the first STATUS later put ~0.36 s of audio into the 30 ms FIFO.
    player = sidplay.LivePlayer(sidplay.SidRenderer(**kw), latency=a.latency, buffer=a.buffer, trim_start=not a.no_play)
    recorded = []
    took = [0, None, None]      # samples the sound card took, first and last callback (monotonic)
    stop = threading.Event()

    def pump():
        while not stop.is_set():
            player.pump()
            if a.no_play:   # no callback consumes: drain the FIFO into the recording here
                got = player.drain()
                if len(got):
                    recorded.append(got)
            time.sleep(0.004)

    stream = None
    if not a.no_play:
        try:
            import sounddevice as sd
        except ImportError:
            sys.exit("playing needs sounddevice (pip install sounddevice), or use --no-play --wav")

        def callback(outdata, frames, _time, status):
            if took[1] is None:
                took[1] = time.monotonic()      # samples counted from the second callback on
            else:
                took[0] += frames
                took[2] = time.monotonic()
            s = player.fill(frames)
            outdata[:] = s
            if a.wav:
                recorded.append(s.copy())

        stream = sd.OutputStream(samplerate=a.rate, channels=channels, dtype="int16",
                                 callback=callback, blocksize=256, latency="low")
    dec = sp.Decoder()
    th = threading.Thread(target=pump, daemon=True)
    th.start()
    if stream:
        stream.start()

    from trsnoop import SerialChan, TcpChan, wait_ack
    ch = TcpChan(a.tcp, a.tcp_port) if a.tcp else SerialChan(a.port)   # the data port wants the command at once
    ch.write(sp.start_command([(b, b + 0x1C) for b in bases], 0, a.send_ms))
    ok, rest = wait_ack(ch)
    if not ok:
        stop.set()
        if stream:
            stream.stop()
        sys.exit(1)
    end = time.time() + a.seconds if a.seconds else None
    last_status = None
    cpu_hz, probe_cyc, probe_free = 600_000_000, 0, 0
    emu_cyc, emu_c64 = 0, 0     # the on-Teensy emulator's cost (firmware built with reSID)
    shown = [0, 0]              # drops and repeats at the last --isr line
    marks = []                  # (arrival, C64 time) of each STATUS: the TeensyROM's clock against this computer's
    try:
        data = rest
        while not (end and time.time() > end):
            for f in dec.feed(data):
                player.feed(f)
                if isinstance(f, sp.Hello):
                    cpu_hz = f.cpu_hz
                if isinstance(f, sp.Status):
                    marks.append((time.monotonic(), f.time))
                    probe_cyc += f.probe_cyc
                    probe_free += f.probe_free
                    emu_cyc += f.emu_cyc
                    emu_c64 += f.emu_c64
                    if a.isr and last_status is not None:
                        secs = max((f.time - last_status.time) / player.r.phi2, 1e-3)
                        print(f"# latency {player.latency * 1000:4.0f} ms  FIFO {player.fifo_len * 1000 / a.rate:4.0f} ms  "
                              f"dropped {player.dropped - shown[0]:4d} repeated {player.repeated - shown[1]:4d}  "
                              f"late bus interrupts/s: {(f.late - last_status.late) / secs:7.1f}  "
                              f">250ns: {(f.late250 - last_status.late250) / secs:6.1f}  "
                              f">1us: {(f.late1000 - last_status.late1000) / secs:6.1f}  {f.probe_text(cpu_hz)}  {f.emu_text(cpu_hz, player.r.phi2)}".rstrip(), flush=True)
                        shown[:] = [player.dropped, player.repeated]
                    last_status = f
                if isinstance(f, sp.Bye):
                    raise KeyboardInterrupt
            data = ch.read()
    except (KeyboardInterrupt, ConnectionError):
        pass
    finally:
        try:
            ch.write(sp.stop_command())
        except OSError:
            pass
        stop.set()
        th.join(1)
        if stream:
            stream.stop()
        ch.close()
    if stream is not None:
        print(f"# sound card output latency {stream.latency * 1000:.0f} ms")
    held = f" (at the end {player.fifo_len * 1000 / a.rate:.0f} ms)" if stream is not None else ""
    print(f"# latency {player.latency * 1000:.0f} ms{' (automatic)' if player.auto else ''}, FIFO {a.buffer * 1000:.0f} ms{held}")
    print(f"# writes late {player.r.late}, underruns {player.underruns}, drift: dropped {player.dropped} "
          f"repeated {player.repeated}, lost frames {dec.lost_frames}")
    if last_status is not None:
        secs = max(last_status.time / player.r.phi2, 1e-3)
        print(f"# late bus interrupts in {secs:.0f} s: {last_status.late} (>250ns {last_status.late250}, "
              f">1us {last_status.late1000})")
    if probe_cyc:
        print(f"# CPU free for the main loop, average: {probe_free * 100 / probe_cyc:.1f}%")
    if emu_c64:
        print(f"# reSID on the TeensyROM (6581, 48 kHz, fast sampling), average: {emu_cyc / cpu_hz / (emu_c64 / player.r.phi2) * 100:.1f}% "
              f"of the time (wall clock: interrupts included; at 100% it no longer keeps up) over {emu_c64 / player.r.phi2:.0f} s")
    # The player renders C64 time at the nominal phi2 and the sound card takes `rate` a second; a steady stream
    # of drops (or repeats) means one of the two clocks is off.  Least arrival delay at each end, over >= 10 s.
    if len(marks) >= 12:
        n = max(3, len(marks) // 10)
        m0 = min(marks[:n], key=lambda m: m[0] - m[1] / player.r.phi2)
        m1 = min(marks[-n:], key=lambda m: m[0] - m[1] / player.r.phi2)
        if m1[0] - m0[0] > 5:
            ppm = ((m1[1] - m0[1]) / player.r.phi2 / (m1[0] - m0[0]) - 1) * 1e6
            print(f"# C64 time from the TeensyROM vs this computer's clock: {ppm:+.0f} ppm over {m1[0] - m0[0]:.0f} s "
                  f"(phi2 {player.r.phi2} Hz, cpu {cpu_hz} Hz in HELLO)")
    if took[2] is not None and took[2] - took[1] > 5:
        print(f"# the sound card took {took[0] / (took[2] - took[1]):.1f} samples/s (asked {a.rate})")
    if a.wav and recorded:
        import numpy as np
        write_wav(a.wav, np.concatenate(recorded), a.rate, channels)
        print(f"# wrote {a.wav}")


if __name__ == "__main__":
    main()
