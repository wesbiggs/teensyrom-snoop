#!/usr/bin/env python3
"""Real-time check of native/trsid-native's live path, like tests/live_tcp.py for trsid.py: tools/fake_tr.py
streams a synthetic tune (random notes, so a match can only be at the right place) at the C64's pace,
trsid-native --tcp --no-play --wav records it, and the recording is compared with trsid-native's offline
render of the same stream.  Needs `make -C native` and numpy; timing-dependent, so not in `make test`.

  tests/live_native.py [extra trsid-native options, e.g. --sampling interpolate --stereo --sid D400 --sid D420]
"""
import os
import random
import subprocess
import sys
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
import snoopproto as sp  # noqa: E402

B = os.path.join(ROOT, "build", "native")
EXE = os.path.join(B, "trsid-native")
SNOOP = os.path.join(B, "live.snoop")
REF = os.path.join(B, "live_ref.wav")
OUT = os.path.join(B, "live.wav")
PORT = "23461"
PAL = sp.PHI2_PAL


def tune(seconds=10, bases=(0xD400,)):
    """A 50 Hz player on three voices of each SID, a new random note on each voice every 5 frames."""
    rnd = random.Random(64)
    blocks = []
    for n in range(seconds * 50):
        t = 1000 + n * (PAL // 50)
        for s, base in enumerate(bases):
            if n == 0:
                blocks.append((t + 5 * s, base + 0x15, bytes([0, 0x40, 0xF2, 0x1F])))
            for v in range(3):
                if n % 5 == 0:
                    f = rnd.randrange(800, 9000)
                    wave_ = rnd.choice([0x11, 0x21, 0x41])
                    blocks.append((t + 40 * v + 7 * s + 20, base + 7 * v,
                                   bytes([f & 0xFF, f >> 8, 0x00, 0x08, wave_ & 0xFE, 0x09, 0xA9])))
                    blocks.append((t + 40 * v + 7 * s + 400, base + 7 * v + 4, bytes([wave_])))
    blocks.sort(key=lambda b: b[0])
    ranges = [(b, b + 0x1C) for b in bases]
    w = sp.frame(sp.hello_payload(ranges, max_gap=0))
    seq = 0
    i = 0
    while i < len(blocks):           # as the firmware: a frame goes 2 ms after its first block (or when full)
        k = i + 1
        while k < len(blocks) and k - i < 20 and blocks[k][0] - blocks[i][0] < PAL // 500:
            k += 1
        w += sp.frame(sp.writes_payload(seq, blocks[i:k]))
        seq += 1
        i = k
    return w + sp.frame(sp.status_payload(seq, 1000 + seconds * PAL)) + sp.frame(sp.bye_payload())


def wav(path):
    with wave.open(path) as w:
        a = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
        return a.reshape(-1, w.getnchannels())[:, 0]


def main():
    extra = sys.argv[1:]
    if not os.path.exists(EXE):
        sys.exit("build it first: make -C native")
    bases = [int(extra[i + 1], 16) for i, x in enumerate(extra) if x == "--sid"] or [0xD400]
    with open(SNOOP, "wb") as fh:
        fh.write(tune(bases=bases))
    opts = extra if "--sampling" in extra else extra + ["--sampling", "fast"]
    subprocess.run([EXE, "--replay", SNOOP, "--wav", REF] + opts, check=True, capture_output=True)
    fake = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "fake_tr.py"), SNOOP, "--port", PORT,
                             "--once"], stdout=subprocess.PIPE, text=True)
    fake.stdout.readline()
    r = subprocess.run([EXE, "--tcp", "127.0.0.1", "--tcp-port", PORT, "--no-play", "--wav", OUT,
                        "--seconds", "5"] + opts, capture_output=True, text=True, timeout=60)
    fake.wait(10)
    print(r.stdout.strip())
    if r.returncode:
        print(r.stderr.strip())
        sys.exit("FAIL")
    x, y = wav(OUT), wav(REF)
    seg = x[48000:96000]
    c = np.correlate(y[: 48000 * 6], seg, "valid")
    k = int(np.argmax(c))
    a = y[k:k + 48000]
    snr = 10 * np.log10(np.sum(a ** 2) / np.sum((seg - a) ** 2))
    secs = len(x) / 48000
    print(f"live: {secs:.2f} s recorded, 1 s window matches the offline render at {k / 48000:.3f} s, SNR {snr:.1f} dB")
    ok = 4.5 < secs < 7 and snr > 30 and "late 0," in r.stdout
    print("OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
