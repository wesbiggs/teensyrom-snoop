#!/usr/bin/env python3
"""Real-time check of the live path: tools/fake_tr.py streams a captured tune at the C64's pace,
host/trsid.py --tcp --no-play --wav records it, and the recording is compared with the offline
render of the same capture.  Timing-dependent (a busy machine makes late writes), so it is
`make test-live`, not part of `make test`.  Needs `make demo` first (build/demo/Popcorn.snoop)."""
import os
import subprocess
import sys
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SNOOP = os.path.join(ROOT, "build", "demo", "Popcorn.snoop")
REF = os.path.join(ROOT, "build", "demo", "Popcorn.wav")
OUT = os.path.join(ROOT, "build", "live.wav")
PORT = "23459"


def wav(path, frames=None):
    with wave.open(path) as w:
        return np.frombuffer(w.readframes(frames or w.getnframes()), dtype="<i2").astype(float)


fake = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools", "fake_tr.py"), SNOOP, "--port", PORT, "--once"],
                        stdout=subprocess.PIPE, text=True)
fake.stdout.readline()
r = subprocess.run([sys.executable, os.path.join(ROOT, "host", "trsid.py"), "--tcp", "127.0.0.1", "--tcp-port", PORT,
                    "--no-play", "--wav", OUT, "--seconds", "5"], capture_output=True, text=True, timeout=60)
fake.wait(10)
print(r.stdout.strip())
x = wav(OUT)
y = wav(REF, 48000 * 8)
seg = x[48000:96000]
c = np.correlate(y[: 48000 * 4], seg, "valid")
k = int(np.argmax(c))
a = y[k:k + 48000]
snr = 10 * np.log10(np.sum(a ** 2) / np.sum((seg - a) ** 2))
secs = len(x) / 48000
print(f"live: {secs:.2f} s recorded, 1 s window matches the offline render at {k / 48000:.3f} s, SNR {snr:.1f} dB")
ok = 4.5 < secs < 7 and snr > 30 and "underruns 0" in r.stdout
print("OK" if ok else "FAIL")
sys.exit(0 if ok else 1)
