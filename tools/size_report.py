#!/usr/bin/env python3
"""Memory cost of the snoop patch, from TeensyROM+ builds of the same upstream commit (.github/workflows/size.yml).

  tools/size_report.py label=dir [label=dir ...]      the first is the baseline (stock)

Each dir holds, for the main image: main.log (arduino-cli's output with Teensyduino's memory report),
main.syms (`nm -S -C` of Teensy.ino.elf) and optionally main.size (`size -A`) and isr.s (isrPHI2's
disassembly); minimal.log for MinimalBoot.  Prints Markdown: the memory report of each build with its
change from the baseline, RAM1 split into ITCM (code) and DTCM (data + stack), and the symbols each
build adds, by memory and by component (the tap, the USB audio path, reSID, other).
"""
import os
import re
import sys

# Teensy 4.1 memory map
REGIONS = [  # (name, first, end)
    ("ITCM", 0x00000000, 0x00080000),
    ("DTCM", 0x20000000, 0x20080000),
    ("RAM2", 0x20200000, 0x20280000),
    ("flash", 0x60000000, 0x61000000),
]


def region(addr):
    for name, a, b in REGIONS:
        if a <= addr < b:
            return name
    return None


def component(name):
    n = name
    if n.startswith("reSID::") or "reSID" in n:
        return "reSID"
    if re.search(r"USBAudioOut|isoaudio|usbhost_iso|iso_out|SnoopAudio|SnoopTone|SnoopDongle|SnoopSid|siTD|sitd", n):
        return "audio"
    if re.search(r"snoop|Snoop", n):
        return "tap"
    return "other"


def mem_report(path):
    """Teensyduino's 'Memory Usage on Teensy 4.1' as {'FLASH': {'code': n, ...}, 'RAM1': ..., 'RAM2': ...}"""
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path, errors="replace"):
        m = re.match(r"\s*(FLASH|RAM1|RAM2):\s*(.*)", line)
        if not m:
            continue
        out[m.group(1)] = {k.strip(): int(v) for k, v in re.findall(r"([A-Za-z/ ]+?):\s*(\d+)", m.group(2))}
    return out


def symbols(path):
    """{(name, region): size} from `nm -S -C` (symbols with a size only)"""
    syms = {}
    if not os.path.exists(path):
        return syms
    for line in open(path, errors="replace"):
        parts = line.rstrip("\n").split(" ", 3)
        if len(parts) != 4:
            continue
        try:
            addr, size = int(parts[0], 16), int(parts[1], 16)
        except ValueError:
            continue
        r = region(addr)
        if r is None or parts[2] in "Uuw":
            continue
        key = (parts[3], r)
        syms[key] = syms.get(key, 0) + size
    return syms


def saves(path):
    if not os.path.exists(path):
        return "?"
    text = open(path, errors="replace").read()
    m = re.search(r"<(?:_Z7isrPHI2v|isrPHI2\(\))>:(.*?)(?:\n\n|\Z)", text, re.S)
    if not m:
        return "?"
    for line in m.group(1).splitlines():
        if re.search(r"\b(push|stmdb)\b", line):
            return line.split("\t")[-1].strip() if "\t" in line else line.strip()
    return "none"


def fmt(n, base=None):
    s = f"{n:,}"
    if base is not None and n != base:
        s += f" ({n - base:+,})"
    return s


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    builds = []
    for arg in sys.argv[1:]:
        label, _, d = arg.partition("=")
        builds.append((label, d, mem_report(os.path.join(d, "main.log")), symbols(os.path.join(d, "main.syms")),
                       mem_report(os.path.join(d, "minimal.log")), saves(os.path.join(d, "isr.s"))))
    base = builds[0]

    print("## Main image: Teensyduino's memory report\n")
    rows = [("FLASH", "code"), ("FLASH", "data"), ("FLASH", "headers"), ("FLASH", "free for files"),
            ("RAM1", "variables"), ("RAM1", "code"), ("RAM1", "padding"), ("RAM1", "free for local variables"),
            ("RAM2", "variables"), ("RAM2", "free for malloc/new")]
    print("| | " + " | ".join(b[0] for b in builds) + " |")
    print("|---|" + "---|" * len(builds))
    for sec, key in rows:
        vals = [b[2].get(sec, {}).get(key) for b in builds]
        bv = vals[0]
        print(f"| {sec} {key} | " + " | ".join("?" if v is None else fmt(v, bv) for v in vals) + " |")

    def derived(m):
        r1 = m.get("RAM1", {})
        fl = m.get("FLASH", {})
        if not r1 or not fl:
            return None
        itcm = r1.get("code", 0) + r1.get("padding", 0)
        return {
            "ITCM (RAM1 code + padding to 32 KB banks)": itcm,
            "ITCM banks (32 KB)": itcm // 32768,
            "DTCM variables (RAM1 variables)": r1.get("variables", 0),
            "DTCM left for the stack (free for local variables)": r1.get("free for local variables", 0),
            "flash used (code + data + headers)": fl.get("code", 0) + fl.get("data", 0) + fl.get("headers", 0),
        }

    ds = [derived(b[2]) for b in builds]
    if all(ds):
        print("\n## RAM1 split (ITCM / DTCM) and flash\n")
        print("| | " + " | ".join(b[0] for b in builds) + " |")
        print("|---|" + "---|" * len(builds))
        for k in ds[0]:
            print(f"| {k} | " + " | ".join(fmt(d[k], ds[0][k]) for d in ds) + " |")

    if any(b[4] for b in builds):
        print("\n## MinimalBoot (the snoop patch does not build into it)\n")
        print("| | " + " | ".join(b[0] for b in builds) + " |")
        print("|---|" + "---|" * len(builds))
        for sec, key in rows:
            vals = [b[4].get(sec, {}).get(key) for b in builds]
            print(f"| {sec} {key} | " + " | ".join("?" if v is None else fmt(v, vals[0]) for v in vals) + " |")

    print("\n## isrPHI2 prologue (registers saved on every phi2 interrupt)\n")
    for b in builds:
        print(f"* {b[0]}: `{b[5]}`")

    print("\n## What each build adds over the one before it, by symbol (nm -S)\n")
    print("Symbol sizes by address: ITCM is code run from RAM1, DTCM is RAM1 data, RAM2 is DMAMEM, flash is FLASHMEM code"
          " and const data.  Static sizes only: reSID's SID object is allocated on the heap (RAM2) when a capture starts."
          "  'other' is upstream code that changed size (inlining, the command switch) or library code pulled in.\n")
    regions = [r[0] for r in REGIONS]
    for prev, cur in zip(builds, builds[1:]):
        print(f"### {prev[0]} -> {cur[0]}\n")
        delta = {}
        keys = set(prev[3]) | set(cur[3])
        detail = []
        for k in keys:
            d = cur[3].get(k, 0) - prev[3].get(k, 0)
            if d:
                c = component(k[0])
                delta[(c, k[1])] = delta.get((c, k[1]), 0) + d
                detail.append((d, k[1], c, k[0]))
        comps = ["tap", "audio", "reSID", "other"]
        print("| component | " + " | ".join(regions) + " |")
        print("|---|" + "---|" * len(regions))
        for c in comps:
            if any(delta.get((c, r)) for r in regions):
                print(f"| {c} | " + " | ".join(f"{delta.get((c, r), 0):+,}" for r in regions) + " |")
        print("| **total** | " + " | ".join(f"{sum(delta.get((c, r), 0) for c in comps):+,}" for r in regions) + " |")
        print("\n<details><summary>symbols (largest changes first)</summary>\n")
        print("| bytes | memory | component | symbol |")
        print("|---|---|---|---|")
        for d, r, c, n in sorted(detail, key=lambda x: -abs(x[0]))[:80]:
            print(f"| {d:+,} | {r} | {c} | `{n.replace('|', '/')}` |")
        if len(detail) > 80:
            print(f"| | | | ... {len(detail) - 80} more |")
        print("\n</details>\n")


if __name__ == "__main__":
    main()
