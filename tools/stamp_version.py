#!/usr/bin/env python3
"""Stamp the snoop build into a patched TeensyROM tree's version: TRVersion "0.9" -> "0.9-sn1".

  tools/stamp_version.py /path/to/TeensyROM

Run after `git apply firmware/teensyrom-snoop.patch` (and the teensyrom-vdrive patch, if used).
The number after "sn" is firmware/SNREV. Whatever version is there keeps its stamp, so with
vdrive it becomes "0.9.0.45-sn1". A "-snN" from an earlier run is replaced, so running it twice
is harmless. A tree with reSID in src/reSID/ (tools/fetch_resid.py, run before this) gets "-snNr": its
image includes the GPL SID emulator. The menu shows the result at the top ("TeensyROM+ v0.9-sn1"), and the hex is named
after it (TeensyROM+_0.9-sn1_full.hex).

It is a separate step, not part of the patch, because teensyrom-vdrive's patch rewrites the same
line: a hunk there would stop the two patches applying together.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFS = "Source/Teensy/MinimalBoot/Common/Common_Defs.h"
LINE = re.compile(r'^(#define TRVersion\s+")([^"]+)(".*)$', re.M)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    rev = open(os.path.join(ROOT, "firmware", "SNREV")).read().strip()
    if not rev.isdigit():
        sys.exit(f"firmware/SNREV must be a number, not {rev!r}")
    path = os.path.join(sys.argv[1], DEFS)
    with open(path, newline="") as fh:
        text = fh.read()
    m = LINE.search(text)
    if not m:
        sys.exit(f"{DEFS}: no #define TRVersion line")
    if "FeatWriteSnoop" not in open(os.path.join(sys.argv[1], "Source/Teensy/TeensyROM.h")).read():
        sys.exit("the snoop patch is not applied to this tree (no FeatWriteSnoop in TeensyROM.h)")
    base = re.sub(r"-sn\d+r?$", "", m.group(2))
    # "r": the tree has reSID (tools/fetch_resid.py), so the image is GPL-2.0-or-later as a whole
    resid = os.path.isfile(os.path.join(sys.argv[1], "Source/Teensy/src/reSID/sid.h"))
    version = f"{base}-sn{rev}{'r' if resid else ''}"
    text = text[:m.start()] + m.group(1) + version + m.group(3) + text[m.end():]
    with open(path, "w", newline="") as fh:
        fh.write(text)
    print(f"TRVersion {m.group(2)} -> {version}")


if __name__ == "__main__":
    main()
