#!/usr/bin/env python3
"""Fetch reSID 0.16 (Dag Lem, GPL-2.0-or-later) and adapt it for the Teensy, for phase 2 (docs/USB-AUDIO.md).

  tools/fetch_resid.py --tree /path/to/TeensyROM    into Source/Teensy/src/reSID/ of a patched tree
  tools/fetch_resid.py --dest build/resid           anywhere (the host tests use build/resid)

reSID is GPL: it is never committed to this repository, and the snoop patch builds without it. A tree with
src/reSID/ builds the SID emulator into WriteSnoop.ino (it checks with __has_include), and such a firmware
image is GPL-2.0-or-later as a whole: whoever distributes it must offer its source (this repository, the
TeensyROM tree, and reSID at the commit below). tools/stamp_version.py marks the version "-snNr".

The source is github.com/daglem/reSID at the reSID 0.16 release (2004), pinned by commit. The edits, each
checked against reSID's text:

  *.cc -> *.cpp           the Arduino build compiles .cpp, not .cc
  version.cc dropped      it needs autoconf's VERSION, and nothing uses it
  wave tables             reg8 (unsigned int) arrays, 8 x 16 KB of writable data, which on the Teensy 4 would
                          land in RAM1: now const unsigned char (the values are 8-bit), 8 x 4 KB, in flash
                          (PROGMEM) on the Teensy, and the wave pointers const to match
  no resampling           set_sampling_parameters refuses SAMPLE_RESAMPLE_*: its FIR setup links log() and
                          friends, whose tables TeensyROM's build refuses (heavy-libc.mjs)
  namespace reSID         around every file's contents: the TeensyROM sketch has globals with reSID's names
                          (`stcVoiceInfo Voice[3]` hid class Voice and broke the build)
"""
import argparse
import hashlib
import io
import os
import shutil
import sys
import tarfile
import urllib.request

REPO = "daglem/reSID"
COMMIT = "03e31bcbcd59b754a5e5068110316b8731412a47"   # "reSID 0.16", 2004-06-11
URL = f"https://codeload.github.com/{REPO}/tar.gz/{COMMIT}"
KEEP_EXT = (".cc", ".h")
KEEP_DOCS = ("COPYING", "AUTHORS", "README", "NEWS")
TABLES = [f"wave{m}_{w}" for m in ("6581", "8580") for w in ("_ST", "P_T", "PS_", "PST")]


def edit(files, name, old, new, count=1):
    s = files[name]
    n = s.count(old)
    if n != count:
        sys.exit(f"{name}: expected {count} match(es) of {old!r}, found {n} (wrong reSID?)")
    files[name] = s.replace(old, new)


def fetch():
    with urllib.request.urlopen(URL, timeout=60) as r:
        data = r.read()
    files = {}
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tf:
        for m in tf.getmembers():
            parts = m.name.split("/")
            if not m.isfile() or len(parts) != 3 or parts[1] != "src":
                continue
            base = parts[2]
            if base.endswith(KEEP_EXT) or base in KEEP_DOCS:
                files[base] = tf.extractfile(m).read().decode("latin-1")
    for need in ("sid.cc", "sid.h", "siddefs.h", "wave.h", "COPYING") + tuple(t + ".cc" for t in TABLES):
        if need not in files:
            sys.exit(f"{URL}: no src/{need}")
    return files


def adapt(files):
    del files["version.cc"]
    # Tables: 8-bit constants, in flash on the Teensy
    edit(files, "siddefs.h", "#define __SIDDEFS_H__\n", """#define __SIDDEFS_H__

// teensyrom-snoop (tools/fetch_resid.py): read-only tables stay in flash on the Teensy (const data is
// otherwise copied to RAM1 there).
#if defined(__IMXRT1062__)
#include <avr/pgmspace.h>
#define RESID_ROTABLE PROGMEM
#else
#define RESID_ROTABLE
#endif // RESID_ROTABLE
""")
    for t in TABLES:
        edit(files, "wave.h", f"  static reg8 {t}[];", f"  static const unsigned char {t}[];")
        edit(files, f"{t}.cc", f"reg8 WaveformGenerator::{t}[] =",
             f"const unsigned char WaveformGenerator::{t}[] RESID_ROTABLE =")
    for p in ("wave__ST", "wave_P_T", "wave_PS_", "wave_PST"):
        edit(files, "wave.h", f"  reg8* {p};", f"  const unsigned char* {p};")
    # No resampling: its FIR setup links log(), log10(), sin() and sqrt(), whose tables TeensyROM's build refuses
    # in RAM1 (tools/lib/heavy-libc.mjs: __log_data).  SAMPLE_FAST and SAMPLE_INTERPOLATE never use it.
    s = files["sid.cc"]
    head = s.index("SID::set_sampling_parameters(")
    body = s.index("{\n", head) + 2
    fir = s.index("  const double pi = 3.1415926535897932385;", body)
    end = s.index("\n  return true;\n}\n", fir)
    files["sid.cc"] = (s[:body]
                       + "  // teensyrom-snoop (tools/fetch_resid.py): no resampling in this build, see below.\n"
                       + "  if (method == SAMPLE_RESAMPLE_INTERPOLATE || method == SAMPLE_RESAMPLE_FAST) {\n"
                       + "    return false;\n  }\n\n"
                       + s[body:fir]
                       + "  // teensyrom-snoop (tools/fetch_resid.py): the FIR table setup for resampling is cut here.\n"
                       + "  return false;\n}\n"
                       + s[end + len("\n  return true;\n}\n"):])
    for name in list(files):
        if name.endswith((".h", ".cc")):
            files[name] = in_namespace(name, files[name])
    out = {}
    for name, text in files.items():
        out[name[:-3] + ".cpp" if name.endswith(".cc") else name] = text
    return out


def in_namespace(name, text):
    """Everything after the includes (or the include guard) goes into namespace reSID: the TeensyROM sketch has
    globals of its own with reSID's names (`stcVoiceInfo Voice[3]` hid class Voice)."""
    lines = text.split("\n")
    opener = [i for i, l in enumerate(lines)
              if l.startswith("#include") or l.startswith("#endif // RESID_ROTABLE")
              or (name.endswith(".h") and l.startswith("#define __") and l.rstrip().endswith("_H__"))]
    if not opener:
        sys.exit(f"{name}: no include or include guard to open the namespace after")
    at = opener[-1] + 1
    if name.endswith(".h"):
        ends = [i for i, l in enumerate(lines) if l.startswith("#endif")]
        if not ends or ends[-1] < at:
            sys.exit(f"{name}: no closing #endif after the includes")
        close = ends[-1]
    else:
        close = len(lines)
    return "\n".join(lines[:at] + ["namespace reSID {"] + lines[at:close] + ["} // namespace reSID"] + lines[close:])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--tree", help="a TeensyROM checkout with the snoop patch applied")
    g.add_argument("--dest", help="a directory to write the sources to")
    a = ap.parse_args()
    if a.tree:
        if "FeatWriteSnoop" not in open(os.path.join(a.tree, "Source/Teensy/TeensyROM.h")).read():
            sys.exit("apply the snoop patch first (no FeatWriteSnoop in TeensyROM.h)")
        dest = os.path.join(a.tree, "Source/Teensy/src/reSID")
    else:
        dest = a.dest
    files = adapt(fetch())
    if os.path.isdir(dest):
        shutil.rmtree(dest)
    os.makedirs(dest)
    for name, text in sorted(files.items()):
        with open(os.path.join(dest, name), "w", newline="\n") as fh:
            fh.write(text)
    digest = hashlib.sha256("".join(files[k] for k in sorted(files)).encode()).hexdigest()[:16]
    print(f"reSID 0.16 ({REPO}@{COMMIT[:12]}, GPL-2.0-or-later) -> {dest}: {len(files)} files, content {digest}")


if __name__ == "__main__":
    main()
