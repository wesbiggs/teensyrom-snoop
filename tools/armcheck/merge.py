"""Rough stand-in for the Arduino sketch preprocessor: Teensy.ino first, then the other .ino
files alphabetically, with forward declarations of every top-level function in the .ino files."""
import os, re, sys
d = sys.argv[1]
inos = ["Teensy.ino"] + sorted(f for f in os.listdir(d) if f.endswith(".ino") and f != "Teensy.ino")
protos = []
pat = re.compile(r'^((?:FLASHMEM |FASTRUN |static |inline )*[A-Za-z_][\w<>:]*[\s\*&]+[A-Za-z_]\w*\s*\([^;{)]*\))\s*(?://.*)?$')
body = []
for f in inos:
    lines = open(os.path.join(d, f), encoding="latin-1").read().split("\n")
    body.append(f'#line 1 "{f}"')
    depth = 0
    for i, l in enumerate(lines):
        if depth == 0:
            m = pat.match(l)
            nxt = lines[i + 1].strip() if i + 1 < len(lines) else ""
            if m and (l.rstrip().endswith("{") or nxt.startswith("{")) and not l.lstrip().startswith(("else", "if", "for", "while", "switch", "return")):
                sig = m.group(1)
                sig = re.sub(r'\s*=\s*[^,)]+', '', sig)    # default args stay on the definition only
                if " main(" not in sig and "setup(" not in sig and "loop(" not in sig:
                    protos.append((f, sig))
        depth += l.count("{") - l.count("}")
        body.append(l)
# prototypes after Teensy.ino's includes: just before the first function of Teensy.ino would be ideal;
# put them right after the last top-level #include of Teensy.ino
t = body[: body.index('#line 1 "' + inos[1] + '"')]
last_inc = max(i for i, l in enumerate(t) if l.startswith("#include"))
out = ["#include <Arduino.h>"] + body[: last_inc + 1] + ["// ---- prototypes"] + [s + ";" for _, s in protos] + body[last_inc + 1:]
open(sys.argv[2], "w").write("\n".join(out))
print(len(protos), "prototypes")
