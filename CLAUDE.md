# CLAUDE.md — teensyrom-snoop

Capture C64 bus writes to chosen addresses in the TeensyROM+ phi2 ISR and stream them (COBS + CRC-16
frames, cycle times) over USB serial / TCP. Read `README.md` and `docs/PROTOCOL.md` first.

## Rules
* `core/` is C99 and must stay C++-clean (the firmware compiles it as C++). Every function is `SNOOP_COLD`
  (FLASHMEM on the Teensy: code defaults to RAM1 there, which is scarce). Only `snoop_isr_match` /
  `snoop_isr_store` (inline) and `SnoopTap` run in the ISR; `snoop_hot_t` must stay in DTCM.
* Keep the ISR hook a single byte-load test. After any change near it, check the `isrPHI2` prologue in
  the disassembly (CI prints it): in teensyrom-vdrive one extra test in the snoop path made GCC save six
  registers on every cycle and corrupted bus reads.
* After changing `core/` or `firmware/`, run `tools/make_tr_patch.py <pristine TeensyROM at firmware/UPSTREAM>`
  and commit the patch; CI fails when it is stale. Keep the hunks off the lines teensyrom-vdrive's patch edits
  (both must apply in either order).
* Protocol changes: bump `SNOOP_PROTO_VERSION` and `PROTO_VERSION` in `host/snoopproto.py`, update docs/PROTOCOL.md.
* No Commodore ROM bytes in the repo.

## Commands
* `make test` — core via ctypes, C++ syntax check, firmware glue against host stubs. Needs cc/c++, Python 3.
* `make build/libsnoop.so` before running `tests/test_core.py` alone (ctypes loads it).

## Status
Host-tested only. No firmware build here (pjrc.com is not reachable from the cloud container; CI builds it),
no hardware run yet.
