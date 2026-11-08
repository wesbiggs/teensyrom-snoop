# CLAUDE.md — teensyrom-snoop

Capture C64 bus writes to chosen addresses in the TeensyROM+ phi2 ISR and stream them (COBS + CRC-16
frames, exact cycle times) over USB serial / TCP; `host/trsid.py` plays SID writes through reSIDfp.
Read `README.md` (status table, decisions) and `docs/PROTOCOL.md` first; `docs/USB-AUDIO.md` is phase 2 (emulator on the Teensy, USB audio dongle on the host port).

## Rules
* `core/` is C99 and must stay C++-clean (the firmware compiles it as C++). Every function is `SNOOP_COLD`
  (FLASHMEM on the Teensy: code defaults to RAM1 there, which is scarce). Only `snoop_isr_match` /
  `snoop_isr_store` (inline) and `SnoopTap` run in the ISR; `snoop_hot_t` must stay in DTCM.
  Keep `SNOOP_BARRIER()` around the ring's head/tail updates (GCC moved the entry stores past head without it).
* Keep the ISR hook a single byte-load test. After any change near it, check the `isrPHI2` prologue:
  `tools/armcheck/build.sh` prints the saved registers (stock: `{r3-r9, sl, fp, lr}`), CI prints the
  Teensyduino build's disassembly. In teensyrom-vdrive one extra test in the snoop path made GCC save six
  registers on every cycle and corrupted bus reads.
* After changing `core/` or `firmware/`, run `tools/make_tr_patch.py <TeensyROM checkout at firmware/UPSTREAM>`
  and commit the patch; CI fails when it is stale. Keep the hunks off the lines teensyrom-vdrive's patch edits
  (both must apply in either order; check with `git apply` on a `git archive` tree).
* **Version stamp:** `tools/stamp_version.py <tree>` (after the patches) makes TRVersion `<base>-sn<firmware/SNREV>`,
  shown at the top of the TR menu. **Bump `firmware/SNREV` on any commit that changes firmware behaviour** (`core/`,
  `firmware/`, the patch edits). Keep the stamp out of the patch: teensyrom-vdrive's patch edits the same line.
* Protocol changes: bump `SNOOP_PROTO_VERSION` and `PROTO_VERSION` in `host/snoopproto.py`, update docs/PROTOCOL.md
  and both the C encoder and the Python encoder (tests use the Python one to build streams).
* No Commodore ROM bytes and no SID tunes in the repo: tests read the TeensyROM checkout's bundled tunes (`TEENSYROM=`).

## Commands
* `make test` — core via ctypes, C++ syntax check, firmware glue against host stubs, the player, real tunes end to end
  (the last two skip without `pip install pyresidfp numpy py65` / a TeensyROM checkout, default `../3p/TeensyROM`).
* `make demo` — WAVs of bundled tunes in `build/demo/` (capture -> wire -> reSIDfp, no hardware).
* `make build/libsnoop.so` before running `tests/test_core.py` alone (ctypes loads it).
* `tools/armcheck/build.sh <3p> <TeensyROM> <name> [patches]` — ARM compile + isrPHI2 saved registers.

## Status
Builds with Teensyduino 1.61 in CI (the `firmware` job uploads the `.hex`; pjrc.com is not reachable from the cloud
container, so build there or with `tools/armcheck`). Upstream base `6518b966`, the same as teensyrom-vdrive 0.9.0.45.
**0.9-sn1 works on hardware** (owner, TCP and USB serial), but USB serial glitched cartridge games: every USB
transfer in the Teensy core runs with all interrupts off. 0.9-sn2 batches USB sends (20 ms): Ms. Pac-Man still
misbehaves on USB (owner), cause not settled. 0.9-sn3: Nagle batching on TCP, late phi2 interrupts counted (STATUS v3,
`trsid.py --isr`) to find out. sn3 on hardware: TCP much improved (Nagle stays); Ms. Pac-Man on USB still crashes, with
no missed cycles and only 12 >250 ns late interrupts in 16 s (8 in the crash second): the USB data path is not clearly
the cause. **Settled (owner): any USB write from the TeensyROM does it, stock firmware too** (pings over USB crash
Ms. Pac-Man; a merely open port does not): `schedule_transfer()` in the Teensy core's `usb.c` runs with all interrupts
off. `tools/teensy_core_usb_basepri.py` (BASEPRI 32 there, the phi2 interrupt at 16 keeps running) **on hardware (owner,
2026-11-03): no interrupt >250 ns late on USB any more** (only more small lateness: ~800/s against TCP's 300-400), and
Ms. Pac-Man lasts a minute to several minutes instead of crashing at once, **but still crashes**: not explained by late
interrupts. `--send-ms 5` and `200` changed neither the counts nor clearly the crash time. Next: ~10-minute runs with no
capture and over TCP (TCP runs so far were ~16 s); if only USB crashes, suspect ground/power from the Mac (powered hub or
USB isolator). Keep USB writes few and large.
Host only (no SNREV change): `trsid.py` live latency is automatic by default (`LivePlayer`: 99th percentile of frame
arrival delay over 6 s + 10 ms, 15-500 ms, slewed), FIFO 30 ms, `sounddevice` low latency. The old fixed 250 ms on TCP
was the lag the owner heard. Not yet on hardware.
0.9-sn4: snoop's own TCP data port 2113 (`SnoopDataPortService`; the clients try it first, then 2112); host-tested, not
on hardware. 0.9-sn5: that port is 6581 instead of 2113 (sn4 firmware: the clients fall back to 2112). NativeEthernet's `EthernetServer::available()` blocks up to 10 s for a new client's first bytes.
0.9-sn6 (phase 2 step 1): the main-loop CPU probe (`snoop_cpu_probe`, 1 ms every 100 ms while a capture runs, STATUS v4,
`trsid.py --isr` prints `CPU free`). It touches no ISR code. On hardware (owner, menu idle, TCP): 60.9% free, longest
interruption 41 us with no late phi2 (a lower-priority interrupt); Ms. Pac-Man running: 43.3% free. At 816 MHz that is
~497 / ~353 M cycles/s. Step 1 done.
Host only: the live player's drift correction drops/repeats up to 4 samples per callback (was 1): at 1 it removed
3.9 ms/s while the automatic latency came down at 10 ms/s, and the excess sat in the FIFO as unreported delay (the
owner's run dropped on nearly every callback for 50 s). Rerun (owner): latency steady at 60 ms, yet dropped 20888 / repeated 1974 /
8 underruns in 58 s. Not a clock: `trsid.py` now prints both at the end, and they were fine (TeensyROM +117 ppm,
sound card 47946/s; HELLO says cpu 816 MHz, phi2 NTSC). The cause: HELLO's emulator reset (1.2 s warm-up, ~0.36 s
on the Mac) ran before C64 time 0 was marked, and that backlog was rendered into the 30 ms FIFO at once: ~360 ms of
unreported delay, dropped over the next ~25 s. That fix was not enough on hardware (15562 drops in 33 s): trsid.py
also built the renderer (the same warm-up) after the start command, so HELLO waited in `rest`. Now trsid.py builds
the player and starts the sound card before the start command, and a forward jump of "now" by more than 4 FIFOs while
playing is skipped, not queued. `--isr` lines show the FIFO and the drops/repeats of each second. On hardware (owner,
menu idle, 23 s): dropped 966, repeated 777, 1 underrun, FIFO 34 ms at the end: fixed; what is left is the "now"
estimate's jitter with one STATUS a second. The "sound card took N samples/s" line is too coarse (47881/s would have
meant ~2700 net drops; there were ~190): it counts from the first callback, during PortAudio's start-up fill. With Ms. Pac-Man (TCP, 42 s): 113 underruns, latency raised to
103 ms: the automatic latency rose at 50 ms/s and repeats add only 15.6 ms/s, so the FIFO ran dry (reproduced in a
simulation: 16 underruns, 0 at 12 ms/s). AUTO_UP is now 12 ms/s. Not yet on hardware.
`tools/armcheck` unpacks under this repo's `build/`: the tree needs a repository of its own or `git apply` skips every
file silently (fixed in sn3; it now fails when the patches change nothing). Its results before that were for stock trees.
