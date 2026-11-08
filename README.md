# teensyrom-snoop

Capture C64 bus writes to chosen addresses with a TeensyROM+ and stream them to a computer over USB
serial or TCP, each with its exact C64 cycle.

The first use is the SID: watch `$D400-$D41C`, and `host/trsid.py` plays the register writes
through reSIDfp. You hear what the C64 is playing on a C64 with no SID chip at all, or record it to WAV.
Nothing in the firmware is SID-specific: any 1-16 address ranges (up to 16 distinct 256-byte
pages) can be watched, for example a second SID at `$D500`, the VIC, or a cartridge's IO1 registers.

## Status

| What | State |
|---|---|
| Capture core, wire format, decoder | host-tested (`make test`) |
| Firmware glue (`WriteSnoop.h/.ino`) | host-tested against stubs; **compiles for the Teensy 4.1** (ARM GCC 13 from GitHub sources, `tools/armcheck`) on its own and together with teensyrom-vdrive |
| `isrPHI2` cost | same registers saved as stock (checked in the ARM disassembly); a read cycle pays a compare and branch, an unwatched write ~7 instructions |
| Patch | applies to upstream `6518b966` alone and with teensyrom-vdrive 0.9.0.45 (its HEAD), either order; stock, snoop, vdrive and vdrive + snoop compile for the M7 (`tools/armcheck`) with the same `isrPHI2` saves, and the on-time path is as long as stock's. (Before sn3, `tools/armcheck` silently built the unpatched tree: its earlier results for patched trees were wrong; the CI builds were real.) |
| Player (`trsid.py`) | renders to WAV; the live path (TCP, real time, `tools/fake_tr.py` as the TeensyROM) matches the offline render (62 dB, no underruns, `make test-live`); not tried on a sound card (the container has none) |
| End to end with real tunes | three of the TeensyROM's bundled SIDs, played in a 6502 emulator, through the core, the wire and reSIDfp: every write at its exact cycle, audio equal to a direct render |
| Teensyduino build (GCC 11.3, link, size) | **builds** in CI (`TeensyROM+_0.9_full.hex`; main image RAM1 free for locals 75004, padding 26888). `isrPHI2` saves `{r3-r9, sl, fp, lr}` (already every callee-saved register, so the hook cannot add saves); write cycles branch out to the page-map test; `SnoopTap` stores the entry before `head` |
| Hardware (owner, 2026-11-03, 0.9-sn1) | **works** over TCP and USB serial. TCP over a multi-hop WiFi link gave many late writes at the old 30 ms latency (now 250 ms by default on TCP). USB serial gave ~0 late writes but glitched or crashed cartridge games (Ms. Pac-Man): every USB transfer runs with interrupts off. 0.9-sn2 batches USB sends (20 ms): **Ms. Pac-Man still misbehaves on USB** (owner). 0.9-sn3 on hardware (owner, 2026-11-03): **TCP much better** (late writes in a 30-45 s clip down from thousands, 8000 on one run, to 538, no audible drops). Ms. Pac-Man on USB still crashes; `--isr` over 16 s showed no missed cycles (`>1us` 0), 12 interrupts >250 ns late (8 of them in the second it crashed) and ~265/s small (<250 ns) lateness, ~1500/s during the launch. Next: the same over TCP, the USB cable with no capture, and a capture of an unwritten address |

## How it works

```
 C64 write ──> isrPHI2 (every phi2 cycle)
                 if (!R_Wn && SnoopHot.page[addr>>8]) SnoopTap(addr)     <- one byte load on a write
                     bit test, wait for the data bus, ring <- {CPU stamp, addr, data}
 loop()  ──> SnoopService()
                 ring -> C64-cycle times -> blocks {dt, addr, len, bytes} -> frames (COBS + CRC-16)
                 -> the channel that started the capture, never more than availableForWrite()
```

* `core/`: portable C99, which the firmware also compiles as C++. It holds the two-level address filter
  (a 256-byte page map plus one 256-bit map per watched page), the ISR ring, the stamp-to-cycle
  conversion and the framing. `docs/PROTOCOL.md` has the wire format.
* `firmware/`: the TeensyROM glue and `teensyrom-snoop.patch`, which adds it to a TeensyROM tree.
  * `WriteSnoop.h` is the ISR half, in DTCM.
  * `WriteSnoop.ino` holds the command and the loop service.
* `host/`:
  * `snoopproto.py`: the decoder and encoder, with no dependencies.
  * `trsnoop.py`: prints or records a stream.
  * `sidplay.py` + `trsid.py`: the reSIDfp player.
* `tools/`:
  * `make_tr_patch.py` regenerates the patch.
  * `sidtrace.py` turns a PSID into the write stream the TeensyROM would capture, with no hardware.
  * `armcheck/` is the ARM compile check.

Cost on the bus: every write cycle loads one byte from the page map, and with nothing watched that is
all it costs. A watched write waits until the data bus is valid (`nS_DataSetup`, 220 ns), as every IO1/IO2 write
handler already does, then stores 8 bytes. The tap runs after the existing snoops, so Kernal Replace's
timing is unchanged. Memory: `SnoopHot` 4.9 KB DTCM, `SnoopTap` 180 bytes RAM1 code, `SnoopState`
8.9 KB RAM2, the rest in flash (from the ARM object's sections).

## Using it

Build and flash the firmware with the patch (see below), then on the computer:

```
pip install pyserial pyresidfp numpy sounddevice   # serial, emulation, sound card

host/trsid.py --tcp 192.168.1.50                   # hear the SID live
host/trsid.py --port /dev/ttyACM0 --wav out.wav    # hear it and record it
host/trsid.py --tcp 192.168.1.50 --sid D400 --sid D500 --stereo --model 8580

host/trsnoop.py --port /dev/ttyACM0 --range D400-D41C          # print the writes
host/trsnoop.py --tcp 192.168.1.50 --range D020-D021 --raw x.snoop
host/trsid.py --replay x.snoop --wav x.wav                      # a recording to WAV
```

`trsnoop.py` prints one line per block (C64 cycles, seconds, address, bytes) plus HELLO/STATUS/BYE lines.

**`--gap`** (trsnoop; default 0): writes to consecutive addresses within N cycles of each other share
one block and its time. Measured on real tunes (`tests/test_e2e.py`, numbers from three tunes):

| gap | wire bytes/s | audio vs exact (SNR) |
|---|---|---|
| 0 | 2.6-6.7 K | equal (reSIDfp's own dither, >55 dB) |
| 8 | 1.9-5.0 K | 36-65 dB |
| 32 | 1.6-3.6 K | 14-44 dB |

A SID player's register writes are 4-10 cycles apart, so grouping moves them by up to ~30 cycles,
and the waveforms show it. Bandwidth is negligible either way, so gap 0 is the default, and `trsid.py`
always uses it. A non-zero gap suits memory or screen writes, where only the order matters.

While a capture streams on the USB serial port, other TeensyROM commands on that port still work, but
their answers are mixed into the stream (the decoder skips them). Send them over TCP, or stop the capture first.

Live play stays a little behind the C64, so that writes arriving over the link are in hand before their time is
played. **By default the delay is automatic:** the player measures how late each frame arrives (its oldest write
against the estimated C64 "now"), and keeps the delay at the 99th percentile of the last 6 s plus 10 ms. It never
goes below 15 ms or above 500 ms, rises by at most 50 ms a second and falls by at most 10 ms a second. A fast link
settles near 15 ms; simulated 20 ms jitter settles near 30 ms. `--latency S` fixes it instead. After the delay comes a
30 ms sound-card FIFO (`--buffer`), and the sound card's own buffer (`sounddevice` asked for its low-latency
setting; `trsid.py` prints what it got). Drift between the C64 crystal and the sound card is corrected by dropping or
repeating single samples. A write that arrives after its time was played is applied at once, a few ms late, and
counted ("writes late" at the end). `--isr` lines also show the current delay.

Until this change the default was a fixed 250 ms on TCP (chosen in sn2 for the owner's multi-hop WiFi, where 30 ms
was far too little). That, not the firmware's Nagle batching (at most one round trip, ~10-20 ms there), made TCP
play noticeably late. On TCP the TeensyROM's Nagle wait can be replaced by a fixed interval with `--send-ms 4`
(about 250 small packets a second), which trades some of that round trip for more packets.

**USB serial and cartridge games.** Every USB transfer the Teensy core starts (`schedule_transfer` in its
`usb.c`) runs with all interrupts off, the bus interrupt included, and each small write starts one. At
~500 transfers a second, 0.9-sn1 made cartridge games glitch or crash (Ms. Pac-Man) on USB serial, while TCP
was fine. From 0.9-sn2 the TeensyROM sends at most every 20 ms on USB serial (or once 1 KB is waiting): about
50 transfers a second. `--send-ms N` changes the interval. Longer is safer for the games, and adds latency.
**That did not fix Ms. Pac-Man (owner, sn2)**, so the cause is not settled. From 0.9-sn3 the firmware counts
late phi2 interrupts (any, >250 ns, >1 us), reported in every STATUS and shown live by `trsid.py --isr`. Compare
USB and TCP while the game runs; if USB's `>250ns` count climbs, the USB path is holding the bus interrupt up.

**Found (owner, 2026-11-03): it is any USB write, not snoop.** Stock firmware answering pings over USB 20 times a
second gives late `>250ns` interrupts at once and crashes Ms. Pac-Man. A port that is only held open (`cat`) does
neither, and nor does TCP. Every USB transmit goes through `schedule_transfer()` in the Teensy core's `usb.c`, which
runs with all interrupts off while it polls USB registers. `tools/teensy_core_usb_basepri.py <cores/teensy4>`
(experimental; `--revert` undoes it) masks only priority 32 and lower there (`BASEPRI`), so the phi2 interrupt
(priority 16) keeps running. Patch the installed Teensyduino core, rebuild, rerun the ping test.
**On hardware (owner, 2026-11-03):** with the patch, no interrupt over USB is late by more than 250 ns, and Ms. Pac-Man
lasts a minute to several minutes instead of crashing at once. It still crashes eventually, without any late interrupt to
show for it, so that cause is open. Long runs with no capture and over TCP come next; if only USB crashes, the Mac's
ground or 5 V through USB is the suspect (try a powered hub or a USB isolator).

**Port 6581 (0.9-sn5; 2113 in sn4).** Over TCP the clients connect to snoop's own data port, 6581, and the stream
stays on that connection. The TeensyROM's port 2112 stays free for other tools during a capture (`trls`, `trvdrive`, file
transfers), within the TeensyROM's own limits: most commands answer "Busy!" while a program runs. With firmware
before sn4 the clients fall back to 2112 by themselves (`--tcp-port` picks one).

On TCP (0.9-sn3) the default is Nagle's rule, as in upstream's SwiftLink Tx (PR #80): send at once while
everything sent earlier has been acknowledged, otherwise collect until 1 KB or 20 ms. A fast LAN keeps
near-zero latency, and a slow link gets fewer, larger segments instead of one small one every ~2 ms.
`--send-ms N` replaces the rule with a fixed interval.

`tools/fake_tr.py RECORDING` serves a recording on TCP like a TeensyROM, at the C64's pace, so the
clients can be tried without hardware: `make demo`, then `tools/fake_tr.py build/demo/Popcorn.snoop`
and `host/trsid.py --tcp 127.0.0.1`.

## Building the firmware

```
git clone https://github.com/SensoriumEmbedded/TeensyROM && cd TeensyROM
git checkout $(cat ../teensyrom-snoop/firmware/UPSTREAM)
git apply ../teensyrom-snoop/firmware/teensyrom-snoop.patch
python3 ../teensyrom-snoop/tools/stamp_version.py .   # TRVersion 0.9 -> 0.9-sn4
node tools/build-firmware.mjs --target tr-plus      # arduino-cli, teensy:avr@1.61.0, CRC32@2.0.1
```

The stamp makes the build obvious: the TR menu's top line reads `TeensyROM+ v0.9-sn4`, and the hex is
`TeensyROM+_0.9-sn4_full.hex`. The number is `firmware/SNREV`. The stamp is a separate step because
teensyrom-vdrive's patch rewrites the same `TRVersion` line. To add teensyrom-vdrive as well, apply its patch
too (either order), then stamp: `0.9.0.45` becomes `0.9.0.45-sn4`.

The TeensyROM's host tools read the version with `v[\d.]+` (`tools/lib/trserial.mjs`), so they show the
part before `-sn1` (`v0.9`). The menu, the serial banner and the file name show all of it.

`tools/make_tr_patch.py /path/to/TeensyROM` regenerates the patch after a change to `core/` or
`firmware/`. It works from `git archive HEAD` of that checkout and asserts on the upstream text it edits.

## Tests

`make test`:

* `tests/test_core.py`: the core through ctypes. Covers filtering, block grouping, cycle-exact times with
  stamp jitter and 32-bit wrap, ring and output overflow, and a 20000-write fuzz with text mixed into the stream.
* `tests/fw_host` + `tests/test_fw.py`: the firmware glue against host stubs. Covers serial and TCP runs,
  replacing a running capture, TCP disconnect, DTR drop, refusals and NTSC.
* `tests/test_sidplay.py`: the renderer (pitch, sub-sample timing, stereo, 8580/NTSC) and the live
  player under late, jittery frames and 0.5 % sound-card drift.
* `tests/test_e2e.py`: real tunes end to end (needs `pip install py65 pyresidfp numpy` and a TeensyROM checkout, `TEENSYROM=`).

`make demo` writes WAVs of four bundled tunes to `build/demo/` through the whole host path.
`make test-live` plays one in real time through `fake_tr.py` and `trsid.py --tcp` and compares the result.

`tools/armcheck/build.sh` compiles the main sketch for the Cortex-M7 from GitHub sources and prints
the registers `isrPHI2` saves (see the script for the repositories it needs).

## Decisions and assumptions (made while the owner was away; easy to revisit)

1. **Default gap 0** (exact times), per the table above. The block grouping stays in the format.
2. **Token `0x6453`** (`'d' 'S'`) for the command. It is unused upstream (checked at `136baa83` and `6518b966`) and in teensyrom-vdrive
   (`0x64E3/E4`). It sits in `ProcessCommand`'s first switch, so it works while a program runs.
3. **The stream goes to the channel that started it.** Serial or TCP, one capture at a time. A start
   from another channel takes over, and the old one gets BYE "replaced".
4. **Stops by itself** when its TCP client disconnects, or when a USB serial host that had DTR up drops
   it (closes the port). If the host never raised DTR, the capture runs until stopped.
5. **Times use the nominal clock:** 985248 Hz PAL or 1022727 Hz NTSC, from the TeensyROM's own detection
   (`wRegVid_TOD_Clks`), counted from the start command. The stamps come from the Teensy's cycle
   counter, so each write lands on its exact cycle (tested with ±150-tick ISR-entry jitter).
   The real crystal's ppm offset is not tracked.
6. **Limits:** a 512-entry ring, about 0.5 s of a busy tune if `loop()` stalls (a long file transfer
   command); losses are counted in STATUS. Up to 16 ranges / 16 pages, and an 8 KB output FIFO.
7. **A capture ends when the TeensyROM leaves its main image** (a reboot, a firmware update, or a launch that
   switches to the minimal image): the USB or TCP connection drops with it, and the client reports a closed
   connection. It survives the C64's resets and program launches from the menu.
8. **What is not seen:** the TeensyROM's own DMA writes (an REU into `$D400`, the remote memory
   writes), a C128's `$00/$01` writes during the DMA HIRAM sense, and the REU's `$FF00` trigger.
   These are the cycles where the earlier snoops end the ISR.
9. **The player uses reSIDfp through pyresidfp (GPL-2.0+)** as a run-time dependency, not shipped.
   The model defaults to 6581 because the stream does not say which chip (the C64 has none).
10. **No licence file yet:** that is the owner's call. The TeensyROM is MIT, and the patch adds files under it.

## Next

* On hardware: `trsnoop.py --range D400-D41C` while the TeensyROM's SID player runs a tune, then `trsid.py`.
* Phase 2, the SID emulated on the Teensy, out through a USB audio dongle on the host port: `docs/USB-AUDIO.md`.
  Step 1, the CPU left over, is in 0.9-sn6 (not yet on hardware): `trsid.py --isr` prints each second the share of
  the CPU the interrupts leave to the main loop (`CPU free`), the worst 1 ms and the longest interrupt, and the
  average at the end. Run it with the menu showing and with a busy game (STATUS v4, docs/PROTOCOL.md).
