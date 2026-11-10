# Phase 2: the SID emulated on the Teensy, out through a USB audio dongle

This note is a plan, not code. Phase 1 (this repository) streams the register writes to a computer,
and the computer runs the emulator. Phase 2 moves the emulator onto the Teensy and plays it through a
USB audio dongle (a cheap "USB sound card" stick with a 3.5 mm line out) plugged into the TeensyROM's
**USB host port**, the one used for thumb drives and MIDI. No computer, no client program.

## Why the host port and not the device port

An earlier version of this plan made the TeensyROM's device port (the one facing the computer) a USB
sound card. The dongle is better on three counts:

* **The device descriptors stay as they are.** Changing the USB type would change the COM port, `trserial`
  and the flash tools for every TeensyROM user, and need upstream's agreement or a separate build flavour.
* **The device port is what crashes games.** Every transfer on it runs `schedule_transfer()` in the Teensy
  core's `usb.c` with all interrupts off, and any USB write from the TeensyROM crashes Ms. Pac-Man, stock
  firmware too (README, status). The host stack masks only its own interrupt while it queues a transfer
  (`NVIC_DISABLE_IRQ(IRQ_USBHS)`, `USBHost_t36/ehci.cpp:775-828`; the global masks there are a few
  instructions in the timer code), so the phi2 interrupt keeps running. TeensyROM already runs host-port MIDI
  under C64 programs, so the host controller and `isrPHI2` coexist today.
* **One clock fewer** (see "Clocks" below).

The dongle can share the port with a thumb drive or MIDI interface through a hub: TeensyROM declares
`USBHub hub1, hub2` next to `USBHost myusbHost` (`MinimalBoot/Common/IOHandlers.h`).

## What phase 1 already provides

The capture (`SnoopTap`, the ring) and the exact cycle times are what an on-Teensy emulator needs.
Instead of `snoop_poll` framing them, a consumer in `loop()` (or a low-priority interrupt) would clock
the emulator to each write's cycle and write the register. That is what `host/sidplay.py`'s
`SidRenderer.render_to` does, so it is the reference implementation to port.

## Pieces and risks

1. **Emulator.** reSID has been ported to the Teensy before (Frank Bösing's Teensy64 runs it on a
   Teensy 3.6). reSIDfp is heavier. Start with reSID in "fast" sampling mode at 48 kHz (the dongle's native rate).
   * **CPU budget, the unknown.** `isrPHI2` runs on every phi2 cycle and takes a large share of the
     816 MHz (TeensyROM sets it at boot). It busy-waits to the data setup and hold times, so much of each 1 µs cycle is gone.
     Measure first, with the menu showing and with a game running (step 1 below: the main-loop probe in 0.9-sn6).
     Then compare with reSID's cost on an M7 at 48 kHz (reported in the tens of percent of a slower M4 core).
   * **RAM.** reSID 0.16's eight combined-waveform tables are writable `unsigned int` arrays (8 x 16 KB):
     `tools/fetch_resid.py` makes them 8-bit constants in flash (`PROGMEM`, 8 x 4 KB). The `SID` object holds 16 KB
     of filter tables and goes on the heap (RAM2). RAM1 (66 KB free for locals with vdrive) only gets reSID's code.
   * **Licence.** reSID is GPL-2.0-or-later: it is fetched at build time, never committed here, and only an image
     built with it (`-snNr`) is GPL (owner, 2026-11-07).
2. **USB audio host driver (new code).** TeensyROM's bundled USBHost_t36 has no isochronous transfers
   (`ehci.cpp:1259`: "TODO: skip past iTD, siTD when/if we support isochronous") and no audio class driver.
   Audio streams only over isochronous endpoints, so this is the main new work:
   * A `USBDriver` that claims a UAC1 device: find the AudioStreaming interface alternate setting for
     16-bit stereo PCM at 48 kHz, `SET_INTERFACE` to it, `SET_CUR` the sampling frequency on the endpoint,
     and optionally unmute and set the volume through the feature unit.
   * Isochronous OUT in `ehci.cpp`. Dongles are almost all full speed (UAC1), so the transfers are
     **siTDs** in the periodic frame list, one per 1 ms frame (the i.MX RT's EHCI has an embedded
     transaction translator for a full-speed device on the root port, and a high-speed hub's TT works the same
     way). A ring of siTDs a few frames deep, refilled from the USBHS interrupt at its default priority, below phi2.
     The periodic schedule walk must learn to skip past siTDs (the TODO above), since MIDI and HID use interrupt
     QHs in the same list.
   * 48 kHz x 2 channels x 2 bytes is 192 bytes per frame, well inside a full-speed frame alongside MIDI and a drive.
   * Patch footprint: the driver lives in this repository's patch and edits the bundled `src/USBHost_t36`.
     Keep its hunks away from teensyrom-vdrive's, as for the rest of the patch.
3. **Clocks.** A dongle with an *adaptive* or *synchronous* sink takes its sample clock from the host's
   1 ms start-of-frame, which the Teensy generates. So the USB side runs on the Teensy's crystal and needs no
   FIFO against a foreign clock. Only the C64's phi2 differs from it, and the cycle counter measures that
   exactly. Send 47, 48 or 49 samples per frame so the long-run rate follows the C64 (an adaptive sink tracks
   that), or run reSID at the measured phi2 rate against a fixed 48 kHz; either replaces `LivePlayer`'s
   drop-or-repeat. Latency: the siTD ring plus one render block, a few ms.
   * **Pick the dongle accordingly.** *Asynchronous* sinks (often the better DACs) have their own clock and a
     feedback endpoint that tells the host the rate to send; supporting them means reading that endpoint too.
     C-Media CM108/CM119-class sticks are adaptive. Check the endpoint descriptor's `bmAttributes` sync bits
     (bits 3-2: 01 async, 10 adaptive, 11 sync) on a computer before buying more than one.
   * **The owner's dongle** (MOSWAG USB to 3.5 mm adapter; `lsusb -v` on a Pi, 2026-11-08): `0020:0b21` "AB13X USB
     Audio" (a Bluetrum AB13x chip), **full speed**, UAC1. Playback: interface 2, alternate setting 1, endpoint
     `0x03` OUT, isochronous, **synchronous**, `bInterval` 1 (every frame), `wMaxPacketSize` 384, one format only:
     16-bit stereo PCM at **48000 Hz** (`tSamFreq` has a single entry). On hardware with sn8 it took the stream (no errors) and stayed silent; sn9 always
     sends SET_CUR of the rate, as Linux and macOS do.
     Also a capture interface (1, endpoint `0x83` IN) and a HID interface (2 interrupt endpoints, the buttons),
     which the driver leaves alone. A synchronous sink takes its sample clock from the start-of-frame, so it wants
     exactly 48 frames (192 bytes) every 1 ms frame, on the Teensy's crystal. The snoop timeline already runs on
     that crystal (C64 cycles are counted from the Teensy's cycle counter at the nominal phi2), so rendering 1 ms
     of that timeline gives exactly 48 samples a frame: no rate matching at all. The real C64 crystal's offset
     (~100 ppm) only shifts where writes land by that much, which is inaudible.
4. **Reads.** Out of scope here. A C64 with no SID still floats on `$D419-$D41C` (paddles, `$D41B` random).
   Answering those means driving the data bus on reads without a chip select, which is a separate, riskier change.

## Suggested order

1. Measure the CPU left over, on hardware, with a busy game. **In 0.9-sn6:** the main-loop probe (1 ms every
   100 ms during a capture) reports it in STATUS v4; `trsid.py --isr` prints it. Menu idle (hardware, 2026-11-07):
   60.9% of the CPU free; **Ms. Pac-Man running: 43.3%**. The TeensyROM runs at 816 MHz (`set_arm_clock` in
   Teensy.ino), so that is ~497 and **~353 M cycles/s** for an emulator. The longest interruption was 41 us (a
   lower-priority interrupt with phi2 nested in it, no late phi2), so the audio path needs a buffer of well over
   41 us, which a few ms is. Step 2 must fit reSID at 48 kHz in well under 353 M cycles/s, with room for the rest of
   `loop()` and the USB driver.
2. Port `render_to` with reSID into a `snoop` consumer. Output to a buffer only, and time it. **In 0.9-sn7r:**
   the core passes every captured write with its exact C64 time to a consumer (`snoop_t.on_write`); the glue clocks
   one reSID 0.16 (6581, 48 kHz, `SAMPLE_FAST`) to each `$D400-$D41F` write and then to "now" on every loop pass,
   and times it with the cycle counter. STATUS v5 carries the cost (`emu_cyc` for `emu_c64`); `trsid.py --isr`
   prints reSID's share of the time. On the host, the glue's samples equal reSID driven directly with the same
   writes (`make resid test-sid`). **On hardware (owner, 2026-11-08, sn7r, Ms. Pac-Man running, TCP, 39 s): reSID
   takes 26.0% of the time** (wall clock, interrupts included), with 48.7% of the CPU free for the main loop and no
   phi2 interrupt >250 ns late. Its own compute is then ~26% x 48.7% = ~13% of the 816 MHz, ~100 M cycles/s, about
   100 CPU cycles per C64 cycle: it fits with ~74% of the main loop's time to spare for the USB driver and the rest.
   Step 2 done.
3. Independently of 2: the UAC1 host driver and siTD support, tested with a sine wave on one adaptive dongle,
   and with Ms. Pac-Man running to confirm the host port leaves `isrPHI2` alone (`trsid.py --isr`'s late counts over TCP).
   **In 0.9-sn8 (host-tested and compiled for the M7, not yet on hardware):** isochronous OUT through siTDs in the
   bundled USBHost_t36, a UAC1 output driver, and a 1 kHz test tone switched by command `64 53 03` (protocol 6,
   `trsnoop.py --tone on|off|status`). See "Step 3 in 0.9-sn8" below, and "Trying it on hardware" for the owner's checks.
   On hardware (owner, 2026-11-09, sn8): the dongle was claimed and streamed (`frames` rising, errors 0, missed 0,
   0 dB, unmuted) but was silent. **0.9-sn9** always sends `SET_CUR` of the rate (sn8 skipped it with one rate listed)
   and reports volumes in dB. **sn9 on hardware (owner, 2026-11-09): the 1 kHz tone plays** with the dongle straight
   on the host port. Through a USB 2.0 hub (`1a40:0101`, a Terminus FE1.1s: single TT) it streams the same (`frames`
   ~1000/s, errors 0, missed 0, 0 dB, unmuted) and is silent: the hub's transaction translator drops the packets, and a
   start-split for isochronous OUT has no handshake, so nothing reaches the error count. Suspects: the 188 + 4 byte
   split (begin/end) or the start microframe; a test with 188-byte packets and a chosen microframe would tell. **Parked**
   (owner): no hub for now. Whether sn8's silent run was on the hub is not known, so sn9's `SET_CUR` of the rate is
   not proven necessary (harmless either way).
   **With Ms. Pac-Man (owner, 2026-11-09, sn9 without reSID, TCP, 179 s, tone on): no phi2 interrupt late by >250 ns
   (64914 small ones, ~360/s, as over TCP before the dongle), CPU free 42.1% (sn6: 43.3%, so ~1 point for the
   refill), no crash, the tone clean, and on port 2112 `frames 339705, errors 0, missed 0`.** The live player's
   latency rose to 299 ms (sn7r: 107 ms), but a shorter run with the dongle unplugged reached 245 ms: the WiFi.
   Unplug and replug (owner): passes. **Step 3 done on hardware** (with the dongle on the host port, no hub).
   **0.9-sn10** fixes the menu's firmware update with a dongle in (below, step 1). **On hardware (owner, 2026-11-09):
   sn10 updated to itself from the menu with the dongle in and the tone on, and booted.**
4. Connect them: the emulator renders into the siTD ring, samples per frame following phi2. **In 0.9-sn11r
   (host-tested, built with Teensyduino 1.61 here; not yet on hardware):** audio mode 3 (protocol 7) sends reSID's
   samples through a ring to the USB interrupt; see "Step 4 in 0.9-sn11r" and "Trying step 4 on hardware" below.
   **On hardware (owner, 2026-11-10, sn11r, Ms. Pac-Man, TCP, 159 s, `trsid.py --dongle`): it sounds great**; no phi2
   interrupt >250 ns late (56877 small ones), no crash, CPU free 43.9%, reSID 28.0% of the time (sn7r 26.0%), USB
   `frames 590809, errors 0, missed 0`; the buffer's report: `lowest 9.7 ms` since the previous query, but
   `underruns 3, trimmed 138.2 ms` since mode 3 was set (earlier runs and their ends included: three ~50 ms stalls
   of the main loop somewhere, probably at a start; not audible). **Step 4 done.** 0.9-sn12r counts them per capture,
   leaves the drain after a capture out, and puts them in STATUS (protocol 8) so `trsid.py --isr` shows the second a
   stall happens in; not yet on hardware.
5. Compare the dongle's line out with `trsid.py`'s rendering of the same stream (both reSID-family, so the SNR
   test in `tests/test_e2e.py` carries over), recorded through a computer's line in.

## Step 3 in 0.9-sn8

All of it is MIT and in the plain patch (no reSID needed). The sources are in `firmware/usbhost/`; `tools/make_tr_patch.py`
copies them to the tree's `Source/Teensy/src/USBHost_t36/` and edits `ehci.cpp` there (teensyrom-vdrive's patch does not
touch the library).

* **Isochronous OUT (`ehci.cpp`, `isochronous.h`).** One siTD for each of the 32 slots of the periodic frame list, linked
  first in its slot, ahead of the interrupt QHs; `add_qh_to_periodic_schedule` now steps past siTDs to the QH tree (its
  "TODO: skip past iTD, siTD"). A siTD goes out in its slot's frame when active. The refill keeps frames cur+2 .. cur+8
  queued (8 ms of latency, and the refill may come up to ~6 ms late without a gap): it runs in `USBHost::isr()`, on each
  siTD's interrupt-on-complete, so once a frame at the USBHS interrupt's default priority, below phi2. It reads the
  status of the siTD a slot last carried (counts `frames`, `errors`, and `missed` when the queue ran dry), asks the fill
  function for the next packet, flushes the buffer from the data cache and activates the siTD. A 192-byte packet takes
  two start-splits (188 + 4 bytes), in microframes picked from the bandwidth table like an interrupt pipe's.
  * Memory: the siTDs (1 KB) live in DTCM like the QHs, because the schedule code rewrites the link word of whatever
    precedes a QH, a siTD included, and that write must reach the controller without a cache flush. The packet buffers
    (32 x 256 bytes, 256-aligned so none crosses a 4 KB page) are in RAM2 and flushed after each fill. RAM1 in all:
    1.6 KB measured in CI (RAM1 free for locals 73276, sn7 74876: the siTDs, the driver object, ~320 bytes of refill
    code); the setup code and the report's strings are in flash.
  * Interrupt masking: only `NVIC_DISABLE_IRQ(IRQ_USBHS)` while the siTDs are linked or unlinked, as the library does when
    it queues a transfer. No global masking. `isrPHI2` and `SnoopTap` compile to the same code as in sn7 (`tools/armcheck`).
* **The driver (`USBAudioOut.{h,cpp}`, portable parsing in `isoaudio.{h,c}`).** USBHost_t36 offers every interface
  descriptor to every free driver. The driver remembers the AudioControl interface's terminals and units (without
  claiming it), and claims the first AudioStreaming interface with an alternate setting that takes Type I PCM, 2 channels
  of 16 bits, 48000 Hz (in its list or its range), on an isochronous OUT endpoint that is synchronous, adaptive or
  unsynchronised and carries at least 192 bytes. Capture and HID interfaces stay free. Full speed only. Then:
  `SET_INTERFACE` to that alternate setting; `SET_CUR` of the sampling rate on the endpoint (sn8 sent it only when the
  format listed more than one rate, and the owner's dongle stayed silent; from sn9 always, a refusal only noted); the stream starts; then, through the feature unit downstream of the stream's input
  terminal (if any), the volume is set to 0 dB clamped into its `GET_MIN`..`GET_MAX` range and mute is switched off. A
  refused request is noted in the report and skipped, except `SET_INTERFACE`, without which nothing streams.
  Unplugging stops the stream and unlinks the siTDs; plugging back in goes through all of it again.
* **The source (`WriteSnoop.ino`).** `SnoopAudioFill` asks `SnoopAudioSource` for 48 stereo frames (192 bytes) each
  frame, or writes silence when it is NULL. The test tone is a 1 kHz sine at -12 dBFS from a 1024-step table with linear
  interpolation (integer only; measured spurs about -90 dB, tested below -80 dB), phase-continuous across packets. Step 4 sets the source to a
  function that hands over 1 ms of reSID's output; the USB frames and the snoop timeline both run on the Teensy's
  crystal, so 48 samples per frame stay exact.
* **Host tests** (`make test`): `tests/test_usbaudio.py` parses a fixture of the owner's dongle (built from its `lsusb -v`
  summary above; the unit IDs and HID details are typical values, not read from the device) and variants (24-bit, mono,
  44.1 kHz, two rates, a continuous range, asynchronous, small packets, IN, bulk, a feature unit behind a mixer,
  per-channel controls, 3000 corrupted copies), the siTD words, the frame plan against a model of the controller (no
  gaps when the refill comes in time, every empty frame counted when it does not), and the tone. `tests/fw_host` runs the
  command through the glue with a stand-in for the driver: the reports, 1000 packets of exactly 192 bytes equal to one
  continuous tone, left = right, silence when off, a refused mode, a cut-short command, and the command on a capture's
  channel. `tests/test_fw.py` checks the tone's spectrum.
* **Risks only hardware can settle.** That the i.MX RT's embedded transaction translator runs siTDs for a full-speed
  device on the root port (the EHCI way, which the library's interrupt QHs already rely on, with hub address 0 and port
  0); that a siTD's interrupt-on-complete raises the periodic interrupt (UPI; the driver also enables the plain USB
  interrupt, UI, while streaming, in case it does not); that the dongle plays once it has the rate (sn8, without it: streaming, no errors, silent).

## Step 4 in 0.9-sn11r

* **The ring (`isoaudio.{h,c}`, portable, host-tested).** reSID still runs in the main loop, clocked to each captured
  write and to "now" on every pass (sn7). With audio mode 3 its samples also go into an 8192-sample ring (170 ms,
  16 KB in RAM2), and the USB interrupt takes 48 a frame. One producer and one consumer on one core: head is
  stored by the loop, tail by the interrupt, each after its samples (a compiler barrier).
* **No resampling.** The snoop timeline counts C64 cycles from the Teensy's cycle counter at the nominal phi2, and
  the USB frames run on the same crystal, so reSID's 48000 samples per second of that timeline are exactly what
  1000 frames of 48 take. The ring's fill only moves with the loop's bursts.
* **Latency.** The interrupt waits until `ms` are buffered (default 10, 2-60, in the command), then plays. A packet
  that finds fewer than 48 plays what is there, **holds the last sample** for the rest (the 6581's output has a DC
  level: dropping to 0 would click) and waits for the target again (an underrun). Above target + max(target, 20 ms)
  (the loop caught up after a stall, or the dongle was out) it skips the oldest samples back to the target
  (trimmed). The dongle's own latency adds the siTD queue's 8 ms.
* **When it plays.** Only while a capture of `$D400-$D41F` runs: the emulator lives in the capture. `trsid.py
  --dongle` sends mode 3 and starts the capture together; without a capture the dongle holds its last sample.
* **Cost.** RAM1 free for locals 69116 in the sn11r image (Teensyduino 1.61, built here), RAM2 53.7 KB of
  variables (+16 KB for the ring); `isrPHI2` still saves `{r3-r9, sl, fp, lr}`. reSID's CPU is as in sn7r: the ring
  adds a copy of each sample.
* **Host tests.** `tests/test_usbaudio.py`: the ring (waits for the target, in order across wraps with a bursty
  producer, underrun holds and waits, trim after a stall, drops when full). `tests/fw_host` with reSID (`make
  test-sid`): mode 3 before a capture, the packets the dongle takes equal reSID's own output sample for sample
  (6912 samples, no underrun, trim or drop), the report, the hold at the end, a new buffer size, mode 0. Without
  reSID: mode 3 is refused and its byte still read.

## Trying step 4 on hardware (0.9-sn11r)

1. **Flash** `TeensyROM+_0.9-sn11r_full.hex` (built as in the README with `tools/fetch_resid.py`; GPL as a whole).
   The menu's top line reads `v0.9-sn11r`. The dongle straight on the host port, no hub. The menu update is safe
   with the dongle in from sn10 on.
2. **Ms. Pac-Man on the dongle:** launch the game, then `host/trsid.py --tcp <ip> --dongle --isr --no-play` for a
   few minutes (`--no-play`: only the dongle sounds). The `# Snoop audio: SID ...` line comes first. Expect the game's
   sound from the dongle, `>250ns` and `>1us` at 0, no crash, and `reSID N%` about sn7r's 26%.
3. **The buffer's health**, on port 2112 while trsid runs, a few times:
   `host/trsnoop.py --tcp <ip> --tcp-port 2112 --audio status`. `underruns` should stay 0 and `lowest` above 0;
   `trimmed` may grow once at the start. If underruns grow, try a larger buffer: `trsid.py ... --dongle 25`.
   From sn12r the `--isr` lines end with `dongle lowest N ms, underruns N, trimmed N ms` for each second, and the
   summary gives the totals: a stall shows in the second it happened in (compare with the status queries).
4. **Both at once:** the same without `--no-play`: the Mac plays trsid's reSIDfp, ~100-300 ms behind (the WiFi), the
   dongle reSID, ~20 ms behind the C64. They should sound alike (6581 filters differ a little between the two).
5. **Stop** trsid: the dongle goes quiet (the last sample held). `--audio off` sets it back to silence.

Report the `--audio status` lines (the SID part), the `--isr` lines (late counts, reSID %, CPU free) and anything
audible (clicks, dropouts, pitch).

## Trying it on hardware (0.9-sn9)

1. **Flash** `TeensyROM+_0.9-sn9_full.hex` (CI's `firmware` artifact, or build it as in the README). The TR menu's top line
   reads `v0.9-sn9`. Plug the dongle straight into the TeensyROM's USB host port (no hub for the first try), and connect
   its 3.5 mm output to an amplifier or powered speakers at a low volume: the tone is -12 dBFS at the dongle's 0 dB.
   No hub: through the owner's FE1.1s hub it streams silently (see step 3 above).
   **While sn8 or sn9 is installed, unplug the dongle before a firmware update from the TR menu.** The updater
   (`FXUtil.cpp`) masks the phi2, menu button, Ethernet and PIT interrupts before it copies the new image over flash, but
   not `IRQ_USBHS`; with a dongle in, that interrupt runs the fill code (in flash) every 1 ms, so the copy faults once
   it erases the main image. It happened (owner, sn8 to sn9): MinimalBoot then says `Invalid magic numbers, main flash
   image not found`. Recovery: Teensy Loader (the Teensy's program button), which does not depend on the firmware.
   The running image does the copy, so the fix helps from the update *after* sn10 is installed: sn10's patch masks
   `IRQ_USBHS` there too. To check it, update sn10 to itself from the menu with the dongle in and the tone on.
2. **Is it streaming?** `host/trsnoop.py --tcp <ip> --tone status`, twice a few seconds apart. Expected:
   `Snoop audio: silence; 0020:0B21 interface 2 alt 1 ep 03, streaming; frames N, errors 0, missed 0; volume 0.0 dB (-60.0..0.0), unmuted`
   (the volume is what was set, 0 dB, and the dongle's range; `SET_CUR rate failed` at the end means it refused the rate),
   with `frames` rising by ~1000 a second. If not:
   * `no USB audio device`: it was not claimed (the bracket says why, if an audio interface was offered).
   * `starting` that stays: a setup request never finished. `failed` / `SET_INTERFACE failed`: the device refused it.
   * `errors` rising: the controller flags the packets; report the `last siTD status` byte (40 halted, 20 buffer error,
     10 babble, 08 transaction error, 04 missed microframe).
   * `frames` stuck at a few dozen: the refill is not called (no interrupt for siTDs).
3. **The tone.** `host/trsnoop.py --tcp <ip> --tone on`: a steady 1 kHz from the dongle. `--tone off`: silence. Clicks or
   gaps show up as `missed` or `errors` in `--tone status`.
4. **With Ms. Pac-Man** (tone on): launch the game, then `host/trsid.py --tcp <ip> --isr` for a few minutes. The `>250ns`
   and `>1us` late counts should stay at 0, as in sn7r, and the game must not crash. While trsid holds port 6581, ask for
   the status on the TeensyROM's own port: `host/trsnoop.py --tcp <ip> --tcp-port 2112 --tone status` (`missed` and
   `errors` should stay 0). For comparison, the same run with the dongle unplugged.
5. **Unplug and replug** the dongle with the tone on. `--tone status` says `no USB audio device` while it is out; about a
   second after it is back it says `streaming` with `frames` counting from 0 again, and the tone plays with no new
   command (the source stays the tone until switched off or the TeensyROM restarts).

Report back the `--tone status` lines, the late counts, and anything audible (clicks, wrong pitch: 1 kHz exactly).
