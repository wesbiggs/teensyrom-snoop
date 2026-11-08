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
   * **RAM.** reSID's wave tables are about 4 x 4 KB plus filter tables. They go in flash (`PROGMEM`) or RAM2,
     never RAM1 (66 KB free for locals with vdrive).
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
2. Port `render_to` with reSID into a `snoop` consumer. Output to a buffer only, and time it.
3. Independently of 2: the UAC1 host driver and siTD support, tested with a sine wave on one adaptive dongle,
   and with Ms. Pac-Man running to confirm the host port leaves `isrPHI2` alone (`trsid.py --isr`'s late counts over TCP).
4. Connect them: the emulator renders into the siTD ring, samples per frame following phi2.
5. Compare the dongle's line out with `trsid.py`'s rendering of the same stream (both reSID-family, so the SNR
   test in `tests/test_e2e.py` carries over), recorded through a computer's line in.
