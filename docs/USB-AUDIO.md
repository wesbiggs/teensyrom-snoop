# Phase 2: the SID emulated on the Teensy, out as a USB sound card

This note is a plan, not code. Phase 1 (this repository) streams the register writes to a computer,
and the computer runs the emulator. Phase 2 moves the emulator onto the Teensy, so the TeensyROM's USB
port shows up as an audio device on any computer, phone or Pi, with no client program.

## What phase 1 already provides

The capture (`SnoopTap`, the ring) and the exact cycle times are what an on-Teensy emulator needs.
Instead of `snoop_poll` framing them, a consumer in `loop()` (or a low-priority interrupt) would clock
the emulator to each write's cycle and write the register. That is what `host/sidplay.py`'s
`SidRenderer.render_to` does, so it is the reference implementation to port.

## Findings so far (2026-11)

* **reSID 1.0 does not fit.** Its 6581 filter model is lookup tables: 21.5 MB of RAM with the newer 8580 filter
  (`filter8580new.cc`), 17.3 MB with `filter.cc` (measured on the `native/` build). The Teensy 4.1 has 1 MB.
  So: **reSID 0.16** (the classic, small filter; what Frank Bösing ported to the Teensy 3.x), with its wave
  tables (~32 KB) in flash. Its 6581 filter is less faithful than 1.0's.
* **Cost estimate.** On a Raspberry Pi Zero (1 GHz ARM11), reSID 1.0 with fast sampling takes 22-29 % of the core
  for one SID (tune / 8 kHz digi; `native/`, owner's measurement). A 600 MHz M7 is roughly as fast for this code, and
  0.16 is cheaper: guess 10-20 % of the M7, not measured. Run from flash (RAM1 is full), so add a margin.
* **The CPU left over is measured from 0.9-sn5:** STATUS version 4 carries the share interrupts take from the
  main loop (average and worst 0.5 ms sample) and the loop's passes and longest pass. `trsid.py --isr` or
  `trsid-native --isr` prints them every second. Run it with the menu showing and with a busy game.
* **USB audio is now the riskiest piece.** Since this plan was written it was settled that any USB write from the
  TeensyROM can crash a cartridge game (Ms. Pac-Man), stock firmware too: `schedule_transfer()` runs with
  interrupts off. The BASEPRI change (`tools/teensy_core_usb_basepri.py`) removed the late interrupts but games still
  crash, later. USB audio sends a packet every millisecond without pause: the worst case for that. Alternatives:
  an I2S DAC or MQS/PWM output if the TeensyROM leaves those pins free (check the schematic), or the rendered PCM over
  Ethernet (768 kbit/s; NativeEthernet has blocked the main loop before).
* **What the Pi already does:** `native/trsid-native` on a Pi Zero plays and records live, and serves listeners
  (`--serve`). The on-Teensy version would gain no second computer, lower latency (the cycle counter is "now"),
  and could answer SID reads.

## Pieces and risks

1. **Emulator.** reSID has been ported to the Teensy before (Frank Bösing's Teensy64 runs it on a
   Teensy 3.6). reSIDfp is heavier. Start with reSID in "fast" sampling mode at 44.1 kHz.
   * **CPU budget, the unknown.** `isrPHI2` runs on every phi2 cycle and takes a large share of the
     600 MHz. It busy-waits to the data setup and hold times, so much of each 1 µs cycle is gone.
     Measure first: run a `loop()` idle counter against cycles per second with the menu showing and with a game running.
     Then compare with reSID's cost on an M7 at 44.1 kHz (reported in the tens of percent of a slower M4 core).
   * **RAM.** reSID's wave tables are about 4 x 4 KB plus filter tables. They go in flash (`PROGMEM`) or RAM2,
     never RAM1 (66 KB free for locals with vdrive).
2. **USB audio.** Teensyduino's "Serial + MIDI + Audio" USB type and `AudioOutputUSB` from the Audio
   library. The risks:
   * Changing the USB type changes the descriptors every TeensyROM user sees: COM port enumeration,
     `trserial`, the flash tools. It needs upstream's agreement, or a separate build flavour.
   * The Audio library wants its own update interrupt (`AudioStream`, every 128 samples, about 2.9 ms).
     That is another interrupt competing with `isrPHI2`; give it a lower priority.
3. **Clock domains.** The C64 time comes from the phi2 counter, the USB audio time from the host's
   SOF clock. `LivePlayer` in `host/sidplay.py` shows the scheme: render behind "now" and hold a FIFO
   level by dropping or repeating single samples. On the Teensy, "now" is exact (the cycle counter),
   so the transport margin disappears and the latency can be one Audio block plus the FIFO (~10 ms).
4. **Reads.** Out of scope here. A C64 with no SID still floats on `$D419-$D41C` (paddles, `$D41B` random).
   Answering those means driving the data bus on reads without a chip select, which is a separate, riskier change.

## Suggested order

1. Measure the CPU left over (an idle counter in `loop()`), on hardware, with a busy game.
2. Port `render_to` with reSID into a `snoop` consumer. Output to a buffer only, and time it.
3. If it fits: add the USB audio type in a separate build flavour, `AudioOutputUSB`, and the FIFO with drop/repeat.
4. Compare the USB audio with `trsid.py`'s rendering of the same stream (both reSID-family, so the SNR test
   in `tests/test_e2e.py` carries over).
