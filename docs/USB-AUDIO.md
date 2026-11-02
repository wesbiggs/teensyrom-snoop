# Phase 2: the SID emulated on the Teensy, out as a USB sound card

This note is a plan, not code. Phase 1 (this repository) streams the register writes to a computer,
and the computer runs the emulator. Phase 2 moves the emulator onto the Teensy, so the TeensyROM's USB
port shows up as an audio device on any computer, phone or Pi, with no client program.

## What phase 1 already provides

The capture (`SnoopTap`, the ring) and the exact cycle times are what an on-Teensy emulator needs.
Instead of `snoop_poll` framing them, a consumer in `loop()` (or a low-priority interrupt) would clock
the emulator to each write's cycle and write the register. That is what `host/sidplay.py`'s
`SidRenderer.render_to` does, so it is the reference implementation to port.

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
