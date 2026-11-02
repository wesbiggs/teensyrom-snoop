# teensyrom-snoop

Capture C64 bus writes to chosen addresses with a TeensyROM+ and stream them to a computer over USB
serial or TCP, with cycle-exact times.

The first use in mind is the SID: watch `$D400-$D41C`, and a program on the computer can feed the
register writes to a SID emulator (reSID, reSIDfp) and play what the C64 is playing, on a C64 with no
SID chip at all, or record it. Nothing in the firmware is SID-specific: any 1-16 address ranges
(up to 16 distinct 256-byte pages) can be watched, e.g. a second SID at `$D500`, the VIC, a
cartridge's IO1 registers.

**Status: host-tested only.** The capture core, the firmware glue (against host stubs) and the
Python decoder pass `make test`. The patch applies to upstream TeensyROM `136baa83` alone and together with
the teensyrom-vdrive patch (either order). **Not yet built with the Teensy toolchain** (CI does that:
`.github/workflows/test.yml`) and **not run on hardware.**

## How it works

```
 C64 write ──> isrPHI2 (every phi2 cycle)
                 if (!R_Wn && SnoopHot.page[addr>>8]) SnoopTap(addr)     <- one byte load on a write
                     bit test, wait for the data bus, ring <- {CPU stamp, addr, data}
 loop()  ──> SnoopService()
                 ring -> C64-cycle times -> blocks {dt, addr, len, bytes} -> frames (COBS + CRC-16)
                 -> the channel that started the capture, never more than availableForWrite()
```

* `core/` portable C99 (also compiled as C++ by the firmware): the two-level address filter
  (256-byte page map + one 256-bit map per watched page), the ISR ring, the stamp conversion
  and the framing. `docs/PROTOCOL.md` has the wire format.
* `firmware/` the TeensyROM glue (`WriteSnoop.h`: the ISR half, DTCM; `WriteSnoop.ino`: the command
  and the loop service) and `teensyrom-snoop.patch`, which adds them to a TeensyROM tree.
* `host/` `snoopproto.py` (decoder, no dependencies) and `trsnoop.py` (command-line client).

Cost on the bus: every write cycle loads one byte from the page map; with nothing watched that
is all. A watched write waits until the data bus is valid (`nS_DataSetup`, 220 ns), as every IO1/IO2
write handler already does, and stores 8 bytes. The tap runs after the existing snoops, so Kernal
Replace's timing is unchanged. RAM: ~4.9 KB DTCM (ring of 512 entries, maps), ~9 KB RAM2.

## Using it

Build the firmware with the patch (see below), flash it, then on the computer:

```
pip install pyserial                     # for USB serial; TCP needs nothing
host/trsnoop.py --port /dev/ttyACM0 --range D400-D41C
host/trsnoop.py --tcp 192.168.1.50 --range D400-D41C --raw sid.snoop   # also record the stream
host/trsnoop.py --replay sid.snoop
```

Output, one line per block (C64 cycles, seconds, address, bytes):

```
# hello v1 phi2 985248 Hz, cpu 600000000 Hz, gap 32, ranges D400-D41C
     1234567   1.253049  D400: 1F 0C 00 08 41 09 A8
     1234612   1.253094  D418: 0F
# status t=1970496 writes=1325 dropped=0 out_dropped=0 ring_hiwater=25
```

`--gap N` (default 32): writes to consecutive addresses within N cycles of each other share one
block and its time. `--gap 0` times every write on its own (digis through `$D418`).

While a capture streams on the USB serial port, other TeensyROM commands on the same port still
work, but their answers are mixed into the stream: send them over TCP, or stop the capture first.

## Building the firmware

```
git clone https://github.com/SensoriumEmbedded/TeensyROM && cd TeensyROM
git checkout $(cat ../teensyrom-snoop/firmware/UPSTREAM)
git apply ../teensyrom-snoop/firmware/teensyrom-snoop.patch
node tools/build-firmware.mjs --target tr-plus      # arduino-cli, teensy:avr@1.61.0, CRC32@2.0.1
```

With teensyrom-vdrive: apply its patch as well, in either order.

`tools/make_tr_patch.py /path/to/TeensyROM` regenerates the patch after a change to `core/` or
`firmware/` (from `git archive HEAD` of that checkout; it asserts on the upstream text it edits).

## Tests

`make test`: the core through ctypes (`tests/test_core.py`: filtering, block grouping, cycle-exact times
with stamp jitter and 32-bit wrap, ring and output overflow, a 20000-write fuzz with text mixed into the
stream), the core as C++, and the firmware glue against host stubs (`tests/fw_host`, decoded by
`tests/test_fw.py`: serial and TCP runs, replace, disconnect, refusals, NTSC).

## Not done yet

* A firmware build and the `isrPHI2` disassembly check (CI prints both); a hardware run.
* A player: `trsnoop.py` prints and records. Next: feed reSID/reSIDfp and play it live.
* An on-Teensy SID emulator with USB audio out (a later step; this stream is the first half).
