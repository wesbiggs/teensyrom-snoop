# teensyrom-snoop protocol (version 1)

All multi-byte numbers are little-endian unless marked otherwise. `varint` is unsigned LEB128
(7 bits per byte, low group first, high bit = more).

## Commands (host to TeensyROM)

On the TeensyROM's command channel (USB serial, or TCP port 2112), like its other remote commands.
They work while a C64 program runs (they are in the always-available part of `ProcessCommand`).

| bytes | meaning |
|---|---|
| `64 53 01 n {first, last}*n gap flags` | **start**: watch `n` (1-16) inclusive ranges, `first`/`last` u16 each. `gap` (u8) see below; `flags` u8, reserved, send 0. |
| `64 53 00` | **stop** |
| `64 53 02` | **status**: queue a STATUS frame now |

Answer: `CC 64` (AckToken) or `7F 9B` (FailToken) followed by a text line. The TeensyROM refuses
a start whose ranges cover more than 16 distinct 256-byte pages, a range with `first > last`, or 0 or
more than 16 ranges; a refused start leaves a running capture alone.

A start answers Ack, then the stream begins on **that channel**, starting with HELLO. A start while
a capture runs replaces it: the old channel gets STATUS and BYE (reason 1) first. Stop sends
STATUS and BYE (reason 0) to the capture's channel. If the capture's TCP client disconnects, the
capture stops by itself.

## Stream (TeensyROM to host)

Frames, each `00 COBS(payload ‖ crc16) 00`:

* a 0x00 delimiter **before and after** each frame (two zeros between frames; empty chunks are skipped),
  so text the TeensyROM prints on the same channel (debug lines, `Busy!`, other commands' answers) cannot
  spoil a frame: it lands between delimiters and fails the CRC.
* `crc16`: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor) of the payload,
  low byte first.
* COBS: consistent overhead byte stuffing, so 0x00 never occurs inside a frame.

The first payload byte is the type.

### HELLO `01`

`01 version:u8 phi2_hz:u32 cpu_hz:u32 gap:u8 n:u8 {first:u16 last:u16}*n`

Starts a run. Time 0 is the start command; `phi2_hz` is the C64 clock the times are counted in
(985248 PAL, 1022727 NTSC, from the TeensyROM's own PAL/NTSC detection); `cpu_hz` is the stamp clock
(the Teensy's, for information).

### WRITES `02`

`02 seq:u16 t0:varint block*`

`block` = `dt:varint addr:u16 len:u8 data[len]`

* `seq` counts WRITES frames from 0 in each run (wraps at 65536). A gap means frames were dropped
  (output buffer full: the host did not read fast enough).
* A block is a run of writes to **consecutive ascending** addresses `addr, addr+1, ...`, each no more
  than `gap` C64 cycles after the previous one. Its time is that of its **first** write: the later bytes
  are up to `gap` cycles later each. `gap` = 0 makes every write its own block, so every write has its
  own exact time. Use 0 for anything timing-sensitive, such as the SID: on real tunes a gap of 32 moved
  writes by up to ~30 cycles, and the rendered audio differed from the exact version by 14-44 dB SNR
  (README). The host tools default to 0.
* Block time = previous block's time + `dt`; the first block's `dt` is from `t0` (0 in practice).
* Times are C64 cycles since the start, exact to the cycle (the Teensy stamps each write at phi2's
  rising edge; the conversion carries its remainder, so it never drifts against the nominal clock).

A frame is sent when it is full (512 payload bytes), 2 ms after its first block, or before any
STATUS/BYE, so latency is a few ms.

### STATUS `03`

`03 next_seq:u16 time:varint dropped:u32 out_dropped:u32 ring_hiwater:u16 writes:u32`

Once a second, on the status command, and before BYE.

* `next_seq`: the seq the next WRITES frame will carry, so the host also sees frames lost at the tail.
* `time`: C64 cycles since the start, now (it moves on with no writes too).
* `dropped`: writes lost because the ISR ring (511 entries) was full, this run.
* `out_dropped`: frames lost because the output buffer (8 KB) was full, this run.
* `ring_hiwater`: most writes waiting in the ring at once since the last STATUS.
* `writes`: writes captured this run.

### BYE `04`

`04 reason:u8`: 0 stop command, 1 replaced by a new start, 2 channel gone.

## What is and is not seen

* Only **C64 CPU writes** (and writes by other bus masters that clock phi2 normally). Not reads.
* Not the TeensyROM's **own DMA** writes (an REU transfer into `$D400`, the remote memory writes):
  the ISR runs the DMA engine instead of the normal path on those cycles.
* The tap runs after the Kernal Replace snoop and the global `fKernRepl`. Those end the cycle early
  (so the write is not seen) only for a C128's `$00/$01` writes with the DMA HIRAM sense, and for the
  REU's `$FF00` trigger write. Every other address is seen.
* The main loop must run at least every 3.5 s (the 32-bit stamp at 600 MHz); a longer stall
  (a long file transfer command) loses that much time from the timeline, not the writes.
