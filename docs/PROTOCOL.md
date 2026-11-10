# teensyrom-snoop protocol (version 8)

All multi-byte numbers are little-endian unless marked otherwise. `varint` is unsigned LEB128
(7 bits per byte, low group first, high bit = more).

## Commands (host to TeensyROM)

On **TCP port 6581**, snoop's own data port (firmware 0.9-sn5+; sn4 used 2113), or on the TeensyROM's command channel (USB
serial, or TCP port 2112) like its other remote commands. They work while a C64 program runs (they are in the
always-available part of `ProcessCommand`).

**Port 6581** takes only these commands; anything else gets `7F 9B` and a line of text. The stream goes back on
the same connection, so port 2112 stays free for other tools during a capture (`trls`, `trvdrive`, file
transfers). The TeensyROM serves one client on each port. NativeEthernet hands a new connection over only once
it has sent something, and waits up to 10 s for that while holding up the TeensyROM's main loop: send the
command right after connecting. The port listens once the TeensyROM's Ethernet is up (when its own 2112
listens). The protocol on 6581 is the same; only the port is new, so the version stays 3.

| bytes | meaning |
|---|---|
| `64 53 01 n {first, last}*n gap send_ms` | **start**: watch `n` (1-16) inclusive ranges, `first`/`last` u16 each. `gap` (u8) see below. `send_ms` (u8): send at most this often, in ms; 0 = the channel's default (20 on USB serial; on TCP, Nagle's rule from version 3: at once while nothing is unacknowledged, else up to 20 ms or 1 KB). Version 1 called this byte `flags` and required 0, which still means the default. |
| `64 53 00` | **stop** |
| `64 53 02` | **status**: queue a STATUS frame now |
| `64 53 03 mode` | **audio** (version 6, firmware 0.9-sn8+): what the USB audio dongle on the TeensyROM's host port plays. `mode` 0 silence, 1 the test tone (1 kHz sine, -12 dBFS, both channels), 2 no change. Answered with Ack and **one line of text** reporting the source and the dongle (below). Works with or without a capture running, on any channel; the tone plays until switched off or the TeensyROM restarts. |
| `64 53 03 03 ms` | **audio: the SID** (version 7, an image with reSID, 0.9-sn11r+): the dongle plays the TeensyROM's own reSID, the emulator that runs while a capture of `$D400-$D41F` does, with `ms` (u8) of buffer between it and the USB frames (0 = 10; clamped to 2-60). Answered like the other modes. Without a capture the dongle holds its last sample (silence). A build without reSID answers `7F 9B` and `Snoop audio: no SID emulator in this build ...` (the `ms` byte is still read). |

Answer: `CC 64` (AckToken) or `7F 9B` (FailToken) followed by a text line (a refusal; the audio command's report
after an Ack). The audio command refuses a `mode` above 3. The TeensyROM refuses
a start whose ranges cover more than 16 distinct 256-byte pages, a range with `first > last`, or 0 or
more than 16 ranges; a refused start leaves a running capture alone.

A start answers Ack, then the stream begins on **that channel**, starting with HELLO. A start while
a capture runs replaces it: the old channel gets STATUS and BYE (reason 1) first. Stop sends
STATUS and BYE (reason 0) to the capture's channel. If the capture's TCP client disconnects, the
capture stops by itself.

The audio command's report, for example:

```
Snoop audio: tone 1000 Hz; 0020:0B21 interface 2 alt 1 ep 03, streaming; frames 52311, errors 0, missed 0; volume 0.0 dB (-45.0..0.0), unmuted
Snoop audio: SID (reSID 6581), 10 ms buffer; 0020:0B21 interface 2 alt 1 ep 03, streaming; frames 90210, errors 0, missed 0; volume 0.0 dB (-60.0..0.0), unmuted; SID buffered 11.2 ms, lowest 9.8 ms, underruns 0, trimmed 0.0 ms, dropped 0.0 ms
Snoop audio: silence; no USB audio device
```

* the source (`tone 1000 Hz`, `silence`, or `SID (reSID 6581), N ms buffer`, with `, no capture running` when none is), then the device: USB vendor:product, the AudioStreaming interface and
  alternate setting, the endpoint, and `streaming`, `starting` (its setup requests are under way) or `failed`.
* `frames`, `errors`, `missed`: 1 ms packets sent without an error, packets the host controller flagged (with the last
  siTD status byte), and frames that went out with nothing queued because the refill came too late. Counted
  since the device was plugged in; the counts trail by up to 32 ms (a packet is counted when its slot is reused).
* the feature unit's volume as set (0 dB clamped into the device's range; in dB from 0.9-sn9, 1/256 dB units before) and whether it was unmuted,
  or `no feature unit`; `... failed` names a setup request the device refused (the stream starts anyway unless it was
  `SET_INTERFACE`).
* with nothing plugged in, `no USB audio device`, and in brackets why an audio device that was offered was not taken
  (no 16-bit stereo 48 kHz, an asynchronous sink, packets too small).
* with the SID as the source (version 7): the emulator's samples waiting for the USB interrupt now (`buffered`), the
  fewest there were when a packet was taken since the last report (`lowest`, `-` before any; each report restarts
  it), `underruns` (packets that found fewer than 48: the rest held the last sample, then the buffer filled to its
  target again before playing on), `trimmed` (samples skipped when the buffer held more than the target + max(target,
  20 ms), after the main loop stalled), `dropped` (samples that did not fit the 170 ms ring). From 0.9-sn12 these
  three count from the start of the current (or last) capture, and the drain after a capture stops is not an
  underrun (in sn11 they counted from when mode 3 was set, every capture's end included).

The protocol on the stream is unchanged in versions 6 and 7: only the command is new (HELLO carries the version).
Version 8 adds three fields to STATUS (below).

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

**Batching (version 2).** Every USB transfer the Teensy core starts runs with all interrupts off, the bus
interrupt included, and each small write starts one. Streaming in small pieces made cartridge games glitch or
crash on USB serial. So the TeensyROM hands the channel what it holds at most every `send_ms` (default 20 on
USB serial), or once 1 KB is waiting. That is about 50 transfers a second instead of ~500, and it adds up to
`send_ms` of latency.

A frame is sent when it is full (512 payload bytes), 2 ms after its first block, or before any
STATUS/BYE, so latency is a few ms.

### STATUS `03`

`03 next_seq:u16 time:varint dropped:u32 out_dropped:u32 ring_hiwater:u16 writes:u32 late:u32 late250:u32 late1000:u32 probe_cyc:u32 probe_free:u32 probe_worst:u16 probe_stall:u32 emu_cyc:u32 emu_c64:u32 audio_low:u16 audio_under:u32 audio_trim:u32`

The three `late` counters are new in version 3, the four `probe` fields in version 4, the two `emu` fields in
version 5, the three `audio` fields in version 8. A version 2 STATUS ends after `writes`, a version 3 one after
`late1000`, a version 4 one after `probe_stall`, a version 5-7 one after `emu_c64`, and the decoder accepts all five.

Once a second, on the status command, and before BYE.

* `next_seq`: the seq the next WRITES frame will carry, so the host also sees frames lost at the tail.
* `time`: C64 cycles since the start, now (it moves on with no writes too).
* `dropped`: writes lost because the ISR ring (511 entries) was full, this run.
* `out_dropped`: frames lost because the output buffer (8 KB) was full, this run.
* `ring_hiwater`: most writes waiting in the ring at once since the last STATUS.
* `writes`: writes captured this run.
* `late`, `late250`, `late1000`: phi2 interrupts this run that came late: any amount (more than
  `nS_MaxAdj` after the previous one), more than ~250 ns (a cartridge ROM read answered then can be wrong),
  and more than ~1 us (a bus cycle missed). This is a diagnostic of what else holds the bus interrupt up. It is
  counted in isrPHI2's existing late branch, so on-time cycles pay nothing.
* `probe_cyc`, `probe_free`, `probe_worst`, `probe_stall` (version 4, firmware 0.9-sn6+): the main-loop CPU probe,
  **since the last STATUS** (they restart at each one). While a capture runs, the main loop spins for 1 ms every
  100 ms reading the CPU cycle counter; a gap of more than 40 CPU cycles between two reads is an interrupt
  (isrPHI2 above all). `probe_cyc` is the CPU cycles probed, `probe_free` those with no interrupt running, so
  `probe_free / probe_cyc` is the share of the CPU the interrupts leave to the main loop: what an emulator there
  (phase 2, docs/USB-AUDIO.md) could use. `probe_worst` is the lowest share of any single 1 ms probe, per mille
  (1000 when none ran), and `probe_stall` the longest single interruption in CPU cycles. Interrupts shorter than
  40 cycles count as free, so the share is an upper bound. The probe holds the TeensyROM's main loop up for
  1 ms each time (1% of the time).
* `emu_cyc`, `emu_c64` (version 5, firmware 0.9-sn7r: a build with reSID, `tools/fetch_resid.py`), **since the last
  STATUS**: the SID emulator on the TeensyROM took `emu_cyc` CPU cycles, measured on the wall clock (interrupts that
  came in meanwhile included), to emulate `emu_c64` C64 cycles. `emu_cyc / cpu_hz` over `emu_c64 / phi2_hz` is the
  share of the time it takes: it keeps up while that stays well below 1. Both are 0 in a build without reSID.
  From version 7 the emulator's output can go to the USB audio dongle (`64 53 03 03 ms`, phase 2 step 4).
* `audio_low`, `audio_under`, `audio_trim` (version 8, firmware 0.9-sn12r), **since the last STATUS**, while the dongle
  plays the emulator (audio mode 3): the fewest samples (48 kHz) waiting when the USB interrupt took a packet
  (`FFFF`: no packet taken, e.g. another audio mode), the packets that found fewer than 48 (underruns), and the
  samples skipped to get back to the buffer's target after a stall (trimmed). `FFFF`, 0, 0 without reSID.
  `trsid.py --isr` prints them each second, so a stall shows in the second it happened.

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
