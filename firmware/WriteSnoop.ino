// teensyrom-snoop: the main-loop half.  Command 0x6453 from the USB serial or TCP channel, and
// SnoopService() from loop().  Protocol: docs/PROTOCOL.md in teensyrom-snoop.

#ifdef FeatWriteSnoop

#include "snoop/snoop.c"

bool SnoopReady = false;
bool SnoopDtrAtStart = false;   // a USB serial run whose host had DTR up: stop when it drops (port closed)

// Sending in batches.  Every USB transfer the Teensy core starts (schedule_transfer in usb.c) runs with
// all interrupts off, the phi2 ISR included, and a small write starts one of its own.  Streaming in small
// pieces (~500 transfers/s) made cartridge games glitch or crash (Ms. Pac-Man) while TCP was fine.  So the
// output goes out at most every SnoopSendEveryMs (20 on USB serial unless the start command asks for
// another value; 0 = at once, TCP's default), or when SnoopBatchBytes are waiting, in as few writes as
// the channel takes.
//
// On TCP (from 0.9-sn3) the default is Nagle's rule, as upstream's SwiftLink Tx (PR #80): send at once while
// nothing sent earlier is unacknowledged (availableForWrite() back at the full socket size), otherwise collect
// until 1 KB or SnoopTcpMaxMs.  A fast LAN keeps near-zero latency; a slow link (multi-hop WiFi) gets fewer,
// larger segments instead of one small one every ~2 ms.  A send interval in the start command replaces the rule.
#define SnoopUsbDefaultMs  20
#define SnoopTcpMaxMs      20
#define SnoopBatchBytes    1024
bool SnoopNagle = false;
uint32_t SnoopSendEveryMs = 0;
uint32_t SnoopLastSendMs = 0;
DMAMEM uint8_t SnoopTxBuf[2048];   // one USB serial transmit buffer (TX_SIZE) per write

// CPU probe (0.9-sn6), for phase 2 (docs/USB-AUDIO.md): while a capture runs, every SnoopProbeEveryMs the loop
// spins for SnoopProbeUs on the cycle counter, and snoop_cpu_probe counts the gaps the interrupts took.  The share
// left goes out in STATUS (protocol 4).  It holds up loop() for that 1 ms, 1% of the time.
#define SnoopProbeEveryMs  100
#define SnoopProbeUs       1000
#ifndef SnoopCycles
#define SnoopCycles() ARM_DWT_CYCCNT
#endif
uint32_t SnoopLastProbeMs = 0;
FLASHMEM uint32_t SnoopReadCycles() { return SnoopCycles(); }

#ifdef FeatTCPListen
// The data port (0.9-sn4).  A client connects to SnoopDataPort and sends the same commands there; the stream
// goes back on that connection, so the TeensyROM's own port (2112) stays free for other tools (trls, trvdrive,
// file transfers) during a capture.  A start on 2112 or USB serial still works as before.
// NativeEthernet's EthernetServer::available() listens on first call, and hands over a client only once it has
// sent something, waiting up to 10 s for that: a client must send its command right after connecting.
#define SnoopDataPort 6581
EthernetServer SnoopServer(SnoopDataPort);
EthernetClient SnoopClient;
#endif

FLASHMEM void SnoopSetup()
{
   if (SnoopReady) return;
   snoop_init(&SnoopState, &SnoopHot);
   SnoopReady = true;
}

FLASHMEM uint32_t SnoopPhi2Hz()
{
   return (IO1[wRegVid_TOD_Clks] & rvtcNTSC) ? 1022727 : 985248;
}

// The SID emulator (0.9-sn7, phase 2 step 2, docs/USB-AUDIO.md): only in a tree that has reSID in src/reSID/
// (tools/fetch_resid.py; GPL-2.0-or-later, so such an image is GPL as a whole).  One 6581 at 48 kHz, fast
// sampling, clocked to each captured write to $D400-$D41F on its exact C64 cycle and then to "now" on every
// loop pass.  STATUS (protocol 5) reports its cost as emu_cyc (CPU cycles taken, interrupts included) for
// emu_c64 (C64 cycles emulated).  From 0.9-sn11 (phase 2 step 4) its samples can go to the USB audio dongle:
// audio mode 3 puts them in SnoopSidRing, which the USB interrupt empties 48 a frame (below).
// SnoopNoAudio (not defined in a normal build): leave out the dongle driver, the audio command and reSID,
// so a build can measure the write tap and its streaming on their own (.github/workflows/size.yml).
#if !defined(SNOOP_RESID_H) && !defined(SnoopNoAudio) && defined(__has_include)
#if __has_include("src/reSID/sid.h")
#define SNOOP_RESID_H "src/reSID/sid.h"
#endif
#endif
#if defined(SNOOP_RESID_H) && defined(SnoopNoAudio)
#error "SnoopNoAudio leaves out the audio path reSID feeds"
#endif
#ifdef SNOOP_RESID_H
#include SNOOP_RESID_H
#define SnoopSidBase    0xD400
#define SnoopSidRate    48000
reSID::SID *SnoopSid = NULL;               // on the heap (RAM2): its filter tables are 16 KB
uint64_t SnoopSidAt = 0;                   // C64 cycles emulated this run
DMAMEM short SnoopSidBuf[256];
void (*SnoopSidSink)(const short *Samples, int n) = NULL;   // a copy of the samples (the host test)

// The samples for the dongle.  reSID renders up to "now" on each loop pass, in bursts; the USB interrupt takes
// 48 every 1 ms frame once SnoopSidMs are buffered (the latency, settable in the audio command).  The snoop
// timeline counts C64 cycles from the Teensy's cycle counter and the USB frames run on the same crystal, so the
// two rates are the same and nothing is resampled.  After a stall the ring trims back to the target (isoaudio.h).
#include "src/USBHost_t36/isoaudio.h"
#define SnoopSidRingSize 8192                // samples: 170 ms, 16 KB in RAM2
DMAMEM int16_t SnoopSidRingBuf[SnoopSidRingSize];
isoaudio_ring_t SnoopSidRing;
bool SnoopSidFeed = false;                   // audio mode 3: the samples go to the ring
// The ring's counts for this capture (0.9-sn12): underruns while a capture runs (the drain after it is not
// one), counted by the USB interrupt; the lowest fill since the last audio report; what STATUS already has.
volatile uint32_t SnoopSidUnderruns = 0;
uint32_t SnoopSidLowReport = 0xFFFFFFFFu;
uint32_t SnoopSidSeenUnder = 0, SnoopSidSeenTrim = 0;

FLASHMEM void SnoopSidCountsReset()
{
   SnoopSidUnderruns = 0;
   SnoopSidRing.trimmed = SnoopSidRing.dropped = 0;
   SnoopSidRing.low = SnoopSidLowReport = 0xFFFFFFFFu;
   SnoopSidSeenUnder = SnoopSidSeenTrim = 0;
}

FLASHMEM void SnoopSidClockTo(uint64_t Time)
{
   if (SnoopSid == NULL || Time <= SnoopSidAt) return;
   const uint64_t Span = Time - SnoopSidAt;
   const uint32_t Start = SnoopCycles();
   uint64_t Left = Span;
   while (Left)
   {
      reSID::cycle_count d = Left > 0x40000000u ? 0x40000000 : (reSID::cycle_count)Left;
      Left -= (uint64_t)d;
      while (d > 0)
      {
         const int n = SnoopSid->clock(d, SnoopSidBuf, (int)(sizeof SnoopSidBuf / sizeof SnoopSidBuf[0]));
         if (SnoopSidSink && n > 0) SnoopSidSink(SnoopSidBuf, n);
         if (SnoopSidFeed && n > 0) isoaudio_ring_put(&SnoopSidRing, SnoopSidBuf, (uint32_t)n);
      }
   }
   SnoopState.emu_cyc += SnoopCycles() - Start;
   SnoopState.emu_c64 += (uint32_t)Span;
   SnoopSidAt = Time;
}

FLASHMEM void SnoopSidWrite(void *, uint64_t Time, uint16_t Address, uint8_t Data)
{
   if ((uint16_t)(Address - SnoopSidBase) >= 0x20) return;
   SnoopSidClockTo(Time);
   SnoopSid->write(Address & 0x1F, Data);
}

FLASHMEM void SnoopSidStart()
{
   if (SnoopSid == NULL) SnoopSid = new reSID::SID();
   SnoopSid->set_chip_model(reSID::MOS6581);
   SnoopSid->set_sampling_parameters(SnoopPhi2Hz(), reSID::SAMPLE_FAST, SnoopSidRate);
   SnoopSid->reset();
   SnoopSidAt = 0;
   SnoopSidCountsReset();   // the dongle's counts are per capture
   SnoopState.on_write = SnoopSidWrite;
   SnoopState.on_write_ctx = NULL;
}
#endif

// Send what the FIFO holds, never more than the channel takes without blocking
// (EthernetClient::write busy-waits for buffer space; USB serial waits for the host).
FLASHMEM void SnoopSend(Stream *Out)
{
   while (snoop_out_avail(&SnoopState))
   {
      int Room = Out->availableForWrite();
      if (Room <= 0) return;
      uint32_t n = snoop_out_take(&SnoopState, SnoopTxBuf, Room < (int)sizeof SnoopTxBuf ? (uint32_t)Room : sizeof SnoopTxBuf);
      Out->write(SnoopTxBuf, n);
   }
}

// Give a stopped run's last frames (STATUS, BYE) to their channel, for up to TimeoutmS
FLASHMEM void SnoopFlushTo(Stream *Out, uint32_t TimeoutmS)
{
   const uint32_t Start = millis();
   while (Out != NULL && snoop_out_avail(&SnoopState) && millis() - Start < TimeoutmS) SnoopSend(Out);
   SnoopState.out_tail = SnoopState.out_head;   // whatever is left (channel gone or too slow) is dropped
}

FLASHMEM bool SnoopGetByte(uint8_t *b)
{
   uint32_t v;
   if (!GetUInt(&v, 1)) return false;   // GetUInt sends FailToken + "Timeout!" itself
   *b = (uint8_t)v;
   return true;
}

#ifndef SnoopNoAudio
// The USB audio dongle on the host port (0.9-sn8, phase 2 step 3, docs/USB-AUDIO.md).  USBAudioOut (in
// src/USBHost_t36, with isochronous OUT added to ehci.cpp) claims a USB Audio Class 1 device that takes 16-bit
// stereo at 48 kHz, and streams whatever SnoopAudioFill gives it: 48 stereo frames (192 bytes) every 1 ms frame,
// asked for in the USBHS interrupt (below phi2's priority) a few frames ahead.  The source is silence, or the
// 1 kHz test tone, or (0.9-sn11, phase 2 step 4, an image with reSID) the SID emulator through SnoopSidRing,
// switched by command 64 53 03.
#ifndef SNOOP_AUDIO_STUB   // tests/fw_host brings a stand-in
#include "src/USBHost_t36/USBAudioOut.h"
#else
#include "src/USBHost_t36/isoaudio.h"
#endif
#define SnoopAudioRate  48000
#define SnoopToneHz     1000
#define SnoopToneAmp    8192     // -12 dBFS
isoaudio_tone_t SnoopTone;
// The sample source: Frames stereo frames, 16-bit little endian, to Packet.  NULL: silence.  Runs in the USBHS
// interrupt, so it must not block; swap it by storing a new pointer (one aligned word).
typedef void (*SnoopAudioSourceFn)(uint8_t *Packet, uint32_t Frames);
volatile SnoopAudioSourceFn SnoopAudioSource = NULL;

FLASHMEM void SnoopToneSource(uint8_t *Packet, uint32_t Frames)
{
   isoaudio_tone_packet(&SnoopTone, Packet, Frames);
}

#ifdef SNOOP_RESID_H
#define SnoopSidDefaultMs 10
#define SnoopSidMinMs     2
#define SnoopSidMaxMs     60
uint8_t SnoopSidMs = SnoopSidDefaultMs;
FLASHMEM void SnoopSidSource(uint8_t *Packet, uint32_t Frames)
{
   if (isoaudio_ring_packet(&SnoopSidRing, Packet, Frames) && SnoopState.running) SnoopSidUnderruns++;
}

// From SnoopService, before snoop_poll (which may send STATUS): what the dongle's buffer did since the last pass,
// into the STATUS fields (version 8).  Taking the lowest fill and starting it afresh can lose one packet's value
// if the interrupt comes in between: harmless for a minimum taken every pass.
FLASHMEM void SnoopSidFold()
{
   if (SnoopAudioSource != SnoopSidSource) return;
   const uint32_t Low = SnoopSidRing.low;
   SnoopSidRing.low = 0xFFFFFFFFu;
   if (Low < SnoopSidLowReport) SnoopSidLowReport = Low;
   if (Low < SnoopState.audio_low) SnoopState.audio_low = (uint16_t)Low;
   const uint32_t U = SnoopSidUnderruns, T = SnoopSidRing.trimmed;
   SnoopState.audio_under += U - SnoopSidSeenUnder;
   SnoopState.audio_trim += T - SnoopSidSeenTrim;
   SnoopSidSeenUnder = U;
   SnoopSidSeenTrim = T;
}
#endif

FLASHMEM void SnoopAudioFill(void *, uint8_t *Packet, uint32_t Len)
{
   const SnoopAudioSourceFn Source = SnoopAudioSource;
   if (Source) Source(Packet, Len / 4);
   else memset(Packet, 0, Len);
}

USBAudioOut SnoopDongle(myusbHost, SnoopAudioFill);   // a USBHost_t36 driver: waits for a device on the host port

// One line: the source, and what the dongle is doing.  The strings stay in flash (PSTR): RAM1 is scarce.
FLASHMEM const char *SnoopAudioStep(uint8_t Step)
{
   switch (Step)
   {
   case 1: return PSTR("SET_INTERFACE");
   case 2: return PSTR("SET_CUR rate");
   case 3: return PSTR("GET_MIN volume");
   case 4: return PSTR("GET_MAX volume");
   case 5: return PSTR("SET_CUR volume");
   case 6: return PSTR("SET_CUR mute");
   default: return PSTR("?");
   }
}

// A USB Audio volume (1/256 dB) as dB with one decimal, to Out
FLASHMEM void SnoopAudioDb(Stream *Out, int16_t V)
{
   const int32_t Tenths = ((int32_t)V * 10 + (V < 0 ? -128 : 128)) / 256;   // rounded half away from zero
   const int32_t A = Tenths < 0 ? -Tenths : Tenths;
   Out->printf(PSTR("%s%ld.%ld"), Tenths < 0 ? PSTR("-") : PSTR(""), (long)(A / 10), (long)(A % 10));
}

// samples at 48 kHz as ms with one decimal, to Out
FLASHMEM void SnoopAudioMs(Stream *Out, uint32_t Samples)
{
   const uint32_t Tenths = (Samples * 10 + 24) / 48;
   Out->printf(PSTR("%lu.%lu"), (unsigned long)(Tenths / 10), (unsigned long)(Tenths % 10));
}

FLASHMEM void SnoopAudioReport(Stream *Out)
{
   USBAudioOut::Info i;
   SnoopDongle.info(&i);
#ifdef SNOOP_RESID_H
   if (SnoopAudioSource == SnoopSidSource)
      Out->printf(PSTR("Snoop audio: SID (reSID 6581), %u ms buffer%s; "), SnoopSidMs,
                  SnoopState.running ? PSTR("") : PSTR(", no capture running"));
   else
#endif
   Out->printf(PSTR("Snoop audio: %s; "), SnoopAudioSource == SnoopToneSource ? PSTR("tone 1000 Hz") : PSTR("silence"));
   if (!i.attached)
   {
      Out->printf(PSTR("no USB audio device%s\n"),
                  i.last_reject == ISOAUDIO_NO_FORMAT ? PSTR(" (one was offered: no 16-bit stereo 48 kHz)") :
                  i.last_reject == ISOAUDIO_ASYNC ? PSTR(" (one was offered: asynchronous, not supported)") :
                  i.last_reject == ISOAUDIO_SMALL ? PSTR(" (one was offered: packets too small)") : PSTR(""));
      return;
   }
   Out->printf(PSTR("%04X:%04X interface %u alt %u ep %02X, %s; frames %lu, errors %lu, missed %lu"),
               i.vid, i.pid, i.iface, i.alt, i.ep,
               i.streaming ? PSTR("streaming") : i.step < 0 ? PSTR("failed") : PSTR("starting"),
               (unsigned long)i.iso.frames, (unsigned long)i.iso.errors, (unsigned long)i.iso.missed);
   if (i.iso.errors) Out->printf(PSTR(" (last siTD status %02lX)"), (unsigned long)i.iso.status);
   if (i.fu)
   {
      Out->printf(PSTR("; volume "));
      SnoopAudioDb(Out, i.volume);
      Out->printf(PSTR(" dB ("));
      SnoopAudioDb(Out, i.vol_min);
      Out->printf(PSTR(".."));
      SnoopAudioDb(Out, i.vol_max);
      Out->printf(PSTR(")%s"), i.unmuted ? PSTR(", unmuted") : PSTR(""));
   }
   else Out->printf(PSTR("; no feature unit"));
   if (i.failed_step) Out->printf(PSTR("; %s failed"), SnoopAudioStep(i.failed_step));
#ifdef SNOOP_RESID_H
   if (SnoopAudioSource == SnoopSidSource)
   {  // the ring: buffered now, the lowest since the last report (at a packet); this capture's underruns, trimmed, dropped
      const uint32_t RingLow = SnoopSidRing.low;
      const uint32_t Low = RingLow < SnoopSidLowReport ? RingLow : SnoopSidLowReport;
      SnoopSidRing.low = SnoopSidLowReport = 0xFFFFFFFFu;
      Out->printf(PSTR("; SID buffered "));
      SnoopAudioMs(Out, isoaudio_ring_fill(&SnoopSidRing));
      Out->printf(PSTR(" ms, lowest "));
      if (Low == 0xFFFFFFFFu) Out->printf(PSTR("-"));
      else SnoopAudioMs(Out, Low);
      Out->printf(PSTR(" ms, underruns %lu, trimmed "), (unsigned long)SnoopSidUnderruns);
      SnoopAudioMs(Out, SnoopSidRing.trimmed);
      Out->printf(PSTR(" ms, dropped "));
      SnoopAudioMs(Out, SnoopSidRing.dropped);
      Out->printf(PSTR(" ms"));
   }
#endif
   Out->printf(PSTR("\n"));
}

// 64 53 03 mode: 0 silence, 1 the test tone, 2 only report, 3 ms: the SID emulator with ms of buffer (0 = 10).
// The USB interrupt cannot be inside a source while this runs (it preempts the loop, not the other way round),
// so each source is set up while the pointer is elsewhere and switched in with one store.
FLASHMEM void SnoopAudioCommand()
{
   uint8_t Mode, Ms = 0;
   if (!SnoopGetByte(&Mode)) return;
   if (Mode == 3 && !SnoopGetByte(&Ms)) return;
   if (Mode > 3)
   {
      SendU16(FailToken);
      CmdChannel->printf(PSTR("Snoop audio: mode 0 silence, 1 tone, 2 report, 3 SID\n"));
      return;
   }
#ifndef SNOOP_RESID_H
   if (Mode == 3)
   {
      SendU16(FailToken);
      CmdChannel->printf(PSTR("Snoop audio: no SID emulator in this build (an -snNr image has reSID)\n"));
      return;
   }
#else
   if (Mode == 3)
   {
      if (Ms == 0) Ms = SnoopSidDefaultMs;
      if (Ms < SnoopSidMinMs) Ms = SnoopSidMinMs;
      if (Ms > SnoopSidMaxMs) Ms = SnoopSidMaxMs;
      if (SnoopAudioSource != SnoopSidSource || Ms != SnoopSidMs)
      {
         SnoopAudioSource = NULL;
         const uint32_t Target = (uint32_t)Ms * (SnoopAudioRate / 1000);
         isoaudio_ring_init(&SnoopSidRing, SnoopSidRingBuf, SnoopSidRingSize, Target,
                            Target + (Target > 960 ? Target : 960));   // trim above target + max(target, 20 ms)
         SnoopSidCountsReset();
         SnoopSidMs = Ms;
         SnoopSidFeed = true;
         SNOOP_BARRIER();
         SnoopAudioSource = SnoopSidSource;
      }
   }
   if (Mode == 0 || Mode == 1) SnoopSidFeed = false;
#endif
   if (Mode == 0) SnoopAudioSource = NULL;
   if (Mode == 1 && SnoopAudioSource != SnoopToneSource)
   {
      SnoopAudioSource = NULL;
      isoaudio_tone_init(&SnoopTone, SnoopToneHz, SnoopAudioRate, SnoopToneAmp);
      SNOOP_BARRIER();   // the tone is ready before the interrupt can see the pointer
      SnoopAudioSource = SnoopToneSource;
   }
   SendU16(AckToken);
   SnoopAudioReport(CmdChannel);
}
#endif // SnoopNoAudio

// 0x64 0x53, then: 0 stop | 1 start n {first lo,hi, last lo,hi}*n gap flags | 2 status | 3 audio mode
FLASHMEM void SnoopCommand()
{
   SnoopSetup();
   uint8_t Op;
   if (!SnoopGetByte(&Op)) return;

   if (Op == 0)
   {
      SendU16(AckToken);
      if (SnoopState.running)
      {
         snoop_stop(&SnoopState, SNOOP_BYE_STOPPED, micros(), ARM_DWT_CYCCNT);
         SnoopFlushTo(SnoopOut, 200);
      }
      SnoopOut = NULL;
      return;
   }
   if (Op == 2)
   {
      SendU16(AckToken);
      if (SnoopState.running) snoop_status(&SnoopState);
      return;
   }
#ifndef SnoopNoAudio
   if (Op == 3)
   {
      SnoopAudioCommand();
      return;
   }
#endif
   if (Op != 1)
   {
      SendU16(FailToken);
      CmdChannel->println("Snoop: unknown op");
      return;
   }

   uint8_t N, Gap, Flags, b[4];
   if (!SnoopGetByte(&N)) return;
   if (N == 0 || N > SNOOP_MAX_RANGES)
   {
      SendU16(FailToken);
      CmdChannel->printf("Snoop: 1-%d ranges\n", SNOOP_MAX_RANGES);
      return;
   }
   snoop_range_t Ranges[SNOOP_MAX_RANGES];
   for (uint8_t i = 0; i < N; i++)
   {
      for (uint8_t k = 0; k < 4; k++) if (!SnoopGetByte(&b[k])) return;
      Ranges[i].first = (uint16_t)(b[0] | b[1] << 8);
      Ranges[i].last  = (uint16_t)(b[2] | b[3] << 8);
   }
   if (!SnoopGetByte(&Gap) || !SnoopGetByte(&Flags)) return;   // Flags: send interval in ms, 0 = the channel's default

   int Res = snoop_check(Ranges, N);   // before anything stops: a refused start leaves a running capture alone
   if (Res == 0)
   {
      if (SnoopState.running && SnoopOut != CmdChannel)
      {  // a start from another channel: the old one gets its BYE first
         snoop_stop(&SnoopState, SNOOP_BYE_REPLACED, micros(), ARM_DWT_CYCCNT);
         SnoopFlushTo(SnoopOut, 200);
      }
      Res = snoop_start(&SnoopState, Ranges, N, Gap, SnoopPhi2Hz(), F_CPU_ACTUAL, micros(), ARM_DWT_CYCCNT);
   }
   if (Res != 0)
   {
      SendU16(FailToken);
      if (Res == -2) CmdChannel->println("Snoop: range first > last");
      else if (Res == -3) CmdChannel->printf("Snoop: more than %d pages (256-byte) watched\n", SNOOP_POOL_PAGES);
      else CmdChannel->println("Snoop: bad ranges");
      return;
   }
   SendU16(AckToken);     // before HELLO, which the next SnoopService() sends
   SnoopOut = CmdChannel;
   // Only a host that raised DTR can be seen closing the port (pyserial raises it; some
   // serial libraries leave it down by default, and then the run goes on until stopped).
   SnoopDtrAtStart = (SnoopOut == (Stream *)&Serial) && Serial.dtr();
   SnoopSendEveryMs = Flags ? Flags : (SnoopOut == (Stream *)&Serial ? SnoopUsbDefaultMs : SnoopTcpMaxMs);
#ifdef FeatTCPListen
   SnoopNagle = !Flags && (SnoopOut == (Stream *)&tcpClient || SnoopOut == (Stream *)&SnoopClient);
#endif
   SnoopLastSendMs = millis();
   SnoopLastProbeMs = SnoopLastSendMs - SnoopProbeEveryMs;   // the first probe on the next pass
#ifdef SNOOP_RESID_H
   SnoopSidStart();
#endif
}

#ifdef FeatTCPListen
// The data port: accept a client, run the commands it sends (the same 0x64 0x53 ... as on 2112), drop it when
// it goes.  CmdChannel points at it while a command runs, so SnoopCommand() reads and answers there.
FLASHMEM void SnoopDataPortService()
{
   if (!NetListenEnable) return;   // Ethernet not up (the TeensyROM's own port is not listening either)
   if (!SnoopClient) SnoopClient = SnoopServer.available();
   if (!SnoopClient) return;
   if (!SnoopClient.connected())
   {
      if (SnoopOut == (Stream *)&SnoopClient)
      {
         snoop_stop(&SnoopState, SNOOP_BYE_CHANNEL, micros(), ARM_DWT_CYCCNT);
         SnoopFlushTo(NULL, 0);
         SnoopOut = NULL;
      }
      SnoopClient.stop();
      return;
   }
   while (SnoopClient.available())
   {
      if (SnoopClient.peek() != 0x64) { SnoopClient.read(); continue; }   // anything but a command lead is ignored
      if (SnoopClient.available() < 2) break;            // the token byte on a later pass (GetUInt waits for the rest)
      SnoopClient.read();
      Stream *Was = CmdChannel;
      CmdChannel = &SnoopClient;
      if (SnoopClient.read() == (SnoopToken & 0xFF)) SnoopCommand();
      else
      {
         SendU16(FailToken);
         CmdChannel->println("Snoop data port: only the snoop command (0x64 0x53)");
      }
      CmdChannel = Was;
   }
}
#endif

// From loop(), every pass.
FLASHMEM void SnoopService()
{
#ifdef FeatTCPListen
   SnoopDataPortService();
#endif
   if (!SnoopReady || SnoopOut == NULL) return;
#ifdef FeatTCPListen
   if ((SnoopOut == (Stream *)&tcpClient && !tcpClient.connected()) ||
       (SnoopOut == (Stream *)&SnoopClient && !SnoopClient.connected()))
   {
      snoop_stop(&SnoopState, SNOOP_BYE_CHANNEL, micros(), ARM_DWT_CYCCNT);
      SnoopFlushTo(NULL, 0);
      SnoopOut = NULL;
      return;
   }
#endif
   if (SnoopOut == (Stream *)&Serial && SnoopDtrAtStart && !Serial.dtr())
   {  // the host closed the port: nobody is reading
      snoop_stop(&SnoopState, SNOOP_BYE_CHANNEL, micros(), ARM_DWT_CYCCNT);
      SnoopFlushTo(NULL, 0);
      SnoopOut = NULL;
      return;
   }
#ifdef SNOOP_RESID_H
   SnoopSidFold();
#endif
   snoop_poll(&SnoopState, micros(), ARM_DWT_CYCCNT);
#ifdef SNOOP_RESID_H
   SnoopSidClockTo(SnoopState.c64_time);
#endif
   const uint32_t Now = millis();
   if (Now - SnoopLastProbeMs >= SnoopProbeEveryMs)
   {
      SnoopLastProbeMs = Now;
      snoop_cpu_probe(&SnoopState, SnoopReadCycles, (uint32_t)((uint64_t)F_CPU_ACTUAL * SnoopProbeUs / 1000000u));
   }
   const uint32_t Avail = snoop_out_avail(&SnoopState);
   bool Due = Avail && (Avail >= SnoopBatchBytes || Now - SnoopLastSendMs >= SnoopSendEveryMs);
#ifdef FeatTCPListen
   if (Avail && !Due && SnoopNagle) Due = SnoopOut->availableForWrite() >= FNET_SOCKET_DEFAULT_SIZE;   // nothing in flight
#endif
   if (Due)
   {
      SnoopSend(SnoopOut);
      SnoopLastSendMs = Now;
   }
}

#endif
