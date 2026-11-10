// The firmware glue (firmware/WriteSnoop.h + .ino) compiled as C++ against host stubs of the
// TeensyROM pieces it uses.  Runs a scenario through SnoopCommand / the ISR tap / SnoopService
// and writes each channel's bytes to build/ for tests/test_fw.py to decode.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <deque>
#include <vector>
#include <string>
#include <algorithm>

#define FLASHMEM
#define FASTRUN
#define DMAMEM
#define PSTR(s) (s)
#define FeatWriteSnoop
#define FeatTCPListen
#define F_CPU_ACTUAL 600000000u
#define AckToken  0x64CC
#define FailToken 0x9B7F
#define wRegVid_TOD_Clks 7
#define rvtcNTSC 0x01
#define FNET_SOCKET_DEFAULT_SIZE 1024 * 2   // as NativeEthernet.h (no parentheses there either)

uint8_t IO1[256];
uint32_t ARM_DWT_CYCCNT = 1000;
volatile uint32_t StartCycCnt;
uint32_t NowUs = 0;
uint8_t BusData;
uint32_t micros() { return NowUs; }
uint32_t millis() { NowUs += 1; return NowUs / 1000; }   // a millis() spin moves time on
uint8_t DataPortWaitRead() { return BusData; }
// the CPU probe's counter reads: 6 cycles each, and an interrupt of 300 cycles every 50 reads
static uint32_t ProbeReads = 0;
static uint32_t HostCycles() { ARM_DWT_CYCCNT += 6; if (++ProbeReads % 50 == 0) ARM_DWT_CYCCNT += 300; return ARM_DWT_CYCCNT; }
#define SnoopCycles() HostCycles()

class Stream {
public:
   std::deque<uint8_t> in;
   std::vector<uint8_t> out;
   int room = 1 << 30;
   virtual ~Stream() {}
   int available() { return (int)in.size(); }
   int read() { if (in.empty()) return -1; int c = in.front(); in.pop_front(); return c; }
   int peek() { return in.empty() ? -1 : in.front(); }
   int availableForWrite() { return room; }
   size_t write(uint8_t b) { out.push_back(b); return 1; }
   std::vector<uint32_t> bursts;   // NowUs of each write(p, n) call: one USB transfer each on the Teensy
   size_t write(const uint8_t *p, size_t n)
   {
      if ((int)n > room) { fprintf(stderr, "wrote past availableForWrite\n"); exit(1); }
      bursts.push_back(NowUs);
      out.insert(out.end(), p, p + n); return n;
   }
   void println(const char *s) { while (*s) write((uint8_t)*s++); write('\n'); }
   void printf(const char *f, ...)
   {
      char b[256]; va_list ap; va_start(ap, f); vsnprintf(b, sizeof b, f, ap); va_end(ap);
      for (char *p = b; *p; p++) write((uint8_t)*p);
   }
   void feed(const std::vector<uint8_t> &v) { in.insert(in.end(), v.begin(), v.end()); }
};

class EthernetClient : public Stream {
public:
   bool up = false;
   bool assigned = false;          // operator bool: holds a socket (a new EthernetClient holds none, as NativeEthernet's)
   bool connected() { return up; }
   operator bool() { return assigned; }
   void stop() { assigned = up = false; }
};

// NativeEthernet's server, as far as the glue uses it: available() hands over a client that has sent something
class EthernetServer {
public:
   EthernetClient pending;
   bool has = false;
   int calls = 0;
   explicit EthernetServer(uint16_t) { pending.assigned = pending.up = false; }
   EthernetClient available()
   {
      calls++;
      if (has) { has = false; return pending; }
      EthernetClient none;
      none.assigned = none.up = false;
      return none;
   }
   void offer(const std::vector<uint8_t> &bytes)   // a client connects and sends these
   {
      pending = EthernetClient();
      pending.assigned = pending.up = true;
      pending.feed(bytes);
      has = true;
   }
};
bool NetListenEnable = true;

class UsbSerial : public Stream {
public:
   bool line = true;
   uint8_t dtr() { return line; }
};

UsbSerial Serial;
EthernetClient tcpClient;
Stream *CmdChannel = &Serial;

// as in SerUSBIO.ino (no wait: the stub has every byte already, or none)
bool SerialAvailabeTimeout() { return CmdChannel->available() > 0; }
void SendU16(uint16_t v) { CmdChannel->write((uint8_t)v); CmdChannel->write((uint8_t)(v >> 8)); }
bool GetUInt(uint32_t *InVal, uint8_t NumBytes)
{
   *InVal = 0;
   for (int8_t ByteNum = NumBytes - 1; ByteNum >= 0; ByteNum--)
   {
      if (!SerialAvailabeTimeout()) { SendU16(FailToken); CmdChannel->printf("Timeout!\n"); return false; }
      *InVal += ((uint32_t)CmdChannel->read() << (ByteNum * 8));
   }
   return true;
}

// The USB audio driver (src/USBHost_t36/USBAudioOut.h on the Teensy) as far as the glue uses it: a device can be
// "plugged in", and Frame() asks the glue's fill function for one 1 ms packet as the USBHS interrupt would.
#define SNOOP_AUDIO_STUB
#include "../../firmware/usbhost/isoaudio.c"
typedef void (*usbhost_iso_fill_t)(void *ctx, uint8_t *packet, uint32_t len);
typedef struct { uint32_t frames, errors, missed, status, shift; uint8_t running; } usbhost_iso_stats_t;
class USBHost {};
USBHost myusbHost;
class USBAudioOut {
public:
   enum { RATE = 48000, FRAME_BYTES = RATE / 1000 * 4 };
   struct Info {
      bool attached, streaming;
      uint16_t vid, pid;
      uint8_t iface, alt, ep, sync, fu;
      int8_t step;
      uint8_t failed_step;
      int16_t volume, vol_min, vol_max;
      bool unmuted;
      int last_reject;
      usbhost_iso_stats_t iso;
   };
   Info state;
   usbhost_iso_fill_t fill;
   void *ctx;
   USBAudioOut(USBHost &, usbhost_iso_fill_t f, void *c = NULL) : fill(f), ctx(c) { memset(&state, 0, sizeof state); }
   void info(Info *i) { *i = state; }
   void plug()   // the owner's dongle, set up as USBAudioOut does it (tests/test_usbaudio.py has its descriptors)
   {
      state.attached = state.streaming = true;
      state.vid = 0x0020; state.pid = 0x0B21; state.iface = 2; state.alt = 1; state.ep = 0x03; state.sync = 3;
      state.fu = 2; state.step = 7; state.vol_min = -45 * 256; state.vol_max = 0; state.unmuted = true;
   }
   void unplug() { state.attached = state.streaming = false; }
   std::vector<uint8_t> Frame()
   {
      std::vector<uint8_t> p(FRAME_BYTES + 4, 0xEE);   // and a guard: exactly FRAME_BYTES are written
      fill(ctx, p.data(), FRAME_BYTES);
      if (p[FRAME_BYTES] != 0xEE) { fprintf(stderr, "fill wrote past the packet\n"); exit(1); }
      p.resize(FRAME_BYTES);
      if (state.streaming) state.iso.frames++;
      return p;
   }
};

// the source tree layout of the patched TeensyROM: snoop/ next to the .ino files
#include "../../firmware/WriteSnoop.h"
#include "../../firmware/WriteSnoop.ino"

static uint32_t LateBy = 0;   // the next Cycle() comes this many CPU cycles late (isrPHI2's late branch)

// what isrPHI2 does with the patch, for one bus cycle
static void Cycle(uint16_t Address, bool R_Wn, uint8_t Data)
{
   ARM_DWT_CYCCNT += 609;               // ~one PAL cycle at 600 MHz
   if (LateBy) { ARM_DWT_CYCCNT += LateBy; snoop_isr_late(&SnoopHot, 609 + LateBy); LateBy = 0; }
   StartCycCnt = ARM_DWT_CYCCNT;
   BusData = Data;
   if (!R_Wn && SnoopHot.page[Address >> 8]) SnoopTap(Address);
}

// ProcessCommand: lead byte 0x64 then the token's second byte
static void Command(Stream *ch, const std::vector<uint8_t> &bytes)
{
   ch->feed(bytes);
   CmdChannel = ch;
   if (ch->read() != 0x64 || ch->read() != 0x53) { fprintf(stderr, "bad command lead\n"); exit(1); }
   SnoopCommand();
}

#ifdef SNOOP_RESID_H
// What the SID consumer did, in order: a write (a < 0x10000) or the end of a loop pass's emulation (a = 0x10000).
// reSID's filter integrates in steps of up to 8 cycles, so the same span split differently gives slightly
// different samples: the reference render replays these split points to compare exactly.
struct SidEvent { uint64_t t; uint32_t a; uint8_t d; };
static std::vector<SidEvent> SidLog;
static bool SidLogging = false;
#endif

static void Loop(int n = 1)
{
   for (int i = 0; i < n; i++)
   {
      NowUs += 500;
      SnoopService();
#ifdef SNOOP_RESID_H
      if (SidLogging) SidLog.push_back({SnoopSidAt, 0x10000, 0});
#endif
   }
}

// the text a channel got; one channel of 16-bit stereo packets
static std::string Text(const Stream &s) { return std::string(s.out.begin(), s.out.end()); }
static std::vector<int16_t> Samples(const std::vector<uint8_t> &p, int ch)
{
   std::vector<int16_t> v;
   for (size_t i = 0; i + 3 < p.size(); i += 4) v.push_back((int16_t)(p[i + 2 * ch] | p[i + 2 * ch + 1] << 8));
   return v;
}

static int Fails = 0;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); Fails++; } } while (0)

static void Dump(const char *path, const Stream &s)
{
   FILE *f = fopen(path, "wb");
   if (!f) { perror(path); exit(1); }
   fwrite(s.out.data(), 1, s.out.size(), f);
   fclose(f);
}

int main()
{
   tcpClient.assigned = tcpClient.up = true;   // a client on 2112
   // 1. serial: start SID $D400-$D41C, gap 32; a register dump, reads and other writes ignored
   Command(&Serial, {0x64, 0x53, 1, 1, 0x00, 0xD4, 0x1C, 0xD4, 32, 0});
   CHECK(Serial.out.size() == 2 && Serial.out[0] == 0xCC && Serial.out[1] == 0x64);
   Loop();
   for (int k = 0; k <= 0x18; k++)
   {
      Cycle(0xD400 + k, true, 0x55);       // a read: ignored
      Cycle(0x0400 + k, false, 0x20);      // screen write: ignored
      Cycle(0xD400 + k, false, (uint8_t)(0x40 + k));
   }
   Loop(10);
   Serial.room = 0;                        // host stops reading: nothing more is written
   for (int k = 0; k < 40; k++) Cycle(0xD418, false, (uint8_t)k);
   Loop(10);
   Serial.room = 1 << 30;
   Loop(10);
   Command(&Serial, {0x64, 0x53, 2});      // status
   Loop();
   // 2. a start over TCP replaces the serial run: serial gets BYE(replaced)
   Command(&tcpClient, {0x64, 0x53, 1, 2, 0x18, 0xD4, 0x18, 0xD4, 0x00, 0xDE, 0x00, 0xDE, 0, 0});
   Dump("build/fw_serial.bin", Serial);
   for (int k = 0; k < 5; k++) { Cycle(0xD418, false, (uint8_t)(0xF0 + k)); Cycle(0xDE00, false, (uint8_t)k); }
   Loop(10);
   // 2b. a refused start from another channel leaves the TCP run alone
   {
      Stream Third;
      Command(&Third, {0x64, 0x53, 1, 1, 0x00, 0x00, 0xFF, 0xFF, 0, 0});
      CHECK(Third.out.size() >= 2 && Third.out[0] == 0x7F && Third.out[1] == 0x9B);
      CHECK(SnoopState.running && SnoopOut == &tcpClient);
   }
   Cycle(0xD418, false, 0xAA);
   Loop(10);
   // 3. the TCP client goes away: the run stops by itself
   tcpClient.up = false;
   Loop();
   CHECK(!SnoopState.running && SnoopOut == NULL);
   Cycle(0xD418, false, 1);
   CHECK(SnoopHot.head == SnoopHot.tail);
   Dump("build/fw_tcp.bin", tcpClient);
   // 4. refusals
   Stream Other;
   Command(&Other, {0x64, 0x53, 1, 1, 0x1C, 0xD4, 0x00, 0xD4, 32, 0});    // first > last
   Command(&Other, {0x64, 0x53, 1, 1, 0x00, 0x00, 0xFF, 0xFF, 32, 0});    // 256 pages
   Command(&Other, {0x64, 0x53, 1, 0});                                    // no ranges
   Command(&Other, {0x64, 0x53, 9});                                       // bad op
   Command(&Other, {0x64, 0x53, 1, 1, 0x00});                              // cut short
   std::string txt(Other.out.begin(), Other.out.end());
   CHECK(txt.find("range first > last") != std::string::npos);
   CHECK(txt.find("more than 16 pages") != std::string::npos);
   CHECK(txt.find("1-16 ranges") != std::string::npos);
   CHECK(txt.find("unknown op") != std::string::npos);
   CHECK(txt.find("Timeout!") != std::string::npos);
   CHECK(!SnoopState.running);
   // 5. NTSC clock, and stop over the same channel
   IO1[wRegVid_TOD_Clks] = rvtcNTSC;
   Stream Ntsc;
   Command(&Ntsc, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});
   Loop();
   LateBy = 600;                           // one interrupt ~1 us late: counted in all three
   Cycle(0xD418, false, 0x0F);
   Loop();
   Command(&Ntsc, {0x64, 0x53, 0});
   CHECK(!SnoopState.running && SnoopOut == NULL);
   Dump("build/fw_ntsc.bin", Ntsc);
   // 6. USB serial: the host closes the port (DTR drops) -> the run stops; DTR never up -> it runs on
   IO1[wRegVid_TOD_Clks] = 0;
   Serial.out.clear();
   Command(&Serial, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});
   Loop();
   CHECK(SnoopState.running);
   Serial.line = false;
   Loop();
   CHECK(!SnoopState.running && SnoopOut == NULL);
   Command(&Serial, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});
   Loop(3);
   CHECK(SnoopState.running);
   Command(&Serial, {0x64, 0x53, 0});
   CHECK(!SnoopState.running);
   // 7. batching: USB serial sends at most every 20 ms (each write is a USB transfer with interrupts off),
   //    TCP at once, and the start command's last byte sets the interval
   {
      tcpClient.up = true;
      for (int pass = 0; pass < 4; pass++)
      {
         Stream *ch = pass == 0 ? (Stream *)&Serial : (Stream *)&tcpClient;
         const uint8_t ms = pass == 2 ? 50 : 0;
         tcpClient.room = pass == 3 ? 1500 : 1 << 30;   // pass 3: earlier data unacknowledged, Nagle waits
         Command(ch, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, ms});
         ch->bursts.clear();
         const uint32_t t0 = NowUs;
         for (int k = 0; k < 400; k++) { Cycle(0xD418, false, (uint8_t)k); Loop(); }   // 400 writes over 200 ms
         const uint32_t secs_ms = (NowUs - t0) / 1000;
         const size_t n = ch->bursts.size();
         if (pass == 0) CHECK(n >= secs_ms / 20 - 1 && n <= secs_ms / 20 + 1);
         if (pass == 1) CHECK(n >= 80);                                   // a frame every 2 ms or so, sent at once
         if (pass == 2) CHECK(n >= secs_ms / 50 - 1 && n <= secs_ms / 50 + 1);
         if (pass == 3) CHECK(n >= secs_ms / 20 - 1 && n <= secs_ms / 20 + 1);
         for (size_t i = 1; i < n && pass != 1; i++) CHECK(ch->bursts[i] - ch->bursts[i - 1] >= (pass == 2 ? 49000u : 19000u));
         printf("fw_host: %s: %zu sends in %u ms\n", pass == 0 ? "USB serial" : pass == 1 ? "TCP, all acknowledged" :
                pass == 2 ? "TCP at 50 ms" : "TCP, data in flight", n, secs_ms);
         Command(ch, {0x64, 0x53, 0});
      }
   }

   // 8. the data port (6581): start and stream there, 2112 stays free, junk and other commands refused there
   {
      tcpClient = EthernetClient();                     // a client on 2112, idle
      tcpClient.assigned = tcpClient.up = true;
      NetListenEnable = false;
      SnoopServer.offer({0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});
      Loop(3);
      CHECK(!SnoopState.running && SnoopServer.has);   // Ethernet not up: not even listening
      NetListenEnable = true;
      Loop();
      CHECK(SnoopState.running && SnoopOut == (Stream *)&SnoopClient);
      CHECK(SnoopClient.out.size() >= 2 && SnoopClient.out[0] == 0xCC && SnoopClient.out[1] == 0x64);
      for (int k = 0; k < 6; k++) { Cycle(0xD418, false, (uint8_t)(0x30 + k)); Loop(); }
      Command(&tcpClient, {0x64, 0x53, 2});            // a command on 2112: answered there, the stream stays put
      CHECK(tcpClient.out.size() == 2 && tcpClient.out[0] == 0xCC);
      CHECK(SnoopOut == (Stream *)&SnoopClient);
      SnoopClient.feed({0x00, 0x64});                  // junk, then a lead whose token comes later
      Loop();
      CHECK(SnoopClient.available() == 1);
      SnoopClient.feed({0x55});                        // ping: not served on the data port
      Loop();
      std::string txt(SnoopClient.out.begin(), SnoopClient.out.end());
      CHECK(txt.find("only the snoop command") != std::string::npos);
      SnoopClient.feed({0x64, 0x53, 2});               // status over the data port
      for (int k = 0; k < 4; k++) { Cycle(0xD418, false, (uint8_t)(0x40 + k)); Loop(); }
      Loop(10);
      Dump("build/fw_dataport.bin", SnoopClient);
      SnoopClient.up = false;                          // the client goes: the run stops, the slot frees
      Loop();
      CHECK(!SnoopState.running && SnoopOut == NULL && !SnoopClient);
      SnoopServer.offer({0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});   // and the next one is taken
      Loop();
      CHECK(SnoopState.running && SnoopOut == (Stream *)&SnoopClient);
      Command(&SnoopClient, {0x64, 0x53, 0});
      CHECK(!SnoopState.running);
   }

   // 10. the USB audio dongle: 64 53 03 mode switches the source between silence and the 1 kHz tone, and reports
   {
      Stream Au;
      Command(&Au, {0x64, 0x53, 3, 2});                   // report: nothing plugged in
      CHECK(Au.out.size() > 2 && Au.out[0] == 0xCC && Au.out[1] == 0x64);
      CHECK(Text(Au).find("Snoop audio: silence; no USB audio device\n") != std::string::npos);
      std::vector<uint8_t> p = SnoopDongle.Frame();
      CHECK(p == std::vector<uint8_t>(192, 0));          // silence, all the same

      SnoopDongle.plug();
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 1});                   // tone on
      std::string t = Text(Au);
      CHECK(Au.out[0] == 0xCC && Au.out[1] == 0x64);
      CHECK(t.find("Snoop audio: tone 1000 Hz; 0020:0B21 interface 2 alt 1 ep 03, streaming; frames 0, errors 0, "
                   "missed 0; volume 0.0 dB (-45.0..0.0), unmuted\n") != std::string::npos);
      // one second of frames: 192 bytes each, left = right, 1000 Hz at -12 dBFS, the same as one continuous tone
      isoaudio_tone_t Ref;
      isoaudio_tone_init(&Ref, 1000, 48000, 8192);
      std::vector<uint8_t> All, Want(192 * 1000);
      for (int f = 0; f < 1000; f++)
      {
         p = SnoopDongle.Frame();
         CHECK(p.size() == 192);
         All.insert(All.end(), p.begin(), p.end());
         if (f == 499)
         {  // a second "on" mid-way leaves the phase alone
            Command(&Au, {0x64, 0x53, 3, 1});
         }
      }
      isoaudio_tone_packet(&Ref, Want.data(), 48 * 1000);
      CHECK(All == Want);
      std::vector<int16_t> L = Samples(All, 0), R = Samples(All, 1);
      CHECK(L == R);
      int Ups = 0, Peak = 0;
      for (size_t i = 1; i < L.size(); i++) if (L[i - 1] < 0 && L[i] >= 0) Ups++;
      for (int16_t v : L) Peak = std::max(Peak, (int)std::abs(v));
      CHECK(Ups >= 999 && Ups <= 1000);
      CHECK(Peak >= 8190 && Peak <= 8192);
      FILE *f = fopen("build/fw_tone.raw", "wb");   // for tests/test_fw.py: the spectrum
      if (f) { fwrite(All.data(), 1, All.size(), f); fclose(f); }
      printf("fw_host: tone: %zu frames of 192 bytes, %d periods, peak %d\n", All.size() / 192, Ups, Peak);

      // the tone runs on during a capture, and the command works on the capture's own channel
      Command(&Serial, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, 0});
      Loop();
      Command(&Serial, {0x64, 0x53, 3, 2});
      CHECK(SnoopState.running && SnoopOut == (Stream *)&Serial);
      CHECK(Samples(SnoopDongle.Frame(), 0) != std::vector<int16_t>(48, 0));
      Command(&Serial, {0x64, 0x53, 0});

      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 0});                   // tone off: silence
      CHECK(Text(Au).find("Snoop audio: silence; 0020:0B21") != std::string::npos);
      CHECK(SnoopDongle.Frame() == std::vector<uint8_t>(192, 0));
      // volumes in dB with one decimal: the dongle's range, a refused rate request
      SnoopDongle.state.volume = -1 * 256 - 128; SnoopDongle.state.vol_min = -15360; SnoopDongle.state.vol_max = 6 * 256 + 13;
      SnoopDongle.state.failed_step = 2;
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 2});
      CHECK(Text(Au).find("; volume -1.5 dB (-60.0..6.1), unmuted; SET_CUR rate failed\n") != std::string::npos);
      SnoopDongle.state.volume = -12; SnoopDongle.state.failed_step = 0;   // -0.05 dB rounds to 0.0, not -0.0
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 2});
      CHECK(Text(Au).find("; volume 0.0 dB (") != std::string::npos);
      SnoopDongle.state.volume = 0; SnoopDongle.state.vol_min = -45 * 256; SnoopDongle.state.vol_max = 0;
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 7});                   // bad mode: refused, nothing changes
      CHECK(Au.out[0] == 0x7F && Au.out[1] == 0x9B && Text(Au).find("mode 0 silence, 1 tone, 2 report") != std::string::npos);
#ifndef SNOOP_RESID_H
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 3, 10});               // the SID: refused without reSID, its byte still read
      CHECK(Au.out[0] == 0x7F && Au.out[1] == 0x9B && Text(Au).find("no SID emulator in this build") != std::string::npos);
      CHECK(SnoopAudioSource == NULL);
      CHECK(!Au.available());
#endif
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3});                      // cut short
      CHECK(Text(Au).find("Timeout!") != std::string::npos);
      SnoopDongle.unplug();
      SnoopDongle.state.last_reject = ISOAUDIO_ASYNC;      // an asynchronous one was offered meanwhile
      Au.out.clear();
      Command(&Au, {0x64, 0x53, 3, 2});
      CHECK(Text(Au).find("no USB audio device (one was offered: asynchronous, not supported)") != std::string::npos);
   }

#ifdef SNOOP_RESID_H
   // 9. the SID emulator (a tree with reSID; make test-sid): each write applied on its cycle, the run
   //    emulated up to "now", the samples equal to reSID driven directly with the same writes
   {
      static std::vector<short> Got;
      SnoopSidSink = [](const short *p, int n) { Got.insert(Got.end(), p, p + n); };
      IO1[wRegVid_TOD_Clks] = 0;
      Stream Sid;
      SnoopDongle.plug();
      Command(&Sid, {0x64, 0x53, 3, 3, 0});               // the dongle plays the SID, 10 ms buffer (0 = default)
      CHECK(Text(Sid).find("Snoop audio: SID (reSID 6581), 10 ms buffer, no capture running; 0020:0B21") != std::string::npos);
      CHECK(SnoopAudioSource == SnoopSidSource && SnoopSidRing.target == 480 && SnoopSidRing.trim_above == 1440);
      CHECK(SnoopDongle.Frame() == std::vector<uint8_t>(192, 0));   // nothing rendered yet: silence
      // What the dongle plays: packets taken whenever 10 ms + a packet are buffered (the interrupt keeps up)
      static std::vector<short> Played;
      auto Drain = [&]() {
         while (isoaudio_ring_fill(&SnoopSidRing) >= 480 + 48)
         {
            std::vector<uint8_t> p = SnoopDongle.Frame();
            std::vector<int16_t> L = Samples(p, 0), R = Samples(p, 1);
            CHECK(L == R);
            Played.insert(Played.end(), L.begin(), L.end());
         }
      };
      Sid.out.clear();
      Command(&Sid, {0x64, 0x53, 1, 1, 0x00, 0xD4, 0x1C, 0xD4, 0, 0});
      CHECK(SnoopState.on_write == SnoopSidWrite);
      SnoopState.on_write = [](void *c, uint64_t t, uint16_t a, uint8_t d) { SidLog.push_back({t, a, d}); SnoopSidWrite(c, t, a, d); };
      SidLogging = true;
      const std::vector<std::pair<uint16_t, uint8_t>> Tune = {
         {0xD418, 0x0F}, {0xD405, 0x09}, {0xD406, 0xF0}, {0xD400, 0xD6}, {0xD401, 0x1C}, {0xD404, 0x11},
         {0xD402, 0x00}, {0xD403, 0x08}, {0xD404, 0x41}, {0xD401, 0x0E}, {0xD417, 0xF1}, {0xD416, 0x40},
         {0xD418, 0x1F}, {0xD404, 0x40}, {0xD41A, 0x55}, {0xD020, 0x01}};   // $D41A: read-only; $D020: not watched
      size_t Extra = 0;
      for (size_t k = 0; k < Tune.size(); k++)
      {
         Cycle(Tune[k].first, false, Tune[k].second);
         if (k % 3 == 2 && Tune[k].first < 0xD41C) { Cycle(Tune[k].first + 1, false, 0x00); Extra++; }   // 1 cycle apart
         ARM_DWT_CYCCNT += 609u * (2000u + 997u * (uint32_t)k);   // the bus idles: up to ~17000 cycles
         Loop(1 + (int)(k % 4));
         Drain();
      }
      Loop(20);
      Drain();
      Command(&Sid, {0x64, 0x53, 0});
      Dump("build/fw_sid.bin", Sid);
      // reSID driven directly with what the consumer was given, to the time it emulated
      reSID::SID Ref;
      Ref.set_chip_model(reSID::MOS6581);
      Ref.set_sampling_parameters(SnoopPhi2Hz(), reSID::SAMPLE_FAST, SnoopSidRate);
      Ref.reset();
      std::vector<short> Want;
      short Buf[256];
      uint64_t At = 0;
      auto ClockTo = [&](uint64_t t) {
         reSID::cycle_count d = (reSID::cycle_count)(t - At);
         while (d > 0) { int n = Ref.clock(d, Buf, 256); Want.insert(Want.end(), Buf, Buf + n); }
         At = t;
      };
      SidLogging = false;
      size_t Writes = 0;
      for (const SidEvent &e : SidLog)
      {
         if (e.a == 0x10000) { ClockTo(e.t); continue; }
         Writes++;
         if (e.a - 0xD400u < 0x20u) { ClockTo(e.t); Ref.write(e.a & 0x1F, e.d); }
      }
      ClockTo(SnoopSidAt);
      int Peak = 0;
      for (short v : Got) Peak = std::max(Peak, v < 0 ? -(int)v : (int)v);
      CHECK(Writes == Tune.size() - 1 + Extra);   // all but $D020 ($D41A is in the range: reSID ignores it)
      if (Got != Want)
      {
         size_t i = 0;
         while (i < Got.size() && i < Want.size() && Got[i] == Want[i]) i++;
         fprintf(stderr, "SID: got %zu samples, want %zu, first difference at %zu\n", Got.size(), Want.size(), i);
      }
      CHECK(Got.size() > 48000u / 10);
      CHECK(Got == Want);
      CHECK(Peak > 1000);
      printf("fw_host: SID: %zu writes, %zu samples (%.2f s), peak %d, equal to a direct render: %s\n",
             Writes, Got.size(), Got.size() / 48000.0, Peak, Got == Want ? "yes" : "NO");
      // the dongle got the same samples, in order, from the first one, in whole packets
      CHECK(Played.size() >= Got.size() - 480 - 48 && Played.size() % 48 == 0);
      CHECK(std::equal(Played.begin(), Played.end(), Got.begin()));
      CHECK(SnoopSidUnderruns == 0 && SnoopSidRing.trimmed == 0 && SnoopSidRing.dropped == 0);
      Stream Au;
      Command(&Au, {0x64, 0x53, 3, 2});
      const std::string t = Text(Au);
      CHECK(t.find("Snoop audio: SID (reSID 6581), 10 ms buffer, no capture running; 0020:0B21") != std::string::npos);
      CHECK(t.find("; SID buffered ") != std::string::npos && t.find(" ms, underruns 0, trimmed 0.0 ms, dropped 0.0 ms\n") != std::string::npos);
      // the rest plays out, then the last sample is held (no click); with no capture running that is no underrun
      while (isoaudio_ring_fill(&SnoopSidRing) >= 48) SnoopDongle.Frame();
      std::vector<int16_t> Tail = Samples(SnoopDongle.Frame(), 0);
      CHECK(Tail.back() == Got.back() && SnoopSidUnderruns == 0);
      CHECK(Samples(SnoopDongle.Frame(), 0) == std::vector<int16_t>(48, Got.back()));
      // a second capture: the counts start again; a packet that finds the buffer short during it is an underrun,
      // and the next pass puts it, the lowest fill and the trimmed samples into STATUS (version 8)
      SnoopSidRing.trimmed = 77;
      SnoopSidUnderruns = 5;
      Command(&Au, {0x64, 0x53, 1, 1, 0x00, 0xD4, 0x1C, 0xD4, 0, 0});
      CHECK(SnoopSidUnderruns == 0 && SnoopSidRing.trimmed == 0 && SnoopState.audio_low == SNOOP_AUDIO_NONE);
      Cycle(0xD418, false, 0x0F);
      ARM_DWT_CYCCNT += 609u * 30000u;                  // ~29 ms of C64 time
      Loop();
      CHECK(isoaudio_ring_fill(&SnoopSidRing) > 1440);  // above target + 20 ms: the next packet trims
      SnoopDongle.Frame();
      const uint32_t Trimmed = SnoopSidRing.trimmed;
      CHECK(Trimmed > 0 && isoaudio_ring_fill(&SnoopSidRing) == 480 - 48);
      for (int k = 0; k < 10; k++) SnoopDongle.Frame(); // 9 take the 432 left, the 10th is short
      CHECK(SnoopSidUnderruns == 1);
      Loop();
      CHECK(SnoopState.audio_under == 1 && SnoopState.audio_trim == Trimmed && SnoopState.audio_low == 0);
      Command(&Au, {0x64, 0x53, 2});                      // STATUS now: sent, then cleared
      CHECK(SnoopState.audio_under == 0 && SnoopState.audio_trim == 0 && SnoopState.audio_low == SNOOP_AUDIO_NONE);
      Command(&Au, {0x64, 0x53, 0});
      Dump("build/fw_sid_dongle.bin", Au);
      // a new buffer size: the ring starts again; mode 0 stops the feed
      Command(&Au, {0x64, 0x53, 3, 3, 200});              // clamped to 60 ms
      CHECK(SnoopSidMs == 60 && SnoopSidRing.target == 2880 && SnoopSidRing.trim_above == 5760 && SnoopSidUnderruns == 0);
      Command(&Au, {0x64, 0x53, 3, 0});
      CHECK(SnoopAudioSource == NULL && !SnoopSidFeed);
      SnoopDongle.unplug();
      printf("fw_host: SID to the dongle: %zu samples in %zu packets, equal to reSID's output\n", Played.size(), Played.size() / 48);
   }
#endif

   if (Fails) { fprintf(stderr, "fw_host: %d failed\n", Fails); return 1; }
   printf("fw_host: ok\n");
   return 0;
}
