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

#define FLASHMEM
#define FASTRUN
#define DMAMEM
#define FeatWriteSnoop
#define FeatTCPListen
#define F_CPU_ACTUAL 600000000u
#define AckToken  0x64CC
#define FailToken 0x9B7F
#define wRegVid_TOD_Clks 7
#define rvtcNTSC 0x01

uint8_t IO1[256];
uint32_t ARM_DWT_CYCCNT = 1000;
volatile uint32_t StartCycCnt;
uint32_t NowUs = 0;
uint8_t BusData;
uint32_t micros() { return NowUs; }
uint32_t millis() { NowUs += 1; return NowUs / 1000; }   // a millis() spin moves time on
uint8_t DataPortWaitRead() { return BusData; }

class Stream {
public:
   std::deque<uint8_t> in;
   std::vector<uint8_t> out;
   int room = 1 << 30;
   virtual ~Stream() {}
   int available() { return (int)in.size(); }
   int read() { if (in.empty()) return -1; int c = in.front(); in.pop_front(); return c; }
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
   bool up = true;
   bool connected() { return up; }
};

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

// the source tree layout of the patched TeensyROM: snoop/ next to the .ino files
#include "../../firmware/WriteSnoop.h"
#include "../../firmware/WriteSnoop.ino"

// what isrPHI2 does with the patch, for one bus cycle
static void Cycle(uint16_t Address, bool R_Wn, uint8_t Data)
{
   ARM_DWT_CYCCNT += 609;               // ~one PAL cycle at 600 MHz
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

static void Loop(int n = 1)
{
   for (int i = 0; i < n; i++) { NowUs += 500; SnoopService(); }
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
      for (int pass = 0; pass < 3; pass++)
      {
         Stream *ch = pass == 0 ? (Stream *)&Serial : (Stream *)&tcpClient;
         const uint8_t ms = pass == 2 ? 50 : 0;
         Command(ch, {0x64, 0x53, 1, 1, 0x18, 0xD4, 0x18, 0xD4, 0, ms});
         ch->bursts.clear();
         const uint32_t t0 = NowUs;
         for (int k = 0; k < 400; k++) { Cycle(0xD418, false, (uint8_t)k); Loop(); }   // 400 writes over 200 ms
         const uint32_t secs_ms = (NowUs - t0) / 1000;
         const size_t n = ch->bursts.size();
         if (pass == 0) CHECK(n >= secs_ms / 20 - 1 && n <= secs_ms / 20 + 1);
         if (pass == 1) CHECK(n >= 80);                                   // a frame every 2 ms or so, sent at once
         if (pass == 2) CHECK(n >= secs_ms / 50 - 1 && n <= secs_ms / 50 + 1);
         for (size_t i = 1; i < n && pass != 1; i++) CHECK(ch->bursts[i] - ch->bursts[i - 1] >= (pass == 0 ? 19000u : 49000u));
         printf("fw_host: %s: %zu sends in %u ms\n", pass == 0 ? "USB serial" : pass == 1 ? "TCP" : "TCP at 50 ms", n, secs_ms);
         Command(ch, {0x64, 0x53, 0});
      }
   }

   if (Fails) { fprintf(stderr, "fw_host: %d failed\n", Fails); return 1; }
   printf("fw_host: ok\n");
   return 0;
}
