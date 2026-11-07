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

// 0x64 0x53, then: 0 stop | 1 start n {first lo,hi, last lo,hi}*n gap flags | 2 status
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
   snoop_poll(&SnoopState, micros(), ARM_DWT_CYCCNT);
   const uint32_t Now = millis();
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
