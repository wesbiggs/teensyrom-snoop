// teensyrom-snoop: the main-loop half.  Command 0x6453 from the USB serial or TCP channel, and
// SnoopService() from loop().  Protocol: docs/PROTOCOL.md in teensyrom-snoop.

#ifdef FeatWriteSnoop

#include "snoop/snoop.c"

bool SnoopReady = false;
bool SnoopDtrAtStart = false;   // a USB serial run whose host had DTR up: stop when it drops (port closed)

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
   uint8_t Buf[512];
   while (snoop_out_avail(&SnoopState))
   {
      int Room = Out->availableForWrite();
      if (Room <= 0) return;
      uint32_t n = snoop_out_take(&SnoopState, Buf, Room < (int)sizeof Buf ? (uint32_t)Room : sizeof Buf);
      Out->write(Buf, n);
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
   if (!SnoopGetByte(&Gap) || !SnoopGetByte(&Flags)) return;
   (void)Flags;

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
}

// From loop(), every pass.
FLASHMEM void SnoopService()
{
   if (!SnoopReady || SnoopOut == NULL) return;
#ifdef FeatTCPListen
   if (SnoopOut == &tcpClient && !tcpClient.connected())
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
   SnoopSend(SnoopOut);
}

#endif
