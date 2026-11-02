// teensyrom-snoop: the phi2-ISR half.  Included by Teensy.ino just before MinimalBoot/Common/ISRs.c
// (main image only: FeatWriteSnoop is defined in TeensyROM.h).
//
// isrPHI2 calls SnoopTap() on a C64 write whose page is in the map:
//    if (!R_Wn && SnoopHot.page[Address >> 8]) SnoopTap(Address);
// That test is the whole cost on every other cycle (one byte load from DTCM on a write, nothing on a
// read).  SnoopTap waits for the data bus (nS_DataSetup after phi2 rise, as every IO1/IO2 write
// handler does), so only watched writes pay that wait.
#ifdef FeatWriteSnoop

#define SnoopToken 0x6453   // 'd' 'S': start / stop / status, see docs/PROTOCOL.md

#define SNOOP_COLD FLASHMEM
#include "snoop/snoop.h"

snoop_hot_t SnoopHot;             // plain global: DTCM, the ISR's fast RAM (~4.9 KB)
DMAMEM snoop_t SnoopState;        // the main loop's half: RAM2 (~9 KB)
Stream *SnoopOut = NULL;          // the channel that started the run

FASTRUN __attribute__((noinline)) void SnoopTap(uint16_t Address)
{
   if (!snoop_isr_match(&SnoopHot, Address)) return;
   snoop_isr_store(&SnoopHot, Address, DataPortWaitRead(), StartCycCnt);
}

#endif
