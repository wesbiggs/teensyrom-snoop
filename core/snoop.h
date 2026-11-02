/* teensyrom-snoop: capture C64 bus writes to chosen addresses and stream them.
 *
 * Portable C99, also compiled as C++ by the TeensyROM firmware (keep it C++-clean).
 *
 * Two halves:
 *   - the ISR side (snoop_hot_t, snoop_isr_match / snoop_isr_store): a two-level address filter and
 *     a single-producer ring of {stamp, address, data}.  This is all the phi2 interrupt touches;
 *     keep it small and in fast RAM (DTCM on the Teensy).
 *   - the main-loop side (snoop_t, snoop_poll): drains the ring, converts CPU-clock stamps to C64
 *     cycles, groups writes to consecutive addresses into blocks {dt, addr, len, bytes} and frames
 *     them (COBS + CRC-16) into an output FIFO that the glue sends without blocking.
 *
 * Wire format: docs/PROTOCOL.md.
 */
#ifndef SNOOP_H
#define SNOOP_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNOOP_PROTO_VERSION 1

#ifndef SNOOP_MAX_RANGES
#define SNOOP_MAX_RANGES   16     /* address ranges per start command */
#endif
#ifndef SNOOP_POOL_PAGES
#define SNOOP_POOL_PAGES   16     /* distinct 256-byte pages that may hold watched addresses */
#endif
#ifndef SNOOP_RING
#define SNOOP_RING         512    /* ISR ring entries, power of two (8 bytes each) */
#endif
#ifndef SNOOP_FRAME_MAX
#define SNOOP_FRAME_MAX    512    /* largest frame payload before COBS/CRC */
#endif
#ifndef SNOOP_OUT_SIZE
#define SNOOP_OUT_SIZE     8192   /* output FIFO (framed bytes waiting for the channel), power of two */
#endif

#if (SNOOP_RING & (SNOOP_RING - 1)) != 0 || (SNOOP_OUT_SIZE & (SNOOP_OUT_SIZE - 1)) != 0
#error "SNOOP_RING and SNOOP_OUT_SIZE must be powers of two"
#endif

/* frame types (first payload byte) */
enum {
   SNOOP_F_HELLO  = 0x01,
   SNOOP_F_WRITES = 0x02,
   SNOOP_F_STATUS = 0x03,
   SNOOP_F_BYE    = 0x04
};

/* BYE reasons */
enum {
   SNOOP_BYE_STOPPED  = 0,   /* stop command */
   SNOOP_BYE_REPLACED = 1,   /* a new start command took over */
   SNOOP_BYE_CHANNEL  = 2    /* the channel went away (glue decides) */
};

typedef struct {
   uint32_t stamp;   /* CPU cycle counter at phi2 rise */
   uint16_t addr;
   uint8_t  data;
   uint8_t  pad;
} snoop_ev_t;

typedef struct {
   uint16_t first, last;   /* inclusive */
} snoop_range_t;

/* Everything the phi2 ISR touches. */
typedef struct {
   uint8_t           page[256];                      /* 0: page not watched, else 1 + pool index */
   uint32_t          bits[SNOOP_POOL_PAGES][8];      /* one bit per address in the page */
   volatile uint32_t head;                           /* written by the ISR only */
   volatile uint32_t tail;                           /* written by the main loop only */
   volatile uint32_t dropped;                        /* ISR: ring full */
   snoop_ev_t        ring[SNOOP_RING];
} snoop_hot_t;

/* ---- ISR side ------------------------------------------------------------------------------- */

/* Second-level test, after the caller saw hot->page[addr >> 8] != 0. */
static inline int snoop_isr_match(const snoop_hot_t *hot, uint16_t addr)
{
   const uint8_t p = hot->page[addr >> 8];
   if (!p) return 0;
   return (hot->bits[p - 1][(addr & 0xff) >> 5] >> (addr & 31)) & 1u;
}

static inline void snoop_isr_store(snoop_hot_t *hot, uint16_t addr, uint8_t data, uint32_t stamp)
{
   const uint32_t h = hot->head;
   const uint32_t next = (h + 1) & (SNOOP_RING - 1);
   if (next == hot->tail) { hot->dropped++; return; }
   snoop_ev_t *e = &hot->ring[h];
   e->stamp = stamp;
   e->addr = addr;
   e->data = data;
   hot->head = next;
}

/* ---- main-loop side ------------------------------------------------------------------------- */

typedef struct {
   snoop_hot_t   *hot;
   int            running;

   /* configuration of the current run */
   snoop_range_t  ranges[SNOOP_MAX_RANGES];
   uint8_t        nranges;
   uint8_t        max_gap;        /* C64 cycles: a write joins the open block when within this of the previous one */
   uint32_t       phi2_hz;        /* C64 clock used for stamp conversion */
   uint32_t       cpu_hz;         /* stamp clock */
   uint16_t       flush_us;       /* send a non-empty WRITES frame after this long */
   uint16_t       status_ms;      /* STATUS frame period, 0 = only on request/stop */

   /* time conversion */
   uint32_t       last_stamp;
   uint64_t       c64_time;       /* C64 cycles since the start */
   uint64_t       rem;            /* remainder of the stamp -> cycle division, in cpu_hz units */
   int            have_stamp;

   /* WRITES frame being built (payload only) */
   uint8_t        frame[SNOOP_FRAME_MAX];
   uint16_t       flen;
   uint16_t       seq;
   uint64_t       block_time;     /* time of the open block's first write */
   uint64_t       last_write;     /* time of the open block's last write */
   uint64_t       prev_block;     /* time of the previous block in this frame (dt base) */
   uint16_t       block_addr;
   uint16_t       len_at;         /* frame offset of the open block's length byte, 0 = no open block */
   uint32_t       frame_born_us;

   /* statistics */
   uint32_t       dropped_seen;   /* hot->dropped at the last STATUS */
   uint32_t       out_dropped;    /* frames lost because the output FIFO was full */
   uint32_t       ring_hiwater;
   uint32_t       last_status_us;
   uint32_t       writes;

   /* output FIFO */
   uint8_t        out[SNOOP_OUT_SIZE];
   uint32_t       out_head, out_tail;
} snoop_t;

/* Clear everything; hot is the ISR's block (may live elsewhere in memory). */
void snoop_init(snoop_t *s, snoop_hot_t *hot);

/* Start a run.  Returns 0, or a negative error:
 *   -1 bad range count, -2 a range with first > last, -3 more than SNOOP_POOL_PAGES pages.
 * The ISR sees nothing until the page map is written, which is done last; a run already going
 * is stopped first (BYE REPLACED).  now_us is the glue's microsecond clock. */
int  snoop_start(snoop_t *s, const snoop_range_t *ranges, uint8_t nranges, uint8_t max_gap,
                 uint32_t phi2_hz, uint32_t cpu_hz, uint32_t now_us, uint32_t now_stamp);

/* Stop capturing (page map cleared first), drain what the ring holds, send STATUS and BYE. */
void snoop_stop(snoop_t *s, uint8_t reason, uint32_t now_us, uint32_t now_stamp);

/* Main loop: drain the ring, close frames that are full or older than flush_us, STATUS on its
 * period.  now_stamp is the stamp clock read BEFORE the call (so every event already in the ring
 * is no later than it, give or take the ISR's late-entry adjustment, which is clamped); the time
 * base moves up to it even with no writes, so a long quiet spell cannot wrap the 32-bit stamp.
 * The glue must call it at least every 2^31 stamp ticks (3.5 s at 600 MHz).
 * Returns the number of events taken. */
uint32_t snoop_poll(snoop_t *s, uint32_t now_us, uint32_t now_stamp);

/* Queue a STATUS frame now (closes the open WRITES frame first). */
void snoop_status(snoop_t *s);

/* Output FIFO: bytes ready, and taking them. */
uint32_t snoop_out_avail(const snoop_t *s);
uint32_t snoop_out_take(snoop_t *s, uint8_t *dst, uint32_t max);

/* Helpers, exposed for tests and the host side. */
uint16_t snoop_crc16(const uint8_t *p, size_t n);          /* CRC-16/CCITT-FALSE */
size_t   snoop_cobs_encode(const uint8_t *in, size_t n, uint8_t *out);  /* out >= n + n/254 + 1 */

#ifdef __cplusplus
}
#endif
#endif
