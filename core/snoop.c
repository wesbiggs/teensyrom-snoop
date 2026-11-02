/* teensyrom-snoop: main-loop side.  See snoop.h and docs/PROTOCOL.md. */
#include "snoop.h"
#include <string.h>

#ifndef SNOOP_COLD
#define SNOOP_COLD   /* the firmware puts these in flash (FLASHMEM) */
#endif

/* ---- helpers -------------------------------------------------------------------------------- */

SNOOP_COLD uint16_t snoop_crc16(const uint8_t *p, size_t n)
{
   uint16_t crc = 0xFFFF;
   while (n--)
   {
      crc ^= (uint16_t)(*p++) << 8;
      for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
   }
   return crc;
}

SNOOP_COLD size_t snoop_cobs_encode(const uint8_t *in, size_t n, uint8_t *out)
{
   size_t o = 1, code_at = 0;
   uint8_t code = 1;
   for (size_t i = 0; i < n; i++)
   {
      if (in[i] == 0)
      {
         out[code_at] = code;
         code_at = o++;
         code = 1;
      }
      else
      {
         out[o++] = in[i];
         if (++code == 0xFF)
         {
            out[code_at] = code;
            code_at = o++;
            code = 1;
         }
      }
   }
   out[code_at] = code;
   return o;
}

static SNOOP_COLD size_t put_varint(uint8_t *p, uint64_t v)
{
   size_t n = 0;
   do
   {
      uint8_t b = (uint8_t)(v & 0x7F);
      v >>= 7;
      p[n++] = (uint8_t)(b | (v ? 0x80 : 0));
   } while (v);
   return n;
}

static SNOOP_COLD size_t put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return 2; }
static SNOOP_COLD size_t put_u32(uint8_t *p, uint32_t v)
{
   p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
   return 4;
}

/* ---- output FIFO ---------------------------------------------------------------------------- */

static SNOOP_COLD uint32_t out_used(const snoop_t *s) { return (s->out_head - s->out_tail) & (SNOOP_OUT_SIZE - 1); }
static SNOOP_COLD uint32_t out_free(const snoop_t *s) { return SNOOP_OUT_SIZE - 1 - out_used(s); }

SNOOP_COLD uint32_t snoop_out_avail(const snoop_t *s) { return out_used(s); }

SNOOP_COLD uint32_t snoop_out_take(snoop_t *s, uint8_t *dst, uint32_t max)
{
   uint32_t n = 0;
   while (n < max && s->out_tail != s->out_head)
   {
      dst[n++] = s->out[s->out_tail];
      s->out_tail = (s->out_tail + 1) & (SNOOP_OUT_SIZE - 1);
   }
   return n;
}

/* Frame a payload: CRC-16 appended, COBS, 0x00 delimiters on both sides.  A frame that does not fit is dropped
 * whole (counted), so the stream never carries a cut frame. */
static SNOOP_COLD void emit(snoop_t *s, const uint8_t *payload, size_t n)
{
   uint8_t raw[SNOOP_FRAME_MAX + 2];
   uint8_t enc[SNOOP_FRAME_MAX + 2 + (SNOOP_FRAME_MAX + 2) / 254 + 2];
   memcpy(raw, payload, n);
   const uint16_t crc = snoop_crc16(payload, n);
   raw[n] = (uint8_t)crc;
   raw[n + 1] = (uint8_t)(crc >> 8);
   const size_t e = snoop_cobs_encode(raw, n + 2, enc);
   if (out_free(s) < e + 2) { s->out_dropped++; return; }
   s->out[s->out_head] = 0;   /* a delimiter before as well: text the TR printed since cannot spoil this frame */
   s->out_head = (s->out_head + 1) & (SNOOP_OUT_SIZE - 1);
   for (size_t i = 0; i < e; i++)
   {
      s->out[s->out_head] = enc[i];
      s->out_head = (s->out_head + 1) & (SNOOP_OUT_SIZE - 1);
   }
   s->out[s->out_head] = 0;
   s->out_head = (s->out_head + 1) & (SNOOP_OUT_SIZE - 1);
}

/* ---- WRITES frames -------------------------------------------------------------------------- */

static SNOOP_COLD void frame_close(snoop_t *s)
{
   if (!s->len_at) return;     /* no block: nothing to send */
   emit(s, s->frame, s->flen);
   s->len_at = 0;
   s->flen = 0;
}

/* Header of a new WRITES frame: type, seq, t0 (absolute time of its first block). */
static SNOOP_COLD void frame_open(snoop_t *s, uint64_t t0, uint32_t now_us)
{
   s->flen = 0;
   s->frame[s->flen++] = SNOOP_F_WRITES;
   s->flen += (uint16_t)put_u16(s->frame + s->flen, s->seq++);
   s->flen += (uint16_t)put_varint(s->frame + s->flen, t0);
   s->prev_block = t0;
   s->frame_born_us = now_us;
}

#define BLOCK_HEAD_MAX (10 + 2 + 1)   /* dt varint, addr, len */

static SNOOP_COLD void add_write(snoop_t *s, uint16_t addr, uint8_t data, uint64_t t, uint32_t now_us)
{
   if (s->len_at)
   {
      uint8_t *len = &s->frame[s->len_at];
      if (addr == (uint16_t)(s->block_addr + *len) && *len < 255 &&
          t - s->last_write <= s->max_gap && s->flen < SNOOP_FRAME_MAX)
      {
         s->frame[s->flen++] = data;
         (*len)++;
         s->last_write = t;
         return;
      }
      if (s->flen + BLOCK_HEAD_MAX + 1 > SNOOP_FRAME_MAX) frame_close(s);
   }
   if (!s->len_at) frame_open(s, t, now_us);
   s->flen += (uint16_t)put_varint(s->frame + s->flen, t - s->prev_block);
   s->flen += (uint16_t)put_u16(s->frame + s->flen, addr);
   s->len_at = s->flen;
   s->frame[s->flen++] = 1;
   s->frame[s->flen++] = data;
   s->block_addr = addr;
   s->block_time = s->last_write = s->prev_block = t;
}

/* ---- time ----------------------------------------------------------------------------------- */

static SNOOP_COLD void advance(snoop_t *s, uint32_t stamp)
{
   const uint32_t d = stamp - s->last_stamp;
   if ((int32_t)d <= 0) return;   /* a late-entry stamp a little behind the base: same time */
   const uint64_t num = (uint64_t)d * s->phi2_hz + s->rem;
   s->c64_time += num / s->cpu_hz;
   s->rem = num % s->cpu_hz;
   s->last_stamp = stamp;
}

/* ---- control -------------------------------------------------------------------------------- */

SNOOP_COLD void snoop_init(snoop_t *s, snoop_hot_t *hot)
{
   memset(hot, 0, sizeof *hot);
   memset(s, 0, sizeof *s);
   s->hot = hot;
   s->flush_us = 2000;
   s->status_ms = 1000;
}

SNOOP_COLD void snoop_status(snoop_t *s)
{
   uint8_t p[1 + 2 + 10 + 4 + 4 + 2 + 4];
   size_t n = 0;
   frame_close(s);
   p[n++] = SNOOP_F_STATUS;
   n += put_u16(p + n, s->seq);              /* next WRITES seq: the host sees frames lost at the tail too */
   n += put_varint(p + n, s->c64_time);
   n += put_u32(p + n, s->hot->dropped);
   n += put_u32(p + n, s->out_dropped);
   n += put_u16(p + n, (uint16_t)(s->ring_hiwater > 0xFFFF ? 0xFFFF : s->ring_hiwater));
   n += put_u32(p + n, s->writes);
   emit(s, p, n);
   s->ring_hiwater = 0;
}

/* Page map for the ranges; 0 or an error (see snoop_start). */
static SNOOP_COLD int build_pmap(const snoop_range_t *ranges, uint8_t nranges, uint8_t pmap[256])
{
   if (nranges == 0 || nranges > SNOOP_MAX_RANGES) return -1;
   for (uint8_t i = 0; i < nranges; i++) if (ranges[i].first > ranges[i].last) return -2;
   uint8_t npages = 0;
   memset(pmap, 0, 256);
   for (uint8_t i = 0; i < nranges; i++)
      for (uint32_t pg = ranges[i].first >> 8; pg <= (uint32_t)(ranges[i].last >> 8); pg++)
         if (!pmap[pg])
         {
            if (npages == SNOOP_POOL_PAGES) return -3;
            pmap[pg] = ++npages;
         }
   return 0;
}

SNOOP_COLD int snoop_check(const snoop_range_t *ranges, uint8_t nranges)
{
   uint8_t pmap[256];
   return build_pmap(ranges, nranges, pmap);
}

SNOOP_COLD int snoop_start(snoop_t *s, const snoop_range_t *ranges, uint8_t nranges, uint8_t max_gap,
                           uint32_t phi2_hz, uint32_t cpu_hz, uint32_t now_us, uint32_t now_stamp)
{
   /* checks first, so a refused start leaves a running capture alone */
   uint8_t pmap[256];
   const int err = build_pmap(ranges, nranges, pmap);
   if (err) return err;
   if (phi2_hz == 0 || cpu_hz == 0) return -1;

   if (s->running) snoop_stop(s, SNOOP_BYE_REPLACED, now_us, now_stamp);

   snoop_hot_t *hot = s->hot;
   memset(hot->page, 0, sizeof hot->page);    /* the ISR is idle from here (it already was) */
   memset(hot->bits, 0, sizeof hot->bits);
   hot->head = hot->tail = 0;
   hot->dropped = 0;

   memcpy(s->ranges, ranges, nranges * sizeof *ranges);
   s->nranges = nranges;
   s->max_gap = max_gap;
   s->phi2_hz = phi2_hz;
   s->cpu_hz = cpu_hz;
   s->last_stamp = now_stamp;
   s->c64_time = 0;
   s->rem = cpu_hz / 2;                       /* round to the nearest cycle */
   s->flen = 0;
   s->len_at = 0;
   s->seq = 0;
   s->out_dropped = 0;
   s->ring_hiwater = 0;
   s->writes = 0;
   s->last_status_us = now_us;

   uint8_t p[1 + 1 + 4 + 4 + 1 + 1 + 4 * SNOOP_MAX_RANGES];
   size_t n = 0;
   p[n++] = SNOOP_F_HELLO;
   p[n++] = SNOOP_PROTO_VERSION;
   n += put_u32(p + n, phi2_hz);
   n += put_u32(p + n, cpu_hz);
   p[n++] = max_gap;
   p[n++] = nranges;
   for (uint8_t i = 0; i < nranges; i++)
   {
      n += put_u16(p + n, ranges[i].first);
      n += put_u16(p + n, ranges[i].last);
   }
   emit(s, p, n);

   for (uint8_t i = 0; i < nranges; i++)
      for (uint32_t a = ranges[i].first; a <= ranges[i].last; a++)
         hot->bits[pmap[a >> 8] - 1][(a & 0xFF) >> 5] |= 1u << (a & 31);
   s->running = 1;
   memcpy(hot->page, pmap, sizeof pmap);      /* last: arms the ISR */
   return 0;
}

SNOOP_COLD uint32_t snoop_poll(snoop_t *s, uint32_t now_us, uint32_t now_stamp)
{
   if (!s->running) return 0;
   snoop_hot_t *hot = s->hot;
   uint32_t taken = 0;
   uint32_t tail = hot->tail;
   const uint32_t head = hot->head;
   SNOOP_BARRIER();          /* no entry is read before head */
   const uint32_t used = (head - tail) & (SNOOP_RING - 1);
   if (used > s->ring_hiwater) s->ring_hiwater = used;
   while (tail != head)
   {
      const snoop_ev_t e = hot->ring[tail];
      tail = (tail + 1) & (SNOOP_RING - 1);
      SNOOP_BARRIER();            /* the entry is copied before its slot is handed back */
      hot->tail = tail;
      advance(s, e.stamp);
      add_write(s, e.addr, e.data, s->c64_time, now_us);
      taken++;
   }
   s->writes += taken;
   advance(s, now_stamp);

   if (s->len_at && (uint32_t)(now_us - s->frame_born_us) >= s->flush_us) frame_close(s);
   if (s->status_ms && (uint32_t)(now_us - s->last_status_us) >= (uint32_t)s->status_ms * 1000u)
   {
      s->last_status_us = now_us;
      snoop_status(s);
   }
   return taken;
}

SNOOP_COLD void snoop_stop(snoop_t *s, uint8_t reason, uint32_t now_us, uint32_t now_stamp)
{
   if (!s->running) return;
   memset(s->hot->page, 0, sizeof s->hot->page);   /* first: the ISR stops adding */
   snoop_poll(s, now_us, now_stamp);                /* what it already added */
   s->running = 0;
   snoop_status(s);
   uint8_t p[2] = { SNOOP_F_BYE, reason };
   emit(s, p, sizeof p);
}
