/* teensyrom-snoop: isochronous audio OUT, the portable half.  See isoaudio.h. */
#include "isoaudio.h"

#include <string.h>

/* descriptor types and audio class codes (USB 2.0 chapter 9, USB Audio 1.0 appendix A) */
#define DT_INTERFACE     4
#define DT_ENDPOINT      5
#define DT_IAD           11
#define DT_CS_INTERFACE  0x24
#define DT_CS_ENDPOINT   0x25
#define AC_HEADER        1
#define AC_INPUT         2
#define AC_OUTPUT        3
#define AC_MIXER         4
#define AC_SELECTOR      5
#define AC_FEATURE       6
#define AC_PROCESSING    7
#define AC_EXTENSION     8
#define AS_GENERAL       1
#define AS_FORMAT_TYPE   2
#define EP_GENERAL       1

static ISOAUDIO_COLD uint32_t le16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static ISOAUDIO_COLD uint32_t le24(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16; }

/* the next descriptor, or NULL at the end or on a malformed length */
static ISOAUDIO_COLD const uint8_t *next_desc(const uint8_t *p, const uint8_t *end)
{
   if (p[0] < 2) return NULL;
   p += p[0];
   if (p + 2 > end || p + p[0] > end || p[0] < 2) return NULL;
   return p;
}

ISOAUDIO_COLD int isoaudio_parse_control(const uint8_t *d, uint32_t len, isoaudio_control_t *ac)
{
   const uint8_t *end = d + len;
   memset(ac, 0, sizeof *ac);
   if (len < 9 || d[0] < 9 || d[1] != DT_INTERFACE || d[5] != 1 || d[6] != 1) return 0;
   ac->iface = d[2];
   for (const uint8_t *p = next_desc(d, end); p; p = next_desc(p, end))
   {
      if (p[1] == DT_INTERFACE || p[1] == DT_IAD) break;
      if (p[1] != DT_CS_INTERFACE || p[0] < 4 || ac->n >= ISOAUDIO_MAX_UNITS) continue;
      isoaudio_unit_t u;
      memset(&u, 0, sizeof u);
      u.subtype = p[2];
      u.id = p[3];
      switch (p[2])
      {
      case AC_INPUT:                    /* bTerminalID wTerminalType bAssocTerminal bNrChannels ... */
         if (p[0] < 12) continue;
         u.type = (uint16_t)le16(p + 4);
         break;
      case AC_OUTPUT:                   /* bTerminalID wTerminalType bAssocTerminal bSourceID iTerminal */
         if (p[0] < 9) continue;
         u.type = (uint16_t)le16(p + 4);
         u.nsrc = 1;
         u.src[0] = p[7];
         break;
      case AC_MIXER:                    /* bUnitID bNrInPins baSourceID[] ... */
      case AC_SELECTOR:
         if (p[0] < 5 || p[0] < 5u + p[4]) continue;
         for (uint8_t i = 0; i < p[4] && i < 4; i++) u.src[u.nsrc++] = p[5 + i];
         break;
      case AC_FEATURE:                  /* bUnitID bSourceID bControlSize bmaControls[ch+1] iFeature */
      {
         if (p[0] < 7 || p[5] == 0) continue;
         const uint8_t size = p[5];
         const uint32_t nctl = (uint32_t)(p[0] - 7) / size;   /* master + logical channels */
         u.nsrc = 1;
         u.src[0] = p[4];
         u.ctl_master = p[6];
         if (nctl > 1) u.ctl_ch[0] = p[6 + size];
         if (nctl > 2) u.ctl_ch[1] = p[6 + 2 * size];
         break;
      }
      case AC_PROCESSING:               /* bUnitID wProcessType bNrInPins baSourceID[] ... */
      case AC_EXTENSION:
         if (p[0] < 7 || p[0] < 7u + p[6]) continue;
         for (uint8_t i = 0; i < p[6] && i < 4; i++) u.src[u.nsrc++] = p[7 + i];
         break;
      default:                          /* the header, or something this parser does not follow */
         continue;
      }
      ac->unit[ac->n++] = u;
   }
   return 1;
}

/* one alternate setting: d at its interface descriptor; s gets what it offers.  Returns its verdict. */
static ISOAUDIO_COLD int check_alt(const uint8_t *d, const uint8_t *end, uint32_t rate, isoaudio_stream_t *s)
{
   int pcm = 0, format = 0, out = 0;
   memset(s, 0, sizeof *s);
   s->iface = d[2];
   s->alt = d[3];
   for (const uint8_t *p = next_desc(d, end); p; p = next_desc(p, end))
   {
      if (p[1] == DT_INTERFACE || p[1] == DT_IAD) break;
      if (p[1] == DT_CS_INTERFACE && p[2] == AS_GENERAL && p[0] >= 7)
      {  /* bTerminalLink bDelay wFormatTag */
         s->terminal = p[3];
         pcm = le16(p + 5) == 0x0001;
      }
      else if (p[1] == DT_CS_INTERFACE && p[2] == AS_FORMAT_TYPE && p[0] >= 8 && p[3] == 1)
      {  /* Type I: bNrChannels bSubframeSize bBitResolution bSamFreqType tSamFreq[] */
         s->channels = p[4];
         s->subframe = p[5];
         s->bits = p[6];
         s->rates = p[7];
         int has = 0;
         if (p[7] == 0)
            has = p[0] >= 14 && le24(p + 8) <= rate && rate <= le24(p + 11);
         else
            for (uint32_t i = 0; i < p[7] && 8u + 3 * i + 3 <= p[0]; i++) has |= le24(p + 8 + 3 * i) == rate;
         format = has && p[4] == 2 && p[5] == 2 && p[6] == 16;
      }
      else if (p[1] == DT_ENDPOINT && p[0] >= 7 && (p[3] & 3) == 1 && !(p[2] & 0x80) && !s->ep)
      {  /* bEndpointAddress bmAttributes wMaxPacketSize bInterval: the first isochronous OUT endpoint */
         s->ep = p[2];
         s->sync = (uint8_t)((p[3] >> 2) & 3);
         s->max_packet = (uint16_t)(le16(p + 4) & 0x7FF);
         s->interval = p[6];
         out = 1;
      }
      else if (p[1] == DT_CS_ENDPOINT && p[2] == EP_GENERAL && p[0] >= 4 && out)
         s->freq_control = p[3] & 1;
   }
   if (!out) return ISOAUDIO_NO_OUT;
   if (!pcm || !format) return ISOAUDIO_NO_FORMAT;
   if (s->sync == ISOAUDIO_SYNC_ASYNC) return ISOAUDIO_ASYNC;
   if (s->max_packet < (rate + 999) / 1000 * 4) return ISOAUDIO_SMALL;
   return ISOAUDIO_OK;
}

ISOAUDIO_COLD int isoaudio_find_playback(const uint8_t *d, uint32_t len, uint32_t rate, isoaudio_stream_t *s)
{
   const uint8_t *end = d + len;
   memset(s, 0, sizeof *s);
   if (len < 9 || d[0] < 9 || d[1] != DT_INTERFACE || d[5] != 1 || d[6] != 2) return ISOAUDIO_NOT_AS;
   const uint8_t iface = d[2];
   int best = ISOAUDIO_NO_OUT;
   for (const uint8_t *p = d; p; p = next_desc(p, end))
   {
      if (p[1] == DT_IAD) break;
      if (p[1] != DT_INTERFACE || p[0] < 9) continue;
      if (p[2] != iface) break;   /* the next interface: its alternate settings are its own */
      isoaudio_stream_t t;
      const int r = check_alt(p, end, rate, &t);
      if (r == ISOAUDIO_OK) { *s = t; return r; }
      if (r < best) { best = r; *s = t; }   /* the most specific reason: FORMAT before OUT, ASYNC before FORMAT ... */
   }
   return best;
}

ISOAUDIO_COLD const isoaudio_unit_t *isoaudio_feature_unit(const isoaudio_control_t *ac, uint8_t terminal)
{
   uint8_t id = terminal;
   for (int hop = 0; hop < 8; hop++)
   {
      const isoaudio_unit_t *down = NULL;
      for (uint8_t i = 0; i < ac->n && !down; i++)
      {
         const isoaudio_unit_t *u = &ac->unit[i];
         if (u->subtype == AC_INPUT || u->subtype == AC_OUTPUT) continue;
         for (uint8_t k = 0; k < u->nsrc; k++) if (u->src[k] == id) { down = u; break; }
      }
      if (!down) return NULL;
      if (down->subtype == AC_FEATURE) return down;
      id = down->id;
   }
   return NULL;
}

ISOAUDIO_COLD int isoaudio_fu_channels(const isoaudio_unit_t *fu, uint8_t control_bit, uint8_t ch[2])
{
   int n = 0;
   if (!fu) return 0;
   if (fu->ctl_master & control_bit) { ch[0] = 0; return 1; }
   for (uint8_t i = 0; i < 2; i++) if (fu->ctl_ch[i] & control_bit) ch[n++] = (uint8_t)(i + 1);
   return n;
}

/* ---- siTD ------------------------------------------------------------------------------------------- */

ISOAUDIO_COLD uint32_t isoaudio_splits(uint32_t len)
{
   return len == 0 ? 1 : (len + ISOAUDIO_SSPLIT_MAX - 1) / ISOAUDIO_SSPLIT_MAX;
}

ISOAUDIO_COLD int isoaudio_sitd_out(uint32_t w[7], uint8_t dev_addr, uint8_t ep, uint8_t hub_addr, uint8_t hub_port,
                                    uint32_t shift, uint32_t buf, uint32_t len, int ioc)
{
   const uint32_t splits = isoaudio_splits(len);
   if (len > 1023 || shift + splits > 6) return -1;   /* the last start-split no later than microframe 5 */
   /* word 1: direction (0 OUT), port number, hub address, endpoint, device address */
   w[1] = (uint32_t)(hub_port & 0x7F) << 24 | (uint32_t)(hub_addr & 0x7F) << 16 | (uint32_t)(ep & 0x0F) << 8 | (dev_addr & 0x7F);
   /* word 2: complete-split mask 0 (OUT has none), start-split mask: `splits` microframes from `shift` */
   w[2] = ((1u << splits) - 1) << shift;
   /* word 3: interrupt on complete, page select 0, total bytes, status active */
   w[3] = (ioc ? 0x80000000u : 0) | len << 16 | ISOAUDIO_SITD_ACTIVE;
   /* word 4: page 0 and current offset; word 5: page 1, transaction position (all / begin) and T-count */
   w[4] = buf;
   w[5] = ((buf & 0xFFFFF000u) + 0x1000u) | (splits > 1 ? 1u << 3 : 0u) | splits;
   /* word 6: back pointer, unused (T set) */
   w[6] = 1;
   return 0;
}

ISOAUDIO_COLD uint32_t isoaudio_plan(uint32_t cur, uint32_t *next, uint32_t size, uint32_t depth, uint32_t *missed)
{
   const uint32_t mask = size - 1;
   uint32_t ahead = (*next - cur) & mask;
   *missed = 0;
   if (ahead < 2 || ahead > depth + 1)
   {  /* frames cur+1 .. (the old next) went out with nothing queued: start again at cur+2 */
      const uint32_t resume = (cur + 2) & mask;
      *missed = (resume - *next) & mask;
      *next = resume;
      ahead = 2;
   }
   return depth + 1 - ahead;
}

/* ---- sample sources ------------------------------------------------------------------------------------- */

/* a quarter of a sine, 256 steps, peak 32767 (round(32767 * sin(pi/2 * i/256))) */
static const int16_t quarter_sine[257] ISOAUDIO_ROM = {
       0,   201,   402,   603,   804,  1005,  1206,  1407,  1608,  1809,  2009,  2210,
    2410,  2611,  2811,  3012,  3212,  3412,  3612,  3811,  4011,  4210,  4410,  4609,
    4808,  5007,  5205,  5404,  5602,  5800,  5998,  6195,  6393,  6590,  6786,  6983,
    7179,  7375,  7571,  7767,  7962,  8157,  8351,  8545,  8739,  8933,  9126,  9319,
    9512,  9704,  9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605,
   11793, 11980, 12167, 12353, 12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828,
   14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269, 15446, 15623, 15800, 15976,
   16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
   18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000,
   20159, 20317, 20475, 20631, 20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856,
   22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027, 23170, 23311, 23452, 23592,
   23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
   25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674,
   26790, 26905, 27019, 27133, 27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001,
   28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803, 28898, 28992, 29085, 29177,
   29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
   30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050,
   31113, 31176, 31237, 31297, 31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736,
   31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098, 32137, 32176, 32213, 32250,
   32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
   32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752,
   32757, 32761, 32765, 32766, 32767};

/* sine at step i of 1024 */
static ISOAUDIO_COLD int32_t sine1024(uint32_t i)
{
   i &= 1023;
   const uint32_t q = i >> 8, k = i & 255;
   const int32_t v = (q & 1) ? quarter_sine[256 - k] : quarter_sine[k];
   return q & 2 ? -v : v;
}

ISOAUDIO_COLD void isoaudio_tone_init(isoaudio_tone_t *t, uint32_t hz, uint32_t rate, int32_t amp)
{
   t->phase = 0;
   t->step = rate ? (uint32_t)((((uint64_t)hz << 32) + rate / 2) / rate) : 0;
   t->amp = amp < 0 ? 0 : amp > 32767 ? 32767 : amp;
}

ISOAUDIO_COLD int16_t isoaudio_tone_next(isoaudio_tone_t *t)
{
   const uint32_t i = t->phase >> 22, frac = (t->phase >> 6) & 0xFFFF;
   const int32_t a = sine1024(i), b = sine1024(i + 1);
   const int32_t s = a + (int32_t)(((int64_t)(b - a) * (int32_t)frac) >> 16);
   t->phase += t->step;
   return (int16_t)((s * t->amp) / 32767);
}

ISOAUDIO_COLD void isoaudio_tone_packet(isoaudio_tone_t *t, uint8_t *pkt, uint32_t frames)
{
   for (uint32_t i = 0; i < frames; i++)
   {
      const uint16_t v = (uint16_t)isoaudio_tone_next(t);
      pkt[0] = pkt[2] = (uint8_t)v;
      pkt[1] = pkt[3] = (uint8_t)(v >> 8);
      pkt += 4;
   }
}

/* ---- the sample ring -------------------------------------------------------------------------------------- */

#define RING_BARRIER() __asm__ __volatile__("" ::: "memory")   /* one core: order the stores, nothing more */

ISOAUDIO_COLD void isoaudio_ring_init(isoaudio_ring_t *r, int16_t *buf, uint32_t size, uint32_t target, uint32_t trim_above)
{
   memset(r, 0, sizeof *r);
   r->buf = buf;
   r->mask = size - 1;
   r->target = target;
   r->trim_above = trim_above;
   r->low = 0xFFFFFFFFu;
}

ISOAUDIO_COLD uint32_t isoaudio_ring_fill(const isoaudio_ring_t *r)
{
   return r->head - r->tail;
}

ISOAUDIO_COLD uint32_t isoaudio_ring_put(isoaudio_ring_t *r, const int16_t *s, uint32_t n)
{
   const uint32_t head = r->head;
   const uint32_t room = r->mask + 1 - (head - r->tail);
   if (n > room)
   {
      r->dropped += n - room;
      n = room;
   }
   for (uint32_t i = 0; i < n; i++) r->buf[(head + i) & r->mask] = s[i];
   RING_BARRIER();             /* the samples are in before head says so */
   r->head = head + n;
   return n;
}

ISOAUDIO_COLD void isoaudio_ring_packet(isoaudio_ring_t *r, uint8_t *pkt, uint32_t frames)
{
   const uint32_t head = r->head;
   RING_BARRIER();             /* head before the samples it covers */
   uint32_t tail = r->tail;
   uint32_t fill = head - tail;
   uint32_t n = 0;
   if (!r->playing && fill >= r->target) r->playing = 1;
   if (r->playing)
   {
      if (fill > r->trim_above)
      {
         r->trimmed += fill - r->target;
         tail += fill - r->target;
         fill = r->target;
      }
      if (fill < r->low) r->low = fill;
      n = fill < frames ? fill : frames;
      for (uint32_t i = 0; i < n; i++)
      {
         const uint16_t v = (uint16_t)r->buf[(tail + i) & r->mask];
         pkt[0] = pkt[2] = (uint8_t)v;
         pkt[1] = pkt[3] = (uint8_t)(v >> 8);
         pkt += 4;
      }
      if (n) r->last = r->buf[(tail + n - 1) & r->mask];
      if (n < frames)
      {
         r->underruns++;
         r->playing = 0;
      }
      RING_BARRIER();          /* done reading before the producer may reuse the space */
      r->tail = tail + n;
   }
   const uint16_t h = (uint16_t)r->last;
   for (uint32_t i = n; i < frames; i++)
   {
      pkt[0] = pkt[2] = (uint8_t)h;
      pkt[1] = pkt[3] = (uint8_t)(h >> 8);
      pkt += 4;
   }
}
