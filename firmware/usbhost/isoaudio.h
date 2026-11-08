/* teensyrom-snoop: isochronous audio OUT on the USB host port, the portable half (phase 2 step 3,
 * docs/USB-AUDIO.md).  Copied into the TeensyROM tree as src/USBHost_t36/isoaudio.{h,c} by
 * tools/make_tr_patch.py; host-tested through ctypes (tests/test_usbaudio.py).
 *
 *   - USB Audio Class 1 descriptor parsing: the AudioControl units, and the AudioStreaming alternate
 *     setting that takes 16-bit stereo PCM at a given rate on an isochronous OUT endpoint.
 *   - siTD words (EHCI 1.0 section 3.4) for one full-speed isochronous OUT packet, and the frame plan:
 *     which periodic-list slots to fill so the controller always has a few frames queued.
 *   - Sample sources: a sine test tone (integer only, the same samples on the host and the Teensy)
 *     and silence, written as 16-bit little-endian stereo packets.
 *
 * Plain C99, also compiled as C++ (keep it C++-clean).  No hardware access, no allocation.
 * MIT, as the rest of the patch.  Written from the USB Audio 1.0 and EHCI 1.0 specifications.
 */
#ifndef ISOAUDIO_H
#define ISOAUDIO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Teensy 4: setup code to flash (code defaults to RAM1 there, which is scarce); data tables too. */
#ifndef ISOAUDIO_COLD
#if defined(__IMXRT1062__)
#define ISOAUDIO_COLD __attribute__((section(".flashmem")))
#else
#define ISOAUDIO_COLD
#endif
#endif
#ifndef ISOAUDIO_ROM
#if defined(__IMXRT1062__)
#define ISOAUDIO_ROM __attribute__((section(".progmem")))
#else
#define ISOAUDIO_ROM
#endif
#endif

#define ISOAUDIO_MAX_UNITS 16      /* AudioControl terminals and units remembered */

/* isoaudio_find_playback results */
enum {
   ISOAUDIO_OK        = 1,
   ISOAUDIO_NOT_AS    = 0,     /* not an AudioStreaming interface */
   ISOAUDIO_NO_OUT    = -1,    /* no alternate setting with an isochronous OUT endpoint (a capture interface) */
   ISOAUDIO_NO_FORMAT = -2,    /* OUT, but no 16-bit stereo PCM at the rate */
   ISOAUDIO_ASYNC     = -3,    /* the only match is an asynchronous sink (needs a feedback endpoint: not supported) */
   ISOAUDIO_SMALL     = -4     /* the endpoint's wMaxPacketSize cannot carry one frame's samples */
};

/* endpoint synchronisation type, bmAttributes bits 3-2 */
enum { ISOAUDIO_SYNC_NONE = 0, ISOAUDIO_SYNC_ASYNC = 1, ISOAUDIO_SYNC_ADAPTIVE = 2, ISOAUDIO_SYNC_SYNC = 3 };

typedef struct {
   uint8_t  iface, alt;        /* AudioStreaming interface number and the alternate setting to select */
   uint8_t  ep;                /* endpoint address (bit 7 clear: OUT) */
   uint8_t  interval;          /* bInterval (full speed: frames, 1 for audio) */
   uint16_t max_packet;        /* wMaxPacketSize */
   uint8_t  sync;              /* ISOAUDIO_SYNC_* */
   uint8_t  terminal;          /* bTerminalLink: the AudioControl input terminal this stream feeds */
   uint8_t  rates;             /* bSamFreqType: number of discrete rates, 0 = a continuous range */
   uint8_t  freq_control;      /* the class-specific endpoint has a sampling frequency control */
   uint8_t  channels, subframe, bits;
} isoaudio_stream_t;

typedef struct {
   uint8_t id;
   uint8_t subtype;            /* 2 input terminal, 3 output terminal, 4 mixer, 5 selector, 6 feature unit, ... */
   uint8_t nsrc;               /* sources remembered (up to 4) */
   uint8_t src[4];
   uint8_t ctl_master;         /* feature unit: bmaControls(0), low byte (bit 0 mute, bit 1 volume) */
   uint8_t ctl_ch[2];          /* feature unit: bmaControls(1), (2), low byte */
   uint16_t type;              /* terminals: wTerminalType */
} isoaudio_unit_t;

typedef struct {
   uint8_t iface;              /* the AudioControl interface number (wIndex low byte of unit requests) */
   uint8_t n;
   isoaudio_unit_t unit[ISOAUDIO_MAX_UNITS];
} isoaudio_control_t;

/* d points at an interface descriptor, len bytes to the end of the configuration.
 * isoaudio_parse_control: 1 when it is an AudioControl interface (class 1, subclass 1); its terminals and
 * units are read into ac (up to the next interface descriptor). */
int isoaudio_parse_control(const uint8_t *d, uint32_t len, isoaudio_control_t *ac);

/* An AudioStreaming interface (class 1, subclass 2): the first alternate setting of this interface number
 * with an isochronous OUT endpoint taking Type I PCM, 2 channels of 16 bits (2-byte subframes) at rate Hz
 * (a discrete rate or within a continuous range), a packet of at least rate/1000 frames, and a synchronous,
 * adaptive or unsynchronised endpoint.  Returns ISOAUDIO_OK and fills s, or why not (the enum above). */
int isoaudio_find_playback(const uint8_t *d, uint32_t len, uint32_t rate, isoaudio_stream_t *s);

/* The feature unit downstream of input terminal `terminal` (through at most 8 units: mixers, selectors,
 * processing units), or NULL. */
const isoaudio_unit_t *isoaudio_feature_unit(const isoaudio_control_t *ac, uint8_t terminal);

/* Which channels of a feature unit to address for a control bit (1 mute, 2 volume): {0} when the master
 * channel has it, else those of channels 1-2 that have it.  Returns the count, channels in ch[]. */
int isoaudio_fu_channels(const isoaudio_unit_t *fu, uint8_t control_bit, uint8_t ch[2]);

/* ---- siTD (EHCI split-transaction isochronous transfer descriptor) ----------------------------------- */

#define ISOAUDIO_SSPLIT_MAX 188    /* bytes one OUT start-split carries (EHCI 4.12.3.1) */

/* Words 1-6 of a siTD for one full-speed isochronous OUT packet of len bytes (1-1023) at buffer address buf
 * (in 32-bit DMA space), starting its start-splits in microframe `shift`.  Word 0 (the next link) is left
 * alone.  Word 3 is the active transfer state with interrupt-on-complete if ioc.  Returns 0, or -1 when the
 * splits do not fit before microframe 7 (the controller needs the last one free). */
int isoaudio_sitd_out(uint32_t w[7], uint8_t dev_addr, uint8_t ep, uint8_t hub_addr, uint8_t hub_port,
                      uint32_t shift, uint32_t buf, uint32_t len, int ioc);

/* start-splits one OUT packet of len bytes needs (the siTD's T-count) */
uint32_t isoaudio_splits(uint32_t len);

/* siTD status byte (word 3, bits 7-0) */
#define ISOAUDIO_SITD_ACTIVE   0x80
#define ISOAUDIO_SITD_ERRORS   0x7C    /* ERR, data buffer error, babble, transaction error, missed microframe */

/* The frame plan.  cur: the periodic-list slot the controller is in; *next: the first slot not yet filled;
 * size: the list's length (a power of two); depth: frames to keep queued.  Slots cur+2 .. cur+depth are kept
 * filled (cur+1 may already be fetched).  Returns how many slots to fill from *next on.  When the queue ran
 * dry (the refill came too late), *next moves to cur+2 and *missed is the number of frames that went out
 * with nothing queued; else *missed is 0. */
uint32_t isoaudio_plan(uint32_t cur, uint32_t *next, uint32_t size, uint32_t depth, uint32_t *missed);

/* ---- sample sources ------------------------------------------------------------------------------------- */

typedef struct {
   uint32_t phase;             /* 2^32 = one period */
   uint32_t step;              /* phase per sample */
   int32_t  amp;               /* peak, 0-32767 */
} isoaudio_tone_t;

/* hz at rate samples a second, peak amp (0-32767) */
void isoaudio_tone_init(isoaudio_tone_t *t, uint32_t hz, uint32_t rate, int32_t amp);

/* The next sample (a 1024-step sine, linearly interpolated: spurs ~-90 dB at -12 dBFS), phase moved on. */
int16_t isoaudio_tone_next(isoaudio_tone_t *t);

/* frames stereo frames (both channels the same), 16-bit little endian: 4 * frames bytes to pkt.
 * Packets written one after another continue the phase exactly. */
void isoaudio_tone_packet(isoaudio_tone_t *t, uint8_t *pkt, uint32_t frames);

#ifdef __cplusplus
}
#endif

#endif
