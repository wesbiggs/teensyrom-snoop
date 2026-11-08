

// teensyrom-snoop: isochronous OUT through siTDs (isochronous.h).  Phase 2 step 3 of teensyrom-snoop
// (docs/USB-AUDIO.md there): a USB audio dongle on the host port.  Written from EHCI 1.0, section 3.4
// (siTD) and 4.12.3 (split isochronous OUT).
//
// One siTD for each slot of the periodic frame list, linked first in the slot, ahead of the interrupt QHs
// (add_qh_to_periodic_schedule steps past it).  A siTD goes out in the frame of its slot when active; the
// refill (from isr(), on every siTD's interrupt-on-complete) keeps the next ISO_DEPTH frames queued.
// The siTDs live in DTCM like the QHs: the QH scheduling code rewrites the link word of whatever precedes a
// QH, a siTD included, so they must not sit behind the data cache.  The packet buffers are in RAM2, and
// each is flushed from the cache after it is filled.
#include "isoaudio.h"

#define ISO_SLOTS  PERIODIC_LIST_SIZE  // one siTD per slot
#define ISO_DEPTH  8                   // frames kept queued: the latency, and how late the refill may come
#define ISO_BUF    256                 // bytes per packet buffer; 256-aligned, so a packet never crosses a 4 KB page
#if ISO_DEPTH + 2 > ISO_SLOTS
#error "ISO_DEPTH must leave room in the periodic list"
#endif

typedef struct {
	volatile uint32_t w[8];        // EHCI siTD (7 words), padded to 32 bytes
} siTD_t;

static siTD_t iso_sitd[ISO_SLOTS] __attribute__ ((aligned(32)));
DMAMEM static uint8_t iso_buf[ISO_SLOTS][ISO_BUF] __attribute__ ((aligned(256)));

static struct {
	Device_t *dev;
	usbhost_iso_fill_t fill;
	void *ctx;
	uint32_t len;
	uint32_t ep;
	uint32_t shift;                // first microframe of the start-splits
	uint32_t splits;
	uint32_t stime;                // bandwidth per start-split, in uframe_bandwidth's units
	uint32_t next;                 // the first slot not yet filled
	uint32_t used;                 // bit n: slot n was queued since the start (its status counts)
	bool running;
} iso;
static usbhost_iso_stats_t iso_stats;

FLASHMEM bool usbhost_iso_out_start(Device_t *dev, uint32_t endpoint, uint32_t len, usbhost_iso_fill_t fill, void *ctx)
{
	if (iso.running || dev == NULL || dev->speed != 0 || fill == NULL || len == 0 || len > ISO_BUF) return false;
	// bandwidth as allocate_interrupt_pipe_bandwidth() counts a full-speed OUT start-split: the payload
	// (up to 188 bytes each) with worst-case bit stuffing, plus overhead, in 32-byte units
	const uint32_t splits = isoaudio_splits(len);
	const uint32_t payload = len < ISOAUDIO_SSPLIT_MAX ? len : ISOAUDIO_SSPLIT_MAX;
	const uint32_t stime = (100 + 32 + ((payload * 76459) >> 16)) >> 5;
	uint32_t best_shift = 0, best_bandwidth = 0xFFFFFFFF;
	for (uint32_t shift = 0; shift + splits <= 6; shift++) {
		uint32_t worst = 0;
		for (uint32_t i = 0; i < PERIODIC_LIST_SIZE; i++) {
			for (uint32_t k = 0; k < splits; k++) {
				const uint32_t bw = uframe_bandwidth[(i << 3) + shift + k] + stime;
				if (bw > worst) worst = bw;
			}
		}
		if (worst < best_bandwidth) {
			best_bandwidth = worst;
			best_shift = shift;
		}
	}
	println("iso_out_start: shift ", best_shift);
	if (best_bandwidth > 187) return false; // as for interrupt pipes: at most 80% of a microframe

	const bool irq_was_enabled = NVIC_IS_ENABLED(IRQ_USBHS);
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	for (uint32_t i = 0; i < PERIODIC_LIST_SIZE; i++) {
		for (uint32_t k = 0; k < splits; k++) uframe_bandwidth[(i << 3) + best_shift + k] += stime;
	}
	iso.dev = dev;
	iso.fill = fill;
	iso.ctx = ctx;
	iso.len = len;
	iso.ep = endpoint & 0x0F;
	iso.shift = best_shift;
	iso.splits = splits;
	iso.stime = stime;
	iso.used = 0;
	memset(&iso_stats, 0, sizeof(iso_stats));
	iso_stats.shift = best_shift;
	for (uint32_t i = 0; i < ISO_SLOTS; i++) {
		siTD_t *t = &iso_sitd[i];
		t->w[0] = periodictable[i];    // first in the slot: whatever was there follows it
		for (uint32_t k = 1; k < 8; k++) t->w[k] = 0;
		t->w[6] = 1;                   // back pointer: none
		__asm__ volatile("dsb" ::: "memory");
		periodictable[i] = (uint32_t)t | 4; // 4 = siTD, inactive until filled
	}
	iso.next = ((USBHS_FRINDEX >> 3) + 2) & (ISO_SLOTS - 1);
	iso.running = true;
	iso_stats.running = 1;
	USBHS_USBINTR |= USB_USBINTR_UE;   // a siTD's interrupt-on-complete, should the controller not flag it as periodic (UPI)
	usbhost_iso_out_service();
	if (irq_was_enabled) NVIC_ENABLE_IRQ(IRQ_USBHS);
	return true;
}

FLASHMEM void usbhost_iso_out_stop(void)
{
	const bool irq_was_enabled = NVIC_IS_ENABLED(IRQ_USBHS);
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	if (iso.running) {
		iso.running = false;
		iso_stats.running = 0;
		for (uint32_t i = 0; i < ISO_SLOTS; i++) {
			siTD_t *t = &iso_sitd[i];
			t->w[3] = 0;                // inactive, should the controller still reach it this frame
			volatile uint32_t *link = &periodictable[i];
			while (!(*link & 1)) {      // every descriptor type has its next link in word 0
				if ((*link & 0xFFFFFFE0) == (uint32_t)t) {
					*link = t->w[0];
					break;
				}
				link = (volatile uint32_t *)(*link & 0xFFFFFFE0);
			}
			for (uint32_t k = 0; k < iso.splits; k++) uframe_bandwidth[(i << 3) + iso.shift + k] -= iso.stime;
		}
		iso.dev = NULL;
	}
	if (irq_was_enabled) NVIC_ENABLE_IRQ(IRQ_USBHS);
}

FLASHMEM void usbhost_iso_out_stats(usbhost_iso_stats_t *s)
{
	const bool irq_was_enabled = NVIC_IS_ENABLED(IRQ_USBHS);
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	*s = iso_stats;
	if (irq_was_enabled) NVIC_ENABLE_IRQ(IRQ_USBHS);
}

// The refill: in the USBHS interrupt (RAM1, like the rest of isr()).
void usbhost_iso_out_service(void)
{
	if (!iso.running) return;
	const uint32_t cur = (USBHS_FRINDEX >> 3) & (ISO_SLOTS - 1);
	uint32_t missed;
	uint32_t n = isoaudio_plan(cur, &iso.next, ISO_SLOTS, ISO_DEPTH, &missed);
	iso_stats.missed += missed;
	const Device_t *dev = iso.dev;
	while (n--) {
		const uint32_t s = iso.next;
		siTD_t *t = &iso_sitd[s];
		const uint32_t st = t->w[3];
		if (st & ISOAUDIO_SITD_ACTIVE) break; // not gone yet: the controller is behind, try on the next interrupt
		if (iso.used & (1u << s)) {
			if (st & ISOAUDIO_SITD_ERRORS) {
				iso_stats.errors++;
				iso_stats.status = st & 0xFF;
			} else {
				iso_stats.frames++;
			}
		}
		uint8_t *buf = iso_buf[s];
		iso.fill(iso.ctx, buf, iso.len);
		arm_dcache_flush(buf, iso.len);
		uint32_t w[7];
		isoaudio_sitd_out(w, dev->address, iso.ep, dev->hub_address, dev->hub_port, iso.shift,
			(uint32_t)buf, iso.len, 1);
		t->w[1] = w[1];
		t->w[2] = w[2];
		t->w[4] = w[4];
		t->w[5] = w[5];
		t->w[6] = w[6];
		__asm__ volatile("dsb" ::: "memory");
		t->w[3] = w[3];                    // active last
		iso.used |= 1u << s;
		iso.next = (s + 1) & (ISO_SLOTS - 1);
	}
}
