/* Test shim: a flat C API over core/snoop.c for ctypes (tests/test_core.py).
 * sim_bus() does what the firmware's phi2 ISR does on a write cycle. */
#include "../core/snoop.h"

static snoop_hot_t hot;
static snoop_t st;

void sim_init(void) { snoop_init(&st, &hot); }

int sim_start(const uint16_t *pairs, int n, int gap, uint32_t phi2, uint32_t cpu, uint32_t now_us, uint32_t stamp)
{
   snoop_range_t r[SNOOP_MAX_RANGES + 1];
   if (n < 0 || n > SNOOP_MAX_RANGES + 1) return -100;
   for (int i = 0; i < n; i++) { r[i].first = pairs[2 * i]; r[i].last = pairs[2 * i + 1]; }
   return snoop_start(&st, r, (uint8_t)n, (uint8_t)gap, phi2, cpu, now_us, stamp);
}

/* returns 1 when the write was captured (or dropped for a full ring), 0 when not watched */
int sim_bus(uint16_t addr, uint8_t data, uint32_t stamp)
{
   if (!hot.page[addr >> 8]) return 0;          /* the ISR's fast-path test */
   if (!snoop_isr_match(&hot, addr)) return 0;
   snoop_isr_store(&hot, addr, data, stamp);
   return 1;
}

uint32_t sim_poll(uint32_t now_us, uint32_t stamp) { return snoop_poll(&st, now_us, stamp); }
void sim_stop(int reason, uint32_t now_us, uint32_t stamp) { snoop_stop(&st, (uint8_t)reason, now_us, stamp); }
void sim_status(void) { snoop_status(&st); }
uint32_t sim_take(uint8_t *dst, uint32_t max) { return snoop_out_take(&st, dst, max); }
uint32_t sim_avail(void) { return snoop_out_avail(&st); }
uint32_t sim_dropped(void) { return hot.dropped; }
int sim_running(void) { return st.running; }
void sim_set(int flush_us, int status_ms) { st.flush_us = (uint16_t)flush_us; st.status_ms = (uint16_t)status_ms; }
int sim_page(int pg) { return hot.page[pg & 0xFF]; }
uint16_t sim_crc16(const uint8_t *p, uint32_t n) { return snoop_crc16(p, n); }
uint32_t sim_cobs(const uint8_t *in, uint32_t n, uint8_t *out) { return (uint32_t)snoop_cobs_encode(in, n, out); }
void sim_late(uint32_t cyc) { snoop_isr_late(&hot, cyc); }
void sim_load(uint32_t window, uint32_t irq) { snoop_load_sample(&st, window, irq); }
uint32_t sim_late_cyc(int i) { return hot.late_cyc[i]; }
int sim_ring_size(void) { return SNOOP_RING; }
int sim_out_size(void) { return SNOOP_OUT_SIZE; }
