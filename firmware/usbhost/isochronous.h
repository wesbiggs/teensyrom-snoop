/* teensyrom-snoop: isochronous OUT for USBHost_t36 (phase 2 step 3, docs/USB-AUDIO.md in teensyrom-snoop).
 *
 * One stream at a time to a full-speed device: one packet every 1 ms frame, through siTDs (EHCI split
 * transaction isochronous descriptors) linked first in every slot of the periodic frame list, ahead of the
 * interrupt QHs that MIDI and HID use.  The controller's interrupt refills the ring: each packet is asked of
 * the caller's fill function a few frames before it goes out, so the latency is ISO_DEPTH frames.
 *
 * The implementation is at the end of ehci.cpp (it uses the periodic list and the bandwidth table there).
 * The fill function runs in the USBHS interrupt, below the TeensyROM's phi2 interrupt.
 */
#ifndef USBHOST_ISOCHRONOUS_H
#define USBHOST_ISOCHRONOUS_H

#include <stdint.h>

typedef struct Device_struct Device_t;

/* Fill packet with len bytes, for the frame it goes out in.  Called from the USBHS interrupt. */
typedef void (*usbhost_iso_fill_t)(void *ctx, uint8_t *packet, uint32_t len);

typedef struct {
   uint32_t frames;     /* packets sent without an error */
   uint32_t errors;     /* packets the controller flagged (transaction error, babble, buffer error, missed microframe) */
   uint32_t missed;     /* frames that went out with nothing queued: the refill came too late */
   uint32_t status;     /* the last flagged siTD status byte (0 when none) */
   uint32_t shift;      /* the microframe the start-splits begin in */
   uint8_t  running;
} usbhost_iso_stats_t;

/* Start streaming len bytes (1-256) every frame to OUT endpoint `endpoint` (number only) of a full-speed
 * device.  Fails when one already runs, the device is not full speed, or no microframes have room. */
bool usbhost_iso_out_start(Device_t *dev, uint32_t endpoint, uint32_t len, usbhost_iso_fill_t fill, void *ctx);

/* Stop and unlink the siTDs (the driver's disconnect(), or any time: it masks the USBHS interrupt itself). */
void usbhost_iso_out_stop(void);

/* A copy of the counters, for reports */
void usbhost_iso_out_stats(usbhost_iso_stats_t *s);

/* The refill, from USBHost::isr() */
void usbhost_iso_out_service(void);

#endif
