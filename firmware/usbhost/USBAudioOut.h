/* teensyrom-snoop: a USB Audio Class 1 output driver for USBHost_t36 (phase 2 step 3, docs/USB-AUDIO.md in
 * teensyrom-snoop).  MIT, as the rest of the patch; written from the USB Audio 1.0 specification.
 *
 * Claims only an AudioStreaming interface with an alternate setting that takes 16-bit stereo PCM at 48 kHz on
 * an isochronous OUT endpoint (synchronous, adaptive or unsynchronised: no feedback endpoint), full speed only.
 * Capture and HID interfaces of the same device are left to others.  It selects that alternate setting, sets
 * the rate (SET_CUR on the endpoint; a refusal is not fatal), starts the stream (isochronous.h: 48 stereo frames every 1 ms
 * frame, asked of the fill function in the USBHS interrupt), then unmutes and sets 0 dB through the feature
 * unit when there is one.  Unplugging stops the stream; plugging back in starts it again.
 */
#ifndef USB_AUDIO_OUT_H
#define USB_AUDIO_OUT_H

#include "USBHost_t36.h"
#include "isochronous.h"
#include "isoaudio.h"

class USBAudioOut : public USBDriver {
public:
	enum { RATE = 48000, FRAME_BYTES = RATE / 1000 * 4 };   // 48 stereo frames of 16 bits: 192 bytes a frame

	// fill gets FRAME_BYTES every frame (16-bit little endian, left then right), in the USBHS interrupt
	USBAudioOut(USBHost &host, usbhost_iso_fill_t fill, void *ctx = NULL) : fill_(fill), ctx_(ctx) { init(); }

	struct Info {
		bool attached;           // a matching device is claimed
		bool streaming;          // its siTDs are queued
		uint16_t vid, pid;
		uint8_t iface, alt, ep, sync, fu;
		int8_t step;             // the setup step it is in (see control()); -1 when it failed there
		uint8_t failed_step;
		int16_t volume, vol_min, vol_max;   // 1/256 dB; volume is what was set (0 dB clamped to the range)
		bool unmuted;
		int last_reject;         // why the last AudioStreaming interface offered was not claimed (isoaudio.h)
		usbhost_iso_stats_t iso;
	};
	void info(Info *i);

protected:
	virtual bool claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len);
	virtual void control(const Transfer_t *transfer);
	virtual void disconnect();
	void init();

private:
	void next_step(bool ok);
	bool send(uint32_t type, uint32_t request, uint32_t value, uint32_t index, uint32_t length);

	usbhost_iso_fill_t fill_;
	void *ctx_;
	setup_t setup_ __attribute__ ((aligned(16)));
	uint8_t buf_[4] __attribute__ ((aligned(4)));
	Transfer_t transfers_[3] __attribute__ ((aligned(32)));

	Device_t *ac_dev_ = NULL;          // the device whose AudioControl interface ac_ holds
	isoaudio_control_t ac_;
	isoaudio_stream_t as_;
	uint8_t fu_ = 0;
	uint8_t mute_ch_[2], vol_ch_[2];
	uint8_t n_mute_ = 0, n_vol_ = 0, k_ = 0;
	volatile int8_t step_ = 0;
	uint8_t failed_step_ = 0;
	volatile bool streaming_ = false;
	bool unmuted_ = false;
	int16_t vol_min_ = 0, vol_max_ = 0, volume_ = 0;
	int last_reject_ = 0;
};

#endif
