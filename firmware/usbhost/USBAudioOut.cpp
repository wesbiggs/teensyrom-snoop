/* teensyrom-snoop: a USB Audio Class 1 output driver for USBHost_t36.  See USBAudioOut.h. */
#include <Arduino.h>
#include "USBAudioOut.h"

#define print   USBHost::print_
#define println USBHost::println_

// setup steps; control() moves through them in order, skipping what the device does not have
enum {
	STEP_IDLE = 0,
	STEP_SET_INTERFACE,   // select the alternate setting
	STEP_SET_RATE,        // SET_CUR sampling frequency on the endpoint (always: a refusal is noted, not fatal)
	STEP_GET_MIN,         // the feature unit's volume range...
	STEP_GET_MAX,
	STEP_SET_VOLUME,      // ...0 dB clamped into it, on each channel that has volume
	STEP_UNMUTE,          // mute off, on each channel that has mute
	STEP_DONE
};

FLASHMEM void USBAudioOut::init()
{
	contribute_Transfers(transfers_, sizeof(transfers_) / sizeof(Transfer_t));
	driver_ready_for_device(this);
}

FLASHMEM bool USBAudioOut::claim(Device_t *dev, int type, const uint8_t *d, uint32_t len)
{
	if (type == 0) {               // a new device: what an earlier one's AudioControl held is stale
		ac_dev_ = NULL;
		return false;
	}
	if (type != 1 || len < 9 || d[0] < 9 || d[1] != 4 || d[5] != 1) return false;   // audio interfaces only
	if (d[6] == 1) {               // AudioControl: remember its units for the streaming interface after it
		if (isoaudio_parse_control(d, len, &ac_)) ac_dev_ = dev;
		return false;
	}
	isoaudio_stream_t s;
	const int r = isoaudio_find_playback(d, len, RATE, &s);
	if (r != ISOAUDIO_OK) {
		if (r != ISOAUDIO_NOT_AS && r != ISOAUDIO_NO_OUT) last_reject_ = r;   // a playback interface we cannot drive
		return false;
	}
	if (dev->speed != 0) {         // siTDs are full-speed only
		println("USBAudioOut: not full speed");
		return false;
	}
	println("USBAudioOut claim, interface ", s.iface);
	as_ = s;
	const isoaudio_unit_t *fu = ac_dev_ == dev ? isoaudio_feature_unit(&ac_, s.terminal) : NULL;
	fu_ = fu ? fu->id : 0;
	n_mute_ = (uint8_t)isoaudio_fu_channels(fu, 1, mute_ch_);
	n_vol_ = (uint8_t)isoaudio_fu_channels(fu, 2, vol_ch_);
	unmuted_ = false;
	streaming_ = false;
	failed_step_ = 0;
	vol_min_ = vol_max_ = volume_ = 0;
	last_reject_ = 0;
	device = dev;                  // send() needs it before claim_drivers() sets it
	step_ = STEP_SET_INTERFACE;
	k_ = 0;
	if (!send(0x01, 11, as_.alt, as_.iface, 0)) {   // SET_INTERFACE
		step_ = -1;
		failed_step_ = STEP_SET_INTERFACE;
	}
	return true;
}

FLASHMEM bool USBAudioOut::send(uint32_t type, uint32_t request, uint32_t value, uint32_t index, uint32_t length)
{
	mk_setup(setup_, type, request, value, index, length);
	return queue_Control_Transfer(device, &setup_, length ? buf_ : NULL, this);
}

FLASHMEM void USBAudioOut::control(const Transfer_t *transfer)
{
	const bool ok = !(transfer->qtd.token & 0x40);   // a STALL halts the transfer
	next_step(ok);
}

// After the request of step_ finished (ok or not): use its answer, send the next one.
FLASHMEM void USBAudioOut::next_step(bool ok)
{
	const uint32_t ac_index = (uint32_t)fu_ << 8 | ac_.iface;
	while (step_ > STEP_IDLE && step_ < STEP_DONE) {
		switch (step_) {
		case STEP_SET_INTERFACE:
			if (!ok) {             // the device refused the alternate setting: no stream
				failed_step_ = STEP_SET_INTERFACE;
				step_ = -1;
				return;
			}
			step_ = STEP_SET_RATE;
			// Sent even when the format lists one rate and when the endpoint claims no rate control, as Linux
			// and macOS do: the owner's AB13X dongle took sn8's stream (no errors) and stayed silent without it.
			buf_[0] = (uint8_t)RATE;
			buf_[1] = (uint8_t)(RATE >> 8);
			buf_[2] = (uint8_t)(RATE >> 16);
			if (send(0x22, 0x01, 0x0100, as_.ep, 3)) return;   // SET_CUR SAMPLING_FREQ_CONTROL
			ok = false;
			break;
		case STEP_SET_RATE:
			if (!ok) failed_step_ = STEP_SET_RATE;   // carry on: the device may run 48 kHz anyway
			streaming_ = usbhost_iso_out_start(device, as_.ep, FRAME_BYTES, fill_, ctx_);
			println("USBAudioOut streaming ", streaming_);
			step_ = STEP_GET_MIN;
			if (n_vol_ && send(0xA1, 0x82, 0x0200 | vol_ch_[0], ac_index, 2)) return;   // GET_MIN VOLUME_CONTROL
			step_ = STEP_SET_VOLUME;
			k_ = n_vol_;           // no volume control: nothing to set
			ok = false;
			break;
		case STEP_GET_MIN:
			vol_min_ = ok ? (int16_t)(buf_[0] | buf_[1] << 8) : (int16_t)-32767;
			step_ = STEP_GET_MAX;
			if (send(0xA1, 0x83, 0x0200 | vol_ch_[0], ac_index, 2)) return;            // GET_MAX
			ok = false;
			break;
		case STEP_GET_MAX:
			vol_max_ = ok ? (int16_t)(buf_[0] | buf_[1] << 8) : 0;
			volume_ = vol_min_ > 0 ? vol_min_ : vol_max_ < 0 ? vol_max_ : 0;
			step_ = STEP_SET_VOLUME;
			k_ = 0;
			ok = true;
			// fall through to send the first SET_CUR
		case STEP_SET_VOLUME:
			if (k_ > 0 && !ok) failed_step_ = STEP_SET_VOLUME;
			if (k_ < n_vol_) {
				buf_[0] = (uint8_t)volume_;
				buf_[1] = (uint8_t)((uint16_t)volume_ >> 8);
				if (send(0x21, 0x01, 0x0200 | vol_ch_[k_++], ac_index, 2)) return;     // SET_CUR VOLUME_CONTROL
			}
			step_ = STEP_UNMUTE;
			k_ = 0;
			ok = true;
			break;
		case STEP_UNMUTE:
			if (k_ > 0) {
				if (ok) unmuted_ = true;
				else failed_step_ = STEP_UNMUTE;
			}
			if (k_ < n_mute_) {
				buf_[0] = 0;
				if (send(0x21, 0x01, 0x0100 | mute_ch_[k_++], ac_index, 1)) return;    // SET_CUR MUTE_CONTROL
			}
			step_ = STEP_DONE;
			break;
		}
	}
}

FLASHMEM void USBAudioOut::disconnect()
{
	if (streaming_) usbhost_iso_out_stop();
	streaming_ = false;
	step_ = STEP_IDLE;
	ac_dev_ = NULL;
}

FLASHMEM void USBAudioOut::info(Info *i)
{
	Device_t *dev = *(Device_t * volatile *)&device;
	memset(i, 0, sizeof(*i));
	i->attached = dev != NULL;
	i->streaming = streaming_;
	if (dev) {
		i->vid = dev->idVendor;
		i->pid = dev->idProduct;
	}
	i->iface = as_.iface;
	i->alt = as_.alt;
	i->ep = as_.ep;
	i->sync = as_.sync;
	i->fu = fu_;
	i->step = step_;
	i->failed_step = failed_step_;
	i->volume = volume_;
	i->vol_min = vol_min_;
	i->vol_max = vol_max_;
	i->unmuted = unmuted_;
	i->last_reject = last_reject_;
	usbhost_iso_out_stats(&i->iso);
}
