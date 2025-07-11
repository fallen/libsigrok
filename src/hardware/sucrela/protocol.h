#ifndef LIBSIGROK_HARDWARE_SUCRELA_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SUCRELA_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "libuartbone/uartbone.h"

#define LOG_PREFIX "sucrela"

#define USB_INTERFACE           0
#define BUF_SIZE 4096
#define NUM_SAMPLERATES 15

struct dev_context {
	uint64_t dig_samplerate;
	uint64_t limit_samples;
	uint64_t sent_samples;
	uint64_t num_frames;
	uint64_t max_samplerate;
	uint64_t sample_size; // size of 1 sample
	uint8_t probe_number;
	unsigned int num_transfers;
	bool acq_aborted;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;
	unsigned int submitted_transfers;
	struct libusb_transfer **transfers;
	struct uartbone_ctx *uartbone_ctx;
	uint64_t samplerates[NUM_SAMPLERATES];
};

SR_PRIV int dev_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV void sucrela_abort_acquisition(struct dev_context *devc);

#endif
