#ifndef LIBSIGROK_HARDWARE_SUCRELA_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SUCRELA_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "libuartbone/uartbone.h"

#define LOG_PREFIX	"sucrela"

#define USB_INTERFACE	0
#define BUF_SIZE	4096
#define NUM_SAMPLERATES	15

struct dev_context {
	uint64_t dig_samplerate;
	uint64_t limit_samples;
	uint64_t sent_samples;
	uint64_t num_frames;
	uint64_t max_samplerate;
	uint64_t sample_size; // size of 1 sample
	uint64_t prev_frame_counter;
	uint8_t probe_number;
	uint8_t oversampler_phy_ratio; // what the oversampler PHY is capable of
	uint8_t oversampling_ratio; // what oversampling setting we are using right now
	uint64_t current_buffer_offset;
	uint8_t has_framecounter; // TRUE if HSPI module is synthesized with frame_counter=True
	uint8_t hspi_width;
	gboolean continuous;
	gboolean aborting_capture;
	unsigned int num_transfers;
	bool acq_aborted;
	gboolean trigger_fired;
	struct soft_trigger_logic *stl;
	unsigned int submitted_transfers;
	struct libusb_transfer **transfers;
	struct uartbone_ctx *uartbone_ctx;
	pthread_t abort_thread;
	uint64_t samplerates[NUM_SAMPLERATES];
};

SR_PRIV int sucrela_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di);
SR_PRIV int dev_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int sucrela_abort_acquisition(struct dev_context *devc);

#endif
