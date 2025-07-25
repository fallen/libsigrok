/*
 * SucréLA hardware support code
 * Author: Yann Sionneau
 * This is *very* heavily inspired from fx2lafw support code
 * in src/hardware/fx2lafw
 */

#include "protocol.h"
#include "csr.h"
#include <time.h>

static int sucrela_start_sampling(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct uartbone_ctx *ctx = devc->uartbone_ctx;
	uint16_t subsampler = 0;
	int ret;

	sr_err("max_samplerate: %"PRIu64"\n", devc->max_samplerate);
	sr_err("cur samplerate: %"PRIu64"\n", devc->dig_samplerate);
	sr_err("oversampler phy ratio: %d\n", devc->oversampler_phy_ratio);
	sr_err("oversampling ratio: %d\n", devc->oversampling_ratio);

	if (devc->oversampling_ratio == 2) {
		ret = uartbone_write(ctx, CSR_LA_OVERSAMPLER_RATIO_ADDR, 2);
	} else if (devc->oversampling_ratio == 4) {
		ret = uartbone_write(ctx, CSR_LA_OVERSAMPLER_RATIO_ADDR, 4);
	} else {
		ret = uartbone_write(ctx, CSR_LA_OVERSAMPLER_RATIO_ADDR, 1);
	}

	// let's use the subsampler if needed
	if (devc->dig_samplerate <= devc->max_samplerate) {
		subsampler = (devc->max_samplerate / devc->dig_samplerate) - 1;
		sr_err("using subsampler of: %d\n", subsampler);
	}

	ret |= uartbone_write(ctx, CSR_LA_TRIGGER_MEM_MASK_ADDR, 0);
	ret |= uartbone_write(ctx, CSR_LA_TRIGGER_MEM_VALUE_ADDR, 0);
	ret |= uartbone_write(ctx, CSR_LA_TRIGGER_MEM_WRITE_ADDR, 1);
	ret |= uartbone_write(ctx, CSR_LA_SUBSAMPLER_VALUE_ADDR, subsampler);
	ret |= uartbone_write(ctx, CSR_LA_HSPI_TX_MAX_PACKET_SIZE_ADDR, 4096);
	ret |= uartbone_write(ctx, CSR_LA_HSPI_TX_MAX_PACKET_NUM_R_ADDR, 0);
	ret |= uartbone_write(ctx, CSR_LA_HSPI_TX_ENABLE_ADDR, 1);
	ret |= uartbone_write(ctx, CSR_LA_TRIGGER_ENABLE_ADDR, 1);

	if (ret)
		sr_err("could not write some register\n");

	return ret;
}

SR_PRIV int sucrela_abort_acquisition(struct dev_context *devc)
{
	struct uartbone_ctx *ctx = devc->uartbone_ctx;
	uint32_t hspi_tx_enable;
	int ret;
	unsigned int timeout = 10;

	sr_err("sucrela_abort_acquisition()\n");

	do {
		ret = uartbone_write(ctx, CSR_LA_HSPI_TX_ENABLE_ADDR, 0);
		if (!timeout--)
			break;
	} while (ret);

	if (ret)
		sr_err("Could not stop HSPI TX\n");

	timeout = 10;
	do {
		ret = uartbone_read(ctx, CSR_LA_HSPI_TX_ENABLE_ADDR, &hspi_tx_enable);
		if (!timeout--)
			break;
	} while (ret);
	if (ret)
		sr_err("Could not read-back hspi_enable register\n");

	devc->acq_aborted = TRUE;
	if (hspi_tx_enable != 0) {
		sr_err("Could not stop the transfers\n");
		return SR_ERR;
	}

	for (int i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}

	return SR_OK;
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct drv_context *drvc = sdi->driver->context;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, drvc->sr_ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void sucrela_send_data_proc(struct sr_dev_inst *sdi, uint8_t *data,
				   size_t length, size_t sample_width)
{
	const struct sr_datafeed_logic logic = { .length = length,
						 .unitsize = sample_width,
						 .data = data };

	const struct sr_datafeed_packet packet = { .type = SR_DF_LOGIC,
						   .payload = &logic };

	sr_session_send(sdi, &packet);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi = transfer->user_data;
	struct dev_context *devc = sdi->priv;
	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset, cur_sample_count, unitsize;
	int pre_trigger_samples;
	int oversampling = devc->dig_samplerate > devc->max_samplerate;
	int oversampling_ratio = devc->dig_samplerate / devc->max_samplerate;
	unsigned char *buffer = NULL;
	uint32_t frame_counter;
	unsigned int cur_transfer_offset = 0;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	if (devc->aborting_capture) {
		resubmit_transfer(transfer);
		return;
	}

	unitsize = devc->probe_number >> 3;

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		sr_err("!!!!!!!!!!! NO DEVICE !!!!!!!!!\n");
		devc->aborting_capture = TRUE;
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
		break;
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		sr_err("!! libusb transfer timed out !!\n");
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		packet_has_error = TRUE;
		sr_err("libusb transfer canceled\n");
		break;
	case LIBUSB_TRANSFER_ERROR:
		packet_has_error = TRUE;
		sr_err("libusb transfer error\n");
		break;
	case LIBUSB_TRANSFER_STALL:
		packet_has_error = TRUE;
		sr_err("libusb transfer stalled\n");
		break;
	case LIBUSB_TRANSFER_OVERFLOW:
		packet_has_error = TRUE;
		sr_err("libusb transfer overflow\n");
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (packet_has_error) {
		sr_err("resubmit and bail out because packet has error %d\n", transfer->status);
		resubmit_transfer(transfer);
		return;
	}

	while (cur_transfer_offset != transfer->actual_length) {
		unsigned int remaining_transfer_length = transfer->actual_length - cur_transfer_offset;
		unsigned int cur_hspi_packet_size = remaining_transfer_length >= 4096 ? 4096 : remaining_transfer_length;
		unsigned char *current_buffer = transfer->buffer + cur_transfer_offset;
		unsigned int cur_buffer_length = cur_hspi_packet_size;

		if (devc->has_framecounter) {
			switch (devc->hspi_width) {
				case 8:
					frame_counter = *(uint8_t *)(current_buffer);
					current_buffer += 1;
					cur_buffer_length -= 1;
					break;
				case 16:
					frame_counter = *(uint16_t *)(current_buffer);
					current_buffer += 2;
					cur_buffer_length -= 2;
					break;
				case 32:
					frame_counter = *(uint32_t *)(current_buffer);
					current_buffer += 4;
					cur_buffer_length -= 4;
					break;
				default:
					devc->aborting_capture = TRUE;
					sr_err("au secours hspi_width n'est ni 8 ni 16 ni 32\n");
					free_transfer(transfer);
					return;
			}

			if (devc->prev_frame_counter != -1) {
				if (frame_counter != (devc->prev_frame_counter + 1) % (1 << devc->hspi_width))
					sr_err("ERROR prev frame counter: %"PRIu64" current one: %d\n", devc->prev_frame_counter, frame_counter);
			}
			devc->prev_frame_counter = frame_counter;
		}

		cur_sample_count = cur_buffer_length / unitsize;
		if (!oversampling)
			buffer = current_buffer;
		else {
			cur_sample_count *= oversampling_ratio;
			buffer = g_malloc(cur_buffer_length * oversampling_ratio);
			if (oversampling_ratio == 2) {
				for (unsigned int i = 0; i < cur_buffer_length;
				i++) {
					buffer[2 * i] = current_buffer[i] & 0x0f;
					buffer[2 * i + 1] =
						(current_buffer[i] & 0xf0) >> 4;
				}
			}
			if (oversampling_ratio == 4) {
				for (unsigned int i = 0; i < cur_buffer_length;
				i++) {
					buffer[4 * i] = current_buffer[i] & 0x03;
					buffer[4 * i + 1] =
						(current_buffer[i] & 0x0c) >> 2;
					buffer[4 * i + 2] =
						(current_buffer[i] & 0x30) >> 4;
					buffer[4 * i + 3] =
						(current_buffer[i] & 0xc0) >> 6;
				}
			}
		}

		if (devc->trigger_fired) {
			if (!devc->limit_samples ||
			devc->sent_samples < devc->limit_samples) {
				/* Send the incoming transfer to the session bus. */
				num_samples = cur_sample_count;
				if (devc->limit_samples &&
				devc->sent_samples + num_samples >
					devc->limit_samples)
					num_samples = devc->limit_samples -
						devc->sent_samples;

				sucrela_send_data_proc(
					sdi,
					(uint8_t *)buffer,
					num_samples * unitsize, unitsize);
				devc->sent_samples += num_samples;
			}
		} else {
			trigger_offset = soft_trigger_logic_check(
				devc->stl, buffer,
				cur_sample_count,
				&pre_trigger_samples);

			if (trigger_offset > -1) {
				std_session_send_df_frame_begin(sdi);
				devc->sent_samples += pre_trigger_samples;
				num_samples = cur_sample_count -
					trigger_offset;
				if (devc->limit_samples &&
				devc->sent_samples + num_samples >
					devc->limit_samples)
					num_samples = devc->limit_samples -
						devc->sent_samples;

				sucrela_send_data_proc(
					sdi,
					(uint8_t *)buffer +
						trigger_offset * unitsize,
					num_samples * unitsize, unitsize);
				devc->sent_samples += num_samples;

				devc->trigger_fired = TRUE;
			}
		}
		cur_transfer_offset += cur_hspi_packet_size;
	}

	const int frame_ended = devc->limit_samples &&
				(devc->sent_samples >= devc->limit_samples);

	if (frame_ended) {
		devc->aborting_capture = TRUE;
		sr_err("End of capture!\n");
		devc->num_frames++;
		devc->sent_samples = 0;
		devc->trigger_fired = FALSE;
		std_session_send_df_frame_end(sdi);
	}

	resubmit_transfer(transfer); // we still need to process USB until capture is really stopped
	if (oversampling)
		g_free(buffer);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct drv_context *drvc = sdi->driver->context;
	struct timeval tv;

	(void)fd;
	(void)revents;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv,
					       NULL);

	return TRUE;
}

static unsigned int to_bytes_per_ms(unsigned int samplerate)
{
	return samplerate / 1000;
}

static size_t get_transfer_size(struct dev_context *devc)
{
	size_t s;

	/*
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of 4096.
	 */
	s = 10 * to_bytes_per_ms(devc->dig_samplerate) / devc->oversampling_ratio;
	return (s + 4095) & ~4095;
}

static void *abort_thread(void *sdi) {
	struct dev_context *devc = ((const struct sr_dev_inst *)sdi)->priv;
	struct timespec duration;

	// 100 ms
	duration.tv_sec = 0;
	duration.tv_nsec = 100000000;
	do {
		nanosleep(&duration, NULL);
	} while (!devc->aborting_capture);


	sucrela_abort_acquisition(devc);
	devc->aborting_capture = FALSE;

	return NULL;
}

SR_PRIV int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct drv_context *drvc = sdi->driver->context;
	struct libusb_transfer *transfer;
	struct sr_usb_dev_inst *usb;
	uint8_t *buf;
	unsigned int i, ret;
	struct sr_trigger *trigger;
	unsigned int transfer_size;

	usb = sdi->conn;

	if ((trigger = sr_session_trigger_get(sdi->session))) {
		int pre_trigger_samples = 0;
		;
		devc->stl = soft_trigger_logic_new(sdi, trigger,
						   pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;
	} else {
		std_session_send_df_frame_begin(sdi);
		devc->trigger_fired = TRUE;
	}

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->num_transfers = 16;
	devc->prev_frame_counter = -1;
	devc->current_buffer_offset = 0;
	transfer_size = get_transfer_size(devc);
	sr_err("transfer_size: %d\n", transfer_size);

	devc->transfers =
		g_malloc0(sizeof(*devc->transfers) * devc->num_transfers);
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_malloc(transfer_size);
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
					  2 | LIBUSB_ENDPOINT_IN, buf, transfer_size,
					  receive_transfer, (void *)sdi, 1000);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			sucrela_abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	usb_source_add(sdi->session, drvc->sr_ctx, 500, receive_data, (void *)sdi);

	std_session_send_df_header(sdi);

	pthread_create(&devc->abort_thread, NULL, abort_thread, (void *)sdi);

	ret = sucrela_start_sampling(sdi);

	return ret;
}

SR_PRIV int sucrela_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	int ret = SR_ERR, i, j;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct drv_context *drvc;
	struct dev_context *devc = sdi->priv;
	char connection_id[64];
	char ident_str[256];
	uint32_t val;
	ssize_t device_count;
	struct sr_channel *channels_to_remove[16];
	struct sr_channel *ch;

	sr_err("sucrela_dev_open()\n");

	drvc = di->context;
	usb = sdi->conn;

	device_count =
		libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}
	for (i = 0; i < device_count; i++) {
		ret = libusb_get_device_descriptor(devlist[i], &des);
		if (ret) {
			sr_err("Failed to get device descriptor: %s.",
			       libusb_error_name(ret));
			continue;
		}

		if ((sdi->status == SR_ST_INITIALIZING) ||
		    (sdi->status == SR_ST_INACTIVE)) {
			/*
			 * Check device by its physical USB bus/port address.
			 */
			if (usb_get_port_path(devlist[i], connection_id,
					      sizeof(connection_id)) != SR_OK) {
				sr_err("failed to call usb_get_port_path\n");
				continue;
			}

			if (strcmp(sdi->connection_id, connection_id)) {
				sr_err("bad conn id: %s, looking for %s\n",
				       connection_id, sdi->connection_id);
				/* This is not the one. */
				continue;
			} else {
				sr_err("Found it!\n");
			}
		}
		if ((ret = libusb_open(devlist[i], &usb->devhdl))) {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}
		if (libusb_has_capability(
			    LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl,
							USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(
					     usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
					       libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}
		ret = SR_OK;
		sr_info("Found device SucréLA\n");
		break;
	}

	devc->uartbone_ctx = g_malloc0(sizeof(*devc->uartbone_ctx));
	devc->uartbone_ctx->usb_handle = usb->devhdl;
	devc->uartbone_ctx->endpoint = 1;
	uartbone_unix_init(devc->uartbone_ctx, "usb://", 0, 4);
	libusb_free_device_list(devlist, 1);

	uartbone_read(devc->uartbone_ctx, CSR_LA_SAMPLERATE_ADDR, &val);
	devc->max_samplerate = val;
	uartbone_read(devc->uartbone_ctx,
				   CSR_LA_HSPI_TX_HSPI_WIDTH_R_ADDR, &val);
	devc->hspi_width = val;
	uartbone_read(devc->uartbone_ctx, CSR_LA_NUM_PROBES_ADDR, &val);
	devc->probe_number = val;
	uartbone_read(
		devc->uartbone_ctx, CSR_LA_OVERSAMPLER_PHY_RATIO_ADDR, &val);
	devc->oversampler_phy_ratio = val;
	uartbone_read(devc->uartbone_ctx, CSR_LA_HSPI_TX_HAS_FRAME_COUNTER_ADDR, &val);
	devc->has_framecounter = val;
	// Disable HSPI if it was still bursting from a previous badly-ended session
	uartbone_write(devc->uartbone_ctx, CSR_LA_HSPI_TX_ENABLE_ADDR, 0);
	devc->dig_samplerate = devc->max_samplerate;
	devc->oversampling_ratio = 1;
	devc->aborting_capture = FALSE;
	sr_err("max_samplerate: %"PRIu64"\n", devc->max_samplerate);
	sr_err("hspi_width: %d\n", devc->hspi_width);
	sr_err("probe_number: %d\n", devc->probe_number);
	sr_err("oversampler phy ratio: %d\n", devc->oversampler_phy_ratio);
	sr_err("hspi has framecounter: %s\n", devc->has_framecounter ? "True" : "False");

	memset(channels_to_remove, 0, sizeof(channels_to_remove));

	for (j = 15; j >= devc->probe_number; j--) {
		ch = g_slist_nth(sdi->channels, j)->data;
		sdi->channels = g_slist_remove(sdi->channels, ch);
		sr_err("removing channel %d %s\n", j, ch->name);
	}

	// Create samplerate array
	for (i = 0; i < NUM_SAMPLERATES; i++) {
		devc->samplerates[i] = devc->max_samplerate >> i;
	}

	i = 0;
	memset(ident_str, '\0', sizeof(ident_str));
	do {
		uartbone_read(devc->uartbone_ctx,
				  CSR_IDENTIFIER_MEM_BASE + i * 4, &val);
		ident_str[i++] = val;
	} while (val);
	sr_err("ident: %s\n", ident_str);

	return ret;
}
