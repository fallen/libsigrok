/*
 * SucréLA hardware support code
 * Author: Yann Sionneau
 * This is *very* heavily inspired from fx2lafw support code
 * in src/hardware/fx2lafw
 */

#include "protocol.h"
#include "csr.h"

static int sucrela_start_sampling(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;
	struct uartbone_ctx *ctx = devc->uartbone_ctx;
	
	uartbone_write(ctx, CSR_LA_TRIGGER_MEM_MASK_ADDR, 0);
	uartbone_write(ctx, CSR_LA_TRIGGER_MEM_VALUE_ADDR, 0);
	uartbone_write(ctx, CSR_LA_TRIGGER_MEM_WRITE_ADDR, 1);
	uartbone_write(ctx, CSR_LA_STORAGE_OFFSET_ADDR, 0);
	uartbone_write(ctx, CSR_LA_STORAGE_LENGTH_ADDR, 4095);
	uartbone_write(ctx, CSR_LA_SUBSAMPLER_VALUE_ADDR, 0);
	uartbone_write(ctx, CSR_LA_HSPI_TX_MAX_PACKET_SIZE_ADDR, 4096);
	uartbone_write(ctx, CSR_LA_HSPI_TX_MAX_PACKET_NUM_R_ADDR, 0);
	uartbone_write(ctx, CSR_LA_HSPI_TX_ENABLE_ADDR, 1);
	uartbone_write(ctx, CSR_LA_STORAGE_ENABLE_ADDR, 1);
	uartbone_write(ctx, CSR_LA_TRIGGER_ENABLE_ADDR, 1);

	return 0;
}

SR_PRIV void sucrela_abort_acquisition(struct dev_context *devc)      
{       
        int i;
        
        devc->acq_aborted = TRUE;                                     
        
        for (i = devc->num_transfers - 1; i >= 0; i--) {              
                if (devc->transfers[i])                               
                        libusb_cancel_transfer(devc->transfers[i]);   
        }
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

static void sucrela_send_data_proc(struct sr_dev_inst *sdi,
        uint8_t *data, size_t length, size_t sample_width)
{
        const struct sr_datafeed_logic logic = {
                .length = length,
                .unitsize = sample_width,
                .data = data
        };

        const struct sr_datafeed_packet packet = {
                .type = SR_DF_LOGIC,
                .payload = &logic
        };

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
        struct sr_dev_inst *sdi;
        struct dev_context *devc;
	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset, cur_sample_count, unitsize, processed_samples;
	int pre_trigger_samples;

        sdi = transfer->user_data;
        devc = sdi->priv;

        /*
         * If acquisition has already ended, just free any queued up
         * transfer that come in.
         */
        if (devc->acq_aborted) {
                free_transfer(transfer);
                return;
        }

	sr_err("receive_transfer(): status %s received %d bytes.", libusb_error_name(transfer->status), transfer->actual_length);
	unitsize = 1;
	cur_sample_count = transfer->actual_length / unitsize;

	switch (transfer->status) {
        case LIBUSB_TRANSFER_NO_DEVICE:
                sucrela_abort_acquisition(devc);
                free_transfer(transfer);
                return;
        case LIBUSB_TRANSFER_COMPLETED:
        case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
                break;
        default:
                packet_has_error = TRUE;
                break;
        }

        if (packet_has_error) {
		sr_err("packet has error\n");
                resubmit_transfer(transfer);
                return;
        }
check_trigger:
	if (devc->trigger_fired) {
                if (!devc->limit_samples || devc->sent_samples < devc->limit_samples) {
                        /* Send the incoming transfer to the session bus. */
                        num_samples = cur_sample_count - processed_samples;
                        if (devc->limit_samples && devc->sent_samples + num_samples > devc->limit_samples)
                                num_samples = devc->limit_samples - devc->sent_samples;

                        sucrela_send_data_proc(sdi, (uint8_t *)transfer->buffer + processed_samples * unitsize,
                                num_samples * unitsize, unitsize);
                        devc->sent_samples += num_samples;
                        processed_samples += num_samples;
                }
        } else {
		trigger_offset = soft_trigger_logic_check(devc->stl,
			transfer->buffer + processed_samples * unitsize,
			transfer->actual_length - processed_samples * unitsize,
			&pre_trigger_samples);

		if (trigger_offset > -1) {
			std_session_send_df_frame_begin(sdi);
			devc->sent_samples += pre_trigger_samples;
			num_samples = cur_sample_count - processed_samples - trigger_offset;
			if (devc->limit_samples &&
					devc->sent_samples + num_samples > devc->limit_samples)
				num_samples = devc->limit_samples - devc->sent_samples;

			sucrela_send_data_proc(sdi, (uint8_t *)transfer->buffer
					+ processed_samples * unitsize
					+ trigger_offset * unitsize,
					num_samples * unitsize, unitsize);
			devc->sent_samples += num_samples;
			processed_samples += trigger_offset + num_samples;

			devc->trigger_fired = TRUE;
		}
	}

	const int frame_ended = devc->limit_samples && (devc->sent_samples >= devc->limit_samples);
        const int final_frame = devc->limit_frames && (devc->num_frames >= (devc->limit_frames - 1));

        if (frame_ended) {
                devc->num_frames++;
                devc->sent_samples = 0;
                devc->trigger_fired = FALSE;
                std_session_send_df_frame_end(sdi);

                /* There may be another trigger in the remaining data, go back and check for it */
                if (processed_samples < cur_sample_count) {
                        /* Reset the trigger stage */
                        if (devc->stl)
                                devc->stl->cur_stage = 0;
                        else {
                                std_session_send_df_frame_begin(sdi);
                                devc->trigger_fired = TRUE;
                        }
                        if (!final_frame)
                                goto check_trigger;
                }
        }

	if (frame_ended && final_frame) {
                sucrela_abort_acquisition(devc);
                free_transfer(transfer);
        } else
                resubmit_transfer(transfer);

}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct drv_context *drvc = sdi->driver->context;
	struct timeval tv;

	(void)fd;

	sr_err("we RX data!\n");

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
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

	usb = sdi->conn;

        if ((trigger = sr_session_trigger_get(sdi->session))) {
                int pre_trigger_samples = 0;
                if (devc->limit_samples > 0)
                        pre_trigger_samples = (devc->capture_ratio * devc->limit_samples) / 100;
                devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
                if (!devc->stl)
                        return SR_ERR_MALLOC;
                devc->trigger_fired = FALSE;
        } else {
                std_session_send_df_frame_begin(sdi);
                devc->trigger_fired = TRUE;
        }

	devc->sent_samples = 0;
        devc->acq_aborted = FALSE;
	devc->num_transfers = 1;
	devc->transfers = g_malloc0(sizeof(*devc->transfers) * 100);
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_malloc(BUF_SIZE);
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
			2 | LIBUSB_ENDPOINT_IN, buf, BUF_SIZE,
			receive_transfer, (void *)sdi, 0);
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

	usb_source_add(sdi->session, drvc->sr_ctx, 5000, receive_data, (void *)sdi);

	std_session_send_df_header(sdi);

	sucrela_start_sampling(sdi);

	return ret;
}

SR_PRIV int sucrela_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	int ret = SR_ERR, i, device_count;
	struct sr_usb_dev_inst *usb;
        struct libusb_device_descriptor des;
        struct drv_context *drvc;
	struct dev_context *devc = sdi->priv;
	char connection_id[64];
	char ident_str[256];
	char c;

	drvc = di->context;
        usb = sdi->conn;

        device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
        if (device_count < 0) {
                sr_err("Failed to get device list: %s.",
                       libusb_error_name(device_count));
                return SR_ERR;
        }
        for (i = 0; i < device_count; i++) {
                libusb_get_device_descriptor(devlist[i], &des);

                if ((sdi->status == SR_ST_INITIALIZING) ||
                                (sdi->status == SR_ST_INACTIVE)) {
                        /*
                         * Check device by its physical USB bus/port address.
                         */
                        if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0) {
				sr_err("failed to call usb_get_port_path\n");
                                continue;
			}

                        if (strcmp(sdi->connection_id, connection_id)) {
				sr_err("bad conn id: %s\n", connection_id);
                                /* This is not the one. */
                                continue;
			}
                }
                if ((ret = libusb_open(devlist[i], &usb->devhdl))) {
                        sr_err("Failed to open device: %s.",
                               libusb_error_name(ret));
                        ret = SR_ERR;
                        break;
                }
                if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
                        if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
                                if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
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

        memset(ident_str, '\0', sizeof(ident_str));
        do {
            c = uartbone_read(devc->uartbone_ctx, CSR_IDENTIFIER_MEM_BASE+i*4);
            ident_str[i++] = c;
        } while (c);
        sr_err("ident: %s\n", ident_str);

	return ret;	
}
