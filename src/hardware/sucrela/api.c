/*
 * SucréLA hardware support code
 * Author: Yann Sionneau
 * This is *very* heavily inspired from fx2lafw support code
 * in src/hardware/fx2lafw
 */

#include <config.h>
#include <glib.h>
#include "protocol.h"

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
};

static const uint64_t samplerates[] = {
	SR_MHZ(1),
	SR_MHZ(2),
	SR_KHZ(2500),
	SR_MHZ(10),
	SR_MHZ(25),
	SR_MHZ(50),
};

static const char *channel_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
};

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc = sdi->priv;

	(void)cg;

	switch (key) {
	case SR_CONF_CONN:
		if (!sdi || !sdi->conn)
			return SR_ERR_ARG;
		usb = sdi->conn;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_SAMPLERATE:
		if (!sdi)
			return SR_ERR;
		devc = sdi->priv;
		*data = g_variant_new_uint64(devc->dig_samplerate);
		break;
       case SR_CONF_LIMIT_SAMPLES:
                *data = g_variant_new_uint64(devc->limit_samples);
                break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	(void)cg;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		devc->dig_samplerate = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(ARRAY_AND_SIZE(samplerates));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static gboolean scan_firmware(libusb_device *dev)
{
	struct libusb_device_descriptor des;
	struct libusb_device_handle *hdl;
	gboolean ret;
	unsigned char strdesc[64];

	hdl = NULL;
	ret = FALSE;

	libusb_get_device_descriptor(dev, &des);

	if (libusb_open(dev, &hdl) != 0)
		goto out;

	if (libusb_get_string_descriptor_ascii(hdl,
		des.iManufacturer, strdesc, sizeof(strdesc)) < 0)
		goto out;
	if (strcmp((const char *)strdesc, "HydraBus"))
		goto out;

	if (libusb_get_string_descriptor_ascii(hdl,
		des.iProduct, strdesc, sizeof(strdesc)) < 0)
		goto out;

	sr_err("iProduct: %s\n", strdesc);

	ret = TRUE;

out:
	if (hdl)
		libusb_close(hdl);

	return ret;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	libusb_device **devlist;
	GSList *devices, *conn_devices;
	struct sr_dev_inst *sdi;
	struct libusb_device_descriptor des;
	char connection_id[64];
	const char *conn;

	drvc = di->context;
	drvc->instances = NULL;
	conn = NULL;

	for (GSList *l = options; l; l = l->next) {
		struct sr_config *src = l->data;

		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (unsigned int i = 0; devlist[i]; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != 0x16c0 || des.idProduct != 0x05dc)
			continue;

                if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
                        continue;


		if (!scan_firmware(devlist[i]))
			sr_info("Found a SucréLA device.");


		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup("Yann Sionneau");
		sdi->model = g_strdup("SucréLA");
		sdi->connection_id = g_strdup(connection_id);

		for (unsigned int j = 0; j < ARRAY_SIZE(channel_names); j++)
			sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE,
				       channel_names[j]);

		sr_dbg("Found a SucréLA device.");
		sdi->status = SR_ST_INACTIVE;
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
						libusb_get_device_address(devlist[i]), NULL);

		devc = g_malloc0(sizeof(struct dev_context));
		sdi->priv = devc;
		devices = g_slist_append(devices, sdi);
	}

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct sr_dev_driver *di;

	di = sdi->driver;
	usb = sdi->conn;

	ret = sucrela_dev_open(sdi, di);
	if (ret) {
		sr_err("Unable to open the device\n");
		return SR_ERR;
	}

        ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
        if (ret != 0) {
                switch (ret) {
                case LIBUSB_ERROR_BUSY:
                        sr_err("Unable to claim USB interface. Another "
                               "program or driver has already claimed it.");
                        break;
                case LIBUSB_ERROR_NO_DEVICE:
                        sr_err("Device has been disconnected.");
                        break;
                default:
                        sr_err("Unable to claim interface: %s.",
                               libusb_error_name(ret));
                        break;
                }

                return SR_ERR;
        }

	return ret;
}

static int sucrela_dev_close(struct sr_dev_inst *sdi)
{

}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	sucrela_abort_acquisition(sdi->priv);
}

static struct sr_dev_driver sucrela_driver_info = {
	.name = "sucrela",
	.longname = "SucréLA Logic Analyzer",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = sucrela_dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sucrela_driver_info);
