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
	SR_CONF_CONTINUOUS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
};

static const double threshold_ranges[][2] = {
	{ 3.3, 3.3},
};


static const uint64_t samplerates[] = {
	SR_MHZ(256),
	SR_MHZ(128),
	SR_MHZ(64),
	SR_MHZ(32),
	SR_MHZ(16),
	SR_MHZ(8),
	SR_MHZ(4),
	SR_MHZ(2),
	SR_MHZ(1),
	SR_KHZ(500),
	SR_KHZ(250),
	SR_KHZ(125),
};

static const char *channel_names[] = {
	"0", "1", "2",	"3",  "4",  "5",  "6",	"7",
	"8", "9", "10", "11", "12", "13", "14", "15",
};


static char *key_to_str(uint32_t key)
{
	switch (key) {
	case SR_CONF_SAMPLERATE:
		return "SR_CONF_SAMPLERATE";
	case SR_CONF_LIMIT_SAMPLES:
		return "SR_CONF_LIMIT_SAMPLES";
	case SR_CONF_CONN:
		return "SR_CONF_CONN";
	case SR_CONF_SCAN_OPTIONS:
		return "SR_CONF_SCAN_OPTIONS";
	case SR_CONF_DEVICE_OPTIONS:
		return "SR_CONF_DEVICE_OPTIONS";
	case SR_CONF_TRIGGER_MATCH:
		return "SR_CONF_TRIGGER_MATCH";
	case SR_CONF_VOLTAGE_THRESHOLD:
		return "SR_CONF_VOLTAGE_THRESHOLD";
	case SR_CONF_CONTINUOUS:
		return "SR_CONF_CONTINUOUS";
	default:
		sr_err("unknown key: %d\n", key);
		return "unknown key ?!";
	}
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc = sdi->priv;

	sr_err("config_get(key=%s, sdi=%p)\n", key_to_str(key), sdi);

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
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_tuple_double(3.3, 3.3);
		break;
	case SR_CONF_CONTINUOUS:
		*data = g_variant_new_boolean(devc->continuous);
		break;

	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static void update_probe_number(const struct sr_dev_inst *sdi) {
	struct dev_context *devc = sdi->priv;

	if (devc->oversampling_ratio == 1)
		return;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	int idx;

	sr_err("config_set(key=%s, sdi=%p)\n", key_to_str(key), sdi);

	(void)cg;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		devc->dig_samplerate = g_variant_get_uint64(data);
		sr_err("set to %"PRIu64"\n", devc->dig_samplerate);
		devc->oversampling_ratio = 1;
		if (devc->dig_samplerate == 2 * devc->max_samplerate) {
			// Let's enable DDRx1 inputs
			if (devc->oversampler_phy_ratio < 2) {
				sr_err("ERROR: Impossible to oversample, oversampler is synthesized with phy_ratio of 1 in the SoC\n");
				return SR_ERR_SAMPLERATE;
			}
			sr_err("oversampling x2\n");
			devc->oversampling_ratio = 2;
		}
		if (devc->dig_samplerate == 4 * devc->max_samplerate) {
			// Let's enable DDRx2 inputs
			if (devc->oversampler_phy_ratio < 4) {
				sr_err("ERROR: Impossible to oversample, oversampler is synthesized with phy_ratio of %d in the SoC\n", devc->oversampler_phy_ratio);
				return SR_ERR_SAMPLERATE;
			}
			sr_err("oversampling x4\n");
			devc->oversampling_ratio = 4;
		}
		update_probe_number(sdi);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		idx = std_double_tuple_idx(data, ARRAY_AND_SIZE(threshold_ranges));
		if (idx < 0)
			return SR_ERR_ARG;
		break;
	case SR_CONF_CONTINUOUS:
		devc->continuous = g_variant_get_boolean(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	sr_err("config_list(key=%s, sdi=%p)\n", key_to_str(key), sdi);

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(ARRAY_AND_SIZE(samplerates));
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_thresholds(ARRAY_AND_SIZE(threshold_ranges));
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
	GSList *devices;
	struct sr_dev_inst *sdi;
	struct libusb_device_descriptor des;
	char connection_id[64];
	const char *conn;

	sr_err("scan()\n");

	drvc = di->context;
	drvc->instances = NULL;
	devices = NULL;

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


		if (scan_firmware(devlist[i]))
			sr_err("Found a SucréLA device.");
		else
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup("YS");
		sdi->model = g_strdup("SucréLA");
		sdi->connection_id = g_strdup(connection_id);

		sdi->status = SR_ST_INACTIVE;
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
						libusb_get_device_address(devlist[i]), NULL);

		devc = g_malloc0(sizeof(struct dev_context));
		sdi->priv = devc;

		for (unsigned int j = 0; j < 16; j++) {
			sr_err("creating channel %d %s\n", j, channel_names[j]);
			sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE,
				       channel_names[j]);
		}


		devices = g_slist_append(devices, sdi);
	}

	return std_scan_complete(di, devices);
}

SR_PRIV int sucrela_dev_open(
	struct sr_dev_inst *sdi,
	struct sr_dev_driver *di);

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct sr_dev_driver *di;

	sr_err("dev_open()\n");

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

	return SR_OK;
}

static int sucrela_dev_close(struct sr_dev_inst *sdi)
{
	sr_err("sucrela_dev_close()\n");
	(void)sdi;
	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	return sucrela_abort_acquisition(sdi->priv);
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
