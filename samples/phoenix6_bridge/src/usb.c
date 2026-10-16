/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB device with every CDC ACM port of the devicetree (a composite device).
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_DECLARE(phoenix6_bridge_sample);

USBD_DEVICE_DEFINE(sample_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_SAMPLE_USB_VID,
		   CONFIG_SAMPLE_USB_PID);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "CANBusRP2040");
USBD_DESC_PRODUCT_DEFINE(sample_product, "CANBusRP2040 Phoenix 6 bridge");
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn)));

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");
USBD_CONFIGURATION_DEFINE(sample_fs_config, 0, 250, &fs_cfg_desc);
USBD_CONFIGURATION_DEFINE(sample_hs_config, 0, 250, &hs_cfg_desc);

static int add_speed(enum usbd_speed speed, struct usbd_config_node *config)
{
	int err;

	err = usbd_add_configuration(&sample_usbd, speed, config);
	if (err != 0) {
		return err;
	}

	err = usbd_register_all_classes(&sample_usbd, speed, 1, NULL);
	if (err != 0) {
		return err;
	}

	/* Interface association descriptors for the CDC ACM functions */
	return usbd_device_set_code_triple(&sample_usbd, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

int sample_usb_enable(void)
{
	struct usbd_desc_node *const descs[] = {&sample_lang, &sample_mfr, &sample_product,
						IF_ENABLED(CONFIG_HWINFO, (&sample_sn,)) };
	int err;

	ARRAY_FOR_EACH(descs, i) {
		err = usbd_add_descriptor(&sample_usbd, descs[i]);
		if (err != 0) {
			LOG_ERR("USB descriptor (err %d)", err);
			return err;
		}
	}

	if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(&sample_usbd) == USBD_SPEED_HS) {
		err = add_speed(USBD_SPEED_HS, &sample_hs_config);
		if (err != 0) {
			LOG_ERR("USB high speed configuration (err %d)", err);
			return err;
		}
	}

	err = add_speed(USBD_SPEED_FS, &sample_fs_config);
	if (err != 0) {
		LOG_ERR("USB full speed configuration (err %d)", err);
		return err;
	}

	err = usbd_init(&sample_usbd);
	if (err != 0) {
		LOG_ERR("USB init (err %d)", err);
		return err;
	}

	return usbd_enable(&sample_usbd);
}
