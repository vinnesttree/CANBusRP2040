/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phoenix 6 bridge sample. The board shows up as two USB serial ports:
 *
 *   1. slcan: the host's SocketCAN interface (slcand) on the MCP25625
 *   2. the link to the host bridge, which runs Phoenix 6 requests from this
 *      firmware on the Talon FX devices of that CAN bus
 *
 * The p6 shell commands (console UART) send any Phoenix 6 request by name.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <canbus_rp2040/phoenix6.h>
#include <canbus_rp2040/slcan.h>

LOG_MODULE_REGISTER(phoenix6_bridge_sample, LOG_LEVEL_INF);

int sample_usb_enable(void);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const slcan_uart = DEVICE_DT_GET(DT_NODELABEL(slcan_cdc_acm));
static const struct device *const bridge_uart = DEVICE_DT_GET(DT_NODELABEL(phoenix6_uart));

int main(void)
{
	int err;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN controller %s not ready", can_dev->name);
		return 0;
	}

	err = sample_usb_enable();
	if (err != 0) {
		LOG_ERR("USB failed (err %d)", err);
		return 0;
	}

	err = slcan_uart_start(slcan_uart, can_dev, NULL, NULL);
	if (err != 0) {
		LOG_ERR("slcan failed (err %d)", err);
	}

	err = phoenix6_bridge_start(bridge_uart);
	if (err != 0) {
		LOG_ERR("Phoenix 6 bridge client failed (err %d)", err);
		return 0;
	}

	LOG_INF("Phoenix 6 %s API, waiting for the host bridge", PHOENIX6_GENERATED_VERSION);

	return 0;
}
