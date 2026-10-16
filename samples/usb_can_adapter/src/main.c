/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CANable-style USB-to-CAN adapter: the MCP25625 CAN controller is exposed
 * over USB CDC ACM using the slcan protocol.
 *
 * Status LED (led0), modelled on the CANable LEDs:
 *   - channel closed: short blink once per second
 *   - channel open:   on, briefly off for every received/transmitted frame
 *   - error:          fast blinking for two seconds (bus error, lost frames)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <canbus_rp2040/slcan.h>

LOG_MODULE_REGISTER(usb_can_adapter, LOG_LEVEL_INF);

#define LED_ACTIVITY_MS 25
#define LED_ERROR_MS    2000

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(slcan_cdc_acm));

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(led0))
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAVE_LED 1
#else
#define HAVE_LED 0
#endif

static atomic_t channel_open;
static atomic_t last_activity_ms;
static atomic_t last_error_ms;

static void slcan_event(enum slcan_event event, void *user_data)
{
	uint32_t now = k_uptime_get_32();

	ARG_UNUSED(user_data);

	switch (event) {
	case SLCAN_EVENT_OPEN:
		atomic_set(&channel_open, 1);
		LOG_INF("channel opened");
		break;
	case SLCAN_EVENT_CLOSE:
		atomic_set(&channel_open, 0);
		LOG_INF("channel closed");
		break;
	case SLCAN_EVENT_RX:
	case SLCAN_EVENT_TX:
		atomic_set(&last_activity_ms, (atomic_val_t)now);
		break;
	case SLCAN_EVENT_ERROR:
		atomic_set(&last_error_ms, (atomic_val_t)now);
		break;
	}
}

static bool led_state(uint32_t now)
{
	uint32_t error_age = now - (uint32_t)atomic_get(&last_error_ms);
	uint32_t activity_age = now - (uint32_t)atomic_get(&last_activity_ms);

	if (atomic_get(&last_error_ms) != 0 && error_age < LED_ERROR_MS) {
		return ((now / 50U) % 2U) == 0U;
	}

	if (atomic_get(&channel_open) == 0) {
		return (now % 1000U) < 50U;
	}

	return activity_age >= LED_ACTIVITY_MS;
}

int main(void)
{
	int err;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN controller %s not ready (check wiring and osc-freq)", can_dev->name);
		return 0;
	}

#if HAVE_LED
	if (gpio_is_ready_dt(&led)) {
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}
#endif

	err = slcan_uart_start(uart_dev, can_dev, slcan_event, NULL);
	if (err != 0) {
		LOG_ERR("failed to start slcan on %s (err %d)", uart_dev->name, err);
		return 0;
	}

	LOG_INF("slcan USB-CAN adapter ready on %s", can_dev->name);

	while (true) {
#if HAVE_LED
		(void)gpio_pin_set_dt(&led, led_state(k_uptime_get_32()) ? 1 : 0);
#endif
		k_msleep(5);
	}

	return 0;
}
