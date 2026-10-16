/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Basic CAN example: sends a counter frame every second and prints every
 * received frame. Works with any Zephyr CAN controller; with the MCP25625 it
 * also prints the driver specific statistics.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mcp25625)
#include <canbus_rp2040/drivers/can/mcp25625.h>
#endif

#define COUNTER_ID 0x123

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

CAN_MSGQ_DEFINE(rx_msgq, 16);

static const char *state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

static void state_changed(const struct device *dev, enum can_state state,
			  struct can_bus_err_cnt err_cnt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	printk("CAN state: %s (TEC %u, REC %u)\n", state_name(state), err_cnt.tx_err_cnt,
	       err_cnt.rx_err_cnt);
}

static void tx_done(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (error != 0) {
		printk("transmission failed (err %d)\n", error);
	}
}

static void print_frame(const struct can_frame *frame)
{
	printk("RX %s 0x%0*x [%u]", (frame->flags & CAN_FRAME_IDE) != 0U ? "ext" : "std",
	       (frame->flags & CAN_FRAME_IDE) != 0U ? 8 : 3, frame->id, frame->dlc);

	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		printk(" remote request");
	} else {
		for (uint8_t i = 0; i < frame->dlc; i++) {
			printk(" %02x", frame->data[i]);
		}
	}

	printk("\n");
}

int main(void)
{
	const struct can_filter all_std = {.id = 0, .mask = 0, .flags = 0};
	const struct can_filter all_ext = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};
	struct can_frame frame = {.id = COUNTER_ID, .dlc = 4};
	uint32_t counter = 0;
	int64_t next_tx = 0;
	int err;

	if (!device_is_ready(can_dev)) {
		printk("CAN controller %s not ready\n", can_dev->name);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SAMPLE_CAN_LOOPBACK)) {
		err = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
		if (err != 0) {
			printk("loopback mode not supported (err %d)\n", err);
			return 0;
		}
	}

	(void)can_add_rx_filter_msgq(can_dev, &rx_msgq, &all_std);
	(void)can_add_rx_filter_msgq(can_dev, &rx_msgq, &all_ext);
	can_set_state_change_callback(can_dev, state_changed, NULL);

	err = can_start(can_dev);
	if (err != 0) {
		printk("failed to start CAN controller (err %d)\n", err);
		return 0;
	}

	printk("CAN controller %s started\n", can_dev->name);

	while (true) {
		struct can_frame rx;
		int64_t now = k_uptime_get();

		if (now >= next_tx) {
			sys_put_be32(counter++, frame.data);
			err = can_send(can_dev, &frame, K_NO_WAIT, tx_done, NULL);
			if (err != 0 && err != -EAGAIN) {
				printk("failed to queue frame (err %d)\n", err);
			}

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_mcp25625)
			struct mcp25625_stats stats;

			if (mcp25625_get_stats(can_dev, &stats) == 0) {
				printk("stats: rx %u tx %u overruns %u errors %u bus-off %u\n",
				       stats.rx_frames, stats.tx_frames, stats.rx_overruns,
				       stats.message_errors, stats.bus_off);
			}
#endif
			next_tx = now + 1000;
		}

		if (k_msgq_get(&rx_msgq, &rx, K_MSEC(next_tx - now)) == 0) {
			print_frame(&rx);
		}
	}

	return 0;
}
