/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * End-to-end test of the slcan UART transport: UART emulator <-> slcan
 * thread <-> MCP25625 driver <-> MCP25625 emulator.
 */

#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/drivers/can/mcp25625_emul.h>
#include <canbus_rp2040/slcan.h>

#define MCP25625_NODE DT_NODELABEL(mcp25625)

static const struct device *const can_dev = DEVICE_DT_GET(MCP25625_NODE);
static const struct emul *const can_emul = EMUL_DT_GET(MCP25625_NODE);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(euart0));
static const struct device *const uart_b_dev = DEVICE_DT_GET(DT_NODELABEL(euart1));
static const struct device *const can_b_dev = DEVICE_DT_GET(DT_NODELABEL(mcp25625_b));

static atomic_t rx_events;

static void event_cb(enum slcan_event event, void *user_data)
{
	ARG_UNUSED(user_data);

	if (event == SLCAN_EVENT_RX) {
		atomic_inc(&rx_events);
	}
}

static void host_write(const char *str)
{
	zassert_equal(uart_emul_put_rx_data(uart_dev, (const uint8_t *)str, strlen(str)),
		      strlen(str));
}

/* Collect host-bound output until it matches or the timeout expires */
static void host_expect(const char *expected)
{
	char buf[512] = {0};
	size_t len = 0;
	int64_t deadline = k_uptime_get() + 500;

	while (len < strlen(expected) && k_uptime_get() < deadline) {
		len += uart_emul_get_tx_data(uart_dev, (uint8_t *)&buf[len], sizeof(buf) - 1 - len);
		k_msleep(1);
	}

	zassert_str_equal(buf, expected, "got \"%s\"", buf);
}

static void *slcan_uart_setup(void)
{
	zassert_true(device_is_ready(uart_dev));
	zassert_ok(slcan_uart_start(uart_dev, can_dev, event_cb, NULL));
	zassert_equal(slcan_uart_start(uart_dev, can_dev, NULL, NULL), -EALREADY);

	return NULL;
}

ZTEST(slcan_uart, test_end_to_end)
{
	const struct can_frame rx = {
		.id = 0x0204B401,
		.flags = CAN_FRAME_IDE,
		.dlc = 4,
		.data = {1, 2, 3, 4},
	};
	struct can_frame tx;

	host_write("V\r");
	host_expect("V0101\r");

	/* Commands split across writes are reassembled */
	host_write("S8\rO");
	host_write("\rt12");
	host_write("31AB\r");
	host_expect("\r\r\r");

	k_msleep(10);
	zassert_ok(mcp25625_emul_get_tx_frame(can_emul, &tx));
	zassert_equal(tx.id, 0x123);
	zassert_equal(tx.dlc, 1);
	zassert_equal(tx.data[0], 0xAB);

	zassert_ok(mcp25625_emul_rx_frame(can_emul, &rx));
	host_expect("T0204B401401020304\r");
	zassert_equal(atomic_get(&rx_events), 1);

	/* Many frames in a row */
	for (int i = 0; i < 20; i++) {
		struct can_frame f = {.id = 0x200 + i, .dlc = 0};

		zassert_ok(mcp25625_emul_rx_frame(can_emul, &f));
		k_msleep(1);
	}

	host_expect("t2000\rt2010\rt2020\rt2030\rt2040\rt2050\rt2060\rt2070\rt2080\rt2090\r"
		    "t20A0\rt20B0\rt20C0\rt20D0\rt20E0\rt20F0\rt2100\rt2110\rt2120\rt2130\r");

	host_write("C\r");
	host_expect("\r");
}

ZTEST(slcan_uart, test_instances)
{
#if CONFIG_SLCAN_UART_INSTANCES == 1
	zassert_equal(slcan_uart_start(uart_b_dev, can_b_dev, NULL, NULL), -ENOMEM);
#else
	char buf[16] = {0};
	size_t len = 0;
	int64_t deadline;

	/* A second UART bridges a second CAN controller */
	zassert_ok(slcan_uart_start(uart_b_dev, can_b_dev, NULL, NULL));
	zassert_equal(slcan_uart_start(uart_b_dev, can_b_dev, NULL, NULL), -EALREADY);

	zassert_equal(uart_emul_put_rx_data(uart_b_dev, (const uint8_t *)"S6\rO\rV\rC\r", 9), 9);
	deadline = k_uptime_get() + 500;
	while (len < 9 && k_uptime_get() < deadline) {
		len += uart_emul_get_tx_data(uart_b_dev, (uint8_t *)&buf[len],
					     sizeof(buf) - 1 - len);
		k_msleep(1);
	}
	zassert_str_equal(buf, "\r\rV0101\r\r");

	/* The first instance is unaffected */
	host_write("V\r");
	host_expect("V0101\r");
#endif
}

ZTEST_SUITE(slcan_uart, NULL, slcan_uart_setup, NULL, NULL, NULL);
