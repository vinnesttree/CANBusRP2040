/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests of the slcan protocol engine, running on top of the MCP25625 driver
 * and its emulator.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/drivers/can/mcp25625_emul.h>
#include <canbus_rp2040/slcan.h>

#define MCP25625_NODE DT_NODELABEL(mcp25625)

#define REG_CANCTRL 0x0F
#define REG_CNF1    0x2A

#define OPMODE_NORMAL      0
#define OPMODE_LISTEN_ONLY 3
#define OPMODE_CONFIG      4

static const struct device *const can_dev = DEVICE_DT_GET(MCP25625_NODE);
static const struct emul *const emul = EMUL_DT_GET(MCP25625_NODE);

static struct slcan slcan;
static char out[1024];
static size_t out_len;
static int events[SLCAN_EVENT_ERROR + 1];

static void capture(const uint8_t *data, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);

	zassert_true(out_len + len < sizeof(out), "output buffer overflow");
	memcpy(&out[out_len], data, len);
	out_len += len;
	out[out_len] = '\0';
}

static void event_cb(enum slcan_event event, void *user_data)
{
	ARG_UNUSED(user_data);

	events[event]++;
}

static void cmd(const char *str)
{
	slcan_receive(&slcan, (const uint8_t *)str, strlen(str));
}

static void clear_output(void)
{
	out_len = 0;
	out[0] = '\0';
}

#define expect_output(expected)                                                                    \
	do {                                                                                       \
		zassert_str_equal(out, expected, "got \"%s\"", out);                               \
		clear_output();                                                                    \
	} while (0)

/* Wait for the driver to deliver frames and forward them */
static void process(void)
{
	k_msleep(5);
	(void)slcan_process(&slcan);
}

static void *slcan_setup(void)
{
	zassert_true(device_is_ready(can_dev));
	zassert_ok(slcan_init(&slcan, can_dev, capture, NULL));
	slcan_set_event_callback(&slcan, event_cb, NULL);

	return NULL;
}

static void slcan_before(void *fixture)
{
	ARG_UNUSED(fixture);

	cmd("C\rS8\rZ0\rA1\rM0\r");
	mcp25625_emul_set_tx_ack(emul, true);
	mcp25625_emul_set_error_counters(emul, 0, 0);
	mcp25625_emul_clear_tx_frames(emul);
	(void)slcan_process(&slcan);
	clear_output();
	memset(events, 0, sizeof(events));
}

ZTEST(slcan, test_format_frame)
{
	const struct {
		struct can_frame frame;
		bool ts;
		uint16_t ts_ms;
		const char *expected;
	} cases[] = {
		{{.id = 0x123, .dlc = 2, .data = {0xDE, 0xAD}}, false, 0, "t1232DEAD\r"},
		{{.id = 0x7FF, .dlc = 0}, false, 0, "t7FF0\r"},
		{{.id = 0x0204ABCD,
		  .flags = CAN_FRAME_IDE,
		  .dlc = 8,
		  .data = {0, 1, 2, 3, 0xFC, 0xFD, 0xFE, 0xFF}},
		 false,
		 0,
		 "T0204ABCD800010203FCFDFEFF\r"},
		{{.id = 0x12, .flags = CAN_FRAME_RTR, .dlc = 4}, false, 0, "r0124\r"},
		{{.id = 0x1, .flags = CAN_FRAME_IDE | CAN_FRAME_RTR, .dlc = 0},
		 false,
		 0,
		 "R000000010\r"},
		{{.id = 0x123, .dlc = 1, .data = {0x55}}, true, 59999, "t123155EA5F\r"},
	};
	char buf[SLCAN_MTU + 1];

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		int len = slcan_format_frame(&cases[i].frame, cases[i].ts, cases[i].ts_ms, buf,
					     sizeof(buf));

		zassert_equal(len, strlen(cases[i].expected), "case %zu", i);
		zassert_str_equal(buf, cases[i].expected, "case %zu: \"%s\"", i, buf);
	}

	/* The longest message fits into SLCAN_MTU */
	const struct can_frame longest = {.id = 0x1FFFFFFF, .flags = CAN_FRAME_IDE, .dlc = 8};

	zassert_true(slcan_format_frame(&longest, true, 0, buf, sizeof(buf)) > 0);
	zassert_equal(slcan_format_frame(&longest, true, 0, buf, 10), -ENOSPC);
}

ZTEST(slcan, test_parse_frame)
{
	struct can_frame f;

	zassert_ok(slcan_parse_frame("t1232DEAD", 9, &f));
	zassert_equal(f.id, 0x123);
	zassert_equal(f.flags, 0);
	zassert_equal(f.dlc, 2);
	zassert_equal(f.data[0], 0xDE);
	zassert_equal(f.data[1], 0xAD);

	zassert_ok(slcan_parse_frame("T1FFFFFFF1ab", 12, &f));
	zassert_equal(f.id, 0x1FFFFFFF);
	zassert_equal(f.flags, CAN_FRAME_IDE);
	zassert_equal(f.data[0], 0xAB);

	zassert_ok(slcan_parse_frame("r7FF8", 5, &f));
	zassert_equal(f.flags, CAN_FRAME_RTR);
	zassert_equal(f.dlc, 8);

	zassert_ok(slcan_parse_frame("R000000010", 10, &f));
	zassert_equal(f.flags, CAN_FRAME_IDE | CAN_FRAME_RTR);

	/* Malformed commands */
	static const char *const invalid[] = {
		"t123",       /* missing DLC */
		"t1239",      /* DLC out of range */
		"t1232DEA",   /* too short */
		"t1232DEADB", /* too long */
		"t8000",      /* standard ID out of range */
		"T200000000", /* extended ID out of range */
		"t12G0",      /* invalid hex digit */
		"t1231XX",    /* invalid data */
		"r12310",     /* data in a remote frame */
		"x1230",      /* unknown frame type */
	};

	for (size_t i = 0; i < ARRAY_SIZE(invalid); i++) {
		zassert_equal(slcan_parse_frame(invalid[i], strlen(invalid[i]), &f), -EINVAL,
			      "accepted \"%s\"", invalid[i]);
	}
}

ZTEST(slcan, test_btr_conversion)
{
	uint32_t bitrate;
	uint16_t sp;

	/* Common Lawicel values for a 16 MHz SJA1000 */
	zassert_ok(slcan_btr_to_bitrate(0x03, 0x1C, &bitrate, &sp));
	zassert_equal(bitrate, 125000);
	zassert_equal(sp, 875);

	zassert_ok(slcan_btr_to_bitrate(0x00, 0x1C, &bitrate, &sp));
	zassert_equal(bitrate, 500000);

	zassert_ok(slcan_btr_to_bitrate(0x00, 0x14, &bitrate, &sp));
	zassert_equal(bitrate, 1000000);
	zassert_equal(sp, 750);

	/* 8 MHz / (3 * 7) is not an integer bitrate */
	zassert_equal(slcan_btr_to_bitrate(0x02, 0x04, &bitrate, &sp), -EINVAL);
}

ZTEST(slcan, test_commands_while_closed)
{
	cmd("V\r");
	expect_output("V0101\r");

	cmd("N\r");
	expect_output("N0000\r");

	/* Status and transmission require an open channel */
	cmd("F\r");
	expect_output("\a");
	cmd("t1230\r");
	expect_output("\a");

	/* Unknown and malformed commands */
	cmd("X\r");
	expect_output("\a");
	cmd("S9\r");
	expect_output("\a");
	cmd("S\r");
	expect_output("\a");
	cmd("Z2\r");
	expect_output("\a");

	/* Configuration */
	cmd("S4\r");
	expect_output("\r");
	cmd("s031C\r");
	expect_output("\r");
	cmd("M00000000\rmFFFFFFFF\r");
	expect_output("\r\r");
	cmd("m00000000\r");
	expect_output("\a");

	/* Closing a closed channel and empty lines */
	cmd("C\r\r\n");
	expect_output("\r");
}

ZTEST(slcan, test_open_configures_controller)
{
	cmd("S6\rO\r");
	expect_output("\r\r");
	zassert_true(slcan_is_open(&slcan));
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_NORMAL);
	zassert_equal(events[SLCAN_EVENT_OPEN], 1);

	/* 500 kbit/s from a 16 MHz oscillator: 16 time quanta, prescaler 1 */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_CNF1) & 0x3F, 0);

	/* No reconfiguration while open */
	cmd("O\rS8\rZ1\rA0\rM1\r");
	expect_output("\a\a\a\a\a");

	cmd("C\r");
	expect_output("\r");
	zassert_false(slcan_is_open(&slcan));
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_CONFIG);
	zassert_equal(events[SLCAN_EVENT_CLOSE], 1);

	/* SJA1000 bit timing: 125 kbit/s */
	cmd("s031C\rO\r");
	expect_output("\r\r");
	zassert_equal(mcp25625_emul_get_reg(emul, REG_CNF1) & 0x3F, 3);
}

ZTEST(slcan, test_transmit)
{
	struct can_frame f;

	cmd("O\r");
	expect_output("\r");

	cmd("T0204ABCD2AA55\r");
	expect_output("\r");
	k_msleep(5);
	zassert_ok(mcp25625_emul_get_tx_frame(emul, &f));
	zassert_equal(f.id, 0x0204ABCD);
	zassert_equal(f.flags, CAN_FRAME_IDE);
	zassert_equal(f.dlc, 2);
	zassert_equal(f.data[0], 0xAA);
	zassert_equal(f.data[1], 0x55);

	cmd("r1238\r");
	expect_output("\r");
	k_msleep(5);
	zassert_ok(mcp25625_emul_get_tx_frame(emul, &f));
	zassert_equal(f.id, 0x123);
	zassert_equal(f.flags, CAN_FRAME_RTR);
	zassert_equal(f.dlc, 8);

	/* Malformed frame */
	cmd("t12\r");
	expect_output("\a");

	zassert_equal(events[SLCAN_EVENT_TX], 2);
}

ZTEST(slcan, test_receive)
{
	const struct can_frame std = {.id = 0x123, .dlc = 2, .data = {0xDE, 0xAD}};
	const struct can_frame ext = {
		.id = 0x02041401,
		.flags = CAN_FRAME_IDE,
		.dlc = 3,
		.data = {1, 2, 3},
	};
	const struct can_frame rtr = {.id = 0x10, .flags = CAN_FRAME_RTR, .dlc = 1};

	/* Frames are not forwarded while the channel is closed */
	zassert_equal(mcp25625_emul_rx_frame(emul, &std), -EACCES);

	cmd("O\r");
	expect_output("\r");

	zassert_ok(mcp25625_emul_rx_frame(emul, &std));
	process();
	zassert_ok(mcp25625_emul_rx_frame(emul, &ext));
	process();
	zassert_ok(mcp25625_emul_rx_frame(emul, &rtr));
	process();

	expect_output("t1232DEAD\rT020414013010203\rr0101\r");
	zassert_equal(events[SLCAN_EVENT_RX], 3);
}

ZTEST(slcan, test_receive_timestamps)
{
	const struct can_frame std = {.id = 0x321, .dlc = 1, .data = {0x42}};
	unsigned int ts;

	cmd("Z1\rO\r");
	expect_output("\r\r");

	zassert_ok(mcp25625_emul_rx_frame(emul, &std));
	process();

	zassert_equal(out_len, strlen("t321142TTTT\r"), "got \"%s\"", out);
	zassert_mem_equal(out, "t321142", 7);
	zassert_equal(sscanf(&out[7], "%4x", &ts), 1);
	zassert_true(ts < 60000, "timestamp %u out of range", ts);
	clear_output();
}

ZTEST(slcan, test_listen_only)
{
	cmd("L\r");
	expect_output("\r");
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_LISTEN_ONLY);

	cmd("t1230\r");
	expect_output("\a");

	/* After closing, O opens in normal mode again */
	cmd("C\rO\r");
	expect_output("\r\r");
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_NORMAL);

	/* CANable silent mode */
	cmd("C\rM1\rO\r");
	expect_output("\r\r\r");
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_LISTEN_ONLY);
}

ZTEST(slcan, test_one_shot_and_status)
{
	cmd("A0\rO\r");
	expect_output("\r\r");
	zassert_true((mcp25625_emul_get_reg(emul, REG_CANCTRL) & BIT(3)) != 0, "OSM not set");

	cmd("F\r");
	expect_output("F00\r");

	/* Without acknowledgement the single attempt fails with a bus error */
	mcp25625_emul_set_tx_ack(emul, false);
	cmd("t1230\r");
	expect_output("\r");
	k_msleep(10);

	cmd("F\r");
	expect_output("F80\r");

	/* Flags are cleared by reading them */
	cmd("F\r");
	expect_output("F00\r");
}

ZTEST(slcan, test_state_reports)
{
	cmd("O\r");
	expect_output("\r");

	mcp25625_emul_set_error_counters(emul, 100, 0);
	process();
	/* s<state><rx errors><tx errors> */
	expect_output("sw000100\r");
	zassert_equal(events[SLCAN_EVENT_ERROR], 1);

	cmd("F\r");
	expect_output("F04\r");

	mcp25625_emul_set_error_counters(emul, 256, 0);
	process();
	expect_output("sb000255\r");

	mcp25625_emul_set_error_counters(emul, 0, 0);
	k_msleep(100);
	(void)slcan_process(&slcan);
	expect_output("sa000000\r");
}

ZTEST(slcan, test_receive_overrun)
{
	struct can_frame f = {.id = 0x100, .dlc = 1};

	cmd("O\r");
	expect_output("\r");

	/* CONFIG_SLCAN_RX_QUEUE_SIZE is 4: the last two frames are dropped */
	for (int i = 0; i < 6; i++) {
		f.data[0] = (uint8_t)i;
		zassert_ok(mcp25625_emul_rx_frame(emul, &f));
		k_msleep(1);
	}

	process();
	expect_output("e1o\rt100100\rt100101\rt100102\rt100103\r");

	cmd("F\r");
	expect_output("F09\r");
}

ZTEST(slcan, test_line_overflow)
{
	cmd("t1230000000000000000000000000000000000000000\r");
	expect_output("\a");

	cmd("V\r");
	expect_output("V0101\r");
}

ZTEST_SUITE(slcan, NULL, slcan_setup, slcan_before, NULL, NULL);
