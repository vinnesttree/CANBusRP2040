/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MCP25625 driver tests against the MCP25625 SPI emulator.
 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/drivers/can/mcp25625.h>
#include <canbus_rp2040/drivers/can/mcp25625_emul.h>

/* Register addresses and values used by the tests (see the MCP25625 data sheet) */
#define REG_RXF0SIDH 0x00
#define REG_RXF2SIDH 0x08
#define REG_CANCTRL  0x0F
#define REG_RXM0SIDH 0x20
#define REG_RXM1SIDH 0x24
#define REG_CNF3     0x28
#define REG_CNF2     0x29
#define REG_CNF1     0x2A
#define REG_CANINTE  0x2B
#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70

#define OPMODE_NORMAL      0
#define OPMODE_SLEEP       1
#define OPMODE_LOOPBACK    2
#define OPMODE_LISTEN_ONLY 3
#define OPMODE_CONFIG      4

#define CANINTE_DEFAULT 0xA7 /* MERRE | ERRIE | TX0IE | RX1IE | RX0IE */
#define CNF2_SAM        BIT(6)

#define MCP25625_NODE DT_NODELABEL(mcp25625)

static const struct device *const can_dev = DEVICE_DT_GET(MCP25625_NODE);
static const struct emul *const emul = EMUL_DT_GET(MCP25625_NODE);
static const struct gpio_dt_spec standby_gpio = GPIO_DT_SPEC_GET(MCP25625_NODE, standby_gpios);
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET(MCP25625_NODE, reset_gpios);

#define TIMEOUT K_MSEC(200)

CAN_MSGQ_DEFINE(rx_msgq, 8);

struct tx_result {
	struct k_sem done;
	int status;
};

static struct tx_result tx_result;

static void tx_callback(const struct device *dev, int error, void *user_data)
{
	struct tx_result *res = user_data;

	ARG_UNUSED(dev);

	res->status = error;
	k_sem_give(&res->done);
}

struct state_event {
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
};

K_MSGQ_DEFINE(state_msgq, sizeof(struct state_event), 8, 4);

static void state_callback(const struct device *dev, enum can_state state,
			   struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct state_event ev = {.state = state, .err_cnt = err_cnt};

	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	(void)k_msgq_put(&state_msgq, &ev, K_NO_WAIT);
}

static const struct can_filter std_filter_all = {.id = 0, .mask = 0, .flags = 0};
static const struct can_filter ext_filter_all = {.id = 0, .mask = 0, .flags = CAN_FILTER_IDE};

/* Filters added by a test; removed after each test even if the test fails */
static int filter_ids[8];
static size_t filter_count;

static int add_msgq_filter(const struct can_filter *filter)
{
	int id = can_add_rx_filter_msgq(can_dev, &rx_msgq, filter);

	zassert_true(id >= 0, "failed to add filter (err %d)", id);
	zassert_true(filter_count < ARRAY_SIZE(filter_ids));
	filter_ids[filter_count++] = id;

	return id;
}

static void remove_filter(int id)
{
	for (size_t i = 0; i < filter_count; i++) {
		if (filter_ids[i] == id) {
			filter_ids[i] = filter_ids[--filter_count];
			break;
		}
	}

	can_remove_rx_filter(can_dev, id);
}

static void assert_frames_equal(const struct can_frame *a, const struct can_frame *b)
{
	zassert_equal(a->id, b->id, "ID mismatch 0x%x != 0x%x", a->id, b->id);
	zassert_equal(a->flags, b->flags, "flags mismatch 0x%x != 0x%x", a->flags, b->flags);
	zassert_equal(a->dlc, b->dlc, "DLC mismatch %u != %u", a->dlc, b->dlc);
	if ((a->flags & CAN_FRAME_RTR) == 0U) {
		zassert_mem_equal(a->data, b->data, a->dlc, "data mismatch");
	}
}

static void start_in_mode(can_mode_t mode)
{
	zassert_ok(can_set_mode(can_dev, mode));
	zassert_ok(can_start(can_dev));
}

static void send_with_callback(const struct can_frame *frame)
{
	k_sem_reset(&tx_result.done);
	tx_result.status = INT_MIN;
	zassert_ok(can_send(can_dev, frame, TIMEOUT, tx_callback, &tx_result));
}

static int wait_tx_result(void)
{
	zassert_ok(k_sem_take(&tx_result.done, TIMEOUT), "TX callback not called");
	return tx_result.status;
}

static void *mcp25625_setup(void)
{
	k_sem_init(&tx_result.done, 0, 1);
	zassert_true(device_is_ready(can_dev), "MCP25625 not ready");

	return NULL;
}

static void mcp25625_before(void *fixture)
{
	ARG_UNUSED(fixture);

	(void)can_stop(can_dev);
	can_set_state_change_callback(can_dev, NULL, NULL);
	zassert_ok(mcp25625_set_hw_acceptance(can_dev, NULL));
	zassert_ok(can_set_mode(can_dev, CAN_MODE_NORMAL));
	zassert_ok(can_set_bitrate(can_dev, 1000000));
	mcp25625_emul_set_tx_ack(emul, true);
	mcp25625_emul_set_tx_hold(emul, false);
	mcp25625_emul_set_tx_lose_arbitration(emul, false);
	mcp25625_emul_set_error_counters(emul, 0, 0);
	mcp25625_emul_clear_tx_frames(emul);
	k_msgq_purge(&rx_msgq);
	k_msgq_purge(&state_msgq);
}

static void mcp25625_after(void *fixture)
{
	ARG_UNUSED(fixture);

	(void)can_stop(can_dev);

	while (filter_count > 0) {
		remove_filter(filter_ids[filter_count - 1]);
	}

	zassert_equal(mcp25625_emul_get_config_violations(emul), 0,
		      "driver wrote configuration registers outside configuration mode");
}

ZTEST(mcp25625, test_init_configuration)
{
	uint8_t canctrl = mcp25625_emul_get_reg(emul, REG_CANCTRL);

	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_CONFIG);
	/* CLKOUT disabled (clkout-prescaler defaults to 0) */
	zassert_equal(canctrl & 0x07, 0, "CLKOUT not disabled (CANCTRL 0x%02x)", canctrl);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_CANINTE), CANINTE_DEFAULT);
	/* Receive any message, RXB0 rolls over into RXB1 */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB0CTRL), 0x64);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB1CTRL), 0x60);
	/* Transceiver in standby while stopped, RESET released */
	zassert_equal(gpio_emul_output_get_dt(&standby_gpio), 1);
	zassert_equal(gpio_emul_output_get_dt(&reset_gpio), 1);
}

ZTEST(mcp25625, test_core_clock_and_capabilities)
{
	uint32_t rate;
	can_mode_t cap;

	zassert_ok(can_get_core_clock(can_dev, &rate));
	zassert_equal(rate, 8000000, "core clock must be osc-freq / 2");

	zassert_ok(can_get_capabilities(can_dev, &cap));
	zassert_equal(cap, CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY |
				   CAN_MODE_ONE_SHOT | CAN_MODE_3_SAMPLES);
}

ZTEST(mcp25625, test_bit_timing_registers)
{
	static const uint32_t bitrates[] = {1000000, 500000, 250000, 125000, 50000, 10000};

	for (size_t i = 0; i < ARRAY_SIZE(bitrates); i++) {
		struct can_timing expected;
		uint8_t cnf1;
		uint8_t cnf2;
		uint8_t cnf3;
		uint32_t total_tq;

		zassert_true(can_calc_timing(can_dev, &expected, bitrates[i], 0) >= 0);
		zassert_ok(can_set_bitrate(can_dev, bitrates[i]), "bitrate %u", bitrates[i]);

		cnf1 = mcp25625_emul_get_reg(emul, REG_CNF1);
		cnf2 = mcp25625_emul_get_reg(emul, REG_CNF2);
		cnf3 = mcp25625_emul_get_reg(emul, REG_CNF3);

		zassert_equal((cnf1 >> 6) + 1, expected.sjw);
		zassert_equal((cnf1 & 0x3F) + 1, expected.prescaler);
		zassert_true((cnf2 & BIT(7)) != 0, "BTLMODE must be set");
		zassert_equal(((cnf2 >> 3) & 0x07) + 1, expected.phase_seg1);
		zassert_equal((cnf2 & 0x07) + 1, expected.prop_seg);
		zassert_equal((cnf3 & 0x07) + 1, expected.phase_seg2);

		/* The register contents must produce the requested bitrate */
		total_tq =
			1 + ((cnf2 & 0x07) + 1) + (((cnf2 >> 3) & 0x07) + 1) + ((cnf3 & 0x07) + 1);
		zassert_equal(8000000 / (((cnf1 & 0x3F) + 1) * total_tq), bitrates[i]);
	}
}

ZTEST(mcp25625, test_bit_timing_constraints)
{
	/* Phase segment 2 must be longer than SJW */
	struct can_timing timing = {
		.sjw = 2,
		.prop_seg = 4,
		.phase_seg1 = 4,
		.phase_seg2 = 2,
		.prescaler = 1,
	};

	zassert_equal(can_set_timing(can_dev, &timing), -ENOTSUP);

	/* Propagation + phase segment 1 must not be shorter than phase segment 2 */
	timing = (struct can_timing){
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 3,
		.prescaler = 1,
	};
	zassert_equal(can_set_timing(can_dev, &timing), -ENOTSUP);
}

ZTEST(mcp25625, test_start_stop_controls_standby)
{
	start_in_mode(CAN_MODE_NORMAL);
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_NORMAL);
	zassert_equal(gpio_emul_output_get_dt(&standby_gpio), 0, "transceiver still in standby");
	zassert_equal(can_start(can_dev), -EALREADY);
	zassert_equal(can_set_bitrate(can_dev, 500000), -EBUSY);
	zassert_equal(can_set_mode(can_dev, CAN_MODE_LOOPBACK), -EBUSY);

	zassert_ok(can_stop(can_dev));
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_CONFIG);
	zassert_equal(gpio_emul_output_get_dt(&standby_gpio), 1, "transceiver not in standby");
	zassert_equal(can_stop(can_dev), -EALREADY);
}

ZTEST(mcp25625, test_unsupported_modes_and_frames)
{
	struct can_frame frame = {.id = 0x123, .dlc = 0};

	zassert_equal(can_set_mode(can_dev, CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY), -ENOTSUP);
	zassert_equal(can_set_mode(can_dev, CAN_MODE_FD), -ENOTSUP);
	zassert_equal(can_set_mode(can_dev, CAN_MODE_MANUAL_RECOVERY), -ENOTSUP);

	zassert_equal(can_send(can_dev, &frame, TIMEOUT, NULL, NULL), -ENETDOWN);

	start_in_mode(CAN_MODE_LOOPBACK);

	frame.flags = CAN_FRAME_FDF;
	zassert_equal(can_send(can_dev, &frame, TIMEOUT, NULL, NULL), -ENOTSUP);

	frame.flags = 0;
	frame.dlc = 9;
	zassert_equal(can_send(can_dev, &frame, TIMEOUT, NULL, NULL), -EINVAL);
}

ZTEST(mcp25625, test_loopback_frames)
{
	const struct can_frame frames[] = {
		{.id = 0x7FF, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 7, 8}},
		{.id = 0x000, .dlc = 0},
		{.id = 0x1FFFFFFF, .flags = CAN_FRAME_IDE, .dlc = 3, .data = {0xAA, 0x55, 0x01}},
		{.id = 0x02040001,
		 .flags = CAN_FRAME_IDE,
		 .dlc = 8,
		 .data = {0xFF, 0, 0x80, 0x7F, 1, 2, 3, 4}},
		{.id = 0x123, .flags = CAN_FRAME_RTR, .dlc = 4},
		{.id = 0x15555555, .flags = CAN_FRAME_IDE | CAN_FRAME_RTR, .dlc = 2},
	};
	struct can_frame rx;
	int std_id;
	int ext_id;

	std_id = add_msgq_filter(&std_filter_all);
	ext_id = add_msgq_filter(&ext_filter_all);

	start_in_mode(CAN_MODE_LOOPBACK);
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_LOOPBACK);

	for (size_t i = 0; i < ARRAY_SIZE(frames); i++) {
		zassert_ok(can_send(can_dev, &frames[i], TIMEOUT, NULL, NULL), "frame %zu", i);
		zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT), "frame %zu not received", i);
		assert_frames_equal(&rx, &frames[i]);
	}

	zassert_equal(k_msgq_num_used_get(&rx_msgq), 0, "unexpected extra frames");

	remove_filter(std_id);
	remove_filter(ext_id);
}

ZTEST(mcp25625, test_normal_mode_transmit)
{
	const struct can_frame frame = {
		.id = 0x0204ABCD,
		.flags = CAN_FRAME_IDE,
		.dlc = 5,
		.data = {9, 8, 7, 6, 5},
	};
	struct mcp25625_stats stats;
	struct can_frame bus;

	start_in_mode(CAN_MODE_NORMAL);

	send_with_callback(&frame);
	zassert_ok(wait_tx_result());

	zassert_ok(mcp25625_emul_get_tx_frame(emul, &bus));
	assert_frames_equal(&bus, &frame);

	/* Blocking send */
	zassert_ok(can_send(can_dev, &frame, TIMEOUT, NULL, NULL));
	zassert_ok(mcp25625_emul_get_tx_frame(emul, &bus));
	assert_frames_equal(&bus, &frame);

	zassert_ok(mcp25625_get_stats(can_dev, &stats));
	zassert_equal(stats.tx_frames, 2);
}

ZTEST(mcp25625, test_receive_filtering)
{
	const struct can_filter exact_std = {.id = 0x123, .mask = CAN_STD_ID_MASK};
	/* FRC style: device type 2, manufacturer 4, any API, any device number */
	const struct can_filter masked_ext = {
		.id = 0x02040000,
		.mask = 0x1FFF0000,
		.flags = CAN_FILTER_IDE,
	};
	const struct can_frame std_match = {.id = 0x123, .dlc = 1, .data = {0x42}};
	const struct can_frame std_other = {.id = 0x124, .dlc = 1, .data = {0x42}};
	const struct can_frame ext_match = {
		.id = 0x0204B10A,
		.flags = CAN_FRAME_IDE,
		.dlc = 2,
		.data = {1, 2},
	};
	const struct can_frame ext_other = {
		.id = 0x0205B10A,
		.flags = CAN_FRAME_IDE,
		.dlc = 2,
		.data = {1, 2},
	};
	/* Same numeric ID as the standard filter, but extended */
	const struct can_frame ext_same_id = {.id = 0x123, .flags = CAN_FRAME_IDE, .dlc = 0};
	struct can_frame rx;
	int id1;
	int id2;

	id1 = add_msgq_filter(&exact_std);
	id2 = add_msgq_filter(&masked_ext);
	start_in_mode(CAN_MODE_NORMAL);

	zassert_ok(mcp25625_emul_rx_frame(emul, &std_other));
	zassert_ok(mcp25625_emul_rx_frame(emul, &ext_other));
	zassert_ok(mcp25625_emul_rx_frame(emul, &ext_same_id));
	zassert_ok(mcp25625_emul_rx_frame(emul, &std_match));
	zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
	assert_frames_equal(&rx, &std_match);

	zassert_ok(mcp25625_emul_rx_frame(emul, &ext_match));
	zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
	assert_frames_equal(&rx, &ext_match);

	remove_filter(id2);
	zassert_ok(mcp25625_emul_rx_frame(emul, &ext_match));
	zassert_equal(k_msgq_get(&rx_msgq, &rx, K_MSEC(20)), -EAGAIN,
		      "frame delivered to removed filter");

	remove_filter(id1);
}

ZTEST(mcp25625, test_receive_order_and_overflow)
{
	struct can_frame frames[3];
	struct mcp25625_stats stats;
	struct can_frame rx;
	int id;

	for (size_t i = 0; i < ARRAY_SIZE(frames); i++) {
		frames[i] = (struct can_frame){
			.id = 0x100 + i,
			.dlc = 1,
			.data = {(uint8_t)i},
		};
	}

	id = add_msgq_filter(&std_filter_all);
	start_in_mode(CAN_MODE_NORMAL);

	/* Two frames arrive before the driver runs: the second rolls over into RXB1 */
	k_sched_lock();
	zassert_ok(mcp25625_emul_rx_frame(emul, &frames[0]));
	zassert_ok(mcp25625_emul_rx_frame(emul, &frames[1]));
	k_sched_unlock();

	for (size_t i = 0; i < 2; i++) {
		zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
		assert_frames_equal(&rx, &frames[i]);
	}

	/* A third frame overflows both buffers */
	k_sched_lock();
	zassert_ok(mcp25625_emul_rx_frame(emul, &frames[0]));
	zassert_ok(mcp25625_emul_rx_frame(emul, &frames[1]));
	zassert_equal(mcp25625_emul_rx_frame(emul, &frames[2]), -ENOBUFS);
	k_sched_unlock();

	for (size_t i = 0; i < 2; i++) {
		zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
		assert_frames_equal(&rx, &frames[i]);
	}

	/* Give the driver a moment to process the error interrupt */
	k_msleep(10);
	zassert_ok(mcp25625_get_stats(can_dev, &stats));
	zassert_equal(stats.rx_overruns, 1);
	zassert_equal(stats.rx_frames, 4);
	zassert_equal(mcp25625_emul_get_reg(emul, 0x2D) & 0xC0, 0, "overrun flags not cleared");

	remove_filter(id);
}

ZTEST(mcp25625, test_one_shot_without_ack)
{
	const struct can_frame frame = {.id = 0x321, .dlc = 1, .data = {1}};
	struct mcp25625_stats stats;

	start_in_mode(CAN_MODE_ONE_SHOT);
	zassert_true((mcp25625_emul_get_reg(emul, REG_CANCTRL) & BIT(3)) != 0, "OSM not set");

	mcp25625_emul_set_tx_ack(emul, false);
	send_with_callback(&frame);
	zassert_equal(wait_tx_result(), -EIO);

	zassert_ok(mcp25625_get_stats(can_dev, &stats));
	zassert_equal(stats.tx_failures, 1);

	/* The next frame goes through once the bus acknowledges again */
	mcp25625_emul_set_tx_ack(emul, true);
	send_with_callback(&frame);
	zassert_ok(wait_tx_result());
}

ZTEST(mcp25625, test_one_shot_lost_arbitration)
{
	const struct can_frame frame = {.id = 0x321, .dlc = 0};

	start_in_mode(CAN_MODE_ONE_SHOT);

	/* Losing arbitration raises no interrupt; the driver must poll */
	mcp25625_emul_set_tx_lose_arbitration(emul, true);
	send_with_callback(&frame);
	zassert_equal(wait_tx_result(), -EBUSY);
}

ZTEST(mcp25625, test_normal_mode_retries_without_ack)
{
	const struct can_frame frame = {.id = 0x321, .dlc = 0};
	struct can_frame bus;

	start_in_mode(CAN_MODE_NORMAL);
	zassert_true((mcp25625_emul_get_reg(emul, REG_CANCTRL) & BIT(3)) == 0, "OSM set");

	mcp25625_emul_set_tx_ack(emul, false);
	send_with_callback(&frame);
	zassert_equal(k_sem_take(&tx_result.done, K_MSEC(50)), -EAGAIN,
		      "transmission completed although it was not acknowledged");

	/* Stopping the controller aborts the pending transmission */
	zassert_ok(can_stop(can_dev));
	zassert_equal(wait_tx_result(), -ENETDOWN);
	zassert_equal(mcp25625_emul_get_tx_frame(emul, &bus), -ENOMSG);
}

ZTEST(mcp25625, test_error_state_changes)
{
	struct state_event ev;
	enum can_state state;
	struct can_bus_err_cnt cnt;

	can_set_state_change_callback(can_dev, state_callback, NULL);
	start_in_mode(CAN_MODE_NORMAL);

	zassert_ok(can_get_state(can_dev, &state, &cnt));
	zassert_equal(state, CAN_STATE_ERROR_ACTIVE);

	mcp25625_emul_set_error_counters(emul, 100, 5);
	zassert_ok(k_msgq_get(&state_msgq, &ev, TIMEOUT));
	zassert_equal(ev.state, CAN_STATE_ERROR_WARNING);
	zassert_equal(ev.err_cnt.tx_err_cnt, 100);
	zassert_equal(ev.err_cnt.rx_err_cnt, 5);

	mcp25625_emul_set_error_counters(emul, 100, 130);
	zassert_ok(k_msgq_get(&state_msgq, &ev, TIMEOUT));
	zassert_equal(ev.state, CAN_STATE_ERROR_PASSIVE);
	zassert_equal(ev.err_cnt.rx_err_cnt, 130);

	zassert_ok(can_get_state(can_dev, &state, &cnt));
	zassert_equal(state, CAN_STATE_ERROR_PASSIVE);
	zassert_equal(cnt.tx_err_cnt, 100);
	zassert_equal(cnt.rx_err_cnt, 130);

	/* Recovery raises no interrupt and is detected by polling */
	mcp25625_emul_set_error_counters(emul, 0, 0);
	zassert_ok(k_msgq_get(&state_msgq, &ev, TIMEOUT));
	zassert_equal(ev.state, CAN_STATE_ERROR_ACTIVE);

	zassert_ok(can_stop(can_dev));
	zassert_ok(can_get_state(can_dev, &state, NULL));
	zassert_equal(state, CAN_STATE_STOPPED);
}

ZTEST(mcp25625, test_bus_off_aborts_transmission)
{
	const struct can_frame frame = {.id = 0x10, .dlc = 0};
	struct mcp25625_stats stats;
	struct state_event ev;

	can_set_state_change_callback(can_dev, state_callback, NULL);
	start_in_mode(CAN_MODE_NORMAL);

	/* Keep the frame pending, then go bus-off */
	mcp25625_emul_set_tx_hold(emul, true);
	send_with_callback(&frame);
	mcp25625_emul_set_error_counters(emul, 256, 0);

	zassert_equal(wait_tx_result(), -ENETUNREACH);
	zassert_ok(k_msgq_get(&state_msgq, &ev, TIMEOUT));
	zassert_equal(ev.state, CAN_STATE_BUS_OFF);
	zassert_equal(can_send(can_dev, &frame, TIMEOUT, NULL, NULL), -ENETUNREACH);

	/* Automatic recovery */
	mcp25625_emul_set_tx_hold(emul, false);
	mcp25625_emul_set_error_counters(emul, 0, 0);
	zassert_ok(k_msgq_get(&state_msgq, &ev, TIMEOUT));
	zassert_equal(ev.state, CAN_STATE_ERROR_ACTIVE);
	zassert_ok(can_send(can_dev, &frame, TIMEOUT, NULL, NULL));

	zassert_ok(mcp25625_get_stats(can_dev, &stats));
	zassert_equal(stats.bus_off, 1);
}

ZTEST(mcp25625, test_hw_acceptance)
{
	const struct mcp25625_hw_acceptance acc = {
		.rxb0 =
			{
				.ide = true,
				/* CTRE (4) motor controllers (2), any API, any device */
				.mask = 0x1FFF0000,
				.id = {0x02040000, 0x02040000},
			},
		.rxb1 =
			{
				.ide = false,
				.mask = 0x7F0,
				.id = {0x120, 0x120, 0x120, 0x120},
			},
	};
	struct mcp25625_hw_acceptance bad = acc;
	const struct can_frame accepted[] = {
		{.id = 0x0204A3C5, .flags = CAN_FRAME_IDE, .dlc = 1, .data = {1}},
		{.id = 0x12A, .dlc = 2, .data = {0xDE, 0xAD}},
	};
	const struct can_frame rejected[] = {
		{.id = 0x0205A3C5, .flags = CAN_FRAME_IDE, .dlc = 1, .data = {1}},
		{.id = 0x13A, .dlc = 0},
		{.id = 0x120, .flags = CAN_FRAME_IDE, .dlc = 0},
	};
	struct can_frame rx;
	int std_id;
	int ext_id;

	bad.rxb1.id[2] = 0x800;
	zassert_equal(mcp25625_set_hw_acceptance(can_dev, &bad), -EINVAL);

	zassert_ok(mcp25625_set_hw_acceptance(can_dev, &acc));
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB0CTRL) & 0x60, 0, "filters not enabled");
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB1CTRL) & 0x60, 0, "filters not enabled");
	/* RXM0 = 0x1FFF0000: SID 0x7FF, EID 0x30000 */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM0SIDH), 0xFF);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM0SIDH + 1), 0xE3);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM0SIDH + 2), 0x00);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM0SIDH + 3), 0x00);
	/* RXF0 = 0x02040000 extended: SID 0x081, EXIDE, EID 0x00000 */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXF0SIDH), 0x10);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXF0SIDH + 1), 0x28);
	/* RXM1 = 0x7F0 standard, EID part zero so data bytes are not filtered */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM1SIDH), 0xFE);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM1SIDH + 1), 0x00);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM1SIDH + 2), 0x00);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXM1SIDH + 3), 0x00);
	/* RXF2 = 0x120 standard */
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXF2SIDH), 0x24);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXF2SIDH + 1), 0x00);

	std_id = add_msgq_filter(&std_filter_all);
	ext_id = add_msgq_filter(&ext_filter_all);
	start_in_mode(CAN_MODE_NORMAL);

	zassert_equal(mcp25625_set_hw_acceptance(can_dev, NULL), -EBUSY);

	for (size_t i = 0; i < ARRAY_SIZE(rejected); i++) {
		zassert_equal(mcp25625_emul_rx_frame(emul, &rejected[i]), -EACCES,
			      "frame %zu not rejected", i);
	}

	for (size_t i = 0; i < ARRAY_SIZE(accepted); i++) {
		zassert_ok(mcp25625_emul_rx_frame(emul, &accepted[i]), "frame %zu", i);
		zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
		assert_frames_equal(&rx, &accepted[i]);
	}

	zassert_ok(can_stop(can_dev));
	zassert_ok(mcp25625_set_hw_acceptance(can_dev, NULL));
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB0CTRL), 0x64);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_RXB1CTRL), 0x60);

	remove_filter(std_id);
	remove_filter(ext_id);
}

ZTEST(mcp25625, test_listen_only)
{
	const struct can_frame frame = {.id = 0x55, .dlc = 1, .data = {0x99}};
	struct can_frame rx;
	int id;

	id = add_msgq_filter(&std_filter_all);
	start_in_mode(CAN_MODE_LISTENONLY);
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_LISTEN_ONLY);

	zassert_ok(mcp25625_emul_rx_frame(emul, &frame));
	zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
	assert_frames_equal(&rx, &frame);

	remove_filter(id);
}

ZTEST(mcp25625, test_triple_sampling)
{
	start_in_mode(CAN_MODE_3_SAMPLES);
	zassert_true((mcp25625_emul_get_reg(emul, REG_CNF2) & CNF2_SAM) != 0, "SAM not set");
	zassert_ok(can_stop(can_dev));

	start_in_mode(CAN_MODE_NORMAL);
	zassert_true((mcp25625_emul_get_reg(emul, REG_CNF2) & CNF2_SAM) == 0, "SAM still set");
}

ZTEST(mcp25625, test_power_management)
{
	enum pm_device_state pm_state;
	struct can_frame rx;
	const struct can_frame frame = {.id = 0x77, .dlc = 0};
	int id;

	zassert_ok(pm_device_action_run(can_dev, PM_DEVICE_ACTION_SUSPEND));
	zassert_ok(pm_device_state_get(can_dev, &pm_state));
	zassert_equal(pm_state, PM_DEVICE_STATE_SUSPENDED);
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_SLEEP);

	zassert_ok(pm_device_action_run(can_dev, PM_DEVICE_ACTION_RESUME));
	zassert_equal(mcp25625_emul_get_opmode(emul), OPMODE_CONFIG);
	zassert_equal(mcp25625_emul_get_reg(emul, REG_CANINTE), CANINTE_DEFAULT);

	/* Busy while filters are installed or the controller is started */
	id = add_msgq_filter(&std_filter_all);
	zassert_equal(pm_device_action_run(can_dev, PM_DEVICE_ACTION_SUSPEND), -EBUSY);

	start_in_mode(CAN_MODE_LOOPBACK);
	zassert_ok(can_send(can_dev, &frame, TIMEOUT, NULL, NULL));
	zassert_ok(k_msgq_get(&rx_msgq, &rx, TIMEOUT));
	assert_frames_equal(&rx, &frame);

	remove_filter(id);
	zassert_equal(pm_device_action_run(can_dev, PM_DEVICE_ACTION_SUSPEND), -EBUSY);
}

ZTEST_SUITE(mcp25625, NULL, mcp25625_setup, mcp25625_before, mcp25625_after, NULL);
