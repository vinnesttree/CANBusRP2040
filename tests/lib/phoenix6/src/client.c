/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Bridge client against a fake host bridge on the UART emulator.
 */

#include <string.h>

#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/phoenix6.h>

#include "phoenix6_proto.h"

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(euart0));

/* Fake host behaviour */
static struct {
	atomic_t silent; /* do not answer anything */
	atomic_t reject; /* answer HELLO with a wrong schema hash */
	atomic_t answer; /* answer requests with RESULT */
	atomic_t hellos;
	atomic_t pings;
	atomic_t enables;
	int32_t status;
	double value;
	uint16_t seq;
} host;

struct host_msg {
	uint8_t type;
	uint16_t seq;
	uint8_t body_len;
	uint8_t body[200];
};

K_MSGQ_DEFINE(host_msgq, sizeof(struct host_msg), 16, 4);
static K_MUTEX_DEFINE(host_tx_lock);

static void host_send(uint8_t type, const uint8_t *body, size_t len)
{
	uint8_t out[PHOENIX6_MAX_ENCODED];
	int n;

	k_mutex_lock(&host_tx_lock, K_FOREVER);
	n = phoenix6_packet_encode(type, host.seq++, body, len, out, sizeof(out));
	zassert_true(n > 0);
	zassert_equal(uart_emul_put_rx_data(uart_dev, out, n), n);
	k_mutex_unlock(&host_tx_lock);
}

static void host_send_result(uint16_t seq, uint8_t type, uint8_t device, int32_t status,
			     double value)
{
	uint8_t body[16];
	struct phoenix6_writer w;

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u16(&w, seq);
	phoenix6_put_u8(&w, type);
	phoenix6_put_u8(&w, device);
	phoenix6_put_i32(&w, status);
	phoenix6_put_f64(&w, value);
	host_send(PHOENIX6_MSG_RESULT, body, w.len);
}

static void host_handle(const struct phoenix6_packet *pkt)
{
	struct host_msg msg = {
		.type = pkt->type,
		.seq = pkt->seq,
		.body_len = MIN(pkt->body_len, sizeof(msg.body)),
	};
	uint8_t body[48];
	struct phoenix6_writer w;

	memcpy(msg.body, pkt->body, msg.body_len);

	if (atomic_get(&host.silent)) {
		return;
	}

	switch (pkt->type) {
	case PHOENIX6_MSG_HELLO:
		atomic_inc(&host.hellos);
		phoenix6_writer_init(&w, body, sizeof(body));
		phoenix6_put_u8(&w, PHOENIX6_PROTO_VERSION);
		phoenix6_put_u32(&w, atomic_get(&host.reject) ? 0x12345678U : PHOENIX6_SCHEMA_HASH);
		phoenix6_put_u8(&w, atomic_get(&host.reject) ? 0U : 1U);
		phoenix6_put_u8(&w, 6);
		memcpy(&body[w.len], "26.3.0", 6);
		host_send(PHOENIX6_MSG_HELLO_REPLY, body, w.len + 6);
		return;
	case PHOENIX6_MSG_PING:
		atomic_inc(&host.pings);
		host_send(PHOENIX6_MSG_PONG, NULL, 0);
		return;
	case PHOENIX6_MSG_ENABLE:
		atomic_inc(&host.enables);
		break;
	default:
		if (atomic_get(&host.answer) && pkt->type != PHOENIX6_MSG_SET_CONTROL) {
			host_send_result(pkt->seq, pkt->type, pkt->body[0], host.status,
					 host.value);
		}
		break;
	}

	(void)k_msgq_put(&host_msgq, &msg, K_NO_WAIT);
}

static void host_thread(void *p1, void *p2, void *p3)
{
	static struct phoenix6_frame_decoder decoder;
	struct phoenix6_packet pkt;
	uint8_t buf[256];

	phoenix6_frame_decoder_init(&decoder);

	while (true) {
		uint32_t n = uart_emul_get_tx_data(uart_dev, buf, sizeof(buf));

		for (uint32_t i = 0; i < n; i++) {
			if (phoenix6_frame_decoder_feed(&decoder, buf[i], &pkt)) {
				host_handle(&pkt);
			}
		}

		if (n == 0U) {
			k_msleep(1);
		}
	}
}

K_THREAD_DEFINE(fake_host, 2048, host_thread, NULL, NULL, NULL, 4, 0, 0);

/* Wait for the next message of a type other than PING/HELLO */
static void host_expect(uint8_t type, struct host_msg *msg)
{
	zassert_ok(k_msgq_get(&host_msgq, msg, K_MSEC(500)), "no message 0x%02x", type);
	zassert_equal(msg->type, type, "got 0x%02x, expected 0x%02x", msg->type, type);
}

static void host_reset(void)
{
	atomic_set(&host.silent, 0);
	atomic_set(&host.reject, 0);
	atomic_set(&host.answer, 1);
	host.status = 0;
	host.value = 0.0;
	k_msgq_purge(&host_msgq);
}

static void *client_setup(void)
{
	zassert_true(device_is_ready(uart_dev));
	host_reset();

	zassert_false(phoenix6_bridge_is_connected());
	zassert_equal(phoenix6_feed_enable(100), -ENOTCONN);
	zassert_ok(phoenix6_bridge_start(uart_dev));
	zassert_equal(phoenix6_bridge_start(uart_dev), -EALREADY);

	return NULL;
}

static void client_before(void *fixture)
{
	ARG_UNUSED(fixture);

	host_reset();
	zassert_ok(phoenix6_bridge_wait_connected(K_SECONDS(3)));
	k_msgq_purge(&host_msgq);
}

ZTEST(phoenix6_client, test_handshake)
{
	zassert_true(phoenix6_bridge_is_connected());
	zassert_str_equal(phoenix6_bridge_host_version(), "26.3.0");
	zassert_true(atomic_get(&host.hellos) >= 1);
}

ZTEST(phoenix6_client, test_keepalive)
{
	atomic_val_t start = atomic_get(&host.pings);

	/* Idle client: one PING per 100 ms */
	k_msleep(1000);
	zassert_within(atomic_get(&host.pings) - start, 10, 2);
	zassert_true(phoenix6_bridge_is_connected());
}

ZTEST(phoenix6_client, test_link_stays_up_while_feeding_enable)
{
	atomic_val_t pings = atomic_get(&host.pings);
	int64_t end = k_uptime_get() + 1500;

	/* ENABLE gets no answer; the PING/PONG keepalive must continue */
	while (k_uptime_get() < end) {
		zassert_ok(phoenix6_feed_enable(100));
		zassert_true(phoenix6_bridge_is_connected());
		k_msleep(20);
	}

	zassert_true(atomic_get(&host.pings) - pings >= 13);
}

ZTEST(phoenix6_client, test_link_loss_and_recovery)
{
	struct phoenix6_control req = PHOENIX6_NEUTRAL_OUT();
	int64_t start = k_uptime_get();

	atomic_set(&host.silent, 1);
	while (phoenix6_bridge_is_connected() && k_uptime_get() - start < 2000) {
		k_msleep(10);
	}

	/* The last answer arrived up to one keepalive interval before the host went silent */
	zassert_false(phoenix6_bridge_is_connected());
	zassert_true(k_uptime_get() - start >=
		     CONFIG_PHOENIX6_LINK_TIMEOUT_MS - CONFIG_PHOENIX6_KEEPALIVE_MS - 10);
	zassert_true(k_uptime_get() - start <= CONFIG_PHOENIX6_LINK_TIMEOUT_MS + 30);
	zassert_equal(phoenix6_set_control(1, &req), -ENOTCONN);
	zassert_equal(phoenix6_config_get(1, PHOENIX6_CONFIG_SLOT0_K_P, &(double){0}, 10, NULL),
		      -ENOTCONN);

	atomic_set(&host.silent, 0);
	zassert_ok(phoenix6_bridge_wait_connected(K_MSEC(500)));
}

ZTEST(phoenix6_client, test_schema_mismatch_rejected)
{
	int64_t start = k_uptime_get();

	/* Drop the link, then answer HELLO with another schema */
	atomic_set(&host.reject, 1);
	atomic_set(&host.silent, 1);
	while (phoenix6_bridge_is_connected() && k_uptime_get() - start < 2000) {
		k_msleep(10);
	}
	atomic_set(&host.silent, 0);

	zassert_equal(phoenix6_bridge_wait_connected(K_SECONDS(1)), -EPROTO);
	zassert_false(phoenix6_bridge_is_connected());

	atomic_set(&host.reject, 0);
	zassert_ok(phoenix6_bridge_wait_connected(K_SECONDS(2)));
}

ZTEST(phoenix6_client, test_feed_enable)
{
	struct host_msg msg;
	struct phoenix6_reader r;

	zassert_ok(phoenix6_feed_enable(100));
	host_expect(PHOENIX6_MSG_ENABLE, &msg);
	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u16(&r), 100);
	zassert_equal(r.pos, r.len);

	zassert_ok(phoenix6_feed_enable(0));
	host_expect(PHOENIX6_MSG_ENABLE, &msg);
	zassert_equal(sys_get_le16(msg.body), 0);

	/* Saturated to the field size */
	zassert_ok(phoenix6_feed_enable(100000));
	host_expect(PHOENIX6_MSG_ENABLE, &msg);
	zassert_equal(sys_get_le16(msg.body), UINT16_MAX);
}

ZTEST(phoenix6_client, test_set_control)
{
	struct phoenix6_control req = PHOENIX6_VELOCITY_VOLTAGE(10.0);
	uint8_t expected[128];
	struct phoenix6_writer w;
	struct phoenix6_reader r;
	struct host_msg msg;
	int64_t start;

	req.velocity_voltage.slot = 1;
	req.update_freq_hz = 50.0;

	zassert_equal(phoenix6_last_control_status(3), PHOENIX6_STATUS_NONE);
	zassert_ok(phoenix6_set_control(3, &req));
	host_expect(PHOENIX6_MSG_SET_CONTROL, &msg);

	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u8(&r), 3);
	zassert_equal(phoenix6_get_u16(&r), PHOENIX6_CONTROL_ID_VELOCITY_VOLTAGE);
	zassert_equal(phoenix6_get_f64(&r), 50.0);

	phoenix6_writer_init(&w, expected, sizeof(expected));
	zassert_ok(phoenix6_encode_control_params(&w, phoenix6_control_desc_get(req.id),
						  (const uint8_t *)&req +
							  PHOENIX6_CONTROL_PARAMS_OFFSET));
	zassert_equal(r.len - r.pos, w.len);
	zassert_mem_equal(&r.buf[r.pos], expected, w.len);

	/* Result of the request (CTR StatusCode.InvalidModeToGetSignal = -101 as an example) */
	host_send_result(msg.seq, PHOENIX6_MSG_SET_CONTROL, 3, -101, 0.0);
	start = k_uptime_get();
	while (phoenix6_last_control_status(3) == PHOENIX6_STATUS_NONE &&
	       k_uptime_get() - start < 500) {
		k_msleep(1);
	}
	zassert_equal(phoenix6_last_control_status(3), -101);
	zassert_equal(phoenix6_last_control_status(4), PHOENIX6_STATUS_NONE);

	host_send_result(msg.seq, PHOENIX6_MSG_SET_CONTROL, 3, PHOENIX6_STATUS_OK, 0.0);
	k_msleep(50);
	zassert_equal(phoenix6_last_control_status(3), PHOENIX6_STATUS_OK);
}

ZTEST(phoenix6_client, test_set_control_invalid)
{
	struct phoenix6_control req = PHOENIX6_DUTY_CYCLE_OUT(0.5);

	zassert_equal(phoenix6_set_control(63, &req), -EINVAL);
	zassert_equal(phoenix6_set_control(1, NULL), -EINVAL);
	req.id = PHOENIX6_CONTROL_ID_COUNT;
	zassert_equal(phoenix6_set_control(1, &req), -EINVAL);
	req.id = PHOENIX6_CONTROL_ID_NONE;
	zassert_equal(phoenix6_set_control(1, &req), -EINVAL);
	zassert_equal(phoenix6_last_control_status(63), PHOENIX6_STATUS_NONE);
	zassert_equal(k_msgq_num_used_get(&host_msgq), 0);
}

ZTEST(phoenix6_client, test_config_set_get)
{
	struct host_msg msg;
	struct phoenix6_reader r;
	int32_t status = 1;
	double value = 0.0;

	host.status = 0;
	host.value = 0.25;
	zassert_ok(phoenix6_config_get(5, PHOENIX6_CONFIG_SLOT0_K_P, &value, 50, &status));
	zassert_equal(status, 0);
	zassert_equal(value, 0.25);
	host_expect(PHOENIX6_MSG_CONFIG_GET, &msg);
	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u8(&r), 5);
	zassert_equal(phoenix6_get_u16(&r), PHOENIX6_CONFIG_SLOT0_K_P);
	zassert_equal(phoenix6_get_u16(&r), 50);
	zassert_equal(r.pos, r.len);

	host.status = -1001;
	zassert_ok(phoenix6_config_set(5, PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE,
				       PHOENIX6_NEUTRAL_MODE_BRAKE, 50, &status));
	zassert_equal(status, -1001);
	host_expect(PHOENIX6_MSG_CONFIG_SET, &msg);
	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u8(&r), 5);
	zassert_equal(phoenix6_get_u16(&r), PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE);
	zassert_equal(phoenix6_get_f64(&r), 1.0);
	zassert_equal(phoenix6_get_u16(&r), 50);
	zassert_equal(r.pos, r.len);

	zassert_equal(phoenix6_config_set(63, PHOENIX6_CONFIG_SLOT0_K_P, 0, 50, NULL), -EINVAL);
	zassert_equal(phoenix6_config_set(1, PHOENIX6_CONFIG_NONE, 0, 50, NULL), -EINVAL);
	zassert_equal(phoenix6_config_get(1, PHOENIX6_CONFIG_COUNT, &value, 50, NULL), -EINVAL);
	zassert_equal(phoenix6_config_get(1, PHOENIX6_CONFIG_SLOT0_K_P, NULL, 50, NULL), -EINVAL);
}

ZTEST(phoenix6_client, test_device_requests)
{
	struct host_msg msg;
	struct phoenix6_reader r;
	int32_t status = 1;

	zassert_ok(phoenix6_factory_default(7, 100, &status));
	zassert_equal(status, 0);
	host_expect(PHOENIX6_MSG_FACTORY_DEFAULT, &msg);
	zassert_equal(msg.body_len, 3);
	zassert_equal(msg.body[0], 7);
	zassert_equal(sys_get_le16(&msg.body[1]), 100);

	zassert_ok(phoenix6_clear_sticky_faults(7, 100, NULL));
	host_expect(PHOENIX6_MSG_CLEAR_STICKY_FAULTS, &msg);
	zassert_equal(msg.body_len, 3);

	zassert_ok(phoenix6_set_position(7, -1.5, 100, &status));
	host_expect(PHOENIX6_MSG_SET_POSITION, &msg);
	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u8(&r), 7);
	zassert_equal(phoenix6_get_f64(&r), -1.5);
	zassert_equal(phoenix6_get_u16(&r), 100);
	zassert_equal(r.pos, r.len);
}

ZTEST(phoenix6_client, test_request_timeout_and_late_answer)
{
	struct host_msg msg;
	double value = 0.0;
	int64_t start;

	atomic_set(&host.answer, 0);
	start = k_uptime_get();
	zassert_equal(phoenix6_config_get(2, PHOENIX6_CONFIG_SLOT0_K_P, &value, 10, NULL),
		      -ETIMEDOUT);
	zassert_within(k_uptime_get() - start, 10 + CONFIG_PHOENIX6_RESPONSE_MARGIN_MS, 20);
	host_expect(PHOENIX6_MSG_CONFIG_GET, &msg);

	/* A late answer is ignored and does not count as a control result */
	host_send_result(msg.seq, PHOENIX6_MSG_CONFIG_GET, 2, 0, 1.0);
	k_msleep(50);
	zassert_equal(phoenix6_last_control_status(2), PHOENIX6_STATUS_NONE);

	/* A result for another request does not complete a pending one */
	atomic_set(&host.answer, 1);
	host.value = 2.0;
	zassert_ok(phoenix6_config_get(2, PHOENIX6_CONFIG_SLOT0_K_P, &value, 10, NULL));
	zassert_equal(value, 2.0);
}

static void send_signals(uint8_t device, uint16_t signal, int32_t status, double value,
			 float age_ms)
{
	uint8_t body[1 + PHOENIX6_SIGNAL_ENTRY_SIZE];
	struct phoenix6_writer w;

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, 1);
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, signal);
	phoenix6_put_i32(&w, status);
	phoenix6_put_f64(&w, value);
	phoenix6_put_f32(&w, age_ms);
	zassert_false(w.overflow);
	host_send(PHOENIX6_MSG_SIGNALS, body, w.len);
}

ZTEST(phoenix6_client, test_signals)
{
	struct phoenix6_signal_value v;
	struct host_msg msg;
	struct phoenix6_reader r;
	int32_t status = 1;
	int64_t start;

	zassert_equal(phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, &v), -ENODATA);

	zassert_ok(phoenix6_subscribe(9, PHOENIX6_SIGNAL_VELOCITY, 50.0f, &status));
	zassert_equal(status, 0);
	host_expect(PHOENIX6_MSG_SUBSCRIBE, &msg);
	phoenix6_reader_init(&r, msg.body, msg.body_len);
	zassert_equal(phoenix6_get_u8(&r), 9);
	zassert_equal(phoenix6_get_u16(&r), PHOENIX6_SIGNAL_VELOCITY);
	zassert_equal(phoenix6_get_f32(&r), 50.0f);

	zassert_equal(phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, &v), -ENODATA);

	start = k_uptime_get();
	send_signals(9, PHOENIX6_SIGNAL_VELOCITY, 0, 12.5, 3.0f);
	while (phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, &v) != 0 &&
	       k_uptime_get() - start < 500) {
		k_msleep(1);
	}
	zassert_ok(phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, &v));
	zassert_equal(v.value, 12.5);
	zassert_equal(v.status, 0);
	zassert_equal(v.age_ms, 3.0f);
	zassert_true(v.received_ms >= start);

	/* Other device / signal unaffected */
	zassert_equal(phoenix6_get_signal(8, PHOENIX6_SIGNAL_VELOCITY, &v), -ENODATA);
	zassert_equal(phoenix6_get_signal(9, PHOENIX6_SIGNAL_POSITION, &v), -ENODATA);

	/* Unsubscribe drops the cached value */
	zassert_ok(phoenix6_subscribe(9, PHOENIX6_SIGNAL_VELOCITY, 0.0f, NULL));
	host_expect(PHOENIX6_MSG_SUBSCRIBE, &msg);
	zassert_equal(phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, &v), -ENODATA);

	zassert_equal(phoenix6_subscribe(9, PHOENIX6_SIGNAL_NONE, 1.0f, NULL), -EINVAL);
	zassert_equal(phoenix6_subscribe(9, PHOENIX6_SIGNAL_COUNT, 1.0f, NULL), -EINVAL);
	zassert_equal(phoenix6_subscribe(9, PHOENIX6_SIGNAL_VELOCITY, -1.0f, NULL), -EINVAL);
	zassert_equal(phoenix6_get_signal(9, PHOENIX6_SIGNAL_VELOCITY, NULL), -EINVAL);
}

ZTEST(phoenix6_client, test_corrupt_input_is_ignored)
{
	static const uint8_t garbage[] = {0x13, 0x37, 0x00, 0x05, 0x83, 0x00, 0x00, 0x00};
	int32_t status = 1;

	zassert_equal(uart_emul_put_rx_data(uart_dev, garbage, sizeof(garbage)), sizeof(garbage));
	zassert_ok(phoenix6_clear_sticky_faults(1, 10, &status));
	zassert_equal(status, 0);
	zassert_true(phoenix6_bridge_is_connected());
}

ZTEST_SUITE(phoenix6_client, NULL, client_setup, client_before, NULL, NULL);
