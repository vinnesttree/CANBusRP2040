/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * End-to-end test: the Phoenix 6 client against the real host bridge, which
 * runs the requests on simulated Talon FX devices of the Phoenix 6 library.
 * Run with scripts/phoenix6_e2e.py.
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/phoenix6.h>

#define TIMEOUT_MS 200

/* Each test uses its own device so the tests do not interfere */
#define DEV_CONTROL  1
#define DEV_CONFIG   2
#define DEV_POSITION 3
#define DEV_COMPOUND 4

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(phoenix6_uart));

static bool wait_signal(uint8_t device, enum phoenix6_signal signal, double expected,
			double tolerance, struct phoenix6_signal_value *v)
{
	int64_t end = k_uptime_get() + 3000;

	while (k_uptime_get() < end) {
		if (phoenix6_get_signal(device, signal, v) == 0 && v->status == 0 &&
		    fabs(v->value - expected) <= tolerance) {
			return true;
		}
		k_msleep(20);
	}

	return false;
}

static int32_t wait_control_status(uint8_t device)
{
	int64_t end = k_uptime_get() + 1000;

	while (phoenix6_last_control_status(device) == PHOENIX6_STATUS_NONE &&
	       k_uptime_get() < end) {
		k_msleep(5);
	}

	return phoenix6_last_control_status(device);
}

static void *e2e_setup(void)
{
	int err;

	zassert_ok(phoenix6_bridge_start(uart));

	/* The bridge checks its schema and starts Phoenix 6 first */
	err = phoenix6_bridge_wait_connected(K_SECONDS(30));
	zassert_ok(err, "no connection to the host bridge (%d)", err);
	printk("connected to Phoenix 6 %s\n", phoenix6_bridge_host_version());

	return NULL;
}

static void e2e_before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_true(phoenix6_bridge_is_connected());
	zassert_ok(phoenix6_feed_enable(1000));
}

ZTEST(phoenix6_e2e, test_velocity_closed_loop)
{
	struct phoenix6_control req = PHOENIX6_VELOCITY_VOLTAGE(10.0);
	struct phoenix6_signal_value v;
	int32_t status = -1;

	zassert_ok(phoenix6_set_control(DEV_CONTROL, &req));
	zassert_equal(wait_control_status(DEV_CONTROL), PHOENIX6_STATUS_OK);

	zassert_ok(phoenix6_subscribe(DEV_CONTROL, PHOENIX6_SIGNAL_CLOSED_LOOP_REFERENCE, 50.0f,
				      &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_true(
		wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CLOSED_LOOP_REFERENCE, 10.0, 1e-6, &v),
		"closed loop reference %f", v.value);

	zassert_ok(phoenix6_subscribe(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE, 50.0f, NULL));
	zassert_true(wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE,
				 PHOENIX6_CONTROL_MODE_VELOCITY_VOLTAGE_FOC, 0, &v) ||
			     wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE,
					 PHOENIX6_CONTROL_MODE_VELOCITY_VOLTAGE, 0, &v),
		     "control mode %d", (int)v.value);

	/* Change the request: a new target and slot */
	req.velocity_voltage.velocity = -3.5;
	req.velocity_voltage.slot = 2;
	zassert_ok(phoenix6_set_control(DEV_CONTROL, &req));
	zassert_true(
		wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CLOSED_LOOP_REFERENCE, -3.5, 1e-6, &v));
	zassert_ok(phoenix6_subscribe(DEV_CONTROL, PHOENIX6_SIGNAL_CLOSED_LOOP_SLOT, 50.0f, NULL));
	zassert_true(wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CLOSED_LOOP_SLOT, 2, 0, &v));

	req = PHOENIX6_NEUTRAL_OUT();
	zassert_ok(phoenix6_set_control(DEV_CONTROL, &req));
	zassert_true(wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE,
				 PHOENIX6_CONTROL_MODE_NEUTRAL_OUT, 0, &v));
}

ZTEST(phoenix6_e2e, test_configuration)
{
	int32_t status = -1;
	double value = -1.0;

	zassert_ok(phoenix6_config_set(DEV_CONFIG, PHOENIX6_CONFIG_SLOT0_K_P, 2.5, TIMEOUT_MS,
				       &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_ok(phoenix6_config_set(DEV_CONFIG, PHOENIX6_CONFIG_SLOT0_K_V, 0.125, TIMEOUT_MS,
				       &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);

	/* Setting k_v kept k_p (read-modify-write of the group) */
	zassert_ok(phoenix6_config_get(DEV_CONFIG, PHOENIX6_CONFIG_SLOT0_K_P, &value, TIMEOUT_MS,
				       &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_equal(value, 2.5);

	zassert_ok(phoenix6_config_set(DEV_CONFIG, PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE,
				       PHOENIX6_NEUTRAL_MODE_BRAKE, TIMEOUT_MS, &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_ok(phoenix6_config_get(DEV_CONFIG, PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE,
				       &value, TIMEOUT_MS, &status));
	zassert_equal(value, PHOENIX6_NEUTRAL_MODE_BRAKE);

	zassert_ok(phoenix6_factory_default(DEV_CONFIG, TIMEOUT_MS, &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_ok(phoenix6_config_get(DEV_CONFIG, PHOENIX6_CONFIG_SLOT0_K_P, &value, TIMEOUT_MS,
				       &status));
	zassert_equal(value, 0.0);
}

ZTEST(phoenix6_e2e, test_position_and_faults)
{
	struct phoenix6_signal_value v;
	int32_t status = -1;

	zassert_ok(phoenix6_set_position(DEV_POSITION, 4.0, TIMEOUT_MS, &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);
	zassert_ok(phoenix6_subscribe(DEV_POSITION, PHOENIX6_SIGNAL_POSITION, 100.0f, NULL));
	zassert_true(wait_signal(DEV_POSITION, PHOENIX6_SIGNAL_POSITION, 4.0, 1e-3, &v),
		     "position %f", v.value);
	zassert_true(v.age_ms >= 0.0f && v.age_ms < 1000.0f);

	zassert_ok(phoenix6_clear_sticky_faults(DEV_POSITION, TIMEOUT_MS, &status));
	zassert_equal(status, PHOENIX6_STATUS_OK);

	zassert_ok(phoenix6_subscribe(DEV_POSITION, PHOENIX6_SIGNAL_POSITION, 0.0f, NULL));
	zassert_equal(phoenix6_get_signal(DEV_POSITION, PHOENIX6_SIGNAL_POSITION, &v), -ENODATA);
}

ZTEST(phoenix6_e2e, test_enum_and_compound_requests)
{
	struct phoenix6_control follower =
		PHOENIX6_FOLLOWER(DEV_CONTROL, PHOENIX6_MOTOR_ALIGNMENT_OPPOSED);
	struct phoenix6_control diff = PHOENIX6_DIFF_VELOCITY_VOLTAGE_POSITION(
		PHOENIX6_VELOCITY_VOLTAGE_PARAMS(5.0), PHOENIX6_POSITION_VOLTAGE_PARAMS(0.25));
	int32_t status;

	zassert_ok(phoenix6_set_control(DEV_COMPOUND, &follower));
	zassert_equal(wait_control_status(DEV_COMPOUND), PHOENIX6_STATUS_OK);

	/*
	 * Differential requests need a CANivore; the simulation may refuse them,
	 * but the bridge must build and send them.
	 */
	zassert_ok(phoenix6_set_control(DEV_COMPOUND, &diff));
	k_msleep(100);
	status = phoenix6_last_control_status(DEV_COMPOUND);
	zassert_true(status != PHOENIX6_STATUS_BRIDGE_BAD_REQUEST &&
			     status != PHOENIX6_STATUS_BRIDGE_EXCEPTION,
		     "status %d", status);
}

ZTEST(phoenix6_e2e, test_session_stays_up_while_enabled)
{
	struct phoenix6_control req = PHOENIX6_VOLTAGE_OUT(2.0);
	struct phoenix6_signal_value v;
	int64_t end;

	zassert_ok(phoenix6_set_control(DEV_CONTROL, &req));
	zassert_ok(phoenix6_subscribe(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE, 20.0f, NULL));
	zassert_true(wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE,
				 PHOENIX6_CONTROL_MODE_VOLTAGE_OUT, 0, &v) ||
		     wait_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE,
				 PHOENIX6_CONTROL_MODE_VOLTAGE_FOC, 0, &v));

	/* Only the enable is fed for 2 s: the session (and the request) persist */
	end = k_uptime_get() + 2000;
	while (k_uptime_get() < end) {
		zassert_ok(phoenix6_feed_enable(100));
		zassert_true(phoenix6_bridge_is_connected());
		k_msleep(50);
	}

	zassert_ok(phoenix6_get_signal(DEV_CONTROL, PHOENIX6_SIGNAL_CONTROL_MODE, &v));
	zassert_true(k_uptime_get() - v.received_ms < 200);
	zassert_true(v.value == PHOENIX6_CONTROL_MODE_VOLTAGE_OUT ||
			     v.value == PHOENIX6_CONTROL_MODE_VOLTAGE_FOC,
		     "control mode %d", (int)v.value);
}

ZTEST_SUITE(phoenix6_e2e, NULL, e2e_setup, e2e_before, NULL, NULL);
