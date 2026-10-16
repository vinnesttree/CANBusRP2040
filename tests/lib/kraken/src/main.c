/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tests of the FIRST CAN helpers and the Kraken support library.
 */

#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm/pwm_fake.h>
#include <zephyr/fff.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include <canbus_rp2040/frc_can.h>
#include <canbus_rp2040/kraken.h>

DEFINE_FFF_GLOBALS;

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct pwm_dt_spec pwm_spec = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

#define assert_close(a, b, tol)                                                                    \
	zassert_true(fabsf((a) - (b)) <= (tol), "%f != %f", (double)(a), (double)(b))

ZTEST(frc_can, test_id_pack_unpack)
{
	struct frc_can_id id;

	/* The universal heartbeat: robot controller, NI, API 0x061, device 0 */
	frc_can_id_unpack(FRC_CAN_HEARTBEAT_ID, &id);
	zassert_equal(id.device_type, FRC_CAN_DEV_ROBOT_CONTROLLER);
	zassert_equal(id.manufacturer, FRC_CAN_MFR_NI);
	zassert_equal(id.api_id, 0x061);
	zassert_equal(id.device_number, 0);
	zassert_equal(frc_can_id_pack(&id), FRC_CAN_HEARTBEAT_ID);

	id = (struct frc_can_id){
		.device_type = FRC_CAN_DEV_MOTOR_CONTROLLER,
		.manufacturer = FRC_CAN_MFR_CTR_ELECTRONICS,
		.api_id = frc_can_api_id(0x3F, 0xF),
		.device_number = 0x3F,
	};
	zassert_equal(frc_can_id_pack(&id), 0x0204FFFF);

	zassert_equal(frc_can_api_id(0x05, 0x2), 0x052);
	zassert_equal(frc_can_api_class(0x052), 0x05);
	zassert_equal(frc_can_api_index(0x052), 0x2);

	zassert_str_equal(frc_can_manufacturer_name(4), "CTR Electronics");
	zassert_str_equal(frc_can_device_type_name(2), "Motor Controller");
	zassert_is_null(frc_can_manufacturer_name(200));
}

ZTEST(frc_can, test_filter_and_disable_frame)
{
	struct can_filter filter;
	struct can_frame frame;

	frc_can_filter_init(&filter, FRC_CAN_DEV_MOTOR_CONTROLLER, FRC_CAN_MFR_CTR_ELECTRONICS);
	zassert_equal(filter.id, 0x02040000);
	zassert_equal(filter.mask, 0x1FFF0000);
	zassert_equal(filter.flags, CAN_FILTER_IDE);

	frame = (struct can_frame){.id = 0x0204B4C5, .flags = CAN_FRAME_IDE};
	zassert_true(can_frame_matches_filter(&frame, &filter));
	frame.id = 0x0205B4C5;
	zassert_false(can_frame_matches_filter(&frame, &filter));

	frc_can_disable_frame(&frame);
	zassert_equal(frame.id, 0);
	zassert_equal(frame.dlc, 0);
	zassert_equal(frame.flags, CAN_FRAME_IDE);
}

ZTEST(frc_can, test_heartbeat_layout)
{
	struct frc_can_heartbeat hb = {0};
	struct frc_can_heartbeat out;
	uint8_t data[8];

	/* Match time is in the last byte */
	hb.match_time_s = 0xAB;
	frc_can_heartbeat_encode(&hb, data);
	zassert_equal(data[7], 0xAB);
	for (int i = 0; i < 7; i++) {
		zassert_equal(data[i], 0);
	}

	/* Enabled is bit 25: second bit of the fifth byte */
	hb = (struct frc_can_heartbeat){.enabled = true};
	frc_can_heartbeat_encode(&hb, data);
	zassert_equal(data[4], 0x02);

	/* System watchdog is bit 28 */
	hb = (struct frc_can_heartbeat){.system_watchdog = true};
	frc_can_heartbeat_encode(&hb, data);
	zassert_equal(data[4], 0x10);

	/* Hour occupies the top 5 bits of the first byte */
	hb = (struct frc_can_heartbeat){.tod_hour = 31};
	frc_can_heartbeat_encode(&hb, data);
	zassert_equal(data[0], 0xF8);

	/* Round trip with all fields at their maximum */
	hb = (struct frc_can_heartbeat){
		.match_time_s = 255,
		.match_number = 1023,
		.replay_number = 31,
		.ftc_motor_override = true,
		.red_alliance = true,
		.enabled = true,
		.autonomous = true,
		.test_mode = true,
		.system_watchdog = true,
		.tournament_type = 7,
		.tod_year = 63,
		.tod_month = 15,
		.tod_day = 31,
		.tod_sec = 63,
		.tod_min = 63,
		.tod_hour = 31,
	};
	frc_can_heartbeat_encode(&hb, data);
	for (int i = 0; i < 8; i++) {
		zassert_equal(data[i], 0xFF, "byte %d", i);
	}
	frc_can_heartbeat_decode(data, &out);
	zassert_mem_equal(&out, &hb, sizeof(hb));

	/* Fields do not overlap */
	hb = (struct frc_can_heartbeat){.match_number = 0x2AA, .tod_month = 0x9, .tod_sec = 0x2A};
	frc_can_heartbeat_encode(&hb, data);
	frc_can_heartbeat_decode(data, &out);
	zassert_mem_equal(&out, &hb, sizeof(hb));
}

ZTEST_SUITE(frc_can, NULL, NULL, NULL, NULL, NULL);

ZTEST(kraken, test_motor_specs)
{
	const struct kraken_motor_spec *x60 = kraken_motor_spec_get(KRAKEN_X60, KRAKEN_TRAPEZOIDAL);
	const struct kraken_motor_spec *x60_foc = kraken_motor_spec_get(KRAKEN_X60, KRAKEN_FOC);
	const struct kraken_motor_spec *x44 = kraken_motor_spec_get(KRAKEN_X44, KRAKEN_TRAPEZOIDAL);
	const struct kraken_motor_spec *x44_foc = kraken_motor_spec_get(KRAKEN_X44, KRAKEN_FOC);

	zassert_not_null(x60);
	zassert_not_null(x60_foc);
	zassert_not_null(x44);
	zassert_not_null(x44_foc);
	zassert_is_null(kraken_motor_spec_get(2, KRAKEN_FOC));

	assert_close(kraken_rad_s_to_rpm(x60->free_speed), 6000.0f, 0.01f);
	assert_close(x60->stall_torque, 7.09f, 1e-6f);
	assert_close(x60->stall_current, 366.0f, 1e-6f);
	assert_close(kraken_rad_s_to_rpm(x60_foc->free_speed), 5800.0f, 0.01f);
	assert_close(x60_foc->stall_torque, 9.37f, 1e-6f);
	assert_close(kraken_rad_s_to_rpm(x44->free_speed), 7758.0f, 0.01f);
	assert_close(kraken_rad_s_to_rpm(x44_foc->free_speed), 7368.0f, 0.01f);
}

ZTEST(kraken, test_motor_model)
{
	const struct kraken_motor_spec *spec =
		kraken_motor_spec_get(KRAKEN_X60, KRAKEN_TRAPEZOIDAL);
	struct kraken_motor_model m;
	float speed;

	kraken_motor_model_init(&m, spec);

	assert_close(m.resistance, 12.0f / 366.0f, 1e-6f);
	assert_close(m.kt, 7.09f / 366.0f, 1e-6f);
	assert_close(m.kv, spec->free_speed / (12.0f - 2.0f * 12.0f / 366.0f), 1e-3f);

	/* Stall: full voltage, no speed */
	assert_close(kraken_motor_current(&m, 0.0f, 12.0f), 366.0f, 0.01f);
	assert_close(kraken_motor_torque(&m, 366.0f), 7.09f, 1e-4f);

	/* Free speed draws the free current */
	assert_close(kraken_motor_current(&m, spec->free_speed, 12.0f), 2.0f, 0.01f);

	/* voltage() and speed() are inverses of each other */
	speed = kraken_motor_speed(&m, 1.5f, 10.0f);
	assert_close(kraken_motor_voltage(&m, 1.5f, speed), 10.0f, 1e-3f);
	assert_close(kraken_motor_torque(&m, kraken_motor_current(&m, speed, 10.0f)), 1.5f, 1e-3f);

	assert_close(kraken_rotations_to_rad(1.0f), 6.2831853f, 1e-5f);
	assert_close(kraken_rad_to_rotations(3.14159265f), 0.5f, 1e-6f);
}

ZTEST(kraken, test_pwm_pulse_mapping)
{
	zassert_equal(kraken_pwm_pulse_ns(0.0f), 1500000);
	zassert_equal(kraken_pwm_pulse_ns(1.0f), 2004000);
	zassert_equal(kraken_pwm_pulse_ns(-1.0f), 997000);
	zassert_equal(kraken_pwm_pulse_ns(0.5f), 1762000);
	zassert_equal(kraken_pwm_pulse_ns(-0.5f), 1238500);
	/* Any non-zero output leaves the deadband */
	zassert_equal(kraken_pwm_pulse_ns(0.001f), 1520484);
	zassert_equal(kraken_pwm_pulse_ns(-0.001f), 1479517);
	/* Clamping and NaN */
	zassert_equal(kraken_pwm_pulse_ns(5.0f), 2004000);
	zassert_equal(kraken_pwm_pulse_ns(-5.0f), 997000);
	zassert_equal(kraken_pwm_pulse_ns(NAN), 1500000);
}

ZTEST(kraken, test_pwm_output)
{
	struct kraken_pwm motor;

	zassert_ok(kraken_pwm_init(&motor, &pwm_spec));
	zassert_equal(fake_pwm_set_cycles_fake.call_count, 1);
	zassert_equal(fake_pwm_set_cycles_fake.arg1_val, 3, "wrong channel");
	zassert_equal(fake_pwm_set_cycles_fake.arg2_val, CONFIG_KRAKEN_PWM_PERIOD_US * 1000U);
	zassert_equal(fake_pwm_set_cycles_fake.arg3_val, 1500000);

	zassert_ok(kraken_pwm_set(&motor, 1.0f));
	zassert_equal(fake_pwm_set_cycles_fake.arg3_val, 2004000);
	zassert_true(motor.enabled);

	kraken_pwm_set_inverted(&motor, true);
	zassert_ok(kraken_pwm_set(&motor, 1.0f));
	zassert_equal(fake_pwm_set_cycles_fake.arg3_val, 997000);
	assert_close(motor.output, 1.0f, 0.0f);

	zassert_ok(kraken_pwm_disable(&motor));
	zassert_equal(fake_pwm_set_cycles_fake.arg3_val, 0);
	zassert_false(motor.enabled);
}

static void send_frame(uint32_t id, uint8_t dlc, const uint8_t *data)
{
	struct can_frame frame = {.id = id, .flags = CAN_FRAME_IDE, .dlc = dlc};

	if (data != NULL) {
		memcpy(frame.data, data, dlc);
	}

	zassert_ok(can_send(can_dev, &frame, K_MSEC(100), NULL, NULL));
}

ZTEST(kraken, test_can_monitor)
{
	struct kraken_can_device devices[8];
	struct frc_can_heartbeat hb = {
		.enabled = true, .system_watchdog = true, .match_time_s = 42};
	struct frc_can_heartbeat out;
	uint8_t data[8];
	int64_t age;
	size_t n;

	zassert_ok(can_set_mode(can_dev, CAN_MODE_LOOPBACK));
	zassert_ok(can_start(can_dev));
	zassert_ok(kraken_can_monitor_start(can_dev));
	zassert_equal(kraken_can_monitor_start(can_dev), -EALREADY);

	zassert_equal(kraken_can_monitor_heartbeat(&out, &age), -ENODATA);

	/* Two motor controllers (device numbers 1 and 2), one with two APIs */
	send_frame(0x02041401, 8, NULL);
	send_frame(0x02041441, 8, NULL);
	send_frame(0x02041401, 8, NULL);
	send_frame(0x02041402, 8, NULL);
	/* Another CTRE device type and a non-CTRE device */
	send_frame(0x08041405, 8, NULL);
	send_frame(0x02051401, 8, NULL);

	frc_can_heartbeat_encode(&hb, data);
	send_frame(FRC_CAN_HEARTBEAT_ID, 8, data);

	k_msleep(10);

	n = kraken_can_monitor_devices(devices, ARRAY_SIZE(devices), 0);
	zassert_equal(n, 3);

	zassert_equal(devices[0].device_type, FRC_CAN_DEV_MOTOR_CONTROLLER);
	zassert_equal(devices[0].device_number, 1);
	zassert_equal(devices[0].frames, 3);
	zassert_equal(devices[0].api_id_count, 2);
	zassert_equal(devices[0].api_ids[0], 0x050);
	zassert_equal(devices[0].api_ids[1], 0x051);

	zassert_equal(devices[1].device_number, 2);
	zassert_equal(devices[1].frames, 1);
	zassert_equal(devices[2].device_type, FRC_CAN_DEV_POWER_DISTRIBUTION);

	zassert_ok(kraken_can_monitor_heartbeat(&out, &age));
	zassert_true(out.enabled);
	zassert_true(out.system_watchdog);
	zassert_equal(out.match_time_s, 42);
	zassert_true(age >= 0 && age < 100);

	/* Age filtering */
	k_msleep(50);
	send_frame(0x02041402, 0, NULL);
	k_msleep(5);
	n = kraken_can_monitor_devices(devices, ARRAY_SIZE(devices), 20);
	zassert_equal(n, 1);
	zassert_equal(devices[0].device_number, 2);

	kraken_can_monitor_reset();
	zassert_equal(kraken_can_monitor_devices(devices, ARRAY_SIZE(devices), 0), 0);

	zassert_ok(kraken_can_monitor_stop());
	zassert_equal(kraken_can_monitor_stop(), -EALREADY);
	zassert_ok(can_stop(can_dev));
}

ZTEST_SUITE(kraken, NULL, NULL, NULL, NULL, NULL);
