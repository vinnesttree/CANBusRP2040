/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kraken X60 / X44 motor data and DC motor model.
 *
 * The values are the ones published by CTR Electronics and used by WPILib's
 * DCMotor class (wpimath DCMotor.java, 2026 release):
 *   - Kraken X60: store.ctr-electronics.com/announcing-kraken-x60
 *   - Kraken X44: motors.ctr-electronics.com/dyno/dynometer-testing
 */

#include <zephyr/sys/util.h>

#include <canbus_rp2040/kraken.h>

#define RPM_TO_RAD_S(rpm) ((rpm) * (2.0f * 3.14159265358979f / 60.0f))

/* clang-format off */
static const struct kraken_motor_spec kraken_specs[2][2] = {
	[KRAKEN_X60] = {
		[KRAKEN_TRAPEZOIDAL] = {
			.name = "Kraken X60",
			.nominal_voltage = 12.0f,
			.stall_torque = 7.09f,
			.stall_current = 366.0f,
			.free_current = 2.0f,
			.free_speed = RPM_TO_RAD_S(6000.0f),
		},
		[KRAKEN_FOC] = {
			.name = "Kraken X60 (FOC)",
			.nominal_voltage = 12.0f,
			.stall_torque = 9.37f,
			.stall_current = 483.0f,
			.free_current = 2.0f,
			.free_speed = RPM_TO_RAD_S(5800.0f),
		},
	},
	[KRAKEN_X44] = {
		[KRAKEN_TRAPEZOIDAL] = {
			.name = "Kraken X44",
			.nominal_voltage = 12.0f,
			.stall_torque = 4.11f,
			.stall_current = 279.0f,
			.free_current = 2.0f,
			.free_speed = RPM_TO_RAD_S(7758.0f),
		},
		[KRAKEN_FOC] = {
			.name = "Kraken X44 (FOC)",
			.nominal_voltage = 12.0f,
			.stall_torque = 5.01f,
			.stall_current = 329.0f,
			.free_current = 2.0f,
			.free_speed = RPM_TO_RAD_S(7368.0f),
		},
	},
};
/* clang-format on */

const struct kraken_motor_spec *kraken_motor_spec_get(enum kraken_motor_type type,
						      enum kraken_commutation commutation)
{
	if ((unsigned int)type >= ARRAY_SIZE(kraken_specs) ||
	    (unsigned int)commutation >= ARRAY_SIZE(kraken_specs[0])) {
		return NULL;
	}

	return &kraken_specs[type][commutation];
}

void kraken_motor_model_init(struct kraken_motor_model *model, const struct kraken_motor_spec *spec)
{
	model->resistance = spec->nominal_voltage / spec->stall_current;
	model->kv =
		spec->free_speed / (spec->nominal_voltage - model->resistance * spec->free_current);
	model->kt = spec->stall_torque / spec->stall_current;
}

float kraken_motor_current(const struct kraken_motor_model *model, float speed, float voltage)
{
	return (voltage - speed / model->kv) / model->resistance;
}

float kraken_motor_torque(const struct kraken_motor_model *model, float current)
{
	return current * model->kt;
}

float kraken_motor_voltage(const struct kraken_motor_model *model, float torque, float speed)
{
	return torque / model->kt * model->resistance + speed / model->kv;
}

float kraken_motor_speed(const struct kraken_motor_model *model, float torque, float voltage)
{
	return voltage * model->kv - torque / model->kt * model->resistance * model->kv;
}
