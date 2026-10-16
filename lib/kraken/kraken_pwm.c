/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PWM control of a Kraken (Talon FX). Pulse widths follow WPILib's
 * PWMTalonFX: 2.004 ms full forward, 1.520 ms / 1.480 ms deadband edges,
 * 1.500 ms neutral and 0.997 ms full reverse. CTR Electronics specifies a
 * 1-2 ms pulse with a 2.9-100 ms period.
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#include <canbus_rp2040/kraken.h>

#define US_TO_NS(us) ((float)(us) * 1000.0f)

uint32_t kraken_pwm_pulse_ns(float output)
{
	float ns;

	/* NaN compares unequal to itself; treat it as neutral */
	if (output != output || output == 0.0f) {
		return KRAKEN_PWM_CENTER_US * 1000U;
	}

	output = CLAMP(output, -1.0f, 1.0f);

	if (output > 0.0f) {
		ns = US_TO_NS(KRAKEN_PWM_DEADBAND_HIGH_US) +
		     output * US_TO_NS(KRAKEN_PWM_FULL_FORWARD_US - KRAKEN_PWM_DEADBAND_HIGH_US);
	} else {
		ns = US_TO_NS(KRAKEN_PWM_DEADBAND_LOW_US) +
		     output * US_TO_NS(KRAKEN_PWM_DEADBAND_LOW_US - KRAKEN_PWM_FULL_REVERSE_US);
	}

	return (uint32_t)(ns + 0.5f);
}

static int kraken_pwm_apply(struct kraken_pwm *motor, uint32_t pulse_ns)
{
	return pwm_set_dt(&motor->spec, PWM_USEC(CONFIG_KRAKEN_PWM_PERIOD_US), pulse_ns);
}

int kraken_pwm_set(struct kraken_pwm *motor, float output)
{
	int err;

	err = kraken_pwm_apply(motor, kraken_pwm_pulse_ns(motor->inverted ? -output : output));
	if (err == 0) {
		motor->output = output;
		motor->enabled = true;
	}

	return err;
}

int kraken_pwm_neutral(struct kraken_pwm *motor)
{
	return kraken_pwm_set(motor, 0.0f);
}

int kraken_pwm_disable(struct kraken_pwm *motor)
{
	int err = kraken_pwm_apply(motor, 0U);

	if (err == 0) {
		motor->output = 0.0f;
		motor->enabled = false;
	}

	return err;
}

void kraken_pwm_set_inverted(struct kraken_pwm *motor, bool inverted)
{
	motor->inverted = inverted;
}

int kraken_pwm_init(struct kraken_pwm *motor, const struct pwm_dt_spec *spec)
{
	if (!pwm_is_ready_dt(spec)) {
		return -ENODEV;
	}

	motor->spec = *spec;
	motor->inverted = false;
	motor->enabled = false;
	motor->output = 0.0f;

	return kraken_pwm_neutral(motor);
}
