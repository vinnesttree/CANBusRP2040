/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Support for WestCoast Products Kraken X60 / X44 motors (Talon FX
 * integrated motor controller by CTR Electronics).
 *
 * What this module provides, and what it deliberately does not:
 *
 * - Motor data and a DC motor model for Kraken X60 and X44 with trapezoidal
 *   commutation and FOC, using the constants published by CTR Electronics
 *   (as used by WPILib's DCMotor class).
 * - PWM control. The Talon FX inside the Kraken detects CAN or PWM control
 *   automatically on the same pair of wires (PWM signal on CAN-H / yellow,
 *   ground on CAN-L / green). Pulses follow the WPILib PWMTalonFX bounds.
 * - A CAN bus monitor that lists CTR Electronics devices (FRC device type
 *   "motor controller" = Talon FX based motors such as Kraken, and other CTRE
 *   motor controllers) by device ID, plus the robot controller heartbeat.
 *
 * The CAN protocol used by Phoenix 6 to configure and command a Talon FX
 * (control requests, configs, status signals, FOC, Motion Magic, ...) is not
 * publicly documented by CTR Electronics, so it is NOT reimplemented here.
 * To use every Kraken feature over CAN, run CTRE's Phoenix 6 library on a
 * Linux host and use this board as a USB-to-CAN adapter (see the
 * usb_can_adapter sample and the project README).
 */

#ifndef CANBUS_RP2040_INCLUDE_KRAKEN_H_
#define CANBUS_RP2040_INCLUDE_KRAKEN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#include <canbus_rp2040/frc_can.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Motor data and model
 * @{
 */

/** Kraken motor variants. */
enum kraken_motor_type {
	/** Kraken X60. */
	KRAKEN_X60,
	/** Kraken X44. */
	KRAKEN_X44,
};

/** Motor commutation used by the Talon FX. */
enum kraken_commutation {
	/** Trapezoidal (six-step) commutation. */
	KRAKEN_TRAPEZOIDAL,
	/** Field oriented control (requires a Phoenix Pro license). */
	KRAKEN_FOC,
};

/** Published motor characteristics at the nominal voltage. */
struct kraken_motor_spec {
	/** Human readable name. */
	const char *name;
	/** Nominal supply voltage in V. */
	float nominal_voltage;
	/** Stall torque in N*m. */
	float stall_torque;
	/** Stall current in A. */
	float stall_current;
	/** Free (no load) current in A. */
	float free_current;
	/** Free speed in rad/s. */
	float free_speed;
};

/**
 * @brief Get the published characteristics of a Kraken motor.
 *
 * @return Motor data, or NULL for an invalid combination.
 */
const struct kraken_motor_spec *kraken_motor_spec_get(enum kraken_motor_type type,
						      enum kraken_commutation commutation);

/** Linear DC motor model constants derived from a @ref kraken_motor_spec. */
struct kraken_motor_model {
	/** Winding resistance in ohm. */
	float resistance;
	/** Velocity constant in (rad/s)/V. */
	float kv;
	/** Torque constant in N*m/A. */
	float kt;
};

/**
 * @brief Derive the DC motor model.
 *
 * R = V / I_stall, Kv = w_free / (V - R * I_free), Kt = T_stall / I_stall
 * (the same model as WPILib's DCMotor).
 */
void kraken_motor_model_init(struct kraken_motor_model *model,
			     const struct kraken_motor_spec *spec);

/** @brief Current in A drawn at @a speed (rad/s) with @a voltage applied. */
float kraken_motor_current(const struct kraken_motor_model *model, float speed, float voltage);

/** @brief Torque in N*m produced by @a current (A). */
float kraken_motor_torque(const struct kraken_motor_model *model, float current);

/** @brief Voltage needed to produce @a torque (N*m) at @a speed (rad/s). */
float kraken_motor_voltage(const struct kraken_motor_model *model, float torque, float speed);

/** @brief Speed in rad/s reached with @a voltage applied while producing @a torque. */
float kraken_motor_speed(const struct kraken_motor_model *model, float torque, float voltage);

/** @brief Convert revolutions per minute to rad/s. */
static inline float kraken_rpm_to_rad_s(float rpm)
{
	return rpm * (2.0f * 3.14159265358979f / 60.0f);
}

/** @brief Convert rad/s to revolutions per minute. */
static inline float kraken_rad_s_to_rpm(float rad_s)
{
	return rad_s * (60.0f / (2.0f * 3.14159265358979f));
}

/** @brief Convert rotations (the Phoenix 6 position unit) to radians. */
static inline float kraken_rotations_to_rad(float rotations)
{
	return rotations * (2.0f * 3.14159265358979f);
}

/** @brief Convert radians to rotations. */
static inline float kraken_rad_to_rotations(float rad)
{
	return rad / (2.0f * 3.14159265358979f);
}

/** @} */

/**
 * @name PWM control
 * @{
 */

/** Pulse width for full forward output (WPILib PWMTalonFX). */
#define KRAKEN_PWM_FULL_FORWARD_US  2004
/** Upper end of the neutral deadband. */
#define KRAKEN_PWM_DEADBAND_HIGH_US 1520
/** Neutral (center of the deadband). */
#define KRAKEN_PWM_CENTER_US        1500
/** Lower end of the neutral deadband. */
#define KRAKEN_PWM_DEADBAND_LOW_US  1480
/** Pulse width for full reverse output. */
#define KRAKEN_PWM_FULL_REVERSE_US  997

/** Shortest PWM period accepted by the Talon FX (2.9 ms). */
#define KRAKEN_PWM_PERIOD_MIN_US 2900
/** Longest PWM period accepted by the Talon FX (100 ms). */
#define KRAKEN_PWM_PERIOD_MAX_US 100000

/** A Kraken driven by a PWM output. */
struct kraken_pwm {
	/** PWM output connected to the motor's CAN-H (yellow) wire. */
	struct pwm_dt_spec spec;
	/** Invert the direction of rotation. */
	bool inverted;
	/** Pulses are being generated. */
	bool enabled;
	/** Last requested output in [-1, 1]. */
	float output;
};

/**
 * @brief Pulse width in nanoseconds for an output in [-1, 1].
 *
 * Positive outputs map linearly onto [1520 us, 2004 us], negative outputs
 * onto [997 us, 1480 us] and zero to 1500 us (WPILib PWM scaling). Values
 * outside [-1, 1] are clamped.
 */
uint32_t kraken_pwm_pulse_ns(float output);

/**
 * @brief Initialise a PWM controlled Kraken and output neutral.
 *
 * The PWM period is CONFIG_KRAKEN_PWM_PERIOD_US; the period in @a spec is
 * ignored.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the PWM controller is not ready.
 * @retval <0 PWM driver error.
 */
int kraken_pwm_init(struct kraken_pwm *motor, const struct pwm_dt_spec *spec);

/** @brief Set the output in [-1, 1] (duty cycle of the supply voltage). */
int kraken_pwm_set(struct kraken_pwm *motor, float output);

/** @brief Output the neutral pulse (1500 us). */
int kraken_pwm_neutral(struct kraken_pwm *motor);

/**
 * @brief Stop generating pulses. The motor controller treats the missing
 * signal as a loss of control and disables its output.
 */
int kraken_pwm_disable(struct kraken_pwm *motor);

/** @brief Invert the direction of rotation for subsequent kraken_pwm_set() calls. */
void kraken_pwm_set_inverted(struct kraken_pwm *motor, bool inverted);

/** @} */

/**
 * @name CAN bus monitor
 * @{
 */

/** Number of distinct API identifiers recorded per device. */
#ifdef CONFIG_KRAKEN_CAN_MONITOR_API_IDS
#define KRAKEN_CAN_MONITOR_API_IDS CONFIG_KRAKEN_CAN_MONITOR_API_IDS
#else
#define KRAKEN_CAN_MONITOR_API_IDS 1
#endif

/** A CTR Electronics device seen on the CAN bus. */
struct kraken_can_device {
	/** FRC device type (2 = motor controller, e.g. Kraken / Talon FX). */
	uint8_t device_type;
	/** Device number (the CAN ID configured in Phoenix Tuner). */
	uint8_t device_number;
	/** Number of frames received from the device. */
	uint32_t frames;
	/** Uptime in ms when the last frame was received. */
	int64_t last_seen_ms;
	/** Number of valid entries in @a api_ids. */
	uint8_t api_id_count;
	/** Distinct API identifiers seen (the first KRAKEN_CAN_MONITOR_API_IDS). */
	uint16_t api_ids[KRAKEN_CAN_MONITOR_API_IDS];
};

/**
 * @brief Start listening for CTR Electronics devices and the robot heartbeat.
 *
 * Installs RX filters on @a can. The CAN controller must be configured and
 * started by the application.
 *
 * @retval 0 on success.
 * @retval -EALREADY if the monitor is already running.
 * @retval <0 if the RX filters could not be installed.
 */
int kraken_can_monitor_start(const struct device *can);

/** @brief Stop the monitor and remove its RX filters. */
int kraken_can_monitor_stop(void);

/** @brief Forget all devices seen so far. */
void kraken_can_monitor_reset(void);

/**
 * @brief Copy the devices seen within the last @a max_age_ms milliseconds.
 *
 * @param devices Destination array.
 * @param max Capacity of @a devices.
 * @param max_age_ms Only report devices seen recently (0: report all).
 *
 * @return Number of devices copied.
 */
size_t kraken_can_monitor_devices(struct kraken_can_device *devices, size_t max,
				  uint32_t max_age_ms);

/**
 * @brief Get the last robot controller heartbeat.
 *
 * @param hb Decoded heartbeat.
 * @param age_ms Time since it was received in ms.
 *
 * @retval 0 on success.
 * @retval -ENODATA if no heartbeat was received.
 */
int kraken_can_monitor_heartbeat(struct frc_can_heartbeat *hb, int64_t *age_ms);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_KRAKEN_H_ */
