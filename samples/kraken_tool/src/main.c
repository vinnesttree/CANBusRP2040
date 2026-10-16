/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kraken motor tool. Shell commands:
 *
 *   kraken spec <x60|x44> [foc]       motor data and DC motor model
 *   kraken pwm set <-1.0..1.0>        PWM output (duty cycle)
 *   kraken pwm neutral                neutral pulse (1.5 ms)
 *   kraken pwm off                    stop pulses (motor controller disables)
 *   kraken pwm invert <on|off>        invert direction
 *   kraken pwm status
 *   kraken can start [listen]         start CAN (bitrate from devicetree) and the monitor
 *   kraken can stop
 *   kraken can devices                CTR Electronics devices seen in the last second
 *   kraken can heartbeat              last robot controller heartbeat
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <canbus_rp2040/frc_can.h>
#include <canbus_rp2040/kraken.h>

#define PWM_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(PWM_NODE, pwms)
static const struct pwm_dt_spec pwm_spec = PWM_DT_SPEC_GET(PWM_NODE);
static struct kraken_pwm motor;
static bool motor_ready;
#define HAVE_PWM 1
#else
#define HAVE_PWM 0
#endif

#if DT_HAS_CHOSEN(zephyr_canbus)
static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
#define HAVE_CAN 1
#else
#define HAVE_CAN 0
#endif

static int cmd_spec(const struct shell *sh, size_t argc, char **argv)
{
	enum kraken_motor_type type;
	enum kraken_commutation commutation = KRAKEN_TRAPEZOIDAL;
	const struct kraken_motor_spec *spec;
	struct kraken_motor_model model;

	if (strcmp(argv[1], "x60") == 0) {
		type = KRAKEN_X60;
	} else if (strcmp(argv[1], "x44") == 0) {
		type = KRAKEN_X44;
	} else {
		shell_error(sh, "unknown motor %s (x60 or x44)", argv[1]);
		return -EINVAL;
	}

	if (argc > 2) {
		if (strcmp(argv[2], "foc") != 0) {
			shell_error(sh, "unknown option %s", argv[2]);
			return -EINVAL;
		}
		commutation = KRAKEN_FOC;
	}

	spec = kraken_motor_spec_get(type, commutation);
	kraken_motor_model_init(&model, spec);

	shell_print(sh, "%s at %.1f V", spec->name, (double)spec->nominal_voltage);
	shell_print(sh, "  free speed     %.0f RPM", (double)kraken_rad_s_to_rpm(spec->free_speed));
	shell_print(sh, "  free current   %.1f A", (double)spec->free_current);
	shell_print(sh, "  stall torque   %.2f N*m", (double)spec->stall_torque);
	shell_print(sh, "  stall current  %.0f A", (double)spec->stall_current);
	shell_print(sh, "  resistance     %.4f ohm", (double)model.resistance);
	shell_print(sh, "  Kv             %.1f RPM/V", (double)kraken_rad_s_to_rpm(model.kv));
	shell_print(sh, "  Kt             %.4f N*m/A", (double)model.kt);

	return 0;
}

#if HAVE_PWM
static int pwm_check(const struct shell *sh)
{
	if (!motor_ready) {
		shell_error(sh, "PWM output not available");
		return -ENODEV;
	}

	return 0;
}

static int cmd_pwm_set(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	float output;
	int err;

	ARG_UNUSED(argc);

	err = pwm_check(sh);
	if (err != 0) {
		return err;
	}

	output = strtof(argv[1], &end);
	if (*end != '\0' || output < -1.0f || output > 1.0f) {
		shell_error(sh, "output must be between -1.0 and 1.0");
		return -EINVAL;
	}

	err = kraken_pwm_set(&motor, output);
	if (err != 0) {
		shell_error(sh, "failed to set PWM (err %d)", err);
		return err;
	}

	shell_print(sh, "output %.3f, pulse %u us", (double)output,
		    kraken_pwm_pulse_ns(motor.inverted ? -output : output) / 1000U);

	return 0;
}

static int cmd_pwm_neutral(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = pwm_check(sh);
	if (err == 0) {
		err = kraken_pwm_neutral(&motor);
	}

	return err;
}

static int cmd_pwm_off(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = pwm_check(sh);
	if (err == 0) {
		err = kraken_pwm_disable(&motor);
	}

	return err;
}

static int cmd_pwm_invert(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);

	err = pwm_check(sh);
	if (err != 0) {
		return err;
	}

	if (strcmp(argv[1], "on") == 0) {
		kraken_pwm_set_inverted(&motor, true);
	} else if (strcmp(argv[1], "off") == 0) {
		kraken_pwm_set_inverted(&motor, false);
	} else {
		shell_error(sh, "expected on or off");
		return -EINVAL;
	}

	/* Re-apply the current output with the new direction */
	return motor.enabled ? kraken_pwm_set(&motor, motor.output) : 0;
}

static int cmd_pwm_status(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = pwm_check(sh);
	if (err != 0) {
		return err;
	}

	shell_print(sh, "pulses %s, output %.3f, inverted %s, period %u us",
		    motor.enabled ? "on" : "off", (double)motor.output,
		    motor.inverted ? "yes" : "no", CONFIG_KRAKEN_PWM_PERIOD_US);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pwm, SHELL_CMD_ARG(set, NULL, "Set output <-1.0..1.0>", cmd_pwm_set, 2, 0),
	SHELL_CMD(neutral, NULL, "Neutral pulse (1.5 ms)", cmd_pwm_neutral),
	SHELL_CMD(off, NULL, "Stop pulses", cmd_pwm_off),
	SHELL_CMD_ARG(invert, NULL, "Invert direction <on|off>", cmd_pwm_invert, 2, 0),
	SHELL_CMD(status, NULL, "Show PWM state", cmd_pwm_status), SHELL_SUBCMD_SET_END);
#endif /* HAVE_PWM */

#if HAVE_CAN
static int cmd_can_start(const struct shell *sh, size_t argc, char **argv)
{
	can_mode_t mode = CAN_MODE_NORMAL;
	int err;

	if (argc > 1) {
		if (strcmp(argv[1], "listen") != 0) {
			shell_error(sh, "unknown option %s", argv[1]);
			return -EINVAL;
		}
		mode = CAN_MODE_LISTENONLY;
	}

	if (!device_is_ready(can_dev)) {
		shell_error(sh, "CAN controller not ready");
		return -ENODEV;
	}

	err = can_set_mode(can_dev, mode);
	if (err == 0) {
		err = kraken_can_monitor_start(can_dev);
		err = (err == -EALREADY) ? 0 : err;
	}
	if (err == 0) {
		err = can_start(can_dev);
	}

	if (err != 0) {
		shell_error(sh, "failed to start CAN (err %d)", err);
		return err;
	}

	shell_print(sh, "CAN started%s", mode == CAN_MODE_LISTENONLY ? " (listen-only)" : "");

	return 0;
}

static int cmd_can_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	(void)can_stop(can_dev);
	(void)kraken_can_monitor_stop();
	shell_print(sh, "CAN stopped");

	return 0;
}

static int cmd_can_devices(const struct shell *sh, size_t argc, char **argv)
{
	struct kraken_can_device devices[CONFIG_KRAKEN_CAN_MONITOR_MAX_DEVICES];
	size_t n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	n = kraken_can_monitor_devices(devices, ARRAY_SIZE(devices), 1000);
	if (n == 0) {
		shell_print(sh, "no CTR Electronics devices seen in the last second");
		return 0;
	}

	shell_print(sh, "type                  id  frames  APIs seen");
	for (size_t i = 0; i < n; i++) {
		const char *name = frc_can_device_type_name(devices[i].device_type);
		char apis[8 * KRAKEN_CAN_MONITOR_API_IDS + 1] = "";
		size_t pos = 0;

		for (uint8_t a = 0; a < devices[i].api_id_count; a++) {
			pos += snprintf(&apis[pos], sizeof(apis) - pos, " 0x%03x",
					devices[i].api_ids[a]);
		}

		shell_print(sh, "%-20s %3u  %6u %s", name != NULL ? name : "unknown",
			    devices[i].device_number, devices[i].frames, apis);
	}

	return 0;
}

static int cmd_can_heartbeat(const struct shell *sh, size_t argc, char **argv)
{
	struct frc_can_heartbeat hb;
	int64_t age;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (kraken_can_monitor_heartbeat(&hb, &age) != 0) {
		shell_print(sh, "no robot controller heartbeat received");
		return 0;
	}

	shell_print(sh, "heartbeat %lld ms ago%s", (long long)age,
		    age > FRC_CAN_HEARTBEAT_TIMEOUT_MS ? " (timed out)" : "");
	shell_print(sh, "  enabled %u, watchdog %u, autonomous %u, test %u, red %u", hb.enabled,
		    hb.system_watchdog, hb.autonomous, hb.test_mode, hb.red_alliance);
	shell_print(sh, "  match %u, replay %u, match time %u s", hb.match_number, hb.replay_number,
		    hb.match_time_s);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_can,
	SHELL_CMD_ARG(start, NULL, "Start CAN and the device monitor [listen]", cmd_can_start, 1,
		      1),
	SHELL_CMD(stop, NULL, "Stop CAN", cmd_can_stop),
	SHELL_CMD(devices, NULL, "List CTR Electronics devices", cmd_can_devices),
	SHELL_CMD(heartbeat, NULL, "Show robot controller heartbeat", cmd_can_heartbeat),
	SHELL_SUBCMD_SET_END);
#endif /* HAVE_CAN */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_kraken,
			       SHELL_CMD_ARG(spec, NULL, "Motor data <x60|x44> [foc]", cmd_spec, 2,
					     1),
#if HAVE_PWM
			       SHELL_CMD(pwm, &sub_pwm, "PWM control", NULL),
#endif
#if HAVE_CAN
			       SHELL_CMD(can, &sub_can, "CAN bus monitor", NULL),
#endif
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(kraken, &sub_kraken, "Kraken X60/X44 tools", NULL);

int main(void)
{
#if HAVE_PWM
	int err = kraken_pwm_init(&motor, &pwm_spec);

	if (err == 0) {
		/* Start without pulses: the motor stays disabled until commanded */
		err = kraken_pwm_disable(&motor);
	}

	motor_ready = (err == 0);
	if (!motor_ready) {
		printk("PWM output not available (err %d)\n", err);
	}
#endif

	printk("Kraken tool ready, type 'kraken help'\n");

	return 0;
}
