/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * p6 shell commands. Everything is looked up by its Phoenix 6 name in the
 * generated descriptor tables, so every request, parameter, configuration
 * field and signal is available without request-specific code.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <canbus_rp2040/phoenix6.h>

#define REQUEST_TIMEOUT_MS 200

static const char *const type_names[] = {
	[PHOENIX6_TYPE_FLOAT] = "float",     [PHOENIX6_TYPE_BOOL] = "bool",
	[PHOENIX6_TYPE_INT] = "int",         [PHOENIX6_TYPE_ENUM] = "enum",
	[PHOENIX6_TYPE_CONTROL] = "request",
};

static void enable_feed(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(enable_work, enable_feed);
static atomic_t enabled;

static void enable_feed(struct k_work *work)
{
	if (!atomic_get(&enabled)) {
		return;
	}

	(void)phoenix6_feed_enable(CONFIG_SAMPLE_ENABLE_FEED_MS);
	(void)k_work_reschedule(k_work_delayable_from_work(work),
				K_MSEC(CONFIG_SAMPLE_ENABLE_FEED_MS / 2));
}

static int parse_device(const struct shell *sh, const char *str, uint8_t *device)
{
	char *end;
	unsigned long v = strtoul(str, &end, 0);

	if (*end != '\0' || v > 62) {
		shell_error(sh, "invalid device ID %s (0-62)", str);
		return -EINVAL;
	}

	*device = (uint8_t)v;
	return 0;
}

/* Parse a value of the given type; enums by Phoenix 6 member name or number */
static int parse_value(const struct shell *sh, uint8_t type, uint8_t enum_index, const char *str,
		       double *out)
{
	char *end;

	switch (type) {
	case PHOENIX6_TYPE_BOOL:
		if (strcmp(str, "true") == 0 || strcmp(str, "1") == 0) {
			*out = 1.0;
			return 0;
		}
		if (strcmp(str, "false") == 0 || strcmp(str, "0") == 0) {
			*out = 0.0;
			return 0;
		}
		break;
	case PHOENIX6_TYPE_ENUM: {
		int32_t v;

		if (phoenix6_enum_value(enum_index, str, &v) == 0) {
			*out = v;
			return 0;
		}
		__fallthrough;
	}
	case PHOENIX6_TYPE_INT: {
		long v = strtol(str, &end, 0);

		if (*end == '\0' && (type == PHOENIX6_TYPE_INT ||
				     phoenix6_enum_name(enum_index, (int32_t)v) != NULL)) {
			*out = v;
			return 0;
		}
		break;
	}
	case PHOENIX6_TYPE_FLOAT:
		*out = strtod(str, &end);
		if (*end == '\0') {
			return 0;
		}
		break;
	default:
		break;
	}

	shell_error(sh, "invalid %s value '%s'", type_names[type], str);
	if (type == PHOENIX6_TYPE_ENUM) {
		const struct phoenix6_enum_desc *e = &phoenix6_enum_descs[enum_index];

		for (uint8_t i = 0; i < e->count; i++) {
			shell_print(sh, "  %s (%d)", e->members[i].name, e->members[i].value);
		}
	}

	return -EINVAL;
}

static void store_value(uint8_t type, uint8_t *field, double v)
{
	if (type == PHOENIX6_TYPE_FLOAT) {
		memcpy(field, &v, sizeof(v));
	} else if (type == PHOENIX6_TYPE_BOOL) {
		bool b = v != 0.0;

		memcpy(field, &b, sizeof(b));
	} else {
		int32_t i = (int32_t)v;

		memcpy(field, &i, sizeof(i));
	}
}

static void print_value(const struct shell *sh, const char *name, uint8_t type, uint8_t enum_index,
			double v, const char *unit)
{
	const char *member;

	switch (type) {
	case PHOENIX6_TYPE_BOOL:
		shell_print(sh, "%s = %s", name, v != 0.0 ? "true" : "false");
		break;
	case PHOENIX6_TYPE_ENUM:
		member = phoenix6_enum_name(enum_index, (int32_t)v);
		shell_print(sh, "%s = %s (%d)", name, member ? member : "?", (int)v);
		break;
	case PHOENIX6_TYPE_INT:
		shell_print(sh, "%s = %d", name, (int)v);
		break;
	default:
		shell_print(sh, "%s = %.6g%s%s", name, v, unit ? " " : "", unit ? unit : "");
		break;
	}
}

static void print_status(const struct shell *sh, int err, int32_t status)
{
	if (err != 0) {
		shell_error(sh, "request failed: %d%s", err,
			    err == -ENOTCONN ? " (host bridge not connected)" : "");
	} else if (status != PHOENIX6_STATUS_OK) {
		shell_warn(sh, "Phoenix 6 status %d", status);
	} else {
		shell_print(sh, "OK");
	}
}

/* Set "name" or "nested.name" of a request's parameter struct */
static int set_param(const struct shell *sh, const struct phoenix6_control_desc *desc,
		     uint8_t *params, const char *path, const char *value)
{
	const char *dot = strchr(path, '.');
	size_t len = (dot != NULL) ? (size_t)(dot - path) : strlen(path);

	for (uint8_t i = 0; i < desc->param_count; i++) {
		const struct phoenix6_param_desc *p = &desc->params[i];
		double v;

		if (strlen(p->name) != len || strncmp(p->name, path, len) != 0) {
			continue;
		}

		if (p->type == PHOENIX6_TYPE_CONTROL) {
			if (dot == NULL) {
				shell_error(sh, "use %s.<parameter>", p->name);
				return -EINVAL;
			}
			return set_param(sh, phoenix6_control_desc_get(p->control),
					 params + p->offset, dot + 1, value);
		}

		if (dot != NULL || parse_value(sh, p->type, p->enum_index, value, &v) != 0) {
			return -EINVAL;
		}

		store_value(p->type, params + p->offset, v);
		return 0;
	}

	shell_error(sh, "%s has no parameter %.*s (see p6 info %s)", desc->name, (int)len, path,
		    desc->name);
	return -EINVAL;
}

static bool params_complete(const struct shell *sh, const struct phoenix6_control_desc *desc,
			    const char *prefix, int argc, char **argv)
{
	bool ok = true;

	for (uint8_t i = 0; i < desc->param_count; i++) {
		const struct phoenix6_param_desc *p = &desc->params[i];
		char path[64];
		bool given = false;

		snprintf(path, sizeof(path), "%s%s", prefix, p->name);

		if (p->type == PHOENIX6_TYPE_CONTROL) {
			strncat(path, ".", sizeof(path) - strlen(path) - 1);
			ok &= params_complete(sh, phoenix6_control_desc_get(p->control), path, argc,
					      argv);
			continue;
		}

		if (!p->required) {
			continue;
		}

		for (int a = 0; a < argc; a++) {
			size_t n = strlen(path);

			if (strncmp(argv[a], path, n) == 0 && argv[a][n] == '=') {
				given = true;
			}
		}

		if (!given) {
			shell_error(sh, "missing required parameter %s", path);
			ok = false;
		}
	}

	return ok;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Phoenix 6 API %s, schema 0x%08x", PHOENIX6_GENERATED_VERSION,
		    PHOENIX6_SCHEMA_HASH);
	shell_print(sh, "host bridge: %s%s%s",
		    phoenix6_bridge_is_connected() ? "connected, Phoenix 6 " : "not connected",
		    phoenix6_bridge_is_connected() ? phoenix6_bridge_host_version() : "",
		    atomic_get(&enabled) ? ", actuators enabled" : "");

	return 0;
}

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (strcmp(argv[1], "on") == 0) {
		atomic_set(&enabled, 1);
		(void)k_work_reschedule(&enable_work, K_NO_WAIT);
		shell_print(sh, "feeding enable (%d ms)", CONFIG_SAMPLE_ENABLE_FEED_MS);
	} else if (strcmp(argv[1], "off") == 0) {
		atomic_set(&enabled, 0);
		(void)k_work_cancel_delayable(&enable_work);
		print_status(sh, phoenix6_feed_enable(0), PHOENIX6_STATUS_OK);
	} else {
		shell_error(sh, "on or off");
		return -EINVAL;
	}

	return 0;
}

static int cmd_control(const struct shell *sh, size_t argc, char **argv)
{
	const struct phoenix6_control_desc *desc = phoenix6_control_desc_find(argv[2]);
	struct phoenix6_control req;
	uint8_t device;
	int64_t start;
	int err;

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	if (desc == NULL) {
		shell_error(sh, "unknown request %s (see p6 list controls)", argv[2]);
		return -EINVAL;
	}

	if (!params_complete(sh, desc, "", argc - 3, &argv[3])) {
		return -EINVAL;
	}

	(void)phoenix6_control_init(&req, desc->id);

	for (size_t i = 3; i < argc; i++) {
		char *eq = strchr(argv[i], '=');

		if (eq == NULL) {
			shell_error(sh, "expected parameter=value, got %s", argv[i]);
			return -EINVAL;
		}

		*eq = '\0';
		if (strcmp(argv[i], "update_freq_hz") == 0) {
			req.update_freq_hz = strtod(eq + 1, NULL);
			continue;
		}

		err = set_param(sh, desc, (uint8_t *)&req + PHOENIX6_CONTROL_PARAMS_OFFSET, argv[i],
				eq + 1);
		if (err != 0) {
			return err;
		}
	}

	err = phoenix6_set_control(device, &req);
	if (err != 0) {
		print_status(sh, err, 0);
		return err;
	}

	/* The result arrives asynchronously */
	start = k_uptime_get();
	k_msleep(50);
	while (k_uptime_get() - start < REQUEST_TIMEOUT_MS) {
		int32_t status = phoenix6_last_control_status(device);

		if (status != PHOENIX6_STATUS_NONE) {
			print_status(sh, 0, status);
			break;
		}
		k_msleep(10);
	}

	if (!atomic_get(&enabled)) {
		shell_warn(sh, "actuators are disabled, see p6 enable on");
	}

	return 0;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	struct phoenix6_control req = PHOENIX6_NEUTRAL_OUT();
	uint8_t device;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	print_status(sh, phoenix6_set_control(device, &req), PHOENIX6_STATUS_OK);

	return 0;
}

static int cmd_config_get(const struct shell *sh, size_t argc, char **argv)
{
	const struct phoenix6_config_desc *d = phoenix6_config_desc_find(argv[2]);
	int32_t status;
	uint8_t device;
	double value;
	int err;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	if (d == NULL) {
		shell_error(sh, "unknown field %s (see p6 list configs)", argv[2]);
		return -EINVAL;
	}

	err = phoenix6_config_get(device, d->id, &value, REQUEST_TIMEOUT_MS, &status);
	if (err != 0 || status != PHOENIX6_STATUS_OK) {
		print_status(sh, err, status);
		return 0;
	}

	print_value(sh, argv[2], d->type, d->enum_index, value, NULL);

	return 0;
}

static int cmd_config_set(const struct shell *sh, size_t argc, char **argv)
{
	const struct phoenix6_config_desc *d = phoenix6_config_desc_find(argv[2]);
	int32_t status = 0;
	uint8_t device;
	double value;
	int err;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	if (d == NULL) {
		shell_error(sh, "unknown field %s (see p6 list configs)", argv[2]);
		return -EINVAL;
	}

	if (parse_value(sh, d->type, d->enum_index, argv[3], &value) != 0) {
		return -EINVAL;
	}

	err = phoenix6_config_set(device, d->id, value, REQUEST_TIMEOUT_MS, &status);
	print_status(sh, err, status);

	return 0;
}

static int cmd_factory_default(const struct shell *sh, size_t argc, char **argv)
{
	int32_t status = 0;
	uint8_t device;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	print_status(sh, phoenix6_factory_default(device, REQUEST_TIMEOUT_MS, &status), status);

	return 0;
}

static int cmd_setpos(const struct shell *sh, size_t argc, char **argv)
{
	int32_t status = 0;
	uint8_t device;
	double rotations;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0 ||
	    parse_value(sh, PHOENIX6_TYPE_FLOAT, 0xFF, argv[2], &rotations) != 0) {
		return -EINVAL;
	}

	print_status(sh, phoenix6_set_position(device, rotations, REQUEST_TIMEOUT_MS, &status),
		     status);

	return 0;
}

static int cmd_clear_faults(const struct shell *sh, size_t argc, char **argv)
{
	int32_t status = 0;
	uint8_t device;

	ARG_UNUSED(argc);

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	print_status(sh, phoenix6_clear_sticky_faults(device, REQUEST_TIMEOUT_MS, &status), status);

	return 0;
}

static int cmd_signal(const struct shell *sh, size_t argc, char **argv)
{
	const struct phoenix6_signal_desc *d = phoenix6_signal_desc_find(argv[2]);
	struct phoenix6_signal_value v;
	int32_t status = 0;
	double freq = 10.0;
	uint8_t device;
	int64_t start;
	int err;

	if (parse_device(sh, argv[1], &device) != 0) {
		return -EINVAL;
	}

	if (d == NULL) {
		shell_error(sh, "unknown signal %s (see p6 list signals)", argv[2]);
		return -EINVAL;
	}

	if (argc > 3 && parse_value(sh, PHOENIX6_TYPE_FLOAT, 0xFF, argv[3], &freq) != 0) {
		return -EINVAL;
	}

	if (argc > 3 || phoenix6_get_signal(device, d->id, &v) == -ENODATA) {
		err = phoenix6_subscribe(device, d->id, (float)freq, &status);
		if (err != 0 || status != PHOENIX6_STATUS_OK || freq == 0.0) {
			print_status(sh, err, status);
			return 0;
		}
	}

	start = k_uptime_get();
	while (phoenix6_get_signal(device, d->id, &v) != 0) {
		if (k_uptime_get() - start > 1000) {
			shell_error(sh, "no value received");
			return 0;
		}
		k_msleep(10);
	}

	print_value(sh, d->name, d->type, d->enum_index, v.value, d->unit);
	if (v.status != PHOENIX6_STATUS_OK) {
		shell_warn(sh, "Phoenix 6 status %d", v.status);
	}
	shell_print(sh, "received %lld ms ago, %.1f ms old when sent",
		    (long long)(k_uptime_get() - v.received_ms), (double)v.age_ms);

	return 0;
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (strcmp(argv[1], "controls") == 0) {
		for (int i = 0; i < PHOENIX6_CONTROL_ID_COUNT - 1; i++) {
			const struct phoenix6_control_desc *d = &phoenix6_control_descs[i];

			shell_print(sh, "%-48s%s%s", d->name,
				    (d->flags & PHOENIX6_CONTROL_FLAG_PRO) ? " Pro" : "",
				    (d->flags & PHOENIX6_CONTROL_FLAG_CANIVORE) ? " CANivore" : "");
		}
	} else if (strcmp(argv[1], "configs") == 0) {
		for (int i = 0; i < PHOENIX6_CONFIG_COUNT - 1; i++) {
			const struct phoenix6_config_desc *d = &phoenix6_config_descs[i];

			shell_print(sh, "%s.%s (%s)", d->group, d->field, type_names[d->type]);
		}
	} else if (strcmp(argv[1], "signals") == 0) {
		for (int i = 0; i < PHOENIX6_SIGNAL_COUNT - 1; i++) {
			const struct phoenix6_signal_desc *d = &phoenix6_signal_descs[i];

			shell_print(sh, "%s (%s%s%s)", d->name,
				    (d->type == PHOENIX6_TYPE_ENUM)
					    ? phoenix6_enum_descs[d->enum_index].name
					    : type_names[d->type],
				    d->unit ? ", " : "", d->unit ? d->unit : "");
		}
	} else {
		shell_error(sh, "controls, configs or signals");
		return -EINVAL;
	}

	return 0;
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	const struct phoenix6_control_desc *d = phoenix6_control_desc_find(argv[1]);

	ARG_UNUSED(argc);

	if (d == NULL) {
		shell_error(sh, "unknown request %s (see p6 list controls)", argv[1]);
		return -EINVAL;
	}

	shell_print(sh, "%s: default update_freq_hz=%u%s%s", d->name, d->update_freq_hz,
		    (d->flags & PHOENIX6_CONTROL_FLAG_PRO) ? ", requires Phoenix Pro" : "",
		    (d->flags & PHOENIX6_CONTROL_FLAG_CANIVORE) ? " and a CANivore" : "");

	for (uint8_t i = 0; i < d->param_count; i++) {
		const struct phoenix6_param_desc *p = &d->params[i];

		if (p->type == PHOENIX6_TYPE_CONTROL) {
			shell_print(sh, "  %s: %s (set as %s.<parameter>)", p->name,
				    phoenix6_control_desc_get(p->control)->name, p->name);
		} else if (p->required) {
			shell_print(sh, "  %s: %s, required", p->name,
				    (p->type == PHOENIX6_TYPE_ENUM)
					    ? phoenix6_enum_descs[p->enum_index].name
					    : type_names[p->type]);
		} else if (p->type == PHOENIX6_TYPE_ENUM) {
			shell_print(sh, "  %s: %s, default %s", p->name,
				    phoenix6_enum_descs[p->enum_index].name,
				    phoenix6_enum_name(p->enum_index, (int32_t)p->default_value));
		} else if (p->type == PHOENIX6_TYPE_BOOL) {
			shell_print(sh, "  %s: bool, default %s", p->name,
				    p->default_value != 0.0 ? "true" : "false");
		} else {
			shell_print(sh, "  %s: %s, default %g", p->name, type_names[p->type],
				    p->default_value);
		}
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	p6_config_cmds, SHELL_CMD_ARG(get, NULL, "<device> <group.field>", cmd_config_get, 3, 0),
	SHELL_CMD_ARG(set, NULL, "<device> <group.field> <value>", cmd_config_set, 4, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	p6_cmds, SHELL_CMD(status, NULL, "Bridge connection", cmd_status),
	SHELL_CMD_ARG(enable, NULL, "on|off: feed the actuator enable", cmd_enable, 2, 0),
	SHELL_CMD_ARG(control, NULL,
		      "<device> <Request> [param=value ...]\n"
		      "e.g. p6 control 1 VelocityVoltage velocity=10 slot=1",
		      cmd_control, 3, 40),
	SHELL_CMD_ARG(stop, NULL, "<device>: NeutralOut", cmd_stop, 2, 0),
	SHELL_CMD(config, &p6_config_cmds, "Configuration fields", NULL),
	SHELL_CMD_ARG(factory_default, NULL, "<device>", cmd_factory_default, 2, 0),
	SHELL_CMD_ARG(setpos, NULL, "<device> <rotations>", cmd_setpos, 3, 0),
	SHELL_CMD_ARG(clear_faults, NULL, "<device>: clear sticky faults", cmd_clear_faults, 2, 0),
	SHELL_CMD_ARG(signal, NULL, "<device> <signal> [Hz, 0 stops]", cmd_signal, 3, 1),
	SHELL_CMD_ARG(list, NULL, "controls|configs|signals", cmd_list, 2, 0),
	SHELL_CMD_ARG(info, NULL, "<Request>: parameters and defaults", cmd_info, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(p6, &p6_cmds, "Phoenix 6 requests through the host bridge", NULL);
