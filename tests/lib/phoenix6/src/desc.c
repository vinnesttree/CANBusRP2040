/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generated Phoenix 6 descriptor tables, initializers and lookup helpers.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include <canbus_rp2040/phoenix6.h>

#include "phoenix6_proto.h"

#define CONTROL_COUNT (PHOENIX6_CONTROL_ID_COUNT - 1)
#define CONFIG_COUNT  (PHOENIX6_CONFIG_COUNT - 1)
#define SIGNAL_COUNT  (PHOENIX6_SIGNAL_COUNT - 1)

static size_t type_size(uint8_t type)
{
	switch (type) {
	case PHOENIX6_TYPE_FLOAT:
		return sizeof(double);
	case PHOENIX6_TYPE_BOOL:
		return sizeof(bool);
	case PHOENIX6_TYPE_INT:
	case PHOENIX6_TYPE_ENUM:
		return sizeof(int32_t);
	default:
		return 0;
	}
}

/* Encode the parameters of a request as sent on the wire */
static size_t encode(const struct phoenix6_control *req, uint8_t *buf, size_t size)
{
	struct phoenix6_writer w;

	phoenix6_writer_init(&w, buf, size);
	zassert_ok(phoenix6_encode_control_params(&w, phoenix6_control_desc_get(req->id),
						  (const uint8_t *)req +
							  PHOENIX6_CONTROL_PARAMS_OFFSET));

	return w.len;
}

ZTEST(phoenix6_desc, test_control_table)
{
	zassert_equal(CONTROL_COUNT, 99, "Phoenix 6 %s has 99 Talon FX control requests",
		      PHOENIX6_GENERATED_VERSION);

	for (int i = 0; i < CONTROL_COUNT; i++) {
		const struct phoenix6_control_desc *d = &phoenix6_control_descs[i];

		zassert_equal(d->id, i + 1);
		zassert_equal_ptr(phoenix6_control_desc_get(d->id), d);
		zassert_equal_ptr(phoenix6_control_desc_find(d->name), d, "%s", d->name);
		zassert_true(d->size > 0 && d->size <= sizeof(struct phoenix6_control) -
							       PHOENIX6_CONTROL_PARAMS_OFFSET);
		/* EmptyControl is the only request without an update frequency */
		zassert_true(d->update_freq_hz > 0 || strcmp(d->name, "EmptyControl") == 0, "%s",
			     d->name);

		for (uint8_t j = 0; j < d->param_count; j++) {
			const struct phoenix6_param_desc *p = &d->params[j];

			if (p->type == PHOENIX6_TYPE_CONTROL) {
				const struct phoenix6_control_desc *sub =
					phoenix6_control_desc_get(p->control);

				zassert_not_null(sub, "%s.%s", d->name, p->name);
				zassert_true(p->offset + sub->size <= d->size);
				zassert_true(p->required);
			} else {
				zassert_true(type_size(p->type) > 0, "%s.%s", d->name, p->name);
				zassert_true(p->offset + type_size(p->type) <= d->size);
				zassert_equal(p->type == PHOENIX6_TYPE_ENUM, p->enum_index != 0xFF,
					      "%s.%s", d->name, p->name);
			}

			if (j > 0) {
				zassert_true(p->offset > d->params[j - 1].offset,
					     "%s: parameters out of order", d->name);
			}
		}
	}

	zassert_is_null(phoenix6_control_desc_get(PHOENIX6_CONTROL_ID_NONE));
	zassert_is_null(phoenix6_control_desc_get(PHOENIX6_CONTROL_ID_COUNT));
	zassert_is_null(phoenix6_control_desc_find("VelocityVoltage2"));
}

ZTEST(phoenix6_desc, test_every_request_fits_a_packet)
{
	struct phoenix6_control req;
	uint8_t buf[PHOENIX6_MAX_PACKET];

	for (int id = 1; id <= CONTROL_COUNT; id++) {
		/* SET_CONTROL: type, seq, device, id, update_freq, params, CRC */
		size_t overhead = 3 + 1 + 2 + 8 + 2;

		zassert_ok(phoenix6_control_init(&req, id));
		zassert_true(encode(&req, buf, sizeof(buf)) + overhead <= PHOENIX6_MAX_PACKET);
	}
}

ZTEST(phoenix6_desc, test_flags)
{
	int pro = 0;
	int canivore = 0;

	for (int i = 0; i < CONTROL_COUNT; i++) {
		pro += (phoenix6_control_descs[i].flags & PHOENIX6_CONTROL_FLAG_PRO) ? 1 : 0;
		canivore +=
			(phoenix6_control_descs[i].flags & PHOENIX6_CONTROL_FLAG_CANIVORE) ? 1 : 0;
	}

	zassert_equal(pro, 66);
	zassert_equal(canivore, 60);

	zassert_equal(phoenix6_control_desc_find("DutyCycleOut")->flags, 0);
	zassert_equal(phoenix6_control_desc_find("VelocityTorqueCurrentFOC")->flags,
		      PHOENIX6_CONTROL_FLAG_PRO);
	zassert_equal(phoenix6_control_desc_find("Diff_VelocityVoltage_Position")->flags,
		      PHOENIX6_CONTROL_FLAG_PRO | PHOENIX6_CONTROL_FLAG_CANIVORE);
}

ZTEST(phoenix6_desc, test_velocity_voltage)
{
	const struct phoenix6_control_desc *d = phoenix6_control_desc_find("VelocityVoltage");
	static const char *const names[] = {
		"velocity",
		"acceleration",
		"enable_foc",
		"feed_forward",
		"slot",
		"override_brake_dur_neutral",
		"limit_forward_motion",
		"limit_reverse_motion",
		"ignore_hardware_limits",
		"ignore_software_limits",
		"use_timesync",
	};

	zassert_not_null(d);
	zassert_equal(d->id, PHOENIX6_CONTROL_ID_VELOCITY_VOLTAGE);
	zassert_equal(d->size, sizeof(struct phoenix6_velocity_voltage));
	zassert_equal(d->update_freq_hz, 100);
	zassert_equal(d->param_count, ARRAY_SIZE(names));

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		zassert_str_equal(d->params[i].name, names[i]);
	}

	zassert_true(d->params[0].required);
	zassert_equal(d->params[0].offset, offsetof(struct phoenix6_velocity_voltage, velocity));
	zassert_false(d->params[2].required);
	zassert_equal(d->params[2].type, PHOENIX6_TYPE_BOOL);
	zassert_equal(d->params[2].default_value, 1.0);
	zassert_equal(d->params[4].type, PHOENIX6_TYPE_INT);
	zassert_equal(d->params[4].offset, offsetof(struct phoenix6_velocity_voltage, slot));
}

ZTEST(phoenix6_desc, test_initializers_match_control_init)
{
	struct {
		struct phoenix6_control macro;
		enum phoenix6_control_id id;
	} cases[] = {
		{PHOENIX6_DUTY_CYCLE_OUT(0.0), PHOENIX6_CONTROL_ID_DUTY_CYCLE_OUT},
		{PHOENIX6_VOLTAGE_OUT(0.0), PHOENIX6_CONTROL_ID_VOLTAGE_OUT},
		{PHOENIX6_VELOCITY_VOLTAGE(0.0), PHOENIX6_CONTROL_ID_VELOCITY_VOLTAGE},
		{PHOENIX6_POSITION_VOLTAGE(0.0), PHOENIX6_CONTROL_ID_POSITION_VOLTAGE},
		{PHOENIX6_MOTION_MAGIC_VOLTAGE(0.0), PHOENIX6_CONTROL_ID_MOTION_MAGIC_VOLTAGE},
		{PHOENIX6_MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC(0.0),
		 PHOENIX6_CONTROL_ID_MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC},
		{PHOENIX6_TORQUE_CURRENT_FOC(0.0), PHOENIX6_CONTROL_ID_TORQUE_CURRENT_FOC},
		{PHOENIX6_FOLLOWER(0, 0), PHOENIX6_CONTROL_ID_FOLLOWER},
		{PHOENIX6_NEUTRAL_OUT(), PHOENIX6_CONTROL_ID_NEUTRAL_OUT},
		{PHOENIX6_COAST_OUT(), PHOENIX6_CONTROL_ID_COAST_OUT},
		{PHOENIX6_STATIC_BRAKE(), PHOENIX6_CONTROL_ID_STATIC_BRAKE},
		{PHOENIX6_MUSIC_TONE(0.0), PHOENIX6_CONTROL_ID_MUSIC_TONE},
		{PHOENIX6_DIFF_VELOCITY_VOLTAGE_POSITION(PHOENIX6_VELOCITY_VOLTAGE_PARAMS(0.0),
							 PHOENIX6_POSITION_VOLTAGE_PARAMS(0.0)),
		 PHOENIX6_CONTROL_ID_DIFF_VELOCITY_VOLTAGE_POSITION},
	};
	struct phoenix6_control init;
	uint8_t a[PHOENIX6_MAX_PACKET];
	uint8_t b[PHOENIX6_MAX_PACKET];

	ARRAY_FOR_EACH(cases, i) {
		size_t la;
		size_t lb;

		zassert_equal(cases[i].macro.id, cases[i].id);
		zassert_ok(phoenix6_control_init(&init, cases[i].id));
		zassert_equal(cases[i].macro.update_freq_hz, init.update_freq_hz);

		la = encode(&cases[i].macro, a, sizeof(a));
		lb = encode(&init, b, sizeof(b));
		zassert_equal(la, lb);
		zassert_mem_equal(a, b, la, "%s", phoenix6_control_desc_get(cases[i].id)->name);
	}

	zassert_equal(phoenix6_control_init(&init, PHOENIX6_CONTROL_ID_NONE), -EINVAL);
	zassert_equal(phoenix6_control_init(&init, PHOENIX6_CONTROL_ID_COUNT), -EINVAL);
}

ZTEST(phoenix6_desc, test_control_init_defaults)
{
	struct phoenix6_control req;

	zassert_ok(phoenix6_control_init(&req, PHOENIX6_CONTROL_ID_MOTION_MAGIC_VELOCITY_VOLTAGE));
	zassert_equal(req.id, PHOENIX6_CONTROL_ID_MOTION_MAGIC_VELOCITY_VOLTAGE);
	zassert_equal(req.update_freq_hz, 100.0);
	zassert_true(req.motion_magic_velocity_voltage.enable_foc);
	zassert_equal(req.motion_magic_velocity_voltage.slot, 0);

	/* Nested requests get their own defaults */
	zassert_ok(phoenix6_control_init(&req, PHOENIX6_CONTROL_ID_DIFF_VELOCITY_VOLTAGE_POSITION));
	zassert_true(req.diff_velocity_voltage_position.average_request.enable_foc);
	zassert_true(req.diff_velocity_voltage_position.differential_request.enable_foc);
}

ZTEST(phoenix6_desc, test_config_table)
{
	const struct phoenix6_config_desc *d;

	zassert_equal(CONFIG_COUNT, 100);

	for (int i = 0; i < CONFIG_COUNT; i++) {
		char name[96];

		d = &phoenix6_config_descs[i];
		zassert_equal(d->id, i + 1);
		snprintf(name, sizeof(name), "%s.%s", d->group, d->field);
		zassert_equal_ptr(phoenix6_config_desc_find(name), d, "%s", name);
		zassert_equal(d->type == PHOENIX6_TYPE_ENUM, d->enum_index != 0xFF, "%s", name);
	}

	d = phoenix6_config_desc_find("slot0.k_p");
	zassert_not_null(d);
	zassert_equal(d->id, PHOENIX6_CONFIG_SLOT0_K_P);
	zassert_equal(d->type, PHOENIX6_TYPE_FLOAT);

	d = phoenix6_config_desc_find("motor_output.neutral_mode");
	zassert_not_null(d);
	zassert_equal(d->id, PHOENIX6_CONFIG_MOTOR_OUTPUT_NEUTRAL_MODE);
	zassert_equal(d->type, PHOENIX6_TYPE_ENUM);
	zassert_str_equal(phoenix6_enum_descs[d->enum_index].name, "NeutralModeValue");

	zassert_is_null(phoenix6_config_desc_find("slot0"));
	zassert_is_null(phoenix6_config_desc_find("slot0.k_q"));
	zassert_is_null(phoenix6_config_desc_find("slot.k_p"));
	zassert_is_null(phoenix6_config_desc_find(".k_p"));
}

ZTEST(phoenix6_desc, test_signal_table)
{
	const struct phoenix6_signal_desc *d;

	zassert_equal(SIGNAL_COUNT, 112);

	for (int i = 0; i < SIGNAL_COUNT; i++) {
		d = &phoenix6_signal_descs[i];
		zassert_equal(d->id, i + 1);
		zassert_equal_ptr(phoenix6_signal_desc_find(d->name), d, "%s", d->name);
		zassert_equal(d->type == PHOENIX6_TYPE_ENUM, d->enum_index != 0xFF, "%s", d->name);
	}

	d = phoenix6_signal_desc_find("velocity");
	zassert_not_null(d);
	zassert_equal(d->id, PHOENIX6_SIGNAL_VELOCITY);
	zassert_str_equal(d->unit, "rotations_per_second");

	d = phoenix6_signal_desc_find("control_mode");
	zassert_not_null(d);
	zassert_equal(d->type, PHOENIX6_TYPE_ENUM);
	zassert_str_equal(phoenix6_enum_descs[d->enum_index].name, "ControlModeValue");

	zassert_is_null(phoenix6_signal_desc_find("get_velocity"));
}

ZTEST(phoenix6_desc, test_enums)
{
	const struct phoenix6_config_desc *d =
		phoenix6_config_desc_find("motor_output.neutral_mode");
	int32_t value;

	zassert_ok(phoenix6_enum_value(d->enum_index, "BRAKE", &value));
	zassert_equal(value, PHOENIX6_NEUTRAL_MODE_BRAKE);
	zassert_ok(phoenix6_enum_value(d->enum_index, "COAST", &value));
	zassert_equal(value, PHOENIX6_NEUTRAL_MODE_COAST);
	zassert_equal(phoenix6_enum_value(d->enum_index, "Brake", &value), -ENOENT);
	zassert_equal(phoenix6_enum_value(0xFF, "BRAKE", &value), -ENOENT);

	zassert_str_equal(phoenix6_enum_name(d->enum_index, 1), "BRAKE");
	zassert_is_null(phoenix6_enum_name(d->enum_index, 7));
	zassert_is_null(phoenix6_enum_name(0xFF, 0));

	/* Enum values are Phoenix 6 values, not indices */
	zassert_equal(PHOENIX6_REVERSE_LIMIT_SOURCE_DISABLED, 3);
	zassert_equal(PHOENIX6_REVERSE_LIMIT_SOURCE_REMOTE_CANCODER, 4);
}

ZTEST_SUITE(phoenix6_desc, NULL, NULL, NULL, NULL, NULL);
