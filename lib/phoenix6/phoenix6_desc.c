/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lookup helpers for the generated Phoenix 6 descriptor tables.
 */

#include <errno.h>
#include <string.h>

#include <canbus_rp2040/phoenix6.h>

const struct phoenix6_control_desc *phoenix6_control_desc_get(enum phoenix6_control_id id)
{
	if (id <= PHOENIX6_CONTROL_ID_NONE || id >= PHOENIX6_CONTROL_ID_COUNT) {
		return NULL;
	}

	return &phoenix6_control_descs[id - 1];
}

const struct phoenix6_control_desc *phoenix6_control_desc_find(const char *name)
{
	for (int i = 0; i < PHOENIX6_CONTROL_ID_COUNT - 1; i++) {
		if (strcmp(phoenix6_control_descs[i].name, name) == 0) {
			return &phoenix6_control_descs[i];
		}
	}

	return NULL;
}

const struct phoenix6_config_desc *phoenix6_config_desc_find(const char *name)
{
	const char *dot = strchr(name, '.');

	if (dot == NULL) {
		return NULL;
	}

	for (int i = 0; i < PHOENIX6_CONFIG_COUNT - 1; i++) {
		const struct phoenix6_config_desc *d = &phoenix6_config_descs[i];
		size_t glen = strlen(d->group);

		if ((size_t)(dot - name) == glen && strncmp(name, d->group, glen) == 0 &&
		    strcmp(dot + 1, d->field) == 0) {
			return d;
		}
	}

	return NULL;
}

const struct phoenix6_signal_desc *phoenix6_signal_desc_find(const char *name)
{
	for (int i = 0; i < PHOENIX6_SIGNAL_COUNT - 1; i++) {
		if (strcmp(phoenix6_signal_descs[i].name, name) == 0) {
			return &phoenix6_signal_descs[i];
		}
	}

	return NULL;
}

int phoenix6_enum_value(uint8_t enum_index, const char *name, int32_t *value)
{
	const struct phoenix6_enum_desc *e;

	if (enum_index >= phoenix6_enum_count) {
		return -ENOENT;
	}

	e = &phoenix6_enum_descs[enum_index];
	for (uint8_t i = 0; i < e->count; i++) {
		if (strcmp(e->members[i].name, name) == 0) {
			*value = e->members[i].value;
			return 0;
		}
	}

	return -ENOENT;
}

const char *phoenix6_enum_name(uint8_t enum_index, int32_t value)
{
	const struct phoenix6_enum_desc *e;

	if (enum_index >= phoenix6_enum_count) {
		return NULL;
	}

	e = &phoenix6_enum_descs[enum_index];
	for (uint8_t i = 0; i < e->count; i++) {
		if (e->members[i].value == value) {
			return e->members[i].name;
		}
	}

	return NULL;
}

static int init_params(const struct phoenix6_control_desc *desc, uint8_t *params)
{
	memset(params, 0, desc->size);

	for (uint8_t i = 0; i < desc->param_count; i++) {
		const struct phoenix6_param_desc *p = &desc->params[i];
		uint8_t *field = params + p->offset;

		switch (p->type) {
		case PHOENIX6_TYPE_FLOAT: {
			double v = p->default_value;

			memcpy(field, &v, sizeof(v));
			break;
		}
		case PHOENIX6_TYPE_BOOL: {
			bool v = p->default_value != 0.0;

			memcpy(field, &v, sizeof(v));
			break;
		}
		case PHOENIX6_TYPE_INT:
		case PHOENIX6_TYPE_ENUM: {
			int32_t v = (int32_t)p->default_value;

			memcpy(field, &v, sizeof(v));
			break;
		}
		case PHOENIX6_TYPE_CONTROL: {
			const struct phoenix6_control_desc *sub =
				phoenix6_control_desc_get(p->control);

			if (sub == NULL || init_params(sub, field) != 0) {
				return -EINVAL;
			}
			break;
		}
		default:
			return -EINVAL;
		}
	}

	return 0;
}

int phoenix6_control_init(struct phoenix6_control *request, enum phoenix6_control_id id)
{
	const struct phoenix6_control_desc *desc = phoenix6_control_desc_get(id);

	if (desc == NULL) {
		return -EINVAL;
	}

	memset(request, 0, sizeof(*request));
	request->id = id;
	request->update_freq_hz = desc->update_freq_hz;

	return init_params(desc, (uint8_t *)request + PHOENIX6_CONTROL_PARAMS_OFFSET);
}
