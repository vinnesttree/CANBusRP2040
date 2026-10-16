/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Passive CAN bus monitor for CTR Electronics devices (Kraken / Talon FX and
 * other CTRE devices) and the FIRST universal heartbeat. Only the publicly
 * documented FIRST CAN identifier fields are interpreted; frame payloads of
 * CTRE devices are not decoded.
 */

#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <canbus_rp2040/frc_can.h>
#include <canbus_rp2040/kraken.h>

static struct {
	struct k_spinlock lock;
	const struct device *can;
	int ctre_filter;
	int heartbeat_filter;
	size_t count;
	struct kraken_can_device devices[CONFIG_KRAKEN_CAN_MONITOR_MAX_DEVICES];
	bool heartbeat_valid;
	int64_t heartbeat_time;
	uint8_t heartbeat_data[8];
} mon = {
	.ctre_filter = -1,
	.heartbeat_filter = -1,
};

static struct kraken_can_device *find_or_add(uint8_t device_type, uint8_t device_number)
{
	for (size_t i = 0; i < mon.count; i++) {
		if (mon.devices[i].device_type == device_type &&
		    mon.devices[i].device_number == device_number) {
			return &mon.devices[i];
		}
	}

	if (mon.count == ARRAY_SIZE(mon.devices)) {
		return NULL;
	}

	struct kraken_can_device *dev = &mon.devices[mon.count++];

	memset(dev, 0, sizeof(*dev));
	dev->device_type = device_type;
	dev->device_number = device_number;

	return dev;
}

static void ctre_rx(const struct device *can, struct can_frame *frame, void *user_data)
{
	struct kraken_can_device *dev;
	struct frc_can_id id;
	int64_t now = k_uptime_get();
	k_spinlock_key_t key;

	ARG_UNUSED(can);
	ARG_UNUSED(user_data);

	frc_can_id_unpack(frame->id, &id);

	key = k_spin_lock(&mon.lock);

	dev = find_or_add(id.device_type, id.device_number);
	if (dev != NULL) {
		bool known = false;

		dev->frames++;
		dev->last_seen_ms = now;

		for (uint8_t i = 0; i < dev->api_id_count; i++) {
			if (dev->api_ids[i] == id.api_id) {
				known = true;
				break;
			}
		}

		if (!known && dev->api_id_count < ARRAY_SIZE(dev->api_ids)) {
			dev->api_ids[dev->api_id_count++] = id.api_id;
		}
	}

	k_spin_unlock(&mon.lock, key);
}

static void heartbeat_rx(const struct device *can, struct can_frame *frame, void *user_data)
{
	k_spinlock_key_t key;

	ARG_UNUSED(can);
	ARG_UNUSED(user_data);

	if (frame->dlc != 8U || (frame->flags & CAN_FRAME_RTR) != 0U) {
		return;
	}

	key = k_spin_lock(&mon.lock);
	memcpy(mon.heartbeat_data, frame->data, sizeof(mon.heartbeat_data));
	mon.heartbeat_time = k_uptime_get();
	mon.heartbeat_valid = true;
	k_spin_unlock(&mon.lock, key);
}

int kraken_can_monitor_start(const struct device *can)
{
	const struct can_filter heartbeat = {
		.id = FRC_CAN_HEARTBEAT_ID,
		.mask = CAN_EXT_ID_MASK,
		.flags = CAN_FILTER_IDE,
	};
	/* Any device type, manufacturer CTR Electronics */
	const struct can_filter ctre = {
		.id = (uint32_t)FRC_CAN_MFR_CTR_ELECTRONICS << FRC_CAN_MANUFACTURER_SHIFT,
		.mask = (uint32_t)FRC_CAN_MANUFACTURER_MASK << FRC_CAN_MANUFACTURER_SHIFT,
		.flags = CAN_FILTER_IDE,
	};
	int id;

	if (mon.can != NULL) {
		return -EALREADY;
	}

	id = can_add_rx_filter(can, ctre_rx, NULL, &ctre);
	if (id < 0) {
		return id;
	}
	mon.ctre_filter = id;

	id = can_add_rx_filter(can, heartbeat_rx, NULL, &heartbeat);
	if (id < 0) {
		can_remove_rx_filter(can, mon.ctre_filter);
		mon.ctre_filter = -1;
		return id;
	}
	mon.heartbeat_filter = id;
	mon.can = can;

	return 0;
}

int kraken_can_monitor_stop(void)
{
	if (mon.can == NULL) {
		return -EALREADY;
	}

	can_remove_rx_filter(mon.can, mon.ctre_filter);
	can_remove_rx_filter(mon.can, mon.heartbeat_filter);
	mon.ctre_filter = -1;
	mon.heartbeat_filter = -1;
	mon.can = NULL;

	return 0;
}

void kraken_can_monitor_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&mon.lock);

	mon.count = 0;
	mon.heartbeat_valid = false;
	k_spin_unlock(&mon.lock, key);
}

size_t kraken_can_monitor_devices(struct kraken_can_device *devices, size_t max,
				  uint32_t max_age_ms)
{
	int64_t now = k_uptime_get();
	k_spinlock_key_t key;
	size_t n = 0;

	key = k_spin_lock(&mon.lock);

	for (size_t i = 0; i < mon.count && n < max; i++) {
		if (max_age_ms == 0U || (now - mon.devices[i].last_seen_ms) <= max_age_ms) {
			devices[n++] = mon.devices[i];
		}
	}

	k_spin_unlock(&mon.lock, key);

	return n;
}

int kraken_can_monitor_heartbeat(struct frc_can_heartbeat *hb, int64_t *age_ms)
{
	uint8_t data[8];
	int64_t time;
	bool valid;
	k_spinlock_key_t key;

	key = k_spin_lock(&mon.lock);
	valid = mon.heartbeat_valid;
	time = mon.heartbeat_time;
	memcpy(data, mon.heartbeat_data, sizeof(data));
	k_spin_unlock(&mon.lock, key);

	if (!valid) {
		return -ENODATA;
	}

	frc_can_heartbeat_decode(data, hb);
	if (age_ms != NULL) {
		*age_ms = k_uptime_get() - time;
	}

	return 0;
}
