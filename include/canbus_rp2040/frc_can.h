/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief FIRST (FRC/FTC) CAN identifier helpers.
 *
 * Implements the publicly documented parts of the "FIRST CAN Device
 * Specifications" (WPILib documentation, docs/software/can-devices/
 * can-addressing): the 29-bit identifier layout, device type and manufacturer
 * codes, the broadcast Disable message and the universal heartbeat.
 *
 * Identifier layout (extended frames):
 *
 * @code{.text}
 *   28     24 23          16 15       10 9      6 5          0
 *  +---------+--------------+-----------+--------+------------+
 *  | device  | manufacturer | API class | API    | device     |
 *  | type    |              |           | index  | number     |
 *  +---------+--------------+-----------+--------+------------+
 * @endcode
 */

#ifndef CANBUS_RP2040_INCLUDE_FRC_CAN_H_
#define CANBUS_RP2040_INCLUDE_FRC_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name Identifier field positions and masks
 * @{
 */
#define FRC_CAN_DEVICE_TYPE_SHIFT  24
#define FRC_CAN_DEVICE_TYPE_MASK   0x1FU
#define FRC_CAN_MANUFACTURER_SHIFT 16
#define FRC_CAN_MANUFACTURER_MASK  0xFFU
#define FRC_CAN_API_ID_SHIFT       6
#define FRC_CAN_API_ID_MASK        0x3FFU
#define FRC_CAN_API_CLASS_SHIFT    4
#define FRC_CAN_API_CLASS_MASK     0x3FU
#define FRC_CAN_API_INDEX_MASK     0x0FU
#define FRC_CAN_DEVICE_NUMBER_MASK 0x3FU
/** @} */

/** Device number reserved for device specific broadcast messages. */
#define FRC_CAN_DEVICE_NUMBER_BROADCAST 0x3FU

/** Device types assigned by FIRST. */
enum frc_can_device_type {
	FRC_CAN_DEV_BROADCAST = 0,
	FRC_CAN_DEV_ROBOT_CONTROLLER = 1,
	FRC_CAN_DEV_MOTOR_CONTROLLER = 2,
	FRC_CAN_DEV_RELAY_CONTROLLER = 3,
	FRC_CAN_DEV_GYRO_SENSOR = 4,
	FRC_CAN_DEV_ACCELEROMETER = 5,
	FRC_CAN_DEV_DISTANCE_SENSOR = 6,
	FRC_CAN_DEV_ENCODER = 7,
	FRC_CAN_DEV_POWER_DISTRIBUTION = 8,
	FRC_CAN_DEV_PNEUMATICS_CONTROLLER = 9,
	FRC_CAN_DEV_MISCELLANEOUS = 10,
	FRC_CAN_DEV_IO_BREAKOUT = 11,
	FRC_CAN_DEV_SERVO_CONTROLLER = 12,
	FRC_CAN_DEV_COLOR_SENSOR = 13,
	FRC_CAN_DEV_FIRMWARE_UPDATE = 31,
};

/**
 * Manufacturer codes assigned by FIRST (current specification; codes 2 and 3
 * were listed as Luminary Micro and DEKA in older revisions).
 */
enum frc_can_manufacturer {
	FRC_CAN_MFR_BROADCAST = 0,
	FRC_CAN_MFR_NI = 1,
	FRC_CAN_MFR_LIMELIGHT = 2,
	FRC_CAN_MFR_FIRST = 3,
	FRC_CAN_MFR_CTR_ELECTRONICS = 4,
	FRC_CAN_MFR_REV_ROBOTICS = 5,
	FRC_CAN_MFR_GRAPPLE = 6,
	FRC_CAN_MFR_MINDSENSORS = 7,
	FRC_CAN_MFR_TEAM_USE = 8,
	FRC_CAN_MFR_KAUAI_LABS = 9,
	FRC_CAN_MFR_COPPERFORGE = 10,
	FRC_CAN_MFR_PLAYING_WITH_FUSION = 11,
	FRC_CAN_MFR_STUDICA = 12,
	FRC_CAN_MFR_THE_THRIFTY_BOT = 13,
	FRC_CAN_MFR_REDUX_ROBOTICS = 14,
	FRC_CAN_MFR_ANDYMARK = 15,
	FRC_CAN_MFR_VIVID_HOSTING = 16,
	FRC_CAN_MFR_VERTOS_ROBOTICS = 17,
	FRC_CAN_MFR_SWYFT_ROBOTICS = 18,
	FRC_CAN_MFR_LUMYN_LABS = 19,
	FRC_CAN_MFR_BRUSHLAND_LABS = 20,
};

/** Decoded FIRST CAN identifier. */
struct frc_can_id {
	/** Device type (5 bits), see @ref frc_can_device_type. */
	uint8_t device_type;
	/** Manufacturer (8 bits), see @ref frc_can_manufacturer. */
	uint8_t manufacturer;
	/** API identifier (10 bits): API class (6 bits) and API index (4 bits). */
	uint16_t api_id;
	/** Device number (6 bits). */
	uint8_t device_number;
};

/** @brief Combine an API class and API index into a 10-bit API identifier. */
static inline uint16_t frc_can_api_id(uint8_t api_class, uint8_t api_index)
{
	return (uint16_t)(((api_class & FRC_CAN_API_CLASS_MASK) << FRC_CAN_API_CLASS_SHIFT) |
			  (api_index & FRC_CAN_API_INDEX_MASK));
}

/** @brief API class of a 10-bit API identifier. */
static inline uint8_t frc_can_api_class(uint16_t api_id)
{
	return (uint8_t)((api_id >> FRC_CAN_API_CLASS_SHIFT) & FRC_CAN_API_CLASS_MASK);
}

/** @brief API index of a 10-bit API identifier. */
static inline uint8_t frc_can_api_index(uint16_t api_id)
{
	return (uint8_t)(api_id & FRC_CAN_API_INDEX_MASK);
}

/** @brief Build a 29-bit CAN identifier from its fields. */
static inline uint32_t frc_can_id_pack(const struct frc_can_id *id)
{
	return ((uint32_t)(id->device_type & FRC_CAN_DEVICE_TYPE_MASK)
		<< FRC_CAN_DEVICE_TYPE_SHIFT) |
	       ((uint32_t)(id->manufacturer & FRC_CAN_MANUFACTURER_MASK)
		<< FRC_CAN_MANUFACTURER_SHIFT) |
	       ((uint32_t)(id->api_id & FRC_CAN_API_ID_MASK) << FRC_CAN_API_ID_SHIFT) |
	       (uint32_t)(id->device_number & FRC_CAN_DEVICE_NUMBER_MASK);
}

/** @brief Split a 29-bit CAN identifier into its fields. */
static inline void frc_can_id_unpack(uint32_t can_id, struct frc_can_id *id)
{
	id->device_type =
		(uint8_t)((can_id >> FRC_CAN_DEVICE_TYPE_SHIFT) & FRC_CAN_DEVICE_TYPE_MASK);
	id->manufacturer =
		(uint8_t)((can_id >> FRC_CAN_MANUFACTURER_SHIFT) & FRC_CAN_MANUFACTURER_MASK);
	id->api_id = (uint16_t)((can_id >> FRC_CAN_API_ID_SHIFT) & FRC_CAN_API_ID_MASK);
	id->device_number = (uint8_t)(can_id & FRC_CAN_DEVICE_NUMBER_MASK);
}

/**
 * @brief Initialise a filter matching extended frames of one device type and
 * manufacturer, any API and any device number.
 */
static inline void frc_can_filter_init(struct can_filter *filter, uint8_t device_type,
				       uint8_t manufacturer)
{
	struct frc_can_id id = {.device_type = device_type, .manufacturer = manufacturer};

	filter->id = frc_can_id_pack(&id);
	filter->mask = ((uint32_t)FRC_CAN_DEVICE_TYPE_MASK << FRC_CAN_DEVICE_TYPE_SHIFT) |
		       ((uint32_t)FRC_CAN_MANUFACTURER_MASK << FRC_CAN_MANUFACTURER_SHIFT);
	filter->flags = CAN_FILTER_IDE;
}

/**
 * @brief Broadcast Disable message: arbitration ID 0, no data. Devices must
 * disable immediately when they receive it.
 */
static inline void frc_can_disable_frame(struct can_frame *frame)
{
	*frame = (struct can_frame){.id = 0U, .flags = CAN_FRAME_IDE, .dlc = 0U};
}

/** CAN ID of the universal heartbeat sent by the robot controller. */
#define FRC_CAN_HEARTBEAT_ID         0x01011840U
/** Heartbeat transmission period. */
#define FRC_CAN_HEARTBEAT_PERIOD_MS  20
/** Devices act as if the robot were disabled after this time without heartbeat. */
#define FRC_CAN_HEARTBEAT_TIMEOUT_MS 100

/**
 * @brief Universal heartbeat contents.
 *
 * The 8 data bytes hold a 64-bit bit field. Per the byte/bit table of the
 * specification the match time occupies the last data byte and the hour the
 * first one, i.e. the payload is the big-endian representation of a 64-bit
 * value whose least significant bits are the match time.
 *
 * This follows the current specification revision. Earlier revisions
 * (roboRIO) define the replay number as 6 bits wide, in which case replay
 * numbers above 31 set @a ftc_motor_override.
 */
struct frc_can_heartbeat {
	/** Match time in seconds (8 bits). */
	uint8_t match_time_s;
	/** Match number (10 bits). */
	uint16_t match_number;
	/** Replay number (5 bits). */
	uint8_t replay_number;
	/** FTC motor override: FTC motors enabled without a driver station. */
	bool ftc_motor_override;
	/** Red alliance. */
	bool red_alliance;
	/** Robot enabled. */
	bool enabled;
	/** Autonomous mode. */
	bool autonomous;
	/** Test mode. */
	bool test_mode;
	/** System watchdog: motor controllers are enabled when set. */
	bool system_watchdog;
	/** Tournament type (3 bits). */
	uint8_t tournament_type;
	/** Time of day: year (6 bits, as transmitted). */
	uint8_t tod_year;
	/** Time of day: month (4 bits). */
	uint8_t tod_month;
	/** Time of day: day (5 bits). */
	uint8_t tod_day;
	/** Time of day: seconds (6 bits). */
	uint8_t tod_sec;
	/** Time of day: minutes (6 bits). */
	uint8_t tod_min;
	/** Time of day: hours (5 bits). */
	uint8_t tod_hour;
};

/** @brief Encode a heartbeat into 8 data bytes. */
void frc_can_heartbeat_encode(const struct frc_can_heartbeat *hb, uint8_t data[8]);

/** @brief Decode 8 heartbeat data bytes. */
void frc_can_heartbeat_decode(const uint8_t data[8], struct frc_can_heartbeat *hb);

/** @brief Name of a manufacturer code, or NULL if unknown. */
const char *frc_can_manufacturer_name(uint8_t manufacturer);

/** @brief Name of a device type, or NULL if unknown. */
const char *frc_can_device_type_name(uint8_t device_type);

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_FRC_CAN_H_ */
