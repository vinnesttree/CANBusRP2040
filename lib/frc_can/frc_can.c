/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <canbus_rp2040/frc_can.h>

/* Bit positions of the heartbeat fields (LSB first, see frc_can.h) */
#define HB_MATCH_TIME_POS   0
#define HB_MATCH_TIME_LEN   8
#define HB_MATCH_NUMBER_POS 8
#define HB_MATCH_NUMBER_LEN 10
#define HB_REPLAY_POS       18
#define HB_REPLAY_LEN       5
#define HB_FTC_OVERRIDE_POS 23
#define HB_RED_POS          24
#define HB_ENABLED_POS      25
#define HB_AUTONOMOUS_POS   26
#define HB_TEST_POS         27
#define HB_WATCHDOG_POS     28
#define HB_TOURNAMENT_POS   29
#define HB_TOURNAMENT_LEN   3
#define HB_YEAR_POS         32
#define HB_YEAR_LEN         6
#define HB_MONTH_POS        38
#define HB_MONTH_LEN        4
#define HB_DAY_POS          42
#define HB_DAY_LEN          5
#define HB_SEC_POS          47
#define HB_SEC_LEN          6
#define HB_MIN_POS          53
#define HB_MIN_LEN          6
#define HB_HOUR_POS         59
#define HB_HOUR_LEN         5

static inline uint64_t field_put(uint64_t value, unsigned int pos, unsigned int len)
{
	return (value & BIT64_MASK(len)) << pos;
}

static inline uint32_t field_get(uint64_t raw, unsigned int pos, unsigned int len)
{
	return (uint32_t)((raw >> pos) & BIT64_MASK(len));
}

void frc_can_heartbeat_encode(const struct frc_can_heartbeat *hb, uint8_t data[8])
{
	uint64_t raw = 0U;

	raw |= field_put(hb->match_time_s, HB_MATCH_TIME_POS, HB_MATCH_TIME_LEN);
	raw |= field_put(hb->match_number, HB_MATCH_NUMBER_POS, HB_MATCH_NUMBER_LEN);
	raw |= field_put(hb->replay_number, HB_REPLAY_POS, HB_REPLAY_LEN);
	raw |= field_put(hb->ftc_motor_override, HB_FTC_OVERRIDE_POS, 1);
	raw |= field_put(hb->red_alliance, HB_RED_POS, 1);
	raw |= field_put(hb->enabled, HB_ENABLED_POS, 1);
	raw |= field_put(hb->autonomous, HB_AUTONOMOUS_POS, 1);
	raw |= field_put(hb->test_mode, HB_TEST_POS, 1);
	raw |= field_put(hb->system_watchdog, HB_WATCHDOG_POS, 1);
	raw |= field_put(hb->tournament_type, HB_TOURNAMENT_POS, HB_TOURNAMENT_LEN);
	raw |= field_put(hb->tod_year, HB_YEAR_POS, HB_YEAR_LEN);
	raw |= field_put(hb->tod_month, HB_MONTH_POS, HB_MONTH_LEN);
	raw |= field_put(hb->tod_day, HB_DAY_POS, HB_DAY_LEN);
	raw |= field_put(hb->tod_sec, HB_SEC_POS, HB_SEC_LEN);
	raw |= field_put(hb->tod_min, HB_MIN_POS, HB_MIN_LEN);
	raw |= field_put(hb->tod_hour, HB_HOUR_POS, HB_HOUR_LEN);

	sys_put_be64(raw, data);
}

void frc_can_heartbeat_decode(const uint8_t data[8], struct frc_can_heartbeat *hb)
{
	uint64_t raw = sys_get_be64(data);

	hb->match_time_s = (uint8_t)field_get(raw, HB_MATCH_TIME_POS, HB_MATCH_TIME_LEN);
	hb->match_number = (uint16_t)field_get(raw, HB_MATCH_NUMBER_POS, HB_MATCH_NUMBER_LEN);
	hb->replay_number = (uint8_t)field_get(raw, HB_REPLAY_POS, HB_REPLAY_LEN);
	hb->ftc_motor_override = field_get(raw, HB_FTC_OVERRIDE_POS, 1) != 0U;
	hb->red_alliance = field_get(raw, HB_RED_POS, 1) != 0U;
	hb->enabled = field_get(raw, HB_ENABLED_POS, 1) != 0U;
	hb->autonomous = field_get(raw, HB_AUTONOMOUS_POS, 1) != 0U;
	hb->test_mode = field_get(raw, HB_TEST_POS, 1) != 0U;
	hb->system_watchdog = field_get(raw, HB_WATCHDOG_POS, 1) != 0U;
	hb->tournament_type = (uint8_t)field_get(raw, HB_TOURNAMENT_POS, HB_TOURNAMENT_LEN);
	hb->tod_year = (uint8_t)field_get(raw, HB_YEAR_POS, HB_YEAR_LEN);
	hb->tod_month = (uint8_t)field_get(raw, HB_MONTH_POS, HB_MONTH_LEN);
	hb->tod_day = (uint8_t)field_get(raw, HB_DAY_POS, HB_DAY_LEN);
	hb->tod_sec = (uint8_t)field_get(raw, HB_SEC_POS, HB_SEC_LEN);
	hb->tod_min = (uint8_t)field_get(raw, HB_MIN_POS, HB_MIN_LEN);
	hb->tod_hour = (uint8_t)field_get(raw, HB_HOUR_POS, HB_HOUR_LEN);
}

const char *frc_can_manufacturer_name(uint8_t manufacturer)
{
	static const char *const names[] = {
		[FRC_CAN_MFR_BROADCAST] = "Broadcast",
		[FRC_CAN_MFR_NI] = "NI",
		[FRC_CAN_MFR_LIMELIGHT] = "Limelight",
		[FRC_CAN_MFR_FIRST] = "FIRST",
		[FRC_CAN_MFR_CTR_ELECTRONICS] = "CTR Electronics",
		[FRC_CAN_MFR_REV_ROBOTICS] = "REV Robotics",
		[FRC_CAN_MFR_GRAPPLE] = "Grapple",
		[FRC_CAN_MFR_MINDSENSORS] = "MindSensors",
		[FRC_CAN_MFR_TEAM_USE] = "Team Use",
		[FRC_CAN_MFR_KAUAI_LABS] = "Kauai Labs",
		[FRC_CAN_MFR_COPPERFORGE] = "Copperforge",
		[FRC_CAN_MFR_PLAYING_WITH_FUSION] = "Playing With Fusion",
		[FRC_CAN_MFR_STUDICA] = "Studica",
		[FRC_CAN_MFR_THE_THRIFTY_BOT] = "The Thrifty Bot",
		[FRC_CAN_MFR_REDUX_ROBOTICS] = "Redux Robotics",
		[FRC_CAN_MFR_ANDYMARK] = "AndyMark",
		[FRC_CAN_MFR_VIVID_HOSTING] = "Vivid Hosting",
		[FRC_CAN_MFR_VERTOS_ROBOTICS] = "Vertos Robotics",
		[FRC_CAN_MFR_SWYFT_ROBOTICS] = "SWYFT Robotics",
		[FRC_CAN_MFR_LUMYN_LABS] = "Lumyn Labs",
		[FRC_CAN_MFR_BRUSHLAND_LABS] = "Brushland Labs",
	};

	return (manufacturer < ARRAY_SIZE(names)) ? names[manufacturer] : NULL;
}

const char *frc_can_device_type_name(uint8_t device_type)
{
	static const char *const names[] = {
		[FRC_CAN_DEV_BROADCAST] = "Broadcast",
		[FRC_CAN_DEV_ROBOT_CONTROLLER] = "Robot Controller",
		[FRC_CAN_DEV_MOTOR_CONTROLLER] = "Motor Controller",
		[FRC_CAN_DEV_RELAY_CONTROLLER] = "Relay Controller",
		[FRC_CAN_DEV_GYRO_SENSOR] = "Gyro Sensor",
		[FRC_CAN_DEV_ACCELEROMETER] = "Accelerometer",
		[FRC_CAN_DEV_DISTANCE_SENSOR] = "Distance Sensor",
		[FRC_CAN_DEV_ENCODER] = "Encoder",
		[FRC_CAN_DEV_POWER_DISTRIBUTION] = "Power Distribution Module",
		[FRC_CAN_DEV_PNEUMATICS_CONTROLLER] = "Pneumatics Controller",
		[FRC_CAN_DEV_MISCELLANEOUS] = "Miscellaneous",
		[FRC_CAN_DEV_IO_BREAKOUT] = "IO Breakout",
		[FRC_CAN_DEV_SERVO_CONTROLLER] = "Servo Controller",
		[FRC_CAN_DEV_COLOR_SENSOR] = "Color Sensor",
	};

	if (device_type == FRC_CAN_DEV_FIRMWARE_UPDATE) {
		return "Firmware Update";
	}

	return (device_type < ARRAY_SIZE(names)) ? names[device_type] : NULL;
}
