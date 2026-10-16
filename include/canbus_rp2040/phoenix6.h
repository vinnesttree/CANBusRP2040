/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Phoenix 6 control requests, configuration and status signals for a
 * Talon FX (Kraken X60 / X44) from the RP2040.
 *
 * CTR Electronics does not publish the CAN protocol of its devices, so the
 * requests are not encoded on the CAN bus by the RP2040. Instead they are sent
 * over a serial link (USB CDC ACM or UART) to the host bridge
 * (host/phoenix6_bridge), which executes them with CTR Electronics' official
 * Phoenix 6 library. The bridge can use this board as its SocketCAN adapter.
 *
 * All control requests of the Talon FX in the Phoenix 6 version the API was
 * generated from are available (see phoenix6_generated.h): open-loop
 * duty cycle / voltage / torque current, position and velocity closed loops,
 * Motion Magic (including Expo, Velocity and Dynamic variants), differential
 * mechanisms, followers, neutral / coast / static brake and music tones. Each
 * request has a struct with the Phoenix 6 parameter names and an initializer
 * macro with the Phoenix 6 defaults, for example:
 *
 * @code{.c}
 * phoenix6_bridge_start(DEVICE_DT_GET(DT_NODELABEL(phoenix6_uart)));
 * phoenix6_bridge_wait_connected(K_SECONDS(10));
 *
 * struct phoenix6_control req = PHOENIX6_VELOCITY_VOLTAGE(10.0);  // 10 rot/s
 * req.velocity_voltage.slot = 1;
 *
 * while (true) {
 *         phoenix6_feed_enable(100);
 *         phoenix6_set_control(1, &req);     // Talon FX with CAN ID 1
 *         k_msleep(20);
 * }
 * @endcode
 *
 * Phoenix 6 resends a control request periodically (update_freq_hz), so
 * phoenix6_set_control() only needs to be called when the request changes.
 * Actuators are only enabled while phoenix6_feed_enable() is called
 * periodically and the link to the host bridge is up.
 */

#ifndef CANBUS_RP2040_INCLUDE_PHOENIX6_H_
#define CANBUS_RP2040_INCLUDE_PHOENIX6_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <canbus_rp2040/phoenix6_generated.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Status codes
 *
 * Results reported by the host bridge are Phoenix 6 StatusCode values
 * (0 = OK, negative = error, positive = warning) or one of the bridge codes
 * below.
 * @{
 */
/** The request completed successfully. */
#define PHOENIX6_STATUS_OK                  0
/** The bridge rejected a malformed request. */
#define PHOENIX6_STATUS_BRIDGE_BAD_REQUEST  (-20001)
/** The bridge did not accept the session (schema mismatch). */
#define PHOENIX6_STATUS_BRIDGE_NOT_ACCEPTED (-20002)
/** The Phoenix 6 library raised an exception on the host. */
#define PHOENIX6_STATUS_BRIDGE_EXCEPTION    (-20003)
/** No result has been received yet. */
#define PHOENIX6_STATUS_NONE                INT32_MIN
/** @} */

/**
 * @name Descriptors of the generated API
 *
 * The tables describe every control request, configuration field and status
 * signal by name, for generic tools such as the shell.
 * @{
 */

/** Value types. */
enum phoenix6_type {
	PHOENIX6_TYPE_FLOAT,
	PHOENIX6_TYPE_BOOL,
	PHOENIX6_TYPE_INT,
	PHOENIX6_TYPE_ENUM,
	PHOENIX6_TYPE_CONTROL,
};

/** The control request requires a Phoenix Pro license. */
#define PHOENIX6_CONTROL_FLAG_PRO      BIT(0)
/** The control request requires a CANivore (CAN FD). */
#define PHOENIX6_CONTROL_FLAG_CANIVORE BIT(1)

/** Value of an enumeration. */
struct phoenix6_enum_member {
	const char *name;
	int32_t value;
};

/** Enumeration. */
struct phoenix6_enum_desc {
	const char *name;
	uint8_t count;
	const struct phoenix6_enum_member *members;
};

/** Parameter of a control request. */
struct phoenix6_param_desc {
	/** Phoenix 6 parameter name. */
	const char *name;
	/** Offset within the request's parameter struct. */
	uint16_t offset;
	/** Value type, see @ref phoenix6_type. */
	uint8_t type;
	/** Index into phoenix6_enum_descs, or 0xFF. */
	uint8_t enum_index;
	/** Nested request ID for PHOENIX6_TYPE_CONTROL. */
	uint16_t control;
	/** The parameter has no default value. */
	bool required;
	/** Phoenix 6 default value (booleans 0/1, enums their value). */
	double default_value;
};

/** Control request. */
struct phoenix6_control_desc {
	/** Phoenix 6 class name. */
	const char *name;
	/** Request ID. */
	uint16_t id;
	/** Size of the parameter struct. */
	uint16_t size;
	/** Default update frequency in Hz. */
	uint16_t update_freq_hz;
	/** Number of parameters. */
	uint8_t param_count;
	/** PHOENIX6_CONTROL_FLAG_* */
	uint8_t flags;
	/** Parameters, in Phoenix 6 order. */
	const struct phoenix6_param_desc *params;
};

/** Configuration field. */
struct phoenix6_config_desc {
	/** Configuration group (e.g. "slot0"). */
	const char *group;
	/** Field name (e.g. "k_p"). */
	const char *field;
	/** Field ID. */
	uint16_t id;
	/** Value type, see @ref phoenix6_type. */
	uint8_t type;
	/** Index into phoenix6_enum_descs, or 0xFF. */
	uint8_t enum_index;
};

/** Status signal. */
struct phoenix6_signal_desc {
	/** Signal name (the Phoenix 6 getter without "get_"). */
	const char *name;
	/** Unit, or NULL. */
	const char *unit;
	/** Signal ID. */
	uint16_t id;
	/** Value type, see @ref phoenix6_type. */
	uint8_t type;
	/** Index into phoenix6_enum_descs, or 0xFF. */
	uint8_t enum_index;
};

/** Control requests, indexed by ID - 1. */
extern const struct phoenix6_control_desc phoenix6_control_descs[];
/** Configuration fields, indexed by ID - 1. */
extern const struct phoenix6_config_desc phoenix6_config_descs[];
/** Status signals, indexed by ID - 1. */
extern const struct phoenix6_signal_desc phoenix6_signal_descs[];
/** Enumerations. */
extern const struct phoenix6_enum_desc phoenix6_enum_descs[];
/** Number of enumerations. */
extern const size_t phoenix6_enum_count;

/** @brief Descriptor of a control request, or NULL for an invalid ID. */
const struct phoenix6_control_desc *phoenix6_control_desc_get(enum phoenix6_control_id id);

/** @brief Find a control request by its Phoenix 6 name (e.g. "VelocityVoltage"). */
const struct phoenix6_control_desc *phoenix6_control_desc_find(const char *name);

/** @brief Find a configuration field by "group.field" (e.g. "slot0.k_p"). */
const struct phoenix6_config_desc *phoenix6_config_desc_find(const char *name);

/** @brief Find a status signal by name (e.g. "velocity"). */
const struct phoenix6_signal_desc *phoenix6_signal_desc_find(const char *name);

/** @brief Look up an enum member by name. @retval 0 on success, -ENOENT if unknown. */
int phoenix6_enum_value(uint8_t enum_index, const char *name, int32_t *value);

/** @brief Name of an enum value, or NULL. */
const char *phoenix6_enum_name(uint8_t enum_index, int32_t value);

/**
 * @brief Initialise a control request with the Phoenix 6 defaults.
 *
 * Required parameters (those without a default) are set to zero.
 *
 * @retval 0 on success.
 * @retval -EINVAL for an invalid ID.
 */
int phoenix6_control_init(struct phoenix6_control *request, enum phoenix6_control_id id);

/** @} */

/**
 * @name Bridge link
 * @{
 */

/**
 * @brief Start the bridge client on a UART (e.g. a USB CDC ACM instance).
 *
 * @retval 0 on success.
 * @retval -EALREADY if already started.
 * @retval -ENODEV if the UART is not ready.
 */
int phoenix6_bridge_start(const struct device *uart);

/** @brief Whether the host bridge accepted the session and the link is alive. */
bool phoenix6_bridge_is_connected(void);

/**
 * @brief Wait until the host bridge is connected.
 *
 * @retval 0 when connected.
 * @retval -EPROTO on timeout, the bridge rejected the session (its Phoenix 6
 *         version or schema differs from the one the firmware was built for).
 * @retval -EAGAIN on timeout, no answer from the bridge.
 */
int phoenix6_bridge_wait_connected(k_timeout_t timeout);

/** @brief Phoenix 6 version reported by the host bridge (empty if unknown). */
const char *phoenix6_bridge_host_version(void);

/** @} */

/**
 * @name Talon FX operations
 *
 * @a device is the CAN ID of the Talon FX on the bridge's CAN bus.
 * Functions return 0 when the request was delivered and answered, with the
 * Phoenix 6 result in @a status (may be NULL), or a negative errno:
 * -ENOTCONN (link down), -ETIMEDOUT (no answer), -EINVAL (invalid argument),
 * -ENOBUFS (too many requests in flight) or -EIO (transmit failure).
 * @{
 */

/**
 * @brief Feed the enable signal (Phoenix 6 unmanaged feed_enable).
 *
 * Actuators stay enabled for @a timeout_ms after the last call; 0 disables
 * them immediately.
 */
int phoenix6_feed_enable(uint32_t timeout_ms);

/**
 * @brief Send a control request. Does not wait for the result, see
 * phoenix6_last_control_status().
 */
int phoenix6_set_control(uint8_t device, const struct phoenix6_control *request);

/** @brief Result of the last control request sent to @a device. */
int32_t phoenix6_last_control_status(uint8_t device);

/**
 * @brief Set one configuration field. The other fields of its group are
 * preserved (the group is read, modified and written).
 */
int phoenix6_config_set(uint8_t device, enum phoenix6_config_field field, double value,
			uint32_t timeout_ms, int32_t *status);

/** @brief Read one configuration field from the device. */
int phoenix6_config_get(uint8_t device, enum phoenix6_config_field field, double *value,
			uint32_t timeout_ms, int32_t *status);

/** @brief Restore the factory default configuration. */
int phoenix6_factory_default(uint8_t device, uint32_t timeout_ms, int32_t *status);

/** @brief Set the mechanism position in rotations. */
int phoenix6_set_position(uint8_t device, double rotations, uint32_t timeout_ms, int32_t *status);

/** @brief Clear all sticky faults. */
int phoenix6_clear_sticky_faults(uint8_t device, uint32_t timeout_ms, int32_t *status);

/**
 * @brief Stream a status signal from the bridge at @a freq_hz (0 stops).
 *
 * The bridge also sets the signal's update frequency on the CAN bus to
 * @a freq_hz.
 */
int phoenix6_subscribe(uint8_t device, enum phoenix6_signal signal, float freq_hz, int32_t *status);

/** Latest value of a status signal. */
struct phoenix6_signal_value {
	/** Value (booleans as 0/1, enums as their Phoenix 6 value). */
	double value;
	/** Phoenix 6 status of the value. */
	int32_t status;
	/** Uptime in ms when the value was received. */
	int64_t received_ms;
	/** Age of the value in ms when the bridge sent it. */
	float age_ms;
};

/**
 * @brief Get the latest value of a subscribed status signal.
 *
 * @retval 0 on success.
 * @retval -ENODATA if no value was received yet.
 */
int phoenix6_get_signal(uint8_t device, enum phoenix6_signal signal,
			struct phoenix6_signal_value *value);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_PHOENIX6_H_ */
