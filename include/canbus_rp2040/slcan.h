/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief slcan (Lawicel / CANable) serial line CAN protocol.
 *
 * Implements the ASCII protocol spoken by CANable/CANtact adapters with the
 * "slcan" firmware and by Lawicel CAN232/CANUSB adapters, so that a Zephyr
 * CAN controller can be used from Linux SocketCAN (slcand / slcan driver),
 * python-can, SavvyCAN, Cangaroo and similar tools.
 *
 * Supported commands (terminated by CR):
 *
 * | Command        | Description                                         |
 * |----------------|-----------------------------------------------------|
 * | ``O``          | Open channel (normal mode)                          |
 * | ``L``          | Open channel in listen-only mode                    |
 * | ``C``          | Close channel                                       |
 * | ``S0``-``S8``  | Bitrate 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k, 1M |
 * | ``sxxyy``      | Bit timing from SJA1000 BTR0/BTR1 (16 MHz crystal)  |
 * | ``M0``/``M1``  | CANable: normal / silent (listen-only) mode         |
 * | ``A0``/``A1``  | CANable: disable / enable automatic retransmission  |
 * | ``Mxxxxxxxx``  | Lawicel acceptance code (accepted, see ``m``)       |
 * | ``mxxxxxxxx``  | Lawicel acceptance mask (only ``mFFFFFFFF``)        |
 * | ``Z0``/``Z1``  | Disable / enable RX timestamps (ms, 0-59999)        |
 * | ``tiiildd..``  | Transmit standard data frame                        |
 * | ``Tiiiiiiiildd..`` | Transmit extended data frame                    |
 * | ``riiil``      | Transmit standard remote frame                      |
 * | ``Riiiiiiiil`` | Transmit extended remote frame                      |
 * | ``F``          | Read and clear status flags                         |
 * | ``V``          | Hardware/software version                           |
 * | ``N``          | Serial number                                       |
 *
 * Successful commands are acknowledged with CR and failed commands with BEL
 * (0x07) when @kconfig{CONFIG_SLCAN_CMD_ACK} is enabled.
 *
 * Received frames are reported in the same format as the transmit commands,
 * optionally followed by a 4 digit hexadecimal timestamp. Error state changes
 * are reported as ``sXrrrttt`` and lost frames as ``e1o`` (the format parsed
 * by the Linux slcan driver) when enabled in Kconfig.
 *
 * The protocol engine is transport independent: feed received bytes into
 * slcan_receive() and call slcan_process() whenever slcan_poll_events()
 * signals pending work. All slcan_* calls for an instance must be made from
 * the same thread. slcan_uart_start() provides a ready-made UART / USB CDC ACM
 * transport.
 */

#ifndef CANBUS_RP2040_INCLUDE_SLCAN_H_
#define CANBUS_RP2040_INCLUDE_SLCAN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum length of a slcan message including the terminating CR. */
#define SLCAN_MTU 32

/**
 * @name Status flags reported by the F command (Lawicel definition)
 * @{
 */
/** The receive queue was full (frames were lost). */
#define SLCAN_STATUS_RX_FULL      BIT(0)
/** A transmit request was rejected because no TX buffer became free. */
#define SLCAN_STATUS_TX_FULL      BIT(1)
/** The controller is (or was) in the error warning state. */
#define SLCAN_STATUS_ERR_WARNING  BIT(2)
/** Received frames were lost. */
#define SLCAN_STATUS_DATA_OVERRUN BIT(3)
/** The controller is (or was) in the error passive state. */
#define SLCAN_STATUS_ERR_PASSIVE  BIT(5)
/** A frame lost arbitration (automatic retransmission disabled). */
#define SLCAN_STATUS_ARB_LOST     BIT(6)
/** A transmission failed or the controller went bus-off. */
#define SLCAN_STATUS_BUS_ERROR    BIT(7)
/** @} */

/** Events reported to the optional event callback (e.g. to drive LEDs). */
enum slcan_event {
	/** The channel was opened. */
	SLCAN_EVENT_OPEN,
	/** The channel was closed. */
	SLCAN_EVENT_CLOSE,
	/** A received frame was forwarded to the host. */
	SLCAN_EVENT_RX,
	/** A frame from the host was queued for transmission. */
	SLCAN_EVENT_TX,
	/** Frames were lost or the controller left the error-active state. */
	SLCAN_EVENT_ERROR,
};

/**
 * @brief Output function receiving protocol bytes for the host.
 */
typedef void (*slcan_write_t)(const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Event callback. Always called from the thread calling slcan_receive()
 * or slcan_process().
 */
typedef void (*slcan_event_cb_t)(enum slcan_event event, void *user_data);

/** @cond INTERNAL_HIDDEN */
struct slcan_rx_item {
	struct can_frame frame;
	uint32_t timestamp_ms;
};
/** @endcond */

/**
 * @brief slcan protocol instance. Treat all members as private.
 */
struct slcan {
	/** @cond INTERNAL_HIDDEN */
	const struct device *can;
	slcan_write_t write;
	void *write_user_data;
	slcan_event_cb_t event_cb;
	void *event_user_data;

	/* Frames received from the CAN controller, waiting to be sent to the host */
	struct k_msgq rx_msgq;
	char __aligned(4) rx_msgq_buf[CONFIG_SLCAN_RX_QUEUE_SIZE * sizeof(struct slcan_rx_item)];

	/* Raised from CAN callbacks when slcan_process() has work to do */
	struct k_poll_signal signal;
	atomic_t pending;
	atomic_t status;

	struct k_spinlock lock;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;

	/* Command line assembly */
	char line[SLCAN_MTU];
	size_t line_len;
	bool line_overflow;

	/* Channel configuration */
	bool open;
	bool listen_only;
	bool silent;
	bool auto_retransmit;
	bool timestamps;
	uint32_t bitrate;
	uint16_t sample_point;
	/** @endcond */
};

/**
 * @brief Initialise a slcan instance for a CAN controller.
 *
 * Stops the CAN controller, installs RX filters accepting all frames and
 * registers a state change callback. The channel starts closed.
 *
 * @param s Instance.
 * @param can CAN controller device.
 * @param write Function sending bytes to the host.
 * @param user_data User data passed to @a write.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the CAN controller is not ready.
 * @retval <0 if the RX filters could not be installed.
 */
int slcan_init(struct slcan *s, const struct device *can, slcan_write_t write, void *user_data);

/**
 * @brief Set the event callback (optional).
 */
void slcan_set_event_callback(struct slcan *s, slcan_event_cb_t cb, void *user_data);

/**
 * @brief Process bytes received from the host.
 *
 * Complete commands (terminated by CR) are executed immediately. LF
 * characters are ignored.
 */
void slcan_receive(struct slcan *s, const uint8_t *data, size_t len);

/**
 * @brief Forward received frames and pending notifications to the host.
 *
 * @return Number of frames forwarded.
 */
int slcan_process(struct slcan *s);

/**
 * @brief Initialise poll events that signal pending work for slcan_process().
 *
 * @param s Instance.
 * @param events Array of two poll events to initialise.
 */
void slcan_poll_events(struct slcan *s, struct k_poll_event events[2]);

/**
 * @brief Whether the channel is open.
 */
bool slcan_is_open(const struct slcan *s);

/**
 * @brief Format a CAN frame as a slcan message.
 *
 * @param frame Frame to format (classic CAN only).
 * @param timestamp Append the timestamp @a ts_ms.
 * @param ts_ms Timestamp in milliseconds (0-59999).
 * @param buf Destination buffer, NUL terminated on success.
 * @param size Size of @a buf.
 *
 * @return Length of the message (including the CR), or a negative error code.
 */
int slcan_format_frame(const struct can_frame *frame, bool timestamp, uint16_t ts_ms, char *buf,
		       size_t size);

/**
 * @brief Parse a slcan transmit command (t, T, r or R) into a CAN frame.
 *
 * @param cmd Command without the terminating CR.
 * @param len Length of @a cmd.
 * @param frame Parsed frame.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the command is malformed.
 */
int slcan_parse_frame(const char *cmd, size_t len, struct can_frame *frame);

/**
 * @brief Convert a SJA1000 BTR0/BTR1 pair (16 MHz crystal) to a bitrate.
 *
 * @param btr0 BTR0 register value.
 * @param btr1 BTR1 register value.
 * @param bitrate Resulting bitrate in bit/s.
 * @param sample_point Resulting sample point in permille.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the register values are invalid.
 */
int slcan_btr_to_bitrate(uint8_t btr0, uint8_t btr1, uint32_t *bitrate, uint16_t *sample_point);

/**
 * @brief Start the UART / USB CDC ACM transport.
 *
 * Creates a thread that connects the UART @a uart (for example a
 * ``zephyr,cdc-acm-uart`` instance) to the CAN controller @a can using the
 * slcan protocol. Only one transport instance is supported.
 *
 * @param uart UART device (interrupt driven API required).
 * @param can CAN controller device.
 * @param cb Optional event callback (may be NULL).
 * @param user_data User data passed to @a cb.
 *
 * @retval 0 on success.
 * @retval -EALREADY if the transport is already running.
 * @retval <0 on error.
 */
int slcan_uart_start(const struct device *uart, const struct device *can, slcan_event_cb_t cb,
		     void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_SLCAN_H_ */
