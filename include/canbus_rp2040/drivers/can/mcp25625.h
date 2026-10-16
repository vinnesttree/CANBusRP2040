/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MCP25625 specific extensions to the Zephyr CAN controller API.
 *
 * The MCP25625 is used through the standard Zephyr CAN API
 * (<zephyr/drivers/can.h>). The functions in this header expose hardware
 * features that the generic API has no notion of.
 */

#ifndef CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_H_
#define CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of acceptance filters attached to receive buffer 0. */
#define MCP25625_RXB0_FILTERS 2
/** Number of acceptance filters attached to receive buffer 1. */
#define MCP25625_RXB1_FILTERS 4

/**
 * @brief Hardware acceptance filter configuration.
 *
 * The MCP25625 has one acceptance mask and two filters for receive buffer 0
 * and one mask and four filters for receive buffer 1. A frame is accepted by
 * a buffer when <tt>(frame_id & mask) == (filter_id & mask)</tt> for at least
 * one of the buffer's filters and the frame type (standard/extended) matches
 * @a ide. Frames accepted by buffer 0 roll over into buffer 1 when buffer 0
 * is full.
 *
 * Hardware filtering only reduces the number of frames that are read over
 * SPI. Frames that pass the hardware filters are still matched against the
 * filters installed with can_add_rx_filter(). By default no hardware
 * filtering is configured and every frame is read.
 *
 * Unused filter slots must be set to one of the used IDs (for example by
 * repeating the first ID) since every slot is always active.
 */
struct mcp25625_hw_acceptance {
	/** Receive buffer 0: mask RXM0 and filters RXF0, RXF1. */
	struct {
		/** true: filters match extended (29-bit) IDs, false: standard IDs. */
		bool ide;
		/** Acceptance mask (11 or 29 bits depending on @a ide). */
		uint32_t mask;
		/** Filter IDs. */
		uint32_t id[MCP25625_RXB0_FILTERS];
	} rxb0;
	/** Receive buffer 1: mask RXM1 and filters RXF2 to RXF5. */
	struct {
		/** true: filters match extended (29-bit) IDs, false: standard IDs. */
		bool ide;
		/** Acceptance mask (11 or 29 bits depending on @a ide). */
		uint32_t mask;
		/** Filter IDs. */
		uint32_t id[MCP25625_RXB1_FILTERS];
	} rxb1;
};

/**
 * @brief Configure the MCP25625 hardware acceptance filters.
 *
 * The configuration is retained and used every time the controller is
 * started. The controller must be stopped when calling this function.
 *
 * @param dev MCP25625 device.
 * @param acc Acceptance configuration, or NULL to receive every frame
 *            (the default).
 *
 * @retval 0 on success.
 * @retval -EBUSY if the CAN controller is started.
 * @retval -EINVAL if an ID or mask is out of range for its frame type.
 * @retval -EIO on SPI communication failure.
 */
int mcp25625_set_hw_acceptance(const struct device *dev, const struct mcp25625_hw_acceptance *acc);

/**
 * @brief MCP25625 specific counters.
 */
struct mcp25625_stats {
	/** Frames read from the receive buffers. */
	uint32_t rx_frames;
	/** Frames transmitted successfully. */
	uint32_t tx_frames;
	/** Frames lost because both receive buffers were full. */
	uint32_t rx_overruns;
	/** Message error interrupts (bus errors seen during RX or TX). */
	uint32_t message_errors;
	/** Transmissions that ended without success (one-shot mode or bus-off). */
	uint32_t tx_failures;
	/** Number of times the controller entered the bus-off state. */
	uint32_t bus_off;
};

/**
 * @brief Read the MCP25625 specific counters.
 *
 * The counters are reset when the controller is started.
 *
 * @param dev MCP25625 device.
 * @param stats Destination for the counters.
 *
 * @retval 0 on success.
 */
int mcp25625_get_stats(const struct device *dev, struct mcp25625_stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_H_ */
