/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Backdoor API of the MCP25625 SPI emulator used for testing.
 *
 * The emulator models the MCP25625/MCP2515 register set and SPI instruction
 * set closely enough to exercise the driver on native_sim: operation modes,
 * the TXB0..TXB2 and RXB0/RXB1 buffers including rollover, acceptance masks
 * and filters, interrupt flags with the INT pin, error flags and counters.
 */

#ifndef CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_EMUL_H_
#define CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_EMUL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/emul.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simulate the reception of a frame from the CAN bus.
 *
 * @retval 0 if the frame was stored in a receive buffer.
 * @retval -EACCES if the frame was not accepted (filters or operation mode).
 * @retval -ENOBUFS if the frame was lost due to a receive buffer overflow.
 */
int mcp25625_emul_rx_frame(const struct emul *target, const struct can_frame *frame);

/**
 * @brief Fetch the oldest frame transmitted onto the simulated bus.
 *
 * @retval 0 if a frame was returned.
 * @retval -ENOMSG if no frame was transmitted.
 */
int mcp25625_emul_get_tx_frame(const struct emul *target, struct can_frame *frame);

/** @brief Discard all recorded transmitted frames. */
void mcp25625_emul_clear_tx_frames(const struct emul *target);

/**
 * @brief Select whether transmissions are acknowledged by the simulated bus.
 *
 * Without acknowledgement the transmit error flags are set and, unless
 * one-shot mode is enabled, the transmission stays pending.
 */
void mcp25625_emul_set_tx_ack(const struct emul *target, bool ack);

/** @brief Make the next transmission in one-shot mode lose arbitration. */
void mcp25625_emul_set_tx_lose_arbitration(const struct emul *target, bool lose);

/**
 * @brief Hold pending transmissions (simulates a busy bus).
 *
 * Releasing the hold processes all pending transmission requests.
 */
void mcp25625_emul_set_tx_hold(const struct emul *target, bool hold);

/**
 * @brief Set the error counters and update EFLG accordingly.
 *
 * @param tec Transmit error counter. Values above 255 put the controller
 *            into the bus-off state.
 * @param rec Receive error counter.
 *
 * ERRIF is raised when an error flag becomes set. Clearing error flags does
 * not raise an interrupt.
 */
void mcp25625_emul_set_error_counters(const struct emul *target, uint16_t tec, uint8_t rec);

/** @brief Read a register without side effects. */
uint8_t mcp25625_emul_get_reg(const struct emul *target, uint8_t addr);

/** @brief Current operation mode (CANSTAT.OPMOD). */
uint8_t mcp25625_emul_get_opmode(const struct emul *target);

/**
 * @brief Number of writes to configuration registers attempted outside of
 * configuration mode. Such writes are ignored by the device.
 */
uint32_t mcp25625_emul_get_config_violations(const struct emul *target);

#ifdef __cplusplus
}
#endif

#endif /* CANBUS_RP2040_INCLUDE_DRIVERS_CAN_MCP25625_EMUL_H_ */
