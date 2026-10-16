/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr CAN controller driver for the Microchip MCP25625 (SPI CAN 2.0B
 * controller with integrated high-speed CAN transceiver).
 *
 * The structure follows the in-tree Zephyr MCP2515 driver. In addition this
 * driver controls the STBY pin of the integrated transceiver, supports
 * one-shot and triple-sampling modes, completes pending transmissions on
 * bus-off, reports error state changes (including recovery) and offers
 * optional hardware acceptance filtering.
 */

#define DT_DRV_COMPAT microchip_mcp25625

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <canbus_rp2040/drivers/can/mcp25625.h>

#include "can_mcp25625.h"

LOG_MODULE_REGISTER(can_mcp25625, CONFIG_CAN_LOG_LEVEL);

/* Interrupts handled by the driver */
#define MCP25625_CANINTE_DEFAULT                                                                   \
	(MCP25625_INT_RX0 | MCP25625_INT_RX1 | MCP25625_INT_TX0 | MCP25625_INT_ERR |               \
	 MCP25625_INT_MERR)

/* Maximum time to wait for an operation mode change to take effect */
#define MCP25625_MODE_CHANGE_TIMEOUT_MS 50
/* Maximum time to wait for the device to come out of reset */
#define MCP25625_RESET_TIMEOUT_MS       100
/* Interval for re-sending the RESET instruction while waiting for the device */
#define MCP25625_RESET_RETRY_MS         10
/* Error state polling interval while not in the error-active state */
#define MCP25625_STATE_POLL_MS          CONFIG_CAN_MCP25625_STATE_POLL_INTERVAL_MS
/* Upper bound of the length of a classic CAN frame in bits, including stuff bits */
#define MCP25625_MAX_FRAME_BITS         160U

#define MCP25625_SUPPORTED_MODES                                                                   \
	(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT | CAN_MODE_3_SAMPLES)

struct mcp25625_config {
	const struct can_driver_config common;

	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec standby_gpio;
	struct gpio_dt_spec reset_gpio;

	uint32_t osc_freq;
	/* CANCTRL CLKEN/CLKPRE bits */
	uint8_t canctrl_clkout;

	k_thread_stack_t *int_thread_stack;
	size_t int_thread_stack_size;
	int int_thread_priority;
};

struct mcp25625_rx_filter {
	can_rx_callback_t callback;
	void *user_data;
	struct can_filter filter;
};

struct mcp25625_data {
	struct can_driver_data common;
	const struct device *dev;

	/* Interrupt handling */
	struct gpio_callback int_gpio_cb;
	struct k_thread int_thread;
	struct k_sem int_sem;
	struct k_work_delayable poll_work;

	/* Serialises register access sequences and the driver state below */
	struct k_mutex lock;

	/* Transmit state (a single TX buffer is used to keep frames in order) */
	struct k_sem tx_sem;
	bool tx_busy;
	can_tx_callback_t tx_callback;
	void *tx_user_data;

	/* Receive filters */
	struct k_mutex filter_lock;
	struct mcp25625_rx_filter filters[CONFIG_CAN_MCP25625_MAX_FILTERS];

	/* Controller state */
	enum can_state state;
	uint32_t bitrate;
	bool hw_acc_enabled;
	struct mcp25625_hw_acceptance hw_acc;
	struct mcp25625_stats stats;
};

/*
 * Low level SPI access. Every access is a single full-duplex transfer with
 * equally sized TX and RX buffers.
 */
static int mcp25625_transfer(const struct device *dev, uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct mcp25625_config *config = dev->config;
	const struct spi_buf tx_buf = {.buf = tx, .len = len};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	if (rx == NULL) {
		return spi_write_dt(&config->bus, &tx_set);
	}

	const struct spi_buf rx_buf = {.buf = rx, .len = len};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	return spi_transceive_dt(&config->bus, &tx_set, &rx_set);
}

static int mcp25625_cmd_reset(const struct device *dev)
{
	uint8_t tx[] = {MCP25625_OP_RESET};

	return mcp25625_transfer(dev, tx, NULL, sizeof(tx));
}

static int mcp25625_read_regs(const struct device *dev, uint8_t addr, uint8_t *data, size_t len)
{
	uint8_t tx[2 + MCP25625_FRAME_LEN] = {MCP25625_OP_READ, addr};
	uint8_t rx[2 + MCP25625_FRAME_LEN];
	int err;

	__ASSERT_NO_MSG(len <= MCP25625_FRAME_LEN);

	err = mcp25625_transfer(dev, tx, rx, 2 + len);
	if (err == 0) {
		memcpy(data, &rx[2], len);
	}

	return err;
}

static int mcp25625_read_reg(const struct device *dev, uint8_t addr, uint8_t *value)
{
	return mcp25625_read_regs(dev, addr, value, 1);
}

static int mcp25625_write_regs(const struct device *dev, uint8_t addr, const uint8_t *data,
			       size_t len)
{
	uint8_t tx[2 + MCP25625_FRAME_LEN] = {MCP25625_OP_WRITE, addr};

	__ASSERT_NO_MSG(len <= MCP25625_FRAME_LEN);

	memcpy(&tx[2], data, len);

	return mcp25625_transfer(dev, tx, NULL, 2 + len);
}

static int mcp25625_write_reg(const struct device *dev, uint8_t addr, uint8_t value)
{
	return mcp25625_write_regs(dev, addr, &value, 1);
}

static int mcp25625_bit_modify(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t value)
{
	uint8_t tx[] = {MCP25625_OP_BIT_MODIFY, addr, mask, value};

	return mcp25625_transfer(dev, tx, NULL, sizeof(tx));
}

/* Read a complete frame from RXB0 (idx 0) or RXB1 (idx 1). Clears RXnIF. */
static int mcp25625_read_rx_buffer(const struct device *dev, uint8_t idx, uint8_t *frame)
{
	uint8_t tx[1 + MCP25625_FRAME_LEN] = {MCP25625_OP_READ_RX_BUFFER | (idx << 2)};
	uint8_t rx[1 + MCP25625_FRAME_LEN];
	int err;

	err = mcp25625_transfer(dev, tx, rx, sizeof(tx));
	if (err == 0) {
		memcpy(frame, &rx[1], MCP25625_FRAME_LEN);
	}

	return err;
}

/* Load TXB0 starting at TXB0SIDH and request its transmission */
static int mcp25625_load_and_send_txb0(const struct device *dev, const uint8_t *frame, size_t len)
{
	uint8_t tx[1 + MCP25625_FRAME_LEN] = {MCP25625_OP_LOAD_TX_BUFFER};
	uint8_t rts[] = {MCP25625_OP_RTS | BIT(0)};
	int err;

	__ASSERT_NO_MSG(len <= MCP25625_FRAME_LEN);

	memcpy(&tx[1], frame, len);

	err = mcp25625_transfer(dev, tx, NULL, 1 + len);
	if (err != 0) {
		return err;
	}

	return mcp25625_transfer(dev, rts, NULL, sizeof(rts));
}

/* Encode an identifier into the SIDH, SIDL, EID8, EID0 register layout */
static void mcp25625_encode_id(uint32_t id, bool ide, uint8_t *regs)
{
	if (ide) {
		regs[0] = (uint8_t)(id >> 21);
		regs[1] = (uint8_t)((((id >> 18) & 0x07U) << 5) | MCP25625_SIDL_IDE |
				    ((id >> 16) & 0x03U));
		regs[2] = (uint8_t)(id >> 8);
		regs[3] = (uint8_t)id;
	} else {
		regs[0] = (uint8_t)(id >> 3);
		regs[1] = (uint8_t)((id & 0x07U) << 5);
		regs[2] = 0U;
		regs[3] = 0U;
	}
}

static size_t mcp25625_frame_to_regs(const struct can_frame *frame, uint8_t *regs)
{
	bool rtr = (frame->flags & CAN_FRAME_RTR) != 0U;

	mcp25625_encode_id(frame->id, (frame->flags & CAN_FRAME_IDE) != 0U, regs);
	regs[MCP25625_FRAME_DLC] = (rtr ? MCP25625_DLC_RTR : 0U) | (frame->dlc & 0x0FU);

	if (rtr) {
		return MCP25625_FRAME_D0;
	}

	memcpy(&regs[MCP25625_FRAME_D0], frame->data, frame->dlc);

	return MCP25625_FRAME_D0 + frame->dlc;
}

static void mcp25625_regs_to_frame(const uint8_t *regs, struct can_frame *frame)
{
	uint8_t sidl = regs[MCP25625_FRAME_SIDL];
	uint8_t dlc_reg = regs[MCP25625_FRAME_DLC];
	bool rtr;

	memset(frame, 0, sizeof(*frame));

	if ((sidl & MCP25625_SIDL_IDE) != 0U) {
		frame->flags |= CAN_FRAME_IDE;
		frame->id = ((uint32_t)regs[MCP25625_FRAME_SIDH] << 21) |
			    ((uint32_t)(sidl >> 5) << 18) | ((uint32_t)(sidl & 0x03U) << 16) |
			    ((uint32_t)regs[MCP25625_FRAME_EID8] << 8) | regs[MCP25625_FRAME_EID0];
		/* For extended frames the RTR bit is located in RXBnDLC */
		rtr = (dlc_reg & MCP25625_DLC_RTR) != 0U;
	} else {
		frame->id = ((uint32_t)regs[MCP25625_FRAME_SIDH] << 3) | (sidl >> 5);
		/* For standard frames the RTR bit is reported as SRR in RXBnSIDL */
		rtr = (sidl & MCP25625_SIDL_SRR) != 0U;
	}

	/* DLC values 9..15 mean 8 data bytes in classic CAN */
	frame->dlc = MIN(dlc_reg & MCP25625_DLC_DLC_MASK, CAN_MAX_DLC);

	if (rtr) {
		frame->flags |= CAN_FRAME_RTR;
	} else {
		memcpy(frame->data, &regs[MCP25625_FRAME_D0], frame->dlc);
	}
}

static enum can_state mcp25625_eflg_to_state(uint8_t eflg)
{
	if ((eflg & MCP25625_EFLG_TXBO) != 0U) {
		return CAN_STATE_BUS_OFF;
	}

	if ((eflg & (MCP25625_EFLG_TXEP | MCP25625_EFLG_RXEP)) != 0U) {
		return CAN_STATE_ERROR_PASSIVE;
	}

	if ((eflg & (MCP25625_EFLG_EWARN | MCP25625_EFLG_TXWAR | MCP25625_EFLG_RXWAR)) != 0U) {
		return CAN_STATE_ERROR_WARNING;
	}

	return CAN_STATE_ERROR_ACTIVE;
}

static int mcp25625_set_opmode(const struct device *dev, uint8_t opmode)
{
	int64_t deadline = k_uptime_get() + MCP25625_MODE_CHANGE_TIMEOUT_MS;
	uint8_t canstat;
	int err;

	err = mcp25625_bit_modify(dev, MCP25625_REG_CANCTRL, MCP25625_CANCTRL_REQOP_MASK,
				  FIELD_PREP(MCP25625_CANCTRL_REQOP_MASK, opmode));
	if (err != 0) {
		return err;
	}

	while (true) {
		err = mcp25625_read_reg(dev, MCP25625_REG_CANSTAT, &canstat);
		if (err != 0) {
			return err;
		}

		if (FIELD_GET(MCP25625_CANSTAT_OPMOD_MASK, canstat) == opmode) {
			return 0;
		}

		if (k_uptime_get() > deadline) {
			LOG_ERR("timeout entering operation mode %u (CANSTAT 0x%02x)", opmode,
				canstat);
			return -EIO;
		}

		k_usleep(100);
	}
}

static void mcp25625_set_standby(const struct device *dev, bool standby)
{
	const struct mcp25625_config *config = dev->config;
	int err;

	if (config->standby_gpio.port == NULL) {
		return;
	}

	err = gpio_pin_set_dt(&config->standby_gpio, standby ? 1 : 0);
	if (err != 0) {
		LOG_ERR("failed to set STBY pin (err %d)", err);
	}
}

/* Program RXBnCTRL, masks and filters. Must be called in configuration mode. */
static int mcp25625_apply_rx_config(const struct device *dev)
{
	struct mcp25625_data *data = dev->data;
	static const uint8_t filter_addr[] = {
		MCP25625_REG_RXF0SIDH, MCP25625_REG_RXF1SIDH, MCP25625_REG_RXF2SIDH,
		MCP25625_REG_RXF3SIDH, MCP25625_REG_RXF4SIDH, MCP25625_REG_RXF5SIDH,
	};
	const struct mcp25625_hw_acceptance *acc = &data->hw_acc;
	uint8_t rxm = data->hw_acc_enabled ? MCP25625_RXBCTRL_RXM_FILTER : MCP25625_RXBCTRL_RXM_ANY;
	uint8_t regs[4];
	int err;

	if (data->hw_acc_enabled) {
		/*
		 * Masks have no EXIDE bit. For standard frames the EID part of the
		 * mask would be applied to the first two data bytes, so it is left
		 * cleared for standard ID configurations.
		 */
		mcp25625_encode_id(acc->rxb0.mask, acc->rxb0.ide, regs);
		regs[1] &= ~MCP25625_SIDL_IDE;
		err = mcp25625_write_regs(dev, MCP25625_REG_RXM0SIDH, regs, sizeof(regs));
		if (err != 0) {
			return err;
		}

		mcp25625_encode_id(acc->rxb1.mask, acc->rxb1.ide, regs);
		regs[1] &= ~MCP25625_SIDL_IDE;
		err = mcp25625_write_regs(dev, MCP25625_REG_RXM1SIDH, regs, sizeof(regs));
		if (err != 0) {
			return err;
		}

		for (size_t i = 0; i < ARRAY_SIZE(filter_addr); i++) {
			if (i < MCP25625_RXB0_FILTERS) {
				mcp25625_encode_id(acc->rxb0.id[i], acc->rxb0.ide, regs);
			} else {
				mcp25625_encode_id(acc->rxb1.id[i - MCP25625_RXB0_FILTERS],
						   acc->rxb1.ide, regs);
			}

			err = mcp25625_write_regs(dev, filter_addr[i], regs, sizeof(regs));
			if (err != 0) {
				return err;
			}
		}
	}

	/* RXB0 rolls over into RXB1 when full */
	err = mcp25625_write_reg(dev, MCP25625_REG_RXB0CTRL,
				 FIELD_PREP(MCP25625_RXBCTRL_RXM_MASK, rxm) |
					 MCP25625_RXB0CTRL_BUKT);
	if (err != 0) {
		return err;
	}

	return mcp25625_write_reg(dev, MCP25625_REG_RXB1CTRL,
				  FIELD_PREP(MCP25625_RXBCTRL_RXM_MASK, rxm));
}

/* Claim the pending transmission, if any. Must be called with data->lock held. */
static bool mcp25625_claim_tx(struct mcp25625_data *data, can_tx_callback_t *callback,
			      void **user_data)
{
	if (!data->tx_busy) {
		return false;
	}

	*callback = data->tx_callback;
	*user_data = data->tx_user_data;
	data->tx_busy = false;
	data->tx_callback = NULL;
	data->tx_user_data = NULL;

	return true;
}

/* Complete a claimed transmission. Must be called without data->lock held. */
static void mcp25625_complete_tx(const struct device *dev, can_tx_callback_t callback,
				 void *user_data, int status)
{
	struct mcp25625_data *data = dev->data;

	k_sem_give(&data->tx_sem);

	if (callback != NULL) {
		callback(dev, status, user_data);
	}
}

static k_timeout_t mcp25625_tx_poll_delay(const struct mcp25625_data *data)
{
	uint32_t bitrate = MAX(data->bitrate, 1000U);
	uint32_t us = (2U * MCP25625_MAX_FRAME_BITS * USEC_PER_SEC) / bitrate;

	return K_USEC(MAX(us, 200U));
}

/* Schedule polling where no interrupt reports progress. Called with data->lock held. */
static void mcp25625_update_polling(const struct device *dev)
{
	struct mcp25625_data *data = dev->data;

	if (!data->common.started) {
		(void)k_work_cancel_delayable(&data->poll_work);
		return;
	}

	if (data->tx_busy && (data->common.mode & CAN_MODE_ONE_SHOT) != 0U) {
		/* Lost arbitration in one-shot mode raises no interrupt */
		(void)k_work_reschedule(&data->poll_work, mcp25625_tx_poll_delay(data));
	} else if (data->state != CAN_STATE_ERROR_ACTIVE) {
		/* Error state recovery raises no interrupt */
		(void)k_work_reschedule(&data->poll_work, K_MSEC(MCP25625_STATE_POLL_MS));
	} else {
		(void)k_work_cancel_delayable(&data->poll_work);
	}
}

static void mcp25625_poll_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mcp25625_data *data = CONTAINER_OF(dwork, struct mcp25625_data, poll_work);

	k_sem_give(&data->int_sem);
}

static void mcp25625_dispatch_rx(const struct device *dev, const uint8_t *regs)
{
	struct mcp25625_data *data = dev->data;
	struct can_frame frame;
	struct can_frame tmp;

	mcp25625_regs_to_frame(regs, &frame);

#ifndef CONFIG_CAN_ACCEPT_RTR
	if ((frame.flags & CAN_FRAME_RTR) != 0U) {
		return;
	}
#endif /* !CONFIG_CAN_ACCEPT_RTR */

	k_mutex_lock(&data->filter_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(data->filters); i++) {
		struct mcp25625_rx_filter *f = &data->filters[i];

		if (f->callback == NULL || !can_frame_matches_filter(&frame, &f->filter)) {
			continue;
		}

		/* Each callback gets its own copy in case it modifies the frame */
		tmp = frame;
		f->callback(dev, &tmp, f->user_data);
	}

	k_mutex_unlock(&data->filter_lock);
}

/*
 * Handle all pending interrupt sources once.
 *
 * Register accesses are done with data->lock held. Callbacks are invoked
 * after releasing it so they may call back into the driver.
 */
static int mcp25625_service(const struct device *dev)
{
	struct mcp25625_data *data = dev->data;
	uint8_t rx_regs[2][MCP25625_FRAME_LEN];
	bool rx_valid[2] = {false, false};
	can_tx_callback_t tx_callback = NULL;
	void *tx_user_data = NULL;
	bool tx_claimed = false;
	int tx_status = 0;
	can_state_change_callback_t state_cb = NULL;
	void *state_cb_data = NULL;
	struct can_bus_err_cnt err_cnt = {0};
	enum can_state new_state = CAN_STATE_STOPPED;
	uint8_t flags[2];
	uint8_t intf;
	uint8_t eflg;
	uint8_t clear;
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);

	/* CANINTF and EFLG are adjacent registers */
	err = mcp25625_read_regs(dev, MCP25625_REG_CANINTF, flags, sizeof(flags));
	if (err != 0) {
		goto unlock;
	}

	intf = flags[0];
	eflg = flags[1];

	if ((intf & MCP25625_INT_RX0) != 0U) {
		err = mcp25625_read_rx_buffer(dev, 0, rx_regs[0]);
		if (err != 0) {
			goto unlock;
		}
		rx_valid[0] = true;
		data->stats.rx_frames++;

		/*
		 * A frame may have rolled over into RXB1 while RXB0 was read. It is
		 * older than anything that can arrive in the now empty RXB0, so read
		 * it in this pass to preserve the reception order.
		 */
		if ((intf & MCP25625_INT_RX1) == 0U) {
			err = mcp25625_read_regs(dev, MCP25625_REG_CANINTF, flags, sizeof(flags));
			if (err != 0) {
				goto unlock;
			}
			intf |= flags[0] & ~MCP25625_INT_RX0;
			eflg |= flags[1];
		}
	}

	if ((intf & MCP25625_INT_RX1) != 0U) {
		err = mcp25625_read_rx_buffer(dev, 1, rx_regs[1]);
		if (err != 0) {
			goto unlock;
		}
		rx_valid[1] = true;
		data->stats.rx_frames++;
	}

	if ((eflg & (MCP25625_EFLG_RX0OVR | MCP25625_EFLG_RX1OVR)) != 0U) {
		if ((eflg & MCP25625_EFLG_RX0OVR) != 0U) {
			data->stats.rx_overruns++;
			CAN_STATS_RX_OVERRUN_INC(dev);
		}
		if ((eflg & MCP25625_EFLG_RX1OVR) != 0U) {
			data->stats.rx_overruns++;
			CAN_STATS_RX_OVERRUN_INC(dev);
		}

		err = mcp25625_bit_modify(dev, MCP25625_REG_EFLG,
					  eflg & (MCP25625_EFLG_RX0OVR | MCP25625_EFLG_RX1OVR), 0U);
		if (err != 0) {
			goto unlock;
		}
	}

	if ((intf & MCP25625_INT_MERR) != 0U) {
		data->stats.message_errors++;
	}

	/* RXnIF are cleared by the READ RX BUFFER instruction */
	clear = intf & ~(MCP25625_INT_RX0 | MCP25625_INT_RX1);
	if (clear != 0U) {
		err = mcp25625_bit_modify(dev, MCP25625_REG_CANINTF, clear, 0U);
		if (err != 0) {
			goto unlock;
		}
	}

	if (data->common.started) {
		new_state = mcp25625_eflg_to_state(eflg);
	}

	if (data->tx_busy) {
		if ((intf & MCP25625_INT_TX0) != 0U) {
			tx_claimed = mcp25625_claim_tx(data, &tx_callback, &tx_user_data);
			tx_status = 0;
			data->stats.tx_frames++;
		} else if (new_state == CAN_STATE_BUS_OFF) {
			/* No transmission is possible in bus-off; abort the request */
			err = mcp25625_bit_modify(dev, MCP25625_REG_TXB0CTRL,
						  MCP25625_TXBCTRL_TXREQ, 0U);
			if (err != 0) {
				goto unlock;
			}
			tx_claimed = mcp25625_claim_tx(data, &tx_callback, &tx_user_data);
			tx_status = -ENETUNREACH;
			data->stats.tx_failures++;
		} else if ((data->common.mode & CAN_MODE_ONE_SHOT) != 0U) {
			uint8_t txbctrl;
			uint8_t intf2;

			err = mcp25625_read_reg(dev, MCP25625_REG_TXB0CTRL, &txbctrl);
			if (err != 0) {
				goto unlock;
			}

			if ((txbctrl & MCP25625_TXBCTRL_TXREQ) == 0U) {
				/*
				 * The single attempt is over. TXREQ clears together with
				 * TX0IF being set on success, so check TX0IF again now.
				 */
				err = mcp25625_read_reg(dev, MCP25625_REG_CANINTF, &intf2);
				if (err != 0) {
					goto unlock;
				}

				if ((intf2 & MCP25625_INT_TX0) != 0U) {
					err = mcp25625_bit_modify(dev, MCP25625_REG_CANINTF,
								  MCP25625_INT_TX0, 0U);
					if (err != 0) {
						goto unlock;
					}
					tx_status = 0;
					data->stats.tx_frames++;
				} else if ((txbctrl & MCP25625_TXBCTRL_MLOA) != 0U) {
					tx_status = -EBUSY;
					data->stats.tx_failures++;
				} else {
					tx_status = -EIO;
					data->stats.tx_failures++;
				}

				tx_claimed = mcp25625_claim_tx(data, &tx_callback, &tx_user_data);
			}
		}
	}

	if (data->common.started && new_state != data->state) {
		uint8_t cnt[2];

		if (new_state == CAN_STATE_BUS_OFF) {
			data->stats.bus_off++;
		}

		data->state = new_state;

		/* TEC and REC are adjacent registers */
		err = mcp25625_read_regs(dev, MCP25625_REG_TEC, cnt, sizeof(cnt));
		if (err == 0) {
			err_cnt.tx_err_cnt = cnt[0];
			err_cnt.rx_err_cnt = cnt[1];
		}

		state_cb = data->common.state_change_cb;
		state_cb_data = data->common.state_change_cb_user_data;
	}

	mcp25625_update_polling(dev);

unlock:
	k_mutex_unlock(&data->lock);

	if (err != 0) {
		LOG_ERR("failed to service interrupt (err %d)", err);
	}

	for (size_t i = 0; i < ARRAY_SIZE(rx_valid); i++) {
		if (rx_valid[i]) {
			mcp25625_dispatch_rx(dev, rx_regs[i]);
		}
	}

	if (tx_claimed) {
		mcp25625_complete_tx(dev, tx_callback, tx_user_data, tx_status);
	}

	if (state_cb != NULL) {
		state_cb(dev, new_state, err_cnt, state_cb_data);
	}

	return err;
}

static void mcp25625_int_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct mcp25625_config *config = dev->config;
	struct mcp25625_data *data = dev->data;
	int ret;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->int_sem, K_FOREVER);

		/*
		 * The interrupt is edge triggered, so keep servicing until the INT
		 * pin is released, otherwise no further edge would be seen.
		 */
		while (true) {
			if (mcp25625_service(dev) != 0) {
				/* Avoid monopolising the CPU on a broken SPI link */
				k_sleep(K_MSEC(1));
			}

			ret = gpio_pin_get_dt(&config->int_gpio);
			if (ret <= 0) {
				if (ret < 0) {
					LOG_ERR("failed to read INT pin (err %d)", ret);
				}
				break;
			}
		}
	}
}

static void mcp25625_int_gpio_callback(const struct device *port, struct gpio_callback *cb,
				       uint32_t pins)
{
	struct mcp25625_data *data = CONTAINER_OF(cb, struct mcp25625_data, int_gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_sem_give(&data->int_sem);
}

static int mcp25625_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | MCP25625_SUPPORTED_MODES;

	return 0;
}

static int mcp25625_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct mcp25625_config *config = dev->config;

	/* TQ = 2 * (BRP + 1) / Fosc */
	*rate = config->osc_freq / 2U;

	return 0;
}

static int mcp25625_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ide);

	return CONFIG_CAN_MCP25625_MAX_FILTERS;
}

static int mcp25625_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct mcp25625_config *config = dev->config;
	struct mcp25625_data *data = dev->data;
	uint8_t regs[3];
	int err;

	if (data->common.started) {
		return -EBUSY;
	}

	/* Additional bit timing requirements of the MCP2515/MCP25625 */
	if (timing->phase_seg2 <= timing->sjw) {
		LOG_ERR("phase segment 2 must be longer than SJW");
		return -ENOTSUP;
	}

	if (timing->prop_seg + timing->phase_seg1 < timing->phase_seg2) {
		LOG_ERR("propagation + phase segment 1 must not be shorter than phase segment 2");
		return -ENOTSUP;
	}

	/* CNF3, CNF2 and CNF1 are consecutive registers */
	regs[0] = FIELD_PREP(MCP25625_CNF3_PHSEG2_MASK, timing->phase_seg2 - 1U);
	regs[1] = MCP25625_CNF2_BTLMODE |
		  (((data->common.mode & CAN_MODE_3_SAMPLES) != 0U) ? MCP25625_CNF2_SAM : 0U) |
		  FIELD_PREP(MCP25625_CNF2_PHSEG1_MASK, timing->phase_seg1 - 1U) |
		  FIELD_PREP(MCP25625_CNF2_PRSEG_MASK, timing->prop_seg - 1U);
	regs[2] = FIELD_PREP(MCP25625_CNF1_SJW_MASK, timing->sjw - 1U) |
		  FIELD_PREP(MCP25625_CNF1_BRP_MASK, timing->prescaler - 1U);

	k_mutex_lock(&data->lock, K_FOREVER);

	err = mcp25625_write_regs(dev, MCP25625_REG_CNF3, regs, sizeof(regs));
	if (err == 0) {
		data->bitrate = (config->osc_freq / 2U) /
				(timing->prescaler *
				 (1U + timing->prop_seg + timing->phase_seg1 + timing->phase_seg2));
	} else {
		LOG_ERR("failed to write bit timing (err %d)", err);
		err = -EIO;
	}

	k_mutex_unlock(&data->lock);

	return err;
}

static int mcp25625_set_mode(const struct device *dev, can_mode_t mode)
{
	struct mcp25625_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	if ((mode & ~MCP25625_SUPPORTED_MODES) != 0U) {
		LOG_ERR("unsupported mode 0x%08x", mode);
		return -ENOTSUP;
	}

	if ((mode & CAN_MODE_LOOPBACK) != 0U && (mode & CAN_MODE_LISTENONLY) != 0U) {
		LOG_ERR("loopback and listen-only cannot be combined");
		return -ENOTSUP;
	}

	data->common.mode = mode;

	return 0;
}

static int mcp25625_start(const struct device *dev)
{
	const struct mcp25625_config *config = dev->config;
	struct mcp25625_data *data = dev->data;
	can_mode_t mode = data->common.mode;
	uint8_t opmode;
	uint8_t eflg;
	int err;

	if (data->common.started) {
		return -EALREADY;
	}

	if (config->common.phy != NULL) {
		err = can_transceiver_enable(config->common.phy, mode);
		if (err != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", err);
			return err;
		}
	}

	if ((mode & CAN_MODE_LOOPBACK) != 0U) {
		opmode = MCP25625_MODE_LOOPBACK;
	} else if ((mode & CAN_MODE_LISTENONLY) != 0U) {
		opmode = MCP25625_MODE_LISTEN_ONLY;
	} else {
		opmode = MCP25625_MODE_NORMAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	CAN_STATS_RESET(dev);
	memset(&data->stats, 0, sizeof(data->stats));

	err = mcp25625_bit_modify(dev, MCP25625_REG_CNF2, MCP25625_CNF2_SAM,
				  ((mode & CAN_MODE_3_SAMPLES) != 0U) ? MCP25625_CNF2_SAM : 0U);
	if (err != 0) {
		goto err_unlock;
	}

	err = mcp25625_apply_rx_config(dev);
	if (err != 0) {
		goto err_unlock;
	}

	/* Discard stale interrupt flags and overrun indications */
	err = mcp25625_write_reg(dev, MCP25625_REG_CANINTF, 0U);
	if (err != 0) {
		goto err_unlock;
	}

	err = mcp25625_bit_modify(dev, MCP25625_REG_EFLG,
				  MCP25625_EFLG_RX0OVR | MCP25625_EFLG_RX1OVR, 0U);
	if (err != 0) {
		goto err_unlock;
	}

	err = mcp25625_bit_modify(dev, MCP25625_REG_CANCTRL, MCP25625_CANCTRL_OSM,
				  ((mode & CAN_MODE_ONE_SHOT) != 0U) ? MCP25625_CANCTRL_OSM : 0U);
	if (err != 0) {
		goto err_unlock;
	}

	mcp25625_set_standby(dev, false);

	err = mcp25625_set_opmode(dev, opmode);
	if (err != 0) {
		mcp25625_set_standby(dev, true);
		goto err_unlock;
	}

	err = mcp25625_read_reg(dev, MCP25625_REG_EFLG, &eflg);
	data->state = (err == 0) ? mcp25625_eflg_to_state(eflg) : CAN_STATE_ERROR_ACTIVE;
	err = 0;

	data->common.started = true;
	pm_device_busy_set(dev);
	mcp25625_update_polling(dev);

	k_mutex_unlock(&data->lock);

	/* Service anything that became pending while switching modes */
	k_sem_give(&data->int_sem);

	return 0;

err_unlock:
	k_mutex_unlock(&data->lock);

	LOG_ERR("failed to start CAN controller (err %d)", err);

	if (config->common.phy != NULL) {
		(void)can_transceiver_disable(config->common.phy);
	}

	return -EIO;
}

static bool mcp25625_filters_in_use(struct mcp25625_data *data)
{
	for (size_t i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].callback != NULL) {
			return true;
		}
	}

	return false;
}

static int mcp25625_stop(const struct device *dev)
{
	const struct mcp25625_config *config = dev->config;
	struct mcp25625_data *data = dev->data;
	can_tx_callback_t tx_callback = NULL;
	void *tx_user_data = NULL;
	bool tx_claimed;
	int err;

	if (!data->common.started) {
		return -EALREADY;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Abort a pending transmission before entering configuration mode */
	err = mcp25625_bit_modify(dev, MCP25625_REG_TXB0CTRL, MCP25625_TXBCTRL_TXREQ, 0U);
	if (err == 0) {
		err = mcp25625_set_opmode(dev, MCP25625_MODE_CONFIG);
	}

	if (err != 0) {
		k_mutex_unlock(&data->lock);
		LOG_ERR("failed to enter configuration mode (err %d)", err);
		return -EIO;
	}

	data->common.started = false;
	data->state = CAN_STATE_STOPPED;
	tx_claimed = mcp25625_claim_tx(data, &tx_callback, &tx_user_data);
	mcp25625_set_standby(dev, true);
	mcp25625_update_polling(dev);

	k_mutex_unlock(&data->lock);

	if (tx_claimed) {
		mcp25625_complete_tx(dev, tx_callback, tx_user_data, -ENETDOWN);
	}

	if (config->common.phy != NULL) {
		err = can_transceiver_disable(config->common.phy);
		if (err != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", err);
			return err;
		}
	}

	k_mutex_lock(&data->filter_lock, K_FOREVER);
	if (!mcp25625_filters_in_use(data)) {
		pm_device_busy_clear(dev);
	}
	k_mutex_unlock(&data->filter_lock);

	return 0;
}

static int mcp25625_send(const struct device *dev, const struct can_frame *frame,
			 k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	struct mcp25625_data *data = dev->data;
	uint8_t regs[MCP25625_FRAME_LEN];
	size_t len;
	int err;

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %u exceeds maximum (%u)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if (data->state == CAN_STATE_BUS_OFF) {
		return -ENETUNREACH;
	}

	if (k_sem_take(&data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->common.started) {
		err = -ENETDOWN;
		goto err_unlock;
	}

	if (data->state == CAN_STATE_BUS_OFF) {
		err = -ENETUNREACH;
		goto err_unlock;
	}

	data->tx_busy = true;
	data->tx_callback = callback;
	data->tx_user_data = user_data;

	len = mcp25625_frame_to_regs(frame, regs);
	err = mcp25625_load_and_send_txb0(dev, regs, len);
	if (err != 0) {
		LOG_ERR("failed to load TX buffer (err %d)", err);
		data->tx_busy = false;
		data->tx_callback = NULL;
		data->tx_user_data = NULL;
		err = -EIO;
		goto err_unlock;
	}

	mcp25625_update_polling(dev);

	k_mutex_unlock(&data->lock);

	return 0;

err_unlock:
	k_mutex_unlock(&data->lock);
	k_sem_give(&data->tx_sem);

	return err;
}

static int mcp25625_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				  void *user_data, const struct can_filter *filter)
{
	struct mcp25625_data *data = dev->data;
	int filter_id = -ENOSPC;

	if ((filter->flags & ~CAN_FILTER_IDE) != 0U) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->filter_lock, K_FOREVER);

	for (size_t i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].callback == NULL) {
			data->filters[i].filter = *filter;
			data->filters[i].user_data = user_data;
			data->filters[i].callback = callback;
			filter_id = (int)i;
			break;
		}
	}

	if (filter_id >= 0) {
		pm_device_busy_set(dev);
	}

	k_mutex_unlock(&data->filter_lock);

	return filter_id;
}

static void mcp25625_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct mcp25625_data *data = dev->data;

	if (filter_id < 0 || filter_id >= CONFIG_CAN_MCP25625_MAX_FILTERS) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	k_mutex_lock(&data->filter_lock, K_FOREVER);

	data->filters[filter_id].callback = NULL;
	data->filters[filter_id].user_data = NULL;

	if (!data->common.started && !mcp25625_filters_in_use(data)) {
		pm_device_busy_clear(dev);
	}

	k_mutex_unlock(&data->filter_lock);
}

static int mcp25625_get_state(const struct device *dev, enum can_state *state,
			      struct can_bus_err_cnt *err_cnt)
{
	struct mcp25625_data *data = dev->data;
	uint8_t eflg;
	uint8_t cnt[2];
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);

	err = mcp25625_read_reg(dev, MCP25625_REG_EFLG, &eflg);
	if (err == 0 && err_cnt != NULL) {
		err = mcp25625_read_regs(dev, MCP25625_REG_TEC, cnt, sizeof(cnt));
	}

	if (err == 0) {
		if (state != NULL) {
			*state = data->common.started ? mcp25625_eflg_to_state(eflg)
						      : CAN_STATE_STOPPED;
		}

		if (err_cnt != NULL) {
			err_cnt->tx_err_cnt = cnt[0];
			err_cnt->rx_err_cnt = cnt[1];
		}
	}

	k_mutex_unlock(&data->lock);

	if (err != 0) {
		LOG_ERR("failed to read error state (err %d)", err);
		return -EIO;
	}

	return 0;
}

static void mcp25625_set_state_change_callback(const struct device *dev,
					       can_state_change_callback_t callback,
					       void *user_data)
{
	struct mcp25625_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->common.state_change_cb = callback;
	data->common.state_change_cb_user_data = user_data;
	k_mutex_unlock(&data->lock);
}

#ifdef CONFIG_PM_DEVICE
static int mcp25625_wake(const struct device *dev)
{
	const struct mcp25625_config *config = dev->config;
	uint8_t regs[2] = {MCP25625_INT_WAK, MCP25625_INT_WAK};
	int err;

	/*
	 * Force a wake-up interrupt (CANINTE.WAKIE and CANINTF.WAKIF are
	 * adjacent) to restart the oscillator, then return to configuration mode.
	 */
	err = mcp25625_write_regs(dev, MCP25625_REG_CANINTE, regs, sizeof(regs));
	if (err != 0) {
		return err;
	}

	k_usleep(MCP25625_OST_US);

	err = mcp25625_write_reg(dev, MCP25625_REG_CANCTRL,
				 FIELD_PREP(MCP25625_CANCTRL_REQOP_MASK, MCP25625_MODE_CONFIG) |
					 config->canctrl_clkout);
	if (err != 0) {
		return err;
	}

	err = mcp25625_set_opmode(dev, MCP25625_MODE_CONFIG);
	if (err != 0) {
		return err;
	}

	regs[0] = MCP25625_CANINTE_DEFAULT;
	regs[1] = 0U;

	return mcp25625_write_regs(dev, MCP25625_REG_CANINTE, regs, sizeof(regs));
}

static int mcp25625_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct mcp25625_data *data = dev->data;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (pm_device_is_busy(dev)) {
			return -EBUSY;
		}

		k_mutex_lock(&data->lock, K_FOREVER);
		err = mcp25625_set_opmode(dev, MCP25625_MODE_SLEEP);
		k_mutex_unlock(&data->lock);
		break;
	case PM_DEVICE_ACTION_RESUME:
		k_mutex_lock(&data->lock, K_FOREVER);
		err = mcp25625_wake(dev);
		k_mutex_unlock(&data->lock);
		break;
	default:
		return -ENOTSUP;
	}

	return (err == 0) ? 0 : -EIO;
}
#endif /* CONFIG_PM_DEVICE */

static int mcp25625_hw_reset(const struct device *dev)
{
	const struct mcp25625_config *config = dev->config;
	int64_t deadline = k_uptime_get() + MCP25625_RESET_TIMEOUT_MS;
	int64_t next_reset = 0;
	uint8_t canstat = 0U;
	uint8_t canctrl = 0U;
	int err;

	if (config->reset_gpio.port != NULL) {
		/* RESET must be held low for at least 2 us */
		gpio_pin_set_dt(&config->reset_gpio, 1);
		k_busy_wait(10);
		gpio_pin_set_dt(&config->reset_gpio, 0);
	}

	/*
	 * The oscillator may still be starting (or the device may still be held
	 * in reset by its oscillator start-up timer), so keep sending RESET until
	 * the device reports configuration mode with the CANCTRL reset value.
	 */
	while (true) {
		if (k_uptime_get() >= next_reset) {
			err = mcp25625_cmd_reset(dev);
			if (err != 0) {
				return err;
			}
			next_reset = k_uptime_get() + MCP25625_RESET_RETRY_MS;
			k_usleep(MCP25625_OST_US);
		}

		err = mcp25625_read_reg(dev, MCP25625_REG_CANSTAT, &canstat);
		if (err == 0) {
			err = mcp25625_read_reg(dev, MCP25625_REG_CANCTRL, &canctrl);
		}
		if (err != 0) {
			return err;
		}

		if (FIELD_GET(MCP25625_CANSTAT_OPMOD_MASK, canstat) == MCP25625_MODE_CONFIG &&
		    canctrl == MCP25625_CANCTRL_RESET_VALUE) {
			return 0;
		}

		if (k_uptime_get() > deadline) {
			LOG_ERR("MCP25625 not detected (CANSTAT 0x%02x, CANCTRL 0x%02x); check "
				"SPI wiring, power and oscillator",
				canstat, canctrl);
			return -ENODEV;
		}

		k_msleep(1);
	}
}

static int mcp25625_init(const struct device *dev)
{
	const struct mcp25625_config *config = dev->config;
	struct mcp25625_data *data = dev->data;
	struct can_timing timing = {0};
	k_tid_t tid;
	int err;

	data->dev = dev;
	data->state = CAN_STATE_STOPPED;
	k_mutex_init(&data->lock);
	k_mutex_init(&data->filter_lock);
	/* Start with one pending event so the thread services a pre-asserted INT */
	k_sem_init(&data->int_sem, 1, 1);
	k_sem_init(&data->tx_sem, 1, 1);
	k_work_init_delayable(&data->poll_work, mcp25625_poll_work_handler);

	if (config->common.phy != NULL && !device_is_ready(config->common.phy)) {
		LOG_ERR("CAN transceiver not ready");
		return -ENODEV;
	}

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("INT GPIO not ready");
		return -ENODEV;
	}

	if (config->standby_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->standby_gpio)) {
			LOG_ERR("STBY GPIO not ready");
			return -ENODEV;
		}

		/* Keep the transceiver in standby until the controller is started */
		err = gpio_pin_configure_dt(&config->standby_gpio, GPIO_OUTPUT_ACTIVE);
		if (err != 0) {
			LOG_ERR("failed to configure STBY GPIO (err %d)", err);
			return err;
		}
	}

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			LOG_ERR("RESET GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			LOG_ERR("failed to configure RESET GPIO (err %d)", err);
			return err;
		}
	}

	err = mcp25625_hw_reset(dev);
	if (err != 0) {
		return err;
	}

	err = mcp25625_write_reg(dev, MCP25625_REG_CANCTRL,
				 FIELD_PREP(MCP25625_CANCTRL_REQOP_MASK, MCP25625_MODE_CONFIG) |
					 config->canctrl_clkout);
	if (err == 0) {
		err = mcp25625_apply_rx_config(dev);
	}
	if (err == 0) {
		uint8_t regs[2] = {MCP25625_CANINTE_DEFAULT, 0U};

		/* CANINTE and CANINTF are adjacent registers */
		err = mcp25625_write_regs(dev, MCP25625_REG_CANINTE, regs, sizeof(regs));
	}
	if (err != 0) {
		LOG_ERR("failed to configure MCP25625 (err %d)", err);
		return -EIO;
	}

	err = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (err != 0) {
		LOG_ERR("failed to configure INT GPIO (err %d)", err);
		return err;
	}

	gpio_init_callback(&data->int_gpio_cb, mcp25625_int_gpio_callback,
			   BIT(config->int_gpio.pin));

	err = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
	if (err != 0) {
		LOG_ERR("failed to add INT GPIO callback (err %d)", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		LOG_ERR("failed to configure INT GPIO interrupt (err %d)", err);
		return err;
	}

	tid = k_thread_create(&data->int_thread, config->int_thread_stack,
			      config->int_thread_stack_size, mcp25625_int_thread, (void *)dev, NULL,
			      NULL, K_PRIO_COOP(config->int_thread_priority), 0, K_NO_WAIT);
	(void)k_thread_name_set(tid, dev->name);

	err = can_calc_timing(dev, &timing, config->common.bitrate, config->common.sample_point);
	if (err < 0) {
		LOG_ERR("no bit timing found for %u bit/s with a %u MHz oscillator (err %d)",
			config->common.bitrate, config->osc_freq / 1000000U, err);
		return -EIO;
	}

	LOG_DBG("prescaler %u, prop_seg %u, phase_seg1 %u, phase_seg2 %u, sjw %u", timing.prescaler,
		timing.prop_seg, timing.phase_seg1, timing.phase_seg2, timing.sjw);

	err = can_set_timing(dev, &timing);
	if (err != 0) {
		return err;
	}

	return can_set_mode(dev, CAN_MODE_NORMAL);
}

/* clang-format off */
static DEVICE_API(can, mcp25625_api) = {
	.get_capabilities = mcp25625_get_capabilities,
	.set_timing = mcp25625_set_timing,
	.start = mcp25625_start,
	.stop = mcp25625_stop,
	.set_mode = mcp25625_set_mode,
	.send = mcp25625_send,
	.add_rx_filter = mcp25625_add_rx_filter,
	.remove_rx_filter = mcp25625_remove_rx_filter,
	.get_state = mcp25625_get_state,
	.set_state_change_callback = mcp25625_set_state_change_callback,
	.get_core_clock = mcp25625_get_core_clock,
	.get_max_filters = mcp25625_get_max_filters,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 4,
		.prop_seg = 8,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 64,
	},
};
/* clang-format on */

static bool mcp25625_id_valid(uint32_t id, bool ide)
{
	return (id & ~(ide ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK)) == 0U;
}

int mcp25625_set_hw_acceptance(const struct device *dev, const struct mcp25625_hw_acceptance *acc)
{
	struct mcp25625_data *data = dev->data;
	int err = 0;

	if (dev->api != &mcp25625_api) {
		return -ENOTSUP;
	}

	if (acc != NULL) {
		bool valid = mcp25625_id_valid(acc->rxb0.mask, acc->rxb0.ide) &&
			     mcp25625_id_valid(acc->rxb1.mask, acc->rxb1.ide);

		for (size_t i = 0; i < MCP25625_RXB0_FILTERS; i++) {
			valid = valid && mcp25625_id_valid(acc->rxb0.id[i], acc->rxb0.ide);
		}

		for (size_t i = 0; i < MCP25625_RXB1_FILTERS; i++) {
			valid = valid && mcp25625_id_valid(acc->rxb1.id[i], acc->rxb1.ide);
		}

		if (!valid) {
			return -EINVAL;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->common.started) {
		err = -EBUSY;
		goto unlock;
	}

	if (acc != NULL) {
		data->hw_acc = *acc;
		data->hw_acc_enabled = true;
	} else {
		data->hw_acc_enabled = false;
	}

	/* The device is in configuration mode while stopped */
	if (mcp25625_apply_rx_config(dev) != 0) {
		err = -EIO;
	}

unlock:
	k_mutex_unlock(&data->lock);

	return err;
}

int mcp25625_get_stats(const struct device *dev, struct mcp25625_stats *stats)
{
	struct mcp25625_data *data = dev->data;

	if (dev->api != &mcp25625_api) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	*stats = data->stats;
	k_mutex_unlock(&data->lock);

	return 0;
}

#define MCP25625_CLKOUT_BITS(prescaler)                                                            \
	(((prescaler) == 0)   ? 0U                                                                 \
	 : ((prescaler) == 1) ? (MCP25625_CANCTRL_CLKEN | 0U)                                      \
	 : ((prescaler) == 2) ? (MCP25625_CANCTRL_CLKEN | 1U)                                      \
	 : ((prescaler) == 4) ? (MCP25625_CANCTRL_CLKEN | 2U)                                      \
			      : (MCP25625_CANCTRL_CLKEN | 3U))

#define MCP25625_INIT(inst)                                                                        \
	static K_KERNEL_STACK_DEFINE(mcp25625_int_thread_stack_##inst,                             \
				     CONFIG_CAN_MCP25625_INT_THREAD_STACK_SIZE);                   \
                                                                                                   \
	static struct mcp25625_data mcp25625_data_##inst;                                          \
                                                                                                   \
	static const struct mcp25625_config mcp25625_config_##inst = {                             \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),                         \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                                \
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                \
		.standby_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, standby_gpios, {0}),                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.osc_freq = DT_INST_PROP(inst, osc_freq),                                          \
		.canctrl_clkout = MCP25625_CLKOUT_BITS(DT_INST_PROP(inst, clkout_prescaler)),      \
		.int_thread_stack = mcp25625_int_thread_stack_##inst,                              \
		.int_thread_stack_size = K_KERNEL_STACK_SIZEOF(mcp25625_int_thread_stack_##inst),  \
		.int_thread_priority = CONFIG_CAN_MCP25625_INT_THREAD_PRIO,                        \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, mcp25625_pm_action);                                        \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, mcp25625_init, PM_DEVICE_DT_INST_GET(inst),                \
				  &mcp25625_data_##inst, &mcp25625_config_##inst, POST_KERNEL,     \
				  CONFIG_CAN_INIT_PRIORITY, &mcp25625_api);

DT_INST_FOREACH_STATUS_OKAY(MCP25625_INIT)
