/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI emulator for the MCP25625 CAN controller (register compatible with the
 * MCP2515). Used to test the driver on native_sim.
 */

#define DT_DRV_COMPAT microchip_mcp25625

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <canbus_rp2040/drivers/can/mcp25625_emul.h>

#include "can_mcp25625.h"

LOG_MODULE_REGISTER(can_mcp25625_emul, CONFIG_CAN_LOG_LEVEL);

#define MCP25625_EMUL_MAX_XFER   64
#define MCP25625_EMUL_TX_LOG_LEN 16

static const uint8_t txb_ctrl_addr[] = {
	MCP25625_REG_TXB0CTRL,
	MCP25625_REG_TXB1CTRL,
	MCP25625_REG_TXB2CTRL,
};

static const uint8_t filter_addr[] = {
	MCP25625_REG_RXF0SIDH, MCP25625_REG_RXF1SIDH, MCP25625_REG_RXF2SIDH,
	MCP25625_REG_RXF3SIDH, MCP25625_REG_RXF4SIDH, MCP25625_REG_RXF5SIDH,
};

struct mcp25625_emul_cfg {
	struct gpio_dt_spec int_gpio;
};

struct mcp25625_emul_data {
	struct k_mutex lock;
	uint8_t regs[MCP25625_REG_COUNT];

	bool tx_ack;
	bool tx_lose_arbitration;
	bool tx_hold;

	struct can_frame tx_log[MCP25625_EMUL_TX_LOG_LEN];
	size_t tx_log_head;
	size_t tx_log_count;

	uint32_t config_violations;
};

static uint8_t emul_opmode(const struct mcp25625_emul_data *data)
{
	return FIELD_GET(MCP25625_CANSTAT_OPMOD_MASK, data->regs[MCP25625_REG_CANSTAT]);
}

static void emul_set_opmode(struct mcp25625_emul_data *data, uint8_t opmode)
{
	data->regs[MCP25625_REG_CANSTAT] =
		(data->regs[MCP25625_REG_CANSTAT] & ~MCP25625_CANSTAT_OPMOD_MASK) |
		FIELD_PREP(MCP25625_CANSTAT_OPMOD_MASK, opmode);
}

static void emul_update_int(const struct emul *target)
{
	const struct mcp25625_emul_cfg *cfg = target->cfg;
	struct mcp25625_emul_data *data = target->data;
	bool asserted = (data->regs[MCP25625_REG_CANINTF] & data->regs[MCP25625_REG_CANINTE]) != 0U;

	/*
	 * INT is active low. Setting the input fails until the driver has
	 * configured the pin as input, which is fine: it is updated again on
	 * every SPI transaction.
	 */
	(void)gpio_emul_input_set_dt(&cfg->int_gpio, asserted ? 0 : 1);
}

static void emul_reset(struct mcp25625_emul_data *data)
{
	memset(data->regs, 0, sizeof(data->regs));
	data->regs[MCP25625_REG_CANCTRL] = MCP25625_CANCTRL_RESET_VALUE;
	emul_set_opmode(data, MCP25625_MODE_CONFIG);
}

static bool emul_is_config_only(uint8_t addr)
{
	/* Filters, masks and CNF1..CNF3 can only be written in configuration mode */
	return (addr <= 0x0BU) || (addr >= 0x10U && addr <= 0x1BU) ||
	       (addr >= 0x20U && addr <= MCP25625_REG_CNF1);
}

static bool emul_is_bit_modifiable(uint8_t addr)
{
	switch (addr) {
	case MCP25625_REG_BFPCTRL:
	case MCP25625_REG_TXRTSCTRL:
	case MCP25625_REG_CANCTRL:
	case MCP25625_REG_CNF3:
	case MCP25625_REG_CNF2:
	case MCP25625_REG_CNF1:
	case MCP25625_REG_CANINTE:
	case MCP25625_REG_CANINTF:
	case MCP25625_REG_EFLG:
	case MCP25625_REG_TXB0CTRL:
	case MCP25625_REG_TXB1CTRL:
	case MCP25625_REG_TXB2CTRL:
	case MCP25625_REG_RXB0CTRL:
	case MCP25625_REG_RXB1CTRL:
		return true;
	default:
		return false;
	}
}

static void emul_regs_to_frame(const uint8_t *regs, struct can_frame *frame)
{
	uint8_t sidl = regs[MCP25625_FRAME_SIDL];

	memset(frame, 0, sizeof(*frame));

	if ((sidl & MCP25625_SIDL_IDE) != 0U) {
		frame->flags |= CAN_FRAME_IDE;
		frame->id = ((uint32_t)regs[0] << 21) | ((uint32_t)(sidl >> 5) << 18) |
			    ((uint32_t)(sidl & 0x03U) << 16) | ((uint32_t)regs[2] << 8) | regs[3];
	} else {
		frame->id = ((uint32_t)regs[0] << 3) | (sidl >> 5);
	}

	frame->dlc = MIN(regs[MCP25625_FRAME_DLC] & MCP25625_DLC_DLC_MASK, CAN_MAX_DLC);

	/* On transmission the RTR bit is in TXBnDLC for both frame formats */
	if ((regs[MCP25625_FRAME_DLC] & MCP25625_DLC_RTR) != 0U) {
		frame->flags |= CAN_FRAME_RTR;
	} else {
		memcpy(frame->data, &regs[MCP25625_FRAME_D0], frame->dlc);
	}
}

/* Store a received frame in the RXBn register layout */
static void emul_frame_to_rx_regs(const struct can_frame *frame, uint8_t *regs)
{
	bool rtr = (frame->flags & CAN_FRAME_RTR) != 0U;

	memset(regs, 0, MCP25625_FRAME_LEN);

	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		regs[0] = (uint8_t)(frame->id >> 21);
		regs[1] = (uint8_t)((((frame->id >> 18) & 0x07U) << 5) | MCP25625_SIDL_IDE |
				    ((frame->id >> 16) & 0x03U));
		regs[2] = (uint8_t)(frame->id >> 8);
		regs[3] = (uint8_t)frame->id;
		regs[MCP25625_FRAME_DLC] = rtr ? MCP25625_DLC_RTR : 0U;
	} else {
		regs[0] = (uint8_t)(frame->id >> 3);
		regs[1] = (uint8_t)(((frame->id & 0x07U) << 5) | (rtr ? MCP25625_SIDL_SRR : 0U));
	}

	regs[MCP25625_FRAME_DLC] |= frame->dlc & MCP25625_DLC_DLC_MASK;

	if (!rtr) {
		memcpy(&regs[MCP25625_FRAME_D0], frame->data, MIN(frame->dlc, CAN_MAX_DLC));
	}
}

static bool emul_filter_match(const struct mcp25625_emul_data *data, uint8_t mask_addr,
			      uint8_t filt_addr, const struct can_frame *frame)
{
	const uint8_t *m = &data->regs[mask_addr];
	const uint8_t *f = &data->regs[filt_addr];
	bool frame_ide = (frame->flags & CAN_FRAME_IDE) != 0U;
	bool filter_ide = (f[1] & MCP25625_SIDL_IDE) != 0U;
	uint32_t mask_sid = ((uint32_t)m[0] << 3) | (m[1] >> 5);
	uint32_t filt_sid = ((uint32_t)f[0] << 3) | (f[1] >> 5);
	uint32_t mask_eid = ((uint32_t)(m[1] & 0x03U) << 16) | ((uint32_t)m[2] << 8) | m[3];
	uint32_t filt_eid = ((uint32_t)(f[1] & 0x03U) << 16) | ((uint32_t)f[2] << 8) | f[3];
	uint32_t frame_sid;

	if (frame_ide != filter_ide) {
		return false;
	}

	if (frame_ide) {
		uint32_t frame_eid = frame->id & 0x3FFFFU;

		frame_sid = frame->id >> 18;

		return ((frame_sid ^ filt_sid) & mask_sid) == 0U &&
		       ((frame_eid ^ filt_eid) & mask_eid) == 0U;
	}

	frame_sid = frame->id;
	if (((frame_sid ^ filt_sid) & mask_sid) != 0U) {
		return false;
	}

	/* For standard frames EID15..0 of the mask apply to data bytes 0 and 1 */
	if ((mask_eid & 0xFFFFU) != 0U) {
		uint32_t d = 0U;

		if (frame->dlc > 0U) {
			d |= (uint32_t)frame->data[0] << 8;
		}
		if (frame->dlc > 1U) {
			d |= frame->data[1];
		}

		return ((d ^ filt_eid) & mask_eid & 0xFFFFU) == 0U;
	}

	return true;
}

static bool emul_accept(const struct mcp25625_emul_data *data, uint8_t rxb,
			const struct can_frame *frame, uint8_t *filhit)
{
	uint8_t ctrl = data->regs[rxb == 0U ? MCP25625_REG_RXB0CTRL : MCP25625_REG_RXB1CTRL];
	uint8_t first = (rxb == 0U) ? 0U : 2U;
	uint8_t last = (rxb == 0U) ? 2U : 6U;

	if (FIELD_GET(MCP25625_RXBCTRL_RXM_MASK, ctrl) == MCP25625_RXBCTRL_RXM_ANY) {
		*filhit = first;
		return true;
	}

	for (uint8_t i = first; i < last; i++) {
		if (emul_filter_match(data,
				      rxb == 0U ? MCP25625_REG_RXM0SIDH : MCP25625_REG_RXM1SIDH,
				      filter_addr[i], frame)) {
			*filhit = i;
			return true;
		}
	}

	return false;
}

static void emul_store_rx(struct mcp25625_emul_data *data, uint8_t rxb, uint8_t filhit,
			  const struct can_frame *frame)
{
	uint8_t base = (rxb == 0U) ? MCP25625_REG_RXB0CTRL : MCP25625_REG_RXB1CTRL;
	bool rtr = (frame->flags & CAN_FRAME_RTR) != 0U;

	emul_frame_to_rx_regs(frame, &data->regs[base + 1]);

	if (rxb == 0U) {
		data->regs[base] =
			(data->regs[base] & ~(MCP25625_RXBCTRL_RXRTR | MCP25625_RXB0CTRL_FILHIT0)) |
			(rtr ? MCP25625_RXBCTRL_RXRTR : 0U) | (filhit & 0x01U);
		data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_RX0;
	} else {
		data->regs[base] = (data->regs[base] &
				    ~(MCP25625_RXBCTRL_RXRTR | MCP25625_RXB1CTRL_FILHIT_MASK)) |
				   (rtr ? MCP25625_RXBCTRL_RXRTR : 0U) | (filhit & 0x07U);
		data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_RX1;
	}
}

static void emul_rx_overflow(struct mcp25625_emul_data *data, uint8_t ovr_flag)
{
	data->regs[MCP25625_REG_EFLG] |= ovr_flag;
	data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_ERR;
}

/* Receive logic of the MCP2515 family, including RXB0 to RXB1 rollover */
static int emul_receive(struct mcp25625_emul_data *data, const struct can_frame *frame)
{
	uint8_t intf = data->regs[MCP25625_REG_CANINTF];
	uint8_t filhit;

	if (emul_accept(data, 0, frame, &filhit)) {
		if ((intf & MCP25625_INT_RX0) == 0U) {
			emul_store_rx(data, 0, filhit, frame);
			return 0;
		}

		if ((data->regs[MCP25625_REG_RXB0CTRL] & MCP25625_RXB0CTRL_BUKT) != 0U) {
			if ((intf & MCP25625_INT_RX1) == 0U) {
				emul_store_rx(data, 1, filhit, frame);
				return 0;
			}

			emul_rx_overflow(data, MCP25625_EFLG_RX1OVR);
			return -ENOBUFS;
		}

		emul_rx_overflow(data, MCP25625_EFLG_RX0OVR);
		return -ENOBUFS;
	}

	if (emul_accept(data, 1, frame, &filhit)) {
		if ((intf & MCP25625_INT_RX1) == 0U) {
			emul_store_rx(data, 1, filhit, frame);
			return 0;
		}

		emul_rx_overflow(data, MCP25625_EFLG_RX1OVR);
		return -ENOBUFS;
	}

	return -EACCES;
}

static void emul_tx_log_push(struct mcp25625_emul_data *data, const struct can_frame *frame)
{
	size_t idx = (data->tx_log_head + data->tx_log_count) % MCP25625_EMUL_TX_LOG_LEN;

	if (data->tx_log_count == MCP25625_EMUL_TX_LOG_LEN) {
		/* Drop the oldest entry */
		data->tx_log_head = (data->tx_log_head + 1U) % MCP25625_EMUL_TX_LOG_LEN;
		data->tx_log_count--;
		idx = (data->tx_log_head + data->tx_log_count) % MCP25625_EMUL_TX_LOG_LEN;
	}

	data->tx_log[idx] = *frame;
	data->tx_log_count++;
}

/* Attempt the transmission of all buffers with TXREQ set */
static void emul_process_tx(struct mcp25625_emul_data *data)
{
	uint8_t opmode = emul_opmode(data);
	bool one_shot = (data->regs[MCP25625_REG_CANCTRL] & MCP25625_CANCTRL_OSM) != 0U;

	if (opmode != MCP25625_MODE_NORMAL && opmode != MCP25625_MODE_LOOPBACK) {
		return;
	}

	if (data->tx_hold) {
		return;
	}

	if (opmode == MCP25625_MODE_NORMAL &&
	    (data->regs[MCP25625_REG_EFLG] & MCP25625_EFLG_TXBO) != 0U) {
		return;
	}

	/* Higher buffer number wins for equal priority; priorities are not modelled */
	for (int b = ARRAY_SIZE(txb_ctrl_addr) - 1; b >= 0; b--) {
		uint8_t *ctrl = &data->regs[txb_ctrl_addr[b]];
		struct can_frame frame;

		if ((*ctrl & MCP25625_TXBCTRL_TXREQ) == 0U) {
			continue;
		}

		emul_regs_to_frame(&data->regs[txb_ctrl_addr[b] + 1], &frame);

		if (opmode == MCP25625_MODE_NORMAL) {
			if (one_shot && data->tx_lose_arbitration) {
				data->tx_lose_arbitration = false;
				*ctrl = (*ctrl & ~MCP25625_TXBCTRL_TXREQ) | MCP25625_TXBCTRL_MLOA |
					MCP25625_TXBCTRL_ABTF;
				continue;
			}

			if (!data->tx_ack) {
				*ctrl |= MCP25625_TXBCTRL_TXERR;
				data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_MERR;
				if (one_shot) {
					*ctrl = (*ctrl & ~MCP25625_TXBCTRL_TXREQ) |
						MCP25625_TXBCTRL_ABTF;
				}
				continue;
			}

			emul_tx_log_push(data, &frame);
		} else {
			/* Loopback: the frame is received internally */
			(void)emul_receive(data, &frame);
		}

		*ctrl &= ~(MCP25625_TXBCTRL_TXREQ | MCP25625_TXBCTRL_TXERR | MCP25625_TXBCTRL_MLOA |
			   MCP25625_TXBCTRL_ABTF);
		data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_TX0 << b;
	}
}

static void emul_write_reg(struct mcp25625_emul_data *data, uint8_t addr, uint8_t value)
{
	uint8_t old = data->regs[addr];

	if (emul_is_config_only(addr) && emul_opmode(data) != MCP25625_MODE_CONFIG) {
		data->config_violations++;
		LOG_WRN("write to 0x%02x outside of configuration mode ignored", addr);
		return;
	}

	switch (addr) {
	case MCP25625_REG_CANSTAT:
	case MCP25625_REG_TEC:
	case MCP25625_REG_REC:
		/* Read-only */
		return;
	case MCP25625_REG_EFLG:
		/* Only the overrun flags are writable (and only cleared by the MCU) */
		data->regs[addr] = old & (value | ~(MCP25625_EFLG_RX0OVR | MCP25625_EFLG_RX1OVR));
		return;
	case MCP25625_REG_CANCTRL: {
		uint8_t reqop = FIELD_GET(MCP25625_CANCTRL_REQOP_MASK, value);

		data->regs[addr] = value & ~MCP25625_CANCTRL_ABAT;

		if ((value & MCP25625_CANCTRL_ABAT) != 0U) {
			for (size_t b = 0; b < ARRAY_SIZE(txb_ctrl_addr); b++) {
				uint8_t *ctrl = &data->regs[txb_ctrl_addr[b]];

				if ((*ctrl & MCP25625_TXBCTRL_TXREQ) != 0U) {
					*ctrl = (*ctrl & ~MCP25625_TXBCTRL_TXREQ) |
						MCP25625_TXBCTRL_ABTF;
				}
			}
		}

		/* A sleeping device only wakes up through a wake-up event */
		if (emul_opmode(data) != MCP25625_MODE_SLEEP) {
			emul_set_opmode(data, reqop);
		}

		emul_process_tx(data);
		return;
	}
	case MCP25625_REG_CANINTF:
		data->regs[addr] = value;
		if ((value & ~old & MCP25625_INT_WAK) != 0U &&
		    (data->regs[MCP25625_REG_CANINTE] & MCP25625_INT_WAK) != 0U &&
		    emul_opmode(data) == MCP25625_MODE_SLEEP) {
			/* The device wakes up into listen-only mode */
			emul_set_opmode(data, MCP25625_MODE_LISTEN_ONLY);
			data->regs[MCP25625_REG_CANCTRL] =
				(data->regs[MCP25625_REG_CANCTRL] & ~MCP25625_CANCTRL_REQOP_MASK) |
				FIELD_PREP(MCP25625_CANCTRL_REQOP_MASK, MCP25625_MODE_LISTEN_ONLY);
		}
		return;
	case MCP25625_REG_TXB0CTRL:
	case MCP25625_REG_TXB1CTRL:
	case MCP25625_REG_TXB2CTRL:
		/* ABTF, MLOA and TXERR are read-only */
		data->regs[addr] = (old & (MCP25625_TXBCTRL_ABTF | MCP25625_TXBCTRL_MLOA |
					   MCP25625_TXBCTRL_TXERR)) |
				   (value & (MCP25625_TXBCTRL_TXREQ | MCP25625_TXBCTRL_TXP_MASK));
		if ((old & MCP25625_TXBCTRL_TXREQ) != 0U &&
		    (value & MCP25625_TXBCTRL_TXREQ) == 0U) {
			data->regs[addr] |= MCP25625_TXBCTRL_ABTF;
		}
		if ((value & MCP25625_TXBCTRL_TXREQ) != 0U) {
			data->regs[addr] &= ~(MCP25625_TXBCTRL_ABTF | MCP25625_TXBCTRL_MLOA |
					      MCP25625_TXBCTRL_TXERR);
			emul_process_tx(data);
		}
		return;
	default:
		data->regs[addr] = value;
		return;
	}
}

static void emul_process(const struct emul *target, const uint8_t *in, uint8_t *out, size_t len)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t op = in[0];
	uint8_t addr;

	memset(out, 0xFF, len);

	if (op == MCP25625_OP_RESET) {
		emul_reset(data);
	} else if (op == MCP25625_OP_READ && len >= 2U) {
		addr = in[1];
		for (size_t i = 2; i < len; i++) {
			out[i] = data->regs[(addr + i - 2U) & (MCP25625_REG_COUNT - 1U)];
		}
	} else if (op == MCP25625_OP_WRITE && len >= 2U) {
		addr = in[1];
		for (size_t i = 2; i < len; i++) {
			emul_write_reg(data, (addr + i - 2U) & (MCP25625_REG_COUNT - 1U), in[i]);
		}
	} else if (op == MCP25625_OP_BIT_MODIFY && len >= 4U) {
		uint8_t mask = emul_is_bit_modifiable(in[1]) ? in[2] : 0xFFU;

		addr = in[1];
		emul_write_reg(data, addr, (data->regs[addr] & ~mask) | (in[3] & mask));
	} else if (op == MCP25625_OP_READ_STATUS) {
		uint8_t intf = data->regs[MCP25625_REG_CANINTF];
		uint8_t status = 0U;

		status |= (intf & MCP25625_INT_RX0) ? MCP25625_STATUS_RX0IF : 0U;
		status |= (intf & MCP25625_INT_RX1) ? MCP25625_STATUS_RX1IF : 0U;
		status |= (data->regs[MCP25625_REG_TXB0CTRL] & MCP25625_TXBCTRL_TXREQ)
				  ? MCP25625_STATUS_TX0REQ
				  : 0U;
		status |= (intf & MCP25625_INT_TX0) ? MCP25625_STATUS_TX0IF : 0U;
		status |= (data->regs[MCP25625_REG_TXB1CTRL] & MCP25625_TXBCTRL_TXREQ)
				  ? MCP25625_STATUS_TX1REQ
				  : 0U;
		status |= (intf & MCP25625_INT_TX1) ? MCP25625_STATUS_TX1IF : 0U;
		status |= (data->regs[MCP25625_REG_TXB2CTRL] & MCP25625_TXBCTRL_TXREQ)
				  ? MCP25625_STATUS_TX2REQ
				  : 0U;
		status |= (intf & MCP25625_INT_TX2) ? MCP25625_STATUS_TX2IF : 0U;

		for (size_t i = 1; i < len; i++) {
			out[i] = status;
		}
	} else if (op == MCP25625_OP_RX_STATUS) {
		uint8_t intf = data->regs[MCP25625_REG_CANINTF];
		uint8_t status = (intf & (MCP25625_INT_RX0 | MCP25625_INT_RX1)) << 6;

		for (size_t i = 1; i < len; i++) {
			out[i] = status;
		}
	} else if ((op & 0xF9U) == MCP25625_OP_READ_RX_BUFFER) {
		uint8_t n = (op >> 2) & 0x01U;
		uint8_t m = (op >> 1) & 0x01U;
		uint8_t start = (n == 0U ? MCP25625_REG_RXB0CTRL : MCP25625_REG_RXB1CTRL) +
				(m == 0U ? 1U : 1U + MCP25625_FRAME_D0);

		for (size_t i = 1; i < len; i++) {
			out[i] = data->regs[(start + i - 1U) & (MCP25625_REG_COUNT - 1U)];
		}

		/* RXnIF is cleared when CS is raised at the end of the instruction */
		data->regs[MCP25625_REG_CANINTF] &=
			~(n == 0U ? MCP25625_INT_RX0 : MCP25625_INT_RX1);
	} else if ((op & 0xF8U) == MCP25625_OP_LOAD_TX_BUFFER && (op & 0x07U) <= 5U) {
		uint8_t abc = op & 0x07U;
		uint8_t start =
			txb_ctrl_addr[abc / 2U] + ((abc % 2U) == 0U ? 1U : 1U + MCP25625_FRAME_D0);

		for (size_t i = 1; i < len; i++) {
			data->regs[(start + i - 1U) & (MCP25625_REG_COUNT - 1U)] = in[i];
		}
	} else if ((op & 0xF8U) == MCP25625_OP_RTS) {
		for (size_t b = 0; b < ARRAY_SIZE(txb_ctrl_addr); b++) {
			if ((op & BIT(b)) != 0U) {
				emul_write_reg(data, txb_ctrl_addr[b],
					       data->regs[txb_ctrl_addr[b]] |
						       MCP25625_TXBCTRL_TXREQ);
			}
		}
	} else {
		LOG_WRN("unknown SPI instruction 0x%02x", op);
	}
}

static int mcp25625_emul_io(const struct emul *target, const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t in[MCP25625_EMUL_MAX_XFER] = {0};
	uint8_t out[MCP25625_EMUL_MAX_XFER];
	size_t len = 0;
	size_t pos;

	ARG_UNUSED(config);

	if (tx_bufs == NULL) {
		return -EIO;
	}

	/* Flatten the transmit buffers into one transaction */
	for (size_t i = 0; i < tx_bufs->count; i++) {
		const struct spi_buf *buf = &tx_bufs->buffers[i];

		if (len + buf->len > sizeof(in)) {
			return -EIO;
		}
		if (buf->buf != NULL) {
			memcpy(&in[len], buf->buf, buf->len);
		}
		len += buf->len;
	}

	if (len == 0U) {
		return -EIO;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	emul_process(target, in, out, len);
	emul_update_int(target);
	k_mutex_unlock(&data->lock);

	if (rx_bufs != NULL) {
		pos = 0;
		for (size_t i = 0; i < rx_bufs->count && pos < len; i++) {
			const struct spi_buf *buf = &rx_bufs->buffers[i];
			size_t n = MIN(buf->len, len - pos);

			if (buf->buf != NULL) {
				memcpy(buf->buf, &out[pos], n);
			}
			pos += buf->len;
		}
	}

	return 0;
}

int mcp25625_emul_rx_frame(const struct emul *target, const struct can_frame *frame)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t opmode;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	opmode = emul_opmode(data);
	if (opmode == MCP25625_MODE_NORMAL || opmode == MCP25625_MODE_LISTEN_ONLY) {
		ret = emul_receive(data, frame);
	} else {
		ret = -EACCES;
	}

	emul_update_int(target);
	k_mutex_unlock(&data->lock);

	return ret;
}

int mcp25625_emul_get_tx_frame(const struct emul *target, struct can_frame *frame)
{
	struct mcp25625_emul_data *data = target->data;
	int ret = -ENOMSG;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->tx_log_count > 0U) {
		*frame = data->tx_log[data->tx_log_head];
		data->tx_log_head = (data->tx_log_head + 1U) % MCP25625_EMUL_TX_LOG_LEN;
		data->tx_log_count--;
		ret = 0;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

void mcp25625_emul_clear_tx_frames(const struct emul *target)
{
	struct mcp25625_emul_data *data = target->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->tx_log_head = 0U;
	data->tx_log_count = 0U;
	k_mutex_unlock(&data->lock);
}

void mcp25625_emul_set_tx_ack(const struct emul *target, bool ack)
{
	struct mcp25625_emul_data *data = target->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->tx_ack = ack;
	k_mutex_unlock(&data->lock);
}

void mcp25625_emul_set_tx_lose_arbitration(const struct emul *target, bool lose)
{
	struct mcp25625_emul_data *data = target->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->tx_lose_arbitration = lose;
	k_mutex_unlock(&data->lock);
}

void mcp25625_emul_set_tx_hold(const struct emul *target, bool hold)
{
	struct mcp25625_emul_data *data = target->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->tx_hold = hold;
	if (!hold) {
		emul_process_tx(data);
	}
	emul_update_int(target);
	k_mutex_unlock(&data->lock);
}

void mcp25625_emul_set_error_counters(const struct emul *target, uint16_t tec, uint8_t rec)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t old;
	uint8_t eflg = 0U;

	k_mutex_lock(&data->lock, K_FOREVER);

	old = data->regs[MCP25625_REG_EFLG];

	if (tec >= 96U) {
		eflg |= MCP25625_EFLG_TXWAR;
	}
	if (rec >= 96U) {
		eflg |= MCP25625_EFLG_RXWAR;
	}
	if ((eflg & (MCP25625_EFLG_TXWAR | MCP25625_EFLG_RXWAR)) != 0U) {
		eflg |= MCP25625_EFLG_EWARN;
	}
	if (tec >= 128U) {
		eflg |= MCP25625_EFLG_TXEP;
	}
	if (rec >= 128U) {
		eflg |= MCP25625_EFLG_RXEP;
	}
	if (tec > 255U) {
		eflg |= MCP25625_EFLG_TXBO;
	}

	data->regs[MCP25625_REG_TEC] = (uint8_t)MIN(tec, 255U);
	data->regs[MCP25625_REG_REC] = rec;
	data->regs[MCP25625_REG_EFLG] =
		(old & (MCP25625_EFLG_RX0OVR | MCP25625_EFLG_RX1OVR)) | eflg;

	if ((eflg & ~old) != 0U) {
		data->regs[MCP25625_REG_CANINTF] |= MCP25625_INT_ERR;
	}

	if ((eflg & MCP25625_EFLG_TXBO) == 0U) {
		/* Leaving bus-off allows pending transmissions to proceed */
		emul_process_tx(data);
	}

	emul_update_int(target);
	k_mutex_unlock(&data->lock);
}

uint8_t mcp25625_emul_get_reg(const struct emul *target, uint8_t addr)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t value;

	k_mutex_lock(&data->lock, K_FOREVER);
	value = data->regs[addr & (MCP25625_REG_COUNT - 1U)];
	k_mutex_unlock(&data->lock);

	return value;
}

uint8_t mcp25625_emul_get_opmode(const struct emul *target)
{
	struct mcp25625_emul_data *data = target->data;
	uint8_t opmode;

	k_mutex_lock(&data->lock, K_FOREVER);
	opmode = emul_opmode(data);
	k_mutex_unlock(&data->lock);

	return opmode;
}

uint32_t mcp25625_emul_get_config_violations(const struct emul *target)
{
	struct mcp25625_emul_data *data = target->data;
	uint32_t count;

	k_mutex_lock(&data->lock, K_FOREVER);
	count = data->config_violations;
	k_mutex_unlock(&data->lock);

	return count;
}

static int mcp25625_emul_init(const struct emul *target, const struct device *parent)
{
	struct mcp25625_emul_data *data = target->data;

	ARG_UNUSED(parent);

	k_mutex_init(&data->lock);
	data->tx_ack = true;
	emul_reset(data);

	return 0;
}

static struct spi_emul_api mcp25625_emul_spi_api = {
	.io = mcp25625_emul_io,
};

#define MCP25625_EMUL(n)                                                                           \
	static struct mcp25625_emul_data mcp25625_emul_data_##n;                                   \
	static const struct mcp25625_emul_cfg mcp25625_emul_cfg_##n = {                            \
		.int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),                                   \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, mcp25625_emul_init, &mcp25625_emul_data_##n,                        \
			    &mcp25625_emul_cfg_##n, &mcp25625_emul_spi_api, NULL)

DT_INST_FOREACH_STATUS_OKAY(MCP25625_EMUL)
