/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register map of the Microchip MCP25625. The CAN controller part of the
 * MCP25625 is register compatible with the MCP2515.
 */

#ifndef CANBUS_RP2040_DRIVERS_CAN_MCP25625_H_
#define CANBUS_RP2040_DRIVERS_CAN_MCP25625_H_

#include <zephyr/sys/util.h>

/* SPI instructions */
#define MCP25625_OP_WRITE          0x02
#define MCP25625_OP_READ           0x03
#define MCP25625_OP_BIT_MODIFY     0x05
#define MCP25625_OP_LOAD_TX_BUFFER 0x40 /* | abc */
#define MCP25625_OP_RTS            0x80 /* | nnn */
#define MCP25625_OP_READ_RX_BUFFER 0x90 /* | n << 2 | m << 1 */
#define MCP25625_OP_READ_STATUS    0xA0
#define MCP25625_OP_RX_STATUS      0xB0
#define MCP25625_OP_RESET          0xC0

/* Register addresses */
#define MCP25625_REG_RXF0SIDH  0x00
#define MCP25625_REG_RXF1SIDH  0x04
#define MCP25625_REG_RXF2SIDH  0x08
#define MCP25625_REG_BFPCTRL   0x0C
#define MCP25625_REG_TXRTSCTRL 0x0D
#define MCP25625_REG_CANSTAT   0x0E
#define MCP25625_REG_CANCTRL   0x0F
#define MCP25625_REG_RXF3SIDH  0x10
#define MCP25625_REG_RXF4SIDH  0x14
#define MCP25625_REG_RXF5SIDH  0x18
#define MCP25625_REG_TEC       0x1C
#define MCP25625_REG_REC       0x1D
#define MCP25625_REG_RXM0SIDH  0x20
#define MCP25625_REG_RXM1SIDH  0x24
#define MCP25625_REG_CNF3      0x28
#define MCP25625_REG_CNF2      0x29
#define MCP25625_REG_CNF1      0x2A
#define MCP25625_REG_CANINTE   0x2B
#define MCP25625_REG_CANINTF   0x2C
#define MCP25625_REG_EFLG      0x2D
#define MCP25625_REG_TXB0CTRL  0x30
#define MCP25625_REG_TXB1CTRL  0x40
#define MCP25625_REG_TXB2CTRL  0x50
#define MCP25625_REG_RXB0CTRL  0x60
#define MCP25625_REG_RXB1CTRL  0x70

#define MCP25625_REG_COUNT 0x80

/* CANCTRL */
#define MCP25625_CANCTRL_REQOP_MASK  GENMASK(7, 5)
#define MCP25625_CANCTRL_ABAT        BIT(4)
#define MCP25625_CANCTRL_OSM         BIT(3)
#define MCP25625_CANCTRL_CLKEN       BIT(2)
#define MCP25625_CANCTRL_CLKPRE_MASK GENMASK(1, 0)
/* CANCTRL value after reset: configuration mode, CLKOUT enabled, /8 */
#define MCP25625_CANCTRL_RESET_VALUE 0x87

/* CANSTAT */
#define MCP25625_CANSTAT_OPMOD_MASK GENMASK(7, 5)
#define MCP25625_CANSTAT_ICOD_MASK  GENMASK(3, 1)

/* Operation modes (REQOP/OPMOD field values) */
#define MCP25625_MODE_NORMAL      0x0
#define MCP25625_MODE_SLEEP       0x1
#define MCP25625_MODE_LOOPBACK    0x2
#define MCP25625_MODE_LISTEN_ONLY 0x3
#define MCP25625_MODE_CONFIG      0x4

/* CNF1 */
#define MCP25625_CNF1_SJW_MASK GENMASK(7, 6)
#define MCP25625_CNF1_BRP_MASK GENMASK(5, 0)

/* CNF2 */
#define MCP25625_CNF2_BTLMODE     BIT(7)
#define MCP25625_CNF2_SAM         BIT(6)
#define MCP25625_CNF2_PHSEG1_MASK GENMASK(5, 3)
#define MCP25625_CNF2_PRSEG_MASK  GENMASK(2, 0)

/* CNF3 */
#define MCP25625_CNF3_SOF         BIT(7)
#define MCP25625_CNF3_WAKFIL      BIT(6)
#define MCP25625_CNF3_PHSEG2_MASK GENMASK(2, 0)

/* CANINTE / CANINTF */
#define MCP25625_INT_RX0  BIT(0)
#define MCP25625_INT_RX1  BIT(1)
#define MCP25625_INT_TX0  BIT(2)
#define MCP25625_INT_TX1  BIT(3)
#define MCP25625_INT_TX2  BIT(4)
#define MCP25625_INT_ERR  BIT(5)
#define MCP25625_INT_WAK  BIT(6)
#define MCP25625_INT_MERR BIT(7)

/* EFLG */
#define MCP25625_EFLG_EWARN  BIT(0)
#define MCP25625_EFLG_RXWAR  BIT(1)
#define MCP25625_EFLG_TXWAR  BIT(2)
#define MCP25625_EFLG_RXEP   BIT(3)
#define MCP25625_EFLG_TXEP   BIT(4)
#define MCP25625_EFLG_TXBO   BIT(5)
#define MCP25625_EFLG_RX0OVR BIT(6)
#define MCP25625_EFLG_RX1OVR BIT(7)

/* TXBnCTRL */
#define MCP25625_TXBCTRL_ABTF     BIT(6)
#define MCP25625_TXBCTRL_MLOA     BIT(5)
#define MCP25625_TXBCTRL_TXERR    BIT(4)
#define MCP25625_TXBCTRL_TXREQ    BIT(3)
#define MCP25625_TXBCTRL_TXP_MASK GENMASK(1, 0)

/* RXB0CTRL / RXB1CTRL */
#define MCP25625_RXBCTRL_RXM_MASK     GENMASK(6, 5)
#define MCP25625_RXBCTRL_RXM_FILTER   0x0 /* receive messages matching the filters */
#define MCP25625_RXBCTRL_RXM_ANY      0x3 /* masks and filters off */
#define MCP25625_RXBCTRL_RXRTR        BIT(3)
#define MCP25625_RXB0CTRL_BUKT        BIT(2)
#define MCP25625_RXB0CTRL_FILHIT0     BIT(0)
#define MCP25625_RXB1CTRL_FILHIT_MASK GENMASK(2, 0)

/* READ STATUS instruction result */
#define MCP25625_STATUS_RX0IF  BIT(0)
#define MCP25625_STATUS_RX1IF  BIT(1)
#define MCP25625_STATUS_TX0REQ BIT(2)
#define MCP25625_STATUS_TX0IF  BIT(3)
#define MCP25625_STATUS_TX1REQ BIT(4)
#define MCP25625_STATUS_TX1IF  BIT(5)
#define MCP25625_STATUS_TX2REQ BIT(6)
#define MCP25625_STATUS_TX2IF  BIT(7)

/*
 * Layout of a frame in a TX/RX buffer, starting at TXBnSIDH/RXBnSIDH:
 * SIDH, SIDL, EID8, EID0, DLC, D0..D7
 */
#define MCP25625_FRAME_SIDH 0
#define MCP25625_FRAME_SIDL 1
#define MCP25625_FRAME_EID8 2
#define MCP25625_FRAME_EID0 3
#define MCP25625_FRAME_DLC  4
#define MCP25625_FRAME_D0   5
#define MCP25625_FRAME_LEN  13

/* SIDL bits */
#define MCP25625_SIDL_SID_MASK GENMASK(7, 5)
#define MCP25625_SIDL_SRR      BIT(4) /* RX only: standard frame remote request */
#define MCP25625_SIDL_IDE      BIT(3) /* RX: IDE, TX/filter: EXIDE */
#define MCP25625_SIDL_EID_MASK GENMASK(1, 0)

/* DLC register bits */
#define MCP25625_DLC_RTR      BIT(6)
#define MCP25625_DLC_DLC_MASK GENMASK(3, 0)

/*
 * Oscillator start-up timer: the device is held in reset for 128 OSC1 cycles
 * after the oscillator starts. At the slowest supported oscillator (1 MHz)
 * that is 128 us.
 */
#define MCP25625_OST_US 128U

#endif /* CANBUS_RP2040_DRIVERS_CAN_MCP25625_H_ */
