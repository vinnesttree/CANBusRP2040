/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * slcan (Lawicel / CANable) protocol engine.
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_HWINFO
#include <zephyr/drivers/hwinfo.h>
#endif

#include <canbus_rp2040/slcan.h>

LOG_MODULE_REGISTER(slcan, CONFIG_SLCAN_LOG_LEVEL);

/* Bits in struct slcan::pending */
#define SLCAN_PENDING_STATE   0
#define SLCAN_PENDING_OVERRUN 1

/* Lawicel timestamps wrap around after one minute */
#define SLCAN_TIMESTAMP_PERIOD_MS 60000U

/* SJA1000 CAN clock with the 16 MHz crystal used by Lawicel adapters */
#define SLCAN_SJA1000_CLOCK 8000000U

/* Bitrates selected by S0..S8 (Lawicel, also used by the Linux slcan driver) */
static const uint32_t slcan_bitrates[] = {
	10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000,
};

static const char hex_chars[] = "0123456789ABCDEF";

static int hex_value(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}

	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}

	return -1;
}

static int parse_hex(const char *str, size_t len, uint32_t *value)
{
	uint32_t v = 0U;

	for (size_t i = 0; i < len; i++) {
		int d = hex_value(str[i]);

		if (d < 0) {
			return -EINVAL;
		}

		v = (v << 4) | (uint32_t)d;
	}

	*value = v;

	return 0;
}

static void put_hex(char *buf, uint32_t value, size_t digits)
{
	for (size_t i = 0; i < digits; i++) {
		buf[digits - 1U - i] = hex_chars[value & 0x0FU];
		value >>= 4;
	}
}

int slcan_format_frame(const struct can_frame *frame, bool timestamp, uint16_t ts_ms, char *buf,
		       size_t size)
{
	bool ide = (frame->flags & CAN_FRAME_IDE) != 0U;
	bool rtr = (frame->flags & CAN_FRAME_RTR) != 0U;
	size_t id_len = ide ? 8U : 3U;
	size_t data_len = rtr ? 0U : frame->dlc;
	size_t len = 1U + id_len + 1U + 2U * data_len + (timestamp ? 4U : 0U) + 1U;
	size_t pos = 0;

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0U || frame->dlc > CAN_MAX_DLC) {
		return -ENOTSUP;
	}

	if (size < len + 1U) {
		return -ENOSPC;
	}

	if (rtr) {
		buf[pos++] = ide ? 'R' : 'r';
	} else {
		buf[pos++] = ide ? 'T' : 't';
	}

	put_hex(&buf[pos], frame->id, id_len);
	pos += id_len;

	buf[pos++] = hex_chars[frame->dlc];

	for (size_t i = 0; i < data_len; i++) {
		put_hex(&buf[pos], frame->data[i], 2);
		pos += 2U;
	}

	if (timestamp) {
		put_hex(&buf[pos], ts_ms, 4);
		pos += 4U;
	}

	buf[pos++] = '\r';
	buf[pos] = '\0';

	return (int)pos;
}

int slcan_parse_frame(const char *cmd, size_t len, struct can_frame *frame)
{
	size_t id_len;
	uint32_t id;
	uint32_t byte;
	size_t pos;
	int dlc;

	if (len < 1U) {
		return -EINVAL;
	}

	memset(frame, 0, sizeof(*frame));

	switch (cmd[0]) {
	case 't':
		id_len = 3U;
		break;
	case 'T':
		id_len = 8U;
		frame->flags = CAN_FRAME_IDE;
		break;
	case 'r':
		id_len = 3U;
		frame->flags = CAN_FRAME_RTR;
		break;
	case 'R':
		id_len = 8U;
		frame->flags = CAN_FRAME_IDE | CAN_FRAME_RTR;
		break;
	default:
		return -EINVAL;
	}

	if (len < 1U + id_len + 1U) {
		return -EINVAL;
	}

	if (parse_hex(&cmd[1], id_len, &id) != 0) {
		return -EINVAL;
	}

	if (id > (((frame->flags & CAN_FRAME_IDE) != 0U) ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK)) {
		return -EINVAL;
	}

	frame->id = id;

	dlc = hex_value(cmd[1U + id_len]);
	if (dlc < 0 || dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}

	frame->dlc = (uint8_t)dlc;
	pos = 2U + id_len;

	if ((frame->flags & CAN_FRAME_RTR) != 0U) {
		/* Remote frames carry no data */
		return (len == pos) ? 0 : -EINVAL;
	}

	if (len != pos + 2U * (size_t)dlc) {
		return -EINVAL;
	}

	for (int i = 0; i < dlc; i++) {
		if (parse_hex(&cmd[pos], 2, &byte) != 0) {
			return -EINVAL;
		}

		frame->data[i] = (uint8_t)byte;
		pos += 2U;
	}

	return 0;
}

int slcan_btr_to_bitrate(uint8_t btr0, uint8_t btr1, uint32_t *bitrate, uint16_t *sample_point)
{
	/* BTR0: SJW[7:6] BRP[5:0], BTR1: SAM[7] TSEG2[6:4] TSEG1[3:0] */
	uint32_t brp = (btr0 & 0x3FU) + 1U;
	uint32_t tseg1 = (btr1 & 0x0FU) + 1U;
	uint32_t tseg2 = ((btr1 >> 4) & 0x07U) + 1U;
	uint32_t total = 1U + tseg1 + tseg2;

	/* Only bit timings resulting in an integer bitrate can be reproduced */
	if ((SLCAN_SJA1000_CLOCK % (brp * total)) != 0U) {
		return -EINVAL;
	}

	*bitrate = SLCAN_SJA1000_CLOCK / (brp * total);
	*sample_point = (uint16_t)(((1U + tseg1) * 1000U) / total);

	return 0;
}

static void slcan_write(struct slcan *s, const char *str, size_t len)
{
	s->write((const uint8_t *)str, len, s->write_user_data);
}

static void slcan_ok(struct slcan *s)
{
	if (IS_ENABLED(CONFIG_SLCAN_CMD_ACK)) {
		slcan_write(s, "\r", 1);
	}
}

static void slcan_error(struct slcan *s)
{
	if (IS_ENABLED(CONFIG_SLCAN_CMD_ACK)) {
		slcan_write(s, "\a", 1);
	}
}

static void slcan_event(struct slcan *s, enum slcan_event event)
{
	if (s->event_cb != NULL) {
		s->event_cb(event, s->event_user_data);
	}
}

static void slcan_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct slcan *s = user_data;
	struct slcan_rx_item item = {
		.frame = *frame,
		.timestamp_ms = k_uptime_get_32(),
	};

	ARG_UNUSED(dev);

	if (k_msgq_put(&s->rx_msgq, &item, K_NO_WAIT) != 0) {
		atomic_or(&s->status, SLCAN_STATUS_RX_FULL | SLCAN_STATUS_DATA_OVERRUN);
		atomic_set_bit(&s->pending, SLCAN_PENDING_OVERRUN);
		k_poll_signal_raise(&s->signal, 0);
	}
}

static void slcan_tx_callback(const struct device *dev, int error, void *user_data)
{
	struct slcan *s = user_data;

	ARG_UNUSED(dev);

	if (error == -EBUSY) {
		atomic_or(&s->status, SLCAN_STATUS_ARB_LOST);
	} else if (error != 0 && error != -ENETDOWN) {
		atomic_or(&s->status, SLCAN_STATUS_BUS_ERROR);
	}
}

static void slcan_state_callback(const struct device *dev, enum can_state state,
				 struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct slcan *s = user_data;
	k_spinlock_key_t key;

	ARG_UNUSED(dev);

	key = k_spin_lock(&s->lock);
	s->state = state;
	s->err_cnt = err_cnt;
	k_spin_unlock(&s->lock, key);

	if (state == CAN_STATE_ERROR_WARNING) {
		atomic_or(&s->status, SLCAN_STATUS_ERR_WARNING);
	} else if (state == CAN_STATE_ERROR_PASSIVE) {
		atomic_or(&s->status, SLCAN_STATUS_ERR_PASSIVE);
	} else if (state == CAN_STATE_BUS_OFF) {
		atomic_or(&s->status, SLCAN_STATUS_BUS_ERROR);
	}

	atomic_set_bit(&s->pending, SLCAN_PENDING_STATE);
	k_poll_signal_raise(&s->signal, 0);
}

static void slcan_cmd_open(struct slcan *s, bool listen_only)
{
	can_mode_t mode = listen_only ? CAN_MODE_LISTENONLY : CAN_MODE_NORMAL;
	struct can_timing timing;
	k_spinlock_key_t key;
	int err;

	if (s->open) {
		slcan_error(s);
		return;
	}

	if (s->bitrate != 0U) {
		if (s->sample_point != 0U) {
			err = can_calc_timing(s->can, &timing, s->bitrate, s->sample_point);
			err = (err < 0) ? err : can_set_timing(s->can, &timing);
		} else {
			err = can_set_bitrate(s->can, s->bitrate);
		}

		if (err != 0) {
			LOG_ERR("bitrate %u not supported (err %d)", s->bitrate, err);
			slcan_error(s);
			return;
		}
	}

	if (!s->auto_retransmit) {
		mode |= CAN_MODE_ONE_SHOT;
	}

	err = can_set_mode(s->can, mode);
	if (err != 0) {
		LOG_ERR("CAN mode 0x%x not supported (err %d)", mode, err);
		slcan_error(s);
		return;
	}

	k_msgq_purge(&s->rx_msgq);
	atomic_clear(&s->status);
	atomic_clear(&s->pending);

	key = k_spin_lock(&s->lock);
	s->state = CAN_STATE_ERROR_ACTIVE;
	s->err_cnt = (struct can_bus_err_cnt){0};
	k_spin_unlock(&s->lock, key);

	err = can_start(s->can);
	if (err != 0) {
		LOG_ERR("failed to start CAN controller (err %d)", err);
		slcan_error(s);
		return;
	}

	s->open = true;
	s->listen_only = listen_only;
	slcan_ok(s);
	slcan_event(s, SLCAN_EVENT_OPEN);
}

static void slcan_cmd_close(struct slcan *s)
{
	if (s->open) {
		(void)can_stop(s->can);
		s->open = false;
		k_msgq_purge(&s->rx_msgq);
		slcan_event(s, SLCAN_EVENT_CLOSE);
	}

	/* Closing a closed channel is accepted (Linux sends C before S) */
	slcan_ok(s);
}

static void slcan_cmd_bitrate(struct slcan *s, const char *cmd, size_t len)
{
	int idx;

	if (s->open || len != 2U) {
		slcan_error(s);
		return;
	}

	idx = hex_value(cmd[1]);
	if (idx < 0 || idx >= (int)ARRAY_SIZE(slcan_bitrates)) {
		slcan_error(s);
		return;
	}

	s->bitrate = slcan_bitrates[idx];
	s->sample_point = 0U;
	slcan_ok(s);
}

static void slcan_cmd_btr(struct slcan *s, const char *cmd, size_t len)
{
	uint32_t btr;
	uint32_t bitrate;
	uint16_t sample_point;

	if (s->open || len != 5U || parse_hex(&cmd[1], 4, &btr) != 0 ||
	    slcan_btr_to_bitrate((uint8_t)(btr >> 8), (uint8_t)btr, &bitrate, &sample_point) != 0) {
		slcan_error(s);
		return;
	}

	s->bitrate = bitrate;
	s->sample_point = sample_point;
	slcan_ok(s);
}

static void slcan_cmd_mode_or_acceptance(struct slcan *s, const char *cmd, size_t len)
{
	uint32_t value;

	if (s->open) {
		slcan_error(s);
		return;
	}

	if (len == 2U) {
		/* CANable: M0 normal, M1 silent. Silent mode is applied by O. */
		if (cmd[1] == '0') {
			s->silent = false;
		} else if (cmd[1] == '1') {
			s->silent = true;
		} else {
			slcan_error(s);
			return;
		}

		slcan_ok(s);
		return;
	}

	if (len != 9U || parse_hex(&cmd[1], 8, &value) != 0) {
		slcan_error(s);
		return;
	}

	/*
	 * Lawicel acceptance code (M) and mask (m) program the SJA1000 filter.
	 * Only "accept everything" (mask FFFFFFFF, any code) is supported.
	 */
	if (cmd[0] == 'm' && value != 0xFFFFFFFFU) {
		LOG_WRN("acceptance mask 0x%08x not supported", value);
		slcan_error(s);
		return;
	}

	slcan_ok(s);
}

static void slcan_cmd_flag(struct slcan *s, const char *cmd, size_t len, bool *flag)
{
	if (s->open || len != 2U || (cmd[1] != '0' && cmd[1] != '1')) {
		slcan_error(s);
		return;
	}

	*flag = (cmd[1] == '1');
	slcan_ok(s);
}

static void slcan_cmd_version(struct slcan *s)
{
	char buf[8];
	int len;

	len = snprintf(buf, sizeof(buf), "V%02u%02u\r", (unsigned int)CONFIG_SLCAN_HW_VERSION,
		       (unsigned int)CONFIG_SLCAN_SW_VERSION);
	slcan_write(s, buf, len);
}

static void slcan_cmd_serial(struct slcan *s)
{
	uint16_t serial = 0U;
	char buf[8];

#ifdef CONFIG_HWINFO
	uint8_t id[16];
	ssize_t n = hwinfo_get_device_id(id, sizeof(id));

	if (n >= 2) {
		serial = sys_get_be16(&id[n - 2]);
	}
#endif

	buf[0] = 'N';
	put_hex(&buf[1], serial, 4);
	buf[5] = '\r';
	slcan_write(s, buf, 6);
}

static void slcan_cmd_status(struct slcan *s)
{
	uint8_t flags;
	enum can_state state;
	k_spinlock_key_t key;
	char buf[4];

	if (!s->open) {
		slcan_error(s);
		return;
	}

	key = k_spin_lock(&s->lock);
	state = s->state;
	k_spin_unlock(&s->lock, key);

	flags = (uint8_t)atomic_clear(&s->status);

	/* Report the current error state in addition to latched events */
	if (state == CAN_STATE_ERROR_WARNING) {
		flags |= SLCAN_STATUS_ERR_WARNING;
	} else if (state == CAN_STATE_ERROR_PASSIVE) {
		flags |= SLCAN_STATUS_ERR_PASSIVE;
	} else if (state == CAN_STATE_BUS_OFF) {
		flags |= SLCAN_STATUS_BUS_ERROR;
	}

	buf[0] = 'F';
	put_hex(&buf[1], flags, 2);
	buf[3] = '\r';
	slcan_write(s, buf, sizeof(buf));
}

static void slcan_cmd_transmit(struct slcan *s, const char *cmd, size_t len)
{
	struct can_frame frame;
	int err;

	if (!s->open || s->listen_only || slcan_parse_frame(cmd, len, &frame) != 0) {
		slcan_error(s);
		return;
	}

	err = can_send(s->can, &frame, K_MSEC(CONFIG_SLCAN_TX_TIMEOUT_MS), slcan_tx_callback, s);
	if (err != 0) {
		if (err == -EAGAIN) {
			atomic_or(&s->status, SLCAN_STATUS_TX_FULL);
		} else if (err == -ENETUNREACH) {
			atomic_or(&s->status, SLCAN_STATUS_BUS_ERROR);
		}

		LOG_DBG("failed to send frame (err %d)", err);
		slcan_error(s);
		return;
	}

	if (IS_ENABLED(CONFIG_SLCAN_TX_ACK)) {
		slcan_write(s, ((frame.flags & CAN_FRAME_IDE) != 0U) ? "Z\r" : "z\r", 2);
	} else {
		slcan_ok(s);
	}

	slcan_event(s, SLCAN_EVENT_TX);
}

static void slcan_execute(struct slcan *s, const char *cmd, size_t len)
{
	if (len == 0U) {
		/* Empty lines are used by hosts to flush the line; ignore them */
		return;
	}

	switch (cmd[0]) {
	case 'O':
		if (len == 1U) {
			slcan_cmd_open(s, s->silent);
			return;
		}
		break;
	case 'L':
		if (len == 1U) {
			slcan_cmd_open(s, true);
			return;
		}
		break;
	case 'C':
		if (len == 1U) {
			slcan_cmd_close(s);
			return;
		}
		break;
	case 'S':
		slcan_cmd_bitrate(s, cmd, len);
		return;
	case 's':
		slcan_cmd_btr(s, cmd, len);
		return;
	case 'M':
	case 'm':
		slcan_cmd_mode_or_acceptance(s, cmd, len);
		return;
	case 'A':
	case 'a':
		slcan_cmd_flag(s, cmd, len, &s->auto_retransmit);
		return;
	case 'Z':
		slcan_cmd_flag(s, cmd, len, &s->timestamps);
		return;
	case 'V':
		if (len == 1U) {
			slcan_cmd_version(s);
			return;
		}
		break;
	case 'N':
		if (len == 1U) {
			slcan_cmd_serial(s);
			return;
		}
		break;
	case 'F':
		if (len == 1U) {
			slcan_cmd_status(s);
			return;
		}
		break;
	case 't':
	case 'T':
	case 'r':
	case 'R':
		slcan_cmd_transmit(s, cmd, len);
		return;
	default:
		break;
	}

	slcan_error(s);
}

void slcan_receive(struct slcan *s, const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		char c = (char)data[i];

		if (c == '\r') {
			if (s->line_overflow) {
				slcan_error(s);
			} else {
				slcan_execute(s, s->line, s->line_len);
			}

			s->line_len = 0U;
			s->line_overflow = false;
		} else if (c == '\n') {
			continue;
		} else if (s->line_len < sizeof(s->line)) {
			s->line[s->line_len++] = c;
		} else {
			s->line_overflow = true;
		}
	}
}

static char slcan_state_char(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return 'a';
	case CAN_STATE_ERROR_WARNING:
		return 'w';
	case CAN_STATE_ERROR_PASSIVE:
		return 'p';
	case CAN_STATE_BUS_OFF:
		return 'b';
	default:
		return '\0';
	}
}

int slcan_process(struct slcan *s)
{
	struct slcan_rx_item item;
	char buf[SLCAN_MTU + 1];
	int count = 0;
	int len;

	/* Reset first so a signal raised while processing is not lost */
	k_poll_signal_reset(&s->signal);

	if (atomic_test_and_clear_bit(&s->pending, SLCAN_PENDING_STATE)) {
		enum can_state state;
		struct can_bus_err_cnt err_cnt;
		k_spinlock_key_t key = k_spin_lock(&s->lock);

		state = s->state;
		err_cnt = s->err_cnt;
		k_spin_unlock(&s->lock, key);

		if (s->open && slcan_state_char(state) != '\0') {
			if (IS_ENABLED(CONFIG_SLCAN_STATE_REPORTS)) {
				/* Format parsed by Linux: s<state><rx errors><tx errors> */
				len = snprintf(buf, sizeof(buf), "s%c%03u%03u\r",
					       slcan_state_char(state),
					       (unsigned int)err_cnt.rx_err_cnt,
					       (unsigned int)err_cnt.tx_err_cnt);
				slcan_write(s, buf, len);
			}

			if (state != CAN_STATE_ERROR_ACTIVE) {
				slcan_event(s, SLCAN_EVENT_ERROR);
			}
		}
	}

	if (atomic_test_and_clear_bit(&s->pending, SLCAN_PENDING_OVERRUN) && s->open) {
		if (IS_ENABLED(CONFIG_SLCAN_ERROR_REPORTS)) {
			/* Error message parsed by Linux: one error, 'o' = RX overrun */
			slcan_write(s, "e1o\r", 4);
		}

		slcan_event(s, SLCAN_EVENT_ERROR);
	}

	while (k_msgq_get(&s->rx_msgq, &item, K_NO_WAIT) == 0) {
		if (!s->open) {
			continue;
		}

		len = slcan_format_frame(&item.frame, s->timestamps,
					 (uint16_t)(item.timestamp_ms % SLCAN_TIMESTAMP_PERIOD_MS),
					 buf, sizeof(buf));
		if (len < 0) {
			/* CAN FD frames cannot be represented */
			continue;
		}

		slcan_write(s, buf, len);
		slcan_event(s, SLCAN_EVENT_RX);
		count++;
	}

	return count;
}

void slcan_poll_events(struct slcan *s, struct k_poll_event events[2])
{
	k_poll_event_init(&events[0], K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
			  &s->rx_msgq);
	k_poll_event_init(&events[1], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &s->signal);
}

bool slcan_is_open(const struct slcan *s)
{
	return s->open;
}

void slcan_set_event_callback(struct slcan *s, slcan_event_cb_t cb, void *user_data)
{
	s->event_cb = cb;
	s->event_user_data = user_data;
}

int slcan_init(struct slcan *s, const struct device *can, slcan_write_t write, void *user_data)
{
	const struct can_filter std_all = {.id = 0U, .mask = 0U, .flags = 0U};
	const struct can_filter ext_all = {.id = 0U, .mask = 0U, .flags = CAN_FILTER_IDE};
	int err;

	memset(s, 0, sizeof(*s));
	s->can = can;
	s->write = write;
	s->write_user_data = user_data;
	s->auto_retransmit = true;
	s->state = CAN_STATE_STOPPED;

	k_msgq_init(&s->rx_msgq, s->rx_msgq_buf, sizeof(struct slcan_rx_item),
		    CONFIG_SLCAN_RX_QUEUE_SIZE);
	k_poll_signal_init(&s->signal);

	if (!device_is_ready(can)) {
		LOG_ERR("CAN controller %s not ready", can->name);
		return -ENODEV;
	}

	err = can_stop(can);
	if (err != 0 && err != -EALREADY) {
		return err;
	}

	err = can_add_rx_filter(can, slcan_rx_callback, s, &std_all);
	if (err < 0) {
		return err;
	}

	err = can_add_rx_filter(can, slcan_rx_callback, s, &ext_all);
	if (err < 0) {
		return err;
	}

	can_set_state_change_callback(can, slcan_state_callback, s);

	return 0;
}
