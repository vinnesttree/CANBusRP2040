/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Client of the Phoenix 6 host bridge.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include <canbus_rp2040/phoenix6.h>

#include "phoenix6_proto.h"

LOG_MODULE_REGISTER(phoenix6, CONFIG_PHOENIX6_LOG_LEVEL);

/* Highest valid CAN device ID of a Talon FX */
#define PHOENIX6_MAX_DEVICE_ID 62U

#define HELLO_INTERVAL_MS          250
#define HELLO_REJECTED_INTERVAL_MS 1000
/* Longest time the client thread sleeps without received data */
#define THREAD_TICK_MS             20

enum link_state {
	LINK_DISCONNECTED,
	LINK_CONNECTED,
	LINK_REJECTED,
};

struct pending_request {
	bool used;
	bool armed;
	uint8_t type;
	uint16_t seq;
	struct k_sem done;
	int32_t status;
	double value;
};

struct signal_entry {
	bool used;
	bool valid;
	uint8_t device;
	uint16_t signal;
	float freq_hz;
	struct phoenix6_signal_value value;
};

static struct {
	const struct device *uart;
	atomic_t started;

	struct k_spinlock rx_lock;
	struct ring_buf rx_rb;
	uint8_t rx_buf[CONFIG_PHOENIX6_RX_BUF_SIZE];
	struct k_sem rx_sem;
	atomic_t rx_paused;

	struct k_mutex tx_mutex;
	struct k_spinlock tx_lock;
	struct ring_buf tx_rb;
	uint8_t tx_buf[CONFIG_PHOENIX6_TX_BUF_SIZE];
	struct k_sem tx_space_sem;
	uint16_t seq;

	atomic_t state;
	/* Only used by the client thread */
	uint32_t last_rx_ms;
	uint32_t last_hello_ms;
	uint32_t last_ping_ms;
	char host_version[32];

	struct k_mutex pending_lock;
	struct pending_request pending[CONFIG_PHOENIX6_MAX_PENDING];

	struct k_spinlock status_lock;
	int32_t control_status[PHOENIX6_MAX_DEVICE_ID + 1];

	struct k_spinlock signal_lock;
	struct signal_entry signals[CONFIG_PHOENIX6_SIGNAL_CACHE_SIZE];

	struct phoenix6_frame_decoder decoder;
	struct k_thread thread;
} p6;

static K_THREAD_STACK_DEFINE(phoenix6_stack, CONFIG_PHOENIX6_THREAD_STACK_SIZE);

static void phoenix6_uart_isr(const struct device *dev, void *user_data)
{
	k_spinlock_key_t key;
	uint8_t *ptr;
	uint32_t size;
	int n;

	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
		if (uart_irq_rx_ready(dev) > 0) {
			key = k_spin_lock(&p6.rx_lock);
			size = ring_buf_put_claim(&p6.rx_rb, &ptr, p6.rx_rb.size);
			if (size == 0U) {
				uart_irq_rx_disable(dev);
				atomic_set(&p6.rx_paused, 1);
				n = 0;
			} else {
				/* Not inside MAX(): it evaluates its argument twice */
				n = uart_fifo_read(dev, ptr, size);
				n = MAX(n, 0);
			}
			ring_buf_put_finish(&p6.rx_rb, n);
			k_spin_unlock(&p6.rx_lock, key);

			k_sem_give(&p6.rx_sem);
		}

		if (uart_irq_tx_ready(dev) > 0) {
			key = k_spin_lock(&p6.tx_lock);
			size = ring_buf_get_claim(&p6.tx_rb, &ptr, p6.tx_rb.size);
			if (size == 0U) {
				uart_irq_tx_disable(dev);
				n = 0;
			} else {
				n = uart_fifo_fill(dev, ptr, size);
				n = MAX(n, 0);
			}
			ring_buf_get_finish(&p6.tx_rb, n);
			k_spin_unlock(&p6.tx_lock, key);

			k_sem_give(&p6.tx_space_sem);

			if (size != 0U && n == 0) {
				break;
			}
		}
	}
}

/*
 * Send a packet. If @a slot is not NULL it is armed with the sequence number
 * before the packet is queued, so the answer cannot overtake it.
 */
static int phoenix6_send(uint8_t type, const uint8_t *body, size_t len,
			 struct pending_request *slot)
{
	uint8_t frame[PHOENIX6_MAX_ENCODED];
	const uint8_t *data = frame;
	k_spinlock_key_t key;
	uint16_t seq;
	size_t left;
	int n;
	int err = 0;

	k_mutex_lock(&p6.tx_mutex, K_FOREVER);

	seq = p6.seq++;
	n = phoenix6_packet_encode(type, seq, body, len, frame, sizeof(frame));
	if (n < 0) {
		k_mutex_unlock(&p6.tx_mutex);
		return n;
	}

	if (slot != NULL) {
		k_mutex_lock(&p6.pending_lock, K_FOREVER);
		slot->seq = seq;
		slot->type = type;
		slot->armed = true;
		k_mutex_unlock(&p6.pending_lock);
	}

	left = (size_t)n;
	while (left > 0U) {
		uint32_t put;

		key = k_spin_lock(&p6.tx_lock);
		put = ring_buf_put(&p6.tx_rb, data, left);
		k_spin_unlock(&p6.tx_lock, key);

		if (put > 0U) {
			uart_irq_tx_enable(p6.uart);
			data += put;
			left -= put;
			continue;
		}

		if (k_sem_take(&p6.tx_space_sem, K_MSEC(CONFIG_PHOENIX6_TX_TIMEOUT_MS)) != 0) {
			/* A partial packet is dropped by the receiver's CRC check */
			err = -EIO;
			break;
		}
	}

	k_mutex_unlock(&p6.tx_mutex);

	return err;
}

static void phoenix6_send_hello(void)
{
	uint8_t body[5];
	struct phoenix6_writer w;

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, PHOENIX6_PROTO_VERSION);
	phoenix6_put_u32(&w, PHOENIX6_SCHEMA_HASH);
	(void)phoenix6_send(PHOENIX6_MSG_HELLO, body, w.len, NULL);
	p6.last_hello_ms = k_uptime_get_32();
}

/* SUBSCRIBE without waiting for the answer */
static void send_subscribe(uint8_t device, uint16_t signal, float freq_hz)
{
	uint8_t body[7];
	struct phoenix6_writer w;

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, signal);
	phoenix6_put_f32(&w, freq_hz);
	(void)phoenix6_send(PHOENIX6_MSG_SUBSCRIBE, body, w.len, NULL);
}

/* A new session starts without subscriptions on the host: restore them */
static void restore_subscriptions(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(p6.signals); i++) {
		k_spinlock_key_t key = k_spin_lock(&p6.signal_lock);
		struct signal_entry e = p6.signals[i];

		k_spin_unlock(&p6.signal_lock, key);

		if (e.used && e.freq_hz > 0.0f) {
			send_subscribe(e.device, e.signal, e.freq_hz);
		}
	}
}

static void handle_hello_reply(struct phoenix6_reader *r)
{
	uint8_t version = phoenix6_get_u8(r);
	uint32_t hash = phoenix6_get_u32(r);
	uint8_t accepted = phoenix6_get_u8(r);
	uint8_t vlen = phoenix6_get_u8(r);

	if (r->error || r->pos + vlen > r->len) {
		return;
	}

	vlen = MIN(vlen, sizeof(p6.host_version) - 1U);
	memcpy(p6.host_version, &r->buf[r->pos], vlen);
	p6.host_version[vlen] = '\0';

	if (version == PHOENIX6_PROTO_VERSION && hash == PHOENIX6_SCHEMA_HASH && accepted != 0U) {
		if (atomic_set(&p6.state, LINK_CONNECTED) != LINK_CONNECTED) {
			LOG_INF("connected to host bridge (Phoenix 6 %s)", p6.host_version);
			restore_subscriptions();
		}
	} else {
		if (atomic_set(&p6.state, LINK_REJECTED) != LINK_REJECTED) {
			LOG_ERR("bridge rejected session: protocol %u/%u, schema 0x%08x/0x%08x",
				version, PHOENIX6_PROTO_VERSION, hash, PHOENIX6_SCHEMA_HASH);
			LOG_ERR("host has Phoenix 6 %s, firmware was built for %s", p6.host_version,
				PHOENIX6_GENERATED_VERSION);
		}
	}
}

static void handle_result(struct phoenix6_reader *r)
{
	uint16_t seq = phoenix6_get_u16(r);
	uint8_t type = phoenix6_get_u8(r);
	uint8_t device = phoenix6_get_u8(r);
	int32_t status = phoenix6_get_i32(r);
	double value = phoenix6_get_f64(r);
	k_spinlock_key_t key;

	if (r->error) {
		return;
	}

	if (type == PHOENIX6_MSG_SET_CONTROL) {
		if (device <= PHOENIX6_MAX_DEVICE_ID) {
			key = k_spin_lock(&p6.status_lock);
			p6.control_status[device] = status;
			k_spin_unlock(&p6.status_lock, key);
		}
		return;
	}

	k_mutex_lock(&p6.pending_lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(p6.pending); i++) {
		struct pending_request *slot = &p6.pending[i];

		if (slot->used && slot->armed && slot->seq == seq && slot->type == type) {
			slot->status = status;
			slot->value = value;
			slot->armed = false;
			k_sem_give(&slot->done);
			break;
		}
	}
	k_mutex_unlock(&p6.pending_lock);
}

static struct signal_entry *signal_find(uint8_t device, uint16_t signal, bool add)
{
	struct signal_entry *free_entry = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(p6.signals); i++) {
		struct signal_entry *e = &p6.signals[i];

		if (e->used && e->device == device && e->signal == signal) {
			return e;
		}

		if (!e->used && free_entry == NULL) {
			free_entry = e;
		}
	}

	if (add && free_entry != NULL) {
		memset(free_entry, 0, sizeof(*free_entry));
		free_entry->used = true;
		free_entry->device = device;
		free_entry->signal = signal;
		return free_entry;
	}

	return NULL;
}

static void handle_signals(struct phoenix6_reader *r)
{
	uint8_t count = phoenix6_get_u8(r);
	int64_t now = k_uptime_get();

	for (uint8_t i = 0; i < count && !r->error; i++) {
		uint8_t device = phoenix6_get_u8(r);
		uint16_t signal = phoenix6_get_u16(r);
		int32_t status = phoenix6_get_i32(r);
		double value = phoenix6_get_f64(r);
		float age = phoenix6_get_f32(r);
		struct signal_entry *e;
		k_spinlock_key_t key;

		if (r->error) {
			break;
		}

		/* Only subscribed signals: values sent before an unsubscribe are dropped */
		key = k_spin_lock(&p6.signal_lock);
		e = signal_find(device, signal, false);
		if (e != NULL) {
			e->valid = true;
			e->value.value = value;
			e->value.status = status;
			e->value.received_ms = now;
			e->value.age_ms = age;
		}
		k_spin_unlock(&p6.signal_lock, key);
	}
}

static void handle_log(struct phoenix6_reader *r)
{
	char text[96];
	uint8_t level = phoenix6_get_u8(r);
	size_t n;

	if (r->error) {
		return;
	}

	n = MIN(r->len - r->pos, sizeof(text) - 1U);
	memcpy(text, &r->buf[r->pos], n);
	text[n] = '\0';

	if (level == 0U) {
		LOG_ERR("bridge: %s", text);
	} else if (level == 1U) {
		LOG_WRN("bridge: %s", text);
	} else {
		LOG_INF("bridge: %s", text);
	}
}

static void handle_packet(const struct phoenix6_packet *pkt)
{
	struct phoenix6_reader r;

	phoenix6_reader_init(&r, pkt->body, pkt->body_len);
	p6.last_rx_ms = k_uptime_get_32();

	switch (pkt->type) {
	case PHOENIX6_MSG_HELLO_REPLY:
		handle_hello_reply(&r);
		break;
	case PHOENIX6_MSG_PONG:
		break;
	case PHOENIX6_MSG_RESULT:
		handle_result(&r);
		break;
	case PHOENIX6_MSG_SIGNALS:
		handle_signals(&r);
		break;
	case PHOENIX6_MSG_LOG:
		handle_log(&r);
		break;
	default:
		LOG_DBG("ignoring message type 0x%02x", pkt->type);
		break;
	}
}

/* Link supervision. Returns the time in ms until it needs to run again. */
static uint32_t phoenix6_housekeeping(void)
{
	uint32_t now = k_uptime_get_32();
	atomic_val_t state = atomic_get(&p6.state);
	uint32_t wait;

	if (state == LINK_CONNECTED && now - p6.last_rx_ms > CONFIG_PHOENIX6_LINK_TIMEOUT_MS) {
		atomic_set(&p6.state, LINK_DISCONNECTED);
		LOG_WRN("host bridge link lost");
		state = LINK_DISCONNECTED;
	}

	if (state != LINK_CONNECTED) {
		uint32_t interval =
			(state == LINK_REJECTED) ? HELLO_REJECTED_INTERVAL_MS : HELLO_INTERVAL_MS;

		if (now - p6.last_hello_ms >= interval) {
			phoenix6_send_hello();
			now = p6.last_hello_ms;
		}

		return interval - (now - p6.last_hello_ms);
	}

	/*
	 * PING regardless of other traffic: requests such as ENABLE get no
	 * answer, so the PONGs are what keeps the link alive on this side.
	 */
	if (now - p6.last_ping_ms >= CONFIG_PHOENIX6_KEEPALIVE_MS) {
		(void)phoenix6_send(PHOENIX6_MSG_PING, NULL, 0, NULL);
		p6.last_ping_ms = now;
	}

	wait = CONFIG_PHOENIX6_KEEPALIVE_MS - (now - p6.last_ping_ms);
	wait = MIN(wait, CONFIG_PHOENIX6_LINK_TIMEOUT_MS + 1U - (now - p6.last_rx_ms));

	return wait;
}

static void phoenix6_thread(void *p1, void *p2, void *p3)
{
	struct phoenix6_packet pkt;
	uint8_t buf[64];
	k_spinlock_key_t key;
	uint32_t wait = 0;
	uint32_t n;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uart_irq_rx_enable(p6.uart);

	while (true) {
		(void)k_sem_take(&p6.rx_sem, K_MSEC(CLAMP(wait, 1U, THREAD_TICK_MS)));

		do {
			key = k_spin_lock(&p6.rx_lock);
			n = ring_buf_get(&p6.rx_rb, buf, sizeof(buf));
			k_spin_unlock(&p6.rx_lock, key);

			for (uint32_t i = 0; i < n; i++) {
				if (phoenix6_frame_decoder_feed(&p6.decoder, buf[i], &pkt)) {
					handle_packet(&pkt);
				}
			}
		} while (n > 0U);

		if (atomic_cas(&p6.rx_paused, 1, 0)) {
			uart_irq_rx_enable(p6.uart);
		}

		wait = phoenix6_housekeeping();
	}
}

int phoenix6_bridge_start(const struct device *uart)
{
	k_tid_t tid;
	int err;

	if (!atomic_cas(&p6.started, 0, 1)) {
		return -EALREADY;
	}

	if (!device_is_ready(uart)) {
		atomic_clear(&p6.started);
		return -ENODEV;
	}

	p6.uart = uart;
	ring_buf_init(&p6.rx_rb, sizeof(p6.rx_buf), p6.rx_buf);
	ring_buf_init(&p6.tx_rb, sizeof(p6.tx_buf), p6.tx_buf);
	k_sem_init(&p6.rx_sem, 0, 1);
	k_sem_init(&p6.tx_space_sem, 0, 1);
	k_mutex_init(&p6.tx_mutex);
	k_mutex_init(&p6.pending_lock);
	phoenix6_frame_decoder_init(&p6.decoder);
	atomic_set(&p6.state, LINK_DISCONNECTED);

	for (size_t i = 0; i < ARRAY_SIZE(p6.pending); i++) {
		k_sem_init(&p6.pending[i].done, 0, 1);
	}

	for (size_t i = 0; i < ARRAY_SIZE(p6.control_status); i++) {
		p6.control_status[i] = PHOENIX6_STATUS_NONE;
	}

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);

	err = uart_irq_callback_user_data_set(uart, phoenix6_uart_isr, NULL);
	if (err != 0) {
		LOG_ERR("UART %s has no interrupt driven API (err %d)", uart->name, err);
		atomic_clear(&p6.started);
		return err;
	}

	tid = k_thread_create(&p6.thread, phoenix6_stack, K_THREAD_STACK_SIZEOF(phoenix6_stack),
			      phoenix6_thread, NULL, NULL, NULL, CONFIG_PHOENIX6_THREAD_PRIORITY, 0,
			      K_NO_WAIT);
	(void)k_thread_name_set(tid, "phoenix6");

	return 0;
}

bool phoenix6_bridge_is_connected(void)
{
	return atomic_get(&p6.state) == LINK_CONNECTED;
}

int phoenix6_bridge_wait_connected(k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);

	while (atomic_get(&p6.state) != LINK_CONNECTED) {
		if (sys_timepoint_expired(end)) {
			return (atomic_get(&p6.state) == LINK_REJECTED) ? -EPROTO : -EAGAIN;
		}

		k_msleep(10);
	}

	return 0;
}

const char *phoenix6_bridge_host_version(void)
{
	return p6.host_version;
}

static struct pending_request *pending_alloc(void)
{
	struct pending_request *slot = NULL;

	k_mutex_lock(&p6.pending_lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(p6.pending); i++) {
		if (!p6.pending[i].used) {
			slot = &p6.pending[i];
			slot->used = true;
			slot->armed = false;
			k_sem_reset(&slot->done);
			break;
		}
	}
	k_mutex_unlock(&p6.pending_lock);

	return slot;
}

static void pending_free(struct pending_request *slot)
{
	k_mutex_lock(&p6.pending_lock, K_FOREVER);
	slot->armed = false;
	slot->used = false;
	k_mutex_unlock(&p6.pending_lock);
}

static int phoenix6_request(uint8_t type, const uint8_t *body, size_t len, uint32_t timeout_ms,
			    int32_t *status, double *value)
{
	struct pending_request *slot;
	int err;

	if (!phoenix6_bridge_is_connected()) {
		return -ENOTCONN;
	}

	slot = pending_alloc();
	if (slot == NULL) {
		return -ENOBUFS;
	}

	err = phoenix6_send(type, body, len, slot);
	if (err == 0 &&
	    k_sem_take(&slot->done, K_MSEC(timeout_ms + CONFIG_PHOENIX6_RESPONSE_MARGIN_MS)) != 0) {
		err = -ETIMEDOUT;
	}

	if (err == 0) {
		if (status != NULL) {
			*status = slot->status;
		}
		if (value != NULL) {
			*value = slot->value;
		}
	}

	pending_free(slot);

	return err;
}

static bool device_valid(uint8_t device)
{
	return device <= PHOENIX6_MAX_DEVICE_ID;
}

int phoenix6_feed_enable(uint32_t timeout_ms)
{
	uint8_t body[2];
	struct phoenix6_writer w;

	if (!phoenix6_bridge_is_connected()) {
		return -ENOTCONN;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u16(&w, (uint16_t)MIN(timeout_ms, UINT16_MAX));

	return phoenix6_send(PHOENIX6_MSG_ENABLE, body, w.len, NULL);
}

int phoenix6_set_control(uint8_t device, const struct phoenix6_control *request)
{
	const struct phoenix6_control_desc *desc;
	uint8_t body[PHOENIX6_MAX_PACKET - 5];
	struct phoenix6_writer w;
	int err;

	if (!device_valid(device) || request == NULL) {
		return -EINVAL;
	}

	desc = phoenix6_control_desc_get(request->id);
	if (desc == NULL) {
		return -EINVAL;
	}

	if (!phoenix6_bridge_is_connected()) {
		return -ENOTCONN;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, (uint16_t)request->id);
	phoenix6_put_f64(&w, request->update_freq_hz);

	err = phoenix6_encode_control_params(
		&w, desc, (const uint8_t *)request + PHOENIX6_CONTROL_PARAMS_OFFSET);
	if (err != 0) {
		return err;
	}

	return phoenix6_send(PHOENIX6_MSG_SET_CONTROL, body, w.len, NULL);
}

int32_t phoenix6_last_control_status(uint8_t device)
{
	k_spinlock_key_t key;
	int32_t status;

	if (!device_valid(device)) {
		return PHOENIX6_STATUS_NONE;
	}

	key = k_spin_lock(&p6.status_lock);
	status = p6.control_status[device];
	k_spin_unlock(&p6.status_lock, key);

	return status;
}

static int device_timeout_request(uint8_t type, uint8_t device, uint32_t timeout_ms,
				  int32_t *status)
{
	uint8_t body[3];
	struct phoenix6_writer w;

	if (!device_valid(device)) {
		return -EINVAL;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, (uint16_t)MIN(timeout_ms, UINT16_MAX));

	return phoenix6_request(type, body, w.len, timeout_ms, status, NULL);
}

int phoenix6_config_set(uint8_t device, enum phoenix6_config_field field, double value,
			uint32_t timeout_ms, int32_t *status)
{
	uint8_t body[13];
	struct phoenix6_writer w;

	if (!device_valid(device) || field <= PHOENIX6_CONFIG_NONE ||
	    field >= PHOENIX6_CONFIG_COUNT) {
		return -EINVAL;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, (uint16_t)field);
	phoenix6_put_f64(&w, value);
	phoenix6_put_u16(&w, (uint16_t)MIN(timeout_ms, UINT16_MAX));

	return phoenix6_request(PHOENIX6_MSG_CONFIG_SET, body, w.len, timeout_ms, status, NULL);
}

int phoenix6_config_get(uint8_t device, enum phoenix6_config_field field, double *value,
			uint32_t timeout_ms, int32_t *status)
{
	uint8_t body[5];
	struct phoenix6_writer w;

	if (!device_valid(device) || field <= PHOENIX6_CONFIG_NONE ||
	    field >= PHOENIX6_CONFIG_COUNT || value == NULL) {
		return -EINVAL;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, (uint16_t)field);
	phoenix6_put_u16(&w, (uint16_t)MIN(timeout_ms, UINT16_MAX));

	return phoenix6_request(PHOENIX6_MSG_CONFIG_GET, body, w.len, timeout_ms, status, value);
}

int phoenix6_factory_default(uint8_t device, uint32_t timeout_ms, int32_t *status)
{
	return device_timeout_request(PHOENIX6_MSG_FACTORY_DEFAULT, device, timeout_ms, status);
}

int phoenix6_set_position(uint8_t device, double rotations, uint32_t timeout_ms, int32_t *status)
{
	uint8_t body[11];
	struct phoenix6_writer w;

	if (!device_valid(device)) {
		return -EINVAL;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_f64(&w, rotations);
	phoenix6_put_u16(&w, (uint16_t)MIN(timeout_ms, UINT16_MAX));

	return phoenix6_request(PHOENIX6_MSG_SET_POSITION, body, w.len, timeout_ms, status, NULL);
}

int phoenix6_clear_sticky_faults(uint8_t device, uint32_t timeout_ms, int32_t *status)
{
	return device_timeout_request(PHOENIX6_MSG_CLEAR_STICKY_FAULTS, device, timeout_ms, status);
}

int phoenix6_subscribe(uint8_t device, enum phoenix6_signal signal, float freq_hz, int32_t *status)
{
	uint8_t body[7];
	struct phoenix6_writer w;
	struct signal_entry *e;
	k_spinlock_key_t key;
	int err;

	if (!device_valid(device) || signal <= PHOENIX6_SIGNAL_NONE ||
	    signal >= PHOENIX6_SIGNAL_COUNT || !(freq_hz >= 0.0f)) {
		return -EINVAL;
	}

	key = k_spin_lock(&p6.signal_lock);
	e = signal_find(device, (uint16_t)signal, freq_hz > 0.0f);
	if (e != NULL) {
		if (freq_hz > 0.0f) {
			e->freq_hz = freq_hz;
		} else {
			e->used = false;
		}
	}
	k_spin_unlock(&p6.signal_lock, key);

	if (freq_hz > 0.0f && e == NULL) {
		return -ENOBUFS;
	}

	phoenix6_writer_init(&w, body, sizeof(body));
	phoenix6_put_u8(&w, device);
	phoenix6_put_u16(&w, (uint16_t)signal);
	phoenix6_put_f32(&w, freq_hz);

	/* Changing the update frequency of a signal on the CAN bus takes up to 100 ms */
	err = phoenix6_request(PHOENIX6_MSG_SUBSCRIBE, body, w.len, 100, status, NULL);

	return err;
}

int phoenix6_get_signal(uint8_t device, enum phoenix6_signal signal,
			struct phoenix6_signal_value *value)
{
	struct signal_entry *e;
	k_spinlock_key_t key;
	int err = -ENODATA;

	if (value == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&p6.signal_lock);
	e = signal_find(device, (uint16_t)signal, false);
	if (e != NULL && e->valid) {
		*value = e->value;
		err = 0;
	}
	k_spin_unlock(&p6.signal_lock, key);

	return err;
}
