/*
 * Copyright (c) 2026 CANBusRP2040 contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART / USB CDC ACM transport for the slcan protocol engine.
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include <canbus_rp2040/slcan.h>

LOG_MODULE_DECLARE(slcan, CONFIG_SLCAN_LOG_LEVEL);

struct slcan_uart {
	const struct device *uart;
	struct slcan slcan;

	struct k_spinlock rx_lock;
	struct ring_buf rx_rb;
	uint8_t rx_buf[CONFIG_SLCAN_UART_RX_BUF_SIZE];
	struct k_sem rx_sem;
	atomic_t rx_paused;

	struct k_spinlock tx_lock;
	struct ring_buf tx_rb;
	uint8_t tx_buf[CONFIG_SLCAN_UART_TX_BUF_SIZE];
	struct k_sem tx_space_sem;
	uint32_t tx_dropped;

	struct k_thread thread;
	atomic_t started;
};

static struct slcan_uart slcan_uarts[CONFIG_SLCAN_UART_INSTANCES];

static K_THREAD_STACK_ARRAY_DEFINE(slcan_uart_stacks, CONFIG_SLCAN_UART_INSTANCES,
				   CONFIG_SLCAN_UART_THREAD_STACK_SIZE);

static K_MUTEX_DEFINE(slcan_uart_lock);

static void slcan_uart_isr(const struct device *dev, void *user_data)
{
	struct slcan_uart *su = user_data;
	k_spinlock_key_t key;
	uint8_t *ptr;
	uint32_t size;
	int n;

	while (uart_irq_update(dev) > 0 && uart_irq_is_pending(dev) > 0) {
		if (uart_irq_rx_ready(dev) > 0) {
			key = k_spin_lock(&su->rx_lock);
			size = ring_buf_put_claim(&su->rx_rb, &ptr, su->rx_rb.size);
			if (size == 0U) {
				/*
				 * The engine is behind: stop reading so the UART (or the
				 * USB host via CDC ACM flow control) is held off.
				 */
				uart_irq_rx_disable(dev);
				atomic_set(&su->rx_paused, 1);
				n = 0;
			} else {
				n = uart_fifo_read(dev, ptr, size);
				n = MAX(n, 0);
			}
			ring_buf_put_finish(&su->rx_rb, n);
			k_spin_unlock(&su->rx_lock, key);

			k_sem_give(&su->rx_sem);
		}

		if (uart_irq_tx_ready(dev) > 0) {
			key = k_spin_lock(&su->tx_lock);
			size = ring_buf_get_claim(&su->tx_rb, &ptr, su->tx_rb.size);
			if (size == 0U) {
				uart_irq_tx_disable(dev);
				n = 0;
			} else {
				n = uart_fifo_fill(dev, ptr, size);
				n = MAX(n, 0);
			}
			ring_buf_get_finish(&su->tx_rb, n);
			k_spin_unlock(&su->tx_lock, key);

			k_sem_give(&su->tx_space_sem);

			if (size != 0U && n == 0) {
				/* The UART accepted nothing; wait for the next interrupt */
				break;
			}
		}
	}
}

static void slcan_uart_write(const uint8_t *data, size_t len, void *user_data)
{
	struct slcan_uart *su = user_data;
	k_spinlock_key_t key;
	uint32_t n;

	while (len > 0U) {
		key = k_spin_lock(&su->tx_lock);
		n = ring_buf_put(&su->tx_rb, data, len);
		k_spin_unlock(&su->tx_lock, key);

		if (n > 0U) {
			uart_irq_tx_enable(su->uart);
			data += n;
			len -= n;
			continue;
		}

		/* Buffer full: wait for the UART to drain it, but never forever */
		if (k_sem_take(&su->tx_space_sem, K_MSEC(CONFIG_SLCAN_UART_TX_TIMEOUT_MS)) != 0) {
			su->tx_dropped += len;
			LOG_WRN_ONCE("host not reading, dropping output");
			return;
		}
	}
}

static void slcan_uart_thread(void *p1, void *p2, void *p3)
{
	struct slcan_uart *su = p1;
	struct k_poll_event events[3];
	uint8_t buf[64];
	k_spinlock_key_t key;
	uint32_t n;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_poll_event_init(&events[0], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
			  &su->rx_sem);
	slcan_poll_events(&su->slcan, &events[1]);

	uart_irq_rx_enable(su->uart);

	while (true) {
		(void)k_poll(events, ARRAY_SIZE(events), K_FOREVER);

		if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
			(void)k_sem_take(&su->rx_sem, K_NO_WAIT);
		}

		do {
			key = k_spin_lock(&su->rx_lock);
			n = ring_buf_get(&su->rx_rb, buf, sizeof(buf));
			k_spin_unlock(&su->rx_lock, key);

			if (n > 0U) {
				slcan_receive(&su->slcan, buf, n);
			}
		} while (n > 0U);

		if (atomic_cas(&su->rx_paused, 1, 0)) {
			uart_irq_rx_enable(su->uart);
		}

		(void)slcan_process(&su->slcan);

		for (size_t i = 0; i < ARRAY_SIZE(events); i++) {
			events[i].state = K_POLL_STATE_NOT_READY;
		}
	}
}

int slcan_uart_start(const struct device *uart, const struct device *can, slcan_event_cb_t cb,
		     void *user_data)
{
	struct slcan_uart *su = NULL;
	char name[16];
	size_t idx = 0;
	k_tid_t tid;
	int err;

	k_mutex_lock(&slcan_uart_lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(slcan_uarts); i++) {
		if (atomic_get(&slcan_uarts[i].started) && slcan_uarts[i].uart == uart) {
			k_mutex_unlock(&slcan_uart_lock);
			return -EALREADY;
		}

		if (su == NULL && !atomic_get(&slcan_uarts[i].started)) {
			su = &slcan_uarts[i];
			idx = i;
		}
	}

	if (su == NULL) {
		k_mutex_unlock(&slcan_uart_lock);
		LOG_ERR("all %d instances in use (CONFIG_SLCAN_UART_INSTANCES)",
			CONFIG_SLCAN_UART_INSTANCES);
		return -ENOMEM;
	}

	atomic_set(&su->started, 1);
	su->uart = uart;
	k_mutex_unlock(&slcan_uart_lock);

	if (!device_is_ready(uart)) {
		LOG_ERR("UART %s not ready", uart->name);
		atomic_clear(&su->started);
		return -ENODEV;
	}

	ring_buf_init(&su->rx_rb, sizeof(su->rx_buf), su->rx_buf);
	ring_buf_init(&su->tx_rb, sizeof(su->tx_buf), su->tx_buf);
	k_sem_init(&su->rx_sem, 0, 1);
	k_sem_init(&su->tx_space_sem, 0, 1);

	err = slcan_init(&su->slcan, can, slcan_uart_write, su);
	if (err != 0) {
		LOG_ERR("failed to initialise slcan (err %d)", err);
		atomic_clear(&su->started);
		return err;
	}

	slcan_set_event_callback(&su->slcan, cb, user_data);

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);

	err = uart_irq_callback_user_data_set(uart, slcan_uart_isr, su);
	if (err != 0) {
		LOG_ERR("UART %s has no interrupt driven API (err %d)", uart->name, err);
		atomic_clear(&su->started);
		return err;
	}

	tid = k_thread_create(&su->thread, slcan_uart_stacks[idx],
			      K_THREAD_STACK_SIZEOF(slcan_uart_stacks[idx]), slcan_uart_thread, su,
			      NULL, NULL, CONFIG_SLCAN_UART_THREAD_PRIORITY, 0, K_NO_WAIT);
	snprintk(name, sizeof(name), "slcan_uart%u", (unsigned int)idx);
	(void)k_thread_name_set(tid, name);

	return 0;
}
