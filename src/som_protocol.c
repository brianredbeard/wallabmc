/*
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "som_protocol.h"

LOG_MODULE_REGISTER(som_protocol, LOG_LEVEL_INF);

#define UART_NODE DT_NODELABEL(uart4)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

static struct k_sem cmd_sem;
static struct k_sem cmd_done;
static struct som_message cmd_reply;
static uint32_t cmd_pending_id;

static som_notify_cb_t notify_cb;
static bool som_alive;

static __nocache uint8_t rx_buf[sizeof(struct som_message)];
static size_t rx_pos;

K_THREAD_STACK_DEFINE(som_rx_stack, CONFIG_SOM_PROTOCOL_STACK_SIZE);
static struct k_thread som_rx_thread_data;

static uint8_t som_checksum(const struct som_message *msg)
{
	uint8_t cs = 0;

	cs ^= msg->msg_type;
	cs ^= msg->cmd_type;
	cs ^= msg->data_len;
	for (int i = 0; i < msg->data_len; i++) {
		cs ^= msg->data[i];
	}
	return cs;
}

static void handle_rx_message(const struct som_message *msg)
{
	if (msg->header != SOM_FRAME_HEADER || msg->tail != SOM_FRAME_TAIL) {
		LOG_WRN("Invalid frame header/tail");
		return;
	}

	if (som_checksum(msg) != msg->checksum) {
		LOG_WRN("Checksum mismatch");
		return;
	}

	if (msg->msg_type == SOM_MSG_REPLY) {
		if (msg->task_id == cmd_pending_id) {
			memcpy(&cmd_reply, msg, sizeof(cmd_reply));
			k_sem_give(&cmd_done);
		} else {
			LOG_WRN("Reply for unknown task_id 0x%08x", msg->task_id);
		}
	} else if (msg->msg_type == SOM_MSG_NOTIFY) {
		LOG_INF("SOM notification: cmd=0x%02x", msg->cmd_type);
		if (notify_cb) {
			notify_cb(msg->cmd_type);
		}
	} else {
		LOG_WRN("Unknown message type: 0x%02x", msg->msg_type);
	}
}

static void som_rx_thread(void *a, void *b, void *c)
{
	uint8_t byte;
	const uint8_t hdr_bytes[] = {
		(SOM_FRAME_HEADER >>  0) & 0xFF,
		(SOM_FRAME_HEADER >>  8) & 0xFF,
		(SOM_FRAME_HEADER >> 16) & 0xFF,
		(SOM_FRAME_HEADER >> 24) & 0xFF,
	};
	int hdr_match = 0;

	LOG_INF("SOM protocol RX thread started");

	while (1) {
		if (uart_poll_in(uart_dev, &byte) < 0) {
			k_msleep(1);
			continue;
		}

		if (rx_pos == 0) {
			if (byte == hdr_bytes[hdr_match]) {
				rx_buf[hdr_match] = byte;
				hdr_match++;
				if (hdr_match == 4) {
					rx_pos = 4;
					hdr_match = 0;
				}
			} else {
				hdr_match = 0;
			}
			continue;
		}

		rx_buf[rx_pos++] = byte;

		if (rx_pos >= sizeof(struct som_message)) {
			handle_rx_message((const struct som_message *)rx_buf);
			rx_pos = 0;
		}
	}
}

int som_cmd(uint8_t cmd, void *data, size_t data_len, uint32_t timeout)
{
	struct som_message msg = {0};
	int ret;

	k_sem_take(&cmd_sem, K_FOREVER);

	msg.header = SOM_FRAME_HEADER;
	msg.task_id = k_uptime_get_32();
	msg.msg_type = SOM_MSG_REQUEST;
	msg.cmd_type = cmd;
	msg.data_len = data_len;
	if (data && data_len > 0) {
		memcpy(msg.data, data, MIN(data_len, SOM_FRAME_DATA_MAX));
	}
	msg.checksum = som_checksum(&msg);
	msg.tail = SOM_FRAME_TAIL;

	cmd_pending_id = msg.task_id;
	k_sem_reset(&cmd_done);

	const uint8_t *tx = (const uint8_t *)&msg;

	for (size_t i = 0; i < sizeof(msg); i++) {
		uart_poll_out(uart_dev, tx[i]);
	}

	ret = k_sem_take(&cmd_done, K_MSEC(timeout));
	if (ret == -EAGAIN) {
		LOG_WRN("SOM command 0x%02x timed out", cmd);
		k_sem_give(&cmd_sem);
		return -ETIMEDOUT;
	}

	if (cmd_reply.cmd_result != 0) {
		LOG_WRN("SOM command 0x%02x failed: result=%d", cmd, cmd_reply.cmd_result);
		k_sem_give(&cmd_sem);
		return -EIO;
	}

	if (data && data_len > 0 && cmd_reply.data_len > 0) {
		memcpy(data, cmd_reply.data, MIN(data_len, cmd_reply.data_len));
	}

	k_sem_give(&cmd_sem);
	return 0;
}

bool som_is_alive(void)
{
	return som_alive;
}

void som_set_alive(bool alive)
{
	som_alive = alive;
}

void som_set_notify_callback(som_notify_cb_t cb)
{
	notify_cb = cb;
}

int som_protocol_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART4 device not ready");
		return -ENODEV;
	}

	k_sem_init(&cmd_sem, 1, 1);
	k_sem_init(&cmd_done, 0, 1);
	rx_pos = 0;

	k_thread_create(&som_rx_thread_data, som_rx_stack,
			K_THREAD_STACK_SIZEOF(som_rx_stack),
			som_rx_thread,
			NULL, NULL, NULL,
			CONFIG_SOM_PROTOCOL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&som_rx_thread_data, "som_protocol");

	LOG_INF("SOM protocol initialized (UART4)");

	return 0;
}
