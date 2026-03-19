# P550 Feature Parity Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Port missing hardware management features from the vendor BMC firmware (pio-hifive-mcu) to WallaBMC, bringing the P550 board support to full feature parity.

**Architecture:** Each feature is a self-contained Zephyr module (`.c`/`.h` pair) following the existing wallabmc pattern: init function called from `main.c`, Kconfig gating, shell commands via `SHELL_CMD_REGISTER`, and DTS-driven hardware binding. Features build on each other in dependency order — SOM protocol is foundational, power sequencing builds on it, and sensor/monitoring features layer on top.

**Tech Stack:** Zephyr RTOS, Zephyr device tree (DTS), Kconfig, Zephyr UART async API, Zephyr GPIO, Zephyr I2C, Zephyr PWM, Zephyr sensor API, Zephyr shell

**Reference documents:**
- `boards/sifive/hifive_premier_p550_mcu/doc/hardware.rst` — Complete pin map, DTS snippets, I2C addresses
- `pio-hifive-mcu/src/hf_protocol_process.c` — Vendor UART4 protocol implementation
- `pio-hifive-mcu/src/hf_power_process.c` — Vendor power sequencing state machine
- `pio-hifive-mcu/src/hf_i2c.c` — Vendor I2C/sensor drivers
- `pio-hifive-mcu/include/hf_common.h` — Vendor protocol structures and constants

---

## Dependency Graph

```
Task 1: SOM Protocol (UART4)
  |
  +---> Task 2: SOM Keepalive Daemon
  |       |
  |       +---> Task 7: PVT (temp/fan from SOM)
  |
  +---> Task 9: Graceful Shutdown Timers
  |
  +---> Task 3: Power Sequencing State Machine
          |
          +---> Task 6: Power Monitoring (INA226) [also needs Task 5]

Task 4: Boot Select Control (independent)

Task 5: I2C EEPROM Board Identity (independent)
  |
  +---> Task 6: Power Monitoring (INA226)

Task 8: Fan PWM Control (independent)

Task 10: Factory Reset Button (independent)
```

**Recommended execution order:** 4, 10, 5, 8, 1, 2, 9, 3, 7, 6

Start with the independent tasks (4, 10, 5, 8) which have no dependencies and can be built/tested incrementally, then tackle the protocol chain (1 → 2 → 9 → 3 → 7 → 6).

---

## Task 1: SOM-MCU Communication Protocol (UART4)

**Goal:** Implement a framed message protocol over UART4 for bidirectional communication between the BMC and the SOM CPU, matching the vendor firmware's UART4 protocol.

**Background:** The vendor firmware (`hf_protocol_process.c`) uses UART4 to exchange framed messages with the SOM. Each message has a 4-byte header (`0xA55AAA55`), task ID, message type, command type, result, data payload (up to 250 bytes), XOR checksum, and 4-byte tail (`0xBDBABDBA`). The MCU sends requests; the SOM replies or sends notifications (e.g., "shutdown complete"). This is the backbone for coordinated power management, sensor reading, and SOM health monitoring.

**Files:**
- Create: `src/som_protocol.c`
- Create: `src/som_protocol.h`
- Modify: `CMakeLists.txt` — add `target_sources_ifdef` for new Kconfig
- Modify: `Kconfig` — add `SOM_PROTOCOL` config block
- Modify: `src/main.c` — add `som_protocol_init()` call
- Reference: `boards/sifive/hifive_premier_p550_mcu/doc/hardware.rst:88-89` (UART4 pins PC10/PC11, already in DTS)

**Step 1: Add Kconfig option**

Add to `Kconfig` (before `source "Kconfig.zephyr"`):

```kconfig
config SOM_PROTOCOL
	bool "SOM-MCU UART4 communication protocol"
	default y
	depends on $(dt_node_has_status,uart4,okay)
	select SERIAL
	select UART_ASYNC_API
	select SERIAL_SUPPORT_ASYNC

if SOM_PROTOCOL

config SOM_PROTOCOL_STACK_SIZE
	int "SOM protocol thread stack size"
	default 2048

config SOM_PROTOCOL_PRIORITY
	int "SOM protocol thread priority"
	default 5

config SOM_PROTOCOL_TX_TIMEOUT_MS
	int "SOM command response timeout (ms)"
	default 2000

endif # SOM_PROTOCOL
```

**Step 2: Add source to CMakeLists.txt**

Add after the existing `target_sources_ifdef` blocks:

```cmake
target_sources_ifdef(CONFIG_SOM_PROTOCOL app PRIVATE
	src/som_protocol.c
)
```

**Step 3: Create `src/som_protocol.h`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __SOM_PROTOCOL_H__
#define __SOM_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * SOM-MCU UART4 framed message protocol.
 *
 * Frame format (packed, little-endian):
 *   header(4) | task_id(4) | msg_type(1) | cmd_type(1) | cmd_result(1) |
 *   data_len(1) | data(0..250) | checksum(1) | tail(4)
 *
 * Matches the vendor firmware (pio-hifive-mcu hf_protocol_process.c).
 */

#define SOM_FRAME_HEADER	0xA55AAA55
#define SOM_FRAME_TAIL		0xBDBABDBA
#define SOM_FRAME_DATA_MAX	250

/* Message types */
enum som_msg_type {
	SOM_MSG_REQUEST  = 0x01,
	SOM_MSG_REPLY    = 0x02,
	SOM_MSG_NOTIFY   = 0x03,
};

/* Command types — must match SOM-side daemon */
enum som_cmd_type {
	SOM_CMD_POWER_OFF    = 0x01,
	SOM_CMD_REBOOT       = 0x02,
	SOM_CMD_BOARD_INFO   = 0x03,
	SOM_CMD_CONTROL_LED  = 0x04,
	SOM_CMD_PVT_INFO     = 0x05,
	SOM_CMD_BOARD_STATUS = 0x06,
	SOM_CMD_POWER_INFO   = 0x07,
	SOM_CMD_RESTART      = 0x08, /* cold reboot: power off then on */
};

struct som_message {
	uint32_t header;
	uint32_t task_id;
	uint8_t  msg_type;
	uint8_t  cmd_type;
	uint8_t  cmd_result;
	uint8_t  data_len;
	uint8_t  data[SOM_FRAME_DATA_MAX];
	uint8_t  checksum;
	uint32_t tail;
} __packed;

/* PVT (Process/Voltage/Temperature) info from SOM */
struct som_pvt_info {
	int32_t cpu_temp;   /* Celsius (integer part) */
	int32_t npu_temp;   /* Celsius (integer part) */
	int32_t fan_speed;  /* RPM, -1 if unavailable */
} __packed;

/* Power info from SOM */
struct som_power_info {
	uint32_t consumption; /* mW */
	uint32_t current;     /* mA */
	uint32_t voltage;     /* mV */
} __packed;

/**
 * Initialize the SOM protocol layer. Starts the RX thread.
 * Call after UART4 is available (after kernel init).
 */
#ifdef CONFIG_SOM_PROTOCOL
int som_protocol_init(void);
#else
static inline int som_protocol_init(void) { return 0; }
#endif

/**
 * Send a command to the SOM and wait for a reply.
 *
 * @param cmd       Command type (enum som_cmd_type)
 * @param data      Buffer for response data (may be NULL if data_len == 0)
 * @param data_len  Size of data buffer
 * @param timeout   Timeout in milliseconds
 * @return 0 on success, -ETIMEDOUT on timeout, -EIO on protocol error
 */
int som_cmd(uint8_t cmd, void *data, size_t data_len, uint32_t timeout);

/**
 * Check if SOM communication is established (keepalive responding).
 */
bool som_is_alive(void);

/**
 * Register a callback for SOM notifications (e.g., shutdown complete).
 * Only one callback supported. Set to NULL to deregister.
 */
typedef void (*som_notify_cb_t)(uint8_t cmd_type);
void som_set_notify_callback(som_notify_cb_t cb);

#endif /* __SOM_PROTOCOL_H__ */
```

**Step 4: Create `src/som_protocol.c`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
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

/*
 * Pending command: only one outstanding command at a time.
 * Protected by cmd_sem (binary semaphore as mutex) and
 * signaled by cmd_done (given when reply arrives).
 */
static struct k_sem cmd_sem;    /* serialize command sends */
static struct k_sem cmd_done;   /* signal reply received */
static struct som_message cmd_reply;
static uint32_t cmd_pending_id;

static som_notify_cb_t notify_cb;
static bool som_alive;

/* RX state machine */
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

/*
 * UART RX thread: reads bytes into a frame-sized buffer.
 * Scans for the 4-byte header, then reads the rest of the frame.
 * This is simple and robust — the vendor firmware uses a similar approach.
 */
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
		/* Blocking poll read, one byte at a time */
		if (uart_poll_in(uart_dev, &byte) < 0) {
			k_msleep(1);
			continue;
		}

		if (rx_pos == 0) {
			/* Scanning for header */
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

	/* Build request frame */
	msg.header = SOM_FRAME_HEADER;
	msg.task_id = k_uptime_get_32(); /* unique-ish ID */
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

	/* TX: blocking poll send */
	const uint8_t *tx = (const uint8_t *)&msg;

	for (size_t i = 0; i < sizeof(msg); i++) {
		uart_poll_out(uart_dev, tx[i]);
	}

	/* Wait for reply */
	ret = k_sem_take(&cmd_done, K_MSEC(timeout));
	if (ret == -EAGAIN) {
		LOG_WRN("SOM command 0x%02x timed out", cmd);
		k_sem_give(&cmd_sem);
		return -ETIMEDOUT;
	}

	/* Copy response data back to caller */
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
```

**Step 5: Add init call to `src/main.c`**

Add `#include "som_protocol.h"` to the includes, then add after the console bridge WS init block:

```c
	LOG_DBG("SOM protocol init");
	if (som_protocol_init() < 0) {
		LOG_ERR("SOM protocol init failed");
		/* Continue — non-fatal, SOM communication won't work */
	}
```

**Step 6: Build verification**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc -- -DCONFIG_SOM_PROTOCOL=y
```

Expected: Build succeeds. No hardware test possible without SOM-side daemon, but the code compiles and links.

**Step 7: Commit**

```bash
git add src/som_protocol.c src/som_protocol.h Kconfig CMakeLists.txt src/main.c
git commit -m "feat: add SOM-MCU UART4 communication protocol

Implements framed message protocol over UART4 for bidirectional
communication with the SOM CPU. Matches the vendor firmware's
UART4 protocol format (header/tail framing, XOR checksum,
request/reply pattern).

Supports: command send with timeout, notification callbacks,
connection state tracking.

Ported from: pio-hifive-mcu/src/hf_protocol_process.c"
```

---

## Task 2: SOM Keepalive Daemon

**Goal:** Poll the SOM every second to detect whether its OS is running, and update LED status accordingly.

**Background:** The vendor firmware (`deamon_keeplive_task` in `hf_protocol_process.c`) sends `CMD_BOARD_STATUS` to the SOM every 1 second. After 5 consecutive failures, it marks the SOM as offline. This is the only mechanism to know whether the host OS has booted.

**Files:**
- Modify: `src/som_protocol.c` — add keepalive thread
- Modify: `src/som_protocol.h` — expose `som_is_alive()`
- Modify: `Kconfig` — add keepalive config options under `SOM_PROTOCOL`

**Depends on:** Task 1

**Step 1: Add Kconfig options**

Add inside the existing `if SOM_PROTOCOL` block in `Kconfig`:

```kconfig
config SOM_KEEPALIVE
	bool "SOM keepalive health monitor"
	default y
	depends on SOM_PROTOCOL

config SOM_KEEPALIVE_INTERVAL_MS
	int "Keepalive poll interval (ms)"
	default 1000
	depends on SOM_KEEPALIVE

config SOM_KEEPALIVE_FAIL_THRESHOLD
	int "Consecutive failures before marking SOM offline"
	default 5
	depends on SOM_KEEPALIVE

config SOM_KEEPALIVE_STACK_SIZE
	int "Keepalive thread stack size"
	default 1024
	depends on SOM_KEEPALIVE
```

**Step 2: Add keepalive thread to `src/som_protocol.c`**

Add after the existing `som_protocol_init()`:

```c
#ifdef CONFIG_SOM_KEEPALIVE

K_THREAD_STACK_DEFINE(som_keepalive_stack, CONFIG_SOM_KEEPALIVE_STACK_SIZE);
static struct k_thread som_keepalive_thread_data;

static void som_keepalive_thread(void *a, void *b, void *c)
{
	int fail_count = 0;
	bool was_alive = false;

	LOG_INF("SOM keepalive thread started");

	while (1) {
		k_msleep(CONFIG_SOM_KEEPALIVE_INTERVAL_MS);

		if (!power_get_state()) {
			/* Host power is off, skip polling */
			if (som_alive) {
				som_set_alive(false);
				LOG_INF("SOM marked offline (host power off)");
			}
			fail_count = 0;
			continue;
		}

		int ret = som_cmd(SOM_CMD_BOARD_STATUS, NULL, 0,
				  CONFIG_SOM_KEEPALIVE_INTERVAL_MS);

		if (ret != 0) {
			fail_count++;
			if (fail_count >= CONFIG_SOM_KEEPALIVE_FAIL_THRESHOLD &&
			    som_alive) {
				som_set_alive(false);
				LOG_WRN("SOM keepalive lost after %d failures",
					fail_count);
			}
		} else {
			if (!som_alive) {
				LOG_INF("SOM keepalive established");
			}
			som_set_alive(true);
			fail_count = 0;
		}

		if (was_alive != som_alive) {
			LOG_INF("SOM daemon state: %s",
				som_alive ? "ONLINE" : "OFFLINE");
			was_alive = som_alive;
		}
	}
}

static int som_keepalive_init(void)
{
	k_thread_create(&som_keepalive_thread_data, som_keepalive_stack,
			K_THREAD_STACK_SIZEOF(som_keepalive_stack),
			som_keepalive_thread,
			NULL, NULL, NULL,
			CONFIG_SOM_PROTOCOL_PRIORITY + 1, 0, K_NO_WAIT);
	k_thread_name_set(&som_keepalive_thread_data, "som_keepalive");

	return 0;
}
#endif /* CONFIG_SOM_KEEPALIVE */
```

Add `#include "power.h"` at the top of `som_protocol.c`.

Call `som_keepalive_init()` at the end of `som_protocol_init()`:

```c
#ifdef CONFIG_SOM_KEEPALIVE
	som_keepalive_init();
#endif
```

**Step 3: Add shell command to check SOM status**

Add to `src/som_protocol.c`:

```c
#include <zephyr/shell/shell.h>

static int cmd_som_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "SOM daemon: %s", som_is_alive() ? "ONLINE" : "OFFLINE");
	shell_print(sh, "Host power: %s", power_get_state() ? "ON" : "OFF");

	return 0;
}

SHELL_CMD_REGISTER(som, NULL, "Show SOM communication status", cmd_som_status);
```

**Step 4: Build verification**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
```

**Step 5: Commit**

```bash
git add src/som_protocol.c src/som_protocol.h Kconfig
git commit -m "feat: add SOM keepalive health monitor

Polls SOM via UART4 CMD_BOARD_STATUS every 1s. After 5 consecutive
failures, marks SOM as offline. Adds 'som' shell command to check
status.

Ported from: pio-hifive-mcu deamon_keeplive_task()"
```

---

## Task 3: Power Sequencing State Machine

**Goal:** Replace the simple GPIO toggle power control with a multi-step power sequencing state machine that matches the real P550 hardware requirements.

**Background:** The vendor firmware (`hf_power_task` in `hf_power_process.c`) follows this sequence: ATX_PS_ON → DC_POWER_EN → wait for DC_POWER_GOOD → init I2C buses → release SOM reset → enable PMIC LED. Shutdown reverses the sequence. WallaBMC currently just toggles two GPIOs simultaneously.

**Files:**
- Modify: `src/power.c` — replace `power_on()`/`power_off()` with state machine
- Modify: `src/power.h` — add power state enum, SOM reset control
- Modify: `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add `pwrok`, `som_reset`, `som_rst_detect` GPIO nodes
- Modify: `boards/hifive_premier_p550_mcu.overlay` — add `reset-gpio` alias, `power-good` alias
- Modify: `Kconfig` — add power sequencing options

**Depends on:** Task 1 (SOM protocol for coordinated shutdown)

**Step 1: Add DTS nodes**

Add to the `gpio_keys` node in `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts`:

```dts
		pwrok: pwrok {
			gpios = <&gpioe 5 GPIO_ACTIVE_HIGH>;
			label = "DCDC_PWR_OK";
		};

		som_reset: som_reset {
			gpios = <&gpiod 5 GPIO_ACTIVE_LOW>;
			label = "SOM_RESET";
		};

		som_rst_detect: som_rst_detect {
			gpios = <&gpiod 6 GPIO_ACTIVE_LOW>;
			label = "SOM_RST_DETECT";
		};

		power_button: power_button {
			gpios = <&gpioa 12 GPIO_ACTIVE_LOW>;
			label = "POWER_BUTTON";
		};

		pwr_led: pwr_led_key {
			gpios = <&gpiod 10 GPIO_ACTIVE_HIGH>;
			label = "POWER_LED";
		};
```

**Step 2: Add aliases to overlay**

Add to the `aliases` block in `boards/hifive_premier_p550_mcu.overlay`:

```dts
		reset-gpio = &som_reset;
		power-good = &pwrok;
		power-button = &power_button;
		power-led = &pwr_led;
```

**Step 3: Add Kconfig options**

Add to `Kconfig`:

```kconfig
config POWER_SEQUENCING
	bool "Hardware power sequencing state machine"
	default y
	depends on $(dt_alias_exists,power-good)
	help
	  Enable multi-step power sequencing (ATX -> DC -> wait power good ->
	  I2C init -> SOM reset release) instead of simple GPIO toggle.

config POWER_GOOD_TIMEOUT_MS
	int "Power-good wait timeout (ms)"
	default 2000
	depends on POWER_SEQUENCING

config POWER_GOOD_POLL_MS
	int "Power-good poll interval (ms)"
	default 200
	depends on POWER_SEQUENCING

config POWER_BUTTON
	bool "Front-panel power button support"
	default y
	depends on $(dt_alias_exists,power-button)
```

**Step 4: Rewrite `power_on()` / `power_off()` in `src/power.c`**

Replace the simple GPIO toggle with a sequenced approach. The existing `power_on()` becomes:

```c
#define GPIO_POWER_GOOD DT_ALIAS(power_good)
#define GPIO_POWER_LED  DT_ALIAS(power_led)

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_GOOD)
static const struct gpio_dt_spec power_good_gpio =
	GPIO_DT_SPEC_GET(GPIO_POWER_GOOD, gpios);
#endif

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_LED)
static const struct gpio_dt_spec power_led_gpio =
	GPIO_DT_SPEC_GET(GPIO_POWER_LED, gpios);
#endif

static int power_on(void)
{
	int ret;

	/* Step 1: Assert ATX_PS_ON */
	for (int i = 0; i < ARRAY_SIZE(power_gpios); i++) {
		ret = gpio_pin_set_dt(&power_gpios[i], 1);
		if (ret < 0) {
			LOG_ERR("Could not assert power GPIO %d", i);
			return ret;
		}
	}

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_GOOD)
	/* Step 2: Wait for power good */
	int retries = CONFIG_POWER_GOOD_TIMEOUT_MS / CONFIG_POWER_GOOD_POLL_MS;

	while (retries > 0) {
		k_msleep(CONFIG_POWER_GOOD_POLL_MS);
		if (gpio_pin_get_dt(&power_good_gpio) == 1) {
			break;
		}
		retries--;
	}

	if (retries <= 0) {
		LOG_ERR("Power-good timeout, aborting power on");
		/* Reverse: turn off power */
		for (int i = 0; i < ARRAY_SIZE(power_gpios); i++) {
			gpio_pin_set_dt(&power_gpios[i], 0);
		}
		return -ETIMEDOUT;
	}

	LOG_INF("Power good detected");
#endif

	/* Step 3: Release SOM reset (handled by reset_init, already done) */

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_LED)
	/* Step 4: Turn on power LED */
	gpio_pin_set_dt(&power_led_gpio, 1);
#endif

	system_power_state = true;
	return 0;
}

static int power_off(void)
{
	int ret;

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_LED)
	gpio_pin_set_dt(&power_led_gpio, 0);
#endif

	for (int i = 0; i < ARRAY_SIZE(power_gpios); i++) {
		ret = gpio_pin_set_dt(&power_gpios[i], 0);
		if (ret < 0) {
			LOG_ERR("Could not deassert power GPIO %d", i);
			return ret;
		}
	}

	system_power_state = false;
	return 0;
}
```

Update `power_init()` to configure the new GPIOs:

```c
int power_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(power_gpios); i++) {
		if (!gpio_is_ready_dt(&power_gpios[i])) {
			LOG_INF("Power GPIO %d not ready", i);
			return -1;
		}
		if (gpio_pin_configure_dt(&power_gpios[i], GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_INF("Could not configure power GPIO %d", i);
			return -1;
		}
	}

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_GOOD)
	if (!gpio_is_ready_dt(&power_good_gpio)) {
		LOG_ERR("Power-good GPIO not ready");
		return -1;
	}
	if (gpio_pin_configure_dt(&power_good_gpio, GPIO_INPUT) < 0) {
		LOG_ERR("Could not configure power-good GPIO");
		return -1;
	}
#endif

#if DT_NODE_HAS_STATUS_OKAY(GPIO_POWER_LED)
	if (gpio_is_ready_dt(&power_led_gpio)) {
		gpio_pin_configure_dt(&power_led_gpio, GPIO_OUTPUT_INACTIVE);
	}
#endif

	if (config_host_auto_poweron()) {
		return power_on();
	}

	return 0;
}
```

**Step 5: Build verification**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
```

**Step 6: Commit**

```bash
git add src/power.c src/power.h Kconfig \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts \
  boards/hifive_premier_p550_mcu.overlay
git commit -m "feat: add power sequencing with power-good monitoring

Replaces simple GPIO toggle with sequenced power-on:
ATX_PS_ON -> DC_EN -> wait for DCDC_PWR_OK -> power LED on.
Adds power-good timeout detection, front-panel power LED,
SOM reset, and power button DTS nodes.

Ported from: pio-hifive-mcu/src/hf_power_process.c"
```

---

## Task 4: Boot Select Control

**Goal:** Allow the BMC to control the SOM's boot source (eMMC, SPI NOR, USB, UART) via GPIO pins PD0-PD3, with hardware (DIP switch) and software modes.

**Background:** The vendor firmware (`bootsel()` in `hf_power_process.c`, `init_bootsel()` / `set_bootsel()` in `hf_common.c`) controls 4 GPIOs that determine the EIC7700X boot mode. In hardware mode, GPIOs are inputs (following DIP switches). In software mode, GPIOs are outputs driven by the MCU. The `hardware.rst` already documents these pins and expected shell commands (`bootsel-g`, `bootsel-s`).

**Files:**
- Create: `src/bootsel.c`
- Create: `src/bootsel.h`
- Modify: `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add boot select GPIO nodes
- Modify: `boards/hifive_premier_p550_mcu.overlay` — add aliases
- Modify: `CMakeLists.txt` — add source
- Modify: `Kconfig` — add `BOOTSEL` config
- Modify: `src/main.c` — add `bootsel_init()` call

**Depends on:** None (independent)

**Step 1: Add DTS nodes**

Add to `gpio_keys` node in `hifive_premier_p550_mcu.dts`:

```dts
		bootsel0: bootsel0 {
			gpios = <&gpiod 0 GPIO_ACTIVE_HIGH>;
			label = "BOOT_SEL0";
		};
		bootsel1: bootsel1 {
			gpios = <&gpiod 1 GPIO_ACTIVE_HIGH>;
			label = "BOOT_SEL1";
		};
		bootsel2: bootsel2 {
			gpios = <&gpiod 2 GPIO_ACTIVE_HIGH>;
			label = "BOOT_SEL2";
		};
		bootsel3: bootsel3 {
			gpios = <&gpiod 3 GPIO_ACTIVE_HIGH>;
			label = "BOOT_SEL3";
		};
```

Add aliases in the overlay:

```dts
		bootsel-0 = &bootsel0;
		bootsel-1 = &bootsel1;
		bootsel-2 = &bootsel2;
		bootsel-3 = &bootsel3;
```

**Step 2: Add Kconfig**

```kconfig
config BOOTSEL
	bool "SOM boot mode selection"
	default y
	depends on $(dt_alias_exists,bootsel-0)
	help
	  Control the SOM boot source via BOOT_SEL[3:0] GPIO pins.
	  Supports hardware mode (follow DIP switch) and software mode
	  (MCU drives the pins).
```

**Step 3: Add to CMakeLists.txt**

```cmake
target_sources_ifdef(CONFIG_BOOTSEL app PRIVATE
	src/bootsel.c
)
```

**Step 4: Create `src/bootsel.h`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BOOTSEL_H__
#define __BOOTSEL_H__

#ifdef CONFIG_BOOTSEL
int bootsel_init(void);
#else
static inline int bootsel_init(void) { return 0; }
#endif

#endif
```

**Step 5: Create `src/bootsel.c`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(bootsel, LOG_LEVEL_INF);

#define BOOTSEL_COUNT 4

static const struct gpio_dt_spec bootsel_gpios[BOOTSEL_COUNT] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(bootsel_0), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(bootsel_1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(bootsel_2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(bootsel_3), gpios),
};

/* false = HW mode (input, follows DIP switch), true = SW mode (output) */
static bool sw_mode;

static int bootsel_set_hw_mode(void)
{
	for (int i = 0; i < BOOTSEL_COUNT; i++) {
		int ret = gpio_pin_configure_dt(&bootsel_gpios[i], GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to set BOOT_SEL%d to input: %d", i, ret);
			return ret;
		}
	}
	sw_mode = false;
	LOG_INF("Boot select: hardware mode (following DIP switch)");
	return 0;
}

static int bootsel_set_sw_mode(uint8_t value)
{
	for (int i = 0; i < BOOTSEL_COUNT; i++) {
		int ret = gpio_pin_configure_dt(&bootsel_gpios[i],
						GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to set BOOT_SEL%d to output: %d",
				i, ret);
			return ret;
		}
		ret = gpio_pin_set_dt(&bootsel_gpios[i], (value >> i) & 1);
		if (ret < 0) {
			LOG_ERR("Failed to drive BOOT_SEL%d: %d", i, ret);
			return ret;
		}
	}
	sw_mode = true;
	LOG_INF("Boot select: software mode, value=0x%x", value);
	return 0;
}

static int bootsel_read(uint8_t *value)
{
	*value = 0;
	for (int i = 0; i < BOOTSEL_COUNT; i++) {
		int val = gpio_pin_get_dt(&bootsel_gpios[i]);
		if (val < 0) {
			return val;
		}
		*value |= (val & 1) << i;
	}
	return 0;
}

static const char *bootsel_boot_source(uint8_t sel)
{
	/* EIC7700X boot modes (OTP security bit = 0, all 4 bits) */
	switch (sel & 0x0F) {
	case 0x00: return "SCPU ROM -> UART";
	case 0x01: return "SCPU ROM -> eMMC";
	case 0x02: return "SCPU ROM -> SPI NOR";
	case 0x03: return "SCPU ROM -> USB";
	case 0x04: return "SCPU SPI NOR -> UART";
	case 0x05: return "SCPU SPI NOR -> eMMC";
	case 0x06: return "SCPU SPI NOR -> SPI NOR";
	case 0x07: return "SCPU SPI NOR -> USB";
	default:
		if ((sel & 0x03) == 0x00) return "U84 SPI NOR -> UART";
		if ((sel & 0x03) == 0x01) return "U84 SPI NOR -> eMMC";
		if ((sel & 0x03) == 0x02) return "U84 SPI NOR -> SPI NOR";
		if ((sel & 0x03) == 0x03) return "U84 SPI NOR -> USB";
		return "Unknown";
	}
}

/* Shell commands */

static int cmd_bootsel_get(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t val;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = bootsel_read(&val);
	if (ret < 0) {
		shell_error(sh, "Failed to read boot select: %d", ret);
		return ret;
	}

	shell_print(sh, "Mode: %s", sw_mode ? "SOFTWARE" : "HARDWARE (DIP switch)");
	shell_print(sh, "BOOT_SEL[3:0] = 0x%x (0b%d%d%d%d)",
		    val,
		    (val >> 3) & 1, (val >> 2) & 1,
		    (val >> 1) & 1, (val >> 0) & 1);
	shell_print(sh, "Boot source: %s", bootsel_boot_source(val));

	return 0;
}

static int cmd_bootsel_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: bootsel set <0-15|hw>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "hw") == 0) {
		return bootsel_set_hw_mode();
	}

	unsigned long val = strtoul(argv[1], NULL, 0);
	if (val > 15) {
		shell_error(sh, "Value must be 0-15 (4-bit boot select)");
		return -EINVAL;
	}

	int ret = bootsel_set_sw_mode((uint8_t)val);
	if (ret < 0) {
		shell_error(sh, "Failed to set boot select: %d", ret);
		return ret;
	}

	shell_print(sh, "Boot select set to 0x%lx (%s)", val,
		    bootsel_boot_source((uint8_t)val));

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_bootsel_cmds,
	SHELL_CMD(get, NULL, "Get current boot select configuration", cmd_bootsel_get),
	SHELL_CMD_ARG(set, NULL,
		"Set boot select: <0-15> for SW mode, 'hw' for HW (DIP switch) mode",
		cmd_bootsel_set, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(bootsel, &sub_bootsel_cmds, "SOM boot mode selection", NULL);

int bootsel_init(void)
{
	for (int i = 0; i < BOOTSEL_COUNT; i++) {
		if (!gpio_is_ready_dt(&bootsel_gpios[i])) {
			LOG_ERR("BOOT_SEL%d GPIO not ready", i);
			return -ENODEV;
		}
	}

	/* Default: hardware mode (follow DIP switch) */
	return bootsel_set_hw_mode();
}
```

**Step 6: Add init call to `src/main.c`**

Add `#include "bootsel.h"` and after LED init:

```c
	LOG_DBG("Boot select init");
	if (bootsel_init() < 0) {
		LOG_ERR("Boot select init failed");
		/* Continue */
	}
```

**Step 7: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/bootsel.c src/bootsel.h CMakeLists.txt Kconfig src/main.c \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts \
  boards/hifive_premier_p550_mcu.overlay
git commit -m "feat: add SOM boot mode selection (BOOT_SEL[3:0])

Controls PD0-PD3 GPIO pins to select SOM boot source (eMMC, SPI NOR,
USB, UART). Supports hardware mode (follow DIP switch SW1) and
software mode (MCU drives pins). Shell commands: bootsel get/set.

Ported from: pio-hifive-mcu bootsel(), init_bootsel()"
```

---

## Task 5: I2C EEPROM Board Identity

**Goal:** Read carrier board identity data (serial number, MAC addresses, board revision) from the AT24C02C EEPROM via I2C1.

**Background:** The vendor firmware (`es_init_info_in_eeprom()` in `hf_common.c`) reads a `CarrierBoardInfo` struct from EEPROM address 0x50 on I2C1. This contains the factory-programmed MAC address used for Ethernet, board serial number, and PCB revision. The EEPROM is write-protected via GPIO PC8 and the I2C bus is shared with the SoC via a TMUX1574 mux controlled by PA3.

**Files:**
- Create: `src/board_identity.c`
- Create: `src/board_identity.h`
- Modify: `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add I2C1, EEPROM, I2C mux/WP GPIO nodes
- Modify: `boards/hifive_premier_p550_mcu.overlay` — add aliases
- Modify: `CMakeLists.txt` — add source
- Modify: `Kconfig` — add `BOARD_IDENTITY` config
- Modify: `src/main.c` — add init call
- Reference: `boards/sifive/hifive_premier_p550_mcu/doc/hardware.rst:404-498` (I2C details, DTS snippets)

**Depends on:** None (independent)

**Step 1: Add I2C1, EEPROM, and control GPIOs to DTS**

Add to `hifive_premier_p550_mcu.dts`:

```dts
&i2c1 {
	pinctrl-0 = <&i2c1_scl_pb6 &i2c1_sda_pb7>;
	pinctrl-names = "default";
	status = "okay";
	clock-frequency = <I2C_BITRATE_STANDARD>;

	eeprom: eeprom@50 {
		compatible = "atmel,at24";
		reg = <0x50>;
		size = <256>;
	};
};
```

Add to the `gpio_keys` node:

```dts
		i2c_mux_en: i2c_mux_en {
			gpios = <&gpioa 3 GPIO_ACTIVE_HIGH>;
			label = "I2C_MUX_EN";
		};
		eeprom_wp: eeprom_wp {
			gpios = <&gpioc 8 GPIO_ACTIVE_HIGH>;
			label = "EEPROM_WP";
		};
```

Add aliases in overlay:

```dts
		i2c-mux-en = &i2c_mux_en;
		eeprom-wp = &eeprom_wp;
```

**Step 2: Add Kconfig**

```kconfig
config BOARD_IDENTITY
	bool "Board identity from EEPROM"
	default y
	depends on I2C
	select EEPROM
	select EEPROM_AT2X
	help
	  Read carrier board identity (serial number, MAC addresses,
	  PCB revision) from AT24C02C EEPROM on I2C1.
```

**Step 3: Add to CMakeLists.txt**

```cmake
target_sources_ifdef(CONFIG_BOARD_IDENTITY app PRIVATE
	src/board_identity.c
)
```

**Step 4: Create `src/board_identity.h`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BOARD_IDENTITY_H__
#define __BOARD_IDENTITY_H__

#include <stdint.h>

#define BOARD_SERIAL_LEN 18

/*
 * Carrier board info layout in EEPROM (matches vendor firmware).
 * Offset 0x00, size 64 bytes. CRC32 protected.
 */
struct carrier_board_info {
	uint32_t magic;                      /* 0x45505EF1 */
	uint8_t  format_version;
	uint16_t product_id;
	uint8_t  pcb_revision;
	uint8_t  bom_revision;
	uint8_t  bom_variant;
	char     serial_number[BOARD_SERIAL_LEN];
	uint8_t  mfg_test_status;
	uint8_t  mac_som0[6];
	uint8_t  mac_som1[6];
	uint8_t  mac_mcu[6];
	uint32_t crc32;
} __packed;

#define CBINFO_MAGIC 0x45505EF1

#ifdef CONFIG_BOARD_IDENTITY
int board_identity_init(void);
const struct carrier_board_info *board_identity_get(void);
const uint8_t *board_identity_mac(void);
const char *board_identity_serial(void);
#else
static inline int board_identity_init(void) { return 0; }
static inline const uint8_t *board_identity_mac(void) { return NULL; }
static inline const char *board_identity_serial(void) { return "unknown"; }
#endif

#endif
```

**Step 5: Create `src/board_identity.c`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <string.h>

#include "board_identity.h"

LOG_MODULE_REGISTER(board_identity, LOG_LEVEL_INF);

static const struct device *eeprom_dev = DEVICE_DT_GET(DT_NODELABEL(eeprom));

#define I2C_MUX_EN_NODE DT_ALIAS(i2c_mux_en)
#if DT_NODE_EXISTS(I2C_MUX_EN_NODE)
static const struct gpio_dt_spec i2c_mux_gpio =
	GPIO_DT_SPEC_GET(I2C_MUX_EN_NODE, gpios);
#endif

#define EEPROM_WP_NODE DT_ALIAS(eeprom_wp)
#if DT_NODE_EXISTS(EEPROM_WP_NODE)
static const struct gpio_dt_spec eeprom_wp_gpio =
	GPIO_DT_SPEC_GET(EEPROM_WP_NODE, gpios);
#endif

static struct carrier_board_info cbinfo;
static bool cbinfo_valid;

const struct carrier_board_info *board_identity_get(void)
{
	return cbinfo_valid ? &cbinfo : NULL;
}

const uint8_t *board_identity_mac(void)
{
	return cbinfo_valid ? cbinfo.mac_mcu : NULL;
}

const char *board_identity_serial(void)
{
	static char serial_str[BOARD_SERIAL_LEN + 1];

	if (!cbinfo_valid) {
		return "unknown";
	}

	memcpy(serial_str, cbinfo.serial_number, BOARD_SERIAL_LEN);
	serial_str[BOARD_SERIAL_LEN] = '\0';
	return serial_str;
}

static int cmd_board_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!cbinfo_valid) {
		shell_error(sh, "Board identity not available");
		return -ENODATA;
	}

	shell_print(sh, "--- Carrier Board Info ---");
	shell_print(sh, "Serial: %s", board_identity_serial());
	shell_print(sh, "Product ID: 0x%04x", cbinfo.product_id);
	shell_print(sh, "PCB rev: %d, BOM rev: %d, BOM variant: %d",
		    cbinfo.pcb_revision, cbinfo.bom_revision,
		    cbinfo.bom_variant);
	shell_print(sh, "MCU MAC: %02x:%02x:%02x:%02x:%02x:%02x",
		    cbinfo.mac_mcu[0], cbinfo.mac_mcu[1], cbinfo.mac_mcu[2],
		    cbinfo.mac_mcu[3], cbinfo.mac_mcu[4], cbinfo.mac_mcu[5]);
	shell_print(sh, "SOM MAC0: %02x:%02x:%02x:%02x:%02x:%02x",
		    cbinfo.mac_som0[0], cbinfo.mac_som0[1], cbinfo.mac_som0[2],
		    cbinfo.mac_som0[3], cbinfo.mac_som0[4], cbinfo.mac_som0[5]);
	shell_print(sh, "SOM MAC1: %02x:%02x:%02x:%02x:%02x:%02x",
		    cbinfo.mac_som1[0], cbinfo.mac_som1[1], cbinfo.mac_som1[2],
		    cbinfo.mac_som1[3], cbinfo.mac_som1[4], cbinfo.mac_som1[5]);

	return 0;
}

SHELL_CMD_REGISTER(board_info, NULL, "Show carrier board identity from EEPROM",
		   cmd_board_info);

int board_identity_init(void)
{
	int ret;

	if (!device_is_ready(eeprom_dev)) {
		LOG_ERR("EEPROM device not ready");
		return -ENODEV;
	}

#if DT_NODE_EXISTS(I2C_MUX_EN_NODE)
	/* Configure I2C mux enable and claim bus for MCU */
	if (gpio_is_ready_dt(&i2c_mux_gpio)) {
		gpio_pin_configure_dt(&i2c_mux_gpio, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&i2c_mux_gpio, 1);
	}
#endif

#if DT_NODE_EXISTS(EEPROM_WP_NODE)
	/* Configure write-protect pin (keep protected) */
	if (gpio_is_ready_dt(&eeprom_wp_gpio)) {
		gpio_pin_configure_dt(&eeprom_wp_gpio, GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(&eeprom_wp_gpio, 1); /* write protect ON */
	}
#endif

	/* Read carrier board info from offset 0 */
	ret = eeprom_read(eeprom_dev, 0, &cbinfo, sizeof(cbinfo));
	if (ret < 0) {
		LOG_ERR("Failed to read EEPROM: %d", ret);
		return ret;
	}

	if (cbinfo.magic != CBINFO_MAGIC) {
		LOG_WRN("EEPROM magic mismatch: 0x%08x (expected 0x%08x)",
			cbinfo.magic, CBINFO_MAGIC);
		cbinfo_valid = false;
		return -EINVAL;
	}

	/* Verify CRC32 (over all fields except crc32 itself) */
	size_t crc_len = offsetof(struct carrier_board_info, crc32);
	uint32_t calc_crc = crc32_ieee((const uint8_t *)&cbinfo, crc_len);

	if (calc_crc != cbinfo.crc32) {
		LOG_WRN("EEPROM CRC mismatch: calc=0x%08x stored=0x%08x",
			calc_crc, cbinfo.crc32);
		/* Try backup at offset 80 (per vendor layout) */
		ret = eeprom_read(eeprom_dev, 80, &cbinfo, sizeof(cbinfo));
		if (ret < 0 || cbinfo.magic != CBINFO_MAGIC) {
			LOG_ERR("Backup EEPROM also invalid");
			cbinfo_valid = false;
			return -EINVAL;
		}
		calc_crc = crc32_ieee((const uint8_t *)&cbinfo, crc_len);
		if (calc_crc != cbinfo.crc32) {
			LOG_ERR("Backup EEPROM CRC also invalid");
			cbinfo_valid = false;
			return -EINVAL;
		}
		LOG_INF("Using backup EEPROM data");
	}

	cbinfo_valid = true;
	LOG_INF("Board serial: %s", board_identity_serial());
	LOG_INF("MCU MAC: %02x:%02x:%02x:%02x:%02x:%02x",
		cbinfo.mac_mcu[0], cbinfo.mac_mcu[1], cbinfo.mac_mcu[2],
		cbinfo.mac_mcu[3], cbinfo.mac_mcu[4], cbinfo.mac_mcu[5]);

	return 0;
}
```

**Step 6: Add init call to `src/main.c`**

Add `#include "board_identity.h"` and before network init:

```c
	LOG_DBG("Board identity init");
	if (board_identity_init() < 0) {
		LOG_ERR("Board identity init failed, MAC from EEPROM unavailable");
		/* Continue */
	}
```

**Step 7: Enable I2C in board defconfig**

Add to `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu_defconfig`:

```kconfig
CONFIG_I2C=y
```

**Step 8: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/board_identity.c src/board_identity.h CMakeLists.txt Kconfig src/main.c \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts \
  boards/hifive_premier_p550_mcu.overlay \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu_defconfig
git commit -m "feat: add board identity from EEPROM (serial, MAC)

Reads carrier board info (serial number, MAC addresses, PCB revision)
from AT24C02C EEPROM on I2C1. Includes I2C mux control (PA3) and
write-protect (PC8). CRC32 validation with backup fallback.
Shell command: board_info.

Ported from: pio-hifive-mcu/src/hf_common.c es_init_info_in_eeprom()"
```

---

## Task 6: Power Monitoring (INA226)

**Goal:** Read 12V rail voltage, current, and power consumption from the INA226 power monitor IC on I2C3.

**Background:** The vendor firmware (`get_board_power()` in `hf_i2c.c`) reads the INA226 at address 0x44 on I2C3. Zephyr has an upstream `ti,ina226` sensor driver that handles all the register-level details. The `hardware.rst` provides the DTS snippet and notes a 1 mOhm shunt resistor.

**Files:**
- Modify: `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add I2C3 with INA226
- Modify: `boards/hifive_premier_p550_mcu.conf` — enable sensor subsystem
- Modify: `src/redfish.c` — add power telemetry to Redfish Power resource (future)
- No new source files — Zephyr sensor API handles this

**Depends on:** Task 5 (I2C enabled in defconfig)

**Step 1: Add I2C3 and INA226 to DTS**

Add to `hifive_premier_p550_mcu.dts`:

```dts
&i2c3 {
	pinctrl-0 = <&i2c3_scl_pa8 &i2c3_sda_pc9>;
	pinctrl-names = "default";
	status = "okay";
	clock-frequency = <I2C_BITRATE_STANDARD>;

	ina226: ina226@44 {
		compatible = "ti,ina226";
		reg = <0x44>;
		rshunt-micro-ohms = <1000>;
	};
};
```

**Step 2: Enable sensor in board conf**

Add to `boards/hifive_premier_p550_mcu.conf`:

```kconfig
CONFIG_SENSOR=y
```

**Step 3: Add shell command for power reading**

Create `src/power_monitor.c`:

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_monitor, LOG_LEVEL_INF);

#define INA226_NODE DT_NODELABEL(ina226)

#if DT_NODE_EXISTS(INA226_NODE)
static const struct device *ina226_dev = DEVICE_DT_GET(INA226_NODE);

int power_monitor_read(int32_t *voltage_mv, int32_t *current_ma,
		       int32_t *power_mw)
{
	struct sensor_value val;
	int ret;

	if (!device_is_ready(ina226_dev)) {
		return -ENODEV;
	}

	ret = sensor_sample_fetch(ina226_dev);
	if (ret < 0) {
		LOG_ERR("INA226 sample fetch failed: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_VOLTAGE, &val);
	if (ret == 0 && voltage_mv) {
		*voltage_mv = val.val1 * 1000 + val.val2 / 1000;
	}

	ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_CURRENT, &val);
	if (ret == 0 && current_ma) {
		*current_ma = val.val1 * 1000 + val.val2 / 1000;
	}

	ret = sensor_channel_get(ina226_dev, SENSOR_CHAN_POWER, &val);
	if (ret == 0 && power_mw) {
		*power_mw = val.val1 * 1000 + val.val2 / 1000;
	}

	return 0;
}

static int cmd_power_info(const struct shell *sh, size_t argc, char **argv)
{
	int32_t voltage_mv, current_ma, power_mw;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = power_monitor_read(&voltage_mv, &current_ma, &power_mw);
	if (ret < 0) {
		shell_error(sh, "Failed to read power monitor: %d", ret);
		return ret;
	}

	shell_print(sh, "12V Rail:");
	shell_print(sh, "  Voltage: %d.%03d V", voltage_mv / 1000,
		    voltage_mv % 1000);
	shell_print(sh, "  Current: %d.%03d A", current_ma / 1000,
		    current_ma % 1000);
	shell_print(sh, "  Power:   %d.%03d W", power_mw / 1000,
		    power_mw % 1000);

	return 0;
}

SHELL_CMD_REGISTER(power_info, NULL, "Show 12V rail power (INA226)",
		   cmd_power_info);

#endif /* DT_NODE_EXISTS(INA226_NODE) */
```

Create `src/power_monitor.h`:

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __POWER_MONITOR_H__
#define __POWER_MONITOR_H__

int power_monitor_read(int32_t *voltage_mv, int32_t *current_ma,
		       int32_t *power_mw);

#endif
```

**Step 4: Add Kconfig and CMakeLists entry**

Kconfig:

```kconfig
config POWER_MONITOR
	bool "12V rail power monitoring (INA226)"
	default y
	depends on SENSOR
	depends on $(dt_nodelabel_enabled,ina226)
```

CMakeLists.txt:

```cmake
target_sources_ifdef(CONFIG_POWER_MONITOR app PRIVATE
	src/power_monitor.c
)
```

**Step 5: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/power_monitor.c src/power_monitor.h CMakeLists.txt Kconfig \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts \
  boards/hifive_premier_p550_mcu.conf
git commit -m "feat: add 12V rail power monitoring (INA226 on I2C3)

Uses Zephyr upstream ti,ina226 sensor driver to read voltage, current,
and power from the carrier board's 12V input rail power monitor.
Shell command: power_info.

Ported from: pio-hifive-mcu/src/hf_i2c.c get_board_power()"
```

---

## Task 7: PVT Info (Temperature/Fan Speed from SOM)

**Goal:** Request CPU temperature, NPU temperature, and fan speed from the SOM via the UART4 protocol.

**Background:** The vendor firmware sends `CMD_PVT_INFO` to the SOM and receives a `PVTInfo` struct containing `cpu_temp`, `npu_temp`, and `fan_speed`. This data comes from the SOM's Linux kernel.

**Files:**
- Modify: `src/som_protocol.c` — add `som_get_pvt_info()` convenience function
- Modify: `src/som_protocol.h` — expose the function

**Depends on:** Task 1 (SOM protocol)

**Step 1: Add convenience function to `src/som_protocol.c`**

```c
int som_get_pvt_info(struct som_pvt_info *info)
{
	if (!info) {
		return -EINVAL;
	}

	memset(info, 0, sizeof(*info));
	info->fan_speed = -1; /* default unavailable */

	return som_cmd(SOM_CMD_PVT_INFO, info, sizeof(*info),
		       CONFIG_SOM_PROTOCOL_TX_TIMEOUT_MS);
}
```

**Step 2: Add shell command**

```c
static int cmd_som_pvt(const struct shell *sh, size_t argc, char **argv)
{
	struct som_pvt_info info;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = som_get_pvt_info(&info);
	if (ret < 0) {
		shell_error(sh, "Failed to get PVT info from SOM: %d", ret);
		return ret;
	}

	shell_print(sh, "CPU temp: %d C", info.cpu_temp);
	shell_print(sh, "NPU temp: %d C", info.npu_temp);
	if (info.fan_speed >= 0) {
		shell_print(sh, "Fan speed: %d RPM", info.fan_speed);
	} else {
		shell_print(sh, "Fan speed: N/A");
	}

	return 0;
}
```

Update the `som` shell command to be a subcommand group:

```c
SHELL_STATIC_SUBCMD_SET_CREATE(sub_som_cmds,
	SHELL_CMD(status, NULL, "Show SOM communication status", cmd_som_status),
	SHELL_CMD(temp, NULL, "Show SOM temperature and fan speed", cmd_som_pvt),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(som, &sub_som_cmds, "SOM commands", NULL);
```

**Step 3: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/som_protocol.c src/som_protocol.h
git commit -m "feat: add SOM PVT info (temperature, fan speed)

Requests CPU/NPU temperature and fan speed from SOM via UART4
CMD_PVT_INFO. Shell command: som temp.

Ported from: pio-hifive-mcu CMD_PVT_INFO handling"
```

---

## Task 8: Fan PWM Control

**Goal:** Control two fan channels via TIM4 PWM outputs and read tachometer inputs for RPM sensing.

**Background:** The vendor firmware (`es_set_fan_duty()` in `hf_protocol_process.c`) uses TIM4 channels 1 and 2 (PD12, PD13) for PWM fan speed control. The `hardware.rst` documents these pins and provides DTS snippets.

**Files:**
- Create: `src/fan.c`
- Create: `src/fan.h`
- Modify: `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts` — add TIM4 PWM
- Modify: `CMakeLists.txt` — add source
- Modify: `Kconfig` — add `FAN_CONTROL` config
- Modify: `src/main.c` — add init call

**Depends on:** None (independent)

**Step 1: Add TIM4 PWM to DTS**

Add to `hifive_premier_p550_mcu.dts`:

```dts
&timers4 {
	status = "okay";
	pwm4: pwm {
		status = "okay";
		pinctrl-0 = <&tim4_ch1_pd12 &tim4_ch2_pd13>;
		pinctrl-names = "default";
	};
};
```

**Step 2: Add Kconfig**

```kconfig
config FAN_CONTROL
	bool "Fan PWM speed control"
	default y
	depends on PWM
	depends on $(dt_nodelabel_enabled,pwm4)

if FAN_CONTROL

config FAN_DEFAULT_DUTY_PCT
	int "Default fan duty cycle (%)"
	range 0 100
	default 50

config FAN_PWM_PERIOD_US
	int "PWM period (microseconds)"
	default 40
	help
	  PWM period. 40us = 25kHz, standard for 4-pin PC fans.

endif # FAN_CONTROL
```

**Step 3: Add to CMakeLists.txt**

```cmake
target_sources_ifdef(CONFIG_FAN_CONTROL app PRIVATE
	src/fan.c
)
```

**Step 4: Enable PWM in board defconfig**

Add to `boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu_defconfig`:

```kconfig
CONFIG_PWM=y
```

**Step 5: Create `src/fan.h`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __FAN_H__
#define __FAN_H__

#ifdef CONFIG_FAN_CONTROL
int fan_init(void);
int fan_set_duty(int fan_num, int duty_pct);
int fan_get_duty(int fan_num);
#else
static inline int fan_init(void) { return 0; }
#endif

#endif
```

**Step 6: Create `src/fan.c`**

```c
/*
 * SPDX-FileCopyrightText: (C) 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(fan, LOG_LEVEL_INF);

#define FAN_COUNT 2

/* TIM4 channels: CH1 (index 0) = CPU fan, CH2 (index 1) = chassis fan */
static const struct pwm_dt_spec fan_pwm[FAN_COUNT] = {
	PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm4), 0),
	PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm4), 1),
};

static int fan_duty[FAN_COUNT];

int fan_set_duty(int fan_num, int duty_pct)
{
	if (fan_num < 0 || fan_num >= FAN_COUNT) {
		return -EINVAL;
	}
	if (duty_pct < 0 || duty_pct > 100) {
		return -EINVAL;
	}

	uint32_t period = PWM_USEC(CONFIG_FAN_PWM_PERIOD_US);
	uint32_t pulse = period * duty_pct / 100;

	int ret = pwm_set_dt(&fan_pwm[fan_num], period, pulse);
	if (ret < 0) {
		LOG_ERR("Failed to set fan %d duty to %d%%: %d",
			fan_num, duty_pct, ret);
		return ret;
	}

	fan_duty[fan_num] = duty_pct;
	LOG_INF("Fan %d duty set to %d%%", fan_num, duty_pct);
	return 0;
}

int fan_get_duty(int fan_num)
{
	if (fan_num < 0 || fan_num >= FAN_COUNT) {
		return -EINVAL;
	}
	return fan_duty[fan_num];
}

static int cmd_fan_get(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (int i = 0; i < FAN_COUNT; i++) {
		shell_print(sh, "Fan %d: %d%%", i, fan_duty[i]);
	}
	return 0;
}

static int cmd_fan_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: fan set <0|1> <0-100>");
		return -EINVAL;
	}

	int fan_num = atoi(argv[1]);
	int duty = atoi(argv[2]);

	int ret = fan_set_duty(fan_num, duty);
	if (ret < 0) {
		shell_error(sh, "Failed to set fan: %d", ret);
		return ret;
	}

	shell_print(sh, "Fan %d set to %d%%", fan_num, duty);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_fan_cmds,
	SHELL_CMD(get, NULL, "Get fan duty cycles", cmd_fan_get),
	SHELL_CMD_ARG(set, NULL, "Set fan duty: <fan 0|1> <duty 0-100>",
		      cmd_fan_set, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fan, &sub_fan_cmds, "Fan control", NULL);

int fan_init(void)
{
	for (int i = 0; i < FAN_COUNT; i++) {
		if (!pwm_is_ready_dt(&fan_pwm[i])) {
			LOG_ERR("Fan %d PWM device not ready", i);
			return -ENODEV;
		}
	}

	/* Set default duty cycle */
	for (int i = 0; i < FAN_COUNT; i++) {
		fan_set_duty(i, CONFIG_FAN_DEFAULT_DUTY_PCT);
	}

	LOG_INF("Fan control initialized (%d fans, default %d%%)",
		FAN_COUNT, CONFIG_FAN_DEFAULT_DUTY_PCT);
	return 0;
}
```

**Step 7: Add init call to `src/main.c`**

Add `#include "fan.h"` and after power init:

```c
	LOG_DBG("Fan init");
	if (fan_init() < 0) {
		LOG_ERR("Fan init failed");
		/* Continue */
	}
```

**Step 8: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/fan.c src/fan.h CMakeLists.txt Kconfig src/main.c \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu.dts \
  boards/sifive/hifive_premier_p550_mcu/hifive_premier_p550_mcu_defconfig
git commit -m "feat: add fan PWM speed control (TIM4 CH1/CH2)

Controls two fan channels via TIM4 PWM at 25kHz. Shell commands:
fan get, fan set <0|1> <0-100>.

Ported from: pio-hifive-mcu es_set_fan_duty()/es_get_fan_duty()"
```

---

## Task 9: Graceful Shutdown Timers

**Goal:** When requesting SOM power-off or reboot, set a safety timer. If the SOM doesn't acknowledge within the timeout, force the power transition.

**Background:** The vendor firmware (`xSomPowerOffTimer`, `xSomRebootTimer`, `xSomRestartTimer` in `hf_protocol_process.c`) creates one-shot timers (5-6 seconds) when a power-off/reboot is requested. If the SOM sends a notification acknowledging shutdown, the timer is cancelled. If the timer fires, the BMC forces the action. This prevents the system from hanging if the SOM is unresponsive.

**Files:**
- Modify: `src/power.c` — add graceful shutdown with timer fallback
- Modify: `src/power.h` — add `power_graceful_off()`, `power_graceful_restart()`
- Modify: `src/som_protocol.c` — register notification handler for power-off/restart ack

**Depends on:** Task 1 (SOM protocol)

**Step 1: Add Kconfig options**

```kconfig
config GRACEFUL_SHUTDOWN_TIMEOUT_MS
	int "Graceful shutdown timeout (ms)"
	default 6000
	depends on SOM_PROTOCOL
	help
	  Time to wait for SOM to acknowledge shutdown before forcing
	  power off.

config GRACEFUL_REBOOT_TIMEOUT_MS
	int "Graceful reboot timeout (ms)"
	default 5000
	depends on SOM_PROTOCOL
```

**Step 2: Add to `src/power.c`**

```c
#ifdef CONFIG_SOM_PROTOCOL
#include "som_protocol.h"

static struct k_timer shutdown_timer;
static struct k_timer reboot_timer;

static void shutdown_timer_expiry(struct k_timer *timer)
{
	LOG_WRN("SOM shutdown timeout, forcing power off");
	power_off();
}

static void reboot_timer_expiry(struct k_timer *timer)
{
	LOG_WRN("SOM reboot timeout, forcing reset");
	power_reset();
}

static void som_power_notify(uint8_t cmd_type)
{
	if (cmd_type == SOM_CMD_POWER_OFF) {
		k_timer_stop(&shutdown_timer);
		LOG_INF("SOM acknowledged shutdown, powering off");
		power_off();
	} else if (cmd_type == SOM_CMD_RESTART) {
		k_timer_stop(&reboot_timer);
		LOG_INF("SOM acknowledged restart, resetting");
		power_reset();
	}
}

int power_graceful_off(void)
{
	int ret;

	if (!power_get_state()) {
		return 0; /* Already off */
	}

	/* Ask SOM to shut down */
	ret = som_cmd(SOM_CMD_POWER_OFF, NULL, 0,
		      CONFIG_SOM_PROTOCOL_TX_TIMEOUT_MS);
	if (ret < 0) {
		LOG_WRN("Could not send shutdown to SOM, forcing off");
		return power_off();
	}

	/* Start safety timer */
	k_timer_start(&shutdown_timer,
		       K_MSEC(CONFIG_GRACEFUL_SHUTDOWN_TIMEOUT_MS), K_NO_WAIT);

	LOG_INF("Graceful shutdown initiated, timeout %d ms",
		CONFIG_GRACEFUL_SHUTDOWN_TIMEOUT_MS);
	return 0;
}

int power_graceful_restart(void)
{
	int ret;

	if (!power_get_state()) {
		return power_on(); /* Just power on */
	}

	ret = som_cmd(SOM_CMD_RESTART, NULL, 0,
		      CONFIG_SOM_PROTOCOL_TX_TIMEOUT_MS);
	if (ret < 0) {
		LOG_WRN("Could not send restart to SOM, forcing reset");
		return power_reset();
	}

	k_timer_start(&reboot_timer,
		       K_MSEC(CONFIG_GRACEFUL_REBOOT_TIMEOUT_MS), K_NO_WAIT);

	LOG_INF("Graceful restart initiated, timeout %d ms",
		CONFIG_GRACEFUL_REBOOT_TIMEOUT_MS);
	return 0;
}

static void power_graceful_init(void)
{
	k_timer_init(&shutdown_timer, shutdown_timer_expiry, NULL);
	k_timer_init(&reboot_timer, reboot_timer_expiry, NULL);
	som_set_notify_callback(som_power_notify);
}
#endif /* CONFIG_SOM_PROTOCOL */
```

Call `power_graceful_init()` at the end of `power_init()`.

**Step 3: Update shell commands**

Add to the `sub_power_cmds` in `power.c`:

```c
#ifdef CONFIG_SOM_PROTOCOL
static int cmd_power_graceful_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return power_graceful_off();
}

static int cmd_power_graceful_restart(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return power_graceful_restart();
}
#endif
```

Update `sub_power_cmds`:

```c
SHELL_STATIC_SUBCMD_SET_CREATE(sub_power_cmds,
	SHELL_CMD(on,    NULL, "Power on.", cmd_power_on),
	SHELL_CMD(off,   NULL, "Force power off.", cmd_power_off),
#ifdef CONFIG_SOM_PROTOCOL
	SHELL_CMD(shutdown, NULL, "Graceful shutdown (ask SOM first).", cmd_power_graceful_off),
	SHELL_CMD(restart, NULL, "Graceful restart (ask SOM first).", cmd_power_graceful_restart),
#endif
	SHELL_SUBCMD_SET_END
);
```

**Step 4: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add src/power.c src/power.h Kconfig
git commit -m "feat: add graceful shutdown with safety timers

Sends shutdown/restart commands to SOM via UART4 protocol, then
starts a safety timer. If SOM acknowledges, power is cut cleanly.
If timeout fires, power is forced off. Shell: power shutdown/restart.

Ported from: pio-hifive-mcu xSomPowerOffTimer/xSomRebootTimer"
```

---

## Task 10: Factory Reset Button Improvement

**Goal:** Fix the recovery button (`K_REC` on PB1) to actually work (missing DTS alias) and add long-press (10s) factory reset behavior matching the vendor firmware.

**Background:** The vendor firmware (`key_user_rst_process()` in `hf_gpio_process.c`) requires a 10-second hold to trigger factory reset, with LED feedback during the hold. WallaBMC's `button.c` has a 1-second hold, but it doesn't work because the `hardware.rst` notes: "Bug: `reset-button` alias missing in overlay, so `button_init()` silently skips."

**Files:**
- Modify: `boards/hifive_premier_p550_mcu.overlay` — add `reset-button` alias
- Modify: `src/button.c` — increase hold time, add LED feedback

**Depends on:** None (independent)

**Step 1: Fix the DTS alias**

Add to the `aliases` block in `boards/hifive_premier_p550_mcu.overlay`:

```dts
		reset-button = &user_button_recovery;
```

**Step 2: Update hold time and add feedback in `src/button.c`**

Change `RESET_HOLD_TIME_MS` from 1000 to 10000:

```c
#define RESET_HOLD_TIME_MS 10000
```

Update `button_work_fn` to log the hold requirement:

```c
static void button_work_fn(struct k_work *work)
{
	int val = gpio_pin_get_dt(&button);
	if (val == 1) {
		/* Pressed */
		LOG_INF("Recovery button pressed, hold for %d seconds to factory reset",
			RESET_HOLD_TIME_MS / 1000);
		k_work_schedule(&reset_work, K_MSEC(RESET_HOLD_TIME_MS));
	} else if (val == 0) {
		/* Released before timeout */
		k_work_cancel_delayable(&reset_work);
	}
}
```

**Step 3: Build and commit**

```bash
cd zephyr && west build --sysbuild -b hifive_premier_p550_mcu ../wallabmc
git add boards/hifive_premier_p550_mcu.overlay src/button.c
git commit -m "fix: enable recovery button and increase hold time to 10s

Adds missing reset-button alias so button_init() activates on P550.
Increases factory reset hold time from 1s to 10s to match vendor
firmware behavior and prevent accidental resets.

Fixes: hardware.rst noted 'reset-button alias missing in overlay'"
```

---

## Summary

| Task | Feature | New Files | Kconfig |
|------|---------|-----------|---------|
| 1 | SOM UART4 Protocol | `som_protocol.c/h` | `SOM_PROTOCOL` |
| 2 | SOM Keepalive | (in som_protocol.c) | `SOM_KEEPALIVE` |
| 3 | Power Sequencing | (modifies power.c) | `POWER_SEQUENCING` |
| 4 | Boot Select | `bootsel.c/h` | `BOOTSEL` |
| 5 | EEPROM Board Identity | `board_identity.c/h` | `BOARD_IDENTITY` |
| 6 | Power Monitor (INA226) | `power_monitor.c/h` | `POWER_MONITOR` |
| 7 | PVT Info from SOM | (in som_protocol.c) | (part of SOM_PROTOCOL) |
| 8 | Fan PWM | `fan.c/h` | `FAN_CONTROL` |
| 9 | Graceful Shutdown | (modifies power.c) | (part of SOM_PROTOCOL) |
| 10 | Factory Reset Button | (modifies button.c) | (existing) |

**DTS changes** (all in `hifive_premier_p550_mcu.dts` and overlay):
- GPIO nodes: `pwrok`, `som_reset`, `som_rst_detect`, `power_button`, `pwr_led`, `bootsel0-3`, `i2c_mux_en`, `eeprom_wp`
- Peripherals: `&i2c1` with EEPROM, `&i2c3` with INA226, `&timers4` with PWM
- Aliases: `reset-gpio`, `power-good`, `power-button`, `power-led`, `bootsel-0` through `bootsel-3`, `i2c-mux-en`, `eeprom-wp`, `reset-button`
