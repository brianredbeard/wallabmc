/*
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent AI ULC
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

static const struct pwm_dt_spec fan_pwm[FAN_COUNT] = {
	PWM_DT_SPEC_GET(DT_NODELABEL(fan0)),
	PWM_DT_SPEC_GET(DT_NODELABEL(fan1)),
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

	for (int i = 0; i < FAN_COUNT; i++) {
		fan_set_duty(i, CONFIG_FAN_DEFAULT_DUTY_PCT);
	}

	LOG_INF("Fan control initialized (%d fans, default %d%%)",
		FAN_COUNT, CONFIG_FAN_DEFAULT_DUTY_PCT);
	return 0;
}
