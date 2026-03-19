/*
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __POWER_MONITOR_H__
#define __POWER_MONITOR_H__

int power_monitor_read(int32_t *voltage_mv, int32_t *current_ma,
		       int32_t *power_mw);

#endif
