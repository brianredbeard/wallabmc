/*
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent AI ULC
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
