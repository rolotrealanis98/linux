/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * The T6050 boards a deep sleep is set up on.
 *
 * The sleep was brought up on the J714s (14-inch M5 Pro). The J716s (16-inch M5 Pro) has the
 * same SoC, the same port controllers on the same buses (hpm1, hpm2 and the MagSafe's hpm5),
 * the N1 on PCIe port 0 with the same warm reset (SMC key 'bpli', ADT pci-bridge0
 * function-warm_reset) and the same SMC; its board tree includes the J714s sleep description.
 * The display's sleep on firmware 27 stays the J714s's own (dcp->v27_j714s).
 */
#ifndef _LINUX_SOC_APPLE_T6050_SLEEP_BOARD_H_
#define _LINUX_SOC_APPLE_T6050_SLEEP_BOARD_H_

#include <linux/of.h>

static const char *const apple_t6050_sleep_boards[] __maybe_unused = {
	"apple,j714s",
	"apple,j716s",
	NULL
};

static inline bool apple_t6050_sleep_board(void)
{
	return of_machine_compatible_match(apple_t6050_sleep_boards);
}

#endif
