/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCP_PANELCAL_FP_H
#define APPLE_DCP_PANELCAL_FP_H

#include <linux/types.h>

unsigned int dcp_panelcal_fp_bin(const u32 *lo, const u32 *hi, unsigned int n, u32 mix_bits,
				 u32 raw_lo, u32 raw_hi, u32 *t_bits);
void dcp_panelcal_fp_blend(u8 *out, const u8 *a, const u8 *b, unsigned int n, u32 t_bits);
u32 dcp_panelcal_fp_llc_bin(u32 v);
u32 dcp_panelcal_fp_ratio(u32 num, u32 den);
#endif
