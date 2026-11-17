// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * EXPERIMENTAL (K116s): the two single-precision steps of the panel's pixel compensation
 * tables (dcp-panelcal.inc): where the panel sits between the model file's calibration bins,
 * and the blend of two bins' tables. Kept bit-exact with what macOS hands the display
 * firmware, hence floating point. Callers hold kernel_fpu_begin().
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "dcp-panelcal-fp.h"

static float panelcal_f(u32 bits)
{
	float f;

	memcpy(&f, &bits, sizeof(f));
	return f;
}

static float panelcal_bin(u32 raw, const u32 *pts, unsigned int n)
{
	float x = (float)raw / 65536.0f, frac = 0.0f;
	unsigned int i;

	for (i = 0; i < n; i++)
		if (x <= panelcal_f(pts[i]))
			break;
	if (i >= 1 && i + 2 <= n)
		frac = (x - panelcal_f(pts[i - 1])) /
		       (panelcal_f(pts[i]) - panelcal_f(pts[i - 1]));
	return frac + (float)(i ? i : 1);
}

/* The lower bin's number; *t_bits is the weight of the next one, a float's bits (0: none) */
unsigned int dcp_panelcal_fp_bin(const u32 *lo, const u32 *hi, unsigned int n, u32 mix_bits,
				 u32 raw_lo, u32 raw_hi, u32 *t_bits)
{
	float mix = panelcal_f(mix_bits), rest = (1.0f - mix) * panelcal_bin(raw_hi, hi, n);
	float bin = __builtin_fmaf(mix, panelcal_bin(raw_lo, lo, n), rest), t;
	unsigned int ibin = (unsigned int)bin;

	t = bin - (float)ibin;
	if (ibin + 2 > n)
		t = 0.0f;
	memcpy(t_bits, &t, sizeof(t));
	return ibin;
}

/* @n 16-bit words: a + t * (b - a), rounded as macOS's loader rounds it */
void dcp_panelcal_fp_blend(u8 *out, const u8 *a, const u8 *b, unsigned int n, u32 t_bits)
{
	float t = panelcal_f(t_bits);

	for (unsigned int i = 0; i < n; i++) {
		int x = get_unaligned_le16(a + 2 * i), y = get_unaligned_le16(b + 2 * i);
		float r = __builtin_fmaf(t, (float)(y - x), (float)x);

		put_unaligned_le16((u16)(int)((double)r + 0.5), out + 2 * i);
	}
}

/* Lateral leakage: the panel's bin, 16.16, from its device-tree value (a 0.24 fraction of 100) */
u32 dcp_panelcal_fp_llc_bin(u32 v)
{
	float f = (float)v / 16777216.0f;

	f *= 100.0f;
	f *= 65536.0f;
	return (u32)(int)((double)f + 0.5);
}

/* A float's bits: @num / @den */
u32 dcp_panelcal_fp_ratio(u32 num, u32 den)
{
	float f = (float)num / (float)den;
	u32 bits;

	memcpy(&bits, &f, sizeof(bits));
	return bits;
}
