/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright The Asahi Linux Contributors */
/* Copyright 2026 Aurora Silicon */

#ifndef APPLE_IOMFB_V27_H
#define APPLE_IOMFB_V27_H

#include "iomfb_h17p.h"

struct dcp_swap_v27 {

	u64 timestamp[7];
	u64 unk_38;

	u64 debug_info[2];

	u64 event_wait[SWAP_SURFACES];
	u64 event_signal_on_glass;
	u64 event_signal[SWAP_SURFACES];

	u32 swap_id;

	u32 surf_ids[SWAP_SURFACES];
	struct dcp_rect src_rect[SWAP_SURFACES];
	u32 surf_flags[SWAP_SURFACES];
	u32 surf_unk[SWAP_SURFACES];
	struct dcp_rect dst_rect[SWAP_SURFACES];
	u32 swap_enabled;
	u32 swap_completed;

	u32 bg_color;
	u8 reserved_158[0x1fc];

	u8 brightness_update;
	u8 reserved_355[3];
	/* As desktop 26.x: [1] limit, [2] EDR scale, [4] headroom used */
	u8 bl_flags[6];
	u64 brightness_nits;
	u8 reserved_366[0x10];
	u64 bl_limit;
	u64 edr_scale;
	u8 reserved_386[0x60];
	u64 edr_headroom;
	u8 reserved_3ee[0x2f2];
} __packed;

struct dcp_surface_v27 {
	struct dcp_surface base;
	u8 padding[47];
} __packed;

struct dcp_swap_submit_req_v27 {
	struct dcp_swap_v27 swap;
	struct dcp_surface_v27 surf[SWAP_SURFACES];
	u64 surf_iova[SWAP_SURFACES];
	u64 unk_u64_a[SWAP_SURFACES];

	u8 unkbool;
	u64 unkdouble;
	u64 unkU64;
	u8 unkbool2;
	u32 clear;
	u32 unkU32Ptr;
	u8 swap_null;
	u8 surf_null[SWAP_SURFACES];
	u8 unkoutbool_null;
	u8 unknown_pointer_null;
	u8 unknown_output_null;
	u8 padding[2];
} __packed;

struct dcp_swap_submit_resp_v27 {
	u8 unkoutbool;
	u32 unkU32out;
	u32 ret;
	u8 padding[3];
} __packed;

static_assert(sizeof(struct dcp_swap_v27) == 0x6e0);
static_assert(sizeof(struct dcp_surface_v27) == 0x22c);
static_assert(sizeof(struct dcp_swap_submit_req_v27) == 0xff4);
static_assert(offsetof(struct dcp_swap_submit_req_v27, surf) == 0x6e0);
static_assert(offsetof(struct dcp_swap_submit_req_v27, surf_iova) == 0xf90);
static_assert(offsetof(struct dcp_swap_submit_req_v27, swap_null) == 0xfea);
static_assert(offsetof(struct dcp_swap_v27, brightness_update) == 0x354);
static_assert(offsetof(struct dcp_swap_v27, brightness_nits) == 0x35e);
static_assert(offsetof(struct dcp_swap_v27, bl_flags) == 0x358);
static_assert(offsetof(struct dcp_swap_v27, bl_limit) == 0x376);
static_assert(offsetof(struct dcp_swap_v27, edr_scale) == 0x37e);
static_assert(offsetof(struct dcp_swap_v27, edr_headroom) == 0x3e6);

/*
 * Firmware 27.3 grew IOMFBSwap from 0x6e0 to 0x830 bytes, so its swap_submit
 * request is 0x150 bytes longer; the rest of the request is unchanged. Of the
 * 27.2 swap fields, those from swap_id (0x98) up to 0x298 are 0x40 bytes
 * further on, those from 0x2cc up to 0x534 0x60 bytes and those from 0x6b8 on
 * 0x150 bytes, so the new bytes lie before swap_id, between 0x298 and 0x2cc
 * and between 0x534 and 0x6b8. This driver sends zeros in all three ranges,
 * so the 27.3 swap is the 27.2 one with that many zeros inserted at one point
 * of each range. No swap of the macOS 27.3 driver has been traced yet, so a
 * trace does not confirm this layout; the 27.3 firmware of a J716s takes the
 * request and shows its frames.
 */
#define DCP_SWAP_V27_3_GAP0	0x098
#define DCP_SWAP_V27_3_GAP0_LEN	0x40
#define DCP_SWAP_V27_3_GAP1	0x2cc
#define DCP_SWAP_V27_3_GAP1_LEN	0x20
#define DCP_SWAP_V27_3_GAP2	0x6b8
#define DCP_SWAP_V27_3_GAP2_LEN	0xf0
#define DCP_SWAP_V27_3_SIZE	0x830

struct dcp_swap_submit_req_v27_3 {
	u8 swap[DCP_SWAP_V27_3_SIZE];
	struct dcp_surface_v27 surf[SWAP_SURFACES];
	u8 args[sizeof(struct dcp_swap_submit_req_v27) -
		offsetof(struct dcp_swap_submit_req_v27, surf_iova)];
} __packed;

static_assert(sizeof(struct dcp_swap_v27) + DCP_SWAP_V27_3_GAP0_LEN +
	      DCP_SWAP_V27_3_GAP1_LEN + DCP_SWAP_V27_3_GAP2_LEN ==
	      DCP_SWAP_V27_3_SIZE);
static_assert(sizeof(struct dcp_swap_submit_req_v27_3) == 0x1144);
static_assert(offsetof(struct dcp_swap_submit_req_v27_3, surf) == 0x830);

void iomfb_serialize_present_v27(struct dcp_swap_submit_req_v27 *wire,
				const struct dcp_swap_submit_req_h17p *request);
struct apple_dcp;
bool iomfb_v27_swap_is_27_3(const struct apple_dcp *dcp);
void *iomfb_v27_swap_wire(const struct apple_dcp *dcp,
			  struct dcp_swap_submit_req_v27 *request,
			  struct dcp_swap_submit_req_v27_3 *scratch, u32 *len);
bool iomfb_v27_callback_size_valid(unsigned int tag, u32 input, u32 output);
void iomfb_firmware_state_cleanup_v27(void *data);
int iomfb_modeset_v27(struct apple_dcp *dcp, struct drm_crtc_state *state);
void iomfb_flush_v27(struct apple_dcp *dcp, struct drm_crtc *crtc,
		     struct drm_atomic_commit *commit);
void iomfb_start_v27(struct apple_dcp *dcp);
void iomfb_shutdown_v27(struct apple_dcp *dcp);
void iomfb_poweron_v27(struct apple_dcp *dcp);
void iomfb_poweroff_v27(struct apple_dcp *dcp);
void iomfb_sleep_v27(struct apple_dcp *dcp);
void iomfb_present_backlight_v27(struct apple_dcp *dcp);

#endif
