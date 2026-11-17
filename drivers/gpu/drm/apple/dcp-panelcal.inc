// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * EXPERIMENTAL (K116s, firmware 27.2): the internal panel's factory compensation tables.
 *
 * macOS's display loader reads the panel's calibration (factory data class ADCL, one file per
 * panel serial on the machine's Hardware volume), converts each record to a fixed struct and
 * hands it to the display firmware with set_block. The tables live in the firmware's memory
 * only: a firmware that was restarted (a firmware-driven sleep) has lost them and says so on
 * every frame ("IRDC Device data not valid for media RR mode 0").
 *
 * This sends what the calibration file itself holds: the IR-drop compensation's device data
 * (block 118, one per refresh-rate mode) and the ELVSS tables (blocks 122 and 123). The file
 * is taken as firmware "apple/panel-adcl.bin": a copy of the panel's ADCL file, which only the
 * machine's owner can provide. Vendor data (block 119) and the pixel compensation tables come
 * from other files and are not sent yet.
 */
#include <linux/firmware.h>
#include <linux/sort.h>

#define PANELCAL_FW		"apple/panel-adcl.bin"
#define PANELCAL_TAG(a, b, c, d) ((u32)(d) | (u32)(c) << 8 | (u32)(b) << 16 | (u32)(a) << 24)
#define PANELCAL_IRDC		PANELCAL_TAG('I', 'R', 'D', 'C')
#define PANELCAL_ELVS		PANELCAL_TAG('E', 'L', 'V', 'S')
#define PANELCAL_ADCL		PANELCAL_TAG('A', 'D', 'C', 'L')

#define PANELCAL_BLOCK_IRDC_DATA	118
#define PANELCAL_BLOCK_ELVS_CONFIG	122
#define PANELCAL_BLOCK_ELVS_LUTS	123

#define PANELCAL_IRDC_TAPS	45
#define PANELCAL_IRDC_TAP_BYTES	(3 * 257 * 4)
#define PANELCAL_IRDC_BUF	0x24000
#define PANELCAL_ELVS_TAPS	40
#define PANELCAL_ELVS_BUF	0x4000
#define PANELCAL_ELVS_MAX	10

struct panelcal_rec {
	const u8 *body;
	size_t len;
	s16 key;
};

/* The 20-byte header of a block that comes with a buffer */
static void panelcal_header(u8 *h, u32 size)
{
	memset(h, 0, 20);
	put_unaligned_le32(1, h);
	put_unaligned_le32(size, h + 8);
}

static int panelcal_send(struct apple_dcp *dcp, u32 block, const char *what, int index,
			 const void *data, size_t data_size, const void *buf, size_t buf_size)
{
	u32 status = ~0U;
	int ret = iomfb_v27_set_block(dcp, block, data, data_size, buf, buf_size, &status);

	if (ret || status)
		dev_warn(dcp->dev, "panel calibration: %s %d (block %u): %d, status %#x\n", what,
			 index, block, ret, status);
	else
		dev_info(dcp->dev, "panel calibration: %s %d (block %u) accepted\n", what, index,
			 block);
	return ret ?: status ? -EREMOTEIO : 0;
}

static int panelcal_cmp(const void *a, const void *b)
{
	return ((const struct panelcal_rec *)a)->key - ((const struct panelcal_rec *)b)->key;
}

static int dcp_panelcal_load(struct apple_dcp *dcp)
{
	struct panelcal_rec irdc[3], elvs[PANELCAL_ELVS_MAX];
	unsigned int nirdc = 0, nelvs = 0, i, sent = 0;
	const struct firmware *fw;
	const u8 *p = NULL, *end;
	size_t off;
	u8 hdr[20], cfg[28];
	u8 *buf;
	int ret;

	if (dcp->fw_compat != DCP_FIRMWARE_V_27_0 || !dcp->main_display)
		return -EOPNOTSUPP;
	ret = request_firmware(&fw, PANELCAL_FW, dcp->dev);
	if (ret)
		return ret;
	/* The container sits in an IMG4 payload wrapper: its top record starts with the tag. */
	for (off = 0; off + 12 <= fw->size && off < 256; off++)
		if (get_unaligned_be32(fw->data + off) ==
		    swab32(PANELCAL_ADCL)) {
			p = fw->data + off;
			break;
		}
	ret = -EINVAL;
	if (!p)
		goto out;
	end = p + min_t(size_t, fw->size - (p - fw->data), (size_t)get_unaligned_le32(p + 8) * 4);
	for (off = 12; p + off + 12 <= end;) {
		u32 tag = get_unaligned_le32(p + off);
		u16 ver = get_unaligned_le16(p + off + 6);
		size_t len = (size_t)get_unaligned_le32(p + off + 8) * 4;
		const u8 *body = p + off + 12;

		if (len < 12 || len > (size_t)(end - (p + off)))
			break;
		if (tag == PANELCAL_IRDC && ver == 3 && nirdc < ARRAY_SIZE(irdc)) {
			irdc[nirdc++] = (struct panelcal_rec) { body, len - 12,
								get_unaligned_le16(body + 2) };
		} else if (tag == PANELCAL_ELVS && ver == 1 && nelvs < ARRAY_SIZE(elvs)) {
			elvs[nelvs++] = (struct panelcal_rec) { body, len - 12,
								(s8)body[2] };
		}
		off += len;
	}
	dev_info(dcp->dev, "panel calibration: %u IRDC and %u ELVS records in %zu bytes\n", nirdc,
		 nelvs, fw->size);
	buf = kvzalloc(PANELCAL_IRDC_BUF, GFP_KERNEL);
	ret = -ENOMEM;
	if (!buf)
		goto out;

	/* ELVSS: the temperatures' table, then one LUT per temperature, ascending */
	if (nelvs) {
		sort(elvs, nelvs, sizeof(elvs[0]), panelcal_cmp, NULL);
		memset(cfg, 0, sizeof(cfg));
		put_unaligned_le32(1, cfg);
		put_unaligned_le16(get_unaligned_le16(elvs[0].body), cfg + 4);
		for (i = 0; i < nelvs; i++)
			put_unaligned_le16(elvs[i].key, cfg + 6 + 2 * i);
		put_unaligned_le16(nelvs, cfg + 26);
		ret = panelcal_send(dcp, PANELCAL_BLOCK_ELVS_CONFIG, "ELVS config", nelvs, cfg,
				    sizeof(cfg), NULL, 0);
		for (i = 0; !ret && i < nelvs; i++) {
			unsigned int n = get_unaligned_le16(elvs[i].body + 4);

			if (n > PANELCAL_ELVS_TAPS || elvs[i].len < 6 + 6 * (size_t)n) {
				ret = -EINVAL;
				break;
			}
			memset(buf, 0, PANELCAL_ELVS_BUF);
			put_unaligned_le32(1, buf);
			put_unaligned_le16(elvs[i].key, buf + 6);
			put_unaligned_le16(n, buf + 8);
			memcpy(buf + 0x10, elvs[i].body + 6, 2 * n);
			memcpy(buf + 0x60, elvs[i].body + 6 + 2 * n, 4 * n);
			panelcal_header(hdr, PANELCAL_ELVS_BUF);
			ret = panelcal_send(dcp, PANELCAL_BLOCK_ELVS_LUTS, "ELVS LUT at", elvs[i].key,
					    hdr, sizeof(hdr), buf, PANELCAL_ELVS_BUF);
			sent += !ret;
		}
		if (ret)
			dev_warn(dcp->dev, "panel calibration: ELVS stopped: %d\n", ret);
	}

	/* IR-drop compensation: the device data of each refresh-rate mode */
	for (i = 0; i < nirdc; i++) {
		unsigned int n = get_unaligned_le16(irdc[i].body + 4);

		if (n > PANELCAL_IRDC_TAPS ||
		    irdc[i].len < 6 + (size_t)n * PANELCAL_IRDC_TAP_BYTES) {
			ret = -EINVAL;
			break;
		}
		memset(buf, 0, PANELCAL_IRDC_BUF);
		put_unaligned_le32(3, buf);
		put_unaligned_le16(irdc[i].key, buf + 4);
		put_unaligned_le16(get_unaligned_le16(irdc[i].body), buf + 6);
		put_unaligned_le16(n, buf + 8);
		memcpy(buf + 0xa, irdc[i].body + 6, (size_t)n * PANELCAL_IRDC_TAP_BYTES);
		panelcal_header(hdr, PANELCAL_IRDC_BUF);
		ret = panelcal_send(dcp, PANELCAL_BLOCK_IRDC_DATA, "IRDC device data, RR mode",
				    irdc[i].key, hdr, sizeof(hdr), buf, PANELCAL_IRDC_BUF);
		if (ret)
			break;
		sent++;
	}
	kvfree(buf);
	dev_info(dcp->dev, "panel calibration: %u blocks with a buffer accepted, last result %d\n",
		 sent, ret);
out:
	release_firmware(fw);
	return ret;
}
