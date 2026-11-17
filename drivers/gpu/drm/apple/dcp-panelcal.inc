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
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/sort.h>

#define PANELCAL_FW		"apple/panel-adcl.bin"
/* The model's IR-drop vendor data: macOS's /System/Library/Display/<model>/<index>-irdc.bin */
#define PANELCAL_FW_IRDC	"apple/panel-irdc.bin"
/* Likewise <index>-lcdca.bin and <index>-ptuc.bin; both optional */
#define PANELCAL_FW_CA		"apple/panel-lcdca.bin"
#define PANELCAL_FW_PTUC	"apple/panel-ptuc.bin"
#define PANELCAL_TAG(a, b, c, d) ((u32)(d) | (u32)(c) << 8 | (u32)(b) << 16 | (u32)(a) << 24)
#define PANELCAL_IRDC		PANELCAL_TAG('I', 'R', 'D', 'C')
#define PANELCAL_ELVS		PANELCAL_TAG('E', 'L', 'V', 'S')
#define PANELCAL_ADCL		PANELCAL_TAG('A', 'D', 'C', 'L')
#define PANELCAL_IRDF		PANELCAL_TAG('I', 'R', 'D', 'f')
#define PANELCAL_CA		PANELCAL_TAG(' ', ' ', 'C', 'A')
#define PANELCAL_PTUP		PANELCAL_TAG('P', 'T', 'U', 'p')
#define PANELCAL_PTUC		PANELCAL_TAG('P', 'T', 'U', 'C')

#define PANELCAL_BLOCK_IRDC_DATA	118
#define PANELCAL_BLOCK_CA		8
#define PANELCAL_BLOCK_PTUC_CONFIG	115
#define PANELCAL_BLOCK_PTUC_LUTS	116
#define PANELCAL_BLOCK_IRDC_VENDOR	119
#define PANELCAL_BLOCK_ELVS_CONFIG	122
#define PANELCAL_BLOCK_ELVS_LUTS	123

#define PANELCAL_IRDC_TAPS	45
#define PANELCAL_IRDC_TAP_BYTES	(3 * 257 * 4)
#define PANELCAL_IRDC_BUF	0x24000
#define PANELCAL_IRDV_BUF	0x6c000
#define PANELCAL_IRDV_REPS	10
#define PANELCAL_CA_SIZE	0xc24
#define PANELCAL_CA_RLUT	600
#define PANELCAL_PTUC_CFG	125
#define PANELCAL_PTUC_BUF	0x4000
#define PANELCAL_PTUC_BODY	(96 + 2 * 3 * 2 * 3 * 11 * 21)
#define PANELCAL_PTUC_NITS	16
#define PANELCAL_PTUC_MAX	32
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

/* One record of a container: false at the end or at a record that does not fit */
static bool panelcal_next(const u8 *p, size_t end, size_t *off, u32 *tag, u16 *ver,
			  const u8 **body, size_t *len)
{
	size_t l;

	if (*off + 12 > end)
		return false;
	l = (size_t)get_unaligned_le32(p + *off + 8) * 4;
	if (l < 12 || l > end - *off)
		return false;
	*tag = get_unaligned_le32(p + *off);
	*ver = get_unaligned_le16(p + *off + 6);
	*body = p + *off + 12;
	*len = l - 12;
	*off += l;
	return true;
}

/* A system-volume file: no wrapper, the container's record first */
static int panelcal_file(struct apple_dcp *dcp, const char *name, const struct firmware **fw,
			 size_t *end)
{
	int ret = firmware_request_nowarn(fw, name, dcp->dev);

	if (ret)
		return ret;
	if ((*fw)->size < 24 || get_unaligned_le32((*fw)->data) != PANELCAL_ADCL) {
		release_firmware(*fw);
		return -EINVAL;
	}
	*end = min_t(size_t, (*fw)->size, (size_t)get_unaligned_le32((*fw)->data + 8) * 4);
	return 0;
}

/* Color adjustment data (block 8): the record's sub-records, each at its place */
static const struct {
	u32 tag;
	u16 off, words;
} panelcal_ca_map[] = {
	{ PANELCAL_TAG('I', 'R', 'E', 'X'), 0x004, 3 },
	{ PANELCAL_TAG('S', 'I', 'E', 'X'), 0x010, 3 },
	{ PANELCAL_TAG('S', '1', 'E', 'X'), 0x01c, 3 },
	{ PANELCAL_TAG('S', '2', 'E', 'X'), 0x028, 3 },
	{ PANELCAL_TAG('S', '3', 'E', 'X'), 0x034, 3 },
	{ PANELCAL_TAG('R', 'L', 'U', 'T'), 0x044, PANELCAL_CA_RLUT },
	{ PANELCAL_TAG('C', 'R', 'X', 'P'), 0x9a4, 1 },
	{ PANELCAL_TAG('C', 'R', 'V', 'L'), 0x9a8, 1 },
	{ PANELCAL_TAG('C', 'T', 'X', 'P'), 0x9ac, 1 },
	{ PANELCAL_TAG('C', 'T', 'V', 'L'), 0x9b0, 4 },
	{ PANELCAL_TAG('V', 'L', 'E', 'X'), 0x9c0, 1 },
	{ PANELCAL_TAG('V', 'L', 'V', 'L'), 0x9c4, 65 },
	{ PANELCAL_TAG('V', 'I', 'E', 'X'), 0xac8, 1 },
	{ PANELCAL_TAG('V', '1', 'E', 'X'), 0xacc, 1 },
	{ PANELCAL_TAG('V', '2', 'E', 'X'), 0xad0, 1 },
	{ PANELCAL_TAG('V', '3', 'E', 'X'), 0xad4, 1 },
	{ PANELCAL_TAG('R', 'F', 'E', 'X'), 0xad8, 1 },
	{ PANELCAL_TAG('R', 'D', 'S', 'F'), 0xadc, 1 },
	{ PANELCAL_TAG('P', 'R', 'C', 'W'), 0xae0, 16 },
	{ PANELCAL_TAG('P', 'R', 'C', 'D'), 0xb20, 16 },
	{ PANELCAL_TAG('M', 'S', 'F', 'D'), 0xb60, 1 },
};

static int dcp_panelcal_ca(struct apple_dcp *dcp)
{
	const struct firmware *fw;
	const u8 *body, *sb;
	size_t end, off = 12, len, soff, slen;
	u32 tag = 0, version = 1;
	unsigned int i;
	u16 ver;
	u8 *ca;
	int ret;

	ret = panelcal_file(dcp, PANELCAL_FW_CA, &fw, &end);
	if (ret)
		return ret;
	ret = -EINVAL;
	while (panelcal_next(fw->data, end, &off, &tag, &ver, &body, &len))
		if (tag == PANELCAL_CA)
			break;
	if (tag != PANELCAL_CA)
		goto out;
	ret = -ENOMEM;
	ca = kzalloc(PANELCAL_CA_SIZE, GFP_KERNEL);
	if (!ca)
		goto out;
	/* The tail as macOS's loader leaves it */
	memset(ca + 0xb64, 0xaa, PANELCAL_CA_SIZE - 0xb64);
	for (soff = 0; panelcal_next(body, len, &soff, &tag, &ver, &sb, &slen);) {
		for (i = 0; i < ARRAY_SIZE(panelcal_ca_map); i++)
			if (panelcal_ca_map[i].tag == tag)
				break;
		if (i == ARRAY_SIZE(panelcal_ca_map))
			continue;
		memcpy(ca + panelcal_ca_map[i].off, sb,
		       min_t(size_t, slen, 4 * panelcal_ca_map[i].words));
		if (tag == PANELCAL_TAG('R', 'L', 'U', 'T'))
			put_unaligned_le32(slen / 4, ca + 0x40);
		if (tag == PANELCAL_TAG('M', 'S', 'F', 'D'))
			version = 2;
	}
	put_unaligned_le32(version, ca);
	dev_info(dcp->dev, "panel calibration: CA data crc %#x\n",
		 crc32_le(~0, ca, PANELCAL_CA_SIZE) ^ ~0U);
	ret = panelcal_send(dcp, PANELCAL_BLOCK_CA, "CA data, version", version, ca,
			    PANELCAL_CA_SIZE, NULL, 0);
	kfree(ca);
out:
	release_firmware(fw);
	return ret;
}

/*
 * PTUC (blocks 115, 116): a configuration built from the PTUp record and the LUT records'
 * brightness and refresh-rate values, then each LUT record with its place in those two lists.
 */
static int panelcal_u16_cmp(const void *a, const void *b)
{
	return *(const u16 *)a - *(const u16 *)b;
}

static int dcp_panelcal_ptuc(struct apple_dcp *dcp)
{
	const u8 *lut[PANELCAL_PTUC_MAX], *body, *ptup = NULL;
	u16 nits[PANELCAL_PTUC_NITS], rrs[2];
	unsigned int nlut = 0, nnits = 0, nrr = 0, i, k, sent = 0;
	size_t end, off = 12, len;
	const struct firmware *fw;
	u8 cfg[PANELCAL_PTUC_CFG], hdr[20];
	int r, ret;
	u32 tag;
	u16 ver;
	u8 *buf;

	ret = panelcal_file(dcp, PANELCAL_FW_PTUC, &fw, &end);
	if (ret)
		return ret;
	while (panelcal_next(fw->data, end, &off, &tag, &ver, &body, &len)) {
		if (tag == PANELCAL_PTUP && ver == 3 && len >= 48) {
			ptup = body;
		} else if (tag == PANELCAL_PTUC && ver == 3 && len >= PANELCAL_PTUC_BODY &&
			   nlut < ARRAY_SIZE(lut)) {
			u16 b = get_unaligned_le16(body + 8), rr = get_unaligned_le16(body + 10);

			for (i = 0; i < nnits && nits[i] != b; i++)
				;
			if (i == nnits && nnits < ARRAY_SIZE(nits))
				nits[nnits++] = b;
			else if (i == nnits)
				continue;
			for (i = 0; i < nrr && rrs[i] != rr; i++)
				;
			if (i == nrr && nrr < ARRAY_SIZE(rrs))
				rrs[nrr++] = rr;
			else if (i == nrr)
				continue;
			lut[nlut++] = body;
		}
	}
	ret = -EINVAL;
	if (!ptup || !nlut)
		goto out;
	sort(nits, nnits, sizeof(nits[0]), panelcal_u16_cmp, NULL);
	sort(rrs, nrr, sizeof(rrs[0]), panelcal_u16_cmp, NULL);
	memset(cfg, 0, sizeof(cfg));
	put_unaligned_le32(3, cfg);
	memcpy(cfg + 9, ptup, 48);
	for (i = 0; i < nnits; i++)
		put_unaligned_le16(nits[i], cfg + 57 + 2 * i);
	put_unaligned_le16(nnits, cfg + 89);
	put_unaligned_le16(nrr, cfg + 91);
	for (i = 0; i < nrr; i++)
		put_unaligned_le16(rrs[i], cfg + 93 + 2 * i);
	dev_info(dcp->dev, "panel calibration: PTUC config crc %#x, %u LUTs\n",
		 crc32_le(~0, cfg, sizeof(cfg)) ^ ~0U, nlut);
	ret = panelcal_send(dcp, PANELCAL_BLOCK_PTUC_CONFIG, "PTUC config, brightness values",
			    nnits, cfg, sizeof(cfg), NULL, 0);
	if (ret)
		goto out;
	ret = -ENOMEM;
	buf = kvzalloc(PANELCAL_PTUC_BUF, GFP_KERNEL);
	if (!buf)
		goto out;
	ret = 0;
	/* macOS's order: the higher refresh-rate flag first, brightness ascending in each */
	for (r = nrr - 1; !ret && r >= 0; r--)
		for (k = 0; !ret && k < nnits; k++)
			for (i = 0; !ret && i < nlut; i++) {
				if (get_unaligned_le16(lut[i] + 8) != nits[k] ||
				    get_unaligned_le16(lut[i] + 10) != rrs[r])
					continue;
				memset(buf, 0, PANELCAL_PTUC_BUF);
				put_unaligned_le32(3, buf);
				memcpy(buf + 0x9, lut[i], 2);
				put_unaligned_le64(0xffffffffULL, buf + 0x11);
				put_unaligned_le16(nits[k], buf + 0x19);
				put_unaligned_le16(rrs[r], buf + 0x1b);
				put_unaligned_le16(k, buf + 0x1d);
				put_unaligned_le16(r, buf + 0x1f);
				memcpy(buf + 0x21, lut[i] + 12, PANELCAL_PTUC_BODY - 12);
				panelcal_header(hdr, PANELCAL_PTUC_BUF);
				ret = panelcal_send(dcp, PANELCAL_BLOCK_PTUC_LUTS, "PTUC LUT",
						    k * 10 + r, hdr, sizeof(hdr), buf,
						    PANELCAL_PTUC_BUF);
				sent += !ret;
			}
	kvfree(buf);
	dev_info(dcp->dev, "panel calibration: %u PTUC LUTs accepted, last result %d\n", sent, ret);
out:
	release_firmware(fw);
	return ret;
}

/*
 * IR-drop vendor data: an IRDf record's fields follow one another at the record's own tap and
 * repetition counts; the firmware's struct has them at fixed capacities (45 taps, 10 repetitions).
 */
static int panelcal_irdc_vendor(u8 *d, const u8 *s, size_t len)
{
	const u8 *e = s + len;
	unsigned int n, q, i, j;

	if (len < 6)
		return -EINVAL;
	n = get_unaligned_le16(s + 4);
	if (n > PANELCAL_IRDC_TAPS)
		return -EINVAL;
	memset(d, 0, PANELCAL_IRDV_BUF);
	put_unaligned_le32(1, d);
	memcpy(d + 0x4, s + 2, 2);		/* RR mode */
	memcpy(d + 0x6, s, 2);
	memcpy(d + 0x8, s + 4, 2);
	s += 6;
#define TAKE(dst, bytes) do {				\
	if ((size_t)(e - s) < (size_t)(bytes))		\
		return -EINVAL;				\
	memcpy(d + (dst), s, (bytes));			\
	s += (bytes);					\
} while (0)
	TAKE(0xa, 4 * n);				/* the taps' brightness values */
	TAKE(0xbe, 4);
	TAKE(0xc2, 2 * 16);
	for (j = 0; j < 5; j++)
		for (i = 0; i < n; i++)
			TAKE(0xe2 + j * 4320 + i * 96, 96);
	for (j = 0; j < 5; j++)
		for (i = 0; i < n; i++)
			TAKE(0x5542 + j * 4320 + i * 96, 96);
	for (j = 0; j < 5; j++)
		for (i = 0; i < n; i++)
			TAKE(0xa9a2 + j * 1440 + i * 32, 32);
	TAKE(0xc5c2, 2 * 3);
	TAKE(0xc5ca, 4 * 20 * 10 * 21 * 11);
	TAKE(0x397aa, 4 * 20 * 10 * 21 * 11);
	TAKE(0x6698a, 2 * 21 * 11);
	TAKE(0x66b58, 96 * n);
	TAKE(0x67c38, 2);
	if ((size_t)(e - s) < 2)
		return -EINVAL;
	q = get_unaligned_le16(s);
	if (q > PANELCAL_IRDV_REPS)
		return -EINVAL;
	TAKE(0x67c3a, 2);
	/* The file's first per-tap list is the struct's second */
	for (i = 0; i < n; i++)
		TAKE(0x67fc0 + i * 2 * PANELCAL_IRDV_REPS, 2 * q);
	for (i = 0; i < n; i++)
		TAKE(0x67c3c + i * 2 * PANELCAL_IRDV_REPS, 2 * q);
	TAKE(0x68344, 4);
	TAKE(0x68348, 2 * 21);
	TAKE(0x68372, 2 * 4 * 64);
	TAKE(0x68572, 2 * 5);
#undef TAKE
	return 0;
}

static int dcp_panelcal_vendor(struct apple_dcp *dcp)
{
	const struct firmware *fw;
	const u8 *p, *end;
	unsigned int sent = 0;
	size_t off;
	u8 hdr[20];
	u8 *buf;
	int ret;

	ret = firmware_request_nowarn(&fw, PANELCAL_FW_IRDC, dcp->dev);
	if (ret)
		return ret;
	p = fw->data;
	ret = -EINVAL;
	if (fw->size < 24 || get_unaligned_le32(p) != PANELCAL_ADCL)
		goto out;
	end = p + min_t(size_t, fw->size, (size_t)get_unaligned_le32(p + 8) * 4);
	buf = kvzalloc(PANELCAL_IRDV_BUF, GFP_KERNEL);
	ret = -ENOMEM;
	if (!buf)
		goto out;
	ret = 0;
	for (off = 12; !ret && p + off + 12 <= end;) {
		u32 tag = get_unaligned_le32(p + off);
		u16 ver = get_unaligned_le16(p + off + 6);
		size_t len = (size_t)get_unaligned_le32(p + off + 8) * 4;

		if (len < 12 || len > (size_t)(end - (p + off)))
			break;
		if (tag == PANELCAL_IRDF && ver == 1) {
			ret = panelcal_irdc_vendor(buf, p + off + 12, len - 12);
			if (ret)
				break;
			panelcal_header(hdr, PANELCAL_IRDV_BUF);
			ret = panelcal_send(dcp, PANELCAL_BLOCK_IRDC_VENDOR,
					    "IRDC vendor data, RR mode", get_unaligned_le16(buf + 4),
					    hdr, sizeof(hdr), buf, PANELCAL_IRDV_BUF);
			sent += !ret;
		}
		off += len;
	}
	kvfree(buf);
	dev_info(dcp->dev, "panel calibration: %u IRDC vendor blocks accepted, last result %d\n",
		 sent, ret);
out:
	release_firmware(fw);
	return ret;
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
	ret = firmware_request_nowarn(&fw, PANELCAL_FW, dcp->dev);
	if (ret)
		return ret;
	/* The model's optional tables first, in macOS's order; a failure there stops nothing */
	ret = dcp_panelcal_ca(dcp);
	if (ret && ret != -ENOENT)
		dev_warn(dcp->dev, "panel calibration: CA data: %d\n", ret);
	ret = dcp_panelcal_ptuc(dcp);
	if (ret && ret != -ENOENT)
		dev_warn(dcp->dev, "panel calibration: PTUC: %d\n", ret);
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
	/* Vendor data only behind the device data: the firmware compares the tap counts */
	if (!ret)
		ret = dcp_panelcal_vendor(dcp);
	return ret;
}

/*
 * After each firmware start. The files may not be reachable yet at boot (root not mounted):
 * tried again a few times, then given up quietly, as on a machine that has none.
 */
#define PANELCAL_TRIES		12
#define PANELCAL_RETRY_MS	5000

static void dcp_panelcal_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp, panelcal_wq);
	int ret;

	if (!READ_ONCE(dcp->active) || dcp->crashed)
		return;
	ret = dcp_panelcal_load(dcp);
	if (ret == -ENOENT && ++dcp->panelcal_tries < PANELCAL_TRIES)
		schedule_delayed_work(&dcp->panelcal_wq, msecs_to_jiffies(PANELCAL_RETRY_MS));
	else if (ret == -ENOENT)
		dev_info(dcp->dev, "panel calibration: no %s or %s: the panel runs without its compensation tables\n",
			 PANELCAL_FW, PANELCAL_FW_IRDC);
	else if (ret)
		dev_warn(dcp->dev, "panel calibration: not loaded: %d\n", ret);
}

void dcp_panelcal_schedule(struct apple_dcp *dcp, unsigned int delay_ms)
{
	if (dcp->fw_compat != DCP_FIRMWARE_V_27_0 || !of_machine_is_compatible("apple,k116s"))
		return;
	dcp->panelcal_tries = 0;
	mod_delayed_work(system_wq, &dcp->panelcal_wq, msecs_to_jiffies(delay_ms));
}
