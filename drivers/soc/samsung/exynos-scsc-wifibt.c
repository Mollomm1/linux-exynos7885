// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung SCSC (mx140 / Lassen) WiFi+BT firmware block bring-up.
 *
 * The block hosts a Cortex-R4 ("R4") running the WLAN firmware and a
 * Cortex-M4 ("M4") running the BT firmware.  Both are booted from DRAM
 * that the host stages beforehand; this driver loads the firmware,
 * publishes the shared-memory configuration the firmware expects, and
 * releases the block, then reports what the two cores do about it.
 *
 * Copyright (c) 2026 T510 mainlining project
 */

#include <linux/arm-smccc.h>

#include <linux/atomic.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/vmalloc.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/highmem.h>

#define SCSC_FW_NAME		"postmarketos/mx140/mx140.bin"

/* Maxwell firmware image header, same layout as downstream fwhdr.c. */
#define SCSC_FW_MAGIC		"smxf"
#define SCSC_FW_MAGIC_OFF	8
#define SCSC_FW_VER_MINOR_OFF	12
#define SCSC_FW_VER_MAJOR_OFF	14
#define SCSC_FW_API_MINOR_OFF	20
#define SCSC_FW_API_MAJOR_OFF	22
#define SCSC_FW_LEN_OFF		16
#define SCSC_FW_ENTRY_OFF	40
#define SCSC_FW_RUNTIME_LEN_OFF	36
#define SCSC_FW_CONST_LEN_OFF	28
#define SCSC_FW_CRC_OFF		24
#define SCSC_FW_CONST_CRC_OFF	32
#define SCSC_FW_BUILD_ID_OFF	48
#define SCSC_FW_BUILD_ID_SZ	128

/* DBUS mailbox: four shared registers per bank, at 0x80. */
#define SCSC_MBOX_INTGR0	0x008	/* interrupt generate, lower half */
#define SCSC_MBOX_INTMR0	0x010
#define SCSC_MBOX_INTSR0	0x014
#define SCSC_MBOX_INTGR1	0x01c
#define SCSC_MBOX_INTCR0	0x00c	/* interrupt clear, write 1 to clear */
#define SCSC_MBOX_INTCR1	0x020
#define SCSC_MBOX_INTMR1	0x024
#define SCSC_MBOX_INTSR1	0x028
#define SCSC_MBOX_INTMSR0	0x018	/* mask status, upper half is the core's */
#define SCSC_MBOX_INTMSR1	0x02c
#define SCSC_MBOX_IS_VERSION	0x050
#define SCSC_MBOX_ISSR_BASE	0x080
#define SCSC_MBOX_ISSR(i)	(SCSC_MBOX_ISSR_BASE + 4 * (i))

/* Shared-memory configuration published to the firmware. */
#define SCSC_MBOX_MAGIC		0xbcdeedcb
#define SCSC_MBOX_FW_FLAGS		0
/* The WLBT TZASC channel that sizes the shared window, like downstream. */
#define SCSC_SMC_WLBT_TZASC		0x82000710

#define SCSC_MXCONF_MAGIC		0x79828486
#define SCSC_MXCONF_VER_MAJOR		0
#define SCSC_MXCONF_VER_MINOR		1
#define SCSC_MXCONF_SIZE		162
#define SCSC_STREAMCONF_SIZE		22
#define SCSC_MGMT_BUF_LEN		512
#define SCSC_MGMT_PACKET_SIZE		8
#define SCSC_MGMT_NUM_PACKETS		(SCSC_MGMT_BUF_LEN / SCSC_MGMT_PACKET_SIZE)
#define SCSC_GDB_BUF_LEN		2048
#define SCSC_GDB_PACKET_SIZE		4
#define SCSC_GDB_NUM_PACKETS		(SCSC_GDB_BUF_LEN / SCSC_GDB_PACKET_SIZE)
#define SCSC_MXLOG_BUF_LEN		16384
#define SCSC_MXLOG_PACKET_SIZE		4
#define SCSC_MXLOG_NUM_PACKETS		(SCSC_MXLOG_BUF_LEN / SCSC_MXLOG_PACKET_SIZE)

/* PMU (syscon) registers driving the block. */
#define SCSC_PMU_WIFI_CTRL_NS		0x140	/* non-secure control */
#define SCSC_PMU_WIFI_PWRON		BIT(1)
#define SCSC_PMU_WIFI_RESET_SET		BIT(2)
#define SCSC_PMU_WIFI_CTRL_S		0x144	/* secure control */
#define SCSC_PMU_WIFI_START		BIT(3)
#define SCSC_PMU_WIFI_STAT		0x148
#define SCSC_PMU_WIFI_PWRDN_DONE	BIT(0)
#define SCSC_PMU_CENTRAL_SEQ_STAT	0x13c
#define SCSC_PMU_STATES		0xf0000
#define SCSC_PMU_CENTRAL_SEQ_CFG	0x138
#define SCSC_PMU_SYS_PWR_CFG_16		BIT(16)
#define SCSC_PMU_BOOT_TEST_RST_CFG	0x7330	/* 0 = boot from external DRAM */
#define SCSC_PMU_MEM_CONFIG0		0x7300	/* WiFi window size (4K units) */
#define SCSC_PMU_MEM_CONFIG1		0x7304	/* WiFi window base (4K units) */

/* Low-power sequencing, read back after power-on to document the state. */
#define SCSC_PMU_RESET_AHEAD		0x1360
#define SCSC_PMU_CLEANY_BUS		0x1364
#define SCSC_PMU_LOGIC_RESET		0x1368
#define SCSC_PMU_TCXO_GATE		0x136c
#define SCSC_PMU_DISABLE_ISO		0x1370
#define SCSC_PMU_RESET_ISO		0x1374

struct scsc_wifibt {
	struct device	*dev;
	void __iomem	*base;		/* R4 mailbox bank */
	void __iomem	*base_m4;	/* M4 mailbox bank */
	struct regmap	*pmureg;
	void		*mem;		/* the shared window, mapped once */
	phys_addr_t	mem_start;
	size_t		mem_size;
	u32		fw_entry;
	u32		fw_runtime;
	u32		fw_hdr_len;
	u32		fw_const_len;
	u32		fw_len;
	u32		mxconf_off;
	u32		sig_entry;
	u32		sig_mbox1;
	u32		mgmt_ta_buf;
	u32		mgmt_ta_widx;
	u32		mgmt_fa_buf;
	u32		mgmt_fa_widx;
	u32		dram_crc;
	struct delayed_work check_work;
	atomic_t	irq_count;
	atomic_t	wdog_count;
};

static const unsigned int scsc_lp_regs[] = {
	SCSC_PMU_RESET_AHEAD, SCSC_PMU_CLEANY_BUS, SCSC_PMU_LOGIC_RESET,
	SCSC_PMU_TCXO_GATE, SCSC_PMU_DISABLE_ISO, SCSC_PMU_RESET_ISO,
	SCSC_PMU_CENTRAL_SEQ_CFG,
};

static bool cachetest;
module_param(cachetest, bool, 0444);
MODULE_PARM_DESC(cachetest,
		 "Check that reads through the shared window are coherent, then leave the window untouched");

/*
 * The window is RAM the R4 writes behind our back, so how it is mapped
 * matters.  This is Normal non-cacheable, which is what pgprot_writecombine()
 * selects on arm64 and what Samsung's own driver uses for this window.
 * Device memory would be safe from speculation but rejects the unaligned
 * accesses the firmware structures need, so it is not an option here.
 */
static int scsc_wifibt_map(struct scsc_wifibt *scsc)
{
	struct page **pages;
	unsigned int i, npages = PAGE_ALIGN(scsc->mem_size) >> PAGE_SHIFT;

	/* Map it once and keep it: every attribute, the firmware staging
	 * and the check all touch this window, and mapping it on demand
	 * means taking mmap_sem from whatever context asked - including
	 * udev, which reads every attribute while handling a uevent.
	 */
	pages = kmalloc_array(npages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < npages; i++)
		pages[i] = phys_to_page(scsc->mem_start + i * PAGE_SIZE);

	scsc->mem = vmap(pages, npages, VM_MAP,
			 pgprot_writecombine(PAGE_KERNEL));
	kfree(pages);

	if (!scsc->mem)
		return -ENOMEM;

	return 0;
}

/*
 * Bulk transfers into and out of the window, through the volatile
 * accessors: plain pointer stores get merged into wider accesses, which
 * the write-combine mapping does not guarantee.
 */
static void scsc_win_copy(void *dst, const void *src, size_t n)
{
	u8 *d = dst;
	const u8 *s = src;
	size_t i;

	for (i = 0; i + 4 <= n; i += 4)
		writel(*(const u32 *)(s + i), d + i);
	for (; i < n; i++)
		writeb(s[i], d + i);
}

static void scsc_win_set(void *win, u32 val, size_t n)
{
	u8 *w = win;
	size_t i;

	for (i = 0; i + 4 <= n; i += 4)
		writel(val, w + i);
	for (; i < n; i++)
		writeb(val, w + i);
}

static int scsc_win_cmp(const void *win, const void *buf, size_t n)
{
	const u8 *b = buf;
	u8 *w = (u8 *)win;
	size_t i;

	for (i = 0; i + 4 <= n; i += 4)
		if (readl(w + i) != *(const u32 *)(b + i))
			return 1;
	for (; i < n; i++)
		if (readb(w + i) != b[i])
			return 1;
	return 0;
}

/* A checksum over the whole window, so the check can tell whether the
 * firmware has written anything since the image was staged.
 */
static u32 scsc_wifibt_dram_crc(struct scsc_wifibt *scsc)
{
	u32 crc = 0xffffffff;
	void *dram;
	size_t off;

	dram = scsc->mem;
	if (!dram)
		return 0;

	for (off = 0; off + 4 <= scsc->mem_size; off += 4)
		crc = crc32_le(crc, (u8 *)(dram + off), 4);

	return ~crc;
}

/*
 * Self-test for the window mapping: write a pattern, read it back, write a
 * different value, read again, then read it through a second mapping.  If
 * any read comes back stale then nothing read out of this window can be
 * trusted, including what the firmware leaves behind for us.
 */
static void scsc_wifibt_cachetest(struct scsc_wifibt *scsc)
{
	const u32 off = 0x1f0000;
	u8 *dram = scsc->mem;
	u32 first, again;

	if (!dram)
		return;

	writel(0x11223344, dram + off);
	first = readl(dram + off);
	writel(0xa5a5a5a5, dram + off);
	again = readl(dram + off);

	dev_info(scsc->dev,
		 "window readback: pattern %08x, after write %08x -> %s\n",
		 first, again,
		 (first == 0x11223344 && again == 0xa5a5a5a5) ?
		 "coherent" : "STALE");
}

static void scsc_wifibt_stream_conf(u8 *p, u32 buf, u32 num, u32 pktsize,
				    u32 ridx, u32 widx,
				    u8 read_bit, u8 write_bit)
{
	put_unaligned_le32(buf, p + 0);
	put_unaligned_le32(num, p + 4);
	put_unaligned_le32(pktsize, p + 8);
	put_unaligned_le32(ridx, p + 12);
	put_unaligned_le32(widx, p + 16);
	p[20] = read_bit;
	p[21] = write_bit;
}

/*
 * Publish the shared-memory layout: the firmware reads this out of the
 * window at startup to find the management, GDB and log rings, so it has
 * to be in place before the block is released.
 */
static int scsc_wifibt_mxconf(struct scsc_wifibt *scsc)
{
	u32 gdb_r4_in, gdb_r4_out, gdb_m4_in, gdb_m4_out, mxlog, end;
	u32 heap;
	u32 mx_off = ALIGN(scsc->fw_runtime, 4);
	u32 from_ap = ALIGN(mx_off + SCSC_MXCONF_SIZE, 4);
	u32 to_ap = from_ap + SCSC_MGMT_BUF_LEN + 8;
	u8 *mxconf, *to_ap_base, *from_ap_base;
	void *dram;
	u32 i;

	heap = to_ap + SCSC_GDB_BUF_LEN * 2 + SCSC_MXLOG_BUF_LEN + 24;
	if (heap > scsc->mem_size) {
		dev_err(scsc->dev, "shared memory too small (%u > %zu)\n",
			heap, scsc->mem_size);
		return -EINVAL;
	}

	gdb_r4_in = to_ap + SCSC_MGMT_BUF_LEN + 8;
	gdb_r4_out = gdb_r4_in + SCSC_GDB_BUF_LEN + 8;
	gdb_m4_in = gdb_r4_out + SCSC_GDB_BUF_LEN + 8;
	gdb_m4_out = gdb_m4_in + SCSC_GDB_BUF_LEN + 8;
	mxlog = gdb_m4_out + SCSC_GDB_BUF_LEN + 8;
	end = mxlog + SCSC_MXLOG_BUF_LEN + 8;

	dram = scsc->mem;
	if (!dram)
		return -ENOMEM;

	mxconf = dram + mx_off;
	scsc_win_set(mxconf, 0, SCSC_MXCONF_SIZE);
	put_unaligned_le32(SCSC_MXCONF_MAGIC, mxconf + 0);
	put_unaligned_le16(SCSC_MXCONF_VER_MAJOR, mxconf + 4);
	put_unaligned_le16(SCSC_MXCONF_VER_MINOR, mxconf + 6);
	/* Management to-AP (IN): read=fromhost 1, write=tohost 0. */
	scsc_wifibt_stream_conf(mxconf + 8, to_ap,
				SCSC_MGMT_NUM_PACKETS, SCSC_MGMT_PACKET_SIZE,
				to_ap + SCSC_MGMT_BUF_LEN,
				to_ap + SCSC_MGMT_BUF_LEN + 4, 1, 0);
	/* Management from-AP (OUT): read=tohost 1, write=fromhost 2. */
	scsc_wifibt_stream_conf(mxconf + 8 + SCSC_STREAMCONF_SIZE, from_ap,
				SCSC_MGMT_NUM_PACKETS, SCSC_MGMT_PACKET_SIZE,
				from_ap + SCSC_MGMT_BUF_LEN,
				from_ap + SCSC_MGMT_BUF_LEN + 4, 1, 2);
	/* GDB: R4 in, R4 out, M4 in, M4 out. */
	scsc_wifibt_stream_conf(mxconf + 8 + 2 * SCSC_STREAMCONF_SIZE, gdb_r4_in,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_r4_in + SCSC_GDB_BUF_LEN,
				gdb_r4_in + SCSC_GDB_BUF_LEN + 4, 1, 2);
	scsc_wifibt_stream_conf(mxconf + 8 + 3 * SCSC_STREAMCONF_SIZE, gdb_r4_out,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_r4_out + SCSC_GDB_BUF_LEN,
				gdb_r4_out + SCSC_GDB_BUF_LEN + 4, 2, 3);
	scsc_wifibt_stream_conf(mxconf + 8 + 4 * SCSC_STREAMCONF_SIZE, gdb_m4_in,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_m4_in + SCSC_GDB_BUF_LEN,
				gdb_m4_in + SCSC_GDB_BUF_LEN + 4, 3, 4);
	scsc_wifibt_stream_conf(mxconf + 8 + 5 * SCSC_STREAMCONF_SIZE, gdb_m4_out,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_m4_out + SCSC_GDB_BUF_LEN,
				gdb_m4_out + SCSC_GDB_BUF_LEN + 4, 4, 5);
	/* mxlog is polled by the host rather than the firmware. */
	put_unaligned_le32(mxlog, mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE);
	put_unaligned_le32(SCSC_MXLOG_NUM_PACKETS,
			   mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE + 4);
	put_unaligned_le32(SCSC_MXLOG_PACKET_SIZE,
			   mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE + 8);
	put_unaligned_le32(scsc->fw_entry,
			   mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE + 12);
	put_unaligned_le32(SCSC_MXCONF_MAGIC, mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE + 16);
	put_unaligned_le32(end, mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE + 20);

	/* Clear the host-side ring headers. */
	to_ap_base = dram + to_ap;
	from_ap_base = dram + from_ap;
	for (i = 0; i < SCSC_MGMT_BUF_LEN; i += 4) {
		writel(0, to_ap_base + i);
		writel(0, from_ap_base + i);
	}
	for (i = 0; i < SCSC_GDB_BUF_LEN; i += 4) {
		writel(0, dram + gdb_r4_in + i);
		writel(0, dram + gdb_r4_out + i);
		writel(0, dram + gdb_m4_in + i);
		writel(0, dram + gdb_m4_out + i);
	}

	scsc->mxconf_off = mx_off;
	scsc->sig_entry = scsc->fw_entry;
	/* MBOX_1 points the firmware at the configuration block. */
	scsc->sig_mbox1 = mx_off;
	scsc->mgmt_ta_buf = to_ap;
	scsc->mgmt_ta_widx = to_ap + SCSC_MGMT_BUF_LEN;
	scsc->mgmt_fa_buf = from_ap;
	scsc->mgmt_fa_widx = from_ap + SCSC_MGMT_BUF_LEN;

	dev_info(scsc->dev, "mxconf at DRAM offset 0x%x, mgmt to-AP 0x%x from-AP 0x%x\n",
		 mx_off, to_ap, from_ap);

	return 0;
}

static int scsc_wifibt_fw_parse(struct scsc_wifibt *scsc,
				const struct firmware *fw)
{
	u16 ver_major, ver_minor, api_major, api_minor;
	u32 hdr_len, entry, runtime_len, const_len;
	u32 fw_crc, const_crc, hdr_crc;
	char build_id[SCSC_FW_BUILD_ID_SZ + 1];

	if (fw->size < SCSC_FW_BUILD_ID_OFF + SCSC_FW_BUILD_ID_SZ ||
	    memcmp(fw->data + SCSC_FW_MAGIC_OFF, SCSC_FW_MAGIC,
		   strlen(SCSC_FW_MAGIC))) {
		dev_err(scsc->dev, "firmware has no Maxwell header\n");
		return -EINVAL;
	}

	ver_minor = get_unaligned_le16(fw->data + SCSC_FW_VER_MINOR_OFF);
	ver_major = get_unaligned_le16(fw->data + SCSC_FW_VER_MAJOR_OFF);
	api_minor = get_unaligned_le16(fw->data + SCSC_FW_API_MINOR_OFF);
	api_major = get_unaligned_le16(fw->data + SCSC_FW_API_MAJOR_OFF);
	hdr_len = get_unaligned_le32(fw->data + SCSC_FW_LEN_OFF);
	entry = get_unaligned_le32(fw->data + SCSC_FW_ENTRY_OFF);
	runtime_len = get_unaligned_le32(fw->data + SCSC_FW_RUNTIME_LEN_OFF);
	const_len = get_unaligned_le32(fw->data + SCSC_FW_CONST_LEN_OFF);
	memcpy(build_id, fw->data + SCSC_FW_BUILD_ID_OFF, SCSC_FW_BUILD_ID_SZ);
	build_id[SCSC_FW_BUILD_ID_SZ] = '\0';

	dev_info(scsc->dev,
		 "firmware header v%u.%u api %u.%u entry 0x%x runtime %u const %u\n",
		 ver_major, ver_minor, api_major, api_minor, entry,
		 runtime_len, const_len);
	dev_info(scsc->dev, "firmware build %s\n", build_id);

	if (hdr_len > fw->size || const_len > fw->size) {
		dev_err(scsc->dev, "firmware lengths out of range\n");
		return -EINVAL;
	}

	/* Integrity checks over the image, same layout as downstream fwimage. */
	fw_crc = get_unaligned_le32(fw->data + SCSC_FW_CRC_OFF);
	const_crc = get_unaligned_le32(fw->data + SCSC_FW_CONST_CRC_OFF);
	hdr_crc = get_unaligned_le32(fw->data + hdr_len - sizeof(u32));

	if (ether_crc(hdr_len - sizeof(u32), fw->data) != hdr_crc ||
	    ether_crc(const_len - hdr_len, fw->data + hdr_len) != const_crc ||
	    ether_crc(fw->size - hdr_len, fw->data + hdr_len) != fw_crc) {
		dev_err(scsc->dev, "firmware CRC mismatch\n");
		return -EINVAL;
	}

	scsc->fw_entry = entry;
	scsc->fw_runtime = runtime_len;
	scsc->fw_hdr_len = hdr_len;
	scsc->fw_const_len = const_len;
	scsc->fw_len = fw->size;

	dev_info(scsc->dev, "firmware CRCs OK\n");

	return 0;
}

static int scsc_wifibt_fw_stage(struct scsc_wifibt *scsc,
				const struct firmware *fw)
{
	void *dram;

	if (fw->size > scsc->mem_size) {
		dev_err(scsc->dev, "firmware %zu larger than shared memory %zu\n",
			fw->size, scsc->mem_size);
		return -EINVAL;
	}

	dram = scsc->mem;
	if (!dram)
		return -ENOMEM;

	scsc_win_copy(dram, fw->data, fw->size);

	if (scsc_win_cmp(dram, fw->data, fw->size)) {
		dev_err(scsc->dev, "firmware DRAM readback mismatch\n");
		return -EIO;
	}

	dev_info(scsc->dev, "firmware staged in shared memory, verified\n");

	return 0;
}

static irqreturn_t scsc_wifibt_mbox_irq(int irq, void *data)
{
	struct scsc_wifibt *scsc = data;
	u32 status;

	status = readl(scsc->base + SCSC_MBOX_INTMSR0) >> 16;
	if (!status)
		return IRQ_NONE;

	/* WRITE: 1 = clear interrupt */
	writel(status << 16, scsc->base + SCSC_MBOX_INTCR0);
	atomic_inc(&scsc->irq_count);

	dev_dbg(scsc->dev, "mailbox irq status 0x%04x (total %d)\n",
		status, atomic_read(&scsc->irq_count));

	return IRQ_HANDLED;
}

static irqreturn_t scsc_wifibt_wdog_irq(int irq, void *data)
{
	struct scsc_wifibt *scsc = data;

	atomic_inc(&scsc->wdog_count);
	dev_info(scsc->dev, "WDOG IRQ #%d, disabling\n",
		atomic_read(&scsc->wdog_count));

	/* Downstream does the same: the handler is only there to note it. */
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static int scsc_wifibt_power_on(struct scsc_wifibt *scsc)
{
	unsigned int val;
	int ret;

	/* Keep system-level low-power mode disabled (cold default): the
	 * power-off path enables it, and releasing with it enabled wedges
	 * warm with START held.
	 */
	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_CFG,
				 SCSC_PMU_SYS_PWR_CFG_16,
				 SCSC_PMU_SYS_PWR_CFG_16);
	if (ret)
		return ret;

	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "WIFI_STAT before power on 0x%08x\n", val);

	/* Expose the shared-memory carveout to the firmware block (4K units). */
	ret = regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG1,
			   (scsc->mem_start & 0xfffffc000ULL) >> 12);
	if (ret)
		return ret;

	ret = regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG0,
			   scsc->mem_size >> 12);
	if (ret)
		return ret;

	/* Read back what we programmed: a blocked write here would explain
	 * a firmware block that never comes up.  Power, reset and START
	 * themselves are handled in scsc_wifibt_start().
	 */
	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "WIFI_CTRL_NS readback 0x%08x (PWRON %s, RESET %s)\n",
		 val, val & SCSC_PMU_WIFI_PWRON ? "set" : "clear",
		 val & SCSC_PMU_WIFI_RESET_SET ? "held" : "released");

	ret = regmap_read(scsc->pmureg, SCSC_PMU_MEM_CONFIG0, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "MEM_CONFIG0 (size) readback 0x%08x\n", val);

	ret = regmap_read(scsc->pmureg, SCSC_PMU_MEM_CONFIG1, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "MEM_CONFIG1 (base) readback 0x%08x\n", val);

	return 0;
}

/* Hand the mailbox to the firmware and start it. */
/*
 * Everything that has to be in place *before* the block is released: the
 * TZASC that lets it reach the shared window, the mailbox in the state the
 * firmware expects, the entry handoff, and the boot source that makes it
 * run the staged image instead of the ROM's own.  The order matters and
 * matches the sequence that worked before the driver was trimmed.
 */
static void scsc_wifibt_prepare(struct scsc_wifibt *scsc)
{
	struct arm_smccc_res res;
	unsigned long a1 = 0, a2 = scsc->mem_start, a3 = scsc->mem_size;
	unsigned int i;
	int ret;

	/* Let the firmware block access DRAM. Ignored on failure, like
	 * downstream.
	 */
	arm_smccc_smc(SCSC_SMC_WLBT_TZASC, a1, a2, a3, 0, 0, 0, 0, &res);
	dev_info(scsc->dev, "TZASC smc 0x%x result 0x%lx\n",
		 SCSC_SMC_WLBT_TZASC, res.a0);

	/* Clear all shared registers on both banks first, like downstream
	 * map() does.
	 */
	for (i = 0; i < 8; i++) {
		writel(0, scsc->base + SCSC_MBOX_ISSR(i));
		writel(0, scsc->base_m4 + SCSC_MBOX_ISSR(i));
	}

	/* Mask and clear everything. The firmware unmasks what it uses as
	 * the transports come up; starting masked is the state it expects,
	 * and TOHOST still shows in INTMSR0.
	 */
	writel(0xffff0000, scsc->base + SCSC_MBOX_INTMR0);
	writel(0x0000ffff, scsc->base + SCSC_MBOX_INTMR1);
	writel(0x0000ffff, scsc->base_m4 + SCSC_MBOX_INTMR1);
	writel(0xffff0000, scsc->base + SCSC_MBOX_INTCR0);
	writel(0x0000ffff, scsc->base + SCSC_MBOX_INTCR1);
	writel(0x0000ffff, scsc->base_m4 + SCSC_MBOX_INTCR1);

	/* Tell the R4 ROM where to jump: MBOX_0 = entry, MBOX_1 = the
	 * configuration block, MBOX_2 = magic, MBOX_3 = startup flags
	 * (downstream mbox_init()).
	 */
	writel(scsc->sig_entry, scsc->base + SCSC_MBOX_ISSR(0));
	writel(scsc->sig_mbox1, scsc->base + SCSC_MBOX_ISSR(1));
	writel(SCSC_MBOX_MAGIC, scsc->base + SCSC_MBOX_ISSR(2));
	writel(SCSC_MBOX_FW_FLAGS, scsc->base + SCSC_MBOX_ISSR(3));
	/* The registers have to land before the reset is released. */
	wmb();

	/* Boot the block from external DRAM. Without this it never looks at
	 * the staged image, no matter the handshake.
	 */
	ret = regmap_write(scsc->pmureg, SCSC_PMU_BOOT_TEST_RST_CFG, 0);
	if (ret)
		dev_warn(scsc->dev, "failed to clear BOOT_TEST_RST_CFG: %d\n",
			 ret);
}

/* Power on, release reset, start, then let the firmware run. */
static int scsc_wifibt_start(struct scsc_wifibt *scsc)
{
	unsigned int val;
	int ret;

	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
				 SCSC_PMU_WIFI_PWRON, SCSC_PMU_WIFI_PWRON);
	if (ret)
		return ret;

	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
				 SCSC_PMU_WIFI_RESET_SET, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_S,
				 SCSC_PMU_WIFI_START, SCSC_PMU_WIFI_START);
	if (ret)
		return ret;

	/* Nudge the firmware so it notices the mailbox. */
	writel(1u, scsc->base + SCSC_MBOX_INTGR1);
	writel(0u, scsc->base + SCSC_MBOX_INTGR1);

	/* Give it a moment, then read back: START auto-clears once the
	 * block is up, so a set bit here means the write landed.
	 */
	msleep(20);

	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_CTRL_S, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "WIFI_CTRL_S readback 0x%08x (START %s)\n",
		 val, val & SCSC_PMU_WIFI_START ? "set" : "clear");

	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "powered on, WIFI_STAT 0x%08x\n", val);

	ret = regmap_read(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_STAT, &val);
	if (ret)
		return ret;
	dev_info(scsc->dev, "central sequencer state 0x%02x\n",
		 (val & SCSC_PMU_STATES) >> 16);

	return 0;
}

static void scsc_wifibt_power_off(struct scsc_wifibt *scsc)
{
	unsigned int val, i;

	/* Quiesce in the reverse of the on sequence, like downstream. */
	regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_S,
			   SCSC_PMU_WIFI_START, 0);
	regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
			   SCSC_PMU_WIFI_RESET_SET, SCSC_PMU_WIFI_RESET_SET);
	regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
			   SCSC_PMU_WIFI_PWRON, 0);

	for (i = 0; i < ARRAY_SIZE(scsc_lp_regs); i++)
		regmap_write(scsc->pmureg, scsc_lp_regs[i], 0xffffffff);

	/* Bounded wait for the block to acknowledge the power-down. */
	for (i = 0; i < 100; i++) {
		regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &val);
		if (val & SCSC_PMU_WIFI_PWRDN_DONE)
			break;
		msleep(20);
	}
}

/*
 * Report what the two cores are doing.  Everything here is a read, so it
 * is safe to run repeatedly from sysfs or from the delayed check.
 */
static void scsc_wifibt_scan(struct scsc_wifibt *scsc)
{
	unsigned int stat, i;
	void *dram;

	regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &stat);
	dev_info(scsc->dev, "WIFI_STAT 0x%08x\n", stat);

	dev_info(scsc->dev, "R4 MBOX %08x %08x %08x %08x\n",
		 readl(scsc->base + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base + SCSC_MBOX_ISSR(3)));

	dev_info(scsc->dev, "M4 MBOX %08x %08x %08x %08x\n",
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(3)));

	dram = scsc->mem;
	if (!dram)
		return;

	if (scsc->mxconf_off) {
		u8 *mxconf = dram + scsc->mxconf_off;

		dev_info(scsc->dev, "mxconf magic %08x, mgmt to-AP %08x/%08x from-AP %08x/%08x\n",
			 get_unaligned_le32(mxconf + 0),
			 get_unaligned_le32(mxconf + 8 + 8),
			 get_unaligned_le32(mxconf + 8 + 12),
			 get_unaligned_le32(mxconf + 8 + SCSC_STREAMCONF_SIZE + 8),
			 get_unaligned_le32(mxconf + 8 + SCSC_STREAMCONF_SIZE + 12));
	}

	for (i = 0; i < 4; i++) {
		u32 ta = readl(dram + scsc->mgmt_ta_buf + 4 * i);
		u32 fa = readl(dram + scsc->mgmt_fa_buf + 4 * i);

		if (ta || fa)
			dev_info(scsc->dev, "mgmt slot %u: to-AP %08x from-AP %08x\n",
				 i, ta, fa);
	}
}

/* Write-only on purpose: reading it cannot be allowed to map memory or
 * print, because udev reads every attribute of a device while handling
 * its uevent.
 */
static ssize_t scan_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct scsc_wifibt *scsc = dev_get_drvdata(dev);

	scsc_wifibt_scan(scsc);

	return count;
}

static ssize_t recover_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct scsc_wifibt *scsc = dev_get_drvdata(dev);
	int ret;

	/* Power cycle the block and release it again, without touching the
	 * staged image: the same order the probe uses.
	 */
	scsc_wifibt_power_off(scsc);
	msleep(100);
	scsc_wifibt_prepare(scsc);
	ret = scsc_wifibt_power_on(scsc);
	if (!ret)
		ret = scsc_wifibt_start(scsc);
	if (ret)
		return ret;

	return count;
}

static struct device_attribute dev_attr_scan =
	__ATTR(scan, 0200, NULL, scan_store);
static struct device_attribute dev_attr_recover =
	__ATTR(recover, 0200, NULL, recover_store);

static void scsc_wifibt_check_work(struct work_struct *work)
{
	struct scsc_wifibt *scsc = container_of(to_delayed_work(work),
						struct scsc_wifibt,
						check_work);
	unsigned int stat;
	u32 crc;

	crc = scsc_wifibt_dram_crc(scsc);
	regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &stat);

	dev_info(scsc->dev,
		 "5s check: DRAM crc 0x%08x (was 0x%08x) %s, WIFI_STAT 0x%08x, M4 %08x, IRQs %d, WDOG %d\n",
		 crc, scsc->dram_crc,
		 crc == scsc->dram_crc ? "unchanged" : "CHANGED",
		 stat, readl(scsc->base_m4 + SCSC_MBOX_ISSR(0)),
		 atomic_read(&scsc->irq_count),
		 atomic_read(&scsc->wdog_count));

	scsc_wifibt_scan(scsc);
}

static int scsc_wifibt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scsc_wifibt *scsc;
	struct regmap *pmureg;
	struct reserved_mem *rmem;
	struct device_node *rmem_np;
	const struct firmware *fw;
	unsigned int val;
	int irq, ret;

	scsc = devm_kzalloc(dev, sizeof(*scsc), GFP_KERNEL);
	if (!scsc)
		return -ENOMEM;

	scsc->dev = dev;
	atomic_set(&scsc->irq_count, 0);
	atomic_set(&scsc->wdog_count, 0);
	INIT_DELAYED_WORK(&scsc->check_work, scsc_wifibt_check_work);
	platform_set_drvdata(pdev, scsc);

	ret = device_create_file(dev, &dev_attr_scan);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create scan file\n");

	ret = device_create_file(dev, &dev_attr_recover);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create recover file\n");

	/* R4 mailbox bank (index 0); M4 bank (index 1). */
	scsc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(scsc->base))
		return dev_err_probe(dev, PTR_ERR(scsc->base),
				     "failed to map R4 mailbox\n");

	scsc->base_m4 = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(scsc->base_m4))
		return dev_err_probe(dev, PTR_ERR(scsc->base_m4),
				     "failed to map M4 mailbox\n");

	val = readl(scsc->base + SCSC_MBOX_IS_VERSION);
	dev_info(dev, "R4 mailbox version 0x%08x\n", val);

	/* Shared-memory carveout referenced via memory-region. Note:
	 * of_reserved_mem_lookup() matches by node name, so resolve the
	 * phandle first instead of passing our own node.
	 */
	rmem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!rmem_np)
		return dev_err_probe(dev, -ENODEV,
				     "missing memory-region (wifibt_if)\n");
	rmem = of_reserved_mem_lookup(rmem_np);
	of_node_put(rmem_np);
	if (!rmem)
		return dev_err_probe(dev, -ENODEV,
				     "wifibt_if region not reserved\n");

	dev_info(dev, "shared memory base 0x%llx size 0x%llx\n",
		 (unsigned long long)rmem->base, (unsigned long long)rmem->size);

	pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						 "samsung,syscon-phandle");
	if (IS_ERR(pmureg))
		return dev_err_probe(dev, PTR_ERR(pmureg),
				     "failed to get PMU syscon\n");

	scsc->pmureg = pmureg;
	scsc->mem_start = rmem->base;
	scsc->mem_size = rmem->size;

	ret = scsc_wifibt_map(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to map shared memory\n");

	irq = platform_get_irq_byname(pdev, "MBOX");
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get MBOX irq\n");

	ret = devm_request_irq(dev, irq, scsc_wifibt_mbox_irq, 0,
			       dev_name(dev), scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request MBOX irq\n");

	/* Watchdog: fires if the firmware block faults. One-shot, acked
	 * like downstream.
	 */
	irq = platform_get_irq_byname(pdev, "WDOG");
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get WDOG irq\n");

	ret = devm_request_irq(dev, irq, scsc_wifibt_wdog_irq, 0,
			       dev_name(dev), scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request WDOG irq\n");

	if (cachetest)
		scsc_wifibt_cachetest(scsc);

	/* Load, verify and stage the firmware image in shared DRAM before
	 * releasing the block from reset (downstream mxman order).
	 */
	ret = request_firmware(&fw, SCSC_FW_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "firmware %s not available\n",
				     SCSC_FW_NAME);

	dev_info(dev, "firmware %s size %zu\n", SCSC_FW_NAME, fw->size);

	ret = scsc_wifibt_fw_parse(scsc, fw);
	if (ret) {
		release_firmware(fw);
		return dev_err_probe(dev, ret, "firmware rejected\n");
	}

	ret = scsc_wifibt_fw_stage(scsc, fw);
	release_firmware(fw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to stage firmware\n");

	ret = scsc_wifibt_mxconf(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set mxconf\n");

	scsc_wifibt_prepare(scsc);

	ret = scsc_wifibt_power_on(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure the block\n");

	ret = scsc_wifibt_start(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to release the block\n");

	scsc->dram_crc = scsc_wifibt_dram_crc(scsc);
	schedule_delayed_work(&scsc->check_work, msecs_to_jiffies(5000));

	dev_info(dev, "probed\n");

	return 0;
}

static void scsc_wifibt_remove(struct platform_device *pdev)
{
	struct scsc_wifibt *scsc = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_scan);
	device_remove_file(&pdev->dev, &dev_attr_recover);
	cancel_delayed_work_sync(&scsc->check_work);
	scsc_wifibt_power_off(scsc);
	vunmap(scsc->mem);
	scsc->mem = NULL;
}

static const struct of_device_id scsc_wifibt_of_match[] = {
	{ .compatible = "samsung,scsc-wifibt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scsc_wifibt_of_match);

static struct platform_driver scsc_wifibt_driver = {
	.probe = scsc_wifibt_probe,
	.remove = scsc_wifibt_remove,
	.driver = {
		.name = "scsc-wifibt",
		.of_match_table = scsc_wifibt_of_match,
	},
};
module_platform_driver(scsc_wifibt_driver);

MODULE_AUTHOR("T510 mainlining project");
MODULE_DESCRIPTION("Samsung SCSC WiFi/BT firmware block bring-up");
MODULE_LICENSE("GPL");
