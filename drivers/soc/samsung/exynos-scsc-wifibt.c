// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal probe-only driver for the Samsung SLSI SCSC (mx140/Lassen)
 * WiFi/BT mailbox found on Exynos7885/7904 (e.g. Galaxy Tab A 10.1 2019).
 *
 * It maps the R4/M4 mailbox registers, reports the firmware block version,
 * checks the shared-memory carveout and PMU state, and verifies the
 * firmware image is loadable via the firmware loader. It can also power
 * the block on/off through the PMU; booting firmware and the 802.11
 * network interface itself are future work.
 *
 * Register map legislation: downstream Samsung Android kernel for T510
 * (drivers/misc/samsung/scsc/mif_reg_S5E7885.h), used as documentation only.
 */

#include <linux/atomic.h>
#include <linux/arm-smccc.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

/* Mailbox (AP side, R4 bank) register offsets */
#define SCSC_MBOX_INTMSR0	0x018 /* Interrupt mask status, upper half is FROM R4/M4 */
#define SCSC_MBOX_INTCR0	0x00c /* Interrupt clear, write 1 to clear */
#define SCSC_MBOX_IS_VERSION	0x050 /* Firmware block version information */
#define SCSC_MBOX_ISSR_BASE	0x080 /* Shared registers, 4 bytes each */
#define SCSC_MARK_MBOX_REG	0x080 /* M4 MBOX0: proven R4-writable, so it validates probes */
#define SCSC_MBOX_ISSR(i)	(SCSC_MBOX_ISSR_BASE + 4 * (i))

/* Full mailbox control block offsets (beyond INTMSR/INTCR/version). */
#define SCSC_MBOX_INTGR0	0x008
#define SCSC_MBOX_INTMR0	0x010
#define SCSC_MBOX_INTSR0	0x014
#define SCSC_MBOX_INTGR1	0x01c
#define SCSC_MBOX_INTCR1	0x020
#define SCSC_MBOX_INTMR1	0x024
#define SCSC_MBOX_INTSR1	0x028
#define SCSC_MBOX_INTMSR1	0x02c
#define SCSC_MBOX_MIF_INIT	0x04c

/* Boot handshake values (downstream mbox_init, documentation only) */
#define SCSC_CACHETEST_OFF	0x1f0000
#define SCSC_PANIC_OFF		0x160804
#define SCSC_PANIC_LEN		0xd4
#define SCSC_MBOX_MAGIC		0xbcdeedcb
#define SCSC_MBOX_FW_FLAGS	0x0 /* Bit 0 = spin at start of CRT0 */

/* Minimal mxconf (infrastructure config) for the R4. Layout mirrors the
 * downstream heap order with the downstream bit allocation: management,
 * GDB R4, GDB M4, mxlog. The firmware reads buffer locations and
 * signalling bits from here, so every stream gets real buffers.
 */
#define SCSC_MXCONF_MAGIC		0x79828486
#define SCSC_MXCONF_VER_MAJOR		0
#define SCSC_MXCONF_VER_MINOR		1
#define SCSC_MGMT_BUF_LEN		512
#define SCSC_MGMT_PACKET_SIZE		8
#define SCSC_MGMT_NUM_PACKETS		(SCSC_MGMT_BUF_LEN / SCSC_MGMT_PACKET_SIZE)
#define SCSC_GDB_BUF_LEN		2048
#define SCSC_GDB_PACKET_SIZE		4
#define SCSC_GDB_NUM_PACKETS		(SCSC_GDB_BUF_LEN / SCSC_GDB_PACKET_SIZE)
#define SCSC_MXLOG_BUF_LEN		16384
#define SCSC_MXLOG_PACKET_SIZE		4
#define SCSC_MXLOG_NUM_PACKETS		(SCSC_MXLOG_BUF_LEN / SCSC_MXLOG_PACKET_SIZE)
#define SCSC_MXCONF_SIZE		162
#define SCSC_STREAMCONF_SIZE		22

/* TZASC: allow the firmware block DRAM access (downstream SMC cmd) */
#define SCSC_SMC_WLBT_TZASC	0x82000710

static bool signal_r4 = true;
module_param(signal_r4, bool, 0644);
MODULE_PARM_DESC(signal_r4, "Write the R4 boot handshake (TZASC, MBOX regs) before reset release");

/* Skip the TZASC SMC: if our call narrows EL3 defaults to
 * read-only, skipping restores the POR behavior for comparison.
 */
static bool skip_tzasc;
module_param(skip_tzasc, bool, 0644);
MODULE_PARM_DESC(skip_tzasc, "Skip the WLBT TZASC SMC call");

/* Queue a minimal GDB halt-reason query in the GDB R4 from-AP stream
 * instead of sending a bare panic pulse. A live GDB stub answers in
 * the to-AP stream (and need not panic); an absent one panics or
 * stays silent exactly like the bare pulse.
 */
static bool gdb_probe;
module_param(gdb_probe, bool, 0644);
MODULE_PARM_DESC(gdb_probe, "Send a GDB query to the R4 stub instead of a bare panic pulse");

/* Pulse INTGR1 after unmasking/re-arming. Withheld (=0) to test whether
 * the second watchdog comes from the re-enable alone (pending) or from
 * the pulse (firmware response).
 */
static bool poke_intgr = true;
module_param(poke_intgr, bool, 0644);
MODULE_PARM_DESC(poke_intgr, "Pulse INTGR1 bit 0 after unmasking (0 tests re-enable alone)");

static bool null_mxconf;
module_param(null_mxconf, bool, 0644);
MODULE_PARM_DESC(null_mxconf, "Hand the R4 a null mxconf pointer instead of the fabricated one");

/* R4 execution probe: Thumb-2 payload that writes MARKER to DRAM offset
 * MARK_OFF, staged at the probe entry (masked to even) and entered via
 * MBOX_0 (odd = Thumb, same convention as the firmware entry 0x1a9).
 */
#define SCSC_PROBE_MARK_OFF	0x1000
#define SCSC_PROBE_MARKER	0xdeadbeef
static bool r4_probe;
module_param(r4_probe, bool, 0644);
MODULE_PARM_DESC(r4_probe, "Point the R4 at a marker-writing probe payload instead of the firmware");

static uint probe_entry = 0x200001;
module_param(probe_entry, uint, 0644);
MODULE_PARM_DESC(probe_entry, "DRAM offset used as R4 entry (and payload location) when r4_probe is set");

/* Overwrite the staged image at the firmware entry point with the probe
 * payload. The ROM accepts 0x1a9 (it runs, starts the M4 and faults),
 * so if the R4 truly jumps there, the marker must appear.
 */
static bool patch_entry;
module_param(patch_entry, bool, 0644);
MODULE_PARM_DESC(patch_entry, "Replace the firmware image at its entry point with the probe payload");

/* NOP out the table_A wait loop so the firmware runs past a missing
 * publisher. Layout-specific to the 2019 image (verified before
 * patching); CRCs are repaired afterwards like any other patch.
 */
static bool nop_wait;
module_param(nop_wait, bool, 0644);
MODULE_PARM_DESC(nop_wait, "Replace the table wait loop with NOPs");

/* Stage a hand-built MM_START_IND sender at the firmware entry instead
 * of the sweep payload: all-zero management packet to the to-AP buffer,
 * write index 0 to 1, spin. If R4 DRAM writes work, the packet lands
 * visibly and the AP receive path can be built against it.
 */
static bool graft_mm;
module_param(graft_mm, bool, 0644);
MODULE_PARM_DESC(graft_mm, "Stage an MM_START_IND graft at the firmware entry");

/* Override MBOX_0 with an arbitrary value on the intact-image path.
 * Isolates whether the entry VALUE alone (independent of image
 * content) decides the firmware block's response.
 */
static uint mbox0_override;
module_param(mbox0_override, uint, 0644);
MODULE_PARM_DESC(mbox0_override, "Write this instead of the firmware entry point to MBOX_0 (0 keeps fw_entry)");

/* Fill the DRAM gap beyond the image with a pattern. Firmware BSS
 * clearing (or any other R4 write) then shows as a CRC change; an
 * untouched gap stays patterned.
 */
static bool fill_gap;
module_param(fill_gap, bool, 0644);
MODULE_PARM_DESC(fill_gap, "Fill shared DRAM beyond the image with 0xAA before boot");

/* PMU (system-controller syscon) register offsets */
#define SCSC_PMU_WIFI_CTRL_NS		0x140 /* non-secure control */
#define SCSC_PMU_WIFI_PWRON		BIT(1)
#define SCSC_PMU_WIFI_RESET_SET		BIT(2)
#define SCSC_PMU_WIFI_RESET_REQ_CLR	BIT(8)
#define SCSC_PMU_WIFI_CTRL_S		0x144 /* secure control */
#define SCSC_PMU_WIFI_START		BIT(3)
#define SCSC_PMU_WIFI_STAT		0x148
#define SCSC_PMU_WIFI_PWRDN_DONE	BIT(0)
/* 0 = boot the firmware block from external DRAM (not ROM/test). */
#define SCSC_PMU_BOOT_TEST_RST_CFG	0x7330

/* Shared-memory (BAAW) window configuration */
#define SCSC_PMU_MEM_CONFIG0		0x7300 /* WiFi window size (4K units) */
#define SCSC_PMU_MEM_CONFIG1		0x7304 /* WiFi window base (4K units) */
#define SCSC_PMU_ABOX_CONFIG0		0x7308 /* BT-ABOX window size */
#define SCSC_PMU_ABOX_CONFIG1		0x730c /* BT-ABOX window base */

/* Downstream programs the adjacent BT-ABOX window before releasing the
 * block (abox_rmem in the stock exynos7885-rmem.dtsi). The M4 lives on
 * that side of the pair, so the firmware may refuse to go on without
 * it. Kept as a parameter because the DT region is not wired to us.
 */
#define SCSC_ABOX_BASE			0xe9400000ul
#define SCSC_ABOX_SIZE			0x400000ul
static bool abox_win;
module_param(abox_win, bool, 0644);
MODULE_PARM_DESC(abox_win, "Program the stock BT-ABOX window before release");

/* Low-power sequencing (power-off path) */
#define SCSC_PMU_RESET_AHEAD		0x1360
#define SCSC_PMU_CLEANY_BUS		0x1364
#define SCSC_PMU_LOGIC_RESET		0x1368
#define SCSC_PMU_TCXO_GATE		0x136c
#define SCSC_PMU_DISABLE_ISO		0x1370
#define SCSC_PMU_RESET_ISO		0x1374
#define SCSC_PMU_CENTRAL_SEQ_CFG	0x0380
#define SCSC_PMU_CENTRAL_SEQ_STAT	0x0384
#define SCSC_PMU_STATES			0xff0000
#define SCSC_PMU_SYS_PWR_CFG		BIT(0)
#define SCSC_PMU_SYS_PWR_CFG_2		(BIT(0) | BIT(1))
#define SCSC_PMU_SYS_PWR_CFG_16		BIT(16)
#define SCSC_PMU_SM_DOWN		0x80

/* BAAW access windows (never written by anyone so far; dumped to see
 * whether the firmware block is actually allowed past its DRAM window).
 */
#define SCSC_PMU_MIF_WIN0	0x7318
#define SCSC_PMU_MIF_WIN1	0x731c
#define SCSC_PMU_PERI_WIN0	0x7320
#define SCSC_PMU_PERI_WIN1	0x7324
#define SCSC_PMU_PERI_WIN2	0x7328
#define SCSC_PMU_PERI_WIN3	0x732c

/* Shared-register option register. Downstream flips a bit here around
 * the whole power sequence (mcu_ipc shared_reg_enable/disable) so the
 * modem co-processor does not own the shared registers while the
 * firmware block comes up.
 */
#define SCSC_PMU_SHARED_OPTION	0x3648
static unsigned int shared_opt;
module_param(shared_opt, uint, 0644);
MODULE_PARM_DESC(shared_opt, "PMU shared-option bits to set before the power sequence");

/* Open every BAAW access window. The MIF windows read 0x55555555
 * (per-region 01); if 01 means read-only, writes need all-ones.
 * Experimental: if it breaks working reads, reboot restores POR.
 */
static bool open_wins;
module_param(open_wins, bool, 0644);
MODULE_PARM_DESC(open_wins, "Write all-ones to the BAAW access windows before release");

/* The reset value of the MIF access control is 0x55555555, which reads
 * as "fetch works, stores do not" from the firmware's point of view.
 * Sweep the value to find the encoding that grants the block write
 * access to the shared window.
 */
static unsigned int mif_win;
module_param(mif_win, uint, 0644);
MODULE_PARM_DESC(mif_win, "Value for the MIF access control registers (0 = leave reset value)");

/* Low-power controls touched by the power-off path. Snapshotted at
 * probe (cold defaults) and restored on power-on: releasing without
 * them leaves a warm block wedged with START held.
 */
static const unsigned int scsc_lp_regs[] = {
	SCSC_PMU_RESET_AHEAD, SCSC_PMU_CLEANY_BUS, SCSC_PMU_LOGIC_RESET,
	SCSC_PMU_TCXO_GATE, SCSC_PMU_DISABLE_ISO, SCSC_PMU_RESET_ISO,
	SCSC_PMU_CENTRAL_SEQ_CFG,
};

/*
 * Firmware image name as shipped by the firmware-samsung-gta3xlwifi aport.
 * The final WLAN driver will use the linux-firmware style path instead.
 */
#define SCSC_FW_NAME	"postmarketos/mx140/mx140.bin"

/* Maxwell firmware header (format v0.2/v1.0, magic "smxf"). Offsets from
 * the downstream header parser, used as documentation only.
 */
#define SCSC_FW_MAGIC_OFF		8
#define SCSC_FW_MAGIC			"smxf"
#define SCSC_FW_VER_MINOR_OFF		12
#define SCSC_FW_VER_MAJOR_OFF		14
#define SCSC_FW_LEN_OFF			16
#define SCSC_FW_API_MINOR_OFF		20
#define SCSC_FW_API_MAJOR_OFF		22
#define SCSC_FW_CRC_OFF			24
#define SCSC_FW_CONST_LEN_OFF		28
#define SCSC_FW_CONST_CRC_OFF		32
#define SCSC_FW_ENTRY_OFF		40
#define SCSC_FW_BUILD_ID_OFF		48
#define SCSC_FW_BUILD_ID_SZ		128
#define SCSC_FW_RUNTIME_LEN_OFF		36
#define SCSC_FW_CONST_LEN_OFF		28

struct scsc_wifibt {
	struct device	*dev;
	void __iomem	*base;
	void __iomem	*base_m4;
	struct regmap	*pmureg;
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
	u32		mxlog_off;
	u32		gdb_fa_buf;
	u32		gdb_fa_widx;
	u32		gdb_ta_buf;
	u32		mgmt_ta_buf;
	u32		mgmt_ta_widx;
	u32		mgmt_fa_buf;
	u32		mgmt_fa_widx;
	u32		dram_crc;
	struct delayed_work check_work;
	atomic_t	irq_count;
	atomic_t	wdog_count;
};

/* Shared DRAM is accessed by a non-coherent firmware block: map it
 * write-combined (uncached) like downstream's vmap WRITE_COMBINE, so
 * staged data is visible to the R4 and its writes are visible to us.
 * A cached mapping starves the R4 (stale DRAM) and blinds our reads.
 * memremap WC refuses RAM that already has a cached linear alias, so
 * build the mapping the downstream way instead.
 */
/*
 * The window is RAM the R4 writes behind our back, so it has to be
 * mapped so that every read reaches memory.  This is Normal
 * non-cacheable, which is what pgprot_writecombine() selects on arm64
 * and what Samsung's own driver uses for this window.  Device memory
 * would be safe from speculation but rejects the unaligned accesses
 * put_unaligned_le32() and friends perform, so it is not an option
 * here; scsc_wifibt_cachetest() below measures whether reads through
 * this mapping come back stale.
 */
static void *scsc_wifibt_map(struct scsc_wifibt *scsc)
{
	struct page **pages;
	void *vmem;
	unsigned int i, npages = PAGE_ALIGN(scsc->mem_size) >> PAGE_SHIFT;

	pages = kmalloc_array(npages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < npages; i++)
		pages[i] = phys_to_page(scsc->mem_start + i * PAGE_SIZE);

	vmem = vmap(pages, npages, VM_MAP,
		    pgprot_writecombine(PAGE_KERNEL));
	kfree(pages);

	return vmem;
}

static void scsc_wifibt_unmap(const void *vmem)
{
	vunmap(vmem);
}

static bool cachetest;
module_param(cachetest, bool, 0644);
MODULE_PARM_DESC(cachetest,
		 "Self-test whether reads through the window mapping are coherent");

/*
 * Bulk transfers into and out of the window, through the volatile
 * accessors so the compiler cannot merge them into wider accesses.
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

static void scsc_win_set(void *dst, u32 val, size_t n)
{
	u8 *d = dst;
	size_t i;

	for (i = 0; i + 4 <= n; i += 4)
		writel(val, d + i);
	for (; i < n; i++)
		writeb(val, d + i);
}

/*
 * Self-test for the window mapping: write a pattern, read it back, write
 * a different value and read again, then read it through a second
 * mapping.  If any read comes back stale we cannot trust anything we
 * read out of this window, and the R4's own writes are what we are here
 * to look at.
 */
static void scsc_wifibt_cachetest(struct scsc_wifibt *scsc)
{
	void *dram;
	u32 first, again, fresh;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return;

	scsc_win_set(dram + SCSC_CACHETEST_OFF, 0x11223344, 0x1000);
	first = readl(dram + SCSC_CACHETEST_OFF);
	writel(0xa5a5a5a5, dram + SCSC_CACHETEST_OFF);
	again = readl(dram + SCSC_CACHETEST_OFF);
	scsc_wifibt_unmap(dram);

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return;
	fresh = readl(dram + SCSC_CACHETEST_OFF);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev,
		 "cachetest: pattern %08x, after write %08x, remap %08x -> %s\n",
		 first, again, fresh,
		 (first == 0x11223344 && again == 0xa5a5a5a5 &&
		  fresh == 0xa5a5a5a5) ? "coherent" : "STALE");
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

static int scsc_wifibt_power_on(struct scsc_wifibt *scsc)
{
	unsigned int val, tmp, i;
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

	/* NOTE: the low-power controls are NOT restored here. They read
	 * all-ones while the block is off, so a snapshot would be garbage;
	 * the power-off path (downstream-faithful) is the only writer.
	 */
	for (i = 0; i < ARRAY_SIZE(scsc_lp_regs); i++) {
		ret = regmap_read(scsc->pmureg, scsc_lp_regs[i], &val);
		if (ret)
			return ret;
		dev_info(scsc->dev, "LP reg 0x%03x post-power 0x%08x\n",
			 scsc_lp_regs[i], val);
	}

	/* Expose the shared-memory carveout to the firmware block (4K units).
	 * The BT-ABOX window (CONFIG2/3) stays cleared; BT comes later.
	 */
	ret = regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG1,
			   (scsc->mem_start & 0xfffffc000ULL) >> 12);
	if (ret)
		return ret;

	ret = regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG0,
			   scsc->mem_size >> 12);
	if (ret)
		return ret;

	if (abox_win) {
		ret = regmap_write(scsc->pmureg, SCSC_PMU_ABOX_CONFIG1,
				   (SCSC_ABOX_BASE & 0xfffffc000UL) >> 12);
		if (ret)
			return ret;

		ret = regmap_write(scsc->pmureg, SCSC_PMU_ABOX_CONFIG0,
				   SCSC_ABOX_SIZE >> 12);
		if (ret)
			return ret;

		regmap_read(scsc->pmureg, SCSC_PMU_ABOX_CONFIG0, &val);
		regmap_read(scsc->pmureg, SCSC_PMU_ABOX_CONFIG1, &tmp);
		dev_info(scsc->dev, "ABOX window %08x %08x\n", val, tmp);
	}

	/* Power on, release reset, start: mirrors downstream 8.6.6 sequence. */
	if (shared_opt) {
		ret = regmap_update_bits(scsc->pmureg,
					 SCSC_PMU_SHARED_OPTION,
					 shared_opt, shared_opt);
		if (ret)
			return ret;
		regmap_read(scsc->pmureg, SCSC_PMU_SHARED_OPTION, &val);
		dev_info(scsc->dev, "shared option 0x%08x (want bits 0x%x)\n",
			 val, shared_opt);
	}

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

	/* Fast sampling for 50 ms after release: whoever acts in the
	 * first milliseconds (M4 status, mailbox signals) shows here.
	 * Logs only changes.
	 */
	usleep_range(100, 300);
	for (i = 0; i < 200; i++) {
		static u32 lm0, lm1, lr0, lr1, lp, lpa;
		u32 m0 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(0));
		u32 m1 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(1));
		u32 r0 = readl(scsc->base + SCSC_MBOX_ISSR(0));
		u32 r1 = readl(scsc->base + SCSC_MBOX_ISSR(1));
		u32 p = readl(scsc->base_m4 + SCSC_MARK_MBOX_REG);
		u32 pa = readl(scsc->base_m4 + SCSC_MARK_MBOX_REG + 0x10);

		if (i == 0 || m0 != lm0 || m1 != lm1 || r0 != lr0 ||
		    r1 != lr1 || p != lp || pa != lpa)
			dev_info(scsc->dev,
				 "t+%ums M4 %08x %08x R4 %08x %08x probe %08x/%08x\n",
				 2 + i * 2, m0, m1, r0, r1, p, pa);

		lpa = pa;
		lp = p;
		lm0 = m0;
		lm1 = m1;
		lr0 = r0;
		lr1 = r1;
		usleep_range(100, 300);
	}

	/* Immediate readback: the START bit auto-clears on UP, which takes
	 * longer than this, so 1 here means the write landed.
	 */
	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_CTRL_S, &val);
	if (ret)
		return ret;

	dev_info(scsc->dev, "WIFI_CTRL_S immediate readback 0x%08x\n", val);

	usleep_range(10000, 20000);

	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &val);
	if (ret)
		return ret;

	dev_info(scsc->dev, "powered on, WIFI_STAT 0x%08x%s\n", val,
		 val & SCSC_PMU_WIFI_PWRDN_DONE ? " (power-down done)" : "");

	ret = regmap_read(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_STAT, &val);
	if (ret)
		return ret;

	dev_info(scsc->dev, "central sequencer state 0x%02x\n",
		 (val & SCSC_PMU_STATES) >> 16);

	/* Read back what we programmed: a blocked write here would explain
	 * a silent R4.
	 */
	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS, &val);
	if (ret)
		return ret;

	dev_info(scsc->dev, "WIFI_CTRL_NS readback 0x%08x (PWRON %s, RESET %s)\n",
		 val, val & SCSC_PMU_WIFI_PWRON ? "set" : "clear",
		 val & SCSC_PMU_WIFI_RESET_SET ? "held" : "released");

	ret = regmap_read(scsc->pmureg, SCSC_PMU_WIFI_CTRL_S, &val);
	if (ret)
		return ret;

	dev_info(scsc->dev, "WIFI_CTRL_S readback 0x%08x (START %s)\n",
		 val, val & SCSC_PMU_WIFI_START ? "set" : "clear");

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

	scsc->fw_entry = entry;
	scsc->fw_runtime = runtime_len;
	scsc->fw_hdr_len = hdr_len;
	scsc->fw_const_len = const_len;
	scsc->fw_len = fw->size;

	/* Integrity checks over the image, same layout as downstream fwimage. */
	fw_crc = get_unaligned_le32(fw->data + SCSC_FW_CRC_OFF);
	const_crc = get_unaligned_le32(fw->data + SCSC_FW_CONST_CRC_OFF);
	hdr_crc = get_unaligned_le32(fw->data + hdr_len - sizeof(u32));

	if (hdr_len > fw->size || const_len > fw->size) {
		dev_err(scsc->dev, "firmware lengths out of range\n");
		return -EINVAL;
	}

	if (ether_crc(hdr_len - sizeof(u32), fw->data) != hdr_crc ||
	    ether_crc(const_len - hdr_len, fw->data + hdr_len) != const_crc ||
	    ether_crc(fw->size - hdr_len, fw->data + hdr_len) != fw_crc) {
		dev_err(scsc->dev, "firmware CRC mismatch\n");
		return -EINVAL;
	}

	dev_info(scsc->dev, "firmware CRCs OK\n");

	return 0;
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

static void scsc_wifibt_stream_mem(u8 *dram, u32 buf, u32 len, bool fill_ff)
{
	scsc_win_set(dram + buf, fill_ff ? 0xff : 0, len);
	put_unaligned_le32(0, dram + buf + len);
	put_unaligned_le32(0, dram + buf + len + 4);
}

/* Lay out a full mxconf plus all transport stream buffers in the heap
 * area (right after the firmware runtime), mirroring the downstream heap
 * order and bit allocation. Returns the mxconf offset (R4-relative ref
 * for MBOX_1).
 */
static int scsc_wifibt_mxconf(struct scsc_wifibt *scsc)
{
	u8 *dram, *mxconf;
	u32 heap, to_ap, from_ap;
	u32 gdb_r4_in, gdb_r4_out, gdb_m4_in, gdb_m4_out, mxlog, end;
	u32 mx_off = ALIGN(scsc->fw_runtime, 4);

	heap = mx_off + SCSC_MXCONF_SIZE;
	to_ap = ALIGN(heap, 4);
	from_ap = to_ap + SCSC_MGMT_BUF_LEN + 8;
	gdb_r4_in = from_ap + SCSC_MGMT_BUF_LEN + 8;
	gdb_r4_out = gdb_r4_in + SCSC_GDB_BUF_LEN + 8;
	gdb_m4_in = gdb_r4_out + SCSC_GDB_BUF_LEN + 8;
	gdb_m4_out = gdb_m4_in + SCSC_GDB_BUF_LEN + 8;
	mxlog = gdb_m4_out + SCSC_GDB_BUF_LEN + 8;
	end = mxlog + SCSC_MXLOG_BUF_LEN + 8;

	if (end > scsc->mem_size) {
		dev_err(scsc->dev, "mxconf layout 0x%x exceeds window 0x%zx\n",
			end, scsc->mem_size);
		return -EINVAL;
	}

	if (cachetest)
		scsc_wifibt_cachetest(scsc);

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	mxconf = dram + mx_off;
	memset(mxconf, 0, SCSC_MXCONF_SIZE);
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
	/* GDB R4 to-AP (IN): read=fromhost 3, write=tohost 2. */
	scsc_wifibt_stream_conf(mxconf + 8 + 2 * SCSC_STREAMCONF_SIZE,
				gdb_r4_in,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_r4_in + SCSC_GDB_BUF_LEN,
				gdb_r4_in + SCSC_GDB_BUF_LEN + 4, 3, 2);
	/* GDB R4 from-AP (OUT): read=tohost 3, write=fromhost 0. */
	scsc_wifibt_stream_conf(mxconf + 8 + 3 * SCSC_STREAMCONF_SIZE,
				gdb_r4_out,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_r4_out + SCSC_GDB_BUF_LEN,
				gdb_r4_out + SCSC_GDB_BUF_LEN + 4, 3, 0);
	/* GDB M4 to-AP (IN): read=fromhost(M4) 1, write=tohost 4. */
	scsc_wifibt_stream_conf(mxconf + 8 + 4 * SCSC_STREAMCONF_SIZE,
				gdb_m4_in,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_m4_in + SCSC_GDB_BUF_LEN,
				gdb_m4_in + SCSC_GDB_BUF_LEN + 4, 1, 4);
	/* GDB M4 from-AP (OUT): read=tohost 5, write=fromhost(M4) 0. */
	scsc_wifibt_stream_conf(mxconf + 8 + 5 * SCSC_STREAMCONF_SIZE,
				gdb_m4_out,
				SCSC_GDB_NUM_PACKETS, SCSC_GDB_PACKET_SIZE,
				gdb_m4_out + SCSC_GDB_BUF_LEN,
				gdb_m4_out + SCSC_GDB_BUF_LEN + 4, 5, 0);
	/* Mxlog (IN): read=fromhost 4, write=tohost 6. */
	scsc_wifibt_stream_conf(mxconf + 8 + 6 * SCSC_STREAMCONF_SIZE,
				mxlog,
				SCSC_MXLOG_NUM_PACKETS, SCSC_MXLOG_PACKET_SIZE,
				mxlog + SCSC_MXLOG_BUF_LEN,
				mxlog + SCSC_MXLOG_BUF_LEN + 4, 4, 6);

	/* Buffers and indices. IN-direction buffers start all-ones. */
	scsc_wifibt_stream_mem(dram, to_ap, SCSC_MGMT_BUF_LEN, true);
	scsc_wifibt_stream_mem(dram, from_ap, SCSC_MGMT_BUF_LEN, false);
	scsc_wifibt_stream_mem(dram, gdb_r4_in, SCSC_GDB_BUF_LEN, true);
	scsc_wifibt_stream_mem(dram, gdb_r4_out, SCSC_GDB_BUF_LEN, false);
	scsc_wifibt_stream_mem(dram, gdb_m4_in, SCSC_GDB_BUF_LEN, true);
	scsc_wifibt_stream_mem(dram, gdb_m4_out, SCSC_GDB_BUF_LEN, false);
	scsc_wifibt_stream_mem(dram, mxlog, SCSC_MXLOG_BUF_LEN, true);

	scsc_wifibt_unmap(dram);

	scsc->mxconf_off = mx_off;
	scsc->mxlog_off = mxlog;
	scsc->gdb_fa_buf = gdb_r4_out;
	scsc->gdb_fa_widx = gdb_r4_out + SCSC_GDB_BUF_LEN;
	scsc->gdb_ta_buf = gdb_r4_in;
	scsc->mgmt_ta_buf = to_ap;
	scsc->mgmt_ta_widx = to_ap + SCSC_MGMT_BUF_LEN;
	scsc->mgmt_fa_buf = from_ap;
	scsc->mgmt_fa_widx = from_ap + SCSC_MGMT_BUF_LEN;
	dev_info(scsc->dev, "mxconf at DRAM offset 0x%x\n", mx_off);

	return 0;
}

/* Sweep payload: for each of 8 R4-view addresses, load it and MARKER
 * from the literal pool and store. Generated and decode-checked
 * offline; only LDR.W-literal/STR/B patterns are used. A marker scan
 * of the window afterwards reveals where (if anywhere) stores land.
 */
static const u8 scsc_probe_payload[] = {
	0xdf, 0xf8, 0x4e, 0x00, 0xdf, 0xf8, 0x4e, 0x10,
	0x01, 0x60, 0xdf, 0xf8, 0x4c, 0x00, 0xdf, 0xf8,
	0x4c, 0x10, 0x01, 0x60, 0xdf, 0xf8, 0x4a, 0x00,
	0xdf, 0xf8, 0x4a, 0x10, 0x01, 0x60, 0xdf, 0xf8,
	0x48, 0x00, 0xdf, 0xf8, 0x48, 0x10, 0x01, 0x60,
	0xdf, 0xf8, 0x46, 0x00, 0xdf, 0xf8, 0x46, 0x10,
	0x01, 0x60, 0xdf, 0xf8, 0x44, 0x00, 0xdf, 0xf8,
	0x44, 0x10, 0x01, 0x60, 0xdf, 0xf8, 0x42, 0x00,
	0xdf, 0xf8, 0x42, 0x10, 0x01, 0x60, 0xdf, 0xf8,
	0x40, 0x00, 0xdf, 0xf8, 0x40, 0x10, 0x01, 0x60,
	0xfe, 0xe7, 0x00, 0x10, 0x00, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x01, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x02, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x04, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x08, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x10, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x20, 0x80, 0xef, 0xbe,
	0xad, 0xde, 0x00, 0x00, 0x30, 0x80, 0xef, 0xbe,
	0xad, 0xde,
};

static int scsc_wifibt_r4_probe(struct scsc_wifibt *scsc)
{
	u32 off = probe_entry & ~1u;
	void *dram;

	if (!(probe_entry & 1u) || off + sizeof(scsc_probe_payload) > scsc->mem_size) {
		dev_err(scsc->dev, "probe entry 0x%x out of range\n",
			probe_entry);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	scsc_win_copy(dram + off, scsc_probe_payload,
		      sizeof(scsc_probe_payload));

	if (scsc_win_cmp(dram + off, scsc_probe_payload,
		   sizeof(scsc_probe_payload))) {
		dev_err(scsc->dev, "probe payload DRAM readback mismatch\n");
		scsc_wifibt_unmap(dram);
		return -EIO;
	}

	scsc_wifibt_unmap(dram);

	scsc->sig_entry = probe_entry;
	scsc->sig_mbox1 = 0;
	dev_info(scsc->dev, "R4 probe payload staged at 0x%x, entry 0x%x\n",
		 off, scsc->sig_entry);

	return 0;
}

/* MM_START_IND graft (28 bytes, Thumb-2, R4 DRAM view base 0x80000000):
 *   ldr r0, [pc, #16]  ; to-ap buffer address
 *   movs r1, #0
 *   str r1, [r0]
 *   strb r1, [r0, #4]
 *   ldr r0, [pc, #10]  ; write-index address
 *   movs r1, #1
 *   str r1, [r0]
 *   b .
 *   .word to_ap, widx
 * Sends an all-zero management packet (ma_msg MM_START_IND) the same
 * way the firmware would, to test the DRAM write path plus the AP
 * receive side together.
 */
#define SCSC_GRAFT_LEN		28

static void scsc_wifibt_build_graft(struct scsc_wifibt *scsc, u8 *g)
{
	u32 to_ap = 0x80000000u + scsc->mgmt_ta_buf;
	u32 widx = 0x80000000u + scsc->mgmt_ta_widx;

	/* LDR.W literal is DF F8 imm-lo Rt:imm-hi in memory order. */
	g[0] = 0xdf; g[1] = 0xf8; g[2] = 0x10; g[3] = 0x00;
	g[4] = 0x00; g[5] = 0x21;
	g[6] = 0x01; g[7] = 0x60;
	g[8] = 0x01; g[9] = 0x71;
	g[10] = 0xdf; g[11] = 0xf8; g[12] = 0x0a; g[13] = 0x10;
	g[14] = 0x01; g[15] = 0x21;
	g[16] = 0x01; g[17] = 0x60;
	g[18] = 0xfe; g[19] = 0xe7;
	put_unaligned_le32(to_ap, g + 20);
	put_unaligned_le32(widx, g + 24);
}

static int scsc_wifibt_graft_mm(struct scsc_wifibt *scsc)
{
	u32 off = scsc->fw_entry & ~1u;
	u8 graft[SCSC_GRAFT_LEN];
	void *dram;

	if (!(scsc->fw_entry & 1u) ||
	    off + sizeof(graft) > scsc->mem_size) {
		dev_err(scsc->dev, "firmware entry 0x%x not graftable\n",
			scsc->fw_entry);
		return -EINVAL;
	}

	scsc_wifibt_build_graft(scsc, graft);

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	scsc_win_copy(dram + off, graft, sizeof(graft));
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "grafted MM_START_IND sender at 0x%x\n", off);

	return 0;
}

static int scsc_wifibt_patch_entry(struct scsc_wifibt *scsc)
{
	u32 off = scsc->fw_entry & ~1u;
	void *dram;

	if (!(scsc->fw_entry & 1u) ||
	    off + sizeof(scsc_probe_payload) > scsc->mem_size) {
		dev_err(scsc->dev, "firmware entry 0x%x not patchable\n",
			scsc->fw_entry);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	scsc_win_copy(dram + off, scsc_probe_payload,
		      sizeof(scsc_probe_payload));
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "patched probe payload over image at 0x%x\n",
		 off);

	return 0;
}

/* Expected bytes of the table_A wait block (file 0x1e6: dispatch call,
 * compare, wait loop). The 2019 and 2021 images differ only in branch
 * targets. Replaced with NOPs to run past a missing publisher (and any
 * shared-SRAM read on the way there).
 */
static const u8 scsc_wait_loop_2019[] = {
	0x4f, 0xf0, 0x02, 0x00, 0x53, 0xf0, 0x4f, 0xfe,
	0xdf, 0xf8, 0xcc, 0x14, 0x88, 0x42, 0x07, 0xd1,
	0x4f, 0xf0, 0x03, 0x00, 0x53, 0xf0, 0x47, 0xfe,
	0xc0, 0xf3, 0x00, 0x00, 0x00, 0xb1, 0xf7, 0xe7,
};
static const u8 scsc_wait_loop_2021[] = {
	0x4f, 0xf0, 0x02, 0x00, 0xd7, 0xf0, 0xc5, 0xf9,
	0xdf, 0xf8, 0xcc, 0x14, 0x88, 0x42, 0x07, 0xd1,
	0x4f, 0xf0, 0x03, 0x00, 0xd7, 0xf0, 0xbd, 0xf9,
	0xc0, 0xf3, 0x00, 0x00, 0x00, 0xb1, 0xf7, 0xe7,
};
#define SCSC_WAIT_OFF		0x1e6
#define SCSC_WAIT_LEN		32

/* MPU setup block (file 0x206: log call, feature clear, region clears).
 * Skipping leaves whatever the ROM configured (possibly open).
 */
static const u8 scsc_mpu_block_2019[] = {
	0x4f, 0xf0, 0x04, 0x00, 0xdf, 0xf8, 0xb4, 0x14,
	0x51, 0xf0, 0x65, 0xfd, 0x1f, 0xee, 0x10, 0x0f,
	0x20, 0xf0, 0x03, 0x00, 0x0f, 0xee, 0x10, 0x0f,
	0x4f, 0xf0, 0x00, 0x00, 0x06, 0xee, 0x12, 0x0f,
	0x4f, 0xf0, 0x00, 0x00, 0xbf, 0xf3, 0x4f, 0x8f,
	0x06, 0xee, 0x51, 0x0f, 0xbf, 0xf3, 0x6f, 0x8f,
};
static const u8 scsc_mpu_block_2021[] = {
	0x4f, 0xf0, 0x04, 0x00, 0xdf, 0xf8, 0xb4, 0x14,
	0xd5, 0xf0, 0xdb, 0xf8, 0x1f, 0xee, 0x10, 0x0f,
	0x20, 0xf0, 0x03, 0x00, 0x0f, 0xee, 0x10, 0x0f,
	0x4f, 0xf0, 0x00, 0x00, 0x06, 0xee, 0x12, 0x0f,
	0x4f, 0xf0, 0x00, 0x00, 0xbf, 0xf3, 0x4f, 0x8f,
	0x06, 0xee, 0x51, 0x0f, 0xbf, 0xf3, 0x6f, 0x8f,
};
#define SCSC_MPU_OFF		0x206
#define SCSC_MPU_LEN		48

static bool skip_mpu;
module_param(skip_mpu, bool, 0644);
MODULE_PARM_DESC(skip_mpu, "Replace the MPU setup block with NOPs");

/* The region-configuration block that follows the M4 handshake (the
 * R4 describing 0x80000000/8 MB as its DRAM region). A store injected
 * just before it lands, so the region descriptors are what take the
 * shared window away afterwards.
 */
#define SCSC_REGION_OFF		0x330
#define SCSC_REGION_LEN		0x4e
static bool skip_regions;
module_param(skip_regions, bool, 0644);
MODULE_PARM_DESC(skip_regions, "NOP the R4 DRAM/MPU region descriptors");

/* The last thing the R4 gets through is the vector-table region
 * (0xffff0000, where the firmware's own map puts the VIC), configured
 * just before its first stack push.
 */
#define SCSC_VECREGION_OFF	0x64c
#define SCSC_VECREGION_LEN	0x24

/* Attribute literal for that region (base 0xffff0000 lives at 0x78c). */
#define SCSC_VECREGION_RASR	0x794
static unsigned int vecregion_rasr;
module_param(vecregion_rasr, uint, 0644);
MODULE_PARM_DESC(vecregion_rasr, "Override the vector-region attribute literal");

/* Generic NOP window, for bisecting a suspect instruction sequence. */
static unsigned int nop_at;
module_param(nop_at, uint, 0644);
MODULE_PARM_DESC(nop_at, "Firmware offset to NOP (0 = off)");
static unsigned int nop_len = 4;
module_param(nop_len, uint, 0644);
MODULE_PARM_DESC(nop_len, "Length of the NOP window in bytes");
static bool skip_vecregion;
module_param(skip_vecregion, bool, 0644);
MODULE_PARM_DESC(skip_vecregion, "NOP the R4 vector-table MPU region setup");

/* Halt the R4 at its first instruction (branch to self at the entry).
 * If M4 and the watchdog still appear, they come from ROM or power
 * logic without any image execution; if they vanish, image execution
 * (at least to M4 start) is required for them.
 */
static bool halt_entry;
module_param(halt_entry, bool, 0644);
MODULE_PARM_DESC(halt_entry, "Replace the firmware entry with an infinite loop");

/* Halt the R4 at a chosen firmware offset and leave a sign that it got
 * there: store 0xa5 to eight words spread over the upper window, then
 * spin. Words that stayed preset mean the R4 never reached that offset;
 * words that changed mean it ran at least up to it, and the pattern says
 * which parts of the window it can write at all. Sweeping the halt
 * offset bisects where execution stops.
 */
static unsigned int mark_at;
module_param(mark_at, uint, 0644);
MODULE_PARM_DESC(mark_at, "Halt the R4 at this firmware offset and mark DRAM (0 = off)");

/* Non-halting markers: stamp DRAM and let the firmware carry on. Only
 * r2 and r3 are clobbered, so probes along the boot say how far it gets
 * and whether the stamps still land at each point.
 */
struct scsc_mark_site {
	u32	off;		/* firmware offset of the probe */
	u32	value;		/* DRAM offset it stamps */
};

static bool mark_run;
module_param(mark_run, bool, 0644);
MODULE_PARM_DESC(mark_run, "Stamp three points of the boot in shared DRAM and continue");

/* Counting variant: bump a DRAM counter instead of stamping, so a
 * restart loop (the R4 kept coming back to the same point) shows up as
 * a count above one.
 */
static bool mark_count;
module_param(mark_count, bool, 0644);
MODULE_PARM_DESC(mark_count, "Count R4 passes over 0x330 in shared DRAM");
#define SCSC_MARK_COUNT_OFF	0x330
#define SCSC_MARK_COUNT_SLOT	0x200000

static const struct scsc_mark_site scsc_mark_run_sites[] = {
	{ 0x330, 0x200000 },
	{ 0x382, 0x280000 },
	{ 0x400, 0x300000 },
};

/* The entry code zeroes the stack pointer five times before it sets one
 * up, so any call that pushes during early boot lands at 0xfffffffc.
 * Point it at the top of the 32K ATCM instead (the firmware's own map
 * calls 0x0000-0x7fff ATCM), which is where the rest of the boot keeps
 * its stack.
 */
static bool stack_fix;
module_param(stack_fix, bool, 0644);
MODULE_PARM_DESC(stack_fix, "Give the early boot a stack pointer at the ATCM top");

/* TZASC grant variants. Downstream calls the SMC with (0, base, size) and
 * treats a non-zero result as failure; ours answers 0x3, which would
 * leave the firmware block unable to write the shared window. Try the
 * nearby argument shapes and keep whichever the firmware answers.
 */
static unsigned int tzasc_args;
module_param(tzasc_args, uint, 0644);
MODULE_PARM_DESC(tzasc_args, "TZASC SMC argument variant (0 = downstream shape)");

/* Raw SMC id, so we can tell "the secure firmware rejected the grant"
 * apart from "this EL3 does not implement the call at all" by calling
 * an id that cannot exist.
 */
static unsigned int tzasc_cmd = SCSC_SMC_WLBT_TZASC;
module_param(tzasc_cmd, uint, 0644);
MODULE_PARM_DESC(tzasc_cmd, "Raw SMC id to call for the TZASC grant");

/* Mailbox progress probes. The R4 demonstrably writes its mailbox (the
 * M4 only boots because of it) while no shared-DRAM write has ever been
 * seen, so stamp three M4-side mailbox words from three points of the
 * early boot and read them back: each stamp proves the R4 ran that far.
 */
static bool mark_mbox;
module_param(mark_mbox, bool, 0644);
static int timeline;
module_param(timeline, int, 0644);
static int fine;
static int fine_us = 500;
module_param(fine, int, 0644);
module_param(fine_us, int, 0644);
MODULE_PARM_DESC(fine,
		 "Sample N cheap registers every fine_us after releasing the R4");
MODULE_PARM_DESC(fine_us, "Interval in us between the fine samples");
MODULE_PARM_DESC(timeline,
		 "Sample the WLBT state every N ms during the boot check (0 = off)");
MODULE_PARM_DESC(mark_mbox, "Stamp the M4 mailbox from three early-boot points");

#define SCSC_MARK_MBOX_SITES	3
/* Sequential stamps into one M4 mailbox word: the last value to land
 * says how far the R4 got. Each probe keeps r0 and r1, so the code
 * around it still runs (with stack_fix enabled).
 */
static const struct scsc_mark_site scsc_mark_mbox_sites[] = {
	{ 0x1a8, 0xa1a10001 },
	{ 0x212, 0xa1a10002 },
	{ 0x236, 0xa1a10003 },
};

#define SCSC_MARK_RUN_LEN	16
#define SCSC_MARK_RUN_SLOT	0x200000
static const u8 scsc_mark_run_stub[] = {
	0x02, 0x4a,			/* ldr r2, [pc, #8] */
	0x4a, 0xf2, 0xa5, 0x53,		/* movw r3, #0xa5a5 */
	0xca, 0xf2, 0xa5, 0x53,		/* movt r3, #0xa5a5 */
	0x13, 0x60,			/* str r3, [r2] */
};

/* Stub: mov.w r0, #0xa5; per DRAM slot: ldr r1, [pc, #imm]; str r0, [r1];
 * mov.w r0, #0x5a; ldr r1, [pc, #imm]; str r0, [r1]; b .; then one
 * literal address per store. A mailbox hit proves the R4 runs and can
 * write peripherals even if the DRAM stores fault.
 */
#define SCSC_MARK_VALUE		0xdeadbeef
#define SCSC_MARK_STAMP		0xa5
#define SCSC_MARK_MBOX_STAMP	0x5a
#define SCSC_MARK_MBOX		0xa20c0088u
#define SCSC_MARK_STAMP16	0x0000a5a5u
/* All above the image (runtime is 0x1cee28): a stamp must not land in
 * the firmware's own text, and every slot must be plain shared DRAM.
 */
static const u32 scsc_mark_slots[] = {
	0x200000, 0x240000, 0x280000, 0x2c0000,
	0x300000, 0x340000, 0x380000, 0x3c0000,
};
#define SCSC_MARK_SLOTS		(ARRAY_SIZE(scsc_mark_slots) + 1)
#define SCSC_MARK_LEN		(0x2c + 4 * SCSC_MARK_SLOTS)

static int scsc_wifibt_repair_crcs(struct scsc_wifibt *scsc)
{
	void *dram;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	/* Repair the image CRCs over the patched copy so a validating ROM
	 * still accepts it, then verify the repair.
	 */
	put_unaligned_le32(ether_crc(scsc->fw_const_len - scsc->fw_hdr_len,
				    dram + scsc->fw_hdr_len),
			   dram + SCSC_FW_CONST_CRC_OFF);
	put_unaligned_le32(ether_crc(scsc->fw_len - scsc->fw_hdr_len,
				    dram + scsc->fw_hdr_len),
			   dram + SCSC_FW_CRC_OFF);

	if (ether_crc(scsc->fw_const_len - scsc->fw_hdr_len,
		      dram + scsc->fw_hdr_len) !=
	    get_unaligned_le32(dram + SCSC_FW_CONST_CRC_OFF) ||
	    ether_crc(scsc->fw_len - scsc->fw_hdr_len,
		      dram + scsc->fw_hdr_len) !=
	    get_unaligned_le32(dram + SCSC_FW_CRC_OFF)) {
		dev_err(scsc->dev, "patched CRC repair mismatch\n");
		scsc_wifibt_unmap(dram);
		return -EIO;
	}
	scsc_wifibt_unmap(dram);

	return 0;
}

static void scsc_wifibt_dump_panic(struct scsc_wifibt *scsc);
static void scsc_wifibt_panic_print(struct device *dev, const u32 *rec,
				       unsigned int words);

static int scsc_wifibt_nop_wait(struct scsc_wifibt *scsc)
{
	void *dram;
	unsigned int i;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	if (scsc_win_cmp(dram + SCSC_WAIT_OFF, scsc_wait_loop_2019,
		   SCSC_WAIT_LEN) &&
	    scsc_win_cmp(dram + SCSC_WAIT_OFF, scsc_wait_loop_2021,
		   SCSC_WAIT_LEN)) {
		dev_err(scsc->dev, "wait loop bytes mismatch, not patching\n");
		scsc_wifibt_unmap(dram);
		return -EINVAL;
	}

	for (i = 0; i < SCSC_WAIT_LEN; i += 2)
		put_unaligned_le16(0xbf00, dram + SCSC_WAIT_OFF + i);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "nopped table wait loop\n");

	return 0;
}

static int scsc_wifibt_skip_mpu(struct scsc_wifibt *scsc)
{
	void *dram;
	unsigned int i;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	if (scsc_win_cmp(dram + SCSC_MPU_OFF, scsc_mpu_block_2019, SCSC_MPU_LEN) &&
	    scsc_win_cmp(dram + SCSC_MPU_OFF, scsc_mpu_block_2021, SCSC_MPU_LEN)) {
		dev_err(scsc->dev, "mpu block bytes mismatch, not patching\n");
		scsc_wifibt_unmap(dram);
		return -EINVAL;
	}

	for (i = 0; i < SCSC_MPU_LEN; i += 2)
		put_unaligned_le16(0xbf00, dram + SCSC_MPU_OFF + i);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "nopped mpu setup block\n");

	return 0;
}

static int scsc_wifibt_skip_regions(struct scsc_wifibt *scsc)
{
	void *dram;
	unsigned int i;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	for (i = 0; i < SCSC_REGION_LEN; i += 2)
		put_unaligned_le16(0xbf00, dram + SCSC_REGION_OFF + i);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "nopped region descriptors at 0x%x\n",
		 SCSC_REGION_OFF);

	return 0;
}

static int scsc_wifibt_skip_vecregion(struct scsc_wifibt *scsc)
{
	void *dram;
	unsigned int i;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	for (i = 0; i < SCSC_VECREGION_LEN; i += 2)
		put_unaligned_le16(0xbf00, dram + SCSC_VECREGION_OFF + i);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "nopped vector region at 0x%x\n",
		 SCSC_VECREGION_OFF);

	return 0;
}

static int scsc_wifibt_nop_window(struct scsc_wifibt *scsc)
{
	void *dram;
	unsigned int i;

	if (!nop_at || nop_at & 1 || nop_len & 1 ||
	    nop_at + nop_len > scsc->mem_size) {
		dev_err(scsc->dev, "nop window 0x%x+0x%x unusable\n",
			nop_at, nop_len);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	for (i = 0; i < nop_len; i += 2)
		put_unaligned_le16(0xbf00, dram + nop_at + i);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "nopped 0x%x bytes at 0x%x\n", nop_len, nop_at);

	return 0;
}

static int scsc_wifibt_vecregion_rasr(struct scsc_wifibt *scsc)
{
	void *dram;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	put_unaligned_le32(vecregion_rasr, dram + SCSC_VECREGION_RASR);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "vector region attrs now 0x%08x\n",
		 vecregion_rasr);

	return 0;
}

static int scsc_wifibt_halt_entry(struct scsc_wifibt *scsc)
{
	u32 off = scsc->fw_entry & ~1u;
	void *dram;

	if (!(scsc->fw_entry & 1u) ||
	    off + 2 > scsc->mem_size) {
		dev_err(scsc->dev, "firmware entry 0x%x not haltable\n",
			scsc->fw_entry);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	put_unaligned_le16(0xe7fe, dram + off);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "halted firmware entry at 0x%x\n", off);

	return 0;
}

static int scsc_wifibt_mark_at(struct scsc_wifibt *scsc)
{
	u8 stub[SCSC_MARK_LEN] = {
		0x4f, 0xf0, 0xa5, 0x00, /* mov.w r0, #0xa5 */
	};
	unsigned int i, n = 0;
	void *dram;

	if (mark_at & 1 || mark_at + SCSC_MARK_LEN > scsc->mem_size - 16) {
		dev_err(scsc->dev, "mark offset 0x%x unusable\n", mark_at);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	for (i = 0; i < SCSC_MARK_SLOTS; i++) {
		u32 off = i < ARRAY_SIZE(scsc_mark_slots) ?
			  scsc_mark_slots[i] : scsc->mem_size - 16;
		u8 *code = stub + 4 + 4 * n;
		u8 *lit = stub + 0x2c + 4 * n;

		put_unaligned_le32(0x80000000u + off, lit);
		put_unaligned_le16(0x4909, code);
		put_unaligned_le16(0x6008, code + 2);
		put_unaligned_le32(SCSC_MARK_VALUE, dram + off);
		n++;
	}

	/* Spin once the stamps are out, so nothing after the halt matters. */
	put_unaligned_le16(0xe7fe, stub + 0x28);
	scsc_win_copy(dram + mark_at, stub, sizeof(stub));
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "halted and marked at 0x%x\n", mark_at);

	return 0;
}

static void scsc_mark_mov_imm(u8 *p, u32 val, bool top)
{
	u16 hi = 0xf240 | (top ? 0x80 : 0);
	u16 lo;

	hi |= ((val >> 11) & 1) << 10;
	hi |= (val >> 12) & 0xf;
	lo = ((val >> 8) & 7) << 12;
	lo |= 3 << 8;
	lo |= val & 0xff;
	put_unaligned_le16(hi, p);
	put_unaligned_le16(lo, p + 2);
}

static int scsc_wifibt_mark_mbox(struct scsc_wifibt *scsc)
{
	u8 stub[] = {
		0x02, 0x4a,			/* ldr r2, [pc, #8] */
		0x00, 0x00, 0x00, 0x00,		/* movw r3, #lo */
		0x00, 0x00, 0x00, 0x00,		/* movt r3, #hi */
		0x13, 0x60,			/* str r3, [r2] */
		0x00, 0x00, 0x00, 0x00,		/* address */
	};
	unsigned int i;
	void *dram;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(scsc_mark_mbox_sites); i++) {
		const struct scsc_mark_site *s = &scsc_mark_mbox_sites[i];

		scsc_mark_mov_imm(stub + 4, s->value & 0xffff, false);
		scsc_mark_mov_imm(stub + 8, s->value >> 16, true);
		put_unaligned_le32(0xa20e0000ul + SCSC_MARK_MBOX_REG,
				   stub + sizeof(stub) - 4);
		scsc_win_copy(dram + s->off, stub, sizeof(stub));
	}
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "mailbox probes installed at 0x%x, 0x%x, 0x%x\n",
		 scsc_mark_mbox_sites[0].off, scsc_mark_mbox_sites[1].off,
		 scsc_mark_mbox_sites[2].off);

	return 0;
}

static int scsc_wifibt_stack_fix(struct scsc_wifibt *scsc)
{
	void *dram;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	if (scsc_win_cmp(dram + 0x1b4, "\x4f\xf0\x00\x00", 4)) {
		dev_err(scsc->dev, "stack patch site mismatch, not patching\n");
		scsc_wifibt_unmap(dram);
		return -EINVAL;
	}

	/* mov.w r0, #0x800: stack top of the 32K ATCM. */
	scsc_win_copy(dram + 0x1b4, "\x4f\xf4\x00\x60", 4);
	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "early boot stack set to 0x800\n");

	return 0;
}

static int scsc_wifibt_mark_run(struct scsc_wifibt *scsc)
{
	static const u8 count_stub[] = {
		0x02, 0x4a,		/* ldr r2, [pc, #8] */
		0x13, 0x68,		/* ldr r3, [r2] */
		0x01, 0x33,		/* adds r3, #1 */
		0x13, 0x60,		/* str r3, [r2] */
		0x02, 0xe7,		/* b .+8 */
		0x00, 0xbf,		/* nop */
		0x00, 0x00, 0x00, 0x00,	/* address */
	};
	u8 stub[SCSC_MARK_RUN_LEN];
	unsigned int i;
	void *dram;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	if (mark_count) {
		memcpy(stub, count_stub, sizeof(count_stub));
		put_unaligned_le32(0x80000000u + SCSC_MARK_COUNT_SLOT,
				   stub + sizeof(count_stub) - 4);
		put_unaligned_le32(0, dram + SCSC_MARK_COUNT_SLOT);
		scsc_win_copy(dram + SCSC_MARK_COUNT_OFF, stub, sizeof(stub));
		scsc_wifibt_unmap(dram);
		dev_info(scsc->dev, "counting passes at 0x%x\n",
			 SCSC_MARK_COUNT_OFF);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(scsc_mark_run_sites); i++) {
		const struct scsc_mark_site *s = &scsc_mark_run_sites[i];

		if (s->off & 1 || s->off + SCSC_MARK_RUN_LEN >
		    scsc->mem_size - 16 || s->value + 4 > scsc->mem_size) {
			dev_err(scsc->dev, "run offset 0x%x unusable\n",
				s->off);
			scsc_wifibt_unmap(dram);
			return -EINVAL;
		}

		memcpy(stub, scsc_mark_run_stub, sizeof(scsc_mark_run_stub));
		put_unaligned_le32(0x80000000u + s->value,
				   stub + sizeof(scsc_mark_run_stub));
		put_unaligned_le32(SCSC_MARK_VALUE, dram + s->value);
		scsc_win_copy(dram + s->off, stub, sizeof(stub));
	}
	scsc_wifibt_unmap(dram);

	/* Same stamp aimed at the BT-ABOX window: if that one lands and
	 * the WiFi one does not, the main window is what is misconfigured.
	 */
	if (abox_win) {
		void __iomem *abox = ioremap(SCSC_ABOX_BASE, 0x1000);

		if (abox) {
			writel(SCSC_MARK_VALUE, abox + SCSC_MARK_RUN_SLOT);
			iounmap(abox);
		}
	}

	dev_info(scsc->dev, "passing marks at 0x%x, 0x%x, 0x%x\n",
		 scsc_mark_run_sites[0].off, scsc_mark_run_sites[1].off,
		 scsc_mark_run_sites[2].off);

	return 0;
}

static void scsc_wifibt_signal(struct scsc_wifibt *scsc)
{
	struct arm_smccc_res res;
	unsigned int tmp;
	int i, ret;

	/* Let the firmware block access DRAM (ignored on failure,
	 * like downstream).
	 */
	if (!skip_tzasc) {
		unsigned long a1 = 0, a2 = scsc->mem_start, a3 = scsc->mem_size;

		switch (tzasc_args) {
		case 1:
			a1 = 1;
			break;
		case 2:
			a2 = 0x80000000ul;
			break;
		case 3:
			a1 = scsc->mem_start;
			a2 = scsc->mem_size;
			break;
		case 4:
			a3 = 0x200000;
			break;
		default:
			break;
		}
		arm_smccc_smc(tzasc_cmd, a1, a2, a3, 0, 0, 0, 0, &res);
		dev_info(scsc->dev,
			 "TZASC smc 0x%x variant %u result 0x%lx (a1 0x%lx a2 0x%lx a3 0x%lx)\n",
			 tzasc_cmd, tzasc_args, res.a0, a1, a2, a3);
	} else {
		dev_info(scsc->dev, "TZASC config skipped by parameter\n");
	}

	/* Mailbox initial state like downstream map(): clear all shared
	 * registers on both banks first.
	 */
	for (i = 0; i < 8; i++) {
		writel(0, scsc->base + SCSC_MBOX_ISSR(i));
		writel(0, scsc->base_m4 + SCSC_MBOX_ISSR(i));
	}

	/* Mask and clear everything like downstream map(). The firmware
	 * unmasks what it uses as transports come up; starting masked is
	 * the state it expects. TOHOST signals still show in INTMSR0.
	 */
	writel(0xffff0000, scsc->base + SCSC_MBOX_INTMR0);
	writel(0x0000ffff, scsc->base + SCSC_MBOX_INTMR1);
	writel(0x0000ffff, scsc->base_m4 + SCSC_MBOX_INTMR1);
	writel(0xffff0000, scsc->base + SCSC_MBOX_INTCR0);
	writel(0x0000ffff, scsc->base + SCSC_MBOX_INTCR1);
	writel(0x0000ffff, scsc->base_m4 + SCSC_MBOX_INTCR1);

	/* Tell the R4 ROM where to jump, then release it. */
	writel(mbox0_override ? mbox0_override : scsc->sig_entry,
	       scsc->base + SCSC_MBOX_ISSR(0));
	writel(scsc->sig_mbox1, scsc->base + SCSC_MBOX_ISSR(1));
	writel(SCSC_MBOX_MAGIC, scsc->base + SCSC_MBOX_ISSR(2));
	writel(SCSC_MBOX_FW_FLAGS, scsc->base + SCSC_MBOX_ISSR(3));
	/* CPU memory barrier: registers must land before reset release. */
	wmb();

	/* Boot the block from external DRAM. Without this it never looks
	 * at the staged image, no matter the handshake.
	 */
	ret = regmap_write(scsc->pmureg, SCSC_PMU_BOOT_TEST_RST_CFG, 0);
	if (ret)
		dev_warn(scsc->dev, "failed to clear BOOT_TEST_RST_CFG: %d\n",
			 ret);

	if (mif_win) {
		ret = regmap_write(scsc->pmureg, SCSC_PMU_MIF_WIN0, mif_win);
		if (ret)
			dev_warn(scsc->dev, "MIF_WIN0 write failed: %d\n",
				 ret);
		ret = regmap_write(scsc->pmureg, SCSC_PMU_MIF_WIN1, mif_win);
		if (ret)
			dev_warn(scsc->dev, "MIF_WIN1 write failed: %d\n",
				 ret);
		regmap_read(scsc->pmureg, SCSC_PMU_MIF_WIN0, &i);
		regmap_read(scsc->pmureg, SCSC_PMU_MIF_WIN1, &tmp);
		dev_info(scsc->dev, "MIF access control set to 0x%x: %08x %08x\n",
			 mif_win, i, tmp);
	}

	if (open_wins) {
		static const unsigned int wins[] = {
			SCSC_PMU_MIF_WIN0, SCSC_PMU_MIF_WIN1,
			SCSC_PMU_PERI_WIN0, SCSC_PMU_PERI_WIN1,
			SCSC_PMU_PERI_WIN2, SCSC_PMU_PERI_WIN3,
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(wins); i++) {
			ret = regmap_write(scsc->pmureg, wins[i], 0xffffffff);
			if (ret) {
				dev_warn(scsc->dev,
					 "failed to open window 0x%x: %d\n",
					 wins[i], ret);
				break;
			}
		}
		dev_info(scsc->dev, "access windows opened\n");
	}
}

static void scsc_wifibt_observe(struct scsc_wifibt *scsc)
{
	unsigned int i, status = 0;
	int irq;

	/* Early timeline: sample the M4 status and mailbox every 50 ms
	 * for 2 s after release, logging only changes. Shows the boot
	 * choreography (who writes what when) instead of one snapshot.
	 */
	for (i = 0; i < 40; i++) {
		static u32 last_m4_0, last_m4_1, last_msr;
		u32 m4_0 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(0));
		u32 m4_1 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(1));
		u32 msr = readl(scsc->base + SCSC_MBOX_INTMSR0) >> 16;

		if (i == 0 || m4_0 != last_m4_0 || m4_1 != last_m4_1 ||
		    msr != last_msr)
			dev_info(scsc->dev,
				 "t+%ums M4 %08x %08x MSR %04x IRQs %d WDOG %d\n",
				 i * 50, m4_0, m4_1, msr,
				 atomic_read(&scsc->irq_count),
				 atomic_read(&scsc->wdog_count));

		last_m4_0 = m4_0;
		last_m4_1 = m4_1;
		last_msr = msr;
		msleep(50);
	}

	/* The whole R4 boot is over in a few milliseconds, so the
	 * timeline above starts far too late to see it.  Sample the few
	 * registers both cores touch, at a resolution we can afford to
	 * spin for, right here where the R4 has just been released.
	 */
	/* The R4 writes its panic record within microseconds of the
	 * release and clears it again just as fast, so poll for it as
	 * tightly as we can and keep the first complete copy.  Logging
	 * inside the loop is far too slow: each line costs more than the
	 * record lives.
	 */
	{
		static u32 saved[64];
		unsigned int saved_words = 0;
		void *dram = scsc_wifibt_map(scsc);

		if (dram) {
			for (i = 0; i < 200000 && !saved_words; i++) {
				if (readl(dram + SCSC_PANIC_OFF) == 2) {
					unsigned int j;

					saved_words = min_t(u32,
							    ARRAY_SIZE(saved),
							    readl(dram + SCSC_PANIC_OFF) / 4);
					for (j = 0; j < saved_words; j++)
						saved[j] = readl(dram + SCSC_PANIC_OFF + 4 * j);
				}
			}
			scsc_wifibt_unmap(dram);
		}

		if (saved_words) {
			dev_info(scsc->dev,
				 "caught a record after %u polls\n", i);
			scsc_wifibt_panic_print(scsc->dev, saved, saved_words);
		} else {
			dev_info(scsc->dev,
				 "no record in %u polls\n", i);
		}
	}

	if (fine > 0) {
		void *dram = scsc_wifibt_map(scsc);

		for (i = 0; i < fine; i++) {
			u32 m4 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(0));
			u32 r4 = readl(scsc->base + SCSC_MBOX_ISSR(0));
			u32 r4sr = readl(scsc->base + SCSC_MBOX_INTMSR1);
			u32 ver = dram ? readl(dram + SCSC_PANIC_OFF) : 0;

			if (i % 10 == 0)
				dev_info(scsc->dev,
					 "fine %3d m4 %08x r4 %08x r4sr %08x ver %08x %08x %08x %08x %08x %08x\n",
					 i, m4, r4, r4sr, ver,
					 dram ? readl(dram + SCSC_PANIC_OFF + 4) : 0,
					 dram ? readl(dram + 0x160840) : 0,
					 dram ? readl(dram + 0x160844) : 0,
					 dram ? readl(dram + 0x160848) : 0,
					 dram ? readl(dram + 0x16084c) : 0);
			udelay(fine_us);
		}
		if (dram)
			scsc_wifibt_unmap(dram);
	}

	scsc_wifibt_dump_panic(scsc);

	for (i = 0; i < 10; i++) {
		msleep(100);
		status = readl(scsc->base + SCSC_MBOX_INTMSR0) >> 16;
		if (status)
			break;
	}

	dev_info(scsc->dev, "R4 response status 0x%04x raw 0x%04x, MBOX IRQs seen %d\n",
		 status, readl(scsc->base + SCSC_MBOX_INTSR0) >> 16,
		 atomic_read(&scsc->irq_count));
	dev_info(scsc->dev, "MBOX regs %08x %08x %08x %08x\n",
		 readl(scsc->base + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base + SCSC_MBOX_ISSR(3)));
	dev_info(scsc->dev, "M4 status 0x%04x, M4 regs %08x %08x %08x %08x\n",
		 readl(scsc->base_m4 + SCSC_MBOX_INTMSR0) >> 16,
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(3)));

	/* Kick the reserved panic bit (FROMHOST 0): unmask it, re-arm the
	 * watchdog, and pulse. A running firmware with panic
	 * infrastructure answers (panic record and/or watchdog); a
	 * pre-transport stall stays silent. With gdb_probe set, queue a
	 * halt-reason query first: a live GDB stub answers in its stream
	 * instead of panicking.
	 */
	irq = platform_get_irq_byname(to_platform_device(scsc->dev), "WDOG");
	if (irq >= 0 && atomic_read(&scsc->wdog_count))
		enable_irq(irq);

	writel(readl(scsc->base + SCSC_MBOX_INTMR1) & ~1u,
	       scsc->base + SCSC_MBOX_INTMR1);

	if (gdb_probe) {
		static const u8 query[] = { '$', '?', '#', '3', 'f' };
		void *dram = scsc_wifibt_map(scsc);

		if (dram) {
			scsc_win_copy(dram + scsc->gdb_fa_buf, query,
				      sizeof(query));
			put_unaligned_le32(2, dram + scsc->gdb_fa_widx);
			scsc_wifibt_unmap(dram);
			dev_info(scsc->dev, "gdb query queued\n");
		}
	}

	if (poke_intgr)
		writel(1u, scsc->base + SCSC_MBOX_INTGR1);
	dev_info(scsc->dev, "poked FROMHOST bit 0 (pulse %s) MR1 now %08x\n",
		 poke_intgr ? "sent" : "withheld",
		 readl(scsc->base + SCSC_MBOX_INTMR1));
}

static u32 scsc_wifibt_dram_crc(struct scsc_wifibt *scsc)
{
	void *dram;
	u32 crc = 0;

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return 0;

	crc = crc32_le(~0, dram, scsc->mem_size);
	scsc_wifibt_unmap(dram);

	return crc;
}

/* Runs 5 s after probe: any change in the shared window, or in the PMU /
 * mailbox state, means the firmware block did something on its own.
 */
/* R4 panic record (header field 0x160804, v2 layout per the downstream
 * fw_panic_record.c: version, byte length, two clock stamps,
 * R0-R12/SP/LR/SPSR/PC/CPSR, panic info, XOR checksum).  A core that
 * faults writes it into the shared window, and the firmware clears it
 * again shortly after, so it has to be read early to catch anything.
 */
static void scsc_wifibt_panic_print(struct device *dev, const u32 *rec,
			       unsigned int words)
{
	static const char * const regs[18] = {
		"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
		"r8", "r9", "r10", "r11", "r12", "sp", "lr",
		"spsr", "pc", "cpsr",
	};
	u32 sum = 0xa5a5a5a5;
	unsigned int i;

	if (rec[0] != 2) {
		dev_info(dev, "no R4 panic record (v=%u)\n", rec[0]);
		return;
	}

	dev_info(dev, "R4 panic: len %u bytes, t1m %u t32k %u\n",
		 rec[1], rec[2], rec[3]);
	for (i = 0; i < 18 && 4 + i < words; i++)
		dev_info(dev, "  %-4s %08x\n", regs[i], rec[4 + i]);

	for (i = 22; i + 1 < words; i++)
		sum ^= rec[i];
	sum ^= 0xa5a5a5a5;
	dev_info(dev, "  info:");
	for (i = 22; i + 1 < words; i++)
		dev_info(dev, " %08x", rec[i]);
	dev_info(dev, "\n  cksum rec %08x calc %08x %s\n",
		 rec[words - 1], sum,
		 rec[words - 1] == sum ? "OK" : "BAD");
}

static void scsc_wifibt_dump_panic(struct scsc_wifibt *scsc)
{
	void *dram = scsc_wifibt_map(scsc);
	u32 rec[64] = { 0 };
	unsigned int i, words = 0;

	if (dram) {
		words = min_t(u32, ARRAY_SIZE(rec),
			      readl(dram + SCSC_PANIC_OFF) / 4);
		for (i = 0; i < words; i++)
			rec[i] = readl(dram + SCSC_PANIC_OFF + 4 * i);
		scsc_wifibt_unmap(dram);
	}

	scsc_wifibt_panic_print(scsc->dev, rec, words);
}

static void scsc_wifibt_check_work(struct work_struct *work)
{
	struct scsc_wifibt *scsc = container_of(to_delayed_work(work),
						struct scsc_wifibt,
						check_work);
	unsigned int stat, seq, status, i;
	u32 crc;

	/* A single snapshot at t+5s cannot tell an immediate fault from
	 * a wait that times out seconds later, so sample the state the
	 * R4 and M4 touch on a timeline and let the reader see when
	 * progress actually stops.
	 */
	if (timeline > 0) {
		unsigned int t;

		for (t = 0; t < 20; t++) {
			u32 m4 = readl(scsc->base_m4 + SCSC_MBOX_ISSR(0));
			u32 m4sr = readl(scsc->base_m4 + SCSC_MBOX_INTMSR1);
			u32 r4sr = readl(scsc->base + SCSC_MBOX_INTMSR1);

			dev_info(scsc->dev,
				 "tl t+%4ums crc %08x m4issr %08x m4sr %08x r4sr %08x wd %d\n",
				 500 * t, scsc_wifibt_dram_crc(scsc), m4, m4sr,
				 r4sr, atomic_read(&scsc->wdog_count));
			msleep(timeline);
		}
	}

	crc = scsc_wifibt_dram_crc(scsc);
	regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &stat);
	regmap_read(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_STAT, &seq);
	status = readl(scsc->base + SCSC_MBOX_INTMSR0) >> 16;

	dev_info(scsc->dev,
		 "5s check: DRAM crc 0x%08x (was 0x%08x) %s, WIFI_STAT 0x%08x, seq 0x%02x, status 0x%04x, IRQs %d, WDOG %d\n",
		 crc, scsc->dram_crc,
		 crc == scsc->dram_crc ? "unchanged" : "CHANGED",
		 stat, (seq & SCSC_PMU_STATES) >> 16, status,
		 atomic_read(&scsc->irq_count), atomic_read(&scsc->wdog_count));

	scsc_wifibt_dump_panic(scsc);

	if (mark_mbox) {
		u32 val = readl(scsc->base_m4 + SCSC_MARK_MBOX_REG);
		u32 val2 = readl(scsc->base_m4 + SCSC_MARK_MBOX_REG + 0x10);

		dev_info(scsc->dev,
			 "mbox probe word 0x%08x/0x%08x: %s\n", val, val2,
			 val == scsc_mark_mbox_sites[0].value ? "reached 0x1a8" :
			 val == scsc_mark_mbox_sites[1].value ? "reached 0x212" :
			 val == scsc_mark_mbox_sites[2].value ? "reached 0x236" :
			 "no probe landed");
	}

	if (mark_run || mark_count) {
		void *dram = scsc_wifibt_map(scsc);

		if (dram && mark_count) {
			dev_info(scsc->dev, "count 0x%x passes: %u\n",
				 SCSC_MARK_COUNT_OFF,
				 readl(dram + SCSC_MARK_COUNT_SLOT));
		} else if (dram) {
			for (i = 0; i < ARRAY_SIZE(scsc_mark_run_sites); i++) {
				const struct scsc_mark_site *s =
					&scsc_mark_run_sites[i];
				u32 val = readl(dram + s->value);

				dev_info(scsc->dev,
					 "mark 0x%x: slot 0x%x = 0x%08x %s\n",
					 s->off, s->value, val,
					 val == 0xa5a5a5a5 ? "PASSED" :
					 "not reached");
			}
			scsc_wifibt_unmap(dram);
		}

		if (abox_win) {
			void __iomem *abox = ioremap(SCSC_ABOX_BASE, 0x1000);

			if (abox) {
				u32 val = readl(abox +
						scsc_mark_run_sites[0].value);

				dev_info(scsc->dev,
					 "mark abox slot 0x%x = 0x%08x %s\n",
					 scsc_mark_run_sites[0].value, val,
					 val == 0xa5a5a5a5 ? "PASSED" :
					 "not reached");
				iounmap(abox);
			}
		}
	}

	if (mark_at) {
		void *dram = scsc_wifibt_map(scsc);
		unsigned int i, hits = 0;
		u8 seen[SCSC_MARK_SLOTS] = { 0 };
		u32 raw[SCSC_MARK_SLOTS];

		memset(raw, 0, sizeof(raw));
		if (dram) {
			for (i = 0; i < SCSC_MARK_SLOTS; i++) {
				u32 off = i < ARRAY_SIZE(scsc_mark_slots) ?
					  scsc_mark_slots[i] :
					  scsc->mem_size - 16;
				u32 val = readl(dram + off);

				raw[i] = val;
				seen[i] = val == SCSC_MARK_STAMP;
				hits += seen[i];
			}
			scsc_wifibt_unmap(dram);
		}
		/* Raw values matter: the M4 writes the shared window too, so a
		 * slot that is neither the preset nor our stamp was taken
		 * from under us and the halt point is still unknown.
		 */
		dev_info(scsc->dev, "mark at 0x%x: %u/%u ours %*ph raw %*ph\n",
			 mark_at, hits, (unsigned int)SCSC_MARK_SLOTS,
			 (int)sizeof(seen), seen,
			 (int)sizeof(raw), raw);
		for (i = 0; i < 12; i++) {
			u32 val = readl(scsc->base + 0x80 + 4 * i);

			if (val == SCSC_MARK_MBOX_STAMP)
				dev_info(scsc->dev,
					 "mark at 0x%x: R4 mailbox stamp at MBOX+0x%02x\n",
					 mark_at, 0x80 + 4 * i);
		}
	}

	if (r4_probe || patch_entry) {
		void *dram = scsc_wifibt_map(scsc);
		u32 off, found = 0;

		if (!dram)
			return;

		/* The stock image holds one natural marker at 0x88bcc;
		 * anything else is an R4 store.
		 */
		for (off = 0; off < scsc->mem_size; off += 4) {
			if (readl(dram + off) != SCSC_PROBE_MARKER)
				continue;
			if (found < 8)
				dev_info(scsc->dev, "marker at 0x%x\n", off);
			found++;
		}
		scsc_wifibt_unmap(dram);
		dev_info(scsc->dev, "marker scan: %u hits\n", found);
	}
	dev_info(scsc->dev,
		 "scan M4 status 0x%04x, M4 regs %08x %08x %08x %08x\n",
		 readl(scsc->base_m4 + SCSC_MBOX_INTMSR0) >> 16,
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(3)));
	dev_info(scsc->dev,
		 "scan MBOX ctl GR0 %08x MR0 %08x SR0 %08x GR1 %08x MR1 %08x SR1 %08x MSR1 %08x INIT %08x\n",
		 readl(scsc->base + SCSC_MBOX_INTGR0),
		 readl(scsc->base + SCSC_MBOX_INTMR0),
		 readl(scsc->base + SCSC_MBOX_INTSR0),
		 readl(scsc->base + SCSC_MBOX_INTGR1),
		 readl(scsc->base + SCSC_MBOX_INTMR1),
		 readl(scsc->base + SCSC_MBOX_INTSR1),
		 readl(scsc->base + SCSC_MBOX_INTMSR1),
		 readl(scsc->base + SCSC_MBOX_MIF_INIT));
	dev_info(scsc->dev, "scan ISSR4-7 %08x %08x %08x %08x\n",
		 readl(scsc->base + SCSC_MBOX_ISSR(4)),
		 readl(scsc->base + SCSC_MBOX_ISSR(5)),
		 readl(scsc->base + SCSC_MBOX_ISSR(6)),
		 readl(scsc->base + SCSC_MBOX_ISSR(7)));
}

static void scsc_wifibt_scan(struct scsc_wifibt *scsc)
{
	unsigned int stat, seq, status;
	void *dram;
	u32 off, found = 0;
	u32 crc;

	/* A single snapshot at t+5s cannot tell an immediate fault from
	 * a wait that times out seconds later, so sample the state the
	 * R4 and M4 touch on a timeline and let the reader see when
	 * progress actually stops.
	 */
	crc = scsc_wifibt_dram_crc(scsc);
	regmap_read(scsc->pmureg, SCSC_PMU_WIFI_STAT, &stat);
	regmap_read(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_STAT, &seq);
	status = readl(scsc->base + SCSC_MBOX_INTMSR0) >> 16;

	dev_info(scsc->dev,
		 "scan: DRAM crc 0x%08x (probe 0x%08x) %s, STAT 0x%08x, seq 0x%02x, status 0x%04x, IRQs %d, WDOG %d\n",
		 crc, scsc->dram_crc,
		 crc == scsc->dram_crc ? "unchanged" : "CHANGED",
		 stat, (seq & SCSC_PMU_STATES) >> 16, status,
		 atomic_read(&scsc->irq_count), atomic_read(&scsc->wdog_count));

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return;

	for (off = 0; off < scsc->mem_size; off += 4) {
		if (readl(dram + off) != SCSC_PROBE_MARKER)
			continue;
		if (found < 8)
			dev_info(scsc->dev, "scan marker at 0x%x\n", off);
		found++;
	}
	scsc_wifibt_unmap(dram);
	dev_info(scsc->dev, "scan markers: %u hits\n", found);

	dev_info(scsc->dev,
		 "scan M4 status 0x%04x, M4 regs %08x %08x %08x %08x\n",
		 readl(scsc->base_m4 + SCSC_MBOX_INTMSR0) >> 16,
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(0)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(1)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(2)),
		 readl(scsc->base_m4 + SCSC_MBOX_ISSR(3)));
	dev_info(scsc->dev, "scan M4 INTMR1 %08x INTGR1 %08x\n",
		 readl(scsc->base_m4 + SCSC_MBOX_INTMR1),
		 readl(scsc->base_m4 + SCSC_MBOX_INTGR1));
	dev_info(scsc->dev,
		 "scan MBOX ctl GR0 %08x MR0 %08x SR0 %08x GR1 %08x MR1 %08x SR1 %08x MSR1 %08x INIT %08x\n",
		 readl(scsc->base + SCSC_MBOX_INTGR0),
		 readl(scsc->base + SCSC_MBOX_INTMR0),
		 readl(scsc->base + SCSC_MBOX_INTSR0),
		 readl(scsc->base + SCSC_MBOX_INTGR1),
		 readl(scsc->base + SCSC_MBOX_INTMR1),
		 readl(scsc->base + SCSC_MBOX_INTSR1),
		 readl(scsc->base + SCSC_MBOX_INTMSR1),
		 readl(scsc->base + SCSC_MBOX_MIF_INIT));
	dev_info(scsc->dev, "scan ISSR4-7 %08x %08x %08x %08x\n",
		 readl(scsc->base + SCSC_MBOX_ISSR(4)),
		 readl(scsc->base + SCSC_MBOX_ISSR(5)),
		 readl(scsc->base + SCSC_MBOX_ISSR(6)),
		 readl(scsc->base + SCSC_MBOX_ISSR(7)));

	{
		unsigned int wins[6] = {
			SCSC_PMU_MIF_WIN0, SCSC_PMU_MIF_WIN1,
			SCSC_PMU_PERI_WIN0, SCSC_PMU_PERI_WIN1,
			SCSC_PMU_PERI_WIN2, SCSC_PMU_PERI_WIN3,
		};
		unsigned int vals[6], i;

		for (i = 0; i < 6; i++)
			regmap_read(scsc->pmureg, wins[i], &vals[i]);

		dev_info(scsc->dev,
			 "scan access wins %08x %08x %08x %08x %08x %08x\n",
			 vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
	}

	if (scsc->mxlog_off) {
		void *dram = scsc_wifibt_map(scsc);

		if (dram) {
			dev_info(scsc->dev,
				 "scan mxlog ridx %08x widx %08x data %08x %08x %08x %08x\n",
				 readl(dram + scsc->mxlog_off +
				       SCSC_MXLOG_BUF_LEN),
				 readl(dram + scsc->mxlog_off +
				       SCSC_MXLOG_BUF_LEN + 4),
				 readl(dram + scsc->mxlog_off),
				 readl(dram + scsc->mxlog_off + 4),
				 readl(dram + scsc->mxlog_off + 8),
				 readl(dram + scsc->mxlog_off + 12));
			dev_info(scsc->dev,
				 "scan gdb-ta %08x %08x %08x %08x\n",
				 readl(dram + scsc->gdb_ta_buf),
				 readl(dram + scsc->gdb_ta_buf + 4),
				 readl(dram + scsc->gdb_ta_buf + 8),
				 readl(dram + scsc->gdb_ta_buf + 12));
			dev_info(scsc->dev,
				 "scan mgmt-ta %08x %08x widx %08x\n",
				 readl(dram + scsc->mgmt_ta_buf),
				 readl(dram + scsc->mgmt_ta_buf + 4),
				 readl(dram + scsc->mgmt_ta_widx));
			dev_info(scsc->dev,
				 "scan mgmt-fa %08x %08x widx %08x\n",
				 readl(dram + scsc->mgmt_fa_buf),
				 readl(dram + scsc->mgmt_fa_buf + 4),
				 readl(dram + scsc->mgmt_fa_widx));
			scsc_wifibt_unmap(dram);
		}
	}
}

static ssize_t scan_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct scsc_wifibt *scsc = dev_get_drvdata(dev);

	scsc_wifibt_scan(scsc);

	return count;
}
static DEVICE_ATTR_WO(scan);

/* Recover the block the way stock does on a reset request: full power
 * cycle plus a fresh handshake, then observe. If the ROM boots in
 * stages separated by reset requests, this runs stage 2.
 */
static int scsc_wifibt_power_on(struct scsc_wifibt *scsc);
static void scsc_wifibt_power_off(struct scsc_wifibt *scsc);
static void scsc_wifibt_signal(struct scsc_wifibt *scsc);
static void scsc_wifibt_observe(struct scsc_wifibt *scsc);

static ssize_t recover_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct scsc_wifibt *scsc = dev_get_drvdata(dev);

	scsc_wifibt_power_off(scsc);
	msleep(100);
	scsc_wifibt_signal(scsc);

	if (scsc_wifibt_power_on(scsc))
		dev_warn(dev, "recovery power-on failed\n");
	else
		dev_info(dev, "recovered, watching\n");

	scsc_wifibt_observe(scsc);

	return count;
}
static DEVICE_ATTR_WO(recover);

static int scsc_wifibt_fw_stage(struct scsc_wifibt *scsc,
				const struct firmware *fw)
{
	void *dram;

	if (fw->size > scsc->mem_size) {
		dev_err(scsc->dev, "firmware %zu larger than shared memory %zu\n",
			fw->size, scsc->mem_size);
		return -EINVAL;
	}

	dram = scsc_wifibt_map(scsc);
	if (!dram)
		return -ENOMEM;

	scsc_win_copy(dram, fw->data, fw->size);

	if (scsc_win_cmp(dram, fw->data, fw->size)) {
		dev_err(scsc->dev, "firmware DRAM readback mismatch\n");
		scsc_wifibt_unmap(dram);
		return -EIO;
	}

	/* Wipe any panic record left in the carveout: a warm reboot keeps
	 * the shared window, and a stale record would look like a fresh
	 * fault on this boot.
	 */
	scsc_win_set(dram + SCSC_PANIC_OFF, 0, SCSC_PANIC_LEN);
	dev_info(scsc->dev, "panic record area cleared (%u bytes)\n",
		 SCSC_PANIC_LEN);

	if (fill_gap) {
		scsc_win_set(dram + fw->size, 0xaa,
			     scsc->mem_size - fw->size);
		dev_info(scsc->dev, "gap filled 0x%zx bytes with 0xaa\n",
			 scsc->mem_size - fw->size);
	}

	scsc_wifibt_unmap(dram);

	dev_info(scsc->dev, "firmware staged in shared memory, verified\n");

	return 0;
}
static void scsc_wifibt_power_off(struct scsc_wifibt *scsc)
{
	unsigned int val;
	int ret;

	/* Minimal shutdown: hold the cores, revoke the window, drop power.
	 * The suspend-time low-power controls are deliberately left alone:
	 * gating them here wedges warm re-release, and nothing on the
	 * start path restores them.
	 */
	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
				 SCSC_PMU_WIFI_RESET_SET,
				 SCSC_PMU_WIFI_RESET_SET);
	if (ret) {
		dev_warn(scsc->dev, "failed to hold reset: %d\n", ret);
		return;
	}

	ret = regmap_read_poll_timeout(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_STAT,
				       val, ((val & SCSC_PMU_STATES) >> 16) ==
				       SCSC_PMU_SM_DOWN, 1000, 500000);
	if (ret)
		dev_warn(scsc->dev,
			 "timeout waiting for DOWN state, STAT 0x%08x\n", val);

	/* Revoke the shared-memory window. */
	regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG0, 0);
	regmap_write(scsc->pmureg, SCSC_PMU_MEM_CONFIG1, 0);

	/* Full power-down so the next probe gets a real PWRON edge instead
	 * of a wedged warm re-release (downstream reset case 1).
	 */
	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
				 SCSC_PMU_WIFI_PWRON, 0);
	if (ret)
		dev_warn(scsc->dev, "failed to clear PWRON: %d\n", ret);
}

static irqreturn_t scsc_wifibt_wdog_irq(int irq, void *data)
{
	struct scsc_wifibt *scsc = data;

	atomic_inc(&scsc->wdog_count);
	dev_info(scsc->dev, "WDOG IRQ #%d, disabling\n",
		 atomic_read(&scsc->wdog_count));
	/* Ack like downstream, then shut it up until re-probe. */
	disable_irq_nosync(irq);
	regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
			   SCSC_PMU_WIFI_RESET_REQ_CLR,
			   SCSC_PMU_WIFI_RESET_REQ_CLR);

	return IRQ_HANDLED;
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

	/* R4 mailbox bank (index 0); M4 bank (index 1) is read-only for now. */
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
	dev_info(dev, "pre-release R4 INTMR1 0x%08x M4 INTMR1 0x%08x\n",
		 readl(scsc->base + SCSC_MBOX_INTMR1),
		 readl(scsc->base_m4 + SCSC_MBOX_INTMR1));

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

	/* PMU syscon: state readout first, then power sequencing. */
	pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						 "samsung,syscon-phandle");
	if (IS_ERR(pmureg))
		return dev_err_probe(dev, PTR_ERR(pmureg),
				     "failed to get PMU syscon\n");

	scsc->pmureg = pmureg;
	scsc->mem_start = rmem->base;
	scsc->mem_size = rmem->size;

	ret = regmap_read(pmureg, SCSC_PMU_WIFI_STAT, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read WIFI_STAT\n");

	dev_info(dev, "PMU WIFI_STAT 0x%08x\n", val);

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

	if (!signal_r4) {
		dev_info(dev, "R4 signalling disabled by parameter\n");
	} else {
		if (r4_probe) {
			ret = scsc_wifibt_r4_probe(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to stage probe\n");
		} else {
			if (!null_mxconf) {
				ret = scsc_wifibt_mxconf(scsc);
				if (ret)
					return dev_err_probe(dev, ret,
							     "failed to write mxconf\n");
			}

			scsc->sig_entry = scsc->fw_entry;
			scsc->sig_mbox1 = null_mxconf ? 0 : scsc->mxconf_off;
		}

		if (graft_mm) {
			ret = scsc_wifibt_graft_mm(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to graft\n");
		} else if (patch_entry) {
			ret = scsc_wifibt_patch_entry(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to patch entry\n");
		}

		if (nop_wait) {
			ret = scsc_wifibt_nop_wait(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to nop wait\n");
		}

		if (skip_mpu) {
			ret = scsc_wifibt_skip_mpu(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to skip mpu\n");
		}

		if (halt_entry) {
			ret = scsc_wifibt_halt_entry(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to halt entry\n");
		}

		if (skip_regions) {
			ret = scsc_wifibt_skip_regions(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to skip regions\n");
		}

		if (nop_at) {
			ret = scsc_wifibt_nop_window(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to nop window\n");
		}

		if (vecregion_rasr) {
			ret = scsc_wifibt_vecregion_rasr(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to set vec rasr\n");
		}

		if (skip_vecregion) {
			ret = scsc_wifibt_skip_vecregion(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to skip vec region\n");
		}

		if (mark_mbox) {
			ret = scsc_wifibt_mark_mbox(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to install mailbox probes\n");
		}

		if (stack_fix) {
			ret = scsc_wifibt_stack_fix(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to fix early stack\n");
		}

		if (mark_run || mark_count) {
			ret = scsc_wifibt_mark_run(scsc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to stamp boot points\n");
		}

	if (mark_at) {
		ret = scsc_wifibt_mark_at(scsc);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to mark at 0x%x\n",
					     mark_at);
	}

	if (patch_entry || nop_wait || graft_mm || skip_mpu ||
	    halt_entry || mark_at || mark_run || mark_count || mark_mbox ||
	    stack_fix ||
	    skip_regions || skip_vecregion || vecregion_rasr || nop_at) {
		ret = scsc_wifibt_repair_crcs(scsc);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to repair CRCs\n");
	}

		scsc_wifibt_signal(scsc);
	}

	ret = scsc_wifibt_power_on(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	scsc_wifibt_observe(scsc);

	/* Baseline for the delayed signs-of-life check. */
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
MODULE_DESCRIPTION("Samsung SCSC WiFi/BT mailbox stub driver (probe only)");
MODULE_LICENSE("GPL");
