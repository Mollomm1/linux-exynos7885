// SPDX-License-Identifier: GPL-2.0-only
/* Explicit-load Exynos7885 ABOX remoteproc driver. */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/memremap.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/sizes.h>
#include <linux/soc/samsung/exynos7885-abox.h>
#include <linux/vmalloc.h>

#define ABOX_SRAM_SIZE		0x28000
#define ABOX_DRAM_SIZE		(SZ_8M + SZ_4M)
#define ABOX_DRAM_IOVA		0x80000000
#define ABOX_PCM_IOVA		0x81000000
#define ABOX_PCM_SIZE		SZ_128K
#define ABOX_RDMA_BASE		0x1000
#define ABOX_RDMA_INTERVAL	0x100
#define ABOX_RDMA_STATUS	0x30
#define ABOX_RDMA_PROGRESS	BIT(31)
#define ABOX_RDMA_RBUF_OFFSET	GENMASK(28, 16)
#define ABOX_RDMA_RBUF_CNT	GENMASK(12, 0)
#define ABOX_IRQ_RDMA0_BUF_EMPTY	11
#define ABOX_ROUTE_CTRL0	0x0040
#define ABOX_ROUTE_UAIF_SPK(n)	GENMASK(4 * (n) + 3, 4 * (n))
#define ABOX_ROUTE_SIFS1	2
#define ABOX_SPUS_CTRL0		0x0200
#define ABOX_SPUS_RDMA0_CHAIN	GENMASK(3, 0)
#define ABOX_SPUS_RDMA0_ASRC	BIT(0)
#define ABOX_SPUS_CTRL1		0x0204
#define ABOX_SPUS_SIFS1_SEL	GENMASK(18, 16)
#define ABOX_UAIF_CTRL0(n)	(0x0500 + (n) * 0x10)
#define ABOX_UAIF_CTRL1(n)	(0x0504 + (n) * 0x10)
#define ABOX_UAIF_SPK_ENABLE	BIT(0)
#define ABOX_UAIF_MIC_ENABLE	BIT(1)
#define ABOX_UAIF_MODE		BIT(2)
#define ABOX_UAIF_FORMAT	GENMASK(28, 24)
#define ABOX_UAIF_BCLK_POLARITY	BIT(23)
#define ABOX_UAIF_WS_MODE	BIT(22)
#define ABOX_UAIF_WS_POLARITY	BIT(21)
#define ABOX_UAIF_SLOT_MAX	GENMASK(20, 18)
#define ABOX_UAIF_SBIT_MAX	GENMASK(17, 12)
#define ABOX_UAIF_VALID_START	GENMASK(11, 6)
#define ABOX_UAIF_VALID_END	GENMASK(5, 0)
#define ABOX_NUM_UAIF		5
#define ABOX_PMU_IOVA		0x11c80000
#define ABOX_PMU_SIZE		SZ_64K
#define ABOX_AUDSYS_IOVA	0x12090000
#define ABOX_AUDSYS_SIZE	PAGE_SIZE
#define ABOX_SYSPOWER_CTRL	0x0010
#define ABOX_SYSPOWER_STATUS	0x0014
#define ABOX_SYSPOWER_ON	BIT(0)
#define ABOX_REMAP_ADDR		0x0028
#define ABOX_TIMER0_CTRL1	0x0604
#define ABOX_TIMER_CTRL0(n)	(0x0600 + (n) * 0x20)
#define ABOX_NUM_TIMERS		4
#define ABOX_SRAM_PHYS		0x14b00000
#define ABOX_DISPAUD_STATUS	0x4024
#define ABOX_CA7_CONFIGURATION	0x2520
#define ABOX_CA7_STATUS		0x2524
#define ABOX_CA7_OPTION		0x2528
#define ABOX_CA7_LOCAL_PWR	BIT(0)
#define ABOX_CA7_STATUS_WFI	BIT(28)
#define ABOX_CA7_STATUS_ON	BIT(0)
#define ABOX_CA7_ENABLE		BIT(15)
#define ABOX_IPC_TX_OFFSET	0x22000
#define ABOX_IPC_TX_ACK_OFFSET	0x222fc
#define ABOX_IPC_MSG_SIZE	(ABOX_IPC_TX_ACK_OFFSET - ABOX_IPC_TX_OFFSET)
#define ABOX_IPC_RX_OFFSET	0x22300
#define ABOX_IPC_RX_ACK_OFFSET	0x225fc
#define ABOX_IPC_WORDS		EXYNOS7885_ABOX_IPC_WORDS
#define ABOX_GIC_SGIR		0x0f00
#define ABOX_IPC_SYSTEM		1
#define ABOX_IPC_CONFIG		0xa
#define ABOX_SET_OUT1_SAMPLE_RATE	2
#define ABOX_SET_OUT1_FORMAT	0x11
#define ABOX_SYSTEM_SUSPEND	1
#define ABOX_BOOT_DONE		3
#define ABOX_IPC_TIMEOUT_US	20000
#define ABOX_WFI_TIMEOUT_US	100000
#define ABOX_STOP_TIMEOUT_US	20000
#define ABOX_FAULT_LIMIT	8
#define ABOX_BOOTARGS_SIZE	SZ_512

#define ABOX_BOOT_TIMEOUT_MS	10000
#define ABOX_AUD_PLL_RATE_48K	1179648040
#define ABOX_AUD_PLL_RATE_44K	1083801605
/*
 * AUDIF is the PLL divided by 48 (24.576 or 22.5792 MHz). The PLL runs a few
 * Hz above 48 times that, and dividers never round above a request.
 */
#define ABOX_AUDIF_DIV		48
#define ABOX_AUDIF_RATE(pll)	DIV_ROUND_UP(pll, ABOX_AUDIF_DIV)
#define ABOX_CA7_RATE		393216000
#define ABOX_GIC_IRQ_LIMIT	32
#define ABOX_GIC_SPURIOUS	1021
#define ABOX_NUM_CLOCKS		8

enum exynos7885_abox_clock_id {
	ABOX_CLK_PLL,
	ABOX_CLK_CA7,
	ABOX_CLK_AUDIF,
	ABOX_CLK_UAIF3_DIV,
	ABOX_CLK_UAIF3_MUX,
	ABOX_CLK_ACLK,
	ABOX_CLK_UAIF3_BCLK,
	ABOX_CLK_UAIF3_SYNC,
};

static const char * const exynos7885_abox_clock_names[ABOX_NUM_CLOCKS] = {
	"pll", "ca7", "audif", "uaif3-div", "uaif3-mux", "aclk",
	"uaif3-bclk", "uaif3-sync",
};

struct exynos7885_abox_uaif {
	struct clk *div;
	struct clk *mux;
	struct clk *bclk;
	struct clk *sync;
	bool enabled;
};

struct exynos7885_abox_rproc {
	struct device *dev;
	struct regmap *pmu;
	struct device *sysmmu_dev;
	struct clk_bulk_data clocks[ABOX_NUM_CLOCKS];
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_idle;
	void __iomem *sfr;
	void __iomem *sysreg;
	struct reserved_mem *dram_rmem;
	struct reserved_mem *pcm_rmem;
	void *pcm_area;
	struct rproc_mem_entry *dram;
	void __iomem *sram;
	void __iomem *gicd;
	void __iomem *gicc;
	struct completion boot_done;
	struct mutex ipc_lock;
	struct mutex handler_lock;
	/* Serializes PCM mapping with RDMA teardown and remoteproc stop. */
	struct mutex pcm_lock;
	/* Serializes UAIF3 register and clock operations. */
	struct mutex uaif_lock;
	exynos7885_abox_ipc_handler_t ipc_handler;
	void *ipc_handler_data;
	int irq;
	u32 firmware_version;
	size_t sram_size;
	bool rpm_held;
	bool aclk_held;
	bool ca7_held;
	int sysmmu_irq;
	bool sysmmu_irq_off;
	atomic_t faults;
	bool pmu_mapped;
	bool audsys_mapped;
	bool sram_loaded;
	bool irq_requested;
	bool irq_enabled;
	unsigned long enabled_clocks;
	bool pins_active_selected;
	bool dram_requested;
	bool pcm_mapped;
	bool pcm_shutting_down;
	/* Indexed by UAIF number; only UAIF0 and UAIF3 are wired up. */
	struct exynos7885_abox_uaif uaif[ABOX_NUM_UAIF];
	/*
	 * Codec master clock on the UAIF2 clock pin. A leaf lock: the clk core
	 * calls in with its prepare lock held, and uaif_lock nests that lock.
	 */
	struct clk_hw mclk_hw;
	struct mutex mclk_lock;
	bool mclk_allowed;
	bool mclk_on;
	bool firmware_running;
	bool firmware_ready;
	bool stopping;
};

static bool exynos7885_abox_is_dma_irq(unsigned int irq)
{
	return irq == 8 || irq == 9 || (irq >= 11 && irq <= 14);
}

static void exynos7885_abox_handle_ipc(struct exynos7885_abox_rproc *abox, unsigned int irq)
{
	u32 message[ABOX_IPC_MSG_SIZE / sizeof(u32)];
	u32 ipc_id, msg_type, firmware_version = 0;
	exynos7885_abox_ipc_handler_t handler;
	void *data;

	if (exynos7885_abox_is_dma_irq(irq)) {
		/*
		 * RDMA0 finished a period. This carries no payload; report it
		 * as a POINTER message with a zero pointer, meaning "read the
		 * position from RDMA0 STATUS".
		 */
		if (irq != ABOX_IRQ_RDMA0_BUF_EMPTY)
			return;
		memset(message, 0, sizeof(message));
		message[0] = EXYNOS7885_ABOX_IPC_PCM_PLAYBACK;
		message[2] = EXYNOS7885_ABOX_PCM_POINTER;
		goto dispatch;
	}

	memcpy_fromio(message, abox->sram + ABOX_IPC_RX_OFFSET,
		      sizeof(message));
	ipc_id = message[0];
	msg_type = message[2];
	if (ipc_id == ABOX_IPC_SYSTEM && msg_type == ABOX_BOOT_DONE)
		firmware_version = message[5];

	/* Acknowledge non-DMA messages after reading the shared payload. */
	writel(0, abox->sram + ABOX_IPC_RX_ACK_OFFSET);

	if (ipc_id == ABOX_IPC_SYSTEM && msg_type == ABOX_BOOT_DONE) {
		WRITE_ONCE(abox->firmware_version, firmware_version);
		complete(&abox->boot_done);
		return;
	}

dispatch:
	/* Pair with registration so the callback's data is already visible. */
	handler = smp_load_acquire(&abox->ipc_handler);
	data = READ_ONCE(abox->ipc_handler_data);
	if (handler && READ_ONCE(abox->firmware_ready))
		handler(abox->dev, message, data);
}

static irqreturn_t exynos7885_abox_irq(int irq, void *data)
{
	struct exynos7885_abox_rproc *abox = data;
	unsigned int handled = 0;
	unsigned int i;
	u32 irqstat, irqnr;

	for (i = 0; i < ABOX_GIC_IRQ_LIMIT; i++) {
		irqstat = readl(abox->gicc + GIC_CPU_INTACK);
		irqnr = irqstat & GICC_IAR_INT_ID_MASK;
		if (irqnr >= ABOX_GIC_SPURIOUS)
			break;

		writel(irqstat, abox->gicc + GIC_CPU_EOI);
		if (irqnr < 16) {
			writel(irqstat, abox->gicc + GIC_CPU_DEACTIVATE);
			exynos7885_abox_handle_ipc(abox, irqnr);
		}
		handled++;
	}

	if (i == ABOX_GIC_IRQ_LIMIT)
		dev_err_ratelimited(abox->dev,
				    "ABOX GIC interrupt budget exhausted\n");

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable,
		 "Explicitly register the opt-in Exynos7885 ABOX remoteproc");

static bool allow_firmware_start;
module_param(allow_firmware_start, bool, 0444);
MODULE_PARM_DESC(allow_firmware_start,
		 "Allow explicit remoteproc requests to execute ABOX firmware");

static char *bootargs;
module_param(bootargs, charp, 0444);
MODULE_PARM_DESC(bootargs,
		 "Firmware boot arguments when the DT has no samsung,bootargs");

static unsigned int bootargs_offset;
module_param(bootargs_offset, uint, 0444);
MODULE_PARM_DESC(bootargs_offset,
		 "SRAM offset of the module bootargs");

static bool populate_children = true;
module_param(populate_children, bool, 0444);
MODULE_PARM_DESC(populate_children,
		 "Populate ABOX PCM child devices after remoteproc registration");

static int exynos7885_abox_check_idle(struct exynos7885_abox_rproc *abox)
{
	unsigned int dispaud, cpu, option;
	int ret;

	ret = regmap_read(abox->pmu, ABOX_DISPAUD_STATUS, &dispaud);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_OPTION, &option);
	if (ret)
		return ret;

	/*
	 * The bootloader leaves the CA7 local-power request set while the
	 * core is held and off (CONFIGURATION 0x1, STATUS 0x10010000 on the
	 * T510). Like the vendor driver, treat a held, off core as idle;
	 * start clears the power request before setting the reset vector.
	 */
	return (dispaud & 0xf) == 0xf && !(cpu & ABOX_CA7_STATUS_ON) &&
	       !(option & ABOX_CA7_ENABLE) ? 0 : -EBUSY;
}

static int exynos7885_abox_sanity_check(struct rproc *rproc,
					const struct firmware *fw)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;

	if (!fw->size || fw->size > ABOX_SRAM_SIZE)
		return -EINVAL;

	/* Called before remoteproc attaches the System MMU. */
	return exynos7885_abox_check_idle(abox);
}

/* Translation faults are logged; reporting them unhandled panics. */
static int exynos7885_abox_iommu_fault(struct iommu_domain *domain,
				       struct device *dev, unsigned long iova,
				       int flags, void *token)
{
	struct exynos7885_abox_rproc *abox = token;

	dev_err_ratelimited(abox->dev, "System MMU fault at IOVA %#lx, flags %#x\n",
			    iova, flags);
	/*
	 * A retried access faults forever. Hold the CA7 and mask the fault
	 * interrupt, leaving the access stalled instead of flooding the log.
	 */
	if (atomic_inc_return(&abox->faults) == ABOX_FAULT_LIMIT) {
		/* Like the vendor panic handler: clear enable and power. */
		regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				   ABOX_CA7_ENABLE, 0);
		regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				   ABOX_CA7_LOCAL_PWR, 0);
		disable_irq_nosync(abox->sysmmu_irq);
		WRITE_ONCE(abox->sysmmu_irq_off, true);
		dev_crit(abox->dev, "%d System MMU faults; holding the ABOX CPU\n",
			 ABOX_FAULT_LIMIT);
	}
	return 0;
}

/*
 * Hold the CA7 and drop its local-power request, as the vendor driver does.
 * A running core only reports off once local power is removed, so callers
 * must have confirmed it idle (WFI) or already off.
 */
static int exynos7885_abox_hold_cpu(struct exynos7885_abox_rproc *abox)
{
	unsigned int status;
	int ret;

	ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				 ABOX_CA7_ENABLE, 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				 ABOX_CA7_LOCAL_PWR, 0);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(abox->pmu, ABOX_CA7_STATUS, status,
					!(status & ABOX_CA7_STATUS_ON), 100,
					ABOX_STOP_TIMEOUT_US);
}

static int exynos7885_abox_prepare(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	struct iommu_domain *domain = rproc->domain;
	int ret;

	ret = exynos7885_abox_check_idle(abox);
	if (ret)
		return ret;

	/*
	 * Like the vendor driver, hold the CA7 before clocking ABOX: a core
	 * left with local power on can issue accesses through the System MMU.
	 */
	ret = exynos7885_abox_hold_cpu(abox);
	if (ret)
		return ret;

	/*
	 * The remoteproc core already set a handler that reports faults as
	 * unhandled, which makes the Exynos System MMU panic, and a domain
	 * takes only one iommu_set_fault_handler() call. Replace it while
	 * the System MMU is not yet translating.
	 */
	atomic_set(&abox->faults, 0);
	domain->handler_token = abox;
	domain->handler = exynos7885_abox_iommu_fault;

	/*
	 * Set the start rates now, so the PLL is reprogrammed while nothing
	 * uses it and .start() finds them already applied.
	 */
	ret = clk_set_rate(abox->clocks[ABOX_CLK_PLL].clk,
			   ABOX_AUD_PLL_RATE_48K);
	if (ret)
		return ret;
	ret = clk_set_rate(abox->clocks[ABOX_CLK_CA7].clk, ABOX_CA7_RATE);
	if (ret)
		return ret;

	/*
	 * The System MMU only programs the remoteproc domain while runtime
	 * active; its device link resumes it with this consumer. Without
	 * this, ABOX DMA to the firmware IOVAs would bypass translation.
	 */
	ret = pm_runtime_resume_and_get(abox->dev);
	if (ret)
		return ret;
	abox->rpm_held = true;

	/* SRAM, SFR and local-GIC accesses all stall without the ABOX ACLK. */
	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_ACLK].clk);
	if (ret)
		goto put_rpm;
	abox->aclk_held = true;

	/* SRAM and the local GIC also stall without the CA7 clock. */
	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_CA7].clk);
	if (ret)
		goto disable_aclk;
	abox->ca7_held = true;

	ret = iommu_map(domain, ABOX_PMU_IOVA, ABOX_PMU_IOVA,
			ABOX_PMU_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		goto disable_ca7;
	abox->pmu_mapped = true;

	ret = iommu_map(domain, ABOX_AUDSYS_IOVA, ABOX_AUDSYS_IOVA,
			ABOX_AUDSYS_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		iommu_unmap(domain, ABOX_PMU_IOVA, ABOX_PMU_SIZE);
		abox->pmu_mapped = false;
		goto disable_ca7;
	}
	abox->audsys_mapped = true;

	return 0;

disable_ca7:
	clk_disable_unprepare(abox->clocks[ABOX_CLK_CA7].clk);
	abox->ca7_held = false;
disable_aclk:
	clk_disable_unprepare(abox->clocks[ABOX_CLK_ACLK].clk);
	abox->aclk_held = false;
put_rpm:
	pm_runtime_put_sync(abox->dev);
	abox->rpm_held = false;
	return ret;
}

static int exynos7885_abox_unprepare(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	struct iommu_domain *domain = rproc->domain;
	unsigned int cpu;
	bool cpu_off;
	int ret;

	/* The core frees carveout entries before calling unprepare. */
	abox->dram = NULL;

	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
	cpu_off = !ret && !(cpu & ABOX_CA7_STATUS_ON);

	if (abox->sram_loaded) {
		if (!cpu_off) {
			dev_warn(rproc->dev.parent,
				 "leaving ABOX SRAM intact; CPU-off state is unconfirmed\n");
		} else {
			memset_io(abox->sram, 0, abox->sram_size);
			/* Complete SRAM writes before dropping its IOMMU context. */
			wmb();
			abox->sram_loaded = false;
		}
	}

	if (abox->audsys_mapped) {
		iommu_unmap(domain, ABOX_AUDSYS_IOVA, ABOX_AUDSYS_SIZE);
		abox->audsys_mapped = false;
	}
	if (abox->pmu_mapped) {
		iommu_unmap(domain, ABOX_PMU_IOVA, ABOX_PMU_SIZE);
		abox->pmu_mapped = false;
	}

	/* A possibly running CA7 keeps its bus clock and translation. */
	if (!cpu_off) {
		dev_crit(rproc->dev.parent,
			 "keeping ABOX clocks and System MMU active\n");
		return -EBUSY;
	}
	if (abox->sysmmu_irq_off) {
		enable_irq(abox->sysmmu_irq);
		abox->sysmmu_irq_off = false;
	}
	if (abox->ca7_held) {
		clk_disable_unprepare(abox->clocks[ABOX_CLK_CA7].clk);
		abox->ca7_held = false;
	}
	if (abox->aclk_held) {
		clk_disable_unprepare(abox->clocks[ABOX_CLK_ACLK].clk);
		abox->aclk_held = false;
	}
	if (abox->rpm_held) {
		pm_runtime_put_sync(abox->dev);
		abox->rpm_held = false;
	}

	return 0;
}

static int exynos7885_abox_alloc_dram(struct rproc *rproc,
				      struct rproc_mem_entry *mem)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	int ret;

	/*
	 * The carveout is no-map and ABOX is not cache coherent: use an
	 * uncached mapping, since there is no linear alias for DMA syncs.
	 */
	mem->va = memremap(abox->dram_rmem->base, mem->len, MEMREMAP_WC);
	if (!mem->va)
		return -ENOMEM;
	mem->dma = abox->dram_rmem->base;

	ret = iommu_map(rproc->domain, mem->da, mem->dma, mem->len,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		iommu_unmap(rproc->domain, mem->da, mem->len);
		goto free_dram;
	}

	return 0;

free_dram:
	memunmap(mem->va);
	mem->va = NULL;
	return ret;
}

static int exynos7885_abox_release_dram(struct rproc *rproc,
					struct rproc_mem_entry *mem)
{
	size_t unmapped;

	if (!mem->va)
		return 0;

	unmapped = iommu_unmap(rproc->domain, mem->da, mem->len);
	if (unmapped != mem->len)
		dev_warn(rproc->dev.parent,
			 "DRAM IOMMU unmap covered %zu of %zu bytes\n",
			 unmapped, mem->len);
	memunmap(mem->va);
	mem->va = NULL;
	return 0;
}

static int exynos7885_abox_parse_fw(struct rproc *rproc,
				    const struct firmware *fw)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;

	/* Resource cleanup frees each entry, so register one for every attempt. */
	abox->dram = rproc_mem_entry_init(rproc->dev.parent, NULL, 0,
					 ABOX_DRAM_SIZE, ABOX_DRAM_IOVA,
					 exynos7885_abox_alloc_dram,
					 exynos7885_abox_release_dram,
					 "dram");
	if (!abox->dram)
		return -ENOMEM;
	rproc_add_carveout(rproc, abox->dram);

	return 0;
}

/*
 * The firmware reads its options from SRAM; the T510 passes "vss=0", without
 * which the firmware writes to the unmapped modem voice window.
 */
static int exynos7885_abox_write_bootargs(struct exynos7885_abox_rproc *abox,
					  size_t image_size)
{
	struct device_node *np = abox->dev->of_node;
	const char *args = bootargs;
	u32 offset = bootargs_offset;
	size_t len;

	if (!of_property_read_string(np, "samsung,bootargs", &args) &&
	    of_property_read_u32(np, "samsung,bootargs-offset", &offset))
		return -EINVAL;
	if (!args)
		return 0;

	len = strlen(args) + 1;
	if (len > ABOX_BOOTARGS_SIZE || offset < image_size ||
	    offset > abox->sram_size - ABOX_BOOTARGS_SIZE)
		return -EINVAL;

	memcpy_toio(abox->sram + offset, args, len);
	dev_info(abox->dev, "firmware bootargs \"%s\" at SRAM +%#x\n",
		 args, offset);
	return 0;
}

static int exynos7885_abox_load(struct rproc *rproc,
				const struct firmware *sram)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	const struct firmware *dram;
	struct device_node *child;
	int ret;

	ret = request_firmware(&dram, "postmarketos/calliope_dram.bin",
			       rproc->dev.parent);
	if (ret)
		return ret;
	if (!dram->size || dram->size > abox->dram->len) {
		ret = -EINVAL;
		goto out;
	}
	if (!sram->size || sram->size > abox->sram_size) {
		ret = -EINVAL;
		goto out;
	}

	memset(abox->dram->va, 0, abox->dram->len);
	memcpy(abox->dram->va, dram->data, dram->size);
	for_each_available_child_of_node(rproc->dev.parent->of_node, child) {
		const struct firmware *extra;
		const char *name;
		u32 area, offset;

		ret = of_property_read_string(child, "samsung,name", &name);
		if (ret)
			continue;
		ret = of_property_read_u32(child, "samsung,area", &area);
		if (ret)
			continue;
		ret = of_property_read_u32(child, "samsung,offset", &offset);
		if (ret)
			continue;
		if (area != 1) {
			dev_err(abox->dev,
				"unsupported extra firmware area %u for %s\n",
				area, name);
			ret = -EINVAL;
			goto out_put_child;
		}

		ret = request_firmware(&extra, name, abox->dev);
		if (ret) {
			dev_err(abox->dev, "failed to load ABOX firmware %s: %d\n",
				name, ret);
			goto out_put_child;
		}
		if (offset > abox->dram->len ||
		    extra->size > abox->dram->len - offset) {
			dev_err(abox->dev,
				"ABOX firmware %s exceeds DRAM carveout\n", name);
			release_firmware(extra);
			ret = -EINVAL;
			goto out_put_child;
		}

		memcpy((u8 *)abox->dram->va + offset, extra->data, extra->size);
		dev_info(abox->dev, "staged ABOX firmware %s at DRAM +0x%x\n",
			 name, offset);
		release_firmware(extra);
	}
	/* The DRAM image is write-combined; drain it before SRAM staging. */
	wmb();
	memset_io(abox->sram, 0, abox->sram_size);
	memcpy_toio(abox->sram, sram->data, sram->size);
	ret = exynos7885_abox_write_bootargs(abox, sram->size);
	if (ret)
		goto out;
	/* Firmware must be visible in SRAM before a later start operation. */
	wmb();
	abox->sram_loaded = true;
	dev_info(&rproc->dev, "staged SRAM %zu bytes and DRAM %zu bytes; CPU remains off\n",
		 sram->size, dram->size);
	ret = 0;
	goto out;

out_put_child:
	of_node_put(child);
out:
	release_firmware(dram);
	return ret;
}

static int exynos7885_abox_start(struct rproc *rproc);

static int exynos7885_abox_send_ipc_locked(struct exynos7885_abox_rproc *abox,
					   const void *message, size_t size,
					   bool stopping)
{
	void __iomem *tx, *ack;
	u32 pending, ipc_id;
	int ret;

	memcpy(&ipc_id, message, sizeof(ipc_id));
	if (!ipc_id || ipc_id >= 16)
		return -EINVAL;
	tx = abox->sram + ABOX_IPC_TX_OFFSET;
	ack = abox->sram + ABOX_IPC_TX_ACK_OFFSET;

	if (abox->stopping && !stopping)
		return -ESHUTDOWN;
	if ((!abox->firmware_running || !abox->firmware_ready) &&
	    !(abox->stopping && stopping))
		return -EHOSTDOWN;

	ret = readl_poll_timeout(ack, pending, !pending, 10,
				 ABOX_IPC_TIMEOUT_US);
	if (ret) {
		dev_err(abox->dev,
			"AP-to-ABOX IPC is still pending; refusing another request\n");
		return ret;
	}

	memset_io(tx, 0, ABOX_IPC_MSG_SIZE);
	memcpy_toio(tx, message, size);
	writel(1, ack);
	/* Publish the message and acknowledgement before ringing the SGI. */
	wmb();
	writel(BIT(16) | ipc_id, abox->gicd + ABOX_GIC_SGIR);

	ret = readl_poll_timeout(ack, pending, !pending, 10,
				 ABOX_IPC_TIMEOUT_US);
	if (ret)
		dev_err(abox->dev, "ABOX IPC %u timed out\n", ipc_id);

	return ret;
}

int exynos7885_abox_send_ipc(struct device *dev, const void *message,
			     size_t size)
{
	struct rproc *rproc;
	struct exynos7885_abox_rproc *abox;
	int ret;

	if (!dev || !message || size < sizeof(u32) ||
	    size > ABOX_IPC_MSG_SIZE || !IS_ALIGNED(size, sizeof(u32)))
		return -EINVAL;

	rproc = dev_get_drvdata(dev);
	if (!rproc)
		return -ENODEV;

	abox = rproc->priv;
	mutex_lock(&abox->ipc_lock);
	ret = exynos7885_abox_send_ipc_locked(abox, message, size, false);
	mutex_unlock(&abox->ipc_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_send_ipc);

int exynos7885_abox_send_pcm(struct device *dev, u32 channel, u32 type,
			     u32 param0, u32 param1, u32 param2)
{
	u32 message[ABOX_IPC_WORDS] = {};

	if (channel >= 8)
		return -EINVAL;

	switch (type) {
	case EXYNOS7885_ABOX_PCM_OPEN:
	case EXYNOS7885_ABOX_PCM_CLOSE:
	case EXYNOS7885_ABOX_PCM_HW_PARAMS:
	case EXYNOS7885_ABOX_PCM_HW_FREE:
	case EXYNOS7885_ABOX_PCM_PREPARE:
	case EXYNOS7885_ABOX_PCM_TRIGGER:
	case EXYNOS7885_ABOX_PCM_SET_BUFFER:
		break;
	default:
		return -EINVAL;
	}

	message[0] = EXYNOS7885_ABOX_IPC_PCM_PLAYBACK;
	message[1] = channel;
	message[2] = type;
	message[3] = channel;
	message[4] = param0;
	message[5] = param1;
	message[6] = param2;

	return exynos7885_abox_send_ipc(dev, message, sizeof(message));
}
EXPORT_SYMBOL_GPL(exynos7885_abox_send_pcm);

static struct rproc *exynos7885_abox_get_rproc(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

int exynos7885_abox_boot(struct device *dev)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);

	return rproc ? rproc_boot(rproc) : -ENODEV;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_boot);

int exynos7885_abox_shutdown(struct device *dev)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);

	if (!rproc)
		return -ENODEV;

	return rproc_shutdown(rproc);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_shutdown);

int exynos7885_abox_get_pcm_buffer(struct device *dev, void **area,
				   phys_addr_t *phys, size_t *size)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	void *pcm_area;

	if (!rproc || !area || !phys || !size)
		return -EINVAL;

	abox = rproc->priv;
	if (!abox->pcm_rmem)
		return -ENODEV;

	mutex_lock(&abox->pcm_lock);
	if (!abox->pcm_area) {
		/* No-map ring shared with non-coherent ABOX: uncached. */
		pcm_area = devm_memremap(abox->dev, abox->pcm_rmem->base,
					 ABOX_PCM_SIZE, MEMREMAP_WC);
		if (!pcm_area) {
			mutex_unlock(&abox->pcm_lock);
			return -ENOMEM;
		}
		abox->pcm_area = pcm_area;
	}

	*area = abox->pcm_area;
	*phys = abox->pcm_rmem->base;
	*size = min_t(size_t, abox->pcm_rmem->size, ABOX_PCM_SIZE);
	mutex_unlock(&abox->pcm_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_get_pcm_buffer);

int exynos7885_abox_map_pcm_buffer(struct device *dev)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	int ret;

	if (!rproc)
		return -ENODEV;

	abox = rproc->priv;
	if (!abox->pcm_rmem)
		return -ENODEV;
	mutex_lock(&abox->pcm_lock);
	if (abox->pcm_shutting_down ||
	    !READ_ONCE(abox->firmware_ready) || !rproc->domain) {
		ret = -EHOSTDOWN;
	} else if (abox->pcm_mapped) {
		ret = -EBUSY;
	} else {
		ret = iommu_map(rproc->domain, ABOX_PCM_IOVA,
				abox->pcm_rmem->base, ABOX_PCM_SIZE,
				IOMMU_READ, GFP_KERNEL);
		if (!ret)
			abox->pcm_mapped = true;
	}
	mutex_unlock(&abox->pcm_lock);
	if (ret)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_map_pcm_buffer);

ssize_t exynos7885_abox_rdma_pointer(struct device *dev, unsigned int channel,
				     size_t buffer_bytes, size_t period_bytes)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	ssize_t offset, bytes;
	u32 status;

	if (!rproc || channel >= 8 || !period_bytes ||
	    buffer_bytes < period_bytes)
		return -EINVAL;

	abox = rproc->priv;
	status = readl(abox->sfr + ABOX_RDMA_BASE +
		       channel * ABOX_RDMA_INTERVAL + ABOX_RDMA_STATUS);
	if (!(status & ABOX_RDMA_PROGRESS))
		return 0;

	/* Same period normalisation of the read offset as the vendor driver. */
	offset = FIELD_GET(ABOX_RDMA_RBUF_OFFSET, status) << 4;
	bytes = buffer_bytes;
	while ((offset % period_bytes) && bytes >= 0) {
		bytes -= period_bytes;
		if ((bytes & offset) == offset)
			offset = bytes;
	}
	return offset + FIELD_GET(ABOX_RDMA_RBUF_CNT, status);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_rdma_pointer);

int exynos7885_abox_wait_rdma_idle(struct device *dev, unsigned int channel,
				   unsigned int timeout_us)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	void __iomem *status_reg;
	unsigned int status;

	if (!rproc || channel >= 8 || !timeout_us || timeout_us > 1000000)
		return -EINVAL;

	abox = rproc->priv;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;
	status_reg = abox->sfr + ABOX_RDMA_BASE +
		     channel * ABOX_RDMA_INTERVAL + ABOX_RDMA_STATUS;

	return readl_poll_timeout(status_reg, status,
				  !(status & ABOX_RDMA_PROGRESS), 100,
				  timeout_us);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_wait_rdma_idle);

int exynos7885_abox_unmap_pcm_buffer(struct device *dev)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	size_t unmapped;
	int ret;

	if (!rproc)
		return -ENODEV;

	abox = rproc->priv;
	mutex_lock(&abox->pcm_lock);
	if (!abox->pcm_mapped) {
		mutex_unlock(&abox->pcm_lock);
		return 0;
	}
	ret = exynos7885_abox_wait_rdma_idle(dev, 0, ABOX_STOP_TIMEOUT_US);
	if (ret) {
		mutex_unlock(&abox->pcm_lock);
		return ret;
	}
	unmapped = iommu_unmap(rproc->domain, ABOX_PCM_IOVA, ABOX_PCM_SIZE);
	if (unmapped != ABOX_PCM_SIZE) {
		mutex_unlock(&abox->pcm_lock);
		dev_err(abox->dev, "PCM IOMMU unmap covered %zu of %u bytes\n",
			unmapped, ABOX_PCM_SIZE);
		return -EIO;
	}

	abox->pcm_mapped = false;
	mutex_unlock(&abox->pcm_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_unmap_pcm_buffer);

int exynos7885_abox_sync_pcm_buffer(struct device *dev, size_t offset,
				    size_t size)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;

	if (!rproc || !size || offset > ABOX_PCM_SIZE ||
	    size > ABOX_PCM_SIZE - offset)
		return -EINVAL;

	abox = rproc->priv;
	mutex_lock(&abox->pcm_lock);
	if (!READ_ONCE(abox->firmware_ready) || !abox->pcm_mapped) {
		mutex_unlock(&abox->pcm_lock);
		return -EHOSTDOWN;
	}

	/* The ring is write-combined; order its data before later IPC. */
	wmb();
	mutex_unlock(&abox->pcm_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_sync_pcm_buffer);

/* UAIF0 (headphone codec) is optional; take all four clocks or none. */
static int exynos7885_abox_get_uaif0_clocks(struct exynos7885_abox_rproc *abox)
{
	static const char * const names[] = {
		"uaif0-div", "uaif0-mux", "uaif0-bclk", "uaif0-sync",
	};
	struct clk *clks[ARRAY_SIZE(names)];
	unsigned int i, found = 0;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		clks[i] = devm_clk_get_optional(abox->dev, names[i]);
		if (IS_ERR(clks[i]))
			return dev_err_probe(abox->dev, PTR_ERR(clks[i]),
					     "failed to get %s\n", names[i]);
		found += !!clks[i];
	}
	if (!found)
		return 0;
	if (found != ARRAY_SIZE(names))
		return dev_err_probe(abox->dev, -EINVAL,
				     "incomplete UAIF0 clocks\n");

	abox->uaif[0] = (struct exynos7885_abox_uaif) {
		.div = clks[0], .mux = clks[1], .bclk = clks[2], .sync = clks[3],
	};
	return 0;
}

/* The UAIF @n, or NULL if this board does not describe its clocks. */
static struct exynos7885_abox_uaif *
exynos7885_abox_get_uaif(struct exynos7885_abox_rproc *abox, unsigned int n)
{
	if (n >= ABOX_NUM_UAIF || !abox->uaif[n].div)
		return NULL;
	return &abox->uaif[n];
}

static bool exynos7885_abox_any_uaif_enabled(struct exynos7885_abox_rproc *abox)
{
	unsigned int n;

	for (n = 0; n < ABOX_NUM_UAIF; n++)
		if (abox->uaif[n].enabled)
			return true;
	return false;
}

int exynos7885_abox_uaif_set_fmt(struct device *dev, unsigned int n,
				 unsigned int format, bool invert_bclk,
				 bool invert_frame, bool abox_master)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	struct exynos7885_abox_uaif *uaif;
	u32 ctrl0, ctrl1;

	if (!rproc)
		return -ENODEV;
	if (format > EXYNOS7885_ABOX_UAIF3_DSP_A)
		return -EINVAL;
	if (!abox_master)
		return -EOPNOTSUPP;

	abox = rproc->priv;
	uaif = exynos7885_abox_get_uaif(abox, n);
	if (!uaif)
		return -ENODEV;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;

	mutex_lock(&abox->uaif_lock);
	if (uaif->enabled) {
		mutex_unlock(&abox->uaif_lock);
		return -EBUSY;
	}
	ctrl0 = readl(abox->sfr + ABOX_UAIF_CTRL0(n));
	ctrl1 = readl(abox->sfr + ABOX_UAIF_CTRL1(n));
	ctrl0 &= ~(ABOX_UAIF_MODE | ABOX_UAIF_SPK_ENABLE);
	if (abox_master)
		ctrl0 |= ABOX_UAIF_MODE;
	ctrl1 &= ~(ABOX_UAIF_BCLK_POLARITY | ABOX_UAIF_WS_MODE |
		   ABOX_UAIF_WS_POLARITY);
	if (!invert_bclk)
		ctrl1 |= ABOX_UAIF_BCLK_POLARITY;
	if (format == EXYNOS7885_ABOX_UAIF3_DSP_A)
		ctrl1 |= ABOX_UAIF_WS_MODE;
	if (invert_frame)
		ctrl1 |= ABOX_UAIF_WS_POLARITY;
	writel(ctrl0, abox->sfr + ABOX_UAIF_CTRL0(n));
	writel(ctrl1, abox->sfr + ABOX_UAIF_CTRL1(n));
	/* Publish frame format before a later clock enable starts the pins. */
	wmb();
	mutex_unlock(&abox->uaif_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif_set_fmt);

int exynos7885_abox_uaif3_set_fmt(struct device *dev, unsigned int format,
				  bool invert_bclk, bool invert_frame,
				  bool abox_master)
{
	return exynos7885_abox_uaif_set_fmt(dev, 3, format, invert_bclk,
					    invert_frame, abox_master);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif3_set_fmt);

static int exynos7885_abox_send_config(struct device *dev, u32 type,
				       u32 param1, u32 param2)
{
	u32 message[ABOX_IPC_WORDS] = {};

	/* IPC_ABOX_CONFIG: ipcid, task, msgtype, param1, param2. */
	message[0] = ABOX_IPC_CONFIG;
	message[2] = type;
	message[3] = param1;
	message[4] = param2;
	return exynos7885_abox_send_ipc(dev, message, sizeof(message));
}

int exynos7885_abox_uaif_hw_params(struct device *dev, unsigned int n,
				   unsigned int rate, unsigned int width,
				   unsigned int channels)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	struct exynos7885_abox_uaif *uaif;
	unsigned int slot_width, sample_code;
	unsigned long pll_rate, bclk_rate, bclk_div;
	u32 ctrl1, spus0, spus1, route0;
	int ret;

	if (!rproc || channels != 2 || (width != 16 && width != 24 && width != 32) ||
	    rate < 8000 || rate > 192000)
		return -EINVAL;

	abox = rproc->priv;
	uaif = exynos7885_abox_get_uaif(abox, n);
	if (!uaif)
		return -ENODEV;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;
	slot_width = width == 24 ? 32 : width;
	sample_code = slot_width / 8 - 1;
	bclk_rate = (unsigned long)rate * channels * slot_width;
	/* All standard 44.1 kHz-family rates are integer multiples of 11.025 kHz. */
	if (!(rate % 11025))
		pll_rate = ABOX_AUD_PLL_RATE_44K;
	else
		pll_rate = ABOX_AUD_PLL_RATE_48K;
	/* BCLK is an integer division of the nominal AUDIF rate. */
	bclk_div = pll_rate / ABOX_AUDIF_DIV / bclk_rate;
	if (!bclk_div || pll_rate / ABOX_AUDIF_DIV % bclk_rate)
		return -EINVAL;
	/* AUDIF runs a few Hz fast, so request the rounded-up quotient. */
	bclk_rate = DIV_ROUND_UP(ABOX_AUDIF_RATE(pll_rate), bclk_div);

	mutex_lock(&abox->uaif_lock);
	/* The PLL and AUDIF are shared by every UAIF. */
	if (exynos7885_abox_any_uaif_enabled(abox)) {
		ret = -EBUSY;
		goto out_unlock;
	}
	ret = clk_set_rate(abox->clocks[ABOX_CLK_PLL].clk, pll_rate);
	if (ret)
		goto out_unlock;
	ret = clk_set_rate(abox->clocks[ABOX_CLK_AUDIF].clk,
			   ABOX_AUDIF_RATE(pll_rate));
	if (ret)
		goto out_unlock;
	ret = clk_set_parent(uaif->mux, uaif->div);
	if (ret)
		goto out_unlock;
	/* Consumer handles differ from the provider's; compare the clocks. */
	if (!clk_is_match(clk_get_parent(uaif->mux), uaif->div)) {
		ret = -EIO;
		goto out_unlock;
	}
	ret = clk_set_rate(uaif->div, bclk_rate);
	if (ret)
		goto out_unlock;
	if (clk_get_rate(uaif->div) != bclk_rate ||
	    clk_get_rate(uaif->bclk) != bclk_rate) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ctrl1 = readl(abox->sfr + ABOX_UAIF_CTRL1(n));
	ctrl1 &= ~(ABOX_UAIF_FORMAT | ABOX_UAIF_SLOT_MAX |
		   ABOX_UAIF_SBIT_MAX | ABOX_UAIF_VALID_START |
		   ABOX_UAIF_VALID_END);
	ctrl1 |= FIELD_PREP(ABOX_UAIF_FORMAT,
			    1 | (sample_code << 3));
	ctrl1 |= FIELD_PREP(ABOX_UAIF_SLOT_MAX, channels - 1);
	ctrl1 |= FIELD_PREP(ABOX_UAIF_SBIT_MAX, slot_width - 1);
	writel(ctrl1, abox->sfr + ABOX_UAIF_CTRL1(n));

	/*
	 * Route RDMA0 -> SPUS OUT0 -> SIFS1 -> this UAIF, with the SPUS ASRC
	 * on. The vendor speaker path uses the same SIFS1 -> UAIF3 leg; the
	 * headphone UAIF0 is fed from the same SIFS1.
	 */
	spus0 = readl(abox->sfr + ABOX_SPUS_CTRL0);
	spus1 = readl(abox->sfr + ABOX_SPUS_CTRL1);
	route0 = readl(abox->sfr + ABOX_ROUTE_CTRL0);
	spus0 &= ~ABOX_SPUS_RDMA0_CHAIN;
	spus0 |= ABOX_SPUS_RDMA0_ASRC;
	spus1 &= ~ABOX_SPUS_SIFS1_SEL;
	route0 &= ~ABOX_ROUTE_UAIF_SPK(n);
	route0 |= ABOX_ROUTE_SIFS1 << (4 * n);
	writel(spus0, abox->sfr + ABOX_SPUS_CTRL0);
	writel(spus1, abox->sfr + ABOX_SPUS_CTRL1);
	writel(route0, abox->sfr + ABOX_ROUTE_CTRL0);
	dev_dbg(abox->dev, "UAIF%u route: SPUS %#x %#x ROUTE %#x\n", n, spus0,
		spus1, route0);
	/* Make the slot configuration visible before the PCM trigger. */
	wmb();

out_unlock:
	mutex_unlock(&abox->uaif_lock);
	if (ret)
		return ret;

	/*
	 * SIFS1 ("OUT1") paces RDMA0 at its configured rate; unconfigured,
	 * the firmware drains the ring several times faster than real time.
	 * The vendor mixer paths set it to the UAIF rate and format.
	 */
	ret = exynos7885_abox_send_config(dev, ABOX_SET_OUT1_SAMPLE_RATE,
					  rate, 0);
	if (ret)
		return ret;
	return exynos7885_abox_send_config(dev, ABOX_SET_OUT1_FORMAT, width,
					   channels);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif_hw_params);

int exynos7885_abox_uaif3_hw_params(struct device *dev, unsigned int rate,
				    unsigned int width,
				    unsigned int channels)
{
	return exynos7885_abox_uaif_hw_params(dev, 3, rate, width, channels);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif3_hw_params);

int exynos7885_abox_uaif_set_enabled(struct device *dev, unsigned int n,
				     bool enable)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	struct exynos7885_abox_uaif *uaif;
	u32 ctrl0;
	int ret;

	if (!rproc)
		return -ENODEV;
	abox = rproc->priv;
	uaif = exynos7885_abox_get_uaif(abox, n);
	if (!uaif)
		return -ENODEV;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;

	mutex_lock(&abox->uaif_lock);
	if (enable == uaif->enabled) {
		ret = 0;
		goto out_unlock;
	}
	if (enable) {
		ret = clk_prepare_enable(uaif->bclk);
		if (ret)
			goto out_unlock;
		ret = clk_prepare_enable(uaif->sync);
		if (ret) {
			clk_disable_unprepare(uaif->bclk);
			goto out_unlock;
		}
		ctrl0 = readl(abox->sfr + ABOX_UAIF_CTRL0(n));
		writel(ctrl0 | ABOX_UAIF_SPK_ENABLE,
		       abox->sfr + ABOX_UAIF_CTRL0(n));
		/* Enable the output before asking firmware to start RDMA. */
		wmb();
		uaif->enabled = true;
		ret = 0;
		goto out_unlock;
	}

	ret = exynos7885_abox_wait_rdma_idle(dev, 0, ABOX_STOP_TIMEOUT_US);
	if (ret)
		goto out_unlock;
	ctrl0 = readl(abox->sfr + ABOX_UAIF_CTRL0(n));
	writel(ctrl0 & ~ABOX_UAIF_SPK_ENABLE,
	       abox->sfr + ABOX_UAIF_CTRL0(n));
	/* Disable the output before removing its clocks. */
	wmb();
	clk_disable_unprepare(uaif->sync);
	clk_disable_unprepare(uaif->bclk);
	uaif->enabled = false;
	ret = 0;

out_unlock:
	mutex_unlock(&abox->uaif_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif_set_enabled);

int exynos7885_abox_uaif3_set_enabled(struct device *dev, bool enable)
{
	return exynos7885_abox_uaif_set_enabled(dev, 3, enable);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif3_set_enabled);

int exynos7885_abox_register_ipc_handler(struct device *dev,
					 exynos7885_abox_ipc_handler_t handler,
					 void *data)
{
	struct rproc *rproc;
	struct exynos7885_abox_rproc *abox;
	int ret = 0;

	if (!dev || !handler)
		return -EINVAL;

	rproc = dev_get_drvdata(dev);
	if (!rproc)
		return -ENODEV;

	abox = rproc->priv;
	mutex_lock(&abox->handler_lock);
	if (abox->ipc_handler) {
		ret = -EBUSY;
	} else {
		WRITE_ONCE(abox->ipc_handler_data, data);
		/* Publish the data before the IRQ can observe the handler. */
		smp_store_release(&abox->ipc_handler, handler);
	}
	mutex_unlock(&abox->handler_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_register_ipc_handler);

void exynos7885_abox_unregister_ipc_handler(struct device *dev,
					    exynos7885_abox_ipc_handler_t handler)
{
	struct rproc *rproc;
	struct exynos7885_abox_rproc *abox;

	if (!dev || !handler)
		return;

	rproc = dev_get_drvdata(dev);
	if (!rproc)
		return;

	abox = rproc->priv;
	mutex_lock(&abox->handler_lock);
	if (abox->ipc_handler == handler) {
		/* Stop new callbacks before waiting for an in-flight one. */
		smp_store_release(&abox->ipc_handler, NULL);
		if (abox->irq_requested)
			synchronize_irq(abox->irq);
		WRITE_ONCE(abox->ipc_handler_data, NULL);
	}
	mutex_unlock(&abox->handler_lock);
}
EXPORT_SYMBOL_GPL(exynos7885_abox_unregister_ipc_handler);

static int exynos7885_abox_request_suspend(struct exynos7885_abox_rproc *abox)
{
	u32 message[ABOX_IPC_MSG_SIZE / sizeof(u32)] = {};

	/* IPC_SYSTEM, task 0, ABOX_SUSPEND. */
	message[0] = ABOX_IPC_SYSTEM;
	message[2] = ABOX_SYSTEM_SUSPEND;

	return exynos7885_abox_send_ipc_locked(abox, message, sizeof(message),
					       true);
}

static int exynos7885_abox_stop_cpu(struct exynos7885_abox_rproc *abox)
{
	unsigned int status;
	int ipc_ret, ret;

	ipc_ret = exynos7885_abox_request_suspend(abox);

	ret = regmap_read_poll_timeout(abox->pmu, ABOX_CA7_STATUS, status,
				       status & ABOX_CA7_STATUS_WFI, 100,
				       ABOX_WFI_TIMEOUT_US);
	if (ret) {
		dev_err(abox->dev,
			"ABOX did not acknowledge suspend or enter WFI\n");
		return ipc_ret ?: ret;
	}
	if (ipc_ret)
		dev_warn(abox->dev,
			 "suspend IPC was not acknowledged; CA7 WFI confirms idle\n");

	/* The CA7 is quiescent; hold it and remove its local power. */
	ret = exynos7885_abox_hold_cpu(abox);
	if (ret)
		dev_err(abox->dev,
			"ABOX CA7 did not report off; preserving mappings\n");
	return ret;
}

static int exynos7885_abox_release_resources(struct exynos7885_abox_rproc *abox)
{
	unsigned int status;
	int i, ret;

	ret = exynos7885_abox_check_idle(abox);
	if (ret)
		return ret;

	/*
	 * The vendor driver reads a non-zero TIMER0_CTRL1 as firmware left
	 * resident by a suspend. The CA7 is off here and the next start
	 * reloads the firmware, so stop the timers and clear that marker.
	 */
	for (i = 0; i < ABOX_NUM_TIMERS; i++)
		writel(0, abox->sfr + ABOX_TIMER_CTRL0(i));
	writel(0, abox->sfr + ABOX_TIMER0_CTRL1);

	if (abox->dram_requested) {
		/* Release DRAM only after confirming CA7-off. */
		writel(0, abox->sfr + ABOX_SYSPOWER_CTRL);
		ret = readl_poll_timeout(abox->sfr + ABOX_SYSPOWER_STATUS, status,
					 !(status & ABOX_SYSPOWER_ON), 100,
					 ABOX_STOP_TIMEOUT_US);
		if (ret) {
			dev_err(abox->dev,
				"ABOX DRAM power request did not clear; preserving mappings\n");
			return ret;
		}
		abox->dram_requested = false;
	}

	if (abox->irq_enabled) {
		disable_irq(abox->irq);
		abox->irq_enabled = false;
	}
	mutex_lock(&abox->handler_lock);
	if (abox->irq_requested) {
		devm_free_irq(abox->dev, abox->irq, abox);
		abox->irq_requested = false;
	}
	mutex_unlock(&abox->handler_lock);
	if (abox->pins_active_selected) {
		ret = pinctrl_select_state(abox->pinctrl, abox->pins_idle);
		if (ret)
			return ret;
		abox->pins_active_selected = false;
	}
	for (i = ARRAY_SIZE(abox->clocks) - 1; i >= 0; i--) {
		if (!(abox->enabled_clocks & BIT(i)))
			continue;
		clk_disable_unprepare(abox->clocks[i].clk);
		abox->enabled_clocks &= ~BIT(i);
	}

	return 0;
}

static void exynos7885_abox_init_local_gic(struct exynos7885_abox_rproc *abox)
{
	unsigned int i;

	/* Match the downstream local-GIC setup; keep all firmware IRQs enabled. */
	writel(GICD_ENABLE, abox->gicd + GIC_DIST_CTRL);
	for (i = 0; i < 4; i++)
		writel(0, abox->gicd + GIC_DIST_IGROUP + i * sizeof(u32));
	writel(GICC_INT_PRI_THRESHOLD, abox->gicc + GIC_CPU_PRIMASK);
	for (i = 0; i < 40; i++)
		writel(0x10101010, abox->gicd + GIC_DIST_PRI + i * sizeof(u32));
	writel(3, abox->gicd + GIC_DIST_CTRL);
	writel(3, abox->gicc + GIC_CPU_CTRL);
}

static int exynos7885_abox_start(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	unsigned int option, status, configuration;
	long waited;
	int ret, start_ret, stop_ret;

	if (!allow_firmware_start)
		return -EOPNOTSUPP;

	if (!abox->sram_loaded || !abox->dram || !abox->dram->va)
		return -EINVAL;

	ret = exynos7885_abox_check_idle(abox);
	if (ret)
		return ret;

	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_PLL].clk);
	if (ret)
		return ret;
	abox->enabled_clocks |= BIT(ABOX_CLK_PLL);
	ret = clk_set_rate(abox->clocks[ABOX_CLK_PLL].clk,
			   ABOX_AUD_PLL_RATE_48K);
	if (ret)
		goto fail_before_release;
	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_ACLK].clk);
	if (ret)
		goto fail_before_release;
	abox->enabled_clocks |= BIT(ABOX_CLK_ACLK);
	if (readl(abox->sfr + ABOX_SYSPOWER_STATUS) & ABOX_SYSPOWER_ON) {
		ret = -EBUSY;
		goto fail_before_release;
	}

	/* An early-wake firmware instance needs a separate attach/resume path. */
	if (readl(abox->sfr + ABOX_TIMER0_CTRL1)) {
		ret = -EBUSY;
		goto fail_before_release;
	}

	/* Keep the CA7 held while setting its reset vector and clocks. */
	ret = exynos7885_abox_hold_cpu(abox);
	if (ret)
		goto fail_before_release;
	writel(ABOX_SRAM_PHYS, abox->sfr + ABOX_REMAP_ADDR);
	/* Ensure the reset vector is visible before clocks and CPU release. */
	wmb();
	if (readl(abox->sfr + ABOX_REMAP_ADDR) != ABOX_SRAM_PHYS) {
		ret = -EIO;
		goto fail_before_release;
	}

	ret = clk_set_rate(abox->clocks[ABOX_CLK_CA7].clk, ABOX_CA7_RATE);
	if (ret)
		goto fail_before_release;
	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_CA7].clk);
	if (ret)
		goto fail_before_release;
	abox->enabled_clocks |= BIT(ABOX_CLK_CA7);

	ret = clk_set_rate(abox->clocks[ABOX_CLK_AUDIF].clk,
			   ABOX_AUDIF_RATE(ABOX_AUD_PLL_RATE_48K));
	if (ret)
		goto fail_before_release;
	ret = clk_prepare_enable(abox->clocks[ABOX_CLK_AUDIF].clk);
	if (ret)
		goto fail_before_release;
	abox->enabled_clocks |= BIT(ABOX_CLK_AUDIF);

	exynos7885_abox_init_local_gic(abox);
	reinit_completion(&abox->boot_done);

	/* Request the parent IRQ only after the ABOX GIC is initialized. */
	mutex_lock(&abox->handler_lock);
	if (!abox->irq_requested) {
		abox->irq = platform_get_irq_byname(to_platform_device(abox->dev),
						    "abox");
		if (abox->irq < 0) {
			ret = dev_err_probe(abox->dev, abox->irq,
					    "failed to get ABOX parent IRQ\n");
			mutex_unlock(&abox->handler_lock);
			goto fail_before_release;
		}

		ret = devm_request_irq(abox->dev, abox->irq,
				       exynos7885_abox_irq, IRQF_NO_AUTOEN,
				       dev_name(abox->dev), abox);
		if (ret) {
			ret = dev_err_probe(abox->dev, ret,
					    "failed to request ABOX parent IRQ\n");
			mutex_unlock(&abox->handler_lock);
			goto fail_before_release;
		}
		abox->irq_requested = true;
	}
	mutex_unlock(&abox->handler_lock);

	/* The consumer state is named "active" to avoid pinctrl auto-selection. */
	abox->pins_active_selected = true;
	ret = pinctrl_select_state(abox->pinctrl, abox->pins_active);
	if (ret)
		goto fail_before_release;

	if (!abox->irq_enabled) {
		enable_irq(abox->irq);
		abox->irq_enabled = true;
	}

	dev_dbg(abox->dev, "ABOX clocks and GIC ready; releasing CPU\n");
	abox->dram_requested = true;
	writel(ABOX_SYSPOWER_ON, abox->sfr + ABOX_SYSPOWER_CTRL);
	/* Publish the DRAM power request before polling its status. */
	wmb();
	ret = readl_poll_timeout(abox->sfr + ABOX_SYSPOWER_STATUS, status,
				 status & ABOX_SYSPOWER_ON, 100,
				 ABOX_STOP_TIMEOUT_US);
	if (ret)
		goto fail_before_release;

	ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				 ABOX_CA7_LOCAL_PWR, ABOX_CA7_LOCAL_PWR);
	if (ret)
		goto fail_before_release;

	mutex_lock(&abox->ipc_lock);
	abox->firmware_running = true;
	abox->firmware_ready = false;
	abox->stopping = false;

	/* An uncertain PMU write is handled as post-release from this point. */
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				 ABOX_CA7_ENABLE, ABOX_CA7_ENABLE);
	if (ret)
		goto fail_after_release;

	ret = regmap_read_poll_timeout(abox->pmu, ABOX_CA7_STATUS, status,
				       status & ABOX_CA7_STATUS_ON, 100,
				       ABOX_STOP_TIMEOUT_US);
	if (ret)
		goto fail_after_release;

	waited = wait_for_completion_timeout(&abox->boot_done,
					     msecs_to_jiffies(ABOX_BOOT_TIMEOUT_MS));
	if (!waited) {
		ret = -ETIMEDOUT;
		goto fail_after_release;
	}

	WRITE_ONCE(abox->firmware_ready, true);
	mutex_unlock(&abox->ipc_lock);
	mutex_lock(&abox->mclk_lock);
	abox->mclk_allowed = true;
	mutex_unlock(&abox->mclk_lock);
	dev_info(abox->dev, "Calliope firmware %u booted\n",
		 READ_ONCE(abox->firmware_version));
	return 0;

fail_after_release:
	start_ret = ret;
	abox->stopping = true;
	stop_ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &status);
	if (stop_ret || (status & ABOX_CA7_STATUS_ON)) {
		stop_ret = exynos7885_abox_stop_cpu(abox);
	} else {
		stop_ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
					      ABOX_CA7_ENABLE, 0);
		if (!stop_ret)
			stop_ret = regmap_update_bits(abox->pmu,
						      ABOX_CA7_CONFIGURATION,
						      ABOX_CA7_LOCAL_PWR, 0);
	}
	if (!stop_ret) {
		stop_ret = exynos7885_abox_release_resources(abox);
		if (!stop_ret) {
			abox->firmware_running = false;
			abox->firmware_ready = false;
			abox->stopping = false;
			mutex_unlock(&abox->ipc_lock);
			return start_ret;
		}
	}

	/* Keep remoteproc mappings attached if the CA7 stop cannot be verified. */
	dev_crit(abox->dev,
		 "ABOX startup unconfirmed and stop failed (%d); retaining resources\n",
		 stop_ret);
	abox->firmware_running = true;
	abox->firmware_ready = false;
	abox->stopping = false;
	mutex_unlock(&abox->ipc_lock);
	return 0;

fail_before_release:
	start_ret = ret;
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				 ABOX_CA7_ENABLE, 0);
	if (ret)
		goto fail_before_release_uncertain;
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				 ABOX_CA7_LOCAL_PWR, 0);
	if (ret)
		goto fail_before_release_uncertain;
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &status);
	if (ret)
		goto fail_before_release_uncertain;
	if (status & ABOX_CA7_STATUS_ON) {
		ret = -EBUSY;
		goto fail_before_release_uncertain;
	}
	ret = regmap_read(abox->pmu, ABOX_CA7_OPTION, &option);
	if (ret)
		goto fail_before_release_uncertain;
	if (option & ABOX_CA7_ENABLE) {
		ret = -EBUSY;
		goto fail_before_release_uncertain;
	}
	ret = regmap_read(abox->pmu, ABOX_CA7_CONFIGURATION, &configuration);
	if (ret)
		goto fail_before_release_uncertain;
	if (configuration & ABOX_CA7_LOCAL_PWR) {
		ret = -EBUSY;
		goto fail_before_release_uncertain;
	}
	ret = exynos7885_abox_release_resources(abox);
	if (ret) {
		dev_crit(abox->dev,
			 "pre-start cleanup failed (%d); retaining remoteproc resources\n",
			 ret);
		goto fail_before_release_retain;
	}
	return start_ret;

fail_before_release_uncertain:
	dev_crit(abox->dev,
		 "ABOX pre-start cleanup is unconfirmed (%d); retaining resources\n",
		 ret);
fail_before_release_retain:
	mutex_lock(&abox->ipc_lock);
	abox->firmware_running = true;
	abox->firmware_ready = false;
	mutex_unlock(&abox->ipc_lock);
	return 0;
}

/* Turn the codec MCLK off and refuse it until the next firmware boot. */
static void exynos7885_abox_mclk_block(struct exynos7885_abox_rproc *abox)
{
	mutex_lock(&abox->mclk_lock);
	if (abox->mclk_on) {
		dev_warn(abox->dev, "stopping with the codec MCLK still on\n");
		writel(0, abox->sfr + ABOX_UAIF_CTRL0(2));
		abox->mclk_on = false;
	}
	abox->mclk_allowed = false;
	mutex_unlock(&abox->mclk_lock);
}

static int exynos7885_abox_stop(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	unsigned int status;
	int ret;

	mutex_lock(&abox->pcm_lock);
	if (abox->pcm_mapped || abox->pcm_shutting_down) {
		mutex_unlock(&abox->pcm_lock);
		return -EBUSY;
	}
	abox->pcm_shutting_down = true;
	mutex_unlock(&abox->pcm_lock);

	exynos7885_abox_mclk_block(abox);

	mutex_lock(&abox->ipc_lock);
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &status);
	if (ret)
		goto out_unlock;
	if (status & ABOX_CA7_STATUS_ON) {
		if (!abox->firmware_running && !abox->stopping) {
			dev_err(rproc->dev.parent,
				"ABOX CPU is on without a tracked firmware start\n");
			ret = -EBUSY;
			goto out_unlock;
		}

		abox->stopping = true;
		ret = exynos7885_abox_stop_cpu(abox);
		if (ret)
			goto out_unlock;
	} else if (abox->stopping || abox->firmware_running) {
		ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
					 ABOX_CA7_ENABLE, 0);
		if (ret)
			goto out_unlock;
		ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
					 ABOX_CA7_LOCAL_PWR, 0);
		if (ret)
			goto out_unlock;
	}
	ret = exynos7885_abox_release_resources(abox);
	if (ret)
		goto out_unlock;
	abox->firmware_running = false;
	abox->firmware_ready = false;
	abox->stopping = false;

	ret = 0;
out_unlock:
	mutex_unlock(&abox->ipc_lock);
	mutex_lock(&abox->pcm_lock);
	abox->pcm_shutting_down = false;
	mutex_unlock(&abox->pcm_lock);
	return ret;
}

static const struct rproc_ops exynos7885_abox_ops = {
	.prepare = exynos7885_abox_prepare,
	.unprepare = exynos7885_abox_unprepare,
	.start = exynos7885_abox_start,
	.stop = exynos7885_abox_stop,
	.sanity_check = exynos7885_abox_sanity_check,
	.parse_fw = exynos7885_abox_parse_fw,
	.load = exynos7885_abox_load,
};

static void exynos7885_abox_put_sysmmu_dev(void *data)
{
	put_device(data);
}

/*
 * The board routes the UAIF2 clock pin to the headphone codec as its master
 * clock. As in the vendor abox_enable_mclk(), running UAIF2 in microphone
 * mode drives that pin from the AUDIF divider. The SFR is only reachable
 * while the firmware is up, so the clock cannot be prepared before a boot.
 */
static int exynos7885_abox_mclk_prepare(struct clk_hw *hw)
{
	struct exynos7885_abox_rproc *abox =
		container_of(hw, struct exynos7885_abox_rproc, mclk_hw);
	int ret = 0;

	mutex_lock(&abox->mclk_lock);
	if (!abox->mclk_allowed) {
		ret = -EHOSTDOWN;
	} else if (!abox->mclk_on) {
		writel(ABOX_UAIF_MIC_ENABLE, abox->sfr + ABOX_UAIF_CTRL0(2));
		abox->mclk_on = true;
	}
	mutex_unlock(&abox->mclk_lock);
	return ret;
}

static void exynos7885_abox_mclk_unprepare(struct clk_hw *hw)
{
	struct exynos7885_abox_rproc *abox =
		container_of(hw, struct exynos7885_abox_rproc, mclk_hw);

	mutex_lock(&abox->mclk_lock);
	if (abox->mclk_on && abox->mclk_allowed)
		writel(0, abox->sfr + ABOX_UAIF_CTRL0(2));
	abox->mclk_on = false;
	mutex_unlock(&abox->mclk_lock);
}

static int exynos7885_abox_mclk_is_prepared(struct clk_hw *hw)
{
	struct exynos7885_abox_rproc *abox =
		container_of(hw, struct exynos7885_abox_rproc, mclk_hw);

	return abox->mclk_on;
}

static const struct clk_ops exynos7885_abox_mclk_ops = {
	.prepare = exynos7885_abox_mclk_prepare,
	.unprepare = exynos7885_abox_mclk_unprepare,
	.is_prepared = exynos7885_abox_mclk_is_prepared,
};

static int exynos7885_abox_register_mclk(struct exynos7885_abox_rproc *abox)
{
	struct device *dev = abox->dev;
	struct clk_init_data init = {
		.ops = &exynos7885_abox_mclk_ops,
		/* Never touch the SFR from clk_disable_unused(). */
		.flags = CLK_IGNORE_UNUSED,
	};
	int ret;

	if (!of_property_present(dev->of_node, "#clock-cells"))
		return 0;

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-mclk", dev_name(dev));
	if (!init.name)
		return -ENOMEM;
	abox->mclk_hw.init = &init;
	ret = devm_clk_hw_register(dev, &abox->mclk_hw);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					   &abox->mclk_hw);
}

static int exynos7885_abox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7885_abox_rproc *abox;
	struct platform_device *sysmmu_pdev;
	struct resource *sfr, *sysreg, *sram;
	struct resource *gicd, *gicc;
	struct device_node *mem_np, *pcm_np, *iommu_np;
	struct rproc *rproc;
	unsigned int i;
	int ret;

	if (!enable)
		return -ENODEV;

	dev_info(dev, "ABOX probe: mapping controller resources\n");

	sram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!sram || resource_size(sram) < ABOX_SRAM_SIZE)
		return -EINVAL;

	sfr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfr");
	if (!sfr)
		return -EINVAL;

	sysreg = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	if (!sysreg)
		return -EINVAL;

	rproc = devm_rproc_alloc(dev, "exynos7885-abox",
				 &exynos7885_abox_ops,
				 "postmarketos/calliope_sram.bin",
				 sizeof(*abox));
	if (!rproc)
		return -ENOMEM;

	abox = rproc->priv;
	abox->dev = dev;
	abox->sram_size = resource_size(sram);
	abox->sfr = devm_ioremap_resource(dev, sfr);
	if (IS_ERR(abox->sfr))
		return PTR_ERR(abox->sfr);

	abox->sysreg = devm_ioremap_resource(dev, sysreg);
	if (IS_ERR(abox->sysreg))
		return PTR_ERR(abox->sysreg);

	abox->sram = devm_ioremap_resource(dev, sram);
	if (IS_ERR(abox->sram))
		return PTR_ERR(abox->sram);

	gicd = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gicd");
	if (!gicd)
		return -EINVAL;
	abox->gicd = devm_ioremap_resource(dev, gicd);
	if (IS_ERR(abox->gicd))
		return PTR_ERR(abox->gicd);

	gicc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gicc");
	if (!gicc)
		return -EINVAL;
	abox->gicc = devm_ioremap_resource(dev, gicc);
	if (IS_ERR(abox->gicc))
		return PTR_ERR(abox->gicc);
	dev_info(dev, "ABOX probe: controller resources mapped\n");

	init_completion(&abox->boot_done);
	mutex_init(&abox->ipc_lock);
	mutex_init(&abox->handler_lock);
	mutex_init(&abox->pcm_lock);
	mutex_init(&abox->uaif_lock);
	mutex_init(&abox->mclk_lock);
	abox->pmu = syscon_regmap_lookup_by_compatible("samsung,exynos7885-pmu");
	if (IS_ERR(abox->pmu))
		return PTR_ERR(abox->pmu);

	mem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_np)
		return dev_err_probe(dev, -EINVAL,
				     "missing DRAM firmware memory-region\n");
	abox->dram_rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!abox->dram_rmem || abox->dram_rmem->size < ABOX_DRAM_SIZE ||
	    !IS_ALIGNED(abox->dram_rmem->base, SZ_1M))
		return dev_err_probe(dev, -EINVAL,
				     "DRAM firmware region must be at least 12 MiB and 1 MiB aligned\n");
	dev_info(dev, "ABOX probe: firmware memory region ready\n");

	pcm_np = of_parse_phandle(dev->of_node, "memory-region", 1);
	if (pcm_np) {
		abox->pcm_rmem = of_reserved_mem_lookup(pcm_np);
		of_node_put(pcm_np);
		if (!abox->pcm_rmem ||
		    abox->pcm_rmem->size < ABOX_PCM_SIZE ||
		    !IS_ALIGNED(abox->pcm_rmem->base, PAGE_SIZE))
			return dev_err_probe(dev, -EINVAL,
					     "PCM ring region must be at least 128 KiB and page aligned\n");
	}

	iommu_np = of_parse_phandle(dev->of_node, "iommus", 0);
	if (!iommu_np)
		return dev_err_probe(dev, -EINVAL,
				     "missing ABOX System MMU phandle\n");
	sysmmu_pdev = of_find_device_by_node(iommu_np);
	of_node_put(iommu_np);
	if (!sysmmu_pdev)
		return -EPROBE_DEFER;
	if (!platform_get_drvdata(sysmmu_pdev)) {
		put_device(&sysmmu_pdev->dev);
		return -EPROBE_DEFER;
	}
	abox->sysmmu_dev = &sysmmu_pdev->dev;
	ret = devm_add_action_or_reset(dev, exynos7885_abox_put_sysmmu_dev,
				      abox->sysmmu_dev);
	if (ret)
		return ret;
	abox->sysmmu_irq = platform_get_irq(sysmmu_pdev, 0);
	if (abox->sysmmu_irq < 0)
		return abox->sysmmu_irq;
	dev_info(dev, "ABOX probe: System MMU provider ready\n");

	for (i = 0; i < ARRAY_SIZE(abox->clocks); i++)
		abox->clocks[i].id = exynos7885_abox_clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(abox->clocks), abox->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get ABOX clocks\n");
	abox->uaif[3] = (struct exynos7885_abox_uaif) {
		.div = abox->clocks[ABOX_CLK_UAIF3_DIV].clk,
		.mux = abox->clocks[ABOX_CLK_UAIF3_MUX].clk,
		.bclk = abox->clocks[ABOX_CLK_UAIF3_BCLK].clk,
		.sync = abox->clocks[ABOX_CLK_UAIF3_SYNC].clk,
	};
	ret = exynos7885_abox_get_uaif0_clocks(abox);
	if (ret)
		return ret;
	dev_info(dev, "ABOX probe: clocks resolved\n");

	abox->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(abox->pinctrl))
		return dev_err_probe(dev, PTR_ERR(abox->pinctrl),
				     "failed to get ABOX pinctrl\n");

	abox->pins_active = pinctrl_lookup_state(abox->pinctrl, "active");
	if (IS_ERR(abox->pins_active))
		return dev_err_probe(dev, PTR_ERR(abox->pins_active),
				     "failed to find active pin state\n");

	abox->pins_idle = pinctrl_lookup_state(abox->pinctrl, "idle");
	if (IS_ERR(abox->pins_idle))
		return dev_err_probe(dev, PTR_ERR(abox->pins_idle),
				     "failed to find idle pin state\n");
	dev_info(dev, "ABOX probe: pin states resolved\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	rproc->has_iommu = true;
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	platform_set_drvdata(pdev, rproc);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;
	dev_info(dev, "ABOX probe: remoteproc registered offline\n");

	ret = exynos7885_abox_register_mclk(abox);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register codec MCLK\n");

	if (populate_children) {
		dev_info(dev, "ABOX probe: populating child devices\n");
		ret = devm_of_platform_populate(dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to populate ABOX child devices\n");
		dev_info(dev, "ABOX probe: PCM child devices populated\n");
	} else {
		dev_info(dev, "ABOX probe: child population disabled by parameter\n");
	}

	dev_info(dev, "ABOX remoteproc registered; firmware start is opt-in\n");
	return 0;
}

static const struct of_device_id exynos7885_abox_of_match[] = {
	{ .compatible = "samsung,exynos7885-abox" },
	{ }
};
/* No module alias: load explicitly with enable=1. */

static struct platform_driver exynos7885_abox_driver = {
	.probe = exynos7885_abox_probe,
	.driver = {
		.name = "exynos7885-abox-rproc",
		.of_match_table = exynos7885_abox_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(exynos7885_abox_driver);

MODULE_DESCRIPTION("Explicit-load Exynos7885 ABOX remoteproc");
MODULE_LICENSE("GPL");
