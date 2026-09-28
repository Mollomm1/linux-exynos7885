// SPDX-License-Identifier: GPL-2.0-only
/* Explicit-load, non-booting Exynos7885 ABOX remoteproc staging driver. */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
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
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/sizes.h>
#include <linux/soc/samsung/exynos7885-abox.h>
#include <linux/vmalloc.h>

#define ABOX_SRAM_SIZE		0x28000
#define ABOX_DRAM_SIZE		(SZ_8M + SZ_4M)
#define ABOX_DRAM_IOVA		0x80000000
#define ABOX_PMU_IOVA		0x11c80000
#define ABOX_PMU_SIZE		SZ_64K
#define ABOX_AUDSYS_IOVA	0x12090000
#define ABOX_AUDSYS_SIZE	PAGE_SIZE
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
#define ABOX_GIC_SGIR		0x0f00
#define ABOX_IPC_SYSTEM		1
#define ABOX_SYSTEM_SUSPEND	1
#define ABOX_BOOT_DONE		3
#define ABOX_IPC_TIMEOUT_US	20000
#define ABOX_WFI_TIMEOUT_US	100000
#define ABOX_STOP_TIMEOUT_US	20000
#define ABOX_GIC_IRQ_LIMIT	32
#define ABOX_GIC_SPURIOUS	1021
#define ABOX_NUM_CLOCKS		6

static const char * const exynos7885_abox_clock_names[ABOX_NUM_CLOCKS] = {
	"pll", "ca7", "audif", "aclk", "uaif3-bclk", "uaif3-sync",
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
	struct rproc_mem_entry *dram;
	void __iomem *sram;
	void __iomem *gicd;
	void __iomem *gicc;
	struct completion boot_done;
	struct mutex ipc_lock;
	int irq;
	u32 firmware_version;
	size_t sram_size;
	bool pmu_mapped;
	bool audsys_mapped;
	bool sram_loaded;
	bool irq_enabled;
	bool clocks_enabled;
	bool pins_active_selected;
	bool firmware_running;
	bool stopping;
};

static bool exynos7885_abox_is_dma_irq(unsigned int irq)
{
	return irq == 8 || irq == 9 || (irq >= 11 && irq <= 14);
}

static void exynos7885_abox_handle_ipc(struct exynos7885_abox_rproc *abox, unsigned int irq)
{
	u32 ipc_id, msg_type, firmware_version = 0;

	if (exynos7885_abox_is_dma_irq(irq))
		return;

	ipc_id = readl(abox->sram + ABOX_IPC_RX_OFFSET);
	msg_type = readl(abox->sram + ABOX_IPC_RX_OFFSET + sizeof(u32) * 2);
	if (ipc_id == ABOX_IPC_SYSTEM && msg_type == ABOX_BOOT_DONE)
		firmware_version =
			readl(abox->sram + ABOX_IPC_RX_OFFSET + sizeof(u32) * 5);

	/* Acknowledge non-DMA messages after reading the shared payload. */
	writel(0, abox->sram + ABOX_IPC_RX_ACK_OFFSET);

	if (ipc_id == ABOX_IPC_SYSTEM && msg_type == ABOX_BOOT_DONE) {
		WRITE_ONCE(abox->firmware_version, firmware_version);
		complete(&abox->boot_done);
	}
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
MODULE_PARM_DESC(enable, "Explicitly enable non-booting ABOX remoteproc staging");

static int exynos7885_abox_check_idle(struct exynos7885_abox_rproc *abox)
{
	unsigned int dispaud, cpu;
	int ret;

	ret = regmap_read(abox->pmu, ABOX_DISPAUD_STATUS, &dispaud);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
	if (ret)
		return ret;

	return (dispaud & 0xf) == 0xf && !(cpu & 1) ? 0 : -EBUSY;
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

static int exynos7885_abox_prepare(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	struct iommu_domain *domain = rproc->domain;
	int ret;

	ret = exynos7885_abox_check_idle(abox);
	if (ret)
		return ret;

	ret = iommu_map(domain, ABOX_PMU_IOVA, ABOX_PMU_IOVA,
			ABOX_PMU_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		return ret;
	abox->pmu_mapped = true;

	ret = iommu_map(domain, ABOX_AUDSYS_IOVA, ABOX_AUDSYS_IOVA,
			ABOX_AUDSYS_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		iommu_unmap(domain, ABOX_PMU_IOVA, ABOX_PMU_SIZE);
		abox->pmu_mapped = false;
		return ret;
	}
	abox->audsys_mapped = true;

	return 0;
}

static int exynos7885_abox_unprepare(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	struct iommu_domain *domain = rproc->domain;
	unsigned int cpu;
	int ret;

	/* The core frees carveout entries before calling unprepare. */
	abox->dram = NULL;

	if (abox->sram_loaded) {
		ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
		if (ret || (cpu & 1)) {
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

	return 0;
}

static int exynos7885_abox_alloc_dram(struct rproc *rproc,
				      struct rproc_mem_entry *mem)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	int ret;

	mem->va = memremap(abox->dram_rmem->base, mem->len, MEMREMAP_WB);
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

static int exynos7885_abox_load(struct rproc *rproc,
				const struct firmware *sram)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	const struct firmware *dram;
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
	/* Match Exynos SysMMU page-table cache maintenance: sync physical RAM. */
	dma_sync_single_for_device(abox->sysmmu_dev, abox->dram_rmem->base,
				   abox->dram->len, DMA_TO_DEVICE);
	memset_io(abox->sram, 0, abox->sram_size);
	memcpy_toio(abox->sram, sram->data, sram->size);
	/* Firmware must be visible in SRAM before a later start operation. */
	wmb();
	abox->sram_loaded = true;
	dev_info(&rproc->dev, "staged SRAM %zu bytes and DRAM %zu bytes; CPU remains off\n",
		 sram->size, dram->size);
out:
	release_firmware(dram);
	return ret;
}

static int exynos7885_abox_start(struct rproc *rproc)
{
	/*
	 * The remoteproc core does not call .stop() when .start() fails; it
	 * unprepares the device and releases its IOMMU mappings. Keep execution
	 * disabled until every failure after CA7 release has a verified rollback.
	 */
	return -EOPNOTSUPP;
}

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
	if (!abox->firmware_running && !(abox->stopping && stopping))
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

	/* Hold the quiescent CA7 before removing its local power. */
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				 ABOX_CA7_ENABLE, 0);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(abox->pmu, ABOX_CA7_STATUS, status,
				       !(status & ABOX_CA7_STATUS_ON), 100,
				       ABOX_STOP_TIMEOUT_US);
	if (ret) {
		dev_err(abox->dev,
			"ABOX CA7 did not report off; preserving power and mappings\n");
		return ret;
	}

	ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				 ABOX_CA7_LOCAL_PWR, 0);
	if (ret)
		return ret;

	return 0;
}

static int exynos7885_abox_stop(struct rproc *rproc)
{
	struct exynos7885_abox_rproc *abox = rproc->priv;
	unsigned int cpu;
	int ret;

	mutex_lock(&abox->ipc_lock);
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
	if (ret)
		goto out_unlock;
	if (cpu & ABOX_CA7_STATUS_ON) {
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
		ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
					 ABOX_CA7_LOCAL_PWR, 0);
		if (ret)
			goto out_unlock;
	}
	ret = exynos7885_abox_check_idle(abox);
	if (ret) {
		dev_err(rproc->dev.parent,
			"ABOX idle state is unconfirmed; preserving mappings\n");
		goto out_unlock;
	}

	if (abox->irq_enabled) {
		disable_irq(abox->irq);
		abox->irq_enabled = false;
	}
	if (abox->pins_active_selected) {
		ret = pinctrl_select_state(abox->pinctrl, abox->pins_idle);
		if (ret)
			goto out_unlock;
		abox->pins_active_selected = false;
	}
	if (abox->clocks_enabled) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(abox->clocks), abox->clocks);
		abox->clocks_enabled = false;
	}
	abox->firmware_running = false;
	abox->stopping = false;

	ret = 0;
out_unlock:
	mutex_unlock(&abox->ipc_lock);
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

static int exynos7885_abox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7885_abox_rproc *abox;
	struct platform_device *sysmmu_pdev;
	struct resource *sfr, *sysreg, *sram;
	struct resource *gicd, *gicc;
	struct device_node *mem_np, *iommu_np;
	struct rproc *rproc;
	unsigned int i;
	int ret;

	if (!enable)
		return -ENODEV;

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

	abox->irq = platform_get_irq_byname(pdev, "abox");
	if (abox->irq < 0)
		return dev_err_probe(dev, abox->irq,
				     "failed to get ABOX parent IRQ\n");

	init_completion(&abox->boot_done);
	mutex_init(&abox->ipc_lock);
	/* Keep the parent IRQ off until a future start path initializes the GIC. */
	ret = devm_request_irq(dev, abox->irq, exynos7885_abox_irq,
			       IRQF_NO_AUTOEN, dev_name(dev), abox);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request disabled ABOX parent IRQ\n");
	abox->irq_enabled = false;

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

	for (i = 0; i < ARRAY_SIZE(abox->clocks); i++)
		abox->clocks[i].id = exynos7885_abox_clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(abox->clocks), abox->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get ABOX clocks\n");

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

	rproc->has_iommu = true;
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	platform_set_drvdata(pdev, rproc);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

	dev_info(dev, "inert ABOX remoteproc registered; start is disabled\n");
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

MODULE_DESCRIPTION("Non-booting Exynos7885 ABOX remoteproc staging");
MODULE_LICENSE("GPL");
