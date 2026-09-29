// SPDX-License-Identifier: GPL-2.0-only
/* Explicit-load Exynos7885 ABOX remoteproc driver. */

#include <linux/bitops.h>
#include <linux/bitfield.h>
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
#define ABOX_UAIF3_CTRL0	0x0530
#define ABOX_UAIF3_CTRL1	0x0534
#define ABOX_UAIF3_SPK_ENABLE	BIT(0)
#define ABOX_UAIF3_MODE		BIT(2)
#define ABOX_UAIF3_FORMAT	GENMASK(28, 24)
#define ABOX_UAIF3_BCLK_POLARITY	BIT(23)
#define ABOX_UAIF3_WS_MODE	BIT(22)
#define ABOX_UAIF3_WS_POLARITY	BIT(21)
#define ABOX_UAIF3_SLOT_MAX	GENMASK(20, 18)
#define ABOX_UAIF3_SBIT_MAX	GENMASK(17, 12)
#define ABOX_UAIF3_VALID_START	GENMASK(11, 6)
#define ABOX_UAIF3_VALID_END	GENMASK(5, 0)
#define ABOX_PMU_IOVA		0x11c80000
#define ABOX_PMU_SIZE		SZ_64K
#define ABOX_AUDSYS_IOVA	0x12090000
#define ABOX_AUDSYS_SIZE	PAGE_SIZE
#define ABOX_SYSPOWER_CTRL	0x0010
#define ABOX_SYSPOWER_STATUS	0x0014
#define ABOX_SYSPOWER_ON	BIT(0)
#define ABOX_REMAP_ADDR		0x0028
#define ABOX_TIMER0_CTRL1	0x0604
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
#define ABOX_SYSTEM_SUSPEND	1
#define ABOX_BOOT_DONE		3
#define ABOX_IPC_TIMEOUT_US	20000
#define ABOX_WFI_TIMEOUT_US	100000
#define ABOX_STOP_TIMEOUT_US	20000
#define ABOX_BOOT_TIMEOUT_MS	10000
#define ABOX_AUD_PLL_RATE_48K	1179648040
#define ABOX_AUD_PLL_RATE_44K	1083801605
#define ABOX_AUDIF_RATE		24576000
#define ABOX_AUDIF_RATE_44K	22579200
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
	bool uaif3_enabled;
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

	if (exynos7885_abox_is_dma_irq(irq))
		return;

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

static bool populate_children = true;
module_param(populate_children, bool, 0444);
MODULE_PARM_DESC(populate_children,
		 "Populate ABOX PCM child devices after remoteproc registration");

static int exynos7885_abox_check_idle(struct exynos7885_abox_rproc *abox)
{
	unsigned int dispaud, cpu, configuration, option;
	int ret;

	ret = regmap_read(abox->pmu, ABOX_DISPAUD_STATUS, &dispaud);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_STATUS, &cpu);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_CONFIGURATION, &configuration);
	if (ret)
		return ret;
	ret = regmap_read(abox->pmu, ABOX_CA7_OPTION, &option);
	if (ret)
		return ret;

	return (dispaud & 0xf) == 0xf && !(cpu & ABOX_CA7_STATUS_ON) &&
	       !(configuration & ABOX_CA7_LOCAL_PWR) &&
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
		pcm_area = devm_memremap(abox->dev, abox->pcm_rmem->base,
					 ABOX_PCM_SIZE, MEMREMAP_WB);
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
		if (ret)
			iommu_unmap(rproc->domain, ABOX_PCM_IOVA,
				    ABOX_PCM_SIZE);
		else
			abox->pcm_mapped = true;
	}
	mutex_unlock(&abox->pcm_lock);
	if (ret)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_map_pcm_buffer);

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

	dma_sync_single_range_for_device(abox->sysmmu_dev,
					 abox->pcm_rmem->base, offset,
					 size, DMA_TO_DEVICE);
	mutex_unlock(&abox->pcm_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_sync_pcm_buffer);

int exynos7885_abox_uaif3_set_fmt(struct device *dev, unsigned int format,
				  bool invert_bclk, bool invert_frame,
				  bool abox_master)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	u32 ctrl0, ctrl1;

	if (!rproc)
		return -ENODEV;
	if (format > EXYNOS7885_ABOX_UAIF3_DSP_A)
		return -EINVAL;
	if (!abox_master)
		return -EOPNOTSUPP;

	abox = rproc->priv;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;

	mutex_lock(&abox->uaif_lock);
	if (abox->uaif3_enabled) {
		mutex_unlock(&abox->uaif_lock);
		return -EBUSY;
	}
	ctrl0 = readl(abox->sfr + ABOX_UAIF3_CTRL0);
	ctrl1 = readl(abox->sfr + ABOX_UAIF3_CTRL1);
	ctrl0 &= ~(ABOX_UAIF3_MODE | ABOX_UAIF3_SPK_ENABLE);
	if (abox_master)
		ctrl0 |= ABOX_UAIF3_MODE;
	ctrl1 &= ~(ABOX_UAIF3_BCLK_POLARITY | ABOX_UAIF3_WS_MODE |
		   ABOX_UAIF3_WS_POLARITY);
	if (!invert_bclk)
		ctrl1 |= ABOX_UAIF3_BCLK_POLARITY;
	if (format == EXYNOS7885_ABOX_UAIF3_DSP_A)
		ctrl1 |= ABOX_UAIF3_WS_MODE;
	if (invert_frame)
		ctrl1 |= ABOX_UAIF3_WS_POLARITY;
	writel(ctrl0, abox->sfr + ABOX_UAIF3_CTRL0);
	writel(ctrl1, abox->sfr + ABOX_UAIF3_CTRL1);
	/* Publish frame format before a later clock enable starts the pins. */
	wmb();
	mutex_unlock(&abox->uaif_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif3_set_fmt);

int exynos7885_abox_uaif3_hw_params(struct device *dev, unsigned int rate,
				    unsigned int width,
				    unsigned int channels)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	unsigned int slot_width, sample_code;
	unsigned long pll_rate, audif_rate, bclk_rate;
	u32 ctrl1;
	int ret;

	if (!rproc || channels != 2 || (width != 16 && width != 24 && width != 32) ||
	    rate < 8000 || rate > 192000)
		return -EINVAL;

	abox = rproc->priv;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;
	slot_width = width == 24 ? 32 : width;
	sample_code = slot_width / 8 - 1;
	bclk_rate = (unsigned long)rate * channels * slot_width;
	/* All standard 44.1 kHz-family rates are integer multiples of 11.025 kHz. */
	if (!(rate % 11025)) {
		pll_rate = ABOX_AUD_PLL_RATE_44K;
		audif_rate = ABOX_AUDIF_RATE_44K;
	} else {
		pll_rate = ABOX_AUD_PLL_RATE_48K;
		audif_rate = ABOX_AUDIF_RATE;
	}

	mutex_lock(&abox->uaif_lock);
	if (abox->uaif3_enabled) {
		ret = -EBUSY;
		goto out_unlock;
	}
	ret = clk_set_rate(abox->clocks[ABOX_CLK_PLL].clk, pll_rate);
	if (ret)
		goto out_unlock;
	ret = clk_set_rate(abox->clocks[ABOX_CLK_AUDIF].clk, audif_rate);
	if (ret)
		goto out_unlock;
	ret = clk_set_parent(abox->clocks[ABOX_CLK_UAIF3_MUX].clk,
			     abox->clocks[ABOX_CLK_UAIF3_DIV].clk);
	if (ret)
		goto out_unlock;
	if (clk_get_parent(abox->clocks[ABOX_CLK_UAIF3_MUX].clk) !=
	    abox->clocks[ABOX_CLK_UAIF3_DIV].clk) {
		ret = -EIO;
		goto out_unlock;
	}
	ret = clk_set_rate(abox->clocks[ABOX_CLK_UAIF3_DIV].clk, bclk_rate);
	if (ret)
		goto out_unlock;
	if (clk_get_rate(abox->clocks[ABOX_CLK_UAIF3_DIV].clk) != bclk_rate ||
	    clk_get_rate(abox->clocks[ABOX_CLK_UAIF3_BCLK].clk) != bclk_rate) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ctrl1 = readl(abox->sfr + ABOX_UAIF3_CTRL1);
	ctrl1 &= ~(ABOX_UAIF3_FORMAT | ABOX_UAIF3_SLOT_MAX |
		   ABOX_UAIF3_SBIT_MAX | ABOX_UAIF3_VALID_START |
		   ABOX_UAIF3_VALID_END);
	ctrl1 |= FIELD_PREP(ABOX_UAIF3_FORMAT,
			    1 | (sample_code << 3));
	ctrl1 |= FIELD_PREP(ABOX_UAIF3_SLOT_MAX, channels - 1);
	ctrl1 |= FIELD_PREP(ABOX_UAIF3_SBIT_MAX, slot_width - 1);
	writel(ctrl1, abox->sfr + ABOX_UAIF3_CTRL1);
	/* Make the slot configuration visible before the PCM trigger. */
	wmb();

out_unlock:
	mutex_unlock(&abox->uaif_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos7885_abox_uaif3_hw_params);

int exynos7885_abox_uaif3_set_enabled(struct device *dev, bool enable)
{
	struct rproc *rproc = exynos7885_abox_get_rproc(dev);
	struct exynos7885_abox_rproc *abox;
	u32 ctrl0;
	int ret;

	if (!rproc)
		return -ENODEV;
	abox = rproc->priv;
	if (!READ_ONCE(abox->firmware_ready))
		return -EHOSTDOWN;

	mutex_lock(&abox->uaif_lock);
	if (enable == abox->uaif3_enabled) {
		ret = 0;
		goto out_unlock;
	}
	if (enable) {
		ret = clk_prepare_enable(abox->clocks[ABOX_CLK_UAIF3_BCLK].clk);
		if (ret)
			goto out_unlock;
		ret = clk_prepare_enable(abox->clocks[ABOX_CLK_UAIF3_SYNC].clk);
		if (ret) {
			clk_disable_unprepare(abox->clocks[ABOX_CLK_UAIF3_BCLK].clk);
			goto out_unlock;
		}
		ctrl0 = readl(abox->sfr + ABOX_UAIF3_CTRL0);
		writel(ctrl0 | ABOX_UAIF3_SPK_ENABLE,
		       abox->sfr + ABOX_UAIF3_CTRL0);
		/* Enable the output before asking firmware to start RDMA. */
		wmb();
		abox->uaif3_enabled = true;
		ret = 0;
		goto out_unlock;
	}

	ret = exynos7885_abox_wait_rdma_idle(dev, 0, ABOX_STOP_TIMEOUT_US);
	if (ret)
		goto out_unlock;
	ctrl0 = readl(abox->sfr + ABOX_UAIF3_CTRL0);
	writel(ctrl0 & ~ABOX_UAIF3_SPK_ENABLE,
	       abox->sfr + ABOX_UAIF3_CTRL0);
	/* Disable the output before removing its clocks. */
	wmb();
	clk_disable_unprepare(abox->clocks[ABOX_CLK_UAIF3_SYNC].clk);
	clk_disable_unprepare(abox->clocks[ABOX_CLK_UAIF3_BCLK].clk);
	abox->uaif3_enabled = false;
	ret = 0;

out_unlock:
	mutex_unlock(&abox->uaif_lock);
	return ret;
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

static int exynos7885_abox_release_resources(struct exynos7885_abox_rproc *abox)
{
	unsigned int status;
	int i, ret;

	ret = exynos7885_abox_check_idle(abox);
	if (ret)
		return ret;

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
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_OPTION,
				 ABOX_CA7_ENABLE, 0);
	if (ret)
		goto fail_before_release;
	ret = regmap_read_poll_timeout(abox->pmu, ABOX_CA7_STATUS, status,
				       !(status & ABOX_CA7_STATUS_ON), 100,
				       ABOX_STOP_TIMEOUT_US);
	if (ret)
		goto fail_before_release;
	ret = regmap_update_bits(abox->pmu, ABOX_CA7_CONFIGURATION,
				 ABOX_CA7_LOCAL_PWR, 0);
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

	ret = clk_set_rate(abox->clocks[ABOX_CLK_AUDIF].clk, ABOX_AUDIF_RATE);
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
	dev_info(dev, "ABOX probe: System MMU provider ready\n");

	for (i = 0; i < ARRAY_SIZE(abox->clocks); i++)
		abox->clocks[i].id = exynos7885_abox_clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(abox->clocks), abox->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get ABOX clocks\n");
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

	rproc->has_iommu = true;
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	platform_set_drvdata(pdev, rproc);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;
	dev_info(dev, "ABOX probe: remoteproc registered offline\n");

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
