// SPDX-License-Identifier: GPL-2.0-only
/* Explicit-load, non-booting Exynos7885 ABOX remoteproc staging driver. */

#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/memremap.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>

#define ABOX_SRAM_SIZE		0x28000
#define ABOX_DRAM_SIZE		(SZ_8M + SZ_4M)
#define ABOX_DRAM_IOVA		0x80000000
#define ABOX_PMU_IOVA		0x11c80000
#define ABOX_PMU_SIZE		SZ_64K
#define ABOX_AUDSYS_IOVA	0x12090000
#define ABOX_AUDSYS_SIZE	PAGE_SIZE
#define ABOX_DISPAUD_STATUS	0x4024
#define ABOX_CA7_STATUS		0x2524

struct exynos7885_abox_rproc {
	struct regmap *pmu;
	struct reserved_mem *dram_rmem;
	struct rproc_mem_entry *dram;
	bool pmu_mapped;
	bool audsys_mapped;
};

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

	/* The core frees carveout entries before calling unprepare. */
	abox->dram = NULL;

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

	memset(abox->dram->va, 0, abox->dram->len);
	memcpy(abox->dram->va, dram->data, dram->size);
	dev_info(&rproc->dev, "validated SRAM %zu bytes and staged DRAM %zu bytes; CPU remains off\n",
		 sram->size, dram->size);
out:
	release_firmware(dram);
	return ret;
}

static int exynos7885_abox_start(struct rproc *rproc)
{
	/* Firmware execution requires a verified boot-ack and stop path. */
	return -EOPNOTSUPP;
}

static int exynos7885_abox_stop(struct rproc *rproc)
{
	return 0;
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

static int exynos7885_abox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7885_abox_rproc *abox;
	struct resource *sram;
	struct device_node *mem_np;
	struct rproc *rproc;
	int ret;

	if (!enable)
		return -ENODEV;

	sram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!sram || resource_size(sram) < ABOX_SRAM_SIZE)
		return -EINVAL;

	rproc = devm_rproc_alloc(dev, "exynos7885-abox",
				 &exynos7885_abox_ops,
				 "postmarketos/calliope_sram.bin",
				 sizeof(*abox));
	if (!rproc)
		return -ENOMEM;

	abox = rproc->priv;
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
