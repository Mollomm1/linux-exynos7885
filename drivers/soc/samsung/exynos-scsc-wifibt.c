// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal probe-only driver for the Samsung SLSI SCSC (mx140/Lassen)
 * WiFi/BT mailbox found on Exynos7885/7904 (e.g. Galaxy Tab A 10.1 2019).
 *
 * It maps the R4/M4 mailbox registers, reports the firmware block version,
 * checks the shared-memory carveout and PMU state, and verifies the
 * firmware image is loadable via the firmware loader. It performs reads
 * only: powering the block and booting firmware are future work, as is the
 * 802.11 network interface itself.
 *
 * Register map legislation: downstream Samsung Android kernel for T510
 * (drivers/misc/samsung/scsc/mif_reg_S5E7885.h), used as documentation only.
 */

#include <linux/atomic.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Mailbox (AP side, R4 bank) register offsets */
#define SCSC_MBOX_INTMSR0	0x018 /* Interrupt mask status, upper half is FROM R4/M4 */
#define SCSC_MBOX_INTCR0	0x00c /* Interrupt clear, write 1 to clear */
#define SCSC_MBOX_IS_VERSION	0x050 /* Firmware block version information */

/* PMU (system-controller syscon) register offsets */
#define SCSC_PMU_WIFI_STAT	0x148

/*
 * Firmware image name as shipped by the firmware-samsung-gta3xlwifi aport.
 * The final WLAN driver will use the linux-firmware style path instead.
 */
#define SCSC_FW_NAME	"postmarketos/mx140/mx140.bin"

struct scsc_wifibt {
	struct device	*dev;
	void __iomem	*base;
	atomic_t	irq_count;
};

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
	platform_set_drvdata(pdev, scsc);

	/* R4 mailbox bank (index 0); M4 bank (index 1) is reserved for now. */
	scsc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(scsc->base))
		return dev_err_probe(dev, PTR_ERR(scsc->base),
				     "failed to map R4 mailbox\n");

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

	/* PMU syscon: read-only state check, no power sequencing yet. */
	pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						 "samsung,syscon-phandle");
	if (IS_ERR(pmureg))
		return dev_err_probe(dev, PTR_ERR(pmureg),
				     "failed to get PMU syscon\n");

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

	/* Validate the firmware packaging; the boot itself comes later. */
	ret = request_firmware(&fw, SCSC_FW_NAME, dev);
	if (ret) {
		dev_info(dev, "firmware %s not available yet: %d\n",
			 SCSC_FW_NAME, ret);
	} else {
		dev_info(dev, "firmware %s size %zu\n", SCSC_FW_NAME, fw->size);
		release_firmware(fw);
	}

	dev_info(dev, "probed (stub, reads only)\n");

	return 0;
}

static const struct of_device_id scsc_wifibt_of_match[] = {
	{ .compatible = "samsung,scsc-wifibt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scsc_wifibt_of_match);

static struct platform_driver scsc_wifibt_driver = {
	.probe = scsc_wifibt_probe,
	.driver = {
		.name = "scsc-wifibt",
		.of_match_table = scsc_wifibt_of_match,
	},
};
module_platform_driver(scsc_wifibt_driver);

MODULE_AUTHOR("T510 mainlining project");
MODULE_DESCRIPTION("Samsung SCSC WiFi/BT mailbox stub driver (probe only)");
MODULE_LICENSE("GPL");
