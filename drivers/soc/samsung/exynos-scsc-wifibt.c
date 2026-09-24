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
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/unaligned.h>

/* Mailbox (AP side, R4 bank) register offsets */
#define SCSC_MBOX_INTMSR0	0x018 /* Interrupt mask status, upper half is FROM R4/M4 */
#define SCSC_MBOX_INTCR0	0x00c /* Interrupt clear, write 1 to clear */
#define SCSC_MBOX_IS_VERSION	0x050 /* Firmware block version information */

/* PMU (system-controller syscon) register offsets */
#define SCSC_PMU_WIFI_CTRL_NS		0x140 /* non-secure control */
#define SCSC_PMU_WIFI_PWRON		BIT(1)
#define SCSC_PMU_WIFI_RESET_SET		BIT(2)
#define SCSC_PMU_WIFI_CTRL_S		0x144 /* secure control */
#define SCSC_PMU_WIFI_START		BIT(3)
#define SCSC_PMU_WIFI_STAT		0x148
#define SCSC_PMU_WIFI_PWRDN_DONE	BIT(0)

/* Shared-memory (BAAW) window configuration */
#define SCSC_PMU_MEM_CONFIG0		0x7300 /* WiFi window size (4K units) */
#define SCSC_PMU_MEM_CONFIG1		0x7304 /* WiFi window base (4K units) */

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
	struct regmap	*pmureg;
	phys_addr_t	mem_start;
	size_t		mem_size;
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

static int scsc_wifibt_power_on(struct scsc_wifibt *scsc)
{
	unsigned int val;
	int ret;

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

	/* Power on, release reset, start: mirrors downstream 8.6.6 sequence. */
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

	return 0;
}

static void scsc_wifibt_fw_parse(struct scsc_wifibt *scsc,
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
		return;
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

	/* Integrity checks over the image, same layout as downstream fwimage. */
	fw_crc = get_unaligned_le32(fw->data + SCSC_FW_CRC_OFF);
	const_crc = get_unaligned_le32(fw->data + SCSC_FW_CONST_CRC_OFF);
	hdr_crc = get_unaligned_le32(fw->data + hdr_len - sizeof(u32));

	if (hdr_len > fw->size || const_len > fw->size) {
		dev_err(scsc->dev, "firmware lengths out of range\n");
		return;
	}

	if (ether_crc(hdr_len - sizeof(u32), fw->data) != hdr_crc ||
	    ether_crc(const_len - hdr_len, fw->data + hdr_len) != const_crc ||
	    ether_crc(fw->size - hdr_len, fw->data + hdr_len) != fw_crc)
		dev_err(scsc->dev, "firmware CRC mismatch\n");
	else
		dev_info(scsc->dev, "firmware CRCs OK\n");
}

static void scsc_wifibt_power_off(struct scsc_wifibt *scsc)
{	unsigned int val;
	int ret;

	ret = regmap_update_bits(scsc->pmureg, SCSC_PMU_RESET_AHEAD,
				 SCSC_PMU_SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_CLEANY_BUS,
				  SCSC_PMU_SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_LOGIC_RESET,
				  SCSC_PMU_SYS_PWR_CFG_2, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_TCXO_GATE,
				  SCSC_PMU_SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_DISABLE_ISO,
				  SCSC_PMU_SYS_PWR_CFG, SCSC_PMU_SYS_PWR_CFG);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_RESET_ISO,
				  SCSC_PMU_SYS_PWR_CFG, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_CENTRAL_SEQ_CFG,
				  SCSC_PMU_SYS_PWR_CFG_16, 0);
	ret |= regmap_update_bits(scsc->pmureg, SCSC_PMU_WIFI_CTRL_NS,
				  SCSC_PMU_WIFI_RESET_SET,
				  SCSC_PMU_WIFI_RESET_SET);
	if (ret) {
		dev_warn(scsc->dev, "power-off sequencing write failed: %d\n",
			 ret);
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

	/* Validate the firmware packaging; the boot itself comes later. */
	ret = request_firmware(&fw, SCSC_FW_NAME, dev);
	if (ret) {
		dev_info(dev, "firmware %s not available yet: %d\n",
			 SCSC_FW_NAME, ret);
	} else {
		dev_info(dev, "firmware %s size %zu\n", SCSC_FW_NAME, fw->size);
		scsc_wifibt_fw_parse(scsc, fw);
		release_firmware(fw);
	}

	ret = scsc_wifibt_power_on(scsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	dev_info(dev, "probed\n");

	return 0;
}

static void scsc_wifibt_remove(struct platform_device *pdev)
{
	struct scsc_wifibt *scsc = platform_get_drvdata(pdev);

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
