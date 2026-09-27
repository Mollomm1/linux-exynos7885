// SPDX-License-Identifier: GPL-2.0-only
/* Inert Exynos7885 ABOX resource and firmware preflight. */

#include <linux/firmware.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define ABOX_SRAM_SIZE		0x28000
#define ABOX_DRAM_LIMIT		(12 * 1024 * 1024)

struct abox_preflight {
	struct mutex lock;
	size_t sram_firmware_size;
	size_t dram_firmware_size;
	int last_result;
	bool checked;
};

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Explicitly enable inert ABOX resource binding");

static ssize_t firmware_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct abox_preflight *abox = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&abox->lock);
	if (!abox->checked)
		len = sysfs_emit(buf, "unchecked\n");
	else
		len = sysfs_emit(buf, "result=%d sram=%zu dram=%zu\n",
				 abox->last_result, abox->sram_firmware_size,
				 abox->dram_firmware_size);
	mutex_unlock(&abox->lock);

	return len;
}
static DEVICE_ATTR_RO(firmware_status);

static ssize_t verify_firmware_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct abox_preflight *abox = dev_get_drvdata(dev);
	const struct firmware *sram = NULL, *dram = NULL;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&abox->lock);
	abox->sram_firmware_size = 0;
	abox->dram_firmware_size = 0;
	ret = request_firmware(&sram, "postmarketos/calliope_sram.bin", dev);
	if (ret)
		goto out;
	abox->sram_firmware_size = sram->size;
	if (!sram->size || sram->size > ABOX_SRAM_SIZE) {
		ret = -EINVAL;
		goto out;
	}

	ret = request_firmware(&dram, "postmarketos/calliope_dram.bin", dev);
	if (ret)
		goto out;
	abox->dram_firmware_size = dram->size;
	if (!dram->size || dram->size > ABOX_DRAM_LIMIT)
		ret = -EINVAL;
out:
	abox->last_result = ret;
	abox->checked = true;
	release_firmware(dram);
	release_firmware(sram);
	mutex_unlock(&abox->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(verify_firmware);

static struct attribute *abox_preflight_attrs[] = {
	&dev_attr_firmware_status.attr,
	&dev_attr_verify_firmware.attr,
	NULL,
};
ATTRIBUTE_GROUPS(abox_preflight);

static int abox_preflight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct abox_preflight *abox;
	struct resource *sfr, *sysreg, *sram;
	ktime_t start = ktime_get();

	if (!enable)
		return -ENODEV;

	sfr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfr");
	sysreg = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysreg");
	sram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!sfr || !sysreg || !sram || resource_size(sfr) < 0x10000 ||
	    resource_size(sysreg) < 0x10000 ||
	    resource_size(sram) < ABOX_SRAM_SIZE)
		return dev_err_probe(dev, -EINVAL, "invalid ABOX resources\n");

	abox = devm_kzalloc(dev, sizeof(*abox), GFP_KERNEL);
	if (!abox)
		return -ENOMEM;
	mutex_init(&abox->lock);
	platform_set_drvdata(pdev, abox);

	dev_info(dev, "inert: resources %pr %pr %pr validated in %lld us; no MMIO or firmware start\n",
		 sfr, sysreg, sram, ktime_us_delta(ktime_get(), start));
	return 0;
}

static const struct of_device_id abox_preflight_of_match[] = {
	{ .compatible = "samsung,exynos7885-abox" },
	{ }
};
/* No module alias: binding requires an explicit module load with enable=1. */

static struct platform_driver abox_preflight_driver = {
	.probe = abox_preflight_probe,
	.driver = {
		.name = "exynos7885-abox-preflight",
		.of_match_table = abox_preflight_of_match,
		.dev_groups = abox_preflight_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(abox_preflight_driver);

MODULE_DESCRIPTION("Inert Exynos7885 ABOX preflight");
MODULE_LICENSE("GPL");
