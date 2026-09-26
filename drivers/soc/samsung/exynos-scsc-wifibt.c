// SPDX-License-Identifier: GPL-2.0-only
/* Resource-only bring-up for the Exynos7885 WiFi/BT subsystem. */

#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Explicitly enable resource-only binding (no firmware)");

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sysfs_emit(buf, "inert\n");
}
static DEVICE_ATTR_RO(state);

static struct attribute *scsc_attrs[] = {
	&dev_attr_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(scsc);

static int scsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct resource mem;
	void __iomem *base;
	ktime_t start = ktime_get();
	int ret;

	if (!enable)
		return -ENODEV;

	/* Map the mailboxes, but never access an unpowered register bank. */
	base = devm_platform_ioremap_resource_byname(pdev, "r4");
	if (IS_ERR(base))
		return PTR_ERR(base);
	base = devm_platform_ioremap_resource_byname(pdev, "m4");
	if (IS_ERR(base))
		return PTR_ERR(base);

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return -EINVAL;
	ret = of_address_to_resource(np, 0, &mem);
	if (!of_property_read_bool(np, "no-map"))
		ret = -EINVAL;
	of_node_put(np);
	if (ret)
		return ret;
	if (!devm_request_mem_region(dev, mem.start, resource_size(&mem),
				     dev_name(dev)))
		return -EBUSY;

	/* No IRQs, work, PMU changes or firmware activity to tear down. */
	dev_info(dev, "inert: reserved %pr in %lld us; no firmware execution\n",
		 &mem, ktime_us_delta(ktime_get(), start));
	return 0;
}

static const struct of_device_id scsc_of_match[] = {
	{ .compatible = "samsung,exynos7885-wifibt" },
	{ }
};
/* Deliberately no MODULE_DEVICE_TABLE: this experiment must not autoload. */

static struct platform_driver scsc_driver = {
	.probe = scsc_probe,
	.driver = {
		.name = "exynos-scsc-wifibt",
		.of_match_table = scsc_of_match,
		.dev_groups = scsc_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(scsc_driver);

MODULE_DESCRIPTION("Exynos7885 WiFi/BT inert resource bring-up");
MODULE_LICENSE("GPL");
