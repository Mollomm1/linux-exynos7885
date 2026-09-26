// SPDX-License-Identifier: GPL-2.0-only
/* Non-executing bring-up for the Exynos7885 WiFi/BT subsystem. */

#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define SCSC_FW_MIN_HEADER	188
#define SCSC_FW_DIRECTORY	"postmarketos/mx140/"

struct scsc_device {
	struct regmap *pmu;
	struct mutex lock;
	size_t mem_size;
	bool checked;
	int firmware_result;
	u32 runtime_length;
	u32 entry_point;
};

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Explicitly enable binding (never starts firmware)");

/* Offsets and CRC coverage are documented in downstream fwhdr/fwimage.c. */
static int scsc_check_firmware(struct scsc_device *scsc,
			       const struct firmware *fw)
{
	const u8 *data = fw->data;
	u32 header, constant, runtime, entry;
	u16 major, minor;

	if (fw->size < SCSC_FW_MIN_HEADER || fw->size > scsc->mem_size)
		return -EINVAL;
	if (memcmp(data + 8, "smxf", 4))
		return -ENOEXEC;

	minor = get_unaligned_le16(data + 12);
	major = get_unaligned_le16(data + 14);
	if (!((major == 0 && minor == 2) || (major == 1 && minor == 0)))
		return -EPROTONOSUPPORT;
	/* Only the mxconf v0.2 firmware interface has been inspected. */
	if (get_unaligned_le16(data + 20) != 2 ||
	    get_unaligned_le16(data + 22) != 0)
		return -EPROTONOSUPPORT;

	header = get_unaligned_le32(data + 16);
	constant = get_unaligned_le32(data + 28);
	runtime = get_unaligned_le32(data + 36);
	entry = get_unaligned_le32(data + 40);
	if (header < SCSC_FW_MIN_HEADER || header > fw->size ||
	    constant < header || constant > fw->size ||
	    runtime < fw->size || runtime > scsc->mem_size ||
	    (entry & ~1U) < header || (entry & ~1U) >= constant)
		return -EINVAL;

	/* ether_crc uses reflected CRC32 with all-one seed, then bit reversal. */
	if (bitrev32(crc32_le(~0U, data, header - 4)) !=
	    get_unaligned_le32(data + header - 4) ||
	    bitrev32(crc32_le(~0U, data + header, constant - header)) !=
	    get_unaligned_le32(data + 32) ||
	    bitrev32(crc32_le(~0U, data + header, fw->size - header)) !=
	    get_unaligned_le32(data + 24))
		return -EBADMSG;

	scsc->runtime_length = runtime;
	scsc->entry_point = entry;
	return 0;
}

static ssize_t verify_firmware_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	const struct firmware *fw;
	char name[128];
	ktime_t start;
	int ret;

	if (!count || count >= sizeof(name) || memchr(buf, '\0', count))
		return -EINVAL;
	memcpy(name, buf, count);
	name[count] = '\0';
	strim(name);
	if (strncmp(name, SCSC_FW_DIRECTORY, strlen(SCSC_FW_DIRECTORY)) ||
	    strstr(name, ".."))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	start = ktime_get();
	scsc->checked = true;
	scsc->runtime_length = 0;
	scsc->entry_point = 0;
	/* No userspace fallback, shared-memory writes, or hardware activity. */
	ret = request_firmware_direct(&fw, name, dev);
	if (!ret) {
		ret = scsc_check_firmware(scsc, fw);
		release_firmware(fw);
	}
	scsc->firmware_result = ret;
	dev_info(dev, "firmware check result=%d in %lld us; not staged or started\n",
		 ret, ktime_us_delta(ktime_get(), start));
	mutex_unlock(&scsc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(verify_firmware);

static ssize_t firmware_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&scsc->lock);
	if (!scsc->checked)
		len = sysfs_emit(buf, "unchecked\n");
	else
		len = sysfs_emit(buf, "result=%d runtime=%u entry=0x%x executed=0\n",
				scsc->firmware_result, scsc->runtime_length,
				scsc->entry_point);
	mutex_unlock(&scsc->lock);
	return len;
}
static DEVICE_ATTR_RO(firmware_status);

static ssize_t pmu_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	/* Always-on PMU only: never scan a range or read unpowered mailboxes. */
	static const u32 offsets[] = { 0x0140, 0x0144, 0x0148, 0x0384,
				       0x7300, 0x7304 };
	unsigned int values[ARRAY_SIZE(offsets)];
	ssize_t len = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		ret = regmap_read(scsc->pmu, offsets[i], &values[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(offsets); i++)
		len += sysfs_emit_at(buf, len, "%04x: %08x\n", offsets[i], values[i]);
	return len;
}
static DEVICE_ATTR_ADMIN_RO(pmu_state);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sysfs_emit(buf, "inert\n");
}
static DEVICE_ATTR_RO(state);

static struct attribute *scsc_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_verify_firmware.attr,
	&dev_attr_firmware_status.attr,
	&dev_attr_pmu_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(scsc);

static int scsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scsc_device *scsc;
	struct device_node *np;
	struct resource mem;
	void __iomem *base;
	ktime_t start = ktime_get();
	int ret;

	if (!enable)
		return -ENODEV;
	scsc = devm_kzalloc(dev, sizeof(*scsc), GFP_KERNEL);
	if (!scsc)
		return -ENOMEM;
	mutex_init(&scsc->lock);
	scsc->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						  "samsung,pmu-syscon");
	if (IS_ERR(scsc->pmu))
		return PTR_ERR(scsc->pmu);
	platform_set_drvdata(pdev, scsc);

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
	scsc->mem_size = resource_size(&mem);

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
