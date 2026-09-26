// SPDX-License-Identifier: GPL-2.0-only
/* Non-executing bring-up for the Exynos7885 WiFi/BT subsystem. */

#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/arm-smccc.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#define SCSC_FW_MIN_HEADER	188
#define SCSC_FW_DIRECTORY	"postmarketos/mx140/"
#define SCSC_TZASC_SMC		0x82000710
#define SCSC_TZASC_WLBT		0
#define SCSC_PMU_WIFI_CTRL_NS	0x0140
#define SCSC_PMU_WIFI_CTRL_S	0x0144
#define SCSC_PMU_WIFI_STAT	0x0148
#define SCSC_PMU_BAAW_SIZE0	0x7300
#define SCSC_PMU_BAAW_BASE0	0x7304
#define SCSC_PMU_CP_STATUS	0x0038
#define SCSC_PMU_SHARED_REG_STATUS	0x3644
#define SCSC_PMU_SHARED_REG_OPTION	0x3648
#define SCSC_WIFI_PWRON		BIT(1)
#define SCSC_WIFI_START		BIT(3)
#define SCSC_MXCONF_MAGIC	0x79828486
#define SCSC_MXCONF_SIZE	162
#define SCSC_MXCONF_VERSION_MAJOR	0
#define SCSC_MXCONF_VERSION_MINOR	1
#define SCSC_MIFRAM_BLOCK_SIZE	64
#define SCSC_MIFRAM_MAX_BLOCKS	65536
#define SCSC_MGMT_BUFFER_SIZE	512
#define SCSC_MGMT_PACKET_SIZE	8
#define SCSC_GDB_BUFFER_SIZE	2048
#define SCSC_GDB_PACKET_SIZE	4
#define SCSC_MXLOG_BUFFER_SIZE	(16 * 1024)
#define SCSC_MXLOG_PACKET_SIZE	4
#define SCSC_MXCONF_STREAM_SIZE	22
#define SCSC_MIF_NUM_MAILBOXES	8
#define SCSC_MIF_ISSR_BASE	0x80

struct scsc_stream_config {
	u32 buffer;
	u32 num_packets;
	u32 packet_size;
	u32 read_index;
	u32 write_index;
	u8 read_bit;
	u8 write_bit;
	bool firmware_to_host;
};

struct scsc_fw_allocator {
	size_t cursor;
	size_t heap_offset;
	size_t pool_size;
	u32 block_count;
	int tohost_bit;
	int fromhost_r4_bit;
	int fromhost_m4_bit;
};

struct scsc_device {
	struct regmap *pmu;
	struct mutex lock;
	void __iomem *memory;
	void __iomem *r4_mailbox[SCSC_MIF_NUM_MAILBOXES];
	void __iomem *m4_mailbox[SCSC_MIF_NUM_MAILBOXES];
	phys_addr_t mem_start;
	size_t mem_size;
	resource_size_t r4_reg_size;
	resource_size_t m4_reg_size;
	bool checked;
	bool staged;
	bool memory_ready;
	bool config_ready;
	u32 config_offset;
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

static ssize_t stage_firmware_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	const struct firmware *fw;
	void *readback;
	size_t fw_size;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	if (scsc->staged) {
		ret = -EALREADY;
		goto out_unlock;
	}

	ret = request_firmware_direct(&fw,
				      SCSC_FW_DIRECTORY "mx140.bin", dev);
	if (ret)
		goto out_unlock;

	ret = scsc_check_firmware(scsc, fw);
	scsc->checked = true;
	scsc->firmware_result = ret;
	if (ret)
		goto out_release_fw;
	fw_size = fw->size;

	readback = kvzalloc(fw_size, GFP_KERNEL);
	if (!readback) {
		ret = -ENOMEM;
		goto out_release_fw;
	}

	memcpy_toio(scsc->memory, fw->data, fw_size);
	/* Ensure posted writes reach reserved memory before reading it back. */
	wmb();
	memcpy_fromio(readback, scsc->memory, fw_size);
	if (memcmp(fw->data, readback, fw_size))
		ret = -EIO;
	else {
		scsc->staged = true;
		ret = 0;
		scsc->firmware_result = 0;
	}
	kvfree(readback);

out_release_fw:
	release_firmware(fw);
	if (!ret)
		dev_info(dev, "staged and verified %zu bytes; firmware not started\n",
			 fw_size);
out_unlock:
	if (ret)
		dev_err(dev, "reserved DRAM aperture preparation failed: %d\n",
			ret);
	mutex_unlock(&scsc->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(stage_firmware);

/* Configure only the reserved DRAM aperture; never power or release WLBT. */
static ssize_t prepare_memory_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	struct arm_smccc_res res;
	unsigned int ctrl_ns, ctrl_s, status, size, base;
	u32 encoded_base;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	if (!scsc->staged) {
		ret = -EINVAL;
		goto out_unlock;
	}
	if (scsc->memory_ready) {
		ret = -EALREADY;
		goto out_unlock;
	}

	ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_CTRL_NS, &ctrl_ns);
	if (ret)
		goto out_unlock;
	ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_CTRL_S, &ctrl_s);
	if (ret)
		goto out_unlock;
	ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_STAT, &status);
	if (ret)
		goto out_unlock;
	if ((ctrl_ns & SCSC_WIFI_PWRON) || (ctrl_s & SCSC_WIFI_START) || status) {
		ret = -EBUSY;
		goto out_unlock;
	}
	if (!IS_ALIGNED(scsc->mem_start, SZ_4K) ||
	    !IS_ALIGNED(scsc->mem_size, SZ_4K) || scsc->mem_size > SZ_1G) {
		ret = -EINVAL;
		goto out_unlock;
	}

	/* Match downstream's WLBT TZASC call, but fail closed on its result. */
	arm_smccc_smc(SCSC_TZASC_SMC, SCSC_TZASC_WLBT, scsc->mem_start,
		      scsc->mem_size, 0, 0, 0, 0, &res);
	if ((long)res.a0) {
		ret = (long)res.a0 < 0 ? (long)res.a0 : -EIO;
		goto out_unlock;
	}

	encoded_base = (scsc->mem_start & 0xfffffc000ULL) >> 12;
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_SIZE0,
			   scsc->mem_size >> 12);
	if (ret)
		goto out_unlock;
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_BASE0, encoded_base);
	if (ret)
		goto out_unlock;

	ret = regmap_read(scsc->pmu, SCSC_PMU_BAAW_SIZE0, &size);
	if (ret)
		goto out_unlock;
	ret = regmap_read(scsc->pmu, SCSC_PMU_BAAW_BASE0, &base);
	if (ret)
		goto out_unlock;
	if (size != scsc->mem_size >> 12 || base != encoded_base) {
		ret = -EIO;
		goto out_unlock;
	}

	scsc->memory_ready = true;
	dev_info(dev, "reserved DRAM aperture prepared at %pa size %zu; WLBT remains off\n",
		 &scsc->mem_start, scsc->mem_size);
	ret = 0;
out_unlock:
	mutex_unlock(&scsc->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(prepare_memory);

static int scsc_fw_alloc(struct scsc_device *scsc,
			 struct scsc_fw_allocator *allocator, size_t size,
			 u32 *offset)
{
	size_t allocation_size, rounded_size, end, data_offset;
	u32 blocks;

	/* Match miframman_alloc(): 8-byte alignment plus its pointer header. */
	if (check_add_overflow(size, (size_t)16, &allocation_size))
		return -EOVERFLOW;
	blocks = DIV_ROUND_UP(allocation_size, SCSC_MIFRAM_BLOCK_SIZE);
	if (check_mul_overflow((size_t)blocks,
			       (size_t)SCSC_MIFRAM_BLOCK_SIZE, &rounded_size) ||
	    check_add_overflow(allocator->cursor, rounded_size, &end))
		return -EOVERFLOW;
	if (end > allocator->pool_size || blocks > SCSC_MIFRAM_MAX_BLOCKS ||
	    allocator->block_count > SCSC_MIFRAM_MAX_BLOCKS - blocks)
		return -ENOSPC;
	if (check_add_overflow(allocator->heap_offset, allocator->cursor,
			       &data_offset) ||
	    check_add_overflow(data_offset, (size_t)16, &data_offset) ||
	    data_offset > scsc->mem_size || data_offset > U32_MAX)
		return -EOVERFLOW;

	*offset = data_offset;
	allocator->cursor += rounded_size;
	allocator->block_count += blocks;
	return 0;
}

static int scsc_fw_stream_alloc(struct scsc_device *scsc,
				struct scsc_fw_allocator *allocator,
				struct scsc_stream_config *stream,
				u32 buffer_size, u32 packet_size,
				bool firmware_to_host, bool m4,
				bool reserve_panic_bit)
{
	u32 index_size = sizeof(u32);
	int fromhost_bit, tohost_bit, ret;

	if (!packet_size || buffer_size % packet_size)
		return -EINVAL;
	stream->num_packets = buffer_size / packet_size;
	stream->packet_size = packet_size;
	stream->firmware_to_host = firmware_to_host;
	ret = scsc_fw_alloc(scsc, allocator, buffer_size, &stream->buffer);
	if (ret)
		return ret;
	ret = scsc_fw_alloc(scsc, allocator, index_size, &stream->read_index);
	if (ret)
		return ret;
	ret = scsc_fw_alloc(scsc, allocator, index_size, &stream->write_index);
	if (ret)
		return ret;

	if (allocator->tohost_bit >= 16)
		return -ENOSPC;
	tohost_bit = allocator->tohost_bit++;
	if (reserve_panic_bit) {
		fromhost_bit = 0;
	} else if (m4) {
		if (allocator->fromhost_m4_bit >= 16)
			return -ENOSPC;
		fromhost_bit = allocator->fromhost_m4_bit++;
	} else {
		if (allocator->fromhost_r4_bit >= 16)
			return -ENOSPC;
		fromhost_bit = allocator->fromhost_r4_bit++;
	}

	if (firmware_to_host) {
		stream->read_bit = fromhost_bit;
		stream->write_bit = tohost_bit;
	} else {
		stream->read_bit = tohost_bit;
		stream->write_bit = fromhost_bit;
	}
	return 0;
}

static void scsc_fw_stream_serialize(u8 *dst,
				     const struct scsc_stream_config *stream)
{
	put_unaligned_le32(stream->buffer, dst);
	put_unaligned_le32(stream->num_packets, dst + 4);
	put_unaligned_le32(stream->packet_size, dst + 8);
	put_unaligned_le32(stream->read_index, dst + 12);
	put_unaligned_le32(stream->write_index, dst + 16);
	dst[20] = stream->read_bit;
	dst[21] = stream->write_bit;
}

static int scsc_fw_write_stream(struct scsc_device *scsc,
				const struct scsc_stream_config *stream,
				bool clear_to_ff)
{
	size_t buffer_size = (size_t)stream->num_packets * stream->packet_size;

	if (stream->buffer > scsc->mem_size ||
	    buffer_size > scsc->mem_size - stream->buffer ||
	    stream->read_index > scsc->mem_size - sizeof(u32) ||
	    stream->write_index > scsc->mem_size - sizeof(u32))
		return -EINVAL;
	memset_io(scsc->memory + stream->buffer, clear_to_ff ? 0xff : 0,
		  buffer_size);
	iowrite32(0, scsc->memory + stream->read_index);
	iowrite32(0, scsc->memory + stream->write_index);
	return 0;
}

/* Prepare the documented SCSC ring layout without touching WLBT registers. */
static ssize_t prepare_config_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	struct scsc_fw_allocator allocator = { };
	struct scsc_stream_config stream[7] = { };
	u8 config[SCSC_MXCONF_SIZE] = { };
	u8 readback[SCSC_MXCONF_SIZE];
	size_t end;
	u32 config_offset;
	int ret, i;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	if (!scsc->staged || !scsc->memory_ready) {
		ret = -EINVAL;
		goto out_unlock;
	}
	if (scsc->config_ready) {
		ret = -EALREADY;
		goto out_unlock;
	}
	if (scsc->runtime_length >= scsc->mem_size ||
	    !IS_ALIGNED(scsc->runtime_length, sizeof(u64))) {
		ret = -EINVAL;
		goto out_unlock;
	}

	allocator.heap_offset = scsc->runtime_length;
	allocator.pool_size = scsc->mem_size - allocator.heap_offset;
	allocator.fromhost_r4_bit = 1; /* bit 0 is reserved for R4 panic */
	allocator.fromhost_m4_bit = 1; /* bit 0 is reserved for M4 panic */
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[0],
				   SCSC_MGMT_BUFFER_SIZE, SCSC_MGMT_PACKET_SIZE,
				   true, false, false);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[1],
				   SCSC_MGMT_BUFFER_SIZE, SCSC_MGMT_PACKET_SIZE,
				   false, false, false);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[2],
				   SCSC_GDB_BUFFER_SIZE, SCSC_GDB_PACKET_SIZE,
				   true, false, false);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[3],
				   SCSC_GDB_BUFFER_SIZE, SCSC_GDB_PACKET_SIZE,
				   false, false, true);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[4],
				   SCSC_GDB_BUFFER_SIZE, SCSC_GDB_PACKET_SIZE,
				   true, true, false);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[5],
				   SCSC_GDB_BUFFER_SIZE, SCSC_GDB_PACKET_SIZE,
				   false, true, true);
	if (ret)
		goto out_unlock;
	ret = scsc_fw_stream_alloc(scsc, &allocator, &stream[6],
				   SCSC_MXLOG_BUFFER_SIZE, SCSC_MXLOG_PACKET_SIZE,
				   true, false, false);
	if (ret)
		goto out_unlock;

	ret = scsc_fw_alloc(scsc, &allocator, SCSC_MXCONF_SIZE,
			    &config_offset);
	if (ret)
		goto out_unlock;
	if (check_add_overflow((size_t)config_offset,
			       (size_t)SCSC_MXCONF_SIZE, &end) ||
	    end > scsc->mem_size) {
		ret = -ENOSPC;
		goto out_unlock;
	}

	put_unaligned_le32(SCSC_MXCONF_MAGIC, config);
	put_unaligned_le16(SCSC_MXCONF_VERSION_MAJOR, config + 4);
	put_unaligned_le16(SCSC_MXCONF_VERSION_MINOR, config + 6);
	for (i = 0; i < ARRAY_SIZE(stream); i++)
		scsc_fw_stream_serialize(config + 8 + i * SCSC_MXCONF_STREAM_SIZE,
					 &stream[i]);

	for (i = 0; i < ARRAY_SIZE(stream); i++) {
		ret = scsc_fw_write_stream(scsc, &stream[i],
					  stream[i].firmware_to_host);
		if (ret)
			goto out_unlock;
	}
	memcpy_toio(scsc->memory + config_offset, config, sizeof(config));
	/* Ensure the SCSC config reaches DRAM before verifying readback. */
	wmb();
	memcpy_fromio(readback, scsc->memory + config_offset,
		      sizeof(readback));
	if (memcmp(config, readback, sizeof(config))) {
		ret = -EIO;
		goto out_unlock;
	}

	scsc->config_offset = config_offset;
	scsc->config_ready = true;
	dev_info(dev,
		 "prepared and verified SCSC config at 0x%x; %u allocator blocks; firmware not started\n",
		 config_offset, allocator.block_count);
	ret = 0;
out_unlock:
	if (ret)
		dev_err(dev, "SCSC shared-memory config preparation failed: %d\n",
			ret);
	mutex_unlock(&scsc->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(prepare_config);

static ssize_t firmware_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&scsc->lock);
	if (!scsc->checked)
		len = sysfs_emit(buf, "unchecked\n");
	else
		len = sysfs_emit(buf,
				 "result=%d runtime=%u entry=0x%x staged=%u memory_ready=%u config_ready=%u config_offset=0x%x executed=0\n",
				 scsc->firmware_result, scsc->runtime_length,
				 scsc->entry_point, scsc->staged, scsc->memory_ready,
				 scsc->config_ready, scsc->config_offset);
	mutex_unlock(&scsc->lock);
	return len;
}
static DEVICE_ATTR_RO(firmware_status);

static ssize_t pmu_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	/* Always-on PMU only: never scan a range or read unpowered mailboxes. */
	static const u32 offsets[] = {
		SCSC_PMU_CP_STATUS, 0x0140, 0x0144, 0x0148, 0x0384,
		0x7300, 0x7304, SCSC_PMU_SHARED_REG_STATUS,
		SCSC_PMU_SHARED_REG_OPTION,
	};
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

/* Build an ISSR pointer without accessing the powered-down mailbox bank. */
static void __iomem *scsc_mailbox_slot(void __iomem *base,
				      resource_size_t reg_size, u32 index)
{
	size_t offset;

	if (index >= SCSC_MIF_NUM_MAILBOXES ||
	    check_mul_overflow((size_t)index, sizeof(u32), &offset) ||
	    check_add_overflow(offset, (size_t)SCSC_MIF_ISSR_BASE, &offset) ||
	    offset > reg_size || reg_size - offset < sizeof(u32))
		return NULL;

	return base + offset;
}

static struct attribute *scsc_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_verify_firmware.attr,
	&dev_attr_stage_firmware.attr,
	&dev_attr_prepare_memory.attr,
	&dev_attr_prepare_config.attr,
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
	struct resource mem, r4_regs, m4_regs;
	void __iomem *r4_base, *m4_base;
	ktime_t start = ktime_get();
	int ret, i;

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

	/* Map and validate mailbox slots, but never access the powered-down bank. */
	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, "r4") ||
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "m4"))
		return -EINVAL;
	r4_regs = *platform_get_resource_byname(pdev, IORESOURCE_MEM, "r4");
	m4_regs = *platform_get_resource_byname(pdev, IORESOURCE_MEM, "m4");
	scsc->r4_reg_size = resource_size(&r4_regs);
	scsc->m4_reg_size = resource_size(&m4_regs);
	if (scsc->r4_reg_size < SCSC_MIF_ISSR_BASE +
	    SCSC_MIF_NUM_MAILBOXES * sizeof(u32) ||
	    scsc->m4_reg_size < SCSC_MIF_ISSR_BASE +
	    SCSC_MIF_NUM_MAILBOXES * sizeof(u32))
		return -EINVAL;
	r4_base = devm_platform_ioremap_resource_byname(pdev, "r4");
	if (IS_ERR(r4_base))
		return PTR_ERR(r4_base);
	m4_base = devm_platform_ioremap_resource_byname(pdev, "m4");
	if (IS_ERR(m4_base))
		return PTR_ERR(m4_base);
	for (i = 0; i < SCSC_MIF_NUM_MAILBOXES; i++) {
		scsc->r4_mailbox[i] = scsc_mailbox_slot(r4_base,
						       scsc->r4_reg_size, i);
		scsc->m4_mailbox[i] = scsc_mailbox_slot(m4_base,
						       scsc->m4_reg_size, i);
		if (!scsc->r4_mailbox[i] || !scsc->m4_mailbox[i])
			return -EINVAL;
	}

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
	scsc->mem_start = mem.start;
	scsc->memory = devm_ioremap_wc(dev, mem.start, scsc->mem_size);
	if (!scsc->memory)
		return -ENOMEM;

	/* Bind only; firmware staging is a separate explicit sysfs operation. */
	dev_info(dev,
		 "inert: reserved %pr and mapped %u R4/M4 mailbox slots in %lld us; no MMIO access or firmware execution\n",
		 &mem, SCSC_MIF_NUM_MAILBOXES,
		 ktime_us_delta(ktime_get(), start));
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
