// SPDX-License-Identifier: GPL-2.0-only
/* Non-executing bring-up for the Exynos7885 WiFi/BT subsystem. */

#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/io.h>
#include <linux/arm-smccc.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "exynos-scsc-mif-intr.h"
#include "exynos-scsc-shared-rail.h"
#include "scsc_mif_abs.h"

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
#define SCSC_PMU_CP_REFERENCE		0x10
#define SCSC_PMU_SHARED_REG_REFERENCE	0x20001
#define SCSC_PMU_SHARED_REG_OPTION_BIT	BIT(2)
#define SCSC_CP_ISSR2_OFFSET		0x88
#define SCSC_CP_ISSR3_OFFSET		0x8c
#define SCSC_CP_WAKEUP_BIT		BIT(0)
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
#define SCSC_MIF_INTGR1		0x01c
#define SCSC_MIF_INTCR0		0x00c
#define SCSC_MIF_INTMR0		0x010
#define SCSC_MIF_INTMSR0	0x018
#define SCSC_MIF_INTCR1		0x020
#define SCSC_MIF_INTMR1		0x024
#define SCSC_MIF_INTMSR1	0x02c
#define SCSC_PMU_WIFI_RESET_SET	BIT(2)
#define SCSC_PMU_WIFI_RESET_REQ_CLR	BIT(8)
#define SCSC_PMU_RESET_AHEAD	0x1360
#define SCSC_PMU_CLEANY_BUS	0x1364
#define SCSC_PMU_LOGIC_RESET	0x1368
#define SCSC_PMU_TCXO_GATE	0x136c
#define SCSC_PMU_WIFI_DISABLE_ISO	0x1370
#define SCSC_PMU_WIFI_RESET_ISO	0x1374
#define SCSC_PMU_CENTRAL_SEQ_CONFIG	0x0380
#define SCSC_PMU_CENTRAL_SEQ_STATUS	0x0384
#define SCSC_SYS_PWR_CFG	BIT(0)
#define SCSC_SYS_PWR_CFG_2	(BIT(0) | BIT(1))
#define SCSC_SYS_PWR_CFG_16	BIT(16)
#define SCSC_RESET_TIMEOUT_MS	500

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
	const struct acpm_handle *acpm;
	struct device *dev;
	struct mutex lock;
	void __iomem *memory;
	void __iomem *cp_mailbox;
	void __iomem *r4_regs;
	void __iomem *m4_regs;
	void __iomem *r4_mailbox[SCSC_MIF_NUM_MAILBOXES];
	void __iomem *m4_mailbox[SCSC_MIF_NUM_MAILBOXES];
	spinlock_t mif_reg_lock;
	struct scsc_mif_intr mif_intr;
	struct scsc_shared_rail shared_rail;
	struct scsc_mif_abs mif_abs;
	void (*mif_irq_handler)(int irq, void *data);
	void *mif_irq_data;
	void (*mif_reset_handler)(int irq, void *data);
	void *mif_reset_data;
	int (*mif_suspend)(struct scsc_mif_abs *interface, void *data);
	void (*mif_resume)(struct scsc_mif_abs *interface, void *data);
	void *mif_suspend_data;
	phys_addr_t mem_start;
	size_t mem_size;
	resource_size_t r4_reg_size;
	resource_size_t m4_reg_size;
	int mbox_irq;
	int wdog_irq;
	bool checked;
	bool staged;
	bool memory_ready;
	bool config_ready;
	bool voltage_request_attempted;
	bool voltage_prepared;
	bool start_armed;
	bool start_in_progress;
	bool mif_mapped;
	bool wlbt_may_be_running;
	bool stop_failed;
	bool mbox_irq_enabled;
	bool wdog_irq_enabled;
	bool module_pinned;
	u32 config_offset;
	int firmware_result;
	u32 runtime_length;
	u32 entry_point;
};

/* The downstream SCSC core is optional and binds through this singleton API. */
static DEFINE_MUTEX(scsc_mif_registry_lock);
static struct scsc_device *scsc_mif_device;
static struct scsc_mif_abs_driver *scsc_mif_client;

static int scsc_prepare_voltage(struct scsc_device *scsc);

static void scsc_unmap_reserved_memory(void *data)
{
	struct scsc_device *scsc = data;

	if (scsc->memory) {
		vunmap((void __force *)scsc->memory);
		scsc->memory = NULL;
	}
}

static int scsc_map_reserved_memory(struct scsc_device *scsc)
{
	struct page **pages;
	unsigned long first_pfn;
	size_t page_count, i;
	void *memory;

	if (!PAGE_ALIGNED(scsc->mem_start) ||
	    !PAGE_ALIGNED(scsc->mem_size) || !scsc->mem_size)
		return -EINVAL;

	page_count = scsc->mem_size >> PAGE_SHIFT;
	first_pfn = PHYS_PFN(scsc->mem_start);
	pages = kcalloc(page_count, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < page_count; i++) {
		if (!pfn_valid(first_pfn + i)) {
			kfree(pages);
			return -EINVAL;
		}
		pages[i] = pfn_to_page(first_pfn + i);
	}

	/* Match downstream's write-combine vmap of the reserved DRAM pages. */
	memory = vmap(pages, page_count, VM_MAP,
		      pgprot_writecombine(PAGE_KERNEL));
	kfree(pages);
	if (!memory)
		return -ENOMEM;

	scsc->memory = (void __iomem __force *)memory;
	return devm_add_action_or_reset(scsc->dev,
				       scsc_unmap_reserved_memory, scsc);
}

static struct scsc_device *scsc_from_mif(struct scsc_mif_abs *interface)
{
	return container_of(interface, struct scsc_device, mif_abs);
}

static irqreturn_t scsc_mbox_irq(int irq, void *data)
{
	struct scsc_device *scsc = data;
	void (*handler)(int irq, void *data);
	void *handler_data;

	if (!READ_ONCE(scsc->mif_mapped))
		return IRQ_NONE;
	handler = READ_ONCE(scsc->mif_irq_handler);
	handler_data = READ_ONCE(scsc->mif_irq_data);
	if (handler)
		handler(irq, handler_data);

	return IRQ_HANDLED;
}

static irqreturn_t scsc_wdog_irq_thread(int irq, void *data)
{
	struct scsc_device *scsc = data;
	void (*handler)(int irq, void *data);
	void *handler_data;
	int ret;

	if (!READ_ONCE(scsc->mif_mapped))
		return IRQ_NONE;
	handler = READ_ONCE(scsc->mif_reset_handler);
	handler_data = READ_ONCE(scsc->mif_reset_data);
	if (handler)
		handler(irq, handler_data);
	else {
		dev_warn(scsc->dev, "unhandled WLBT watchdog/reset-request IRQ\n");
		disable_irq_nosync(scsc->wdog_irq);
		WRITE_ONCE(scsc->wdog_irq_enabled, false);
	}

	ret = regmap_update_bits(scsc->pmu, SCSC_PMU_WIFI_CTRL_NS,
				 SCSC_PMU_WIFI_RESET_REQ_CLR,
				 SCSC_PMU_WIFI_RESET_REQ_CLR);
	if (ret)
		dev_err(scsc->dev, "failed to acknowledge WLBT reset request: %d\n",
			ret);

	return IRQ_HANDLED;
}

static void scsc_mif_destroy(struct scsc_mif_abs *interface)
{
	/* The platform device owns this interface; module dependencies pin it. */
}

static char *scsc_mif_get_uid(struct scsc_mif_abs *interface)
{
	return "gta3xlwifi";
}

static int scsc_pmu_update(struct scsc_device *scsc, u32 offset, u32 mask,
			   u32 value)
{
	int ret;

	ret = regmap_update_bits(scsc->pmu, offset, mask, value);
	if (ret)
		dev_err(scsc->dev, "PMU update 0x%x mask 0x%x failed: %d\n",
			offset, mask, ret);

	return ret;
}

static int scsc_mif_stop_locked(struct scsc_device *scsc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(SCSC_RESET_TIMEOUT_MS);
	unsigned int value;
	int ret;

	if (!scsc->wlbt_may_be_running)
		return 0;

	/* Match the downstream Exynos7885 bounded reset/quiesce sequence. */
	ret = scsc_pmu_update(scsc, SCSC_PMU_RESET_AHEAD,
			      SCSC_SYS_PWR_CFG_2, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_CLEANY_BUS,
			      SCSC_SYS_PWR_CFG, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_LOGIC_RESET,
			      SCSC_SYS_PWR_CFG_2, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_TCXO_GATE,
			      SCSC_SYS_PWR_CFG, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_DISABLE_ISO,
			      SCSC_SYS_PWR_CFG, SCSC_SYS_PWR_CFG);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_RESET_ISO,
			      SCSC_SYS_PWR_CFG, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_CENTRAL_SEQ_CONFIG,
			      SCSC_SYS_PWR_CFG_16, 0);
	if (ret)
		goto fault;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_CTRL_NS,
			      SCSC_PMU_WIFI_RESET_SET,
			      SCSC_PMU_WIFI_RESET_SET);
	if (ret)
		goto fault;

	do {
		ret = regmap_read(scsc->pmu, SCSC_PMU_CENTRAL_SEQ_STATUS,
				  &value);
		if (ret)
			goto fault;
		if (((value >> 16) & 0xff) == 0x80)
			break;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, timeout));

	if (((value >> 16) & 0xff) != 0x80) {
		ret = -ETIMEDOUT;
		goto fault;
	}
	if (scsc->shared_rail.users) {
		ret = scsc_shared_rail_put(&scsc->shared_rail);
		if (ret)
			goto fault;
	}

	/*
	 * Keep the secure grant and narrow, reserved-DRAM aperture across an
	 * inert stop. EL3 rejects a duplicate grant, while the block is held in
	 * reset and WIFI_STAT confirms it is quiescent.
	 */

	scsc->wlbt_may_be_running = false;
	scsc->stop_failed = false;
	return 0;

fault:
	scsc->stop_failed = true;
	dev_err(scsc->dev,
		"WLBT quiesce failed (%d); retaining MIF, IRQ and module ownership\n",
		ret);
	return ret;
}

static int scsc_mif_reset(struct scsc_mif_abs *interface, bool reset)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	bool need_delay = false;
	unsigned int base, size;
	int ret;

	mutex_lock(&scsc->lock);
	if (reset) {
		ret = scsc_mif_stop_locked(scsc);
		goto out_unlock;
	}
	if (!scsc->mif_mapped || scsc->wlbt_may_be_running ||
	    !scsc->memory_ready || !scsc->config_ready ||
	    !scsc->start_in_progress) {
		ret = -EHOSTDOWN;
		goto out_unlock;
	}
	base = (scsc->mem_start & 0xfffffc000ULL) >> 12;
	size = scsc->mem_size >> 12;
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_SIZE0, size);
	if (ret)
		goto out_unlock;
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_BASE0, base);
	if (ret)
		goto out_unlock;
	/* No BT ABOX aperture is reserved by this board DT. */
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_SIZE0 + 8, 0);
	if (ret)
		goto out_unlock;
	ret = regmap_write(scsc->pmu, SCSC_PMU_BAAW_BASE0 + 8, 0);
	if (ret)
		goto out_unlock;

	/* Match downstream order: map DRAM, prepare voltage, then release WLBT. */
	ret = scsc_prepare_voltage(scsc);
	if (ret)
		goto out_unlock;
	ret = scsc_shared_rail_get(&scsc->shared_rail, &need_delay);
	if (ret) {
		if (scsc->shared_rail.faulted)
			scsc->stop_failed = true;
		goto out_unlock;
	}

	/* A failed write may have partially changed power state: require stop. */
	scsc->wlbt_may_be_running = true;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_CTRL_NS,
			      SCSC_WIFI_PWRON, SCSC_WIFI_PWRON);
	if (ret)
		goto out_put_rail;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_CTRL_NS,
			      SCSC_PMU_WIFI_RESET_SET, 0);
	if (ret)
		goto out_put_rail;
	ret = scsc_pmu_update(scsc, SCSC_PMU_WIFI_CTRL_S,
			      SCSC_WIFI_START, SCSC_WIFI_START);

out_put_rail:
	{
		int rail_ret = scsc_shared_rail_put(&scsc->shared_rail);

		if (rail_ret) {
			scsc->stop_failed = true;
			if (!ret)
				ret = rail_ret;
		}
	}
out_unlock:
	mutex_unlock(&scsc->lock);
	return ret;
}

static void *scsc_mif_map(struct scsc_mif_abs *interface, size_t *allocated)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	unsigned int i;
	int ret = 0;

	if (allocated)
		*allocated = 0;

	mutex_lock(&scsc->lock);
	if (!scsc->start_armed || scsc->mif_mapped || scsc->stop_failed ||
	    !scsc->staged || !scsc->memory_ready || !scsc->config_ready ||
	    !scsc->memory) {
		ret = -EHOSTDOWN;
		goto fail;
	}
	if (!try_module_get(THIS_MODULE)) {
		ret = -ENODEV;
		goto fail;
	}
	scsc->module_pinned = true;
	scsc->start_armed = false;

	/* Match platform_mif_map(): clear mailbox state before registering IRQs. */
	for (i = 0; i < SCSC_MIF_NUM_MAILBOXES; i++) {
		writel(0, scsc->r4_mailbox[i]);
		writel(0, scsc->m4_mailbox[i]);
	}
	writel(0xffff0000, scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(0x0000ffff, scsc->r4_regs + SCSC_MIF_INTMR1);
	writel(0x0000ffff, scsc->m4_regs + SCSC_MIF_INTMR1);
	writel(0xffff0000, scsc->r4_regs + SCSC_MIF_INTCR0);
	writel(0x0000ffff, scsc->r4_regs + SCSC_MIF_INTCR1);
	writel(0x0000ffff, scsc->m4_regs + SCSC_MIF_INTCR1);
	ret = regmap_write(scsc->pmu, 0x7330, 0);
	if (ret)
		goto fail_pinned;

	scsc_mif_intr_set_active(&scsc->mif_intr, true);
	scsc->mif_mapped = true;
	scsc->start_in_progress = true;
	enable_irq(scsc->mbox_irq);
	scsc->mbox_irq_enabled = true;
	enable_irq(scsc->wdog_irq);
	scsc->wdog_irq_enabled = true;
	if (allocated)
		*allocated = scsc->mem_size;
	mutex_unlock(&scsc->lock);
	return (void __force *)scsc->memory;

fail_pinned:
	writel(0xffff0000, scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(0x0000ffff, scsc->r4_regs + SCSC_MIF_INTMR1);
	writel(0x0000ffff, scsc->m4_regs + SCSC_MIF_INTMR1);
	scsc->module_pinned = false;
	module_put(THIS_MODULE);
fail:
	dev_err(scsc->dev, "MIF map refused/failed: %d\n", ret);
	mutex_unlock(&scsc->lock);
	return NULL;
}

static void scsc_mif_unmap(struct scsc_mif_abs *interface, void *mem)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	bool put_module = false;

	mutex_lock(&scsc->lock);
	if (!scsc->mif_mapped || scsc->stop_failed) {
		mutex_unlock(&scsc->lock);
		return;
	}
	if (scsc->wlbt_may_be_running) {
		dev_err(scsc->dev, "refusing MIF unmap before WLBT quiescence\n");
		scsc->stop_failed = true;
		mutex_unlock(&scsc->lock);
		return;
	}

	writel(0xffff0000, scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(0x0000ffff, scsc->r4_regs + SCSC_MIF_INTMR1);
	writel(0x0000ffff, scsc->m4_regs + SCSC_MIF_INTMR1);
	scsc_mif_intr_set_active(&scsc->mif_intr, false);
	if (scsc->mbox_irq_enabled) {
		disable_irq(scsc->mbox_irq);
		synchronize_irq(scsc->mbox_irq);
		scsc->mbox_irq_enabled = false;
	}
	if (scsc->wdog_irq_enabled) {
		disable_irq(scsc->wdog_irq);
		synchronize_irq(scsc->wdog_irq);
		scsc->wdog_irq_enabled = false;
	}
	scsc->mif_mapped = false;
	scsc->start_in_progress = false;
	if (scsc->module_pinned) {
		scsc->module_pinned = false;
		put_module = true;
	}
	mutex_unlock(&scsc->lock);
	if (put_module)
		module_put(THIS_MODULE);
}

static u32 *scsc_mif_get_mbox_ptr(struct scsc_mif_abs *interface, u32 index)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (!READ_ONCE(scsc->mif_mapped) || index >= SCSC_MIF_NUM_MAILBOXES)
		return NULL;
	return (u32 __force *)READ_ONCE(scsc->r4_mailbox[index]);
}

static u32 scsc_mif_irq_mask_status_get(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (!READ_ONCE(scsc->mif_mapped))
		return 0;
	return readl(scsc->r4_regs + SCSC_MIF_INTMR0) >> 16;
}

static u32 scsc_mif_irq_get(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (!READ_ONCE(scsc->mif_mapped))
		return 0;
	return readl(scsc->r4_regs + SCSC_MIF_INTMSR0) >> 16;
}

static void scsc_mif_irq_bit_clear(struct scsc_mif_abs *interface, int bit)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (READ_ONCE(scsc->mif_mapped) && bit >= 0 && bit < 16)
		writel(BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTCR0);
}

static void scsc_mif_irq_bit_mask(struct scsc_mif_abs *interface, int bit)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	unsigned long flags;
	u32 value;

	if (!READ_ONCE(scsc->mif_mapped) || bit < 0 || bit >= 16)
		return;
	spin_lock_irqsave(&scsc->mif_reg_lock, flags);
	value = readl(scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(value | BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTMR0);
	spin_unlock_irqrestore(&scsc->mif_reg_lock, flags);
}

static void scsc_mif_irq_bit_unmask(struct scsc_mif_abs *interface, int bit)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	unsigned long flags;
	u32 value;

	if (!READ_ONCE(scsc->mif_mapped) || bit < 0 || bit >= 16)
		return;
	spin_lock_irqsave(&scsc->mif_reg_lock, flags);
	value = readl(scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(value & ~BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTMR0);
	spin_unlock_irqrestore(&scsc->mif_reg_lock, flags);
}

static void scsc_mif_irq_bit_set(struct scsc_mif_abs *interface, int bit,
				 enum scsc_mif_abs_target target)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	void __iomem *regs;

	if (!READ_ONCE(scsc->mif_mapped) || bit < 0 || bit >= 16)
		return;
	if (target == SCSC_MIF_ABS_TARGET_R4)
		regs = scsc->r4_regs;
	else if (target == SCSC_MIF_ABS_TARGET_M4)
		regs = scsc->m4_regs;
	else
		return;
	writel(BIT(bit), regs + SCSC_MIF_INTGR1);
}

static void scsc_mif_irq_reg_handler(struct scsc_mif_abs *interface,
				     void (*handler)(int irq, void *data),
				     void *data)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_irq_data, data);
	WRITE_ONCE(scsc->mif_irq_handler, handler);
}

static void scsc_mif_irq_unreg_handler(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_irq_handler, NULL);
	if (scsc->mbox_irq_enabled)
		synchronize_irq(scsc->mbox_irq);
	WRITE_ONCE(scsc->mif_irq_data, NULL);
}

static void scsc_mif_irq_clear(void)
{
}

static void scsc_mif_noop_interface(struct scsc_mif_abs *interface)
{
}

static void scsc_mif_reg_reset_handler(struct scsc_mif_abs *interface,
				       void (*handler)(int irq, void *data),
				       void *data)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_reset_data, data);
	WRITE_ONCE(scsc->mif_reset_handler, handler);
	if (handler && scsc->mif_mapped && !scsc->wdog_irq_enabled) {
		enable_irq(scsc->wdog_irq);
		WRITE_ONCE(scsc->wdog_irq_enabled, true);
	}
}

static void scsc_mif_unreg_reset_handler(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_reset_handler, NULL);
	if (scsc->wdog_irq_enabled)
		synchronize_irq(scsc->wdog_irq);
	WRITE_ONCE(scsc->mif_reset_data, NULL);
}

static void scsc_mif_suspend_reg_handler(struct scsc_mif_abs *interface,
					 int (*suspend)(struct scsc_mif_abs *, void *),
					 void (*resume)(struct scsc_mif_abs *, void *),
					 void *data)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_suspend_data, data);
	WRITE_ONCE(scsc->mif_suspend, suspend);
	WRITE_ONCE(scsc->mif_resume, resume);
}

static void scsc_mif_suspend_unreg_handler(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	WRITE_ONCE(scsc->mif_suspend, NULL);
	WRITE_ONCE(scsc->mif_resume, NULL);
	WRITE_ONCE(scsc->mif_suspend_data, NULL);
}

static void *scsc_mif_get_ram_ptr(struct scsc_mif_abs *interface,
				  scsc_mifram_ref ref)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (!READ_ONCE(scsc->mif_mapped) || ref < 0 ||
	    (size_t)ref >= scsc->mem_size)
		return NULL;
	return (void __force *)(scsc->memory + ref);
}

static int scsc_mif_get_ram_ref(struct scsc_mif_abs *interface, void *ptr,
				scsc_mifram_ref *ref)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	uintptr_t start, end, address;

	if (!ref || !ptr || !READ_ONCE(scsc->mif_mapped))
		return -EHOSTDOWN;
	start = (uintptr_t)scsc->memory;
	address = (uintptr_t)ptr;
	if (check_add_overflow(start, scsc->mem_size, &end) ||
	    address < start || address >= end || address - start > S32_MAX)
		return -ERANGE;
	*ref = (scsc_mifram_ref)(address - start);
	return 0;
}

static uintptr_t scsc_mif_get_ram_pfn(struct scsc_mif_abs *interface)
{
	struct scsc_device *scsc = scsc_from_mif(interface);

	if (!READ_ONCE(scsc->mif_mapped) || !scsc->memory)
		return 0;
	return vmalloc_to_pfn((void __force *)scsc->memory);
}

static void *scsc_mif_get_ram_phy_ptr(struct scsc_mif_abs *interface,
				      scsc_mifram_ref ref)
{
	struct scsc_device *scsc = scsc_from_mif(interface);
	phys_addr_t address;

	if (!READ_ONCE(scsc->mif_mapped) || ref < 0 ||
	    (size_t)ref >= scsc->mem_size ||
	    check_add_overflow(scsc->mem_start, (phys_addr_t)ref, &address))
		return NULL;
	return (void *)(uintptr_t)address;
}

static struct device *scsc_mif_get_device(struct scsc_mif_abs *interface)
{
	return scsc_from_mif(interface)->dev;
}

static void scsc_mif_get_abox_shared_mem(struct scsc_mif_abs *interface,
					void **data)
{
	if (data)
		*data = NULL;
}

static const struct scsc_mif_abs scsc_mif_template = {
	.destroy = scsc_mif_destroy,
	.get_uid = scsc_mif_get_uid,
	.reset = scsc_mif_reset,
	.map = scsc_mif_map,
	.unmap = scsc_mif_unmap,
	.get_mbox_ptr = scsc_mif_get_mbox_ptr,
	.irq_bit_mask_status_get = scsc_mif_irq_mask_status_get,
	.irq_get = scsc_mif_irq_get,
	.irq_bit_clear = scsc_mif_irq_bit_clear,
	.irq_bit_mask = scsc_mif_irq_bit_mask,
	.irq_bit_unmask = scsc_mif_irq_bit_unmask,
	.irq_bit_set = scsc_mif_irq_bit_set,
	.irq_reg_handler = scsc_mif_irq_reg_handler,
	.irq_unreg_handler = scsc_mif_irq_unreg_handler,
	.irq_clear = scsc_mif_irq_clear,
	.irq_reg_reset_request_handler = scsc_mif_reg_reset_handler,
	.irq_unreg_reset_request_handler = scsc_mif_unreg_reset_handler,
	.suspend_reg_handler = scsc_mif_suspend_reg_handler,
	.suspend_unreg_handler = scsc_mif_suspend_unreg_handler,
	.get_mifram_ptr = scsc_mif_get_ram_ptr,
	.get_mifram_ref = scsc_mif_get_ram_ref,
	.get_mifram_pfn = scsc_mif_get_ram_pfn,
	.get_mifram_phy_ptr = scsc_mif_get_ram_phy_ptr,
	.get_mif_device = scsc_mif_get_device,
	.mif_dump_registers = scsc_mif_noop_interface,
	.mif_cleanup = scsc_mif_noop_interface,
	.mif_restart = scsc_mif_noop_interface,
	.get_abox_shared_mem = scsc_mif_get_abox_shared_mem,
};

static void scsc_mif_notify_probe(struct scsc_device *scsc)
{
	mutex_lock(&scsc_mif_registry_lock);
	scsc_mif_device = scsc;
	if (scsc_mif_client) {
		dev_info(scsc->dev, "probing downstream MIF client %s; firmware start remains disabled\n",
			 scsc_mif_client->name);
		scsc_mif_client->probe(scsc_mif_client, &scsc->mif_abs);
	}
	mutex_unlock(&scsc_mif_registry_lock);
}

static void scsc_mif_notify_remove(struct scsc_device *scsc)
{
	mutex_lock(&scsc_mif_registry_lock);
	if (scsc_mif_device == scsc) {
		if (scsc_mif_client)
			scsc_mif_client->remove(&scsc->mif_abs);
		scsc_mif_device = NULL;
	}
	mutex_unlock(&scsc_mif_registry_lock);
}

void scsc_mif_abs_register(struct scsc_mif_abs_driver *driver)
{
	mutex_lock(&scsc_mif_registry_lock);
	if (scsc_mif_client) {
		pr_err("exynos-scsc-wifibt: MIF client %s already registered\n",
		       scsc_mif_client->name);
		mutex_unlock(&scsc_mif_registry_lock);
		return;
	}
	if (!driver || !driver->name || !driver->probe || !driver->remove) {
		pr_err("exynos-scsc-wifibt: invalid MIF client registration\n");
		mutex_unlock(&scsc_mif_registry_lock);
		return;
	}
	scsc_mif_client = driver;
	if (scsc_mif_device) {
		dev_info(scsc_mif_device->dev,
			 "registered downstream MIF client %s; firmware start remains disabled\n",
			 driver->name);
		driver->probe(driver, &scsc_mif_device->mif_abs);
	}
	mutex_unlock(&scsc_mif_registry_lock);
}
EXPORT_SYMBOL_GPL(scsc_mif_abs_register);

void scsc_mif_abs_unregister(struct scsc_mif_abs_driver *driver)
{
	mutex_lock(&scsc_mif_registry_lock);
	if (scsc_mif_client == driver) {
		/* The core destroys its devices before unregistering this callback. */
		scsc_mif_client = NULL;
	}
	mutex_unlock(&scsc_mif_registry_lock);
}
EXPORT_SYMBOL_GPL(scsc_mif_abs_unregister);

/* Resolve ACPM lazily so an inert module bind does not require ACPM to load. */
static int scsc_prepare_voltage(struct scsc_device *scsc)
{
	struct device_node *np;
	int ret;

	if (scsc->voltage_request_attempted && !scsc->voltage_prepared)
		return -EALREADY;

	if (!scsc->acpm) {
		np = of_find_compatible_node(NULL, NULL,
					     "samsung,exynos7885-acpm-ipc");
		if (!np)
			return -ENODEV;
		scsc->acpm = devm_acpm_get_by_node(scsc->dev, np);
		of_node_put(np);
		if (IS_ERR(scsc->acpm)) {
			ret = PTR_ERR(scsc->acpm);
			scsc->acpm = NULL;
			return ret;
		}
	}

	if (!scsc->acpm->ops.dvfs_ops.set_wlbt_flag)
		return -EOPNOTSUPP;

	/* A failed transfer may have left a request queued; never retry it here. */
	scsc->voltage_request_attempted = true;
	ret = scsc->acpm->ops.dvfs_ops.set_wlbt_flag(scsc->acpm);
	scsc->voltage_prepared = !ret;
	return ret;
}

static ssize_t prepare_voltage_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&scsc->lock);
	len = sysfs_emit(buf, "attempted=%u prepared=%u\n",
			 scsc->voltage_request_attempted,
			 scsc->voltage_prepared);
	mutex_unlock(&scsc->lock);

	return len;
}

static ssize_t prepare_voltage_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	ret = scsc_prepare_voltage(scsc);
	mutex_unlock(&scsc->lock);
	if (ret)
		return ret;

	dev_info(dev, "ACPM WLBT voltage preparation completed\n");
	return count;
}
static DEVICE_ATTR_ADMIN_RW(prepare_voltage);

static u32 scsc_mif_get_pending(void *context)
{
	struct scsc_device *scsc = context;

	return readl(scsc->r4_regs + SCSC_MIF_INTMSR0) >> 16;
}

static void scsc_mif_mask_to_host(void *context, unsigned int bit)
{
	struct scsc_device *scsc = context;
	unsigned long flags;
	u32 mask;

	spin_lock_irqsave(&scsc->mif_reg_lock, flags);
	mask = readl(scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(mask | BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTMR0);
	spin_unlock_irqrestore(&scsc->mif_reg_lock, flags);
}

static void scsc_mif_clear_to_host(void *context, unsigned int bit)
{
	struct scsc_device *scsc = context;

	writel(BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTCR0);
}

static void scsc_mif_unmask_to_host(void *context, unsigned int bit)
{
	struct scsc_device *scsc = context;
	unsigned long flags;
	u32 mask;

	spin_lock_irqsave(&scsc->mif_reg_lock, flags);
	mask = readl(scsc->r4_regs + SCSC_MIF_INTMR0);
	writel(mask & ~BIT(bit + 16), scsc->r4_regs + SCSC_MIF_INTMR0);
	spin_unlock_irqrestore(&scsc->mif_reg_lock, flags);
}

static void scsc_mif_raise_from_host(void *context,
				     enum scsc_mif_target target,
				     unsigned int bit)
{
	struct scsc_device *scsc = context;
	void __iomem *regs;

	regs = target == SCSC_MIF_TARGET_R4 ? scsc->r4_regs : scsc->m4_regs;
	writel(BIT(bit), regs + SCSC_MIF_INTGR1);
}

static const struct scsc_mif_intr_ops scsc_mif_intr_ops = {
	.get_pending = scsc_mif_get_pending,
	.mask_to_host = scsc_mif_mask_to_host,
	.clear_to_host = scsc_mif_clear_to_host,
	.unmask_to_host = scsc_mif_unmask_to_host,
	.raise_from_host = scsc_mif_raise_from_host,
};

static int scsc_shared_set_cp_wakeup(void *context, bool enable)
{
	struct scsc_device *scsc = context;
	u32 value;

	if (!scsc->cp_mailbox)
		return -ENODEV;

	value = readl(scsc->cp_mailbox + SCSC_CP_ISSR2_OFFSET);
	if (enable)
		value |= SCSC_CP_WAKEUP_BIT;
	else
		value &= ~SCSC_CP_WAKEUP_BIT;
	writel(value, scsc->cp_mailbox + SCSC_CP_ISSR2_OFFSET);
	readl(scsc->cp_mailbox + SCSC_CP_ISSR2_OFFSET);

	return 0;
}

static int scsc_shared_set_option(void *context, bool enable)
{
	struct scsc_device *scsc = context;

	return regmap_update_bits(scsc->pmu, SCSC_PMU_SHARED_REG_OPTION,
				  SCSC_PMU_SHARED_REG_OPTION_BIT,
				  enable ? SCSC_PMU_SHARED_REG_OPTION_BIT : 0);
}

static int scsc_shared_get_option(void *context, bool *enabled)
{
	struct scsc_device *scsc = context;
	unsigned int value;
	int ret;

	ret = regmap_read(scsc->pmu, SCSC_PMU_SHARED_REG_OPTION, &value);
	if (!ret)
		*enabled = !!(value & SCSC_PMU_SHARED_REG_OPTION_BIT);

	return ret;
}

static int scsc_shared_cp_ready(void *context, bool *ready)
{
	struct scsc_device *scsc = context;
	u32 value;

	if (!scsc->cp_mailbox)
		return -ENODEV;

	value = readl(scsc->cp_mailbox + SCSC_CP_ISSR3_OFFSET);
	*ready = !!(value & GENMASK(4, 1));

	return 0;
}

static const struct scsc_shared_rail_ops scsc_shared_rail_ops = {
	.set_cp_wakeup = scsc_shared_set_cp_wakeup,
	.get_option = scsc_shared_get_option,
	.set_option = scsc_shared_set_option,
	.cp_ready = scsc_shared_cp_ready,
};

static bool enable;
module_param(enable, bool, 0444);
	MODULE_PARM_DESC(enable, "Explicitly enable binding; firmware start remains one-shot gated");

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
	if ((ctrl_s & SCSC_WIFI_START) || status ||
	    ((ctrl_ns & SCSC_WIFI_PWRON) &&
	     !(ctrl_ns & SCSC_PMU_WIFI_RESET_SET))) {
		ret = -EBUSY;
		goto out_unlock;
	}
	if (!IS_ALIGNED(scsc->mem_start, SZ_4K) ||
	    !IS_ALIGNED(scsc->mem_size, SZ_4K) || scsc->mem_size > SZ_1G) {
		ret = -EINVAL;
		goto out_unlock;
	}

	encoded_base = (scsc->mem_start & 0xfffffc000ULL) >> 12;
	ret = regmap_read(scsc->pmu, SCSC_PMU_BAAW_SIZE0, &size);
	if (ret)
		goto out_unlock;
	ret = regmap_read(scsc->pmu, SCSC_PMU_BAAW_BASE0, &base);
	if (ret)
		goto out_unlock;
	/*
	 * TZASC grants survive an inert module unload. On the same boot, the
	 * exact reserved-window BAAW values are the persistent marker that its
	 * secure grant was already completed; repeating that SMC is rejected by
	 * EL3. Still require the WLAN power/reset state above to be quiescent.
	 */
	if (size == scsc->mem_size >> 12 && base == encoded_base) {
		scsc->memory_ready = true;
		dev_info(dev, "reusing prepared reserved DRAM aperture at %pa size %zu\n",
			 &scsc->mem_start, scsc->mem_size);
		ret = 0;
		goto out_unlock;
	}

	/* Match downstream's WLBT TZASC call, but fail closed on its result. */
	arm_smccc_smc(SCSC_TZASC_SMC, SCSC_TZASC_WLBT, scsc->mem_start,
		      scsc->mem_size, 0, 0, 0, 0, &res);
	if ((long)res.a0) {
		ret = (long)res.a0 < 0 ? (long)res.a0 : -EIO;
		goto out_unlock;
	}

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
				 "result=%d runtime=%u entry=0x%x staged=%u memory_ready=%u config_ready=%u config_offset=0x%x voltage_attempted=%u voltage_prepared=%u executed=%u\n",
				 scsc->firmware_result, scsc->runtime_length,
				 scsc->entry_point, scsc->staged, scsc->memory_ready,
				 scsc->config_ready, scsc->config_offset,
				 scsc->voltage_request_attempted,
				 scsc->voltage_prepared,
				 scsc->wlbt_may_be_running);
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

static ssize_t shared_rail_state_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	unsigned int cp_status, shared_status;
	int ret;

	ret = regmap_read(scsc->pmu, SCSC_PMU_CP_STATUS, &cp_status);
	if (ret)
		return ret;
	ret = regmap_read(scsc->pmu, SCSC_PMU_SHARED_REG_STATUS,
			  &shared_status);
	if (ret)
		return ret;

	return sysfs_emit(buf,
			  "cp_mailbox=%s cp_status=0x%x cp_status_matches_reference=%u shared_status=0x%x shared_status_matches_reference=%u\n",
			  scsc->cp_mailbox ? "mapped" : "missing", cp_status,
			  cp_status == SCSC_PMU_CP_REFERENCE, shared_status,
			  shared_status == SCSC_PMU_SHARED_REG_REFERENCE);
}
static DEVICE_ATTR_ADMIN_RO(shared_rail_state);

static ssize_t cp_mailbox_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	u32 issr2, issr3;

	if (!scsc->cp_mailbox)
		return -ENODEV;

	/* Explicit, bounded read of only the two downstream handshake ISSRs. */
	issr2 = readl(scsc->cp_mailbox + SCSC_CP_ISSR2_OFFSET);
	issr3 = readl(scsc->cp_mailbox + SCSC_CP_ISSR3_OFFSET);

	return sysfs_emit(buf, "issr2=0x%08x wakeup=%u issr3=0x%08x ready=%u\n",
			  issr2, !!(issr2 & SCSC_CP_WAKEUP_BIT), issr3,
			  !!(issr3 & GENMASK(4, 1)));
}
static DEVICE_ATTR_ADMIN_RO(cp_mailbox_state);

static ssize_t shared_rail_selftest_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int ret = scsc_shared_rail_selftest();

	if (ret)
		return ret;

	return sysfs_emit(buf, "pass\n");
}
static DEVICE_ATTR_ADMIN_RO(shared_rail_selftest);

static ssize_t mif_intr_selftest_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret = scsc_mif_intr_selftest();

	if (ret)
		return ret;

	return sysfs_emit(buf, "pass\n");
}
static DEVICE_ATTR_ADMIN_RO(mif_intr_selftest);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	const char *state;

	mutex_lock(&scsc->lock);
	if (scsc->stop_failed)
		state = "stop-failed";
	else if (scsc->wlbt_may_be_running)
		state = "running-or-starting";
	else if (scsc->mif_mapped)
		state = "mapped";
	else if (scsc->start_armed)
		state = "armed";
	else
		state = "inert";
	mutex_unlock(&scsc->lock);

	return sysfs_emit(buf, "%s\n", state);
}
static DEVICE_ATTR_RO(state);

static ssize_t arm_start_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	int ret = 0;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	if (scsc->stop_failed || scsc->start_armed || scsc->mif_mapped ||
	    scsc->wlbt_may_be_running) {
		ret = -EBUSY;
		goto out_unlock;
	}
	if (!scsc->staged || !scsc->memory_ready || !scsc->config_ready) {
		ret = -EINVAL;
		goto out_unlock;
	}
	scsc->start_armed = true;
	dev_warn(dev,
		 "one-shot WLBT start armed; next downstream service open will power firmware\n");
out_unlock:
	mutex_unlock(&scsc->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(arm_start);

/* Recover only a stale, unmapped WLBT state after a host-side reboot. */
static ssize_t quiesce_stale_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct scsc_device *scsc = dev_get_drvdata(dev);
	unsigned int ctrl_ns, ctrl_s, status;
	bool put_module = false;
	int ret = 0;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;

	mutex_lock(&scsc->lock);
	if (scsc->mif_mapped || scsc->start_armed) {
		ret = -EBUSY;
		goto out_unlock;
	}
	if (!scsc->wlbt_may_be_running) {
		ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_CTRL_NS, &ctrl_ns);
		if (ret)
			goto out_unlock;
		ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_CTRL_S, &ctrl_s);
		if (ret)
			goto out_unlock;
		ret = regmap_read(scsc->pmu, SCSC_PMU_WIFI_STAT, &status);
		if (ret)
			goto out_unlock;
		if (!(ctrl_ns & SCSC_WIFI_PWRON) &&
		    !(ctrl_s & SCSC_WIFI_START) && !status) {
			ret = -EALREADY;
			goto out_unlock;
		}
		if (!try_module_get(THIS_MODULE)) {
			ret = -ENODEV;
			goto out_unlock;
		}
		scsc->module_pinned = true;
		scsc->wlbt_may_be_running = true;
	}

	ret = scsc_mif_stop_locked(scsc);
	if (!ret && scsc->module_pinned) {
		scsc->module_pinned = false;
		put_module = true;
	}
out_unlock:
	mutex_unlock(&scsc->lock);
	if (put_module)
		module_put(THIS_MODULE);
	if (!ret)
		dev_info(dev, "stale WLBT state quiesced and held in reset\n");
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(quiesce_stale);

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
	&dev_attr_prepare_voltage.attr,
	&dev_attr_arm_start.attr,
	&dev_attr_quiesce_stale.attr,
	&dev_attr_firmware_status.attr,
	&dev_attr_pmu_state.attr,
	&dev_attr_shared_rail_state.attr,
	&dev_attr_cp_mailbox_state.attr,
	&dev_attr_shared_rail_selftest.attr,
	&dev_attr_mif_intr_selftest.attr,
	NULL,
};
ATTRIBUTE_GROUPS(scsc);

static int scsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scsc_device *scsc;
	struct device_node *np;
	struct resource mem, r4_regs, m4_regs, cp_regs;
	ktime_t start = ktime_get();
	int ret, i;

	if (!enable)
		return -ENODEV;
	scsc = devm_kzalloc(dev, sizeof(*scsc), GFP_KERNEL);
	if (!scsc)
		return -ENOMEM;
	mutex_init(&scsc->lock);
	scsc->dev = dev;
	scsc->mif_abs = scsc_mif_template;
	spin_lock_init(&scsc->mif_reg_lock);
	scsc->mbox_irq = platform_get_irq_byname(pdev, "mbox");
	if (scsc->mbox_irq < 0)
		return dev_err_probe(dev, scsc->mbox_irq,
				     "failed to get mailbox IRQ\n");
	scsc->wdog_irq = platform_get_irq_byname(pdev, "watchdog");
	if (scsc->wdog_irq < 0)
		return dev_err_probe(dev, scsc->wdog_irq,
				     "failed to get watchdog IRQ\n");
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
	scsc->r4_regs = devm_platform_ioremap_resource_byname(pdev, "r4");
	if (IS_ERR(scsc->r4_regs))
		return PTR_ERR(scsc->r4_regs);
	scsc->m4_regs = devm_platform_ioremap_resource_byname(pdev, "m4");
	if (IS_ERR(scsc->m4_regs))
		return PTR_ERR(scsc->m4_regs);
	ret = scsc_mif_intr_init(&scsc->mif_intr, &scsc_mif_intr_ops, scsc);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, scsc->mbox_irq, scsc_mbox_irq,
			       IRQF_NO_AUTOEN, dev_name(dev), scsc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to reserve inactive mailbox IRQ\n");
	ret = devm_request_threaded_irq(dev, scsc->wdog_irq, NULL,
					scsc_wdog_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), scsc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to reserve inactive watchdog IRQ\n");
	for (i = 0; i < SCSC_MIF_NUM_MAILBOXES; i++) {
		scsc->r4_mailbox[i] = scsc_mailbox_slot(scsc->r4_regs,
						       scsc->r4_reg_size, i);
		scsc->m4_mailbox[i] = scsc_mailbox_slot(scsc->m4_regs,
						       scsc->m4_reg_size, i);
		if (!scsc->r4_mailbox[i] || !scsc->m4_mailbox[i])
			return -EINVAL;
	}

	/* Required for an active Exynos7885 shared-regulator handshake. */
	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "cp")) {
		cp_regs = *platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"cp");
		if (resource_size(&cp_regs) <
		    SCSC_CP_ISSR3_OFFSET + sizeof(u32))
			return -EINVAL;
		scsc->cp_mailbox = devm_platform_ioremap_resource_byname(pdev,
									"cp");
		if (IS_ERR(scsc->cp_mailbox))
			return PTR_ERR(scsc->cp_mailbox);
	}
	if (!scsc->cp_mailbox)
		return dev_err_probe(dev, -EINVAL,
				     "missing CP mailbox needed by WLBT rail sequence\n");
	ret = scsc_shared_rail_init(&scsc->shared_rail,
				   &scsc_shared_rail_ops, scsc);
	if (ret)
		return ret;

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
	ret = scsc_map_reserved_memory(scsc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to map reserved WLBT DRAM\n");

	/* Bind only; firmware staging is a separate explicit sysfs operation. */
	dev_info(dev,
		 "inert: reserved %pr, mapped %u R4/M4 slots and reserved inactive MBOX/watchdog IRQs %d/%d in %lld us; no MMIO access or firmware execution\n",
		 &mem, SCSC_MIF_NUM_MAILBOXES,
		 scsc->mbox_irq, scsc->wdog_irq,
		 ktime_us_delta(ktime_get(), start));
	scsc_mif_notify_probe(scsc);
	return 0;
}

static void scsc_remove(struct platform_device *pdev)
{
	struct scsc_device *scsc = platform_get_drvdata(pdev);

	scsc_mif_notify_remove(scsc);
}

static const struct of_device_id scsc_of_match[] = {
	{ .compatible = "samsung,exynos7885-wifibt" },
	{ }
};
/* Deliberately no MODULE_DEVICE_TABLE: this experiment must not autoload. */

static struct platform_driver scsc_driver = {
	.probe = scsc_probe,
	.remove = scsc_remove,
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
