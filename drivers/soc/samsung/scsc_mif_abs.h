/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SCSC_MIF_ABS_H
#define __SCSC_MIF_ABS_H

#include <linux/device.h>
#include <linux/types.h>

typedef s32 scsc_mifram_ref;

enum scsc_mif_abs_target {
	SCSC_MIF_ABS_TARGET_R4 = 0,
	SCSC_MIF_ABS_TARGET_M4 = 1,
};

struct scsc_mif_abs {
	void (*destroy)(struct scsc_mif_abs *interface);
	char *(*get_uid)(struct scsc_mif_abs *interface);
	int (*reset)(struct scsc_mif_abs *interface, bool reset);
	void *(*map)(struct scsc_mif_abs *interface, size_t *allocated);
	void (*unmap)(struct scsc_mif_abs *interface, void *mem);
	u32 *(*get_mbox_ptr)(struct scsc_mif_abs *interface, u32 mbox_index);
	u32 (*irq_bit_mask_status_get)(struct scsc_mif_abs *interface);
	u32 (*irq_get)(struct scsc_mif_abs *interface);
	void (*irq_bit_clear)(struct scsc_mif_abs *interface, int bit_num);
	void (*irq_bit_mask)(struct scsc_mif_abs *interface, int bit_num);
	void (*irq_bit_unmask)(struct scsc_mif_abs *interface, int bit_num);
	void (*irq_bit_set)(struct scsc_mif_abs *interface, int bit_num,
			    enum scsc_mif_abs_target target);
	void (*irq_reg_handler)(struct scsc_mif_abs *interface,
				void (*handler)(int irq, void *data), void *dev);
	void (*irq_unreg_handler)(struct scsc_mif_abs *interface);
	void (*irq_clear)(void);
	void (*irq_reg_reset_request_handler)(struct scsc_mif_abs *interface,
					      void (*handler)(int irq, void *data),
					      void *dev);
	void (*irq_unreg_reset_request_handler)(struct scsc_mif_abs *interface);
	void (*suspend_reg_handler)(struct scsc_mif_abs *interface,
				    int (*suspend)(struct scsc_mif_abs *, void *),
				    void (*resume)(struct scsc_mif_abs *, void *),
				    void *data);
	void (*suspend_unreg_handler)(struct scsc_mif_abs *interface);
	void *(*get_mifram_ptr)(struct scsc_mif_abs *interface,
				scsc_mifram_ref ref);
	int (*get_mifram_ref)(struct scsc_mif_abs *interface, void *ptr,
			      scsc_mifram_ref *ref);
	uintptr_t (*get_mifram_pfn)(struct scsc_mif_abs *interface);
	void *(*get_mifram_phy_ptr)(struct scsc_mif_abs *interface,
				    scsc_mifram_ref ref);
	struct device *(*get_mif_device)(struct scsc_mif_abs *interface);
	void (*mif_dump_registers)(struct scsc_mif_abs *interface);
	void (*mif_cleanup)(struct scsc_mif_abs *interface);
	void (*mif_restart)(struct scsc_mif_abs *interface);
	void (*get_abox_shared_mem)(struct scsc_mif_abs *interface, void **data);
};

struct scsc_mif_abs_driver {
	char *name;
	void (*probe)(struct scsc_mif_abs_driver *driver,
		      struct scsc_mif_abs *interface);
	void (*remove)(struct scsc_mif_abs *interface);
};

void scsc_mif_abs_register(struct scsc_mif_abs_driver *driver);
void scsc_mif_abs_unregister(struct scsc_mif_abs_driver *driver);

#endif /* __SCSC_MIF_ABS_H */
