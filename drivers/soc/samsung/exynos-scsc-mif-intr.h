/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __EXYNOS_SCSC_MIF_INTR_H
#define __EXYNOS_SCSC_MIF_INTR_H

#include <linux/bitmap.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define SCSC_MIF_INTR_COUNT	16

/*
 * Interrupt-controller registers in each Exynos7885 WLBT mailbox bank.
 * The driver accesses these only after the explicit MIF-active transition.
 */
#define SCSC_MIF_INTGR1		0x01c
#define SCSC_MIF_INTCR0		0x00c
#define SCSC_MIF_INTMR0		0x010
#define SCSC_MIF_INTMSR0	0x018

enum scsc_mif_target {
	SCSC_MIF_TARGET_R4,
	SCSC_MIF_TARGET_M4,
};

typedef void (*scsc_mif_intr_handler_t)(unsigned int bit, void *data);

struct scsc_mif_intr_ops {
	u32 (*get_pending)(void *context);
	void (*mask_to_host)(void *context, unsigned int bit);
	void (*clear_to_host)(void *context, unsigned int bit);
	void (*unmask_to_host)(void *context, unsigned int bit);
	void (*raise_from_host)(void *context, enum scsc_mif_target target,
				unsigned int bit);
};

struct scsc_mif_intr_slot {
	scsc_mif_intr_handler_t handler;
	void *data;
};

struct scsc_mif_intr {
	const struct scsc_mif_intr_ops *ops;
	void *context;
	struct scsc_mif_intr_slot slot[SCSC_MIF_INTR_COUNT];
	DECLARE_BITMAP(to_host, SCSC_MIF_INTR_COUNT);
	DECLARE_BITMAP(from_host_r4, SCSC_MIF_INTR_COUNT);
	DECLARE_BITMAP(from_host_m4, SCSC_MIF_INTR_COUNT);
	spinlock_t lock;
	bool active;
};

int scsc_mif_intr_init(struct scsc_mif_intr *intr,
		       const struct scsc_mif_intr_ops *ops, void *context);
void scsc_mif_intr_deinit(struct scsc_mif_intr *intr);
void scsc_mif_intr_set_active(struct scsc_mif_intr *intr, bool active);
int scsc_mif_intr_alloc_to_host(struct scsc_mif_intr *intr,
				scsc_mif_intr_handler_t handler, void *data);
int scsc_mif_intr_free_to_host(struct scsc_mif_intr *intr, int bit);
int scsc_mif_intr_alloc_from_host(struct scsc_mif_intr *intr,
				  enum scsc_mif_target target);
int scsc_mif_intr_free_from_host(struct scsc_mif_intr *intr, int bit,
				 enum scsc_mif_target target);
int scsc_mif_intr_raise(struct scsc_mif_intr *intr, unsigned int bit,
			enum scsc_mif_target target);
void scsc_mif_intr_ack(struct scsc_mif_intr *intr, unsigned int bit);
int scsc_mif_intr_dispatch(struct scsc_mif_intr *intr);
int scsc_mif_intr_selftest(void);

#endif /* __EXYNOS_SCSC_MIF_INTR_H */
