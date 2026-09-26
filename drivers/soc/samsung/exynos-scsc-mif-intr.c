// SPDX-License-Identifier: GPL-2.0-only
/* Software interrupt-bit management for the Exynos7885 SCSC MIF. */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/string.h>

#include "exynos-scsc-mif-intr.h"

static void scsc_mif_intr_default_handler(unsigned int bit, void *data)
{
	struct scsc_mif_intr *intr = data;

	scsc_mif_intr_ack(intr, bit);
}

int scsc_mif_intr_init(struct scsc_mif_intr *intr,
		       const struct scsc_mif_intr_ops *ops, void *context)
{
	unsigned int bit;

	if (!intr || !ops || !ops->get_pending || !ops->mask_to_host ||
	    !ops->clear_to_host || !ops->unmask_to_host ||
	    !ops->raise_from_host)
		return -EINVAL;

	memset(intr, 0, sizeof(*intr));
	intr->ops = ops;
	intr->context = context;
	spin_lock_init(&intr->lock);
	bitmap_zero(intr->to_host, SCSC_MIF_INTR_COUNT);
	bitmap_zero(intr->from_host_r4, SCSC_MIF_INTR_COUNT);
	bitmap_zero(intr->from_host_m4, SCSC_MIF_INTR_COUNT);
	set_bit(0, intr->from_host_r4);
	set_bit(0, intr->from_host_m4);
	for (bit = 0; bit < SCSC_MIF_INTR_COUNT; bit++) {
		intr->slot[bit].handler = scsc_mif_intr_default_handler;
		intr->slot[bit].data = intr;
	}

	return 0;
}

void scsc_mif_intr_deinit(struct scsc_mif_intr *intr)
{
	unsigned long flags;
	unsigned int bit;

	spin_lock_irqsave(&intr->lock, flags);
	intr->active = false;
	for (bit = 0; bit < SCSC_MIF_INTR_COUNT; bit++) {
		intr->slot[bit].handler = scsc_mif_intr_default_handler;
		intr->slot[bit].data = intr;
	}
	bitmap_zero(intr->to_host, SCSC_MIF_INTR_COUNT);
	bitmap_zero(intr->from_host_r4, SCSC_MIF_INTR_COUNT);
	bitmap_zero(intr->from_host_m4, SCSC_MIF_INTR_COUNT);
	set_bit(0, intr->from_host_r4);
	set_bit(0, intr->from_host_m4);
	spin_unlock_irqrestore(&intr->lock, flags);
}

void scsc_mif_intr_set_active(struct scsc_mif_intr *intr, bool active)
{
	unsigned long flags;

	spin_lock_irqsave(&intr->lock, flags);
	intr->active = active;
	spin_unlock_irqrestore(&intr->lock, flags);
}

int scsc_mif_intr_alloc_to_host(struct scsc_mif_intr *intr,
				scsc_mif_intr_handler_t handler, void *data)
{
	unsigned long flags;
	unsigned int bit;

	if (!intr || !handler)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	if (!intr->active) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -EHOSTDOWN;
	}
	bit = find_first_zero_bit(intr->to_host, SCSC_MIF_INTR_COUNT);
	if (bit >= SCSC_MIF_INTR_COUNT) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -ENOSPC;
	}

	intr->ops->mask_to_host(intr->context, bit);
	intr->ops->clear_to_host(intr->context, bit);
	intr->slot[bit].handler = handler;
	intr->slot[bit].data = data;
	intr->ops->unmask_to_host(intr->context, bit);
	set_bit(bit, intr->to_host);
	spin_unlock_irqrestore(&intr->lock, flags);

	return bit;
}

int scsc_mif_intr_free_to_host(struct scsc_mif_intr *intr, int bit)
{
	unsigned long flags;

	if (!intr || bit < 0 || bit >= SCSC_MIF_INTR_COUNT)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	if (!test_bit(bit, intr->to_host)) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -ENOENT;
	}

	if (intr->active) {
		intr->ops->mask_to_host(intr->context, bit);
		intr->ops->clear_to_host(intr->context, bit);
	}
	intr->slot[bit].handler = scsc_mif_intr_default_handler;
	intr->slot[bit].data = intr;
	clear_bit(bit, intr->to_host);
	spin_unlock_irqrestore(&intr->lock, flags);

	return 0;
}

static unsigned long *scsc_mif_intr_from_host_bitmap(
		struct scsc_mif_intr *intr, enum scsc_mif_target target)
{
	switch (target) {
	case SCSC_MIF_TARGET_R4:
		return intr->from_host_r4;
	case SCSC_MIF_TARGET_M4:
		return intr->from_host_m4;
	default:
		return NULL;
	}
}

int scsc_mif_intr_alloc_from_host(struct scsc_mif_intr *intr,
				  enum scsc_mif_target target)
{
	unsigned long flags;
	unsigned long *bitmap;
	unsigned int bit;

	if (!intr)
		return -EINVAL;

	bitmap = scsc_mif_intr_from_host_bitmap(intr, target);
	if (!bitmap)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	bit = find_first_zero_bit(bitmap, SCSC_MIF_INTR_COUNT);
	if (bit >= SCSC_MIF_INTR_COUNT) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -ENOSPC;
	}
	set_bit(bit, bitmap);
	spin_unlock_irqrestore(&intr->lock, flags);

	return bit;
}

int scsc_mif_intr_free_from_host(struct scsc_mif_intr *intr, int bit,
				 enum scsc_mif_target target)
{
	unsigned long flags;
	unsigned long *bitmap;

	if (!intr || bit < 0 || bit >= SCSC_MIF_INTR_COUNT)
		return -EINVAL;

	bitmap = scsc_mif_intr_from_host_bitmap(intr, target);
	if (!bitmap)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	if (!test_bit(bit, bitmap) || bit == 0) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -ENOENT;
	}
	clear_bit(bit, bitmap);
	spin_unlock_irqrestore(&intr->lock, flags);

	return 0;
}

int scsc_mif_intr_raise(struct scsc_mif_intr *intr, unsigned int bit,
			enum scsc_mif_target target)
{
	unsigned long flags;
	unsigned long *bitmap;

	if (!intr || bit >= SCSC_MIF_INTR_COUNT)
		return -EINVAL;

	bitmap = scsc_mif_intr_from_host_bitmap(intr, target);
	if (!bitmap)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	if (!intr->active) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -EHOSTDOWN;
	}
	if (!test_bit(bit, bitmap)) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -ENOENT;
	}
	intr->ops->raise_from_host(intr->context, target, bit);
	spin_unlock_irqrestore(&intr->lock, flags);

	return 0;
}

void scsc_mif_intr_ack(struct scsc_mif_intr *intr, unsigned int bit)
{
	if (intr && intr->active && bit < SCSC_MIF_INTR_COUNT)
		intr->ops->clear_to_host(intr->context, bit);
}

int scsc_mif_intr_dispatch(struct scsc_mif_intr *intr)
{
	unsigned long flags;
	u32 pending;
	unsigned int bit;

	if (!intr)
		return -EINVAL;

	spin_lock_irqsave(&intr->lock, flags);
	if (!intr->active) {
		spin_unlock_irqrestore(&intr->lock, flags);
		return -EHOSTDOWN;
	}
	pending = intr->ops->get_pending(intr->context);
	for (bit = 0; bit < SCSC_MIF_INTR_COUNT; bit++)
		if (pending & BIT(bit))
			intr->slot[bit].handler(bit, intr->slot[bit].data);
	spin_unlock_irqrestore(&intr->lock, flags);

	return 0;
}

irqreturn_t scsc_mif_intr_irq(int irq, void *data)
{
	struct scsc_mif_intr *intr = data;
	int ret;

	(void)irq;
	ret = scsc_mif_intr_dispatch(intr);
	if (ret == -EHOSTDOWN)
		return IRQ_NONE;
	if (ret)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

struct scsc_mif_intr_test_context {
	u32 pending;
	u32 masked;
	u32 cleared;
	u32 unmasked;
	u32 raised_r4;
	u32 raised_m4;
	unsigned int handled;
};

static u32 scsc_mif_intr_test_pending(void *context)
{
	return ((struct scsc_mif_intr_test_context *)context)->pending;
}

static void scsc_mif_intr_test_mask(void *context, unsigned int bit)
{
	((struct scsc_mif_intr_test_context *)context)->masked |= BIT(bit);
}

static void scsc_mif_intr_test_clear(void *context, unsigned int bit)
{
	struct scsc_mif_intr_test_context *test = context;

	test->cleared |= BIT(bit);
	test->pending &= ~BIT(bit);
}

static void scsc_mif_intr_test_unmask(void *context, unsigned int bit)
{
	((struct scsc_mif_intr_test_context *)context)->unmasked |= BIT(bit);
}

static void scsc_mif_intr_test_raise(void *context,
				     enum scsc_mif_target target,
				     unsigned int bit)
{
	struct scsc_mif_intr_test_context *test = context;

	if (target == SCSC_MIF_TARGET_R4)
		test->raised_r4 |= BIT(bit);
	else
		test->raised_m4 |= BIT(bit);
}

static void scsc_mif_intr_test_handler(unsigned int bit, void *data)
{
	struct scsc_mif_intr_test_context *test = data;

	test->handled++;
	test->pending &= ~BIT(bit);
}

static const struct scsc_mif_intr_ops scsc_mif_intr_test_ops = {
	.get_pending = scsc_mif_intr_test_pending,
	.mask_to_host = scsc_mif_intr_test_mask,
	.clear_to_host = scsc_mif_intr_test_clear,
	.unmask_to_host = scsc_mif_intr_test_unmask,
	.raise_from_host = scsc_mif_intr_test_raise,
};

int scsc_mif_intr_selftest(void)
{
	struct scsc_mif_intr_test_context test = {};
	struct scsc_mif_intr intr;
	int allocated[SCSC_MIF_INTR_COUNT - 1];
	unsigned int i;
	int bit, ret;

	ret = scsc_mif_intr_init(&intr, &scsc_mif_intr_test_ops, &test);
	if (ret)
		return ret;
	if (scsc_mif_intr_alloc_to_host(&intr,
					scsc_mif_intr_test_handler, &test) !=
		    -EHOSTDOWN || scsc_mif_intr_dispatch(&intr) != -EHOSTDOWN ||
	    scsc_mif_intr_irq(0, &intr) != IRQ_NONE ||
	    scsc_mif_intr_raise(&intr, 0, SCSC_MIF_TARGET_R4) != -EHOSTDOWN ||
	    test.masked || test.cleared || test.unmasked || test.raised_r4 ||
	    test.raised_m4) {
		ret = -EINVAL;
		goto out;
	}
	scsc_mif_intr_set_active(&intr, true);

	for (i = 0; i < ARRAY_SIZE(allocated); i++) {
		bit = scsc_mif_intr_alloc_from_host(&intr, SCSC_MIF_TARGET_R4);
		if (bit != i + 1) {
			ret = -EINVAL;
			goto out;
		}
		allocated[i] = bit;
	}
	if (scsc_mif_intr_alloc_from_host(&intr, SCSC_MIF_TARGET_R4) != -ENOSPC ||
	    scsc_mif_intr_free_from_host(&intr, -1, SCSC_MIF_TARGET_R4) != -EINVAL ||
	    scsc_mif_intr_free_from_host(&intr, 0, SCSC_MIF_TARGET_R4) != -ENOENT) {
		ret = -EINVAL;
		goto out;
	}
	if (scsc_mif_intr_raise(&intr, allocated[0], SCSC_MIF_TARGET_R4) ||
	    test.raised_r4 != BIT(1)) {
		ret = -EINVAL;
		goto out;
	}

	bit = scsc_mif_intr_alloc_from_host(&intr, SCSC_MIF_TARGET_M4);
	if (bit != 1 || scsc_mif_intr_raise(&intr, bit, SCSC_MIF_TARGET_M4) ||
	    test.raised_m4 != BIT(1)) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < SCSC_MIF_INTR_COUNT; i++) {
		bit = scsc_mif_intr_alloc_to_host(&intr,
						  scsc_mif_intr_test_handler, &test);
		if (bit != i) {
			ret = -EINVAL;
			goto out;
		}
	}
	if (scsc_mif_intr_alloc_to_host(&intr, scsc_mif_intr_test_handler,
					&test) != -ENOSPC ||
	    test.masked != GENMASK(15, 0) || test.cleared != GENMASK(15, 0) ||
	    test.unmasked != GENMASK(15, 0)) {
		ret = -EINVAL;
		goto out;
	}
	test.pending = BIT(5);
	if (scsc_mif_intr_irq(0, &intr) != IRQ_HANDLED) {
		ret = -EINVAL;
		goto out;
	}
	if (test.handled != 1 || test.pending) {
		ret = -EINVAL;
		goto out;
	}
	if (scsc_mif_intr_free_to_host(&intr, -1) != -EINVAL ||
	    scsc_mif_intr_free_to_host(&intr, 16) != -EINVAL ||
	    scsc_mif_intr_free_to_host(&intr, 5) ||
	    scsc_mif_intr_free_to_host(&intr, 5) != -ENOENT) {
		ret = -EINVAL;
		goto out;
	}
	test.pending = BIT(5);
	ret = scsc_mif_intr_dispatch(&intr);
	if (ret)
		goto out;
	if (test.pending || !(test.cleared & BIT(5))) {
		ret = -EINVAL;
		goto out;
	}
	ret = 0;
out:
	scsc_mif_intr_deinit(&intr);
	return ret;
}
