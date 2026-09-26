// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7885 AP-to-APM ACPM doorbell mailbox. */
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/mailbox/exynos-message.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define EXYNOS7885_INTGR0	0x08
#define EXYNOS7885_INTMR1	0x24
#define EXYNOS7885_CHANNELS	16

struct exynos7885_mbox {
	void __iomem *regs;
	u32 original_mask;
	struct mbox_controller controller;
	struct mbox_chan channels[EXYNOS7885_CHANNELS];
};

static int exynos7885_mbox_send(struct mbox_chan *chan, void *data)
{
	struct exynos7885_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct exynos_mbox_msg *msg = data;

	if (!msg || msg->chan_type != EXYNOS_MBOX_CHAN_TYPE_DOORBELL ||
	    msg->chan_id >= EXYNOS7885_CHANNELS)
		return -EINVAL;

	/* Exynos7885 sends AP-to-APM interrupts in the upper half of INTGR0. */
	writel(BIT(msg->chan_id + 16), mbox->regs + EXYNOS7885_INTGR0);
	return 0;
}

static const struct mbox_chan_ops exynos7885_mbox_ops = {
	.send_data = exynos7885_mbox_send,
};

static struct mbox_chan *exynos7885_mbox_xlate(struct mbox_controller *ctl,
					       const struct of_phandle_args *sp)
{
	int i;

	if (sp->args_count)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < ctl->num_chans; i++)
		if (!ctl->chans[i].cl)
			return &ctl->chans[i];

	return ERR_PTR(-EBUSY);
}

static void exynos7885_mbox_restore_mask(void *data)
{
	struct exynos7885_mbox *mbox = data;

	writel(mbox->original_mask, mbox->regs + EXYNOS7885_INTMR1);
}

static int exynos7885_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7885_mbox *mbox;
	int i, ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbox->regs))
		return PTR_ERR(mbox->regs);

	mbox->controller.dev = dev;
	mbox->controller.ops = &exynos7885_mbox_ops;
	mbox->controller.of_xlate = exynos7885_mbox_xlate;
	mbox->controller.chans = mbox->channels;
	mbox->controller.num_chans = EXYNOS7885_CHANNELS;
	for (i = 0; i < EXYNOS7885_CHANNELS; i++)
		mbox->channels[i].mbox = &mbox->controller;

	platform_set_drvdata(pdev, mbox);
	/* Polling clients do not service the APM-to-AP interrupt. */
	mbox->original_mask = readl(mbox->regs + EXYNOS7885_INTMR1);
	ret = devm_add_action_or_reset(dev, exynos7885_mbox_restore_mask, mbox);
	if (ret)
		return ret;
	writel(mbox->original_mask | GENMASK(15, 0),
	       mbox->regs + EXYNOS7885_INTMR1);
	return devm_mbox_controller_register(dev, &mbox->controller);
}

static const struct of_device_id exynos7885_mbox_match[] = {
	{ .compatible = "samsung,exynos7885-acpm-mbox" },
	{ }
};

static struct platform_driver exynos7885_mbox_driver = {
	.probe = exynos7885_mbox_probe,
	.driver = {
		.name = "exynos7885-acpm-mbox",
		.of_match_table = exynos7885_mbox_match,
	},
};
module_platform_driver(exynos7885_mbox_driver);

/* No module alias: only an explicit module load may probe this hardware. */
MODULE_DESCRIPTION("Exynos7885 ACPM doorbell mailbox");
MODULE_LICENSE("GPL");
