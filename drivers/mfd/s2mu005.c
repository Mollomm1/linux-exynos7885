// SPDX-License-Identifier: GPL-2.0-only
/* Samsung S2MU005 interface PMIC. */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>

static const struct regmap_config s2mu005_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xc0,
};

static const struct regmap_irq s2mu005_irqs[] = {
	REGMAP_IRQ_REG(0, 0, BIT(0)),
	REGMAP_IRQ_REG(1, 0, BIT(1)),
	REGMAP_IRQ_REG(2, 0, BIT(2)),
	REGMAP_IRQ_REG(3, 0, BIT(3)),
	REGMAP_IRQ_REG(4, 0, BIT(4)),
	REGMAP_IRQ_REG(5, 0, BIT(5)),
	REGMAP_IRQ_REG(6, 0, BIT(6)),
	REGMAP_IRQ_REG(7, 0, BIT(7)),
};

static const struct regmap_irq_chip s2mu005_irq_chip = {
	.name = "s2mu005",
	.status_base = 0x00,
	.mask_base = 0x01,
	.num_regs = 1,
	.irqs = s2mu005_irqs,
	.num_irqs = ARRAY_SIZE(s2mu005_irqs),
};

static const struct resource s2mu005_charger_irqs[] = {
	DEFINE_RES_IRQ_NAMED(3, "event"),
	DEFINE_RES_IRQ_NAMED(4, "charge"),
	DEFINE_RES_IRQ_NAMED(7, "vbus"),
};

static const struct mfd_cell s2mu005_cells[] = {
	{
		.name = "s2mu005-charger",
		.of_compatible = "samsung,s2mu005-charger",
		.resources = s2mu005_charger_irqs,
		.num_resources = ARRAY_SIZE(s2mu005_charger_irqs),
	}, {
		.name = "s2mu005-mux",
		.of_compatible = "samsung,s2mu005-mux",
	},
};

static int s2mu005_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap_irq_chip_data *irq_data;
	struct regmap *map;
	unsigned int revision;
	int ret;

	map = devm_regmap_init_i2c(client, &s2mu005_regmap);
	if (IS_ERR(map))
		return PTR_ERR(map);

	ret = regmap_read(map, 0x73, &revision);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read revision\n");

	/* Unused flash/MUIC interrupts share the charger interrupt line. */
	ret = regmap_write(map, 0x03, 0xff);
	if (ret)
		return ret;
	ret = regmap_write(map, 0x06, 0xff);
	if (ret)
		return ret;
	ret = regmap_write(map, 0x07, 0xff);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, map, client->irq, IRQF_ONESHOT,
				       0, &s2mu005_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register interrupts\n");

	dev_info(dev, "revision %#x\n", revision);
	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, s2mu005_cells,
				    ARRAY_SIZE(s2mu005_cells), NULL, 0,
				    regmap_irq_get_domain(irq_data));
}

static const struct of_device_id s2mu005_of_match[] = {
	{ .compatible = "samsung,s2mu005" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu005_of_match);

static struct i2c_driver s2mu005_driver = {
	.driver = {
		.name = "s2mu005",
		.of_match_table = s2mu005_of_match,
	},
	.probe = s2mu005_probe,
};
module_i2c_driver(s2mu005_driver);

MODULE_DESCRIPTION("Samsung S2MU005 interface PMIC");
MODULE_LICENSE("GPL");
