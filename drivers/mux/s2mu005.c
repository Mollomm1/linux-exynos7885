// SPDX-License-Identifier: GPL-2.0-only
/* S2MU005 D+/D- switch: state 0 is open, state 1 connects USB to the AP. */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mux/driver.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static int s2mu005_mux_set(struct mux_control *mux, int state)
{
	struct regmap *map = *(struct regmap **)mux_chip_priv(mux->chip);

	/* Preserve the charger and jig bits. */
	return regmap_update_bits(map, 0xb5, GENMASK(7, 2),
				  state == 1 ? 0x24 : 0);
}

static const struct mux_control_ops s2mu005_mux_ops = {
	.set = s2mu005_mux_set,
};

static int s2mu005_mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *map = dev_get_regmap(dev->parent, NULL);
	struct mux_chip *chip;
	int ret;

	if (!map)
		return -ENODEV;
	chip = devm_mux_chip_alloc(dev, 1, sizeof(map));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	*(struct regmap **)mux_chip_priv(chip) = map;
	chip->ops = &s2mu005_mux_ops;
	chip->mux->states = 2;
	chip->mux->idle_state = 0;

	/* Clear AUTO_SW; keep wait/open control and mask unused MUIC IRQs. */
	ret = regmap_update_bits(map, 0xb2, GENMASK(4, 0), 0x13);
	if (ret)
		return ret;
	return devm_mux_chip_register(dev, chip);
}

static const struct of_device_id s2mu005_mux_match[] = {
	{ .compatible = "samsung,s2mu005-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu005_mux_match);

static struct platform_driver s2mu005_mux_driver = {
	.probe = s2mu005_mux_probe,
	.driver = {
		.name = "s2mu005-mux",
		.of_match_table = s2mu005_mux_match,
	},
};
module_platform_driver(s2mu005_mux_driver);

MODULE_DESCRIPTION("Samsung S2MU005 USB data switch");
MODULE_LICENSE("GPL");
