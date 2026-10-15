// SPDX-License-Identifier: GPL-2.0-only
/*
 * S2MPU08 regulators controlled through ACPM.
 *
 * On Exynos7885 the S2MPU08 PMIC sits behind the APM, so its registers are
 * reached with ACPM PMIC read/write messages. For a buck, BUCK2 (A73
 * cluster), BUCK3 (A53 clusters) or BUCK5 (GPU), this driver only sets the
 * output voltage; enabling and disabling it stay with the PMU and the
 * bootloader. For an LDO, LDO2 or LDO35 (the SD card I/O and card
 * supplies), it also switches the output on and off.
 */

#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define S2MPU08_ACPM_TYPE_PMIC	1
/* voltage select registers */
#define S2MPU08_B2CTRL2		0x17
#define S2MPU08_B3CTRL2		0x19
#define S2MPU08_B5CTRL2		0x1d
/* As in the vendor driver; it matches the boot voltages to the ECT tables. */
#define S2MPU08_BUCK_MIN_UV	300000
#define S2MPU08_BUCK_STEP_UV	6250
#define S2MPU08_BUCK_RAMP_UV_US	12000
/* LDO control registers: on/off in bits 7:6, voltage in bits 5:0 */
#define S2MPU08_L2CTRL1		0x2d
#define S2MPU08_L35CTRL		0x4f
#define S2MPU08_LDO_ON		0xc0
#define S2MPU08_LDO_VSEL	0x3f
#define S2MPU08_LDO_MIN_UV	1800000
#define S2MPU08_LDO_STEP_UV	25000
/* As in the vendor driver */
#define S2MPU08_LDO_RAMP_UV_US	12000
#define S2MPU08_LDO_ENABLE_US	128

struct s2mpu08_acpm_buck {
	const struct acpm_handle *acpm;
	unsigned int chan;
	u8 vsel_reg;
};

static int s2mpu08_acpm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpu08_acpm_buck *buck = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = buck->acpm->ops.pmic_ops.read_reg(buck->acpm, buck->chan,
						S2MPU08_ACPM_TYPE_PMIC,
						buck->vsel_reg, 0, &val);
	if (ret)
		return ret > 0 ? -EIO : ret;

	return val;
}

static int s2mpu08_acpm_set_voltage_sel(struct regulator_dev *rdev,
					unsigned int sel)
{
	struct s2mpu08_acpm_buck *buck = rdev_get_drvdata(rdev);
	int ret;

	ret = buck->acpm->ops.pmic_ops.write_reg(buck->acpm, buck->chan,
						 S2MPU08_ACPM_TYPE_PMIC,
						 buck->vsel_reg, 0, sel);
	return ret > 0 ? -EIO : ret;
}

static int s2mpu08_acpm_read(struct s2mpu08_acpm_buck *buck, u8 *val)
{
	int ret;

	ret = buck->acpm->ops.pmic_ops.read_reg(buck->acpm, buck->chan,
						S2MPU08_ACPM_TYPE_PMIC,
						buck->vsel_reg, 0, val);
	return ret > 0 ? -EIO : ret;
}

/* The regulator core serializes the operations on one regulator. */
static int s2mpu08_acpm_update(struct s2mpu08_acpm_buck *buck, u8 mask,
			       u8 val)
{
	u8 old;
	int ret;

	ret = s2mpu08_acpm_read(buck, &old);
	if (ret)
		return ret;

	ret = buck->acpm->ops.pmic_ops.write_reg(buck->acpm, buck->chan,
						 S2MPU08_ACPM_TYPE_PMIC,
						 buck->vsel_reg, 0,
						 (old & ~mask) | (val & mask));
	return ret > 0 ? -EIO : ret;
}

static int s2mpu08_acpm_ldo_get_voltage_sel(struct regulator_dev *rdev)
{
	struct s2mpu08_acpm_buck *buck = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = s2mpu08_acpm_read(buck, &val);
	if (ret)
		return ret;

	return val & S2MPU08_LDO_VSEL;
}

static int s2mpu08_acpm_ldo_set_voltage_sel(struct regulator_dev *rdev,
					    unsigned int sel)
{
	return s2mpu08_acpm_update(rdev_get_drvdata(rdev), S2MPU08_LDO_VSEL,
				   sel);
}

static int s2mpu08_acpm_ldo_enable(struct regulator_dev *rdev)
{
	return s2mpu08_acpm_update(rdev_get_drvdata(rdev), S2MPU08_LDO_ON,
				   S2MPU08_LDO_ON);
}

static int s2mpu08_acpm_ldo_disable(struct regulator_dev *rdev)
{
	return s2mpu08_acpm_update(rdev_get_drvdata(rdev), S2MPU08_LDO_ON, 0);
}

static int s2mpu08_acpm_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpu08_acpm_buck *buck = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = s2mpu08_acpm_read(buck, &val);
	if (ret)
		return ret;

	return !!(val & S2MPU08_LDO_ON);
}

static const struct regulator_ops s2mpu08_acpm_ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = s2mpu08_acpm_ldo_get_voltage_sel,
	.set_voltage_sel = s2mpu08_acpm_ldo_set_voltage_sel,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.enable = s2mpu08_acpm_ldo_enable,
	.disable = s2mpu08_acpm_ldo_disable,
	.is_enabled = s2mpu08_acpm_ldo_is_enabled,
};

static const struct regulator_ops s2mpu08_acpm_buck_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = s2mpu08_acpm_get_voltage_sel,
	.set_voltage_sel = s2mpu08_acpm_set_voltage_sel,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

struct s2mpu08_acpm_buck_data {
	struct regulator_desc desc;
	u8 vsel_reg;
};

#define S2MPU08_ACPM_BUCK(_name, _reg)					\
	{								\
		.desc = {						\
			.name = _name,					\
			.ops = &s2mpu08_acpm_buck_ops,			\
			.type = REGULATOR_VOLTAGE,			\
			.owner = THIS_MODULE,				\
			.min_uV = S2MPU08_BUCK_MIN_UV,			\
			.uV_step = S2MPU08_BUCK_STEP_UV,		\
			.n_voltages = 256,				\
			.ramp_delay = S2MPU08_BUCK_RAMP_UV_US,		\
		},							\
		.vsel_reg = _reg,					\
	}

static const struct s2mpu08_acpm_buck_data s2mpu08_acpm_buck2 =
	S2MPU08_ACPM_BUCK("buck2", S2MPU08_B2CTRL2);
static const struct s2mpu08_acpm_buck_data s2mpu08_acpm_buck3 =
	S2MPU08_ACPM_BUCK("buck3", S2MPU08_B3CTRL2);
static const struct s2mpu08_acpm_buck_data s2mpu08_acpm_buck5 =
	S2MPU08_ACPM_BUCK("buck5", S2MPU08_B5CTRL2);

#define S2MPU08_ACPM_LDO(_name, _reg)					\
	{								\
		.desc = {						\
			.name = _name,					\
			.ops = &s2mpu08_acpm_ldo_ops,			\
			.type = REGULATOR_VOLTAGE,			\
			.owner = THIS_MODULE,				\
			.min_uV = S2MPU08_LDO_MIN_UV,			\
			.uV_step = S2MPU08_LDO_STEP_UV,			\
			.n_voltages = S2MPU08_LDO_VSEL + 1,		\
			.ramp_delay = S2MPU08_LDO_RAMP_UV_US,		\
			.enable_time = S2MPU08_LDO_ENABLE_US,		\
		},							\
		.vsel_reg = _reg,					\
	}

static const struct s2mpu08_acpm_buck_data s2mpu08_acpm_ldo2 =
	S2MPU08_ACPM_LDO("ldo2", S2MPU08_L2CTRL1);
static const struct s2mpu08_acpm_buck_data s2mpu08_acpm_ldo35 =
	S2MPU08_ACPM_LDO("ldo35", S2MPU08_L35CTRL);

static int s2mpu08_acpm_buck_probe(struct platform_device *pdev)
{
	const struct s2mpu08_acpm_buck_data *data;
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct s2mpu08_acpm_buck *buck;
	struct of_phandle_args args;
	struct regulator_dev *rdev;
	int ret;

	data = of_device_get_match_data(dev);

	buck = devm_kzalloc(dev, sizeof(*buck), GFP_KERNEL);
	if (!buck)
		return -ENOMEM;

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "samsung,acpm-ipc",
					       1, 0, &args);
	if (ret)
		return dev_err_probe(dev, ret, "missing samsung,acpm-ipc\n");

	buck->chan = args.args[0];
	buck->acpm = devm_acpm_get_by_node(dev, args.np);
	of_node_put(args.np);
	if (IS_ERR(buck->acpm))
		return dev_err_probe(dev, PTR_ERR(buck->acpm),
				     "failed to get the ACPM handle\n");
	buck->vsel_reg = data->vsel_reg;

	config.dev = dev;
	config.driver_data = buck;
	config.of_node = dev->of_node;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node,
						      &data->desc);
	if (!config.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, &data->desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register the regulator\n");

	return 0;
}

static const struct of_device_id s2mpu08_acpm_buck_of_match[] = {
	{ .compatible = "samsung,s2mpu08-acpm-buck2", .data = &s2mpu08_acpm_buck2 },
	{ .compatible = "samsung,s2mpu08-acpm-buck3", .data = &s2mpu08_acpm_buck3 },
	{ .compatible = "samsung,s2mpu08-acpm-buck5", .data = &s2mpu08_acpm_buck5 },
	{ .compatible = "samsung,s2mpu08-acpm-ldo2", .data = &s2mpu08_acpm_ldo2 },
	{ .compatible = "samsung,s2mpu08-acpm-ldo35", .data = &s2mpu08_acpm_ldo35 },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpu08_acpm_buck_of_match);

static struct platform_driver s2mpu08_acpm_buck_driver = {
	.driver = {
		.name = "s2mpu08-acpm-buck",
		.of_match_table = s2mpu08_acpm_buck_of_match,
	},
	.probe = s2mpu08_acpm_buck_probe,
};
module_platform_driver(s2mpu08_acpm_buck_driver);

MODULE_DESCRIPTION("Samsung S2MPU08 buck and LDO control over ACPM");
MODULE_LICENSE("GPL");
