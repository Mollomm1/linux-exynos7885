// SPDX-License-Identifier: GPL-2.0-only
/* S2MU005 5 V charger and OTG regulator. Register sequences from Samsung. */
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/workqueue.h>

#define SC_STATUS0	0x08
#define SC_STATUS3	0x0b
#define SC_CTRL0	0x0e
#define SC_MODE		GENMASK(2, 0)
#define SC_CHG_EN	BIT(4)
#define SC_BUCK		1
#define SC_CHARGE	2
#define SC_OTG		4
#define SC_INPUT	0x10
#define SC_OCP		0x12
#define SC_BOOST_V	0x13
#define SC_CC_COOL	0x14
#define SC_CC		0x15
#define SC_CV		0x16
#define SC_TOPOFF	0x18
#define SC_FREQ		0x19
#define SC_WDT		0x1a
#define SC_WDT_KICK	0x1b
#define SC_OTG_EN	0x1d
#define MUIC_DEVICE_TYPE1 0x4a
#define MUIC_DCP	BIT(6)
#define MUIC_CDP	BIT(5)

struct s2mu005_charger {
	struct device *dev;
	struct regmap *map;
	struct mutex lock;
	struct iio_channel *thermistor;
	struct power_supply *psy;
	struct delayed_work work;
	unsigned int revision;
	unsigned int offset;
	u32 adc_limits[4];
	bool boost;
	bool charging;
	bool online;
	bool thermal_block;
	bool suspended;
	bool stopping;
	int health;
	int input_ua;
	int gadget_ua;
	int charge_code;
};

/* Caller holds lock, including during regulator callbacks. */
static int s2mu005_charge_off(struct s2mu005_charger *chg)
{
	int ret, err;

	ret = regmap_update_bits(chg->map, SC_CTRL0, SC_MODE,
				 chg->revision >= 4 ? SC_BUCK : 0);
	err = regmap_update_bits(chg->map, SC_WDT, 3, 1);
	if (!ret)
		ret = err;
	if (!ret)
		chg->charging = false;
	return ret;
}

static int s2mu005_boost_off(struct s2mu005_charger *chg)
{
	int ret, err;

	/* Do not enable charging until a valid sink contract is observed. */
	ret = s2mu005_charge_off(chg);
	err = regmap_update_bits(chg->map, 0x88, 0x0c, 0x08);
	if (!ret)
		ret = err;
	err = regmap_write(chg->map, 0x98, chg->offset);
	if (!ret)
		ret = err;
	err = regmap_update_bits(chg->map, 0x96, BIT(0), BIT(0));
	if (!ret)
		ret = err;
	if (!ret)
		chg->boost = false;
	return ret;
}

static int s2mu005_vbus_enable(struct regulator_dev *rdev)
{
	struct s2mu005_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->lock);
	ret = s2mu005_charge_off(chg);
	if (ret)
		goto out;
	/* Normal (non-factory) OTG sequence, including OCP offset workaround. */
	ret = regmap_update_bits(chg->map, SC_OTG_EN, 0x0c, 0);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, SC_OCP, BIT(5), BIT(5));
	if (ret)
		goto fail;
	ret = regmap_write(chg->map, 0x98,
			   chg->offset > 50 ? chg->offset - 50 : 0);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, 0x96, BIT(0), 0);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, SC_CTRL0, SC_MODE, SC_OTG);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, 0x88, 0x0c, 0x0c);
	if (ret)
		goto fail;
	usleep_range(5000, 6000);
	ret = regmap_update_bits(chg->map, SC_OTG_EN, 0x0c, BIT(2));
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, SC_OCP, BIT(5), 0);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, SC_FREQ, 0x60, 0x40);
	if (ret)
		goto fail;
	/* Samsung's 1.5 A OCP threshold and switch-off-on-overcurrent setting. */
	ret = regmap_update_bits(chg->map, SC_OCP, 0x3c, 0x2c);
	if (ret)
		goto fail;
	ret = regmap_update_bits(chg->map, SC_BOOST_V, 0x1f, 0x16);
	if (ret)
		goto fail;
	chg->boost = true;
	goto out;
fail:
	if (s2mu005_boost_off(chg))
		dev_err(chg->dev, "failed to stop boost after I/O error\n");
out:
	mutex_unlock(&chg->lock);
	return ret;
}

static int s2mu005_vbus_disable(struct regulator_dev *rdev)
{
	struct s2mu005_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->lock);
	ret = s2mu005_boost_off(chg);
	mutex_unlock(&chg->lock);
	return ret;
}

static int s2mu005_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct s2mu005_charger *chg = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(chg->map, SC_CTRL0, &val);
	return ret ? ret : (val & SC_MODE) == SC_OTG;
}

static const struct regulator_ops s2mu005_vbus_ops = {
	.enable = s2mu005_vbus_enable,
	.disable = s2mu005_vbus_disable,
	.is_enabled = s2mu005_vbus_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regulator_desc s2mu005_vbus_desc = {
	.name = "s2mu005-vbus",
	.of_match = "vbus",
	.ops = &s2mu005_vbus_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = 1,
	.min_uV = 5100000,
	.fixed_uV = 5100000,
};

static int s2mu005_charge_on(struct s2mu005_charger *chg, int ua)
{
	int ret;

	ret = regmap_update_bits(chg->map, SC_INPUT, 0x3f,
				 (ua - 100000) / 50000);
	if (ret)
		return ret;
	if (chg->charging)
		return regmap_write_bits(chg->map, SC_WDT_KICK, BIT(0), BIT(0));

	/* Older revisions need a temporary QBAT current and settling delay. */
	if (chg->revision <= 2) {
		ret = regmap_update_bits(chg->map, SC_CC, 0x3f, 33);
		if (ret)
			return ret;
		msleep(20);
	}
	if (chg->revision < 4) {
		ret = regmap_update_bits(chg->map, SC_CTRL0, SC_MODE, 0);
		if (ret)
			return ret;
	}
	if (chg->revision <= 2) {
		msleep(50);
		ret = regmap_update_bits(chg->map, 0x2a, BIT(3), 0);
		if (ret)
			return ret;
	}
	ret = regmap_update_bits(chg->map, SC_CTRL0, SC_MODE | SC_CHG_EN,
				 SC_CHARGE | SC_CHG_EN);
	if (ret)
		return ret;
	if (chg->revision <= 2) {
		msleep(150);
		ret = regmap_update_bits(chg->map, 0x2a, BIT(3), BIT(3));
		if (ret)
			return ret;
	}
	/* Battery charge limit; input is limited separately. */
	ret = regmap_update_bits(chg->map, SC_CC, 0x3f, chg->charge_code);
	if (ret)
		return ret;
	ret = regmap_update_bits(chg->map, SC_WDT_KICK, BIT(0), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(chg->map, SC_WDT, 3, 2);
	if (!ret)
		chg->charging = true;
	return ret;
}

static int s2mu005_sink_current(struct s2mu005_charger *chg)
{
	union power_supply_propval val;
	struct power_supply *source;
	unsigned int device_type;
	int ret, ua = 0;

	/* Lazy lookup avoids the Type-C controller/VBUS regulator probe cycle. */
	source = power_supply_get_by_phandle(chg->dev->of_node,
					    "samsung,usb-port");
	if (IS_ERR_OR_NULL(source))
		return 0;
	ret = power_supply_get_property(source, POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret || !val.intval)
		goto out;
	ret = power_supply_get_property(source, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret || val.intval != 5000000)
		goto out;
	ret = power_supply_get_property(source, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (!ret) {
		ua = min(val.intval, 1500000);
		/* The MUIC identifies BC1.2 chargers independently of Type-C Rp. */
		ret = regmap_read(chg->map, MUIC_DEVICE_TYPE1, &device_type);
		if (!ret && (device_type & (MUIC_DCP | MUIC_CDP)))
			ua = 1500000;
		else if (chg->gadget_ua > 100000)
			ua = max(ua, chg->gadget_ua);
	}
out:
	power_supply_put(source);
	return ua;
}

static void s2mu005_charge_work(struct work_struct *work)
{
	struct s2mu005_charger *chg = container_of(to_delayed_work(work),
						struct s2mu005_charger, work);
	unsigned int status = 0, event = 0, input_status = 0;
	int adc = 0, ua, ret;

	mutex_lock(&chg->lock);
	if (chg->stopping || chg->suspended)
		goto out;
	if (chg->boost)
		goto requeue;
	ua = s2mu005_sink_current(chg);
	if (!ua)
		chg->gadget_ua = 0;
	ret = iio_read_channel_raw(chg->thermistor, &adc);
	chg->online = ua > 0;
	if (ret < 0) {
		chg->health = POWER_SUPPLY_HEALTH_UNKNOWN;
		goto disable;
	}
	if (adc <= chg->adc_limits[0] || adc >= chg->adc_limits[3])
		chg->thermal_block = true;
	else if (adc >= chg->adc_limits[1] && adc <= chg->adc_limits[2])
		chg->thermal_block = false;
	chg->health = chg->thermal_block ? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
					 POWER_SUPPLY_HEALTH_GOOD;
	ret = regmap_read(chg->map, SC_STATUS0, &status);
	if (ret)
		goto disable;
	ret = regmap_read(chg->map, SC_STATUS3, &event);
	if (ret)
		goto disable;
	chg->online = ua > 0 && (status & BIT(7));
	if (!chg->online || chg->thermal_block || ua < 100000)
		goto disable;
	/* Only thermal shutdown and VSYS overvoltage are persistent faults. */
	switch (event & 0x0f) {
	case 1:
	case 3:
		chg->health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto disable;
	case 5:
		/* A watchdog suspension needs a full charger restart. */
		ret = s2mu005_charge_off(chg);
		if (ret)
			goto disable;
		break;
	default:
		break;
	}
	ret = regmap_read(chg->map, 0x09, &input_status);
	if (ret)
		goto disable;
	if ((input_status & 0x70) != 0x30 &&
	    (input_status & 0x70) != 0x50) {
		chg->health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto disable;
	}
	ret = s2mu005_charge_on(chg, ua);
	if (ret)
		goto disable;
	chg->input_ua = ua;
	goto requeue;
disable:
	if (ua > 0)
		dev_info_ratelimited(chg->dev,
			"charge blocked: adc=%d status0=%#x status1=%#x status3=%#x current=%d\n",
			adc, status, input_status, event, ua);
	if (s2mu005_charge_off(chg))
		dev_err_ratelimited(chg->dev, "failed to disable charging\n");
	chg->input_ua = 0;
requeue:
	schedule_delayed_work(&chg->work, HZ);
out:
	mutex_unlock(&chg->lock);
	power_supply_changed(chg->psy);
}

static irqreturn_t s2mu005_charger_irq(int irq, void *data)
{
	struct s2mu005_charger *chg = data;

	if (!READ_ONCE(chg->stopping) && !READ_ONCE(chg->suspended))
		mod_delayed_work(system_wq, &chg->work, 0);
	return IRQ_HANDLED;
}

static const enum power_supply_property s2mu005_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int s2mu005_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct s2mu005_charger *chg = power_supply_get_drvdata(psy);
	unsigned int status;
	int ret = 0;

	mutex_lock(&chg->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online && !chg->boost;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(chg->map, SC_STATUS0, &status);
		if (ret)
			break;
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		if (chg->boost || !chg->online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (chg->charging)
			val->intval = (status & 0xf) == 8 ? POWER_SUPPLY_STATUS_FULL :
							 POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->health;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chg->boost ? 0 : chg->input_ua;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&chg->lock);
	return ret;
}

static int s2mu005_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct s2mu005_charger *chg = power_supply_get_drvdata(psy);

	if (prop != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;
	if (val->intval < 0 || val->intval > 500000)
		return -EINVAL;
	mutex_lock(&chg->lock);
	chg->gadget_ua = val->intval;
	mutex_unlock(&chg->lock);
	mod_delayed_work(system_wq, &chg->work, 0);
	return 0;
}

static int s2mu005_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static const struct power_supply_desc s2mu005_psy_desc = {
	.name = "s2mu005-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = s2mu005_properties,
	.num_properties = ARRAY_SIZE(s2mu005_properties),
	.get_property = s2mu005_get_property,
	.set_property = s2mu005_set_property,
	.property_is_writeable = s2mu005_property_is_writeable,
};

static void s2mu005_stop(void *data)
{
	struct s2mu005_charger *chg = data;

	mutex_lock(&chg->lock);
	chg->stopping = true;
	mutex_unlock(&chg->lock);
	cancel_delayed_work_sync(&chg->work);
	mutex_lock(&chg->lock);
	if (s2mu005_boost_off(chg))
		dev_err(chg->dev, "failed to turn off power\n");
	mutex_unlock(&chg->lock);
}

static int s2mu005_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config psy_cfg = {};
	struct regulator_config reg_cfg = {};
	struct regulator_dev *rdev;
	struct power_supply_battery_info *battery;
	struct s2mu005_charger *chg;
	unsigned int trim;
	int ret, i, irq;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;
	chg->dev = dev;
	chg->map = dev_get_regmap(dev->parent, NULL);
	if (!chg->map)
		return -ENODEV;
	mutex_init(&chg->lock);
	INIT_DELAYED_WORK(&chg->work, s2mu005_charge_work);
	platform_set_drvdata(pdev, chg);
	chg->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	chg->thermal_block = true;
	chg->thermistor = devm_iio_channel_get(dev, "battery-temperature");
	if (IS_ERR(chg->thermistor))
		return dev_err_probe(dev, PTR_ERR(chg->thermistor), "no thermistor\n");
	ret = device_property_read_u32_array(dev, "samsung,temperature-adc-limits",
					     chg->adc_limits, 4);
	if (ret)
		return ret;
	for (i = 1; i < 4; i++)
		if (chg->adc_limits[i] <= chg->adc_limits[i - 1])
			return -EINVAL;
	ret = regmap_read(chg->map, 0x73, &chg->revision);
	if (ret)
		return ret;
	chg->revision &= 0x0f;
	ret = regmap_read(chg->map, 0x98, &chg->offset);
	if (ret)
		return ret;
	ret = regmap_read(chg->map, 0x96, &trim);
	if (ret)
		return ret;
	if (!(trim & BIT(0)))
		chg->offset = min(chg->offset + 50, 255U);
	ret = s2mu005_boost_off(chg);
	if (ret)
		return ret;
	/* Samsung keeps CHG_EN asserted and selects off/buck/charge/boost by mode. */
	ret = regmap_update_bits(chg->map, SC_CTRL0, SC_CHG_EN, SC_CHG_EN);
	if (ret)
		return ret;
	psy_cfg.drv_data = chg;
	psy_cfg.of_node = dev->of_node;
	chg->psy = devm_power_supply_register(dev, &s2mu005_psy_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return PTR_ERR(chg->psy);
	ret = power_supply_get_battery_info(chg->psy, &battery);
	if (ret)
		return ret;
	if (battery->constant_charge_voltage_max_uv < 3900000 ||
	    battery->constant_charge_voltage_max_uv > 4400000 ||
	    battery->constant_charge_current_max_ua < 100000 ||
	    battery->constant_charge_current_max_ua > 2600000 ||
	    battery->charge_term_current_ua < 100000 ||
	    battery->charge_term_current_ua > 475000) {
		power_supply_put_battery_info(chg->psy, battery);
		return -EINVAL;
	}
	chg->charge_code = (battery->constant_charge_current_max_ua - 100000) /
			  50000 + 1;
	ret = regmap_update_bits(chg->map, SC_CV, 0x7e,
				 (battery->constant_charge_voltage_max_uv - 3900000) /
				 10000 << 1);
	if (!ret)
		ret = regmap_update_bits(chg->map, SC_CC_COOL, 0x3f,
					 min(chg->charge_code, 19));
	if (!ret)
		ret = regmap_write(chg->map, SC_TOPOFF,
				   (battery->charge_term_current_ua - 100000) / 25000);
	power_supply_put_battery_info(chg->psy, battery);
	if (ret)
		return ret;
	ret = regmap_update_bits(chg->map, 0x20, 0x3f, 0x35);
	if (ret)
		return ret;
	if (chg->revision == 3) {
		ret = regmap_update_bits(chg->map, 0xaf, BIT(7), BIT(7));
		if (ret)
			return ret;
	}
	reg_cfg.dev = dev;
	reg_cfg.driver_data = chg;
	reg_cfg.of_node = dev->of_node;
	rdev = devm_regulator_register(dev, &s2mu005_vbus_desc, &reg_cfg);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	ret = devm_add_action_or_reset(dev, s2mu005_stop, chg);
	if (ret)
		return ret;
	for (i = 0; i < 3; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;
		ret = devm_request_threaded_irq(dev, irq, NULL, s2mu005_charger_irq,
						IRQF_ONESHOT, dev_name(dev), chg);
		if (ret)
			return ret;
	}
	schedule_delayed_work(&chg->work, 0);
	return 0;
}

static void s2mu005_charger_shutdown(struct platform_device *pdev)
{
	s2mu005_stop(platform_get_drvdata(pdev));
}

static int s2mu005_charger_suspend(struct device *dev)
{
	struct s2mu005_charger *chg = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&chg->lock);
	chg->suspended = true;
	if (!chg->boost)
		ret = s2mu005_charge_off(chg);
	mutex_unlock(&chg->lock);
	cancel_delayed_work_sync(&chg->work);
	if (ret) {
		mutex_lock(&chg->lock);
		chg->suspended = false;
		mutex_unlock(&chg->lock);
		mod_delayed_work(system_wq, &chg->work, 0);
	}
	return ret;
}

static int s2mu005_charger_resume(struct device *dev)
{
	struct s2mu005_charger *chg = dev_get_drvdata(dev);

	mutex_lock(&chg->lock);
	chg->suspended = false;
	mutex_unlock(&chg->lock);
	mod_delayed_work(system_wq, &chg->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2mu005_charger_pm,
			      s2mu005_charger_suspend, s2mu005_charger_resume);

static const struct of_device_id s2mu005_charger_match[] = {
	{ .compatible = "samsung,s2mu005-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu005_charger_match);

static struct platform_driver s2mu005_charger_driver = {
	.probe = s2mu005_charger_probe,
	.shutdown = s2mu005_charger_shutdown,
	.driver = {
		.name = "s2mu005-charger",
		.of_match_table = s2mu005_charger_match,
		.pm = pm_sleep_ptr(&s2mu005_charger_pm),
	},
};
module_platform_driver(s2mu005_charger_driver);

MODULE_DESCRIPTION("Samsung S2MU005 charger and OTG regulator");
MODULE_LICENSE("GPL");
