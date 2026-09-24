// SPDX-License-Identifier: GPL-2.0-only
/* Samsung S2MM005 resident-firmware Type-C port controller. */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/usb/typec.h>
#include <linux/workqueue.h>

#include "s2mm005.h"

#define S2MM005_CMD		0x0010
#define S2MM005_FUNC		0x0020
#define S2MM005_IRQ		0x0030
#define S2MM005_LP		0x0060
#define S2MM005_REQUEST		0x0240
#define S2MM005_SOURCE_CAPS	0x0260

struct s2mm005 {
	struct i2c_client *client;
	struct mutex lock;
	spinlock_t supply_lock;
	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	struct mux_control *mux;
	struct typec_port *port;
	struct typec_partner *partner;
	struct power_supply *psy;
	struct delayed_work work;
	struct fwnode_handle *connector;
	enum usb_role role;
	bool sourcing;
	bool role_dirty;
	bool mux_selected;
	bool irq_disabled;
	bool stopped;
	bool suspended;
	bool five_volt_requested;
	bool pd;
	bool drp_restored;
	int supply_ua;
	int supply_uv;
	u32 last_func;
	u32 last_lp;
};

static int s2mm005_read(struct s2mm005 *s, u16 reg, void *data, u16 len)
{
	u8 address[] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{ .addr = s->client->addr, .len = 2, .buf = address },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = len, .buf = data },
	};
	int ret, attempt;

	/* A sleeping controller can NAK the first transfer while it wakes. */
	for (attempt = 0; attempt < 5; attempt++) {
		ret = i2c_transfer(s->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			return 0;
		usleep_range(1000, 2000);
	}
	return ret < 0 ? ret : -EIO;
}

static int s2mm005_command(struct s2mm005 *s, const u8 *data, size_t len)
{
	u8 buf[7] = { 0, S2MM005_CMD };
	u8 wake;
	int ret, i;

	if (len > sizeof(buf) - 2)
		return -EINVAL;
	/* The controller may be in auto-LPM; wake it before every command. */
	for (i = 0; i < 5; i++) {
		ret = s2mm005_read(s, 0x0008, &wake, sizeof(wake));
		if (ret)
			return ret;
	}
	udelay(10);
	memcpy(buf + 2, data, len);
	ret = i2c_master_send(s->client, buf, len + 2);
	return ret == len + 2 ? 0 : ret < 0 ? ret : -EIO;
}

static void s2mm005_supply(struct s2mm005 *s, int uv, int ua)
{
	unsigned long flags;
	bool changed;

	spin_lock_irqsave(&s->supply_lock, flags);
	changed = s->supply_uv != uv || s->supply_ua != ua;
	s->supply_uv = uv;
	s->supply_ua = ua;
	spin_unlock_irqrestore(&s->supply_lock, flags);
	if (changed)
		power_supply_changed(s->psy);
}

static int s2mm005_disconnect(struct s2mm005 *s)
{
	int ret = 0, err;

	s2mm005_supply(s, 0, 0);
	if (s->sourcing) {
		ret = regulator_disable(s->vbus);
		if (!ret)
			s->sourcing = false;
	}
	if (s->role != USB_ROLE_NONE || s->role_dirty) {
		err = usb_role_switch_set_role(s->role_sw, USB_ROLE_NONE);
		s->role_dirty = !!err;
		if (!ret)
			ret = err;
	}
	if (s->mux_selected) {
		err = mux_control_deselect(s->mux);
		s->mux_selected = false;
		if (!ret)
			ret = err;
	}
	if (s->partner) {
		typec_unregister_partner(s->partner);
		s->partner = NULL;
	}
	s->role = USB_ROLE_NONE;
	s->pd = false;
	s->five_volt_requested = false;
	typec_set_pwr_opmode(s->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(s->port, TYPEC_SINK);
	typec_set_data_role(s->port, TYPEC_DEVICE);
	return ret;
}

static int s2mm005_current(struct s2mm005 *s, u32 func, int *ua)
{
	u8 caps[32], request[8];
	u32 pdo;
	unsigned int rp = FIELD_GET(S2MM005_RP, func);
	u8 select[] = { 0x03, 0x03, 1 };
	const u8 negotiate[] = { 0x03, 0x02, 17 };
	int ret;

	/* USB default current until enumeration; do not assume a 500 mA grant. */
	*ua = rp == 3 ? 3000000 : rp == 2 ? 1500000 : 100000;
	if (FIELD_GET(S2MM005_STATE, func) != 21) {
		if (s->five_volt_requested || s->pd)
			*ua = 0;
		return 0;
	}
	ret = s2mm005_read(s, S2MM005_SOURCE_CAPS, caps, sizeof(caps));
	if (ret)
		return ret;
	ret = s2mm005_read(s, S2MM005_REQUEST, request, sizeof(request));
	if (ret)
		return ret;
	ret = s2mm005_pd_current(caps, request);
	if (ret < 0 && ret != -ERANGE)
		return ret;
	if (ret == -ERANGE) {
		*ua = 0;
		/* PDO 1 is required to describe vSafe5V. Validate before selecting. */
		pdo = get_unaligned_le32(caps + 4);
		if (pdo_type(pdo) != PDO_TYPE_FIXED || pdo_fixed_voltage(pdo) != 5000)
			return -EPROTO;
		if (s->five_volt_requested)
			return 0;
		ret = s2mm005_command(s, select, sizeof(select));
		if (ret)
			return ret;
		ret = s2mm005_command(s, negotiate, sizeof(negotiate));
		if (!ret)
			s->five_volt_requested = true;
		return ret;
	}
	*ua = ret;
	s->pd = true;
	s->five_volt_requested = false;
	return 0;
}

static int s2mm005_update(struct s2mm005 *s)
{
	struct typec_partner_desc partner = {};
	struct s2mm005_state state;
	u8 func_buf[4], lp_buf[4], irq_buf[48];
	const u8 ack[] = { 0x01 };
	const u8 auto_lp[] = { 0x0f, 0x06 };
	const u8 drp[] = { 0x02, 0x01, 0x00, 0x50, 0x03 };
	u32 func, lp;
	int ret, ua = 0;

	ret = s2mm005_read(s, S2MM005_FUNC, func_buf, sizeof(func_buf));
	if (ret)
		return ret;
	ret = s2mm005_read(s, S2MM005_LP, lp_buf, sizeof(lp_buf));
	if (ret)
		return ret;
	ret = s2mm005_read(s, S2MM005_IRQ, irq_buf, sizeof(irq_buf));
	if (ret)
		return ret;
	func = get_unaligned_le32(func_buf);
	lp = get_unaligned_le32(lp_buf);
	ret = s2mm005_command(s, ack, sizeof(ack));
	if (ret)
		return ret;
	if (!(lp & BIT(0))) {
		ret = s2mm005_command(s, auto_lp, sizeof(auto_lp));
		if (ret)
			return ret;
	}
	if (func & S2MM005_RESET)
		s->drp_restored = false;
	state = s2mm005_decode(func, lp);
	if (func != s->last_func || lp != s->last_lp) {
		dev_info(&s->client->dev,
			 "state=%u data=%s power=%s status=%#x lp=%#x\n",
			 (unsigned int)FIELD_GET(S2MM005_STATE, func),
			 state.role == USB_ROLE_HOST ? "host" :
			 state.role == USB_ROLE_DEVICE ? "device" : "none",
			 state.source ? "source" : "sink", func, lp);
		s->last_func = func;
		s->last_lp = lp;
	}
	if (state.role == USB_ROLE_NONE) {
		ret = s2mm005_disconnect(s);
		if (ret)
			return ret;
		/* Do not disturb an attached PC, a wet port or error recovery. */
		if (!FIELD_GET(S2MM005_STATE, func) && !s->drp_restored &&
		    (lp & S2MM005_DRY) && !(lp & S2MM005_WATER)) {
			ret = s2mm005_command(s, drp, sizeof(drp));
			if (!ret)
				s->drp_restored = true;
		}
		return ret;
	}
	s->drp_restored = false;

	/* Invalidate the previous sink grant before changing any power role. */
	if (state.source || FIELD_GET(S2MM005_STATE, func) == 52 ||
	    (func & S2MM005_RESET))
		s2mm005_supply(s, 0, 0);
	if (s->sourcing && !state.source) {
		ret = regulator_disable(s->vbus);
		if (ret)
			return ret;
		s->sourcing = false;
	}
	if (!s->mux_selected) {
		ret = mux_control_select(s->mux, 1);
		if (ret)
			return ret;
		s->mux_selected = true;
	}
	if (s->role != state.role) {
		s->role_dirty = true;
		ret = usb_role_switch_set_role(s->role_sw, state.role);
		if (ret)
			return ret;
		s->role = state.role;
		s->role_dirty = false;
	}
	if (state.source && !s->sourcing) {
		ret = regulator_enable(s->vbus);
		if (ret)
			return ret;
		s->sourcing = true;
	}
	if (!state.source &&
	    ((FIELD_GET(S2MM005_STATE, func) >= 17 &&
	      FIELD_GET(S2MM005_STATE, func) <= 21) ||
	     FIELD_GET(S2MM005_STATE, func) == 29)) {
		ret = s2mm005_current(s, func, &ua);
		if (ret)
			return ret;
		s2mm005_supply(s, ua ? 5000000 : 0, ua);
	} else {
		s2mm005_supply(s, 0, 0);
	}
	if (state.source && FIELD_GET(S2MM005_STATE, func) == 6)
		s->pd = true;
	if (!s->partner) {
		partner.usb_pd = s->pd;
		s->partner = typec_register_partner(s->port, &partner);
		if (IS_ERR(s->partner)) {
			ret = PTR_ERR(s->partner);
			s->partner = NULL;
			return ret;
		}
	}
	typec_set_data_role(s->port, state.role == USB_ROLE_HOST ? TYPEC_HOST : TYPEC_DEVICE);
	typec_set_pwr_role(s->port, state.source ? TYPEC_SOURCE : TYPEC_SINK);
	typec_set_pwr_opmode(s->port, s->pd ? TYPEC_PWR_MODE_PD :
			    ua >= 3000000 ? TYPEC_PWR_MODE_3_0A :
			    ua >= 1500000 ? TYPEC_PWR_MODE_1_5A : TYPEC_PWR_MODE_USB);
	return 0;
}

static void s2mm005_service(struct s2mm005 *s)
{
	int ret;

	mutex_lock(&s->lock);
	if (s->stopped || s->suspended)
		goto out;
	ret = s2mm005_update(s);
	if (ret) {
		dev_err_ratelimited(&s->client->dev, "port update failed: %d\n", ret);
		if (s2mm005_disconnect(s))
			dev_err_ratelimited(&s->client->dev, "port shutdown failed\n");
		/* A failed clear must not leave a level interrupt spinning. */
		if (!s->irq_disabled) {
			disable_irq_nosync(s->client->irq);
			s->irq_disabled = true;
		}
	} else if (s->irq_disabled) {
		enable_irq(s->client->irq);
		s->irq_disabled = false;
	}
	mod_delayed_work(system_wq, &s->work, HZ);
out:
	mutex_unlock(&s->lock);
}

static void s2mm005_work(struct work_struct *work)
{
	struct s2mm005 *s = container_of(to_delayed_work(work), struct s2mm005, work);

	s2mm005_service(s);
}

static irqreturn_t s2mm005_irq(int irq, void *data)
{
	s2mm005_service(data);
	return IRQ_HANDLED;
}

static const enum power_supply_property s2mm005_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int s2mm005_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct s2mm005 *s = power_supply_get_drvdata(psy);
	unsigned long flags;
	int ret = 0;

	/* Regulator callbacks may hold the charger lock: never take s->lock here. */
	spin_lock_irqsave(&s->supply_lock, flags);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = s->supply_ua > 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = s->supply_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = s->supply_ua;
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&s->supply_lock, flags);
	return ret;
}

static const struct power_supply_desc s2mm005_supply_desc = {
	.name = "s2mm005-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = s2mm005_props,
	.num_properties = ARRAY_SIZE(s2mm005_props),
	.get_property = s2mm005_get_property,
};

static void s2mm005_stop(void *data)
{
	struct s2mm005 *s = data;

	mutex_lock(&s->lock);
	if (s->stopped) {
		mutex_unlock(&s->lock);
		return;
	}
	s->stopped = true;
	mutex_unlock(&s->lock);
	disable_irq(s->client->irq);
	cancel_delayed_work_sync(&s->work);
	mutex_lock(&s->lock);
	if (s2mm005_disconnect(s))
		dev_err(&s->client->dev, "failed to disconnect port\n");
	mutex_unlock(&s->lock);
	device_init_wakeup(&s->client->dev, false);
}

static void s2mm005_put_port(void *data)
{
	struct s2mm005 *s = data;

	typec_unregister_port(s->port);
}

static void s2mm005_put_role(void *data)
{
	struct s2mm005 *s = data;

	usb_role_switch_put(s->role_sw);
	fwnode_handle_put(s->connector);
}

static int s2mm005_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_capability cap = {};
	struct power_supply_config psy_cfg = {};
	struct s2mm005 *s;
	u8 version[4];
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->client = client;
	mutex_init(&s->lock);
	spin_lock_init(&s->supply_lock);
	INIT_DELAYED_WORK(&s->work, s2mm005_work);
	i2c_set_clientdata(client, s);
	s->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(s->vbus))
		return dev_err_probe(dev, PTR_ERR(s->vbus), "no VBUS regulator\n");
	s->mux = devm_mux_control_get(dev, "usb");
	if (IS_ERR(s->mux))
		return dev_err_probe(dev, PTR_ERR(s->mux), "no USB mux\n");
	s->connector = device_get_named_child_node(dev, "connector");
	if (!s->connector)
		return -EINVAL;
	s->role_sw = fwnode_usb_role_switch_get(s->connector);
	if (IS_ERR_OR_NULL(s->role_sw)) {
		ret = s->role_sw ? PTR_ERR(s->role_sw) : -ENODEV;
		fwnode_handle_put(s->connector);
		return dev_err_probe(dev, ret, "no USB role switch\n");
	}
	ret = devm_add_action_or_reset(dev, s2mm005_put_role, s);
	if (ret)
		return ret;
	ret = s2mm005_read(s, 0x0008, version, sizeof(version));
	if (ret)
		return dev_err_probe(dev, ret, "cannot read firmware version\n");
	if ((!version[0] && !version[1] && !version[2] && !version[3]) ||
	    (version[0] == 0xff && version[1] == 0xff &&
	     version[2] == 0xff && version[3] == 0xff))
		return dev_err_probe(dev, -ENODEV, "no resident firmware\n");
	cap.type = TYPEC_PORT_DRP;
	cap.data = TYPEC_PORT_DRD;
	cap.revision = 0x0120;
	cap.pd_revision = 0x0200;
	cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	cap.fwnode = s->connector;
	cap.driver_data = s;
	s->port = typec_register_port(dev, &cap);
	if (IS_ERR(s->port))
		return PTR_ERR(s->port);
	ret = devm_add_action_or_reset(dev, s2mm005_put_port, s);
	if (ret)
		return ret;
	psy_cfg.drv_data = s;
	psy_cfg.of_node = dev->of_node;
	s->psy = devm_power_supply_register(dev, &s2mm005_supply_desc, &psy_cfg);
	if (IS_ERR(s->psy))
		return PTR_ERR(s->psy);
	ret = devm_request_threaded_irq(dev, client->irq, NULL, s2mm005_irq,
					IRQF_ONESHOT, dev_name(dev), s);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, s2mm005_stop, s);
	if (ret)
		return ret;
	device_init_wakeup(dev, true);
	dev_info(dev, "resident firmware %*ph\n", (int)sizeof(version), version);
	/* Read initial state even when the cable was connected before boot. */
	s2mm005_service(s);
	return 0;
}

static void s2mm005_shutdown(struct i2c_client *client)
{
	s2mm005_stop(i2c_get_clientdata(client));
}

static int s2mm005_suspend(struct device *dev)
{
	struct s2mm005 *s = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&s->lock);
	s->suspended = true;
	mutex_unlock(&s->lock);
	cancel_delayed_work_sync(&s->work);
	disable_irq(s->client->irq);
	if (device_may_wakeup(dev))
		ret = enable_irq_wake(s->client->irq);
	if (ret) {
		mutex_lock(&s->lock);
		s->suspended = false;
		mutex_unlock(&s->lock);
		enable_irq(s->client->irq);
		mod_delayed_work(system_wq, &s->work, 0);
	}
	return ret;
}

static int s2mm005_resume(struct device *dev)
{
	struct s2mm005 *s = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(s->client->irq);
	mutex_lock(&s->lock);
	s->suspended = false;
	mutex_unlock(&s->lock);
	enable_irq(s->client->irq);
	s2mm005_service(s);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s2mm005_pm, s2mm005_suspend, s2mm005_resume);

static const struct of_device_id s2mm005_match[] = {
	{ .compatible = "samsung,s2mm005" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mm005_match);

static struct i2c_driver s2mm005_driver = {
	.driver = {
		.name = "s2mm005",
		.of_match_table = s2mm005_match,
		.pm = pm_sleep_ptr(&s2mm005_pm),
	},
	.probe = s2mm005_probe,
	.shutdown = s2mm005_shutdown,
};
module_i2c_driver(s2mm005_driver);

MODULE_DESCRIPTION("Samsung S2MM005 Type-C port controller");
MODULE_LICENSE("GPL");
