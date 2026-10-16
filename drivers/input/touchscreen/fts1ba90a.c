// SPDX-License-Identifier: GPL-2.0
/*
 * STMicroelectronics FTS1BA90A touchscreen driver
 *
 * Derived from the protocol of Samsung's stm_fts1ba90a driver for the
 * Galaxy Tab A 10.1 (2019, SM-T510): I2C address 0x49, chip ID at 0x22
 * (0x39 0x36), 8-byte events read through 0x60/0x61.
 *
 * The controller is powered while its input device is open. It senses
 * normally while the panel it follows is on; while the panel is off or the
 * system sleeps it is put in low-power mode, where a double tap reports
 * KEY_WAKEUP, if it may wake the system, and powered off otherwise. ESD
 * errors from the controller are recovered by power cycling it. While an
 * external supply is connected, the controller is told so, which makes it
 * filter charger noise.
 *
 * Firmware update and Samsung's factory interfaces are not supported.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#define FTS1BA90A_DEV_NAME		"fts1ba90a"

#define FTS_CMD_SENSE_ON		0x10
#define FTS_CMD_FORCE_CALIBRATION	0x13
#define FTS_READ_DEVICE_ID		0x22
#define FTS_READ_FW_VERSION		0x24
#define FTS_CMD_SET_TOUCHTYPE		0x30
#define FTS_CMD_SET_OPMODE		0x31
#define FTS_CMD_SET_CHARGER_MODE	0x32
#define FTS_CMD_WAKEUP_GESTURE		0x39
#define FTS_READ_ONE_EVENT		0x60
#define FTS_READ_ALL_EVENT		0x61
#define FTS_CMD_CLEAR_ALL_EVENT		0x62
#define FTS_CMD_SET_SCANMODE		0xa0
#define FTS_CMD_SPONGE_WRITE		0xaa
#define FTS_CMD_SPONGE_NOTIFY		0xc0

#define FTS_ID0				0x39
#define FTS_ID1				0x36

#define FTS_FIFO_MAX			32
#define FTS_EVENT_SIZE			8
#define FTS_FINGER_MAX			10
#define FTS_I2C_RETRY_CNT		3
#define FTS_INIT_RETRY_CNT		3

/* event id, byte 0 bits 1:0 */
#define FTS_EV_COORDINATE		0
#define FTS_EV_STATUS			1
#define FTS_EV_GESTURE			2

/* coordinate event action, byte 0 bits 7:6 */
#define FTS_ACTION_PRESS		1
#define FTS_ACTION_MOVE			2
#define FTS_ACTION_RELEASE		3

/* coordinate event touch types */
#define FTS_TTYPE_NORMAL		0
#define FTS_TTYPE_GLOVE			3
#define FTS_TTYPE_PALM			5
#define FTS_TTYPE_WET			6

/* status event types (byte 0 bits 5:2) and ids (byte 1) */
#define FTS_STYPE_ERROR			1
#define FTS_STYPE_INFO			2
#define FTS_ERR_QUEUE_FULL		0x01
#define FTS_ERR_ESD			0x02
#define FTS_INFO_READY			0x00
#define FTS_INFO_WET_MODE		0x01
#define FTS_INFO_NOISE_MODE		0x02

/* gesture events: Samsung library (byte 0 bits 7:6 = 1) or vendor */
#define FTS_GESTURE_SAMSUNG		1
#define FTS_SPONGE_DOUBLETAP		1
#define FTS_VENDOR_DOUBLETAP		0x01

/* touch types to report: touch | palm | wet */
#define FTS_TOUCHTYPE_DEFAULT		(0x01 | 0x20 | 0x40)
#define FTS_SCANMODE_MS_SS		0x01
#define FTS_OPMODE_NORMAL		0
#define FTS_OPMODE_LOWPOWER		1
#define FTS_CHARGER_WIRED		0x01
/* Samsung gesture library: mode byte at offset 0, double tap is bit 2 */
#define FTS_SPONGE_OFFSET_MODE		0x0000
#define FTS_SPONGE_MODE_DOUBLETAP	BIT(2)

/* Look for the panel every second for two minutes, then every 10 s. */
#define FTS_PANEL_RETRY_MS		1000
#define FTS_PANEL_RETRY_CNT		120
#define FTS_PANEL_RETRY_SLOW_MS		10000

enum fts1ba90a_state {
	FTS_STATE_OFF,
	FTS_STATE_LOWPOWER,
	FTS_STATE_ACTIVE,
};

struct fts1ba90a_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator *vdd;
	struct regulator *avdd;
	/* serializes I2C transfers between the IRQ thread and commands */
	struct mutex i2c_lock;

	/* state and the inputs that decide it, under @lock */
	struct mutex lock;
	enum fts1ba90a_state state;
	bool irq_enabled;
	bool opened;
	bool panel_on;
	bool suspended;
	bool charger;

	struct drm_panel_follower follower;
	bool follower_added;
	bool panel_pending;
	unsigned int panel_retries;
	struct delayed_work panel_work;
	struct work_struct update_work;

	struct work_struct reset_work;
	struct work_struct charger_work;
	struct notifier_block psy_nb;
};

static int fts1ba90a_write(struct fts1ba90a_data *ts, const u8 *buf, u16 len)
{
	struct i2c_msg msg = {
		.addr = ts->client->addr,
		.flags = 0,
		.len = len,
		.buf = (u8 *)buf,
	};
	int ret, retry = FTS_I2C_RETRY_CNT;

	mutex_lock(&ts->i2c_lock);
	do {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret >= 0)
			break;
		usleep_range(10000, 11000);
	} while (--retry > 0);
	mutex_unlock(&ts->i2c_lock);

	return ret < 0 ? ret : 0;
}

static int fts1ba90a_read(struct fts1ba90a_data *ts,
			  const u8 *wbuf, u16 wlen, u8 *rbuf, u16 rlen)
{
	struct i2c_msg msgs[2] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = wlen,
			.buf = (u8 *)wbuf,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = rlen,
			.buf = rbuf,
		},
	};
	int ret, retry = FTS_I2C_RETRY_CNT;

	mutex_lock(&ts->i2c_lock);
	do {
		ret = i2c_transfer(ts->client->adapter, msgs, 2);
		if (ret >= 0)
			break;
		usleep_range(10000, 11000);
	} while (--retry > 0);
	mutex_unlock(&ts->i2c_lock);

	return ret < 0 ? ret : 0;
}

static int fts1ba90a_command(struct fts1ba90a_data *ts, const u8 *buf,
			     u16 len, unsigned int delay_ms)
{
	int ret = fts1ba90a_write(ts, buf, len);

	if (!ret && delay_ms)
		msleep(delay_ms);
	return ret;
}

static void fts1ba90a_release_all(struct fts1ba90a_data *ts)
{
	int i;

	for (i = 0; i < FTS_FINGER_MAX; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void fts1ba90a_handle_coordinate(struct fts1ba90a_data *ts,
					const u8 *ev)
{
	unsigned int tid = (ev[0] >> 2) & 0x0f;
	unsigned int action = (ev[0] >> 6) & 0x03;
	unsigned int x = (ev[1] << 4) | ((ev[3] >> 4) & 0x0f);
	unsigned int y = (ev[2] << 4) | (ev[3] & 0x0f);
	unsigned int z = ev[6] & 0x3f;
	unsigned int ttype = ((ev[6] >> 6) & 0x03) << 2 |
			     ((ev[7] >> 6) & 0x03);
	int tool;

	if (tid >= FTS_FINGER_MAX) {
		dev_dbg(&ts->client->dev, "tid %u out of range\n", tid);
		return;
	}

	switch (ttype) {
	case FTS_TTYPE_NORMAL:
	case FTS_TTYPE_GLOVE:
	case FTS_TTYPE_WET:
		tool = MT_TOOL_FINGER;
		break;
	case FTS_TTYPE_PALM:
		tool = MT_TOOL_PALM;
		break;
	default:
		return;
	}

	input_mt_slot(ts->input, tid);

	switch (action) {
	case FTS_ACTION_PRESS:
	case FTS_ACTION_MOVE:
		/*
		 * The slot state comes from these events alone, so a lost
		 * press or release cannot leave BTN_TOUCH out of step.
		 */
		input_mt_report_slot_state(ts->input, tool, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ev[4]);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, ev[5]);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z ?: 1);
		break;
	case FTS_ACTION_RELEASE:
		input_mt_report_slot_inactive(ts->input);
		break;
	default:
		break;
	}
}

static void fts1ba90a_handle_status(struct fts1ba90a_data *ts, const u8 *ev)
{
	unsigned int stype = (ev[0] >> 2) & 0x0f;

	if (stype == FTS_STYPE_ERROR && ev[1] == FTS_ERR_QUEUE_FULL) {
		dev_warn(&ts->client->dev, "event queue full\n");
		fts1ba90a_release_all(ts);
	} else if (stype == FTS_STYPE_ERROR && ev[1] == FTS_ERR_ESD) {
		dev_err(&ts->client->dev, "ESD detected, resetting\n");
		schedule_work(&ts->reset_work);
	} else if (stype == FTS_STYPE_INFO && ev[1] == FTS_INFO_WET_MODE) {
		dev_dbg(&ts->client->dev, "wet mode %s\n",
			ev[2] ? "on" : "off");
	} else if (stype == FTS_STYPE_INFO && ev[1] == FTS_INFO_NOISE_MODE) {
		dev_dbg(&ts->client->dev, "noise mode %s\n",
			ev[2] ? "on" : "off");
	}
}

static void fts1ba90a_handle_gesture(struct fts1ba90a_data *ts, const u8 *ev)
{
	unsigned int sf = (ev[0] >> 6) & 0x03;
	unsigned int stype = (ev[0] >> 2) & 0x0f;
	bool doubletap;

	if (sf == FTS_GESTURE_SAMSUNG)
		doubletap = stype == FTS_SPONGE_DOUBLETAP;
	else
		doubletap = ev[1] == FTS_VENDOR_DOUBLETAP;

	/* The interrupt is disabled while @state changes. */
	if (!doubletap || ts->state != FTS_STATE_LOWPOWER)
		return;

	pm_wakeup_event(&ts->client->dev, 0);
	input_report_key(ts->input, KEY_WAKEUP, 1);
	input_sync(ts->input);
	input_report_key(ts->input, KEY_WAKEUP, 0);
	input_sync(ts->input);
}

static irqreturn_t fts1ba90a_irq(int irq, void *dev_id)
{
	struct fts1ba90a_data *ts = dev_id;
	u8 cmd;
	u8 data[FTS_FIFO_MAX * FTS_EVENT_SIZE];
	int left, i, ret;

	cmd = FTS_READ_ONE_EVENT;
	ret = fts1ba90a_read(ts, &cmd, 1, data, FTS_EVENT_SIZE);
	if (ret) {
		dev_dbg(&ts->client->dev, "failed to read one event: %d\n", ret);
		return IRQ_HANDLED;
	}

	left = data[7] & 0x3f;
	if (left >= FTS_FIFO_MAX)
		left = FTS_FIFO_MAX - 1;

	if (left > 0) {
		cmd = FTS_READ_ALL_EVENT;
		ret = fts1ba90a_read(ts, &cmd, 1,
				     &data[FTS_EVENT_SIZE],
				     left * FTS_EVENT_SIZE);
		if (ret) {
			dev_dbg(&ts->client->dev, "failed to read all events: %d\n", ret);
			return IRQ_HANDLED;
		}
	}

	for (i = 0; i <= left; i++) {
		const u8 *ev = &data[i * FTS_EVENT_SIZE];

		switch (ev[0] & 0x03) {
		case FTS_EV_COORDINATE:
			fts1ba90a_handle_coordinate(ts, ev);
			break;
		case FTS_EV_STATUS:
			fts1ba90a_handle_status(ts, ev);
			break;
		case FTS_EV_GESTURE:
			fts1ba90a_handle_gesture(ts, ev);
			break;
		default:
			/* vendor events: command echoes and debug output */
			break;
		}
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static int fts1ba90a_wait_ready(struct fts1ba90a_data *ts)
{
	int i;

	for (i = 0; i < 30; i++) {
		u8 cmd = FTS_READ_ONE_EVENT;
		u8 data[FTS_EVENT_SIZE];
		int ret = fts1ba90a_read(ts, &cmd, 1, data, sizeof(data));

		if (!ret && (data[0] & 0x03) == FTS_EV_STATUS &&
		    ((data[0] >> 2) & 0x0f) == FTS_STYPE_INFO &&
		    data[1] == FTS_INFO_READY)
			return 0;

		msleep(20);
	}

	return -ETIMEDOUT;
}

static int fts1ba90a_set_charger(struct fts1ba90a_data *ts)
{
	u8 buf[2] = { FTS_CMD_SET_CHARGER_MODE,
		      ts->charger ? FTS_CHARGER_WIRED : 0 };

	return fts1ba90a_command(ts, buf, sizeof(buf), 0);
}

/*
 * Bring a powered controller into normal sensing; @probe also checks its ID,
 * logs its firmware and calibrates it, as the vendor driver does once.
 */
static int fts1ba90a_init_chip(struct fts1ba90a_data *ts, bool probe)
{
	static const u8 reset_seq[] = { 0xfa, 0x20, 0x00, 0x00, 0x24, 0x81 };
	static const u8 touchtype[] = { FTS_CMD_SET_TOUCHTYPE,
					FTS_TOUCHTYPE_DEFAULT & 0xff,
					FTS_TOUCHTYPE_DEFAULT >> 8 };
	static const u8 scanmode[] = { FTS_CMD_SET_SCANMODE, 0x00,
				       FTS_SCANMODE_MS_SS };
	struct device *dev = &ts->client->dev;
	u8 cmd;
	u8 id[5];
	u8 ver[FTS_EVENT_SIZE];
	int ret;

	/* system reset, as the vendor fts_systemreset() */
	ret = fts1ba90a_command(ts, reset_seq, sizeof(reset_seq), 10);
	if (ret)
		return ret;

	ret = fts1ba90a_wait_ready(ts);
	if (ret) {
		dev_err(dev, "timeout waiting for ready\n");
		return ret;
	}

	if (probe) {
		cmd = FTS_READ_DEVICE_ID;
		ret = fts1ba90a_read(ts, &cmd, 1, id, sizeof(id));
		if (ret) {
			dev_err(dev, "failed to read chip id: %d\n", ret);
			return ret;
		}
		if (id[2] != FTS_ID0 || id[3] != FTS_ID1) {
			dev_err(dev, "unexpected chip id %02x %02x (want %02x %02x)\n",
				id[2], id[3], FTS_ID0, FTS_ID1);
			return -ENODEV;
		}

		cmd = FTS_READ_FW_VERSION;
		ret = fts1ba90a_read(ts, &cmd, 1, ver, sizeof(ver));
		if (ret)
			return ret;
		dev_info(dev, "chip %02x%02x rev %02x, firmware %04x, config %04x, main %04x\n",
			 id[2], id[3], id[4], ver[0] << 8 | ver[1],
			 ver[2] << 8 | ver[3], ver[5] << 8 | ver[4]);
	}

	ret = fts1ba90a_command(ts, touchtype, sizeof(touchtype), 10);
	if (ret)
		return ret;

	if (probe) {
		cmd = FTS_CMD_FORCE_CALIBRATION;
		ret = fts1ba90a_command(ts, &cmd, 1, 20);
		if (ret)
			return ret;
	}

	cmd = FTS_CMD_CLEAR_ALL_EVENT;
	ret = fts1ba90a_command(ts, &cmd, 1, 10);
	if (ret)
		return ret;

	ret = fts1ba90a_set_charger(ts);
	if (ret)
		return ret;

	ret = fts1ba90a_command(ts, scanmode, sizeof(scanmode), 10);
	if (ret)
		return ret;

	cmd = FTS_CMD_SENSE_ON;
	return fts1ba90a_command(ts, &cmd, 1, 20);
}

static void fts1ba90a_irq_enable(struct fts1ba90a_data *ts, bool enable)
{
	if (ts->irq_enabled == enable)
		return;
	if (enable)
		enable_irq(ts->client->irq);
	else
		disable_irq(ts->client->irq);
	ts->irq_enabled = enable;
}

static int fts1ba90a_power_on(struct fts1ba90a_data *ts)
{
	int ret;

	pinctrl_pm_select_default_state(&ts->client->dev);

	/* vendor order: avdd 3.3 V, 1 ms, dvdd 1.8 V */
	ret = regulator_enable(ts->avdd);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = regulator_enable(ts->vdd);
	if (ret) {
		regulator_disable(ts->avdd);
		return ret;
	}
	msleep(20);

	return 0;
}

static void fts1ba90a_power_off(struct fts1ba90a_data *ts)
{
	regulator_disable(ts->vdd);
	regulator_disable(ts->avdd);
	/* the unpowered controller no longer drives its interrupt line */
	pinctrl_pm_select_sleep_state(&ts->client->dev);
}

/* Power the controller and start sensing; with @probe, retry on failure. */
static int fts1ba90a_start(struct fts1ba90a_data *ts, bool probe)
{
	int retry = probe ? FTS_INIT_RETRY_CNT : 1;
	int ret;

	do {
		ret = fts1ba90a_power_on(ts);
		if (ret)
			return ret;
		ret = fts1ba90a_init_chip(ts, probe);
		if (!ret)
			return 0;
		fts1ba90a_power_off(ts);
		/* the vendor driver discharges the controller for 500 ms */
		msleep(500);
	} while (--retry > 0);

	return ret;
}

/* As the vendor fts_set_opmode() and fts_write_to_sponge() */
static int fts1ba90a_set_opmode(struct fts1ba90a_data *ts, bool lowpower)
{
	const u8 opmode[] = { FTS_CMD_SET_OPMODE,
			      lowpower ? FTS_OPMODE_LOWPOWER : FTS_OPMODE_NORMAL };
	static const u8 gesture[] = { FTS_CMD_WAKEUP_GESTURE, 0x02 };
	/* Samsung's gesture library: enable double tap, then notify it */
	static const u8 sponge_mode[] = { FTS_CMD_SPONGE_WRITE,
					  FTS_SPONGE_OFFSET_MODE >> 8,
					  FTS_SPONGE_OFFSET_MODE & 0xff,
					  FTS_SPONGE_MODE_DOUBLETAP };
	static const u8 sponge_notify[] = { FTS_CMD_SPONGE_NOTIFY,
					    FTS_SPONGE_OFFSET_MODE >> 8,
					    FTS_SPONGE_OFFSET_MODE & 0xff };
	int ret;

	ret = fts1ba90a_command(ts, opmode, sizeof(opmode), 20);
	if (ret || !lowpower)
		return ret;

	ret = fts1ba90a_command(ts, gesture, sizeof(gesture), 10);
	if (ret)
		return ret;

	ret = fts1ba90a_command(ts, sponge_mode, sizeof(sponge_mode), 0);
	if (ret)
		return ret;

	return fts1ba90a_command(ts, sponge_notify, sizeof(sponge_notify), 10);
}

static enum fts1ba90a_state fts1ba90a_wanted(struct fts1ba90a_data *ts)
{
	if (!ts->opened)
		return FTS_STATE_OFF;
	if (ts->panel_on && !ts->suspended)
		return FTS_STATE_ACTIVE;
	if (device_may_wakeup(&ts->client->dev))
		return FTS_STATE_LOWPOWER;
	return FTS_STATE_OFF;
}

/* Move to the wanted state; called with @lock held. */
static void fts1ba90a_update(struct fts1ba90a_data *ts, bool force_reset)
{
	struct device *dev = &ts->client->dev;
	enum fts1ba90a_state want = fts1ba90a_wanted(ts);
	int ret = 0;

	if (want == ts->state && !force_reset)
		return;

	fts1ba90a_irq_enable(ts, false);
	fts1ba90a_release_all(ts);

	if (ts->state != FTS_STATE_OFF &&
	    (want == FTS_STATE_OFF || force_reset)) {
		fts1ba90a_power_off(ts);
		ts->state = FTS_STATE_OFF;
		if (force_reset)
			msleep(500);
	}

	if (want == FTS_STATE_OFF)
		return;

	if (ts->state == FTS_STATE_OFF) {
		ret = fts1ba90a_start(ts, false);
		if (ret) {
			dev_err(dev, "failed to start: %d\n", ret);
			return;
		}
		ts->state = FTS_STATE_ACTIVE;
	}

	if (want != ts->state) {
		ret = fts1ba90a_set_opmode(ts, want == FTS_STATE_LOWPOWER);
		if (ret && want == FTS_STATE_ACTIVE) {
			/* leaving low power failed: start again from scratch */
			fts1ba90a_power_off(ts);
			ts->state = FTS_STATE_OFF;
			msleep(500);
			ret = fts1ba90a_start(ts, false);
		}
		if (ret) {
			dev_err(dev, "failed to enter %s mode: %d\n",
				want == FTS_STATE_LOWPOWER ? "low-power" : "normal",
				ret);
			if (ts->state == FTS_STATE_OFF)
				return;
		}
		ts->state = want;
	}

	fts1ba90a_irq_enable(ts, true);
}

static void fts1ba90a_update_work(struct work_struct *work)
{
	struct fts1ba90a_data *ts = container_of(work, struct fts1ba90a_data,
						 update_work);

	mutex_lock(&ts->lock);
	if (!ts->suspended)
		fts1ba90a_update(ts, false);
	mutex_unlock(&ts->lock);
}

static void fts1ba90a_reset_work(struct work_struct *work)
{
	struct fts1ba90a_data *ts = container_of(work, struct fts1ba90a_data,
						 reset_work);

	mutex_lock(&ts->lock);
	if (ts->state != FTS_STATE_OFF && !ts->suspended)
		fts1ba90a_update(ts, true);
	mutex_unlock(&ts->lock);
}

static int fts1ba90a_open(struct input_dev *input)
{
	struct fts1ba90a_data *ts = input_get_drvdata(input);

	mutex_lock(&ts->lock);
	ts->opened = true;
	fts1ba90a_update(ts, false);
	mutex_unlock(&ts->lock);

	return 0;
}

static void fts1ba90a_close(struct input_dev *input)
{
	struct fts1ba90a_data *ts = input_get_drvdata(input);

	mutex_lock(&ts->lock);
	ts->opened = false;
	fts1ba90a_update(ts, false);
	mutex_unlock(&ts->lock);
}

static void fts1ba90a_charger_work(struct work_struct *work)
{
	struct fts1ba90a_data *ts = container_of(work, struct fts1ba90a_data,
						 charger_work);
	bool charger = power_supply_is_system_supplied() > 0;

	mutex_lock(&ts->lock);
	if (charger != ts->charger) {
		ts->charger = charger;
		dev_dbg(&ts->client->dev, "charger %s\n",
			charger ? "connected" : "disconnected");
		if (ts->state != FTS_STATE_OFF && !ts->suspended &&
		    fts1ba90a_set_charger(ts))
			dev_warn(&ts->client->dev, "failed to set charger mode\n");
	}
	mutex_unlock(&ts->lock);
}

static int fts1ba90a_psy_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct fts1ba90a_data *ts = container_of(nb, struct fts1ba90a_data,
						 psy_nb);

	if (event == PSY_EVENT_PROP_CHANGED)
		schedule_work(&ts->charger_work);
	return NOTIFY_OK;
}

/*
 * Powering and starting the controller takes up to a few hundred ms; do it
 * from a work item instead of holding up the display.
 */
static void fts1ba90a_panel_changed(struct drm_panel_follower *follower,
				    bool on)
{
	struct fts1ba90a_data *ts = container_of(follower,
						 struct fts1ba90a_data,
						 follower);

	mutex_lock(&ts->lock);
	if (ts->follower_added)
		ts->panel_on = on;
	else
		ts->panel_pending = on;
	mutex_unlock(&ts->lock);

	schedule_work(&ts->update_work);
}

static int fts1ba90a_panel_prepared(struct drm_panel_follower *follower)
{
	fts1ba90a_panel_changed(follower, true);
	return 0;
}

static int fts1ba90a_panel_unpreparing(struct drm_panel_follower *follower)
{
	fts1ba90a_panel_changed(follower, false);
	return 0;
}

static const struct drm_panel_follower_funcs fts1ba90a_follower_funcs = {
	.panel_prepared = fts1ba90a_panel_prepared,
	.panel_unpreparing = fts1ba90a_panel_unpreparing,
};

/*
 * Follow the panel once its driver is there. Until then the panel is taken
 * to be on, so the touchscreen works even without that driver.
 */
static void fts1ba90a_panel_work(struct work_struct *work)
{
	struct fts1ba90a_data *ts = container_of(to_delayed_work(work),
						 struct fts1ba90a_data,
						 panel_work);
	struct device *dev = &ts->client->dev;
	int ret;

	ts->follower.funcs = &fts1ba90a_follower_funcs;
	/* drm_panel_add_follower() calls panel_prepared if the panel is on */
	mutex_lock(&ts->lock);
	ts->panel_pending = false;
	mutex_unlock(&ts->lock);
	ret = drm_panel_add_follower(dev, &ts->follower);
	if (ret == -EPROBE_DEFER) {
		/*
		 * The panel driver may only load from the root filesystem,
		 * after the touchscreen has served the initramfs (e.g. to
		 * unlock an encrypted root), so never stop looking.
		 */
		if (ts->panel_retries < FTS_PANEL_RETRY_CNT &&
		    ++ts->panel_retries == FTS_PANEL_RETRY_CNT)
			dev_info(dev, "panel not found yet, still looking\n");
		schedule_delayed_work(&ts->panel_work,
				      msecs_to_jiffies(ts->panel_retries < FTS_PANEL_RETRY_CNT ?
						       FTS_PANEL_RETRY_MS :
						       FTS_PANEL_RETRY_SLOW_MS));
		return;
	}
	if (ret) {
		dev_warn(dev, "failed to follow the panel: %d\n", ret);
		return;
	}

	mutex_lock(&ts->lock);
	ts->follower_added = true;
	ts->panel_on = ts->panel_pending;
	mutex_unlock(&ts->lock);
	schedule_work(&ts->update_work);
	dev_info(dev, "following the panel\n");
}

static void fts1ba90a_cleanup(void *data)
{
	struct fts1ba90a_data *ts = data;

	cancel_delayed_work_sync(&ts->panel_work);
	if (ts->follower_added)
		drm_panel_remove_follower(&ts->follower);
	power_supply_unreg_notifier(&ts->psy_nb);
	cancel_work_sync(&ts->update_work);
	cancel_work_sync(&ts->charger_work);
	cancel_work_sync(&ts->reset_work);

	mutex_lock(&ts->lock);
	fts1ba90a_irq_enable(ts, false);
	if (ts->state != FTS_STATE_OFF)
		fts1ba90a_power_off(ts);
	ts->state = FTS_STATE_OFF;
	mutex_unlock(&ts->lock);
}

static int fts1ba90a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fts1ba90a_data *ts;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	mutex_init(&ts->i2c_lock);
	mutex_init(&ts->lock);
	INIT_WORK(&ts->update_work, fts1ba90a_update_work);
	INIT_WORK(&ts->reset_work, fts1ba90a_reset_work);
	INIT_WORK(&ts->charger_work, fts1ba90a_charger_work);
	INIT_DELAYED_WORK(&ts->panel_work, fts1ba90a_panel_work);
	ts->psy_nb.notifier_call = fts1ba90a_psy_notify;
	ts->panel_on = true;
	i2c_set_clientdata(client, ts);

	ts->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(ts->avdd))
		return dev_err_probe(dev, PTR_ERR(ts->avdd),
				     "failed to get avdd\n");

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "failed to get vdd\n");

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = FTS1BA90A_DEV_NAME;
	ts->input->phys = "input/ts";
	ts->input->id.bustype = BUS_I2C;
	ts->input->open = fts1ba90a_open;
	ts->input->close = fts1ba90a_close;

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 4095, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 4095, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 63, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_capability(ts->input, EV_KEY, KEY_WAKEUP);

	ret = input_mt_init_slots(ts->input, FTS_FINGER_MAX, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	input_set_drvdata(ts->input, ts);

	ret = fts1ba90a_start(ts, true);
	if (ret)
		return dev_err_probe(dev, ret, "failed to start the controller\n");
	ts->state = FTS_STATE_ACTIVE;

	ret = devm_add_action_or_reset(dev, fts1ba90a_cleanup, ts);
	if (ret)
		return ret;

	/* stays off until the input device is opened */
	ts->charger = power_supply_is_system_supplied() > 0;
	ret = devm_request_threaded_irq(dev, client->irq, NULL, fts1ba90a_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					FTS1BA90A_DEV_NAME, ts);
	if (ret)
		return dev_err_probe(dev, ret, "irq request failed\n");

	mutex_lock(&ts->lock);
	fts1ba90a_update(ts, false);
	mutex_unlock(&ts->lock);

	device_init_wakeup(dev, device_property_read_bool(dev, "wakeup-source"));

	ret = power_supply_reg_notifier(&ts->psy_nb);
	if (ret)
		return ret;
	schedule_work(&ts->charger_work);

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	if (drm_is_panel_follower(dev))
		schedule_delayed_work(&ts->panel_work, 0);

	dev_info(dev, "%ux%u touchscreen\n",
		 ts->prop.max_x + 1, ts->prop.max_y + 1);

	return 0;
}

static int fts1ba90a_suspend(struct device *dev)
{
	struct fts1ba90a_data *ts = dev_get_drvdata(dev);

	/* a panel change may still be queued; it is applied here */
	cancel_work_sync(&ts->update_work);
	cancel_work_sync(&ts->reset_work);

	mutex_lock(&ts->lock);
	ts->suspended = true;
	fts1ba90a_update(ts, false);
	if (ts->state == FTS_STATE_LOWPOWER)
		enable_irq_wake(ts->client->irq);
	mutex_unlock(&ts->lock);

	return 0;
}

static int fts1ba90a_resume(struct device *dev)
{
	struct fts1ba90a_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->lock);
	if (ts->state == FTS_STATE_LOWPOWER)
		disable_irq_wake(ts->client->irq);
	ts->suspended = false;
	fts1ba90a_update(ts, false);
	mutex_unlock(&ts->lock);

	schedule_work(&ts->charger_work);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fts1ba90a_pm_ops,
				fts1ba90a_suspend, fts1ba90a_resume);

static const struct of_device_id fts1ba90a_of_match[] = {
	{ .compatible = "st,fts1ba90a" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts1ba90a_of_match);

static const struct i2c_device_id fts1ba90a_id[] = {
	{ FTS1BA90A_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fts1ba90a_id);

static struct i2c_driver fts1ba90a_driver = {
	.driver = {
		.name = FTS1BA90A_DEV_NAME,
		.of_match_table = fts1ba90a_of_match,
		.pm = pm_sleep_ptr(&fts1ba90a_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = fts1ba90a_probe,
	.id_table = fts1ba90a_id,
};
module_i2c_driver(fts1ba90a_driver);

MODULE_DESCRIPTION("STMicroelectronics FTS1BA90A touchscreen driver");
MODULE_LICENSE("GPL");
