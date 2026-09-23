// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal STMicroelectronics FTS1BA90A touchscreen driver
 *
 * Derived from Samsung downstream stm_fts1ba90a (T510/gta3xlwifi) protocol:
 *  - I2C addr 0x49 on exynos7885 i2c_4, IRQ gpa0-0, AVDD 3.3V / DVDD 1.8V
 *  - chip ID via 0x22 (expect 0x39/0x36), events via 0x60/0x61
 *
 * Basic touch only: no firmware update, no factory/sec_cmd, no sponge/AOT.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#define FTS1BA90A_DEV_NAME		"fts1ba90a"

#define FTS_CMD_SENSE_ON		0x10
#define FTS_CMD_SENSE_OFF		0x11
#define FTS_CMD_SW_RESET		0x12
#define FTS_CMD_FORCE_CALIBRATION	0x13
#define FTS_CMD_CLEAR_ALL_EVENT		0x62

#define FTS_READ_DEVICE_ID		0x22
#define FTS_READ_FW_VERSION		0x24
#define FTS_CMD_SET_GET_TOUCHTYPE	0x30
#define FTS_READ_ONE_EVENT		0x60
#define FTS_READ_ALL_EVENT		0x61

#define FTS_ID0				0x39
#define FTS_ID1				0x36

#define FTS_FIFO_MAX			32
#define FTS_EVENT_SIZE			8
#define FTS_FINGER_MAX			10
#define FTS_I2C_RETRY_CNT		3

/* event id (byte0[1:0]) */
#define FTS_EV_COORDINATE		0
#define FTS_EV_STATUS			1

/* coordinate action (byte0[7:6]) */
#define FTS_ACTION_NONE			0
#define FTS_ACTION_PRESS		1
#define FTS_ACTION_MOVE			2
#define FTS_ACTION_RELEASE		3

/* touch types we report */
#define FTS_TTYPE_NORMAL		0
#define FTS_TTYPE_GLOVE			3
#define FTS_TTYPE_PALM			5
#define FTS_TTYPE_WET			6
#define FTS_TOUCHTYPE_DEFAULT		(0x01 | 0x20 | 0x40) /* touch|palm|wet */

/* scanmode: A0 00 <mode>, bit0 = MS+SS scan */
#define FTS_SCANMODE_MS_SS		0x01

#define FTS_STATUS_STYPE_INFO		2
#define FTS_INFO_READY			0x00

struct fts1ba90a_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator *vdd;
	struct regulator *avdd;
	struct mutex i2c_lock;
	int touch_count;
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

static void fts1ba90a_release_all(struct fts1ba90a_data *ts)
{
	int i;

	for (i = 0; i < FTS_FINGER_MAX; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);
	ts->touch_count = 0;
}

static bool fts1ba90a_ttype_reportable(u8 ttype)
{
	return ttype == FTS_TTYPE_NORMAL || ttype == FTS_TTYPE_PALM ||
	       ttype == FTS_TTYPE_WET || ttype == FTS_TTYPE_GLOVE;
}

static void fts1ba90a_handle_coordinate(struct fts1ba90a_data *ts,
					const u8 *ev)
{
	unsigned int tid = (ev[0] >> 2) & 0x0f;
	unsigned int action = (ev[0] >> 6) & 0x03;
	unsigned int x = (ev[1] << 4) | ((ev[3] >> 4) & 0x0f);
	unsigned int y = (ev[2] << 4) | (ev[3] & 0x0f);
	unsigned int major = ev[4];
	unsigned int minor = ev[5];
	unsigned int z = ev[6] & 0x3f;
	unsigned int ttype = ((ev[6] >> 6) & 0x03) << 2 |
			     ((ev[7] >> 6) & 0x03);

	if (tid >= FTS_FINGER_MAX) {
		dev_dbg(&ts->client->dev, "tid %u out of range\n", tid);
		return;
	}

	if (!fts1ba90a_ttype_reportable(ttype))
		return;

	if (z == 0)
		z = 1;

	switch (action) {
	case FTS_ACTION_PRESS:
	case FTS_ACTION_MOVE:
		if (action == FTS_ACTION_PRESS)
			ts->touch_count++;
		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_key(ts->input, BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
		break;
	case FTS_ACTION_RELEASE:
		input_mt_slot(ts->input, tid);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		if (ts->touch_count > 0)
			ts->touch_count--;
		if (ts->touch_count == 0) {
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_report_key(ts->input, BTN_TOOL_FINGER, 0);
		}
		break;
	default:
		break;
	}
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

		if ((ev[0] & 0x03) == FTS_EV_COORDINATE)
			fts1ba90a_handle_coordinate(ts, ev);
		/* status/gesture events ignored in minimal driver */
	}

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
		    ((data[0] >> 2) & 0x0f) == FTS_STATUS_STYPE_INFO &&
		    data[1] == FTS_INFO_READY)
			return 0;

		msleep(20);
	}

	return -ETIMEDOUT;
}

static int fts1ba90a_init_chip(struct fts1ba90a_data *ts)
{
	u8 buf[3];
	u8 id_cmd = FTS_READ_DEVICE_ID;
	u8 id[5];
	int ret;

	/* SW reset sequence from downstream fts_systemreset(): FA 20 00 00 24 81 */
	{
		u8 reset_seq[6] = { 0xFA, 0x20, 0x00, 0x00, 0x24, 0x81 };

		ret = fts1ba90a_write(ts, reset_seq, sizeof(reset_seq));
		if (ret)
			return ret;
		msleep(10);
	}

	ret = fts1ba90a_wait_ready(ts);
	if (ret) {
		dev_err(&ts->client->dev, "timeout waiting for ready\n");
		return ret;
	}

	ret = fts1ba90a_read(ts, &id_cmd, 1, id, sizeof(id));
	if (ret) {
		dev_err(&ts->client->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	dev_info(&ts->client->dev, "chip id: %c %c %02x %02x %02x\n",
		 id[0], id[1], id[2], id[3], id[4]);

	if (id[2] != FTS_ID0 || id[3] != FTS_ID1) {
		dev_err(&ts->client->dev,
			"unexpected chip id %02x %02x (want %02x %02x)\n",
			id[2], id[3], FTS_ID0, FTS_ID1);
		return -ENODEV;
	}

	/* default touch type: touch|palm|wet */
	buf[0] = FTS_CMD_SET_GET_TOUCHTYPE;
	buf[1] = FTS_TOUCHTYPE_DEFAULT & 0xff;
	buf[2] = (FTS_TOUCHTYPE_DEFAULT >> 8) & 0xff;
	ret = fts1ba90a_write(ts, buf, 3);
	if (ret)
		return ret;
	msleep(10);

	/* force calibration + clear events */
	buf[0] = FTS_CMD_FORCE_CALIBRATION;
	ret = fts1ba90a_write(ts, buf, 1);
	if (ret)
		return ret;
	msleep(20);

	buf[0] = FTS_CMD_CLEAR_ALL_EVENT;
	ret = fts1ba90a_write(ts, buf, 1);
	if (ret)
		return ret;
	msleep(10);

	/* scan mode: MS + SS */
	buf[0] = 0xA0;
	buf[1] = 0x00;
	buf[2] = FTS_SCANMODE_MS_SS;
	ret = fts1ba90a_write(ts, buf, 3);
	if (ret)
		return ret;
	msleep(10);

	/* sense on */
	buf[0] = FTS_CMD_SENSE_ON;
	ret = fts1ba90a_write(ts, buf, 1);
	if (ret)
		return ret;
	msleep(20);

	return 0;
}

static void fts1ba90a_reg_disable(void *data)
{
	regulator_disable(data);
}

static int fts1ba90a_probe(struct i2c_client *client)
{
	struct fts1ba90a_data *ts;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	mutex_init(&ts->i2c_lock);
	i2c_set_clientdata(client, ts);

	ts->avdd = devm_regulator_get(&client->dev, "avdd");
	if (IS_ERR(ts->avdd))
		return dev_err_probe(&client->dev, PTR_ERR(ts->avdd),
				     "failed to get avdd\n");

	ts->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(&client->dev, PTR_ERR(ts->vdd),
				     "failed to get vdd\n");

	/* downstream order: avdd 3.3V, 1ms, dvdd 1.8V, 5ms */
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

	ret = devm_add_action_or_reset(&client->dev,
				       fts1ba90a_reg_disable,
				       ts->avdd);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&client->dev,
				       fts1ba90a_reg_disable,
				       ts->vdd);
	if (ret)
		return ret;

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = FTS1BA90A_DEV_NAME;
	ts->input->id.bustype = BUS_I2C;
	ts->input->dev.parent = &client->dev;

	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->input, EV_KEY, BTN_TOOL_FINGER);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 4096, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 4096, 0, 0);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	if (!ts->prop.max_x)
		ts->prop.max_x = 1199;
	if (!ts->prop.max_y)
		ts->prop.max_y = 1919;
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 63, 0, 0);

	ret = input_mt_init_slots(ts->input, FTS_FINGER_MAX, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	input_set_drvdata(ts->input, ts);

	ret = fts1ba90a_init_chip(ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, fts1ba90a_irq,
					IRQF_ONESHOT,
					FTS1BA90A_DEV_NAME, ts);
	if (ret)
		return dev_err_probe(&client->dev, ret, "irq request failed\n");

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	dev_info(&client->dev, "FTS1BA90A basic touch probed (%ux%u)\n",
		 ts->prop.max_x + 1, ts->prop.max_y + 1);

	return 0;
}

static void fts1ba90a_remove(struct i2c_client *client)
{
	struct fts1ba90a_data *ts = i2c_get_clientdata(client);
	u8 cmd = FTS_CMD_SENSE_OFF;

	fts1ba90a_release_all(ts);
	/* best effort sleep; ignore errors during remove */
	fts1ba90a_write(ts, &cmd, 1);
}

#ifdef CONFIG_OF
static const struct of_device_id fts1ba90a_of_match[] = {
	{ .compatible = "stm,fts_touch" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts1ba90a_of_match);
#endif

static const struct i2c_device_id fts1ba90a_id[] = {
	{ FTS1BA90A_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fts1ba90a_id);

static struct i2c_driver fts1ba90a_driver = {
	.driver = {
		.name = FTS1BA90A_DEV_NAME,
		.of_match_table = of_match_ptr(fts1ba90a_of_match),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = fts1ba90a_probe,
	.remove = fts1ba90a_remove,
	.id_table = fts1ba90a_id,
};
module_i2c_driver(fts1ba90a_driver);

MODULE_AUTHOR("T510 mainlining");
MODULE_DESCRIPTION("Minimal ST FTS1BA90A touchscreen (basic touch)");
MODULE_LICENSE("GPL");
