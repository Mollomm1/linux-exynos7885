// SPDX-License-Identifier: GPL-2.0-only
/* SM-T510 TFA9896 setup from its CHSA=0 vendor container. */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define TFA9896_REVISION	0x03
#define TFA9896_SYS_CTRL	0x09
#define TFA9896_SYS_CTRL_PWDN	BIT(0)
#define TFA9896_SYS_CTRL_AMPE	BIT(3)
#define TFA9896_HIDE		0x40

struct tfa9896_t510 {
	struct regmap *regmap;
	struct mutex lock;
	bool active;
	bool slot_one;
};

struct tfa9896_field {
	u16 code;
	u16 value;
};

/* Device fields audited from vendor/firmware/Tfa9896.cnt. */
static const struct tfa9896_field tfa9896_common_fields[] = {
	{ 0x09e0, 0 }, { 0x04c3, 8 }, { 0x10a4, 15 }, { 0x14c3, 0 },
	{ 0x0461, 0 }, { 0x4900, 1 }, { 0x0a04, 10 }, { 0x0aa1, 3 },
	{ 0x6007, 0x005a }, { 0x45a0, 1 }, { 0x0613, 6 },
	{ 0x1030, 0 }, { 0x1011, 2 }, { 0x1103, 1 }, { 0x1144, 15 },
	{ 0x1090, 1 }, { 0x11e0, 1 }, { 0x12e0, 0 }, { 0x12f0, 0 },
	{ 0x1310, 1 }, { 0x1320, 1 }, { 0x1280, 1 }, { 0x1290, 1 },
	{ 0x12b0, 1 }, { 0x12c0, 0 },
};

static int tfa9896_write_field(struct tfa9896_t510 *tfa,
			       const struct tfa9896_field *field)
{
	unsigned int reg = field->code >> 8;
	unsigned int shift = (field->code >> 4) & 0xf;
	unsigned int width = (field->code & 0xf) + 1;
	unsigned int mask;

	if (width > 16 - shift || field->value >= (1U << width))
		return -EINVAL;
	mask = GENMASK(shift + width - 1, shift);

	return regmap_update_bits(tfa->regmap, reg, mask,
				  (field->value << shift) & mask);
}

static int tfa9896_write_fields(struct tfa9896_t510 *tfa,
				const struct tfa9896_field *fields,
				unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = tfa9896_write_field(tfa, &fields[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int tfa9896_t510_start(struct tfa9896_t510 *tfa)
{
	/* TFA9896-specific initialization from the vendor driver. */
	static const struct reg_sequence init[] = {
		{ 0x06, 0x000b }, { 0x07, 0x3e7f }, { 0x0a, 0x0d8a },
		{ 0x48, 0x0300 }, { 0x88, 0x0100 },
	};
	struct tfa9896_field fields[ARRAY_SIZE(tfa9896_common_fields) + 5];
	unsigned int val, n = 0;
	int ret;

	/* Wake the part, but leave AMPE clear while configuration is written. */
	ret = regmap_clear_bits(tfa->regmap, TFA9896_SYS_CTRL,
				TFA9896_SYS_CTRL_PWDN | TFA9896_SYS_CTRL_AMPE);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(tfa->regmap, init, ARRAY_SIZE(init));
	if (ret)
		goto fail;

	ret = regmap_read(tfa->regmap, 0x49, &val);
	if (ret)
		goto fail;
	ret = regmap_write(tfa->regmap, 0x49, val & ~BIT(0));
	if (ret)
		goto fail;

	/* Unlock hidden fields, apply the per-device TDM channel, then re-hide. */
	ret = regmap_write(tfa->regmap, TFA9896_HIDE, 0x5a6b);
	if (ret)
		goto fail;
	ret = tfa9896_write_fields(tfa, tfa9896_common_fields,
				    ARRAY_SIZE(tfa9896_common_fields));
	if (ret)
		goto hide;

	fields[n++] = (struct tfa9896_field) {
		.code = 0x1373, .value = tfa->slot_one ? 1 : 0,
	};
	fields[n++] = (struct tfa9896_field) {
		.code = 0x13b3, .value = tfa->slot_one ? 0 : 1,
	};
	fields[n++] = (struct tfa9896_field) {
		.code = 0x1443, .value = tfa->slot_one ? 1 : 0,
	};
	fields[n++] = (struct tfa9896_field) {
		.code = 0x1483, .value = tfa->slot_one ? 0 : 1,
	};
	fields[n++] = (struct tfa9896_field) {
		.code = 0x1030, .value = 1,
	};
	ret = tfa9896_write_fields(tfa, fields, n);

hide:
	if (regmap_write(tfa->regmap, TFA9896_HIDE, 0) && !ret)
		ret = -EIO;
	if (ret)
		goto fail;

	/* HQ profile: CHSA=0, CFE=0, AMPC=0, DCA=1; AMPE is last. */
	ret = regmap_update_bits(tfa->regmap, TFA9896_SYS_CTRL,
				 BIT(2) | BIT(4) | BIT(6), BIT(4));
	if (ret)
		goto fail;
	ret = regmap_set_bits(tfa->regmap, TFA9896_SYS_CTRL,
			      TFA9896_SYS_CTRL_AMPE);
	if (ret)
		goto fail;
	return 0;

fail:
	regmap_clear_bits(tfa->regmap, TFA9896_SYS_CTRL,
			  TFA9896_SYS_CTRL_AMPE);
	regmap_set_bits(tfa->regmap, TFA9896_SYS_CTRL,
			TFA9896_SYS_CTRL_PWDN);
	return ret;
}

static int tfa9896_t510_stop(struct tfa9896_t510 *tfa)
{
	int ret, powerdown_ret;

	ret = regmap_clear_bits(tfa->regmap, TFA9896_SYS_CTRL,
				TFA9896_SYS_CTRL_AMPE);
	powerdown_ret = regmap_set_bits(tfa->regmap, TFA9896_SYS_CTRL,
					TFA9896_SYS_CTRL_PWDN);
	return ret ?: powerdown_ret;
}

static int tfa9896_t510_mute(struct snd_soc_dai *dai, int mute,
			     int direction)
{
	struct tfa9896_t510 *tfa = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	if (direction != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&tfa->lock);
	if (mute) {
		if (tfa->active)
			ret = tfa9896_t510_stop(tfa);
		if (!ret)
			tfa->active = false;
	} else if (!tfa->active) {
		ret = tfa9896_t510_start(tfa);
		if (!ret)
			tfa->active = true;
	}
	mutex_unlock(&tfa->lock);

	return ret;
}

static void tfa9896_t510_shutdown(void *data)
{
	struct tfa9896_t510 *tfa = data;

	mutex_lock(&tfa->lock);
	if (tfa->active && !tfa9896_t510_stop(tfa))
		tfa->active = false;
	mutex_unlock(&tfa->lock);
}

static const struct snd_soc_dai_ops tfa9896_t510_dai_ops = {
	.mute_stream = tfa9896_t510_mute,
	.mute_unmute_on_trigger = 1,
};

static struct snd_soc_dai_driver tfa9896_t510_dai = {
	.name = "tfa9896-aif",
	.playback = {
		.stream_name = "TFA9896 Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tfa9896_t510_dai_ops,
};

static const struct snd_soc_component_driver tfa9896_t510_component = {
	.dapm_widgets = (const struct snd_soc_dapm_widget[]) {
		SND_SOC_DAPM_OUTPUT("OUT"),
		SND_SOC_DAPM_AIF_IN("AIFIN", "TFA9896 Playback", 0,
				    SND_SOC_NOPM, 0, 0),
	},
	.num_dapm_widgets = 2,
	.dapm_routes = (const struct snd_soc_dapm_route[]) {
		{ "OUT", NULL, "AIFIN" },
	},
	.num_dapm_routes = 1,
	.endianness = 1,
};

static const struct regmap_config tfa9896_t510_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
	.cache_type = REGCACHE_NONE,
};

static int tfa9896_t510_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct tfa9896_t510 *tfa;
	unsigned int revision;
	int ret;

	tfa = devm_kzalloc(dev, sizeof(*tfa), GFP_KERNEL);
	if (!tfa)
		return -ENOMEM;

	mutex_init(&tfa->lock);
	tfa->regmap = devm_regmap_init_i2c(i2c, &tfa9896_t510_regmap_config);
	if (IS_ERR(tfa->regmap))
		return PTR_ERR(tfa->regmap);

	/* Probe is read-only; all writes wait until a prepared PCM starts. */
	ret = regmap_read(tfa->regmap, TFA9896_REVISION, &revision);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read revision\n");
	if ((revision & 0xff) != 0x96)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected revision %#x\n", revision);
	if (i2c->addr != 0x34 && i2c->addr != 0x35)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported SM-T510 amplifier address\n");
	tfa->slot_one = i2c->addr == 0x34;
	i2c_set_clientdata(i2c, tfa);

	ret = devm_snd_soc_register_component(dev, &tfa9896_t510_component,
					      &tfa9896_t510_dai, 1);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, tfa9896_t510_shutdown, tfa);
}

static const struct of_device_id tfa9896_t510_of_match[] = {
	{ .compatible = "nxp,tfa9896" },
	{ }
};
MODULE_DEVICE_TABLE(of, tfa9896_t510_of_match);

static struct i2c_driver tfa9896_t510_driver = {
	.driver = {
		.name = "tfa9896-t510-bypass",
		.of_match_table = tfa9896_t510_of_match,
	},
	.probe = tfa9896_t510_probe,
};
module_i2c_driver(tfa9896_t510_driver);

MODULE_DESCRIPTION("SM-T510 TFA9896 vendor bypass codec driver");
MODULE_LICENSE("GPL");
