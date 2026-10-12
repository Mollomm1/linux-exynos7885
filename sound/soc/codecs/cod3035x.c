// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung COD3035X headphone codec, playback only.
 *
 * The codec sits in the S2MPU08 PMIC package on the SPEEDY bus. Like the
 * vendor driver, all register accesses go through the ACPM firmware's MFD
 * service: SPEEDY type 3 is the codec core, type 7 its analog block. The
 * register sequences follow the vendor cod3035x driver for its "LassenA"
 * 48 kHz headphone path.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define COD3035X_ACPM_TYPE_CODEC	3
#define COD3035X_ACPM_TYPE_ANALOG	7

#define COD3035X_10_PD_REF		0x10
#define COD3035X_18_PWAUTO_DA		0x18
#define COD3035X_PW_AUTO_DA		BIT(4)
#define COD3035X_APW_HP			BIT(1)
#define COD3035X_30_VOL_HPL		0x30
#define COD3035X_31_VOL_HPR		0x31
#define COD3035X_37_MIX_DA1		0x37
#define COD3035X_EN_HP_MIXL_DCTL	BIT(7)
#define COD3035X_EN_HP_MIXR_DCTR	BIT(2)
#define COD3035X_3E_OVP_2		0x3e
#define COD3035X_LOCK_UP_TIME_SLOT	GENMASK(7, 6)
#define COD3035X_LOCK_UP_TIME_1_5	2
#define COD3035X_OVP_APON		BIT(4)
#define COD3035X_40_DIGITAL_POWER	0x40
#define COD3035X_PDB_DACDIG		BIT(5)
#define COD3035X_RSTB_DAT_DA		BIT(4)
#define COD3035X_41_IF1_FORMAT1		0x41
#define COD3035X_42_IF1_FORMAT2		0x42
#define COD3035X_I2S1_DL		GENMASK(5, 0)
#define COD3035X_I2S1_DL_16BIT		0x10
#define COD3035X_43_IF1_FORMAT3		0x43
#define COD3035X_I2S1_XFS		GENMASK(7, 0)
#define COD3035X_I2S1_XFS_32FS		0x20
#define COD3035X_45_IF1_FORMAT5		0x45
#define COD3035X_DAC_DATA_SEL		BIT(4)
#define COD3035X_50_DAC1		0x50
#define COD3035X_DAC1_INGN_MODE		BIT(3)
#define COD3035X_51_DVOLL		0x51
#define COD3035X_52_DVOLR		0x52
#define COD3035X_DVOL_MUTE		0xff
#define COD3035X_53_UHQA		0x53
#define COD3035X_UHQA_MODE		BIT(0)
#define COD3035X_58_AVC5		0x58
#define COD3035X_73_DSM1_DA		0x73
#define COD3035X_74_DSM2_DA		0x74
#define COD3035X_77_CHOP_DA		0x77
#define COD3035X_EN_HP_CHOP		BIT(4)
#define COD3035X_EN_EP_CHOP		BIT(3)
#define COD3035X_7A_MAN_GN1		0x7a
#define COD3035X_7B_MAN_GN2		0x7b
#define COD3035X_7D_AVC_PARA29		0x7d
#define COD3035X_80_PDB_ACC1		0x80
#define COD3035X_PDB_JD_CLK_EN		BIT(1)
#define COD3035X_81_PDB_ACC2		0x81
#define COD3035X_T_PDB_IMP		BIT(3)
#define COD3035X_BB_AUTO_HP12		0xbb
#define COD3035X_DC_CTRL_EPS		0xdc
#define COD3035X_CTMI_EP_A		GENMASK(7, 6)
#define COD3035X_F2_PRESET_AVC		0xf2
#define COD3035X_F3_PRESET_AVC		0xf3
#define COD3035X_F5_PRESET_AVC		0xf5
#define COD3035X_F6_PRESET_AVC		0xf6

/* Analog block (SPEEDY type 7). */
#define COD3035A_41_DNC1		0x41
#define COD3035A_42_DNC2		0x42
#define COD3035A_43_DNC3		0x43
#define COD3035A_44_DNC4		0x44
#define COD3035A_4A_DNC10		0x4a
#define COD3035A_4B_DNC11		0x4b
#define COD3035A_4C_DNC12		0x4c
#define COD3035A_4D_DNC13		0x4d
#define COD3035A_4E_DNC14		0x4e
#define COD3035A_50_OFF_DLY1		0x50
#define COD3035A_51_OFF_DLY2		0x51

/* Default digital volume, restored after the power-up mute. */
#define COD3035X_DVOL_DEFAULT		0x54

struct cod3035x {
	struct device *dev;
	struct regmap *regmap;
	const struct acpm_handle *acpm;
	unsigned int acpm_chan;
	struct clk *mclk;
};

static int cod3035x_acpm_read(struct cod3035x *cod, u8 type, u8 reg, u8 *val)
{
	int ret;

	ret = cod->acpm->ops.pmic_ops.read_reg(cod->acpm, cod->acpm_chan, type,
					       reg, 0, val);
	return ret > 0 ? -EIO : ret;
}

static int cod3035x_acpm_write(struct cod3035x *cod, u8 type, u8 reg, u8 val)
{
	int ret;

	ret = cod->acpm->ops.pmic_ops.write_reg(cod->acpm, cod->acpm_chan, type,
						reg, 0, val);
	return ret > 0 ? -EIO : ret;
}

static int cod3035x_reg_read(void *context, unsigned int reg,
			     unsigned int *val)
{
	u8 v;
	int ret;

	ret = cod3035x_acpm_read(context, COD3035X_ACPM_TYPE_CODEC, reg, &v);
	if (!ret)
		*val = v;
	return ret;
}

static int cod3035x_reg_write(void *context, unsigned int reg,
			      unsigned int val)
{
	return cod3035x_acpm_write(context, COD3035X_ACPM_TYPE_CODEC, reg, val);
}

static const struct regmap_config cod3035x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_read = cod3035x_reg_read,
	.reg_write = cod3035x_reg_write,
	/* Status, IRQ and jack registers change under us; keep no cache. */
	.cache_type = REGCACHE_NONE,
};

/* Vendor boot-time defaults (cod3035x_codec_initialize, T510 DT values). */
static const struct reg_sequence cod3035x_init_seq[] = {
	/* IRQ masks */
	{ 0x05, 0x00 }, { 0x06, 0x00 }, { 0x07, 0x53 }, { 0x08, 0x53 },
	/* OVP and pop thresholds (dis-ovp-det-mode unset) */
	{ 0x3d, 0xd7 }, { 0x84, 0xbe },
	{ 0xa6, 0x42 }, { 0x93, 0x00 }, { 0x94, 0x00 },
	{ 0x8e, 0xc0 }, { 0x8f, 0xc0 },
	/* Jack debounce: jackin 0xd0 | jackout 0x04 */
	{ 0x8d, 0xd4 },
	{ 0x9a, 0x01 }, { 0x44, 0xff },
	/* HP amplifier reference current 8 uA */
	{ 0xb0, 0xa0 }, { 0xb4, 0x01 }, { 0xb5, 0xfc }, { 0xb6, 0xff },
	{ 0xb7, 0xb7 }, { 0xb8, 0x55 }, { 0xba, 0x02 }, { 0xbb, 0x66 },
	{ 0xbe, 0x60 },
	/* AVC mute and DRC off, AVC presets */
	{ 0x54, 0x8f }, { 0x55, 0x78 }, { 0x57, 0x00 },
	{ 0xe1, 0xf1 }, { 0xe3, 0x00 }, { 0xe5, 0x00 }, { 0xe7, 0x00 },
	{ 0xe8, 0x00 }, { 0xe9, 0x00 }, { 0xea, 0x01 }, { 0xf2, 0x03 },
	{ 0xf3, 0x1f }, { 0xf4, 0x55 }, { 0x5a, 0x44 }, { 0xf5, 0x1a },
	{ 0xf6, 0xc6 }, { 0x1b, 0x01 },
	/* Accessory block power, then on with a 100 ms settle */
	{ 0x80, 0x02, 100000 }, { 0x80, 0x03 },
};

static const struct reg_sequence cod3035x_init_seq2[] = {
	/* Default HP analog and DAC digital volumes, reference power */
	{ COD3035X_30_VOL_HPL, 0x0a }, { COD3035X_31_VOL_HPR, 0x0a },
	{ COD3035X_51_DVOLL, COD3035X_DVOL_DEFAULT },
	{ COD3035X_52_DVOLR, COD3035X_DVOL_DEFAULT },
	{ COD3035X_10_PD_REF, 0x10 },
};

static int cod3035x_hw_init(struct cod3035x *cod)
{
	struct regmap *map = cod->regmap;
	int ret;

	/* Jack-detect clock first, as the vendor resume path does. */
	ret = regmap_set_bits(map, COD3035X_80_PDB_ACC1,
			      COD3035X_PDB_JD_CLK_EN);
	if (ret)
		return ret;
	usleep_range(15000, 16000);
	/* No output selected until a stream starts. */
	ret = regmap_clear_bits(map, COD3035X_77_CHOP_DA,
				COD3035X_EN_HP_CHOP | COD3035X_EN_EP_CHOP);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(map, cod3035x_init_seq,
				     ARRAY_SIZE(cod3035x_init_seq));
	if (ret)
		return ret;
	ret = regmap_set_bits(map, COD3035X_81_PDB_ACC2, COD3035X_T_PDB_IMP);
	if (ret)
		return ret;
	/* Receiver reference current 2 uA (rcv-drv-current = 1) */
	ret = regmap_update_bits(map, COD3035X_DC_CTRL_EPS, COD3035X_CTMI_EP_A,
				 FIELD_PREP(COD3035X_CTMI_EP_A, 1));
	if (ret)
		return ret;
	return regmap_multi_reg_write(map, cod3035x_init_seq2,
				      ARRAY_SIZE(cod3035x_init_seq2));
}

static int cod3035x_dac_mute(struct cod3035x *cod, bool mute)
{
	unsigned int vol = mute ? COD3035X_DVOL_MUTE : COD3035X_DVOL_DEFAULT;
	int ret;

	ret = regmap_write(cod->regmap, COD3035X_51_DVOLL, vol);
	if (ret)
		return ret;
	return regmap_write(cod->regmap, COD3035X_52_DVOLR, vol);
}

static int cod3035x_dac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);
	struct regmap *map = cod->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* DAC digital power on, then pulse its data reset. */
		regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				COD3035X_PDB_DACDIG);
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_DA);
		return regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				       COD3035X_RSTB_DAT_DA);
	case SND_SOC_DAPM_PRE_PMD:
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_DA);
		return regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
					 COD3035X_PDB_DACDIG);
	}
	return 0;
}

static int cod3035x_hpdrv_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);
	struct regmap *map = cod->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3035x_dac_mute(cod, true);
		regmap_set_bits(map, COD3035X_77_CHOP_DA, COD3035X_EN_HP_CHOP);
		/* cod3035x_hp_playback_init */
		regmap_set_bits(map, COD3035X_45_IF1_FORMAT5,
				COD3035X_DAC_DATA_SEL);
		regmap_write(map, COD3035X_73_DSM1_DA, 0x20);
		regmap_write(map, COD3035X_74_DSM2_DA, 0x89);
		usleep_range(100, 200);
		return 0;
	case SND_SOC_DAPM_POST_PMU:
		/* post_hpdrv_normal */
		regmap_write(map, 0x33, 0x00);
		regmap_write(map, COD3035X_BB_AUTO_HP12, 0x33);
		regmap_set_bits(map, COD3035X_37_MIX_DA1,
				COD3035X_EN_HP_MIXL_DCTL |
				COD3035X_EN_HP_MIXR_DCTR);
		regmap_set_bits(map, COD3035X_18_PWAUTO_DA, COD3035X_APW_HP);
		msleep(135);
		cod3035x_dac_mute(cod, false);
		return regmap_update_bits(map, COD3035X_3E_OVP_2,
				COD3035X_LOCK_UP_TIME_SLOT | COD3035X_OVP_APON,
				FIELD_PREP(COD3035X_LOCK_UP_TIME_SLOT,
					   COD3035X_LOCK_UP_TIME_1_5) |
				COD3035X_OVP_APON);
	case SND_SOC_DAPM_PRE_PMD:
		/* normal_ear_power_off, earpiece unused */
		cod3035x_dac_mute(cod, true);
		msleep(100);
		regmap_clear_bits(map, COD3035X_18_PWAUTO_DA,
				  COD3035X_PW_AUTO_DA | COD3035X_APW_HP);
		usleep_range(3000, 3100);
		regmap_clear_bits(map, COD3035X_3E_OVP_2, COD3035X_OVP_APON);
		regmap_clear_bits(map, COD3035X_37_MIX_DA1,
				  COD3035X_EN_HP_MIXL_DCTL |
				  COD3035X_EN_HP_MIXR_DCTR);
		regmap_write(map, COD3035X_F6_PRESET_AVC, 0xc7);
		return regmap_clear_bits(map, COD3035X_77_CHOP_DA,
					 COD3035X_EN_HP_CHOP);
	}
	return 0;
}

static const struct snd_soc_dapm_widget cod3035x_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("DAC", "AIF Playback", SND_SOC_NOPM, 0, 0,
			   cod3035x_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cod3035x_hpdrv_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("HPOUT"),
};

static const struct snd_soc_dapm_route cod3035x_dapm_routes[] = {
	{ "HPDRV", NULL, "DAC" },
	{ "HPOUT", NULL, "HPDRV" },
};

/* cod3035x_lassenA_normal: 48 kHz (non-UHQA) DAC and analog presets. */
static int cod3035x_lassena_normal(struct cod3035x *cod)
{
	static const struct reg_sequence codec_seq[] = {
		{ COD3035X_F2_PRESET_AVC, 0x03 },
		{ COD3035X_F3_PRESET_AVC, 0x1f },
		{ COD3035X_F5_PRESET_AVC, 0x1a },
		{ COD3035X_F6_PRESET_AVC, 0xc7 },
		{ COD3035X_58_AVC5, 0x81 },
	};
	static const u8 analog_seq[][2] = {
		{ COD3035A_50_OFF_DLY1, 0x1a }, { COD3035A_51_OFF_DLY2, 0x68 },
		{ COD3035A_41_DNC1, 0x01 }, { COD3035A_42_DNC2, 0xda },
		{ COD3035A_43_DNC3, 0x32 }, { COD3035A_44_DNC4, 0x32 },
		{ COD3035A_4A_DNC10, 0xfe }, { COD3035A_4B_DNC11, 0x85 },
		{ COD3035A_4C_DNC12, 0x00 }, { COD3035A_4D_DNC13, 0x83 },
		{ COD3035A_4E_DNC14, 0x10 },
	};
	struct regmap *map = cod->regmap;
	unsigned int i;
	int ret;

	ret = regmap_multi_reg_write(map, codec_seq, ARRAY_SIZE(codec_seq));
	if (ret)
		return ret;
	regmap_clear_bits(map, COD3035X_50_DAC1, COD3035X_DAC1_INGN_MODE);
	regmap_clear_bits(map, COD3035X_53_UHQA, COD3035X_UHQA_MODE);
	regmap_write(map, COD3035X_7A_MAN_GN1, 0x2f);
	regmap_write(map, COD3035X_7B_MAN_GN2, 0x80);
	usleep_range(100, 200);
	for (i = 0; i < ARRAY_SIZE(analog_seq); i++) {
		ret = cod3035x_acpm_write(cod, COD3035X_ACPM_TYPE_ANALOG,
					  analog_seq[i][0], analog_seq[i][1]);
		if (ret)
			return ret;
	}
	usleep_range(100, 200);
	regmap_write(map, COD3035X_BB_AUTO_HP12, 0x33);
	ret = regmap_write(map, COD3035X_7D_AVC_PARA29, 0x8b);
	usleep_range(1000, 1100);
	return ret;
}

static int cod3035x_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static int cod3035x_dai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct cod3035x *cod = snd_soc_component_get_drvdata(dai->component);
	int ret;

	/* Only the tested vendor mode: 48 kHz, 16-bit I2S with 32 fs BCLK. */
	if (params_rate(params) != 48000 || params_width(params) != 16 ||
	    params_channels(params) != 2)
		return -EINVAL;

	ret = regmap_write(cod->regmap, COD3035X_41_IF1_FORMAT1, 0x00);
	if (ret)
		return ret;
	ret = regmap_update_bits(cod->regmap, COD3035X_42_IF1_FORMAT2,
				 COD3035X_I2S1_DL, COD3035X_I2S1_DL_16BIT);
	if (ret)
		return ret;
	ret = regmap_update_bits(cod->regmap, COD3035X_43_IF1_FORMAT3,
				 COD3035X_I2S1_XFS, COD3035X_I2S1_XFS_32FS);
	if (ret)
		return ret;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return cod3035x_lassena_normal(cod);
	return 0;
}

static const struct snd_soc_dai_ops cod3035x_dai_ops = {
	.set_fmt = cod3035x_dai_set_fmt,
	.hw_params = cod3035x_dai_hw_params,
};

static struct snd_soc_dai_driver cod3035x_dai = {
	.name = "cod3035x-aif",
	.playback = {
		.stream_name = "AIF Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cod3035x_dai_ops,
};

static int cod3035x_component_probe(struct snd_soc_component *component)
{
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	return cod3035x_hw_init(cod);
}

static const struct snd_soc_component_driver cod3035x_component = {
	.probe = cod3035x_component_probe,
	.dapm_widgets = cod3035x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cod3035x_dapm_widgets),
	.dapm_routes = cod3035x_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cod3035x_dapm_routes),
	.endianness = 1,
};

static void cod3035x_put_node(void *data)
{
	of_node_put(data);
}

static int cod3035x_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	struct cod3035x *cod;
	int ret;

	cod = devm_kzalloc(dev, sizeof(*cod), GFP_KERNEL);
	if (!cod)
		return -ENOMEM;
	cod->dev = dev;
	platform_set_drvdata(pdev, cod);

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "samsung,acpm-ipc",
					       1, 0, &args);
	if (ret)
		return dev_err_probe(dev, ret, "missing samsung,acpm-ipc\n");
	ret = devm_add_action_or_reset(dev, cod3035x_put_node, args.np);
	if (ret)
		return ret;
	cod->acpm_chan = args.args[0];
	cod->acpm = devm_acpm_get_by_node(dev, args.np);
	if (IS_ERR(cod->acpm))
		return dev_err_probe(dev, PTR_ERR(cod->acpm),
				     "failed to get ACPM handle\n");

	/* The codec runs from the SoC's 26 MHz CLKOUT0. */
	cod->mclk = devm_clk_get_enabled(dev, "mclk");
	if (IS_ERR(cod->mclk))
		return dev_err_probe(dev, PTR_ERR(cod->mclk),
				     "failed to enable mclk\n");

	cod->regmap = devm_regmap_init(dev, NULL, cod, &cod3035x_regmap_config);
	if (IS_ERR(cod->regmap))
		return PTR_ERR(cod->regmap);

	return devm_snd_soc_register_component(dev, &cod3035x_component,
					       &cod3035x_dai, 1);
}

static const struct of_device_id cod3035x_of_match[] = {
	{ .compatible = "samsung,cod3035x" },
	{ }
};
MODULE_DEVICE_TABLE(of, cod3035x_of_match);

static struct platform_driver cod3035x_driver = {
	.driver = {
		.name = "cod3035x",
		.of_match_table = cod3035x_of_match,
	},
	.probe = cod3035x_probe,
};
module_platform_driver(cod3035x_driver);

MODULE_DESCRIPTION("Samsung COD3035X headphone codec (playback)");
MODULE_LICENSE("GPL");
