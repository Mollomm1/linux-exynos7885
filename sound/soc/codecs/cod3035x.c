// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung COD3035X headphone and microphone codec.
 *
 * The codec sits in the S2MPU08 PMIC package on the SPEEDY bus. Like the
 * vendor driver, all register accesses go through the ACPM firmware's MFD
 * service: SPEEDY type 3 is the codec core, type 7 its analog block. The
 * register sequences follow the vendor cod3035x driver for its "LassenA"
 * 48 kHz headphone path and its analog microphone recording path.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define COD3035X_ACPM_TYPE_PMIC		1
#define COD3035X_ACPM_TYPE_CODEC	3
#define COD3035X_ACPM_TYPE_ANALOG	7

#define COD3035X_01_IRQ1PEND		0x01
#define COD3035X_IRQ1_ST_JACKIN_R	BIT(7)
#define COD3035X_IRQ1_ST_JACKOUT_R	BIT(6)
#define COD3035X_IRQ1_ST_APCHECK_R	BIT(5)
#define COD3035X_09_STATUS1		0x09
#define COD3035X_STATUS1_JACK_DET	BIT(0)
#define COD3035X_10_PD_REF		0x10
#define COD3035X_PDB_VMID		BIT(5)
#define COD3035X_PDB_IGEN		BIT(4)
#define COD3035X_PDB_IGEN_AD		BIT(3)
#define COD3035X_PDB_MCB1		BIT(2)
#define COD3035X_PDB_MCB2_CODEC		BIT(1)
#define COD3035X_PDB_MCB_LDO_CODEC	BIT(0)
#define COD3035X_11_PD_AD1		0x11
#define COD3035X_PDB_MIXL		BIT(7)
#define COD3035X_PDB_MIXR		BIT(6)
#define COD3035X_EN_DSML_PREQ		BIT(5)
#define COD3035X_EN_DSMR_PREQ		BIT(4)
#define COD3035X_PDB_DSML		BIT(3)
#define COD3035X_PDB_DSMR		BIT(2)
#define COD3035X_RESETB_DSML		BIT(1)
#define COD3035X_RESETB_DSMR		BIT(0)
#define COD3035X_12_PD_AD2		0x12
/* Boost and PGA power: MIC1 bits 5 and 2, MIC2 bits 4 and 1. */
#define COD3035X_PDB_MIC(n)		(BIT(5 - (n)) | BIT(2 - (n)))
#define COD3035X_13_PD_AD3		0x13
#define COD3035X_EN_BST_DIODE		BIT(7)
#define COD3035X_PDB_MIXC		BIT(6)
#define COD3035X_EN_DSMC_PREQ		BIT(5)
#define COD3035X_PDB_DSMC		BIT(4)
#define COD3035X_RESETB_DSMC		BIT(3)
#define COD3035X_RESETB_BST(n)		BIT(2 - (n))
#define COD3035X_18_PWAUTO_DA		0x18
#define COD3035X_PW_AUTO_DA		BIT(4)
#define COD3035X_APW_HP			BIT(1)
#define COD3035X_19_CTRL_REF		0x19
#define COD3035X_CTMF_VMID		GENMASK(7, 6)
#define COD3035X_CTMF_VMID_1K		0
#define COD3035X_CTMF_VMID_60K		1
#define COD3035X_CTRV_MCB1		GENMASK(5, 4)
#define COD3035X_MCB_2_8V		1
#define COD3035X_20_VOL_AD1		0x20
#define COD3035X_21_VOL_AD2		0x21
#define COD3035X_CTVOL_BST_SHIFT	5
#define COD3035X_25_MIX_AD1		0x25
/* ADC mixer inputs: MIC1 L/R bits 5/4, MIC2 L/R bits 3/2. */
#define COD3035X_EN_MIX_MICL(n)		BIT(5 - 2 * (n))
#define COD3035X_EN_MIX_MICR(n)		BIT(4 - 2 * (n))
#define COD3035X_26_MIX_ADC		0x26
#define COD3035X_EN_MIX_MICC(n)		BIT(7 - (n))
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
#define COD3035X_PDB_ADCDIG		BIT(7)
#define COD3035X_RSTB_DAT_AD		BIT(6)
#define COD3035X_PDB_DACDIG		BIT(5)
#define COD3035X_RSTB_DAT_DA		BIT(4)
#define COD3035X_41_IF1_FORMAT1		0x41
#define COD3035X_42_IF1_FORMAT2		0x42
#define COD3035X_I2S1_DL		GENMASK(5, 0)
#define COD3035X_I2S1_DL_16BIT		0x10
#define COD3035X_43_IF1_FORMAT3		0x43
#define COD3035X_I2S1_XFS		GENMASK(7, 0)
#define COD3035X_I2S1_XFS_32FS		0x20
#define COD3035X_44_IF1_FORMAT4		0x44
/* Interface slot sources: slot 0 ADC1 (left), slot 1 ADC2 (right). */
#define COD3035X_ADC_SEL_OFF		0xff
#define COD3035X_ADC_SEL_LR		0xe4
#define COD3035X_45_IF1_FORMAT5		0x45
#define COD3035X_DAC_DATA_SEL		BIT(4)
#define COD3035X_46_ADC1		0x46
#define COD3035X_ADC_MUTE		BIT(0)
#define COD3035X_47_AVOLL1		0x47
#define COD3035X_48_AVOLR1		0x48
#define COD3035X_AD_DVOL_MAX		0x96
#define COD3035X_4A_DMIC2		0x4a
#define COD3035X_DMIC_ST		GENMASK(1, 0)
#define COD3035X_4B_ADC2		0x4b
#define COD3035X_PDB_ADCDIG2		BIT(7)
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
#define COD3035X_83_CTR_MCB		0x83
#define COD3035X_CTRV_MCB2		GENMASK(3, 2)
#define COD3035X_CTRV_MCB_LDO		GENMASK(1, 0)
#define COD3035X_MCB_LDO_3_3V		3
#define COD3035X_93_CTR_DLY6		0x93
#define COD3035X_AP_JACK_IN		BIT(5)
#define COD3035X_94_JACK_CTR		0x94
#define COD3035X_JACK_CTR_3POLE		0x20
#define COD3035X_BB_AUTO_HP12		0xbb
#define COD3035X_BC_ODSEL0		0xbc
#define COD3035X_DC_CTRL_EPS		0xdc
#define COD3035X_CTMI_EP_A		GENMASK(7, 6)
#define COD3035X_F2_PRESET_AVC		0xf2
#define COD3035X_F3_PRESET_AVC		0xf3
#define COD3035X_F5_PRESET_AVC		0xf5
#define COD3035X_F6_PRESET_AVC		0xf6

#define COD3035X_D0_CTRL_IREF1		0xd0
#define COD3035X_CTMI_VCM		GENMASK(6, 4)
#define COD3035X_CTMI_VCM_4U		2
#define COD3035X_CTMI_MIX		GENMASK(2, 0)
#define COD3035X_CTMI_MIX_2U		2

/* S2MPU08 LDO36/LDO37, the codec supplies (SPEEDY type 1). */
#define S2MPU08_L36CTRL			0x50
#define S2MPU08_L37CTRL			0x51
#define S2MPU08_LDO_ENABLE		GENMASK(7, 6)

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

/* Vendor delay before the ADC is unmuted, past the input settling. */
#define COD3035X_ADC_UNMUTE_MS		200

/* The codec interrupt reaches the AP through the PMIC; poll instead. */
#define COD3035X_JACK_POLL_MS		250
/* Vendor mic_det_delay between the AP jack-in and the status read. */
#define COD3035X_JACK_SETTLE_MS		300

struct cod3035x {
	struct device *dev;
	struct regmap *regmap;
	const struct acpm_handle *acpm;
	unsigned int acpm_chan;
	struct snd_soc_jack *jack;
	struct delayed_work jack_work;
	struct delayed_work adc_unmute_work;
	/* Microphone inputs powered, by index; serialized by DAPM. */
	unsigned long mics_on;
	bool jack_in;
	bool dac_on;
	bool capture_on;
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
	/* LassenA additions from the vendor codec probe */
	{ 0x57, 0x80 }, { 0xbe, 0x00 }, { 0x5a, 0x05 }, { 0x55, 0x6e },
	{ 0xf4, 0xd3 }, { 0xe9, 0x1f }, { 0xea, 0x1f }, { 0xe1, 0x22 },
};

static int cod3035x_enable_ldo(struct cod3035x *cod, u8 reg)
{
	u8 val;
	int ret;

	ret = cod3035x_acpm_read(cod, COD3035X_ACPM_TYPE_PMIC, reg, &val);
	if (ret)
		return ret;
	if ((val & S2MPU08_LDO_ENABLE) == S2MPU08_LDO_ENABLE)
		return 0;
	ret = cod3035x_acpm_write(cod, COD3035X_ACPM_TYPE_PMIC, reg,
				  val | S2MPU08_LDO_ENABLE);
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	return 0;
}

/*
 * The vendor keeps both codec supplies always on. LDO36 feeds the core
 * (without it core registers read back as zero); LDO37 feeds the headphone
 * output stage (without it the DAC runs but the jack only pops). Nothing
 * else describes the PMIC on this board, so switch both on here, keeping
 * their boot-time voltages.
 */
static int cod3035x_enable_supply(struct cod3035x *cod)
{
	int ret;

	ret = cod3035x_enable_ldo(cod, S2MPU08_L36CTRL);
	if (ret)
		return ret;
	return cod3035x_enable_ldo(cod, S2MPU08_L37CTRL);
}

static int cod3035x_hw_init(struct cod3035x *cod)
{
	struct regmap *map = cod->regmap;
	int ret;

	ret = cod3035x_enable_supply(cod);
	if (ret)
		return ret;

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
	ret = regmap_update_bits(map, COD3035X_D0_CTRL_IREF1,
				 COD3035X_CTMI_VCM | COD3035X_CTMI_MIX,
				 FIELD_PREP(COD3035X_CTMI_VCM, COD3035X_CTMI_VCM_4U) |
				 FIELD_PREP(COD3035X_CTMI_MIX, COD3035X_CTMI_MIX_2U));
	if (ret)
		return ret;
	/*
	 * T510 vendor DT: mic-bias1-voltage and mic-bias2-voltage 2.8 V,
	 * mic-bias-ldo-voltage 3.3 V.
	 */
	ret = regmap_update_bits(map, COD3035X_19_CTRL_REF, COD3035X_CTRV_MCB1,
				 FIELD_PREP(COD3035X_CTRV_MCB1,
					    COD3035X_MCB_2_8V));
	if (ret)
		return ret;
	ret = regmap_update_bits(map, COD3035X_83_CTR_MCB,
				 COD3035X_CTRV_MCB2 | COD3035X_CTRV_MCB_LDO,
				 FIELD_PREP(COD3035X_CTRV_MCB2, COD3035X_MCB_2_8V) |
				 FIELD_PREP(COD3035X_CTRV_MCB_LDO,
					    COD3035X_MCB_LDO_3_3V));
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
		cod->dac_on = true;
		/* DAC digital power on, then pulse its data reset. */
		regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				COD3035X_PDB_DACDIG);
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_DA);
		return regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				       COD3035X_RSTB_DAT_DA);
	case SND_SOC_DAPM_PRE_PMD:
		cod->dac_on = false;
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_DA);
		/* The vendor recording path also runs with DAC digital power. */
		if (cod->capture_on)
			return 0;
		return regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
					 COD3035X_PDB_DACDIG);
	}
	return 0;
}

static int cod3035x_adc_mute(struct cod3035x *cod, bool mute)
{
	if (mute)
		return regmap_set_bits(cod->regmap, COD3035X_46_ADC1,
				       COD3035X_ADC_MUTE);
	return regmap_clear_bits(cod->regmap, COD3035X_46_ADC1,
				 COD3035X_ADC_MUTE);
}

/* cod3035x_capture_init and its manual-mode reference charging. */
static int cod3035x_vmid_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);
	struct regmap *map = cod->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod->capture_on = true;
		regmap_write(map, COD3035X_44_IF1_FORMAT4, COD3035X_ADC_SEL_OFF);
		cod3035x_adc_mute(cod, true);
		regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				COD3035X_PDB_DACDIG | COD3035X_PDB_ADCDIG);
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_AD);
		regmap_set_bits(map, COD3035X_40_DIGITAL_POWER,
				COD3035X_RSTB_DAT_AD);
		regmap_set_bits(map, COD3035X_4B_ADC2, COD3035X_PDB_ADCDIG2);

		regmap_write(map, COD3035X_BC_ODSEL0, 0x03);
		regmap_set_bits(map, COD3035X_10_PD_REF, COD3035X_PDB_VMID);
		/* VMID fast charging until the first microphone is up. */
		regmap_update_bits(map, COD3035X_19_CTRL_REF, COD3035X_CTMF_VMID,
				   FIELD_PREP(COD3035X_CTMF_VMID,
					      COD3035X_CTMF_VMID_1K));
		/* VREFP_AD pre-charging */
		regmap_set_bits(map, COD3035X_11_PD_AD1,
				COD3035X_EN_DSML_PREQ | COD3035X_EN_DSMR_PREQ);
		regmap_set_bits(map, COD3035X_13_PD_AD3, COD3035X_EN_DSMC_PREQ);
		return regmap_set_bits(map, COD3035X_10_PD_REF,
				       COD3035X_PDB_IGEN);
	case SND_SOC_DAPM_PRE_PMD:
		regmap_clear_bits(map, COD3035X_10_PD_REF, COD3035X_PDB_VMID);
		regmap_write(map, COD3035X_BC_ODSEL0, 0x00);
		regmap_clear_bits(map, COD3035X_4B_ADC2, COD3035X_PDB_ADCDIG2);
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_RSTB_DAT_AD);
		regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
				  COD3035X_PDB_ADCDIG);
		if (!cod->dac_on)
			regmap_clear_bits(map, COD3035X_40_DIGITAL_POWER,
					  COD3035X_PDB_DACDIG);
		cod->capture_on = false;
		return regmap_write(map, COD3035X_44_IF1_FORMAT4,
				    COD3035X_ADC_SEL_OFF);
	}
	return 0;
}

/* MICBIAS1 shares the bias LDO with MICBIAS2 (headset, unused here). */
static int cod3035x_micbias1_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);
	unsigned int ref, mask = COD3035X_PDB_MCB1;
	int ret;

	ret = regmap_read(cod->regmap, COD3035X_10_PD_REF, &ref);
	if (ret)
		return ret;
	if (!(ref & COD3035X_PDB_MCB2_CODEC))
		mask |= COD3035X_PDB_MCB_LDO_CODEC;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return regmap_set_bits(cod->regmap, COD3035X_10_PD_REF, mask);
	case SND_SOC_DAPM_POST_PMD:
		return regmap_clear_bits(cod->regmap, COD3035X_10_PD_REF, mask);
	}
	return 0;
}

/* cod3035_power_on_mic1/2: the widget's shift is the microphone index. */
static void cod3035x_mic_power_on(struct cod3035x *cod, unsigned int n)
{
	unsigned int mix_lr = COD3035X_EN_MIX_MICL(n) | COD3035X_EN_MIX_MICR(n);
	unsigned int mix_c = COD3035X_EN_MIX_MICC(n);
	struct regmap *map = cod->regmap;
	unsigned int mix1 = 0, mix2 = 0;

	/* Keep the mixer inputs off while the input stage powers up. */
	regmap_read(map, COD3035X_25_MIX_AD1, &mix1);
	regmap_read(map, COD3035X_26_MIX_ADC, &mix2);
	regmap_clear_bits(map, COD3035X_25_MIX_AD1, mix_lr);
	regmap_clear_bits(map, COD3035X_26_MIX_ADC, mix_c);

	regmap_set_bits(map, COD3035X_10_PD_REF, COD3035X_PDB_IGEN_AD);
	regmap_set_bits(map, COD3035X_12_PD_AD2, COD3035X_PDB_MIC(n));
	regmap_set_bits(map, COD3035X_11_PD_AD1,
			COD3035X_PDB_MIXL | COD3035X_PDB_MIXR);
	regmap_set_bits(map, COD3035X_13_PD_AD3, COD3035X_PDB_MIXC);
	regmap_set_bits(map, COD3035X_11_PD_AD1,
			COD3035X_PDB_DSML | COD3035X_PDB_DSMR);
	regmap_set_bits(map, COD3035X_13_PD_AD3, COD3035X_PDB_DSMC);
	regmap_clear_bits(map, COD3035X_13_PD_AD3, COD3035X_RESETB_BST(n));

	regmap_update_bits(map, COD3035X_25_MIX_AD1, mix_lr, mix1);
	regmap_update_bits(map, COD3035X_26_MIX_ADC, mix_c, mix2);
	msleep(50);

	regmap_clear_bits(map, COD3035X_13_PD_AD3, COD3035X_EN_BST_DIODE);
	regmap_set_bits(map, COD3035X_13_PD_AD3, COD3035X_RESETB_BST(n));
	/* End VREFP_AD pre-charging and VMID fast charging. */
	regmap_clear_bits(map, COD3035X_11_PD_AD1,
			  COD3035X_EN_DSML_PREQ | COD3035X_EN_DSMR_PREQ);
	regmap_clear_bits(map, COD3035X_13_PD_AD3, COD3035X_EN_DSMC_PREQ);
	regmap_update_bits(map, COD3035X_19_CTRL_REF, COD3035X_CTMF_VMID,
			   FIELD_PREP(COD3035X_CTMF_VMID,
				      COD3035X_CTMF_VMID_60K));
	regmap_set_bits(map, COD3035X_11_PD_AD1,
			COD3035X_RESETB_DSML | COD3035X_RESETB_DSMR);
	regmap_set_bits(map, COD3035X_13_PD_AD3, COD3035X_RESETB_DSMC);
	__set_bit(n, &cod->mics_on);
}

static void cod3035x_mic_power_off(struct cod3035x *cod, unsigned int n)
{
	struct regmap *map = cod->regmap;

	__clear_bit(n, &cod->mics_on);
	/* The ADC modulators and mixers are shared by the inputs. */
	if (!cod->mics_on) {
		regmap_clear_bits(map, COD3035X_11_PD_AD1,
				  COD3035X_RESETB_DSML | COD3035X_RESETB_DSMR);
		regmap_clear_bits(map, COD3035X_13_PD_AD3,
				  COD3035X_RESETB_DSMC);
		regmap_clear_bits(map, COD3035X_11_PD_AD1,
				  COD3035X_PDB_DSML | COD3035X_PDB_DSMR);
		regmap_clear_bits(map, COD3035X_13_PD_AD3, COD3035X_PDB_DSMC);
		regmap_clear_bits(map, COD3035X_11_PD_AD1,
				  COD3035X_PDB_MIXL | COD3035X_PDB_MIXR);
		regmap_clear_bits(map, COD3035X_13_PD_AD3, COD3035X_PDB_MIXC);
	}
	regmap_clear_bits(map, COD3035X_12_PD_AD2, COD3035X_PDB_MIC(n));
	if (!cod->mics_on)
		regmap_clear_bits(map, COD3035X_10_PD_REF,
				  COD3035X_PDB_IGEN_AD);
}

static int cod3035x_mic_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3035x_mic_power_on(cod, w->shift);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		cod3035x_mic_power_off(cod, w->shift);
		break;
	}
	return 0;
}

static void cod3035x_adc_unmute_work(struct work_struct *work)
{
	struct cod3035x *cod = container_of(to_delayed_work(work),
					    struct cod3035x, adc_unmute_work);

	cod3035x_adc_mute(cod, false);
	usleep_range(2000, 2100);
	regmap_write(cod->regmap, COD3035X_44_IF1_FORMAT4, COD3035X_ADC_SEL_LR);
}

static int cod3035x_adc_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Unmute once the inputs have settled, as the vendor does. */
		queue_delayed_work(system_power_efficient_wq,
				   &cod->adc_unmute_work,
				   msecs_to_jiffies(COD3035X_ADC_UNMUTE_MS));
		return 0;
	case SND_SOC_DAPM_PRE_PMD:
		cancel_delayed_work_sync(&cod->adc_unmute_work);
		return cod3035x_adc_mute(cod, true);
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

/* Boost: 0, 12 or 20 dB. */
static const DECLARE_TLV_DB_RANGE(cod3035x_bst_tlv,
	0, 1, TLV_DB_SCALE_ITEM(0, 1200, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0)
);

/*
 * ADC digital volume, register value descending: 0x00-0x86 is +12 to
 * -55 dB in 0.5 dB steps, then 1, 2 and 6 dB steps down to -84 dB at 0x96.
 */
static const DECLARE_TLV_DB_RANGE(cod3035x_adc_tlv,
	0, 1, TLV_DB_SCALE_ITEM(-8400, 600, 0),
	2, 4, TLV_DB_SCALE_ITEM(-7200, 200, 0),
	5, 15, TLV_DB_SCALE_ITEM(-6600, 100, 0),
	16, COD3035X_AD_DVOL_MAX, TLV_DB_SCALE_ITEM(-5500, 50, 0)
);

static const struct snd_kcontrol_new cod3035x_controls[] = {
	SOC_SINGLE_TLV("MIC1 Boost Volume", COD3035X_20_VOL_AD1,
		       COD3035X_CTVOL_BST_SHIFT, 2, 0, cod3035x_bst_tlv),
	SOC_SINGLE_TLV("MIC2 Boost Volume", COD3035X_21_VOL_AD2,
		       COD3035X_CTVOL_BST_SHIFT, 2, 0, cod3035x_bst_tlv),
	SOC_DOUBLE_R_TLV("ADC Capture Volume", COD3035X_47_AVOLL1,
			 COD3035X_48_AVOLR1, 0, COD3035X_AD_DVOL_MAX, 1,
			 cod3035x_adc_tlv),
};

static const struct snd_kcontrol_new cod3035x_adcl_mix[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", COD3035X_25_MIX_AD1, 5, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", COD3035X_25_MIX_AD1, 3, 1, 0),
};

static const struct snd_kcontrol_new cod3035x_adcr_mix[] = {
	SOC_DAPM_SINGLE("MIC1 Switch", COD3035X_25_MIX_AD1, 4, 1, 0),
	SOC_DAPM_SINGLE("MIC2 Switch", COD3035X_25_MIX_AD1, 2, 1, 0),
};

static const struct snd_soc_dapm_widget cod3035x_dapm_widgets[] = {
	/* The DAC and HP power sequencer run from MCLK; it exists only while
	 * the ABOX firmware is up, so it is a stream-time supply.
	 */
	SND_SOC_DAPM_CLOCK_SUPPLY("mclk"),
	SND_SOC_DAPM_DAC_E("DAC", "AIF Playback", SND_SOC_NOPM, 0, 0,
			   cod3035x_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cod3035x_hpdrv_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("HPOUT"),

	SND_SOC_DAPM_INPUT("IN1L"),
	SND_SOC_DAPM_INPUT("IN2L"),
	SND_SOC_DAPM_SUPPLY("VMID", SND_SOC_NOPM, 0, 0, cod3035x_vmid_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("MICBIAS1", SND_SOC_NOPM, 0, 0,
			    cod3035x_micbias1_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("MIC1 PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
			   cod3035x_mic_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("MIC2 PGA", SND_SOC_NOPM, 1, 0, NULL, 0,
			   cod3035x_mic_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER("ADCL Mixer", SND_SOC_NOPM, 0, 0,
			   cod3035x_adcl_mix, ARRAY_SIZE(cod3035x_adcl_mix)),
	SND_SOC_DAPM_MIXER("ADCR Mixer", SND_SOC_NOPM, 0, 0,
			   cod3035x_adcr_mix, ARRAY_SIZE(cod3035x_adcr_mix)),
	SND_SOC_DAPM_ADC_E("ADC", "AIF Capture", SND_SOC_NOPM, 0, 0,
			   cod3035x_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route cod3035x_dapm_routes[] = {
	{ "DAC", NULL, "mclk" },
	{ "HPDRV", NULL, "DAC" },
	{ "HPOUT", NULL, "HPDRV" },

	{ "MIC1 PGA", NULL, "IN1L" },
	{ "MIC1 PGA", NULL, "VMID" },
	{ "MIC2 PGA", NULL, "IN2L" },
	{ "MIC2 PGA", NULL, "VMID" },
	{ "ADCL Mixer", "MIC1 Switch", "MIC1 PGA" },
	{ "ADCL Mixer", "MIC2 Switch", "MIC2 PGA" },
	{ "ADCR Mixer", "MIC1 Switch", "MIC1 PGA" },
	{ "ADCR Mixer", "MIC2 Switch", "MIC2 PGA" },
	{ "ADC", NULL, "ADCL Mixer" },
	{ "ADC", NULL, "ADCR Mixer" },
	{ "ADC", NULL, "mclk" },
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
	/* capture_hw_params: normal (non-UHQA) recording mode. */
	ret = regmap_clear_bits(cod->regmap, COD3035X_53_UHQA,
				COD3035X_UHQA_MODE);
	if (ret)
		return ret;
	return regmap_clear_bits(cod->regmap, COD3035X_4A_DMIC2,
				 COD3035X_DMIC_ST);
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
	.capture = {
		.stream_name = "AIF Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cod3035x_dai_ops,
};

/*
 * On insertion the codec raises "AP check" and waits for the AP to rule out
 * moisture (the vendor reads a ground-detect ADC) and to answer with AP jack
 * in; unanswered, it falls back to jack out. There is no ADC described on
 * this board, so confirm every insertion. As in the vendor jack work, then
 * release the override and keep the jack only if the codec still sees it.
 */
static bool cod3035x_jack_confirm(struct cod3035x *cod)
{
	struct regmap *map = cod->regmap;
	unsigned int status;

	if (regmap_write(map, COD3035X_93_CTR_DLY6, COD3035X_AP_JACK_IN))
		return false;
	msleep(COD3035X_JACK_SETTLE_MS);
	if (regmap_read(map, COD3035X_09_STATUS1, &status))
		status = 0;
	regmap_write(map, COD3035X_93_CTR_DLY6, 0);
	if (!(status & COD3035X_STATUS1_JACK_DET))
		return false;
	/* Only headphones are supported, so treat every jack as 3-pole. */
	return !regmap_write(map, COD3035X_94_JACK_CTR,
			     COD3035X_JACK_CTR_3POLE);
}

static void cod3035x_jack_work(struct work_struct *work)
{
	struct cod3035x *cod = container_of(to_delayed_work(work),
					    struct cod3035x, jack_work);
	struct regmap *map = cod->regmap;
	unsigned int pend, status;
	bool in = cod->jack_in;

	/* The pending bits latch events between polls. */
	if (regmap_read(map, COD3035X_01_IRQ1PEND, &pend) ||
	    regmap_read(map, COD3035X_09_STATUS1, &status))
		goto requeue;

	if (!in && ((status & COD3035X_STATUS1_JACK_DET) ||
		    (pend & (COD3035X_IRQ1_ST_APCHECK_R |
			     COD3035X_IRQ1_ST_JACKIN_R)))) {
		in = cod3035x_jack_confirm(cod);
	} else if (in && (!(status & COD3035X_STATUS1_JACK_DET) ||
			  (pend & COD3035X_IRQ1_ST_JACKOUT_R))) {
		regmap_write(map, COD3035X_93_CTR_DLY6, 0);
		regmap_write(map, COD3035X_94_JACK_CTR, 0);
		in = false;
	}

	if (in != cod->jack_in) {
		cod->jack_in = in;
		dev_dbg(cod->dev, "headphone jack %s\n", in ? "in" : "out");
		snd_soc_jack_report(cod->jack, in ? SND_JACK_HEADPHONE : 0,
				    SND_JACK_HEADPHONE);
	}
requeue:
	queue_delayed_work(system_power_efficient_wq, &cod->jack_work,
			   msecs_to_jiffies(COD3035X_JACK_POLL_MS));
}

static int cod3035x_set_jack(struct snd_soc_component *component,
			     struct snd_soc_jack *jack, void *data)
{
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&cod->jack_work);
	cod->jack = jack;
	cod->jack_in = false;
	if (jack)
		queue_delayed_work(system_power_efficient_wq, &cod->jack_work,
				   0);
	return 0;
}

static void cod3035x_component_remove(struct snd_soc_component *component)
{
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&cod->jack_work);
	cancel_delayed_work_sync(&cod->adc_unmute_work);
}

static int cod3035x_component_probe(struct snd_soc_component *component)
{
	struct cod3035x *cod = snd_soc_component_get_drvdata(component);

	return cod3035x_hw_init(cod);
}

static const struct snd_soc_component_driver cod3035x_component = {
	.probe = cod3035x_component_probe,
	.remove = cod3035x_component_remove,
	.set_jack = cod3035x_set_jack,
	.controls = cod3035x_controls,
	.num_controls = ARRAY_SIZE(cod3035x_controls),
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
	INIT_DELAYED_WORK(&cod->jack_work, cod3035x_jack_work);
	INIT_DELAYED_WORK(&cod->adc_unmute_work, cod3035x_adc_unmute_work);
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

MODULE_DESCRIPTION("Samsung COD3035X headphone and microphone codec");
MODULE_LICENSE("GPL");
