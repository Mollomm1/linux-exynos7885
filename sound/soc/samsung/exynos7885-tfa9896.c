// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos7885 GTA3XL Wi-Fi card: UAIF3 to two TFA9896 speaker amplifiers and,
 * with a "headphone" node, UAIF0 to the COD3035X headphone codec. Both are
 * fed by the same playback stream.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/soc.h>

#define GTA3XL_NUM_AMPS		2
#define GTA3XL_MAX_CPUS		2
#define GTA3XL_MAX_CODECS	(GTA3XL_NUM_AMPS + 1)

struct exynos7885_tfa9896_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpus[GTA3XL_MAX_CPUS];
	struct snd_soc_dai_link_component codecs[GTA3XL_MAX_CODECS];
	struct snd_soc_dai_link_ch_map ch_maps[GTA3XL_MAX_CODECS];
};

static const struct snd_soc_dapm_widget exynos7885_tfa9896_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
};

/*
 * The amplifiers take DSP_A frames from UAIF3 and the headphone codec takes
 * I2S from UAIF0, so the formats are set per DAI rather than per link. ABOX
 * drives BCLK and the frame clock on both.
 */
static int exynos7885_tfa9896_init(struct snd_soc_pcm_runtime *rtd)
{
	const unsigned int amp_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF;
	const unsigned int hp_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;
	struct snd_soc_dai *dai;
	unsigned int fmt;
	int i, ret;

	for_each_rtd_cpu_dais(rtd, i, dai) {
		fmt = (i ? hp_fmt : amp_fmt) | SND_SOC_DAIFMT_BP_FP;
		ret = snd_soc_dai_set_fmt(dai, fmt);
		if (ret)
			return ret;
	}
	for_each_rtd_codec_dais(rtd, i, dai) {
		fmt = (i < GTA3XL_NUM_AMPS ? amp_fmt : hp_fmt) |
		      SND_SOC_DAIFMT_CBC_CFC;
		ret = snd_soc_dai_set_fmt(dai, fmt);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}
	return 0;
}

static void exynos7885_tfa9896_put_node(void *data)
{
	of_node_put(data);
}

static int exynos7885_tfa9896_get_dlc(struct device *dev,
					     struct device_node *node,
					     const char *property,
					     int index,
					     struct snd_soc_dai_link_component *dlc)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(node, property, "#sound-dai-cells",
					 index, &args);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse %s[%d]\n",
				     property, index);

	ret = snd_soc_get_dlc(&args, dlc);
	if (ret) {
		of_node_put(args.np);
		return dev_err_probe(dev, ret, "failed to resolve %s[%d]\n",
				     property, index);
	}

	/* Keep args.np referenced for the lifetime of the DAI link. */
	ret = devm_add_action_or_reset(dev, exynos7885_tfa9896_put_node,
				       args.np);
	if (ret)
		return ret;

	return 0;
}

static int exynos7885_tfa9896_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *cpu_node, *codec_node, *hp_node, *hp_cpu, *hp_codec;
	struct exynos7885_tfa9896_card *priv;
	struct snd_soc_card *card;
	int num_codecs, i, ret;

	cpu_node = of_get_child_by_name(node, "cpu");
	codec_node = of_get_child_by_name(node, "codecs");
	hp_node = of_get_child_by_name(node, "headphone");
	hp_cpu = hp_node ? of_get_child_by_name(hp_node, "cpu") : NULL;
	hp_codec = hp_node ? of_get_child_by_name(hp_node, "codec") : NULL;
	if (hp_node && (!hp_cpu || !hp_codec)) {
		ret = -EINVAL;
		goto put_nodes;
	}
	if (!cpu_node || !codec_node) {
		ret = -EINVAL;
		goto put_nodes;
	}

	num_codecs = of_count_phandle_with_args(codec_node, "sound-dai",
						"#sound-dai-cells");
	if (num_codecs != GTA3XL_NUM_AMPS) {
		ret = dev_err_probe(dev, num_codecs < 0 ? num_codecs : -EINVAL,
				    "expected two TFA9896 codec DAIs\n");
		goto put_nodes;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto put_nodes;
	}

	ret = exynos7885_tfa9896_get_dlc(dev, cpu_node, "sound-dai", 0,
					 &priv->cpus[0]);
	if (ret)
		goto put_nodes;
	for (i = 0; i < num_codecs; i++) {
		ret = exynos7885_tfa9896_get_dlc(dev, codec_node, "sound-dai", i,
						 &priv->codecs[i]);
		if (ret)
			goto put_nodes;
		priv->ch_maps[i] = (struct snd_soc_dai_link_ch_map) {
			.cpu = 0, .codec = i,
		};
	}

	priv->link.name = "UAIF3 Speaker";
	priv->link.stream_name = "Speaker Playback";
	priv->link.num_cpus = 1;
	if (hp_node) {
		ret = exynos7885_tfa9896_get_dlc(dev, hp_cpu, "sound-dai", 0,
						 &priv->cpus[1]);
		if (ret)
			goto put_nodes;
		ret = exynos7885_tfa9896_get_dlc(dev, hp_codec, "sound-dai", 0,
						 &priv->codecs[num_codecs]);
		if (ret)
			goto put_nodes;
		priv->ch_maps[num_codecs] = (struct snd_soc_dai_link_ch_map) {
			.cpu = 1, .codec = num_codecs,
		};
		num_codecs++;
		priv->link.num_cpus = 2;
		priv->link.name = "Speaker and Headphone";
		priv->link.stream_name = "Playback";
		/*
		 * The codec MCLK comes from the ABOX firmware, which stops when
		 * the PCM closes; power the codec down before that, not later.
		 */
		priv->link.ignore_pmdown_time = 1;
	}
	priv->link.cpus = priv->cpus;
	priv->link.codecs = priv->codecs;
	priv->link.num_codecs = num_codecs;
	priv->link.ch_maps = priv->ch_maps;
	priv->link.init = exynos7885_tfa9896_init;
	/* Codec mute uses I2C in trigger context; ABOX also waits for IPC. */
	priv->link.nonatomic = 1;
	priv->link.playback_only = 1;

	card = &priv->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dai_link = &priv->link;
	card->num_links = 1;
	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		goto put_nodes;
	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret)
		goto put_nodes;
	card->dapm_widgets = exynos7885_tfa9896_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(exynos7885_tfa9896_widgets);

	platform_set_drvdata(pdev, priv);
	ret = devm_snd_soc_register_card(dev, card);

put_nodes:
	of_node_put(hp_codec);
	of_node_put(hp_cpu);
	of_node_put(hp_node);
	of_node_put(codec_node);
	of_node_put(cpu_node);
	return ret;
}

static const struct of_device_id exynos7885_tfa9896_card_of_match[] = {
	{ .compatible = "samsung,exynos7885-gta3xlwifi-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos7885_tfa9896_card_of_match);

static struct platform_driver exynos7885_tfa9896_card_driver = {
	.probe = exynos7885_tfa9896_card_probe,
	.driver = {
		.name = "exynos7885-tfa9896-card",
		.of_match_table = exynos7885_tfa9896_card_of_match,
	},
};
module_platform_driver(exynos7885_tfa9896_card_driver);

MODULE_DESCRIPTION("Exynos7885 GTA3XL Wi-Fi speaker and headphone card");
MODULE_LICENSE("GPL");
