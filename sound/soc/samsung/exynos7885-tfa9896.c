// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7885 GTA3XL Wi-Fi UAIF3 dual TFA9896 speaker card. */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/soc.h>

struct exynos7885_tfa9896_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component *codecs;
};

static const struct snd_soc_dapm_widget exynos7885_tfa9896_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

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
	struct device_node *cpu_node, *codec_node;
	struct exynos7885_tfa9896_card *priv;
	struct snd_soc_card *card;
	int num_codecs, i, ret;

	cpu_node = of_get_child_by_name(node, "cpu");
	codec_node = of_get_child_by_name(node, "codecs");
	if (!cpu_node || !codec_node) {
		ret = -EINVAL;
		goto put_nodes;
	}

	num_codecs = of_count_phandle_with_args(codec_node, "sound-dai",
						"#sound-dai-cells");
	if (num_codecs != 2) {
		ret = dev_err_probe(dev, num_codecs < 0 ? num_codecs : -EINVAL,
				    "expected two TFA9896 codec DAIs\n");
		goto put_nodes;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto put_nodes;
	}
	priv->codecs = devm_kcalloc(dev, num_codecs, sizeof(*priv->codecs),
				    GFP_KERNEL);
	if (!priv->codecs) {
		ret = -ENOMEM;
		goto put_nodes;
	}

	ret = exynos7885_tfa9896_get_dlc(dev, cpu_node, "sound-dai", 0,
					 &priv->cpu);
	if (ret)
		goto put_nodes;
	for (i = 0; i < num_codecs; i++) {
		ret = exynos7885_tfa9896_get_dlc(dev, codec_node, "sound-dai", i,
						 &priv->codecs[i]);
		if (ret)
			goto put_nodes;
	}

	priv->link.name = "UAIF3 Speaker";
	priv->link.stream_name = "Speaker Playback";
	priv->link.cpus = &priv->cpu;
	priv->link.num_cpus = 1;
	priv->link.codecs = priv->codecs;
	priv->link.num_codecs = num_codecs;
	priv->link.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			     SND_SOC_DAIFMT_CBC_CFC |
			     SND_SOC_DAIFMT_NB_NF;
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

MODULE_DESCRIPTION("Exynos7885 GTA3XL Wi-Fi dual TFA9896 speaker card");
MODULE_LICENSE("GPL");
