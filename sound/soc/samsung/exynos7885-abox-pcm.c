// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7885 ABOX RDMA0 playback PCM component. */

#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/samsung/exynos7885-abox.h>
#include <linux/uaccess.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define ABOX_PCM_BUFFER_BYTES	SZ_128K
#define ABOX_PCM_PERIOD_MIN	SZ_4K
#define ABOX_PCM_PERIOD_MAX	SZ_64K
#define ABOX_PCM_PERIODS_MIN	2
#define ABOX_PCM_PERIODS_MAX	32
#define ABOX_PCM_CHANNEL	0
#define ABOX_PCM_POINTER	4

struct exynos7885_abox_pcm {
	struct device *abox_dev;
	void *area;
	phys_addr_t phys;
	size_t size;
	struct snd_pcm_substream *substream;
	snd_pcm_uframes_t hw_ptr;
	snd_pcm_uframes_t appl_ptr;
	bool booted;
	bool opened;
	bool mapped;
	bool configured;
	bool running;
	bool dai_fmt_valid;
	bool invert_bclk;
	bool invert_frame;
	bool abox_master;
	unsigned int dai_format;
};

static int exynos7885_abox_pcm_hw_free(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream);

static const struct snd_pcm_hardware exynos7885_abox_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = ABOX_PCM_BUFFER_BYTES,
	.period_bytes_min = ABOX_PCM_PERIOD_MIN,
	.period_bytes_max = ABOX_PCM_PERIOD_MAX,
	.periods_min = ABOX_PCM_PERIODS_MIN,
	.periods_max = ABOX_PCM_PERIODS_MAX,
};

static void exynos7885_abox_pcm_ipc(struct device *dev, const u32 *message,
					    void *data)
{
	struct exynos7885_abox_pcm *pcm = data;
	struct snd_pcm_substream *substream = READ_ONCE(pcm->substream);
	u32 pointer;

	if (!substream || !READ_ONCE(pcm->configured) ||
	    !READ_ONCE(pcm->running) ||
	    message[0] != EXYNOS7885_ABOX_IPC_PCM_PLAYBACK ||
	    message[2] != EXYNOS7885_ABOX_PCM_POINTER ||
	    message[3] != ABOX_PCM_CHANNEL)
		return;

	pointer = message[ABOX_PCM_POINTER];
	if (pointer >= 0x81000000)
		pointer -= 0x81000000;
	if (pointer >= substream->runtime->dma_bytes)
		return;

	WRITE_ONCE(pcm->hw_ptr,
		   bytes_to_frames(substream->runtime, pointer));
	snd_pcm_period_elapsed(substream);
}

static int exynos7885_abox_pcm_open(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	int ret;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;

	ret = exynos7885_abox_boot(pcm->abox_dev);
	if (ret)
		return ret;
	pcm->booted = true;

	ret = exynos7885_abox_get_pcm_buffer(pcm->abox_dev, &pcm->area,
					    &pcm->phys, &pcm->size);
	if (ret)
		goto shutdown;

	snd_soc_set_runtime_hwparams(substream, &exynos7885_abox_pcm_hardware);
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret)
		goto shutdown;

	ret = exynos7885_abox_register_ipc_handler(pcm->abox_dev,
						   exynos7885_abox_pcm_ipc, pcm);
	if (ret)
		goto shutdown;
	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_OPEN, 0, 0, 0);
	if (ret) {
		exynos7885_abox_unregister_ipc_handler(pcm->abox_dev,
						       exynos7885_abox_pcm_ipc);
		goto shutdown;
	}
	pcm->opened = true;

	WRITE_ONCE(pcm->substream, substream);
	pcm->hw_ptr = 0;
	return 0;

shutdown:
	if (!exynos7885_abox_shutdown(pcm->abox_dev))
		pcm->booted = false;
	return ret;
}

static int exynos7885_abox_pcm_close(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	int ret = 0;

	WRITE_ONCE(pcm->substream, NULL);
	exynos7885_abox_unregister_ipc_handler(pcm->abox_dev,
					       exynos7885_abox_pcm_ipc);
	if (READ_ONCE(pcm->running))
		return -EBUSY;
	if (pcm->configured) {
		ret = exynos7885_abox_pcm_hw_free(component, substream);
		if (ret)
			return ret;
	}
	if (pcm->opened) {
		ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
					       EXYNOS7885_ABOX_PCM_CLOSE, 0, 0, 0);
		if (ret)
			return ret;
		pcm->opened = false;
	}
	if (pcm->mapped) {
		ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
		if (ret)
			return ret;
		pcm->mapped = false;
	}
	if (pcm->booted) {
		ret = exynos7885_abox_shutdown(pcm->abox_dev);
		if (!ret)
			pcm->booted = false;
	}
	return ret;
}

static int exynos7885_abox_pcm_copy(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    int channel, unsigned long pos,
				    struct iov_iter *iter, unsigned long bytes)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t ring_bytes = runtime->dma_bytes;
	size_t first, second;

	if (channel || !ring_bytes || pos >= ring_bytes || bytes > ring_bytes)
		return -EINVAL;

	first = min_t(size_t, bytes, ring_bytes - pos);
	if (!copy_from_iter_full((u8 *)pcm->area + pos, first, iter))
		return -EFAULT;
	second = bytes - first;
	if (second && !copy_from_iter_full(pcm->area, second, iter))
		return -EFAULT;

	if (first) {
		int ret = exynos7885_abox_sync_pcm_buffer(pcm->abox_dev, pos, first);

		if (ret)
			return ret;
	}
	if (second)
		return exynos7885_abox_sync_pcm_buffer(pcm->abox_dev, 0, second);

	return 0;
}

static int exynos7885_abox_pcm_ack(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t appl_ptr = READ_ONCE(runtime->control->appl_ptr);
	snd_pcm_uframes_t frames;
	size_t offset, bytes, first;
	int ret;

	if (!pcm->mapped || !runtime->buffer_size)
		return 0;
	frames = appl_ptr - pcm->appl_ptr;
	if (frames > runtime->buffer_size)
		frames = runtime->buffer_size;
	pcm->appl_ptr = appl_ptr;
	bytes = frames_to_bytes(runtime, frames);
	if (!bytes)
		return 0;
	offset = frames_to_bytes(runtime, (appl_ptr - frames) %
				 runtime->buffer_size);
	first = min_t(size_t, bytes, runtime->dma_bytes - offset);
	ret = exynos7885_abox_sync_pcm_buffer(pcm->abox_dev, offset, first);
	if (ret)
		return ret;
	if (bytes > first)
		return exynos7885_abox_sync_pcm_buffer(pcm->abox_dev, 0,
						       bytes - first);

	return 0;
}

static int exynos7885_abox_pcm_hw_params(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret, cleanup_ret;

	if (params_buffer_bytes(params) > pcm->size ||
	    params_periods(params) < ABOX_PCM_PERIODS_MIN)
		return -EINVAL;

	ret = exynos7885_abox_map_pcm_buffer(pcm->abox_dev);
	if (ret)
		return ret;
	pcm->mapped = true;

	memset(pcm->area, 0, pcm->size);
	runtime->dma_area = pcm->area;
	runtime->dma_addr = 0x81000000;
	runtime->dma_bytes = params_buffer_bytes(params);

	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_SET_BUFFER,
				       0x81000000, params_period_bytes(params),
				       params_periods(params));
	if (ret)
		goto free_fw;

	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_HW_PARAMS,
				       params_rate(params),
				       params_width(params),
				       params_channels(params));
	if (ret)
		goto free_fw;

	WRITE_ONCE(pcm->configured, true);
	pcm->hw_ptr = 0;
	pcm->appl_ptr = 0;
	return 0;

free_fw:
	cleanup_ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
					       EXYNOS7885_ABOX_PCM_HW_FREE,
					       0, 0, 0);
	if (cleanup_ret)
		return ret;
	cleanup_ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
	if (cleanup_ret)
		return cleanup_ret;
	pcm->mapped = false;
	runtime->dma_area = NULL;
	runtime->dma_bytes = 0;
	return ret;
}

static int exynos7885_abox_pcm_hw_free(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	int ret;

	if (!pcm->configured)
		return 0;
	if (READ_ONCE(pcm->running))
		return -EBUSY;

	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_HW_FREE, 0, 0, 0);
	if (ret)
		return ret;
	ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
	if (ret)
		return ret;

	pcm->mapped = false;
	WRITE_ONCE(pcm->configured, false);
	return 0;
}

static int exynos7885_abox_pcm_prepare(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	int ret;

	if (!pcm->configured)
		return -EINVAL;
	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_PREPARE, 0, 0, 0);
	if (!ret)
		pcm->hw_ptr = 0;
	if (!ret)
		pcm->appl_ptr = READ_ONCE(substream->runtime->control->appl_ptr);
	return ret;
}

static int exynos7885_abox_pcm_trigger(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream,
				       int cmd)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	bool start;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		start = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		start = false;
		break;
	default:
		return -EINVAL;
	}

	if (start) {
		ret = exynos7885_abox_uaif3_set_enabled(pcm->abox_dev, true);
		if (ret)
			return ret;
		WRITE_ONCE(pcm->running, true);
	}
	ret = exynos7885_abox_send_pcm(pcm->abox_dev, ABOX_PCM_CHANNEL,
				       EXYNOS7885_ABOX_PCM_TRIGGER, start, 0, 0);
	if (ret) {
		if (start &&
		    !exynos7885_abox_uaif3_set_enabled(pcm->abox_dev, false))
			WRITE_ONCE(pcm->running, false);
		return ret;
	}
	if (start)
		return 0;
	ret = exynos7885_abox_wait_rdma_idle(pcm->abox_dev, ABOX_PCM_CHANNEL,
					     20000);
	if (ret)
		return ret;
	ret = exynos7885_abox_uaif3_set_enabled(pcm->abox_dev, false);
	if (!ret)
		WRITE_ONCE(pcm->running, false);
	return ret;
}

static snd_pcm_uframes_t
exynos7885_abox_pcm_pointer(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);

	return READ_ONCE(pcm->hw_ptr);
}

static const struct snd_soc_component_driver exynos7885_abox_component = {
	.name = "exynos7885-abox-pcm",
	.open = exynos7885_abox_pcm_open,
	.close = exynos7885_abox_pcm_close,
	.copy = exynos7885_abox_pcm_copy,
	.ack = exynos7885_abox_pcm_ack,
	.hw_params = exynos7885_abox_pcm_hw_params,
	.hw_free = exynos7885_abox_pcm_hw_free,
	.prepare = exynos7885_abox_pcm_prepare,
	.trigger = exynos7885_abox_pcm_trigger,
	.pointer = exynos7885_abox_pcm_pointer,
};

static int exynos7885_abox_dai_set_fmt(struct snd_soc_dai *dai,
				       unsigned int fmt)
{
	struct exynos7885_abox_pcm *pcm =
		snd_soc_component_get_drvdata(dai->component);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		pcm->dai_format = EXYNOS7885_ABOX_UAIF3_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		pcm->dai_format = EXYNOS7885_ABOX_UAIF3_DSP_A;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		pcm->invert_bclk = false;
		pcm->invert_frame = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		pcm->invert_bclk = false;
		pcm->invert_frame = true;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		pcm->invert_bclk = true;
		pcm->invert_frame = false;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		pcm->invert_bclk = true;
		pcm->invert_frame = true;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		pcm->abox_master = true;
		break;
	default:
		return -EOPNOTSUPP;
	}

	pcm->dai_fmt_valid = true;
	return 0;
}

static int exynos7885_abox_dai_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	struct exynos7885_abox_pcm *pcm =
		snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (!pcm->dai_fmt_valid)
		return -EINVAL;

	ret = exynos7885_abox_uaif3_set_fmt(pcm->abox_dev, pcm->dai_format,
					    pcm->invert_bclk,
					    pcm->invert_frame,
					    pcm->abox_master);
	if (ret)
		return ret;

	return exynos7885_abox_uaif3_hw_params(pcm->abox_dev,
					       params_rate(params),
					       params_width(params),
					       params_channels(params));
}

static const struct snd_soc_dai_ops exynos7885_abox_dai_ops = {
	.set_fmt = exynos7885_abox_dai_set_fmt,
	.hw_params = exynos7885_abox_dai_hw_params,
};

static struct snd_soc_dai_driver exynos7885_abox_dai = {
	.name = "exynos7885-abox-uaif3",
	.playback = {
		.stream_name = "UAIF3 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &exynos7885_abox_dai_ops,
};

static int exynos7885_abox_pcm_probe(struct platform_device *pdev)
{
	struct exynos7885_abox_pcm *pcm;
	int ret;

	if (!pdev->dev.parent)
		return -ENODEV;

	pcm = devm_kzalloc(&pdev->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;
	pcm->abox_dev = pdev->dev.parent;
	platform_set_drvdata(pdev, pcm);

	ret = exynos7885_abox_get_pcm_buffer(pcm->abox_dev, &pcm->area,
					     &pcm->phys, &pcm->size);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "ABOX PCM carveout is unavailable\n");

	return devm_snd_soc_register_component(&pdev->dev,
					       &exynos7885_abox_component,
					       &exynos7885_abox_dai, 1);
}

static const struct of_device_id exynos7885_abox_pcm_of_match[] = {
	{ .compatible = "samsung,exynos7885-abox-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos7885_abox_pcm_of_match);

static struct platform_driver exynos7885_abox_pcm_driver = {
	.probe = exynos7885_abox_pcm_probe,
	.driver = {
		.name = "exynos7885-abox-audio",
		.of_match_table = exynos7885_abox_pcm_of_match,
	},
};
module_platform_driver(exynos7885_abox_pcm_driver);

MODULE_DESCRIPTION("Exynos7885 ABOX RDMA0 playback PCM");
MODULE_LICENSE("GPL");
