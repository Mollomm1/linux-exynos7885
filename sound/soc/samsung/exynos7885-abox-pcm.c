// SPDX-License-Identifier: GPL-2.0-only
/* Exynos7885 ABOX RDMA0 playback and WDMA1 capture PCM component. */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/samsung/exynos7885-abox.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/*
 * Each direction has a 64 KiB ring, half of the PCM region. Larger
 * playback buffers faulted at the end of the mapping, though that was
 * before the SIFS1 rate was configured and RDMA0 ran unpaced.
 */
#define ABOX_PCM_BUFFER_BYTES	SZ_64K
#define ABOX_PCM_PERIOD_MIN	SZ_4K
#define ABOX_PCM_PERIOD_MAX	SZ_64K
#define ABOX_PCM_PERIODS_MIN	2
#define ABOX_PCM_PERIODS_MAX	32
/* PCM task and DMA numbers: RDMA0 plays, WDMA1 records SIFM0. */
#define ABOX_PCM_PLAYBACK_DMA	0
#define ABOX_PCM_CAPTURE_DMA	1
#define ABOX_PCM_POINTER	4
/* Ring addresses in the ABOX IOVA space, as the firmware reports them. */
#define ABOX_PCM_PLAYBACK_IOVA	0x81000000
#define ABOX_PCM_CAPTURE_IOVA	0x82100000

/*
 * DAI 0 drives UAIF3 (speaker amplifiers), DAI 1 UAIF0 (headphone codec);
 * DAI 2 records from the UAIF0 input (codec microphones).
 */
#define ABOX_PCM_NUM_PLAYBACK_DAIS	2
#define ABOX_PCM_MIC_DAI		2
#define ABOX_PCM_NUM_DAIS		3

struct exynos7885_abox_pcm_uaif {
	unsigned int format;
	bool fmt_valid;
	bool invert_bclk;
	bool invert_frame;
	bool abox_master;
	bool configured;
	bool enabled;
};

struct exynos7885_abox_pcm_stream {
	bool capture;
	unsigned int dma;
	void *area;
	size_t size;
	struct snd_pcm_substream *substream;
	/* The link is nonatomic: period_elapsed takes a mutex, not IRQ-safe. */
	struct work_struct period_work;
	/* Ring geometry for the IPC handler, which must not use the runtime. */
	size_t dma_bytes;
	size_t period_bytes;
	size_t frame_bytes;
	snd_pcm_uframes_t hw_ptr;
	snd_pcm_uframes_t appl_ptr;
	bool booted;
	bool opened;
	bool mapped;
	bool configured;
	bool running;
};

struct exynos7885_abox_pcm {
	struct device *abox_dev;
	struct exynos7885_abox_pcm_stream stream[2];
	/*
	 * Serializes UAIF start/stop with output changes from the control,
	 * and the shared IPC handler registration.
	 */
	struct mutex lock;
	unsigned int ipc_users;
	struct exynos7885_abox_pcm_uaif uaif[ABOX_PCM_NUM_DAIS];
	/* DAI index of the UAIF that plays; both UAIFs share SIFS1. */
	unsigned int output;
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

static struct exynos7885_abox_pcm_stream *
exynos7885_abox_pcm_stream(struct exynos7885_abox_pcm *pcm,
			   struct snd_pcm_substream *substream)
{
	return &pcm->stream[substream->stream == SNDRV_PCM_STREAM_CAPTURE];
}

static int exynos7885_abox_pcm_send(struct exynos7885_abox_pcm *pcm,
				    struct exynos7885_abox_pcm_stream *st,
				    u32 type, u32 param0, u32 param1,
				    u32 param2)
{
	return exynos7885_abox_send_pcm_stream(pcm->abox_dev, st->capture,
					       st->dma, type, param0, param1,
					       param2);
}

static void exynos7885_abox_pcm_ipc(struct device *dev, const u32 *message,
				    void *data)
{
	struct exynos7885_abox_pcm *pcm = data;
	struct exynos7885_abox_pcm_stream *st;
	u32 pointer, base;
	ssize_t pos;

	if (message[0] == EXYNOS7885_ABOX_IPC_PCM_PLAYBACK)
		st = &pcm->stream[SNDRV_PCM_STREAM_PLAYBACK];
	else if (message[0] == EXYNOS7885_ABOX_IPC_PCM_CAPTURE)
		st = &pcm->stream[SNDRV_PCM_STREAM_CAPTURE];
	else
		return;

	if (!READ_ONCE(st->substream) || !READ_ONCE(st->configured) ||
	    !READ_ONCE(st->running) ||
	    message[2] != EXYNOS7885_ABOX_PCM_POINTER ||
	    message[3] != st->dma)
		return;

	pointer = message[ABOX_PCM_POINTER];
	base = st->capture ? ABOX_PCM_CAPTURE_IOVA : ABOX_PCM_PLAYBACK_IOVA;
	if (!pointer) {
		/* Buffer interrupt: read the position from the DMA itself. */
		if (st->capture)
			pos = exynos7885_abox_wdma_pointer(pcm->abox_dev,
							   st->dma,
							   st->dma_bytes,
							   st->period_bytes);
		else
			pos = exynos7885_abox_rdma_pointer(pcm->abox_dev,
							   st->dma,
							   st->dma_bytes,
							   st->period_bytes);
		if (pos < 0)
			return;
		pointer = pos % st->dma_bytes;
	} else if (pointer >= base) {
		pointer -= base;
	}
	if (pointer >= st->dma_bytes)
		return;

	WRITE_ONCE(st->hw_ptr, pointer / st->frame_bytes);
	schedule_work(&st->period_work);
}

static void exynos7885_abox_pcm_period_work(struct work_struct *work)
{
	struct exynos7885_abox_pcm_stream *st =
		container_of(work, struct exynos7885_abox_pcm_stream,
			     period_work);
	struct snd_pcm_substream *substream = READ_ONCE(st->substream);

	if (substream && READ_ONCE(st->running))
		snd_pcm_period_elapsed(substream);
}

/* One IPC handler serves both directions while either is open. */
static int exynos7885_abox_pcm_get_ipc(struct exynos7885_abox_pcm *pcm)
{
	int ret;

	guard(mutex)(&pcm->lock);
	if (!pcm->ipc_users) {
		ret = exynos7885_abox_register_ipc_handler(pcm->abox_dev,
						exynos7885_abox_pcm_ipc, pcm);
		if (ret)
			return ret;
	}
	pcm->ipc_users++;
	return 0;
}

static void exynos7885_abox_pcm_put_ipc(struct exynos7885_abox_pcm *pcm)
{
	guard(mutex)(&pcm->lock);
	if (!--pcm->ipc_users)
		exynos7885_abox_unregister_ipc_handler(pcm->abox_dev,
						       exynos7885_abox_pcm_ipc);
}

static int exynos7885_abox_pcm_open(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	phys_addr_t phys;
	int ret;

	ret = exynos7885_abox_boot(pcm->abox_dev);
	if (ret)
		return ret;
	st->booted = true;

	if (st->capture)
		ret = exynos7885_abox_get_capture_buffer(pcm->abox_dev,
							 &st->area, &st->size);
	else
		ret = exynos7885_abox_get_pcm_buffer(pcm->abox_dev, &st->area,
						     &phys, &st->size);
	if (ret)
		goto shutdown;

	snd_soc_set_runtime_hwparams(substream, &exynos7885_abox_pcm_hardware);
	/* Returns 1 when it narrowed the constraint. */
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto shutdown;
	/*
	 * RDMA0 ran past a ring of 4800-byte periods without wrapping, and the
	 * vendor pointer code masks offsets as powers of two; 16 KiB x 4 plays.
	 */
	ret = snd_pcm_hw_constraint_pow2(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES);
	if (ret < 0)
		goto shutdown;
	ret = snd_pcm_hw_constraint_pow2(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES);
	if (ret < 0)
		goto shutdown;

	ret = exynos7885_abox_pcm_get_ipc(pcm);
	if (ret)
		goto shutdown;
	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_OPEN,
				       0, 0, 0);
	if (ret) {
		exynos7885_abox_pcm_put_ipc(pcm);
		goto shutdown;
	}
	st->opened = true;

	st->hw_ptr = 0;
	WRITE_ONCE(st->substream, substream);
	return 0;

shutdown:
	if (!exynos7885_abox_shutdown(pcm->abox_dev))
		st->booted = false;
	return ret;
}

static int exynos7885_abox_pcm_close(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	int ret = 0;

	if (READ_ONCE(st->substream)) {
		WRITE_ONCE(st->substream, NULL);
		exynos7885_abox_pcm_put_ipc(pcm);
	}
	/* A late IPC may queue the work again; it then finds no substream. */
	cancel_work_sync(&st->period_work);
	if (READ_ONCE(st->running))
		return -EBUSY;
	if (st->configured) {
		ret = exynos7885_abox_pcm_hw_free(component, substream);
		if (ret)
			return ret;
	}
	if (st->opened) {
		ret = exynos7885_abox_pcm_send(pcm, st,
					       EXYNOS7885_ABOX_PCM_CLOSE,
					       0, 0, 0);
		if (ret)
			return ret;
		st->opened = false;
	}
	if (st->mapped) {
		if (st->capture)
			ret = exynos7885_abox_unmap_capture_buffer(pcm->abox_dev);
		else
			ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
		if (ret)
			return ret;
		st->mapped = false;
	}
	if (st->booted) {
		ret = exynos7885_abox_shutdown(pcm->abox_dev);
		if (!ret)
			st->booted = false;
	}
	return ret;
}

static int exynos7885_abox_pcm_copy(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    int channel, unsigned long pos,
				    struct iov_iter *iter, unsigned long bytes)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t ring_bytes = runtime->dma_bytes;
	size_t first, second;

	if (channel || !ring_bytes || pos >= ring_bytes || bytes > ring_bytes)
		return -EINVAL;

	first = min_t(size_t, bytes, ring_bytes - pos);
	second = bytes - first;

	if (st->capture) {
		/* Read the samples only after the position that covers them. */
		rmb();
		if (copy_to_iter((u8 *)st->area + pos, first, iter) != first)
			return -EFAULT;
		if (second && copy_to_iter(st->area, second, iter) != second)
			return -EFAULT;
		return 0;
	}

	if (!copy_from_iter_full((u8 *)st->area + pos, first, iter))
		return -EFAULT;
	if (second && !copy_from_iter_full(st->area, second, iter))
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
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t appl_ptr = READ_ONCE(runtime->control->appl_ptr);
	snd_pcm_uframes_t frames;
	size_t offset, bytes, first;
	int ret;

	/* WDMA1 overwrites its ring freely; only playback data needs a sync. */
	if (st->capture || !st->mapped || !runtime->buffer_size)
		return 0;
	frames = appl_ptr - st->appl_ptr;
	if (frames > runtime->buffer_size)
		frames = runtime->buffer_size;
	st->appl_ptr = appl_ptr;
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
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 iova = st->capture ? ABOX_PCM_CAPTURE_IOVA : ABOX_PCM_PLAYBACK_IOVA;
	int ret, cleanup_ret;

	if (params_buffer_bytes(params) > st->size ||
	    params_periods(params) < ABOX_PCM_PERIODS_MIN)
		return -EINVAL;
	/* A second hw_params replaces the first configuration. */
	ret = exynos7885_abox_pcm_hw_free(component, substream);
	if (ret)
		return ret;

	if (st->capture)
		ret = exynos7885_abox_map_capture_buffer(pcm->abox_dev);
	else
		ret = exynos7885_abox_map_pcm_buffer(pcm->abox_dev);
	if (ret)
		return ret;
	st->mapped = true;

	memset(st->area, 0, st->size);
	runtime->dma_area = st->area;
	runtime->dma_addr = iova;
	runtime->dma_bytes = params_buffer_bytes(params);
	st->dma_bytes = runtime->dma_bytes;
	st->period_bytes = params_period_bytes(params);
	st->frame_bytes = params_channels(params) *
			  params_physical_width(params) / 8;

	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_SET_BUFFER,
				       iova, params_period_bytes(params),
				       params_periods(params));
	if (ret)
		goto free_fw;

	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_HW_PARAMS,
				       params_rate(params),
				       params_width(params),
				       params_channels(params));
	if (ret)
		goto free_fw;

	dev_dbg(component->dev,
		"%s buffer %zu bytes, period %u bytes x %u, rate %u, width %u, channels %u\n",
		st->capture ? "capture" : "playback",
		runtime->dma_bytes, params_period_bytes(params),
		params_periods(params), params_rate(params),
		params_width(params), params_channels(params));
	WRITE_ONCE(st->configured, true);
	st->hw_ptr = 0;
	st->appl_ptr = 0;
	return 0;

free_fw:
	cleanup_ret = exynos7885_abox_pcm_send(pcm, st,
					       EXYNOS7885_ABOX_PCM_HW_FREE,
					       0, 0, 0);
	if (cleanup_ret)
		return ret;
	if (st->capture)
		cleanup_ret = exynos7885_abox_unmap_capture_buffer(pcm->abox_dev);
	else
		cleanup_ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
	if (cleanup_ret)
		return cleanup_ret;
	st->mapped = false;
	runtime->dma_area = NULL;
	runtime->dma_bytes = 0;
	return ret;
}

static int exynos7885_abox_pcm_hw_free(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	int ret;

	if (!st->configured)
		return 0;
	if (READ_ONCE(st->running))
		return -EBUSY;

	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_HW_FREE,
				       0, 0, 0);
	if (ret)
		return ret;
	if (st->capture)
		ret = exynos7885_abox_unmap_capture_buffer(pcm->abox_dev);
	else
		ret = exynos7885_abox_unmap_pcm_buffer(pcm->abox_dev);
	if (ret)
		return ret;

	st->mapped = false;
	WRITE_ONCE(st->configured, false);
	return 0;
}

static int exynos7885_abox_pcm_prepare(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
	int ret;

	if (!st->configured)
		return -EINVAL;
	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_PREPARE,
				       0, 0, 0);
	if (ret)
		return ret;
	st->hw_ptr = 0;
	st->appl_ptr = READ_ONCE(substream->runtime->control->appl_ptr);
	return 0;
}

/* Each UAIF's number, by its CPU DAI's id. */
static const unsigned int exynos7885_abox_pcm_uaif_id[ABOX_PCM_NUM_DAIS] = {
	3, 0, 0,
};

/*
 * Start only the selected output, or stop every running UAIF. Two UAIFs
 * reading SIFS1 each take part of the samples and RDMA0 runs twice as fast,
 * so the speakers and the headphones cannot play together.
 */
static int exynos7885_abox_pcm_set_uaifs(struct exynos7885_abox_pcm *pcm,
					 bool enable)
{
	struct exynos7885_abox_pcm_uaif *uaif;
	unsigned int output = pcm->output;
	int i, ret, first_err = 0;

	lockdep_assert_held(&pcm->lock);

	/* Without a configured headphone DAI, play on what is there. */
	if (!pcm->uaif[output].configured)
		output = 0;

	for (i = 0; i < ABOX_PCM_NUM_PLAYBACK_DAIS; i++) {
		uaif = &pcm->uaif[i];
		if (!uaif->configured || uaif->enabled == enable)
			continue;
		if (enable && i != output)
			continue;
		ret = exynos7885_abox_uaif_set_enabled(pcm->abox_dev,
					exynos7885_abox_pcm_uaif_id[i], enable);
		if (!ret) {
			uaif->enabled = enable;
			continue;
		}
		if (!enable) {
			first_err = first_err ?: ret;
			continue;
		}
		/* Undo a partial start. */
		while (--i >= 0)
			if (pcm->uaif[i].enabled &&
			    !exynos7885_abox_uaif_set_enabled(pcm->abox_dev,
					exynos7885_abox_pcm_uaif_id[i], false))
				pcm->uaif[i].enabled = false;
		return ret;
	}
	return first_err;
}

static int exynos7885_abox_pcm_set_mic(struct exynos7885_abox_pcm *pcm,
				       bool enable)
{
	struct exynos7885_abox_pcm_uaif *mic = &pcm->uaif[ABOX_PCM_MIC_DAI];
	int ret;

	lockdep_assert_held(&pcm->lock);

	if (mic->enabled == enable)
		return 0;
	if (enable && !mic->configured)
		return -EINVAL;
	ret = exynos7885_abox_uaif_set_mic_enabled(pcm->abox_dev,
			exynos7885_abox_pcm_uaif_id[ABOX_PCM_MIC_DAI], enable);
	if (!ret)
		mic->enabled = enable;
	return ret;
}

/*
 * The opposite order to playback, as the vendor capture links trigger
 * WDMA before the UAIF on start and after it on stop.
 */
static int exynos7885_abox_pcm_capture_trigger(struct exynos7885_abox_pcm *pcm,
					       struct exynos7885_abox_pcm_stream *st,
					       bool start)
{
	int ret, stop_ret;

	lockdep_assert_held(&pcm->lock);

	if (start) {
		WRITE_ONCE(st->running, true);
		ret = exynos7885_abox_pcm_send(pcm, st,
					       EXYNOS7885_ABOX_PCM_TRIGGER,
					       1, 0, 0);
		if (ret) {
			WRITE_ONCE(st->running, false);
			return ret;
		}
		ret = exynos7885_abox_pcm_set_mic(pcm, true);
		if (!ret)
			return 0;
	} else {
		ret = exynos7885_abox_pcm_set_mic(pcm, false);
	}

	/* Stop WDMA1 even if the input failed to stop. */
	stop_ret = exynos7885_abox_pcm_send(pcm, st,
					    EXYNOS7885_ABOX_PCM_TRIGGER,
					    0, 0, 0);
	if (!stop_ret)
		stop_ret = exynos7885_abox_wait_wdma_idle(pcm->abox_dev, st->dma,
							  20000);
	if (!stop_ret)
		WRITE_ONCE(st->running, false);
	return ret ?: stop_ret;
}

static int exynos7885_abox_pcm_trigger(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream,
				       int cmd)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	struct exynos7885_abox_pcm_stream *st =
		exynos7885_abox_pcm_stream(pcm, substream);
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

	guard(mutex)(&pcm->lock);
	if (st->capture)
		return exynos7885_abox_pcm_capture_trigger(pcm, st, start);

	/* The UAIF runs before RDMA0 starts and stops after it is idle. */
	if (start) {
		ret = exynos7885_abox_pcm_set_uaifs(pcm, true);
		if (ret)
			return ret;
		WRITE_ONCE(st->running, true);
	}
	ret = exynos7885_abox_pcm_send(pcm, st, EXYNOS7885_ABOX_PCM_TRIGGER,
				       start, 0, 0);
	if (ret) {
		if (start && !exynos7885_abox_pcm_set_uaifs(pcm, false))
			WRITE_ONCE(st->running, false);
		return ret;
	}
	if (start)
		return 0;
	ret = exynos7885_abox_wait_rdma_idle(pcm->abox_dev, st->dma, 20000);
	if (!ret)
		ret = exynos7885_abox_pcm_set_uaifs(pcm, false);
	if (!ret)
		WRITE_ONCE(st->running, false);
	return ret;
}

static snd_pcm_uframes_t
exynos7885_abox_pcm_pointer(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);

	return READ_ONCE(exynos7885_abox_pcm_stream(pcm, substream)->hw_ptr);
}

static const char * const exynos7885_abox_output_texts[] = {
	"Speaker", "Headphone",
};

static SOC_ENUM_SINGLE_EXT_DECL(exynos7885_abox_output_enum,
				exynos7885_abox_output_texts);

static int exynos7885_abox_output_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = pcm->output;
	return 0;
}

static int exynos7885_abox_output_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct exynos7885_abox_pcm *pcm = snd_soc_component_get_drvdata(component);
	unsigned int output = ucontrol->value.enumerated.item[0];
	int i, ret;

	if (output >= ARRAY_SIZE(exynos7885_abox_output_texts))
		return -EINVAL;

	guard(mutex)(&pcm->lock);
	if (output == pcm->output)
		return 0;
	/* Move a running stream now; otherwise it applies at the next start. */
	for (i = 0; i < ABOX_PCM_NUM_PLAYBACK_DAIS; i++) {
		if (!pcm->uaif[i].enabled || i == output ||
		    !pcm->uaif[output].configured)
			continue;
		ret = exynos7885_abox_uaif_switch(pcm->abox_dev,
					exynos7885_abox_pcm_uaif_id[i],
					exynos7885_abox_pcm_uaif_id[output]);
		if (ret)
			return ret;
		pcm->uaif[i].enabled = false;
		pcm->uaif[output].enabled = true;
		break;
	}
	pcm->output = output;
	return 1;
}

static const struct snd_kcontrol_new exynos7885_abox_controls[] = {
	SOC_ENUM_EXT("Playback Output", exynos7885_abox_output_enum,
		     exynos7885_abox_output_get, exynos7885_abox_output_put),
};

static const struct snd_soc_component_driver exynos7885_abox_component = {
	.name = "exynos7885-abox-pcm",
	.controls = exynos7885_abox_controls,
	.num_controls = ARRAY_SIZE(exynos7885_abox_controls),
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

static struct exynos7885_abox_pcm_uaif *
exynos7885_abox_dai_uaif(struct snd_soc_dai *dai)
{
	struct exynos7885_abox_pcm *pcm =
		snd_soc_component_get_drvdata(dai->component);

	return &pcm->uaif[dai->id];
}

static int exynos7885_abox_dai_set_fmt(struct snd_soc_dai *dai,
				       unsigned int fmt)
{
	struct exynos7885_abox_pcm_uaif *uaif = exynos7885_abox_dai_uaif(dai);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		uaif->format = EXYNOS7885_ABOX_UAIF3_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		uaif->format = EXYNOS7885_ABOX_UAIF3_DSP_A;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		uaif->invert_bclk = false;
		uaif->invert_frame = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		uaif->invert_bclk = false;
		uaif->invert_frame = true;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		uaif->invert_bclk = true;
		uaif->invert_frame = false;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		uaif->invert_bclk = true;
		uaif->invert_frame = true;
		break;
	default:
		return -EINVAL;
	}

	/* ASoC flips the link format, so BP_FP means the UAIF drives BCLK/FS. */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		uaif->abox_master = true;
		break;
	default:
		return -EOPNOTSUPP;
	}

	uaif->fmt_valid = true;
	return 0;
}

static int exynos7885_abox_dai_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	struct exynos7885_abox_pcm *pcm =
		snd_soc_component_get_drvdata(dai->component);
	struct exynos7885_abox_pcm_uaif *uaif = exynos7885_abox_dai_uaif(dai);
	unsigned int n = exynos7885_abox_pcm_uaif_id[dai->id];
	int ret;

	if (!uaif->fmt_valid)
		return -EINVAL;

	/* With the other direction of this UAIF running, the format must match. */
	ret = exynos7885_abox_uaif_set_fmt(pcm->abox_dev, n, uaif->format,
					   uaif->invert_bclk,
					   uaif->invert_frame,
					   uaif->abox_master);
	if (ret)
		return ret;

	if (dai->id == ABOX_PCM_MIC_DAI)
		ret = exynos7885_abox_uaif_mic_hw_params(pcm->abox_dev, n,
							 params_rate(params),
							 params_width(params),
							 params_channels(params));
	else
		ret = exynos7885_abox_uaif_hw_params(pcm->abox_dev, n,
						     params_rate(params),
						     params_width(params),
						     params_channels(params));
	if (ret)
		return ret;
	scoped_guard(mutex, &pcm->lock)
		uaif->configured = true;
	return 0;
}

static int exynos7885_abox_dai_hw_free(struct snd_pcm_substream *substream,
				       struct snd_soc_dai *dai)
{
	struct exynos7885_abox_pcm *pcm =
		snd_soc_component_get_drvdata(dai->component);
	struct exynos7885_abox_pcm_uaif *uaif = exynos7885_abox_dai_uaif(dai);

	guard(mutex)(&pcm->lock);
	if (uaif->enabled)
		return -EBUSY;
	uaif->configured = false;
	return 0;
}

static const struct snd_soc_dai_ops exynos7885_abox_dai_ops = {
	.set_fmt = exynos7885_abox_dai_set_fmt,
	.hw_params = exynos7885_abox_dai_hw_params,
	.hw_free = exynos7885_abox_dai_hw_free,
};

#define EXYNOS7885_ABOX_UAIF_STREAM(_stream)				\
	{								\
		.stream_name = _stream,					\
		.channels_min = 2,					\
		.channels_max = 2,					\
		.rates = SNDRV_PCM_RATE_8000_192000,			\
		.formats = SNDRV_PCM_FMTBIT_S16_LE |			\
			   SNDRV_PCM_FMTBIT_S24_LE |			\
			   SNDRV_PCM_FMTBIT_S32_LE,			\
	}

static struct snd_soc_dai_driver exynos7885_abox_dais[ABOX_PCM_NUM_DAIS] = {
	{
		.id = 0,
		.name = "exynos7885-abox-uaif3",
		.playback = EXYNOS7885_ABOX_UAIF_STREAM("UAIF3 Playback"),
		.ops = &exynos7885_abox_dai_ops,
	},
	{
		.id = 1,
		.name = "exynos7885-abox-uaif0",
		.playback = EXYNOS7885_ABOX_UAIF_STREAM("UAIF0 Playback"),
		.ops = &exynos7885_abox_dai_ops,
	},
	{
		.id = ABOX_PCM_MIC_DAI,
		.name = "exynos7885-abox-uaif0-mic",
		.capture = EXYNOS7885_ABOX_UAIF_STREAM("UAIF0 Capture"),
		.ops = &exynos7885_abox_dai_ops,
	},
};

static int exynos7885_abox_pcm_probe(struct platform_device *pdev)
{
	struct exynos7885_abox_pcm *pcm;
	int i, ret;

	if (!pdev->dev.parent)
		return -ENODEV;

	pcm = devm_kzalloc(&pdev->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;
	ret = devm_mutex_init(&pdev->dev, &pcm->lock);
	if (ret)
		return ret;
	pcm->abox_dev = pdev->dev.parent;
	for (i = 0; i < ARRAY_SIZE(pcm->stream); i++) {
		pcm->stream[i].capture = i == SNDRV_PCM_STREAM_CAPTURE;
		pcm->stream[i].dma = pcm->stream[i].capture ?
				     ABOX_PCM_CAPTURE_DMA :
				     ABOX_PCM_PLAYBACK_DMA;
		INIT_WORK(&pcm->stream[i].period_work,
			  exynos7885_abox_pcm_period_work);
	}
	platform_set_drvdata(pdev, pcm);

	return devm_snd_soc_register_component(&pdev->dev,
					       &exynos7885_abox_component,
					       exynos7885_abox_dais,
					       ARRAY_SIZE(exynos7885_abox_dais));
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

MODULE_DESCRIPTION("Exynos7885 ABOX RDMA0 playback and WDMA1 capture PCM");
MODULE_LICENSE("GPL");
