// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 MIPI CSI-2 receiver (CSIS v5) with its built-in DMA.
 *
 * Each CSIS instance receives one sensor over a dedicated D-PHY and writes
 * raw Bayer frames to memory through the DMA of virtual channel 0, without
 * the FIMC-IS ISP. RAW10 is stored unpacked, 16 bits per pixel.
 *
 * Register layout and sequences follow Samsung's fimc-is2 CSIS v5.0 code.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#define CSIS_CMN_CTRL			0x0004
#define  CMN_CTRL_PHY_SEL		BIT(21)
#define  CMN_CTRL_UPDATE_SHADOW		GENMASK(19, 16)
#define  CMN_CTRL_DESKEW_ENABLE		BIT(12)
#define  CMN_CTRL_INTERLEAVE_MODE	GENMASK(11, 10)
#define  CMN_CTRL_LANE_NUMBER		GENMASK(9, 8)
#define  CMN_CTRL_SW_RESET		BIT(1)
#define  CMN_CTRL_CSI_EN		BIT(0)
#define CSIS_INT_MSK0			0x0010
#define CSIS_INT_SRC0			0x0014
#define  INT0_ERR_MASK			0x000fff1f
#define CSIS_INT_MSK1			0x0018
#define CSIS_INT_SRC1			0x001c
#define CSIS_ISP_CONFIG_CH0		0x0040
#define  ISP_CONFIG_PIXEL_MODE		GENMASK(13, 12)
#define  ISP_CONFIG_PARALLEL		BIT(11)
#define  ISP_CONFIG_DATAFORMAT		GENMASK(7, 2)
#define  ISP_CONFIG_VC			GENMASK(1, 0)
#define CSIS_ISP_RESOL_CH0		0x0044
#define CSIS_FRM_CNT_CH0		0x0100
#define CSIS_PHY_STATUS			0x0700
#define CSIS_PHY_CMN_CTRL		0x0704
#define  PHY_CMN_HSSETTLE		GENMASK(31, 24)
#define  PHY_CMN_S_CLKSETTLECTL		GENMASK(23, 16)
#define  PHY_CMN_S_BYTE_CLK_ENABLE	BIT(7)
#define  PHY_CMN_ENABLE_DAT		GENMASK(4, 1)
#define  PHY_CMN_ENABLE_CLK		BIT(0)
#define CSIS_DMA0_CTRL			0x1000
/* On Exynos7885 the DMA_N_DISABLE bit enables the DMA when set. */
#define  DMA_CTRL_ENABLE		BIT(0)
#define  DMA_CTRL_UPDT_PTR_EN		BIT(1)
#define CSIS_DMA0_FMT			0x1004
#define CSIS_DMA0_ADDR(n)		(0x1010 + 4 * (n))
#define CSIS_DMA0_ACT_CTRL		0x1030
#define CSIS_DMA0_RESOL			0x1040
#define CSIS_DMA_DATA_CTRL		0x1400
#define  DMA_DATA_INPUT_PRL		BIT(0)
#define CSIS_DMA_INT_MASK		0x1404
#define CSIS_DMA_INT_SRC		0x1408
#define  DMA_INT_ABORT_DONE		BIT(16)
#define  DMA_INT_ERROR			BIT(12)
#define  DMA_INT_FRM_END0		BIT(8)
#define  DMA_INT_FRM_START0		BIT(4)
#define CSIS_DMA_CMN_CTRL		0x1414
#define  DMA_CMN_ABORT_REQ		BIT(0)
#define CSIS_DMA_ERR_CODE		0x1418
#define CSIS_OTF_FORMAT			0x141c

#define CSIS_INT_MSK0_ON		0x011fff1f
#define CSIS_DMA_INT_MSK_ON		0x00f11ff0
#define CSIS_DT_RAW10			0x2b
#define CSIS_DMA_SLOTS			8

/* Common CSIS DMA block, shared by all CSIS instances */
#define CSIS_DMA_ARB_PRI		0x00
#define CSIS_DMA_CTRL			0x08
#define  CSIS_DMA_IP_PROCESSING		BIT(1)
#define CSIS_DMA_SRAM_SPLIT		0x0c
#define CSIS_DMA_MATRIX			0x10
/* Each of the two SRAMs holds 640 units of 16 bytes. */
#define CSIS_DMA_SRAM_UNITS		640
#define CSIS_DMA_SRAM_MIN		4

#define CSIS_MAX_WIDTH			8192
#define CSIS_MAX_HEIGHT			16383
#define CSIS_MIN_SIZE			16

#define CSIS_DRIVER_NAME		"exynos7885-csis"

struct csis_format {
	u32 fourcc;
	u32 code;
};

static const struct csis_format csis_formats[] = {
	{ V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10 },
};

struct csis_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct csis {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct regmap *pmu;
	struct regmap *sysreg;
	struct regmap *dma;
	u32 iso_offset;
	u32 rst_offset;
	u32 rst_bit;
	u32 dma_index;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct media_pad vpad;
	struct media_pipeline pipe;
	struct v4l2_async_notifier notifier;
	struct v4l2_subdev *sensor;
	struct media_pad *sensor_pad;
	unsigned int lanes;

	/* Serializes the video device and the queue. */
	struct mutex lock;
	struct vb2_queue queue;
	struct v4l2_pix_format pix;
	const struct csis_format *fmt;

	/* Protects the buffer pointers below, shared with the interrupt. */
	spinlock_t slock;
	struct list_head queued;
	struct csis_buffer *cur;	/* written by the frame in progress */
	struct csis_buffer *next;	/* programmed for the next frame */
	bool in_frame;
	unsigned int sequence;

	void *scratch;
	dma_addr_t scratch_addr;
	size_t scratch_size;
};

/* The common DMA SRAM is split between whichever instances are streaming. */
static DEFINE_MUTEX(csis_dma_lock);
static unsigned int csis_dma_active;

static inline struct csis_buffer *to_csis_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct csis_buffer, vb);
}

static void csis_rmw(struct csis *csis, u32 reg, u32 mask, u32 val)
{
	writel((readl(csis->base + reg) & ~mask) | (val & mask),
	       csis->base + reg);
}

static const struct csis_format *csis_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csis_formats); i++)
		if (csis_formats[i].fourcc == fourcc)
			return &csis_formats[i];
	return NULL;
}

static const struct csis_format *csis_find_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csis_formats); i++)
		if (csis_formats[i].code == code)
			return &csis_formats[i];
	return NULL;
}

static void csis_fill_pix(struct v4l2_pix_format *pix)
{
	const struct csis_format *fmt = csis_find_format(pix->pixelformat);

	if (!fmt)
		pix->pixelformat = csis_formats[0].fourcc;
	pix->width = clamp_t(u32, ALIGN(pix->width, 8), CSIS_MIN_SIZE,
			     CSIS_MAX_WIDTH);
	pix->height = clamp_t(u32, pix->height, CSIS_MIN_SIZE, CSIS_MAX_HEIGHT);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
}

static void csis_set_dma_addr(struct csis *csis, dma_addr_t addr)
{
	unsigned int i;

	/* The DMA cycles through all slots; keep them identical. */
	for (i = 0; i < CSIS_DMA_SLOTS; i++)
		writel(lower_32_bits(addr), csis->base + CSIS_DMA0_ADDR(i));
}

/* Program the SRAM split the way Samsung's driver does for the active set. */
static void csis_dma_common_update(struct csis *csis, bool on)
{
	unsigned int split0 = CSIS_DMA_SRAM_UNITS / 2;
	unsigned int split1 = CSIS_DMA_SRAM_UNITS / 2;
	unsigned int arb = 0, matrix = 0;

	mutex_lock(&csis_dma_lock);
	if (on)
		csis_dma_active |= BIT(csis->dma_index);
	else
		csis_dma_active &= ~BIT(csis->dma_index);

	switch (csis_dma_active) {
	case BIT(0):
		split0 = CSIS_DMA_SRAM_UNITS;
		arb = 1;
		break;
	case BIT(1):
		split0 = CSIS_DMA_SRAM_MIN;
		arb = 2;
		break;
	case BIT(0) | BIT(1):
		split0 = CSIS_DMA_SRAM_UNITS;
		split1 = CSIS_DMA_SRAM_UNITS;
		matrix = 2;
		break;
	default:
		break;
	}

	if (csis_dma_active) {
		regmap_update_bits(csis->dma, CSIS_DMA_CTRL,
				   CSIS_DMA_IP_PROCESSING,
				   CSIS_DMA_IP_PROCESSING);
		regmap_write(csis->dma, CSIS_DMA_ARB_PRI, arb);
		regmap_write(csis->dma, CSIS_DMA_SRAM_SPLIT,
			     split1 << 16 | split0);
		regmap_write(csis->dma, CSIS_DMA_MATRIX, matrix);
	}
	mutex_unlock(&csis_dma_lock);
}

static int csis_phy_power(struct csis *csis, bool on)
{
	int ret;

	if (on) {
		ret = regmap_update_bits(csis->pmu, csis->iso_offset, BIT(0),
					 BIT(0));
		if (ret)
			return ret;
		regmap_update_bits(csis->sysreg, csis->rst_offset,
				   BIT(csis->rst_bit), 0);
		return regmap_update_bits(csis->sysreg, csis->rst_offset,
					  BIT(csis->rst_bit),
					  BIT(csis->rst_bit));
	}

	regmap_update_bits(csis->sysreg, csis->rst_offset, BIT(csis->rst_bit),
			   0);
	return regmap_update_bits(csis->pmu, csis->iso_offset, BIT(0), 0);
}

/*
 * A stream stopped in the middle of a frame leaves the DMA busy: every later
 * frame start then only raises DMA_OTF_OVERLAP. Only an abort recovers it.
 */
static void csis_dma_abort(struct csis *csis)
{
	u32 src;

	writel(DMA_CMN_ABORT_REQ, csis->base + CSIS_DMA_CMN_CTRL);
	if (readl_poll_timeout(csis->base + CSIS_DMA_INT_SRC, src,
			       src & DMA_INT_ABORT_DONE, 100, 100000))
		dev_warn(csis->dev, "DMA abort timed out\n");
	writel(DMA_INT_ABORT_DONE, csis->base + CSIS_DMA_INT_SRC);
}

static void csis_hw_reset(struct csis *csis)
{
	unsigned int i;

	csis_rmw(csis, CSIS_CMN_CTRL, CMN_CTRL_SW_RESET, CMN_CTRL_SW_RESET);
	for (i = 0; i < 10; i++) {
		udelay(10);
		if (!(readl(csis->base + CSIS_CMN_CTRL) & CMN_CTRL_SW_RESET))
			break;
	}
	writel(0, csis->base + CSIS_DMA0_CTRL);
}

static void csis_hw_start(struct csis *csis, u32 settle)
{
	u32 width = csis->pix.width, height = csis->pix.height;

	csis_dma_abort(csis);
	csis_hw_reset(csis);

	csis_rmw(csis, CSIS_PHY_CMN_CTRL,
		 PHY_CMN_HSSETTLE | PHY_CMN_S_CLKSETTLECTL |
		 PHY_CMN_S_BYTE_CLK_ENABLE,
		 FIELD_PREP(PHY_CMN_HSSETTLE, settle) |
		 FIELD_PREP(PHY_CMN_S_CLKSETTLECTL, 1) |
		 PHY_CMN_S_BYTE_CLK_ENABLE);
	/* No deskew: Samsung enables it only above 1.5 Gbps per lane. */
	csis_rmw(csis, CSIS_CMN_CTRL,
		 CMN_CTRL_LANE_NUMBER | CMN_CTRL_DESKEW_ENABLE |
		 CMN_CTRL_INTERLEAVE_MODE,
		 FIELD_PREP(CMN_CTRL_LANE_NUMBER, csis->lanes - 1));
	csis_rmw(csis, CSIS_PHY_CMN_CTRL, PHY_CMN_ENABLE_DAT,
		 FIELD_PREP(PHY_CMN_ENABLE_DAT, GENMASK(csis->lanes - 1, 0)));

	csis_rmw(csis, CSIS_ISP_CONFIG_CH0,
		 ISP_CONFIG_PIXEL_MODE | ISP_CONFIG_PARALLEL |
		 ISP_CONFIG_DATAFORMAT | ISP_CONFIG_VC,
		 FIELD_PREP(ISP_CONFIG_PIXEL_MODE, 1) |
		 FIELD_PREP(ISP_CONFIG_DATAFORMAT, CSIS_DT_RAW10));
	writel(height << 16 | width, csis->base + CSIS_ISP_RESOL_CH0);
	writel(0, csis->base + CSIS_OTF_FORMAT);

	writel(CSIS_INT_MSK0_ON, csis->base + CSIS_INT_MSK0);
	writel(CSIS_DMA_INT_MSK_ON, csis->base + CSIS_DMA_INT_MASK);

	/* 2D, legacy storage, RAW10 unpacked */
	writel(0, csis->base + CSIS_DMA0_FMT);
	writel(width, csis->base + CSIS_DMA0_RESOL);
	csis_rmw(csis, CSIS_DMA_DATA_CTRL, DMA_DATA_INPUT_PRL,
		 DMA_DATA_INPUT_PRL);
	writel(DMA_CTRL_ENABLE | DMA_CTRL_UPDT_PTR_EN,
	       csis->base + CSIS_DMA0_CTRL);

	csis_rmw(csis, CSIS_CMN_CTRL, CMN_CTRL_UPDATE_SHADOW | CMN_CTRL_PHY_SEL,
		 CMN_CTRL_UPDATE_SHADOW);
	csis_rmw(csis, CSIS_PHY_CMN_CTRL, PHY_CMN_ENABLE_CLK,
		 PHY_CMN_ENABLE_CLK);
	csis_rmw(csis, CSIS_CMN_CTRL, CMN_CTRL_CSI_EN, CMN_CTRL_CSI_EN);
}

static void csis_hw_stop(struct csis *csis)
{
	writel(0, csis->base + CSIS_INT_MSK0);
	writel(0, csis->base + CSIS_DMA_INT_MASK);
	csis_dma_abort(csis);
	csis_rmw(csis, CSIS_PHY_CMN_CTRL, PHY_CMN_ENABLE_CLK |
		 PHY_CMN_ENABLE_DAT, 0);
	csis_rmw(csis, CSIS_CMN_CTRL, CMN_CTRL_CSI_EN, 0);
	csis_hw_reset(csis);
}

static void csis_complete(struct csis *csis, struct csis_buffer *buf,
			  enum vb2_buffer_state state)
{
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = csis->sequence;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, state);
}

static irqreturn_t csis_irq(int irq, void *data)
{
	struct csis *csis = data;
	u32 src0, src1, dsrc;

	src0 = readl(csis->base + CSIS_INT_SRC0);
	src1 = readl(csis->base + CSIS_INT_SRC1);
	dsrc = readl(csis->base + CSIS_DMA_INT_SRC);
	writel(src0, csis->base + CSIS_INT_SRC0);
	writel(src1, csis->base + CSIS_INT_SRC1);
	writel(dsrc, csis->base + CSIS_DMA_INT_SRC);

	if ((src0 & INT0_ERR_MASK) || (dsrc & DMA_INT_ERROR))
		dev_err_ratelimited(csis->dev,
				    "errors: src0 %#x src1 %#x dma %#x code %#x\n",
				    src0, src1, dsrc,
				    readl(csis->base + CSIS_DMA_ERR_CODE));

	spin_lock(&csis->slock);
	/* A pending end belongs to the frame before a pending start. */
	if ((dsrc & DMA_INT_FRM_END0) && csis->in_frame) {
		if (csis->cur)
			csis_complete(csis, csis->cur, VB2_BUF_STATE_DONE);
		csis->cur = NULL;
		csis->sequence++;
		csis->in_frame = false;
	}
	if (dsrc & DMA_INT_FRM_START0) {
		/* This frame latched the address programmed before it. */
		csis->cur = csis->next;
		csis->next = list_first_entry_or_null(&csis->queued,
						      struct csis_buffer, list);
		if (csis->next)
			list_del(&csis->next->list);
		csis_set_dma_addr(csis, csis->next ?
				  vb2_dma_contig_plane_dma_addr(&csis->next->vb.vb2_buf, 0) :
				  csis->scratch_addr);
		csis->in_frame = true;
	}
	spin_unlock(&csis->slock);

	return IRQ_HANDLED;
}

static int csis_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct csis *csis = vb2_get_drv_priv(vq);

	if (*nplanes)
		return sizes[0] < csis->pix.sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = csis->pix.sizeimage;
	return 0;
}

static int csis_buf_prepare(struct vb2_buffer *vb)
{
	struct csis *csis = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < csis->pix.sizeimage)
		return -EINVAL;
	/* The DMA address registers are 32 bits wide. */
	if (upper_32_bits(vb2_dma_contig_plane_dma_addr(vb, 0)))
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, csis->pix.sizeimage);
	return 0;
}

static void csis_buf_queue(struct vb2_buffer *vb)
{
	struct csis *csis = vb2_get_drv_priv(vb->vb2_queue);
	struct csis_buffer *buf = to_csis_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	list_add_tail(&buf->list, &csis->queued);
	spin_unlock_irqrestore(&csis->slock, flags);
}

static void csis_return_buffers(struct csis *csis, enum vb2_buffer_state state)
{
	struct csis_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	if (csis->cur)
		vb2_buffer_done(&csis->cur->vb.vb2_buf, state);
	if (csis->next && csis->next != csis->cur)
		vb2_buffer_done(&csis->next->vb.vb2_buf, state);
	csis->cur = NULL;
	csis->next = NULL;
	list_for_each_entry_safe(buf, tmp, &csis->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&csis->slock, flags);
}

static int csis_check_sensor_format(struct csis *csis)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = csis->sensor_pad->index,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(csis->sensor, pad, get_fmt, &fmt);
	if (ret)
		return ret;
	if (fmt.format.width != csis->pix.width ||
	    fmt.format.height != csis->pix.height ||
	    fmt.format.code != csis->fmt->code) {
		dev_dbg(csis->dev, "sensor %ux%u %#x, video %ux%u %#x\n",
			fmt.format.width, fmt.format.height, fmt.format.code,
			csis->pix.width, csis->pix.height, csis->fmt->code);
		return -EPIPE;
	}
	return 0;
}

static int csis_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct csis *csis = vb2_get_drv_priv(vq);
	unsigned long flags;
	s64 link_freq;
	u32 settle, lane_mbps;
	int ret;

	ret = video_device_pipeline_start(&csis->vdev, &csis->pipe);
	if (ret)
		goto err_return;
	ret = csis_check_sensor_format(csis);
	if (ret)
		goto err_pipeline;

	link_freq = v4l2_get_link_freq(csis->sensor_pad, 10, 2 * csis->lanes);
	if (link_freq < 0) {
		ret = link_freq;
		goto err_pipeline;
	}
	/* Samsung's 14 nm settle table is close to lane rate / 45.7 MHz. */
	lane_mbps = div_u64(link_freq * 2, 1000000);
	settle = max_t(u32, DIV_ROUND_CLOSEST(lane_mbps * 46, 2100), 2) - 1;

	if (csis->scratch_size < csis->pix.sizeimage) {
		if (csis->scratch)
			dma_free_coherent(csis->dev, csis->scratch_size,
					  csis->scratch, csis->scratch_addr);
		csis->scratch_size = csis->pix.sizeimage;
		csis->scratch = dma_alloc_coherent(csis->dev,
						   csis->scratch_size,
						   &csis->scratch_addr,
						   GFP_KERNEL);
		if (!csis->scratch) {
			csis->scratch_size = 0;
			ret = -ENOMEM;
			goto err_pipeline;
		}
	}

	spin_lock_irqsave(&csis->slock, flags);
	csis->sequence = 0;
	csis->in_frame = false;
	csis->cur = NULL;
	csis->next = list_first_entry_or_null(&csis->queued,
					      struct csis_buffer, list);
	if (csis->next)
		list_del(&csis->next->list);
	csis_set_dma_addr(csis, csis->next ?
			  vb2_dma_contig_plane_dma_addr(&csis->next->vb.vb2_buf, 0) :
			  csis->scratch_addr);
	spin_unlock_irqrestore(&csis->slock, flags);

	ret = csis_phy_power(csis, true);
	if (ret)
		goto err_pipeline;
	csis_dma_common_update(csis, true);
	csis_hw_start(csis, settle);
	enable_irq(csis->irq);
	dev_dbg(csis->dev, "%ux%u, %u lanes at %u Mbps, settle %u\n",
		csis->pix.width, csis->pix.height, csis->lanes, lane_mbps,
		settle);

	ret = v4l2_subdev_enable_streams(csis->sensor, csis->sensor_pad->index,
					 BIT_ULL(0));
	if (ret)
		goto err_hw;
	return 0;

err_hw:
	disable_irq(csis->irq);
	csis_hw_stop(csis);
	csis_dma_common_update(csis, false);
	csis_phy_power(csis, false);
err_pipeline:
	video_device_pipeline_stop(&csis->vdev);
err_return:
	csis_return_buffers(csis, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void csis_stop_streaming(struct vb2_queue *vq)
{
	struct csis *csis = vb2_get_drv_priv(vq);

	v4l2_subdev_disable_streams(csis->sensor, csis->sensor_pad->index,
				    BIT_ULL(0));
	disable_irq(csis->irq);
	csis_hw_stop(csis);
	csis_dma_common_update(csis, false);
	csis_phy_power(csis, false);
	video_device_pipeline_stop(&csis->vdev);
	csis_return_buffers(csis, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops csis_vb2_ops = {
	.queue_setup = csis_queue_setup,
	.buf_prepare = csis_buf_prepare,
	.buf_queue = csis_buf_queue,
	.start_streaming = csis_start_streaming,
	.stop_streaming = csis_stop_streaming,
};

static int csis_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	struct csis *csis = video_drvdata(file);

	strscpy(cap->driver, CSIS_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, csis->mdev.model, sizeof(cap->card));
	return 0;
}

static int csis_enum_fmt(struct file *file, void *priv,
			 struct v4l2_fmtdesc *f)
{
	const struct csis_format *fmt;

	if (f->mbus_code) {
		if (f->index)
			return -EINVAL;
		fmt = csis_find_code(f->mbus_code);
		if (!fmt)
			return -EINVAL;
	} else {
		if (f->index >= ARRAY_SIZE(csis_formats))
			return -EINVAL;
		fmt = &csis_formats[f->index];
	}
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int csis_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct csis *csis = video_drvdata(file);

	f->fmt.pix = csis->pix;
	return 0;
}

static int csis_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	csis_fill_pix(&f->fmt.pix);
	return 0;
}

static int csis_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct csis *csis = video_drvdata(file);

	if (vb2_is_busy(&csis->queue))
		return -EBUSY;
	csis_fill_pix(&f->fmt.pix);
	csis->pix = f->fmt.pix;
	csis->fmt = csis_find_format(csis->pix.pixelformat);
	return 0;
}

static int csis_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !csis_find_format(fsize->pixel_format))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = CSIS_MIN_SIZE;
	fsize->stepwise.max_width = CSIS_MAX_WIDTH;
	fsize->stepwise.step_width = 8;
	fsize->stepwise.min_height = CSIS_MIN_SIZE;
	fsize->stepwise.max_height = CSIS_MAX_HEIGHT;
	fsize->stepwise.step_height = 1;
	return 0;
}

static const struct v4l2_ioctl_ops csis_ioctl_ops = {
	.vidioc_querycap = csis_querycap,
	.vidioc_enum_fmt_vid_cap = csis_enum_fmt,
	.vidioc_g_fmt_vid_cap = csis_g_fmt,
	.vidioc_try_fmt_vid_cap = csis_try_fmt,
	.vidioc_s_fmt_vid_cap = csis_s_fmt,
	.vidioc_enum_framesizes = csis_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations csis_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int csis_link_validate(struct media_link *link)
{
	/* Formats are checked against the sensor when streaming starts. */
	return 0;
}

static const struct media_entity_operations csis_entity_ops = {
	.link_validate = csis_link_validate,
};

static int csis_notify_bound(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *sd,
			     struct v4l2_async_connection *asc)
{
	struct csis *csis = container_of(notifier, struct csis, notifier);
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;
	csis->sensor = sd;
	csis->sensor_pad = &sd->entity.pads[pad];
	return media_create_pad_link(&sd->entity, pad, &csis->vdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static int csis_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct csis *csis = container_of(notifier, struct csis, notifier);

	return v4l2_device_register_subdev_nodes(&csis->v4l2_dev);
}

static const struct v4l2_async_notifier_operations csis_notify_ops = {
	.bound = csis_notify_bound,
	.complete = csis_notify_complete,
};

static int csis_parse_endpoint(struct csis *csis)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_async_connection *asc;
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(csis->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(csis->dev, -ENODEV, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret) {
		dev_err_probe(csis->dev, ret, "invalid endpoint\n");
		goto out;
	}
	csis->lanes = vep.bus.mipi_csi2.num_data_lanes;
	if (csis->lanes < 1 || csis->lanes > 4) {
		ret = dev_err_probe(csis->dev, -EINVAL, "invalid data-lanes\n");
		goto out;
	}

	asc = v4l2_async_nf_add_fwnode_remote(&csis->notifier, ep,
					      struct v4l2_async_connection);
	ret = PTR_ERR_OR_ZERO(asc);
out:
	fwnode_handle_put(ep);
	return ret;
}

static int csis_get_syscons(struct csis *csis)
{
	struct device_node *np = csis->dev->of_node;
	unsigned int args[2];

	csis->pmu = syscon_regmap_lookup_by_phandle_args(np, "samsung,pmu-syscon",
							 1, args);
	if (IS_ERR(csis->pmu))
		return dev_err_probe(csis->dev, PTR_ERR(csis->pmu),
				     "no PHY isolation register\n");
	csis->iso_offset = args[0];

	csis->sysreg = syscon_regmap_lookup_by_phandle_args(np, "samsung,sysreg",
							    2, args);
	if (IS_ERR(csis->sysreg))
		return dev_err_probe(csis->dev, PTR_ERR(csis->sysreg),
				     "no PHY reset register\n");
	csis->rst_offset = args[0];
	csis->rst_bit = args[1];

	csis->dma = syscon_regmap_lookup_by_phandle_args(np, "samsung,csis-dma",
							 1, args);
	if (IS_ERR(csis->dma))
		return dev_err_probe(csis->dev, PTR_ERR(csis->dma),
				     "no common CSIS DMA\n");
	csis->dma_index = args[0];
	if (csis->dma_index > 1)
		return dev_err_probe(csis->dev, -EINVAL,
				     "unsupported CSIS DMA %u\n", csis->dma_index);
	return 0;
}

static int csis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct csis *csis;
	struct vb2_queue *q;
	int ret;

	csis = devm_kzalloc(dev, sizeof(*csis), GFP_KERNEL);
	if (!csis)
		return -ENOMEM;
	csis->dev = dev;
	mutex_init(&csis->lock);
	spin_lock_init(&csis->slock);
	INIT_LIST_HEAD(&csis->queued);
	platform_set_drvdata(pdev, csis);

	csis->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(csis->base))
		return PTR_ERR(csis->base);
	ret = csis_get_syscons(csis);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	csis->irq = platform_get_irq(pdev, 0);
	if (csis->irq < 0)
		return csis->irq;
	ret = devm_request_irq(dev, csis->irq, csis_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), csis);
	if (ret)
		return ret;

	/* The receiver idles in reset with its D-PHY isolated. */
	csis_hw_reset(csis);

	csis->pix.width = 640;
	csis->pix.height = 480;
	csis->pix.pixelformat = csis_formats[0].fourcc;
	csis_fill_pix(&csis->pix);
	csis->fmt = &csis_formats[0];

	csis->mdev.dev = dev;
	strscpy(csis->mdev.driver_name, CSIS_DRIVER_NAME,
		sizeof(csis->mdev.driver_name));
	snprintf(csis->mdev.model, sizeof(csis->mdev.model),
		 "Exynos7885 CSIS%u", csis->dma_index);
	media_device_init(&csis->mdev);

	csis->v4l2_dev.mdev = &csis->mdev;
	ret = v4l2_device_register(dev, &csis->v4l2_dev);
	if (ret)
		goto err_media;

	q = &csis->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = csis;
	q->buf_struct_size = sizeof(struct csis_buffer);
	q->ops = &csis_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &csis->lock;
	q->dev = dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_v4l2;

	vdev = &csis->vdev;
	snprintf(vdev->name, sizeof(vdev->name), "%s", csis->mdev.model);
	vdev->fops = &csis_fops;
	vdev->ioctl_ops = &csis_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->v4l2_dev = &csis->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &csis->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE | V4L2_CAP_IO_MC;
	vdev->entity.ops = &csis_entity_ops;
	video_set_drvdata(vdev, csis);
	csis->vpad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &csis->vpad);
	if (ret)
		goto err_v4l2;

	/* Links to the sensor need the video entity in the media device. */
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	v4l2_async_nf_init(&csis->notifier, &csis->v4l2_dev);
	ret = csis_parse_endpoint(csis);
	if (ret)
		goto err_notifier;
	csis->notifier.ops = &csis_notify_ops;
	ret = v4l2_async_nf_register(&csis->notifier);
	if (ret)
		goto err_notifier;

	ret = media_device_register(&csis->mdev);
	if (ret)
		goto err_unregister_nf;
	return 0;

err_unregister_nf:
	v4l2_async_nf_unregister(&csis->notifier);
err_notifier:
	v4l2_async_nf_cleanup(&csis->notifier);
	vb2_video_unregister_device(vdev);
err_entity:
	media_entity_cleanup(&vdev->entity);
err_v4l2:
	v4l2_device_unregister(&csis->v4l2_dev);
err_media:
	media_device_cleanup(&csis->mdev);
	return ret;
}

static void csis_remove(struct platform_device *pdev)
{
	struct csis *csis = platform_get_drvdata(pdev);

	media_device_unregister(&csis->mdev);
	v4l2_async_nf_unregister(&csis->notifier);
	v4l2_async_nf_cleanup(&csis->notifier);
	vb2_video_unregister_device(&csis->vdev);
	media_entity_cleanup(&csis->vdev.entity);
	v4l2_device_unregister(&csis->v4l2_dev);
	media_device_cleanup(&csis->mdev);
	if (csis->scratch)
		dma_free_coherent(csis->dev, csis->scratch_size, csis->scratch,
				  csis->scratch_addr);
	mutex_destroy(&csis->lock);
}

static const struct of_device_id csis_of_match[] = {
	{ .compatible = "samsung,exynos7885-csis" },
	{ }
};
MODULE_DEVICE_TABLE(of, csis_of_match);

static struct platform_driver csis_driver = {
	.probe = csis_probe,
	.remove = csis_remove,
	.driver = {
		.name = CSIS_DRIVER_NAME,
		.of_match_table = csis_of_match,
	},
};
module_platform_driver(csis_driver);

MODULE_DESCRIPTION("Samsung Exynos7885 MIPI CSI-2 receiver with DMA");
MODULE_LICENSE("GPL");
