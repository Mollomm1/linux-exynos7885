// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 display processing unit (DECON0 + DPP IDMA_G0).
 *
 * The bootloader leaves DECON0, the DSI host and the panel running in video
 * mode, scanning out its framebuffer through window 3 and IDMA_G0. This
 * driver takes that pipeline over: it moves the IDMA_G0 fetch address to
 * DRM framebuffers and latches each change at frame start through the DECON
 * shadow registers, so page flips are tear-free. The DSI host and the panel
 * are reached through the DRM bridge chain; clocks and power are left as the
 * bootloader set them.
 */

#include <linux/aperture.h>
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_plane.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

/* DECON registers */
#define DECON_GLOBAL_CONTROL		0x0000
#define DECON_GLOBAL_RUN_STATUS		BIT(4)
#define DECON_INTERRUPT_ENABLE		0x0040
#define DECON_FRAME_START_INT_EN	BIT(12)
#define DECON_INT_EN			BIT(0)
#define DECON_EXTRA_INTERRUPT_ENABLE	0x0044
#define DECON_INTERRUPT_PENDING		0x004c
#define DECON_FRAME_START_INT_PEND	BIT(12)
#define DECON_EXTRA_INTERRUPT_PENDING	0x0050
#define DECON_SHADOW_REG_UPDATE_REQ	0x0060
#define DECON_UPDATE_REQ_GLOBAL		BIT(31)
#define DECON_UPDATE_REQ_WIN(w)		BIT(w)
#define DECON_DATA_PATH_CONTROL_0	0x0214
#define DECON_WIN_EN(w)			BIT(4 * (w))
#define DECON_DATA_PATH_CONTROL_1	0x0218
#define DECON_WIN_CHMAP(w)		(0x3 << (4 * (w)))

/* IDMA registers */
#define IDMA_SRC_SIZE			0x0010
#define IDMA_SRC_HEIGHT			GENMASK(29, 16)
#define IDMA_SRC_WIDTH			GENMASK(15, 0)
#define IDMA_SRC_OFFSET			0x0014
#define IDMA_SRC_OFFSET_Y		GENMASK(28, 16)
#define IDMA_SRC_OFFSET_X		GENMASK(12, 0)
#define IDMA_IMG_SIZE			0x0018
#define IDMA_IMG_HEIGHT			GENMASK(28, 16)
#define IDMA_IMG_WIDTH			GENMASK(12, 0)
#define IDMA_IN_BASE_ADDR_Y		0x0040

/* The bootloader scans out through window 3, which is wired to IDMA_G0. */
#define DPU_WIN				3
#define DPU_CH_IDMA_G0			3

#define DPU_UPDATE_REQ	(DECON_UPDATE_REQ_GLOBAL | DECON_UPDATE_REQ_WIN(DPU_WIN))

struct exynos7885_dpu {
	struct drm_device drm;
	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_display_mode mode;

	void __iomem *decon;
	void __iomem *idma;
	int irq;

	/* IDMA_G0 setup the bootloader scans out, restored when disabled. */
	u32 boot_src_size, boot_src_offset, boot_img_size, boot_addr;

	/* Protects DECON read-modify-writes and the pending event. */
	spinlock_t lock;
	struct drm_pending_vblank_event *event;
};

#define to_dpu(x) container_of(x, struct exynos7885_dpu, drm)

static void dpu_decon_update_bits(struct exynos7885_dpu *dpu, u32 reg,
				  u32 mask, u32 val)
{
	u32 tmp = readl(dpu->decon + reg);

	writel((tmp & ~mask) | (val & mask), dpu->decon + reg);
}

/* Latch the shadow registers at the next frame start. */
static void dpu_request_update(struct exynos7885_dpu *dpu)
{
	dpu_decon_update_bits(dpu, DECON_SHADOW_REG_UPDATE_REQ,
			      DPU_UPDATE_REQ, DPU_UPDATE_REQ);
}

/* Wait until the hardware has latched the requested shadow registers. */
static int dpu_wait_update(struct exynos7885_dpu *dpu)
{
	u32 val;

	/* One frame is about 17 ms. */
	return readl_poll_timeout(dpu->decon + DECON_SHADOW_REG_UPDATE_REQ,
				  val, !(val & DPU_UPDATE_REQ), 1000, 100000);
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;
	int ret;

	if (!new_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);
	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, true);
	if (ret || !new_state->visible)
		return ret;

	/* Window 3 always covers the whole panel. */
	if (drm_rect_width(&new_state->dst) != crtc_state->mode.hdisplay ||
	    drm_rect_height(&new_state->dst) != crtc_state->mode.vdisplay)
		return -EINVAL;

	if (new_state->fb->pitches[0] % 4)
		return -EINVAL;

	return 0;
}

static void dpu_plane_atomic_update(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(plane->dev);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_rect src;
	dma_addr_t addr;

	if (!new_state->visible)
		return;

	drm_rect_fp_to_int(&src, &new_state->src);
	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);

	/*
	 * drm_fb_dma_get_gem_addr() already applies the source offset, so the
	 * IDMA reads from the top-left of its source window.
	 */
	writel(FIELD_PREP(IDMA_SRC_HEIGHT, fb->height - src.y1) |
	       FIELD_PREP(IDMA_SRC_WIDTH, fb->pitches[0] / 4),
	       dpu->idma + IDMA_SRC_SIZE);
	writel(0, dpu->idma + IDMA_SRC_OFFSET);
	writel(FIELD_PREP(IDMA_IMG_HEIGHT, drm_rect_height(&src)) |
	       FIELD_PREP(IDMA_IMG_WIDTH, drm_rect_width(&src)),
	       dpu->idma + IDMA_IMG_SIZE);
	writel(lower_32_bits(addr), dpu->idma + IDMA_IN_BASE_ADDR_Y);
}

static void dpu_plane_atomic_disable(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(plane->dev);
	unsigned long flags;

	/*
	 * Turning window 3 off stops this video-mode pipeline (DECON stops
	 * starting frames and the DSI line counter freezes). Scan out the
	 * bootloader framebuffer again instead, and wait for that to latch so
	 * the old framebuffer is no longer read when it is released.
	 */
	writel(dpu->boot_src_size, dpu->idma + IDMA_SRC_SIZE);
	writel(dpu->boot_src_offset, dpu->idma + IDMA_SRC_OFFSET);
	writel(dpu->boot_img_size, dpu->idma + IDMA_IMG_SIZE);
	writel(dpu->boot_addr, dpu->idma + IDMA_IN_BASE_ADDR_Y);

	spin_lock_irqsave(&dpu->lock, flags);
	dpu_request_update(dpu);
	spin_unlock_irqrestore(&dpu->lock, flags);

	if (dpu_wait_update(dpu))
		drm_warn(plane->dev, "scanout restore did not latch\n");
}

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
	.atomic_disable = dpu_plane_atomic_disable,
};

static const struct drm_plane_funcs dpu_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

/*
 * IDMA_G0 stays in the bootloader's ARGB8888 input format. DECON ignores the
 * alpha byte for this bottom window, so XRGB8888 buffers display unchanged.
 */
static const u32 dpu_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static int dpu_crtc_atomic_check(struct drm_crtc *crtc,
				 struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(crtc->dev);
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);

	/* The DSI link keeps the bootloader's timing. */
	if (crtc_state->enable && !drm_mode_equal(&crtc_state->mode, &dpu->mode))
		return -EINVAL;

	return 0;
}

/*
 * The DSI link and DECON keep running as the bootloader left them; the
 * window follows the primary plane.
 */
static void dpu_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static void dpu_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	drm_crtc_vblank_off(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void dpu_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;

	if (!crtc->state->active)
		return;

	if (event) {
		crtc->state->event = NULL;
		if (drm_crtc_vblank_get(crtc)) {
			/* No vblank to wait for; complete the event now. */
			spin_lock_irqsave(&crtc->dev->event_lock, flags);
			drm_crtc_send_vblank_event(crtc, event);
			spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
			event = NULL;
		}
	}

	/*
	 * Arm the event together with the update request, so the frame-start
	 * interrupt only completes it once the hardware has latched the new
	 * registers.
	 */
	spin_lock_irqsave(&dpu->lock, flags);
	dpu_request_update(dpu);
	if (event) {
		WARN_ON(dpu->event);
		dpu->event = event;
	}
	spin_unlock_irqrestore(&dpu->lock, flags);
}

static int dpu_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct exynos7885_dpu *dpu = to_dpu(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&dpu->lock, flags);
	writel(DECON_FRAME_START_INT_PEND, dpu->decon + DECON_INTERRUPT_PENDING);
	dpu_decon_update_bits(dpu, DECON_INTERRUPT_ENABLE,
			      DECON_FRAME_START_INT_EN | DECON_INT_EN,
			      DECON_FRAME_START_INT_EN | DECON_INT_EN);
	spin_unlock_irqrestore(&dpu->lock, flags);

	return 0;
}

static void dpu_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct exynos7885_dpu *dpu = to_dpu(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&dpu->lock, flags);
	dpu_decon_update_bits(dpu, DECON_INTERRUPT_ENABLE,
			      DECON_FRAME_START_INT_EN | DECON_INT_EN, 0);
	spin_unlock_irqrestore(&dpu->lock, flags);
}

static const struct drm_crtc_helper_funcs dpu_crtc_helper_funcs = {
	.atomic_check = dpu_crtc_atomic_check,
	.atomic_enable = dpu_crtc_atomic_enable,
	.atomic_disable = dpu_crtc_atomic_disable,
	.atomic_flush = dpu_crtc_atomic_flush,
};

static const struct drm_crtc_funcs dpu_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dpu_crtc_enable_vblank,
	.disable_vblank = dpu_crtc_disable_vblank,
};

static irqreturn_t dpu_frame_start_irq(int irq, void *data)
{
	struct exynos7885_dpu *dpu = data;
	struct drm_pending_vblank_event *event = NULL;
	u32 pend;

	pend = readl(dpu->decon + DECON_INTERRUPT_PENDING);
	if (!pend)
		return IRQ_NONE;
	/* Acknowledge everything so the line cannot stay asserted. */
	writel(pend, dpu->decon + DECON_INTERRUPT_PENDING);
	if (!(pend & DECON_FRAME_START_INT_PEND))
		return IRQ_HANDLED;

	drm_crtc_handle_vblank(&dpu->crtc);

	spin_lock(&dpu->lock);
	if (dpu->event && !(readl(dpu->decon + DECON_SHADOW_REG_UPDATE_REQ) &
			    DPU_UPDATE_REQ)) {
		event = dpu->event;
		dpu->event = NULL;
	}
	spin_unlock(&dpu->lock);

	if (event) {
		spin_lock(&dpu->drm.event_lock);
		drm_crtc_send_vblank_event(&dpu->crtc, event);
		spin_unlock(&dpu->drm.event_lock);
		drm_crtc_vblank_put(&dpu->crtc);
	}

	return IRQ_HANDLED;
}

/*
 * Enable the CRTC (and with it vblank) before committing planes, so the
 * flush can arm its event. Release the old framebuffers only after
 * flip_done: the vblank event is sent once the hardware latched the new
 * address, which can be one frame after the next vblank.
 */
static void dpu_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);
	drm_atomic_helper_commit_modeset_enables(dev, state);
	drm_atomic_helper_commit_planes(dev, state, 0);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_flip_done(dev, state);
	drm_atomic_helper_cleanup_planes(dev, state);
}

static const struct drm_mode_config_helper_funcs dpu_mode_config_helpers = {
	.atomic_commit_tail = dpu_atomic_commit_tail,
};

static const struct drm_mode_config_funcs dpu_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(dpu_fops);

static const struct drm_driver dpu_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &dpu_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "exynos7885-dpu",
	.desc = "Samsung Exynos7885 display processing unit",
	.major = 1,
	.minor = 0,
};

static int dpu_kms_init(struct exynos7885_dpu *dpu)
{
	struct drm_device *drm = &dpu->drm;
	struct drm_connector *connector;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = dpu->mode.hdisplay;
	drm->mode_config.max_width = dpu->mode.hdisplay;
	drm->mode_config.min_height = dpu->mode.vdisplay;
	drm->mode_config.max_height = dpu->mode.vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &dpu_mode_config_funcs;
	drm->mode_config.helper_private = &dpu_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &dpu->plane, 0, &dpu_plane_funcs,
				       dpu_plane_formats,
				       ARRAY_SIZE(dpu_plane_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&dpu->plane, &dpu_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &dpu->crtc, &dpu->plane, NULL,
					&dpu_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&dpu->crtc, &dpu_crtc_helper_funcs);

	ret = drm_simple_encoder_init(drm, &dpu->encoder, DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;
	dpu->encoder.possible_crtcs = drm_crtc_mask(&dpu->crtc);

	ret = drm_bridge_attach(&dpu->encoder, dpu->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, &dpu->encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);

	ret = drm_connector_attach_encoder(connector, &dpu->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	return 0;
}

static int dpu_check_handover(struct exynos7885_dpu *dpu)
{
	struct device *dev = dpu->drm.dev;
	u32 ctrl, path0, path1;

	ctrl = readl(dpu->decon + DECON_GLOBAL_CONTROL);
	path0 = readl(dpu->decon + DECON_DATA_PATH_CONTROL_0);
	path1 = readl(dpu->decon + DECON_DATA_PATH_CONTROL_1);

	if (!(ctrl & DECON_GLOBAL_RUN_STATUS) ||
	    !(path0 & DECON_WIN_EN(DPU_WIN)) ||
	    (path1 & DECON_WIN_CHMAP(DPU_WIN)) !=
	    (DPU_CH_IDMA_G0 << (4 * DPU_WIN))) {
		dev_err(dev, "bootloader display not running (ctrl %#x path %#x/%#x)\n",
			ctrl, path0, path1);
		return -ENODEV;
	}

	dpu->boot_src_size = readl(dpu->idma + IDMA_SRC_SIZE);
	dpu->boot_src_offset = readl(dpu->idma + IDMA_SRC_OFFSET);
	dpu->boot_img_size = readl(dpu->idma + IDMA_IMG_SIZE);
	dpu->boot_addr = readl(dpu->idma + IDMA_IN_BASE_ADDR_Y);

	return 0;
}

static int dpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct display_timing timing;
	struct exynos7885_dpu *dpu;
	struct videomode vm;
	int ret;

	dpu = devm_drm_dev_alloc(dev, &dpu_driver, struct exynos7885_dpu, drm);
	if (IS_ERR(dpu))
		return PTR_ERR(dpu);
	spin_lock_init(&dpu->lock);
	platform_set_drvdata(pdev, dpu);

	dpu->decon = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(dpu->decon))
		return PTR_ERR(dpu->decon);
	dpu->idma = devm_platform_ioremap_resource_byname(pdev, "idma-g0");
	if (IS_ERR(dpu->idma))
		return PTR_ERR(dpu->idma);
	dpu->irq = platform_get_irq_byname(pdev, "frame-start");
	if (dpu->irq < 0)
		return dpu->irq;

	/* The timing the bootloader programmed; the panel gives the size. */
	ret = of_get_display_timing(np, "panel-timing", &timing);
	if (ret)
		return dev_err_probe(dev, ret, "invalid panel-timing\n");
	videomode_from_timing(&timing, &vm);
	drm_display_mode_from_videomode(&vm, &dpu->mode);

	/* The DSI host bridge appears once the panel attached to it. */
	dpu->bridge = devm_drm_of_get_bridge(dev, np, 0, 0);
	if (IS_ERR(dpu->bridge))
		return dev_err_probe(dev, PTR_ERR(dpu->bridge),
				     "failed to get the DSI bridge\n");

	/* IDMA base address registers are 32 bits wide. */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = dpu_check_handover(dpu);
	if (ret)
		return ret;

	ret = dpu_kms_init(dpu);
	if (ret)
		return ret;

	/*
	 * The bootloader leaves frame start/done, underflow, extra and dimming
	 * interrupts enabled with their status pending. Mask them all and
	 * acknowledge what is pending; enabling vblank unmasks frame start.
	 */
	writel(0, dpu->decon + DECON_INTERRUPT_ENABLE);
	writel(0, dpu->decon + DECON_EXTRA_INTERRUPT_ENABLE);
	writel(readl(dpu->decon + DECON_EXTRA_INTERRUPT_PENDING),
	       dpu->decon + DECON_EXTRA_INTERRUPT_PENDING);
	writel(readl(dpu->decon + DECON_INTERRUPT_PENDING),
	       dpu->decon + DECON_INTERRUPT_PENDING);

	ret = devm_request_irq(dev, dpu->irq, dpu_frame_start_irq, 0,
			       dev_name(dev), dpu);
	if (ret)
		return ret;

	ret = aperture_remove_all_conflicting_devices(dpu_driver.name);
	if (ret)
		return ret;

	ret = drm_dev_register(&dpu->drm, 0);
	if (ret)
		return ret;

	drm_client_setup(&dpu->drm, NULL);

	return 0;
}

static void dpu_remove(struct platform_device *pdev)
{
	struct exynos7885_dpu *dpu = platform_get_drvdata(pdev);

	drm_dev_unregister(&dpu->drm);
	drm_atomic_helper_shutdown(&dpu->drm);
}

static void dpu_shutdown(struct platform_device *pdev)
{
	struct exynos7885_dpu *dpu = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&dpu->drm);
}

static const struct of_device_id dpu_of_match[] = {
	{ .compatible = "samsung,exynos7885-dpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, dpu_of_match);

static struct platform_driver dpu_platform_driver = {
	.driver = {
		.name = "exynos7885-dpu",
		.of_match_table = dpu_of_match,
	},
	.probe = dpu_probe,
	.remove = dpu_remove,
	.shutdown = dpu_shutdown,
};
module_platform_driver(dpu_platform_driver);

MODULE_DESCRIPTION("Samsung Exynos7885 display processing unit DRM driver");
MODULE_LICENSE("GPL");
