// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 display processing unit (DECON0 + DPP IDMA_G0).
 *
 * The bootloader leaves DECON0, the DSI host and the panel running in video
 * mode, scanning out its framebuffer through window 3 and IDMA_G0. This
 * driver takes that pipeline over: it moves the scanout to window 0, the
 * bottom one, so IDMA_G1 and IDMA_GF can be shown above it as overlay and
 * cursor planes in windows 1 and 2. It moves the IDMA fetch addresses to
 * DRM framebuffers and latches each change at frame start through the
 * DECON shadow registers, so page flips are tear-free. The DSI host and the panel
 * are reached through the DRM bridge chain. While the display is off the
 * DECON is stopped; clocks and power are left as the bootloader set them.
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
#include <drm/drm_blend.h>
#include <drm/drm_modeset_helper.h>
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
#define DECON_GLOBAL_EN_F		BIT(0)
#define DECON_GLOBAL_EN			BIT(1)
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
#define DECON_WIN_BASE(w)		(0x1000 + 0x30 * (w))
#define DECON_WIN_CONTROL_0(w)		(DECON_WIN_BASE(w) + 0x00)
#define DECON_WIN_ALPHA0		GENMASK(23, 16)
#define DECON_WIN_FUNC_USER		(0xd << 8)
#define DECON_WIN_ALPHA_MULT_AF		0x2
#define DECON_WIN_CONTROL_1(w)		(DECON_WIN_BASE(w) + 0x04)
#define DECON_WIN_START_POSITION(w)	(DECON_WIN_BASE(w) + 0x08)
#define DECON_WIN_END_POSITION(w)	(DECON_WIN_BASE(w) + 0x0c)
#define DECON_WIN_POS_Y			GENMASK(29, 16)
#define DECON_WIN_POS_X			GENMASK(13, 0)
#define DECON_WIN_COLORMAP_0(w)		(DECON_WIN_BASE(w) + 0x10)
#define DECON_WIN_PIXEL_COUNT(w)	(DECON_WIN_BASE(w) + 0x1c)
#define DECON_WIN_NUM_REGS		8

/*
 * Blend coefficients (vendor decon_reg_set_win_sub_coeff()): FG and BG
 * factors for the colour, then for the alpha. ALPHA_MULT is the pixel
 * alpha times the plane alpha.
 */
#define DECON_COEF_PLANE_ALPHA0		0x6
#define DECON_COEF_1_M_PLANE_ALPHA0	0x7
#define DECON_COEF_ALPHA_MULT		0xa
#define DECON_COEF_1_M_ALPHA_MULT	0xb
#define DECON_WIN_COEF(fg, bg)		((fg) << 24 | (bg) << 16 | (fg) << 8 | (bg))

/* IDMA registers */
#define IDMA_ENABLE			0x0000
#define IDMA_CLOCK_GATE_EN		(BIT(10) | BIT(9))
#define IDMA_IN_CON			0x0008
#define IDMA_IN_IC_MAX			(0x20 << 19)
#define IDMA_IMG_FORMAT			GENMASK(15, 11)
#define IDMA_FORMAT_ARGB8888		0
#define IDMA_FORMAT_XRGB8888		4
#define IDMA_OUT_CON			0x000c
#define IDMA_OUT_FRAME_ALPHA_MAX	(0xff << 24)
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
#define IDMA_DYNAMIC_GATING_EN		0x0058
#define IDMA_DYNAMIC_GATING_ALL		0xff

/*
 * The bootloader scans out through window 3, the top one, whose channel is
 * IDMA_G0. Windows with a higher number are shown above the lower ones.
 */
#define DPU_BOOT_WIN			3
#define DPU_CH_IDMA_G0			3
#define DPU_CH_IDMA_G1			1
#define DPU_CH_IDMA_GF			2
#define DPU_PRIMARY_WIN			0

/* The IDMA channels take 16 to 4096 pixels per side. */
#define DPU_PLANE_MIN			16

/* all windows: the scanout stays in window 3 without the other channels */
#define DPU_UPDATE_REQ	(DECON_UPDATE_REQ_GLOBAL | DECON_UPDATE_REQ_WIN(0) | \
			 DECON_UPDATE_REQ_WIN(1) | DECON_UPDATE_REQ_WIN(2) | \
			 DECON_UPDATE_REQ_WIN(3))

enum dpu_plane_id {
	DPU_PLANE_PRIMARY,
	DPU_PLANE_OVERLAY,
	DPU_PLANE_CURSOR,
	DPU_NUM_PLANES,
};

struct dpu_plane {
	struct drm_plane base;
	void __iomem *idma;
	unsigned int win;
	unsigned int ch;
};

#define to_dpu_plane(x) container_of(x, struct dpu_plane, base)

struct exynos7885_dpu {
	struct drm_device drm;
	struct dpu_plane planes[DPU_NUM_PLANES];
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_display_mode mode;

	void __iomem *decon;
	void __iomem *idma;
	unsigned int num_planes;
	int irq;

	/* IDMA_G0 setup the bootloader scans out, restored when disabled. */
	u32 boot_src_size, boot_src_offset, boot_img_size, boot_addr;

	/* Protects DECON read-modify-writes and the pending event. */
	spinlock_t lock;
	struct drm_pending_vblank_event *event;
	/* DECON stopped while the display is off */
	bool stopped;
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

	/* The scanout window always covers the whole panel. */
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
	 * Running the DECON without any window stops this video-mode pipeline
	 * (the DSI line counter freezes). Scan out the bootloader framebuffer
	 * again instead, and wait for that to latch so the old framebuffer is
	 * no longer read when it is released.
	 */
	writel(dpu->boot_src_size, dpu->idma + IDMA_SRC_SIZE);
	writel(dpu->boot_src_offset, dpu->idma + IDMA_SRC_OFFSET);
	writel(dpu->boot_img_size, dpu->idma + IDMA_IMG_SIZE);
	writel(dpu->boot_addr, dpu->idma + IDMA_IN_BASE_ADDR_Y);

	spin_lock_irqsave(&dpu->lock, flags);
	dpu_request_update(dpu);
	spin_unlock_irqrestore(&dpu->lock, flags);

	/* A stopped DECON reads nothing; it latches this when restarted. */
	if (!dpu->stopped && dpu_wait_update(dpu))
		drm_warn(plane->dev, "scanout restore did not latch\n");
}

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
	.atomic_disable = dpu_plane_atomic_disable,
};

static int dpu_overlay_atomic_check(struct drm_plane *plane,
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
						  true, true);
	if (ret || !new_state->visible)
		return ret;

	if (new_state->fb->pitches[0] % 4)
		return -EINVAL;

	return 0;
}

static void dpu_window_enable(struct exynos7885_dpu *dpu, unsigned int win,
			      unsigned int ch, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&dpu->lock, flags);
	if (enable)
		dpu_decon_update_bits(dpu, DECON_DATA_PATH_CONTROL_1,
				      DECON_WIN_CHMAP(win), ch << (4 * win));
	dpu_decon_update_bits(dpu, DECON_DATA_PATH_CONTROL_0, DECON_WIN_EN(win),
			      enable ? DECON_WIN_EN(win) : 0);
	spin_unlock_irqrestore(&dpu->lock, flags);
}

static void dpu_overlay_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(plane->dev);
	struct dpu_plane *dp = to_dpu_plane(plane);
	struct drm_plane_state *new_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_gem_dma_object *gem;
	struct drm_rect src, *dst = &new_state->dst;
	u32 alpha = new_state->alpha >> 8;
	u32 fmt, coef;

	drm_rect_fp_to_int(&src, &new_state->src);

	/* A window too small for the IDMA is left out. */
	if (!new_state->visible ||
	    drm_rect_width(dst) < DPU_PLANE_MIN ||
	    drm_rect_height(dst) < DPU_PLANE_MIN) {
		dpu_window_enable(dpu, dp->win, dp->ch, false);
		return;
	}

	gem = drm_fb_dma_get_gem_obj(fb, 0);

	if (fb->format->format == DRM_FORMAT_XRGB8888 ||
	    new_state->pixel_blend_mode == DRM_MODE_BLEND_PIXEL_NONE) {
		fmt = IDMA_FORMAT_XRGB8888;
		coef = DECON_WIN_COEF(DECON_COEF_PLANE_ALPHA0,
				      DECON_COEF_1_M_PLANE_ALPHA0);
	} else if (new_state->pixel_blend_mode == DRM_MODE_BLEND_COVERAGE) {
		fmt = IDMA_FORMAT_ARGB8888;
		coef = DECON_WIN_COEF(DECON_COEF_ALPHA_MULT,
				      DECON_COEF_1_M_ALPHA_MULT);
	} else {
		fmt = IDMA_FORMAT_ARGB8888;
		coef = DECON_WIN_COEF(DECON_COEF_PLANE_ALPHA0,
				      DECON_COEF_1_M_ALPHA_MULT);
	}

	writel(IDMA_CLOCK_GATE_EN, dp->idma + IDMA_ENABLE);
	writel(IDMA_IN_IC_MAX | FIELD_PREP(IDMA_IMG_FORMAT, fmt),
	       dp->idma + IDMA_IN_CON);
	writel(IDMA_OUT_FRAME_ALPHA_MAX, dp->idma + IDMA_OUT_CON);
	writel(FIELD_PREP(IDMA_SRC_HEIGHT, fb->height) |
	       FIELD_PREP(IDMA_SRC_WIDTH, fb->pitches[0] / 4),
	       dp->idma + IDMA_SRC_SIZE);
	writel(FIELD_PREP(IDMA_SRC_OFFSET_Y, src.y1) |
	       FIELD_PREP(IDMA_SRC_OFFSET_X, src.x1),
	       dp->idma + IDMA_SRC_OFFSET);
	writel(FIELD_PREP(IDMA_IMG_HEIGHT, drm_rect_height(&src)) |
	       FIELD_PREP(IDMA_IMG_WIDTH, drm_rect_width(&src)),
	       dp->idma + IDMA_IMG_SIZE);
	writel(lower_32_bits(gem->dma_addr + fb->offsets[0]),
	       dp->idma + IDMA_IN_BASE_ADDR_Y);
	writel(IDMA_DYNAMIC_GATING_ALL, dp->idma + IDMA_DYNAMIC_GATING_EN);

	writel(FIELD_PREP(DECON_WIN_ALPHA0, alpha) | DECON_WIN_FUNC_USER |
	       DECON_WIN_ALPHA_MULT_AF, dpu->decon + DECON_WIN_CONTROL_0(dp->win));
	writel(coef, dpu->decon + DECON_WIN_CONTROL_1(dp->win));
	writel(FIELD_PREP(DECON_WIN_POS_Y, dst->y1) |
	       FIELD_PREP(DECON_WIN_POS_X, dst->x1),
	       dpu->decon + DECON_WIN_START_POSITION(dp->win));
	writel(FIELD_PREP(DECON_WIN_POS_Y, dst->y2 - 1) |
	       FIELD_PREP(DECON_WIN_POS_X, dst->x2 - 1),
	       dpu->decon + DECON_WIN_END_POSITION(dp->win));

	dpu_window_enable(dpu, dp->win, dp->ch, true);
}

static void dpu_overlay_atomic_disable(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(plane->dev);
	struct dpu_plane *dp = to_dpu_plane(plane);
	unsigned long flags;

	dpu_window_enable(dpu, dp->win, dp->ch, false);

	/* The window stops reading its framebuffer once this latched. */
	spin_lock_irqsave(&dpu->lock, flags);
	dpu_request_update(dpu);
	spin_unlock_irqrestore(&dpu->lock, flags);
	if (!dpu->stopped && dpu_wait_update(dpu))
		drm_warn(plane->dev, "window %u did not switch off\n", dp->win);
}

static const struct drm_plane_helper_funcs dpu_overlay_helper_funcs = {
	.atomic_check = dpu_overlay_atomic_check,
	.atomic_update = dpu_overlay_atomic_update,
	.atomic_disable = dpu_overlay_atomic_disable,
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
 * IDMA_G0 stays in the bootloader's ARGB8888 input format. The scanout
 * window copies its pixels without blending, so XRGB8888 buffers display
 * unchanged.
 */
static const u32 dpu_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const u32 dpu_overlay_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
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

/*
 * Stop and restart the DECON while the display is off, between the panel
 * being switched off and the DSI link being stopped, as the vendor driver
 * does. The scanout window stays enabled: running the DECON without a
 * window instead stops the video pipeline in a way that only a reboot
 * recovers. The DECON registers keep their values meanwhile.
 */
static void dpu_encoder_atomic_disable(struct drm_encoder *encoder,
				       struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(encoder->dev);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&dpu->lock, flags);
	dpu_decon_update_bits(dpu, DECON_GLOBAL_CONTROL, DECON_GLOBAL_EN_F, 0);
	dpu_decon_update_bits(dpu, DECON_SHADOW_REG_UPDATE_REQ,
			      DECON_UPDATE_REQ_GLOBAL, DECON_UPDATE_REQ_GLOBAL);
	spin_unlock_irqrestore(&dpu->lock, flags);

	/* It stops at the end of the current frame. */
	if (readl_poll_timeout(dpu->decon + DECON_GLOBAL_CONTROL, val,
			       !(val & DECON_GLOBAL_RUN_STATUS), 100, 50000))
		drm_warn(encoder->dev, "DECON did not stop\n");
	dpu->stopped = true;
}

static void dpu_encoder_atomic_enable(struct drm_encoder *encoder,
				      struct drm_atomic_state *state)
{
	struct exynos7885_dpu *dpu = to_dpu(encoder->dev);
	unsigned long flags;
	u32 val;

	/* The bootloader leaves the DECON running. */
	if (!dpu->stopped)
		return;

	spin_lock_irqsave(&dpu->lock, flags);
	dpu_decon_update_bits(dpu, DECON_GLOBAL_CONTROL,
			      DECON_GLOBAL_EN | DECON_GLOBAL_EN_F,
			      DECON_GLOBAL_EN | DECON_GLOBAL_EN_F);
	dpu_decon_update_bits(dpu, DECON_SHADOW_REG_UPDATE_REQ,
			      DECON_UPDATE_REQ_GLOBAL, DECON_UPDATE_REQ_GLOBAL);
	spin_unlock_irqrestore(&dpu->lock, flags);

	if (readl_poll_timeout(dpu->decon + DECON_GLOBAL_CONTROL, val,
			       val & DECON_GLOBAL_RUN_STATUS, 100, 50000))
		drm_warn(encoder->dev, "DECON did not start\n");
	dpu->stopped = false;
}

static const struct drm_encoder_helper_funcs dpu_encoder_helper_funcs = {
	.atomic_enable = dpu_encoder_atomic_enable,
	.atomic_disable = dpu_encoder_atomic_disable,
};

static int dpu_kms_init(struct exynos7885_dpu *dpu)
{
	struct drm_device *drm = &dpu->drm;
	struct drm_connector *connector;
	unsigned int i;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = dpu->mode.hdisplay;
	drm->mode_config.max_width = dpu->mode.hdisplay;
	drm->mode_config.min_height = dpu->mode.vdisplay;
	drm->mode_config.max_height = dpu->mode.vdisplay;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.cursor_width = 64;
	drm->mode_config.cursor_height = 64;
	drm->mode_config.funcs = &dpu_mode_config_funcs;
	drm->mode_config.helper_private = &dpu_mode_config_helpers;

	ret = drm_universal_plane_init(drm, &dpu->planes[DPU_PLANE_PRIMARY].base,
				       0, &dpu_plane_funcs, dpu_plane_formats,
				       ARRAY_SIZE(dpu_plane_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&dpu->planes[DPU_PLANE_PRIMARY].base,
			     &dpu_plane_helper_funcs);
	drm_plane_create_zpos_immutable_property(&dpu->planes[DPU_PLANE_PRIMARY].base,
						 0);

	for (i = DPU_PLANE_OVERLAY; i < dpu->num_planes; i++) {
		struct drm_plane *plane = &dpu->planes[i].base;

		ret = drm_universal_plane_init(drm, plane, 0, &dpu_plane_funcs,
					       dpu_overlay_formats,
					       ARRAY_SIZE(dpu_overlay_formats),
					       NULL, i == DPU_PLANE_CURSOR ?
					       DRM_PLANE_TYPE_CURSOR :
					       DRM_PLANE_TYPE_OVERLAY, NULL);
		if (ret)
			return ret;
		drm_plane_helper_add(plane, &dpu_overlay_helper_funcs);
		drm_plane_create_zpos_immutable_property(plane, i);
		drm_plane_create_alpha_property(plane);
		drm_plane_create_blend_mode_property(plane,
					BIT(DRM_MODE_BLEND_PIXEL_NONE) |
					BIT(DRM_MODE_BLEND_PREMULTI) |
					BIT(DRM_MODE_BLEND_COVERAGE));
	}

	ret = drm_crtc_init_with_planes(drm, &dpu->crtc,
					&dpu->planes[DPU_PLANE_PRIMARY].base,
					dpu->num_planes > DPU_PLANE_CURSOR ?
					&dpu->planes[DPU_PLANE_CURSOR].base : NULL,
					&dpu_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&dpu->crtc, &dpu_crtc_helper_funcs);

	ret = drm_simple_encoder_init(drm, &dpu->encoder, DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;
	dpu->encoder.possible_crtcs = drm_crtc_mask(&dpu->crtc);
	drm_encoder_helper_add(&dpu->encoder, &dpu_encoder_helper_funcs);

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

	/* This driver may have moved the scanout to window 0 already. */
	if (!(ctrl & DECON_GLOBAL_RUN_STATUS) ||
	    !((path0 & DECON_WIN_EN(DPU_BOOT_WIN)) ||
	      ((path0 & DECON_WIN_EN(DPU_PRIMARY_WIN)) &&
	       (path1 & DECON_WIN_CHMAP(DPU_PRIMARY_WIN)) ==
	       (DPU_CH_IDMA_G0 << (4 * DPU_PRIMARY_WIN))))) {
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

/*
 * Move the scanout from window 3, the top one, to window 0, the bottom
 * one, so the other channels can be shown above it. Window 0 takes window
 * 3's setup, and both change in one shadow update, so the DECON never
 * starts a frame without a window. Window 3's channel stays IDMA_G0.
 */
static int dpu_move_to_bottom(struct exynos7885_dpu *dpu)
{
	u32 path0 = readl(dpu->decon + DECON_DATA_PATH_CONTROL_0);
	u32 req = DECON_UPDATE_REQ_GLOBAL | DECON_UPDATE_REQ_WIN(DPU_BOOT_WIN) |
		  DECON_UPDATE_REQ_WIN(DPU_PRIMARY_WIN);
	unsigned int i;
	u32 val;

	if (!(path0 & DECON_WIN_EN(DPU_BOOT_WIN)))
		return 0;

	for (i = 0; i < DECON_WIN_NUM_REGS; i++)
		writel(readl(dpu->decon + DECON_WIN_BASE(DPU_BOOT_WIN) + 4 * i),
		       dpu->decon + DECON_WIN_BASE(DPU_PRIMARY_WIN) + 4 * i);
	dpu_decon_update_bits(dpu, DECON_DATA_PATH_CONTROL_1,
			      DECON_WIN_CHMAP(DPU_PRIMARY_WIN),
			      DPU_CH_IDMA_G0 << (4 * DPU_PRIMARY_WIN));
	writel(DECON_WIN_EN(DPU_PRIMARY_WIN),
	       dpu->decon + DECON_DATA_PATH_CONTROL_0);
	dpu_decon_update_bits(dpu, DECON_SHADOW_REG_UPDATE_REQ, req, req);

	return readl_poll_timeout(dpu->decon + DECON_SHADOW_REG_UPDATE_REQ, val,
				  !(val & req), 1000, 100000);
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
	dpu->planes[DPU_PLANE_PRIMARY].idma = dpu->idma;
	dpu->planes[DPU_PLANE_PRIMARY].win = DPU_PRIMARY_WIN;
	dpu->planes[DPU_PLANE_PRIMARY].ch = DPU_CH_IDMA_G0;
	dpu->num_planes = 1;

	/* Overlay and cursor planes, if their channels are described. */
	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "idma-g1") &&
	    platform_get_resource_byname(pdev, IORESOURCE_MEM, "idma-gf")) {
		dpu->planes[DPU_PLANE_OVERLAY].idma =
			devm_platform_ioremap_resource_byname(pdev, "idma-g1");
		if (IS_ERR(dpu->planes[DPU_PLANE_OVERLAY].idma))
			return PTR_ERR(dpu->planes[DPU_PLANE_OVERLAY].idma);
		dpu->planes[DPU_PLANE_OVERLAY].win = 1;
		dpu->planes[DPU_PLANE_OVERLAY].ch = DPU_CH_IDMA_G1;

		dpu->planes[DPU_PLANE_CURSOR].idma =
			devm_platform_ioremap_resource_byname(pdev, "idma-gf");
		if (IS_ERR(dpu->planes[DPU_PLANE_CURSOR].idma))
			return PTR_ERR(dpu->planes[DPU_PLANE_CURSOR].idma);
		dpu->planes[DPU_PLANE_CURSOR].win = 2;
		dpu->planes[DPU_PLANE_CURSOR].ch = DPU_CH_IDMA_GF;
		dpu->num_planes = DPU_NUM_PLANES;
	}
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

	if (dpu->num_planes > 1) {
		ret = dpu_move_to_bottom(dpu);
		if (ret)
			return dev_err_probe(dev, ret,
					     "scanout did not move to window 0\n");
	}

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

/* System sleep switches the display path off and on like DPMS. */
static int dpu_suspend(struct device *dev)
{
	struct exynos7885_dpu *dpu = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&dpu->drm);
}

static int dpu_resume(struct device *dev)
{
	struct exynos7885_dpu *dpu = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&dpu->drm);
}

static DEFINE_SIMPLE_DEV_PM_OPS(dpu_pm_ops, dpu_suspend, dpu_resume);

static struct platform_driver dpu_platform_driver = {
	.driver = {
		.name = "exynos7885-dpu",
		.of_match_table = dpu_of_match,
		.pm = pm_sleep_ptr(&dpu_pm_ops),
	},
	.probe = dpu_probe,
	.remove = dpu_remove,
	.shutdown = dpu_shutdown,
};
module_platform_driver(dpu_platform_driver);

MODULE_DESCRIPTION("Samsung Exynos7885 display processing unit DRM driver");
MODULE_LICENSE("GPL");
