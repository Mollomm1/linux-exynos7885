// SPDX-License-Identifier: GPL-2.0-only
/*
 * Himax HX8279D 1200x1920 video-mode DSI panel of the Samsung Galaxy Tab A
 * 10.1 (2019, SM-T510, "gta3xl").
 *
 * The bootloader powers and initializes the panel. This driver switches
 * the display on and off and drives the backlight, which the controller
 * sets through a DSI register. Commands and the brightness curve follow
 * the vendor driver (hx8279d_gta3xllte).
 */

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#define HX8279D_PAGE		0xb0	/* register page select */
#define HX8279D_DISPLAY_CTRL	0xb3	/* page 0 */
#define HX8279D_DISPLAY_ON	0x08
#define HX8279D_DISPLAY_BL_OFF	0x8c	/* display off, backlight off */
#define HX8279D_BRIGHTNESS	0xb8	/* page 4 */

#define HX8279D_MAX_BRIGHTNESS	255
#define HX8279D_DEFAULT_LEVEL	117

struct hx8279d {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
};

static inline struct hx8279d *to_hx8279d(struct drm_panel *panel)
{
	return container_of(panel, struct hx8279d, panel);
}

/*
 * Vendor curve from UI level (0-255) to the brightness register; the
 * register's top values are kept for the vendor's outdoor mode.
 */
static u8 hx8279d_level_to_reg(unsigned int level)
{
	/* 255 -> 156, linear with the vendor table to within one step */
	return DIV_ROUND_CLOSEST(level * 156, HX8279D_MAX_BRIGHTNESS);
}

static const struct drm_display_mode hx8279d_mode = {
	.clock = 164976,
	.hdisplay = 1200,
	.hsync_start = 1200 + 150,
	.hsync_end = 1200 + 150 + 10,
	.htotal = 1200 + 150 + 10 + 40,
	.vdisplay = 1920,
	.vsync_start = 1920 + 22,
	.vsync_end = 1920 + 22 + 4,
	.vtotal = 1920 + 22 + 4 + 18,
	.width_mm = 135,
	.height_mm = 216,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int hx8279d_enable(struct drm_panel *panel)
{
	struct mipi_dsi_multi_context ctx = { .dsi = to_hx8279d(panel)->dsi };

	mipi_dsi_dcs_write_seq_multi(&ctx, HX8279D_PAGE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, HX8279D_DISPLAY_CTRL,
				     HX8279D_DISPLAY_ON);
	mipi_dsi_msleep(&ctx, 40);

	return ctx.accum_err;
}

static int hx8279d_disable(struct drm_panel *panel)
{
	struct mipi_dsi_multi_context ctx = { .dsi = to_hx8279d(panel)->dsi };

	mipi_dsi_dcs_write_seq_multi(&ctx, HX8279D_PAGE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&ctx, HX8279D_DISPLAY_CTRL,
				     HX8279D_DISPLAY_BL_OFF);
	mipi_dsi_msleep(&ctx, 50);

	return ctx.accum_err;
}

static int hx8279d_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &hx8279d_mode);
}

static const struct drm_panel_funcs hx8279d_panel_funcs = {
	.enable = hx8279d_enable,
	.disable = hx8279d_disable,
	.get_modes = hx8279d_get_modes,
};

static int hx8279d_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_multi_context ctx = { .dsi = bl_get_data(bl) };
	u8 cmd[2] = {
		HX8279D_BRIGHTNESS,
		hx8279d_level_to_reg(backlight_get_brightness(bl)),
	};

	mipi_dsi_dcs_write_seq_multi(&ctx, HX8279D_PAGE, 0x04);
	mipi_dsi_dcs_write_buffer_multi(&ctx, cmd, sizeof(cmd));

	return ctx.accum_err;
}

static const struct backlight_ops hx8279d_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = hx8279d_bl_update_status,
};

static int hx8279d_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = HX8279D_DEFAULT_LEVEL,
		.max_brightness = HX8279D_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct hx8279d *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	drm_panel_init(&ctx->panel, dev, &hx8279d_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

	ctx->panel.prepare_prev_first = true;
	ctx->panel.backlight = devm_backlight_device_register(dev,
					dev_name(dev), dev, dsi,
					&hx8279d_bl_ops, &props);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to register backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void hx8279d_remove(struct mipi_dsi_device *dsi)
{
	struct hx8279d *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hx8279d_of_match[] = {
	{ .compatible = "samsung,gta3xl-hx8279d" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx8279d_of_match);

static struct mipi_dsi_driver hx8279d_driver = {
	.driver = {
		.name = "panel-himax-hx8279d-gta3xl",
		.of_match_table = hx8279d_of_match,
	},
	.probe = hx8279d_probe,
	.remove = hx8279d_remove,
};
module_mipi_dsi_driver(hx8279d_driver);

MODULE_DESCRIPTION("Himax HX8279D panel driver for the Samsung SM-T510");
MODULE_LICENSE("GPL");
