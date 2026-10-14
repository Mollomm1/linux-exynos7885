// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 MIPI DSI host (DSIM).
 *
 * The bootloader leaves the DSIM link running in video mode. This driver
 * provides the DSI host, so a panel driver can send commands while video
 * streams, and a DRM bridge between the DPU and the panel. While the
 * display is off it stops the link, switches the D-PHY PLL off, resets the
 * DSIM and isolates the D-PHY, as the vendor driver does; to start it
 * again it programs the registers the bootloader had set, saved at probe,
 * in the vendor's order. The panel is powered off before and on after. As in the vendor driver, commands are queued with packet-go held
 * and released, without waiting for each to be sent: in video mode the
 * link sends queued packets in the vertical front porch, so a frame takes
 * as many as fit there.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>

#define DSIM_SWRST			0x04
#define DSIM_DPHY_RST			BIT(16)
#define DSIM_SWRST_RESET		BIT(0)
#define DSIM_LINK_STATUS0		0x08
#define DSIM_VIDEO_MODE_STATUS		BIT(24)
#define DSIM_VM_LINE_CNT		GENMASK(12, 0)
#define DSIM_LINK_STATUS2		0x10
#define DSIM_CLK_LANE_STATUS		GENMASK(5, 4)
#define DSIM_LINK_STATUS3		0x14
#define DSIM_PLL_STABLE			BIT(0)
#define DSIM_DPHY_STATUS		0x1c
#define DSIM_TX_READY_HS_CLK		BIT(10)
#define DSIM_CLK_CTRL			0x20
#define DSIM_TX_REQUEST_HSCLK		BIT(20)
#define DSIM_WORDCLK_EN			BIT(17)
#define DSIM_ESCCLK_EN			BIT(16)
#define DSIM_LANE_ESCCLK_EN		GENMASK(12, 8)
#define DSIM_ESC_PRESCALER		GENMASK(7, 0)
#define DSIM_CONFIG			0x4c
#define DSIM_LANES_EN			GENMASK(4, 0)
#define DSIM_PKTHDR			0x58
#define DSIM_PAYLOAD			0x5c
#define DSIM_FIFOCTRL			0x68
#define DSIM_FIFOCTRL_NUM_PH_SFR	GENMASK(21, 16)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(10)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(8)
#define DSIM_CMD_CONFIG			0x80
#define DSIM_CMD_CONFIG_PKT_GO_EN	BIT(16)
#define DSIM_PLLCTRL			0xa0
#define DSIM_PLL_EN			BIT(23)
#define DSIM_PLL_CTRL1			0xa4
#define DSIM_PLL_CTRL2			0xa8
#define DSIM_PLLTMR			0xac
#define DSIM_PHY_TIMING2		0xd8

/* PMU register isolating the D-PHY when 0 */
#define PMU_MIPI_PHY_CON		0x0678
#define PMU_MIPI_PHY_ENABLE		BIT(0)

/* The payload FIFO holds 2 KiB; panel commands are far smaller. */
#define DSIM_MAX_PAYLOAD		256
#define DSIM_TIMEOUT_US			50000
/*
 * Packets the vendor queues at most in video mode: the front porch lines
 * that may carry commands, VFP (22) - 2 stable lines - 3.
 */
#define DSIM_VIDEO_MAX_PH		17

/*
 * Link registers the software reset clears (they are programmed after
 * it), in the vendor's dsim_reg_set_config() order by address.
 */
static const u16 dsim_link_regs[] = {
	0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c, 0x40, 0x44, 0x48,
	DSIM_CONFIG, 0x54, 0x64, DSIM_CMD_CONFIG, 0x90, 0x9c,
};

/* PHY timing and control registers, from 0xb0 to PHY_TIMING2 */
#define DSIM_PHY_REG(i)			(0xb0 + 4 * (i))
#define DSIM_NUM_PHY_REGS		((DSIM_PHY_TIMING2 - 0xb0) / 4 + 1)

struct exynos7885_dsim {
	struct device *dev;
	void __iomem *regs;
	struct mipi_dsi_host host;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct regmap *pmu;
	/* Serializes command transfers and link state changes. */
	struct mutex lock;
	bool stopped;

	/* bootloader register values */
	u32 clk_ctrl, config, pllctrl, pll_ctrl1, pll_ctrl2, plltmr;
	u32 link[ARRAY_SIZE(dsim_link_regs)];
	u32 phy[DSIM_NUM_PHY_REGS];
};

static inline struct exynos7885_dsim *host_to_dsim(struct mipi_dsi_host *h)
{
	return container_of(h, struct exynos7885_dsim, host);
}

static inline struct exynos7885_dsim *bridge_to_dsim(struct drm_bridge *b)
{
	return container_of(b, struct exynos7885_dsim, bridge);
}

static int dsim_wait_fifo_empty(struct exynos7885_dsim *dsim, u32 mask)
{
	u32 val;

	return readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				  (val & mask) == mask, 10, DSIM_TIMEOUT_US);
}

static ssize_t dsim_host_transfer(struct mipi_dsi_host *host,
				  const struct mipi_dsi_msg *msg)
{
	struct exynos7885_dsim *dsim = host_to_dsim(host);
	struct mipi_dsi_packet packet;
	u32 cfg, val;
	size_t i;
	int ret;

	/* Only writes are needed so far; reads need bus turnaround. */
	if (msg->rx_len)
		return -EOPNOTSUPP;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;
	if (packet.payload_length > DSIM_MAX_PAYLOAD)
		return -EINVAL;

	guard(mutex)(&dsim->lock);

	if (dsim->stopped ||
	    !(readl(dsim->regs + DSIM_LINK_STATUS0) & DSIM_VIDEO_MODE_STATUS))
		return -EIO;

	/* Wait for room in the header FIFO, and an empty payload FIFO. */
	ret = readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				 FIELD_GET(DSIM_FIFOCTRL_NUM_PH_SFR, val) <
				 DSIM_VIDEO_MAX_PH, 10, DSIM_TIMEOUT_US);
	if (!ret && packet.payload_length)
		ret = dsim_wait_fifo_empty(dsim, DSIM_FIFOCTRL_EMPTY_PL_SFR);
	if (ret)
		return ret;

	/* Do not queue on line 0, the frame boundary. */
	ret = readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS0, val,
				 FIELD_GET(DSIM_VM_LINE_CNT, val), 10,
				 DSIM_TIMEOUT_US);
	if (ret)
		return ret;

	cfg = readl(dsim->regs + DSIM_CMD_CONFIG);
	writel(cfg | DSIM_CMD_CONFIG_PKT_GO_EN, dsim->regs + DSIM_CMD_CONFIG);

	for (i = 0; i < packet.payload_length; i += 4) {
		u32 word = 0;
		size_t j;

		for (j = 0; j < 4 && i + j < packet.payload_length; j++)
			word |= packet.payload[i + j] << (8 * j);
		writel(word, dsim->regs + DSIM_PAYLOAD);
	}

	writel(packet.header[0] | packet.header[1] << 8 |
	       packet.header[2] << 16, dsim->regs + DSIM_PKTHDR);
	writel(cfg & ~DSIM_CMD_CONFIG_PKT_GO_EN, dsim->regs + DSIM_CMD_CONFIG);

	return msg->tx_len;
}

static int dsim_host_attach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct exynos7885_dsim *dsim = host_to_dsim(host);
	struct drm_bridge *next;

	/* The link keeps the bootloader's lane count, format and timing. */
	next = devm_drm_of_get_bridge(dsim->dev, dsim->dev->of_node, 1, 0);
	if (IS_ERR(next))
		return PTR_ERR(next);

	dsim->next_bridge = next;
	drm_bridge_add(&dsim->bridge);

	return 0;
}

static int dsim_host_detach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct exynos7885_dsim *dsim = host_to_dsim(host);

	drm_bridge_remove(&dsim->bridge);

	return 0;
}

static const struct mipi_dsi_host_ops dsim_host_ops = {
	.attach = dsim_host_attach,
	.detach = dsim_host_detach,
	.transfer = dsim_host_transfer,
};

static void dsim_update_bits(struct exynos7885_dsim *dsim, u32 reg, u32 mask,
			     u32 val)
{
	u32 tmp = readl(dsim->regs + reg);

	writel((tmp & ~mask) | (val & mask), dsim->regs + reg);
}

static int dsim_sw_reset(struct exynos7885_dsim *dsim)
{
	u32 val;

	dsim_update_bits(dsim, DSIM_SWRST, DSIM_SWRST_RESET, DSIM_SWRST_RESET);
	return readl_poll_timeout(dsim->regs + DSIM_SWRST, val,
				  !(val & DSIM_SWRST_RESET), 10, 10000);
}

static void dsim_save_config(struct exynos7885_dsim *dsim)
{
	unsigned int i;

	dsim->clk_ctrl = readl(dsim->regs + DSIM_CLK_CTRL);
	dsim->config = readl(dsim->regs + DSIM_CONFIG);
	dsim->pllctrl = readl(dsim->regs + DSIM_PLLCTRL);
	dsim->pll_ctrl1 = readl(dsim->regs + DSIM_PLL_CTRL1);
	dsim->pll_ctrl2 = readl(dsim->regs + DSIM_PLL_CTRL2);
	dsim->plltmr = readl(dsim->regs + DSIM_PLLTMR);
	for (i = 0; i < ARRAY_SIZE(dsim_link_regs); i++)
		dsim->link[i] = readl(dsim->regs + dsim_link_regs[i]);
	for (i = 0; i < DSIM_NUM_PHY_REGS; i++)
		dsim->phy[i] = readl(dsim->regs + DSIM_PHY_REG(i));
}

/* vendor dsim_disable(), after the DECON has stopped */
static void dsim_bridge_atomic_post_disable(struct drm_bridge *bridge,
					    struct drm_atomic_state *state)
{
	struct exynos7885_dsim *dsim = bridge_to_dsim(bridge);
	u32 val;

	guard(mutex)(&dsim->lock);

	if (dsim->stopped)
		return;

	dsim_update_bits(dsim, DSIM_CLK_CTRL, DSIM_TX_REQUEST_HSCLK, 0);
	if (readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS2, val,
			       !(val & DSIM_CLK_LANE_STATUS), 10, 10000))
		dev_warn(dsim->dev, "HS clock did not stop\n");
	dsim_update_bits(dsim, DSIM_CONFIG, DSIM_LANES_EN, 0);
	usleep_range(400, 500);
	dsim_update_bits(dsim, DSIM_CLK_CTRL, DSIM_LANE_ESCCLK_EN, 0);

	/* D-PHY PLL and escape clock off */
	writel(0, dsim->regs + DSIM_PLL_CTRL1);
	writel(0, dsim->regs + DSIM_PLL_CTRL2);
	dsim_update_bits(dsim, DSIM_CLK_CTRL,
			 DSIM_ESCCLK_EN | DSIM_ESC_PRESCALER, 0xff);
	dsim_update_bits(dsim, DSIM_PLLCTRL, DSIM_PLL_EN, 0);
	if (dsim_sw_reset(dsim))
		dev_warn(dsim->dev, "software reset timed out\n");

	regmap_update_bits(dsim->pmu, PMU_MIPI_PHY_CON, PMU_MIPI_PHY_ENABLE, 0);
	dsim->stopped = true;
}

/* vendor dsim_enable(), with the panel powered and before the DECON starts */
static void dsim_bridge_atomic_pre_enable(struct drm_bridge *bridge,
					  struct drm_atomic_state *state)
{
	struct exynos7885_dsim *dsim = bridge_to_dsim(bridge);
	unsigned int i;
	u32 val;

	guard(mutex)(&dsim->lock);

	if (!dsim->stopped)
		return;

	regmap_update_bits(dsim->pmu, PMU_MIPI_PHY_CON, PMU_MIPI_PHY_ENABLE,
			   PMU_MIPI_PHY_ENABLE);

	dsim_update_bits(dsim, DSIM_SWRST, DSIM_DPHY_RST, 0);
	dsim_update_bits(dsim, DSIM_SWRST, DSIM_DPHY_RST, DSIM_DPHY_RST);
	if (dsim_sw_reset(dsim))
		dev_warn(dsim->dev, "software reset timed out\n");

	/* escape clock, D-PHY timing and PLL */
	dsim_update_bits(dsim, DSIM_CLK_CTRL,
			 DSIM_ESCCLK_EN | DSIM_ESC_PRESCALER, dsim->clk_ctrl);
	for (i = 0; i < DSIM_NUM_PHY_REGS; i++)
		writel(dsim->phy[i], dsim->regs + DSIM_PHY_REG(i));
	writel(dsim->pll_ctrl1, dsim->regs + DSIM_PLL_CTRL1);
	writel(dsim->pll_ctrl2, dsim->regs + DSIM_PLL_CTRL2);
	writel(dsim->pllctrl & ~DSIM_PLL_EN, dsim->regs + DSIM_PLLCTRL);
	writel(dsim->plltmr, dsim->regs + DSIM_PLLTMR);
	dsim_update_bits(dsim, DSIM_PLLCTRL, DSIM_PLL_EN, DSIM_PLL_EN);
	if (readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS3, val,
			       val & DSIM_PLL_STABLE, 10, 10000))
		dev_warn(dsim->dev, "D-PHY PLL did not lock\n");

	/* lanes, lane escape clocks, word clock, then the link setup */
	dsim_update_bits(dsim, DSIM_CONFIG, DSIM_LANES_EN, dsim->config);
	usleep_range(400, 500);
	dsim_update_bits(dsim, DSIM_CLK_CTRL,
			 DSIM_LANE_ESCCLK_EN | DSIM_WORDCLK_EN, dsim->clk_ctrl);
	writel(dsim->clk_ctrl & ~DSIM_TX_REQUEST_HSCLK,
	       dsim->regs + DSIM_CLK_CTRL);
	for (i = 0; i < ARRAY_SIZE(dsim_link_regs); i++)
		writel(dsim->link[i], dsim->regs + dsim_link_regs[i]);

	/* HS clock: the link starts video once the DECON sends frames */
	dsim_update_bits(dsim, DSIM_CLK_CTRL, DSIM_TX_REQUEST_HSCLK,
			 DSIM_TX_REQUEST_HSCLK);
	if (readl_poll_timeout(dsim->regs + DSIM_DPHY_STATUS, val,
			       val & DSIM_TX_READY_HS_CLK, 10, 10000))
		dev_warn(dsim->dev, "HS clock not ready\n");
	dsim->stopped = false;
}

static int dsim_bridge_attach(struct drm_bridge *bridge,
			      enum drm_bridge_attach_flags flags)
{
	struct exynos7885_dsim *dsim = bridge_to_dsim(bridge);

	return drm_bridge_attach(bridge->encoder, dsim->next_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs dsim_bridge_funcs = {
	.attach = dsim_bridge_attach,
	.atomic_pre_enable = dsim_bridge_atomic_pre_enable,
	.atomic_post_disable = dsim_bridge_atomic_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static int dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos7885_dsim *dsim;
	int ret;

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;
	dsim->dev = dev;
	platform_set_drvdata(pdev, dsim);

	ret = devm_mutex_init(dev, &dsim->lock);
	if (ret)
		return ret;

	dsim->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim->regs))
		return PTR_ERR(dsim->regs);

	if (!(readl(dsim->regs + DSIM_LINK_STATUS0) & DSIM_VIDEO_MODE_STATUS))
		return dev_err_probe(dev, -ENODEV,
				     "bootloader DSI link not running\n");
	dsim_save_config(dsim);

	dsim->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "samsung,pmu-syscon");
	if (IS_ERR(dsim->pmu))
		return dev_err_probe(dev, PTR_ERR(dsim->pmu),
				     "failed to get the PMU\n");

	dsim->bridge.funcs = &dsim_bridge_funcs;
	dsim->bridge.of_node = dev->of_node;
	dsim->bridge.type = DRM_MODE_CONNECTOR_DSI;

	dsim->host.dev = dev;
	dsim->host.ops = &dsim_host_ops;

	return mipi_dsi_host_register(&dsim->host);
}

static void dsim_remove(struct platform_device *pdev)
{
	struct exynos7885_dsim *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
}

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos7885-dsim" },
	{ }
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static struct platform_driver dsim_platform_driver = {
	.driver = {
		.name = "exynos7885-dsim",
		.of_match_table = dsim_of_match,
	},
	.probe = dsim_probe,
	.remove = dsim_remove,
};
module_platform_driver(dsim_platform_driver);

MODULE_DESCRIPTION("Samsung Exynos7885 MIPI DSI host driver");
MODULE_LICENSE("GPL");
