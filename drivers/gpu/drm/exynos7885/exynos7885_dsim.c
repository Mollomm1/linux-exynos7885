// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 MIPI DSI host (DSIM).
 *
 * The bootloader leaves the DSIM link running in video mode. This driver
 * keeps it as is and provides the DSI host, so a panel driver can send
 * commands while video streams, and a DRM bridge between the DPU and the
 * panel. Commands are queued with packet-go held, released, and the packet
 * header FIFO is polled until it drains, as the vendor driver does.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>

#define DSIM_LINK_STATUS0		0x08
#define DSIM_VIDEO_MODE_STATUS		BIT(24)
#define DSIM_VM_LINE_CNT		GENMASK(12, 0)
#define DSIM_PKTHDR			0x58
#define DSIM_PAYLOAD			0x5c
#define DSIM_FIFOCTRL			0x68
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(10)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(8)
#define DSIM_CMD_CONFIG			0x80
#define DSIM_CMD_CONFIG_PKT_GO_EN	BIT(16)

/* The payload FIFO holds 2 KiB; panel commands are far smaller. */
#define DSIM_MAX_PAYLOAD		256
#define DSIM_TIMEOUT_US			50000

struct exynos7885_dsim {
	struct device *dev;
	void __iomem *regs;
	struct mipi_dsi_host host;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	/* Serializes command transfers. */
	struct mutex lock;
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

	if (!(readl(dsim->regs + DSIM_LINK_STATUS0) & DSIM_VIDEO_MODE_STATUS))
		return -EIO;

	ret = dsim_wait_fifo_empty(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR |
				   DSIM_FIFOCTRL_EMPTY_PL_SFR);
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

	ret = dsim_wait_fifo_empty(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR);
	if (ret) {
		dev_err(dsim->dev, "command %#x not sent\n", msg->type);
		return ret;
	}

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

static int dsim_bridge_attach(struct drm_bridge *bridge,
			      enum drm_bridge_attach_flags flags)
{
	struct exynos7885_dsim *dsim = bridge_to_dsim(bridge);

	return drm_bridge_attach(bridge->encoder, dsim->next_bridge, bridge,
				 flags);
}

static const struct drm_bridge_funcs dsim_bridge_funcs = {
	.attach = dsim_bridge_attach,
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
