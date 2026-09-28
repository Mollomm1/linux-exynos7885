/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H
#define __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H

#include <linux/device.h>
#include <linux/types.h>

/* Called from the ABOX parent IRQ; handlers must not sleep. */
typedef void (*exynos7885_abox_ipc_handler_t)(struct device *dev,
						      const u32 *message,
						      void *data);

/* @dev is the Exynos7885 ABOX platform device; @message is its full IPC struct. */
int exynos7885_abox_send_ipc(struct device *dev, const void *message,
			     size_t size);
int exynos7885_abox_register_ipc_handler(struct device *dev,
					 exynos7885_abox_ipc_handler_t handler,
					 void *data);
/* Unregister from process context; waits for any in-flight IRQ callback. */
void exynos7885_abox_unregister_ipc_handler(struct device *dev,
					    exynos7885_abox_ipc_handler_t handler);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H */
