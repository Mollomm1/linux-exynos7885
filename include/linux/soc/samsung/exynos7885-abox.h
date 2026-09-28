/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H
#define __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H

#include <linux/device.h>
#include <linux/types.h>

/* @dev is the Exynos7885 ABOX platform device; @message is its full IPC struct. */
int exynos7885_abox_send_ipc(struct device *dev, const void *message,
			     size_t size);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H */
