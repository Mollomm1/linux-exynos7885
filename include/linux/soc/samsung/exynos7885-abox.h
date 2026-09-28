/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H
#define __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H

#include <linux/device.h>
#include <linux/types.h>

#define EXYNOS7885_ABOX_IPC_WORDS	191
#define EXYNOS7885_ABOX_IPC_SIZE	(EXYNOS7885_ABOX_IPC_WORDS * sizeof(u32))

enum exynos7885_abox_ipc_id {
	EXYNOS7885_ABOX_IPC_SYSTEM = 1,
	EXYNOS7885_ABOX_IPC_PCM_PLAYBACK = 2,
};

enum exynos7885_abox_pcm_message_type {
	EXYNOS7885_ABOX_PCM_HW_PARAMS = 14,
	EXYNOS7885_ABOX_PCM_HW_FREE = 15,
	EXYNOS7885_ABOX_PCM_PREPARE = 16,
	EXYNOS7885_ABOX_PCM_TRIGGER = 17,
	EXYNOS7885_ABOX_PCM_POINTER = 18,
	EXYNOS7885_ABOX_PCM_SET_BUFFER = 20,
};

/* Called from the ABOX parent IRQ; handlers must not sleep. */
typedef void (*exynos7885_abox_ipc_handler_t)(struct device *dev,
						      const u32 *message,
						      void *data);

/* @dev is the Exynos7885 ABOX platform device; @message is its full IPC struct. */
int exynos7885_abox_send_ipc(struct device *dev, const void *message,
			     size_t size);
int exynos7885_abox_send_pcm(struct device *dev, u32 channel, u32 type,
			     u32 param0, u32 param1, u32 param2);
int exynos7885_abox_register_ipc_handler(struct device *dev,
					 exynos7885_abox_ipc_handler_t handler,
					 void *data);
/* Unregister from process context; waits for any in-flight IRQ callback. */
void exynos7885_abox_unregister_ipc_handler(struct device *dev,
					    exynos7885_abox_ipc_handler_t handler);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H */
