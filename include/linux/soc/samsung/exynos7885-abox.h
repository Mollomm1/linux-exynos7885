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
	EXYNOS7885_ABOX_IPC_PCM_CAPTURE = 3,
};

enum exynos7885_abox_pcm_message_type {
	EXYNOS7885_ABOX_PCM_OPEN = 11,
	EXYNOS7885_ABOX_PCM_CLOSE = 12,
	EXYNOS7885_ABOX_PCM_HW_PARAMS = 14,
	EXYNOS7885_ABOX_PCM_HW_FREE = 15,
	EXYNOS7885_ABOX_PCM_PREPARE = 16,
	EXYNOS7885_ABOX_PCM_TRIGGER = 17,
	EXYNOS7885_ABOX_PCM_POINTER = 18,
	EXYNOS7885_ABOX_PCM_SET_BUFFER = 20,
};

enum exynos7885_abox_uaif3_format {
	EXYNOS7885_ABOX_UAIF3_I2S,
	EXYNOS7885_ABOX_UAIF3_DSP_A,
};

/*
 * Called from the ABOX parent IRQ; handlers must not sleep. An RDMA0
 * buffer-empty or WDMA1 buffer-full interrupt arrives as a PCM_POINTER
 * message, for playback or capture, with pointer 0.
 */
typedef void (*exynos7885_abox_ipc_handler_t)(struct device *dev,
						      const u32 *message,
						      void *data);

/* @dev is the Exynos7885 ABOX platform device; @message is its full IPC struct. */
int exynos7885_abox_send_ipc(struct device *dev, const void *message,
			     size_t size);
int exynos7885_abox_send_pcm(struct device *dev, u32 channel, u32 type,
			     u32 param0, u32 param1, u32 param2);
/* Same, for the playback (RDMA) or capture (WDMA) PCM task. */
int exynos7885_abox_send_pcm_stream(struct device *dev, bool capture,
				    u32 channel, u32 type, u32 param0,
				    u32 param1, u32 param2);
int exynos7885_abox_boot(struct device *dev);
int exynos7885_abox_shutdown(struct device *dev);
int exynos7885_abox_get_pcm_buffer(struct device *dev, void **area,
				   phys_addr_t *phys, size_t *size);
int exynos7885_abox_map_pcm_buffer(struct device *dev);
int exynos7885_abox_unmap_pcm_buffer(struct device *dev);
int exynos7885_abox_sync_pcm_buffer(struct device *dev, size_t offset,
				    size_t size);
int exynos7885_abox_wait_rdma_idle(struct device *dev, unsigned int channel,
				   unsigned int timeout_us);
/* Byte position of RDMA @channel in its ring; safe from the IPC handler. */
ssize_t exynos7885_abox_rdma_pointer(struct device *dev, unsigned int channel,
				     size_t buffer_bytes, size_t period_bytes);
int exynos7885_abox_uaif3_set_fmt(struct device *dev, unsigned int format,
				  bool invert_bclk, bool invert_frame,
				  bool abox_master);
int exynos7885_abox_uaif3_hw_params(struct device *dev, unsigned int rate,
				    unsigned int width,
				    unsigned int channels);
int exynos7885_abox_uaif3_set_enabled(struct device *dev, bool enable);
/* Same as the uaif3 helpers, for UAIF @n (0: headphone codec, 3: amps). */
int exynos7885_abox_uaif_set_fmt(struct device *dev, unsigned int n,
				 unsigned int format, bool invert_bclk,
				 bool invert_frame, bool abox_master);
int exynos7885_abox_uaif_hw_params(struct device *dev, unsigned int n,
				   unsigned int rate, unsigned int width,
				   unsigned int channels);
int exynos7885_abox_uaif_set_enabled(struct device *dev, unsigned int n,
				     bool enable);
/* Move a running stream from UAIF @from to UAIF @to, both configured. */
int exynos7885_abox_uaif_switch(struct device *dev, unsigned int from,
				unsigned int to);
/* The input of UAIF @n, routed to WDMA1. */
int exynos7885_abox_uaif_mic_hw_params(struct device *dev, unsigned int n,
				       unsigned int rate, unsigned int width,
				       unsigned int channels);
int exynos7885_abox_uaif_set_mic_enabled(struct device *dev, unsigned int n,
					 bool enable);
/* The WDMA1 capture ring, mapped at its own IOVA. */
int exynos7885_abox_get_capture_buffer(struct device *dev, void **area,
				       size_t *size);
int exynos7885_abox_map_capture_buffer(struct device *dev);
int exynos7885_abox_unmap_capture_buffer(struct device *dev);
int exynos7885_abox_wait_wdma_idle(struct device *dev, unsigned int channel,
				   unsigned int timeout_us);
/* Byte position of WDMA @channel in its ring; safe from the IPC handler. */
ssize_t exynos7885_abox_wdma_pointer(struct device *dev, unsigned int channel,
				     size_t buffer_bytes, size_t period_bytes);
int exynos7885_abox_register_ipc_handler(struct device *dev,
					 exynos7885_abox_ipc_handler_t handler,
					 void *data);
/* Unregister from process context; waits for any in-flight IRQ callback. */
void exynos7885_abox_unregister_ipc_handler(struct device *dev,
					    exynos7885_abox_ipc_handler_t handler);

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS7885_ABOX_H */
