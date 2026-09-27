/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __EXYNOS_SCSC_SHARED_RAIL_H
#define __EXYNOS_SCSC_SHARED_RAIL_H

#include <linux/mutex.h>
#include <linux/types.h>

struct scsc_shared_rail_ops {
	int (*set_cp_wakeup)(void *context, bool enable);
	int (*get_option)(void *context, bool *enabled);
	int (*set_option)(void *context, bool enable);
	int (*cp_ready)(void *context, bool *ready);
};

struct scsc_shared_rail {
	const struct scsc_shared_rail_ops *ops;
	void *context;
	struct mutex lock;
	unsigned int users;
	bool saved_option;
	bool faulted;
};

int scsc_shared_rail_init(struct scsc_shared_rail *rail,
			  const struct scsc_shared_rail_ops *ops,
			  void *context);
int scsc_shared_rail_get(struct scsc_shared_rail *rail, bool *need_delay);
int scsc_shared_rail_put(struct scsc_shared_rail *rail);
int scsc_shared_rail_selftest(void);

#endif /* __EXYNOS_SCSC_SHARED_RAIL_H */
