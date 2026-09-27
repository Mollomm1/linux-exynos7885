// SPDX-License-Identifier: GPL-2.0-only
/* CP-coordinated shared-rail reference management for Exynos7885 WLBT. */

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/string.h>

#include "exynos-scsc-shared-rail.h"

int scsc_shared_rail_init(struct scsc_shared_rail *rail,
			  const struct scsc_shared_rail_ops *ops,
			  void *context)
{
	if (!rail || !ops || !ops->set_cp_wakeup || !ops->get_option ||
	    !ops->set_option ||
	    !ops->cp_ready)
		return -EINVAL;

	memset(rail, 0, sizeof(*rail));
	mutex_init(&rail->lock);
	rail->ops = ops;
	rail->context = context;

	return 0;
}

int scsc_shared_rail_get(struct scsc_shared_rail *rail, bool *need_delay)
{
	bool ready, option_was_enabled;
	int ret, rollback;

	if (!rail || !need_delay)
		return -EINVAL;

	*need_delay = false;
	mutex_lock(&rail->lock);
	if (rail->faulted) {
		ret = -EIO;
		goto out_unlock;
	}
	if (rail->users == UINT_MAX) {
		ret = -EOVERFLOW;
		goto out_unlock;
	}
	if (rail->users) {
		rail->users++;
		ret = 0;
		goto out_unlock;
	}

	ret = rail->ops->get_option(rail->context, &option_was_enabled);
	if (ret)
		goto out_unlock;

	ret = rail->ops->set_cp_wakeup(rail->context, true);
	if (ret)
		goto out_unlock;

	ret = option_was_enabled ? 0 :
		rail->ops->set_option(rail->context, true);
	if (ret) {
		rollback = rail->ops->set_cp_wakeup(rail->context, false);
		if (rollback)
			rail->faulted = true;
		goto out_unlock;
	}

	ret = rail->ops->cp_ready(rail->context, &ready);
	if (ret) {
		rollback = option_was_enabled ? 0 :
			rail->ops->set_option(rail->context, false);
		if (rail->ops->set_cp_wakeup(rail->context, false))
			rollback = -EIO;
		if (rollback)
			rail->faulted = true;
		goto out_unlock;
	}

	rail->users = 1;
	rail->saved_option = option_was_enabled;
	*need_delay = !ready;

out_unlock:
	mutex_unlock(&rail->lock);
	return ret;
}

int scsc_shared_rail_put(struct scsc_shared_rail *rail)
{
	int ret, rollback;

	if (!rail)
		return -EINVAL;

	mutex_lock(&rail->lock);
	if (rail->faulted) {
		ret = -EIO;
		goto out_unlock;
	}
	if (!rail->users) {
		ret = -EINVAL;
		goto out_unlock;
	}
	if (rail->users > 1) {
		rail->users--;
		ret = 0;
		goto out_unlock;
	}

	if (!rail->saved_option) {
		ret = rail->ops->set_option(rail->context, false);
		if (ret)
			goto out_unlock;
	}

	ret = rail->ops->set_cp_wakeup(rail->context, false);
	if (ret) {
		rollback = rail->ops->set_option(rail->context, true);
		if (rollback)
			rail->faulted = true;
		goto out_unlock;
	}

	rail->users = 0;
	rail->saved_option = false;

out_unlock:
	mutex_unlock(&rail->lock);
	return ret;
}

struct scsc_shared_rail_test_context {
	char events[16];
	size_t event_count;
	char fail_event;
	bool ready;
	bool option;
};

static int scsc_shared_rail_test_event(
		struct scsc_shared_rail_test_context *test, char event)
{
	if (test->event_count >= sizeof(test->events))
		return -ENOSPC;
	test->events[test->event_count++] = event;
	return test->fail_event == event ? -EIO : 0;
}

static int scsc_shared_rail_test_wakeup(void *context, bool enable)
{
	struct scsc_shared_rail_test_context *test = context;

	return scsc_shared_rail_test_event(test, enable ? 'C' : 'c');
}

static int scsc_shared_rail_test_option(void *context, bool enable)
{
	struct scsc_shared_rail_test_context *test = context;
	int ret;

	ret = scsc_shared_rail_test_event(test, enable ? 'O' : 'o');
	if (!ret)
		test->option = enable;
	return ret;
}

static int scsc_shared_rail_test_get_option(void *context, bool *enabled)
{
	struct scsc_shared_rail_test_context *test = context;
	int ret;

	ret = scsc_shared_rail_test_event(test, 'S');
	if (!ret)
		*enabled = test->option;
	return ret;
}

static int scsc_shared_rail_test_ready(void *context, bool *ready)
{
	struct scsc_shared_rail_test_context *test = context;
	int ret;

	ret = scsc_shared_rail_test_event(test, 'R');
	if (!ret)
		*ready = test->ready;
	return ret;
}

static const struct scsc_shared_rail_ops scsc_shared_rail_test_ops = {
	.set_cp_wakeup = scsc_shared_rail_test_wakeup,
	.get_option = scsc_shared_rail_test_get_option,
	.set_option = scsc_shared_rail_test_option,
	.cp_ready = scsc_shared_rail_test_ready,
};

int scsc_shared_rail_selftest(void)
{
	struct scsc_shared_rail_test_context test = {
		.ready = false,
	};
	struct scsc_shared_rail rail;
	bool need_delay;
	int ret;

	ret = scsc_shared_rail_init(&rail, &scsc_shared_rail_test_ops, &test);
	if (ret)
		return ret;
	ret = scsc_shared_rail_get(&rail, &need_delay);
	if (ret || !need_delay || rail.users != 1)
		return -EINVAL;
	ret = scsc_shared_rail_get(&rail, &need_delay);
	if (ret || need_delay || rail.users != 2)
		return -EINVAL;
	ret = scsc_shared_rail_put(&rail);
	if (ret || rail.users != 1)
		return -EINVAL;
	ret = scsc_shared_rail_put(&rail);
	if (ret || rail.users || rail.faulted)
		return -EINVAL;
	if (test.event_count != 6 || memcmp(test.events, "SCORoc", 6))
		return -EINVAL;
	if (test.option)
		return -EINVAL;
	if (scsc_shared_rail_put(&rail) != -EINVAL)
		return -EINVAL;

	memset(&test, 0, sizeof(test));
	test.fail_event = 'O';
	ret = scsc_shared_rail_init(&rail, &scsc_shared_rail_test_ops, &test);
	if (ret)
		return ret;
	ret = scsc_shared_rail_get(&rail, &need_delay);
	if (ret != -EIO || rail.users || rail.faulted ||
	    test.event_count != 4 || memcmp(test.events, "SCOc", 4))
		return -EINVAL;

	memset(&test, 0, sizeof(test));
	test.ready = true;
	test.option = true;
	ret = scsc_shared_rail_init(&rail, &scsc_shared_rail_test_ops, &test);
	if (ret)
		return ret;
	ret = scsc_shared_rail_get(&rail, &need_delay);
	if (ret || need_delay || !rail.saved_option || !test.option)
		return -EINVAL;
	ret = scsc_shared_rail_put(&rail);
	if (ret || rail.users || !test.option ||
	    test.event_count != 4 || memcmp(test.events, "SCRc", 4))
		return -EINVAL;

	return 0;
}
