/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>

#include "kernel/os/os.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_ccm.h"

#include "pm/pm.h"
#include "_pm_define.h"
#include "pm_i.h"
#include "port.h"

#ifdef CONFIG_PM_WAKELOCKS

#define wl_to_device(ptr_module) \
	__containerof(ptr_module, struct wakelock, node)

extern int __pm_wake_lock(struct wakelock *wl, enum pm_wakelock_t type, uint32_t timeout_ms);

extern OS_Semaphore_t wakelocks_wait;

extern struct list_head wakelocks_list;
extern OS_Timer_t _wl_timer;
extern struct wakelock *_wl_timer_arg;
static uint8_t _wl_null = 1;
static uint8_t _wl_touch;

static int _pm_wake_unlock(struct wakelock *wl, enum pm_wakelock_t type);

static uint32_t _wakelock_update(void)
{
	extern uint32_t _wl_timer_expires;

	uint32_t ret = 0;
	uint32_t expires = OS_WAIT_FOREVER;
	struct wakelock *wl = NULL, *wl_max = NULL;
	struct list_head *pos, *n;
	struct list_head *head = &wakelocks_list;
	uint32_t now = OS_GetTicks();
	unsigned long flags;
	uint32_t ms, update_flg = 0;

	flags = arch_irq_save();
	list_for_each_safe(pos, n, head) {
		wl = wl_to_device(pos);
		if (wl->ref) {
			ret = 2;
			PM_LOGD("%s none %s(%p) ref:%d\n", __func__, wl->name, wl, wl->ref);
			goto out;
		} else if (OS_TimeAfterEqual(now, wl->expires)) {
			list_del(&wl->node);
			PM_LOGD("%s del %s(%p)\n", __func__, wl->name, wl);
			continue;
		} else if ((expires == OS_WAIT_FOREVER) || (OS_TimeBefore(expires, wl->expires))) {
			expires = wl->expires;
			wl_max = wl;
		}
	}

	if (!wl || (expires == OS_WAIT_FOREVER)) {
		PM_LOGD("%s,%d null\n", __func__, __LINE__);
		OS_SemaphoreWait(&wakelocks_wait, 0);
		_wl_null = 1;
		OS_SemaphoreRelease(&wakelocks_wait);
		goto out;
	}

	if (OS_TimeAfter(expires, OS_GetTicks())) {
		ms = expires - OS_GetTicks();
		update_flg = 1;
		_wl_timer_arg = wl_max;
		_wl_timer_expires = expires;
		ret = 1;
		PM_LOGD("%s to %s(%p) expires:%d\n", __func__, wl_max->name, wl_max, expires);
	} else {
		PM_LOGD("%s faid now:%d expires:%d\n", __func__, OS_GetTicks(), expires);
	}

out:
	if (ret > 0)
		_wl_null = 0;
	arch_irq_restore(flags);
	if (update_flg)
		OS_TimerChangePeriod(&_wl_timer, ms);

	return ret;
}

static void _wakelock_timer_cb(void *arg)
{
	unsigned long flags;

	PM_LOGD("%s ref:%d\n", __func__, _wl_timer_arg->ref);

	flags = arch_irq_save();
	if (_wl_timer_arg && !_wl_timer_arg->ref) {
		list_del(&_wl_timer_arg->node);
		PM_LOGD("%s del %s(%p)\n", __func__, _wl_timer_arg->name, _wl_timer_arg);
	}
	arch_irq_restore(flags);

	_wakelock_update();
}

int pm_wake_lock(struct wakelock *wl)
{
	int ret = __pm_wake_lock(wl, PM_WKL_WAIT_FOREVER, 0);

	_wakelock_update();

	return ret;
}

int pm_wake_lock_timeout(struct wakelock *wl, uint32_t timeout_ms)
{
	int ret;

	if (timeout_ms == 0) {
		return _pm_wake_unlock(wl, PM_WKL_WAIT_TIMEOUT);
	}
	ret = __pm_wake_lock(wl, PM_WKL_WAIT_TIMEOUT, timeout_ms);

	_wakelock_update();

	return ret;
}

static int _pm_wake_unlock(struct wakelock *wl, enum pm_wakelock_t type)
{
	int ret = 0;
	struct wakelock *_wl = NULL;
	struct list_head *list;
	struct list_head *head = &wakelocks_list;
	unsigned long flags;

	if (!wl) {
		PM_LOGE("%s null wakelock!\n", __func__);
		return -EINVAL;
	}

	flags = arch_irq_save();

	list_for_each(list, head) {
		_wl = wl_to_device(list);
		if (_wl == wl)
			break;
	}
	if (!_wl) {
		PM_LOGN("%s unknow wakelock!\n", __func__);
		goto out;
	}
	if (type == PM_WKL_WAIT_TIMEOUT && !wl->ref) { /* to and no fv */
		list_del(&wl->node);
		PM_LOGD("%s del %s(%p)\n", __func__, wl->name, wl);
	} else if (type == PM_WKL_WAIT_FOREVER && wl->ref) {
		--wl->ref; /* only minus ref, will del in update if expires timeout */
		PM_LOGD("%s ref-- %s(%p) ref:%d exp:%d\n", __func__, wl->name, wl, wl->ref, wl->expires);
	} else {
		PM_LOGD("%s no del %s(%p) ref:%d exp:%d\n", __func__, wl->name, wl, wl->ref, wl->expires);
	}

out:
	arch_irq_restore(flags);
	_wakelock_update();

	return ret;
}

int pm_wake_unlock(struct wakelock *wl)
{
	return _pm_wake_unlock(wl, PM_WKL_WAIT_FOREVER);
}

void pm_wakelocks_touch(void)
{
	_wl_touch = 1;
	OS_SemaphoreWait(&wakelocks_wait, 0);
	OS_SemaphoreRelease(&wakelocks_wait);
}

uint32_t pm_wakelocks_is_touched(void)
{
	return _wl_touch;
}

uint32_t pm_wakelocks_is_active(void)
{
	return !_wl_null || _wl_touch;
}

uint32_t pm_wakelocks_wait(uint32_t timeout)
{
	uint32_t ret = 0;

	while (!_wl_null) {
		ret = OS_SemaphoreWait(&wakelocks_wait, timeout);
		if (_wl_touch) {
			_wl_touch = 0;
			ret = OS_FAIL;
			break;
		}
		if (ret == OS_E_TIMEOUT)
			break;
	}

	return ret;
}

void pm_wakelocks_init(void)
{
	OS_SemaphoreCreate(&wakelocks_wait, 0, OS_SEMAPHORE_MAX_COUNT);
	OS_TimerCreate(&_wl_timer, OS_TIMER_ONCE,
	               _wakelock_timer_cb, NULL, OS_WAIT_FOREVER);
}
#else

#endif /* CONFIG_PM_WAKELOCKS */
