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

#include "rom/libc/string.h"

#include "rom/rom_debug.h"
#include "rom/driver/chip/hal_wakeup.h"
#include "rom/driver/chip/hal_ccm.h"

#include "rom/pm/pm.h"
#include "_pm_define.h"
#include "pm_i.h"
#include "port.h"

#ifdef CONFIG_PM_WAKELOCKS

/**
 * A wake lock is a mechanism to indicate that your application needs to have
 * the device stay on. Any application can using a WakeLock.
 * Call pm_wake_lock to acquire the wake lock and force the device to stay on.
 *
 * Call pm_wake_unlock when you are done and don't need the lock anymore. It is
 * very important to do this as soon as possible to avoid running down the
 * device's battery excessively.
 */

#define wl_to_device(ptr_module) \
	__containerof(ptr_module, struct wakelock, node)

static OS_Semaphore_t wakelocks_wait;

/* Default and minimum suspend timeout in milliseconds. */
static unsigned int wakelocks_default_suspend_to = 500;

static LIST_HEAD_DEF(wakelocks_list);
static OS_Timer_t _wl_timer;
struct wakelock *_wl_timer_arg;
static uint32_t _wl_timer_expires;

/**
 * @brief Set delay ms in debug mode.
 * @note To prevent mutual interference between devices.
 * @param ms:
 *        @arg ms->The delayed ms between two suspend.
 */
void pm_wakelocks_set_suspend_timeout(unsigned int ms)
{
	if (ms < WAKELOCKS_DEF_SUSPEND_TO_MIN) {
		PM_LOGE("%s set timeout too short!\n", __func__);
		return;
	}
	wakelocks_default_suspend_to = ms;
}

unsigned int pm_wakelocks_get_suspend_timeout(void)
{
	return wakelocks_default_suspend_to;
}

/**
 * _wakelock_active_timer - Add wakelock timer to list and update wakelock timer.
 * @wl: Wakeup lock to add.
 * @msec: Anticipated event processing time (in milliseconds).
 *
 */
static void _wakelock_active_timer(struct wakelock *wl, unsigned int msec)
{
	uint32_t expires;

	PM_ABORT(!wl);

	if (!msec) {
		PM_LOGE("%s,%d no msec!\n", __func__, __LINE__);
		return;
	}

	expires = OS_GetTicks() + OS_MSecsToTicks(msec);
	if (expires == OS_WAIT_FOREVER)
		expires++;

	wl->expires = expires;

	if (_wl_timer_expires == OS_WAIT_FOREVER)
		return;

	if (OS_TimeAfter(expires, _wl_timer_expires)) {
		OS_TimerChangePeriod(&_wl_timer, msec);
		_wl_timer_arg = wl;
		_wl_timer_expires = expires;
		PM_LOGD("%s,%d update msec to %d\n", __func__, __LINE__, expires);
	}
}

/**
 * _wakelock_stay_awake - Notify the PM core of a forever wakeup lock.
 * @wl: Wakeup lock to active.
 *
 */
static void _wakelock_stay_awake(struct wakelock *wl)
{
	if (!wl)
		return;

	wl->ref++;
	OS_TimerStop(&_wl_timer);
	_wl_timer_expires = OS_WAIT_FOREVER;
	PM_LOGD("%s,%d stop timer\n", __func__, __LINE__);
}

static uint32_t _wakelock_update(void)
{
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
			goto out;
			PM_LOGD("%s none %s(%p) ref:%d\n", __func__, wl->name, wl, wl->ref);
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
		OS_SemaphoreRelease(&wakelocks_wait);
		goto out;
	}

	if (OS_TimeAfter(expires, OS_GetTicks())) {

		PM_ABORT(!wl_max);
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

int pm_wake_lock(struct wakelock *wl, enum pm_wakelock_t type, uint32_t timeout_ms)
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
	if (type >= PM_WKL_WAIT_NUM) {
		PM_LOGE("%s unkonw type:%d !\n", __func__, type);
		return -EINVAL;
	} else if (type == PM_WKL_WAIT_TIMEOUT && !timeout_ms) {
		PM_LOGE("%s no timeout!\n", __func__);
		return -EINVAL;
	} else if (type == PM_WKL_WAIT_FOREVER && timeout_ms) {
		PM_LOGW("%s wait forever ignore timeout!\n", __func__);
		timeout_ms = 0;
	}

	flags = arch_irq_save();

	list_for_each(list, head) {
		_wl = wl_to_device(list);
		if (_wl == wl)
			break;
	}
	if (_wl != wl) {
		INIT_LIST_HEAD(&wl->node);
		list_add(&wl->node, list);
		PM_LOGD("%s add new %s(%p)\n", __func__, wl->name, wl);
	} else {
		PM_LOGD("%s update old %s(%p)\n", __func__, wl->name, wl);
	}
	if (type == PM_WKL_WAIT_TIMEOUT) {
		_wakelock_active_timer(wl, timeout_ms);
	} else if (type == PM_WKL_WAIT_FOREVER) {
		_wakelock_stay_awake(wl);
	}

	arch_irq_restore(flags);

	return ret;
}

int pm_wake_unlock(struct wakelock *wl, enum pm_wakelock_t type)
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
	} else if (type == PM_WKL_WAIT_FOREVER) {
		--wl->ref; /* only minus ref, will del in update if expires timeout */
	} else {
		PM_LOGD("%s no del %s(%p) ref:%d exp:%d", __func__, wl->name, wl, wl->ref, wl->expires);
	}

out:
	arch_irq_restore(flags);
	_wakelock_update();

	return ret;
}

uint32_t pm_wait_wakelocks(void)
{
	uint32_t ret = 0;

	OS_SemaphoreWait(&wakelocks_wait, OS_WAIT_FOREVER);

	return ret;
}

void pm_show_wakelocks(void)
{
	struct wakelock *wl;
	struct list_head *list;
	struct list_head *head = &wakelocks_list;
	unsigned long flags;

	flags = arch_irq_save();

	PM_LOGD("wakelocks:");
	list_for_each(list, head) {
		wl = wl_to_device(list);
		PM_LOGD(" -->%s(%p) ref:%d exp:%d", wl->name, wl, wl->ref, wl->expires);
	}
	PM_LOGD("\n");

	arch_irq_restore(flags);
}

void pm_wakelocks_init(void)
{
	OS_SemaphoreCreate(&wakelocks_wait, 0, OS_SEMAPHORE_MAX_COUNT);
	OS_TimerCreate(&_wl_timer, OS_TIMER_ONCE,
                       _wakelock_timer_cb, NULL, OS_WAIT_FOREVER);
}

void pm_wakelocks_deinit(void)
{
	OS_TimerDelete(&_wl_timer);
	OS_SemaphoreDelete(&wakelocks_wait);
}

#else

#endif /* CONFIG_PM_WAKELOCKS */
