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

#ifdef CONFIG_PM

const char *const pm_states[PM_MODE_MAX] = {
	[PM_MODE_ON]            = "on",
	[PM_MODE_SLEEP]         = "sleep",
	[PM_MODE_STANDBY]       = "standby",
	[PM_MODE_HIBERNATION]   = "hibernation",
};

uint16_t pm_debug_mask = ROM_INF_MASK | ROM_WRN_MASK | ROM_ERR_MASK | ROM_ANY_MASK;

uint32_t g_resume_entry;

void pm_set_debug_mask(uint16_t debug_mask)
{
	pm_debug_mask = debug_mask;
}

struct platform_suspend_ops suspend_ops;

static volatile int user_sel_power_mode = PM_MODE_ON;

static int valid_state(enum suspend_state_t state)
{
	if (user_sel_power_mode == PM_MODE_ON)
		return 0;

	return (user_sel_power_mode == state);
}

int pm_mode_platform_config = PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY | \
                              PM_SUPPORT_HIBERNATION;

/**
 * @brief Select pm modes used on this platform.
 * @note Select modes at init for some modes are not used on some platforms.
 *        This will prevent enter unselect modes.
 * @param select:
 *        @arg select->The selected modes set.
 */
void pm_mode_platform_select(unsigned int select)
{
	pm_mode_platform_config = select;
	if (!(select & (PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY | \
	    PM_SUPPORT_HIBERNATION))) {
		PM_LOGW("slect wrong mode!\n");
		return ;
	}
	PM_LOGN("mode select:%x\n", select);
}

pm_module_power_onoff pm_wlan_power_onoff_cb = NULL;
int pm_wlan_mode_platform_config = PM_SUPPORT_HIBERNATION;

/**
 * @brief Select wlan power modes when enter pm.
 * @note Wlan power on/off calback will called when pm enter select modes.
 * @param wlan_power_cb:
 *        @arg wlan_power_cb->Wlan power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_wlan_power_onoff(pm_module_power_onoff wlan_power_cb, unsigned int select)
{
	if ((select & (PM_SUPPORT_HIBERNATION)) !=
	    (PM_SUPPORT_HIBERNATION)) {
		PM_LOGW("wlan should power off when hibernateion/poweroff!\n");
		return -1;
	}
	pm_wlan_power_onoff_cb = wlan_power_cb;
	pm_wlan_mode_platform_config = select | PM_SUPPORT_HIBERNATION;
	PM_LOGN("wlan mode:%x\n", pm_wlan_mode_platform_config);

	return 0;
}

/** @brief unregister wlan power on/off callback. */
void pm_unregister_wlan_power_onoff(void)
{
	pm_wlan_power_onoff_cb = NULL;
}

/* len, addr */
uint32_t pm_debug_dump_addr[PM_DEBUG_DUMP_NUM][2];

/**
 * @brief Set dump addr and len for debug.
 */
void pm_set_dump_addr(uint32_t addr, uint32_t len, uint32_t idx)
{
	if (idx >= PM_DEBUG_DUMP_NUM) {
		PM_LOGE("only support %d dump\n", PM_DEBUG_DUMP_NUM);
		return ;
	}

	pm_debug_dump_addr[idx][0] = len;
	pm_debug_dump_addr[idx][1] = addr;
}

static int pm_wlan_alive_platform_config = PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY;

int pm_wlan_alive_platform_select(unsigned int select)
{
	if (select & (PM_SUPPORT_HIBERNATION)) {
		PM_LOGW("net can't be alive when hiberantion or poweroff!\n");
		return -1;
	}
	pm_wlan_alive_platform_config = select;
	PM_LOGN("net alive select:%x\n", select);

	return 0;
}

static int pm_test_level = TEST_NONE;

static unsigned long suspend_test_start_time;

void suspend_test_start(void)
{
	suspend_test_start_time = ktime_get();
}

void suspend_test_finish(const char *label)
{
	long nj = ktime_get() - suspend_test_start_time;
	unsigned msec;

	msec = ktime_to_msecs(abs(nj));
	PM_LOGN("%s took %d ms\n", label, msec);
}

int suspend_test(int level)
{
	if (pm_test_level == level) {
		PM_LOGD("suspend debug:%d Return.\n", level);
		return 1;
	}
	return 0;
}

/**
 * @brief Set suspend test level.
 * @param level:
 *        @arg level->Suspend will exit when run up to setted level.
 */
void pm_set_test_level(enum suspend_test_level_t level)
{
	pm_test_level = level;
}

static void dpm_show_time(ktime_t starttime, enum suspend_state_t state, char *info)
{
	ktime_t calltime;

	calltime = ktime_get();
	PM_LOGD("%s of devices complete after %d ms\n", info ? info : "",
	        (int)ktime_to_msecs(calltime - starttime));
}

static LIST_HEAD_DEF(dpm_late_early_list);
static LIST_HEAD_DEF(dpm_noirq_list);

static LIST_HEAD_DEF(dpm_list);
static LIST_HEAD_DEF(dpm_suspended_list);

static int pm_enter_latency;    /* in US */
static int pm_exit_latency;     /* in US */

struct suspend_stats suspend_stats;

void parse_dpm_list(unsigned int idx)
{
	struct list_head *list;
	struct soc_device *dev;
	struct list_head *head;

	if (idx == PM_OP_NORMAL)
		head = &dpm_list;
	else if (idx == PM_OP_NOIRQ)
		head = &dpm_late_early_list;
	else
		return ;

	PM_LOGD("(%p)", head);
	list_for_each(list, head) {
		dev = to_device(list, idx);
		PM_LOGD("-->%s(%p)", dev->name, dev);
	}
	PM_LOGD("\n");
}

static unsigned int initcall_debug_delay_us = 0;

/**
 * @brief Set delay ms in debug mode.
 * @note To prevent mutual interference between devices.
 * @param ms:
 *        @arg ms->The delayed ms between two devices.
 */
void pm_set_debug_delay_ms(unsigned int ms)
{
	initcall_debug_delay_us = ms * 1000;
}

/**
 * check if any wake-up interrupts are pending
 */
int check_wakeup_irqs(void)
{
	return pm_check_wakeup_irqs();
}

/**
 * dpm_suspend_noirq - Execute "noirq suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Prevent device drivers from receiving interrupts and call the "noirq" suspend
 * handlers for all non-sysdev devices.
 */
int dpm_suspend_noirq(enum suspend_state_t state)
{
	int wakeup;
	struct soc_device *dev = NULL;
	ktime_t starttime = ktime_get();
	int error = 0;

	while (!list_empty(&dpm_late_early_list)) {
		dev = to_device(dpm_late_early_list.next, PM_OP_NOIRQ);

		get_device(dev);
		PM_DUMP("suspend noirq dev:%s\n", dev->name);

		error = dev->driver->suspend_noirq(dev, state);
		if (initcall_debug_delay_us > 0) {
			PM_LOGD("%s sleep %d us for debug.\n", dev->name,
			        initcall_debug_delay_us);
			pm_udelay(initcall_debug_delay_us);
		}

		if (error || !PM_IRQ_GET_FLAGS()) {
			PM_LOGE("%s suspend noirq failed! primask:%d\n",
			        dev->name, (int)PM_IRQ_GET_FLAGS());
			put_device(dev);
			break;
		}
		list_move(&dev->node[PM_OP_NOIRQ], &dpm_noirq_list);
		dsb();
		isb();
		put_device(dev);

		wakeup = check_wakeup_irqs();
		if (wakeup && (state >= PM_MODE_HIBERNATION)) {
			PM_REBOOT();
		}
		if (wakeup) {
			error = -1;
			break;
		}
	}

	if (error) {
		suspend_stats.failed_suspend_noirq++;
		if (dev && dev->name)
			memcpy(suspend_stats.failed_devs, dev->name, MAX_DEV_NAME);
		suspend_stats.last_failed_step = TEST_DEVICES;
		dpm_show_time(starttime, state, "noirq");
	}

	return error;
}

/**
 * dpm_suspend - Execute "suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend(enum suspend_state_t state)
{
	struct soc_device *dev = NULL;
	ktime_t starttime = ktime_get();
	int error = 0;

	while (!list_empty(&dpm_list)) {
		dev = to_device(dpm_list.next, PM_OP_NORMAL);

		get_device(dev);
		PM_DUMP("suspend dev:%s\n", dev->name);

		error = dev->driver->suspend(dev, state);
		if (initcall_debug_delay_us > 0) {
			PM_LOGD("sleep %d ms for debug.\n", initcall_debug_delay_us);
			pm_udelay(initcall_debug_delay_us);
		}

		if (error) {
			PM_LOGE("%s suspend failed!\n", dev->name);
			put_device(dev);
			break;
		}
		list_move(&dev->node[PM_OP_NORMAL], &dpm_suspended_list);
		dsb();
		isb();
		put_device(dev);
	}

	if (error) {
		suspend_stats.failed_suspend++;
		if (dev && dev->name)
			memcpy(suspend_stats.failed_devs, dev->name, MAX_DEV_NAME);
		suspend_stats.last_failed_step = TEST_DEVICES;
		dpm_show_time(starttime, state, "suspend");
	}

	return error;
}

/**
 * dpm_resume_noirq - Execute "noirq resume" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Call the "noirq" resume handlers for all devices in dpm_noirq_list and
 * enable device drivers to receive interrupts.
 */
void dpm_resume_noirq(enum suspend_state_t state)
{
	struct soc_device *dev;
	ktime_t starttime = ktime_get();
	int error;

	while (!list_empty(&dpm_noirq_list)) {
		dev = to_device(dpm_noirq_list.next, PM_OP_NOIRQ);

		get_device(dev);
		PM_DUMP("resume noirq dev:%s\n", dev->name);

		list_move(&dev->node[PM_OP_NOIRQ], &dpm_late_early_list);
		dsb();
		isb();

		error = dev->driver->resume_noirq(dev, state);
		if (error) {
			suspend_stats.failed_resume_noirq++;
			PM_LOGE("%s resume noirq failed!\n", dev->name);
		}
		put_device(dev);
	}
	dpm_show_time(starttime, state, "noirq");
}

/**
 * dpm_resume - Execute "resume" callbacks for non-sysdev devices.
 * @state: PM transition of the system being carried out.
 *
 * Execute the appropriate "resume" callback for all devices whose status
 * indicates that they are suspended.
 */
void dpm_resume(enum suspend_state_t state)
{
	struct soc_device *dev;
	ktime_t starttime = ktime_get();
	int error;

	while (!list_empty(&dpm_suspended_list)) {
		dev = to_device(dpm_suspended_list.next, PM_OP_NORMAL);

		get_device(dev);
		PM_DUMP("resume dev:%s\n", dev->name);

		list_move(&dev->node[PM_OP_NORMAL], &dpm_list);
		dsb();
		isb();

		error = dev->driver->resume(dev, state);
		if (error) {
			suspend_stats.failed_resume++;
			PM_LOGE("%s resume failed!\n", dev->name);
		}
		put_device(dev);
	}
	dpm_show_time(starttime, state, "resume");
}

/**
 * suspend_enter - Make the system enter the given sleep state.
 * @state: System sleep state to enter.
 * @wakeup: Returns information that the sleep state should not be re-entered.
 *
 * This function should be called after devices have been suspended.
 */
static int suspend_enter(enum suspend_state_t state)
{
	int wakeup = 0;
	int error;

	if (suspend_ops.prepare) {
		error = suspend_ops.prepare(state);
		if (error)
			goto Platform_finish;
	}

	arch_suspend_disable_irqs();

	wakeup = check_wakeup_irqs();
	if (wakeup && (state >= PM_MODE_HIBERNATION)) {
		PM_REBOOT();
	}
	if (wakeup) {
		error = -1;
		goto Platform_finish;
	}

	__record_dbg_status(PM_SUSPEND_DEVICES | 0x10);
	suspend_test_start();
	error = dpm_suspend_noirq(state);
	if (error) {
//#ifdef CONFIG_PM_DEBUG
		suspend_stats.fail++;
		PM_LOGE("Some devices noirq failed to suspend\n");
		parse_dpm_list(PM_OP_NOIRQ);
//#endif
		goto Resume_noirq_devices;
	}

	suspend_test_finish("suspend noirq devices");
	if (suspend_test(TEST_DEVICES))
		goto Resume_noirq_devices;

	__record_dbg_status(PM_SUSPEND_ENTER);

	__record_dbg_status(PM_SUSPEND_ENTER | 1);
	if (suspend_test(TEST_PLATFORM))
		goto Platform_wake;

	__record_dbg_status(PM_SUSPEND_ENTER | 2);
	wakeup = check_wakeup_irqs();
	if (wakeup && (state >= PM_MODE_HIBERNATION)) {
		PM_REBOOT();
	}

	if (suspend_ops.prepare_late)
		suspend_ops.prepare_late(state);

	if (!(suspend_test(TEST_CORE) || wakeup)) {
		__record_dbg_status(PM_SUSPEND_ENTER | 3);
		suspend_ops.enter(state);
		__record_dbg_status(PM_SUSPEND_ENTER | 4);
	}

Platform_wake:
	if (suspend_ops.wake)
		suspend_ops.wake(state);

Resume_noirq_devices:
	__record_dbg_status(PM_RESUME_DEVICES | 0x10);
	suspend_test_start();
	dpm_resume_noirq(state);
	suspend_test_finish("resume noirq devices");

Platform_finish:
	if (error == PM_WAKEUP_SRC_WLAN) {
		HAL_Wakeup_SetEvent(PM_WAKEUP_SRC_WLAN);
	} else if (wakeup || error) {
		if (!wakeup)
			wakeup = check_wakeup_irqs();
		HAL_Wakeup_SetEvent(wakeup);
	}
	arch_suspend_enable_irqs();

	if (suspend_ops.finish)
		suspend_ops.finish(state);

	return wakeup;
}

/**
 * suspend_devices_and_enter - Suspend devices and enter system sleep state.
 * @state: System sleep state to enter.
 */
int suspend_devices_and_enter(enum suspend_state_t state)
{
	int error;

	__record_dbg_status(PM_EARLY_SUSPEND);
	if (!valid_state(state)) {
		return -1;
	}

	__record_dbg_status(PM_SUSPEND_BEGIN);
	if (suspend_ops.begin) {
		error = suspend_ops.begin(state);
		if (error)
			goto Close;
	}

	__record_dbg_status(PM_SUSPEND_DEVICES);
	suspend_test_start();
	error = dpm_suspend(state);
	if (error) {
#ifdef CONFIG_PM_DEBUG
		suspend_stats.fail++;
#endif
		PM_LOGE("Some devices failed to suspend\n");
		goto Resume_devices;
	}
#ifdef CONFIG_PM_DEBUG
	suspend_stats.success++;
#endif
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES))
		goto Resume_devices;

	OS_ThreadSuspendScheduler();
	__record_dbg_status(PM_SUSPEND_ENTER);
	error = suspend_enter(state);
	OS_ThreadResumeScheduler();

Resume_devices:
	__record_dbg_status(PM_RESUME_DEVICES);
	suspend_test_start();
	dpm_resume(state);
	suspend_test_finish("resume devices");

Close:
	__record_dbg_status(PM_RESUME_END);
	if (suspend_ops.end)
		suspend_ops.end(state);
	__record_dbg_status(PM_RESUME_COMPLETE);

	if (error == PM_WAKEUP_SRC_WLAN) {
		HAL_Wakeup_SetEvent(PM_WAKEUP_SRC_WLAN);
	}

	return error;
}

/**
 * @brief Register a set of system core operations.
 * @note Not use printf for this func maybe called very earlier.
 * @param dev:
 *        @arg dev->Device will be registered.
 * @retval  0 if success or other if failed.
 */
int pm_register_ops(struct soc_device *dev)
{
	struct list_head *hd;
	struct soc_device *dev_c;
	unsigned long flags;
	unsigned int valid;

	if (!dev)
		return -EINVAL;

	valid = dev->node[PM_OP_NORMAL].next || dev->node[PM_OP_NORMAL].prev;
	if (valid) {
		if (!list_empty(&dev->node[PM_OP_NORMAL])) {;
			PM_LOGE("BUG at %s:%d dev(%p):%s!\n", __func__,	\
			        __LINE__, dev, dev->name);
			return -1;
		}
	}
	valid = dev->node[PM_OP_NOIRQ].next || dev->node[PM_OP_NOIRQ].prev;
	if (valid) {
		if (!list_empty(&dev->node[PM_OP_NOIRQ])) {
			PM_LOGE("BUG at %s:%d dev(%p):%s!\n", __func__,	\
			        __LINE__, dev, dev->name);
			return -1;
		}
	}
	if (!dev->driver ||
	    ((!dev->driver->suspend_noirq || !dev->driver->resume_noirq) &&
	    (!dev->driver->suspend || !dev->driver->resume))) {
		PM_LOGE("BUG at %s:%d dev(%p):%s!\n", __func__,	\
		        __LINE__, dev, dev->name);
		return -1;
	}

	if (dev->driver->suspend || dev->driver->resume) {
		if (!dev->driver->suspend || !dev->driver->resume) {
			PM_LOGE("BUG at %s:%d dev(%p):%s!\n", __func__, \
				__LINE__, dev, dev->name);
			return -1;
		}
		list_for_each(hd, &dpm_list) {
			dev_c = to_device(hd, PM_OP_NORMAL);
			if (dev_c == dev) {
				goto next;
			}
		}

		INIT_LIST_HEAD(&dev->node[PM_OP_NORMAL]);
		flags = PM_IRQ_SAVE();
		list_add(&dev->node[PM_OP_NORMAL], &dpm_list);
		PM_IRQ_RESTORE(flags);
	}

next:
	if (dev->driver->suspend_noirq || dev->driver->resume_noirq) {
		if (!dev->driver->suspend_noirq || !dev->driver->resume_noirq) {
			PM_LOGE("BUG at %s:%d dev(%p):%s!\n", __func__, \
				__LINE__, dev, dev->name);
			return -1;
		}
		list_for_each(hd, &dpm_late_early_list) {
			dev_c = to_device(hd, PM_OP_NOIRQ);
			if (dev_c == dev) {
				return -1;
			}
		}

		INIT_LIST_HEAD(&dev->node[PM_OP_NOIRQ]);
		flags = PM_IRQ_SAVE();
		list_add(&dev->node[PM_OP_NOIRQ], &dpm_late_early_list);
		PM_IRQ_RESTORE(flags);
	}
	flags = PM_IRQ_SAVE();
	pm_enter_latency += dev->driver->enter_latency;
	pm_exit_latency += dev->driver->exit_latency;
	PM_IRQ_RESTORE(flags);

	return 0;
}

/**
 * @brief Unregister a set of system core operations.
 * @param dev:
 *        @arg dev->Device will be unregistered.
 * @retval  0 if success or other if failed.
 */
int pm_unregister_ops(struct soc_device *dev)
{
	unsigned long flags;

	if (!dev)
		return -EINVAL;

	if (dev->driver->suspend) {
		if (!dev->node[PM_OP_NORMAL].next || !dev->node[PM_OP_NORMAL].prev) {
			PM_LOGE("BUG at %s:%d dev:%s(%p)!\n", __func__, \
				__LINE__, dev->name, dev);
			return -1;
		}
		if (list_empty(&dev->node[PM_OP_NORMAL])) {
			PM_LOGE("BUG at %s:%d dev:%s(%p)!\n", __func__,	\
			        __LINE__, dev->name, dev);
			return -1;
		}
	}
	if (dev->driver->suspend_noirq) {
		if (!dev->node[PM_OP_NOIRQ].next || !dev->node[PM_OP_NOIRQ].prev) {
			PM_LOGE("BUG at %s:%d dev:%s(%p)!\n", __func__,	\
			        __LINE__, dev->name, dev);
			return -1;
		}
		if (list_empty(&dev->node[PM_OP_NOIRQ])) {
			PM_LOGE("BUG at %s:%d dev:%s(%p)!\n", __func__,	\
			        __LINE__, dev->name, dev);
			return -1;
		}
	}

	flags = PM_IRQ_SAVE();
	pm_enter_latency -= dev->driver->enter_latency;
	if (pm_enter_latency < 0)
		pm_enter_latency = 0;
	pm_exit_latency -= dev->driver->exit_latency;
	if (pm_exit_latency < 0)
		pm_exit_latency = 0;
	if (dev->driver->suspend)
		list_del(&dev->node[PM_OP_NORMAL]);
	if (dev->driver->suspend_noirq)
		list_del(&dev->node[PM_OP_NOIRQ]);
	PM_IRQ_RESTORE(flags);

	return 0;
}

void pm_set_suspend_resume_latency(unsigned int enter_lat, unsigned int exit_lat)
{
	unsigned long flags;

	flags = PM_IRQ_SAVE();
	pm_enter_latency = enter_lat;
	pm_exit_latency = exit_lat;
	PM_IRQ_RESTORE(flags);
}

unsigned int pm_get_suspend_latency(void)
{
	return pm_enter_latency;
}

unsigned int pm_get_resume_latency(void)
{
	return pm_exit_latency;
}

unsigned int pm_get_suspend_resume_latency(void)
{
	return pm_enter_latency + pm_exit_latency;
}

void pm_set_resume_entry(uint32_t entry)
{
	g_resume_entry = entry;
}

void pm_select_mode(enum suspend_state_t state)
{
	user_sel_power_mode = state;
}

/** @brief Show suspend statistic info. */
void pm_stats_show(void)
{
	PM_LOGN("suspend state:\n"
	        "  success:%d\n"
	        "  fail:%d\n"
	        "  failed_suspend:%d\n"
	        "  failed_resume:%d\n"
	        "  last_failed_step:%d\n"
	        "  last_failed_device:%s\n"
	        "  last_wakeup_event:%x\n",
	        suspend_stats.success, suspend_stats.fail,
	        suspend_stats.failed_suspend, suspend_stats.failed_resume,
	        suspend_stats.last_failed_step, suspend_stats.failed_devs,
	        HAL_Wakeup_GetEvent());
}
#else
int check_wakeup_irqs(void)
{
	return 0;
}
#endif
