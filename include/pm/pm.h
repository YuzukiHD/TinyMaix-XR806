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

#ifndef __XRADIO_PM_H
#define __XRADIO_PM_H

#include <stddef.h>
#include <stdint.h>
#include "sys/list.h"
#include "driver/chip/hal_prcm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Defined all supported low power state.
 * @note:
 *       PM_MODE_ON is used for test.
 *       PM_MODE_SLEEP is used for devices wakeup system. In this mode CPU is
 *         in WFI mode and running at low frequency, all devices are powered on.
 *         set to work mode if you want this device to wakeup system, or else
 *         disable this device to save power.
 *       PM_MODE_STANDBY is used for network or some special wakeup sources to
 *         wakeup system. In this mode CPU and all devices has been powered off,
 *         network can work normally and can wakeup system by received data from
 *         network. Also some special wakeup sources like wakeup timer or IO can
 *         wakeup system if you set this wakeup sources properly.
 *       PM_MODE_HIBERNATION is used for some special wakeup sources to wakeup system.
 *         System will restartup when wakeup. In this mode CPU and all devices
 *         has been powered off beside network. Only some special wakeup sources
 *         can startup system, and can get wakeup event at startup.
 */
enum suspend_state_t {
	PM_MODE_ON              = 0,
	PM_MODE_SLEEP           = 1,
	PM_MODE_STANDBY         = 2,
	PM_MODE_HIBERNATION     = 3,
	PM_MODE_MAX             = 4,
};

#define PM_MODE_MAGIC           (0x7FF20000)

/** @brief Platform pm mode support. */
#define PM_SUPPORT_SLEEP        (1<<PM_MODE_SLEEP)
#define PM_SUPPORT_STANDBY      (1<<PM_MODE_STANDBY)
#define PM_SUPPORT_HIBERNATION  (1<<PM_MODE_HIBERNATION)

/** @brief Suspend test levels. */
enum suspend_test_level_t {
	TEST_NONE,              /* keep first */
	TEST_CORE,
	TEST_PLATFORM,
	TEST_DEVICES,
	__TEST_AFTER_LAST       /* keep last */
};

/** @brief Wlan power on/off callback. */
typedef int (*pm_module_power_onoff)(unsigned int enable);

enum pm_op_t {
	PM_OP_NORMAL = 0,
	PM_OP_NOIRQ,
	PM_OP_NUM,
};

struct soc_device;

/**
 * @brief The basic device driver structure.
 * @suspend:    Called to put the device to sleep mode. Usually to a
 *              low power state.
 * @resume:     Called to bring a device from sleep mode.
 * @suspend_noirq:
 *  1. The device will be in a low-power state after suspend_noirq() has
 *      returned successfully.
 *  2. If the device can generate system wakeup signals and is enabled to wake
 *      up the system, it should be configured to do so at that time.
 * @resume_noirq:
 *  1. Handle device wakeup signal if device can generate system wakeup signals.
 *  2. Resume device to work mode.
 */
struct soc_device_driver {
	const char *name;               /* name of the device driver. */
	unsigned int enter_latency;     /* in US */
	unsigned int exit_latency;      /* in US */
	/*const struct soc_device *dev;*/

	int (*suspend)(struct soc_device *dev, enum suspend_state_t state);
	int (*resume)(struct soc_device *dev, enum suspend_state_t state);

	int (*suspend_noirq)(struct soc_device *dev, enum suspend_state_t state);
	int (*resume_noirq)(struct soc_device *dev, enum suspend_state_t state);
};

/**
 * @brief The basic device structure.
 * @note For devices on custom boards, as typical of embedded and SOC based
 *   hardware, You uses platform_data to point to board-specific structures
 *   describing devices and how they are wired.
 */
struct soc_device {
	struct list_head node[PM_OP_NUM];
	unsigned int ref;

	const char *name;               /* initial name of the device */

	const struct soc_device_driver *driver; /* which driver has allocated this device */
	void *platform_data;                    /* Platform specific data, device core doesn't touch it */
};

#ifdef CONFIG_PM_WAKELOCKS

#define WAKELOCKS_DEF_SUSPEND_TO_MIN    200

enum pm_wakelock_t {
	PM_WKL_WAIT_FOREVER = 0,
	PM_WKL_WAIT_TIMEOUT,
	PM_WKL_WAIT_NUM,
};

/**
 * @brief The wakelock structure.
 * @note Wakelock is used for block system to suspend, a wakelock can use as
 *       PM_WKL_WAIT_FOREVER mode and PM_WKL_WAIT_TIMEOUT mode toghter.
 *       If wakelock's ref is not 0, it meas someone has took this lock as
 *       FOREVER mode, this wakelock is active. This wakelock is also active if
 *       it is locked as timeout mode before timeout, A wakelock is not active
 *       only it's ref reduce to 0 and after timeout. System will not suspend
 *       if any of wakelocks active.
 */
struct wakelock {
	char                    *name;

	/* pm core use, keep 0 when first used */
	struct list_head        node;
	uint32_t                expires;
	uint16_t                ref;
	uint16_t                reserve;
};
#endif

#ifdef CONFIG_PM
/**
 * @brief Register a set of system core operations.
 * @note Not use printf for this func maybe called very earlier.
 * @param dev:
 *        @arg dev->Device will be registered.
 * @retval  0 if success or other if failed.
 */
extern int pm_register_ops(struct soc_device *dev);

/**
 * @brief Unregister a set of system core operations.
 * @param dev:
 *        @arg dev->Device will be unregistered.
 * @retval  0 if success or other if failed.
 */
extern int pm_unregister_ops(struct soc_device *dev);

/**
 * @brief Set a magin to synchronize with net.
 */
extern void pm_set_sync_magic(void);

/**
 * @brief Set system to a lowpower mode with timeout.
 * @param state:
 *        @arg state->The lowpower mode will enter.
 * @param tmo:
 *        @arg tmo->Timeout based on mS.
 * @retval  0 if success or other if failed.
 */
extern int pm_enter_mode_timeout(enum suspend_state_t state, uint32_t tmo);

/**
 * @brief Set system to a lowpower mode.
 * @param state:
 *        @arg state->The lowpower mode will enter.
 * @retval  0 if success or other if failed.
 */
extern int pm_enter_mode(enum suspend_state_t state);

/**
 * @brief Abort suspend.
 */
extern void pm_suspend_abort(void);

/**
 * @brief Get system pm mode.
 * @retval  PM_MODE_ON: if runing or other if during suspending or resumeing.
 */
extern enum suspend_state_t pm_get_mode(void);

/**
 * @brief Initialize the PM-related part of a device object.
 * @note not use printf for this fun is called very earlier.
 * @retval  0 if success or other if failed.
 */
extern int pm_init(void);

/**
 * @brief Alloc resources.
 */
extern void pm_start(void);

/**
 * @brief Releas resources.
 */
extern void pm_stop(void);

/**
 * @brief Set dump addr and len for debug.
 */
extern void pm_set_dump_addr(uint32_t addr, uint32_t len, uint32_t idx);

/**
 * @brief Set suspend test level.
 * @param level:
 *        @arg level->Suspend will exit when run up to setted level.
 */
extern void pm_set_test_level(enum suspend_test_level_t level);

/**
 * @brief Set delay ms in debug mode.
 * @note To prevent mutual interference between devices.
 * @param ms:
 *        @arg ms->The delayed ms between two devices.
 */
extern void pm_set_debug_delay_ms(unsigned int ms);

/** @brief Show suspend statistic info. */
extern void pm_stats_show(void);

/**
 * @brief Select pm modes used on this platform.
 * @note Select modes at init for some modes are not used on some platforms.
 *        This will prevent enter unselected modes.
 * @param select:
 *        @arg select->The selected modes set.
 */
extern void pm_mode_platform_select(unsigned int select);

#ifdef CONFIG_WLAN
/**
 * @brief register wlan power on/off callback.
 * @note Wlan power on/off calback will be called when pm enter select modes.
 * @param wlan_power_cb:
 *        @arg wlan_power_cb->Wlan power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
extern int pm_register_wlan_power_onoff(pm_module_power_onoff wlan_power_cb,
                                        unsigned int select);

/** @brief unregister wlan power on/off callback. */
extern void pm_unregister_wlan_power_onoff(void);
#endif

#ifdef CONFIG_BT
/**
 * @brief Select bt power modes when enter pm.
 * @note bt power on/off calback will called when pm enter select modes.
 * @param bt_power_cb:
 *        @arg bt_power_cb->bt power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_bt_power_onoff(pm_module_power_onoff bt_power_cb, unsigned int select);

/** @brief unregister bt power on/off callback. */
void pm_unregister_bt_power_onoff(void);
#endif

#ifdef CONFIG_BLE
/**
 * @brief Select ble power modes when enter pm.
 * @note ble power on/off calback will called when pm enter select modes.
 * @param ble_power_cb:
 *        @arg ble_power_cb->ble power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_ble_power_onoff(pm_module_power_onoff ble_power_cb, unsigned int select);

/** @brief unregister ble power on/off callback. */
void pm_unregister_ble_power_onoff(void);
#endif

/**
 * check if any wake-up interrupts are pending
 */
extern int pm_check_wakeup_irqs(void);

extern void pm_set_suspend_resume_latency(unsigned int enter_lat, unsigned int exit_lat);
extern unsigned int pm_get_suspend_latency(void);
extern unsigned int pm_get_resume_latency(void);
extern unsigned int pm_get_suspend_resume_latency(void);

extern int pm_test(void);
extern void pm_set_debug_mask(uint16_t debug_mask);
void pm_standby_sram_retention_only(uint32_t sramN);

#ifdef CONFIG_PM_WAKELOCKS
/**
 * @brief Lock wl.
 * @param[in] wl Wakelock to lock, assign wl's name and set other members
 *            to 0 if first use this wl.
 * @retval 0 if success or other if failed.
 *
 * @note system will not goto suspend before unlock this wl, support lock many
 *       times, Use FOREVER mode, TIMEOUT mode can also used at the same time,
 *       see struct wakelock note.
 */
int pm_wake_lock(struct wakelock *wl);

/**
 * @brief Unlock wl.
 * @param[in] wl Wakelock to unlock.
 * @retval 0 if success or other if failed.
 *
 * @note Unlock wl locked by pm_wake_lock, must used in pairs with pm_wake_lock.
 */
int pm_wake_unlock(struct wakelock *wl);

/**
 * @brief Update wl's timeout.
 * @param[in] wl Wakelock to operated, assign wl's name and set other members
 *            to 0 if first use this wl.
 * @param[in] timeout Based on ms.
 * @retval 0 if success or other if failed.
 *
 * @note System will not goto suspend before timeout. Use TIMEOUT mode, FOREVER
 *       mode can also used at the same time, see struct wakelock note. Set
 *       timeout to 0 to disable this wl's timeout mode.
 */
int pm_wake_lock_timeout(struct wakelock *wl, uint32_t timeout);

/* @brief Show all wakelocks info. */
void pm_wakelocks_show(void);
#endif

#else /* CONFIG_PM */

static inline int pm_register_ops(struct soc_device *dev) { return 0; }
static inline int pm_unregister_ops(struct soc_device *dev) { return 0; }
static inline void pm_set_sync_magic(void) { ; }
static inline int pm_enter_mode_timeout(enum suspend_state_t state, uint32_t tmo) { return 0; }
static inline int pm_enter_mode(enum suspend_state_t state) { return 0; }
static inline void pm_suspend_abort(void) { ; }
static inline enum suspend_state_t pm_get_mode(void) { return PM_MODE_ON; }
static inline int pm_init(void) { return 0; }
static inline void pm_start(void) { ; }
static inline void pm_stop(void) { ; }
static inline void pm_set_test_level(enum suspend_test_level_t level) { ; }
static inline void pm_set_debug_delay_ms(unsigned int ms) { ; }
static inline void pm_stats_show(void) { ; }
static inline void pm_mode_platform_select(unsigned int select) { ; }
static inline int pm_register_wlan_power_onoff(pm_module_power_onoff wlan_power_cb,
                                               unsigned int select) { return 0; }
static inline int pm_register_bt_power_onoff(pm_module_power_onoff bt_power_cb, unsigned int select) { return 0; }
static inline void pm_unregister_bt_power_onoff(void) { ; }
static inline int pm_register_ble_power_onoff(pm_module_power_onoff ble_power_cb, unsigned int select) { return 0; }
static inline void pm_unregister_ble_power_onoff(void) { ; }

static inline int pm_check_wakeup_irqs(void) { return 0; }

static inline int pm_test(void) { return 0; }
#endif /* CONFIG_PM */

#ifdef __cplusplus
}
#endif

#endif /* __XRADIO_PM_H */
