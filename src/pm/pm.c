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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h> /* for timeofday */

#include "sys/io.h"
#include "errno.h"
#include "sys/list.h"
#include "sys/interrupt.h"
#include "sys/param.h"
#include "kernel/os/os_thread.h"
#include "kernel/os/os_semaphore.h"
#include "kernel/os/os_time.h"
#include "FreeRTOS.h"

#include "driver/chip/system_chip.h"
#include "driver/chip/chip.h"
#include "driver/chip/hal_global.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_rcosc_cali.h"

#include "pm/pm.h"
#include "_pm_define.h"
#include "pm_i.h"
#include "port.h"

#ifdef CONFIG_PM

static struct arm_CMX_core_regs vault_arm_registers;

#define PM_TIMEOFDAY_SAVE() timeofday_save()

#define CONFIG_PM_TRY_MS_STEP_WHEN_FAIL 100
#if ((defined CONFIG_PM_WAKELOCKS) || ((defined CONFIG_PM_TRY_NUM_WHEN_FAIL) && (CONFIG_PM_TRY_NUM_WHEN_FAIL > 0)))
static uint32_t _pm_fail_try_again_ms = 50;
#endif
static uint32_t _pm_tmo = OS_WAIT_FOREVER;

#ifdef CONFIG_STANDBY_RET_SRAM
#ifdef CONFIG_STANDBY_RET_SRAM_ALL
#define STANDBY_APP_SRAM_RETENTION_CFG      (~(PRCM_SYS_APP_SRAM_PWR_CTRL_MASK))
#else
static __inline uint32_t _standby_app_sram_retention_cfg(void)
{
	uint32_t __cfg = PRCM_SYS_APP_SRAM_PWR_CTRL_MASK;
#ifdef CONFIG_STANDBY_RET_SRAM0L_16K
	__cfg &= ~PRCM_SYS_SRAM_16K_SWM6_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SRAM0H_16K
	__cfg &= ~PRCM_SYS_SRAM_16K_SWM5_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SRAM1_32K
	__cfg &= ~PRCM_SYS_SRAM_32K_SWM4_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SRAM2_3_4_96K
	__cfg &= ~PRCM_SYS_SRAM_96K_SWM3_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SRAM5_6_7_96K
	__cfg &= ~PRCM_SYS_SRAM_96K_SWM2_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SRAM8_32K
	__cfg &= ~PRCM_SYS_SRAM_32K_SWM1_BIT;
#endif
#ifdef CONFIG_STANDBY_RET_SSRAM9_10L_48K
	__cfg &= ~PRCM_SYS_SRAM_48K_SWM7_BIT;
#endif
	return __cfg;
}
#define STANDBY_APP_SRAM_RETENTION_CFG      _standby_app_sram_retention_cfg()
#endif
#else
#define STANDBY_APP_SRAM_RETENTION_CFG      (~(PRCM_SYS_APP_SRAM_PWR_CTRL_MASK))
#endif

#ifdef CONFIG_HIBERNATION_RET_SRAM
#ifdef CONFIG_HIBERNATION_RET_SRAM_ALL
#define HIBERNATION_APP_SRAM_RETENTION_CFG  (~(PRCM_SYS_APP_SRAM_PWR_CTRL_MASK))
#else
static __inline uint32_t _hibernation_app_sram_retention_cfg(void)
{
	uint32_t __cfg = PRCM_SYS_APP_SRAM_PWR_CTRL_MASK;
#ifdef CONFIG_HIBERNATION_RET_SRAM0L_16K
	__cfg &= ~PRCM_SYS_SRAM_16K_SWM6_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SRAM0H_16K
	__cfg &= ~PRCM_SYS_SRAM_16K_SWM5_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SRAM1_32K
	__cfg &= ~PRCM_SYS_SRAM_32K_SWM4_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SRAM2_3_4_96K
	__cfg &= ~PRCM_SYS_SRAM_96K_SWM3_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SRAM5_6_7_96K
	__cfg &= ~PRCM_SYS_SRAM_96K_SWM2_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SRAM8_32K
	__cfg &= ~PRCM_SYS_SRAM_32K_SWM1_BIT;
#endif
#ifdef CONFIG_HIBERNATION_RET_SSRAM9_10L_48K
	__cfg &= ~PRCM_SYS_SRAM_48K_SWM7_BIT;
#endif
	return __cfg;
}
#define HIBERNATION_APP_SRAM_RETENTION_CFG  _hibernation_app_sram_retention_cfg()
#endif
#else
#define HIBERNATION_APP_SRAM_RETENTION_CFG  (PRCM_SYS_APP_SRAM_PWR_CTRL_MASK)
#endif

static int __suspend_begin(enum suspend_state_t state)
{
	/* set SEVONPEND flag */
	SCB->SCR = 0x10;

	return 0;
}

/* hibernation whole system, save user data to flash before call this func.
 * BE SUURE: all hardware has been closed and it's prcm config setted to default value.
 */
static void pm_hibernation(void)
{
	__record_dbg_status(PM_HIBERNATION | 0);

	/* step 1 & 2 has been done when wlan sys poweroff */
	/* step3: writel(0x0f, GPRCM_SYS1_WAKEUP_CTRL) to tell PMU that turn on
	 * SW1, SW2, SRSW1, LDO before release application system reset signal.
	 */
	HAL_PRCM_SetSys1WakeupPowerFlags(
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1));
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PRCM_SetSys1SleepPowerFlags(
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1) |
	    PRCM_SYS_SRAM_PWR_CTRL_MASK | PRCM_SYS_CACHE_SRAM_PWR_CTRL_BIT | PRCM_SYS_CACHE_SRAM_SWM1_BIT);
#elif (CONFIG_CHIP_ARCH_VER == 3)
	HAL_PRCM_SetSys1SleepPowerFlags(
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1));
	uint32_t ret_cfg = HAL_PRCM_GetSys1SramSleepPowerFlags() & PRCM_SYS_SRAM_96K_SWM8_BIT;
	ret_cfg |= (HIBERNATION_APP_SRAM_RETENTION_CFG & ~(PRCM_SYS_SRAM_96K_SWM8_BIT));
	HAL_PRCM_SetSys1SramSleepPowerFlags(ret_cfg);
#endif
	HAL_PRCM_SetEXTLDOMdoe(PRCM_EXTLDO_ALWAYS_ON);
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PRCM_SetTOPLDOForceActive(0);
#elif (CONFIG_CHIP_ARCH_VER == 3)
#ifdef CONFIG_PWR_INTERNAL_DCDC
	HAL_PRCM_SetSYSStandbySMPSOffActive(1); /* will turn off */
	if (HAL_GlobalGetInternalFlag(49) == 0) {
		HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_2V1);
	}
#endif
#endif
	HAL_PRCM_EnalbeEXTLDOSwMode(1); /* EXT_LDO will enter lower mode when hibernation */

	__record_dbg_status(PM_HIBERNATION | 5);

	/* step4: writel(0x0f, GPRCM_SYS1_SLEEP_CTRL) to tell PMU that turn off SW1,
	 * SW3 SRSW1, LDO after pull down application system reset signal.
	 */
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PRCM_EnableTOPLDODeepsleep(0);
	HAL_PRCM_EnableLDOModeSWSelEnable(0);
	HAL_PRCM_EnableTOPLDOLQModeEnable(0);
	HAL_PRCM_EnableSysLDOLQModeEnable(0);
#endif
	__record_dbg_status(PM_HIBERNATION | 7);

	/* step5: switch to HOSC, close SYS1_CLK. */
	PM_SystemDeinit();
	__record_dbg_status(PM_HIBERNATION | 9);

	/* step6: set nvic deepsleep flag, and enter wfe. */
	SCB->SCR = 0x14;
	PM_SetCPUBootFlag(PRCM_CPUA_BOOT_FROM_COLD_RESET);
	PM_TIMEOFDAY_SAVE();

	__disable_fault_irq();
	__disable_irq();

	if (check_wakeup_irqs()) {
		PM_REBOOT();
	}

	wfe();
	if (check_wakeup_irqs()) {
		PM_REBOOT();
	}

	extern void cpu_tz_hibernation(void);
	cpu_tz_hibernation();

	wfe();
	/* some irq generated when second wfe */
	PM_REBOOT();

	__record_dbg_status(PM_HIBERNATION | 0x0ff);
}

static void __suspend_enter(enum suspend_state_t state)
{
	__record_dbg_status(PM_SUSPEND_ENTER | 5);

	__record_dbg_status(PM_SUSPEND_ENTER | 6);
	if (HAL_Wakeup_SetSrc(1))
		return;

	PM_LOGD("device info. rst:%x clk:%x\n", CCM->BUS_PERIPH_RST_CTRL,
	        CCM->BUS_PERIPH_CLK_CTRL); /* debug info. */

	PM_SetCPUBootArg((uint32_t)&vault_arm_registers);

#ifdef CONFIG_PM_DEBUG
	for (int i = 0; i < PM_DEBUG_DUMP_NUM; i++) {
		if (pm_debug_dump_addr[i][0])
			print_hex_dump_words((const void *)pm_debug_dump_addr[i][1],
			                     pm_debug_dump_addr[i][0]);
	}
#endif

	if (state == PM_MODE_HIBERNATION) {
		__record_dbg_status(PM_SUSPEND_ENTER | 7);
		HAL_PRCM_SetCPUABootArg(PRCM_BOOT_FLAG(PRCM_BOOT_VAL_HIBERNATION));
#ifdef CONFIG_ARCH_APP_CORE
		/* clear */
		HAL_PRCM_WakeupIOClearEventDetected(HAL_PRCM_WakeupIOGetEventStatus());
		/* set hold */
		HAL_Wakeup_SetIOHold(WAKEUP_IO_MASK);
#endif
		pm_hibernation(); /* never return */
	} else if (state < PM_MODE_STANDBY) {
		__record_dbg_status(PM_SUSPEND_ENTER | 8);
		/* TODO: set system bus to low freq */
		__cpu_sleep(state);
		SysSetStartupState(SYS_SLEEP);
		/* TODO: restore system bus to normal freq */
	} else {
		HAL_PRCM_SetSys1WakeupPowerFlags(
		    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
		    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1));
		HAL_PRCM_SetSys1SleepPowerFlags(
		    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
#if (CONFIG_CHIP_ARCH_VER == 2)
		    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1) |
#endif
		    0);
		uint32_t ret_cfg = HAL_PRCM_GetSys1SramSleepPowerFlags() & PRCM_SYS_SRAM_96K_SWM8_BIT;
		ret_cfg |= (STANDBY_APP_SRAM_RETENTION_CFG & ~(PRCM_SYS_SRAM_96K_SWM8_BIT));
		HAL_PRCM_SetSys1SramSleepPowerFlags(ret_cfg);

#if (CONFIG_CHIP_ARCH_VER > 1)
		HAL_MODIFY_REG(PRCM->DCXO_CTRL, PRCM_ICTRL_OFFSET_MASK, 0x00 << PRCM_ICTRL_OFFSET_SHIFT);
#endif

#if (CONFIG_CHIP_ARCH_VER == 3)
#if (!defined(CONFIG_PSRAM))
#ifdef CONFIG_PWR_INTERNAL_DCDC
		if (HAL_GlobalGetInternalFlag(48) == 0) {
			HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_1V5);
		}
#endif
#endif
#endif

		__record_dbg_status(PM_SUSPEND_ENTER | 9);
		__cpu_suspend(state);
#if (CONFIG_CHIP_ARCH_VER == 3)
#if (!defined(CONFIG_PSRAM))
#ifdef CONFIG_PWR_INTERNAL_DCDC
		if (HAL_GlobalGetInternalFlag(48) == 0) {
			HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_1V8);
		}
#endif
#endif
#endif

#if (CONFIG_CHIP_ARCH_VER > 1)
		HAL_MODIFY_REG(PRCM->DCXO_CTRL, PRCM_ICTRL_OFFSET_MASK, 0x10 << PRCM_ICTRL_OFFSET_SHIFT);
#endif

		SysSetStartupState(SYS_STANDBY);
	}
#if 0
	PM_BUG_ON(NULL, !PM_IRQ_GET_FLAGS());
#endif
	__record_dbg_status(PM_SUSPEND_ENTER | 0xa);
	HAL_Wakeup_ClrSrc(1);

	__record_dbg_status(PM_SUSPEND_ENTER | 0xb);
}

static void __suspend_end(enum suspend_state_t state)
{
	/* clear SEVONPEND flag */
	SCB->SCR = 0x0;
}

#ifdef CONFIG_PM_DEBUG
void pm_dump_regs(unsigned int flag)
{
	if (flag & 1 << 0) { /* cpu */
		int i, j;

		PM_LOGD("regs:\n");
		PM_LOGD("msp:0x%08x, psp:0x%08x, psr:0x%08x, primask:0x%08x\n",
		        vault_arm_registers.msp, vault_arm_registers.psp,
		        vault_arm_registers.psr, vault_arm_registers.primask);
		PM_LOGD("faultmask:0x%08x, basepri:0x%08x, control:0x%08x\n",
		        vault_arm_registers.faultmask, vault_arm_registers.basepri,
		        vault_arm_registers.control);
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 4; j++) {
				PM_LOGD("reg[%d]:0x%08x ", j + i * 4,
				        vault_arm_registers.reg12[j + i * 4]);
			}
			PM_LOGD("\n");
		}
		PM_LOGD("last step:%x\n", __get_last_record_step());
	}
	if (flag & 1 << 1) { /* nvic */
		/*nvic_print_regs();*/
	}
	if (flag & 1 << 2) { /* ccmu */
		/*ccm_print_regs();*/
	}
	if (flag & 1 << 3) { /* gpio */
		/*gpio_print_regs();*/
	}
}

#else
#define suspend_test_start()
#define suspend_test_finish(x...)
#define suspend_test(l)         0
void pm_set_test_level(enum suspend_test_level_t level) { ; }
#define pm_dump_regs(flag)
#endif

/**
 * check if any wake-up interrupts are pending
 */
int pm_check_wakeup_irqs(void)
{
	int i;
	unsigned int *addr;
	unsigned int val, ret = 0;

	/* Check if there are any interrupts pending */
	addr = (unsigned int *)NVIC->ISPR;
	for (i = 0; i < DIV_ROUND_UP(NVIC_PERIPH_IRQ_NUM, 32); i++) {
		val = addr[i];
		if (val & nvic_int_mask[i]) {
			PM_LOGD("nvic[%d]:%x, mask:%x en:%x\n", i, val,
			        nvic_int_mask[i], NVIC->ISER[i]);
			if ((val & (1 << A_WAKEUP_IRQn)) || (val & (1 << GPIOA_IRQn))) {
				ret = PM_WAKEUP_SRC_WKTIMER | (PM_WAKEUP_SRC_WKTIMER - 1);
			} else if (val & (1 << WIFIC_IRQn)) {
				PM_LOGD("wakeup wlan pending\n");
				return PM_WAKEUP_SRC_WLAN;
			} else if (val & (1 << RTC_SEC_ALARM_IRQn)) {
				PM_LOGD("RTC pending\n");
				return PM_WAKEUP_SRC_RTC_SEC;
			} else if (val & (1 << RTC_WDAY_ALARM_IRQn)) {
				PM_LOGD("RTC pending\n");
				return PM_WAKEUP_SRC_RTC_WDAY;
			} else {
				PM_LOGD("wakeup devices pending\n");
				return PM_WAKEUP_SRC_DEVICES;
			}
		}
	}

	if (HAL_Wakeup_ReadTimerPending()) {
		PM_LOGD("wakeup timer pending\n");
		return PM_WAKEUP_SRC_WKTIMER;
	}

#ifdef CONFIG_ARCH_APP_CORE
	ret = HAL_Wakeup_ReadIO();
	if (ret) {
		PM_LOGD("wakeup io pending\n");
		return ret;
	}
#endif

#ifdef CONFIG_PM_WAKELOCKS
	if (pm_wakelocks_is_active()) {
		PM_LOGD("wakelock locked\n");
		return PM_WAKEUP_SRC_SOFTWARE;
	}
#else
	if (!_pm_tmo) {
		PM_LOGD("suspend abort\n");
		return PM_WAKEUP_SRC_SOFTWARE;
	}
#endif

	return ret;
}

/**
 * @brief Initialize the PM-related part of a device object.
 * @note not use printf for this fun is called very earlier.
 * @retval  0 if success or other if failed.
 */
int pm_init(void)
{
	uint32_t mode;

	pm_set_debug_mask(~((1<<(ROM_DEBUG_SHIFT-CONFIG_PM_DEBUG_LEVEL))-1));

#if (CONFIG_CHIP_ARCH_VER == 3)
#ifdef CONFIG_PWR_INTERNAL_DCDC
	if (HAL_GlobalGetInternalFlag(49) == 0) {
		HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V4);
		HAL_PRCM_SetSMPSSwTrim(1); /* EXT_LDO enter 100uA mode during suspend */
	}
#endif
#endif
	HAL_PRCM_EnalbeEXTLDOSwMode(0); /* EXT_LDO not enter lower mode when suspend */
#ifdef CONFIG_PWR_INTERNAL_DCDC
	HAL_PRCM_SetSYSStandbySMPSOffActive(0); /* always turn on during standby */
#endif

	mode = HAL_PRCM_GetCPUABootArg();
#if (CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3)
	if (mode == PRCM_BOOT_FLAG(PRCM_BOOT_VAL_HIBERNATION))
		HAL_Wakeup_ClrSrc(0);
#endif

	HAL_Wakeup_Init();
	HAL_Wakeup_SetDebugMask(ROM_WRN_MASK | ROM_ERR_MASK | ROM_ANY_MASK);

	suspend_ops.begin = __suspend_begin;
	suspend_ops.prepare = platform_prepare;
	suspend_ops.prepare_late = platform_prepare_late;
	suspend_ops.enter = __suspend_enter;
	suspend_ops.wake = platform_wake;
	suspend_ops.finish = platform_finish;
	suspend_ops.end = __suspend_end;

#ifdef CONFIG_ARCH_APP_CORE
	/* set prcm to default value for prcm keep it's last time value. */
	HAL_PRCM_SetSys1WakeupPowerFlags(
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1) |
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_LDO1));
	HAL_PRCM_SetSys1SleepPowerFlags(
	    PRCM_SYS_WS_PWR_FLAG(PRCM_SYS_WS_PWR_VAL_ON, PRCM_SYS_WS_PWR_TYPE_SYS1_PWR1));
	uint32_t ret_cfg = HAL_PRCM_GetSys1SramSleepPowerFlags() & PRCM_SYS_SRAM_96K_SWM8_BIT;
	HAL_PRCM_SetSys1SramSleepPowerFlags(ret_cfg);
#if ((defined CONFIG_BLE) && (defined CONFIG_CACHE_SIZE_32K))
	/* HAL_PRCM_SetSys1SramSleepPowerFlags(PRCM_SYS_SRAM_48K_SWM7_BIT); */ /* always power on for ble use */
#endif

	HAL_PRCM_EnableLDOModeSWSelEnable(0);  /* switch PWM/PFM mode by hw, enter LQ mode by hw. */

#endif

#ifdef CONFIG_ROM
	pm_set_suspend_resume_latency(150, 350);
#else
	pm_set_suspend_resume_latency(30, 50);
#endif

	return 0;
}

#ifdef CONFIG_ARCH_APP_CORE
/**
 * @brief Set a magin to synchronize with net.
 */
void pm_set_sync_magic(void)
{
	PM_SetCPUBootArg(PM_SYNC_MAGIC); /* set flag to notify net to run */
}
#endif

#ifdef CONFIG_BT
pm_module_power_onoff pm_bt_power_onoff_cb;
int pm_bt_mode_platform_config = PM_SUPPORT_HIBERNATION;

/**
 * @brief Select bt power modes when enter pm.
 * @note bt power on/off calback will called when pm enter select modes.
 * @param bt_power_cb:
 *        @arg bt_power_cb->bt power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_bt_power_onoff(pm_module_power_onoff bt_power_cb, unsigned int select)
{
	pm_bt_power_onoff_cb = bt_power_cb;
	PM_LOGN("bt mode:%x\n", pm_bt_mode_platform_config);

	return 0;
}

/** @brief unregister bt power on/off callback. */
void pm_unregister_bt_power_onoff(void)
{
	pm_bt_power_onoff_cb = NULL;
}
#endif

#ifdef CONFIG_BLE
pm_module_power_onoff pm_ble_power_onoff_cb;
int pm_ble_mode_platform_config = PM_SUPPORT_HIBERNATION;

/**
 * @brief Select ble power modes when enter pm.
 * @note ble power on/off calback will called when pm enter select modes.
 * @param ble_power_cb:
 *        @arg ble_power_cb->ble power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_ble_power_onoff(pm_module_power_onoff ble_power_cb, unsigned int select)
{
	pm_ble_power_onoff_cb = ble_power_cb;
	PM_LOGN("ble mode:%x\n", pm_ble_mode_platform_config);

	return 0;
}

/** @brief unregister ble power on/off callback. */
void pm_unregister_ble_power_onoff(void)
{
	pm_ble_power_onoff_cb = NULL;
}
#endif

/**
 * @brief Set system to a lowpower mode.
 * @param state:
 *        @arg state->The lowpower mode will enter.
 * @retval  0 if success or other if failed.
 */
static int _pm_enter_mode(enum suspend_state_t state)
{
	int err, record;
	enum suspend_state_t state_use = state;
#if ((defined CONFIG_PM_TRY_NUM_WHEN_FAIL) && (CONFIG_PM_TRY_NUM_WHEN_FAIL > 0))
	int cnt = 0;
#endif
#ifdef CONFIG_ARCH_APP_CORE
	int loop;

	if (!(pm_mode_platform_config & (1 << state))) {
		for (loop = (1 << state_use); loop; loop >>= 1) {
			if (pm_mode_platform_config & loop) {
				break;
			}
			state_use--;
		}
	}

	if (!HAL_Wakeup_CheckIOMode()) {
		PM_LOGE("some wakeup io not EINT mode\n");
		return -1;
	}
#endif

	if (state_use >= PM_MODE_MAX) {
		PM_LOGE("%s:%d err mode:%d!\n", __func__, __LINE__, state_use);
		return -1;
	}

	HAL_Wakeup_SetEvent(0);

	if (state_use < PM_MODE_SLEEP)
		return 0;

	pm_select_mode(state_use);
	PM_LOGA(PM_SYS" enter mode: %s\n", pm_states[state_use]);
	record = __get_last_record_step();
	if (record != PM_RESUME_COMPLETE)
		PM_LOGN("last suspend record:%x\n", record);
#ifdef CONFIG_PM_DEBUG
	parse_dpm_list(PM_OP_NORMAL);  /* debug info. */
	parse_dpm_list(PM_OP_NOIRQ);
#endif
#ifdef CONFIG_ARCH_APP_CORE
#ifdef CONFIG_WLAN
	if ((pm_wlan_mode_platform_config & (1 << state_use)) &&
	    pm_wlan_power_onoff_cb) {
		pm_wlan_power_onoff_cb(0);
	}
#endif
#ifdef CONFIG_BT
	if ((pm_bt_mode_platform_config & (1 << state_use)) &&
	    pm_bt_power_onoff_cb) {
		pm_bt_power_onoff_cb(0);
	}
#endif
#ifdef CONFIG_BLE
	if ((pm_ble_mode_platform_config & (1 << state_use)) &&
	    pm_ble_power_onoff_cb) {
		pm_ble_power_onoff_cb(0);
	}
#endif
#endif

#if ((defined CONFIG_PM_TRY_NUM_WHEN_FAIL) && (CONFIG_PM_TRY_NUM_WHEN_FAIL > 0))
again:
#endif
	err = suspend_devices_and_enter(state_use);
#if ((defined CONFIG_PM_TRY_NUM_WHEN_FAIL) && (CONFIG_PM_TRY_NUM_WHEN_FAIL > 0))
	if ((err != 0) && (cnt++ < CONFIG_PM_TRY_NUM_WHEN_FAIL)) {
		OS_MSleep(pm_fail_try_again_ms + CONFIG_PM_TRY_MS_STEP_WHEN_FAIL * cnt);
		goto again;
	}
#endif

#ifdef CONFIG_ARCH_APP_CORE
	pm_set_sync_magic();

#ifdef CONFIG_BLE
	if ((pm_ble_mode_platform_config & (1 << state_use)) &&
	    pm_ble_power_onoff_cb) {
		pm_ble_power_onoff_cb(1);
	}
#endif
#ifdef CONFIG_BT
	if ((pm_bt_mode_platform_config & (1 << state_use)) &&
	    pm_bt_power_onoff_cb) {
		pm_bt_power_onoff_cb(1);
	}
#endif
#ifdef CONFIG_WLAN
	if ((pm_wlan_mode_platform_config & (1 << state_use)) &&
	    pm_wlan_power_onoff_cb) {
		pm_wlan_power_onoff_cb(1);
	}
#endif
#endif

	return err;
}

static uint32_t _pm_state = PM_MODE_ON;

#ifdef CONFIG_PM_WAKELOCKS
static struct wakelock _pm_wakelock = {
	.name = "pm_wl",
};

static uint32_t _pm_internal_wake_try_again_ms = 100;

struct pm_thread_t {
	OS_Thread_t thread;
	OS_Semaphore_t wait;
	int wake_ret;
};

static struct pm_thread_t *_pm_thread;
static uint8_t _pm_step;

static void pm_task(void *arg)
{
	int ret;
	uint32_t mask, wk_event;
#ifdef CONFIG_ROM
	int fail_cnt;
#endif

#ifdef CONFIG_TRUSTZONE
	portALLOCATE_SECURE_CONTEXT(configMAXIMAL_SECURE_STACK_SIZE);
#endif

	while (1) {
		ret = pm_wakelocks_wait(_pm_tmo);
		if (ret == OS_FAIL) {
			continue;
		} else if (ret == OS_E_TIMEOUT) {
			HAL_Wakeup_SetEvent(PM_WAKEUP_SRC_SOFTWARE);
			goto out;
		} else if (ret) {
			pm_wakelocks_show();
			OS_MSleep(_pm_fail_try_again_ms);
			continue;
		}
		if (_pm_state >= PM_MODE_MAX) {
			PM_LOGW("%s exit!\n", __func__);
			break;
		}

		PM_LOGD("%s() endter %d\n", __func__, _pm_state);
		_pm_step = 1;
#ifdef CONFIG_ROM
		fail_cnt = suspend_stats.fail;
#endif
		ret = _pm_enter_mode(_pm_state);
		if (ret < 0) { /* eg. some devies(WiFI/...) suspend faild */
			goto out;
		}
#ifdef CONFIG_ROM
		if (ret == 0 && !HAL_Wakeup_GetEvent() && ((suspend_stats.fail - fail_cnt) > 0)) {
			ret = -1;
		}
#endif
		mask = HAL_Wakeup_GetEvent() & PM_WAKEUP_SRC_INTERNAL;
		wk_event = HAL_Wakeup_GetEvent() &
			(~(PM_WAKEUP_SRC_INTERNAL | PM_WAKEUP_SRC_WLAN | PM_WAKEUP_SRC_BLE | PM_WAKEUP_SRC_BT));
		if (mask && !wk_event) {
			OS_MSleep(_pm_internal_wake_try_again_ms);
			continue;
		}
out:
		_pm_tmo = OS_WAIT_FOREVER;
		_pm_thread->wake_ret = ret;
		OS_SemaphoreRelease(&_pm_thread->wait);
		while (_pm_step == 1) {
			OS_MSleep(5);
		}
	}

	OS_ThreadDelete(&_pm_thread->thread);
}
#endif

enum suspend_state_t pm_get_mode(void)
{
	return _pm_state;
}

int pm_enter_mode_timeout(enum suspend_state_t state, uint32_t tmo)
{
	int ret;

	if (state < PM_MODE_SLEEP)
		return 0;

	if (state >= PM_MODE_MAX) {
		PM_LOGN("%s exit!\n", __func__);
		return -1;
	}
#if 0
	PM_BUG_ON(NULL, PM_IRQ_GET_FLAGS());
#endif
	if (PM_IRQ_GET_FLAGS()) {
		PM_LOGE("%s exit for irq disabled!\n", __func__);
		return -1;
	}

	if (_pm_state != PM_MODE_ON) {
		PM_LOGN("%s exit!\n", __func__);
		return -1;
	}

	_pm_tmo = tmo;
#ifdef CONFIG_PM_WAKELOCKS
	_pm_state = state;
	pm_wakelocks_show();
	PM_LOGD("%s(),%d state:%d timeout:%d\n", __func__, __LINE__, _pm_state, _pm_tmo);
	if (tmo != OS_WAIT_FOREVER) {
		pm_wakelocks_touch();
		while (pm_wakelocks_is_touched()) {
			OS_MSleep(2);
		}
	}
	pm_wake_unlock(&_pm_wakelock);
	OS_SemaphoreWait(&_pm_thread->wait, OS_WAIT_FOREVER);

	/* wakeup */
	pm_wake_lock(&_pm_wakelock);
	ret = _pm_thread->wake_ret;
	_pm_step = 2;
#else
	ret = _pm_enter_mode(state);
#endif
	_pm_state = PM_MODE_ON;

	return ret;
}

int pm_enter_mode(enum suspend_state_t state)
{
	return pm_enter_mode_timeout(state, OS_WAIT_FOREVER);
}

void pm_suspend_abort(void)
{
	_pm_tmo = 0;
#ifdef CONFIG_PM_WAKELOCKS
	pm_wakelocks_touch();
#endif
}

void pm_start(void)
{
#ifdef CONFIG_PM_WAKELOCKS
	pm_wakelocks_init();
	pm_wake_lock(&_pm_wakelock);

	if (_pm_thread) {
		PM_LOGE("thread start again\n");
		return;
	}
	_pm_thread = malloc(sizeof(struct pm_thread_t));
	if (!_pm_thread) {
		PM_LOGE("thread malloc failed\n");
		return;
	}
	memset(_pm_thread, 0, sizeof(struct pm_thread_t));
	OS_SemaphoreCreate(&_pm_thread->wait, 0, OS_SEMAPHORE_MAX_COUNT);

	if (OS_ThreadIsValid(&_pm_thread->thread)) {
		PM_LOGE("thread invalid\n");
		return;
	}

	if (OS_ThreadCreate(&_pm_thread->thread,
	                    "pm",
	                    pm_task,
	                    NULL,
	                    OS_PRIORITY_NORMAL,
	                    (2 * 1024)) != 0) {
		PM_LOGE("create thread failed\n");
		return;
	}
#endif
}

void pm_stop(void)
{
#ifdef CONFIG_PM_WAKELOCKS
	if (!_pm_thread) {
		PM_LOGE("thread stop again\n");
		return;
	}

	if (!OS_ThreadIsValid(&_pm_thread->thread))
		return;

	_pm_state = PM_MODE_ON;
	pm_wake_unlock(&_pm_wakelock);
	while (OS_ThreadIsValid(&_pm_thread->thread)) {
		OS_MSleep(1);
	};
	OS_SemaphoreDelete(&_pm_thread->wait);
	free(_pm_thread);
	_pm_thread = NULL;

	pm_wake_unlock(&_pm_wakelock);
	pm_wakelocks_deinit();
#endif
}

#endif /* CONFIG_PM */
