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

#include "hal_base.h"
#include "driver/chip/system_chip.h"
#include "driver/chip/hal_global.h"
#include "sys/xr_debug.h"
#include "pm/pm.h"

#ifdef CONFIG_OS_FREERTOS
extern uint32_t SystemCoreClock; /* defined in FreeRTOS */
#else
uint32_t SystemCoreClock;
#endif
extern const unsigned char __VECTOR_BASE[];    /* SRAM start address */

#ifndef CONFIG_BOOTLOADER
static SystemStartupState startup_state = SYS_POWERON;

SystemStartupState SysGetStartupState(void)
{
	return startup_state;
}

void SysSetStartupState(SystemStartupState state)
{
	startup_state = state;
}

__STATIC_INLINE void SysStartupStateInit(void)
{
	uint32_t rstSrc = HAL_PRCM_GetResetSource();
	uint32_t flag = HAL_PRCM_GetCPUABootArg();
	uint32_t val;

	if (PRCM_BOOT_FLAG_IS_VALID(flag)) {
		val = PRCM_BOOT_VAL(flag);
	} else {
		val = PRCM_BOOT_VAL_INVALID;
	}

	if ((rstSrc & PRCM_CPU_IS_PWRON_RST_BIT) != 0) {
		startup_state = SYS_POWERON;
	} else if ((rstSrc & PRCM_CPU_IS_WATCHDOG_ALL_RST_BIT) != 0) {
		if (val == PRCM_BOOT_VAL_WDG_SYS_RST) {
			startup_state = SYS_REBOOT;
		} else {
			startup_state = SYS_WATCHDOG_CHIP_RST;
		}
	} else if ((rstSrc & PRCM_CPU_IS_WATCHDOG_CPU_RST_MASK) != 0) {
		if (val == PRCM_BOOT_VAL_WDG_CPU_RST) {
			startup_state = SYS_CPU_RST;
		} else {
			startup_state = SYS_WATCHDOG_CPU_RST;
		}
	} else {
		if (val == PRCM_BOOT_VAL_NVIC_CPU_RST) {
			startup_state = SYS_NVIC_RST;
		} else if (val == PRCM_BOOT_VAL_HIBERNATION) {
			startup_state = SYS_HIBERNATION;
		}
	}

	if (PRCM_BOOT_FLAG_IS_VALID(flag)) {
		HAL_PRCM_SetCPUABootArg(0x0); /* reset boot flag */
	}
	HAL_PRCM_ClrResetSource();
}

#endif

__STATIC_INLINE void SystemChipAdjust(void)
{
#if (CONFIG_CHIP_ARCH_VER == 3)
	HAL_PRCM_SetBANDGAPSTABLE_TIME(1);

	HAL_PRCM_SetLDOTOPWMSTABLE_TIME(0);
	HAL_PRCM_SetDCDCSTABLE_TIME(5);

	HAL_PRCM_SetRFIPLDODIGSTABLE_TIME(1);
	HAL_PRCM_SetLDOSTABLE_TIME(2);

	HAL_PRCM_SetDCXOSTABLE_TIME(HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS)/2000); /* LFCLK*0.0005 */

	HAL_PRCM_SetDPLLSTABLE_TIME(2);

	HAL_PRCM_SetDigSWRefTime(0x1DB0);
#endif

#if (CONFIG_CHIP_ARCH_VER > 1)
	HAL_MODIFY_REG(PRCM->DCXO_CTRL, PRCM_ICTRL_OFFSET_MASK, 0x10 << PRCM_ICTRL_OFFSET_SHIFT);
#endif
}

void SystemInit(void)
{
#if (CONFIG_CHIP_ARCH_VER == 2)
	if (HAL_GlobalGetTopLdoVsel() == 0) {
		HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V8);
	}
	HAL_PRCM_SetRTCLDOVoltage(PRCM_RTC_LDO_RETENTION_VOLT_675MV, PRCM_RTC_LDO_WORK_VOLT_1075MV);
#ifdef CONFIG_WLAN
	HAL_CLR_BIT(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_WLAN_SRAM_116K_SWM5_BIT);
#else
	HAL_SET_BIT(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_WLAN_SRAM_116K_SWM5_BIT);
#endif
#elif (CONFIG_CHIP_ARCH_VER == 3)

/* need rebuild bootloader */
#ifdef CONFIG_PWR_INTERNAL_DCDC
	if (HAL_GlobalGetChipVer() == 0x0A) {
		HAL_PRCM_SetTOPLDOForceActive(1); /* always turn on TOPLDO */
	} else if (HAL_GlobalGetInternalFlag(10) == 0) {
		HAL_PRCM_SetTOPLDOForceActive(0); /* always turn on TOPLDO */
	} else {
		HAL_PRCM_SetTOPLDOForceActive(1); /* always turn off TOPLDO */
	}
	if (HAL_GlobalGetInternalFlag(10) == 0) {
		HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V4);
	}
#ifdef CONFIG_PSRAM
	HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_1V8);
#else
	if (HAL_GlobalGetInternalFlag(48) == 0) {
		HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_1V8);
	} else {
		HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_1V5);
	}
#endif
	HAL_PRCM_EnableSMPSPwmSel(1); /* enable PWM mode */
	HAL_PRCM_SetSMPSPwmSelActive(PRCM_SMPS_PWM_SEL_CLK_RFIPDPLL);
	if (HAL_GlobalGetInternalFlag(9) == 0) {
		if (!HAL_GlobalGetConnectMode()) {
			HAL_PRCM_SetOvrSMPSDetctActive(1); /* turn on, set ctrl by software not by powerctrl */
			HAL_PRCM_SetSMPSDetctValueActive(0); /* default is 0 */
		}
	}
#else
	if (HAL_GlobalGetChipVer() == 0x0A) {
		HAL_PRCM_SetTOPLDOForceActive(0); /* always turn off TOPLDO */
	} else { /* 0x0B or 0x0C */
		HAL_PRCM_SetTOPLDOForceActive(1); /* always turn off TOPLDO */
	}
#endif
	HAL_PRCM_SetDIGLDOVolt(PRCM_DIGLDO_VOLT_1125MV, PRCM_DIGLDO_RET_VOLT_725MV);
	HAL_PRCM_SetRTCLDOVoltage(PRCM_RTC_LDO_VOLT_675MV);
#endif
	SCB->VTOR = (uint32_t)__VECTOR_BASE; /* Vector Table Relocation in Internal SRAM. */

#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
	/* FPU settings, set CP10 and CP11 Full Access */
	SCB->CPACR |= ((3UL << 20) | (3UL << 22));
#endif

	/* set flash chip type */
#ifdef CONFIG_EXT_FLASH_ONLY
	HAL_PRCM_SetFlashExt(1);
#endif

	/* set clock */
	HAL_BoardIoctl(HAL_BIR_CHIP_CLOCK_INIT, 0, 0);

	SystemCoreClock = HAL_GetCPUClock();
	SystemChipAdjust();
#ifndef CONFIG_BOOTLOADER
	pm_init();
	SysStartupStateInit();
	HAL_NVIC_Init();
	HAL_CCM_Init();
	HAL_RTC_Init();
#endif
}

void SystemDeInit(uint32_t flag)
{
	/* disable irq0~63 */
	NVIC->ICER[0] = NVIC->ISER[0];
	NVIC->ICER[1] = NVIC->ISER[1];

#ifndef CONFIG_BOOTLOADER
	/* clear irq pending0~63 */
	NVIC->ICPR[0] = NVIC->ISPR[0];
	NVIC->ICPR[1] = NVIC->ISPR[1];
#endif

	/* disable systick irq */
	SysTick->CTRL = 0;

	/* clear pendsv irq */
	SCB->ICSR = (SCB->ICSR & SCB_ICSR_PENDSVSET_Msk) >> (SCB_ICSR_PENDSVSET_Pos - SCB_ICSR_PENDSVCLR_Pos);

	/* clear systick irq */
	SCB->ICSR = (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) >> (SCB_ICSR_PENDSTSET_Pos - SCB_ICSR_PENDSTCLR_Pos);

	if (flag & SYSTEM_DEINIT_FLAG_RESET_CLK) {
		/* reset clock for restart */
		HAL_PRCM_SetCPUAClk(PRCM_CPU_CLK_SRC_HFCLK, PRCM_SYS_CLK_FACTOR_80M);
		HAL_CCM_BusSetClock(CCM_AHB2_CLK_DIV_1, CCM_APB_CLK_SRC_HFCLK, CCM_APB_CLK_DIV_2);
		HAL_PRCM_DisableSysPLL();
#if (CONFIG_CHIP_ARCH_VER > 1)
		HAL_CCM_BusSetAPBSClock(CCM_APBS_CLK_SRC_HFCLK, 0);
#endif
	}
}

void SystemCoreClockUpdate(void)
{
	SystemCoreClock = HAL_GetCPUClock();
}
