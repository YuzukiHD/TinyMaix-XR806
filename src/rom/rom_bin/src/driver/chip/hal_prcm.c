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

#include "rom/driver/chip/hal_prcm.h"

#include "hal_base.h"

/*
 * Power
 *   - DCDC
 *   - Power switch
 *   - LDO
 */

uint32_t HAL_PRCM_GetTOPLDOVoltage(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_VOLT_MASK);
}

void HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDOVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_VOLT_MASK, volt);
}

void HAL_PRCM_SetTOPLDOForceActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_FORCE_ACTIVE_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_FORCE_ACTIVE_BIT);
}

#if (CONFIG_CHIP_ARCH_VER == 3)
uint32_t HAL_PRCM_GetSMPSVoltage(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_VOLT_MASK);
}

void HAL_PRCM_SetSMPSVoltage(PRCM_SMPSVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_VOLT_MASK, volt);
}

uint32_t HAL_PRCM_GetDIGLDOOffSMPSOn(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_DIGLDO_OFF_SMPS_ON_BIT);
}

void HAL_PRCM_SetDIGLDOOffSMPSOnActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_DIGLDO_OFF_SMPS_ON_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_DIGLDO_OFF_SMPS_ON_BIT);
}

uint32_t HAL_PRCM_GetSYSStandbySMPSOff(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SYS_STANDBY_SMPS_OFF_BIT);
}

void HAL_PRCM_SetSYSStandbySMPSOffActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SYS_STANDBY_SMPS_OFF_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SYS_STANDBY_SMPS_OFF_BIT);
}

uint32_t HAL_PRCM_GetSMPSPwmSel(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_PWM_SEL_MASK);
}

void HAL_PRCM_SetSMPSPwmSelActive(PRCM_SMPSPwmSel sel)
{
	HAL_MODIFY_REG(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_PWM_SEL_MASK, sel);
}

uint32_t HAL_PRCM_GetOvrSMPSDetct(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_OVR_SMPS_DETECT_BIT);
}

void HAL_PRCM_SetOvrSMPSDetctActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_OVR_SMPS_DETECT_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_OVR_SMPS_DETECT_BIT);
}

uint32_t HAL_PRCM_GetSMPSDetctValue(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_VALUE_BIT);
}

void HAL_PRCM_SetSMPSDetctValueActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_VALUE_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_VALUE_BIT);
}

uint32_t HAL_PRCM_GetSMPSDetct(void)
{
	return HAL_GET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_BIT);
}

void HAL_PRCM_SetSMPSDetctActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_SMPS_DETECT_BIT);
}

#endif

uint32_t HAL_PRCM_GetSysPowerEnableFlags(void)
{
#if (CONFIG_CHIP_ARCH_VER == 2)
	return HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL,
	                   PRCM_SW4_STATUS_BIT | PRCM_SW5_STATUS_BIT | PRCM_SR_SW3_BIT);
#elif (CONFIG_CHIP_ARCH_VER == 3)
	return HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL,
	                   PRCM_SW5_STATUS_BIT |
	                   PRCM_SW1_STATUS_BIT | PRCM_DIGLDO_STATUS_BIT);
#endif
}

#if (CONFIG_CHIP_ARCH_VER == 2)
uint8_t HAL_PRCM_GetLDO1Status(void)
{
	return !!HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_STATUS_BIT);
}

void HAL_PRCM_SetLDO1Voltage(PRCM_LDO1Volt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK, volt);
}

void HAL_PRCM_SetLDO1RETVolt(PRCM_LDO1RetVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_RET_VOLT_MASK, volt);
}

void HAL_PRCM_SetLDO1WorkVolt(PRCM_LDO1Volt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK, volt);
}

void HAL_PRCM_SetLDO1Volt(PRCM_LDO1Volt work_volt, PRCM_LDO1RetVolt ret_volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK | PRCM_LDO1_RET_VOLT_MASK, work_volt | ret_volt);
}

void HAL_PRCM_SelectEXTLDOVolt(PRCM_EXTLDOVolt volt)
{
	if (volt)
		HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_BIT);
}
#elif (CONFIG_CHIP_ARCH_VER == 3)
uint8_t HAL_PRCM_GetDIGLDOStatus(void)
{
	return !!HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_DIGLDO_STATUS_BIT);
}

void HAL_PRCM_SetDIGLDOVolt(PRCM_DIGLDOVolt work_volt, PRCM_DIGLDORetVolt ret_volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_DIGLDO_VOLT_MASK | PRCM_DIGLDO_RET_VOLT_MASK, work_volt | ret_volt);
}

void HAL_PRCM_SelectEXTLDOVolt(PRCM_EXTLDOVolt volt)
{
	if (volt)
		HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_MASK, volt);
	else
		HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_MASK, volt);
}
#endif

void HAL_PRCM_SetEXTLDOMdoe(PRCM_ExtLDOMode mode)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_MODE_MASK, mode);
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PRCM_SetPLLLdoActive(uint8_t active)
{
	if (active)
		HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_PLL_LDO_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_PLL_LDO_EN_BIT);
}
#endif

void HAL_PRCM_SetPadClkOut(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_EN_BIT);
}

void HAL_PRCM_SetPadClkOutSource(PRCM_PadClkOutSource source)
{
	HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_SOURCE_MASK, source);
}

void HAL_PRCM_SetPadClkOutFactorM(uint16_t value)
{
	HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_FACTOR_M_MASK,
	               PRCM_PAD_CLK_OUT_FACTOR_M_VAL(value));
}

/*
 * Clock
 */
void HAL_PRCM_SetLFCLKBaseSource(PRCM_LFCLKBaseSrc src)
{
	/* always enable inter 32K for external 32K is not ready at startup */
	uint32_t clr_mask = PRCM_LFCLK_BASE_SRC_MASK | PRCM_LFCLK_EXT32K_EN_BIT;
	uint32_t set_mask = src | PRCM_LFCLK_INTER32K_EN_BIT;
	if (src == PRCM_LFCLK_BASE_SRC_EXT32K) {
		set_mask |= PRCM_LFCLK_EXT32K_EN_BIT;
	}
	HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, clr_mask, set_mask);
}

void HAL_PRCM_SetLFCLKSource(PRCM_LFClkModule module, PRCM_LFClkSrc src)
{
	switch (module) {
	case PRCM_LFCLK_MODULE_SYS:
		if (src == PRCM_LFCLK_SRC_RCCAL) {
			HAL_SET_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_SYS_CLK32K_RCO_CALIB_SEL_BIT);
		} else {
			HAL_CLR_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_SYS_CLK32K_RCO_CALIB_SEL_BIT);
		}
		break;
	case PRCM_LFCLK_MODULE_WKTIMER:
		if (src == PRCM_LFCLK_SRC_RCCAL) {
			HAL_SET_BIT(PRCM->CLK_32K_CTRL, PRCM_WKTIMER_32K_CLK_RCO_CALIB);
		} else {
			HAL_CLR_BIT(PRCM->CLK_32K_CTRL, PRCM_WKTIMER_32K_CLK_RCO_CALIB);
		}
		break;
	case PRCM_LFCLK_MODULE_WLAN:
		if (src == PRCM_LFCLK_SRC_RCCAL) {
			HAL_SET_BIT(PRCM->CLK_32K_CTRL, PRCM_WLAN_32K_CLK_RCO_CALIB);
		} else {
			HAL_CLR_BIT(PRCM->CLK_32K_CTRL, PRCM_WLAN_32K_CLK_RCO_CALIB);
		}
		break;
	case PRCM_LFCLK_MODULE_BLE:
		HAL_CLR_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_AUTO_SW_EN_BIT); /* disable hosc 32K*/
		if (src == PRCM_LFCLK_SRC_RCCAL) {
			HAL_MODIFY_REG(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_SEL_MASK,
				PRCM_BLE_CLK32K_SEL_RCCAL);
		} else {
			HAL_MODIFY_REG(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_SEL_MASK,
				PRCM_BLE_CLK32K_SEL_RCOSCORLFCLK);
		}
		break;
	case PRCM_LFCLK_MODULE_RTC:
		if (src == PRCM_LFCLK_SRC_RCCAL) {
			HAL_MODIFY_REG(PRCM->CLK_32K_CTRL, PRCM_RTC_32K_CLK_SOURCE_MASK, PRCM_RTC_32K_CLK_RCO_CALIB);
		} else if (src == PRCM_LFCLK_SRC_DIV32K) {
			HAL_MODIFY_REG(PRCM->CLK_32K_CTRL, PRCM_RTC_32K_CLK_SOURCE_MASK, PRCM_RTC_32K_CLK_HOSC_DIV32K);
		} else {
			HAL_MODIFY_REG(PRCM->CLK_32K_CTRL, PRCM_RTC_32K_CLK_SOURCE_MASK, PRCM_RTC_32K_CLK_RCO_LOSC);
		}
		break;
	default:
		break;
	}
}

void HAL_PRCM_SetHOSCType(PRCM_HOSCType type)
{
	HAL_MODIFY_REG(PRCM->SYS_HOSC_CTRL, PRCM_HOSC_TYPE_MASK, type);
}

uint32_t HAL_PRCM_GetHOSCType(void)
{
	return HAL_GET_BIT(PRCM->SYS_HOSC_CTRL, PRCM_HOSC_TYPE_MASK);
}

uint32_t rom_HAL_PRCM_GetHFClock(void)
{
	static const uint32_t PRCM_HOSCClock[] =
#if (CONFIG_CHIP_ARCH_VER == 2)
		{ HOSC_CLOCK_26M, HOSC_CLOCK_40M, HOSC_CLOCK_24M, HOSC_CLOCK_52M };
#elif (CONFIG_CHIP_ARCH_VER == 3)
		{ HOSC_CLOCK_26M, HOSC_CLOCK_40M, HOSC_CLOCK_24M, HOSC_CLOCK_32M };
#endif

	uint32_t val;

	val = HAL_GET_BIT_VAL(PRCM->SYS_HOSC_CTRL,
	                      PRCM_HOSC_TYPE_SHIFT,
	                      PRCM_HOSC_TYPE_VMASK);
	return PRCM_HOSCClock[val];
}

uint32_t HAL_PRCM_GetInter32KFreq(void)
{
	return (10 * HAL_GET_BIT_VAL(PRCM->SYS_RCOSC_CALIB_CTRL,
	                             PRCM_RCOSC_CALIB_FREQ_SHIFT,
	                             PRCM_RCOSC_CALIB_FREQ_VMASK));
}

uint32_t HAL_PRCM_EnableInter32KCalib(void)
{
	return HAL_SET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

uint32_t HAL_PRCM_DisableInter32KCalib(void)
{
	return HAL_CLR_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

int HAL_PRCM_IsInter32KCalibEnabled(void)
{
	return HAL_GET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

/*
 *fix that inter32kfreq regs is 0 before 24M clock complete checking the inter32k clock.
 *32768 is recommended value.
 */
uint32_t HAL_PRCM_GetBaseLFClock(void)
{
	uint32_t val = HAL_GET_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_LFCLK_BASE_SRC_MASK);

	if (val == PRCM_LFCLK_BASE_SRC_INTER32K &&
	    HAL_GET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT)) {
		if (!HAL_PRCM_GetInter32KFreq()) {
			return SYS_LFCLOCK;
		} else {
			return HAL_PRCM_GetInter32KFreq();
		}
	} else {
		return SYS_LFCLOCK;
	}
}

uint32_t rom_HAL_PRCM_GetLFClock(PRCM_LFClkModule module)
{
	uint32_t src;
	uint32_t clk = 0;

	switch (module) {
	case PRCM_LFCLK_MODULE_SYS:
		src = HAL_GET_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_SYS_CLK32K_RCO_CALIB_SEL_BIT);
		if (src == PRCM_SYS_CLK32K_RCO_CALIB_SEL_BIT) {
			clk = 32 * 1000;
		}
		break;
	case PRCM_LFCLK_MODULE_WKTIMER:
		src = HAL_GET_BIT(PRCM->CLK_32K_CTRL, PRCM_WKTIMER_32K_CLK_RCO_CALIB);
		if (src == PRCM_WKTIMER_32K_CLK_RCO_CALIB) {
			clk = 32 * 1000;
		}
		break;
	case PRCM_LFCLK_MODULE_WLAN:
		src = HAL_GET_BIT(PRCM->CLK_32K_CTRL, PRCM_WLAN_32K_CLK_RCO_CALIB);
		if (src == PRCM_WLAN_32K_CLK_RCO_CALIB) {
			clk = 32 * 1000;
		}
		break;
	case PRCM_LFCLK_MODULE_RTC:
		src = HAL_GET_BIT(PRCM->CLK_32K_CTRL, PRCM_RTC_32K_CLK_SOURCE_MASK);
		if ((src == PRCM_RTC_32K_CLK_RCO_CALIB) || (src == PRCM_RTC_32K_CLK_HOSC_DIV32K)) {
			clk = 32 * 1000;
		}
		break;
	default:
		break;
	}

	if (!clk)
		clk = HAL_PRCM_GetBaseLFClock();

	return clk;
}

void HAL_PRCM_SetSysPLL(PRCM_SysPLLParam param)
{
	PRCM->SYS_PLL_CTRL = PRCM_SYS_PLL_EN_BIT | param; /* NB: enable system PLL */
}

void HAL_PRCM_DisableSysPLL(void)
{
	HAL_CLR_BIT(PRCM->SYS_PLL_CTRL, PRCM_SYS_PLL_EN_BIT);
}

void HAL_PRCM_SetCPUAClk(PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	/* TODO: change factor m and n seperately, add DSB, ISB */
	switch (src) {
	case PRCM_CPU_CLK_SRC_HFCLK:
	case PRCM_CPU_CLK_SRC_LFCLK:
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_CPU_CLK_SRC_MASK, src);
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL,
		               PRCM_SYS_CLK_FACTOR_MASK, /* PRCM_SYS_CLK_EN_BIT removed */
		               PRCM_SYS_CLK_FACTOR_80M); /* disable system clock */
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default:
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_FACTOR_MASK, factor);
		/* HAL_SET_BIT(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_EN_BIT); */ /* removed */
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_CPU_CLK_SRC_MASK, src);
		break;
	}
}

uint32_t HAL_PRCM_GetCPUAClk(void)
{
	uint32_t reg = PRCM->SYS_CLK1_CTRL;
	uint32_t freq;

	switch (reg & PRCM_CPU_CLK_SRC_MASK) {
	case PRCM_CPU_CLK_SRC_HFCLK:
		freq = HAL_GetHFClock();
		break;
	case PRCM_CPU_CLK_SRC_LFCLK:
		freq = HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS);
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default: {
		uint32_t div;
		uint32_t divm, divn;

		divm = HAL_GET_BIT_VAL(reg,
		                       PRCM_SYS_CLK_FACTORM_SHIFT,
		                       PRCM_SYS_CLK_FACTORM_VMASK) + PRCM_SYS_CLK_FACTORM_START;
		divn = HAL_GET_BIT_VAL(reg,
		                       PRCM_SYS_CLK_FACTORN_SHIFT,
		                       PRCM_SYS_CLK_FACTORN_VMASK) + 1;
		div = divm * divn;
		freq = SYS_PLL_CLOCK / div;
	}
		break;
	}
	return freq;
}

void HAL_PRCM_EnableCPUWClk(uint32_t enable)
{
	if (enable) {
		HAL_SET_BIT(PRCM->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	}
}

#if (CONFIG_CHIP_ARCH_VER == 2)
__xip_text
void HAL_PRCM_SetAudioPLLParam(PRCM_AudPLLParam param)
{
	PRCM->AUD_PLL_CTRL = param; /* NB: it will disable system PLL */
}

__xip_text
void HAL_PRCM_EnableAudioPLL(void)
{
	HAL_SET_BIT(PRCM->AUD_PLL_CTRL, PRCM_AUD_PLL_EN_BIT);
}

__xip_text
void HAL_PRCM_DisableAudioPLL(void)
{
	HAL_CLR_BIT(PRCM->AUD_PLL_CTRL, PRCM_AUD_PLL_EN_BIT);
}
#elif (CONFIG_CHIP_ARCH_VER == 3)

__xip_text
void HAL_PRCM_EnableAudioCkadcAud(void)
{
	HAL_SET_BIT(PRCM->DEV_CLK_CTRL, PRCM_CKADC_AUD_EN_BIT);
}

__xip_text
void HAL_PRCM_DisableAudioCkadcAud(void)
{
	HAL_CLR_BIT(PRCM->DEV_CLK_CTRL, PRCM_CKADC_AUD_EN_BIT);
}

__xip_text
void HAL_PRCM_EnableAudioCkcldAud(void)
{
	HAL_SET_BIT(PRCM->DEV_CLK_CTRL, PRCM_CKCLD_AUD_EN_BIT);
}

__xip_text
void HAL_PRCM_DisableAudioCkcldAud(void)
{
	HAL_CLR_BIT(PRCM->DEV_CLK_CTRL, PRCM_CKCLD_AUD_EN_BIT);
}

__xip_text
void HAL_PRCM_SetAudioCkcldParam(uint32_t n)
{
	HAL_MODIFY_REG(PRCM->DEV_CLK_CTRL, PRCM_CKCLD_FACTOR_N_MASK, PRCM_CKCLD_FACTOR_N(n));
}

#endif

void HAL_PRCM_SetDevClock(PRCM_DevClkFactor factor)
{
	HAL_MODIFY_REG(PRCM->DEV_CLK_CTRL, PRCM_DEV1_FACTOR_N_MASK | PRCM_DEV1_FACTOR_M_MASK, factor);
}

uint32_t rom_HAL_PRCM_GetDevClock(void)
{
	uint32_t div;
	uint32_t divm, divn;

	divm = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV_CLK_FACTORM_SHIFT,
	                       PRCM_DEV_CLK_FACTORM_VMASK) + PRCM_DEV_CLK_FACTORM_START;
	divn = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV_CLK_FACTORN_SHIFT,
	                       PRCM_DEV_CLK_FACTORN_VMASK) + 1;
	div = divm * divn;

	return (SYS_PLL_CLOCK / div);
}

#if (CONFIG_CHIP_ARCH_VER == 2)
void HAL_PRCM_SetDev2Clock(PRCM_Dev2ClkFactor factor)
{
	HAL_MODIFY_REG(PRCM->DEV_CLK_CTRL, PRCM_DEV2_FACTOR_N_MASK | PRCM_DEV2_FACTOR_M_MASK, factor);
}

void HAL_PRCM_EnableDev2Clock(void)
{
	HAL_SET_BIT(PRCM->DEV_CLK_CTRL, PRCM_DEV2_CLK_EN_BIT);
}

void HAL_PRCM_DisableDev2Clock(void)
{
	HAL_CLR_BIT(PRCM->DEV_CLK_CTRL, PRCM_DEV2_CLK_EN_BIT);
}

uint32_t HAL_PRCM_GetDev2Clock(void)
{
	uint32_t divm, divn;

	divm = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV2_FACTOR_M_SHIFT,
	                       PRCM_DEV2_FACTOR_M_VMASK) + 5;
	divn = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV2_FACTOR_N_SHIFT,
	                       PRCM_DEV2_FACTOR_N_VMASK) + 1;
	return (SYS_PLL_CLOCK / divm / divn);
}

__xip_text
void HAL_PRCM_SetAudioPLLPatternParam(PRCM_AudPLLPatParam param)
{
	PRCM->AUD_PLL_PAT_CTRL = param; /* NB: it will disable system PLL */
}

__xip_text
void HAL_PRCM_EnableAudioPLLPattern(void)
{
	HAL_SET_BIT(PRCM->AUD_PLL_PAT_CTRL, PRCM_AUD_DIG_DELT_PAT_EN_BIT);
}

__xip_text
void HAL_PRCM_DisableAudioPLLPattern(void)
{
	HAL_CLR_BIT(PRCM->AUD_PLL_PAT_CTRL, PRCM_AUD_DIG_DELT_PAT_EN_BIT);
}

void HAL_PRCM_EnableHXTALOUT(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->HXTALOUT_CTRL, PRCM_HXTALOUT_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->HXTALOUT_CTRL, PRCM_HXTALOUT_EN_BIT);
}
#endif

void HAL_PRCM_ForceSys3Reset(void)
{
	HAL_CLR_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
}

void HAL_PRCM_ReleaseSys3Reset(void)
{
	HAL_SET_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
}

int HAL_PRCM_IsSys3Release(void)
{
	return HAL_GET_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
}

__xip_text
int HAL_PRCM_IsSys3Alive(void)
{
	return HAL_GET_BIT(PRCM->SYS3_STATUS, PRCM_SYS3_ALIVE_BIT);
}

void HAL_PRCM_SetSys1WakeupPowerFlags(uint32_t flags)
{
	PRCM->SYS1_WAKEUP_CTRL = flags & PRCM_SYS_WS_PWR_FLAGS_MASK;
}

uint32_t HAL_PRCM_GetSys1WakeupPowerFlags(void)
{
	return (PRCM->SYS1_WAKEUP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

uint32_t HAL_PRCM_GetSys1SleepPowerFlags(void)
{
	return (PRCM->SYS1_SLEEP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

void HAL_PRCM_SetSys1SleepPowerFlags(uint32_t flags)
{
	HAL_MODIFY_REG(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_WS_PWR_FLAGS_MASK, flags & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

void HAL_PRCM_SetBANDGAPSTABLE_TIME(uint32_t time)
{
	PRCM->BANDGAP_STABLE_REF_TIME = (time & PRCM_BANDGAP_STABLE_REF_TIME_MASK);
}

void HAL_PRCM_SetRTCLDOVoltage(PRCM_RTCLDORetentionVolt retenVolt, PRCM_RTCLDOWorkVolt workVolt)
{
	PRCM->RTC_LDO_VOLT_CTRL = retenVolt | workVolt;
}

uint32_t HAL_PRCM_GetBANDGAPSTABLE_TIME(void)
{
	return (PRCM->BANDGAP_STABLE_REF_TIME & PRCM_BANDGAP_STABLE_REF_TIME_MASK);
}

void HAL_PRCM_SetDCDCSTABLE_TIME(uint32_t time)
{
	PRCM->DCDC_STABLE_REF_TIME = (time & PRCM_DCDC_STABLE_REF_TIME_VMASK) << PRCM_DCDC_STABLE_RFE_TIME_SHIFT;
}

uint32_t HAL_PRCM_GetDCDCSTABLE_TIME(void)
{
	return (PRCM->DCDC_STABLE_REF_TIME & (PRCM_DCDC_STABLE_REF_TIME_VMASK >> PRCM_DCDC_STABLE_RFE_TIME_SHIFT));
}

#if (CONFIG_CHIP_ARCH_VER == 3)

__xip_text
void HAL_PRCM_ReleaseBLEReset(void)
{
	HAL_SET_BIT(PRCM->BLE_RTC_RST_CTRL, PRCM_BLE_RTC_RST_RST_BIT);
}

__xip_text
void HAL_PRCM_ForceBLEReset(void)
{
	HAL_CLR_BIT(PRCM->BLE_RTC_RST_CTRL, PRCM_BLE_RTC_RST_RST_BIT);
}

__xip_text
uint32_t HAL_PRCM_EnableBLE32MClk(void)
{
	return HAL_SET_BIT(PRCM->BLE_RTC_CLK_CTRL, PRCM_BLE_32M_RST_CLK_EN_BIT);
}

__xip_text
uint32_t HAL_PRCM_DisableBLE32MClk(void)
{
	return HAL_CLR_BIT(PRCM->BLE_RTC_CLK_CTRL, PRCM_BLE_32M_RST_CLK_EN_BIT);
}

__xip_text
uint32_t HAL_PRCM_EnableBLE48MClk(void)
{
	return HAL_SET_BIT(PRCM->BLE_RTC_CLK_CTRL, PRCM_BLE_48M_RST_CLK_EN_BIT);
}

__xip_text
uint32_t HAL_PRCM_DisableBLE48MClk(void)
{
	return HAL_CLR_BIT(PRCM->BLE_RTC_CLK_CTRL, PRCM_BLE_48M_RST_CLK_EN_BIT);
}

__xip_text
void HAL_PRCM_ReleaseRFASReset(void)
{
	HAL_SET_BIT(PRCM->RFAS_CTRL_REG, PRCM_BLE_RFAS_RST_BIT);
}

__xip_text
void HAL_PRCM_ForceRFASReset(void)
{
	HAL_CLR_BIT(PRCM->RFAS_CTRL_REG, PRCM_BLE_RFAS_RST_BIT);
}

#endif

void HAL_PRCM_SetCPUABootFlag(PRCM_CPUABootFlag flag)
{
	PRCM->CPUA_BOOT_FLAG = PRCM_CPUA_BOOT_FLAG_WR_LOCK | flag;
}

uint32_t HAL_PRCM_GetCPUABootFlag(void)
{
	return HAL_GET_BIT(PRCM->CPUA_BOOT_FLAG, PRCM_CPUA_BOOT_FLAG_MASK);
}

void HAL_PRCM_SetCPUABootAddr(uint32_t addr)
{
	PRCM->CPUA_BOOT_ADDR = addr;
}

uint32_t HAL_PRCM_GetCPUABootAddr(void)
{
	return PRCM->CPUA_BOOT_ADDR;
}

void HAL_PRCM_SetCPUABootArg(uint32_t arg)
{
	PRCM->CPUA_BOOT_ARG = arg;
}

uint32_t HAL_PRCM_GetCPUABootArg(void)
{
	return PRCM->CPUA_BOOT_ARG;
}

void HAL_PRCM_SetCPUAPrivateData(uint32_t id, uint32_t data)
{
	if (id < PRCM_CPUA_PRIV_DATA_ID_NUM) {
		PRCM->CPUA_PRIV_REG_0T3[id] = data;
	}
}

uint32_t HAL_PRCM_GetCPUAPrivateData(uint32_t id)
{
	if (id < PRCM_CPUA_PRIV_DATA_ID_NUM) {
		return PRCM->CPUA_PRIV_REG_0T3[id];
	} else {
		return 0;
	}
}

void HAL_PRCM_SetPrivateTime(uint64_t val)
{
	PRCM->CPUA_PRIV_TIME_L = (uint32_t)(val & 0xffffffff);
	PRCM->CPUA_PRIV_TIME_H = (uint32_t)((val >> 32) & 0xffffffff);
}

uint64_t HAL_PRCM_GetPrivateTime(void)
{
	return (((uint64_t)PRCM->CPUA_PRIV_TIME_H << 32) | PRCM->CPUA_PRIV_TIME_L);
}

uint32_t HAL_PRCM_GetWakeupTimerEnable(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

void HAL_PRCM_WakeupTimerEnable(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

void HAL_PRCM_WakeupTimerDisable(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

uint32_t HAL_PRCM_WakeupTimerGetCurrentValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_CUR_VAL_MASK);
}

uint32_t HAL_PRCM_GetWakeupTimerPending(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_PENDING_BIT);
}

void HAL_PRCM_ClearWakeupTimerPending(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_TIMER_CMP, PRCM_CPUx_WAKE_TIMER_PENDING_BIT);
}

void HAL_PRCM_WakeupTimerSetCompareValue(uint32_t val)
{
	PRCM->CPUA_WAKE_TIMER_CMP = val & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK;
}

uint32_t HAL_PRCM_WakeupTimerGetCompareValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}

void HAL_PRCM_WakeupIOEnable(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

void HAL_PRCM_WakeupIODisable(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

#if (CONFIG_CHIP_ARCH_VER == 2)
void HAL_PRCM_WakeupIOSetRisingEvent(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}

void HAL_PRCM_WakeupIOSetFallingEvent(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}
#elif (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PRCM_WakeupIOSetRisingEvent(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask << PRCM_WAKEUP_IOx_MODE_SHIFT);
}

void HAL_PRCM_WakeupIOSetFallingEvent(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask << PRCM_WAKEUP_IOx_MODE_SHIFT);
}
#endif

uint32_t HAL_PRCM_WakeupIOGetEventStatus(void)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, WAKEUP_IO_MASK);
}

int HAL_PRCM_WakeupIOIsEventDetected(uint32_t ioMask)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_WakeupIOClearEventDetected(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_WakeupIOEnableCfgHold(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_WakeupIODisableCfgHold(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_WakeupIOEnableGlobal(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}

void HAL_PRCM_WakeupIODisableGlobal(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}

void HAL_PRCM_EnableWakeupIOx(uint8_t ioIndex, uint8_t enable)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	if (enable)
		HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, 1 << ioIndex);
	else
		HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, 1 << ioIndex);
}

#if (CONFIG_CHIP_ARCH_VER == 2)
void HAL_PRCM_SetWakeupIOxDebouce(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	if (ioIndex < 4) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_EN,
		               PRCM_WAKEUP_IO0T3_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO0T3_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	} else if (ioIndex < WAKEUP_IO_MAX) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_GLOBAL_EN,
		               PRCM_WAKEUP_IO4T9_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO4T9_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	}
}

void HAL_PRCM_SetWakeupIOxDebSrc(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	if (val)
		HAL_SET_BIT(PRCM->CPUA_WAKE_IO_MODE, 1 << (ioIndex + PRCM_WAKEUP_IOX_DEB_CLK_SRC_SHIFT));
	else
		HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_MODE, 1 << (ioIndex + PRCM_WAKEUP_IOX_DEB_CLK_SRC_SHIFT));
}

void HAL_PRCM_SetWakeupDebClk0(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_MODE, PRCM_WKAEUP_DEB_CLK0_MASK, val << PRCM_WKAEUP_DEB_CLK0_SHIFT);
}

void HAL_PRCM_SetWakeupDebClk1(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_MODE, PRCM_WKAEUP_DEB_CLK1_MASK, val << PRCM_WKAEUP_DEB_CLK1_SHIFT);
}

void HAL_PRCM_SetWakeupIOxDebounce(uint8_t ioIndex, uint8_t val)
{
}
#elif (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PRCM_SetWakeupIOxDebouce(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	if (ioIndex < 8) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_DEB_CYCLES0,
		               PRCM_WAKEUP_IO0T7_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO0T7_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	} else if (ioIndex < WAKEUP_IO_MAX) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_DEB_CYCLES1,
		               PRCM_WAKEUP_IO8T13_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO8T13_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	}
}

void HAL_PRCM_SetWakeupIOxDebSrc(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	if (val)
		HAL_SET_BIT(PRCM->CPUA_WAKE_IO_DEB_CLK, 1 << ioIndex);
	else
		HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_DEB_CLK, 1 << ioIndex);
}

void HAL_PRCM_SetWakeupDebClk0(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_DEB_CLK, PRCM_WKAEUP_DEB_CLK0_MASK, val << PRCM_WKAEUP_DEB_CLK0_SHIFT);
}

void HAL_PRCM_SetWakeupDebClk1(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_DEB_CLK, PRCM_WKAEUP_DEB_CLK1_MASK, val << PRCM_WKAEUP_DEB_CLK1_SHIFT);
}

void HAL_PRCM_SetWakeupIOxDebounce(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < WAKEUP_IO_MAX);

	//TODO: define here
}
#endif

static uint8_t __ext_flash_flg = 0;
int HAL_PRCM_IsFlashSip(void)
{
	if (__ext_flash_flg) {
		return 0;
	}
	return HAL_GET_BIT(PRCM->BONDING_IO, PRCM_FLASH_SIP_EN_BIT);
}

void HAL_PRCM_SetFlashExt(uint8_t ext)
{
	__ext_flash_flg = ext;
}

#if (CONFIG_CHIP_ARCH_VER == 2)
uint32_t HAL_PRCM_GetFlashSipMode(void)
{
	return HAL_GET_BIT(PRCM->BONDING_IO, PRCM_FLASH_SIP_MODE_MASK);
}
#endif

uint32_t HAL_PRCM_GetResetSource(void)
{
	return PRCM->CPU_RESET_SOURCE;
}

void HAL_PRCM_ClrResetSource(void)
{
	HAL_SET_BIT(PRCM->CPU_RESET_SOURCE, PRCM_CPU_IS_PWRON_RST_BIT | PRCM_CPU_IS_PMU_RST_BIT | \
	            PRCM_CPU_IS_WATCHDOG_ALL_RST_BIT | PRCM_CPU_IS_WATCHDOG_CPU_RST_MASK);
}

void HAL_PRCM_SetWdgNoResetPeriph(uint32_t periphMask, int8_t enable)
{
	if (enable) {
		HAL_SET_BIT(PRCM->WDG_NORESET_PERIPH, periphMask);
	} else {
		HAL_CLR_BIT(PRCM->WDG_NORESET_PERIPH, periphMask);
	}

	if (PRCM->WDG_NORESET_PERIPH & PRCM_WDG_NORESET_PERIPH_MASK) {
		HAL_SET_BIT(PRCM->WDG_NORESET_PERIPH, PRCM_WDG_NORESET_PERIPH_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->WDG_NORESET_PERIPH, PRCM_WDG_NORESET_PERIPH_EN_BIT);
	}
}

void HAL_PRCM_SetDigSWRefTime(uint32_t val)
{
	HAL_MODIFY_REG(PRCM->DIG_SWITCH_REF_TIME, PRCM_RESET_UP_REF_TIME_MASK, val << PRCM_RESET_UP_REF_TIME_SHIFT);
}

void HAL_PRCM_EnableLDOModeSWSelEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_LDO_MODE_SW_SEL_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_LDO_MODE_SW_SEL_EN_BIT);
}

void HAL_PRCM_EnableSysLDOLQModeEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_SYS_LDO_LQ_MODE_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_SYS_LDO_LQ_MODE_BIT);
}

void HAL_PRCM_EnableTOPLDOLQModeEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_TOP_LDO_LQ_MODE_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_TOP_LDO_LQ_MODE_BIT);
}

void HAL_PRCM_EnableTOPLDODeepsleep(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->DCDC_PARAM_CTRL, PRCM_TOP_LDO_DEEPSLEEP_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->DCDC_PARAM_CTRL, PRCM_TOP_LDO_DEEPSLEEP_EN_BIT);
}

void HAL_PRCM_EnableWlanCPUClk(uint8_t enable)
{
	if (enable)
		HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_BIT); /* 0 is enable */
	else
		HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_BIT);
}

void HAL_PRCM_ReleaseWlanCPUReset(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_BIT);
}

void HAL_PRCM_ForceWlanCPUReset(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_BIT);
}

void HAL_PRCM_WakeUpWlan(uint8_t wakeup)
{
	if (wakeup)
		HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_BIT);
	else
		HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_BIT);
}

void HAL_PRCM_EnableWlanCPUClkOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanCPUClkOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_OVR_HIF_BIT);
}

void HAL_PRCM_ReleaseWlanCPUOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_OVR_HIF_BIT);
}

void HAL_PRCM_ResetWlanCPUOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_OVR_HIF_BIT);
}

void HAL_PRCM_EnableWlanWUPOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanWUPOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_OVR_HIF_BIT);
}

void HAL_PRCM_EnableWlanIRQOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_IRQ_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanIRQOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_IRQ_OVR_HIF_BIT);
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PRCM_SetWlanSramShare(PRCM_WLAN_ShareSramType type)
{
	HAL_MODIFY_REG(PRCM->WLAN_SRAM_SHARE_CTRL, PRCM_WLAN_SHARE_SRAM_MASK, type);
}

void HAL_PRCM_SetBLESramShare(uint32_t en)
{
	if (en) {
		HAL_SET_BIT(PRCM->BLE_16K_SRAM_SHARE_CTRL, PRCM_BLE_16K_SHARE_SRAM_BIT);
	} else {
		HAL_CLR_BIT(PRCM->BLE_16K_SRAM_SHARE_CTRL, PRCM_BLE_16K_SHARE_SRAM_BIT);
	}
}

__nonxip_text
void HAL_PRCM_EnableLpuart0Clk(void)
{
	HAL_SET_BIT(PRCM->LPUART0_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_EN_BIT);
}

__nonxip_text
void HAL_PRCM_DisableLpuart0Clk(void)
{
	HAL_CLR_BIT(PRCM->LPUART0_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_EN_BIT);
}

__nonxip_text
void HAL_PRCM_SelectLpuart0WakeupIOIn(PRCM_LPUART_WAKEUP_IN_SEL in)
{
	HAL_MODIFY_REG(PRCM->LPUART0_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_IN_SEL_MASK, in << PRCM_LPUART_WAKEUP_IN_SEL_SHIFT);
}

void HAL_PRCM_SelectLpuart0ClkSource(PRCM_LPUART_ClkSource sel)
{
	HAL_MODIFY_REG(PRCM->LPUART0_WAKEUP_CTRL, PRCM_LPUART_CLK_SOURCE_MASK, sel);
}

__nonxip_text
void HAL_PRCM_EnableLpuart1Clk(void)
{
	HAL_SET_BIT(PRCM->LPUART1_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_EN_BIT);
}

__nonxip_text
void HAL_PRCM_DisableLpuart1Clk(void)
{
	HAL_CLR_BIT(PRCM->LPUART1_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_EN_BIT);
}

__nonxip_text
void HAL_PRCM_SelectLpuart1WakeupIOIn(PRCM_LPUART_WAKEUP_IN_SEL in)
{
	HAL_MODIFY_REG(PRCM->LPUART1_WAKEUP_CTRL, PRCM_LPUART_WAKEUP_IN_SEL_MASK, in << PRCM_LPUART_WAKEUP_IN_SEL_SHIFT);
}

void HAL_PRCM_SelectLpuart1ClkSource(PRCM_LPUART_ClkSource sel)
{
	HAL_MODIFY_REG(PRCM->LPUART1_WAKEUP_CTRL, PRCM_LPUART_CLK_SOURCE_MASK, sel);
}

void HAL_PRCM_GPADC_EnableMClock(void)
{
	HAL_SET_BIT(PRCM->GPADC_CLK_CTRL, PRCM_GPADC_MCLK_EN_BIT);
}

void HAL_PRCM_GPADC_DisableMClock(void)
{
	HAL_CLR_BIT(PRCM->GPADC_CLK_CTRL, PRCM_GPADC_MCLK_EN_BIT);
}

uint32_t HAL_PRCM_GPADC_GetMClock(void)
{
	if (HAL_GET_BIT(PRCM->GPADC_CLK_CTRL, PRCM_GPADC_MCLK_SOURCE_MASK)) {
		return HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS);
	} else {
		return HAL_GetHFClock();
	}
}

void HAL_PRCM_SelectGpadcClkSource(PRCM_GPADC_MClkSource sel)
{
	HAL_MODIFY_REG(PRCM->GPADC_CLK_CTRL, PRCM_GPADC_MCLK_SOURCE_MASK, sel);
}

void HAL_PRCM_SetGpadcClock(PRCM_GPADC_ClkFactorN factorN, uint16_t factorM)
{
	HAL_MODIFY_REG(PRCM->GPADC_CLK_CTRL, PRCM_GPADC_FACTOR_N_MASK | PRCM_GPADC_CLK_FACTORM_MASK,
	               factorN | PRCM_GPADC_CLK_FACTORM_VALUE(factorM));
}

__nonxip_text
void HAL_PRCM_WAKEUP_SRC_EnableClkGating(PRCM_WAKEUP_SRC_BusClkBit periphMask)
{
	HAL_SET_BIT(PRCM->WAKEUP_SRC_BUS_CLK_CTRL, periphMask);
}

__nonxip_text
void HAL_PRCM_WAKEUP_SRC_DisableClkGating(PRCM_WAKEUP_SRC_BusClkBit periphMask)
{
	HAL_CLR_BIT(PRCM->WAKEUP_SRC_BUS_CLK_CTRL, periphMask);
}

/**
 * @brief Release wakeup source reset
 * @param[in] Bitmask of wakeup source, refer to PRCM_WAKEUP_SRC_RstBit
 * @return None
 */
__nonxip_text
void HAL_PRCM_ReleaseWakeupSrcReset(PRCM_WAKEUP_SRC_RstBit periphMask)
{
	HAL_SET_BIT(PRCM->WAKEUP_SRC_RST_CTRL, periphMask);
}

/**
 * @brief Force wakeup source reset
 * @param[in] Bitmask of wakeup source, refer to PRCM_WAKEUP_SRC_RstBit
 * @return None
 */
void HAL_PRCM_ForceWakeupSrcReset(PRCM_WAKEUP_SRC_RstBit periphMask)
{
	HAL_CLR_BIT(PRCM->WAKEUP_SRC_RST_CTRL, periphMask);
}

/**
 * @brief get LPUART module clock
 * @param[in] void
 * @return None
 */
uint32_t HAL_PRCM_GetLpuartMClock(void)
{
	return HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS);
}

void HAL_PRCM_SetFlashCryptoNonce(uint8_t *nonce)
{
	PRCM->FLASH_ENCRYPT_AES_NONCE1 = HAL_REG_16BIT(nonce);
	PRCM->FLASH_ENCRYPT_AES_NONCE0 = HAL_REG_32BIT(&nonce[2]);
}

void HAL_PRCM_SetCLK32kDiv(uint32_t en, PRCM_BLE_CLK32K_DivSrcSel sel)
{
	uint32_t div;

	if (en) {
		if (sel == PRCM_BLE_CLK32K_DIV_SRC_SEL_32M) {
			div = PRCM_BLE_CLK32K_DIV_HALFCYCLE_32M;
		} else {
			div = HAL_GetHFClock() / (32 * 1000) / 2 - 1;
		}
		HAL_MODIFY_REG(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_DIV_MASK,
			       PRCM_BLE_CLK32K_DIV_VALUE(div));
		HAL_MODIFY_REG(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_DIV_SRC_SEL_MASK, sel);
		HAL_SET_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_DIV_CLK_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_DIV_CLK_EN_BIT);
	}
}

void HAL_PRCM_SetCLK32kAutoSw(uint32_t en)
{
	if (en) {
		HAL_SET_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_AUTO_SW_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->BLE_CLK32K_SWITCH0, PRCM_BLE_CLK32K_AUTO_SW_EN_BIT);
	}
}

void HAL_PRCM_SetRcoCalib(uint32_t en, PRCM_RCOSC_WkModSel mode, PRCM_RCOSC_NormalWkTimesSel sel,
                          uint32_t phase, uint32_t wk_time_en, uint32_t wk_time)
{
	if (en) {
		uint32_t mask;

		/* method */
		mask = PRCM_RCOSC_WK_MODE_SEL_MASK | PRCM_RCOSC_NORMAL_WK_TIMES_SEL_MASK |
		       PRCM_RCOSC_SCALE_PHASE1_NUM_MASK | PRCM_RCOSC_SCALE_PHASE2_NUM_SHIFT |
		       PRCM_RCOSC_SCALE_PHASE2_WK_TIMES_MASK | PRCM_RCOSC_SCALE_PHASE3_WK_TIMES_MASK;
		HAL_MODIFY_REG(PRCM->BLE_RCOSC_CALIB_CTRL1, mask, mode | sel | phase);

		HAL_SET_BIT(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_BLE_RCOSC_CALIB_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_BLE_RCOSC_CALIB_EN_BIT);
	}

	if (wk_time_en) {
		/* eg. phase3 = wup_time * PRCM_RCOSC_SCALE_PHASE3_WUP_TIMES_10 = 2.5ms,
		 *     8 / 32000 = 0.25ms
		 */
		HAL_MODIFY_REG(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_RCOSC_WK_TIME_MASK, wk_time << PRCM_RCOSC_WK_TIME_SHIFT);
		HAL_SET_BIT(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_RCOSC_WK_TIME_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_RCOSC_WK_TIME_EN_BIT);
	}
}

#endif
