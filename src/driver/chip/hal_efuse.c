/**
 * @file  hal_efuse.c
 * @author  XRADIO IOT WLAN Team
 */

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

#include "driver/chip/hal_efuse.h"
#include "hal_base.h"

#ifdef CONFIG_ROM

typedef enum {
	EFUSE_STATE_INVALID = 0,
	EFUSE_STATE_READY   = 1,
	EFUSE_STATE_BUSY    = 2
} EFUSE_State;

EFUSE_State gEfuseState = EFUSE_STATE_INVALID;


#ifndef CONFIG_TZ_EFUSE

__STATIC_INLINE void EFUSE_EnableClkGate(void)
{
	HAL_SET_BIT(EFUSE->CTRL, EFUSE_CLK_GATE_EN_BIT);
}

__STATIC_INLINE void EFUSE_DisableClkGate(void)
{
	HAL_CLR_BIT(EFUSE->CTRL, EFUSE_CLK_GATE_EN_BIT);
}

__STATIC_INLINE void EFUSE_SetIndex(uint32_t index)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_INDEX_MASK,
	               HAL_GET_BIT(index << EFUSE_INDEX_SHIFT, EFUSE_INDEX_MASK));
}

#if (CONFIG_CHIP_ARCH_VER == 3)
__STATIC_INLINE void EFUSE_SetUpdate(void)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_OPERA_LOCK_MASK,
	               (EFUSE_OPERA_UNLOCK_VAL << EFUSE_OPERA_LOCK_SHIFT) | EFUSE_UPDATE_ALL_BITS_BIT);
}

__STATIC_INLINE void EFUSE_WaitUpdateFinish(void)
{
	while (HAL_GET_BIT(EFUSE->CTRL, EFUSE_UPDATE_ALL_BITS_BIT))
		;
}

__STATIC_INLINE void EFUSE_ClearUpdate(void)
{
	HAL_SET_BIT(EFUSE->CTRL, EFUSE_UPDATE_ALL_BITS_BIT);
}
#endif

__STATIC_INLINE uint32_t EFUSE_GetHwReadStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_HW_READ_STATUS_BIT);
}

__STATIC_INLINE void EFUSE_StartRead(void)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_OPERA_LOCK_MASK,
	               (EFUSE_OPERA_UNLOCK_VAL << EFUSE_OPERA_LOCK_SHIFT) | EFUSE_SW_READ_START_BIT);
}

__STATIC_INLINE uint32_t EFUSE_GetSwReadStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_SW_READ_START_BIT);
}

__STATIC_INLINE void EFUSE_StartProgram(void)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_OPERA_LOCK_MASK,
	               (EFUSE_OPERA_UNLOCK_VAL << EFUSE_OPERA_LOCK_SHIFT) | EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE uint32_t EFUSE_GetSwProgStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE void EFUSE_ClrCtrlReg(void)
{
	HAL_CLR_BIT(EFUSE->CTRL, EFUSE_INDEX_MASK |
	            EFUSE_OPERA_LOCK_MASK |
	            EFUSE_SW_READ_START_BIT |
	            EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE void EFUSE_SetProgValue(uint32_t value)
{
	EFUSE->PROGRAM_VALUE = value;
}

__STATIC_INLINE uint32_t EFUSE_GetReadValue(void)
{
	return EFUSE->READ_VALUE;
}

__STATIC_INLINE void EFUSE_SetTimingParam(EFUSE_TimingParam timingParam)
{
	EFUSE->TIMING_CTRL = timingParam;
}


HAL_Status EFUSE_Init(void)
{
	uint32_t clk;
	EFUSE_TimingParam timingParam;

	clk = HAL_GetHFClock();

	if (clk == HOSC_CLOCK_24M) {
		timingParam = EFUSE_TIMING_PARAM_24M;
	} else if (clk == HOSC_CLOCK_26M) {
		timingParam = EFUSE_TIMING_PARAM_26M;
	} else if (clk == HOSC_CLOCK_32M) {
		timingParam = EFUSE_TIMING_PARAM_32M;
	} else if (clk == HOSC_CLOCK_40M) {
		timingParam = EFUSE_TIMING_PARAM_40M;
	} else {
		HAL_ERR("unsupport HOSC %u\n", clk);
		return HAL_ERROR;
	}
	EFUSE_SetTimingParam(timingParam);

	return HAL_OK;
}

void EFUSE_ReadData(uint8_t index, uint32_t *pData)
{
#if 0
	EFUSE_EnableClkGate();
	EFUSE_SetIndex(index << 2);
	EFUSE_StartRead();

	while (EFUSE_GetSwReadStatus())
		;

	while (EFUSE_GetHwReadStatus())
		;

	*pData = EFUSE_GetReadValue();
	EFUSE_ClrCtrlReg();
	EFUSE_DisableClkGate();
#else
	*pData = EFUSE->VALUE[index];
#endif
}

void EFUSE_WriteData(uint8_t index, uint32_t data)
{
	EFUSE_EnableClkGate();
	EFUSE_SetIndex(index << 2);
	EFUSE_SetProgValue(data);
	EFUSE_StartProgram();

	while (EFUSE_GetSwProgStatus())
		;

	EFUSE_ClrCtrlReg();
	EFUSE_DisableClkGate();
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void EFUSE_UpdateAll(void)
{
	EFUSE_EnableClkGate();
	EFUSE_SetUpdate();
	EFUSE_WaitUpdateFinish();

	EFUSE_ClrCtrlReg();
	EFUSE_DisableClkGate();
}
#endif

#endif /* CONFIG_TZ_EFUSE */

HAL_Status HAL_EFUSE_Read(uint32_t start_bit, uint32_t bit_num, uint8_t *data)
{
	unsigned long flags;

	if ((data == NULL) ||
	    (start_bit >= HAL_EFUSE_BIT_NUM) ||
	    (bit_num == 0) ||
	    (bit_num > HAL_EFUSE_BIT_NUM) ||
	    (start_bit + bit_num > HAL_EFUSE_BIT_NUM)) {
		HAL_ERR("start bit %u, bit num %u, data %p\n", start_bit, bit_num, data);
		return HAL_ERROR;
	}

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_INVALID) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
		if (EFUSE_Init() != HAL_OK) {
			flags = HAL_EnterCriticalSection();
			gEfuseState = EFUSE_STATE_INVALID;
			HAL_ExitCriticalSection(flags);
			return HAL_ERROR;
		}
	} else if (gEfuseState == EFUSE_STATE_READY) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("EFUSE state %d\n", gEfuseState);
		return HAL_BUSY;
	}

	uint32_t start_word = start_bit / 32;
	uint8_t word_shift = start_bit % 32;
	uint8_t word_cnt = (start_bit + bit_num) / 32 - start_word;
	if ((start_bit + bit_num) % 32)
		word_cnt++;

	uint32_t *sid_data = HAL_Malloc(word_cnt * 4);
	int i;
	for (i = 0; i < word_cnt; i++) {
		EFUSE_ReadData(start_word + i, sid_data + i);
	}
	for (i = 0; i < (word_cnt - 1); i++) {
		sid_data[i] = (sid_data[i] >> word_shift) | (sid_data[i + 1] << (32 - word_shift));
	}
	sid_data[i] = (sid_data[i] >> word_shift);
	((uint8_t *)sid_data)[(bit_num - 1) / 8] &= ((1 << (bit_num % 8 == 0 ? 8 : bit_num % 8)) - 1);
	HAL_Memcpy(data, (uint8_t *)sid_data, (bit_num + 7) / 8);

	HAL_Free(sid_data);
	flags = HAL_EnterCriticalSection();
	gEfuseState = EFUSE_STATE_READY;
	HAL_ExitCriticalSection(flags);

	return HAL_OK;
}

HAL_Status HAL_EFUSE_Write(uint32_t start_bit, uint32_t bit_num, uint8_t *data)
{
	unsigned long flags;

	if ((data == NULL) ||
	    (start_bit >= HAL_EFUSE_BIT_NUM) ||
	    (bit_num == 0) ||
	    (bit_num > HAL_EFUSE_BIT_NUM) ||
	    (start_bit + bit_num > HAL_EFUSE_BIT_NUM)) {
		HAL_ERR("start bit %u, bit num %u, data %p\n", start_bit, bit_num, data);
		return HAL_ERROR;
	}

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_INVALID) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
		if (EFUSE_Init() != HAL_OK) {
			flags = HAL_EnterCriticalSection();
			gEfuseState = EFUSE_STATE_INVALID;
			HAL_ExitCriticalSection(flags);
			return HAL_ERROR;
		}
	} else if (gEfuseState == EFUSE_STATE_READY) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("EFUSE state %d\n", gEfuseState);
		return HAL_BUSY;
	}

	uint8_t *p_data = data;
	uint32_t bit_shift = start_bit & (32 - 1);
	uint32_t word_idx = start_bit >> 5;

	uint64_t buf = 0;
	uint32_t *efuse_word = (uint32_t *)&buf;
	uint32_t bit_cnt = bit_num;

	data[(bit_num - 1) / 8] &= ((1 << (bit_num % 8 == 0 ? 8 : bit_num % 8)) - 1);
	HAL_Memcpy(&efuse_word[1], p_data, sizeof(efuse_word[1]));
	if (bit_cnt < 32)
		efuse_word[1] &= (1U << bit_cnt) - 1;
	efuse_word[1] = efuse_word[1] << bit_shift;

#if (CONFIG_CHIP_ARCH_VER == 2)
	PRCM_SysClkFactor old_sysclk;
	old_sysclk = HAL_GET_BIT(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_FACTOR_MASK);
#endif
#ifdef CONFIG_PWR_INTERNAL_DCDC
	uint32_t old_volt;
	old_volt = HAL_PRCM_GetSMPSVoltage();
#elif (defined CONFIG_PWR_INTERNAL_LDO)
	uint32_t old_volt;
	old_volt = HAL_PRCM_GetEXTLDOVolt();
#elif (defined CONFIG_PWR_EXTERNAL)
	HAL_Status ret;
#endif
	HAL_ThreadSuspendScheduler();
	HAL_PRCM_DisableVddioSipSw();
	HAL_UDelay(100);
#if (CONFIG_CHIP_ARCH_VER == 2)
	if (old_sysclk != PRCM_SYS_CLK_FACTOR_160M) {
		HAL_PRCM_SetCPUAClk(PRCM_CPU_CLK_SRC_SYSCLK, PRCM_SYS_CLK_FACTOR_160M);
	}
#endif
#ifdef CONFIG_PWR_INTERNAL_DCDC
	HAL_PRCM_SetSMPSVoltage(PRCM_SMPS_VOLT_2V5);
#elif (defined CONFIG_PWR_INTERNAL_LDO)
	HAL_PRCM_SelectEXTLDOVolt(PRCM_EXT_LDO_2V5);
#elif (defined CONFIG_PWR_EXTERNAL)
	ret = HAL_BoardIoctl(HAL_BIR_POWER_CTRL, HAL_MKDEV(HAL_DEV_MAJOR_EFUSE, 0), 1); /* switch to 2.5V */
	if (ret != HAL_OK) {
		HAL_PRCM_EnableVddioSipSw();
		HAL_ThreadResumeScheduler();
		HAL_ERR("Define voltage switch at board config!\n");
		return HAL_ERROR;
	}
#endif
	HAL_UDelay(200);

	EFUSE_WriteData((uint8_t)word_idx, efuse_word[1]);

	word_idx++;
	bit_cnt -= (bit_cnt <= 32 - bit_shift) ? bit_cnt : 32 - bit_shift;

	while (bit_cnt > 0) {
		HAL_Memcpy(&buf, p_data, sizeof(buf));
		buf = buf << bit_shift;
		if (bit_cnt < 32)
			efuse_word[1] &= (1U << bit_cnt) - 1;

		EFUSE_WriteData((uint8_t)word_idx, efuse_word[1]);

		word_idx++;
		p_data += 4;
		bit_cnt -= (bit_cnt <= 32) ? bit_cnt : 32;
	}
#ifdef CONFIG_PWR_INTERNAL_DCDC
	HAL_PRCM_SetSMPSVoltage(old_volt);
#elif (defined CONFIG_PWR_INTERNAL_LDO)
	HAL_PRCM_SelectEXTLDOVolt(old_volt);
#elif (defined CONFIG_PWR_EXTERNAL)
	HAL_BoardIoctl(HAL_BIR_POWER_CTRL, HAL_MKDEV(HAL_DEV_MAJOR_EFUSE, 0), 0); /* switch to old voltage */
#endif
	HAL_UDelay(200);
	HAL_PRCM_EnableVddioSipSw();
	HAL_UDelay(100);
#if (CONFIG_CHIP_ARCH_VER == 2)
	if (old_sysclk != PRCM_SYS_CLK_FACTOR_160M) {
		HAL_PRCM_SetCPUAClk(PRCM_CPU_CLK_SRC_SYSCLK, old_sysclk);
	}
#endif
	HAL_ThreadResumeScheduler();

	flags = HAL_EnterCriticalSection();
	gEfuseState = EFUSE_STATE_READY;
	HAL_ExitCriticalSection(flags);

	HAL_EFUSE_UpdateAll();
	return HAL_OK;
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_EFUSE_UpdateAll(void)
{
	EFUSE_UpdateAll();
}
#endif

#endif /* CONFIG_ROM */
