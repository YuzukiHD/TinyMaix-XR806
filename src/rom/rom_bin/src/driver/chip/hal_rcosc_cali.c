/**
  * @file  hal_rcosc_cali.c
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

#include "hal_base.h"
#include "rom/driver/chip/hal_rcosc_cali.h"

/**
 * @brief config the RcoscCali according to the specified parameters
 * @param[in] param Pointer to RCOCALI_ConfigParam structure
 * @retval HAL_Status, HAL_OK on success
 */

HAL_Status HAL_RcoscCali_Config(const RCOCALI_ConfigParam *param)
{
	HAL_ASSERT_PARAM(param);
	//phase2 = wup_time*phase2_times = xxxms,phase3 = wup_time*phase3_times = xxxms
	//phase1_num=5, phase2_num=3
	//wup_time/32000=xxxms
	HAL_MODIFY_REG(PRCM->BLE_RCOSC_CALIB_CTRL1, PRCM_RCOSC_WK_MODE_SEL_MASK | PRCM_RCOSC_SCALE_PHASE2_WK_TIMES_MASK | PRCM_RCOSC_SCALE_PHASE3_WK_TIMES_MASK | PRCM_RCOSC_SCALE_PHASE1_NUM_MASK | PRCM_RCOSC_SCALE_PHASE2_NUM_MASK, \
					param->mode | param->phase2_times | param->phase3_times | (param->phase1_num << PRCM_RCOSC_SCALE_PHASE1_NUM_SHIFT) | (param->phase2_num << PRCM_RCOSC_SCALE_PHASE2_NUM_SHIFT));
	HAL_MODIFY_REG(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_RCOSC_WK_TIME_MASK, param->wup_time << PRCM_RCOSC_WK_TIME_SHIFT);
	return 0;
}

/**
 * @brief start the RcoscCali
 * @retval none
 */

void HAL_RcoscCali_Start(void)
{
	HAL_SET_BIT(PRCM->BLE_RCOSC_CALIB_CTRL0, PRCM_RCOSC_WK_TIME_EN_BIT | PRCM_BLE_RCOSC_CALIB_EN_BIT);
	HAL_UDelay(200);//delay 200us
}

/**
 * @brief Initialize the RcoscCali according to the specified parameters
 * @param[in] param Pointer to RCOCALI_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_RcoscCali_Init(const RCOCALI_InitParam *param)
{
	uint32_t dividend;

	HAL_ASSERT_PARAM(param);
	HAL_ASSERT_PARAM(param->out_clk);

	dividend = (uint32_t)(((uint64_t)(param->cnt_n) * HAL_GetHFClock()) / param->out_clk);
	HAL_PRCM_ReleaseWakeupSrcReset(PRCM_WAKEUP_SRC_RST_BIT_RCCAL);
	HAL_PRCM_WAKEUP_SRC_EnableClkGating(PRCM_WAKEUP_SRC_BUS_CLK_BIT_RCCAL);
	HAL_MODIFY_REG(RCOCALI_CTRL->CNT_TARGET, RCOCALI_CNT_TARGET_MASK, param->cnt_n);
	HAL_MODIFY_REG(RCOCALI_CTRL->DIVIDEND, RCOCALI_RCO_DIVIDEND_MASK, dividend);

	return HAL_OK;
}


/**
 * @brief DeInitialize the RcoscCali
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_RcoscCali_DeInit(void)
{
	HAL_PRCM_WAKEUP_SRC_DisableClkGating(PRCM_WAKEUP_SRC_BUS_CLK_BIT_RCCAL);
	HAL_PRCM_ForceWakeupSrcReset(PRCM_WAKEUP_SRC_RST_BIT_RCCAL);

	return HAL_OK;
}
