/**
 * @file  hal_csi.c
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

#if (CONFIG_CHIP_ARCH_VER == 1)

#include "driver/chip/hal_csi.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_dma.h"
#include "hal_base.h"

#define CSI_DEBUG 1

#define HAL_CSI_LOG(flags, fmt, arg...) \
	do {                                \
		if (flags)                      \
			printf(fmt, ##arg);         \
	} while (0)

#define HAL_CSI_DBG(fmt, arg...)    \
	HAL_CSI_LOG(CSI_DEBUG, "[HAL_CSI] "fmt, ##arg)

uint8_t csi_is_run;

void CSI_ModuleEnable(void)
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);
}

void CSI_ModuleDisable(void)
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);
}

void CSI_InputFormat(void)
{
	HAL_CLR_BIT(CSI->CSI_CFG_REG, CSI_CFG_INPUT_FORMAT);
}

static CSI_Call_Back private_csi_cb;

__nonxip_text
static void CSI_IRQHandler(void)
{
	if (private_csi_cb.callBack != NULL)
		private_csi_cb.callBack(private_csi_cb.arg);
}

static void CSI_Irq_Enable(void)
{
	HAL_NVIC_ConfigExtIRQ(CSI_IRQn, CSI_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);
}

static void CSI_Irq_Disable(void)
{
	HAL_NVIC_DisableIRQ(CSI_IRQn);
}

HAL_Status HAL_CSI_Config(CSI_Config *param)
{
	if (csi_is_run) {
		HAL_WRN("%s, %d csi is busy\n", __func__, __LINE__);
		return HAL_BUSY;
	}
	CSI_ModuleEnable();
	HAL_CCM_CSI_SetMClock(param->src_Clk.clk, param->src_Clk.divN, param->src_Clk.divM);
	HAL_CCM_CSI_EnableMClock();

	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_CSI, 0), 0);
	csi_is_run = 1;
	CSI_InputFormat();
	return HAL_OK;
}

void HAL_CSI_DeInit(void)
{
	HAL_CLR_BIT(CSI->CSI_EN_REG, CSI_EN);
	HAL_CCM_CSI_DisableMClock();
	CSI_ModuleDisable();
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_CSI, 0), 0);
	csi_is_run = 0;
}

void HAL_CSI_Moudle_Enalbe(CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_EN_REG, CSI_EN);
	else
		HAL_CLR_BIT(CSI->CSI_EN_REG, CSI_EN);
}

void HAL_CSI_Sync_Signal_Polarity_Cfg(CSI_Sync_Signal *signal)
{
	uint32_t csi_sync_pol;
	csi_sync_pol = signal->vsync + (signal->herf << 1) + (signal->p_Clk << 2);
	HAL_CLR_BIT(CSI->CSI_CFG_REG, CSI_CFG_SYNC_SIGNAL_POL);
	HAL_SET_BIT(CSI->CSI_CFG_REG, csi_sync_pol);
}

void HAL_CSI_Capture_Enable(CSI_CAPTURE_MODE mode, CSI_CTRL ctrl)
{
	HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_CAP_MODE);

	if (CSI_ENABLE == ctrl) {
		if (mode == CSI_STILL_MODE) {
			HAL_SET_BIT(CSI->CSI_CAP_REG, HAL_BIT(0));
		} else if (mode == CSI_VIDEO_MODE)
			HAL_SET_BIT(CSI->CSI_CAP_REG, HAL_BIT(1));
	}
}

void HAL_CSI_Interval_Capture_Cfg(uint8_t ver_mask, uint16_t hor_mask)
{
	HAL_CLR_BIT(CSI->CSI_SCALE_REG, CSI_VER_MASK);
	HAL_CLR_BIT(CSI->CSI_SCALE_REG, CSI_HER_MASK);

	HAL_SET_BIT(CSI->CSI_SCALE_REG, ver_mask << 24);
	HAL_SET_BIT(CSI->CSI_SCALE_REG, hor_mask);
}

void HAL_CSI_Selection_Next_FIFO (CSI_FIFO fifo_num)
{
	HAL_CLR_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(2));
	HAL_SET_BIT(CSI->CSI_BUF_CTL_REG, fifo_num << 2);
}

CSI_FIFO HAL_CSI_Current_FIFO(void)
{
	return  (HAL_GET_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(1)) >> 1);
}

void HAL_CSI_Double_FIFO_Mode_Enable(CSI_CTRL ctrl)
{
	HAL_CLR_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(0));

	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(0));
}

CSI_Status HAL_CSI_Status(void)
{
	CSI_Status sta;
	sta.luminance = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, LUM_STATIS) >> 8;
	sta.still_Mode = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, HAL_BIT(1)) >> 1;
	sta.video_Mode = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, HAL_BIT(0));
	return sta;
}

void HAL_CSI_Interrupt_Cfg(CSI_INTERRUPT_SIGNAL irq_signel, CSI_CTRL ctrl)
{
	if (ctrl == CSI_ENABLE)
		HAL_SET_BIT(CSI->CSI_INT_EN_REG, irq_signel);
	else
		HAL_CLR_BIT(CSI->CSI_INT_EN_REG, irq_signel);
}

__nonxip_text
__IO uint32_t HAL_CSI_Interrupt_Sta(void)
{
	return CSI->CSI_INT_STA_REG;
}

__nonxip_text
void HAL_CSI_Interrupt_Clear(void)
{
	HAL_SET_BIT(CSI->CSI_INT_STA_REG, CSI->CSI_INT_STA_REG);
}

HAL_Status HAL_CSI_Set_Picture_Size(CSI_Picture_Size *size)
{
	if (size->hor_start > (HAL_BIT(14) - 1)) {
		HAL_WRN("%s, %d csi Picture size error hor_start = %d\n",
		        __func__, __LINE__, size->hor_start);
		return HAL_ERROR;
	}

	if (size->hor_len > (HAL_BIT(14) - 1)) {
		HAL_WRN("%s, %d csi Picture size error hor_len = %d\n",
		        __func__, __LINE__, size->hor_len);
		return HAL_ERROR;
	}

	HAL_CLR_BIT(CSI->CSI_HSIZE_REG, CSI_SIZE_REG);

	HAL_SET_BIT(CSI->CSI_HSIZE_REG, size->hor_len << 16);
	HAL_SET_BIT(CSI->CSI_HSIZE_REG, size->hor_start);
	return HAL_OK;
}

__nonxip_text
CSI_FIFO_Data_Len HAL_CSI_FIFO_Data_Len(void)
{
	CSI_FIFO_Data_Len len;
	len.FIFO_0_B_Data_Len = HAL_GET_BIT(CSI->CSI_TRUE_DATA_NUM, (CSI_VALID_DATA_LEN << 16)) >> 16;
	len.FIFO_0_A_Data_Len = HAL_GET_BIT(CSI->CSI_TRUE_DATA_NUM, CSI_VALID_DATA_LEN);
	return len;
}

void HAL_CIS_JPEG_Mode_Enable(CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_JPEG_MOD_SEL,  HAL_BIT(0));
	else
		HAL_CLR_BIT(CSI->CSI_JPEG_MOD_SEL, HAL_BIT(0));
}

void HAL_CSI_Interrupt_Enable(CSI_Call_Back *cb, CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl) {
		if (cb == NULL) {
			private_csi_cb.callBack = NULL;
			private_csi_cb.arg = NULL;
		} else
			private_csi_cb = *cb;
		CSI_Irq_Enable();
	} else {
		private_csi_cb.callBack = NULL;
		private_csi_cb.arg = NULL;
		CSI_Irq_Disable();
	}
}

void CSI_Printf(void)
{
	printf("CSI_EN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_EN_REG, CSI->CSI_EN_REG);
	printf("CSI_CFG_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_CFG_REG, CSI->CSI_CFG_REG);
	printf("CSI_CAP_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_CAP_REG, CSI->CSI_CAP_REG);
	printf("CSI_SCALE_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_SCALE_REG, CSI->CSI_SCALE_REG);
	printf("CSI_BUF_CTL_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BUF_CTL_REG, CSI->CSI_BUF_CTL_REG);
	printf("CSI_BUF_STA_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BUF_STA_REG, CSI->CSI_BUF_STA_REG);
	printf("CSI_INT_EN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_INT_EN_REG, CSI->CSI_INT_EN_REG);
	printf("CSI_INT_STA_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_INT_STA_REG, CSI->CSI_INT_STA_REG);
	printf("CSI_HSIZE_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_HSIZE_REG, CSI->CSI_HSIZE_REG);
	printf("CSI_BF_LEN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BF_LEN_REG, CSI->CSI_BF_LEN_REG);
	printf("CSI_TRUE_DATA_NUM 0x%x 0x%x\n", (uint32_t)&CSI->CSI_TRUE_DATA_NUM, CSI->CSI_TRUE_DATA_NUM);
	printf("CSI_JPEG_MOD_SEL 0x%x 0x%x\n", (uint32_t)&CSI->CSI_JPEG_MOD_SEL, CSI->CSI_JPEG_MOD_SEL);
}
#endif
