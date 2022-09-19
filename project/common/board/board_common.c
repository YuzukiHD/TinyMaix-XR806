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

#include "pm/pm.h"
#include "board_common.h"
#include "board.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_rcosc_cali.h"

HAL_Status board_pinmux_cfg(HAL_BoardIoctlReq req,
                            const GPIO_PinMuxParam *pinmux, uint32_t count)
{
	switch (req) {
	case HAL_BIR_PINMUX_INIT:
		HAL_GPIO_PinMuxConfig(pinmux, count);
		break;
	case HAL_BIR_PINMUX_DEINIT:
		HAL_GPIO_PinMuxDeConfig(pinmux, count);
		break;
#ifdef CONFIG_PM
	case HAL_BIR_PINMUX_SUSPEND:
	{
		GPIO_PinMuxParam *_pinmux = HAL_Malloc(sizeof(GPIO_PinMuxParam) * count);
		uint32_t _count = 0;

		if (!_pinmux) {
			printf("malloc faild!\n");
			return HAL_INVALID;
		}
		HAL_GPIO_PinMuxDeConfig(pinmux, count);

		for (int i = 0; i < count; i++) {
			if (pinmux[i].config.pull != GPIO_PULL_NONE) {
				HAL_Memcpy(&_pinmux[_count], &pinmux[i], sizeof(GPIO_PinMuxParam));
				_pinmux[_count].config.mode = GPIOx_Pn_F0_INPUT;
				_count++;
			}
		}
		HAL_GPIO_PinMuxConfig(_pinmux, _count);
		HAL_Free(_pinmux);
		break;
	}
	case HAL_BIR_PINMUX_RESUME:
	{
		GPIO_PinMuxParam *_pinmux = HAL_Malloc(sizeof(GPIO_PinMuxParam) * count);
		uint32_t _count = 0;

		if (!_pinmux) {
			printf("malloc faild!\n");
			return HAL_INVALID;
		}
		for (int i = 0; i < count; i++) {
			if (pinmux[i].config.pull != GPIO_PULL_NONE) {
				HAL_Memcpy(&_pinmux[_count], &pinmux[i], sizeof(GPIO_PinMuxParam));
				_count++;
			}
		}
		HAL_GPIO_PinMuxDeConfig(_pinmux, _count);
		HAL_GPIO_PinMuxConfig(pinmux, count);
		HAL_Free(_pinmux);
		break;
	}
#endif
	default:
		return HAL_INVALID;
	}

	return HAL_OK;
}

#ifndef CONFIG_BOOTLOADER

#ifdef CONFIG_PM
static int lfclk_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		break;
	case PM_MODE_HIBERNATION:
#if (defined(BOARD_LOSC_EXTERNAL) && BOARD_LOSC_EXTERNAL)
#else
		/* Disable inter 32K calibration before enter standby, hibernation,
		 * poweroff, etc. to avoid wrong RTC time counting.
		 */
#if (CONFIG_CHIP_ARCH_VER == 2)
		if (state == PM_MODE_STANDBY) {
			uint16_t  u16RrefTime;
			u16RrefTime = 10 * HAL_GetHFClock() / HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS) + 0x200;
			HAL_PRCM_SetDigSWRefTime(u16RrefTime);
		}
		HAL_PRCM_DisableInter32KCalib();
#elif (CONFIG_CHIP_ARCH_VER == 3)
		{
			RCOCALI_ConfigParam configParm = {
				.mode = PRCM_RCOSC_WK_MODE_SEL_SCALE,
				.phase2_times = PRCM_RCOSC_SCALE_PHASE2_WK_TIMES_4,
				.phase3_times = PRCM_RCOSC_SCALE_PHASE3_WK_TIMES_128,
				.phase1_num = 1,
				.phase2_num = 1,
				.wup_time = 7680,
			};
			HAL_RcoscCali_Stop();
			HAL_RcoscCali_Config(&configParm);
			HAL_RcoscCali_Start();
		}
#endif
#endif
		break;
	default:
		break;
	}

	return 0;
}

static int lfclk_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
#if (defined(BOARD_LOSC_EXTERNAL) && BOARD_LOSC_EXTERNAL)
#else
#if (CONFIG_CHIP_ARCH_VER == 2)
		HAL_PRCM_EnableInter32KCalib();
#endif
#endif
		break;
	case PM_MODE_HIBERNATION:
		break;
	default:
		break;
	}

	return 0;
}

static const struct soc_device_driver lfclk_drv = {
	.name = "lfclk",
	.suspend_noirq = lfclk_suspend,
	.resume_noirq = lfclk_resume,
};

static struct soc_device lfclk_dev = {
	.name = "lfclk",
	.driver = &lfclk_drv,
};
#endif

static void lfclk_init(void)
{
#if (defined(BOARD_LOSC_EXTERNAL) && BOARD_LOSC_EXTERNAL)
	HAL_PRCM_SetLFCLKBaseSource(PRCM_LFCLK_BASE_SRC_EXT32K);
	HAL_PRCM_DisableInter32KCalib();

	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_SYS, PRCM_LFCLK_SRC_LOSC);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_WKTIMER, PRCM_LFCLK_SRC_LOSC);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_WLAN, PRCM_LFCLK_SRC_LOSC);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_BLE, PRCM_LFCLK_SRC_LOSC);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_RTC, PRCM_LFCLK_SRC_LOSC);

	HAL_RcoscCali_DeInit();
#else
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PRCM_SetLFCLKBaseSource(PRCM_LFCLK_BASE_SRC_INTER32K);
	HAL_PRCM_EnableInter32KCalib();
#elif (CONFIG_CHIP_ARCH_VER == 3)
	HAL_PRCM_SetLFCLKBaseSource(PRCM_LFCLK_BASE_SRC_INTER32K);  /* should set again for release LOSC pin */
	HAL_PRCM_DisableInter32KCalib();

	static RCOCALI_InitParam initParam;
	initParam.cnt_n = 8192;
	initParam.out_clk = 32000;
	HAL_RcoscCali_Init(&initParam);

	RCOCALI_ConfigParam configParm = {
		.mode = PRCM_RCOSC_WK_MODE_SEL_SCALE,
		.phase2_times = PRCM_RCOSC_SCALE_PHASE2_WK_TIMES_4,
		.phase3_times = PRCM_RCOSC_SCALE_PHASE3_WK_TIMES_24,
		.phase1_num = 1,
		.phase2_num = 1,
		.wup_time = 640,
	};
	HAL_RcoscCali_Config(&configParm);
	HAL_PRCM_SetRcoscCalStartSrc(PRCM_RCOSC_CAL_START_APPANDWLANSLEEP);
	HAL_RcoscCali_Start();

	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_SYS, PRCM_LFCLK_SRC_RCCAL);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_WKTIMER, PRCM_LFCLK_SRC_RCCAL);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_WLAN, PRCM_LFCLK_SRC_RCCAL);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_BLE, PRCM_LFCLK_SRC_RCCAL);
	HAL_PRCM_SetLFCLKSource(PRCM_LFCLK_MODULE_RTC, PRCM_LFCLK_SRC_RCCAL);
#endif /* CONFIG_CHIP_ARCH_VER */
#endif
#ifdef CONFIG_PM
	pm_register_ops(&lfclk_dev);
#endif
}
#endif

void board_chip_clock_init(void)
{
#ifdef CONFIG_HOSC_TYPE_24M
	HAL_PRCM_SetHOSCType(PRCM_HOSC_TYPE_24M);
	HAL_PRCM_SetSysPLL(PRCM_SYS_PLL_PARAM_HOSC24M);
#elif (defined CONFIG_HOSC_TYPE_26M)
	HAL_PRCM_SetHOSCType(PRCM_HOSC_TYPE_26M);
	HAL_PRCM_SetSysPLL(PRCM_SYS_PLL_PARAM_HOSC26M);
#elif (defined CONFIG_HOSC_TYPE_32M)
	HAL_PRCM_SetHOSCType(PRCM_HOSC_TYPE_32M);
	HAL_PRCM_SetSysPLL(PRCM_SYS_PLL_PARAM_HOSC32M);
#elif (defined CONFIG_HOSC_TYPE_40M)
	HAL_PRCM_SetHOSCType(PRCM_HOSC_TYPE_40M);
	HAL_PRCM_SetSysPLL(PRCM_SYS_PLL_PARAM_HOSC40M);
#elif (defined CONFIG_HOSC_TYPE_52M)
	HAL_PRCM_SetHOSCType(PRCM_HOSC_TYPE_52M);
	HAL_PRCM_SetSysPLL(PRCM_SYS_PLL_PARAM_HOSC52M);
#else
	#error "Invalid HOSC value!"
#endif

#ifndef CONFIG_BOOTLOADER
	lfclk_init();
#endif

	HAL_PRCM_SetCPUAClk(BOARD_CPU_CLK_SRC, BOARD_CPU_CLK_FACTOR);
	HAL_PRCM_SetDevClock(BOARD_DEV_CLK_FACTOR);
	HAL_CCM_BusSetClock(BOARD_AHB2_CLK_DIV, BOARD_APB_CLK_SRC, BOARD_APB_CLK_DIV);
#ifdef CONFIG_PSRAM
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PRCM_SetDev2Clock(BOARD_DEV2_CLK_FACTOR);
	HAL_PRCM_EnableDev2Clock();
#endif
#endif
#if (CONFIG_CHIP_ARCH_VER > 1)
	HAL_CCM_BusSetAPBSClock(BOARD_APBS_CLK_SRC, BOARD_APBS_CLK_FACTOR);
#endif
}

static GPIO_PinMuxParam g_pinmux_flashc_sip[] = {
	{ GPIO_PORT_B, GPIO_PIN_8,   { GPIOB_P8_F2_FLASH_WP,    GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_9,   { GPIOB_P9_F2_FLASH_HOLD,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_10,  { GPIOB_P10_F2_FLASH_MOSI, GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_11,  { GPIOB_P11_F2_FLASH_MISO, GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_12,  { GPIOB_P12_F2_FLASH_CS1,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_13,  { GPIOB_P13_F2_FLASH_CLK,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
};

#ifdef CONFIG_PSRAM
#if (CONFIG_CHIP_ARCH_VER == 2)
static GPIO_PinMuxParam g_pinmux_psram_sip[] = {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	{ GPIO_PORT_C, GPIO_PIN_4,  { GPIOC_P4_F4_PSRAM_SIO3, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_5,  { GPIOC_P5_F4_PSRAM_CLK,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_7,  { GPIOC_P7_F4_PSRAM_CE,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_9,  { GPIOC_P9_F4_PSRAM_SIO1, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_10, { GPIOC_P10_F4_PSRAM_SIO2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_11, { GPIOC_P11_F4_PSRAM_SIO0, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	{ GPIO_PORT_C, GPIO_PIN_0,  { GPIOC_P0_F2_PSRAM_DM,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_1,  { GPIOC_P1_F2_PSRAM_DQ0,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_2,  { GPIOC_P2_F2_PSRAM_DQ1,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_3,  { GPIOC_P3_F2_PSRAM_DQ2,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_4,  { GPIOC_P4_F2_PSRAM_DQ3,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_5,  { GPIOC_P5_F2_PSRAM_CE,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_6,  { GPIOC_P6_F2_PSRAM_CLK_N, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_7,  { GPIOC_P7_F2_PSRAM_CLK,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_8,  { GPIOC_P8_F2_PSRAM_DQ4,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_9,  { GPIOC_P9_F2_PSRAM_DQ5,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_10, { GPIOC_P10_F2_PSRAM_DQ6, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_11, { GPIOC_P11_F2_PSRAM_DQ7, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_12, { GPIOC_P12_F2_PSRAM_DQS, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	{ GPIO_PORT_C, GPIO_PIN_0,  { GPIOC_P0_F3_PSRAM_DQS,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_1,  { GPIOC_P1_F3_PSRAM_DQ7,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_2,  { GPIOC_P2_F3_PSRAM_DQ6,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_3,  { GPIOC_P3_F3_PSRAM_DQ5,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_4,  { GPIOC_P4_F3_PSRAM_DQ4,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_5,  { GPIOC_P5_F3_PSRAM_CLK,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_7,  { GPIOC_P7_F3_PSRAM_CE,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_8,  { GPIOC_P8_F3_PSRAM_DQ3,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_9,  { GPIOC_P9_F3_PSRAM_DQ2,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_10, { GPIOC_P10_F3_PSRAM_DQ1, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_C, GPIO_PIN_11, { GPIOC_P11_F3_PSRAM_DQ0, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
#endif
};
#elif (CONFIG_CHIP_ARCH_VER == 3)
static const GPIO_PinMuxParam g_pinmux_psram_sip[] = {
	{ GPIO_PORT_B, GPIO_PIN_8,  { GPIOB_P8_F2_FLASH_WP,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP  } },
	{ GPIO_PORT_B, GPIO_PIN_9,  { GPIOB_P9_F2_FLASH_HOLD,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP  } },
	{ GPIO_PORT_B, GPIO_PIN_10, { GPIOB_P10_F2_FLASH_MOSI, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_11, { GPIOB_P11_F2_FLASH_MISO, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_12, { GPIOB_P12_F2_FLASH_CS1,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_13, { GPIOB_P13_F2_FLASH_CLK,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

#endif    /*CONFIG_CONFIG_CHIP_ARCH_VER */
#endif    /* CONFIG_PSRAM */

HAL_Status board_get_flashc_sip_pinmux_cfg(const GPIO_PinMuxParam **param,
                                           uint32_t *count)
{
	if (HAL_PRCM_IsFlashSip()) {
		*param = g_pinmux_flashc_sip;
		*count = HAL_ARRAY_SIZE(g_pinmux_flashc_sip);
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

#ifdef CONFIG_PSRAM
HAL_Status board_get_psram_sip_pinmux_cfg(const GPIO_PinMuxParam **param,
                                          uint32_t *count)
{
	if (0) {
	//if (HAL_PRCM_IsPsramSip()) {
		*param = g_pinmux_psram_sip;
		*count = HAL_ARRAY_SIZE(g_pinmux_psram_sip);
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}
#endif
