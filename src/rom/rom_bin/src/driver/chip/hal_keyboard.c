/**
  * @file  hal_keyboard.c
  * @author  XRADIO IOT Team
  */

/*
 * Copyright (C) 2019 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
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
/* Includes ------------------------------------------------------------------*/
#if (CONFIG_CHIP_ARCH_VER == 3)

#include "hal_base.h"
#include "rom/driver/chip/hal_keyboard.h"
#include "rom/driver/chip/hal_gpio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DBG_KEYB 1

#if (DBG_KEYB == 1)
#define KEYB_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)
#define KEYB_DBG(fmt, arg...) KEYB_LOG(0, "[KEYB DBG] "fmt, ##arg)
#define KEYB_INF(fmt, arg...) KEYB_LOG(DBG_KEYB, "[KEYB INF] "fmt, ##arg)
#define KEYB_WRN(fmt, arg...) KEYB_LOG(DBG_KEYB, "[KEYB WRN] "fmt, ##arg)
#define KEYB_ERR(fmt, arg...) KEYB_LOG(DBG_KEYB, "[KEYB ERR] "fmt, ##arg)
#else
#define KEYB_INF(...)
#define KEYB_WRN(...)
#define KEYB_ERR(...)
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
* @brief KeyBoard Interface enable/disable
* @param en: 1/0
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_InterfaceCmd(uint8_t en)
{
	if (en)
		HAL_SET_BIT(KEYBOARD->COL_CTL, KEYBOARD_INTERFACE_EN_BIT);
	else
		HAL_CLR_BIT(KEYBOARD->COL_CTL, KEYBOARD_INTERFACE_EN_BIT);
}

/*
* @brief
* @param cols: each bit dicate a column(max 8 column, cols max value is 0xff)
*        rows: each bit dicate a row(max 16 row, rows max value is 0xffff)
*        set bit corresponding is mask
*    Example: cols:0x67 (seven fourth third column is selected);
*             rows:0xfff3 (third second row is selected);
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_SetColsAndRows(uint32_t cols, uint32_t rows)
{
	HAL_ASSERT_PARAM(cols <= 0xff && rows <= 0xffff);

	HAL_MODIFY_REG(KEYBOARD->COL_CTL, KEYBOARD_COL_OUTPUT_MASK, cols << KEYBOARD_COL_OUTPUT_SHIFT);
	HAL_MODIFY_REG(KEYBOARD->ROW_CTL, KEYBOARD_ROW_INPUT_MASK, rows << KEYBOARD_ROW_INPUT_SHIFT);
}

/*
* @brief
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_SetSampleCfg(uint8_t sp_continue, uint32_t timecycle)
{
	if (sp_continue)
		HAL_SET_BIT(KEYBOARD->SAMPLE_CTL, KEYBOARD_SMPL_CTU_EN_BIT);
	else
		HAL_CLR_BIT(KEYBOARD->SAMPLE_CTL, KEYBOARD_SMPL_CTU_EN_BIT);

	HAL_MODIFY_REG(KEYBOARD->SAMPLE_CTL, KEYBOARD_DUR_TIMECYCLE_MASK, KEYBOARD_DUR_TIMECYCLE(timecycle));
	HAL_CLR_BIT(KEYBOARD->SAMPLE_CTL, KEYBOARD_SMPL_OPTM_EN_BIT);
}

/*
* @brief
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_SetTimeCfg(uint16_t scan, uint16_t debounce)
{
	HAL_ASSERT_PARAM(scan <= 0xffff && debounce <= 0xffff);

	HAL_MODIFY_REG(KEYBOARD->TIMING, KEYBOARD_SCAN_CYCLE_MASK | KEYBOARD_DBC_CYCLE_MASK,
	               scan << KEYBOARD_SCAN_CYCLE_SHIFT | debounce << KEYBOARD_DBC_CYCLE_SHIFT);
}

/*
* @brief
* @KEYBOARD_IT:
*   KEYBOARD_IT_FEDGE_IRQ
*   KEYBOARD_IT_REDGE_IRQ
*   KEYBOARD_IT_DECREASE_IRQ
*@NewState: ENABLE&DISABLE
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_ITConfig(uint32_t IT, uint8_t en)
{
	HAL_ASSERT_PARAM((IT & 0x07) != (uint32_t)0);

	if (en)
		HAL_SET_BIT(KEYBOARD->INT_CFG, IT);
	else
		HAL_CLR_BIT(KEYBOARD->INT_CFG, IT);
}

/*
* @brief
* @IT_FLAG:
*   KEYBOARD_IT_FEDGE_PENDING
*   KEYBOARD_IT_REDGE_PENDING
*/
__STATIC_INLINE uint8_t KeyBoard_GetITStatus(uint32_t IT_FLAG)
{
	HAL_ASSERT_PARAM((IT_FLAG & 0x07) != (uint32_t)0);

	uint8_t bitstatus = 0;

	if (HAL_GET_BIT(KEYBOARD->INT_STA, IT_FLAG) != (uint32_t)0) {
		bitstatus = 1;
	}
	return bitstatus;
}

/*
* @brief
* @IT_FLAG:
*   KEYBOARD_IT_FEDGE_PENDING
*   KEYBOARD_IT_REDGE_PENDING
*/
__nonxip_text
__STATIC_INLINE void KeyBoard_ClearITPending(uint32_t IT_FLAG)
{
	HAL_ASSERT_PARAM((IT_FLAG & 0x07) != (uint32_t)0);

	HAL_SET_BIT(KEYBOARD->INT_STA, IT_FLAG);
}

/*
* @brief
*/
__STATIC_INLINE uint32_t KeyBoard_GetData(uint8_t cols)
{
	HAL_ASSERT_PARAM(cols < 8);

	return KEYBOARD->IN_DATA[cols];
}

/* clock config: CLK_SRC/N/M
 * SYS_CRYSTAL: 24M/32K
 * 24M: 24M/(2^0)/(1+0) = 24M
 *
 */
__nonxip_text
int32_t HAL_KeyBoard_ConfigCCMU(uint32_t clk)
{
	CCM_AHBPeriphClkSrc src;
	uint32_t mclk;
	uint32_t div;
	CCM_PeriphClkDivN div_n = 0;
	CCM_PeriphClkDivM div_m = 0;

	if (clk > HAL_GetHFClock()) {
		mclk = HAL_GetHFClock();
		src = CCM_APB_PERIPH_CLK_SRC_HFCLK;
	} else {
		mclk = HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS);
		src = CCM_APB_PERIPH_CLK_SRC_LFCLK;
	}

	div = (mclk + clk - 1) / clk;
	div = (div == 0) ? 1 : div;

	if (div > (16 * 8))
		return 0;

	if (div > 64) {
		div_n = CCM_PERIPH_CLK_DIV_N_8;
		div_m = (CCM_PeriphClkDivM)((div >> 3) - 1);
	} else if (div > 32) {
		div_n = CCM_PERIPH_CLK_DIV_N_4;
		div_m = (CCM_PeriphClkDivM)((div >> 2) - 1);
	} else if (div > 16) {
		div_n = CCM_PERIPH_CLK_DIV_N_2;
		div_m = (CCM_PeriphClkDivM)((div >> 1) - 1);
	} else {
		div_n = CCM_PERIPH_CLK_DIV_N_1;
		div_m = (CCM_PeriphClkDivM)((div >> 0) - 1);
	}

	HAL_CCM_KEYBOARD_DisableMClock();
	HAL_CCM_KEYBOARD_SetMClock(src, div_n, div_m);
	HAL_CCM_KEYBOARD_EnableMClock();
	KEYB_INF("KEYBOARD CTRL MCLK:%u MHz clock=%u MHz,src:%x, n:%d, m:%d\n", mclk / 1000000,
	         mclk / (1 << div_n) / (div_m + 1) / 1000000, (int)src, (int)div_n, (int)div_m);

	return 1;
}

#ifdef CONFIG_PM
static keyboard_InitParam hal_keyboard_config;
static uint8_t hal_keyboard_suspending = 0;

__nonxip_text
static int keyboard_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	hal_keyboard_suspending = 1;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		HAL_KeyBoard_DeInit();
		break;
	default:
		break;
	}

	return 0;
}

__nonxip_text
static int keyboard_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		HAL_KeyBoard_Init(&hal_keyboard_config);
		break;
	default:
		break;
	}

	hal_keyboard_suspending = 0;

	return 0;
}

static struct soc_device_driver keyboard_drv = {
	.name = "keyboard",
	.suspend_noirq = keyboard_suspend,
	.resume_noirq = keyboard_resume,
};

static struct soc_device keyboard_dev = {
	.name = "keyboard",
	.driver = &keyboard_drv,
};

#define KEYBOARD_DEV 	(&keyboard_dev)
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

typedef struct {
	KeyBoard_IRQCallback    callback;
	uint32_t                buf[KEY_SCAN_BUFFER_MAX];
} KeyBoard_Private;

static KeyBoard_Private *gPrivate;

/**
 * @brief KEYBOARD IRQHandler function
 * @retval None
 */
void KeyBoard_IRQHandler(void)
{
	if (KeyBoard_GetITStatus(KEYBOARD_IT_FEDGE_PENDING)) {
		KeyBoard_ClearITPending(KEYBOARD_IT_FEDGE_PENDING);
		printf("press!\n");
		for (uint8_t col = 0; col < KEY_SCAN_BUFFER_MAX; col++) {
			gPrivate->buf[col] = KeyBoard_GetData(col);
		}
	}
	if (KeyBoard_GetITStatus(KEYBOARD_IT_REDGE_PENDING)) {
		KeyBoard_ClearITPending(KEYBOARD_IT_REDGE_PENDING);
		printf("loose!\n");
		HAL_Memset((uint8_t *)gPrivate->buf, 0xff, sizeof(gPrivate->buf));
	}

	if (gPrivate->callback)
		gPrivate->callback(gPrivate->buf, KEY_SCAN_BUFFER_MAX);
}

/**
 * @brief Initialize the Keypad according to the specified parameters
 * @param param Pointer to keypad_initParam structure
 * @retval HAL_Status, HAL_OK on success
 */
__nonxip_text
HAL_Status HAL_KeyBoard_Init(keyboard_InitParam *cfg)
{
	if (!cfg)
		return HAL_INVALID;

	/* enable keypad clock and release */
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_KEYSCAN);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_KEYSCAN);
	HAL_KeyBoard_ConfigCCMU(cfg->freq);
	/* config pinmux */
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_KEYBOARD, 0), 0);

	KeyBoard_SetColsAndRows(cfg->cols, cfg->rows);
	KeyBoard_SetSampleCfg(1, cfg->timecycle);
	KeyBoard_SetTimeCfg(cfg->scan, cfg->debounce);
	KeyBoard_InterfaceCmd(1);

#ifdef CONFIG_PM
	if (!hal_keyboard_suspending) {
		memcpy(&hal_keyboard_config, cfg, sizeof(keyboard_InitParam));
		pm_register_ops(KEYBOARD_DEV);
	}
#endif

	KeyBoard_ClearITPending(KEYBOARD_IT_FEDGE_PENDING | KEYBOARD_IT_REDGE_PENDING);
	KeyBoard_ITConfig(KEYBOARD_IT_FEDGE_IRQ | KEYBOARD_IT_REDGE_IRQ, 1);
	HAL_NVIC_ConfigExtIRQ(KEYSCAN_IRQn, KeyBoard_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

	return HAL_OK;
}

/**
 * @brief DeInitialize the specified Keypad
 * @retval HAL_Status, HAL_OK on success
 */
__nonxip_text
HAL_Status HAL_KeyBoard_DeInit(void)
{
#ifdef CONFIG_PM
	if (!hal_keyboard_suspending) {
		pm_unregister_ops(KEYBOARD_DEV);
	}
#endif
	/* enable keypad clock and release */
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_KEYSCAN);
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_KEYSCAN);
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_KEYBOARD, 0), 0);
	KeyBoard_ClearITPending(KEYBOARD_IT_FEDGE_PENDING | KEYBOARD_IT_REDGE_PENDING);
	HAL_NVIC_DisableIRQ(KEYSCAN_IRQn);

	KeyBoard_InterfaceCmd(0);

	return HAL_OK;
}

KeyBoard_Private *HAL_KeyBoard_Create(void)
{
	gPrivate = HAL_Malloc(sizeof(KeyBoard_Private));
	if (gPrivate == NULL) {
		KEYB_ERR("%s no mem!\n", __func__);
	} else {
		HAL_Memset(gPrivate, 0, sizeof(KeyBoard_Private));
	}
	return gPrivate;
}

HAL_Status HAL_KeyBoard_Destory(void)
{
	if (gPrivate == NULL) {
		KEYB_ERR("gPrivate %p\n", gPrivate);
	} else {
		HAL_Free(gPrivate);
	}
	return HAL_OK;
}

HAL_Status HAL_KeyBoard_Open(keyboard_InitParam *cfg, KeyBoard_IRQCallback cb)
{
	KeyBoard_Private *priv;

	HAL_ASSERT_PARAM(cfg != NULL);

	priv = HAL_KeyBoard_Create();
	if (priv == NULL) {
		KEYB_ERR("no mem\n");
		return HAL_ERROR;
	}
	priv->callback = cb;

	HAL_KeyBoard_Init(cfg);

	return HAL_OK;
}

HAL_Status HAL_KeyBoard_Close(void)
{
	HAL_KeyBoard_DeInit();
	KeyBoard_InterfaceCmd(0);
	HAL_KeyBoard_Destory();

	return HAL_OK;
}

#endif /* #if (CONFIG_CHIP_VRCH_VER == 3) */
