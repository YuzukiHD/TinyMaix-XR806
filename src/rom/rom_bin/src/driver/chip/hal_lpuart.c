/**
  * @file  hal_lpuart.c
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

#include "rom/driver/chip/hal_lpuart.h"
#include "rom/pm/pm.h"

#include "hal_base.h"

typedef struct {
	LPUART_T                *lpuart;
	HAL_Semaphore           rxSem;
	int32_t                 rxBufSize;
	LPUART_RxCmpCallback    rxCmpCallback;
	void                    *arg;
	uint8_t                 *rxBuf;
	uint8_t                 bind_uart;
#ifdef CONFIG_PM
	uint8_t                 bypassPmMode;
	uint8_t                 txDelay;
	LPUART_InitParam        param;
	struct soc_device       dev;
#endif
} LPUART_Private;

static LPUART_Private *gLpuartPrivate[LPUART_NUM];

__STATIC_INLINE LPUART_Private *LPUART_GetLpuartPriv(LPUART_ID lpuartID)
{
	if (lpuartID < LPUART_NUM) {
		return gLpuartPrivate[lpuartID];
	} else {
		return NULL;
	}
}

__STATIC_INLINE void LPUART_SetLpuartPriv(LPUART_ID lpuartID, LPUART_Private *priv)
{
	gLpuartPrivate[lpuartID] = priv;
}

__STATIC_INLINE LPUART_T *LPUART_GetInstance(LPUART_Private *priv)
{
	return priv->lpuart;
}

__STATIC_INLINE void LPUART_EnableIRQ(LPUART_T *lpuart, uint32_t mask)
{
	HAL_SET_BIT(lpuart->IRQ_EN, mask);
}

__STATIC_INLINE void LPUART_DisableIRQ(LPUART_T *lpuart, uint32_t mask)
{
	HAL_CLR_BIT(lpuart->IRQ_EN, mask);
}

__nonxip_text
__STATIC_INLINE void LPUART_DisableAllIRQ(LPUART_T *lpuart)
{
	lpuart->IRQ_EN = 0U;
}

__nonxip_text
__STATIC_INLINE uint32_t LPUART_GetIRQFlag(LPUART_T *lpuart, uint32_t mask)
{
	return HAL_GET_BIT(lpuart->IRQ_S, mask);
}

__nonxip_text
__STATIC_INLINE void LPUART_ClrIRQFlag(LPUART_T *lpuart, uint32_t mask)
{
	HAL_SET_BIT(lpuart->IRQ_CLR, mask);
}

__nonxip_text
__STATIC_INLINE uint8_t LPUART_GetRxCmpNum(LPUART_T *lpuart)
{
	uint8_t cmp_num = 0;

	cmp_num = HAL_GET_BIT_VAL(lpuart->RX_CMP1, LPUART_RX_CMP_LEN_SHIFT, LPUART_RX_CMP_LEN_MASK);

	cmp_num = cmp_num == 0 ? 1 : cmp_num;

	return cmp_num;
}

/**
 * @brief Get one byte received data of the specified LPUART
 * @param[in] lpuart LPUART hardware instance
 * @return One byte received data
 *
 * @note Before calling this function, make sure the specified LPUART has
 *       receive data by calling HAL_LPUART_IsRxReady()
 */
__nonxip_text
__STATIC_INLINE uint32_t LPUART_GetRxData(LPUART_T *lpuart)
{
	return HAL_GET_BIT(lpuart->RX_DATA, LPUART_RX_DATA_MASK);
}

__nonxip_text
__STATIC_INLINE void LPUART_SoftReset(LPUART_T *lpuart)
{
	HAL_SET_BIT(lpuart->CTRL, LPUART_CFG_SOFT_RST_BIT);
	while (HAL_GET_BIT(lpuart->CTRL, LPUART_CFG_SOFT_RST_BIT))
		;
}

/*
 * @brief Algorithm for finding remainder
 */
__nonxip_text
void LPUART_SetBaudrate(LPUART_T *lpuart, uint32_t baudRate)
{
	uint32_t mclock, divisor, quotient, reaminder;
	uint32_t x, y, z;

	mclock = HAL_PRCM_GetLpuartMClock() << 1;

	x = mclock;
	y = baudRate;
	while (y) {
		z = x % y;
		x = y;
		y = z;
	}

	divisor = (baudRate / x) & LPUART_DIV_MASK;
	quotient = (mclock / baudRate) & LPUART_QUOT_MASK;
	reaminder = ((mclock % baudRate) / x) & LPUART_DIV_MASK;

	HAL_MODIFY_REG(lpuart->BAUD_CFG, \
	               (LPUART_DIV_MASK << LPUART_CFG_DIVISOR_POS) | \
	               (LPUART_DIV_MASK << LPUART_CFG_REMAINDER_POS) | \
	               (LPUART_QUOT_MASK << LPUART_CFG_QUOTIENT_POS), \
	               (divisor << LPUART_CFG_DIVISOR_POS) | \
	               (reaminder << LPUART_CFG_REMAINDER_POS) | \
	               (quotient << LPUART_CFG_QUOTIENT_POS));
}

/*
 * @brief
 */
__nonxip_text
static void LPUART_IRQHandler(LPUART_T *lpuart, LPUART_Private *priv)
{
	uint32_t int_flag = LPUART_GetIRQFlag(lpuart, LPUART_IRQ_FLAG_MASK);
	uint32_t irq_flag[2];
	//uint8_t  cmpNum;

	int_flag &= HAL_GET_BIT(lpuart->IRQ_EN, LPUART_IRQ_FLAG_MASK);

	HAL_IT_LOG(HAL_DBG_ON, "int_flag = 0x%x\n", int_flag);

	irq_flag[0] = int_flag & LPUART_RX_DATA_FLAG_BIT;
	irq_flag[1] = int_flag & LPUART_RX_DATA_CMP_FLAG_BIT;

	if (irq_flag[0]) {
		*priv->rxBuf = LPUART_GetRxData(lpuart);
		++priv->rxBuf;
		--priv->rxBufSize;

		if (!priv->rxBufSize) {
			HAL_SemaphoreRelease(&priv->rxSem); /* end receiving */
		}

		LPUART_ClrIRQFlag(lpuart, irq_flag[0]);
		while (LPUART_GetIRQFlag(lpuart, irq_flag[0]));
	}

	if (irq_flag[1]) {
		/*cmpNum = LPUART_GetRxCmpNum(lpuart);
		while (cmpNum > 0) {
			LPUART_GetRxData(lpuart);
			--cmpNum;
		}*/

		if (priv->rxCmpCallback) {
			priv->rxCmpCallback(NULL);
		}

		LPUART_ClrIRQFlag(lpuart, irq_flag[1]);
		while (LPUART_GetIRQFlag(lpuart, irq_flag[1]));
	}
}

__nonxip_text
void LPUART0_IRQHandler(void)
{
	LPUART_IRQHandler(LPUART0, LPUART_GetLpuartPriv(LPUART0_ID));
}

__nonxip_text
void LPUART1_IRQHandler(void)
{
	LPUART_IRQHandler(LPUART1, LPUART_GetLpuartPriv(LPUART1_ID));
}

__nonxip_text
static LPUART_T *LPUART_HwInit(LPUART_ID lpuartID, const LPUART_InitParam *param)
{
	LPUART_T *lpuart;
	PRCM_WAKEUP_SRC_BusClkBit prcmBusClkBit;
	PRCM_WAKEUP_SRC_RstBit prcmPeriphBit;
	IRQn_Type IRQn;
	NVIC_IRQHandler IRQHandler;

	switch (lpuartID) {
	case LPUART1_ID:
		lpuart = LPUART1;
		IRQn = LPUART1_IRQn;
		IRQHandler = LPUART1_IRQHandler;
		prcmPeriphBit = PRCM_WAKEUP_SRC_RST_BIT_LPUART1;
		prcmBusClkBit = PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART1;
		HAL_PRCM_EnableLpuart1Clk();
		HAL_PRCM_SelectLpuart1WakeupIOIn(param->input_uart);
		break;
	case LPUART0_ID:
	default:
		lpuart = LPUART0;
		IRQn = LPUART0_IRQn;
		IRQHandler = LPUART0_IRQHandler;
		prcmPeriphBit = PRCM_WAKEUP_SRC_RST_BIT_LPUART0;
		prcmBusClkBit = PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART0;
		HAL_PRCM_EnableLpuart0Clk();
		HAL_PRCM_SelectLpuart0WakeupIOIn(param->input_uart);
		break;
	}

	/* config pinmux */
	if (!HAL_UART_GetPinmuxStatus(param->input_uart)) {
		HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_UART, param->input_uart), 0);
	}
	HAL_UART_SetPinmuxStatus(param->input_uart, UART_Pin_USER_LPUART);

	/* enable lpuart clock and release reset */
	HAL_PRCM_WAKEUP_SRC_EnableClkGating(prcmBusClkBit);
	HAL_PRCM_ReleaseWakeupSrcReset(prcmPeriphBit);

	LPUART_SoftReset(lpuart);
	LPUART_DisableAllIRQ(lpuart); /* disable all IRQ */

	/* set baud rate, parity, msb, data bits */
	LPUART_SetBaudrate(lpuart, param->baudRate);

	HAL_MODIFY_REG(lpuart->RX_CFG, \
	               LPUART_CFG_MSB_FIRST_MASK | LPUART_DATA_WIDTH_MASK | \
	               LPUART_PARITY_SEL_MASK | LPUART_RX_EN, \
	               param->msbFirst | param->parity | param->dataWidth | LPUART_RX_EN);

	HAL_NVIC_ConfigExtIRQ(IRQn, IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

	return lpuart;
}

__nonxip_text
static void LPUART_HwDeInit(LPUART_Private *priv, LPUART_ID lpuartID)
{
	LPUART_T *lpuart;
	PRCM_WAKEUP_SRC_BusClkBit prcmBusClkBit;
	PRCM_WAKEUP_SRC_RstBit prcmPeriphBit;
	IRQn_Type IRQn;

	switch (lpuartID) {
	case LPUART1_ID:
		lpuart = LPUART1;
		IRQn = LPUART1_IRQn;
		prcmPeriphBit = PRCM_WAKEUP_SRC_RST_BIT_LPUART1;
		prcmBusClkBit = PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART1;
		HAL_PRCM_DisableLpuart1Clk();
		break;
	case LPUART0_ID:
	default:
		lpuart = LPUART0;
		IRQn = LPUART0_IRQn;
		prcmPeriphBit = PRCM_WAKEUP_SRC_RST_BIT_LPUART0;
		prcmBusClkBit = PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART0;
		HAL_PRCM_DisableLpuart0Clk();
		break;
	}

	HAL_NVIC_DisableIRQ(IRQn);

	LPUART_DisableAllIRQ(lpuart);
	HAL_CLR_BIT(lpuart->RX_CFG, LPUART_RX_EN);

	/* disable lpuart clock and force reset */
	HAL_PRCM_ForceWakeupSrcReset(prcmPeriphBit);
	HAL_PRCM_WAKEUP_SRC_DisableClkGating(prcmBusClkBit);

	/* De-config pinmux */
	HAL_UART_ClrPinmuxStatus(priv->bind_uart, UART_Pin_USER_LPUART);
	if (!HAL_UART_GetPinmuxStatus(priv->bind_uart)) {
		HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_UART, priv->bind_uart), 0);
	}
}

#ifdef CONFIG_PM
__nonxip_text
static int lpuart_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	LPUART_ID lpuartID = (LPUART_ID)dev->platform_data;
	LPUART_Private *priv = LPUART_GetLpuartPriv(lpuartID);

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->bypassPmMode & PM_SUPPORT_SLEEP) {
			break;
		}
	case PM_MODE_STANDBY: /* if disable lpuart pclk, it will switch to 32K pclk, rather than gate off. */
		if (priv->bypassPmMode & PM_SUPPORT_STANDBY) {
			HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_UART, priv->param.input_uart), 0);
			HAL_PRCM_WAKEUP_SRC_DisableClkGating(lpuartID == LPUART0_ID ? \
				PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART0 : PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART1);
			break;
		}
	case PM_MODE_HIBERNATION:
		LPUART_HwDeInit(priv, lpuartID);
		break;
	default:
		break;
	}
	HAL_DBG("%s ok, id %d\n", __func__, lpuartID);

	return 0;
}

__nonxip_text
static int lpuart_resume(struct soc_device *dev, enum suspend_state_t state)
{
	LPUART_ID lpuartID = (LPUART_ID)dev->platform_data;
	LPUART_Private *priv = LPUART_GetLpuartPriv(lpuartID);

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->bypassPmMode & PM_SUPPORT_SLEEP) {
			break;
		}
	case PM_MODE_STANDBY: /* if enable lpuart pclk, switch to the real pclk. */
		if (priv->bypassPmMode & PM_SUPPORT_STANDBY) {
			HAL_PRCM_WAKEUP_SRC_EnableClkGating(lpuartID == LPUART0_ID ? \
				PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART0 : PRCM_WAKEUP_SRC_BUS_CLK_BIT_LPUART1);
			break;
		}
	case PM_MODE_HIBERNATION:
		LPUART_HwInit(lpuartID, &priv->param);
		break;
	default:
		break;
	}
	HAL_DBG("%s ok, id %d\n", __func__, lpuartID);

	return 0;
}

const static struct soc_device_driver lpuart_drv = {
	.name = "lpuart",
	.suspend_noirq = lpuart_suspend,
	.resume_noirq = lpuart_resume,
};

static const char gLpuartName[LPUART_NUM][8] = {
	{ "lpuart0" },
	{ "lpuart1" }
};
#endif /* CONFIG_PM */

/**
 * @brief Initialize the LPUART according to the specified parameters
 * @param[in] param Pointer to LPUART_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_Init(LPUART_ID lpuartID, const LPUART_InitParam *param)
{
	LPUART_Private *priv;

	if (lpuartID > LPUART_NUM) {
		HAL_DBG("invalid lpuart id %d\n", lpuartID);
		return HAL_ERROR;
	}

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv != NULL) {
		HAL_DBG("lpuart %d is inited\n", lpuartID);
		return HAL_BUSY;
	}

	priv = HAL_Malloc(sizeof(LPUART_Private));
	if (priv == NULL) {
		HAL_ERR("no mem\n");
		return HAL_ERROR;
	}
	HAL_Memset(priv, 0, sizeof(LPUART_Private));
	LPUART_SetLpuartPriv(lpuartID, priv);

	HAL_SemaphoreInitBinary(&priv->rxSem);
	priv->lpuart = LPUART_HwInit(lpuartID, param);

	priv->rxBuf = NULL;
	priv->rxBufSize = 0;
	priv->rxCmpCallback = NULL;
	priv->arg = NULL;
	priv->bind_uart = param->input_uart;

#ifdef CONFIG_PM
	HAL_Memcpy(&priv->param, param, sizeof(LPUART_InitParam));
	priv->dev.name = gLpuartName[lpuartID];
	priv->dev.driver = &lpuart_drv;
	priv->dev.platform_data = (void *)lpuartID;
	pm_register_ops(&priv->dev);
#endif

	return HAL_OK;
}

/**
 * @brief DeInitialize the specified LPUART
 * @param[in] lpuartID ID of the specified LPUART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_DeInit(LPUART_ID lpuartID)
{
	LPUART_Private *priv;

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return HAL_OK;
	}

	if (priv->rxCmpCallback != NULL) {
		HAL_WRN("rx cb enabled\n");
		HAL_LPUART_DisableRxCmpCallback(lpuartID);
	}

#ifdef CONFIG_PM
	pm_unregister_ops(&priv->dev);
#endif

	LPUART_HwDeInit(priv, lpuartID);
	HAL_SemaphoreDeinit(&priv->rxSem);

	LPUART_SetLpuartPriv(lpuartID, NULL);
	HAL_Free(priv);

	return HAL_OK;
}

/**
 * @brief Enable receive compare callback function for the specified LPUART
 * @param[in] cb The LPUART receive ready callback function
 * @param[in] arg Argument of the LPUART receive ready callback function
 * @retval HAL_Status, HAL_OK on success
 *
 * @note To handle receive data externally, use this function to enable the
 *       receive ready callback function, then receive and process the data in
 *       the callback function.
 * @note If the receive ready callback function is enabled, all other receive
 *       series functions cannot be used to receive data.
 * @note This function is not thread safe. If using the LPUART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
HAL_Status HAL_LPUART_EnableRxCmpCallback(LPUART_ID lpuartID, LPUART_RxCmpCallback cb, void *arg)
{
	LPUART_Private *priv;

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return HAL_ERROR;
	}

	priv->rxCmpCallback = cb;
	priv->arg = arg;
	LPUART_EnableIRQ(LPUART_GetInstance(priv), LPUART_RX_DATA_CMP_EN_BIT);

	return HAL_OK;
}

/**
 * @brief Disable receive compare callback function for the specified LPUART
 * @param[in] lpuartID ID of the specified LPUART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_DisableRxCmpCallback(LPUART_ID lpuartID)
{
	LPUART_Private *priv;

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return HAL_ERROR;
	}

	LPUART_DisableIRQ(LPUART_GetInstance(priv), LPUART_RX_DATA_CMP_EN_BIT);
	priv->rxCmpCallback = NULL;
	priv->arg = NULL;

	return HAL_OK;
}

/**
 * @brief Receive an amount of data in interrupt mode
 * @param[in] lpuartID ID of the specified LPUART
 * @param[out] buf Pointer to the data buffer
 * @param[in] size The maximum number of bytes to be received.
 *                 The actual received bytes can be less than this.
 * @param[in] msec Timeout value in millisecond to receive data.
 *                 HAL_WAIT_FOREVER for no timeout.
 * @return Number of bytes received, -1 on error
 *
 * @note This function is not thread safe. If using the LPUART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
int32_t HAL_LPUART_Receive_IT(LPUART_ID lpuartID, uint8_t *buf, int32_t size, uint32_t msec)
{
	LPUART_T *lpuart;
	LPUART_Private *priv;

	if (buf == NULL || size <= 0) {
		return -1;
	}

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return -1;
	}

	priv->rxBuf = (uint8_t *)buf;
	priv->rxBufSize = size;

	lpuart = LPUART_GetInstance(priv);
	LPUART_EnableIRQ(lpuart, LPUART_RX_DATA_EN_BIT);
	HAL_SemaphoreWait(&priv->rxSem, msec);
	LPUART_DisableIRQ(lpuart, LPUART_RX_DATA_EN_BIT);

	priv->rxBuf = NULL;
	size -= priv->rxBufSize;
	priv->rxBufSize = 0;

	return size;
}

/**
 * @brief setup rx data compare function
 * @param[in] lpuartID ID of the specified LPUART
 * @param[in] cmp_len the length of compare data
 * @param[in] cmp_data buffer point to compare data
 * @retval HAL_Status, HAL_OK on success
 *
 * @note
 */
HAL_Status HAL_LPUART_EnableRxCmp(LPUART_ID lpuartID, uint8_t cmp_len, uint8_t *cmp_data)
{
	uint32_t i, reg_value1 = 0, reg_value2 = 0;
	uint8_t pos[5] = {LPUART_RX_CMP_DATA0_POS, LPUART_RX_CMP_DATA1_POS,
	                  LPUART_RX_CMP_DATA2_POS, LPUART_RX_CMP_DATA3_POS, LPUART_RX_CMP_DATA4_POS};
	LPUART_T *lpuart;
	LPUART_Private *priv;

	if ((cmp_data == NULL) || (cmp_len == 0) || (cmp_len > LPUART_RX_CMP_DATA_NUM_MAX)) {
		return HAL_ERROR;
	}

	for (i = 0; i < cmp_len; i++) {
		if (i >= 3) {
			HAL_SET_BIT(reg_value2, cmp_data[i] << pos[i]);
		} else {
			HAL_SET_BIT(reg_value1, cmp_data[i] << pos[i]);
		}
	}

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return HAL_ERROR;
	}
	lpuart = LPUART_GetInstance(priv);

	lpuart->RX_CMP1 = 0;
	lpuart->RX_CMP1 = reg_value1 | (cmp_len << LPUART_RX_CMP_LEN_SHIFT);
	lpuart->RX_CMP2 = 0;
	lpuart->RX_CMP2 = reg_value2;

	LPUART_EnableIRQ(lpuart, LPUART_RX_DATA_CMP_EN_BIT);

	return HAL_OK;
}

#ifdef CONFIG_PM
/**
 * @brief Set PM mode to be bypassed
 * @param[in] lpuartID ID of the specified LPUART
 * @param[in] mode Bit mask of PM mode to be bypassed
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_SetBypassPmMode(LPUART_ID lpuartID, uint8_t mode)
{
	LPUART_Private *priv;

	priv = LPUART_GetLpuartPriv(lpuartID);
	if (priv == NULL) {
		HAL_DBG("lpuart %d not inited\n", lpuartID);
		return HAL_ERROR;
	}
	priv->bypassPmMode = mode;
	return HAL_OK;
}
#endif
