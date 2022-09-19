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
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_adc.h"
#include "pm/pm.h"
#include "hal_base.h"

#ifdef CONFIG_ROM

typedef enum {
	ADC_STATE_INVALID   = 0,
	ADC_STATE_INIT      = 1, /* Initializing        */
	ADC_STATE_DEINIT    = 2, /* Deinitializing    */
	ADC_STATE_READY     = 3,
	ADC_STATE_BUSY      = 4
} ADC_State;

#ifdef CONFIG_PM
struct adc_chan_config {
	uint8_t             is_config;
	uint8_t             select;
	uint8_t             irqmode;
	uint8_t             workmode;
	uint32_t            lowValue;
	uint32_t            highValue;
};
#endif

typedef struct {
	uint16_t            chanPinMux;
	ADC_State           state;
	ADC_WorkMode        mode;
	uint32_t            lowPending;
	uint32_t            highPending;
	uint32_t            dataPending;

	ADC_IRQCallback     IRQCallback[ADC_CHANNEL_NUM];
	void                *arg[ADC_CHANNEL_NUM];
#ifdef CONFIG_PM
	uint8_t                 suspend_bypass;
	uint8_t                 suspend;
	struct soc_device       dev;
	ADC_InitParam           param;
	struct adc_chan_config  chan_config[ADC_CHANNEL_NUM];
#endif
} ADC_Private;

extern ADC_Private *_adc_priv;

#define ADC_ASSERT_CHANNEL(chan)    HAL_ASSERT_PARAM((chan) < ADC_CHANNEL_NUM)

extern HAL_Status __HAL_ADC_Conv_Polling(ADC_Channel chan, uint32_t *data, uint32_t msec);
extern HAL_Status __HAL_ADC_ConfigChannel(ADC_Channel chan, ADC_Select select, ADC_IRQMode mode, uint32_t lowValue, uint32_t highValue);
extern HAL_Status __HAL_ADC_FifoConfigChannel(ADC_Channel chan, ADC_Select select);
extern HAL_Status __HAL_ADC_Init(ADC_InitParam *initParam);
extern HAL_Status HAL_ADC_Init(ADC_InitParam *initParam);

__STATIC_INLINE void ADC_SetChanPinMux(ADC_Channel chan)
{
	HAL_SET_BIT(_adc_priv->chanPinMux, HAL_BIT(chan));
}

__STATIC_INLINE void ADC_ClrChanPinMux(ADC_Channel chan)
{
	HAL_CLR_BIT(_adc_priv->chanPinMux, HAL_BIT(chan));
}

__STATIC_INLINE uint8_t ADC_GetChanPinMux(ADC_Channel chan)
{
	return !!HAL_GET_BIT(_adc_priv->chanPinMux, HAL_BIT(chan));
}

__STATIC_INLINE void ADC_EnableADC(void)
{
	HAL_SET_BIT(ADC->CTRL, ADC_EN_BIT);
}

__STATIC_INLINE void ADC_DisableADC(void)
{
	HAL_CLR_BIT(ADC->CTRL, ADC_EN_BIT);
}

__STATIC_INLINE void ADC_EnableVbatDetec(void)
{
	HAL_SET_BIT(ADC->CTRL, ADC_VBAT_EN_BIT);
}

__STATIC_INLINE void ADC_DisableVbatDetec(void)
{
	HAL_CLR_BIT(ADC->CTRL, ADC_VBAT_EN_BIT);
}

__xip_text
__STATIC_INLINE void ADC_DisableAllChanCmp(void)
{
	HAL_CLR_BIT(ADC->CMP_SEL_EN, ADC_CMP_EN_MASK);
}

__STATIC_INLINE void ADC_EnableChanSel(ADC_Channel chan)
{
	HAL_SET_BIT(ADC->CMP_SEL_EN, HAL_BIT(ADC_SEL_EN_SHIFT + chan));
}

__STATIC_INLINE void ADC_DisableChanSel(ADC_Channel chan)
{
	HAL_CLR_BIT(ADC->CMP_SEL_EN, HAL_BIT(ADC_SEL_EN_SHIFT + chan));
}

__xip_text
__STATIC_INLINE void ADC_DisableAllChanSel(void)
{
	HAL_CLR_BIT(ADC->CMP_SEL_EN, ADC_SEL_EN_MASK);
}

__xip_text
__STATIC_INLINE void ADC_DisableAllChanIRQ(void)
{
	HAL_CLR_BIT(ADC->LOW_CONFIG, ADC_LOW_IRQ_MASK);
	HAL_CLR_BIT(ADC->HIGH_CONFIG, ADC_HIGH_IRQ_MASK);
	HAL_CLR_BIT(ADC->DATA_CONFIG, ADC_DATA_IRQ_MASK);
}

__STATIC_INLINE uint32_t ADC_GetDataPending(void)
{
	return HAL_GET_BIT(ADC->DATA_STATUS, ADC_DATA_PENDING_MASK);
}

__STATIC_INLINE void ADC_ClrDataPending(uint32_t dataPending)
{
	ADC->DATA_STATUS = dataPending;
}

__STATIC_INLINE uint32_t ADC_GetValue(ADC_Channel chan)
{
	return HAL_GET_BIT(ADC->DATA[chan], ADC_DATA_MASK);
}


__xip_text
HAL_Status HAL_ADC_Conv_Polling_VBAT(ADC_Channel chan, uint32_t *data, uint32_t msec)
{
	unsigned long flags;
	ADC_Private *priv = _adc_priv;
	uint32_t stopTime;
	uint8_t isTimeout;

	if (!priv) {
		HAL_WRN("%s not exist\n", __func__);
		return HAL_ERROR;
	}

	ADC_ASSERT_CHANNEL(chan);

	flags = HAL_EnterCriticalSection();
	if (priv->state == ADC_STATE_READY) {
		priv->state = ADC_STATE_BUSY;
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("ADC state: %d\n", priv->state);
		return HAL_ERROR;
	}
	HAL_ExitCriticalSection(flags);

	if ((!ADC_GetChanPinMux(chan)) && (chan != ADC_CHANNEL_VBAT)) {
		HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_ADC, chan), 0);
		ADC_SetChanPinMux(chan);
	}

	ADC_DisableAllChanSel();
	ADC_DisableAllChanCmp();
	ADC_DisableAllChanIRQ();

	if (chan == ADC_CHANNEL_VBAT)
		ADC_EnableVbatDetec();

	ADC_EnableChanSel(chan);

	if (msec == HAL_WAIT_FOREVER)
		stopTime = 0xFFFFFFFF;
	else
		stopTime = HAL_TicksToMSecs(HAL_Ticks()) + msec;

	if (stopTime < msec) {
		HAL_ERR("stopTime overflow.\n");
		return HAL_ERROR;
	}

	isTimeout = 1;
	ADC_EnableADC();
	while (HAL_TicksToMSecs(HAL_Ticks()) <= stopTime) {
		if (HAL_GET_BIT(ADC_GetDataPending(), HAL_BIT(chan))) {
			*data = ADC_GetValue(chan);
			ADC_ClrDataPending(ADC_GetDataPending());
			isTimeout = 0;
			break;
		}
	}
	ADC_DisableADC();
	ADC_DisableChanSel(chan);

	if (chan == ADC_CHANNEL_VBAT)
		ADC_DisableVbatDetec();

	if (ADC_GetChanPinMux(chan) && (chan != ADC_CHANNEL_VBAT)) {
		HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_ADC, chan), 0);
		ADC_ClrChanPinMux(chan);
	}

	flags = HAL_EnterCriticalSection();
	priv->state = ADC_STATE_READY;
	HAL_ExitCriticalSection(flags);

	if (isTimeout) {
		HAL_WRN("ADC timeout.\n");
		return HAL_TIMEOUT;
	} else {
		return HAL_OK;
	}
}

__xip_text
HAL_Status HAL_ADC_Conv_Polling(ADC_Channel chan, uint32_t *data, uint32_t msec)
{
	if (chan == ADC_CHANNEL_VBAT) {
		return HAL_ADC_Conv_Polling_VBAT(chan, data, msec);
	} else {
		return __HAL_ADC_Conv_Polling(chan, data, msec);
	}
}

HAL_Status HAL_ADC_ConfigChannel(ADC_Channel chan, ADC_Select select, ADC_IRQMode mode, uint32_t lowValue, uint32_t highValue)
{
	unsigned long flags;
	ADC_Private *priv = _adc_priv;

	if (chan != ADC_CHANNEL_VBAT) {
		return __HAL_ADC_ConfigChannel(chan, select, mode, lowValue, highValue);
	}

	if (!priv) {
		HAL_WRN("%s not exist\n", __func__);
		return HAL_ERROR;
	}

	ADC_ASSERT_CHANNEL(chan);

	if (priv->mode != ADC_CONTI_CONV) {
		HAL_ERR("Invalid call.\n");
		return HAL_ERROR;
	}

	if (((mode == ADC_IRQ_LOW_HIGH_DATA) || (mode == ADC_IRQ_LOW_HIGH)) && (lowValue > highValue)) {
		HAL_ERR("lowValue greater than highValue.\n");
		return HAL_ERROR;
	}

	flags = HAL_EnterCriticalSection();
	if ((priv->state == ADC_STATE_READY) || (priv->state == ADC_STATE_BUSY)) {
		if (priv->state == ADC_STATE_BUSY)
			ADC_DisableADC();
		if (select == ADC_SELECT_DISABLE) {
			ADC_DisableVbatDetec();
		} else {
			ADC_EnableVbatDetec();
		}
		__HAL_ADC_ConfigChannel(chan, select, mode, lowValue, highValue);
		HAL_ExitCriticalSection(flags);
		return HAL_OK;
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("ADC state: %d\n", priv->state);
		return HAL_ERROR;
	}
}

HAL_Status HAL_ADC_FifoConfigChannel(ADC_Channel chan, ADC_Select select)
{
	unsigned long flags;
	ADC_Private *priv = _adc_priv;

	if (chan != ADC_CHANNEL_VBAT) {
		return __HAL_ADC_FifoConfigChannel(chan, select);
	}

	if (!priv) {
		HAL_WRN("%s not exist\n", __func__);
		return HAL_ERROR;
	}

	ADC_ASSERT_CHANNEL(chan);

	if (priv->mode != ADC_BURST_CONV) {
		HAL_ERR("Invalid call.\n");
		return HAL_ERROR;
	}

	flags = HAL_EnterCriticalSection();
	if ((priv->state == ADC_STATE_READY) || (priv->state == ADC_STATE_BUSY)) {
		if (priv->state == ADC_STATE_BUSY)
			ADC_DisableADC();
		if (select == ADC_SELECT_DISABLE) {
			ADC_ClrChanPinMux(chan);
			ADC_DisableVbatDetec();
		} else {
			ADC_SetChanPinMux(chan);
			ADC_EnableVbatDetec();
		}
		__HAL_ADC_FifoConfigChannel(chan, select);
		HAL_ExitCriticalSection(flags);
		return HAL_OK;
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("ADC state: %d\n", priv->state);
		return HAL_ERROR;
	}
}

#ifdef CONFIG_PM

__xip_text
static int adc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	ADC_Private *priv = dev->platform_data;

	priv->suspend = 1;

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->suspend_bypass & PM_SUPPORT_SLEEP)
			break;
	case PM_MODE_STANDBY:
#if (CONFIG_CHIP_ARCH_VER == 3)
		if (priv->suspend_bypass & PM_SUPPORT_STANDBY) {
			HAL_PRCM_SelectGpadcClkSource(PRCM_GPADC_MCLK_LFCLK_CRYSTAL);
			HAL_PRCM_WAKEUP_SRC_DisableClkGating(PRCM_WAKEUP_SRC_BUS_CLK_BIT_GPADC);
			break;
		}
#endif
	case PM_MODE_HIBERNATION:
		if (priv->state == ADC_STATE_BUSY)
			HAL_ADC_Stop_Conv_IT();
		HAL_ADC_DeInit();
		break;
	default:
		break;
	}

	return 0;
}

__xip_text
static int adc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	static ADC_Channel chan;
	ADC_Private *priv = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->suspend_bypass & PM_SUPPORT_SLEEP)
			break;
	case PM_MODE_STANDBY:
#if (CONFIG_CHIP_ARCH_VER == 3)
		if (priv->suspend_bypass & PM_SUPPORT_STANDBY) {
			HAL_PRCM_WAKEUP_SRC_EnableClkGating(PRCM_WAKEUP_SRC_BUS_CLK_BIT_GPADC);
			HAL_PRCM_SelectGpadcClkSource(priv->param.work_clk == ADC_WORKCLK_LFCLK ? PRCM_GPADC_MCLK_LFCLK_CRYSTAL : PRCM_GPADC_MCLK_HFCLK_CRYSTAL);
			break;
		}
#endif
	case PM_MODE_HIBERNATION:
		HAL_ADC_Init(&priv->param);
		for (chan = ADC_CHANNEL_0; chan < ADC_CHANNEL_NUM; chan++) {
			if (priv->chan_config[chan].is_config)
				HAL_ADC_ConfigChannel(chan,
				                      (ADC_Select)priv->chan_config[chan].select,
				                      (ADC_IRQMode)priv->chan_config[chan].irqmode,
				                      priv->chan_config[chan].lowValue,
				                      priv->chan_config[chan].highValue);
		}
		HAL_ADC_Start_Conv_IT();
		break;
	default:
		break;
	}

	priv->suspend = 0;

	return 0;
}

extern char adc_name[];

__xip_rodata
static const struct soc_device_driver adc_drv = {
	.name = adc_name,
	.suspend = adc_suspend,
	.resume = adc_resume,
};

#endif

__xip_text
HAL_Status HAL_ADC_Init(ADC_InitParam *initParam)
{
	HAL_Status status;
	ADC_Private *priv;

	status = __HAL_ADC_Init(initParam);
	if (status != HAL_OK) {
		return status;
	}
	priv = _adc_priv;

#ifdef CONFIG_PM
	if (!priv->suspend) {
		pm_unregister_ops(&priv->dev);
		priv->suspend_bypass = initParam->suspend_bypass;
		priv->dev.name = adc_name;
		priv->dev.driver = &adc_drv;
		HAL_Memcpy(&priv->param, initParam, sizeof(ADC_InitParam));
		HAL_Memset(priv->chan_config, 0, ADC_CHANNEL_NUM * sizeof(struct adc_chan_config));
		priv->dev.platform_data = priv;
		pm_register_ops(&priv->dev);
	}
#else
	(void)priv;
#endif
	return HAL_OK;
}

#endif /*CONFIG_ROM*/
