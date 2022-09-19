/**
 * @file  hal_adc.h
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

#ifndef _DRIVER_CHIP_HAL_ADC_H_
#define _DRIVER_CHIP_HAL_ADC_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADC channel definition
 */
typedef enum {
	ADC_CHANNEL_0   = 0,
	ADC_CHANNEL_1   = 1,
	ADC_CHANNEL_2   = 2,
	ADC_CHANNEL_3   = 3,
	ADC_CHANNEL_4   = 4,
	ADC_CHANNEL_5   = 5,
	ADC_CHANNEL_6   = 6,
	ADC_CHANNEL_8   = 8,
	ADC_CHANNEL_NUM
} ADC_Channel;

/* ADC channel for VBAT voltage detection */
#define ADC_CHANNEL_VBAT    ADC_CHANNEL_8

/**
 * @brief ADC register block structure
 */
typedef struct {
	__IO uint32_t SAMPLE_RATE;               /* offset: 0x00, GPADC sample rate configure register */
	__IO uint32_t CTRL;                      /* offset: 0x04, GPADC control Register */
	__IO uint32_t CMP_SEL_EN;                /* offset: 0x08, GPADC compare and select enable register */
	__IO uint32_t FIFO_CTRL;                 /* offset: 0x0C, GPADC FIFO interrupt control register */
	__IO uint32_t FIFO_STATUS;               /* offset: 0x10, GPADC FIFO interrupt status register */
	__IO uint32_t FIFO_DATA;                 /* offset: 0x14, GPADC FIFO data register */
	__IO uint32_t CALIB_DATA;                /* offset: 0x18, GPADC calibration data register */
	__IO uint32_t RESERVED0;
	__IO uint32_t LOW_CONFIG;                /* offset: 0x20, GPADC data low interrupt configure register */
	__IO uint32_t HIGH_CONFIG;               /* offset: 0x24, GPADC data high interrupt configure register */
	__IO uint32_t DATA_CONFIG;               /* offset: 0x28, GPADC data interrupt configure register */
	__IO uint32_t RESERVED1;
	__IO uint32_t LOW_STATUS;                /* offset: 0x30, GPADC data low interrupt status register */
	__IO uint32_t HIGH_STATUS;               /* offset: 0x34, GPADC data high interrupt status register */
	__IO uint32_t DATA_STATUS;               /* offset: 0x38, GPADC data interrupt status register */
	__IO uint32_t RESERVED2;
	__IO uint32_t CMP_DATA[ADC_CHANNEL_NUM]; /* offset: 0x40, GPADC CH n compare data register */
	__IO uint32_t RESERVED3[7];
	__I  uint32_t DATA[ADC_CHANNEL_NUM];     /* offset: 0x80, GPADC CH n data register */
} ADC_T;

#define ADC     ((ADC_T *)GPADC_BASE)   /* address: 0x40043000 */

/* ADC->SAMPLE_RATE */
#define ADC_FS_DIV_SHIFT                16
#define ADC_FS_DIV_MASK                 (0xFFFFU << ADC_FS_DIV_SHIFT)

#define ADC_TACQ_SHIFT                  0
#define ADC_TACQ_MASK                   (0xFFFFU << ADC_TACQ_SHIFT)

/* ADC->CTRL */
#define ADC_FIRST_DELAY_SHIFT           24
#define ADC_FIRST_DELAY_MASK            (0xFFU << ADC_FIRST_DELAY_SHIFT)

#define ADC_OP_BIAS_SHIFT               20
#define ADC_OP_BIAS_MASK                (0x3U << ADC_OP_BIAS_SHIFT)

#define ADC_WORK_MODE_SHIFT             18
#define ADC_WORK_MODE_MASK              (0x3U << ADC_WORK_MODE_SHIFT)
typedef enum {
	ADC_SINGLE_CONV     = 0U,
	ADC_SINGLE_CYCLE    = 1U,
	ADC_CONTI_CONV      = 2U,
	ADC_BURST_CONV      = 3U
} ADC_WorkMode;

#define ADC_CALIB_EN_BIT                HAL_BIT(17)
#define ADC_EN_BIT                      HAL_BIT(16)
#define ADC_VBAT_EN_BIT                 HAL_BIT(5)

#if (CONFIG_CHIP_ARCH_VER > 1)
#define ADC_VREF_MODE_SEL_SHIFT         1
#define ADC_VREF_MODE_SEL_MASK          (0x7U << ADC_VREF_MODE_SEL_SHIFT)

/**
 * @brief ADC vref mode select
 */
typedef enum {
	ADC_VREF_MODE_0         = 0U,   /*Vref select inside voltage, input range of 0~1.4V*/
	ADC_VREF_MODE_1         = 1U,   /*Vref select inside voltage, input range of 0~2.5V*/
	ADC_VREF_MODE_2         = 2U,   /*Vref select outside pin VDDIO, EXT_LDO 3.3V*/
	ADC_VREF_MODE_3         = 6U,   /*Vref select outside pin VDDIO, EXT_LDO 3.1V*/
} ADC_VrefMode;
#endif /* CONFIG_CHIP_ARCH_VER */

#define ADC_LDO_EN_BIT                  HAL_BIT(0)

/* ADC->CMP_SEL_EN */
#define ADC_CMP_EN_SHIFT                16
#define ADC_CMP_EN_MASK                 (0x1FFU << ADC_CMP_EN_SHIFT)

#define ADC_SEL_EN_SHIFT                0
#define ADC_SEL_EN_MASK                 (0x1FFU << ADC_SEL_EN_SHIFT)

/*
 * ADC->LOW_CONFIG
 * ADC->HIGH_CONFIG
 * ADC->DATA_CONFIG
 */
#define ADC_LOW_IRQ_MASK                0x1FFU
#define ADC_HIGH_IRQ_MASK               0x1FFU
#define ADC_DATA_IRQ_MASK               0x1FFU

/*
 * ADC->FIFO_CTRL
 */
#define ADC_FIFO_DATA_DRQ_MASK          HAL_BIT(18)
#define ADC_FIFO_OVERUN_IRQ_MASK        HAL_BIT(17)
#define ADC_FIFO_DATA_IRQ_MASK          HAL_BIT(16)

#define ADC_FIFO_LEVEL_SHIFT            8
#define ADC_FIFO_LEVEL_MASK             (0x3FU << ADC_FIFO_LEVEL_SHIFT)

#define ADC_FIFO_FLUSH_MASK             HAL_BIT(4)

/*
 * ADC->FIFO_STATUS
 */
#define ADC_FIFO_OVERUN_PENDING_MASK    HAL_BIT(17)
#define ADC_FIFO_DATA_PENDING_MASK      HAL_BIT(16)

/* ADC->FIFO_DATA */
#define ADC_FIFO_DATA_MASK              0xFFFU

/*
 * ADC->LOW_STATUS
 * ADC->HIGH_STATUS
 * ADC->DATA_STATUS
 */
#define ADC_LOW_PENDING_MASK            0x1FFU
#define ADC_HIGH_PENDING_MASK           0x1FFU
#define ADC_DATA_PENDING_MASK           0x1FFU

/* ADC->CMP_DATA */
#define ADC_HIGH_DATA_SHIFT             16
#define ADC_HIGH_DATA_MASK              (0xFFFU << ADC_HIGH_DATA_SHIFT)

#define ADC_LOW_DATA_SHIFT              0
#define ADC_LOW_DATA_MASK               (0xFFFU << ADC_LOW_DATA_SHIFT)

/* ADC->DATA */
#define ADC_DATA_MASK                   0xFFFU

/******************************************************************************/

/**
 * @brief ADC work clock selected definition
 */
typedef enum {
	ADC_WORKCLK_HFCLK   = 0,
	ADC_WORKCLK_LFCLK   = 1
} ADC_WorkClk;

/**
 * @brief ADC channel initialization parameters
 */
typedef struct {
#if (CONFIG_CHIP_ARCH_VER == 3)
	ADC_WorkClk         work_clk;   /* ADC word clock select */
#endif
	uint32_t            freq;       /* ADC sample frequency */
	uint8_t             delay;      /* the number of delayed samples in first conversion */
	uint8_t         suspend_bypass;
#if (CONFIG_CHIP_ARCH_VER > 1)
	ADC_VrefMode        vref_mode;  /* VERF mode select */
#endif
	ADC_WorkMode        mode;       /* ADC work mode */
} ADC_InitParam;

/**
 * @brief ADC channel selected state definition
 */
typedef enum {
	ADC_SELECT_DISABLE  = 0,
	ADC_SELECT_ENABLE   = 1
} ADC_Select;

/**
 * @brief ADC interrupt mode definition
 */
typedef enum {
	ADC_IRQ_NONE            = 0,
	ADC_IRQ_DATA            = 1,
	ADC_IRQ_LOW             = 2,
	ADC_IRQ_HIGH            = 3,
	ADC_IRQ_LOW_DATA        = 4,
	ADC_IRQ_HIGH_DATA       = 5,
	ADC_IRQ_LOW_HIGH        = 6,
	ADC_IRQ_LOW_HIGH_DATA   = 7
} ADC_IRQMode;

/**
 * @brief ADC interrupt state definition
 */
typedef enum {
	ADC_NO_IRQ          = 0,
	ADC_DATA_IRQ        = 1,
	ADC_LOW_IRQ         = 2,
	ADC_HIGH_IRQ        = 3,
	ADC_LOW_DATA_IRQ    = 4,
	ADC_HIGH_DATA_IRQ   = 5
} ADC_IRQState;

/** @brief Type define of ADC interrupt callback function */
typedef void (*ADC_IRQCallback)(void *arg);

/**
 * @brief Initialize the ADC according to the specified parameters.
 * @param[in] initParam Pointer to ADC_InitParam structure.
 * @retval HAL_Status, HAL_OK on success.
 */
HAL_Status HAL_ADC_Init(ADC_InitParam *initParam);

/**
 * @brief DeInitialize the ADC
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_DeInit(void);

/**
 * @brief The specified ADC channel convert once in polling mode
 * @param[in] chan The specified ADC channel
 * @param[in] data Pointer to the output data
 * @param[in] msec Timeout value in millisecond of conversion
 *                 HAL_WAIT_FOREVER for no timeout
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_Conv_Polling(ADC_Channel chan, uint32_t *data, uint32_t msec);

/**
 * @brief Enable interrupt callback function for the specified ADC channel
 * @param[in] chan The specified ADC channel
 * @param[in] cb The interrupt callback function
 * @param[in] arg Argument of the interrupt callback function
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_EnableIRQCallback(ADC_Channel chan, ADC_IRQCallback cb, void *arg);

/**
 * @brief Disable interrupt callback function for the specified ADC channel
 * @param[in] chan The specified ADC channel
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_DisableIRQCallback(ADC_Channel chan);

/**
 * @brief Configure the specified ADC channel for conversion in interrupt mode(CHAN mode)
 * @param[in] chan The specified ADC channel
 * @param[in] select ADC channel selected state
 * @param[in] mode ADC interrupt mode
 * @param[in] lowValue lower limit value in interrupt mode of ADC_IRQ_LOW,
 *            ADC_IRQ_LOW_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA
 * @param[in] highValue Upper limit value in interrupt mode of ADC_IRQ_HIGH,
 *            ADC_IRQ_HIGH_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA
 * @retval HAL_Status, HAL_OK on success, HAL_ERROR on fail
 */
HAL_Status HAL_ADC_ConfigChannel(ADC_Channel chan, ADC_Select select, ADC_IRQMode mode, uint32_t lowValue, uint32_t highValue);

/**
 * @brief Configure the specified ADC channel for conversion in interrupt mode(FIFO mode)
 * @param[in] chan The specified ADC channel
 * @param[in] select ADC channel selected state
 * @retval HAL_Status, HAL_OK on success, HAL_ERROR on fail, HAL_INVALID on invalid argument
 * @note When this function is called, the FIFO  will be flushed firstly
 */
HAL_Status HAL_ADC_FifoConfigChannel(ADC_Channel chan, ADC_Select select);

/**
 * @brief Start the ADC conversion in interrupt mode
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_Start_Conv_IT(void);

/**
 * @brief Stop the ADC conversion in interrupt mode
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_Stop_Conv_IT(void);

/**
 * @brief Get interrupt mode of the specified ADC channel
 * @param[in] chan The specified ADC channel
 * @retval HAL_Status, HAL_OK on success
 */
ADC_IRQState HAL_ADC_GetIRQState(ADC_Channel chan);

/**
 * @brief Get digital value of the specified ADC channel
 * @param[in] chan The specified ADC channel
 * @return Digital value converted by the specified ADC channel
 */
uint32_t HAL_ADC_GetValue(ADC_Channel chan);

/**
 * @brief Get digital value of the ADC fifo
 * @param[in] NULL
 * @return Digital value converted by the ADC fifo
 * @note before call it, fifo data should be available
 */
uint32_t HAL_ADC_GetFifoData(void);

/**
 * @brief Get data count of ADC fifo
 * @param[in] NULL
 * @return the specified data count of ADC fifo
 */
uint8_t HAL_ADC_GetFifoDataCount(void);

/**
 * @brief modify adc vref voltage
 * @param Mode Mode 0 usage is verf inside soc, and input voltage range of 0~1.4v.
 * @           Mode 1 usage is verf inside soc, and input voltage range of 0~2.5v.
 * @           Mode 2 usage is verf outside pin, and input voltage range of 0~2.5v.
 * @return None
 * @NOTE: Mode 0 and Mode 1 can calculate virtual voltage according to ADC Value But Mode 2 and Mode 3 NOT.
          Mode 2 and Mode 3 adc value will not expand when vddio and channel voltage change at the same time.
 */
void HAL_ADC_Set_VrefMode(ADC_VrefMode mode);

/**
 * @brief Set PM mode to be bypassed
 * @param[in] mode Bit mask of PM mode to be bypassed
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_ADC_SetBypassPmMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_ADC_H_ */
