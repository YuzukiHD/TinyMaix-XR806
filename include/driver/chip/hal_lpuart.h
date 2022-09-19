/**
 * @file  hal_lpuart.h
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

#ifndef _DRIVER_CHIP_HAL_LPUART_H_
#define _DRIVER_CHIP_HAL_LPUART_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LPUART ID definition
 */
typedef enum {
	LPUART0_ID = 0U,
	LPUART1_ID,
	LPUART_NUM,
	LPUART_INVALID_ID = 0xFFU
} LPUART_ID;

/**
 * @brief LPUART register block structure
 */
typedef struct {
	__IO uint32_t CTRL;             /* offset: 0x00, LPUART control register */
	__IO uint32_t BAUD_CFG;         /* offset: 0x04, LPUART baudrate config register */
		 uint32_t RESERVED1[2];
	__IO uint32_t RX_CFG;           /* offset: 0x10, LPUART RX config register */
	__I  uint32_t RX_DATA;          /* offset: 0x14, LPUART RX data register */
		 uint32_t RESERVED2[2];
	__IO uint32_t IRQ_EN;           /* offset: 0x20, LPUART interrupt enable register */
	__I  uint32_t IRQ_S;            /* offset: 0x24, LPUART interrupt status register */
	__IO uint32_t IRQ_CLR;          /* offset: 0x28, LPUART interrupt clear register */
	__IO uint32_t RX_CMP1;          /* offset: 0x2C, LPUART RX compare_1 register */
	__IO uint32_t RX_CMP2;          /* offset: 0x30, LPUART RX compare_2 register */
} LPUART_T;

#define LPUART0  ((LPUART_T *)LPUART0_BASE)   /* address: 0x40045800 */
#define LPUART1  ((LPUART_T *)LPUART1_BASE)   /* address: 0x40045C00 */


/* LPUART->MODEM_CTRL, R/W */
#define LPUART_CFG_SOFT_RST_BIT         HAL_BIT(8)

/* LPUART->BAUD_CFG, R/W */
#define LPUART_DIV_MASK                 0xFFU
#define LPUART_QUOT_MASK                0xFFFFU
#define LPUART_CFG_DIVISOR_POS          24
#define LPUART_CFG_REMAINDER_POS        16
#define LPUART_CFG_QUOTIENT_POS         0

/* LPUART->RX_CFG, R/W */
#define LPUART_CFG_MSB_FIRST_SHIFT      12
#define LPUART_CFG_MSB_FIRST_MASK       (0x1U << LPUART_CFG_MSB_FIRST_SHIFT)

#define LPUART_DATA_WIDTH_SHIFT         8
#define LPUART_DATA_WIDTH_MASK          (0x7U << LPUART_DATA_WIDTH_SHIFT)
typedef enum {
	LPUART_DATA_WIDTH_4            = (0x0U << LPUART_DATA_WIDTH_SHIFT),
	LPUART_DATA_WIDTH_5            = (0x1U << LPUART_DATA_WIDTH_SHIFT),
	LPUART_DATA_WIDTH_6            = (0x2U << LPUART_DATA_WIDTH_SHIFT),
	LPUART_DATA_WIDTH_7            = (0x3U << LPUART_DATA_WIDTH_SHIFT),
	LPUART_DATA_WIDTH_8            = (0x4U << LPUART_DATA_WIDTH_SHIFT),
	LPUART_DATA_WIDTH_9            = (0x5U << LPUART_DATA_WIDTH_SHIFT)
} LPUART_Data_Width;

#define LPUART_PARITY_SEL_SHIFT         4
#define LPUART_PARITY_SEL_MASK          (0x7U << LPUART_PARITY_SEL_SHIFT)
typedef enum {
	LPUART_PARITY_NONE   = 0U,
	LPUART_PARITY_EVEN   = (0x1U << LPUART_PARITY_SEL_SHIFT),
	LPUART_PARITY_ODD    = (0x2U << LPUART_PARITY_SEL_SHIFT),
	LPUART_PARITY_SPACE  = (0x3U << LPUART_PARITY_SEL_SHIFT),     /* the parity bit always present 0 */
	LPUART_PARITY_MARK   = (0x4U << LPUART_PARITY_SEL_SHIFT)      /* the parity bit always present 1 */
} LPUART_Parity;

#define LPUART_RPE_RPT                  HAL_BIT(2)
#define LPUART_RX_EN                    HAL_BIT(0)

/* LPUART->RX_DATA, R/W */
#define LPUART_RX_DATA_MASK             0x1FFU

/* LPUART->IRQ_EN, R/W */
#define LPUART_RX_DATA_EN_BIT           HAL_BIT(9)
#define LPUART_RX_DATA_CMP_EN_BIT       HAL_BIT(7)

/* LPUART->IRQ_S, R */
#define LPUART_IRQ_FLAG_MASK            0x280U
#define LPUART_RX_DATA_FLAG_BIT         HAL_BIT(9)
#define LPUART_RX_DATA_CMP_FLAG_BIT     HAL_BIT(7)

/* LPUART->IRQ_CLR, W */
#define LPUART_RX_DATA_CLR_BIT          HAL_BIT(9)
#define LPUART_RX_DATA_CMP_CLR_BIT      HAL_BIT(7)

/* LPUART->RX_CMP1,LPUART->RX_CMP2 R/W */
#define LPUART_RX_CMP_LEN_MASK          0x7U
#define LPUART_RX_CMP_LEN_SHIFT         0
#define LPUART_RX_CMP_DATA_MASK         0x1FFU
#define LPUART_RX_CMP_DATA0_POS         3
#define LPUART_RX_CMP_DATA1_POS         12
#define LPUART_RX_CMP_DATA2_POS         21
#define LPUART_RX_CMP_DATA3_POS         3
#define LPUART_RX_CMP_DATA4_POS         12
typedef enum {
	LPUART_RX_CMP_DATA_NUM_1    = 1,
	LPUART_RX_CMP_DATA_NUM_2,
	LPUART_RX_CMP_DATA_NUM_3,
	LPUART_RX_CMP_DATA_NUM_4,
	LPUART_RX_CMP_DATA_NUM_5,
	LPUART_RX_CMP_DATA_NUM_MAX  = 5,
} LPUART_RX_CMP_DATA_NUM;

/**
 * @brief LPUART initialization parameters
 */
typedef struct {
	uint32_t            baudRate;               /* Baud rate, in bps */
	uint32_t            msbFirst;               /* tx msb first */
	PRCM_LPUART_WAKEUP_IN_SEL input_uart;       /* select which uart as input */
	LPUART_Parity       parity;                 /* Parity */
	LPUART_Data_Width   dataWidth;              /* Data width */
} LPUART_InitParam;

/** @brief Type define of LPUART receive compare callback function */
typedef void (*LPUART_RxCmpCallback)(void *arg);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize the LPUART according to the specified parameters
 * @param[in] param Pointer to LPUART_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_Init(LPUART_ID lpuartID, const LPUART_InitParam *param);

/**
 * @brief DeInitialize the specified LPUART
 * @param[in] lpuartID ID of the specified LPUART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_DeInit(LPUART_ID lpuartID);

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
int32_t HAL_LPUART_Receive_IT(LPUART_ID lpuartID, uint8_t *buf, int32_t size, uint32_t msec);

/**
 * @brief setup rx data compare function
 * @param[in] lpuartID ID of the specified LPUART
 * @param[in] cmp_len the length of compare data
 * @param[in] cmp_data buffer point to compare data
 * @retval HAL_Status, HAL_OK on success
 *
 * @note
 */
HAL_Status HAL_LPUART_EnableRxCmp(LPUART_ID lpuartID, uint8_t cmp_len, uint8_t *cmp_data);

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
HAL_Status HAL_LPUART_EnableRxCmpCallback(LPUART_ID lpuartID, LPUART_RxCmpCallback cb, void *arg);

/**
 * @brief Disable receive compare callback function for the specified LPUART
 * @param[in] lpuartID ID of the specified LPUART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_DisableRxCmpCallback(LPUART_ID lpuartID);

/**
 * @brief Set PM mode to be bypassed
 * @param[in] lpuartID ID of the specified LPUART
 * @param[in] mode Bit mask of PM mode to be bypassed
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_LPUART_SetBypassPmMode(LPUART_ID lpuartID, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_LPUART_H_ */
