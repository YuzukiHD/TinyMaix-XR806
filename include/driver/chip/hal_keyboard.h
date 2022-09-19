/**
 * @file  hal_keyboard.h
 * @author  XRADIO IOT Team
 */

/*
 * Copyright (C) 2018 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef _HAL_KEYBOARD_H_
#define _HAL_KEYBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hal_def.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/**
  * Digital KeyPad Controller
  */
typedef struct {
	__IO uint32_t COL_CTL;          /* Keypad Column Control Register, Address offset: 0x00 */
	__IO uint32_t ROW_CTL;          /* Keypad Row Control Register, Address offset: 0x04 */
	__IO uint32_t SAMPLE_CTL;       /* Keypad Sample Control Register, Address offset: 0x08 */
	__IO uint32_t TIMING;           /* Keypad Timing Register, Address offset: 0x0c */
	__IO uint32_t INT_CFG;          /* Keypad Interrupt Configure Register, Address offset: 0x10 */
	__IO uint32_t INT_STA;          /* Keypad Interrupt Status Register, Address offset: 0x14 */
	__IO uint32_t IN_DATA[8];       /* Keypad Input Data Register, Address offset: 0x18--0x34 */
} KeyBoard_T;

#define KEYBOARD       ((KeyBoard_T *)KEYSCAN_CTRL_BASE)

/*
 * Bits definition for COL_CTL Register (0x0000)
 */
#define KEYBOARD_INTERFACE_EN_BIT       HAL_BIT(0)
#define KEYBOARD_COL_OUTPUT_SHIFT       1
#define KEYBOARD_COL_OUTPUT_MASK        (0xff << KEYBOARD_COL_OUTPUT_SHIFT)
/*
 * Bits definition for ROW_CTL Register (0x0004)
 */
#define KEYBOARD_ROW_INPUT_SHIFT        0
#define KEYBOARD_ROW_INPUT_MASK         (0xfffff << KEYBOARD_ROW_INPUT_SHIFT)

/*
 * Bits definition for SAMPLE_CTL Register (0x0008)
 */
#define KEYBOARD_SMPL_OPTM_EN_BIT       HAL_BIT(0)
#define KEYBOARD_SMPL_CTU_EN_BIT        HAL_BIT(1)

#define KEYBOARD_DUR_TIMECYCLE_SHIFT    2
#define KEYBOARD_DUR_TIMECYCLE_MASK     (0x7ffff << KEYBOARD_DUR_TIMECYCLE_SHIFT)
#define KEYBOARD_DUR_TIMECYCLE(TIME)    (TIME << KEYBOARD_DUR_TIMECYCLE_SHIFT)

/*
 * Bits definition for TIMING Register (0x000C)
 */
#define KEYBOARD_SCAN_CYCLE_SHIFT       0
#define KEYBOARD_SCAN_CYCLE_MASK        (0xffff << KEYBOARD_SCAN_CYCLE_SHIFT)

#define KEYBOARD_DBC_CYCLE_SHIFT        16
#define KEYBOARD_DBC_CYCLE_MASK         (0xffff << KEYBOARD_DBC_CYCLE_SHIFT)

/*
 * Bits definition for INT_CFG Register (0x0010)
 */
#define KEYBOARD_IT_FEDGE_IRQ           HAL_BIT(0)
#define KEYBOARD_IT_REDGE_IRQ           HAL_BIT(1)
#define KEYBOARD_IT_OPTM_IRQ            HAL_BIT(2)

/*
 * Bits definition for INT_CFG Register (0x0014)
 */
#define KEYBOARD_IT_FEDGE_PENDING       HAL_BIT(0)
#define KEYBOARD_IT_REDGE_PENDING       HAL_BIT(1)


typedef void (*KeyBoard_IRQCallback)(uint32_t *rowdata, uint8_t len);

typedef struct {
	uint32_t freq;          /* moudel clock*/
	uint32_t cols;          /* column output mask (1-mask) max 8-channel */
	uint32_t rows;          /* row input mask (1-mask) max 16-channel */
	uint32_t timecycle;     /* sample cycle */
	uint16_t scan;          /* scan period clock cycle */
	uint16_t debounce;      /* debounce clock cycle */
} keyboard_InitParam;

#define KEY_SCAN_BUFFER_MAX         4
#define KEY_SCAN_COLS_MAX           8
#define KEY_SCAN_ROWS_MAX           16


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

HAL_Status HAL_KeyBoard_Open(keyboard_InitParam *cfg, KeyBoard_IRQCallback cb);

HAL_Status HAL_KeyBoard_Close(void);

/**
 * @brief Initialize the Keypad according to the specified parameters
 * @param param Pointer to keypad_initParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_KeyBoard_Init(keyboard_InitParam *cfg);

/**
 * @brief DeInitialize the specified Keypad
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_KeyBoard_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_KEYBOARD_H_ */

