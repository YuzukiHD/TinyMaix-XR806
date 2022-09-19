/**
 * @file  hal_efuse.h
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

#ifndef _DRIVER_CHIP_HAL_EFUSE_H_
#define _DRIVER_CHIP_HAL_EFUSE_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief EFUSE register block structure
 */
typedef struct {
	     uint32_t RESERVED0[16];
	__IO uint32_t CTRL;          /* offset: 0x0040 EFUSE program/read control register */
	     uint32_t RESERVED1[3];
	__IO uint32_t PROGRAM_VALUE; /* offset: 0x0050 EFUSE program key value register */
	     uint32_t RESERVED2[3];
	__I  uint32_t READ_VALUE;    /* offset: 0x0060 EFUSE read key value register */
	     uint32_t RESERVED3[11];
	__IO uint32_t TIMING_CTRL;   /* offset: 0x0090 EFUSE burned timing control register */
	__IO uint32_t DEBUG_REG;     /* offset: 0x0094 EFUSE debug register */
	     uint32_t RESERVED4[26];
	__IO uint32_t VALUE[32];     /* offset: 0x0100 EFUSE value register */
} EFUSE_T;

#define EFUSE   ((EFUSE_T *)SID_BASE) /* address: 0x40043C00 */

/* EFUSE->CTRL */
#define EFUSE_CLK_GATE_EN_BIT           HAL_BIT(28)

#define EFUSE_INDEX_SHIFT               16
#define EFUSE_INDEX_MASK                ((0xFFU) << EFUSE_INDEX_SHIFT)

#define EFUSE_OPERA_LOCK_SHIFT          8
#define EFUSE_OPERA_LOCK_MASK           ((0xFFU) << EFUSE_OPERA_LOCK_SHIFT)
#define EFUSE_OPERA_UNLOCK_VAL          0xACU

#if (CONFIG_CHIP_ARCH_VER == 3)
#define EFUSE_UPDATE_ALL_BITS_BIT       HAL_BIT(4)
#endif

#define EFUSE_HW_READ_STATUS_BIT        HAL_BIT(2)
#define EFUSE_SW_READ_START_BIT         HAL_BIT(1)
#define EFUSE_SW_PROG_START_BIT         HAL_BIT(0)

/* EFUSE->TIMING_CTRL */
typedef enum {
#if (CONFIG_CHIP_ARCH_VER == 1)
	EFUSE_TIMING_PARAM_24M = 0x63321190U,
	EFUSE_TIMING_PARAM_26M = 0x63321190U,
	EFUSE_TIMING_PARAM_40M = 0xb55012A8U,
	EFUSE_TIMING_PARAM_52M = 0xC6642320U,
#elif (CONFIG_CHIP_ARCH_VER == 2)
	EFUSE_TIMING_PARAM_24M = 0x631180F0U,
	EFUSE_TIMING_PARAM_26M = 0x63136104U,
	EFUSE_TIMING_PARAM_40M = 0x631DC190U,
	EFUSE_TIMING_PARAM_52M = 0x8426C208U,
#elif (CONFIG_CHIP_ARCH_VER == 3)
	EFUSE_TIMING_PARAM_24M = 0x421220F3U,
	EFUSE_TIMING_PARAM_26M = 0x42139107U,
	EFUSE_TIMING_PARAM_32M = 0x5217D140U,
	EFUSE_TIMING_PARAM_40M = 0x631DC190U,
#endif
} EFUSE_TimingParam;

/******************************************************************************/

/** @brief The number of bits on chip EFUSE */
#define HAL_EFUSE_BIT_NUM       (1024)

#ifdef CONFIG_TZ_EFUSE

#include "trustzone/nsc_table.h"
#include "trustzone/tz_thread.h"

#define EFUSE_Init(...)                 TEE_FUNC_CALL(EFUSE_Init_NSC, ##__VA_ARGS__)
#define EFUSE_ReadData(...)             TEE_FUNC_CALL(EFUSE_ReadData_NSC, ##__VA_ARGS__)
#define EFUSE_WriteData(...)            TEE_FUNC_CALL(EFUSE_WriteData_NSC, ##__VA_ARGS__)
#if (CONFIG_CHIP_ARCH_VER == 3)
#define EFUSE_UpdateAll(...)            TEE_FUNC_CALL(EFUSE_UpdateAll_NSC, ##__VA_ARGS__)
#endif

#else /* CONFIG_TZ_EFUSE */

HAL_Status EFUSE_Init(void);
void EFUSE_ReadData(uint8_t index, uint32_t *pData);
void EFUSE_WriteData(uint8_t index, uint32_t data);
#if (CONFIG_CHIP_ARCH_VER == 3)
void EFUSE_UpdateAll(void);
#endif

#endif /* CONFIG_TZ_EFUSE */

/**
 * @brief Read an amount of data from EFUSE
 * @param[in] start_bit The first bit to be read on EFUSE
 * @param[in] bit_num Number of bits to be read
 * @param[in] data Pointer to the data buffer
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_EFUSE_Read(uint32_t start_bit, uint32_t bit_num, uint8_t *data);

/**
 * @brief Write an amount of data to EFUSE
 * @param[in] start_bit The first bit to be written on EFUSE
 * @param[in] bit_num Number of bits to be written
 * @param[in] data Pointer to the data buffer
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_EFUSE_Write(uint32_t start_bit, uint32_t bit_num, uint8_t *data);

/**
 * @brief update all EFUSE bits
 */
void HAL_EFUSE_UpdateAll(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_EFUSE_H_ */
