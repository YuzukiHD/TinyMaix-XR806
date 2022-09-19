/**
 * @file  hal_wdg.h
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

#ifndef _DRIVER_CHIP_HAL_SYSCTL_H_
#define _DRIVER_CHIP_HAL_SYSCTL_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SYSCTL register block structure
 */
typedef struct {
	     uint32_t RESERVED0[17];     /* offset: 0x00 */
	__IO uint32_t SIP_TEST_MAP;      /* offset: 0x44 */
	__IO uint32_t SRAM_SHARE;        /* offset: 0x48 */
	     uint32_t RESERVED1;         /* offset: 0x4C */
	__IO uint32_t PS_CTL_REG;        /* offset: 0x50 */
	__IO uint32_t PS_CNT_REG;        /* offset: 0x54 */
#if (CONFIG_CHIP_ARCH_VER == 2)
	     uint32_t RESERVED2[22];     /* offset: 0x58 */
#elif (CONFIG_CHIP_ARCH_VER == 3)
	     uint32_t RESERVED58[2];     /* offset: 0x58 */
	__IO uint32_t FLASH_PSRAM_PIN_MAP_REG;      /* offset: 0x60 */
	     uint32_t RESERVED64[19];    /* offset: 0x64 */
#endif
	__IO uint32_t GENRAL_DBG_REG[2]; /* offset: 0xB0 */
} SYSCTL_T;

#define SYSCTL ((SYSCTL_T *)SYSCTL_BASE) /* address: 0x4000A000 */

/* SYSCTL->SRAM_SHARE 0x48 */
#define SYSCTL_PIN_MCU_MODE_SHIFT       (16)
#define SYSCTL_PIN_MCU_MODE_EN          (0x1 << SYSCTL_PIN_MCU_MODE_SHIFT)

#if (CONFIG_CHIP_ARCH_VER == 2)
#define SYSCTL_CSI_JPEG_SHARE_SRAM_SHIFT (8)
#define SYSCTL_CSI_JPEG_SHARE_SRAM_MASK  (0x3 << SYSCTL_CSI_JPEG_SHARE_SRAM_SHIFT)
typedef enum {
	SYSCTL_CSI_JPEG_SHARE_NONE       = (0 << SYSCTL_CSI_JPEG_SHARE_SRAM_SHIFT),
	SYSCTL_CSI_JPEG_SHARE_32K        = (1 << SYSCTL_CSI_JPEG_SHARE_SRAM_SHIFT),
	SYSCTL_CSI_JPEG_SHARE_64K        = (2 << SYSCTL_CSI_JPEG_SHARE_SRAM_SHIFT),
} SYSCTL_CSI_JPEG_ShareSramType;

#define SYSCTL_WLAN_SHARE_SRAM_SHIFT    (0)
#define SYSCTL_WLAN_SHARE_SRAM_MASK     (0x7 << SYSCTL_WLAN_SHARE_SRAM_SHIFT)
typedef enum {
	SYSCTL_WLAN_SHARE_NONE          = (0 << SYSCTL_WLAN_SHARE_SRAM_SHIFT),
	SYSCTL_WLAN_SHARE_32K           = (1 << SYSCTL_WLAN_SHARE_SRAM_SHIFT),
	SYSCTL_WLAN_SHARE_64K           = (2 << SYSCTL_WLAN_SHARE_SRAM_SHIFT),
	SYSCTL_WLAN_SHARE_96K           = (3 << SYSCTL_WLAN_SHARE_SRAM_SHIFT),
	SYSCTL_WLAN_SHARE_128K          = (4 << SYSCTL_WLAN_SHARE_SRAM_SHIFT),
} SYSCTL_WLAN_ShareSramType;
#endif

/* SYSCTL->PS_CTL_REG 0x50 */
#define SYSCTL_CLK250M_CNT_RDY_SHIFT    (16)
#define SYSCTL_CLK250M_CNT_RDY_MASK     (0x1 << SYSCTL_CLK250M_CNT_RDY_SHIFT)

#define SYSCTL_DEVICE_SEL_SHIFT         (8)
#define SYSCTL_DEVICE_SEL_MASK          (0x7 << SYSCTL_DEVICE_SEL_SHIFT)
typedef enum {
	SYSCTL_PSENSOR_0                = (0U << SYSCTL_DEVICE_SEL_SHIFT),
	SYSCTL_PSENSOR_1                = (1U << SYSCTL_DEVICE_SEL_SHIFT),
} SYSCTL_PsensorId;

#define SYSCTL_PS_EN_SEL_SHIFT          (4)
#define SYSCTL_PS_EN_SEL_MASK           (0x3 << SYSCTL_PS_EN_SEL_SHIFT)
typedef enum {
	SYSCTL_OSC_DISABLE              = (0U  << SYSCTL_PS_EN_SEL_SHIFT),
	SYSCTL_OSC_RVT_CASECODE         = (1U  << SYSCTL_PS_EN_SEL_SHIFT),
	SYSCTL_OSC_LVT_CASECODE         = (2U  << SYSCTL_PS_EN_SEL_SHIFT),
	SYSCTL_OSC_RVT_NORMAL           = (3U  << SYSCTL_PS_EN_SEL_SHIFT),
} SYSCTL_OSCSelect;

#define SYSCTL_PS_N_PRD_SHIFT           (1)
#define SYSCTL_PS_N_PRD_MASK            (0x7 << SYSCTL_PS_N_PRD_SHIFT)
#define SYSCTL_PS_N_PRD_VAL(v)          ((v & 0x7) << SYSCTL_PS_N_PRD_SHIFT)

#define SYSCTL_PS_EN_SHIFT              (0)
#define SYSCTL_PS_EN_MASK               (0x1 << SYSCTL_PS_EN_SHIFT)

#if (CONFIG_CHIP_ARCH_VER == 3)
/* SYSCTL->FLASH_PSRAM_PIN_MAP_REG 0x60 */
/* GPIOB08 ~ GPIOB11 */
#define SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT       (16)
#define SYSCTL_FLASH_PSRAM_PIN_MAP1_MASK        (0x3 << SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT)
typedef enum {
	SYSCTL_FLASH_PSRAM_PIN_MAP1_RESERVED    = (0U  << SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP1_ONLY_CS0    = (1U  << SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP1_ONLY_CS1    = (2U  << SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP1_BOTH_CS0_1  = (2U  << SYSCTL_FLASH_PSRAM_PIN_MAP1_SHIFT),
} SYSCTL_FlashPsramPinMap1;

/* GPIOB02 ~ GPIOB05 */
#define SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT       (0)
#define SYSCTL_FLASH_PSRAM_PIN_MAP0_MASK        (0x3 << SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT)
typedef enum {
	SYSCTL_FLASH_PSRAM_PIN_MAP0_RESERVED    = (0U  << SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP0_ONLY_CS0    = (1U  << SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP0_ONLY_CS1    = (2U  << SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT),
	SYSCTL_FLASH_PSRAM_PIN_MAP0_BOTH_CS0_1  = (2U  << SYSCTL_FLASH_PSRAM_PIN_MAP0_SHIFT),
} SYSCTL_FlashPsramPinMap0;
#endif

/* SYSCTL->SIP_TEST_MAP 0x44 */
#define SYSCTL_SIP_TEST_MAP_WRITE_SHIFT          (16)
#define SYSCTL_SIP_TEST_MAP_WRITE_EN             (0x429D << SYSCTL_SIP_TEST_MAP_WRITE_SHIFT)
#define SYSCTL_SIP_FLASH_TEST_MAP_SHIFT          (0)
#define SYSCTL_SIP_FLASH_TEST_MAP_EN             (0x1 << SYSCTL_SIP_FLASH_TEST_MAP_SHIFT)
#define SYSCTL_SIP_PSRAM_TEST_MAP_SHIFT          (1)
#define SYSCTL_SIP_PSRAM_TEST_MAP_EN             (0x1 << SYSCTL_SIP_PSRAM_TEST_MAP_SHIFT)

void HAL_SYSCTL_SetMcuMode(uint32_t en);
#if (CONFIG_CHIP_ARCH_VER == 2)
void HAL_SYSCTL_SetWlanSramShare(SYSCTL_WLAN_ShareSramType type);
void HAL_SYSCTL_SetCSIJPEGSramShare(SYSCTL_CSI_JPEG_ShareSramType mode);
#endif
void HAL_SYSCTL_SetPsensorControl(SYSCTL_PsensorId id, SYSCTL_OSCSelect osc_sel,
                                  uint32_t ps_n, uint32_t en);
void HAL_SYSCTL_WaitPsensorRdyAndClean(void);
uint32_t HAL_SYSCTL_GetPsensorCnt(void);
#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_SYSCTL_SelectFlashPsramPinMap(SYSCTL_FlashPsramPinMap0 cs0, SYSCTL_FlashPsramPinMap1 cs1);
#endif

void HAL_SYSCTL_SetDbgData(uint32_t id, uint32_t data);
uint32_t HAL_SYSCTL_GetDegData(uint32_t id);
uint32_t HAL_SYSCTL_GetPsensor(uint32_t id);

void HAL_SYSCTL_SetSipFlashTestMapMode(void);
void HAL_SYSCTL_SetSipPsramTestMapMode(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_SYSCTL_H_ */
