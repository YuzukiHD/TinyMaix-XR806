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

#ifndef _ROM_DRIVER_CHIP_HAL_FLASHCTRL_H_
#define _ROM_DRIVER_CHIP_HAL_FLASHCTRL_H_

#include "driver/chip/hal_flashctrl.h"
#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ROM

#define HAL_Flashc_Delay \
	RAM_TBL_FUN(void (*)(struct flash_controller *ctrl, unsigned int us), HAL_Flashc_Delay)

#define HAL_Flashc_Xip_Init \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl, XIP_Config *cfg), HAL_Flashc_Xip_Init)

#define HAL_Flashc_Xip_Deinit \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl), HAL_Flashc_Xip_Deinit)

#define HAL_Flashc_Init \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl, const Flashc_Config *cfg), HAL_Flashc_Init)

#define HAL_Flashc_Deinit \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl), HAL_Flashc_Deinit)

#define HAL_Flashc_Open \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl), HAL_Flashc_Open)

#define HAL_Flashc_Close \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl), HAL_Flashc_Close)

#define HAL_Flashc_Ioctl \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl, uint32_t op, void *arg), HAL_Flashc_Ioctl)

#define HAL_Flashc_Transfer \
	RAM_TBL_FUN(HAL_Status (*)(struct flash_controller *ctrl, int write, \
	                           FC_InstructionField *cmd, FC_InstructionField *addr, \
	                           FC_InstructionField *dummy, FC_InstructionField *data, bool dma), \
	            HAL_Flashc_Transfer)

#define FC_GetDelayCycle \
	RAM_TBL_FUN(void (*)(Flash_Ctrl_DelayCycle *delay, uint32_t freq), FC_GetDelayCycle)

#define FlashCBUS_AddrRequire \
	RAM_TBL_FUN(int32_t (*)(uint8_t field, uint32_t sAddr, uint32_t bias, uint32_t userLen), FlashCBUS_AddrRequire)
#define FlashCryptoRequest \
	RAM_TBL_FUN(int32_t (*)(uint32_t startAddr, uint32_t endAddr, uint8_t *key), FlashCryptoRequest)

#else /* CONFIG_ROM */

#define rom_HAL_Flashc_Delay        HAL_Flashc_Delay
#define rom_HAL_Flashc_Xip_Init     HAL_Flashc_Xip_Init
#define rom_HAL_Flashc_Xip_Deinit   HAL_Flashc_Xip_Deinit
#define rom_HAL_Flashc_Init         HAL_Flashc_Init
#define rom_HAL_Flashc_Deinit       HAL_Flashc_Deinit
#define rom_HAL_Flashc_Open         HAL_Flashc_Open
#define rom_HAL_Flashc_Close        HAL_Flashc_Close
#define rom_HAL_Flashc_Ioctl        HAL_Flashc_Ioctl
#define rom_HAL_Flashc_Transfer     HAL_Flashc_Transfer
#endif /* CONFIG_ROM */

#ifdef __cplusplus
}
#endif

#endif /*_ROM_DRIVER_CHIP_HAL_FLASHCTRL_H_*/
