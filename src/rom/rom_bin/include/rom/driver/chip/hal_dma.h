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

#ifndef _ROM_DRIVER_CHIP_HAL_DMA_H_
#define _ROM_DRIVER_CHIP_HAL_DMA_H_

#include "driver/chip/hal_dma.h"
#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ROM

#define HAL_DMA_Init \
	RAM_TBL_FUN(HAL_Status *(*)(DMA_Channel chan, const DMA_ChannelInitParam *param), HAL_DMA_Init)

#define HAL_DMA_Start \
	RAM_TBL_FUN(HAL_Status *(*)(DMA_Channel chan, uint32_t srcAddr, uint32_t dstAddr, uint32_t datalen), HAL_DMA_Start)

#define HAL_DMA_Stop \
	RAM_TBL_FUN(HAL_Status (*)(DMA_Channel chan), HAL_DMA_Stop)

#define HAL_DMA_FlashSbus_Iscoexist \
    RAM_TBL_FUN(uint32_t (*)(void), HAL_DMA_FlashSbus_Iscoexist)

#else /* CONFIG_ROM */

#define rom_HAL_DMA_Init  HAL_DMA_Init
#define rom_HAL_DMA_Start HAL_DMA_Start
#define rom_HAL_DMA_Stop  HAL_DMA_Stop

#endif /* CONFIG_ROM */

#ifdef __cplusplus
}
#endif

#endif /* _ROM_DRIVER_CHIP_HAL_DMA_H_ */
