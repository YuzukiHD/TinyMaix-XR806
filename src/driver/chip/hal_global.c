/**
 * @file  hal_global.c
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

#include "hal_base.h"

void HAL_GlobalInit(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_DEFAULT);
#if (CONFIG_CHIP_ARCH_VER == 1)
	HAL_CCM_BusForceAllPeriphReset();
	HAL_CCM_BusDisableAllPeriphClock();
#elif (CONFIG_CHIP_ARCH_VER > 1)
	HAL_CCM_BusDisablePeriphClock(~CCM->BUS_PERIPH_RST_CTRL);
#endif
}

#define GET_UNSTUFF_REG_BITS(resp, start, size)                 \
	({                                                          \
		const int32_t __size = size;                            \
		const uint32_t __mask = (__size < 32 ? 1 << __size : 0) - 1;    \
		const int32_t __off = ((start) / 32);                   \
		const int32_t __shft = (start) & 31;                    \
		uint32_t __res;                                         \
		                                                        \
		__res = resp[__off] >> __shft;                          \
		if (__size + __shft > 32)                               \
			__res |= resp[__off + 1] << ((32 - __shft) % 32);   \
		__res & __mask;                                         \
	})
#define GlobalGetBitValue(start_bit, bit_cnt) GET_UNSTUFF_REG_BITS(((uint32_t *)0x40043D00), start_bit, bit_cnt)

uint32_t HAL_GlobalGetChipVer(void)
{
#if (CONFIG_CHIP_ARCH_VER == 1)
	static uint8_t g_chip_version;

	uint32_t start_bit;

	if (g_chip_version == 0) {
		start_bit = GlobalGetBitValue(608, 2);
		start_bit = (start_bit == 0 ? 200 : 610) + 22;
		g_chip_version = GlobalGetBitValue(start_bit, 6);
	}

	return g_chip_version;
#elif (CONFIG_CHIP_ARCH_VER == 2)
	return GlobalGetBitValue(136, 6);
#elif (CONFIG_CHIP_ARCH_VER == 3)
	return GlobalGetBitValue(150, 6);
#endif
}

uint32_t HAL_GlobalGetChipArch(void)
{
#if (CONFIG_CHIP_ARCH_VER == 1)
	uint32_t start_bit;

	start_bit = GlobalGetBitValue(608, 2);
	start_bit = (start_bit == 0 ? 200 : 610) + 28;
	return GlobalGetBitValue(start_bit, 24);
#elif (CONFIG_CHIP_ARCH_VER == 2)
	return GlobalGetBitValue(142, 24);
#elif (CONFIG_CHIP_ARCH_VER == 3)
	return GlobalGetBitValue(156, 24);
#endif
}

uint32_t HAL_GlobalGetChipType(void)
{
#if (CONFIG_CHIP_ARCH_VER == 1)
	uint32_t start_bit;

	start_bit = GlobalGetBitValue(608, 2);
	start_bit = (start_bit == 0 ? 200 : 610) + 10;
	return GlobalGetBitValue(start_bit, 6);
#elif (CONFIG_CHIP_ARCH_VER == 2)
	return GlobalGetBitValue(124, 6);
#elif (CONFIG_CHIP_ARCH_VER == 3)
	return GlobalGetBitValue(138, 6);
#endif
}

uint8_t HAL_GlobalGetSmpsBgtr(void)
{
	uint8_t val = GlobalGetBitValue(0, 4);

	if (val == 0) {
		val = GlobalGetBitValue(72, 5);
		val = (val << 1) & 0x1f;
		return val;
	}
	return 0xff;
}

uint8_t HAL_GlobalGetTopLdoVsel(void)
{
	return GlobalGetBitValue(68, 4);
}

uint8_t HAL_GlobalGetConnectMode(void)
{
	return GlobalGetBitValue(123, 2);
}

#if (CONFIG_CHIP_ARCH_VER == 2)
uint8_t HAL_GlobalGetDigLdoVsel(void)
{
	return GlobalGetBitValue(166, 4);
}
#endif

uint32_t HAL_GlobalGetInternalFlag(uint32_t bit)
{
	return GlobalGetBitValue(bit, 1);
}

