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

#include "image/flash.h"
#include "image_debug.h"
#include "driver/chip/hal_global.h"

#if (CONFIG_CHIP_ARCH_VER == 3)
uint32_t flash_read_crypto(uint32_t flash, uint32_t addr,
                           void *buf, uint32_t size, uint8_t *key)
{
	uint32_t ret = 0;

	if (HAL_GlobalGetChipVer() == 0x0A) {
		FLASH_ERR("do not support flash read crypto\n");
		return -1;
	}

	FC_Crypto_Enable(key);
	ret = flash_rw(flash, addr, buf, size, 0);
	FC_Clr_Crypto_Infor();

	FlashCryptoRelease(0);

	return ret;
}

uint32_t flash_write_crypto(uint32_t flash, uint32_t addr,
                            const void *buf, uint32_t size, uint8_t *key)
{
	uint32_t ret = 0;

	if (HAL_GlobalGetChipVer() == 0x0A) {
		FLASH_ERR("do not support flash write crypto\n");
		return -1;
	}

	FC_Crypto_Enable(key);
	ret = flash_rw(flash, addr, (void *)buf, size, 1);
	FC_Clr_Crypto_Infor();

	FlashCryptoRelease(0);

	return ret;
}
#endif
