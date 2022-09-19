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

#ifndef __FLASH_CYRPTO_H
#define __FLASH_CYRPTO_H

#include "driver/chip/hal_def.h"

#define ROOT_AESKEY_BIT_POS     (256)
#define ROOT_AESKEY_BIT_END     (256+128-1)

typedef struct {
	__IO uint32_t ENC_ENABLE;    /* offset: 0x00, flash crypto enable register */
	__IO uint32_t RESERVED1;
	__IO uint32_t ENC_RANGE_ST;  /* offset: 0x08, flash crypto range start register */
	__IO uint32_t ENC_RANGE_ED;  /* offset: 0x0C, flash crypto range end register */
	__IO uint32_t ENC_KEY[4];    /* offset: 0x10, flash crypto key register */
} FLASH_CRYPTO_T_SINGLE;

typedef struct {
	FLASH_CRYPTO_T_SINGLE flash_crypto[8];
} FLASH_CRYPTO_T;

#define FLASH_CRYPTO ((FLASH_CRYPTO_T *)FLASH_ENCRYPT_CTRL_BASE) /* address: 0x40045000 */

#define FLASH_CRYPTO_SOFT_RESET_BIT     HAL_BIT(0)
#define FLASH_CRYPTO_ENABLE_BIT         HAL_BIT(0)
#define FLASH_CRYPTO_RANGE_MASK         (0xFFFFFFF0U)

/*
 * @brief FCRYPTO_RANGE_5 -> fixed used for tz_xip
 *        FCRYPTO_RANGE_4 -> fixed used for tz_psram
 *        FCRYPTO_RANGE_3 -> maybe for app_psram
 *        FCRYPTO_RANGE_2 -> maybe for app_xip
 *        FCRYPTO_RANGE_1 -> fixed used for image_read/bootloader
 *        FCRYPTO_RANGE_0 -> for user data(flash_read_crypto/flash_write_crypto, etc)
 */
typedef enum {
	FCRYPTO_RANGE_0 = 0,
	FCRYPTO_RANGE_1,
	FCRYPTO_RANGE_2,
	FCRYPTO_RANGE_3,
	FCRYPTO_RANGE_4,
	FCRYPTO_RANGE_5,
	FCRYPTO_RANGE_NUM
} FLASH_CRYPTO_RANGE;

void RootAESKey_Get(uint8_t *key);
void FCRYPTO_GetRange(FLASH_CRYPTO_RANGE range, uint32_t *startAddr, uint32_t *endAddr);

void flash_crypto_init(void);
void flash_crypto_deinit(void);

int32_t flash_crypto_enable(uint32_t startAddr, uint32_t endAddr, uint8_t *key);
void flash_crypto_disable(uint32_t channel);

int32_t flash_bin_decrypt_enable(FLASH_CRYPTO_RANGE range_num,
                                 uint32_t startAddr, uint32_t endAddr);
void flash_bin_decrypt_disable(uint32_t channel);

void flash_crypto_suspend(uint32_t *back_regs, uint32_t back_size);
void flash_crypto_resume(uint32_t *back_regs, uint32_t back_size);

#endif /* __FLASH_CYRPTO_H */
