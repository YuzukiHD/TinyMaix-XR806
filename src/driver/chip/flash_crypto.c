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

#ifndef CONFIG_TRUSTZONE
#include <stdio.h>
#include "driver/chip/flash_crypto.h"
#include "driver/chip/hal_efuse.h"
#include "driver/chip/hal_global.h"
#include "driver/chip/hal_ccm.h"
#include "image/image.h"
#include "sys/param.h"

#define FCR_SYSLOG    printf

#define FCR_LOG(flags, fmt, arg...)                    \
	do {                                               \
		if (flags) {                                   \
			__sram_rodata static char __fmt[] = fmt;   \
			FCR_SYSLOG(__fmt, ##arg);                  \
		}                                              \
	} while (0)

/* flash crypto debug */
#define FCRYPTO_DBG_ON        0
#define FCRYPTO_ERR_ON        1
#define FCRYPTO_WRN_ON        1

#define FCR_NX_DBG(fmt, arg...) FCR_LOG(FCRYPTO_DBG_ON, "[FCR D] "fmt"\n", ##arg)
#define FCR_NX_ERR(fmt, arg...) FCR_LOG(FCRYPTO_ERR_ON, "[FCR E] "fmt"\n", ##arg)
#define FCR_NX_WRN(fmt, arg...) FCR_LOG(FCRYPTO_WRN_ON, "[FCR W] "fmt"\n", ##arg)

__sram_text
void RootAESKey_Get(uint8_t *key)
{
	uint32_t i = 0, word_pos = ROOT_AESKEY_BIT_POS >> 5;

	for (i = 0; i < 4; i++) {
		*((uint32_t *)key + i) = EFUSE->VALUE[word_pos + i];
	}
}

__sram_text
static __inline void FCRYPTO_Enable(FLASH_CRYPTO_RANGE ch)
{
	HAL_SET_BIT(FLASH_CRYPTO->flash_crypto[ch].ENC_ENABLE, FLASH_CRYPTO_ENABLE_BIT);
}

static __inline void FCRYPTO_Disable(FLASH_CRYPTO_RANGE ch)
{
	HAL_CLR_BIT(FLASH_CRYPTO->flash_crypto[ch].ENC_ENABLE, FLASH_CRYPTO_ENABLE_BIT);
}

static __inline uint32_t FCRYPTO_GetStatus(FLASH_CRYPTO_RANGE ch)
{
	return HAL_GET_BIT(FLASH_CRYPTO->flash_crypto[ch].ENC_ENABLE,
	                   FLASH_CRYPTO_ENABLE_BIT);
}

__sram_text
static __inline void FCRYPTO_SetRange(FLASH_CRYPTO_RANGE ch, uint32_t startAddr,
                                      uint32_t endAddr)
{
	FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ST = startAddr &
	                                              FLASH_CRYPTO_RANGE_MASK;
	FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ED = endAddr & FLASH_CRYPTO_RANGE_MASK;
}

void FCRYPTO_GetRange(FLASH_CRYPTO_RANGE ch, uint32_t *startAddr,
                      uint32_t *endAddr)
{
	*startAddr = FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ST;
	*endAddr = FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ED;
}

static __inline void FCRYPTO_ClrRange(FLASH_CRYPTO_RANGE ch)
{
	FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ST = 0UL;
	FLASH_CRYPTO->flash_crypto[ch].ENC_RANGE_ED = 0UL;
}

__sram_text
static void FCRYPTO_SetKey(FLASH_CRYPTO_RANGE ch, uint32_t *key)
{
	uint8_t i;

	for (i = 0; i < 4; i++) {
		FLASH_CRYPTO->flash_crypto[ch].ENC_KEY[i] = *(key + i);
	}
}

static void FCRYPTO_ClrKey(FLASH_CRYPTO_RANGE ch)
{
	uint8_t i;

	for (i = 0; i < 4; i++) {
		FLASH_CRYPTO->flash_crypto[ch].ENC_KEY[i] = 0UL;
	}
}

__sram_text
void flash_crypto_init(void)
{
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_ENC);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_FLASH_ENC);
}

void flash_crypto_deinit(void)
{
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_FLASH_ENC);
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_ENC);
}

/**
 * @brief enable ahb-bus crypto function, so the data to flash will be encrypt,
 *        and data from flash will be decrypt, use aes-128 count mode,
 *        channel_1~5 use for system dedicated, the channel_0 reserved for user data
 * @param[in] startAddr the flash start address
 * @param[in] endAddr the flash end address
 * @param[in] key the 128 bit key for aes-128 count mode
 * @return channel num, -1 on failure
 * @note: it is necessary to ensure that the startAddr and endAddr are 16-Byte aligned.
 */
int32_t flash_crypto_enable(uint32_t startAddr, uint32_t endAddr, uint8_t *key)
{
	uint32_t channel;

	if ((startAddr >= endAddr) || (key == NULL)) {
		return -1;
	}

	/* fix channel 0 for user flash data crypto temporary */
	for (channel = 0; channel < 1; channel++) {
		if (!FCRYPTO_GetStatus(channel)) {
			break;
		}
	}

	if (channel == 1) {
		FCR_NX_ERR("NO free crypto channel to use!\n");
		return -1;
	}

	FCRYPTO_SetRange(channel, startAddr, endAddr);
	FCRYPTO_SetKey(channel, (uint32_t *)key);
	FCRYPTO_Enable(channel);

	FCR_NX_DBG("Flash Crypto channel-[%u] address-[0x%x, 0x%x] config Success\n",
	           channel, startAddr, endAddr);
	return channel;
}

/**
 * @brief disable ahb-bus crypto function, so read/write flash to be direct
 * @param[in] channel the flash crypto channel
 * @return null
 */
void flash_crypto_disable(uint32_t channel)
{
	if (HAL_GlobalGetChipVer() == 0x0A) {
		FCR_NX_ERR("not support flash crypto disable\n");
		return;
	}

	if (channel < 0 || channel >= FCRYPTO_RANGE_NUM) {
		FCR_NX_ERR("Invalid channel:%d\n", channel);
		return;
	}

	FCRYPTO_ClrRange(channel);
	FCRYPTO_ClrKey(channel);
	FCRYPTO_Disable(channel);
}

/**
 * @brief for image bin decryption.
 *        such as XIP function, CBUS fetch instruction and data from flash, will be decrypt.
 *        channel_5 use for tz XIP bin,    channel_4 use for tz PSRAM bin.
 *        channel_3 use for app PSRAM bin, channel_2 use for app XIP bin.
 *        channel_1 use for other app bin, channel_0 use for user flash data.
 * @param[in] startAddr the start address for decrypt
 * @param[in] endAddr the end address for decrypt
 * @param[in] key the key for decrypt
 * @retval: non negative: channel number; negative number: failure
 */
__sram_text
int32_t flash_bin_decrypt_enable(FLASH_CRYPTO_RANGE ch, uint32_t startAddr,
                                 uint32_t endAddr)
{
	uint8_t aes_key[16];

	if (startAddr >= endAddr) {
		return -1;
	}

	RootAESKey_Get(aes_key);

	FCRYPTO_SetRange(ch, startAddr, roundup2(endAddr, FCR_AES_ALIGN_SIZE));
	FCRYPTO_SetKey(ch, (uint32_t *)aes_key);
	FCRYPTO_Enable(ch);

	return ch;
}

void flash_bin_decrypt_disable(uint32_t channel)
{
	if (channel < 0 || channel >= FCRYPTO_RANGE_NUM) {
		FCR_NX_ERR("Invalid channel:%d\n", channel);
		return;
	}

	FCRYPTO_ClrRange(channel);
	FCRYPTO_ClrKey(channel);
	FCRYPTO_Disable(channel);
}

void flash_crypto_suspend(uint32_t *back_regs, uint32_t back_size)
{
	uint32_t i;

	for (i = 0; i < back_size / 4; i++) {
		back_regs[i] = HAL_REG_32BIT(FLASH_ENCRYPT_CTRL_BASE + i * 4);
	}
}

void flash_crypto_resume(uint32_t *back_regs, uint32_t back_size)
{
	uint32_t i;
	flash_crypto_init();

	/* APB Devices Reg can't write byte by byte */
	for (i = 0; i < back_size / 4; i++) {
		HAL_REG_32BIT(FLASH_ENCRYPT_CTRL_BASE + i * 4) = back_regs[i];
	}
}
#endif /* CONFIG_TRUSTZONE */
