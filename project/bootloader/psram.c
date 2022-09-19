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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "sys/param.h"
#include "sys/io.h"
#include "sys/xr_debug.h"

#include "image/fdcm.h"
#include "image/image.h"
#include "driver/chip/hal_util.h"
#include "common/board/board.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"

#ifdef CONFIG_SECURE_BOOT
#include "../src/trustzone/include/crypto/sha256.h"
#include "../src/trustzone/include/crypto/ecc.h"
#endif

#ifdef CONFIG_FLASH_CRYPTO
#include "driver/chip/flash_crypto.h"
#endif

#if (defined(CONFIG_PSRAM) && defined(CONFIG_TRUSTZONE_BOOT))

#define LOAD_SIZE               (1 * 1024)

#define LOAD_BY_DBUS_CPU        0
#define LOAD_BY_DBUS_DMA        1
#define LOAD_BY_DBUS_FLASH_DMA  2

#define PSRAM_LOAD_METHOD       LOAD_BY_DBUS_CPU

#ifdef CONFIG_FLASH_CRYPTO
extern FLASH_CRYPTO_RANGE bl_flash_crypto_enable(uint32_t saddr, uint32_t eaddr);
extern void bl_flash_crypto_disable(FLASH_CRYPTO_RANGE ch);
#endif

extern uint32_t bl_sha256_burst(uint32_t faddr, uint32_t len, uint8_t *digest, uint8_t flash_psram);
extern uint32_t bl_verify_signature(uint8_t *input, uint32_t len, uint8_t *signature);
extern int uECC_verify(const uint8_t *p_public_key, const uint8_t *p_message_hash,
        unsigned int p_hash_size, const uint8_t *p_signature, uECC_Curve curve);

static int load_psram_bin_and_set_addr(struct psram_chip *chip)
{
	int ret = -1;
	uint32_t len = 0;
	section_header_t sh;
	uint8_t *buf;
	uint32_t i, num_blocks, size, img_len, r_len;
	uint8_t digest[TC_SHA256_DIGEST_SIZE], pkey[ECC_PKEY_SIZE], sign[ECC_SIGN_SIZE];

	/* Load psram header */
	if (image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_HEADER, 0, &sh, IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
		PSRAM_ERR("load section (id: %#08x) header failed\n", IMAGE_TZ_PSRAM_ID);
		return ret;
	}

	/* Check psram header */
	if (image_check_header(&sh) == IMAGE_INVALID) {
		PSRAM_ERR("check section (id: %#08x) header failed\n", IMAGE_TZ_PSRAM_ID);
		return ret;
	}

	/* Map psram physical addr */
	HAL_PsramCtrl_Set_Address_Field(NULL, FLASH_ADDR_TZ_PSRAM_IDX, sh.load_addr,
	                                sh.load_addr + PRJ_TZ_PSRAM_SIZE_BYTE,
	                                PRJ_TZ_PSRAM_START_PHY);

	if (sh.body_len == 0) { /* psram only used to store data */
		PSRAM_INF("psram only used store data\n");
		ret = 0;
		return ret;
	}

	/* Config load buffer */
#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_CPU)
	buf = malloc(LOAD_SIZE);
	if (buf == NULL) {
		PSRAM_ERR("Malloc buf fail!");
		return ret;
	}
	uint8_t *psram_vir_addr = (uint8_t *)sh.load_addr;
#else
	buf = (uint8_t *)sh.load_addr;
#endif

	img_len = sh.body_len + ECC_SIGN_SIZE;
	num_blocks = (img_len + LOAD_SIZE - 1) / LOAD_SIZE;
	PSRAM_INF("img_len=%d load_times:%d load addr:%#x\n", img_len, num_blocks, sh.load_addr);

	/* Load psram body code */
	for (i = 0; i < num_blocks; i++) {
		size = ((img_len - len) > LOAD_SIZE) ? LOAD_SIZE : (img_len - len);
		PSRAM_DBG("psram loading idx:%d, load addr[0x%x], load size:%d\n", i, sh.load_addr + len, size);

		len += image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_BODY, len, (void *)buf, size);
		//PSRAM_DUMP(buf, size);                /* dump psram for debug */

#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_CPU)
		memcpy(psram_vir_addr, buf, size);     /* don't support DMA, copy by CPU */
		psram_vir_addr += size;
#else
		buf += size;                        /* support DMA, copy by image */
#endif
	}

	/* Check data checksum */
	if (len != img_len) {
		PSRAM_ERR("psram body and signature size %u, read %u\n", img_len, len);
		goto out;
	}

	if (!(sh.attribute & IMAGE_ATTR_FLAG_ENC) &&
	    image_check_data(&sh, (void *)sh.load_addr, sh.body_len,
	                     (void *)(sh.load_addr + sh.body_len),
	                     ECC_SIGN_SIZE) == IMAGE_INVALID) {
		PSRAM_ERR("invalid psram bin body\n");
		goto out;
	}

	/* Verify signature */
#if defined(CONFIG_FLASH_CRYPTO)
	if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
		FLASH_CRYPTO_RANGE crypto_ch = FCRYPTO_RANGE_NUM;
		crypto_ch = bl_flash_crypto_enable(sh.load_addr, sh.load_addr + sh.body_len);
		if (bl_sha256_burst(sh.load_addr, sh.body_len, digest, 1)) {
			bl_flash_crypto_disable(crypto_ch);
			PSRAM_ERR("caclulate sha256 by burst failure !\n");
			ret = -2;
			goto out;
		}
		bl_flash_crypto_disable(crypto_ch);

		r_len = image_read(IMAGE_BOOT_ID, IMAGE_SEG_TAILER, 0, (void *)pkey, ECC_PKEY_SIZE);
		if (r_len != ECC_PKEY_SIZE) {
			PSRAM_WRN("ecc256 public key size %u, read %u\n", ECC_PKEY_SIZE, r_len);
			ret = -2;
			goto out;
		}
		r_len = image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_TAILER, 0, (void *)sign, ECC_SIGN_SIZE);
		if (r_len != ECC_SIGN_SIZE) {
			PSRAM_WRN("ecc256 signature size %u, read %u\n", ECC_SIGN_SIZE, r_len);
			ret = -2;
			goto out;
		}

		if (!uECC_verify(pkey, digest, TC_SHA256_DIGEST_SIZE, (uint8_t *)sign, &curve_secp256r1)) {
			memset((void *)sh.load_addr, 0, sh.body_len);
			PSRAM_ERR("verify encrypt tz_psram signature failure !\n");
			ret = -2;
			goto out;
		} else {
			PSRAM_INF("verify encrypt tz_psram signature Success.\n");
		}
	} else
#endif /* CONFIG_FLASH_CRYPTO */
	{
		if (bl_verify_signature((uint8_t *)sh.load_addr, sh.body_len,
		                        (uint8_t *)(sh.load_addr + sh.body_len))) {
			memset((void *)sh.load_addr, 0, sh.body_len);
			PSRAM_ERR("verify tz psram signature failure !\n");
			goto out;
		} else {
			PSRAM_INF("verify tz psram signature Success.\n");
		}
	}

	ret = 0;

out:
#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_CPU)
	free(buf);
#endif

	return ret;
}

__sram_data
static struct psram_chip chip = {0};

__sram_text
void load_psram_bin(void)
{
	uint32_t addr, p_type = PSRAM_CHIP_MAX;
	struct psram_ctrl *ctrl = NULL;
	PSRAMCtrl_InitParam cfg;
	PSRAMChip_InitParam psram_chip_cfg;

	addr = image_get_section_addr(IMAGE_TZ_PSRAM_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		PSRAM_INF("no psram section\n");
		return;
	}
	PSRAM_INF("get psram section ok @:0x%x\n", addr);

#if (defined CONFIG_PSRAM_CHIP_SQPI)
	p_type = PSRAM_CHIP_SQPI;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	p_type = PSRAM_CHIP_OPI_APS32;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	p_type = PSRAM_CHIP_OPI_APS64;
#endif

	cfg.freq = PSRAM_FREQ;
	cfg.p_type = p_type;
	if (cfg.p_type == PSRAM_CHIP_SQPI)
		cfg.rdata_w = 0; /* wait 0 half cycle on fpga */
	else
		cfg.rdata_w = 1; /* wait 1 half cycle on fpga */
	ctrl = HAL_PsramCtrl_Create(0, &cfg);
	if (!ctrl) {
		PSRAM_ERR("psram create faile\n");
		return;
	}

#ifndef CONFIG_XIP
	addr = image_get_section_addr(IMAGE_APP_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		return;
	}
	/* TODO: check section's validity */
	HAL_Xip_Init(PRJCONF_IMG_FLASH, addr + IMAGE_HEADER_SIZE);
#endif

	if (HAL_PsramCtrl_Init(ctrl)) {
		PSRAM_ERR("psram ctrl init faild!\n");
		goto out3;
	}

	ctrl = HAL_PsramCtrl_Open(0);
	if (!ctrl) {
		PSRAM_ERR("psram open faile\n");
		goto out2;
	}

	psram_chip_cfg.freq = PSRAM_FREQ;
	psram_chip_cfg.p_type = p_type;
	if (psram_init(&chip, ctrl, &psram_chip_cfg)) {
		PSRAM_ERR("psram chip init faild!\n");
		goto out1;
	}
	PSRAM_DBG("psram chip %s init ok!, freq %d\n", chip.name, cfg.freq);

	if (load_psram_bin_and_set_addr(&chip)) {
		PSRAM_ERR("load psram bin faild!\n");
	}
	PSRAM_DBG("load psram bin ok\n");

	psram_deinit(&chip);

out1:
	HAL_PsramCtrl_Close(ctrl);

out2:
	HAL_PsramCtrl_Deinit(ctrl);

out3:
	HAL_PsramCtrl_Destory(ctrl);

	HAL_Xip_Deinit(PRJCONF_IMG_FLASH);
}

#endif /* CONFIG_PSRAM && CONFIG_TRUSTZONE_BOOT */
