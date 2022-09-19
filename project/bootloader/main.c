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

#include <stdint.h>
#ifdef CONFIG_BIN_COMPRESS
#include <stdlib.h>
#include "xz/xz.h"
#endif

#include "driver/chip/system_chip.h"
#include "driver/chip/hal_chip.h"
#include "driver/chip/hal_util.h"
#include "image/image.h"
#include "kernel/os/os_time.h"
#include "image/flash.h"
#include "ota/ota.h"
#include "ota/ota_opt.h"
#include "sys/io.h"

#include "common/board/board.h"
#include "bl_debug.h"

#if defined(CONFIG_SECURE_BOOT)
#include "../src/trustzone/include/crypto/sha256.h"
#include "../src/trustzone/include/crypto/ecc.h"
#endif

#if defined(CONFIG_FLASH_CRYPTO)
#include "driver/chip/flash_crypto.h"
#endif

#define BL_SHOW_INFO    0    /* for internal debug only */

#define BL_INVALID_APP_ENTRY    0xFFFFFFFFU
#define BL_INVALID_APP_UNZIP    0xFFFFFFFEU

/* return values for bl_load_bin_by_id() */
#define BL_LOAD_BIN_OK      (0)     /* success */
#define BL_LOAD_BIN_NO_SEC  (-1)    /* no section */
#define BL_LOAD_BIN_INVALID (-2)    /* invalid section */
#define BL_LOAD_BIN_TOOBIG  (-3)    /* section too big */

#ifdef CONFIG_SECURE_BOOT
#define BL_SB_VERIFY_FAIL_IF_NO_BIN        0

#define BL_SB_TEST_LOAD_BOOT_BIN           0
#define BL_SB_TEST_FAKE_EFUSE_PUBKEY_HASH  0

#if BL_SB_TEST_LOAD_BOOT_BIN
#define BL_TEST_BOOT_BIN_LOAD_ADDR    0x00218000
#endif

#if BL_SB_TEST_FAKE_EFUSE_PUBKEY_HASH
static const uint8_t efuse_pubkey_hash[32] = {
	0x38, 0x39, 0x0f, 0xae, 0xec, 0xed, 0xa2, 0xe8,
	0x68, 0x56, 0x80, 0xbb, 0x57, 0x90, 0x71, 0xe7,
	0xa2, 0xd2, 0xb3, 0xd7, 0x35, 0x76, 0x88, 0xf9,
	0x03, 0x8f, 0x6b, 0xa2, 0x3e, 0x78, 0x90, 0xef
};
#endif

#endif /* CONFIG_SECURE_BOOT */

static __inline void bl_upgrade(void)
{
//	HAL_PRCM_SetCPUABootFlag(PRCM_CPUA_BOOT_FROM_COLD_RESET);
	HAL_PRCM_SetCPUABootFlag(PRCM_CPUA_BOOT_FROM_SYS_UPDATE);
	HAL_WDG_Reboot();
}

static __inline void bl_flash_init(void)
{
	int dev;
	FlashBoardCfg *cfg;

#if (CONFIG_CHIP_ARCH_VER == 3)
	HAL_SYSCTL_SelectFlashPsramPinMap(BOARD_FLASH_PSRAM_PIN_MAP0, BOARD_FLASH_PSRAM_PIN_MAP1);
#endif

	HAL_Flash_SetDbgMask(0);
	dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, PRJCONF_IMG_FLASH);
	HAL_BoardIoctl(HAL_BIR_GET_CFG, dev, (uint32_t)&cfg);
	if (cfg == NULL) {
		BL_ERR("getFlashBoardCfg failed");
		return;
	}

	if (cfg->type == FLASH_DRV_FLASHC) {
		cfg->flashc.param.freq = cfg->clk;
#if (CONFIG_CHIP_ARCH_VER == 3)
		if (HAL_PRCM_IsFlashSip()) {
			cfg->flashc.param.cs_mode = FLASHC_FLASHCS1_PSRAMCS0;
		} else {
			cfg->flashc.param.cs_mode = FLASHC_FLASHCS0_PSRAMCS1;
		}
#endif
	}
	if (HAL_Flash_Init(PRJCONF_IMG_FLASH, cfg) != HAL_OK) {
		BL_ERR("flash init fail\n");
	}
}

static __inline void bl_flash_deinit(void)
{
	HAL_Flash_Deinit(PRJCONF_IMG_FLASH);
}

static __inline void bl_hw_init(void)
{
#ifdef CONFIG_FLASH_CRYPTO
	uint8_t nonce[6] = {0x50, 0x00, 0x06, 0x20, 0x00, 0x00};
	flash_crypto_init();
	HAL_PRCM_SetFlashCryptoNonce(nonce);
#endif
}

static __inline void bl_hw_deinit(void)
{
#if PRJCONF_UART_EN
#if BL_DBG_ON
	while (!HAL_UART_IsTxEmpty(HAL_UART_GetInstance(BOARD_MAIN_UART_ID))) {
		;
	}
#endif
	board_uart_deinit(BOARD_MAIN_UART_ID);
#endif
	SystemDeInit(0);
}

#ifdef CONFIG_FLASH_CRYPTO
FLASH_CRYPTO_RANGE bl_flash_crypto_enable(uint32_t saddr, uint32_t eaddr)
{
	FLASH_CRYPTO_RANGE ch = FCRYPTO_RANGE_1;
	flash_bin_decrypt_enable(ch, saddr, eaddr);
	return ch;
}

void bl_flash_crypto_disable(FLASH_CRYPTO_RANGE ch)
{
	flash_bin_decrypt_disable(ch);
}
#endif /* CONFIG_FLASH_CRYPTO */

#ifdef CONFIG_BIN_COMPRESS

#define BL_DEC_BIN_INBUF_SIZE (4 * 1024)
#define BL_DEC_BIN_DICT_MAX   (32 * 1024)

static int bl_decompress_bin(const section_header_t *sh, uint32_t max_size)
{
	uint8_t *in_buf;
	uint32_t read_size, len, id, offset, left;
	uint16_t chksum;
	struct xz_dec *s;
	struct xz_buf b;
	enum xz_ret xzret;
	int ret = -1;
#if BL_DBG_ON
	OS_Time_t tm;
#endif

#if BL_DBG_ON
	BL_DBG("%s() start\n", __func__);
	tm = OS_GetTicks();
#endif

	in_buf = malloc(BL_DEC_BIN_INBUF_SIZE);
	if (in_buf == NULL) {
		BL_ERR("no mem\n");
		return ret;
	}

	/*
	 * Support up to BL_DEC_BIN_DICT_MAX KiB dictionary. The actually
	 * needed memory is allocated once the headers have been parsed.
	 */
	s = xz_dec_init(XZ_DYNALLOC, BL_DEC_BIN_DICT_MAX);
	if (s == NULL) {
		BL_ERR("no mem\n");
		goto out;
	}

	b.in = in_buf;
	b.in_pos = 0;
	b.in_size = 0;
	b.out = (uint8_t *)sh->load_addr;
	b.out_pos = 0;
	b.out_size = max_size;

	id = sh->id;
	offset = 0;
	left = sh->body_len;
	chksum = sh->data_chksum;

	while (1) {
		if (b.in_pos == b.in_size) {
			if (left == 0) {
				BL_ERR("no more input data\n");
				break;
			}
			read_size = left > BL_DEC_BIN_INBUF_SIZE ?
			            BL_DEC_BIN_INBUF_SIZE : left;
			len = image_read(id, IMAGE_SEG_BODY, offset, in_buf, read_size);
			if (len != read_size) {
				BL_ERR("read img body fail, id %#x, off %u, len %u != %u\n",
				         id, offset, len, read_size);
				break;
			}
			chksum += image_get_checksum(in_buf, len);
			offset += len;
			left -= len;
			b.in_size = len;
			b.in_pos = 0;
		}

		xzret = xz_dec_run(s, &b);

		if (b.out_pos == b.out_size) {
			BL_ERR("decompress size >= %u\n", b.out_size);
			break;
		}

		if (xzret == XZ_OK) {
			continue;
		} else if (xzret == XZ_STREAM_END) {
#if BL_DBG_ON
			tm = OS_GetTicks() - tm;
			BL_DBG("%s() end, size %u --> %u, cost %u ms\n", __func__,
			       sh->body_len, b.out_pos, tm);
#endif
			if (chksum != 0xFFFF) {
				BL_ERR("invalid checksum %#x\n", chksum);
			} else {
				ret = 0;
			}
			break;
		} else {
			BL_ERR("xz_dec_run() fail %d\n", xzret);
			break;
		}
	}

out:
	xz_dec_end(s);
	free(in_buf);
	return ret;
}

#endif /* CONFIG_BIN_COMPRESS */

#define BL_UPDATE_DEBUG_SIZE_UNIT (50 * 1024)

#define BL_DEC_IMG_INBUF_SIZE  (4 * 1024)
#define BL_DEC_IMG_OUTBUF_SIZE (4 * 1024)
#define BL_DEC_IMG_DICT_MAX    (8 * 1024)

static uint8_t bl_dec_inbuf[BL_DEC_IMG_INBUF_SIZE] = {0};
static uint8_t bl_dec_outbuf[BL_DEC_IMG_OUTBUF_SIZE] = {0};

static int bl_xz_image(image_seq_t seq)
{
	int ret = -1;
	struct xz_dec *s = NULL;
	struct xz_buf b;
	enum xz_ret xzret;
	uint8_t *in_buf = bl_dec_inbuf, *out_buf = bl_dec_outbuf;
	uint32_t left, read_size, offset;
	uint32_t ota_addr;
	uint32_t ota_xz_addr;
	uint32_t image_addr;
	const image_ota_param_t *iop;
	section_header_t xz_sh;
	section_header_t boot_sh;
	uint32_t len;
	uint32_t write_pos;
	uint32_t maxsize;
	uint32_t debug_size = BL_UPDATE_DEBUG_SIZE_UNIT;
/*
	uint32_t *verify_value;
	ota_verify_t verify_type;
	ota_verify_data_t verify_data;
*/
	image_cfg_t cfg;
#if BL_DBG_ON
	OS_Time_t tm;
#endif

#if BL_DBG_ON
	BL_DBG("%s() start\n", __func__);
	tm = OS_GetTicks();
#endif

	iop = image_get_ota_param();
	maxsize = IMAGE_AREA_SIZE(iop->img_max_size);
	BL_DBG("%s, data maxsize size = 0x%x\n", __func__, maxsize);

	/* get the compressed image address */
	ota_addr = iop->ota_addr + iop->ota_size;
	ota_xz_addr = iop->ota_addr + iop->ota_size + IMAGE_HEADER_SIZE;
	BL_DBG("iop addr = 0x%x, ota addr = 0x%x, ota xz addr = 0x%x, ota info size = 0x%x\n",
	       ota_addr, iop->ota_addr, ota_xz_addr, iop->ota_size);
	len = flash_read(iop->flash[seq], ota_addr, &xz_sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_ERR("%s, image read failed!\n", __func__);
		return ret;
	}
	BL_DBG("%s, ota load size = 0x%x, ota attribute = 0x%x\n", __func__,
	       xz_sh.body_len, xz_sh.attribute);

	if (!(xz_sh.attribute & IMAGE_ATTR_FLAG_COMPRESS)) {
		BL_ERR("the ota image is not a compress image!\n");
		ret = -2;
		goto out;
	}

	BL_DBG("xz file begin...\n");
	s = xz_dec_init(XZ_PREALLOC, BL_DEC_IMG_DICT_MAX);
	if (s == NULL) {
		BL_ERR("xz_dec_init malloc failed\n");
		goto out;
	}

	/* get boot section header */
	image_set_running_seq(0);
	len = image_read(IMAGE_BOOT_ID, IMAGE_SEG_HEADER, 0, &boot_sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_ERR("bin header size %u, read %u\n", IMAGE_HEADER_SIZE, len);
		goto out;
	}

	/* Erase the area from behind the BootLoader to IMG_MAX_SIZE */
	image_addr = iop->addr[0];
	BL_DBG("erase image start, flash:%d addr:0x%x size:%d\n", iop->flash[seq],
	                        image_addr, IMAGE_AREA_SIZE(iop->img_max_size));
	if (flash_erase(iop->flash[seq], image_addr, IMAGE_AREA_SIZE(iop->img_max_size)) == -1) {
		BL_ERR("%s, image erase err\n", __func__);
		goto out;
	}
	BL_DBG("erase image end...\n");

	write_pos = 0;
	offset = 0;
	left = xz_sh.body_len;

	b.in = in_buf;
	b.in_pos = 0;
	b.in_size = 0;
	b.out = out_buf;
	b.out_pos = 0;
	b.out_size = BL_DEC_IMG_OUTBUF_SIZE;

	while (1) {
		if (b.in_pos == b.in_size) {
			if (left == 0) {
				BL_DBG("no more input data\n");
				break;
			}

			read_size = left > BL_DEC_IMG_INBUF_SIZE ? BL_DEC_IMG_INBUF_SIZE : left;
			len = flash_read(iop->flash[seq], ota_xz_addr + offset, in_buf, read_size);

			if (len == 0) {
				BL_ERR("flash read err\n");
				break;
			}

			offset += len;
			left -= len;
			b.in_size = len;
			b.in_pos = 0;
		}

		xzret = xz_dec_run(s, &b);
		if (xzret == XZ_OK) {

			len = flash_write(iop->flash[seq], image_addr + write_pos, out_buf, b.out_pos);
			if (len != b.out_pos) {
				BL_ERR("flash write err len:%d out_pos:%d\n", len, b.out_pos);
				break;
			}

			if (write_pos >= debug_size) {
				BL_DBG("decompress data:%dK\n", write_pos / 1024);
				debug_size += BL_UPDATE_DEBUG_SIZE_UNIT;
			}

			write_pos += b.out_pos;
			b.out_pos = 0;
			continue;
		} else if (xzret == XZ_STREAM_END) {
			len = flash_write(iop->flash[seq], image_addr + write_pos, out_buf, b.out_pos);
			if (len != b.out_pos) {
				BL_ERR("flash write err len:%d out_pos:%d\n", len, b.out_pos);
				break;
			}
			write_pos += b.out_pos;
#if BL_DBG_ON
			tm = OS_GetTicks() - tm;
			BL_DBG("%s() end, size %u --> %u, cost %u ms\n", __func__,
			       xz_sh.body_len, b.out_pos, tm);
#endif
			break;
		} else {
			BL_ERR("xz stream failed %d\n", xzret);
			break;
		}
	}

	/* check every section */
	if (image_check_ota_sections(seq) == IMAGE_INVALID) {
		BL_ERR("ota check image failed\n");
		goto out;
	}
/*
	if (ota_get_verify_data(&verify_data) != OTA_STATUS_OK) {
		verify_type = OTA_VERIFY_NONE;
		verify_value = NULL;
	} else {
		verify_type = verify_data.ov_type;
		verify_value = (uint32_t*)(verify_data.ov_data);
	}

	if (ota_verify_image(verify_type, verify_value)  != OTA_STATUS_OK) {
		BL_ERR("ota file verify image failed\n");
		goto out;
	}
*/
	cfg.seq = 0;
	cfg.state = IMAGE_STATE_VERIFIED;
	if (image_set_cfg(&cfg) != 0)
		goto out;

	ret = 0;

out:
	if (s)
		xz_dec_end(s);

	BL_DBG("xz file end...\n");
	return ret;
}

static int bl_uncompress_image(image_seq_t *c_seq, image_seq_t *l_seq)
{
	/* init image */
	if (image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR) != 0) {
		BL_LOG(1, "img init fail\n");
		return BL_INVALID_APP_ENTRY;
	}

	const image_ota_param_t *iop = image_get_ota_param();
	if (iop->ota_addr == IMAGE_INVALID_ADDR) {
		image_set_running_seq(0);
		return 0; /* ota is disable */
	}

	/* ota is enabled */
	image_cfg_t cfg;
	image_seq_t cfg_seq, load_seq;

	if (image_get_cfg(&cfg) == 0) {
		BL_DBG("img seq %d, state %d\n", cfg.seq, cfg.state);
		if (cfg.state == IMAGE_STATE_VERIFIED) {
			cfg_seq = cfg.seq;
		} else {
			BL_WRN("invalid img state %d, seq %d\n", cfg.state, cfg.seq);
			cfg_seq = IMAGE_SEQ_NUM; /* set to invalid sequence */
		}
	} else {
		BL_WRN("ota read cfg fail\n");
		cfg_seq = IMAGE_SEQ_NUM; /* set to invalid sequence */
	}

	/* load app bin */
	load_seq = (cfg_seq == IMAGE_SEQ_NUM) ? 0 : cfg_seq;

	/* if img_xz_max_size is not invalid size, mean use image compression mode */
	if ((cfg_seq == 1) && (cfg.state == IMAGE_STATE_VERIFIED) &&
	    (iop->img_xz_max_size != IMAGE_INVALID_SIZE)) {
		int ret = bl_xz_image(cfg_seq);
		if (ret == 0) {
			load_seq = 0;
		} else if (ret == -2) {
			/* image is not compressed (eg. ETF image), try to load it */
		} else {
			return BL_INVALID_APP_UNZIP;
		}
	}

	if (l_seq) {
		*l_seq = load_seq;
	}
	if (c_seq) {
		*c_seq = cfg_seq;
	}
	image_set_running_seq(load_seq);

	return 0;
}

#ifdef CONFIG_SECURE_BOOT
extern int uECC_verify(const uint8_t *p_public_key, const uint8_t *p_message_hash,
        unsigned int p_hash_size, const uint8_t *p_signature, uECC_Curve curve);

void bl_sha256(const uint8_t *input, uint32_t ilen, uint8_t *digest)
{
	TCSha256State_t sha256_t;

	sha256_t = (TCSha256State_t)malloc(sizeof(TCSha256State_v));

	tc_sha256_init(sha256_t);
	tc_sha256_update(sha256_t, input, ilen);
	tc_sha256_final(digest, sha256_t);

	free(sha256_t);
}

uint32_t bl_sha256_burst(uint32_t faddr, uint32_t len, uint8_t *digest, uint8_t flash_psram)
{
	uint8_t *hash_burst_buf;
	uint32_t read_addr = faddr, hash_burst_size = 0x1000;
	int32_t  calc_len, remain_len = len;
	TCSha256State_t sha256_t;

	hash_burst_buf = (uint8_t *)malloc(hash_burst_size);
	if (hash_burst_buf == NULL) {
		BL_ERR("no heap space !\n");
		return BL_LOAD_BIN_INVALID;
	}
	sha256_t = (TCSha256State_t)malloc(sizeof(TCSha256State_v));
	if (sha256_t == NULL) {
		BL_ERR("no heap space !\n");
		free(hash_burst_buf);
		return BL_LOAD_BIN_INVALID;
	}

	tc_sha256_init(sha256_t);
	while (remain_len > 0) {
		calc_len = remain_len > hash_burst_size ? hash_burst_size : remain_len;
		if (!flash_psram) {
			if (flash_read(PRJCONF_IMG_FLASH, read_addr,
			               (void *)hash_burst_buf, calc_len) != calc_len) {
				return BL_LOAD_BIN_INVALID;
			}
		} else {
			memcpy((void *)hash_burst_buf, (void *)read_addr, calc_len);
		}
		tc_sha256_update(sha256_t, hash_burst_buf, calc_len);
		read_addr += calc_len;
		remain_len -= calc_len;
	}
	tc_sha256_final(digest, sha256_t);

	free(sha256_t);
	free(hash_burst_buf);

	return BL_LOAD_BIN_OK;
}

uint32_t bl_verify_signature(uint8_t *input, uint32_t len, uint8_t *signature)
{
	uint32_t r_len;
	uint8_t digest[32], pkey[64];

	r_len = image_read(IMAGE_BOOT_ID, IMAGE_SEG_TAILER, 0, (void *)pkey, ECC_PKEY_SIZE);
	if (r_len != ECC_PKEY_SIZE) {
		BL_WRN("ecc256 public key size %u, read %u\n", ECC_PKEY_SIZE, r_len);
		return BL_LOAD_BIN_NO_SEC;
	}

	bl_sha256(input, len, digest);
	if (!uECC_verify(pkey, digest, TC_SHA256_DIGEST_SIZE, signature, &curve_secp256r1)) {
		return BL_LOAD_BIN_INVALID;
	}

	return BL_LOAD_BIN_OK;
}

/* @note: verification of bootloader bin itself is not supported */
static uint32_t bl_verify_bin_by_id(uint32_t id)
{
	section_header_t sh;
	uint32_t flash_addr, r_len;
	uint8_t digest[TC_SHA256_DIGEST_SIZE], pkey[ECC_PKEY_SIZE], sign[ECC_SIGN_SIZE];

	BL_DBG("%s(), id %#x\n", __func__, id);

	flash_addr = image_get_section_addr(id);
	if (flash_addr == IMAGE_INVALID_ADDR) {
		BL_WRN("no bin(id:0x%x)\n", id);
#if BL_SB_VERIFY_FAIL_IF_NO_BIN
		return BL_LOAD_BIN_INVALID;
#else
		return BL_LOAD_BIN_OK;
#endif
	}

	r_len = image_read(id, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (r_len != IMAGE_HEADER_SIZE) {
		BL_WRN("invalid header(id:0x%x)\n", id);
		return BL_LOAD_BIN_INVALID;
	}

	if (!(sh.attribute & IMAGE_ATTR_FLAG_SIGN)) {
		BL_ERR("invalid sign flag(id:0x%x)\n", id);
		return BL_LOAD_BIN_INVALID;
	}

	r_len = sh.body_len;
	flash_addr += IMAGE_HEADER_SIZE;

#ifdef CONFIG_FLASH_CRYPTO
	FLASH_CRYPTO_RANGE crypto_ch = FCRYPTO_RANGE_NUM;
	if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
		crypto_ch = bl_flash_crypto_enable(flash_addr, flash_addr + r_len);
	}
#endif

	if (bl_sha256_burst(flash_addr, r_len, digest, 0)) {
#ifdef CONFIG_FLASH_CRYPTO
		if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
			bl_flash_crypto_disable(crypto_ch);
		}
#endif
		BL_ERR("calc sha256 by burst fail !\n");
		return BL_LOAD_BIN_INVALID;
	}

#ifdef CONFIG_FLASH_CRYPTO
	if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
		bl_flash_crypto_disable(crypto_ch);
	}
#endif

	r_len = image_read(IMAGE_BOOT_ID, IMAGE_SEG_TAILER, 0, (void *)pkey, ECC_PKEY_SIZE);
	if (r_len != ECC_PKEY_SIZE) {
		BL_WRN("ecc256 pub key size %u, read %u\n", ECC_PKEY_SIZE, r_len);
		return BL_LOAD_BIN_NO_SEC;
	}
	r_len = image_read(id, IMAGE_SEG_TAILER, 0, (void *)sign, ECC_SIGN_SIZE);
	if (r_len != ECC_SIGN_SIZE) {
		BL_WRN("ecc256 sign size %u, read %u\n", ECC_SIGN_SIZE, r_len);
		return BL_LOAD_BIN_NO_SEC;
	}

	if (!uECC_verify(pkey, digest, TC_SHA256_DIGEST_SIZE, (uint8_t *)sign, &curve_secp256r1)) {
		BL_ERR("verify bin(id:%#x) sign fail !\n", id);
		return BL_LOAD_BIN_INVALID;
	}

	return BL_LOAD_BIN_OK;
}

#endif /* CONFIG_SECURE_BOOT */

static int bl_load_bin_by_id(uint32_t id, uint32_t max_addr, uint32_t *entry)
{
	uint32_t len;
	section_header_t sh;

	BL_DBG("%s(), id %#x\n", __func__, id);

	len = image_read(id, IMAGE_SEG_HEADER, 0, &sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_WRN("bin header size %u, read %u\n", IMAGE_HEADER_SIZE, len);
		return BL_LOAD_BIN_NO_SEC;
	}

	if (image_check_header(&sh) == IMAGE_INVALID) {
		BL_WRN("invalid bin header\n");
		return BL_LOAD_BIN_INVALID;
	}
#if (defined(CONFIG_SECURE_BOOT) && BL_SB_TEST_LOAD_BOOT_BIN)
	if (id == IMAGE_BOOT_ID) {
		sh.load_addr = BL_TEST_BOOT_BIN_LOAD_ADDR;
	}
#endif
#ifdef CONFIG_BIN_COMPRESS
	if (sh.attribute & IMAGE_ATTR_FLAG_COMPRESS) {
#if (defined(CONFIG_SECURE_BOOT)) && (CONFIG_CHIP_ARCH_VER == 3)
		if (bl_verify_bin_by_id(id)) {
			BL_ERR("verify bin %#x fail\n", id);
			return BL_LOAD_BIN_INVALID;
		}
#endif
		if (bl_decompress_bin(&sh, max_addr - sh.load_addr) != 0) {
			BL_ERR("decompress bin %#x failed\n", id);
			return BL_LOAD_BIN_INVALID;
		}
	} else
#endif /* CONFIG_BIN_COMPRESS */
	{
#if BL_DBG_ON
		OS_Time_t tm;
		tm = OS_GetTicks();
#endif
		if (sh.body_len == 0) {
			BL_WRN("body_len %u\n", sh.body_len);
			return BL_LOAD_BIN_NO_SEC;
		}

		if (sh.load_addr + sh.data_size > max_addr) {
			BL_WRN("bin too big, %#x + %#x > %x\n", sh.load_addr, sh.data_size,
			       max_addr);
			return BL_LOAD_BIN_TOOBIG;
		}

		len = image_read(id, IMAGE_SEG_BODY, 0, (void *)sh.load_addr,
		                 sh.body_len);
		if (len != sh.body_len) {
			BL_WRN("bin body size %u, read %u\n", sh.body_len, len);
			return BL_LOAD_BIN_INVALID;
		}

#ifdef CONFIG_SECURE_BOOT
		uint8_t *sign_buf = (uint8_t *)sh.load_addr + sh.body_len;
		uint32_t sign_len = ECC_SIGN_SIZE;
		len = image_read(id, IMAGE_SEG_TAILER, 0, (void *)sign_buf, sign_len);
		if (len != sign_len) {
			BL_WRN("bin tailer size %u, read %u\n", sign_len, len);
			return BL_LOAD_BIN_INVALID;
		}
#else
		uint8_t *sign_buf = NULL;
		uint32_t sign_len = 0;
#endif

		if (image_check_data(&sh, (void *)sh.load_addr, sh.body_len,
		                     (void *)sign_buf, sign_len) == IMAGE_INVALID) {
			BL_WRN("invalid bin body\n");
			return BL_LOAD_BIN_INVALID;
		}
#if BL_DBG_ON
		tm = OS_GetTicks() - tm;
		BL_DBG("%s() cost %u ms\n", __func__, tm);
#endif

#if (defined(CONFIG_SECURE_BOOT)) && (CONFIG_CHIP_ARCH_VER == 2)
		if (sh.attribute & IMAGE_ATTR_FLAG_SIGN) {
#if BL_SB_TEST_FAKE_EFUSE_PUBKEY_HASH
			if (secureboot_verify(&sh, efuse_pubkey_hash) != 0) {
#else
			if (secureboot_verify(&sh, NULL) != 0) {
#endif
				BL_ERR("check sign bin %#x failed\n", id);
				return BL_LOAD_BIN_INVALID;
			}
		} else {
			BL_ERR("invalid sign flag for secure boot\n");
			return BL_LOAD_BIN_INVALID;
		}
#endif
#if (defined(CONFIG_SECURE_BOOT)) && (CONFIG_CHIP_ARCH_VER == 3)
		if (sh.attribute & IMAGE_ATTR_FLAG_SIGN) {
			if (bl_verify_signature((uint8_t *)sh.load_addr, sh.body_len,
			                        (uint8_t *)sign_buf)) {
				BL_ERR("verify bin(id:%#x) sign fail !\n", id);
				return BL_LOAD_BIN_INVALID;
			}
		} else {
			BL_ERR("invalid sign flag(id:%#x)\n", id);
			return BL_LOAD_BIN_INVALID;
		}
#endif
	}

	if (entry) {
		*entry = sh.entry;
	}
	return BL_LOAD_BIN_OK;
}

#if defined(CONFIG_TRUSTZONE_BOOT)
static uint32_t bl_relocate_tz_params(image_seq_t load_seq)
{
	uint32_t len, addr, region_s, region_e, tz_params_entry = 0;
	uint8_t *tz_params;
	section_header_t sh;
	int ret = BL_LOAD_BIN_INVALID;

	tz_params = (uint8_t *)malloc(TZ_PARAMS_SIZE + ECC_SIGN_SIZE);
	if (tz_params == NULL) {
		BL_ERR("no heap space !\n");
		return BL_LOAD_BIN_INVALID;
	}

	/* 1. load tz_params.bin from flash */
	len = image_read(IMAGE_TZ_PARAM_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_WRN("tz params size %u, read %u\n", IMAGE_HEADER_SIZE, len);
		goto end;
	}
	if (image_check_header(&sh) == IMAGE_INVALID) {
		BL_WRN("invalid bin header\n");
		goto end;
	}

	len = image_read(IMAGE_TZ_PARAM_ID, IMAGE_SEG_BODY, 0, (void *)tz_params, sh.body_len + ECC_SIGN_SIZE);
	if (len != sh.body_len + ECC_SIGN_SIZE) {
		BL_WRN("bin body size %u, read %u\n", sh.body_len, len);
		goto end;
	}
	if (image_check_data(&sh, (void *)tz_params, sh.body_len,
	                     (void *)(tz_params + sh.body_len), ECC_SIGN_SIZE) == IMAGE_INVALID) {
		BL_WRN("invalid bin body\n");
		goto end;
	}
	tz_params_entry = sh.load_addr;

	/* 2. verify signature of tz_params by ext_ecc_publickey */
	if (bl_verify_signature(tz_params, TZ_PARAMS_SIZE, &tz_params[TZ_PARAMS_SIZE])) {
		BL_ERR("verify tz_params signature failure !\n");
		goto end;
	}

	/* 3. transmit tz_params into it's entry, which would be used during tz init stage */
	memcpy((void *)tz_params_entry, (void *)tz_params, TZ_PARAMS_SIZE);
	*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE + 0xc) = 0ul;
	if (load_seq == 1) {
		const image_ota_param_t *iop = image_get_ota_param();
		*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE + 0xc) = iop->ota_addr;
	}

	/* 4. transmit the flash offset of tz_xip.bin to tz if exist */
	addr = image_get_section_addr(IMAGE_TZ_XIP_ID);
	if (addr != IMAGE_INVALID_ADDR) {
		*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE) = addr + IMAGE_HEADER_SIZE;
	}

	/* 5. transmit the psram offset of tz_psram.bin to tz if exist */
	//len = image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	//if (len == IMAGE_HEADER_SIZE) {
		*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE + 0x4) = PRJ_TZ_PSRAM_START_PHY;//sh.load_addr;
	//}

	/* 6. transmit the tz_xip.bin & tz_psram.bin crypto information to trustzone */
	len = image_read(IMAGE_TZ_XIP_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len == IMAGE_HEADER_SIZE) {
		if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
			*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE + 0x8) = 1;
		}
	}
	len = image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len == IMAGE_HEADER_SIZE) {
		if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
			*(uint32_t *)(tz_params_entry + TZ_PARAMS_SIZE + 0x8) |= 2;
		}
	}

	/****************** Generate NS physical region automatically ******************/

	/* The first 3 regions must be NS FLASH 1&2 and NS PSRAM in order to config TZASC */

	/*** Automatically Dynamic Config Region ***/

	/* region-[1]. NS FLASH Region-1, fixed for bootloader */
	region_s = image_get_section_addr(IMAGE_BOOT_ID); // 0x00000000
	region_e = image_get_section_addr(IMAGE_TZ_PARAM_ID);
	if (region_s == IMAGE_INVALID_ADDR || region_e == IMAGE_INVALID_ADDR) {
		BL_ERR("Read bootloader region physical addr in flash error!\n");
		goto end;
	}
	*(uint32_t *)(tz_params_entry + 0x00) = region_s;
	*(uint32_t *)(tz_params_entry + 0x04) = region_e - 1;

	/* region-[2]. NS FLASH Region-2, used for app+wlan+other NS */
	region_s = image_get_section_addr(IMAGE_APP_ID);
	region_e = 0x00FFFFFF; // flash VMA max size is 16M, which will set in SAU
	if (region_s == IMAGE_INVALID_ADDR) {
		BL_ERR("Read app region physical addr in flash error!\n");
		goto end;
	}
	*(uint32_t *)(tz_params_entry + 0x08) = region_s;
	*(uint32_t *)(tz_params_entry + 0x0C) = region_e;

	/* region-[3]. NS PSRAM, used for NS PSRAM */
	len = image_read(IMAGE_TZ_PSRAM_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len == IMAGE_HEADER_SIZE) {
		region_s = 0x00000000;
		region_e = PRJ_TZ_PSRAM_START_PHY - 1; // 0x001F7FFF
	} else {
		region_s = 0x00000000;
		region_e = 0x01000000-1; // Invalid end addr, will be replace in TZ_APP according to not define CONFIG TZ PSRAM
	}
	*(uint32_t *)(tz_params_entry + 0x10) = region_s;
	*(uint32_t *)(tz_params_entry + 0x14) = region_e;

	/* region-[4]. NS SRAM Region-1, used for NS ROM */
	region_s = 0x00200000;
	region_e = 0x00201000 - 1;
	len = image_read(IMAGE_TZ_APP_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len == IMAGE_HEADER_SIZE) {
		if (sh.load_addr == 0x00200000) { // TZ APP go forward to SRAM start addr 0x00200000, indicate not use ROM
			region_s = 0x00000000;
			region_e = 0x00000000; // Invalid end addr,will be skip in TZ_APP
		}
	}
	*(uint32_t *)(tz_params_entry + 0x18) = region_s;
	*(uint32_t *)(tz_params_entry + 0x1C) = region_e;

	/* region-[5]. NS SRAM Region-2, used for NS APP */
	len = image_read(IMAGE_APP_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (len == IMAGE_HEADER_SIZE) {
		region_s = sh.load_addr; // 0x00208000
		region_e = 0x00253FFF; // SRAM max size = 336K
		*(uint32_t *)(tz_params_entry + 0x20) = region_s;
		*(uint32_t *)(tz_params_entry + 0x24) = region_e;
	} else {
		BL_ERR("Read app region header error!\n");
		goto end;
	}

	/*** User Config Region ***/
#if 0
	/* region-[6]. NS ROM Region, used for NS ROM */
	region_s = 0x0000A800;
	region_e = 0x00028000-1;
	*(uint32_t *)(tz_params_entry + 0x28) = region_s;
	*(uint32_t *)(tz_params_entry + 0x2C) = region_e;

	/* region-[7]. NS All Peripherals Region */
	region_s = 0x40000000;
	region_e = 0xBAFFFFFF;
	*(uint32_t *)(tz_params_entry + 0x30) = region_s;
	*(uint32_t *)(tz_params_entry + 0x34) = region_e;
#endif


	/*** Printf all the NS region config information ***/
	BL_INF("/****** NS region config information ******/\n");
	BL_INF("FLASH Region 1: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x00), readl(tz_params_entry + 0x04));
	BL_INF("FLASH Region 2: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x08), readl(tz_params_entry + 0x0C));
	BL_INF("PSRAM Region 1: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x10), readl(tz_params_entry + 0x14));
	BL_INF("SRAM  Region 1: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x18), readl(tz_params_entry + 0x1C));
	BL_INF("SRAM  Region 2: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x20), readl(tz_params_entry + 0x24));
	BL_INF("USER  Region 1: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x28), readl(tz_params_entry + 0x2C));
	BL_INF("USER  Region 2: [0x%08lX, 0x%08lX]\n", readl(tz_params_entry + 0x30), readl(tz_params_entry + 0x34));
	BL_INF("/******************************************/\n");
	/************************************************************************/

	ret = BL_LOAD_BIN_OK;

end:
	free(tz_params);
	return ret;
}

static uint32_t bl_load_verify_secure_bin(uint32_t *entry)
{
	extern const unsigned char __RAM_BASE[]; /* SRAM start address of bl */
	int ret;

	ret = bl_load_bin_by_id(IMAGE_TZ_APP_ID, (uint32_t)__RAM_BASE, entry);
	if (ret != BL_LOAD_BIN_OK) {
		return BL_INVALID_APP_ENTRY;
	}

	return BL_LOAD_BIN_OK;
}

static __inline uint32_t bl_load_verify_secure_xip_bin(void)
{
	return bl_verify_bin_by_id(IMAGE_TZ_XIP_ID);
}
#endif

static uint32_t bl_load_app_bin(void)
{
	extern const unsigned char __RAM_BASE[]; /* SRAM start address of bl */
	uint32_t entry;
	int ret;

#if (defined(CONFIG_SECURE_BOOT) && BL_SB_TEST_LOAD_BOOT_BIN)
	bl_load_bin_by_id(IMAGE_BOOT_ID, (uint32_t)__RAM_BASE, &entry);
#endif

	ret = bl_load_bin_by_id(IMAGE_APP_ID, (uint32_t)__RAM_BASE, &entry);
	if (ret != BL_LOAD_BIN_OK) {
		return BL_INVALID_APP_ENTRY;
	}

	return entry;
}

#if (defined(CONFIG_PSRAM) && defined(CONFIG_TRUSTZONE_BOOT))
static uint32_t bl_load_psram_bin(void)
{
	extern void load_psram_bin(void);

	load_psram_bin();

	return BL_LOAD_BIN_OK;
}
#endif

static uint32_t bl_load_bin(image_seq_t cfg_seq, image_seq_t load_seq)
{
	uint32_t entry;
	image_cfg_t cfg;
	image_seq_t i;

	for (i = 0; i < IMAGE_SEQ_NUM; ++i) {
		image_set_running_seq(load_seq);
		entry = bl_load_app_bin();
		if (entry != BL_INVALID_APP_ENTRY) {
			if (load_seq != cfg_seq) {
				BL_WRN("boot from seq %u, cfg_seq %u\n", load_seq, cfg_seq);
				cfg.seq = load_seq;
				cfg.state = IMAGE_STATE_VERIFIED;
				if (image_set_cfg(&cfg) != 0) {
					BL_ERR("write img cfg fail\n");
				}
			}
			return entry;
		} else {
			BL_WRN("load app bin fail, seq %u\n", load_seq);
			load_seq = (load_seq + 1) % IMAGE_SEQ_NUM;
		}
	}

	return BL_INVALID_APP_ENTRY;
}

#if BL_SHOW_INFO
static void bl_show_info(void)
{
	extern uint8_t __text_start__[];
	extern uint8_t __text_end__[];
	extern uint8_t __etext[];
	extern uint8_t __data_start__[];
	extern uint8_t __data_end__[];
	extern uint8_t __bss_start__[];
	extern uint8_t __bss_end__[];
	extern uint8_t __heap_start__[];
	extern uint8_t __heap_end__[];
	extern uint8_t __end__[];
	extern uint8_t end[];
	extern uint8_t __HeapLimit[];
	extern uint8_t __StackLimit[];
	extern uint8_t __StackTop[];
	extern uint8_t __stack[];
	extern uint8_t _estack[];

	BL_LOG(1, "__text_start__ %p\n", __text_start__);
	BL_LOG(1, "__text_end__   %p\n", __text_end__);
	BL_LOG(1, "__etext        %p\n", __etext);
	BL_LOG(1, "__data_start__ %p\n", __data_start__);
	BL_LOG(1, "__data_end__   %p\n", __data_end__);
	BL_LOG(1, "__bss_start__  %p\n", __bss_start__);
	BL_LOG(1, "__bss_end__    %p\n", __bss_end__);
	BL_LOG(1, "__end__        %p\n", __end__);
	BL_LOG(1, "end            %p\n", end);
	BL_LOG(1, "__HeapLimit    %p\n", __HeapLimit);
	BL_LOG(1, "__StackLimit   %p\n", __StackLimit);
	BL_LOG(1, "__StackTop     %p\n", __StackTop);
	BL_LOG(1, "__stack        %p\n", __stack);
	BL_LOG(1, "_estack        %p\n", _estack);
	BL_LOG(1, "\n");

	BL_LOG(1, "heap space [%p, %p), size %u\n\n",
	           __heap_start__, __heap_end__,
	           __heap_end__ - __heap_start__);
}
#endif /* BL_SHOW_INFO */

int main(void)
{
	uint32_t boot_flag;
	register uint32_t entry;
	image_seq_t cfg_seq = 0, load_seq = 0;
	int ret;

	bl_hw_init();

	BL_DBG("start\n");
#if BL_SHOW_INFO
	bl_show_info();
#endif

	boot_flag = HAL_PRCM_GetCPUABootFlag();
	if (boot_flag == PRCM_CPUA_BOOT_FROM_COLD_RESET) {
try_again:
		bl_flash_init();

		ret = bl_uncompress_image(&cfg_seq, &load_seq);
		if (ret == BL_INVALID_APP_UNZIP) {
			BL_ERR("uncompress image fail, enter upgrade mode\n");
			goto upgrade;
		}
		if (ret == BL_INVALID_APP_ENTRY) {
			if (HAL_PRCM_IsFlashSip()) {
				BL_LOG(1, "load sip flash app bin fail, try ext flash\n");
				HAL_PRCM_SetFlashExt(1);
				bl_flash_deinit();
				goto try_again;
			}
			BL_ERR("load ext flash app bin fail, enter upgrade mode\n");
			goto upgrade;
		}

#if defined(CONFIG_SECURE_BOOT)
#if defined(CONFIG_APP_XIP_BIN_VERIFY)
		if (bl_verify_bin_by_id(IMAGE_APP_XIP_ID)) {
			BL_ERR("verify app_xip bin fail\n");
			HAL_UDelay(7000);
			goto upgrade;
		}
		BL_DBG("verify bin(id:%#x) success\n", IMAGE_APP_XIP_ID);
#endif
#if defined(CONFIG_APP_PSRAM_BIN_VERIFY)
		if (bl_verify_bin_by_id(IMAGE_APP_PSRAM_ID)) {
			BL_ERR("verify app_psram bin fail\n");
			HAL_UDelay(7000);
			goto upgrade;
		}
		BL_DBG("verify bin(id:%#x) success\n", IMAGE_APP_PSRAM_ID);
#endif
#endif /* CONFIG_SECURE_BOOT */

#if defined(CONFIG_TRUSTZONE_BOOT)
		uint32_t tz_entry;

		if (bl_relocate_tz_params(load_seq)) {
			goto upgrade;
		}
		if (bl_load_verify_secure_bin(&tz_entry)) {
			goto upgrade;
		}
		if (bl_load_verify_secure_xip_bin()) {
			goto upgrade;
		}
		BL_DBG("enable trustzone feature !\n");
#else
		BL_DBG("no trustzone feature !\n");
#endif

#if (defined(CONFIG_PSRAM) && defined(CONFIG_TRUSTZONE_BOOT))
		entry = bl_load_psram_bin();
		if (entry == BL_INVALID_APP_ENTRY) {
			BL_DBG("no psram bin\n");
		}
#endif

		entry = bl_load_bin(cfg_seq, load_seq);
		if (entry == BL_INVALID_APP_ENTRY) {
			if (HAL_PRCM_IsFlashSip()) {
				BL_LOG(1, "load sip flash app bin fail, try ext flash\n");
				HAL_PRCM_SetFlashExt(1);
				bl_flash_deinit();
				goto try_again;
			}
			if (entry == BL_INVALID_APP_ENTRY) {
				BL_ERR("load app bin fail, enter upgrade mode\n");
				goto upgrade;
			}
		}
		bl_flash_deinit();

#if defined(CONFIG_TRUSTZONE_BOOT)
		entry = tz_entry;
#endif
#if (defined(CONFIG_CPU_CM4F) || defined(CONFIG_CPU_CM3) || defined(CONFIG_CPU_CM33F))
		entry |= 0x1; /* set thumb bit */
#endif
		BL_DBG("goto %#x\n", entry);
		goto run_app;
	} else {
		BL_ERR("boot flag %#x\n", boot_flag);
		BL_ABORT();
	}

upgrade:
	bl_hw_deinit();
	bl_upgrade();
	BL_ABORT();

run_app:
	bl_hw_deinit();

	__disable_fault_irq();
	__disable_irq();
	__set_CONTROL(0); /* reset to Privileged Thread mode and use MSP */
	__DSB();
	__ISB();

	((NVIC_IRQHandler)entry)(); /* never return, if enable trustzone, boot to tz_app */

	/* BL_ERR("unreachable\n"); */
	BL_ABORT();

	return -1;
}
