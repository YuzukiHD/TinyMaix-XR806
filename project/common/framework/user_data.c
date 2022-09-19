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
#include "image/fdcm.h"
#include "image/image.h"
#include "driver/chip/hal_flashctrl.h"

#include "user_data.h"
#include "user_data_debug.h"

static uint8_t flash_enc_nonce[6] = {0x50, 0x00, 0x06, 0x20, 0x00, 0x00};

static struct user_data g_user_data;
static fdcm_handle_t *g_user_data_fdcm_hdl;
static int8_t user_data_crypto_channel = 0xff;

int user_data_init(void)
{
#if PRJCONF_USER_DATA_CHECK_OVERLAP
	image_ota_param_t *iop;
	int i;
	uint32_t image_start, image_end;

	iop = (image_ota_param_t *)image_get_ota_param();
	for (i = 0; i < IMAGE_SEQ_NUM; ++i) {
		image_start = iop->addr[i];
		if (i == 0)
			image_end = iop->addr[i] + IMAGE_AREA_SIZE(iop->img_max_size);
		else
#if (defined(CONFIG_OTA_POLICY_PINGPONG))
			image_end = iop->addr[i] + IMAGE_AREA_SIZE(iop->img_max_size);
#else
			image_end = iop->addr[i] + IMAGE_AREA_SIZE(iop->img_xz_max_size);
#endif
		if (PRJCONF_USER_DATA_ADDR >= image_start && PRJCONF_USER_DATA_ADDR < image_end) {
			USER_DATA_ERR("user_data: %#x has overlay image%d: %#x - %#x\n",
			    PRJCONF_USER_DATA_ADDR, i, image_start, image_end);
			return -1;
		}
	}

	if (PRJCONF_USER_DATA_ADDR >= iop->ota_addr &&
		PRJCONF_USER_DATA_ADDR < iop->ota_addr + iop->ota_size) {
		USER_DATA_ERR("user_data: %#x has overlay ota area: %#x - %#x\n",
		    PRJCONF_USER_DATA_ADDR, iop->ota_addr, iop->ota_addr + iop->ota_size);
		return -1;
	}
#endif

	g_user_data_fdcm_hdl = fdcm_open(PRJCONF_USER_DATA_FLASH, PRJCONF_USER_DATA_ADDR, PRJCONF_USER_DATA_SIZE);
	if (g_user_data_fdcm_hdl == NULL) {
		USER_DATA_ERR("fdcm open failed, hdl %p\n", g_user_data_fdcm_hdl);
		return -1;
	}

	//#undef FLASH_XIP_OPT_READ
	if (user_data_crypto_channel == 0xff) {
		uint8_t  key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79, 0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
		HAL_FlashCrypto_Init(flash_enc_nonce);
		user_data_crypto_channel = FlashCryptoRequest(PRJCONF_USER_DATA_ADDR, PRJCONF_USER_DATA_ADDR + PRJCONF_USER_DATA_SIZE - 1, key);
		if (user_data_crypto_channel < 0) {
			user_data_crypto_channel = 0xff;
			fdcm_close(g_user_data_fdcm_hdl);
			USER_DATA_ERR("User data request flash crypto channel fail!\n");
			return -1;
		}
	}

	return 0;
}

void user_data_deinit(void)
{
	fdcm_close(g_user_data_fdcm_hdl);

	if (user_data_crypto_channel != 0xff)
		FlashCryptoRelease(user_data_crypto_channel);
}

int user_data_save(void)
{
	if (g_user_data_fdcm_hdl == NULL) {
		USER_DATA_ERR("uninitialized hdl %p\n", g_user_data_fdcm_hdl);
		return -1;
	}

	if (fdcm_write(g_user_data_fdcm_hdl, &g_user_data, USER_DATA_SIZE) != USER_DATA_SIZE) {
		USER_DATA_ERR("fdcm write failed\n");
		return -1;
	}

	USER_DATA_DBG("save user_data to flash\n");

	return 0;
}

int user_data_load(void)
{
	if (g_user_data_fdcm_hdl == NULL) {
		USER_DATA_ERR("uninitialized hdl %p\n", g_user_data_fdcm_hdl);
		return -1;
	}

	if (fdcm_read(g_user_data_fdcm_hdl, &g_user_data, USER_DATA_SIZE) != USER_DATA_SIZE) {
		USER_DATA_ERR("fdcm read failed\n");
		return -1;
	}

	USER_DATA_DBG("load user_data from flash\n");

	return 0;
}

struct user_data *user_data_get(void)
{
	if (g_user_data_fdcm_hdl == NULL) {
		USER_DATA_ERR("uninitialized hdl %p\n", g_user_data_fdcm_hdl);
		return NULL;
	}

	return &g_user_data;
}

