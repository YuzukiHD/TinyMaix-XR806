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

#include "ota_i.h"
#include "ota_debug.h"
#include "ota_file.h"

#if OTA_OPT_PROTOCOL_FLASH
typedef struct {
	uint32_t flash_num;
	uint32_t ota_image_addr;
	uint32_t ota_image_size;
	uint32_t current_read_pos;
} ota_flash_param_t;
static ota_flash_param_t g_ota_flash_param;

ota_status_t ota_update_flash_init(void *url)
{
	const image_ota_param_t *iop = ota_priv.iop;
	image_seq_t seq;

	seq = ota_get_update_seq();

	g_ota_flash_param.flash_num = iop->flash[seq];
	g_ota_flash_param.ota_image_addr = 0x200000;
	g_ota_flash_param.ota_image_size = 0xC0000;
	g_ota_flash_param.current_read_pos = 0;
#if (defined CONFIG_OTA_POLICY_IMAGE_COMPRESSION)
	g_ota_flash_param.ota_image_size = 0x4b000;
#endif

	if (HAL_Flash_Open(g_ota_flash_param.flash_num, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("open flash %u fail\n", g_ota_flash_param.flash_num);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), success\n", __func__);
	return OTA_STATUS_OK;
}

ota_status_t ota_update_flash_get(uint8_t *buf, uint32_t buf_size, uint32_t *recv_size, uint8_t *eof_flag)
{
	ota_flash_param_t *ota_get_t = &g_ota_flash_param;

	if (ota_get_t->ota_image_size - ota_get_t->current_read_pos > buf_size) {
		*recv_size = buf_size;
		*eof_flag = 0;
	} else {
		*recv_size = ota_get_t->ota_image_size - ota_get_t->current_read_pos;
		*eof_flag = 1;
	}

	if (HAL_Flash_Read(ota_get_t->flash_num,
		ota_get_t->ota_image_addr + ota_get_t->current_read_pos,
		buf, *recv_size) != HAL_OK) {
		return OTA_STATUS_ERROR;
	}

	ota_get_t->current_read_pos += buf_size;

	return OTA_STATUS_OK;
}

#endif /* OTA_OPT_PROTOCOL_FLASH */
