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
#include "image/image.h"
#include "image_debug.h"
#ifdef CONFIG_SRAM_BIN_ENCRYPT
#include "driver/chip/hal_flashctrl.h"
#endif

uint32_t image_get_addr(image_priv_t *img, image_seq_t seq, uint32_t id)
{
#ifdef CONFIG_TRUSTZONE_BOOT
	return tzboot_image_get_addr(img, seq, id);
#else
	return app_image_get_addr(img, seq, id);
#endif
}

uint32_t image_get_bl_size(section_header_t *sh)
{
#if (defined(CONFIG_TRUSTZONE) || defined(CONFIG_TRUSTZONE_BOOT))
	return sh->priv[5];
#else
	return sh->next_addr;
#endif
}

#ifdef CONFIG_SRAM_BIN_ENCRYPT
uint32_t image_read(uint32_t id, image_seg_t seg, uint32_t offset, void *buf, uint32_t size)
{
#if (defined(CONFIG_TRUSTZONE_BOOT) || defined(CONFIG_TRUSTZONE))
	if (seg == IMAGE_SEG_BODY && id != IMAGE_APP_PSRAM_ID && id != IMAGE_TZ_PSRAM_ID) {
#else
	if (seg == IMAGE_SEG_BODY && id != IMAGE_APP_PSRAM_ID) {
#endif
		section_header_t sh;
		uint32_t r_len;
		FLASH_CRYPTO_RANGE crypto_ch = FCRYPTO_RANGE_NUM;

		r_len = image_rw(id, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE, 0);
		if (r_len != IMAGE_HEADER_SIZE) {
			IMAGE_WRN("invalid header(id:0x%x)\n", id);
			return -1;
		}

		if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
			uint32_t flash_addr = image_get_section_addr(id) + IMAGE_HEADER_SIZE;
			crypto_ch = FlashcBinDecryptEnable(FCRYPTO_RANGE_1, flash_addr + offset, flash_addr + offset + size);
		}

		r_len = image_rw(id, seg, offset, buf, size, 0);

		if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
			FlashcBinDecryptDisable(crypto_ch);
		}

		return r_len;
	} else {
		return image_rw(id, seg, offset, buf, size, 0);
	}
}
#endif /* CONFIG_SRAM_BIN_ENCRYPT */
