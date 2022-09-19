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

#ifndef _ROM_IMAGE_IMAGE_H_
#define _ROM_IMAGE_IMAGE_H_

#include "image/image.h"
#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ROM

#define image_checksum16 \
	RAM_TBL_FUN(uint16_t (*)(uint8_t *data, uint32_t len), image_checksum16)

#define image_get_addr \
	RAM_TBL_FUN(uint32_t (*)(image_priv_t *img, image_seq_t seq, uint32_t id), image_get_addr)

#define image_get_bl_size \
	RAM_TBL_FUN(uint32_t (*)(section_header_t *sh), image_get_bl_size)

#else /* CONFIG_ROM */

#define rom_image_checksum16   image_checksum16

#define rom_image_get_addr   image_get_addr

#define rom_image_get_bl_size   image_get_bl_size

#endif /* CONFIG_ROM */

#ifdef __cplusplus
}
#endif

#endif /* _ROM_IMAGE_IMAGE_H_ */
