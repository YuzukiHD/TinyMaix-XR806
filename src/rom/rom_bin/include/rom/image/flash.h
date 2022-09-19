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

#ifndef _ROM_IMAGE_FLASH_H_
#define _ROM_IMAGE_FLASH_H_

#include "image/flash.h"
#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ROM

#define flash_rw \
	RAM_TBL_FUN(int (*)(uint32_t flash, uint32_t addr, void *buf, uint32_t size, int do_write), flash_rw)

#define flash_read(flash, addr, buf, size) \
	flash_rw(flash, addr, buf, size, 0)

#define flash_write(flash, addr, buf, size) \
	flash_rw(flash, addr, (void *)buf, size, 1)

#define flash_get_erase_block \
	RAM_TBL_FUN(int32_t (*)(uint32_t flash, uint32_t addr, uint32_t size), flash_get_erase_block)

#define flash_erase \
	RAM_TBL_FUN(int (*)(uint32_t flash, uint32_t addr, uint32_t size), flash_erase)

#else /* CONFIG_ROM */

#define rom_flash_rw                flash_rw
#define rom_flash_get_erase_block   flash_get_erase_block
#define rom_flash_erase             flash_erase

#endif /* CONFIG_ROM */

#ifdef __cplusplus
}
#endif

#endif /* _ROM_IMAGE_FLASH_H_ */
