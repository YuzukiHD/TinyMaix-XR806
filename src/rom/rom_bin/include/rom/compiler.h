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

#ifndef _ROM_COMPILER_H_
#define _ROM_COMPILER_H_

#include "compiler.h"

#ifdef CONFIG_ROM
#undef __xip_text
#define __xip_text
#undef __xip_rodata
#define __xip_rodata

#undef __xip_text
#define __xip_text
#undef __xip_rodata
#define __xip_rodata
#undef __nonxip_text
#define __nonxip_text
#undef __nonxip_rodata
#define __nonxip_rodata
#undef __nonxip_data
#define __nonxip_data
#undef __nonxip_bss
#define __nonxip_bss

#undef __nonxip_text
#define __nonxip_text
#undef __nonxip_rodata
#define __nonxip_rodata
#undef __nonxip_data
#define __nonxip_data
#undef __nonxip_bss
#define __nonxip_bss

#undef __sram_text
#define __sram_text
#undef __sram_rodata
#define __sram_rodata
#undef __sram_data
#define __sram_data
#undef __sram_bss
#define __sram_bss

#undef __psram_text
#define __psram_text
#undef __psram_rodata
#define __psram_rodata
#undef __psram_data
#define __psram_data
#undef __psram_bss
#define __psram_bss
#else

#endif /* CONFIG_ROM */

#endif /* _ROM_COMPILER_H_ */
