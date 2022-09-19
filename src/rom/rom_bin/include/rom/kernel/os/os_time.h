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

#ifndef _ROM_KERNEL_OS_OS_TIME_H_
#define _ROM_KERNEL_OS_OS_TIME_H_

#include "kernel/os/os_time.h"

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#undef  OS_HZ
#define OS_HZ \
	RAM_TBL_DATA(uint32_t, OS_HZ)

#undef  OS_GetTicks
#define OS_GetTicks \
	RAM_TBL_FUN(OS_Time_t (*)(void), OS_GetTicks)

#undef  OS_MSleep
#define OS_MSleep \
	RAM_TBL_FUN(void (*)(uint32_t), OS_MSleep)

#undef  OS_Rand32
#define OS_Rand32 \
	RAM_TBL_FUN(uint32_t (*)(void), OS_Rand32)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ROM */

#endif /* _ROM_KERNEL_OS_OS_TIME_H_ */
