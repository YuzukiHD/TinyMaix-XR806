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

#ifndef _ROM_KERNEL_OS_OS_SEMAPHORE_H_
#define _ROM_KERNEL_OS_OS_SEMAPHORE_H_

#include "kernel/os/os_semaphore.h"

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OS_SemaphoreCreate \
	RAM_TBL_FUN(OS_Status (*)(OS_Semaphore_t *sem, uint32_t initCount, uint32_t maxCount), OS_SemaphoreCreate)

#define OS_SemaphoreCreateBinary \
	RAM_TBL_FUN(OS_Status (*)(OS_Semaphore_t *sem), OS_SemaphoreCreateBinary)

#define OS_SemaphoreDelete \
	RAM_TBL_FUN(OS_Status (*)(OS_Semaphore_t *sem), OS_SemaphoreDelete)

#define OS_SemaphoreWait \
	RAM_TBL_FUN(OS_Status (*)(OS_Semaphore_t *sem, OS_Time_t waitMS), OS_SemaphoreWait)

#define OS_SemaphoreRelease \
	RAM_TBL_FUN(OS_Status (*)(OS_Semaphore_t *sem), OS_SemaphoreRelease)

#define OS_SemaphoreIsValid \
	RAM_TBL_FUN(int (*)(OS_Semaphore_t *sem), OS_SemaphoreIsValid)

#define OS_SemaphoreSetInvalid \
	RAM_TBL_FUN(void (*)(OS_Semaphore_t *sem), OS_SemaphoreSetInvalid)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ROM */

#endif /* _ROM_KERNEL_OS_OS_SEMAPHORE_H_ */
