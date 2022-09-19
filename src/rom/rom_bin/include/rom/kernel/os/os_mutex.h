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

#ifndef _ROM_KERNEL_OS_OS_MUTEX_H_
#define _ROM_KERNEL_OS_OS_MUTEX_H_

#include "kernel/os/os_mutex.h"

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OS_MutexCreate \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_MutexCreate)

#define OS_MutexDelete \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_MutexDelete)

#define OS_MutexLock \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex, OS_Time_t waitMS), OS_MutexLock)

#define OS_MutexUnlock \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_MutexUnlock)

#define OS_RecursiveMutexCreate \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_RecursiveMutexCreate)

#define OS_RecursiveMutexDelete \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_RecursiveMutexDelete)

#define OS_RecursiveMutexLock \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex, OS_Time_t waitMS), OS_RecursiveMutexLock)

#define OS_RecursiveMutexUnlock \
	RAM_TBL_FUN(OS_Status (*)(OS_Mutex_t *mutex), OS_RecursiveMutexUnlock)

#define OS_MutexIsValid \
	RAM_TBL_FUN(int (*)(OS_Mutex_t *mutex), OS_MutexIsValid)

#define OS_MutexSetInvalid \
	RAM_TBL_FUN(void (*)(OS_Mutex_t *mutex), OS_MutexSetInvalid)

#define OS_MutexGetOwner \
	RAM_TBL_FUN(OS_ThreadHandle_t (*)(OS_Mutex_t *mutex), OS_MutexGetOwner)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ROM */

#endif /* _ROM_KERNEL_OS_OS_MUTEX_H_ */
