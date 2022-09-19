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

#ifndef _ROM_KERNEL_OS_OS_THREAD_H_
#define _ROM_KERNEL_OS_OS_THREAD_H_

#include "kernel/os/os_thread.h"

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OS_ThreadCreate \
	RAM_TBL_FUN(OS_Status (*)(OS_Thread_t *thread, const char *name, \
	                          OS_ThreadEntry_t entry, void *arg, \
	                          OS_Priority priority, uint32_t stackSize), \
	            OS_ThreadCreate)

#define OS_ThreadDelete \
	RAM_TBL_FUN(OS_Status (*)(OS_Thread_t *thread), OS_ThreadDelete)

#define OS_ThreadSleep \
	RAM_TBL_FUN(void (*)(OS_Time_t msec), OS_ThreadSleep)

#define OS_ThreadYield \
	RAM_TBL_FUN(void (*)(void), OS_ThreadYield)

#define OS_ThreadGetCurrentHandle \
	RAM_TBL_FUN(OS_ThreadHandle_t (*)(void), OS_ThreadGetCurrentHandle)

#define OS_ThreadStartScheduler \
	RAM_TBL_FUN(void (*)(void), OS_ThreadStartScheduler)

#define OS_ThreadSuspendScheduler \
	RAM_TBL_FUN(void (*)(void), OS_ThreadSuspendScheduler)

#define OS_ThreadResumeScheduler \
	RAM_TBL_FUN(void (*)(void), OS_ThreadResumeScheduler)

#define OS_ThreadIsSchedulerRunning \
	RAM_TBL_FUN(int (*)(void), OS_ThreadIsSchedulerRunning)

#define OS_ThreadGetStackMinFreeSize \
	RAM_TBL_FUN(uint32_t (*)(OS_Thread_t *thread), OS_ThreadGetStackMinFreeSize)

#define OS_ThreadList \
	RAM_TBL_FUN(void (*)(void), OS_ThreadList)

#define OS_ThreadIsValid \
	RAM_TBL_FUN(int (*)(OS_Thread_t *thread), OS_ThreadIsValid)

#define OS_ThreadSetInvalid \
	RAM_TBL_FUN(void (*)(OS_Thread_t *thread), OS_ThreadSetInvalid)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ROM */

#endif /* _ROM_KERNEL_OS_OS_THREAD_H_ */
