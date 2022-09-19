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

#ifndef _ROM_KERNEL_OS_OS_TIMER_H_
#define _ROM_KERNEL_OS_OS_TIMER_H_

#include "kernel/os/os_timer.h"

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OS_TimerCreate \
	RAM_TBL_FUN(OS_Status (*)(OS_Timer_t *timer, OS_TimerType type, \
	                          OS_TimerCallback_t cb, void *arg, \
	                          OS_Time_t periodMS), \
	            OS_TimerCreate)

#define OS_TimerDelete \
	RAM_TBL_FUN(OS_Status (*)(OS_Timer_t *timer), OS_TimerDelete)

#define OS_TimerStart \
	RAM_TBL_FUN(OS_Status (*)(OS_Timer_t *timer), OS_TimerStart)

#define OS_TimerChangePeriod \
	RAM_TBL_FUN(OS_Status (*)(OS_Timer_t *timer, OS_Time_t periodMS), OS_TimerChangePeriod)

#define OS_TimerStop \
	RAM_TBL_FUN(OS_Status (*)(OS_Timer_t *timer), OS_TimerStop)

#define OS_TimerIsActive \
	RAM_TBL_FUN(int (*)(OS_Timer_t *timer), OS_TimerIsActive)

#define OS_TimerIsValid \
	RAM_TBL_FUN(int (*)(OS_Timer_t *timer), OS_TimerIsValid)

#define OS_TimerSetInvalid \
	RAM_TBL_FUN(void (*)(OS_Timer_t *timer), OS_TimerSetInvalid)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ROM */

#endif /* _ROM_KERNEL_OS_OS_TIMER_H_ */
