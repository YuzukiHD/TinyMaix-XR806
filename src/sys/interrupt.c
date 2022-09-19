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

#include <stdint.h>
#include "sys/interrupt.h"
#include "driver/chip/hal_cmsis.h"

#if (CONFIG_CHIP_ARCH_VER == 3 && \
     defined(CONFIG_OS_FREERTOS) && CONFIG_OS_FREERTOS_VER == 100201)

extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);

__sram_text
void arch_irq_disable_nested(void)
{
	vPortEnterCritical();
}

__sram_text
void arch_irq_enable_nested(void)
{
	vPortExitCritical();
}

#else /* (CONFIG_CHIP_ARCH_VER == 3 && ... CONFIG_OS_FREERTOS_VER == 100201) */

static uint32_t g_irq_cnt = 0;

__sram_text
void arch_irq_disable_nested(void)
{
	arch_irq_disable();
	++g_irq_cnt;

	/* Barriers are normally not required but do ensure the code is
	 * completely within the specified behaviour for the architecture.
	 */
	__DSB();
	__ISB();
}

__sram_text
void arch_irq_enable_nested(void)
{
	if (g_irq_cnt > 0) {
		--g_irq_cnt;
	}

	if (g_irq_cnt == 0) {
		arch_irq_enable();
	}
}

#endif /* (CONFIG_CHIP_ARCH_VER == 3 && ... CONFIG_OS_FREERTOS_VER == 100201) */

static uint32_t g_fiq_cnt = 0;

__sram_text
void arch_fiq_disable_nested(void)
{
	arch_fiq_disable();
	++g_fiq_cnt;

	/* Barriers are normally not required but do ensure the code is
	 * completely within the specified behaviour for the architecture.
	 */
	__DSB();
	__ISB();
}

__sram_text
void arch_fiq_enable_nested(void)
{
	if (g_fiq_cnt > 0) {
		--g_fiq_cnt;
	}

	if (g_fiq_cnt == 0) {
		arch_fiq_enable();
	}
}
