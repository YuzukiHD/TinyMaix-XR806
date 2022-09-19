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

#ifndef _DRIVER_CHIP_SYSTEM_CHIP_H_
#define _DRIVER_CHIP_SYSTEM_CHIP_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SYS_POWERON,
	SYS_WATCHDOG_CHIP_RST,
	SYS_WATCHDOG_CPU_RST,
	SYS_SLEEP,
	SYS_STANDBY,
	SYS_HIBERNATION,
	SYS_REBOOT,
	SYS_CPU_RST,
	SYS_NVIC_RST
} SystemStartupState;

#define SYSTEM_DEINIT_FLAG_RESET_CLK    HAL_BIT(0)

/**
 * @brief Initialize the chip system, including power, FPU setting, vector
 *        table location, clock, etc.
 * @return None
 */
void SystemInit(void);

/**
 * @brief DeInitialize the chip system by disabling and cleaning the system IRQ
 * @param flag Rest system clock if bit SYSTEM_DEINIT_FLAG_RESET_CLK is set in
 *             the flag
 * @return None
 */
void SystemDeInit(uint32_t flag);

/**
 * @brief Update system core (cpu) clock
 * @return None
 */
void SystemCoreClockUpdate(void);

#ifndef CONFIG_BOOTLOADER
void SysSetStartupState(SystemStartupState state);
SystemStartupState SysGetStartupState(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_SYSTEM_CHIP_H_ */
