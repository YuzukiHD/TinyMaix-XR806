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
#include "driver/chip/system_chip.h"
#include "driver/chip/hal_wdg.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_gpio.h"
#include "hal_base.h"

#ifdef CONFIG_ROM

extern void __HAL_WDG_Reboot(void);
extern void __HAL_WDG_ResetCpu(WDG_ResetCpuMode mode);

void HAL_WDG_Reboot(void)
{
	HAL_DisableIRQ();
#if ((CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3))
	HAL_GPIO_PinMuxRebootReset();
#endif
#if (CONFIG_CHIP_ARCH_VER == 3)
	/* wdg soc reset without ble and rf */
	HAL_PRCM_ForceBLEReset();
	HAL_PRCM_ForceRFASReset();
	/* wdg soc reset will reset wlan but not prcm */
	HAL_PRCM_ForceSys3Reset();
	HAL_PRCM_EnableCPUWClk(0);
#endif /* CONFIG_CHIP_ARCH_VER == 3 */
	HAL_PRCM_SetCPUABootArg(PRCM_BOOT_FLAG(PRCM_BOOT_VAL_WDG_SYS_RST));
	__HAL_WDG_Reboot();
}

void HAL_WDG_ResetCpu(WDG_ResetCpuMode mode)
{
	HAL_DisableIRQ();
#if ((CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3))
	HAL_GPIO_PinMuxRebootReset();
#endif
	HAL_PRCM_SetCPUABootArg(PRCM_BOOT_FLAG(PRCM_BOOT_VAL_WDG_CPU_RST));
	__HAL_WDG_ResetCpu(mode);
}

#endif /* CONFIG_ROM */
