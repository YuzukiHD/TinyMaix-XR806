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

#include <string.h>
#include <stdio.h>

#include "sys/xr_debug.h"
#include "sys/io.h"
#include "pm/pm.h"

#include "../hal_base.h"
#include "driver/chip/hal_dma.h"

#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "_psram.h"

#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"

/* #define PSRAM_SQPI_USE_1LINE */

static int32_t psram_enter_quad_mode(struct psram_chip *chip)
{
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.cmd.opcode = SQ_Enter_Quad_Mode;
	mrq.cmd.dummy = 0;
	mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	chip->buswidth = 4;

	return 0;
}

static int32_t psram_exit_quad_mode(struct psram_chip *chip)
{
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.cmd.opcode = SQ_Exit_Quad_Mode;
	mrq.cmd.dummy = 0;
	mrq.cmd.busconfig = PSRAMC_BUS_CMD_4BIT;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_sqpi_suspend(struct psram_chip *chip)
{
	return 0;
}

int32_t psram_sqpi_resume(struct psram_chip *chip)
{
	HAL_PsramCtrl_CacheCfg(chip->ctrl, 0);
#if (CONFIG_CHIP_ARCH_VER == 3)
	HAL_Flashc_CBUS_Dma_Enable(1);
#endif
	return 0;
}

/* define for tCPH
 * freq./tCPH, WL, Write[Min tCPH, Max BL], Read[Min tCPH, Max BL]
 *   133MHz,   0,          4,        656,           2,        2
 *   166MHz,   0,          4,        768,           2,        2
 *   200MHz,   2,          4,        768,           2,        2
 *   200MHz,   2,          6,        984,           2,        2
 *   233HMz,   2,          8,       1024,           2,        2
 */

/* define for Latency
 * MR0[5]=0b(Variable Latency)
 * MR0[4:2], Latency, Max push out, Standard, Extended
 *   010        2          4           66        66
 *   011        3          6          109       109
 *   100        4(default) 8          133       133
 *   101        5         10          166       166
 *   110        6         12        200/233    200/233
 * MR0[5]=1b(Fixed Latency)
 * MR0[4:2], Latency, Standard, Extended
 *   010        4           66        66
 *   011        6          109       109
 *   100        8(default) 133       133
 *   101       10          166       166
 *   110       12        200/233    200/233
 */

int32_t psram_sqpi_init(struct psram_chip *chip, struct psram_ctrl *ctrl)
{
	uint32_t psram_init;

	chip->buswidth = 1;
	chip->wrap_len = PSRAMC_PAGE_SZ_64;

#if ((defined CONFIG_TZ_PSRAM) && (!defined CONFIG_BOOTLOADER))
	psram_init = TZ_PSRAMBinExist_NSC();
#else
	psram_init = 0;
#endif
	if (!psram_init) {
		psram_exit_quad_mode(chip); /* switch to 1 line mode every time */
		psram_sw_reset(chip, 0);
		psram_sw_reset(chip, 1);
		psram_set_driver_strength(chip, DRV_STR_50_OHM);
		Psram_Read_Info(chip);
		if (chip->kgd != 0x5D) {
			PR_ERR("SQPI BAD kgd:0x%x\n", chip->kgd);
			return -1;
		}
	} else {
		chip->kgd = 0x5D;
	}

	HAL_PsramCtrl_MaxCE_LowCyc(chip->ctrl, chip->freq);

#ifdef PSRAM_SQPI_USE_1LINE
#if 0
	psram_set_wrap_dbt(chip, S_RST); /* not use wrap mode */
#endif
	psram_idbus_op_cmd(chip, SQPI_BUSWIDTH1_READ);
	psram_idbus_op_cmd(chip, SQPI_BUSWIDTH1_WRITE);
#else
	psram_enter_quad_mode(chip);
#if 0
	psram_set_wrap_dbt(chip, Q_RST); /* not use wrap mode */
	printf("disable dbt\n");
#endif
	psram_idbus_op_cmd(chip, SQPI_BUSWIDTH4_READ); /*Q_FAST_READ_QUAD*/
	psram_idbus_op_cmd(chip, SQPI_BUSWIDTH4_WRITE);
#endif

	HAL_PsramCtrl_CacheCfg(chip->ctrl, 0);
#if (CONFIG_CHIP_ARCH_VER == 3)
	HAL_Flashc_CBUS_Dma_Enable(1);
#endif

	chip->name = "SQPI";
	chip->capacity = 2 * 1024 * 1024;

	HAL_UDelay(1000);
	/* sqpi no need do DQS calibration */

	return 0;
}
