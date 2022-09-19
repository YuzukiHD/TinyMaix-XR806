/*
 * Copyright (C) 2019 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
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

#include "string.h"
#include "cmd_util.h"
#include "sys/xr_debug.h"
#include "driver/chip/hal_scr.h"

static void scr_irq_callback(void *arg)
{

}

/*
 * drv scr start f=<freq>
 */
static enum cmd_status cmd_scr_start_exec(char *cmd)
{
	int cnt;
	uint32_t freq;
	SCR_InitParam g_scr_config;

	cnt = cmd_sscanf(cmd, "f=%u", &freq);
	if (cnt != 1) {
		return CMD_STATUS_INVALID_ARG;
	}

	if ((freq != 2000000) && (freq != 4000000) && (freq != 6000000)) {
		CMD_ERR("invalid frequence %u\n", freq);
		return CMD_STATUS_INVALID_ARG;
	}

	g_scr_config.mclk       = freq;
	g_scr_config.fifo_thr   = SCR_FIFO_DEPTH/2; //set the txfifo trigger level as half full
	g_scr_config.repeat_num = 3;                //The maximum number of reposts is 3
	g_scr_config.act_time   = 1;                //1*256 clock cycle
	g_scr_config.rst_time   = 1;                //1*256 clock cycle
	g_scr_config.atr_time   = (35000>>8);       //400~40000, After RST is pulled up,IO should make the time limit of ATR
	g_scr_config.guard_time = 2;                /* 2*ETUs, GT = 12etu + R * N/f, Default extra guard time is 0 */
	g_scr_config.char_limit = 9600;             //1024 * (10 + g_scr_config.guard_time);

	g_scr_config.IRQCallback = scr_irq_callback;
	g_scr_config.arg = NULL;

	HAL_SCR_Init(&g_scr_config);

	while (1) {
		HAL_SCR_Process(&g_scr_config);
	}

	return CMD_STATUS_OK;
}

/*
 * drv scr stop
 */
static enum cmd_status cmd_scr_stop_exec(char *cmd)
{
	HAL_SCR_DeInit();

	return CMD_STATUS_OK;
}

static struct cmd_data g_scr_cmds[] = {
	{ "start",       cmd_scr_start_exec },
	{ "stop",        cmd_scr_stop_exec },
};

enum cmd_status cmd_scr_exec(char *cmd)
{
	return cmd_exec(cmd, g_scr_cmds, cmd_nitems(g_scr_cmds));
}
