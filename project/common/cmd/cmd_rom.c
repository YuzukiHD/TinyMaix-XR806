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

#include "cmd_util.h"
#include "cmd_rom.h"
#include "pm/pm.h"
#include "driver/chip/hal_rtc.h"
#include "driver/chip/hal_wakeup.h"

unsigned int rom_get_version(char str[8]);
void rom_get_build_time(char str[64]);

/* drv rom ver
 */
static enum cmd_status cmd_rom_version_exec(char *cmd)
{
	char str[68];

	rom_get_version(str);
	printf("version:%s\n", str);
	rom_get_build_time(str);
	printf("build time:%s\n", str);

	return CMD_STATUS_OK;
}

/* drv rom bench <len> <cnt>
 *  drv rom bench 64 1000
 */
static enum cmd_status cmd_rom_bench_exec(char *cmd)
{
	int32_t cnt;
	uint32_t len, loop;
	uint8_t *buff = NULL;
	uint64_t t_start, t_use;

	cnt = cmd_sscanf(cmd, "%d %d", &len, &loop);
	if (cnt != 2) {
		CMD_ERR("invalid cmd %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	buff = malloc(len * 1024);
	if (buff == NULL) {
		CMD_ERR("no buffer\n");
		return CMD_STATUS_INVALID_ARG;
	}

	t_start = HAL_RTC_GetFreeRunTime();
	for (int i = 0; i < loop; i++) {
		memcpy(buff, buff + len / 2, len / 2);
	}
	t_use = HAL_RTC_GetFreeRunTime() - t_start;
	if (!t_use)
		t_use = 1;
	CMD_DBG("memcpy @ %p copy %d KB %d times use:%d us\n", memcpy, len/2,
	        loop, (uint32_t)t_use);

	free(buff);

	return CMD_STATUS_OK;
}

static void _test_float(float *X, float *Y, float *Z)
{
	float X1, Y1;
	float T, T2;

	T  = .499975;
	T2 = 2.0;

	X1 = *X;
	Y1 = *Y;
	X1 = T * (X1 + Y1);
	Y1 = T * (X1 + Y1);
	*X = X1;
	*Y = Y1;
	*Z  = (X1 + Y1) / T2;
}

/* drv rom float <cnt> <pm>
 *  drv rom float 100 0
 *  drv rom float 100 1
 */
static enum cmd_status cmd_rom_float_exec(char *cmd)
{
	int32_t cnt;
	uint32_t loop, pm;
	uint64_t t_start, t_use;
	float X, Y, Z;

	X = 1.0;
	Y = 1.0;
	Z = 1.0;

	cnt = cmd_sscanf(cmd, "%d %d", &loop, &pm);
	if (cnt != 2) {
		CMD_ERR("invalid cmd %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	t_start = HAL_RTC_GetFreeRunTime();
	for (int i = 0; i < loop; i++) {
		_test_float(&X, &Y, &Z); /* check objdump code */
		if (pm) {
			HAL_Wakeup_SetTimer_mS(1000);
			pm_enter_mode(PM_MODE_STANDBY);
		}
	}
	t_use = HAL_RTC_GetFreeRunTime() - t_start;
	if (!t_use)
		t_use = 1;
	CMD_DBG("float test result:%f %d times use:%d us\n",
	        Z, loop, (uint32_t)t_use);

	return CMD_STATUS_OK;
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PRCM_SetWlanSramShare(PRCM_WLAN_ShareSramType type);
void HAL_PRCM_SetBLESramShare(uint32_t en);

/* drv rom shsram <"user"> <type>
 *  drv rom shsram "ble" 0x0/0x1
 *  drv rom shsram "wlan" 0x0/0x1/0x3/0x7/0xf/0x1f
 */
static enum cmd_status cmd_rom_shsram_exec(char *cmd)
{
	int32_t cnt;
	char user[8];
	uint32_t type;

	cnt = cmd_sscanf(cmd, "%s 0x%x", user, &type);
	if (cnt != 2) {
		CMD_ERR("invalid cmd %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (!strcmp(user, "ble")) {
		HAL_PRCM_SetBLESramShare(type);
	} else if (!strcmp(user, "wlan")) {
		HAL_PRCM_SetWlanSramShare(type);
	}

	return CMD_STATUS_OK;
}
#endif

static const struct cmd_data g_rom_cmds[] = {
	{ "ver",        cmd_rom_version_exec },
	{ "bench",      cmd_rom_bench_exec },
	{ "float",      cmd_rom_float_exec },
#if (CONFIG_CHIP_ARCH_VER == 3)
	{ "shsram",     cmd_rom_shsram_exec },
#endif
};

enum cmd_status cmd_rom_exec(char *cmd)
{
	return cmd_exec(cmd, g_rom_cmds, cmd_nitems(g_rom_cmds));
}
