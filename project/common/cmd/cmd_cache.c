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

#ifdef CONFIG_CACHE
#include <stdlib.h>

#include "sys/io.h"
#include "sys/xr_debug.h"
#include "cmd_util.h"
#include "cmd_psram.h"

#include "driver/chip/hal_rtc.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_xip.h"
#include "sys/sys_heap.h"

/*
 * drv cache flush <0xadd> <len>
 */
static enum cmd_status cmd_cache_flush_exec(char *cmd)
{
	uint32_t add, len;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "0x%x %d", &add, &len);
	if (cnt != 2) {
		printf("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	HAL_Dcache_Flush(add, len);

	return CMD_STATUS_OK;
}

/*
 * drv cache clean <0xadd> <len>
 */
static enum cmd_status cmd_cache_clean_exec(char *cmd)
{
	uint32_t add, len;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "0x%x %d", &add, &len);
	if (cnt != 2) {
		printf("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	HAL_Dcache_Clean(add, len);

	return CMD_STATUS_OK;
}

#ifdef CONFIG_PSRAM

/*
 * drv cache hitmis <mode> <type> <len>
 *   mode: 0: sram->sram 1: sram -> cache, 2: cache -> sram, 3: cache -> cache
 *   type: 0: not clean + not flush 1:not clean src, 2: not flush dst, 3: clean + flush
 *   len: test lenght
 * eg. cache hitmis 1 1 1024
 */
static enum cmd_status cmd_cache_hitmis_exec(char *cmd)
{
	uint32_t mode, type, len;
	uint32_t cnt;
	uint32_t *addS = NULL, *addD = NULL, *buff;
	uint32_t s_cache, d_cache;
	//uint32_t wtIdx = 0;

	cnt = cmd_sscanf(cmd, "%d %d %d", &mode, &type, &len);
	if (cnt != 3 || mode > 3) {
		printf("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	if (mode == 0) {
		addS = malloc(len);
		addD = malloc(len);
	} else if (mode == 1) {
		addS = malloc(len);
		addD = psram_malloc(len);
	} else if (mode == 2) {
		addS = psram_malloc(len);
		addD = malloc(len);
	} else if (mode == 3) {
		addS = psram_malloc(len);
		addD = psram_malloc(len);
	}
	s_cache = HAL_Dcache_IsPSramCacheable((uint32_t)addS, len);
	d_cache = HAL_Dcache_IsPSramCacheable((uint32_t)addD, len);
	if ((mode > 0) && (!s_cache) && (!d_cache)) {
		printf("should not set cache heap write through!\n");
		goto out;
	}
	//wtIdx = HAL_Dcache_Request_WriteThroughIndex();
	//HAL_Dcache_Config_WriteThrough(wtIdx, (uint32_t)addD, (uint32_t)addD + len);
	buff = malloc(64);
	printf("test Src add:%p Dst add:%p buf:%p\n", addS, addD, buff);
	memset(addS, 0xA5, len);
	printf("base MissHit\n");
	HAL_Dcache_DumpMissHit();
	psram_rw_op[1](0, (uint32_t)addS, (uint8_t *)buff, 64, NULL); /* DMA */
	print_hex_dump("", DUMP_PREFIX_ADDRESS, 32, 1, buff, 64, 0);
	memset(addD, 0x5A, len);
	if (type > 1) {
		if (s_cache)
			HAL_Dcache_Clean((uint32_t)addS, len);
		printf("MissHit after clean, should more\n");
		HAL_Dcache_DumpMissHit();
	}
	psram_rw_op[1](0, (uint32_t)addD, (uint8_t *)buff, 64, NULL); /* DMA */
	print_hex_dump("", DUMP_PREFIX_ADDRESS, 32, 1, buff, 64, 0);
	psram_rw_op[1](0, (uint32_t)addS, (uint8_t *)addD, len, NULL); /* DMA */
	printf("base MissHit\n");
	HAL_Dcache_DumpMissHit();
	if (type & 0x1) {
		if (d_cache)
			HAL_Dcache_Flush((uint32_t)addD, len);
		printf("MissHit after flush, should same\n");
		HAL_Dcache_DumpMissHit();
	}
	free(buff);
	buff = addD;
	print_hex_dump("", DUMP_PREFIX_ADDRESS, 32, 1, addS, 64, 0);
	print_hex_dump("", DUMP_PREFIX_ADDRESS, 32, 1, addD, 64, 0);
	for (int i = 0; i < len/4; i++, buff++) {
		if (*buff != 0xA5A5A5A5) {
			printf("err at idx:%d [%p]:%x\n", i, buff, *buff);
			print_hex_dump("", DUMP_PREFIX_ADDRESS, 32, 1, buff, 64, 1);
			break;
		}
	}
out:
	if (mode == 0) {
		 free(addS);
		 free(addD);
	} else if (mode == 1) {
		free(addS);
		psram_free(addD);
	} else if (mode == 2) {
		psram_free(addS);
		free(addD);
	} else if (mode == 3) {
		psram_free(addS);
		psram_free(addD);
	}
	//HAL_Dcache_Release_WriteThroughIndex(wtIdx);

	return CMD_STATUS_OK;
}
#endif

#ifndef CONFIG_BLE
/*
 * drv cache sharesram
 */
static enum cmd_status cmd_cache_sharesram_exec(char *cmd)
{
	DCache_Config dcache_cfg = { 0 };

#if (!(defined CONFIG_CACHE_SIZE_32K))
	printf("should config cache size to 32K!\n");
	return CMD_STATUS_FAIL;
#endif

	dcache_cfg.vc_en = 1;
	dcache_cfg.wrap_en = 1;

	arch_irq_disable();
	HAL_Dcache_CleanAll();
	HAL_Dcache_DeInit();
	HAL_PRCM_SetBLESramShare(1);
	dcache_cfg.way_mode = DCACHE_ASSOCIATE_MODE_TWO_WAY;
	HAL_Dcache_Init(&dcache_cfg);
	arch_irq_enable();

	HAL_Dcache_DumpMissHit();
	OS_MSleep(5000);
	HAL_Dcache_DumpMissHit();

	arch_irq_disable();
	HAL_Dcache_CleanAll();
	HAL_Dcache_DeInit();
	HAL_PRCM_SetBLESramShare(0);
	dcache_cfg.way_mode = DCACHE_ASSOCIATE_MODE_FOUR_WAY;
	HAL_Dcache_Init(&dcache_cfg);
	arch_irq_enable();

	return CMD_STATUS_OK;
}
#endif

/*
 * drv cache config m=<mode> v=<vc> w=<wrap>
 */
static enum cmd_status cmd_cache_config_exec(char *cmd)
{
	DCache_Config dcache_cfg = { 0 };
	uint32_t mode, vc, wrap;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "m=%d v=%d w=%d", &mode, &vc, &wrap);
	if (cnt != 3 || mode > 3) {
		printf("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (mode > DCACHE_ASSOCIATE_MODE_FOUR_WAY) {
		printf("invalid mode:%d, should less than %d\n", mode, DCACHE_ASSOCIATE_MODE_FOUR_WAY);
		return CMD_STATUS_INVALID_ARG;
	}

	arch_irq_disable();
	HAL_Dcache_CleanAll();
	HAL_Dcache_DeInit();
	dcache_cfg.vc_en = vc;
	dcache_cfg.wrap_en = wrap;
	dcache_cfg.way_mode = mode;
	HAL_Dcache_Init(&dcache_cfg);
	arch_irq_enable();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_cache_cmds[] = {
	{ "flush",      cmd_cache_flush_exec },
	{ "clean",      cmd_cache_clean_exec },
#ifdef CONFIG_PSRAM
	{ "hitmis",     cmd_cache_hitmis_exec },
#endif
#ifndef CONFIG_BLE
	{ "sharesram",  cmd_cache_sharesram_exec },
#endif
	{ "config",     cmd_cache_config_exec },
};

enum cmd_status cmd_cache_exec(char *cmd)
{
	return cmd_exec(cmd, g_cache_cmds, cmd_nitems(g_cache_cmds));
}
#endif
