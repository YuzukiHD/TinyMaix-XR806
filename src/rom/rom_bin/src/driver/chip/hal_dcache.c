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

#include "rom/sys/io.h"
//#include "rom/sys/xr_debug.h"
#include "rom/driver/chip/hal_def.h"
#include "rom/driver/chip/hal_util.h"
#include "rom/driver/chip/hal_ccm.h"
#include "rom/driver/chip/hal_dcache.h"
#include "rom/driver/chip/hal_xip.h"

#include "rom/driver/chip/private/hal_debug.h"
#include "rom/driver/chip/private/hal_os.h"
#include "rom/pm/pm.h"

static uint8_t __dcache_init;

//#define DCACHE_DBG

#ifdef DCACHE_DBG
#define DCACHE_DUMP_REGS() \
    { \
        printf("dcache base addr:[0x%08x]\n", (uint32_t)(DCACHE_CTRL)); \
        printf("DCACHE_COM_CFG:0x%08x\n", DCACHE_CTRL->DCACHE_COM_CFG);\
        printf("MISS_COUNT_H:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_H);\
        printf("MISS_COUNT_L:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_L);\
        printf("HIT_COUNT_H:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_H);\
        printf("HIT_COUNT_L:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_L);\
        printf("DCACHE_STA:\t0x%08x\n", DCACHE_CTRL->DCACHE_STA);\
        printf("CLEAN_FLUSH_SADDR:0x%08x\n", DCACHE_CTRL->CLEAN_FLUSH_SADDR);\
        printf("CLEAN_FLUSH_LEN:0x%08x\n", DCACHE_CTRL->CLEAN_FLUSH_LEN);\
    }
#else
#define DCACHE_DUMP_REGS()
#endif

static uint8_t gCacheWTIdxUsed = 0;
static uint32_t _dcache_addr[DCACHE_WT_ADDR_MAX * 2];

static __INLINE void HAL_Dcache_WaitIdle(void)
{
	while (DCACHE_CTRL->DCACHE_STA)
		;
	return;
}

#define RANGEOF_CACHEBYPASS(addr, len, start, end) (((addr) >= (start)) && (((addr)+(len)) <= (end)))

int32_t HAL_Dcache_IsBypass(uint32_t addr, uint32_t len)
{
	for (int i = 0; i < DCACHE_WT_ADDR_MAX; i++) {
		if (RANGEOF_CACHEBYPASS(addr, len, _dcache_addr[i * 2], _dcache_addr[i * 2 + 1] | 0xF)) {
			return 1;
		}
	}
	return 0;
}

int32_t HAL_Dcache_IsPSramCacheable(uint32_t addr, uint32_t len)
{
	return (__dcache_init && (RANGEOF_CACHEBYPASS(addr, len, PSRAM_START_ADDR, PSRAM_END_ADDR)) &&
	        !HAL_Dcache_IsBypass(addr, len));
}


/* only addr >= PSRAM_START_ADDR and addr + len <= PSRAM_END_ADDR and
 *      [addr, addr + len) not in any of write through area, will return 1.
 * otherwise reutn 0.
 */
int32_t HAL_Dcache_IsCacheable(uint32_t addr, uint32_t len)
{
	return (__dcache_init && (RANGEOF_CACHEBYPASS(addr, len, PSRAM_START_ADDR, PSRAM_END_ADDR)) &&
	        (RANGEOF_CACHEBYPASS(addr, len, FLASH_XIP_START_ADDR, FLASH_XIP_START_ADDR)) &&
	        !HAL_Dcache_IsBypass(addr, len));
}

void HAL_Dcache_Flush(uint32_t sadd, uint32_t len)
{
	unsigned long flag;
	HAL_ASSERT_PARAM(len > 0);
	HAL_ASSERT_PARAM(!(sadd & 0x0F));
	HAL_ASSERT_PARAM(!(len & 0x0F));

	if (!__dcache_init)
		return;

	flag = HAL_EnterCriticalSection();
	HAL_Dcache_WaitIdle();
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = 0;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = 0;
	HAL_ExitCriticalSection(flag);
}

void HAL_Dcache_FlushAll(void)
{
	unsigned long flag;

	if (!__dcache_init)
		return;

	flag = HAL_EnterCriticalSection();
	HAL_Dcache_WaitIdle();
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, (DCACHE_FLUSH_START_MASK | DCACHE_FLUSH_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_CLEAN_START_MASK);
	HAL_ExitCriticalSection(flag);
}

void HAL_Dcache_CleanAll(void)
{
	unsigned long flag;

	if (!__dcache_init)
		return;

	flag = HAL_EnterCriticalSection();
	HAL_Dcache_WaitIdle();
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_CLEAN_START_MASK | DCACHE_FLUSH_CLEAN_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_CLEAN_START_MASK)
		;
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_CLEAN_START_MASK);
	HAL_ExitCriticalSection(flag);
}

void HAL_Dcache_Clean(uint32_t sadd, uint32_t len)
{
	unsigned long flag;

	HAL_ASSERT_PARAM(len > 0);
	HAL_ASSERT_PARAM((sadd >= PSRAM_START_ADDR) && ((sadd + len) <= PSRAM_END_ADDR));
	HAL_ASSERT_PARAM(!(sadd & 0x0F));
	HAL_ASSERT_PARAM(!(len & 0x0F));

	if (!__dcache_init)
		return;

	flag = HAL_EnterCriticalSection();
	HAL_Dcache_WaitIdle();
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_CLEAN_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_CLEAN_START_MASK)
		;
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = 0;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = 0;
	HAL_ExitCriticalSection(flag);
}

int32_t HAL_Dcache_Request_WriteThroughIndex(void)
{
	int32_t idx = -1;
	unsigned long flag;

	if (!__dcache_init)
		return -1;

	flag = HAL_EnterCriticalSection();
	for (uint8_t i = 0; i < ARRAY_SIZE(DCACHE_CTRL->WT_ADDR); i++) {
		if (HAL_GET_BIT(gCacheWTIdxUsed, HAL_BIT(i)) == 0) {
			HAL_SET_BIT(gCacheWTIdxUsed, HAL_BIT(i));
			idx = i;
			break;
		}
	}
	HAL_ExitCriticalSection(flag);

	return idx;
}

int32_t HAL_Dcache_Release_WriteThroughIndex(int32_t index)
{
	unsigned long flags;

	if (!__dcache_init || index >= ARRAY_SIZE(DCACHE_CTRL->WT_ADDR))
		return -1;

	flags = HAL_EnterCriticalSection();
	HAL_CLR_BIT(gCacheWTIdxUsed, HAL_BIT(index));
	_dcache_addr[index * 2 + 1] = 0;
	_dcache_addr[index * 2] = 0;
	DCACHE_CTRL->WT_ADDR[index].END_ADDR = 0;
	DCACHE_CTRL->WT_ADDR[index].START_ADDR = 0;

	HAL_ExitCriticalSection(flags);

	return 0;
}

int32_t HAL_Dcache_Config_WriteThrough(int32_t index, uint32_t sadd, uint32_t eadd)
{
	unsigned long flag;
	HAL_ASSERT_PARAM(!(sadd & 0x0F));
	HAL_ASSERT_PARAM(!(eadd & 0x0F));
	HAL_ASSERT_PARAM(eadd > sadd);

	if (!__dcache_init || index >= ARRAY_SIZE(DCACHE_CTRL->WT_ADDR))
		return -1;

	flag = HAL_EnterCriticalSection();
	if (RANGEOF_CACHEBYPASS(sadd, eadd - sadd, PSRAM_START_ADDR, PSRAM_END_ADDR)) {
		HAL_Dcache_Clean(sadd, eadd - sadd);
	}
	DCACHE_CTRL->WT_ADDR[index].START_ADDR = sadd;
	DCACHE_CTRL->WT_ADDR[index].END_ADDR = eadd;
	_dcache_addr[index * 2] = sadd;
	_dcache_addr[index * 2 + 1] = eadd;
	HAL_ExitCriticalSection(flag);

	return 0;
}

void HAL_Dcache_DumpMissHit(void)
{
	printf("MISS_COUNT_H:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_H);
	printf("MISS_COUNT_L:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_L);
	printf("HIT_COUNT_H:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_H);
	printf("HIT_COUNT_L:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_L);
}

void HAL_Dcache_DumpCbusCnt(void)
{
	printf("CBUS_WRITE_CNT:\t0x%08x\n", DCACHE_CTRL->CBUS_WRITE_CNT);
	printf("CBUS_READ_CNT:\t0x%08x\n", DCACHE_CTRL->CBUS_READ_CNT);
}

static void HAL_Dcache_Config(DCache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DCACHE);
	HAL_UDelay(100);
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ENABLE_MASK);
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_COUNTER_EN_MASK);

	if (cfg->vc_en)
		HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
		               DCACHE_EN_VICTIM_MASK, DCACHE_EN_VICTIM);
	else
		HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_EN_VICTIM);

	if (cfg->wrap_en)
		HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
		               DCACHE_EN_RD_WRAP_MASK, DCACHE_EN_RD_WRAP);
	else
		HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_EN_RD_WRAP);

	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ASSOCIATE_MODE_MASK,
	               (cfg->way_mode << DCACHE_ASSOCIATE_MODE_SHIFT));

#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               DCACHE_MIXED_MODE_DCACHE | DCACHE_MIXED_IDBUS_EN,
	               cfg->mixed_mode);
#endif
	__dcache_init = 1;

	HAL_Dcache_FlushAll();

	DCACHE_DUMP_REGS();
}

static void HAL_Dcache_DeConfig(void)
{
	__dcache_init = 0;
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_COUNTER_EN_MASK);
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ENABLE_MASK);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_DCACHE);
}

#ifdef CONFIG_PM
static DCache_Config _dcache_cfg;

static int dcache_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Dcache_CleanAll();
		HAL_Dcache_DeConfig();
		break;
	default:
		break;
	}
	return 0;
}

static int dcache_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Dcache_Config(&_dcache_cfg);
		for (int i = 0; i < DCACHE_WT_ADDR_MAX; i++) {
			DCACHE_CTRL->WT_ADDR[i].START_ADDR = _dcache_addr[i * 2];
			DCACHE_CTRL->WT_ADDR[i].END_ADDR = _dcache_addr[i * 2 + 1];
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct soc_device_driver dcache_drv = {
	.name = "dcache",
	.suspend_noirq = dcache_suspend,
	.resume_noirq = dcache_resume,
};

static struct soc_device dcache_dev = {
	.name = "dcache",
	.driver = &dcache_drv,
};

#define DCACHE_DEV (&dcache_dev)

#endif/* CONFIG_PM */

void HAL_Dcache_Init(DCache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);
	gCacheWTIdxUsed = 0;
	HAL_Dcache_Config(cfg);

#ifdef CONFIG_PM
	HAL_Memcpy(&_dcache_cfg, cfg, sizeof(DCache_Config));
	pm_register_ops(DCACHE_DEV);
#endif
}

void HAL_Dcache_DeInit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(DCACHE_DEV);
	HAL_Memset(&_dcache_cfg, 0, sizeof(DCache_Config));
#endif
	HAL_Dcache_DeConfig();
}
