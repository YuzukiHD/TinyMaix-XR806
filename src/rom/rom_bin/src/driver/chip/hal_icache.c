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

#include "hal_base.h"
#include "rom/driver/chip/hal_icache.h"
#include "rom/pm/pm.h"

#include "sys/xr_debug.h"

#if 1
#define FCC_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#define FCC_ERROR(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#else
#define FCC_DEBUG(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#define FCC_ERROR(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#endif

#if 0 //(CONFIG_CHIP_ARCH_VER == 2)

#define ICACHE_REG_ALL() \
     { \
	 FCC_DEBUG("icache reg: base addr   0x%8x\n.", (uint32_t)&(ICACHE_CTRL->CACHE_COM_CFG)); \
	 FCC_DEBUG("CACHE_COM_CFG;	    0x%8x\n.", ICACHE_CTRL->CACHE_COM_CFG);\
	 FCC_DEBUG("MISS_COUNT_H;	    0x%8x\n.", ICACHE_CTRL->MISS_COUNT_H);\
	 FCC_DEBUG("MISS_COUNT_L;	    0x%8x\n.", ICACHE_CTRL->MISS_COUNT_L);\
	 FCC_DEBUG("HIT_COUNT_H;	    0x%8x\n.", ICACHE_CTRL->HIT_COUNT_H);\
	 FCC_DEBUG("HIT_COUNT_L;	    0x%8x\n.", ICACHE_CTRL->HIT_COUNT_L);\
	 FCC_DEBUG("CACHE_STA;		    0x%8x\n.", ICACHE_CTRL->CACHE_STA);\
	 FCC_DEBUG("INSTR_WAIT_H;	    0x%8x\n.", ICACHE_CTRL->INSTR_WAIT_H);\
	 FCC_DEBUG("INSTR_WAIT_L;	    0x%8x\n.", ICACHE_CTRL->INSTR_WAIT_L);\
     }
static inline void Icache_Reg_All(void)
{
	//ICACHE_REG_ALL();
}

HAL_Status HAL_ICache_EnablePrefetch(ICache_PrefetchConfig *cfg)
{

	return HAL_OK;
}

HAL_Status HAL_ICache_DisablePrefetch(void)
{

	return HAL_OK;
}

void HAL_ICache_Flush(void)
{
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_FLUSH_ALL_MASK);
	while (HAL_GET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_FLUSH_ALL_MASK))
		;
}

HAL_Status HAL_ICache_Config(ICache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	FCC_DEBUG("%s: vc:%d wrap:%d mode:%d bypass:%d\n",
		  __func__, cfg->vc_en, cfg->wrap_en, cfg->way_mode, cfg->bypass);

	/* CCMU Enable */
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_ICACHE);
	HAL_UDelay(20);

	/* cache hm count enable */
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_COUNTER_EN_MASK);

	HAL_ICache_Flush();

	if (cfg->vc_en) { /* cache victim enable */
		HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_VICTIM_CACHE_MASK);
	} else { /* cache victim disable */
		HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_VICTIM_CACHE_MASK);
	}

	if (cfg->wrap_en) { /* cache wrap enable */
		HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_ICACHE_WRAP_MASK);
	} else { /* cache wrap disable */
		HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_ICACHE_WRAP_MASK);
	}

	/* cache set asso mode */
	HAL_MODIFY_REG(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ASSOCIATE_MODE_MASK,
	               (cfg->way_mode << ICACHE_ASSOCIATE_MODE_SHIFT));

	/* cache enable */
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ENABLE_MASK);

	HAL_ICache_Flush();
	Icache_Reg_All();

	return HAL_OK;
}

HAL_Status HAL_ICache_DeConfig(void)
{
	/* cache hm count disable */
	HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_COUNTER_EN_MASK);

	/* cache disable */
	HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ENABLE_MASK);

	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_ICACHE);

	return HAL_OK;
}

#ifdef CONFIG_PM
static ICache_Config _icache_cfg;

static int icache_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_ICache_Flush();
		HAL_ICache_DeConfig();
		break;
	default:
		break;
	}
	return 0;
}

static int icache_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_ICache_Config(&_icache_cfg);
		break;
	default:
		break;
	}
	return 0;
}

static const struct soc_device_driver icache_drv = {
	.name = "icache",
	.suspend_noirq = icache_suspend,
	.resume_noirq = icache_resume,
};

static struct soc_device icache_dev = {
	.name = "icache",
	.driver = &icache_drv,
};

#define ICACHE_DEV (&icache_dev)

#endif/*CONFIG_PM*/

HAL_Status HAL_ICache_Init(ICache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	HAL_ICache_Config(cfg);

#ifdef CONFIG_PM
	HAL_Memcpy(&_icache_cfg, cfg, sizeof(ICache_Config));
	pm_register_ops(ICACHE_DEV);
#endif

	return HAL_OK;
}

HAL_Status HAL_ICache_Deinit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(ICACHE_DEV);
	HAL_Memset(&_icache_cfg, 0, sizeof(ICache_Config));
#endif
	HAL_ICache_DeConfig();

	return HAL_OK;
}

#endif /* CONFIG_CHIP_XXX */
