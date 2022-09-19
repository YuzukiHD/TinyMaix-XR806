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

#include <stdio.h>
#include "compiler.h"
#include "version.h"
#include "pm/pm.h"
#include "image/image.h"

#include "driver/chip/system_chip.h"
#include "driver/chip/hal_prcm.h"
#include "common/board/board.h"
#include "sysinfo.h"
#include "sdd/sdd.h"
#if PRJCONF_NET_EN
#include "net_ctrl.h"
#endif
#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "bt_ctrl.h"
#endif
#endif
#include "fs_ctrl.h"
#include "sys_ctrl/sys_ctrl.h"
#include "fwk_debug.h"

#if PRJCONF_AUDIO_SNDCARD_EN
#include "audio/manager/audio_manager.h"
#include "audio/pcm/audio_pcm.h"
#if PRJCONF_AUDIO_CTRL_EN
#include "audio_ctrl.h"
#endif
#endif
#if PRJCONF_CONSOLE_EN
#include "console/console.h"
#include "command.h"
#endif
//#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
#include "efpg/efpg.h"
//#endif
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#ifdef CONFIG_PSRAM
#include "psram.h"
#endif

#if (defined(CONFIG_XIP) || defined(CONFIG_PSRAM))
#include "driver/chip/hal_xip.h"
#endif

#ifdef CONFIG_XPLAYER
#include "cedarx/cedarx.h"
#endif

#if (CONFIG_CHIP_ARCH_VER > 1)
#include "driver/chip/hal_trng.h"
#endif

#if PRJCONF_BLE_EN
#include "blec.h"
#endif
#include "image/flash.h"
#ifdef CONFIG_WAKEUP_PIN
#include "driver/chip/hal_wkpin.h"
#endif

#define PLATFORM_SHOW_DEBUG_INFO    0    /* for internal debug only */

static void platform_show_info(void)
{
	uint8_t dbg_en = 0;
	uint32_t buf[4] = {0};

#if PLATFORM_SHOW_DEBUG_INFO
	dbg_en = 1;
#endif /* PLATFORM_SHOW_INFO */

	extern uint8_t __text_start__[];
	extern uint8_t __text_end__[];
	extern uint8_t __etext[];
	extern uint8_t __data_start__[];
	extern uint8_t __data_end__[];
	extern uint8_t __bss_start__[];
	extern uint8_t __bss_end__[];
	extern uint8_t __heap_start__[];
	extern uint8_t __heap_end__[];
	extern uint8_t __end__[];
	extern uint8_t end[];
	extern uint8_t __HeapLimit[];
	extern uint8_t __StackLimit[];
	extern uint8_t __StackTop[];
	extern uint8_t __stack[];
	extern uint8_t _estack[];
#ifdef CONFIG_ROM
	extern uint8_t __ram_table_lma_start__[];
	extern uint8_t __ram_table_lma_end__[];
#endif

#if PRJCONF_NET_EN
	uint8_t mac_addr[6] = {0};
	struct sysinfo *sys_info = NULL;
#endif

	if (efpg_read(EFPG_FIELD_CHIPID, (uint8_t *)buf) != 0) {
		FWK_LOG(1, "efpg read chipinfo failed!\n");
	}

	FWK_LOG(1, "\nplatform information ===============================================\n");
	FWK_LOG(1, "XR806 SDK "SDK_VERSION_STR" "SDK_STAGE_STR" "__DATE__" "__TIME__" %08X\n\n",
	           buf[0]);

	FWK_LOG(dbg_en, "__text_start__ %p\n", __text_start__);
	FWK_LOG(dbg_en, "__text_end__   %p\n", __text_end__);
	FWK_LOG(dbg_en, "__etext        %p\n", __etext);
	FWK_LOG(dbg_en, "__data_start__ %p\n", __data_start__);
	FWK_LOG(dbg_en, "__data_end__   %p\n", __data_end__);
	FWK_LOG(dbg_en, "__bss_start__  %p\n", __bss_start__);
	FWK_LOG(dbg_en, "__bss_end__    %p\n", __bss_end__);
	FWK_LOG(dbg_en, "__end__        %p\n", __end__);
	FWK_LOG(dbg_en, "end            %p\n", end);
	FWK_LOG(dbg_en, "__HeapLimit    %p\n", __HeapLimit);
	FWK_LOG(dbg_en, "__StackLimit   %p\n", __StackLimit);
	FWK_LOG(dbg_en, "__StackTop     %p\n", __StackTop);
	FWK_LOG(dbg_en, "__stack        %p\n", __stack);
	FWK_LOG(dbg_en, "_estack        %p\n", _estack);
#ifdef CONFIG_ROM
	FWK_LOG(dbg_en, "__ram_table_lma_start__ %p\n", __ram_table_lma_start__);
	FWK_LOG(dbg_en, "__ram_table_lma_end__ %p\n", __ram_table_lma_end__);
#endif
	FWK_LOG(dbg_en, "\n");

	FWK_LOG(1, "heap space [%p, %p), size %u\n\n",
	           __heap_start__, __heap_end__,
	           __heap_end__ - __heap_start__);

	FWK_LOG(1,      "cpu  clock %9u Hz\n", HAL_GetCPUClock());
	FWK_LOG(dbg_en, "ahb1 clock %9u Hz\n", HAL_GetAHB1Clock());
	FWK_LOG(dbg_en, "ahb2 clock %9u Hz\n", HAL_GetAHB2Clock());
	FWK_LOG(dbg_en, "apb  clock %9u Hz\n", HAL_GetAPBClock());
	FWK_LOG(dbg_en, "dev  clock %9u Hz\n", HAL_GetDevClock());
	FWK_LOG(dbg_en, "apbs clock %9u Hz\n", HAL_GetAPBSClock());
#ifdef CONFIG_PSRAM
#if (CONFIG_CHIP_ARCH_VER == 2)
	FWK_LOG(dbg_en, "dev2 clock %9u Hz\n", HAL_GetDev2Clock());
#endif
#endif
	FWK_LOG(1,      "HF   clock %9u Hz\n", HAL_GetHFClock());
	FWK_LOG(dbg_en, "LF   clock %9u Hz\n", HAL_GetLFClock(PRCM_LFCLK_MODULE_SYS));
	FWK_LOG(1, "\n");

#if (defined(CONFIG_XIP) || defined(CONFIG_PSRAM) || defined(CONFIG_SECURE_BOOT))
	FWK_LOG(1, "sdk option:\n");
#if (defined(CONFIG_XIP))
	FWK_LOG(1, "    %-14s: enable\n", "XIP");
#if (defined(CONFIG_APP_XIP_ENCRYPT))
	FWK_LOG(dbg_en, "    %-14s: enable\n", "XIP ENCRYPT");
#endif
#endif
#if (defined(CONFIG_PSRAM))
	FWK_LOG(1, "    %-14s: enable\n", "PSRAM");
#if (defined(CONFIG_APP_PSRAM_ENCRYPT))
	FWK_LOG(dbg_en, "    %-14s: enable\n", "PSRAM ENCRYPT");
#endif
	FWK_LOG(1, "psram heap space [%p, %p), size %u\n\n",
	           __PSRAM_End, __PSRAM_Top,
	           __PSRAM_Top - __PSRAM_End);
#endif
#if (defined(CONFIG_SECURE_BOOT))
	FWK_LOG(1, "    %-14s: enable\n", "Security Boot");
#endif
#if (defined(CONFIG_FLASH_CRYPTO))
	FWK_LOG(1, "    %-14s: enable\n", "Flash Crypto");
#endif

#if (BOARD_LOSC_EXTERNAL == 1)
	FWK_LOG(1, "    %-14s: enable\n", "EXT LF OSC");
#elif (BOARD_LOSC_EXTERNAL == 0)
	FWK_LOG(1, "    %-14s: enable\n", "INT LF OSC");
#else
	FWK_LOG(1, "\n%s error(%d)\n", "BOARD_LOSC_EXTERNAL", BOARD_LOSC_EXTERNAL);
#endif
#endif

#ifdef CONFIG_PWR_INTERNAL_DCDC
	FWK_LOG(1, "    %-14s: select\n", "INT DCDC");
#elif (defined(CONFIG_PWR_INTERNAL_LDO))
	FWK_LOG(1, "    %-14s: select\n", "INT LDO");
#elif (defined(CONFIG_PWR_EXTERNAL))
	FWK_LOG(1, "	%-14s: select\n", "EXT PWR");
#endif
	if (HAL_PRCM_GetSMPSDetct()) {
		FWK_LOG(1, "    %-14s: enable\n", "ILDO/EPWR");
	} else {
		FWK_LOG(1, "    %-14s: enable\n", "INT DCDC");
	}

	if (HAL_PRCM_IsFlashSip()) {
		FWK_LOG(1, "    %-14s: enable\n\n", "SIP flash");
	} else {
		FWK_LOG(1, "    %-14s: enable\n\n", "EXT flash");
	}

#if PRJCONF_NET_EN
	FWK_LOG(1, "mac address:\n");
	efpg_read(EFPG_FIELD_MAC_WLAN, mac_addr);
	FWK_LOG(1, "    %-14s: %02x:%02x:%02x:%02x:%02x:%02x\n", "efuse",
	    mac_addr[0], mac_addr[1], mac_addr[2],
	    mac_addr[3], mac_addr[4], mac_addr[5]);
	sys_info = sysinfo_get();
	FWK_LOG(1, "    %-14s: %02x:%02x:%02x:%02x:%02x:%02x\n", "in use",
	    sys_info->mac_addr[0], sys_info->mac_addr[1], sys_info->mac_addr[2],
	    sys_info->mac_addr[3], sys_info->mac_addr[4], sys_info->mac_addr[5]);
#endif
	FWK_LOG(1, "====================================================================\n\n");
}

#ifdef CONFIG_XIP
__sram_text
static void platform_xip_init(void)
{
	uint32_t addr;

	addr = image_get_section_addr(IMAGE_APP_XIP_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		FWK_NX_ERR("no xip section\n");
		return;
	}

#ifdef CONFIG_APP_XIP_ENCRYPT
	section_header_t sh;
	image_read(IMAGE_APP_XIP_ID, IMAGE_SEG_HEADER, 0, (void *)&sh, IMAGE_HEADER_SIZE);
	if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
		if (FlashcXipDecryptEnRequest((uint32_t)__XIP_Base, (uint32_t)__XIP_End) < 0) {
			FWK_NX_ERR("Request Xip decrypt fail !\n");
		} else {
			FWK_NX_DBG("Request Xip decrypt Success.\n");
		}
	}
#endif /* CONFIG_APP_XIP_ENCRYPT */

	/* TODO: check section's validity */
	HAL_Xip_Init(PRJCONF_IMG_FLASH, addr + IMAGE_HEADER_SIZE);
}
#endif /* CONFIG_XIP */

#if PRJCONF_WDG_EN
static void platform_wdg_init(void)
{
	WDG_InitParam param;
#if (CONFIG_CHIP_ARCH_VER == 3)
	if (SysGetStartupState() == SYS_WATCHDOG_CHIP_RST) {
		FWK_DBG("watchdog chip reset\n");
		/* wdg soc reset without ble and rf */
		HAL_PRCM_ForceBLEReset();
		HAL_PRCM_ForceRFASReset();
		/* wdg soc reset will reset wlan but not prcm */
		HAL_PRCM_ForceSys3Reset();
		HAL_PRCM_EnableCPUWClk(0);
	}
#endif /* CONFIG_CHIP_ARCH_VER == 3 */

	/* init watchdog */
	param.hw.event = PRJCONF_WDG_EVENT_TYPE;
#if (CONFIG_CHIP_ARCH_VER > 1)
	param.hw.resetCpuMode = PRJCONF_WDG_RESET_CPU_MODE;
#endif /* CONFIG_CHIP_ARCH_VER > 1 */
	param.hw.timeout = PRJCONF_WDG_TIMEOUT;
	param.hw.resetCycle = WDG_DEFAULT_RESET_CYCLE;
	HAL_WDG_Init(&param);
}

static void platform_wdg_feed(void *arg)
{
	FWK_DBG("feed wdg @ %u sec\n", OS_GetTime());
	HAL_WDG_Feed();
}

static void platform_wdg_start(void)
{
	OS_Timer_t timer;

	/* create OS timer to feed watchdog */
	OS_TimerSetInvalid(&timer);
	if (OS_TimerCreate(&timer, OS_TIMER_PERIODIC, platform_wdg_feed, NULL,
	                   PRJCONF_WDG_FEED_PERIOD) != OS_OK) {
		FWK_WRN("wdg timer create failed\n");
		HAL_WDG_DeInit();
		return;
	}

	HAL_WDG_Start(); /* start watchdog */
	OS_TimerStart(&timer); /* start OS timer to feed watchdog */
}
#endif /* PRJCONF_WDG_EN */

#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
#define RAND_SYS_TICK() ((SysTick->VAL & 0xffffff) | (OS_GetTicks() << 24))

static void platform_prng_init_seed(void)
{
	uint32_t seed[6];
	HAL_Status status;
	int i;

	for (i = 0; i < 5; ++i) {
		status = HAL_TRNG_Extract(0, seed);
		if (status == HAL_OK) {
			seed[4] = seed[0] ^ seed[1];
			seed[5] = seed[2] ^ seed[3];
			FWK_DBG("prng seed %08x %08x %08x %08x %08x %08x\n",
			        seed[0], seed[1], seed[2], seed[3], seed[4], seed[5]);
			break;
		} else {
			FWK_WRN("gen trng fail %d\n", status);
		}
	}

	if (status != HAL_OK) {
		ADC_InitParam initParam;
		initParam.delay = 0;
		initParam.freq = 1000000;
		initParam.mode = ADC_CONTI_CONV;
		status = HAL_ADC_Init(&initParam);
		if (status != HAL_OK) {
			FWK_WRN("adc init err %d\n", status);
		} else {
			status = HAL_ADC_Conv_Polling(ADC_CHANNEL_VBAT, &seed[0], 1000);
			if (status != HAL_OK) {
				FWK_WRN("adc conv err %d\n", status);
			}
			HAL_ADC_DeInit();
		}

		seed[0] ^= RAND_SYS_TICK();
		efpg_read(EFPG_FIELD_CHIPID, (uint8_t *)&seed[1]); /* 16-byte */
		seed[5] = RAND_SYS_TICK();
	}

	HAL_PRNG_SetSeed(seed);
}
#endif /* (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED) */

#ifdef CONFIG_XPLAYER
/* initial cedarx default features */
__weak void platform_cedarx_init(void)
{
	/* for media player */
	CedarxStreamListInit();
#if PRJCONF_NET_EN
	CedarxStreamRegisterHttps();
	CedarxStreamRegisterSsl();
	CedarxThreadStackSizeSet(DEMUX_THREAD, 8 * 1024);
	CedarxStreamRegisterHttp();
	CedarxStreamRegisterTcp();
#endif
	CedarxStreamRegisterFlash();
#ifdef CONFIG_FILESYSTEMS
	CedarxStreamRegisterFile();
#endif
	CedarxStreamRegisterFifo();
	CedarxStreamRegisterCustomer();

	CedarxParserListInit();
	CedarxParserRegisterM3U();
	CedarxParserRegisterM4A();
	CedarxParserRegisterAAC();
	CedarxParserRegisterAMR();
	CedarxParserRegisterMP3();
	CedarxParserRegisterWAV();
	CedarxParserRegisterTS();

	CedarxDecoderListInit();
	CedarxDecoderRegisterAAC();
	CedarxDecoderRegisterAMR();
	CedarxDecoderRegisterMP3();
	CedarxDecoderRegisterWAV();

	SoundStreamListInit();
	SoundStreamRegisterCard();
	SoundStreamRegisterReverb();

	/* for media recorder */
	CedarxWriterListInit();
#ifdef CONFIG_FILESYSTEMS
	CedarxWriterRegisterFile();
#endif
	CedarxWriterRegisterCallback();
	CedarxWriterRegisterCustomer();

	CedarxMuxerListInit();
	CedarxMuxerRegisterAmr();
	CedarxMuxerRegisterPcm();

	CedarxEncoderListInit();
	CedarxEncoderRegisterAmr();
	CedarxEncoderRegisterPcm();
}
#endif

#if (defined(CONFIG_CACHE))
__nonxip_text
static void platform_cache_init(void)
{
	DCache_Config dcache_cfg = { 0 };

	dcache_cfg.vc_en = 1;
	dcache_cfg.wrap_en = 1;
#if (defined(CONFIG_CACHE_SIZE_8K))
	dcache_cfg.way_mode = DCACHE_ASSOCIATE_MODE_DIRECT;
#elif (defined(CONFIG_CACHE_SIZE_16K))
	dcache_cfg.way_mode = DCACHE_ASSOCIATE_MODE_TWO_WAY;
#elif (defined(CONFIG_CACHE_SIZE_32K))
	dcache_cfg.way_mode = DCACHE_ASSOCIATE_MODE_FOUR_WAY;
#else
	#error "config cache size in menuconfig"
#endif
#if (CONFIG_CHIP_ARCH_VER == 2)
	dcache_cfg.mixed_mode = DCACHE_MIXED_MODE_D;
#endif

	HAL_Dcache_Init(&dcache_cfg);
}
#endif

/* init basic platform hardware and services */
__sram_text
__weak void platform_init_level0(void)
{
	int dev;
	FlashBoardCfg *cfg = NULL;

	pm_start();

#ifdef CONFIG_FLASH_CRYPTO
	__sram_rodata static uint8_t flash_enc_nonce[6] = {0x50, 0x00, 0x06, 0x20, 0x00, 0x00};
	HAL_FlashCrypto_Init(flash_enc_nonce);
#endif

#if (CONFIG_CHIP_ARCH_VER == 3)
	HAL_SYSCTL_SelectFlashPsramPinMap(BOARD_FLASH_PSRAM_PIN_MAP0, BOARD_FLASH_PSRAM_PIN_MAP1);
#endif

	dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, PRJCONF_IMG_FLASH);
	HAL_BoardIoctl(HAL_BIR_GET_CFG, dev, (uint32_t)&cfg);
	if (cfg == NULL) {
		FWK_NX_LOG(1, "getFlashBoardCfg failed");
		return;
	}

	if (cfg->type == FLASH_DRV_FLASHC) {
		cfg->flashc.param.freq = cfg->clk;
#if (CONFIG_CHIP_ARCH_VER == 3)
		if (HAL_PRCM_IsFlashSip()) {
			cfg->flashc.param.cs_mode = FLASHC_FLASHCS1_PSRAMCS0;
		} else {
			cfg->flashc.param.cs_mode = FLASHC_FLASHCS0_PSRAMCS1;
		}
#endif
	} else if (cfg->type == FLASH_DRV_SPI) {
		cfg->spi.port = BOARD_SPI_PORT;
		cfg->spi.cs = SPI_TCTRL_SS_SEL_SS0;
		cfg->spi.cs_level = BOARD_SPI_CS_LEVEL;
	}
	HAL_Flash_Init(PRJCONF_IMG_FLASH, cfg);

	image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR);

#if (defined(CONFIG_XIP))
	platform_xip_init();
#ifdef CONFIG_TZ_XIP
	TZ_FlashXipInit_NSC();
#endif
#endif

#if (defined(CONFIG_PSRAM))
	platform_psram_init();
#ifdef CONFIG_TZ_PSRAM
	TZ_PSRAMInit_NSC();
#endif
#endif

#if PRJCONF_BLE_EN
	HAL_PRCM_SetBLESramShare(1);
#endif

#ifdef CONFIG_WLAN_SHARE_RAM
	PRCM_WLAN_ShareSramType type;
#ifdef CONFIG_WLAN_SHARE_RAM_SIZE_16K
	type = PRCM_WLAN_SHARE_16K;
#elif defined(CONFIG_WLAN_SHARE_RAM_SIZE_32K)
	type = PRCM_WLAN_SHARE_32K;
#elif defined(CONFIG_WLAN_SHARE_RAM_SIZE_64K)
	type = PRCM_WLAN_SHARE_64K;
#elif defined(CONFIG_WLAN_SHARE_RAM_SIZE_96K)
	type = PRCM_WLAN_SHARE_96K;
#elif defined(CONFIG_WLAN_SHARE_RAM_SIZE_128K)
	type = PRCM_WLAN_SHARE_128K;
#else
	type = PRCM_WLAN_SHARE_NONE;
#endif
	FWK_NX_LOG(1, "Wlan share app sram type:%x\n", type);
	HAL_PRCM_SetWlanSramShare(type);
#endif

#if (defined(CONFIG_CACHE))
	platform_cache_init();
#endif

	if (cfg->type == FLASH_DRV_FLASHC) {
#ifdef FLASH_XIP_OPT_READ
		cfg->flashc.param.optimize_mask = FLASH_OPTIMIZE_READ;
#else
		cfg->flashc.param.optimize_mask = 0;
#endif
//#ifdef FLASH_XIP_OPT_ERASR
//		cfg->flashc.param.optimize_mask |= FLASH_OPTIMIZE_ERASE;
//#endif
#ifdef FLASH_XIP_OPT_WRITE
		cfg->flashc.param.optimize_mask |= FLASH_OPTIMIZE_WRITE;
#endif
		cfg->flashc.param.ispr_mask[0] = NVIC_PERIPH_IRQ_MASK0;
		cfg->flashc.param.ispr_mask[1] = NVIC_PERIPH_IRQ_MASK1;
		cfg->flashc.param.switch_out_ms = FLASH_SWITCH_OUT_MS;
		cfg->flashc.param.check_busy_us = FLASH_CHECK_BUSY_US;
		cfg->flashc.param.busy_timeout_us = FLASH_BUSY_TIMEOUT_US;
	}

	HAL_Flash_InitLater(PRJCONF_IMG_FLASH, cfg);

#if (defined(CONFIG_PSRAM))
	platform_psram_init_later();
#endif
}

#if PRJCONF_NET_EN
#ifdef CONFIG_WLAN_AP
__weak const wlan_ap_default_conf_t g_wlan_ap_default_conf = {
	.ssid = "AP-XRADIO",
	.ssid_len = 9,
	.psk = "123456789",
	.hw_mode = WLAN_AP_HW_MODE_IEEE80211G,
	.ieee80211n = 1,
	.key_mgmt = WPA_KEY_MGMT_PSK,
	.wpa_pairwise_cipher = WPA_CIPHER_TKIP,
	.rsn_pairwise_cipher = WPA_CIPHER_CCMP,
	.proto = WPA_PROTO_WPA | WPA_PROTO_RSN,
	.auth_alg = WPA_AUTH_ALG_OPEN,
	.group_rekey = 3600,
	.gmk_rekey = 86400,
	.ptk_rekey = 0,
	.strict_rekey = 1,
	.channel = 1,
	.beacon_int = 100,
	.dtim = 1,
	.max_num_sta = 4,
	.country = {'C', 'N', ' '},
};
#endif
#endif

/* init standard platform hardware and services */
__weak void platform_init_level1(void)
{
#if PRJCONF_WDG_EN
	platform_wdg_init();
	platform_wdg_start();
#endif

#if PRJCONF_CE_EN
	HAL_CE_Init();
#endif

#if (defined(CONFIG_WLAN) || defined(CONFIG_BLE))
	uint16_t freq_trim = 0;

	if (efpg_read_dcxo((uint8_t *)&freq_trim) != EFPG_ACK_OK) {
		int ret;
		struct sdd sdd;

		ret = sdd_request(&sdd);
		if (ret > 0) {
			struct sdd_ie *id = sdd_get_ie(&sdd, SDD_WLAN_DYNAMIC_SECT_ID, SDD_XTAL_TRIM_ELT_ID);
			freq_trim = le16dec(id->data);
			sdd_release(&sdd);
		} else {
			FWK_WRN("not find sdd!\n");
		}
	}
	if (freq_trim)
		HAL_PRCM_SetDcxoFreqTrim(freq_trim);
#endif

#if PRJCONF_SYS_CTRL_EN
	sys_ctrl_create();
  #if PRJCONF_NET_EN
	net_ctrl_init();
  #endif
#endif

#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
	platform_prng_init_seed(); /* init prng seed */
#endif

	sysinfo_init();

#if PRJCONF_CONSOLE_EN
	console_param_t cparam;
	cparam.uart_id = BOARD_MAIN_UART_ID;
	cparam.cmd_exec = main_cmd_exec;
	cparam.stack_size = PRJCONF_CONSOLE_STACK_SIZE;
	console_start(&cparam);
#endif

	pm_mode_platform_select(PRJCONF_PM_MODE);

#if PRJCONF_NET_EN
#if (CONFIG_WPA_HEAP_MODE == 1)
	wpa_set_heap_fn(psram_malloc, psram_realloc, psram_free);
#endif
#if (CONFIG_UMAC_HEAP_MODE == 1)
	umac_set_heap_fn(psram_malloc, psram_free);
#endif
#if (CONFIG_LMAC_HEAP_MODE == 1)
	lmac_set_heap_fn(psram_malloc, psram_free);
#endif
#if (CONFIG_MBUF_HEAP_MODE == 1)
	wlan_ext_request(NULL, WLAN_EXT_CMD_SET_RX_QUEUE_SIZE, 256);
#endif
#ifdef CONFIG_WLAN_AP
	wlan_ap_set_default_conf(&g_wlan_ap_default_conf);
#endif

#ifndef CONFIG_ETF
	net_sys_init();
#endif

	struct sysinfo *sysinfo = sysinfo_get();
	net_sys_start(sysinfo->wlan_mode);
#endif /* PRJCONF_NET_EN */
}

/* init extern platform hardware and services */
__weak void platform_init_level2(void)
{
#if (defined(CONFIG_FAT_FS) || defined(CONFIG_LITTLE_FS) || defined(CONFIG_SPIF_FS))
	fs_ctrl_init();
#endif
#ifdef CONFIG_SDMMC
	 board_sdcard_init(sdcard_detect_callback);
#endif
#if (defined(CONFIG_LITTLE_FS) || defined(CONFIG_SPIF_FS))
	fs_ctrl_mount(FS_MNT_DEV_TYPE_FLASH, 0);
#endif
#if PRJCONF_AUDIO_SNDCARD_EN
	board_soundcard_init();

	audio_manager_init();
	snd_pcm_init();
  #if PRJCONF_AUDIO_CTRL_EN
	audio_ctrl_init();
  #endif
#endif

#ifdef CONFIG_XPLAYER
	platform_cedarx_init();
#endif
	extern int debug_init(void);
	debug_init();

#ifdef CONFIG_CPLUSPLUS
	extern int cpp_init(void);
	cpp_init();
#endif
}

__sram_text
void platform_init(void)
{
	platform_init_level0();
	platform_init_level1();
	platform_init_level2();
	platform_show_info();
}
