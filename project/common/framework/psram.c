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
#include <stdlib.h>
#include <stdio.h>

#include "sys/param.h"
#include "sys/io.h"
#include "sys/xr_debug.h"

#include "image/fdcm.h"
#include "image/image.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_xip.h"
#include "driver/chip/hal_dcache.h"
#include "psram.h"
#include "driver/chip/psram/psram.h"
#include "common/board/board.h"

#ifdef CONFIG_PSRAM

#ifndef CONFIG_LOAD_PSRAM_BY_BL
#define LOAD_SIZE               (1 * 1024)

#define LOAD_BY_DBUS_CPU        0
#define LOAD_BY_DBUS_DMA        1
#define LOAD_BY_DBUS_FLASH_DMA  2

#define PSRAM_LOAD_METHOD       LOAD_BY_DBUS_DMA

#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_DMA)
#include "driver/chip/hal_dma.h"
__sram_data
static uint32_t dma_wburst0 = 0, dma_wburst1 = 0;
__sram_data
static uint32_t dma_wwidth0 = 0, dma_wwidth1 = 0;
__sram_data
static uint32_t dma_rburst0 = 0, dma_rburst1 = 0;
__sram_data
static uint32_t dma_rwidth0 = 0, dma_rwidth1 = 0;
__sram_data
static OS_Semaphore_t dmaSem;
__sram_data
uint32_t dma_burst_type[2] = {DMA_BURST_LEN_1, DMA_BURST_LEN_4};
__sram_data
uint32_t dma_width_type[3] = {DMA_DATA_WIDTH_8BIT, DMA_DATA_WIDTH_16BIT, DMA_DATA_WIDTH_32BIT};

__sram_text
static void psram_DMARelease(void *arg)
{
	OS_SemaphoreRelease(&dmaSem);
}

__sram_text
static int psram_dma_read_write(uint32_t write, uint32_t addr,
                                uint8_t *buf, uint32_t len)
{
	OS_Status ret;
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;
	DMA_Periph periph;

#if (CONFIG_CHIP_ARCH_VER == 2)
	periph = DMA_PERIPH_PSRAMC;
#elif (CONFIG_CHIP_ARCH_VER == 3)
	periph = DMA_PERIPH_FLASHC;
#endif

	dma_ch = HAL_DMA_Request();
	if (dma_ch == DMA_CHANNEL_INVALID) {
		PSRAM_ERR("%s,%d\n", __func__, __LINE__);
		return -1;
	}

	OS_SemaphoreCreate(&dmaSem, 0, 1);

	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = (DMA_IRQCallback)psram_DMARelease;
	dmaParam.endArg = NULL;
	if (write) {
		if (++dma_wburst0 >= 2) {
			dma_wburst0 = 0;
			if (++dma_wburst1 >= 2) {
				dma_wburst1 = 0;
				if (++dma_wwidth0 >= 3) {
					dma_wwidth0 = 0;
					if (++dma_wwidth1 >= 3)
						dma_wwidth1 = 0;
				}
			}
		}
	} else {
		if (++dma_rburst0 >= 2) {
			dma_rburst0 = 0;
			if (++dma_rburst1 >= 2) {
				dma_rburst1 = 0;
				if (++dma_rwidth0 >= 3) {
					dma_rwidth0 = 0;
					if (++dma_rwidth1 >= 3)
						dma_rwidth1 = 0;
				}
			}
		}
	}
	if (write) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		               DMA_WAIT_CYCLE_2,
		               DMA_BYTE_CNT_MODE_REMAIN,
		               dma_width_type[dma_wwidth0],
		               dma_burst_type[dma_wburst0],
		               DMA_ADDR_MODE_INC,
		               periph,
		               dma_width_type[dma_wwidth1],
		               dma_burst_type[dma_wburst1],
		               DMA_ADDR_MODE_INC,
		               DMA_PERIPH_SRAM);
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		               DMA_WAIT_CYCLE_2,
		               DMA_BYTE_CNT_MODE_REMAIN,
		               dma_width_type[dma_rwidth0],
		               dma_burst_type[dma_rburst0],
		               DMA_ADDR_MODE_INC,
		               DMA_PERIPH_SRAM,
		               dma_width_type[dma_rwidth1],
		               dma_burst_type[dma_rburst1],
		               DMA_ADDR_MODE_INC,
		               periph);
	}
	HAL_DMA_Init(dma_ch, &dmaParam);
	//PSRAM_DBG("%s,%d rw:%d\n", __func__, __LINE__, write);
	//PSRAM_DBG("wb0:%d wb1:%d ww0:%d ww1:%d\n", dma_wburst0, dma_wburst1, dma_wwidth0, dma_wwidth1);
	//PSRAM_DBG("rb0:%d rb1:%d rw0:%d rw1:%d\n", dma_rburst0, dma_rburst1, dma_rwidth0, dma_rwidth1);
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PsramCtrl_DMACrossEnable(1);
#endif
	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)buf, (uint32_t)addr, len);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)addr, (uint32_t)buf, len);

	ret = OS_SemaphoreWait(&dmaSem, 5000);
	if (ret != OS_OK)
		PSRAM_ERR("sem wait failed: %d", ret);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);
#if (CONFIG_CHIP_ARCH_VER == 2)
	HAL_PsramCtrl_DMACrossEnable(0);
#endif

	OS_SemaphoreDelete(&dmaSem);

	return 0;
}
#endif
#endif /* CONFIG_LOAD_PSRAM_BY_BL */

__sram_text
static int load_psram_bin_and_set_addr(struct psram_chip *chip)
{
	extern uint8_t __psram_start__[];
	extern uint8_t __psram_end__[];
	extern uint8_t __psram_data_start__[];
	extern uint8_t __psram_data_end__[];
	extern uint8_t __psram_bss_start__[];
	extern uint8_t __psram_bss_end__[];

	int ret = 1;
#ifdef CONFIG_LOAD_PSRAM_BY_BL
	PsramCBUS_AddrRequire(FLASH_ADDR_PSRAM_IDX, PSRAM_START_ADDR,
	                      PSRAM_START_ADDR - PSRAM_MAPPING_ADDR,
	                      PSRAM_END_ADDR - PSRAM_START_ADDR);
	ret = 0;
#else
	uint32_t put = 0, len = 0;
	section_header_t sh;
#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_FLASH_DMA)
	uint8_t *buf = (uint8_t *)PSRAM_START_ADDR;
#else
	uint8_t *buf = malloc(LOAD_SIZE + PSRAM_DBG_CHECK * LOAD_SIZE);
#endif
	uint32_t i, num_blocks, size, img_len;

	if (!buf) {
		PSRAM_ERR("%s malloc faild!\n", __func__);
		return -1;
	}

	if (image_read(IMAGE_APP_PSRAM_ID, IMAGE_SEG_HEADER, 0, &sh,
	               IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
		PSRAM_ERR("load section (id: %#08x) header failed\n", IMAGE_APP_PSRAM_ID);
		ret = -1;
		goto out;
	}

	/* Check psram header */
	if (image_check_header(&sh) == IMAGE_INVALID) {
		PSRAM_ERR("check section (id: %#08x) header failed\n", IMAGE_APP_PSRAM_ID);
		ret = -1;
		goto out;
	}

	/* Map psram physical addr */
	PsramCBUS_AddrRequire(FLASH_ADDR_PSRAM_IDX, PSRAM_START_ADDR,
	                      PSRAM_START_ADDR - PSRAM_MAPPING_ADDR,
	                      PSRAM_END_ADDR - PSRAM_START_ADDR);

	img_len = sh.data_size; // if turn secure boot, img_len = sh.data_size; else img_len = sh.body_len;
	                        // ensure that the check data is passed.

	if (img_len == 0) { /* psram only used to store data */
		PSRAM_INF("psram only used store data\n");
		goto clr_bss;
	}

	num_blocks = (img_len + LOAD_SIZE - 1) / LOAD_SIZE;
	PSRAM_INF("img_len=%d load_times:%d\n", img_len, num_blocks);

	/* Load psram body code */
	for (i = 0; i < num_blocks; i++) {
		//PSRAM_DBG("psram loading idx:%d addr[0x%x] len:%d\n", i, PSRAM_START_ADDR + len, len);
		size = ((img_len - len) > LOAD_SIZE) ? LOAD_SIZE : (img_len - len);
		len += image_read(IMAGE_APP_PSRAM_ID, IMAGE_SEG_BODY, len, (void *)buf, size);
#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_CPU)
		memcpy((void *)(PSRAM_START_ADDR + put), buf, size); /* if support DMA, copy by image */
#elif (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_DMA)
		psram_dma_read_write(1, (uint32_t)(PSRAM_START_ADDR + put), buf, size);
		//PSRAM_DUMP((const void *)(PSRAM_START_ADDR + put), 64);
#endif

#if (PSRAM_LOAD_METHOD == LOAD_BY_DBUS_FLASH_DMA)
		buf += size;
#endif
		put += size;
	}

	/* Check data checksum */
	if (len != img_len) {
		PSRAM_ERR("psram body size %u, read %u\n", img_len, len);
		ret = -1;
		goto out;
	}

#if ((defined CONFIG_PSRAM_CHIP_SQPI) && (CONFIG_CHIP_ARCH_VER == 2))
	psram_sw_reset(chip, 0);
#endif

	if (!(sh.attribute & IMAGE_ATTR_FLAG_ENC) && image_check_data(&sh, (void *)PSRAM_START_ADDR, sh.data_size,
	                     NULL, 0) == IMAGE_INVALID) {
		PSRAM_ERR("invalid psram bin body\n");
		//PSRAM_DUMP((const void *)PSRAM_START_ADDR, sh.body_len);
		ret = -1;
		goto out;
	}

#ifdef CONFIG_APP_PSRAM_ENCRYPT
	/* open the real-time decryption function of app_psram bin */
	if (sh.attribute & IMAGE_ATTR_FLAG_ENC) {
		if (FlashcPsramDecryptEnRequest((uint32_t)__PSRAM_Base, (uint32_t)__psram_data_end__) < 0) {
			PSRAM_ERR("Request Psram decrypt fail !\n");
		}
	}
#endif /* CONFIG_APP_PSRAM_ENCRYPT */

clr_bss:
	memset(__psram_bss_start__, 0, __psram_bss_end__ - __psram_bss_start__);

#if PSRAM_DBG_CHECK
	for (uint32_t *addr = (uint32_t *)__psram_bss_start__;
	     addr < (uint32_t *)__psram_bss_end__; addr++) {
		if (readl(addr)) {
			PSRAM_ERR("bss not cleared! at[%p]:0x%x\n", addr, readl(addr));
			break;
		}
	}
#endif

out:
#ifndef LOAD_BY_DBUS_FLASH_DMA
	free(buf);
#endif
#endif /* CONFIG_LOAD_PSRAM_BY_BL */

	PSRAM_INF("__psram_start__\t%p\n", __psram_start__);
	PSRAM_INF("__psram_data_start__\t%p\n", __psram_data_start__);
	PSRAM_INF("__psram_data_end__\t%p\n", __psram_data_end__);
	PSRAM_INF("__psram_bss_start__\t%p\n", __psram_bss_start__);
	PSRAM_INF("__psram_bss_end__\t%p\n", __psram_bss_end__);
	PSRAM_INF("__psram_end__\t\t%p\n", __psram_end__);

	return ret;
}

__sram_data
static struct psram_chip chip = {0};

__sram_text
void platform_psram_init(void)
{
	int ret;
	uint32_t addr, p_type = PSRAM_CHIP_MAX;
	struct psram_ctrl *ctrl = NULL;
	PSRAMCtrl_InitParam cfg;
	PSRAMChip_InitParam psram_chip_cfg;

	addr = image_get_section_addr(IMAGE_APP_PSRAM_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		PSRAM_ERR("no psram section\n");
		return ;
	}
	PSRAM_INF("get psram section ok @:0x%x\n", addr);

#if (defined CONFIG_PSRAM_CHIP_SQPI)
	p_type = PSRAM_CHIP_SQPI;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	p_type = PSRAM_CHIP_OPI_APS32;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	p_type = PSRAM_CHIP_OPI_APS64;
#endif

	cfg.freq = PSRAM_FREQ;
	cfg.p_type = p_type;
	if (cfg.p_type == PSRAM_CHIP_SQPI)
		cfg.rdata_w = 0; /* wait 0 half cycle on fpga */
	else
		cfg.rdata_w = 1; /* wait 1 half cycle on fpga */
	ctrl = HAL_PsramCtrl_Create(0, &cfg);
	if (!ctrl) {
		PSRAM_ERR("psram create faile\n");
		return ;
	}

#ifndef CONFIG_XIP
	addr = image_get_section_addr(IMAGE_APP_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		PSRAM_INF("no app section\n");
		goto out4;
	}
	/* TODO: check section's validity */
	HAL_Xip_Init(PRJCONF_IMG_FLASH, addr + IMAGE_HEADER_SIZE);
#endif

	if (HAL_PsramCtrl_Init(ctrl)) {
		PSRAM_ERR("psram ctrl init faild!\n");
		goto out3;
	}

	ctrl = HAL_PsramCtrl_Open(0);
	if (!ctrl) {
		PSRAM_ERR("psram open faile\n");
		goto out2;
	}

	psram_chip_cfg.freq = PSRAM_FREQ;
	psram_chip_cfg.p_type = p_type;
	if (psram_init(&chip, ctrl, &psram_chip_cfg)) {
		PSRAM_ERR("psram chip init faild!\n");
		goto out1;
	}
	PSRAM_DBG("psram chip %s init ok!, freq %d\n", chip.name, cfg.freq);

	ret = load_psram_bin_and_set_addr(&chip);
	if (ret < 0) {
		PSRAM_ERR("load psram bin faild!\n");
		goto out;
	} else if (ret > 0) {
		PSRAM_DBG("load psram bin ok\n");
	} else {
		PSRAM_DBG("load psram bin in bootloader\n");
	}

	return ;

out:
	psram_deinit(&chip);

out1:
	HAL_PsramCtrl_Close(ctrl);

out2:
	HAL_PsramCtrl_Deinit(ctrl);

out3:
#ifndef CONFIG_XIP
	HAL_Xip_Deinit(PRJCONF_IMG_FLASH);

out4:
#endif
	HAL_PsramCtrl_Destory(ctrl);

	return;
}

__sram_text
void platform_psram_init_later(void)
{
#ifdef CONFIG_PSRAM_HEAP_WRITETHROUGH
	extern uint8_t __psram_bss_end__[];

	HAL_Dcache_Config_WriteThrough(HAL_Dcache_Request_WriteThroughIndex(), \
	                               rounddown2((uint32_t)__psram_bss_end__, 16), \
	                               PSRAM_END_ADDR);
#endif
	HAL_PsramCtrl_DMACrossEnable(1);
}

#endif /* CONFIG_PSRAM */
