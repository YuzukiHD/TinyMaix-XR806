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

#include "pm/pm.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_global.h"
#include "sys/param.h"
#include "hal_base.h"
#include "flashchip/flash_debug.h"

#ifdef CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ
#define FLASH_ERASE_BLOCK_SIZE_ADAPTIVE 1
#endif

#if (defined(CONFIG_FLASH_POWER_DOWN_PROTECT) || defined(CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ))
extern HAL_Status __HAL_Flash_Erase(uint32_t flash, FlashEraseMode blk_size,
                                    uint32_t addr, uint32_t blk_cnt);
#endif

#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
#define DEFAULT_WP_BLOCK_LOCK_BIT          HAL_BIT(2)
#define FD_CHECK_ON                        0

extern HAL_Status __HAL_Flash_Write(uint32_t flash, uint32_t addr,
                                    const uint8_t *data, uint32_t size);

static int flashwpSetBlockLockMode(struct FlashDev *dev)
{
	int ret;
	uint8_t status;

	dev->drv->open(dev->chip);

	/* read status register-3 */
	ret = dev->chip->readStatus(dev->chip, FLASH_STATUS3, &status);
	FD_DEBUG("read status: %x", status);
	if (ret != HAL_OK) {
		FD_ERROR("read status err");
	}

	/* configure status: select block lock mode */
	HAL_SET_BIT(status, DEFAULT_WP_BLOCK_LOCK_BIT);
	FD_DEBUG("cfg status: %x", status);

	/* write status register-3 use volatile style */
	defaultWriteSREnableForVolatile(dev->chip);
	ret = dev->chip->writeStatus(dev->chip, FLASH_STATUS3, &status);
	HAL_Flash_WaitCompl(dev, 5000);
	/*dev->chip->writeDisable(dev->chip);*//* no need */

#if FD_CHECK_ON
	dev->chip->readStatus(dev->chip, FLASH_STATUS3, &status);
	FD_DEBUG("cfg OK, status is: %x", status);
#endif

	dev->drv->close(dev->chip);
	return ret;
}

/* configure the protect state for the whole chip(all blocks) */
static int flashwpSetAllBlocksState(struct FlashDev *dev, FlashBlockLockState state)
{
	int ret;
	/* check flash protect mode */
#if FD_CHECK_ON
	uint8_t status, wps_bit;
	dev->drv->open(dev->chip);
	ret = dev->chip->readStatus(dev->chip, FLASH_STATUS3, &status);
	if (ret != HAL_OK) {
		dev->drv->close(dev->chip);
		FD_ERROR("read status err");
		return -1;
	}
	FD_DEBUG("read status3: %x", status);
	wps_bit = HAL_GET_BIT(status, DEFAULT_WP_BLOCK_LOCK_BIT);
	if (!wps_bit) {
		dev->drv->close(dev->chip);
		FD_ERROR("not in block protect mode, wps_bit : %d", wps_bit);
		return -1;
	}
	dev->drv->close(dev->chip);
#endif

	/* set all block state */
	dev->drv->open(dev->chip);
	dev->chip->writeEnable(dev->chip);
	ret = defaultSetGlobalBlockLockState(dev->chip, state);
	HAL_Flash_WaitCompl(dev, 5000);
	dev->chip->writeDisable(dev->chip);
	dev->drv->close(dev->chip);

	return ret;
}

static int flashGetWpCfg(struct FlashDev *dev)
{
	/* get flash ID */
	uint32_t jedec;
	dev->drv->open(dev->chip);
	int ret = dev->chip->jedecID(dev->chip, &jedec);
	dev->drv->close(dev->chip);
	if (ret != HAL_OK) {
		FD_ERROR("get jedecID");
	}

	/* check if support flash wp */
	int wp_len = 0;
	const FlashChipWpCfg *flash_wp = (const FlashChipWpCfg *)FlashChipGetWpCfgList(&wp_len);
	for (int i = 0; i < wp_len; i++) {
		if (flash_wp[i].mJedec == jedec) {
			dev->chip->mWpCfg = (void *)&flash_wp[i];
			return 0;
		}
	}

	FD_ERROR("WP not support");
	return -1;
}

#if FD_CHECK_ON
static int flashwpGetBlockState(struct FlashDev *dev, uint32_t addr, uint8_t *state)
{
	int ret;

	dev->drv->open(dev->chip);
	ret = defaultReadBlockLockStatus(dev->chip, addr, state);
	/*HAL_Flash_WaitCompl(dev, 5000);*/
	dev->drv->close(dev->chip);

	return ret;
}

static int flashwpPrintAllBlockState(struct FlashDev *dev)
{
	int ret = 0;
	const FlashChipWpCfg *WpCfg = (const FlashChipWpCfg *)dev->chip->mWpCfg;
	const FlashChipBlockLockModeCfg *WpBlockCfg = WpCfg->mWpBlockLockCfg;

	uint32_t area1_btm_sector_num = WpBlockCfg->mBtmBlockBoundaryAddr /
	                                WpBlockCfg->mWpSectorSize;
	uint32_t area2_mid_block_num  = (WpBlockCfg->mTopBlockBoundaryAddr -
	                                WpBlockCfg->mBtmBlockBoundaryAddr) /
	                                WpBlockCfg->mWpBlockSize;
	uint32_t area3_top_sector_num = area1_btm_sector_num;

	uint32_t area2_block_id_offs  = WpBlockCfg->mBtmBlockBoundaryAddr /
	                                WpBlockCfg->mWpBlockSize;
	uint32_t area3_sector_id_offs = WpBlockCfg->mTopBlockBoundaryAddr /
	                                WpBlockCfg->mWpSectorSize;

	uint8_t *state1_sector = (uint8_t *)HAL_Malloc(area1_btm_sector_num);
	if (!state1_sector) {
		ret = -1;
		FD_ERROR("no mem");
		goto out3;
	}
	uint8_t *state2_block = (uint8_t *)HAL_Malloc(area2_mid_block_num);
	if (!state2_block) {
		ret = -1;
		FD_ERROR("no mem");
		goto out2;
	}
	uint8_t *state3_sector = (uint8_t *)HAL_Malloc(area3_top_sector_num);
	if (!state3_sector) {
		ret = -1;
		FD_ERROR("no mem");
		goto out1;
	}
	uint8_t *state_p;

	/* get bottom area sector state, such as: sector0 ~ sector15 */
	state_p = state1_sector;
	uint32_t s_addr = 0;
	for (int i = 0; i < area1_btm_sector_num; i++) {
		ret = flashwpGetBlockState(dev, s_addr, state_p);
		s_addr += WpBlockCfg->mWpSectorSize;
		state_p++;
		FD_INFO("state_sector[%d] = %d", i, state1_sector[i]);
	}

	/* get mid area block state, such as: block1 ~ block30 */
	s_addr = WpBlockCfg->mBtmBlockBoundaryAddr;
	state_p = state2_block;
	for (int i = 0; i < area2_mid_block_num; i++) {
		ret = flashwpGetBlockState(dev, s_addr, state_p);
		s_addr += WpBlockCfg->mWpBlockSize;
		state_p++;
		FD_INFO("state2_block[%d] = %d", i + area2_block_id_offs, state2_block[i]);
	}

	/* get top area sector state, such as sector(31*16+0) ~ sector(31*16+15) */
	s_addr = WpBlockCfg->mTopBlockBoundaryAddr;
	state_p = state3_sector;
	for (int i = 0; i < area3_top_sector_num; i++) {
		ret = flashwpGetBlockState(dev, s_addr, state_p);
		s_addr += WpBlockCfg->mWpSectorSize;
		state_p++;
		FD_INFO("state3_sector[%d] = %d", i + area3_sector_id_offs, state3_sector[i]);
	}
	FD_INFO("get all block state OK\n");

	HAL_Free(state3_sector);
out1:
	HAL_Free(state2_block);
out2:
	HAL_Free(state1_sector);
out3:
	return ret;
}
#endif /* FD_CHECK_ON */

static int flashwpBlockLockInit(uint32_t flash)
{
	int ret;
	struct FlashDev *dev = getFlashDev(flash);
	if (dev == NULL) {
		FD_ERROR("invalid dev");
		return -1;
	}

	/* get flashchip write protect cfg */
	ret = flashGetWpCfg(dev);
	if (ret != 0) {
		return ret;
	}

	/* check if support block lock mode */
	const FlashChipWpCfg *WpCfg = (const FlashChipWpCfg *)dev->chip->mWpCfg;
	if (!WpCfg->mWpBlockLockCfg) {
		FD_ERROR("WP not support blk mode");
		return -1;
	}

	/* set flash block protect mode */
	ret = flashwpSetBlockLockMode(dev);
	if (ret != 0) {
		return ret;
	}

	/* set unprotect all block: for BROM can upgrade success in no powerdown */
	ret = flashwpSetAllBlocksState(dev, FLASH_BLOCK_STATE_UNLOCK);
#if FD_CHECK_ON
	flashwpPrintAllBlockState(dev);
#endif

	return ret;
}

/* configure the protect state for a sector/block */
static int flashwpSetBlockState(uint32_t flash, uint32_t addr,
                                FlashBlockLockState state)
{
	int ret;
	struct FlashDev *dev = getFlashDev(flash);

	dev->drv->open(dev->chip);
	dev->chip->writeEnable(dev->chip);
	if (state == FLASH_BLOCK_STATE_LOCK) {
		ret = defaultLockBlock(dev->chip, addr);
	} else if (state == FLASH_BLOCK_STATE_UNLOCK) {
		ret = defaultUnLockBlock(dev->chip, addr);
	} else {
		ret = -1;
	}
	HAL_Flash_WaitCompl(dev, 5000);
	dev->chip->writeDisable(dev->chip);
	dev->drv->close(dev->chip);

	/*#if FD_CHECK_ON
		flashwpPrintAllBlockState(dev);
	#endif*/
	return ret;
}

static HAL_Status flashwpEraseArea(uint32_t flash, FlashEraseMode blk_size,
                                   uint32_t addr, uint32_t blk_cnt,
                                   uint32_t wp_size)
{
	HAL_Status ret;
	uint32_t cnt;

	if (wp_size < blk_size) {
		FD_ERROR("%u < %u", wp_size, blk_size);
		return HAL_ERROR; /* TODO: support this case */
	}

	/* handle wp_size >= blk_size only */
	while (blk_cnt > 0) {
		cnt = wp_size > blk_size ? ((wp_size - addr % wp_size) / blk_size) : 1;
		if (cnt > blk_cnt) {
			cnt = blk_cnt;
		}
		flashwpSetBlockState(flash, addr, FLASH_BLOCK_STATE_UNLOCK);
		FD_DEBUG("%s(), flash %d, blk_size %u, addr 0x%x, cnt %d", __func__, flash,
		         blk_size, addr, cnt);
		ret = __HAL_Flash_Erase(flash, blk_size, addr, cnt);
		flashwpSetBlockState(flash, addr, FLASH_BLOCK_STATE_LOCK);
		if (ret != HAL_OK) {
			return ret;
		}
		addr += blk_size * cnt;
		blk_cnt -= cnt;
	}

	return HAL_OK;
}

static HAL_Status flashwpErase(uint32_t flash, FlashChipWpCfg *WpCfg,
                               FlashEraseMode blk_size, uint32_t addr,
                               uint32_t blk_cnt)
{
	HAL_Status ret;
	FlashEraseMode erz_mode;
	uint32_t len, wp_size;
	uint32_t left = blk_size * blk_cnt;

	const FlashChipBlockLockModeCfg *WpBlkCfg = WpCfg->mWpBlockLockCfg;
	uint32_t wp_btm_boundary = WpBlkCfg->mBtmBlockBoundaryAddr;
	uint32_t wp_top_boundary = WpBlkCfg->mTopBlockBoundaryAddr;

	FD_DEBUG("%s(), (0x%x, 0x%x), (%u, %u, %u)", __func__, wp_btm_boundary,
	         wp_top_boundary, blk_size, WpBlkCfg->mWpSectorSize,
	         WpBlkCfg->mWpBlockSize);

	while (left > 0) {
		if (addr < wp_btm_boundary) {
			len = wp_btm_boundary - addr;
			wp_size = WpBlkCfg->mWpSectorSize;
		} else if (addr < wp_top_boundary) {
			len = wp_top_boundary - addr;
			wp_size = WpBlkCfg->mWpBlockSize;
		} else {
			len = left;
			wp_size = WpBlkCfg->mWpSectorSize;
		}

		if (len > left) {
			len = left;
		}

		if (blk_size <= wp_size) {
			erz_mode = blk_size;
		} else {
			erz_mode = wp_size;
		}

		FD_DEBUG("%s(), erz_mode %u, addr 0x%x, len %u, wp_size %u",
		         __func__, erz_mode, addr, len, wp_size);
		ret = flashwpEraseArea(flash, erz_mode, addr, len / erz_mode, wp_size);
		if (ret != HAL_OK) {
			return ret;
		}

		addr += len;
		left -= len;
	}

	return HAL_OK;
}

static HAL_Status flashwpWriteArea(uint32_t flash, uint32_t addr,
                                   const uint8_t *data,
                                   uint32_t size, uint32_t wp_size)
{
	HAL_Status ret;
	uint32_t len;

	while (size > 0) {
		len = wp_size - (addr % wp_size);
		if (len > size) {
			len = size;
		}
		flashwpSetBlockState(flash, addr, FLASH_BLOCK_STATE_UNLOCK);
		ret = __HAL_Flash_Write(flash, addr, data, len);
		flashwpSetBlockState(flash, addr, FLASH_BLOCK_STATE_LOCK);
		if (ret != HAL_OK) {
			return ret;
		}
		addr += len;
		data += len;
		size -= len;
	}

	return HAL_OK;
}

static HAL_Status flashwpWrite(uint32_t flash, const FlashChipWpCfg *WpCfg,
                               uint32_t addr, const uint8_t *data, uint32_t size)
{
	HAL_Status ret;
	uint32_t len;
	uint32_t wp_size;

	const FlashChipBlockLockModeCfg *WpBlkCfg = WpCfg->mWpBlockLockCfg;
	uint32_t wp_btm_boundary = WpBlkCfg->mBtmBlockBoundaryAddr;
	uint32_t wp_top_boundary = WpBlkCfg->mTopBlockBoundaryAddr;

	while (size > 0) {
		if (addr < wp_btm_boundary) {
			len = wp_btm_boundary - addr;
			wp_size = WpBlkCfg->mWpSectorSize;
		} else if (addr < wp_top_boundary) {
			len = wp_top_boundary - addr;
			wp_size = WpBlkCfg->mWpBlockSize;
		} else {
			len = size;
			wp_size = WpBlkCfg->mWpSectorSize;
		}

		if (len > size) {
			len = size;
		}

		ret = flashwpWriteArea(flash, addr, data, len, wp_size);
		if (ret != HAL_OK) {
			return ret;
		}

		addr += len;
		size -= len;
		data += len;
	}

	return HAL_OK;
}

#endif /* CONFIG_FLASH_POWER_DOWN_PROTECT */

HAL_Status HAL_Flash_Ioctl(uint32_t flash, FlashControlCmd attr, uint32_t arg)
{
	HAL_Status ret = HAL_ERROR;
	struct FlashDev *dev = getFlashDev(flash);

	switch (attr) {
	/*TODO: 1.return min erase size */
	case FLASH_GET_MIN_ERASE_SIZE: {
		*((FlashEraseMode *)arg) = dev->chip->minEraseSize(dev->chip);
		ret = HAL_OK;
		break;
	}
	case FLASH_WRITE_STATUS: {
		FlashControlStatus *tmp = (FlashControlStatus *)arg;
		dev->drv->open(dev->chip);
		dev->chip->writeEnable(dev->chip);
		ret = dev->chip->writeStatus(dev->chip, tmp->status, tmp->data);
		HAL_Flash_WaitCompl(dev, 5000);
		dev->chip->writeDisable(dev->chip);
		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_WRITE_STATUS_VOLATILE: {
		FlashControlStatus *tmp = (FlashControlStatus *)arg;
		dev->drv->open(dev->chip);
		defaultWriteSREnableForVolatile(dev->chip);
		ret = dev->chip->writeStatus(dev->chip, tmp->status, tmp->data);
		HAL_Flash_WaitCompl(dev, 5000);
		/*dev->chip->writeDisable(dev->chip);*/
		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_READ_STATUS: {
		FlashControlStatus *tmp = (FlashControlStatus *)arg;
		dev->drv->open(dev->chip);
		ret = dev->chip->readStatus(dev->chip, tmp->status, tmp->data);
		HAL_Flash_WaitCompl(dev, 5000);
		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_SET_READ_MODE: {
		FlashReadMode old_rmode = dev->rmode;

		dev->rmode = (FlashReadMode)arg;
		if (!(dev->rmode & dev->chip->cfg.mReadSupport)) {
			FD_ERROR("not support read mode %d", dev->rmode);
			return HAL_INVALID;
		}

		if (old_rmode == FLASH_READ_QPI_MODE) {
			dev->drv->open(dev->chip);
			ret = dev->chip->disableQPIMode(dev->chip);
			if (ret != HAL_OK) {
				return ret;
			}
			HAL_Flash_WaitCompl(dev, 5000); /* wait busy */
			dev->drv->close(dev->chip);
		}

		dev->drv->open(dev->chip);
		ret = dev->chip->switchReadMode(dev->chip, dev->rmode);

		if (ret != HAL_OK) {
			return ret;
		}

		HAL_Flash_WaitCompl(dev, 5000); /* wait busy */

		if (dev->rmode & FLASH_READ_QPI_MODE) {
			ret = dev->chip->enableQPIMode(dev->chip);
		}

		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_SET_PAGEPROGRAM_MODE: {
		dev->wmode = (FlashPageProgramMode)arg;
		if (!(dev->wmode & dev->chip->cfg.mPageProgramSupport)) {
			FD_ERROR("not support page program mode %d", dev->wmode);
			return HAL_INVALID;
		}
		ret = HAL_OK;
		break;
	}
	case FLASH_SET_SUSPEND_PARAM: {
		Flashc_Config *param = (Flashc_Config *)arg;

		dev->ispr_mask[0] = param->ispr_mask[0];
		dev->ispr_mask[1] = param->ispr_mask[1];
		dev->switch_out_ms = param->switch_out_ms;
		dev->check_busy_us = param->check_busy_us;
		dev->busy_timeout_us = param->busy_timeout_us;
		ret = HAL_OK;
		break;
	}
	/*TODO: tbc...*/
	default:
		return HAL_INVALID;
	}

	return ret;
}

HAL_Status HAL_Flash_InitLater(uint32_t flash, FlashBoardCfg *cfg)
{
	struct FlashDev *dev = getFlashDev(flash);

	if (!cfg) {
		FD_ERROR("cfg is NULL!");
		return HAL_ERROR;
	}

#if (defined(FLASH_XIP_OPT_ERASR))
	HAL_Flash_Ioctl(flash, FLASH_SET_SUSPEND_PARAM, (uint32_t)&cfg->flashc.param);
#endif

	if (!dev->chip->cfg.mSuspendSupport) {
		cfg->flashc.param.optimize_mask &= ~(FLASH_OPTIMIZE_WRITE |
		                                     FLASH_OPTIMIZE_ERASE);
	}

	if (cfg->type == FLASH_DRV_FLASHC) {
		dev->drv->ioctl(dev->chip, FLASH_SET_OPTIMIZE_MASK,
		                cfg->flashc.param.optimize_mask);
	}

	return HAL_OK;
}

#ifdef CONFIG_ROM
#ifdef CONFIG_PM
extern struct soc_device_driver flash_drv;

static int PM_FlashSuspend(struct soc_device *dev, enum suspend_state_t state)
{
#ifdef CONFIG_FLASH_PM_ALLOW_ENTER_PWR_DOWN
	struct FlashDev *fdev;
	struct FlashDrv *drv;
	struct FlashChip *chip;
#endif

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
#ifdef CONFIG_FLASH_PM_ALLOW_ENTER_PWR_DOWN
		fdev = getFlashDev((uint32_t)dev->platform_data);
		drv = fdev->drv;
		chip = fdev->chip;
		if (fdev->usercnt != 0) {
			FD_ERROR("flash still using: %d", fdev->usercnt);
			return -1;
		}
		drv->open(chip);
		/* Reset flash chip is not required to enter power-down mode, and it will clear
		 * some bits of the status register, such as Quad Enable bit,
		 * As flash work in Quad mode, it will cause resume exception if not reinit flash.
		 */
		// chip->reset(chip);
		chip->control(chip, DEFAULT_FLASH_POWERDOWN, NULL);
		drv->close(chip);
#endif
		break;
	default:
		break;
	}

	return 0;
}

static int PM_FlashResume(struct soc_device *dev, enum suspend_state_t state)
{
#ifdef CONFIG_FLASH_PM_ALLOW_ENTER_PWR_DOWN
	struct FlashDev *fdev;
	struct FlashDrv *drv;
	struct FlashChip *chip;
#endif

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		/*
		 * TODO: In order to ensure no exceptions caused by enter
		 * the brom in pm case, it is necessary to exit power-down mode in brom.
		 */
#ifdef CONFIG_FLASH_PM_ALLOW_ENTER_PWR_DOWN
		fdev = getFlashDev((uint32_t)dev->platform_data);
		drv = fdev->drv;
		chip = fdev->chip;
		drv->open(chip);
		defaultReleasePowerDown(chip);
		drv->close(chip);
#endif
		break;
	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_PM */

static void flashReadModeCompatibleCfg(uint32_t flash, FlashBoardCfg *cfg)
{
	struct FlashDev *dev = getFlashDev(flash);
	if (dev == NULL) {
		FD_ERROR("invalid dev");
		return;
	}

	FlashReadMode rmod = cfg->mode;

	while (rmod != FLASH_READ_NORMAL_MODE) {
		if (dev->chip->cfg.mReadSupport & rmod) { /* if support this rmod */
			break;
		}
		rmod >>= 1; /* decrease the level of rmod */
		FD_INFO("Decrease rmode level");
	}
	dev->rmode = rmod;

	if (dev->rmode & (FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE |
	    FLASH_READ_QPI_MODE)) {
		dev->drv->open(dev->chip);

		dev->chip->switchReadMode(dev->chip, dev->rmode);
		HAL_Flash_WaitCompl(dev, 5000);

		if (dev->rmode & FLASH_READ_QPI_MODE) {
			dev->chip->enableQPIMode(dev->chip);
		}

		dev->drv->close(dev->chip);
	}
}

/**
  * @brief Initializes flash Device.
  * @note The flash device configuration is in the board_config g_flash_cfg.
  *       Device number is the g_flash_cfg vector sequency number.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Flash_Init(uint32_t flash, FlashBoardCfg *cfg)
{
	extern HAL_Status __HAL_Flash_Init(uint32_t flash, FlashBoardCfg *cfg);

	HAL_Status ret;

#ifdef CONFIG_PM
	flash_drv.suspend_noirq = PM_FlashSuspend;
	flash_drv.resume_noirq = PM_FlashResume;
	flash_drv.enter_latency = 15;   /* at CPU 160MHz */
	flash_drv.exit_latency = 6;     /* at CPU 160MHz */
#endif
	ret = __HAL_Flash_Init(flash, cfg);

	flashReadModeCompatibleCfg(flash, cfg);

#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
	if (!flashwpBlockLockInit(flash)) {
		FD_INFO("blk protect init ok");
	} else {
		FD_ERROR("blk protect init err");
	}
#endif
	return ret;
}

#if (defined(CONFIG_FLASH_POWER_DOWN_PROTECT) || defined(CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ))
HAL_Status HAL_Flash_Erase(uint32_t flash, FlashEraseMode blk_size,
                           uint32_t addr, uint32_t blk_cnt)
{
#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
	struct FlashDev *dev = getFlashDev(flash);
	if (NULL == dev) {
		FD_ERROR("Invalid dev");
		return HAL_INVALID;
	}
#endif

#ifdef CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ
#if FLASH_ERASE_BLOCK_SIZE_ADAPTIVE
	/* In order to reduce the time of continuous closing irq, minimal erase mode is adopted */
	if (blk_size == FLASH_ERASE_32KB || blk_size == FLASH_ERASE_64KB) {
#ifndef CONFIG_FLASH_POWER_DOWN_PROTECT
		struct FlashDev *dev = getFlashDev(flash);
		if (NULL == dev) {
			FD_ERROR("Invalid dev");
			return HAL_INVALID;
		}
#endif
		FlashEraseMode min_erase_size = dev->chip->minEraseSize(dev->chip);

		if (min_erase_size < blk_size) {
			blk_cnt = blk_size * blk_cnt / min_erase_size;
			blk_size = min_erase_size;
		}
	}
#endif /* FLASH_ERASE_BLOCK_SIZE_ADAPTIVE */
#endif /* CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ */

#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
	int ret;

	FlashChipWpCfg *WpCfg = (FlashChipWpCfg *)dev->chip->mWpCfg;

	if (!WpCfg || !WpCfg->mWpBlockLockCfg) {
		return __HAL_Flash_Erase(flash, blk_size, addr, blk_cnt);
	}

	FD_DEBUG("%u: e%u * %u, a: 0x%x", flash, (uint32_t)blk_size, blk_cnt, addr);

	if ((addr + blk_size * blk_cnt) > dev->chip->cfg.mSize) {
		FD_ERROR("space overflow");
		return HAL_INVALID;
	}

	if (addr % blk_size) {/* 4K align at least */
		FD_ERROR("addr not align, 0x%x, %u", addr, blk_size);
		return HAL_INVALID;
	}

	if (blk_size == FLASH_ERASE_CHIP) {
		if (blk_cnt != 1) {
			FD_ERROR("blk_cnt %d", blk_cnt);
			return HAL_INVALID;
		} else {
			return __HAL_Flash_Erase(flash, blk_size, addr, blk_cnt);
		}
	}

	/* set to lock all blocks */
	ret = flashwpSetAllBlocksState(dev, FLASH_BLOCK_STATE_LOCK);
	if (ret) {
		FD_ERROR("lock all blk err");
		return HAL_ERROR;
	}

	/* flash erase protect process */
	FD_DEBUG("%s(), blk_size %u, addr 0x%x, blk_cnt %d", __func__, blk_size, addr,
	         blk_cnt);
	ret = flashwpErase(flash, WpCfg, blk_size, addr, blk_cnt);

	/* unlock all block, for brom upgrade(E/W) in not powerdown */
	ret = flashwpSetAllBlocksState(dev, FLASH_BLOCK_STATE_UNLOCK);
	if (ret) {
		FD_ERROR("unlock all blk err");
		return ret;
	}

	return ret;
#else
	return __HAL_Flash_Erase(flash, blk_size, addr, blk_cnt);
#endif
}
#endif /* CONFIG_FLASH_POWER_DOWN_PROTECT || CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ */

#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
HAL_Status HAL_Flash_Write(uint32_t flash, uint32_t addr, const uint8_t *data,
                           uint32_t size)
{
	int ret;

	/* param check */
	struct FlashDev *dev = getFlashDev(flash);
	if (NULL == dev) {
		FD_ERROR("Invalid dev");
		return HAL_INVALID;
	}

	FlashChipWpCfg *WpCfg = (FlashChipWpCfg *)dev->chip->mWpCfg;
	if (!WpCfg || !WpCfg->mWpBlockLockCfg) {
		return __HAL_Flash_Write(flash, addr, data, size);
	}

	if ((NULL == dev->chip->pageProgram) || (0 == size) ||
	    (addr + size > dev->chip->cfg.mSize)) {
		FD_ERROR("Invalid param");
		if (dev->chip) {
			FD_ERROR("pp:%p sz:%d ad:%x ms:%d", dev->chip->pageProgram, size, addr,
			         dev->chip->cfg.mSize);
		}
		return HAL_INVALID;
	}

	/* set to lock all blocks */
	ret = flashwpSetAllBlocksState(dev, FLASH_BLOCK_STATE_LOCK);
	if (ret) {
		FD_ERROR("lock all blk err");
		return ret;
	}

	/* write protect process */
	ret = flashwpWrite(flash, WpCfg, addr, data, size);

	/* set to unlock all block, for brom upgrade(E/W) in not powerdown */
	ret = flashwpSetAllBlocksState(dev, FLASH_BLOCK_STATE_UNLOCK);
	if (ret) {
		FD_ERROR("unlock all blk err");
		return ret;
	}

	return ret;
}
#endif /* CONFIG_FLASH_POWER_DOWN_PROTECT */
#endif /* CONFIG_ROM */

#if (CONFIG_CHIP_ARCH_VER == 3)
HAL_Status HAL_Flash_Overwrite_Crypto(uint32_t flash, uint32_t addr,
                                      uint8_t *data, uint32_t size,
                                      uint8_t *key)
{
	HAL_Status ret;

	if (HAL_GlobalGetChipVer() == 0x0A) {
		FD_ERROR("not support flash write crypto\n");
		return HAL_ERROR;
	}

	FC_Crypto_Enable(key);
	ret = HAL_Flash_Overwrite(flash, addr, data, size);
	FC_Clr_Crypto_Infor();

	FlashCryptoRelease(0);

	return ret;
}
#endif

#if FLASH_SPI_ENABLE

/*
	struct SpiFlashDrv
*/
struct SpiFlashDrv {
	SPI_Port port;
	SPI_CS cs;
	SPI_Config config;
};

HAL_Status spiFlashOpen(struct FlashChip *chip)
{
	/*TODO: it should be suspend schedule and deinit xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

HAL_Status spiFlashClose(struct FlashChip *chip)
{
	/*TODO: it should be resume schedule and init xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

#define FD_SPI_WRITE(impl, ins, line_max)                         \
	do {                                                          \
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Transmit);    \
		if (ret != HAL_OK) {                                      \
			FD_ERROR("spi instruction param error");              \
			goto failed;                                          \
		}                                                         \
	} while (0)

#define FD_SPI_READ(impl, ins, line_max)                          \
	do {                                                          \
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Receive);     \
		if (ret != HAL_OK) {                                      \
			FD_ERROR("spi instruction param error");              \
			goto failed;                                          \
		}                                                         \
	} while (0)

static HAL_Status insToSpi(struct SpiFlashDrv *impl, InstructionField *ins,
                           uint32_t line_max, HAL_Status(*fun)(SPI_Port, uint8_t *, uint32_t))
{
	uint8_t *p;

	if (ins) {
		if (ins->line > line_max) {
			return HAL_INVALID;
		}
		if (!ins->pdata && ins->len > 4) {
			return HAL_ERROR;
		}

		p = ins->pdata ? ins->pdata : (uint8_t *)&ins->data;
		return fun(impl->port, p, ins->len);
	} else {
		return HAL_OK;
	}
}

static HAL_Status spiFlashWrite(struct FlashChip *dev, InstructionField *cmd,
                                InstructionField *addr, InstructionField *dummy,
                                InstructionField *data)
{
	struct FlashDrv *drv = dev->mDriver;
	struct SpiFlashDrv *spi_drv = drv->platform_data;
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (drv->sizeToDma - 1)) {
		spi_drv->config.opMode = SPI_OPERATION_MODE_DMA;
	} else {
		spi_drv->config.opMode = SPI_OPERATION_MODE_POLL;
	}

	ret = HAL_SPI_Open(spi_drv->port, spi_drv->cs, &spi_drv->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	HAL_SPI_CS(spi_drv->port, 1);
	FD_SPI_WRITE(spi_drv, cmd, 1);
	if (addr) {
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata) {
			addr->pdata = (uint8_t *)&addr->data;
		}
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) |
		             ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(spi_drv, &naddr, 1);
	}
	FD_SPI_WRITE(spi_drv, dummy, 1);
	FD_SPI_WRITE(spi_drv, data, 1);
	HAL_SPI_CS(spi_drv->port, 0);

failed:
	HAL_SPI_Close(spi_drv->port);
	return ret;
}

static HAL_Status spiFlashRead(struct FlashChip *dev, InstructionField *cmd,
                               InstructionField *addr, InstructionField *dummy,
                               InstructionField *data)
{
	struct FlashDrv *drv = dev->mDriver;
	struct SpiFlashDrv *spi_drv = drv->platform_data;
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (drv->sizeToDma - 1)) {
		spi_drv->config.opMode = SPI_OPERATION_MODE_DMA;
	} else {
		spi_drv->config.opMode = SPI_OPERATION_MODE_POLL;
	}

	ret = HAL_SPI_Open(spi_drv->port, spi_drv->cs, &spi_drv->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	if (data && data->line == 2) {
		HAL_SPI_Config(spi_drv->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
	} else {
		HAL_SPI_Config(spi_drv->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);
	}

	HAL_SPI_CS(spi_drv->port, 1);
	FD_SPI_WRITE(spi_drv, cmd, 1);
	if (addr) {
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata) {
			addr->pdata = (uint8_t *)&addr->data;
		}
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) |
		             ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(spi_drv, &naddr, 1);
	}
	FD_SPI_WRITE(spi_drv, dummy, 1);
	FD_SPI_READ(spi_drv, data, 2);
	HAL_SPI_CS(spi_drv->port, 0);

failed:
	HAL_SPI_Close(spi_drv->port);
	return ret;
}

static HAL_Status spiFlashSetFreq(struct FlashChip *dev, uint32_t freq)
{
	/* TODO: tbc... */
	return HAL_INVALID;
}

void spiFlashMsleep(struct FlashChip *dev, uint32_t ms)
{
	HAL_MSleep(ms);
}

void spiFlashMsleepReuseFlashc(struct FlashChip *dev, uint32_t ms)
{
	HAL_Flashc_Delay(dev->flash_ctrl, ms * 1000);
}

static void spiFlashDestroy(struct FlashChip *dev)   /* to check */
{
	struct SpiFlashDrv *spi_drv = dev->mDriver->platform_data;

	HAL_SPI_Deinit(spi_drv->port);
	HAL_Free(spi_drv);
	HAL_Free(dev->mDriver);
}

/**
  * @internal
  * @brief flash driver control.
  * @param base: Driver.
  * @param attr: flash control cmd.
  * @param arg: flash control arguement
  * @retval HAL_Status: The status of driver
  */
HAL_Status spiFlashIoctl(struct FlashChip *dev, FlashControlCmd cmd,
                         uint32_t arg)
{
	HAL_Status ret = HAL_INVALID;

	switch (cmd) {
	case FLASH_ENABLE_32BIT_ADDR:
		break;
	default:
		break;
	}
	return ret;
}

struct FlashDrv *spiDriverCreate(int dev, FlashBoardCfg *bcfg)
{
#if FLASH_SPI_ENABLE
	/*TODO: check read mode*/
	struct FlashDrv *drv;
	struct SpiFlashDrv *spi_drv;
	SPI_Global_Config spi_param;

	drv = HAL_Malloc(sizeof(struct FlashDrv));
	if (!drv) {
		FD_ERROR("malloc fail");
		return NULL;
	}
	HAL_Memset(drv, 0, sizeof(struct FlashDrv));
	spi_drv = HAL_Malloc(sizeof(struct SpiFlashDrv));
	if (!spi_drv) {
		FD_ERROR("malloc fail");
		HAL_Free(drv);
		return NULL;
	}
	HAL_Memset(spi_drv, 0, sizeof(struct SpiFlashDrv));
	spi_param.mclk = bcfg->clk;
	spi_param.cs_level = bcfg->spi.cs_level;
	HAL_SPI_Init(bcfg->spi.port, &spi_param);
	drv->platform_data = spi_drv;

	spi_drv->port = bcfg->spi.port;
	spi_drv->cs = bcfg->spi.cs;
	spi_drv->config.sclk = bcfg->clk;
	spi_drv->config.firstBit = SPI_TCTRL_FBS_MSB;
	spi_drv->config.mode = SPI_CTRL_MODE_MASTER;
	spi_drv->config.opMode = SPI_OPERATION_MODE_DMA; /*spi0 must be poll on xip;*/
	spi_drv->config.sclkMode = SPI_SCLK_Mode0;

	/* FD_DEBUG("type: %d; port: %d; cs: %d; sclk: %d;",
	             bcfg->type, spi_drv->port, spi_drv->cs, spi_drv->config.sclk); */

	drv->dev = dev;
	drv->open = spiFlashOpen;
	drv->close = spiFlashClose;
	drv->read = spiFlashRead;
	drv->write = spiFlashWrite;
	drv->setFreq = spiFlashSetFreq;
	drv->destroy = spiFlashDestroy;
	drv->ioctl = spiFlashIoctl;
	drv->sizeToDma = FLASH_DMA_TRANSFER_MIN_SIZE;
#if (CONFIG_CHIP_ARCH_VER == 2)
	if (bcfg->spi.port == SPI1) {
		drv->msleep = spiFlashMsleep;
	} else
#endif
	{
		drv->msleep = spiFlashMsleepReuseFlashc;
	}

	return drv;
#endif
}

#endif /* FLASH_SPI_ENABLE */
