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

#include "driver/chip/flashchip/flash_chip.h"
#include "driver/chip/hal_flash.h"
#include "../hal_base.h"
#include "flash_debug.h"

#define EN25QH16X_JEDEC    0x15701C /* for EN25QH16A and EN25QH16B(2A) */

#define EN25QH16X_SFDP_RMOD_QUAD_O_BIT    HAL_BIT(6)
#define EN25QH16X_SFDP_RMOD_PARAM_ADDR    (0x32)

/* internal macros for flash chip instruction */
#define FCI_CMD(idx)    instruction[idx]
#define FCI_ADDR(idx)   instruction[idx]
#define FCI_DUMMY(idx)  instruction[idx]
#define FCI_DATA(idx)   instruction[idx]

typedef enum {
	FLASH_INSTRUCTION_RDSFDP = 0x5A,
} eSF_Instruction;

__sram_rodata
static const FlashChipCfg _EN25QH16X_FlashChipCfg = {
	/* FLASH_EN25QH16A or FLASH_EN25QH16B */
	.mJedec = 0x15701C,
	.mSize = 32 * 16 * 0x1000,
	.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
	.mPageProgramSupport = FLASH_PAGEPROGRAM,
	.mReadStausSupport = FLASH_STATUS1,
	.mWriteStatusSupport = FLASH_STATUS1,
	.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE |
	                FLASH_READ_DUAL_IO_MODE | /*FLASH_READ_QUAD_O_MODE |*/ FLASH_READ_QUAD_IO_MODE,
	.mMaxFreq = 104 * 1000 * 1000,
	.mMaxReadFreq = 50 * 1000 * 1000,
	.mSuspendSupport = 0,
	.mSuspend_Latency = 0,
	.mResume_Latency = 0,
};

static int EN25QH16X_FlashInit(struct FlashChip *chip)
{
	PCHECK(chip);

	chip->writeEnable = defaultWriteEnable;
	chip->writeDisable = defaultWriteDisable;
	chip->readStatus = defaultReadStatus;
	chip->erase = defaultErase;
	chip->jedecID = defaultGetJedecID;
	chip->pageProgram = defaultPageProgram;
	chip->read = defaultRead;

	chip->driverWrite = defaultDriverWrite;
	chip->driverRead = defaultDriverRead;
	chip->setFreq = defaultSetFreq;
	chip->switchReadMode = defaultSwitchReadMode;
	chip->xipDriverCfg = defaultXipDriverCfg;
	chip->enableXIP = defaultEnableXIP;
	chip->disableXIP = defaultDisableXIP;
	chip->isBusy = defaultIsBusy;
	chip->control = defaultControl;
	chip->minEraseSize = defaultGetMinEraseSize;
	chip->writeStatus = defaultWriteStatus;
	chip->enableQPIMode = defaultEnableQPIMode;
	chip->disableQPIMode = defaultDisableQPIMode;
	/* chip->enableReset = defaultEnableReset; */
	chip->reset = defaultReset;

	chip->suspendErasePageprogram = defaultSuspendErasePageprogram;
	chip->resumeErasePageprogram = defaultResumeErasePageprogram;
	chip->isSuspend = defaultIsSuspend;
	chip->powerDown = NULL;
	chip->releasePowerDown = NULL;
	chip->uniqueID = NULL;

	return 0;
}

static int EN25QH16X_FlashDeinit(struct FlashChip *chip)
{
	PCHECK(chip);

	/* HAL_Free(chip); */

	return 0;
}

static int getReadmodeSupport(struct FlashChip *chip, uint8_t *rdata)
{
	PCHECK(chip);
	InstructionField instruction[4];
	int ret = 0;

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FCI_CMD(0).data = FLASH_INSTRUCTION_RDSFDP;
	FCI_CMD(0).len = 1;
	FCI_CMD(0).line = 1;
	FCI_ADDR(1).data = EN25QH16X_SFDP_RMOD_PARAM_ADDR;
	FCI_ADDR(1).len = 3;
	FCI_ADDR(1).line = 1;
	FCI_DUMMY(2).len = 1;
	FCI_DUMMY(2).line = 1;
	FCI_DATA(3).line = 1;
	FCI_DATA(3).pdata = rdata;
	FCI_DATA(3).len = 1;

	chip->mDriver->open(chip);
	ret = chip->mDriver->read(chip, &FCI_CMD(0), &FCI_ADDR(1), &FCI_DUMMY(2),
	                          &FCI_DATA(3));
	chip->mDriver->close(chip);

	return ret;
}

static void EN25QH16X_AdaptReadMode(struct FlashChip *chip)
{
	uint8_t rmod_param;

	int ret = getReadmodeSupport(chip, &rmod_param);
	if (ret) {
		FLASH_ERROR("get SFDP rmod err(%d)", ret);
		return;
	}

	if (rmod_param & EN25QH16X_SFDP_RMOD_QUAD_O_BIT) {
		chip->cfg.mReadSupport |= FLASH_READ_QUAD_O_MODE;
	}
}

static struct FlashChip *EN25QH16X_FlashCtor(struct FlashChip *chip,
                                             uint32_t arg)
{
	uint32_t jedec = arg;
	PCHECK(chip);

	if (jedec != EN25QH16X_JEDEC) {
		return NULL;
	}

	HAL_Memcpy(&chip->cfg, &_EN25QH16X_FlashChipCfg, sizeof(FlashChipCfg));

	chip->mPageSize = 256;
	chip->mFlashStatus = 0;
	chip->mDummyCount = 1;

	/*
	 * Since EN25QH16A and EN25QH16B(2A) have the same jedec value,
	 * but have different Quad output fast read mode support,
	 * here we distinguish this by reading the read mode parameter in SFDP.
	 */
	EN25QH16X_AdaptReadMode(chip);

	return chip;
}

FlashChipCtor EN25QH16X_FlashChip = {
	.mJedecId = EN25QH16X_JEDEC,
	.enumerate = EN25QH16X_FlashCtor,
	.init = EN25QH16X_FlashInit,
	.destory = EN25QH16X_FlashDeinit,
};
