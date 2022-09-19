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
#include "driver/chip/hal_xip.h"
#include "flash_debug.h"

typedef enum {
	FLASH_INSTRUCTION_WREN = 0x06,                /* write enable */
	FLASH_INSTRUCTION_WRDI = 0x04,                /* write disable */
	FLASH_INSTRUCTION_RDID = 0x9F,                /* jedec id */
	FLASH_INSTRUCTION_RDSR1 = 0x05,               /* read status register-1 */
	FLASH_INSTRUCTION_WRSR1 = 0x01,               /* write status register-1 */
	FLASH_INSTRUCTION_READ = 0x03,                /* read data */
	FLASH_INSTRUCTION_FAST_READ = 0x0B,           /* fast read */
	FLASH_INSTRUCTION_PP = 0x02,                  /* page program */
	FLASH_INSTRUCTION_ERASE_64KB = 0xD8,          /* erase block(sector) 64k */
	FLASH_INSTRUCTION_ERASE_32KB = 0x52,          /* erase block(sector) 32k */
	FLASH_INSTRUCTION_ERASE_4KB = 0x20,           /* erase sector 4k */
	FLASH_INSTRUCTION_ERASE_CHIP = 0xC7,          /* chip erase */
	FLASH_INSTRUCTION_WRSR = 0X01,                /* write status register */
	FLASH_INSTRUCTION_FAST_READ_DO = 0x3B,        /* fast read dual output */
	FLASH_INSTRUCTION_RDSR2 = 0x35,
	FLASH_INSTRUCTION_RDSR3 = 0x15,
	FLASH_INSTRUCTION_WRSR2 = 0x31,
	FLASH_INSTRUCTION_WRSR3 = 0x11,
	FLASH_INSTRUCTION_RDSFDP = 0x5A,              /* read SFDP */
	FLASH_INSTRUCTION_SRWREN = 0x50,              /* write enable for volatile */
	FLASH_INSTRUCTION_CE = 0x60,
	FLASH_INSTRUCTION_EPSP = 0x75,
	FLASH_INSTRUCTION_EPRS = 0x7A,
	FLASH_INSTRUCTION_PWDN = 0xB9,
	FLASH_INSTRUCTION_REL = 0xAB,
	FLASH_INSTRUCTION_FAST_READ_DIO = 0xBB,
	FLASH_INSTRUCTION_FAST_READ_QO = 0x6B,
	FLASH_INSTRUCTION_FAST_READ_QIO = 0xEB,
	FLASH_INSTRUCTION_EN_QPI = 0x38,
	FLASH_INSTRUCTION_DIS_QPI = 0xFF,
	FLASH_INSTRUCTION_RSEN = 0x66,
	FLASH_INSTRUCTION_RESET = 0x99,
	FLASH_INSTRUCTION_QPP = 0x32,
	FLASH_INSTRUCTION_SRP = 0xC0,
	FLASH_INSTRUCTION_BLOCK_LOCK = 0x36,           /* block lock mode command */
	FLASH_INSTRUCTION_BLOCK_UNLOCK = 0x39,
	FLASH_INSTRUCTION_RD_BLOCKLOCK = 0x3D,
	FLASH_INSTRUCTION_ALL_BLOCK_LOCK = 0x7E,
	FLASH_INSTRUCTION_ALL_BLOCK_UNLOCK = 0x98,
} eSF_Instruction;

/* internal macros for flash chip instruction */
#define FCI_CMD(idx)    instruction[idx]
#define FCI_ADDR(idx)   instruction[idx]
#define FCI_DUMMY(idx)  instruction[idx]
#define FCI_DATA(idx)   instruction[idx]

#ifdef FLASH_DEFAULTCHIP
extern FlashChipCtor DefaultFlashChip;
#endif
#ifdef FLASH_XT25F16B
extern FlashChipCtor  XT25F16B_FlashChip;
#endif
#ifdef FLASH_XT25F32B
extern FlashChipCtor  XT25F32B_FlashChip;
#endif
#ifdef FLASH_XT25F64B
extern FlashChipCtor  XT25F64B_FlashChip;
#endif
#ifdef FLASH_P25Q80H
extern FlashChipCtor  P25Q80H_FlashChip;
#endif
#ifdef FLASH_P25Q40H
extern FlashChipCtor  P25Q40H_FlashChip;
#endif
#ifdef FLASH_P25Q16H
extern FlashChipCtor  P25Q16H_FlashChip;
#endif
#ifdef FLASH_P25Q32H
extern FlashChipCtor  P25Q32H_FlashChip;
#endif
#ifdef FLASH_P25Q64H
extern FlashChipCtor  P25Q64H_FlashChip;
#endif
#if defined(FLASH_EN25QH16A) || defined(FLASH_EN25QH16B)
extern FlashChipCtor  EN25QH16X_FlashChip;
#endif
#ifdef FLASH_EN25QH64A
extern FlashChipCtor  EN25QH64A_FlashChip;
#endif
#ifdef FLASH_XM25QH64A
extern FlashChipCtor  XM25QH64A_FlashChip;
#endif

FlashChipCtor *flashChipList[] = {
#ifdef FLASH_DEFAULTCHIP
	&DefaultFlashChip, /*default chip must be at the first*/
#endif
#ifdef FLASH_XT25F16B
	&XT25F16B_FlashChip,
#endif
#ifdef FLASH_XT25F32B
	&XT25F32B_FlashChip,
#endif
#ifdef FLASH_XT25F64B
	&XT25F64B_FlashChip,
#endif
#ifdef FLASH_P25Q80H
	&P25Q80H_FlashChip,
#endif
#ifdef FLASH_P25Q40H
	&P25Q40H_FlashChip,
#endif
#ifdef FLASH_P25Q16H
	&P25Q16H_FlashChip,
#endif
#ifdef FLASH_P25Q32H
	&P25Q32H_FlashChip,
#endif
#ifdef FLASH_P25Q64H
	&P25Q64H_FlashChip,
#endif
#if defined(FLASH_EN25QH16A) || defined(FLASH_EN25QH16B)
	&EN25QH16X_FlashChip,
#endif
#ifdef FLASH_EN25QH64A
	&EN25QH64A_FlashChip,
#endif
#ifdef FLASH_XM25QH64A
	&XM25QH64A_FlashChip,
#endif
};

const FlashChipCtor **FlashChipGetChipList(int *len)
{
	*len = HAL_ARRAY_SIZE(flashChipList);
	return (const FlashChipCtor **)flashChipList;
}

#ifdef CONFIG_ROM
int defaultReleasePowerDown(struct FlashChip *chip)
{
	InstructionField cmd;

	if (!chip) {
		printf("chip is NULL!");
		return HAL_INVALID;
	}

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_REL;
	return chip->driverWrite(chip, &cmd, NULL, NULL, NULL);
}

/* need not to write disable */
int defaultWriteSREnableForVolatile(struct FlashChip *chip)
{
	PCHECK(chip);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));

	cmd.data = FLASH_INSTRUCTION_SRWREN;
	cmd.line = 1;

	return chip->driverWrite(chip, &cmd, NULL, NULL, NULL);
}

int defaultGetSFDP(struct FlashChip *chip, uint32_t raddr, uint8_t *rdata,
                   uint32_t rlen)
{
	PCHECK(chip);
	InstructionField instruction[4];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FCI_CMD(0).data = FLASH_INSTRUCTION_RDSFDP;
	FCI_ADDR(1).line = 1;
	FCI_ADDR(1).data = raddr;
	FCI_DUMMY(2).len = 1;
	FCI_DUMMY(2).line = 1;
	FCI_DATA(3).line = 1;
	FCI_DATA(3).pdata = rdata;
	FCI_DATA(3).len = rlen;

	return chip->driverRead(chip, &FCI_CMD(0), &FCI_ADDR(1), &FCI_DUMMY(2),
	                        &FCI_DATA(3));
}

#ifdef CONFIG_FLASH_POWER_DOWN_PROTECT
int defaultUnLockBlock(struct FlashChip *chip, uint32_t addr)
{
	PCHECK(chip);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FD_DEBUG("UnLock addr is 0x%x\n", addr);
	FCI_CMD(0).data = FLASH_INSTRUCTION_BLOCK_UNLOCK;
	FCI_ADDR(1).data = addr;
	FCI_ADDR(1).line = 1;

	return chip->driverWrite(chip, &FCI_CMD(0), &FCI_ADDR(1), NULL, NULL);
}

int defaultLockBlock(struct FlashChip *chip, uint32_t addr)
{
	PCHECK(chip);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FD_DEBUG("Lock addr is 0x%x\n", addr);
	FCI_CMD(0).data = FLASH_INSTRUCTION_BLOCK_LOCK;
	FCI_ADDR(1).data = addr;
	FCI_ADDR(1).line = 1;

	return chip->driverWrite(chip, &FCI_CMD(0), &FCI_ADDR(1), NULL, NULL);
}

int defaultSetGlobalBlockLockState(struct FlashChip *chip,
                                   FlashBlockLockState state)
{
	PCHECK(chip);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));

	if (state == FLASH_BLOCK_STATE_LOCK) {
		cmd.data = FLASH_INSTRUCTION_ALL_BLOCK_LOCK;
	} else if (state == FLASH_BLOCK_STATE_UNLOCK) {
		cmd.data = FLASH_INSTRUCTION_ALL_BLOCK_UNLOCK;
	} else {
		FLASH_NOWAY();
	}
	cmd.line = 1;

	return chip->driverWrite(chip, &cmd, NULL, NULL, NULL);
}

int defaultReadBlockLockStatus(struct FlashChip *chip, uint32_t raddr,
                               uint8_t *rdata)
{
	PCHECK(chip);
	InstructionField instruction[3];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	if (raddr > chip->cfg.mSize) {
		return -1;
	}

	FCI_CMD(0).data = FLASH_INSTRUCTION_RD_BLOCKLOCK;
	FCI_ADDR(1).line = 1;
	FCI_ADDR(1).data = raddr;
	FCI_DATA(2).line = 1;
	FCI_DATA(2).pdata = rdata;
	FCI_DATA(2).len = 1;

	return chip->driverRead(chip, &FCI_CMD(0), &FCI_ADDR(1), NULL, &FCI_DATA(2));
}
#endif /* CONFIG_FLASH_POWER_DOWN_PROTECT */
#endif /* CONFIG_ROM */
