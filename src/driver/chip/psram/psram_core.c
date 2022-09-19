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

#ifdef CONFIG_PSRAM

#include <string.h>
#include <stdio.h>

#include "sys/param.h"
#include "sys/xr_debug.h"
#include "sys/io.h"
#include "pm/pm.h"
#include "image/image.h"

#include "../hal_base.h"
#include "driver/chip/hal_dma.h"

#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"

#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#include "_psram.h"

/**
 *    mmc_wait_for_req - start a request and wait for completion
 *    @host: MMC host to start command
 *    @mrq: MMC request to start
 *
 *    Start a new MMC custom command request for a host, and wait
 *    for the command to complete. Does not attempt to parse the
 *    response.
 */
int32_t psram_wait_for_req(struct psram_chip *chip, struct psram_request *mrq)
{
	return HAL_PsramCtrl_Request(chip->ctrl, mrq);
}

int32_t psram_sw_reset(struct psram_chip *chip, uint32_t step)
{
	struct psram_request mrq = { { 0 }, { 0 } };

#if (defined CONFIG_PSRAM_CHIP_SQPI)
	if (chip->type == PSRAM_CHIP_SQPI) {
		if (step == 0) {
			mrq.cmd.opcode = SQ_Reset_Enable;
		} else if (step == 1) {
			mrq.cmd.opcode = SQ_Reset;
		}
		if (chip->buswidth == 1) { /* reset enable */
			mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT;
		} else if (chip->buswidth == 4) {
			mrq.cmd.busconfig = PSRAMC_BUS_CMD_4BIT;
		} else {
			PR_ERR("%s wrong buswidth:%d\n", __func__, chip->buswidth);
			return -1;
		}
		mrq.cmd.dummy = 0;
	}
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	uint32_t value = 0;

	if (chip->type == PSRAM_CHIP_OPI_APS32 || chip->type == PSRAM_CHIP_OPI_APS64) {
		/* global reset */
		mrq.cmd.opcode = Global_Reaet;
		mrq.data.blksz = 1;
		mrq.data.blocks = 2;
		value = 0;
		mrq.data.buff = (uint8_t *)&value;
		mrq.data.flags = PSRAM_DATA_WRITE_SHORT;
	}
#endif
	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

#if (defined CONFIG_PSRAM_CHIP_SQPI)
	if (chip->type == PSRAM_CHIP_SQPI && (step == 1)) {
		chip->buswidth = 1;
	}
#endif
	HAL_UDelay(200); /* 50ns */

	return 0;
}

int32_t psram_get_driver_strength(struct psram_chip *chip, uint32_t drv)
{
	uint8_t mr_val;

	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.cmd.resp = &mr_val;
	mrq.data.blksz = 1;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT;
		mrq.data.busconfig = PSRAMC_BUS_DATA_1BIT;
		break;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case PSRAM_CHIP_OPI_APS32:
	case PSRAM_CHIP_OPI_APS64:
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		break;
#endif
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	return mr_val;
}

int32_t Psram_Read_Info(struct psram_chip *chip)
{
	uint8_t mode_regs[8] = {0};
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;
	mrq.data.blocks = 2;
	mrq.data.flags = PSRAM_DATA_READ_SHORT;
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		mrq.cmd.opcode = SQ_Read_ID;
		mrq.cmd.dummy = 0;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT;
		mrq.data.busconfig = PSRAMC_BUS_DATA_1BIT;
		break;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case PSRAM_CHIP_OPI_APS32:
	case PSRAM_CHIP_OPI_APS64:
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.dummy = 0;
		break;
#endif
	default:
		break;
	}

	for (int i = 0; i < 4; i++) {
		mrq.cmd.resp = &mode_regs[i * 2];
		mrq.cmd.addr = i * 2;
		if (psram_wait_for_req(chip, &mrq) != 0)
			return -1;
	}
	PR_DUMP((const void *)mode_regs, 8);
	chip->mf_id = mode_regs[0];
	chip->kgd = mode_regs[1];

#if ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	{
		chip->die = (p_id_raw >> 7) & 0x1; /* bit7: 1:PASS, 0:FAIL */
	}
#endif

	return 0;
}

#if ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
int32_t psram_set_read_latency(struct psram_chip *chip, uint32_t fixed, uint32_t rlc)
{
	uint8_t rval;
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.cmd.resp = &rval;
	mrq.data.blksz = 1;

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR0;
	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~((1 << 5) | (0x7 << 2));
#if (defined CONFIG_PSRAM_CHIP_OPI64)
	if (chip->type == PSRAM_CHIP_OPI_APS64)
		rval |= (fixed << 5) | ((rlc - 3) << 2);
#else
	rval |= (fixed << 5) | (rlc << 2);
#endif
	PR_DBG("%s,%d set rval:%x\n", __func__, __LINE__, rval);

	mrq.data.buff = &rval;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.cmd.addr = MR0;
	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}
#endif

void psram_idbus_op_cmd(struct psram_chip *chip, uint32_t opcmd)
{
	uint32_t write = 0, opcfg = 0;
	uint32_t wait = 0, waitcfg = 0;

	switch (opcmd) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case S_READ:
		opcfg = SQ_Read << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT | PSRAMC_BUS_DATA_1BIT;
		break;
	case S_FAST_READ:
		opcfg = SQ_Fast_Read << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT | PSRAMC_BUS_DUMY_1BIT |
		        PSRAMC_BUS_DUMY_WID(8) | PSRAMC_BUS_DATA_1BIT;
		break;
	case S_FAST_READ_QUAD:
		opcfg = SQ_Fast_Read_Quad << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_4BIT | PSRAMC_BUS_DATA_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case S_WRITE:
		write = 1;
		opcfg = SQ_Write << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT | PSRAMC_BUS_DATA_1BIT;
		break;
	case S_QAUD_WRITE:
		write = 1;
		opcfg = SQ_Quad_Write << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_4BIT | PSRAMC_BUS_DATA_4BIT;
		break;
	case Q_FAST_READ:
		opcfg = SQ_Fast_Read << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_4BIT | PSRAMC_BUS_ADDR_4BIT | PSRAMC_BUS_DATA_4BIT;
		wait = 1;
		waitcfg = (4U << 24);
		break;
	case Q_FAST_READ_QUAD:
		opcfg = SQ_Fast_Read_Quad << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_4BIT | PSRAMC_BUS_ADDR_4BIT | PSRAMC_BUS_DATA_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case Q_WRITE:
		write = 1;
		opcfg = SQ_Write << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_4BIT | PSRAMC_BUS_ADDR_4BIT | PSRAMC_BUS_DATA_4BIT;
		break;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case O_SYNC_READ:
		opcfg = Sync_Read << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DATA_8BIT;
		break;
	case O_SYNC_WRITE:
		write = 1;
		opcfg = Sync_Write << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DATA_8BIT;
		break;
	case O_SYNC_BURST_READ:
		opcfg = Sync_Burst_Read << 24 |
		        PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DATA_8BIT;
		break;
	case O_SYNC_BURST_WRITE:
		write = 1;
		opcfg = Sync_Burst_Write << PSRAMC_BUS_RW_CMD_SHIFT |
		        PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DATA_8BIT;
		break;
#endif
	default:
		PR_ERR("%s,%d unkonw cmd:0x%x\n", __func__, __LINE__, opcmd);
		return;
	}
	if (write)
		chip->cbus_wcmd = opcfg >> PSRAMC_BUS_RW_CMD_SHIFT;
	else
		chip->cbus_rcmd = opcfg >> PSRAMC_BUS_RW_CMD_SHIFT;
	HAL_PsramCtrl_IDbusCfg(chip->ctrl, write, opcfg, wait, waitcfg);
}

int32_t psram_set_driver_strength(struct psram_chip *chip, uint32_t drv)
{
	uint8_t mr_val = 0;
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.cmd.resp = &mr_val;
	mrq.data.blksz = 1;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		mrq.cmd.dummy = 8;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT;
		mrq.data.busconfig = PSRAMC_BUS_DATA_1BIT;
		break;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case PSRAM_CHIP_OPI_APS32:
	case PSRAM_CHIP_OPI_APS64:
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
#endif
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	mr_val &= 0xFC;
	mr_val |= drv;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		mrq.cmd.opcode = SQ_Mode_Reg_Write;
		mrq.cmd.addr = MR0;
		mrq.cmd.dummy = 0;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.data.buff = &mr_val;
		mrq.data.busconfig = PSRAMC_BUS_DATA_1BIT;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case PSRAM_CHIP_OPI_APS32:
	case PSRAM_CHIP_OPI_APS64:
		mrq.cmd.opcode = Mode_Reg_Write;
		/* PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_DATA_8BIT */
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.addr = MR0;
		break;
#endif
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		mrq.cmd.dummy = 8;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.data.busconfig = PSRAMC_BUS_DATA_1BIT;
		break;
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
	case PSRAM_CHIP_OPI_APS32:
	case PSRAM_CHIP_OPI_APS64:
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
#endif
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	return 0;
}

#if ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
int32_t psram_enter_hsleep_mode(struct psram_chip *chip)
{
	PR_DBG("sqpi do nothing when suspend\n");
	uint8_t rval;
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;
	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.cmd.addr = MR6;
	rval = 0xF0;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d success\n", __func__, __LINE__);

	return 0;
}

int32_t psram_enter_dpdown_mode(struct psram_chip *chip)
{
	uint8_t rval;
	struct psram_request mrq = { { 0 }, { 0 } };

	rval = 0xC0;
	mrq.data.blksz = 1;
	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.cmd.addr = MR6;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;
	PR_DBG("%s,%d success\n", __func__, __LINE__);

	return 0;
}

int32_t psram_set_write_latency(struct psram_chip *chip, uint32_t p_type, uint32_t wlc)
{
	uint8_t rval;
#if (defined CONFIG_PSRAM_CHIP_OPI64)
	uint8_t wlc_tables[5] = {0, 4, 2, 6, 1};
#endif
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR4;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(0xe0);
#if (defined CONFIG_PSRAM_CHIP_OPI32)
	if (p_type == PSRAM_CHIP_OPI_APS32)
		rval |= wlc << 6;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	if (p_type == PSRAM_CHIP_OPI_APS64)
		rval |= wlc_tables[wlc - 3] << 5;
#endif
	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_drv(struct psram_chip *chip, uint32_t drv)
{
	uint8_t rval;
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR0;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(0x3);
	rval |= drv;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_rf(struct psram_chip *chip, uint32_t fast_en)
{
	uint8_t rval;
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR4;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(1 << 3);
	rval |= fast_en << 3;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}
#endif

#if (defined CONFIG_PSRAM_CHIP_SQPI)
int32_t psram_set_wrap_dbt(struct psram_chip *chip, uint32_t m_type)
{
	struct psram_request mrq = { { 0 }, { 0 } };

	mrq.data.blksz = 1;

	/* reset enable */
	if (m_type == S_RST) {
		mrq.cmd.opcode = SQ_Wrap;
		mrq.data.blocks = 0;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_1BIT;
	} else if (m_type == Q_RST) {
		mrq.cmd.opcode = SQ_Wrap;
		mrq.data.blocks = 0;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.busconfig = PSRAMC_BUS_CMD_4BIT;
	}

	if (psram_wait_for_req(chip, &mrq) != 0) {
		return -1;
	}

	return 0;
}
#endif

static struct psram_chip *_chip_priv;

struct psram_chip *psram_open(uint32_t id)
{
	if (_chip_priv)
		_chip_priv->ref++;

	return _chip_priv;
}

HAL_Status psram_close(struct psram_chip *chip)
{
	if (chip)
		chip->ref--;

	return HAL_OK;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PSRAM_PM_DATA_CHECK
static __inline uint16_t psram_data_checksum(void)
{
	return image_checksum16((uint8_t *)PSRAM_START_ADDR, (uint32_t)PSRAM_END_ADDR);
}
#endif

static int psram_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	int ret = 0;
	struct psram_chip *chip = dev->platform_data;

#ifdef CONFIG_PSRAM_PM_DATA_CHECK
	if (state > PM_MODE_SLEEP)
		chip->pm_checksum = psram_data_checksum();
#endif

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
		case PSRAM_CHIP_SQPI:
			ret = psram_sqpi_suspend(chip);
			break;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
		case PSRAM_CHIP_OPI_APS32:
			ret = psram_enter_hsleep_mode(chip);
			break;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
		case PSRAM_CHIP_OPI_APS64:
			ret = psram_enter_hsleep_mode(chip);
			break;
#endif
		default:
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}

static int psram_resume(struct soc_device *dev, enum suspend_state_t state)
{
	int32_t ret = 0;
	struct psram_chip *chip = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
		case PSRAM_CHIP_SQPI:
			ret = psram_sqpi_resume(chip);
			break;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
		case PSRAM_CHIP_OPI_APS32:
			ret = psram_exit_hsleep_mode(chip);
			break;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
		case PSRAM_CHIP_OPI_APS64:
			ret = psram_exit_hsleep_mode(chip);
			break;
#endif
		default:
			break;
		}
		break;
	default:
		break;
	}

#ifdef CONFIG_PSRAM_PM_DATA_CHECK
	if (state > PM_MODE_SLEEP) {
		if (psram_data_checksum() != chip->pm_checksum) {
			PR_ERR("ERROR: data checksum failed\n");
			return -1;
		}
	}
#endif

	return ret;
}

static const struct soc_device_driver psram_drv = {
	.name = "psram",
	.suspend_noirq = psram_suspend,
	.resume_noirq = psram_resume,
};

static struct soc_device psram_dev = {
	.name = "psram",
	.driver = &psram_drv,
};
#endif

int32_t psram_init(struct psram_chip *chip, struct psram_ctrl *ctrl, PSRAMChip_InitParam *param)
{
	int32_t ret = -1;

	chip->type = param->p_type;
	chip->ctrl = ctrl;
	chip->freq = param->freq;

	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		ret = psram_sqpi_init(chip, ctrl);
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	case PSRAM_CHIP_OPI_APS32:
		ret = psram_aps32_init(chip, ctrl);
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	case PSRAM_CHIP_OPI_APS64:
		ret = psram_aps64_init(chip, ctrl);
		break;
#endif
	default:
		break;
	}
	_chip_priv = chip;
#ifdef CONFIG_PM
	psram_dev.platform_data = chip;
	pm_register_ops(&psram_dev);
#endif
	PR_INF("%s @:%p ret:%d\n", __func__, chip, ret);

	return ret;
}

int32_t psram_deinit(struct psram_chip *chip)
{
	switch (chip->type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	case PSRAM_CHIP_OPI_APS32:
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	case PSRAM_CHIP_OPI_APS64:
		break;
#endif
	}
	_chip_priv = NULL;
	memset(chip, 0, sizeof(struct psram_chip));
#ifdef CONFIG_PM
	pm_unregister_ops(&psram_dev);
#endif

	return 0;
}

void psram_info_dump(struct psram_chip *chip)
{
	if (!chip)
		return;

	PR_INF("Chip @        : %p\n", chip);
	PR_INF("    Type      : %s\n", chip->name);
#if 0
	PR_INF("    Die       : 0x%x\n", chip->die);
#endif
	PR_INF("    speed     : %u KHz\n", chip->freq / 1000);
	PR_INF("    capacity  : %dKB\n", chip->capacity / 1024);
	PR_INF("    bus_width : %d\n", chip->buswidth);
	PR_INF("    wrap_len  : %d\n", chip->wrap_len);
}

#endif /* CONFIG_PSRAM */
