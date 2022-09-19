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
#include <stdio.h>

#include "sys/xr_debug.h"
#include "sys/io.h"
#include "pm/pm.h"

#include "../hal_base.h"

#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"

#include "driver/chip/hal_xip.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_gpio.h"

#define CONFIG_PSRAM_DMA_USED
#define PRC_DMA_TIMEOUT         2000

#define DBG_PRC_DBG 0
#define DBG_PRC 1

#if (DBG_PRC == 1)
#define PRC_LOG(flags, fmt, arg...) \
	do {                            \
		if (flags)                  \
			printf(fmt, ##arg);     \
	} while (0)

/*#define PRC_DUMP_REGS() print_hex_dump_words((const void *)PSRAM_CTRL_BASE, 0x200)*/

#define PRC_DUMP_REGS() \
	{ \
		printf("PSRAM CTRL base addr\t[0x%08x]\n", (uint32_t)PSRAM_CTRL); \
		printf("MEM_COM_CFG:\t\t0x%08x\n", PSRAM_CTRL->MEM_COM_CFG);\
		printf("OPI_CTRL_COM_CFG:\t0x%08x\n", PSRAM_CTRL->OPI_CTRL_COM_CFG);\
		printf("CACHE_RLVT_CFG:\t\t0x%08x\n", PSRAM_CTRL->CACHE_RLVT_CFG);\
		printf("MEM_AC_CFG:\t\t0x%08x\n", PSRAM_CTRL->MEM_AC_CFG);\
		printf("C_RD_OPRT_CFG:\t\t0x%08x\n", PSRAM_CTRL->C_RD_OPRT_CFG);\
		printf("C_WD_OPRT_CFG:\t\t0x%08x\n", PSRAM_CTRL->C_WD_OPRT_CFG);\
		printf("C_RD_DUMMY_DATA_H:\t0x%08x\n", PSRAM_CTRL->C_RD_DUMMY_DATA_H);\
		printf("C_RD_DUMMY_DATA_L:\t0x%08x\n", PSRAM_CTRL->C_RD_DUMMY_DATA_L);\
		printf("C_WD_DUMMY_DATA_H:\t0x%08x\n", PSRAM_CTRL->C_WD_DUMMY_DATA_H);\
		printf("C_WD_DUMMY_DATA_L:\t0x%08x\n", PSRAM_CTRL->C_WD_DUMMY_DATA_L);\
		printf("C_IO_SW_WAIT_TIME:\t0x%08x\n", PSRAM_CTRL->C_IO_SW_WAIT_TIME);\
		printf("S_RW_OPRT_CFG:\t\t0x%08x\n", PSRAM_CTRL->S_RW_OPRT_CFG);\
		printf("S_ADDR_CFG:\t\t0x%08x\n", PSRAM_CTRL->S_ADDR_CFG);\
		printf("S_DUMMY_DATA_H:\t\t0x%08x\n", PSRAM_CTRL->S_DUMMY_DATA_H);\
		printf("S_DUMMY_DATA_L:\t\t0x%08x\n", PSRAM_CTRL->S_DUMMY_DATA_L);\
		printf("S_IO_SW_WAIT_TIME:\t0x%08x\n", PSRAM_CTRL->S_IO_SW_WAIT_TIME);\
		printf("S_WD_DATA_BYTE_NUM:\t0x%08x\n", PSRAM_CTRL->S_WD_DATA_BYTE_NUM);\
		printf("S_RD_DATA_BYTE_NUM:\t0x%08x\n", PSRAM_CTRL->S_RD_DATA_BYTE_NUM);\
		printf("S_START_SEND_REG:\t0x%08x\n", PSRAM_CTRL->S_START_SEND_REG);\
		printf("FIFO_TRIGGER_LEVEL:\t0x%08x\n", PSRAM_CTRL->FIFO_TRIGGER_LEVEL);\
		printf("FIFO_STATUS_REG:\t0x%08x\n", PSRAM_CTRL->FIFO_STATUS_REG);\
		printf("INT_ENABLE_REG:\t\t0x%08x\n", PSRAM_CTRL->INT_ENABLE_REG);\
		printf("INT_STATUS_REG:\t\t0x%08x\n", PSRAM_CTRL->INT_STATUS_REG);\
		printf("XIP_WARP_MODE_EXE_IDCT:\t0x%08x\n", PSRAM_CTRL->XIP_WARP_MODE_EXE_IDCT);\
		printf("MEM_CTRL_DBG_STATE:\t0x%08x\n", PSRAM_CTRL->MEM_CTRL_DBG_STATE);\
		printf("MEM_CTRL_SBUS_DBG_CNTH:\t0x%08x\n", PSRAM_CTRL->MEM_CTRL_SBUS_DBG_CNTH);\
		printf("MEM_CTRL_SBUS_DBG_CNTL:\t0x%08x\n", PSRAM_CTRL->MEM_CTRL_SBUS_DBG_CNTL);\
		printf("PSRAM_FORCE_CFG:\t0x%08x\n", PSRAM_CTRL->PSRAM_FORCE_CFG);\
		printf("PSRAM_COM_CFG:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_COM_CFG);\
		printf("PSRAM_LAT_CFG:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_LAT_CFG);\
		printf("PSRAM_TIM_CFG:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_TIM_CFG);\
		printf("PSRAM_DQS_DELAY_CFG:\t0x%08x\n", PSRAM_CTRL->PSRAM_DQS_DELAY_CFG);\
		printf("S_WD_DATA_REG:\t\t0x%08x\n", PSRAM_CTRL->S_WD_DATA_REG);\
		printf("S_RD_DATA_REG:\t\t0x%08x\n", PSRAM_CTRL->S_RD_DATA_REG);\
		printf("START_ADDR0:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_ADDR[0].START_ADDR);\
		printf("END_ADDR0:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_ADDR[0].END_ADDR);\
		printf("BIAS_ADDR0:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_ADDR[0].BIAS_ADDR);\
	}

#define PRC_DBG(fmt, arg...) PRC_LOG(DBG_PRC_DBG, "[PRC DBG] "fmt, ##arg)
#define PRC_INF(fmt, arg...) PRC_LOG(DBG_PRC, "[PRC INF] "fmt, ##arg)
#define PRC_WRN(fmt, arg...) PRC_LOG(DBG_PRC, "[PRC WRN] "fmt, ##arg)
#define PRC_ERR(fmt, arg...) PRC_LOG(DBG_PRC, "[PRC ERR] "fmt, ##arg)
#else
#define PRC_DUMP_REGS()
#define PRC_INF(...)
#define PRC_WRN(...)
#define PRC_ERR(...)
#endif

#define PRC_WARN_ON(v) do { if (v) { printf("WARN at %s:%d!\n", __func__, __LINE__); } } while (0)
#define PRC_BUG_ON(v) do { if (v) { printf("BUG at %s:%d!\n", __func__, __LINE__); while (1); } } while (0)

static struct psram_ctrl *_psram_priv;

static void __psram_init_io(uint32_t p_type)
{
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_PSRAM, 0), 0);
	PRC_DBG("%s,%d type:%d\n", __func__, __LINE__, p_type);
}

void __psram_deinit_io(uint32_t p_type)
{
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_PSRAM, 0), 0);
	PRC_DBG("%s,%d type:%d\n", __func__, __LINE__, p_type);
}

void HAL_PsramCtrl_Set_RD_BuffSize(PSRAMC_CacheLLCfg size)
{
	HAL_MODIFY_REG(PSRAM_CTRL->CACHE_RLVT_CFG, PSRAMC_CACHE_LL_CFG_MASK, size);
}

void HAL_PsramCtrl_Set_WR_BuffSize(PSRAMC_WrCacheLL size)
{
	HAL_MODIFY_REG(PSRAM_CTRL->CACHE_RLVT_CFG, PSRAMC_WR_CACHE_LINE_LEN_MASK, size);
}

/* clock config: CLK_SRC/N/M
 * SYS_CRYSTAL: 24M
 * 24M: 24M/(2^0)/(1+0) = 24M
 *
 * SYS_CRYSTAL: DEV2
 *  66M: 192M/(2^0)/(1+2) = 64M
 * 109M: 192M/(2^0)/(1+1) = 96M
 * 133M: 192M/(2^0)/(1+1) = 96M
 * 192M: 192M/(2^0)/(1+0) = 192M
 * 200M: 192M/(2^0)/(1+0) = 192M
 * 233M: 192M/(2^0)/(1+0) = 192M
 */
int32_t HAL_PsramCtrl_SetClk(uint32_t clk)
{
	CCM_AHBPeriphClkSrc src;
	uint32_t mclk;
	uint32_t div;
	CCM_PeriphClkDivN div_n = 0;
	CCM_PeriphClkDivM div_m = 0;

	clk = clk * 2;
	if (clk > HAL_GetHFClock()) {
		mclk = HAL_GetDev2Clock();
		src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK; /* same with DEV2 */
	} else {
		mclk = HAL_GetHFClock();
		src = CCM_AHB_PERIPH_CLK_SRC_HFCLK;
	}

	div = (mclk + clk - 1) / clk;
	div = (div == 0) ? 1 : div;

	if (div > (16 * 8))
		return -1;

	if (div > 64) {
		div_n = CCM_PERIPH_CLK_DIV_N_8;
		div_m = (CCM_PeriphClkDivM)((div >> 3) - 1);
	} else if (div > 32) {
		div_n = CCM_PERIPH_CLK_DIV_N_4;
		div_m = (CCM_PeriphClkDivM)((div >> 2) - 1);
	} else if (div > 16) {
		div_n = CCM_PERIPH_CLK_DIV_N_2;
		div_m = (CCM_PeriphClkDivM)((div >> 1) - 1);
	} else {
		div_n = CCM_PERIPH_CLK_DIV_N_1;
		div_m = (CCM_PeriphClkDivM)((div >> 0) - 1);
	}

	HAL_CCM_PSRAMC_DisableMClock();
	HAL_CCM_PSRAMC_SetMClock(src, div_n, div_m);
	HAL_CCM_PSRAMC_EnableMClock();
	PRC_INF("PSRAM CTRL MCLK:%u MHz clock=%u MHz,src:%x, n:%d, m:%d\n", mclk / 1000000,
	        mclk / (1 << div_n) / (div_m + 1) / 1000000, (int)src, (int)div_n, (int)div_m);

	return 0;
}

static int32_t PSRAM_CTRL_PollHandler(void)
{
	struct psram_ctrl *ctrl = _psram_priv;
	struct psram_request *mrq = ctrl->mrq;
	uint32_t data_len;
	uint32_t i;
	uint32_t status_int, inte;
	uint32_t len = 0;
	uint32_t flags = mrq->data.flags;
	int32_t ret = -1;

	PRC_BUG_ON(!ctrl);

	if (mrq) {
		len = mrq->data.blksz * mrq->data.blocks;
	}

	status_int = PSRAM_CTRL->INT_STATUS_REG;
	inte = PSRAM_CTRL->INT_ENABLE_REG;
	ctrl->status_int = status_int;
	ctrl->inte = inte;
	PRC_DBG("%d %s status_int:0x%x enable_int:0x%x\n", __LINE__, __func__, status_int, inte);

	if ((PSRAMC_RD_TOUT_INT_EN | PSRAMC_DMA_WR_CROSS_INT_EN) & status_int) {
		PSRAM_CTRL->INT_STATUS_REG = 0;
		PSRAM_CTRL->INT_STATUS_REG = PSRAMC_RD_TOUT_INT_EN | PSRAMC_DMA_WR_CROSS_INT_EN;
		PRC_ERR("%d %s status_int:0x%x enable_int:0x%x\n", __LINE__, __func__, status_int, inte);
	}

	if ((PSRAMC_ISR_TRANS_END & status_int) && (PSRAMC_IER_TRANS_ENB & inte)) {
		PSRAM_CTRL->INT_ENABLE_REG = 0;
		PSRAM_CTRL->INT_STATUS_REG = PSRAMC_ISR_TRANS_END;
		ret = 0
	}

	return ret;
}

/**
 * Check psram transfer data finish
 */
static int32_t __psram_ctrl_request_done(struct psram_ctrl *ctrl)
{
	int32_t ret = 0;
	struct psram_request *mrq = ctrl->mrq;

	if (mrq && mrq->cmd.resp) {
		uint32_t idx = 0;
		uint32_t len = mrq->data.blksz * mrq->data.blocks;

		while (len--)
			mrq->cmd.resp[idx++] = readb(&PSRAM_CTRL->S_RD_DATA_REG);
	}
	PSRAM_CTRL->PSRAM_COM_CFG &= ~PSRAMC_MR_REG_ADDR_EN;

	return ret;
}

#define PSRAM_WR_CHECK_LEN      (12)
#define PSRAM_MAGIC_WORD_DATA1  (0x00ff00ff)
#define PSRAM_MAGIC_WORD_DATA2  (0x5aa55aa5)
#define PSRAM_MAGIC_WORD_DATA3  (0xa55aa55a)

static int32_t __psram_wr_check(uint32_t addr)
{
	uint32_t *ptr = (uint32_t *)addr;

	for (int i = 0; i < PSRAM_WR_CHECK_LEN; i += 3) {
		ptr[i] = PSRAM_MAGIC_WORD_DATA1;
		ptr[i + 1] = PSRAM_MAGIC_WORD_DATA2;
		ptr[i + 2] = PSRAM_MAGIC_WORD_DATA3;
	}
	HAL_Memset((uint8_t *)0x1500000, 0, 128);
	for (int i = 0; i < PSRAM_WR_CHECK_LEN; i += 3) {
		if (ptr[i] != PSRAM_MAGIC_WORD_DATA1
		    || ptr[i + 1] != PSRAM_MAGIC_WORD_DATA2
		    || ptr[i + 2] != PSRAM_MAGIC_WORD_DATA3
		   ) {
			return -1;
		}
	}
	return 0;
}

int32_t HAL_PsramCtrl_DQS_Delay_Calibration(struct psram_ctrl *ctrl)
{
	uint8_t dqsCal;
	uint8_t minCal = 0;
	uint8_t maxCal = 0;
	int32_t dcacheWT_idx = -1;

	/*no check return*/
	dcacheWT_idx = HAL_Dcache_Enable_WriteThrough(PSRAM_START_ADDR, rounddown2(PSRAM_END_ADDR, 16));
	minCal = 0;
	maxCal = 0;
	for (int j = 0; j < 64; j++) {
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_DQS_DELAY_CFG, PSRAMC_OVERWR_CAL_MASK | PSRAMC_OVERWR_CAL,
		               PSRAMC_OVERWR_CAL_VAL(j) | PSRAMC_OVERWR_CAL);
		HAL_UDelay(10);
		if (!(__psram_wr_check(PSRAM_START_ADDR)
		      || __psram_wr_check(PSRAM_END_ADDR + 1 - PSRAM_WR_CHECK_LEN * sizeof(uint32_t)))) {
			if (minCal == 0) {
				minCal = j;
			}
			maxCal = j;
		} else {
			if ((maxCal != 0) && (minCal != maxCal))
				break;
			else if (minCal != 0) {
				minCal = 0;
				maxCal = 0;
			}
		}
	}

	if (maxCal - minCal < 2) {
		HAL_ERR("%s: Can not find correct dqs\n", __func__);
		goto out;
	}
	dqsCal = (minCal + maxCal) / 2;
	PRC_DBG("%s: minCal=%d, maxCal=%d, set cal=%d\n", __func__, minCal, maxCal, (minCal + maxCal) / 2);
	HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_DQS_DELAY_CFG, PSRAMC_OVERWR_CAL_MASK | PSRAMC_OVERWR_CAL,
	               PSRAMC_OVERWR_CAL_VAL(dqsCal) | PSRAMC_OVERWR_CAL);
	HAL_UDelay(10);
out:
	HAL_Dcache_Disable_WriteThrough(dcacheWT_idx);

	return -1;
}

int32_t HAL_PsramCtrl_Set_DQS_Delay_Cal(uint32_t clk)
{
	uint32_t cal_val;
	uint32_t mclk;
	uint32_t div;

	if (clk > HAL_GetHFClock()) {
		mclk = HAL_GetDev2Clock();
	} else {
		mclk = HAL_GetHFClock();
	}
	div = (mclk + clk - 1) / clk;
	div = (div == 0) ? 1 : div;

	if (div < 2) {
		PSRAM_CTRL->PSRAM_DQS_DELAY_CFG = PSRAMC_OVERWR_CAL_VAL(0x20);
		HAL_UDelay(500);
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_DQS_DELAY_CFG, PSRAMC_OVERWR_CAL, 0);
		PSRAM_CTRL->PSRAM_DQS_DELAY_CFG = PSRAMC_START_DQS_DELAY_CAL;
		while (PSRAM_CTRL->PSRAM_DQS_DELAY_CFG & PSRAMC_START_DQS_DELAY_CAL)
			;
		cal_val = (PSRAM_CTRL->PSRAM_DQS_DELAY_CFG & PSRAMC_CAL_RESULT_VAL_MASK) >> PSRAMC_CAL_RESULT_VAL_SHIFT;
		if (PSRAM_CTRL->PSRAM_DQS_DELAY_CFG & PSRAMC_CAL_SUCCEED)
			cal_val = cal_val / 4 + 1;
		else
			cal_val = cal_val / 4 + 1;
	} else if (div == 2) {
		cal_val = 20;
	} else if (div == 3) {
		cal_val = 26;
	} else if (div == 5) {
		cal_val = 39;
	} else if (div == 16) {
		cal_val = 104;
	} else
		return -1;

	HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_DQS_DELAY_CFG, PSRAMC_OVERWR_CAL_MASK | PSRAMC_OVERWR_CAL,
	               PSRAMC_OVERWR_CAL_VAL(cal_val) | PSRAMC_OVERWR_CAL);

	return 0;
}

void HAL_PsramCtrl_MaxCE_LowCyc(struct psram_ctrl *ctrl, uint32_t clk)
{
	uint32_t ce_cyc = 0;

	ce_cyc = 4 * (clk / 1000 / 1000) - 32;
	ce_cyc = clk < 8000000 ? 1 : ce_cyc;

	HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_COM_CFG, PSRAMC_MAX_CEN_LOW_CYC_MASK,
	               PSRAMC_MAX_CEN_LOW_CYC_NUM(ce_cyc));
}

static void __psram_ctrl_send_cmd(struct psram_ctrl *ctrl, struct psram_command *cmd)
{
#if 0
	PRC_DBG("psram cmd:%d(%x), arg:%x ie:%x wt:%x len:%u\n",
	         (cmd_val & SDXC_CMD_OPCODE), cmd_val, cmd->arg, imask, wait,
	         cmd->data ? cmd->data->blksz * cmd->data->blocks : 0);
#endif
	PSRAM_CTRL->S_START_SEND_REG = 0x1; /* sbus read start */
}

int32_t HAL_PsramCtrl_Request(struct psram_ctrl *ctrl, struct psram_request *mrq)
{
	int32_t ret;
	int32_t len;
	unsigned long iflags;

	len = mrq->data.blksz * mrq->data.blocks;
	PRC_DBG("%s,%d cmd:0x%x addr:0x%x len:%d flag:0x%x bc:0x%x io_wait:%x\n", __func__, __LINE__,
	        mrq->cmd.opcode, mrq->cmd.addr, len, mrq->data.flags, ctrl->busconfig, PSRAM_CTRL->S_IO_SW_WAIT_TIME);

	iflags = HAL_EnterCriticalSection();
	PSRAM_CTRL->S_RW_OPRT_CFG = mrq->cmd.opcode << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
	                            ctrl->busconfig;

	if (mrq->data.flags & PSRAM_DATA_READ_MASK) {
		PSRAM_CTRL->S_ADDR_CFG = mrq->cmd.addr;
		PSRAM_CTRL->S_RD_DATA_BYTE_NUM = len;
		PSRAM_CTRL->PSRAM_COM_CFG |= PSRAMC_MR_REG_ADDR_EN;
	}
	if (mrq->data.flags & PSRAM_DATA_WRITE_MASK) {
		uint32_t value;
		uint8_t *buf;

		PSRAM_CTRL->S_ADDR_CFG = mrq->cmd.addr;
		PSRAM_CTRL->S_WD_DATA_BYTE_NUM = len;
		buf = mrq->data.buff;
		while (len > 0) {
			if (mrq->data.flags & PSRAM_DATA_WRITE_BYTE) {
				value = readb(buf);
				writeb(value, &PSRAM_CTRL->S_WD_DATA_REG);
				len--;
				buf++;
			} else if (mrq->data.flags & PSRAM_DATA_WRITE_SHORT) {
				value = readw(buf);
				writew(value, &PSRAM_CTRL->S_WD_DATA_REG);
				len -= 2;
				buf += 2;
			} else if (mrq->data.flags & PSRAM_DATA_WRITE_WORD) {
				value = readl(buf);
				writel(value, &PSRAM_CTRL->S_WD_DATA_REG);
				len -= 4;
				buf += 4;
			}
		}
	}

	HAL_MODIFY_REG(PSRAM_CTRL->INT_ENABLE_REG, PSRAMC_IER_TRANS_ENB_MASK,
	               PSRAMC_IER_TRANS_ENB | PSRAMC_RD_TOUT_INT_EN | PSRAMC_DMA_WR_CROSS_INT_EN);
	HAL_ExitCriticalSection(iflags);

	ctrl->mrq = mrq;
	__psram_ctrl_send_cmd(ctrl, &mrq->cmd);

	for (uint16_t tryCnt = 0; tryCnt < 0x3FFF; tryCnt++) {
		if (PSRAM_CTRL_PollHandler()) {
			;
		}
	}
	HAL_NVIC_ClearPendingIRQ(PSRAMC_IRQn);

	ret = __psram_ctrl_request_done(ctrl);

out:
	PSRAM_CTRL->PSRAM_COM_CFG &= ~PSRAMC_MR_REG_ADDR_EN;
	ctrl->mrq = NULL;
	if (ret)
		PRC_ERR("%s,%d err:%d!!\n", __func__, __LINE__, ret);

	return ret;
}

void HAL_Psram_SbusCfg(struct psram_ctrl *ctrl, uint32_t opcfg, uint32_t wait, uint32_t waitcfg)
{
	PSRAM_CTRL->S_RW_OPRT_CFG = opcfg;
	if (wait) {
		HAL_MODIFY_REG(PSRAM_CTRL->S_IO_SW_WAIT_TIME, (0xFFU << 24), waitcfg);
	}
}

void HAL_PsramCtrl_CacheCfg(struct psram_ctrl *ctrl, uint32_t cbus_wsize_bus)
{
	if (cbus_wsize_bus) {
		HAL_SET_BIT(PSRAM_CTRL->CACHE_RLVT_CFG, PSRAMC_CBUS_WR_SEL_BUS);
	} else {
		HAL_CLR_BIT(PSRAM_CTRL->CACHE_RLVT_CFG, PSRAMC_CBUS_WR_SEL_BUS);
	}
}

void HAL_PsramCtrl_IDbusCfg(struct psram_ctrl *ctrl, uint32_t write, uint32_t opcfg, uint32_t wait, uint32_t waitcfg)
{
	HAL_CLR_BIT(PSRAM_CTRL->MEM_COM_CFG, PSRAMC_CBUS_RW_EN);
	if (write) {
		PSRAM_CTRL->C_WD_OPRT_CFG = opcfg;
	} else {
		PSRAM_CTRL->C_RD_OPRT_CFG = opcfg;
	}
	if (wait) {
		HAL_MODIFY_REG(PSRAM_CTRL->C_IO_SW_WAIT_TIME, (0xFFU << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT), waitcfg);
	}
	HAL_SET_BIT(PSRAM_CTRL->MEM_COM_CFG, PSRAMC_CBUS_RW_EN);
}

typedef struct {
	uint32_t cbus_read_op;
	uint32_t cbus_write_op;
	uint32_t common_cfg;
	uint32_t p_common_cfg;
} PsramCtrl_Delay;

static void __psram_ctrl_bus_delay(struct psram_ctrl *ctrl, PsramCtrl_Delay *delay)
{
	PSRAM_CTRL->C_RD_OPRT_CFG = delay->cbus_read_op;
	PSRAM_CTRL->C_WD_OPRT_CFG = delay->cbus_write_op;
	PSRAM_CTRL->PSRAM_COM_CFG = delay->common_cfg;
	PSRAM_CTRL->MEM_COM_CFG = delay->p_common_cfg;
}

#ifdef CONFIG_PM
int psramc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	struct psram_ctrl *ctrl = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:

		HAL_PsramCtrl_Deinit(ctrl);
		break;
	default:
		break;
	}

	return 0;
}

int psramc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	struct psram_ctrl *ctrl = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_PsramCtrl_Init(ctrl);
		break;
	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_PM */

void HAL_PsramCtrl_Set_DBUS_WR_LATENCY(struct psram_ctrl *ctrl, uint32_t lat)
{
	HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_LAT_CFG, PSRAMC_DBUS_WR_LC_MASK, lat);
	PRC_DBG("%s set DBUS write latency:0x%x\n", __func__, lat);
}

void HAL_PsramCtrl_Set_SBUS_WR_LATENCY(struct psram_ctrl *ctrl, uint32_t lat)
{
	HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_LAT_CFG, PSRAMC_SBUS_WR_LC_MASK, lat);
	PRC_DBG("%s set SBUS write latency:0x%x\n", __func__, lat);
}

uint32_t HAL_PsramCtrl_Set_BusWidth(struct psram_ctrl *ctrl, uint32_t width)
{
	uint32_t back_width;

	HAL_ASSERT_PARAM(ctrl != NULL);

	back_width = ctrl->busconfig;
	ctrl->busconfig = width;

	return back_width;
}

void HAL_PsramCtrl_DMACrossEnable(uint32_t en)
{
	uint32_t rval;

	if (en)
		rval = PSRAMC_IDBUS_DMA_EN;
	else
		rval = 0;
	HAL_MODIFY_REG(PSRAM_CTRL->C_RD_OPRT_CFG, PSRAMC_IDBUS_DMA_EN, rval);
	PRC_DBG("%s set IDBUS DMA enable:0x%x\n", __func__, PSRAM_CTRL->C_RD_OPRT_CFG);
}

void HAL_PsramCtrl_Set_Address_Field(struct psram_ctrl *ctrl, uint32_t id,
                                     uint32_t startaddr, uint32_t endaddr, uint32_t bias_addr)
{
	HAL_ASSERT_PARAM(id < 8);

	PSRAM_CTRL->PSRAM_ADDR[id].END_ADDR = PSRAMC_END_POS(endaddr);
	PSRAM_CTRL->PSRAM_ADDR[id].START_ADDR = PSRAMC_START_POS(startaddr);
	PSRAM_CTRL->PSRAM_ADDR[id].BIAS_ADDR = bias_addr | PSRAMC_ADDR_BIAS_EN;
}

HAL_Status HAL_PsramCtrl_Init(struct psram_ctrl *ctrl)
{
	PsramCtrl_Delay delay;

	HAL_ASSERT_PARAM(ctrl != NULL);

	__psram_init_io(ctrl->p_type);

	/* enable ccmu */
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_PSRAM_CTRL);
	HAL_UDelay(200);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_PSRAM_CTRL);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_PSRAM_CTRL);
	HAL_PsramCtrl_SetClk(ctrl->freq);

	PSRAM_CTRL->S_RW_OPRT_CFG = 0;
	PSRAM_CTRL->S_DUMMY_DATA_H = 0;
	PSRAM_CTRL->S_DUMMY_DATA_L = 0;
	PSRAM_CTRL->S_IO_SW_WAIT_TIME = PSRAMC_SBUS_DUM_WAIT(0) | PSRAMC_SBUS_ADR_WAIT(0) | PSRAMC_SBUS_CMD_WAIT(0);

	PSRAM_CTRL->OPI_CTRL_COM_CFG = PSRAM_SPI_MODE0 | PSRAM_SPI_MSB | PSRAM_SPI_CS_L |
	                               PSRAM_SPI_RDWAIT_Cycle(ctrl->rdata_w) | PSRAM_DMA_CROSS_OP_DIS;
	PSRAM_CTRL->C_IO_SW_WAIT_TIME = PSRAMC_CBUS_DUM_WAIT(0) | PSRAMC_CBUS_ADR_WAIT(0) | PSRAMC_CBUS_CMD_WAIT(0);
	PSRAM_CTRL->FIFO_TRIGGER_LEVEL = PSRAMC_FIFO_WR_FULL_TRIG(0x38) | PSRAMC_FIFO_WR_EMPT_TRIG(0x0F) |
	                                 PSRAMC_FIFO_RD_FULL_TRIG(0x38) | PSRAMC_FIFO_RD_EMPT_TRIG(0x0F);

	switch (ctrl->p_type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		delay.cbus_read_op = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT | PSRAMC_BUS_DUMY_0BIT |
		                     PSRAMC_BUS_DUMY_WID(0) | PSRAMC_BUS_DATA_1BIT;
		delay.cbus_write_op = PSRAMC_BUS_RW_CMD(0x02) | PSRAMC_BUS_CMD_1BIT |
		                      PSRAMC_BUS_ADDR_1BIT |
		                      PSRAMC_BUS_DUMY_0BIT | PSRAMC_BUS_DUMY_WID(0) |
		                      PSRAMC_BUS_DATA_1BIT;
		delay.common_cfg = PSRAMC_MAX_READ_LATENCY(0x0CU) | PSRAMC_MAX_CEN_LOW_CYC_NUM(0x1C0U) |
		                   PSRAMC_COM_DQS_READ_WAIT(0x2U) | PSRAMC_MIN_WR_CLC_1 | PSRAMC_CLK_OUTPUT_HLD;
		delay.p_common_cfg = PSRAMC_IOX_DEF_OUTPUT_MODE(1, PSRAMC_IOX_DEF_OUTPUTZ) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(2, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(3, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAM_SELECT | PSRAMC_ADDR_SIZE_24BIT | PSRAMC_CBUS_RW_EN;
		HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_DATA_1BIT);
		HAL_PsramCtrl_Set_RD_BuffSize(PSRAMC_CACHE_LL_256BIT);
		HAL_PsramCtrl_Set_WR_BuffSize(PSRAMC_WR_CACHE_LINE_LEN_16B);
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI32)
	case PSRAM_CHIP_OPI_APS32:
#if 0
		HAL_PsramCtrl_Set_DQS_Delay_Cal(ctrl->freq);
#endif
		delay.cbus_read_op = PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DUMY_8BIT |
		                     PSRAMC_BUS_DUMY_WID(0) | PSRAMC_BUS_DATA_8BIT;
		delay.cbus_write_op = PSRAMC_BUS_RW_CMD(0x80) | PSRAMC_BUS_CMD_8BIT |
		                      PSRAMC_BUS_ADDR_8BIT |
		                      PSRAMC_BUS_DUMY_8BIT | PSRAMC_BUS_DUMY_WID(0) |
		                      PSRAMC_BUS_DATA_8BIT;
		delay.common_cfg = PSRAMC_MAX_READ_LATENCY(0x0CU) | PSRAMC_MAX_CEN_LOW_CYC_NUM(0x1C0U) |
		                   PSRAMC_MIN_WR_CLC_2 | PSRAMC_DDR_MODE_EN | PSRAMC_CLK_OUTPUT_HLD |
		                   PSRAMC_WR_NEED_DQS | PSRAMC_WR_AF_DQS_DUMMY | PSRAMC_WR_AF_DM_DUMMY |
		                   PSRAMC_CEDIS_CLK_VALID;
		if (ctrl->freq > 66000000) {
			delay.common_cfg |= PSRAMC_COM_DQS_READ_WAIT(0x2U);
		} else {
			delay.common_cfg |= PSRAMC_COM_DQS_READ_WAIT(0x1U);
		}
		delay.p_common_cfg = PSRAMC_IOX_DEF_OUTPUT_MODE(1, PSRAMC_IOX_DEF_OUTPUTZ) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(2, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(3, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(4, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(5, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(6, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(7, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAM_SELECT | PSRAMC_ADDR_SIZE_24BIT | PSRAMC_CBUS_RW_EN;
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, PSRAMC_CS_OUTP_DHCYC_MASK, PSRAMC_CS_OUTP_DHCYC(2));
		HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_DATA_8BIT);
#if 0
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, PSRAMC_CS_OUTP_DHCYC_MASK, PSRAMC_CS_OUTP_DHCYC(3));
#endif
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, PSRAMC_DQS_OUTP_DHCYC_MASK, PSRAMC_DQS_OUTP_DHCYC(1));
#ifdef CONFIG_PLATFORM_FPGA
		if (cfg->freq >= 24 * 1000 * 1000) {
			HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, PSRAMC_DQS_OUTP_DHCYC_MASK, PSRAMC_DQS_OUTP_DHCYC(3));
		}
#endif
		break;
#elif (defined CONFIG_PSRAM_CHIP_OPI64)
	case PSRAM_CHIP_OPI_APS64:
#if 0
		HAL_PsramCtrl_Set_DQS_Delay_Cal(cfg->freq);
#endif
		delay.cbus_read_op = PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_ADDR_8BIT | PSRAMC_BUS_DUMY_8BIT |
		                     PSRAMC_BUS_DUMY_WID(0) | PSRAMC_BUS_DATA_8BIT; /*0x444004*/
		delay.cbus_write_op = PSRAMC_BUS_RW_CMD(0x80) | PSRAMC_BUS_CMD_8BIT |
		                      PSRAMC_BUS_ADDR_8BIT |
		                      PSRAMC_BUS_DUMY_8BIT | PSRAMC_BUS_DUMY_WID(0) |
		                      PSRAMC_BUS_DATA_8BIT; /*0x80444004*/
		delay.common_cfg = PSRAMC_MAX_READ_LATENCY(0xCU) | PSRAMC_MAX_CEN_LOW_CYC_NUM(0xA0U) |
		                   PSRAMC_COM_DQS_READ_WAIT(0x2U) | PSRAMC_CMD_HLD_THCYC | PSRAMC_DDR_MODE_EN;
		delay.p_common_cfg = PSRAMC_IOX_DEF_OUTPUT_MODE(1, PSRAMC_IOX_DEF_OUTPUTZ) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(2, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(3, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(4, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(5, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(6, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAMC_IOX_DEF_OUTPUT_MODE(7, PSRAMC_IOX_DEF_OUTPUT1) |
		                     PSRAM_SELECT | PSRAMC_CBUS_RW_EN | PSRAMC_ADDR_SIZE_32BIT;
		HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_BUS_CMD_8BIT | PSRAMC_BUS_DATA_8BIT);
		HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, (PSRAMC_CS_OUTP_DHCYC_MASK | PSRAMC_DQS_OUTP_DHCYC_MASK),
		               (PSRAMC_CS_OUTP_DHCYC(2) | PSRAMC_DQS_OUTP_DHCYC(2)));
		break;
#endif
	default:
		PRC_ERR("%s,%d not support chip:%x\n", __func__, __LINE__, ctrl->p_type);
		goto fail;
	}
	__psram_ctrl_bus_delay(ctrl, &delay);

	PSRAM_CTRL->INT_ENABLE_REG = 0; /* clear all irq */
	PSRAM_CTRL->INT_STATUS_REG = 0xffffffff;
#if 0
	PRC_DUMP_REGS();
#endif

#ifdef CONFIG_PM
	if (!ctrl->suspending) {
		HAL_Memset(&ctrl->psramc_drv, 0, sizeof(struct soc_device_driver));
		ctrl->psramc_drv.name = "psramc";
		ctrl->psramc_drv.suspend_noirq = psramc_suspend;
		ctrl->psramc_drv.resume_noirq = psramc_resume;
		HAL_Memset(&ctrl->psramc_dev, 0, sizeof(struct soc_device));
		ctrl->psramc_dev.name = "psramc";
		ctrl->psramc_dev.driver = &ctrl->psramc_drv;
		ctrl->psramc_dev.platform_data = ctrl;
		pm_register_ops(&ctrl->psramc_dev);
		ctrl->suspending = 1;
	}
#endif
	PRC_INF("%s busconfig:0x%x\n", __func__, ctrl->busconfig);

	return HAL_OK;

fail:
	HAL_CCM_PSRAMC_DisableMClock();

	return HAL_ERROR;
}

HAL_Status HAL_PsramCtrl_Deinit(struct psram_ctrl *ctrl)
{
	__psram_deinit_io(ctrl->p_type);

#ifdef CONFIG_PM
	if (!ctrl->suspending) {
		pm_unregister_ops(&ctrl->psramc_dev);
		ctrl->psramc_dev.platform_data = NULL;
	}
#endif

	HAL_CCM_PSRAMC_DisableMClock();
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_PSRAM_CTRL);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_PSRAM_CTRL);

	return HAL_OK;
}

struct psram_ctrl *HAL_PsramCtrl_Open(uint32_t id)
{
	_psram_priv->ref++;

	return _psram_priv;
}

HAL_Status HAL_PsramCtrl_Close(struct psram_ctrl *ctrl)
{
	ctrl->ref--;

	return HAL_OK;
}

struct psram_ctrl *HAL_PsramCtrl_Create(uint32_t id, const PSRAMCtrl_InitParam *cfg)
{
	struct psram_ctrl *ctrl;

	ctrl = HAL_Malloc(sizeof(struct psram_ctrl));
	if (ctrl == NULL) {
		PRC_ERR("%s no mem!\n", __func__);
	} else {
		HAL_Memset(ctrl, 0, sizeof(struct psram_ctrl));
		ctrl->freq = cfg->freq;
		ctrl->p_type = cfg->p_type;
		ctrl->rdata_w = cfg->rdata_w;
#ifdef CONFIG_PM
		HAL_Memcpy(&ctrl->pm_sbus_cfg, cfg, sizeof(PSRAMCtrl_InitParam));
#endif
		_psram_priv = ctrl;
	}
	PRC_INF("%s @%p\n", __func__, ctrl);

	return ctrl;
}

HAL_Status HAL_PsramCtrl_Destory(struct psram_ctrl *ctrl)
{
	if (ctrl == NULL) {
		PRC_ERR("ctrl %p", ctrl);
	} else {
		_psram_priv = NULL;
		HAL_Free(ctrl);
	}
	PRC_INF("%s @%p\n", __func__, ctrl);

	return HAL_OK;
}
