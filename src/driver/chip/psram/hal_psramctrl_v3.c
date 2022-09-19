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
#include "driver/chip/hal_sysctl.h"
#include "driver/chip/psram/hal_psramctrl.h"

#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_flash.h"

#include "driver/chip/hal_gpio.h"

#define CONFIG_PSRAM_DMA_USED
#define PRC_DMA_TIMEOUT         2000

#define DBG_PRC_DBG 0
#define DBG_PRC_INF 1
#define DBG_PRC_WRN 1
#define DBG_PRC_ERR 1

#define PRC_LOG(flags, fmt, arg...) \
	do {                            \
		if (flags)                  \
			printf(fmt, ##arg);     \
	} while (0)

#if (DBG_PRC_DBG == 1)
/*#define PRC_DUMP_REGS() print_hex_dump_words((const void *)PSRAM_CTRL_BASE, 0x200)*/

#define PRC_DUMP_REGS() \
	{ \
		printf("PSRAM CTRL base addr\t[0x%08x]\n", (uint32_t)PSRAM_CTRL); \
		printf("MEM_COM_CFG:\t\t0x%08x\n", PSRAM_CTRL->MEM_COM_CFG);\
		printf("OPI_CTRL_COM_CFG:\t0x%08x\n", PSRAM_CTRL->OPI_CTRL_COM_CFG);\
		printf("CACHE_RLVT_CFG:\t\t0x%08x\n", PSRAM_CTRL->CACHE_RLVT_CFG);\
		printf("C_RD_OPRT_CFG:\t\t0x%08x\n", PSRAM_CTRL->C_RD_OPRT_CFG);\
		printf("C_WD_OPRT_CFG:\t\t0x%08x\n", PSRAM_CTRL->C_WD_OPRT_CFG);\
		printf("C_RD_DUMMY_DATA_H:\t0x%08x\n", PSRAM_CTRL->C_RD_DUMMY_DATA_H);\
		printf("C_RD_DUMMY_DATA_L:\t0x%08x\n", PSRAM_CTRL->C_RD_DUMMY_DATA_L);\
		printf("C_WD_DUMMY_DATA_H:\t0x%08x\n", PSRAM_CTRL->C_WD_DUMMY_DATA_H);\
		printf("C_WD_DUMMY_DATA_L:\t0x%08x\n", PSRAM_CTRL->C_WD_DUMMY_DATA_L);\
		printf("C_IO_SW_WAIT_TIME:\t0x%08x\n", PSRAM_CTRL->C_IO_SW_WAIT_TIME);\
		printf("PSRAM_FORCE_CFG:\t0x%08x\n", PSRAM_CTRL->PSRAM_FORCE_CFG);\
		printf("PSRAM_COM_CFG:\t\t0x%08x\n", PSRAM_CTRL->PSRAM_COM_CFG);\
	}
#else
#define PRC_DUMP_REGS()
#endif

#define PRC_DBG(fmt, arg...) PRC_LOG(DBG_PRC_DBG, "[PRC DBG] "fmt, ##arg)
#define PRC_INF(fmt, arg...) PRC_LOG(DBG_PRC_INF, "[PRC INF] "fmt, ##arg)
#define PRC_WRN(fmt, arg...) PRC_LOG(DBG_PRC_WRN, "[PRC WRN] "fmt, ##arg)
#define PRC_ERR(fmt, arg...) PRC_LOG(DBG_PRC_ERR, "[PRC ERR] "fmt, ##arg)

#define PRC_WARN_ON(v) \
	do { if (v) { printf("WARN at %s:%d!\n", __func__, __LINE__); } } while (0)
#define PRC_BUG_ON(v) \
	do { if (v) { printf("BUG at %s:%d!\n", __func__, __LINE__); while (1); } } while (0)

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

#define PSRAMC_WHILE_TIMEOUT(cond)    \
	do { \
		uint32_t __cnt = 0x3FFFFFF;    \
		do { if (--__cnt == 0) { return HAL_ERROR; } } while (cond); \
	} while (0)

/*
 * Debug State:
 *   0x0 Idle or Sbus complete;
 *   0x2 Send CMD;
 *   0x4 Send Address;
 *   0x6 Send Dummy;
 *   0x8 Send Data;
 *   0x9 Get Data;
 *   0xc Ibus read complete;
 */
static int PSRAM_Sbus_GetFIFOCnt(uint32_t write)
{
	if (write) {
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG,
		                       FLASHC_WR_FIFO_COUNTER_SHIFT,
		                       FLASHC_WR_FIFO_COUNTER_VMASK);
	} else {
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG,
		                       FLASHC_RD_FIFO_COUNTER_SHIFT,
		                       FLASHC_RD_FIFO_COUNTER_VMASK);
	}
}

static inline int PSRAM_WaitState(int state)
{
	while (HAL_GET_BIT_VAL(OPI_MEM_CTRL->MEM_CTRL_DEBUG_STATE,
	                       FLASHC_MEM_CTRL_STATUE_DEBUG_SHIFT,
	                       FLASHC_MEM_CTRL_STATUE_DEBUG_VMASK) != state)
		;
	return 0;
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

	if (clk > HAL_GetHFClock()) {
		mclk = HAL_GetDevClock();
		src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK;
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
	PRC_INF("PSRAM CTRL MCLK:%u MHz clock=%u MHz,src:%x, n:%d, m:%d\n",
	        mclk / 1000000, mclk / (1 << div_n) / (div_m + 1) / 1000000,
	        (int)src, (int)div_n, (int)div_m);

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

void HAL_PsramCtrl_CacheCfg(struct psram_ctrl *ctrl, uint32_t cbus_wsize_bus)
{
	if (cbus_wsize_bus) {
		HAL_SET_BIT(OPI_MEM_CTRL->CACHE_RLVT_CONFG, FLASHC_CBUS_WR_SIZE_SELECT_BIT);
	} else {
		HAL_CLR_BIT(OPI_MEM_CTRL->CACHE_RLVT_CONFG, FLASHC_CBUS_WR_SIZE_SELECT_BIT);
	}
}

void HAL_PsramCtrl_IDbusCfg(struct psram_ctrl *ctrl, uint32_t write,
                            uint32_t opcfg, uint32_t wait, uint32_t waitcfg)
{
	HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_CBUS_RW_ENABLE_BIT);
	if (write) {
		PSRAM_CTRL->C_WD_OPRT_CFG = opcfg;
	} else {
		PSRAM_CTRL->C_RD_OPRT_CFG = opcfg;
	}
	if (wait) {
		HAL_MODIFY_REG(PSRAM_CTRL->C_IO_SW_WAIT_TIME,
		               PSRAMC_CBUS_RW_LAT_WAIT_MASK, waitcfg);
	}
	HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_CBUS_RW_ENABLE_BIT);
}

int32_t HAL_PsramCtrl_Request(struct psram_ctrl *ctrl, struct psram_request *mrq)
{
	int32_t ret = HAL_OK;
	int32_t len;
	uint8_t *buf = NULL;
	unsigned long iflags;

	len = mrq->data.blksz * mrq->data.blocks;
	PRC_DBG("%s,%d cmd:0x%x addr:0x%x len:%d flag:0x%x bc:0x%x io_wait:%x\n",
	        __func__, __LINE__, mrq->cmd.opcode, mrq->cmd.addr, len,
	        mrq->data.flags, ctrl->busconfig, OPI_MEM_CTRL->SBUS_IO_SW_WAIT_TIME);

	iflags = HAL_EnterCriticalSection();
	OPI_MEM_CTRL->SBUS_RW_OPRT_CONFG = mrq->cmd.opcode << PSRAMC_BUS_RW_CMD_SHIFT |
	                                   FLASHC_SBUS_DEV_SEL_PSRAM | mrq->cmd.busconfig | mrq->data.busconfig;
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_IO_SW_WAIT_TIME,
	               (0xFFU << FLASHC_SBUS_LATENCY_WAIT_CYCLE_SHIFT),
	               mrq->cmd.dummy << FLASHC_SBUS_LATENCY_WAIT_CYCLE_SHIFT);

	if (mrq->data.flags & PSRAM_DATA_READ_MASK) {
		OPI_MEM_CTRL->SBUS_ADDR_CONFG = mrq->cmd.addr;
		OPI_MEM_CTRL->SBUS_RD_DATA_BYTE_NUM = len;
		buf = mrq->cmd.resp;
	} else if (mrq->data.flags & PSRAM_DATA_WRITE_MASK) {
		OPI_MEM_CTRL->SBUS_ADDR_CONFG = mrq->cmd.addr;
		OPI_MEM_CTRL->SBUS_WR_DATA_BYTE_NUM = len;
		buf = mrq->data.buff;
	}
	if (mrq->data.flags)
		PSRAM_CTRL->PSRAM_COM_CFG |= PSRAMC_MR_REG_ADDR_EN;

	ctrl->mrq = mrq;
	HAL_SET_BIT(OPI_MEM_CTRL->SBUS_START_SEND_REG, FLASHC_ENABLE_SBUS_MASK);
	if (mrq->data.flags & PSRAM_DATA_WRITE_MASK) {
		uint32_t value;

		while (len > 0) {
			PSRAMC_WHILE_TIMEOUT(PSRAM_Sbus_GetFIFOCnt(1) > 100);

			if (mrq->data.flags & PSRAM_DATA_WRITE_BYTE) {
				value = readb(buf);
				writeb(value, &OPI_MEM_CTRL->SBUS_WR_DATA_REG);
				len--;
				buf++;
			} else if (mrq->data.flags & PSRAM_DATA_WRITE_SHORT) {
				value = readw(buf);
				writew(value, &OPI_MEM_CTRL->SBUS_WR_DATA_REG);
				len -= 2;
				buf += 2;
			} else if (mrq->data.flags & PSRAM_DATA_WRITE_WORD) {
				value = readl(buf);
				writel(value, &OPI_MEM_CTRL->SBUS_WR_DATA_REG);
				len -= 4;
				buf += 4;
			} else {
				PRC_ERR("%s not support type:%x\n", __func__, mrq->data.flags);
				ret = HAL_ERROR;
				break;
			}
		}
	} else if (mrq->data.flags & PSRAM_DATA_READ_MASK) {
		while (len--) {
			PSRAMC_WHILE_TIMEOUT(PSRAM_Sbus_GetFIFOCnt(0) == 0);
			*(buf++) = readb(&OPI_MEM_CTRL->SBUS_RD_DATA_REG);
		}
	}
	PSRAMC_WHILE_TIMEOUT((!!HAL_GET_BIT(OPI_MEM_CTRL->SBUS_START_SEND_REG,
	                                    FLASHC_ENABLE_SBUS_MASK)));
	if (PSRAM_WaitState(0))
		ret = HAL_ERROR;
	HAL_CLR_BIT(OPI_MEM_CTRL->INT_STATUS_REG,
	            FLASHC_TRAN_COMPLETED_INT_FLAG_MASK |  FLASHC_START_SEND_HW_CLEAR_INT_FLAG_MASK);
	HAL_ExitCriticalSection(iflags);

	PSRAM_CTRL->PSRAM_COM_CFG &= ~PSRAMC_MR_REG_ADDR_EN;
	ctrl->mrq = NULL;
	if (ret)
		PRC_ERR("%s,%d err:%d!!\n", __func__, __LINE__, ret);

	return ret;
}

typedef struct {
	uint32_t cbus_read_op;
	uint32_t cbus_write_op;
	uint32_t p_common_cfg;
} PsramCtrl_Delay;

static void __psram_ctrl_bus_delay(struct psram_ctrl *ctrl, PsramCtrl_Delay *delay)
{
	PSRAM_CTRL->C_RD_OPRT_CFG = delay->cbus_read_op;
	PSRAM_CTRL->C_WD_OPRT_CFG = delay->cbus_write_op;
	PSRAM_CTRL->MEM_COM_CFG = delay->p_common_cfg;
}

void _psramc_flashc_caback(struct flash_controller *ctrl, FC_OPERATE op, uint32_t step)
{
	PRC_BUG_ON(!_psram_priv);
	uint32_t *back_regs = _psram_priv->back_regs;

	switch (op) {
	case FC_OPEN:
		if (step != 0)
			break;
		back_regs[0] = PSRAM_CTRL->MEM_COM_CFG;
		back_regs[1] = PSRAM_CTRL->OPI_CTRL_COM_CFG;
		back_regs[2] = PSRAM_CTRL->CACHE_RLVT_CFG;
		back_regs[3] = PSRAM_CTRL->C_RD_OPRT_CFG;
		back_regs[4] = PSRAM_CTRL->C_WD_OPRT_CFG;
		back_regs[5] = PSRAM_CTRL->C_IO_SW_WAIT_TIME;
		back_regs[6] = PSRAM_CTRL->PSRAM_FORCE_CFG;
		back_regs[7] = PSRAM_CTRL->PSRAM_COM_CFG;
		break;
	case FC_CLOSE:
		if (step != 1)
			break;
		PSRAM_CTRL->MEM_COM_CFG         = back_regs[0];
		PSRAM_CTRL->OPI_CTRL_COM_CFG    = back_regs[1];
		PSRAM_CTRL->CACHE_RLVT_CFG      = back_regs[2];
		PSRAM_CTRL->C_RD_OPRT_CFG       = back_regs[3];
		PSRAM_CTRL->C_WD_OPRT_CFG       = back_regs[4];
		PSRAM_CTRL->C_IO_SW_WAIT_TIME   = back_regs[5];
		PSRAM_CTRL->PSRAM_FORCE_CFG     = back_regs[6];
		PSRAM_CTRL->PSRAM_COM_CFG       = back_regs[7];
		if (back_regs[7] != PSRAM_CTRL->PSRAM_COM_CFG)
			PRC_ERR("%s,%d err:%d!!\n", __func__, __LINE__, PSRAM_CTRL->PSRAM_COM_CFG);
		break;
	default:
		break;
	}
}

#ifdef CONFIG_PM
static int psramc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	struct psram_ctrl *ctrl = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		ctrl->suspending = 1;
		__psram_deinit_io(ctrl->p_type);
		_psramc_flashc_caback(NULL, FC_OPEN, 0);
		break;
	default:
		break;
	}

	return 0;
}

static int psramc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	struct psram_ctrl *ctrl = dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		_psramc_flashc_caback(NULL, FC_CLOSE, 1);
		__psram_init_io(ctrl->p_type);
		HAL_PsramCtrl_SetClk(ctrl->freq);
		HAL_PsramCtrl_DMACrossEnable(1);
		ctrl->suspending = 0;
		break;
	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_PM */

uint32_t HAL_PsramCtrl_Set_BusWidth(struct psram_ctrl *ctrl, uint32_t width)
{
	uint32_t back_width;

	HAL_ASSERT_PARAM(ctrl != NULL);

	back_width = ctrl->busconfig;
	ctrl->busconfig = width;

	return back_width;
}

#if (CONFIG_CHIP_ARCH_VER == 3)
void HAL_PsramCtrl_ClrCacheFIFO(void)
{
	HAL_SET_BIT(PSRAM_CTRL->OPI_CTRL_COM_CFG,
	            (PSRAM_CLEAR_CACHELINE_FIFO_MASK |  PSRAM_CLEAR_CACHELINE_TIMEOUT_MASK));
	while (HAL_GET_BIT(PSRAM_CTRL->OPI_CTRL_COM_CFG,
	                   (PSRAM_CLEAR_CACHELINE_FIFO_MASK |  PSRAM_CLEAR_CACHELINE_TIMEOUT_MASK)))
		;
}
#endif

void HAL_PsramCtrl_Set_Address_Field(struct psram_ctrl *ctrl, uint32_t id,
                                     uint32_t startaddr, uint32_t endaddr,
                                     uint32_t bias_addr)
{
	OPI_MEM_CTRL->FLASH_ADDR[id].END_ADDR = PSRAMC_END_POS(endaddr);
	OPI_MEM_CTRL->FLASH_ADDR[id].START_ADDR = PSRAMC_START_POS(startaddr);
	OPI_MEM_CTRL->FLASH_ADDR[id].BIAS_ADDR = bias_addr | PSRAMC_ADDR_BIAS_EN;
	PRC_INF("PsramCtrl set addr start:%x end:%x bias:%x\n",
	        OPI_MEM_CTRL->FLASH_ADDR[id].START_ADDR, OPI_MEM_CTRL->FLASH_ADDR[id].END_ADDR,
	        OPI_MEM_CTRL->FLASH_ADDR[id].BIAS_ADDR);
}

void HAL_PsramCtrl_DMACrossEnable(uint32_t en)
{
	if (en)
		HAL_CLR_BIT(PSRAM_CTRL->OPI_CTRL_COM_CFG, PSRAM_DMA_CROSS_OP_DIS);
	else
		HAL_SET_BIT(PSRAM_CTRL->OPI_CTRL_COM_CFG, PSRAM_DMA_CROSS_OP_DIS);
}

int32_t HAL_PsramCtrl_DQS_Delay_Calibration(struct psram_ctrl *ctrl)
{
	PRC_ERR("%s,%d not support now\n", __func__, __LINE__);

	return 0;
}

HAL_Status HAL_PsramCtrl_Init(struct psram_ctrl *ctrl)
{
	PsramCtrl_Delay delay;

	HAL_ASSERT_PARAM(ctrl != NULL);

	__psram_init_io(ctrl->p_type);

	/* config clk */
	HAL_PsramCtrl_SetClk(ctrl->freq);

	HAL_PsramCtrl_DMACrossEnable(1);

	switch (ctrl->p_type) {
#if (defined CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		delay.cbus_read_op = PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT |
		                     PSRAMC_BUS_DUMY_0BIT |
		                     PSRAMC_BUS_DUMY_WID(0) | PSRAMC_BUS_DATA_1BIT;
		delay.cbus_write_op = PSRAMC_BUS_RW_CMD(0x02) | PSRAMC_BUS_CMD_1BIT |
		                      PSRAMC_BUS_ADDR_1BIT |
		                      PSRAMC_BUS_DUMY_0BIT | PSRAMC_BUS_DUMY_WID(0) |
		                      PSRAMC_BUS_DATA_1BIT;
		delay.p_common_cfg = PSRAMC_ADDR_SIZE_24BIT;
		HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_BUS_CMD_1BIT | PSRAMC_BUS_ADDR_1BIT | PSRAMC_BUS_DATA_1BIT);
		break;
#else
#error "not support"
#endif
	default :
		PRC_ERR("%s,%d not support chip:%x\n", __func__, __LINE__, ctrl->p_type);
		goto fail;
	}
	__psram_ctrl_bus_delay(ctrl, &delay);
#ifdef CONFIG_PLATFORM_FPGA
	PSRAM_CTRL->PSRAM_FORCE_CFG = PSRAMC_WAIT_HALF_CYC_NUM(1);
#else
	PSRAM_CTRL->PSRAM_FORCE_CFG = PSRAMC_WAIT_HALF_CYC_NUM(2);
#endif
	HAL_Flashc_SetCacheLineLen(FC_RD_CACHE_LINE_LEN_32B, FC_WT_CACHE_LINE_LEN_64B);

#if 0
	PRC_DUMP_REGS();
#endif
	HAL_Flashc_RegisterCb(_psramc_flashc_caback);

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
	unsigned long iflags;

	iflags = HAL_EnterCriticalSection();
	FC_SbusPrepare();
	FC_SbusFinish();
	HAL_ExitCriticalSection(iflags);

	__psram_deinit_io(ctrl->p_type);

#ifdef CONFIG_PM
	if (!ctrl->suspending) {
		pm_unregister_ops(&ctrl->psramc_dev);
		ctrl->psramc_dev.platform_data = NULL;
	}
#endif
	HAL_Flashc_RegisterCb(NULL);

	HAL_PsramCtrl_DMACrossEnable(0);

#if 0
	/* not close psram clk for flash use this for a while bofore read flash */
	HAL_CCM_PSRAMC_DisableMClock();
#endif

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
