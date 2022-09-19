/**
 * @file  hal_scr.c
 * @author  XRADIO IOT WLAN Team
 */

/*
 * Copyright (C) 2019 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
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

#include "driver/chip/hal_scr.h"
#include "pm/pm.h"
#include "hal_base.h"

typedef struct {
	SCR_IRQCallback     IRQCallback;
	void                *arg;
#ifdef CONFIG_PM
	uint8_t             bypassPmMode;
	uint8_t             txDelay;
	SCR_InitParam       param;
	struct soc_device   dev;
#endif
} SCR_Private;

typedef struct {
	volatile uint32_t wptr;
	volatile uint32_t rptr;
	volatile uint8_t buffer[SCR_BUFFER_SIZE_MASK + 1];
} SCR_RxPriv;

typedef struct {
	uint8_t  *buf;
	int32_t size;
	HAL_Semaphore sem;
} SCR_RXTxPriv;

typedef struct {
	SCR_RxPriv rxbuf;
	SCR_RXTxPriv rxpriv;
	SCR_RXTxPriv txpriv;
	volatile uint32_t detected;
	volatile uint32_t activated;
	volatile uint32_t atr_resp;
	uint32_t chto_flag;
} SCR_WorkStruct;

static SCR_Private *gSCRPrivate;
static SCR_WorkStruct *g_scr_work;

/* static scr_fsm_record scr_fsm; */
static scatr_struct scatr;
static upps_struct pps;

static volatile SCR_TEST_STAGE stage = STS_IDLE;
static int state_dly;
static volatile uint32_t card_id;

#define SMART_CARD_NUM 9
static const uint8_t ic_card_atr[SMART_CARD_NUM][30] = {
	{7,  0x3b, 0x93, 0x11, 0x00, 0x00, 0x40, 0x41}, /* laixi youxian */
	{18, 0x3b, 0x7d, 0x94, 0x00, 0x00, 0x57, 0x44, 0x53, 0x67, 0x96, 0x86, 0x93, 0x03, 0x9d, 0xf7, 0x10, 0x00, 0x9d}, /* shenzhou dazhong1 */
	{15, 0x3b, 0xb9, 0x94, 0x00, 0x40, 0x14, 0x47, 0x47, 0x33, 0x53, 0x30, 0x35, 0x41, 0x53, 0x30}, /* shenzhou dazhong2 */
	{17, 0x3b, 0x9d, 0x95, 0x00, 0x13, 0x61, 0x40, 0x36, 0x13, 0x85, 0xe9, 0x44, 0x34, 0x8f, 0x78, 0x8f, 0x4a}, /* digital TV1 */
	{17, 0x3b, 0x9d, 0x95, 0x00, 0x13, 0x61, 0x40, 0x36, 0x13, 0x85, 0xe9, 0x44, 0x34, 0xf3, 0x78, 0x8f, 0x4a}, /* digital TV2 */
	{22, 0x3b, 0x9f, 0x95, 0x80, 0x1f, 0xc3, 0x80, 0x31, 0xe0, 0x73, 0xfe, 0x21, 0x13, 0x57, 0x86, 0x81, 0x02, 0x86, 0x98, 0x44, 0x18, 0xa8}, /* dianxin 4G */
	{20, 0x3b, 0xfb, 0x94, 0x00, 0x00, 0x80, 0x1f, 0x83, 0x80, 0x65, 0x92, 0x10, 0x26, 0x86, 0x53, 0x83, 0x00, 0x90, 0x00, 0xf4}, /* liantong */
	{16, 0x3b, 0x7b, 0x94, 0x00, 0x00, 0x97, 0x88, 0x84, 0x86, 0x60, 0xa0, 0x04, 0x01, 0x00, 0x04, 0x00}, /* yiodng */
	{20, 0x3b, 0x7f, 0x12, 0x00, 0x00, 0x44, 0x56, 0x4e, 0x20, 0x54, 0x45, 0x53, 0x54, 0x20, 0x43, 0x41, 0x52, 0x44, 0x76, 0x31} /* digital TV */
};

static uint8_t ic_card_send_cmd[SMART_CARD_NUM][30] = {
	{5, 0xe5, 0x04, 0x00, 0x00, 0x04},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{7, 0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00},
	{0}
};

static const uint8_t ic_card_rev_data[SMART_CARD_NUM][30] = {
	{7, 0x04, 0x4d, 0x33, 0x4f, 0x4b, 0x90, 0x00},
	{0},
	{0},
	{0},
	{0},
	{0},
	{0},
	{3, 0xa4, 0x9f, 0x1b},
	{0}
};

__STATIC_INLINE SCR_Private *SCR_GetPriv(void)
{
	return gSCRPrivate;
}

__STATIC_INLINE void SCR_SetPriv(SCR_Private *priv)
{
	gSCRPrivate = priv;
}

__STATIC_INLINE SCR_WorkStruct *SCR_GetWorkParams(void)
{
	return g_scr_work;
}

__STATIC_INLINE void SCR_SetWorkParams(SCR_WorkStruct *priv)
{
	g_scr_work = priv;
}

__STATIC_INLINE void SCR_EnableTSRx(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_TS_RX_EN);
}

__STATIC_INLINE uint32_t SCR_ISDetected(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_CSR, SCR_DETECT_BIT);
}

__STATIC_INLINE void SCR_StartDeact(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_DEACT_BIT);
}

__STATIC_INLINE void SCR_StartAct(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_ACT_BIT);
}

__STATIC_INLINE void SCR_EnableWarmRst(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_WARM_RESET_BIT);
}

__STATIC_INLINE void SCR_StopClk(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_CLK_STOP_BIT);
}

__STATIC_INLINE void SCR_RestartClk(void)
{
	HAL_CLR_BIT(SCREDAER->SCR_CSR, SCR_CLK_STOP_BIT);
}

__STATIC_INLINE void SCR_EnableGlobalINT(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_GINT_EN);
}

__STATIC_INLINE void SCR_DisableGlobalINT(void)
{
	HAL_CLR_BIT(SCREDAER->SCR_CSR, SCR_GINT_EN);
}

__STATIC_INLINE void SCR_EnableRX(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_RX_EN);
}

__STATIC_INLINE void SCR_DisableRX(void)
{
	HAL_CLR_BIT(SCREDAER->SCR_CSR, SCR_RX_EN);
}

__STATIC_INLINE void SCR_EnableTX(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_TX_EN);
}

__STATIC_INLINE void SCR_DisableTX(void)
{
	HAL_CLR_BIT(SCREDAER->SCR_CSR, SCR_TX_EN);
}

__STATIC_INLINE void SCR_EnableATRSTFlush(void)
{
	HAL_SET_BIT(SCREDAER->SCR_CSR, SCR_ATR_ST_FLUSH_FIFO_BIT);
}

__STATIC_INLINE void SCR_EnableIRQ(uint32_t irq)
{
	HAL_SET_BIT(SCREDAER->SCR_INT_EN, irq);
}

__STATIC_INLINE void SCR_DisableIRQ(uint32_t irq)
{
	HAL_CLR_BIT(SCREDAER->SCR_INT_EN, irq);
}

__STATIC_INLINE uint32_t SCR_GetIRQEnableStatus(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_INT_EN, SCR_INT_ALL_MASK);
}

__STATIC_INLINE uint32_t SCR_GetIRQStatus(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_INT_ST, SCR_INT_ALL_MASK);
}

__STATIC_INLINE void SCR_ClearIRQStatus(uint32_t irq)
{
	HAL_SET_BIT(SCREDAER->SCR_INT_ST, irq);
}

/* fifo_type: SCR_FLUSH_RXFIFO | SCR_FLUSH_TXFIFO */
__STATIC_INLINE void SCR_FlushFIFO(uint32_t fifo_type)
{
	HAL_SET_BIT(SCREDAER->SCR_FIFO_CSR, fifo_type);
}

__STATIC_INLINE uint32_t SCR_TXFIFO_ISEmpty(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_FIFO_CSR, SCR_TXFIFO_EMPTY);
}

__STATIC_INLINE uint32_t SCR_TXFIFO_ISFull(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_FIFO_CSR, SCR_TXFIFO_FULL);
}

__STATIC_INLINE uint32_t SCR_RXFIFO_ISEmpty(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_FIFO_CSR, SCR_RXFIFO_EMPTY);
}

__STATIC_INLINE uint32_t SCR_RXFIFO_ISFull(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_FIFO_CSR, SCR_RXFIFO_FULL);
}

__STATIC_INLINE void SCR_SetFIFOThreshold(uint32_t txfifo_thr, uint32_t rxfifo_thr)
{
	HAL_MODIFY_REG(SCREDAER->SCR_FIFO_CNT, \
	               (SCR_FIFO_CNT_MASK << SCR_TXFIFO_THR_SHIFT) | (SCR_FIFO_CNT_MASK << SCR_RXFIFO_THR_SHIFT), \
	               (txfifo_thr << SCR_TXFIFO_THR_SHIFT) | (txfifo_thr << SCR_RXFIFO_THR_SHIFT));
}

__STATIC_INLINE uint32_t SCR_GetTXFIFOCount(void)
{
	return HAL_GET_BIT_VAL(SCREDAER->SCR_FIFO_CNT, SCR_TXFIFO_CNT_SHIFT, SCR_FIFO_CNT_MASK);
}

__STATIC_INLINE uint32_t SCR_GetRXFIFOCount(void)
{
	return HAL_GET_BIT_VAL(SCREDAER->SCR_FIFO_CNT, SCR_RXFIFO_CNT_SHIFT, SCR_FIFO_CNT_MASK);
}

__STATIC_INLINE void SCR_SetRepeat(uint32_t tx_repeat, uint32_t rx_repeat)
{
	HAL_MODIFY_REG(SCREDAER->SCR_FIFO_REPEAT, \
	               (SCR_REPEAT_MASK << SCR_TX_REPEAT_NUM_SHIFT) | (SCR_REPEAT_MASK << SCR_RX_REPEAT_NUM_SHIFT), \
	               (tx_repeat << SCR_TX_REPEAT_NUM_SHIFT) | (rx_repeat << SCR_RX_REPEAT_NUM_SHIFT));
}

__STATIC_INLINE void SCR_SetClkDivisor(uint32_t baud_div, uint32_t clk_div)
{
	SCREDAER->SCR_CLKDIV = (baud_div << SCR_BAUD_DIV_SHIFT) | (clk_div << SCR_MCLK_DIV_SHIFT);
}

__STATIC_INLINE uint32_t SCR_GetClkDivisor(void)
{
	return HAL_GET_BIT_VAL(SCREDAER->SCR_CLKDIV, SCR_MCLK_DIV_SHIFT, SCR_CLKDIV_MASK);
}

__STATIC_INLINE uint32_t SCR_GetBaudDivisor(void)
{
	return HAL_GET_BIT_VAL(SCREDAER->SCR_CLKDIV, SCR_BAUD_DIV_SHIFT, SCR_CLKDIV_MASK);
}

__STATIC_INLINE void SCR_SetActTime(uint32_t value)
{
	HAL_MODIFY_REG(SCREDAER->SCR_LTIM, (SCR_LTIM_8BTI_MASK << SCR_LTIM_ACT_SHIFT), \
	               (value << SCR_LTIM_ACT_SHIFT));
}

__STATIC_INLINE void SCR_SetRstTime(uint32_t value)
{
	HAL_MODIFY_REG(SCREDAER->SCR_LTIM, (SCR_LTIM_8BTI_MASK << SCR_LTIM_RST_SHIFT), \
	               (value << SCR_LTIM_RST_SHIFT));
}

__STATIC_INLINE void SCR_SetAtrTime(uint32_t value)
{
	HAL_MODIFY_REG(SCREDAER->SCR_LTIM, (SCR_LTIM_8BTI_MASK << SCR_LTIM_ATR_SHIFT), \
	               (value << SCR_LTIM_ATR_SHIFT));
}

__STATIC_INLINE void SCR_SetCharLimit(uint32_t value)
{
	HAL_MODIFY_REG(SCREDAER->SCR_CTIM, (SCR_CTIM_CHARLIMT_MASK << SCR_CTIM_CHARLIMT_SHIFT), \
	               (value << SCR_CTIM_CHARLIMT_SHIFT));
}

__STATIC_INLINE void SCR_SetGuardTime(uint32_t value)
{
	HAL_MODIFY_REG(SCREDAER->SCR_CTIM, (SCR_CTIM_GUARDTIME_MASK << SCR_CTIM_GUARDTIME_SHIFT), \
	               (value << SCR_CTIM_GUARDTIME_SHIFT));
}

__STATIC_INLINE void SCR_EnableAutoVpp(void)
{
	HAL_SET_BIT(SCREDAER->SCR_PAD, SCR_PAD_AUTO_ACT_VPP);
}

__STATIC_INLINE void SCR_DisableAutoVpp(void)
{
	HAL_CLR_BIT(SCREDAER->SCR_PAD, SCR_PAD_AUTO_ACT_VPP);
}

__STATIC_INLINE void SCR_SetDebounce(uint32_t debounce_time)
{
	SCREDAER->SCR_DT = debounce_time;
}

__STATIC_INLINE void SCR_WriteFIFO(uint8_t data)
{
	SCREDAER->SCR_FIFO_DATA = data;
}

__STATIC_INLINE uint8_t SCR_ReadFIFO(void)
{
	return HAL_GET_BIT_VAL(SCREDAER->SCR_FIFO_DATA, 0, 0xFF);
}

__STATIC_INLINE uint32_t SCR_GetFSM(void)
{
	return HAL_GET_BIT(SCREDAER->SCR_FSM, (uint32_t)~0);
}

#ifdef CONFIG_PM
static int scr_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_SCR_DeInit();
		break;
	default:
		break;
	}

	return 0;
}

static int scr_resume(struct soc_device *dev, enum suspend_state_t state)
{
	SCR_Private *priv = SCR_GetPriv();

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		HAL_SCR_Init(&priv->param);
		break;
	default:
		break;
	}

	return 0;
}

static const char scr_name[] = "scr";

static struct soc_device_driver scr_drv = {
	.name = scr_name,
	.suspend_noirq = scr_suspend,
	.resume_noirq = scr_resume,
};
#endif

static uint32_t SCR_RxBuf_ISEmpty(SCR_RxPriv *pbuf)
{
	return (pbuf->wptr == pbuf->rptr);
}

#if 0
static uint32_t SCR_RxBuf_ISFull(SCR_RxPriv *pbuf)
{
	return ((pbuf->wptr ^ pbuf->rptr) == (SCR_BUFFER_SIZE_MASK + 1));
}
#endif

static void SCR_RxBuf_Flush(SCR_RxPriv *pbuf)
{
	pbuf->wptr = pbuf->rptr = 0;
}

/**
 * @brief SCR send data to the assigned buffer 'rxbuf'
 * @retval void
 */
static void HAL_SCR_FillRxbuf(SCR_RxPriv *pbuf, uint8_t data)
{
	pbuf->buffer[pbuf->wptr] = data;
	pbuf->wptr++;
	if (pbuf->wptr == (SCR_BUFFER_SIZE_MASK + 1)) {
		pbuf->wptr = 0;
	}
}

int32_t HAL_SCR_SendByPoll(uint8_t *buf, int32_t size)
{
	int32_t i, repeat = 20;

	if ((buf == NULL) || (size <= 0)) {
		return -1;
	}

	SCR_FlushFIFO(SCR_FLUSH_TXFIFO);
	for (i = 0; i < size; i++) {
		while (SCR_RXFIFO_ISFull() && repeat--) {
			SCR_MDelay(20);
		}
		if (repeat < 0) {
			HAL_ERR("TX FIFO full, write timeout\n");
			break;
		}

		SCR_WriteFIFO(buf[i]);
		repeat = 20;
	}

	return i;
}

int32_t HAL_SCR_SendByIT(uint8_t *buf, int32_t size)
{
	SCR_WorkStruct *pscr = SCR_GetWorkParams();

	if ((buf == NULL) || (size <= 0)) {
		return -1;
	}

	pscr->txpriv.buf = (uint8_t *)buf;
	pscr->txpriv.size = size;

	SCR_FlushFIFO(SCR_FLUSH_TXFIFO);
	SCR_EnableIRQ(SCR_INT_TXFIFO_EMPTY);
	HAL_SemaphoreWait(&pscr->txpriv.sem, HAL_WAIT_FOREVER);
	SCR_DisableIRQ(SCR_INT_TXFIFO_EMPTY);

	pscr->txpriv.buf = NULL;
	size -= pscr->txpriv.size;
	pscr->txpriv.size = 0;

	return size;
}

int32_t HAL_SCR_ReceiveByIT(uint8_t *buf, int32_t size)
{
	SCR_WorkStruct *pscr = SCR_GetWorkParams();

	if ((buf == NULL) || (size <= 0)) {
		return -1;
	}

	pscr->rxpriv.buf = (uint8_t *)buf;
	pscr->rxpriv.size = size;

	SCR_EnableIRQ(SCR_INT_RX_DONE);
	HAL_SemaphoreWait(&pscr->rxpriv.sem, 2000);
	SCR_DisableIRQ(SCR_INT_RX_DONE);
	SCR_FlushFIFO(SCR_FLUSH_RXFIFO);

	pscr->rxpriv.buf = NULL;
	size -= pscr->rxpriv.size;
	pscr->rxpriv.size = 0;

	return size;
}

/**
 * @brief designed a buffer locate in heap, with 'wptr' report the current
 *        last rx_data location, and 'rptr' report the last location that
 *        have been read by cpu.
 * @retval void
 */
static void SCR_ReadRxBufAll(SCR_WorkStruct *pscr, uint8_t *buffer)
{
	uint32_t i = 0;

	while (pscr->rxbuf.wptr != pscr->rxbuf.rptr) {
		if (pscr->rxbuf.wptr < pscr->rxbuf.rptr) {
			for (i = 0; i < (SCR_BUFFER_SIZE_MASK + 1 - pscr->rxbuf.rptr); i++) {
				*(buffer++) = *((uint8_t *)(pscr->rxbuf.buffer + pscr->rxbuf.rptr + i));
			}
			for (i = 0; i < pscr->rxbuf.wptr; i++) {
				*(buffer++) = *((uint8_t *)(pscr->rxbuf.buffer + i));
			}
			pscr->rxbuf.rptr = pscr->rxbuf.wptr;
		} else {
			for (i = 0; i < (pscr->rxbuf.wptr - pscr->rxbuf.rptr); i++) {
				*(buffer++) = *((uint8_t *)(pscr->rxbuf.buffer + pscr->rxbuf.rptr + i));
			}
			pscr->rxbuf.rptr = pscr->rxbuf.wptr;
		}
	}
}

static uint8_t SCR_ReadRxBufByte(SCR_RxPriv *pbuf)
{
	uint8_t data;

	data = pbuf->buffer[pbuf->rptr & SCR_BUFFER_SIZE_MASK];
	pbuf->rptr++;
	pbuf->rptr &= (SCR_BUFFER_SIZE_MASK << 1) | 0x1;

	return data;
}

#if 0
static void SCR_DumpRxBufAll(SCR_RxPriv *pbuf)
{
	uint32_t i = 0;

	if (!SCR_RxBuf_ISEmpty(pbuf)) {
		for (i = pbuf->rptr; (i & ((SCR_BUFFER_SIZE_MASK << 1) | 0x1)) != pbuf->wptr; i++) {
			HAL_DBG("0x%x ", pbuf->buffer[i & SCR_BUFFER_SIZE_MASK]);

			pbuf->rptr++;
			if (pbuf->rptr >= SCR_BUFFER_SIZE_MASK) {
				pbuf->rptr = 0;
			}
		}
		HAL_DBG("\n");
	} else {
		HAL_DBG("Buffer is Empty when Display!!\n");
	}
}
#endif

void SCR_ParamsInit(scatr_struct *pscatr)
{
	pscatr->TS = 0x3B;
	pscatr->TK_NUM = 0x00;

	pscatr->T = 0;      /* T=0 Protocol */
	pscatr->FMAX = 4;   /* 4MHz */
	pscatr->F = 372;
	pscatr->D = 1;
	pscatr->I = 50;     /* 50mA */
	pscatr->P = 5;      /* 5V */
	pscatr->N = 2;
}

static void SCR_TA1_Decode(scatr_struct *pscatr, uint8_t ta1)
{
	uint16_t Fifmax_list[14][2] = {{372, 4}, {372, 5}, {558, 6}, {774, 8}, {1116, 12}, {1488, 16}, {1860, 20},
		{0, 0}, {0, 0}, {512, 5}, {768, 7}, {1024, 10}, {1536, 15}, {2048, 20}
	};
	uint8_t Di_list[10] = {0, 1, 2, 4, 8, 16, 32, 0, 12, 20};

	pscatr->F = (uint32_t)Fifmax_list[(ta1 >> 4) & 0xf][0];
	pscatr->FMAX = (uint32_t)Fifmax_list[(ta1 >> 4) & 0xf][1];
	pscatr->D = (uint32_t)Di_list[ta1 & 0xf];
}

static void SCR_TB1_Decode(scatr_struct *pscatr, uint8_t tb1)
{
	uint8_t I_list[4] = {25, 50, 100, 50};

	pscatr->I = (uint32_t)I_list[(tb1 >> 5) & 0x3];

	if (((tb1 & 0x1f) > 4) && ((tb1 & 0x1f) < 26)) {
		pscatr->P = (tb1 & 0x1f); /* 5~25 in Volts */
	} else if ((tb1 & 0x1f) == 0) {
		pscatr->P = 0;  /* NC */
	} else {
		pscatr->P = 5;  /* NC */
	}
}

/* ATR:TS T0 TA1 TB1 TC1 TD1 TA2 TB2 TC2 TD2 ... ,TAi,TBi,TCi,TDi,T1 T2 T3 ... ,TK,TCK */
uint32_t SCR_ATR_Decode(scatr_struct *pscatr, uint8_t *pdata, upps_struct *pps, uint32_t with_ts)
{
	uint32_t i, index = 0;
	uint8_t  tag;

	pps->ppss = 0xff;   /* PPSS */
	pps->pps0 = 0;

	if (with_ts) {      /* TS */
		pscatr->TS = pdata[0];
		index++;
	}
	tag = pdata[index]; /* T0 */
	index++;
	pscatr->TK_NUM = tag & 0xf;

	if (tag & 0x10) {   /* TA1 */
		SCR_TA1_Decode(pscatr, pdata[index]);
		pps->pps0 |= 0x01 << 4;     /* pps0 */
		pps->pps1 = pdata[index];   /* pps1=TA1 Fd Dd */
		index++;
	}
	if (tag & 0x20) {   /* TB1 */
		SCR_TB1_Decode(pscatr, pdata[index]);
		index++;
	}
	if (tag & 0x40) {   /* TC1 */
		pscatr->N = pdata[index] & 0xff;
		index++;
	}
	if (tag & 0x80) {   /* TD1 */
		tag = pdata[index];
		pscatr->T = tag & 0xf;
		pps->pps0 |= tag & 0xf;
		if (pscatr->N == 0xff) {    /* N = 255,it stands for: when T=0,guard time=2 etu; when T=1,guard time=1 etu. */
			if (pscatr->T == 1) {   /* T=1,guard time = 1 etu */
				pscatr->N = 1;
			} else {                /* T=0,guard time = 2 etu */
				pscatr->N = 2;
			}
		}
		index++;
	} else {
		if (pscatr->N == 0xff) {    /* when T=0, guard time=2 etu. */
			pscatr->N = 2;
		}
		goto rx_tk;
	}

	if (tag & 0x10) {   /* TA2 */
		HAL_DBG("TA2 Exist!!\n");
		index++;
	}
	if (tag & 0x20) {   /* TB2 */
		HAL_DBG("TB2 Exist!!\n");
		index++;
	}
	if (tag & 0x40) {   /* TC2 */
		HAL_DBG("TC2 Exist!!\n");
		index++;
	}
	if (tag & 0x80) {   /* TD2 */
		HAL_DBG("TD2 Exist!!\n");
		tag = pdata[index];
		index++;
	} else {
		goto rx_tk;
	}

	if (tag & 0x10) {   /* TA3 */
		HAL_DBG("TA3 Exist!!\n");
		index++;
	}
	if (tag & 0x20) {   /* TB3 */
		HAL_DBG("TB3 Exist!!\n");
		index++;
	}
	if (tag & 0x40) {   /* TC3 */
		HAL_DBG("TC3 Exist!!\n");
		index++;
	}
	if (tag & 0x80) {   /* TD3 */
		HAL_DBG("TD3 Exist!!\n");
		tag = pdata[index];
		index++;
	} else {
		goto rx_tk;
	}

	if (tag & 0x10) {   /* TA4 */
		HAL_DBG("TA4 Exist!!\n");
		index++;
	}
	if (tag & 0x20) {   /* TB4 */
		HAL_DBG("TB4 Exist!!\n");
		index++;
	}
	if (tag & 0x40) {   /* TC4 */
		HAL_DBG("TC4 Exist!!\n");
		index++;
	}
	if (tag & 0x80) {   /* TD4 */
		HAL_DBG("TD4 Exist!!\n");
		tag = pdata[index];
		index++;
	} else {
		goto rx_tk;
	}

rx_tk:
	for (i = 0; i < pscatr->TK_NUM; i++) {
		pscatr->TK[i] = pdata[index++];
	}

	pps->pck = pps->ppss;
	pps->pck ^= pps->pps0;
	if (pps->pps0 & (0x1 << 4)) {
		pps->pck ^= pps->pps1;
	}
	if (pps->pps0 & (0x1 << 5)) {
		pps->pck ^= pps->pps2;
	}
	if (pps->pps0 & (0x1 << 6)) {
		pps->pck ^= pps->pps3;
	}

	return 0;
}

void SCR_FSMRrecordStart(pscr_fsm_record pfsm)
{
	pfsm->count = 1;
	pfsm->old = SCR_GetFSM();
	pfsm->record[0] = pfsm->old;
}

void SCR_FSMRecordRun(pscr_fsm_record pfsm)
{
	uint32_t data;

	if (pfsm->count >= SCR_FSM_MAX_RECORD)
		return;
	data = SCR_GetFSM();
	if (pfsm->old != data) {
		pfsm->old = data;
		pfsm->record[pfsm->count] = data;
		pfsm->count++;
	}
}

void SCR_IRQHandler(void)
{
	SCR_WorkStruct *pscr = SCR_GetWorkParams();
	uint32_t irqStatus = SCR_GetIRQStatus() & SCR_GetIRQEnableStatus();
	uint32_t rxCnt = 0;

	/* HAL_DBG("irq status -> 0x%08x\n", irqStatus); */

	SCR_ClearIRQStatus(irqStatus);

	if (irqStatus & SCR_INT_INSERT) {
		if (SCR_ISDetected()) { /* Smart Card Detected --- Input is active at least for a debounce time */
			pscr->detected = 1;
			pscr->atr_resp = SCR_ATR_RESP_INVALID;
			stage = STS_WAIT_CONNECT;
		}
	}

	if (irqStatus & SCR_INT_REMOVE) {
		if (!SCR_ISDetected()) {
			pscr->detected = 0;
			pscr->atr_resp = SCR_ATR_RESP_INVALID;
			stage = STS_START_DEACT;
		}
	}

	if (irqStatus & SCR_INT_ACT) {
		/* if (SCR_ISDetected()) */
		{
			pscr->activated = 1;
			pscr->atr_resp = SCR_ATR_RESP_INVALID;
		}
	}

	if (irqStatus & SCR_INT_DEACT) {
		/* if (!SCR_ISDetected()) */
		{
			pscr->activated = 0;
			pscr->atr_resp = SCR_ATR_RESP_INVALID;
			/* insert_sta = 1; */
		}
	}

	if (irqStatus & SCR_INT_ATR_FAIL) {
		pscr->atr_resp = SCR_ATR_RESP_FAIL;
	}

	if (irqStatus & SCR_INT_ATR_DONE) {
		pscr->atr_resp = SCR_ATR_RESP_OK;
	}

	if ((irqStatus & SCR_INT_TXFIFO_DONE) || (irqStatus & SCR_INT_TXFIFO_EMPTY) || (irqStatus & SCR_INT_TXFIFO_THR)) {
		while (pscr->txpriv.size > 0) {
			if (!SCR_TXFIFO_ISFull()) {
				SCR_WriteFIFO(*pscr->txpriv.buf);
				++pscr->txpriv.buf;
				--pscr->txpriv.size;
			}
		}

		if (!pscr->txpriv.size) {
			HAL_SemaphoreRelease(&pscr->txpriv.sem);
		}
	}

	if (irqStatus & SCR_INT_TX_DONE) {

	}

	if (irqStatus & SCR_INT_TXPARITY_ERR) {

	}

	if ((irqStatus & SCR_INT_RXFIFO_THR) || (irqStatus & SCR_INT_RXFIFO_FULL) || (irqStatus & SCR_INT_RX_DONE)) {
		/*if (stage != STS_RW_TEST) {
			while (!SCR_RXFIFO_ISEmpty()) {
				HAL_SCR_FillRxbuf(&pscr->rxbuf, SCR_ReadFIFO());
			}
		} else {
			*pscr->rxpriv.buf = SCR_ReadFIFO();
			++pscr->rxpriv.buf;
			--pscr->rxpriv.size;

			if (!pscr->rxpriv.size) {
				HAL_SemaphoreRelease(&pscr->rxpriv.sem);
			}
		}*/
		if (stage != STS_RW_TEST) {
			while (!SCR_RXFIFO_ISEmpty()) {
				HAL_SCR_FillRxbuf(&pscr->rxbuf, SCR_ReadFIFO());
			}
		} else {
			uint32_t difCnt;
			difCnt = SCR_GetRXFIFOCount();
#if 0
			HAL_DBG("rx_fifo -> %d\n", difCnt);
#endif
			rxCnt = difCnt > pscr->rxpriv.size ? pscr->rxpriv.size : difCnt;
			difCnt = difCnt - rxCnt;
			while (rxCnt) {
				*pscr->rxpriv.buf = SCR_ReadFIFO();
				++pscr->rxpriv.buf;
				--pscr->rxpriv.size;
				--rxCnt;
			}
			while (difCnt) {
				SCR_ReadFIFO();
				--difCnt;
			}

			if (!pscr->rxpriv.size)
				HAL_SemaphoreRelease(&pscr->rxpriv.sem);
		}
	}

	if (irqStatus & SCR_INT_RXPARITY_ERR) {

	}

	if (irqStatus & SCR_INT_CLOCK_STOPRUN) {

	}

	if (irqStatus & SCR_INT_C2CFULL) {
		pscr->chto_flag++;
	}
}

HAL_Status HAL_SCR_Init(SCR_InitParam *initParam)
{
	uint32_t apb_clk, baud_div, clk_div;
	SCR_Private *priv;
	SCR_WorkStruct *workScr;

	priv = SCR_GetPriv();
	workScr = SCR_GetWorkParams();
	if ((priv != NULL) || (workScr != NULL)) {
		HAL_DBG("smartcard-reader is inited already !\n");
		return HAL_BUSY;
	}

	priv = HAL_Malloc(sizeof(SCR_Private));
	workScr = HAL_Malloc(sizeof(SCR_WorkStruct));
	if ((priv == NULL) || (workScr == NULL)) {
		HAL_ERR("no mem\n");
		return HAL_ERROR;
	}
	HAL_Memset(priv, 0, sizeof(SCR_Private));
	SCR_SetPriv(priv);
	SCR_SetWorkParams(workScr);
	HAL_SemaphoreInit(&workScr->txpriv.sem, 0, 1);
	HAL_SemaphoreInit(&workScr->rxpriv.sem, 0, 1);

	priv->IRQCallback = initParam->IRQCallback;
	priv->arg = initParam->arg;

#ifdef CONFIG_PM
	priv->dev.name = scr_name;
	priv->dev.driver = &scr_drv;
	HAL_Memcpy(&priv->param, initParam, sizeof(SCR_InitParam));
	priv->dev.platform_data = priv;
	pm_register_ops(&priv->dev);
#endif

	/* enable SmartCard-Reader clock and release reset */
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_SMCARD);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_SMCARD);

	/* config pinmux */
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_SCR, 0), 0);

	SCR_DisableGlobalINT();
	SCR_DisableIRQ(SCR_INT_ALL_MASK);
	SCR_ClearIRQStatus(SCR_INT_ALL_MASK);
	SCR_EnableTSRx();
	SCR_EnableATRSTFlush();
	SCR_FlushFIFO(SCR_FLUSH_RXFIFO | SCR_FLUSH_TXFIFO);
	SCR_SetFIFOThreshold(initParam->fifo_thr, initParam->fifo_thr);
	SCR_SetRepeat(initParam->repeat_num, initParam->repeat_num);

	apb_clk = HAL_GetAPBClock();
	clk_div = (apb_clk / (initParam->mclk << 1)) - 1;
	baud_div = (clk_div + 1) * 372 - 1; /* before ATR response, the baud should be 372 mclock cycle */
	SCR_SetClkDivisor(baud_div, clk_div);
	HAL_DBG("apb clock -> %d \n", apb_clk);
	HAL_DBG("clk_div -> %d, baud_div -> %d \n", clk_div, baud_div);

	SCR_SetActTime(initParam->act_time);
	SCR_SetRstTime(initParam->rst_time);
	SCR_SetAtrTime(initParam->atr_time);
	SCR_SetCharLimit(initParam->char_limit);
	SCR_SetGuardTime(initParam->guard_time);
	SCR_EnableAutoVpp();
	SCR_SetDebounce(0xfffff);
	SCR_EnableRX();
	SCR_EnableTX();

	workScr->rxbuf.rptr = 0;
	workScr->rxbuf.wptr = 0;
	workScr->detected = 0;
	workScr->activated = 0;
	workScr->atr_resp = SCR_ATR_RESP_INVALID;
	workScr->chto_flag = 0;

	/* register NVIC IRQ */
	HAL_NVIC_ConfigExtIRQ(SMCARD_IRQn, SCR_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

	SCR_ClearIRQStatus(SCR_INT_ALL_MASK);
	/* enable SCR interrupt */
	SCR_EnableIRQ(SCR_INT_DEACT | SCR_INT_ACT | SCR_INT_INSERT | SCR_INT_REMOVE | SCR_INT_ATR_DONE | \
	              SCR_INT_ATR_FAIL | SCR_INT_C2CFULL | SCR_INT_CLOCK_STOPRUN | SCR_INT_RXPARITY_ERR | \
	              SCR_INT_RX_DONE | SCR_INT_RXFIFO_THR | SCR_INT_RXFIFO_FULL);
#if 0
	SCR_EnableIRQ(SCR_INT_ALL_MASK);
#endif
	SCR_EnableGlobalINT();

#if 0
	workScr->detected = 1;
	workScr->atr_resp = SCR_ATR_RESP_INVALID;
	stage = STS_WAIT_CONNECT;
#endif

	return HAL_OK;
}

HAL_Status HAL_SCR_DeInit(void)
{
	SCR_Private *priv;
	SCR_WorkStruct *workScr;

	priv = SCR_GetPriv();
	workScr = SCR_GetWorkParams();

#ifdef CONFIG_PM
	pm_unregister_ops(&priv->dev);
#endif

	SCR_DisableIRQ(SCR_INT_ALL_MASK);
	HAL_NVIC_DisableIRQ(SMCARD_IRQn);

	/* disable qdec clock and force reset */
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_SMCARD);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_SMCARD);

	/* De-config pinmux */
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_SCR, 0), 0);

	HAL_SemaphoreDeinit(&workScr->txpriv.sem);
	HAL_SemaphoreDeinit(&workScr->rxpriv.sem);
	SCR_SetPriv(NULL);
	SCR_SetWorkParams(NULL);
	HAL_Free(priv);
	HAL_Free(workScr);

	return HAL_OK;
}

SCR_TEST_STAGE HAL_SCR_Process(SCR_InitParam *initParam)
{
	uint32_t i = 0;
	uint8_t data = 0;
	uint8_t rsp_temp[30];
	SCR_WorkStruct *workScr;

	workScr = SCR_GetWorkParams();

	switch (stage) {
	case STS_WAIT_CONNECT:
		if (workScr->detected) {        /* if detected card, active the card */
			if (++state_dly >= 50) {
				SCR_ParamsInit(&scatr); /* set smart card protocol, v,i,frequency */
				SCR_MDelay(10);
				SCR_StartAct();         /* Activation */
				HAL_DBG("SmartCard Inserted!!\n");
				stage = STS_WAIT_ACT;
				state_dly = 0;
			}
		} else {
			state_dly = 0;
		}
		break;

	case STS_WAIT_ACT:
		if (workScr->activated) {
			stage = STS_WAIT_ATR;
			HAL_DBG("SmartCard Activated!!\n");
		}
		break;

	case STS_WAIT_ATR:
		if (workScr->atr_resp != SCR_ATR_RESP_INVALID) {
			if (workScr->atr_resp == SCR_ATR_RESP_OK) {     /* ATR response is successful */
				HAL_LOG(1, "ATR = ");
				for (i = 0; i < workScr->rxbuf.wptr; i++) {
					HAL_LOG(1, "%2x ", *((uint8_t *)(workScr->rxbuf.buffer + i)));
					rsp_temp[i] = *((uint8_t *)(workScr->rxbuf.buffer + i));
				}
				HAL_LOG(1, " \n");
				workScr->rxbuf.rptr = workScr->rxbuf.wptr;
				SCR_ATR_Decode(&scatr, (uint8_t *)workScr->rxbuf.buffer, &pps, 1);
				SCR_RxBuf_Flush(&workScr->rxbuf);   /* clean rxbuffer */
				stage = STS_START_PPS;

				for (i = 0; i < SMART_CARD_NUM; i++) {
					if (!HAL_Memcmp(rsp_temp, &ic_card_atr[i][1], ic_card_atr[i][0])) {
						card_id = i;
						HAL_DBG("card index in test list is -> %d\n", i);
						HAL_DBG("ATR Function PASS!!\n");
						break;
					}
				}

				if (i >= SMART_CARD_NUM) {
					HAL_DBG("ATR Function FAIL!!\n");
				}
			} else {
				stage = STS_START_DEACT;
			}
		}
		break;

	case STS_START_PPS:     /* Protocol and parameters selection */
		workScr->chto_flag = 0;
		SCR_RxBuf_Flush(&workScr->rxbuf);

		SCR_WriteFIFO(pps.ppss);
		HAL_DBG("ppss: %x\n", pps.ppss);
		SCR_WriteFIFO(pps.pps0);
		HAL_DBG("pps0: %x\n", pps.pps0);

		if (pps.pps0 & (0x1 << 4)) {
			SCR_WriteFIFO(pps.pps1);
			HAL_DBG("pps1: %x\n", pps.pps1);
		}
		if (pps.pps0 & (0x1 << 5)) {
			SCR_WriteFIFO(pps.pps2);
			HAL_DBG("pps2: %x\n", pps.pps2);
		}
		if (pps.pps0 & (0x1 << 6)) {
			SCR_WriteFIFO(pps.pps3);
			HAL_DBG("pps3: %x\n", pps.pps3);
		}

		SCR_WriteFIFO(pps.pck);
		HAL_DBG("pck: %x\n", pps.pck);

		stage = STS_WAIT_PPS_RESP;
		break;

	case STS_WAIT_PPS_RESP:
		if (workScr->chto_flag) {    /* Wait Data Timeout, time is too long, time is out */
			if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {    /* RX Buffer, No Response */
				stage = STS_WARM_RESET;
				HAL_LOG(1, "No PPS Response!!\n");
			} else {
				data = SCR_ReadRxBufByte(&workScr->rxbuf);     /* read ppss */
				HAL_LOG(1, "ppss: %x\n", data);

				if (data != pps.ppss) {    /* check ppss */
					HAL_ERR("PPS Resp Start Error: 0x%x !!\n", data);
					break;
				}
				if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {    /* no pps0 */
					HAL_DBG("PPS Resp Too Short 1\n");
					break;
				}

				data = SCR_ReadRxBufByte(&workScr->rxbuf);     /* read pps0 */
				HAL_DBG("pps0: %x\n", data);

				if (data != pps.pps0) {    /* check pps0 */
					HAL_ERR("PPS Resp PPS0 Error: 0x%x vs 0x%x !!\n", pps.pps0, data);
					break;
				}
				if (pps.pps0 & (0x1 << 4)) {
					if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {
						HAL_DBG("PPS Resp Too Short 2\n");
						break;
					}
					data = SCR_ReadRxBufByte(&workScr->rxbuf);
					HAL_LOG(1, "pps1: %x\n", data);
					if (data != pps.pps1) {    /* pps1 */
						HAL_ERR("PPS Resp PPS1 Error: 0x%x vs 0x%x !!\n", pps.pps1, data);
						break;
					}
				}
				if (pps.pps0 & (0x1 << 5)) {
					if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {
						HAL_DBG("PPS Resp Too Short 3\n");
						break;
					}
					data = SCR_ReadRxBufByte(&workScr->rxbuf);
					HAL_LOG(1, "pps2: %x\n", data);
					if (data != pps.pps2) {    /* pps2 */
						HAL_ERR("PPS Resp PPS2 Error: 0x%x vs 0x%x !!\n", pps.pps2, data);
						break;
					}
				}
				if (pps.pps0 & (0x1 << 6)) {
					if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {
						HAL_DBG("PPS Resp Too Short 4\n");
						break;
					}
					data = SCR_ReadRxBufByte(&workScr->rxbuf);
					HAL_LOG(1, "pps3: %x\n", data);
					if (data != pps.pps3) {    /* pps3 */
						HAL_ERR("PPS Resp PPS3 Error: 0x%x vs 0x%x !!\n", pps.pps3, data);
						break;
					}
				}
				if (SCR_RxBuf_ISEmpty(&workScr->rxbuf)) {
					HAL_DBG("PPS Resp Too Short 5\n");
					break;
				}
				data = SCR_ReadRxBufByte(&workScr->rxbuf);
				HAL_LOG(1, "pck: %x\n", data);
				if (data != pps.pck) {    /* /pck */
					HAL_ERR("PPS Resp PCK Error: 0x%x vs 0x%x !!\n", pps.pck, data);
					break;
				}

				SCR_RxBuf_Flush(&workScr->rxbuf);
				HAL_DBG("PPS Response OK!!\n");

#ifdef CONFIG_SCR_CARD_COMPATIBILITY_TEST
				stage = STS_RW_TEST;
				return stage;
#else
				stage = STS_SEND_CMD;
#endif
			}
		}
		break;

	case STS_WARM_RESET:
		workScr->atr_resp = SCR_ATR_RESP_INVALID;     /* IS OK ? */
		SCR_EnableWarmRst();
		stage = STS_WAIT_ATR_AGAIN;
		break;

	case STS_WAIT_ATR_AGAIN:
		if (workScr->atr_resp != SCR_ATR_RESP_INVALID) {
			if (workScr->atr_resp == SCR_ATR_RESP_OK) {
				HAL_LOG(1, "ATR = ");
				for (i = 0; i < workScr->rxbuf.wptr; i++) {
					HAL_LOG(1, "%2x ", *((uint8_t *)(workScr->rxbuf.buffer + i)));
					rsp_temp[i] = *((uint8_t *)(workScr->rxbuf.buffer + i));
				}
				HAL_LOG(1, " \n");

				SCR_ATR_Decode(&scatr, (uint8_t *)workScr->rxbuf.buffer, &pps, 1);

				stage = STS_START_PPS;

				for (i = 0; i < SMART_CARD_NUM; i++) {
					if (!HAL_Memcmp(rsp_temp, &ic_card_atr[i][1], ic_card_atr[i][0])) {
						card_id = i;
						HAL_DBG("ATR Function PASS!!\n");
						break;
					}
				}

				if (i >= SMART_CARD_NUM) {
					HAL_DBG("ATR Function FAIL!!\n");
				}

				SCR_SetClkDivisor((((SCR_GetClkDivisor() + 1) * scatr.F) / scatr.D) - 1, SCR_GetClkDivisor());
				SCR_RxBuf_Flush(&workScr->rxbuf);
#ifdef CONFIG_SCR_CARD_COMPATIBILITY_TEST
				stage = STS_RW_TEST;
				return stage;
#else
				stage = STS_SEND_CMD;
#endif
			} else {
				stage = STS_START_DEACT;
			}
		}
		break;

	case STS_SEND_CMD:
		if (ic_card_send_cmd[card_id][0]) {
			/*for (i = 0; i < ic_card_send_cmd[card_id][0]; i++) {
				data = ic_card_send_cmd[card_id][i + 1];
				SCR_WriteFIFO(data);
			}*/
			HAL_SCR_SendByPoll(&ic_card_send_cmd[card_id][i + 1], ic_card_send_cmd[card_id][0]);
			SCR_MDelay(1000);
			SCR_ReadRxBufAll(workScr, rsp_temp);

			if (!HAL_Memcmp(rsp_temp, &ic_card_rev_data[card_id][1], ic_card_rev_data[card_id][0])) {
				HAL_DBG("Communication Command Respone PASS!!\n");
			} else {
				HAL_DBG("Communication Command Error: ");
				for (i = 0; i < ic_card_rev_data[card_id][0]; i++) {
					HAL_LOG(1, "0x%02x  ", rsp_temp[i]);
				}
				HAL_LOG(1, "\n");
			}
		} else {
			HAL_DBG("No communication command, No command test!!\n");
			stage = STS_IDLE;
		}

		SCR_MDelay(1000);
		break;

	case STS_START_DEACT:
		stage = STS_WAIT_DEACT;
		HAL_DBG("SmartCard Removed!!\n\n");
		SCR_StartDeact();    /* Deactivation */
		SCR_MDelay(10);
		break;

	case STS_WAIT_DEACT:
		if (!workScr->activated) {
			stage = STS_WAIT_DISCONNECT;
#if 0
			HAL_SCR_DeInit();
			HAL_SCR_Init(scr_config);
#endif
			HAL_DBG("SmartCard Deactivated!!\n");
			return HAL_INVALID;
		}
		break;

	case STS_WAIT_DISCONNECT:
		if (!workScr->detected) {
			stage = STS_WAIT_CONNECT;
		}
		break;

	case STS_IDLE:
		SCR_MDelay(50);
		break;

	default:
		stage = STS_IDLE;
		break;
	}

	return stage;
}

HAL_Status HAL_SCR_APDUCmd(SCR_WRData *apdu_t)
{
	SCR_WRData *wr_data = apdu_t;
	int32_t rdata_len = 0;

	SCR_DisableIRQ(SCR_INT_RX_DONE | SCR_INT_RXFIFO_THR | SCR_INT_RXFIFO_FULL);

	if (5 == wr_data->cmd_len) { /* type1 */
		HAL_SCR_SendByPoll(wr_data->cmd_buf, 5);
		rdata_len = wr_data->cmd_buf[4] + 3;
		HAL_SCR_ReceiveByIT(wr_data->rtn_data, rdata_len);

		wr_data->rtn_len = wr_data->cmd_buf[4];
		HAL_Memmove(&wr_data->rtn_data[0], &wr_data->rtn_data[1], wr_data->cmd_buf[4]);
	} else if (wr_data->cmd_buf[4] + 5 == wr_data->cmd_len) { /* type2 */
		HAL_SCR_SendByPoll(wr_data->cmd_buf, 5);
		HAL_SCR_ReceiveByIT(wr_data->rtn_data, 1);
		if (wr_data->rtn_data[0] != wr_data->cmd_buf[1]) {
			HAL_ERR("do not support this instruction\n");
			return HAL_ERROR;
		}

		HAL_SCR_SendByPoll(&wr_data->cmd_buf[5], wr_data->cmd_buf[4]);
		rdata_len = 3;
		HAL_SCR_ReceiveByIT(wr_data->rtn_data, rdata_len);

		wr_data->rtn_len = 0;
	} else if (wr_data->cmd_buf[4] + 6 == wr_data->cmd_len) { /* type3 */
		HAL_SCR_SendByPoll(wr_data->cmd_buf, 5);
		if (wr_data->rtn_data[0] != wr_data->cmd_buf[1]) {
			HAL_ERR("do not support this instruction\n");
			return HAL_ERROR;
		}

		HAL_SCR_SendByPoll(&wr_data->cmd_buf[5], wr_data->cmd_buf[4] + 1);
		rdata_len = wr_data->cmd_buf[wr_data->cmd_len - 1] + 3;
		HAL_SCR_ReceiveByIT(wr_data->rtn_data, wr_data->cmd_buf[4] + 3);

		wr_data->rtn_len = rdata_len - 3;
		HAL_Memmove(&wr_data->rtn_data[0], &wr_data->rtn_data[1], wr_data->rtn_len);
	} else {
		HAL_ERR("invalid command format\n");
		return HAL_ERROR;
	}

	wr_data->psw1 = wr_data->rtn_data[rdata_len - 2];
	wr_data->psw2 = wr_data->rtn_data[rdata_len - 1];

	return HAL_OK;
}
