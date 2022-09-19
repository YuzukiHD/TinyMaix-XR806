/**
 * @file  hal_scr.h
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

#ifndef _DRIVER_CHIP_HAL_SCR_H_
#define _DRIVER_CHIP_HAL_SCR_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCR_MDelay(ms) HAL_MSleep(ms)
#define SCR_UDelay(us) HAL_UDelay(us)

#define SCR_ATR_RESP_INVALID        0
#define SCR_ATR_RESP_FAIL           1
#define SCR_ATR_RESP_OK             2

#define SCR_BUFFER_SIZE_MASK        0x3f  /* 64 */

/**
 * @brief SmartCard-Reader register block structure
 */
typedef struct {
	__IO uint32_t SCR_CSR;                  /* offset: 0x00, SmartCard-Reader control and status register */
	__IO uint32_t SCR_INT_EN;               /* offset: 0x04, SmartCard-Reader interrupt enable register */
	__IO uint32_t SCR_INT_ST;               /* offset: 0x08, SmartCard-Reader interrupt status register */
	__IO uint32_t SCR_FIFO_CSR;             /* offset: 0x0C, SmartCard-Reader FIFO control and status register */
	__IO uint32_t SCR_FIFO_CNT;             /* offset: 0x10, SmartCard-Reader FIFO counter register */
	__IO uint32_t SCR_FIFO_REPEAT;          /* offset: 0x14, SmartCard-Reader repeat control register */
	__IO uint32_t SCR_CLKDIV;               /* offset: 0x18, SmartCard-Reader clock divisor register */
	__IO uint32_t SCR_LTIM;                 /* offset: 0x1C, SmartCard-Reader line time register */
	__IO uint32_t SCR_CTIM;                 /* offset: 0x20, SmartCard-Reader character register */
	     uint32_t RESERVED1[3];
	__IO uint32_t SCR_PAD;                  /* offset: 0x30, SmartCard-Reader line control register */
	     uint32_t RESERVED2[2];
	__I  uint32_t SCR_FSM;                  /* offset: 0x3C, SmartCard-Reader FSM register */
	__IO uint32_t SCR_DT;                   /* offset: 0x40, SmartCard-Reader debounce time register */
	     uint32_t RESERVED3[47];
	__IO uint32_t SCR_FIFO_DATA;            /* offset: 0x100, SmartCard-Reader FIFO data register */
} SCREADER_T;

#define SCREDAER ((SCREADER_T *)ISO7816_CTRL_BASE)

#define SCR_FIFO_DEPTH              16

/* SCREDAER->SCR_CSR */
#define SCR_DETECT_BIT              HAL_BIT(31)
#define SCR_DETECT_POL_HIGH         HAL_BIT(24)

#define SCR_PROTOCOL_SELECT_SHIFT   22
#define SCR_PROTOCOL_SELECT_MASK    (0x3UL << SCR_PROTOCOL_SELECT_SHIFT)
typedef enum {
	SCR_PROTOCOL_T0     = 0UL,
	SCR_PROTOCOL_T1,
} SCR_PROTOL_SEL;

#define SCR_ATR_ST_FLUSH_FIFO_BIT   HAL_BIT(21)
#define SCR_TS_RX_EN                HAL_BIT(20)
#define SCR_CLK_STOP_PROTOL_BIT     HAL_BIT(19)
#define SCR_PARITY_ERR_RX_EN        HAL_BIT(18)
#define SCR_MSB_FIRST_EN            HAL_BIT(17)
#define SCR_DATA_POL_BIT            HAL_BIT(16)
#define SCR_DEACT_BIT               HAL_BIT(11)
#define SCR_ACT_BIT                 HAL_BIT(10)
#define SCR_WARM_RESET_BIT          HAL_BIT(9)
#define SCR_CLK_STOP_BIT            HAL_BIT(8)
#define SCR_GINT_EN                 HAL_BIT(2)
#define SCR_RX_EN                   HAL_BIT(1)
#define SCR_TX_EN                   HAL_BIT(0)

/* SCREDAER->SCR_INT_EN & SCREDAER->SCR_INT_ST */
#define SCR_INT_ALL_MASK            0xFF1E1FUL
#define SCR_INT_DEACT               HAL_BIT(23)
#define SCR_INT_ACT                 HAL_BIT(22)
#define SCR_INT_INSERT              HAL_BIT(21)
#define SCR_INT_REMOVE              HAL_BIT(20)
#define SCR_INT_ATR_DONE            HAL_BIT(19)
#define SCR_INT_ATR_FAIL            HAL_BIT(18)
#define SCR_INT_C2CFULL             HAL_BIT(17)  /* Character Timout */
#define SCR_INT_CLOCK_STOPRUN       HAL_BIT(16)
#define SCR_INT_RXPARITY_ERR        HAL_BIT(12)
#define SCR_INT_RX_DONE             HAL_BIT(11)
#define SCR_INT_RXFIFO_THR          HAL_BIT(10)
#define SCR_INT_RXFIFO_FULL         HAL_BIT(9)
#define SCR_INT_TXPARITY_ERR        HAL_BIT(4)
#define SCR_INT_TX_DONE             HAL_BIT(3)
#define SCR_INT_TXFIFO_THR          HAL_BIT(2)
#define SCR_INT_TXFIFO_EMPTY        HAL_BIT(1)
#define SCR_INT_TXFIFO_DONE         HAL_BIT(0)

/* SCREDAER->SCR_FIFO_CSR */
#define SCR_FLUSH_RXFIFO            HAL_BIT(10)
#define SCR_RXFIFO_FULL             HAL_BIT(9)
#define SCR_RXFIFO_EMPTY            HAL_BIT(8)
#define SCR_FLUSH_TXFIFO            HAL_BIT(2)
#define SCR_TXFIFO_FULL             HAL_BIT(1)
#define SCR_TXFIFO_EMPTY            HAL_BIT(0)

/* SCREDAER->SCR_FIFO_CNT */
#define SCR_FIFO_CNT_MASK           0xFFUL
#define SCR_RXFIFO_THR_SHIFT        24
#define SCR_TXFIFO_THR_SHIFT        16
#define SCR_RXFIFO_CNT_SHIFT        8
#define SCR_TXFIFO_CNT_SHIFT        0

/* SCREDAER->SCR_FIFO_REPEAT */
#define SCR_REPEAT_MASK             0xFUL
#define SCR_RX_REPEAT_NUM_SHIFT     4
#define SCR_TX_REPEAT_NUM_SHIFT     0

/* SCREDAER->SCR_CLKDIV */
#define SCR_CLKDIV_MASK             0xFFFFUL
#define SCR_BAUD_DIV_SHIFT          16
#define SCR_MCLK_DIV_SHIFT          0

/* SCREDAER->SCR_LTIM */
#define SCR_LTIM_8BTI_MASK          0xFFUL
#define SCR_LTIM_ATR_SHIFT          16
#define SCR_LTIM_RST_SHIFT          8
#define SCR_LTIM_ACT_SHIFT          0

/* SCREDAER->SCR_CTIM */
#define SCR_CTIM_CHARLIMT_MASK      0xFFFFUL
#define SCR_CTIM_CHARLIMT_SHIFT     16
#define SCR_CTIM_GUARDTIME_MASK     0xFFUL
#define SCR_CTIM_GUARDTIME_SHIFT    0

/* SCREDAER->SCR_PAD */
#define SCR_PAD_DSCVPP_PP           HAL_BIT(7)
#define SCR_PAD_DSCVPP_EN           HAL_BIT(6)
#define SCR_PAD_AUTO_ACT_VPP        HAL_BIT(5)
#define SCR_PAD_DSC_VCC             HAL_BIT(4)
#define SCR_PAD_DSC_RST             HAL_BIT(3)
#define SCR_PAD_DSC_CLK             HAL_BIT(2)
#define SCR_PAD_DSC_IO              HAL_BIT(1)
#define SCR_PAD_DIR_AC_PADS         HAL_BIT(0)

/* SCREDAER->SCR_FSM */
#define SCR_RSM_8BIT_MASK           0xFFUL
#define SCR_FSM_ATR_STRUCT          (SCR_RSM_8BIT_MASK << 24)
#define SCR_FSM_ATR                 (SCR_RSM_8BIT_MASK << 16)
#define SCR_FSM_ACT                 (SCR_RSM_8BIT_MASK << 8)
#define SCR_FSM_SCR                 (SCR_RSM_8BIT_MASK << 0)

/* SCREDAER->SCR_FIFO_DATA */
#define SCR_FIFO_DATA_MASK          0xFFUL

/* SCR Card State FSM */
#define SCR_CARD_IDLE               0x0
#define SCR_CARD_ACT                0x1
#define SCR_CARD_ATR                0x2
#define SCR_CARD_DATA               0x3
#define SCR_CARD_DEACT              0x4
/* SCR Active/Deactive State FSM */
#define SCR_ACT_INACTIVE            0x0
#define SCR_ACT_A1                  0x1  /* Vcc shall be powered */
#define SCR_ACT_A2                  0x2  /* I/O shall be put in reception mode */
#define SCR_ACT_A3                  0x3  /* Vpp shall be raised to idle state (if AutoVPP) */
#define SCR_ACT_ACTIVE              0x4  /* CLK shall be provided with a suitable and stable clock */
#define SCR_ACT_D1                  0x5  /* State L on RST */
#define SCR_ACT_D2                  0x6  /* State L on CLK */
#define SCR_ACT_D3                  0x7  /* Vpp inactive (if AutoVPP) */
#define SCR_ACT_D4                  0x8  /* State A on I/O */
#define SCR_ACT_D5                  0x9  /* Vcc inactive */
/* SCR ATR State FSM */
#define SCR_ATR_IDLE                0x0
#define SCR_ATR_RST                 0x1
#define SCR_ATR_WAIT                0x2
#define SCR_ATR_START               0x3
#define SCR_ATR_DONEx               0x4
#define SCR_ATR_FAILx               0x5
/* SCR ATR Structure State FSM */
#define SCR_STR_IDLE                0x0
#define SCR_STR_TS                  0x1
#define SCR_STR_T0                  0x2
#define SCR_STR_TX                  0x3
#define SCR_STR_TK                  0x4
#define SCR_STR_TCK                 0x5
#define SCR_STR_DONE                0x6
#define SCR_STR_FAIL                0x7

#define SCR_FSM_MAX_RECORD          1024
typedef struct {
	uint32_t count;
	uint32_t old;
	uint32_t record[SCR_FSM_MAX_RECORD];
} scr_fsm_record, *pscr_fsm_record;

typedef struct {
	uint8_t TS;
	uint8_t TK[15];
	uint8_t TK_NUM;
	uint32_t T;     /*Protocol */
	uint32_t FMAX;  /*in MHz */
	uint32_t F;
	uint32_t D;
	uint32_t I;     /* Max Cunrrent for Program, in mA */
	uint32_t P;     /* Program Voltage */
	uint32_t N;     /* Extra Guard Time, in ETUs */
} scatr_struct, *pscatr_struct;

typedef struct {
	uint8_t ppss;
	uint8_t pps0;
	uint8_t pps1;
	uint8_t pps2;
	uint8_t pps3;
	uint8_t pck;
} upps_struct, *ppps_struct;

typedef enum {
	STS_WAIT_CONNECT    = 0,
	STS_WAIT_ACT,
	STS_WAIT_ATR,
	STS_WARM_RESET,
	STS_WAIT_ATR_AGAIN,
	STS_START_PPS,
	STS_WAIT_PPS_RESP,
	STS_SEND_CMD,
	STS_START_DEACT,
	STS_WAIT_DEACT,
	STS_WAIT_DISCONNECT,
	STS_IDLE,
	STS_RW_TEST,
} SCR_TEST_STAGE;

/** @brief Type define of QDEC interrupt callback function */
typedef void (*SCR_IRQCallback)(void *arg);

/**
 * @brief SCR initialization parameters
 */
typedef struct {
	uint32_t mclk;              /* smart card clock frequency */
	uint8_t fifo_thr;           /* tx & rx fifo threshold */
	uint8_t repeat_num;         /* the re-transmission time when wrong parity */
	uint8_t act_time;           /* activation/deactivation time */
	uint8_t rst_time;           /* reset duration */
	uint8_t atr_time;           /* atr start limit */
	uint8_t guard_time;         /* character guard time */
	uint32_t char_limit;        /* character limit */
	SCR_IRQCallback    IRQCallback;
	void *arg;
} SCR_InitParam;

/* write cmd and read data immediately */
typedef struct {
	uint8_t *cmd_buf;
	uint32_t cmd_len;
	uint8_t *rtn_data;
	uint32_t rtn_len;
	uint8_t  psw1;
	uint8_t  psw2;
} SCR_WRData;

/**
 * @brief Initialize the SCR
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_SCR_Init(SCR_InitParam *initParam);

/**
 * @brief DeInitialize the SCR
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_SCR_DeInit(void);

SCR_TEST_STAGE HAL_SCR_Process(SCR_InitParam *initParam);

/**
 * @brief SCR write the assigned length data, send in polling mode
 * @retval Number of bytes transmitted, -1 on error
 */
int32_t HAL_SCR_SendByPoll(uint8_t *buf, int32_t size);

/**
 * @brief SCR write the assigned length data, suspend current thread until send finish
 * @retval Number of bytes transmitted, -1 on error
 */
int32_t HAL_SCR_SendByIT(uint8_t *buf, int32_t size);

/**
 * @brief SCR read the assigned length data, suspend current thread until received finish
 * @retval Number of bytes received, -1 on error
 */
int32_t HAL_SCR_ReceiveByIT(uint8_t *buf, int32_t size);

/**
 * @brief Send APDU command to card, and receive data from card
 * @param[in] apdu_t load apdu command parameters
 * @return 0 on success, -1 on error
 * @note APDU, smart card cammand format:
 *       send format:
 *         type1: CLS + INS + P1 + P2 + le -> only read, le=read size
 *         type2: CLS + INS + P1 + P2 + lc + data only lc, write data, lc=data size
 *         type3: CLS + INS + P1 + P2 + lc + data +le -> le + lc
 *       respond format:
 *         type1: INS(apdu_t->cmd_buf[1]) + valid_data(apdu_t->cmd_buf[4]) + SW1 + SW2
 *         type2: INS + SW1 + SW2
 *         type3: INS + valid_data + SW1 + SW2
 */
HAL_Status HAL_SCR_APDUCmd(SCR_WRData *apdu_t);

#ifdef __cplusplus
}
#endif

#endif    /* _DRIVER_CHIP_HAL_SCR_H_ */
