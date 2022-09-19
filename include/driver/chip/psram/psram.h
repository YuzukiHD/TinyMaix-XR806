/**
 * @file  psram.h
 * @author  XRADIO IOT WLAN Team
 */

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

#ifndef _PSRAM_H
#define _PSRAM_H

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_xip.h"
#include "kernel/os/os_semaphore.h"

#ifdef CONFIG_PSRAM

#define PSRAM_DBG_CHECK   0
#define PSRAM_DBG_ON      0
#define PSRAM_WRN_ON      1
#define PSRAM_ERR_ON      1
#define PSRAM_ABORT_ON    1

#define PSRAM_SYSLOG      printf
#define PSRAM_ABORT()     do { } while (0)

#define PSRAM_LOG(flags, fmt, arg...)   \
	do {                                \
		if (flags) {                    \
			__sram_rodata static char __fmt[] = fmt;    \
			PSRAM_SYSLOG(__fmt, ##arg); \
		}                               \
	} while (0)

#define PSRAM_DBG(fmt, arg...) \
	PSRAM_LOG(PSRAM_DBG_ON, "[psram] "fmt, ##arg)

#define PSRAM_INF(fmt, arg...) \
	PSRAM_LOG(PSRAM_WRN_ON, "[psram] "fmt, ##arg)

#define PSRAM_WRN(fmt, arg...) \
	PSRAM_LOG(PSRAM_WRN_ON, "[psram WRN] "fmt, ##arg)

#define PSRAM_ERR(fmt, arg...)                              \
	do {                                                    \
		PSRAM_LOG(PSRAM_ERR_ON, "[psram ERR] "fmt, ##arg);  \
		if (PSRAM_ABORT_ON)                                 \
			PSRAM_ABORT();                                  \
	} while (0)

#if PSRAM_DBG_ON
	#define PSRAM_DUMP(a, l) print_hex_dump_bytes(a, l)
#else
	#define PSRAM_DUMP(a, l)
#endif

#ifdef CONFIG_PLATFORM_FPGA
	#define PSRAM_FREQ (6000000)
#else
	#if CONFIG_PSRAM_FREQ_96M
		#define PSRAM_FREQ (96000000)
	#elif CONFIG_PSRAM_FREQ_120M
		#define PSRAM_FREQ (120000000)
	#else
		#error "select a freq for psram!"
	#endif
#endif

/*---------PSRAM SPI/QPI Command set------*/
#if (defined CONFIG_PSRAM_CHIP_SQPI)
#define SQ_Read                 0x03
#define SQ_Fast_Read            0x0B    /* 66MHz, wait 4 cycle every read */
#define SQ_Fast_Read_Quad       0xEB    /* 144/84MHz, wait 6 cycle every read */
#define SQ_Write                0x02
#define SQ_Quad_Write           0x38
#define SQ_Mode_Reg_Read        0xB5
#define SQ_Mode_Reg_Write       0xB1
#define SQ_Wrapped_Read         0x8B
#define SQ_Wrapped_Write        0x82
#define SQ_ModeResister_Read    0xB5
#define SQ_ModeResister_Write   0xB1
#define SQ_Enter_Quad_Mode      0x35
#define SQ_Exit_Quad_Mode       0xF5
#define SQ_Reset_Enable         0x66
#define SQ_Reset                0x99
#define SQ_Wrap                 0xC0
#define SQ_Read_ID              0x9F
/*--------PSRAM OPI Command set-----------*/
#elif ((defined CONFIG_PSRAM_CHIP_OPI32) || (defined CONFIG_PSRAM_CHIP_OPI64))
#define Sync_Read               0x00
#define Sync_Write              0x80
#define Sync_Burst_Read         0x20
#define Sync_Burst_Write        0xA0
#define Mode_Reg_Read           0x40
#define Mode_Reg_Write          0xC0
#define Global_Reaet            0xFF
#endif

/*--------OPI Mode Register Address-------*/
#define MR0                     0x00
#define MR1                     0x01
#define MR2                     0x02
#define MR3                     0x03
#define MR4                     0x04
#define MR5                     0x05
#define MR6                     0x06
#define MR7                     0x07

/*--------Drive Strength-----------------*/
#define DRV_STR_50_OHM          0
#define DRV_STR_100_OHM         1
#define DRV_STR_200_OHM         2

/*--------------define by myself----------*/
#define S_READ                  0x00
#define S_FAST_READ             0x01
#define S_FAST_READ_QUAD        0x02
#define S_WRITE                 0x03
#define S_QAUD_WRITE            0x04
#define Q_FAST_READ             0x05
#define Q_FAST_READ_QUAD        0x06
#define Q_WRITE                 0x07
#define O_SYNC_READ             0x08
#define O_SYNC_WRITE            0x09
#define O_SYNC_BURST_READ       0x0A
#define O_SYNC_BURST_WRITE      0x0B

/*--------SQPI MODE Reset-----------------*/
#define S_RST                   0
#define Q_RST                   1

#define P_DMA_B1W8              0
#define P_DMA_B1W16             1
#define P_DMA_B1W32             2
#define P_DMA_B4W8              3
#define P_DMA_B4W16             4
#define P_DMA_B4W32             5

#define FLUSH_LEN_HSIZE         32
#define FLUSH_LEN_CSIZE_128     128
#define FLUSH_LEN_CSIZE_256     256
#define FLUSH_LEN_CSIZE_896     896

#define FLUSH_MODE              FLUSH_LEN_CSIZE_128

#define DCACHE_OPEN

/*  chip information  */
#define PSRAM_CHIP_SQPI         0
#define PSRAM_CHIP_OPI_APS32    1
#define PSRAM_CHIP_OPI_APS64    2
#define PSRAM_CHIP_MAX          3

/**
 * @brief PSRAM initialization parameters
 */
typedef struct {
	uint32_t p_type;
	uint32_t freq;          /*!< PSRAM Chip working frequency */
} PSRAMChip_InitParam;

struct psram_chip {
	uint8_t                 id;
	uint8_t                 type;
	uint8_t                 ref;
	uint8_t                 suspend;

	uint8_t                 cbus_rcmd;
	uint8_t                 cbus_wcmd;
	uint8_t                 mf_id;
	uint8_t                 kgd;

	uint32_t                die;

	uint32_t                buswidth;
	uint32_t                wrap_len;
	uint32_t                capacity;
	uint32_t                freq;
	char                    *name;
#ifdef CONFIG_PSRAM_PM_DATA_CHECK
	uint16_t                pm_checksum;
#endif
	struct psram_ctrl       *ctrl;
};

#define PSRAM_DATA_WRITE_BYTE   (1 << 0)
#define PSRAM_DATA_WRITE_SHORT  (1 << 1)
#define PSRAM_DATA_WRITE_WORD   (1 << 2)
#define PSRAM_DATA_WRITE_MASK   (0x07 << 0)
#define PSRAM_DATA_READ_BYTE    (1 << 4)
#define PSRAM_DATA_READ_SHORT   (1 << 5)
#define PSRAM_DATA_READ_WORD    (1 << 6)
#define PSRAM_DATA_READ_MASK    (0x07 << 4)

struct psram_data {
	uint32_t blksz;         /* data block size */
	uint32_t blocks;        /* number of blocks */
	uint32_t flags;
	uint32_t busconfig;
	uint8_t *buff;
};

#define PSRAM_ADDR_PRESENT      (1 << 0)


struct psram_command {
	uint32_t opcode;
	uint32_t addr;
	uint8_t *resp;
	uint32_t flags;         /* expected response type */
	uint32_t dummy;
	uint32_t busconfig;
};

struct psram_request {
	struct psram_command    cmd;
	struct psram_data       data;
};

struct psram_ctrl;

extern uint8_t __PSRAM_BASE[];
extern uint8_t __PSRAM_END[];
extern uint8_t __PSRAM_LENGTH[];
extern uint8_t __psram_start__[];
extern uint8_t __psram_end__[];
extern uint8_t __psram_data_start__[];
extern uint8_t __psram_data_end__[];
extern uint8_t __psram_bss_start__[];
extern uint8_t __psram_bss_end__[];

int32_t psram_init(struct psram_chip *chip, struct psram_ctrl *ctrl, PSRAMChip_InitParam *param);
int32_t psram_deinit(struct psram_chip *chip);

/**
 * @brief Open psram controller SBUS.
 * @note At the same time, it will disable XIP and suspend schedule.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
struct psram_chip *psram_open(uint32_t id);

/**
 * @brief Close psram controller SBUS.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
HAL_Status psram_close(struct psram_chip *chip);

void psram_info_dump(struct psram_chip *chip);

#endif /* CONFIG_PSRAM */

#define RANGEOF_PSRAM(addr, len) \
	((((uint32_t)addr) >= (PSRAM_START_ADDR)) && ((((uint32_t)addr)+(len)) <= (PSRAM_END_ADDR)))

#endif /* _PSRAM_H */
