/**
  * @file  cmd_ble_etf.c
  * @author  XRADIO Bluetooth Team
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

/*
 * ===========================================================================
 *
 * -------------------------- temporary --------------------------------------
 *
 * ===========================================================================
 */
#ifdef PRJCONF_BLE_ETF
#include <stdio.h>
#include "cmd_util.h"

#include "blec.h"
#include "driver/chip/hal_wdg.h"
#include "kernel/os/os.h"

/* 0x01: command, 0x02: acl data, 0x04:event */
#define  LE_ETF_HCI_CMD_PCKT 1
#define  LE_ETF_HCI_ACL_DATA_PCKT 2
#define  LE_ETF_HCI_EVNT_PCKT 4

#define LE_ETF_OP(ogf, ocf)                         ((ocf) | ((ogf) << 10))
#define LE_ETF_OGF_LE                               0x08
#define LE_ETF_OGF_VS                               0x3f

#define LE_ETF_HCI_OP_LE_TX_TEST                    LE_ETF_OP(LE_ETF_OGF_LE, 0x001e)
#define LE_ETF_HCI_OP_LE_RX_TEST                    LE_ETF_OP(LE_ETF_OGF_LE, 0x001d)
#define LE_ETF_HCI_OP_LE_TEST_END                   LE_ETF_OP(LE_ETF_OGF_LE, 0x001f)
#define LE_ETF_HCI_OP_LE_ENH_RX_TEST                LE_ETF_OP(LE_ETF_OGF_LE, 0x0033)
#define LE_ETF_HCI_OP_LE_ENH_TX_TEST                LE_ETF_OP(LE_ETF_OGF_LE, 0x0034)
#define LE_ETF_HCI_SINGLE_TONE_OPEN                 LE_ETF_OP(LE_ETF_OGF_VS, 0x0043)
#define LE_ETF_HCI_SET_TEST_PWR_FEC_OPCODE          LE_ETF_OP(LE_ETF_OGF_VS, 0x0044)
#define LE_ETF_HCI_READ_RSSI                        LE_ETF_OP(LE_ETF_OGF_VS, 0x0015)
#define LE_ETF_HCI_OP_LE_SET_POWER                  LE_ETF_OP(LE_ETF_OGF_VS, 0x0303)
#define LE_ETF_HCI_OP_LE_SET_POWER_MAX              LE_ETF_OP(LE_ETF_OGF_VS, 0x0304)
#define LE_ETF_HCI_DBG_RD_MEM_CMD_OPCODE            LE_ETF_OP(LE_ETF_OGF_VS, 0x0001)
#define LE_ETF_HCI_DBG_WR_MEM_CMD_OPCODE            LE_ETF_OP(LE_ETF_OGF_VS, 0x0002)

#define CMD_OGF(pckt)     (((uint8_t)pckt[1]) >> 2)
#define CMD_OCF(pckt)     (((((uint16_t)pckt[1])&0x03) << 8)| ((uint16_t)pckt[0]))

#define HCI_MAX_EVENT_SIZE    260

#define UINT8_TO_STREAM(p, u8) \
  { *(p)++ = (uint8_t)(u8); }

#define UINT16_TO_STREAM(p, u16)    \
  {                                 \
	*(p)++ = (uint8_t)(u16);        \
	*(p)++ = (uint8_t)((u16) >> 8); \
  }

typedef struct {
	uint8_t  chan;
	uint8_t  payload_len;
	uint8_t  payload_type;
	uint8_t  phy;
	uint8_t  mod_index;
	uint8_t  power_level;
	uint8_t  power;
	uint8_t  hopping_enable;
} ble_etf_config;

#define BLE_ETF_HCI_CMD_TIMEOUT 2000

#define BLE_ETF_ENABLE_CHECK()  \
	do { \
		if (!ble_etf_enable) { \
			CMD_ERR("ble etf not enable\n"); \
			return CMD_STATUS_FAIL; \
		} \
	} while (0)

#define HCI_TRACE_PRINTK_MAX_SIZE 128
#define _8_Bit                              8
/// 16 bit access types
#define _16_Bit                             16
/// 32 bit access types
#define _32_Bit                             32

struct dbg_rd_mem_cmd {
	///Start address to read
	uint32_t start_addr;
	///Access size
	uint8_t type;
	///Length to read
	uint8_t length;
};

struct buffer_tag {
	/// length of buffer
	uint8_t length;
	/// data of 128 bytes length
	uint8_t data[HCI_TRACE_PRINTK_MAX_SIZE];
};

struct dbg_wr_mem_cmd {
	///Start address to read
	uint32_t start_addr;
	///Access size
	uint8_t type;
	///buffer structure to return
	struct buffer_tag buf;
};

int etf_blec_hci_c2h(unsigned char hci_type,
                     const unsigned char *buff,
                     unsigned int offset,
                     unsigned int len);

int etf_blec_hci_h2c_cb(unsigned char status,
                        const unsigned char *buff,
                        unsigned int offset,
                        unsigned int len);

static blec_hci_t ble_hci_hst = {
	.blec_hci_c2h = etf_blec_hci_c2h,
	.blec_hci_h2c_cb = etf_blec_hci_h2c_cb,
};

static ble_etf_config ble_etf_cfg = {
	.chan = 0,
	.payload_len = 37,
	.payload_type = 0,
	.phy = 0x01,
	.mod_index = 0x00,
	.power_level = 3,
	.power = 0xff,
	.hopping_enable = 0,
};

OS_Semaphore_t ble_etf_sem;
static uint8_t ble_etf_enable;

static const char *ble_etf_help =
	"tx                start tx test\n"
	"tx_stop           stop tx test\n"
	"rx                start rx test\n"
	"rx_stop           stop rx test\n"
	"channel <param>   set chan\n"
	"                      0~39 Frequency Range:2402 MHz to 2480 MHz\n"
	"rate <param>      set rate\n"
	"                      when tx: 0x01 phy 1M, 0x02 2M, 0x03 coded s=8, 0x04 coded s=2\n"
	"                      when rx, 0x01 phy 1M, 0x02 2M, 0x03 coded\n"
	"payload <param>   set payload type\n"
	"                      0x00 Pseudo-Random bit sequence 9\n"
	"                      0x01 Pattern of alternating bits '11110000'\n"
	"                      0x02 Pattern of alternating bits '10101010'\n"
	"                      0x03 Pseudo-Random bit sequence 15\n"
	"                      0x04 Pattern of All '1' bits\n"
	"                      0x05 Pattern of All '0' bits\n"
	"                      0x06 Pattern of alternating bits '00001111'\n"
	"                      0x07 Pattern of alternating bits '0101'\n"
	"len <param>       set payload len\n"
	"                      0~251 Length in bytes of payload data in each packet.\n"
	"hopping           start hopping when tx/rx\n"
	"hopping_stop      stop hopping when tx/rx\n"
	"power_level <param> set power level\n"
	"                       0~12 power level.\n"
	"read_mem <addr> <len> read mem value,example 0x60110404 4\n"
	"                          len must 4times and 0~128.\n"
	"                          addr must 4 times.\n"
	"write_mem <addr> <len> write mem value,example 0x60110404 0x00000008\n"
	"                          addr must 4 times.\n"
	"set_channel_fec <ch0_pwr_fec> <ch20_pwr_fec> <ch39_pwr_fec> set test power fec."
	"                          pwr_fec must be -128 to 127";

static const char *bt_etf_help =
	"ble               ble cmd, you can use <btetf ble help> to see cmd\n"
	"tone              start send single tone\n"
	"tone_stop         stop send single tone\n"
	"connect           ble fw init\n"
	"disconnect        ble fw deinit, sys reboot\n";

void ble_etf_hci_printf(uint8_t hci_type, uint8_t *buf, uint16_t len)
{
	switch (hci_type) {
	case LE_ETF_HCI_CMD_PCKT:
		printf("C-CMD(ogf 0x%02x, ocf 0x%04x) [%d]\n", CMD_OGF(buf), CMD_OCF(buf), len);
		break;

	case LE_ETF_HCI_EVNT_PCKT:
		printf("C-EVT(ogf 0x%02x, ocf 0x%04x) [%d]\n", (((uint8_t)buf[4]) >> 2),
		               (((((uint16_t)buf[4])&0x03) << 8)| ((uint16_t)buf[3])), len);
		break;

	case LE_ETF_HCI_ACL_DATA_PCKT:
		printf("C-ACL [%d]\n", len);
		break;

	default:
		printf("C-ERR [%d]\n", len);
		break;
	}

	for (uint16_t i = 0; i < len; i++) {
		printf("%02X ", *(buf + i));
		if ((i % 16 == 0) && (i != 0))
			printf("\n");
	}
	printf("\n");
}

static int cmd_ble_etf_parse_int(const char *value, int min, int max, int *dst)
{
	int val;
	char *end;

	val = cmd_strtol(value, &end, 10);
	if (*end) {
		CMD_ERR("Invalid number '%s'", value);
		return -1;
	}

	if (val < min || val > max) {
		CMD_ERR("out of range value %d (%s), range is [%d, %d]\n",
		     val, value, min, max);
		return -1;
	}

	*dst = val;
	return 0;
}

void etf_blec_hcic_init(void)
{
	blec_hci_init(&ble_hci_hst);
}

int etf_blec_hci_h2c_cb(unsigned char status,
                     const unsigned char *buff,
                     unsigned int offset,
                     unsigned int len)
{
	//CMD_DBG("h2c_cb status %d, offset %d, len %d\n", status, offset, len);
	if (status != 0) {
		CMD_ERR("fw rx cmd err:%s status %d, offset %d, len %d\n",
		    __func__, status, offset, len);
	}
#if 0
	int i = 0;
	for (i = 0; i < len; i++) {
		CMD_DBG("%02x ", buff[i]);
	}
	CMD_DBG("\n");
#endif
	return status;
}
uint32_t read_start_addr;

/* Event_code | param_tot_len | num cmd pkt of host to control|opcode | status */
/*   1byte    |       1       |               1               |   2   |    1     */

int etf_blec_hci_c2h(unsigned char hci_type,
                 const unsigned char *buff,
                 unsigned int offset,
                 unsigned int len)
{
	int status = 0;
	uint16_t opcode = 0;
	int event_status = 0;
	uint16_t rx_pkt_count = 0;
	int8_t rssi = 0;
	int i = 0;
	int read_mem_len = 0;
	uint8_t *read_mem_buff;

	ble_etf_hci_printf(hci_type, (uint8_t *)buff, len);

	if ((hci_type == LE_ETF_HCI_EVNT_PCKT) && (len >= 6)) {
		opcode = *((uint16_t *)&buff[3]);
		event_status = buff[5];
		printf("event status %d\n", event_status);
		switch (opcode) {
		case LE_ETF_HCI_OP_LE_TEST_END:
			rx_pkt_count = *((uint16_t  *)&buff[6]);
			printf("rx_pkt_count %u\n", rx_pkt_count);
			break;
		case LE_ETF_HCI_READ_RSSI:
			rssi = (int8_t)buff[6];
			printf("rssi %u\n", rssi);
			break;
		case LE_ETF_HCI_DBG_RD_MEM_CMD_OPCODE:
			read_mem_buff = (uint8_t *)&buff[6];
			read_mem_len = (uint8_t)buff[1]-4;
			printf("read len %d\n", read_mem_len);
			for (i = 0; i < read_mem_len; i += 4) {
				printf("addr:0x%08x, value:0x%08x\n", (read_start_addr + i),
				    *((uint32_t *)(read_mem_buff + i)));
			}
			read_start_addr = 0;
		default:
			break;
		}
	} else {
		printf("err: not hci event\n");
	}

	blec_hci_c2h_cb(0, buff, offset, len);

	OS_SemaphoreRelease(&ble_etf_sem);

	return status;
}

#if 0
static int ble_etf_tx(uint8_t chan, uint8_t payload_len, uint8_t payload)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_TX_TEST;
	uint8_t len = 0x03;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, chan);
	UINT8_TO_STREAM(ptr, payload_len);
	UINT8_TO_STREAM(ptr, payload);
	buff_len = 6; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_rx(uint8_t chan)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_RX_TEST;
	uint8_t len = 0x01;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, chan);
	buff_len = 4; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;

}
#endif

static int ble_etf_tx_rx_stop(void)
{
	int ret;

	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_TEST_END;
	uint8_t len = 0x00;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	buff_len = 3; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_enhanced_tx(uint8_t chan, uint8_t payload_len, uint8_t payload_type, uint8_t phy)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_ENH_TX_TEST;
	uint8_t len = 0x04;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, chan);
	UINT8_TO_STREAM(ptr, payload_len);
	UINT8_TO_STREAM(ptr, payload_type);
	UINT8_TO_STREAM(ptr, phy);
	buff_len = 7; /*acording to up context*/

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_enhanced_rx(uint8_t chan, uint8_t phy, uint8_t mod_index)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_ENH_RX_TEST;
	uint8_t len = 0x03;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	if (phy == 0x04) {
		phy = 0x03;
	}

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, chan);
	UINT8_TO_STREAM(ptr, phy);
	UINT8_TO_STREAM(ptr, mod_index); /* fw no use*/
	buff_len = 6; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_stone(uint8_t chan)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_SINGLE_TONE_OPEN;
	uint8_t len = 0x03;
	uint8_t open = 0x01;
	uint8_t power = 0x01; /* power param no use*/

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, open);
	UINT8_TO_STREAM(ptr, chan);
	UINT8_TO_STREAM(ptr, power);
	buff_len = 6; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_stone_stop(void)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_SINGLE_TONE_OPEN;
	uint8_t len = 0x01;
	uint8_t open = 0x00;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, open);

	buff_len = 4; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_set_pwr_fec(int8_t pwr_fec_ch0, int8_t pwr_fec_ch20, int8_t pwr_fec_ch39)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_SET_TEST_PWR_FEC_OPCODE;
	uint8_t len = 0x03;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, (uint8_t)pwr_fec_ch0);
	UINT8_TO_STREAM(ptr, (uint8_t)pwr_fec_ch20);
	UINT8_TO_STREAM(ptr, (uint8_t)pwr_fec_ch39);
	buff_len = 6; /*acording to up context*/

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_get_rssi(void)
{
	int ret;

	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_READ_RSSI;
	uint8_t len = 0x00;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	buff_len = 3; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_set_power(uint8_t power)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_SET_POWER;
	uint8_t len = 0x01;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, power);
	buff_len = 4; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_set_power_max(uint8_t power)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_OP_LE_SET_POWER_MAX;
	uint8_t len = 0x01;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	UINT8_TO_STREAM(ptr, len);
	UINT8_TO_STREAM(ptr, power);
	buff_len = 4; /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_read_mem(uint32_t start_addr, uint32_t read_len)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_DBG_RD_MEM_CMD_OPCODE;
	uint8_t len = 0;
	struct dbg_rd_mem_cmd param;
	read_start_addr = start_addr;

	param.start_addr = start_addr;
	param.type = _32_Bit;
	param.length = read_len;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	len = sizeof(struct dbg_rd_mem_cmd);
	UINT8_TO_STREAM(ptr, len);
	memcpy(ptr, &param, sizeof(struct dbg_rd_mem_cmd));
	buff_len = 2 + 1 + sizeof(struct dbg_rd_mem_cmd); /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		read_start_addr = 0;
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, buff_len);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static int ble_etf_write_mem(uint32_t start_addr, uint32_t value)
{
	int ret = -1;
	uint8_t buf[HCI_MAX_EVENT_SIZE];
	uint8_t *ptr = buf;
	uint16_t opcode = LE_ETF_HCI_DBG_WR_MEM_CMD_OPCODE;
	uint8_t len = 0;
	struct dbg_wr_mem_cmd param;

	param.start_addr = start_addr;
	param.type = _32_Bit;
	param.buf.length = 4;
	memset(param.buf.data, 0, HCI_TRACE_PRINTK_MAX_SIZE);
	*((uint32_t *)(&param.buf.data[0])) = value;

	uint32_t buff_len = 0;
	uint32_t buff_offset = 0;
	uint32_t hci_type = LE_ETF_HCI_CMD_PCKT;
	uint8_t *buff_start = buf;

	UINT16_TO_STREAM(ptr, opcode);
	len = sizeof(struct dbg_wr_mem_cmd);
	UINT8_TO_STREAM(ptr, len);
	memcpy(ptr, &param, sizeof(struct dbg_wr_mem_cmd));
	buff_len = 2 + 1 + sizeof(struct dbg_wr_mem_cmd); /*acording to up context*/

	if (OS_SemaphoreWait(&ble_etf_sem, BLE_ETF_HCI_CMD_TIMEOUT) != OS_OK) {
		CMD_DBG("sem wait fail\n");
		read_start_addr = 0;
		return -1;
	}

	ble_etf_hci_printf(hci_type, buff_start, param.buf.length);

	ret = blec_hci_h2c(hci_type, buff_start, buff_offset, buff_len);

	return ret;
}

static enum cmd_status cmd_ble_etf_tx_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_enhanced_tx(ble_etf_cfg.chan, ble_etf_cfg.payload_len,
	          ble_etf_cfg.payload_type, ble_etf_cfg.phy);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_tx_stop_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_tx_rx_stop();

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_rx_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_enhanced_rx(ble_etf_cfg.chan, ble_etf_cfg.phy, 00);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_rx_stop_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ble_etf_get_rssi();
	ret = ble_etf_tx_rx_stop();

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_chan_exec(char *cmd)
{
	int ret = 0;
	int chan;

	BLE_ETF_ENABLE_CHECK();
	ret = cmd_ble_etf_parse_int(cmd, 0, 39, &chan);
	if (ret == 0) {
		ble_etf_cfg.chan = chan;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_rate_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	if (cmd_strcmp(cmd, "1M") == 0) {
		ble_etf_cfg.phy = 0x01;
	} else if (cmd_strcmp(cmd, "2M") == 0) {
		ble_etf_cfg.phy = 0x02;
	} else if (cmd_strcmp(cmd, "S2") == 0) {
		ble_etf_cfg.phy = 0x03;
	} else if (cmd_strcmp(cmd, "S8") == 0) {
		ble_etf_cfg.phy = 0x04;
	} else {
		if (cmd) {
			CMD_ERR("err:not support rate %s\n", cmd);
		}
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_payload_exec(char *cmd)
{
	int ret = 0;
	int payload;

	BLE_ETF_ENABLE_CHECK();
	ret = cmd_ble_etf_parse_int(cmd, 0, 7, &payload);
	if (ret == 0) {
		ble_etf_cfg.payload_type = payload;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_payload_len_exec(char *cmd)
{
	int ret = 0;
	int payload_len;

	BLE_ETF_ENABLE_CHECK();
	ret = cmd_ble_etf_parse_int(cmd, 0, 251, &payload_len);
	if (ret == 0) {
		ble_etf_cfg.payload_len = payload_len;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static OS_Thread_t g_hopping_thread;
#define THREAD_STACK_SIZE       (1 * 1024)
uint8_t hopping_run_flag;
HAL_Status HAL_PRNG_Generate(uint8_t *random, uint32_t size);

static void hopping_task(void *arg)
{
	uint8_t random_value = 0;
	uint8_t chan = 1;
	while (hopping_run_flag) {
		if (HAL_PRNG_Generate(&random_value, 1) == HAL_OK) {
			chan = random_value % 40;
		} else {
			CMD_DBG("get random value err\n");
		}
		ble_etf_tx_rx_stop();
		OS_MSleep(10);
		ble_etf_enhanced_tx(chan, ble_etf_cfg.payload_len,
		          ble_etf_cfg.payload_type, ble_etf_cfg.phy);

		OS_MSleep(50);
	}

	OS_ThreadDelete(&g_hopping_thread);
}

static enum cmd_status cmd_ble_etf_hopping_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();

	hopping_run_flag = 1;
	if (OS_ThreadCreate(&g_hopping_thread,
	                    "hopping",
	                    hopping_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    THREAD_STACK_SIZE) != OS_OK) {
		CMD_ERR("create ak thread failed\n");
		ret = -1;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_hopping_stop_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	hopping_run_flag = 0;
	OS_MSleep(50);
	ble_etf_tx_rx_stop();

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_power_exec(char *cmd)
{
	int ret = 0;
	int power_level;

	BLE_ETF_ENABLE_CHECK();
	ret = cmd_ble_etf_parse_int(cmd, 0, 12, &power_level);
	if (ret == 0) {
		ret = ble_etf_set_power(power_level);
		if (ret == 0)
			ble_etf_cfg.power_level = power_level;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_power_max_value_exec(char *cmd)
{
	int ret = 0;
	int power;

	BLE_ETF_ENABLE_CHECK();
	ret = cmd_ble_etf_parse_int(cmd, 0, 255, &power);
	if (ret == 0) {
		ret = ble_etf_set_power_max(power);
		if (ret == 0)
			ble_etf_cfg.power = power;
	}

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_tone_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_stone(ble_etf_cfg.chan);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_tone_stop_exec(char *cmd)
{
	int ret = 0;

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_stone_stop();

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_read_mem_exec(char *cmd)
{
	int ret = 0;
	int32_t cnt;
	uint32_t addr;
	int len;

	cnt = cmd_sscanf(cmd, "0x%x %d", &addr, &len);
	if (cnt != 2) {
		CMD_ERR("example: 0x10 0x10\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if (addr % 4 != 0) {
		CMD_ERR("addr must be 4 byte align\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if ((len % 4 != 0) || (len <= 0) || (len > 128)) {
		CMD_ERR("len must be 4 times and 0~128\n");
		return CMD_STATUS_INVALID_ARG;
	}

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_read_mem(addr, len);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_write_mem_exec(char *cmd)
{
	int ret = 0;
	int32_t cnt;
	uint32_t addr;
	uint32_t value;

	cnt = cmd_sscanf(cmd, "0x%x 0x%x", &addr, &value);
	if (cnt != 2) {
		CMD_ERR("example: 0x10 0x10\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if (addr % 4 != 0) {
		CMD_ERR("addr must be 4 byte align\n");
		return CMD_STATUS_INVALID_ARG;
	}

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_write_mem(addr, value);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_set_channel_fec_exec(char *cmd)
{
	int ret = 0;
	int32_t cnt;
	int32_t pwr_fec_ch0;
	int32_t pwr_fec_ch20;
	int32_t pwr_fec_ch39;

	cnt = cmd_sscanf(cmd, "%d %d %d", &pwr_fec_ch0, &pwr_fec_ch20, &pwr_fec_ch39);
	if (cnt != 3) {
		CMD_ERR("example: -8 12 16\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if ((pwr_fec_ch0 < -128 || pwr_fec_ch0 > 127) ||
	    (pwr_fec_ch20 < -128 || pwr_fec_ch20 > 127) ||
	    (pwr_fec_ch39 < -128 || pwr_fec_ch39 > 127)) {
		CMD_ERR("value must be form -128 to 127\n");
		return CMD_STATUS_INVALID_ARG;
	}

	BLE_ETF_ENABLE_CHECK();
	ret = ble_etf_set_pwr_fec(pwr_fec_ch0, pwr_fec_ch20, pwr_fec_ch39);

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}


static enum cmd_status cmd_bt_etf_help_exec(char *cmd)
{
	printf("%s\n", bt_etf_help);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_ble_etf_help_exec(char *cmd)
{
	printf("%s\n", ble_etf_help);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_ble_etf_connect_exec(char *cmd)
{
	int ret = 0;

	if (OS_SemaphoreCreate(&ble_etf_sem, 1, 1) != OS_OK) {
		CMD_DBG("sem create fail\n");
		ret = -1;
	}

	blec_init();
	etf_blec_hcic_init();
	ble_etf_enable = 1;

	return (ret == 0) ? CMD_STATUS_OK : CMD_STATUS_FAIL;
}

static enum cmd_status cmd_ble_etf_disconnect_exec(char *cmd)
{
	BLE_ETF_ENABLE_CHECK();
	OS_SemaphoreDelete(&ble_etf_sem);
	ble_etf_enable = 0;
	HAL_WDG_Reboot();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_ble_etf_cmds[] = {
	{ "tx",              cmd_ble_etf_tx_exec },
	{ "tx_stop",         cmd_ble_etf_tx_stop_exec },
	{ "rx",              cmd_ble_etf_rx_exec },
	{ "rx_stop",         cmd_ble_etf_rx_stop_exec },
	{ "channel",         cmd_ble_etf_set_chan_exec },
	{ "rate",            cmd_ble_etf_set_rate_exec },
	{ "payload",         cmd_ble_etf_set_payload_exec },
	{ "len",             cmd_ble_etf_set_payload_len_exec },
	{ "hopping",         cmd_ble_etf_hopping_exec },
	{ "hopping_stop",    cmd_ble_etf_hopping_stop_exec },
	{ "power_level",     cmd_ble_etf_set_power_exec },
	{ "power",           cmd_ble_etf_set_power_max_value_exec },
	{ "read_mem",        cmd_ble_etf_read_mem_exec },
	{ "write_mem",       cmd_ble_etf_write_mem_exec },
	{ "set_channel_fec", cmd_ble_etf_set_channel_fec_exec },
	{ "help",            cmd_ble_etf_help_exec },
};

enum cmd_status cmd_ble_etf_exec(char *cmd)
{
	return cmd_exec(cmd, g_ble_etf_cmds, cmd_nitems(g_ble_etf_cmds));
}

static const struct cmd_data g_btetf_cmds[] = {
	{ "ble",             cmd_ble_etf_exec },
	{ "tone",            cmd_ble_etf_tone_exec },
	{ "tone_stop",       cmd_ble_etf_tone_stop_exec },
	{ "connect",         cmd_ble_etf_connect_exec },
	{ "disconnect",      cmd_ble_etf_disconnect_exec },
	{ "help",            cmd_bt_etf_help_exec },
};

enum cmd_status cmd_btetf_exec(char *cmd)
{
	return cmd_exec(cmd, g_btetf_cmds, cmd_nitems(g_btetf_cmds));
}

#endif
