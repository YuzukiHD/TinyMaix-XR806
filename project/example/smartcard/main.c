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




/************************demo********************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sys/xr_debug.h"

#include "kernel/os/os.h"

#include "common/framework/platform_init.h"

#include "driver/chip/hal_rtc.h"
#include "driver/chip/hal_scr.h"
#include "../src/driver/chip/hal_base.h"

#define THREAD_STACK_SIZE_SmartCard        (3 * 1024)

static OS_Thread_t threadSmartCard;

static void scr_irq_cb(void *arg)
{

}

static int32_t scr_apdu_test(uint32_t num)
{
	SCR_WRData apdu_cmd;
	uint8_t *dataSend, *rx_buf;
	int time_us = 200, test_num;
	uint64_t start_time = HAL_RTC_GetFreeRunTime();

	dataSend = (uint8_t *)malloc(128);
	if (dataSend == NULL) {
		printf("no heap space !\n");
	}
	rx_buf = (uint8_t *)malloc(128);
	if (rx_buf == NULL) {
		printf("no heap space !\n");
	}

	for (test_num = 0; test_num < num; test_num++) {
		SCR_UDelay(time_us);
		printf("The %d times test ->\n", test_num + 1);
		memset((void *)&apdu_cmd, 0, sizeof(apdu_cmd));
		dataSend[0] = 0xe5;
		dataSend[1] = 0x04;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x04;
		apdu_cmd.cmd_buf = dataSend;
		apdu_cmd.cmd_len = 5;
		apdu_cmd.rtn_data = rx_buf;
		HAL_SCR_APDUCmd(&apdu_cmd);
		printf("SW1 SW2 : %02x %02x\n", apdu_cmd.psw1, apdu_cmd.psw2);
		printf("data length response from Card: %d\n", apdu_cmd.rtn_len);
		print_hex_dump_bytes(rx_buf, apdu_cmd.rtn_len);
		printf("\n");

		// clear authorization
	#if 0
		int rst_len = 0;
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x2;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		printf("HAL_SCR_ReceiveByIT  = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x3f;
		dataSend[1] = 0x0;
		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		print_hex_dump_bytes(rx_buf, rst_len);
	#endif

	#if 0
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xb2;
		dataSend[2] = 0x4;
		dataSend[3] = 0x4;
		dataSend[4] = 0x8;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		printf("HAL_SCR_ReceiveByIT  = %d\n", rst_len);
		if ((11 != rst_len) || (0xb2 != rx_buf[0]) || (0x90 != rx_buf[rst_len-2]) || (0x0 != rx_buf[rst_len-1])) {
			printf("not 11 bytes or invalid value in the test_num = %d!!\n", test_num);
			printf("rx_buf[0]=0x%02x\n", rx_buf[0]);
			printf("rx_buf[rst_len-2]=0x%02x\n", rx_buf[rst_len-2]);
			printf("rx_buf[rst_len-1]=0x%02x\n", rx_buf[rst_len-1]);
			return -1;
		}
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x2;
		dataSend[3] = 0x0;
		dataSend[4] = 0x2;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		printf("HAL_SCR_ReceiveByIT  = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x10;
		dataSend[1] = 0x8;
		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		printf("HAL_SCR_ReceiveByIT  = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xb0;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x40;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll  = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 128);
		printf("HAL_SCR_ReceiveByIT  = %d\n", rst_len);
		if ((67 != rst_len) || (0xb0 != rx_buf[0]) || (0x90 != rx_buf[rst_len-2]) || (0x0 != rx_buf[rst_len-1])) {
			printf("not 67 bytes or invalid value in the test_num = %d!!\n", test_num);
			printf("rx_buf[0]=0x%02x\n", rx_buf[0]);
			printf("rx_buf[rst_len-2]=0x%02x\n", rx_buf[rst_len-2]);
			printf("rx_buf[rst_len-1]=0x%02x\n", rx_buf[rst_len-1]);
			return -1;
		}
		print_hex_dump_bytes(rx_buf, rst_len);
	#endif

	#if 0
		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x2;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x3f;
		dataSend[1] = 0x0;
		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x2;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x10;
		dataSend[1] = 0x20;
		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x80;
		dataSend[1] = 0xf8;
		dataSend[2] = 0x0;
		dataSend[3] = 0x3;
		dataSend[4] = 0x18;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x10;
		dataSend[1] = 0x3;
		dataSend[2] = 0x1;
		dataSend[3] = 0x1;
		dataSend[4] = 0xa;
		dataSend[5] = 0xf;
		dataSend[6] = 0x1;
		dataSend[7] = 0xf;
		dataSend[8] = 0x88;
		dataSend[9] = 0xe5;
		dataSend[10] = 0x9f;
		dataSend[11] = 0xf1;
		dataSend[12] = 0x43;
		dataSend[13] = 0x14;
		dataSend[14] = 0x46;
		dataSend[15] = 0xf7;
		dataSend[16] = 0x56;
		dataSend[17] = 0x80;
		HAL_Memset((void *)&dataSend[18], 0, 6);

		rst_len = HAL_SCR_SendByPoll(dataSend, 24);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);
		//OS_Sleep(2);

		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xc0;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x18;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("HAL_SCR_SendByPoll rst_len = %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("HAL_SCR_ReceiveByIT rst_len = %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);
	#endif

	#if 0
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x02;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write1: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read1: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x3f;
		dataSend[1] = 0x0;

		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("write2: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read2: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x02;
		dataSend[3] = 0x0;
		dataSend[4] = 0x02;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write3: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read3: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x10;
		dataSend[1] = 0x02;

		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("write4: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read4: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xdc;
		dataSend[2] = 0x01;
		dataSend[3] = 0x04;
		dataSend[4] = 0x08;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write5: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read5: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x8;
		HAL_Memset((void *)&dataSend[1], 0, 7);
		rst_len = HAL_SCR_SendByPoll(dataSend, 8);
		printf("write6: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read6: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************
		SCR_UDelay(time_us);
		dataSend[0] = 0x0;
		dataSend[1] = 0xdc;
		dataSend[2] = 0x02;
		dataSend[3] = 0x04;
		dataSend[4] = 0x08;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write7: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read7: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		HAL_Memset((void *)&dataSend[0], 0, 8);
		rst_len = HAL_SCR_SendByPoll(dataSend, 8);
		printf("write8: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read8: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************

		dataSend[0] = 0x0;
		dataSend[1] = 0xdc;
		dataSend[2] = 0x03;
		dataSend[3] = 0x04;
		dataSend[4] = 0x08;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write9: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read9: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		HAL_Memset((void *)&dataSend[0], 0, 8);
		rst_len = HAL_SCR_SendByPoll(dataSend, 8);
		printf("write10: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read10: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************

		dataSend[0] = 0x0;
		dataSend[1] = 0xdc;
		dataSend[2] = 0x04;
		dataSend[3] = 0x04;
		dataSend[4] = 0x08;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write11: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read11: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		HAL_Memset((void *)&dataSend[0], 0, 8);
		rst_len = HAL_SCR_SendByPoll(dataSend, 8);
		printf("write12: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read12: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************

		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x02;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write13: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read13: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x3f;
		dataSend[1] = 0x0;

		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("write14: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read14: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************

		dataSend[0] = 0x0;
		dataSend[1] = 0xa4;
		dataSend[2] = 0x02;
		dataSend[3] = 0x0;
		dataSend[4] = 0x02;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write15: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read15: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		SCR_UDelay(time_us);
		dataSend[0] = 0x10;
		dataSend[1] = 0x8;

		rst_len = HAL_SCR_SendByPoll(dataSend, 2);
		printf("write16: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read16: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

	//***********************************************************

		dataSend[0] = 0x0;
		dataSend[1] = 0xd6;
		dataSend[2] = 0x0;
		dataSend[3] = 0x0;
		dataSend[4] = 0x40;
		rst_len = HAL_SCR_SendByPoll(dataSend, 5);
		printf("write17: %d\n", rst_len);
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read17: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);

		dataSend[0] = 0x80;
		HAL_Memset((void *)&dataSend[1], 0, 63);
		rst_len = HAL_SCR_SendByPoll(dataSend, 64);

		printf("write18: %d\n", rst_len);     //write 18 bytes only, unread later.
		SCR_UDelay(time_us);
		rst_len = HAL_SCR_ReceiveByIT(rx_buf, 64);
		printf("read18: %d\n", rst_len);
		print_hex_dump_bytes(rx_buf, rst_len);
	#endif
	}

	printf("time-consuming: %dms\n", (uint32_t)((HAL_RTC_GetFreeRunTime() - start_time) / 1000));

	free(dataSend);
	free(rx_buf);
	return 0;
}

static void SCR_RWTest_task(void *arg)
{
	SCR_InitParam g_scr_config;

	g_scr_config.mclk = 2000000;
	g_scr_config.fifo_thr = SCR_FIFO_DEPTH / 4;    //set the txfifo trigger level as half full
	g_scr_config.repeat_num = 3;                //The maximum number of reposts is 3
	g_scr_config.act_time = 1;                    //1*256 clock cycle
	g_scr_config.rst_time = 1;                    //1*256 clock cycle
	g_scr_config.atr_time = (35000>>8);            //400~40000, After RST is pulled up,IO should make the time limit of ATR
	g_scr_config.guard_time = 2;                 /* 2*ETUs, GT = 12etu + R * N/f, Default extra guard time is 0 */
	g_scr_config.char_limit = 9600;                //1024 * (10 + g_scr_config.guard_time);

	g_scr_config.IRQCallback = scr_irq_cb;
	g_scr_config.arg = NULL;

	HAL_SCR_Init(&g_scr_config);

	while (1) {
		if (HAL_SCR_Process(&g_scr_config) == STS_RW_TEST) {
			break;
		}
	}

	printf("begin smartcard HAL_SCR_ReceiveByIT HAL_SCR_SendByPoll test flow -> \n");

	if (!scr_apdu_test(100)) {
		printf("smartcard test 100 times successful !\n");
	} else{
		printf("smartcard test failure !\n");
	}

	OS_ThreadDelete(&threadSmartCard);
}

int main(void)
{
	platform_init();

	if (OS_ThreadCreate(&threadSmartCard,
	                    "SMARTCARD",
	                    SCR_RWTest_task,
	                    NULL,
	                    OS_THREAD_PRIO_DRV_BH,
	                    THREAD_STACK_SIZE_SmartCard) != OS_OK) {
		printf("create Task[SMARTCARD] failed\n");
		return -1;
	}

	return 0;
}
