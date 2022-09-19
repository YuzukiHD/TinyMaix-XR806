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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kernel/os/os.h"
#include "sys/xr_debug.h"
#include "driver/chip/hal_lpuart.h"
#include "pm/pm.h"
#include "common/framework/platform_init.h"

#define LPUART_TRANSFER_TEST       1
#define LPUART_COMPARE_TEST        1
#define LPUART_WAKEUP_TEST         1

#define LPUARTID LPUART1_ID

static int lpuart_init(void)
{
	LPUART_InitParam lpuart_param;

	/* PC com needs to set stop-bit to 2 if use baudRate 9600 */
	lpuart_param.baudRate = 9600;
	lpuart_param.parity = LPUART_PARITY_NONE;
	lpuart_param.msbFirst = 1;
	lpuart_param.dataWidth = LPUART_DATA_WIDTH_8;
	lpuart_param.input_uart = PRCM_LPUART_WAKEUP_IN_USE_UART1;

	if (HAL_LPUART_Init(LPUARTID, &lpuart_param) != HAL_OK) {
		printf("lpuart HW_Init failure !\n");
		return -1;
	}
	HAL_LPUART_SetBypassPmMode(LPUARTID, PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY);
	printf("lpuart config finish !\n");

	return 0;
}

static void lpuart_deinit(void)
{
	HAL_LPUART_DeInit(LPUARTID);
}

static void lpuart_disable_cb(void)
{
	HAL_LPUART_DisableRxCmpCallback(LPUARTID);
}


#if LPUART_TRANSFER_TEST
#define LPUART_TRANSFER_DATA_LEN    10
typedef struct {
	LPUART_ID        lpuart_id;
	uint32_t         rx_len;
	uint32_t         timeout;
	OS_Queue_t       queue;
	OS_Thread_t      thread;
} lpuart_priv;
static lpuart_priv g_lpuart_info;

static void lpuart_transfer_task(void *arg)
{
	lpuart_priv *priv = &g_lpuart_info;
	uint8_t *buf = NULL;
	int32_t cnt;
	uint32_t test_end_flag = 0;

	buf = (uint8_t *)malloc(512);
	if (buf == NULL) {
		printf("no memory\n");
		goto out;
	}

	while (1) {
		if (OS_QueueReceive(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
			printf("wait msg fail\n");
			continue;
		}
		if (test_end_flag == 1) {
			break;
		}

		cnt = HAL_LPUART_Receive_IT(priv->lpuart_id, buf, priv->rx_len, priv->timeout);
		if (cnt > 0) {
			printf("received %d chars\n", cnt);
			print_hex_dump_bytes(buf, cnt);
		}
	}

out:
	if (buf)
		free(buf);
	OS_QueueDelete(&priv->queue);
	OS_ThreadDelete(&priv->thread);
}

static int lpuart_transfer_data(void)
{
	uint32_t test_end_flag = 0;
	lpuart_priv *priv = &g_lpuart_info;

	priv->lpuart_id = LPUARTID;
	priv->rx_len = LPUART_TRANSFER_DATA_LEN;
	priv->timeout = HAL_WAIT_FOREVER;

	if (OS_ThreadIsValid(&priv->thread) && OS_QueueIsValid(&priv->queue)) {
		if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
			return -1;
		}
		return 0;
	}
	if (OS_QueueCreate(&priv->queue,
		               1,
		               sizeof(uint32_t)) != OS_OK) {
		printf("create queue failed\n");
		return -1;
	}
	if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
	   return -1;
	}
	if (OS_ThreadCreate(&priv->thread,
	                   "lpuart-test",
	                   lpuart_transfer_task,
	                   NULL,
	                   OS_PRIORITY_BELOW_NORMAL,
	                   1024) != OS_OK) {
	   printf("create Task[lpuart-test] failed\n");
	   return -1;
	}

	return 0;
}


static int lpuart_transfer_stop(void)
{
	lpuart_priv *priv = &g_lpuart_info;
	uint32_t test_end_flag = 1;

	if (OS_QueueIsValid(&priv->queue)) {
		if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK)
			return -1;
	}

	while (OS_ThreadIsValid(&priv->thread)) {
		OS_MSleep(1);
	}

	return 0;
}
#endif

#if LPUART_COMPARE_TEST
#define CMP_DATA_NUM        5

static void compare_callback(void *arg)
{
	printf("data compare success!\n");
}

static void lpuart_compare_config(void)
{
	uint8_t  cmp_buf[CMP_DATA_NUM] = {'1', '2', '3', '4', '5'};
	printf("compare data : %c %c %c %c %c\n",
			cmp_buf[0], cmp_buf[1], cmp_buf[2], cmp_buf[3], cmp_buf[4]);
	HAL_LPUART_EnableRxCmp(LPUARTID, LPUART_RX_CMP_DATA_NUM_5, cmp_buf);
	HAL_LPUART_EnableRxCmpCallback(LPUARTID, compare_callback, NULL);
}
#endif

#if LPUART_WAKEUP_TEST
#define WAKEUP_DATA_NUM        5

static void wakeup_callback(void *arg)
{
	printf("wakeup by lpuart!\n");
}

static void lpuart_wakeup_config(void)
{
	uint8_t  wakeup_buf[WAKEUP_DATA_NUM] = {'6', '7', '8', '9', '0'};
	printf("cmpare data : %c %c %c %c %c\n",
	      wakeup_buf[0], wakeup_buf[1], wakeup_buf[2], wakeup_buf[3], wakeup_buf[4]);
	HAL_LPUART_EnableRxCmp(LPUARTID, LPUART_RX_CMP_DATA_NUM_5, wakeup_buf);
	HAL_LPUART_EnableRxCmpCallback(LPUARTID, wakeup_callback, NULL);
	pm_enter_mode(PM_MODE_STANDBY);
}
#endif

/* Run this example, please connect the uart0 and uart1 */
int main(void)
{
	platform_init();

	printf("lpuart example started.\n\n");
	lpuart_init();
	printf("please connect to uart%d serial com.\n", LPUARTID);

#if LPUART_TRANSFER_TEST
	OS_Sleep(1);
	lpuart_transfer_data();
	printf("\ntransfer test start!\nplease enter %d byte data to uart%d in 10 seconds.\n",
	        LPUART_TRANSFER_DATA_LEN, LPUARTID);
	OS_Sleep(10);
	lpuart_transfer_stop();
	printf("transfer test stop.\n");
#endif

#if LPUART_COMPARE_TEST
	OS_Sleep(1);
	printf("\ncompare test start!\nplease enter %d byte compare data to uart%d in 10 seconds.\n",
	         CMP_DATA_NUM, LPUARTID);
	lpuart_compare_config();
	OS_Sleep(10);
	printf("compare test stop.\n");
#endif

#if LPUART_WAKEUP_TEST
	OS_Sleep(1);
	printf("\nwakeup test start!\nplease enter %d byte wakeup data to uart%d.\n",
	         WAKEUP_DATA_NUM, LPUARTID);
	lpuart_wakeup_config();
	printf("wakeup test stop.\n");
#endif

	OS_Sleep(1);
#if (LPUART_COMPARE_TEST || LPUART_WAKEUP_TEST)
	lpuart_disable_cb();
#endif
	lpuart_deinit();
	printf("\nlpuart example over.\n");

	return 0;
}
