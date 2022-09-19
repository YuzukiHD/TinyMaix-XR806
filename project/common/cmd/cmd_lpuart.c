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

#if (CONFIG_CHIP_ARCH_VER > 2)

#include "string.h"
#include "cmd_util.h"
#include "cmd_lpuart.h"
#include "sys/xr_debug.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_lpuart.h"
#include "pm/pm.h"

LPUART_Data_Width lpuart_data_bits[6] = {
	LPUART_DATA_WIDTH_4, LPUART_DATA_WIDTH_5, LPUART_DATA_WIDTH_6,
	LPUART_DATA_WIDTH_7, LPUART_DATA_WIDTH_8, LPUART_DATA_WIDTH_9
};

typedef struct {
	LPUART_ID        lpuart_id;
	uint32_t         rx_len;
	uint32_t         timeout;
	OS_Queue_t       queue;
	OS_Thread_t      thread;
} cmd_lpuart_priv;
static cmd_lpuart_priv cmd_lpuart_info;

static void rx_cmp_callback(void *arg)
{
	CMD_LOG(1, "wakeup by lpuart !\n");
}

static void cmd_lpuart_transfer_task(void *arg)
{
	cmd_lpuart_priv *priv = &cmd_lpuart_info;
	uint8_t *buf = NULL;
	int32_t cnt;
	uint32_t test_end_flag = 0;

	buf = (uint8_t *)cmd_malloc(512);
	if (buf == NULL) {
		CMD_ERR("no memory\n");
		goto out;
	}

	while (1) {
		if (OS_QueueReceive(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
			CMD_ERR("wait msg fail\n");
			continue;
		}
		if (test_end_flag == 1) {
			break;
		}

		cnt = HAL_LPUART_Receive_IT(priv->lpuart_id, buf, priv->rx_len, priv->timeout);
		if (cnt > 0) {
			CMD_DBG("received %d chars\n", cnt);
			print_hex_dump_bytes(buf, cnt);
		}
	}

out:
	if (buf)
		cmd_free(buf);
	OS_QueueDelete(&priv->queue);
	OS_ThreadDelete(&priv->thread);
}

/*
 * drv lpuart config i=<id> b=<baud-rate> d=<data-bits> p=<parity> m=<msb> s=<in_sel>
 */
static enum cmd_status cmd_lpuart_config_exec(char *cmd)
{
	uint32_t id, baud_rate, data_bits, msb, in_sel;
	char parity[8];
	int cnt;
	LPUART_Parity lpuart_parity;
	LPUART_InitParam lpuart_param;

	cnt = cmd_sscanf(cmd, "i=%u b=%u d=%u p=%7s m=%u s=%u", &id, &baud_rate,
	                 &data_bits, parity, &msb, &in_sel);
	if (cnt != 6) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (id >= LPUART_NUM) {
		CMD_ERR("invalid id %u\n", id);
		return CMD_STATUS_INVALID_ARG;
	}

	if (baud_rate < 300 || baud_rate > 9600) {
		CMD_ERR("invalid baud rate %u\n", baud_rate);
		return CMD_STATUS_INVALID_ARG;
	}

	if (data_bits < 4 || data_bits > 9) {
		CMD_ERR("invalid data bits %u\n", data_bits);
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(parity, "none") == 0) {
		lpuart_parity = LPUART_PARITY_NONE;
	} else if (cmd_strcmp(parity, "odd") == 0) {
		lpuart_parity = LPUART_PARITY_ODD;
	} else if (cmd_strcmp(parity, "even") == 0) {
		lpuart_parity = LPUART_PARITY_EVEN;
	} else {
		CMD_ERR("invalid parity %s\n", parity);
		return CMD_STATUS_INVALID_ARG;
	}

	if (msb > 2) {
		CMD_ERR("invalid msb param %u\n", msb);
		return CMD_STATUS_INVALID_ARG;
	}

	if (in_sel >= UART_NUM) {
		CMD_ERR("invalid input uart select %u\n", in_sel);
		return CMD_STATUS_INVALID_ARG;
	}

	lpuart_param.baudRate = baud_rate;
	lpuart_param.parity = lpuart_parity;
	lpuart_param.msbFirst = msb;
	lpuart_param.dataWidth = lpuart_data_bits[data_bits - 4];
	lpuart_param.input_uart = in_sel;

	if (HAL_LPUART_Init(id, &lpuart_param) != HAL_OK) {
		CMD_DBG("lpuart HW_Init failure !\r\n");
		return CMD_STATUS_FAIL;
	}
	HAL_LPUART_SetBypassPmMode(id, PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY);
	CMD_DBG("lpuart config finish ! \r\n");

	return CMD_STATUS_OK;
}

/*
 * drv lpuart deconfig i=<id>
 */
static enum cmd_status cmd_lpuart_deconfig_exec(char *cmd)
{
	uint32_t id;
	int cnt;

	cnt = cmd_sscanf(cmd, "i=%u", &id);
	if (cnt != 1) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (id >= LPUART_NUM) {
		CMD_ERR("invalid id %u\n", id);
		return CMD_STATUS_INVALID_ARG;
	}

	if (HAL_LPUART_DeInit(id) != HAL_OK) {
		CMD_DBG("lpuart HW_DeInit failure !\r\n");
		return CMD_STATUS_FAIL;
	}
	CMD_DBG("lpuart deconfig finish ! \r\n");

	return CMD_STATUS_OK;
}

/*
 * drv lpuart transfer-data i=<id> l=<data-length> t=<rx-timeout>
 */
static enum cmd_status cmd_lpuart_transfer_data_exec(char *cmd)
{
	int32_t cnt;
	uint32_t id, len, timeout, test_end_flag = 0;
	cmd_lpuart_priv *priv = &cmd_lpuart_info;

	cnt = cmd_sscanf(cmd, "i=%u l=%u t=%u", &id, &len, &timeout);
	if (cnt != 3) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (id >= LPUART_NUM) {
		CMD_ERR("invalid id %u\n", id);
		return CMD_STATUS_INVALID_ARG;
	}

	if (len == 0 || len > 512) {
		CMD_ERR("invalid data length %u\n", len);
		return CMD_STATUS_INVALID_ARG;
	}

	if (timeout == 0) {
		timeout = HAL_WAIT_FOREVER;
	}
	priv->lpuart_id = id;
	priv->rx_len = len;
	priv->timeout = timeout;

	if (OS_ThreadIsValid(&priv->thread) && OS_QueueIsValid(&priv->queue)) {
		if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
			return CMD_STATUS_FAIL;
		}
		return CMD_STATUS_OK;
	}
	if (OS_QueueCreate(&priv->queue,
	                   1,
	                   sizeof(uint32_t)) != OS_OK) {
		CMD_ERR("create queue failed\n");
		return CMD_STATUS_FAIL;
	}
	if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK) {
	   return CMD_STATUS_FAIL;
	}
	if (OS_ThreadCreate(&priv->thread,
	                    "lpuart-test",
	                    cmd_lpuart_transfer_task,
	                    NULL,
	                    OS_PRIORITY_BELOW_NORMAL,
	                    1024) != OS_OK) {
	   CMD_LOG(1, "create Task[lpuart-test] failed\n");
	   return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

/*
 * drv lpuart wakeup-config i=<id> l=<data-length> cn(n=1~5)=<compare-data_n>
 */
static enum cmd_status cmd_lpuart_wakeup_config_exec(char *cmd)
{
	int32_t  cnt;
	uint32_t id, len;
	uint8_t  buf[5];

	cnt = cmd_sscanf(cmd, "i=%u l=%u c1=%u c2=%u c3=%u c4=%u c5=%u", &id, &len, \
	                 (uint32_t *)&buf[0], (uint32_t *)&buf[1],
	                 (uint32_t *)&buf[2], (uint32_t *)&buf[3],
	                 (uint32_t *)&buf[4]);
	if (cnt != (len + 2)) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (id >= LPUART_NUM) {
		CMD_ERR("invalid id %u\n", id);
		return CMD_STATUS_INVALID_ARG;
	}

	if (len == 0 || len > LPUART_RX_CMP_DATA_NUM_MAX) {
		CMD_ERR("invalid compare data length %u\n", len);
		return CMD_STATUS_INVALID_ARG;
	}

	CMD_DBG("cmpare data : %d %d %d %d %d\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
	HAL_LPUART_EnableRxCmp(id, len, buf);

	HAL_LPUART_EnableRxCmpCallback(id, rx_cmp_callback, NULL);

	//pm_enter_mode(PM_MODE_STANDBY);

	return CMD_STATUS_OK;
}

/*
 * drv lpuart transfer-stop i=<id>
 */
static enum cmd_status cmd_lpuart_transfer_stop_exec(char *cmd)
{
	uint32_t id;
	int cnt;
	cmd_lpuart_priv *priv = &cmd_lpuart_info;
	uint32_t test_end_flag = 1;

	cnt = cmd_sscanf(cmd, "i=%u", &id);
	if (cnt != 1) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (id >= LPUART_NUM) {
		CMD_ERR("invalid id %u\n", id);
		return CMD_STATUS_INVALID_ARG;
	}

	HAL_LPUART_DeInit(id);

	if (OS_QueueIsValid(&priv->queue)) {
		if (OS_QueueSend(&priv->queue, &test_end_flag, OS_WAIT_FOREVER) != OS_OK)
			return CMD_STATUS_FAIL;
	}

	while (OS_ThreadIsValid(&priv->thread)) {
		OS_MSleep(1);
	}

	return CMD_STATUS_OK;
}

static struct cmd_data g_lpuart_cmds[] = {
	{ "config",             cmd_lpuart_config_exec },
	{ "deconfig",           cmd_lpuart_deconfig_exec },
	{ "transfer-data",      cmd_lpuart_transfer_data_exec },
	{ "transfer-stop",      cmd_lpuart_transfer_stop_exec },
	{ "wakeup-config",      cmd_lpuart_wakeup_config_exec },
};

enum cmd_status cmd_lpuart_exec(char *cmd)
{
	return cmd_exec(cmd, g_lpuart_cmds, cmd_nitems(g_lpuart_cmds));
}
#endif
