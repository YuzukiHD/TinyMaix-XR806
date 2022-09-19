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
#include <string.h>

#include "kernel/os/os.h"
#include "common/framework/platform_init.h"
#include "smartlink/blink/blink.h"
#include "ble/bluetooth/conn.h"

#include "smartlink/sc_assistant.h"
#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "common/framework/bt_ctrl.h"
#endif
#endif

#define BLINK_TIMEOUT_MS    300000
#define BLINK_PRINT_EN        0

struct netif *blink_wlan_netif = NULL;

static int blink_sc_assistant_init(void)
{
	int ret;
	sc_assistant_fun_t sca_fun;
	sc_assistant_time_config_t config;

	printf("try to init sc_assistant.\n");
	sc_assistant_get_fun(&sca_fun);
	config.time_total = 120000;
	config.time_sw_ch_long = 0;
	config.time_sw_ch_short = 0;
	ret = sc_assistant_init(blink_wlan_netif, &sca_fun, &config);
	if (ret) {
		printf("sc_assistant_init fail.\n");
		return -1;
	}

	return 0;
}

static void blink_sc_assistant_deinit(void)
{
	sc_assistant_deinit(blink_wlan_netif);
}

static void bt_ready(int err)
{
	blink_param_t param;
	memset(&param, 0, sizeof(blink_param_t));
	blink_start(&param);
}

static void connected(struct bt_conn *conn, u8_t err)
{
	bt_addr_le_t rpa = {0};
	int result;
	struct bt_conn_info info;
	if (err) {
		printf("[H] Connectionfailed reason %u\n", err);
	} else {
		result = bt_conn_get_info(conn, &info);
		if (result) {
			printf("Failed to get info\n");
			return ;
		}

		memcpy(&rpa, &info.le.src, sizeof(bt_addr_le_t));
		printf("[H] Connected!! \n");
		printf("========== Connection Parameter ==========\n");
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printf("[H] Disconnected\n");
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void blink_bt_app_init(void)
{
	int err;
	printf("[%s]\n", __func__);

	bt_ctrl_enable();

	err = bt_enable(bt_ready);
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return;
	}

	//optional
	bt_conn_cb_register(&conn_callbacks);
}

int main(void)
{
	platform_init();

	printf("blink example start...\n");

	blink_ret_t ret;
	blink_result_t result;

	blink_sc_assistant_init();

	blink_bt_app_init();

	OS_MSleep(200);

	if (blink_wait(BLINK_TIMEOUT_MS) != BLINK_OK) {
		printf("time out\n");
		return -1;
	}

	ret = blink_get_result(&result);
	if (ret != BLINK_OK) {
		printf("get result fail\n");
		return -1;
	}

#if BLINK_PRINT_EN
	for (uint32_t i = 0; i < result.ssid_len; i++) {
		printf("%c", result.ssid[i]);
	}
	printf("\n<---------------SSID\n");
#endif

#if BLINK_PRINT_EN
	for (int i = 0; i < result.passphrase_len; i++) {
		printf("%c", result.passphrase[i]);
	}
	printf("\n<---------------Passphrase\n");
#endif

	blink_wlan_netif = sc_assistant_open_sta();
	if (sc_assistant_connect_ap(result.ssid, result.ssid_len, result.passphrase, 120000) < 0) {
		printf("connect ap time out\n");
		blink_set_state(BLINK_STATE_FAIL);
	} else {
		printf("connect ap success\n");
		blink_set_state(BLINK_STATE_SUCCESS);
	}

	OS_MSleep(500);

	blink_stop();

	blink_sc_assistant_deinit();

	return 0;
}

