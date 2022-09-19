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
/*
*******************************************************************************
*
* @file    central.c
* @brief   This is the example for BLE central.
* @steps   1)ble init;
*          2)scan peripheral device;
*          3)connect with special peripheral device;
*          4)bonding with connected peripheral device;
*          5)start timer and subscribe peripheral gatt service when timeout
*          6)write peripheral gatt service periodically
******************************************************************************
*/
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <ble/sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "common/framework/bt_ctrl.h"
#endif
#endif

/***************Internal Function**************************/
static void start_scan(void);
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
            struct net_buf_simple *ad);
static struct bt_conn *find_conn_pointer(struct bt_conn *conn);
static void conn_count_update(bool b);
static void conn_clean(struct bt_conn *conn);
static void disconnected(struct bt_conn *conn, uint8_t reason);

/***************Global Variables*******************************/
#define NAME_LEN    30
/*connection mamager*/
#define CONN_MAX CONFIG_BT_MAX_CONN
static int conn_count = 0;
static struct bt_conn *default_conn[CONN_MAX] = {0};
static bt_security_t security_level;
static OS_Timer_t central_timer;
static OS_Timer_t subscribe_timer;

#define DISCOVER_SERVICE_CTS        1
#define DISCOVER_SERVICE_HRS        2
#define DISCOVER_SERVICE_BAS        3
#define DISCOVER_SERVICE_VND        4
#define DISCOVER_SERVICE_VND_WRITE  5
uint8_t discover_service;
#define CENTRAL_PERIOD           1000   //1000ms
#define SUBSCRIBE_PERIOD         2000   //2000ms
#define CTS_INVAL                120
#define HRS_INVAL                30
#define BAS_INVAL                60
#define VND_INVAL                1
#define VND_WRITE_INVAL          10
#define VND_WRITE_LONG_INVAL     13
uint8_t notify_enable_flag[5] = {0};

/***************Peripheral Custom Service Variables************************/
/* Peripheral Custom Service UUID */
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_notify_uuid = BT_UUID_INIT_128(
	0xf6, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/***************Central Custom Service Variables************************/
/* Central Custom Service UUID */
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_uuid_128 uuid_128 = BT_UUID_INIT_128(0);

/* Central Custom Service Variables */
struct bt_subscrible_params {
	struct bt_conn *conn;
	struct bt_uuid_16 uuid;
	struct bt_gatt_subscribe_params sub_param;
};
static struct bt_gatt_discover_params discover_params;
static struct bt_subscrible_params subscribe_param[CONN_MAX];
static struct bt_gatt_discover_params discover_write_params;
static struct bt_gatt_write_params write_params[CONN_MAX];
static uint8_t write_data[CONN_MAX] = {0};

/**********************Custom Service Function********************/
/***********************Notify Function*******************/
static uint8_t cts_notify_func(struct bt_conn *conn,
                struct bt_gatt_subscribe_params *params,
                const void *data, uint16_t length)
{
	uint8_t ct[length];
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(ct, data, length);

	printf("[NOTIFICATION](%s,CT %d%d-%d-%d %d:%d:%d %d %d %d)\n", addr, ct[0], ct[1], ct[2],
			ct[3], ct[4], ct[5], ct[6], ct[7], ct[8], ct[9]);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t hrs_notify_func(struct bt_conn *conn,
                struct bt_gatt_subscribe_params *params,
                const void *data, uint16_t length)
{
	uint8_t hrm[length];
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(hrm, data, length);

	if (length == 1) {
		printf("[NOTIFICATION](%s, HRS: %x)\n", addr, (uint8_t)hrm[0]);
	} else {
		printf("[NOTIFICATION](%s, HRS: %s)\n", addr, hrm);
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t bas_notify_func(struct bt_conn *conn,
                struct bt_gatt_subscribe_params *params,
                const void *data, uint16_t length)
{
	uint8_t bas;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(&bas, data, length);

	printf("[NOTIFICATION](%s, BAS: %x)\n", addr, bas);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t vnd_notify_func(struct bt_conn *conn,
                struct bt_gatt_subscribe_params *params,
                const void *data, uint16_t length)
{
	char vnd[length];
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(vnd, data, length);

	if (length == 1) {
		printf("[NOTIFICATION](%s, VND: %x)\n", addr, (uint8_t)vnd[0]);
	} else {
		printf("[NOTIFICATION](%s, VND: %s)\n", addr, vnd);
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t vnd_write_ind_func(struct bt_conn *conn,
                struct bt_gatt_subscribe_params *params,
                const void *data, uint16_t length)
{
	char vnd[length];
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(vnd, data, length);

	if (length == 1) {
		printf("[INDICATION](%s, VND: %x)\n", addr, (uint8_t)vnd[0]);
	} else {
		printf("[INDICATION](%s, VND: %s)\n", addr, vnd);
	}

	return BT_GATT_ITER_CONTINUE;
}

/***********************Discover Function*******************/
static int subscribe_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                int index, struct bt_uuid *primary_uuid, struct bt_uuid *charc_uuid,
                bt_gatt_notify_func_t notify_func, uint16_t sub_value, char *sub_string)
{
	int err;
	int i;

	if (!bt_uuid_cmp(discover_params.uuid, primary_uuid)) {
		if (charc_uuid->type == BT_UUID_TYPE_128) {
			memcpy(&uuid_128, charc_uuid, sizeof(uuid_128));
			discover_params.uuid = &uuid_128.uuid;
		} else {
			memcpy(&uuid, charc_uuid, sizeof(uuid));
			discover_params.uuid = &uuid.uuid;
		}
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printf("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, charc_uuid)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_param[index].sub_param.value_handle = attr->handle + 1;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printf("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_param[index].sub_param.notify = notify_func;
		subscribe_param[index].sub_param.value = sub_value;
		subscribe_param[index].sub_param.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_param[index].sub_param);
		if (err && err != -EALREADY) {
			printf("Subscribe failed (err %d)\n", err);
		} else {
			subscribe_param[index].conn = conn;
			printf("[%s SUBSCRIBED]\n", sub_string);
			discover_service = 0;
		}
	}

	if (err) {
		for (i = 0; i < CONN_MAX; i++) {
			if (conn == default_conn[i])
				break;
		}
		if (i == CONN_MAX) {
			printf("Connection not be found\n");
			return BT_GATT_ITER_STOP;
		}
		notify_enable_flag[discover_service - 1] &= ~(1 << i);
		discover_service = 0;
	}
	return BT_GATT_ITER_STOP;
}

static uint8_t bt_get_subscribe_params_index_find(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(subscribe_param); i++) {
		if (!subscribe_param[i].conn) {
			break;
		}
	}
	return i;
}

static uint8_t discover_func(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                struct bt_gatt_discover_params *params)
{
	int i;
	uint8_t status = BT_GATT_ITER_STOP;

	for (i = 0; i < CONN_MAX; i++) {
		if (conn == default_conn[i])
			break;
	}
	if (i == CONN_MAX) {
		printf("Connection not be found\n");
		return status;
	}

	if (!attr) {
		printf("Discover complete\n");
		memset(params, 0, sizeof(*params));
		notify_enable_flag[discover_service - 1] &= ~(1 << i);
		discover_service = 0;
		return BT_GATT_ITER_STOP;
	}

	printf("[ATTRIBUTE] handle %u\n", attr->handle);

	i = bt_get_subscribe_params_index_find();
	if (i == CONN_MAX) {
		printf("No enough subscribe number\n");
		notify_enable_flag[discover_service - 1] &= ~(1 << i);
		discover_service = 0;
		return status;
	}

	switch (discover_service) {
	case DISCOVER_SERVICE_CTS:
		status = subscribe_discover_func(conn, attr, i, BT_UUID_CTS, BT_UUID_CTS_CURRENT_TIME,
		                                    cts_notify_func, BT_GATT_CCC_NOTIFY, "CTS");
		break;
	case DISCOVER_SERVICE_HRS:
		status = subscribe_discover_func(conn, attr, i, BT_UUID_HRS, BT_UUID_HRS_MEASUREMENT,
		                                    hrs_notify_func, BT_GATT_CCC_NOTIFY, "HRS");
		break;
	case DISCOVER_SERVICE_BAS:
		status = subscribe_discover_func(conn, attr, i, BT_UUID_BAS, BT_UUID_BAS_BATTERY_LEVEL,
		                                    bas_notify_func, BT_GATT_CCC_NOTIFY, "BAS");
		break;
	case DISCOVER_SERVICE_VND:
		status = subscribe_discover_func(conn, attr, i, &vnd_uuid.uuid, &vnd_notify_uuid.uuid,
		                                    vnd_notify_func, BT_GATT_CCC_NOTIFY, "VND");
		break;
	case DISCOVER_SERVICE_VND_WRITE:
		status = subscribe_discover_func(conn, attr, i, &vnd_uuid.uuid, &vnd_enc_uuid.uuid,
		                                    vnd_write_ind_func, BT_GATT_CCC_INDICATE, "VND WRITE");
		break;
	default:
		break;
	}

	return status;
}

/***********************VND Realted Write*******************/
static void write_func(struct bt_conn *conn, uint8_t err,
                     struct bt_gatt_write_params *params)
{
	int i;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printf("Write 0x%x to %s SUCCESS\n", *(uint8_t *)params->data, addr);

		for (i = 0; i < CONN_MAX; i++) {
			if (conn == default_conn[i])
				break;
		}

		if (write_data[i] == 255) {
			write_data[i] = write_data[i] % 255;
		} else {
			write_data[i] += 1;
		}
	} else {
		printf("Write to %s FAIL %d\n", addr, err);
	}
}

static uint8_t discover_write_func(struct bt_conn *conn,
                 const struct bt_gatt_attr *attr,
                 struct bt_gatt_discover_params *params)
{
	int i;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!attr) {
		printf("Discover write %s complete\n", addr);
		memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printf("[ATTRIBUTE] %s write handle %u\n", addr, attr->handle);

	for (i = 0; i < CONN_MAX; i++) {
		if (conn == default_conn[i])
			break;
	}

	write_params[i].handle = attr->handle;
	write_params[i].func = write_func;
	write_params[i].data = &write_data[i];
	write_params[i].length = 1;
	return BT_GATT_ITER_CONTINUE;
}

/****************Connection*************************/
static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn_p = find_conn_pointer(conn);
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printf("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(conn_p);

		start_scan();
		return;
	}

	printf("Connected: %s\n", addr);
	conn_count_update(true);

	err = bt_conn_set_security(conn_p, security_level);
	if (err) {
		printf("Failed to set security %d\n", err);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
	struct bt_conn *conn_p = find_conn_pointer(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Disconnected: %s (reason %u)\n", addr, reason);

	if (conn_p != conn) {
		return;
	}

	bt_conn_unref(conn_p);
	conn_clean(conn_p);

	if (conn_count == 0) {
		/* This demo doesn't require active scan */
		err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
		if (err) {
			printf("Scanning failed to start (err %d)\n", err);
		}
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

/********************Auth Method***********************/
enum AUTH_CB_METHOD_TYPES {
	AUTH_CB_METHOD_NONE,
	AUTH_CB_METHOD_CONFIRM,
	AUTH_CB_METHOD_DISPLAY_YES_NO,
	AUTH_CB_METHOD_DISPLAY,
	AUTH_CB_METHOD_INPUT,
	AUTH_CB_METHOD_OOB,
	AUTH_CB_METHOD_ALL,
};
#define BT_AUTH_CB_METHOD   AUTH_CB_METHOD_CONFIRM

#define BT_LE_PASSKEY_STR_LEN   7

static unsigned int fixed_passkey = 123456;

/*******************Auth Method Function*******************/
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	printf("  [Passkey for %s: %06u]\n", addr, passkey);
	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n");
}

static void auth_passkey_entry(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn_p = find_conn_pointer(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Enter passkey for %s\n", addr);

	if (!conn_p) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_passkey_entry(conn_p, fixed_passkey);
		printf("Enter Passkey Success\n");
	}
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	char passkey_str[BT_LE_PASSKEY_STR_LEN];
	struct bt_conn *conn_p = find_conn_pointer(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	snprintf(passkey_str, 7, "%06u", passkey);

	printf("Confirm passkey for %s: %s\n", addr, passkey_str);

	if (!conn_p) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_passkey_confirm(conn_p);
		printf("Passkey Confirm Success\n");
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Pairing cancelled: %s\n", addr);
}

static void auth_pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn_p = find_conn_pointer(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\n", addr);

	if (!conn_p) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_pairing_confirm(conn_p);
		printf("Pairing Confirm Success\n");
	}
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn *conn_p = find_conn_pointer(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Pairing failed with %s reason 0x%02x\n", addr, reason);

	disconnected(conn_p, reason);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	OS_Status status;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("%s with %s\n", bonded ? "Bonded" : "Paired", addr);

	status = OS_TimerStart(&subscribe_timer);
	if (status != OS_OK) {
		printf("subscribe timer start error\n");
	}
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.passkey_confirm = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static struct bt_conn_auth_cb auth_cb_display_yes_no = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static struct bt_conn_auth_cb auth_cb_input = {
	.passkey_display = NULL,
	.passkey_entry = auth_passkey_entry,
	.passkey_confirm = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static struct bt_conn_auth_cb auth_cb_confirm = {
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static struct bt_conn_auth_cb auth_cb_all = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = auth_passkey_entry,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = auth_pairing_confirm,
	.pairing_failed = auth_pairing_failed,
	.pairing_complete = auth_pairing_complete,
};

static struct bt_conn_auth_cb *bt_auth_cb_slect(int bt_auth_cb_type)
{
	switch (bt_auth_cb_type) {
	case AUTH_CB_METHOD_CONFIRM:
		security_level = BT_SECURITY_L2;
		return &auth_cb_confirm;
	case AUTH_CB_METHOD_DISPLAY_YES_NO:
		security_level = BT_SECURITY_L2;
		return &auth_cb_display_yes_no;
	case AUTH_CB_METHOD_DISPLAY:
		security_level = BT_SECURITY_L3;
		bt_passkey_set(fixed_passkey);
		return &auth_cb_display;
	case AUTH_CB_METHOD_INPUT:
		security_level = BT_SECURITY_L3;
		return &auth_cb_input;
	case AUTH_CB_METHOD_ALL:
		security_level = BT_SECURITY_L2;
		return &auth_cb_all;
	case AUTH_CB_METHOD_OOB:
	case AUTH_CB_METHOD_NONE:
	default:
		security_level = BT_SECURITY_L2;
		return NULL;
	}

	return NULL;
}

/***************************************************************************/
/*                          CONNECTION MANAGER                             */
/***************************************************************************/
static void conn_init_clean(void)
{
	int i = 0;

	for (i = 0; i < CONN_MAX; i++) {
		default_conn[i] = NULL;
	}
}

static struct bt_conn *find_conn_pointer(struct bt_conn *conn)
{
	int i = 0;

	for (i = 0; i < CONN_MAX; i++) {
		if (conn == default_conn[i]) {
			return default_conn[i];
		}
	}
	return NULL;
}

static void update_conn_pointer(struct bt_conn *conn)
{
	int i = 0;

	for (i = 0; i < CONN_MAX; i++) {
		if (NULL == default_conn[i]) {
			default_conn[i] = conn;
			return;
		}
	}
}

static void conn_count_update(bool b)
{
	if (b && conn_count < CONN_MAX) {
		conn_count++;
	} else if (!b && conn_count) {
		conn_count--;
	}
}

static void conn_clean(struct bt_conn *conn)
{
	int i = 0;

	for (i = 0; i < CONN_MAX; i++) {
		if (conn == default_conn[i]) {
			default_conn[i] = NULL;
			conn_count_update(false);
			return;
		}
	}
}

/********************Scan Operation***************************/
static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	struct bt_conn *conn_p;
	int i;

	switch (data->type) {
	case BT_DATA_UUID128_SOME:
	case BT_DATA_UUID128_ALL:
		if (data->data_len % (sizeof(uint8_t) * 16) != 0) {
			printf("AD malformed\n");
			return true;
		}

		printf("[AD]: %u data_len %u\n", data->type, data->data_len);

		for (i = 0; i < data->data_len; i += (sizeof(uint8_t) * 16)) {
			struct bt_uuid_128 uuid;
			int err;

			uuid.uuid.type = BT_UUID_TYPE_128;
			memcpy(&uuid.val, &data->data[i], 16);
			if (bt_uuid_cmp(&uuid.uuid, &vnd_uuid.uuid)) {
				continue;
			}

			err = bt_le_scan_stop();
			if (err) {
				printf("Stop LE scan failed (err %d)\n", err);
				continue;
			}

			printf("Start connect %d\n", conn_count);
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn_p);
			if (err) {
				printf("Create connection failed (err %d)\n", err);
				start_scan();
				return false;
			}
			printf("Connection pointer %p\n", conn_p);
			update_conn_pointer(conn_p);
			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printf("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
			dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type == BT_HCI_ADV_IND || type == BT_HCI_ADV_DIRECT_IND) {
		bt_data_parse(ad, eir_found, (void *)addr);
	}
}

static void start_scan(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		printf("Scanning failed to start (err %d)\n", err);
		return;
	}

	printf("Scanning successfully started\n");
}

/****************Central Task****************************/
static void central_timer_cb(void *arg)
{
	static uint32_t i;
	uint8_t index;
	struct bt_conn *conn_p;
	int err;
	//char str[BT_UUID_STR_LEN];
	char addr[BT_ADDR_LE_STR_LEN];

	for (index = 0; index < conn_count; index++) {
		conn_p = default_conn[index];

		if (!(i % VND_WRITE_LONG_INVAL)) {
			bt_addr_le_to_str(bt_conn_get_dst(conn_p), addr, sizeof(addr));
			if (write_params[index].handle) {
				err = bt_gatt_write(conn_p, &write_params[index]);
				if (err) {
					printf("Write to %s FAIL! %d\n", addr, err);
				} else {
					printf("Write Req 0x%x to %s Success!\n", *(uint8_t *)write_params[index].data, addr);
				}
			}
		}
	}

	i++;
}

/****************Subscribe Task****************************/
static void subscribe_timer_cb(void *arg)
{
	uint8_t index;
	int err;
	bool subscribe_all = true;
	struct bt_conn *conn_p;
	char str[BT_UUID_STR_LEN];
	OS_Status status;

	for (index = 0; index < conn_count; index++) {
		conn_p = default_conn[index];

		if (!discover_service) {
			if (!(notify_enable_flag[DISCOVER_SERVICE_CTS - 1] & (1 << index))) {
				memcpy(&uuid, BT_UUID_CTS, sizeof(uuid));
				discover_params.uuid = &uuid.uuid;
				discover_params.func = discover_func;
				discover_params.start_handle = 0x0001;
				discover_params.end_handle = 0xffff;
				discover_params.type = BT_GATT_DISCOVER_PRIMARY;

				bt_uuid_to_str(&uuid.uuid, str, sizeof(str));
				printf("Discover CTS:UUID %s\n", str);
				discover_service = DISCOVER_SERVICE_CTS;
				err = bt_gatt_discover(conn_p, &discover_params);
				if (err) {
					printf("Discover failed (err %d)\n", err);
					return;
				}
				notify_enable_flag[DISCOVER_SERVICE_CTS - 1] |= 1 << index;
				subscribe_all = false;
				goto next_subscribe;
			}

			if (!(notify_enable_flag[DISCOVER_SERVICE_HRS - 1] & (1 << index))) {
				memcpy(&uuid, BT_UUID_HRS, sizeof(uuid));
				discover_params.uuid = &uuid.uuid;
				discover_params.func = discover_func;
				discover_params.start_handle = 0x0001;
				discover_params.end_handle = 0xffff;
				discover_params.type = BT_GATT_DISCOVER_PRIMARY;

				bt_uuid_to_str(&uuid.uuid, str, sizeof(str));
				printf("Discover HRS:UUID %s\n", str);
				discover_service = DISCOVER_SERVICE_HRS;
				err = bt_gatt_discover(conn_p, &discover_params);
				if (err) {
					printf("Discover failed (err %d)\n", err);
					return;
				}
				notify_enable_flag[DISCOVER_SERVICE_HRS - 1] |= 1 << index;
				subscribe_all = false;
				goto next_subscribe;
			}

			if (!(notify_enable_flag[DISCOVER_SERVICE_BAS - 1] & (1 << index))) {
				memcpy(&uuid, BT_UUID_BAS, sizeof(uuid));
				discover_params.uuid = &uuid.uuid;
				discover_params.func = discover_func;
				discover_params.start_handle = 0x0001;
				discover_params.end_handle = 0xffff;
				discover_params.type = BT_GATT_DISCOVER_PRIMARY;

				bt_uuid_to_str(&uuid.uuid, str, sizeof(str));
				printf("Discover BAS:UUID %s\n", str);
				discover_service = DISCOVER_SERVICE_BAS;
				err = bt_gatt_discover(conn_p, &discover_params);
				if (err) {
					printf("Discover failed (err %d)\n", err);
					return;
				}
				notify_enable_flag[DISCOVER_SERVICE_BAS - 1] |= 1 << index;
				subscribe_all = false;
				goto next_subscribe;
			}

			if (!(notify_enable_flag[DISCOVER_SERVICE_VND - 1] & (1 << index))) {
				discover_params.uuid = &vnd_uuid.uuid;
				discover_params.func = discover_func;
				discover_params.start_handle = 0x0001;
				discover_params.end_handle = 0xffff;
				discover_params.type = BT_GATT_DISCOVER_PRIMARY;

				bt_uuid_to_str(&vnd_notify_uuid.uuid, str, sizeof(str));
				printk("Discover VND:UUID %s\n", str);
				discover_service = DISCOVER_SERVICE_VND;
				err = bt_gatt_discover(conn_p, &discover_params);
				if (err) {
					printk("Discover failed (err %d)\n", err);
					return;
				}
				notify_enable_flag[DISCOVER_SERVICE_VND - 1] |= 1 << index;
				subscribe_all = false;
				goto next_subscribe;
			}

			if (!(notify_enable_flag[DISCOVER_SERVICE_VND_WRITE - 1] & (1 << index))) {
				discover_params.uuid = &vnd_uuid.uuid;
				discover_params.func = discover_func;
				discover_params.start_handle = 0x0001;
				discover_params.end_handle = 0xffff;
				discover_params.type = BT_GATT_DISCOVER_PRIMARY;

				bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
				printf("Discover ENC VND:UUID %s\n", str);
				discover_service = DISCOVER_SERVICE_VND_WRITE;
				err = bt_gatt_discover(conn_p, &discover_params);
				if (err) {
					printf("Discover failed (err %d)\n", err);
					return;
				}
				notify_enable_flag[DISCOVER_SERVICE_VND_WRITE - 1] |= 1 << index;
				subscribe_all = false;
				goto next_subscribe;
			}

			if (!write_params[index].func) {
				discover_write_params.uuid = &vnd_long_uuid.uuid;
				discover_write_params.func = discover_write_func;
				discover_write_params.start_handle = 0x0001;
				discover_write_params.end_handle = 0xffff;
				discover_write_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

				bt_uuid_to_str(&vnd_long_uuid.uuid, str, sizeof(str));
				printf("Discover LONG VND:UUID %s\n", str);
				err = bt_gatt_discover(conn_p, &discover_write_params);
				if (err) {
					printf("Discover failed (err %d)\n", err);
					return;
				}
			}
		}
	}

next_subscribe:
	if (subscribe_all) {
		status = OS_TimerStart(&central_timer);
		if (status != OS_OK) {
			printf("central timer start error\n");
			return;
		}
	} else {
		status = OS_TimerStart(&subscribe_timer);
		if (status != OS_OK) {
			printf("subscribe timer start error\n");
			return;
		}
	}
}

/****************Init Operation**************************/
static void bt_ready(int err)
{
	struct bt_conn_auth_cb *auth_cb_method;

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	printf("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);
	auth_cb_method = bt_auth_cb_slect(BT_AUTH_CB_METHOD);
	bt_conn_auth_cb_register(auth_cb_method);

	start_scan();
}

void bt_app_init(void)
{
	int err;
	OS_Status status;

	conn_init_clean();

	err = bt_ctrl_enable();
	if (err) {
		printf("Bluetooth controller init failed (err %d)\n", err);
		return;
	}

	status = OS_TimerCreate(&central_timer, OS_TIMER_PERIODIC, central_timer_cb, NULL, CENTRAL_PERIOD);
	if (status != OS_OK) {
		printf("central timer create error (err 0x%02x)\n", status);
		return;
	}

	status = OS_TimerCreate(&subscribe_timer, OS_TIMER_ONCE, subscribe_timer_cb, NULL, SUBSCRIBE_PERIOD);
	if (status != OS_OK) {
		printf("subscribe timer create error (err 0x%02x)\n", status);
		return;
	}

	err = bt_enable(bt_ready);
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return;
	}
}
