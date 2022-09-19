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
* includes
*/
#include <stdio.h>
#include <string.h>

#include "common/framework/platform_init.h"
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

/***************External Call Function**************************/
extern const char *bt_addr_str_real(const bt_addr_t *addr);

#define PERIPHERAL_BEACON_EXAMPLE   0
#define PERIPHERAL_BOND_EXAMPLE     0
#define PERIPHERAL_HT_EXAMPLE       0
#define CENTRAL_SCAN_EXAMPLE        0
#define CENTRAL_BOND_EXAMPLE        1
#define CENTRAL_HT_EXAMPLE          0

#if (PERIPHERAL_BEACON_EXAMPLE || PERIPHERAL_BOND_EXAMPLE || PERIPHERAL_HT_EXAMPLE)
#define EXAMPLE_USE_PERIPHERAL      1
#endif

#if (CENTRAL_SCAN_EXAMPLE || CENTRAL_BOND_EXAMPLE || CENTRAL_HT_EXAMPLE)
#define EXAMPLE_USE_CENTRAL      1
#endif

/*****************Global Variable*******************/
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#if (EXAMPLE_USE_PERIPHERAL)
bt_addr_le_t pripheral_static_id_addr = { BT_ADDR_LE_RANDOM, { {0x12, 0x34, 0x56, 0x78, 0x90, 0xC0} } };

/*Set Advertising Data*/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
	             0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
	              0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	              0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

/*Set Scan Response Data*/
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
#endif

#if (EXAMPLE_USE_CENTRAL)
bt_addr_le_t central_static_id_addr = { BT_ADDR_LE_RANDOM, { {0x09, 0x87, 0x65, 0x43, 0x21, 0xC0} } };
#endif

/*****************PERIPHERAL BEACON EXAMPLE*********************/
#if (PERIPHERAL_BEACON_EXAMPLE)
static void bt_ready(int err)
{
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	printf("Bluetooth initialized\n");

	/*Start advertising*/
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printf("Advertising failed to start (err %d)\n", err);
		return ;
	}

	printf("Beacon started\n");
}
#endif

/**************PERIPHERAL BOND EXAMPLE********************/
#if (PERIPHERAL_BOND_EXAMPLE)
enum AUTH_CB_METHOD_TYPES {
	AUTH_CB_METHOD_NONE,
	AUTH_CB_METHOD_CONFIRM,
	AUTH_CB_METHOD_DISPLAY_YES_NO,
	AUTH_CB_METHOD_DISPLAY,
	AUTH_CB_METHOD_INPUT,
	AUTH_CB_METHOD_OOB,
	AUTH_CB_METHOD_ALL,
};
#define BT_AUTH_CB_METHOD   AUTH_CB_METHOD_DISPLAY
#define BT_LE_PASSKEY_STR_LEN   7
static unsigned int fixed_passkey = 123456;

/*connection managerment*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printf("Failed to get info\n");
		return ;
	}

	printf("[H] Disconnected %s reason 0x%02x \n", bt_addr_str_real(&info.le.dst->a), reason);
}

static void connected(struct bt_conn *conn, uint8_t err)
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
		printf("= Remote Address %s\n", bt_addr_str_real(&rpa.a));
		printf("= Internval      %d\n", info.le.interval);
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Enter passkey for %s\n", addr);

	bt_conn_auth_passkey_entry(conn, fixed_passkey);
	printf("Enter Passkey Success\n");
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	char passkey_str[BT_LE_PASSKEY_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	snprintf(passkey_str, 7, "%06u", passkey);

	printf("Confirm passkey for %s: %s\n", addr, passkey_str);

	bt_conn_auth_passkey_confirm(conn);
	printf("Passkey Confirm Success\n");

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\n", addr);

	bt_conn_auth_pairing_confirm(conn);
	printf("Pairing Confirm Success\n");
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Pairing failed with %s reason 0x%02x\n", addr, reason);

	disconnected(conn, reason);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("%s with %s\n", bonded ? "Bonded" : "Paired", addr);
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
		return &auth_cb_confirm;
	case AUTH_CB_METHOD_DISPLAY_YES_NO:
		return &auth_cb_display_yes_no;
	case AUTH_CB_METHOD_DISPLAY:
		bt_passkey_set(fixed_passkey);
		return &auth_cb_display;
	case AUTH_CB_METHOD_INPUT:
		return &auth_cb_input;
	case AUTH_CB_METHOD_ALL:
		return &auth_cb_all;
	case AUTH_CB_METHOD_OOB:
	case AUTH_CB_METHOD_NONE:
	default:
		return NULL;
	}

	return NULL;
}

static void bt_ready(int err)
{
	struct bt_conn_auth_cb *auth_cb_method;

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	printf("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printf("Advertising failed to start (err 0x%02x)\n", err);
		return;
	}

	printf("Advertising successfully started\n");

	auth_cb_method = bt_auth_cb_slect(BT_AUTH_CB_METHOD);
	bt_conn_auth_cb_register(auth_cb_method);
	bt_conn_cb_register(&conn_callbacks);
}
#endif

#if (PERIPHERAL_HT_EXAMPLE)
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_ind_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_write_uuid = BT_UUID_INIT_128(
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_notify_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

#define PERIPHERAL_PERIOD           1000 //1000ms
#define PERIPHERAL_NOTIFY_VALUE     3000
static OS_Timer_t peripheral_timer;

/*connection managerment*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printf("Failed to get info\n");
		return ;
	}

	printf("[H] Disconnected %s reason 0x%02x \n", bt_addr_str_real(&info.le.dst->a), reason);
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printf("Advertising failed to start (err 0x%02x)\n", err);
		return;
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
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
		printf("= Remote Address %s\n", bt_addr_str_real(&rpa.a));
		printf("= Internval      %d\n", info.le.interval);
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

#define MAX_DATA 74
static uint32_t vnd_ind_val = 0;
static uint8_t simulate_vnd;
static struct bt_gatt_indicate_params ind_params;
static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

static ssize_t write_ind_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset,
                                uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_ind_val)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static void vnd_ccc_ind_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;

	printf("VND indication %s\n", simulate_vnd ? "enabled" : "disabled");
}

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	printf("Indication %s\n", err != 0U ? "fail" : "success");
}

static uint8_t vnd_value = 0;

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	printf("write vnd:%d\n", *value);

	return len;
}

static uint8_t vnd_notify_enabled;
static uint32_t vnd_notify_value = 0;
static struct bt_gatt_notify_params notify_params;
static void notify_cb(struct bt_conn *conn, void *user_data)
{
	printf("Nofication sent to conn %p\n", conn);
}

static void vnd_ccc_notify_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	vnd_notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	printf("VND notifications %s\n", vnd_notify_enabled ? "enabled" : "disabled");
}

/*Vendor Primary Service Declaration*/
static struct bt_gatt_attr vnd_attrs[] = {
	/* Vendor Primary Service Declaration */
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&vnd_ind_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
	                        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	                        read_vnd, write_ind_vnd, &vnd_ind_val),
	BT_GATT_CCC(vnd_ccc_ind_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&vnd_write_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	                BT_GATT_PERM_READ |BT_GATT_PERM_WRITE,
	                read_vnd, write_vnd, &vnd_value),
	BT_GATT_CHARACTERISTIC(&vnd_notify_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	                BT_GATT_PERM_READ, read_vnd, NULL, &vnd_notify_value),
	BT_GATT_CCC(vnd_ccc_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service vnd_svc = BT_GATT_SERVICE(vnd_attrs);

static void peripheral_timer_cb(void *arg)
{
	if (simulate_vnd) {
		ind_params.attr = &vnd_svc.attrs[2];
		ind_params.func = indicate_cb;
		ind_params.data = &vnd_ind_val;
		ind_params.len = sizeof(vnd_ind_val);

		if (bt_gatt_indicate(NULL, &ind_params) == 0) {
			vnd_ind_val++;

			if (vnd_ind_val == UINT8_MAX) {
				vnd_ind_val = 0;
			}
		}
	}

	if (vnd_notify_enabled) {
		notify_params.uuid = &vnd_notify_uuid.uuid;
		notify_params.attr = &vnd_svc.attrs[7];
		notify_params.data = &vnd_notify_value;
		notify_params.len  = sizeof(vnd_notify_value);
		notify_params.func = notify_cb;

		if (bt_gatt_notify_cb(NULL, &notify_params) == 0) {
			vnd_notify_value++;

			if (vnd_notify_value == UINT8_MAX) {
				vnd_notify_value = 0;
			}
		}
	}
}

static void bt_ready(int err)
{
	int status;

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	bt_gatt_service_register(&vnd_svc);
	bt_conn_cb_register(&conn_callbacks);

	printf("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printf("Advertising failed to start (err 0x%02x)\n", err);
		return;
	}

	printf("Advertising successfully started\n");

	status = OS_TimerCreate(&peripheral_timer, OS_TIMER_PERIODIC, peripheral_timer_cb, NULL, PERIPHERAL_PERIOD);
	if (status != OS_OK) {
		printf("peripheral timer create error (err 0x%02x)\n", status);
		return ;
	}
	OS_TimerStart(&peripheral_timer);
}

#endif

#if (CENTRAL_SCAN_EXAMPLE)
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
		type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return ;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printf("Device found: %s (RSSI %d)\n", addr_str, rssi);
}

static void start_scan(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printf("Scanning failed to start (err %d)\n", err);
		return ;
	}

	printf("Scanning successfully started\n");
}

static void bt_ready(int err)
{

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	printf("Bluetooth initialized\n");

	start_scan();
}
#endif

#if (CENTRAL_BOND_EXAMPLE)
static void start_scan(void);

enum AUTH_CB_METHOD_TYPES {
	AUTH_CB_METHOD_NONE,
	AUTH_CB_METHOD_CONFIRM,
	AUTH_CB_METHOD_DISPLAY_YES_NO,
	AUTH_CB_METHOD_DISPLAY,
	AUTH_CB_METHOD_INPUT,
	AUTH_CB_METHOD_OOB,
	AUTH_CB_METHOD_ALL,
};

#define BT_AUTH_CB_METHOD   AUTH_CB_METHOD_INPUT
#define BT_LE_PASSKEY_STR_LEN   7

static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static unsigned int fixed_passkey = 123456;
static struct bt_conn *default_conn;
static bt_security_t security_level;

/*connection managerment*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printf("Failed to get info\n");
		return ;
	}

	printf("[H] Disconnected %s reason 0x%02x \n", bt_addr_str_real(&info.le.dst->a), reason);
	start_scan();
}

static void connected(struct bt_conn *conn, uint8_t err)
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
		printf("= Remote Address %s\n", bt_addr_str_real(&rpa.a));
		printf("= Internval      %d\n", info.le.interval);
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");
		err = bt_conn_set_security(conn, security_level);
		if (err) {
			printk("Failed to set security %d\n", err);
			disconnected(conn, err);
		}
	}

}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Enter passkey for %s\n", addr);

	bt_conn_auth_passkey_entry(conn, fixed_passkey);
	printf("Enter Passkey Success\n");
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	char passkey_str[BT_LE_PASSKEY_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	snprintf(passkey_str, 7, "%06u", passkey);

	printf("Confirm passkey for %s: %s\n", addr, passkey_str);

	bt_conn_auth_passkey_confirm(conn);
	printf("Passkey Confirm Success\n");

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\n", addr);

	bt_conn_auth_pairing_confirm(conn);
	printf("Pairing Confirm Success\n");
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Pairing failed with %s reason 0x%02x\n", addr, reason);

	disconnected(conn, reason);
}

static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("%s with %s\n", bonded ? "Bonded" : "Paired", addr);
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
		return NULL;
	}

	return NULL;
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;
	struct bt_conn *conn_p;

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

			conn_p = default_conn;
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn_p);
			if (err) {
				printf("Create connection failed (err %d)\n", err);
				start_scan();
				return false;
			}
			return false;
		}
	default:
		break;
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

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printf("Scanning failed to start (err %d)\n", err);
		return ;
	}

	printf("Scanning successfully started\n");
}

static void bt_ready(int err)
{
	struct bt_conn_auth_cb *auth_cb_method;

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	printf("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);
	auth_cb_method = bt_auth_cb_slect(BT_AUTH_CB_METHOD);
	bt_conn_auth_cb_register(auth_cb_method);

	start_scan();
}

#endif

#if (CENTRAL_HT_EXAMPLE)
static void start_scan(void);

#define CENTRAL_PERIOD       1000 //1000ms

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params[2];
static struct bt_gatt_discover_params discover_write_params;
static struct bt_gatt_write_params write_params;
static uint8_t write_data = 0;
static OS_Timer_t central_timer;
static uint8_t notify_flag;
static uint8_t indicate_flag;
static uint8_t write_flag;
static uint8_t discover_service = 0;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);

static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_ind_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_write_uuid = BT_UUID_INIT_128(
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 vnd_notify_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint8_t indicate_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
	uint32_t vnd;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(&vnd, data, length);

	printk("[INDICATION](%s, VND: %d)\n", addr, vnd);

	return BT_GATT_ITER_CONTINUE;
}


static uint8_t notify_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params);
static uint8_t indicate_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printf("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printf("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, &vnd_uuid.uuid)) {
		discover_params.uuid = &vnd_ind_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printf("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, &vnd_ind_uuid.uuid)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params[0].value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params[0].notify = indicate_func;
		subscribe_params[0].value = BT_GATT_CCC_INDICATE;
		subscribe_params[0].ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params[0]);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
			indicate_flag = 1;
			discover_service = 0;
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                        const void *data, uint16_t length)
{
	uint32_t vnd;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!data) {
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	memcpy(&vnd, data, length);

	printk("[NOTIFICATION](%s, VND: %d)\n", addr, vnd);

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t notify_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printf("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printf("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, &vnd_uuid.uuid)) {
		discover_params.uuid = &vnd_notify_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printf("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, &vnd_notify_uuid.uuid)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params[1].value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params[1].notify = notify_func;
		subscribe_params[1].value = BT_GATT_CCC_NOTIFY;
		subscribe_params[1].ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params[1]);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
			notify_flag = 1;
			discover_service = 0;
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void write_func(struct bt_conn *conn, uint8_t err,
                struct bt_gatt_write_params *params)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Write 0x%x to %s SUCCESS\n", *(uint8_t *)params->data, addr);

		if (write_data == 255) {
			write_data = write_data % 255;
		} else {
			write_data += 1;
		}
	} else {
		printk("Write to %s FAIL %d\n", addr, err);
	}
}

static uint8_t discover_write_func(struct bt_conn *conn,
                    const struct bt_gatt_attr *attr,
                    struct bt_gatt_discover_params *params)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!attr) {
		printk("Discover write %s complete\n", addr);
		memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] %s write handle %u\n", addr, attr->handle);

	write_params.handle = attr->handle;
	write_params.func = write_func;
	write_params.data = &write_data;
	write_params.length = 1;
	write_flag = 1;
	discover_service = 0;
	return BT_GATT_ITER_CONTINUE;
}

static void central_timer_cb(void *arg)
{
	int err = 0;
	char addr[BT_ADDR_LE_STR_LEN];

	if (!indicate_flag && !discover_service) {
		printf("indicate flag\n");
		discover_params.uuid = &vnd_uuid.uuid;
		discover_params.func = indicate_discover_func;
		discover_params.start_handle = 0x0001;
		discover_params.end_handle = 0xffff;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;
		discover_service = 1;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printf("Discover failed(err %d)\n", err);
			return ;
		}
	}

	if (!notify_flag && !discover_service) {
		printf("notify flag\n");
		discover_params.uuid = &vnd_uuid.uuid;
		discover_params.func = notify_discover_func;
		discover_params.start_handle = 0x0001;
		discover_params.end_handle = 0xffff;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;
		discover_service = 1;

		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printf("Discover failed(err %d)\n", err);
			return ;
		}
	}

	if (!write_flag && !discover_service) {
		printf("write_flag\n");
		discover_write_params.uuid = &vnd_write_uuid.uuid;
		discover_write_params.func = discover_write_func;
		discover_write_params.start_handle = 0x0001;
		discover_write_params.end_handle = 0xffff;
		discover_write_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		discover_service = 1;

		err = bt_gatt_discover(default_conn, &discover_write_params);
		if (err) {
			printk("Discover write failed(err %d)\n", err);
			return;
		}
	} else {
		printf("write vnd\n");
		bt_addr_le_to_str(bt_conn_get_dst(default_conn), addr, sizeof(addr));
		if (write_params.handle) {
			err = bt_gatt_write(default_conn, &write_params);
			if (err) {
				printf("Write to %s FAIL! %d\n", addr, err);
			} else {
				printf("Write Req 0x%x to %s Success!\n", *(uint8_t *)write_params.data, addr);
			}
		}
	}
}

/*connection managerment*/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printf("Failed to get info\n");
		return ;
	}

	printf("[H] Disconnected %s reason 0x%02x \n", bt_addr_str_real(&info.le.dst->a), reason);

	if (default_conn != conn) {
		return ;
	}

	default_conn = NULL;
	start_scan();
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	bt_addr_le_t rpa = {0};
	int result;
	struct bt_conn_info info;
	OS_Status status;

	if (conn_err) {
		printf("[H] Connectionfailed reason %u\n", conn_err);
		default_conn = NULL;
		start_scan();
	} else {
		result = bt_conn_get_info(conn, &info);
		if (result) {
			printf("Failed to get info\n");
			return ;
		}
		memcpy(&rpa, &info.le.src, sizeof(bt_addr_le_t));
		printf("[H] Connected!! \n");
		printf("========== Connection Parameter ==========\n");
		printf("= Remote Address %s\n", bt_addr_str_real(&rpa.a));
		printf("= Internval      %d\n", info.le.interval);
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");

		if (conn == default_conn) {
			status = OS_TimerCreate(&central_timer, OS_TIMER_PERIODIC, central_timer_cb, NULL, CENTRAL_PERIOD);
			if (status != OS_OK) {
				printk("central timer create error (err 0x%02x)\n", status);
				return ;
			}

			OS_TimerStart(&central_timer);
		}
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
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

			printf("addr:%s\n", bt_addr_str_real(&addr->a));
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &default_conn);
			if (err) {
				printf("Create connection failed (err %d)\n", err);
				start_scan();
				return false;
			}
			return false;
		}
	default:
		break;
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

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printf("Scanning failed to start (err %d)\n", err);
		return ;
	}

	printf("Scanning successfully started\n");
}

static void bt_ready(int err)
{
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return ;
	}

	bt_conn_cb_register(&conn_callbacks);

	printf("Bluetooth initialized\n");

	start_scan();
}
#endif

/*****************************************************************
*****************************************************************/
int main(void)
{
	int err;

	platform_init();

	bt_ctrl_enable();

	printf("Start BLE Example!\n");

#if (EXAMPLE_USE_PERIPHERAL)
	err = bt_id_create(&pripheral_static_id_addr, NULL);
	if (err) {
		printf("Unable to set peripheral identity address (err: %d)\n", err);
		return err;
	}
#elif (EXAMPLE_USE_CENTRAL)
	err = bt_id_create(&central_static_id_addr, NULL);
	if (err) {
		printf("Unable to set central identity address (err: %d)\n", err);
		return err;
	}
#endif

	err = bt_enable(bt_ready);
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	return 0;
}

