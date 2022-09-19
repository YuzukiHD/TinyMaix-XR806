/*
********************************************************************************
* @file    peripheral.c
* @brief   This is the example for BLE peripheral.
******************************************************************************
*/
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <ble/sys/byteorder.h>
#include <zephyr.h>

#include "settings/settings.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <gatt/hrs.h>
#include <gatt/dis.h>
#include <gatt/bas.h>
#include <gatt/cts.h>
#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "common/framework/bt_ctrl.h"
#endif
#endif

/***************External Call Function**************************/
extern const char *bt_addr_str_real(const bt_addr_t *addr);

/***************Global Variables*******************************/
#define BT_LE_ADV_CONN_NAME_ONE_SHOT BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | \
                                           BT_LE_ADV_OPT_ONE_TIME | \
                                           BT_LE_ADV_OPT_USE_NAME, \
                                           BT_GAP_ADV_FAST_INT_MIN_2, \
                                           0X00B0, NULL)

#define PERIPHERAL_PERIOD       1000 //1000ms
#define CTS_UPDATE_IVAL         120
#define HRS_UPDATE_IVAL         30
#define BAS_UPDATE_IVAL         60
#define VND_UPDATE_IVAL         1

static struct bt_conn *default_conn;
static OS_Timer_t peripheral_timer;

/***************Custom Service Variables************************/
/* Custom Service UUID */
static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
	0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
	0xf5, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const struct bt_uuid_128 vnd_notify_uuid = BT_UUID_INIT_128(
	0xf6, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* Custom Service Data */
#define MAX_LONG_DATA 20    //max 20
static uint8_t vnd_value[] = {
	'0', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'1', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'2', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'3', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'4', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'5', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'6', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'7', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'8', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'9', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'a', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'b', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'-', 'E', 'N', 'D', 'S', 'T', 'R', 'I', 'N', 'G', '-',
	'\0',
};

static uint8_t vnd_long_value[] = {
	'0', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'1', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'2', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'3', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'4', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'5', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'6', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'7', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'8', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'9', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'a', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'b', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'-', 'E', 'N', 'D', 'S', 'T', 'R', 'I', 'N', 'G', '-',
	'\0',
};

static uint8_t vnd_long_long_value[] = {
	'0', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'1', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'2', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'3', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'4', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'5', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'6', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'7', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'8', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'9', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'a', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'b', ':', 'T', 'E', 'S', 'T', 'S', 'T', 'R', 'I', 'N', 'G', '-', 'F', 'O', 'R', 'B', 'L', 'E', ';',
	'-', 'E', 'N', 'D', 'S', 'T', 'R', 'I', 'N', 'G', '-',
	'\0',
};

static uint8_t vnd_notify_value[] = {
	'X', 'r', 'a', 'd', 'i', 'o', ' ', 'B', 'L', 'E', ' ', 'D', 'e', 'm', 'o',
	'!', '!', '!', '!', '\0',
};

static int signed_value = 0;

static uint8_t indicating = 0;

/*Custom Service Global variable*/
static uint8_t simulate_vnd;
static struct bt_gatt_indicate_params ind_params;
static struct bt_gatt_cep vnd_long_cep = {
	.properties = BT_GATT_CEP_RELIABLE_WRITE,
};
static bool vnd_notif_enabled = false;

/**********************Custom Service Function********************/
/**********************vnd_enc_uuid/vnd_auth_uuid*****************/
static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            const void *buf, uint16_t len, uint16_t offset,
            uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(vnd_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memset(value, 0, sizeof(vnd_value));

	memcpy(value + offset, buf, len);

	*(value + offset + len) = '\0';

	return len;
}

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
            uint8_t err)
{
	printf("Indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}

/**********************vnd_long_uuid*****************************/
static ssize_t read_long_vnd(struct bt_conn *conn,
                const struct bt_gatt_attr *attr, void *buf,
                uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
	            sizeof(vnd_long_value));
}

static ssize_t write_long_vnd(struct bt_conn *conn,
                const struct bt_gatt_attr *attr, const void *buf,
                uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > sizeof(vnd_long_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memset(value, 0, sizeof(vnd_long_value));

	memcpy(value + offset, buf, len);

	*(value + offset + len) = '\0';

	return len;
}

/**********************vnd_signed_uuid*****************************/
static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
	            sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                const void *buf, uint16_t len, uint16_t offset,
                uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(signed_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

/**********************vnd_notify_uuid*****************************/
static void vnd_ccc_notify_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	vnd_notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	printf("VND notifications %s\n", vnd_notif_enabled ? "enabled" : "disabled");
}

/**********************vnd_write_cmd_uuid*****************************/
static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                const void *buf, uint16_t len, uint16_t offset,
                uint8_t flags)
{
	uint8_t *value = attr->user_data;

	/* Write request received. Reject it since this char only accepts
	 * Write Commands.
	 */
	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}


	if (offset + len > sizeof(vnd_long_long_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memset(value, 0, sizeof(vnd_long_long_value));

	memcpy(value + offset, buf, len);

	*(value + offset + len) = '\0';

	return len;
}

/* Vendor Primary Service Declaration */
static struct bt_gatt_attr vnd_attrs[] = {
	/* Vendor Primary Service Declaration */
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&vnd_enc_uuid.uuid,
	                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE |
	                BT_GATT_CHRC_INDICATE,
	                BT_GATT_PERM_READ_ENCRYPT |
	                BT_GATT_PERM_WRITE_ENCRYPT,
	                read_vnd, write_vnd, &indicating),
	BT_GATT_CCC(vnd_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(&vnd_auth_uuid.uuid,
	                BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	                BT_GATT_PERM_READ_AUTHEN |
	                BT_GATT_PERM_WRITE_AUTHEN,
	                read_vnd, write_vnd, &vnd_value),
	BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid, BT_GATT_CHRC_READ |
	                BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
	                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
	                BT_GATT_PERM_PREPARE_WRITE,
	                read_long_vnd, write_long_vnd, &vnd_long_value),
	BT_GATT_CEP(&vnd_long_cep),
	BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid, BT_GATT_CHRC_READ |
	                BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
	                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	                read_signed, write_signed, &signed_value),
	BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid, BT_GATT_CHRC_READ |
	                BT_GATT_CHRC_WRITE_WITHOUT_RESP,
	                BT_GATT_PERM_READ |
	                BT_GATT_PERM_WRITE, read_vnd,
	                write_without_rsp_vnd, &vnd_long_long_value),
	BT_GATT_CHARACTERISTIC(&vnd_notify_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	                BT_GATT_PERM_READ, read_vnd, NULL, &vnd_notify_value),
	BT_GATT_CCC(vnd_ccc_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service vnd_svc = BT_GATT_SERVICE(vnd_attrs);

/********************Advertise Data*********************/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
	                0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
	                0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	                0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
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

/*****************Connection Realted*******************/
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
			return;
		}
		memcpy(&rpa, &info.le.src, sizeof(bt_addr_le_t));
		printf("[H] Connected!! \n");
		printf("========== Connection Parameter ==========\n");
		printf("= Remote Address %s\n", bt_addr_str_real(&rpa.a));
		printf("= Internval      %d\n", info.le.interval);
		printf("= Latency        %d\n", info.le.latency);
		printf("= Timeout        %d\n", info.le.timeout);
		printf("==========================================\n");
		default_conn = conn;
		OS_TimerStart(&peripheral_timer);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err;
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printf("Failed to get info\n");
		return;
	}

	printf("[H] Disconnected %s reason 0x%02x \n", bt_addr_str_real(&info.le.dst->a), reason);
	default_conn = NULL;
	OS_TimerStop(&peripheral_timer);
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME_ONE_SHOT, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printf("Advertising failed to start (err 0x%02x)\n", err);
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Enter passkey for %s\n", addr);

	if (!default_conn) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_passkey_entry(default_conn, fixed_passkey);
		printf("Enter Passkey Success\n");
	}
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	char passkey_str[BT_LE_PASSKEY_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	snprintf(passkey_str, 7, "%06u", passkey);

	printf("Confirm passkey for %s: %s\n", addr, passkey_str);

	if (!default_conn) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_passkey_confirm(default_conn);
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

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Confirm pairing for %s\n", addr);

	if (!default_conn) {
		printf("Not Connected\n");
	} else {
		bt_conn_auth_pairing_confirm(default_conn);
		printf("Pairing Confirm Success\n");
	}
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printf("Pairing failed with %s reason 0x%02x\n", addr, reason);

	disconnected(default_conn, reason);
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

static void vnd_notify(void)
{
	static uint8_t vnd[MAX_LONG_DATA];

	if (!vnd_notif_enabled)
		return;

	memcpy(vnd, vnd_notify_value, MAX_LONG_DATA);

	bt_gatt_notify(NULL, &vnd_svc.attrs[14], vnd, sizeof(vnd));
}

static void peripheral_timer_cb(void *arg)
{
	static uint8_t i;
	static uint8_t heartrate = 90;

	/* Current Time Service updates only when time is changed */
	if (!(i % CTS_UPDATE_IVAL)) {
		cts_notify();
	}

	/* Heartrate measurements simulation */
	if (!(i % HRS_UPDATE_IVAL)) {
		bt_gatt_hrs_notify((heartrate++ > 160) ? (heartrate = 90) : heartrate);
	}

	/* Battery level simulation */
	if (!(i % BAS_UPDATE_IVAL)) {
		bas_notify();
	}

	if (!(i % VND_UPDATE_IVAL)) {
		vnd_notify();
		/* Vendor indication simulation */
		if (simulate_vnd) {
			if (!indicating) {
				ind_params.attr = &vnd_attrs[2];
				ind_params.func = indicate_cb;
				ind_params.data = &indicating;
				ind_params.len = sizeof(indicating);

				if (bt_gatt_indicate(NULL, &ind_params) == 0) {
					indicating = 1U;
				}
			}
		}
	}

	i++;
}

static void bt_ready(int err)
{
	struct bt_conn_auth_cb *auth_cb_method;

	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printf("Bluetooth initialized\n");
	dis_init("XR Model", "XRadio");
	hrs_init(0x01);
	bas_init();
	cts_init();
	bt_gatt_service_register(&vnd_svc);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bt_conn_cb_register(&conn_callbacks);
	auth_cb_method = bt_auth_cb_slect(BT_AUTH_CB_METHOD);
	bt_conn_auth_cb_register(auth_cb_method);

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME_ONE_SHOT, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printf("Advertising failed to start (err 0x%02x)\n", err);
		return;
	}

	printf("Advertising successfully started\n");
}

void bt_app_init(void)
{
	int err;
	OS_Status status;

	err = bt_ctrl_enable();
	if (err) {
		printf("Bluetooth controller init failed (err %d)\n", err);
		return;
	}

	status = OS_TimerCreate(&peripheral_timer, OS_TIMER_PERIODIC, peripheral_timer_cb, NULL, PERIPHERAL_PERIOD);
	if (status != OS_OK) {
		printf("peripheral timer create error (err 0x%02x)\n", status);
		return;
	}

	err = bt_enable(bt_ready);
	if (err) {
		printf("Bluetooth init failed (err %d)\n", err);
		return;
	}
}

