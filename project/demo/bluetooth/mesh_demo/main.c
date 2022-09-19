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
#include "ble/bluetooth/bluetooth.h"
#include "ble/bluetooth/mesh.h"

#include "ble/bluetooth/mesh/mesh_common.h"
#include "ble/bluetooth/mesh/generic_onoff_srv.h"
#include "ble/bluetooth/mesh/generic_onoff_cli.h"
#include "ble/bluetooth/mesh/transition.h"
#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "common/framework/bt_ctrl.h"
#endif
#endif

#if defined(CONFIG_BT_MESH_FRIEND)
#define MESH_FRIEND_FUNC_ENABLE     1
#endif

#define DEV_UUID_MATCH_1            'X'
#define DEV_UUID_MATCH_2            'R'

/***************Global Variable**************************/
static uint16_t primary_addr;
static uint16_t primary_net_idx;

static uint8_t g_onoff_value;

/* Disable OOB security for SILabs Android app */
static uint8_t dev_uuid[16] = { DEV_UUID_MATCH_1, DEV_UUID_MATCH_2, 0xdc, 0xdb };

/***************Publication Declarations**************************
  * The publication messages are initialized to the
  * the size of the opcode + content
  *
  * For publication, the message must be in static or global as
  * it is re-transmitted several times. This occurs
  * after the function that called bt_mesh_model_publish() has
  * exited and the stack is no longer valid.
  *
  * Note that the additional 4 bytes for the AppMIC is not needed
  * because it is added to a stack variable at the time a
  * transmission occurs.
  *
*****************************************************************/
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 4);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 4);

/***************Model Processing*******************/
/***************Configuration Client Declaration*******************/
static struct bt_mesh_cfg_cli cfg_cli = {
};

/***************Health Configuration Declaration*******************/
static void healthsrv_attention_on(struct bt_mesh_model *model)
{
	printf("*****attention_on*****\n");
}

static void healthsrv_attention_off(struct bt_mesh_model *model)
{
	printf("*****attention_off*****\n");
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = healthsrv_attention_on,
	.attn_off = healthsrv_attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

static void show_faults(uint8_t test_id, uint16_t cid, uint8_t *faults, size_t fault_count)
{
	size_t i;

	if (!fault_count) {
		printf("Health Test ID 0x%02x Company ID 0x%04x: no faults\n", test_id, cid);
		return;
	}

	printf("Health Test ID 0x%02x Company ID 0x%04x Fault Count %u:\n", test_id, cid, fault_count);

	for (i = 0; i < fault_count; i++) {
		printf("\t0x%02x\n", faults[i]);
	}
}

static void health_current_status(struct bt_mesh_health_cli *cli, uint16_t addr,
                    uint8_t test_id, uint16_t cid, uint8_t *faults,
                    size_t fault_count)
{
	printf("Health Current Status from 0x%04x\n", addr);
	show_faults(test_id, cid, faults, fault_count);
}

static struct bt_mesh_health_cli health_cli = {
	.current_status = health_current_status,
};

/***************Onoff Configuration Declaration*******************/
static void app_onoff_srv_set_cb(const struct bt_mesh_model *model, uint8_t onoff,
                uint8_t target_onoff, const struct bt_mesh_transition_status *opt)
{
	g_onoff_value = onoff;
	printf("[app] onoff set(%d)", onoff);
	if (opt) {
		printf("target onoff(%d), total_steps(%d), steps(%d)",
		       target_onoff, opt->total_steps, opt->present_steps);
	}
	printf("\n");
}

static void app_onoff_srv_get_cb(const struct bt_mesh_model *model,
                          uint8_t *onoff)
{
	printf("[app] onoff get(%d)\n", g_onoff_value);
	*onoff = g_onoff_value;
}

static struct bt_mesh_gen_onoff_srv onoff_srv_user_data = {
	.set_cb = app_onoff_srv_set_cb,
	.get_cb = app_onoff_srv_get_cb,
};

void app_onoff_cli_status_cb(const struct bt_mesh_model *model, uint8_t present,
                uint8_t target, const struct bt_mesh_transition_remain_time *opt)
{
	printf("present onoff(%d), target onoff(%d)", present, target);
	if (opt)
		printf(", remain time(%d)", opt->remain_time);
	printf("\n");
}

static struct bt_mesh_gen_onoff_cli onoff_cli_user_data = {
	.status_cb = app_onoff_cli_status_cb
};

/****************Element Model Declarations*********************/
/*
* Element 0 Root Models
*/
static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL_HEALTH_CLI(&health_cli),
	BT_MESH_MODEL_GEN_ONOFF_SRV(&onoff_srv_user_data, &gen_onoff_pub_srv),
	BT_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_user_data, &gen_onoff_pub_cli),
};

/*********Root and Secondary Element Declarations*********/
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/***************Provisioning Processing*******************/
static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printf("***** OOB number : %u\n", number);
	return 0;
}

static int output_string(const char *str)
{
	printf("***** OOB String : %s\n", str);
	return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
#if (CONFIG_BT_MESH_FRIEND)
#if (MESH_FRIEND_FUNC_ENABLE)
	int err;
	uint8_t frnd;
#endif
#endif

	primary_addr = addr;
	primary_net_idx = net_idx;
	printf("***** Provisioning - complete\n");
	printf("net_idx 0x%04x\naddr 0x%04x\n", net_idx, addr);

#if (CONFIG_BT_MESH_FRIEND)
#if (MESH_FRIEND_FUNC_ENABLE)
	err = bt_mesh_cfg_friend_set(primary_net_idx, primary_addr, 1, &frnd);
	if (err) {
		printf("Unable to send Friend Get/Set (err %d)\n", err);
	} else {
		printf("Friend is set to 0x%02x\n", frnd);
	}
#endif
#endif
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.oob_info = (BT_MESH_PROV_OOB_NUMBER | BT_MESH_PROV_OOB_STRING),
	.output_size = 6,
	.output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
	.output_number = output_number,
	.output_string = output_string,
	.complete = prov_complete,
	.reset = prov_reset,
};

/****************Init Operation**************************/
static void bt_ready(int err)
{
	struct bt_le_oob oob;

	if (err) {
		printf("bt_ready : Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printf("bt_ready : Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		printf("bt_ready : Identity Address unavailable\n");
	} else {
		memcpy(dev_uuid + 2, oob.addr.a.val, 6);
	}

	if (bt_mesh_is_provisioned()) {
		printf("Mesh network restored from flash\n");
	} else {
		bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
	}

	printf("bt_ready : Mesh initialized\n");
}

int main(void)
{
	int err;

	platform_init();

	err = bt_ctrl_enable();
	if (err) {
		printf("Bluetooth controller init failed (err %d)\n", err);
		return 0;
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printf("main : Bluetooth init failed (err %d)\n", err);
	}

	return 0;
}
