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

#if PRJCONF_BLE_EN
#ifndef PRJCONF_BLE_ETF
#include "common/framework/bt_ctrl.h"
#endif
#endif

#define PROVISION_NODE_CNT_MAX      (CONFIG_BT_MESH_CDB_NODE_COUNT - 1)
#define ONOFF_CHANGE_PERIOD         (10000) //10s
#define ONOFF_THREAD_STACK_SIZE     (1024 * 2)
#define NODE_UUID_MATCH_1           'X'
#define NODE_UUID_MATCH_2           'R'

/***************Global Variable**************************/
static const uint16_t net_idx;
static const uint16_t app_idx;
static uint16_t self_addr = 1, node_addr[PROVISION_NODE_CNT_MAX] = {0};
static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };
static uint8_t node_uuid[16];
static uint8_t node_uuid_match[2] = {NODE_UUID_MATCH_1, NODE_UUID_MATCH_2};
static bool node_provising = false;
static uint16_t node_provising_addr;

static OS_Thread_t onoff_task_thread;

static OS_Semaphore_t sem_unprov_beacon;
static OS_Semaphore_t sem_node_added;

static uint8_t g_onoff_value;

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
		printf(" target onoff(%d), total_steps(%d), steps(%d)",
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
	BT_MESH_MODEL_GEN_ONOFF_SRV(&onoff_srv_user_data, NULL),
	BT_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_user_data, NULL),
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

static void unprovisioned_beacon(uint8_t uuid[16],
                 bt_mesh_prov_oob_info_t oob_info,
                 uint32_t *uri_hash)
{
	if (!node_provising && memcmp(uuid, node_uuid_match, sizeof(node_uuid_match)) == 0) {
		memcpy(node_uuid, uuid, 16);
		OS_SemaphoreRelease(&sem_unprov_beacon);
	}
}

static void node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{
	node_provising_addr = addr;
	OS_SemaphoreRelease(&sem_node_added);
}

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.unprovisioned_beacon = unprovisioned_beacon,
	.node_added = node_added,
};

static void cdb_local_node_add(uint8_t dev_key[16])
{
	struct bt_mesh_cdb_node *node;

	node = bt_mesh_cdb_node_alloc(prov.uuid, self_addr,
	                  comp.elem_count, net_idx);
	if (node == NULL) {
		printf("Failed to allocate database node");
		return;
	}
	memcpy(node->dev_key, dev_key, 16);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		bt_mesh_cdb_node_store(node);
	}
}

static void setup_cdb(void)
{
	struct bt_mesh_cdb_app_key *key;

	key = bt_mesh_cdb_app_key_alloc(net_idx, app_idx);
	if (key == NULL) {
		printf("Failed to allocate app-key 0x%04x\n", app_idx);
		return;
	}

	bt_rand(key->keys[0].app_key, 16);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_app_key_store(key);
	}
}

static void configure_self(struct bt_mesh_cdb_node *self)
{
	struct bt_mesh_cdb_app_key *key;
	uint8_t status = 0;
	int err, i;

	printf("Configuring self 0x%04x...\n", self->addr);

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printf("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(self->net_idx, self->addr, self->net_idx,
	                  app_idx, key->keys[0].app_key, &status);
	if (err || status) {
		printf("Failed to add app-key (err %d, status %d)\n", err,
		       status);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(root_models); i++) {
		if (root_models[i].id == BT_MESH_MODEL_ID_CFG_CLI ||
		    root_models[i].id == BT_MESH_MODEL_ID_CFG_SRV) {
			continue;
		}
		printf("Binding AppKey to model 0x%03x:%04x\n",
		           self->addr, root_models[i].id);
		err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr,
		                   app_idx, root_models[i].id,
		                   &status);
		if (err || status) {
			printf("Failed to bind app-key (err %d, status %d)\n", err,
			        status);
			return;
		}
	}
	atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(self);
	}

	printf("Configuration complete\n");
}

static void configure_node(struct bt_mesh_cdb_node *node)
{
	NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_RX_SDU_MAX);
	struct bt_mesh_cdb_app_key *key;
	uint8_t status;
	int err, elem_addr;

	printf("Configuring node 0x%04x...\n", node->addr);

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printf("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx,
	                  key->keys[0].app_key, &status);
	if (err || status) {
		printf("Failed to add app-key (err %d status %d)\n", err, status);
		return;
	}

	/* Get the node's composition data and bind all models to the appkey */
	err = bt_mesh_cfg_comp_data_get(net_idx, node->addr, 0, &status, &buf);
	if (err || status) {
		printf("Failed to get Composition data (err %d, status: %d)\n",
		        err, status);
		return;
	}

	printf("\tCID      0x%04x\n", net_buf_simple_pull_le16(&buf));
	printf("\tPID      0x%04x\n", net_buf_simple_pull_le16(&buf));
	printf("\tVID      0x%04x\n", net_buf_simple_pull_le16(&buf));
	printf("\tCRPL     0x%04x\n", net_buf_simple_pull_le16(&buf));
	printf("\tFeatures 0x%04x\n", net_buf_simple_pull_le16(&buf));

	elem_addr = node->addr;
	while (buf.len > 4) {
		uint8_t sig, vnd;
		uint16_t loc;
		int i;

		loc = net_buf_simple_pull_le16(&buf);
		sig = net_buf_simple_pull_u8(&buf);
		vnd = net_buf_simple_pull_u8(&buf);

		printf("\tElement loc 0x%04x\n", loc);

		if (buf.len < ((sig * 2U) + (vnd * 4U))) {
			printf("\t\t...truncated data!\n");
			break;
		}

		if (sig) {
			printf("\t\tSIG Models:\n");
		} else {
			printf("\t\tNo SIG Models\n");
		}

		for (i = 0; i < sig; i++) {
			uint16_t mod_id = net_buf_simple_pull_le16(&buf);
			if (mod_id == BT_MESH_MODEL_ID_CFG_CLI ||
			    mod_id == BT_MESH_MODEL_ID_CFG_SRV) {
				continue;
			}
			printf("Binding AppKey to model 0x%03x:%04x\n",
			       elem_addr, mod_id);

			err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr,
			                   elem_addr, app_idx, mod_id,
			                   &status);
			if (err || status) {
				printf("Failed (err: %d, status: %d)\n", err,
				       status);
			}
		}

		if (vnd) {
			printf("\t\tVendor Models:\n");
		} else {
			printf("\t\tNo Vendor Models\n");
		}

		for (i = 0; i < vnd; i++) {
			uint16_t cid = net_buf_simple_pull_le16(&buf);
			uint16_t mod_id = net_buf_simple_pull_le16(&buf);

			printf("Binding AppKey to model 0x%03x:%04x:%04x\n",
			       elem_addr, cid, mod_id);

			err = bt_mesh_cfg_mod_app_bind_vnd(net_idx, node->addr,
			                   elem_addr, app_idx,
			                   mod_id, cid,
			                   &status);
			if (err || status) {
				printf("Failed (err: %d, status: %d)\n", err,
				       status);
			}
		}

		elem_addr++;
	}

	atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(node);
	}

	printf("Configuration complete\n");
}

static uint8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
	if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED)) {
		if (node->addr == self_addr) {
			configure_self(node);
		} else {
			configure_node(node);
		}
	}

	return BT_MESH_CDB_ITER_CONTINUE;
}

static int cdb_node_cnt_get(void)
{
	struct bt_mesh_cdb_node *node;
	int i, total = 0;

	for (i = 0; i < ARRAY_SIZE(bt_mesh_cdb.nodes); ++i) {
		node = &bt_mesh_cdb.nodes[i];
		if (node->addr == BT_MESH_ADDR_UNASSIGNED) {
			continue;
		}
		total++;
	}

	return total;
}

static uint16_t cdb_node_addr_get(uint8_t index)
{
	struct bt_mesh_cdb_node *node;

	if (index >= ARRAY_SIZE(bt_mesh_cdb.nodes))
		return 0;

	node = &bt_mesh_cdb.nodes[index];
	return node->addr;
}

static int cdb_subnet_restore(void)
{
	int i;
	int cnt = cdb_node_cnt_get();
	if (cnt > 1) {
		for (i = 1; i < cnt; i++) {
			node_addr[i-1] = cdb_node_addr_get(i);
		}
	}

	return -1;
}

static void onoff_task(void *arg)
{
	while (1) {
		int err, i;
		uint32_t keys_cnt, node_cnt;
		uint16_t keys[16];
		static uint8_t onoff_value = 0;

		keys_cnt = ARRAY_SIZE(keys);

		node_cnt = cdb_node_cnt_get();
		onoff_value = onoff_value ? 0 : 1;

		for (i = 0; i < (node_cnt - 1) && node_addr[i] != 0; i++) {
			err = bt_mesh_cfg_net_key_get(net_idx, node_addr[i], keys, &keys_cnt);
			if (err)
				continue;
			bt_mesh_gen_onoff_cli_set(&root_models[5], node_addr[i], onoff_value, NULL);
		}
		OS_MSleep(ONOFF_CHANGE_PERIOD);
	}

	OS_ThreadDelete(&onoff_task_thread);
}

static int onoff_task_init(void)
{
	if (OS_ThreadCreate(&onoff_task_thread,
	                    "onoff_task",
	                    onoff_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    ONOFF_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}

	return 0;
}

static int prov_task_init(void)
{
	OS_SemaphoreCreate(&sem_unprov_beacon, 0, 1);
	OS_SemaphoreCreate(&sem_node_added, 0, 1);

	cdb_subnet_restore();

	/* Create onoff task */
	if (onoff_task_init() != 0) {
		OS_SemaphoreDelete(&sem_unprov_beacon);
		OS_SemaphoreDelete(&sem_node_added);
		return -1;
	}

	return 0;
}

static void prov_task_start(void)
{
	int err, cnt;
	char uuid_hex_str[32 + 1];

	while (1) {
		bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

		cnt = cdb_node_cnt_get();
		if (!cnt || (cnt && (cnt - 1) >= PROVISION_NODE_CNT_MAX))
			break;

		printf("Waiting for unprovisioned beacon...\n");
		err = OS_SemaphoreWait(&sem_unprov_beacon, K_SECONDS(10));
		if (err == OS_E_TIMEOUT) {
			continue;
		}
		node_provising = true;

		bin2hex(node_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));
		printf("Provisioning for node %s\n", uuid_hex_str);

		err = bt_mesh_provision_adv(node_uuid, net_idx, 0, 0);
		if (err < 0) {
			printf("Provisioning failed (err %d)\n", err);
			node_provising = false;
			continue;
		}

		printf("Waiting for node to be added...\n");
		err = OS_SemaphoreWait(&sem_node_added, K_SECONDS(50));
		if (err == OS_E_TIMEOUT) {
			printf("Timeout waiting for node to be added\n");
			node_provising = false;
			continue;
		}

		printf("Added node 0x%04x\n", node_provising_addr);
		node_addr[cnt - 1] = node_provising_addr;
		node_provising = false;
	}
}

/****************Init Operation**************************/
static void bt_ready(int err)
{
	uint8_t net_key[16], dev_key[16];

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

	/* Created CDB */
	bt_rand(net_key, 16);
	err = bt_mesh_cdb_create(net_key);
	if (err == -EALREADY) {
		printf("Using stored CDB\n");
	} else if (err) {
		printf("Failed to create CDB (err %d)\n", err);
		return;
	} else {
		printf("Created CDB\n");
		setup_cdb();
	}

	/* Provision the local Mesh Node */
	bt_rand(dev_key, 16);
	err = bt_mesh_provision(net_key, BT_MESH_NET_PRIMARY, 0, 0, self_addr,
				dev_key);
	if (err == -EALREADY) {
		printf("Using stored settings\n");
	} else if (err) {
		printf("Provisioning local failed (err %d)\n", err);
		return;
	} else {
		cdb_local_node_add(dev_key);
		printf("Provisioning local completed\n");
	}

	bt_mesh_cfg_cli_timeout_set(8000);

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
	err = bt_enable(NULL);
	if (err) {
		printf("main : Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	bt_ready(err);

	/* Init provision task */
	if (prov_task_init() != 0) {
		printf("prov task init fail\n");
		return -1;
	}

	/* Run mesh provision task */
	prov_task_start();

	return 0;
}
