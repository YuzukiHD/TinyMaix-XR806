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
#ifdef CONFIG_BLEHOST
#include <stdint.h>
#include "ble/drivers/bluetooth/hci_driver.h"
#include "driver/bluetooth/default/bt_ctrl_driver.h"
#include "zephyr.h"
#include <stdio.h>
#include "blec.h"
#include "kernel/os/os.h"
#include "net/buf.h"

#define H4_NONE 0x00
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04

static OS_Semaphore_t tx_sem;

static struct {
	uint8_t type;
	struct net_buf *buf;
	struct k_fifo   fifo;
} tx = {
	.fifo = Z_FIFO_INITIALIZER(tx.fifo),
};

int blec_hci_h2c_cb(unsigned char status,
                    const unsigned char *buff,
                    unsigned int offset,
                    unsigned int len)
{
	OS_SemaphoreRelease(&tx_sem);
	return 0;
}


int virtual_hci_h2c(struct net_buf *buf)
{
	unsigned char status = 0;
	unsigned char h2c_type = 0xFF;
	//struct net_buf *buf = (struct net_buf *)b;


	if (!bt_ctrl_driver_is_ready())
		return -ENODEV;

	uint8_t event_type = bt_buf_get_type(buf);


	net_buf_put(&tx.fifo, buf);
	if (!tx.buf) {
		tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
		if (!tx.buf) {
			printf("TX no pending buffer!");
			return 1;
		}
	}

	HOSTMINI_LOG("[H2C] %d ", tx.buf->len);

	switch (event_type) {
	case BT_BUF_CMD:
		h2c_type = H4_CMD; // cmd
		HOSTMINI_LOG("Opcode %02x%02x", tx.buf->data[1], tx.buf->data[0]);
		break;
	case BT_BUF_ACL_OUT:
		h2c_type = H4_ACL; // acl out
		break;
	default:
		//ASSERT(0);
		break;
	}

	HOSTMINI_LOG("\n");

#if CONFIG_BT_DEBUG_LOG_WITH_HCI_PRINT
	int i;
	printf("[h2c] data : ");
	for (i = 0; i < tx.buf->len; i++)
		printf("0x%02x ", *(tx.buf->data+i));
	printf("\n");
#endif

	if ((h2c_type == 1) || (h2c_type == 2)) {
#ifdef CONFIG_BTSNOOP
		void btsnoop_capture(uint8_t type, const uint8_t *buf, bool is_received);
		btsnoop_capture(h2c_type, tx.buf->data, 0);
#endif
		status = blec_hci_h2c(h2c_type, tx.buf->data, 0, tx.buf->len);
		if (status != 0) {
			printf("h2c err %d %d!\n", event_type, status);
		}

		if (OS_SemaphoreWait(&tx_sem, 5000) != OS_OK)
			printf("h2c cb timeout\n");

		net_buf_unref(tx.buf);
		tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
	} else {
		printf("h2c err h2c_type %d!\n", h2c_type);
	}

	return 0;
}

static struct net_buf *get_rx(uint8_t id, const uint8_t *buf)
{
	if (id == H4_EVT && (buf[0] == BT_HCI_EVT_CMD_COMPLETE ||
	                     buf[0] == BT_HCI_EVT_CMD_STATUS)) {
		struct net_buf *cmd_cpl = bt_buf_get_cmd_complete(K_FOREVER);
		net_buf_reset(cmd_cpl);
		return cmd_cpl;
	}
	if (id == H4_ACL) {
		return bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
	} else {
		return bt_buf_get_rx(BT_BUF_EVT, K_FOREVER);
	}
}


int blec_hci_c2h(unsigned char hci_type, const unsigned char *buff, unsigned int offset, unsigned int len)
{
	struct net_buf *tmp_buf = NULL;
	uint8_t *origin_buff = (uint8_t *)buff;
	buff += offset;

	if ((hci_type != H4_ACL) && (hci_type != H4_EVT)) {
		printf("hci c2h hci type errr: %d\n", hci_type);
		return 1;
	}

	tmp_buf = get_rx(hci_type, buff);
	if (tmp_buf == NULL) {
		blec_hci_c2h_cb(1, origin_buff, offset, len);
		return 2;
	}
	net_buf_add_mem(tmp_buf, buff, len);
#if CONFIG_BT_DEBUG_LOG_WITH_HCI_PRINT
	int i;
	printf("[c2h](%d, %d) : ", tmp_buf->len, len);
	for (i = 0; i < tmp_buf->len; i++)
		printf("0x%02x ", *(tmp_buf->data+i));
	printf("\n");
#endif

#ifdef CONFIG_BTSNOOP
	void btsnoop_capture(uint8_t type, const uint8_t *buf, bool is_received);
	btsnoop_capture(hci_type, tmp_buf->data, 1);
#endif

//	if (hci_type == H4_EVT
//	    && (BT_HCI_EVT_FLAG_RECV_PRIO & bt_hci_evt_get_flags(buff[0]))) {
//		bt_recv_prio(tmp_buf);
//		printf("c2h prio had return\n");
//	} else {
		bt_recv(tmp_buf);
#if CONFIG_BT_DEBUG_HCI_DRIVER
		printf("bt_recv had return\n");
#endif
//	}

	blec_hci_c2h_cb(0, origin_buff, offset, len);

	return 0;
}

static blec_hci_t ble_hci_hst = {
	.blec_hci_c2h = blec_hci_c2h,
	.blec_hci_h2c_cb = blec_hci_h2c_cb,
};

int virtual_hci_open(void)
{
	int ret;

	if (!OS_SemaphoreIsValid(&tx_sem))
		OS_SemaphoreCreate(&tx_sem, 0, OS_SEMAPHORE_MAX_COUNT);

	if ((ret = blec_hci_init(&ble_hci_hst)) != 0) {
		printf("blec_hci_init failed(%d)\n", ret);

		if (ret == 1)
			return -EALREADY;
		else if (ret == 2)
			return -EINVAL;
		else
			return -EPERM;
	}

	return 0;
}

static const struct bt_hci_driver drv = {
	.name        = "",
	.bus         = BT_HCI_DRIVER_BUS_VIRTUAL,
	.open        = virtual_hci_open,
	.send        = virtual_hci_h2c,
};

const struct bt_hci_driver *bt_ctrl_get_zephyr_interface(void)
{
	return &drv;
}

#endif /* CONFIG_BLEHOST */
