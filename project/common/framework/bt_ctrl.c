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
#ifdef CONFIG_BLE_FEATURE

#include <stdio.h>
#include <string.h>
#include "errno.h"
#include "efpg/efpg.h"
#include "bt_ctrl.h"
#include "common/framework/sysinfo.h"
#include "driver/bluetooth/default/bt_ctrl_driver.h"
#ifdef CONFIG_BLEHOST
#include "ble/drivers/bluetooth/hci_driver.h"
#include "driver/bluetooth/default/bt_zephyr_adapter.h"
#endif /* CONFIG_BLEHOST */

#define BT_CTRL_ERR(fmt, arg...)  printf("E " fmt, ##arg)
#define BT_CTRL_WARN(fmt, arg...) printf("W " fmt, ##arg)

__WEAK void bt_public_addr_modify(uint8_t *addr)
{
	addr[5]++;

	/* Be careful about BREDR public address rule:
	 *      1. addr[2] should not be 0x9E.
	 *      2. only LAP taken effect.
	 * while may cause same address even thought the address look different.
	 */
}

static inline void addr_big_to_little_end(uint8_t *addr)
{
	uint8_t revert[6];
	memcpy(revert, addr, 6);

	addr[5] = revert[0];
	addr[4] = revert[1];
	addr[3] = revert[2];
	addr[2] = revert[3];
	addr[1] = revert[4];
	addr[0] = revert[5];
}

static int bt_get_public_addr(uint8_t *addr)
{
	switch (PRJCONF_BT_PUBLIC_ADDR_SOURCE) {
	case BT_PUBLIC_ADDR_BY_EFUSE:
		if (efpg_read_mac(EFPG_MAC_BT, addr) != EFPG_ACK_OK) {
			BT_CTRL_WARN("No Efuse BT Address.\n");
			return -EINVAL;
		}
		break;
	case BT_PUBLIC_ADDR_BY_WLAN:
#if PRJCONF_NET_EN
		{
			struct sysinfo *sysinfo = sysinfo_get();
			if (sysinfo == NULL) {
				BT_CTRL_WARN("failed to get sysinfo %p\n", sysinfo);
				return -EINVAL;
			}

			/* bt address is little ending, but wifi address big ending */
			memcpy(addr, sysinfo->mac_addr, 6);

			bt_public_addr_modify(addr);
			break;
		}
#else
#warning "Net has already disable, no mac address."
		BT_CTRL_WARN("Wlan disabled, default no public address.\n");
#endif
	case BT_PUBLIC_ADDR_NONE:
	default:
		memset(addr, 0, 6);
		break;
	}

	addr_big_to_little_end(addr);

	return 0;
}

#ifdef CONFIG_BLEHOST
static int bt_zephyr_adapter_register(void)
{
	const struct bt_hci_driver *drv = bt_ctrl_get_zephyr_interface();

	int ret = bt_hci_driver_register(drv);

	if (!(ret == -EALREADY || ret == 0))
		return -EINVAL;

	return 0;
}

static int bt_zephyr_adapter_unregister(void)
{
	return 0;
}

static inline int bt_adapter_register(void)
{
	return bt_zephyr_adapter_register();
}

static inline int bt_adapter_unregister(void)
{
	return bt_zephyr_adapter_unregister();
}
#endif /* CONFIG_BLEHOST */

static int bt_controller_init(void)
{
	int ret = 0;
	uint8_t addr[6];

	bt_get_public_addr(addr);

	if (bt_ctrl_driver_init(addr) != 0)
		ret = -EINVAL;

	return ret;
}

static int bt_controller_deinit(void)
{
	bt_ctrl_driver_deinit();
	return 0;
}

int bt_ctrl_enable(void)
{
	int ret;

	ret = bt_controller_init();
	if (ret != 0) {
		BT_CTRL_ERR("bt_controller_init failed(%d)\n", ret);
		return ret;
	}

#ifdef CONFIG_BLEHOST
	ret = bt_adapter_register();
	if (ret != 0) {
		BT_CTRL_ERR("bt_adapter_register failed(%d)\n", ret);
		bt_controller_deinit();
	}
#endif

	return ret;
}

int bt_ctrl_disable(void)
{
#ifdef CONFIG_BLEHOST
	bt_adapter_unregister();
#endif

	bt_controller_deinit();

	return 0;
}
#endif /* CONFIG_BLE_FEATURE */