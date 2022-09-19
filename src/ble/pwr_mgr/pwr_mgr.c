/**
  * @file  pwr_mgr.c
  * @author  XRADIO IOT WLAN Team
  */

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

#include <zephyr.h>
#include <bluetooth/conn.h>


#include "host/hci_core.h"

#include "pm/pm.h"
#include "pwr_mgr.h"

#ifdef CONFIG_BT_PWR_MGR

extern int k_delayed_work_timeout_min(void);

static int k_delayed_work_is_ignored(void)
{
	uint32_t delay_min = k_delayed_work_timeout_min();
	if (delay_min == 0 || delay_min > (pm_get_suspend_resume_latency()/1000))
		return 1;
	return 0;
}

static int ble_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	//struct bt_dev *priv = dev->platform_data;
	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		if (!k_fifo_is_empty(&bt_dev.cmd_tx_queue) ||
#if !defined(CONFIG_BT_RECV_IS_RX_THREAD)
			!k_fifo_is_empty(&bt_dev.rx_queue) ||
#endif
			!k_queue_is_empty(&k_sys_work_q.queue) ||
			!k_delayed_work_is_ignored()) {
			/* can not enter PM */
			return -1;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ble_resume(struct soc_device *dev, enum suspend_state_t state)
{
	return 0;
}

static const struct soc_device_driver ble_drv = {
    .name = "ble",
    .suspend = ble_suspend,
    .resume  = ble_resume,
};

struct soc_device ble_dev = {
	.name = "ble",
	.driver = &ble_drv,
};

#ifdef CONFIG_PM_WAKELOCKS

static struct wakelock ble_wake_lock = {
	.name = "ble_wake_lock",
	.ref = 0,
};

#endif

void pwr_mgr_init(void *data)
{
	struct soc_device *priv = &ble_dev;
	priv->platform_data = data;
	pm_register_ops(priv);
}

#ifdef CONFIG_PM_WAKELOCKS

void pwr_mgr_lock(void)
{
	pm_wake_lock(&ble_wake_lock);
}

void pwr_mgr_unlock(void)
{
	pm_wake_unlock(&ble_wake_lock);
}

#endif

void pwr_mgr_deinit(void)
{
	struct soc_device *priv = &ble_dev;
	pm_unregister_ops(priv);
}

#endif
