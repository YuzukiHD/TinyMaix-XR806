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
#include "sys/xr_debug.h"
#include "fs_ctrl.h"
#include "fs/vfs.h"
#include "fs/littlefs/vfs_lfs.h"
#include "fs/fatfs/vfs_fatfs.h"
#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "fs/spiffs/vfs_spiffs.h"

#define FS_DBG_ON    0
#define FS_INF_ON    1
#define FS_WRN_ON    1
#define FS_ERR_ON    1

#define FS_LOG(flags, fmt, arg...)  \
	do {                            \
		if (flags)                  \
			printf(fmt, ##arg);     \
	} while (0)

#define FS_DBG(fmt, arg...) FS_LOG(FS_DBG_ON, "[FS DBG] "fmt, ##arg)
#define FS_INF(fmt, arg...) FS_LOG(FS_INF_ON, "[FS INF] "fmt, ##arg)
#define FS_WRN(fmt, arg...) FS_LOG(FS_WRN_ON, "[FS WRN] "fmt, ##arg)
#define FS_ERR(fmt, arg...)                         \
	do {                                            \
		FS_LOG(FS_ERR_ON, "[FS ERR] %s():%d, "fmt,  \
		       __func__, __LINE__, ##arg);          \
	} while (0)

int fs_ctrl_mount(enum fs_mnt_dev_type dev_type, uint32_t dev_id)
{
	int ret = 0;
#if PRJCONF_SYS_CTRL_EN
	enum fs_mnt_status status = FS_MNT_STATUS_MOUNT_FAIL;
#endif
#ifdef CONFIG_LITTLE_FS
	struct vfs_lfs_config config_lfs;
#elif defined(CONFIG_SPIF_FS)
	struct vfs_spiffs_config config_spiffs;
#endif

	switch (dev_type) {
#ifdef CONFIG_FAT_FS
	case FS_MNT_DEV_TYPE_SDCARD:
		ret = vfs_fatfs_mount(dev_id);
		break;
#endif
#ifdef CONFIG_LITTLE_FS
	case FS_MNT_DEV_TYPE_FLASH:
		config_lfs.start_addr = CONFIG_LITTLE_FS_START_ADDR;
		config_lfs.block_size = CONFIG_LITTLE_FS_BLOCK_SIZE;
		config_lfs.block_count = CONFIG_LITTLE_FS_BLOCK_COUNT;
		ret = vfs_lfs_mount(dev_id, &config_lfs);
		break;
#elif defined(CONFIG_SPIF_FS)
	case FS_MNT_DEV_TYPE_FLASH:
		config_spiffs.start_addr = CONFIG_SPIF_FS_START_ADDR;
		config_spiffs.block_size = CONFIG_SPIF_FS_BLOCK_SIZE;
		config_spiffs.fs_size = CONFIG_SPIF_FS_PHY_SIZE;
		ret = vfs_spiffs_mount(dev_id, &config_spiffs);
		break;
#endif
	default:
		ret = -1;
		break;
	}

#if PRJCONF_SYS_CTRL_EN
	status = (ret == 0 ? FS_MNT_STATUS_MOUNT_OK : FS_MNT_STATUS_MOUNT_FAIL);
	if (sys_event_send(CTRL_MSG_TYPE_FS,
	                   FS_CTRL_MSG_FS_MNT,
	                   FS_MNT_MSG_PARAM(dev_type, dev_id, status),
	                   0) != 0) {
		FS_ERR("send event fail\n");
	}
#endif

	return ret;
}

int fs_ctrl_unmount(enum fs_mnt_dev_type dev_type, uint32_t dev_id)
{
	int ret = 0;

	switch (dev_type) {
#ifdef CONFIG_FAT_FS
	case FS_MNT_DEV_TYPE_SDCARD:
		ret = vfs_fatfs_unmount(dev_id);
		break;
#endif
#ifdef CONFIG_LITTLE_FS
	case FS_MNT_DEV_TYPE_FLASH:
		ret = vfs_lfs_unmount(dev_id);
		break;
#elif defined(CONFIG_SPIF_FS)
	case FS_MNT_DEV_TYPE_FLASH:
		ret = vfs_spiffs_unmount(dev_id);
		break;
#endif
	default:
		ret = -1;
		break;
	}

#if PRJCONF_SYS_CTRL_EN
	if (ret == 0) {
		if (sys_event_send(CTRL_MSG_TYPE_FS,
		                   FS_CTRL_MSG_FS_MNT,
		                   FS_MNT_MSG_PARAM(dev_type, dev_id,
		                                    FS_MNT_STATUS_UNMOUNT),
		                   0) != 0) {
			FS_ERR("send event fail\n");
		}
	}
#endif

	return ret;
}

int fs_ctrl_format(enum fs_mnt_dev_type dev_type, uint32_t dev_id)
{
	int ret = 0;

	switch (dev_type) {
#ifdef CONFIG_FAT_FS
	case FS_MNT_DEV_TYPE_SDCARD:
		//TODO: add fatfs format function.
		break;
#endif
#ifdef CONFIG_LITTLE_FS
	case FS_MNT_DEV_TYPE_FLASH:
		ret = vfs_lfs_format(dev_id);
		break;
#elif defined(CONFIG_SPIF_FS)
	case FS_MNT_DEV_TYPE_FLASH:
		ret = vfs_spiffs_format(dev_id);
		break;
#endif
	default:
		ret = -1;
		break;
	}

	return ret;
}

#ifdef CONFIG_FAT_FS
#if PRJCONF_SYS_CTRL_EN

static void fs_ctrl_msg_process(uint32_t event, uint32_t data, void *arg)
{
	switch (EVENT_SUBTYPE(event)) {
	case SD_CARD_MSG_INSERT:
		fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0);
		break;
	case SD_CARD_MSG_REMOVE:
		fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0);
		break;
	default:
		break;
	}
}

#endif
#endif

/*
 * @brief Init file system control module
 * @return 0 on success, -1 on failure
 */
int fs_ctrl_init(void)
{
	int ret = 0;

	vfs_list_init();

#ifdef CONFIG_LITTLE_FS
	ret = vfs_register_lfs();
	if (ret != 0) {
		FS_ERR("register lfs fail\n");
		return -1;
	}
#elif defined(CONFIG_SPIF_FS)
	ret = vfs_register_spiffs();
	if (ret != 0) {
		FS_ERR("register spiffs fail\n");
		return -1;
	}
#endif

#ifdef CONFIG_FAT_FS
	ret = vfs_register_fatfs();
	if (ret != 0) {
		FS_ERR("register fatfs fail\n");
		return -1;
	}

#if PRJCONF_SYS_CTRL_EN
	observer_base *base = sys_callback_observer_create(CTRL_MSG_TYPE_SDCARD,
	                                                   SD_CARD_MSG_ALL,
	                                                   fs_ctrl_msg_process,
	                                                   NULL);
	if (base == NULL) {
		FS_ERR("create fail\n");
		return -1;
	}

	if (sys_ctrl_attach(base) != 0) {
		FS_ERR("attach fail\n");
		return -1;
	}
#endif
#endif

	return ret;
}

/*
 * @brief SD card detect callback fucntion
 * @param[in] present 1 for card inserted, 0 for card removed
 * @return none
 */
void sdcard_detect_callback(uint32_t present)
{
#if PRJCONF_SYS_CTRL_EN
	uint16_t subtype;

	if (present) {
		FS_INF("card insert\n");
		subtype = SD_CARD_MSG_INSERT;
	} else {
		FS_INF("card remove\n");
		subtype = SD_CARD_MSG_REMOVE;
	}

	if (sys_event_send(CTRL_MSG_TYPE_SDCARD, subtype, 0, 0) != 0) {
		FS_ERR("send event fail\n");
	}
#endif
}
