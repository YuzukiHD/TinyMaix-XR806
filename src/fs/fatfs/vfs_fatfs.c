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
#include <stdlib.h>
#include <string.h>
#include "sys/defs.h"
#include "fs/vfs.h"
#include "fs/fatfs/ff.h"
#include "fs/fatfs/vfs_fatfs.h"
#include "driver/chip/sdmmc/sdmmc.h"
#include "sys/xr_debug.h"

typedef struct {
	OS_Mutex_t lock;
	FATFS *fs;
} fatfs_manager_t;

typedef struct {
	vfs_file_t base;
	FIL file;
} vfs_fatfs_file;

typedef struct {
	vfs_file_t base;
	DIR dir;
} vfs_fatfs_dir;

static fatfs_manager_t fatfs_manager;

static int fatfs_init(void)
{
	int ret;

	if (OS_MutexIsValid(&fatfs_manager.lock)) {
		return 0;
	}

	ret = OS_MutexCreate(&fatfs_manager.lock);
	if (ret != OS_OK) {
		return -1;
	}

	return 0;
}

static int fatfs_deinit(void)
{
	if (!OS_MutexIsValid(&fatfs_manager.lock)) {
		return -1;
	}

	OS_MutexDelete(&fatfs_manager.lock);
	memset(&fatfs_manager, 0, sizeof(fatfs_manager_t));

	return 0;
}

int vfs_fatfs_mount(uint32_t dev_id)
{
	int ret;
	FATFS *fs;
	FRESULT fs_ret;
	struct mmc_card *card;
	SDCard_InitTypeDef card_param = { 0 };

	ret = fatfs_init();
	if (ret != 0) {
		return -1;
	}

	OS_MutexLock(&fatfs_manager.lock, OS_WAIT_FOREVER);

	if (fatfs_manager.fs != NULL) {
		ret = 0;
		goto out; /* already mounted, nothing to do */
	}

	card_param.debug_mask = ROM_WRN_MASK | ROM_ERR_MASK | ROM_ANY_MASK;
	card_param.type = MMC_TYPE_SD; /* define to speed up scan card */
	ret = mmc_card_create(dev_id, &card_param);
	if (ret != 0) {
		VFS_ERR("mmc create fail\n");
		ret = -1;
		goto out;
	}

	card = mmc_card_open(dev_id);
	if (card == NULL) {
		VFS_ERR("mmc open fail\n");
		ret = -1;
		goto err_card;
	}
	if (!mmc_card_present(card)) {
		int mmc_ret = mmc_rescan(card, 0);
		if (mmc_ret != 0) {
			VFS_ERR("mmc scan fail\n");
			mmc_card_close(dev_id);
			ret = -1;
			goto err_card;
		} else {
			VFS_INF("mmc init\n");
		}
	}
	mmc_card_close(dev_id);

	fs = malloc(sizeof(FATFS));
	if (fs == NULL) {
		VFS_ERR("no mem\n");
		ret = -1;
		goto err_fs;
	}
	fs_ret = f_mount(fs, "", 0);
	if (fs_ret != FR_OK) {
		VFS_ERR("mount fail, err %d\n", fs_ret);
		free(fs);
		ret = -1;
		goto err_fs;
	}

	VFS_INF("mount success\n");
	fatfs_manager.fs = fs;
	ret = 0;

	goto out;

err_fs:
	if (mmc_card_present(card)) {
		mmc_card_deinit(card);
	}

err_card:
	mmc_card_delete(dev_id);

out:
	OS_MutexUnlock(&fatfs_manager.lock);
	return ret;
}


int vfs_fatfs_unmount(uint32_t dev_id)
{
	int ret = 0;
	FRESULT fs_ret;
	struct mmc_card *card;

	OS_MutexLock(&fatfs_manager.lock, OS_WAIT_FOREVER);

	if (fatfs_manager.fs == NULL) {
		goto out; /* unmounted, nothing to do */
	}

	fs_ret = f_mount(NULL, "", 0);
	if (fs_ret != FR_OK) {
		VFS_ERR("unmount fail, err %d\n", fs_ret);
		ret = -1;
	} else {
		free(fatfs_manager.fs);
		fatfs_manager.fs = NULL;
	}

	card = mmc_card_open(dev_id);
	if (card == NULL) {
		VFS_ERR("card open fail\n");
	} else {
		if (mmc_card_present(card)) {
			mmc_card_deinit(card);
		}
		mmc_card_close(dev_id);
		mmc_card_delete(dev_id);
	}

out:
	OS_MutexUnlock(&fatfs_manager.lock);
	fatfs_deinit();
	return ret;
}

static int fatfs_mode_convert(int mode)
{
	int flags = 0;
	int acc_mode;

	acc_mode = mode & VFS_ACCMODE;
	if (acc_mode == VFS_RDONLY) {
		flags |= FA_READ;
	} else if (acc_mode == VFS_WRONLY) {
		flags |= FA_WRITE;
	} else if (acc_mode == VFS_RDWR) {
		flags |= (FA_READ | FA_WRITE);
	}

	if (mode & VFS_CREAT) {
		flags |= FA_OPEN_ALWAYS;
	}
	if (mode & VFS_EXCL) {
		flags |= FA_CREATE_NEW;
	}
	if (mode & VFS_TRUNC) {
		flags |= FA_CREATE_ALWAYS;
	}
	if (mode & VFS_APPEND) {
		flags |= FA_OPEN_APPEND;
	}
	return flags;
}

static int fatfs_ret_convert(int fatfs_ret)
{
	int ret;

	switch (fatfs_ret) {
	case FR_OK:
		ret = 0;
		break;
	case FR_DISK_ERR:
		ret = -EIO;
		break;
	case FR_INT_ERR:
		ret = -EIO;
		break;
	case FR_NOT_READY:
		ret = -ENODEV;
		break;
	case FR_NO_FILE:
		ret = -ENOENT;
		break;
	case FR_NO_PATH:
		ret = -ENOENT;
		break;
	case FR_INVALID_NAME:
		ret = -EINVAL;
		break;
	case FR_DENIED:
		ret = -EACCES;
		break;
	case FR_EXIST:
		ret = -EEXIST;
		break;
	case FR_INVALID_OBJECT:
		ret = -EBADF;
		break;
	case FR_WRITE_PROTECTED:
		ret = -EACCES;
		break;
	case FR_INVALID_DRIVE:
		ret = -ENXIO;
		break;
	case FR_NOT_ENABLED:
		ret = -ENODEV;
		break;
	case FR_NO_FILESYSTEM:
		ret = -ENODEV;
		break;
	case FR_MKFS_ABORTED:
		ret = -EINTR;
		break;
	case FR_TIMEOUT:
		ret = -ETIMEDOUT;
		break;
	case FR_LOCKED:
		ret = -EACCES;
		break;
	case FR_NOT_ENOUGH_CORE:
		ret = -ENOMEM;
		break;
	case FR_TOO_MANY_OPEN_FILES:
		ret = -ENFILE;
		break;
	case FR_INVALID_PARAMETER:
		ret = -EINVAL;
		break;
	default:
		ret = fatfs_ret;
		break;
	}
	return ret;
}

static int vfs_fatfs_open(vfs_file_t *vfs, const char *path, int mode)
{
	FRESULT ret;
	int flags;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	flags = fatfs_mode_convert(mode);
	ret = f_open(&impl->file, path, flags);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_close(vfs_file_t *vfs)
{
	FRESULT ret;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_close(&impl->file);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_read(vfs_file_t *vfs, void *buffer, unsigned int len)
{
	FRESULT ret;
	unsigned int read_size;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_read(&impl->file, buffer, len, &read_size);
	if (ret == FR_OK) {
		return read_size;
	}

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_write(vfs_file_t *vfs, const void *buffer,
                           unsigned int len)
{
	FRESULT ret;
	unsigned int write_size;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_write(&impl->file, buffer, len, &write_size);
	if (ret == FR_OK) {
		return write_size;
	}

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_seek(vfs_file_t *vfs, int64_t offset, int whence)
{
	int64_t cur_pos;
	int64_t new_pos;
	int64_t size;
	FRESULT ret;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	if (whence == SEEK_SET) {
		new_pos = offset;
	} else if (whence == SEEK_CUR) {
		cur_pos = f_tell(&impl->file);
		new_pos = cur_pos + offset;
	} else if (whence == SEEK_END) {
		size    = f_size(&impl->file);
		new_pos = size + offset;
	} else {
		return fatfs_ret_convert(FR_INVALID_PARAMETER);
	}
	ret = f_lseek(&impl->file, new_pos);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_sync(vfs_file_t *vfs)
{
	FRESULT ret;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_sync(&impl->file);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_stat(const char *path, struct vfs_info *info)
{
	FRESULT ret;
	FILINFO f_info;

	memset(info, 0, sizeof(struct vfs_info));
	ret = f_stat(path, &f_info);
	if (ret != FR_OK) {
		return fatfs_ret_convert(ret);
	}
	if (f_info.fattrib & AM_DIR) {
		info->type = VFS_TYPE_DIR;
	} else {
		info->type = VFS_TYPE_FILE;
		info->size = f_info.fsize;
	}

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_unlink(const char *path)
{
	FRESULT ret;

	ret = f_unlink(path);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_rename(const char *old_name, const char *new_name)
{
	FRESULT ret;

	ret = f_rename(old_name, new_name);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_tell(vfs_file_t *vfs)
{
	int ret;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_tell(&impl->file);

	return ret;
}

static int vfs_fatfs_size(vfs_file_t *vfs)
{
	int ret;
	vfs_fatfs_file *impl;

	impl = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);

	ret = f_size(&impl->file);

	return ret;
}

static int vfs_fatfs_opendir(vfs_file_t *vfs, const char *path)
{
	FRESULT ret;
	vfs_fatfs_dir *impl;

	impl = (vfs_fatfs_dir *)container_of(vfs, vfs_fatfs_dir, base);

	ret = f_opendir(&impl->dir, path);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_closedir(vfs_file_t *vfs)
{
	FRESULT ret;
	vfs_fatfs_dir *impl;

	impl = (vfs_fatfs_dir *)container_of(vfs, vfs_fatfs_dir, base);

	ret = f_closedir(&impl->dir);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_readdir(vfs_file_t *vfs, struct vfs_info *info)
{
	FRESULT ret;
	vfs_fatfs_dir *impl;
	FILINFO f_info;

	memset(info, 0, sizeof(struct vfs_info));
	impl = (vfs_fatfs_dir *)container_of(vfs, vfs_fatfs_dir, base);
	ret = f_readdir(&impl->dir, &f_info);
	if (ret != FR_OK) {
		return fatfs_ret_convert(ret);
	}
	if (f_info.fname[0] != 0) {
		if (f_info.fattrib & AM_DIR) {
			info->type = VFS_TYPE_DIR;
		} else {
			info->type = VFS_TYPE_FILE;
			info->size = f_info.fsize;
		}
		strlcpy(info->name, f_info.fname, sizeof(info->name));
	}

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_mkdir(const char *path)
{
	FRESULT ret;

	ret = f_mkdir(path);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_rmdir(const char *path)
{
	FRESULT ret;

	ret = f_rmdir(path);

	return fatfs_ret_convert(ret);
}

static int vfs_fatfs_release(vfs_file_t *vfs, VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_fatfs_file *impl_file;
	vfs_fatfs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = vfs;
		free(impl_none);
		break;
	case VFS_PATH_FILE:
		impl_file = (vfs_fatfs_file *)container_of(vfs, vfs_fatfs_file, base);
		free(impl_file);
		break;
	case VFS_PATH_DIR:
		impl_dir = (vfs_fatfs_dir *)container_of(vfs, vfs_fatfs_dir, base);
		free(impl_dir);
		break;
	default:
		break;
	}

	return 0;
}

static const struct vfs_file_ops fatfs_ops = {
	.vfs_open     = vfs_fatfs_open,
	.vfs_close    = vfs_fatfs_close,
	.vfs_read     = vfs_fatfs_read,
	.vfs_write    = vfs_fatfs_write,
	.vfs_seek     = vfs_fatfs_seek,
	.vfs_sync     = vfs_fatfs_sync,
	.vfs_stat     = vfs_fatfs_stat,
	.vfs_unlink   = vfs_fatfs_unlink,
	.vfs_rename   = vfs_fatfs_rename,
	.vfs_tell     = vfs_fatfs_tell,
	.vfs_size     = vfs_fatfs_size,
	.vfs_opendir  = vfs_fatfs_opendir,
	.vfs_closedir = vfs_fatfs_closedir,
	.vfs_readdir  = vfs_fatfs_readdir,
	.vfs_mkdir    = vfs_fatfs_mkdir,
	.vfs_rmdir    = vfs_fatfs_rmdir,
	.vfs_release  = vfs_fatfs_release,
};

static vfs_file_t *vfs_fatfs_create(VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_fatfs_file *impl_file;
	vfs_fatfs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = malloc(sizeof(vfs_file_t));
		if (impl_none == NULL) {
			return NULL;
		}
		memset(impl_none, 0, sizeof(vfs_file_t));
		impl_none->ops = &fatfs_ops;
		return impl_none;
	case VFS_PATH_FILE:
		impl_file = malloc(sizeof(vfs_fatfs_file));
		if (impl_file == NULL) {
			return NULL;
		}
		memset(impl_file, 0, sizeof(vfs_fatfs_file));
		impl_file->base.ops = &fatfs_ops;
		return &impl_file->base;
	case VFS_PATH_DIR:
		impl_dir = malloc(sizeof(vfs_fatfs_dir));
		if (impl_dir == NULL) {
			return NULL;
		}
		memset(impl_dir, 0, sizeof(vfs_fatfs_dir));
		impl_dir->base.ops = &fatfs_ops;
		return &impl_dir->base;
	default:
		return NULL;
	}
}

static const vfs_creator_t vfs_fatfs_ctor = {
	.create = vfs_fatfs_create,
};

int vfs_register_fatfs(void)
{
	return vfs_register(&vfs_fatfs_ctor, "sdcard");
}

