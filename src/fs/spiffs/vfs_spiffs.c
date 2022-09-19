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
#include "kernel/os/os_mutex.h"
#include "fs/vfs.h"
#include "image/flash.h"
#include "fs/spiffs/spiffs_config.h"
#include "fs/spiffs/spiffs.h"
#include "spiffs_nucleus.h"
#include "fs/spiffs/vfs_spiffs.h"

#define SPIFFS_DEFAULT_FLASH   0

#define _SPIFFS_MANAGER_CHECK(spiffs_mgr)       \
	do {                                        \
		if (NULL == spiffs_mgr) {               \
			return -ENODEV;                     \
		}                                       \
	} while (0)

typedef OS_Mutex_t spiffs_lock_t;

typedef struct {
	spiffs_lock_t lock;
	spiffs spif_fs;
	spiffs_config cfg;
	uint8_t *work_buf;
	uint32_t work_buf_sz;
	uint8_t *fd_buf;
	uint32_t fd_buf_sz;
	uint8_t *cache_buf;
	uint32_t cache_buf_sz;
} spiffs_manager_t;

typedef struct {
	vfs_file_t base;
	spiffs_file file;
} vfs_spiffs_file;

typedef struct {
	vfs_file_t base;
	spiffs_DIR dir;
} vfs_spiffs_dir;

static spiffs_manager_t *spiffs_manager = NULL;

static inline int spiffs_lock_create(spiffs_lock_t *lock)
{
	if (OS_MutexCreate(lock) != OS_OK) {
		return -1;
	}

	return 0;
}

static inline void spiffs_lock_destory(spiffs_lock_t *lock)
{
	OS_MutexDelete(lock);
}

static inline void spiffs_lock(spiffs_lock_t *lock)
{
	OS_MutexLock(lock, OS_WAIT_FOREVER);
}

static inline void spiffs_unlock(spiffs_lock_t *lock)
{
	OS_MutexUnlock(lock);
}

s32_t spiffs_spi_flash_read(uint32_t read_addr, uint32_t read_size,
                            uint8_t *buf)
{
	s32_t status;

	status = flash_read(SPIFFS_DEFAULT_FLASH, read_addr, buf, read_size);

	return status < 0 ? status : 0;
}

s32_t spiffs_spi_flash_write(uint32_t write_addr, uint32_t write_size,
                             uint8_t *buf)
{
	s32_t status;

	status = flash_write(SPIFFS_DEFAULT_FLASH, write_addr, buf, write_size);

	return status < 0 ? status : 0;
}

s32_t spiffs_spi_flash_erase(uint32_t addr, uint32_t size)
{
	s32_t status;

	status = flash_erase(SPIFFS_DEFAULT_FLASH, addr, size);

	return status < 0 ? status : 0;
}

static void _spiffs_buf_free(void)
{
	if (!spiffs_manager) {
		return;
	}

	if (spiffs_manager->cache_buf) {
		free(spiffs_manager->cache_buf);
		spiffs_manager->cache_buf = NULL;
	}

	if (spiffs_manager->fd_buf) {
		free(spiffs_manager->fd_buf);
		spiffs_manager->fd_buf = NULL;
	}

	if (spiffs_manager->work_buf) {
		free(spiffs_manager->work_buf);
		spiffs_manager->work_buf = NULL;
	}

	free(spiffs_manager);
	spiffs_manager = NULL;
}

static int spiffs_deinit(void)
{
	if (spiffs_manager == NULL) {
		return 0;
	}

	spiffs_lock_destory(&spiffs_manager->lock);

	_spiffs_buf_free();

	return 0;
}

static int spiffs_init(struct vfs_spiffs_config *config)
{
	if (spiffs_manager != NULL) {
		return -1;
	}

	spiffs_manager = (spiffs_manager_t *)malloc(sizeof(spiffs_manager_t));
	if (!spiffs_manager) {
		VFS_ERR("no spiffs_manager mem\n");
		return -1;
	}
	memset(spiffs_manager, 0, sizeof(spiffs_manager_t));

	if ((spiffs_lock_create(&spiffs_manager->lock)) != OS_OK) {
		VFS_ERR("spiffs_lock_create err\n");
		goto err_lock;
	}

	spiffs_manager->work_buf_sz  = SPIFLASH_CFG_LOG_PAGE_SZ * 2;
	spiffs_manager->fd_buf_sz    = sizeof(spiffs_fd) * SPIFLASH_CFG_MAX_OPEN_FILES;
	spiffs_manager->cache_buf_sz = sizeof(spiffs_cache) + SPIFLASH_CFG_MAX_OPEN_FILES *
	                               (sizeof(spiffs_cache_page) + SPIFLASH_CFG_LOG_PAGE_SZ);

	spiffs_manager->work_buf  = (uint8_t *)malloc(spiffs_manager->work_buf_sz);
	spiffs_manager->fd_buf    = (uint8_t *)malloc(spiffs_manager->fd_buf_sz);
	spiffs_manager->cache_buf = (uint8_t *)malloc(spiffs_manager->cache_buf_sz);

	if ((spiffs_manager->work_buf == NULL) || (spiffs_manager->fd_buf == NULL) ||
	    (spiffs_manager->cache_buf == NULL)) {
		VFS_ERR("no spiffs_buf mem\n");
		goto err_buf;
	}

	memset(spiffs_manager->work_buf, 0, spiffs_manager->work_buf_sz);
	memset(spiffs_manager->fd_buf, 0, spiffs_manager->fd_buf_sz);
	memset(spiffs_manager->cache_buf, 0, spiffs_manager->cache_buf_sz);

	spiffs_manager->cfg.phys_size        = config->fs_size;
	spiffs_manager->cfg.phys_addr        = config->start_addr;
	spiffs_manager->cfg.phys_erase_block = config->block_size;
	spiffs_manager->cfg.log_block_size   = config->block_size;
	spiffs_manager->cfg.log_page_size    = SPIFLASH_CFG_LOG_PAGE_SZ;

	spiffs_manager->cfg.hal_read_f  = spiffs_spi_flash_read;
	spiffs_manager->cfg.hal_write_f = spiffs_spi_flash_write;
	spiffs_manager->cfg.hal_erase_f = spiffs_spi_flash_erase;

	return 0;

err_lock:
	_spiffs_buf_free();
	return -1;

err_buf:
	spiffs_deinit();
	return -1;
}

int vfs_spiffs_mount(uint32_t dev_id, struct vfs_spiffs_config *config)
{
	s32_t ret;

	ret = spiffs_init(config);
	if (ret != SPIFFS_OK) {
		return ret;
	}

	ret = SPIFFS_mount(&spiffs_manager->spif_fs, &spiffs_manager->cfg,
	                   spiffs_manager->work_buf, spiffs_manager->fd_buf, spiffs_manager->fd_buf_sz,
	                   spiffs_manager->cache_buf, spiffs_manager->fd_buf_sz, 0);

	if (ret == SPIFFS_ERR_NOT_A_FS) {
		VFS_INF("SPIFFS formating...\n");

		if (SPIFFS_format(&spiffs_manager->spif_fs) == SPIFFS_OK) {
			ret = SPIFFS_mount(&spiffs_manager->spif_fs, &spiffs_manager->cfg,
			                   spiffs_manager->work_buf, spiffs_manager->fd_buf, spiffs_manager->fd_buf_sz,
			                   spiffs_manager->cache_buf, spiffs_manager->fd_buf_sz, 0);
		} else {
			VFS_ERR("SPIFFS format err\n");
			goto err_mount;
		}
	}

	if (ret != SPIFFS_OK) {
		VFS_ERR("SPIFFS mount fail.\n");
		goto err_mount;
	}
	VFS_INF("SPIFFS mount success.\n");
	return ret;

err_mount:
	spiffs_deinit();
	return ret;
}

int vfs_spiffs_unmount(uint32_t dev_id)
{
	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	SPIFFS_unmount(&spiffs_manager->spif_fs);

	spiffs_deinit();

	return 0;
}

int vfs_spiffs_format(uint32_t dev_id)
{
	int ret;

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	SPIFFS_unmount(&spiffs_manager->spif_fs);
	ret = SPIFFS_format(&spiffs_manager->spif_fs);

	spiffs_deinit();

	return ret;
}

static int spiffs_mode_convert(int mode)
{
	int flags = 0;
	int acc_mode;

	acc_mode = mode & VFS_ACCMODE;
	if (acc_mode == VFS_RDONLY) {
		flags |= SPIFFS_O_RDONLY;
	} else if (acc_mode == VFS_WRONLY) {
		flags |= SPIFFS_O_WRONLY;
	} else if (acc_mode == VFS_RDWR) {
		flags |= SPIFFS_O_RDWR;
	}

	if (mode & VFS_CREAT) {
		flags |= SPIFFS_O_CREAT;
	}
	if (mode & VFS_EXCL) {
		flags |= SPIFFS_O_EXCL;
	}
	if (mode & VFS_TRUNC) {
		flags |= SPIFFS_O_TRUNC;
	}
	if (mode & VFS_APPEND) {
		flags |= SPIFFS_O_APPEND;
	}
	return flags;
}

static int spiffs_ret_convert(int spiffs_ret)
{
	int ret;

	if (spiffs_ret > 0) {
		return spiffs_ret;
	}

	switch (spiffs_ret) {
	case SPIFFS_OK:
		return 0;
	case SPIFFS_ERR_NOT_WRITABLE:
	case SPIFFS_ERR_NOT_READABLE:
	case SPIFFS_ERR_NOT_CONFIGURED:
		ret = -EACCES;
		break;
	case SPIFFS_ERR_NOT_MOUNTED:
	case SPIFFS_ERR_NOT_A_FS:
	case SPIFFS_ERR_PROBE_NOT_A_FS:
	case SPIFFS_ERR_MAGIC_NOT_POSSIBLE:
		ret = -ENODEV;
		break;
	case SPIFFS_ERR_FULL:
	case SPIFFS_ERR_PROBE_TOO_FEW_BLOCKS:
		ret = -ENOSPC;
		break;
	case SPIFFS_ERR_BAD_DESCRIPTOR:
	case SPIFFS_ERR_OUT_OF_FILE_DESCS:
		ret = -EBADF;
		break;
	case SPIFFS_ERR_MOUNTED:
	case SPIFFS_ERR_FILE_EXISTS:
		ret = -EEXIST;
		break;
	case SPIFFS_ERR_NOT_FOUND:
	case SPIFFS_ERR_NOT_A_FILE:
	case SPIFFS_ERR_DELETED:
	case SPIFFS_ERR_FILE_DELETED:
	case SPIFFS_ERR_NOT_FINALIZED:
	case SPIFFS_ERR_NOT_INDEX:
	case SPIFFS_ERR_IS_FREE:
	case SPIFFS_ERR_INDEX_SPAN_MISMATCH:
	case SPIFFS_ERR_FILE_CLOSED:
		ret = -ENOENT;
		break;
	case SPIFFS_ERR_NAME_TOO_LONG:
		ret = -ENAMETOOLONG;
		break;
	case SPIFFS_ERR_RO_NOT_IMPL:
	case SPIFFS_ERR_RO_ABORTED_OPERATION:
		ret = -EROFS;
		break;
	case SPIFFS_ERR_SEEK_BOUNDS:
		ret = -EOVERFLOW;
		break;
	case SPIFFS_ERR_END_OF_OBJECT:
	case SPIFFS_ERR_NO_DELETED_BLOCKS:
		ret = -ENODATA;
		break;
	case SPIFFS_ERR_ERASE_FAIL:
		ret = -EIO;
		break;
	default:
		ret = spiffs_ret;
		break;
	}

	return ret;
}

static int vfs_spiffs_open(vfs_file_t *vfs, const char *path, int mode)
{
	s32_t ret;
	int flags;
	vfs_spiffs_file *impl;

	if (strlen(path) > SPIFFS_OBJ_NAME_LEN - 1) {
		VFS_ERR("file name too long, max len is %d.\n", SPIFFS_OBJ_NAME_LEN - 1);
		return -EINVAL;
	}

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);
	flags = spiffs_mode_convert(mode);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	impl->file = SPIFFS_open(&spiffs_manager->spif_fs, path, flags, 0);
	spiffs_unlock(&spiffs_manager->lock);

	if (impl->file < 0) {
		ret = SPIFFS_errno(&spiffs_manager->spif_fs);
	} else {
		ret = 0;
	}

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_close(vfs_file_t *vfs)
{
	s32_t ret;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_close(&spiffs_manager->spif_fs, impl->file);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_read(vfs_file_t *vfs, void *buffer, unsigned int len)
{
	s32_t read_size;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	read_size = SPIFFS_read(&spiffs_manager->spif_fs, impl->file, buffer, len);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(read_size);
}

static int vfs_spiffs_write(vfs_file_t *vfs, const void *buffer,
                            unsigned int len)
{
	s32_t write_size;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	write_size = SPIFFS_write(&spiffs_manager->spif_fs, impl->file, (void *)buffer,
	                          len);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(write_size);
}

static int vfs_spiffs_seek(vfs_file_t *vfs, int64_t offset, int whence)
{
	s32_t ret;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	if (whence == SEEK_SET) {
		whence = SPIFFS_SEEK_SET;
	} else if (whence == SEEK_CUR) {
		whence = SPIFFS_SEEK_CUR;
	} else if (whence == SEEK_END) {
		whence = SPIFFS_SEEK_END;
	} else {
		return -EINVAL;
	}
	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_lseek(&spiffs_manager->spif_fs, impl->file, offset, whence);
	spiffs_unlock(&spiffs_manager->lock);

	if (ret < 0) {
		ret = SPIFFS_errno(&spiffs_manager->spif_fs);
	} else {
		ret = 0;
	}

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_sync(vfs_file_t *vfs)
{
	s32_t ret;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_fflush(&spiffs_manager->spif_fs, impl->file);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_stat(const char *path, struct vfs_info *info)
{
	s32_t ret;
	spiffs_stat s_info;

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_stat(&spiffs_manager->spif_fs, path, &s_info);
	spiffs_unlock(&spiffs_manager->lock);

	if (ret < 0) {
		return spiffs_ret_convert(ret);
	}

	memset(info, 0, sizeof(struct vfs_info));
	if (s_info.type == SPIFFS_TYPE_DIR) {
		info->type = VFS_TYPE_DIR;
	} else {
		info->size = s_info.size;
		info->type = VFS_TYPE_FILE;
	}

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_unlink(const char *path)
{
	s32_t ret;

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_remove(&spiffs_manager->spif_fs, path);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_rename(const char *old_name, const char *new_name)
{
	s32_t ret;

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_rename(&spiffs_manager->spif_fs, old_name, new_name);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_tell(vfs_file_t *vfs)
{
	s32_t ret;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_tell(&spiffs_manager->spif_fs, impl->file);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_size(vfs_file_t *vfs)
{
	s32_t ret;
	spiffs_stat s_info;
	vfs_spiffs_file *impl;

	impl = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_fstat(&spiffs_manager->spif_fs, impl->file, &s_info);
	spiffs_unlock(&spiffs_manager->lock);

	if (ret < 0) {
		return spiffs_ret_convert(ret);
	} else {
		return spiffs_ret_convert(s_info.size);
	}
}

static int vfs_spiffs_opendir(vfs_file_t *vfs, const char *path)
{
	s32_t ret = 0;
	vfs_spiffs_dir *impl;

	impl = (vfs_spiffs_dir *)container_of(vfs, vfs_spiffs_dir, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	if (!SPIFFS_opendir(&spiffs_manager->spif_fs, path, &impl->dir)) {
		ret = SPIFFS_errno(&spiffs_manager->spif_fs);
	}
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_readdir(vfs_file_t *vfs, struct vfs_info *info)
{
	int ret = 0;
	vfs_spiffs_dir *impl;
	struct spiffs_dirent dirent;

	impl = (vfs_spiffs_dir *)container_of(vfs, vfs_spiffs_dir, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	if (!SPIFFS_readdir(&impl->dir, &dirent)) {
		ret = SPIFFS_errno(&spiffs_manager->spif_fs);
	}
	spiffs_unlock(&spiffs_manager->lock);
	if (ret < 0) {
		return spiffs_ret_convert(ret);
	}

	memset(info, 0, sizeof(struct vfs_info));
	if (dirent.name[0] != 0) {
		if (dirent.type == SPIFFS_TYPE_DIR) {
			info->type = VFS_TYPE_DIR;
		} else {
			info->type = VFS_TYPE_FILE;
			info->size = dirent.size;
		}
		strlcpy(info->name, (char *)dirent.name, sizeof(info->name));
	}

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_closedir(vfs_file_t *vfs)
{
	s32_t ret;
	vfs_spiffs_dir *impl;

	impl = (vfs_spiffs_dir *)container_of(vfs, vfs_spiffs_dir, base);

	_SPIFFS_MANAGER_CHECK(spiffs_manager);
	spiffs_lock(&spiffs_manager->lock);
	ret = SPIFFS_closedir(&impl->dir);
	spiffs_unlock(&spiffs_manager->lock);

	return spiffs_ret_convert(ret);
}

static int vfs_spiffs_mkdir(const char *path)
{
	return -ENOTSUP; // spiffs not support.
}

static int vfs_spiffs_rmdir(const char *path)
{
	return -ENOTSUP; // spiffs not support.
}

static int vfs_spiffs_release(vfs_file_t *vfs, VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_spiffs_file *impl_file;
	vfs_spiffs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = vfs;
		free(impl_none);
		break;
	case VFS_PATH_FILE:
		impl_file = (vfs_spiffs_file *)container_of(vfs, vfs_spiffs_file, base);
		free(impl_file);
		break;
	case VFS_PATH_DIR:
		impl_dir = (vfs_spiffs_dir *)container_of(vfs, vfs_spiffs_dir, base);
		free(impl_dir);
		break;
	default:
		break;
	}

	return 0;
}

static const struct vfs_file_ops spiffs_ops = {
	.vfs_open     = vfs_spiffs_open,
	.vfs_close    = vfs_spiffs_close,
	.vfs_read     = vfs_spiffs_read,
	.vfs_write    = vfs_spiffs_write,
	.vfs_seek     = vfs_spiffs_seek,
	.vfs_sync     = vfs_spiffs_sync,
	.vfs_stat     = vfs_spiffs_stat,
	.vfs_unlink   = vfs_spiffs_unlink,
	.vfs_rename   = vfs_spiffs_rename,
	.vfs_tell     = vfs_spiffs_tell,
	.vfs_size     = vfs_spiffs_size,
	.vfs_opendir  = vfs_spiffs_opendir,
	.vfs_closedir = vfs_spiffs_closedir,
	.vfs_readdir  = vfs_spiffs_readdir,
	.vfs_mkdir    = vfs_spiffs_mkdir, // not support
	.vfs_rmdir    = vfs_spiffs_rmdir, // not support
	.vfs_release  = vfs_spiffs_release,
};

static vfs_file_t *vfs_spiffs_create(VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_spiffs_file *impl_file;
	vfs_spiffs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = malloc(sizeof(vfs_file_t));
		if (impl_none == NULL) {
			return NULL;
		}
		memset(impl_none, 0, sizeof(vfs_file_t));
		impl_none->ops = &spiffs_ops;
		return impl_none;
	case VFS_PATH_FILE:
		impl_file = malloc(sizeof(vfs_spiffs_file));
		if (impl_file == NULL) {
			return NULL;
		}
		memset(impl_file, 0, sizeof(vfs_spiffs_file));
		impl_file->base.ops = &spiffs_ops;
		return &impl_file->base;
	case VFS_PATH_DIR:
		impl_dir = malloc(sizeof(vfs_spiffs_dir));
		if (impl_dir == NULL) {
			return NULL;
		}
		memset(impl_dir, 0, sizeof(vfs_spiffs_dir));
		impl_dir->base.ops = &spiffs_ops;
		return &impl_dir->base;
	default:
		return NULL;
	}
}

static const vfs_creator_t vfs_spiffs_ctor = {
	.create = vfs_spiffs_create,
};

int vfs_register_spiffs(void)
{
	return vfs_register(&vfs_spiffs_ctor, "data");
}
