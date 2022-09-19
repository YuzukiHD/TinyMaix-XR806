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
#include "image/flash.h"
#include "kernel/os/os_mutex.h"
#include "fs/vfs.h"
#include "fs/littlefs/lfs.h"
#include "fs/littlefs/vfs_lfs.h"

#define DEFAULT_CACHE_SIZE       256
#define DEFAULT_LOOKAHEAD_SIZE   16

#define LFS_DEFAULT_FLASH   0

#define LFS_USE_STATIC_BUFFER

#ifdef LFS_USE_STATIC_BUFFER
static char lfs_read_buffer[DEFAULT_CACHE_SIZE];
static char lfs_prog_buffer[DEFAULT_CACHE_SIZE];
static char lfs_lookahead_buffer[DEFAULT_LOOKAHEAD_SIZE];
#endif

typedef enum MountedState {
	UNMOUNTED = 0,
	MOUNTED = 1,
} MountedState;

typedef OS_Mutex_t lfs_lock_t;

typedef struct {
	uint8_t             mounted_state;
	struct lfs_config   config;
	lfs_t               lfs;
	lfs_lock_t          lock;
	uint32_t            start_addr;
} lfs_manager_t;

typedef struct {
	vfs_file_t base;
	lfs_file_t file;
} vfs_lfs_file;

typedef struct {
	vfs_file_t base;
	lfs_dir_t dir;
} vfs_lfs_dir;

static lfs_manager_t lfs_manager;

#define _LFS_MANAGER_CHECK(littlefs_mgr)        \
	do {                                        \
		if (!littlefs_mgr.mounted_state) {      \
			return -ENODEV;                     \
		}                                       \
	} while (0)

static inline int lfs_lock_create(lfs_lock_t *lock)
{
	if (OS_MutexCreate(lock) != OS_OK) {
		return -1;
	}

	return 0;
}

static inline void lfs_lock_destory(lfs_lock_t *lock)
{
	OS_MutexDelete(lock);
}

static inline void lfs_lock(lfs_lock_t *lock)
{
	OS_MutexLock(lock, OS_WAIT_FOREVER);
}

static inline void lfs_unlock(lfs_lock_t *lock)
{
	OS_MutexUnlock(lock);
}

static int lfs_block_read(const struct lfs_config *c, lfs_block_t block,
                          lfs_off_t off, void *dst, lfs_size_t size)
{
	uint32_t addr, status;

	addr = c->block_size * block + off + lfs_manager.start_addr;

	status = flash_read(LFS_DEFAULT_FLASH, addr, dst, size);
	return status < 0 ? status : 0;
}

static int lfs_block_write(const struct lfs_config *c, lfs_block_t block,
                           lfs_off_t off, const void *dst, lfs_size_t size)
{
	uint32_t addr, status;

	addr = c->block_size * block + off + lfs_manager.start_addr;

	status = flash_write(LFS_DEFAULT_FLASH, addr, dst, size);
	return status < 0 ? status : 0;
}

static int lfs_block_erase(const struct lfs_config *c, lfs_block_t block)
{
	uint32_t addr;

	addr = c->block_size * block + lfs_manager.start_addr;

	return flash_erase(LFS_DEFAULT_FLASH, addr, c->block_size);
}

static int lfs_block_sync(const struct lfs_config *c)
{
	return 0;
}

static int lfs_init(struct vfs_lfs_config *config)
{
	int ret;

	ret = lfs_lock_create(&lfs_manager.lock);
	if (ret != OS_OK) {
		return -1;
	}

	lfs_manager.config.read  = lfs_block_read;
	lfs_manager.config.prog  = lfs_block_write;
	lfs_manager.config.erase = lfs_block_erase;
	lfs_manager.config.sync  = lfs_block_sync;
	lfs_manager.config.read_size      = 16;
	lfs_manager.config.prog_size      = 16;
	lfs_manager.config.block_size     = config->block_size;
	lfs_manager.config.block_count    = config->block_count;
	lfs_manager.config.cache_size     = DEFAULT_CACHE_SIZE;
	lfs_manager.config.lookahead_size = DEFAULT_LOOKAHEAD_SIZE;
	lfs_manager.config.block_cycles   = 500;

#ifdef LFS_USE_STATIC_BUFFER
	lfs_manager.config.read_buffer = lfs_read_buffer;
	lfs_manager.config.prog_buffer = lfs_prog_buffer;
	lfs_manager.config.lookahead_buffer = lfs_lookahead_buffer;
#endif
	lfs_manager.start_addr = config->start_addr;

	return 0;
}

static int lfs_deinit(void)
{
	lfs_lock_destory(&lfs_manager.lock);
	memset(&lfs_manager, 0, sizeof(lfs_manager_t));

	return 0;
}

int vfs_lfs_mount(uint32_t dev_id, struct vfs_lfs_config *config)
{
	int ret;

	ret = lfs_init(config);
	if (ret != 0) {
		return -1;
	}

	ret = lfs_mount(&lfs_manager.lfs, &lfs_manager.config);
	if (ret < 0) {
		ret = lfs_format(&lfs_manager.lfs, &lfs_manager.config);
		if (ret < 0) {
			VFS_ERR("lfs_format fail.(%d)\n", ret);
			lfs_deinit();
			return -1;
		}
		ret = lfs_mount(&lfs_manager.lfs, &lfs_manager.config);
		if (ret < 0) {
			VFS_ERR("lfs_mount fail.(%d)\n", ret);
			lfs_deinit();
			return -1;
		}
	}
	lfs_manager.mounted_state = MOUNTED;
	VFS_INF("LittleFS mount success.\n");

	return 0;
}

int vfs_lfs_unmount(uint32_t dev_id)
{
	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_unmount(&lfs_manager.lfs);
	lfs_manager.mounted_state = UNMOUNTED;
	lfs_deinit();

	return 0;
}

int vfs_lfs_format(uint32_t dev_id)
{
	int ret;

	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_unmount(&lfs_manager.lfs);
	ret = lfs_format(&lfs_manager.lfs, &lfs_manager.config);
	if (ret < 0) {
		VFS_ERR("lfs_format fail.(%d)\n", ret);
		return -1;
	}
	lfs_manager.mounted_state = UNMOUNTED;
	lfs_deinit();

	return ret;
}

static int lfs_mode_convert(int mode)
{
	int flags = 0;
	int acc_mode;

	acc_mode = mode & VFS_ACCMODE;
	if (acc_mode == VFS_RDONLY) {
		flags |= LFS_O_RDONLY;
	} else if (acc_mode == VFS_WRONLY) {
		flags |= LFS_O_WRONLY;
	} else if (acc_mode == VFS_RDWR) {
		flags |= LFS_O_RDWR;
	}

	if (mode & VFS_CREAT) {
		flags |= LFS_O_CREAT;
	}
	if (mode & VFS_EXCL) {
		flags |= LFS_O_EXCL;
	}
	if (mode & VFS_TRUNC) {
		flags |= LFS_O_TRUNC;
	}
	if (mode & VFS_APPEND) {
		flags |= LFS_O_APPEND;
	}
	return flags;
}

static int lfs_ret_convert(int lfs_ret)
{
	int ret;

	if (lfs_ret > 0) {
		return lfs_ret;
	}
	switch (lfs_ret) {
	case LFS_ERR_OK:
		ret = 0;
		break;
	case LFS_ERR_IO:
		ret = -EIO;
		break;
	case LFS_ERR_CORRUPT:
		ret = -EIO;
		break;
	case LFS_ERR_NOENT:
		ret = -ENOENT;
		break;
	case LFS_ERR_EXIST:
		ret = -EEXIST;
		break;
	case LFS_ERR_NOTDIR:
		ret = -ENOTDIR;
		break;
	case LFS_ERR_ISDIR:
		ret = -EISDIR;
		break;
	case LFS_ERR_NOTEMPTY:
		ret = -ENOTEMPTY;
		break;
	case LFS_ERR_BADF:
		ret = -EBADF;
		break;
	case LFS_ERR_FBIG:
		ret = -EFBIG;
		break;
	case LFS_ERR_INVAL:
		ret = -EINVAL;
		break;
	case LFS_ERR_NOSPC:
		ret = -ENOSPC;
		break;
	case LFS_ERR_NOMEM:
		ret = -ENOMEM;
		break;
	case LFS_ERR_NOATTR:
		ret = -ENODATA;
		break;
	case LFS_ERR_NAMETOOLONG:
		ret = -ENAMETOOLONG;
		break;
	default:
		ret = lfs_ret;
		break;
	}
	return ret;
}

static int vfs_lfs_open(vfs_file_t *vfs, const char *path, int mode)
{
	int ret;
	int flags;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);
	flags = lfs_mode_convert(mode);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_open(&lfs_manager.lfs, &impl->file, path, flags);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_close(vfs_file_t *vfs)
{
	int ret;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_close(&lfs_manager.lfs, &impl->file);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_read(vfs_file_t *vfs, void *buffer, unsigned int len)
{
	lfs_ssize_t read_size;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	read_size = lfs_file_read(&lfs_manager.lfs, &impl->file, buffer, len);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(read_size);
}

static int vfs_lfs_write(vfs_file_t *vfs, const void *buffer, unsigned int len)
{
	lfs_ssize_t write_size;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	write_size = lfs_file_write(&lfs_manager.lfs, &impl->file, buffer, len);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(write_size);
}

static int vfs_lfs_seek(vfs_file_t *vfs, int64_t offset, int whence)
{
	lfs_soff_t ret;
	vfs_lfs_file *impl;
	enum lfs_whence_flags flags;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	if (whence == SEEK_SET) {
		flags = LFS_SEEK_SET;
	} else if (whence == SEEK_CUR) {
		flags = LFS_SEEK_CUR;
	} else if (whence == SEEK_END) {
		flags = LFS_SEEK_END;
	} else {
		return -1;
	}

	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_seek(&lfs_manager.lfs, &impl->file, offset, flags);
	lfs_unlock(&lfs_manager.lock);

	if (ret < 0) {
		return lfs_ret_convert(ret);
	} else {
		return 0;
	}
}

static int vfs_lfs_sync(vfs_file_t *vfs)
{
	int ret;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_sync(&lfs_manager.lfs, &impl->file);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_stat(const char *path, struct vfs_info *info)
{
	int ret;
	struct lfs_info l_info;

	_LFS_MANAGER_CHECK(lfs_manager);
	memset(info, 0, sizeof(struct vfs_info));
	lfs_lock(&lfs_manager.lock);
	ret = lfs_stat(&lfs_manager.lfs, path, &l_info);
	lfs_unlock(&lfs_manager.lock);
	if (ret < 0) {
		return lfs_ret_convert(ret);
	}

	if (l_info.type == LFS_TYPE_REG) {
		info->type = VFS_TYPE_FILE;
		info->size = l_info.size;
	} else {
		info->type = VFS_TYPE_DIR;
	}

	return lfs_ret_convert(ret);
}

static int vfs_lfs_unlink(const char *path)
{
	int ret;

	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_remove(&lfs_manager.lfs, path);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_rename(const char *old_name, const char *new_name)
{
	int ret;

	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_rename(&lfs_manager.lfs, old_name, new_name);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_tell(vfs_file_t *vfs)
{
	lfs_soff_t ret;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_tell(&lfs_manager.lfs, &impl->file);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_size(vfs_file_t *vfs)
{
	lfs_soff_t ret;
	vfs_lfs_file *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_file_size(&lfs_manager.lfs, &impl->file);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_opendir(vfs_file_t *vfs, const char *path)
{
	int ret;
	vfs_lfs_dir *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_dir *)container_of(vfs, vfs_lfs_dir, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_dir_open(&lfs_manager.lfs, &impl->dir, path);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_closedir(vfs_file_t *vfs)
{
	int ret;
	vfs_lfs_dir *impl;

	_LFS_MANAGER_CHECK(lfs_manager);
	impl = (vfs_lfs_dir *)container_of(vfs, vfs_lfs_dir, base);

	lfs_lock(&lfs_manager.lock);
	ret = lfs_dir_close(&lfs_manager.lfs, &impl->dir);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_readdir(vfs_file_t *vfs, struct vfs_info *info)
{
	int ret;
	vfs_lfs_dir *impl;
	struct lfs_info l_info;

	_LFS_MANAGER_CHECK(lfs_manager);
	memset(info, 0, sizeof(struct vfs_info));
	impl = (vfs_lfs_dir *)container_of(vfs, vfs_lfs_dir, base);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_dir_read(&lfs_manager.lfs, &impl->dir, &l_info);
	lfs_unlock(&lfs_manager.lock);
	if (ret < 0) {
		return lfs_ret_convert(ret);
	}
	if (l_info.name[0] != 0) {
		if (l_info.type == LFS_TYPE_REG) {
			info->type = VFS_TYPE_FILE;
			info->size = l_info.size;
		} else {
			info->type = VFS_TYPE_DIR;
		}
		strlcpy(info->name, l_info.name, sizeof(info->name));
	}

	return 0;
}

static int vfs_lfs_mkdir(const char *path)
{
	int ret;

	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_mkdir(&lfs_manager.lfs, path);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_rmdir(const char *path)
{
	int ret;

	_LFS_MANAGER_CHECK(lfs_manager);
	lfs_lock(&lfs_manager.lock);
	ret = lfs_remove(&lfs_manager.lfs, path);
	lfs_unlock(&lfs_manager.lock);

	return lfs_ret_convert(ret);
}

static int vfs_lfs_release(vfs_file_t *vfs, VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_lfs_file *impl_file;
	vfs_lfs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = vfs;
		free(impl_none);
		break;
	case VFS_PATH_FILE:
		impl_file = (vfs_lfs_file *)container_of(vfs, vfs_lfs_file, base);
		free(impl_file);
		break;
	case VFS_PATH_DIR:
		impl_dir = (vfs_lfs_dir *)container_of(vfs, vfs_lfs_dir, base);
		free(impl_dir);
		break;
	default:
		break;
	}

	return 0;
}

static const struct vfs_file_ops lfs_ops = {
	.vfs_open     = vfs_lfs_open,
	.vfs_close    = vfs_lfs_close,
	.vfs_read     = vfs_lfs_read,
	.vfs_write    = vfs_lfs_write,
	.vfs_seek     = vfs_lfs_seek,
	.vfs_sync     = vfs_lfs_sync,
	.vfs_stat     = vfs_lfs_stat,
	.vfs_unlink   = vfs_lfs_unlink,
	.vfs_rename   = vfs_lfs_rename,
	.vfs_tell     = vfs_lfs_tell,
	.vfs_size     = vfs_lfs_size,
	.vfs_opendir  = vfs_lfs_opendir,
	.vfs_closedir = vfs_lfs_closedir,
	.vfs_readdir  = vfs_lfs_readdir,
	.vfs_mkdir    = vfs_lfs_mkdir,
	.vfs_rmdir    = vfs_lfs_rmdir,
	.vfs_release  = vfs_lfs_release,
};

static vfs_file_t *vfs_lfs_create(VFS_PATH_TYPE type)
{
	vfs_file_t *impl_none;
	vfs_lfs_file *impl_file;
	vfs_lfs_dir *impl_dir;

	switch (type) {
	case VFS_PATH_NONE:
		impl_none = malloc(sizeof(vfs_file_t));
		if (impl_none == NULL) {
			return NULL;
		}
		memset(impl_none, 0, sizeof(vfs_file_t));
		impl_none->ops = &lfs_ops;
		return impl_none;
	case VFS_PATH_FILE:
		impl_file = malloc(sizeof(vfs_lfs_file));
		if (impl_file == NULL) {
			return NULL;
		}
		memset(impl_file, 0, sizeof(vfs_lfs_file));
		impl_file->base.ops = &lfs_ops;
		return &impl_file->base;
	case VFS_PATH_DIR:
		impl_dir = malloc(sizeof(vfs_lfs_dir));
		if (impl_dir == NULL) {
			return NULL;
		}
		memset(impl_dir, 0, sizeof(vfs_lfs_dir));
		impl_dir->base.ops = &lfs_ops;
		return &impl_dir->base;
	default:
		return NULL;
	}
}

static const vfs_creator_t vfs_lfs_ctor = {
	.create = vfs_lfs_create,
};

int vfs_register_lfs(void)
{
	return vfs_register(&vfs_lfs_ctor, "data");
}

