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
#include "fs/vfs.h"
#include "sys/list.h"
#include "kernel/os/os_errno.h"

struct vfs_list_s {
	struct list_head list;
	int size;
};

struct vfs_node_s {
	struct list_head node;
	const char *scheme;
	const vfs_creator_t *creator;
};

static struct vfs_list_s vfs_list;

int vfs_list_init(void)
{
	INIT_LIST_HEAD(&vfs_list.list);
	vfs_list.size = 0;
	return 0;
}

int vfs_register(const void *creator, char *type)
{
	struct vfs_node_s *node;

	node = malloc(sizeof(*node));
	if (node == NULL) {
		return -1;
	}
	node->creator = (const vfs_creator_t *)creator;
	node->scheme = type;

	list_add(&node->node, &vfs_list.list);
	vfs_list.size++;
	return 0;
}

static vfs_file_t *vfs_create(const char *path, VFS_PATH_TYPE type, int *offset)
{
	char scheme[16] = {0};
	char *colon;
	const vfs_creator_t *ctor = NULL;
	struct vfs_node_s *vfs_node;

	colon = strchr(path, '/');
	if ((colon == NULL) || ((colon - path) >= 16)) {
		return NULL;
	}
	memcpy(scheme, path, colon - path);
	*offset = colon - path;

	list_for_each_entry(vfs_node, &vfs_list.list, node) {
		if (strcasecmp(vfs_node->scheme, scheme) == 0) {
			ctor = vfs_node->creator;
			break;
		}
	}

	if (NULL == ctor) {
		VFS_ERR("unsupport path. path(%s)", path);
		return NULL;
	}

	vfs_file_t *vfs = ctor->create(type);
	if (!vfs) {
		VFS_ERR("create vfs fail, path(%s)", path);
		return NULL;
	}

	return vfs;
}

vfs_file_t *vfs_open(const char *path, int flags)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_FILE, &offset);
	if (vfs == NULL) {
		return NULL;
	}

	VFS_CHECK_NULL(vfs->ops);
	VFS_CHECK_NULL(vfs->ops->vfs_open);
	ret = vfs->ops->vfs_open(vfs, path + offset, flags);
	if (ret) {
		OS_SetErrno(ret);
		VFS_CHECK_NULL(vfs->ops->vfs_release);
		vfs->ops->vfs_release(vfs, VFS_PATH_FILE);
		return NULL;
	}
	return vfs;
}

int vfs_close(vfs_file_t *vfs)
{
	int ret;

	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_close);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_close(vfs);
	vfs->ops->vfs_release(vfs, VFS_PATH_FILE);
	return ret;
}

int vfs_read(vfs_file_t *vfs, void *buffer, unsigned int len)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_read);
	return vfs->ops->vfs_read(vfs, buffer, len);
}

int vfs_write(vfs_file_t *vfs, const void *buffer, unsigned int len)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_write);
	return vfs->ops->vfs_write(vfs, buffer, len);
}

int vfs_seek(vfs_file_t *vfs, int64_t offset, int whence)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_seek);
	return vfs->ops->vfs_seek(vfs, offset, whence);
}

int vfs_sync(vfs_file_t *vfs)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_sync);
	return vfs->ops->vfs_sync(vfs);
}

int vfs_tell(vfs_file_t *vfs)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_tell);
	return vfs->ops->vfs_tell(vfs);
}

int vfs_size(vfs_file_t *vfs)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_size);
	return vfs->ops->vfs_size(vfs);
}

int vfs_stat(const char *path, struct vfs_info *info)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_NONE, &offset);
	if (vfs == NULL) {
		return -1;
	}

	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_stat);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_stat(path + offset, info);
	vfs->ops->vfs_release(vfs, VFS_PATH_NONE);

	return ret;
}

int vfs_unlink(const char *path)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_NONE, &offset);
	if (vfs == NULL) {
		return -1;
	}

	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_unlink);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_unlink(path + offset);
	vfs->ops->vfs_release(vfs, VFS_PATH_NONE);

	return ret;
}

int vfs_rename(const char *old_name, const char *new_name)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(old_name, VFS_PATH_NONE, &offset);
	if (vfs == NULL) {
		return -1;
	}

	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_rename);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_rename(old_name + offset, new_name + offset);
	vfs->ops->vfs_release(vfs, VFS_PATH_NONE);

	return ret;
}

vfs_file_t *vfs_opendir(const char *path)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_DIR, &offset);
	if (vfs == NULL) {
		return NULL;
	}

	VFS_CHECK_NULL(vfs->ops);
	VFS_CHECK_NULL(vfs->ops->vfs_opendir);
	ret = vfs->ops->vfs_opendir(vfs, path + offset);
	if (ret) {
		OS_SetErrno(ret);
		VFS_CHECK_NULL(vfs->ops->vfs_release);
		vfs->ops->vfs_release(vfs, VFS_PATH_DIR);
		return NULL;
	}
	return vfs;
}

int vfs_readdir(vfs_file_t *vfs, struct vfs_info *info)
{
	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_readdir);
	return vfs->ops->vfs_readdir(vfs, info);
}

int vfs_closedir(vfs_file_t *vfs)
{
	int ret;

	VFS_CHECK(vfs);
	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_closedir);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_closedir(vfs);
	vfs->ops->vfs_release(vfs, VFS_PATH_DIR);
	return ret;
}

int vfs_mkdir(const char *path)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_NONE, &offset);
	if (vfs == NULL) {
		return -1;
	}

	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_mkdir);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_mkdir(path + offset);
	vfs->ops->vfs_release(vfs, VFS_PATH_NONE);

	return ret;
}

int vfs_rmdir(const char *path)
{
	int ret;
	int offset;
	vfs_file_t *vfs;

	vfs = vfs_create(path, VFS_PATH_NONE, &offset);
	if (vfs == NULL) {
		return -1;
	}

	VFS_CHECK(vfs->ops);
	VFS_CHECK(vfs->ops->vfs_rmdir);
	VFS_CHECK(vfs->ops->vfs_release);
	ret = vfs->ops->vfs_rmdir(path + offset);
	vfs->ops->vfs_release(vfs, VFS_PATH_NONE);

	return ret;
}

