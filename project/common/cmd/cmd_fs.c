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

#ifdef CONFIG_FILESYSTEMS
#include "cmd_util.h"
#include "fs/vfs.h"
#include "common/framework/fs_ctrl.h"
#include "kernel/os/os_errno.h"

static vfs_file_t *cmd_file;
static vfs_file_t *cmd_dir;

static enum cmd_status cmd_fs_mount_exec(char *cmd)
{
	int ret;
	int cnt;
	unsigned int dev;

	cnt = cmd_sscanf(cmd, "d=%u", &dev);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch (dev) {
	case FS_MNT_DEV_TYPE_SDCARD:
		ret = fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0);
		break;
	case FS_MNT_DEV_TYPE_FLASH:
		ret = fs_ctrl_mount(FS_MNT_DEV_TYPE_FLASH, 0);
		break;
	default:
		ret = -1;
		break;
	}
	if (ret != 0) {
		CMD_ERR("mount fail\n");
		return CMD_STATUS_FAIL;
	}

	CMD_DBG("mount success\n");
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_unmount_exec(char *cmd)
{
	int ret;
	int cnt;
	unsigned int dev;

	cnt = cmd_sscanf(cmd, "d=%u", &dev);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch (dev) {
	case FS_MNT_DEV_TYPE_SDCARD:
		ret = fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0);
		break;
	case FS_MNT_DEV_TYPE_FLASH:
		ret = fs_ctrl_unmount(FS_MNT_DEV_TYPE_FLASH, 0);
		break;
	default:
		ret = -1;
		break;
	}
	if (ret != 0) {
		CMD_ERR("unmount fail\n");
		return CMD_STATUS_FAIL;
	}

	CMD_DBG("unmount success\n");
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_format_exec(char *cmd)
{
	int ret;
	int cnt;
	unsigned int dev;

	cnt = cmd_sscanf(cmd, "d=%u", &dev);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch (dev) {
	case FS_MNT_DEV_TYPE_SDCARD:
		ret = fs_ctrl_format(FS_MNT_DEV_TYPE_SDCARD, 0);
		break;
	case FS_MNT_DEV_TYPE_FLASH:
		ret = fs_ctrl_format(FS_MNT_DEV_TYPE_FLASH, 0);
		break;
	default:
		ret = -1;
		break;
	}
	if (ret != 0) {
		CMD_ERR("format fail\n");
		return CMD_STATUS_FAIL;
	}

	CMD_DBG("format success\n");
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_open_exec(char *cmd)
{
	if (cmd_file) {
		CMD_ERR("cmd not support open multiple files simultaneously\n");
		return CMD_STATUS_FAIL;
	}

	cmd_file = vfs_open(cmd, VFS_RDWR | VFS_CREAT);
	if (cmd_file == NULL) {
		CMD_ERR("open %s fail, errno:%d\n", cmd, OS_GetErrno());
		return CMD_STATUS_FAIL;
	}
	CMD_DBG("open success\n");
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_close_exec(char *cmd)
{
	if (!cmd_file) {
		CMD_ERR("no file need to be closed\n");
		return CMD_STATUS_FAIL;
	}

	vfs_close(cmd_file);
	cmd_file = NULL;
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_read_exec(char *cmd)
{
	int ret_len;
	uint32_t len, read_len;
	int argc = 0;
	char *argv[1];

	argc = cmd_parse_argv(cmd, argv, 1);
	if (argc != 1) {
		CMD_ERR("invalid param\n");
		return CMD_STATUS_FAIL;
	}

	len = atoi(argv[0]);
	if (len <= 512) {
		read_len = len;
	} else {
		read_len = 512;
	}

	char *buffer = cmd_malloc(read_len);
	if (buffer == NULL) {
		CMD_ERR("no mem\n");
		return CMD_STATUS_FAIL;
	}
	cmd_memset(buffer, 0, read_len);

	while (len) {
		ret_len = vfs_read(cmd_file, buffer, read_len);
		if (ret_len > 0) {
			CMD_LOG(1, "%s", buffer);
			cmd_memset(buffer, 0, read_len);
		} else {
			break;
		}

		len -= read_len;
		if (len < 512) {
			read_len = len;
		}
	}

	CMD_LOG(1, "\n");
	cmd_free(buffer);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_write_exec(char *cmd)
{
	int write_len;

	write_len = vfs_write(cmd_file, cmd, strlen(cmd));
	if (write_len < 0) {
		CMD_ERR("file write fail\n");
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_seek_exec(char *cmd)
{
	int ret;
	int argc = 0;
	char *argv[2];
	int offset;
	int whence;

	argc = cmd_parse_argv(cmd, argv, 2);
	if (argc != 2) {
		CMD_ERR("invalid param\n");
		return CMD_STATUS_FAIL;
	}

	if (cmd_strncmp(argv[0], "begin", 5) == 0) {
		whence = SEEK_SET;
	} else if (cmd_strncmp(argv[0], "current", 7) == 0) {
		whence = SEEK_CUR;
	} else if (cmd_strncmp(argv[0], "end", 3) == 0) {
		whence = SEEK_END;
	} else {
		CMD_ERR("invalid param\n");
		return CMD_STATUS_FAIL;
	}

	offset = atoi(argv[1]);
	ret = vfs_seek(cmd_file, offset, whence);
	if (ret != 0) {
		CMD_ERR("file seek fail, ret(%d)\n", ret);
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_tell_exec(char *cmd)
{
	int pos;

	pos = vfs_tell(cmd_file);
	CMD_LOG(1, "file position:%d\n", pos);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_size_exec(char *cmd)
{
	int size;

	size = vfs_size(cmd_file);
	CMD_LOG(1, "file size:%d\n", size);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_stat_exec(char *cmd)
{
	int ret;
	struct vfs_info info = {0};

	ret = vfs_stat(cmd, &info);
	if (ret != 0) {
		CMD_ERR("stat %s fail. ret:%d\n", cmd, ret);
		return CMD_STATUS_FAIL;
	}
	if (info.type == VFS_TYPE_FILE) {
		CMD_LOG(1, "%s, type:file, file size:%u\n", cmd, info.size);
	} else {
		CMD_LOG(1, "%s, type:directory\n", cmd);
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_rename_exec(char *cmd)
{
	int ret;
	int argc = 0;
	char *argv[2];

	argc = cmd_parse_argv(cmd, argv, 2);
	if (argc != 2) {
		CMD_ERR("invalid param\n");
		return CMD_STATUS_FAIL;
	}
	ret = vfs_rename(argv[0], argv[1]);
	if (ret != 0) {
		CMD_ERR("file rename fail. ret:%d\n", ret);
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "file rename success\n");
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_unlink_exec(char *cmd)
{
	int ret;

	ret = vfs_unlink(cmd);
	if (ret != 0) {
		CMD_ERR("remove %s fail. ret:%d\n", cmd, ret);
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "remove %s success\n", cmd);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_opendir_exec(char *cmd)
{
	if (cmd_dir) {
		CMD_ERR("cmd not support open multiple dir simultaneously\n");
		return CMD_STATUS_FAIL;
	}

	cmd_dir = vfs_opendir(cmd);
	if (cmd_dir == NULL) {
		CMD_ERR("open directory %s fail.errno:%d\n", cmd, OS_GetErrno());
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "open directory %s success\n", cmd);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_closedir_exec(char *cmd)
{
	int ret;

	if (!cmd_dir) {
		CMD_ERR("no dir need to be closed\n");
		return CMD_STATUS_FAIL;
	}

	ret = vfs_closedir(cmd_dir);
	if (ret != 0) {
		CMD_ERR("close directory fail. ret:%d\n", ret);
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "close directory success\n");
	cmd_dir = NULL;
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_readdir_exec(char *cmd)
{
	int ret;
	struct vfs_info info;

	while (1) {
		ret = vfs_readdir(cmd_dir, &info);
		if (ret != 0) {
			CMD_ERR("vfs_readdir fail. ret:%d\n", ret);
			return CMD_STATUS_FAIL;
		}
		if (info.name[0] != 0) {
			if (info.type == VFS_TYPE_FILE) {
				CMD_LOG(1, "%s, type:file, file size:%u\n", info.name,
				        info.size);
			} else {
				CMD_LOG(1, "%s, type:directory\n", info.name);
			}
		} else {
			break;
		}
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_mkdir_exec(char *cmd)
{
	int ret;

	ret = vfs_mkdir(cmd);
	if (ret != 0) {
		CMD_ERR("mkdir %s fail. ret:%d\n", cmd, ret);
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "mkdir %s success\n", cmd);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_rmdir_exec(char *cmd)
{
	int ret;

	ret = vfs_rmdir(cmd);
	if (ret != 0) {
		CMD_ERR("rmdir %s fail. ret:%d\n", cmd, ret);
		return CMD_STATUS_FAIL;
	}
	CMD_LOG(1, "rmdir %s success\n", cmd);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_fs_help_exec(char *cmd);

static const struct cmd_data g_fs_cmds[] = {
	{ "mount",    cmd_fs_mount_exec,    CMD_DESC("mount file system, fs mount d=<dev>, eg. fs mount d=1") },
	{ "unmount",  cmd_fs_unmount_exec,  CMD_DESC("unmount file system, fs unmount d=<dev>, eg. fs unmount d=1") },
	{ "format",   cmd_fs_format_exec,   CMD_DESC("format file system, fs format d=<dev>, eg. fs format d=1") },
	{ "open",     cmd_fs_open_exec,     CMD_DESC("open file, fs open <file-path>, eg. fs open data/test.txt") },
	{ "close",    cmd_fs_close_exec,    CMD_DESC("close file") },
	{ "read",     cmd_fs_read_exec,     CMD_DESC("read file context, fs read <byte_number>, eg. fs read 512") },
	{ "write",    cmd_fs_write_exec,    CMD_DESC("write context to file, fs write <string>, eg. fs write abcdefg") },
	{ "seek",     cmd_fs_seek_exec,     CMD_DESC("seek file, fs seek <whence> <offset>, eg. fs seek begin 32") },
	{ "tell",     cmd_fs_tell_exec,     CMD_DESC("get position of file now") },
	{ "size",     cmd_fs_size_exec,     CMD_DESC("get the size of file") },
	{ "stat",     cmd_fs_stat_exec,     CMD_DESC("get the stat of file, fs stat <path>, eg. fs stat data/test.txt") },
	{ "rename",   cmd_fs_rename_exec,   CMD_DESC("rename file, fs rename <old_path> <new_path>, eg. fs rename data/file1.txt data/file2.txt") },
	{ "unlink",   cmd_fs_unlink_exec,   CMD_DESC("remove file, fs unlink <path>, eg. fs unlink data/file1.txt") },
	{ "opendir",  cmd_fs_opendir_exec,  CMD_DESC("open directory, fs opendir <path>, eg. fs opendir data/") },
	{ "readdir",  cmd_fs_readdir_exec,  CMD_DESC("read directory, show all the file of this directory") },
	{ "closedir", cmd_fs_closedir_exec, CMD_DESC("close directory") },
	{ "mkdir",    cmd_fs_mkdir_exec,    CMD_DESC("create directory, fs mkdir <path>, eg. fs mkdir data/music") },
	{ "rmdir",    cmd_fs_rmdir_exec,    CMD_DESC("remove directory, fs rmdir <path>, eg. fs rmdir data/music") },
	{ "help",     cmd_fs_help_exec,     CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_fs_help_exec(char *cmd)
{
	return cmd_help_exec(g_fs_cmds, cmd_nitems(g_fs_cmds), 8);
}

enum cmd_status cmd_fs_exec(char *cmd)
{
	return cmd_exec(cmd, g_fs_cmds, cmd_nitems(g_fs_cmds));
}
#endif
