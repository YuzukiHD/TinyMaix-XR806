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

#include "common/framework/platform_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kernel/os/os.h"
#include "fs/vfs.h"

static int vfs_example(void)
{
	vfs_file_t *file;
	char *old_filename = "data/xr.txt";
	char *new_filename = "data/sdk.txt";
	char *wr_data1 = "xradio wireless SDK.";
	char *wr_data2 = "For vfs example.";

	// open/create a file
	file = vfs_open(old_filename, VFS_RDWR | VFS_CREAT);
	if (file == NULL) {
		printf("open file fail, errno:%d\n", OS_GetErrno());
		return -1;
	}
	printf("open success\n");

	// write this file
	int write_len = vfs_write(file, wr_data1, strlen(wr_data1));
	if (write_len < 0) {
		printf("file write fail\n");
		return -1;
	}
	printf("write success\n");

	// move the cursor
	int ret = vfs_seek(file, 0, SEEK_SET);
	if (ret != 0) {
		printf("file seek fail, ret(%d)\n", ret);
		return -1;
	}
	printf("seek to file head\n");

	// read this file
	int rdbuf_len = strlen(wr_data1);
	char *read_buf = malloc(rdbuf_len);
	if (read_buf == NULL) {
		printf("no mem1\n");
		return -1;
	}
	memset(read_buf, 0, rdbuf_len);

	ret = vfs_read(file, read_buf, rdbuf_len);
	if (ret > 0) {
		printf("file content:\n%s\n\n", read_buf);
	} else {
		printf("read file fail.\n");
		free(read_buf);
		return -1;
	}
	free(read_buf);
	printf("read success.\n");

	// move the cursor to the fourth from the end
	ret = vfs_seek(file, -4, SEEK_END);
	if (ret != 0) {
		printf("file seek fail, ret(%d)\n", ret);
		return -1;
	}
	printf("move the cursor to the fourth from the end\n");

	// view cursor position
	ret = vfs_tell(file);
	printf("file cursor position:%d\n", ret);

	// close this file
	ret = vfs_close(file);
	if (ret != 0) {
		printf("close file fail\n");
		return -1;
	}
	printf("close success\n");

	// rename this file
	ret = vfs_rename(old_filename, new_filename);
	if (ret != 0) {
		printf("file rename fail. ret:%d\n", ret);
		return -1;
	}
	printf("file rename success\n");

	// open this file with newname
	file = vfs_open(new_filename, VFS_RDWR);
	if (file == NULL) {
		printf("open file with newname fail, errno:%d\n", OS_GetErrno());
		return -1;
	}
	printf("open newname file success\n");

	// read this file with newname, the cursor will move to end
	rdbuf_len = strlen(wr_data1);
	read_buf = malloc(rdbuf_len);
	if (read_buf == NULL) {
		printf("no mem2\n");
		return -1;
	}
	memset(read_buf, 0, rdbuf_len);

	ret = vfs_read(file, read_buf, rdbuf_len);
	if (ret > 0) {
		printf("file with newname content:\n%s\n\n", read_buf);
	} else {
		printf("read file fail.\n");
		free(read_buf);
		return -1;
	}
	free(read_buf);
	printf("read newname file success.\n");

	// add some content from the end of file
	write_len = vfs_write(file, wr_data2, strlen(wr_data2));
	if (write_len < 0) {
		printf("add some content fail\n");
		return -1;
	}
	printf("add content success\n");

	// move the cursor
	ret = vfs_seek(file, 0, SEEK_SET);
	if (ret != 0) {
		printf("file seek fail, ret(%d)\n", ret);
		return -1;
	}
	printf("seek to file head.\n");

	// read all content
	rdbuf_len = strlen(wr_data1) + strlen(wr_data2);
	read_buf = malloc(rdbuf_len);
	if (read_buf == NULL) {
		printf("no mem3\n");
		return -1;
	}
	memset(read_buf, 0, rdbuf_len);

	ret = vfs_read(file, read_buf, rdbuf_len);
	if (ret > 0) {
		printf("file content:\n%s\n\n", read_buf);
	} else {
		printf("read newname file fail.\n");
		free(read_buf);
		return -1;
	}
	free(read_buf);
	printf("last read success.\n");

	// close this file
	ret = vfs_close(file);
	if (ret != 0) {
		printf("close newname file fail\n");
		return -1;
	}
	printf("close newname file success\n");

	return 0;
}

int main(void)
{
	platform_init();

	printf("[vfs-start]start vfs example.\n\n");

	int ret = vfs_example();

	if (ret != 0) {
		printf("\n[vfs-end]vfs example exec fail.\n");
	} else {
		printf("\n[vfs-end]vfs example exec success.\n");
	}

	return ret;
}
