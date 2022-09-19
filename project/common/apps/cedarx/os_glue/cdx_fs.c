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

#if defined(CONFIG_XPLAYER) && defined(CONFIG_FILESYSTEMS)

#include <stdlib.h>
#include <string.h>
#include "cdx_fs.h"
#include "fs/vfs.h"

#define DBG(fmt, ...)   printf("[Cedarx File DBG] %s line %d, " fmt, __func__, __LINE__, ##__VA_ARGS__);

FILE *cdx_fopen(const char *filename, const char *mode)
{
	int fmode = 0;
	vfs_file_t *vfs;

	if (strstr(mode, "a")) {
		fmode |= VFS_APPEND;
	}
	if (strstr(mode, "+")) {
		fmode |= VFS_RDWR;
	}
	if (strstr(mode, "w")) {
		fmode |= (VFS_WRONLY | VFS_CREAT);
	}
	if (strstr(mode, "r")) {
		fmode |= VFS_RDONLY;
	}

	vfs = vfs_open(filename, fmode);

	return vfs;
}

int cdx_fread(void *ptr, int size, int nmemb, FILE *stream)
{
	unsigned int ret;

	if (stream == NULL) {
		return -1;
	}
	ret = vfs_read((vfs_file_t *)stream, ptr, size * nmemb);
	return (int)(ret / size);
}

int cdx_fwrite(const void *ptr, int size, int nmemb, FILE *stream)
{
	unsigned int ret;

	if (stream == NULL) {
		return -1;
	}
	ret = vfs_write((vfs_file_t *)stream, ptr, size * nmemb);
	return (int)(ret / size);
}

int cdx_fseek(FILE *stream, long long offset, int whence)
{
	if (stream == NULL) {
		return -1;
	}
	return vfs_seek((vfs_file_t *)stream, offset, whence);
}

long long cdx_ftell(FILE *stream)
{
	if (stream == NULL) {
		return -1;
	}
	return vfs_tell((vfs_file_t *)stream);
}

int cdx_fclose(FILE *stream)
{
	if (stream == NULL) {
		return -1;
	}
	return vfs_close((vfs_file_t *)stream);
}

int cdx_unlink(const char *filename)
{
	return vfs_unlink(filename);
}

#endif
