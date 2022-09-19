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

#ifndef _VFS_H_
#define _VFS_H_

#include <stdio.h>
#include "libc/errno.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VFS_DBG_ON     0
#define VFS_INF_ON     1
#define VFS_WRN_ON     1
#define VFS_ERR_ON     1
#define VFS_ASSERT_ON  1


#define VFS_LOG(flags, fmt, arg...)  \
	do {                             \
		if (flags)                   \
			printf(fmt, ##arg);      \
	} while (0)

#define VFS_DBG(fmt, arg...) VFS_LOG(VFS_DBG_ON, "[VFS DBG] "fmt, ##arg)
#define VFS_INF(fmt, arg...) VFS_LOG(VFS_INF_ON, "[VFS INF] "fmt, ##arg)
#define VFS_WRN(fmt, arg...) VFS_LOG(VFS_WRN_ON, "[VFS WRN] "fmt, ##arg)
#define VFS_ERR(fmt, arg...)                            \
	do {                                                \
		VFS_LOG(VFS_ERR_ON, "[VFS ERR] %s():%d, "fmt,   \
		        __func__, __LINE__, ##arg);             \
	} while (0)

#if VFS_ASSERT_ON
#define VFS_CHECK(ptr)                 \
	do {                               \
		if (NULL == ptr) {             \
			return -1;                 \
		}                              \
	} while (0)

#define VFS_CHECK_NULL(ptr)            \
	do {                               \
		if (NULL == ptr) {             \
			return NULL;               \
		}                              \
	} while (0)
#else
#define VFS_CHECK(ptr)
#define VFS_CHECK_NULL(ptr)
#endif

typedef enum {
	VFS_PATH_NONE,
	VFS_PATH_FILE,
	VFS_PATH_DIR,
} VFS_PATH_TYPE;

/*
 * Flags for file open
 */
#define VFS_RDONLY  0x0001    // Open a file as read only
#define VFS_WRONLY  0x0002    // Open a file as write only
#define VFS_RDWR    0x0003    // Open a file as read and write
#define VFS_CREAT   0x0100    // Create a file if it does not exist
#define VFS_EXCL    0x0200    // Fail if a file already exists
#define VFS_TRUNC   0x0400    // Truncate the existing file to zero size
#define VFS_APPEND  0x0800    // Move to end of file on every write

#define VFS_ACCMODE (VFS_RDONLY | VFS_WRONLY | VFS_RDWR)

typedef enum {
	VFS_TYPE_FILE,
	VFS_TYPE_DIR,
} VFS_FILE_TYPE;

#ifndef SEEK_SET
#define SEEK_SET  0
#endif

#ifndef SEEK_CUR
#define SEEK_CUR  1
#endif

#ifndef SEEK_END
#define SEEK_END  2
#endif

#define VFS_NAME_MAX 255

typedef struct vfs_file_s vfs_file_t;

struct vfs_info {
	VFS_FILE_TYPE type;
	int size;
	char name[VFS_NAME_MAX + 1];
};

typedef struct vfs_creator_s vfs_creator_t;

struct vfs_creator_s {
	vfs_file_t *(*create)(VFS_PATH_TYPE type);
};

struct vfs_file_ops {
	int (*vfs_open)(vfs_file_t *, const char *path, int flags);
	int (*vfs_close)(vfs_file_t *);
	int (*vfs_read)(vfs_file_t *, void *buffer, unsigned int len);
	int (*vfs_write)(vfs_file_t *, const void *buffer, unsigned int len);
	int (*vfs_seek)(vfs_file_t *, int64_t offset, int whence);
	int (*vfs_sync)(vfs_file_t *);
	int (*vfs_stat)(const char *path, struct vfs_info *);
	int (*vfs_unlink)(const char *path);
	int (*vfs_rename)(const char *old_name, const char *new_name);
	int (*vfs_tell)(vfs_file_t *);
	int (*vfs_size)(vfs_file_t *);
	int (*vfs_opendir)(vfs_file_t *, const char *path);
	int (*vfs_closedir)(vfs_file_t *);
	int (*vfs_readdir)(vfs_file_t *, struct vfs_info *);
	int (*vfs_mkdir)(const char *path);
	int (*vfs_rmdir)(const char *path);
	int (*vfs_release)(vfs_file_t *, VFS_PATH_TYPE type);
};

struct vfs_file_s {
	const struct vfs_file_ops *ops;
};

/**
 * @brief Init list of vfs.
 * @param[in] NULL
 * @return 0 on success, negative error on failure.
 */
int vfs_list_init(void);

/**
 * @brief register file system to vfs.
 * @param[in] creator the creator to create file system
 * @param[in] type    the sysbol of the file system
 * @return 0 on success, negative error on failure.
 */
int vfs_register(const void *creator, char *type);

/**
 * @brief Open the file by its path.
 * @param[in] path   the path of the file to open.
 * @param[in] flags  the flags of open operation.
 *                   VFS_RDONLY Open a file as read only
 *                   VFS_WRONLY Open a file as write only
 *                   VFS_RDWR Open a file as read and write
 *                   VFS_CREAT Create a file if it does not exist
 *                   VFS_EXCL Fail if a file already exists
 *                   VFS_TRUNC Truncate the existing file to zero size
 *                   VFS_APPEND Move to end of file on every write
 * @return file handle on success, NULL on failure.
 */
vfs_file_t *vfs_open(const char *path, int flags);

/**
 * @brief Close the file.
 * @param[in] vfs  the file handle.
 * @return 0 on success, negative error on failure.
 */
int vfs_close(vfs_file_t *vfs);

/**
 * @brief Read the contents of a file into a buffer.
 * @param[in] vfs     the file handle.
 * @param[in] buffer  the buffer to read in to.
 * @param[in] len     the len of bytes to read.
 * @return The number of bytes read, 0 at end of file, negative error on failure.
 */
int vfs_read(vfs_file_t *vfs, void *buffer, unsigned int len);

/**
 * @brief Write the contents of a buffer to file.
 * @param[in] vfs     the file handle.
 * @param[in] buffer  the buffer to write from.
 * @param[in] len     the len of bytes to write.
 * @return The number of bytes write, negative error on failure.
 */
int vfs_write(vfs_file_t *vfs, const void *buffer, unsigned int len);

/**
 * @brief Move the file position to a given offset from a given location.
 * @param[in] vfs     the file handle.
 * @param[in] offset  The offset from whence to move to.
 * @param[in] whence  The start of where to seek.
 *                    SEEK_SET to start from beginning of file.
 *                    SEEK_CUR to start from current position in file.
 *                    SEEK_END to start from end of file.
 * @return 0 on success, negative error on failure.
 */
int vfs_seek(vfs_file_t *vfs, int64_t offset, int whence);

/**
 * @brief Flush any buffers associated with the file.
 * @param[in] vfs  the file handle.
 * @return 0 on success, negative error on failure.
 */
int vfs_sync(vfs_file_t *vfs);

/**
 * @brief Get the position of the file.
 * @param[in] vfs  the file handle.
 * @return the position of the file
 */
int vfs_tell(vfs_file_t *vfs);

/**
 * @brief Get the size of the file.
 * @param[in] vfs  the file handle.
 * @return the size of the file
 */
int vfs_size(vfs_file_t *vfs);

/**
 * @brief Store type and size information about the path in a stat structure.
 * @param[in] path  the path of the file to get information.
 * @param[in] info  the info buffer to write to.
 * @return 0 on success, negative error code on failure.
 */
int vfs_stat(const char *path, struct vfs_info *info);

/**
 * @brief Remove a file from the filesystem.
 * @param[in] path  the path of the file to remove.
 * @return 0 on success, negative error code on failure.
 */
int vfs_unlink(const char *path);

/**
 * @brief Rename a file.
 * @param[in] old_name  The path of the file to rename.
 * @param[in] new_name  The new path
 * @return 0 on success, negative error code on failure.
 */
int vfs_rename(const char *old_name, const char *new_name);

/**
 * @brief Open a directory by its path.
 * @param[in] path  the path of the directory to open.
 * @return file handle on success, NULL on failure.
 */
vfs_file_t *vfs_opendir(const char *path);

/**
 * @brief Read the next directory entry.
 * @param[in] vfs   the file handle.
 * @param[in] info  the info buffer to write to.
 * @return 0 on success, negative error code on failure.
 */
int vfs_readdir(vfs_file_t *vfs, struct vfs_info *info);

/**
 * @brief Close a directory.
 * @param[in] vfs  the file handle.
 * @return 0 on success, negative error on failure.
 */
int vfs_closedir(vfs_file_t *vfs);

/**
 * @brief Create a directory, if they do not already exist.
 * @param[in] path  the path of the directory.
 * @return 0 on success, negative error on failure.
 */
int vfs_mkdir(const char *path);

/**
 * @brief Remove a directory.
 * @param[in] path  the path of the directory.
 * @return 0 on success, negative error on failure.
 */
int vfs_rmdir(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* _VFS_H_ */
