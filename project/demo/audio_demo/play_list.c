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
#include <string.h>
#include <stdlib.h>
#include "kernel/os/os.h"
#include "fs/vfs.h"
#include "play_list.h"

#define PLAYER_SONGS_DIR  "data/music"

#define LIST_LOGE(fmt, arg...)  printf("[LIST_ERR][F:%s][L:%d] " fmt, __func__, __LINE__, ##arg)
#define LIST_LOGD(fmt, arg...)  printf("[LIST_DBG][F:%s][L:%d] " fmt, __func__, __LINE__, ##arg)

static char *support_suffix[] = {".mp3", ".amr", ".wav", ".m4a", ".aac", ".mp2", ".mp1", ".ts"};

#define MAX_PLAY_LIST_URL_NUM 100

struct play_list_info {
	int num;
	int idx;
	char *url[MAX_PLAY_LIST_URL_NUM];
};

static struct play_list_info list_info;

static int add_url_to_list(char *url)
{
	char *url_buffer;
	struct play_list_info *info = &list_info;

	url_buffer = strdup(url);
	if (url_buffer == NULL) {
		return -1;
	}

	info->url[info->num] = url_buffer;
	info->num++;

	return 0;
}

int player_read_song(PLAYER_READ_SONG ctrl, char *buff)
{
	char *url;
	struct play_list_info *info = &list_info;

	if (ctrl == PLAYER_NEXT) {
		info->idx++;
		if (info->idx == info->num) {
			info->idx = 0;
		}
	} else {
		if (info->idx == 0) {
			info->idx = info->num - 1;
		} else {
			info->idx--;
		}
	}

	url = info->url[info->idx];
	strcpy(buff, url);
	return 0;
}

static int create_play_list(void)
{
	int ret;
	char *suffix;
	vfs_file_t *vfs;
	struct vfs_info finfo;
	char f_path[300];

	vfs = vfs_opendir(PLAYER_SONGS_DIR);
	if (vfs == NULL) {
		LIST_LOGE("open dir error\n");
		return -1;
	}

	while (1) {
		int i;
		int suffix_num;

		ret = vfs_readdir(vfs, &finfo);
		if ((ret < 0) || (finfo.name[0] == '\0')) {
			LIST_LOGD("scan mp3 end\n");
			break;
		}

		suffix_num = sizeof(support_suffix) / sizeof(support_suffix[0]);
		for (i = 0; i < suffix_num; i++) {
			suffix = NULL;
			suffix = strstr(finfo.name, support_suffix[i]);
			if (suffix) {
				LIST_LOGD("find a song: %s\n", finfo.name);
				sprintf(f_path, "file://%s/%s", PLAYER_SONGS_DIR, finfo.name);
				add_url_to_list(f_path);
				break;
			}
		}
	}

	vfs_closedir(vfs);

	if (list_info.num == 0) {
		LIST_LOGD("no song found\n");
		return -1;
	}

	return 0;
}

int play_list_init(void)
{
	int ret;

	memset(&list_info, 0, sizeof(list_info));

	ret = create_play_list();
	if (ret != 0) {
		LIST_LOGE("create play list fail\n");
		return -1;
	}

	return 0;
}

void play_list_deinit(void)
{
	int i;
	struct play_list_info *info = &list_info;

	for (i = 0; i < MAX_PLAY_LIST_URL_NUM; i++) {
		free(info->url[i]);
	}
}

