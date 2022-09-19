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
#include "common/apps/player_app.h"
#include "fs/vfs.h"
#include "audiofifo.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"
#include "audio/drc/drc.h"
#include "audio/eq/eq.h"


#define PLAYER_THREAD_STACK_SIZE    (1024 * 4)

#define PLAYER_DRC_EN               0
#define PLAYER_EQ_EN                0

static OS_Thread_t play_thread;
static player_base *player;
static int isCompleted = 0;

#if PLAYER_DRC_EN
static const drc_user_params_t drc_user_params_example = {
	.rms_average_time = 1,  //RMS averaging time in ms,  typical value is 1ms
	.attack_time = 1,       //attack time in ms,   typical value is 1ms
	.release_time = 100,    //release time in ms,  typical value is 100ms
	.target_level = -12,    //target level in dB <=0
};
#endif

#if PLAYER_EQ_EN
static eq_core_prms_t core_prms[10] = {
	{0,  100,   2, HIGHPASS},
	{-1, 200,   1, BANDPASS_PEAK},
	{-3, 400,   1, BANDPASS_PEAK},
	{3,  800,   2, BANDPASS_PEAK},
	{0,  1000,  1, BANDPASS_PEAK},
	{5,  2000,  1, BANDPASS_PEAK},
	{-3, 4000,  2, BANDPASS_PEAK},
	{5,  8000,  1, BANDPASS_PEAK},
	{0,  12000, 1, BANDPASS_PEAK},
	{0,  16000, 1, BANDPASS_PEAK},
};
static const eq_user_params_t eq_user_params_example = {
	.biq_num = 10,
	.core_params = (eq_core_prms_t *)&core_prms,
};
#endif

static void player_demo_callback(player_events event, void *data, void *arg)
{
	switch (event) {
	case PLAYER_EVENTS_MEDIA_PREPARED:
		printf("media is prepared. play media now.\n");
		printf("you can use player to seek, get duration(by size), get current position(by tell) from now on.\n");
		break;
	case PLAYER_EVENTS_MEDIA_STOPPED:
		printf("media is stopped by user.\n");
		isCompleted = 1;
		break;
	case PLAYER_EVENTS_MEDIA_ERROR:
		printf("error occur\n");
		isCompleted = 1;
		break;
	case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
		printf("media play is completed\n");
		isCompleted = 1;
		break;
	default:
		break;
	}
}

static int play_file_music(void)
{
	int ret;
	int duration;
	int position;

	/*
	 * play media in file system.
	 * 1. in this example, you should create a folder named music in your file system
	 * 2. add 1.mp3 to this folder
	 */
	isCompleted = 0;
	player->set_callback(player, player_demo_callback, NULL);
	ret = player->play(player, "file://data/music/1.mp3");
	if (ret != 0) {
		printf("music play fail.\n");
		return -1;
	}

	OS_Sleep(15);
	printf("music pause now, last for 10s\n");
	player->pause(player);
	OS_Sleep(10);

	printf("music resume now.\n");
	player->resume(player);

	OS_Sleep(20);
	duration = player->size(player);
	position = player->tell(player);
	printf("this song is %dms in length, and it has play %dms\n", duration, position);

	printf("we will seek to the half of this song now.\n");
	player->seek(player, duration / 2);

	position = player->tell(player);
	printf("now this song has play %dms\n", position);

	/* wait for playback complete */
	while (!isCompleted) {
		OS_MSleep(100);
	}

	/* stop it */
	player->stop(player);
	return 0;
}

static int play_flash_music(void)
{
	int ret;

	/*
	 * play media in flash.
	 * 1. in this example, we play 1.amr(in image/xr872)
	 * 2. we should add 1.amr to our image.cfg, so we can write it to flash
	 * 3. construct the url according to the offset and size of 1.amr.
	 *    in this example, the offset of 1.amr is (1024 * 1024 + 64) = 1048640,
	 *    and the size of 1.amr is 363750 bytes. so url is "flash://0?addr=1048640&length=363750"
	 * be attention, we will add 64 bytes before the bin in image.cfg, so the offset of 1.amr is (1024 * 1024 + 64)
	 */
	isCompleted = 0;
	player->set_callback(player, player_demo_callback, NULL);
	ret = player->play(player, "flash://0?addr=1048640&length=363750");
	if (ret != 0) {
		printf("music play fail.\n");
		return -1;
	}

	/* wait for playback complete */
	while (!isCompleted) {
		OS_MSleep(100);
	}

	/* stop it */
	player->stop(player);
	return 0;
}

static int play_fifo_music(void)
{
	vfs_file_t *vfs;
	int ret = 0;
	void *file_buffer;
	unsigned int act_read;
	struct AudioFifoS *audiofifo;

	/*
	 * play media by putting media data to player
	 * 1. only support mp3/amr/wav
	 */
	audiofifo = audio_fifo_create();
	if (audiofifo == NULL) {
		return -1;
	}

	file_buffer = (void *)malloc(1024);
	if (file_buffer == NULL) {
		ret = -1;
		goto err1;
	}

	vfs = vfs_open("data/music/1.mp3", VFS_RDONLY);
	if (vfs == NULL) {
		ret = -1;
		goto err2;
	}

	AudioFifoSetPlayer(audiofifo, player);
	AudioFifoStart(audiofifo);
	while (1) {
		act_read = vfs_read(vfs, file_buffer, 1024);
		AudioFifoPutData(audiofifo, file_buffer, act_read);
		if (act_read != 1024)
			break;
	}
	AudioFifoStop(audiofifo, false);

	vfs_close(vfs);
err2:
	free(file_buffer);
err1:
	audio_fifo_destroy(audiofifo);
	return ret;
}

static int play_pcm_music(void)
{
	vfs_file_t *vfs;
	int ret;
	unsigned int act_read;
	unsigned int pcm_buf_size;
	void *pcm_buf;
	struct pcm_config config;

	vfs = vfs_open("data/music/16000_1.pcm", VFS_RDONLY);
	if (vfs == NULL) {
		printf("open pcm file fail\n");
		return -1;
	}

	config.channels = 1;
	config.format = PCM_FORMAT_S16_LE;
	config.period_count = 2;
	config.period_size = 1024;
	config.rate = 16000;
	ret = snd_pcm_open(AUDIO_SND_CARD_DEFAULT, PCM_OUT, &config);
	if (ret != 0) {
		goto err1;
	}

	pcm_buf_size = config.channels * config.period_count * config.period_size;
	pcm_buf = malloc(pcm_buf_size);
	if (pcm_buf == NULL) {
		goto err2;
	}

	while (1) {
		act_read = vfs_read(vfs, pcm_buf, pcm_buf_size);
		if (act_read < 0) {
			printf("read fail.\n");
			break;
		}

		snd_pcm_write(AUDIO_SND_CARD_DEFAULT, pcm_buf, act_read);

		if (act_read != pcm_buf_size) {
			printf("reach file end\n");
			break;
		}
	}

	free(pcm_buf);
	snd_pcm_flush(AUDIO_SND_CARD_DEFAULT);
	snd_pcm_close(AUDIO_SND_CARD_DEFAULT, PCM_OUT);
	vfs_close(vfs);

	return 0;

err2:
	snd_pcm_close(AUDIO_SND_CARD_DEFAULT, PCM_OUT);
err1:
	vfs_close(vfs);
	return -1;
}

static void play_task(void *arg)
{
	player = player_create();
	if (player == NULL) {
		printf("player create fail.\n");
		goto exit;
	}

	printf("player create success.\n");
	printf("you can use it to play, pause, resume, set volume and so on.\n");

	printf("player set volume to 16. valid volume value is from 0~31\n");
	player->setvol(player, 16);

#if PLAYER_DRC_EN
	audio_filter_register(AUDIO_SND_CARD_DEFAULT, AUDIO_FILTER_DRC, (void *)&drc_user_params_example);
#endif

#if PLAYER_EQ_EN
	audio_filter_register(AUDIO_SND_CARD_DEFAULT, AUDIO_FILTER_EQ, (void *)&eq_user_params_example);
#endif

	while (1) {
		printf("===try to play media in file system===\n");
		play_file_music();

		printf("===try to play media in flash===\n");
		play_flash_music();

		printf("===try to play media by fifo===\n");
		play_fifo_music();

		printf("===try to play pcm by audio driver===\n");
		play_pcm_music();
	}

	player_destroy(player);

exit:
	OS_ThreadDelete(&play_thread);
}

int audio_play_start(void)
{
	if (OS_ThreadCreate(&play_thread,
	                    "play_task",
	                    play_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    PLAYER_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create fail.exit\n");
		return -1;
	}
	return 0;
}

