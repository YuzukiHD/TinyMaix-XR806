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

#ifndef _AUDIO_PCM_H_
#define _AUDIO_PCM_H_

#include "sys/list.h"
#include "driver/chip/hal_snd_card.h"

/* Audio filter attribute define */
typedef enum {
	AUDIO_FILTER_DRC = 0, //must start from 0, and the other follow in sequence
	AUDIO_FILTER_EQ,
	AUDIO_FILTER_MAX,
} Audio_Filter_Type;

/* Audio filter stream direction define */
#define AUDIO_FILTER_DIR(fil) ((fil == AUDIO_FILTER_DRC || fil == AUDIO_FILTER_EQ) ? PCM_OUT :\
                              /*(fil == AUDIO_FILTER_***) ? PCM_IN :*/  \
                              0xff)

/* Audio filter function define */
struct audio_filter {
	//audio filter control
	void *handle;
	void *user_params;
	uint8_t user_params_size;
	Snd_Card_Num card_num;
	const char *filter_name;
	Audio_Filter_Type filter_type;

	//audio filter interface
	void* (*filter_create)(void *params);
	void  (*filter_destroy)(void *handle);
	void  (*filter_process)(void *handle, short *buf, int sample_nums);

	//audio filter node
	struct list_head node;
};


/********************************* Audio filter interface *************************************/

int audio_filter_register(Snd_Card_Num card_num, Audio_Filter_Type filter_type, void *user_params);
int audio_filter_unregister(Snd_Card_Num card_num, Audio_Filter_Type filter_type);


/********************************* Audio PCM interface *************************************/

int snd_pcm_init(void);
int snd_pcm_deinit(void);
int snd_pcm_write(Snd_Card_Num card_num, void *data, uint32_t count);
int snd_pcm_read(Snd_Card_Num card_num, void *data, uint32_t count);
int snd_pcm_flush(Snd_Card_Num card_num);
int snd_pcm_open(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg);
int snd_pcm_close(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir);

/**********************************************************************************************/

#endif

