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
#include "sys/defs.h"
#include "driver/chip/hal_i2s.h"
#include "driver/chip/hal_dmic.h"
#include "driver/chip/hal_i2c.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/drc/drc.h"
#include "audio/eq/eq.h"
#include "kernel/os/os_mutex.h"

#if (CONFIG_AUDIO_HEAP_MODE == 1)
#include "sys/sys_heap.h"
#else
#include <stdlib.h>
#endif

#define AUDIO_PCM_DEBUG_EN               0

#if AUDIO_PCM_DEBUG_EN
#define AUDIO_PCM_DEBUG(fmt, arg...)     printf("[AUDIO_PCM]"fmt, ##arg)
#define AUDIO_FILTER_DEBUG(fmt, arg...)  printf("[AUDIO_PCM Filter]"fmt, ##arg)
#else
#define AUDIO_PCM_DEBUG(fmt, arg...)
#define AUDIO_FILTER_DEBUG(fmt, arg...)
#endif

#define AUDIO_PCM_ERROR(fmt, arg...)     printf("[AUDIO_PCM]"fmt, ##arg)
#define AUDIO_FILTER_ERROR(fmt, arg...)  printf("[AUDIO_PCM Filter]"fmt, ##arg)
#define AUDIO_FILTER_ALWAYS(fmt, arg...) printf("[AUDIO_PCM Filter]"fmt, ##arg)

#define pcm_try_lock(m)                  OS_MutexLock(m, 0)
#define pcm_lock(m)                      OS_MutexLock(m, OS_WAIT_FOREVER)
#define pcm_unlock                       OS_MutexUnlock
#define pcm_lock_init                    OS_MutexCreate
#define pcm_lock_deinit                  OS_MutexDelete

LIST_HEAD_DEF(audio_pcm_list);
LIST_HEAD_DEF(audio_filter_list);

struct play_priv {
	uint8_t  *cache;
	uint32_t length;
	uint32_t half_buf_size;
	uint32_t frame_bytes;
};

struct cap_priv {
	uint8_t	 *cache;
	uint32_t length;
	uint32_t half_buf_size;
	uint32_t frame_bytes;
};

struct pcm_priv {
	//card num
	Snd_Card_Num card_num;

	//play/cap priv
	struct play_priv play_priv;
	struct cap_priv  cap_priv;

	//mutex
	OS_Mutex_t play_lock;
	OS_Mutex_t cap_lock;
	OS_Mutex_t write_lock;

	uint8_t play_open;
	uint8_t cap_open;
	uint8_t reserve[2];
	//list node
	struct list_head node;
};

/* Audio filter default user params define */
static const drc_user_params_t drc_user_params_def = {
	.rms_average_time = 1,  //RMS averaging time in ms,  typical value is 1ms
	.attack_time = 1,       //attack time in ms,   typical value is 1ms
	.release_time = 100,    //release time in ms,  typical value is 100ms
	.target_level = -12,    //target level in dB <=0
};

static const eq_core_prms_t eq_core_params_def[] = {
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

static const eq_user_params_t eq_user_params_def = {
	.biq_num = HAL_ARRAY_SIZE(eq_core_params_def),
	.core_params = (eq_core_prms_t *)&eq_core_params_def,
};

/* Audio filter function define */
static const struct audio_filter audio_filter_drc = {
	.user_params = (void *)&drc_user_params_def,
	.user_params_size = sizeof(drc_user_params_t),
	.filter_name = "DRC",
	.filter_type = AUDIO_FILTER_DRC,

	.filter_create = drc_create,
	.filter_destroy = drc_destroy,
	.filter_process = drc_process,
};

static const struct audio_filter audio_filter_eq = {
	.user_params = (void *)&eq_user_params_def,
	.user_params_size = sizeof(eq_user_params_t) + sizeof(eq_core_params_def),
	.filter_name = "EQ",
	.filter_type = AUDIO_FILTER_EQ,

	.filter_create = eq_create,
	.filter_destroy = eq_destroy,
	.filter_process = eq_process,
};

static const struct audio_filter *audio_filter_func[AUDIO_FILTER_MAX] = {
	[AUDIO_FILTER_DRC] = &audio_filter_drc,
	[AUDIO_FILTER_EQ]  = &audio_filter_eq,
};

void *pcm_zalloc(uint32_t size)
{
	void *p;

#if (CONFIG_AUDIO_HEAP_MODE == 1)
	p = psram_malloc(size);
#else
	p = malloc(size);
#endif

	if (p != NULL) {
		memset(p, 0, size);
		return p;
	}
	return NULL;
}

void pcm_free(void *p)
{
	if (p) {
#if (CONFIG_AUDIO_HEAP_MODE == 1)
		psram_free(p);
#else
		free(p);
#endif
		p = NULL;
	}
}

static struct pcm_priv *card_num_to_pcm_priv(Snd_Card_Num card_num)
{
	struct pcm_priv *audio_pcm_priv;

	if (!list_empty(&audio_pcm_list)) {
		list_for_each_entry(audio_pcm_priv, &audio_pcm_list, node) {
			if (audio_pcm_priv->card_num == card_num) {
				return audio_pcm_priv;
			}
		}
	}

	return NULL;
}

int audio_filter_register(Snd_Card_Num card_num, Audio_Filter_Type filter_type, void *user_params)
{
	struct audio_filter *audio_filter_ptr;

	/* Check audio filter attr to be valid */
	if (filter_type >= AUDIO_FILTER_MAX) {
		AUDIO_FILTER_ERROR("Invalid audio filter attr [%d]!\n", filter_type);
		return -1;
	}

	/* Check audio filter has been define or not */
	if (audio_filter_func[filter_type] == NULL) {
		AUDIO_FILTER_ERROR("This audio filter function [%d] hasn't been define!\n", filter_type);
		return -1;
	}

	/* Check the audio filter attached to the card has been register or not */
	if (!list_empty(&audio_filter_list)) {
		list_for_each_entry(audio_filter_ptr, &audio_filter_list, node) {
			if (audio_filter_ptr->filter_type == filter_type &&
			                 audio_filter_ptr->card_num == card_num) {
				AUDIO_FILTER_ERROR("Snd card-[%d] audio filter-[%s] has been register!\n",
					               card_num, audio_filter_ptr->filter_name);
				return -1;
			}
		}
	}

	/* Check audio filter user params ptr valid or not */
	if (user_params == NULL) {
		if (audio_filter_func[filter_type]->user_params) {
			user_params = (void *)audio_filter_func[filter_type]->user_params;
			AUDIO_FILTER_DEBUG("Audio filter-[%s] user params hasn't been config, use default.\n",
			                    audio_filter_func[filter_type]->filter_name);
		} else {
			AUDIO_FILTER_ERROR("Audio filter-[%s] user params hasn't been config!\n",
				                audio_filter_func[filter_type]->filter_name);
			return -1;
		}
	}

	/* Malloc audio filter buffer and init*/
	audio_filter_ptr = pcm_zalloc(sizeof(struct audio_filter));
	if (audio_filter_ptr == NULL) {
		AUDIO_FILTER_ERROR("Malloc audio filter buffer Fail!\n");
		return -1;
	}
	memcpy(audio_filter_ptr, audio_filter_func[filter_type], sizeof(struct audio_filter));
	audio_filter_ptr->card_num = card_num;

	/* Malloc audio filter user params buffer and int */
	if (filter_type == AUDIO_FILTER_EQ) {
		audio_filter_ptr->user_params_size = sizeof(eq_user_params_t) +
		        sizeof(eq_core_prms_t)*((eq_user_params_t *)user_params)->biq_num;
	}

	audio_filter_ptr->user_params = pcm_zalloc(audio_filter_ptr->user_params_size);
	if (audio_filter_ptr->user_params == NULL) {
		AUDIO_FILTER_ERROR("Malloc audio filter user params buffer Fail!\n");
		pcm_free(audio_filter_ptr);
		return -1;
	}

	if (filter_type == AUDIO_FILTER_EQ) {
		((eq_user_params_t *)audio_filter_ptr->user_params)->biq_num =
		                    ((eq_user_params_t *)user_params)->biq_num; //eq user params
		((eq_user_params_t *)audio_filter_ptr->user_params)->core_params =
		                    (eq_core_prms_t *)((char *)audio_filter_ptr->user_params +
		                     sizeof(eq_user_params_t));
		memcpy((char *)audio_filter_ptr->user_params + sizeof(eq_user_params_t),
		       ((eq_user_params_t *)user_params)->core_params,
		       sizeof(eq_core_prms_t)*((eq_user_params_t *)user_params)->biq_num); //eq core params array
	} else {
		memcpy(audio_filter_ptr->user_params, user_params, audio_filter_ptr->user_params_size);
	}

	//add to audio filter list
	list_add(&audio_filter_ptr->node, &audio_filter_list);
	AUDIO_FILTER_DEBUG("Sound card-[%d] register audio filter-[%s] success.\n",
	                   card_num, audio_filter_ptr->filter_name);

	return 0;
}

int audio_filter_unregister(Snd_Card_Num card_num, Audio_Filter_Type filter_type)
{
	int ret = -1;
	struct audio_filter *audio_filter_ptr;

	/* Check audio filter attr to be valid */
	if (filter_type >= AUDIO_FILTER_MAX) {
		AUDIO_FILTER_ERROR("Invalid audio filter attr [%d]!\n", filter_type);
		return -1;
	}

	/* Check audio filter list empty or not */
	if (list_empty(&audio_filter_list)) {
		AUDIO_FILTER_ERROR("Audio filter list is empty, don't need to unregister\n");
		return -1;
	}

	//Get audio filter to unregister
	list_for_each_entry(audio_filter_ptr, &audio_filter_list, node) {
		if (audio_filter_ptr->filter_type == filter_type &&
		                audio_filter_ptr->card_num == card_num) {
			list_del(&audio_filter_ptr->node);
			pcm_free(audio_filter_ptr->user_params);
			pcm_free(audio_filter_ptr);
			AUDIO_FILTER_DEBUG("Sound card [%d] unregister audio filter-[%s] success.\n",
			                   card_num, audio_filter_func[filter_type]->filter_name);
			ret = 0;
			break;
		}
	}

	return ret;
}

static int audio_filter_create_do(struct audio_filter *audio_filter_ptr, struct pcm_config *pcm_cfg)
{
	/* Create audio filter */
	if (audio_filter_ptr->filter_type == AUDIO_FILTER_DRC) {
		/* audio filter -> DRC */
		drc_core_params_t drc_core_params;
		drc_user_params_t *drc_user_params = (drc_user_params_t *)audio_filter_ptr->user_params;

		//Init drc core params
		drc_core_params.sampling_rate = pcm_cfg->rate;
		drc_core_params.channel_nums = pcm_cfg->channels;
		drc_core_params.rms_average_time = drc_user_params->rms_average_time;
		drc_core_params.attack_time = drc_user_params->attack_time;
		drc_core_params.release_time = drc_user_params->release_time;
		drc_core_params.target_level = drc_user_params->target_level;
		drc_core_params.noise_threshold = -45;
		drc_core_params.max_gain = 0;
		drc_core_params.min_gain = drc_user_params->target_level;

		//Creat audio filter
		audio_filter_ptr->handle = audio_filter_ptr->filter_create(&drc_core_params);

	} else if (audio_filter_ptr->filter_type == AUDIO_FILTER_EQ) {
		/* audio filter -> EQ */

		eq_prms_t eq_create_params;
		eq_user_params_t *eq_user_params = (eq_user_params_t *)audio_filter_ptr->user_params;

		//Init eq create params
		eq_create_params.sampling_rate = pcm_cfg->rate;
		eq_create_params.chan = pcm_cfg->channels;
		eq_create_params.biq_num = eq_user_params->biq_num;
		eq_create_params.core_prms = eq_user_params->core_params;

		//Creat audio filter
		audio_filter_ptr->handle = audio_filter_ptr->filter_create(&eq_create_params);
	}

	if (audio_filter_ptr->handle) {
		AUDIO_FILTER_ALWAYS("Snd card-[%d] audio filter-[%s] create success.\n",
		               audio_filter_ptr->card_num, audio_filter_ptr->filter_name);
		return 0;
	} else {
		AUDIO_FILTER_ERROR("Snd card-[%d] audio filter-[%s] create Fail!.\n",
		              audio_filter_ptr->card_num, audio_filter_ptr->filter_name);
		return -1;
	}
}

static void audio_filter_create(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg)
{
	struct audio_filter *audio_filter_ptr;

	/* Check pcm config pointer to be valid */
	if (pcm_cfg == NULL) {
		AUDIO_FILTER_ERROR("Pcm config pointer is NULL!");
		return;
	}

	/* Search audio filter matched to create */
	if (!list_empty(&audio_filter_list)) {
		list_for_each_entry(audio_filter_ptr, &audio_filter_list, node) {
			if (AUDIO_FILTER_DIR(audio_filter_ptr->filter_type) == stream_dir &&
			          audio_filter_ptr->card_num == card_num) {
				 audio_filter_create_do(audio_filter_ptr, pcm_cfg);
			}
		}
	}
}

static void audio_filter_destroy(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir)
{
	struct audio_filter *audio_filter_ptr;

	/* Search audio filter matched to destroy */
	if (!list_empty(&audio_filter_list)) {
		list_for_each_entry(audio_filter_ptr, &audio_filter_list, node) {
			if (AUDIO_FILTER_DIR(audio_filter_ptr->filter_type) == stream_dir &&
			           audio_filter_ptr->card_num == card_num) {
				audio_filter_ptr->filter_destroy(audio_filter_ptr->handle);
				audio_filter_ptr->handle = NULL;
				AUDIO_FILTER_DEBUG("Snd card-[%d] audio filter-[%s] destroy success.\n",
				       audio_filter_ptr->card_num, audio_filter_ptr->filter_name);
			}
		}
	}
}

static int audio_filter_process(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, uint8_t *buf, uint32_t size)
{
	int sample_nums;
	struct pcm_priv *audio_pcm_priv;
	struct audio_filter *audio_filter_ptr;

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_FILTER_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Check buf and  size to be valid*/
	if (buf == NULL || !size) {
		AUDIO_FILTER_ERROR("Invalid buffer or buf_size!\n");
		return -1;
	}

	/* Calculate sample numbers */
	if (stream_dir == PCM_OUT) {
		//according to half_buf_size
		sample_nums = size / audio_pcm_priv->play_priv.half_buf_size *
		              audio_pcm_priv->play_priv.half_buf_size / audio_pcm_priv->play_priv.frame_bytes;
	} else {
		sample_nums = size / audio_pcm_priv->cap_priv.frame_bytes;
	}

	/* Check sample_nums to be valid */
	if (!sample_nums) {
		//AUDIO_FILTER_DEBUG("Invalid sample numbers [%d]!\n",sample_nums);
		return -1;
	}

	/* Process all play/record audio filter */
	if (!list_empty(&audio_filter_list)) {
		list_for_each_entry(audio_filter_ptr, &audio_filter_list, node) {
			if (AUDIO_FILTER_DIR(audio_filter_ptr->filter_type) == stream_dir &&
			                     audio_filter_ptr->card_num == card_num){
				if (audio_filter_ptr->handle) {
					audio_filter_ptr->filter_process(audio_filter_ptr->handle, (short *)buf, sample_nums);
				}
			}
		}
	}

	return 0;
}

int snd_pcm_read(Snd_Card_Num card_num, void *data, uint32_t count)
{
	int ret;
	uint8_t *data_ptr;
	struct cap_priv *cpriv;
	struct pcm_priv *audio_pcm_priv;
	uint32_t half_buf_size, read_remain, hw_read;

	/* Check parms to be valid */
	if (!data || !count) {
		AUDIO_PCM_ERROR("Invalid data/count params!\n");
		return -1;
	}

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Check cap cache */
	if (audio_pcm_priv->cap_priv.cache == NULL) {
		AUDIO_PCM_ERROR("Capture Cache is NULL!\n");
		return -1;
	}

	/* Init misc */
	data_ptr = data;
	read_remain = count;
	cpriv = &audio_pcm_priv->cap_priv;
	half_buf_size = audio_pcm_priv->cap_priv.half_buf_size;

	/* Cache has data to read */
	if (cpriv->length) {
		if (cpriv->length > read_remain) {
			memcpy(data_ptr, cpriv->cache, read_remain);
			cpriv->length -= read_remain;
			memcpy(cpriv->cache, cpriv->cache + read_remain, cpriv->length);
			return count;
		} else {
			memcpy(data_ptr, cpriv->cache, cpriv->length);
			data_ptr += cpriv->length;
			read_remain -= cpriv->length;
			cpriv->length = 0;
			if (!read_remain) {
				return count;
			}
		}
	}

	/* read integer half_buf_size */
	if (read_remain >= half_buf_size) {
		hw_read = (read_remain / half_buf_size) * half_buf_size;
		ret = HAL_SndCard_PcmRead(card_num, data_ptr, hw_read);
		if (ret != hw_read) {
			AUDIO_PCM_ERROR("PCM read error!\n");
			if (ret > 0) {
				audio_filter_process(card_num, PCM_IN, data_ptr, ret);
			}
			return ret > 0 ? count - read_remain + ret : count - read_remain;
		}
		audio_filter_process(card_num, PCM_IN, data_ptr, hw_read);
		data_ptr += hw_read;
		read_remain -= hw_read;
		if (!read_remain) {
			return count;
		}
	}

	/* read remain */
	ret = HAL_SndCard_PcmRead(card_num, cpriv->cache, half_buf_size);
	if (ret != half_buf_size) {
		AUDIO_PCM_ERROR("PCM read half_buf_size error!\n");
		return count-read_remain;
	}
	audio_filter_process(card_num, PCM_IN, cpriv->cache, half_buf_size);
	memcpy(data_ptr, cpriv->cache, read_remain);
	cpriv->length = half_buf_size-read_remain;
	memcpy(cpriv->cache, cpriv->cache + read_remain, cpriv->length);

	return count;
}

int snd_pcm_write(Snd_Card_Num card_num, void *data, uint32_t count)
{
	int ret;
	uint8_t *data_ptr;
	struct play_priv *ppriv;
	struct pcm_priv *audio_pcm_priv;
	uint32_t  half_buf_size, write_size, cache_remain = 0;

	/* Check parms to be valid */
	if (!data || !count) {
		AUDIO_PCM_ERROR("Invalid data/count params!\n");
		return -1;
	}

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Check play cache */
	if (audio_pcm_priv->play_priv.cache == NULL) {
		AUDIO_PCM_ERROR("Play Cache is NULL!\n");
		return -1;
	}

	/* Get write lock */
	if (pcm_try_lock(&audio_pcm_priv->write_lock) != OS_OK) {
		AUDIO_PCM_ERROR("Obtain write lock err.\n");
		return -1;
	}

	/* Init misc */
	data_ptr = data;
	write_size = count;
	ppriv = &audio_pcm_priv->play_priv;
	half_buf_size = audio_pcm_priv->play_priv.half_buf_size;

	/* Cache has data to write */
	if (ppriv->length) {
		cache_remain = half_buf_size - ppriv->length;
		if (cache_remain > write_size) {
			memcpy(ppriv->cache + ppriv->length, data_ptr, write_size);
			ppriv->length += write_size;
			pcm_unlock(&audio_pcm_priv->write_lock);
			return count;
		} else {
			memcpy(ppriv->cache + ppriv->length, data_ptr, cache_remain);
			audio_filter_process(card_num, PCM_OUT, ppriv->cache, half_buf_size);
			HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
			ppriv->length = 0;
			data_ptr += cache_remain;
			write_size -= cache_remain;
			if (!write_size) {
				pcm_unlock(&audio_pcm_priv->write_lock);
				return count;
			}
		}
	}

	/* Pcm write */
	audio_filter_process(card_num, PCM_OUT, data_ptr, write_size);
	ret = HAL_SndCard_PcmWrite(card_num, data_ptr, write_size);
	if (ret > 0 && ret < write_size) {
		//remain some data that not enough half_buf_size data to write, save to cache
		data_ptr += ret;
		ppriv->length = write_size - ret;
		memcpy(ppriv->cache, data_ptr, ppriv->length);
	} else if (ret == write_size) {
		//all data write complete
		ppriv->length = 0;
	} else if (ret == 0) {
		//ret=0, not enough half_buf_size data to write, save to cache
		ppriv->length = write_size < half_buf_size ? write_size : half_buf_size;
		memcpy(ppriv->cache, data_ptr, ppriv->length);
	} else {
		//ret<0, write fail
		pcm_unlock(&audio_pcm_priv->write_lock);
		return cache_remain ? cache_remain : -1;
	}

	pcm_unlock(&audio_pcm_priv->write_lock);
	return count;
}

int snd_pcm_flush(Snd_Card_Num card_num)
{
	uint32_t i, half_buf_size;
	struct play_priv *ppriv;
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n", __func__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Check play cache */
	if (audio_pcm_priv->play_priv.cache == NULL) {
		AUDIO_PCM_ERROR("Play Cache is NULL!\n");
		return -1;
	}

	/* Get write lock */
	if (pcm_try_lock(&audio_pcm_priv->write_lock) != OS_OK) {
		AUDIO_PCM_ERROR("Obtain write lock err.\n");
		return -1;
	}

	/* Init misc */
	ppriv = &audio_pcm_priv->play_priv;
	half_buf_size = audio_pcm_priv->play_priv.half_buf_size;

	/* Cache has data to write */
	if (ppriv->length) {
		memset(ppriv->cache + ppriv->length, 0, half_buf_size - ppriv->length);
		audio_filter_process(card_num, PCM_OUT, ppriv->cache, half_buf_size);
		HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
		ppriv->length = 0;
	}

	/* play void frames */
	memset(ppriv->cache, 0, half_buf_size);
	for (i = 0; i < 2; i++) {
		//zero data, don't need to process throuth audio filter
		HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
	}

	pcm_unlock(&audio_pcm_priv->write_lock);
	return 0;
}

int snd_pcm_open(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg)
{
	uint32_t buf_size;
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n", __func__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Init audio_pcm_priv */
	if (stream_dir == PCM_OUT) {
		//play lock
		if (pcm_try_lock(&audio_pcm_priv->play_lock) != OS_OK) {
			AUDIO_PCM_ERROR("Obtain play lock err...\n");
			return -1;
		}
		if (audio_pcm_priv->play_open) {
			pcm_unlock(&audio_pcm_priv->play_lock);
			AUDIO_PCM_ERROR("play already open\n");
			return -1;
		}

		// Malloc play cache buffer
		buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));
		audio_pcm_priv->play_priv.cache = pcm_zalloc(buf_size/2);
		if (audio_pcm_priv->play_priv.cache == NULL) {
			pcm_unlock(&audio_pcm_priv->play_lock);
			AUDIO_PCM_ERROR("obtain play cache failed...\n");
			return -1;
		}
		audio_pcm_priv->play_priv.length = 0;
		audio_pcm_priv->play_priv.half_buf_size = buf_size/2;
		audio_pcm_priv->play_priv.frame_bytes = pcm_frames_to_bytes(pcm_cfg, 1);
	} else {
		//cap lock
		if (pcm_try_lock(&audio_pcm_priv->cap_lock) != OS_OK) {
			AUDIO_PCM_ERROR("obtain cap lock err...\n");
			return -1;
		}
		if (audio_pcm_priv->cap_open) {
			pcm_unlock(&audio_pcm_priv->cap_lock);
			AUDIO_PCM_ERROR("capture already open\n");
			return -1;
		}
		// Malloc capture cache buffer
		buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));
		audio_pcm_priv->cap_priv.cache = pcm_zalloc(buf_size/2);
		if (audio_pcm_priv->cap_priv.cache == NULL) {
			pcm_unlock(&audio_pcm_priv->cap_lock);
			AUDIO_PCM_ERROR("obtain cap cache failed...\n");
			return -1;
		}
		audio_pcm_priv->cap_priv.length = 0;
		audio_pcm_priv->cap_priv.half_buf_size = buf_size/2;
		audio_pcm_priv->cap_priv.frame_bytes = pcm_frames_to_bytes(pcm_cfg, 1);
	}

	/* Open snd card */
	if (HAL_SndCard_Open(card_num, stream_dir, pcm_cfg) != HAL_OK) {
		AUDIO_PCM_ERROR("Sound card-[%d] open Fail!\n", card_num);
		if (stream_dir == PCM_OUT) {
			pcm_free(audio_pcm_priv->play_priv.cache);
			memset(&(audio_pcm_priv->play_priv), 0, sizeof(struct play_priv));
			pcm_unlock(&audio_pcm_priv->play_lock);
		} else {
			pcm_free(audio_pcm_priv->cap_priv.cache);
			memset(&(audio_pcm_priv->cap_priv), 0, sizeof(struct cap_priv));
			pcm_unlock(&audio_pcm_priv->cap_lock);
		}
		return -1;
	}

	/* Create all play/record audio filter */
	audio_filter_create(card_num, stream_dir, pcm_cfg);

	if (stream_dir == PCM_OUT) {
		audio_pcm_priv->play_open = 1;
		pcm_unlock(&audio_pcm_priv->play_lock);
	} else {
		audio_pcm_priv->cap_open = 1;
		pcm_unlock(&audio_pcm_priv->cap_lock);
	}

	return 0;
}

int snd_pcm_close(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir)
{
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n", __func__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if (audio_pcm_priv == NULL) {
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n", (uint8_t)card_num);
		return -1;
	}

	/* Close snd card */
	HAL_SndCard_Close(card_num, stream_dir);

	/* deinit audio_pcm_priv */
	if (stream_dir == PCM_OUT) {
		pcm_free(audio_pcm_priv->play_priv.cache);
		memset(&(audio_pcm_priv->play_priv), 0, sizeof(struct play_priv));
	} else {
		pcm_free(audio_pcm_priv->cap_priv.cache);
		memset(&(audio_pcm_priv->cap_priv), 0, sizeof(struct cap_priv));
	}

	/* Destroy all play/record audio filter */
	audio_filter_destroy(card_num, stream_dir);

	if (stream_dir == PCM_OUT) {
		pcm_lock(&audio_pcm_priv->play_lock);
		audio_pcm_priv->play_open = 0;
		pcm_unlock(&audio_pcm_priv->play_lock);
	} else {
		pcm_lock(&audio_pcm_priv->cap_lock);
		audio_pcm_priv->cap_open = 0;
		pcm_unlock(&audio_pcm_priv->cap_lock);
	}

	return 0;
}

int snd_pcm_init(void)
{
	char *pcm_priv_temp;
	uint8_t *card_num, card_nums, i;
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n", __func__);

	/* Get snd card nums */
	card_nums = HAL_SndCard_GetCardNums();
	if (!card_nums) {
		AUDIO_PCM_ERROR("card nums is 0!\n");
		return -1;
	}

	/* Malloc buffer and init */
	pcm_priv_temp = (char  *)pcm_zalloc(sizeof(struct pcm_priv) * card_nums);
	card_num = (uint8_t *)pcm_zalloc(sizeof(uint8_t) * card_nums);

	if (!pcm_priv_temp || !card_num) {
		AUDIO_PCM_ERROR("Malloc audio pcm buffer Fail!\n");
		pcm_free(pcm_priv_temp);
		pcm_free(card_num);
		return -1;
	}

	HAL_SndCard_GetAllCardNum(card_num);

	/* Init struct pcm_priv and list add */
	for (i = 0; i < card_nums; i++) {
		//Init struct pcm_priv
		audio_pcm_priv = (struct pcm_priv *)(pcm_priv_temp + sizeof(struct pcm_priv)*i);
		audio_pcm_priv->card_num = (Snd_Card_Num)card_num[i];
		pcm_lock_init(&audio_pcm_priv->play_lock);
		pcm_lock_init(&audio_pcm_priv->cap_lock);
		pcm_lock_init(&audio_pcm_priv->write_lock);

		//list add
		list_add(&audio_pcm_priv->node, &audio_pcm_list);
	}

	/* Free get card num buffer */
	pcm_free(card_num);

	return 0;
}

int snd_pcm_deinit(void)
{
	struct pcm_priv *audio_pcm_priv, *next;
	AUDIO_PCM_DEBUG("--->%s\n", __func__);

	/* Check audio pcm list empty or not */
	if (list_empty(&audio_pcm_list)) {
		AUDIO_PCM_DEBUG("Audio pcm list is empty, don't need to deinit\n");
		return 0;
	}

	/* Get audio pcm priv to deinit */
	list_for_each_entry_safe(audio_pcm_priv, next, &audio_pcm_list, node) {
		//audio pcm priv deinit
		pcm_lock_deinit(&audio_pcm_priv->play_lock);
		pcm_lock_deinit(&audio_pcm_priv->cap_lock);
		pcm_lock_deinit(&audio_pcm_priv->write_lock);

		//delete the audio pcm list and free audio pcm priv buffer
		list_del(&audio_pcm_priv->node);
		pcm_free(audio_pcm_priv);
	}

	return 0;
}

