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

#ifndef DRC_H_
#define DRC_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* dynamic range control core parameters */
typedef struct {
	int sampling_rate;      /* sampling rate */
	int channel_nums;       /* channel numbers */
	int rms_average_time;   /* RMS averaging time in ms */
	int attack_time;        /* attack time in ms */
	int release_time;       /* release time in ms */
	int target_level;       /* target level in dB, <=0 */
	int noise_threshold;    /* noise threshold in dB, <=0 */
	int max_gain;           /* DRC process max gain in dB */
	int min_gain;           /* DRC process min gain in dB */
} drc_core_params_t;


/* DRC user config params */
typedef struct {
	int rms_average_time;   /* RMS averaging time in ms */
	int attack_time;        /* attack time in ms */
	int release_time;       /* release time in ms */
	int target_level;       /* target level in dB, <=0 */
} drc_user_params_t;


/**
 * @brief Create DRC module
 * @param[in] params DRC core configuration parameter
 * @retval Pointer to the DRC handle, NULL on failure
 */
void *drc_create(void *params);

/**
 * @brief DRC process
 * @handle[in] handle DRC module handle
 * @handle[in,out] xout Sample data for DRC process
 * @handle[in] samplenum Number of sample data
 * @retval None
 *
 * @note samplenum = TotalLengthOfDataProcess/2 for mono,
 *                    and TotalLengthOfDataProcess/4 for stereo
 */
void drc_process(void *handle, short *xout, int samplenum);

/**
 * @brief Destroy DRC module
 * @param[in] handle DRC module handle
 * @retval None
 */
void drc_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif //DRC_H_

