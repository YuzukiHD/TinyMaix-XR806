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

#ifndef _DEBUG_HEAP_TRACE_H_
#define _DEBUG_HEAP_TRACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_PSRAM_MALLOC_TRACE)

#define HEAP_MAGIC_LEN	4

typedef struct {
	int entry_cnt;
	int entry_cnt_max;
	size_t sum;
	size_t sum_max;
} heap_info_t;

#ifdef CONFIG_MALLOC_TRACE
void sram_heap_trace_info_show(int verbose);
void sram_heap_trace_info_get(heap_info_t *info);
#endif
#ifdef CONFIG_PSRAM_MALLOC_TRACE
void psram_heap_trace_info_show(int verbose);
void psram_heap_trace_info_get(heap_info_t *info);
#endif

void heap_trace_info_show(int verbose);
void heap_trace_info_get(heap_info_t *info);

int heap_trace_malloc(void *ptr, size_t size);
int heap_trace_free(void *ptr);
int heap_trace_realloc(void *old_ptr, void *new_ptr, size_t new_size);

#endif

#ifdef __cplusplus
}
#endif

#endif /*_DEBUG_HEAP_TRACE_H_*/
