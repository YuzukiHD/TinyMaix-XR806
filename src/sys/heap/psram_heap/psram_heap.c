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

#ifdef CONFIG_PSRAM

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/sys_heap.h"
#include "driver/chip/psram/psram.h"
#include "kernel/os/os_thread.h"
#ifdef CONFIG_PSRAM_MALLOC_TRACE
#include "debug/heap_trace.h"
#endif

static sys_heap_t g_psram_heap;
static int psram_heap_init_flag = 0;

static __always_inline int is_rangeof_psramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__psram_end__, PSRAM_END_ADDR);
}

static int32_t psram_heap_init(void)
{
	uint8_t *psram_heap_base;
	size_t psram_heap_len;

	psram_heap_base = (uint8_t *)__psram_end__;
	psram_heap_len = (size_t)(PSRAM_END_ADDR - (size_t)__psram_end__ - 17);
	sys_heap_default_init(&g_psram_heap, psram_heap_base, psram_heap_len);
	sys_heap_init(&g_psram_heap);
	return 0;
}

void *psram_malloc(size_t size)
{
	void *ptr;

	if (psram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (psram_heap_init_flag == 0) {
			psram_heap_init();
			psram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

#ifdef CONFIG_PSRAM_MALLOC_TRACE
	size += HEAP_MAGIC_LEN;
#endif

	ptr = sys_heap_malloc(&g_psram_heap, size);

#ifdef CONFIG_PSRAM_MALLOC_TRACE
	heap_trace_malloc(ptr, size - HEAP_MAGIC_LEN);
#endif

	return ptr;
}

void psram_free(void *ptr)
{
#ifdef CONFIG_PSRAM_MALLOC_TRACE
	heap_trace_free(ptr);
#endif
	sys_heap_free(&g_psram_heap, ptr);
#ifdef CONFIG_HEAP_FREE_CHECK
	HEAP_ASSERT(is_rangeof_psramheap(ptr));
#endif
}

void *psram_realloc(void *ptr, size_t size)
{
	void *new_ptr;

	if (psram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (psram_heap_init_flag == 0) {
			psram_heap_init();
			psram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

#ifdef CONFIG_PSRAM_MALLOC_TRACE
	malloc_mutex_lock();
	if (size != 0) {
		size += HEAP_MAGIC_LEN;
	}
#endif

	new_ptr = sys_heap_realloc(&g_psram_heap, ptr, size);

#ifdef CONFIG_PSRAM_MALLOC_TRACE
	if (size == 0) {
		heap_trace_realloc(ptr, new_ptr, size);
	} else {
		heap_trace_realloc(ptr, new_ptr, size - HEAP_MAGIC_LEN);
	}
	malloc_mutex_unlock();
#endif

	return new_ptr;
}

void *psram_calloc(size_t nmemb, size_t size)
{
	void *ptr;

	ptr = psram_malloc(nmemb * size);
	if (ptr != NULL) {
		memset(ptr, 0, nmemb * size);
	}
	return ptr;
}

size_t psram_total_heap_size(void)
{
	return PSRAM_END_ADDR - (size_t)__psram_end__ - 17;
}

size_t psram_free_heap_size(void)
{
	if (psram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (psram_heap_init_flag == 0) {
			psram_heap_init();
			psram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

	return sys_heap_xPortGetFreeHeapSize(&g_psram_heap);
}

size_t psram_minimum_ever_free_heap_size(void)
{
	if (psram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (psram_heap_init_flag == 0) {
			psram_heap_init();
			psram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

	return sys_heap_xPortGetMinimumEverFreeHeapSize(&g_psram_heap);
}

void psram_heap_get_space(uint8_t **start, uint8_t **end, uint8_t **current)
{
	*start = (uint8_t *)__psram_end__;
	*end = (uint8_t *)PSRAM_END_ADDR;
	*current = (uint8_t *)(PSRAM_END_ADDR - psram_minimum_ever_free_heap_size());
}

#endif
