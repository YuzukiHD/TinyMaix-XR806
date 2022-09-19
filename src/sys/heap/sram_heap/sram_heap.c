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
#include <stdlib.h>
#include <string.h>
#include "sys/sys_heap.h"
#include "kernel/os/os_thread.h"
#ifdef CONFIG_MALLOC_TRACE
#include "debug/heap_trace.h"
#endif

#if (CONFIG_MALLOC_MODE == 0x00)

void *sram_malloc(size_t size)
{
	return malloc(size);
}

void sram_free(void *ptr)
{
	free(ptr);
}

void *sram_realloc(void *ptr, size_t size)
{
	return realloc(ptr, size);
}

void *sram_calloc(size_t nmemb, size_t size)
{
	return calloc(nmemb, size);
}

#elif (CONFIG_MALLOC_MODE == 0x01)

extern uint8_t __heap_start__[];
extern uint8_t __heap_end__[];

static sys_heap_t g_sram_heap;
static int sram_heap_init_flag = 0;

static __always_inline int is_rangeof_sramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__heap_start__, (uint32_t)__heap_end__);
}

static int32_t sram_heap_init(void)
{
	uint8_t *sram_heap_base;
	size_t sram_heap_len;

	sram_heap_base = (uint8_t *)__heap_start__;
	sram_heap_len = (size_t)(__heap_end__ - __heap_start__);
	sys_heap_default_init(&g_sram_heap, sram_heap_base, sram_heap_len);
	g_sram_heap.portByte_Alignment = 8;
	g_sram_heap.portByte_Alignment_Mask = 0x7;
	sys_heap_init(&g_sram_heap);
	return 0;
}

void *sram_malloc(size_t size)
{
	void *ptr;

	if (sram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (sram_heap_init_flag == 0) {
			sram_heap_init();
			sram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

#ifdef CONFIG_MALLOC_TRACE
	size += HEAP_MAGIC_LEN;
#endif

	ptr = sys_heap_malloc(&g_sram_heap, size);

#ifdef CONFIG_MALLOC_TRACE
	heap_trace_malloc(ptr, size - HEAP_MAGIC_LEN);
#endif
	return ptr;
}

void sram_free(void *ptr)
{
#ifdef CONFIG_MALLOC_TRACE
	heap_trace_free(ptr);
#endif
	sys_heap_free(&g_sram_heap, ptr);
#ifdef CONFIG_HEAP_FREE_CHECK
	HEAP_ASSERT(is_rangeof_sramheap(ptr));
#endif
}

void *sram_realloc(void *ptr, size_t size)
{
	void *new_ptr;

	if (sram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (sram_heap_init_flag == 0) {
			sram_heap_init();
			sram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

#ifdef CONFIG_MALLOC_TRACE
	malloc_mutex_lock();
	if (size != 0) {
		size += HEAP_MAGIC_LEN;
	}
#endif

	new_ptr = sys_heap_realloc(&g_sram_heap, ptr, size);

#ifdef CONFIG_MALLOC_TRACE
	if (size == 0) {
		heap_trace_realloc(ptr, new_ptr, size);
	} else {
		heap_trace_realloc(ptr, new_ptr, size - HEAP_MAGIC_LEN);
	}
	malloc_mutex_unlock();
#endif

	return new_ptr;
}

void *sram_calloc(size_t nmemb, size_t size)
{
	void *ptr;

	ptr = sram_malloc(nmemb * size);
	if (ptr != NULL) {
		memset(ptr, 0, nmemb * size);
	}
	return ptr;
}

size_t sram_total_heap_size(void)
{
	return __heap_end__ - __heap_start__;
}

size_t sram_free_heap_size(void)
{
	if (sram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (sram_heap_init_flag == 0) {
			sram_heap_init();
			sram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

	return sys_heap_xPortGetFreeHeapSize(&g_sram_heap);
}

size_t sram_minimum_ever_free_heap_size(void)
{
	if (sram_heap_init_flag == 0) {
		malloc_mutex_lock();
		if (sram_heap_init_flag == 0) {
			sram_heap_init();
			sram_heap_init_flag = 1;
		}
		malloc_mutex_unlock();
	}

	return sys_heap_xPortGetMinimumEverFreeHeapSize(&g_sram_heap);
}

#endif
