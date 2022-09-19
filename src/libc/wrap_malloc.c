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
#include <sys/reent.h>
#include <stdint.h>
#include <stddef.h>
#include "sys/sys_heap.h"

extern uint8_t __heap_start__[];
extern uint8_t __heap_end__[];

static __always_inline int is_rangeof_sramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__heap_start__, (uint32_t)__heap_end__);
}

#ifdef CONFIG_PSRAM
#include "driver/chip/psram/psram.h"

static __always_inline int is_rangeof_psramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__psram_end__, PSRAM_END_ADDR);
}
#endif /* CONFIG_PSRAM */

#if (CONFIG_MALLOC_MODE == 0x00)

#ifdef CONFIG_MALLOC_TRACE
#include "debug/heap_trace.h"
#endif

void *__real__malloc_r(struct _reent *reent, size_t size);
void *__real__realloc_r(struct _reent *reent, void *ptr, size_t size);
void __real__free_r(struct _reent *reent, void *ptr);

#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_MIX_HEAP_MANAGE)
static uint8_t g_do_reallocing = 0; /* flag for realloc() */
#endif
void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	void *ptr;
	size_t real_size = size;

	malloc_mutex_lock();

#ifdef CONFIG_MALLOC_TRACE
	if (!g_do_reallocing) {
		real_size += HEAP_MAGIC_LEN;
	}
#endif

	ptr = __real__malloc_r(reent, real_size);
#ifdef CONFIG_MIX_HEAP_MANAGE
	if (!g_do_reallocing && ptr == NULL) {
		ptr = psram_malloc(size);
		malloc_mutex_unlock();
		return ptr;
	}
#endif
#ifdef CONFIG_MALLOC_TRACE
	if (!g_do_reallocing) {
		heap_trace_malloc(ptr, real_size - HEAP_MAGIC_LEN);
	}
#endif

	malloc_mutex_unlock();
	return ptr;
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	void *new_ptr = NULL;
	size_t real_size = size;

	malloc_mutex_lock();

#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_MIX_HEAP_MANAGE)
	g_do_reallocing = 1;
#endif
#ifdef CONFIG_MALLOC_TRACE
	if (size != 0) {
		real_size += HEAP_MAGIC_LEN;
	}
#endif
#ifndef CONFIG_MIX_HEAP_MANAGE
	new_ptr = __real__realloc_r(reent, ptr, real_size);
#else
	if (size != 0) {
		if (ptr == NULL) {
			new_ptr = __real__realloc_r(reent, ptr, real_size);
			if (new_ptr == NULL) {
				new_ptr = psram_malloc(size);
				goto out;
			}
		} else if (is_rangeof_sramheap(ptr)) {
			new_ptr = __real__realloc_r(reent, ptr, real_size);
			if (new_ptr == NULL) {
				new_ptr = psram_malloc(size);
				if (new_ptr == NULL) {
					goto out;
				} else {
					memcpy(new_ptr, ptr, size);
					__real__free_r(reent, ptr);
#ifdef CONFIG_MALLOC_TRACE
					heap_trace_free(ptr);
#endif
					goto out;
				}
			}
		} else if (is_rangeof_psramheap(ptr)) {
			new_ptr = psram_realloc(ptr, size);
			goto out;
		}
	} else {
		if (is_rangeof_sramheap(ptr)) {
			new_ptr = __real__realloc_r(reent, ptr, real_size);
		} else if (is_rangeof_psramheap(ptr)) {
			new_ptr = psram_realloc(ptr, size);
			goto out;
		}
	}
#endif
#ifdef CONFIG_MALLOC_TRACE
	if (size == 0) {
		heap_trace_realloc(ptr, new_ptr, size);
	} else {
		heap_trace_realloc(ptr, new_ptr, real_size - HEAP_MAGIC_LEN);
	}
#endif

#ifdef CONFIG_MIX_HEAP_MANAGE
out:
#endif
#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_MIX_HEAP_MANAGE)
	g_do_reallocing = 0;
#endif

	malloc_mutex_unlock();

	return new_ptr;
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
	malloc_mutex_lock();
#ifdef CONFIG_MIX_HEAP_MANAGE
	if (is_rangeof_psramheap(ptr)) {
		psram_free(ptr);
		malloc_mutex_unlock();
		return;
	}
#endif
#ifdef CONFIG_MALLOC_TRACE
	if (!g_do_reallocing) {
		heap_trace_free(ptr);
	}
#endif
	__real__free_r(reent, ptr);
#ifdef CONFIG_HEAP_FREE_CHECK
	HEAP_ASSERT(is_rangeof_sramheap(ptr));
#endif
	malloc_mutex_unlock();
}

#else

#include "sys/sram_heap.h"

void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	void *ptr;

	ptr = sram_malloc(size);
#ifdef CONFIG_MIX_HEAP_MANAGE
	if (ptr == NULL) {
		ptr = psram_malloc(size);
	}
#endif
	return ptr;
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	void *new_ptr = NULL;

#ifndef CONFIG_MIX_HEAP_MANAGE
	new_ptr = sram_realloc(ptr, size);
#else
	if (size != 0) {
		if (ptr == NULL) {
			new_ptr = sram_realloc(ptr, size);
			if (new_ptr == NULL) {
				new_ptr = psram_realloc(ptr, size);
			}
		} else if (is_rangeof_sramheap(ptr)) {
			new_ptr = sram_realloc(ptr, size);
			if (new_ptr == NULL) {
				new_ptr = psram_malloc(size);
				if (new_ptr != NULL) {
					memcpy(new_ptr, ptr, size);
					sram_free(ptr);
				}
			}
		} else if (is_rangeof_psramheap(ptr)) {
			new_ptr = psram_realloc(ptr, size);
		}
	} else {
		if (is_rangeof_sramheap(ptr)) {
			new_ptr = sram_realloc(ptr, size);
		} else if (is_rangeof_psramheap(ptr)) {
			new_ptr = psram_realloc(ptr, size);
		}
	}
#endif
	return new_ptr;
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
#ifndef CONFIG_MIX_HEAP_MANAGE
	sram_free(ptr);
#else
	if (is_rangeof_sramheap(ptr)) {
		sram_free(ptr);
	} else if (is_rangeof_psramheap(ptr)) {
		psram_free(ptr);
	}
#endif
}

#endif
