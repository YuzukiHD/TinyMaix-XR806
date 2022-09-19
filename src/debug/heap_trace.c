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

#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_PSRAM_MALLOC_TRACE)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sys/sys_heap.h"
#include "debug/heap_trace.h"
#include "driver/chip/psram/psram.h"
#include "debug/backtrace.h"

#ifdef CONFIG_MALLOC_TRACE
#define SRAM_HEAP_TRACE
#endif

#ifdef CONFIG_PSRAM_MALLOC_TRACE
#define PSRAM_HEAP_TRACE
#endif

#ifdef CONFIG_BACKTRACE
#define HEAP_BACKTRACE
#endif

#define HEAP_MEM_DBG_ON         0
#define HEAP_MEM_ERR_ON         1

#define HEAP_MEM_DBG_MIN_SIZE   100
#define HEAP_MEM_MAX_CNT        1024
#define HEAP_SYSLOG             printf

#define HEAP_MEM_IS_TRACED(size)    (size > HEAP_MEM_DBG_MIN_SIZE)

#define HEAP_MEM_LOG(flags, fmt, arg...)    \
    do {                                    \
        if (flags)                          \
            HEAP_SYSLOG(fmt, ##arg);        \
    } while (0)

#define HEAP_MEM_DBG(fmt, arg...) \
	HEAP_MEM_LOG(HEAP_MEM_DBG_ON, "[heap] "fmt, ##arg)

#define HEAP_MEM_ERR(fmt, arg...) \
	HEAP_MEM_LOG(HEAP_MEM_ERR_ON, "[heap ERR] %s():%d, "fmt, \
	                              __func__, __LINE__, ##arg);

#ifdef HEAP_BACKTRACE
#define BACKTRACE_COUNT   4
#define BACKTRACE_OFFSET  2
#endif

struct heap_mem {
	void *ptr;
	size_t size;
#ifdef HEAP_BACKTRACE
	void *backtrace[BACKTRACE_COUNT];
#endif
};

static struct heap_mem g_mem[HEAP_MEM_MAX_CNT];

static int g_mem_empty_idx = 0; /* beginning idx to do the search for new one */

static heap_info_t g_mem_info;

#ifdef SRAM_HEAP_TRACE
static heap_info_t g_sram_info;
#endif

#ifdef PSRAM_HEAP_TRACE
static heap_info_t g_psram_info;
#endif

#if (HEAP_MAGIC_LEN)
static const char g_mem_magic[HEAP_MAGIC_LEN] = {0x4a, 0x5b, 0x6c, 0x7f};
#define MEM_SET_MAGIC(p, l)  memcpy((((char *)(p)) + (l)), g_mem_magic, HEAP_MAGIC_LEN)
#define MEM_CHK_MAGIC(p, l)  memcmp((((char *)(p)) + (l)), g_mem_magic, HEAP_MAGIC_LEN)
#else
#define MEM_SET_MAGIC(p, l) do { } while (0)
#define MEM_CHK_MAGIC(p, l) 0
#endif

#ifdef SRAM_HEAP_TRACE
extern uint8_t __heap_start__[];
extern uint8_t __heap_end__[];
static __always_inline int is_rangeof_sramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__heap_start__, (uint32_t)__heap_end__);
}
#endif

#ifdef PSRAM_HEAP_TRACE
static __always_inline int is_rangeof_psramheap(void *ptr)
{
	return RANGEOF((uint32_t)ptr, (uint32_t)__psram_end__, PSRAM_END_ADDR);
}
#endif

static void heap_mem_show(int verbose, struct heap_mem *heap)
{
#ifdef HEAP_BACKTRACE
	int i;
#endif
	if (verbose) {
		HEAP_SYSLOG("%p, %u\n", heap->ptr, heap->size);
#ifdef HEAP_BACKTRACE
		HEAP_SYSLOG("backtrace:");
		for (i = 0; i < BACKTRACE_COUNT; i++) {
			HEAP_SYSLOG("%p ", heap->backtrace[i]);
		}
		HEAP_SYSLOG("\n");
#endif
	}

	if (MEM_CHK_MAGIC(heap->ptr, heap->size)) {
		HEAP_MEM_ERR("mem (%p) corrupt\n", heap->ptr);
#ifdef HEAP_BACKTRACE
		HEAP_SYSLOG("backtrace:");
		for (i = 0; i < BACKTRACE_COUNT; i++) {
			HEAP_SYSLOG("%p ", heap->backtrace[i]);
		}
		HEAP_SYSLOG("\n");
#endif
	}
}

#ifdef SRAM_HEAP_TRACE
void sram_heap_trace_info_show(int verbose)
{
	int i;

	malloc_mutex_lock();
	HEAP_SYSLOG("<<< sram heap info >>>\n"
	            "g_sram_sum        %u (%u KB)\n"
	            "g_sram_sum_max    %u (%u KB)\n"
	            "g_sram_entry_cnt  %u, max %u\n",
	            g_sram_info.sum, g_sram_info.sum / 1024,
	            g_sram_info.sum_max, g_sram_info.sum_max / 1024,
	            g_sram_info.entry_cnt, g_sram_info.entry_cnt_max);

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if ((g_mem[i].ptr != NULL) && (is_rangeof_sramheap(g_mem[i].ptr))) {
			heap_mem_show(verbose, &g_mem[i]);
		}
	}
	malloc_mutex_unlock();
}

void sram_heap_trace_info_get(heap_info_t *info)
{
	malloc_mutex_lock();
	if (info) {
		memcpy(info, &g_sram_info, sizeof(heap_info_t));
	}
	malloc_mutex_unlock();
}

#endif

#ifdef PSRAM_HEAP_TRACE
void psram_heap_trace_info_show(int verbose)
{
	int i;

	malloc_mutex_lock();
	HEAP_SYSLOG("<<< psram heap info >>>\n"
	            "g_psram_sum       %u (%u KB)\n"
	            "g_psram_sum_max   %u (%u KB)\n"
	            "g_psram_entry_cnt %u, max %u\n",
	            g_psram_info.sum, g_psram_info.sum / 1024,
	            g_psram_info.sum_max, g_psram_info.sum_max / 1024,
	            g_psram_info.entry_cnt, g_psram_info.entry_cnt_max);

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if ((g_mem[i].ptr != NULL) && (is_rangeof_psramheap(g_mem[i].ptr))) {
			heap_mem_show(verbose, &g_mem[i]);
		}
	}
	malloc_mutex_unlock();
}

void psram_heap_trace_info_get(heap_info_t *info)
{
	malloc_mutex_lock();
	if (info) {
		memcpy(info, &g_psram_info, sizeof(heap_info_t));
	}
	malloc_mutex_unlock();
}

#endif

void heap_trace_info_show(int verbose)
{
	malloc_mutex_lock();

	HEAP_SYSLOG("<<< total heap info >>>\n"
	            "g_mem_sum         %u (%u KB)\n"
	            "g_mem_sum_max     %u (%u KB)\n"
	            "g_mem_entry_cnt   %u, max %u\n",
	            g_mem_info.sum, g_mem_info.sum / 1024,
	            g_mem_info.sum_max, g_mem_info.sum_max / 1024,
	            g_mem_info.entry_cnt, g_mem_info.entry_cnt_max);

#ifdef SRAM_HEAP_TRACE
	sram_heap_trace_info_show(verbose);
#endif

#ifdef PSRAM_HEAP_TRACE
	psram_heap_trace_info_show(verbose);
#endif

	malloc_mutex_unlock();
}

void heap_trace_info_get(heap_info_t *info)
{
	malloc_mutex_lock();
	if (info) {
		memcpy(info, &g_mem_info, sizeof(heap_info_t));
	}
	malloc_mutex_unlock();
}

/* Note: @ptr != NULL */
static void heap_trace_add_entry(void *ptr, size_t size)
{
	int i;

	MEM_SET_MAGIC(ptr, size);

	for (i = g_mem_empty_idx; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == NULL) {
			g_mem[i].ptr = ptr;
			g_mem[i].size = size;
#ifdef HEAP_BACKTRACE
			void *trace[BACKTRACE_COUNT + BACKTRACE_OFFSET];
			backtrace_get(trace, BACKTRACE_COUNT + BACKTRACE_OFFSET);
			memcpy(g_mem[i].backtrace, &trace[BACKTRACE_OFFSET], sizeof(void *) * BACKTRACE_COUNT);
#endif
			g_mem_info.entry_cnt++;
			g_mem_empty_idx = i + 1;
			g_mem_info.sum += size;
			if (g_mem_info.sum > g_mem_info.sum_max)
				g_mem_info.sum_max = g_mem_info.sum;
			if (g_mem_info.entry_cnt > g_mem_info.entry_cnt_max)
				g_mem_info.entry_cnt_max = g_mem_info.entry_cnt;
#ifdef SRAM_HEAP_TRACE
			if (is_rangeof_sramheap(ptr)) {
				g_sram_info.entry_cnt++;
				g_sram_info.sum += size;
				if (g_sram_info.sum > g_sram_info.sum_max)
					g_sram_info.sum_max = g_sram_info.sum;
				if (g_sram_info.entry_cnt > g_sram_info.entry_cnt_max)
					g_sram_info.entry_cnt_max = g_sram_info.entry_cnt;
			}
#endif
#ifdef PSRAM_HEAP_TRACE
			if (is_rangeof_psramheap(ptr)) {
				g_psram_info.entry_cnt++;
				g_psram_info.sum += size;
				if (g_psram_info.sum > g_psram_info.sum_max)
					g_psram_info.sum_max = g_psram_info.sum;
				if (g_psram_info.entry_cnt > g_psram_info.entry_cnt_max)
					g_psram_info.entry_cnt_max = g_psram_info.entry_cnt;
			}
#endif
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem count exceed %d\n", HEAP_MEM_MAX_CNT);
	}
}

/* Note: @ptr != NULL */
static size_t heap_trace_delete_entry(void *ptr)
{
	int i;
	size_t size;

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == ptr) {
			size = g_mem[i].size;
			if (MEM_CHK_MAGIC(ptr, size)) {
				HEAP_MEM_ERR("mem f (%p, %u) corrupt\n", ptr, size);
#ifdef HEAP_BACKTRACE
				int j;
				HEAP_SYSLOG("backtrace:");
				for (j = 0; j < BACKTRACE_COUNT; j++) {
					HEAP_SYSLOG("%p ", g_mem[i].backtrace[j]);
				}
				HEAP_SYSLOG("\n");
#endif
			}
			g_mem_info.sum -= size;
			g_mem[i].ptr = NULL;
			g_mem[i].size = 0;
			g_mem_info.entry_cnt--;
			if (i < g_mem_empty_idx)
				g_mem_empty_idx = i;
#ifdef SRAM_HEAP_TRACE
			if (is_rangeof_sramheap(ptr)) {
				g_sram_info.sum -= size;
				g_sram_info.entry_cnt--;
			}
#endif
#ifdef PSRAM_HEAP_TRACE
			if (is_rangeof_psramheap(ptr)) {
				g_psram_info.sum -= size;
				g_psram_info.entry_cnt--;
			}
#endif
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem entry (%p) missed\n", ptr);
#ifdef HEAP_BACKTRACE
		backtrace_show();
#endif
		size = -1;
	}

	return size;
}

/* Note: @old_ptr != NULL, @new_ptr != NULL, @new_size != 0 */
static size_t heap_trace_update_entry(void *old_ptr, void *new_ptr, size_t new_size)
{
	int i;
	size_t old_size;

	MEM_SET_MAGIC(new_ptr, new_size);

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == old_ptr) {
			old_size = g_mem[i].size;
			g_mem_info.sum = g_mem_info.sum - old_size + new_size;
			g_mem[i].ptr = new_ptr;
			g_mem[i].size = new_size;
#ifdef HEAP_BACKTRACE
			void *trace[BACKTRACE_COUNT + BACKTRACE_OFFSET];
			backtrace_get(trace, BACKTRACE_COUNT + BACKTRACE_OFFSET);
			memcpy(g_mem[i].backtrace, &trace[BACKTRACE_OFFSET], sizeof(void *) * BACKTRACE_COUNT);
#endif
			if (g_mem_info.sum > g_mem_info.sum_max)
				g_mem_info.sum_max = g_mem_info.sum;
#ifdef SRAM_HEAP_TRACE
			if (is_rangeof_sramheap(old_ptr)) {
				g_sram_info.sum -= old_size;
			}
			if (is_rangeof_sramheap(new_ptr)) {
				g_sram_info.sum += new_size;
			}
			if (g_sram_info.sum > g_sram_info.sum_max)
				g_sram_info.sum_max = g_sram_info.sum;
#endif
#ifdef PSRAM_HEAP_TRACE
			if (is_rangeof_psramheap(old_ptr)) {
				g_psram_info.sum -= old_size;
			}
			if (is_rangeof_psramheap(new_ptr)) {
				g_psram_info.sum += new_size;
			}
			if (g_psram_info.sum > g_psram_info.sum_max)
				g_psram_info.sum_max = g_psram_info.sum;
#endif
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem entry (%p) missed\n", new_ptr);
#ifdef HEAP_BACKTRACE
		backtrace_show();
#endif
		old_size = -1;
	}

	return old_size;
}

int heap_trace_malloc(void *ptr, size_t size)
{
	if (HEAP_MEM_IS_TRACED(size)) {
		HEAP_MEM_DBG("m (%p, %u)\n", ptr, size);
	}
	if (ptr) {
		heap_trace_add_entry(ptr, size);
	} else {
		HEAP_MEM_ERR("heap mem exhausted (%u)\n", size);
	}
	return 0;
}

int heap_trace_free(void *ptr)
{
	size_t size;

	if (ptr == NULL) {
		return 0;
	}

	size = heap_trace_delete_entry(ptr);
	if (HEAP_MEM_IS_TRACED(size)) {
		HEAP_MEM_DBG("f (%p, %u)\n", ptr, size);
	}
	return 0;
}

int heap_trace_realloc(void *old_ptr, void *new_ptr, size_t new_size)
{
	size_t old_size = 0;

	if (new_size == 0) {
		if (old_ptr) {
			old_size = heap_trace_delete_entry(old_ptr);
		}
	} else {
		if (old_ptr == NULL) {
			if (new_ptr != NULL) {
				heap_trace_add_entry(new_ptr, new_size);
			} else {
				HEAP_MEM_ERR("heap mem exhausted (%p, %u)\n", old_ptr, new_size);
			}
		} else {
			if (new_ptr != NULL) {
				old_size = heap_trace_update_entry(old_ptr, new_ptr, new_size);
			} else {
				HEAP_MEM_ERR("heap mem exhausted (%p, %u)\n", old_ptr, new_size);
			}
		}
	}
	if (HEAP_MEM_IS_TRACED(new_size) || HEAP_MEM_IS_TRACED(old_size)) {
		HEAP_MEM_DBG("r (%p, %u) -> (%p, %u)\n", old_ptr, old_size, new_ptr, new_size);
	}
	return 0;
}

#endif

