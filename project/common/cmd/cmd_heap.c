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

#include "cmd_util.h"
#include "debug/heap_trace.h"
#include "sys/sys_heap.h"

extern void heap_get_space(uint8_t **start, uint8_t **end, uint8_t **current);
extern void psram_heap_get_space(uint8_t **start, uint8_t **end, uint8_t **current);

enum cmd_status cmd_heap_space_exec(char *cmd)
{
	uint8_t *start, *end, *current;

	heap_get_space(&start, &end, &current);
	cmd_write_respond(CMD_STATUS_OK,
	         "sram heap total %u, max use %u, min free %u, [%p, %p, %p)",
	         end - start, current - start, end - current, start, current, end);
#ifdef CONFIG_PSRAM
	psram_heap_get_space(&start, &end, &current);
	cmd_write_respond(CMD_STATUS_OK,
	         "psram heap total %u, max use %u, min free %u, [%p, %p, %p)",
	         end - start, current - start, end - current, start, current, end);
#endif
	return CMD_STATUS_ACKED;
}

#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_PSRAM_MALLOC_TRACE)

enum cmd_status cmd_heap_info_exec(char *cmd)
{
#ifdef CONFIG_MALLOC_TRACE
	uint8_t *start, *end, *current;
	heap_info_t sram_info;
#endif
#ifdef CONFIG_PSRAM_MALLOC_TRACE
	size_t psram_heap_size;
	heap_info_t psram_info;
#endif

#ifdef CONFIG_MALLOC_TRACE
	heap_get_space(&start, &end, &current);
	sram_heap_trace_info_get(&sram_info);
	cmd_write_respond(CMD_STATUS_OK, "sram total %u (%u KB), "
	                  "use %u (%u KB), free %u (%u KB), "
	                  "max use %u (%u KB), min free %u (%u KB)",
	                  end - start, (end - start) / 1024,
	                  sram_info.sum, sram_info.sum / 1024,
	                  end - start - sram_info.sum,
	                  (end - start - sram_info.sum) / 1024,
	                  sram_info.sum_max, sram_info.sum_max / 1024,
	                  end - start - sram_info.sum_max,
	                  (end - start - sram_info.sum_max) / 1024);
#endif
#ifdef CONFIG_PSRAM_MALLOC_TRACE
	psram_heap_size = psram_total_heap_size();
	psram_heap_trace_info_get(&psram_info);
	cmd_write_respond(CMD_STATUS_OK, "psram total %u (%u KB), "
	                  "use %u (%u KB), free %u (%u KB), "
	                  "max use %u (%u KB), min free %u (%u KB)",
	                  psram_heap_size, psram_heap_size / 1024,
	                  psram_info.sum, psram_info.sum / 1024,
	                  psram_heap_size - psram_info.sum,
	                  (psram_heap_size - psram_info.sum) / 1024,
	                  psram_info.sum_max, psram_info.sum_max / 1024,
	                  psram_heap_size - psram_info.sum_max,
	                  (psram_heap_size - psram_info.sum_max) / 1024);
#endif

	heap_trace_info_show(cmd_atoi(cmd));

	return CMD_STATUS_ACKED;
}

#endif

static const struct cmd_data g_heap_cmds[] = {
	{ "space",    cmd_heap_space_exec },
#if defined(CONFIG_MALLOC_TRACE) || defined(CONFIG_PSRAM_MALLOC_TRACE)
	{ "info",     cmd_heap_info_exec },
#endif
};

enum cmd_status cmd_heap_exec(char *cmd)
{
	return cmd_exec(cmd, g_heap_cmds, cmd_nitems(g_heap_cmds));
}
