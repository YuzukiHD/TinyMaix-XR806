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
#include "cmd_bench_mark.h"

#ifdef CONFIG_BENCH_MARK
/*
 * benchmark coremark
 */
static enum cmd_status cmd_coremark_exec(char *cmd)
{
	extern void coremark_main(void);

	coremark_main();

	return CMD_STATUS_OK;
}

/*
 * benchmark dhrystonre 1000000
 */
static enum cmd_status cmd_dhrystonre_exec(char *cmd)
{
	extern void dhrystonre_main (int Number_Of_Runs);
	uint32_t num;
	int32_t cnt;

	cnt = cmd_sscanf(cmd, "%u", &num);
	if (cnt != 1) {
		return CMD_STATUS_INVALID_ARG;
	}
	dhrystonre_main(num);
	return CMD_STATUS_OK;
}

/*
 * benchmark whetstone
 */
static enum cmd_status cmd_whetstone_exec(char *cmd)
{
	extern int whetstone_main(void);

	whetstone_main();
	return CMD_STATUS_OK;
}

static const struct cmd_data g_benchmark_cmds[] = {
	{ "coremark",   cmd_coremark_exec },
	{ "dhrystonre", cmd_dhrystonre_exec },
	{ "whetstone",  cmd_whetstone_exec },
};

enum cmd_status cmd_benchmark_exec(char *cmd)
{
	return cmd_exec(cmd, g_benchmark_cmds, cmd_nitems(g_benchmark_cmds));
}
#endif
