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

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd.h"

/*
 * main commands
 */
static enum cmd_status cmd_main_help_exec(char *cmd);

static const struct cmd_data g_main_cmds[] = {
	{ "mem",     cmd_mem_exec, CMD_DESC("memory command") },
	{ "heap",    cmd_heap_exec, CMD_DESC("heap use information command") },
	{ "thread",  cmd_thread_exec, CMD_DESC("thread information command") },
	{ "upgrade", cmd_upgrade_exec, CMD_DESC("upgrade command") },
	{ "reboot",  cmd_reboot_exec, CMD_DESC("reboot command") },
#ifdef CONFIG_FILESYSTEMS
	{ "fs",      cmd_fs_exec },
#endif
	{ "help",    cmd_main_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_main_help_exec(char *cmd)
{
	return cmd_help_exec(g_main_cmds, cmd_nitems(g_main_cmds), 8);
}

void main_cmd_exec(char *cmd)
{
	cmd_main_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
}
