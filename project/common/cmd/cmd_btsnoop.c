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
#include "util/save_log_by_uart.h"
#include "util/btsnoop.h"

/* $btsnoop start_up*/
enum cmd_status cmd_btsnoop_start_up_exec(char *cmd)
{
	int ret = -1;
	ret = btsnoop_start_up();
	if (ret != 0) {
		CMD_ERR("btsnoop_start_up failed: %d\n", ret);
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

/* $btsnoop shut_down */
enum cmd_status cmd_btsnoop_shut_down_exec(char *cmd)
{
	int ret = -1;

	if (get_uart_save_log_type() == UART_SAVE_BT_LOG) {
		ret = btsnoop_shut_down();
	} else if (get_uart_save_log_type() == UART_SAVE_WLAN_LOG) {
		CMD_ERR("uart_save_log_type is: UART_SAVE_WLAN_LOG ! \n");
		return CMD_STATUS_FAIL;
	} else {
		ret = 0;
	}

	if (ret != 0) {
		CMD_ERR("btsnoop_shut_down failed: %d\n", ret);
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

/*
	$btsnoop start_up
	$btsnoop shut_down
*/
static const struct cmd_data g_btsnoop_cmds[] = {
	{ "start_up",      cmd_btsnoop_start_up_exec },
	{ "shut_down",     cmd_btsnoop_shut_down_exec },
};

enum cmd_status cmd_btsnoop_exec(char *cmd)
{
	return cmd_exec(cmd, g_btsnoop_cmds, cmd_nitems(g_btsnoop_cmds));
}