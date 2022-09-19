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

#if PRJCONF_NET_EN

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmd_util.h"
#include "kernel/os/os_time.h"
#include "net/libwebsockets/libwebsockets.h"

typedef enum {
	LWS_CMD_CREATE,
	LWS_CMD_CONNECT,
	LWS_CMD_SEND,
	LWS_CMD_DISCONNECT,
	LWS_CMD_EXIT
} LWS_CMD;

struct lws_msg {
	LWS_CMD type;
	void *data;
	int data_len;
};

struct cmd_lws_common {
	struct lws_context *context;
	struct lws *conn;
	int ssl;
	int port;
	char host[128];
	char address[128];
	char path[128];
	OS_Thread_t thread;
	OS_Queue_t queue;
};

static struct cmd_lws_common cmd_lws;

static int lws_callback(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len);

static struct lws_protocols protocols[] = {
	{
		"dumb-increment-protocol,fake-nonexistant-protocol",
		lws_callback,
		0,
		1024 * 2
	},
	{
		NULL, NULL, 0, 0 /* end */
	}
};

static int lws_callback(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len)
{
	switch (reason) {
	case LWS_CALLBACK_CLIENT_ESTABLISHED:
		CMD_LOG(1, "------ connection established success ------\n");
		lws_callback_on_writable(wsi);
		break;
	case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
		CMD_LOG(1, "------ connection error ------\n");
		break;
	case LWS_CALLBACK_CLOSED:
		CMD_LOG(1, "------ connection closed ------\n");
		break;
	case LWS_CALLBACK_CLIENT_RECEIVE:
		CMD_LOG(1, "------ receive msg from server: %s ------\n", (char *)in);
		break;
	case LWS_CALLBACK_CLIENT_WRITEABLE:
		CMD_LOG(1, "------ connection writeable ------\n");
		break;
	default:
		break;
	}
	return 0;
}

static int lws_cmd_create_handler(void *data, int data_len)
{
	struct lws_context_creation_info *info;

	info = (struct lws_context_creation_info *)data;
	cmd_lws.context = lws_create_context(info);
	if (cmd_lws.context == NULL) {
		CMD_ERR("libwebsocket context create fail\n");
		return -1;
	}
	CMD_DBG("libwebsocket context create success\n");

	return 0;
}

static int lws_cmd_connect_handler(void *data, int data_len)
{
	struct lws_client_connect_info *connect_info;

	connect_info = (struct lws_client_connect_info *)data;
	connect_info->context = cmd_lws.context;
	cmd_lws.conn = lws_client_connect_via_info(connect_info);
	if (cmd_lws.conn == NULL) {
		CMD_ERR("libwebsocket connect fail\n");
		return -1;
	}
	CMD_DBG("libwebsocket connect success\n");

	return 0;
}

static int lws_cmd_send_handler(void *data, int data_len)
{
	unsigned char *buffer;

	buffer = cmd_malloc(data_len + LWS_PRE);
	if (buffer == NULL) {
		return -1;
	}
	cmd_memcpy(buffer + LWS_PRE, data, data_len);
	lws_write(cmd_lws.conn, (buffer + LWS_PRE), data_len, LWS_WRITE_TEXT);
	cmd_free(buffer);
	return 0;
}

static int lws_cmd_disconnect_handler(void *data, int data_len)
{
	lws_context_destroy(cmd_lws.context);
	cmd_lws.context = NULL;
	cmd_lws.conn = NULL;
	return 0;
}

static void lws_client_task(void *arg)
{
	int exit = 0;
	int connected = 0;
	OS_Status status;
	struct lws_msg *msg;

	while (!exit) {
		status = OS_MsgQueueReceive(&cmd_lws.queue, (void **)&msg, 0);
		if (status != OS_OK) {
			if (connected) {
				lws_service(cmd_lws.context, 50);
			}
			OS_MSleep(50);
			continue;
		}

		CMD_DBG("msg type:%d\n", msg->type);
		switch (msg->type) {
		case LWS_CMD_CREATE:
			lws_cmd_create_handler(msg->data, msg->data_len);
			break;
		case LWS_CMD_CONNECT:
			lws_cmd_connect_handler(msg->data, msg->data_len);
			connected = 1;
			break;
		case LWS_CMD_SEND:
			lws_cmd_send_handler(msg->data, msg->data_len);
			break;
		case LWS_CMD_DISCONNECT:
			lws_cmd_disconnect_handler(msg->data, msg->data_len);
			connected = 0;
			break;
		case LWS_CMD_EXIT:
			exit = 1;
			break;
		default:
			break;
		}
		cmd_free(msg->data);
		cmd_free(msg);
	}
	OS_ThreadDelete(&cmd_lws.thread);
}

static enum cmd_status cmd_lws_init_exec(char *cmd)
{
	if (OS_MsgQueueCreate(&cmd_lws.queue, 4) != OS_OK) {
		CMD_ERR("create queue failed\n");
		return CMD_STATUS_FAIL;
	}

	if (OS_ThreadCreate(&cmd_lws.thread,
	                    "lws client",
	                    lws_client_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    (8 * 1024)) != OS_OK) {
		CMD_ERR("create thread failed\n");
		OS_MsgQueueDelete(&cmd_lws.queue);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_lws_deinit_exec(char *cmd)
{
	OS_Status status;
	struct lws_msg *msg;

	msg = cmd_malloc(sizeof(struct lws_msg));
	if (msg == NULL) {
		return CMD_STATUS_FAIL;
	}
	cmd_memset(msg, 0, sizeof(struct lws_msg));
	msg->type = LWS_CMD_EXIT;
	msg->data_len = 0;
	msg->data = NULL;

	status = OS_MsgQueueSend(&cmd_lws.queue, msg, OS_WAIT_FOREVER);
	if (status != OS_OK) {
		cmd_free(msg);
		return CMD_STATUS_FAIL;
	}
	while (OS_ThreadIsValid(&cmd_lws.thread)) {
		OS_MSleep(10);
	}
	OS_MsgQueueDelete(&cmd_lws.queue);
	cmd_memset(&cmd_lws, 0, sizeof(struct cmd_lws_common));

	return CMD_STATUS_OK;
}

static int websocket_url_parser(char *url)
{
	char *pos;
	char *colon;

	if (url == NULL) {
		return -1;
	}
	CMD_DBG("connect url: %s\n", url);

	cmd_memset(cmd_lws.host, 0, sizeof(cmd_lws.host));
	cmd_memset(cmd_lws.address, 0, sizeof(cmd_lws.address));
	cmd_memset(cmd_lws.path, 0, sizeof(cmd_lws.path));

	pos = url;
	if (cmd_strncmp(pos, "wss://", 6) == 0) {
		cmd_lws.ssl = 1;
		pos += 6;
	} else if (cmd_strncmp(pos, "ws://", 5) == 0) {
		cmd_lws.ssl = 0;
		pos += 5;
	} else {
		return -1;
	}

	colon = cmd_strchr(pos, ':');
	if (colon) {
		int host_len = colon - pos;
		if ((host_len + 1 > sizeof(cmd_lws.host))
		    || (host_len + 1 > sizeof(cmd_lws.address))) {
			return -1;
		}

		cmd_memcpy(cmd_lws.host, pos, host_len);
		cmd_lws.host[host_len] = '\0';

		cmd_memcpy(cmd_lws.address, pos, host_len);
		cmd_lws.address[host_len] = '\0';
		pos += (host_len + 1);
	} else {
		return -1;
	}

	cmd_lws.port = cmd_atoi(pos);
	if (cmd_lws.port == 0) {
		return -1;
	}

	colon = cmd_strchr(pos, '/');
	if (colon) {
		cmd_strlcpy(cmd_lws.path, colon, sizeof(cmd_lws.path));
	} else {
		cmd_strlcpy(cmd_lws.path, "/", sizeof(cmd_lws.path));
	}
	return 0;
}

static int cmd_lws_create(void)
{
	OS_Status status;
	struct lws_msg *msg;
	struct lws_context_creation_info *info;

	info = cmd_malloc(sizeof(struct lws_context_creation_info));
	if (info == NULL) {
		return -1;
	}
	cmd_memset(info, 0, sizeof(struct lws_context_creation_info));
	info->port = CONTEXT_PORT_NO_LISTEN;
	info->protocols = protocols;
	info->gid = -1;
	info->uid = -1;
	if (cmd_lws.ssl) {
		info->options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
	}

	msg = cmd_malloc(sizeof(struct lws_msg));
	if (msg == NULL) {
		cmd_free(info);
		return -1;
	}
	cmd_memset(msg, 0, sizeof(struct lws_msg));
	msg->type = LWS_CMD_CREATE;
	msg->data = info;
	msg->data_len = sizeof(struct lws_context_creation_info);

	status = OS_MsgQueueSend(&cmd_lws.queue, msg, OS_WAIT_FOREVER);
	if (status != OS_OK) {
		cmd_free(info);
		cmd_free(msg);
		return -1;
	}
	return 0;
}

static int cmd_lws_connect(void)
{
	OS_Status status;
	struct lws_msg *msg;
	struct lws_client_connect_info *connect_info;

	connect_info = cmd_malloc(sizeof(struct lws_client_connect_info));
	if (connect_info == NULL) {
		return -1;
	}
	cmd_memset(connect_info, 0, sizeof(struct lws_client_connect_info));
	connect_info->ssl_connection = cmd_lws.ssl;
	connect_info->host = cmd_lws.host;
	connect_info->address = cmd_lws.address;
	connect_info->port = cmd_lws.port;
	connect_info->path = cmd_lws.path;
	connect_info->origin = NULL;
	connect_info->protocol = protocols[0].name;

	msg = cmd_malloc(sizeof(struct lws_msg));
	if (msg == NULL) {
		cmd_free(connect_info);
		return -1;
	}
	cmd_memset(msg, 0, sizeof(struct lws_msg));
	msg->type = LWS_CMD_CONNECT;
	msg->data = connect_info;
	msg->data_len = sizeof(struct lws_client_connect_info);

	status = OS_MsgQueueSend(&cmd_lws.queue, msg, OS_WAIT_FOREVER);
	if (status != OS_OK) {
		cmd_free(connect_info);
		cmd_free(msg);
		return -1;
	}
	return 0;
}

static enum cmd_status cmd_lws_connect_exec(char *cmd)
{
	int ret;

	ret = websocket_url_parser(cmd);
	if (ret != 0) {
		CMD_ERR("websocket url parser fail.\n");
		return CMD_STATUS_FAIL;
	}

	ret = cmd_lws_create();
	if (ret != 0) {
		return CMD_STATUS_FAIL;
	}

	ret = cmd_lws_connect();
	if (ret != 0) {
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_lws_send_exec(char *cmd)
{
	void *data;
	int data_len;
	OS_Status status;
	struct lws_msg *msg;

	if (cmd == NULL) {
		return CMD_STATUS_INVALID_ARG;
	}

	data_len = cmd_strlen(cmd) + 1;
	data = cmd_malloc(data_len);
	if (data == NULL) {
		return CMD_STATUS_FAIL;
	}
	cmd_memcpy(data, cmd, data_len);

	msg = cmd_malloc(sizeof(struct lws_msg));
	if (msg == NULL) {
		cmd_free(data);
		return CMD_STATUS_FAIL;
	}
	cmd_memset(msg, 0, sizeof(struct lws_msg));
	msg->type = LWS_CMD_SEND;
	msg->data_len = data_len;
	msg->data = data;

	status = OS_MsgQueueSend(&cmd_lws.queue, msg, OS_WAIT_FOREVER);
	if (status != OS_OK) {
		cmd_free(data);
		cmd_free(msg);
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_lws_disconnect_exec(char *cmd)
{
	OS_Status status;
	struct lws_msg *msg;

	msg = cmd_malloc(sizeof(struct lws_msg));
	if (msg == NULL) {
		return CMD_STATUS_FAIL;
	}
	cmd_memset(msg, 0, sizeof(struct lws_msg));
	msg->type = LWS_CMD_DISCONNECT;
	msg->data_len = 0;
	msg->data = NULL;

	status = OS_MsgQueueSend(&cmd_lws.queue, msg, OS_WAIT_FOREVER);
	if (status != OS_OK) {
		cmd_free(msg);
		return CMD_STATUS_FAIL;
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_lws_help_exec(char *cmd);

static const struct cmd_data g_lws_cmds[] = {
	{ "init",       cmd_lws_init_exec,       CMD_DESC("init. format:net lws init") },
	{ "deinit",     cmd_lws_deinit_exec,     CMD_DESC("deinit. format:net lws deinit") },
	{ "connect",    cmd_lws_connect_exec,    CMD_DESC("connect server. format: lws connect <url>") },
	{ "send",       cmd_lws_send_exec,       CMD_DESC("send message to server. format: lws send <TEXT>") },
	{ "disconnect", cmd_lws_disconnect_exec, CMD_DESC("disconnect. format: lws disconnect") },
	{ "help",       cmd_lws_help_exec,       CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_lws_help_exec(char *cmd)
{
	return cmd_help_exec(g_lws_cmds, cmd_nitems(g_lws_cmds), 8);
}

enum cmd_status cmd_lws_exec(char *cmd)
{
	return cmd_exec(cmd, g_lws_cmds, cmd_nitems(g_lws_cmds));
}

#endif /* PRJCONF_NET_EN */
