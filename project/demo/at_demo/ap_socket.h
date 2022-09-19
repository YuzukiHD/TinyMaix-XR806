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

#ifndef __AP_SOCKET_H
#define __AP_SOCKET_H

#include "stdio.h"

#define AP_SOCKET_DBG_ON                1
#define AP_SOCKET_WRN_ON                1
#define AP_SOCKET_ERR_ON                1

#define AP_SOCKET_SYSLOG                printf

#define AP_SOCKET_LOG(flags, fmt, arg...)    \
	do {                                     \
		if (flags)                           \
			AP_SOCKET_SYSLOG(fmt, ##arg);    \
	} while (0)

#define AP_SOCKET_DBG(fmt, arg...)    \
	AP_SOCKET_LOG(AP_SOCKET_DBG_ON, "[AP_SOCKET DBG] "fmt, ##arg)

#define AP_SOCKET_WRN(fmt, arg...)    \
	AP_SOCKET_LOG(AP_SOCKET_WRN_ON, "[AP_SOCKET WRN] "fmt, ##arg)

#define AP_SOCKET_ERR(fmt, arg...)                              \
	do {                                                        \
		AP_SOCKET_LOG(AP_SOCKET_ERR_ON, "[AP_SOCKET ERR] %s():%d, "fmt,    \
		       __func__, __LINE__, ##arg);                      \
	} while (0)

extern void ap_socket_task(int port);
extern void set_maxconn(int maxconn);
extern void ap_task_delete(void);
extern void close_server(void);

#endif
