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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "blec.h"
#include "errno.h"
#include "kernel/os/os.h"

enum {
	VIRTUAL_HCI_ENABLED     =    1 << 0,
};

static uint32_t driver_flags;

int bt_ctrl_driver_init(uint8_t *pub_addr)
{
	int ret = 0;

	if (driver_flags & VIRTUAL_HCI_ENABLED)
		return -EALREADY;

	ret = blec_init();
	if (ret) {
		printf("blec failed(%d)\n", ret);
		return -ENODEV;
	}

	blec_set_public_address(pub_addr);

	driver_flags |= VIRTUAL_HCI_ENABLED;

	return 0;
}

int bt_ctrl_driver_deinit(void)
{
	driver_flags &= ~VIRTUAL_HCI_ENABLED;

	blec_hci_deinit();
	blec_deinit();

	return 0;
}

bool bt_ctrl_driver_is_ready(void)
{
	return !!(driver_flags | VIRTUAL_HCI_ENABLED);
}

