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
#include <string.h>
#include <stdlib.h>

#include "sys/interrupt.h"
#include "sys/coexist.h"

uint32_t coex_state_flag = 0;   /* bit[0:15] wlan, bit[16:31] bt */

void xr_coex_set_bt_state(enum xr_coex_state_t state)
{
	unsigned long flags = arch_irq_save();
	coex_state_flag &= (uint32_t)~(0xFFFF << 16);
	coex_state_flag |= (state << 16);
	arch_irq_restore(flags);
}

void xr_coex_set_wlan_state(enum xr_coex_state_t state)
{
	unsigned long flags = arch_irq_save();
	coex_state_flag &= (uint32_t)~(0xFFFF << 0);
	coex_state_flag |= (state << 0);
	arch_irq_restore(flags);
}

unsigned short xr_coex_get_bt_state(void)
{
	uint16_t state;

	state = (uint16_t)((coex_state_flag & (0xFFFF << 16)) >> 16);

	return state;
}

unsigned short xr_coex_get_wlan_state(void)
{
	uint16_t state;

	state = (uint16_t)((coex_state_flag & (0xFFFF << 0)) >> 0);

	return state;
}
