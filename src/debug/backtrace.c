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

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define BACKTRACE_LIMIT        0x2000
#define BACKTRACE_DEPTH        32

extern uint8_t __text_start__[];
extern uint8_t __text_end__[];
#if (defined(CONFIG_XIP))
extern uint8_t __xip_start__[];
extern uint8_t __xip_end__[];
#endif
#if (defined(CONFIG_PSRAM))
extern uint8_t __psram_text_start__[];
extern uint8_t __psram_text_end__[];
#endif

static inline int pc_check(char *pc)
{
	if (((uint32_t)pc > (uint32_t)__text_start__) && ((uint32_t)pc < (uint32_t)__text_end__)) {
		return 0;
	}
#if (defined(CONFIG_XIP))
	if (((uint32_t)pc > (uint32_t)__xip_start__) && ((uint32_t)pc < (uint32_t)__xip_end__)) {
		return 0;
	}
#endif
#if (defined(CONFIG_PSRAM))
	if (((uint32_t)pc > (uint32_t)__psram_text_start__) && ((uint32_t)pc < (uint32_t)__psram_text_end__)) {
		return 0;
	}
#endif
	return -1;
}

static int backtrace_by_stack(int **pSP, char **pPC)
{
	char          *CodeAddr = NULL;
	int           *SP       = *pSP;
	char          *PC       = *pPC;
	char          *LR;
	char          *BL;
	int            i;
	unsigned short ins16;
	unsigned int   ins32;
	unsigned int   framesize = 0;
	unsigned int   shift = 0;
	unsigned int   sub = 0;
	unsigned int   offset = 1;

	if (pc_check(*pPC) != 0) {
		//printf("backtrace : invalid pc : %p\n", *pPC);
		return -1;
	}

	/* 1. scan code, find frame size from "push" or "stmdb sp!" */
	for (i = 2; i < BACKTRACE_LIMIT; i += 2) {
		/* find nearest "push   {..., lr}" */
		ins16 = *(unsigned short *)(PC - i);
		if ((ins16 & 0xff00) == 0xb500) {
			framesize = __builtin_popcount((unsigned char)ins16);
			framesize++;
			/* find double push */
			ins16 = *(unsigned short *)(PC - i - 2);
			if ((ins16 & 0xff00) == 0xb400) {
				offset += __builtin_popcount((unsigned char)ins16);
				framesize += __builtin_popcount((unsigned char)ins16);
			}
			CodeAddr = PC - i;
			break;
		}

		/* find "stmdb sp!, ..." */
		ins32 = *(unsigned short *)(PC - i);
		ins32 <<= 16;
		ins32 |= *(unsigned short *)(PC - i + 2);
		if ((ins32 & 0xFFFFF000) == 0xe92d4000) {
			framesize = __builtin_popcount(ins32 & 0xfff);
			framesize++;
			CodeAddr = PC - i;
			break;
		}
	}

	if (CodeAddr == NULL) {
		return -1;
	}

	/* 2. scan code, find frame size from "sub" or "sub.w" */
	for (i = 0; i < BACKTRACE_LIMIT;) {
		if (CodeAddr + i > PC) {
			break;
		}
		/* find "sub    sp, ..." */
		ins16 = *(unsigned short *)(CodeAddr + i);
		if ((ins16 & 0xff80) == 0xb080) {
			framesize += (ins16 & 0x7f);
			break;
		}

		/* find "sub.w	sp, sp, ..." */
		ins32 = *(unsigned short *)(CodeAddr + i);
		ins32 <<= 16;
		ins32 |= *(unsigned short *)(CodeAddr + i + 2);
		if ((ins32 & 0xFBFF8F00) == 0xF1AD0D00) {
			sub = 128 + (ins32 & 0x7f);
			shift  = (ins32 >> 7) & 0x1;
			shift += ((ins32 >> 12) & 0x7) << 1;
			shift += ((ins32 >> 26) & 0x1) << 4;
			framesize += sub<<(30 - shift);
			break;
		}

		if ((ins16 & 0xf800) >= 0xe800) {
			i += 4;
		} else {
			i += 2;
		}
	}

	/* 3. output */
	LR = (char *)*(SP + framesize - offset);
	BL = ((char *)(((uintptr_t)(LR)) & 0xfffffffe));
	if (((int)BL & 0xffffffe0) == 0xffffffe0) {
		offset = 0;
	} else {
		ins16 = *(unsigned short *)(BL - 4);
		if ((ins16 & 0xf000) == 0xf000) {
			offset = 5;
		} else {
			offset = 3;
		}
	}
	*pSP   = SP + framesize;
	*pPC   = LR - offset;

	if (pc_check(*pPC) != 0) {
		//printf("backtrace : invalid pc : %p\n", *pPC);
		return -1;
	}

	return offset == 0 ? 1 : 0;
}

int backtrace_get(void *backtrace[], int len)
{
	char *PC;
	int  *SP;
	int   i;
	int   ret;

	__asm__ volatile("mov %0, sp\n" : "=r"(SP));
	__asm__ volatile("mov %0, pc\n" : "=r"(PC));

	memset(backtrace, 0, len * sizeof(void *));

	for (i = 0; i < len; i++) {
		ret = backtrace_by_stack(&SP, &PC);
		if (ret != 0) {
			break;
		}
		backtrace[i] = PC;
	}
	return 0;
}

int backtrace_show(void)
{
	char *PC;
	int  *SP;
	int   i;
	int   ret;

	__asm__ volatile("mov %0, sp\n" : "=r"(SP));
	__asm__ volatile("mov %0, pc\n" : "=r"(PC));

	printf("========== back trace ==========\n");
	for (i = 0; i < BACKTRACE_DEPTH; i++) {
		ret = backtrace_by_stack(&SP, &PC);
		if (ret != 0) {
			break;
		}
		printf("backtrace:%p\n", PC);
	}
	printf("==========    End     ==========\n");
	return 0;
}

