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

#ifndef _TZ_THREAD_H_
#define _TZ_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* convert macro argument to string */
/* note: this needs one level of indirection, accomplished with the helper macro
 *       __MACRO_TO_STRING */
#define __MACRO_TO_STRING(x) #x
#define MACRO_TO_STRING(x)   __MACRO_TO_STRING(x)

/* select an overloaded macro, so that 0 to 4 arguments can be used */
#define MACRO_SELECT(_0, _1, _2, _3, _4, NAME, ...) NAME

/* count macro arguments */
#define MACRO_NARGS(...) \
    MACRO_SELECT(_0, ##__VA_ARGS__, 4, 3, 2, 1, 0)

/* declare explicit callee-saved registers to hold input arguments (0 to 4) */
/* note: sizeof(type) must be less than or equal to 4 */
#define MACRO_REGS_ARGS(type, ...)                      \
    MACRO_SELECT(_0, ##__VA_ARGS__, MACRO_REGS_ARGS4,   \
                 MACRO_REGS_ARGS3,                      \
                 MACRO_REGS_ARGS2,                      \
                 MACRO_REGS_ARGS1,                      \
                 MACRO_REGS_ARGS0)(type, ##__VA_ARGS__)

#define MACRO_REGS_ARGS0(type)
#define MACRO_REGS_ARGS1(type, a0)             \
    register type r0 asm("r0") = (type) (a0);
#define MACRO_REGS_ARGS2(type, a0, a1)         \
    register type r0 asm("r0") = (type) (a0);  \
    register type r1 asm("r1") = (type) (a1);
#define MACRO_REGS_ARGS3(type, a0, a1, a2)     \
    register type r0 asm("r0") = (type) (a0);  \
    register type r1 asm("r1") = (type) (a1);  \
    register type r2 asm("r2") = (type) (a2);
#define MACRO_REGS_ARGS4(type, a0, a1, a2, a3) \
    register type r0 asm("r0") = (type) (a0);  \
    register type r1 asm("r1") = (type) (a1);  \
    register type r2 asm("r2") = (type) (a2);  \
    register type r3 asm("r3") = (type) (a3);

/* declare explicit callee-saved registers to hold output values */
/* note: currently only one output value is allowed, up to 32bits */
#define MACRO_REGS_RETVAL(type, name) \
    register type name asm("r0");

#define MACRO_GCC_ASM_INPUT(...)                          \
    MACRO_SELECT(_0, ##__VA_ARGS__, MACRO_GCC_ASM_INPUT4, \
                 MACRO_GCC_ASM_INPUT3,                    \
                 MACRO_GCC_ASM_INPUT2,                    \
                 MACRO_GCC_ASM_INPUT1,                    \
                 MACRO_GCC_ASM_INPUT0)(__VA_ARGS__)

#define MACRO_GCC_ASM_INPUT0()               [__dummy] "I" (0)
#define MACRO_GCC_ASM_INPUT1(a0)             [r0] "r" (r0)
#define MACRO_GCC_ASM_INPUT2(a0, a1)         [r0] "r" (r0), [r1] "r" (r1)
#define MACRO_GCC_ASM_INPUT3(a0, a1, a2)     [r0] "r" (r0), [r1] "r" (r1), [r2] "r" (r2)
#define MACRO_GCC_ASM_INPUT4(a0, a1, a2, a3) [r0] "r" (r0), [r1] "r" (r1), [r2] "r" (r2), [r3] "r" (r3)

#define MACRO_GCC_ASM_OUTPUT(name) [res] "=r" (name)

static __always_inline uint32_t TEE_ContextIsValid(void)
{
	uint32_t secure_context = vPortGetSecureContext();

	return (secure_context != 0);
}

static __always_inline uint32_t TEE_Malloc(uint32_t SecureStackSize)
{
	if (TEE_ContextIsValid()) {
		return 1;
	}
	portALLOCATE_SECURE_CONTEXT(SecureStackSize);
	return 0;
}

static __always_inline void TEE_Free(void)
{
	vPortFreeSecureContext_alone();
}

/*
 * You must not define un-volatile variables or register(r0~r3) variables here,
 *    because it maybe as r0~r3 in fact, thus will be modified when nsc-function return.
 * Please don't change it unless you are absolutely sure.
 */
#define TEE_FUNC_CALL(dst_func, ...)                            \
	({                                                          \
		MACRO_REGS_RETVAL(int32_t, res);                        \
		volatile int32_t _ret = ~0, malloc_valid = 0;           \
		if (!TEE_Malloc(configMAXIMAL_SECURE_STACK_SIZE)) {     \
			malloc_valid = 1;                                   \
		}                                                       \
		MACRO_REGS_ARGS(uint32_t, ##__VA_ARGS__);               \
		__asm volatile(                                         \
		               "bl " MACRO_TO_STRING(dst_func) "\n"     \
		               : MACRO_GCC_ASM_OUTPUT(res)              \
		               : MACRO_GCC_ASM_INPUT(__VA_ARGS__)       \
		               );                                       \
		__asm volatile("mov %0, r0" : "=r"(_ret) : : "memory"); \
		if (malloc_valid == 1) {                                \
			TEE_Free();                                         \
		}                                                       \
		res = _ret;                                             \
	})

#ifdef __cplusplus
}
#endif

#endif /* _TZ_THREAD_H_ */
