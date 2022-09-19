/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef __ROM_SECURE_CONTEXT_H__
#define __ROM_SECURE_CONTEXT_H__

#include "secure_context.h"

#include "rom/ram_table.h"

#define SecureContext_FreeContext \
	RAM_TBL_FUN(void (*)( SecureContextHandle_t xSCHandle ), SecureContext_FreeContext)

#define SecureContext_LoadContext \
	RAM_TBL_FUN(void (*)( SecureContextHandle_t xSecureContextHandle ), SecureContext_LoadContext)

#define SecureContext_SaveContext \
	RAM_TBL_FUN(void (*)( SecureContextHandle_t xSecureContextHandle ), SecureContext_SaveContext)

#define SecureContext_Init \
	RAM_TBL_FUN(void (*)( void ), SecureContext_Init)

#if( configENABLE_MPU == 1 )
#define SecureContext_AllocateContext \
	RAM_TBL_FUN(SecureContextHandle_t (*)( uint32_t ulSecureStackSize, uint32_t ulIsTaskPrivileged ), SecureContext_AllocateContext)
#else /* configENABLE_MPU */
#define SecureContext_AllocateContext \
	RAM_TBL_FUN(SecureContextHandle_t (*)( uint32_t ulSecureStackSize ), SecureContext_AllocateContext)
#endif /* configENABLE_MPU */

#endif /* __ROM_SECURE_CONTEXT_H__ */
