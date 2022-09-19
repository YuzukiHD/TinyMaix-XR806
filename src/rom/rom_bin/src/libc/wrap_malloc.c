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

#ifdef CONFIG_ROM

#include "rom/ram_table.h"

#include "rom/libc/sys/reent.h"
#include "rom/libc/stddef.h"

void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
#define _ram_tbl___wrap__malloc_r \
	RAM_TBL_FUN(void *(*)(struct _reent *reent, size_t size), __wrap__malloc_r)

	return _ram_tbl___wrap__malloc_r(reent, size);
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
#define _ram_tbl___wrap__realloc_r \
	RAM_TBL_FUN(void *(*)(struct _reent *reent, void *ptr, size_t size), __wrap__realloc_r)

	return _ram_tbl___wrap__realloc_r(reent, ptr, size);
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
#define _ram_tbl___wrap__free_r \
	RAM_TBL_FUN(void (*)(struct _reent *reent, void *ptr), __wrap__free_r)

	_ram_tbl___wrap__free_r(reent, ptr);
}

void *_sbrk(int incr)
{
#define _ram_tbl__sbrk \
	RAM_TBL_FUN(void *(*)(int incr), _sbrk)

	return _ram_tbl__sbrk(incr);
}

#endif /* CONFIG_ROM */
