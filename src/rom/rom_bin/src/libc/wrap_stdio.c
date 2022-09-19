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

#include "rom/libc/stdarg.h"
#include "rom/libc/stdio.h"

int __wrap_printf(const char *format, ...)
{
	int len;
	va_list ap;

	va_start(ap, format);
	len = vprintf(format, ap);
	va_end(ap);

	return len;
}

int __wrap_vprintf(const char *format, va_list ap)
{
#define _ram_tbl___wrap_vprintf \
	RAM_TBL_FUN(int (*)(const char *format, va_list ap), __wrap_vprintf)

	return _ram_tbl___wrap_vprintf(format, ap);
}

int __wrap_puts(const char *s)
{
#define _ram_tbl___wrap_puts \
	RAM_TBL_FUN(int (*)(const char *s), __wrap_puts)

	return _ram_tbl___wrap_puts(s);
}

int __wrap_fprintf(FILE *stream, const char *format, ...)
{
	int len;
	va_list ap;

	va_start(ap, format);
	len = vfprintf(stream, format, ap);
	va_end(ap);

	return len;
}

int __wrap_vfprintf(FILE *stream, const char *format, va_list ap)
{
#define _ram_tbl___wrap_vfprintf \
	RAM_TBL_FUN(int (*)(FILE *stream, const char *format, va_list ap), __wrap_vfprintf)

	return _ram_tbl___wrap_vfprintf(stream, format, ap);
}

int __wrap_fputs(const char *s, FILE *stream)
{
#define _ram_tbl___wrap_fputs \
	RAM_TBL_FUN(int (*)(const char *s, FILE *stream), __wrap_fputs)

	return _ram_tbl___wrap_fputs(s, stream);
}

int __wrap_putchar(int c)
{
#define _ram_tbl___wrap_putchar \
	RAM_TBL_FUN(int (*)(int c), __wrap_putchar)

	return _ram_tbl___wrap_putchar(c);
}

int __wrap_putc(int c, FILE *stream)
{
#define _ram_tbl___wrap_putc \
	RAM_TBL_FUN(int (*)(int c, FILE *stream), __wrap_putc)

	return _ram_tbl___wrap_putc(c, stream);
}

int __wrap_fputc(int c, FILE *stream)
{
#define _ram_tbl___wrap_fputc \
	RAM_TBL_FUN(int (*)(int c, FILE *stream), __wrap_fputc)

	return _ram_tbl___wrap_fputc(c, stream);
}

int __wrap_fflush(FILE *stream)
{
#define _ram_tbl___wrap_fflush \
	RAM_TBL_FUN(int (*)(FILE *stream), __wrap_fflush)

	return _ram_tbl___wrap_fflush(stream);
}

#endif /* CONFIG_ROM */
