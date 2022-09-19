#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>

#ifdef CONFIG_ROM

#include "rom_version.h"
#include "rom_api.h"
#include "rom/compiler.h"


extern unsigned char __rom_start_bss[];
extern unsigned char __rom_end_bss[];
extern unsigned char __rom_start_data[];
extern unsigned char __rom_end_data[];
extern unsigned char __rom_start_text[];
extern unsigned char __rom_end_text[];

static const char _buildtime[] = __DATE__" "__TIME__;

void *__real_memcpy(void *dst0, const void *src0, size_t len);
void *__real_memset(void *s, int c, size_t n);
void *__real_memmove(void *dst0, const void *src0, size_t len);

unsigned int rom_get_version(char str[8])
{
	if (str)
		memcpy(str, ROM_VERSION_STR, sizeof(ROM_VERSION_STR));

	return ROM_VERSION_NUM;
}

void rom_get_build_time(char str[64])
{
	if (str)
		memcpy(str, _buildtime, sizeof(_buildtime));
}

static __always_inline void rom_bss_init(void)
{
	memset(__rom_start_bss, 0, (unsigned int)__rom_end_bss - (unsigned int)__rom_start_bss);
}

static __always_inline void rom_data_init(void)
{
	memcpy(__rom_start_data, __rom_end_text,
	         (unsigned int)__rom_end_data - (unsigned int)__rom_start_data);
}

void rom_init(void)
{
	rom_data_init();
	rom_bss_init();
}

#define ROM_LESS_LIBC

__attribute__ ((__optimize__ ("-O0"))) void export_rom_symbol()
{
	#define ROM_API_EXPORT(Func) pfunc = (unsigned int *)(Func)
	__attribute__((unused)) volatile unsigned int* pfunc;
	/*libc_nano*/
	ROM_API_EXPORT(atoi);
//	ROM_API_EXPORT(getenv);
//	ROM_API_EXPORT(getopt);
//	ROM_API_EXPORT(gmtime);
//	ROM_API_EXPORT(gmtime_r);
//	ROM_API_EXPORT(localtime);
	ROM_API_EXPORT(malloc);
	ROM_API_EXPORT(realloc);
	ROM_API_EXPORT(free);
	ROM_API_EXPORT(calloc);
	ROM_API_EXPORT(memchr);
	ROM_API_EXPORT(memcmp);
	//ROM_API_EXPORT(memcpy);  /* use wrap_memcpy-armv7m.S version */
	//ROM_API_EXPORT(memmove);  /* use wrap_memmove.c version */
	ROM_API_EXPORT(__real_memset);
	ROM_API_EXPORT(__real_memcpy);
	ROM_API_EXPORT(__real_memmove);
//	ROM_API_EXPORT(mktime);
	//ROM_API_EXPORT(memset);  /* use wrap_memset.c version */
	ROM_API_EXPORT(srand);
	ROM_API_EXPORT(rand);
	ROM_API_EXPORT(nanf);
	ROM_API_EXPORT(snprintf);
	ROM_API_EXPORT(sprintf);
	ROM_API_EXPORT(sscanf);
	ROM_API_EXPORT(strcasecmp);
	ROM_API_EXPORT(strcat);
	ROM_API_EXPORT(strchr);
	ROM_API_EXPORT(strcmp);
	ROM_API_EXPORT(strcpy);
	ROM_API_EXPORT(strcspn);
	ROM_API_EXPORT(strlcpy);
	ROM_API_EXPORT(strerror);
//	ROM_API_EXPORT(strftime);
	ROM_API_EXPORT(strlen);
	ROM_API_EXPORT(strncasecmp);
	ROM_API_EXPORT(strncmp);
	ROM_API_EXPORT(strncpy);
	ROM_API_EXPORT(strrchr);
	ROM_API_EXPORT(strspn);
	ROM_API_EXPORT(strstr);
	ROM_API_EXPORT(strtod);
	ROM_API_EXPORT(strtok);
	ROM_API_EXPORT(strtol);
	ROM_API_EXPORT(strtoll);
	ROM_API_EXPORT(strtoul);
	ROM_API_EXPORT(strdup);
	ROM_API_EXPORT(vsnprintf);
//	ROM_API_EXPORT(getenv);
	ROM_API_EXPORT(labs);
	ROM_API_EXPORT(abs);
	ROM_API_EXPORT(div);
//	ROM_API_EXPORT(environ);
	ROM_API_EXPORT(atol);
	/* for wlan */
	//ROM_API_EXPORT(__aeabi_ldivmod);
	ROM_API_EXPORT(qsort);
#ifndef ROM_LESS_LIBC
	ROM_API_EXPORT(nanf);
	/*libM*/
	int temp_int = (int)__rom_start_data[200];
	double temp_double = temp_int * (-0.66666);
	temp_double = temp_int + (temp_double);
	temp_double = temp_int - (temp_double);
	temp_double = temp_int * (temp_double);
	temp_double = temp_int / (temp_double);
	long long temp_long = 1000 + (-222);
	temp_long = temp_int + (temp_long);
	temp_long = temp_int - (temp_long);
	temp_long = temp_int * (temp_long);
	temp_long = temp_int / (temp_long);
	temp_long = temp_int % (temp_long);
	ROM_API_EXPORT(cos);
	ROM_API_EXPORT(sin);
	ROM_API_EXPORT(floor);
	ROM_API_EXPORT(logl);
	ROM_API_EXPORT(sqrt);
	ROM_API_EXPORT(pow);
	ROM_API_EXPORT(fabs);
	ROM_API_EXPORT(swapfunc);
#else
	//ROM_API_EXPORT(swapfunc);
#endif
}

#endif
