/*
 * OS specific functions
 * Copyright (c) 2005-2009, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef OS_H
#define OS_H

#include <string.h>

#ifndef os_memcpy
#define os_memcpy(d, s, n) memcpy((d), (s), (n))
#endif
#ifndef os_memmove
#define os_memmove(d, s, n) memmove((d), (s), (n))
#endif
#ifndef os_memset
#define os_memset(s, c, n) memset(s, c, n)
#endif
#ifndef os_memcmp
#define os_memcmp(s1, s2, n) memcmp((s1), (s2), (n))
#endif

#ifndef os_strlen
#define os_strlen(s) strlen(s)
#endif
#ifndef os_strcasecmp
#ifdef _MSC_VER
#define os_strcasecmp(s1, s2) _stricmp((s1), (s2))
#else
#define os_strcasecmp(s1, s2) strcasecmp((s1), (s2))
#endif
#endif
#ifndef os_strncasecmp
#ifdef _MSC_VER
#define os_strncasecmp(s1, s2, n) _strnicmp((s1), (s2), (n))
#else
#define os_strncasecmp(s1, s2, n) strncasecmp((s1), (s2), (n))
#endif
#endif
#ifndef os_strchr
#define os_strchr(s, c) strchr((s), (c))
#endif
#ifndef os_strcmp
#define os_strcmp(s1, s2) strcmp((s1), (s2))
#endif
#ifndef os_strncmp
#define os_strncmp(s1, s2, n) strncmp((s1), (s2), (n))
#endif
#ifndef os_strrchr
#define os_strrchr(s, c) strrchr((s), (c))
#endif
#ifndef os_strstr
#define os_strstr(h, n) strstr((h), (n))
#endif

#define TEST_FAIL() 0

#endif /* OS_H */
