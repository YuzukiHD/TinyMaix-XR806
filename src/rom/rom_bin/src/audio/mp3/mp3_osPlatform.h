#ifndef __ROM_MP3_OSPLATFORM_H
#define __ROM_MP3_OSPLATFORM_H
#ifdef CONFIG_ROM
#define __OS_LINUX 1
#define ENABLE_MPEG_LAYER_I_II 1
#define ARM_ADS 1

/*eldk_type.h*/
#ifdef __OS_LINUX
#include <stdint.h>
typedef uint64_t    __u64;
typedef uint32_t    __u32;
typedef uint16_t      __u16;
typedef uint8_t       __u8;
typedef int64_t       __s64;
typedef int32_t          __s32;
typedef int16_t        __s16;
typedef int8_t         __s8;
typedef __s8         __bool;

typedef __u64    u64;
typedef __u32    u32;
typedef __u16    u16;
typedef __u8     u8;
typedef __s64    s64;
typedef __s32    s32;
typedef __s16    s16;
typedef __s8     s8;
//typedef __bool   bool;
#undef  bool
#define bool __bool
typedef s64 __int64;
#endif

/*osplatform.h*/
#ifdef __OS_LINUX

#ifndef 	NULL
#define 	NULL 0
#endif
typedef __s64  int64_t;
typedef __u64 uint64_t;

#ifdef DEFINEFLOAT
typedef long long  __int64;
typedef float FLOAT;
#else
typedef float FLOAT;
typedef __s64  __int64;
#endif
#endif
#endif
#endif
