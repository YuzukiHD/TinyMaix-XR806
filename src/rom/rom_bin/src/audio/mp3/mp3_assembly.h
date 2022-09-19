#ifndef _ROM_MP3_ASSEMBLY_H
#define _ROM_MP3_ASSEMBLY_H
#ifdef CONFIG_ROM
#include "mp3_osPlatform.h"

#ifdef __OS_LINUX
typedef __int64 Word64;
static inline __attribute__((always_inline)) Word64 MADD64(Word64 sum, int a1, int a2)
{
	union {
		uint64_t z;
		unsigned hl[2];
	} z = { sum };
	__asm__ ("smlal %0, %1, %2, %3"
	         : "+r"(z.hl[0]), "+r"(z.hl[1]) : "r"(a1), "r"(a2));
	return z.z;

	//return (sum + ((__int64)x * y));
	/*unsigned int sumLo = ((unsigned int *)&sum)[0];
	int sumHi = ((int *)&sum)[1];

	__asm {
		mov		eax, x
		imul	y
		add		eax, sumLo
		adc		edx, sumHi
	}*/

	/* equivalent to return (sum + ((__int64)x * y)); */
}

static __inline Word64 SHL64(Word64 a1, int n)
{
	return (a1<<n);
}

static __inline Word64 SAR64(Word64 a1, int n)
{
	return (a1>>n);
}
static __inline__ int AW_MULSHIFT32(int a1, int a2)
{
	int vlow;
	__asm__ volatile ("smull %0,%1,%2,%3" : "=&r" (vlow), "=r" (a2) : "r" (a1), "1" (a2));
	return a2;
}

static __inline int AW_FASTABS(int a1)
{
	int s;

	s = a1 >> (sizeof(int) * 8 - 1);
	a1 ^= s;
	a1 -= s;

	return a1;
}
#endif
#endif
#endif
