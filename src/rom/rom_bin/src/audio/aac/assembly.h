#ifndef _ROM_ASSEMBLY_H
#define _ROM_ASSEMBLY_H
#ifdef CONFIG_ROM
/* toolchain:           MSFT Visual C++
 * target architecture: x86
 */
#if (defined (_WIN32) && !defined (_WIN32_WCE)) //|| (defined (__WINS__) && defined (_SYMBIAN)) || (defined (WINCE_EMULATOR)) || (defined (_OPENWAVE_SIMULATOR))

#pragma warning( disable : 4035 )	/* complains about inline asm not returning a value */

static __inline int MULSHIFT32(int a, int b)
{
	__asm {
		mov		eax, a
		imul	b
		mov		eax, edx
	}
}

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return 32;

	/* count leading zeros with binary search */
	nZeros = 1;
	if (!((unsigned int)a >> 16))	{
		nZeros += 16;
		a <<= 16;
	}
	if (!((unsigned int)a >> 24))	{
		nZeros +=  8;
		a <<=  8;
	}
	if (!((unsigned int)a >> 28))	{
		nZeros +=  4;
		a <<=  4;
	}
	if (!((unsigned int)a >> 30))	{
		nZeros +=  2;
		a <<=  2;
	}

	nZeros -= ((unsigned int)a >> 31);

	return nZeros;
}

typedef __int64 word_64;

typedef union _AW_U64 {
	word_64 w64;
	struct {
		/* x86 = little endian */
		unsigned int u_lo32;
		signed int   s_hi32;
	} r;
} U64;

/* returns 64-bit value in [edx:eax] */
static __inline word_64 MADD64(word_64 sum64, int a, int b)
{
//	U64 u;
//	u.w64 = sum64;

	sum64 += (word_64)a * (word_64)b;
	return sum64;

	/* asm version
	 *
	 *	__asm mov	eax, x
	 *	__asm imul	y
	 *	__asm add   eax, u.r.u_lo32
	 *	__asm adc   edx, u.r.s_hi32
	 */
}

/* toolchain:           MSFT Embedded Visual C++
 * target architecture: ARM v.4 and above (require 'M' type processor for 32x32->64 multiplier)
 */
#elif defined (_WIN32) && defined (_WIN32_WCE) && defined (ARM)

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return 32;

	/* count leading zeros with binary search (function should be 17 ARM instructions total) */
	nZeros = 1;
	if (!((unsigned int)a >> 16))	{
		nZeros += 16;
		a <<= 16;
	}
	if (!((unsigned int)a >> 24))	{
		nZeros +=  8;
		a <<=  8;
	}
	if (!((unsigned int)a >> 28))	{
		nZeros +=  4;
		a <<=  4;
	}
	if (!((unsigned int)a >> 30))	{
		nZeros +=  2;
		a <<=  2;
	}

	nZeros -= ((unsigned int)a >> 31);

	return nZeros;
}

/* implemented in asmfunc.s */
#ifdef __cplusplus
extern "C" {
#endif

	typedef __int64 word_64;

	typedef union _AW_U64 {
		word_64 w64;
		struct {
			/* ARM WinCE = little endian */
			unsigned int u_lo32;
			signed int   s_hi32;
		} r;
	} U64;

	/* manual name mangling for just this platform (must match labels in .s file) */
//#define MULSHIFT32	raac_MULSHIFT32
//#define MADD64		raac_MADD64

	int MULSHIFT32(int x, int y);
	word_64 MADD64(word_64 sum64, int x, int y);

#ifdef __cplusplus
}
#endif

/* toolchain:           ARM ADS or RealView
 * target architecture: ARM v.4 and above (requires 'M' type processor for 32x32->64 multiplier)
 */
#elif defined (__arm) && defined (__ARMCC_VERSION)

static __inline int MULSHIFT32(int a, int b)
{
	/* rules for smull RdLo, RdHi, Rm, Rs:
	 *   RdHi != Rm
	 *   RdLo != Rm
	 *   RdHi != RdLo
	 */
	int temp;//zlow;
	__asm {
		smull temp,b,a,b
	}

	return b;
}

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return 32;

	/* count leading zeros with binary search (function should be 17 ARM instructions total) */
	nZeros = 1;
	if (!((unsigned int)a >> 16))	{
		nZeros += 16;
		a <<= 16;
	}
	if (!((unsigned int)a >> 24))	{
		nZeros +=  8;
		a <<=  8;
	}
	if (!((unsigned int)a >> 28))	{
		nZeros +=  4;
		a <<=  4;
	}
	if (!((unsigned int)a >> 30))	{
		nZeros +=  2;
		a <<=  2;
	}

	nZeros -= ((unsigned int)a >> 31);

	return nZeros;

	/* ARM code would look like this, but do NOT use inline asm in ADS for this,
	   because you can't safely use the status register flags intermixed with C code

		__asm {
		    mov		nZeros, #1
			tst		x, 0xffff0000
			addeq	nZeros, nZeros, #16
			moveq	x, x, lsl #16
			tst		x, 0xff000000
			addeq	nZeros, nZeros, #8
			moveq	x, x, lsl #8
			tst		x, 0xf0000000
			addeq	nZeros, nZeros, #4
			moveq	x, x, lsl #4
			tst		x, 0xc0000000
			addeq	nZeros, nZeros, #2
			moveq	x, x, lsl #2
			sub		nZeros, nZeros, x, lsr #31
		}
	*/
	/* reference:
		nZeros = 0;
		while (!(x & 0x80000000)) {
			nZeros++;
			x <<= 1;
		}
	*/
}

typedef __int64 word_64;

typedef union _AW_U64 {
	word_64 w64;
	struct {
		/* ARM ADS = little endian */
		unsigned int u_lo32;
		signed int   s_hi32;
	} r;
} U64;

static __inline word_64 MADD64(word_64 sum64, int a, int b)
{
	U64 u;
	u.w64 = sum64;

	__asm {
		smlal u.r.u_lo32, u.r.s_hi32, a, b
	}

	return u.w64;
}

/* toolchain:           ARM gcc
 * target architecture: ARM v.4 and above (requires 'M' type processor for 32x32->64 multiplier)
 */
#elif defined(__GNUC__) && defined(__arm__)

static __inline__ int MULSHIFT32(int a, int b)
{
	int zlow;
	__asm__ volatile ("smull %0,%1,%2,%3" : "=&r" (zlow), "=r" (b) : "r" (a), "1" (b) : "cc");
	return b;
}

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return (sizeof(int) * 8);

	nZeros = 0;
	while (!(a & 0x80000000)) {
		nZeros++;
		a <<= 1;
	}

	return nZeros;
}

typedef long long word_64;

typedef union _AW_U64 {
	word_64 w64;
	struct {
		/* ARM ADS = little endian */
		unsigned int u_lo32;
		signed int   s_hi32;
	} r;
} U64;

static __inline word_64 MADD64(word_64 sum64, int a, int b)
{
	U64 u;
	u.w64 = sum64;

	__asm__ volatile ("smlal %0,%1,%2,%3" : "+&r" (u.r.u_lo32), "+&r" (u.r.s_hi32) : "r" (a), "r" (b) : "cc");

	return u.w64;
}

/* toolchain:           x86 gcc
 * target architecture: x86
 */
#elif defined(__GNUC__) && defined(__i386__)

typedef long long word_64;

static __inline__ int MULSHIFT32(int a, int b)
{
	int z;

	z = (word_64)a * (word_64)b >> 32;

	return z;
}

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return 32;

	/* count leading zeros with binary search (function should be 17 ARM instructions total) */
	nZeros = 1;
	if (!((unsigned int)a >> 16))	{
		nZeros += 16;
		a <<= 16;
	}
	if (!((unsigned int)a >> 24))	{
		nZeros +=  8;
		a <<=  8;
	}
	if (!((unsigned int)a >> 28))	{
		nZeros +=  4;
		a <<=  4;
	}
	if (!((unsigned int)a >> 30))	{
		nZeros +=  2;
		a <<=  2;
	}

	nZeros -= ((unsigned int)a >> 31);

	return nZeros;
}

typedef union _AW_U64 {
	word_64 w64;
	struct {
		/* x86 = little endian */
		unsigned int u_lo32;
		signed int   s_hi32;
	} r;
} U64;

static __inline word_64 MADD64(word_64 sum64, int a, int b)
{
	sum64 += (word_64)a * (word_64)b;

	return sum64;
}

#elif defined(__GNUC__) && defined(__LP64__)

typedef long long word_64;
typedef union _AW_U64 {
	word_64 w64;
	struct {
		/* x86 = little endian */
		unsigned int u_lo32;
		signed int   s_hi32;
	} r;
} U64;

static __inline int MULSHIFT32(int a, int b)
{
	int z;

	z = (word_64)a * (word_64)b >> 32;

	return z;
}

static __inline short CLIPTOSHORT(int a)
{
	int sign;

	/* clip to [-32768, 32767] */
	sign = a >> 31;
	if (sign != (a >> 15))
		a = sign ^ ((1 << 15) - 1);

	return (short)a;
}

static __inline int FASTABS(int a)
{
	int sign;

	sign = a >> (sizeof(int) * 8 - 1);
	a ^= sign;
	a -= sign;

	return a;
}

static __inline int CLZ(int a)
{
	int nZeros;

	if (!a)
		return (sizeof(int) * 8);

	nZeros = 0;
	while (!(a & 0x80000000)) {
		nZeros++;
		a <<= 1;
	}

	return nZeros;
}

static __inline word_64 MADD64(word_64 sum64, int a, int b)
{
	sum64 += (word_64)a * (word_64)b;

	return sum64;
}

#else

#error Unsupported platform in assembly.h

#endif	/* platforms */
#endif /*CONFIG_ROM*/
#endif /* _ROM_AAC_ASSEMBLY_H */
