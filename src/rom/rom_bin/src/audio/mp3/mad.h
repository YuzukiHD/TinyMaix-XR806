#ifndef _ROM_MAD_H_
#define _ROM_MAD_H_
#ifdef CONFIG_ROM
#include "mp3_osPlatform.h"
# ifdef __cplusplus
extern "C" {
# endif

# define FPM_INTEL
# define AW_SIZEOF_INT 4
# define AW_SIZEOF_LONG 4
# define AW_SIZEOF_LONG_LONG 8

	/* Id: version.h,v 1.26 2004/01/23 09:41:33 rob Exp */
# ifndef LIBMAD_VERSION_H
# define LIBMAD_VERSION_H

# define MAD_VERSION_MAJOR	0
# define MAD_VERSION_MINOR	15
# define MAD_VERSION_PATCH	1
# define MAD_VERSION_EXTRA	" (beta)"

# define MAD_VERSION_STRINGIZE(str)	#str
# define MAD_VERSION_STRING(num)	MAD_VERSION_STRINGIZE(num)

# define MAD_VERSION		MAD_VERSION_STRING(MAD_VERSION_MAJOR) "."  \
				MAD_VERSION_STRING(MAD_VERSION_MINOR) "."  \
				MAD_VERSION_STRING(MAD_VERSION_PATCH)  \
				MAD_VERSION_EXTRA

# define MAD_PUBLISHYEAR	"2000-2004"
# define MAD_AUTHOR		"Underbit Technologies, Inc."
# define MAD_EMAIL		"info@underbit.com"
	extern char const mad_version[];
	extern char const mad_copyright[];
	extern char const mad_author[];
	extern char const mad_build[];
# endif

	/* Id: fixed.h,v 1.38 2004/02/17 02:02:03 rob Exp */
# ifndef LIBAW_MAD_FIXED_H
# define LIBAW_MAD_FIXED_H

# if AW_SIZEOF_INT >= 4
	typedef   signed int aw_mad_fixed_t;

	typedef   signed int mad_fixed64hi_t;
	typedef unsigned int mad_fixed64lo_t;
# else
	typedef   signed long aw_mad_fixed_t;

	typedef   signed long mad_fixed64hi_t;
	typedef unsigned long mad_fixed64lo_t;
# endif

# if defined(_MSC_VER)
#  define mad_fixed64_t  signed __int64
# elif 1 || defined(__GNUC__)
#  define mad_fixed64_t  signed long long
# endif

# if defined(FPM_FLOAT)
	typedef double mad_sample_t;
# else
	typedef aw_mad_fixed_t mad_sample_t;
# endif

	/*
	 * Fixed-point format: 0xABBBBBBB
	 * A == whole part      (sign + 3 bits)
	 * B == fractional part (28 bits)
	 *
	 * Values are signed two's complement, so the effective range is:
	 * 0x80000000 to 0x7fffffff
	 *       -8.0 to +7.9999999962747097015380859375
	 *
	 * The smallest representable value is:
	 * 0x00000001 == 0.0000000037252902984619140625 (i.e. about 3.725e-9)
	 *
	 * 28 bits of fractional accuracy represent about
	 * 8.6 digits of decimal accuracy.
	 *
	 * Fixed-point numbers can be added or subtracted as normal
	 * integers, but multiplication requires shifting the 64-bit result
	 * from 56 fractional bits back to 28 (and rounding.)
	 *
	 * Changing the definition of AW_MAD_F_FRACBITS is only partially
	 * supported, and must be done with care.
	 */

# define AW_MAD_F_FRACBITS		28

# if AW_MAD_F_FRACBITS == 28
#  define AW_MAD_F(x)		((aw_mad_fixed_t) (x##L))
# else
#  if AW_MAD_F_FRACBITS < 28
#   warning "AW_MAD_F_FRACBITS < 28"
#   define AW_MAD_F(x)		((aw_mad_fixed_t)  \
				 (((x##L) +  \
				   (1L << (28 - AW_MAD_F_FRACBITS - 1))) >>  \
				  (28 - AW_MAD_F_FRACBITS)))
#  elif AW_MAD_F_FRACBITS > 28
#   error "AW_MAD_F_FRACBITS > 28 not currently supported"
#   define AW_MAD_F(x)		((aw_mad_fixed_t)  \
				 ((x##L) << (AW_MAD_F_FRACBITS - 28)))
#  endif
# endif

# define AW_MAD_F_MIN		((aw_mad_fixed_t) -0x80000000L)
# define AW_MAD_F_MAX		((aw_mad_fixed_t) +0x7fffffffL)

# define AW_MAD_F_ONE		AW_MAD_F(0x10000000)

# define mad_f_tofixed(x)	((aw_mad_fixed_t)  \
				 ((x) * (double) (1L << AW_MAD_F_FRACBITS) + 0.5))
# define mad_f_todouble(x)	((double)  \
				 ((x) / (double) (1L << AW_MAD_F_FRACBITS)))

# define mad_f_intpart(x)	((x) >> AW_MAD_F_FRACBITS)
# define mad_f_fracpart(x)	((x) & ((1L << AW_MAD_F_FRACBITS) - 1))
	/* (x should be positive) */

# define mad_f_fromint(x)	((x) << AW_MAD_F_FRACBITS)

# define mad_f_add(x, y)	((x) + (y))
# define mad_f_sub(x, y)	((x) - (y))

# if defined(FPM_FLOAT)
#  error "FPM_FLOAT not yet supported"

#  undef AW_MAD_F
#  define AW_MAD_F(x)		mad_f_todouble(x)

#  define aw_mad_f_mul(x, y)	((x) * (y))
#  define mad_f_scale64

#  undef ASO_ZEROCHECK

# elif defined(FPM_64BIT)

	/*
	 * This version should be the most accurate if 64-bit types are supported by
	 * the compiler, although it may not be the most efficient.
	 */
#  if defined(OPT_ACCURACY)
#   define aw_mad_f_mul(x, y)  \
    ((aw_mad_fixed_t)  \
     ((((mad_fixed64_t) (x) * (y)) +  \
       (1L << (AW_MAD_F_SCALEBITS - 1))) >> AW_MAD_F_SCALEBITS))
#  else
#   define aw_mad_f_mul(x, y)  \
    ((aw_mad_fixed_t) (((mad_fixed64_t) (x) * (y)) >> AW_MAD_F_SCALEBITS))
#  endif

#  define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS

	/* --- Intel --------------------------------------------------------------- */

# elif defined(FPM_INTEL)

#  if defined(_MSC_VER)
#   pragma warning(push)
#   pragma warning(disable: 4035)  /* no return value */
	static __forceinline
	aw_mad_fixed_t aw_mad_f_mul_inline(aw_mad_fixed_t x, aw_mad_fixed_t y)
	{
		enum {
			fracbits = AW_MAD_F_FRACBITS
		};

		__asm {
			mov eax, x
			imul y
			shrd eax, edx, fracbits
		}

		/* implicit return of eax */
	}
#   pragma warning(pop)

#   define aw_mad_f_mul		aw_mad_f_mul_inline
#   define mad_f_scale64
#  else
	/*
	 * This Intel version is fast and accurate; the disposition of the least
	 * significant bit depends on OPT_ACCURACY via mad_f_scale64().
	 */
#   define AW_MAD_F_MLX(hi, lo, x, y)  \
    asm ("imull %3"  \
	 : "=a" (lo), "=d" (hi)  \
	 : "%a" (x), "rm" (y)  \
	 : "cc")

#   if defined(OPT_ACCURACY)
	/*
	 * This gives best accuracy but is not very fast.
	 */
#    define AW_MAD_F_MLA(hi, lo, x, y)  \
    ({ mad_fixed64hi_t __hi;  \
       mad_fixed64lo_t __lo;  \
       AW_MAD_F_MLX(__hi, __lo, (x), (y));  \
       asm ("addl %2,%0\n\t"  \
	    "adcl %3,%1"  \
	    : "=rm" (lo), "=rm" (hi)  \
	    : "r" (__lo), "r" (__hi), "0" (lo), "1" (hi)  \
	    : "cc");  \
    })
#   endif  /* OPT_ACCURACY */

#   if defined(OPT_ACCURACY)
	/*
	 * Surprisingly, this is faster than SHRD followed by ADC.
	 */
#    define mad_f_scale64(hi, lo)  \
    ({ mad_fixed64hi_t __hi_;  \
       mad_fixed64lo_t __lo_;  \
       aw_mad_fixed_t __result;  \
       asm ("addl %4,%2\n\t"  \
	    "adcl %5,%3"  \
	    : "=rm" (__lo_), "=rm" (__hi_)  \
	    : "0" (lo), "1" (hi),  \
	      "ir" (1L << (AW_MAD_F_SCALEBITS - 1)), "ir" (0)  \
	    : "cc");  \
       asm ("shrdl %3,%2,%1"  \
	    : "=rm" (__result)  \
	    : "0" (__lo_), "r" (__hi_), "I" (AW_MAD_F_SCALEBITS)  \
	    : "cc");  \
       __result;  \
    })
#   elif defined(OPT_INTEL)
	/*
	 * Alternate Intel scaling that may or may not perform better.
	 */
#    define mad_f_scale64(hi, lo)  \
    ({ aw_mad_fixed_t __result;  \
       asm ("shrl %3,%1\n\t"  \
	    "shll %4,%2\n\t"  \
	    "orl %2,%1"  \
	    : "=rm" (__result)  \
	    : "0" (lo), "r" (hi),  \
	      "I" (AW_MAD_F_SCALEBITS), "I" (32 - AW_MAD_F_SCALEBITS)  \
	    : "cc");  \
       __result;  \
    })
#   else
#    define mad_f_scale64(hi, lo)  \
    ({ aw_mad_fixed_t __result;  \
       asm ("shrdl %3,%2,%1"  \
	    : "=rm" (__result)  \
	    : "0" (lo), "r" (hi), "I" (AW_MAD_F_SCALEBITS)  \
	    : "cc");  \
       __result;  \
    })
#   endif  /* OPT_ACCURACY */

#   define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS
#  endif

	/* --- ARM ----------------------------------------------------------------- */

# elif defined(FPM_ARM)

	/*
	 * This ARM V4 version is as accurate as FPM_64BIT but much faster. The
	 * least significant bit is properly rounded at no CPU cycle cost!
	 */
# if 1
	/*
	 * This is faster than the default implementation via AW_MAD_F_MLX() and
	 * mad_f_scale64().
	 */
#  define aw_mad_f_mul(x, y)  \
    ({ mad_fixed64hi_t __hi;  \
       mad_fixed64lo_t __lo;  \
       aw_mad_fixed_t __result;  \
       asm ("smull	%0, %1, %3, %4\n\t"  \
	    "movs	%0, %0, lsr %5\n\t"  \
	    "adc	%2, %0, %1, lsl %6"  \
	    : "=&r" (__lo), "=&r" (__hi), "=r" (__result)  \
	    : "%r" (x), "r" (y),  \
	      "M" (AW_MAD_F_SCALEBITS), "M" (32 - AW_MAD_F_SCALEBITS)  \
	    : "cc");  \
       __result;  \
    })
# endif

#  define AW_MAD_F_MLX(hi, lo, x, y)  \
    asm ("smull	%0, %1, %2, %3"  \
	 : "=&r" (lo), "=&r" (hi)  \
	 : "%r" (x), "r" (y))

#  define AW_MAD_F_MLA(hi, lo, x, y)  \
    asm ("smlal	%0, %1, %2, %3"  \
	 : "+r" (lo), "+r" (hi)  \
	 : "%r" (x), "r" (y))

#  define AW_MAD_F_MLN(hi, lo)  \
    asm ("rsbs	%0, %2, #0\n\t"  \
	 "rsc	%1, %3, #0"  \
	 : "=r" (lo), "=r" (hi)  \
	 : "0" (lo), "1" (hi)  \
	 : "cc")

#  define mad_f_scale64(hi, lo)  \
    ({ aw_mad_fixed_t __result;  \
       asm ("movs	%0, %1, lsr %3\n\t"  \
	    "adc	%0, %0, %2, lsl %4"  \
	    : "=&r" (__result)  \
	    : "r" (lo), "r" (hi),  \
	      "M" (AW_MAD_F_SCALEBITS), "M" (32 - AW_MAD_F_SCALEBITS)  \
	    : "cc");  \
       __result;  \
    })

#  define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS

	/* --- MIPS ---------------------------------------------------------------- */

# elif defined(FPM_MIPS)

	/*
	 * This MIPS version is fast and accurate; the disposition of the least
	 * significant bit depends on OPT_ACCURACY via mad_f_scale64().
	 */
#  define AW_MAD_F_MLX(hi, lo, x, y)  \
    asm ("mult	%2,%3"  \
	 : "=l" (lo), "=h" (hi)  \
	 : "%r" (x), "r" (y))

# if defined(HAVE_MADD_ASM)
#  define AW_MAD_F_MLA(hi, lo, x, y)  \
    asm ("madd	%2,%3"  \
	 : "+l" (lo), "+h" (hi)  \
	 : "%r" (x), "r" (y))
# elif defined(HAVE_MADD16_ASM)
	/*
	 * This loses significant accuracy due to the 16-bit integer limit in the
	 * multiply/accumulate instruction.
	 */
#  define AW_MAD_F_ML0(hi, lo, x, y)  \
    asm ("mult	%2,%3"  \
	 : "=l" (lo), "=h" (hi)  \
	 : "%r" ((x) >> 12), "r" ((y) >> 16))
#  define AW_MAD_F_MLA(hi, lo, x, y)  \
    asm ("madd16	%2,%3"  \
	 : "+l" (lo), "+h" (hi)  \
	 : "%r" ((x) >> 12), "r" ((y) >> 16))
#  define AW_MAD_F_MLZ(hi, lo)  ((aw_mad_fixed_t) (lo))
# endif

# if defined(OPT_SPEED)
#  define mad_f_scale64(hi, lo)  \
    ((aw_mad_fixed_t) ((hi) << (32 - AW_MAD_F_SCALEBITS)))
#  define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS
# endif

	/* --- SPARC --------------------------------------------------------------- */

# elif defined(FPM_SPARC)

	/*
	 * This SPARC V8 version is fast and accurate; the disposition of the least
	 * significant bit depends on OPT_ACCURACY via mad_f_scale64().
	 */
#  define AW_MAD_F_MLX(hi, lo, x, y)  \
    asm ("smul %2, %3, %0\n\t"  \
	 "rd %%y, %1"  \
	 : "=r" (lo), "=r" (hi)  \
	 : "%r" (x), "rI" (y))

	/* --- PowerPC ------------------------------------------------------------- */

# elif defined(FPM_PPC)

	/*
	 * This PowerPC version is fast and accurate; the disposition of the least
	 * significant bit depends on OPT_ACCURACY via mad_f_scale64().
	 */
#  define AW_MAD_F_MLX(hi, lo, x, y)  \
    do {  \
      asm ("mullw %0,%1,%2"  \
	   : "=r" (lo)  \
	   : "%r" (x), "r" (y));  \
      asm ("mulhw %0,%1,%2"  \
	   : "=r" (hi)  \
	   : "%r" (x), "r" (y));  \
    }  \
    while (0)

#  if defined(OPT_ACCURACY)
	/*
	 * This gives best accuracy but is not very fast.
	 */
#   define AW_MAD_F_MLA(hi, lo, x, y)  \
    ({ mad_fixed64hi_t __hi;  \
       mad_fixed64lo_t __lo;  \
       AW_MAD_F_MLX(__hi, __lo, (x), (y));  \
       asm ("addc %0,%2,%3\n\t"  \
	    "adde %1,%4,%5"  \
	    : "=r" (lo), "=r" (hi)  \
	    : "%r" (lo), "r" (__lo),  \
	      "%r" (hi), "r" (__hi)  \
	    : "xer");  \
    })
#  endif

#  if defined(OPT_ACCURACY)
	/*
	 * This is slower than the truncating version below it.
	 */
#   define mad_f_scale64(hi, lo)  \
    ({ aw_mad_fixed_t __result, __round;  \
       asm ("rotrwi %0,%1,%2"  \
	    : "=r" (__result)  \
	    : "r" (lo), "i" (AW_MAD_F_SCALEBITS));  \
       asm ("extrwi %0,%1,1,0"  \
	    : "=r" (__round)  \
	    : "r" (__result));  \
       asm ("insrwi %0,%1,%2,0"  \
	    : "+r" (__result)  \
	    : "r" (hi), "i" (AW_MAD_F_SCALEBITS));  \
       asm ("add %0,%1,%2"  \
	    : "=r" (__result)  \
	    : "%r" (__result), "r" (__round));  \
       __result;  \
    })
#  else
#   define mad_f_scale64(hi, lo)  \
    ({ aw_mad_fixed_t __result;  \
       asm ("rotrwi %0,%1,%2"  \
	    : "=r" (__result)  \
	    : "r" (lo), "i" (AW_MAD_F_SCALEBITS));  \
       asm ("insrwi %0,%1,%2,0"  \
	    : "+r" (__result)  \
	    : "r" (hi), "i" (AW_MAD_F_SCALEBITS));  \
       __result;  \
    })
#  endif

#  define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS

	/* --- Default ------------------------------------------------------------- */

# elif defined(AW_FPM_DEFAULT)

	/*
	 * This version is the most portable but it loses significant accuracy.
	 * Furthermore, accuracy is biased against the second argument, so care
	 * should be taken when ordering operands.
	 *
	 * The scale factors are constant as this is not used with SSO.
	 *
	 * Pre-rounding is required to stay within the limits of compliance.
	 */
#  if defined(OPT_SPEED)
#   define aw_mad_f_mul(x, y)	(((x) >> 12) * ((y) >> 16))
#  else
#   define aw_mad_f_mul(x, y)	((((x) + (1L << 11)) >> 12) *  \
				 (((y) + (1L << 15)) >> 16))
#  endif

	/* ------------------------------------------------------------------------- */

# else
#  error "no FPM selected"
# endif

	/* default implementations */

# if !defined(aw_mad_f_mul)
#  define aw_mad_f_mul(x, y)  \
    ({ register mad_fixed64hi_t __hi;  \
       register mad_fixed64lo_t __lo;  \
       AW_MAD_F_MLX(__hi, __lo, (x), (y));  \
       mad_f_scale64(__hi, __lo);  \
    })
# endif

# if !defined(AW_MAD_F_MLA)
#  define AW_MAD_F_ML0(hi, lo, x, y)	((lo)  = aw_mad_f_mul((x), (y)))
#  define AW_MAD_F_MLA(hi, lo, x, y)	((lo) += aw_mad_f_mul((x), (y)))
#  define AW_MAD_F_MLN(hi, lo)		((lo)  = -(lo))
#  define AW_MAD_F_MLZ(hi, lo)		((void) (hi), (aw_mad_fixed_t) (lo))
# endif

# if !defined(AW_MAD_F_ML0)
#  define AW_MAD_F_ML0(hi, lo, x, y)	AW_MAD_F_MLX((hi), (lo), (x), (y))
# endif

# if !defined(AW_MAD_F_MLN)
#  define AW_MAD_F_MLN(hi, lo)		((hi) = ((lo) = -(lo)) ? ~(hi) : -(hi))
# endif

# if !defined(AW_MAD_F_MLZ)
#  define AW_MAD_F_MLZ(hi, lo)		mad_f_scale64((hi), (lo))
# endif

# if !defined(mad_f_scale64)
#  if defined(OPT_ACCURACY)
#   define mad_f_scale64(hi, lo)  \
    ((((aw_mad_fixed_t)  \
       (((hi) << (32 - (AW_MAD_F_SCALEBITS - 1))) |  \
	((lo) >> (AW_MAD_F_SCALEBITS - 1)))) + 1) >> 1)
#  else
#   define mad_f_scale64(hi, lo)  \
    ((aw_mad_fixed_t)  \
     (((hi) << (32 - AW_MAD_F_SCALEBITS)) |  \
      ((lo) >> AW_MAD_F_SCALEBITS)))
#  endif
#  define AW_MAD_F_SCALEBITS  AW_MAD_F_FRACBITS
# endif

	/* C routines */

	aw_mad_fixed_t mad_f_abs(aw_mad_fixed_t);
	aw_mad_fixed_t mad_f_div(aw_mad_fixed_t, aw_mad_fixed_t);

# endif

	/* Id: bit.h,v 1.12 2004/01/23 09:41:32 rob Exp */

# ifndef LIBMAD_BIT_H
# define LIBMAD_BIT_H

	typedef struct mad_bitptr {
		unsigned char const *byte;
		unsigned short cache;
		unsigned short left;
	} aw_mad_bitptr;

	void aw_mad_bit_init(aw_mad_bitptr *, unsigned char const *);


	unsigned int aw_mad_bit_length(aw_mad_bitptr const *,
	                               aw_mad_bitptr const *);

//# define mad_bit_bitsleft(bitptr)  ((bitptr)->left)
	unsigned char const *aw_mad_bit_nextbyte(aw_mad_bitptr const *);

//void mad_bit_skip(aw_mad_bitptr *, unsigned int);
	unsigned long aw_mad_bit_read(aw_mad_bitptr *, unsigned int);

	unsigned short aw_mad_bit_crc(aw_mad_bitptr, unsigned int, unsigned short);

# endif

	/* Id: timer.h,v 1.16 2004/01/23 09:41:33 rob Exp */

# ifndef LIBMAD_TIMER_H
# define LIBMAD_TIMER_H

	typedef struct {
		signed long seconds;		/* whole seconds */
		unsigned long fraction;	/* 1/MAD_TIMER_RESOLUTION seconds */
	} aw_mad_timer_t;

	extern aw_mad_timer_t const mad_timer_zero;

# define MAD_TIMER_RESOLUTION	352800000UL

	typedef enum mad_units {
		AW_MAD_UNITS_HOURS	 =    -2,
		AW_MAD_UNITS_MINUTES	 =    -1,
		AW_MAD_UNITS_SECONDS	 =     0,

		/* metric units */

		AW_MAD_UNITS_DECISECONDS	 =    10,
		AW_MAD_UNITS_CENTISECONDS =   100,
		AW_MAD_UNITS_MILLISECONDS =  1000,

		/* audio sample units */

		AW_MAD_UNITS_8000_HZ	 =  8000,
		AW_MAD_UNITS_11025_HZ	 = 11025,
		AW_MAD_UNITS_12000_HZ	 = 12000,

		AW_MAD_UNITS_16000_HZ	 = 16000,
		AW_MAD_UNITS_22050_HZ	 = 22050,
		AW_MAD_UNITS_24000_HZ	 = 24000,

		AW_MAD_UNITS_32000_HZ	 = 32000,
		AW_MAD_UNITS_44100_HZ	 = 44100,
		AW_MAD_UNITS_48000_HZ	 = 48000,

		/* video frame/field units */

		AW_MAD_UNITS_24_FPS	 =    24,
		AW_MAD_UNITS_25_FPS	 =    25,
		AW_MAD_UNITS_30_FPS	 =    30,
		AW_MAD_UNITS_48_FPS	 =    48,
		AW_MAD_UNITS_50_FPS	 =    50,
		AW_MAD_UNITS_60_FPS	 =    60,

		/* CD audio frames */

		AW_MAD_UNITS_75_FPS	 =    75,

		/* video drop-frame units */

		AW_MAD_UNITS_23_976_FPS	 =   -24,
		AW_MAD_UNITS_24_975_FPS	 =   -25,
		AW_MAD_UNITS_29_97_FPS	 =   -30,
		AW_MAD_UNITS_47_952_FPS	 =   -48,
		AW_MAD_UNITS_49_95_FPS	 =   -50,
		AW_MAD_UNITS_59_94_FPS	 =   -60
	} aw_mad_units;

//# define mad_timer_reset(timer)	((void) (*(timer) = mad_timer_zero))

//int mad_timer_compare(aw_mad_timer_t, aw_mad_timer_t);

//# define mad_timer_sign(timer)	mad_timer_compare((timer), mad_timer_zero)

//void mad_timer_negate(aw_mad_timer_t *);
//aw_mad_timer_t mad_timer_abs(aw_mad_timer_t);

	void mad_timer_set(aw_mad_timer_t *, unsigned long, unsigned long, unsigned long);
	void mad_timer_add(aw_mad_timer_t *, aw_mad_timer_t);
	void mad_timer_multiply(aw_mad_timer_t *, signed long);

	signed long mad_timer_count(aw_mad_timer_t, aw_mad_units);
	unsigned long mad_timer_fraction(aw_mad_timer_t, unsigned long);
	void mad_timer_string(aw_mad_timer_t, char *, char const *,
	                      aw_mad_units, aw_mad_units, unsigned long);

# endif

	/* Id: stream.h,v 1.20 2004/02/05 09:02:39 rob Exp */

# ifndef LIBMAD_STREAM_H
# define LIBMAD_STREAM_H


# define MAD_BUFFER_GUARD	8
# define MAD_BUFFER_MDLEN	(511 + 2048 + MAD_BUFFER_GUARD)

	typedef enum mad_error {
		AW_MAD_ERROR_NONE	   = 0x0000,	/* no error */

		AW_MAD_ERROR_BUFLEN	   = 0x0001,	/* input buffer too small (or EOF) */
		AW_MAD_ERROR_BUFPTR	   = 0x0002,	/* invalid (null) buffer pointer */

		AW_MAD_ERROR_NOMEM	   = 0x0031,	/* not enough memory */

		AW_MAD_ERROR_LOSTSYNC	   = 0x0101,	/* lost synchronization */
		AW_MAD_ERROR_BADLAYER	   = 0x0102,	/* reserved header layer value */
		AW_MAD_ERROR_BADBITRATE	   = 0x0103,	/* forbidden bitrate value */
		AW_MAD_ERROR_BADSAMPLERATE  = 0x0104,	/* reserved sample frequency value */
		AW_MAD_ERROR_BADEMPHASIS	   = 0x0105,	/* reserved emphasis value */

		AW_MAD_ERROR_BADCRC	   = 0x0201,	/* CRC check failed */
		AW_MAD_ERROR_BADBITALLOC	   = 0x0211,	/* forbidden bit allocation value */
		AW_MAD_ERROR_BADSCALEFACTOR = 0x0221,	/* bad scalefactor index */
		AW_MAD_ERROR_BADMODE        = 0x0222,	/* bad bitrate/mode combination */
		AW_MAD_ERROR_BADFRAMELEN	   = 0x0231,	/* bad frame length */
		AW_MAD_ERROR_BADBIGVALUES   = 0x0232,	/* bad big_values count */
		AW_MAD_ERROR_BADBLOCKTYPE   = 0x0233,	/* reserved block_type */
		AW_MAD_ERROR_BADSCFSI	   = 0x0234,	/* bad scalefactor selection info */
		AW_MAD_ERROR_BADDATAPTR	   = 0x0235,	/* bad main_data_begin pointer */
		AW_MAD_ERROR_BADPART3LEN	   = 0x0236,	/* bad audio data length */
		AW_MAD_ERROR_BADHUFFTABLE   = 0x0237,	/* bad Huffman table select */
		AW_MAD_ERROR_BADHUFFDATA	   = 0x0238,	/* Huffman data overrun */
		AW_MAD_ERROR_BADSTEREO	   = 0x0239	/* incompatible block_type for JS */
	} aw_mad_error;

# define MAD_RECOVERABLE(error)	((error) & 0xff00)

	typedef struct mad_stream {
		unsigned char const *buffer;		/* input bitstream buffer */
		unsigned char const *bufend;		/* end of buffer */
		unsigned long skiplen;		/* bytes to skip before next frame */

		int sync;				/* stream sync found */
		unsigned long freerate;		/* free bitrate (fixed) */

		unsigned char const *this_frame;	/* start of current frame */
		unsigned char const *next_frame;	/* start of next frame */
		aw_mad_bitptr ptr;		/* current processing bit pointer */

		aw_mad_bitptr anc_ptr;		/* ancillary bits pointer */
		unsigned int anc_bitlen;		/* number of ancillary bits */

		unsigned char (*main_data)[MAD_BUFFER_MDLEN];
		/* Layer III main_data() */
		unsigned int md_len;			/* bytes in main_data */

		int options;				/* decoding options (see below) */
		aw_mad_error error;			/* error code (see above) */
	} aw_mad_stream;

	enum {
		AW_MAD_OPTION_IGNORECRC      = 0x0001,	/* ignore CRC errors */
		AW_MAD_OPTION_HALFSAMPLERATE = 0x0002	/* generate PCM at 1/2 sample rate */
# if 0  /* not yet implemented */
		AW_MAD_OPTION_LEFTCHANNEL    = 0x0010,	/* decode left channel only */
		AW_MAD_OPTION_RIGHTCHANNEL   = 0x0020,	/* decode right channel only */
		AW_MAD_OPTION_SINGLECHANNEL  = 0x0030	/* combine channels */
# endif
	};


# define mad_stream_options(stream, opts)  \
    ((void) ((stream)->options = (opts)))

	void mad_stream_buffer(aw_mad_stream *,
	                       unsigned char const *, unsigned long);





# endif

	/* Id: frame.h,v 1.20 2004/01/23 09:41:32 rob Exp */

# ifndef LIBAW_MAD_FRAME_H
# define LIBAW_MAD_FRAME_H


	typedef enum mad_layer {
		AW_MAD_LAYER_I   = 1,			/* Layer I */
		AW_MAD_LAYER_II  = 2,			/* Layer II */
		AW_MAD_LAYER_III = 3			/* Layer III */
	} aw_mad_layer;

	typedef enum mad_mode {
		AW_MAD_MODE_SINGLE_CHANNEL = 0,		/* single channel */
		AW_MAD_MODE_DUAL_CHANNEL	  = 1,		/* dual channel */
		AW_MAD_MODE_JOINT_STEREO	  = 2,		/* joint (MS/intensity) stereo */
		AW_MAD_MODE_STEREO	  = 3		/* normal LR stereo */
	} aw_mad_mode;

	typedef enum mad_emphasis {
		AW_MAD_EMPHASIS_NONE	  = 0,		/* no emphasis */
		AW_MAD_EMPHASIS_50_15_US	  = 1,		/* 50/15 microseconds emphasis */
		AW_MAD_EMPHASIS_CCITT_J_17 = 3,		/* CCITT J.17 emphasis */
		AW_MAD_EMPHASIS_RESERVED   = 2		/* unknown emphasis */
	} aw_mad_emphasis;

	typedef struct mad_header {
		aw_mad_layer layer;			/* audio layer (1, 2, or 3) */
		aw_mad_mode  mode;			/* channel mode (see above) */
		int mode_extension;			/* additional mode info */
		aw_mad_emphasis emphasis;		/* de-emphasis to use (see above) */

		unsigned long bitrate;		/* stream bitrate (bps) */
		unsigned int samplerate;		/* sampling frequency (Hz) */

		unsigned short crc_check;		/* frame CRC accumulator */
		unsigned short crc_target;		/* final target CRC checksum */

		int flags;				/* flags (see below) */
		int private_bits;			/* private bits (see below) */

		aw_mad_timer_t duration;			/* audio playing time of frame */
	} aw_mad_header;

	typedef struct mad_frame {
		aw_mad_header header;		/* MPEG audio header */

		int options;				/* decoding options (from stream) */

		aw_mad_fixed_t sbsample[2][36][32];	/* synthesis subband filter samples */
		aw_mad_fixed_t (*overlap)[2][32][18];	/* Layer III block overlap data */
	} aw_mad_frame;

# define MAD_NCHANNELS(header)		((header)->mode ? 2 : 1)
# define MAD_NSBSAMPLES(header)  \
  ((header)->layer == AW_MAD_LAYER_I ? 12 :  \
   (((header)->layer == AW_MAD_LAYER_III &&  \
     ((header)->flags & AW_MAD_FLAG_LSF_EXT)) ? 18 : 36))

	enum {
		AW_MAD_FLAG_NPRIVATE_III	= 0x0007,	/* number of Layer III private bits */
		AW_MAD_FLAG_INCOMPLETE	= 0x0008,	/* header but not data is decoded */

		AW_MAD_FLAG_PROTECTION	= 0x0010,	/* frame has CRC protection */
		AW_MAD_FLAG_COPYRIGHT	= 0x0020,	/* frame is copyright */
		AW_MAD_FLAG_ORIGINAL	= 0x0040,	/* frame is original (else copy) */
		AW_MAD_FLAG_PADDING	= 0x0080,	/* frame has additional slot */

		AW_MAD_FLAG_I_STEREO	= 0x0100,	/* uses intensity joint stereo */
		AW_MAD_FLAG_MS_STEREO	= 0x0200,	/* uses middle/side joint stereo */
		AW_MAD_FLAG_FREEFORMAT	= 0x0400,	/* uses free format bitrate */

		AW_MAD_FLAG_LSF_EXT	= 0x1000,	/* lower sampling freq. extension */
		AW_MAD_FLAG_MC_EXT	= 0x2000,	/* multichannel audio extension */
		AW_MAD_FLAG_MPEG_2_5_EXT	= 0x4000	/* MPEG 2.5 (unofficial) extension */
	};

	enum {
		AW_MAD_PRIVATE_HEADER	= 0x0100,	/* header private bit */
		AW_MAD_PRIVATE_III	= 0x001f	/* Layer III private bits (up to 5) */
	};

//void mad_header_init(aw_mad_header *);

//# define mad_header_finish(header)  /* nothing */

	int mad_header_decode(aw_mad_header *, aw_mad_stream *);

//void mad_frame_init(void);
//void mad_frame_finish(void);

	int mad_frame_decode(aw_mad_frame *, aw_mad_stream *);

	void mad_frame_mute(aw_mad_frame *);

# endif

	/* Id: synth.h,v 1.15 2004/01/23 09:41:33 rob Exp */

# ifndef LIBAW_MAD_SYNTH_H
# define LIBAW_MAD_SYNTH_H


	typedef struct mad_pcm {
		unsigned int samplerate;		/* sampling frequency (Hz) */
		unsigned short channels;		/* number of channels */
		unsigned short length;		/* number of samples per channel */
		aw_mad_fixed_t samples[2][1152];		/* PCM output samples [ch][sample] */
	} aw_mad_pcm;

	typedef struct mad_synth {
		aw_mad_fixed_t filter[2][2][2][16][8];	/* polyphase filterbank outputs */
		/* [ch][eo][peo][s][v] */

		unsigned int phase;			/* current processing phase */

		aw_mad_pcm pcm;			/* PCM output */
	} aw_mad_synth;

	/* single channel PCM selector */
	enum {
		AW_MAD_PCM_CHANNEL_SINGLE = 0
	};

	/* dual channel PCM selector */
	enum {
		AW_MAD_PCM_CHANNEL_DUAL_1 = 0,
		AW_MAD_PCM_CHANNEL_DUAL_2 = 1
	};

	/* stereo PCM selector */
	enum {
		AW_MAD_PCM_CHANNEL_STEREO_LEFT  = 0,
		AW_MAD_PCM_CHANNEL_STEREO_RIGHT = 1
	};

//void mad_synth_init(AwMP3DecInfo *mp3infohdl);
//void mad_synth_mute(AwMP3DecInfo *mp3infohdl);


# define mad_synth_finish(synth)  /* nothing */



//void mad_synth_frame(FrameHeader *fh);

# endif

	/* Id: decoder.h,v 1.17 2004/01/23 09:41:32 rob Exp */

# ifndef LIBAW_MAD_DECODER_H
# define LIBAW_MAD_DECODER_H


	typedef enum mad_decoder_mode {
		AW_MAD_DECODER_MODE_SYNC  = 0,
		AW_MAD_DECODER_MODE_ASYNC
	} aw_mad_decoder_mode;

	typedef enum mad_flow {
		AW_MAD_FLOW_CONTINUE = 0x0000,	/* continue normally */
		AW_MAD_FLOW_STOP     = 0x0010,	/* stop decoding normally */
		AW_MAD_FLOW_BREAK    = 0x0011,	/* stop decoding and signal an error */
		AW_MAD_FLOW_IGNORE   = 0x0020	/* ignore the current frame */
	} aw_mad_flow;

	typedef struct mad_decoder {
		aw_mad_decoder_mode mode;

		int options;

		struct {
			long pid;
			int in;
			int out;
		} async;

		struct {
			aw_mad_stream stream;
			aw_mad_frame frame;
			aw_mad_synth synth;
		} *sync;

		void *cb_data;

		aw_mad_flow (*input_func)(void *, aw_mad_stream *);
		aw_mad_flow (*header_func)(void *, aw_mad_header const *);
		aw_mad_flow (*filter_func)(void *,
		                           aw_mad_stream const *, aw_mad_frame *);
		aw_mad_flow (*output_func)(void *,
		                           aw_mad_header const *, aw_mad_pcm *);
		aw_mad_flow (*error_func)(void *, aw_mad_stream *, aw_mad_frame *);
		aw_mad_flow (*message_func)(void *, void *, unsigned int *);
	} aw_mad_decoder;

	void mad_decoder_init(aw_mad_decoder *, void *,
	                      aw_mad_flow (*)(void *, aw_mad_stream *),
	                      aw_mad_flow (*)(void *, aw_mad_header const *),
	                      aw_mad_flow (*)(void *,
	                                      aw_mad_stream const *,
	                                      aw_mad_frame *),
	                      aw_mad_flow (*)(void *,
	                                      aw_mad_header const *,
	                                      aw_mad_pcm *),
	                      aw_mad_flow (*)(void *,
	                                      aw_mad_stream *,
	                                      aw_mad_frame *),
	                      aw_mad_flow (*)(void *, void *, unsigned int *));
	int mad_decoder_finish(aw_mad_decoder *);

# define mad_decoder_options(decoder, opts)  \
    ((void) ((decoder)->options = (opts)))

	int mad_decoder_run(aw_mad_decoder *, aw_mad_decoder_mode);
	int mad_decoder_message(aw_mad_decoder *, void *, unsigned int *);

# endif

# ifdef __cplusplus
}
# endif
#endif
#endif//_ROM_MAD_H_
