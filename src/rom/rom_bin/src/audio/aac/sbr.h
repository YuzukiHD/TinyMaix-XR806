#ifndef _ROM_SBR_H
#define _ROM_SBR_H
#ifdef CONFIG_ROM
#include <stdio.h>
#define STAT_PREFIX		raac

#define STATCC1(x,y,z)	STATCC2(x,y,z)
#define STATCC2(x,y,z)	x##y##z

#ifdef STAT_PREFIX
#define STATNAME(func)	STATCC1(STAT_PREFIX, _, func)
#else
#define STATNAME(func)	func
#endif

#define STATRENAME(func)	aw##_##func


#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif


/* do x <<= n, clipping to range [-2^30, 2^30 - 1] (i.e. output has one guard bit) */
#define CLIP_2N_SHIFT30(x, n) { \
	int sign = (x) >> 31;  \
	if (sign != (x) >> (30 - (n)))  { \
		(x) = sign ^ (0x3fffffff); \
	} else { \
		(x) = (x) << (n); \
	} \
}
#define FBITS_IN_QMFA	14
#define NUM_QMF_DELAY_BUFS	10

#define n_cTabA							STATNAME(n_cTabA)
#define n_cTabS							STATNAME(n_cTabS)

#define FBITS_IN_QMFA	14
#define FBITS_LOST_QMFA	(1 + 2 + 3 + 2 + 1)	/* 1 from cTab, 2 in premul, 3 in FFT, 2 in postmul, 1 for implicit scaling by 2.0 */
#define FBITS_OUT_QMFA	(FBITS_IN_QMFA - FBITS_LOST_QMFA)

#define MIN_GBITS_IN_QMFS		2
#define FBITS_IN_QMFS			FBITS_OUT_QMFA
#define FBITS_LOST_DCT4_64		(2 + 3 + 2)		/* 2 in premul, 3 in FFT, 2 in postmul */
extern const int n_cTabA[165];
extern const int n_cTabS[640];

/* rom_sbrfft.c */
void FFT_32C(int *x);
#endif
#endif
