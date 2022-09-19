#ifdef CONFIG_ROM
#include "sbr.h"
#include "assembly.h"
#include "rom/compiler.h"

/* Pre_Multiply64() table
 * n_format = Q30
 * reordered for sequential access
 *
 * for (i = 0; i < 64/4; i++) {
 *   angle = (i + 0.25) * M_PI / nmdct;
 *   x = (cos(angle) + sin(angle));
 *   x =  sin(angle);
 *
 *   angle = (nmdct/2 - 1 - i + 0.25) * M_PI / nmdct;
 *   x = (cos(angle) + sin(angle));
 *   x =  sin(angle);
 * }
 */
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_rodata
#endif
static const int cos4sin4_tab64[64] = {
	0x40c7d2bd, 0x00c90e90, 0x424ff28f, 0x3ff4e5e0, 0x43cdd89a, 0x03ecadcf, 0x454149fc, 0x3fc395f9,
	0x46aa0d6d, 0x070de172, 0x4807eb4b, 0x3f6af2e3, 0x495aada2, 0x0a2abb59, 0x4aa22036, 0x3eeb3347,
	0x4bde1089, 0x0d415013, 0x4d0e4de2, 0x3e44a5ef, 0x4e32a956, 0x104fb80e, 0x4f4af5d1, 0x3d77b192,
	0x50570819, 0x135410c3, 0x5156b6d9, 0x3c84d496, 0x5249daa2, 0x164c7ddd, 0x53304df6, 0x3b6ca4c4,
	0x5409ed4b, 0x19372a64, 0x54d69714, 0x3a2fcee8, 0x55962bc0, 0x1c1249d8, 0x56488dc5, 0x38cf1669,
	0x56eda1a0, 0x1edc1953, 0x57854ddd, 0x374b54ce, 0x580f7b19, 0x2192e09b, 0x588c1404, 0x35a5793c,
	0x58fb0568, 0x2434f332, 0x595c3e2a, 0x33de87de, 0x59afaf4c, 0x26c0b162, 0x59f54bee, 0x31f79948,
	0x5a2d0957, 0x29348937, 0x5a56deec, 0x2ff1d9c7, 0x5a72c63b, 0x2b8ef77d, 0x5a80baf6, 0x2dce88aa,
};

/* Post_Multiply64() table
 * n_format = Q30
 * reordered for sequential access
 *
 * for (i = 0; i <= (32/2); i++) {
 *   angle = i * M_PI / 64;
 *   x = (cos(angle) + sin(angle));
 *   x = sin(angle);
 * }
 */
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_rodata
#endif
static const int cos1sin1_tab64[34] = {
	0x40000000, 0x00000000, 0x43103085, 0x0323ecbe, 0x45f704f7, 0x0645e9af, 0x48b2b335, 0x09640837,
	0x4b418bbe, 0x0c7c5c1e, 0x4da1fab5, 0x0f8cfcbe, 0x4fd288dc, 0x1294062f, 0x51d1dc80, 0x158f9a76,
	0x539eba45, 0x187de2a7, 0x553805f2, 0x1b5d100a, 0x569cc31b, 0x1e2b5d38, 0x57cc15bc, 0x20e70f32,
	0x58c542c5, 0x238e7673, 0x5987b08a, 0x261feffa, 0x5a12e720, 0x2899e64a, 0x5a6690ae, 0x2afad269,
	0x5a82799a, 0x2d413ccd,
};

/**************************************************************************************
 * Function:    Pre_Multiply64
 *
 * Description: pre-twiddle stage of 64-point DCT-IV
 *
 * Inputs:      buffer of 64 samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out, gains 2 int bits
 *              gbOut = gbIn + 1
 *              output is limited to sqrt(2)/2 plus GB in full GB
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 **************************************************************************************/
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
static void Pre_Multiply64(int *zbuf1)
{
	int i, a_r1, a_i1, a_r2, a_i2, z_1, z_2;
	int t, cm_s2, cps_2a, sin_2a, cps_2b, sin_2b;
	int *pzbuf2;
	const int *pcsptr;

	pzbuf2 = zbuf1 + 64 - 1;
	pcsptr = cos4sin4_tab64;

	/* whole thing should fit in registers - verify that compiler does this */
	for (i = 64 >> 2; i != 0; i--) {
		/* cp_s2 = (cos+sin), sin_2 = sin, cm_s2 = (cos-sin) */
		cps_2a = *pcsptr++;
		sin_2a = *pcsptr++;
		cps_2b = *pcsptr++;
		sin_2b = *pcsptr++;

		a_r1 = *(zbuf1 + 0);
		a_i2 = *(zbuf1 + 1);
		a_i1 = *(pzbuf2 + 0);
		a_r2 = *(pzbuf2 - 1);

		/* gain 2 ints bit from MULSHIFT32 by Q30
		 * max per-sample gain (ignoring implicit scaling) = MAX(sin(angle)+cos(angle)) = 1.414
		 * i.e. gain 1 GB since worst case is sin(angle) = cos(angle) = 0.707 (Q30), gain 2 from
		 *   extra sign bits, and eat one in adding
		 */
		t  = MULSHIFT32(sin_2a, a_r1 + a_i1);
		z_2 = MULSHIFT32(cps_2a, a_i1) - t;
		cm_s2 = cps_2a - 2*sin_2a;
		z_1 = MULSHIFT32(cm_s2, a_r1) + t;
		*zbuf1++ = z_1;	/* cos*a_r1 + sin*a_i1 */
		*zbuf1++ = z_2;	/* cos*a_i1 - sin*a_r1 */

		t  = MULSHIFT32(sin_2b, a_r2 + a_i2);
		z_2 = MULSHIFT32(cps_2b, a_i2) - t;
		cm_s2 = cps_2b - 2*sin_2b;
		z_1 = MULSHIFT32(cm_s2, a_r2) + t;
		*pzbuf2-- = z_2;	/* cos*a_i2 - sin*a_r2 */
		*pzbuf2-- = z_1;	/* cos*a_r2 + sin*a_i2 */
	}
}

/**************************************************************************************
 * Function:    Post_Multiply64
 *
 * Description: post-twiddle stage of 64-point type-IV DCT
 *
 * Inputs:      buffer of 64 samples
 *              number of output samples to calculate
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out, gains 2 int bits
 *              gbOut = gbIn + 1
 *              output is limited to sqrt(2)/2 plus GB in full GB
 *              nSampsOut is rounded up to next multiple of 4, since we calculate
 *                4 samples per loop
 **************************************************************************************/
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
static void Post_Multiply64(int *fft1, int nSampsOut)
{
	int i, a_r1, a_i1, a_r2, a_i2;
	int t, cm_s2, cp_s2, sin_2;
	int *pfft2;
	const int *pcsptr;

	pcsptr = cos1sin1_tab64;
	pfft2 = fft1 + 64 - 1;

	/* load coeffs for first pass
	 * cp_s2 = (cos+sin)/2, sin_2 = sin/2, cm_s2 = (cos-sin)/2
	 */
	cp_s2 = *pcsptr++;
	sin_2 = *pcsptr++;
	cm_s2 = cp_s2 - 2*sin_2;

	for (i = (nSampsOut + 3) >> 2; i != 0; i--) {
		a_r1 = *(fft1 + 0);
		a_i1 = *(fft1 + 1);
		a_r2 = *(pfft2 - 1);
		a_i2 = *(pfft2 + 0);

		/* gain 2 int bits (multiplying by Q30), max gain = sqrt(2) */
		t = MULSHIFT32(sin_2, a_r1 + a_i1);
		*pfft2-- = t - MULSHIFT32(cp_s2, a_i1);
		*fft1++ = t + MULSHIFT32(cm_s2, a_r1);

		cp_s2 = *pcsptr++;
		sin_2 = *pcsptr++;

		a_i2 = -a_i2;
		t = MULSHIFT32(sin_2, a_r2 + a_i2);
		*pfft2-- = t - MULSHIFT32(cp_s2, a_i2);
		cm_s2 = cp_s2 - 2*sin_2;
		*fft1++ = t + MULSHIFT32(cm_s2, a_r2);
	}
}

/**************************************************************************************
 * Function:    QMF_AnalysisConv
 *
 * Description: convolution kernel for analysis QMF
 *
 * Inputs:      pointer to coefficient table, reordered for sequential access
 *              delay buffer of size 32*10 = 320 real-valued PCM samples
 *              index for delay ring buffer (range = [0, 9])
 *
 * Outputs:     64 consecutive 32-bit samples
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrqmfak.s when building for ARM!
 **************************************************************************************/
#if (0)&&(!defined (__LP64__))&&((defined (__arm) && defined (__ARMCC_VERSION)) || (defined (_WIN32) && defined (_WIN32_WCE) && defined (ARM)) || (defined(__GNUC__) && defined(__arm__)))
extern void QMF_AnalysisConv(int *cTab, int *delay, int d_Idx, int *puBuf);
#else
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
static void QMF_AnalysisConv(int *cTab, int *delay, int d_Idx, int *puBuf)
{
	int i, d_Off;
	int *pcPtr0, *pcPtr1;
	U64 u64_lo, u64_hi;

	d_Off = d_Idx*32 + 31;
	pcPtr0 = cTab;
	pcPtr1 = cTab + 33*5 - 1;

	/* special first pass since we need to flip sign to create cTab[384], cTab[512] */
	u64_lo.w64 = 0;
	u64_hi.w64 = 0;
	u64_lo.w64 = MADD64(u64_lo.w64,  *pcPtr0++,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_hi.w64 = MADD64(u64_hi.w64,  *pcPtr0++,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_lo.w64 = MADD64(u64_lo.w64,  *pcPtr0++,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_hi.w64 = MADD64(u64_hi.w64,  *pcPtr0++,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_lo.w64 = MADD64(u64_lo.w64,  *pcPtr0++,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_hi.w64 = MADD64(u64_hi.w64,  *pcPtr1--,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_lo.w64 = MADD64(u64_lo.w64, -(*pcPtr1--), delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_hi.w64 = MADD64(u64_hi.w64,  *pcPtr1--,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_lo.w64 = MADD64(u64_lo.w64, -(*pcPtr1--), delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
	u64_hi.w64 = MADD64(u64_hi.w64,  *pcPtr1--,   delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}

	puBuf[0]  = u64_lo.r.s_hi32;
	puBuf[32] = u64_hi.r.s_hi32;
	puBuf++;
	d_Off--;

	/* max gain for any sample in puBuf, after scaling by cTab, ~= 0.99
	 * so we can just sum the puBuf values with no overflow problems
	 */
	for (i = 1; i <= 31; i++) {
		u64_lo.w64 = 0;
		u64_hi.w64 = 0;
		u64_lo.w64 = MADD64(u64_lo.w64, *pcPtr0++, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_hi.w64 = MADD64(u64_hi.w64, *pcPtr0++, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_lo.w64 = MADD64(u64_lo.w64, *pcPtr0++, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_hi.w64 = MADD64(u64_hi.w64, *pcPtr0++, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_lo.w64 = MADD64(u64_lo.w64, *pcPtr0++, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_hi.w64 = MADD64(u64_hi.w64, *pcPtr1--, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_lo.w64 = MADD64(u64_lo.w64, *pcPtr1--, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_hi.w64 = MADD64(u64_hi.w64, *pcPtr1--, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_lo.w64 = MADD64(u64_lo.w64, *pcPtr1--, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}
		u64_hi.w64 = MADD64(u64_hi.w64, *pcPtr1--, delay[d_Off]);	d_Off -= 32; if (d_Off < 0) {d_Off += 320;}

		puBuf[0]  = u64_lo.r.s_hi32;
		puBuf[32] = u64_hi.r.s_hi32;
		puBuf++;
		d_Off--;
	}
}
#endif

/**************************************************************************************
 * Function:    QMF_Analysis
 *
 * Description: 32-subband analysis QMF (4.6.18.4.1)
 *
 * Inputs:      32 consecutive samples of decoded 32-bit PCM, n_format = Q(fBitsIn)
 *              delay buffer of size 32*10 = 320 PCM samples
 *              number of fraction bits in input PCM
 *              index for delay ring buffer (range = [0, 9])
 *              number of subbands to calculate (range = [0, 32])
 *
 * Outputs:     qmfaBands complex subband samples, n_format = Q(FBITS_OUT_QMFA)
 *              updated delay buffer
 *              updated delay index
 *
 * Return:      guard bit mask
 *
 * Notes:       output stored as RE{X0}, IM{X0}, RE{X1}, IM{X1}, ... RE{X31}, IM{X31}
 *              output stored in int buffer of size 64*2 = 128
 *                (zero-filled from X_Buf[2*qmfaBands] to X_Buf[127])
 **************************************************************************************/
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
int QMF_Analysis(int *inbuf, int *delay, int *X_Buf, int fBitsIn, int *delayIdx, int qmfaBands)
{
	int n, y, shift, gb_Mask;
	int *pdelayPtr, *puBuf, *ptBuf;

	/* use X_Buf[128] as temp buffer for reordering */
	puBuf = X_Buf;		/* first 64 samples */
	ptBuf = X_Buf + 64;	/* second 64 samples */

	/* overwrite oldest PCM with new PCM
	 * delay[n] has 1 GB after shifting (either << or >>)
	 */
	pdelayPtr = delay + (*delayIdx * 32);
	if (fBitsIn > FBITS_IN_QMFA) {
		shift = MIN(fBitsIn - FBITS_IN_QMFA, 31);
		for (n = 32; n != 0; n--) {
			y = (*inbuf) >> shift;
			inbuf++;
			*pdelayPtr++ = y;
		}
	} else {
		shift = MIN(FBITS_IN_QMFA - fBitsIn, 30);
		for (n = 32; n != 0; n--) {
			y = *inbuf++;
			CLIP_2N_SHIFT30(y, shift);
			*pdelayPtr++ = y;
		}
	}

	QMF_AnalysisConv((int *)n_cTabA, delay, *delayIdx, puBuf);

	/* puBuf has at least 2 GB right now (1 from clipping to Q(FBITS_IN_QMFA), one from
	 *   the scaling by cTab (MULSHIFT32(*pdelayPtr--, *cPtr++), with net gain of < 1.0)
	 * TODO - fuse with QMF_AnalysisConv to avoid separate reordering
	 */
    ptBuf[2*0 + 0] = puBuf[0];
    ptBuf[2*0 + 1] = puBuf[1];
    for (n = 1; n < 31; n++) {
        ptBuf[2*n + 0] = -puBuf[64-n];
        ptBuf[2*n + 1] =  puBuf[n+1];
    }
    ptBuf[2*31 + 1] =  puBuf[32];
    ptBuf[2*31 + 0] = -puBuf[33];

	/* fast in-place DCT-IV - only need 2*qmfaBands output samples */
	Pre_Multiply64(ptBuf);	/* 2 GB in, 3 GB out */
	FFT_32C(ptBuf);			/* 3 GB in, 1 GB out */
	Post_Multiply64(ptBuf, qmfaBands*2);	/* 1 GB in, 2 GB out */

	/* TODO - roll into PostMultiply (if enough registers) */
	gb_Mask = 0;
	for (n = 0; n < qmfaBands; n++) {
		X_Buf[2*n+0] =  ptBuf[ n + 0];	/* implicit scaling of 2 in our output Q n_format */
		gb_Mask |= FASTABS(X_Buf[2*n+0]);
		X_Buf[2*n+1] = -ptBuf[63 - n];
		gb_Mask |= FASTABS(X_Buf[2*n+1]);
	}

	/* fill top section with zeros for HF generation */
	for (    ; n < 64; n++) {
		X_Buf[2*n+0] = 0;
		X_Buf[2*n+1] = 0;
	}

	*delayIdx = (*delayIdx == NUM_QMF_DELAY_BUFS - 1 ? 0 : *delayIdx + 1);

	/* minimum of 2 GB in output */
	return gb_Mask;
}

/* lose FBITS_LOST_DCT4_64 in DCT4, gain 6 for implicit scaling by 1/64, lose 1 for cTab multiply (Q31) */
#define FBITS_OUT_QMFS	(FBITS_IN_QMFS - FBITS_LOST_DCT4_64 + 6 - 1)
#define RND_VAL			(1 << (FBITS_OUT_QMFS-1))

/**************************************************************************************
 * Function:    QMF_SynthesisConv
 *
 * Description: final convolution kernel for synthesis QMF
 *
 * Inputs:      pointer to coefficient table, reordered for sequential access
 *              delay buffer of size 64*10 = 640 complex samples (1280 ints)
 *              index for delay ring buffer (range = [0, 9])
 *              number of QMF subbands to process (range = [0, 64])
 *              number of channels
 *
 * Outputs:     64 consecutive 16-bit PCM samples, interleaved by factor of nChans
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrqmfsk.s when building for ARM!
 **************************************************************************************/
#if (0)&&(!defined (__LP64__))&&((defined (__arm) && defined (__ARMCC_VERSION)) || (defined (_WIN32) && defined (_WIN32_WCE) && defined (ARM)) || (defined(__GNUC__) && defined(__arm__)))
extern void QMF_SynthesisConv(int *cPtr, int *delay, int d_Idx, short *outbuf, int nChans);
#else
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
static void QMF_SynthesisConv(int *cPtr, int *delay, int d_Idx, short *outbuf, int nChans)
{
	int i, dOff_0, dOff_1;
	U64 sum_64;

	dOff_0 = (d_Idx)*128;
	dOff_1 = dOff_0 - 1;
	if (dOff_1 < 0)
		dOff_1 += 1280;

	/* scaling note: total gain of coefs (cPtr[0]-cPtr[9] for any i) is < 2.0, so 1 GB in delay values is adequate */
	for (i = 0; i <= 63; i++) {
		sum_64.w64 = 0;
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_0]);	dOff_0 -= 256; if (dOff_0 < 0) {dOff_0 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_1]);	dOff_1 -= 256; if (dOff_1 < 0) {dOff_1 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_0]);	dOff_0 -= 256; if (dOff_0 < 0) {dOff_0 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_1]);	dOff_1 -= 256; if (dOff_1 < 0) {dOff_1 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_0]);	dOff_0 -= 256; if (dOff_0 < 0) {dOff_0 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_1]);	dOff_1 -= 256; if (dOff_1 < 0) {dOff_1 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_0]);	dOff_0 -= 256; if (dOff_0 < 0) {dOff_0 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_1]);	dOff_1 -= 256; if (dOff_1 < 0) {dOff_1 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_0]);	dOff_0 -= 256; if (dOff_0 < 0) {dOff_0 += 1280;}
		sum_64.w64 = MADD64(sum_64.w64, *cPtr++, delay[dOff_1]);	dOff_1 -= 256; if (dOff_1 < 0) {dOff_1 += 1280;}

		dOff_0++;
		dOff_1--;
		*outbuf = CLIPTOSHORT((sum_64.r.s_hi32 + RND_VAL) >> FBITS_OUT_QMFS);
		outbuf += nChans;
	}
}
#endif

/**************************************************************************************
 * Function:    QMF_Synthesis
 *
 * Description: 64-subband synthesis QMF (4.6.18.4.2)
 *
 * Inputs:      64 consecutive complex subband QMF samples, n_format = Q(FBITS_IN_QMFS)
 *              delay buffer of size 64*10 = 640 complex samples (1280 ints)
 *              index for delay ring buffer (range = [0, 9])
 *              number of QMF subbands to process (range = [0, 64])
 *              number of channels
 *
 * Outputs:     64 consecutive 16-bit PCM samples, interleaved by factor of nChans
 *              updated delay buffer
 *              updated delay index
 *
 * Return:      none
 *
 * Notes:       assumes MIN_GBITS_IN_QMFS guard bits in input, either from
 *                QMF_Analysis (if upsampling only) or from MapHF (if SBR on)
 **************************************************************************************/
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
void QMF_Synthesis(int *inbuf, int *delay, int *delayIdx, int qmfs_Bands, short *outbuf, int nChans)
{
	int n, a_0, a_1, b_0, b_1, dOff_0, dOff_1, d_Idx;
	int *ptBufLo, *ptBufHi;

	d_Idx = *delayIdx;
	ptBufLo = delay + d_Idx*128 + 0;
	ptBufHi = delay + d_Idx*128 + 127;

	/* reorder inputs to DCT-IV, only use first qmfs_Bands (complex) samples
	 * TODO - fuse with Pre_Multiply64 to avoid separate reordering steps
	 */
    for (n = 0; n < qmfs_Bands >> 1; n++) {
		a_0 = *inbuf++;
		b_0 = *inbuf++;
		a_1 = *inbuf++;
		b_1 = *inbuf++;
		*ptBufLo++ = a_0;
        *ptBufLo++ = a_1;
        *ptBufHi-- = b_0;
        *ptBufHi-- = b_1;
    }
	if (qmfs_Bands & 0x01) {
		a_0 = *inbuf++;
		b_0 = *inbuf++;
		*ptBufLo++ = a_0;
        *ptBufHi-- = b_0;
        *ptBufLo++ = 0;
		*ptBufHi-- = 0;
		n++;
	}
    for (     ; n < 32; n++) {
		*ptBufLo++ = 0;
        *ptBufHi-- = 0;
        *ptBufLo++ = 0;
        *ptBufHi-- = 0;
	}

	ptBufLo = delay + d_Idx*128 + 0;
	ptBufHi = delay + d_Idx*128 + 64;

	/* 2 GB in, 3 GB out */
	Pre_Multiply64(ptBufLo);
	Pre_Multiply64(ptBufHi);

	/* 3 GB in, 1 GB out */
	FFT_32C(ptBufLo);
	FFT_32C(ptBufHi);

	/* 1 GB in, 2 GB out */
	Post_Multiply64(ptBufLo, 64);
	Post_Multiply64(ptBufHi, 64);

	/* could fuse with Post_Multiply64 to avoid separate pass */
	dOff_0 = d_Idx*128;
	dOff_1 = d_Idx*128 + 64;
	for (n = 32; n != 0; n--) {
		a_0 =  (*ptBufLo++);
		a_1 =  (*ptBufLo++);
		b_0 =  (*ptBufHi++);
		b_1 = -(*ptBufHi++);

		delay[dOff_0++] = (b_0 - a_0);
		delay[dOff_0++] = (b_1 - a_1);
		delay[dOff_1++] = (b_0 + a_0);
		delay[dOff_1++] = (b_1 + a_1);
	}

	QMF_SynthesisConv((int *)n_cTabS, delay, d_Idx, outbuf, nChans);

	*delayIdx = (*delayIdx == NUM_QMF_DELAY_BUFS - 1 ? 0 : *delayIdx + 1);
}
#endif
