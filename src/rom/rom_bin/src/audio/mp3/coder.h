#ifndef ROM_CODER_H
#define ROM_CODER_H
#ifdef CONFIG_ROM
#include "mp3common.h"

#if defined(AW_ASSERT)
#undef AW_ASSERT
#endif
#if defined(_WIN32) && defined(_M_IX86) && (defined (_DEBUG) || defined (REL_ENABLE_ASSERTS))
#define AW_ASSERT(x) if (!(x)) __asm int 3;
#else
#define AW_ASSERT(x) /* do nothing */
#endif

#ifndef AW_MIN
#define AW_MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

#ifndef AW_MAX
#define AW_MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

/* clip to the range [-2^x, 2^x - 1] */
#define AW_CLIP_TO_2N(a, x) { \
	int ss = (a) >> 31;  \
	if (ss != (a) >> (x))  { \
		(a) = ss ^ ((1 << (x)) - 1); \
	} \
}

#define AW_SIBYTES_MPEG1_MONO		17
#define AW_SIBYTES_MPEG1_STEREO	    32
#define AW_SIBYTES_MPEG2_MONO		 9
#define AW_SIBYTES_MPEG2_STEREO	    17

/* number of fraction bits for pow43Tab (see comments there) */
#define AW_POW43_FRACBITS_LOW		22
#define AW_POW43_FRACBITS_HIGH		12

#define AW_DQ_FRACBITS_OUT			25	/*NOTE: number of fraction bits in output of dequant */
#define	AW_IMDCT_SCALE				2	/* additional scaling (by sqrt(2)) for fast IMDCT36 */

#define	AW_HUFF_PAIRTABS			32
#define AW_BLOCK_SIZE				18
#define	AW_MP3_NBANDS					32
#define AW_MP3_MAX_REORDER_SAMPS		((192-126)*3)		/* largest critical band for short blocks (see sfBandTable) */
#define AW_MP3_VBUF_LENGTH				(17 * 2 * AW_MP3_NBANDS)	/* for double-sized vbuf FIFO */

/* additional external symbols to name-mangle for static linking */
//#define	AwSetBitstreamPointer	STATNAME(AwSetBitstreamPointer)
//#define	AwGetBits				STATNAME(AwGetBits)
//#define	AwCalcBitsUsed			STATNAME(AwCalcBitsUsed)
//#define	AwDequantChannel		STATNAME(AwDequantChannel)
//#define	AwMidSideProc			STATNAME(AwMidSideProc)
//#define	AwIntensityProcMPEG1	STATNAME(AwIntensityProcMPEG1)
//#define	AwIntensityProcMPEG2	STATNAME(AwIntensityProcMPEG2)
//#define AwPolyphaseMono			STATNAME(AwPolyphaseMono)
//#define AwPolyphaseStereo		STATNAME(AwPolyphaseStereo)
//#define AWFDCT32				STATNAME(AWFDCT32)
//
//#define	AWISFMpeg1				STATNAME(AWISFMpeg1)
//#define	AWISFMpeg2				STATNAME(AWISFMpeg2)
//#define	AWISFIIP				STATNAME(AWISFIIP)
//#define AWuniqueIDTab			STATNAME(AWuniqueIDTab)
//#define	AWcoef32				STATNAME(AWcoef32)
//#define	AWpolyCoef				STATNAME(AWpolyCoef)
//#define	AWcsa					STATNAME(AWcsa)
//#define	AWimdctWin				STATNAME(AWimdctWin)
//
//#define	AWhuffTable				STATNAME(AWhuffTable)
//#define	AWhuffTabOffset			STATNAME(AWhuffTabOffset)
//#define	AWhuffTabLookup			STATNAME(AWhuffTabLookup)
//#define	AWquadTable				STATNAME(AWquadTable)
//#define	AWquadTabOffset			STATNAME(AWquadTabOffset)
//#define	AWquadTabMaxBits		STATNAME(AWquadTabMaxBits)

/* map these to the corresponding 2-bit values in the frame header */
typedef enum _StereoMode {
	AW_Stereo = 0x00,	/* two independent channels, but L and R frames might have different # of bits */
	AW_Joint = 0x01,	/* coupled channels - layer III: mix of M-S and intensity, Layers I/II: intensity and direct coding only */
	AW_Dual = 0x02,	/* two independent channels, L and R always have exactly 1/2 the total bitrate */
	AW_Mono = 0x03		/* one channel */
} AW_StereoMode;


typedef struct _FrameHeader {
	int verIdx;
	MPEGVersion ver;	/* version ID */
	int layer;			/* layer index (1, 2, or 3) */
	int crc;			/* CRC flag: 0 = disabled, 1 = enabled */
	int brIdx;			/* bitrate index (0 - 15) */
	int srIdx;			/* sample rate index (0 - 2) */
	int paddingBit;		/* padding flag: 0 = no padding, 1 = single pad byte */
	int privateBit;		/* unused */
	AW_StereoMode sMode;	/* mono/stereo mode */
	int modeExt;		/* used to decipher joint stereo mode */
	int copy_orig_emphasis;		/* copyright flag: 0 = no, 1 = yes */
	int CRCWord;		/* CRC word (16 bits, 0 if crc not enabled) */

	int framesize;
	int samprate;
	int bitrate;
	int framesamplecount;

	const SFBandTable *sfBand;
} AwFrameHeader;

typedef struct _BitStreamInfo AW_BitStreamInfo;

struct _BitStreamInfo {
	unsigned char *bytePtr;
	unsigned int iCache;
	int cachedBits;
	int nBytes;
};

typedef struct _AW_SideInfoSub {
	int part23Length;		/* number of bits in main data */
	int nBigvals;			/* 2x this = first set of Huffman cw's (maximum amplitude can be > 1) */
	int globalGain;			/* overall gain for dequantizer */
	int sfCompress;			/* unpacked to figure out number of bits in scale factors */
	int winSwitchFlag;		/* window switching flag */
	int blockType;			/* block type */
	int mixedBlock;			/* 0 = regular block (all short or long), 1 = mixed block */
	int tableSelect[3];		/* index of Huffman tables for the big values regions */
	int subBlockGain[3];	/* subblock gain offset, relative to global gain */
	int region0Count;		/* 1+region0Count = num scale factor bands in first region of bigvals */
	int region1Count;		/* 1+region1Count = num scale factor bands in second region of bigvals */
	int preFlag;			/* for optional high frequency boost */
	int sfactScale;			/* scaling of the scalefactors */
	int count1TableSelect;	/* index of Huffman table for quad codewords */
} AW_SideInfoSub;

typedef struct _AW_SideInfo {
	int mainDataBegin;
	int privateBits;
	int scfsi[MAX_NCHAN][MAX_SCFBD];				/* 4 scalefactor bands per channel */

	AW_SideInfoSub	sis[MAX_NGRAN][MAX_NCHAN];
} AW_SideInfo;

typedef struct {
	int cbType;		/* pure long = 0, pure short = 1, mixed = 2 */
	int cbEndS[3];	/* number nonzero short cb's, per subbblock */
	int cbEndSMax;	/* max of cbEndS[] */
	int cbEndL;		/* number nonzero long cb's  */
} CriticalBandInfo;

typedef struct _AW_DequantInfo {
	int workBuf[AW_MP3_MAX_REORDER_SAMPS];		/* workbuf for reordering short blocks */
	CriticalBandInfo cbi[MAX_NCHAN];	/* filled in dequantizer, used in joint stereo reconstruction */
} AW_DequantInfo;

typedef struct _AW_HuffmanInfo {
	int huffDecBuf[MAX_NCHAN][MAX_NSAMP];		/* used both for decoded Huffman values and dequantized coefficients */
	int nonZeroBound[MAX_NCHAN];				/* number of coeffs in huffDecBuf[ch] which can be > 0 */
	int gb[MAX_NCHAN];							/* minimum number of guard bits in huffDecBuf[ch] */
} AW_HuffmanInfo;

typedef enum _AW_HuffTabType {
	noBits,
	oneShot,
	loopNoLinbits,
	loopLinbits,
	quadA,
	quadB,
	invalidTab
} AW_HuffTabType;

typedef struct _AW_HuffTabLookup {
	int	linBits;
	AW_HuffTabType tabType;
} AW_HuffTabLookup;

typedef struct _AW_IMDCTInfo {
	int outBuf[MAX_NCHAN][AW_BLOCK_SIZE][AW_MP3_NBANDS];	/* output of IMDCT */
	int overBuf[MAX_NCHAN][MAX_NSAMP / 2];		/* overlap-add buffer (by symmetry, only need 1/2 size) */
	int numPrevIMDCT[MAX_NCHAN];				/* how many IMDCT's calculated in this channel on prev. granule */
	int prevType[MAX_NCHAN];
	int prevWinSwitch[MAX_NCHAN];
	int gb[MAX_NCHAN];
} AW_IMDCTInfo;

typedef struct _BlockCount {
	int nBlocksLong;
	int nBlocksTotal;
	int nBlocksPrev;
	int prevType;
	int prevWinSwitch;
	int currWinSwitch;
	int gbIn;
	int gbOut;
} BlockCount;

/* max bits in scalefactors = 5, so use char's to save space */
typedef struct _AW_ScaleFactorInfoSub {
	char l[23];            /* [band] */
	char s[13][3];         /* [band][window] */
} AW_ScaleFactorInfoSub;

/* used in MPEG 2, 2.5 intensity (joint) stereo only */
typedef struct _ScaleFactorJS {
	int intensityScale;
	int	slen[4];
	int	nr[4];
} ScaleFactorJS;

typedef struct _AW_ScaleFactorInfo {
	AW_ScaleFactorInfoSub sfis[MAX_NGRAN][MAX_NCHAN];
	ScaleFactorJS sfjs;
} AW_ScaleFactorInfo;

/* NOTE - could get by with smaller vbuf if memory is more important than speed
 *  (in Subband, instead of replicating each block in AWFDCT32 you would do a memmove on the
 *   last 15 blocks to shift them down one, a hardware style FIFO)
 */
typedef struct _AW_SubbandInfo {
	int vbuf[MAX_NCHAN * AW_MP3_VBUF_LENGTH];		/* vbuf for fast DCT-based synthesis PQMF - double size for speed (no modulo indexing) */
	int vindex;								/* internal index for tracking position in vbuf */
} AW_SubbandInfo;

/* bitstream.c */
void AwSetBitstreamPointer(AW_BitStreamInfo *bsi, int nBytes, unsigned char *buf);
unsigned int AwGetBits(AW_BitStreamInfo *bsi, int nBits);
int AwCalcBitsUsed(AW_BitStreamInfo *bsi, unsigned char *startBuf, int startOffset);

/* dequant.c, dqchan.c, stproc.c */
int AwDequantChannel(int *sampleBuf, int *workBuf, int *nonZeroBound, AwFrameHeader *fh, AW_SideInfoSub *sis,
                     AW_ScaleFactorInfoSub *sfis, CriticalBandInfo *cbi);
void AwMidSideProc(int x[MAX_NCHAN][MAX_NSAMP], int nSamps, int mOut[2]);
void AwIntensityProcMPEG1(int x[MAX_NCHAN][MAX_NSAMP], int nSamps, AwFrameHeader *fh, AW_ScaleFactorInfoSub *sfis,
                          CriticalBandInfo *cbi, int midSideFlag, int mixFlag, int mOut[2]);
void AwIntensityProcMPEG2(int x[MAX_NCHAN][MAX_NSAMP], int nSamps, AwFrameHeader *fh, AW_ScaleFactorInfoSub *sfis,
                          CriticalBandInfo *cbi, ScaleFactorJS *sfjs, int midSideFlag, int mixFlag, int mOut[2]);

/* dct32.c */
void AWFDCT32(int *x, int *d, int offset, int oddBlock, int gb);

/* hufftabs.c */
extern const AW_HuffTabLookup AWhuffTabLookup[AW_HUFF_PAIRTABS];
extern const int AWhuffTabOffset[AW_HUFF_PAIRTABS];
extern const unsigned short AWhuffTable[];
extern const unsigned char AWquadTable[64+16];
extern const int AWquadTabOffset[2];
extern const int AWquadTabMaxBits[2];

/* polyphase.c (or asmpoly.s)
 * some platforms require a C++ compile of all source files,
 * so if we're compiling C as C++ and using native assembly
 * for these functions we need to prevent C++ name mangling.
 */
#ifdef __cplusplus
extern "C" {
#endif
	void AwPolyphaseMono(short *pcm, int *vbuf, const int *coefBase);
	void AwPolyphaseStereo(short *pcm, int *vbuf, const int *coefBase);
#ifdef __cplusplus
}
#endif

extern const int AWpolyCoef[264];
#endif
#endif	/* ROM_CODER_H */
