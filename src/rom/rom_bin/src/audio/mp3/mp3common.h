#ifndef _ROM_MP3COMMON_H
#define _ROM_MP3COMMON_H
#ifdef CONFIG_ROM
#include "stdio.h"

//#define	 ae_debugdata


#include "mp3_osPlatform.h"
#include "mp3dec.h"
#include "mad.h"


#define MAX_SCFBD		4		/* max scalefactor bands per channel */
#define NUM_GRANS_MPEG1	2
#define NUM_GRANS_MPEG2	1

/* 11-bit syncword if MPEG 2.5 extensions are enabled */
#define	SYNCWORDH		0xff
#define	SYNCWORDL		0xe0

/* 12-bit syncword if MPEG 1,2 only are supported
 * #define	SYNCWORDH		0xff
 * #define	SYNCWORDL		0xf0
 */
typedef struct funcflag23 {
	short FFREV_flag;
	int/*short*/ FFREV_Time;
	short ABSet_flag;
} funcflag2;

typedef struct TimeInfo23 {
	unsigned int totalTimeInSec;
	short Time_hour;
	short Time_minue;
	short Time_second;
	unsigned int FFREVSTOPTime;
} TimeInfo2;


typedef struct Strongframecheck23 {
	short initflag;
	short lay_bs_fschangeflag;
	int bsIndex;
	short lastbytechangflag;
	int copy_orig_emphasis;

	int Rev_modifylength;//for rev
	int	ABMODEbackup[10];

} Strongframecheck2;


typedef  struct _FileReadInfo {
	unsigned char readBuf[READBUF_SIZE+GuardByte];
	unsigned char *readPtr;
	int bytesLeft;

	int lastmovelength;
	FILE *infile;
	int eofReached;
	int nRead;
	int offset;
} FileReadInfo;

typedef struct MP3DecInfo {
	/* pointers to platform-specific data structures */
	void *FrameHeaderPS;
	void *SideInfoPS;
	void *ScaleFactorInfoPS;
	void *HuffmanInfoPS;
	void *DequantInfoPS;
	void *IMDCTInfoPS;
	void *SubbandInfoPS;

	/* buffer which must be large enough to hold largest possible main_data section */
	unsigned char mainBuf[MAINBUF_SIZE];

	/* special info for "free" bitrate files */
	int FlagFreeBitRate;
	int SlotsFreeBitRate;

	/* user-accessible info */
	int bitrate;
	int nChans;
	int samprate;
	int nGrans;				/* granules per frame */
	int nGranSamps;			/* samples per granule */
	int nSlots;
	int layer;
	int layerForInitBuf;
	MPEGVersion version;

	int mainDataBegin;
	int mainDataBytes;

	int part23Length[MAX_NGRAN][MAX_NCHAN];

	unsigned int TFrameLength;

	//add
	void *FileReadInfo;
	Strongframecheck2 Strongframecheck;
	FileReadInfo FReadInfo;
#ifdef ENABLE_MPEG_LAYER_I_II
	aw_mad_stream *stream;
	aw_mad_frame *frame;
	aw_mad_synth *synth;
#endif
} AwMP3DecInfo;

typedef struct _SFBandTable {
	short l[23];
	short s[14];
} SFBandTable;

/* decoder functions which must be implemented for each platform */
int BuffersAllocate(AwMP3DecInfo *mp3infohdl);
void BuffersFree(AwMP3DecInfo *mp3infohdl);
int CheckPadBit(AwMP3DecInfo *mp3infohdl);
int UnpackFrameHeader(AwMP3DecInfo *mp3infohdl, unsigned char *buf);
int UnpackSideInfo(AwMP3DecInfo *mp3infohdl, unsigned char *buf);
int DecodeHuffman(AwMP3DecInfo *mp3infohdl, unsigned char *buf, int *bitOffset, int huffBlockBits, int gr, int ch);
int Dequantize(AwMP3DecInfo *mp3infohdl, int gr);
int IMDCT(AwMP3DecInfo *mp3infohdl, int gr, int ch);
int ExtractScaleFactors(AwMP3DecInfo *mp3infohdl, unsigned char *buf, int *bitOffset, int bitsAvail, int gr, int ch);
int Subband(AwMP3DecInfo *mp3infohdl, short *pcmBuf);

/* mp3tabs.c - global ROM tables */
extern const int samplerateTab[3][3];
extern const short bitrateTab[3][3][15];
extern const short samplesPerFrameTab[3][3];
extern const short bitsPerSlotTab[3];
extern const short sideBytesTab[3][2];
extern const short slotTab[3][3][15];
extern const SFBandTable sfBandTable[3][3];
#endif
#endif	/* _ROM_MP3COMMON_H */
