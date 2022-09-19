#ifndef _ROM_MP3DEC_H
#define _ROM_MP3DEC_H
#ifdef CONFIG_ROM

#define READBUF_SIZE	2048	//(1024*6)	/* feel free to change this, but keep big enough for >= one frame at high bitrates */
#define GuardByte            8
#define	READLEN			 2048
/* determining MAINBUF_SIZE:
 *   max mainDataBegin = (2^9 - 1) bytes (since 9-bit offset) = 511
 *   max nSlots (concatenated with mainDataBegin bytes from before) = 1440 - 9 - 4 + 1 = 1428
 *   511 + 1428 = 1939, round up to 1940 (4-byte align)
 */

#define MAINBUF_SIZE	1940

#define MAX_NGRAN		2		/* max granules */
#define MAX_NCHAN		2		/* max channels */
#define MAX_NSAMP		576		/* max samples per channel, per granule */

/* map to 0,1,2 to make table indexing easier */
typedef enum {
	MPEG1 =  0,
	MPEG2 =  1,
	MPEG25 = 2
} MPEGVersion;

typedef void *HAWMP3Decoder;

enum {
	ERR_MP3_NONE =                  0,
	ERR_MP3_INDATA_UNDERFLOW =     -1,
	ERR_MP3_MAINDATA_UNDERFLOW =   -2,
	ERR_MP3_FREE_BITRATE_SYNC =    -3,
	ERR_MP3_OUT_OF_MEMORY =	       -4,
	ERR_MP3_NULL_POINTER =         -5,
	ERR_MP3_INVALID_FRAMEHEADER =  -6,
	ERR_MP3_INVALID_SIDEINFO =     -7,
	ERR_MP3_INVALID_SCALEFACT =    -8,
	ERR_MP3_INVALID_HUFFCODES =    -9,
	ERR_MP3_INVALID_DEQUANTIZE =   -10,
	ERR_MP3_INVALID_IMDCT =        -11,
	ERR_MP3_INVALID_SUBBAND =      -12,
	ERR_MP3_INDATA_NOENOUGH  =     -13,
	ERR_MP3_NOT_SUPPORT      =     -14,

	ERR_UNKNOWN =                  -9999
};

typedef struct _MP3FrameInfo {
	int bitrate;
	int nChans;
	int samprate;
	int bitsPerSample;
	int outputSamps;
	int layer;
	int version;
} AwMP3FrameInfo;

/* public API */
HAWMP3Decoder AwMP3InitDecoder(int layer);
void AwMP3FreeDecoder(HAWMP3Decoder hMP3Decoder);
int AwMP3Decode(HAWMP3Decoder hMP3Decoder, unsigned char **inbuf, int *bytesLeft, short *outbuf, int useSize);

void AwMP3GetLastFrameInfo(HAWMP3Decoder hMP3Decoder, AwMP3FrameInfo *mp3FrameInfo);
int AwMP3GetNextFrameInfo(HAWMP3Decoder hMP3Decoder, AwMP3FrameInfo *mp3FrameInfo, unsigned char *buf);
int AwMP3FindSyncWord(unsigned char *buf, int nBytes);

void layerI_IIDecInit(HAWMP3Decoder hMP3Decoder);
int layerI_IIMallocMember(HAWMP3Decoder hMP3Decoder);
void layerI_IIFreeMember(HAWMP3Decoder hMP3Decoder);
#endif
#endif	/* _ROM_MP3DEC_H */
