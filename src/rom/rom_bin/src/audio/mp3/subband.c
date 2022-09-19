#ifdef CONFIG_ROM

#include "coder.h"
#include "mp3_assembly.h"
#include "rom/compiler.h"

/**************************************************************************************
 * Function:    Subband
 *
 * Description: do subband transform on all the blocks in one granule, all channels
 *
 * Inputs:      filled AwMP3DecInfo structure, after calling IMDCT for all channels
 *              vbuf[ch] and vindex[ch] must be preserved between calls
 *
 * Outputs:     decoded PCM data, interleaved LRLRLR... if stereo
 *
 * Return:      0 on success,  -1 if null input pointers
 **************************************************************************************/
#ifdef KEEP_CRITICAL_SECTION_IN_SRAM
__sram_text
#endif
int Subband(AwMP3DecInfo *mp3infohdl, short *pcmBuf)
{
	int b;
	//AW_HuffmanInfo *hi;
	AW_IMDCTInfo *mi;
	AW_SubbandInfo *sbi;

	/* validate pointers */
	if (!mp3infohdl || !mp3infohdl->HuffmanInfoPS || !mp3infohdl->IMDCTInfoPS || !mp3infohdl->SubbandInfoPS)
		return -1;
	//hi = (AW_HuffmanInfo *)mp3infohdl->HuffmanInfoPS;
	mi = (AW_IMDCTInfo *)(mp3infohdl->IMDCTInfoPS);
	sbi = (AW_SubbandInfo*)(mp3infohdl->SubbandInfoPS);

	if (mp3infohdl->nChans == 2) {
		/* stereo */
		for (b = 0; b < AW_BLOCK_SIZE; b++) {
			AWFDCT32(mi->outBuf[0][b], sbi->vbuf + 0*32, sbi->vindex, (b & 0x01), mi->gb[0]);
			AWFDCT32(mi->outBuf[1][b], sbi->vbuf + 1*32, sbi->vindex, (b & 0x01), mi->gb[1]);
			AwPolyphaseStereo(pcmBuf, sbi->vbuf + sbi->vindex + AW_MP3_VBUF_LENGTH * (b & 0x01), AWpolyCoef);
			sbi->vindex = (sbi->vindex - (b & 0x01)) & 7;
			pcmBuf += (2 * AW_MP3_NBANDS);
		}
	} else {
		/* mono */
		for (b = 0; b < AW_BLOCK_SIZE; b++) {
			AWFDCT32(mi->outBuf[0][b], sbi->vbuf + 0*32, sbi->vindex, (b & 0x01), mi->gb[0]);
			AwPolyphaseMono(pcmBuf, sbi->vbuf + sbi->vindex + AW_MP3_VBUF_LENGTH * (b & 0x01), AWpolyCoef);
			sbi->vindex = (sbi->vindex - (b & 0x01)) & 7;
			pcmBuf += AW_MP3_NBANDS;
		}
	}

	return 0;
}
#endif
