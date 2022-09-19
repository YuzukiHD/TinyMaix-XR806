/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _XRADIO_INTERNAL_CODEC_H_
#define _XRADIO_INTERNAL_CODEC_H_

#ifdef __cplusplus
extern "C" {
#endif

/*** XRADIO Internal Codec Register Define***/

//CLD Control Register
#define CLD_DIG_CTRL             0x00
#define CLD_FIFO_CTRL            0x04
#define CLD_FIFO_STA             0x08
#define CLD_TXDATA               0x0c
#define CLD_TXCNT                0x10

/*** XRADIO Internal Codec Register Bit Define***/

//CLD_DIG_CTRL
#define CLD_CLK_EN_BIT           31
#define CLD_RST_BIT              30
#define CLD_DIG_EN_BIT           28
#define CLD_MODULATE_MODE_BIT    24
#define CLD_DITHER_EN_BIT        23
#define CLD_DITHER_LEVEL_BIT     21
#define CLD_MUTE_DET_EN_BIT      20
#define CLD_MUTE_DET_TIME_BIT    17
#define CLD_HP_GAIN_BIT          14
#define CLD_PTN_SEL_BIT          12
#define CLD_DVC_ZCD_EN_BIT       11
#define CLD_DIG_VOL_BIT          4
#define CLD_TX_MIX_CTRL_BIT      0

//CLD_FIFO_CTRL
#define CLD_SEND_LAST_BIT        26
#define CLD_FIFO_MODE_BIT        24
#define CLD_DRQ_CLR_CNT_BIT      21
#define CLD_TIFO_TRIG_LEVEL_BIT  7
#define CLD_MONO_EN_BIT          6
#define CLD_SAMPLE_RES_BIT       5
#define CLD_DRQ_EN_BIT           4
#define CLD_IRQ_EN_BIT           3
#define CLD_UNDERRUN_IRQ_EN_BIT  2
#define CLD_OVERRUN_IRQ_EN_BIT   1
#define CLD_FIFO_FLUSH_BIT       0

//CLD_FIFO_STA
#define CLD_TX_EMPTY_BIT         23
#define CLD_TXE_CNT_BIT          8
#define CLD_TXE_INT_BIT          3
#define CLD_TXU_INT_BIT          2
#define CLD_TXO_INT_BIT          1

#ifdef __cplusplus
}
#endif

#endif    //_XRADIO_INTERNAL_CODEC_H_

