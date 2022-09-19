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

#include <stdbool.h>
#include "audio_arch.h"
#include "driver/chip/hal_dma.h"
#include "sys/dma_heap.h"

#include "xradio_internal_codec.h"

#if (CONFIG_CHIP_ARCH_VER == 3)

//Debug config
#define XRADIO_CODEC_DBG_EN                 0
#define XRADIO_CODEC_ERR_EN                 1

#define XRADIO_CODEC_DBG(fmt, arg...)       HAL_LOG(XRADIO_CODEC_DBG_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)
#define XRADIO_CODEC_ERR(fmt, arg...)       HAL_LOG(XRADIO_CODEC_ERR_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)
#define XRADIO_CODEC_ALWAYS(fmt, arg...)    HAL_LOG(1, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)

#define XRADIO_CODEC_IT_ERR(fmt, arg...)    HAL_IT_LOG(XRADIO_CODEC_ERR_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)

//Xradio Codec config
#define XRADIO_CODEC_DEF_UNDERRUN_THRES     3
#define XRADIO_CODEC_DEF_CLD_VOL            VOLUME_LEVEL16

//Interface define
#define XRADIO_CODEC_MALLOC                 HAL_Malloc
#define XRADIO_CODEC_FREE                   HAL_Free
#define XRADIO_CODEC_MEMCPY                 HAL_Memcpy
#define XRADIO_CODEC_MEMSET                 HAL_Memset

//Xradio codec priv struct
struct Xradio_Codec_Priv {
	//codec status contrl
	bool isCodecInit;
	bool isTxInit;
	volatile bool txRunning;

	//buffer control
	uint8_t *txBuf;
	uint8_t *writePointer;
	uint32_t txBufSize;

	//DMA control
	uint8_t *txDmaPointer;
	DMA_Channel txDMAChan;
	DMA_DataWidth tx_data_width;
	volatile uint8_t txHalfCallCount;
	volatile uint8_t txEndCallCount;

	//Semaphore control
	HAL_Semaphore txReady;
	bool isTxSemaphore;

	//misc control
	uint16_t tx_underrun_threshold;
	uint16_t cld_vol;
	uint8_t cld_ptn_sel;
};

static struct Xradio_Codec_Priv *xradio_codec_priv;

//const array define
struct real_val_to_reg_val {
	uint32_t real_val;
	uint32_t reg_val;
};

struct cld_clk_div {
	uint32_t sample_rate;
	uint8_t  clk_div;
};

static const struct cld_clk_div xradio_cld_clk_div[] = {
	{48000, 8},
	{24000, 16},
	{12000, 32},

	{32000, 12},
	{16000, 24},
	{8000,  48},

	{44100, 10},
	{22050, 20},
	{11025, 40},
};

static const struct real_val_to_reg_val xradio_cld_vol_gain[] = {
	{VOLUME_GAIN_MINUS_45dB, 4},
	{VOLUME_GAIN_MINUS_42dB, 8},
	{VOLUME_GAIN_MINUS_39dB, 12},
	{VOLUME_GAIN_MINUS_36dB, 16},
	{VOLUME_GAIN_MINUS_33dB, 20},
	{VOLUME_GAIN_MINUS_30dB, 24},
	{VOLUME_GAIN_MINUS_27dB, 28},
	{VOLUME_GAIN_MINUS_24dB, 32},
	{VOLUME_GAIN_MINUS_21dB, 36},
	{VOLUME_GAIN_MINUS_18dB, 40},
	{VOLUME_GAIN_MINUS_15dB, 44},
	{VOLUME_GAIN_MINUS_12dB, 48},
	{VOLUME_GAIN_MINUS_9dB,  52},
	{VOLUME_GAIN_MINUS_6dB,  56},
	{VOLUME_GAIN_MINUS_3dB,  60},

	{VOLUME_GAIN_0dB,  64},

	{VOLUME_GAIN_3dB,  68},
	{VOLUME_GAIN_6dB,  72},
	{VOLUME_GAIN_9dB,  76},
	{VOLUME_GAIN_12dB, 80},
	{VOLUME_GAIN_15dB, 84},
	{VOLUME_GAIN_18dB, 88},
	{VOLUME_GAIN_21dB, 92},
	{VOLUME_GAIN_24dB, 96},
	{VOLUME_GAIN_27dB, 100},
	{VOLUME_GAIN_30dB, 104},
	{VOLUME_GAIN_33dB, 108},
	{VOLUME_GAIN_36dB, 112},
	{VOLUME_GAIN_39dB, 116},
	{VOLUME_GAIN_42dB, 120},
	{VOLUME_GAIN_45dB, 124},
};

static const struct real_val_to_reg_val xradio_cld_vol_level[] = {
	{VOLUME_LEVEL0, 0},
	{VOLUME_LEVEL1, 4},
	{VOLUME_LEVEL2, 8},
	{VOLUME_LEVEL3, 12},
	{VOLUME_LEVEL4, 16},
	{VOLUME_LEVEL5, 20},
	{VOLUME_LEVEL6, 24},
	{VOLUME_LEVEL7, 28},
	{VOLUME_LEVEL8, 32},
	{VOLUME_LEVEL9, 36},
	{VOLUME_LEVEL10, 40},
	{VOLUME_LEVEL11, 44},
	{VOLUME_LEVEL12, 48},
	{VOLUME_LEVEL13, 52},
	{VOLUME_LEVEL14, 56},
	{VOLUME_LEVEL15, 60},

	{VOLUME_LEVEL16, 64},

	{VOLUME_LEVEL17, 68},
	{VOLUME_LEVEL18, 72},
	{VOLUME_LEVEL19, 76},
	{VOLUME_LEVEL20, 80},
	{VOLUME_LEVEL21, 84},
	{VOLUME_LEVEL22, 88},
	{VOLUME_LEVEL23, 92},
	{VOLUME_LEVEL24, 96},
	{VOLUME_LEVEL25, 100},
	{VOLUME_LEVEL26, 104},
	{VOLUME_LEVEL27, 108},
	{VOLUME_LEVEL28, 112},
	{VOLUME_LEVEL29, 116},
	{VOLUME_LEVEL30, 120},
	{VOLUME_LEVEL31, 124},
};

#if 0
#define xradio_codec_reg_read(reg)                      (HAL_REG_32BIT(reg+CODEC_BASE))
#define xradio_codec_reg_write(reg, val)                (HAL_REG_32BIT(reg+CODEC_BASE) = val)
#define xradio_codec_reg_update_bits(reg, mask, val)    (HAL_MODIFY_REG(HAL_REG_32BIT(reg+CODEC_BASE), mask, val))
#endif

//Base read/write Interface
__nonxip_text
static int xradio_codec_reg_read(uint32_t reg)
{
	return HAL_REG_32BIT(reg+CODEC_BASE);
}

__nonxip_text
static int xradio_codec_reg_write(uint32_t reg, uint32_t val)
{
	HAL_REG_32BIT(reg+CODEC_BASE) = val;

	return HAL_OK;
}

__nonxip_text
static int xradio_codec_reg_update_bits(uint32_t reg, uint32_t mask, uint32_t val)
{
	uint32_t val_old, val_new;

	val_old = xradio_codec_reg_read(reg);
	val_new = (val_old & ~mask) | (val & mask);

	if (val_new != val_old) {
		xradio_codec_reg_write(reg, val_new);
	}

	return HAL_OK;
}


/*************************** XRADIO Codec DMA Control ****************************/
__nonxip_text
static void xradio_codec_dma_trigger(Audio_Stream_Dir dir, bool enable)
{
	uint32_t flags;
	uint8_t doProtection = !HAL_IsISRContext();

	if (dir != PCM_OUT) {
		return;
	}

	if (doProtection) {
		flags = HAL_EnterCriticalSection();
	}

	if (enable) {
		//Flush TX_FIFO
		xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_FIFO_FLUSH_BIT, 0x1<<CLD_FIFO_FLUSH_BIT);
		if (xradio_codec_priv->txDMAChan != DMA_CHANNEL_INVALID) {
			//CLD DRQ Enable
			xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_DRQ_EN_BIT, 0x1<<CLD_DRQ_EN_BIT);
			//DMA Start
			HAL_DMA_Start(xradio_codec_priv->txDMAChan, (uint32_t)xradio_codec_priv->txBuf,
			                (uint32_t)(CLD_TXDATA+CODEC_BASE), xradio_codec_priv->txBufSize);
		}
		xradio_codec_priv->txRunning = true;
	} else {
		//Flush TX_FIFO
		xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_FIFO_FLUSH_BIT, 0x1<<CLD_FIFO_FLUSH_BIT);
		if (xradio_codec_priv->txDMAChan != DMA_CHANNEL_INVALID) {
			//DAC DRQ Disable
			xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_DRQ_EN_BIT, 0x0<<CLD_DRQ_EN_BIT);
			//DMA Stop
			HAL_DMA_Stop(xradio_codec_priv->txDMAChan);
		}
		xradio_codec_priv->txRunning = false;
	}

	if (doProtection) {
		HAL_ExitCriticalSection(flags);
	}
}

__nonxip_text
static int xradio_codec_dma_threshold_check(Audio_Stream_Dir dir)
{
	if (dir == PCM_OUT) {
		if (xradio_codec_priv->txHalfCallCount >= xradio_codec_priv->tx_underrun_threshold ||
		    xradio_codec_priv->txEndCallCount  >= xradio_codec_priv->tx_underrun_threshold) {
			XRADIO_CODEC_IT_ERR("Tx : underrun and stop dma tx...\n");
			xradio_codec_dma_trigger(PCM_OUT, false);
			//xradio_codec_priv->txRunning = false; //has been config in trigger interface
			xradio_codec_priv->writePointer = NULL;
			xradio_codec_priv->txDmaPointer = NULL;
			xradio_codec_priv->txHalfCallCount = 0;
			xradio_codec_priv->txEndCallCount = 0;
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

__nonxip_text
static void xradio_codec_dma_half_callback(void *arg)
{
	if (arg == &xradio_codec_priv->txReady) {
		xradio_codec_priv->txHalfCallCount++;
		if (xradio_codec_priv->isTxSemaphore) {
			xradio_codec_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if (xradio_codec_dma_threshold_check(PCM_OUT))
			return;
		xradio_codec_priv->txDmaPointer = xradio_codec_priv->txBuf + xradio_codec_priv->txBufSize/2;
		if (xradio_codec_priv->tx_underrun_threshold != 256)
			XRADIO_CODEC_MEMSET(xradio_codec_priv->txBuf, 0, xradio_codec_priv->txBufSize/2);
	}
}

__nonxip_text
static void xradio_codec_dma_end_callback(void *arg)
{
	if (arg == &xradio_codec_priv->txReady) {
		xradio_codec_priv->txEndCallCount++;
		if (xradio_codec_priv->isTxSemaphore) {
			xradio_codec_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if (xradio_codec_dma_threshold_check(PCM_OUT))
			return;
		xradio_codec_priv->txDmaPointer = xradio_codec_priv->txBuf;
		if (xradio_codec_priv->tx_underrun_threshold != 256)
			XRADIO_CODEC_MEMSET(xradio_codec_priv->txBuf + xradio_codec_priv->txBufSize/2, 0, xradio_codec_priv->txBufSize/2);
	}
}

static int xradio_codec_dma_init(Audio_Stream_Dir dir, DMA_Channel channel)
{
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ

	DMA_ChannelInitParam dmaParam;
	XRADIO_CODEC_MEMSET(&dmaParam, 0, sizeof(dmaParam));

	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return HAL_ERROR;
	}

	dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(
	                DMA_WORK_MODE_CIRCULAR,
	                DMA_WAIT_CYCLE_2,
	                DMA_BYTE_CNT_MODE_REMAIN,
	                xradio_codec_priv->tx_data_width,
	                DMA_BURST_LEN_1,
	                DMA_ADDR_MODE_FIXED,
	                DMA_PERIPH_AUDIO_CODEC,
	                xradio_codec_priv->tx_data_width,
	                DMA_BURST_LEN_1,
	                DMA_ADDR_MODE_INC,
	                DMA_PERIPH_SRAM
	                );
	dmaParam.halfArg = &(xradio_codec_priv->txReady);
	dmaParam.endArg  = &(xradio_codec_priv->txReady);

	dmaParam.irqType = DMA_IRQ_TYPE_BOTH;
	dmaParam.halfCallback = xradio_codec_dma_half_callback;
	dmaParam.endCallback  = xradio_codec_dma_end_callback;

	return HAL_DMA_Init(channel, &dmaParam);

#else

	XRADIO_CODEC_ERR("DMA don't support Half IRQ, dma init Fail!\n");
	return HAL_ERROR;

#endif
}

/*****************************************************************************/


static void xradio_codec_reset(void)
{
	XRADIO_CODEC_DBG("%s, reset all register to their default value\n", __func__);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
}

static void xradio_codec_hw_common_init(Audio_Stream_Dir dir)
{
	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return;
	}

	//CLD CLK&Digital Enable, CLD module reset de-asserted
	xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x1<<CLD_CLK_EN_BIT | 0x1<<CLD_RST_BIT | 0x1<<CLD_DIG_EN_BIT,
	                                           0x1<<CLD_CLK_EN_BIT | 0x1<<CLD_RST_BIT | 0x1<<CLD_DIG_EN_BIT);

	//CLD Pattern Select
	xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x3<<CLD_PTN_SEL_BIT, xradio_codec_priv->cld_ptn_sel<<CLD_PTN_SEL_BIT);
}

static void xradio_codec_hw_common_deinit(Audio_Stream_Dir dir)
{
	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return;
	}

	//CLD CLK&Digital Enable, CLD module reset de-asserted
	xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x1<<CLD_CLK_EN_BIT | 0x1<<CLD_RST_BIT | 0x1<<CLD_DIG_EN_BIT,
	                                           0x0<<CLD_CLK_EN_BIT | 0x0<<CLD_RST_BIT | 0x0<<CLD_DIG_EN_BIT);
}


static int xradio_dai_set_sysclk(Codec_Sysclk_Src sysclk_src, Codec_Pllclk_Src pllclk_src, uint32_t pll_freq_in, uint32_t sample_rate)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);
	uint8_t i, clk_div;

	for (i = 0; i < HAL_ARRAY_SIZE(xradio_cld_clk_div); i++) {
		if (xradio_cld_clk_div[i].sample_rate == sample_rate) {
			clk_div = xradio_cld_clk_div[i].clk_div/2-1;
			break;
		}
	}

	if (i == HAL_ARRAY_SIZE(xradio_cld_clk_div)) {
		XRADIO_CODEC_ERR("Don't match CLD CLK DIV!");
		return HAL_ERROR;
	}

	HAL_PRCM_SetAudioCkcldParam(clk_div);
	HAL_PRCM_EnableAudioCkcldAud();
	//HAL_PRCM_EnableAudioCkadcAud();

	return HAL_OK;
}

static int xradio_dai_set_fmt(uint32_t fmt)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	return HAL_OK;
}

static int xradio_dai_set_volume(Audio_Device device, uint16_t volume)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);
	uint32_t i, reg_val = 0;
	uint16_t vol_set_flag, vol_set_value, vol_array_size = 0;
	const struct real_val_to_reg_val *xradio_codec_vol = NULL;

	vol_set_flag  = volume & VOLUME_SET_MASK;
	vol_set_value = volume & ~VOLUME_SET_MASK;

	switch (device) {
	case AUDIO_OUT_DEV_SPK:
		if (vol_set_flag == VOLUME_SET_LEVEL) {
			xradio_codec_vol = xradio_cld_vol_level;
			vol_array_size = HAL_ARRAY_SIZE(xradio_cld_vol_level);
		} else if (vol_set_flag == VOLUME_SET_GAIN) {
			xradio_codec_vol = xradio_cld_vol_gain;
			vol_array_size = HAL_ARRAY_SIZE(xradio_cld_vol_gain);
		}

		for (i = 0; i < vol_array_size; i++) {
			if (xradio_codec_vol[i].real_val == vol_set_value) {
				reg_val = xradio_codec_vol[i].reg_val;
				break;
			}
		}
		if (i == vol_array_size) {
			XRADIO_CODEC_ERR("Invalid CLD volume %s: %d!\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
			return HAL_INVALID;
		}

		xradio_codec_priv->cld_vol = volume;
		xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x7F<<CLD_DIG_VOL_BIT, reg_val<<CLD_DIG_VOL_BIT);
		XRADIO_CODEC_ALWAYS("CLD set volume %s-[%d]\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
		break;

	default:
		XRADIO_CODEC_ERR("Invalid Audio Device-[0x%08x]!\n", device);
		return HAL_INVALID;
	}

	return HAL_OK;
}

static int xradio_dai_set_route(Audio_Device device, Audio_Dev_State state)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	return HAL_OK;
}

static int xradio_dai_hw_params(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg)
{
	//uint8_t slot_width;
	uint32_t dma_buf_size;
	uint8_t sample_resolution, dma_data_width;
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return HAL_INVALID;
	}

	/* Codec common init */
	xradio_codec_hw_common_init(dir);

	sample_resolution = pcm_format_to_sampleresolution(pcm_cfg->format);
	//slot_width = sample_resolution<=8 ? 8 : (sample_resolution<=16 ? 16 : 32);
	dma_data_width = sample_resolution <= 8 ? DMA_DATA_WIDTH_8BIT :
	                (sample_resolution <= 16 ? DMA_DATA_WIDTH_16BIT : DMA_DATA_WIDTH_32BIT);
	dma_buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));

	/* CLD sample rate config */
	if (pcm_cfg->rate > 48000 || pcm_cfg->rate < 8000) {
		XRADIO_CODEC_ERR("Invalid play sample rate:%d!\n", pcm_cfg->rate);
		return HAL_INVALID;
	}

	if (pcm_cfg->rate%1000) {
		xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x3<<CLD_MODULATE_MODE_BIT, 0x1<<CLD_MODULATE_MODE_BIT);
	} else {
		xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x3<<CLD_MODULATE_MODE_BIT, 0x0<<CLD_MODULATE_MODE_BIT);
	}

	/* CLD channel nums config */
	if (pcm_cfg->channels <= 0 || pcm_cfg->channels > 2) {
		XRADIO_CODEC_ERR("Invalid play channel nums:%d!\n", pcm_cfg->channels);
		return HAL_INVALID;
	}

	if (pcm_cfg->channels == 1) {
		//TX_FIFO data only transfer to L
		xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_MONO_EN_BIT, 0x1<<CLD_MONO_EN_BIT);
		xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x3<<CLD_TX_MIX_CTRL_BIT, 0x0<<CLD_TX_MIX_CTRL_BIT);
	} else {
		//TX_FIFO data transfer to L&R
		xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_MONO_EN_BIT, 0x0<<CLD_MONO_EN_BIT);
		xradio_codec_reg_update_bits(CLD_DIG_CTRL, 0x3<<CLD_TX_MIX_CTRL_BIT, 0x3<<CLD_TX_MIX_CTRL_BIT);
	}

	/* CLD sample resolution config */
	xradio_codec_reg_update_bits(CLD_FIFO_CTRL, 0x1<<CLD_SAMPLE_RES_BIT, (sample_resolution <= 16 ? 0 : 1)<<CLD_SAMPLE_RES_BIT);

	/* dma tx params config */
	xradio_codec_priv->txBufSize = dma_buf_size;
	xradio_codec_priv->tx_data_width = dma_data_width;


	XRADIO_CODEC_DBG("Sample rate-[%d], channel numbers-[%d], sample resolution-[%d]\n",
	                  pcm_cfg->rate, pcm_cfg->channels, sample_resolution);
	return HAL_OK;
}

static int xradio_dai_hw_free(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return HAL_INVALID;
	}

	xradio_codec_hw_common_deinit(dir);

	xradio_codec_reset();
	HAL_PRCM_DisableAudioCkcldAud();
	//HAL_PRCM_DisableAudioCkadcAud();

	return HAL_OK;
}


static int xradio_codec_ioctl_pcm_write(uint8_t *buf, uint32_t size)
{
	uint8_t  xrun_flag;
	uint8_t *pdata = buf;
	uint32_t write_total = 0;
	uint8_t *write_poiter_cur = NULL;
	uint32_t write_single = xradio_codec_priv->txBufSize/2;

	if (!buf || !size) {
		//XRADIO_CODEC_ERR("Tx play buf|size NULL, buf-[0x%08x], size-[%d]!\n",(uint32_t)buf, size);
		return HAL_INVALID;
	}

	for (; size/write_single; pdata += write_single, write_total += write_single, size -= write_single) {
		if (xradio_codec_priv->txRunning == false) {
			if (!xradio_codec_priv->writePointer) {
				xradio_codec_priv->writePointer = xradio_codec_priv->txBuf;
				if (size >= write_single*2) {
					write_poiter_cur = xradio_codec_priv->txBuf;
					XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);
					pdata += write_single;
					write_total += write_single;
					size -= write_single;
					xradio_codec_priv->writePointer = xradio_codec_priv->txBuf + write_single;
				} else {
					XRADIO_CODEC_MEMSET(xradio_codec_priv->txBuf, 0, write_single);
					xradio_codec_priv->writePointer = xradio_codec_priv->txBuf + write_single;
				}
			}
			write_poiter_cur = xradio_codec_priv->writePointer;
			XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);

			/* trigger play, start DMA */
			xradio_codec_dma_trigger(PCM_OUT, true);
			XRADIO_CODEC_DBG("Tx: play start...\n");

		} else {

			xrun_flag = 0;
			HAL_DisableIRQ();

			/* check DMA state */
			if (xradio_codec_priv->txHalfCallCount && xradio_codec_priv->txEndCallCount) {
				//underrun
				xrun_flag = 1;
				xradio_codec_priv->txHalfCallCount = 0;
				xradio_codec_priv->txEndCallCount  = 0;
			} else if (xradio_codec_priv->txHalfCallCount) {
				//DMA half end
				xradio_codec_priv->txHalfCallCount--;
			} else if (xradio_codec_priv->txEndCallCount) {
				//DMA end
				xradio_codec_priv->txEndCallCount--;
			} else {
				//DMA transporting

				/* wait DMA transport end */
				xradio_codec_priv->isTxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(xradio_codec_priv->txReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				/* check DMA state */
				if (xradio_codec_priv->txHalfCallCount && xradio_codec_priv->txEndCallCount) {
					xrun_flag = 1;
					xradio_codec_priv->txHalfCallCount = 0;
					xradio_codec_priv->txEndCallCount  = 0;
				} else if (xradio_codec_priv->txHalfCallCount) {
					xradio_codec_priv->txHalfCallCount--;
				} else if (xradio_codec_priv->txEndCallCount) {
					xradio_codec_priv->txEndCallCount--;
				}
			}

			/* write data to play  */
			if (xradio_codec_priv->txDmaPointer == xradio_codec_priv->txBuf) {
				write_poiter_cur = xradio_codec_priv->txBuf + write_single;
				xradio_codec_priv->writePointer = xradio_codec_priv->txBuf;
			} else {
				write_poiter_cur = xradio_codec_priv->txBuf;
				xradio_codec_priv->writePointer =  xradio_codec_priv->txBuf + write_single;
			}
			XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);
			HAL_EnableIRQ();

			if (xrun_flag) {
				XRADIO_CODEC_ERR("Tx underrun, (H:%u,F:%u)\n",
				    xradio_codec_priv->txHalfCallCount, xradio_codec_priv->txEndCallCount);
			}
		}
	}

	return write_total;
}


static int xradio_codec_ioctl(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	int ret = HAL_ERROR;
	//XRADIO_CODEC_DBG("--->%s\n", __func__);

	switch (cmd) {
	case CODEC_IOCTL_PCM_WRITE:
		if (cmd_param_len != 2)
			return HAL_INVALID;
		ret = xradio_codec_ioctl_pcm_write((uint8_t *)cmd_param[0], cmd_param[1]);
		break;
	case CODEC_IOCTL_HW_CONFIG:
		if (cmd_param_len != 1)
			return HAL_INVALID;
		xradio_codec_priv->cld_ptn_sel = cmd_param[0] & 0x3;
		XRADIO_CODEC_DBG("cld_ptn_sel: %d\n", xradio_codec_priv->cld_ptn_sel);
		ret = HAL_OK;
		break;
	case CODEC_IOCTL_SW_CONFIG:
		if (cmd_param_len != 1)
			return HAL_INVALID;
		xradio_codec_priv->tx_underrun_threshold = cmd_param[0] & 0xffff;
		XRADIO_CODEC_DBG("tx_underrun_threshold: %d\n",
		                  xradio_codec_priv->tx_underrun_threshold);
		ret = HAL_OK;
		break;
	default:
		XRADIO_CODEC_ERR("Invalid ioctl command!\n");
		return HAL_INVALID;
	}

	return ret;
}


static int xradio_codec_open(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return HAL_INVALID;
	}

	if (xradio_codec_priv->isTxInit == true) {
		XRADIO_CODEC_DBG("Codec play device has opened already\n");
		return HAL_OK;
	}
	if (!xradio_codec_priv->txBufSize) {
		XRADIO_CODEC_ERR("TX DMA buffer size is NULL, please set it through IOCTL cmd before codec open!\n");
		return HAL_ERROR;
	}

	xradio_codec_priv->txDMAChan = DMA_CHANNEL_INVALID;
	xradio_codec_priv->writePointer = NULL;
	xradio_codec_priv->txHalfCallCount = 0;
	xradio_codec_priv->txEndCallCount  = 0;

	/* malloc tx buf for DMA */
	xradio_codec_priv->txBuf = dma_malloc(xradio_codec_priv->txBufSize, DMAHEAP_SRAM);
	if (xradio_codec_priv->txBuf == NULL) {
		XRADIO_CODEC_ERR("Malloc Tx buf for DMA faild!\n");
		xradio_codec_priv->txBufSize = 0;
		return HAL_ERROR;
	}
	XRADIO_CODEC_MEMSET(xradio_codec_priv->txBuf, 0, xradio_codec_priv->txBufSize);

	/* request DMA channel */
	xradio_codec_priv->txDMAChan = HAL_DMA_Request();
	if (xradio_codec_priv->txDMAChan == DMA_CHANNEL_INVALID) {
		XRADIO_CODEC_ERR("Request tx DMA channel faild!\n");
		dma_free(xradio_codec_priv->txBuf, DMAHEAP_SRAM);
		xradio_codec_priv->txBuf = NULL;
		xradio_codec_priv->txBufSize = 0;
		return HAL_ERROR;
	}

	/* init DMA */
	xradio_codec_dma_init(PCM_OUT, xradio_codec_priv->txDMAChan);

	/* init semaphore */
	HAL_SemaphoreInitBinary(&xradio_codec_priv->txReady);

	/*int volume and route*/
	xradio_dai_set_volume(AUDIO_OUT_DEV_SPK, xradio_codec_priv->cld_vol);

	xradio_codec_priv->isTxInit = true;


#if XRADIO_CODEC_DBG_EN
	uint32_t i;
	printf("\nXradio Codec Reg:");
	for (i = 0; i < 20; i += 4) {
		if (!(i%16))
			printf("\n");
		printf("Reg[0x%02x] :0x%08x;  ", i, HAL_REG_32BIT(CODEC_BASE+i));
	}
	printf("\n\n");
#endif

	return HAL_OK;
}

static int xradio_codec_close(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (dir != PCM_OUT) {
		XRADIO_CODEC_ERR("CLD don't support record function!\n");
		return HAL_INVALID;
	}

	if (xradio_codec_priv->isTxInit == false) {
		XRADIO_CODEC_DBG("Codec play device has closed already\n");
		return HAL_OK;
	}

	xradio_codec_dma_trigger(PCM_OUT, false);
	//xradio_codec_priv->txRunning = false; /* has been config in trigger interface */
	xradio_codec_priv->writePointer = NULL;
	xradio_codec_priv->txHalfCallCount = 0;
	xradio_codec_priv->txEndCallCount  = 0;

	/* deinit semaphore */
	HAL_SemaphoreDeinit(&xradio_codec_priv->txReady);

	/* deinit DMA */
	HAL_DMA_DeInit(xradio_codec_priv->txDMAChan);

	/* release DMA channel */
	HAL_DMA_Release(xradio_codec_priv->txDMAChan);
	xradio_codec_priv->txDMAChan = DMA_CHANNEL_INVALID;
	xradio_codec_priv->tx_data_width = 0;

	/* free tx buf for DMA */
	dma_free(xradio_codec_priv->txBuf, DMAHEAP_SRAM);
	xradio_codec_priv->txBuf = NULL;
	xradio_codec_priv->txBufSize = 0;

	xradio_codec_priv->isTxInit = false;

	return HAL_OK;
}

static int xradio_internal_codec_init(void)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (xradio_codec_priv->isCodecInit == true) {
		XRADIO_CODEC_DBG("codec has init already\n");
		return HAL_OK;
	}

	//CODEC module CLK gating & reset & MCLK release
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_AudioCodec_EnableMClock();

	xradio_codec_priv->isCodecInit = true;

	return HAL_OK;
}

static void xradio_internal_codec_deinit(void)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	if (xradio_codec_priv->isCodecInit == false) {
		XRADIO_CODEC_DBG("codec has deinit already\n");
		return;
	}

	//CODEC module CLK gating & reset & MCLK lock
	HAL_CCM_AudioCodec_DisableMClock();
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);

	xradio_codec_priv->isCodecInit = false;
}

/*** codec dai ops ****/
static const struct codec_dai_ops xradio_codec_dai_ops = {
	.set_sysclk = xradio_dai_set_sysclk,
	.set_fmt    = xradio_dai_set_fmt,
	.set_volume = xradio_dai_set_volume,
	.set_route  = xradio_dai_set_route,
	.hw_params  = xradio_dai_hw_params,
	.hw_free    = xradio_dai_hw_free,
};

/*** codec ops ****/
static const struct codec_ops xradio_codec_ops = {
	.open  = xradio_codec_open,
	.close = xradio_codec_close,

	.reg_read  = xradio_codec_reg_read,
	.reg_write = xradio_codec_reg_write,

	.ioctl = xradio_codec_ioctl,
};

/*** codec driver ****/
static struct codec_driver xradio_internal_codec_drv = {
	.name = XRADIO_INTERNAL_CODEC_NAME,
	.codec_attr = XRADIO_CODEC_INTERNAL,

	.init = xradio_internal_codec_init,
	.deinit = xradio_internal_codec_deinit,

	.dai_ops = &xradio_codec_dai_ops,
	.codec_ops = &xradio_codec_ops,
};

HAL_Status xradio_internal_codec_register(void)
{
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	/* Malloc xradio codec priv buffer */
	xradio_codec_priv = (struct Xradio_Codec_Priv *)XRADIO_CODEC_MALLOC(sizeof(struct Xradio_Codec_Priv));
	if (xradio_codec_priv == NULL) {
		XRADIO_CODEC_ERR("Malloc Xradio_Codec_Priv buffer Fail!\n");
		return HAL_ERROR;
	}

	XRADIO_CODEC_MEMSET(xradio_codec_priv, 0, sizeof(struct Xradio_Codec_Priv));
	xradio_codec_priv->cld_vol = XRADIO_CODEC_DEF_CLD_VOL;
	xradio_codec_priv->tx_underrun_threshold = XRADIO_CODEC_DEF_UNDERRUN_THRES;

	/* Codec list add */
	list_add(&xradio_internal_codec_drv.node, &hal_snd_codec_list);

	return HAL_OK;
}

HAL_Status xradio_internal_codec_unregister(void)
{
	struct codec_driver *codec_drv_ptr;
	XRADIO_CODEC_DBG("--->%s\n", __func__);

	/* Check snd codec list empty or not */
	if (list_empty(&hal_snd_codec_list)) {
		XRADIO_CODEC_DBG("Hal snd codec list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get codec to unregister */
	list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node) {
		if (codec_drv_ptr == &xradio_internal_codec_drv) {
			list_del(&xradio_internal_codec_drv.node);
			break;
		}
	}

	/* Free  xradio codec priv buffer */
	if (xradio_codec_priv) {
		XRADIO_CODEC_FREE(xradio_codec_priv);
		xradio_codec_priv = NULL;
	}

	return HAL_OK;
}

#endif

