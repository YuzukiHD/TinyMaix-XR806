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

#ifndef _AC101_H_
#define _AC101_H_

#ifdef __cplusplus
extern "C" {
#endif

 /*** AC101 Codec Register Define***/

//Chip Reset
#define CHIP_AUDIO_RST      0x00

//PLL Control
#define PLL_CTRL1           0x01
#define PLL_CTRL2           0x02

//System Clock Control
#define SYSCLK_CTRL         0x03
#define MOD_CLK_EN          0x04
#define MOD_RST_CTRL        0x05

//I2S Common Control
#define I2S_SR_CTRL         0x06
#define I2S1_CLK_CTRL       0x10
#define I2S1_SDOUT_CTRL     0x11
#define I2S1_SDIN_CTRL      0x12
#define I2S1_MXR_SRC        0x13
#define I2S1_DIG_VOL_CTRL1  0x14
#define I2S1_DIG_VOL_CTRL2  0x15
#define I2S1_DIG_VOL_CTRL3  0x16
#define I2S1_DIG_VOL_CTRL4  0x17
#define I2S1_MXR_GAIN       0x18

//ADC Digital Control
#define ADC_DIG_CTRL        0x40
#define ADC_DIG_VOL_CTRL    0x41

//HMIC Control
#define HMIC_CTRL1          0x44
#define HMIC_CTRL2          0x45
#define HMIC_STATUS         0x46

//DAC Digital Control
#define DAC_DIG_CTRL        0x48
#define DAC_DIG_VOL_CTRL    0x49
#define DAC_MXR_SRC         0x4c
#define DAC_MXR_GAIN        0x4d

//ADC Analog Control
#define ADC_ANA_CTRL        0x50
#define ADC_MXR_SRC         0x51
#define ADC_PGA_CTRL        0x52

//DAC Analog Control
#define DAC_ANA_CTRL        0x53
#define OUT_MXR_SRC         0x54
#define OUT_MXR_GAIN        0x55

//HPOUT Control
#define HPOUT_CTRL          0x56

//SPKOUT Control
#define SPKOUT_CTRL         0x58

//Extend Control
#define ADDA_TUNE1          0x5a
#define ADDA_TUNE2          0x5b
#define ADDA_TUNE3          0x5c

//ADC DAP Control
#define ADCL_DAP_CTRL       0x82
#define ADCR_DAP_CTRL       0x83

//DAC DRC/HPF Control
#define DAC_DAP_CTRL        0xa0
#define DAC_HPF_COEF_H      0xa1
#define DAC_HPF_COEF_L      0xa2
#define DACL_DRC_ENERGY_H   0xa3
#define DACL_DRC_ENERGY_L   0xa4
#define DACR_DRC_ENERGY_H   0xa5
#define DACR_DRC_ENERGY_L   0xa6
#define DAC_DRC_DECAY_H     0xa7
#define DAC_DRC_DECAY_L     0xa8
#define DAC_DRC_ATTACK_H    0xa9
#define DAC_DRC_ATTACK_L    0xaa
#define DAC_DRC_THRESHOLD_H 0xab
#define DAC_DRC_THRESHOLD_L 0xac
#define DAC_DRC_K_PARAM_H   0xad
#define DAC_DRC_K_PARAM_L   0xae
#define DAC_DRC_OFFSET_H    0xaf
#define DAC_DRC_OFFSET_L    0xb0
#define DAC_DRC_OPTIMUM     0xb1
#define DAC_DRC_EN_CTRL     0xb5

/*** AC101 Codec Register Bit Define***/

//PLL_CTRL1
#define DPLL_DAC_BIAS_BIT           14
#define PLL_POSTDIV_M_BIT           8
#define PLL_CLOSE_LOOP_BIT          6
#define PLL_INT_BIT                 0

//PLL_CTRL2
#define PLL_EN_BIT                  15
#define PLL_LOCK_STA_BIT            14
#define PLL_PREDIV_NI_BIT           4
#define PLL_POSTDIV_NF_BIT          0


//SYSCLK_CTRL
#define PLLCLK_EN_BIT               15
#define PLLCLK_SRC_BIT              12
#define I2S1CLK_EN_BIT              11
#define I2S1CLK_SRC_BIT             8
#define SYSCLK_EN_BIT               3

//MOD_CLK_EN
#define I2S1_MODCLK_EN_BIT          15
#define HPF_AGC_MODCLK_EN_BIT       7
#define HPF_DRC_MODCLK_EN_BIT       6
#define ADC_DIG_MODCLK_EN_BIT       3
#define DAC_DIG_MODCLK_EN_BIT       2

//MOD_RST_CTRL
#define I2S1_MODRST_CTRL_BIT        15
#define HPF_AGC_MODRST_CTRL_BIT     7
#define HPF_DRC_MODRST_CTRL_BIT     6
#define ADC_DIG_MODRST_CTRL_BIT     3
#define DAC_DIG_MODRST_CTRL_BIT     2


//I2S_SR_CTRL
#define I2S1_ADDA_FS_BIT            12

//I2S1_CLK_CTRL
#define I2S1_ROLE_BIT               15
#define I2S1_BCLK_POLARITY_BIT      14
#define I2S1_LRCK_POLARITY_BIT      13
#define I2S1_BCLK_DIV_BIT           9
#define I2S1_LRCKBCLK_RATIO_BIT     6
#define I2S1_SAMPLE_RES_BIT         4
#define I2S1_MOD_BIT                2
#define I2S1_DSP_MOD_BIT            1
#define I2S1_TDM_EN_BIT             0

//I2S1_SDOUT_CTRL
#define I2S1_TX_SLOT0_L_EN_BIT      15
#define I2S1_TX_SLOT0_R_EN_BIT      14
#define I2S1_TX_SLOT1_L_EN_BIT      13
#define I2S1_TX_SLOT1_R_EN_BIT      12
#define I2S1_TX_SLOT0_L_SRC_BIT     10
#define I2S1_TX_SLOT0_R_SRC_BIT     8
#define I2S1_TX_SLOT1_L_SRC_BIT     6
#define I2S1_TX_SLOT1_R_SRC_BIT     4
#define I2S1_TX_PCM_COMPAND_EN_BIT  3
#define I2S1_TX_PCM_COMPAND_MOD_BIT 2
#define I2S1_TX_TDM_SLOTWIDTH_BIT   0

//I2S1_SDIN_CTRL
#define I2S1_RX_SLOT0_L_EN_BIT      15
#define I2S1_RX_SLOT0_R_EN_BIT      14
#define I2S1_RX_SLOT1_L_EN_BIT      13
#define I2S1_RX_SLOT1_R_EN_BIT      12
#define I2S1_RX_SLOT0_L_SRC_BIT     10
#define I2S1_RX_SLOT0_R_SRC_BIT     8
#define I2S1_RX_SLOT1_L_SRC_BIT     6
#define I2S1_RX_SLOT1_R_SRC_BIT     4
#define I2S1_RX_PCM_COMPAND_EN_BIT  3
#define I2S1_RX_PCM_COMPAND_MOD_BIT 2
#define I2S1_LOOPBACK_EN_BIT        0

//I2S1_MXR_SRC
#define I2S1_ADCL0_MXR_SRC_I2S1_DAC0L_BIT   15
#define I2S1_ADCL0_MXR_SRC_ADCL_BIT         13
#define I2S1_ADCR0_MXR_SRC_I2S1_DAC0R_BIT   11
#define I2S1_ADCR0_MXR_SRC_ADCR_BIT         9
#define I2S1_ADCL1_MXR_SRC_ADCL_BIT         6
#define I2S1_ADCR1_MXR_SRC_ADCR_BIT         2

//I2S1_MXR_GAIN
#define I2S1_ADCL0_MXR_GAIN_I2S1_DAC0L_BIT  15
#define I2S1_ADCL0_MXR_GAIN_ADCL_BIT        13
#define I2S1_ADCR0_MXR_GAIN_I2S1_DAC0R_BIT  11
#define I2S1_ADCR0_MXR_GAIN_ADCR_BIT        9
#define I2S1_ADCL1_MXR_GAIN_ADCL_BIT        6
#define I2S1_ADCR1_MXR_GAIN_ADCR_BIT        2

//I2S1_DIG_VOL_CTRL1
#define I2S1_TX_SLOT0_L_DIG_VOL_BIT         8
#define I2S1_TX_SLOT0_R_DIG_VOL_BIT         0

//I2S1_DIG_VOL_CTRL2
#define I2S1_TX_SLOT1_L_DIG_VOL_BIT         8
#define I2S1_TX_SLOT1_R_DIG_VOL_BIT         0

//I2S1_DIG_VOL_CTRL3
#define I2S1_RX_SLOT0_L_DIG_VOL_BIT         8
#define I2S1_RX_SLOT0_R_DIG_VOL_BIT         0

//I2S1_DIG_VOL_CTRL4
#define I2S1_RX_SLOT1_L_DIG_VOL_BIT         8
#define I2S1_RX_SLOT1_R_DIG_VOL_BIT         0

//ADC_DIG_CTRL
#define ADC_DIG_EN_BIT              15
#define ADC_DMIC_SEL_BIT            14
#define ADC_FIR_TAP_SEL_BIT         13
#define ADC_OUT_DELAY_TIME_BIT      2
#define ADC_OUT_DELAY_EN_BIT        1

//ADC_DIG_VOL_CTRL
#define ADC_L_DIG_VOL_BIT           8
#define ADC_R_DIG_VOL_BIT           0

//HMIC_CTRL1
#define HMIC_M_BIT                  12
#define HMIC_N_BIT                  8
#define HMIC_DATA_IRQ_MOD_BIT       7
#define HMIC_TH1_HYSTERESIS_BIT     5
#define HMIC_PULLOUT_IRQ_EN_BIT     4
#define HMIC_PLUGIN_IRQ_EN_BIT      3
#define HMIC_KEYUP_IRQ_EN_BIT       2
#define HMIC_KEYDOWN_IRQ_EN_BIT     1
#define HMIC_DATA_IRQ_EN_BIT        0

//HMIC_CTRL2
#define HMIC_SAMPLE_SEL_BIT         14
#define HMIC_TH2_HYSTERESIS_BIT     13
#define HMIC_TH2_BIT                8
#define HMIC_SF_BIT                 6
#define HMIC_KEYUP_CLEAR_BIT        5
#define HMIC_TH1_BIT                0

//HMIC_STATUS
#define HMIC_DATA_BIT               8
#define HMIC_PULLOUT_PENDING_BIT    4
#define HMIC_PLUGIN_PENDING_BIT     3
#define HMIC_KEYUP_PENDING_BIT      2
#define HMIC_KEYDOWN_PENDING_BIT    1
#define HMIC_DATA_PENDING_BIT       0

//DAC_DIG_CTRL
#define DAC_DIG_EN_BIT              15
#define DAC_HPF_EN_BIT              14
#define DAC_FIR_TAP_SEL_BIT         13
#define DAC_MODQU_LEVEL_SEL_BIT     8

//DAC_DIG_VOL_CTRL
#define DAC_L_DIG_VOL_BIT           8
#define DAC_R_DIG_VOL_BIT           0

//DAC_MXR_SRC
#define DACL_MXR_SRC_I2S1_DAC0L_BIT 15
#define DACL_MXR_SRC_I2S1_DAC1L_BIT 14
#define DACL_MXR_SRC_ADCL_BIT       12
#define DACR_MXR_SRC_I2S1_DAC0R_BIT 11
#define DACR_MXR_SRC_I2S1_DAC1R_BIT 10
#define DACR_MXR_SRC_ADCR_BIT       8

//DAC_MXR_GAIN
#define DACL_MXR_GAIN_I2S1_DAC0L_BIT        15
#define DACL_MXR_GAIN_I2S1_DAC1L_BIT        14
#define DACL_MXR_GAIN_ADCL_BIT              12
#define DACR_MXR_GAIN_I2S1_DAC0R_BIT        11
#define DACR_MXR_GAIN_I2S1_DAC1R_BIT        10
#define DACR_MXR_GAIN_ADCR_BIT              8

//ADC_ANA_CTRL
#define ADC_R_ANA_EN_BIT                    15
#define ADC_R_INPUT_GAIN_BIT                12
#define ADC_L_ANA_EN_BIT                    11
#define ADC_L_INPUT_GAIN_BIT                8
#define ADC_MICBIAS_EN_BIT                  7
#define ADC_MICBIAS_CHOP_EN_BIT             6
#define ADC_MICBIAS_CHOP_CLK_SEL_BIT        4
#define ADC_HBIAS_WORK_MOD_BIT              2
#define ADC_HBIAS_EN_BIT                    1
#define ADC_HBIAS_CUR_SENSOR_ADC_EN_BIT     0

//ADC_MXR_SRC
#define ADCR_MXR_SRC_MIC1_BIT       13
#define ADCR_MXR_SRC_MIC2_BIT       12
#define ADCR_MXR_SRC_LINEIN_LR_BIT  11
#define ADCR_MXR_SRC_LINEIN_R_BIT   10
#define ADCR_MXR_SRC_OUT_MXR_R_BIT  8
#define ADCR_MXR_SRC_OUT_MXR_L_BIT  7
#define ADCL_MXR_SRC_MIC1_BIT       6
#define ADCL_MXR_SRC_MIC2_BIT       5
#define ADCL_MXR_SRC_LINEIN_LR_BIT  4
#define ADCL_MXR_SRC_LINEIN_L_BIT   3
#define ADCL_MXR_SRC_OUT_MXR_L_BIT  1
#define ADCL_MXR_SRC_OUT_MXR_R_BIT  0

//ADC_PGA_CTRL
#define ADC_MIC1_PGA_EN_BIT         15
#define ADC_MIC1_PGA_GAIN_BIT       12
#define ADC_MIC2_PGA_EN_BIT         11
#define ADC_MIC2_PGA_GAIN_BIT       8
#define ADC_MIC2_PIN_EN_BIT         7
#define ADC_LINEIN_LR_PGA_GAIN_BIT  4

//DAC_ANA_CTRL
#define DAC_R_ANA_EN_BIT            15
#define DAC_L_ANA_EN_BIT            14
#define OUT_MXR_R_ANA_EN_BIT        13
#define OUT_MXR_L_ANA_EN_BIT        12
#define HPOUT_DC_OFFSET_BIT         8

//OUT_MXR_SRC
#define OUT_MXR_R_SRC_MIC1_BIT      13
#define OUT_MXR_R_SRC_MIC2_BIT      12
#define OUT_MXR_R_SRC_LINEIN_LR_BIT 11
#define OUT_MXR_R_SRC_LINEIN_R_BIT  10
#define OUT_MXR_R_SRC_DACR_BIT      8
#define OUT_MXR_R_SRC_DACL_BIT      7
#define OUT_MXR_L_SRC_MIC1_BIT      6
#define OUT_MXR_L_SRC_MIC2_BIT      5
#define OUT_MXR_L_SRC_LINEIN_LR_BIT 4
#define OUT_MXR_L_SRC_LINEIN_L_BIT  3
#define OUT_MXR_L_SRC_DACL_BIT      1
#define OUT_MXR_L_SRC_DACR_BIT      0

//OUT_MXR_GAIN
#define HBIAS_VOLTAGE_SEL_BIT               14
#define MBIAS_VOLTAGE_SEL_BIT               12
#define OUT_MXR_MIC1_GAIN_BIT               6
#define OUT_MXR_MIC2_GAIN_BIT               3
#define OUT_MXR_LINEIN_L_OR_R_GAIN_BIT      0

//HPOUT_CTRL
#define HPOUT_PA_R_SRC_SEL_BIT      15
#define HPOUT_PA_L_SRC_SEL_BIT      14
#define HPOUT_PA_R_UNMUTE_BIT       13
#define HPOUT_PA_L_UNMUTE_BIT       12
#define HPOUT_PA_EN_BIT             11
#define HPOUT_VOL_BIT               4
#define HPOUT_DELAY_TIME_BIT        2
#define HPOUT_PA_CURRENT_BIT        0


//SPKOUT_CTRL
#define HPOUT_CALI_CLK_BIT          13
#define SPKOUT_R_SRC_SEL_BIT        12
#define SPKOUT_R_N_PIN_EN_BIT       11
#define SPKOUT_R_EN_BIT             9
#define SPKOUT_L_SRC_SEL_BIT        8
#define SPKOUT_L_N_PIN_EN_BIT       7
#define SPKOUT_L_EN_BIT             5
#define SPKOUT_VOL_BIT              0

//ADDA_TUNE1
#define VOL_ZERO_CROSS_EN_BIT       8

//ADDA_TUNE3
#define INNER_OSC_EN_BIT            0

//ADCL_DAP_CTRL
#define ADCL_HPF_EN_BIT             13

//ADCR_DAP_CTRL
#define ADCR_HPF_EN_BIT             13

//DAC_DAP_CTRL
#define DRC_EN_BIT                  2
#define DACL_HPF_EN_BIT             1
#define DACR_HPF_EN_BIT             0

//DAC_DRC_EN_CTRL
#define I2S1_TX_SLOT0_DRC_EN_BIT    15
#define I2S1_TX_SLOT1_DRC_EN_BIT    13
#define DAC_DRC_EN_BIT              7

/*** Some Config Value ***/

//I2S BCLK POLARITY Control
#define BCLK_NORMAL_DRIVE_N_SAMPLE_P    0
#define BCLK_INVERT_DRIVE_P_SAMPLE_N    1

//I2S LRCK POLARITY Control
#define LRCK_LEFT_LOW_RIGHT_HIGH        0
#define LRCK_LEFT_HIGH_RIGHT_LOW        1

//I2S Format Selection
#define I2S_FORMAT                      0
#define LEFT_JUSTIFIED_FORMAT           1
#define RIGHT_JUSTIFIED_FORMAT          2
#define PCM_FORMAT                      3

//AC101 I2C Address
#define AC101_I2C_ADDR                  0x1A

#ifdef __cplusplus
}
#endif

#endif /* _AC101_H_ */
