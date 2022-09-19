/**
 * @file  hal_gpio.h
 * @author  XRADIO IOT WLAN Team
 */

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

#ifndef _DRIVER_CHIP_HAL_GPIO_H_
#define _DRIVER_CHIP_HAL_GPIO_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO control register block structure
 */
typedef struct {
	__IO uint32_t MODE[4];      /* offset: 0x00, GPIO working mode configuration register */
	__IO uint32_t DATA;         /* offset: 0x10, GPIO data register */
	__IO uint32_t DRIVING[2];   /* offset: 0x14, GPIO driving level register */
	__IO uint32_t PULL[2];      /* offset: 0x1C, GPIO pull type register */
} GPIO_CTRL_T;

/**
 * @brief GPIO port definition
 */
typedef enum {
	GPIO_PORT_A = 0U,
	GPIO_PORT_B = 1U,
#if (CONFIG_CHIP_ARCH_VER == 2)
	GPIO_PORT_C = 2U,
#endif
	GPIO_PORT_NUM
} GPIO_Port;

/* call in irq disable status for protect BitMap */
#define BITMAP_SET_2USER(BitMap, group, n)      HAL_SET_BIT(BitMap[group/32], HAL_BIT((group * 2) + n))
#define BITMAP_CLR_2USER(BitMap, group, n)      HAL_CLR_BIT(BitMap[group/32], HAL_BIT((group * 2) + n))
#define BITMAP_GET_2USER(BitMap, group)         HAL_GET_BIT_VAL(BitMap[group/32], (group * 2), 0x3)

/**
 * @brief GPIO working mode (function) definition
 */
typedef enum {
	GPIOx_Pn_F0_INPUT       = 0U,   /* for all GPIO pins */
	GPIOx_Pn_F1_OUTPUT      = 1U,   /* for all GPIO pins */
	GPIOx_Pn_F6_EINT        = 6U,   /* XR871/XR809: [PA0, PB13]; XR872/XR808: all */
	GPIOx_Pn_F7_DISABLE     = 7U,   /* for all GPIO pins */

#if (CONFIG_CHIP_ARCH_VER == 2)

	GPIOA_P0_F2_SPI1_MOSI   = 2U,
	GPIOA_P0_F3_SD_CMD      = 3U,
	GPIOA_P0_F4_I2C1_SCL    = 4U,
	GPIOA_P0_F5_CSI_D0      = 5U,

	GPIOA_P1_F2_SPI1_MISO   = 2U,
	GPIOA_P1_F3_SD_DATA0    = 3U,
	GPIOA_P1_F4_I2C1_SDA    = 4U,
	GPIOA_P1_F5_CSI_D1      = 5U,

	GPIOA_P2_F2_SPI1_CLK    = 2U,
	GPIOA_P2_F3_SD_CLK      = 3U,
	GPIOA_P2_F4_UART0_TX    = 4U,
	GPIOA_P2_F5_CSI_D2      = 5U,

	GPIOA_P3_F2_SPI1_CS0    = 2U,
	GPIOA_P3_F3_SD_DATA1    = 3U,
	GPIOA_P3_F4_UART0_RX    = 4U,
	GPIOA_P3_F5_CSI_D3      = 5U,

	GPIOA_P4_F2_UART1_RTS   = 2U,
	GPIOA_P4_F3_SD_DATA2    = 3U,
	GPIOA_P4_F4_I2C0_SCL    = 4U,
	GPIOA_P4_F5_CSI_D4      = 5U,

	GPIOA_P5_F2_UART1_CTS   = 2U,
	GPIOA_P5_F3_SD_DATA3    = 3U,
	GPIOA_P5_F4_I2C0_SDA    = 4U,
	GPIOA_P5_F5_CSI_D5      = 5U,

	GPIOA_P6_F2_UART1_RX    = 2U,
	GPIOA_P6_F3_SPI1_CS1    = 3U,
	GPIOA_P6_F4_I2C0_SCL    = 4U,
	GPIOA_P6_F5_CSI_D6      = 5U,

	GPIOA_P7_F2_UART1_TX    = 2U,
	GPIOA_P7_F3_SPI1_CS2    = 3U,
	GPIOA_P7_F4_I2C0_SDA    = 4U,
	GPIOA_P7_F5_CSI_D7      = 5U,

	GPIOA_P8_F2_FEM_CTRL1   = 2U,
	GPIOA_P8_F3_PWM0_ECT0   = 3U,
	GPIOA_P8_F4_I2C1_SCL    = 4U,
	GPIOA_P8_F5_CSI_PCLK    = 5U,

	GPIOA_P9_F2_FEM_CTRL2   = 2U,
	GPIOA_P9_F3_PWM1_ECT1   = 3U,
	GPIOA_P9_F4_I2C1_SDA    = 4U,
	GPIOA_P9_F5_CSI_MCLK    = 5U,

	GPIOA_P10_F2_ADC_CH0    = 2U,
	GPIOA_P10_F3_PWM2_ECT2  = 3U,
	GPIOA_P10_F4_DMIC_CLK   = 4U,
	GPIOA_P10_F5_CSI_HSYNC  = 5U,

	GPIOA_P11_F2_ADC_CH1    = 2U,
	GPIOA_P11_F3_PWM3_ECT3  = 3U,
	GPIOA_P11_F4_DMIC_DATA  = 4U,
	GPIOA_P11_F5_CSI_VSYNC  = 5U,

	GPIOA_P12_F2_ADC_CH2    = 2U,
	GPIOA_P12_F3_PWM4_ECT4  = 3U,
	GPIOA_P12_F4_I2S_MCLK   = 4U,
	GPIOA_P12_F5_IR_TX      = 5U,

	GPIOA_P13_F2_ADC_CH3    = 2U,
	GPIOA_P13_F3_PWM5_ECT5  = 3U,
	GPIOA_P13_F4_I2S_BCLK   = 4U,
	GPIOA_P13_F5_UART1_TX   = 5U,

	GPIOA_P14_F2_ADC_CH4    = 2U,
	GPIOA_P14_F3_PWM6_ECT6  = 3U,
	GPIOA_P14_F4_I2S_DI     = 4U,
	GPIOA_P14_F5_UART1_RX   = 5U,

	GPIOA_P15_F2_ADC_CH5    = 2U,
	GPIOA_P15_F3_PWM7_ECT7  = 3U,
	GPIOA_P15_F4_I2S_DO     = 4U,
	GPIOA_P15_F5_UART1_CTS  = 5U,

	GPIOA_P16_F2_ADC_CH6    = 2U,
	GPIOA_P16_F3_IR_RX      = 3U,
	GPIOA_P16_F4_I2S_LRCLK  = 4U,
	GPIOA_P16_F5_UART1_RTS  = 5U,

	/*GPIOA_P17_F2_ADC_CH7    = 2U,*/
	GPIOA_P17_F2_I2C0_SCL   = 2U,
	GPIOA_P17_F3_IR_RX      = 3U,
	GPIOA_P17_F4_32KOSCO    = 4U,
	GPIOA_P17_F5_FEM_CTRL1  = 5U,

	GPIOA_P18_F2_I2C0_SDA   = 2U,
	GPIOA_P18_F3_IR_TX      = 3U,
	GPIOA_P18_F4_FEM_CTRL2  = 4U,
	GPIOA_P18_F5_IR_RX      = 5U,

	GPIOA_P19_F2_UART2_RTS  = 2U,
	GPIOA_P19_F3_I2C0_SCL   = 3U,
	GPIOA_P19_F4_PWM0_ECT0  = 4U,
	GPIOA_P19_F5_SPI1_MOSI  = 5U,

	GPIOA_P20_F2_UART2_CTS  = 2U,
	GPIOA_P20_F3_I2C0_SDA   = 3U,
	GPIOA_P20_F4_PWM1_ECT1  = 4U,
	GPIOA_P20_F5_SPI1_MISO  = 5U,

	GPIOA_P21_F2_UART2_RX   = 2U,
	GPIOA_P21_F3_DMIC_CLK   = 3U,
	GPIOA_P21_F4_PWM2_ECT2  = 4U,
	GPIOA_P21_F5_SPI1_CLK   = 5U,

	GPIOA_P22_F2_UART2_TX   = 2U,
	GPIOA_P22_F3_DMIC_DATA  = 3U,
	GPIOA_P22_F4_PWM3_ECT3  = 4U,
	GPIOA_P22_F5_SPI1_CS0   = 5U,

	GPIOA_P23_F2_DCXO_PUP_OUT = 2U,
	GPIOA_P23_F3_DPLL_PUP_OUT = 3U,
	GPIOA_P23_F4_FEM_CTRL1  = 4U,
	GPIOA_P23_F5_FEM_CTRL2  = 5U,

	GPIOB_P0_F2_UART0_TX    = 2U,
	GPIOB_P0_F3_JTAG_TMS    = 3U,
	GPIOB_P0_F4_PWM4_ECT4   = 4U,
	GPIOB_P0_F5_SWD_TMS     = 5U,

	GPIOB_P1_F2_UART0_RX    = 2U,
	GPIOB_P1_F3_JTAG_TCK    = 3U,
	GPIOB_P1_F4_PWM5_ECT5   = 4U,
	GPIOB_P1_F5_SWD_TCK     = 5U,

	GPIOB_P2_F2_SWD_TMS     = 2U,
	GPIOB_P2_F3_JTAG_TD0    = 3U,
	GPIOB_P2_F4_PWM6_ECT6   = 4U,
	GPIOB_P2_F5_FLASH_WP    = 5U,

	GPIOB_P3_F2_SWD_TCK     = 2U,
	GPIOB_P3_F3_JTAG_TDI    = 3U,
	GPIOB_P3_F4_PWM7_ECT7   = 4U,
	GPIOB_P3_F5_FLASH_HOLD  = 5U,

	GPIOB_P4_F2_SPI0_MOSI   = 2U,
	GPIOB_P4_F3_SD_CMD      = 3U,
	GPIOB_P4_F4_UART1_TX    = 4U,
	GPIOB_P4_F5_FLASH_MOSI  = 5U,

	GPIOB_P5_F2_SPI0_MISO   = 2U,
	GPIOB_P5_F3_SD_DATA0    = 3U,
	GPIOB_P5_F4_UART1_RX    = 4U,
	GPIOB_P5_F5_FLASH_MISO  = 5U,

	GPIOB_P6_F2_SPI0_CS0    = 2U,
	GPIOB_P6_F3_FEM_CTRL2   = 3U,
	GPIOB_P6_F4_UART1_CTS   = 4U,
	GPIOB_P6_F5_FLASH_CS    = 5U,

	GPIOB_P7_F2_SPI0_CLK    = 2U,
	GPIOB_P7_F3_SD_CLK      = 3U,
	GPIOB_P7_F4_UART1_RTS   = 4U,
	GPIOB_P7_F5_FLASH_CLK   = 5U,

	GPIOB_P8_F2_FLASH_WP    = 2U,

	GPIOB_P9_F2_FLASH_HOLD  = 2U,

	GPIOB_P10_F2_FLASH_MOSI = 2U,
	GPIOB_P10_F3_SPI0_MOSI  = 3U,

	GPIOB_P11_F2_FLASH_MISO = 2U,
	GPIOB_P11_F3_SPI0_MISO  = 3U,

	GPIOB_P12_F2_FLASH_CS   = 2U,
	GPIOB_P12_F3_SPI0_CS0   = 3U,

	GPIOB_P13_F2_FLASH_CLK  = 2U,
	GPIOB_P13_F3_SPI0_CLK   = 3U,

	GPIOB_P14_F2_I2C0_SCL   = 2U,
	GPIOB_P14_F3_I2C1_SCL   = 3U,
	GPIOB_P14_F4_DMIC_CLK   = 4U,

	GPIOB_P15_F2_I2C0_SDA   = 2U,
	GPIOB_P15_F3_I2C1_SDA   = 3U,
	GPIOB_P15_F4_DMIC_DATA  = 4U,
	GPIOB_P15_F5_I2S_MCLK   = 5U,

	GPIOB_P16_F2_SPI1_MOSI  = 2U,
	GPIOB_P16_F3_SD_CMD     = 3U,
	GPIOB_P16_F4_UART2_CTS  = 4U,
	GPIOB_P16_F5_I2S_BCLK   = 5U,

	GPIOB_P17_F2_SPI1_MISO  = 2U,
	GPIOB_P17_F3_SD_DATA0   = 3U,
	GPIOB_P17_F4_UART2_RTS  = 4U,
	GPIOB_P17_F5_I2S_DI     = 5U,

	GPIOB_P18_F2_SPI1_CLK   = 2U,
	GPIOB_P18_F3_SD_CLK     = 3U,
	GPIOB_P18_F4_UART2_TX   = 4U,
	GPIOB_P18_F5_I2S_DO     = 5U,

	GPIOB_P19_F2_SPI1_CS0   = 2U,
	GPIOB_P19_F3_SD_DATA1   = 3U,
	GPIOB_P19_F4_UART2_RX   = 4U,
	GPIOB_P19_F5_I2S_LRCLK  = 5U,

	GPIOB_P20_F2_SPI1_CS1   = 2U,
	GPIOB_P20_F3_SD_DATA2   = 3U,
	GPIOB_P20_F4_I2C0_SCL   = 4U,
	GPIOB_P20_F5_I2C1_SCL   = 5U,

	GPIOB_P21_F2_SPI1_CS2   = 2U,
	GPIOB_P21_F3_SD_DATA3   = 3U,
	GPIOB_P21_F4_I2C0_SDA   = 4U,
	GPIOB_P21_F5_I2C1_SDA   = 5U,

	GPIOC_P0_F2_PSRAM_DM    = 2U,
	GPIOC_P0_F3_PSRAM_DQS   = 3U,

	GPIOC_P1_F2_PSRAM_DQ0   = 2U,
	GPIOC_P1_F3_PSRAM_DQ7   = 3U,

	GPIOC_P2_F2_PSRAM_DQ1   = 2U,
	GPIOC_P2_F3_PSRAM_DQ6   = 3U,

	GPIOC_P3_F2_PSRAM_DQ2   = 2U,
	GPIOC_P3_F3_PSRAM_DQ5   = 3U,

	GPIOC_P4_F2_PSRAM_DQ3   = 2U,
	GPIOC_P4_F3_PSRAM_DQ4   = 3U,
	GPIOC_P4_F4_PSRAM_SIO3  = 4U,

	GPIOC_P5_F2_PSRAM_CE    = 2U,
	GPIOC_P5_F3_PSRAM_CLK   = 3U,
	GPIOC_P5_F4_PSRAM_CLK   = 4U,

	GPIOC_P6_F2_PSRAM_CLK_N = 2U,
	GPIOC_P6_F3_PSRAM_CLK_N = 3U,

	GPIOC_P7_F2_PSRAM_CLK   = 2U,
	GPIOC_P7_F3_PSRAM_CE    = 3U,
	GPIOC_P7_F4_PSRAM_CE    = 4U,

	GPIOC_P8_F2_PSRAM_DQ4   = 2U,
	GPIOC_P8_F3_PSRAM_DQ3   = 3U,

	GPIOC_P9_F2_PSRAM_DQ5   = 2U,
	GPIOC_P9_F3_PSRAM_DQ2   = 3U,
	GPIOC_P9_F4_PSRAM_SIO1  = 4U,

	GPIOC_P10_F2_PSRAM_DQ6  = 2U,
	GPIOC_P10_F3_PSRAM_DQ1  = 3U,
	GPIOC_P10_F4_PSRAM_SIO2 = 4U,

	GPIOC_P11_F2_PSRAM_DQ7  = 2U,
	GPIOC_P11_F3_PSRAM_DQ0  = 3U,
	GPIOC_P11_F4_PSRAM_SIO0 = 4U,

	GPIOC_P12_F2_PSRAM_DQS  = 2U,
	GPIOC_P12_F3_PSRAM_DM   = 3U,

#elif (CONFIG_CHIP_ARCH_VER == 3)

	GPIOA_P0_F2_FEM_CTRL1   = 2U,
	GPIOA_P0_F3_AUDIO_PWMP  = 3U,
	GPIOA_P0_F4_I2C1_SCL    = 4U,
	GPIOA_P0_F5_IR_RX       = 5U,
	GPIOA_P0_F8_KEY_Y0      = 8U,
	GPIOA_P0_F9_PWM5_ECT5   = 9U,

	GPIOA_P1_F2_FEM_CTRL2   = 2U,
	GPIOA_P1_F3_AUDIO_PWMN  = 3U,
	GPIOA_P1_F4_I2C1_SDA    = 4U,
	GPIOA_P1_F5_FLASH_CS1   = 5U,
	GPIOA_P1_F8_KEY_Y1      = 8U,
	GPIOA_P1_F9_PWM6_ECT6   = 9U,

	GPIOA_P10_F2_ADC_CH0    = 2U,
	GPIOA_P10_F3_SPI0_MOSI  = 3U,
	GPIOA_P10_F5_UART1_RX   = 5U,
	GPIOA_P10_F8_KEY_Y2     = 8U,
	GPIOA_P10_F9_IR_TX      = 9U,

	GPIOA_P11_F2_ADC_CH1    = 2U,
	GPIOA_P11_F3_SPI0_MISO  = 3U,
	GPIOA_P11_F4_I2S_MCLK   = 4U,
	GPIOA_P11_F5_UART1_TX   = 5U,
	GPIOA_P11_F8_KEY_Y3     = 8U,
	GPIOA_P11_F9_IR_RX      = 9U,

	GPIOA_P12_F2_ADC_CH2    = 2U,
	GPIOA_P12_F3_PWM4_ECT4  = 3U,
	GPIOA_P12_F4_I2S_BCLK   = 4U,
	GPIOA_P12_F5_IR_TX      = 5U,
	GPIOA_P12_F8_KEY_Y4     = 8U,
	GPIOA_P12_F9_I2C0_SCL   = 9U,

	GPIOA_P13_F2_ADC_CH3    = 2U,
	GPIOA_P13_F3_PWM5_ECT5  = 3U,
	GPIOA_P13_F4_I2S_DI     = 4U,
	GPIOA_P13_F5_UART2_TX   = 5U,
	GPIOA_P13_F8_KEY_Y5     = 8U,
	GPIOA_P13_F9_I2C0_SDA   = 9U,

	GPIOA_P14_F2_ADC_CH4    = 2U,
	GPIOA_P14_F3_PWM6_ECT6  = 3U,
	GPIOA_P14_F4_I2S_DO     = 4U,
	GPIOA_P14_F5_UART2_RX   = 5U,
	GPIOA_P14_F8_KEY_Y6     = 8U,
	GPIOA_P14_F10_I2S_MCLK  = 10U,

	GPIOA_P15_F2_ADC_CH5    = 2U,
	GPIOA_P15_F3_SPI0_CS0   = 3U,
	GPIOA_P15_F4_I2S_LRCLK  = 4U,
	GPIOA_P15_F5_UART2_CTS  = 5U,
	GPIOA_P15_F8_KEY_Y7     = 8U,
	GPIOA_P15_F9_I2C1_SCL   = 9U,
	GPIOA_P15_F10_I2S_BCLK  = 10U,

	GPIOA_P16_F2_ADC_CH6    = 2U,
	GPIOA_P16_F3_SPI0_CLK   = 3U,
	GPIOA_P16_F5_UART2_RTS  = 5U,
	GPIOA_P16_F8_KEY_X0     = 8U,
	GPIOA_P16_F9_I2C1_SDA   = 9U,
	GPIOA_P16_F10_I2S_DI    = 10U,

	GPIOA_P17_F2_I2C0_SCL   = 2U,
	GPIOA_P17_F3_AUDIO_PWMP = 3U,
	GPIOA_P17_F4_32KOSCO    = 4U,
	GPIOA_P17_F5_IR_TX      = 5U,
	GPIOA_P17_F8_KEY_X1     = 8U,
	GPIOA_P17_F10_I2S_DO    = 10U,

	GPIOA_P18_F2_I2C0_SDA   = 2U,
	GPIOA_P18_F3_AUDIO_PWMN = 3U,
	GPIOA_P18_F4_FEM_CTRL2  = 4U,
	GPIOA_P18_F5_FLASH_CS1  = 5U,
	GPIOA_P18_F8_KEY_X2     = 8U,
	GPIOA_P18_F10_I2S_LRCLK = 10U,

	GPIOA_P19_F2_UART2_RTS  = 2U,
	GPIOA_P19_F3_CARD_DATA  = 3U,
	GPIOA_P19_F4_PWM0_ECT0  = 4U,
	GPIOA_P19_F5_SPI0_MOSI  = 5U,
	GPIOA_P19_F8_KEY_X3     = 8U,
	GPIOA_P19_F9_AUDIO_PWMP = 9U,

	GPIOA_P20_F2_UART2_CTS  = 2U,
	GPIOA_P20_F3_CARD_CLK   = 3U,
	GPIOA_P20_F4_PWM1_ECT1  = 4U,
	GPIOA_P20_F5_SPI0_MISO  = 5U,
	GPIOA_P20_F8_KEY_X4     = 8U,
	GPIOA_P20_F9_AUDIO_PWMN = 9U,

	GPIOA_P21_F2_UART2_RX   = 2U,
	GPIOA_P21_F3_CARD_RST   = 3U,
	GPIOA_P21_F4_PWM2_ECT2  = 4U,
	GPIOA_P21_F5_SPI0_CS0   = 5U,
	GPIOA_P21_F8_KEY_X5     = 8U,
	GPIOA_P21_F9_I2S_DO     = 9U,

	GPIOA_P22_F2_UART2_TX   = 2U,
	GPIOA_P22_F3_CARD_DETECT = 3U,
	GPIOA_P22_F4_PWM3_ECT3  = 4U,
	GPIOA_P22_F5_SPI0_CLK   = 5U,
	GPIOA_P22_F8_KEY_X6     = 8U,
	GPIOA_P22_F9_I2S_LRCLK  = 9U,

	GPIOA_P23_F2_DCXO_PUP_OUT = 2U,
	GPIOA_P23_F3_IR_RX      = 3U,
	GPIOA_P23_F4_FEM_CTRL1  = 4U,
	GPIOA_P23_F5_FEM_CTRL2  = 5U,
	GPIOA_P23_F8_KEY_X7     = 8U,
	GPIOA_P23_F9_I2S_MCLK   = 9U,

	GPIOB_P0_F2_UART0_TX    = 2U,
	GPIOB_P0_F3_JTAG_TMS    = 3U,
	GPIOB_P0_F4_PWM4_ECT4   = 4U,
	GPIOB_P0_F5_SWD_TMS     = 5U,
	GPIOB_P0_F8_KEY_Y8      = 8U,

	GPIOB_P1_F2_UART0_RX    = 2U,
	GPIOB_P1_F3_JTAG_TCK    = 3U,
	GPIOB_P1_F4_PWM5_ECT5   = 4U,
	GPIOB_P1_F5_SWD_TCK     = 5U,
	GPIOB_P1_F8_KEY_Y9      = 8U,

	GPIOB_P2_F2_UART0_CTS   = 2U,
	GPIOB_P2_F3_JTAG_TD0    = 3U,
	GPIOB_P2_F4_PWM6_ECT6   = 4U,
	GPIOB_P2_F5_FLASH_WP    = 5U, /* IO2 */
	GPIOB_P2_F8_KEY_Y10     = 8U,
	GPIOB_P2_F9_SWD_TMS     = 9U,

	GPIOB_P3_F2_UART0_RTS   = 2U,
	GPIOB_P3_F3_JTAG_TDI    = 3U,
	GPIOB_P3_F4_PWM7_ECT7   = 4U,
	GPIOB_P3_F5_FLASH_HOLD  = 5U, /* IO3 */
	GPIOB_P3_F8_KEY_Y11     = 8U,
	GPIOB_P3_F9_SWD_TCK     = 9U,

	GPIOB_P4_F2_SPI0_MOSI   = 2U,
	GPIOB_P4_F3_PWM0_ECT0   = 3U,
	GPIOB_P4_F4_UART1_RTS   = 4U,
	GPIOB_P4_F5_FLASH_MOSI  = 5U, /* IO0 */
	GPIOB_P4_F8_KEY_Y12     = 8U,
	GPIOB_P4_F9_I2S_BCLK    = 9U,

	GPIOB_P5_F2_SPI0_MISO   = 2U,
	GPIOB_P5_F3_PWM1_ECT1   = 3U,
	GPIOB_P5_F4_UART1_CTS   = 4U,
	GPIOB_P5_F5_FLASH_MISO  = 5U, /* IO1 */
	GPIOB_P5_F8_KEY_Y13     = 8U,
	GPIOB_P5_F9_I2S_DI      = 9U,

	GPIOB_P6_F2_SPI0_CS0    = 2U,
	GPIOB_P6_F3_PWM2_ECT2   = 3U,
	GPIOB_P6_F4_UART1_RX    = 4U,
	GPIOB_P6_F5_FLASH_CS0   = 5U,
	GPIOB_P6_F8_KEY_Y14     = 8U,
	GPIOB_P6_F9_I2S_DO      = 9U,

	GPIOB_P7_F2_SPI0_CLK    = 2U,
	GPIOB_P7_F3_PWM3_ECT3   = 3U,
	GPIOB_P7_F4_UART1_TX    = 4U,
	GPIOB_P7_F5_FLASH_CLK   = 5U,
	GPIOB_P7_F8_KEY_Y15     = 8U,
	GPIOB_P7_F9_I2S_LRCLK   = 9U,

	GPIOB_P8_F2_FLASH_WP    = 2U, /* IO2 */

	GPIOB_P9_F2_FLASH_HOLD  = 2U, /* IO3 */

	GPIOB_P10_F2_FLASH_MOSI = 2U, /* IO0 */
	GPIOB_P10_F3_SPI0_MOSI  = 3U,

	GPIOB_P11_F2_FLASH_MISO = 2U, /* IO1 */
	GPIOB_P11_F3_SPI0_MISO  = 3U,

	GPIOB_P12_F2_FLASH_CS1  = 2U,
	GPIOB_P12_F3_SPI0_CS0   = 3U,

	GPIOB_P13_F2_FLASH_CLK  = 2U,
	GPIOB_P13_F3_SPI0_CLK   = 3U,

	GPIOB_P14_F2_UART1_TX   = 2U,
	GPIOB_P14_F3_UART2_TX   = 3U,
	GPIOB_P14_F4_I2C1_SCL   = 4U,
	GPIOB_P14_F5_UART0_CTS  = 5U,
	GPIOB_P14_F8_KEY_Y0     = 8U,
	GPIOB_P14_F9_PWM5_ECT5  = 9U,

	GPIOB_P15_F2_UART1_RX   = 2U,
	GPIOB_P15_F3_UART2_RX   = 3U,
	GPIOB_P15_F4_I2C1_SDA   = 4U,
	GPIOB_P15_F5_UART0_RTS  = 5U,
	GPIOB_P15_F8_KEY_Y1     = 8U,
	GPIOB_P15_F9_PWM6_ECT6  = 9U,
#endif /* CONFIG_CHIP_XXX */
} GPIO_WorkMode;

/**
 * @brief GPIO driving level definition
 */
typedef enum {
	GPIO_DRIVING_LEVEL_0 = 0U,
	GPIO_DRIVING_LEVEL_1 = 1U,
	GPIO_DRIVING_LEVEL_2 = 2U,
	GPIO_DRIVING_LEVEL_3 = 3U
} GPIO_DrivingLevel;

/**
 * @brief GPIO pull type definition
 */
typedef enum {
	GPIO_PULL_NONE = 0U,
	GPIO_PULL_UP   = 1U,
	GPIO_PULL_DOWN = 2U
} GPIO_PullType;

/**
 * @brief Register bits of GPIO_CTRL_T for each GPIO pin
 */
#define GPIO_CTRL_MODE_BITS     4
#define GPIO_CTRL_MODE_VMASK    0xFU
#define GPIO_CTRL_MODE_MAX      GPIOx_Pn_F7_DISABLE

#define GPIO_CTRL_DATA_BITS     1

#define GPIO_CTRL_DRIVING_BITS  2
#define GPIO_CTRL_DRIVING_VMASK 0x3U
#define GPIO_CTRL_DRIVING_MAX   GPIO_DRIVING_LEVEL_3

#define GPIO_CTRL_PULL_BITS     2
#define GPIO_CTRL_PULL_VMASK    0x3U
#define GPIO_CTRL_PULL_MAX      GPIO_PULL_DOWN

/**
 * @brief GPIO interrupt register block structure
 */
typedef struct {
	__IO uint32_t IRQ_MODE[4];  /* offset: 0x00, GPIO interrupt configuration register */
	__IO uint32_t IRQ_EN;       /* offset: 0x10, GPIO interrupt enable register */
	__IO uint32_t IRQ_STATUS;   /* offset: 0x14, GPIO interrupt status register */
	__IO uint32_t IRQ_DEBOUNCE; /* offset: 0x18, GPIO interrupt debounce register */
} GPIO_IRQ_T;

/**
 * @brief GPIO interrupt trigger event definition
 */
typedef enum {
	GPIO_IRQ_EVT_RISING_EDGE  = 0U,
	GPIO_IRQ_EVT_FALLING_EDGE = 1U,
	GPIO_IRQ_EVT_HIGH_LEVEL   = 2U,
	GPIO_IRQ_EVT_LOW_LEVEL    = 3U,
	GPIO_IRQ_EVT_BOTH_EDGE    = 4U
} GPIO_IrqEvent;

/**
 * @brief Register bits of GPIO_IRQ_T for each GPIO pin
 */
#define GPIO_IRQ_EVT_BITS       4
#define GPIO_IRQ_EVT_VMASK      0xFU
#define GPIO_IRQ_EVT_MAX        GPIO_IRQ_EVT_BOTH_EDGE

#define GPIO_IRQ_EN_BITS        1

#define GPIO_IRQ_STAUTS_BITS    1

/* GPIO interrupt debounce clock prescaler */
#define GPIO_IRQ_DEB_CLK_PRESCALER_SHIFT    4   /* R/W */
#define GPIO_IRQ_DEB_CLK_PRESCALER_VMASK    0x7U
typedef enum {
	GPIO_IRQ_DEB_CLK_PRESCALER_1   = 0U,
	GPIO_IRQ_DEB_CLK_PRESCALER_2   = 1U,
	GPIO_IRQ_DEB_CLK_PRESCALER_4   = 2U,
	GPIO_IRQ_DEB_CLK_PRESCALER_8   = 3U,
	GPIO_IRQ_DEB_CLK_PRESCALER_16  = 4U,
	GPIO_IRQ_DEB_CLK_PRESCALER_32  = 5U,
	GPIO_IRQ_DEB_CLK_PRESCALER_64  = 6U,
	GPIO_IRQ_DEB_CLK_PRESCALER_128 = 7U
} GPIO_IrqDebClkPrescaler;

/* GPIO interrupt debounce clock source */
#define GPIO_IRQ_DEB_CLK_SRC_SHIFT  0   /* R/W */
#define GPIO_IRQ_DEB_CLK_SRC_VMASK  0x1U
typedef enum {
	GPIO_IRQ_DEB_CLK_SRC_LFCLK = 0U,
	GPIO_IRQ_DEB_CLK_SRC_HFCLK = 1U
} GPIO_IrqDebClkSrc;

/******************************************************************************/

/**
 * @brief GPIO pin state definition
 */
typedef enum {
	GPIO_PIN_LOW  = 0,
	GPIO_PIN_HIGH = 1,
} GPIO_PinState;

/**
 * @brief GPIO pin number definition
 */
typedef enum {
	GPIO_PIN_0  = 0U,
	GPIO_PIN_1  = 1U,
	GPIO_PIN_2  = 2U,
	GPIO_PIN_3  = 3U,
	GPIO_PIN_4  = 4U,
	GPIO_PIN_5  = 5U,
	GPIO_PIN_6  = 6U,
	GPIO_PIN_7  = 7U,
	GPIO_PIN_8  = 8U,
	GPIO_PIN_9  = 9U,
	GPIO_PIN_10 = 10U,
	GPIO_PIN_11 = 11U,
	GPIO_PIN_12 = 12U,
	GPIO_PIN_13 = 13U,
	GPIO_PIN_14 = 14U,
	GPIO_PIN_15 = 15U,
	GPIO_PIN_16 = 16U,
	GPIO_PIN_17 = 17U,
	GPIO_PIN_18 = 18U,
	GPIO_PIN_19 = 19U,
	GPIO_PIN_20 = 20U,
	GPIO_PIN_21 = 21U,
	GPIO_PIN_22 = 22U,
	GPIO_PIN_23 = 23U,
} GPIO_Pin;

#if (CONFIG_CHIP_ARCH_VER == 2)
#define GPIOA_PIN_NUM    24
#define GPIOB_PIN_NUM    22
#define GPIOC_PIN_NUM    13
#elif (CONFIG_CHIP_ARCH_VER == 3)
#define GPIOA_PIN_NUM    24
#define GPIOB_PIN_NUM    16
#endif

/**
 * @brief Wakeup I/O of GPIO port A definition
 */
typedef enum {
#if (CONFIG_CHIP_ARCH_VER == 2)
	WAKEUP_IO0 = GPIO_PIN_4,
	WAKEUP_IO1 = GPIO_PIN_5,
	WAKEUP_IO2 = GPIO_PIN_6,
	WAKEUP_IO3 = GPIO_PIN_7,
	WAKEUP_IO4 = GPIO_PIN_17,
	WAKEUP_IO5 = GPIO_PIN_19,
	WAKEUP_IO6 = GPIO_PIN_20,
	WAKEUP_IO7 = GPIO_PIN_21,
	WAKEUP_IO8 = GPIO_PIN_22,
	WAKEUP_IO9 = GPIO_PIN_23,
#elif (CONFIG_CHIP_ARCH_VER == 3)
	WAKEUP_IO0 = GPIO_PIN_10,
	WAKEUP_IO1 = GPIO_PIN_11,
	WAKEUP_IO2 = GPIO_PIN_12,
	WAKEUP_IO3 = GPIO_PIN_13,
	WAKEUP_IO4 = GPIO_PIN_14,
	WAKEUP_IO5 = GPIO_PIN_15,
	WAKEUP_IO6 = GPIO_PIN_16,
	WAKEUP_IO7 = GPIO_PIN_17,
	WAKEUP_IO8 = GPIO_PIN_18,
	WAKEUP_IO9 = GPIO_PIN_19,
	WAKEUP_IO10 = GPIO_PIN_20,
	WAKEUP_IO11 = GPIO_PIN_21,
	WAKEUP_IO12 = GPIO_PIN_22,
	WAKEUP_IO13 = GPIO_PIN_23,
#endif
} WAKEUP_IO;


/**
 * @brief GPIO global initialization parameters
 */
typedef struct {
	uint8_t portIRQUsed;     /* one bit for one port */
	uint8_t portPmBackup;    /* one bit for one port */
} GPIO_GlobalInitParam;

/**
 * @brief GPIO initialization parameters
 */
typedef struct {
	GPIO_WorkMode       mode;
	GPIO_DrivingLevel   driving;
	GPIO_PullType       pull;
} GPIO_InitParam;

/**
 * @brief GPIO pinmux configuration parameters
 */
typedef struct {
	GPIO_Port       port;
	GPIO_Pin        pin;
	GPIO_InitParam  config;
} GPIO_PinMuxParam;

/** @brief Type define of GPIO IRQ callback function */
typedef void (*GPIO_IRQCallback) (void *arg);

/**
 * @brief GPIO interrupt enable parameters
 */
typedef struct {
	GPIO_IrqEvent       event;
	GPIO_IRQCallback    callback;
	void                *arg;
} GPIO_IrqParam;

/**
 * @brief GPIO interrupt debounce parameters
 *
 * @note An interrupt event is detected by 4 clock cycles
 *     - clock = clkSrc / clkPrescaler
 */
typedef struct {
	GPIO_IrqDebClkSrc       clkSrc;
	GPIO_IrqDebClkPrescaler clkPrescaler;
} GPIO_IrqDebParam;

void HAL_GPIO_GlobalInit(const GPIO_GlobalInitParam *param);

/**
 * @brief Initialize the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] param Pointer to GPIO_InitParam structure
 * @return None
 */
void HAL_GPIO_Init(GPIO_Port port, GPIO_Pin pin, const GPIO_InitParam *param);

/**
 * @brief Deinitialize the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @return None
 * @note After deinitialization, the GPIO is in its reset state:
 *       (GPIOx_Pn_F7_DISABLE, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE).
 */
void HAL_GPIO_DeInit(GPIO_Port port, GPIO_Pin pin);

/**
 * @brief Get the configuration of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[out] param Pointer to GPIO_InitParam structure
 * @return None
 */
void HAL_GPIO_GetConfig(GPIO_Port port, GPIO_Pin pin, GPIO_InitParam *param);

/**
 * @brief Set work mode of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] mode work mode of the specified GPIO
 * @return None
 */
void HAL_GPIO_SetMode(GPIO_Port port, GPIO_Pin pin, GPIO_WorkMode mode);

/**
 * @brief Set driving level of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] driving driving level of the specified GPIO
 * @return None
 */
void HAL_GPIO_SetDriving(GPIO_Port port, GPIO_Pin pin, GPIO_DrivingLevel driving);

/**
 * @brief Set pull type of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] pull pull type of the specified GPIO
 * @return None
 */
void HAL_GPIO_SetPull(GPIO_Port port, GPIO_Pin pin, GPIO_PullType pull);

/**
 * @brief Set the state of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] state GPIO pin state
 * @return None
 */
void HAL_GPIO_WritePin(GPIO_Port port, GPIO_Pin pin, GPIO_PinState state);

/**
 * @brief Get the state of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @return GPIO pin state
 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_Port port, GPIO_Pin pin);

/**
 * @brief Set the state of the specified GPIO port
 * @param[in] port GPIO port
 * @param[in] portMask GPIO port state, bit mask of all pins
 * @return None
 */
void HAL_GPIO_WritePort(GPIO_Port port, uint32_t portMask);

/**
 * @brief Get the state of the specified GPIO port
 * @param[in] port GPIO port
 * @return GPIO port state, bit mask of all pins
 */
uint32_t HAL_GPIO_ReadPort(GPIO_Port port);

/**
 * @brief Enable the IRQ of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] param Pointer to GPIO_IrqParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_GPIO_EnableIRQ(GPIO_Port port, GPIO_Pin pin, const GPIO_IrqParam *param);

/**
 * @brief Disable the IRQ of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_GPIO_DisableIRQ(GPIO_Port port, GPIO_Pin pin);

/**
 * @brief Mask (Disable) the IRQ of the specified GPIO temporarily
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] isClearPending
 *     @arg !0 Clear IRQ pending
 *     @arg  0 Not clear IRQ pending
 * @return None
 */
void HAL_GPIO_MaskIRQ(GPIO_Port port, GPIO_Pin pin, int8_t isClearPending);

/**
 * @brief Unmask (Enable) the IRQ of the specified GPIO temporarily
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] isClearPending
 *     @arg !0 Clear IRQ pending
 *     @arg  0 Not clear IRQ pending
 * @return None
 */
void HAL_GPIO_UnMaskIRQ(GPIO_Port port, GPIO_Pin pin, int8_t isClearPending);

/**
 * @brief Set IRQ trigger event of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] event IRQ trigger event of the specified GPIO
 * @return None
 */
void HAL_GPIO_SetIRQEvent(GPIO_Port port, GPIO_Pin pin, GPIO_IrqEvent event);

/**
 * @brief Set debounce parameters of the specified GPIO port
 * @param[in] port GPIO port
 * @param[in] param Pointer to GPIO_IrqDebParam structure
 * @return None
 *
 * @note The debounce parameters are for all pins of the specified GPIO port
 */
void HAL_GPIO_SetIRQDebounce(GPIO_Port port, const GPIO_IrqDebParam *param);

/**
 * @brief Configure the GPIOs pinmux by the specified parameters
 * @param[in] param Pointer to the array of GPIO_PinMuxParam structure, one
 *                  array element for one GPIO pinmux
 * @param[in] count Elements number of the GPIO pinmux parameters array
 * @return None
 */
void HAL_GPIO_PinMuxConfig(const GPIO_PinMuxParam *param, uint32_t count);

/**
 * @brief Deconfigure the GPIOs pinmux by the specified parameters
 * @param[in] param Pointer to the array of GPIO_PinMuxParam structure, one
 *                  array element for one GPIO pinmux, param->config is ignored.
 * @param[in] count Elements number of the GPIO pinmux parameters array
 * @return None
 */
void HAL_GPIO_PinMuxDeConfig(const GPIO_PinMuxParam *param, uint32_t count);

#if ((CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3))
/**
 * @brief reset some pinmux to default value before reboot
 * @return None
 */
void HAL_GPIO_PinMuxRebootReset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_GPIO_H_ */
