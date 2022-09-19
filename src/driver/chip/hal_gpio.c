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
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_util.h"

#if ((CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3))
typedef struct {
	GPIO_Port       port;
	GPIO_Pin        pin;
	GPIO_WorkMode   mode;
} GPIO_PinMuxMode;
#endif

#if (CONFIG_CHIP_ARCH_VER == 2)
static const GPIO_PinMuxMode pinmux_spi0[] = {
	{ GPIO_PORT_B, GPIO_PIN_4, GPIOB_P4_F2_SPI0_MOSI },
	{ GPIO_PORT_B, GPIO_PIN_5, GPIOB_P5_F2_SPI0_MISO },
	{ GPIO_PORT_B, GPIO_PIN_6, GPIOB_P6_F2_SPI0_CS0 },
	{ GPIO_PORT_B, GPIO_PIN_7, GPIOB_P7_F2_SPI0_CLK },

	{ GPIO_PORT_B, GPIO_PIN_10, GPIOB_P10_F3_SPI0_MOSI },
	{ GPIO_PORT_B, GPIO_PIN_11, GPIOB_P11_F3_SPI0_MISO },
	{ GPIO_PORT_B, GPIO_PIN_12, GPIOB_P12_F3_SPI0_CS0 },
	{ GPIO_PORT_B, GPIO_PIN_13, GPIOB_P13_F3_SPI0_CLK },
};
#elif (CONFIG_CHIP_ARCH_VER == 3)
static const GPIO_PinMuxMode pinmux_spi0[] = {
	{ GPIO_PORT_A, GPIO_PIN_10, GPIOA_P10_F3_SPI0_MOSI },
	{ GPIO_PORT_A, GPIO_PIN_11, GPIOA_P11_F3_SPI0_MISO },
	{ GPIO_PORT_A, GPIO_PIN_15, GPIOA_P15_F3_SPI0_CS0 },
	{ GPIO_PORT_A, GPIO_PIN_16, GPIOA_P16_F3_SPI0_CLK },

	{ GPIO_PORT_A, GPIO_PIN_19, GPIOA_P19_F5_SPI0_MOSI },
	{ GPIO_PORT_A, GPIO_PIN_20, GPIOA_P20_F5_SPI0_MISO },
	{ GPIO_PORT_A, GPIO_PIN_21, GPIOA_P21_F5_SPI0_CS0 },
	{ GPIO_PORT_A, GPIO_PIN_22, GPIOA_P22_F5_SPI0_CLK },

	{ GPIO_PORT_B, GPIO_PIN_4, GPIOB_P4_F2_SPI0_MOSI },
	{ GPIO_PORT_B, GPIO_PIN_5, GPIOB_P5_F2_SPI0_MISO },
	{ GPIO_PORT_B, GPIO_PIN_6, GPIOB_P6_F2_SPI0_CS0 },
	{ GPIO_PORT_B, GPIO_PIN_7, GPIOB_P7_F2_SPI0_CLK },

	{ GPIO_PORT_B, GPIO_PIN_10, GPIOB_P10_F3_SPI0_MOSI },
	{ GPIO_PORT_B, GPIO_PIN_11, GPIOB_P11_F3_SPI0_MISO },
	{ GPIO_PORT_B, GPIO_PIN_12, GPIOB_P12_F3_SPI0_CS0 },
	{ GPIO_PORT_B, GPIO_PIN_13, GPIOB_P13_F3_SPI0_CLK },
};
#endif

#ifdef CONFIG_ROM
extern GPIO_IRQ_T * const gGpioPortIrq[GPIO_PORT_NUM];
void __HAL_GPIO_GlobalInit(const GPIO_GlobalInitParam *param);

void HAL_GPIO_GlobalInit(const GPIO_GlobalInitParam *param)
{
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_GPIO);
	for (GPIO_Port i = GPIO_PORT_A; i < GPIO_PORT_NUM; i++) {
		if (gGpioPortIrq[i]->IRQ_EN != 0) {
			gGpioPortIrq[i]->IRQ_EN = 0;
		}
	}
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_GPIO);
	__HAL_GPIO_GlobalInit(param);
}

#endif /*CONFIG_ROM*/

#if ((CONFIG_CHIP_ARCH_VER == 2) || (CONFIG_CHIP_ARCH_VER == 3))
void HAL_GPIO_PinMuxRebootReset(void)
{
	int i;
	uint32_t count;
	GPIO_InitParam param;

	/* reset spi pinmux before reboot */
	count = HAL_ARRAY_SIZE(pinmux_spi0);
	for (i = 0; i < count; i++) {
		HAL_GPIO_GetConfig(pinmux_spi0[i].port, pinmux_spi0[i].pin, &param);
		if (param.mode == pinmux_spi0[i].mode) {
			HAL_GPIO_SetMode(pinmux_spi0[i].port, pinmux_spi0[i].pin, GPIOx_Pn_F7_DISABLE);
		}
	}
}
#endif

