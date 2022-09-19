/**
 * @file    hal_csi.h
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

#if (CONFIG_CHIP_ARCH_VER == 1)

#ifndef _DRIVER_CHIP_HAL_CSI_H_
#define _DRIVER_CHIP_HAL_CSI_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The register for CSI.
 */
typedef struct {
	__IO uint32_t CSI_EN_REG;       /*!< 0x00 CSI enable register*/
	__IO uint32_t CSI_CFG_REG;      /*!< 0x04 CSI configuration register*/
	__IO uint32_t CSI_CAP_REG;      /*!< 0x08 CSI capture control register*/
	__IO uint32_t CSI_SCALE_REG;    /*!< 0x0c CSI scale */
		 uint32_t RESERVED1[6];
	__IO uint32_t CSI_BUF_CTL_REG;  /*!< 0x28 CSI output buffer control register*/
	__IO uint32_t CSI_BUF_STA_REG;  /*!< 0x2c CSI status register*/
	__IO uint32_t CSI_INT_EN_REG;   /*!< 0x30 CSI interrupt enable register*/
	__IO uint32_t CSI_INT_STA_REG;  /*!< 0x34 CSI interrupt size register*/
		 uint32_t RESERVED2[2];
	__IO uint32_t CSI_HSIZE_REG;    /*!< 0x40 CSI horizontal size register*/
		 uint32_t RESERVED3[1];
	__IO uint32_t CSI_BF_LEN_REG;   /*!< 0x48 CSI line buffer length register*/
	__IO uint32_t CSI_TRUE_DATA_NUM;/*!< 0x4c CSI true data number of fifo sram register*/
	__IO uint32_t CSI_JPEG_MOD_SEL; /*!< 0x50 CSI JPEG Mode select register*/
} CSI_T ;

#define CSI         ((CSI_T *)CSI_BASE)

#define CSI_FIFO_A  (CSI_BASE + 0X800)
#define CSI_FIFO_B  (CSI_BASE + 0XA00)

#define CSI_EN HAL_BIT(0)
#define CSI_CFG_INPUT_FORMAT ((uint32_t)0x00700000)
#define CSI_CFG_SYNC_SIGNAL_POL ((uint32_t)0x00000007)
#define CSI_CAP_MODE ((uint32_t)0x00000003)

#define CSI_VER_MASK ((uint32_t)0x0F000000)
#define CSI_HER_MASK ((uint32_t)0x000000FF)

#define LUM_STATIS   ((uint32_t)0xFFFFFF00)

#define CSI_INTERRUPT_CLEAR ((uint32_t)0x000003FF)

#define CSI_SIZE_REG ((uint32_t)0xFFFFFFFF)

#define CSI_LUM_LEN ((uint32_t)0x00001FFF)
#define CSI_VALID_DATA_LEN ((uint32_t)0x000003FF)

/**
 * @brief Set CSI clock.
 */
typedef struct {
	CCM_AHBPeriphClkSrc clk;   /*!< The source clock for CSI */
	CCM_PeriphClkDivN divN;    /*!< CSI sourec clock division */
	CCM_PeriphClkDivM divM;    /*!< CSI sourec clock division The division result = divN * divM*/
} CSI_Clk;

/**
 * @brief CSI clock configure.
 */
typedef struct {
	CSI_Clk src_Clk;    /*!< Set CSI clock */
} CSI_Config;

/**
 * @brief Enable or disable func.
 */
typedef enum {
	CSI_DISABLE,    /*!< Enable*/
	CSI_ENABLE,     /*!< Disable*/
} CSI_CTRL;

/**
 * @brief Sync signal polarity.
 */
typedef enum {
	CSI_NEGATIVE,    /*!< Negative*/
	CSI_POSITIVE,    /*!< Positive*/
} CSI_SYNC_SIGNAL_POLARITY;

/**
 * @brief Configure CSI sync signal polarity.
 */
typedef struct {
	CSI_SYNC_SIGNAL_POLARITY vsync;     /*!< Set the polarity for vsync signal */
	CSI_SYNC_SIGNAL_POLARITY herf;      /*!< Set the polarity for herf signal */
	CSI_SYNC_SIGNAL_POLARITY p_Clk;     /*!< Set the polarity for pclk signal */
} CSI_Sync_Signal;

/**
 * @brief CSI capture mode.
 */
typedef enum {
	CSI_STILL_MODE,    /*!< Capture one picture;When a picturee capture done, you need restart capture */
	CSI_VIDEO_MODE,    /*!< Continuous capture of images, shoot video*/
} CSI_CAPTURE_MODE;

/**
 * @brief The FIFO used for cache data.
 */
typedef enum {
	CSI_FIFO_0_A,    /*!< FIFO A */
	CSI_FIFO_0_B,    /*!< FIFO B */
} CSI_FIFO;

/**
 * @brief CSI module status.
 */
typedef enum {
	CSI_FREE,    /*!< The CSI module is free*/
	CSI_BUSY,    /*!< The CSI module is running*/
} CSI_RUN_STA;

/**
 * @brief CSI moudle Current operating status
 */
typedef struct {
	uint32_t luminance;        /*!< luminance statistics*/
	CSI_RUN_STA video_Mode;    /*!< video mode  status*/
	CSI_RUN_STA still_Mode;    /*!< still  mode  status*/
} CSI_Status;

/**
 * @brief Sets the size of the captured image.
 */
typedef struct {
	uint16_t hor_start;  /*!< Start capture position, unit byte.*/
	uint16_t hor_len;    /*!< The picture one row contains the number of data(not number of pixel), unit byte.*/
} CSI_Picture_Size;


/**
 * @brief The CSI interrupt flag.
 */
typedef enum {
	CSI_CAPTURE_DONE_IRQ = HAL_BIT(0),      /*!< When the picture capture done, trigger this flag.*/
	CSI_FRAME_DONE_IRQ = HAL_BIT(1),        /*!< When capture the frame done signal, trigger this flag.*/
	CSI_FIFO_0_OVERFLOW_IRQ = HAL_BIT(2),   /*!< When FIFO is overflow, trigger this flag.*/
	CSI_ALL_FIFO_OVERFLOW_IRQ = HAL_BIT(6), /*!< When all FIFO is overflow, trigger this flag.*/
	CSI_VSYNC_IRQ =  HAL_BIT(7),            /*!< When capture the vsync signal, trigger this flag.*/
	CSI_FIFO_0_A_READY_IRQ = HAL_BIT(8),    /*!< The CSI_FIFO_A is ready for read, trigger this flag.*/
	CSI_FIFO_0_B_READY_IRQ = HAL_BIT(9),    /*!< The CSI_FIFO_B is ready for read, trigger this flag.*/
} CSI_INTERRUPT_SIGNAL;

/**
 * @brief The number of bytes in the FIFO.
 */
typedef struct {
	uint16_t FIFO_0_A_Data_Len;    /*!< The number of bytes in the FIFO_A.*/
	uint16_t FIFO_0_B_Data_Len;    /*!< The number of bytes in the FIFO_B.*/
} CSI_FIFO_Data_Len;

/**
 * @brief Callback.
 */
typedef struct {
	void *arg;
	void(*callBack)(void *arg);
} CSI_Call_Back;

HAL_Status HAL_CSI_Config(CSI_Config *param);

/**
 * @brief Deinitializes all CSI peripherals registers and pins to their default reset
 *           values.
 * @param None
 * @retval None
 */
void HAL_CSI_DeInit(void);

/**
 * @brief Configure the CSI sync signal polarity.
 * @param signal: Set the polarity for vsync, herf,p_Clk.
 * @retval None
 */
void HAL_CSI_Sync_Signal_Polarity_Cfg(CSI_Sync_Signal *signal);

/**
 * @brief Enables or disables the CSI capture data.
 * @param mode: Select the capture mode.
 *        @arg CSI_STILL_MODE: capture one picture.
 *        @arg CSI_VIDEO_MODE:Continuous capture.
 * @param  ctrl: Controls the enable of CSI capture.
 *        @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None
 */
void HAL_CSI_Capture_Enable(CSI_CAPTURE_MODE mode, CSI_CTRL ctrl);

/**
 * @brief Configure the CSI interval capture.
 * @note This function is used to Configure interlaced acquisition and interlaced pixels.
 * @param vermask:  the interlaced pixels value.
 * @param hor_mask: the interlaced acquisition.
 * @retval None
 */
void HAL_CSI_Interval_Capture_Cfg(uint8_t ver_mask, uint16_t hor_mask);

/**
 * @brief Select the next FIFO for cache data.
 * @param fifo_num:
 *        @arg CSI_FIFO_0_A:  Select CSI_FIFO_0_A.
 *        @arg CSI_FIFO_0_B:  Select CSI_FIFO_0_B.
 * @retval None
 */
void HAL_CSI_Selection_Next_FIFO (CSI_FIFO fifo_num);

/**
 * @brief Gets the currently used FIFO.
 * @param None.
 * @retval CSI_FIFO: The FIFO that currently used.
 */
CSI_FIFO HAL_CSI_Current_FIFO(void);

/**
 * @brief Enables or disables used double FIFO for cache the data.
 * @note If you disable the double FIFO mode, the CSI_FIFO_A will be always
 *           used  for cache data.
 * @param ctrl: Controls the enable of double FIFO mode.
 *         @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None.
 */
void HAL_CSI_Double_FIFO_Mode_Enable(CSI_CTRL ctrl);

/**
 * @brief Gets the CSI moudle status.
 * @param None.
 * @retval CSI_Status: The status of CSI.
 */
CSI_Status HAL_CSI_Status(void);

/**
 * @brief Configure the interrupt flag for CSI.
 * @note Set the interrupt source for CSI.
 * @param irq_signel: The interrupt flag of CSI.
 * @param ctrl: Controls the enable of interrupt flag.
 *        @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None.
 */
void HAL_CSI_Interrupt_Cfg(CSI_INTERRUPT_SIGNAL irq_signel, CSI_CTRL ctrl);

/**
 * @brief Gets the CSI IRQ status.
 * @param None.
 * @return The status of CSI IRQ.
 *        @arg CSI_CAPTURE_DONE_IRQ: The picture capture done.
 *        @arg CSI_FRAME_DONE_IRQ:     The ftame done.
 *        @arg CSI_FIFO_0_OVERFLOW_IRQ: The FIFO is overflow.
 *              When this flag appears, you need to restart CSI.
 *        @arg CSI_ALL_FIFO_OVERFLOW_IRQ: ALL FIFO is overflow.
 *              When this flag appears, you need to restart CSI.
 *        @arg CSI_VSYNC_IRQ: Capture a vsync signal.
 *        @arg CSI_FIFO_0_A_READY_IRQ: The CSI_FIFO_A is ready for read.
 *        @arg CSI_FIFO_0_B_READY_IRQ: The CSI_FIFO_B is ready for read.
 */
__IO uint32_t HAL_CSI_Interrupt_Sta(void);

/**
 * @brief Clear the CSI interrupt flag.
 * @param None.
 * @retval None.
 */
void HAL_CSI_Interrupt_Clear(void);

/**
 * @brief Sets the size of the captured image.
 * @param size: Set the start capture position and the number of pixels per row.
 *        @arg size->start: start capture position, unit byte..
 *        @arg size->hor_len: the number of bytes per row.
 * @retval HAL_Status:
 *             The status of driver.
 */
HAL_Status HAL_CSI_Set_Picture_Size(CSI_Picture_Size *size);

/**
 * @brief  Cached data length of FIFO.
 * @param None
 * @retval CSI_FIFO_Data_Len: The data length of CSI_FIFO_A and CSI_FIFO_B.
 *        @arg FIFO_0_A_Data_Len: The data length of CSI_FIFO_A.
 *        @arg FIFO_0_B_Data_Len: The data length of CSI_FIFO_B.
 */
CSI_FIFO_Data_Len HAL_CSI_FIFO_Data_Len(void);

/**
 * @private
 * @brief Enables or disables the CSI JPEG mode.
 * @param ctrl: Controls the enable of CSI JPEG mode.
 *         @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None
 */
void HAL_CIS_JPEG_Mode_Enable(CSI_CTRL ctrl);

/**
 * @brief Enables or disables the CSI interrupt.
 * @param cb: The CSI interrupt callback function.
 * @param ctrl: Controls the enable of CSI interrupt.
 *        @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None
 */
void HAL_CSI_Interrupt_Enable(CSI_Call_Back *cb, CSI_CTRL ctrl);

/**
 * @brief Enables or disables the CSI interface.
 * @param ctrl: Controls the enable of CSI moudle.
 *         @arg This parameter can be: CSI_ENABLE or CSI_DISABLE.
 * @retval None
 */
void HAL_CSI_Moudle_Enalbe(CSI_CTRL ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_CSI_H_ */

#endif

