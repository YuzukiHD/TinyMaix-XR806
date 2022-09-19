/**
  * @file  hal_i2c.c
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

#include "driver/chip/hal_i2c.h"
#include "hal_base.h"

/* I2C_Private.ctrl */
#define I2C_SCCB_MODE_BIT	HAL_BIT(3)

typedef struct {
	uint8_t                 ctrl;
	uint8_t                 memAddrSizeCnt;
	uint16_t                devAddr;
	uint32_t                memAddr;
	uint8_t                 *buf;
	int32_t                 size;

	HAL_Mutex               mtx;
	HAL_Semaphore           sem;
} I2C_Private;

extern void I2C_IRQHandler(I2C_T *i2c, I2C_Private *priv);
extern void I2C_SCCBIRQHandler(I2C_T *i2c, I2C_Private *priv);
extern HAL_Status __HAL_I2C_Init(I2C_ID i2cID, const I2C_InitParam *initParam);

extern I2C_Private gI2CPrivate[I2C_NUM];

static I2C_T           *gI2CInstance[I2C_NUM] = {I2C0, I2C1};

static __inline I2C_T *I2C_GetI2CInstance(I2C_ID i2cID)
{
	return gI2CInstance[i2cID];
}

static __inline void I2C_SoftReset(I2C_T *i2c)
{
	HAL_SET_BIT(i2c->I2C_SOFT_RST, I2C_SOFT_RST_BIT);
}

__nonxip_text
static __inline uint8_t I2C_IsSCCBMode(I2C_Private *priv)
{
	return !!HAL_GET_BIT(priv->ctrl, I2C_SCCB_MODE_BIT);
}

__nonxip_text
static __inline uint32_t I2C_GetIRQStatus(I2C_T *i2c)
{
	return HAL_GET_BIT(i2c->I2C_STATUS, I2C_STATUS_MASK);
}

__nonxip_text
static __inline void I2C_ClrIRQFlag(I2C_T *i2c)
{
	HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_IRQ_FLAG_BIT);
}

__nonxip_text
static __inline uint8_t I2C_GetIRQFlag(I2C_T *i2c)
{
	return !!HAL_GET_BIT(i2c->I2C_CTRL, I2C_IRQ_FLAG_BIT);
}

__nonxip_text
static __inline void I2C_BusErrClear(I2C_T *i2c)
{
	uint32_t IRQStatus = I2C_GetIRQStatus(i2c);

	if (IRQStatus == I2C_BUS_ERROR) {
		HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_IRQ_FLAG_BIT | I2C_STOP_BIT);
		while (HAL_GET_BIT(i2c->I2C_CTRL, I2C_IRQ_FLAG_BIT) || HAL_GET_BIT(i2c->I2C_CTRL, I2C_STOP_BIT))
			;
		I2C_SoftReset(i2c);
		I2C_ClrIRQFlag(i2c);
		while (I2C_GetIRQFlag(i2c))
			;
		return;
	}
}

__nonxip_text
void __I2C0_IRQHandler(void)
{
	I2C_BusErrClear(I2C0);

	if (I2C_IsSCCBMode(&gI2CPrivate[I2C0_ID]))
		I2C_SCCBIRQHandler(I2C0, &gI2CPrivate[I2C0_ID]);
	else
		I2C_IRQHandler(I2C0, &gI2CPrivate[I2C0_ID]);
}

__nonxip_text
void __I2C1_IRQHandler(void)
{
	I2C_BusErrClear(I2C1);

	if (I2C_IsSCCBMode(&gI2CPrivate[I2C1_ID]))
		I2C_SCCBIRQHandler(I2C1, &gI2CPrivate[I2C1_ID]);
	else
		I2C_IRQHandler(I2C1, &gI2CPrivate[I2C1_ID]);
}

HAL_Status HAL_I2C_Init(I2C_ID i2cID, const I2C_InitParam *initParam)
{
	HAL_Status ret;
	IRQn_Type			IRQn;
	NVIC_IRQHandler 	IRQHandler;

	ret = __HAL_I2C_Init(i2cID, initParam);

	/* enable NVIC IRQ */
	if (i2cID == I2C0_ID) {
		IRQn = I2C0_IRQn;
		IRQHandler = __I2C0_IRQHandler;
	} else {
		IRQn = I2C1_IRQn;
		IRQHandler = __I2C1_IRQHandler;
	}
	HAL_NVIC_ConfigExtIRQ(IRQn, IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

	return ret;
}

/**
 * @brief Reset the specified I2C
 * @param[in] i2cID ID of the specified I2C
 * @retval void
 */
void HAL_I2C_Reset(I2C_ID i2cID)
{
	I2C_T *i2c;
	i2c = I2C_GetI2CInstance(i2cID);
	I2C_SoftReset(i2c);
}
