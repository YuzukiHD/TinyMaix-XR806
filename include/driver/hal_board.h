/**
 * @file  hal_board.h
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

#ifndef _DRIVER_HAL_BOARD_H_
#define _DRIVER_HAL_BOARD_H_

#include "driver/chip/hal_def.h"
#include "pm/pm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HAL board I/O control request
 *
 * @note Every request MUST be related to board's configuration
 */
typedef enum {
	HAL_BIR_PINMUX_INIT,        /* init pinmux */
	HAL_BIR_PINMUX_DEINIT,      /* deinit pinmux */
	HAL_BIR_CHIP_CLOCK_INIT,    /* init chip clock */
	HAL_BIR_GET_CFG,            /* get configuration */
	HAL_BIR_GET_PINMUX,         /* get pinmux */
#ifdef CONFIG_PM
	HAL_BIR_PINMUX_SUSPEND,     /* suspend pinmux */
	HAL_BIR_PINMUX_RESUME,      /* resume pinmux */
#endif
	HAL_BIR_POWER_CTRL,         /* power control */
} HAL_BoardIoctlReq;

/** @brief type define of HAL board I/O control callback function */
typedef HAL_Status (*HAL_BoardIoctlCb)(HAL_BoardIoctlReq req, uint32_t param0, uint32_t param1);

/**
 * @brief Register HAL board I/O control callback function
 * @param[in] cb Callback function to be registered
 * @return The previous callback function
 */
HAL_BoardIoctlCb HAL_BoardIoctlCbRegister(HAL_BoardIoctlCb cb);

/**
 * @brief Execute HAL board I/O control request
 * @note The registered callback function is called actually
 * @param[in] req HAL board I/O control request
 * @param[in] param0 Defined by HAL board I/O control request and driver
 * @param[in] param1 Defined by HAL board I/O control request and driver
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_BoardIoctl(HAL_BoardIoctlReq req, uint32_t param0, uint32_t param1);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_HAL_BOARD_H_ */
