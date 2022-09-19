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

#ifndef _DRIVER_CHIP_FLASH_DEBUG_H_
#define _DRIVER_CHIP_FLASH_DEBUG_H_

#include "sys/xr_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_SYSLOG    printf

#define FLASH_LOG(flags, fmt, arg...)   \
	do {                                \
		if (flags)                      \
			FLASH_SYSLOG(fmt, ##arg);   \
	} while (0)

#define PCHECK(p)

/* flash debug */
#define FLASH_DBG_ON    0
#define FLASH_ALE_ON    1
#define FLASH_ERR_ON    1
#define FLASH_NWA_ON    0

#define FD_DBG_ON        0
#define FD_ERR_ON        1
#define FD_INF_ON        1

#define FLASH_DEBUG(fmt, arg...)    FLASH_LOG(FLASH_DBG_ON, fmt"\n", ##arg)
#define FLASH_ALERT(fmt, arg...)    FLASH_LOG(FLASH_ALE_ON, fmt"\n", ##arg)
#define FLASH_ERROR(fmt, arg...)    FLASH_LOG(FLASH_ERR_ON, fmt"\n", ##arg)
#define FLASH_NOWAY(fmt, arg...)    FLASH_LOG(FLASH_NWA_ON, fmt"\n", ##arg)
#define FLASH_NOTSUPPORT()

#define FD_DEBUG(fmt, arg...) FLASH_LOG(FD_DBG_ON, "[FD D] "fmt"\n", ##arg)
#define FD_ERROR(fmt, arg...) FLASH_LOG(FD_ERR_ON, "[FD E] "fmt"\n", ##arg)
#define FD_INFO(fmt, arg...)  FLASH_LOG(FD_INF_ON, "[FD I] "fmt"\n", ##arg)

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_FLASH_DEBUG_H_ */
