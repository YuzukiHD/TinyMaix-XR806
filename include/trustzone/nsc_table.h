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

#ifndef __NSC_FUNCTIONS_H__
#define __NSC_FUNCTIONS_H__

#include <trustzone/tz_rpc.h>
#ifdef CONFIG_PM
#include "pm/pm.h"
#endif
#include "driver/chip/flash_crypto.h"

typedef enum {
	AES_MODE_NSC_ECB     = 0,
	AES_MODE_NSC_CBC,
	AES_MODE_NSC_CTR,
} AES_MODE_NSC;

typedef void (*Callback_t)(void);
uint32_t NSCFunction(Callback_t pxCallback);

void TZ_GetHeapSpace_NSC(uint8_t **start, uint8_t **end, uint8_t **current);

void TZ_StdoutInit_NSC(uint8_t uartID);

#if defined(CONFIG_TZ_XIP)
void TZ_FlashXipInit_NSC(void);
#endif
#if defined(CONFIG_TZ_PSRAM)
void TZ_PSRAMInit_NSC(void);
uint32_t TZ_PSRAMBinExist_NSC(void);
#endif
uint32_t TZ_GetFlashXipStartAddr_NSC(void);

void FlashCryptoInit_NSC(void);
void FlashCryptoDeInit_NSC(void);
/**
 * @brief Determine whether the request about flashc from NS is legitimate or not.
 * @param[in] arg The struct include flash info that to be transmit.
 *              arg->arg0 The field of flashc that NS wants.
 *              arg->arg1 The start flashc virtual address that NS wants.
 *              arg->arg2 The end flashc virtual address that NS wants.
 *              arg->arg3 The flash bias of flashc that wants.
 *              arg->retArg0 The real length that user wants.
 * @return RPC_Status, -1 on fail, else on success
 */
int32_t FlashCBUS_AddrRequire_NSC(tz_remote_call_t *arg);
int32_t FlashCryptoRequest_NSC(uint32_t startAddr, uint32_t endAddr,
                               uint8_t *key);
void FlashCryptoRelease_NSC(uint32_t channel);
int32_t XipDecryptEnable_NSC(uint32_t startAddr, uint32_t endAddr);
int32_t PsramDecryptEnable_NSC(uint32_t startAddr, uint32_t endAddr);
int32_t BinDecryptEnable_NSC(FLASH_CRYPTO_RANGE ch, uint32_t startAddr,
                             uint32_t endAddr);
void BinDecryptDisable_NSC(FLASH_CRYPTO_RANGE ch);

RPC_Status AES_Encrypt_NSC(tz_remote_call_t *arg, uint8_t *iv,
                           AES_MODE_NSC aes_mode);
RPC_Status AES_Decrypt_NSC(tz_remote_call_t *arg, uint8_t *iv,
                           AES_MODE_NSC aes_mode);
RPC_Status CMAC_AES128_NSC(tz_remote_call_t *arg);
RPC_Status CCM_Encrypt_NSC(tz_remote_call_t *arg, uint8_t *nonce,
                           uint8_t mlen, uint8_t *associated_data);
RPC_Status HMAC_SHA256_NSC(tz_remote_call_t *arg);
RPC_Status TZ_ExceptionRegister_NSC(tz_remote_call_t *arg);

#ifdef CONFIG_PM
void PM_SaveSecurePSP(void);
void PM_RestoreSecurePSP(void);
int PM_NoIrqSuspend_NSC(enum suspend_state_t state);
int PM_NoirqResume_NSC(enum suspend_state_t state);
int PM_IrqSuspend_NSC(enum suspend_state_t state);
int PM_IrqResume_NSC(enum suspend_state_t state);
void PM_CPUSuspend_NSC(void);
void PM_CPUHibernation_NSC(void);
#endif

#ifdef CONFIG_TZ_ROM
unsigned int NSC_GetRomVersion(char str[8]);
void NSC_GetRomBuildTime(char str[64]);
#endif

#ifdef CONFIG_TZ_EFUSE
HAL_Status EFUSE_Init_NSC(void);
void EFUSE_ReadData_NSC(uint8_t index, uint32_t *pData);
void EFUSE_WriteData_NSC(uint8_t index, uint32_t data);
void EFUSE_UpdateAll_NSC(void);
#endif

#endif /* __NSC_FUNCTIONS_H__ */

