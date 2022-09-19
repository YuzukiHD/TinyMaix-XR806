/**
  * @file  hal_flashctrl.c
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
#include "hal_base.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "driver/chip/hal_xip.h"
#include "sys/xr_debug.h"
#ifdef CONFIG_TRUSTZONE
#include "nsc_table.h"
#include "tz_rpc.h"
#endif

#if 0
#define FC_DEBUG_ON (DBG_OFF)
#define FC_DEBUG(msg, arg...) XR_DEBUG((FC_DEBUG_ON | XR_LEVEL_ALL), NOEXPAND, "[FC DBG] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define FC_ERROR(msg, arg...) XR_ERROR((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[FC ERR] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#endif

#if 1
#define FC_DEBUG_ON (DBG_ON)
#define FC_DEBUG(msg, arg...) XR_DEBUG((DBG_ON), NOEXPAND, "[FC DBG] %s:" msg "\n", __func__, ##arg)
#define FC_WARN(msg, arg...)  XR_DEBUG((DBG_ON), NOEXPAND, "[FC WRN] %s:" msg "\n", __func__, ##arg)
#define FC_ERROR(msg, arg...) XR_ERROR((DBG_ON), NOEXPAND, "[FC ERR] %s:" msg "\n", __func__, ##arg)
#endif

void FC_GetDelayCycle(Flash_Ctrl_DelayCycle *delay, uint32_t freq)
{
	HAL_Memset(delay, 0, sizeof(Flash_Ctrl_DelayCycle));
	delay->cs_begin = 1;
	/*delay.cs_deselect = t_shsl_ns * (freq / 1000000) / 1000;*/
#ifdef CONFIG_PLATFORM_FPGA
	delay->cs_deselect = 3;
#else
	delay->cs_deselect = 8;
#endif

#ifdef CONFIG_PLATFORM_FPGA
	uint32_t apbs_clk = HAL_GetDevClock();
	if (apbs_clk == HOSC_CLOCK_24M) {
		delay->data = 1;
	} else if (apbs_clk == HOSC_CLOCK_48M) {
		delay->data = 2;
	} else if (apbs_clk > HOSC_CLOCK_48M) {
		delay->data = 3;
	} else {
		delay->data = 0;
	}
#else
	if (freq < 48000000) {
		delay->data = 0;
	} else if (freq <= 64000000) {
		delay->data = 1;
	} else {
		delay->data = 2;
	}
#endif
}

uint32_t FC_GetXipUserEndAddr(void)
{
#ifdef CONFIG_TRUSTZONE
	uint32_t addr = 0;
	addr = TEE_FUNC_CALL(TZ_GetFlashXipStartAddr_NSC);
	addr = (addr == 0) ? PSRAM_START_ADDR : addr;
	return addr;
#else
	return PSRAM_START_ADDR;
#endif
}

int32_t FlashCBUS_AddrRequire(uint8_t field, uint32_t sAddr, uint32_t bias,
                              uint32_t userLen)
{
	int32_t cache_bias;

	if (field > FLASH_ADDR_USER_RD_IDX) {
		return -1;
	}

#ifdef CONFIG_TRUSTZONE
	tz_remote_call_t rpc_t;
	rpc_t.arg0 = field;
	rpc_t.arg1 = sAddr;
	rpc_t.arg2 = bias;
	rpc_t.retArg0 = userLen;
	cache_bias = TEE_FUNC_CALL(FlashCBUS_AddrRequire_NSC, &rpc_t);
#else
	uint32_t eAddr;

	cache_bias = rounddown2(bias, FLASH_XIP_USER_ADDR_STEP);
	eAddr = roundup2(sAddr + userLen + (bias - cache_bias),
	                 FLASH_XIP_USER_ADDR_STEP);

	HAL_MODIFY_REG(OPI_MEM_CTRL->FLASH_ADDR[field].START_ADDR,
	               FLASHC_ADDR_FIELD_START_MASK,
	               (sAddr << FLASHC_ADDR_FIELD_START_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->FLASH_ADDR[field].END_ADDR,
	               FLASHC_ADDR_FIELD_END_MASK,
	               (eAddr << FLASHC_ADDR_FIELD_END_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->FLASH_ADDR[field].BIAS_ADDR,
	               FLASHC_ADDR_FIELD_BIAS_MASK, cache_bias);
	HAL_SET_BIT(OPI_MEM_CTRL->FLASH_ADDR[field].BIAS_ADDR,
	            FLASHC_ADDR_FIELD_BIAS_EN_MASK);
#endif
	return cache_bias;
}

#ifdef CONFIG_PSRAM
int32_t PsramCBUS_AddrRequire(uint8_t field, uint32_t sAddr, uint32_t bias,
                              uint32_t userLen)
{
	int32_t ret = 0;

#ifdef CONFIG_TRUSTZONE
	tz_remote_call_t rpc_t;
	rpc_t.arg0 = field;
	rpc_t.arg1 = sAddr;
	rpc_t.arg2 = bias;
	rpc_t.retArg0 = userLen;
	ret = TEE_FUNC_CALL(FlashCBUS_AddrRequire_NSC, &rpc_t);
#else
	uint32_t eAddr;

	eAddr = sAddr + userLen;
	HAL_PsramCtrl_Set_Address_Field(NULL, field, sAddr, eAddr, bias);
#endif
	return ret;
}
#endif

#if (CONFIG_CHIP_ARCH_VER == 3)
__sram_text
void HAL_FlashCrypto_Init(uint8_t *nonce)
{
#ifdef CONFIG_TRUSTZONE
	TEE_FUNC_CALL(FlashCryptoInit_NSC);
#else
	flash_crypto_init();
#endif
	HAL_PRCM_SetFlashCryptoNonce(nonce);
}

void HAL_FlashCrypto_DeInit(void)
{
#ifdef CONFIG_TRUSTZONE
	TEE_FUNC_CALL(FlashCryptoDeInit_NSC);
#else
	flash_crypto_deinit();
#endif
}

__sram_text
int32_t FlashcXipDecryptEnRequest(uint32_t startAddr, uint32_t endAddr)
{
#ifdef CONFIG_TRUSTZONE
	return TEE_FUNC_CALL(XipDecryptEnable_NSC, startAddr, endAddr);
#else
	return flash_bin_decrypt_enable(FCRYPTO_RANGE_2, startAddr, endAddr);
#endif
}


int32_t FlashcPsramDecryptEnRequest(uint32_t startAddr, uint32_t endAddr)
{
#ifdef CONFIG_TRUSTZONE
	return TEE_FUNC_CALL(PsramDecryptEnable_NSC, startAddr, endAddr);
#else
	return flash_bin_decrypt_enable(FCRYPTO_RANGE_3, startAddr, endAddr);
#endif
}

int32_t FlashcBinDecryptEnable(FLASH_CRYPTO_RANGE ch, uint32_t startAddr,
                               uint32_t endAddr)
{
#ifdef CONFIG_TRUSTZONE
	return TEE_FUNC_CALL(BinDecryptEnable_NSC, ch, startAddr, endAddr);
#else
	return flash_bin_decrypt_enable(ch, startAddr, endAddr);
#endif
}

void FlashcBinDecryptDisable(FLASH_CRYPTO_RANGE ch)
{
#ifdef CONFIG_TRUSTZONE
	TEE_FUNC_CALL(BinDecryptDisable_NSC, ch);
#else
	flash_bin_decrypt_disable(ch);
#endif
}

int32_t FlashCryptoRequest(uint32_t startAddr, uint32_t endAddr, uint8_t *key)
{
#ifdef CONFIG_TRUSTZONE
	return TEE_FUNC_CALL(FlashCryptoRequest_NSC, startAddr, endAddr, key);
#else
	return flash_crypto_enable(startAddr, endAddr, key);
#endif
}

void FlashCryptoRelease(uint32_t channel)
{
#ifdef CONFIG_TRUSTZONE
	TEE_FUNC_CALL(FlashCryptoRelease_NSC, channel);
#else
	flash_crypto_disable(channel);
#endif
}
#endif /* CONFIG_CHIP_ARCH_VER */

#ifdef CONFIG_ROM
#ifdef CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ
void HAL_Flashc_Xip_RawEnable(struct flash_controller *ctrl)
{
	if (!ctrl->xip_on) {
		return;
	}

#if (CONFIG_CHIP_ARCH_VER == 2)
	if (ctrl->xip_continue) {
		HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_CBUS_RW_ENABLE_BIT);
	}
#endif

	HAL_ExitCriticalSection(ctrl->irqsaveflag);
	__ISB();
	FC_DEBUG("%s():%d enable irq\n", __func__, __LINE__);
}

void HAL_Flashc_Xip_RawDisable(struct flash_controller *ctrl)
{
	if (!ctrl->xip_on) {
		return;
	}

	ctrl->irqsaveflag = HAL_EnterCriticalSection();
	FC_DEBUG("%s():%d disable irq\n", __func__, __LINE__);

#if (CONFIG_CHIP_ARCH_VER == 2)
#if 0
	HAL_UDelay(100);
#endif
	while (FC_Sbus_GetDebugState() != 0x00)
		;
	if (ctrl->xip_continue) {
		HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_CBUS_RW_ENABLE_BIT);
	}
#endif
}
#endif /* CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ */

extern flashc_caback _flashc_caback;
HAL_Status HAL_Flashc_Open(struct flash_controller *ctrl)
{
	ctrl->sbusing = 1;
	if (ctrl->xip_on) {
		HAL_Flashc_Xip_RawDisable(ctrl);
#if (CONFIG_CHIP_ARCH_VER == 3)
		FC_SbusPrepare();
#endif
	}

	if (ctrl->resetMask) {
		ctrl->suspending = 1;

		if (ctrl->xip_on) {
			HAL_Flashc_Xip_Deinit(ctrl);

			FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
			ctrl->xip_on = 1;
		}

		if (_flashc_caback) {
			_flashc_caback(ctrl, FC_OPEN, 0);
		}
		HAL_Flashc_Deinit(ctrl);
		HAL_Flashc_Init(ctrl, &ctrl->pm_sbus_cfg);
		if (_flashc_caback) {
			_flashc_caback(ctrl, FC_OPEN, 1);
		}
		FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
	}

	HAL_Flashc_EnableCCMU(ctrl);
	HAL_Flashc_PinInit(ctrl);
	FC_Sbus_ResetFIFO(1, 1);

	return HAL_OK;
}

#ifdef CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ
HAL_Status HAL_Flashc_Close(struct flash_controller *ctrl)
{
	HAL_Flashc_PinDeinit(ctrl);
	HAL_Flashc_DisableCCMU(ctrl);

	if (ctrl->resetMask) {
		if (_flashc_caback) {
			_flashc_caback(ctrl, FC_CLOSE, 0);
		}
		HAL_Flashc_Deinit(ctrl);
		HAL_Flashc_Init(ctrl, &ctrl->pm_sbus_cfg);
		if (ctrl->xip_on) {
			HAL_Flashc_Xip_Init(ctrl, &ctrl->pm_ibus_cfg);
			HAL_UDelay(300);
		}
		if (_flashc_caback) {
			_flashc_caback(ctrl, FC_CLOSE, 1);
		}
		FC_DEBUG("ccmu: %d, pin: %d", ctrl->ccmu_on, ctrl->pin_inited);
		ctrl->suspending = 0;
	}

	if (ctrl->xip_on) {
#if (CONFIG_CHIP_ARCH_VER == 3)
		FC_SbusFinish();
#endif
		HAL_Flashc_Xip_RawEnable(ctrl);
	}
	ctrl->sbusing = 0;

	return HAL_OK;
}
#endif /* CONFIG_FLASH_ERASE_WRITE_DISABLE_IRQ */

#ifdef CONFIG_PM
extern int __flashc_suspend(struct soc_device *dev, enum suspend_state_t state);
extern int __flashc_resume(struct soc_device *dev, enum suspend_state_t state);

int flashc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	int ret;

	ret = __flashc_suspend(dev, state);
	if (ret) {
		return ret;
	}
	if (state == PM_MODE_STANDBY) {
		ret = HAL_BoardIoctl(HAL_BIR_PINMUX_SUSPEND, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
	}

	return ret;
}

int flashc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	int ret;

	if (state == PM_MODE_STANDBY) {
		ret = HAL_BoardIoctl(HAL_BIR_PINMUX_RESUME, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
		if (ret) {
			FC_ERROR("resume pin faild");
		}
	}

	ret = __flashc_resume(dev, state);
	return ret;
}
#endif /* CONFIG_PM */

extern HAL_Status __HAL_Flashc_Init(struct flash_controller *ctrl,
                                    const Flashc_Config *cfg);

/**
 * @brief Initialize Flash controller SBUS.
 * @param cfg:
 *     @arg cfg->freq: Flash working frequency.
 * @retval HAL_Status: The status of driver.
 */
HAL_Status HAL_Flashc_Init(struct flash_controller *ctrl,
                           const Flashc_Config *cfg)
{
	HAL_Status ret = __HAL_Flashc_Init(ctrl, cfg);
#ifdef CONFIG_PM
	if (!ctrl->suspending) {
		pm_unregister_ops(&ctrl->flashc_dev);
		ctrl->flashc_drv.suspend_noirq = flashc_suspend;
		ctrl->flashc_drv.resume_noirq = flashc_resume;
		ctrl->flashc_drv.enter_latency = 21;    /* at CPU 160MHz */
		ctrl->flashc_drv.exit_latency = 327;    /* at CPU 160MHz */
		pm_register_ops(&ctrl->flashc_dev);
	}
#endif

	return ret;
}

#endif

