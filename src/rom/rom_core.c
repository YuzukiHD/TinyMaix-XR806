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

#ifdef CONFIG_ROM

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel/os/os.h"
#ifdef CONFIG_TRUSTZONE
#include "secure/secure_context.h"
#include "non_secure/portasm.h"
#else
struct SecureContext;
typedef struct SecureContext*	SecureContextHandle_t;
void vRestoreContextOfFirstTask( void );
#endif
#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_wdg.h" /* for HAL_WDG_Feed() */
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_xip.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_util.h"
#include "sys/dma_heap.h"
#include "pm/pm.h"
#include "image/flash.h"
#include "image/fdcm.h"
#include "image/image.h"

#include "rom/ram_table.h"

void *__wrap__malloc_r(struct _reent *reent, size_t size);
void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size);
void __wrap__free_r(struct _reent *reent, void *ptr);
void *_sbrk(int incr);
int __wrap_vprintf(const char *format, va_list ap);
int __wrap_puts(const char *s);
int __wrap_vfprintf(FILE *stream, const char *format, va_list ap);
int __wrap_fputs(const char *s, FILE *stream);
int __wrap_putchar(int c);
int __wrap_putc(int c, FILE *stream);
int __wrap_fputc(int c, FILE *stream);
int __wrap_fflush(FILE *stream);
void cpu_suspend_cb(void);

#ifdef CONFIG_OS_FREERTOS
#include "FreeRTOS.h"
#include "task.h"

#define RAM_OS_HZ configTICK_RATE_HZ

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

static int ram_OS_SemaphoreIsValid(OS_Semaphore_t *sem)
{
	return OS_SemaphoreIsValid(sem);
}

static void ram_OS_SemaphoreSetInvalid(OS_Semaphore_t *sem)
{
	OS_SemaphoreSetInvalid(sem);
}

#ifdef CONFIG_OS_FREERTOS
static void ram_vPortAllocateSecureContext( uint32_t ulSecureStackSize )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void vPortAllocateSecureContext( uint32_t ulSecureStackSize ); /* __attribute__ (( naked )) */
	vPortAllocateSecureContext(ulSecureStackSize);
#endif
}

static void ram_vPortFreeSecureContext( uint32_t *pulTCB )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void vPortFreeSecureContext( uint32_t *pulTCB ) /* __attribute__ (( naked )) PRIVILEGED_FUNCTION */;
	vPortFreeSecureContext(pulTCB);
#endif
}

void ram_SecureContext_FreeContext( SecureContextHandle_t xSecureContextHandle )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureContext_FreeContext( SecureContextHandle_t xSecureContextHandle );
	SecureContext_FreeContext(xSecureContextHandle);
#endif
}

void ram_SecureContext_LoadContext( SecureContextHandle_t xSecureContextHandle )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureContext_LoadContext( SecureContextHandle_t xSecureContextHandle );
	SecureContext_LoadContext(xSecureContextHandle);
#endif
}

void ram_SecureContext_SaveContext( SecureContextHandle_t xSecureContextHandle )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureContext_SaveContext( SecureContextHandle_t xSecureContextHandle );
	SecureContext_SaveContext(xSecureContextHandle);
#endif
}

void ram_SecureContext_Init( void )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureContext_Init( void );
	SecureContext_Init();
#endif
}

void ram_SecureInit_DePrioritizeNSExceptions( void )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureInit_DePrioritizeNSExceptions( void );
	SecureInit_DePrioritizeNSExceptions();
#endif
}

#if( configENABLE_MPU == 1 )
SecureContextHandle_t ram_SecureContext_AllocateContext( uint32_t ulSecureStackSize, uint32_t ulIsTaskPrivileged )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern SecureContextHandle_t SecureContext_AllocateContext( uint32_t ulSecureStackSize, uint32_t ulIsTaskPrivileged );

	return SecureContext_AllocateContext(ulSecureStackSize, ulIsTaskPrivileged);
#else
	return NULL;
#endif
}
#else /* configENABLE_MPU */
SecureContextHandle_t ram_SecureContext_AllocateContext( uint32_t ulSecureStackSize )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern SecureContextHandle_t SecureContext_AllocateContext( uint32_t ulSecureStackSize );

	return SecureContext_AllocateContext(ulSecureStackSize);
#else
	return NULL;
#endif
}
#endif /* configENABLE_MPU */

void ram_SecureInit_EnableNSFPUAccess( void )
{
#if( configENABLE_TRUSTZONE == 1 )
	extern void SecureInit_EnableNSFPUAccess( void );
	SecureInit_EnableNSFPUAccess();
#endif
}
#endif /* CONFIG_OS_FREERTOS */

#define RAM_TBL_SET(name, val)          [RAM_TBL_IDX_##name] = (unsigned int)(val)

extern unsigned int __ram_table_lma_start__[];
#define RAM_TBL_ADJUST(name, val)       __ram_table_lma_start__[RAM_TBL_IDX_##name] = (unsigned int)(val)

unsigned int ram_table[256] __attribute__((section(".ram_table"))) = {
	/* chip */
	RAM_TBL_SET(SYS_PLL_CLOCK, SYS_PLL_CLOCK),
	RAM_TBL_SET(SYS_LFCLOCK, SYS_LFCLOCK),
	RAM_TBL_SET(HAL_PRCM_GetHFClock, HAL_PRCM_GetHFClock),
	RAM_TBL_SET(HAL_PRCM_GetLFClock, HAL_PRCM_GetLFClock),
	RAM_TBL_SET(HAL_PRCM_GetDevClock, HAL_PRCM_GetDevClock),
	RAM_TBL_SET(HAL_CCM_BusGetAPBSClock, HAL_CCM_BusGetAPBSClock),

	/* lib */
	RAM_TBL_SET(__wrap__malloc_r, __wrap__malloc_r),
	RAM_TBL_SET(__wrap__realloc_r, __wrap__realloc_r),
	RAM_TBL_SET(__wrap__free_r, __wrap__free_r),
	RAM_TBL_SET(_sbrk, _sbrk),
	RAM_TBL_SET(__wrap_vprintf, __wrap_vprintf),
	RAM_TBL_SET(__wrap_puts, __wrap_puts),
	RAM_TBL_SET(__wrap_vfprintf, __wrap_vfprintf),
	RAM_TBL_SET(__wrap_fputs, __wrap_fputs),
	RAM_TBL_SET(__wrap_putchar, __wrap_putchar),
	RAM_TBL_SET(__wrap_putc, __wrap_putc),
	RAM_TBL_SET(__wrap_fputc, __wrap_fputc),
	RAM_TBL_SET(__wrap_fflush, __wrap_fflush),

#ifdef CONFIG_OS_FREERTOS
	/* FreeRTOS */
	RAM_TBL_SET(configMINIMAL_STACK_SIZE, configMINIMAL_STACK_SIZE),
	RAM_TBL_SET(configTIMER_QUEUE_LENGTH, configTIMER_QUEUE_LENGTH),
	RAM_TBL_SET(configTIMER_TASK_STACK_DEPTH, configTIMER_TASK_STACK_DEPTH),
	RAM_TBL_SET(vApplicationStackOverflowHook, vApplicationStackOverflowHook),
	RAM_TBL_SET(portALLOCATE_SECURE_CONTEXT, ram_vPortAllocateSecureContext),
	RAM_TBL_SET(portCLEAN_UP_TCB, ram_vPortFreeSecureContext),
	RAM_TBL_SET(SecureContext_FreeContext, ram_SecureContext_FreeContext),
	RAM_TBL_SET(SecureContext_LoadContext, ram_SecureContext_LoadContext),
	RAM_TBL_SET(SecureContext_SaveContext, ram_SecureContext_SaveContext),
	RAM_TBL_SET(SecureContext_Init, ram_SecureContext_Init),
	RAM_TBL_SET(vRestoreContextOfFirstTask, vRestoreContextOfFirstTask),
	RAM_TBL_SET(SecureInit_DePrioritizeNSExceptions, ram_SecureInit_DePrioritizeNSExceptions),
	RAM_TBL_SET(SecureContext_AllocateContext, ram_SecureContext_AllocateContext),
	RAM_TBL_SET(pxPortInitialiseStack, pxPortInitialiseStack),
	RAM_TBL_SET(SecureInit_EnableNSFPUAccess, ram_SecureInit_EnableNSFPUAccess),
#endif

	/* OS */
	RAM_TBL_SET(OS_SemaphoreCreate, OS_SemaphoreCreate),
	RAM_TBL_SET(OS_SemaphoreCreateBinary, OS_SemaphoreCreateBinary),
	RAM_TBL_SET(OS_SemaphoreDelete, OS_SemaphoreDelete),
	RAM_TBL_SET(OS_SemaphoreWait, OS_SemaphoreWait),
	RAM_TBL_SET(OS_SemaphoreRelease, OS_SemaphoreRelease),
	RAM_TBL_SET(OS_SemaphoreIsValid, ram_OS_SemaphoreIsValid),
	RAM_TBL_SET(OS_SemaphoreSetInvalid, ram_OS_SemaphoreSetInvalid),

	RAM_TBL_SET(OS_MutexCreate, OS_MutexCreate),
	RAM_TBL_SET(OS_MutexDelete, OS_MutexDelete),
	RAM_TBL_SET(OS_MutexLock, OS_MutexLock),
	RAM_TBL_SET(OS_MutexUnlock, OS_MutexUnlock),
	RAM_TBL_SET(OS_RecursiveMutexCreate, OS_RecursiveMutexCreate),
	RAM_TBL_SET(OS_RecursiveMutexDelete, OS_RecursiveMutexDelete),
	RAM_TBL_SET(OS_RecursiveMutexLock, OS_RecursiveMutexLock),
	RAM_TBL_SET(OS_RecursiveMutexUnlock, OS_RecursiveMutexUnlock),

	RAM_TBL_SET(OS_ThreadList, OS_ThreadList),
	RAM_TBL_SET(OS_ThreadSuspendScheduler, OS_ThreadSuspendScheduler),
	RAM_TBL_SET(OS_ThreadResumeScheduler, OS_ThreadResumeScheduler),
	RAM_TBL_SET(OS_ThreadIsSchedulerRunning, OS_ThreadIsSchedulerRunning),

	RAM_TBL_SET(OS_TimerCreate, OS_TimerCreate),
	RAM_TBL_SET(OS_TimerDelete, OS_TimerDelete),
	RAM_TBL_SET(OS_TimerStart, OS_TimerStart),
	RAM_TBL_SET(OS_TimerChangePeriod, OS_TimerChangePeriod),
	RAM_TBL_SET(OS_TimerStop, OS_TimerStop),
	RAM_TBL_SET(OS_TimerIsActive, OS_TimerIsActive),

	RAM_TBL_SET(OS_HZ, RAM_OS_HZ),
	RAM_TBL_SET(OS_GetTicks, OS_GetTicks),
	RAM_TBL_SET(OS_MSleep, OS_MSleep),

	RAM_TBL_SET(HAL_UDelay, HAL_UDelay),
	RAM_TBL_SET(HAL_Alive, HAL_WDG_Feed),

	/* dma */
	RAM_TBL_SET(HAL_DMA_Init, HAL_DMA_Init),
	RAM_TBL_SET(HAL_DMA_Start, HAL_DMA_Start),
	RAM_TBL_SET(HAL_DMA_Stop, HAL_DMA_Stop),
	RAM_TBL_SET(HAL_DMA_FlashSbus_Iscoexist, HAL_DMA_FlashSbus_Iscoexist),
	RAM_TBL_SET(dma_malloc, dma_malloc),
	RAM_TBL_SET(dma_free, dma_free),
	RAM_TBL_SET(dma_calloc, dma_calloc),
	RAM_TBL_SET(dma_realloc, dma_realloc),

	/* image */
	RAM_TBL_SET(flash_rw, flash_rw),
	RAM_TBL_SET(flash_get_erase_block, flash_get_erase_block),
	RAM_TBL_SET(flash_erase, flash_erase),
	RAM_TBL_SET(fdcm_open, fdcm_open),
	RAM_TBL_SET(fdcm_read, fdcm_read),
	RAM_TBL_SET(fdcm_write, fdcm_write),
	RAM_TBL_SET(image_checksum16, image_checksum16),
	RAM_TBL_SET(image_get_addr, image_get_addr),
	RAM_TBL_SET(image_get_bl_size, image_get_bl_size),

	/* nvic, check HAL_NVIC_SetIRQHandler is used in rom code! */
	RAM_TBL_SET(HAL_NVIC_ConfigExtIRQ, HAL_NVIC_ConfigExtIRQ), /* for unify irq entry */

	/* flash */
	RAM_TBL_SET(HAL_Flash_Open, HAL_Flash_Open),
	RAM_TBL_SET(HAL_Flash_Close, HAL_Flash_Close),
	RAM_TBL_SET(HAL_Flash_Write, HAL_Flash_Write),
	RAM_TBL_SET(HAL_Flash_Read, HAL_Flash_Read),
	RAM_TBL_SET(HAL_Flash_Erase, HAL_Flash_Erase),
	RAM_TBL_SET(HAL_Flash_MemoryOf, HAL_Flash_MemoryOf),
	RAM_TBL_SET(flashcDriverCreate, flashcDriverCreate),
#if FLASH_SPI_ENABLE
	RAM_TBL_SET(spiDriverCreate, spiDriverCreate),
#else
	RAM_TBL_SET(spiDriverCreate, NULL),
#endif

	RAM_TBL_SET(FlashChipGetCfgList, FlashChipGetCfgList),
	RAM_TBL_SET(FlashChipGetChipList, FlashChipGetChipList),
	RAM_TBL_SET(defaultControl, defaultControl),
	RAM_TBL_SET(HAL_Flashc_Delay, HAL_Flashc_Delay),
	RAM_TBL_SET(HAL_Flashc_Xip_Init, HAL_Flashc_Xip_Init),
	RAM_TBL_SET(HAL_Flashc_Xip_Deinit, HAL_Flashc_Xip_Deinit),
	RAM_TBL_SET(HAL_Flashc_Init, HAL_Flashc_Init),
	RAM_TBL_SET(HAL_Flashc_Deinit, HAL_Flashc_Deinit),
	RAM_TBL_SET(HAL_Flashc_Open, HAL_Flashc_Open),
	RAM_TBL_SET(HAL_Flashc_Close, HAL_Flashc_Close),
	RAM_TBL_SET(HAL_Flashc_Ioctl, HAL_Flashc_Ioctl),
	RAM_TBL_SET(HAL_Flashc_Transfer, HAL_Flashc_Transfer),
	RAM_TBL_SET(FC_GetDelayCycle, FC_GetDelayCycle),
	RAM_TBL_SET(FLASH_XIP_START_ADDR, FLASH_XIP_START_ADDR),
	RAM_TBL_SET(FLASH_XIP_END_ADDR, FLASH_XIP_END_ADDR_FAKE),
	RAM_TBL_SET(FLASH_XIP_USER_START_ADDR, FLASH_XIP_USER_START_ADDR_FAKE),
	RAM_TBL_SET(FLASH_XIP_USER_END_ADDR, PSRAM_START_ADDR),
	RAM_TBL_SET(PSRAM_START_ADDR, PSRAM_START_ADDR),
	RAM_TBL_SET(PSRAM_END_ADDR, PSRAM_END_ADDR),
	RAM_TBL_SET(FlashCBUS_AddrRequire, FlashCBUS_AddrRequire),
	RAM_TBL_SET(FlashCryptoRequest, FlashCryptoRequest),

	/* pm */
#ifdef CONFIG_PM
	RAM_TBL_SET(WakeIo_To_Gpio, WakeIo_To_Gpio),
	RAM_TBL_SET(pm_check_wakeup_irqs, pm_check_wakeup_irqs),
	RAM_TBL_SET(cpu_suspend_cb, cpu_suspend_cb),
#endif
};

void ram_table_adjust(void)
{
	uint32_t add = FC_GetXipUserEndAddr();
	RAM_TBL_ADJUST(FLASH_XIP_END_ADDR, FLASH_XIP_END_ADDR);
	RAM_TBL_ADJUST(FLASH_XIP_USER_START_ADDR, FLASH_XIP_USER_START_ADDR);
	RAM_TBL_ADJUST(FLASH_XIP_USER_END_ADDR, add);
}

#endif /* CONFIG_ROM */
