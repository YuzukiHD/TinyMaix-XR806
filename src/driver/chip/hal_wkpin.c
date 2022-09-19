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

#include "pm/pm.h"
#include "hal_base.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_wkpin.h"
#include "driver/chip/private/hal_os.h"
#include "kernel/os/os_thread.h"
#include "kernel/os/os_time.h"

struct wkpin_pinmux_info {
	GPIO_PinMuxParam *pinmux;
	uint32_t count;
};

#define WKPIN_GPIOA_IRQ         ((GPIO_IRQ_T *)GPIOA_IRQ_BASE)
#define WKPIN_GPIOA_CTRL        ((GPIO_CTRL_T *)GPIOA_CTRL_BASE)

/*1: alive; 0: lowpower*/
static uint8_t host_state = WKPIN_HOST_ALIVE;
static OS_Thread_t wkpin_thread;
static OS_Semaphore_t wkpin_sem;
static enum suspend_state_t wkpin_lp_state;

static uint8_t wkpin_host2dev_port;
static uint8_t wkpin_host2dev_pin;
static uint8_t wkpin_dev2host_port;
static uint8_t wkpin_dev2host_pin;

static void wkpin_task(void *arg)
{
	HAL_SemaphoreInit(&wkpin_sem, 0, 1);
	while (1) {
		HAL_SemaphoreWait(&wkpin_sem, HAL_WAIT_FOREVER);
		if (host_state == WKPIN_HOST_LOWPOWER) {
			/* *(enum suspend_state_t *)arg */
			if (pm_enter_mode_timeout(wkpin_lp_state, 20) != 0) {
				HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_LOW);
				HAL_UDelay(500);
				HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_HIGH);
				HAL_UDelay(500);
			}
		} else {
			HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_HIGH);
			HAL_UDelay(500);
		}
	}

}

static void wkpin_task_create(enum suspend_state_t *state)
{
	OS_ThreadSetInvalid(&wkpin_thread);
	if (OS_ThreadCreate(&wkpin_thread,
	                    "wkpin_thread",
	                    wkpin_task,
	                    (void *)state,
	                    OS_PRIORITY_HIGH,
	                    512) != OS_OK) {
		HAL_ERR("create wkpin_thread task failed\n");
	}
}

__nonxip_text static void wkpin_host2dev_irqCb(void *arg)
{
	/* set IRQ trigger mode */
	uint32_t regIdx;
	uint32_t bitShift;
	uint8_t gpioTriMode;
	HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_17, GPIO_PIN_HIGH);
	/*disable gpio irq*/
	HAL_CLR_BIT(WKPIN_GPIOA_IRQ->IRQ_EN, HAL_BIT(wkpin_host2dev_pin));
	bitShift = wkpin_host2dev_pin * GPIO_IRQ_EVT_BITS;
	regIdx = bitShift / 32;
	bitShift = bitShift % 32;
	gpioTriMode = HAL_GET_BIT_VAL(WKPIN_GPIOA_IRQ->IRQ_MODE[regIdx], bitShift, GPIO_IRQ_EVT_VMASK);
	if (gpioTriMode == GPIO_IRQ_EVT_HIGH_LEVEL) {
		gpioTriMode = GPIO_IRQ_EVT_LOW_LEVEL;
		host_state = WKPIN_HOST_ALIVE;
		/*device -> host*/
		HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_HIGH);
	} else {
		gpioTriMode = GPIO_IRQ_EVT_HIGH_LEVEL;
		host_state = WKPIN_HOST_LOWPOWER;
		/*device -> host*/
		//HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_LOW);
	}
	OS_SemaphoreRelease(&wkpin_sem);

	HAL_MODIFY_REG(WKPIN_GPIOA_IRQ->IRQ_MODE[regIdx], GPIO_IRQ_EVT_VMASK << bitShift, gpioTriMode << bitShift);
	/*clear gpio pending*/
	HAL_SET_BIT(WKPIN_GPIOA_IRQ->IRQ_STATUS, HAL_BIT(wkpin_host2dev_pin));
	/*enable gpio irq*/
	HAL_SET_BIT(WKPIN_GPIOA_IRQ->IRQ_EN, HAL_BIT(wkpin_host2dev_pin));
	HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_17, GPIO_PIN_LOW);
	return;
}

static void wkpin_pin_config(void)
{
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_WKPIN, 0), 0);
	HAL_UDelay(1000);
	GPIO_IrqParam irq_param;
	irq_param.event = GPIO_IRQ_EVT_LOW_LEVEL;
	irq_param.callback = wkpin_host2dev_irqCb;
	irq_param.arg = (void *)0;
	HAL_GPIO_EnableIRQ(wkpin_host2dev_port, wkpin_host2dev_pin, &irq_param);
}

static int wkpin_norirq_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Wakeup_SetIO(2, WKUPIO_WK_MODE_RISING_EDGE, GPIO_PULL_DOWN);
		break;
	default:
		break;
	}
	/*device -> host*/
	HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_LOW);
	return 0;
}

static int wkpin_norirq_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		wkpin_pin_config();
		break;
	case PM_MODE_HIBERNATION:
		break;
	default:
		break;
	}
	return 0;
}

static const struct soc_device_driver wkpin_drv = {
	.name = "wkpin",
	.suspend_noirq = wkpin_norirq_suspend,
	.resume_noirq = wkpin_norirq_resume,
};

static struct soc_device wkpin_dev = {
	.name = "wkpin",
	.driver = &wkpin_drv,
};

int HAL_WKPIN_GetHostState(void)
{
	return host_state;
}

void HAL_WKPIN_WakeupHost(void)
{
	if (host_state != WKPIN_HOST_ALIVE) {
		HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_HIGH);
		HAL_UDelay(500);
	}
	return;
}

void HAL_WKPIN_Init(enum suspend_state_t state)
{
	struct wkpin_pinmux_info wkpin_pin_cfg;
	HAL_BoardIoctl(HAL_BIR_GET_PINMUX, HAL_MKDEV(HAL_DEV_MAJOR_WKPIN, 0), (uint32_t)(&wkpin_pin_cfg));
	wkpin_host2dev_port = wkpin_pin_cfg.pinmux[0].port;
	wkpin_host2dev_pin = wkpin_pin_cfg.pinmux[0].pin;
	wkpin_dev2host_port = wkpin_pin_cfg.pinmux[1].port;
	wkpin_dev2host_pin = wkpin_pin_cfg.pinmux[1].pin;
	wkpin_lp_state = state;
	wkpin_task_create(&wkpin_lp_state);
	wkpin_pin_config();
	/*device -> host*/
	HAL_GPIO_WritePin(wkpin_dev2host_port, wkpin_dev2host_pin, GPIO_PIN_HIGH);
	pm_register_ops(&wkpin_dev);
	return;
}
