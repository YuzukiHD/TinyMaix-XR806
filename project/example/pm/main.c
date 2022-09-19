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

#include <stdio.h>
#include <string.h>

#include "kernel/os/os.h"
#include "pm/pm.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_util.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"
#include "image/fdcm.h"
#include "image/image.h"

#define TEST_SLEEP          1
#define TEST_STANDBY        1
#define TEST_HIBERNATION    1
#define TEST_STANDBY_DTIM   0
//#define TEST_STANDBY_MIN_SRAM_RETENTION 0 /*NOT support now*/

#if (CONFIG_CHIP_ARCH_VER == 2)
	#define WAKEUP_IO_PIN_DEF   (5)
	#define WAKEUP_IO_MODE_DEF  (WKUPIO_WK_MODE_FALLING_EDGE)
	#define WAKEUP_IO_PULL_DEF  (GPIO_PULL_UP)
	#define WAKEUP_IO_EVT_BVT_EDGE GPIO_IRQ_EVT_BOTH_EDGE
#elif (CONFIG_CHIP_ARCH_VER == 3)
	#define WAKEUP_IO_PIN_DEF   (13)
	#define WAKEUP_IO_MODE_DEF  (WKUPIO_WK_MODE_RISING_EDGE)
	#define WAKEUP_IO_PULL_DEF  (GPIO_PULL_DOWN)
	#define WAKEUP_IO_EVT_BVT_EDGE GPIO_IRQ_EVT_RISING_EDGE
#endif
#define BUTTON_WAKEUP_PORT_DEF GPIO_PORT_A
#define BUTTON_WAKEUP_PIN_DEF  WakeIo_To_Gpio(WAKEUP_IO_PIN_DEF)

static uint16_t wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;
observer_base *g_ob;

/**
 * @brief wlan msg callback.
 *
 * @retval None.
 */
static void wlan_msg_recv(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);
	printf("%s msg type:%d\n", __func__, type);

	switch (type) {
	case NET_CTRL_MSG_WLAN_CONNECTED:
		wlan_event = NET_CTRL_MSG_WLAN_CONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_DISCONNECTED:
		wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_SCAN_SUCCESS:
	case NET_CTRL_MSG_WLAN_SCAN_FAILED:
	case NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED:
	case NET_CTRL_MSG_WLAN_CONNECT_FAILED:
		break;
	case NET_CTRL_MSG_CONNECTION_LOSS:
		wlan_event = WLAN_EVENT_CONNECTION_LOSS;
		break;
	case NET_CTRL_MSG_NETWORK_UP:
		wlan_event = NET_CTRL_MSG_NETWORK_UP;
		break;
	case NET_CTRL_MSG_NETWORK_DOWN:
		wlan_event = NET_CTRL_MSG_NETWORK_DOWN;
		break;
#if (!defined(CONFIG_LWIP_V1) && LWIP_IPV6)
	case NET_CTRL_MSG_NETWORK_IPV6_STATE:
		break;
#endif
	default:
		printf("unknown msg (%u, %u)\n", type, data);
		break;
	}
}

/**
 * @brief init register wlan msg callback function.
 *
 * @param none
 *
 * @retval None.
 */
static int wlan_msg_init(void)
{
	g_ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
	                                    NET_CTRL_MSG_ALL,
	                                    wlan_msg_recv,
	                                    NULL);
	if (g_ob == NULL)
		return -1;
	if (sys_ctrl_attach(g_ob) != 0)
		return -1;

	return 0;
}

/**
 * @brief deinit wlan mes callback function.
 *
 * @param none
 *
 * @retval None.
 */
static int wlan_msg_deinit(void)
{
	if (g_ob == NULL)
		return -1;
	if (sys_ctrl_detach(g_ob) != 0)
		return -1;

	return 0;
}

/**
 * @brief check wlan connect AP which will keep work at standby mode only.
 *
 * @param none
 *
 * @retval None.
 *
 * @other
 * cmds:
 * 1. net sta config ap_ssid [ap_psk]
 * 2. net sta enable
 * 3. netcmd lmac vif0_set_pm_dtim 8
 *    note: 8 should AP DTIM times(if AP DTIM is 3, set vif0_set_pm_dtim to 9)
 * 4. wifi will goto standby in 3S, and goto standby again if wakeup.
 */
static void pm_wakeup_wlan_connect_ap_attach(void)
{
#if PRJCONF_NET_EN
	wlan_msg_init();
	printf("\nPlease connect AP\n"
	       "Example "
	       "step1: net sta config TES_TPLINK_WDR6500#33 sw4wifionly  "
	       "step2: net sta enable\n");
	struct netif *nif = wlan_netif_get(WLAN_MODE_NONE);
	while (!(nif && NETIF_IS_AVAILABLE(nif))) {
		OS_MSleep(1000);
	}
	wlan_set_ps_mode(nif, 1);
	OS_MSleep(5000);
#else
	printf("\n\nPlease config open PRJCONF_NET_EN \n");
#endif
}

/**
 * @brief detach wlan message callback.
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_wlan_connect_ap_detach(void)
{
#if PRJCONF_NET_EN
	wlan_msg_deinit();
#else
	printf("\n\nPlease config open PRJCONF_NET_EN \n");
#endif
}

#if TEST_SLEEP
/**
 * @brief setup uart0 to wakeup source which will keep work at sleep mode only.
 *
 * @note uart irq only work in sleep mode
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_uartirq_init(void)
{
	HAL_UART_SetBypassPmMode(UART0_ID, PM_SUPPORT_SLEEP);
}

/**
 * @brief disable uart0 to wakeup source.
 *
 * @param none
 *
 * @retval  None.
 */
static void pm_wakeup_uartirq_deinit(void)
{
	HAL_UART_SetBypassPmMode(UART0_ID, 0);
}
#endif

#if TEST_SLEEP || TEST_STANDBY

/**
 * @brief setup wakeup source to timer which after 10 second will create irq.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_timer_init(void)
{
	/*wakeup timer 10 seconds*/
	HAL_Wakeup_SetTimer_mS(10000);
}

/**
 * @brief disable wakeup source to timer.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_timer_deinit(void)
{
	HAL_Wakeup_ClrTimer();
}
#endif

__nonxip_text
static void key_IrqCb(void *arg)
{
	printf("%s,%d\n", __func__, __LINE__);
}

/**
 * @brief setup wakeup source to button which after 10 second will create irq.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_button_init(void)
{
	GPIO_InitParam param;
	GPIO_IrqParam Irq_param;

	param.driving = GPIO_DRIVING_LEVEL_0;
	param.pull = WAKEUP_IO_PULL_DEF;
	param.mode = GPIOx_Pn_F6_EINT;
	HAL_GPIO_Init(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF, &param);

	Irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	Irq_param.callback = key_IrqCb;
	Irq_param.arg = (void *)0;
	HAL_GPIO_EnableIRQ(GPIO_PORT_A, BUTTON_WAKEUP_PIN_DEF, &Irq_param);
	HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);

	/*Sakeup io debounce clock source 0  freq is LFCLK 32K*/
	HAL_PRCM_SetWakeupDebClk0(0);
	/*Wakeup IO 5 debounce clock select source 0*/
	HAL_PRCM_SetWakeupIOxDebSrc(WAKEUP_IO_PIN_DEF, 0);
	/*Wakeup IO 5 input debounce clock cycles is 16+1*/
	HAL_PRCM_SetWakeupIOxDebounce(WAKEUP_IO_PIN_DEF, 1);
	/*Wakeup IO 5 enable, negative edge,  */
	HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
}

/**
 * @brief disable wakeup source to button.
 *
 * @param none
 *
 * @retval None.
 */
static void pm_wakeup_button_deinit(void)
{
	/*Sakeup io debounce clock source 0  freq is LFCLK 32K to default*/
	HAL_PRCM_SetWakeupDebClk0(0);
	/*Wakeup IO 5 debounce clock select source 0 to default*/
	HAL_PRCM_SetWakeupIOxDebSrc(WAKEUP_IO_PIN_DEF, 0);
	/*Wakeup IO 5 input debounce clock cycles to default*/
	HAL_PRCM_SetWakeupIOxDebounce(WAKEUP_IO_PIN_DEF, 0);

	HAL_GPIO_DisableIRQ(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
	HAL_GPIO_DeInit(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
}


int main(void)
{
	platform_init();

	printf("\n\nPM example start!\n");
	printf("Support 3 low power modes: sleep/standby/hibernation\n");
	printf("Sleep support wakeup methods:       timer/button/devirq\n");
	printf("Standby support wakeup methods:     timer/button/wlan\n");
	printf("Hibernation support wakeup methods: timer/button\n\n\n");
	OS_MSleep(3000);

#if TEST_SLEEP
	/*enetr sleep test*/
	printf("\n\nEnter sleep mode, setup wakeup source uartirq&timer&button\n\n");
	pm_wakeup_uartirq_init();
	pm_wakeup_timer_init();
	pm_wakeup_button_init();
	pm_enter_mode(PM_MODE_SLEEP);
	printf("Wakeup event:%x\n", HAL_Wakeup_GetEvent());
	printf("Exit sleep mode\n\n");
	pm_wakeup_uartirq_deinit();
	pm_wakeup_timer_deinit();
	pm_wakeup_button_deinit();
#endif

#if TEST_STANDBY
	/*enetr standby test*/
	printf("\n\nEnter standby mode, setup wakeup source wlan&timer&button\n\n");
	pm_wakeup_wlan_connect_ap_attach();
	pm_wakeup_timer_init();
	pm_wakeup_button_init();
	pm_enter_mode(PM_MODE_STANDBY);
	printf("Wakeup event:%x\n", HAL_Wakeup_GetEvent());
	printf("Exit standby mode\n\n");
	pm_wakeup_timer_deinit();
	pm_wakeup_button_deinit();
	pm_wakeup_wlan_connect_ap_detach();
#endif

#if TEST_STANDBY_DTIM
	pm_wakeup_wlan_connect_ap_attach();
	wlan_sta_scan_interval(4);
	OS_MSleep(3000);
	struct netif *nif = wlan_netif_get(WLAN_MODE_NONE);
	while (1) {
		uint32_t wakeup_event = HAL_Wakeup_GetEvent();
		uint32_t end_time;

		pm_wakeup_button_init();
		printf("wakeup_event:%x\n", wakeup_event);
		if (wakeup_event & PM_WAKEUP_SRC_WLAN) {
			/* Wait wlan event or process recived data.
			 * Need't wait 15ms after data has been processed, if
			 *  you kwow this wakeup by recived data.
			 * Or else wait 15mS to get detail wlan event( maybe
			 *  CONNECTION_LOSS or DISCONNECTED).
			 */
			OS_MSleep(15);
			end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + 180;
			/* maybe disconnect event, wait this event */
			while (((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
			        (nif && NETIF_IS_AVAILABLE(nif))) &&
			       OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
				OS_MSleep(2);
			}
		}
		OS_MSleep(2);
		while ((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
		       (!(nif && NETIF_IS_AVAILABLE(nif)))) {
			OS_MSleep(1);
		}
		pm_enter_mode(PM_MODE_STANDBY);
		printf("Exit standby mode\n\n");
		pm_wakeup_button_deinit();
		pm_wakeup_wlan_connect_ap_detach();
	}
#endif

#if TEST_HIBERNATION
	/*enetr hibernation test*/
	printf("\n\nEnter hibernantion mode, setup wakeup source button&timer\n\n");
	pm_wakeup_button_init();
	pm_wakeup_timer_init();
	pm_enter_mode(PM_MODE_HIBERNATION);
#endif

#if TEST_STANDBY_MIN_SRAM_RETENTION
	printf("\n\nEnter standby min sram retention mode, setup wakeup source button&timer, BUT NOT support now\n\n");
#if (CONFIG_CHIP_ARCH_VER == 2)
	section_header_t sh;
	register uint32_t entry;
	if (image_read(IMAGE_BOOT_ID, IMAGE_SEG_HEADER, 0, &sh,
	               IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
		printf("load section (id: %#08x) header failed\n", IMAGE_BOOT_ID);
		return -1;
	}

	if (image_check_header(&sh) == IMAGE_INVALID) {
		printf("check section (id: %#08x) header failed\n", IMAGE_BOOT_ID);
		return -1;
	}
	/* cache disable */
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_ICACHE | CCM_BUS_PERIPH_BIT_DCACHE);

	image_read(IMAGE_BOOT_ID, IMAGE_SEG_BODY, 0, (void *)sh.load_addr, sh.body_len);
	if (image_check_data(&sh, (void *)sh.load_addr, sh.body_len, NULL, 0) == IMAGE_INVALID) {
		printf("invalid boot bin body\n");
		return -1;
	}
	pm_wakeup_timer_init();
	pm_wakeup_button_init();
	entry = (uint32_t)sh.entry;
#if (defined(CONFIG_CPU_CM4F) || defined(CONFIG_CPU_CM3) || defined(CONFIG_CPU_CM33F))
	entry |= 0x1; /* set thumb bit */
#endif
	pm_standby_set_resume_entry(entry);
	pm_standby_sram_retention_only(PM_SRAM_3);
	pm_enter_mode(PM_MODE_STANDBY);
#endif
#endif
	return 0;
}
