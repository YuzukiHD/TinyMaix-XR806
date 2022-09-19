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

#ifndef _CMD_H_
#define _CMD_H_

#include "common/cmd/cmd_util.h"

#include "common/cmd/cmd_echo.h"
#include "common/cmd/cmd_mem.h"
#include "common/cmd/cmd_heap.h"
#include "common/cmd/cmd_thread.h"
#include "common/cmd/cmd_upgrade.h"
#include "common/cmd/cmd_sysinfo.h"
#include "common/cmd/cmd_hexdump.h"

#include "common/cmd/cmd_gpio.h"
#include "common/cmd/cmd_clock.h"
#include "common/cmd/cmd_uart.h"
#include "common/cmd/cmd_timer.h"
#include "common/cmd/cmd_wdg.h"
#include "common/cmd/cmd_rtc.h"
#include "common/cmd/cmd_irrx.h"
#include "common/cmd/cmd_irtx.h"
#include "common/cmd/cmd_ce.h"
#include "common/cmd/cmd_i2c.h"
#include "common/cmd/cmd_i2s.h"
#include "common/cmd/cmd_codec.h"
#include "common/cmd/cmd_adc.h"
#include "common/cmd/cmd_flash.h"
#include "common/cmd/cmd_pwm.h"
#include "common/cmd/cmd_sd.h"
#include "common/cmd/cmd_efpg.h"
#include "common/cmd/cmd_audio.h"
#include "common/cmd/cmd_cache.h"
#include "common/cmd/cmd_psram.h"
#include "common/cmd/cmd_spi.h"
#include "common/cmd/cmd_camera.h"
#if (CONFIG_CHIP_ARCH_VER == 3)
#include "common/cmd/cmd_lpuart.h"
#include "common/cmd/cmd_scr.h"
#include "common/cmd/cmd_keyboard.h"
#include "common/cmd/cmd_flashc_crypto.h"
#ifdef CONFIG_TRUSTZONE
#include "common/cmd/cmd_trustzone.h"
#endif
#endif

#include "common/cmd/cmd_auddbg.h"
#include "common/cmd/cmd_pm.h"
#include "common/cmd/cmd_ota.h"
#include "common/cmd/cmd_cedarx.h"
#include "common/cmd/cmd_psensor.h"

#include "common/cmd/cmd_oled.h"
#include "common/cmd/cmd_rf.h"

#if PRJCONF_NET_EN
#include "common/cmd/cmd_wlan.h"
#include "common/cmd/cmd_ifconfig.h"
#include "common/cmd/cmd_smart_config.h"
#include "common/cmd/cmd_airkiss.h"
#include "common/cmd/cmd_smartlink.h"
#include "common/cmd/cmd_voice_print.h"
#include "common/cmd/cmd_blink.h"
#include "common/cmd/cmd_ping.h"
#include "common/cmd/cmd_iperf.h"
#include "common/cmd/cmd_sntp.h"
#include "common/cmd/cmd_httpc.h"
#include "common/cmd/cmd_httpd.h"
#include "common/cmd/tls/cmd_tls.h"
#include "common/cmd/cmd_mqtt.h"
#include "common/cmd/cmd_dhcpd.h"
#include "common/cmd/cmd_nopoll.h"
#include "common/cmd/cmd_lws.h"

#include "common/cmd/cmd_broadcast.h"
#include "common/cmd/cmd_arp.h"
#include "common/cmd/cmd_lmac.h"
#include "common/cmd/cmd_wlancmd.h"
#endif /* PRJCONF_NET_EN */
#include "common/cmd/cmd_etf.h"

/* Middleware */
#include "common/cmd/cmd_fs.h"
#include "common/cmd/cmd_xz.h"
#include "common/cmd/cmd_console.h"
#include "common/cmd/cmd_json.h"

/* blec */
#ifdef PRJCONF_BLE_EN
#include "common/cmd/cmd_blec.h"
#ifdef CONFIG_BLEHOST
#include "common/cmd/cmd_ble.h"
#include "common/cmd/cmd_gatt.h"
#include "common/cmd/cmd_mesh.h"
#include "common/cmd/cmd_adv.h"

#endif /* CONFIG_BLEHOST */
#endif
#include "common/cmd/cmd_btsnoop.h"
#include "common/cmd/cmd_ble_etf.h"

#ifdef CONFIG_BENCH_MARK
#include "common/cmd/cmd_bench_mark.h"
#endif

#ifdef CONFIG_SETTINGS
#include "common/cmd/cmd_settings.h"
#endif

#include "common/cmd/cmd_rom.h"

#endif /* _CMD_H_ */
