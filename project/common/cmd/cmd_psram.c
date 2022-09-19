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

#ifdef CONFIG_PSRAM
#include <stdlib.h>
#include "sys/io.h"
#include "sys/xr_debug.h"
#include "cmd_timer.h"
#include "cmd_util.h"
#include "cmd_psram.h"

#include "driver/chip/hal_rtc.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_xip.h"
#include "sys/psram_heap.h"

#define CMD_PSRAM_EXE_AND_BENCH /* run code and read write data in psram */

#ifdef CMD_PSRAM_EXE_AND_BENCH
#define __CMD_SRAM_RODATA __psram_rodata
#define __CMD_SRAM_DATA   __psram_data
#define __CMD_SRAM_TEXT   __psram_text
#else
#define __CMD_SRAM_RODATA __sram_rodata
#define __CMD_SRAM_DATA   __sram_data
#define __CMD_SRAM_TEXT   __sram_text
#endif

#define CMD_PSRAM_DBG(fmt, arg...)                            \
	do {                                                      \
		__CMD_SRAM_RODATA static const char __fmt[] = fmt;    \
		printf(__fmt, ##arg);                                 \
	} while (0)

#define CMD_SRAM_DBG(fmt, arg...)                             \
	do {                                                      \
		__CMD_SRAM_RODATA static const char __fmt[] = fmt;    \
		printf(__fmt, ##arg);                                 \
	} while (0)

#define CMD_PSRAM_WRN(fmt, arg...)                      \
	CMD_SRAM_DBG("[WRN] "fmt, ##arg)

#define CMD_PSRAM_ERR(fmt, arg...)                      \
	do {                                                \
		CMD_SRAM_DBG("[ERR] %s():%d, "fmt,              \
		             __func__, __LINE__, ##arg);        \
	} while (0)


__CMD_SRAM_DATA
static uint32_t dmaPrintFlgCpu = 0;
__CMD_SRAM_DATA
static uint32_t dmaPrintFlgDma = 0;

#if (CONFIG_CHIP_ARCH_VER == 2)
__CMD_SRAM_DATA
#elif (CONFIG_CHIP_ARCH_VER == 3)
__sram_data
#endif
static OS_Semaphore_t dmaSem;

#if (CONFIG_CHIP_ARCH_VER == 2)
__CMD_SRAM_TEXT
#elif (CONFIG_CHIP_ARCH_VER == 3)
__sram_text
#endif
static void psram_DMARelease(void *arg)
{
	OS_SemaphoreRelease(arg);
}

__CMD_SRAM_TEXT
static int psram_dma_read_write(uint32_t write, uint32_t addr,
                                uint8_t *buf, uint32_t len, void *arg)
{
	OS_Status ret;
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;
	DMA_DataWidth dma_data_width;
	OS_Semaphore_t *dma_sem;
	uint32_t wt_saddr, wt_eaddr;
	DMA_Periph periph;
	DMA_Periph speriph = DMA_PERIPH_SRAM;
	DMA_Periph dperiph = DMA_PERIPH_SRAM;

#if (CONFIG_CHIP_ARCH_VER == 2)
	if (addr >= FLASH_XIP_START_ADDR && addr <= PSRAM_START_ADDR)
		speriph = DMA_PERIPH_FLASHC;
	else if (addr >= PSRAM_START_ADDR && addr <= PSRAM_END_ADDR)
		speriph = DMA_PERIPH_PSRAMC;
	else
		speriph = DMA_PERIPH_SRAM;
	if ((uint32_t)buf >= FLASH_XIP_START_ADDR && (uint32_t)buf <= PSRAM_START_ADDR)
		dperiph = DMA_PERIPH_FLASHC;
	else if ((uint32_t)buf >= PSRAM_START_ADDR && (uint32_t)buf <= PSRAM_END_ADDR)
		dperiph = DMA_PERIPH_PSRAMC;
	else
		dperiph = DMA_PERIPH_SRAM;

#elif (CONFIG_CHIP_ARCH_VER == 3)
	if (addr >= FLASH_XIP_START_ADDR && addr <= PSRAM_END_ADDR)
		speriph = DMA_PERIPH_FLASHC;
	else
		speriph = DMA_PERIPH_SRAM;
	if ((uint32_t)buf >= FLASH_XIP_START_ADDR && (uint32_t)buf <= PSRAM_END_ADDR)
		dperiph = DMA_PERIPH_FLASHC;
	else
		dperiph = DMA_PERIPH_SRAM;
#endif
	if (write) {
		periph  = dperiph;
		dperiph = speriph;
		speriph = periph;
	}
	dma_ch = HAL_DMA_Request();
	if (dma_ch == DMA_CHANNEL_INVALID) {
		CMD_PSRAM_ERR("%s,%d dma request faild!\n", __func__, __LINE__);
		return -1;
	}

	if (arg)
		dma_sem = arg;
	else
		dma_sem = &dmaSem;
	OS_SemaphoreCreate(dma_sem, 0, 1);
	if (!dmaPrintFlgDma) {
		CMD_PSRAM_DBG("Flg[%p]:0x%x dmaSem[%x]:0x%x\n",
		              &dmaPrintFlgDma, dmaPrintFlgDma,
		              (uint32_t)dma_sem, (uint32_t)dma_sem->handle);
		dmaPrintFlgDma = 1;
	}

	dmaParam.irqType     = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = (DMA_IRQCallback)psram_DMARelease;
	dmaParam.endArg      = dma_sem;

	if (addr & 0x1)
		dma_data_width   = DMA_DATA_WIDTH_8BIT;
	else if ((addr & 0x3) == 0x2)
		dma_data_width   = DMA_DATA_WIDTH_16BIT;
	else
		dma_data_width   = DMA_DATA_WIDTH_32BIT;

	if (write) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                                          DMA_WAIT_CYCLE_2,
		                                          DMA_BYTE_CNT_MODE_REMAIN,
		                                          dma_data_width,
		                                          DMA_BURST_LEN_1,
		                                          DMA_ADDR_MODE_INC,
		                                          dperiph,
		                                          DMA_DATA_WIDTH_8BIT,
		                                          DMA_BURST_LEN_4,
		                                          DMA_ADDR_MODE_INC,
		                                          speriph);
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                                          DMA_WAIT_CYCLE_2,
		                                          DMA_BYTE_CNT_MODE_REMAIN,
		                                          DMA_DATA_WIDTH_8BIT,
		                                          DMA_BURST_LEN_4,
		                                          DMA_ADDR_MODE_INC,
		                                          dperiph,
		                                          dma_data_width,
		                                          DMA_BURST_LEN_1,
		                                          DMA_ADDR_MODE_INC,
		                                          speriph);
	}
	HAL_DMA_Init(dma_ch, &dmaParam);

	wt_saddr = rounddown2(addr, 16);
	wt_eaddr = roundup2((uint32_t)addr + len, 16);
	//uint32_t wtIdx = HAL_Dcache_Request_WriteThroughIndex();
	//HAL_Dcache_Config_WriteThrough(wtIdx, wt_saddr, wt_eaddr);
	(void)wt_saddr;
	(void)wt_eaddr;

	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)buf, (uint32_t)addr, len);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)addr, (uint32_t)buf, len);

	ret = OS_SemaphoreWait(dma_sem, 2000);
	if (ret != OS_OK)
		CMD_PSRAM_ERR("sem wait failed: %d\n", ret);

	//HAL_Dcache_Release_WriteThroughIndex(wtIdx);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);

	OS_SemaphoreDelete(dma_sem);

	return 0;
}

__CMD_SRAM_TEXT
static int psram_dbus_cpu_read_write(uint32_t write, uint32_t addr,
                                     uint8_t *buf, uint32_t len, void *arg)
{
	if (!dmaPrintFlgCpu) {
		CMD_PSRAM_DBG("Flg[%p]:0x%x\n", &dmaPrintFlgCpu, dmaPrintFlgCpu);
		dmaPrintFlgCpu = 1;
	}

	if (write)
		memcpy((void *)addr, buf, len);
	else
		memcpy(buf, (void *)addr, len);

	return 0;
}

__CMD_SRAM_TEXT
static int psram_dbus_dma_read_write(uint32_t write, uint32_t addr,
                                     uint8_t *buf, uint32_t len, void *arg)
{
	int ret = psram_dma_read_write(write, addr, buf, len, arg);

	return ret;
}

__CMD_SRAM_DATA
psram_read_write psram_rw_op[] = {
	psram_dbus_cpu_read_write,
	psram_dbus_dma_read_write,
};

__CMD_SRAM_DATA
static volatile uint32_t psram_val = 0x12345;

/*
 * psram info
 */
__CMD_SRAM_TEXT
enum cmd_status cmd_info_exec(char *cmd)
{
	struct psram_chip *chip;

	chip = psram_open(0);
	if (!chip) {
		CMD_PSRAM_ERR("invalid chip\n");
		return CMD_STATUS_FAIL;
	}
	psram_info_dump(chip);
	psram_close(0);

	return CMD_STATUS_OK;
}

/*
 * psram run
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_run_exec(char *cmd)
{
	printf("%s read:0x%x\n", __func__, psram_val);

	return CMD_STATUS_OK;
}

/*
 * psram read <b/w/l or B/W/L> <mode> <0xadd>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_read_exec(char *cmd)
{
	int32_t cnt;
	uint32_t *add;
	char type;
	uint32_t mode;
	uint32_t val = 0;

	cnt = cmd_sscanf(cmd, "%c %d 0x%x", &type, &mode, (unsigned int *)&add);
	if (cnt != 3 || mode > 3) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	printf("%s read[%p]:", __func__, add);
	if (type == 'b' || type == 'B') {
		if (mode == 0)
			val = readb(add);
		else
			psram_rw_op[mode](0, (uint32_t)add, (uint8_t *)&val, 1, NULL);
	} else if (type == 'w' || type == 'W') {
		if (mode == 0)
			val = readw(add);
		else
			psram_rw_op[mode](0, (uint32_t)add, (uint8_t *)&val, 2, NULL);
	} else if (type == 'l' || type == 'L') {
		if (mode == 0)
			val = (unsigned int)readl(add);
		else
			psram_rw_op[mode](0, (uint32_t)add, (uint8_t *)&val, 4, NULL);
	}
	printf("0x%x\n", val);

	return CMD_STATUS_OK;
}
extern uint8_t __text_start__[];

/*
 * psram write <b/w/l or B/W/L> <mode> <0xadd> <0xval>
 * or psram write m=<mode> a=<0xadd> s=<size>, sa=<0xval>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_write_exec(char *cmd)
{
	int32_t cnt;
	uint32_t val, *add;
	char type;
	uint32_t mode;

	cnt = cmd_sscanf(cmd, "%c %d 0x%x 0x%x", &type, &mode,
	                 (unsigned int *)&add, &val);
	if (cnt != 4 || mode > 3) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (type == 'b' || type == 'B') {
		if (mode == 0)
			writeb(val, add);
		else
			psram_rw_op[mode](1, (uint32_t)add, (uint8_t *)&val, 1, NULL);
	} else if (type == 'w' || type == 'W') {
		if (mode == 0)
			writew(val, add);
		else
			psram_rw_op[mode](1, (uint32_t)add, (uint8_t *)&val, 2, NULL);
	} else if (type == 'l' || type == 'L') {
		if (mode == 0)
			writel(val, add);
		else
			psram_rw_op[mode](1, (uint32_t)add, (uint8_t *)&val, 4, NULL);
	}

	printf("%s write 0x%x to[%p]\n", __func__, val, add);

	return CMD_STATUS_OK;
}

/*
 * psram rw m=<mode> a=<0xadd> s=<size>, sa=<0xval>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_rw_exec(char *cmd)
{
	int32_t cnt;
	uint32_t mode;
	uint32_t waddr, raddr;
	int32_t size;

	cnt = cmd_sscanf(cmd, "m=%d a=0x%x s=%d sa=0x%x", &mode, &waddr, &size,
	                 &raddr);
	if (cnt != 4) {
		uint8_t *rbuf;

		cnt = cmd_sscanf(cmd, "m=%d a=0x%x s=%d", &mode, &waddr, &size);
		if (cnt != 3) {
			CMD_ERR("invalid param number %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
		rbuf = malloc(size);
		if (!rbuf) {
			CMD_ERR("malloc faild!\n");
			return CMD_STATUS_INVALID_ARG;
		}
		memset(rbuf, 0, size);
		psram_rw_op[mode](0, waddr, rbuf, size, NULL);
		print_hex_dump("", 0, 32, 1, rbuf, size, 1);
		return CMD_STATUS_OK;
	}

	psram_rw_op[mode](1, waddr, (uint8_t *)raddr, size, NULL);

	return CMD_STATUS_OK;
}

/*
 * psram membist m=<mode>
 * eg. psram membist m=0
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_membist_exec(char *cmd)
{
	extern uint8_t __psram_end__[];

	int32_t cnt;
	uint32_t mode;
	uint32_t size;
	uint32_t *add = NULL;
	uint32_t i;
	uint32_t *p;
	enum cmd_status ret = CMD_STATUS_OK;

	cnt = cmd_sscanf(cmd, "m=%d", &mode);
	if (cnt != 1 || mode > 1) {
		CMD_SRAM_DBG("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (mode == 0) {
		size = PSRAM_END_ADDR - (uint32_t)__psram_end__;
		while (!add && size >= 1024) {
			add = psram_malloc(size);
			size -= 1024;
			size &= ~(1024 - 1);
		}
		size += 1024;
	} else if (mode == 1) {
		add = (uint32_t *)PSRAM_START_ADDR;
		size = PSRAM_END_ADDR - PSRAM_START_ADDR;
	}
	size /= 4;
	CMD_SRAM_DBG("%s,%d start:%p size:%d KB\n", __func__, __LINE__, add,
	             size / (1024 / 4));
	if (!add)
		return CMD_STATUS_FAIL;
	for (i = 0, p = add; i < size; i++, p++) {
		*p = 0xA5A5A5A5;
	}
	for (i = 0, p = add; i < size; i++, p++) {
		if (*p != 0xA5A5A5A5) {
			CMD_SRAM_DBG("%s faild at[%p]:%x 0xA5A5A5A5\n",
			             __func__, p, *p);
			ret = CMD_STATUS_FAIL;
			goto out;
		}
	}
	for (i = 0, p = add; i < size; i++, p++) {
		*p = 0x5A5A5A5A;
	}
	for (i = 0, p = add; i < size; i++, p++) {
		if (*p != 0x5A5A5A5A) {
			CMD_SRAM_DBG("%s faild at[%p]:%x 0x5A5A5A5A\n",
			             __func__, p, *p);
			ret = CMD_STATUS_FAIL;
			goto out;
		}
	}
	CMD_SRAM_DBG("%s suscess, start:%p size:%d KB\n", __func__, add,
	             size / (1024 / 4));
out:
	if (mode == 0) {
		psram_free(add);
	}
	return ret;
}

__CMD_SRAM_TEXT
static void cmd_psram_bench_task(void *arg)
{
	int32_t err = 0;
	char *cmd = (char *)arg;
	uint32_t mode, start_addr, type, loop;
	uint32_t cnt;
	float throuth_mb;
	float time_use;

	cnt = cmd_sscanf(cmd, "m=%d t=0x%x n=%d", &mode, &type, &loop);
	if (cnt != 3 || mode > 1 || loop < 1) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		goto out;
	}

	time_perf_init();

	printf("\n");
	for (int _l = 0; _l < loop; _l++) {
		for (int i = 0; i < 20; i++) {
			int j;
			uint32_t bench_size = 1024 * (1 << i);
			uint32_t *buf = cmd_malloc(bench_size);
			start_addr = (uint32_t)psram_malloc(bench_size);
			if (!buf) {
				CMD_PSRAM_ERR("%s test end for malloc buff failed.\n",
				              __func__);
				if (start_addr)
					psram_free((void *)start_addr);
				break;
			}

			for (j = 0; j < bench_size / 4; j++)
				buf[j] = j;

			if (type & 0x2) {
				//HAL_Dcache_DumpMissHit();
				HAL_Dcache_CleanAll();
				time_perf_tag();
				err = psram_rw_op[mode](1, start_addr, (uint8_t *)buf,
				                        bench_size, NULL);
				HAL_Dcache_CleanAll();
				time_use = (float)time_perf_tag() / 1000.0;
				//HAL_Dcache_DumpMissHit();
				if (time_use < 0.001)
					time_use = 0.001;
				if (err) {
					CMD_PSRAM_ERR("write err!\n");
					goto next;
				} else {
					throuth_mb = bench_size * 1000 / 1024 / time_use / 1000;
					CMD_PSRAM_DBG("%s write ok, %3d", __func__,
					              bench_size / 1024);
					CMD_PSRAM_DBG(" KB use:%3.3f ms, throughput:%2.3f MB/S\n",
								  time_use, throuth_mb);
				}
			}

			if (type & 0x4) {
				for (j = 0; j < bench_size / 4; j++)
					buf[j] = 0;

				//HAL_Dcache_DumpMissHit();
				HAL_Dcache_CleanAll();
				time_perf_tag();
				err = psram_rw_op[mode](0, start_addr, (uint8_t *)buf,
				                        bench_size, NULL);
				HAL_Dcache_CleanAll();
				time_use = (float)time_perf_tag() / 1000.0;
				//HAL_Dcache_DumpMissHit();
				if (time_use < 0.001)
					time_use = 0.001;
				if (err) {
					CMD_PSRAM_ERR("read err!\n");
					goto next;
				} else {
					throuth_mb = bench_size * 1000 / 1024 / time_use / 1000;
				}

				err = 0;
				for (j = 0; j < bench_size / 4; j++) {
					if (buf[j] != j) {
						err = -1;
						break;
					}
				}
				if (err) {
					CMD_PSRAM_ERR("bench_size:%d write data err:0x%x should:"
					              "0x%x, idx:%d!\n", bench_size, buf[j], j, j);
					j = j > 16 ? j - 16 : 0;
					print_hex_dump_words((const void *)&buf[j], 256);
					goto next;
				}
				CMD_PSRAM_DBG("%s read ok,  %3d", __func__, bench_size / 1024);
				CMD_PSRAM_DBG(" KB use:%3.3f ms, throughput:%2.3f MB/S\n",
				              time_use, throuth_mb);
			}

next:
			cmd_free(buf);
			psram_free((void *)start_addr);
			if (err)
				break;
		}
	}

out:
	CMD_PSRAM_DBG("%s test end\n", __func__);
	time_perf_deinit();

	cmd_free(arg);
	OS_ThreadDelete(NULL);
}

#define CMD_ARG_LEN  64

/* psram bench <m=0/1> <t=2/4/6> <n=num>
 * m: 0:DBUS CPU, 1:DBUS DMA
 * t: 2: write, 4: read, 6:write+read
 * n:num
 * psram bench m=0 t=0x6 n=1
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_bench_exec(char *cmd)
{
	OS_Thread_t thread;
	char *param;
	uint32_t len;

	len = strlen(cmd);
	if (len >= CMD_ARG_LEN - 1) {
		CMD_PSRAM_ERR("should adjust CMD_ARG_LEN to %d\n", len);
		return CMD_STATUS_FAIL;
	}

#ifdef CONFIG_CACHE
	CMD_PSRAM_ERR("should close CACHE for test accurately!\n");
#endif
	param = cmd_malloc(CMD_ARG_LEN);
	if (!param)
		return CMD_STATUS_FAIL;
	memcpy(param, cmd, len);
	param[len + 1] = 0;

	OS_ThreadSetInvalid(&thread);
	if (OS_ThreadCreate(&thread,
	                    "",
	                    cmd_psram_bench_task,
	                    param,
	                    OS_THREAD_PRIO_APP,
	                    2 * 1024) != OS_OK) {
		CMD_PSRAM_ERR("create psram bench test task failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

#define READ_TEST_LEN  256
#define WRITE_TEST_LEN 256
__CMD_SRAM_DATA
static OS_Semaphore_t sem_wait;

struct psram_test_param {
	uint8_t  task_idx;
	uint8_t  random;
	uint8_t  task_num;
	uint8_t  mode;
	uint32_t arg;
	uint32_t time_sec;
	uint32_t addr;
	uint32_t len;
};

static uint32_t _test_exit;

__CMD_SRAM_TEXT
static int32_t _psram_check_data(uint32_t mode, uint32_t addr, uint8_t *buf,
                                 uint32_t len, OS_Semaphore_t *dma_sem)
{
	uint32_t mal_flg = 0;
	int32_t err = 0;
	uint32_t loop_num = (len + WRITE_TEST_LEN - 1) / WRITE_TEST_LEN;
	uint32_t end = addr + len;

	if (!buf) {
		buf = cmd_malloc(WRITE_TEST_LEN);
		mal_flg = 1;
	}

	for (int i = 0; i < loop_num; i++) {
		int j;

		len = end - addr > WRITE_TEST_LEN ? WRITE_TEST_LEN : end - addr;
		memset(buf, 0, len);
		if (_test_exit)
			break;
		err = psram_rw_op[mode](0, addr, buf, len, dma_sem);
		if (err) {
			CMD_ERR("psram blocks read err!\n");
			break;
		}
		for (j = 0; j < len; j++) {
			if (buf[j] != (j & 0x0FF)) {
				err = -1;
				break;
			}
		}
		if (err) {
			CMD_ERR("check:0x%02x should:0x%02x idx:%d\n", buf[j],
			        (j & 0x0FF), j);
			CMD_ERR("psram check data err! at addr:0x%x idx:%d\n",
			        addr, j);
			if (j < 64)
				j = 0;
			else
				j -= 64;
			print_hex_dump(NULL, DUMP_PREFIX_ADDRESS, 32, 1, &buf[j], 128, 1);
			break;
		} else
			;//CMD_DBG("%s check data ok! at addr:0x%x\n",
			//       __func__, addr);
		addr += WRITE_TEST_LEN;
	}
	if (mal_flg)
		cmd_free(buf);

	return err;
}

__CMD_SRAM_TEXT
static void cmd_psram_press_read_task(void *arg)
{
	int32_t   err;
	uint32_t  mode_s, mode_e;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t  random_sleep = param->random;
	uint8_t  *buf;
	struct psram_chip *chip;
	uint32_t  start_addr = param->addr;
	uint32_t  round = 0;
	uint32_t  loop_num = (param->len + READ_TEST_LEN - 1) / READ_TEST_LEN;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}
	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}

	mode_s = param->mode % 2;
	mode_e = param->mode > 1 ? (1 - mode_s) : mode_s;
	if (param->task_idx == 0) {
		buf = cmd_malloc(READ_TEST_LEN);
		for (int i = 0; i < READ_TEST_LEN; i++) {
			buf[i] = i & 0x0FF;
		}
		CMD_DBG("%s do nothing until chip prepare ok!\n", __func__);
		for (int j = 0; j < loop_num; j++) {
			err = psram_rw_op[mode_s](1, start_addr, buf, READ_TEST_LEN,
			                          &dma_sem);
			if (err) {
				CMD_ERR("%s prepare failed!\n", __func__);
				goto out;
			}
			start_addr += READ_TEST_LEN;
		}
		CMD_DBG("%s chip prepared ok!\n", __func__);

		for (int j = 1; j < param->task_num; j++)
			OS_SemaphoreRelease(&sem_wait);
		cmd_free(buf);
	} else {
		OS_SemaphoreWait(&sem_wait, OS_WAIT_FOREVER);
	}
	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}

	buf = cmd_malloc(READ_TEST_LEN);
	if (!buf) {
		CMD_ERR("%s malloc failed!\n", __func__);
		goto exit;
	}

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);

	start_addr = param->addr;
	CMD_DBG("%s id:%d random:%d start_addr:0x%x end_addr:0x%x len:%d\n",
	        __func__, param->task_idx, random_sleep, start_addr,
	        start_addr + param->len, param->len);

	while (tick_now < tick_end) {
		err = _psram_check_data(mode_e, start_addr, buf, READ_TEST_LEN,
		                        &dma_sem);
		if (err) {
			_test_exit = 1;
			goto out;
		}
		OS_MSleep(random_sleep);
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, start_addr);
			tick_print = tick_now;
		}
		start_addr += READ_TEST_LEN;
		round++;
		if (start_addr >= param->addr + param->len) {
			start_addr = param->addr;
			round = 0;
		}
	}
	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}

out:
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	cmd_free(buf);
exit:
	psram_close(chip);
fail:
	cmd_free(param);
	if (param->task_idx == 0)
		OS_SemaphoreDelete(&sem_wait);
	OS_ThreadDelete(NULL);
}

__CMD_SRAM_TEXT
static void cmd_psram_press_write_task(void *arg)
{
	int32_t   err;
	uint32_t  mode_s, mode_e;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t  random_sleep = param->random;
	uint8_t  *buf;
	uint32_t  start_addr = param->addr;
	uint32_t  round = 0;
	struct psram_chip *chip;
	uint32_t  loop_num = (param->len + WRITE_TEST_LEN - 1) / WRITE_TEST_LEN;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}

	buf = cmd_malloc(WRITE_TEST_LEN);
	if (!buf)
		goto out;

	for (int i = 0; i < WRITE_TEST_LEN; i++)
		buf[i] = i & 0x0FF;

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);
	CMD_DBG("%s id:%d random:%d start addr:0x%x end addr:0x%x len:%d\n",
	        __func__, param->task_idx, random_sleep, param->addr,
	        param->addr + param->len, param->len);

	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}

	mode_s = param->mode % 2;
	mode_e = param->mode > 1 ? (1 - mode_s) : mode_s;
	while (tick_now < tick_end) {
		if (_test_exit)
			goto out;
		err = psram_rw_op[mode_s](1, start_addr, buf, WRITE_TEST_LEN, &dma_sem);
		if (err) {
			_test_exit = 1;
			CMD_ERR("psram write err!\n");
			goto out;
		}
		start_addr += WRITE_TEST_LEN;
		if (start_addr >= param->addr + param->len) {
			start_addr = param->addr;
			round = 1;
			err = _psram_check_data(mode_e, start_addr, buf, param->len,
			                        &dma_sem);
			if (err) {
				_test_exit = 1;
				goto out;
			}
		}
		OS_MSleep(random_sleep);
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, start_addr);
			tick_print = tick_now;
		}
	}
	CMD_DBG("%s test end. round:%d start_addr:0x%x end_addr:0x%x\n", __func__,
	        round, param->addr, start_addr);

	if (!round) {
		CMD_ERR("test time too short!\n");
		goto out;
	}
	start_addr = param->addr;
	CMD_DBG("%s test checking... start_addr:0x%x loop_num:%d\n", __func__,
	        start_addr, loop_num);
	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}
	_psram_check_data(mode_e, start_addr, buf, param->len, &dma_sem);

	if (param->mode > 1) {
		HAL_Dcache_DumpMissHit();
#if (CONFIG_CHIP_ARCH_VER == 3)
		HAL_Flashc_DUMP_CbusCnt();
#endif
	}

out:
	psram_close(chip);
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	cmd_free(buf);
fail:
	cmd_free(param);
	OS_ThreadDelete(NULL);
}

/*
 * psram press r=<threads_num> s=<start_addr> l=<length> m=<mode> w=<threads_num>
 *                 s=<start_addr> l=<length> m=<mode> t=<secons>
 * mode: 0: CPU&CPU, 1:DMA&DMA, 2:CPU&DMA, 3:DMA&CPU
 * eg:
 *    psram press r=2 s=0x1404000 l=10240 m=0 w=2 s=0x1408000 l=10240 m=1 t=120
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_press_exec(char *cmd)
{
	OS_Thread_t thread;
	struct psram_test_param *param;
	uint32_t r_threads, w_threads;
	uint32_t r_len, w_len;
	uint32_t r_mode, w_mode;
	uint32_t cnt;
	uint32_t time_sec;
	uint32_t start_raddr, start_waddr;

	cnt = cmd_sscanf(cmd, "r=%d s=0x%x l=%d m=%d w=%d s=0x%x l=%d m=%d t=%d",
	                 &r_threads, &start_raddr, &r_len, &r_mode,
	                 &w_threads, &start_waddr, &w_len, &w_mode,
	                 &time_sec);
	if (cnt != 9 || r_mode > 3 || w_mode > 3) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	OS_SemaphoreCreate(&sem_wait, 0, OS_SEMAPHORE_MAX_COUNT);

	OS_MSleep(5);

	_test_exit = 0;
	for (uint32_t i = 0; i < r_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = r_threads;
		param->addr     = start_raddr;
		param->len      = r_len;
		param->mode     = r_mode;
		param->time_sec = time_sec;
		param->random   = rand() % 8 + i;
		if (!r_len || time_sec < 2) {
			CMD_ERR("%s read l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_psram_press_read_task,
		                    param,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create psram press read task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		(void)cmd_psram_press_read_task;
		OS_MSleep(2);
	}

	for (uint32_t i = 0; i < w_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = w_threads;
		param->addr = start_waddr;
		param->len = w_len;
		param->mode = w_mode;
		param->time_sec = time_sec;
		param->random = rand() % 20 + i;
		if (!w_len || time_sec < 2) {
			CMD_ERR("%s write l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_psram_press_write_task,
		                    param,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create psram press write task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		OS_MSleep(2);
	}

out:
	return CMD_STATUS_OK;
}

static int32_t cmd_psram_wtIdx = -1;
/*
 * psram malloc <size> <wr>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_malloc_exec(char *cmd)
{
	void *add;
	uint32_t size, wt;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "0x%x %d", &size, &wt);
	if (cnt != 2) {
		cnt = cmd_sscanf(cmd, "%d %d", &size, &wt);
		if (cnt != 2) {
			CMD_PSRAM_ERR("invalid argument %s\n", cmd);
			return CMD_STATUS_INVALID_ARG;
		}
	}
	size = roundup(size, 16);
	add = psram_malloc(size);
	printf("%s malloc:%p\n", __func__, add);
	if (wt) {
		cmd_psram_wtIdx = HAL_Dcache_Request_WriteThroughIndex();
		HAL_Dcache_Config_WriteThrough(cmd_psram_wtIdx, (uint32_t)add, (uint32_t)add + size);
	}

	return CMD_STATUS_OK;
}

/*
 * psram free <0xaddr> <wt>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_free_exec(char *cmd)
{
	void *add;
	uint32_t cnt, wt;

	cnt = cmd_sscanf(cmd, "0x%x %d", (unsigned int *)&add, &wt);
	if (cnt != 2) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	psram_free(add);
	printf("%s free:%p\n", __func__, add);

	if (wt)
		HAL_Dcache_Release_WriteThroughIndex(cmd_psram_wtIdx);

	return CMD_STATUS_OK;
}

/*
 * psram wt <idx> <en> <0xaddr> <size>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_wt_exec(char *cmd)
{
	void *add;
	uint32_t idx, en, size;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "%d %d 0x%x 0x%x", &idx, &en, (unsigned int *)&add,
	                 &size);
	if (cnt != 4) {
		cnt = cmd_sscanf(cmd, "%d %d 0x%x %d", &idx, &en, (unsigned int *)&add,
		                 &size);
		if (cnt != 4) {
			CMD_PSRAM_ERR("invalid argument %s\n", cmd);
			return CMD_STATUS_INVALID_ARG;
		}
	}
	if (idx > 2 || en > 1 || ((((uint32_t)add & 0xF) != 0)
	    || ((size & 0xF) != 0))) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	if (en) {
		idx = HAL_Dcache_Request_WriteThroughIndex();
		HAL_Dcache_Config_WriteThrough(idx, (uint32_t)add,
		                               (uint32_t)add + size);
	} else {
		HAL_Dcache_Release_WriteThroughIndex(idx);
	}
	CMD_SYSLOG("psram write through index = %d\n", idx);
	return CMD_STATUS_OK;
}

__CMD_SRAM_TEXT
static void cmd_psram_press_heap_task(void *arg)
{
	int32_t   err;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t  random_sleep = param->random;
	uint8_t  *buf;
	uint32_t *buf_32;
	struct psram_chip *chip;
	uint32_t  start_addr;
	uint32_t  idx = 0;
	uint32_t  section = param->arg;
	uint32_t  len = param->len / section;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}

	buf = psram_malloc(roundup(len, 4) + section * 4);
	if (!buf) {
		CMD_ERR("%s malloc failed!\n", __func__);
		goto exit;
	}
	buf_32 = (uint32_t *)&buf[roundup(len, 4)];
	for (int i = 0; i < len; i++)
		buf[i] = i & 0xff;

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);

	CMD_DBG("%s id:%d random:%d len:%d section:%d\n", __func__,
	        param->task_idx, random_sleep, len, section);

	while (tick_now < tick_end) {
		if (_test_exit)
			goto out;
		start_addr = (uint32_t)psram_malloc(len);
		if (!start_addr) {
			_test_exit = 1;
			CMD_ERR("%s malloc failed!\n", __func__);
			goto out;
		}
		buf_32[idx++] = start_addr;
		if (_test_exit)
			goto out;
		err = psram_rw_op[0](1, start_addr, buf, len, &dma_sem);
		if (err) {
			_test_exit = 1;
			CMD_ERR("%s write failed!\n", __func__);
			goto out;
		}
		err = _psram_check_data(0, start_addr, NULL, len, &dma_sem);
		if (err) {
			_test_exit = 1;
			goto out;
		}
		//OS_MSleep(random_sleep);
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, start_addr);
			tick_print = tick_now;
		}
		if (idx == section) {
			OS_MSleep(random_sleep);
			for (; idx; )
				psram_free((void *)(buf_32[--idx]));
		}
	}

out:
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	for (; idx; )
		psram_free((void *)(buf_32[--idx]));
	psram_free(buf);
exit:
	psram_close(chip);
fail:
	cmd_free(param);
	OS_ThreadDelete(NULL);
}

/*
 * psram heaptest n=<threads_num> l=<length> s=<section> s=<section> t=<secons>
 * eg:
 *    psram heaptest n=5 l=12800 s=3 s=600 t=100
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_heap_test_exec(char *cmd)
{
	OS_Thread_t thread;
	struct psram_test_param *param;
	uint32_t n_threads;
	uint32_t len, section1, section2;
	uint32_t cnt;
	uint32_t time_sec;

	cnt = cmd_sscanf(cmd, "n=%d l=%d s=%d s=%d t=%d",
	                 &n_threads, &len, &section1, &section2, &time_sec);
	if (cnt != 5 || section1 == 0 || section1 > section2) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	OS_MSleep(5);

	_test_exit = 0;
	if (n_threads > 1)
		section2 = (section2 - section1) / (n_threads - 1);
	else
		section2 = (section2 - section1);
	for (uint32_t i = 0; i < n_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = n_threads;
		param->len = len;
		param->arg = section2 * i + section1;
		param->time_sec = time_sec;
		param->random = rand() % 8 + i;
		if (!len || time_sec < 2) {
			CMD_ERR("%s read l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_psram_press_heap_task,
		                    param,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create psram prheap task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		OS_MSleep(2);
	}

out:
	return CMD_STATUS_OK;
}

__CMD_SRAM_TEXT
static void cmd_psram_chaos_task(void *arg)
{
	int32_t   err;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t  random_sleep = param->random;
	uint8_t  *buf;
	struct psram_chip *chip;
	uint32_t  start_addr = param->addr;
	uint32_t  addr = start_addr;
	uint32_t  len = param->len, rw, size, as;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}

	buf = malloc(roundup(len, 4));
	if (!buf) {
		CMD_ERR("%s malloc failed!\n", __func__);
		goto exit;
	}

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);

	CMD_DBG("%s id:%d random:%d buf:%p len:%d\n", __func__, param->task_idx,
	        random_sleep, buf, len);

	while (tick_now < tick_end) {
		if (_test_exit)
			goto out;
		if (start_addr) {
			as = rand() % 2;
			if (as)
				addr = addr + (uint32_t)((rand() % len));
			else
				addr = addr - (uint32_t)((rand() % len));
			if (addr < PSRAM_START_ADDR)
				addr = PSRAM_START_ADDR;
		} else {
			addr = PSRAM_START_ADDR
			       + (uint32_t)((rand() % (PSRAM_END_ADDR - PSRAM_START_ADDR)));
		}
		rw = rand() % 2;
		if (addr <= (uint32_t)__PSRAM_End)
			rw = 0;
		size = rand() % len;
		if (rw) {
			buf[0] = addr & 0x0f;
			for (int i = 1; i < size; i++)
				buf[i] = (buf[i - 1] + 1) & 0x0f;
		}
		err = psram_rw_op[0](rw, addr, buf, size, &dma_sem);
		//CMD_DBG("%d id:%d add:%x buf:%p rw:%d size:%d\n", __LINE__,
		//        param->task_idx, addr, buf, rw, size);
		if (err) {
			_test_exit = 1;
			CMD_ERR("%s,%d add:%x rw:%d size:%d failed!\n", __func__, __LINE__,
			        addr, rw, size);
			goto out;
		}
		OS_MSleep(random_sleep);
		if (memcmp((void *)addr, buf, size)) {
			//_test_exit = 1;
			CMD_ERR("%s,%d id:%d add:%x rw:%d size:%d failed!\n", __func__,
			        __LINE__, param->task_idx, addr, rw, size);
			print_hex_dump(NULL, DUMP_PREFIX_ADDRESS, 32, 1, (void *)addr,
			               size, 0);
			print_hex_dump(NULL, DUMP_PREFIX_ADDRESS, 32, 1, buf, size, 0);
			//goto out;
		}
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, addr);
			tick_print = tick_now;
		}
	}

out:
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	free(buf);
exit:
	psram_close(chip);
fail:
	cmd_free(param);
	OS_ThreadDelete(NULL);
}

/*
 * psram chaos n=<threads_num> s=<start_addr> r=<random_len> t=<secons>
 * eg:
 *    psram chaos n=5 s=0x0 r=32 t=100
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_chaos_exec(char *cmd)
{
	OS_Thread_t thread;
	struct psram_test_param *param;
	uint32_t n_threads;
	uint32_t random_len, addr;
	uint32_t cnt;
	uint32_t time_sec;

	cnt = cmd_sscanf(cmd, "n=%d s=0x%x r=%d t=%d", &n_threads, &addr,
	                 &random_len, &time_sec);
	if (cnt != 4 || random_len < 1) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	OS_MSleep(5);

	_test_exit = 0;
	for (uint32_t i = 0; i < n_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = n_threads;
		param->addr = addr;
		param->len = random_len;
		param->time_sec = time_sec;
		param->random = rand() % 8 + i;
		if (!random_len || time_sec < 2) {
			CMD_ERR("%s read l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_psram_chaos_task,
		                    param,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create psram prheap task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		OS_MSleep(2);
	}

out:
	return CMD_STATUS_OK;
}

static const struct cmd_data g_psram_cmds[] = {
	{ "info",       cmd_info_exec},
	{ "run",        cmd_psram_run_exec },
	{ "read",       cmd_psram_read_exec },
	{ "write",      cmd_psram_write_exec },
	{ "rw",         cmd_psram_rw_exec },
	{ "membist",    cmd_psram_membist_exec },
	{ "bench",      cmd_psram_bench_exec },
	{ "press",      cmd_psram_press_exec },
	{ "malloc",     cmd_psram_malloc_exec },
	{ "free",       cmd_psram_free_exec },
	{ "wt",         cmd_psram_wt_exec },
	{ "heaptest",   cmd_psram_heap_test_exec },
	{ "chaos",      cmd_psram_chaos_exec },
};

enum cmd_status cmd_psram_exec(char *cmd)
{
	return cmd_exec(cmd, g_psram_cmds, cmd_nitems(g_psram_cmds));
}
#endif
