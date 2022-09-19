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

#include "image/flash.h"
#include "driver/chip/hal_timer.h"
#include "driver/chip/hal_xip.h"
#include "cmd_debug.h"
#include "cmd_timer.h"
#include "cmd_util.h"
#include "cmd_flash.h"
#include "driver/chip/hal_flash.h"
#include "driver/hal_dev.h"
#include "driver/chip/hal_sysctl.h"
#include "../board/board_common.h"

#define MFLASH 0

static enum cmd_status cmd_flash_start_exec(char *cmd)
{
	if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
		CMD_ERR("flash driver open failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_flash_stop_exec(char *cmd)
{
	/* deinie driver */
	if (HAL_Flash_Close(MFLASH) != HAL_OK) {
		CMD_ERR("flash driver close failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

#define FLASH_TEST_BUF_SIZE (0x100)

/* drv flash erase s=4kb a=0x110000
 */
static enum cmd_status cmd_flash_erase_exec(char *cmd)
{
	int32_t cnt;
	char size_str[8];
	uint32_t addr;
	FlashEraseMode size_type;
	int32_t size;
	uint8_t buf[FLASH_TEST_BUF_SIZE];

	/* get param */
	cnt = cmd_sscanf(cmd, "s=%7s a=0x%x", size_str, &addr);

	/* check param */
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(size_str, "chip") == 0) {
		size_type = FLASH_ERASE_CHIP;
		size = 0;
	} else if (cmd_strcmp(size_str, "64kb") == 0) {
		size = 0x10000;
		size_type = FLASH_ERASE_64KB;
	} else if (cmd_strcmp(size_str, "32kb") == 0) {
		size_type = FLASH_ERASE_32KB;
		size = 0x8000;
	} else if (cmd_strcmp(size_str, "4kb") == 0) {
		size = 0x1000;
		size_type = FLASH_ERASE_4KB;
	} else {
		CMD_ERR("invalid size %s\n", size_str);
		return CMD_STATUS_INVALID_ARG;
	}

	/* erase */
	//HAL_Flash_MemoryOf(MFLASH, size_type, addr, &addr);
	if (HAL_Flash_Erase(MFLASH, size_type, addr, 1) != HAL_OK) {
		CMD_ERR("flash erase failed\n");
		return CMD_STATUS_FAIL;
	}

	while (size > 0) {
		int32_t tmp_size =
		    (size < FLASH_TEST_BUF_SIZE) ? size : FLASH_TEST_BUF_SIZE;

		//CMD_DBG("tmp_size: %d\n", tmp_size);

		if (HAL_Flash_Read(MFLASH, addr, buf, tmp_size) != HAL_OK) {
			CMD_ERR("flash read failed\n");
			return CMD_STATUS_FAIL;
		}

		size -= tmp_size;
		addr += tmp_size;

		while (--tmp_size >= 0) {
			if ((uint8_t)(~(buf[tmp_size])) != 0) {
				CMD_ERR("flash erase failed: read data from flash != 0xFF,"
				        " ~data = 0x%x, tmp_size = %d\n",
				        (uint8_t)(~(buf[tmp_size])), tmp_size);
				return CMD_STATUS_FAIL;
			}
		}

	}

	return CMD_STATUS_OK;
}

/* drv flash write a=0x110000 s=32
 *  or drv flash write a=0x110000 s=32 sa=0x120000
 *  or drv flash write a=0x110000 s=32 sa=0x1400000
 */
static enum cmd_status cmd_flash_write_exec(char *cmd)
{
	uint32_t cnt;
	uint32_t waddr, raddr = -1;
	int32_t size;
	uint8_t *wbuf, *wbuf_bk;
	uint8_t *rbuf;

	/* get param */
	cnt = cmd_sscanf(cmd, "a=0x%x s=%d sa=0x%x", &waddr, &size, &raddr);
	/* check param */
	if (cnt != 3) {
		cnt = cmd_sscanf(cmd, "a=0x%x s=%d", &waddr, &size);
		if (cnt != 2) {
			CMD_ERR("invalid param number %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	if (size == 0) {
		// special handle
		wbuf = cmd_malloc(size+5);
		if (wbuf == NULL) {
			CMD_ERR("no memory\n");
			return CMD_STATUS_FAIL;
		}
		memset(wbuf, 0, (size+5));
		wbuf[0] = 0x1;

		if (HAL_Flash_Write(MFLASH, waddr, wbuf, size) != HAL_OK) {
			CMD_ERR("flash write failed\n");
		}
		cmd_free(wbuf);
		return CMD_STATUS_ACKED;
	}

	wbuf = cmd_malloc(size);
	if (wbuf == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}
	wbuf_bk = wbuf;

	rbuf = cmd_malloc(size);
	if (rbuf == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(wbuf);
		return CMD_STATUS_FAIL;
	}

	if (raddr == -1) {
		cmd_raw_mode_enable();
		cmd_raw_mode_read(wbuf, size, 30000);
	} else if ((raddr >= PSRAM_START_ADDR) && (raddr < PSRAM_END_ADDR)) {
#ifndef CONFIG_PSRAM
		CMD_ERR("open psram when write source data from psram!\n");
		cmd_free(wbuf);
		cmd_free(rbuf);
		return CMD_STATUS_FAIL;
#endif
		wbuf = (uint8_t *)raddr;
	} else {
#ifndef CONFIG_XIP
		CMD_ERR("open xip when write source data from xip!\n");
		cmd_free(wbuf);
		cmd_free(rbuf);
		return CMD_STATUS_FAIL;
#endif
		wbuf = (uint8_t *)raddr;
	}

	/* write */
	if (HAL_Flash_Write(MFLASH, waddr, wbuf, size) != HAL_OK) {
		CMD_ERR("flash write failed\n");
	}

	if (HAL_Flash_Read(MFLASH, waddr, rbuf, size) != HAL_OK) {
		CMD_ERR("flash read failed\n");
	}

	if (raddr == -1) {
		cmd_raw_mode_write(rbuf, size);
		cmd_raw_mode_disable();
	} else {
		print_hex_dump_bytes(rbuf, size);
	}

	cmd_free(wbuf_bk);
	cmd_free(rbuf);

	return CMD_STATUS_ACKED;
}

/* drv flash read a=0x10000 s=32
 */
static enum cmd_status cmd_flash_read_exec(char *cmd)
{
	int32_t cnt;
	uint8_t *buf;
	uint32_t addr;
	uint32_t size;
	char *pre_str = "read buf:";

	/* get param */
	cnt = cmd_sscanf(cmd, "a=0x%x s=%u", &addr, &size);

	/* check param */
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	if (size == 0) {
		// special handle
		buf = cmd_malloc(size+5);
		if (buf == NULL) {
			CMD_ERR("no memory\n");
			cmd_free(buf);
			return CMD_STATUS_FAIL;
		}

		if (HAL_Flash_Read(MFLASH, addr, buf, size) != HAL_OK) {
			CMD_ERR("spi driver read failed\n");
		}
		cmd_free(buf);
		return CMD_STATUS_ACKED;
	}

	/* read */
	buf = cmd_malloc(size);
	if (buf == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(buf);
		return CMD_STATUS_FAIL;
	}

	if (HAL_Flash_Read(MFLASH, addr, buf, size) != HAL_OK) {
		CMD_ERR("spi driver read failed\n");
		cmd_free(buf);
		return CMD_STATUS_FAIL;
	}

	cmd_print_uint8_array(buf, size);
	cmd_raw_mode_write((uint8_t *)pre_str, strlen(pre_str));
	cmd_raw_mode_write(buf, size);
	cmd_free(buf);
	return CMD_STATUS_ACKED;
}

/* drv flash overwrite a=0x110000 s=32
 */
static enum cmd_status cmd_flash_overwrite_exec(char *cmd)
{
	uint32_t cnt;
	uint32_t addr;
	int32_t size;
	uint8_t *wbuf;
	int ret;

	/* get param */
	cnt = cmd_sscanf(cmd, "a=0x%x s=%d", &addr, &size);

	/* check param */
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	wbuf = cmd_malloc(size);
	if (wbuf == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	cmd_raw_mode_enable();
	cmd_raw_mode_read(wbuf, size, 30000);

	/* write */
	if ((ret = HAL_Flash_Overwrite(MFLASH, addr, wbuf, size)) != HAL_OK) {
		CMD_ERR("flash write failed: %d\n", ret);
	}

	if ((ret = HAL_Flash_Check(MFLASH, addr, wbuf, size)) != 0) {
		CMD_ERR("flash write not success %d\n", ret);
	}

	cmd_raw_mode_disable();

	cmd_free(wbuf);

	return CMD_STATUS_ACKED;
}

int8_t flash_test_rdwr(uint32_t flash, uint32_t addr, uint32_t max_size, uint8_t *buf)
{
	uint32_t len = max_size;
	uint8_t *tmp1, *tmp2;
	static uint8_t tmpdata = 0x5a;

	tmp1 = buf;
	tmp2 = buf + len;

	cmd_memset(tmp1, tmpdata, len);
	cmd_memset(tmp2, 0, len);
	if (flash_erase(flash, addr, max_size) != 0) {
		printf("Flash Erase failure!(addr: 0x%08x)\r\n", addr);
		return -1;
	}

	if (flash_write(flash, addr, tmp1, len) != len) {
		printf("Flash Write failure!(addr: 0x%08x)\r\n", addr);
		return -1;
	}
	if (flash_read(flash, addr, tmp2, len) != len)    {
		printf("Flash Read failure!(addr: 0x%08x)\r\n", addr);
		return -1;
	}
	if (cmd_memcmp(tmp2, tmp1, len) == 0) {
		tmpdata = ~tmpdata;
		return 0;
	} else {
		printf("Flash w&r check res failure!(addr: 0x%08x)\r\n", addr);
#if 0
		for (uint32_t i = 0x0; i < 4096; i++) {
			if ((i%16) == 0 && (i != 0))
				printf("\n");
			if (tmp1[i] != tmp2[i])
				printf("\n*****error data pos:%d,tmp1:0x%02x,tmp2:0x%02x*****\n",
				       i, tmp1[i], tmp2[i]);
			else
				printf("%02x ", tmp2[i]);
		}
#endif
		return -1;
	}
}

int8_t flash_test_read(uint32_t flash, uint32_t addr, uint32_t len, uint8_t *buf)
{
	cmd_memset(buf, 0, len);
	if (flash_read(flash, addr, buf, len) != len)
		return -1;

	return 0;
}

#if (defined FLASH_XIP_OPT_WRITE) && (defined FLASH_XIP_OPT_ERASR)
#define USER_PRIO       4
#define USER_STACKSIZE  2048

static OS_Thread_t g_user_thread;

void timer0_callback(void *arg)
{
	static uint32_t ti0 = 0;

	if (++ti0 >= 1000) {
		ti0 = 0;
		printf("%s,%d fn:%p\n", __func__, __LINE__, timer0_callback);
	}
}

void timer1_callback(void *arg)
{
	static uint32_t ti1 = 0;

	if (++ti1 >= 1000) {
		ti1 = 0;
		printf("%s,%d fn:%p\n", __func__, __LINE__, timer1_callback);
	}
}

static HAL_Status timer_inital(TIMER_ID id)
{
	TIMER_InitParam param;

	param.cfg = HAL_TIMER_MakeInitCfg(TIMER_MODE_REPEAT, TIMER_CLK_SRC_HFCLK,
	                                  TIMER_CLK_PRESCALER_4); /* 10M */
	param.period = 10000;
	param.isEnableIRQ = 1;
	if (id == TIMER0_ID)
		param.callback = timer0_callback;
	else
		param.callback = timer1_callback;
	param.arg = (void *)id;
	if (HAL_TIMER_Init((TIMER_ID)id, &param) != HAL_OK) {
		return HAL_ERROR;
	}
	HAL_TIMER_Start((TIMER_ID)id);

	return HAL_OK;
}

#define FLASH_TEST_START_ADD 0x180000 /* 1.5M  */
#define FLASH_TEST_END_ADD   0x200000 /* 2M    */
#define FLASH_TEST_ADD_STEP  0x1000   /* 4K    */
#define FLASH_TEST_LOOP_CNT  1000

void optimize_press_task(void *arg)
{
	uint32_t erase_cnt = 0;
	uint32_t write_add = FLASH_TEST_START_ADD, read_add = 0;
	int status = 0;
	uint32_t cnt = 0, read_len = 1;
	uint8_t *buf = cmd_malloc(FLASH_TEST_ADD_STEP * 2 + 16);

	printf("run user task per 100ms...%d\n", erase_cnt);
	while (1) {
		status = flash_test_rdwr(0, write_add, FLASH_TEST_ADD_STEP, buf);
		if (status < 0) {
			printf("\nstop..addr: 0x%08x, earse_cnt: %d\n",
			       write_add, erase_cnt);
			break;
		}

		if (read_len > FLASH_TEST_ADD_STEP)
			read_len = 1;
		status = flash_test_read(0, read_add, read_len, buf);
		if (status < 0) {
			printf("\nstop..addr: 0x%08x, earse_cnt: %d\n",
			       read_add, erase_cnt);
			break;
		}
		if (read_add < FLASH_TEST_END_ADD) {
			read_add += read_len;
		} else {
			printf("addr: 0x%08x, read times reached max....\n",
			       read_add);
			read_add = 0;
		}
		read_len++;

		if (++erase_cnt >= FLASH_TEST_LOOP_CNT) {
			erase_cnt = 0;
			if (write_add < FLASH_TEST_END_ADD) {
				write_add += FLASH_TEST_ADD_STEP;
			} else {
				printf("addr: 0x%08x, Erase times reached max....\n",
				       write_add);
				write_add = FLASH_TEST_START_ADD;
			}
		}
		OS_MSleep(50);
		++cnt;
		if ((cnt % 200) == 1)
			printf("%s cnt:%d radd:%x wadd:%x\n",
			       __func__, cnt, read_add, write_add);
	}
	printf("%s exit\n", __func__);
	cmd_free(buf);
	HAL_TIMER_Stop(TIMER0_ID);
	OS_ThreadDelete(NULL);
}

static enum cmd_status cmd_flash_optimize_exec(char *cmd)
{
	uint32_t cnt, tn;

	printf("%s\n", __func__);
	cnt = cmd_sscanf(cmd, "%d", &tn);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (tn > 0) {
		timer_inital(TIMER0_ID);
		if (tn >= 2)
			timer_inital(TIMER1_ID);
	}

	if (OS_ThreadCreate(&g_user_thread,
	                    "flash_optm",
	                    optimize_press_task,
	                    NULL,
	                    USER_PRIO,
	                    USER_STACKSIZE) != OS_OK)    {
		printf("create user task failed!\r\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}
#endif

/*
 *FLASH_READ_NORMAL_MODE    = 1 << 0,   ->1
 *FLASH_READ_FAST_MODE      = 1 << 1,   ->2
 *FLASH_READ_DUAL_O_MODE    = 1 << 2,   ->4
 *FLASH_READ_DUAL_IO_MODE   = 1 << 3,   ->8
 *FLASH_READ_QUAD_O_MODE    = 1 << 4,   ->16
 *FLASH_READ_QUAD_IO_MODE   = 1 << 5,   ->32
 *FLASH_READ_QPI_MODE       = 1 << 6,   ->64
 */
static enum cmd_status cmd_flash_set_rmode_exec(char *cmd)
{
	uint32_t cnt;
	uint32_t rmod;
	char rmod_str[10];

	/* get param */
	cnt = cmd_sscanf(cmd, "rmod=%9s", rmod_str);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(rmod_str, "normal") == 0) {
		rmod = 0x1;
	} else if (cmd_strcmp(rmod_str, "fast") == 0) {
		rmod = 0x2;
	} else if (cmd_strcmp(rmod_str, "dual_o") == 0) {
		rmod = 0x4;
	} else if (cmd_strcmp(rmod_str, "dual_io") == 0) {
		rmod = 0x8;
	} else if (cmd_strcmp(rmod_str, "quad_o") == 0) {
		rmod = 0x10;//16
	} else if (cmd_strcmp(rmod_str, "quad_io") == 0) {
		rmod = 0x20;//32
	} else if (cmd_strcmp(rmod_str, "qpi") == 0) {
		rmod = 0x40;//64
	} else {
		CMD_ERR("invalid rmod %s\n", rmod_str);
		return CMD_STATUS_INVALID_ARG;
	}

	if (HAL_Flash_Ioctl(MFLASH, FLASH_SET_READ_MODE, rmod) != HAL_OK) {
		CMD_ERR("set flash read mode failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

/*
 *FLASH_PAGEPROGRAM         = 1 << 0,    ->1
 *FLASH_QUAD_PAGEPROGRAM    = 1 << 1,    ->2
 */
static enum cmd_status cmd_flash_set_program_mode_exec(char *cmd)
{
	/* deinie driver */
	uint32_t cnt;
	uint32_t wmod;
	char wmod_str[10];

	/* get param */
	cnt = cmd_sscanf(cmd, "wmod=%9s", wmod_str);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(wmod_str, "page") == 0) {
		wmod = 0x1;
	} else if (cmd_strcmp(wmod_str, "quad_page") == 0) {
		wmod = 0x2;
	} else {
		CMD_ERR("invalid wmod %s\n", wmod_str);
		return CMD_STATUS_INVALID_ARG;
	}

	if (HAL_Flash_Ioctl(MFLASH, FLASH_SET_PAGEPROGRAM_MODE, wmod) != HAL_OK) {
		CMD_ERR("set flash pageprogram mode failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

/*
 * flash press s=0x100000 e=0x1FF000 b=8192 l=50
 *
 * at least run one time when close xip for this test
 */
#define FLAH_WR_PRESS_DEBUG_ON             1
#define FLAH_WR_PRESS_TEST_DEBUG_SIZE     (500 * 1024)

static void cmd_flash_press_test_task(void *arg)
{
	uint32_t ret;
	uint32_t addr, size, len, pos, write_pos;
	uint32_t test_cnt;
	uint8_t *wbuf, *rbuf;
	uint32_t cnt, saddr, eaddr, bs, loop;

	/* get param */
	cnt = cmd_sscanf((char *)arg, "s=0x%x e=0x%x b=%d l=%d",
	                 &saddr, &eaddr, &bs, &loop);
	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		goto fail;
	}
	if ((eaddr - saddr) < bs) {
		CMD_ERR("(end addr - start addr) must > buf size !\n");
		goto fail;
	}
#if FLAH_WR_PRESS_DEBUG_ON
	uint32_t debug_size, debug_size_unit;

	debug_size = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
	debug_size_unit = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
#endif

	len = bs;
	addr = saddr;
	size = eaddr - addr;

	test_cnt = 0;
	pos = len;
	write_pos = 0;

	wbuf = cmd_malloc(len);
	if (wbuf == NULL) {
		CMD_ERR("no memory, malloc size %d fail\n", len);
		goto fail;
	}
	memset(wbuf, 0, len);

	rbuf = cmd_malloc(len);
	if (rbuf == NULL) {
		CMD_ERR("no memory, malloc size %d fail\n", len);
		cmd_free(wbuf);
		goto fail;
	}
	memset(rbuf, 0, len);

	for (int i = 0; i < len; ++i) {
		wbuf[i] = ((i) & 0xff);
	}

	CMD_DBG("flash erase start\n");

	if (flash_erase(MFLASH, addr, size)) {
		CMD_ERR("flash erase err, add: 0x%x, size: 0x%x\n", addr, size);
		goto out;
	}
	CMD_DBG("flash erase ok, add: 0x%x, size: 0x%x\n", addr, size);

	while (1) {
		ret = flash_write(MFLASH, addr + write_pos, wbuf, pos);
		if (ret != pos) {
			CMD_ERR("flash write err len:%d pos:%d\n", len, pos);
			break;
		}
		ret = flash_read(MFLASH, addr + write_pos, rbuf, pos);
		if (ret != pos) {
			CMD_ERR("flash read err len:%d pos:%d\n", len, pos);
			break;
		}

		for (int j = 0; j < pos; ++j) {
			if (rbuf[j] != ((j) & 0xff)) {
				CMD_ERR("flash read check err len:%d pos:%d + %d\n",
				        len, write_pos, j);
				CMD_ERR("j:%d, now pos:%d\n", j, pos);
				if (j < 32)
					j = 0;
				else
					j -= 32;
				print_hex_dump(NULL, DUMP_PREFIX_ADDRESS, 32, 1,
				               &rbuf[j], 64, 0);
				ret = -1;
				break;
			}
		}
		if (ret != pos)
			break;
		memset(rbuf, 0, len);

		write_pos += pos;
#if FLAH_WR_PRESS_DEBUG_ON
		if (write_pos >= debug_size) {
			CMD_DBG("write data:%dKB\n", write_pos / 1024);
			debug_size += debug_size_unit;
		}
#endif
		if ((size - write_pos) > len)
			pos = len;
		else
			pos = size - write_pos;

		if (pos == 0) {
			CMD_DBG("flash write & read over, addr:0x%x ~ 0x%x, len:%dKB\n",
			        addr, addr + write_pos, size / 1024);

			if (++test_cnt >= loop) {
				break;
			} else {
				CMD_DBG("flash test end, run test count:%d, should run:%d\n",
				        test_cnt, loop);

				CMD_DBG("flash erase start\n");
				if (flash_erase(MFLASH, addr, size)) {
					CMD_ERR("flash erase err, add: 0x%x, size: 0x%x\n", addr,
					        size);
					//break;
				} else {
					CMD_DBG("flash erase ok, add: 0x%x, size: 0x%x\n", addr,
					        size);
				}
				pos = len;
				write_pos = 0;
#if FLAH_WR_PRESS_DEBUG_ON
				debug_size = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
#endif
			}
		}
	}

	CMD_DBG("flash test end, run test count:%d, should run:%d\n", test_cnt,
	        loop);

out:
	cmd_free(wbuf);
	cmd_free(rbuf);
fail:
	OS_ThreadDelete(NULL);
}

static void cmd_flash_bench_task(void *arg)
{
	int32_t err = 0;
	char *cmd = (char *)arg;
	uint32_t mode, start_addr, loop;
	uint32_t cnt, erase_kb = 4;
	float throuth_mb;
	float time_use;
	FlashEraseMode size_type = FLASH_ERASE_4KB;

	cnt = cmd_sscanf(cmd, "m=0x%x a=0x%x n=%d", &mode, &start_addr, &loop);
	if (cnt != 3 || loop < 1) {
		CMD_ERR("invalid argument %s\n", cmd);
		goto out;
	}

	time_perf_init();

	printf("\n");
	for (int _l = 0; _l < loop; _l++) {
		for (int i = 0; i < 20; i++) {
			int j;
			uint32_t bench_size = 1024 * (1 << i);
			uint32_t *buf = cmd_malloc(bench_size);

			if (!buf) {
				CMD_DBG("%s test end for malloc buff failed.\n", __func__);
				break;
			}

			for (j = 0; j < bench_size / 4; j++)
				buf[j] = j;

			if (mode & 0x1) {
				cnt = 1;
				if (bench_size <= 4*1024) {
					size_type = FLASH_ERASE_4KB;
					erase_kb = 4;
				} else if (bench_size <= 32*1024) {
					size_type = FLASH_ERASE_32KB;
					erase_kb = 32;
				} else {
					size_type = FLASH_ERASE_64KB;
					cnt = DIV_ROUND_UP(bench_size, 64*1024);
					erase_kb = 64 * cnt;
				}

				time_perf_tag();
				err = HAL_Flash_Erase(MFLASH, size_type, start_addr, cnt);
				time_use = (float)time_perf_tag() / 1000.0;
				if (time_use < 0.001)
					time_use = 0.001;
				if (err) {
					CMD_ERR("erase err!\n");
					goto next;
				} else {
					throuth_mb = erase_kb * 1024 / time_use / 1000;
					printf("%s erase ok, %3d", __func__, erase_kb);
					printf(" KB use:%3.3f ms, throughput:%2.3f MB/S\n",
						   time_use, throuth_mb);
				}
			}

			if (mode & 0x2) {
				time_perf_tag();
				err = HAL_Flash_Write(MFLASH, start_addr,
				                      (const unsigned char *)buf, bench_size);
				time_use = (float)time_perf_tag() / 1000.0;
				if (time_use < 0.001)
					time_use = 0.001;
				if (err) {
					CMD_ERR("write err!\n");
					goto next;
				} else {
					throuth_mb = bench_size * 1000 / 1024 / time_use / 1000;
					printf("%s write ok, %3d", __func__, bench_size / 1024);
					printf(" KB use:%3.3f ms, throughput:%2.3f MB/S\n",
						   time_use, throuth_mb);
				}
			}

			if (mode & 0x4) {
				for (j = 0; j < bench_size / 4; j++)
					buf[j] = 0;

				time_perf_tag();
				err = HAL_Flash_Read(MFLASH, start_addr, (uint8_t *)buf,
				                     bench_size);
				time_use = (float)time_perf_tag() / 1000.0;
				if (time_use < 0.001)
					time_use = 0.001;
				if (err) {
					CMD_ERR("read err!\n");
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
					CMD_ERR("bench_size:%d write data err:0x%x should:0x%x,"
					        " idx:%d!\n", bench_size, buf[j], j, j);
					j = j > 16 ? j - 16 : 0;
					print_hex_dump_bytes((const void *)&buf[j], 256);
					goto next;
				}
				printf("%s read ok,  %3d", __func__, bench_size / 1024);
				printf(" KB use:%3.3f ms, throughput:%2.3f MB/S\n",
					   time_use, throuth_mb);
			}

next:
			cmd_free(buf);
			if (err)
				break;
		}
	}

out:
	CMD_DBG("%s test end\n", __func__);
	time_perf_deinit();

	cmd_free(arg);
	OS_ThreadDelete(NULL);
}

#define CMD_ARG_LEN  64

/* flash bench m=<mode> <a=0xaddr> n=<loop_num>
 *  <mode>: 1: erase, 2: write, 4: read, other is add
 * flash bench m=0x7 a=0x100000 n=1
 */
static enum cmd_status cmd_flash_bench_exec(char *cmd)
{
	OS_Thread_t thread;
	char *param;
	uint32_t len;

	len = strlen(cmd);
	if (len >= CMD_ARG_LEN - 1) {
		CMD_ERR("should adjust CMD_ARG_LEN to %d\n", len);
		return CMD_STATUS_FAIL;
	}

	param = cmd_malloc(CMD_ARG_LEN);
	if (!param)
		return CMD_STATUS_FAIL;
	memcpy(param, cmd, len);
	param[len + 1] = 0;

	OS_ThreadSetInvalid(&thread);
	if (OS_ThreadCreate(&thread,
	                    "",
	                    cmd_flash_bench_task,
	                    param,
	                    OS_THREAD_PRIO_APP,
	                    2 * 1024) != OS_OK) {
		CMD_ERR("create flash bench test task failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

/*
 * flash press s=0x100000 e=0x1FF000 b=8192 l=50
 */
static enum cmd_status cmd_flash_press_exec(char *cmd)
{
	uint32_t cnt, saddr, eaddr, bs, loop;

	/* get param */
	cnt = cmd_sscanf(cmd, "s=0x%x e=0x%x b=%d l=%d", &saddr, &eaddr, &bs,
	                 &loop);

	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	OS_Thread_t thread;

	OS_ThreadSetInvalid(&thread);
	if (OS_ThreadCreate(&thread,
	                    "",
	                    cmd_flash_press_test_task,
	                    cmd,
	                    OS_THREAD_PRIO_APP,
	                    2 * 1024) != OS_OK) {
		CMD_ERR("create flash press test task failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static void cmd_flash_press_r_test_task(void *arg)
{
	uint32_t ret;
	uint32_t addr, size, len, pos, write_pos;
	uint32_t test_cnt;
	uint8_t *wbuf, *rbuf;
	uint32_t cnt, saddr, eaddr, bs, loop;

	/* get param */
	cnt = cmd_sscanf((char *)arg, "s=0x%x e=0x%x b=%d l=%d", &saddr, &eaddr,
	                 &bs, &loop);
	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		goto fail;
	}
	if ((eaddr - saddr) < bs) {
		CMD_ERR("(end addr - start addr) must > buf size !\n");
		goto fail;
	}
#if FLAH_WR_PRESS_DEBUG_ON
	uint32_t debug_size, debug_size_unit;

	debug_size = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
	debug_size_unit = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
#endif

	len = bs;
	addr = saddr;
	size = eaddr - addr;

	test_cnt = 0;
	pos = len;
	write_pos = 0;

	wbuf = cmd_malloc(len);
	if (wbuf == NULL) {
		CMD_ERR("no memory, malloc size %d fail\n", len);
		goto fail;
	}
	memset(wbuf, 0, len);

	rbuf = cmd_malloc(len);
	if (rbuf == NULL) {
		CMD_ERR("no memory, malloc size %d fail\n", len);
		cmd_free(wbuf);
		goto fail;
	}
	memset(rbuf, 0, len);

	for (int i = 0; i < len; ++i) {
		wbuf[i] = ((i) & 0xff);
	}

	CMD_DBG("flash erase start\n");

	if (flash_erase(MFLASH, addr, size)) {
		CMD_ERR("flash erase err, add: 0x%x, size: 0x%x\n", addr, size);
		goto out;
	}
	CMD_DBG("flash erase ok, add: 0x%x, size: 0x%x\n", addr, size);
	while (1) {
		ret = flash_write(MFLASH, addr + write_pos, wbuf, pos);
		if (ret != pos) {
			CMD_ERR("flash write err len:%d pos:%d\n", len, pos);
			goto out;
		}
		CMD_DBG("write data:%dKB\n", write_pos / 1024);
		write_pos += pos;
		if ((size - write_pos) > len)
			pos = len;
		else
			pos = size - write_pos;
		if (pos == 0) {
			CMD_DBG("flash write over, addr:0x%x ~ 0x%x, len:%dKB\n",
			        addr, addr + write_pos, size / 1024);
			break;
		}
	}

	pos = len;
	write_pos = 0;
#if FLAH_WR_PRESS_DEBUG_ON
	debug_size = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
#endif
	while (1) {
		ret = flash_read(MFLASH, addr + write_pos, rbuf, pos);
		if (ret != pos) {
			CMD_ERR("flash read err:%d len pos:%d\n", len, pos);
			goto out;
		}

		for (int j = 0; j < pos; ++j) {
			if (rbuf[j] != ((j) & 0xff)) {
				CMD_ERR("flash read check err len:%d pos:%d + %d\n", len, write_pos, j);
				CMD_ERR("j:%d, now pos:%d\n", j, pos);
#if 1
				for (int dm = ((j > 0) ? (j - 1) : j); dm < pos; ++dm) {
					if ((dm > 0) && (dm % 16) == 0) {
						printf("\n");
					}
					printf(" %02X", rbuf[dm]);
				}
				printf("\n");
#endif
				goto out;
			}
		}
		memset(rbuf, 0, len);

		write_pos += pos;
#if FLAH_WR_PRESS_DEBUG_ON
		if (write_pos >= debug_size) {
			CMD_DBG("read data:%dKB\n", write_pos / 1024);
			debug_size += debug_size_unit;
		}
#endif
		if ((size - write_pos) > len)
			pos = len;
		else
			pos = size - write_pos;

		if (pos == 0) {
			CMD_DBG("flash read over, addr:0x%x ~ 0x%x, len:%dKB\n",
			        addr, addr + write_pos, size / 1024);

			if (++test_cnt >= loop) {
				break;
			} else {
				pos = len;
				write_pos = 0;
#if FLAH_WR_PRESS_DEBUG_ON
				debug_size = FLAH_WR_PRESS_TEST_DEBUG_SIZE;
#endif
			}
		}
	}

	CMD_DBG("flash read test end, run test count:%d, should run:%d\n",
	        test_cnt, loop);

out:
	cmd_free(wbuf);
	cmd_free(rbuf);
fail:
	OS_ThreadDelete(NULL);
}

/*
 * flash press_r s=0x100000 e=0x1FF000 b=8192 l=50
 */
static enum cmd_status cmd_flash_press_r_exec(char *cmd)
{
	uint32_t cnt, saddr, eaddr, bs, loop;

	/* get param */
	cnt = cmd_sscanf(cmd, "s=0x%x e=0x%x b=%d l=%d", &saddr, &eaddr, &bs,
	                 &loop);

	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	OS_Thread_t thread;

	OS_ThreadSetInvalid(&thread);
	if (OS_ThreadCreate(&thread,
	                    "",
	                    cmd_flash_press_r_test_task,
	                    cmd,
	                    OS_THREAD_PRIO_APP,
	                    2 * 1024) != OS_OK) {
		CMD_ERR("create flash read press test task failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_flash_enter_sip_exec(char *cmd)
{
	//writel(0x429D0001, 0x4000A044);
	HAL_SYSCTL_SetSipFlashTestMapMode();
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, NOT_SIP_FLASH), 0);
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, SIP_FLASH), 0);

	return CMD_STATUS_OK;
}

/*
 * brief Flash Auto Test Command
 * command start {spiNum} {csNum} {freq} {mode}
 *         stop
 *         config {ioMode}// -----need to be supported by Flash chip
 *         erase {size} {addr}
 *         write {addr} "{str}"
 *         read {str/hex} {addr} {size} // recommanded that size should not too large
 */
static const struct cmd_data g_flash_cmds[] = {
	{ "start",      cmd_flash_start_exec },
	{ "stop",       cmd_flash_stop_exec },
	{ "erase",      cmd_flash_erase_exec },
	{ "write",      cmd_flash_write_exec },
	{ "read",       cmd_flash_read_exec },
	{ "overwrite",  cmd_flash_overwrite_exec },
#if (defined FLASH_XIP_OPT_WRITE) && (defined FLASH_XIP_OPT_ERASR)
	{ "optimize",   cmd_flash_optimize_exec },
#endif
	{ "set_rmod",   cmd_flash_set_rmode_exec },
	{ "set_wmod",   cmd_flash_set_program_mode_exec },
	{ "bench",      cmd_flash_bench_exec },
	{ "press",      cmd_flash_press_exec },
	{ "press_r",    cmd_flash_press_r_exec },
	{ "enter_sip",  cmd_flash_enter_sip_exec },
};

enum cmd_status cmd_flash_exec(char *cmd)
{
	return cmd_exec(cmd, g_flash_cmds, cmd_nitems(g_flash_cmds));
}
