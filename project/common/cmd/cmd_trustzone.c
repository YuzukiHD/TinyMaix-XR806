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

#ifdef CONFIG_TRUSTZONE
#include "cmd_util.h"
#include "sys/xr_debug.h"
#include "trustzone/nsc_table.h"
#include "trustzone/tz_thread.h"
#include "../src/trustzone/include/tz_internal_debug.h"
#include "kernel/os/os_semaphore.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_xip.h"
#include "sys/sys_heap.h"

typedef struct {
	OS_Semaphore_t sem;
} cmd_tz_t;
static cmd_tz_t cmd_tz_priv;

typedef struct {
	uint8_t  task_idx;
	uint8_t  task_num;
	uint32_t timeout_ms;
	uint32_t test_sec;
	uint32_t in_data1;
	uint32_t in_data2;
	uint32_t out_rst;
} tz_multi_acc_t;

static enum cmd_status cmd_tz_memr_exec(char *cmd)
{
	int32_t cnt;
	uint32_t addr, len;

	cnt = cmd_sscanf(cmd, "%x %x", &addr, &len);
	if (cnt != 2) {
		return CMD_STATUS_INVALID_ARG;
	}

	tz_debug_memr_nsc(addr, len);

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_tz_memw_exec(char *cmd)
{
	int32_t cnt;
	uint32_t addr, len, value;

	cnt = cmd_sscanf(cmd, "%x %x %x", &addr, &len, &value);
	if (cnt != 3) {
		return CMD_STATUS_INVALID_ARG;
	}

	if ((len != 1) && (len != 2) && (len != 4)) {
		CMD_ERR("invalid len %x\n", len);
		return CMD_STATUS_INVALID_ARG;
	}

	tz_debug_memw_nsc(addr, len, value);

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_tz_heap_press_exec(char *cmd)
{
	int32_t cnt;
	uint32_t test_num = 0, pass_num = 0;

	cnt = cmd_sscanf(cmd, "n=%u", &test_num);
	if (cnt != 1) {
		return CMD_STATUS_INVALID_ARG;
	}
	test_num = test_num < 10000 ? 10000 : test_num;

	pass_num = TEE_FUNC_CALL(tz_debug_heap_press_nsc, test_num);
	if (pass_num == test_num) {
		CMD_LOG(1, "%s test %d successful !\n", __func__, pass_num);
	} else {
		CMD_LOG(1, "%s test failed on the %d cycle !\n", __func__, pass_num);
	}

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_tz_heap_space_exec(char *cmd)
{
	uint8_t *start, *end, *current;

	TEE_FUNC_CALL(TZ_GetHeapSpace_NSC, &start, &end, &current);
	cmd_write_respond(CMD_STATUS_OK, "heap total %u, use %u, free %u, [%p, %p, %p)",
	                  end - start, current - start, end - current, start, current, end);
	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_tz_bkpt_exec(char *cmd)
{
	uint32_t ret = 1;

	ret = tz_debug_bkpt_nsc(1);
	while (ret)
		;

	return CMD_STATUS_ACKED;
}

#if (defined(CONFIG_PSRAM))
/*
 * @brief t = [0,1,2,3]
 *          0-> sdma access ns-sram and s-sram
 *          1-> sdma access ns-psram and s-psram
 *          2-> sdma access s-flash
 *          3-> sdma access ns-flash
 *          4-> sdma access the 16KB sram which share with BLE
 */
static enum cmd_status cmd_tz_sdma_mem_exec(char *cmd)
{
	int32_t cnt;
	uint32_t i, type = 3, len = 0;
	uint8_t *buf, *sram_buf, *psram_buf, data = 0x5a;
	tz_remote_call_t rpc_t;
	enum cmd_status ret = CMD_STATUS_ACKED;

	cnt = cmd_sscanf(cmd, "t=%u l=%u", &type, &len);
	if (cnt != 2) {
		return CMD_STATUS_INVALID_ARG;
	}
	if ((len < 10) || (len > 0x1000) || (type > 4)) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_FAIL;
	}

	sram_buf = (uint8_t *)cmd_malloc(len);
	if (sram_buf == NULL) {
		CMD_ERR("no sram heap space !\n");
	}
	psram_buf = (uint8_t *)psram_malloc(len);
	if (psram_buf == NULL) {
		CMD_ERR("no psram heap space !\n");
	}
	buf = sram_buf;

	switch (type) {
	case 0:
		cmd_memset(buf, data, len);
		break;
	case 1:
		buf = psram_buf;
		cmd_memset(buf, data, len);
		HAL_Dcache_Clean((uint32_t)buf, len);
		break;
	case 2:
		rpc_t.retArg0 = 1;
		ret = CMD_STATUS_FAIL;
		break;
	case 3:
		rpc_t.retArg0 = 1;
		rpc_t.arg3 = FLASH_XIP_START_ADDR;
		ret = CMD_STATUS_FAIL;
		break;
	case 4:
		buf = (uint8_t *)0x240000;
		cmd_memset(buf, data, len);
		break;
	default:
		break;
	}

	rpc_t.arg0 = (uint32_t)buf;
	rpc_t.arg1 = len;
	rpc_t.arg2 = type;
	tz_sdma_test_nsc(&rpc_t);

	if ((type != 2) && (type != 3)) {
		for (i = 0; i < len; i++) {
			if (buf[i] != (uint8_t)~data) {
				break;
			}
		}
		if (i != len) {
			CMD_ERR("sdma test error, error index begin: %d, ref data:0x%x !\n", i, (uint8_t)~data);
			print_hex_dump_bytes(buf, len);
		} else {
			CMD_LOG(1, "sdma test successful !\n");
		}
	} else {
		if (!rpc_t.retArg0) {
			ret = CMD_STATUS_ACKED;
			CMD_LOG(1, "sdma test successful !\n");
		}
	}

	cmd_free(sram_buf);
	psram_free(psram_buf);

	return ret;
}
#endif

static enum cmd_status cmd_tz_sdma_dev_exec(char *cmd)
{
	int32_t cnt;
	enum cmd_status ret = CMD_STATUS_FAIL;
	uint32_t i, len = 0, secure = 0;
	uint8_t *buf, digest[32];
	tz_remote_call_t rpc_t;

	cnt = cmd_sscanf(cmd, "s=%u l=%u", &secure, &len);
	if (cnt != 2) {
		return CMD_STATUS_INVALID_ARG;
	}
	if ((len < 10) || (len > 0x10000) || (secure > 1)) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_FAIL;
	}

	buf = (uint8_t *)cmd_malloc(len);
	if (buf == NULL) {
		CMD_ERR("no heap space !\n");
		goto out;
	}
	for (i = 0; i < len; i++) {
		buf[i] = (i % 0x0a);
	}
	cmd_memset((void *)digest, 0, sizeof(digest));

	rpc_t.arg0 = (uint32_t)buf;
	rpc_t.arg1 = len;
	rpc_t.arg2 = secure;
	rpc_t.argPtr = (uint32_t *)digest;
	if (tz_hw_sha256_nsc(&rpc_t) != RPC_OK) {
		CMD_ERR("tz hw_sha256 run error.\n");
		goto out;
	}

	CMD_LOG(1, "tz hw sha256 result ->\n");
	print_hex_dump_bytes(digest, sizeof(digest));

	ret = CMD_STATUS_ACKED;

out:
	cmd_free(buf);
	return ret;
}

static enum cmd_status cmd_tz_sdma_test_exec(char *cmd)
{
	int32_t cnt, ret;
	uint32_t src_addr, dst_addr, len;
	tz_remote_call_t rpc_t;

	cnt = cmd_sscanf(cmd, "s=%x d=%x l=%x", &src_addr, &dst_addr, &len);
	if (cnt != 3) {
		return CMD_STATUS_INVALID_ARG;
	}

	rpc_t.arg0 = src_addr;
	rpc_t.arg1 = dst_addr;
	rpc_t.arg2 = len;

	ret = tz_sdma_test_random_nsc(&rpc_t);

	if (ret == 0) {
		CMD_LOG(1, "sdma test successful !\n");
	} else {
		CMD_LOG(1, "sdma test error !\n");
	}

	return CMD_STATUS_ACKED;
}

void cmd_tz_dma_callback(void *arg)
{
	OS_SemaphoreRelease(&cmd_tz_priv.sem);
}

static enum cmd_status cmd_tz_ndma_test_exec(char *cmd)
{
	int32_t cnt;
	uint32_t src_addr, dst_addr, len;
	DMA_Channel chan = DMA_CHANNEL_INVALID;
	DMA_ChannelInitParam dmaParam;

	cnt = cmd_sscanf(cmd, "s=%x d=%x l=%x", &src_addr, &dst_addr, &len);
	if (cnt != 3) {
		return CMD_STATUS_INVALID_ARG;
	}

	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = cmd_tz_dma_callback;
	dmaParam.endArg = NULL;

	OS_SemaphoreCreateBinary(&cmd_tz_priv.sem);
	dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
	                                          DMA_WAIT_CYCLE_2,
	                                          DMA_BYTE_CNT_MODE_REMAIN,
	                                          DMA_DATA_WIDTH_8BIT,
	                                          DMA_BURST_LEN_1,
	                                          DMA_ADDR_MODE_INC,
	                                          DMA_PERIPH_SRAM,
	                                          DMA_DATA_WIDTH_8BIT,
	                                          DMA_BURST_LEN_1,
	                                          DMA_ADDR_MODE_INC,
	                                          DMA_PERIPH_SRAM);

	chan = HAL_DMA_Request();
	if (chan == DMA_CHANNEL_INVALID) {
		return chan;
	}
	HAL_DMA_Init(chan, &dmaParam);
	HAL_DMA_Start(chan, src_addr, dst_addr, len);
	OS_SemaphoreWait(&cmd_tz_priv.sem, 5000);

	CMD_LOG(1, "ndma test successful !\n");
	OS_SemaphoreDelete(&cmd_tz_priv.sem);
	HAL_DMA_Stop(chan);
	HAL_DMA_DeInit(chan);
	HAL_DMA_Release(chan);

	return CMD_STATUS_ACKED;
}

static uint8_t task_end = 0;
static void cmd_tz_compete_resource_task(void *arg)
{
	tz_remote_call_t rpc_t;
	tz_multi_acc_t *task_priv = (tz_multi_acc_t *)arg;
	static uint32_t run_time = 0;
	OS_Time_t tick_current = 0, tick_record = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_SecsToTicks(task_priv->test_sec);

	rpc_t.arg0 = task_priv->in_data1;
	rpc_t.arg1 = task_priv->in_data2;
	rpc_t.arg2 = task_priv->timeout_ms;

	while (tick_current < tick_end) {
		OS_MSleep(task_priv->timeout_ms >> 1);
		TEE_FUNC_CALL(tz_multi_access_nsc, &rpc_t);
		tick_current = OS_GetTicks();
		if (task_priv->task_idx == 0) {
			if (OS_TicksToSecs(tick_current - tick_record) >= 10) {
				run_time += 10;
				CMD_LOG(1, "A total of %d %ss run normally %04ds...\n", task_priv->task_num, __func__, run_time);
				tick_record = tick_current;
			}
		}
		if (rpc_t.retArg0 != rpc_t.arg0 * rpc_t.arg1) {
			CMD_ERR("expect result : %d, but get %d.\n", rpc_t.arg0 * rpc_t.arg1, rpc_t.retArg0);
			break;
		}
	}

	CMD_LOG(1, "%s task[%d] test finished, exit !\n", __func__, task_priv->task_idx);
	if (++task_end == task_priv->task_num) {
		CMD_LOG(1, "task competition %d times!\n", rpc_t.retArg1);
		rpc_t.arg0 = 0xffffa55a;
		TEE_FUNC_CALL(tz_multi_access_nsc, &rpc_t);
		task_end = 0;
		run_time = 0;
	}
	cmd_free(task_priv);
	OS_ThreadDelete(NULL);
}

static enum cmd_status cmd_tz_multi_thread_exec(char *cmd)
{
	int32_t cnt;
	OS_Thread_t thread;
	uint32_t thread_num, test_sec, i;
	tz_multi_acc_t *task_priv;

	cnt = cmd_sscanf(cmd, "r=%d t=%d", &thread_num, &test_sec);
	if (cnt != 2) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}
	if ((thread_num < 1) || (thread_num > 6) || (test_sec < 10)) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_INVALID_ARG;
	}


	for (i = 0; i < thread_num; i++) {
		task_priv = cmd_malloc(sizeof(tz_multi_acc_t));
		if (!task_priv) {
			CMD_ERR("no heap space !\n");
			return CMD_STATUS_FAIL;
		}
		task_priv->task_num = thread_num;
		task_priv->task_idx = i;
		task_priv->test_sec = test_sec;
		task_priv->timeout_ms = (thread_num - i) * 10;
		task_priv->in_data1 = task_priv->timeout_ms;
		task_priv->in_data2 = i + 2;

		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_tz_compete_resource_task,
		                    (void *)task_priv,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create trustzone test task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_tz_press_test_exec(char *cmd)
{
	enum cmd_status ret = CMD_STATUS_FAIL;
	int32_t  cnt;
	uint32_t i, test_sec = 0, size = 512;
	tz_remote_call_t rpc_t;
	uint8_t iv[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 1, 1};
	uint8_t ctr[16];
	uint8_t *in = NULL, *out = NULL, *dec = NULL;
	OS_Time_t tick_current = 0, tick_record = 0, tick_end = 0;
	static uint32_t run_time = 0;

	cnt = cmd_sscanf(cmd, "t=%d", &test_sec);
	if (cnt != 1) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}
	if (test_sec < 1) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_FAIL;
	}

	in = (uint8_t *)cmd_malloc(size);
	if (!in) {
		CMD_ERR("no heap space !\n");
		goto error;
	}
	out = (uint8_t *)cmd_malloc(size);
	if (!out) {
		CMD_ERR("no heap space !\n");
		goto error;
	}
	dec = (uint8_t *)cmd_malloc(size);
	if (!dec) {
		CMD_ERR("no heap space !\n");
		goto error;
	}
	for (i = 0; i < size; i++) {
		in[i] = (i & 0xff);
	}

	tick_end = OS_GetTicks() + OS_SecsToTicks(test_sec);
	while (tick_current < tick_end) {
		rpc_t.argPtr = (uint32_t *)out;
		rpc_t.arg1 = size;
		rpc_t.arg2 = (uint32_t)in;
		rpc_t.arg3 = size;
		cmd_memcpy(ctr, iv, sizeof(ctr));

		if (AES_Decrypt_NSC(&rpc_t, ctr, AES_MODE_NSC_CTR) != RPC_OK) {
			CMD_ERR("aes_ctr encrypt test failure !\n");
			break;
		} else {
			rpc_t.argPtr = (uint32_t *)dec;
			rpc_t.arg1 = size;
			rpc_t.arg2 = (uint32_t)out;
			rpc_t.arg3 = size;
			cmd_memcpy(ctr, iv, sizeof(ctr));
			if (AES_Decrypt_NSC(&rpc_t, ctr, AES_MODE_NSC_CTR) != RPC_OK) {
				CMD_ERR("aes_ctr decyrpt test failure !\n");
				break;
			}
		}

		if (cmd_memcmp(in, dec, size)) {
			CMD_ERR("aes_crt test error, decrypt data :!\n");
			print_hex_dump_bytes(dec, size);
			break;
		} else {
			tick_current = OS_GetTicks();
			if (OS_TicksToSecs(tick_current - tick_record) >= 10) {
				run_time += 10;
				CMD_LOG(1, "%s run normally %04ds...\n", __func__, run_time);
				tick_record = tick_current;
			}
			OS_MSleep(5);
		}
	}
	if (tick_current >= tick_end) {
		CMD_LOG(1, "tz pressure test successful !\n");
		ret = CMD_STATUS_OK;
	}
	run_time = 0;

error:
	cmd_free(dec);
	cmd_free(out);
	cmd_free(in);
	return ret;
}

static enum cmd_status cmd_tz_ahb_set_exec(char *cmd)
{
	int32_t cnt;
	uint32_t dev_id, sec_en;

	cnt = cmd_sscanf(cmd, "d=%d s=%d", &dev_id, &sec_en);
	if (cnt != 2) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	if (dev_id > 31 || sec_en > 1) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	tz_ahb_secure_set_nsc(dev_id, sec_en);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_tz_apb_set_exec(char *cmd)
{
	int32_t cnt;
	uint32_t dev_id, sec_en;

	cnt = cmd_sscanf(cmd, "d=%d s=%d", &dev_id, &sec_en);
	if (cnt != 2) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	if (dev_id > 31 || sec_en > 1) {
		CMD_ERR("invalid params !\n");
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	tz_apb_secure_set_nsc(dev_id, sec_en);

	return CMD_STATUS_OK;
}

#if (defined(CONFIG_TZ_PSRAM))
static enum cmd_status cmd_tz_psram_run_exec(char *cmd)
{
	cmd_write_respond(CMD_STATUS_OK, "OK");

	tz_psram_run_exec();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_tz_psram_cmds[] = {
	{ "run",        cmd_tz_psram_run_exec },
};

static enum cmd_status cmd_tz_psram_exec(char *cmd)
{
	return cmd_exec(cmd, g_tz_psram_cmds, cmd_nitems(g_tz_psram_cmds));
}
#endif

static enum cmd_status cmd_tz_sau_info_exec(char *cmd)
{
	cmd_write_respond(CMD_STATUS_OK, "OK");

	tz_sau_info_nsc();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_tz_cmds[] = {
	{ "mem-read",        cmd_tz_memr_exec },
	{ "mem-write",       cmd_tz_memw_exec },
	{ "heap-space",      cmd_tz_heap_space_exec },
	{ "heap-press",      cmd_tz_heap_press_exec },
	{ "break",           cmd_tz_bkpt_exec },
#if (defined(CONFIG_PSRAM))
	{ "sdma-mem",        cmd_tz_sdma_mem_exec },
#if (defined(CONFIG_TZ_PSRAM))
	{ "tz_psram",        cmd_tz_psram_exec},
#endif
#endif
	{ "sdma-dev",        cmd_tz_sdma_dev_exec },
	{ "sdma-test",       cmd_tz_sdma_test_exec },
	{ "ndma-test",       cmd_tz_ndma_test_exec },
	{ "multi-thread",    cmd_tz_multi_thread_exec },
	{ "press-test",      cmd_tz_press_test_exec },
	{ "ahb-set",         cmd_tz_ahb_set_exec},
	{ "apb-set",         cmd_tz_apb_set_exec},
	{ "sau-config",      cmd_tz_sau_info_exec},
};

enum cmd_status cmd_trustzone_exec(char *cmd)
{
	return cmd_exec(cmd, g_tz_cmds, cmd_nitems(g_tz_cmds));
}
#endif
