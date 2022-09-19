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

#include "cmd_util.h"
#include "cmd_adc.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_adc.h"
#include "pm/pm.h"

#define CMD_REF_VOL           2500
#define CMD_VBAT_DIV_FACTOR   3
#define CMD_ADC_TEST_DBG      0

static struct cmd_adc_priv {
	uint32_t channel;
	uint32_t enable;
	uint32_t irq_mode;
	uint32_t low_value;
	uint32_t high_value;
	uint32_t voltage;
	uint32_t deviation;
	uint32_t pass;
} adc_priv[ADC_CHANNEL_NUM];

static struct cmd_adc_common {
	uint32_t work_mode;
	uint32_t work_clk;
	uint32_t freq;
	uint32_t delay;
} adc_common;

static enum cmd_status cmd_adc_output_check(uint32_t data, uint32_t voltage, uint32_t deviation)
{
	uint32_t voltage_low, voltage_high;

	voltage_low = (voltage < deviation) ? 0 : (voltage - deviation);
	voltage_high = ((voltage + deviation) > CMD_REF_VOL) ? CMD_REF_VOL : (voltage + deviation);

	voltage_low = voltage_low * 4096 / CMD_REF_VOL;
	voltage_high = voltage_high * 4096 / CMD_REF_VOL;

	if ((data >= voltage_low) && (data <= voltage_high)) {
#if CMD_ADC_TEST_DBG
		CMD_DBG("ADC: data = %u, voltage_low = %u, voltage_high = %u\n",
		        data, voltage_low, voltage_high);
#endif
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("ADC: data = %u, voltage_low = %u, voltage_high = %u\n",
		        data, voltage_low, voltage_high);
		return CMD_STATUS_FAIL;
	}
}

static void cmd_adc_irq_cb(void *arg)
{
	uint32_t data;
	uint32_t channel;
	ADC_IRQState irq_state;
	enum cmd_status status;
	struct cmd_adc_priv *chan_priv = arg;

	chan_priv->pass = 1;
	if (adc_common.work_mode == ADC_BURST_CONV) {
		uint8_t i, num;
		num = HAL_ADC_GetFifoDataCount();
		for (i = 0; i < (num - 1); i++) {
			data = HAL_ADC_GetFifoData();
			status = cmd_adc_output_check(data, chan_priv->voltage, chan_priv->deviation);
			if (status != CMD_STATUS_OK) {
				chan_priv->pass = 0;
				break;
			}
		}
	} else {
		channel = chan_priv->channel;
		data = HAL_ADC_GetValue((ADC_Channel)channel);
		irq_state = HAL_ADC_GetIRQState((ADC_Channel)channel);

#if CMD_ADC_TEST_DBG
		CMD_DBG("channel = %u, data = %u, irq_state = %u\n", chan_priv->channel, data, irq_state);
#endif
		status = cmd_adc_output_check(data, chan_priv->voltage, chan_priv->deviation);
		if (status != CMD_STATUS_OK) {
			chan_priv->pass = 0;
		}
		switch (chan_priv->irq_mode) {
		case ADC_IRQ_NONE:
			chan_priv->pass = 0;
			CMD_ERR("irq_mode: ADC_IRQ_NONE, irq_state = %d\n", irq_state);
			break;
		case ADC_IRQ_DATA:
			if (irq_state != ADC_DATA_IRQ) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_DATA, irq_state = %d\n", irq_state);
			}
			break;
		case ADC_IRQ_LOW:
			if ((irq_state != ADC_LOW_IRQ) || (data > chan_priv->low_value)) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_LOW, irq_state = %d, low_value = %u, data = %u\n",
				        irq_state, chan_priv->low_value, data);
			}
			break;
		case ADC_IRQ_HIGH:
			if ((irq_state != ADC_HIGH_IRQ) || (data < chan_priv->high_value)) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_HIGH, irq_state = %d, high_value = %u, data = %u\n",
				        irq_state, chan_priv->high_value, data);
			}
			break;
		case ADC_IRQ_LOW_DATA:
			if (irq_state != ADC_LOW_DATA_IRQ && irq_state != ADC_DATA_IRQ) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_LOW_DATA, irq_state = %d\n", irq_state);
			}
			break;
		case ADC_IRQ_HIGH_DATA:
			if (irq_state != ADC_HIGH_DATA_IRQ && irq_state != ADC_DATA_IRQ) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_HIGH_DATA, irq_state = %d\n", irq_state);
			}
			break;
		case ADC_IRQ_LOW_HIGH:
			if (irq_state != ADC_LOW_IRQ && irq_state != ADC_HIGH_IRQ) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_LOW_HIGH, irq_state = %d\n", irq_state);
			}
			break;
		case ADC_IRQ_LOW_HIGH_DATA:
			if (irq_state != ADC_LOW_DATA_IRQ && irq_state != ADC_HIGH_DATA_IRQ
			                                  && irq_state != ADC_DATA_IRQ) {
				chan_priv->pass = 0;
				CMD_ERR("irq_mode: ADC_IRQ_LOW_HIGH_DATA, irq_state = %d\n", irq_state);
			}
			break;
		default:
			chan_priv->pass = 0;
			CMD_ERR("channel = %u, irq_mode = %u\n", channel, chan_priv->irq_mode);
			break;
		}
	}
}

static enum cmd_status cmd_adc_init_exec(char *cmd)
{
	int cnt;
	uint32_t work_clk, freq;
	uint32_t delay;
	uint32_t work_mode;
	ADC_InitParam adc_param;
	HAL_Status hal_status;

	cnt = cmd_sscanf(cmd, "m=%u w=%u f=%u d=%u", &work_mode, &work_clk, &freq, &delay);
	if (cnt != 4) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (work_mode > 1) {
		CMD_ERR("invalid work mode %u\n", work_mode);
		return CMD_STATUS_INVALID_ARG;
	}

	if (work_clk > 1) {
		CMD_ERR("invalid work clock %u\n", work_clk);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((freq < 1000) || (freq > 1000000)) {
		CMD_ERR("invalid freq %u\n", freq);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((delay >> 8) != 0) {
		CMD_ERR("invalid delay %u\n", delay);
		return CMD_STATUS_INVALID_ARG;
	}

	cmd_memset(adc_priv, 0, ADC_CHANNEL_NUM * sizeof(struct cmd_adc_priv));
	adc_common.work_mode = work_mode ? ADC_BURST_CONV : ADC_CONTI_CONV;
	adc_common.work_clk = work_clk;
	adc_common.freq = freq;
	adc_common.delay = delay;

#if (CONFIG_CHIP_ARCH_VER == 3)
	adc_param.work_clk  = (ADC_WorkClk)work_clk;
#endif
	adc_param.freq        = freq;
	adc_param.delay       = delay;
	adc_param.mode        = work_mode ? ADC_BURST_CONV : ADC_CONTI_CONV;
#if (CONFIG_CHIP_ARCH_VER > 1)
	adc_param.vref_mode = ADC_VREF_MODE_1;
#endif

	hal_status = HAL_ADC_Init(&adc_param);
	if (hal_status == HAL_OK) {
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("HAL_ADC_Init return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}
}

static enum cmd_status cmd_adc_deinit_exec(char *cmd)
{
	HAL_Status hal_status;

	hal_status = HAL_ADC_DeInit();
	if (hal_status == HAL_OK) {
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("HAL_ADC_DeInit return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}
}

static enum cmd_status cmd_adc_input_exec(char *cmd)
{
	int cnt;
	uint32_t channel, voltage, deviation;

	cnt = cmd_sscanf(cmd, "c=%u v=%u d=%u", &channel, &voltage, &deviation);
	if (cnt != 3) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channel >= ADC_CHANNEL_NUM) {
		CMD_ERR("invalid channel %u\n", channel);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((voltage > CMD_REF_VOL) && (channel != ADC_CHANNEL_VBAT)) {
		CMD_ERR("invalid voltage %u\n", voltage);
		return CMD_STATUS_INVALID_ARG;
	}

	if (deviation > CMD_REF_VOL) {
		CMD_ERR("invalid deviation %u\n", deviation);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channel == ADC_CHANNEL_VBAT) {
		voltage /= CMD_VBAT_DIV_FACTOR;
	}

	adc_priv[channel].voltage   = voltage;
	adc_priv[channel].deviation = deviation;

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_adc_conv_polling_exec(char *cmd)
{
	int cnt;
	uint32_t channel;
	uint32_t data;
	HAL_Status hal_status;

	cnt = cmd_sscanf(cmd, "c=%u", &channel);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channel >= ADC_CHANNEL_NUM) {
		CMD_ERR("invalid channel %u\n", channel);
		return CMD_STATUS_INVALID_ARG;
	}

	hal_status = HAL_ADC_Conv_Polling((ADC_Channel)channel, &data, 10000);
	if (hal_status != HAL_OK) {
		CMD_ERR("HAL_ADC_Conv_Polling return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}

	return cmd_adc_output_check(data, adc_priv[channel].voltage, adc_priv[channel].deviation);
}

static enum cmd_status cmd_adc_config_exec(char *cmd)
{
	int cnt;
	uint32_t channel, enable, irq_mode, low_value, high_value;
	HAL_Status hal_status;

	cnt = cmd_sscanf(cmd, "c=%u e=%u i=%u l=%u h=%u", &channel, &enable,
	                 &irq_mode, &low_value, &high_value);
	if (cnt != 5) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channel >= ADC_CHANNEL_NUM) {
		CMD_ERR("invalid channel %u\n", channel);
		return CMD_STATUS_INVALID_ARG;
	}

	if (enable > 1) {
		CMD_ERR("invalid enable %u\n", enable);
		return CMD_STATUS_INVALID_ARG;
	}

	if (irq_mode > 7) {
		CMD_ERR("invalid irq_mode %u\n", irq_mode);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((low_value >> 12) != 0) {
		CMD_ERR("invalid low_value %u\n", low_value);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((high_value >> 12) != 0) {
		CMD_ERR("invalid high_value %u\n", high_value);
		return CMD_STATUS_INVALID_ARG;
	}

	if (adc_common.work_mode == ADC_BURST_CONV) {
		hal_status = HAL_ADC_FifoConfigChannel((ADC_Channel)channel, (ADC_Select)enable);
		if (hal_status != HAL_OK) {
			CMD_ERR("HAL_ADC_FifoConfigChannel return: hal_status = %d\n", hal_status);
			return CMD_STATUS_FAIL;
		}
	} else {
		hal_status = HAL_ADC_ConfigChannel((ADC_Channel)channel, (ADC_Select)enable,
		                               (ADC_IRQMode)irq_mode, low_value, high_value);
		if (hal_status != HAL_OK) {
			CMD_ERR("HAL_ADC_ConfigChannel return: hal_status = %d\n", hal_status);
			return CMD_STATUS_FAIL;
		}
	}

	adc_priv[channel].channel    = channel;
	adc_priv[channel].enable     = enable;
	adc_priv[channel].irq_mode   = irq_mode;
	adc_priv[channel].low_value  = low_value;
	adc_priv[channel].high_value = high_value;
	adc_priv[channel].pass       = 1;

#if CMD_ADC_TEST_DBG
	CMD_DBG("ADC config: channel = %u, enable = %u, irq_mode = %u, low_value = %u, high_value = %u\n",
	        channel, enable, irq_mode, low_value, high_value);
#endif

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_adc_conv_it_start_exec(char *cmd)
{
	ADC_Channel chan;
	HAL_Status hal_status;

	for (chan = ADC_CHANNEL_0; chan < ADC_CHANNEL_NUM; chan++) {
		if (adc_priv[chan].enable) {
			hal_status = HAL_ADC_EnableIRQCallback(chan, cmd_adc_irq_cb, &adc_priv[chan]);
			if (hal_status != HAL_OK) {
				CMD_ERR("HAL_ADC_EnableIRQCallback return: hal_status = %d\n", hal_status);
				return CMD_STATUS_FAIL;
			}
		}
	}

	hal_status = HAL_ADC_Start_Conv_IT();
	if (hal_status != HAL_OK) {
		CMD_ERR("HAL_ADC_Start_Conv_IT return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_adc_conv_it_stop_exec(char *cmd)
{
	ADC_Channel chan;
	HAL_Status hal_status;

	hal_status = HAL_ADC_Stop_Conv_IT();
	if (hal_status != HAL_OK) {
		CMD_ERR("HAL_ADC_Stop_Conv_IT return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}

	for (chan = ADC_CHANNEL_0; chan < ADC_CHANNEL_NUM; chan++) {
		if (adc_priv[chan].enable) {
			hal_status = HAL_ADC_DisableIRQCallback(chan);
			if (hal_status != HAL_OK) {
				CMD_ERR("HAL_ADC_DisableIRQCallback return: hal_status = %d\n", hal_status);
				return CMD_STATUS_FAIL;
			}
		}
	}

	for (chan = ADC_CHANNEL_0; chan < ADC_CHANNEL_NUM; chan++) {
		if ((adc_priv[chan].enable) && (adc_priv[chan].pass == 0)) {
			return CMD_STATUS_FAIL;
		}
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_adc_wakeup_exec(char *cmd)
{
	int cnt;
	enum cmd_status status;
	uint32_t pm_mode;

	cnt = cmd_sscanf(cmd, "m=%u", &pm_mode);
	if (cnt != 1) {
		return CMD_STATUS_FAIL;
	}

	status = CMD_STATUS_OK;
	switch (pm_mode) {
	case 0:
		HAL_ADC_SetBypassPmMode(PM_SUPPORT_SLEEP);
		break;
	case 1:
		HAL_ADC_SetBypassPmMode(PM_SUPPORT_STANDBY);
		break;
	default:
		CMD_ERR("invalid pm mode:%u\n", pm_mode);
		status = CMD_STATUS_FAIL;
		break;
	}

	return status;
}

static const struct cmd_data g_adc_cmds[] = {
	{ "init",           cmd_adc_init_exec },
	{ "deinit",         cmd_adc_deinit_exec },
	{ "input",          cmd_adc_input_exec },
	{ "conv-polling",   cmd_adc_conv_polling_exec },
	{ "config",         cmd_adc_config_exec },
	{ "conv-it-start",  cmd_adc_conv_it_start_exec },
	{ "conv-it-stop",   cmd_adc_conv_it_stop_exec },
	{ "wakeup",         cmd_adc_wakeup_exec },
};

enum cmd_status cmd_adc_exec(char *cmd)
{
	return cmd_exec(cmd, g_adc_cmds, cmd_nitems(g_adc_cmds));
}
