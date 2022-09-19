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

#include "efpg_i.h"
#include "efpg_debug.h"
#include "driver/chip/hal_efuse.h"

static int efpg_boot_hash_cmp(const uint8_t *data, const uint8_t *buf, uint8_t *err_cnt,
                              uint8_t *err_1st_no, uint8_t *err_2nd_no)
{
	uint8_t byte_cnt;
	uint8_t bit_cnt;

	if (efpg_memcmp(data, buf, EFPG_BOOT_BUF_LEN) == 0) {
		return 0;
	}

	*err_cnt = 0;
	for (byte_cnt = 0; byte_cnt < EFPG_BOOT_BUF_LEN; byte_cnt++) {
		if (data[byte_cnt] == buf[byte_cnt]) {
			continue;
		}

		for (bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
			if ((data[byte_cnt] & (0x1 << bit_cnt)) == (buf[byte_cnt] & (0x1 << bit_cnt))) {
				continue;
			}

			if (*err_cnt == 0) {
				*err_1st_no = (byte_cnt << 3) + bit_cnt;
				*err_cnt = 1;
			} else if (*err_cnt == 1) {
				*err_2nd_no = (byte_cnt << 3) + bit_cnt;
				*err_cnt = 2;
			} else {
				return -1;
			}
		}
	}

	return 0;
}

#if (CONFIG_CHIP_ARCH_VER == 2)
uint16_t efpg_read_mac(efpg_mac_mode_t mode, uint8_t *data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;

	/* flag */
	if (HAL_EFUSE_Read(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_MAC_FLAG_NUM) - 1);
	EFPG_DBG("r start %d, bits %d, flag 0x%x\n", EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, flag);
	if ((flag == 0) || (flag > 7)) {
		EFPG_WARN("%s(), flag (%d, %d) = 0x%x is invalid\n",
		          __func__, EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, flag);
		return EFPG_ACK_NODATA_ERR;
	}

	while ((flag & 0x1) != 0) {
		flag = flag >> 1;
		idx++;
	}

	/* data */
	start_bit = EFPG_MAC_ADDR_START + (idx - 1) * EFPG_MAC_ADDR_NUM;
	EFPG_DBG("r data, start %d, bits %d\n", start_bit, EFPG_MAC_ADDR_NUM);
	if (HAL_EFUSE_Read(start_bit, EFPG_MAC_ADDR_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	if ((data[3] == 0x6d) &&
	    (data[4] == 0x44) &&
	    (data[5] == 0xdc) &&
	    ((data[0] & 0x01) == 0x01)) {
		uint8_t tmp;
		for (int i = 0; i < 3; i++) {
			tmp = data[i];
			data[i] = data[5 - i];
			data[5 - i] = tmp;
		}
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_mac(efpg_mac_mode_t mode, uint8_t *w_data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;
	uint8_t r_data[6];
	uint8_t w_tmp;

	/*read flag */
	if (HAL_EFUSE_Read(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_MAC_FLAG_NUM) - 1);
	EFPG_DBG("w start %d, bits %d, flag 0x%x\n", EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, flag);
	if (flag < 8) {
		if (flag == 0) {
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, &w_tmp) != HAL_OK) ||
				(HAL_EFUSE_Read(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, r_data) != HAL_OK) ||
				(w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		} else {
			while ((flag & 0x1) != 0) {
				flag = flag >> 1;
				idx++;
			}
			idx--;
		}

		while (idx < 3) {
			/*will try compare old mac data with new*/
			start_bit = EFPG_MAC_ADDR_START + idx * EFPG_MAC_ADDR_NUM;
			if (HAL_EFUSE_Read(start_bit, EFPG_MAC_ADDR_NUM, r_data) == HAL_OK) {
				int i = 0;
				for (; i < 6; i++) {
					if (((r_data[i] ^ w_data[i]) & r_data[i]) != 0) {
						break;
					}
				}
				if (i == 6) {
					if ((HAL_EFUSE_Write(start_bit, EFPG_MAC_ADDR_NUM, w_data) == HAL_OK) &&
						(HAL_EFUSE_Read(start_bit, EFPG_MAC_ADDR_NUM, r_data) == HAL_OK) &&
						(memcmp(w_data, r_data, 6) == 0)) {
						return EFPG_ACK_OK;
					}
				}
			}

			if (++idx > 2) {
				EFPG_WARN("mac have not space\n");
				return EFPG_ACK_RW_ERR;
			}
			/*update mac flag as next*/
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, r_data) != HAL_OK) ||
			    (w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		}
	}

	EFPG_WARN("flag (%d, %d) = 0x%x is invalid\n", EFPG_MAC_FLAG_START, EFPG_MAC_FLAG_NUM, flag);
	return EFPG_ACK_NODATA_ERR;
}

uint16_t efpg_read_dcxo(uint8_t *data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;

	/* flag */
	if (HAL_EFUSE_Read(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_DCXO_TRIM_FLAG_NUM) - 1);
	EFPG_DBG("r start %d, bits %d, flag 0x%x\n", EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, flag);
	if ((flag == 0) || (flag > 7)) {
		EFPG_WARN("%s(), flag (%d, %d) = 0x%x is invalid\n",
		          __func__, EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, flag);
		return EFPG_ACK_NODATA_ERR;
	}
	while ((flag & 0x1) != 0) {
		flag = flag >> 1;
		idx++;
	}
	start_bit = EFPG_DCXO_TRIM1_START + (idx - 1) * EFPG_DCXO_TRIM1_NUM;

	/* data */
	EFPG_DBG("r data, start %d, bits %d\n", start_bit, EFPG_DCXO_TRIM1_NUM);
	if (HAL_EFUSE_Read(start_bit, EFPG_DCXO_TRIM1_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_dcxo(uint8_t *w_data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;
	uint8_t r_data[1];
	uint8_t w_tmp;

	/*read flag */
	if (HAL_EFUSE_Read(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_DCXO_TRIM_FLAG_NUM) - 1);
	EFPG_DBG("w start %d, bits %d, flag 0x%x\n", EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, flag);
	if (flag < 8) {
		if (flag == 0) {
			idx = 0;
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, r_data) != HAL_OK) ||
			    (w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		} else {
			while ((flag & 0x1) != 0) {
				flag = flag >> 1;
				idx++;
			}
			idx--;
		}

		while (idx < 3) {
			/*will try compare old mac data with new*/
			start_bit = EFPG_DCXO_TRIM1_START + idx * EFPG_DCXO_TRIM1_NUM;
			if (HAL_EFUSE_Read(start_bit, EFPG_DCXO_TRIM1_NUM, r_data) == HAL_OK) {
				if (((r_data[0] ^ w_data[0]) & r_data[0]) == 0) {
					if ((HAL_EFUSE_Write(start_bit, EFPG_DCXO_TRIM1_NUM, w_data) == HAL_OK) &&
					    (HAL_EFUSE_Read(start_bit, EFPG_DCXO_TRIM1_NUM, r_data) == HAL_OK) &&
					    (w_data[0] == r_data[0])) {
						return EFPG_ACK_OK;
					}
				}
			}

			if (++idx > 2) {
				EFPG_WARN("dcxo have not space\n");
				return EFPG_ACK_RW_ERR;
			}
			/*update mac flag as next*/
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, r_data) != HAL_OK) ||
			    (w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		}
	}

	EFPG_WARN("flag (%d, %d) = 0x%x is invalid\n", EFPG_DCXO_TRIM_FLAG_START, EFPG_DCXO_TRIM_FLAG_NUM, flag);
	return EFPG_ACK_NODATA_ERR;
}

uint16_t efpg_read_pout(uint8_t *data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;

	/* flag */
	if (HAL_EFUSE_Read(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_POUT_CAL_FLAG_NUM) - 1);
	EFPG_DBG("r start %d, bits %d, flag 0x%x\n", EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, flag);
	if ((flag == 0) || (flag > 7)) {
		EFPG_WARN("%s(), flag (%d, %d) = 0x%x is invalid\n", __func__, EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, flag);
		return EFPG_ACK_NODATA_ERR;
	}
	while ((flag & 0x1) != 0) {
		flag = flag >> 1;
		idx++;
	}
	start_bit = EFPG_POUT_CAL1_START + (idx - 1) * EFPG_POUT_CAL1_NUM;

	/* data */
	EFPG_DBG("r data, start %d, bits %d\n", start_bit, EFPG_POUT_CAL1_NUM);
	if (HAL_EFUSE_Read(start_bit, EFPG_POUT_CAL1_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_pout(uint8_t *w_data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;
	uint8_t r_data[EFPG_POUT_BUF_LEN];
	uint8_t w_tmp;

	/*read flag */
	if (HAL_EFUSE_Read(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << EFPG_POUT_CAL_FLAG_NUM) - 1);
	EFPG_DBG("w start %d, bits %d, flag 0x%x\n", EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, flag);
	if (flag < 8) {
		if (flag == 0) {
			idx = 0;
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, r_data) != HAL_OK) ||
			    (w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		} else {
			while ((flag & 0x1) != 0) {
				flag = flag >> 1;
				idx++;
			}
			idx--;
		}

		while (idx < 3) {
			/*will try compare old mac data with new*/
			start_bit = EFPG_POUT_CAL1_START + idx * EFPG_POUT_CAL1_NUM;
			if (HAL_EFUSE_Read(start_bit, EFPG_POUT_CAL1_NUM, r_data) == HAL_OK) {
				r_data[EFPG_POUT_BUF_LEN - 1] &= ((1 << (EFPG_POUT_CAL1_NUM % 8)) - 1);
				if ((((r_data[0] ^ w_data[0]) & r_data[0]) == 0) &&
				    (((r_data[1] ^ w_data[1]) & r_data[1]) == 0) &&
				    (((r_data[2] ^ w_data[2]) & r_data[2]) == 0)) {
					if ((HAL_EFUSE_Write(start_bit, EFPG_POUT_CAL1_NUM, w_data) == HAL_OK) &&
					    (HAL_EFUSE_Read(start_bit, EFPG_POUT_CAL1_NUM, r_data) == HAL_OK)) {
						r_data[EFPG_POUT_BUF_LEN - 1] &= ((1 << (EFPG_POUT_CAL1_NUM % 8)) - 1);
						if (memcmp(w_data, r_data, EFPG_POUT_BUF_LEN) == 0) {
							return EFPG_ACK_OK;
						}
					}
				}
			}
			if (++idx > 2) {
				EFPG_WARN("pout have not space\n");
				return EFPG_ACK_RW_ERR;
			}
			/*update mac flag as next*/
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, r_data) != HAL_OK) ||
			    (w_tmp != (r_data[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		}
	}

	EFPG_WARN("flag (%d, %d) = 0x%x is invalid\n", EFPG_POUT_CAL_FLAG_START, EFPG_POUT_CAL_FLAG_NUM, flag);
	return EFPG_ACK_NODATA_ERR;
}

#elif (CONFIG_CHIP_ARCH_VER == 3)
typedef enum efpg_region_mode {
	EFPG_REGION_READ,
	EFPG_REGION_WRITE,
} efpg_region_mode_t;

typedef struct efpg_region_info {
	uint16_t flag_start;
	uint16_t flag_bits;        /* MUST less than 32-bit */
	uint16_t data_start;
	uint16_t data_bits;
	uint8_t  *buf;             /* temp buffer for write to save read back data */
	uint8_t  buf_len;          /* MUST equal to ((data_bits + 7) / 8) */
} efpg_region_info_t;

#define EFPG_BITS_TO_BYTE_CNT(bits)        (((bits) + 7) / 8)

static uint16_t efpg_read_region(efpg_region_info_t *info, uint8_t *data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;

	/* flag */
	if (HAL_EFUSE_Read(info->flag_start, info->flag_bits, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << info->flag_bits) - 1);
	EFPG_DBG("r start %d, bits %d, flag 0x%x\n", info->flag_start, info->flag_bits, flag);
	if ((flag == 0) || (flag > ((1 << info->flag_bits) - 1))) {
		EFPG_WARN("%s(), flag (%d, %d) = 0x%x is invalid\n", __func__, info->flag_start, info->flag_bits, flag);
		return EFPG_ACK_NODATA_ERR;
	}
	while ((flag & 0x1) != 0) {
		flag = flag >> 1;
		idx++;
	}
	start_bit = info->data_start + (idx - 1) * info->data_bits;

	/* data */
	EFPG_DBG("r data, start %d, bits %d\n", start_bit, info->data_bits);
	if (HAL_EFUSE_Read(start_bit, info->data_bits, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

static uint16_t efpg_write_region(efpg_region_info_t *info, uint8_t *data)
{
	uint8_t idx = 0;
	uint8_t flag;
	uint32_t start_bit;
	uint8_t w_tmp;

	/*read flag */
	if (HAL_EFUSE_Read(info->flag_start, info->flag_bits, (uint8_t *)&flag) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}
	flag &= ((1 << info->flag_bits) - 1);
	EFPG_DBG("w start %d, bits %d, flag 0x%x\n", info->flag_start, info->flag_bits, flag);
	efpg_memset(info->buf, 0, info->buf_len);
	if (flag < (1 << info->flag_bits)) {
		if (flag == 0) {
			idx = 0;
			w_tmp = 1 << idx;
			if ((HAL_EFUSE_Write(info->flag_start, info->flag_bits, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(info->flag_start, info->flag_bits, info->buf) != HAL_OK) ||
			    (w_tmp != (info->buf[0] & w_tmp))) {
				return EFPG_ACK_RW_ERR;
			}
		} else {
			while ((flag & 0x1) != 0) {
				flag = flag >> 1;
				idx++;
			}
			idx--;
		}

		while (idx < info->flag_bits) {
			EFPG_DBG("idx:%d\n", idx);
			/* will try compare old data with new */
			start_bit = info->data_start + idx * info->data_bits;
			efpg_memset(info->buf, 0, info->buf_len);
			if (HAL_EFUSE_Read(start_bit, info->data_bits, info->buf) == HAL_OK) {
				if (info->data_bits % 8) {
					info->buf[info->buf_len - 1] &= ((1 << (info->data_bits % 8)) - 1);
				}
				int i;
				for (i = 0; i < info->buf_len; i++) {
					if (((info->buf[i] ^ data[i]) & info->buf[i]) != 0) {
						break;
					}
				}
				if (i == info->buf_len) {
					efpg_memset(info->buf, 0, info->buf_len);
					EFPG_DBG("w start %d, bits %d\n", start_bit, info->data_bits);
					if ((HAL_EFUSE_Write(start_bit, info->data_bits, data) == HAL_OK) &&
					    (HAL_EFUSE_Read(start_bit, info->data_bits, info->buf) == HAL_OK) &&
					    (efpg_memcmp(data, info->buf, info->buf_len) == 0)) {
						return EFPG_ACK_OK;
					}
				}
			}

			if (++idx > info->flag_bits - 1) {
				EFPG_WARN("region has no space\n");
				return EFPG_ACK_DI_ERR;
			}
			/* update flag as next */
			w_tmp = 1 << idx;
			efpg_memset(info->buf, 0, info->buf_len);
			EFPG_DBG("w start %d, bits %d, w_tmp 0x%x\n", info->flag_start, info->flag_bits, w_tmp);
			if ((HAL_EFUSE_Write(info->flag_start, info->flag_bits, &w_tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(info->flag_start, info->flag_bits, info->buf) != HAL_OK) ||
			    (w_tmp != (info->buf[0] & w_tmp))) {
				EFPG_WARN("fail to write flag\n");
				return EFPG_ACK_RW_ERR;
			}
		}
	}

	EFPG_WARN("flag (%d, %d) = 0x%x is invalid\n", info->flag_start, info->flag_bits, flag);
	return EFPG_ACK_NODATA_ERR;
}

uint16_t efpg_read_efuse_ver(uint8_t *data)
{
	if (HAL_EFUSE_Read(EFPG_EFUSE_VER_START, EFPG_EFUSE_VER_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

static uint16_t efpg_rw_dcxo(efpg_region_mode_t mode, uint8_t *data)
{
#if (EFPG_DCXO_TRIM_NUM == 0)
	EFPG_DBG("no DCXO TRIM defined!\n");
	return EFPG_ACK_NODATA_ERR;
#else
	efpg_region_info_t info;

	info.flag_start = EFPG_DCXO_TRIM_FLAG_START;
	info.flag_bits = EFPG_DCXO_TRIM_FLAG_NUM;
	info.data_start = EFPG_DCXO_TRIM1_START;
	info.data_bits = EFPG_DCXO_TRIM1_NUM;

	if (mode == EFPG_REGION_WRITE) {
		uint8_t buf[EFPG_BITS_TO_BYTE_CNT(EFPG_DCXO_TRIM_LEN)];
		info.buf = buf;
		info.buf_len = sizeof(buf);
		return efpg_write_region(&info, data);
	} else {
		info.buf = NULL;
		info.buf_len = 0;
		return efpg_read_region(&info, data);
	}
#endif
}

uint16_t efpg_read_dcxo(uint8_t *data)
{
	EFPG_DBG("%s()\n", __func__);
	return efpg_rw_dcxo(EFPG_REGION_READ, data);
}

uint16_t efpg_write_dcxo(uint8_t *data)
{
	EFPG_DBG("%s()\n", __func__);
	return efpg_rw_dcxo(EFPG_REGION_WRITE, data);
}

static uint16_t efpg_rw_pout(efpg_region_mode_t reg_mode, efpg_pout_cal_mode_t pout_mode, uint8_t *data)
{
	efpg_region_info_t info;
	if (pout_mode == EFPG_POUT_WLAN) {
#if (EFPG_POUT_CAL_FLAG_NUM == 0)
		EFPG_DBG("no POUT CAL defined!\n");
		return EFPG_ACK_NODATA_ERR;
#else
		info.flag_start = EFPG_POUT_CAL_FLAG_START;
		info.flag_bits = EFPG_POUT_CAL_FLAG_NUM;
		info.data_start = EFPG_POUT_CAL1_START;
		info.data_bits = EFPG_POUT_CAL1_NUM;
#endif
	} else if (pout_mode == EFPG_POUT_BT) {
#if (EFPG_BT_POUT_CAL_FLAG_NUM == 0)
		EFPG_DBG("no BT POUT CAL defined!\n");
		return EFPG_ACK_NODATA_ERR;
#else
		info.flag_start = EFPG_BT_POUT_CAL_FLAG_START;
		info.flag_bits = EFPG_BT_POUT_CAL_FLAG_NUM;
		info.data_start = EFPG_BT_POUT_CAL1_START;
		info.data_bits = EFPG_BT_POUT_CAL1_NUM;
#endif
	} else {
		EFPG_WARN("%s(), mode = %d is invalid\n", __func__, pout_mode);
		return EFPG_ACK_RW_ERR;
	}

	if (reg_mode == EFPG_REGION_WRITE) {
		uint8_t buf[EFPG_BITS_TO_BYTE_CNT(EFPG_POUT_CAL_LEN)];
		info.buf = buf;
		info.buf_len = sizeof(buf);
		return efpg_write_region(&info, data);
	} else {
		info.buf = NULL;
		info.buf_len = 0;
		return efpg_read_region(&info, data);
	}
}

uint16_t efpg_read_pout(efpg_pout_cal_mode_t pout_mode, uint8_t *data)
{
	EFPG_DBG("%s(), mode:%d\n", __func__, pout_mode);
	return efpg_rw_pout(EFPG_REGION_READ, pout_mode, data);
}

uint16_t efpg_write_pout(efpg_pout_cal_mode_t pout_mode, uint8_t *data)
{
	EFPG_DBG("%s(), mode:%d\n", __func__, pout_mode);
	return efpg_rw_pout(EFPG_REGION_WRITE, pout_mode, data);
}

static uint16_t efpg_rw_first_wlan_mac(efpg_region_mode_t reg_mode, uint8_t *data)
{
	efpg_region_info_t info;
	uint8_t efuse_ver;
	if (efpg_read_efuse_ver(&efuse_ver) != EFPG_ACK_OK) {
		EFPG_WARN("%s(), fail to read efuse version\n", __func__);
		return EFPG_ACK_RW_ERR;
	}

	info.flag_start = EFPG_MAC_ADDRESS_FLAG_START;
	info.flag_bits = EFPG_MAC_ADDRESS_FLAG_NUM;
	info.data_start = EFPG_MAC_ADDRESS_START;
	info.data_bits = EFPG_MAC_ADDRESS_NUM;

	if (reg_mode == EFPG_REGION_WRITE) {
		uint8_t buf[EFPG_BITS_TO_BYTE_CNT(EFPG_MAC_ADDRESS_NUM)];
		info.buf = buf;
		info.buf_len = sizeof(buf);

		if (efuse_ver < 2) {
			uint8_t tmp[6];
			for (int i = 0; i < 6; i++) {
				tmp[i] = data[5 - i];
			}
			return efpg_write_region(&info, tmp);
		}

		return efpg_write_region(&info, data);
	} else {
		uint16_t ret;
		info.buf = NULL;
		info.buf_len = 0;
		ret = efpg_read_region(&info, data);
		if (ret != EFPG_ACK_OK) {
			return ret;
		}

		if (efuse_ver < 2) {
			uint8_t tmp;
			for (int i = 0; i < 3; i++) {
				tmp = data[i];
				data[i] = data[5 - i];
				data[5 - i] = tmp;
			}
		}
		return EFPG_ACK_OK;
	}
}

static uint16_t efpg_rw_mac(efpg_region_mode_t reg_mode, efpg_mac_mode_t mac_mode, uint8_t *data)
{
	efpg_region_info_t info;
	if (mac_mode == EFPG_MAC_WLAN) {
#if (EFPG_MAC_WLAN_ADDR_NUM == 0)
		EFPG_DBG("no MAC WLAN ADDR defined!\n");
		return EFPG_ACK_NODATA_ERR;
#else
		info.flag_start = EFPG_MAC_WLAN_ADDR_FLAG_START;
		info.flag_bits = EFPG_MAC_WLAN_ADDR_FLAG_NUM;
		info.data_start = EFPG_MAC_WLAN_ADDR1_START;
		info.data_bits = EFPG_MAC_WLAN_ADDR1_NUM;
#endif
	} else if (mac_mode == EFPG_MAC_BT) {
#if (EFPG_MAC_BT_ADDR_NUM == 0)
		EFPG_DBG("no MAC BT ADDR defined!\n");
		return EFPG_ACK_NODATA_ERR;
#else
		info.flag_start = EFPG_MAC_BT_ADDR_FLAG_START;
		info.flag_bits = EFPG_MAC_BT_ADDR_FLAG_NUM;
		info.data_start = EFPG_MAC_BT_ADDR1_START;
		info.data_bits = EFPG_MAC_BT_ADDR1_NUM;
#endif
	} else {
		EFPG_WARN("%s(), mode = %d is invalid\n", __func__, mac_mode);
		return EFPG_ACK_RW_ERR;
	}

	if (reg_mode == EFPG_REGION_WRITE) {
		uint8_t buf[EFPG_BITS_TO_BYTE_CNT(EFPG_MAC_WLAN_ADDR_LEN)];
		info.buf = buf;
		info.buf_len = sizeof(buf);
		return efpg_write_region(&info, data);
	} else {
		info.buf = NULL;
		info.buf_len = 0;
		return efpg_read_region(&info, data);
	}
}

uint16_t efpg_read_mac(efpg_mac_mode_t mac_mode, uint8_t *data)
{
	uint16_t ret;
	EFPG_DBG("%s(), mode:%d\n", __func__, mac_mode);
	ret = efpg_rw_mac(EFPG_REGION_READ, mac_mode, data);
	if ((mac_mode == EFPG_MAC_WLAN) && (ret == EFPG_ACK_NODATA_ERR)) {
		return efpg_rw_first_wlan_mac(EFPG_REGION_READ, data);
	}
	return ret;
}

uint16_t efpg_write_mac(efpg_mac_mode_t mac_mode, uint8_t *data)
{
	uint16_t ret;
	EFPG_DBG("%s(), mode:%d\n", __func__, mac_mode);
	if (mac_mode == EFPG_MAC_WLAN) {
		uint8_t r_mac[6];
		ret = efpg_rw_mac(EFPG_REGION_READ, mac_mode, r_mac);
		/* do not write mac area if we already have data in wlan mac area. */
		if ((ret == EFPG_ACK_NODATA_ERR) &&
		    (EFPG_ACK_OK == efpg_rw_first_wlan_mac(EFPG_REGION_WRITE, data))) {
			return EFPG_ACK_OK;
		}
	}
	return efpg_rw_mac(EFPG_REGION_WRITE, mac_mode, data);
}

uint16_t efpg_read_all_field(uint16_t start, uint16_t num, uint8_t *data)
{
	if (HAL_EFUSE_Read(start, num, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}
#endif /* CONFIG_CHIP_ARCH_VER == 3 */

uint16_t efpg_read_hosc(uint8_t *data)
{
	if (HAL_EFUSE_Read(EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_read_boot(uint8_t *data)
{
	uint8_t tmp = 0;
	uint8_t byte_cnt;
	uint8_t bit_cnt;

	/* flag */
	if (HAL_EFUSE_Read(EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM, &tmp) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	if (tmp == 0) {
		EFPG_WARN("%s(), %d, boot flag 0\n", __func__, __LINE__);
		return EFPG_ACK_NODATA_ERR;
	}

	/* hash */
	if (HAL_EFUSE_Read(EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	/* correct bit error */
	if (HAL_EFUSE_Read(EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM, &tmp) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	if (tmp != 0) {
		if (HAL_EFUSE_Read(EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM, &tmp) != HAL_OK) {
			return EFPG_ACK_RW_ERR;
		}
		byte_cnt = tmp >> 3;
		bit_cnt = tmp & 0x07;
		data[byte_cnt] ^= (0x1 << bit_cnt);

		if (HAL_EFUSE_Read(EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM, &tmp) != HAL_OK) {
			return EFPG_ACK_RW_ERR;
		}

		if (tmp != 0) {
			if (HAL_EFUSE_Read(EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM, &tmp) != HAL_OK)
				return EFPG_ACK_RW_ERR;
			byte_cnt = tmp >> 3;
			bit_cnt = tmp & 0x07;
			data[byte_cnt] ^= (0x1 << bit_cnt);
		}
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_read_chipid(uint8_t *data)
{
	if (HAL_EFUSE_Read(EFPG_CHIPID_1ST_START, EFPG_CHIPID_1ST_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

#if (CONFIG_CHIP_ARCH_VER == 3)
uint16_t efpg_read_secretkey(uint8_t *data)
{
	if (HAL_EFUSE_Read(EFPG_SECRETKEY_1ST_START, EFPG_SECRETKEY_1ST_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_read_secure_swd(uint8_t *data)
{
	if (HAL_EFUSE_Read(EFPG_SECURE_SWD_START, EFPG_SECURE_SWD_NUM, data) != HAL_OK) {
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}
#endif

uint16_t efpg_read_user_area(uint16_t start, uint16_t num, uint8_t *data)
{
	if ((start >= EFPG_USER_AREA_NUM) ||
	    (num == 0) ||
	    (num > EFPG_USER_AREA_NUM) ||
	    (start + num > EFPG_USER_AREA_NUM)) {
		EFPG_ERR("start %d, num %d\n", start, num);
		return EFPG_ACK_RW_ERR;
	}

	if (HAL_EFUSE_Read(start + EFPG_USER_AREA_START, num, data) != HAL_OK) {
		EFPG_ERR("eFuse read failed\n");
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_hosc(uint8_t *data)
{
	uint8_t buf[EFPG_HOSC_BUF_LEN] = {0};

	if ((HAL_EFUSE_Write(EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM, data) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_HOSC_TYPE_START, EFPG_HOSC_TYPE_NUM, buf) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (efpg_memcmp(data, buf, EFPG_HOSC_BUF_LEN)) {
		EFPG_WARN("%s(), %d, hosc: write %#04x, read %#04x\n",
		          __func__, __LINE__, data[0], buf[0]);
		return EFPG_ACK_DI_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_boot(uint8_t *data)
{
	uint8_t tmp;
	uint8_t err_cnt = 0;
	uint8_t err_1st_no = 0;
	uint8_t err_2nd_no = 0;
	uint8_t buf[EFPG_BOOT_BUF_LEN] = {0};

	/* hash */
	if ((HAL_EFUSE_Write(EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM, data) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_BOOT_HASH_START, EFPG_BOOT_HASH_NUM, buf) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (efpg_boot_hash_cmp(data, buf, &err_cnt, &err_1st_no, &err_2nd_no) < 0) {
		EFPG_WARN("%s(), %d, boot hash: compare failed\n", __func__, __LINE__);
		return EFPG_ACK_DI_ERR;
	}

	/* error bit */
	if (err_cnt > 0) {
		/* bit en */
		tmp = 0x01;
		if ((HAL_EFUSE_Write(EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM, &tmp) != HAL_OK) ||
		    (HAL_EFUSE_Read(EFPG_BOOT_1ST_EN_START, EFPG_BOOT_1ST_EN_NUM, &tmp) != HAL_OK)) {
			return EFPG_ACK_RW_ERR;
		}

		if (tmp != 0x01) {
			EFPG_WARN("%s(), %d, boot 1st en %d\n", __func__, __LINE__, tmp);
			return EFPG_ACK_DI_ERR;
		}

		/* bit no */
		if ((HAL_EFUSE_Write(EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM, &err_1st_no) != HAL_OK) ||
		    (HAL_EFUSE_Read(EFPG_BOOT_1ST_NO_START, EFPG_BOOT_1ST_NO_NUM, &tmp) != HAL_OK)) {
			return EFPG_ACK_RW_ERR;
		}

		if (err_1st_no != tmp) {
			EFPG_WARN("%s(), %d, boot 1st no: write %d, read %d\n",
			          __func__, __LINE__, err_1st_no, tmp);
			return EFPG_ACK_DI_ERR;
		}

		if (err_cnt == 2) {
			/* bit en */
			tmp = 0x01;
			if ((HAL_EFUSE_Write(EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM, &tmp) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_BOOT_2ND_EN_START, EFPG_BOOT_2ND_EN_NUM, &tmp) != HAL_OK)) {
				return EFPG_ACK_RW_ERR;
			}

			if (tmp != 0x01) {
				EFPG_WARN("%s(), %d, boot 2nd en %d\n", __func__, __LINE__, tmp);
				return EFPG_ACK_DI_ERR;
			}

			/* bit no */
			if ((HAL_EFUSE_Write(EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM, &err_2nd_no) != HAL_OK) ||
			    (HAL_EFUSE_Read(EFPG_BOOT_2ND_NO_START, EFPG_BOOT_2ND_NO_NUM, &tmp) != HAL_OK)) {
				return EFPG_ACK_RW_ERR;
			}

			if (err_2nd_no != tmp) {
				EFPG_WARN("%s(), %d, boot 2nd no: write %d, read %d\n",
				          __func__, __LINE__, err_2nd_no, tmp);
				return EFPG_ACK_DI_ERR;
			}
		}
	}

	/* flag */
	tmp = 0x03;
	if ((HAL_EFUSE_Write(EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM, &tmp) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_BOOT_FLAG_START, EFPG_BOOT_FLAG_NUM, &tmp) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (tmp == 0) {
		EFPG_WARN("%s(), %d, boot flag 0\n", __func__, __LINE__);
		return EFPG_ACK_DI_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_write_chipid(uint8_t *data)
{
	uint8_t buf[EFPG_CHIPID_BUF_LEN] = {0};

	if ((HAL_EFUSE_Write(EFPG_CHIPID_1ST_START, EFPG_CHIPID_1ST_NUM, data) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_CHIPID_1ST_START, EFPG_CHIPID_1ST_NUM, buf) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (!efpg_memcmp(data, buf, EFPG_CHIPID_BUF_LEN)) {
		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}

#if (CONFIG_CHIP_ARCH_VER == 3)
uint16_t efpg_write_secretkey(uint8_t *data)
{
	uint8_t buf[EFPG_SK_BUF_LEN] = {0};

	if ((HAL_EFUSE_Write(EFPG_SECRETKEY_1ST_START, EFPG_SECRETKEY_1ST_NUM, data) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_SECRETKEY_1ST_START, EFPG_SECRETKEY_1ST_NUM, buf) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (!efpg_memcmp(data, buf, EFPG_SK_BUF_LEN)) {
		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}

uint16_t efpg_write_secure_swd(uint8_t *data)
{
	uint8_t buf[EFPG_SS_BUF_LEN] = {0};

	if ((HAL_EFUSE_Write(EFPG_SECURE_SWD_START, EFPG_SECURE_SWD_NUM, data) != HAL_OK) ||
	    (HAL_EFUSE_Read(EFPG_SECURE_SWD_START, EFPG_SECURE_SWD_NUM, buf) != HAL_OK)) {
		return EFPG_ACK_RW_ERR;
	}

	if (!efpg_memcmp(data, buf, EFPG_SS_BUF_LEN)) {
		return EFPG_ACK_OK;
	}

	return EFPG_ACK_DI_ERR;
}
#endif

uint16_t efpg_write_user_area(uint16_t start, uint16_t num, uint8_t *data)
{
	if ((start >= EFPG_USER_AREA_NUM) ||
	    (num == 0) ||
	    (num > EFPG_USER_AREA_NUM) ||
	    (start + num > EFPG_USER_AREA_NUM)) {
		EFPG_ERR("start %d, num %d\n", start, num);
		return EFPG_ACK_RW_ERR;
	}

	if (HAL_EFUSE_Write(start + EFPG_USER_AREA_START, num, data) != HAL_OK) {
		EFPG_ERR("eFuse write failed\n");
		return EFPG_ACK_RW_ERR;
	}

	return EFPG_ACK_OK;
}

uint16_t efpg_read_field(efpg_field_t field, uint8_t *data, uint16_t start_bit_addr, uint16_t bit_len)
{
	switch (field) {
	case EFPG_FIELD_HOSC:
		return efpg_read_hosc(data);
	case EFPG_FIELD_BOOT:
		return efpg_read_boot(data);
	case EFPG_FIELD_DCXO:
		return efpg_read_dcxo(data);
	case EFPG_FIELD_POUT_WLAN:
		return efpg_read_pout(EFPG_POUT_WLAN, data);
	case EFPG_FIELD_POUT_BT:
		return efpg_read_pout(EFPG_POUT_BT, data);
	case EFPG_FIELD_MAC_WLAN:
		return efpg_read_mac(EFPG_MAC_WLAN, data);
	case EFPG_FIELD_MAC_BT:
		return efpg_read_mac(EFPG_MAC_BT, data);
	case EFPG_FIELD_CHIPID:
		return efpg_read_chipid(data);
#if (CONFIG_CHIP_ARCH_VER == 3)
	case EFPG_FIELD_SECRETKEY:
		return efpg_read_secretkey(data);
	case EFPG_FIELD_SECURESWD:
		return efpg_read_secure_swd(data);
	case EFPG_FIELD_ALL:
		return efpg_read_all_field(start_bit_addr, bit_len, data);
#endif
	case EFPG_FIELD_UA:
		return efpg_read_user_area(start_bit_addr, bit_len, data);
	default:
		EFPG_WARN("%s(), %d, read field %d\n", __func__, __LINE__, field);
		return EFPG_ACK_RW_ERR;
	}
}


uint16_t efpg_write_field(efpg_field_t field, uint8_t *data, uint16_t start_bit_addr, uint16_t bit_len)
{
	switch (field) {
	case EFPG_FIELD_HOSC:
		return efpg_write_hosc(data);
	case EFPG_FIELD_BOOT:
		return efpg_write_boot(data);
	case EFPG_FIELD_DCXO:
		return efpg_write_dcxo(data);
	case EFPG_FIELD_POUT_WLAN:
		return efpg_write_pout(EFPG_POUT_WLAN, data);
	case EFPG_FIELD_POUT_BT:
		return efpg_write_pout(EFPG_POUT_BT, data);
	case EFPG_FIELD_MAC_WLAN:
		return efpg_write_mac(EFPG_MAC_WLAN, data);
	case EFPG_FIELD_MAC_BT:
		return efpg_write_mac(EFPG_MAC_BT, data);
	case EFPG_FIELD_CHIPID:
		return efpg_write_chipid(data);
#if (CONFIG_CHIP_ARCH_VER == 3)
	case EFPG_FIELD_SECRETKEY:
		return efpg_write_secretkey(data);
	case EFPG_FIELD_SECURESWD:
		return efpg_write_secure_swd(data);
#endif
	case EFPG_FIELD_UA:
		return efpg_write_user_area(start_bit_addr, bit_len, data);
	default:
		EFPG_WARN("%s(), %d, write field %d\n", __func__, __LINE__, field);
		return EFPG_ACK_RW_ERR;
	}
}
