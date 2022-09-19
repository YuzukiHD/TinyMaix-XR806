/** @file  hrs.h
 *  @brief HRS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

void hrs_init(uint8_t blsc);
int bt_gatt_hrs_notify(uint16_t heartrate);//void hrs_notify(void);

#ifdef __cplusplus
}
#endif
