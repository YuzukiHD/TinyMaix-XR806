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

#include "common/framework/platform_init.h"
#include "kernel/os/os.h"
#include <stdio.h>

/* Copyright 2022 Sipeed Technology Co., Ltd. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "stdio.h"
#include "tinymaix/tinymaix.h"

#if TM_MDL_TYPE == TM_MDL_INT8
#include "tmdl/mnist_valid_q.h"
#elif TM_MDL_TYPE == TM_MDL_FP32
#include "tmdl/mnist_valid_f.h"
#elif TM_MDL_TYPE == TM_MDL_FP16
#include "tmdl/mnist_valid_fp16.h"
#elif TM_MDL_TYPE == TM_MDL_FP8_143
#include "tmdl/mnist_fp8_143.h"
#elif TM_MDL_TYPE == TM_MDL_FP8_152
#include "tmdl/mnist_fp8_152.h"
#endif

uint8_t mnist_pic[28 * 28] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   116, 125, 171, 255, 255, 150, 93,  0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   169, 253, 253, 253, 253, 253, 253, 218, 30,  0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   169, 253,
    253, 253, 213, 142, 176, 253, 253, 122, 0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   52,  250, 253, 210, 32,
    12,  0,   6,   206, 253, 140, 0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   77,  251, 210, 25,  0,   0,   0,
    122, 248, 253, 65,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   31,  18,  0,   0,   0,   0,   209, 253,
    253, 65,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   117, 247, 253, 198, 10,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   76,  247, 253, 231, 63,  0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   128, 253, 253, 144, 0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   176, 246, 253, 159, 12,  0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   25,
    234, 253, 233, 35,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   198, 253, 253,
    141, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   78,  248, 253, 189, 12,  0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   19,  200, 253, 253, 141, 0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   134, 253, 253, 173, 12,  0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   248, 253, 253, 25,  0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    248, 253, 253, 43,  20,  20,  20,  20,  5,   0,   5,   20,  20,  37,  150,
    150, 150, 147, 10,  0,   0,   0,   0,   0,   0,   0,   0,   0,   248, 253,
    253, 253, 253, 253, 253, 253, 168, 143, 166, 253, 253, 253, 253, 253, 253,
    253, 123, 0,   0,   0,   0,   0,   0,   0,   0,   0,   174, 253, 253, 253,
    253, 253, 253, 253, 253, 253, 253, 253, 249, 247, 247, 169, 117, 117, 57,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   118, 123, 123, 123, 166,
    253, 253, 253, 155, 123, 123, 41,  0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,
};

static tm_err_t layer_cb(tm_mdl_t *mdl, tml_head_t *lh) { // dump middle result
  int h = lh->out_dims[1];
  int w = lh->out_dims[2];
  int ch = lh->out_dims[3];
  mtype_t *output = TML_GET_OUTPUT(mdl, lh);
  return TM_OK;
  TM_PRINTF("Layer %d callback ========\n", mdl->layer_i);
#if 1
  for (int y = 0; y < h; y++) {
    TM_PRINTF("[");
    for (int x = 0; x < w; x++) {
      TM_PRINTF("[");
      for (int c = 0; c < ch; c++) {
#if TM_MDL_TYPE == TM_MDL_FP32
        TM_PRINTF("%.3f,", output[(y * w + x) * ch + c]);
#else
        TM_PRINTF("%.3f,", TML_DEQUANT(lh, output[(y * w + x) * ch + c]));
#endif
      }
      TM_PRINTF("],");
    }
    TM_PRINTF("],\n");
  }
  TM_PRINTF("\n");
#endif
  return TM_OK;
}

static void parse_output(tm_mat_t *outs) {
  tm_mat_t out = outs[0];
  float *data = out.dataf;
  float maxp = 0;
  int maxi = -1;
  for (int i = 0; i < 10; i++) {
    printf("%d: %.3f\n", i, data[i]);
    if (data[i] > maxp) {
      maxi = i;
      maxp = data[i];
    }
  }
  TM_PRINTF("### Predict output is: Number %d, prob %.3f\n", maxi, maxp);
  return;
}

int main(void) {
  platform_init();

  TM_DBGT_INIT();
  TM_PRINTF("mnist demo\n");
  tm_mdl_t mdl;

  for (int i = 0; i < 28 * 28; i++) {
    TM_PRINTF("%3d,", mnist_pic[i]);
    if (i % 28 == 27)
      TM_PRINTF("\n");
  }

  tm_mat_t in_uint8 = {3, 28, 28, 1, {(mtype_t *)mnist_pic}};
  tm_mat_t in = {3, 28, 28, 1, {NULL}};
  tm_mat_t outs[1];
  tm_err_t res;
  tm_stat((tm_mdlbin_t *)mdl_data);

  res = tm_load(&mdl, mdl_data, NULL, layer_cb, &in);
  if (res != TM_OK) {
    TM_PRINTF("tm model load err %d\n", res);
    return -1;
  }

#if (TM_MDL_TYPE == TM_MDL_INT8) || (TM_MDL_TYPE == TM_MDL_INT16)
  res = tm_preprocess(&mdl, TMPP_UINT2INT, &in_uint8, &in);
#else
  res = tm_preprocess(&mdl, TMPP_UINT2FP01, &in_uint8, &in);
#endif
  TM_DBGT_START();
  res = tm_run(&mdl, &in, outs);
  TM_DBGT("tm_run");
  if (res == TM_OK)
    parse_output(outs);
  else
    TM_PRINTF("tm run error: %d\n", res);
  tm_unload(&mdl);
  return 0;
}
