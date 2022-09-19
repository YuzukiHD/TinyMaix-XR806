/**
  * @file  blec.h
  * @author  XRADIO Bluetooth Team
  */

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

#ifndef _BLEC_H_
#define _BLEC_H_

typedef struct _blec_hci_ctx{
    int (* blec_hci_c2h)(unsigned char hci_type,
                         const unsigned char * buff_start,
                         unsigned int buff_offset,
                         unsigned int buff_len);

    int (* blec_hci_h2c_cb)(unsigned char status,
                            const unsigned char * buff_start,
                            unsigned int buff_offset,
                            unsigned int buff_len);
}blec_hci_t;



int blec_init(void);

void blec_deinit(void);


int blec_hci_init(blec_hci_t * bec_hci);

void blec_hci_deinit(void);

int blec_hci_h2c(unsigned char hci_type,            /* 0x01: command, 0x02: acl data, 0x04:event */
                 const unsigned char * buff_start,  /* not include hci type */
                 unsigned int buff_offset,
                 unsigned int buff_len);            /* not include hci type */

int blec_hci_c2h_cb(unsigned char status,
	                const unsigned char * buff_start,
	                unsigned int buff_offset,
	                unsigned int buff_len);

/**
 * @brief set the LE public Device Address in the Controller.
 *
 * @param[in] bd_addr :pointer to the public address(6 byte).
 *
 * @return 0 on success, error otherwise.
 */
int blec_set_public_address(uint8_t *bd_addr);


#endif /* _BLEC_H_ */
