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

#ifndef _IMAGE_IMAGE_H_
#define _IMAGE_IMAGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* compatibility for old bootloader (version < 0.3) due to OTA upgrade */
#define IMAGE_OPT_BL_COMPATIBILITY    (0)

#define IMAGE_SEQ_NUM    (2)    /* max number of image sequence */

/**
 * @brief Image sequence index, starting from 0
 */
typedef uint8_t image_seq_t;

/**
 * @brief Image verification state definition
 */
typedef enum image_state {
	IMAGE_STATE_UNVERIFIED = 0,
	IMAGE_STATE_VERIFIED   = 1,
	IMAGE_STATE_UNDEFINED  = 2,
} image_state_t;

/**
 * @brief Image configuration definition
 */
typedef struct image_cfg {
	image_seq_t   seq;
	image_state_t state;
} image_cfg_t;

/**
 * @brief Image segment definition
 */
typedef enum image_segment {
	IMAGE_SEG_HEADER    = 0,
	IMAGE_SEG_BODY      = 1,
	IMAGE_SEG_TAILER    = 2,
} image_seg_t;

/**
 * @brief Image validity definition
 */
typedef enum image_validity {
	IMAGE_INVALID        = 0,
	IMAGE_VALID          = 1,
} image_val_t;

typedef enum {
	IMAGE_CHECK_NORMAL    = 0,
	IMAGE_CHECK_OTA       = 1,
} image_check_type_t;

/**
 * @brief Section header magic number definition (AWIH)
 */
#define IMAGE_MAGIC_NUMBER    (0x48495741)

/**
 * @brief Section ID definition
 */
#define IMAGE_BOOT_ID         (0xA5FF5A00)
#define IMAGE_APP_ID          (0xA5FE5A01)
#define IMAGE_APP_XIP_ID      (0xA5FD5A02)
#define IMAGE_NET_ID          (0xA5FC5A03)
#define IMAGE_NET_AP_ID       (0xA5FB5A04)
#define IMAGE_WLAN_BL_ID      (0xA5FA5A05)
#define IMAGE_WLAN_FW_ID      (0xA5F95A06)
#define IMAGE_SYS_SDD_ID      (0xA5F85A07)
#define IMAGE_APP_EXT_ID      (0xA5F75A08) /* for XR32 only */
#define IMAGE_APP_PSRAM_ID    (0xA5F65A09)

#if (defined(CONFIG_TRUSTZONE_BOOT) || defined(CONFIG_TRUSTZONE))
#define IMAGE_TZ_PARAM_ID     (0xA5F55A0A)
#define IMAGE_TZ_APP_ID       (0xA5F45A0B)
#define TZ_PARAMS_SIZE        (0xC0)
#define IMAGE_TZ_XIP_ID       (0xA5FD5A12)
#define IMAGE_TZ_PSRAM_ID     (0xA5F65A19)
#endif

/* section ID defined by user starts from IMAGE_USER_ID */
#define IMAGE_USER_ID         (0xB4FF4B00)

#define ECC_SIGN_SIZE         (0x40)
#define TZ_PARAMS_TOTAL       (TZ_PARAMS_SIZE + ECC_SIGN_SIZE)
#define ECC_PKEY_SIZE         (0x40)
#define AES_KEY_SLOT_SIZE     (0x00)
#define FCR_AES_ALIGN_SIZE    (0x10)

/**
 * @brief Section header definition (64 Bytes)
 */
typedef struct section_header {
	uint32_t magic_number;  /* magic number */
	uint32_t version;       /* version */
	uint16_t header_chksum; /* header checksum */
	uint16_t data_chksum;   /* data checksum */
	uint32_t data_size;     /* data size */
	uint32_t load_addr;     /* load address */
	uint32_t entry;         /* entry point */
	uint32_t body_len;      /* body length */
	uint32_t attribute;     /* attribute */
	uint32_t next_addr;     /* next section address */
	uint32_t id;            /* section ID */
	uint32_t priv[6];       /* private data */
} section_header_t;

#define IMAGE_HEADER_SIZE           sizeof(section_header_t)
#define IMAGE_ATTR_FLAG_SIGN        (1 << 2)
#define IMAGE_ATTR_FLAG_ENC         (1 << 3)
#define IMAGE_ATTR_FLAG_COMPRESS    (1 << 4)
#define IMAGE_ATTR_FLAG_SEC_BIN     (1 << 5)

/**
 * @brief OTA parameter definition
 */
typedef struct image_ota_param {
	uint32_t    ota_flash : 8;          /* flash ID of OTA area */
	uint32_t    ota_size  : 24;         /* size of OTA area */
	uint32_t    ota_addr;               /* start addr of OTA area */
	uint16_t    img_max_size;           /* image max size (excluding bootloader, the unit is K) */
	uint16_t    img_xz_max_size;        /* compressed image max size (the unit is K)*/
	uint32_t    bl_size;                /* bootloader size */
	image_seq_t running_seq;            /* running image sequence */
	uint8_t     flash[IMAGE_SEQ_NUM];   /* flash ID which the image on */
	uint32_t    addr[IMAGE_SEQ_NUM];    /* image start addr, excluding bootloader */
} image_ota_param_t;

typedef struct {
	uint32_t    id;
	uint32_t    addr;
} sec_addr_t;

typedef struct {
	image_ota_param_t    iop;
	uint8_t              sec_num[IMAGE_SEQ_NUM];
	sec_addr_t           *sec_addr[IMAGE_SEQ_NUM];
} image_priv_t;

#define IMG_BL_FLASH(iop) ((iop)->flash[0]) /* bootloader flash */
#define IMG_BL_ADDR(iop)  ((iop)->addr[0] - (iop)->bl_size) /* bootloader addr */

#define IMAGE_AREA_SIZE(size) ((size) * 1024)

#define IMAGE_INVALID_ADDR    (0xFFFFFFFF)
#define IMAGE_INVALID_SIZE    (0xFFFF)

/**
 * @brief Initialize the image module
 * @param[in] flash Flash device number of the 1st image region
 * @param[in] addr Start address of the 1st image region (including bootloader)
 * @retval 0 on success, -1 on failure
 */
int image_init(uint32_t flash, uint32_t addr);

/**
 * @brief DeInitialize the image module
 * @return None
 */
void image_deinit(void);

/**
 * @brief Get OTA parameter
 * @return Pointer to OTA parameter (read only)
 */
const image_ota_param_t *image_get_ota_param(void);

/**
 * @brief Set running image sequence
 * @param[in] seq Image sequence
 * @return None
 */
void image_set_running_seq(image_seq_t seq);

/**
 * @brief Get running image sequence
 * @return running image sequence
 */
image_seq_t image_get_running_seq(void);

/**
 * @brief Get address of the specified section in running image
 * @param[in] id ID of the specified section
 * @return section address
 */
uint32_t image_get_section_addr(uint32_t id);

/**
 * @brief Read/Write an amount of image data from/to flash
 * @param[in] id Section ID of the image data
 * @param[in] seg Section segment of the image data
 * @param[in] offset Offset into the segment from where to read/write data
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be read/written
 * @param[in] do_write Read or write
 * @return Number of bytes read/written
 */
uint32_t image_rw(uint32_t id, image_seg_t seg, uint32_t offset,
                  void *buf, uint32_t size, int do_write);

/*
 * @brief For trustzone bootloader, first find out all the secure image information,
          and look up non-secure information as needed. This is because all the
          secure image must be used in bootloader.
 */
uint32_t tzboot_image_get_addr(image_priv_t *img, image_seq_t seq, uint32_t id);

uint32_t app_image_get_addr(image_priv_t *img, image_seq_t seq, uint32_t id);

uint32_t image_get_addr(image_priv_t *img, image_seq_t seq, uint32_t id);

uint32_t image_get_bl_size(section_header_t *sh);

/**
 * @brief Read an amount of image data from flash
 * @param[in] id Section ID of the image data
 * @param[in] seg Section segment of the image data
 * @param[in] offset Offset into the segment from where to read data
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be read
 * @return Number of bytes read
 */
#ifdef CONFIG_SRAM_BIN_ENCRYPT
uint32_t image_read(uint32_t id, image_seg_t seg, uint32_t offset, void *buf, uint32_t size);
#else
static __inline uint32_t image_read(uint32_t id, image_seg_t seg,
                                    uint32_t offset, void *buf, uint32_t size)
{
	return image_rw(id, seg, offset, buf, size, 0);
}
#endif

/**
 * @brief Write an amount of image data to flash
 * @param[in] id Section ID of the image data
 * @param[in] seg Section segment of the image data
 * @param[in] offset Offset into the segment from where to write data
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be written
 * @return Number of bytes written
 */
static __inline uint32_t image_write(uint32_t id, image_seg_t seg,
                                     uint32_t offset, void *buf, uint32_t size)
{
	return image_rw(id, seg, offset, buf, size, 1);
}

uint16_t image_checksum16(uint8_t *data, uint32_t len);

/**
 * @brief Get 16-bit checksum of the data buffer
 * @param[in] buf Pointer to the data buffer
 * @param[in] len length of the data buffer
 * @return 16-bit checksum
 */
uint16_t image_get_checksum(void *buf, uint32_t len);

/**
 * @brief Check vadility of the section header
 * @param[in] sh Pointer to the section header
 * @retval image_val_t, IMAGE_VALID on valid, IMAGE_INVALID on invalid
 */
image_val_t image_check_header(section_header_t *sh);

/**
 * @brief Check vadility of the section data (body and tailer)
 * @param[in] sh Pointer to the section header
 * @param[in] body Pointer to the section body
 * @param[in] body_len Length of the section body
 * @param[in] tailer Pointer to the section tailer
 * @param[in] tailer_len Length of the section tailer
 * @retval image_val_t, IMAGE_VALID on valid, IMAGE_INVALID on invalid
 */
image_val_t image_check_data(section_header_t *sh, void *body, uint32_t body_len,
                             void *tailer, uint32_t tailer_len);

/**
 * @brief Check vadility of the specified section in specified image
 * @param[in] seq Sequence of the specified image
 * @param[in] id Section ID of the specified section
 * @retval image_val_t, IMAGE_VALID on valid, IMAGE_INVALID on invalid
 */
image_val_t image_check_section(image_seq_t seq, uint32_t id);

/**
 * @brief Check vadility of all sections in the specified image
 * @param[in] seq Sequence of the specified image
 * @retval image_val_t, IMAGE_VALID on valid, IMAGE_INVALID on invalid
 */
image_val_t image_check_sections(image_seq_t seq);

/**
 * @brief Check vadility of all sections in the ota image
 * @param[in] seq Sequence of the specified image
 * @retval image_val_t, IMAGE_VALID on valid, IMAGE_INVALID on invalid
 */
image_val_t image_check_ota_sections(image_seq_t seq);

/**
 * @brief Get the size of the running image (including bootloader)
 * @return the size of the running image, 0 on bad image
 */
uint32_t image_get_size(void);

/**
 * @brief Get the image configuration
 * @param[in] cfg Pointer to image configuration
 * @return 0 on success, -1 on failure
 */
int image_get_cfg(image_cfg_t *cfg);

/**
 * @brief Set the image configuration
 * @param[in] cfg Pointer to image configuration
 * @return 0 on success, -1 on failure
 */
int image_set_cfg(image_cfg_t *cfg);

#ifdef CONFIG_ROM
void image_set_dbg_mask(uint16_t dbg_mask);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _IMAGE_IMAGE_H_ */
