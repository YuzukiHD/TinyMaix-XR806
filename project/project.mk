#
# Rules for building project
#

# ----------------------------------------------------------------------------
# Environment variables.
# ----------------------------------------------------------------------------
export KCONFIG_CONFIG=$(ROOT_PATH)/.config

# ----------------------------------------------------------------------------
# library
# ----------------------------------------------------------------------------
LIBRARY_PATHS := -L$(ROOT_PATH)/lib
LIBRARY_PATHS += -L$(ROOT_PATH)/lib/$(PLATFORM_RELATIVE_PATH)
LIBRARY_PATHS += $(PRJ_EXTRA_LIBS_PATH)

# wlan libs
ifeq ($(CONFIG_WLAN_STA)_$(CONFIG_WLAN_AP), y_y)
  ifeq ($(CONFIG_WLAN_STA_WPS), y)
    LIB_WPA += -lwpas_wps_hostapd
  else
    LIB_WPA += -lwpas_hostapd
  endif
else
  ifeq ($(CONFIG_WLAN_STA), y)
    ifeq ($(CONFIG_WLAN_STA_WPS), y)
      LIB_WPA += -lwpas_wps
    else
      LIB_WPA += -lwpas
    endif
  endif
  ifeq ($(CONFIG_WLAN_AP), y)
    LIB_WPA += -lhostapd
  endif
endif

ifeq ($(CONFIG_ETF), y)
  LIB_WLAN := -lxretf
  ifeq ($(CONFIG_CHIP_ARCH_VER), 1)
    LIB_WLAN += -lwlan
  endif
else
ifeq ($(CONFIG_WLAN), y)
  LIB_WLAN := -lwlan $(LIB_WPA) -lnet80211 -lxrwireless -lnet80211

  ifeq ($(CONFIG_CHIP_ARCH_VER), 3)
     ifeq ($(CONFIG_WLAN), y)
        LIB_WLAN += -lxrwireless_phy
     endif
  endif
endif
endif

# There are strong and weak symbols in "lchip", it may link to the weak symbol
# as they are statc libraries, so use --whole-archive to solve this problem.
LIBRARIES := -Wl,--whole-archive -lchip -lxrsys -lrom -Wl,--no-whole-archive

# add extra libs from specific project
LIBRARIES += $(PRJ_EXTRA_LIBS)

ifneq ($(CONFIG_BOOTLOADER), y)

ifeq ($(CONFIG_OTA), y)
LIBRARIES += -lota
endif
LIBRARIES += -lefpg

LIBRARIES += $(LIB_WLAN)

# audio player libs
ifeq ($(CONFIG_XPLAYER), y)
  LIBRARIES += -lcedarx
  LIBRARIES += -lmp3
  LIBRARIES += -lamr
  LIBRARIES += -lamren
  LIBRARIES += -lwav
  LIBRARIES += -laac
  LIBRARIES += -lcedarx
endif

ifeq ($(CONFIG_BLEHOST), y)
  LIBRARIES += -l_ble_host
endif

ifeq ($(CONFIG_BLE), y)
LIBRARIES += -lbt_driver
LIBRARIES += -lblec
endif

# network libs
ifeq ($(CONFIG_WLAN), y)
LIBRARIES += -lmqtt
LIBRARIES += -lnopoll
LIBRARIES += -llibwebsockets
LIBRARIES += -lhttpd
LIBRARIES += -lhttpcli
LIBRARIES += -lmbedtls
LIBRARIES += -lsntp
LIBRARIES += -lping
LIBRARIES += -ludhcpd
LIBRARIES += -lsmartlink
LIBRARIES += -lairkiss_aes
LIBRARIES += -lsc_assistant
LIBRARIES += -lethernetif
LIBRARIES += -llwip
LIBRARIES += -lmbuf
endif

# other libs
LIBRARIES += -lcjson
ifeq ($(CONFIG_FILESYSTEMS), y)
LIBRARIES += -lfs
endif
LIBRARIES += -lconsole
LIBRARIES += -lcomponent
LIBRARIES += -lreverb
LIBRARIES += -laudmgr
LIBRARIES += -lpcm
LIBRARIES += -ladt
LIBRARIES += -lutil
ifeq ($(CONFIG_JPEG), y)
LIBRARIES += -ljpeg
endif
LIBRARIES += -lsdd
LIBRARIES += -lzbar
LIBRARIES += -leq
LIBRARIES += -ldrc
LIBRARIES += -lopus

endif # CONFIG_BOOTLOADER

ifeq ($(CONFIG_BIN_COMPRESS), y)
ifneq ($(CONFIG_ROM_XZ), y)
  LIBRARIES += -lxz
endif
endif

ifeq ($(CONFIG_PM), y)
LIBRARIES += -lpm
endif

ifeq ($(CONFIG_BENCH_MARK), y)
LIBRARIES += -lcoremark -ldhrystone -lwhetstone
endif

ifeq ($(CONFIG_SECURE_BOOT)_$(CONFIG_CHIP_ARCH_VER), y_2)
LIBRARIES += -lsecureboot
endif

LIBRARIES += -limage -los

ifeq ($(CONFIG_OS_FREERTOS), y)
  LIBRARIES += -lfreertos
endif

LIBRARIES += -ldebug

ifeq ($(CONFIG_WATCHPOINT), y)
  LIBRARIES += -lwatchpoint
endif

ifeq ($(CONFIG_CPLUSPLUS), y)
  LIBRARIES += -lcplusplus
endif

ifeq ($(CONFIG_TRUSTZONE), y)
	LIBRARIES += $(ROOT_PATH)/lib/$(NSC_SYMBOL)
endif

LIBRARIES += -lxrc $(LD_SYS_LIBS) -lxrc

# ----------------------------------------------------------------------------
# extra include path
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_XPLAYER), y)
  CEDARX_INC_DIRS := $(shell find $(ROOT_PATH)/include/cedarx -type d)
  INCLUDE_PATHS += $(foreach dir, $(CEDARX_INC_DIRS), -I$(dir))
endif

INCLUDE_PATHS += -I$(ROOT_PATH)/project

ifneq ($(CONFIG_BOARD),)
  INCLUDE_PATHS += -I$(ROOT_PATH)/project/common/board/$(shell echo $(CONFIG_BOARD))
endif

INCLUDE_PATHS += $(PRJ_EXTRA_INC_PATH)

# ----------------------------------------------------------------------------
# include config header for all project
# ----------------------------------------------------------------------------
CC_FLAGS += -include common/prj_conf_opt.h

# ----------------------------------------------------------------------------
# common suffix
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_XIP), y)
  SUFFIX_XIP := _xip
endif

ifeq ($(CONFIG_PSRAM), y)
  SUFFIX_PSRAM := _psram
endif

ifeq ($(CONFIG_BIN_COMPRESS), y)
  SUFFIX_XZ := _xz
endif

ifeq ($(CONFIG_OTA_POLICY_IMAGE_COMPRESSION), y)
  SUFFIX_IMG_XZ := _img_xz
endif

# ----------------------------------------------------------------------------
# linker script
# ----------------------------------------------------------------------------
# linker script, maybe override by the specific project
LINKER_SCRIPT_PATH ?= $(ROOT_PATH)/project/linker_script/gcc

ifeq ($(CONFIG_BOOTLOADER), y)
  LINKER_SCRIPT ?= $(LINKER_SCRIPT_PATH)/bootloader.ld
else
  LINKER_SCRIPT ?= $(LINKER_SCRIPT_PATH)/appos.ld
endif

# linker script generated automatically for building elf file
PROJECT_LD := .project.ld

# ----------------------------------------------------------------------------
# image
# ----------------------------------------------------------------------------
# original bin path, files and names
BIN_PATH := $(ROOT_PATH)/bin/$(PLATFORM_RELATIVE_PATH)

BOOT_BIN_PATH := $(ROOT_PATH)/bin/xradio_v$(CONFIG_CHIP_ARCH_VER)/boot/$(CONFIG_CHIP_TYPE)
ifeq ($(findstring y, $(CONFIG_TRUSTZONE_BOOT) $(CONFIG_TRUSTZONE)), y)
BOOT_BIN_NAME := boot_$(CONFIG_HOSC_TYPE)M_tz.bin
else
BOOT_BIN_NAME := boot_$(CONFIG_HOSC_TYPE)M.bin
endif
APP_BIN_NAME := app.bin

ifeq ($(CONFIG_XIP), y)
APP_XIP_BIN_NAME := app$(SUFFIX_XIP).bin
endif

ifeq ($(CONFIG_PSRAM), y)
APP_PSRAM_BIN_NAME := app$(SUFFIX_PSRAM).bin
endif

SYS_SDD_NAME := sys_sdd_$(CONFIG_HOSC_TYPE)M.bin

BIN_FILES := $(BOOT_BIN_PATH)/$(BOOT_BIN_NAME)
BIN_FILES += $(BIN_PATH)/wlan_bl.bin
BIN_FILES += $(BIN_PATH)/wlan_fw.bin
BIN_FILES += $(BIN_PATH)/$(SYS_SDD_NAME)
ifeq ($(CONFIG_TRUSTZONE), y)
	TZ_API_BIN := tz_api.bin
	BIN_FILES += $(BIN_PATH)/$(TZ_API_BIN)
ifeq ($(CONFIG_TZ_XIP), y)
	TZ_XIP_BIN := tz_xip.bin
	BIN_FILES += $(BIN_PATH)/$(TZ_XIP_BIN)
endif
ifeq ($(CONFIG_TZ_PSRAM), y)
	TZ_PSRAM_BIN := tz_psram.bin
	BIN_FILES += $(BIN_PATH)/$(TZ_PSRAM_BIN)
endif
endif

BIN_NAMES := $(notdir $(BIN_FILES))

ifeq ($(CONFIG_BIN_COMPRESS), y)

# xz is a tool used to compress bins
XZ_CHECK ?= none
XZ_LZMA2_DICT_SIZE ?= 8KiB
XZ := xz -f -k --no-sparse --armthumb --check=$(XZ_CHECK) \
         --lzma2=preset=6,dict=$(XZ_LZMA2_DICT_SIZE),lc=3,lp=1,pb=1

XZ_DEFAULT_BINS := app.bin
XZ_BINS ?= $(XZ_DEFAULT_BINS)

endif # CONFIG_BIN_COMPRESS

# output image path
IMAGE_PATH := ../image/$(CONFIG_CHIP_TYPE)

# $(IMAGE_TOOL) is relative to $(IMAGE_PATH)
IMAGE_TOOL := ../$(ROOT_PATH)/tools/$(MKIMAGE)

# $(MKFS) is a tool for creating fs bin
ifeq ($(CONFIG_FLASH_FS_IMG_PACK), y)
	ifeq ($(shell uname -o), Cygwin)
		ifeq ($(CONFIG_LITTLE_FS), y)
			MKFS := mklittlefs.exe
		else ifeq ($(CONFIG_SPIF_FS), y)
			MKFS := mkspiffs.exe
		endif
		MKFS_EXEC := $(MKFS)
	else
		ifeq ($(CONFIG_LITTLE_FS), y)
			MKFS := mklittlefs
		else ifeq ($(CONFIG_SPIF_FS), y)
			MKFS := mkspiffs
		endif
		MKFS_EXEC := ./$(MKFS)
	endif
endif

# fs image tool option
ifeq ($(CONFIG_FLASH_FS_IMG_PACK), y)
	FS_IMAGE_TOOL_PATH := $(ROOT_PATH)/tools/fs_img_tools
	FS_IMAGE_TOOL := $(MKFS_EXEC)
	FS_IMAGE_TOOL_DBG_LEVEL := 0
	ifeq ($(CONFIG_LITTLE_FS), y)
		FS_IMG_SUFFIX := littlefs
		FS_PAGE_SIZE ?= 256
		FS_BLOCK_SIZE := $(CONFIG_LITTLE_FS_BLOCK_SIZE)
		FS_PHY_SIZE := `expr $(CONFIG_LITTLE_FS_BLOCK_SIZE) \* $(CONFIG_LITTLE_FS_BLOCK_COUNT)`
	else ifeq ($(CONFIG_SPIF_FS), y)
		FS_IMG_SUFFIX := spiffs
		FS_PAGE_SIZE ?= 256
		FS_BLOCK_SIZE := $(CONFIG_SPIF_FS_BLOCK_SIZE)
		FS_PHY_SIZE := $(CONFIG_SPIF_FS_PHY_SIZE)
	endif
endif

# image config file, maybe override by the specific project
# $(IMAGE_CFG_PATH) is relative to $(IMAGE_PATH)
IMAGE_CFG_PATH ?= ../$(ROOT_PATH)/project/image_cfg
IMAGE_CFG ?= $(IMAGE_CFG_PATH)/image.cfg

# image config file generated automatically for creating image, relative to $(IMAGE_PATH)
PROJECT_IMG_CFG := .image.cfg
PROJECT_IMG_XZ_CFG := .image_xz.cfg

# trustzone memory split parameters
ifeq ($(CONFIG_TRUSTZONE), y)
	TZ_PARAMS_PATH := $(ROOT_PATH)/project/tz_params
	TZ_PARAMS_SCRIPT := transform_tz_param.sh
	TZ_PARAMS_ORG := tz_params.txt
	TZ_PARAMS_BIN := tz_params.bin
endif

# image sign script option
SIGN_PACKAGE_PATH := $(ROOT_PATH)/project/sign_script
SIGN_SCRIPT := gen_signature.sh

# image tool's options to enable/disable OTA
ifeq ($(CONFIG_OTA), y)
  IMAGE_TOOL_OPT := -O
else
  IMAGE_TOOL_OPT :=
endif

# image tool's options to enable pack flash fs.bin(raw bin)
ifeq ($(CONFIG_FLASH_FS_IMG_PACK), y)
  IMAGE_TOOL_OPT += -r
endif

# image tool's options to support flash crypto
ifeq ($(CONFIG_FLASH_CRYPTO), y)
  FLASH_CRYPTO_PATH := ../$(ROOT_PATH)/project/flash_crypto
	FLASH_CRYPTO_INI := flash_crypto.ini
  IMAGE_TOOL_OPT += -e $(FLASH_CRYPTO_PATH)/$(FLASH_CRYPTO_INI)
endif

# image name, maybe override by the specific project
IMAGE_NAME ?= xr_system

# secure boot image mass production package option
ifeq ($(CONFIG_SECURE_BOOT), y)
  SIGNPACK_SH := signpack.sh
  SIGNPACK_TOOL := ../$(ROOT_PATH)/tools/$(SIGNPACK_SH)
  SIGNATURE_DIR_NAME := signature
  SIGNATURE_MKIMG_CMD := "./$(MKIMAGE) $(IMAGE_TOOL_OPT) -c \$$imgcfg -o $(IMAGE_NAME).img"
endif
ifeq ($(CONFIG_SECURE_BOOT)_$(CONFIG_CHIP_ARCH_VER), y_2)
  SIGNPACK_GEN_CERT := ./$(SIGNPACK_TOOL) $(PROJECT_IMG_CFG)
else
  SIGNPACK_GEN_CERT := true
endif

ifeq ($(CONFIG_OTA_POLICY_IMAGE_COMPRESSION), y)
# xz is a tool used to compress image
XZ_CHECK ?= none
XZ_LZMA2_DICT_SIZE ?= 8KiB
XZ := xz -f -k --no-sparse --armthumb --check=$(XZ_CHECK) \
         --lzma2=preset=6,dict=$(XZ_LZMA2_DICT_SIZE),lc=3,lp=1,pb=1
XZ_DEFAULT_IMG := $(IMAGE_NAME).img
IMAGE_XZ_CFG ?= $(IMAGE_CFG_PATH)/image$(SUFFIX_IMG_XZ).cfg
ifeq ($(CONFIG_TRUSTZONE), y)
  BOOTLOADER_LENGTH := $(shell od -An -N4 -j 60 -i $(IMAGE_PATH)/$(XZ_DEFAULT_IMG) | sed 's/ //g')
else
  BOOTLOADER_LENGTH := $(shell od -An -N4 -j 32 -i $(IMAGE_PATH)/$(XZ_DEFAULT_IMG) | sed 's/ //g')
endif
endif # CONFIG_OTA_POLICY_IMAGE_COMPRESSION

# ----------------------------------------------------------------------------
# common targets and building rules
# ----------------------------------------------------------------------------
CC_SYMBOLS += $(PRJ_EXTRA_SYMBOLS)

ifeq ($(MDK_DBG_EN), y)
  ELF_EXT = axf
else
  ELF_EXT = elf
endif

ifeq ($(CONFIG_XIP), y)
  OBJCOPY_R_XIP := -R .xip
  OBJCOPY_J_XIP := -j .xip
endif

ifeq ($(CONFIG_PSRAM), y)
  OBJCOPY_R_PSRAM := -R .psram_text -R .psram_data -R .psram_bss
  OBJCOPY_J_PSRAM := -j .psram_text -j .psram_data
endif

PHONY :=

all: __all

ifeq ($(sdk_cfg_rdy), y)
__all: $(PROJECT).bin size
else
__all: include/generated/autoconf.h
	$(Q)$(MAKE) $(S) sdk_cfg_rdy=y
endif

PHONY += include/generated/autoconf.h
include/generated/autoconf.h:
	$(Q)cd $(ROOT_PATH); $(MAKE) prj=$(PRJ_PARENT_DIR)/$(PROJECT) $@; cd -

$(PROJECT).$(ELF_EXT): lib $(OBJS)
ifeq ($(CONFIG_TRUSTZONE), y)
	cd $(ROOT_PATH)/src/trustzone && make && cd -
endif
	$(Q)$(CC) -E -P -CC $(CC_SYMBOLS) -I$(ROOT_PATH)/include -I$(ROOT_PATH)/lib/xradio_v$(CONFIG_CHIP_ARCH_VER) -include generated/autoconf.h -o $(PROJECT_LD) - < $(LINKER_SCRIPT) && \
	$(Q)$(CC) $(LD_FLAGS) -T$(PROJECT_LD) $(LIBRARY_PATHS) -o $@ $(OBJS) $(LIBRARIES)

%.bin: %.$(ELF_EXT)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_R_XIP) $(OBJCOPY_R_PSRAM) $(OBJCOPY_R_EXT) $< $@
ifeq ($(CONFIG_XIP), y)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_J_XIP) $< $(basename $@)$(SUFFIX_XIP).bin
endif
ifeq ($(CONFIG_PSRAM), y)
	$(Q)$(OBJCOPY) -O binary $(OBJCOPY_J_PSRAM) $< $(basename $@)$(SUFFIX_PSRAM).bin
endif

%.objdump: %.$(ELF_EXT)
	$(Q)$(OBJDUMP) -Sdh $< > $@

objdump: $(PROJECT).objdump

size: $(PROJECT).$(ELF_EXT)
	$(Q)$(SIZE) $(PROJECT).$(ELF_EXT)

PHONY += clean
clean:
	$(Q)-rm -f $(PROJECT_LD) $(PROJECT).* *.bin $(OBJS) $(DEPS)
	$(Q)-rm -rf $(ROOT_PATH)/out/*

lib: __lib

ifeq ($(sdk_cfg_rdy), y)
__lib:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src install
else
__lib: include/generated/autoconf.h
	$(Q)$(MAKE) $(S) $@ sdk_cfg_rdy=y
endif

PHONY += lib_clean
lib_clean:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src clean

PHONY += lib_install_clean
lib_install_clean:
	$(Q)$(MAKE) $(S) -C $(ROOT_PATH)/src install_clean

PHONY += trustzone
trustzone:
ifeq ($(CONFIG_TRUSTZONE), y)
	cd $(ROOT_PATH)/src/trustzone/ && \
	make all
endif

PHONY += trustzone_clean
trustzone_clean:
ifeq ($(CONFIG_TRUSTZONE), y)
	cd $(ROOT_PATH)/src/trustzone/ && \
	make clean
endif

build: __build
ifeq ($(sdk_cfg_rdy), y)
__build: all image
else
__build: include/generated/autoconf.h
	$(Q)$(MAKE) $(S) $@ sdk_cfg_rdy=y
endif

ifeq ($(CONFIG_BOOTLOADER), y)

install: $(PROJECT).bin
	$(Q)$(CP) $(PROJECT).bin $(BOOT_BIN_PATH)/$(BOOT_BIN_NAME)

build_clean: clean lib_clean lib_install_clean
	-$(Q)-rm -rf $(ROOT_PATH)/out/*

image: install

else # CONFIG_BOOTLOADER

install: $(PROJECT).bin
	@mkdir -p $(IMAGE_PATH); \
	$(Q)$(CP) $(PROJECT).bin $(IMAGE_PATH)/app.bin
ifeq ($(CONFIG_XIP), y)
	$(Q)$(CP) $(PROJECT)$(SUFFIX_XIP).bin $(IMAGE_PATH)/app$(SUFFIX_XIP).bin
endif
ifeq ($(CONFIG_PSRAM), y)
	$(Q)$(CP) $(PROJECT)$(SUFFIX_PSRAM).bin $(IMAGE_PATH)/app$(SUFFIX_PSRAM).bin
endif

image: install
	-@if ls $(BIN_PATH)/*.bin > /dev/null 2>&1; then \
		$(Q)$(CP) -t $(IMAGE_PATH) $(BIN_FILES); \
	fi
ifeq ($(CONFIG_BIN_COMPRESS), y)
	cd $(IMAGE_PATH) && \
	$(Q)$(XZ) $(XZ_BINS)
endif
ifeq ($(CONFIG_TRUSTZONE), y)
	cd $(TZ_PARAMS_PATH) && \
	chmod 777 $(TZ_PARAMS_SCRIPT) && ./$(TZ_PARAMS_SCRIPT) $(TZ_PARAMS_ORG) && cd - && \
	mv $(TZ_PARAMS_PATH)/$(TZ_PARAMS_BIN) $(IMAGE_PATH)/
	$(Q)$(CP) -t $(SIGN_PACKAGE_PATH) $(IMAGE_PATH)/$(TZ_PARAMS_BIN) $(IMAGE_PATH)/$(TZ_API_BIN)
ifeq ($(CONFIG_TZ_XIP), y)
	$(Q)$(CP) -t $(SIGN_PACKAGE_PATH) $(IMAGE_PATH)/$(TZ_XIP_BIN)
endif
ifeq ($(CONFIG_TZ_PSRAM), y)
	$(Q)$(CP) -t $(SIGN_PACKAGE_PATH) $(IMAGE_PATH)/$(TZ_PSRAM_BIN)
endif
	cd $(SIGN_PACKAGE_PATH) && chmod 777 $(SIGN_SCRIPT) && \
	./$(SIGN_SCRIPT) $(TZ_PARAMS_BIN) && ./$(SIGN_SCRIPT) $(TZ_API_BIN);
ifeq ($(CONFIG_TZ_XIP), y)
	cd $(SIGN_PACKAGE_PATH) && ./$(SIGN_SCRIPT) $(TZ_XIP_BIN)
endif
ifeq ($(CONFIG_TZ_PSRAM), y)
	cd $(SIGN_PACKAGE_PATH) && ./$(SIGN_SCRIPT) $(TZ_PSRAM_BIN)
endif
	mv $(SIGN_PACKAGE_PATH)/*_sign.bin $(IMAGE_PATH)/
endif

ifeq ($(CONFIG_SECURE_BOOT)_$(CONFIG_CHIP_ARCH_VER), y_3)
	$(Q)$(CP) $(IMAGE_PATH)/$(BOOT_BIN_NAME) $(SIGN_PACKAGE_PATH)/boot.bin
	$(Q)$(CP) $(IMAGE_PATH)/$(APP_BIN_NAME) $(SIGN_PACKAGE_PATH)/$(APP_BIN_NAME)
ifeq ($(CONFIG_XIP)_$(CONFIG_APP_XIP_BIN_SIGN), y_y)
	$(Q)$(CP) $(IMAGE_PATH)/$(APP_XIP_BIN_NAME) $(SIGN_PACKAGE_PATH)/$(APP_XIP_BIN_NAME)
endif
ifeq ($(CONFIG_PSRAM)_$(CONFIG_APP_PSRAM_BIN_SIGN), y_y)
	$(Q)$(CP) $(IMAGE_PATH)/$(APP_PSRAM_BIN_NAME) $(SIGN_PACKAGE_PATH)/$(APP_PSRAM_BIN_NAME)
endif
	cd $(SIGN_PACKAGE_PATH) && chmod 777 $(SIGN_SCRIPT) && ./$(SIGN_SCRIPT) boot.bin
	cd $(SIGN_PACKAGE_PATH) && chmod 777 $(SIGN_SCRIPT) && ./$(SIGN_SCRIPT) $(APP_BIN_NAME)
ifeq ($(CONFIG_XIP)_$(CONFIG_APP_XIP_BIN_SIGN), y_y)
	cd $(SIGN_PACKAGE_PATH) && chmod 777 $(SIGN_SCRIPT) && ./$(SIGN_SCRIPT) $(APP_XIP_BIN_NAME)
endif
ifeq ($(CONFIG_PSRAM)_$(CONFIG_APP_PSRAM_BIN_SIGN), y_y)
	cd $(SIGN_PACKAGE_PATH) && chmod 777 $(SIGN_SCRIPT) && ./$(SIGN_SCRIPT) $(APP_PSRAM_BIN_NAME)
endif
	mv $(SIGN_PACKAGE_PATH)/*_sign.bin $(IMAGE_PATH)/
endif

ifeq ($(CONFIG_FLASH_FS_IMG_PACK), y)
	cd $(FS_IMAGE_TOOL_PATH) && mkdir - && chmod 777 $(FS_IMAGE_TOOL) && \
	$(FS_IMAGE_TOOL) -c ./- -d $(FS_IMAGE_TOOL_DBG_LEVEL) -b $(FS_BLOCK_SIZE) -p $(FS_PAGE_SIZE) -s $(FS_PHY_SIZE) fs_$(FS_IMG_SUFFIX).bin && rm -rf -
	mv $(FS_IMAGE_TOOL_PATH)/fs_$(FS_IMG_SUFFIX).bin $(IMAGE_PATH)/
endif

	cd $(IMAGE_PATH) && \
	chmod a+r *.bin && \
	$(Q)$(CC) -E -P -CC $(CC_SYMBOLS) -I$(ROOT_PATH)/../include/generated -include autoconf.h -o $(PROJECT_IMG_CFG) - < $(IMAGE_CFG) && \
	$(SIGNPACK_GEN_CERT) && \
	chmod 777 $(IMAGE_TOOL) && $(IMAGE_TOOL) $(IMAGE_TOOL_OPT) -c $(PROJECT_IMG_CFG) -o $(IMAGE_NAME).img
	@test -d "$(ROOT_PATH)/out" || mkdir -p "$(ROOT_PATH)/out"
	$(Q)$(CP) -t $(ROOT_PATH)/out/ $(IMAGE_PATH)/*.bin $(IMAGE_PATH)/$(IMAGE_NAME).img *.map

PHONY += image_xz
image_xz:
ifeq ($(CONFIG_OTA_POLICY_IMAGE_COMPRESSION), y)
	cd $(IMAGE_PATH) && \
	dd if=$(XZ_DEFAULT_IMG) of=$(XZ_DEFAULT_IMG).temp skip=$(BOOTLOADER_LENGTH) bs=1c && \
	$(Q)$(XZ) $(XZ_DEFAULT_IMG).temp && \
	mv $(XZ_DEFAULT_IMG).temp.xz image.xz && \
	rm $(XZ_DEFAULT_IMG).temp && \
	$(Q)$(CC) -E -P -CC $(CC_SYMBOLS) -I$(ROOT_PATH)/../include/generated -include autoconf.h -o $(PROJECT_IMG_XZ_CFG) - < $(IMAGE_XZ_CFG) && \
	$(IMAGE_TOOL) $(IMAGE_TOOL_OPT) -c $(PROJECT_IMG_XZ_CFG) -o $(IMAGE_NAME)$(SUFFIX_IMG_XZ).img
	@test -d "$(ROOT_PATH)/out" || mkdir -p "$(ROOT_PATH)/out"
	$(Q)$(CP) -t $(ROOT_PATH)/out/ $(IMAGE_PATH)/$(IMAGE_NAME)$(SUFFIX_IMG_XZ).img
endif

PHONY += image_clean
image_clean:
	-cd $(IMAGE_PATH) && \
	rm -f $(PROJECT_IMG_CFG) $(BIN_NAMES) app*.bin fs_*.bin *_sign.bin *.xz *.crt *.img

ifeq ($(CONFIG_SECURE_BOOT), y)
PHONY += sign
sign:
	$(Q)mkdir -p $(IMAGE_PATH)/$(SIGNATURE_DIR_NAME)
	cd $(IMAGE_PATH) && \
	$(Q)cp -f -t $(SIGNATURE_DIR_NAME) *.* $(IMAGE_TOOL) $(PROJECT_IMG_CFG) $(SIGNPACK_TOOL) && \
	echo $(SIGNATURE_MKIMG_CMD) >> $(SIGNATURE_DIR_NAME)/$(SIGNPACK_SH) && \
	$(Q)rm -f $(SIGNATURE_DIR_NAME)/$(IMAGE_NAME).img
ifeq ($(CONFIG_CHIP_ARCH_VER), 3)
	cd $(IMAGE_PATH) && $(Q)rm -f $(SIGNATURE_DIR_NAME)/*_sign.bin && \
	$(Q)cp -rf $(SIGNATURE_DIR_NAME) ../$(ROOT_PATH)/out && $(Q)rm -rf $(SIGNATURE_DIR_NAME)
endif
endif # CONFIG_SECURE_BOOT

build_clean: image_clean clean lib_clean lib_install_clean trustzone_clean
	-$(Q)-rm -rf $(ROOT_PATH)/out/*

endif # CONFIG_BOOTLOADER

config_cpy: defconfig

PHONY += defconfig
defconfig:
	-@cp ./defconfig ./$(ROOT_PATH)/.config; \
	$(Q)cd $(ROOT_PATH); $(MAKE) $@; cd -

PHONY += config
config:
	$(Q)cd $(ROOT_PATH); $(MAKE) $@; cd -

PHONY += oldconfig
oldconfig:
	$(Q)cd $(ROOT_PATH); $(MAKE) $@; cd -

PHONY += menuconfig
menuconfig:
	$(Q)cd $(ROOT_PATH); $(MAKE) $@; cd -

PHONY += config_clean
config_clean:
	$(Q)cd $(ROOT_PATH); $(MAKE) $@; cd -

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)

.PHONY: $(PHONY)
