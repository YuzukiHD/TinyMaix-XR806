#
# Rules for building sub-modules
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# sub-modules
# ----------------------------------------------------------------------------
SUBDIRS := driver/chip
SUBDIRS += libc
SUBDIRS += image
SUBDIRS += rom
SUBDIRS += sys
SUBDIRS += debug

ifeq ($(CONFIG_OS_FREERTOS), y)
SUBDIRS += kernel/FreeRTOS/FreeRTOS$(CONFIG_OS_FREERTOS_VERSION_STR)
SUBDIRS += kernel/os/FreeRTOS
endif

ifeq ($(CONFIG_BIN_COMPRESS), y)
ifneq ($(CONFIG_ROM_XZ), y)
SUBDIRS += xz
endif
endif

ifeq ($(CONFIG_WLAN), y)
NET_SUBDIRS := net/ethernetif
NET_SUBDIRS += net/$(LWIP_DIR)
NET_SUBDIRS += net/ping
NET_SUBDIRS += net/HTTPClient
NET_SUBDIRS += net/$(MBEDTLS_DIR)
NET_SUBDIRS += net/nopoll
NET_SUBDIRS += net/libwebsockets/lib
NET_SUBDIRS += net/mqtt
NET_SUBDIRS += net/shttpd-1.42
NET_SUBDIRS += net/sntp
NET_SUBDIRS += net/udhcp-0.9.8
NET_SUBDIRS += net/cloud/aliyun
NET_SUBDIRS += smartlink
NET_SUBDIRS += wlan

AT_SUBDIRS := atcmd
endif

ifeq ($(CONFIG_BLE_FEATURE), y)
SUBDIRS += driver/bluetooth
endif

ifeq ($(CONFIG_BLEHOST), y)
SUBDIRS += ble
endif

ifeq ($(CONFIG_PM), y)
SUBDIRS += pm
endif

ifeq ($(CONFIG_BENCH_MARK), y)
SUBDIRS += coremark
SUBDIRS += dhrystone
SUBDIRS += whetstone
endif

ifneq ($(CONFIG_BOOTLOADER), y)
SUBDIRS += driver/component
ifeq ($(CONFIG_OTA), y)
SUBDIRS += ota
endif
SUBDIRS += console
SUBDIRS += efpg
SUBDIRS += fs
SUBDIRS += audio/pcm
SUBDIRS += audio/manager
SUBDIRS += $(NET_SUBDIRS)
SUBDIRS += $(AT_SUBDIRS)
SUBDIRS += cjson
SUBDIRS += util
ifeq ($(CONFIG_JPEG), y)
SUBDIRS += jpeg
endif
SUBDIRS += sdd
endif

ifeq ($(CONFIG_TINYMAIX), y)
SUBDIRS += tinymaix
endif

ifeq ($(CONFIG_CPLUSPLUS), y)
SUBDIRS += cplusplus
endif

# ----------------------------------------------------------------------------
# building rules
# ----------------------------------------------------------------------------
.PHONY: all install clean install_clean $(SUBDIRS) _all _install _clean _install_clean

all:
	$(Q)$(MAKE) _all TARGET=all

install:
	$(Q)$(MAKE) _install TARGET=install

clean:
	$(Q)$(MAKE) _clean TARGET=clean

install_clean:
	$(Q)$(MAKE) _install TARGET=install_clean

$(SUBDIRS):
	$(Q)$(MAKE) $(S) -C $@ $(TARGET)

_all: $(SUBDIRS)

_install: $(SUBDIRS)

_clean: $(SUBDIRS)

_install_clean: $(SUBDIRS)
