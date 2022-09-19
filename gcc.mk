#
# Common rules for GCC Makefile
#

# ----------------------------------------------------------------------------
# cross compiler
# ----------------------------------------------------------------------------
CC_DIR := /home/yuzuki/WorkSpace/xr806_sdk/tools/gcc-arm-none-eabi-8-2019-q3-update/bin
CC_PREFIX := $(CC_DIR)/arm-none-eabi-

AS      := $(CC_PREFIX)as
CC      := $(CC_PREFIX)gcc
CPP     := $(CC_PREFIX)g++
LD      := $(CC_PREFIX)ld
NM      := $(CC_PREFIX)nm
AR      := $(CC_PREFIX)ar
OBJCOPY := $(CC_PREFIX)objcopy
OBJDUMP := $(CC_PREFIX)objdump
SIZE    := $(CC_PREFIX)size
STRIP   := $(CC_PREFIX)strip

# ----------------------------------------------------------------------------
# tools
# ----------------------------------------------------------------------------
CP := cp

# $(MKIMAGE) is a tool for creating image
ifeq ($(shell uname -o), Cygwin)
  MKIMAGE := mkimage.exe
else
  MKIMAGE := mkimage
endif

# ----------------------------------------------------------------------------
# global configuration
# ----------------------------------------------------------------------------
include $(ROOT_PATH)/config.mk

# ----------------------------------------------------------------------------
# options
# ----------------------------------------------------------------------------
QUIET ?= n
OPTIMIZE := y
MDK_DBG_EN := n
HARDFP := n

# building display
ifeq ($(QUIET), y)
  Q := @
  S := -s
endif

ifeq ($(OPTIMIZE), y)
  OPTIMIZE_FLAG := -Os -DNDEBUG
else
  OPTIMIZE_FLAG := -O0 -DDEBUG
endif

ifeq ($(MDK_DBG_EN), y)
  DBG_FLAG := -gdwarf-2
else
  DBG_FLAG := -g
endif

ifeq ($(HARDFP), y)
  FLOAT_ABI := hard
else
  FLOAT_ABI := softfp
endif

ifeq ($(CONFIG_TRUSTZONE), y)
  NSC_SYMBOL := nsc_symbol.o
endif

# ----------------------------------------------------------------------------
# flags for compiler and linker
# ----------------------------------------------------------------------------
# CPU/FPU options
ifeq ($(CONFIG_CPU_ARCH), CM4F)
  CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=$(FLOAT_ABI)
else ifeq ($(CONFIG_CPU_ARCH), CM3)
  CPU := -mcpu=cortex-m3 -mthumb
else ifeq ($(CONFIG_CPU_ARCH), CM33F)
  CPU := -mcpu=cortex-m33 -mtune=cortex-m33 -march=armv8-m.main+dsp -mfpu=fpv5-sp-d16 -mfloat-abi=$(FLOAT_ABI) -mcmse -mthumb
else
  ifeq ($(make_configing), n)
    $(error cpu arch not defined)
  endif
endif

CC_FLAGS = $(CPU) -c $(DBG_FLAG) -fno-common -fmessage-length=0 \
	-fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer \
	-Wall -Werror -Wno-cpp -Wpointer-arith -Wno-error=unused-function \
	-MMD -MP $(OPTIMIZE_FLAG)

CC_FLAGS += -Wno-error=stringop-truncation -Wno-error=restrict

# include config header for all c and cpp files
CC_FLAGS += -include xr_config.h

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs \
	-Wl,-Map=$(basename $@).map,--cref

# config symbols
CC_SYMBOLS = $(CONFIG_SYMBOLS)
AS_SYMBOLS = $(CONFIG_SYMBOLS)
AS_SYMBOLS += -include $(ROOT_PATH)/include/generated/autoconf.h

ifeq ($(CONFIG_LIBC_PRINTF_FLOAT), y)
  LD_FLAGS += -u _printf_float
endif

ifeq ($(CONFIG_LIBC_SCANF_FLOAT), y)
  LD_FLAGS += -u _scanf_float
endif

LD_FLAGS += -Wl,--wrap,main
LD_FLAGS += -Wl,--wrap,exit

LD_FLAGS += -Wl,--wrap,_malloc_r
LD_FLAGS += -Wl,--wrap,_realloc_r
LD_FLAGS += -Wl,--wrap,_free_r

LD_FLAGS += -Wl,--wrap,gettimeofday
LD_FLAGS += -Wl,--wrap,settimeofday
LD_FLAGS += -Wl,--wrap,time

ifeq ($(CONFIG_LIBC_WRAP_STDIO), y)
LD_FLAGS += -Wl,--wrap,printf
LD_FLAGS += -Wl,--wrap,vprintf
LD_FLAGS += -Wl,--wrap,puts
LD_FLAGS += -Wl,--wrap,fprintf
LD_FLAGS += -Wl,--wrap,vfprintf
LD_FLAGS += -Wl,--wrap,fputs
LD_FLAGS += -Wl,--wrap,putchar
LD_FLAGS += -Wl,--wrap,putc
LD_FLAGS += -Wl,--wrap,fputc
LD_FLAGS += -Wl,--wrap,fflush
endif

LD_FLAGS += -Wl,--wrap,memcpy
LD_FLAGS += -Wl,--wrap,memset
LD_FLAGS += -Wl,--wrap,memmove

# standard libraries
LD_SYS_LIBS := -lstdc++ -lsupc++ -lm -lc -lgcc

# include path
INCLUDE_ROOT_PATH := $(ROOT_PATH)/include
INCLUDE_PATHS = -I$(INCLUDE_ROOT_PATH)
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/libc
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/driver/cmsis

ifeq ($(CONFIG_OS_FREERTOS), y)
  INCLUDE_OS_FREERTOS_PATHS := $(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOS$(CONFIG_OS_FREERTOS_VERSION_STR)
  INCLUDE_OS_FREERTOS_PLATFORM_PATHS := $(shell find $(INCLUDE_OS_FREERTOS_PATHS)/portable/GCC/$(CONFIG_OS_FREERTOS_PLATFORM) -type d)
  INCLUDE_PATHS += -I$(INCLUDE_OS_FREERTOS_PATHS)
  INCLUDE_PATHS += $(foreach dir, $(INCLUDE_OS_FREERTOS_PLATFORM_PATHS), -I$(dir))
endif

ifeq ($(CONFIG_WLAN), y)
ifeq ($(CONFIG_LWIP_VER_1_4_1), y)
  LWIP_DIR := lwip-1.4.1
else ifeq ($(CONFIG_LWIP_VER_2_0_3), y)
  LWIP_DIR := lwip-2.0.3
else ifeq ($(CONFIG_LWIP_VER_2_1_2), y)
  LWIP_DIR := lwip-2.1.2
endif

MBEDTLS_DIR := mbedtls-$(CONFIG_MBEDTLS_VER)

INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(LWIP_DIR)
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(MBEDTLS_DIR)
ifeq ($(CONFIG_LWIP_VER_1_4_1), y)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(LWIP_DIR)/ipv4
endif
endif

# platform relative path
PLATFORM_RELATIVE_PATH := xradio_v$(CONFIG_CHIP_ARCH_VER)

ifeq ($(CONFIG_PLATFORM_FPGA), y)
PLATFORM_RELATIVE_PATH := $(PLATFORM_RELATIVE_PATH)/fpga
endif

ifeq ($(CONFIG_ETF), y)
PLATFORM_RELATIVE_PATH := $(PLATFORM_RELATIVE_PATH)/etf
endif

ifeq ($(CONFIG_BLE), y)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/blec
endif

ifeq ($(CONFIG_BLEHOST), y)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/ble
endif

ifeq ($(CONFIG_TRUSTZONE), y)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/trustzone
endif

# ----------------------------------------------------------------------------
# common makefile for library and project
# ----------------------------------------------------------------------------
LIB_MAKE_RULES := $(ROOT_PATH)/src/lib.mk
PRJ_MAKE_RULES := $(ROOT_PATH)/project/project.mk

# ----------------------------------------------------------------------------
# common rules of compiling objects
# ----------------------------------------------------------------------------
%.o: %.asm
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.s
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.S
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.c
	$(Q)$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<

%.o: %.cpp
	$(Q)$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<
