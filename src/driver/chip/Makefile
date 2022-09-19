#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := libchip.a

DIRS_IGNORE := ./psram
ifeq ($(CONFIG_CHIP_ARCH_VER), 3)
DIRS_IGNORE += ./sdmmc
endif

DIRS_ALL := $(shell find . -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

SRCS_FILES := $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))

ifeq ($(CONFIG_CHIP_ARCH_VER), 2)
  CHIP_FILES_IGNORES := ./hal_scr.c ./hal_lpuart.c ./hal_keyboard.c ./hal_rcosc_cali.c
else ifeq ($(CONFIG_CHIP_ARCH_VER)_$(CONFIG_TRUSTZONE), 3_y)
  CHIP_FILES_IGNORES := ./flash_crypto.c
endif

ifeq ($(CONFIG_PSRAM), y)
SRCS_FILES += ./psram/psram_core.c
ifeq ($(CONFIG_CHIP_ARCH_VER), 2)
SRCS_FILES += ./psram/hal_psramctrl_v2.c
else ifeq ($(CONFIG_CHIP_ARCH_VER), 3)
SRCS_FILES += ./psram/hal_psramctrl_v3.c
endif
ifeq ($(CONFIG_PSRAM_CHIP_SQPI), y)
SRCS_FILES += ./psram/psram_sqpi.c
else ifeq ($(CONFIG_PSRAM_CHIP_OPI32), y)
SRCS_FILES += ./psram/psram_opi32.c
else ifeq ($(CONFIG_PSRAM_CHIP_OPI64), y)
SRCS_FILES += ./psram/psram_opi64.c
endif
endif

SRCS_FILES := $(basename $(SRCS_FILES))
CHIP_FILES_IGNORE = $(CHIP_FILES_IGNORES:.c=)

SRCS := $(sort $(filter-out $(CHIP_FILES_IGNORE),$(SRCS_FILES)))

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
