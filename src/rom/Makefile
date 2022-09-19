#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := librom.a

ifeq ($(CONFIG_ROM), y)
  DIRS := .
else
  ROM_DIRS_IGNORE :=
  ifneq ($(CONFIG_ROM_FREERTOS), y)
    ROM_DIRS_IGNORE += $(shell find ./rom_bin/src/kernel/FreeRTOS -type d)
  endif
  ifneq ($(CONFIG_ROM_XZ), y)
    ROM_DIRS_IGNORE += ./rom_bin/src/xz
  endif
  ifeq ($(CONFIG_CHIP_ARCH_VER), 3)
    ROM_DIRS_IGNORE += $(shell find ./rom_bin/src/driver/chip/sdmmc -type d)
  endif
  ROM_DIRS_ALL := $(shell find ./rom_bin -type d)
  ROM_DIRS := $(filter-out $(ROM_DIRS_IGNORE),$(ROM_DIRS_ALL))
  DIRS := . $(ROM_DIRS)
  INCLUDE_PATHS += -I$(ROOT_PATH)/src/rom/rom_bin/include
endif

SRCS_FILES := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

ifneq ($(CONFIG_CHIP_ARCH_VER), 3)
  CHIP_FILES_IGNORES := ./rom_bin/src/driver/chip/hal_scr.c ./rom_bin/src/driver/chip/hal_lpuart.c ./rom_bin/src/driver/chip/hal_keyboard.c
  CHIP_FILES_IGNORE = $(CHIP_FILES_IGNORES:.c=)
endif

SRCS := $(sort $(filter-out $(CHIP_FILES_IGNORE),$(SRCS_FILES)))

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
