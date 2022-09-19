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
LIBS := libfs.a

#DIRS := $(shell find . -type d)

DIRS := ./

ifeq ($(CONFIG_FAT_FS), y)
DIRS += fatfs
endif

ifeq ($(CONFIG_LITTLE_FS), y)
DIRS += littlefs
endif

ifeq ($(CONFIG_SPIF_FS), y)
DIRS += spiffs
endif

SRCS := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
