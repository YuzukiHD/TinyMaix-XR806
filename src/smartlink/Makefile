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
LIBS := libsmartlink.a

DIRS_IGNORE := ./blink

DIRS_ALL := $(shell find . -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))

ifeq ($(CONFIG_BLEHOST)_$(CONFIG_WLAN), y_y)
DIRS += ./blink
endif

SRCS := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

OBJS := $(addsuffix .o,$(SRCS))

# extra include path
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/src/smartlink/alink/include/

ifeq ($(CONFIG_BLEHOST), y)
INCLUDE_PATHS += -I$(ROOT_PATH)/include/ble
endif

CC_SYMBOLS += -DCONFIG_ARM

# library make rules
include $(LIB_MAKE_RULES)
