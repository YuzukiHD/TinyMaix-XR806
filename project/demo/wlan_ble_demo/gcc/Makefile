#
# Rules for building application
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# project name and project root path
# ----------------------------------------------------------------------------
PROJECT := $(notdir $(shell cd .. && pwd))
PRJ_PARENT_DIR := $(notdir $(shell cd ../.. && pwd))
PRJ_ROOT_PATH := $(ROOT_PATH)/project/$(PRJ_PARENT_DIR)/$(PROJECT)
USE_BLE_HOST := y
USE_BLE_HOST_MESH := y

HCIOPTION := VIRTUAL_HCI

HOST_INCLUDES := \
-I$(ROOT_PATH)/src/ble/hci \
-I$(ROOT_PATH)/include/ble \

# ----------------------------------------------------------------------------
# objects
# ----------------------------------------------------------------------------
INCLUDE_PATHS += -I$(PRJ_ROOT_PATH)
INCLUDE_PATHS += -I$(HOST_INCLUDES)

DIRS_IGNORE := $(PRJ_ROOT_PATH)/gcc%
DIRS_IGNORE += $(PRJ_ROOT_PATH)/image%
DIRS_IGNORE += $(ROOT_PATH)/project/common/board/%
DIRS_IGNORE += $(ROOT_PATH)/project/common/board/$(shell echo $(CONFIG_BOARD))

DIRS_ALL := $(shell find $(PRJ_ROOT_PATH) $(ROOT_PATH)/project/common -type d)
DIRS := $(filter-out $(DIRS_IGNORE),$(DIRS_ALL))
DIRS += $(ROOT_PATH)/project/common/board/$(shell echo $(CONFIG_BOARD))
ifeq (,$(wildcard ../my_trace.c))
DIRS += $(ROOT_PATH)/project/bluetooth/trace
endif

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))
OBJS := $(addsuffix .o,$(SRCS))

# to avoid path name coincidence, dont use $(subst .c,,$(INSTRUMENTATION_LIST))
C_FNAMES := $(addsuffix .c,$(SRCS))
C_FILE_INSTRU := $(foreach i,$(C_FNAMES),$(foreach j,$(INSTRUMENTATION_LIST),$(if $(findstring $j,$i),$i)))
C_NAME_INSTRU := $(basename $(C_FILE_INSTRU))

define one_instrument
$(1).o : $(1).c
	$(Q)echo 'Override General Default Rule : Instrument file: $$<'
	$(Q)$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -finstrument-functions -o $$@ $$<
endef

# extra libraries searching path
# PRJ_EXTRA_LIBS_PATH :=

# extra libraries
# PRJ_EXTRA_LIBS :=

# extra header files searching path
# PRJ_EXTRA_INC_PATH :=

# extra symbols (macros)
# PRJ_EXTRA_SYMBOLS :=

# ----------------------------------------------------------------------------
# override project variables
# ----------------------------------------------------------------------------
# linker script file
#   - relative to "./"
#   - define your own "LINKER_SCRIPT" to override the default one
# LINKER_SCRIPT :=

# image config file
#   - relative to "../image/$(CONFIG_CHIP_TYPE)/", eg. "../image/xr806/"
#   - define your own "IMAGE_CFG" to override the default one
 IMAGE_CFG := ./image_wlan_ble.cfg

# image name, default to xr_system
# IMAGE_NAME :=

# project make rules
include $(PRJ_MAKE_RULES)

CONFIG_SYMBOLS += -D$(HCIOPTION)
CONFIG_SYMBOLS += -DCONFIG_ARM
