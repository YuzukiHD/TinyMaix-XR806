#
# Rules for building application
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# project name and project root path
# ----------------------------------------------------------------------------
PROJECT := $(notdir $(shell cd .. && pwd))
PRJ_PARENT_DIR :=
PRJ_ROOT_PATH := $(ROOT_PATH)/project/$(PRJ_PARENT_DIR)/$(PROJECT)

# ----------------------------------------------------------------------------
# objects
# ----------------------------------------------------------------------------
INCLUDE_PATHS += -I$(PRJ_ROOT_PATH)

DIRS := ..
DIRS += $(ROOT_PATH)/project/common/startup/gcc
DIRS += $(ROOT_PATH)/project/common/board
DIRS += $(ROOT_PATH)/project/common/board/$(shell echo $(CONFIG_BOARD))

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

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

# project make rules
include $(PRJ_MAKE_RULES)
