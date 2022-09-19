#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../../../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := libfreertos.a

ifneq ($(CONFIG_ROM_FREERTOS), y)
ifeq ($(CONFIG_TRUSTZONE), y)
  RTOS_DIRS_IGNORE := $(shell find ./Source/portable/GCC/$(CONFIG_OS_FREERTOS_PLATFORM)/secure -type d)
endif
RTOS_DIRS := ./Source $(shell find ./Source/portable/GCC/$(CONFIG_OS_FREERTOS_PLATFORM) -type d)

DIRS := $(filter-out $(RTOS_DIRS_IGNORE),$(RTOS_DIRS))

SRCS := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

SRCS += ./Source/portable/MemMang/heap_adapt
else
SRCS := $(sort $(basename Patch/tasks.c Patch/port.c))
endif

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
