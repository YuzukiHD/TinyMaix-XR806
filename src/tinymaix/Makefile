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
LIBS := libtinymaix.a

DIRS := .

SRCS := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

OBJS := $(addsuffix .o,$(SRCS))

CC_FLAGS += -Wno-unused-variable -Wno-error=unused-but-set-variable -Wno-error=multichar -Wno-unused-but-set-variable -Wno-multichar

# library make rules
include $(LIB_MAKE_RULES)
