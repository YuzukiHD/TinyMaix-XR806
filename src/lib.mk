#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common targets and building rules
# ----------------------------------------------------------------------------
INSTALL_PATH ?= $(ROOT_PATH)/lib

.PHONY: libconfig prjconfig size clean install_clean

all: $(LIBS)

$(LIBS): $(OBJS)
	$(Q)$(AR) -crs $@ $^

libconfig:
	@set -e; \
	if [ -f $(ROOT_PATH)/.config ]; then \
		mv $(ROOT_PATH)/.config ..config.tmp; \
		cp ..config.tmp $(ROOT_PATH)/.config; \
	fi; \
	if [ -f $(ROOT_PATH)/include/generated/autoconf.h ]; then \
		mv $(ROOT_PATH)/include/generated/autoconf.h .autoconf.h.tmp; \
		cp .autoconf.h.tmp $(ROOT_PATH)/include/generated/autoconf.h; \
	fi; \
	if [ -f ./defconfig ]; then \
		cp ./defconfig $(ROOT_PATH)/.config; \
	else \
		cp $(LIB_ROOT_PATH)/defconfig $(ROOT_PATH)/.config; \
	fi
	@cd $(ROOT_PATH) && make include/generated/autoconf.h && cd -

prjconfig:
	@set -e; \
	if [ -f ..config.tmp ]; then \
		mv ..config.tmp $(ROOT_PATH)/.config; \
	fi; \
	if [ -f .autoconf.h.tmp ]; then \
		mv .autoconf.h.tmp $(ROOT_PATH)/include/generated/autoconf.h; \
	fi

install: $(LIBS)
	$(Q)$(CP) -t $(INSTALL_PATH) $^

size:
	$(Q)$(SIZE) -t $(LIBS)

objdump: $(LIBS)
	$(Q)$(OBJDUMP) -Sdh $< > $(basename $(LIBS)).objdump

clean:
	$(Q)-rm -f $(LIBS) $(OBJS) $(DEPS) *.objdump

install_clean:
	$(Q)-rm -f $(INSTALL_PATH)/$(LIBS)

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
