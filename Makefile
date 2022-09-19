# Makefile for buildroot2
#
# Copyright (C) 1999-2005 by Erik Andersen <andersen@codepoet.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#

#--------------------------------------------------------------
# Just run 'make menuconfig', configure stuff, then run 'make'.
# You shouldn't need to mess with anything beyond this point...
#--------------------------------------------------------------

Kconfig := ./Kconfig
MCONF := ./tools/config/mconf
CONF := ./tools/config/conf

KCONFIG_CONFIG := .config
export KCONFIG_CONFIG

KCONFIG_AUTOHEADER := include/generated/autoconf.h
export KCONFIG_AUTOHEADER

-include $(KCONFIG_CONFIG)

PRJ ?= ""

ifneq ($(PRJ), "")
_PROJECT := project/$(PRJ)
else
_PROJECT := project/$(CONFIG_PROJECT)
endif

PHONY :=

__all: all

PHONY += help
help:
	@echo "Welcome to Xradio build system. Some useful make targets:"
	@echo ""
	@echo "make menuconfig - Configure project"
	@echo ""
	@echo "make lib - Build all libraries"
	@echo "make build - Build image"
	@echo "make build_clean - Build clean"
	@echo "make objdump - Generate objdump for debug"


dot-target = $(dir $@).$(notdir $@)

PHONY += FORCE

FORCE:

gcc_tools_unzip: FORCE
ifeq ($(shell uname -o), Cygwin)
	@if ! [ -d ~/tools/gcc-arm-none-eabi-8-2019-q3-update ]; then \
		mkdir -p ~/tools; \
		unzip ./../tools/toolchain/gcc-arm-none-eabi/gcc-arm-none-eabi-8-2019-q3-update.zip -d ~/tools; \
	fi
else
	@if ! [ -d ~/tools/gcc-arm-none-eabi-8-2019-q3-update ]; then \
		mkdir -p ~/tools; \
		tar -jxvf ./../tools/toolchain/gcc-arm-none-eabi/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2 -C ~/tools; \
	fi
endif

dot_config: $(CONF) FORCE
	@set -e; \
	trap "rm -f ..config.tmp" EXIT; \
	mv .config ..config.tmp; \
	if [ -f .defconfig.tmp ]; then \
		mv .defconfig.tmp .config; \
	else \
		cp ..config.tmp .config; \
	fi; \
	$(CONF) --olddefconfig $(Kconfig); \
	if ! cmp -s .config ..config.tmp; then	\
		echo '  UPD     .config';			\
		rm -f ..config.tmp; \
	else \
		echo '  NUPD    .config'; \
		mv ..config.tmp .config; \
	fi

include/generated/autoconf.h: $(CONF) dot_config FORCE
	@set -e; \
	trap "rm -f $(dot-target).tmp" EXIT; \
	if [ -f $@ ]; then \
		mv $@ $(dot-target).tmp; \
	else \
		echo "" > $(dot-target).tmp; \
	fi; \
	$(CONF) --syncconfig $(Kconfig); \
	if ! cmp -s $@ $(dot-target).tmp; then \
		echo '  UPD     $@'; \
		rm -f $(dot-target).tmp; \
	else \
		echo '  NUPD    $@'; \
		mv $(dot-target).tmp $@; \
	fi

PHONY += $(MCONF)
$(MCONF):
	@cd $(@D); $(MAKE) $(@F); cd -

PHONY += $(CONF)
$(CONF):
	@cd $(@D); $(MAKE) $(@F); cd -

menuconfig: $(MCONF)
	$< $(Kconfig)

defconfig: $(CONF)
	@if [ -d project/$(PRJ) ] && [ -f project/$(PRJ)/gcc/defconfig ]; then \
		cp $(_PROJECT)/gcc/defconfig .config; \
	fi; \
	$< --defconfig ./.config $(Kconfig)

config: $(CONF)
	$< $(Kconfig)

oldconfig: $(CONF)
	@$< --defconfig ./.config.old $(Kconfig)

define cd_prj_exe
	@set -e; \
	if [ -d $(1) ]; then \
		cd $(1)/gcc/; make $@; cd -; \
	else \
		echo "$(1) not find"; \
	fi
endef

all: FORCE
	$(call cd_prj_exe, $(_PROJECT))

lib: FORCE
	$(call cd_prj_exe, $(_PROJECT))

lib_clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))

install: FORCE
	$(call cd_prj_exe, $(_PROJECT))

lib_install_clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))

build: FORCE
	$(call cd_prj_exe, $(_PROJECT))

image: FORCE
	$(call cd_prj_exe, $(_PROJECT))

image_clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))

image_xz: FORCE
	$(call cd_prj_exe, $(_PROJECT))

ifeq ($(CONFIG_SECURE_BOOT), y)
sign: FORCE
	$(call cd_prj_exe, $(_PROJECT))
endif

ifeq ($(CONFIG_TRUSTZONE), y)
trustzone: FORCE
	$(call cd_prj_exe, $(_PROJECT))

trustzone_clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))
endif

size: FORCE
	$(call cd_prj_exe, $(_PROJECT))

build_clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))

objdump: FORCE
	$(call cd_prj_exe, $(_PROJECT)); \
	cp $(_PROJECT)/gcc/*.objdump out/

clean: FORCE
	$(call cd_prj_exe, $(_PROJECT))

PHONY += config_clean
config_clean: FORCE
	$(Q)-rm -f .config .config.old
	$(Q)-rm -rf include/generated/* include/config/*
	$(Q)cd ./tools/config/; $(MAKE) clean; cd -

# Declare the contents of the PHONY variable as phony.  We keep that
# information in a variable so we can use it in if_changed and friends.
.PHONY: $(PHONY)
