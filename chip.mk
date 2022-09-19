
make_configing = y
ifneq ($(MAKECMDGOALS), config)
ifneq ($(MAKECMDGOALS), oldconfig)
ifneq ($(MAKECMDGOALS), menuconfig)
ifneq ($(MAKECMDGOALS), config_cpy)
ifneq ($(MAKECMDGOALS), config_clean)
  make_configing = y
endif
endif
endif
endif
endif

#
# chip definition
#

# ----------------------------------------------------------------------------
# chip type
# ----------------------------------------------------------------------------
ifeq ($(ROOT_PATH)/.config, $(wildcard $(ROOT_PATH)/.config))
else
  ifeq ($(make_configing), n)
    $(info ERROR:"copy a defconfig to this project .config from project defconfig or tools/config/xx_defconfig!")
  endif
endif

-include $(ROOT_PATH)/.config

# External high speed crystal oscillator
ifeq ($(CONFIG_HOSC_TYPE_24M), y)
  CONFIG_HOSC_TYPE = "24"
else ifeq ($(CONFIG_HOSC_TYPE_26M), y)
  CONFIG_HOSC_TYPE = "26"
else ifeq ($(CONFIG_HOSC_TYPE_32M), y)
  CONFIG_HOSC_TYPE = "32"
else ifeq ($(CONFIG_HOSC_TYPE_40M), y)
  CONFIG_HOSC_TYPE = "40"
endif

ifeq ($(CONFIG_CHIP_XR806), y)
  CONFIG_CHIP_TYPE = "xr806"
endif

# ----------------------------------------------------------------------------
# chips of arch version 3
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_CHIP_XR806), y)
  CONFIG_CHIP_ARCH_VER := 3
  CONFIG_CPU_ARCH := CM33F
  CONFIG_CPU_CM33F = y
  CONFIG_CHIP_RAM_SIZE = 336
endif

CONFIG_SYMBOLS += -DCONFIG_CHIP_ARCH_VER=$(CONFIG_CHIP_ARCH_VER)

# ----------------------------------------------------------------------------
# arch and core
# ----------------------------------------------------------------------------
CONFIG_ARCH_DUAL_CORE := n
ifeq ($(CONFIG_ARCH_DUAL_CORE), y)
  CONFIG_SYMBOLS += -DCONFIG_ARCH_DUAL_CORE
endif

CONFIG_ARCH_APP_CORE := y
ifeq ($(CONFIG_ARCH_APP_CORE), y)
  CONFIG_SYMBOLS += -DCONFIG_ARCH_APP_CORE
endif

CONFIG_ARCH_NET_CORE := n
ifeq ($(CONFIG_ARCH_NET_CORE), y)
  CONFIG_SYMBOLS += -DCONFIG_ARCH_NET_CORE
endif

# ----------------------------------------------------------------------------
# cpu
# ----------------------------------------------------------------------------
ifeq ($(CONFIG_CPU_ARCH), CM4F)
  CONFIG_SYMBOLS += -DCONFIG_CPU_CM4F
else ifeq ($(CONFIG_CPU_ARCH), CM33F)
  CONFIG_SYMBOLS += -DCONFIG_CPU_CM33F
else
  ifeq ($(make_configing), n)
	$(info ERROR:"select CPU ARCH!")
  endif
endif

#cpu
ifeq ($(CONFIG_CPU_ARCH)_$(CONFIG_TRUSTZONE), CM33F_y)
  CONFIG_OS_FREERTOS_PLATFORM ?= ARM_CM33
else ifeq ($(CONFIG_CPU_ARCH), CM33F)
  CONFIG_OS_FREERTOS_PLATFORM ?= ARM_CM33_NTZ
else
  CONFIG_OS_FREERTOS_PLATFORM ?= ARM_CM4F
endif

# ----------------------------------------------------------------------------
# chip configuration check
# ----------------------------------------------------------------------------
ifeq ($(make_configing), n)

  ifndef CONFIG_CHIP_TYPE
    __nullstring :=
    $(info ERROR:)
    $(info $(__nullstring)  Chip is not defined!)
    $(info $(__nullstring)  Please run `make menuconfig` in your project.)
    $(error )
  endif

  ifndef CONFIG_HOSC_TYPE
    __nullstring :=
    $(info ERROR:)
    $(info $(__nullstring)  External high speed crystal oscillator is not defined!)
    $(info $(__nullstring)  Please run `make menuconfig` in your project.)
    $(error )
  endif

  ifndef CONFIG_CHIP_ARCH_VER
    $(error Invalid chip configuration!)
  endif

endif
