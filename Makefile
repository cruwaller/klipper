# Klipper build system
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Output directory
OUT=out/

# Kconfig includes
export HOSTCC             := $(CC)
export CONFIG_SHELL       := sh
export KCONFIG_AUTOHEADER := autoconf.h
export KCONFIG_CONFIG     := $(CURDIR)/.config
-include $(KCONFIG_CONFIG)

ifneq ($(TOOLSDIR),)
  GCC_PATH=$(TOOLSDIR)/$(CROSS_PREFIX)
else
  GCC_PATH=$(CROSS_PREFIX)
endif
# Common command definitions
CC  = $(GCC_PATH)gcc
AS  = $(GCC_PATH)as
LD  = $(GCC_PATH)ld
OBJCOPY = $(GCC_PATH)objcopy
OBJDUMP = $(GCC_PATH)objdump
STRIP   = $(GCC_PATH)strip
CPP = cpp # $(GCC_PATH)cpp
PYTHON  = python2

# Source files
src-y =
dirs-y =

# Default compiler flags
cc-option=$(shell if test -z "`$(1) $(2) -S -o /dev/null -xc /dev/null 2>&1`" \
    ; then echo "$(2)"; else echo "$(3)"; fi ;)

CFLAGS := -I$(OUT) -Isrc -I$(OUT)board-generic/ -std=gnu11 -O2 -MD -g \
    -Wall -Wold-style-definition $(call cc-option,$(CC),-Wtype-limits,) \
    -ffunction-sections -fdata-sections
CFLAGS += -flto -fwhole-program -fno-use-linker-plugin # -Wextra

CFLAGS_klipper.elf = $(CFLAGS) -Wl,--gc-sections

CPPFLAGS = -I$(OUT) -P -MD -MT $@

# Default targets
target-y := $(OUT)klipper.elf

all:

# Run with "make V=1" to see the actual compile commands
ifdef V
Q=
else
Q=@
MAKEFLAGS += --no-print-directory
endif

# Include board specific makefile
include src/Makefile
proc_makefile = src/$(patsubst "%",%,$(CONFIG_BOARD_DIRECTORY))/Makefile
-include $(proc_makefile)

################ Convert sources to objects

objects-y = $(src-y:%.c=$(OUT)src/%.o)
objects-y := $(objects-y:%.cpp=$(OUT)src/%.o)
objects-y := $(objects-y:%.S=$(OUT)src/%.o)
objects-y := $(objects-y:%.s=$(OUT)src/%.o)
objects-y += $(asmsrc-y:%.s=$(OUT)src/%.o)
objects-y := $(objects-y:%.S=$(OUT)src/%.o)

# parse only c/cpp files
c-files := $(filter %.c %.cpp,$(src-y))
ctr-y = $(c-files:%.c=$(OUT)src/%.o.ctr)
ctr-y := $(ctr-y:%.cpp=$(OUT)src/%.o.ctr)

# Collect all object dirs
dirs-y := $(dir $(objects-y))

################ Common build rules

$(OUT)%.o: %.c $(OUT)autoconf.h $(OUT)board-link
	@echo "  Compiling $@"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

################ Main build rules

$(OUT)board-link: $(KCONFIG_CONFIG) $(proc_makefile) src/Makefile ./Makefile
	@echo "  Creating symbolic link $(OUT)board"
	$(Q)mkdir -p $(dirs-y)
	$(Q)touch $@
	$(Q)ln -Tsf $(PWD)/src/$(CONFIG_BOARD_DIRECTORY) $(OUT)board
	$(Q)mkdir -p $(OUT)board-generic
	$(Q)ln -Tsf $(PWD)/src/generic $(OUT)board-generic/board

$(OUT)%.o.ctr: $(OUT)%.o
	$(Q)$(OBJCOPY) -j '.compile_time_request' -O binary $^ $@

$(OUT)compile_time_request.o: $(ctr-y) ./scripts/buildcommands.py
	@echo "  Building $@"
	$(Q)cat $(ctr-y) > $(OUT)klipper.compile_time_request
	$(Q)$(PYTHON) ./scripts/buildcommands.py -d $(OUT)klipper.dict -t "$(CC);$(AS);$(LD);$(OBJCOPY);$(OBJDUMP);$(STRIP)" $(OUT)klipper.compile_time_request $(OUT)compile_time_request.c
	$(Q)$(CC) $(CFLAGS) -c $(OUT)compile_time_request.c -o $@

$(OUT)klipper.elf: $(OUT)autoconf.h $(objects-y) $(OUT)compile_time_request.o
	@echo "  Linking $@"
	$(Q)$(CC) $^ $(CFLAGS_klipper.elf) -o $@

################ Kconfig rules

define do-kconfig
$(Q)rm -rf $(OUT)
$(Q)mkdir -p $(OUT)/scripts/kconfig/lxdialog
$(Q)mkdir -p $(OUT)/include/config
$(Q)$(MAKE) -C $(OUT) -f $(CURDIR)/scripts/kconfig/Makefile srctree=$(CURDIR) src=scripts/kconfig obj=scripts/kconfig Q=$(Q) Kconfig=$(CURDIR)/src/Kconfig $1
endef

$(OUT)autoconf.h : $(KCONFIG_CONFIG) ; $(call do-kconfig, silentoldconfig)
$(KCONFIG_CONFIG): src/Kconfig ; $(call do-kconfig, olddefconfig)
%onfig: ; $(call do-kconfig, $@)
help: ; $(call do-kconfig, $@)


################ Generic rules

# Make definitions
.PHONY : all clean distclean FORCE
.DELETE_ON_ERROR:

all: $(target-y)

clean:
	$(Q)rm -rf $(OUT)

distclean: clean
	$(Q)rm -f .config .config.old

# include dependency files
-include $(objects-y:%.o=%.d) # $(OUT)*.d $(patsubst %,$(OUT)%/*.d,$(dirs-y))
