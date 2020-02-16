include .config_mk

# OBJECTS and MODULES are the output .o files produced in this directory
# _only_.  Their paths relative to SRCDIR are saved to .objects and
# .modules, resp., after the named files are built.  Other Makefiles can
# then find and use these files appropriately.
#
# note that each item included in MODULES will be built as its own loadable
# module.  Each should be named with a 'streams-' prefix to prevent module
# name conflicts
#
OBJECTS	= Space.o
MODULES	=

# we use the following to make .object paths relative to the parent of
# this directory, instead of SRCDIR
#
relparent = $(call _relpath,$(shell cd ..; pwd))

ifeq ($(ARCH),x86_64)
XOPTS += -mno-red-zone -mcmodel=kernel 
endif

ifeq ($(ARCH),mips)
XOPTS += -nostdinc 
XOPTS += -isystem $(shell $(CC) -print-file-name=include)
XOPTS += -fno-strict-aliasing -fno-common -ffreestanding 
XOPTS += -mno-abicalls -fno-pic -msoft-float -march=octeon -Wa,--trap 
XOPTS += -mabi=64 -mlong-calls -G 0
#XOPTS += -fno-pic -mno-abicalls -fno-common -mlong-calls -msoft-float -G 0
endif

ifeq ($(ARCH),ppc)
XOPTS += -I$(KSRC)/arch/ppc
endif

CC := $(CC) -D__KERNEL__ $(XOPTS) -I$(LIS_INCL)/sys/LiS

ifneq ($(MODULES),)
MODOPT = -DMODULE
ifeq ($(KBUILD_NEEDS_MODNAME),y)
  MODOPT += -DKBUILD_MODNAME=$(*F:%=streams-%)
endif
endif

#
# Prototype makefile for package subdirectories
#
default: all
	$(nothing)

all: 	Space.o
	$(nothing)

modules: .modules
	$(nothing)

.objects: $(OBJECTS)
	$(Q)echo $(relpwddeps) > $@

.modules: $(MODULES)
	$(Q)echo $(relpwddeps) > $@

ifneq ($(OBJECTS),)
$(OBJECTS): %.o: %.c
	$(Q_ECHO) $(qtag_CC)$(relpwdtarget)
	$(Q)$(CC) -o $@ -c $<
endif

ifneq ($(MODULES),)
$(MODULES): streams-%.o: %.c
	$(Q_ECHO) $(qtag_CCm)$(relpwdtarget)
	$(Q)$(CC) $(MODOPT) -DLIS_OBJNAME=$* -o $@ -c $<
endif

clean: FORCE
	$(Q_ECHO) $(qtag_CLEAN)$(relpwd)
	-$(Q)rm -f $(OBJECTS) $(MODULES) .objects .modules


FORCE:


# dependencies

$(OBJECTS): $(LIS_INCL)/sys/LiS/config.h
