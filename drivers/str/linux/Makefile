include .config_mk

#
#  Makefile for LiS drivers
#
#  This one builds drivers for the Linux kernel.
#
#  FIXME - some of these modules (e.g., relay3, inet) can now be moved
#  to a pkg/ subdir of their own. since modules can now be built from
#  there
#
-include $(SRCDIR)/drvrconf.mk

OPT = -DLINUX -D__KERNEL__
MODOPT = -DMODULE
ifeq ($(KBUILD_NEEDS_MODNAME),y)
	MODOPT += -DKBUILD_MODNAME=$(*F:%=streams-%)
endif

# This is necessary due to some dumbass breaking spinlock.h just so that you
# can call your module the same name as a kernel symbol.
# These defines must be added to XOPTS because they are needed whenever
# these files are compiled, not just when they are compiled as modules.
ifeq ($(KBUILD_NEEDS_MODNAME_REDEF),y)
	SNAME=$(*F:%=streams-%)
	XOPTS += -D"KBUILD_STR(s)=\#s" \
	         -D"KBUILD_BASENAME=KBUILD_STR(${SNAME})"  \
	         -D"KBUILD_MODNAME=KBUILD_STR(${SNAME})"
endif

#ifeq ($(DBG_OPT),y)
#  OPT += -DDEBUG
#endif

ifdef KXEN
XEN_OPTS += -nostdinc 
XEN_OPTS += -isystem $(shell $(CC) -print-file-name=include)
XEN_OPTS += -D__XEN_INTERFACE_VERSION__=$(KXEN) -I$(KSRC)/include/asm/mach-xen 
ifeq ($(ARCH),x86_64)
# include/asm/mach-xen got moved in 2.6.28 commit bb898558
XEN_OPTS += -I$(KSRC)/arch/x86/include/mach-xen
endif
XOPTS := $(XEN_OPTS) $(XOPTS)
endif

# For 64 bit x86 architectures we need to specify the "memory model" (ugh..)
# dg@adax.com Feb 11, 2005.

ifeq ($(ARCH),x86_64)
XOPTS += -mno-red-zone -mcmodel=kernel 
endif

ifeq ($(ARCH),mips)
XOPTS += -nostdinc 
XOPTS += -isystem $(shell $(CC) -print-file-name=include)
XOPTS += -fno-strict-aliasing -fno-common -ffreestanding 
XOPTS += -mno-abicalls -fno-pic -msoft-float -march=octeon -Wa,--trap 
XOPTS += -mabi=64 -mlong-calls -G 0
#XOPTS += -fno-pic -mno-abicalls -mlong-calls -msoft-float -G 0
endif

ifeq ($(ARCH),ppc)
XOPTS += -I$(KSRC)/arch/ppc
endif

# Object names known to all targets
OBJS := fifo loop relay clone minimux printk sad pipemod

# Object names only known to the Linux kernel target.
TOBJS := ip_strm_mod connld mtdrv
#
# FIXME - move inet to the pkg/ directory in a subdir of its own; that
# should work now...
#
ifeq ($(CONFIG_INET),y)
ifeq ($(IPV6_MODULE),y)
TOBJS += inet
INET_INST=inet_install
INET_CLEAN=inet_clean
ifeq ($(DBG_OPT),y)
XOPTS += -D_SAFE
endif
endif				# IPV6_MODULE
endif				# CONFIG_INET

ifneq ($(KVER_MAJORMINOR),2.2)
TOBJS += ip_to_dlpi
endif
ifeq ($(ARCH),s390)
TOBJS := connld
endif
ifeq ($(ARCH),s390x)
TOBJS := connld
endif


# Similar to TOBJS, but non-streams modules
ZOBJS := 

# Remove unknown object names
LINKS := $(filter $(OBJS),$(MODCONF_LINKS))
LOADS := $(filter $(OBJS),$(MODCONF_LOADS))
TLINKS := $(filter $(TOBJS),$(MODCONF_LINKS))
TLOADS := $(filter $(TOBJS),$(MODCONF_LOADS))
ZLOADS := $(ZOBJS)

# Add back Linux-only modules not in the Config file
TLOADS += relay3

# Linked object file names
DRVRS := $(LINKS:%=%.o)
TDRVRS := $(TLINKS:%=%.o)

# Loadable object file names
MODLS := $(LOADS:%=streams-%.o)
TMODLS := $(TLOADS:%=streams-%.o)
ZMODLS := $(ZLOADS:%=%.o)

#
# Module support
#
ifdef CONFIG_MODULES

ifdef CONFIG_MODVERSIONS
OPT += -DMODVERSIONS
endif 

# FIXME: are DMODLS and MODULES the same?

MODULES	= $(MODLS) $(TMODLS) $(ZMODLS)

else   # not CONFIG_MODULES

ifneq ($(strip $(MODLS) $(TMODLS) $(ZMODLS)),)
first_rule:
	@echo "Configured loadables, but no kernel module support." >&2
	@/bin/false
endif  # ifdef MODLS

endif  # ifdef CONFIG_MODULES

# 20 July 2009: MarkS@Adax - do this last to ensure vars are set before use
CC := $(CC) -DLiS $(CCREGPARM) $(OPT) $(XOPTS)

# How to compile a loadable only known here ( $(TMODLS) )
streams-%.o: %.c
	$(Q_ECHO) $(qtag_CCm)$(relpwdtarget)
	$(Q)$(CC) $(MODOPT) -DLIS_OBJNAME=$* -o $@ -c $<

# How to compile a loadable known to all targets ( $(MODLS) )
streams-%.o: $(DRVRDIR)/%.c
	$(Q_ECHO) $(qtag_CCm)$(relpwdtarget)
	$(Q)$(CC) $(MODOPT) -DLIS_OBJNAME=$* -o $@ -c $<

%.o: %.c
	$(Q_ECHO) $(qtag_CCm)$(relpwdtarget)
	$(Q)$(CC) $(MODOPT) -DLIS_OBJNAME=$* -o $@ -c $<


# the following uses variables, but defines targets and rules, including
# a 'default' rule which should be first (with the exception of first_rule
# above).
#
# so, variables should be defined before this include, as should rules to
# override, but additional targets and rules after
#
include $(DRVRDIR)/mk.bdy


# A target that makes modules

modules: .modules
	$(nothing)

.modules: $(MODULES)
	$(Q)echo $(relpwddeps) > $@

# 08 Feb 2005 MarkS@Adax - added relay3.c to clean target as it is a 
#	generated file
clean: common-clean $(INET_CLEAN)
	$(Q_ECHO) $(qtag_CLEAN)$(relpwd)
	-$(Q)rm -f $(OBJS:%=streams-%.o)
	-$(Q)rm -f $(TOBJS:%=streams-%.o)
	-$(Q)rm -f $(ZOBJS:%=%.o)
	-$(Q)rm -f .modules
	-$(Q)rm -f relay3.c relay3.i relay3.s
	-$(Q)rm -f $(OBJS:%=%.i)
	-$(Q)rm -f $(OBJS:%=%.s)
	-$(Q)rm -f $(TOBJS:%=%.i)
	-$(Q)rm -f $(TOBJS:%=%.s)
	-$(Q)rm -f $(ZOBJS:%=%.i)
	-$(Q)rm -f $(ZOBJS:%=%.s)

realclean: clean common-realclean
	-$(Q)rm -f .depend.modules

install: common-install $(INET_INST)
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)

uninstall:	common-uninstall
	$(nothing)

#
# Install and clean for inet
#
inet_install:
	$(Q_ECHO) $(qtag_INSTALL)"[inet]"
	-$(Q)(cd $(SRCDIR)/scripts; ./inet_mkdev)

inet_clean:
	$(Q_ECHO) $tag_CLEAN)"[inet]"
	-$(Q)(cd $(SRCDIR)/scripts; ./inet_rmdev)

#
# Common rules
#


#
# Rule for relay3, a special compilation of relay.c
#
streams-relay3.o: relay3.c
	$(Q_ECHO) $(qtag_CCm)$(relpwdtarget)
	$(Q)$(CC) $(MODOPT) -DLIS_OBJNAME=$* -DRELAY3 -o $@ -c $<
#	-$(Q)rm -f $<

relay3.c: $(DRVRDIR)/relay.c
#	-$(Q)rm -f $@
	$(Q)cp $? $@

#
# Dependency stuff
#

-include .depend.modules

dep: relay3.c common-dep
	$(Q_ECHO) $(qtag_DEP)$(relpwd)
ifneq ($(strip $(MODLS) $(TMODLS)),)
	$(Q)$(CC) $(MODOPT) -M $(MODLS:streams-%.o=$(DRVRDIR)/%.c) \
 $(TMODLS:streams-%.o=%.c) >.depend.modules
else
	$(Q)>.depend.modules
endif

