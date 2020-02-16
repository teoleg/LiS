Makefile_ident="@(#) LiS modules/Makefile 1.3 09/09/04"

include .config_mk

default: modules 
	$(nothing)

HERE	= $(shell pwd)
SD	= marshall/
$(shell mkdir -p $(SD))

#
# List of modules consists of anything left behind in the various
# build directories plus anything that was been copied into this directory.
#
MODULES   := $(shell find $(SRCDIR) -name .modules -exec cat {} \;)
MODULES   += $(addprefix modules/,$(wildcard *.o))
#
# All of the object code files for modules with full pathnames
#
obj-m := $(addprefix $(HERE)/$(SD),$(notdir $(MODULES)))

# construct the old hardcoded dest directory, for comparison
#
OLD_INST_DIR = $(dir $(MOD_INST_DIR:%/=%))misc

#
# so that a make install does the same thing as make modules_install
#
install:	modules_install

ifneq ($(KBUILD),y)		############################## !KBUILD

RECURSIVE_MAKE	=

#
# default module subdir may have changed from .../misc - if so,
# we'll remove old modules
#
ifneq ($(MOD_INST_DIR),$(OLD_INST_DIR))
ifeq ($(DESTDIR),)	# we have no DESTDIR
OLD_MODULES = $(wildcard $(addprefix $(OLD_INST_DIR)/,$(notdir $(MODULES))))
endif
endif

modules_install: $(obj-m)
ifneq ($(MOD_INST_DIR),$(OLD_INST_DIR))
ifneq ($(OLD_MODULES),)
	-$(Q)rm -f $(OLD_MODULES)
endif
endif

# 18 Jan 2005 MarkS@Adax - Added DESTDIR to 'install $(obj-m)' line below
#	to support installations where DESTDIR is not null
	$(Q_ECHO) $(qtag_INSTALL)"[modules]" $(relpwd)
	$(Q)install -d $(DESTDIR)$(MOD_INST_DIR)
	$(Q)install $(obj-m) $(DESTDIR)$(MOD_INST_DIR)/.
#	symlink to the installed modules directory
# 18 Jan 2005 MarkS@Adax - It's important that the symlink use relative
#	pathing to support cross platform development where we are installing
#	to an NFS hosted/virtual rooted filesystem.
	$(Q)install -d $(DESTDIR)$(pkgdatadir)/linux-$(KVER)
	$(Q)(cd $(DESTDIR)$(pkgdatadir)/linux-$(KVER) && \
		$(LN) ../../../..$(MOD_INST_DIR) modules && rm -f modules/misc)


else	##################################### !KBUILD

# 23 Aug 2007 MarkS@Adax - Added CC= and LD= to the definition of 
# RECURSIVE_MAKE to support proper cross compilation and linking
# based on the definition of CC and LD that comes from config.mk 
# (included via .config_mk above).  Prior to this change CC and LD
# were undefined in Makefile.2.6 which meant that they were also 
# undefined when Makefile.modpost was called via Makefile.2.6.
RECURSIVE_MAKE	= $(Q)make -f Makefile.2.6 KSRC=$(KSRC) SD=$(SD) V=$(V) CC=$(CC) LD=$(LD)
$(shell mkdir -p scripts/mod $(SD) $(SD)/scripts)

#
# default module subdir may have changed from .../misc - if so,
# we'll remove old modules
#
ifneq ($(MOD_INST_DIR),$(OLD_INST_DIR))
ifeq ($(DESTDIR),)	# we have no DESTDIR
OLD_MODULES = $(wildcard $(addprefix $(OLD_INST_DIR)/,$(notdir $(MODULES:.o=.ko))))
endif
endif

# we make modules_install depend on the .ko files that result from the
# 'modules' target instead of the 'modules' target itself, to avoid
# remaking them if they already exist
#
modules_install: $(obj-m:.o=.ko)
ifneq ($(MOD_INST_DIR),$(OLD_INST_DIR))
ifeq ($(DESTDIR),)	# we have no DESTDIR
ifneq ($(OLD_MODULES),)
	-$(Q)rm -f $(OLD_MODULES)
endif
endif
endif

	$(Q_ECHO) $(qtag_INSTALL)"[modules]" $(relpwd)
	$(Q)install -d $(DESTDIR)$(MOD_INST_DIR)
	$(Q)install $(obj-m:.o=.ko) $(DESTDIR)$(MOD_INST_DIR)/.
#	symlink to the installed modules directory
# 18 Jan 2005 MarkS@Adax - It's important that the symlink use relative
#	pathing to support cross platform development where we are installing
#	to an NFS hosted/virtual rooted filesystem.
	$(Q)install -d $(DESTDIR)$(pkgdatadir)/linux-$(KVER)
	$(Q)(cd $(DESTDIR)$(pkgdatadir)/linux-$(KVER) && \
		$(LN) ../../../..$(MOD_INST_DIR) modules && rm -f modules/misc)

endif	#################################### KBUILD

# 18 Jan 2005 MarkS@Adax - Added uninstall and modules_uninstall targets
# to undo what is done in the above install and modules_install targets.
# Note: this change also required commenting out the section of the 
# primary Makefile that performed the rm of kernel loadable modules
# from $(DESTDIR)$(MOD_INST_DIR) now that the process is handled here.
# 08 Feb 2005 MarkS@Adax - uninstall now also takes care of removing 
# .ko files if KBUILD is set to y (i.e. 2.6 kernels) or .o files if 
# KBUILD is not set.
#
# so that a make uninstall does the same thing as make modules_uninstall
#
uninstall:	modules_uninstall

modules_uninstall:
	$(Q)(cd $(DESTDIR)$(pkgdatadir)/linux-$(KVER) && \
		rm -f modules)
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(pkgdatadir)/linux-$(KVER)
ifneq ($(KBUILD),y)		############################## !KBUILD
	$(Q)rm -f $(addprefix $(DESTDIR)$(MOD_INST_DIR)/,$(notdir $(obj-m)))
else
	$(Q)rm -f $(addprefix $(DESTDIR)$(MOD_INST_DIR)/,$(notdir $(obj-m:.o=.ko)))
endif
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(MOD_INST_DIR)

#
# the following just makes the input .o files local.  It can either copy
# them, or symlink to them
#
# For 2.6 we then invoke a "recursive" make to process all the files
#
modules: FORCE
	$(Q)cp $(addprefix $(SRCDIR)/,$(MODULES)) $(SD)
	$(RECURSIVE_MAKE)

clean:
	$(Q_ECHO) $(qtag_CLEAN)$(relpwd)
	-$(Q)rm -f *.ko *.o *.mod.o *.mod.c ./.*.cmd *symvers *.i *.s
	-$(Q)rm -rf .tmp_versions $(SD) scripts

FORCE:
