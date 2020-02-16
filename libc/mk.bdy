#
# common makefile for all environments
#
# This file is included from a target-specific makefile
# in a subdirectory below this directory.
# The makefile including this file should have defined:
#   - The LiS configuration.
#     config.mk in the top of the source tree should
#     have been included.
#   - LIBOBJS variable.
#     Contains the names of the object files that should be in the
#     library.
#

LIBFILE_SHARED = lib$(LIBBASE).so
LIBFILE_PSHARED = libp$(LIBBASE).so
LIBFILE_STATIC = lib$(LIBBASE).a
LIBFILE_PSTATIC = libp$(LIBBASE).a

LIB_SHARED = $(LIBDIR)/$(LIBFILE_SHARED)
LIB_PSHARED = $(LIBDIR)/$(LIBFILE_PSHARED)
LIB_STATIC = $(LIBDIR)/$(LIBFILE_STATIC)
LIB_PSTATIC = $(LIBDIR)/$(LIBFILE_PSTATIC)

ifeq ($(V), 1)
CC += -fverbose-asm -save-temps
endif

CC += -c $(XOPTS)

all: $(Q_CC) $(LIB_SHARED) $(LIB_PSHARED) $(LIB_STATIC) $(LIB_PSTATIC)
	$(nothing)

$(LIB_SHARED): $(LIBOBJS)
	$(Q_ECHO) $(qtag_LD)$(reltarget)
	$(Q)$(LD) -shared -o $@ $^ -lc

$(LIB_PSHARED): $(PRELDOBJS)
	$(Q_ECHO) $(qtag_LD)$(reltarget)
	$(Q)$(LD) -shared -o $@ $^ -lc

$(LIB_STATIC): $(LIBOBJS)
	-$(Q)rm -f $@
	$(Q_ECHO) $(qtag_AR)$(reltarget)
	$(Q)$(AR) r $@ $^
	$(Q_ECHO) $(qtag_RANLIB)$(reltarget)
	$(Q)$(RANLIB) $@

$(LIB_PSTATIC): $(PRELDOBJS)
	-$(Q)rm -f $@
	$(Q_ECHO) $(qtag_AR)$(reltarget)
	$(Q)$(AR) r $@ $^
	$(Q_ECHO) $(qtag_RANLIB)$(reltarget)
	$(Q)$(RANLIB) $@

%.o: %.c
	$(Q_ECHO) $(qtag_CC)$(relpwdtarget)
	$(Q)$(CC) -o $@ $<

%.o: $(LIBDIR)/%.c
	$(Q_ECHO) $(qtag_CC)$(relpwdtarget)
	$(Q)$(CC) -o $@ $<


clean:
	$(Q_ECHO) $(qtag_CLEAN)$(relpwd)
	-$(Q)rm -f *.o .compiler
	-$(Q)rm -f $(LIB)
	-$(Q)rm -f *.s *.i

# 08 Feb 2005 MarkS@Adax - Added LIB_PSHARED to clean target
realclean: clean
	-$(Q)rm -f .depend
	-$(Q)rm -f $(LIB_SHARED)
	-$(Q)rm -f $(LIB_PSHARED)
	-$(Q)rm -f $(LIB_STATIC)
	-$(Q)rm -f $(LIB_PSTATIC)

install: .compiler all
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)
ifeq ($(ARCH),s390x)
	$(Q)install -d $(DEST_LIB64DIR)
	$(Q)install -m 0755 $(LIB_SHARED) $(DEST_LIB64DIR)/$(LIBFILE_SHARED)
	$(Q)install -m 0755 $(LIB_PSHARED) $(DEST_LIB64DIR)/$(LIBFILE_PSHARED)
	$(Q)install -m 0644 $(LIB_STATIC) $(DEST_LIB64DIR)/$(LIBFILE_STATIC)
	$(Q)install -m 0644 $(LIB_PSTATIC) $(DEST_LIB64DIR)/$(LIBFILE_PSTATIC)
else
ifeq ($(ARCH),x86_64)
	$(Q)install -d $(DEST_LIB64DIR)
	$(Q)install -m 0755 $(LIB_SHARED) $(DEST_LIB64DIR)/$(LIBFILE_SHARED)
	$(Q)install -m 0755 $(LIB_PSHARED) $(DEST_LIB64DIR)/$(LIBFILE_PSHARED)
	$(Q)install -m 0644 $(LIB_STATIC) $(DEST_LIB64DIR)/$(LIBFILE_STATIC)
	$(Q)install -m 0644 $(LIB_PSTATIC) $(DEST_LIB64DIR)/$(LIBFILE_PSTATIC)
else
ifeq ($(ARCH),mips)
	$(Q)install -d $(DEST_LIB64DIR)
	$(Q)install -m 0755 $(LIB_SHARED) $(DEST_LIB64DIR)/$(LIBFILE_SHARED)
	$(Q)install -m 0755 $(LIB_PSHARED) $(DEST_LIB64DIR)/$(LIBFILE_PSHARED)
	$(Q)install -m 0644 $(LIB_STATIC) $(DEST_LIB64DIR)/$(LIBFILE_STATIC)
	$(Q)install -m 0644 $(LIB_PSTATIC) $(DEST_LIB64DIR)/$(LIBFILE_PSTATIC)
else
	$(Q)install -d $(DEST_LIBDIR)
	$(Q)install -m 0755 $(LIB_SHARED) $(DEST_LIBDIR)/$(LIBFILE_SHARED)
	$(Q)install -m 0755 $(LIB_PSHARED) $(DEST_LIBDIR)/$(LIBFILE_PSHARED)
	$(Q)install -m 0644 $(LIB_STATIC) $(DEST_LIBDIR)/$(LIBFILE_STATIC)
	$(Q)install -m 0644 $(LIB_PSTATIC) $(DEST_LIBDIR)/$(LIBFILE_PSTATIC)
endif
endif
endif
ifeq ($(DESTDIR),)	# we have no DESTDIR, ok to run ldconfig
	$(Q)ldconfig
endif
	$(Q)install -d $(DESTDIR)$(pkgdatadir)/$(relpwd)
	$(Q)cat .compiler | \
	    sed "s:^:CC=\(:" | sed "s:$$:\):" | \
	    sed "s:$(SRCDIR)/include:$$\{pkgincludedir\}:g" | \
	    sed "s:$(SRCDIR):$$\{pkgdatadir\}:g" | \
	    sed "s:$(KINCL):$$\{KINCL\}:g" | \
	    sed "s:$(KSRC):$$\{KSRC\}:g" | \
	    cat > $(DESTDIR)$(pkgdatadir)/$(relpwd)/.compiler

# 01 Dec 2004 MarkS@Adax - Added proper uninstall target to undo the 
# items done in the above install target
uninstall:
	$(Q)rm -f $(DESTDIR)$(pkgdatadir)/$(relpwd)/.compiler
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(pkgdatadir)/$(relpwd)
ifeq ($(ARCH),s390x)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_SHARED)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_PSHARED)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_STATIC)
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DEST_LIB64DIR)
else
ifeq ($(ARCH),x86_64)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_SHARED)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_PSHARED)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_STATIC)
	$(Q)rm -f $(DEST_LIB64DIR)/$(LIBFILE_PSTATIC)
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DEST_LIB64DIR)
else
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_SHARED)
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_PSHARED)
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_STATIC)
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_PSTATIC)
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DEST_LIBDIR)
endif
endif
ifeq ($(DESTDIR),)	# we have no DESTDIR, ok to run ldconfig
	$(Q)ldconfig
endif


# the following relates to the Q_CC variable, which may be set to .compiler if
# this target's output is desired
#
.compiler:
	$(Q_ECHO) $(qtag__WD_)$(relpwd)
	$(Q_ECHO) $(qtag__CC_)$(CC)
	@echo $(CC) > $@
