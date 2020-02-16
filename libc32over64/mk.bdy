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

LIB_SHARED = $(LIBDIR32OVER64)/$(LIBFILE_SHARED)
LIB_PSHARED = $(LIBDIR32OVER64)/$(LIBFILE_PSHARED)
LIB_STATIC = $(LIBDIR32OVER64)/$(LIBFILE_STATIC)

ifeq ($(V), 1)
CC += -fverbose-asm -save-temps
endif

CC += -c $(XOPTS)


all: compiler $(LIB_SHARED) $(LIB_PSHARED) $(LIB_STATIC)

compiler:
	$(Q_ECHO) $(qtag__WD_)$(relpwd)
	$(Q_ECHO) $(qtag__CC_)$(CC)

$(LIB_SHARED): $(LIBOBJS)
	$(Q_ECHO) $(qtag_LD)$(reltarget)
	$(Q)$(LD) $(LDOPTS) -shared -o $@ $^

$(LIB_PSHARED): $(PRELDOBJS)
	$(Q_ECHO) $(qtag_LD)$(reltarget)
	$(Q)$(LD) $(LDOPTS) -shared -o $@ $^ 

$(LIB_STATIC): $(LIBOBJS)
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
	-$(Q)rm -f *.o
	-$(Q)rm -f $(LIB)
	-$(Q)rm -f *.s *.i

realclean: clean
	-$(Q)rm -f .depend
	-$(Q)rm -f $(LIB_SHARED)
	-$(Q)rm -f $(LIB_PSHARED)
	-$(Q)rm -f $(LIB_STATIC)


install: all
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)
	$(Q)install -d $(DEST_LIBDIR)
	$(Q)install -m 0755 $(LIB_SHARED) $(DEST_LIBDIR)/$(LIBFILE_SHARED)
	$(Q)install -m 0755 $(LIB_PSHARED) $(DEST_LIBDIR)/$(LIBFILE_PSHARED)
	$(Q)install -m 0644 $(LIB_STATIC) $(DEST_LIBDIR)/$(LIBFILE_STATIC)
ifeq ($(DESTDIR),)	# we have no DESTDIR, ok to run ldconfig
	$(Q)ldconfig
endif

uninstall:
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_SHARED)
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_PSHARED)
	$(Q)rm -f $(DEST_LIBDIR)/$(LIBFILE_STATIC)
	$(Q)rmdir -p --ignore-fail-on-non-empty $(DEST_LIBDIR)

