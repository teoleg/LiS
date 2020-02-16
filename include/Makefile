include .config_mk

default:
	$(nothing)

install:
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)
	$(Q)find . -type d $(Q_PRINT) \
		-exec install -d $(DESTDIR)$(pkgincludedir)/{} \
			>/dev/null \;
	$(Q)find . -type f -name "*.h" $(Q_PRINT) \
		-exec install -m 0644 {} $(DESTDIR)$(pkgincludedir)/{} \
			>/dev/null \;
ifneq ($(pkgincludedir),$(pkgdatatdir)/include)
#	add an 'include' symlink to pkgincludedir
# 18 Jan 2005 MarkS@Adax - It's important that the symlink use relative 
#	pathing to support cross platform development where we are installing 
#	to an NFS hosted/virtual rooted filesystem.
	$(Q)(cd $(DESTDIR)$(pkgdatadir) && $(LN) ../../..$(pkgincludedir) include)
endif
ifneq ($(pkglibdir),$(pkgdatatdir))
#	add an 'include' symlink to pkgincludedir
# 18 Jan 2005 MarkS@Adax - older LiS installations (2.17.x and older), create
#	a physical (non-symlinked) include directory within the pkglibdir
#	which is not removed when a user runs "make uninstall" to remove 
#	old LiS prior to upgrading to this LiS.  This left behind physical
#	include directory interferes with the creation of the below symlink.
#	Delete the old include directory and hope the user did not store
#	any generated files in that location.  Note: it is important that 
#	the symlink use relative pathing to support cross platform 
#	development where we are installing to an NFS hosted/virtual 
#	rooted filesystem.
	$(Q)[ -d $(DESTDIR)$(pkglibdir)/include ] && \
		(rm -r $(DESTDIR)$(pkglibdir)/include) || :
	$(Q)(cd $(DESTDIR)$(pkglibdir) && $(LN) ../../..$(pkgincludedir) include)
endif

# 01 Dec 2004 MarkS@Adax - Added proper uninstall target to undo the 
# items done in the above install target
uninstall:
	$(Q)find . -type f -name "*.h" $(Q_PRINT) \
		-exec rm -f $(DESTDIR)$(pkgincludedir)/{} \
			>/dev/null \;
	$(Q)find * -type d $(Q_PRINT) \
		-exec rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(pkgincludedir)/{} \
			>/dev/null \;
ifneq ($(pkgincludedir),$(pkgdatatdir)/include)
#	remove the 'include' symlink from pkgincludedir
	$(Q)(cd $(DESTDIR)$(pkgdatadir) && rm -r include)
endif
ifneq ($(pkglibdir),$(pkgdatatdir))
#	remove the 'include' symlink from pkglibdir
	$(Q)(cd $(DESTDIR)$(pkglibdir) && rm -r include)
endif

clean:
	$(nothing)
