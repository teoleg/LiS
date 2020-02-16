include .config_mk

default:
	$(nothing)

install:
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)
	$(Q)install -d $(DESTDIR)$(bindir)
	$(Q)install -d $(DESTDIR)$(sbindir)
ifneq ($(bindir),)
#	do substitutions in lis-config.in to yield lis-config -
#	for now we keep this simple - substitute #{CONFIG} occurrences -
#	these will allow lis-config to include .../config.in, so that
#	its contents can be used thereafter as shell variables
#
	$(Q)cat lis-config.in | \
	    sed "s:#{CONFIG}:$(pkgdatadir)/linux-$$\{KVER\}/config.in:g" | \
	    cat > $(DESTDIR)$(bindir)/lis-config
	$(Q)chmod 0755 $(DESTDIR)$(bindir)/lis-config
endif
ifneq ($(sbindir),)
#
	$(Q)install -m 0555 lis-down lis-status lis-up lis_dbg $(DESTDIR)$(sbindir)/.
#	for backward compatibility, symlink to the above three from their
#	old names
	$(Q)(cd $(DESTDIR)$(sbindir) && \
		$(LN) lis-down   strms_down ; \
		$(LN) lis-up     strms_up; \
		$(LN) lis-status strms_status)
	$(Q)install -m 0555 lis-pids $(DESTDIR)$(sbindir)/.
endif

# 18 Jan 2005 MarkS@Adax
# - Enhanced to uninstall what is installed above instead of doing $(nothing)
uninstall:
#	$(nothing)
ifneq ($(sbindir),)
	-$(Q)(cd $(DESTDIR)$(sbindir) && \
		rm -f lis-pids strms_status strms_up strms_down \
			lis-up lis-status lis-down lis_dbg) 
endif
ifneq ($(bindir),)
	-$(Q)(cd $(DESTDIR)$(bindir) && rm -f lis-config)
endif
	-$(Q)rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(bindir)
	-$(Q)rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(sbindir)

clean:
	-$(nothing)
