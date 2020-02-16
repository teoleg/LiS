include .config_mk

default:
	$(nothing)

install:
	$(Q_ECHO) $(qtag_INSTALL)$(relpwd)
	$(Q)install -d $(DESTDIR)$(mandir)/man3
	$(Q)install -d $(DESTDIR)$(mandir)/man8
	$(Q)install -d $(DESTDIR)$(mandir)/man9
	$(Q)install -m 0644 man3/* $(DESTDIR)$(mandir)/man3/.
	$(Q)install -m 0644 man8/* $(DESTDIR)$(mandir)/man8/.
	$(Q)install -m 0644 man9/* $(DESTDIR)$(mandir)/man9/.

# 01 Dec 2004 MarkS@Adax - Added proper uninstall target to undo the 
# items done in the above install target
uninstall:
	$(Q)find man[389] -type f $(Q_PRINT) \
		-exec rm -f $(DESTDIR)$(mandir)/{} > /dev/null \;
	$(Q)find man[389] -type d $(Q_PRINT) \
		-exec rmdir -p --ignore-fail-on-non-empty $(DESTDIR)$(mandir)/{} \
			> /dev/null \;
