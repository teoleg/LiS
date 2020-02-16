include ../version
include ../config.in
include targets

build-rpm: 
	./mk_rpm

test-rpm-install:
	rpm -i -vv RPMS/i386/LiS-$(LISVER)-$(RELEASE).i386.rpm

test-rpm-uninstall:
	rpm -e -vv LiS-$(LISVER)

clean:
	-@rm -rf BUILD SOURCES tmp

realclean:
	-@rm -rf RPMS
