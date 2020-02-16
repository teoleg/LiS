INSTALL_ROOT    = $(O_H)
SC_BUILD        = $(INSTALL_ROOT)
SC_INSTALL_BIN  = $(D_H)/bin
SC_INSTALL_DIR  = $(D_H)
SC_PERL         = perl
SC_PERL_LIB     = omni
SC_PERL_SITE    = site
SC_PERLLIB_LOCAL= $(SC_INSTALL_DIR)/$(SC_PERL)/$(SC_PERL_LIB)
SC_PERLLIB_SITE = $(SC_INSTALL_DIR)/$(SC_PERL)/$(SC_PERL_SITE)
SC_MIBS         = $(SC_INSTALL_DIR)/MIBS


ccflags-y :=-I/usr/src/kernels/`uname -r`/include
ccflags-y +=-I/usr/src/kernels/`uname -r`/arch/x86/include 
ccflags-y +=-I/usr/src/kernels/`uname -r`/include/linux   
ccflags-y +=-imacros ""/usr/src/kernels/`uname -r`/include/generated/autoconf.h"" 
ccflags-y +=-I/usr/include/asm-generic
ccflags-y +=-I/root/LiS/2.18.1/source
ccflags-y +=-I/root/LiS/2.18.1/source/include
ccflags-y +=-DMODVERSIONS
#ccflags-y +=-D__GENKSYMS__
ccflags-y +=-Wno-strict-prototypes
ccflags-y +=-Wno-unused-value
ccflags-y +=-Wno-unused-variable
ccflags-y +=-Wno-declaration-after-statement
ccflags-y +=-DLINUX


#KBUILD_EXTRA_SYMBOLS := $(O_H)/open_ss7/modules/Module.symvers
obj-m  := streamshead.o
streamshead-y := ../buffcall.o ../cmn_err.o ../dki.o ../events.o ../head.o ../linux-mdep.o ../lisnet.o ../mdep.o ../mod.o ../msg.o ../msgutl.o ../osif.o ../poll.o ../queue.o ../safe.o ../stats.o ../stream.o ../strmdbg.o ../version.o ../wait.o exports.o lislocks.o lismem.o lispci.o

KDIR ?= /lib/modules/`uname -r`/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD)

clean: 
	$(MAKE) -C $(KDIR) M=$(PWD) clean

