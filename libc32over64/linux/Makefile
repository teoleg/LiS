include .config_mk

PRELDOBJS = getpmsg.o getmsg.o putpmsg.o putmsg.o \
	  fattach.o fdetach.o isastream.o # mknod.o poll.o

#LIBOBJS = $(PRELDOBJS) pipe.o
LIBOBJS = $(PRELDOBJS) 

ifeq ($(ARCH),s390)
CP_OPTS	= -f
XOPTS  += -D__KERNEL_SYSCALLS__
XOPTS  += -DBLD32OVER64 -m31
LDOPTS = -melf_s390
endif

ifeq ($(ARCH),s390x)
CP_OPTS	= -f
XOPTS  += -D__KERNEL_SYSCALLS__
XOPTS  += -DBLD32OVER64 -m31
LDOPTS = -melf_s390
endif

ifeq ($(ARCH),x86_64)
ifeq ($(CC_NAME32),)
XOPTS  += -m32
else
CC=$(CC_NAME32)
endif
LDOPTS += -melf_i386
ifneq ($(GETMSG_PUTMSG_AS_IOCTL),y)
XOPTS  += -DBLD32OVER64
endif
endif

ifeq ($(ARCH),mips)
XOPTS  += -mabi=n32
LDOPTS += -melf32btsmipn32
ifneq ($(GETMSG_PUTMSG_AS_IOCTL),y)
XOPTS  += -DBLD32OVER64
endif
endif

ifeq ($(GETMSG_PUTMSG_AS_IOCTL),y)
#
# For linux 2.6.11 and later kernels, we can no longer use
# the trick where we overload read/write to handle getmsg/putmsg.
# We have to use ioctl() instead.
XOPTS  += -DLIS_GETMSG_PUTMSG_AS_IOCTL
endif

XOPTS  += -fpic

include $(LIBDIR32OVER64)/mk.bdy


#
# Dependency stuff
#

-include .depend

dep:
	$(Q_ECHO) $(qtag_DEP)$(relpwd)
	$(CC) -M -DDEP $(LIBOBJS:%.o=$(LIBDIR)/%.c) >.depend
