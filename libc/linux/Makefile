include .config_mk

PRELDOBJS = getpmsg.o getmsg.o putpmsg.o putmsg.o \
	  fattach.o fdetach.o isastream.o # mknod.o poll.o

#LIBOBJS = $(PRELDOBJS) pipe.o
LIBOBJS = $(PRELDOBJS)

CP_OPTS	= -f
XOPTS  += -D__KERNEL_SYSCALLS__

XOPTS  += -fpic

ifeq ($(GETMSG_PUTMSG_AS_IOCTL),y)
#
# For linux 2.6.11 and later kernels, we can no longer use
# the trick where we overload read/write to handle getmsg/putmsg.
# We have to use ioctl() instead.
XOPTS  += -DLIS_GETMSG_PUTMSG_AS_IOCTL
endif


include $(LIBDIR)/mk.bdy


#
# Dependency stuff
#

-include .depend

dep:
	$(Q_ECHO) $(qtag_DEP)$(relpwd)
	$(Q)$(CC) -M -DDEP $(LIBOBJS:%.o=$(LIBDIR)/%.c) >.depend

