/************************************************************************
*                        Version File                                   *
*************************************************************************
*									*
* This file is to document changes to STREAMS.  It contains the version	*
* number for STREAMS.  When you make a new version, bump the version	*
* number and add comments in here for what is in the new version.	*
*									*
* We declare a char array in this module that others can use to		*
* refer to the version number in ASCII string form.			*
*									*
* Copyright (C) 1997  David Grothe, Gcom, Inc <dave@gcom.com>		*
*									*
************************************************************************/
/*
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 * 
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330, Cambridge,
 * MA 02139, USA.
 * 
 */

char	lis_version[] = "2.18.1.0" ;
char	lis_date[]    = "13-Jun-2013" ;
char    lis_release[] = "9" ;

#if 0

1.2	Changes made as follows:

	Timeouts mapped correctly into local ticks
	Makefile fix for Red Hat 4.0
	getmsg with several blocks did not work correctly
	Needed prototype for flushband
	Needed to make lis_qdetach extern
	Fix problems with I_FIND, I_PEEK
	Code for I_FDINSERT from Eric Levenez
	Fix DEFAULT_RC_SEL in npi.h
	Read did not return when it ran out of queued data
	putmsg/write did not unblock when downstream flow control opened up
	Remove DMA attribute from memory allocation
	Put in stub for allocb_physreq()

1.3	Changes made as follows:

	Put magic number into queue structure so as to verify validity
	Check both read and write queues for sched list at close time
	Print out version number at boot time

1.4	Changes made as follows:
	Add initialization function for STREAMS drivers.  Declared
	in STREAMS config file.
	More modularity for packages.  Each package has its own config
	file.  All configs are concatenated together.

1.5	Changes made as follows:
	STREAMS subsystem made into loadable module.
	Fix bug in select() on STREAMS files not waking up properly.
	
1.6	Changes made as follows:
	lis_freeb had problem if dblk had ref cnt > 1.  It would sometimes
	free the mblk that contained the dblk that other mblks pointed
	to for benefit of the reference count.
	pullupmsg needs to fail if the mblk has use count > 1 since it
	is going to reassign the base/lim pointers in the dblk.

1.8	Add putnextctl and putnextctl1 functions

1.9	Add GNU Library license notice to all .c and .h files

1.10	For esballoc, keep the frtn structure in the dblk so the user does
        not have to allocate those structures.  NON-SVR4 feature.
	Fix race between strread, strputmmsg versus strrput if strrput were
	to get called at interrupt time.

1.11	Lower mux queues need arming at I_LINK time.
	ioctl with INFTIM was timing out.

1.12	Timeout callback functions changed to type void rather than int

1.13	Changed linux sys include files to linux include files to be able
	to compile on RedHat  5.0

1.14	Had to fix a problem in the 1.13 above
1.15	Had to fix a problem in the 1.14 above

1.16
1.17	putpmsg was misusing canput in such a way as to render it
	ineffective.

1.18	stream head write service routine did not perform correctly to
	wake up a blocked writer when canput blockage relieved.

1.19	open routine not calling driver open routine correctly for 2nd open
	when driver open failed, open freed an in-use queue structure
	bring driver MODOPEN, CLONEOPEN bits into conformance with SVR4
	MUST RECOMPILE DRIVERS w/NEW mod.h TO USE THIS VERSION OF LiS

1.20	Bug in poll using select for non-STREAMS files.
	Clear datap in new mblk in case cannot allocate dblk.
	Move stats printing out of stats.c and into streams.c utility.
	Remove syscall definitions from .h files.  Replace with prototypes.
	Introduce libc routines for getpmsg, putpmsg, poll. libLiS.a
	Use S_ZERO_WR flag for 2.0.36.
	Eliminate #ifdef LINUX in some .h files.
	Removed ()s from macros in stream.h.
	Now have a non-null tihdr.h.  Updates to dlpi.h.
	Delete some extraneous includes leading to dependency anomolies.
	Installation and build procedure now outside of kernel tree.

1.21
1.22
1.23	Fix problem with printk not working when installed as module.
	Eliminate SEM_INIT (2.1 kernel problems).
	Some changes for 2.1 compatibility (not ready for 2.1 yet, though).
	Finalize Configure script.

1.24	More changes for 2.1 compatibility.
	Prevent running of queues interrupting putnext operation.
	Ioctl default timeout now implemented (15 secs).

1.25	Candidate final changes for kernel version 2.2.x compatibility.
	Fixed bug in which stream head putpmsg did not send M_PCPROTO
	downstream if flow control blocked; it returned EAGAIN instead.
	Queues run either as bottom half or as kernel thread.

2.0
2.1
2.2	Much better support for loadable modules and kerneld.
	Support for 2.2.x kernels.
	The ldl driver has been used and tested more.
	Implementation of transparent ioctls.
	make sure getpmsg returns correct more bits not obscured by err codes
	ioctl to print queues, streams -q triggers it
	stream head flow ctl vs M_PCPROTO
	M_SETOPTS incorrect for hi/lo water and min/max psize
	run queues either as bottom half or kernel thread process (real time)
	fix problem  with noenable function
	fix a few minor problems with poll
	Installation improvements
	Documentation in html

2.3	Fix a few installation problems with 2.2.
	Get user mode working.
	Change prefix used by osif to "lis_" instead of "osif_".

2.4	Fix a few more installation problems.
	Ole has a start on a SAD driver.
	Updated ldl driver.

2.5	Fix locking for multiple writers to same stream.
	Update html documentation.

2.6	Make ip_strm_mod work properly and for kernel version 2.2.x.
	Improve installation vis a vis SMP.
	Additions to osif.[ch].
	Add raw HDLC interface type to ldl.
	Improve kernel patch for drivers/char/mem.c -- used for
		linking LiS directly into kernel.
	Clone open now allowed to re-use exiting minor dev.

2.7	qprocson/qprocsoff dummies for SVR4 MP compatibility.
	More Linux kernel interface functions in osif.c.
	Compatibility with 2.3.x kernels.
	Compatibility with new glibc stropts.h and poll.h
	Ability to auto-attach IRQs for STREAMS drivers.
	Callout to a driver terminate routine when unloading driver.
	Make module name size compatible with SVR4 (smaller).
	Integrate changes for SPARC version.
	Read of M_PROTO should requeue msg for SVR4 compatibility.
	Portable poll code was not including POLLERR, POLLHUP in events.
	Include POLLOUT with POLLWRNORM if band zero is writeable.
	Implement I_PUNLINK(MUXID_ALL) in SVR4 compatible way.
	Change iocblk structure to conform with SVR4.
	ldl driver: make sure queued messages stay FIFO.
	ldl driver: kernel 2.3.x compatibility.
	ip_strm_mod: better flow control coupling.
	ip_strm_mod: print msg with SIOCSIFNAME succeeds.
	ip_strm_mod: kernel 2.3.x compatibility.
	strtst: tests for bug fixes and new features.
	strconf: "interrupt" and "terminate" declarations.
	libLiS: dummies for fattach, fdetach, isastream.

2.7.1	Fix a bug in read vs M_PCPROTO requeueing mecanism.
	ip_strm_mod: new flow control technique for net drivers.
	ip_strm_mod: ASSERT interfered with IRDA
	More SPARC compatibility.

2.8	A few more tweaks for SPARC.
	Testing for I_LINKed streams in read/write/getmsg/putmsg.
	ldl compiles on 2.3.x kernels (not tested).
	Fix a few warnings.

2.9	Mainly a beta release on the way to 2.10

2.10	fattach/fdetach implemented
	STREAMS pipes and fifos
	SMP compatibility (defensive)
	LiS insulation for drivers (mem, pci, locks, semaphores)
	Updated documentation
	Compatibility with glibc 2.2 (stropts.h)
	Improved installation scripts

2.11	Compatibility with 2.4.0 kernel

2.12	Aggressive SMP in 2.4.0 kernel
	More abstractions to protect drivers from kernel
	Numerous other changes noted in htdocs

2.15	Fix holding spin locks while sleeping (big change)
    	fattach works again
	pipes/fifos work again
	Compatibility with more kernel versions

2.16	Eliminate use of Linux system calls
	Eliminate all kernel patches from distribution
	Some compatibility changes for newer kernels

FILL IN CHANGES FROM 2.18 to 2.18.0.U!!!!

2.18.0.V
	Backported wait_for_completion_interruptible() to LiS for kernels
	before 2.6.11 to fix bug where getmsg() blocked waiting for data
	could not be killed because wait_for_completion which makes the
	thread uninterruptible was used when the zero-initialized semaphores
	were converted to wait_for_completion/complete for use with
	the new real time preemption patches.

2.18.0.X
	Added fix from Martin Hecht at Comverse to fix a race where a message
	could be freed twice in head/msg.c:lis_freedb().  dg@adax.com

2.18.0.Y
	Added support for 2.6.18 kernels (Red Hat EL 5, etc.).

2.18.0.Y2
	Commented out call to might_sleep() in head/wait.c as it is 
	an unresolved symbol on 2.4 kernels and is only used for 
	debugging on 2.6 kernels.
	Repaired bug in cross-compilation facility that prevented 
	using a 64bit cross-compiler running on a 32bit host.
	Restored ability to build LiS on 64bit 2.4 kernels such 
	as those used by Red Hat EL 3 Update 5/6/7/8.

2.18.0.Y3
	Fixes to the Adax LiS Install script in support of 
	cross-platform installation method related to use of 
	LiS on 2.16.18 kernels (utsrelease.h v.s. version.h)
	and generation of the generic LiS symbolic link pointing
	to the current installation of LiS (Remove script) 
	regardless of LiS version.

2.18.0.Y4
	Fix to Adax pstty implementation required to support MontaVista 3.1.  
	The older compiler on this operating system (gcc 3.3.1) was choking
	on the EXPORT_SYMBOL entries in drivers/str/linux/pstty_dev.h.
	Moved the EXPORT_SYMBOL lines to head/linux/exports.c and renamed
	EXPORT_SYMBOL to EXPORT_SYMBOL_NOVERS to match the syntax used in
	exports.c.  Also added extern entries to the prototypes section for 
	the added pstty items.

2.18.1
	Added support for 32 bit user space over a 64 bit kernel for x86_64
	architectures.
	Abstract GFP_XXX flags for lis_kmalloc() to support multiple kernels
	with a single driver/module binary.

2.18.2
	Fixed compilation error on kernels >= 2.6.12.
	Moved 64 bit libraries to be installed in /usr/lib64.
	Updated to remove new libLiS32 libraries.
	Updated to support PPC and Cavium MIPS targets.
	Updated to better support cross compiler installations and
	installations to empty filesystems.
	Support for kernels up to and including 2.6.21.

2.18.3
	Updated to support kernels up to and including 2.6.23
	(Fedora 8 specifically).
	Fixed operation with SELinux.
	Added lis_touch_softlockup_watchdog() interface.

2.18.4
	Fixed panic on unloading streams module on Red Hat EL4 update 7.
	Fixed panic on 2.6.16 and later kernels for drivers which use 
	lis_pci_find_device.

2.18.5
	Updated for SuSE 11.0, kernel 2.6.25.5.
	Updated for SuSE 11.1, kernel 2.6.27.7.
	Updated for SuSE Enterprise 11, kernel 2.6.27.19-5.
	Note that we do not yet support Linux containers.
	Updated to support Linux on MIPS architectures.
	Updated to support Linux /etc/modprobe.d.
	Updated to support target OS selection.
	Updated to support XEN (3.0 tested).
	Fixed 32 over 64 bit libraries to be named libLiS and installed in 
	/usr/lib.

2.19.0
	Rewrote streams scheduler to fix races.

	Previous versions of LiS had some small races where it would be
	possible for too many queue runner threads to be started or else for
	queue runner threads to not be started even if there were service
	routines queued to be serviced.

	Added new scheduling policies for Queue Runner threads:

	RRP: Round Robin Processor.  This mode will start queue runner threads
	starting from the last processor used run a queue runner thread.
	This ensures that the CPU utilization is spread evenly over all the
	processors in the system, but uses more CPU utilization due to more
	context switches.

	LPF: Last Processor First.  This always starts queue runner threads on
	processors starting from the last processor in the system.  This mode
	tends to have better CPU utilzation over all due to lower context
	switches, but can suffer latency issues if the CPUs become saturated
	at 90-100% utilization.

	Changed the default work threshold to 16.  This is the number
	of service routines that must be queued to start additional queue
	runner threads.  Each stream typically have 2-4 service routines for
	the stream head and lowest layer driver, plus 1-2 per protocol module
	pushed on the stream.

	Added support to 'streams' utility to modify the scheduling policy
	on the fly and print the current value.

	Fixed stack overflow in lis_start_qsched() with long version strings.

	Fixed crash in ip_strm_mod on SuSE 11.1.

	Removed pipe() from libLiS.so and libLiS.a.  pipe() has an unresolved
	bug in close which results in a deadly embrace and hang.  Use the
	native pipe() implementation instead.

2.19.1
	Added touch_softlockup_watchdog() to queuerun to prevent it from
	complaining if a CPU gets caught running service routines constantly
	for 10 seconds.

	Added support of WindRiver PNE Linux 2.0 SDK common_pc and common_pc_64 
	platforms.
	
	Fixed a bug in lis_pci_find_device to ensure there is a 1 to 1 mapping
	between lis_pci_dev_t and struct pci_dev structures.

2.19.2
	Ported to 2.6.32 kernels and earlier. (SLES11 SP1).

2.19.3
	Added support for lis_pci_register_driver to support STREAMS drivers
	which support hot plug and hot swap.
	Added support for 2.6.34 kernels and earlier (OpenSuse 11.3, RHEL 6).

2.19.4
	Security fix: stop setting init process`s umask to 0 when LiS
	is loaded.

	Fixed bug to ensure that LiS queue runner thread name is NULL
	terminated.

	Updated lis_driver_probe to create a kobject for LiS and send
	an UEVENT _after_ the driver is probed and after the driver is
	removed in lis_driver_remove().  These kobjects are visible in the
	/sys/LiS-hotplug directory.  Drivers can create UDEV rules for these
	UEVENTS to create device nodes or other user space processing.	The
	uevent has the DEVPATH set to `/LiS-hotplug/lis-<drivername>-<inst>`
	The driver should return the instance number as the return value
	from it`s probe() routine or -errno.  

	This is only supported for LiS 2.6.16 and later.  Removed 
	lis_pci_register_driver and associated functions for 2.6.15 and 
	older kernels.
=======

/* ### For changes made at Comverse see the spec files or the RCS logs. */

#endif
