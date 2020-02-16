/*                               -*- Mode: C -*- 
 * linux-mdep.c --- Linux kernel dependent support for LiS.
 * Author          : Francisco J. Ballesteros
 * Created On      : Sat Jun  4 20:56:03 1994
 * Last Modified By: John A. Boyd Jr.
 * RCS Id          : $Id: linux-mdep.c,v 9.14 2014/02/25 17:37:12 heenan Exp $
 * Purpose         : provide Linux kernel <-> LiS entry points.
 * ----------------______________________________________________
 *
 *    Copyright (C) 1995  Francisco J. Ballesteros, Denis Froschauer
 *    Copyright (C) 1997-2000
 *			  David Grothe, Gcom, Inc <dave@gcom.com>
 *    Copyright (C) 2000  John A. Boyd Jr.  protologos, LLC
 *
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
 *
 *    You can reach me by email to
 *    nemo@ordago.uc3m.es, 100741.1151@compuserve.com
 *    dave@gcom.com
 *
 * Modified 2001-08-09 by Jeff Goldszer <Jeff_goldszer@cnt.com>
 *    Prevent fdetach from freeing a pipe's inode before the pipe
 *    has been closed.
 *
 * 2002/11/18 - John Boyd - reworked fattach()/fdetach() code to use
 *    mount()/umount() infrastructure in 2.4.x kernels; old code
 *    removed (per Dave's request...).  Filesystem/superblock code
 *    also reworked, for same purpose.
 */

#ident "@(#) LiS linux-mdep.c 2.160 10/13/04 11:07:36 "

/*  -------------------------------------------------------------------  */
/*				 Dependencies                            */

/***********************************************/
/* LiS Diagnostics and Proc Filesystem Support */
/***********************************************/

/* Note that the define for LIS_HAS_PROC_ENTRIES must be defined in the C files */
/* that support the Proc File System usage.  The define must be before the      */
/* include files section so that it is defined for the linux-mdep.h file.       */

#ifndef LIS_HAS_PROC_ENTRIES
#define LIS_HAS_PROC_ENTRIES
#endif

/***********************************/           
/* LiS implementation modules used */
/***********************************/           

#include <sys/LiS/linux-mdep.h>
#include <sys/lislocks.h>
#include <linux/module.h>
#include <sys/LiS/modcnt.h>		/* after linux-mdep.h & module.h */
#include <linux/version.h>

#include <linux/fs_struct.h>
#include <linux/sched.h>
#define	__KERNEL_SYSCALLS__	1	/* to make kernel_thread visible */
#include <linux/unistd.h>		/* for kernel_thread */

# ifdef RH_71_KLUDGE			/* boogered up incls in 2.4.2 */
#  undef CONFIG_HIGHMEM			/* b_page has semi-circular reference */
# endif
#include <asm/signal.h>
#include <asm/io.h>
#include <sys/strport.h>	/* interface */
#include <sys/LiS/errmsg.h>	/* LiS err msg types */
#include <sys/LiS/buffcall.h>	/* bufcalls */
#include <sys/LiS/head.h>	/* stream head */
#include <sys/osif.h>
#include <sys/stream.h>		/* LiS entry points */
#include <sys/cmn_err.h>
#include <sys/poll.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/kconfig.h>
#include <linux/fs.h>		/* linux file sys externs */
#include <linux/vfs.h>		/* linux file sys externs */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#include <linux/fdtable.h>
#endif
#if (defined _X86_64_LIS_ || defined _MIPS64_LIS_) && (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12) 
/* 2.6.12 or later */
#include <linux/ioctl32.h>	/* linux 32/64bit compatible ioctls */
#else
#include <asm/ioctl32.h>	/* linux 32/64bit compatible ioctls */
#endif
#endif
/* MarkS@Adax - 21 Dec 2004 - Added USE_NAMEI_H to allow include 
   of linux/namei.h required for LiS builds on MontaVista CGE 3.1 
   kernel.  See the modification to the shell script named mkgenconf 
   in the LiS top level directory for the related change. */
#if ( defined(KERNEL_2_5) || defined(USE_NAMEI_H) )
#include <linux/namei.h>	/* linux file sys externs */
#endif
#if defined(KERNEL_2_5)
#include <linux/cdev.h>		/* cdev_put */
#endif
#include <linux/pipe_fs_i.h>
#include <linux/mount.h>
#if defined(FATTACH_VIA_MOUNT)
#include <linux/capability.h>
#endif
#include <linux/time.h>

#if defined(KERNEL_2_5)
#include <linux/kobject.h>
#endif

/***********************************************/
/* LiS Diagnostics and Proc Filesystem Support */
/***********************************************/

#ifdef LIS_HAS_PROC_ENTRIES

#include <linux/proc_fs.h>

/* Event and Error Log */
lis_diag_t  lis_diag_log;

/* System Call Trace Log */
lis_sc_trace_t lis_diag_sc_trace_log;

static struct proc_dir_entry *lis_dir = 0;
#define LIS_PROC_DIR "LiS"

/* Note that these are somewhat critical so that the formatting */
/* of the log doesn't overrun the length of mom_output_buffer.  */

/* The following provides room for 1 line for each trace entry  */
/* plus 10 extra lines for headings, etc.                       */

#define LIS_DIAG_MAX_OUTPUT_LINE_SIZE 300

#define LIS_OUTPUT_BUFFER_SIZE (LIS_DIAG_MAX_OUTPUT_LINE_SIZE * (LIS_DIAG_TRACE_ENTRIES + 10))

char lis_output_buffer[LIS_OUTPUT_BUFFER_SIZE];
char lis_out_buf[2048];
int  lis_count_in_buffer;

unsigned long lis_debug_err[NUMBER_OF_DEBUG_COUNTERS];

/*************************/
/* Function Declarations */
/*************************/

int lis_proc_init(void);
int lis_proc_exit(void);
int lis_procfile_read(char *buffer, char **buffer_location, off_t offset, int count, int *eof, void *data);
int lis_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data);

#endif /* LIS_HAS_PROC_ENTRIES */

/***********************/
/* LiS Initalized Flag */
/***********************/

int lis_initialized = 0;

/**********************************/
/* LiS Tunable Parameters Support */
/**********************************/

#ifndef LIS_TUNABLE_PARAMETERS
#define LIS_TUNABLE_PARAMETERS
#endif

#ifdef LIS_TUNABLE_PARAMETERS

/* The following are tunable via modules.conf (Linux 2.4) or modprobe.conf (2.6)     */
/* Defaults for the  maximum queue depth for queue runners and the scheduling policy */

#define LIS_QUEUE_RUNNER_DEPTH 16
#define MAX_QUEUE_RUNNER_DEPTH 512

int queue_runner_depth = LIS_QUEUE_RUNNER_DEPTH;
int scheduling_policy = LIS_SCHEDULER_RRP;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)

/* The following marco hides new definition of MODULE_PARM for integer variables */
#define MODULE_PARM(name,type) module_param(name, int, 0644);

#endif

MODULE_PARM(queue_runner_depth, "i");
MODULE_PARM_DESC(queue_runner_depth, "Max queue runner depth");
MODULE_PARM(scheduling_policy, "i");
MODULE_PARM_DESC(scheduling_policy, "Scheduling policy");

#endif /* LIS_TUNABLE_PARAMETERS */

/*  -------------------------------------------------------------------  */
/*
 * S/390 2.4 kernels export smp_num_cpus
 * other 2.4 kernels export num_online_cpus()
 */
#if ( defined(__s390__) || defined(__s390x__) ) && defined(KERNEL_2_4)
#define NUM_CPUS		smp_num_cpus
#elif defined(KERNEL_2_5)
#define NUM_CPUS		num_online_cpus()
#else
#define NUM_CPUS		smp_num_cpus
#endif

/*  -------------------------------------------------------------------  */

int lis_major;
char lis_kernel_version[]= UTS_RELEASE;

extern char	lis_version[] ;
extern char	lis_date[] ;
extern char     lis_release[] ;

extern void	lis_start_qsched(void);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
struct kset *lis_kset;
extern struct kset *lis_kset_create_and_add(const char *name,
				     const struct kset_uevent_ops *uevent_ops,
				     struct kobject *parent_kobj);
#endif

char	*lis_stropts_file =
#if   defined(_LIS_STROPTS_H)
                "<LiS/include/stropts.h>"
#elif defined(_LIS_SYS_STROPTS_H)
		"<LiS/include/sys/stropts.h>"
#elif defined(_STROPTS_H)
                "<usr/include/stropts.h>"
#elif defined(_SYS_STROPTS_H)
                "<usr/include/sys/stropts.h>"
#elif defined(_BITS_STROPTS_H)
		"<usr/include/bits/stropts.h>"
#else
		"<unknown/stropts.h>"
#endif
;

/************************************************************************
*                      System Call Support                              *
*************************************************************************
*									*
* LiS provides wrappers for a selected few system calls.  The functions	*
* return 0 upon success and a negative errno on failure.		*
*									*
************************************************************************/

static int	lis_errnos[LIS_NR_CPUS] ;
#define	errno	lis_errnos[smp_processor_id()]

#define __NR_syscall_mknod	__NR_mknod
#define __NR_syscall_unlink	__NR_unlink
#define __NR_syscall_mount	__NR_mount
#if defined(_ASM_IA64_UNISTD_H)
#define __NR_syscall_umount2	__NR_umount
#else
#define __NR_syscall_umount2	__NR_umount2
#endif

/*
 * For gcc 3.3.3 the combination of inlining these functions and the
 * register passing conventions causes the parameters to the system
 * call to get messed up.  Simply defeating the inlining takes care
 * of the problem.  You won't see the problem unless you are working
 * with a 2.6 based distribution.  I first noticed it in SuSE 9.1.
 */
#if defined(noinline)		/* kernel has this defined */
#define _NI     noinline
#else				/* no special meaning */
#define	_NI	__attribute__((noinline))
#endif

#if !defined _X86_64_LIS_ && !defined _MIPS64_LIS_ && \
    LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,19)      /* 2.6.19 or earlier */
static _NI _syscall3(long,syscall_mknod,const char *,file,int,mode,int,dev)
static _NI _syscall1(long,syscall_unlink,const char *,file)
static _NI _syscall5(long,syscall_mount,char *,dev,char *,dir,
			char *,type,unsigned long,flg,void *,data)
static _NI _syscall2(long,syscall_umount2,char *,file,int,flags)
#else

/* Doing this rude trick of calling syscalls from inside the kernel
 * has had the official kibbash put on it.  It just crashes the machine
 * when you try it on x86_64.  
 *
 * We'll have to find a new way to do these things for
 * the utilities that use them.  For now, just return error for each
 * of them and hope that those who call it handle the error case correctly.
 * dg@adax.com Feb 11, 2005
 */
#define syscall_mknod(a,b,c) -EINVAL
#define syscall_unlink(a) -EINVAL
#define syscall_mount(a,b,c,d,e) -EINVAL
#define syscall_umount2(a,b) -EINVAL
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)      /* 2.6.16 or later */
/* PIPE_SEM was converted to use mutexes in 1b1dcc1b (v2.6.16-rc1) */
#define PIPE_SEM PIPE_MUTEX
#endif


/************************************************************************
*                            lis_assert_fail                            *
*************************************************************************
*									*
* This is called when the ASSERT() macro fails.                         *
*									*
* It sends a warning message to the kernel logger, but does nothing     *
* else.                                                        		*
*									*
************************************************************************/
void _RP lis_assert_fail(const char *expr, const char *objname,
		         const char *file, unsigned int line)
{
        printk(KERN_CRIT "%s: assert(%s) failed in file %s, line %u\n",
	       objname, expr, file, line);
        /* We cannot just abort() the kernel here :-( */
}

/************************************************************************
*                            Prototypes                                 *
************************************************************************/
extern void do_gettimeofday(struct timeval *tv) _RP;	/* kernel fcn */
extern void lis_spl_init(void);			/* lislocks.c */
extern int  lis_new_file_name_dev(struct file *f, const char *name, dev_t dev);
static struct inode * lis_get_inode( mode_t mode, dev_t dev );
void lis_print_dentry(struct dentry *d, char *comment) ;



/************************************************************************
*                       Storage Declarations                            *
************************************************************************/

lis_spin_lock_t			lis_setqsched_lock ; /* one qsched at a time */
struct task_struct *lis_runq_tasks[LIS_NR_CPUS];
struct completion lis_runq_kill_sems[LIS_NR_CPUS];
extern volatile unsigned long	lis_runq_wakeups[LIS_NR_CPUS] ; /* head.c */
int				lis_runq_sched ;     /* q's are scheduled */
lis_atomic_t			lis_inode_cnt ;
lis_atomic_t                    lis_mnt_cnt;   /* for lis_mnt only, for now */
int                             lis_mnt_init_cnt;  /* initial/final value */
lis_spin_lock_t			lis_task_lock ; /* for creds operations */
int				lis_work_incr;
int				lis_scheduler_policy;
/*
 * The following for counting kmem_cache allocations
 */
lis_atomic_t                    lis_locks_cnt;
lis_atomic_t                    lis_head_cnt;
lis_atomic_t                    lis_qband_cnt;
lis_atomic_t                    lis_queue_cnt;
lis_atomic_t                    lis_qsync_cnt;
lis_atomic_t                    lis_msgb_cnt;

typedef struct lis_free_passfp_tg
{
	lis_spin_lock_t	lock ;	/* syncs global operations */
	mblk_t *head;	
	mblk_t *tail;	
} lis_free_passfp_t;

static lis_free_passfp_t free_passfp;

#if defined(USE_KMEM_CACHE)
lis_kmem_cache_t *lis_msgb_cachep = NULL;
lis_kmem_cache_t *lis_queue_cachep = NULL;
lis_kmem_cache_t *lis_qsync_cachep = NULL;
lis_kmem_cache_t *lis_qband_cachep = NULL;
lis_kmem_cache_t *lis_head_cachep = NULL;
#endif

#if defined(USE_KMEM_TIMER) 
lis_kmem_cache_t *lis_timer_cachep = NULL;
struct lis_timer {
      struct timer_list    lt;
      timo_fcn_t 	  *func;
      caddr_t		   arg;
      volatile toid_t	   handle;
};
#endif

#if defined(FATTACH_VIA_MOUNT)
/*
 * fattach instance data
 *
 *  note that fattach instances must be unique by sb (and thus by mounted
 *  dentry), but NOT by path or mounted STREAM (e.g., inode or head),
 *  since a path can be fattach'ed onto more than once, and a STREAM
 *  can be fattached onto more than one path (and thus onto the same
 *  path more than once).
 */
typedef struct lis_fattach {
    struct file        *file;             /* file passed in (->head) */
    struct vfsmount    *mount;            /* file's f_vfsmnt */
    stdata_t           *head;             /* stream head (->inode) */
    const char         *path;             /* path mounted on */
    struct super_block *sb;               /* mounted LiS sb (->dentry) */
    struct dentry      *dentry;           /* mounted dentry */
    struct list_head    list;
} lis_fattach_t;

/*
 *  global list of fattach instances
 */
static LIST_HEAD(lis_fattaches);
lis_atomic_t num_fattaches_allocd = 0;
lis_atomic_t num_fattaches_listed = 0;
lis_spin_lock_t lis_fattaches_lock;

static lis_fattach_t *lis_fattach_new(struct file *file, const char *path);
static void lis_fattach_delete(lis_fattach_t *data);
static void lis_fattach_insert(lis_fattach_t *data);
static void lis_fattach_remove(lis_fattach_t *data);

#endif  /* FATTACH_VIA_MOUNT */

void lis_show_inode_aliases(struct inode *);

/*
 * Declared in head.c
 */
extern lis_atomic_t		lis_runq_cnt ;	/* # of threads running */
extern int			lis_num_cpus ;
extern int			lis_runq_pids[LIS_NR_CPUS] ;
extern lis_atomic_t		lis_runq_active_flags[LIS_NR_CPUS] ;
extern volatile unsigned long	lis_setqsched_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long	lis_setqsched_isr_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long	lis_setqsched_usr_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long	lis_setqsched_defer_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long	lis_setqsched_poke_cnts[LIS_NR_CPUS] ;
extern lis_atomic_t		lis_queues_running ;
extern lis_atomic_t		lis_putnext_flag ;
extern lis_atomic_t	 	lis_runq_req_cnt ;
extern lis_spin_lock_t	 	lis_qhead_lock ;

#if defined(USE_KMEM_CACHE)
extern void lis_cache_destroy(lis_kmem_cache_t *p, lis_atomic_t *c, char *label);
#endif

/*  -------------------------------------------------------------------  */

/* This should be entry points from the kernel into LiS
 * kernel should be fixed to call them when appropriate.
 */
#if defined(KERNEL_2_5)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
/* flushed changed in 75e1fcc0b18df0a65ab113198e9dc0e98999a08c v.2.6.18-rc1 */
int lis_strflush( struct file *f, fl_owner_t id );
#else
int lis_strflush(struct file *f);
#endif
#endif

/*
 * File operations
 */
struct file_operations
lis_streams_fops = {
    owner:     THIS_MODULE,
    read:      lis_strread,		/* read    		*/
    write:     lis_strwrite,		/* write                */
    poll:      lis_poll_2_1,		/* poll  		*/
#ifdef HAVE_UNLOCKED_IOCTL
    unlocked_ioctl:     lis_strioctl_unlocked,	/* ioctl w/o BKL */
#else
    ioctl:     lis_strioctl_wrapper,		/* ioctl w/ BKL 	*/
#endif
#ifdef HAVE_COMPAT_IOCTL
    compat_ioctl: lis_strioctl_compat,
#endif
    open:      lis_stropen,		/* open                 */
#if defined(KERNEL_2_5)
    flush:     lis_strflush,		/* flush		*/
#endif
    release:   lis_strclose,		/* release 		*/
};

/*
 * Dentry operations
 */
extern int lis_dentry_delete(struct dentry *dentry) ;
extern void lis_dentry_iput(struct dentry *dentry, struct inode *inode);

struct dentry_operations lis_dentry_ops =
{
    d_delete:   lis_dentry_delete,
    d_iput:     lis_dentry_iput
};

/*
 *  D_IS_LIS() is a predicate macro that identifies a dentry as
 *  LiS-specific, i.e., allocated by LiS for LiS use only.
 */
#define D_IS_LIS(d)    ( (d) ? ((d)->d_op == &lis_dentry_ops) : 0 )

/*
 * Inode operations
 */
# if defined(KERNEL_2_5)
  struct dentry *lis_inode_lookup(struct inode *dir, struct dentry *dentry,
	  			  struct nameidata *nd);
# else
  struct dentry *lis_inode_lookup(struct inode *dir, struct dentry *dentry);
# endif

struct inode_operations
lis_streams_iops = {
    lookup:		&lis_inode_lookup,
};

/*
 * LiS inode structure.
 *
 * We use the generic_ip to point back to the stream head.  We also need
 * a place for the LiS dev_t.  In pre-2.6 kernels the i_rdev field is a
 * "short" (16 bits).  We need 32 bits.  So for pre-2.6 kernels we define
 * a structure that we overlay at the "u" field in the inode structure.
 *
 * The 2.6 kernel got rid of the big union, leaving just the generic_ip
 * pointer -- so there is no room for a structure overlay.  This is OK
 * because the 2.6 i_rdev is a 32-bit word, so we can use it directly.
 *
 * This all leads to some messiness of if-def-ing.
 */
#if !defined(KERNEL_2_5)
typedef struct
{
    stdata_t	*hdp ;		/* to stream head structure */
    dev_t	 dev ;		/* LiS device */

} lis_inode_info_t ;

#endif

/*
 * File system operations
 */
//#if defined(KERNEL_2_5)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
int lis_fs_get_sb(struct file_system_type *fs_type,
				  int flags,
				  const char *dev_name,
				  void *ptr, struct vfsmount *mnt);
#else
struct super_block *lis_fs_get_sb(struct file_system_type *fs_type,
				  int flags,
				  const char *dev_name,
				  void *ptr) ;
#endif
void lis_fs_kill_sb(struct super_block *);
//#else
//struct super_block *lis_fs_read_super(struct super_block *sb,
//				      void *ptr,
//				      int silent) ;
//#endif

#define LIS_FS_NAME	"LiS"

struct file_system_type
lis_file_system_ops =
{
    name:	LIS_FS_NAME,
//#if defined(KERNEL_2_5)
//    get_sb:	lis_fs_get_sb,
    kill_sb:	lis_fs_kill_sb,
    owner:	THIS_MODULE,
//#else
    //read_super:	lis_fs_read_super,
    //owner:	NULL,
//#endif
#if defined(KERNEL_2_4_7)
    fs_flags:   0,
#else
    fs_flags:	(FS_NOMOUNT | FS_SINGLE),
#endif
} ;
#define	LIS_SB_MAGIC	( ('L' << 16) | ('i' << 8) | 'S' )

struct vfsmount		*lis_mnt ;
int			 lis_initial_use_cnt ;


void lis_mnt_cnt_sync_fcn(const char *file, int line, const char *fn)
{
    lis_mnt_cnt_sync();

    if (LIS_DEBUG_REFCNTS)
	printk("lis_mnt_cnt_sync() >> [%d] {%s@%d,%s()}\n",
	       K_ATOMIC_READ(&lis_mnt_cnt), file, line, fn);
}

struct vfsmount *lis_mntget_fcn(struct vfsmount *m,
				const char *file, int line, const char *fn)
{
    struct vfsmount *mm = (m ? mntget(m) : NULL) ;

    lis_mnt_cnt_sync();

    if (LIS_DEBUG_REFCNTS)
    {
	if (mm == NULL)
	    printk("lis_mntget(NULL) {%s@%d,%s()}\n", file,line,fn) ;
	else
	    printk("lis_mntget(m@0x%p/++%d) \"%s {%s@%d,%s()}\n",
		   mm, MNT_COUNT(mm), 
		   (lis_mnt && mm == lis_mnt?" <lis_mnt>":""),
		   file,line,fn) ;
    }

    return(mm) ;
}

void lis_mntput_fcn(struct vfsmount *m,
		    const char *file, int line, const char *fn)
{
    if (m == NULL)
    {
	if (LIS_DEBUG_REFCNTS)
	    printk("lis_mntput(NULL) {%s@%d,%s()}\n", file,line,fn) ;
	return ;
    }

    if (LIS_DEBUG_REFCNTS)
	printk("lis_mntput(m@0x%p/%d%s) \"%s {%s@%d,%s()}\n",
	       m,
	       MNT_COUNT(m), (MNT_COUNT(m)>0?"--":""),
	       (lis_mnt && m == lis_mnt?" <lis_mnt>":""),
	       file,line,fn) ;

    if (MNT_COUNT(m) > 0)
	mntput(m) ;

    lis_mnt_cnt_sync_fcn(file, line, fn) ;
}

/*
 * Super block operations
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
/* Changed in 726c334223180e3c0197cc980a432681370d4baf v.2.6.18-rc1 */
int lis_super_statfs(struct dentry *dentry, struct kstatfs *stat);
#elif defined(KERNEL_2_5)
int lis_super_statfs(struct super_block *sb, struct kstatfs *stat);
#else
int lis_super_statfs(struct super_block *sb, struct statfs *stat);
#endif

void lis_super_put_inode(struct inode *) ;

#if defined(KERNEL_2_5)
void lis_drop_inode(struct inode *) ;
#endif
#if defined(FATTACH_VIA_MOUNT)
void lis_super_put_super(struct super_block *) ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) &&\
    LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
/* This existed between 8b512d9a88875affe584bb3d2a7a235f84343b9e (v2.6.18-rc1)
 * and was removed in 42faad99658eed7ca8bd328ffa4bcb7d78c9bcca (v2.6.26-rc1) */
void lis_super_umount_begin( struct vfsmount *vfsmnt, int flags );
#else
void lis_super_umount_begin( struct super_block *sb );
#endif
#endif

struct super_operations lis_super_ops =
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
    /* put_inode removed in 33dcdac (2.6.26.rc1 */
    put_inode:		lis_super_put_inode,
#endif
    statfs:		lis_super_statfs,
#if defined(KERNEL_2_5)
    drop_inode:		lis_drop_inode,
#endif
#if defined(FATTACH_VIA_MOUNT)
    umount_begin:       lis_super_umount_begin,
    put_super:          lis_super_put_super,
#endif
} ;

#if defined(KERNEL_2_5)
#define S_FS_INFO(s)    ((s)->s_fs_info)
#else
#define S_FS_INFO(s)    ((s)->u.generic_sbp)
#endif

/*
 *  the following predicate macros identify structures as
 *  LiS-specific, i.e., allocated by LiS for LiS use only.
 */
#define S_IS_LIS(s)     ( (s) ? ((s)->s_op == &lis_super_ops) : 0 )
#define I_IS_LIS(i)     ( (i) ? ((i)->i_sb && S_IS_LIS((i)->i_sb)) : 0 )

/***********************************************************************/

/*
 * Base name of a file
 */
#define	BFN(fname)	lis_basename(fname)


/* MODCNT and lis_modcnt would be in <sys/LiS/modcnt.h>, except that I
 * consider them dangerous - JB
 */
#define MODCNT()	lis_modcnt(THIS_MODULE)

static inline
long lis_modcnt( struct module *mod )
{
#if defined(THIS_MODULE)
# if defined(KERNEL_2_5)
#  ifdef CONFIG_MODULE_UNLOAD
    return(module_refcount(mod)) ;
#  else
    return (0);			/* refcnts are very buried */
#  endif
# else
    return (mod ? (long) atomic_read(&(mod->uc.usecount)) : 0);
# endif
#else
    return 0
#endif
}



#if defined(KERNEL_2_5)

#define MODSYNC()	lis_modsync_dbg(__LIS_FILE__,__LINE__,__FUNCTION__)

static
void lis_modsync_dbg(const char *file, int line, const char *fn)
{
#ifdef THIS_MODULE
    long mod_cnt = lis_modcnt(THIS_MODULE);

    if (LIS_DEBUG_REFCNTS)
	if ((THIS_MODULE)->name) {
	    printk("lis_modcnt_sync() <\"%s\"/%ld> {%s@%d,%s()}\n",
		   (THIS_MODULE)->name, mod_cnt, BFN(file), line, fn);
	}
#endif
}

#else				/* defined(KERNEL_2_5) */

#define MODSYNC()	do {} while (0)

#endif				/* defined(KERNEL_2_5) */

/* local inlines for separate loadables that have included <linux/module.h> -
 * such sources are self-contained on this issue
 */
static inline
void lis_modget_local(struct module *modulep, 
		      const char *file, int line, const char *fn)
{
#if defined(LINUX) && defined(__KERNEL__) && defined(MODULE)
    if (LIS_DEBUG_REFCNTS)
	printk("lis_modget_local() <\"%s\">++ {%s@%d,%s()}\n",
		     (modulep)->name, file, line, fn) ;

#if defined(KERNEL_2_5)
    try_module_get(modulep);
#else
    __MOD_INC_USE_COUNT(modulep);
#endif
#endif
}

static inline
void lis_modput_local(struct module *modulep, 
		      const char *file, int line, const char *fn)
{
#if defined(LINUX) && defined(__KERNEL__) && defined(MODULE)
    if (LIS_DEBUG_REFCNTS)
	printk("lis_modput_local() <\"%s\">-- {%s@%d,%s()}\n",
		     (modulep)->name, file, line, fn) ;
#if defined(KERNEL_2_5)
    module_put(modulep);
#else
    __MOD_DEC_USE_COUNT(modulep);
#endif
#endif
}

void _RP lis_modget(struct module *modulep, const char *file, int line, const char *fn)
{
    if (LIS_DEBUG_REFCNTS)
	printk("lis_modget() <\"%s\"/%ld>++ {%s@%d,%s()}\n",
	       (modulep)->name,
	       lis_modcnt(modulep),
	       BFN(file), line, fn) ;

    lis_modget_local(modulep, file, line, fn);
}

void _RP lis_modput(struct module *modulep, const char *file, int line, const char *fn)
{
    long cnt ;

    if (LIS_DEBUG_REFCNTS)
	printk("lis_modput() <\"%s\"/%ld>-- {%s@%d,%s()}\n",
	       modulep->name,
	       lis_modcnt(modulep),
	       BFN(file), line, fn) ;

#if !defined(KERNEL_2_5) || defined(CONFIG_MODULE_UNLOAD)
    if ((cnt = lis_modcnt(modulep)) <= 0)
    {
	printk("lis_modput() >> error -- count=%ld {%s@%d,%s()}\n",
	       cnt, BFN(file), line, fn) ;
	return ;
    }
#endif

    lis_modput_local(modulep, file, line, fn);
}

void _RP lis_modget_dbg(const char *file, int line, const char *fn)
{
#ifdef THIS_MODULE
    if (LIS_DEBUG_REFCNTS)
	printk("lis_modget() <\"%s\"/%ld>++ {%s@%d,%s()}\n",
	       (THIS_MODULE)->name,
	       lis_modcnt(THIS_MODULE),
	       BFN(file), line, fn) ;

    lis_modget_local(THIS_MODULE, file, line, fn);
#endif
}

void _RP lis_modput_dbg(const char *file, int line, const char *fn)
{
#ifdef THIS_MODULE
    long cnt ;

    if (LIS_DEBUG_REFCNTS)
	printk("lis_modput() <\"%s\"/%ld>-- {%s@%d,%s()}\n",
	       THIS_MODULE->name,
	       lis_modcnt(THIS_MODULE),
	       BFN(file), line, fn) ;

#if !defined(KERNEL_2_5) || defined(CONFIG_MODULE_UNLOAD)
    if ((cnt = lis_modcnt(THIS_MODULE)) <= 0)
    {
	printk("lis_modput() >> error -- count=%ld {%s@%d,%s()}\n",
	       cnt, BFN(file), line, fn) ;
	return ;
    }
#endif

    lis_modput_local(THIS_MODULE, file, line, fn);
#endif
}

/* some kernel memory has been free'd 
 * tell STREAMS
 */
void lis_memfree( void )
{
}/*lis_memfree*/

/* This will copyin usr string pointed by ustr and return the result  in
 * *kstr. It will stop at  '\0' or max bytes copyed in.
 * caller should call FREE(*kstr) on success.
 * Will return 0 or errno
 * STATUS: complete, untested
 */
int 
lis_copyin_str(struct file *f, const char *ustr, char **kstr, int maxb)
{
	int    error;
	char  *mem ;

	(void) f ;
	if (maxb <= 0)
	    return(-ENOMEM);
	error = -EFAULT;

	if((mem = ALLOCF(maxb,"copyin-buf ")) == NULL)
	    return(-ENOMEM);

	*kstr = mem ;
	error = strncpy_from_user(mem, ustr, maxb) ;
	if (error < 0)
	{
	    FREE(mem);
	    return(error) ;
	}

	if (error >= maxb)
	{
	    FREE(mem);
	    return(-ENAMETOOLONG) ;
	}

	return(0) ;
}/*lis_copyin_str*/

/************************************************************************
*                    Major/Minor Device Number Handling                 *
*************************************************************************
*									*
* These routines handle major/minor device numbers in LiS internal	*
* format.								*
*									*
************************************************************************/

#define LIS_MINOR_BITS          20
#define LIS_MINOR_MASK          ( (1 << LIS_MINOR_BITS) - 1 )

major_t _RP lis_getmajor(dev_t dev)
{
    return( dev >> LIS_MINOR_BITS ) ;
}

minor_t _RP lis_getminor(dev_t dev)
{
    return( dev & LIS_MINOR_MASK ) ;
}

dev_t _RP lis_makedevice(major_t majr, minor_t minr)
{
    return( (majr << LIS_MINOR_BITS) | (minr & LIS_MINOR_MASK) ) ;
}

/*
 * dev is really a kernel dev_t structure
 */
dev_t lis_kern_to_lis_dev(dev_t dev)
{
    return( lis_makedevice(STR_KMAJOR(dev), STR_KMINOR(dev)) ) ;
}


/*
 * Extract i_rdev from an inode.  If it's an LiS inode then not much
 * needs to be done.  If it is still a kernel inode then we need to
 * apply our translation routine to reconstruct the device id as an
 * LiS style dev_t.
 */
dev_t lis_i_rdev(struct inode *i)
{
#if defined(KERNEL_2_5)
    if (I_IS_LIS(i))
	return(RDEV_TO_DEV(i->i_rdev)) ;

    return(lis_kern_to_lis_dev(i->i_rdev)) ;
#else
    if (I_IS_LIS(i))
    {
	lis_inode_info_t *p = (lis_inode_info_t *) &i->u ;
	return( p->dev ) ;
    }

    return(lis_kern_to_lis_dev(i->i_rdev)) ;
#endif
}


/*  -------------------------------------------------------------------  */
/*
 * lis_get_new_inode
 *
 * Depending upon kernel version and distribution this is either
 * get_empty_inode or new_inode.  The configuration script has figured
 * out which it is and set the GET_EMPTY_INODE macro accordingly
 */
static struct inode *lis_get_new_inode(struct super_block *sb)
{
    struct inode *i = NULL;;

    if (!sb)
    {
	printk("lis_get_new_inode() - NULL super block pointer\n") ;
	return(NULL) ;
    }

    i = GET_EMPTY_INODE(sb);

    /*
     *  mark this inode as a LiS inode, and count it
     */
    i->i_sb = sb;          /* ASSERT: sb is or will be lis_mnt->mnt_sb */
    K_ATOMIC_INC(&lis_inode_cnt);

#if 0
    printk("%s:%d lis_inode_cnt %d\n",__FUNCTION__, __LINE__, (K_ATOMIC_READ(&lis_inode_cnt)));
#endif
    
    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	printk("lis_get_new_inode(s@0x%p)%s%s >> i@0x%p/%d%s\n",
	       sb,
	       (S_IS_LIS(sb)?" <LiS>":""),
	       (lis_mnt && sb == lis_mnt->mnt_sb?" <lis_mnt>":""),
	       i, (i?I_COUNT(i):0),
	       (i&&I_IS_LIS(i)?" <LiS>":""));
    return(i);
}


/*  -------------------------------------------------------------------  */
/*				    Timeouts                             */

void
lis_tmout(struct timer_list *tl, void (*fn)(ulong), long arg, long ticks)
{
    init_timer(tl);
    tl->function= fn;
    tl->data	= arg;
    tl->expires	= jiffies + ticks;
    add_timer(tl);
}

void
lis_untmout( struct timer_list *tl)
{
    del_timer(tl);		/* don't care if timeout fcn is running */
}

/************************************************************************
*                             lis_time_till                             *
*************************************************************************
*									*
* Given a target time in terms of elapsed milli-seconds, in other words,*
* the same units as jiffies if jiffies were in milli-seconds, return	*
* the number of milli-seconds that it will take to reach to target time.*
*									*
* The return is negative if the target time is in the past, zero	*
* if it is the same as the current target time, and positive if		*
* target time is in the future.						*
*									*
************************************************************************/
long	_RP lis_time_till(long target_time)
{
	/* ZZZ Deal with odd values of HZ */
    return( target_time - jiffies*(1000/HZ) ) ;

} /* lis_time_till */

/************************************************************************
*                           lis_target_time                             *
*************************************************************************
*									*
* Convert the milli_sec interval to an absolute target time expressed	*
* in milli-seconds.  We compute this relative to the Linux software	*
* clock, jiffies, converted to milliseconds.				*
*									*
************************************************************************/
long	_RP lis_target_time(long milli_sec)
{
    return( jiffies*(1000/HZ) + milli_sec ) ;

} /* lis_target_time */

/************************************************************************
*                           lis_milli_to_ticks                          *
*************************************************************************
*									*
* Convert milli-seconds to native ticks suitable for use with system	*
* timeout routines.							*
*									*
************************************************************************/
long	_RP lis_milli_to_ticks(long milli_sec)
{
    return(milli_sec/(1000/HZ)) ;
}


/*  -------------------------------------------------------------------  */

/* find the head for a file descriptor.
 * STATUS: complete, untested
 */
stdata_t *
lis_fd2str( int fd )
{
    struct file		* file;
    stdata_t		* hd ;

    if ( (file = fcheck(fd)) == NULL)
	return(NULL) ;

    hd = FILE_STR(file) ;
    if (hd != NULL && hd->magic == STDATA_MAGIC)
	return(hd) ;			/* looks like a stdata structure */

    return(NULL) ;			/* not a streams file */
}

struct inode *lis_file_inode(struct file *f)
{
    return((f)->f_dentry->d_inode) ;
}

char *lis_file_name(struct file *f)
{
    return((char *)((f)->f_dentry->d_name.name)) ;
}

struct stdata *lis_file_str(struct file *f)
{
    return((struct stdata *)(f)->private_data) ;
}

void lis_set_file_str(struct file *f, struct stdata *s)
{
    f->private_data = (void *) s ;
}

struct stdata *lis_inode_str(struct inode *i)
{
#if defined(KERNEL_2_5)
#ifdef STRUCT_INODE_IPRIVATE
    return((struct stdata *)(i)->i_private) ;
#else
    return((struct stdata *)(i)->u.generic_ip) ;
#endif
#else
    {
	lis_inode_info_t *p = (lis_inode_info_t *) &i->u ;
	return(p->hdp) ;
    }
#endif
}

void lis_set_inode_str(struct inode *i, struct stdata *s)
{
#if defined(KERNEL_2_5)
#ifdef STRUCT_INODE_IPRIVATE
    i->i_private = (void *) s ;
#else
    i->u.generic_ip = (void *) s ;
#endif
#else
    lis_inode_info_t *p = (lis_inode_info_t *) &i->u ;
    p->hdp = s ;
#endif
}

struct dentry *lis_d_alloc_root(struct inode *i, int mode)
{
#if defined(FATTACH_VIA_MOUNT)
    struct dentry *d = NULL;
    struct qstr dname;
    int for_mount = (mode == LIS_D_ALLOC_ROOT_MOUNT);

    if (i) {
	stdata_t *head = INODE_STR(i);
	char name[48] = "";

	if (for_mount)
	    strcpy( name, LIS_FS_NAME "/" );
	if (head && *(head->sd_name)) {
	    strcat( name, head->sd_name );
	}
	name[47] = 0;

	dname.name = (unsigned char*) name;
	dname.len  = strlen(name) ;
	dname.hash = i->i_ino;

        d = d_alloc(NULL, &dname);
	if (d) {
	    d->d_sb     = i->i_sb;
	    d->d_parent = d;
	    d_instantiate(d, i);
	}
    }
    else
    {
	dname.name = (unsigned char*) LIS_FS_NAME"/";
	dname.len  = strlen((char*)dname.name);
	dname.hash = full_name_hash(dname.name, dname.len);
	d = d_alloc(NULL, &dname);
    }

    /*
     *  The following also identifies the dentry as a LiS-allocated dentry.
     */
    if (d)
	d->d_op = &lis_dentry_ops;

    return d;
#else   
    return(d_alloc_root(i)) ;
#endif
}


/*
 * A 2.6 kernel-ism.
 *
 * This is called when a file open is about to succeed.  The premise is
 * that we have replaced the dentry/inode that was originally passed to
 * lis_stropen so we need to "put" the corresponding cdev entry, if it
 * exists.
 *
 * The only way we have to get to the cdev entry is via the i_cdev field in
 * the inode.  The cdev structure has a list of inodes using it threaded 
 * through i_devices in the inode.
 *
 * It is extremely bad practice to dput the dentry/inode and have the inode
 * deallocated while it is still in the cdev list.  So if we are about to do 
 * that (d_count==1 and i_count==1) then we need to remove the inode from
 * the cdev's list.  We can't do this any earlier since by doing so we would
 * then lose our pointer back to the cdev structures, which would have
 * ref-counts too high.
 *
 * Note that file close does this via __fput().  So if the open fails
 * we don't call lis_cdev_put since there will be an __fput() done soon.
 * Also, if we did not replace the inode we also don't do this because
 * it will be done later at file close time.
 *
 * The spin lock that we use to protect this code takes care of the
 * case of simultaneous opens to LiS.  We really should use the kernel's
 * cdev_lock, but they did not export the symbol, so we can't.
 */
static void lis_cdev_put(struct dentry *d)
{
#if defined(KERNEL_2_5)
    struct inode	*inode = d->d_inode ;
    struct cdev		*cp ;
    static spinlock_t	 lock = SPIN_LOCK_UNLOCKED ;


    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_cdev_put: d@0x%p/%d/\"%s\" i@0x%p/%d\n",
		d, D_COUNT(d), d->d_name.name,
		inode, inode ? I_COUNT(inode) : 0) ;
	if (inode && inode->i_cdev)
	    printk(">> i->i_cdev: c@0x%p/%d/%x \"%s\"\n",
		inode->i_cdev,
/* MarkS@Adax - 31 Jan 2005 - Debian 2.6.8 kernel unfortunately did not
   implement use of the kref structure element as a replacement for
   refcount within the kobject structure defined in kobject.h in the
   kernel source.  LiS mkgenconf defines KOBJ_USES_KREF if needed. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8) && defined(KOBJ_USES_KREF)
		K_ATOMIC_READ(&inode->i_cdev->kobj.kref.refcount), 
#else 
		K_ATOMIC_READ(&inode->i_cdev->kobj.refcount),
#endif
		DEV_TO_INT(inode->i_cdev->dev),
		(inode->i_cdev->owner ?
		 inode->i_cdev->owner->name : "No-Owner")) ;
    }

    if (!inode || !(cp = inode->i_cdev))
	return ;

    spin_lock(&lock) ;
    if (   D_COUNT(d) == 1 && I_COUNT(inode) == 1
	&& !list_empty(&inode->i_devices))
    {
        inode->i_cdev = NULL ;
	list_del_init(&inode->i_devices);
    }

#if 0
    cdev_put(cp) ;
#endif
	/* This is all that cdev_put does... */
	kobject_put(&cp->kobj);
	module_put(cp->owner);

    spin_unlock(&lock) ;
#endif
}

void lis_dput(struct dentry *d)
{
    struct super_block	*sb = NULL ;

    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	lis_print_dentry(d, "lis_dput") ;

    if (d->d_inode)
    {
	sb = d->d_inode->i_sb ;		/* save before dput */
    }

    /*
     *  don't dput() a dentry to a - ref cnt; if that is needed,
     *  something else is wrong...
     */
    if (D_COUNT(d) > 0) {
	dput(d) ;

	if (sb && lis_mnt && sb == lis_mnt->mnt_sb)
	    MNTPUT(lis_mnt) ;
    }
}

struct dentry *lis_dget(struct dentry *d)
{
    if (d->d_inode != NULL && d->d_inode->i_sb == lis_mnt->mnt_sb)
	(void) MNTGET(lis_mnt) ;
    
    d = dget(d);

    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	lis_print_dentry(d, "lis_dget") ;

    return(d) ;
}

/************************************************************************
*                          lis_select                                   *
*************************************************************************
*									*
* Called from the Linux sys_select routine to interrogate the status	*
* of a stream.								*
*									*
* Return 1 if the stream is ready for the requested operation.  	*
* Return 0, and set up the wait queue entry, if it is not.		*
*									*
************************************************************************/
#if !defined(LINUX_POLL)
int      lis_select(struct inode *inode, struct file *file,
		    int sel_type, select_table *wait)
{
    stdata_t		*hd;
    polldat_t		 pdat ;
    int			 evts ;
    int			 msk ;
    int			 rtn ;

    memset(&pdat, 0, sizeof(pdat)) ;		/* just in case */
retry:
    switch (sel_type)
    {
    case SEL_IN:
	pdat.pd_events = POLLIN | POLLPRI | POLLRDNORM | POLLRDBAND ;
	msk = POLLNVAL ;
	break ;
    case SEL_OUT:
	pdat.pd_events = POLLOUT | POLLWRNORM | POLLWRBAND ;
	msk = POLLNVAL ;
	break ;
    case SEL_EX:
    default:
	pdat.pd_events = POLLMSG ;			/* SIG-POLL msg */
	msk = POLLERR | POLLHUP | POLLNVAL ;
	break ;
    }

    evts = lis_strpoll(inode, file, &pdat) ;	/* get stream status */
    rtn = ( (evts & (pdat.pd_events | msk)) != 0 ) ;/* expected status */
    if (rtn) return(1) ;			/* yes, can do that */

    /*
     * Requested events are not present, add ourselves to the
     * wait queue for the stream.  Set a flag to be noticed by
     * str_rput.
     *
     * If we get back from select_wait and the flag has been unset then
     * lis_select_wakeup got called before returning to us and possibly
     * before we even got into the wait list.  In that case, we go around
     * again and return the actual events.
     */
    hd = FILE_STR(file) ;
    SET_SD_FLAG(hd,STRSELPND);
    select_wait(&hd->sd_select.sel_wait, wait) ;
    if (!F_ISSET(hd->sd_flag,STRSELPND))	/* lost the race */
	goto retry ;

    return(0) ;

} /* lis_select */
#endif

/************************************************************************
*                        lis_select_wakeup                              *
*************************************************************************
*									*
* Wake up those waiting on this stream.					*
*									*
************************************************************************/
#if !defined(LINUX_POLL)
void lis_select_wakeup(stdata_t *hd)
{
    CLR_SD_FLAG(hd,STRSELPND);	/* no longer waiting on select */
    if (   hd == NULL
	|| hd->magic != STDATA_MAGIC
	|| hd->sd_select.sel_wait == NULL
	|| hd->sd_select.sel_wait == WAIT_QUEUE_HEAD(&hd->sd_select.sel_wait)
       )
	return ;

    wake_up_interruptible(&hd->sd_select.sel_wait) ;

} /* lis_select_wakeup */
#endif

/*
 *  a debugging routine to show the aliases for an inode
 */

void lis_show_inode_aliases( struct inode *i )
{
#if defined(KERNEL_2_1)
    struct list_head *ent;

    if (!(LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) ||
	list_empty(&(i->i_dentry)))
	return;

    printk("lis_show_inode_aliases(i@0x%p/%d.%d#%lu)%s:\n",
	   i, I_COUNT(i), i->i_nlink, i->i_ino, (I_IS_LIS(i)?" <LiS>":""));

    /*
     *  show aliases in oldest-to-newest order
     */
    for (ent = i->i_dentry.prev;  ent != &(i->i_dentry);  ent = ent->prev) {
	struct dentry *d = list_entry( ent, struct dentry, d_alias );

	lis_print_dentry(d, ">> dentry") ;
    }
#endif
}

/*
 *  kernel assistance to show a file's full path (using kernel's
 *  __d_path() routine)
 */
char *lis_alloc_file_path(void)
{
    return (char *) __get_free_page(GFP_USER);
}

char *lis_format_file_path(struct file *f, char *page)
{
    if (page) {
/* d_path changed in cf28b4863f9ee8f122e8ff3ac0d403e07ba9c6d9 (v.2.6.25-rc2). */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
	char *path =   d_path( &f->f_path, page, PAGE_SIZE);
#elif defined(KERNEL_2_5) 
	char *path =   d_path( f->f_dentry,
			       FILE_MNT(f),
			       page, PAGE_SIZE);
#else
	char *path = __d_path( f->f_dentry,
			       FILE_MNT(f),
			       FILE_MNT(f)->mnt_root,
			       NULL,
			       page, PAGE_SIZE);
#endif
	return path;
    } else
	return page;  /* which is NULL... */
}

void lis_free_file_path(char *page)
{
    if (page)
    	free_page((unsigned long)page);
}

void lis_print_file_path(struct file *f)
{
    char *page = lis_alloc_file_path();

    if (page) {
	char *path = lis_format_file_path(f, page);

	if (path)
	    printk("%s", path);

	lis_free_file_path(page);
    }
}


/*
 * lis_print_dentry
 */
void lis_print_dentry(struct dentry *d, char *comment)
{
    char	dname[100] ;
    int		len = sizeof(dname)-1 ;
    struct inode *i = (d ? d->d_inode : NULL) ;

    if (d == NULL)
    {
	printk("%s  NULL dentry pointer\n", comment) ;
	return ;
    }

    if (comment == NULL)
	comment = "" ;

    if (d->d_name.len < len)  len = d->d_name.len;
    strncpy(dname, (char*)d->d_name.name, len) ;
    dname[len] = 0 ;

    printk("%s: d@0x%p/%d%s m:%d", comment,
	   d, D_COUNT(d), (D_IS_LIS(d)?" <LiS>":""),
	       K_ATOMIC_READ(&lis_mnt_cnt));
    if (i)
    {
	printk(" i@0x%p/%d%s",
	       i, I_COUNT(i), (I_IS_LIS(i)?" <LiS>":""));
#if defined(KERNEL_2_5)
	if (i->i_cdev)
	    printk(" c@0x%p/%x\"%s\"", i->i_cdev, DEV_TO_INT(i->i_cdev->dev),
		(i->i_cdev->owner ? i->i_cdev->owner->name : "No-Owner")) ;
#endif
    }
    if (*dname)
	printk(" \"%s\"", dname );
    printk("\n");

    if (d && d->d_parent != NULL && d->d_parent != d)
	lis_print_dentry(d->d_parent, ">> parent") ;
}

/************************************************************************
*                         lis_super_statfs                              *
*************************************************************************
*									*
* Return file system stats.						*
*									*
************************************************************************/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))
/* Changed in 726c334223180e3c0197cc980a432681370d4baf v.2.6.18-rc1 */
int lis_super_statfs(struct dentry *dentry, struct kstatfs *stat)
#elif defined(KERNEL_2_5)
int lis_super_statfs(struct super_block *sb, struct kstatfs *stat)
#else
int lis_super_statfs(struct super_block *sb, struct statfs *stat)
#endif
{
    stat->f_type = LIS_SB_MAGIC ;
    stat->f_bsize = 1024 ;
    stat->f_namelen = 255 ;
    return(0) ;

}

static
int lis_fs_fattach_sb( struct super_block *sb, void *ptr, int silent )
{
#if defined(FATTACH_VIA_MOUNT)
    lis_fattach_t *data = (ptr ? *((lis_fattach_t **) ptr) : NULL);
    struct file *file   = (data ? data->file : NULL);
    stdata_t *head      = (data ? data->head : NULL);
    const char *path    = (data ? data->path : NULL);
  
  if (file && head &&
      head == FILE_STR(file) &&
      head->magic == STDATA_MAGIC &&
      head->sd_inode) {
      /*
       *  this is an fattach via the mount() syscall - set it up
       */
      struct inode *i_mount = head->sd_inode;
      struct dentry *d_mount;

      lis_head_get(head);                  /* bumps refcnt */
      MNTSYNC();

      if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) {
	  printk("lis_fs_fattach_sb(s@0x%p,@0x%p,...) << [%d]"
		 " f@0x%p/%d f_vfsmnt 0x%p/%d%s%s\n",
		 sb, data,
		 K_ATOMIC_READ(&lis_mnt_cnt),
		 file, (file?F_COUNT(file):0),
		 FILE_MNT(file),
		 (FILE_MNT(file)?MNT_COUNT(FILE_MNT(file)):0),
		 (S_IS_LIS(sb)?" <LiS>":""),
		 (file&&FILE_MNT(file)==lis_mnt?" <lis_mnt>":""));
      }

      LOCK_INO(i_mount);
      
      sb->s_root = d_mount = lis_d_alloc_root(igrab(i_mount),
					      LIS_D_ALLOC_ROOT_MOUNT);
      if (sb->s_root == NULL) {
	  ULOCK_INO(i_mount);
	  lis_head_put(head);
	  return(-ENOMEM) ;
      }
      S_FS_INFO(sb) = data;
      if (FILE_MNT(file))
	  data->mount = MNTGET(FILE_MNT(file));
      else 
	  data->mount = file->f_vfsmnt = MNTGET(lis_mnt);
      data->sb      = sb;
      data->dentry  = d_mount;

      /*
       *  sd_refcnt used to keep a stream alive until all uses were done,
       *  but doclose essentially ignores it now, and uses sd_opencnt
       *  instead.  So, we have to bump both sd_refcnt (via lis_head_get())
       *  and sd_opencnt in order to make sure this stream doesn't disappear
       *  when the attaching file is closed - the file has to disappear, but
       *  the stream has to stay "open".
       */
      K_ATOMIC_INC(&head->sd_opencnt);

      lis_spin_lock(&lis_fattaches_lock);
      K_ATOMIC_INC(&head->sd_fattachcnt);
      SET_SD_FLAG(head,STRATTACH);
      lis_spin_unlock(&lis_fattaches_lock);
      
      /*
       *  we need the following in order to be able to both read and
       *  write the fattached stream, if permissions are appropriately
       *  set as well - the kernel makes it read-only otherwise
       */
      allow_write_access(file);

      ULOCK_INO(i_mount);
      
      if ((LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) &&
	  path) {
	  dev_t dev = GET_I_RDEV(i_mount);
          printk("lis_fs_fattach_sb(s@0x%p,@0x%p,%d) "
		 ">> d@0x%p/%d i@0x%p/%d rdev (%d,%d)\n",
		 sb, data, silent,
		 d_mount, D_COUNT(d_mount),
		 i_mount, I_COUNT(i_mount),
		 getmajor(dev), getminor(dev) );
	  printk("lis_fs_fattach_sb(s@0x%p,@0x%p,...)\n"
		 "    >> f@0x%p/%d h@0x%p/%d/%d \"%s\""
		 " sb@0x%p d@0x%p/%d (i@0x%p/%d)\n",
		 sb, data,
		 file, F_COUNT(file),
		 head, LIS_SD_REFCNT(head), LIS_SD_OPENCNT(head),
		 path,
		 data->sb,
		 d_mount, D_COUNT(d_mount),
		 i_mount, I_COUNT(i_mount));
	  lis_show_inode_aliases(i_mount);
      }

      if ((LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) &&
	  file) {
	  printk("lis_fs_fattach_sb(s@0x%p,@0x%p,...) >> [%d]"
		 " f@0x%p/%d f_vfsmnt 0x%p/%d%s%s\n",
		 sb, data,
		 K_ATOMIC_READ(&lis_mnt_cnt),
		 file, F_COUNT(file),
		 FILE_MNT(file),
		 (FILE_MNT(file)?MNT_COUNT(FILE_MNT(file)):0),
		 (S_IS_LIS(sb)?" <LiS>":""),
		 (FILE_MNT(file)==lis_mnt?" <lis_mnt>":""));
      }

      MNTSYNC();

      return(0);
  } else
      return(-EINVAL);  /* this must be a stream */
#else			/* FATTACH_VIA_MOUNT */
  return(-ENOSYS) ;	/* not implemented */
#endif			/* FATTACH_VIA_MOUNT */
}

static
int lis_fs_kern_mount_sb( struct super_block *sb, void *ptr, int silent )
{
    struct inode *isb = lis_get_new_inode(sb) ;

    if (isb == NULL)
      return(-ENOMEM) ;
  
    isb->i_mode  = S_IFDIR | S_IRUSR | S_IWUSR ;
    isb->i_uid   = 0 ;
    isb->i_gid   = 0 ;
    isb->i_atime = CURRENT_TIME ;
    isb->i_mtime = CURRENT_TIME ;
    isb->i_ctime = CURRENT_TIME ;
    isb->i_op    = &lis_streams_iops;
#if defined(KERNEL_2_5)
    isb->i_rdev  = makedevice(lis_major, 0);	/* LiS dev_t */
#else
    isb->i_rdev  = MKDEV(lis_major, 0);	/* kernel dev_t */
    {
	lis_inode_info_t *p = (lis_inode_info_t *) &isb->u ;
	p->dev = makedevice(lis_major, 0) ;
    }
#endif
    
    sb->s_root = lis_d_alloc_root(isb,LIS_D_ALLOC_ROOT_MOUNT);
    if (sb->s_root == NULL)
        return(-ENOMEM) ;

    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	printk("lis_fs_kern_mount_sb(s@x%p,...)"
	       " >> root d@0x%p/%d i@0x%p/%d rdev (%d,%d)\n",
	       sb,
	       sb->s_root, D_COUNT(sb->s_root),
	       isb, I_COUNT(isb),
	       getmajor(GET_I_RDEV(isb)),
	       getminor(GET_I_RDEV(isb)) );
    
    return(0);
}

int lis_fs_setup_sb(struct super_block *sb, void *ptr, int silent)
{
    sb->s_magic		 = LIS_SB_MAGIC ;
    sb->s_blocksize	 = 1024 ;
    sb->s_blocksize_bits = 10 ;
    sb->s_op		 = &lis_super_ops ;
 
    if (ptr) 
        return lis_fs_fattach_sb( sb, ptr, silent );
    else
        return lis_fs_kern_mount_sb( sb, ptr, silent );
}

#if defined(KERNEL_2_5)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
int lis_fs_get_sb(struct file_system_type *fs_type, int flags,
		  const char *dev_name, void *ptr, struct vfsmount *mnt)
{
    return get_sb_nodev(fs_type, flags, ptr, lis_fs_setup_sb, mnt);
}
#else
struct super_block *lis_fs_get_sb(struct file_system_type *fs_type,
				  int flags,
				  const char *dev_name,
				  void *ptr)
{
    struct super_block *sb;
  /*
   * Not sure which technique to use.  May need to use get_sb_single
   * in order to utilize sys_mount to implement fattach.  We'll see.
   *
   * 2002/11/18 - nodev is the right one for fattach... - JB
   */
#if defined(FATTACH_VIA_MOUNT) && 1
    sb = get_sb_nodev(fs_type, flags, ptr, lis_fs_setup_sb);
#else
    sb = get_sb_single(fs_type, flags, ptr, lis_fs_setup_sb);
#endif
    return sb;
}
#endif

void lis_fs_kill_sb(struct super_block *sb)
{
    kill_anon_super(sb);
}
#else		/* 2.4 kernel */
struct super_block *lis_fs_read_super(struct super_block *sb,
				      void *ptr,
				      int   silent)
{
    if (lis_fs_setup_sb(sb, ptr, silent) < 0)
        return(NULL) ;

    return(sb) ;
}
#endif

/************************************************************************
*                         lis_dentry_delete                             *
*************************************************************************
*									*
* This routine has to be here (in 2.4 kernel) and has to return 1 in	*
* order to get the kernel's dput routine to go ahead and iput the inode	*
* beneath the dentry when the dentry's d_count goes to zero.  Duh.  Like*
* the default behavior should be to just keep the dangling inode around	*
* until the file system unmounts so that then the kernel can print a	*
* message about dangling inodes.					*
*									*
************************************************************************/
int lis_dentry_delete(struct dentry *d)
{
    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	printk("lis_dentry_delete(d@0x%p/%d) [%d]%s\n",
	       d, D_COUNT(d),
	       K_ATOMIC_READ(&lis_mnt_cnt),
	       (D_IS_LIS(d)?" d<LiS>":""));

    return(1);
}

void lis_dentry_iput( struct dentry *d, struct inode *i )
{
    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	lis_print_dentry(d, "lis_dentry_iput") ;

#if 0
        printk("%s:%d I_COUNT(i) = %d, lis_inode_cnt %d\n",
               __FUNCTION__, __LINE__, I_COUNT(i), (K_ATOMIC_READ(&lis_inode_cnt)));
#endif
    
    if (i && I_COUNT(i) > 0)
    {        
	lis_put_inode(i);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)

        if((K_ATOMIC_READ(&lis_inode_cnt)) >= 1)
        {
            K_ATOMIC_DEC(&lis_inode_cnt) ;

#if 0
            printk("%s:%d I_COUNT(i) = %d, lis_inode_cnt %d\n",
                   __FUNCTION__, __LINE__, I_COUNT(i), (K_ATOMIC_READ(&lis_inode_cnt)));
#endif
        }

#endif

    }   
}

/************************************************************************
*                        lis_super_put_inode                            *
*************************************************************************
*									*
* Called when an LiS inode is being "iput".				*
*									*
************************************************************************/
void lis_super_put_inode(struct inode *i)
{
    MNTSYNC();

#if 0
        printk("%s:%d I_COUNT(i) = %d, lis_inode_cnt %d\n",
               __FUNCTION__, __LINE__, I_COUNT(i), (K_ATOMIC_READ(&lis_inode_cnt)));
#endif
    
    if (LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_super_put_inode(i@0x%p/%d)%s "
	       "i_rdev=0x%x <[%d] %d LiS inode(s)>%s\n",
	       i, I_COUNT(i), (I_IS_LIS(i)?" <LiS>":""),
	       GET_I_RDEV(i),
	       K_ATOMIC_READ(&lis_mnt_cnt),
	       K_ATOMIC_READ(&lis_inode_cnt),
	       (I_COUNT(i) == 1 ? "--" : ""));
    }

    if (I_COUNT(i) <= 1)
    {
	if (I_COUNT(i) < 1)
	    printk("lis_super_put_inode(i@0x%p/%d)%s i_rdev=0x%x UNUSED"
		   " <[%d] %d LiS inode(s)>\n",
		   i, I_COUNT(i), (I_IS_LIS(i)?" <LiS>":""),
		   GET_I_RDEV(i),
		   K_ATOMIC_READ(&lis_mnt_cnt),
		   K_ATOMIC_READ(&lis_inode_cnt));
	else
        {
            
	    K_ATOMIC_DEC(&lis_inode_cnt) ;

#if 0
        printk("%s:%d I_COUNT(i) = %d, lis_inode_cnt %d\n",
               __FUNCTION__, __LINE__, I_COUNT(i), (K_ATOMIC_READ(&lis_inode_cnt)));
#endif

        }
            

#if !defined(KERNEL_2_5)
	force_delete(i) ;
#endif
    }
}

#if defined(FATTACH_VIA_MOUNT)
/*
 *  lis_super_ops hooks for supporting fdetach() via sys_umount[2]() -
 *
 *  The following two routines work together to make sure that when
 *  umount[2] is called to do an fdetach(), all the work actually
 *  gets done.  The intent is that 'umount2( path, MNT_FORCE )' will
 *  be invoked, and this is exactly equivalent to 'umount -f path'
 *  from a command line.  Unfortunately, that means that the '-f'
 *  flag could be omitted if attempted from the command line.  Luckily,
 *  (in 2.4.19, anyway - not yet tested in 2.5.x), this case still
 *  manages to invoke lis_super_put_super(), but without invoking
 *  lis_super_umount_begin() first, and after partially clobbering
 *  the superblock and dentry.  So, here, we support both the
 *  intended pattern of umount_begin() -> put_super(), but also
 *  put_super() -> umount_begin().
 *
 *  Keep your fingers crossed - this is likely to be affected by
 *  kernel changes... 8^(
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) &&\
    LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
/* This existed between 8b512d9a88875affe584bb3d2a7a235f84343b9e (v2.6.18-rc1)
 * and was removed in 42faad99658eed7ca8bd328ffa4bcb7d78c9bcca (v2.6.26-rc1) */
void lis_super_umount_begin( struct vfsmount *vfsmnt, int flags )
#else
void lis_super_umount_begin( struct super_block *sb )
#endif
{
    lis_fattach_t *data;
    struct file *file;
    struct vfsmount *mount;
    stdata_t *head;
    struct dentry *d_umount;
    struct inode *i_umount;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) &&\
    LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
    struct super_block *sb;
    sb = vfsmnt->mnt_sb;
#endif

    data = (lis_fattach_t *) S_FS_INFO(sb);
    file = (data ? data->file : NULL);  
    mount = (data ? data->mount : NULL);
    head = (data ? data->head : NULL);
    d_umount = (data ? data->dentry : NULL);
    i_umount = (d_umount ? d_umount->d_inode : NULL);

    MNTSYNC();

    /*
     *  If put_super() was called first, 'd_umount' has been partially
     *  clobbered and its inode has been disconnected already -
     *  deal with that by using the (same) inode from the head
     */
    if (data && head && d_umount && !i_umount && head->sd_inode)
	i_umount = head->sd_inode;

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	printk("lis_super_umount_begin(s@0x%p) << data@0x%p:\n"
	       "    f@0x%p m@0x%p/%d h@0x%p/%d/%d"
	       " d@0x%p/%d (i@0x%p/%d)\n",
	       sb, data, file,
	       mount, (mount?MNT_COUNT(mount):0),
	       head,
	       (head?LIS_SD_REFCNT(head):0),
	       (head?LIS_SD_OPENCNT(head):0),
	       d_umount, (d_umount?D_COUNT(d_umount):0),
	       i_umount, (i_umount?I_COUNT(i_umount):0));

    /*
     *  make sure we have what we need - if we don't, it's not an
     *  fdetach()
     */
    if (data && head && d_umount && i_umount) {
	cred_t creds;

	LOCK_INO(i_umount);

	/*
	 *  give back the write access we established at fattach time
	 */
	put_write_access(i_umount);
	/*
	 *  give back the mount count we set at fattach time
	 */
	if (mount)
	    MNTPUT(mount);

	if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) {
	    printk("lis_super_umount_begin(%p) [fdetach] "
		   "i@0x%p/%d d@0x%p/%d \"%s\"\n",
		   sb,
		   i_umount, I_COUNT(i_umount),
		   d_umount, D_COUNT(d_umount),
		   data->path );

	    lis_show_inode_aliases(i_umount);
	}

	creds.cr_uid  = (uid_t) EUID(current);
	creds.cr_gid  = (gid_t) EGID(current);
	creds.cr_ruid = (uid_t) UID(current);
	creds.cr_rgid = (gid_t) GID(current);
	/*
	 *  clear the STRATTACH flag if no other fattaches for this stream
	 */
	lis_spin_lock(&lis_fattaches_lock);
	K_ATOMIC_DEC(&head->sd_fattachcnt);
	if (K_ATOMIC_READ(&head->sd_fattachcnt) <= 0)
	    CLR_SD_FLAG(head,STRATTACH);
	lis_spin_unlock(&lis_fattaches_lock);

	ULOCK_INO(i_umount);

	lis_doclose( i_umount, NULL, head, &creds );

	/*
	 *  the file field isn't valid after the fattach, but we
	 *  wait to clear it to indicate we've been here, in case
	 *  'umount <path>' instead of 'umount -f <path' gets used
	 *  and lis_super_put_super() is invoked first as a result
	 *
	 *  similarly, we may get here when the fattach is "busy",
	 *  and need to come back again (e.g., via fattach_all()).
	 *  we clear the head field (which put_super() doesn't use)
	 *  so we won't come back here again.
	 */
	data->mount = NULL;
	data->file  = NULL;
	data->head  = NULL;

	if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    printk("lis_super_umount_begin(s@0x%p) [fdetach] "
		   "d@0x%p/%d (i@0x%p/%d) after doclose\n",
		   sb,
		   d_umount, (d_umount?D_COUNT(d_umount):0),
		   i_umount, (i_umount?I_COUNT(i_umount):0));
    }

    MNTSYNC();
}

void lis_super_put_super( struct super_block *sb )
{
    lis_fattach_t *data     = (lis_fattach_t *) S_FS_INFO(sb);
    struct dentry *d_umount = (data ? data->dentry : NULL);
    struct inode *i_umount  = (d_umount ? d_umount->d_inode : NULL);

    MNTSYNC();

    if (data && data->file) {
	if (LIS_DEBUG_FATTACH)
	    printk("lis_super_put_super(s@0x%p) [fdetach] invoked w/o"
		   " lis_super_umount_begin()!!!\n",
		   sb);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18) &&\
    LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
	lis_super_umount_begin(data->mount, 0);
#else
	lis_super_umount_begin(sb);
#endif

	/* make sure these are still valid */
	data     = (lis_fattach_t *) S_FS_INFO(sb);
	d_umount = (data ? data->dentry : NULL);
	/*
	 *  don't bother getting the inode from the dentry - it's been
	 *  clobbered by now - get it from the head
	 */
	i_umount = (data && data->head ? data->head->sd_inode : NULL);
    }
    if (d_umount && i_umount) {
	if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    printk("lis_super_put_super(s@0x%p) [fdetach] "
		   "d@0x%p/%d (i@0x%p/%d)\n",
		   sb,
		   d_umount, D_COUNT(d_umount),
		   i_umount, (i_umount ? I_COUNT(i_umount) : 0) );

	lis_dput(d_umount);
    }
    if (data) {
	/*
	 *  remove the fattach instance from the list and delete it
	 */
	lis_fattach_remove(data);
	lis_fattach_delete(data);

	if (LIS_DEBUG_FATTACH)
	    printk("lis_super_put_super(s@0x%p) [fdetach] done\n", sb);
    }

    S_FS_INFO(sb) = NULL;

    MNTSYNC();
}
#endif /* FATTACH_VIA_MOUNT */

/************************************************************************
*                         lis_new_file_name                             *
*************************************************************************
*									*
* Change the name of the "file".  This means reallocate the dentry.	*
* If the allocation of the new dentry fails we just stick with the old	*
* one.									*
*									*
* This is not a rename operation.  We give up the old dentry and 	*
* allocate a new one with the new name.  The old dentry might actually	*
* represent some real file such as /dev/loop_clone.			*
*									*
* We allocate a new dentry using d_alloc.  We do not do a lookup on	*
* the name, because we want each stream to have a separate dentr and	*
* inode even if two streams have the same name.				*
*									*
************************************************************************/
int	lis_new_file_name(struct file *f, const char *name)
{
    /*
     *  don't do this if the dentry is already LiS and already has
     *  the desired name - we'd be changing dentries for nothing...
     */
    if (D_IS_LIS(f->f_dentry) &&
	strcmp(name, (char*)f->f_dentry->d_name.name) == 0) {
	if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    printk("lis_new_file_name(f@0x%p/%d,\"%s\")"
		   " - same name, already <LiS> - ignoring call\n",
		   f, F_COUNT(f), name);
	return(0);
    } else
	return(lis_new_file_name_dev(f, name, 0)) ;
}

int	lis_new_file_name_dev(struct file *f, const char *name, dev_t dev)
{
    struct qstr     dname ;
    struct dentry  *new ;
    struct dentry  *old = f->f_dentry;
    struct dentry  *lis_parent ;
    struct inode   *oldi = NULL;
    struct vfsmount *oldmnt = FILE_MNT(f);

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) {
	struct dentry *d = (f ? f->f_dentry : NULL);

	printk("lis_new_file_name_dev(f@0x%p/%d,\"%s\",0x%x)%s",
	       f, (f?F_COUNT(f):0), (name?name:""), dev,
	       (FILE_MNT(f)==lis_mnt?" <lis_mnt>":""));
	printk(" \"");
	if (FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(d, ">> dentry") ;
    }
    
    if (dev == 0)			/* must use old inode */
    {
	if (old != NULL)
	    oldi = old->d_inode ;
	else
	    return(-EINVAL) ;
    }
    else
    if ((oldi = lis_get_inode(S_IFCHR|S_IRUSR|S_IWUSR, dev)) == NULL)
	return(-ENOMEM) ;
    else
	oldi->i_mode &= ~current->fs->umask ;	/* umask considerations */

    dname.name = (unsigned char*)name ;			/* set up for d_alloc */
    dname.len  = strlen(name) ;
    dname.hash = full_name_hash(dname.name, dname.len) ;
    lis_parent = lis_mnt->mnt_sb->s_root ;
    new        = d_alloc(lis_parent, &dname) ;

    if (IS_ERR(new))			/* couldn't */
    {
	if (dev != 0)
	    lis_put_inode(oldi) ;
	return(PTR_ERR(new)) ;
    }

    /*
     *  we may have created a new file pointer with no dentry or inode
     *  and no f_vfsmnt set.  In that case, there's nothing to put, and
     *  we will do the initial setting of f_vfsmnt here.
     */
    f->f_vfsmnt = MNTGET(lis_mnt);	/* (re)mount on LiS */

    new->d_op   = &lis_dentry_ops;

    if (dev == 0)
    {					/* using old inode */
	d_add(new, igrab(oldi)) ;	/* old inode into new dentry */
	lis_dput(old) ;			/* does mntput if lis_mnt */
	if (oldmnt && oldmnt != lis_mnt)
	    MNTPUT(oldmnt) ;	        /* do it if not lis_mnt also */
    }
    else				/* using new inode */
	d_add(new, oldi) ;		/* new inode into new dentry */

    f->f_dentry = new ;			/* d_alloc set count */

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS) {
	struct dentry *d = (f ? f->f_dentry : NULL);

	printk("lis_new_file_name_dev(f@0x%p/%d,\"%s\",0x%x)%s",
	       f, (f?F_COUNT(f):0), (name?name:""), dev,
	       (FILE_MNT(f)==lis_mnt?" <lis_mnt>":""));
	printk(" \"");
	if (FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(d, ">> dentry") ;
    }

    return(0) ;
}

/*  -------------------------------------------------------------------  */
/* lis_new_stream_name							 */
/*
 * Helper routine to set up the name of the stream and associated file
 */
void lis_new_stream_name(struct stdata *head, struct file *f)
{
    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_new_stream_name(h@0x%p/%d/%d,f@0x%p/%d)\n",
	       head, LIS_SD_REFCNT(head), LIS_SD_OPENCNT(head),
	       f, F_COUNT(f)) ;
	lis_print_dentry(f->f_dentry, ">> dentry") ;
    }

    sprintf(head->sd_name, "%s%s",
	    lis_strm_name(head), lis_maj_min_name(head));

/* No KMEM-CACHE Work */
#if defined(CONFIG_DEV) || !defined(USE_KMEM_CACHE)
    lis_mark_mem(head, head->sd_name, MEM_STRMHD, BLK_HEAD);
#else
    lis_mark_mem(head, head->sd_name, MEM_STRMHD) ;
#endif
    
    lis_new_file_name(f, head->sd_name) ;

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_new_stream_name(h@0x%p/%d/%d,f@0x%p/%d)\n",
	       head, LIS_SD_REFCNT(head), LIS_SD_OPENCNT(head),
	       f, F_COUNT(f)) ;
	lis_print_dentry(f->f_dentry, ">> dentry") ;
    }
}

/************************************************************************
*                            lis_new_inode                              *
*************************************************************************
*									*
* This routine is called from stropen().				*
*									*
* 'old' is the inode passed to stropen for a clone open.  'oldf'	*
* is the file structure for the file that was opened.  The stdata	*
* structure hangs off of the file structure.				*
*									*
* 'dev' is the major/minor for the device that was really opened.	*
* Presumably, we are called because the i_rdev of the inode struct	*
* does not match up with dev.						*
*									*
* We need to allocate a new inode struct, and dentry for 2.2 kernels,	*
* and get them linked to this file.  The old inode and dentry		*
* represent the clone device and not the device that was actually	*
* opened.								*
*									*
* This operation is performed for every stream open whether clone or not*
*									*
************************************************************************/
struct inode *
lis_new_inode( struct file *f, dev_t dev )
{
    struct inode *old = FILE_INODE(f);
    struct inode *new;
    stdata_t	 *hd = FILE_STR(f) ;
    struct dentry *oldd = f->f_dentry;
    struct dentry *newd = NULL;
    struct qstr    dname ;
    struct vfsmount *oldmnt = FILE_MNT(f);

    if (!old)
    {						/* param checking */
	printk("lis_new_inode(f@0x%p/%d,d0x%x) - "
	       "old inode must be non-NULL\n",
	       f, F_COUNT(f), DEV_TO_INT(dev));
	return(NULL) ;				/* bad return */
    }

    if (lis_mnt == NULL)
    {
	printk("lis_new_inode(f@0x%p/%d,d0x%x) - "
	       "LiS has been unmounted\n",
	       f, F_COUNT(f), DEV_TO_INT(dev));
	return(NULL) ;				/* bad return */
    }

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_new_inode(f@0x%p/%d,dv0x%x) %s%s",
	       f, F_COUNT(f), DEV_TO_INT(dev),
	       (D_IS_LIS(f->f_dentry)?" <LiS>":""),
	       (FILE_MNT(f)==lis_mnt?" <lis_mnt>":"")
	      ) ;
	printk(" \"");
	if (FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(oldd, ">> oldd") ;
    }

    if (hd == NULL)
    {
	printk("lis_new_inode(f@0x%p/%d,d0x%x) - "
	       "no STREAM head\n",
	       f, F_COUNT(f), DEV_TO_INT(dev));
	return(NULL) ;				/* bad return */
    }

    /*
     *  we keep an old inode only if it's a LiS-only inode.  Otherwise,
     *  we'll replace a non-LiS inode here with one that is LiS-only
     */
    if (I_IS_LIS(old) && DEV_SAME(GET_I_RDEV(old), dev))
	return (old);

    new = lis_get_inode( old->i_mode, dev ) ;
    if (new != NULL)
    {						/* got a new inode */
	new->i_state = I_DIRTY;       /* keep it off the dirty list */
	new->i_mode  = old->i_mode;		/* inherit mode */
	/*
	 * Set the user/group ids to the opener, set modification times
	 * to the current time.
	 */
	new->i_uid   = FSUID(current);
	new->i_gid   = FSGID(current);
	new->i_atime = new->i_mtime = new->i_ctime = CURRENT_TIME;
	/*
	 * It is difficult to detach an inode from a dentry without
	 * dput-ing the whole dentry, what with alias lists and all.
	 * So we just allocate a new dentry with LiS as the parent
	 * directory and install our new inode into it.  We can then
	 * just dput the old dentry and be done with it.
	 */
	if (hd->sd_name[0])			/* have name for strm yet?  */
	{
	    dname.name = (unsigned char*)hd->sd_name ;
	    dname.len  = strlen(hd->sd_name) ;
	}
	else					/* use name from old dentry */
	{
	    dname.name = oldd->d_name.name ;
	    dname.len  = oldd->d_name.len ;
	}

	dname.hash = full_name_hash(dname.name, dname.len) ;
	newd = d_alloc(lis_mnt->mnt_sb->s_root, &dname) ;
	if (newd == NULL)
	{
	    lis_put_inode(new) ;			/* oops, couldn't */
	    return(NULL) ;
	}

	/*
	 *  we may have created a new file pointer with no dentry or inode
	 *  and no f_vfsmnt set.  In that case, there's nothing to put, and
	 *  we will do the initial setting of f_vfsmnt here.
	 */
	f->f_vfsmnt = MNTGET(lis_mnt);		/* (re)mount on LiS */
	lis_dput(oldd) ;                        /* does mntput() if lis_mnt */
	if (oldmnt && oldmnt != lis_mnt)
	    MNTPUT(oldmnt) ;                    /* do it if not lis_mnt also */

	newd->d_op = &lis_dentry_ops;
	d_add(newd, new) ;			/* add inode to new dentry */
	f->f_dentry = newd ;
    }

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_new_inode(f@0x%p/%d,d0x%x)%s%s",
	       f, F_COUNT(f), DEV_TO_INT(dev),
	       (D_IS_LIS(f->f_dentry)?" <LiS>":""),
	       (f&&FILE_MNT(f)==lis_mnt?" <lis_mnt>":"")
	      ) ;
	printk(" \"");
	if (f && FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(newd, ">> newd") ;
    }

    return(new) ;				/* use new inode */
    						/* possibly NULL return */
} /* lis_new_inode */

/*
 *  lis_cleanup_file... - do whatever "puts" need to be done to get
 *  operating system object usage counts decremented.
 *
 *  NB - the counts passed in here may or may not be useful; they're
 *  here for possible future use as much as anything (currently, only
 *  oldmnt_cnt is being used).
 */
void lis_cleanup_file_opening(struct file *f, stdata_t *head,
			       int open_fail,
			       struct dentry *oldd, int oldd_cnt,
			       struct vfsmount *oldmnt, int oldmnt_cnt)
{
    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
        printk("lis_cleanup_file_opening(f@0x%p/%d,h@0x%p/%d/%d,%d) %s\n",
	       f, (f?F_COUNT(f):0),
	       head,
	       (head?LIS_SD_REFCNT(head):0),
	       (head?LIS_SD_OPENCNT(head):0),
	       open_fail,
	       (FILE_MNT(f)==lis_mnt?" <lis_mnt>":"")) ;

	printk(" \"");
	if (FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(oldd, ">> dentry") ;
	printk("\n");
    }

    if (open_fail) {
	/*
	 *  open_fail: sys_open has the old dentry and mnt, and will put
	 *  them.  Synchronize accordingly...
	 *
	 *  When open fails chrdev_open does a cdev_put on the cdev
	 *  dangling off of the inode, so we don't do it here.
	 *
	 *  If we're not hooked to the original dentry/inode, put the
	 *  ones we're hooked to now, since sys_open() doesn't care
	 *  about them and won't put them.
	 *
	 */
	if (f->f_dentry != oldd)
	    (void)lis_dput(f->f_dentry);

	if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    printk("    error %d >> oldmnt@0x%p/%d %c oldmnt_cnt %d\n",
		   open_fail,
		   oldmnt, MNT_COUNT(oldmnt),
		   (MNT_COUNT(oldmnt) < oldmnt_cnt ? '<' :
		    MNT_COUNT(oldmnt) > oldmnt_cnt ? '>' : '='),
		   oldmnt_cnt);
	if (MNT_COUNT(oldmnt) < oldmnt_cnt)
	    (void)MNTGET(oldmnt);
	else
	if (MNT_COUNT(oldmnt) > oldmnt_cnt)
	    MNTPUT(oldmnt);
    } else {
	/*
	 *  open OK - these are extra
	 */
	if (f->f_dentry != oldd)
	    lis_cdev_put(oldd) ;
	lis_dput(oldd);
	MNTPUT(oldmnt);
    }
}

void lis_cleanup_file_closing(struct file *f, stdata_t *head)
{
    /*
     *  close KLUDGE: if the file's dentry count is > 1, the kernel
     *  won't do anything but decrement the count.  Among other things
     *  (possibly), it won't decrement the mount count.  So, we do that
     *  here for lis_mnt, if that's where we are (and it should be...).
     */
    struct dentry *d = (f ? f->f_dentry : NULL);

    if (f && d && D_IS_LIS(d) && D_COUNT(d) > 1 && FILE_MNT(f) == lis_mnt)
	MNTPUT(lis_mnt);

    if (LIS_DEBUG_VCLOSE || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
        printk("lis_cleanup_file_closing(f@0x%p/%d,h@0x%p/%d/%d)"
	       " [%d]%s%s",
	       f, (f?F_COUNT(f):0),
	       head,
	       (head?LIS_SD_REFCNT(head):0),
	       (head?LIS_SD_OPENCNT(head):0),
	       K_ATOMIC_READ(&lis_mnt_cnt),
	       (D_IS_LIS(f->f_dentry)?" <LiS>":""),
	       (FILE_MNT(f)==lis_mnt?" <lis_mnt>":"")
	      ) ;
	printk(" \"");
	if (FILE_MNT(f))  lis_print_file_path(f);
	printk("\"\n");
	lis_print_dentry(d, ">> dentry") ;
    }
}

#if defined(KERNEL_2_5)
/*
 * lis_strflush - see file_operations
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
/* flushed changed in 75e1fcc0b18df0a65ab113198e9dc0e98999a08c v.2.6.18-rc1 */
int lis_strflush( struct file *f, fl_owner_t id )
#else
int lis_strflush( struct file *f )
#endif
{
    int err = 0;

    if (LIS_DEBUG_VCLOSE || LIS_DEBUG_REFCNTS)
	printk("lis_strflush(f@0x%p)\n", f);

    MODSYNC();

    return err;
}
#endif

/*
 * lis_inode_lookup - must be present for namei on LiS mounted file system
 * to work properly.  Return of NULL should suffice.
 */
#if defined(KERNEL_2_5)

struct dentry *lis_inode_lookup(struct inode *dir, struct dentry *dentry,
				struct nameidata *nd)
{
    return(NULL) ;
}

void lis_drop_inode(struct inode *inode)
{
    generic_delete_inode(inode) ;
}

#else

struct dentry *lis_inode_lookup(struct inode *dir, struct dentry *dentry)
{
    return(NULL) ;
}

#endif

/*
 *  lis_get_filp() - get a LiS-ready file pointer, or set one up for LiS
 */
static struct file *
lis_get_filp( struct file_operations *f_op )
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
#if 0
    struct file *f = alloc_file(, , , f_op);
#endif
    /* FIXME: get_empty_filp was removed, need to replace with alloc_file,
     * but it's not really clear how.. */
    return NULL;
#else
    struct file *f = get_empty_filp();

    if (!f)  return NULL;

    f->f_pos    = 0;
    f->f_op     = fops_get(f_op);	/* bumps module ref count */
    f->f_flags  = O_RDWR;
    f->f_mode   = FMODE_READ|FMODE_WRITE;

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_REFCNTS)
	printk("lis_get_filp(...) >> f@0x%p\n", f);

    return f;
#endif
}

/*
 * lis_set_up_inode() - make an inode look like an LiS inode
 *
 * Used in stream head when a stream file is opened.
 */
struct inode *lis_set_up_inode(struct file *f, struct inode *inode)
{
    struct inode	*new ;

    if (inode == NULL) return(NULL) ;

    new = lis_new_inode(f, GET_I_RDEV(inode)) ;
    if (new == NULL)
	return(NULL) ;

    /*
     * Set the user/group ids to the opener, set modification times
     * to the current time.
     */
    new->i_uid   = FSUID(current);
    new->i_gid   = FSGID(current);
    new->i_atime = new->i_mtime = new->i_ctime = CURRENT_TIME;

    return(new) ;
}

/*
 * lis_is_stream_inode
 */
int lis_is_stream_inode(struct inode *i)
{
    return(I_IS_LIS(i)) ;
}

/*
 *  lis_get_inode() - allocate and setup a LiS-only inode
 */
static struct inode *
lis_get_inode( mode_t mode, dev_t dev )
{
    struct inode *i = lis_get_new_inode(lis_mnt->mnt_sb);

    if (i)
    {
	i->i_mode  = mode;
	/*
	 *  see the comment above (in lis_new_inode) about the use of
	 *  the dev and rdev fields
	 */
	i->i_op    = &lis_streams_iops;
/*
 *  FIXME - generalize to use an fops per major, so modules can own
 *  their open files.
 */
	i->i_fop   = &lis_streams_fops;
	/*
	 *  char devs are identified by the rdev field; dev identifies
	 *  the hosting file system.  Here, we construct our own dev
	 *  field, reflecting that this is a LiS-only inode which has
	 *  no file system hosting it (other than LiS itself)
	 */
#if defined(KERNEL_2_5)
	i->i_rdev  = DEV_TO_RDEV(dev);	/* set desired dev */
#else
	/*
	 * i_rdev will show our minor device number modulo 256
	 */
	i->i_rdev       = MKDEV( getmajor(dev), getminor(dev) );
	{
	    lis_inode_info_t *p = (lis_inode_info_t *) &i->u ;
	    p->dev = dev ;
	}
#endif

	if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    printk("lis_get_inode(m0x%x,dv0x%x) >> i@0x%p/%d"
		   " <[%d] %d LiS inodes>\n",
		   mode, dev,
		   i, I_COUNT(i),
		   K_ATOMIC_READ(&lis_mnt_cnt),
		   K_ATOMIC_READ(&lis_inode_cnt));
    }

    return i;
}

/*
 *  lis_old_inode() - swap the inode referenced by f with that referenced
 *  by i.
 */
struct inode *
lis_old_inode( struct file *f, struct inode *i )
{
    struct dentry *oldd = f->f_dentry;
    struct dentry *newd ;
    struct vfsmount *oldmnt = FILE_MNT(f);

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS)
	printk("lis_old_inode(f@0x%p/%d,i@0x%p/%d)%s << "
	       "i@0x%p/%d%s (dev 0x%x -> 0x%x)\n",
	       f, F_COUNT(f),
	       i, I_COUNT(i),
	       (I_IS_LIS(FILE_INODE(f))?" <LiS>":""),
	       FILE_INODE(f), I_COUNT(FILE_INODE(f)),
	       (I_IS_LIS(i)?" <LiS>":""),
	       GET_I_RDEV(FILE_INODE(f)),
	       GET_I_RDEV(i));

    newd = lis_d_alloc_root(igrab(i), LIS_D_ALLOC_ROOT_NORMAL);
    if (IS_ERR(newd))
    {
       iput(i);
       return NULL;
    }
    f->f_dentry = newd;

    /*
     *  the caller may have created a new file pointer with no dentry
     *  or inode and no f_vfsmnt set.  In that case, we are doing the
     *  initial setting of f_vfsmnt here.
     */
    f->f_vfsmnt = MNTGET(lis_mnt);	/* (re)mount on LiS */
    lis_dput(oldd);
    if (oldmnt && oldmnt != lis_mnt)  /* lis_dput() does mntput() on lis_mnt */
	MNTPUT(oldmnt);

    if (LIS_DEBUG_VOPEN || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
    {
	printk("lis_old_inode(f@0x%p/%d,i@0x%p/%d)%s\n",
	       f, F_COUNT(f),
	       i, I_COUNT(i),
	       (I_IS_LIS(FILE_INODE(f))?" <LiS>":""));
	lis_print_dentry(newd, ">> dentry") ;
    }

    return (FILE_INODE(f));
}

struct inode *lis_grab_inode(struct inode *ino)
{
    return(igrab(ino)) ;
}

void lis_put_inode(struct inode *ino)
{
    iput(ino) ;
}


/*
 *  lis_get_fifo() - create a new unique fifo
 *
 *  note that lis_get_filp() uses lis_streams_fops, which is what makes
 *  mode S_IFIFO a STREAM instead of a kernel FIFO; i.e., it ensures
 *  that stropen will be called to open the FIFO.
 */
int lis_get_fifo( struct file **f )
{
    dev_t	dev = makedevice( LIS_CLONE, LIS_FIFO );
    char	name[48] ;
    int		error;

    MNTSYNC();

    if (!(*f = lis_get_filp(&lis_streams_fops)))
	return(-ENFILE) ;

    sprintf(name, "clone(%d,%d)", LIS_CLONE, LIS_FIFO) ;

    if ((error = lis_new_file_name_dev(*f, name, dev)) == 0)
    {
	if ((error = lis_stropen( (*f)->f_dentry->d_inode, *f )) < 0)
	{
	    fops_put((*f)->f_op);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	    (*f)->f_op->owner = NULL ;
#endif
	    fput(*f) ;
	    *f = NULL ;
	}
    }

    MNTSYNC();

    return(error);
}

/*
 *  lis_get_pipe()
 *
 *  create a new unique pipe as two new unique file pointers and all
 *  lower "plumbing" from them (i.e., inodes, stream heads, queues).
 *
 *  the process is simply to create two new unique FIFOs, and "twists"
 *  them.
 */
int lis_get_pipe( struct file **f0, struct file **f1 )
{
    stdata_t *hd0, *hd1;
    queue_t *wq0, *wq1;
    int error;

    /*
     *  get a pair of unique FIFOs for the pipe ends
     */
    if ((error = lis_get_fifo( f0 )) < 0)  goto no_fifos;
    if ((error = lis_get_fifo( f1 )) < 0)  goto no_fifo1;

    /*
     *  OK - make them peers, and get the head queues for twisting
     */
    hd0 = FILE_STR(*f0);  wq0 = hd0->sd_wq;
    hd1 = FILE_STR(*f1);  wq1 = hd1->sd_wq;
    hd0->sd_peer = hd1;
    hd1->sd_peer = hd0;
    lis_head_get(hd0) ;			/* balanced in lis_qdetach */
    lis_head_get(hd1) ;
    if (LIS_DEBUG_OPEN)
	printk("lis_get_pipe: hd0:%s hd1:%s\n", hd0->sd_name, hd1->sd_name) ;

    /*
     *  twist the write queues to point to the peers' read queues
     */
    wq0->q_next = RD(wq1);
    wq1->q_next = RD(wq0);

    return 0;
    
no_fifo1:
    lis_strclose( FILE_INODE(*f0), *f0 );
no_fifos:
    return error;
}

/*
 *  lis_pipe()
 *
 *  this routine is just an intermediate layer in the pipe system call
 *  process.  It's job is to turn the file pointers returned by lis_get_pipe()
 *  into a pair of file descriptors.
 */
int
lis_pipe( unsigned int *fd )
{
    int fd0, fd1;
    struct file *fp0, *fp1;
    int error;

    if ((fd0 = get_unused_fd()) < 0) {
	error = fd0;  goto no_fds;
    }
    if ((fd1 = get_unused_fd()) < 0) {
	error = fd1;  goto no_fd1;
    }

    if ((error = lis_get_pipe( &fp0, &fp1 )) < 0)  goto no_pipe;

    /*
     *  got everything - hook it up
     */
    fd_install( fd0, fp0 );
    fd_install( fd1, fp1 );
    fd[0] = fd0;
    fd[1] = fd1;

    return 0;
    
no_pipe:
    put_unused_fd(fd1);
no_fd1:
    put_unused_fd(fd0);
no_fds:
    return error;
}

/*
 *  pipe() ioctl wrapper (pretty much the same as the syscall)
 */
int
lis_ioc_pipe( unsigned int *fildes )
{
    unsigned int fd[2];
    int error;
    
    error = lis_pipe(fd);
    if (!error) {
	if (copy_to_user(fildes, fd, 2*sizeof(unsigned int)))
	    error = -EFAULT;
    }
    return error;
}

/*
 *  the following ..._sync() routines are adaptations of the kernel
 *  fifo open routines, used here to implement conventional FIFO
 *  synchronization at open/close time (and elsewhere).  We actually
 *  use the same data structures, so it makes sense to model this
 *  code closely after the kernel's code.  Note that this is only
 *  done for FIFOs and pipes, and only actually needed for O_RDONLY
 *  and O_WRONLY for FIFOs, but it's here for sake of completeness...
 *
 *  Note also that this is safe to do.  LiS now uses inodes which
 *  are totally independent of any host filesystem - we can do whatever
 *  we want with them (as long as the kernel doesn't complain).
 *
 *  the preliminary routines here are support for fifo_open_sync.
 */

static inline void lis_fifo_wait(struct inode * i)
{
	DECLARE_WAITQUEUE(wait, current);
	current->state = TASK_INTERRUPTIBLE;
	add_wait_queue(PIPE_WAIT(*i), &wait);
	lis_kernel_up(PIPE_SEM(*i));
	schedule();
	remove_wait_queue(PIPE_WAIT(*i), &wait);
	current->state = TASK_RUNNING;
	lis_kernel_down(PIPE_SEM(*i));
}

static inline void lis_fifo_wait_for_partner( struct inode* i,
					      unsigned int* cnt)
{
    int cur = *cnt;	
    while (cur == *cnt) {
	lis_fifo_wait(i);
	if (signal_pending(current))
	    break;
    }
}

static inline void lis_fifo_wake_up_partner(struct inode* i)
{
    lis_wake_up_interruptible(PIPE_WAIT(*i));
}

static struct inode* lis_fifo_info_new(struct inode* i)
{
    i->i_pipe = kmalloc(sizeof(struct pipe_inode_info), GFP_KERNEL);
    if (i->i_pipe) {
	memset(i->i_pipe, 0, sizeof(struct pipe_inode_info));
	init_waitqueue_head(PIPE_WAIT(*i));
	PIPE_RCOUNTER(*i) = PIPE_WCOUNTER(*i) = 1;
	return i;
    } else {
	return NULL;
    }
}

int
lis_fifo_open_sync( struct inode *i, struct file *f )
{
    stdata_t *head = INODE_STR(i);
    long this_open = K_ATOMIC_READ(&lis_open_cnt);
    int ret = 0;

    if (!i || !f) {
	printk("fifo_open_sync(i@0x%p/%d,f@0x%p/%d)#%ld - NULL PARM!\n",
	       i, (i?I_COUNT(i):0),
	       f, (f?F_COUNT(f):0),
	       this_open );
	return(-EINVAL);
    }

    if (LIS_DEBUG_VOPEN)
	printk("lis_fifo_open_sync(i@0x%p/%d,f@0x%p/%d)#%ld"
	       " \"%s\" << mode 0%o flags 0%o\n",
	       i, I_COUNT(i), f, F_COUNT(f),
	       this_open, head ? head->sd_name : "No-Strm",
	       (int)f->f_mode, (int)f->f_flags );


    ret = -ERESTARTSYS;
    if (lis_kernel_down(PIPE_SEM(*i)))
	goto err_nolock_nocleanup;
    
    if (!i->i_pipe) {
	ret = -ENOMEM;
	if(!lis_fifo_info_new(i))
	    goto err_nocleanup;
    }
    f->f_version = 0;

    switch (f->f_mode & (FMODE_READ|FMODE_WRITE)) {
    case FMODE_READ:
	/*
	 *  O_RDONLY
	 *  POSIX.1 says that O_NONBLOCK means return with the FIFO
	 *  opened, even when there is no process writing the FIFO.
	 */
	PIPE_RCOUNTER(*i)++;
	if (PIPE_READERS(*i)++ == 0)
	    lis_fifo_wake_up_partner(i);
	
	if (!PIPE_WRITERS(*i)) {
	    if ((f->f_flags & O_NONBLOCK)) {
		/* suppress POLLHUP until we have seen a writer */
		f->f_version = PIPE_WCOUNTER(*i);
	    } else 
	    {
		lis_fifo_wait_for_partner(i, &PIPE_WCOUNTER(*i));
		if (signal_pending(current))
		    goto err_rd;
	    }
	}
	break;
	
    case FMODE_WRITE:
	/*
	 *  O_WRONLY
	 *  POSIX.1 says that O_NONBLOCK means return -1 with
	 *  errno=ENXIO when there is no process reading the FIFO.
	 */
	ret = -ENXIO;
	if ((f->f_flags & O_NONBLOCK) && !PIPE_READERS(*i))
	    goto err;
	
	PIPE_WCOUNTER(*i)++;
	if (!PIPE_WRITERS(*i)++)
	    lis_fifo_wake_up_partner(i);
	
	if (!PIPE_READERS(*i)) {
	    lis_fifo_wait_for_partner(i, &PIPE_RCOUNTER(*i));
	    if (signal_pending(current))
		goto err_wr;
	}
	break;
	
    case FMODE_READ|FMODE_WRITE:
	/*
	 *  O_RDWR
	 *  POSIX.1 leaves this case "undefined" when O_NONBLOCK is set.
	 *  This implementation will NEVER block on a O_RDWR open, since
	 *  the process can at least talk to itself.
	 */
	PIPE_READERS(*i)++;
	PIPE_WRITERS(*i)++;
	PIPE_RCOUNTER(*i)++;
	PIPE_WCOUNTER(*i)++;
	if (PIPE_READERS(*i) == 1 || PIPE_WRITERS(*i) == 1)
	    lis_fifo_wake_up_partner(i);
	break;
	
    default:
	ret = -EINVAL;
	goto err;
    }

    lis_kernel_up(PIPE_SEM(*i));

    if (LIS_DEBUG_VOPEN)
	printk("lis_fifo_open_sync(i@0x%p/%d,f@0x%p/%d)#%ld"
	       " \"%s\" >> %d reader(s) %d writer(s)\n",
	       i, I_COUNT(i), f, F_COUNT(f),
	       this_open, head ? head->sd_name : "No-Strm",
	       PIPE_READERS(*i), PIPE_WRITERS(*i));

    return 0;
    
err_rd:
    if (!--PIPE_READERS(*i))
	lis_wake_up_interruptible(PIPE_WAIT(*i));
    ret = -ERESTARTSYS;
    goto err;
    
err_wr:
    if (!--PIPE_WRITERS(*i))
	lis_wake_up_interruptible(PIPE_WAIT(*i));
    ret = -ERESTARTSYS;
    goto err;
    
err:
    if (!PIPE_READERS(*i) && !PIPE_WRITERS(*i)) {
	kfree(i->i_pipe);
	i->i_pipe = NULL;
    }

err_nocleanup:
    lis_kernel_up(PIPE_SEM(*i));

err_nolock_nocleanup:
    if (LIS_DEBUG_VOPEN)
	printk("lis_fifo_open_sync(i@0x%p/%d,f@0x%p/%d)#%ld \"%s\""
	       " >> error(%d)\n",
	       i, (i?I_COUNT(i):0),
	       f, (f?F_COUNT(f):0),
	       this_open, head->sd_name, ret) ;

    return ret;
}

void
lis_fifo_close_sync( struct inode *i, struct file *f )
{
    stdata_t *head = INODE_STR(i);
    long this_close = K_ATOMIC_READ(&lis_close_cnt);

    lis_kernel_down(PIPE_SEM(*i));

    PIPE_READERS(*i) -= (f && f->f_mode & FMODE_READ ? 1 : 0);
    PIPE_WRITERS(*i) -= (f && f->f_mode & FMODE_WRITE ? 1 : 0);

    if (LIS_DEBUG_VCLOSE)
	printk("lis_fifo_close_sync(i@0x%p/%d,f@0x%p/%d)#%ld"
	       " \"%s\" >> %d reader(s) %d writer(s)\n",
	       i, I_COUNT(i), f, (f?F_COUNT(f):0),
	       this_close,
	       (head&&head->sd_name?head->sd_name:""),
	       PIPE_READERS(*i), PIPE_WRITERS(*i));

    if (!PIPE_READERS(*i) && !PIPE_WRITERS(*i)) {
	kfree(i->i_pipe);
	i->i_pipe = NULL;
    } else {
	lis_wake_up_interruptible(PIPE_WAIT(*i));
    }

    lis_kernel_up(PIPE_SEM(*i));
}

int
lis_fifo_write_sync( struct inode *i, int written )
{
    if (!PIPE_READERS(*i)) {
	if (!written)
	    send_sig( SIGPIPE, current, 0 );
	return(-EPIPE);
    }
    else
	return 0;
}

static void
lis_fifo_sendfd_sync( struct inode *i, struct file *f )
{
    PIPE_READERS(*i) += (f->f_mode & FMODE_READ ? 1 : 0);
    PIPE_WRITERS(*i) += (f->f_mode & FMODE_WRITE ? 1 : 0);
}

/*
 *  fattach() -
 *
 *  "mount" a stream on a non-STREAM path.  Subsequent opens of the
 *  path will open the mounted stream, until the path is fdetach()'ed.
 *
 *  Note Linux semantics:
 *  - only one mount is done, for the given path, not for all of its
 *    aliases (i.e., hard links).  The reason is that those aliases
 *    can't be efficiently loaded as dentries (if they aren't already)
 *    because the Linux FS org doesn't allow easy access to hard-linked
 *    directory entries.  It _is_ possible to link all such names that
 *    are already in the dcache, but those that are not will be missed.
 *    To have predictable semantics, then, we only mount the given path.
 *  - Permissions are kept at the underlying inodes in the Linux FS.
 *    Since a stream may be mounted on different paths (e.g., to get
 *    around the above limitation, if the user knows the aliases and
 *    calls fattach separately for each), it is not deemed reasonable
 *    to have this routine "initialize" the streams permissions & ids
 *    to those of the underlying file (since there may be many).
 *    Instead (and more simply), we just check here that the caller has
 *    read/write access to the path.
 *  - We allow mounting to any non-directory.  This allows mounting
 *    not only over regular files and streams, but also over Linux FIFOs
 *    and pipes.
 *
 *  2002/11/18 - FIXME - are the above comments still OK?  -JB
 */
int
lis_fattach( struct file *f, const char *path )
{
#if defined(FATTACH_VIA_MOUNT)
    stdata_t *head = FILE_STR(f);
    int result;

    MNTSYNC();

    if (head && head->magic == STDATA_MAGIC) {
	lis_fattach_t *data = lis_fattach_new(f, path);
	
	if (!data)
	    return(-ENOMEM);

	if (LIS_DEBUG_FATTACH)
	    printk("lis_fattach(f@0x%p/%d,\"%s\") << data@0x%p\n",
		   f, F_COUNT(f), path, data );
	
	/*
	 *  We use the sys_mount() syscall now to do an fattach, with
	 *  help from fs_fattach_sb() in the LiS filesystem structure.
	 *  If it finishes its work OK and sys_mount() doesn't complain,
	 *  we will then add the fattach instance to the global list.
	 *
	 *  Note that we pass a reference to data!!!  This ensures that
	 *  its use is effectively "call by reference", since we need
	 *  the stuff under the mount() syscall to set values in data
	 *  that we will use later, and the syscall wrapping stuff would
	 *  otherwise mess things up.
	 *  (This is OK to do, since we're in kernel space already.)
	 */
	result = lis_mount( NULL, (char *)path, LIS_FS_NAME, 0, &data );
	if (result < 0) {
	    putname(data->path);
	    FREE(data);
	} else {
	    lis_fattach_insert(data);  /* OK! - add to fattaches list */

	    if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS || LIS_DEBUG_REFCNTS)
	    {
		printk("lis_fattach(...) data @ 0x%p:\n"
		       "    >> f@0x%p/%d h@0x%p \"%s\" s@0x%p\n",
		       data,
		       data->file, F_COUNT(data->file),
		       data->head, data->path,
		       data->sb) ;
		lis_print_dentry(data->dentry, ">> dentry") ;
	    }
	}
    } else
	result = -EINVAL;  /* f must be a STREAM */

    MNTSYNC();

    return(result);
#else
    static int	twice ;

    if (++twice <= 2)
	printk("\nfattach is no longer implemented in kernels older "
	    "than 2.4.7\n\n") ;
    return(-ENOSYS) ;
#endif			/* FATTACH_VIA_MOUNT */
}

/*
 *  fattach() ioctl wrapper
 *
 *  this is very similar to the syscall wrapper; it differs in the
 *  first parameter (a file * instead of an fd).  It's being used
 *  because lis_check_umem() and lis_copyin_str() don't seem to be
 *  reliable.
 *
 */
int
lis_ioc_fattach( struct file *f, char *path )
{
    return lis_fattach( f, path );
}

/*
 *  fdetach() -
 *
 *  undo an fattach()'s mount.  If the mount reference for the mounted
 *  stream is the last, close the stream.
 *
 *  This routine is written not for the conventional case, where an
 *  fattach mounts a stream on all aliases of a pathname, but to match
 *  our fattach(), which mounts only on the given pathname.  I.e.,
 *  fattach's to links to the pathname, and fattach's to links on
 *  different streams, are not disturbed.
 */
int
lis_fdetach( const char *path )
{
    if (LIS_DEBUG_FATTACH)
	printk("lis_fdetach(\"%s\")\n", path );

#if defined(FATTACH_VIA_MOUNT)
    /*
     *  We now use the sys_umount syscall to do all the work, with the
     *  help of callback entry points in the LiS superblock structure.
     *  If things go OK, lis_super_umount_begin() will undo an fattach
     *  and remove its instance structure from the list; we don't have
     *  to do anything else here.
     *
     *  We use the MNT_DETACH flag to force unmounting if busy; we
     *  don't care about the mount notion of "busy", and we will in
     *  fact want to umount streams that look "busy" to the OS.
     */
#if defined(MNT_DETACH)
    return lis_umount2( (char *)path, MNT_FORCE|MNT_DETACH );
#else
    return lis_umount2( (char *)path, MNT_FORCE);
#endif
# else
    return(-ENOSYS) ;
# endif			/* FATTACH_VIA_MOUNT */
}

/*
 *  lis_fdetach_stream() 
 *
 *  undo any mount(s) (via fattach) on the stream.  The head may be
 *  disposed of in the process, as may a peer stream head if this is
 *  a pipe.
 *
 *  this routine is provided as a means of cleaning up undone fattach
 *  calls.  It should be called when the peer of an fattach'ed pipe end
 *  disappears, but it might also be used before unloading LiS.
 */
void
lis_fdetach_stream( stdata_t *head )
{
#if defined(FATTACH_VIA_MOUNT)
    int n = num_fattaches_listed;  /* just to make sure we terminate */
    int error;

    if (!K_ATOMIC_READ(&head->sd_fattachcnt))
	return;

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_REFCNTS)
	printk("lis_fdetach_stream(h@0x%p/%d/%d): %d fattaches active...\n",
	       head,
	       (head?LIS_SD_REFCNT(head):0),
	       (head?LIS_SD_OPENCNT(head):0),
	       K_ATOMIC_READ(&head->sd_fattachcnt));

    lis_spin_lock(&lis_fattaches_lock);
    while (n-- && !list_empty(&lis_fattaches)) {
	lis_fattach_t *data =
	    list_entry( lis_fattaches.next, lis_fattach_t, list );
	
	if (data->head == head) {
	    lis_spin_unlock(&lis_fattaches_lock);

	    if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS)
	    {
		printk("    fdetaching: "
		       "data 0x%p head@0x%p \"%s\" sb 0x%p\n",
		       data, data->head, data->path, data->sb);
		lis_print_dentry(data->dentry, ">> dentry") ;
	    }

	    if ((error = lis_fdetach(data->path)) < 0) {
		if (LIS_DEBUG_FATTACH)
		    printk("   fdetach 0x%p failed (%d)...\n",
			   data, error);
	    }

	    if (!K_ATOMIC_READ(&head->sd_fattachcnt))
		return;

	    lis_spin_lock(&lis_fattaches_lock);
	}
    }
    lis_spin_unlock(&lis_fattaches_lock);
#endif 	/* FATTACH_VIA_MOUNT */
}

/*
 *  lis_fdetach_all() -
 *
 *  a wrapper for lis_detach(), to abolish all fattach's (or
 *  at least, those for which the caller has the appropriate privilege).
 *
 *  fattaches can be stacked, so we remove the newest ones first.
 */
void
lis_fdetach_all(void)
{
#if defined(FATTACH_VIA_MOUNT)
    int n = num_fattaches_listed;  /* just to make sure we terminate */
    int error;

    if (LIS_DEBUG_FATTACH)
	printk("lis_fdetach_all() << %d fattach(s) active\n",
	       K_ATOMIC_READ(&num_fattaches_listed));

    lis_spin_lock(&lis_fattaches_lock);
    while (n-- && !list_empty(&lis_fattaches)) {
	lis_fattach_t *data =
	    list_entry( lis_fattaches.next, lis_fattach_t, list );
	lis_spin_unlock(&lis_fattaches_lock);

	if (LIS_DEBUG_FATTACH || LIS_DEBUG_ADDRS)
	    printk("    >> fdetaching data 0x%p head 0x%p sb 0x%p d 0x%p...\n",
		   data, data->head, data->sb, data->dentry);

	if ((error = lis_fdetach(data->path)) < 0) {
	    if (LIS_DEBUG_FATTACH)
		printk("    >> fdetach data 0x%p failed (%d)\n",
		       data, error);
	} else
	    if (LIS_DEBUG_FATTACH)
		printk("    fdetached 0x%p OK\n", data);

	lis_spin_lock(&lis_fattaches_lock);
    }
    lis_spin_unlock(&lis_fattaches_lock);

#endif  /* FATTACH_VIA_MOUNT */
}

/*
 *  fdetach() ioctl wrapper
 */
int
lis_ioc_fdetach( char *path )
{
    char *tmp = getname(path);
    int error = PTR_ERR(tmp);

    if (tmp && !IS_ERR(tmp)) {
	if (strcmp( tmp, "*" ) == 0)
	    lis_fdetach_all();
	else
	    error = lis_fdetach(path);  /* arg must be in user space */
	putname(tmp);
    }

    return error;
}

#if defined(FATTACH_VIA_MOUNT)
/*
 *  fattach()/fdetach() instance data support routines
 */

/* allocate a new fattach instance */
static lis_fattach_t *lis_fattach_new(struct file *f, const char *pathname)
{
    lis_fattach_t *data = (lis_fattach_t *) ZALLOC(sizeof(lis_fattach_t));
    char *tmp = (data ? getname(pathname) : NULL);
    struct nameidata nd;
    int error = 0;

    if (data && tmp && !IS_ERR(tmp)) {
	data->file = f;
	data->head = FILE_STR(f);

	/*
	 *  pathname may be relative (to current pwd) -
	 *  convert it to absolute before saving it, since fdetach may
	 *  happen from a different process and thus different pwd
	 *
	 *  Note that the pathname likely lengthens in this process.
	 *  d_path return -ENAMETOOLONG in that case, but it's unlikely
	 *  to happen if PATH_MAX is long, so the handling here (though
	 *  not efficient) is for sake of completeness.  Note we're also
	 *  assuming that PATH_MAX is the size of the buffer getname()
	 *  allocates.
	 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
	{
	struct path path;
	error = user_path(pathname, &path);
	nd.path = path;
	}
#else
	error = user_path_walk(pathname, &nd);
#endif
	if (!error) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
	    data->path = d_path(&nd.path, tmp, PATH_MAX);
	    path_put(&nd.path);
#else
	    data->path = d_path(nd.dentry, nd.mnt, tmp, PATH_MAX);
	    path_release(&nd);
#endif

	    if (IS_ERR(data->path)) {
		/*
		 *  too long? do the getname again, since failing d_path
		 *  will likely have clobbered it.  A relative path is
		 *  better than no path at all.
		 */
		putname(tmp);
		data->path = tmp = getname(pathname);
	    }
	} else
	    data->path = tmp;  /* better than nothing */

	K_ATOMIC_INC(&num_fattaches_allocd);
    }
    else if (data) {
	FREE(data);  data = NULL;
    }

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_REFCNTS) {
	    printk("lis_fattach_new(f@0x%p/%d,\"%s\")"
		   " => data@0x%p (%d/%d)\n",
		   f, F_COUNT(f), pathname, data,
		   K_ATOMIC_READ(&num_fattaches_listed),
		   K_ATOMIC_READ(&num_fattaches_allocd) );
    }

    return data;
}

/* delete an fattach instance - assumed not in list */
static void lis_fattach_delete(lis_fattach_t *data)
{
    K_ATOMIC_DEC(&num_fattaches_allocd);

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_REFCNTS) {
	    printk("lis_fattach_delete(%p) (%d/%d)\n",
		   data,
		   K_ATOMIC_READ(&num_fattaches_listed),
		   K_ATOMIC_READ(&num_fattaches_allocd) );
    }

    if (data && data->path)
	putname(data->path);

    if (data)
	FREE(data);
}

/* insert an instance into the list, at the front */
static void lis_fattach_insert(lis_fattach_t *data)
{
    lis_spin_lock(&lis_fattaches_lock);
    list_add(&data->list, &lis_fattaches);
    lis_spin_unlock(&lis_fattaches_lock);

    K_ATOMIC_INC(&num_fattaches_listed);

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_REFCNTS) {
	    printk("lis_fattach_insert(%p) (%d/%d)\n",
		   data,
		   K_ATOMIC_READ(&num_fattaches_listed),
		   K_ATOMIC_READ(&num_fattaches_allocd) );
    }
}

/* remove an fattach instance from the list */
static void lis_fattach_remove(lis_fattach_t *data)
{
    lis_spin_lock(&lis_fattaches_lock);
    list_del(&data->list);
    lis_spin_unlock(&lis_fattaches_lock);

    K_ATOMIC_DEC(&num_fattaches_listed);

    if (LIS_DEBUG_FATTACH || LIS_DEBUG_REFCNTS) {
	    printk("lis_fattach_remove(%p) (%d/%d)\n",
		   data,
		   K_ATOMIC_READ(&num_fattaches_listed),
		   K_ATOMIC_READ(&num_fattaches_allocd) );
    }
}

#endif /* FATTACH_VIA_MOUNT */

/*
 *  lis_sendfd() -
 *
 *  we allow either a file pointer or a file descriptor as input here,
 *  in order that "internal" uses may be supported (e.g., connld).  In
 *  fact, the file pointer supercedes the file descriptor, if both are
 *  provided.
 *
 *  in the case that an fp is given, it is used with the assumption
 *  the the reference count has been bumped appropriately.
 */
int lis_sendfd( stdata_t *sendhd, unsigned int fd, struct file *fp )
{
    stdata_t *recvhd = NULL;
    mblk_t *mp;
    strrecvfd_t *sendfd;
    struct file *oldfp;
    int error;
    lis_flags_t  psw;

    error = -EPIPE;
    if (!sendhd ||
	!(sendhd->magic == STDATA_MAGIC) ||
	!(recvhd = sendhd->sd_peer) ||
	!(recvhd->magic == STDATA_MAGIC) ||
	recvhd == sendhd ||
	F_ISSET(sendhd->sd_flag, STRHUP))
	goto not_fifo;

    error = -ENOSR;
    if (!(mp = allocb( sizeof(strrecvfd_t), BPRI_HI )))  goto no_msg;

    /*
     *  if fp is set, we assume it's reference count is set so that
     *  we won't lose it.  Otherwise, we must get the fp corresponding
     *  to fd.
     */
    if (!fp) {
	/*
	 *  get the file pointer corresponding to fd in the current (i.e.,
	 *  the sender's) process.  We do fget() here to hold the returned
	 *  fp for us.
	 */
	error = -EBADF;
	if (!(fp = fget(fd)))  goto bad_file;
	oldfp = fp;
    } else
	oldfp = fp;

    /*
     *  there's one case where we don't want the fp's count bumped;
     *  if the fp is for the receiving file itself, it can't be
     *  closed if the count is high and the FD isn't received.  But
     *  we can't tell for sure if this will happen, so to avoid it,
     *  we make a copy of the file pointer and pass it instead.  We
     *  also pass the original file pointer as r.fp, so it can be
     *  used if it is the receiving file.
     */
    if (FILE_INODE(fp) == recvhd->sd_inode)  {
	struct file *dfp = lis_get_filp(&lis_streams_fops);
	struct inode *i = FILE_INODE(fp);

	LOCK_INO(i);
	dfp->f_pos   = fp->f_pos;
	dfp->f_flags = fp->f_flags;
	dfp->f_mode  = fp->f_mode;
	error = -ENOMEM;
	if (!(dfp->f_dentry = lis_d_alloc_root(igrab(i),
					       LIS_D_ALLOC_ROOT_NORMAL))) {
	    ULOCK_INO(i);
	    fops_put(dfp->f_op);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	    dfp->f_op->owner = NULL ;
#endif
	    fput(dfp);
	    fput(fp);
	    goto bad_file;
	}
	if (FILE_MNT(fp))
	    dfp->f_vfsmnt = MNTGET(FILE_MNT(fp));
	if (F_ISSET(recvhd->sd_flag,STFIFO))
	    lis_fifo_sendfd_sync( i, dfp );
	SET_FILE_STR(dfp, FILE_STR(fp));

	ULOCK_INO(i);
	
	fput(fp);
	fp = dfp;
    }
    
    /*
     *  OK - set up an M_PASSFP message containing a strrecvfd and
     *  put it in the peer's stream head read queue.
     */
    mp->b_datap->db_type = M_PASSFP;
    sendfd = (strrecvfd_t *) mp->b_rptr;
    sendfd->f.fp  = fp;
    sendfd->uid   = EUID(current);
    sendfd->gid   = EGID(current);
    sendfd->r.fp  = (FILE_INODE(fp) == recvhd->sd_inode ? oldfp : NULL);
    sendfd->r.hd  = recvhd;
    mp->b_wptr = mp->b_rptr + sizeof(strrecvfd_t);
    lis_spin_lock_irqsave(&recvhd->sd_lock, &psw) ;/* lock rcving strm head */
    (recvhd->sd_rfdcnt)++;
    lis_spin_unlock_irqrestore(&recvhd->sd_lock, &psw) ;

    if (LIS_DEBUG_SNDFD || LIS_DEBUG_IOCTL || LIS_DEBUG_REFCNTS) {
	printk("lis_sendfd(...,%d,f@0x%p/%d) from \"%s\" to \"%s\"",
	       fd,
	       oldfp, F_COUNT(oldfp),
	       sendhd->sd_name, recvhd->sd_name);
	if (sendfd->r.fp)
	    printk(" as f@0x%p/%d\n", fp, F_COUNT(fp));
	else
	    printk("\n");
    }

    /*
     *  the following wakes up a receiver if needed
     */
    if (!(error = lis_lockq(recvhd->sd_rq)))
    {
	lis_strrput( recvhd->sd_rq, mp );
	lis_unlockq(recvhd->sd_rq) ;
	return(0) ;
    }
    /* else discard the message and return the error code */

bad_file:
    freemsg(mp);
no_msg:
not_fifo:
    return error;
}

mblk_t *lis_get_passfp(void)
{
    mblk_t *mp;
    lis_spin_lock(&free_passfp.lock);
    mp = free_passfp.head;
    if(free_passfp.head)
	free_passfp.head =  free_passfp.head->b_next;
    if(free_passfp.tail == mp)
	free_passfp.tail = NULL;
    lis_spin_unlock(&free_passfp.lock) ;
    if(mp)
	mp->b_next = NULL;	
    return mp;	
}

/*
 *  the following will be called from flushq() if an M_PASSFP message
 *  is encountered.  This effectively closes the passed file, then
 *  frees the message.
 */
#if defined(KERNEL_2_5)
void lis_tq_free_passfp( unsigned long arg )
#else
void lis_tq_free_passfp( void *arg )
#endif
{
    mblk_t *mp;
    strrecvfd_t *sent;
    int is_a_stream;

    while ((mp = lis_get_passfp()) != NULL)
    {	
	sent = (strrecvfd_t *) mp->b_rptr;

/* FIXME - need a test not dependent on lis_streams_fops */
	is_a_stream = (sent->f.fp->f_op == (&lis_streams_fops));
/**/

	if (LIS_DEBUG_SNDFD || LIS_DEBUG_VCLOSE || LIS_DEBUG_REFCNTS) {
	    struct file *f = sent->f.fp;

	    printk("lis_tq_free_passfp(m@0x%p)"
		   " %s unreceived %sfile @0x%p/%d\n",
		   mp,
		   (sent->r.fp ? "freeing" : "closing"),
		   (is_a_stream ? "STREAMS " : ""),
		   f, (f?F_COUNT(f):0));
	    if (f) {
		struct dentry *d   = f->f_dentry;
		struct inode *i    = FILE_INODE(f);
		struct vfsmount *m = FILE_MNT(f);
		printk("    << d@0x%p/%d%s i@0x%p/%d%s",
		       d, (d?D_COUNT(d):0),
		       (d&&D_IS_LIS(d)?" <LiS>":""),
		       i, (i?I_COUNT(i):0),
		       (i&&I_IS_LIS(i)?" <LiS>":""));
		printk(" m@0x%p/%d", m, (m?MNT_COUNT(m):0));
		printk("\n");
	    }
	}
	if (is_a_stream && sent->r.fp) {
	    fops_put(sent->f.fp->f_op);
	    sent->f.fp->f_op = NULL;  /* (FIXME?) don't call strclose... */
	}
	fput(sent->f.fp);
    	lis_freemsg(mp);
    }	
}

/*
 *  The following will be called from flushq() if an M_PASSFP message
 *  is encountered.  This code puts the message on a queue and defers the 
 *  freeing of the message until later. This is to prevent recursive calls to 
 *  close and bottleneck any other thread. The messages are actually freed from
 *  the function lis_tq_free_passfp.
 */
void lis_free_passfp( mblk_t *mp )
{
#if defined(KERNEL_2_5)
    static DECLARE_TASKLET(lis_tq, lis_tq_free_passfp,0);
#else
    static struct tq_struct	 lis_tq ;
#endif
    int				 emptyq ;
    lis_flags_t 	         psw;
    strrecvfd_t			*sent;
    stdata_t			*recvhd;

    /*
     *  check the message and its contents for validity.  
    */
    if (mp->b_datap->db_type != M_PASSFP)  goto not_passfp;
    if (mp->b_wptr - mp->b_rptr < sizeof(strrecvfd_t))  goto not_passfp;

    sent = (strrecvfd_t *) mp->b_rptr;
    recvhd = sent->r.hd;
    lis_spin_lock_irqsave(&recvhd->sd_lock, &psw) ;/* lock rcving strm head */
    (recvhd->sd_rfdcnt)--;
    lis_spin_unlock_irqrestore(&recvhd->sd_lock, &psw) ;

    lis_spin_lock(&free_passfp.lock);
    mp->b_next = NULL;	
    if ((emptyq = free_passfp.head == NULL))
	free_passfp.head = mp;
    else	
	free_passfp.tail->b_next =  mp;
    free_passfp.tail = mp;
    lis_spin_unlock(&free_passfp.lock) ;

    if (emptyq)
    {
#if defined(KERNEL_2_5)
	tasklet_schedule(&lis_tq) ;
#else
	lis_tq.routine = lis_tq_free_passfp;	/* 2.4 kernel, do it later */
	schedule_task(&lis_tq);
#endif
    }
    return;

not_passfp:
    lis_freemsg(mp);
}

int lis_recvfd( stdata_t *recvhd, strrecvfd_t *recv, struct file *fp )
{
    mblk_t *mp;
    strrecvfd_t *sent;
    int error;
    lis_flags_t  psw;

    lis_bzero( recv, sizeof(strrecvfd_t) );

    error = -EBADF;
    if (!recvhd || !(recvhd->magic == STDATA_MAGIC))
	goto not_stream;

    /*
     *  we expect the caller to have sync'ed with the sender;
     *  we just fail if no message is waiting
     */

    error = -EAGAIN;
    if (!(mp = getq(recvhd->sd_rq)))
	goto no_msg;

    /*
     *  check the message and its contents for validity.
     */
    if (mp->b_datap->db_type != M_PASSFP)
	goto not_passfp;
    lis_spin_lock_irqsave(&recvhd->sd_lock, &psw) ;/* lock rcving strm head */
    (recvhd->sd_rfdcnt)--;
    lis_spin_unlock_irqrestore(&recvhd->sd_lock, &psw) ;
    if (mp->b_wptr - mp->b_rptr < sizeof(strrecvfd_t))
	goto not_passfp;

    error = -ENFILE;
    if ((recv->f.fd = get_unused_fd()) < 0)
	goto no_fds;

    /*
     *  it's a passed FP - hook up the file that was passed to the new FD
     */
    sent = (strrecvfd_t *) mp->b_rptr;
    recv->uid = sent->uid;
    recv->gid = sent->gid;

    if (sent->r.fp && (sent->r.fp == fp)) {
	if (LIS_DEBUG_SNDFD | LIS_DEBUG_IOCTL || LIS_DEBUG_REFCNTS)
	    printk("lis_recvfd(...,f@0x%p/%d) "
		   "S==R, using fp@0x%p/%d, freeing f@0x%p\n",
		   sent->f.fp, (sent->f.fp?F_COUNT(sent->f.fp):0),
		   fp, (fp?F_COUNT(fp):0),
		   sent->f.fp );

	fd_install( recv->f.fd, fp );
	(void) fget(recv->f.fd);
	fops_put(sent->f.fp->f_op);
	sent->f.fp->f_op = NULL;  /* avoid calling strclose */
	if (F_ISSET(recvhd->sd_flag,STFIFO))
	    lis_fifo_close_sync( FILE_INODE(fp), fp );
	fput(sent->f.fp);
#if 0
	if (LIS_DEBUG_IOCTL)
#endif
	    sent->f.fp = fp;
    } else {
	fd_install( recv->f.fd, sent->f.fp );
    }

    if (LIS_DEBUG_SNDFD || LIS_DEBUG_IOCTL || LIS_DEBUG_REFCNTS) {
	printk("lis_recvfd(...,f@0x%p/%d) as fd %d at \"%s\"\n",
	       sent->f.fp, (sent->f.fp?F_COUNT(sent->f.fp):0),
	       recv->f.fd, recvhd->sd_name);
    }

    freemsg(mp);  /* we can release the sent message now */

    return 0;     /* OK */

no_fds:
not_passfp:
    putbqf( recvhd->sd_rq, mp );   /* put the message back */
no_msg:
not_stream:
    return error;
}

/************************************************************************
*                         Atomic Routines                               *
************************************************************************/

void    _RP lis_atomic_set(lis_atomic_t *atomic_addr, int valu)
{
    atomic_set((atomic_t *)atomic_addr, valu) ;
}

int     _RP lis_atomic_read(lis_atomic_t *atomic_addr)
{
    return(atomic_read(((atomic_t *)atomic_addr))) ;
}

void    _RP lis_atomic_add(lis_atomic_t *atomic_addr, int amt)
{
    atomic_add((amt),((atomic_t *)atomic_addr)) ;
}

void    _RP lis_atomic_sub(lis_atomic_t *atomic_addr, int amt)
{
    atomic_sub((amt),((atomic_t *)atomic_addr)) ;
}

void    _RP lis_atomic_inc(lis_atomic_t *atomic_addr)
{
    atomic_inc(((atomic_t *)atomic_addr)) ;
}

void    _RP lis_atomic_dec(lis_atomic_t *atomic_addr)
{
    atomic_dec(((atomic_t *)atomic_addr)) ;
}

int     _RP lis_atomic_dec_and_test(lis_atomic_t *atomic_addr)
{
    return(atomic_dec_and_test(((atomic_t *)atomic_addr))) ;
}

/************************************************************************
*                       lis_in_interrupt                                *
*************************************************************************
*									*
* Returns true if the kernel is at interrupt level or holding spin	*
* locks (2.6).  Basically, if this returns true then you need to make	*
* memory allocator calls using the "atomic" rather then "kernel" forms	*
* of the routines.							*
*									*
************************************************************************/
int      _RP lis_in_interrupt(void)
{
#if defined(KERNEL_2_5)
    return(in_atomic() || irqs_disabled()) ;
#else
    return(in_interrupt()) ;
#endif
}

/************************************************************************
*                         Kernel Semaphores                             *
*************************************************************************
*									*
* These routines are used with doing a down/up on a kernel semaphore.	*
* lis_down/up are used for LiS type semaphores.  Kernel semaphores 	*
* occur in kernel structures, such as inodes.				*
*									*
************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
void _RP lis_kernel_up(struct mutex *sem)
{
	mutex_unlock(sem);
}
int _RP lis_kernel_down(struct mutex *sem)
{
	return(mutex_lock_interruptible(sem));
}
#else
void _RP lis_kernel_up(struct semaphore *sem)
{
    up(sem) ;
}
int _RP lis_kernel_down(struct semaphore *sem)
{
    return(down_interruptible(sem)) ;
}

#endif

/************************************************************************
*                         User Space Access                             *
*************************************************************************
*									*
* copy to/from user space.						*
*									*
************************************************************************/

int	lis_copyin(struct file *fp, void *kbuf, const void *ubuf, int len)
{
    return(copy_from_user(kbuf,ubuf,len)?-EFAULT:0) ;

}

int	lis_copyout(struct file *fp, const void *kbuf, void *ubuf, int len)
{
    return(copy_to_user  (ubuf,kbuf,len)?-EFAULT:0) ;
}

int lis_check_umem(struct file *fp, int rd_wr_fcn,
		   const void *usr_addr, int len)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
    return(access_ok(rd_wr_fcn, usr_addr, len) ? 0 : -EFAULT);
#else
    return(verify_area(rd_wr_fcn, usr_addr, len)) ;
#endif
}

/************************************************************************
*                         lis_gettimeofday                              *
*************************************************************************
*									*
* A slightly slower version of the kernel routine.			*
*									*
************************************************************************/
void _RP lis_gettimeofday(struct timeval *tv)
{
    return(do_gettimeofday(tv));
}



/*  -------------------------------------------------------------------  */
/*				    Module                               */
#ifdef LINUX


/*
 *  lis_loadable_load - load a loadable module.
 *
 *  This function may sleep. On failure it may return a negative errno, or
 *  it may return 0. The module should have been loaded on return from this
 *  function, and the caller should check for this.
 */
int lis_loadable_load(const char *name)
{
#ifdef LIS_LOADABLE_SUPPORT
	return request_module(name);
#else
	printk("lis_loadable_load: %s: "
		"kernel not compiled for dynamic module loading\n",
		name) ;
	return(-ENOSYS) ;
#endif
}


#if defined _S390X_LIS_ || defined (_X86_64_LIS_) || defined (_MIPS64_LIS_)
#if ! defined HAVE_COMPAT_IOCTL
int lis_ioctl32_str (unsigned int fd, unsigned int cmd, unsigned long arg, struct file *f)
{
	strioctl_t par64;
	strioctl32_t par32;
	strioctl32_t * ptr32;
	mm_segment_t old_fs;
	int rc;

	ptr32 = (strioctl32_t*)arg;
	if (copy_from_user((void*)&par32,(void*)ptr32,sizeof(strioctl32_t)))
		{
		printk(
	"Unable to get parameter block for 32 bit ioctl I_STR (length %d)\n",
		   (int)sizeof(strioctl32_t));
		return(-EFAULT);
		}

	par64.ic_cmd    = par32.ic_cmd;
	par64.ic_timout = par32.ic_timout;
	par64.ic_len    = par32.ic_len;
	par64.ic_dp     = (char*)lis_compat_ptr(par32.ic_dp);

#ifdef DANDEBUG
	printk("ioctl32_str after copyin: par64=0x%p arg=0x%lx\n",
		&par64, arg);
	printk("ic_cmd=0x%x timout=0x%x len=%d dp=0x%p\n",
		par64.ic_cmd, 
		par64.ic_timout, 
		par64.ic_len, 
		par64.ic_dp);
#endif
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = sys_ioctl(fd, I_STR, (unsigned long)&par64);
	set_fs(old_fs);

	par32.ic_cmd    = par64.ic_cmd;
	par32.ic_timout = par64.ic_timout;
	par32.ic_len    = par64.ic_len;
	par32.ic_dp     = (unsigned int)(long)(par64.ic_dp);

#ifdef DANDEBUG
	printk("ioctl32_str about to copyout: par32=0x%p arg=0x%lx\n",
		&par32, arg);
	printk("ic_cmd=0x%x timout=0x%x len=%d dp=0x%x\n",
		par32.ic_cmd, 
		par32.ic_timout, 
		par32.ic_len, 
		par32.ic_dp);
#endif
	if (copy_to_user((void*)arg,(void*)&par32,sizeof(strioctl32_t)))
		{
		printk(
	"Unable to return command parameter for 32 bit ioctl I_STR\n");
		return(-EFAULT);
		}

	return(rc);
}

int lis_ioctl32_peek (unsigned int fd, unsigned int cmd, unsigned long arg, struct file *f)
{
	strpeek_t pb;
	strpeek32_t strpeek32;
	int strpeeklen;
	int err;
	int rc;
	mm_segment_t old_fs;

	strpeeklen = sizeof(strpeek32_t);

	if ((err=lis_check_umem(NULL,VERIFY_READ, (char*)arg, strpeeklen))<0 ||
	    (err=lis_check_umem(NULL,VERIFY_WRITE, (char*)arg, strpeeklen))<0)
		return(err);

	if ((err=lis_copyin(NULL, &strpeek32, (char*)arg, 
		            sizeof(strpeek32_t))) < 0)
		return(err);

	pb.ctlbuf.maxlen = strpeek32.ctlbuf.maxlen;
	pb.ctlbuf.len = strpeek32.ctlbuf.len;
	pb.ctlbuf.buf = (char*) lis_compat_ptr(strpeek32.ctlbuf.buf);
	pb.databuf.maxlen = strpeek32.databuf.maxlen;
	pb.databuf.len = strpeek32.databuf.len;
	pb.databuf.buf = (char*) lis_compat_ptr(strpeek32.databuf.buf);
	pb.flags = strpeek32.flags;

#ifdef DANDEBUG
	printk("ioctl32_peek after copyin: strpeek32=0x%p pb=0x%p\n",
		&strpeek32, &pb);
	printk("ctlbuf: maxlen=%d len=%d buf=%p\n",
		pb.ctlbuf.maxlen,
		pb.ctlbuf.len,
		pb.ctlbuf.buf);
	printk("databuf: maxlen=%d len=%d buf=%p.  flags=0x%x\n",
		pb.databuf.maxlen,
		pb.databuf.len,
		pb.databuf.buf, pb.flags);
#endif
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	rc = sys_ioctl(fd, I_PEEK, (unsigned long)&pb);
	set_fs(old_fs);

	strpeek32.ctlbuf.maxlen = pb.ctlbuf.maxlen;
	strpeek32.ctlbuf.len = pb.ctlbuf.len;
	strpeek32.ctlbuf.buf = (unsigned int)(long) (pb.ctlbuf.buf);
	strpeek32.databuf.maxlen = pb.databuf.maxlen;
	strpeek32.databuf.len = pb.databuf.len;
	strpeek32.databuf.buf = (unsigned int)(long) (pb.databuf.buf);
	strpeek32.flags = pb.flags;

#ifdef DANDEBUG
	printk("ioctl32_peek about to copyout: strpeek32=0x%p pb=0x%p\n",
		&strpeek32, &pb);
	printk("ctlbuf: maxlen=%d len=%d buf=%x\n",
		strpeek32.ctlbuf.maxlen,
		strpeek32.ctlbuf.len,
		strpeek32.ctlbuf.buf);
	printk("databuf: maxlen=%d len=%d buf=%x.  flags=0x%x\n",
		strpeek32.databuf.maxlen,
		strpeek32.databuf.len,
		strpeek32.databuf.buf, strpeek32.flags);
#endif

	lis_copyout(NULL, &strpeek32,(char*)arg, sizeof(strpeek32_t));

	return(rc);
}

int lis_ioctl32_compat (unsigned int fd, unsigned int cmd, unsigned long arg, struct file *f)
{
	int rc;
	mm_segment_t old_fs;
	unsigned int newcmd;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	newcmd = cmd & 0xff;
	newcmd |= __SID;
	rc = sys_ioctl(fd, newcmd, arg);
	set_fs(old_fs);
	return(rc);
}
#endif
#endif

int lis_init_module( void )
{
    extern char	*lis_poll_file ;
    extern void  lis_mem_init(void) ;

    printk(
	"==================================================================\n"
	"Linux STREAMS Subsystem loading...\n");

    lis_mem_init() ;			/* in lismem.c */
    lis_major = register_chrdev(0,"streams",&lis_streams_fops);
    if	(lis_major < 0)
    {
	printk("Unable to register STREAMS Subsystem\n");
	return -EIO;
    }
    /* Initialize every global variable to a default value, if there're
     * modules w/ not-exported globals we should create init_() functions for
     * them and call them from here. */

    lis_spl_init() ;
    lis_spin_lock_init(&lis_setqsched_lock, "SetQsched-Lock") ;
    lis_spin_lock_init(&lis_task_lock, "Task-Lock") ;
    lis_spin_lock_init(&free_passfp.lock,"Free Passfp Lock");
#if defined(FATTACH_VIA_MOUNT)
    lis_spin_lock_init(&lis_fattaches_lock, "fattach list lock");
#endif
    lis_init_head();

    {
	int	 err ;

	err = register_filesystem(&lis_file_system_ops) ;
	if (err == 0)
	{
	    lis_mnt = kern_mount(&lis_file_system_ops) ;
	    err = PTR_ERR(lis_mnt) ;
	    if (IS_ERR(lis_mnt))
		unregister_filesystem(&lis_file_system_ops) ;
	    else {
#if defined(MODULE)
		lis_file_system_ops.owner = THIS_MODULE;
#endif
		lis_mnt_init_cnt = MNT_COUNT(lis_mnt);
		MNTSYNC();
		err = 0 ;
	    }
	}

	if (err != 0)
	{
	    printk(
	     "Linux Streams Subsystem failed to register its file system (%d).\n",
	     err) ;
	    return(err) ;
	}
    }

    lis_start_qsched() ;		/* ensure q running process going */

#if defined _S390X_LIS_ || defined (_X86_64_LIS_) || defined (_MIPS64_LIS_)
#if ! defined HAVE_COMPAT_IOCTL
	{
	int ret;

	ret = register_ioctl32_conversion(I_STR_COMPAT, lis_ioctl32_str);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_STR ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_PEEK_COMPAT, lis_ioctl32_peek);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_PEEK ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_PUSH_COMPAT, lis_ioctl32_compat);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_PUSH ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_POP_COMPAT, lis_ioctl32_compat);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_POP ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_LINK_COMPAT, lis_ioctl32_compat);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_LINK ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_UNLINK_COMPAT, lis_ioctl32_compat);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_UNLINK ret=%d..\n",
			ret);
		}
	ret = register_ioctl32_conversion(I_FLUSH_COMPAT, lis_ioctl32_compat);
	if (ret != 0)
		{
		printk(
	"regsiter_ioctl32_conversion failed for I_FLUSH ret=%d..\n",
			ret);
		}
	}
#endif
#endif
    /* This will create a LiS kset with the name 'LiS-hotplug' located
     * under /sys 
     */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
    lis_kset = lis_kset_create_and_add("LiS-hotplug", NULL, NULL);	
    if (!lis_kset)
	printk("LiS: WARNING! Failed to create hotplug kset!\n");
#endif

    /* short delay to allow printk driver transition */
    lis_udelay(20000);
    
    printk("Linux STREAMS Subsystem ready.\n");
    printk("Copyright (c) 1997-2004 David Grothe, et al, http://www.gcom.com\n");
    printk("Copyright (c) 2004-2011 Adax, Inc, http://www.adax.com\n");
    printk("Comverse modified version of Adax version LiS 2.19.4\n");
    printk("Version %s-%s %s. Compiled for kernel version %s.\n",
           lis_version, lis_release, lis_date, lis_kernel_version);
    printk("Major device number %d.\n",lis_major);

#if defined(USE_KMEM_CACHE) 
    printk("Kernel Memory Cache Enabled in this Version\n");
#else
    printk("Kernel Memory Cache Disabled in this Version\n");
#endif

#ifdef LIS_TUNABLE_PARAMETERS
    
    /********************************************************/
    /* Validate the Max Queue Depth for Queue Runners Value */
    /********************************************************/

    if((queue_runner_depth <= 0) || (queue_runner_depth > MAX_QUEUE_RUNNER_DEPTH))
    {
            /* Invalid value */
            printk("LiS: Invalid value for queue_runner_depth %d, using default %d\n",
                   queue_runner_depth, LIS_QUEUE_RUNNER_DEPTH);

            /* setup the actual value used by LiS */
            lis_work_incr = LIS_QUEUE_RUNNER_DEPTH;
    }
    else
    {
            /* setup the actual value used by LiS */
            lis_work_incr = queue_runner_depth;
    }            

    /****************************************/
    /* Validate the Scheduling Policy Value */
    /****************************************/

    if((scheduling_policy == LIS_SCHEDULER_RRP) ||
       (scheduling_policy == LIS_SCHEDULER_LAST))
    {
            /* Setup the actual value used by LiS */
            lis_scheduler_policy = scheduling_policy;
    }
    else
    {
            /* Invalid value */
            printk("LiS: Invalid value for scheduling_policy %d, using default LAST PROCESSOR FIRST\n",
                   scheduling_policy);

            /* setup the actual value used by LiS */
            lis_scheduler_policy = LIS_SCHEDULER_LAST;
    }

    printk("Max Queue Runner Queue Depth set to %d\n", lis_work_incr);
    
    if(scheduling_policy == LIS_SCHEDULER_RRP)
    {
            printk("Scheduling Policy is Round Robin (%d)\n", lis_scheduler_policy);
    }
    else
    {
            printk("Scheduling Policy is Last Processor First (%d)\n", lis_scheduler_policy);
    }

#endif /* LIS_TUNABLE_PARAMETERS */
    
#ifdef LIS_HAS_PROC_ENTRIES

        /**********************************/
        /* Create the LiS Proc File Entry */
        /**********************************/

        lis_proc_init();
#endif
       
	printk("==================================================================\n");

        /*************************************************/
        /* Signify that LiS has completed initialization */
        /*************************************************/

        lis_initialized = 1;
        
        return(0);
}

#ifdef MODULE			/* for loadable module support */

/*
 * Magic named routine called by kernel module support code.
 */

#ifdef KERNEL_2_5
int _lis_init_module( void )
#else
int init_module( void )
#endif
{
    return(lis_init_module()) ;
}

/*
 * Magic named routine called by kernel module support code.
 */
#ifdef KERNEL_2_5
void _lis_cleanup_module( void )
#else
void cleanup_module( void )
#endif
{
   extern void	lis_kill_qsched(void) ;
   extern void	lis_mem_terminate(void) ;
   extern void  lis_terminate_final(void) ;


#ifdef LIS_HAS_PROC_ENTRIES

    /**********************************/
    /* Remove the LiS Proc File Entry */
    /**********************************/

    lis_proc_exit();

#endif

    /* Upon entrance here we have to increase our module's refcount.
     * This is because MNTPUT will do a module_put (from 
     * deactivate_super()-> put_filesystem() on this module).  Some
     * genius at RedHat decided to add a BUG_ON line to module_put
     * in include/linux/module.h to check to see if the count was >=1
     * in RHEL 4 update 7.  Note that this is not in ANY kernel.org kernel
     * this was a RH invention...  Since our module refcount entering
     * here is 0 (since there are no users of streams) this causes
     * a panic on that one system.
     * We cannot just change the owner of the file system to be THIS_MODULE
     * because we have no way to unmount the LiS "filesystem" so
     * this creates an extra module reference which cannot be removed,
     * preventing the module from unloading.
     * We work around this by just blindly incrementing THIS_MODULE (ie
     * streams) module refcount, so that the the module_put called from
     * the MNTPUT below will have a module_refcount >= 1.  By the end
     * our module refcount is 0 again, so we're all good after the
     * MNTPUT 
     */

#ifdef KERNEL_2_5	/* actually 2.5.69 and beyond.... */
    /* ZZZ We cannot use LIS_MODGET because try_module_get() will fail 
     * from the cleanup routine (!module_is_live).. 
     * We also cannot just use __module_get() because our refcount
     * here is 0.  So they only thing that we can do is just copy
     * the guts from __module_get.  UGH! */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
    /* ZZZ unfortunately the refcounting changed in commit 720eba3 
     * so we only do this for sub 2.6.29 kernels.  The problem was
     * only seen on RHEL4U7 anyways... Update this to 
     * local_inc(__module_ref_addr(module, get_cpu())); 
     * if this is still necessary for post 2.6.29 kernels.
     * see include/linux/module.h */
    local_inc(&THIS_MODULE->ref[get_cpu()].count);
    put_cpu();
#endif
#endif


#if defined(FATTACH_VIA_MOUNT)
   /*
    *  It never made sense to do this here before, but it does now,
    *  as of 2.4.x (using mount() for fattach()), because the fdetach()
    *  process via mount can be left partially undone, e.g., if
    *  umount is used directly, or if an fattach'ed STREAM is "busy"
    *  when the fdetach is first tried (an unreceived PASSFP message,
    *  for example).  In such cases, the mount portion of the fdetach
    *  may have finished, so that the OS doesn't think there's any
    *  reason not to allow STREAMS to unload, but the fattach list
    *  may not be empty, and may reflect busy dentries, inodes, memory,
    *  and such.  So we try here to finish cleaning up after partially
    *  undone fattaches...
    */
   lis_fdetach_all();
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
   /* Remove the LiS hotplug kset */
   kset_unregister(lis_kset);
#endif

   /*
    * Make sure no streams modules are running,
    * and de-register devices unregister_netdev (dev);
    */

    lis_kill_qsched() ;			/* drivers/str/runq.c */
    lis_terminate_head();

#if (!defined(_S390_LIS_) && !defined(_S390X_LIS_))
    {
        extern void	lis_pci_cleanup(void) ;
        lis_pci_cleanup() ;
    } 
#endif          /* S390 or S390X */

    unregister_chrdev(lis_major,"streams");
    MNTSYNC();
    if (lis_mnt && MNT_COUNT(lis_mnt) > lis_mnt_init_cnt) {
	printk("LiS mount count is %d, should be %d\n",
	       MNT_COUNT(lis_mnt), lis_mnt_init_cnt);
    }

    if (lis_mnt != NULL)
	MNTPUT(lis_mnt) ;

    unregister_filesystem(&lis_file_system_ops) ;
    {
	int	n ;

	if ((n = K_ATOMIC_READ(&lis_inode_cnt)) != 0)
	    printk("LiS inode count is %d, should be 0\n", n) ;
    }

#ifdef KERNEL_2_5
    {
	void lis_free_devid_list(void) ;	/* in osif.c */
	lis_free_devid_list() ;
    }
#endif
    lis_terminate_final() ;		/* LiS internal memory */
    lis_mem_terminate() ;		/* LiS use of slab allocator */

#if defined _S390X_LIS_ || defined (_X86_64_LIS_) || defined (_MIPS64_LIS_)
#if ! defined HAVE_COMPAT_IOCTL
    unregister_ioctl32_conversion(I_POP_COMPAT);
    unregister_ioctl32_conversion(I_PUSH_COMPAT);
    unregister_ioctl32_conversion(I_LINK_COMPAT);
    unregister_ioctl32_conversion(I_UNLINK_COMPAT);
    unregister_ioctl32_conversion(I_FLUSH_COMPAT);
 
    unregister_ioctl32_conversion(I_STR_COMPAT);
    unregister_ioctl32_conversion(I_PEEK_COMPAT);
#endif
#endif

#ifdef KERNEL_2_5
    /* In reality we'll have a reference count of 0 here
     * from the MNTPUT() (deactivate_super), so this will do nothing
     * unless something went wrong in the MNTPUT */
    if (lis_modcnt(THIS_MODULE) > 0)
	LIS_MODPUT();
#endif

    printk ("Linux STREAMS Subsystem removed\n");
}

#ifdef KERNEL_2_5
module_init(_lis_init_module) ;
module_exit(_lis_cleanup_module) ;
#endif

#endif


/************************************************************************
*                          streams_init                                 *
*************************************************************************
*									*
* This routine is called from the Linux main.c.  It registers the 	*
* streams driver and initializes some streams variables.		*
*									*
* No memory is allocated.						*
*									*
************************************************************************/
void	streams_init(void)
{

    lis_init_module() ;		/* register STREAMS with Linux */

} /* streams_init */

#endif /* LINUX */

/************************************************************************
*                         Thread Startup                                *
*************************************************************************
*									*
* This routine can be called from other drivers, as well as from LiS	*
* internally, to start up a kernel thread.  It takes care of the	*
* business of shedding user mapped pages and such details.  The idea is	*
* to be able to keep the low-level kernel interactions out of the user's*
* thread code.								*
*									*
* The prototype for this resides in linux-mdep.h and will be automatic-	*
* ally included when you include <sys/stream.h>.			*
*									*
************************************************************************/

/*
 * Some defines for manipulating signals.
 */
#define	MY_SIGS		current->pending.signal
#define	MY_SIG		MY_SIGS.sig[0]
#define	MY_BLKS		current->blocked
#define	MY_BLKD		MY_BLKS.sig[0]

/*
 * This is the function that we start up.  It sheds user memory 
 * and calls the user's thread function.  When the user's function
 * returns, it exits.
 *
 * By default these threads run at normal priority.  LiS's own queue runner
 * thread ups its priority to that of a real-time process.
 */
typedef struct
{
    char	 name[sizeof(current->comm)] ;
    int        (*func)(void *func_arg) ;
    void	*func_arg ;

} arg_t ;

int	lis_thread_func(void *argp)
{
    arg_t		*arg = (arg_t *) argp ;
    int		       (*func)(void *) ;
    void		*func_arg ;

#if defined(KERNEL_2_5)
    daemonize("%s", arg->name) ;	/* make me a daemon */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,16)
    reparent_to_init() ;		/* disown all parentage */
#else
    daemonize() ;			/* make me a daemon */
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    current->uid = 0 ;			/* become root */
    current->euid = 0 ;			/* become root */
#else
    {
    struct cred *new = prepare_creds();
    new->uid = 0;
    new->euid = 0;
    commit_creds(new);
    }
#endif

#if !defined(KERNEL_2_5)
    strcpy(current->comm, arg->name) ;
#endif

    func = arg->func ;
    func_arg = arg->func_arg ;
    FREE(argp) ;			/* don't need args anymore */

    return(func(func_arg)) ;		/* enter caller's function */
    					/* without holding big kernel lock */
}

/*
 * Start the thread with "fcn(arg)" as the entry point.  Return the pid for the
 * new process, or < 0 for error.
 */
pid_t	_RP lis_thread_start(int (*fcn)(void *), void *arg, const char *name)
{
    arg_t	*argp ;

    argp = ALLOCF(sizeof(*argp), "Thread ") ;
    if (argp == NULL)
	return(-ENOMEM) ;

    if (name != NULL && name[0] != 0)
	strncpy(argp->name, name, sizeof(argp->name)) ;
    else
	strcpy(argp->name, "LiS-Thread") ;

    argp->func = fcn ;
    argp->func_arg = arg ;

    return(kernel_thread(lis_thread_func, (void *) argp, 0)) ;
}

int _RP
lis_thread_stop(pid_t pid)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
    /* kill_proc was removed in 19b0cfcca (2.6.27-rc1) */
    /* There are no users of this in LiS, it was only exported, so we 
     * can just get rid of it */
    return(kill_proc(pid, SIGTERM, 1));
#else
    return(kill_pid(find_vpid(pid), SIGKILL, 1) );
#endif
}


/************************************************************************
*                       Queue Running                                   *
*************************************************************************
*									*
* The following routines are used to manage running the STREAMS queues.	*
*									*
* The routines lis_setqsched and lis_kill_qsched are called from outside*
* this module.  lis_setqsched is a request to run the queues.  It means	*
* "now would be a good time to _schedule_ the queues to be run".	*
* list_kill_qsched is called from module unload to kill the kernel	*
* thread.  There is a semaphore interlock on this process to make sure	*
* that the process has been killed before unloading the module.		*
*									*
************************************************************************/


/*
 * This is the entry point of a kernel thread
 *
 * It appears that when this thread starts up it no longer holds the "big
 * kernel lock" (on 2.2 kernels).  So the first order of business is to acquire
 * that lock, sort of like we got here as the result of a system call.  Running
 * the queues, and calling kernel routines, without holding that lock is
 * troublesome for the 2.2 kernel.
 */
int	lis_thread_runqueues(void *p)
{
    intptr_t		 cpu_id = (intptr_t)(unsigned long) p ;
    int			 sig_cnt  = 0 ;
    unsigned long	 seconds = 0 ;
    extern char		 lis_print_buffer[] ;
    extern char		*lis_nxt_print_ptr ;

#if 0
    printk(KERN_INFO "LiS-RunQ-%s running on CPU %d pid=%d taskp=0x%p\n",
           lis_version, cpu_id, current->pid, current) ;
#endif
    
    lis_runq_tasks[cpu_id] = current;

    current->policy = SCHED_FIFO ;	/* real-time: run when ready */
    /* This is important.  For the realtime preemption modifications
     * they now run IRQs in kernel threads.  These kernel threads have
     * SCHED_FIFO policy and a priority of 25 to 50.  We must make sure
     * that the queue runner has a priority lower than the interrupt handler */
    current->rt_priority = 20 ;		/* middle value real-time priority */
    sigfillset(&MY_BLKS) ;		/* block all signals */
    sigdelset(&MY_BLKS, SIGKILL) ;	/* enable KILL */

#if defined(cpumask_of)
    set_cpus_allowed_ptr(current, cpumask_of(cpu_id));
#else
#if defined(SET_CPUS_ALLOWED)
# if defined(KERNEL_2_5)
    set_cpus_allowed(current, cpumask_of_cpu(cpu_id)) ;
# else
    set_cpus_allowed(current, 1 << cpu_id) ;
# endif
#else
    current->cpus_allowed = (1 << cpu_id) ;     /* older 2.4 kernels */
#endif
#endif

    K_ATOMIC_INC(&lis_queues_running);
    K_ATOMIC_INC(&lis_runq_active_flags[cpu_id]);
#if defined(KERNEL_2_5)
    yield() ;				/* reschedule our thread */
    					/* we will wake up on proper CPU */
#else
    schedule() ;			/* maybe this will do the same */
#endif
    
    for (;;)
    {
	queuerun(cpu_id, 1) ;	/* run the STREAMS queues */

	/* queuerun will set the task state to TASK_INTERRUPTIBLE
	 * under lock.  We might be woken up again by lis_setqsched before
	 * we get here so we might be TASK_INTERRUPTIBLE in which case we
	 * will sleep here or else TASK_RUNNABLE in which case schedule will
	 * simply reschedule us and we'll run again */
	schedule();
	if (unlikely(signal_pending(current)))
	{
	    /*
	     * When rebooting the kernel sends a TERM signal to all processes
	     * and then a KILL signal.  We want to keep running after the
	     * TERM so that we can close streams from user processes.  We
	     * will go away from a KILL or INT signal so that we can also
	     * be easily killed from the keyboard.
	     */
#if 0
	    printk("%s: Signalled: signal=0x%lx blocked=0x%lx, exiting\n",
		current->comm, MY_SIG, MY_BLKD) ;
#endif
	    if (   signal_pending(current)	/* killed */
		&& sigismember(&MY_SIGS, SIGKILL)
	       )
		break ;				/* process killed */

	    if (++sig_cnt >= 5)			/* nothing but signals */
	    {
		printk("%s: Signalled: signal=0x%lx blocked=0x%lx, exiting\n",
			current->comm, MY_SIG, MY_BLKD) ;
		sig_cnt = 0 ;
		break ;
	    }
	}
	else
	    sig_cnt = 0 ;

	lis_runq_wakeups[cpu_id]++ ;
	if (cpu_id != smp_processor_id())
	{
	    static int	msg_cnt ;
	    char	buf[200] ;

#if defined(KERNEL_2_5)
/* ugh.. the never ending clusterf**k that is the linux kernel */
/* See commits 76e6eee0 and a4636818 */
#ifdef tsk_cpumask
	    cpumask_scnprintf(buf, sizeof(buf), tsk_cpumask(current));
#elif defined tsk_cpus_allowed
	    cpumask_scnprintf(buf, sizeof(buf), tsk_cpus_allowed(current));
#else
	    cpumask_scnprintf(buf, sizeof(buf), current->cpus_allowed);
#endif
#else
	    sprintf(buf, "0x%lx", current->cpus_allowed) ;
#endif
	    if (++msg_cnt < 5)
		printk("%s woke up running on CPU %d -- cpu_id=%d mask=%s\n",
			current->comm, smp_processor_id(), cpu_id, buf) ;
	}
	/*
	 * If there are characters queued up in need of printing, print them if
	 * some time has elapsed.
	 */
	if (cpu_id == 0 && lis_nxt_print_ptr != lis_print_buffer)
	{
	    if (seconds == 0)			/* not timing yet */
		seconds = lis_secs() ;		/* start timing */
	    else
	    if (lis_secs() - seconds > 1)	/* has enough time gone by? */
	    {
		lis_flush_print_buffer() ;
		seconds = 0 ;			/* stop timing */
	    }
	}
    }

    printk(KERN_INFO "%s exiting pid=%d\n", current->comm, current->pid) ;
    lis_runq_pids[cpu_id] = 0 ;			/* process gone */
    lis_runq_tasks[cpu_id] = NULL ;			/* process gone */

    complete(&lis_runq_kill_sems[cpu_id]) ;	/* OK, we're killed */

    return(0) ;
}

/*
 * Called to start the queue running process.
 */
void	lis_start_qsched(void)
{
    int		cpu ;
    int		ncpus ;
    char	name[16] ;
    char	tmp[64] ;

    ncpus = NUM_CPUS ;
    lis_num_cpus = ncpus ;
    for (cpu = 0; cpu < ncpus; cpu++)
    {
	if (lis_runq_pids[cpu] > 0)		/* already running */
	    continue ;

	init_completion(&lis_runq_kill_sems[cpu]) ;/* initialize semaphore */

	/* The thread can only have a max of 16 chars, and this will
	 * overflow the stack if we use 'name' and the version string is 
	 * too long.  Keep the version string short! */
	sprintf(tmp, "LiS-%s:%u", lis_version, cpu) ;
	strncpy(name, tmp, sizeof(name) - 1) ;
	name[sizeof(name) - 1] = '\0';

	/* ZZZZ kernel_thread is going away circa 2.6.29... */
	lis_runq_pids[cpu] = lis_thread_start(lis_thread_runqueues,
					  (void *)(long)cpu, name) ;
	if (lis_runq_pids[cpu] < 0)		/* failed to fork */
	{
	    printk("lis_start_qsched: %s: lis_thread_start error %d\n",
		    name, lis_runq_pids[cpu]);
	    continue ;
	}

	K_ATOMIC_INC(&lis_runq_cnt) ;		/* one more running */
    }
}

/*
 * The "can_call" parameter is true if it is OK to just call the queue runner
 * from this routine.  It is false if it is necessary to defer the queue
 * running until later.  Typically qenable() passes 0 and others pass 1.
 */
void	lis_setqsched(int can_call)		/* kernel thread style */
{
    static unsigned int last_cpu;
    int		queues_running ;
    int		queue_load_thresh ;
    int		req_cnt ;
    int		in_intr = in_interrupt() ;
    int		my_cpu;
    int		cpu ;
    int		ncpus ;
    lis_flags_t psw;
    int WORK_INCR = lis_work_incr;
    int can_run;
    int i;

    if (WORK_INCR == 0)
    {
#ifdef LIS_TUNABLE_PARAMETERS
            /* Comverse default value */
            WORK_INCR = LIS_QUEUE_RUNNER_DEPTH;
#else
            /* Adax default value */
            /* For the HDC this value works pretty good.  Need
             * to test with other products */
            WORK_INCR = 16;
#endif
    }
    
    ncpus = NUM_CPUS;
    can_run = can_call & !in_intr;

    /* get_cpu() also disables involuntary preemption */
    my_cpu = get_cpu();

    lis_spin_lock_irqsave(&lis_qhead_lock, &psw);

    lis_setqsched_cnts[my_cpu]++ ;		/* keep statistics */
    if (in_intr)
	lis_setqsched_isr_cnts[my_cpu]++ ;	/* keep statistics */
    if (!can_call)
	lis_setqsched_defer_cnts[my_cpu]++ ;	/* keep statistics */

    queues_running = K_ATOMIC_READ(&lis_queues_running) ;
    queue_load_thresh = WORK_INCR * queues_running ;
    req_cnt = K_ATOMIC_READ(&lis_runq_req_cnt) ;

    /* If we're not at interrupt level just go for it.  Waking
     * up the background threads takes too friggin long and requires
     * a context switch. 
     */
    if (can_run && (req_cnt > queue_load_thresh) && 
	!K_ATOMIC_READ(&lis_runq_active_flags[my_cpu]))
	{
	K_ATOMIC_INC(&lis_runq_active_flags[my_cpu]);
	lis_setqsched_usr_cnts[my_cpu]++ ;	/* keep statistics */
	lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw);

	/* Since we are not locked here, new svc routines can be queued by
	 * other threads now */
	/* Note that we do NOT count this thread as a 'lis_queues_running'
	 * and hence other threads may come in here while this is running
	 * and start a queue runner thread for the same set of srv routines
	 * that we are currently running.  This is a necessary evil however
	 * because here we are NOT guarenteed to run all of the service
	 * routines which may become queued while in queuerun, only
	 * the ones queued when we start */
	queuerun(my_cpu, 0);

	lis_spin_lock_irqsave(&lis_qhead_lock, &psw);
	/* After running all the service routines which were queued up
	 * we check these stats again to see how many queue runner threads
	 * are active and how many service routines are queued up.  More
	 * could have arrived while we were running serivce routines.
	 */
	queues_running = K_ATOMIC_READ(&lis_queues_running) ;
	queue_load_thresh = WORK_INCR * queues_running ;
	req_cnt = K_ATOMIC_READ(&lis_runq_req_cnt) ;
	}

    /*
     * If the number of outstanding requests does not cross the upper
     * threhold for needing more queue run threads running, or if they
     * are all running already, just return.  There is nothing that we
     * can do to improve things.
     */
    if ( (req_cnt <= queue_load_thresh) || 
	 (queues_running == K_ATOMIC_READ(&lis_runq_cnt))
       )
	{
	lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw);
	put_cpu();
	return;
	}

    /*
     * We could benefit by having another queue run thread wake up
     * and help with the queue processing.  Don't wake up my cpu
     * until after loop completion since we don't want this loop
     * to be preempted.
     * We count down starting from ncpus because most systems seem to
     * bind PCI perpherial interrupts to CPU 0 and we want to use that
     * CPU last if possible.
     */
    if (lis_scheduler_policy == LIS_SCHEDULER_RRP)
	    cpu = last_cpu;
    else
	    cpu = ncpus;

    for (i = 0; i < ncpus; i++)
	{
	if (lis_scheduler_policy == LIS_SCHEDULER_RRP)
		cpu = (cpu + 1) % ncpus;
	else
		cpu = (cpu - 1) % ncpus;

	if (lis_runq_pids[cpu] == 0)
		continue;
	/* Don't start on my_cpu yet... */
	if ((cpu != my_cpu) && !K_ATOMIC_READ(&lis_runq_active_flags[cpu]))
		{
		/* Mark the queue runner as started, even though it might take
		* a while for it to actually start.  It will be marked again
		* as stopped in queuerun() after it's done running the queues */
		K_ATOMIC_INC(&lis_queues_running);
		K_ATOMIC_INC(&lis_runq_active_flags[cpu]) ;
		lis_setqsched_poke_cnts[cpu]++ ;	/* keep statistics */
		lis_wake_up_process(lis_runq_tasks[cpu]);
		queue_load_thresh += WORK_INCR ;	/* increase threshold */
		if (K_ATOMIC_READ(&lis_runq_req_cnt) <= queue_load_thresh)
			{
			last_cpu = cpu;
			lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw);
			put_cpu();
			return;
			}
		}
	}

    /*
     * If we get to here we still need another queue runner.  See if
     * my_cpu is available.
     */
    if (lis_runq_pids[my_cpu] && !K_ATOMIC_READ(&lis_runq_active_flags[my_cpu]))
	{
	K_ATOMIC_INC(&lis_queues_running);
	K_ATOMIC_INC(&lis_runq_active_flags[my_cpu]) ;
	lis_setqsched_poke_cnts[my_cpu]++ ;	/* keep statistics */
	lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw);
	put_cpu();
	lis_wake_up_process(lis_runq_tasks[my_cpu]);
	return;
	}

    lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw);
    put_cpu();

    return;
} /* lis_setqsched */

/*
 * Called from head code when time to unload LiS module.
 */
void	lis_kill_qsched(void)
{
    int		cpu ;

    for (cpu = 0; cpu < lis_num_cpus; cpu++)
    {
	if (lis_runq_pids[cpu] > 0) /* Only stop the kernel thread if running */
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
            /* kill_proc was removed in 19b0cfcca (2.6.27-rc1) */
	    kill_pid(find_vpid(lis_runq_pids[cpu]), SIGKILL, 1) ;
#else
	    kill_proc(lis_runq_pids[cpu], SIGKILL, 1) ;
#endif
	    wait_for_completion(&lis_runq_kill_sems[cpu]) ;
	    K_ATOMIC_DEC(&lis_runq_cnt) ;		/* one fewer running */
	}
    }
}


/************************************************************************
*                        Process Kill                                   *
************************************************************************/

int      lis_kill_proc(int pid, int sig, int priv)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    /* kill_proc was removed in 19b0cfcca (2.6.27-rc1) */
    return(kill_pid(find_vpid(pid), sig, priv)) ;
#else
    return(kill_proc(pid, sig, priv)) ;
#endif
}

int      lis_kill_pg (int pgrp, int sig, int priv)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	return(kill_pg(pgrp, sig, priv)) ;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	/* ugh.. linux'd again.. */
	return(kill_pgrp(find_pid(pgrp), sig, priv)) ;
#else
	/* This is updated for the new linux containers stuff */
	/* find_pid was removed in e49859e71 (2.6.27-rc1) */
	return(kill_pgrp(find_vpid(pgrp), sig, priv)) ;
#endif
}


/************************************************************************
*                        Signal Sets                                    *
*************************************************************************
*									*
* We can save and restore signals from the current task structure in	*
* an stdata structure.  We need to do this when closing a stream.  The	*
* close procedure needs to do some semaphore waits, but if the stream	*
* is closing because the process was killed (signalled) then the 	*
* semaphore waits will all fail with EINTR.				*
*									*
************************************************************************/

/* MarkS@Adax - 13 Jan 2006 - Added check of RCVOID to support use of
   LiS on MontaVista CGE 3.1 kernels which define recalc_sigpending as
   a void function.  See LiS Configure and config.mk files in the top
   level directory for usage of RCVOID. */
#if defined(SIGMASKLOCK)
#define LOCK_MASK    spin_lock_irq(&current->sigmask_lock)
#if defined(RCVOID)
#define UNLOCK_MASK    recalc_sigpending(); \
		       spin_unlock_irq(&current->sigmask_lock)
#else /* RCVOID */
#define UNLOCK_MASK    recalc_sigpending(current); \
		       spin_unlock_irq(&current->sigmask_lock)
#endif /* else RCVOID */
#else /* SIGMASKLOCK */
#define LOCK_MASK    spin_lock_irq(&current->sighand->siglock)
#define UNLOCK_MASK    recalc_sigpending(); \
		       spin_unlock_irq(&current->sighand->siglock)
#endif /* else SIGMASKLOCK */

void lis_clear_and_save_sigs(stdata_t *hd)
{
    sigset_t	*hd_sigs = (sigset_t *) hd->sd_save_sigs ;

    LOCK_MASK ;
    *hd_sigs = current->blocked ;
    sigfillset(&current->blocked) ;		/* block all signals */
    UNLOCK_MASK ;
}

void lis_restore_sigs(stdata_t *hd)
{
    sigset_t	*hd_sigs = (sigset_t *) hd->sd_save_sigs ;

    LOCK_MASK ;
    current->blocked = *hd_sigs ;
    UNLOCK_MASK ;
}

/************************************************************************
*                      Credentials Manipulation                         *
*************************************************************************
*									*
* These routines copy credentials to/from the task structure.		*
*									*
* This is only needed in a single CPU environment in which we run the	*
* queue schedule on top of a call from users.  In that case we need to	*
* assume kernel credentials while running the queues, so these routines	*
* need to actually do something.					*
*									*
* The lock here is high contention, so we avoid doing this work in a	*
* multi-cpu environment.						*
*									*
************************************************************************/
void	lis_task_to_creds(lis_kcreds_t *cp)
{
#if 0
    int		i ;
    lis_flags_t psw;

    lis_spin_lock_irqsave(&lis_task_lock, &psw) ;
    cp->uid = current->uid ;
    cp->euid = current->euid ;
    cp->suid = current->suid ;
    cp->fsuid = current->fsuid ;
    cp->gid = current->gid ;
    cp->egid = current->egid ;
    cp->sgid = current->sgid ;
    cp->fsgid = current->fsgid ;
    cp->cap_effective = current->cap_effective ;
    cp->cap_inheritable = current->cap_inheritable ;
    cp->cap_permitted = current->cap_permitted ;
    cp->ngroups = current->ngroups ;
    for (i = 0; i < current->ngroups; i++)
	cp->groups[i] = current->groups[i] ;
    lis_spin_unlock_irqrestore(&lis_task_lock, &psw) ;
#endif
}

void	lis_creds_to_task(lis_kcreds_t *cp)
{
#if 0
    int		i ;
    lis_flags_t psw;

    lis_spin_lock_irqsave(&lis_task_lock, &psw) ;
    current->uid = cp->uid ;
    current->euid = cp->euid ;
    current->suid = cp->suid ;
    current->fsuid = cp->fsuid ;
    current->gid = cp->gid ;
    current->egid = cp->egid ;
    current->sgid = cp->sgid ;
    current->fsgid = cp->fsgid ;
    current->cap_effective = cp->cap_effective ;
    current->cap_inheritable = cp->cap_inheritable ;
    current->cap_permitted = cp->cap_permitted ;
    current->ngroups = cp->ngroups ;
    for (i = 0; i < cp->ngroups; i++)
	current->groups[i] = cp->groups[i] ;
    lis_spin_unlock_irqrestore(&lis_task_lock, &psw) ;
#endif
}

/************************************************************************
*                           lis_mknod                                   *
*************************************************************************
*									*
* Like the system call.  "name" is the full pathname to create with the	*
* requested mode and device major/minor.				*
*									*
************************************************************************/
int	_RP lis_mknod(char *name, int mode, dev_t dev)
{
    mm_segment_t	old_fs;
    int			ret;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

#if defined(KENEL_2_5)
    ret = syscall_mknod(name, mode, kdev_val(dev)) ;
#else
    ret = syscall_mknod(name, mode, dev) ;
#endif

    set_fs(old_fs);
    return(ret < 0 ? -errno : ret) ;
}

/************************************************************************
*                           lis_unlink                                  *
*************************************************************************
*									*
* Remove a name from the directory structure.				*
*									*
************************************************************************/
int	_RP lis_unlink(char *name)
{
    mm_segment_t	old_fs;
    int			ret;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    ret = syscall_unlink(name) ;

    set_fs(old_fs);
    return(ret < 0 ? -errno : ret) ;
}


#if defined(FATTACH_VIA_MOUNT)
/*
 *  The following is an adaptation of 'permission(inode, mask)'; we
 *  need it because our argument is a path, not an inode.  Additionally,
 *  however, we want to check ownership for non-superuser processes.
 *
 *  The semantics here are patterned after what Solaris does for fattach,
 *  i.e., (root || (owner && write permission)).  It applies to both
 *  fattach & fdetach (i.e., mount/umount).  We expect EPERM if not non-root
 *  owner, otherwise EACCES if not write permission.
 *                                                     - JB 9/26/03
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
/* Easier to just rewrite this in the face of the changes around 
 * f419a2e3b64def707e1384ee38abb77f99af5f6d */
int mount_permission(char *pathname)
{
	int mask = MAY_WRITE;	/* read/exec access not needed */
	struct path path;
	int error = 0;
	struct inode *inode;

	if (capable(CAP_SYS_ADMIN))
		return 0;

	error = user_path(pathname, &path);
	if (error)
		return(error);   

	inode = path.dentry->d_inode;
	if (EUID(current) != inode->i_uid)
		return(-EPERM);  /* Solaris uses this for 'Not owner' */
	
	error = inode_permission(inode, mask);
	if (error)
		return(error);

	path_put(&path);
	return(error);
}
#else
int mount_permission(char * path)
{
    int mask = MAY_WRITE;	/* read/exec access not needed */
    int error = 0;
    struct nameidata nd;
    
    /*
     *  Always grant permission to superuser; no need for further checks
     */
    if (capable(CAP_SYS_ADMIN))  return 0;
    
    /*
     *  Otherwise, we need to check the owner, and permissions, at the
     *  path's inode.  The owner check we do is process effective user
     *  (FIXME if this isn't appropriate).
     */
    error = user_path_walk(path, &nd);
    if (!error) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
	struct dentry *dentry = nd.path.dentry;
#else
	struct dentry *dentry = nd.dentry;
#endif
	struct inode *inode   = (dentry ? dentry->d_inode : NULL);

#if !defined(KERNEL_2_5)
	/* 2.4.x kernels do something like the following... */
	if (inode && inode->i_op && inode->i_op->revalidate)
	    error = inode->i_op->revalidate(dentry);
#endif
	
	/* check process euid == inode uid */
	if (!error && (current->euid != inode->i_uid))
	    error = -EPERM;  /* Solaris uses this for 'Not owner' */
	
	/* check permission(s) */
	if (!error)
#if defined(KERNEL_2_5)
	    error = permission(inode, mask, &nd);
#else
	    error = permission(inode, mask);
#endif
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
	path_put(&nd.path);
#else
	path_release(&nd);
#endif
    }

    return error;
}
#endif
#endif

/************************************************************************
*                             lis_mount                                 *
*************************************************************************
*									*
* A wrapper for a mount system call.					*
*									*
************************************************************************/
int	lis_mount(char *dev_name,
		  char *dir_name,
		  char *fstype,
		  unsigned long rwflag,
		  void *data)
{
    mm_segment_t	old_fs;
    int			ret;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

#if defined(FATTACH_VIA_MOUNT)
    if (!(ret = mount_permission(dir_name))) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28))
	{
	kernel_cap_t cap = current->cap_effective;
	if (!cap_raise(current->cap_effective, CAP_SYS_ADMIN))
	    return(-EPERM);
	ret = syscall_mount(dev_name, dir_name, fstype, rwflag, data) ;
	current->cap_effective = cap;
	}
#else
	{
	struct cred *new = prepare_creds();
	cap_raise(new->cap_effective, CAP_SYS_ADMIN);
	commit_creds(new);
	ret = syscall_mount(dev_name, dir_name, fstype, rwflag, data) ;
	cap_lower(new->cap_effective, CAP_SYS_ADMIN);
	commit_creds(new);
	}
#endif
    }
#else
    ret = syscall_mount(dev_name, dir_name, fstype, rwflag, data) ;
#endif

    set_fs(old_fs);

    return(ret < 0 ? ret : -ret) ;
}

/************************************************************************
*                            lis_umount                                 *
*************************************************************************
*									*
* A wrapper for a umount system call.					*
*									*
* We must get the flags passed, hence we must use umount2; umount       *
* ignores flags, and fdetach via umount[2]() doesn't work without the   *
* MNT_FORCE flag being passed.   - JB 2002/11/18                        *
************************************************************************/
int	lis_umount2(char *path, int flags)
{
    mm_segment_t	old_fs;
    int			ret;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

#if defined(FATTACH_VIA_MOUNT)
    if (!(ret = mount_permission(path))) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28))
	{
	kernel_cap_t cap = current->cap_effective;
	if (!cap_raise(current->cap_effective, CAP_SYS_ADMIN))
	    return(-EPERM);
	ret = syscall_umount2(path, flags) ;
	current->cap_effective = cap;
	}
#else
	{
	struct cred *new = prepare_creds();
	cap_raise(new->cap_effective, CAP_SYS_ADMIN);
	commit_creds(new);
	ret = syscall_umount2(path, flags) ;
	cap_lower(new->cap_effective, CAP_SYS_ADMIN);
	commit_creds(new);
	}
#endif
    }
#else
    ret = syscall_umount2(path, flags) ;
#endif

    set_fs(old_fs);

    return(ret < 0 ? ret : -ret) ;
}

/************************************************************************
*                         Module Information                            *
************************************************************************/

/*
 * Note:  We are labeling the module license as "GPL and additional rights".
 * This is said to be equivalent to GPL for symbol exporting purposes and
 * is also supposed to span LGPL.
 */
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL and additional rights");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("David Grothe <dave@gcom.com>");
#endif
#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("SVR4 STREAMS for Linux (LGPL Code)");
#endif
#if defined(MODULE_INFO) && defined(VERMAGIC_STRING)
MODULE_INFO(vermagic, VERMAGIC_STRING);
#endif

/************************************************************************
*                 Linux Kernel Cache Memory Routines                    *
*                 for allocating and freeing mdbblocks                  *
*             as a LiS performance improvement under Linux              *
************************************************************************/

#if defined(USE_KMEM_CACHE) 
void lis_init_msg(void)
{
	lis_msgb_cachep = lis_kmem_cache_create(
					"LiS-msgb", sizeof(struct mdbblock),
					0, SLAB_HWCACHE_ALIGN);
        if (!lis_msgb_cachep) 
                printk("lis_init_msg: lis_msgb_cachep is NULL. "
			"lis_kmem_cache_create failed\n");
}

void lis_msgb_cache_freehdr(void *bp)
{
      if (lis_msgb_cachep) {
              LisDownCount(HEADERS);
	      K_ATOMIC_DEC(&lis_msgb_cnt) ;
              kmem_cache_free(lis_msgb_cachep, (struct mdbblock *) bp);
	      if (LIS_DEBUG_CACHE)
		printk("lis_msgb_cache_freehdr: freed %p\n", bp);
      }
}/*lis_msgb_cachep_free*/


struct mdbblock *lis_kmem_cache_allochdr(void)
{
      extern lis_atomic_t  lis_strcount ;     /* # bytes allocated to msgs   */
      extern long          lis_max_msg_mem ;  /* maximum to allocate */

      struct mdbblock *md;

      if (   (   !lis_max_msg_mem
	      || K_ATOMIC_READ(&lis_strcount) < lis_max_msg_mem
	     )
	  && (md = kmem_cache_alloc(lis_msgb_cachep, GFP_ATOMIC))
	 )
      {
	  LisUpCount(HEADERS);
	  K_ATOMIC_INC(&lis_msgb_cnt) ;
	  if (LIS_DEBUG_CACHE)
	      printk("lis_kmem_cache_allochdr: allocated %p\n", md);
	  md->msgblk.m_mblock.b_next = NULL;
	  return (md);
      } 
      
      printk("lis_kmem_cache_allochdr: msgb allocation failed\n");
      LisUpFailCount(HEADERS);
      return (NULL);
}

void lis_terminate_msg(void)
{
    lis_cache_destroy(lis_msgb_cachep, &lis_msgb_cnt, "LiS-msgb") ;
}

/************************************************************************
*                 Linux Kernel Cache Memory Routines                    *
*             for allocating and freeing queues and qbands              *
*             as a LiS performance improvement under Linux              *
************************************************************************/

void lis_cache_destroy(lis_kmem_cache_t *p, lis_atomic_t *c, char *label)
{
    int		n = K_ATOMIC_READ(c);

    if (n)
    {
	printk("lis_cache_destroy: "
	       "Kernel cache \"%s\" has %d blocks still allocated\n"
	       "                   "
	       "Not destroying kernel cache.  You will probably have\n"
	       "                   "
	       "to reboot to clear this condition.\n",
	       label, n) ;
	return ;
    }

    kmem_cache_destroy(p);
}

void lis_init_queues(void)
{
      lis_queue_cachep =
          lis_kmem_cache_create("LiS-queue", sizeof(queue_t)*2, 0,
                            SLAB_HWCACHE_ALIGN);
      lis_qsync_cachep =
          lis_kmem_cache_create("LiS-qsync", sizeof(lis_q_sync_t), 0,
                            SLAB_HWCACHE_ALIGN);
      lis_qband_cachep =
          lis_kmem_cache_create("LiS-qband", sizeof(qband_t), 0,
                            SLAB_HWCACHE_ALIGN);
      lis_head_cachep =
          lis_kmem_cache_create("LiS-head", sizeof(stdata_t), 0,
                            SLAB_HWCACHE_ALIGN);
}

void lis_terminate_queues(void)
{
      lis_cache_destroy(lis_head_cachep, &lis_head_cnt, "LiS-head");
      lis_cache_destroy(lis_qband_cachep, &lis_qband_cnt, "LiS-qband");
      lis_cache_destroy(lis_queue_cachep, &lis_queue_cnt, "LiS-queue");
      lis_cache_destroy(lis_qsync_cachep, &lis_qsync_cnt, "LiS-qsync");
}
#endif

/************************************************************************
*            LINUX kernel cache based SVR4 Compatible timeout           *
*************************************************************************
*                                                                       *
* Uses the kernel cache mechanism to speed allocations and keep timers	*
* grouped together.							*
*                                                                       *
************************************************************************/

int lis_timer_size ;

void lis_init_timers(int size)
{
#if defined(USE_KMEM_TIMER) 
    lis_timer_cachep = lis_kmem_cache_create("lis_timer_cachep", 
				 size, 0, SLAB_HWCACHE_ALIGN);
    if (!lis_timer_cachep) 
	printk("lis_init_dki: lis_timer_cachep is NULL. "
			"lis_kmem_cache_create failed\n");
#endif
    lis_timer_size = size ;
}

void lis_terminate_timers(void)
{
#if defined(USE_KMEM_TIMER) 
    kmem_cache_destroy(lis_timer_cachep) ;
#endif
}

void *lis_alloc_timer(char *file, int line)
{
    void	*t ;

#if defined(CONFIG_DEV) || !defined(USE_KMEM_TIMER)
    t = LISALLOC(lis_timer_size,file, line) ;
#else
    t = kmem_cache_alloc(lis_timer_cachep, GFP_ATOMIC) ;
#endif

    return(t) ;
}

void *lis_free_timer(void *timerp)
{
#if defined(CONFIG_DEV) || !defined(USE_KMEM_TIMER)
    FREE(timerp) ;
#else
    kmem_cache_free(lis_timer_cachep, timerp);
#endif
    return(NULL) ;
}

/******************************************/
/* Linux Proc Filesystem Support Routines */
/******************************************/

#ifdef LIS_HAS_PROC_ENTRIES

/* FUNCTION lis_proc_init() - Create a Proc Filesystem Entry            */
/************************************************************************
FUNCTION
        int lis_proc_init(void)
        
DESCRIPTION
	This procedure creates a proc filesystem entry.

INPUTS
        None

OUTPUTS
	-1 Failure: Not able to create the proc entry
	 0 Success

ERRORS
	Unable to create the proc entry

*************************************************************************/
int lis_proc_init(void)
{
        lis_diag_t *lis_diag = &lis_diag_log;
        lis_sc_trace_t *lis_diag_sc_trace = &lis_diag_sc_trace_log;

        if(!lis_dir)
        {
                lis_dir = create_proc_entry(LIS_PROC_DIR, 0666, 0);
                if(!lis_dir)
                {
                        printk("lis.lis_proc_init: ERROR creating proc filesystem entry LiS\n");
                        return(-1);
                }
                else
                {
                        printk("Created Proc Filesystem Entry LiS\n");
                        
                        lis_dir->read_proc  = lis_procfile_read;
                        lis_dir->write_proc = lis_procfile_write;
                        lis_dir->data       = lis_dir;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
                        lis_dir->owner      = THIS_MODULE;
#endif
                        
                        /* clear the log structure */
                        (void) memset (&lis_diag_log, 0, sizeof(lis_diag_log));

                        /* clear the System Call Trace log structure */
                        (void) memset (&lis_diag_sc_trace_log, 0, sizeof(lis_diag_sc_trace_log));
                        
                        /* set the default event and error logging enabled */
                        lis_diag->event_logging = 1;
                        lis_diag->error_logging = 1;

                        /* set the default System Call Trace logging enabled */
                        lis_diag_sc_trace->trace_logging = 1;
                }
        }

        return(0);
}


/* FUNCTION lis_proc_exit() - Remove a Proc Filesystem Entry            */
/************************************************************************
FUNCTION
        int m2pa_l2_proc_exit(void)

        
DESCRIPTION
	This procedure removes a proc filesystem entry.

INPUTS
        None

OUTPUTS
	0 Success

ERRORS
        None

*************************************************************************/
int lis_proc_exit(void)
{
        if(lis_dir)
        {
                remove_proc_entry(LIS_PROC_DIR, 0);
                lis_dir = 0;
                
                printk("lis.lis_proc_exit: Removed Proc Filesystem Entry LiS\n");
        }

        return(0);
}


/* FUNCTION lis_procfile_read() - Read Rtn Proc Filesystem Entry        */
/************************************************************************
FUNCTION
        int lis_procfile_read(...)
        
DESCRIPTION
	This procedure provides the read capability for the proc filesystem entry.
        Read represents an output to the user.

INPUTS
        Linux specific input fields

OUTPUTS
        -EINVAL -  Incorrect proc filename
        0 Success (Also, other fields are filled in as required)

ERRORS
	Incorrect proc filename

*************************************************************************/
int lis_procfile_read(char *buffer, char **buffer_location, off_t offset, int count, int *eof, void *data)
{
        /*****************************************/
        /* READ represents an output to the User */
        /*****************************************/

        int return_count = 0;
    
#if 0
        printk("lis:lis_procfile_read READ Routine entered\n");
#endif

        if(data == lis_dir)
        {
                /*********************************************/
                /* Check if multiple reads per output buffer */
                /*********************************************/

                if(offset > 0)
                {
                        /******************/
                        /* Multiple reads */
                        /******************/
                        
                        if(lis_count_in_buffer)
                        {
                                *buffer_location = (lis_output_buffer + (int)offset);
                        }
                        else
                        {
                                /* Nothing left in the buffer to output */
                                return(0);
                        }
                }
                else
                {
                        /*************************/
                        /* First read per buffer */
                        /*************************/

                        *buffer_location = lis_output_buffer;
                }

                /*********************************/
                /* Calculate the count to return */
                /*********************************/

                if(lis_count_in_buffer > count)
                {
                        *eof = 0;
                        lis_count_in_buffer -= count;
                        return_count = count;
                }
                else 
                {
                        *eof = 1;
                        return_count = lis_count_in_buffer;

                        /* Buffer output has completed */
                        lis_count_in_buffer = 0;
                }

                return(return_count);
        }
        else
        {
                printk("lis:lis_procfile_read READ Routine ERROR Exit\n");
                
                return(-EINVAL);
        }
}


/* FUNCTION lis_procfile_write() - Write Rtn Proc Filesystem Entry      */
/************************************************************************
FUNCTION
        int lis_procfile_write(...)
        
DESCRIPTION
	This procedure provides the write capability for the proc filesystem entry.
        Write represents an input from the user.

INPUTS
        Linux specific input fields

OUTPUTS
        -EINVAL -  Incorrect proc filename
        Count from copy_from_user() call

ERRORS
	Incorrect proc filename

*************************************************************************/
int lis_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        /*******************************************/
        /* WRITE represents an input from the User */
        /*******************************************/

        int scnt;
        int option;
        int parameter1 = 0;

#if 0
        int parameter2; /* not yet used */
#endif
        char *string_input;
        int not_copied_count = 0;;
        int cnt = 0;;
        char *lis_out_bufp = (char *)&lis_out_buf;
        char *global_buffer = (char *)&lis_output_buffer;

        lis_diag_t *lis_diag;
        int event_logging = 0;
        int error_logging = 0;

        lis_sc_trace_t *lis_diag_sc_trace;
        int trace_logging = 0;
        
        int i;
        
#if 0
        printk("lis: lis_procfile_write WRITE Routine entered\n");
#endif
        
        /* allocate data for input */
        string_input = kmalloc(PAGE_SIZE,GFP_KERNEL);
        bzero(string_input, PAGE_SIZE);

        /* get the input */
        not_copied_count = copy_from_user(string_input, buffer, count);

        if(data == lis_dir)
        {
                scnt = sscanf(string_input, "%d,%d", &option, &parameter1);
                if(!scnt)
                {
                        /* This is actually Option ? */
                        
                        /*************/
                        /* Help Menu */
                        /*************/
                        
                        /* place the menu in the output buffer */
                        cnt = sprintf(global_buffer, "\nLiS DIAGNOSTIC OPTIONS\n");
                        cnt += sprintf(lis_out_bufp,   "----------------------\n\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "1   - Display Event/Error Log\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "2   - Clear Event/Error Log\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "3,i - i = Enable (1) / Disable (0) Event Logging\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "4,i - i = Enable (1) / Disable (0) Error Logging\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "5   - Display Counters\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "6   - Clear Counters\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "7   - Display System Call Trace Log\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "8   - Clear System Call Trace Log\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "9,i - i = Enable (1) / Disable (0) System Call Trace Logging\n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "?   - Display Help Menu \n");
                        strcat(global_buffer,lis_out_bufp);
                        cnt += sprintf(lis_out_bufp, "\n");
                        strcat(global_buffer,lis_out_bufp);                        
                        
                        /* save the count */
                        lis_count_in_buffer = cnt;

                        /* count from copy_from_user call */
                        return(count);
                }
                
                switch(option)
                {
                case 1:        /* Display Event/Error Log */
                {
                        lis_count_in_buffer = lis_diag_disp_log();
                        
                        break;
                }
                case 2:        /* Clear Event/Error Log */
                {
                        lis_diag = &lis_diag_log;
                        
                        /* save the logging enabled flags */
                        event_logging = lis_diag->event_logging;
                        error_logging = lis_diag->error_logging;
                        
                        /* clear the log structure */
                        (void) memset (&lis_diag_log, 0, sizeof(lis_diag_log));
                        
                        /* restore the event and error logging enabled flags */
                        lis_diag->event_logging = event_logging;
                        lis_diag->error_logging = error_logging;

                        lis_count_in_buffer = sprintf(global_buffer, "\nEvent/Error Log Cleared\n\n");
                        break;
                }
                case 3:        /* Enable/Disable Event Logging */
                {
                        if(parameter1 == 1)
                        {
                                lis_diag = &lis_diag_log;

                                /* Enable Event Logging */
                                lis_diag->event_logging = 1;

                                lis_count_in_buffer = sprintf(global_buffer, "\nEvent Logging Enabled\n\n");
                        }
                        else if(parameter1 == 0)
                        {
                                lis_diag = &lis_diag_log;

                                /* Disable Event Logging */
                                lis_diag->event_logging = 0;

                                lis_count_in_buffer = sprintf(global_buffer, "\nEvent Logging Disabled\n\n");
                        }
                        else
                        {
                                lis_count_in_buffer = sprintf(global_buffer, "\nInvalid Parameter %d for Option 3\n\n", parameter1);
                        }
                        
                        break;
                }
                case 4:        /* Enable/Disable Error Logging */
                {
                        if(parameter1 == 1)
                        {
                                lis_diag = &lis_diag_log;

                                /* Enable Error Logging */
                                lis_diag->error_logging = 1;

                                lis_count_in_buffer = sprintf(global_buffer, "\nError Logging Enabled\n\n");
                        }
                        else if(parameter1 == 0)
                        {
                                lis_diag = &lis_diag_log;

                                /* Disable Error Logging */
                                lis_diag->error_logging = 0;

                                lis_count_in_buffer = sprintf(global_buffer, "\nError Logging Disabled\n\n");
                        }
                        else
                        {
                                lis_count_in_buffer = sprintf(global_buffer, "\nInvalid Parameter %d for Option 4\n\n", parameter1);
                        }

                        break;
                }

                case 5:        /* Display Counters */
                {
                        cnt = sprintf(global_buffer, "\nLiS Counters Display\n");
                        cnt += sprintf(lis_out_bufp,   "--------------------\n\n");
                        strcat(global_buffer,lis_out_bufp);

                        for(i=0; i < NUMBER_OF_DEBUG_COUNTERS; i++)
                        {
                                if(lis_debug_err[i])
                                {
                                        cnt+= sprintf(lis_out_bufp,
                                                      "COUNTER #[%d] occurred [%ld] times\n",
                                                      i, lis_debug_err[i]);
                                        strcat(global_buffer,lis_out_bufp);
                                }
                        }

                        /* Add a linefeed */
                        cnt += sprintf(lis_out_bufp, "\n");
                        strcat(global_buffer,lis_out_bufp);                
                        
                        /* save the count */
                        lis_count_in_buffer = cnt;

                        break;
                }
                case 6:        /* Clear Counters */
                {
                        for(i=0; i < NUMBER_OF_DEBUG_COUNTERS; i++)
                        {
                                lis_debug_err[i] = 0;
                        }

                        lis_count_in_buffer = sprintf(global_buffer, "\nLiS Counters Cleared\n\n");
                        break;
                }
                case 7:        /* Display System Call Trace Log */
                {
                        lis_count_in_buffer = lis_diag_disp_sc_trace_log();
                        
                        break;
                }
                case 8:        /* Clear System Call Trace Log */
                {
                        lis_diag_sc_trace = &lis_diag_sc_trace_log;
                        
                        /* save the logging enabled flag */
                        trace_logging = lis_diag_sc_trace->trace_logging;
                        
                        /* clear the log structure */
                        (void) memset (&lis_diag_sc_trace_log, 0, sizeof(lis_diag_sc_trace_log));
                        
                        /* restore the logging enabled flags */
                        lis_diag_sc_trace->trace_logging = trace_logging;

                        lis_count_in_buffer = sprintf(global_buffer, "\nSystem Call Trace Log Cleared\n\n");
                        break;
                }
                case 9:        /* Enable/Disable System Call Trace Logging */
                {
                        if(parameter1 == 1)
                        {
                                lis_diag_sc_trace = &lis_diag_sc_trace_log;

                                /* Enable Trace Logging */
                                lis_diag_sc_trace->trace_logging = 1;

                                lis_count_in_buffer = sprintf(global_buffer, "\nSystem Call Trace Logging Enabled\n\n");
                        }
                        else if(parameter1 == 0)
                        {
                                lis_diag_sc_trace = &lis_diag_sc_trace_log;

                                /* Disable Trace Logging */
                                lis_diag_sc_trace->trace_logging = 0;

                                lis_count_in_buffer = sprintf(global_buffer, "\nSystem Call Trace Logging Disabled\n\n");
                        }
                        else
                        {
                                lis_count_in_buffer = sprintf(global_buffer, "\nInvalid Parameter %d for Option 3\n\n", parameter1);
                        }
                        
                        break;
                }
                default:
                {
                        lis_count_in_buffer = sprintf(global_buffer, "\nlis: lis_procfile_write WRITE Invalid Option = %d\n\n", option);
                        
                        break;
                }
                }

                /* count from copy_from_user call */
                return(count);
        }
        else
        {
                printk("lis: lis_procfile_write WRITE Routine ERROR Exit\n");
                
                return(-EINVAL);
        }
}

/* FUNCTION lis_diag_log_event() - log an event                         */
/*************************************************************************
								
FUNCTION
	void lis_diag_log_event()

DESCRIPTION
	This function is called to log an event.

INPUTS
        int event_type    - The event type
        char *func        - The function name
        int line_number   - The line number in the file
        int counter       - The counter number associated with this event

OUTPUTS
	None

*************************************************************************/
void lis_diag_log_event(int event_type, char *func, int line_number, int counter)
{
	lis_diag_entry_t *pent;

        /*************************************/
        /* Check if event logging is enabled */
        /*************************************/

        if(!lis_diag_log.event_logging)
        {
                return;
        }
                
        /*******************************************/
        /* Get the next available entry in the log */
        /*******************************************/

        pent = lis_diag_get_entry();

        /*************************/
        /* Fill in the log entry */
        /*************************/

        pent->type = LOG_TYPE_EVENT;
        pent->event_type = event_type;
        strncpy (pent->function, func, MAXfunctionNAME);
        pent->line_number = line_number;
        pent->counter = counter;
        
        return;
}

/* FUNCTION lis_diag_log_error() - log an error                          */
/*************************************************************************
								
FUNCTION
	void lis_diag_log_error()

DESCRIPTION
	This function is called to log an error.  
        These are error values returned that usually end up in errno for
        return to the calling process.

INPUTS
        int error_value - The error value as an errno defined value
        char *func      - The function name
        int line_number - The line number in the file
        int counter     - The counter number associated with this event

OUTPUTS
	None

*************************************************************************/
void lis_diag_log_error(int error_value, char *func, int line_number, int counter)
{
	lis_diag_entry_t *pent;

        /*************************************/
        /* Check if error logging is enabled */
        /*************************************/

        if(!lis_diag_log.error_logging)
        {
                return;
        }
                
        /*******************************************/
        /* Get the next available entry in the log */
        /*******************************************/

        pent = lis_diag_get_entry();

        /*************************/
        /* Fill in the log entry */
        /*************************/

        pent->type = LOG_TYPE_ERROR;
        pent->error_value = error_value;
        strncpy (pent->function, func, MAXfunctionNAME);
        pent->line_number = line_number;
        pent->counter = counter;

        return;
}

/* FUNCTION lis_diag_get_entry() - get log entry	                 */
/*************************************************************************
								
FUNCTION
	lis_diag_entry_t *lis_diag_get_entry()

DESCRIPTION
	This function is called to get an event/error log entry.

INPUTS
	None

OUTPUTS
	Pointer to a log entry

*************************************************************************/
lis_diag_entry_t *lis_diag_get_entry()
{
	lis_diag_entry_t *pent;
	int index;
	struct timeval curr_time;

        /*******************************************/
        /* Get the next available entry in the log */
        /*******************************************/

        index = lis_diag_log.index;
        pent = &lis_diag_log.ent[index];

        /***********************/
        /* Timestamp the entry */
        /***********************/

        do_gettimeofday(&curr_time);

	pent->tv_sec  = curr_time.tv_sec;
	pent->tv_usec = curr_time.tv_usec;

        /********************/
        /* Update the index */
        /********************/
        
	index++;

	if(index >= LIS_DIAG_TRACE_ENTRIES)
	{
		lis_diag_log.wrapped = 1;
		index = 0;
	}

        /* index to next entry */
        lis_diag_log.index = index;

        /* return this entry */        
        return(pent);
}

/* FUNCTION lis_diag_disp_log() - Display the Event/Error Log            */
/*************************************************************************
								
FUNCTION
	int lis_diag_disp_log()

DESCRIPTION
	This function is called to display the event/error log.

        Note that this routine currently always displays all of the
        entries in the log.

INPUTS
	None

OUTPUTS
	Count of characters in the global buffer.

*************************************************************************/
int lis_diag_disp_log()
{
        lis_diag_t *lis_diag = &lis_diag_log;
	lis_diag_entry_t *pent;
	int number_of_entries;
        int event_logging;
        int error_logging;

        int cnt = 0;
        char *lis_out_bufp = (char *)&lis_out_buf;
        char *global_buffer = (char *)&lis_output_buffer;
        
	int i;
	int ix;

        /* save the logging enabled flags */
        event_logging = lis_diag->event_logging;
        error_logging = lis_diag->error_logging;

        /* temporarily disable event/error logging */
        lis_diag->event_logging = 0;
        lis_diag->error_logging = 0;

        /******************************************************************/
        /* Set up the starting index and the number of entries to display */
        /******************************************************************/

        number_of_entries = LIS_DIAG_TRACE_ENTRIES;

	if((int)lis_diag->index < number_of_entries)
	{
		if(lis_diag->wrapped)
		{
                        /* start from the current index */
 			ix = lis_diag->index;
		}
		else
		{
                        /* start from the first entry */
                        ix = 0;

                        /* adjust the number of entries to display value */
			number_of_entries = lis_diag->index;
		}
	}
	else
	{
                /* start from the first entry */
		ix = 0;
	}

        cnt = sprintf(global_buffer, "\nLiS Diagnostics Event/Error Log Display\n");
        cnt += sprintf(lis_out_bufp, "---------------------------------------\n");
        strcat(global_buffer,lis_out_bufp);

        cnt += sprintf(lis_out_bufp, "\nKey:\nERROR (counter #): Errno Value, LiS Function Name, Line Number\n");
        strcat(global_buffer,lis_out_bufp);                
        cnt += sprintf(lis_out_bufp, "EVENT (counter #): ..., LiS Function Name, Line Number\n");
        strcat(global_buffer,lis_out_bufp);                
        
        cnt += sprintf(lis_out_bufp, "\nStarting Index %d, Wrapped %d, Index %d\n\n",
                       ix, lis_diag->wrapped, lis_diag->index);
        strcat(global_buffer,lis_out_bufp);                

        /****************/
        /* Display Loop */
        /****************/

        for(i = 0; i < number_of_entries; i++)
	{
                /*****************************************************************/
                /* Check that the Log output isn't too big for the output buffer */
                /*****************************************************************/

                if(cnt >= (LIS_OUTPUT_BUFFER_SIZE - LIS_DIAG_MAX_OUTPUT_LINE_SIZE))
                {
                        /* Informational message which means that the output buffer */
                        /* size must be increased.  This is a compile time issue.   */
                        
                        /* Throw away the buffer content and print an error */
                        cnt = sprintf(global_buffer, "\n\nLog output too large for buffer\n");

                        printk("lis.%s: Log output too large for buffer (cnt = %d\n",
                               __func__, cnt);

                        break;
                }
                        
                /*****************************/
                /* Get an entry from the log */
                /*****************************/

		pent = &lis_diag->ent[ix];

                /* format the timestap */
                cnt += sprintf(lis_out_bufp, "%03d. [!%ld!:%03ld] ",
                               ix, (unsigned long)pent->tv_sec, ((unsigned long)pent->tv_usec/1000));
                strcat(global_buffer,lis_out_bufp);                
                
                /* increment the index and check for wrap */
                ix++;
                
		if(ix >= LIS_DIAG_TRACE_ENTRIES)
		{
                        /* wrap to the beginning */
			ix = 0;
		}
        
                /****************************/
                /* Determine the entry type */
                /****************************/

                if(pent->type == LOG_TYPE_EVENT)
                {
                        /**************************************/
                        /* Process the Log Entry Type = Event */
                        /**************************************/

                        switch(pent->event_type)
                        {
                                case STRPRI_SET_PUSH_HIP:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                                "EVENT (counter %d): STRPRI Set, Push High Priority Message: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }
                                case M_DATA_POLL_NOT_WAITING:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                               "EVENT (counter %d): M_DATA, Poll Not Waiting: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }
                                case M_DATA_POLL_WAITING:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                               "EVENT (counter %d): M_DATA, Poll Waiting: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }
                                case STRPRI_SET_EMPTY:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                               "EVENT (counter %d): STRPRI Set, q_first Empty: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }
                                case STRPRI_SET_NOT_EMPTY:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                               "EVENT (counter %d): STRPRI Set, q_first Not Empty: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }                                
                                case ERR_PUTMSG:
                                {
                                        cnt += sprintf(lis_out_bufp,
                                               "EVENT (counter %d): PUT ERROR, message discarded: ",
                                                       pent->counter);
                                        strcat(global_buffer,lis_out_bufp);
                                        break;
                                }                                
                                default:
                                {
                                        /* Unknown Event Type - abort the display */
                                        cnt += sprintf(lis_out_bufp, "\nUnknown Event Type in Log: %d\n\n",
                                                       pent->event_type);

                                        strcat(global_buffer,lis_out_bufp);
                                        return(cnt);
                                        break;
                                }
                                } /* end switch */

                        /* Common fields */
                        cnt += sprintf(lis_out_bufp, " %s, %d\n",
                               pent->function, pent->line_number);
                        strcat(global_buffer,lis_out_bufp);                
                }
                else
                {
                        /**************************************/
                        /* Process the Log Entry Type = Error */
                        /**************************************/

                        cnt += sprintf(lis_out_bufp, "ERROR (counter %d): %d, %s, %d\n",
                               pent->counter, pent->error_value, pent->function, pent->line_number);
                        strcat(global_buffer,lis_out_bufp);                
                }
	}

        /* Add a linefeed */
        cnt += sprintf(lis_out_bufp, "\n");
        strcat(global_buffer,lis_out_bufp);                

        /* restore the event and error logging enabled flags */
        lis_diag->event_logging = event_logging;
        lis_diag->error_logging = error_logging;

        return(cnt);
}

/* FUNCTION lis_diag_log_trace() - log a System Call Trace checkpoint   */
/*************************************************************************
								
FUNCTION
	void lis_diag_log_trace()

DESCRIPTION
	This function is called to log a System Call trace checkpoint

INPUTS
        char *func        - The function name
        int line_number   - The line number in the file
        int counter       - The counter number associated with this event

OUTPUTS
	None

*************************************************************************/
void lis_diag_log_trace(char *func, int line_number, char *process_name, int fd)
{
	lis_sc_trace_entry_t *pent;

        /*************************************************/
        /* Check if System Call Trace logging is enabled */
        /*************************************************/

        if(!lis_diag_sc_trace_log.trace_logging)
        {
                return;
        }
        
        /*******************************************/
        /* Get the next available entry in the log */
        /*******************************************/

        pent = lis_diag_get_sc_trace_entry();

        /*************************/
        /* Fill in the log entry */
        /*************************/

        strncpy (pent->function, func, MAXfunctionNAME);
        pent->line_number = line_number;
        strncpy (pent->process_name, process_name, sizeof(pent->process_name));
        pent->fd = fd;
        
        return;
}

/* FUNCTION lis_diag_get_sc_trace_entry() - get a trace log entry       */
/*************************************************************************
								
FUNCTION
	lis_sc_trace_entryy_t *lis_diag_get_sc_trace_entry()

DESCRIPTION
	This function is called to get a System Call Trace log entry.

INPUTS
	None

OUTPUTS
	Pointer to a log entry

*************************************************************************/
lis_sc_trace_entry_t *lis_diag_get_sc_trace_entry(void)
{
	lis_sc_trace_entry_t *pent;
	int index;
	struct timeval curr_time;

        /*******************************************/
        /* Get the next available entry in the log */
        /*******************************************/

        index = lis_diag_sc_trace_log.index;
        pent = &lis_diag_sc_trace_log.ent[index];

        /***********************/
        /* Timestamp the entry */
        /***********************/

        do_gettimeofday(&curr_time);

	pent->tv_sec  = curr_time.tv_sec;
	pent->tv_usec = curr_time.tv_usec;

        /********************/
        /* Update the index */
        /********************/
        
	index++;

	if(index >= LIS_DIAG_TRACE_ENTRIES)
	{
		lis_diag_sc_trace_log.wrapped = 1;
		index = 0;
	}

        /* index to next entry */
        lis_diag_sc_trace_log.index = index;

        /* return this entry */        
        return(pent);
}

/* FUNCTION lis_diag_disp_sc_trace_log() - Display System Call Trace Log */
/*************************************************************************
								
FUNCTION
	int lis_diag_disp_sc_trace_log()

DESCRIPTION
	This function is called to display the System Call Trace log.

        Note that this routine currently always displays all of the
        entries in the log.

INPUTS
	None

OUTPUTS
	Count of characters in the global buffer.

*************************************************************************/
int lis_diag_disp_sc_trace_log()
{
        lis_sc_trace_t *lis_diag = &lis_diag_sc_trace_log;
	lis_sc_trace_entry_t *pent;
	int number_of_entries;
        int trace_logging;

        int cnt = 0;
        char *lis_out_bufp = (char *)&lis_out_buf;
        char *global_buffer = (char *)&lis_output_buffer;
        
	int i;
	int ix;

        /* save the trace logging enabled flag */
        trace_logging = lis_diag->trace_logging;

        /* temporarily disable trace logging */
        lis_diag->trace_logging = 0;

        /******************************************************************/
        /* Set up the starting index and the number of entries to display */
        /******************************************************************/

        number_of_entries = LIS_DIAG_TRACE_ENTRIES;

	if((int)lis_diag->index < number_of_entries)
	{
		if(lis_diag->wrapped)
		{
                        /* start from the current index */
 			ix = lis_diag->index;
		}
		else
		{
                        /* start from the first entry */
                        ix = 0;

                        /* adjust the number of entries to display value */
			number_of_entries = lis_diag->index;
		}
	}
	else
	{
                /* start from the first entry */
		ix = 0;
	}

        cnt = sprintf(global_buffer, "\nLiS Diagnostics System Call Trace Log Display\n");
        cnt += sprintf(lis_out_bufp, "---------------------------------------------\n");
        strcat(global_buffer,lis_out_bufp);

        cnt += sprintf(lis_out_bufp, "\nKey:\nProcess Name, File Descriptor (FD)): LiS Function Name, Line Number\n");
        strcat(global_buffer,lis_out_bufp);                
        
        cnt += sprintf(lis_out_bufp, "\nStarting Index %d, Wrapped %d, Index %d\n\n",
                       ix, lis_diag->wrapped, lis_diag->index);
        strcat(global_buffer,lis_out_bufp);                

        /****************/
        /* Display Loop */
        /****************/

        for(i = 0; i < number_of_entries; i++)
	{
                /*****************************************************************/
                /* Check that the Log output isn't too big for the output buffer */
                /*****************************************************************/

                if(cnt >= (LIS_OUTPUT_BUFFER_SIZE - LIS_DIAG_MAX_OUTPUT_LINE_SIZE))
                {
                        /* Informational message which means that the output buffer */
                        /* size must be increased.  This is a compile time issue.   */
                        
                        /* Throw away the buffer content and print an error */
                        cnt = sprintf(global_buffer, "\n\nLog output too large for buffer\n");

                        printk("lis.%s: Log output too large for buffer (cnt = %d\n",
                               __func__, cnt);

                        break;
                }
                        
                /*****************************/
                /* Get an entry from the log */
                /*****************************/

		pent = &lis_diag->ent[ix];

                /* format the timestap */
                cnt += sprintf(lis_out_bufp, "%03d. [!%ld!:%03ld] ",
                               ix, (unsigned long)pent->tv_sec, ((unsigned long)pent->tv_usec/1000));
                strcat(global_buffer,lis_out_bufp);                
                
        
                cnt += sprintf(lis_out_bufp, "%s, FD %d: %s, %d\n",
                               pent->process_name, pent->fd,  pent->function, pent->line_number);
                strcat(global_buffer,lis_out_bufp);                
                
                /* increment the index and check for wrap */
                ix++;
                
		if(ix >= LIS_DIAG_TRACE_ENTRIES)
		{
                        /* wrap to the beginning */
			ix = 0;
		}
        }

        /* Add a linefeed */
        cnt += sprintf(lis_out_bufp, "\n");
        strcat(global_buffer,lis_out_bufp);                

        /* restore the event and error logging enabled flags */
        lis_diag->trace_logging = trace_logging;

        return(cnt);
}

#endif /* LIS_HAS_PROC_ENTRIES */

/*  -------------------------------------------------------------------  */
