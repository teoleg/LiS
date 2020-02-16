/*                               -*- Mode: C -*- 
 * wait.c --- wait queues management
 * Author          : Francisco J. Ballesteros
 * Created On      : Tue May 31 22:25:19 1994
 * Last Modified By: David Grothe
 * RCS Id          : $Id: wait.c,v 9.3 2010/03/31 15:07:40 hecht Exp $
 * Purpose         : keep close all the wait related stuff.
 * ----------------______________________________________________
 *
 *   Copyright (C) 1995  Francisco J. Ballesteros
 *   Copyright (C) 1997  David Grothe, Gcom, Inc <dave@gcom.com>
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
 *    nemo@ordago.uc3m.es
 */

#ident "@(#) LiS wait.c 2.25 10/01/04 15:06:54 "


/*  -------------------------------------------------------------------  */
/*				 Dependencies                            */

#include <sys/strport.h>
#include <sys/strconfig.h>	/* config definitions */
#include <sys/LiS/share.h>	/* streams shared defs*/
#include <sys/LiS/wait.h>	/* interface */
#include <sys/LiS/head.h>	/* stream head */
#include <sys/LiS/queue.h>	/* streams queues */
#include <sys/osif.h>


/*  -------------------------------------------------------------------  */
/*				   Symbols                               */

#define	AINC(v)		K_ATOMIC_INC(&(v))
#define	ADEC(v)		K_ATOMIC_DEC(&(v))

/*  -------------------------------------------------------------------  */
/*				 Global vars                             */
long	lis_wakeup_close_wt_cnt ;
long	lis_sleep_on_close_wt_cnt ;

extern lis_atomic_t	 lis_in_syscall ;	/* in head.c */

int lis_wait_for_completion_interruptible(struct completion *x)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	return(wait_for_completion_interruptible(x));
#else
	int ret = 0;

 	/* MarkS@Adax - 22 Oct 2007 - commented out might_sleep() 
 	   as it is an unresolved symbol on 2.4 kernels and is 
 	   only used for debugging on 2.6 kernels. */
 	/* might_sleep(); */

	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
			__set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			schedule();
			spin_lock_irq(&x->wait.lock);
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
out:
	spin_unlock_irq(&x->wait.lock);

	return ret;
#endif
}

int lis_wake_up_process(struct task_struct *tsk)
{
int ret;

	ret = 0;
	if (tsk != NULL)
		ret = wake_up_process(tsk);
	return(ret);
}

/*  -------------------------------------------------------------------  */
/*			Exported functions & macros                      */

/* 
 * When we are about to sleep, and we have the stream head unlocked,
 * check to see if we need to run the queues.  Often we sleep after having
 * sent something downstream to the driver, so this is a good place
 * to check for queues to run.
 */
/*  -------------------------------------------------------------------  */

int 
lis_sleep_on_close_wt(void *q_str)
{
    stdata_t	*sd = (stdata_t *) q_str ;
    int ret;

    if (sd == NULL || sd->magic != STDATA_MAGIC)
	return(-ENOENT) ;

    lis_clear_and_save_sigs(sd);		/* undo pending signals */
    lis_sleep_on_close_wt_cnt++ ;
    K_ATOMIC_DEC(&lis_in_syscall) ;		/* "done" with a system call */
    lis_runqueues();
    /* wait_for... might return -EINTR if interrupted */
    ret = lis_wait_for_completion_interruptible(&sd->sd_close_wt);
    K_ATOMIC_INC(&lis_in_syscall) ;		/* processing a system call */
    lis_restore_sigs(sd);			/* restore old signals */
    return(ret) ;

}/*lis_sleep_on_close_wt*/

/*  -------------------------------------------------------------------  */
void 
lis_wakeup_close_wt(void *q_str)
{
    stdata_t	*sd = (stdata_t *) q_str ;

    if (   sd != NULL
	&& sd->magic == STDATA_MAGIC
	&& F_ISSET(sd->sd_flag,(STRCLOSEWT|STRFLUSHWT))
       )
    {
	lis_wakeup_close_wt_cnt++ ;
	complete(&sd->sd_close_wt);
    }

}/*lis_wakeup_close_wt*/

/*  -------------------------------------------------------------------  */
int 
lis_sleep_on_wioc(struct stdata * sd, char *f,int l)
{
    int		rslt ;

    K_ATOMIC_DEC(&lis_in_syscall) ;		/* "done" with a system call */
    lis_runqueues();
    rslt = lis_down_fcn(&sd->sd_wioc,f,l);
    K_ATOMIC_INC(&lis_in_syscall) ;		/* processing a system call */
    return(rslt) ;

}/*lis_sleep_on_wioc*/

/*  -------------------------------------------------------------------  */

/*
 * Called to wait for the driver to backenable its queue and wake up
 * a waiting writer.  The stream head write queue is locked when this
 * routine is called.  We unlock it just prior to waiting on the semaphore.
 */
int 
lis_sleep_on_wwrite(struct stdata * sd)
{
int	ret ;

    AINC(sd->sd_wrcnt) ;
    lis_unlockq(sd->sd_wq) ;
    K_ATOMIC_DEC(&lis_in_syscall) ;		/* "done" with a system call */
    lis_runqueues();
    if ((ret = lis_wait_for_completion_interruptible(&sd->sd_wwrite)) < 0)
	{
	ADEC(sd->sd_wrcnt) ;			/* back out */
	}
    else if ((ret = lis_lockq(sd->sd_wq)) < 0)	/* relock queue */
	{
	ADEC(sd->sd_wrcnt) ;			/* back out */
	complete(&sd->sd_wwrite) ;		/* ZZZ?  Needed? */
	}

    K_ATOMIC_INC(&lis_in_syscall) ;		/* processing a system call */
    return(ret) ;

}/*lis_sleep_on_wwrite*/

/*  -------------------------------------------------------------------  */
/* ZZZ This function is never used.... */
void 
lis_wake_up_wwrite(struct stdata * sd)
{
    int	err ;

    if ((err = lis_lockq(sd->sd_wq)) < 0)
    {
	lis_stream_error(sd, err, err) ;
	return ;
    }

    if (K_ATOMIC_READ(&sd->sd_wrcnt) > 0)
    {
	ADEC(sd->sd_wrcnt) ;
	lis_unlockq(sd->sd_wq) ;
	complete(&sd->sd_wwrite);
    }
    else
	lis_unlockq(sd->sd_wq) ;

}/*lis_wake_up_wwrite*/

/*  -------------------------------------------------------------------  */
void 
lis_wake_up_all_wwrite(struct stdata * sd)
{
    /*
     * No queue locking, just wake up everybody and let them contend
     */
    while (K_ATOMIC_READ(&sd->sd_wrcnt) > 0)
    {
	ADEC(sd->sd_wrcnt) ;
	complete(&sd->sd_wwrite);
    }

}/*lis_wake_up_all_wwrite*/

/*  -------------------------------------------------------------------  */
int 
lis_sleep_on_wread(struct stdata * sd)
{
    int		ret ;

    AINC(sd->sd_rdcnt) ;			/* inc while q locked */
    lis_unlockq(sd->sd_rq) ;			/* unlock to let rsrv run */
    K_ATOMIC_DEC(&lis_in_syscall) ;		/* "done" with a system call */
    lis_runqueues();

    if ((ret = lis_wait_for_completion_interruptible(&sd->sd_wread)) < 0)
	{
	ADEC(sd->sd_rdcnt) ;			/* back out */
	}
    else if ((ret = lis_lockq(sd->sd_rq)) < 0)	/* relock queue */
	{
	ADEC(sd->sd_rdcnt) ;			/* back out */
	complete(&sd->sd_wread) ;
	}

    K_ATOMIC_INC(&lis_in_syscall) ;		/* processing a system call */
    return(ret) ;

}/*lis_sleep_on_wread*/

/*  -------------------------------------------------------------------  */
void 
lis_wake_up_wread(struct stdata * sd)
{
    int	err ;

    /* Check this without locking the queue first */
    if (K_ATOMIC_READ(&sd->sd_rdcnt) > 0)		/* someone is reading */
    {
	/* If there actually is someone there, lock it to make sure
	 * they don't do away.  ZZZ is this really needed?  */
	if ((err = lis_lockq(sd->sd_rq)) < 0)
		{
		lis_stream_error(sd, err, err) ;
		return ;
		}
	if (K_ATOMIC_READ(&sd->sd_rdcnt) > 0)		/* someone is reading */
		{
		ADEC(sd->sd_rdcnt) ;
		lis_unlockq(sd->sd_rq) ;
		complete(&sd->sd_wread);
		}
	else
		lis_unlockq(sd->sd_rq) ;
    }

}/*lis_wake_up_wread*/

/*  -------------------------------------------------------------------  */
void 
lis_wake_up_all_wread(struct stdata * sd)
{
    /*
     * No queue locking, just wake up everybody and let them contend
     */
    while (K_ATOMIC_READ(&sd->sd_rdcnt) > 0)
    {
	ADEC(sd->sd_rdcnt) ;
	complete(&sd->sd_wread);
    }

}/*lis_wake_up_all_wread*/

/*  -------------------------------------------------------------------  */
int 
lis_sleep_on_read_sem(struct stdata * sd)
{
    int		ret ;

    AINC(sd->sd_rdsemcnt) ;
    if ((ret = lis_down(&sd->sd_read_sem)) < 0)	/* now sleep */
	ADEC(sd->sd_rdcnt) ;

    return(ret) ;

}/*lis_sleep_on_read_sem*/
/*  -------------------------------------------------------------------  */
void 
lis_wake_up_read_sem(struct stdata * sd)
{
    if (K_ATOMIC_READ(&sd->sd_rdsemcnt) > 0)		/* someone is reading */
    {
	ADEC(sd->sd_rdsemcnt) ;
	lis_up(&sd->sd_read_sem);
    }

}/*lis_wake_up_read_sem*/
/*  -------------------------------------------------------------------  */
void 
lis_wake_up_all_read_sem(struct stdata * sd)
{
    while (K_ATOMIC_READ(&sd->sd_rdsemcnt) > 0)
    {
	ADEC(sd->sd_rdsemcnt) ;
	lis_up(&sd->sd_read_sem);
    }

}/*lis_wake_up_all_read_sem*/

/*  -------------------------------------------------------------------  */
int 
lis_sleep_on_wiocing(struct stdata * sd)
{
int ret;

    K_ATOMIC_DEC(&lis_in_syscall) ;		/* "done" with a system call */
    lis_runqueues();
    ret = lis_wait_for_completion_interruptible(&sd->sd_wiocing);
    K_ATOMIC_INC(&lis_in_syscall) ;		/* processing a system call */
    return(ret) ;

}/*lis_sleep_on_wiocing*/


/*----------------------------------------------------------------------
# Local Variables:      ***
# change-log-default-name: "~/src/prj/streams/src/NOTES" ***
# End: ***
  ----------------------------------------------------------------------*/
