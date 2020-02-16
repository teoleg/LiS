#define QUEUERUN_OPT
/*                               -*- Mode: C -*- 
 * stream.c --- STREAMS entry points and main routines 
 * Author          : Graham Wheeler, Francisco J. Ballesteros
 * Created On      : Tue May 31 22:25:19 1994
 * Last Modified By: David Grothe
 * RCS Id          : $Id: stream.c,v 9.3 2012/08/07 16:05:52 hecht Exp $
 * ----------------______________________________________________
 *
 *   Copyright (C) 1995  Francisco J. Ballesteros, Graham Wheeler,
 *                       Denis Froschauer
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
 *    You can reach us by email to any of
 *    nemo@ordago.uc3m.es, gram@aztec.co.za, 100741.1151@compuserve.com
 */

#ident "@(#) LiS stream.c 2.25 09/10/04 16:34:18 "


/*  -------------------------------------------------------------------  */
/*				 Dependencies                            */

#include <sys/strport.h>
#include <sys/stream.h>	/* this will include the whole stuff */
#include <sys/osif.h>
#include <linux/sched.h>

/*  -------------------------------------------------------------------  */
/*				    Types                                */


/*  -------------------------------------------------------------------  */
/*				  Glob. Vars                             */

volatile queue_t *lis_currq[LIS_NR_CPUS] ;
extern lis_atomic_t	lis_queues_running ;
extern lis_atomic_t	lis_runq_req_cnt ;	/* # req's to run queue runners */
extern lis_atomic_t	lis_runq_active_flags[] ;
extern volatile unsigned long	lis_runq_cnts[] ;
extern volatile unsigned long	lis_runq_srv_rpt_cnts[] ;
extern volatile unsigned long	lis_queuerun_cnts[] ;
extern lis_spin_lock_t	lis_qhead_lock ;
extern lis_spin_lock_t	lis_bc_lock ;		/* buffcall.c */
extern int		lis_down_nintr(lis_semaphore_t *sem) ;
extern void		lis_wakeup_close_wt(void *q_str) ; /* wait.c */

#if defined(CONFIG_DEV)
extern void		lis_cpfl(void *p, long a, 
				 const char *fcn, const char *f, int l);
#define CP(p,a)		lis_cpfl((p),(a),__FUNCTION__,__LIS_FILE__,__LINE__)
#else
#define lis_cpfl(a,b,c,d,e)
#define CP(p,a)		do {} while (0)
#endif


/*  -------------------------------------------------------------------  */
/*			   Local functions & macros                      */
#if !defined(__GNUC__) && !defined(HAS__FUNCTION__)
#define	__FUNCTION__	""
#endif

/*  -------------------------------------------------------------------  */

/*  -------------------------------------------------------------------  */
/* service scheduler
 *
 * Note concerning the QRUNNING flag:  There is a race between this routine and
 * the lis_qdetach routine.  lis_qdetach may be trying to close this stream on
 * another CPU while we are trying to run its service procedure on this CPU.
 * We watch the QPROCSOFF flag in the queue as a signal that this is happening
 * and do not perform queue processing when this flag is set.  Likewise,
 * lis_qdetach may be waiting for us to finish with the queue before proceeding
 * to deallocate the queue structure.  It is watching the QENAB, QSCAN and
 * QRUNNING flag.  We are careful to set QRUNNING as we are clearing either
 * QENAB or QSCAN so that lis_qdetach won't think that the queue is not in use
 * when we are still using it.  However, when we clear QRUNNING, under lock
 * protection, we must consider two cases.  If the QWAITING flag is set then
 * qdetach is trying to close the queue and is waiting on a semaphore for us to
 * finish using the queue.  In that case we need to do a wakeup.  If QWAITING
 * is not set then we just release the queue isr lock.  In that case we need to
 * be sure that this is the LAST thing we do to the queue because qdetach could
 * be executing simultaneously on another CPU, see that it does not have to
 * wait, and proceed to deallocate the queue as soon as it can get the isr
 * lock.
 *
 * Locking order:  When getting the lis_qhead_lock and the queue lock for a
 * given queue simultaneously, get the queue lock first.  This ordering is
 * dictated by lis_qenable which is called from user service procedures with
 * the queue locked, and which needs to get lis_qhead_lock to protect its
 * insertion into the global queue scheduling lists.
 *
 * The parameter qrun_thread is set to 1 when this function is called 
 * from a queue runner thread.  It is set to 0 when this function is called
 * via lis_setqsched from a user space thread.  If called from a user space
 * thread, we will only run the number of queues which are queued up when
 * we enter the routine.  We will not sit in this routine and run all
 * of the service routines which may be on the list.  This prevents us from
 * getting trapped in this routine running service routines for other thread
 * when we are called from user space.
 */
void
queuerun(int cpu_id, int qrun_thread)
{
    queue_t		*q;
    volatile queue_t    *vq ;
    stdata_t		*strmhd ;
    lis_flags_t		 psw, pswq;
    int			 procsoff ;
    unsigned int loopcnt;
    unsigned int maxloops;

    /* Process scan queue head list
     *
     * Process these one at a time.  Take the first entry off of the front of
     * the queue.  Other queue runners can take others off the queue
     * simultaneously, subject to spin lock protection.
     */
    lis_queuerun_cnts[cpu_id]++ ;
    if (LIS_DEBUG_MONITOR_MEM)
	lis_check_mem() ;

    lis_spin_lock_irqsave(&lis_qhead_lock, &psw) ;
    /* Check early to see if there is actually anything to do here and
     * bail if not.. */
    if (K_ATOMIC_READ(&lis_runq_req_cnt) == 0)
	    {
	    if (qrun_thread)
		{
		K_ATOMIC_DEC(&lis_queues_running);
		/* This is necessary for queue runner threads to ensure that we
		 * don't lose any wakeups.  lis_thread_runqueues does not
		 * set TASK_INTERRUPTIBLE at all because we have to set this
		 * under lock.  If we wait to set it in lis_thread_runqueues
		 * (as in previous versions where we called 
		 * lis_schedule_interruptible()), there is a race between 
		 * releasing this lock, receiving the wakeup from another 
		 * thread, then this thread sleeping in 
		 * lis_schedule_interruptible().
		 */
		set_current_state(TASK_INTERRUPTIBLE);
		}
	    K_ATOMIC_DEC(&lis_runq_active_flags[cpu_id]);
	    lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;
	    return;
	    }

    for (vq = lis_scanqhead, q = (queue_t *)vq;
	 q != NULL;
	 vq = lis_scanqhead, q = (queue_t *)vq
	)
    {
	lis_scanqhead = q->q_scnxt ;
	q->q_scnxt    = NULL ;
	if (q == lis_scanqtail)			/* entry at end of list */
	    lis_scanqtail = NULL ;

	K_ATOMIC_DEC(&lis_runq_req_cnt) ;	/* one less thing to do */
	lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;

	lis_currq[cpu_id] = q ;			/* for debugging */
	if (!LIS_CHECK_Q_MAGIC(q))
	{
	    printk("queuerun: scan queue is broken\n") ;
	    lis_scanqhead = NULL ;
	    lis_scanqtail = NULL ;
	    K_ATOMIC_SET(&lis_runq_req_cnt, 0) ;
	    goto scan_loop_bottom ;
	}

	LIS_QISRLOCK(q, &pswq) ;
	q->q_flag |=  QRUNNING ;		/* protects from lis_qdetach */
	q->q_flag &= ~QSCAN;
	procsoff = (q->q_flag & (QCLOSING | QPROCSOFF)) ;
	strmhd = (stdata_t *) q->q_str ;
	LIS_QISRUNLOCK(q, &pswq) ;

	if (lis_lockq(q) < 0)			/* see "locking order" above */
	{
	    LIS_QISRLOCK(q, &pswq) ;
	    q->q_flag &=  ~QRUNNING ;
	    LIS_QISRUNLOCK(q, &pswq) ;
	    goto scan_loop_bottom ;
	}

	if (   !procsoff
	    && strmhd != NULL
	    && strmhd->sd_wmsg != NULL)		/* held msg waiting */
	{
	    lis_runq_cnts[cpu_id]++ ;
	    lis_putnext(LIS_WR(q),strmhd->sd_wmsg);	/* send it */
	    strmhd->sd_wmsg = NULL ;		/* don't reuse msg */
	}

	lis_unlockq(q) ;

	LIS_QISRLOCK(q, &pswq) ;
	q->q_flag &= ~QRUNNING ;		/* see note at head of func */
	if (F_ISSET(q->q_flag, QWAITING) && q->q_wakeup_sem != NULL)
	{
	    LIS_QISRUNLOCK(q, &pswq) ;
	    lis_up(q->q_wakeup_sem) ;
	}
	else
	    LIS_QISRUNLOCK(q, &pswq) ;

        lis_head_put(strmhd) ;			/* balance ins scanq */
scan_loop_bottom:
	lis_spin_lock_irqsave(&lis_qhead_lock, &psw) ; /* for loop iteration */
    }

    lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;

    lis_spin_lock_irqsave(&lis_bc_lock, &psw) ;
    if (lis_strbcflag)
	{
	lis_runq_cnts[cpu_id]++ ;
	lis_spin_unlock_irqrestore(&lis_bc_lock, &psw) ;
	K_ATOMIC_DEC(&lis_runq_req_cnt) ;	/* one less thing to do */
	lis_dobufcall(cpu_id);
	}
    else
      lis_spin_unlock_irqrestore(&lis_bc_lock, &psw) ;

    /*
     * This is the main queue list.  Process one at a time so that other
     * threads can do the same simultaneously.
     */
    lis_spin_lock_irqsave(&lis_qhead_lock, &psw) ;
    maxloops = K_ATOMIC_READ(&lis_runq_req_cnt);
    for (vq = lis_qhead, q = (queue_t *)vq, loopcnt=0;
	 q != NULL;
	 vq = lis_qhead, q = (queue_t *)vq
        )
    {
	lis_qhead = q->q_link ;			/* remove it from the list */
	q->q_link = NULL ;
	if (q == lis_qtail)			/* entry at end of list */
	    lis_qtail = NULL ;

	K_ATOMIC_DEC(&lis_runq_req_cnt) ;	/* one less thing to do */
	lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;

	lis_currq[cpu_id] = q ;			/* for debugging */
	if (!LIS_CHECK_Q_MAGIC(q))
	{
	    printk("queuerun: service queue is broken\n") ;
	    lis_qhead = NULL ;
	    lis_qtail = NULL ;
	    K_ATOMIC_SET(&lis_runq_req_cnt, 0) ;
	    goto qsched_loop_bottom ;
	}

	LIS_QISRLOCK(q, &pswq) ;
	CP(q->q_str, q->q_flag) ;
	q->q_flag |=  QRUNNING ;		/* protects from lis_qdetach */
	q->q_flag &= ~QENAB;			/* allow qenable again */
	procsoff = (q->q_flag & QPROCSOFF) ;	/* queue procs off */
	strmhd = (stdata_t *) q->q_str ;
	LIS_QISRUNLOCK(q, &pswq) ;

	if (lis_lockq(q) < 0)			/* see "locking order" above */
	{
	    LIS_QISRLOCK(q, &pswq) ;
	    q->q_flag &=  ~QRUNNING ;
	    LIS_QISRUNLOCK(q, &pswq) ;
	    goto qsched_loop_bottom ;
	}

	LisDownCount(QSCHEDS) ;

	/* call its service procedure, if any */
	lis_runq_cnts[cpu_id]++ ;		/* count qrun for this cpu */
	if (   !procsoff
	    && q->q_qinfo != NULL
	    && q->q_qinfo->qi_srvp		/* has svc proc */
	   )
	{
	    CP(strmhd, (long)q) ;
#ifdef QUEUERUN_OPT
doitagain:
#endif
	    (*q->q_qinfo->qi_srvp)(q);		/* enter svc proc w/q locked */
#ifdef QUEUERUN_OPT
/* ZZZ Adax: optimization, if the service routine is still
 *  enabled, then just run it again here while the cache is hot...
 */
	    LIS_QISRLOCK(q, &pswq) ;
#ifdef DANDEBUG
	    q->reenable_cnt = 0;
#endif
	    if (F_ISSET(q->q_flag, QWANTENAB) &&     /* enabled while running */
                (!(q->q_next) ||        /* and we're at the end of the stream */
					/* or the next queue is not already 
					 * full if the next queue is full, 
					 * then we just let the qenable below 
					 * get it. */
                !(q->q_next->q_flag & (QFULL|QWASFULL))))  
		{
	        q->q_flag &= ~QWANTENAB;
		LIS_QISRUNLOCK(q, &pswq) ;
		lis_runq_srv_rpt_cnts[cpu_id]++ ;		/* keep statistics */
		goto doitagain;
		}
	    else
		LIS_QISRUNLOCK(q, &pswq) ;
#endif

	    if (lis_own_spl())			/* driver did not do splx */
	    {
		printk("queuerun: STREAMS driver %s returned from service "
			"procedure with splstr locked\n",
			lis_queue_name(q)) ;
		do
		    lis_splx(psw) ;		/* attempt to undo */
		while (lis_own_spl()) ;
	    }
	}

	lis_unlockq(q) ;

	LIS_QISRLOCK(q, &pswq) ;
	CP(q->q_str, q->q_flag) ;
	q->q_flag &= ~QRUNNING ;		/* see note at head of func */
	if (F_ISSET(q->q_flag, QWAITING) && q->q_wakeup_sem != NULL)
	{
	    CP(strmhd, (long)q) ;
	    LIS_QISRUNLOCK(q, &pswq) ;		/* release b4 wakeup */
	    lis_up(q->q_wakeup_sem) ;	/* safe because qdetach is waiting */
	}
	else
	if (F_ISSET(q->q_flag, QWANTENAB))	/* enabled while running */
	{
	    CP(strmhd, (long)q) ;
	    LIS_QISRUNLOCK(q, &pswq) ;
	    lis_retry_qenable(q) ;		/* reschedule the queue */
	}
	else
	{
	    CP(q->q_str, q->q_flag) ;
	    LIS_QISRUNLOCK(q, &pswq) ;		/* last touch of q */
	}

	/*
	 * Balance the head_get done by qenable if this is a stream
	 * head queue.
	 */
	if (   strmhd->magic == STDATA_MAGIC
	    && (strmhd->sd_wq == q || strmhd->sd_rq == q)
	   )
	    lis_head_put(strmhd) ;

qsched_loop_bottom:
	lis_spin_lock_irqsave(&lis_qhead_lock, &psw) ;	/* for loop */

	/* Dont' let this loop run endlessly for user space processes
	 * just process the number of queues that were ready when we entered 
	 * this loop at a maximum */
	if (!qrun_thread && (++loopcnt == maxloops))
		{
		K_ATOMIC_DEC(&lis_runq_active_flags[cpu_id]) ;
		lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;
		return;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14)
        
	/* On large MP machines we can get caught in this loop for a long
	 * time if other CPUs are continuously adding new service routines
	 * to the list.  We only need to tickle this at least once every
	 * ten seconds so every 1000 queue runs should be more than enough */
	if ((loopcnt % 1024) == 0)
		touch_softlockup_watchdog();

#endif

    }

    /*
     * Just in case lis_runq_req_cnt gets over-incremented, check to see if all
     * queuerun conditions are now false.  If so set lis_runq_req_cnt back
     * to zero so that we don't loop indefinitely through here.  We will also
     * correct any decrements through zero here.  We do this while holding the
     * lis_qhead_lock.  All incr/decrs of lis_runq_req_cnt are protected by
     * this lock so we know that it won't change while we evaluate conditions.
     */
    if (   K_ATOMIC_READ(&lis_runq_req_cnt) != 0
	&& lis_scanqhead == NULL && lis_qhead == NULL && lis_strbcflag == 0
       )
	{
	printk("LIS: queuerun: bad lis_runq_req_cnt %d! Setting to 0...\n", 
		K_ATOMIC_READ(&lis_runq_req_cnt));
	K_ATOMIC_SET(&lis_runq_req_cnt, 0) ;
	}

    if (qrun_thread)
	{
	/* Only decrement lis_queues_running if called from a queue 
	 * runner thread not from a user space thread */
	K_ATOMIC_DEC(&lis_queues_running);

	/* This is necessary for queue runner threads to ensure that we
	 * don't lose any wakeups.  Otherwise there is a race between 
	 * releasing this lock, receiving the wakeup from another thread,
	 * then this thread sleeping in lis_schedule_interruptible().
	 */
	set_current_state(TASK_INTERRUPTIBLE);
	}
    K_ATOMIC_DEC(&lis_runq_active_flags[cpu_id]) ;

    lis_spin_unlock_irqrestore(&lis_qhead_lock, &psw) ;
}/*queuerun*/
