/************************************************************************
*									*
*                   Gcom DLPI <-> Linux SKBUF Driver		        *
*									*
*************************************************************************
*									*
*                  Copyright (C) 1995,1996,1997 Gcom, Inc		*
*									*
*									*
*	Author: Among others Mikel L. Matthews Gcom, Inc.		*
*									*
* Copyright (C) 1997-1999  Mikel Matthews, Gcom, Inc <mikel@gcom.com>	*
*                          Dave Grothe, Gcom, Inc <dave@gcom.com>	*
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
 * This is a pushable streams module that bridges a DLPI stream to
 * an IP masquerading as a network driver.
 *
 * The way you use it is as follows:
 *
 * Open your DLPI stream file.
 *
 * Push this module.
 *
 * Do DLPI attach and bind.
 *
 * Send SIOCSIFNAME ioctl with the desired interface name.
 *
 * The ioctl causes the module to register itself with IP as a network
 * interface driver.
 *
 * Do the appropriate ifconfig and route commands.
 *
 * You can tear down the IP side by doing ifconfig <name> off.  You can
 * tear down the whole thing by closing the streams file.
 */


/************************************************************************
*                              SCCS ID                                  *
*************************************************************************
*									*
* The following strings identify this module as to version info.	*
*									*
************************************************************************/

#ident "@(#) LiS ip_strm_mod.c 2.27 09/13/04 10:12:45 "

#define SAFE

/*
 * C language includes
 */

/*
 * UNIX includes
 */
#include <sys/stream.h>
#include <sys/stropts.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <sys/cmn_err.h>
#include <linux/errno.h>
#include <sys/dlpi.h>
#include <sys/npi.h>
//#include <sys/tihdr.h>
#include <linux/netdevice.h>
#include <net/dst.h>
#include <net/route.h>
#include <linux/ip.h>
#include <linux/route.h>

/*
 * These interfere with sock.h in 2.1 kernel
 */
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

/*
 * ASSERT, which we don't use, interferes with irda.
 */
#ifdef ASSERT
#undef ASSERT
#endif
 /*
  * Likewise queue_t
  */
#define queue_t	irda_queue_t

#include <linux/termios.h>
#include <linux/in.h>

#  include <sys/mkdev.h>
#  include <sys/ddi.h>
# ifdef RH_71_KLUDGE	     /* boogered up incls in 2.4.2 */
#  undef CONFIG_HIGHMEM	     /* b_page has semi-circular reference */
# endif
#include <linux/skbuff.h>


/* #include <linux/config.h> */
#include <linux/kernel.h>
#include <linux/param.h>
/*
#include <linux/sched.h>
*/
#include <linux/interrupt.h>
#include <linux/kconfig.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/if.h>

#include <asm/system.h>
#include <asm/io.h>

#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/sock.h>
#include <linux/if_ether.h>     /* For the statistics structure. */
#include <linux/if_arp.h>       /* For ARPHRD_ETHER */
#include <sys/cred.h>

#undef queue_t			/* allow visibility again */
#include <sys/stream.h>

#include <sys/LiS/mod.h>
#include "ip_strm_mod.h"
#include <sys/osif.h>

/************************************************************************
*                        Function Prototypes                            *
************************************************************************/

/*
 * Functions in this file
 */
void ip_strm_init(struct ism_dev *dev) ;
int convert_to_skbuf(ip_to_streams_minor_t *, mblk_t *);
int convert_to_buf(ip_to_streams_minor_t *minor_ptr, mblk_t *mp);

/*

 * The ip_strm_mod module has been enhanced to bridge a DLPI stream to
 * a Linux ppp subsytem and support running Linux PPP or MLPPP
 * over STRAMS-based drivers, like Adax HDC or ATMIII driver.
 *
 * The way you use it is as follows:
 *
 * Open your DLPI stream file.
 *
 * Push this module.
 *
 * Do DLPI attach and bind.
 *
 * Send SIOCSIFLINK ioctl with the desired interface name.
 *
 * The ioctl causes the module to register itself with Linux PPP subsystem
 * as a pseudo synchronized tty device.
 *
 * Do the appropriate linux pppd initialization.
 *
 * You can unregister the module by SIOCIFLINK and 
 * tear down the whole thing by closing the streams file.

			jieyan@adax.com		Nov 22,2006
*/

/* Routine used to transmit data over pstty device */ 
extern int pstty_xmit(struct pstty_dev *pstty,
                        const unsigned char *buf,int count);
/* Routine used to pass ioctl to pstty device */
extern int pstty_ioctl(struct pstty_dev *pstty,
                        unsigned int cmd, unsigned long arg);



/************************************************************************
*                    Streams Entry Point Routines                       *
*************************************************************************
*									*
* The following section of code contains the entry point routines for	*
* the streams driver.  These are the open, close, put and service	*
* routines pointed to by the qinit structures above.			*
*									*
************************************************************************/

extern  int	_RP ip_to_streams_open(queue_t *,dev_t *,int ,int, cred_t *);

extern  int	_RP ip_to_streams_close(queue_t *, int, cred_t *);
extern  int	_RP ip_to_streams_wput(queue_t *,mblk_t *);
extern  int	_RP ip_to_streams_wsrv(queue_t *);
extern  int	_RP ip_to_streams_rput(queue_t *,mblk_t *);
extern  int     _RP ip_to_streams_rsrv(queue_t *);
extern  int	ip_to_streams_conn_req(ip_to_streams_minor_t *minor_ptr,
				      mblk_t *mp, int retry) ;
extern int	ip_to_streams_proto(ip_to_streams_minor_t *, mblk_t *,
					int );

#ifdef GCOM
extern  void	netman_hex_mp (mblk_t *, char *);
extern  void   	Rsys_print_traced_token (char *bufp) ;
extern	void	Rsys_hex_print(char *, unsigned char *, int);
#endif

/************************************************************************
*                    Linkage to Streams System                          *
************************************************************************/

struct module_info ip_to_streams_minfo =
	{ 0		/* mi_idnum  */
	, "ip_to_streams"	/* mi_idname */
	, 0		/* mi_minpsz */
	, INFPSZ	/* mi_maxpsz */
	, 20000		/* mi_hiwat  */
	, 2000		/* mi_lowat  */
	};
struct qinit ip_to_streams_rinit =
	{ ip_to_streams_rput
	, ip_to_streams_rsrv	/* qi_srvp  */
	, ip_to_streams_open	/* qi_open  */
	, ip_to_streams_close	/* qi_close */
	, NULL			/* qi_admin */
	, &ip_to_streams_minfo	/* qi_minfo */
	, NULL		/* qi_mstat */
	};
struct qinit ip_to_streams_winit =
	{ ip_to_streams_wput	/* qi_putp  */
	, ip_to_streams_wsrv	/* qi_srvp  */
	, NULL		/* qi_open  */
	, NULL		/* qi_close */
	, NULL		/* qi_admin */
	, &ip_to_streams_minfo	/* qi_minfo */
	, NULL		/* qi_mstat */
	};

struct streamtab ip_to_streams_info =
	{ &ip_to_streams_rinit	   /* read queue definition */
	, &ip_to_streams_winit	   /* write queue definition */
	, NULL			   /* mux read queue  */
	, NULL			   /* mux write queue */
	};

int	ip_to_streamsdevflag = 0;

/************************************************************************
*                       Storage Declarations                            *
************************************************************************/

unsigned long		 ip_to_streams_debug_mask = 0x0;



#define IP_STRM_MTU (1500)

/************************************************************************
*                           ip_to_streams_open                          *
*************************************************************************
*									*
* The streams open routine. Called when this is pushed on to the stack	*
*									*
************************************************************************/
int _RP
ip_to_streams_open( queue_t *rdq,
	   dev_t   *devp,
	   int      flag,
	   int      sflag,
	   cred_t  *credp)
{
    ip_to_streams_minor_t	*minor_ptr ;
    struct net_device *ndev;

    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	cmn_err(CE_CONT, "ip_to_streams_open: q=%x sflag=%d\n", sflag) ;

    ndev = alloc_netdev(sizeof(ip_to_streams_minor_t), "is", ip_strm_init);
    if (!ndev)
	{
	cmn_err(CE_CONT, "ip_to_streams_open: failed to allocate etherdev privsize=%d\n", 
		sizeof(ip_to_streams_minor_t)) ;
	return(-ENODEV);
	}

/* ### TEMP */
#if defined(KERNEL_2_5)
    minor_ptr = netdev_priv(ndev);
#else
    minor_ptr = (ip_to_streams_minor_p) ALLOC(sizeof(ip_to_streams_minor_t)) ;
    if (minor_ptr == NULL)
	return(-ENOMEM) ;

    memset(minor_ptr, 0, sizeof(*minor_ptr)) ;	/* clear to zero */
#endif
    
    minor_ptr->dl_magic = DL_MAGIC ;
    rdq->q_ptr = (caddr_t) minor_ptr ;
    WR(rdq)->q_ptr = (caddr_t) minor_ptr ;

    minor_ptr->dl_rdq = rdq ;
    minor_ptr->dl_wrq = WR(rdq) ;
    minor_ptr->dl_q   = rdq;
    minor_ptr->dl_err_prim = -1 ;		/* ensure no retry */
    minor_ptr->mydev = ndev;
    strcpy(minor_ptr->myname, "is") ;		/* initial name Ip/Streams */

   /* Initialize the entry point of Pseudo Sync tty device */
    minor_ptr->ttydev.write = pstty_xmit;
    minor_ptr->ttydev.ioctl = pstty_ioctl;
    minor_ptr->ttydev.priv = minor_ptr ;
   /* Mark the pstty is idle */
    minor_ptr->ttydev.index = MAX_PSTTY_DEVICES+1 ;

    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	cmn_err(CE_CONT, "ip_to_streams_open succeeded\n") ;

    return (0);			/* success */

} /* ip_to_streams_open */

/************************************************************************
*                          ip_to_streams_close                          *
*************************************************************************
*									*
* Called when closing the stream					*
*									*
************************************************************************/

int _RP ip_to_streams_close(queue_t *q, int dummy, cred_t *credp)
{
    ip_to_streams_minor_t	*minor_ptr ;

    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	cmn_err(CE_CONT, "\nip_to_streams_close: q=%x ", q) ;

    minor_ptr = (ip_to_streams_minor_t *) q->q_ptr ;

    if (   minor_ptr != (ip_to_streams_minor_t *) NULL
	&& minor_ptr->dl_magic == DL_MAGIC
       )
    {
	if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	    cmn_err(CE_CONT, "ip_to_streams_close: %s\n", minor_ptr->myname) ;

	if (minor_ptr->dev_registered != 0)	/* still open to IP */
	    unregister_netdev(minor_ptr->mydev) ;

	minor_ptr->dl_magic = ~DL_MAGIC ;
	free_netdev(minor_ptr->mydev);
    }
    else
    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	cmn_err(CE_CONT, "ip_to_streams_close: invalid minor ptr q_ptr=%x\n",
			q->q_ptr) ;

    q->q_ptr = NULL ;				/* zot the q ptrs */
    WR(q)->q_ptr = NULL ;			/* zot the q ptrs */

    return(0) ;

} /* ip_to_streams_close */


/************************************************************************
*                          ip_to_streams_ioctl                          *
*************************************************************************
*									*
* Have a look at an ioctl received from the user.  Return 1 if the	*
* ioctl is to be forwarded downstream.  Return 0 if we handled it here.	*
*									*
************************************************************************/
int ip_to_streams_ioctl(queue_t *q, mblk_t  *mp)
{
    struct iocblk		*iocp;
    ip_to_streams_minor_t	*minor_ptr ;
    mblk_t			*xmp = mp->b_cont ;
    int				 result = 0 ;

    iocp = (struct iocblk *) mp -> b_rptr;      /* the request header */
    if ((mp -> b_wptr - mp -> b_rptr) < sizeof (struct iocblk *))   /* small */
	return(1) ;			/* let driver deal with it */

    minor_ptr = (ip_to_streams_minor_t *) q->q_ptr ;
    switch (iocp -> ioc_cmd)
    {
    case SIOCSIFNAME:		/* set interface name */
	{
        /*
         * This ioctl is sent by the streams user.  It is the last step
         * in making this stream available to IP.  Once we know the
         * name we register the device with IP as an interface.
         *
         * As an added feature, the streams user can set the name
         * to the empty string and we will unregister from IP.  When
         * we unregister we leave the streams side alone so that a
         * simple "set if-name" ioctl will make the interface appear
         * under IP again.
         */
	if (xmp != NULL && *xmp->b_rptr != 0)		/* name specified */
		{
		strncpy(minor_ptr->myname, xmp->b_rptr, 
			sizeof(minor_ptr->myname)) ;
		strcpy(minor_ptr->mydev->name, minor_ptr->myname) ;
		if ((result = register_netdev(minor_ptr->mydev)) != 0)
			printk("ip_to_streams_ioctl: "
			       "register_netdev(%s) failed: %d\n",
			       minor_ptr->myname, result);
		else
			{
			netif_start_queue(minor_ptr->mydev) ;
			if ( ip_to_streams_debug_mask & (DBG_OPEN) )
				printk("ip_to_streams_ioctl: SIOCSIFNAME: %s\n",
					minor_ptr->myname) ;
			}

		minor_ptr->dev_registered = (result == 0);
		}
	else				/* nullify name, unregister */
		{
		unregister_netdev(minor_ptr->mydev) ;
		strcpy(minor_ptr->myname, "is") ;
		minor_ptr->contype = 0;	/* not connected */
		minor_ptr->ip_open = 0 ;	/* not open to IP now */
		}

	if (xmp != NULL)
		{
		mp->b_cont = NULL ;
		freemsg(xmp) ;		/* don't return any data */
		}
	}
	break;

       case SIOCSIFLINK:		/* set interface name */
	{
        /*
         * This ioctl will be called by the streams user to bind 
	 pstty device.
	*/
	if (xmp != NULL && *xmp->b_rptr <MAX_PSTTY_DEVICES)/* PS IF specified */
	{
	    if(minor_ptr->ttydev.index <MAX_PSTTY_DEVICES)
	    {
		printk("ip_to_strms_ioctl: already registered to pstty %d, please unregister first.\n",
			minor_ptr->ttydev.index);
		result=-EBUSY;
	    }
	    else
	    {
	       minor_ptr->ttydev.index =*(xmp->b_rptr);
	       if(lis_pstty_register != NULL)
	       {
	         if ((result = (*lis_pstty_register)(&minor_ptr->ttydev)) != 0)
	         {
		   printk("ip_to_streams_ioctl: "
		       "pstty_register index=0x%x failed: %d\n",
		       minor_ptr->ttydev.index, result);
		   result = -EBUSY ;
	         }
	         else
	         {
		/* Mark the module is connected to PSTTY */
		  minor_ptr->contype=PSTTY;
		  printk("ip_to_streams_ioctl: pstty_register succeed.index=0x%x\n",
			minor_ptr->ttydev.index) ;
	         }
	       }
	       else
	       {
		 printk("ip_to_streams_ioctl: The pstty driver is unloaded.\n");
		 result = -EINVAL ;
	       }
	    }
	}
	else				/* nullify name, unregister */
	{
	    if(lis_pstty_unregister != NULL)
	    {
	      if((result=(*lis_pstty_unregister)(&minor_ptr->ttydev))!=0)
	      {
		printk("ip_to_streams_ioctl: failed in pstty_unregister.index=0x%x \n",
			minor_ptr->ttydev.index);
		result=-EIO;
	      }
	      else
	      {
	        /* Mark that the pstty device is idle */
	        minor_ptr->ttydev.index =*(xmp->b_rptr);
	        minor_ptr->contype = 0;	/* not connected */
	        minor_ptr->ip_open = 0 ;	/* not open to IP now */
	      }
	     }
	    else
	    {
		 printk("ip_to_streams_ioctl: The pstty driver is unloaded.\n");
		 result = -EINVAL ;
	    }
	}

	if (xmp != NULL)
	{
	    freemsg(xmp) ;		/* don't return any data */
	    mp->b_cont = NULL ;
	}
	break ;
	}


	break ;
    default:
	printk("ip_to_streams_ioctl: undefined ioctl: 0x%x\n", iocp->ioc_cmd) ;
	result = -EINVAL ;
	break ;
    }

    /*
     * If you exit the switch then we are going to reply to the
     * ioctl and not forward it downstream.
     */
    if (result)						/* error */
    {
	mp -> b_datap -> db_type = M_IOCNAK;
	iocp -> ioc_count = 0;                          /* no data */
    }
    else                                                /* success */
    {
	mp -> b_datap -> db_type = M_IOCACK;
	if ((xmp = mp -> b_cont) == NULL)               /* no chained buf */
	    iocp -> ioc_count = 0;                      /* no data */
	else                                            /* a response */
	    iocp -> ioc_count = xmp -> b_wptr - xmp -> b_rptr;
    }

    iocp -> ioc_error = result;
    qreply(q, mp);
    return(0) ;				/* do not forward */

} /* ip_to_streams_ioctl */

/************************************************************************
*                           ip_to_streams_wput                          *
*************************************************************************
*									*
* Write side put routine						*
*									*
************************************************************************/

int _RP ip_to_streams_wput(queue_t *q, mblk_t  *mp)
{
    ip_to_streams_minor_t	*minor_ptr ;

    if ( ip_to_streams_debug_mask & DBG_PUT )
	cmn_err(CE_CONT, "\nip_to_streams_wput: q=%x mp=%x\n", q, mp) ;

    minor_ptr = (ip_to_streams_minor_t *) q->q_ptr ;

    /* make sure the data structure is valid */
    if (   minor_ptr == (ip_to_streams_minor_t *) NULL
	|| minor_ptr->dl_magic != DL_MAGIC
       )
    {
	if (ip_to_streams_debug_mask & DBG_SQUAWK)
	    cmn_err(CE_NOTE, "ip_to_streams_wput: bad minor") ;

	freemsg(mp) ;
	return(0) ;
    }

    switch(mp->b_datap->db_type)
    {
    case M_DATA:
	if ( canputnext(q) )		/* data uses flow control */
	    putnext(q, mp);
	else
	    putqf(q, mp);
	break;

    case M_PROTO:
    case M_PCPROTO:
    {
	int err;
	union DL_primitives *dlp;

	err = 0;
	dlp = (union DL_primitives *)mp->b_rptr;

	switch(dlp->dl_primitive)
	{
	    case DL_UNITDATA_REQ:
	    {
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("ip_to_streams_wput: DL_UNITDATA_REQ:\n");

		if ( canputnext(q) )		/* data uses flow control */
		    putnext(q, mp);
		else
		    putqf(q, mp);
		return(0) ;
	    }

	    case DL_INFO_REQ:
	    {
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("ip_to_streams_wput: DL_INFO_REQ:\n");

		putnext(q, mp);
		break;
	    }

	    case DL_BIND_REQ:
	    {
		dl_bind_req_t	     *dlp;

		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("ip_to_streams_wput:DL_BIND_REQ:\n");

		minor_ptr->dlstate = DL_BIND_PENDING;

		dlp = (dl_bind_req_t *) mp->b_rptr ;
		minor_ptr->dl_sap = dlp->dl_sap ;	/* save SAP */

		putnext(q, mp);
		break;
	    }

	    case DL_UNBIND_REQ:
	    {
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("DL_UNBIND_REQ:\n");
		minor_ptr->dlstate = DL_UNBIND_PENDING;
		minor_ptr->dl_sap = 0 ;

		putnext(q, mp);
		break;
	    }

	    case DL_ATTACH_REQ:
	    {
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("DL_ATTACH_REQ:\n");

		minor_ptr->dlstate = DL_ATTACH_PENDING;
		putnext(q, mp);

		break;
	    }

	    case DL_DETACH_REQ:
	    {
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("DL_DETACH_REQ:\n");

		minor_ptr->dlstate = DL_DETACH_PENDING ;
		putnext(q, mp);
		break;
	    }
	    break;
	}
	break;
    }

    case M_IOCTL:
	if (ip_to_streams_ioctl(q, mp))		/* forward */
	    putnext(q, mp);
	break;

    case M_FLUSH:
	if (*mp->b_rptr & FLUSHW)
	{
	    flushq(q, FLUSHDATA);
	}

	if (*mp->b_rptr & FLUSHR)
	{
	    flushq(RD(q), FLUSHDATA);
	    *mp->b_rptr &= ~FLUSHW;
	    qreply(q, mp);
	}
	else
	    freemsg(mp);

	break;

    default:
	freemsg(mp) ;
	break;

    } /* switch */

    return(0) ;

} /* ip_to_streams_wput */


/************************************************************************
*                        ip_to_streams_allocb                           *
*************************************************************************
*									*
* Allocate a STREAMS buffer.  Trigger retry mechanism if fail.		*
*									*
* Parameters:								*
*	q	The queue to use for retries.				*
*		NULL q defeats retry procedure.				*
*	mp	The message to insert into the queue if retrying.	*
*		A NULL mp means nothing to insert into the queue	*
*		but the retry occurs anyway.				*
*	type	The type of message to allocate (M_PROTO, ...)		*
*	size	The size of the STREAMS buffer to allocate.		*
*	priority The buffer allocation priority.			*
*	head	Queue the buffer at the head of the queue, not tail.	*
*	bufcall_id Pointer to integer where bufcall id is to be stored.	*
*									*
* Returns:								*
*	ptr	If buffer was allocated.				*
*	NULL	If buffer not allocated.  bufcall() has been called	*
*		and the 'mp' has been placed into the 'q'.		*
*									*
************************************************************************/
mblk_t *ip_to_streams_allocb(queue_t		*q,
		    mblk_t		*mp,
		    int   		 type,
		    int    		 size,
		    int     		 priority,
		    int	    		 head,
		    int			*bufcall_id)
{
    mblk_t	*msg ;

    if ( ip_to_streams_debug_mask & DBG_ALLOCB )
	cmn_err(CE_CONT, "\nip_to_streams_allocb: called\n");

    msg = allocb(size, priority) ;

    if (msg != (mblk_t *) NULL)
    {						/* buffer allocated */
	msg->b_datap->db_type = type ;
	return(msg) ;
    }
    return((mblk_t *) NULL) ;

} /* ip_to_streams_allocb */

/************************************************************************
*                          ip_to_streams_merror                         *
*************************************************************************
*									*
* Send an M_ERROR upstream.						*
*									*
* We will use the mp given to us if we can.				*
*									*
************************************************************************/
int
ip_to_streams_m_error(ip_to_streams_minor_t *minor_ptr, mblk_t *mp, int errno, int retry)
{
    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	cmn_err(CE_CONT, "\nip_to_streams_m_error: called\n");

    if (   minor_ptr == (ip_to_streams_minor_t *) NULL)
    {
	if (ip_to_streams_debug_mask & DBG_SQUAWK)
	    cmn_err(CE_NOTE, "ip_to_streams_m_error: bad or detached minor %x",
		    minor_ptr) ;

	if (mp != (mblk_t *) NULL) freemsg(mp) ;
	return(1) ;				/* message "sent" */
    }

    flushq(minor_ptr->dl_rdq, FLUSHALL);	/* ensure queue empty */
    minor_ptr->dl_retry_proto = 0 ;		/* TLI proto to retry */
    minor_ptr->dl_err_prim = -1 ;		/* for error ack retry */
    minor_ptr->dlstate = 0 ;			/* go to null state */

    if (mp == (mblk_t *) NULL)
    {
	mp = ip_to_streams_allocb(minor_ptr->dl_wrq,
			 mp,
			 M_ERROR,
			 1,
			 BPRI_HI,
			 retry,
			 &minor_ptr->dl_bufcall_id) ;

	minor_ptr->dl_m_error  = errno | 0x100 ;	/* set retry flag */
	if (mp == (mblk_t *) NULL) return(0) ;		/* deferred */
    }
    else
    {
	mp->b_datap->db_type = M_ERROR ;
	mp->b_rptr = mp->b_wptr = mp->b_datap->db_base ;
    }

    *mp->b_wptr++ = errno ;			/* plant error code */

    minor_ptr->dl_m_error  = 0 ;		/* unset retry flag */

#ifdef GCOM
    if ( ip_to_streams_debug_mask & (DBG_UPR_PROTOS | DBG_MERROR) )
	rsys_decode_msg(rsys_nextq_name(minor_ptr->dl_rdq),
			"ip_to_streams_m_error: To",
			mp,
			0) ;
#endif

    putnext(minor_ptr->dl_rdq, mp);
    return(0);
} /* ip_to_streams_merror */


/************************************************************************
*                          ip_to_streams_error_ack                      *
*************************************************************************
*									*
* Send an error_ack upstream.						*
*									*
* Return 1 if succeed, 0 if deferred.					*
*									*
************************************************************************/
int
ip_to_streams_error_ack(ip_to_streams_minor_t *minor_ptr,
	       mblk_t       *mp,
	       long	     err_prim,
	       long          tli_error,
	       long          unix_error,
	       int	     retry)
{
    dl_error_ack_t		*p ;

    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	cmn_err(CE_CONT, "\nip_to_streams_error_ack: called\n");

    if (   mp == (mblk_t *) NULL
	|| mp->b_datap->db_lim - mp->b_datap->db_base < sizeof(*p)
	|| (!retry && (ip_to_streams_debug_mask & DBG_ALLOCB))
       )
    {					/* must allocate a buffer */
	if (mp != (mblk_t *) NULL) freemsg(mp) ;
	mp = ip_to_streams_allocb(minor_ptr->dl_rdq,
			 (mblk_t *) NULL,
			 M_PCPROTO,
			 sizeof(*p),
			 BPRI_MED,
			 retry,
			 &minor_ptr->dl_bufcall_id) ;
	if (mp == (mblk_t *) NULL)
	{
	    minor_ptr->dl_err_prim = err_prim ;
	    minor_ptr->dl_tli_err  = tli_error ;
	    minor_ptr->dl_unix_err = unix_error ;
	    return(0) ;				/* deferred */
	}
    }
    else
    {
	mp->b_datap->db_type = M_PCPROTO ;
	mp->b_rptr = mp->b_wptr = mp->b_datap->db_base ;
    }

    minor_ptr->dl_err_prim = -1 ;		/* ensure no retry */

    /*
     * Have a buffer, build error_ack and send it
     */
    mp->b_rptr = mp->b_datap->db_base ;
    mp->b_wptr = mp->b_datap->db_base ;

    p             = (dl_error_ack_t *) mp->b_wptr ;
    p->dl_primitive       = DL_ERROR_ACK ;
    p->dl_error_primitive = err_prim ;
    p->dl_errno           = tli_error ;
    p->dl_unix_errno      = unix_error ;
    mp->b_wptr   += sizeof(*p) ;

    putnext(minor_ptr->dl_rdq, mp);
    return(1) ;					/* succeed */

} /* ip_to_streams_error_ack */

/************************************************************************
*                         ip_to_streams_ok_ack	                        *
*************************************************************************
*									*
* Send an ok_ack upstream.						*
*									*
* Return 1 if succeed, 0 if deferred.					*
*									*
************************************************************************/
int
ip_to_streams_ok_ack(ip_to_streams_minor_t *minor_ptr,
	       mblk_t    *mp,
	       long	  ok_prim,
	       int	  retry)
{
    dl_ok_ack_t		*p ;

    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	cmn_err(CE_CONT, "\nip_to_streams_ok_ack: called\n");

    if (   minor_ptr == (ip_to_streams_minor_t *) NULL
	|| minor_ptr->dl_magic != DL_MAGIC
       )
    {
	if (ip_to_streams_debug_mask & DBG_SQUAWK)
	    cmn_err(CE_NOTE, "ip_to_streams_ok_ack: bad minor") ;

	if (mp != (mblk_t *) NULL) freemsg(mp) ;
	return(1) ;
    }

    if (   mp == (mblk_t *) NULL
	|| mp->b_datap->db_lim - mp->b_datap->db_base < sizeof(*p)
	|| (!retry && (ip_to_streams_debug_mask & DBG_ALLOCB))
       )
    {					/* must allocate a buffer */
	if (mp != (mblk_t *) NULL) freemsg(mp) ;
	mp = ip_to_streams_allocb(minor_ptr->dl_rdq,
			 (mblk_t *) NULL,
			 M_PCPROTO,
			 sizeof(*p),
			 BPRI_MED,
			 retry,
			 &minor_ptr->dl_bufcall_id) ;
	if (mp == (mblk_t *) NULL)
	{
	    minor_ptr->dl_err_prim = ok_prim ;
	    minor_ptr->dl_tli_err  = 0 ;
	    minor_ptr->dl_unix_err = 0 ;
	    return(0) ;				/* deferred */
	}
    }
    else
    {
	mp->b_datap->db_type = M_PCPROTO ;
	mp->b_rptr = mp->b_wptr = mp->b_datap->db_base ;
    }

    minor_ptr->dl_err_prim = -1 ;		/* ensure no retry */

    /*
     * Have a buffer, build ok_ack and send it
     */
    mp->b_rptr = mp->b_datap->db_base ;
    mp->b_wptr = mp->b_datap->db_base ;

    p               = (dl_ok_ack_t *) mp->b_wptr ;
    p->dl_primitive  = DL_OK_ACK ;
    p->dl_correct_primitive = ok_prim ;
    mp->b_wptr     += sizeof(*p) ;

    putnext(minor_ptr->dl_rdq, mp);
    return(1) ;					/* succeed */

} /* ip_to_streams_ok_ack */

/************************************************************************
*                          ip_to_streams_rsrv                           *
*************************************************************************
*									*
* Upstream retry procedure.						*
*									*
* Return 1 for success, 0 for deferred.					*
*									*
************************************************************************/

int _RP ip_to_streams_rsrv(queue_t *q)
{
    ip_to_streams_minor_t	*minor_ptr ;
    int			 done ;

    if ( ip_to_streams_debug_mask & DBG_SVC )
	cmn_err(CE_CONT, "\nip_to_streams_rsrv: q=%x\n", q) ;

    minor_ptr = (ip_to_streams_minor_t *) q->q_ptr ;

    if (   minor_ptr == (ip_to_streams_minor_t *) NULL
	|| minor_ptr->dl_magic != DL_MAGIC
	|| minor_ptr->dl_rdq != q
       )
    {
	return(0) ;
    }

    done = 1 ;
    if (minor_ptr->dl_m_error > 0)
    {
	flushq(q, FLUSHALL);			/* ensure queue empty */
	ip_to_streams_m_error(minor_ptr, (mblk_t *) NULL, minor_ptr->dl_m_error, 1) ;
	return(0) ;
    }

    if (minor_ptr->dl_err_prim >= 0)
    {
	flushq(q, FLUSHALL);			/* ensure queue empty */
	if (minor_ptr->dl_tli_err == 0)
	{
	    done = ip_to_streams_ok_ack(minor_ptr,
			       (mblk_t *) NULL,
			       minor_ptr->dl_err_prim, 1) ;
	}
	else
	{
	    done = ip_to_streams_error_ack(minor_ptr,
				  (mblk_t *) NULL,
				  minor_ptr->dl_err_prim,
				  minor_ptr->dl_tli_err,
				  minor_ptr->dl_unix_err, 1) ;
	}
    }

    if (!done) return(0) ;		/* still retrying error/ok acks */

    /*
     * Try other protos
     */

    return(0) ;

} /* ip_to_streams_rsrv */

/************************************************************************
*                             ip_to_streams_wsrv                        *
*************************************************************************
*									*
* This is the downstream retry routine.					*
*									*
************************************************************************/

int _RP ip_to_streams_wsrv(queue_t *q)
{
    ip_to_streams_minor_t	*minor_ptr ;
    mblk_t			*mp ;
    struct ism_dev		*dev ;
    int				 wakeup = 0 ;

    if ( ip_to_streams_debug_mask & DBG_SVC )
	cmn_err(CE_CONT, "\nip_to_streams_wsrv: q=%x\n", q) ;

    minor_ptr = (ip_to_streams_minor_t *) q->q_ptr ;

    if (   minor_ptr == (ip_to_streams_minor_t *) NULL
	|| minor_ptr->dl_magic != DL_MAGIC
       )
    {
	return(0) ;			/* bad minor */
    }

    dev = minor_ptr->mydev;
    for (;;)
    {
	if ((mp = getq (minor_ptr -> dl_wrq)) == NULL)
	{
	    if (netif_queue_stopped(dev))
	    {
		netif_start_queue(dev) ;
		wakeup = 1 ;
	    }
	    break;			/* done */
	}

	if (mp -> b_datap -> db_type == M_DATA)
	{
	    if ( canputnext(q) )
	    {
		putnext(q, mp);		/* send downstream */
		if (netif_queue_stopped(dev))
		{
		    netif_start_queue(dev) ;
		    wakeup = 1 ;
		}
	    }
	    else
	    {
		netif_stop_queue(dev) ;
		putbqf(q, mp);
		return(0);			/* quit */
	    }
	    minor_ptr->stats.tx_packets++;
	}
	else
	{
	    if (ip_to_streams_proto(minor_ptr, mp, 1) == 0)
	    {				/* deferred, queued */
		netif_stop_queue(dev) ;	/* stop output from above */
		return(0) ;		/* quit */
	    }

	    if (netif_queue_stopped(dev))
	    {
		netif_start_queue(dev) ;
		wakeup = 1 ;
	    }
	}
    }

    if (wakeup)
	netif_wake_queue(dev) ;		/* have netif look at our queue */

    return(0) ;

} /* ip_to_streams_wsrv */


/************************************************************************
*                           ip_to_streams_proto                         *
*************************************************************************
*									*
* Handle a DLPI proto message from above.				*
*									*
* Return 1 if succeed in handling msg, 0 if deferred.			*
*									*
************************************************************************/
int
ip_to_streams_proto(ip_to_streams_minor_t *minor_ptr, mblk_t *mp, int retry)
{
    queue_t *q = minor_ptr->dl_wrq ;
    long	*lp ;

    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	cmn_err(CE_CONT, "\nip_to_streams_proto: called\n");

    lp = (long *) mp->b_rptr ;

    switch (*lp)
    {
    case DL_UNITDATA_REQ:
	if ( canputnext(q) )
	{
	    putnext(q, mp);
	    return(1) ;
	}

	putqf(q, mp);

	break ;

    default:
	if ( canputnext(q) )
	{
	    putnext(q, mp);
	    return(1) ;
	}

	putqf(q, mp);
	break ;
    }

    return(0);				/* deferred */

} /* ip_to_streams_proto */

/************************************************************************
*                           ip_to_streams_rput                          *
*************************************************************************
*									*
* Handle a message from below.						*
*									*
************************************************************************/
int _RP ip_to_streams_rput(queue_t *q, mblk_t  *mp)
{
    ip_to_streams_minor_t      *minor_ptr = q->q_ptr;
    struct ism_dev *dev ;
    int convert_to_skbuf(ip_to_streams_minor_t *, mblk_t *);

    if (minor_ptr == NULL)
    {
	cmn_err(CE_WARN, "ip_to_streams_rput: q->q_ptr NULL") ;
	freemsg(mp) ;
	return(0) ;
    }

    dev = minor_ptr->mydev;
    if ( ip_to_streams_debug_mask & DBG_PUT )
	printk("ip_to_streams_rput: q=0x%p mp=0x%p minor:0x%p dev:0x%p\n",
		 q, mp, minor_ptr, dev) ;

    /* checking, checking, checking... */
    if (   minor_ptr == (ip_to_streams_minor_t *) NULL )
    {
	if (ip_to_streams_debug_mask & DBG_SQUAWK)
	    cmn_err(CE_NOTE, "ip_to_streams_rput: bad stream") ;

	if (mp != (mblk_t *) NULL) freemsg(mp) ;

	/* mark an error, so the world can see it */
	minor_ptr->stats.rx_errors++;
	return(0) ;
    }

    /*
     * the way this is structured, you can convert the buffer to a linux skbuf
     * and also send it upstairs to the process what has the stream open.
     * Or you could pass this over to another 'device driver' that looks
     * like a device that tcp_dump can look at.
     * This way you could have a hook for tcp_dump into this area.
     */
    switch(mp->b_datap->db_type)
    {
	case M_DATA:
	{
            if( minor_ptr->contype == PSTTY )
            {
                /* if we are putting up to PSTTY, convert to buff */
                if ( convert_to_buf(minor_ptr, mp) != 0 )
                {
                    minor_ptr->stats.rx_errors++;
                    break;
                }
                minor_ptr->stats.rx_packets++;
            }
	    else if ( minor_ptr->contype == IP )
	    {
		/* if we are putting up to IP, convert to sk buff */
		if ( convert_to_skbuf(minor_ptr, mp) != 0 )
		{
		    minor_ptr->stats.rx_errors++;
		    break;
		}
		minor_ptr->stats.rx_packets++;
	    }
	    else
	    {
		/* It will go here if we are not putting to IP */
		if ( canputnext(q) )
		    putnext(q, mp);
		else
		    putqf(q, mp);
	    }
	    break;
	}

	case M_PROTO:
	case M_PCPROTO:
	{
	    int err;
	    union DL_primitives *dlp;

	    err = 0;
	    dlp = (union DL_primitives *)mp->b_rptr;

	    switch(dlp->dl_primitive)
	    {
	    case DL_BIND_ACK:
		if ( ip_to_streams_debug_mask & DBG_WPUT)
		    printk("ip_to_streams_rput: DL_BIND_ACK:\n");

		minor_ptr->dlstate = DL_DATAXFER;
		putnext(q, mp);
		break;

	    case DL_UNITDATA_IND:
		/*
		 * When we get the unitdata indication, we need to determine
		 * where to put it.  If an ip stream is connected here, we need
		 * to convert the msg buf to a linux sk buf and send it up
		 * to IP.
		 * 
		 * If ip is not connected, then we just need to do a putnext.
		 */
                if ( minor_ptr->contype == PSTTY )
                {
                    if (convert_to_buf(minor_ptr, mp) != 0 )
                    {
                        minor_ptr->stats.rx_errors++;
                        break;
                    }
                    minor_ptr->stats.rx_packets++;
                }
		else if ( minor_ptr->contype == IP )
		{
		    if (convert_to_skbuf(minor_ptr, mp) != 0 )
		    {
			minor_ptr->stats.rx_errors++;
			break;
		    }
		    minor_ptr->stats.rx_packets++;
		}
		else
		{
		    if ( canputnext(q) )
			putnext(q, mp);
		    else
			putqf(q, mp);
		}
		break;

	    default:		/* everything else just goes upstream */
		if ( canputnext(q) )
		    putnext(q, mp);
		else
		    putqf(q, mp);
		break;

	    }
	    break;
	}

	case M_FLUSH:
	{
	    if (*mp->b_rptr & FLUSHR)
	    {
		flushq(q, FLUSHDATA);
	    }

	    if (*mp->b_rptr & FLUSHW)
	    {
		flushq(WR(q), FLUSHDATA);
		*mp->b_rptr &= ~FLUSHR;

		qreply(q, mp);
	    }
	    else
		freemsg(mp);

	    break;
	}

	default:		/* send upstream */
	{
	    putnext(q, mp);
	    break;
	}

    } /* switch */

    return(0) ;

} /* ip_to_streams_rput */

/* convert_to_buf:
 Convert from a streams msg buffer to a pstty buffer.
*/

int convert_to_buf( ip_to_streams_minor_t *minor_ptr, mblk_t *mp)
{
    char 	  *buf;
    int		   len;
    mblk_t 	   *tmp = mp;
    char	   *ctmp ;

    if ( ( mp->b_datap->db_type == M_PROTO)
        || ( mp->b_datap->db_type == M_PCPROTO)
       )
	tmp = mp->b_cont;

    len = msgdsize(tmp);

    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	printk("convert_to_buf: minor_ptr 0x%p, mp 0x%p, len %d\n", 
		minor_ptr, mp, len);

    buf = kmalloc(len+4,GFP_KERNEL);
    if ( buf == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	    printk("convert_to_buf: could not allocate %d bytes\n", len);
       return(-1);
    }


    ctmp = buf;

    /*
     * skip over proto stuff, and copy the data over.
     */

    while(tmp)
    {
	len = (tmp->b_wptr - tmp->b_rptr);
        memcpy(ctmp, tmp->b_rptr, len);
	ctmp = ctmp+len;
        tmp = tmp->b_cont;
    }

    /* make sure we don't lose any buffers */
    freemsg(mp);

    /* inform pstty driver to pick up the received data.*/
    if(lis_pstty_read_frame != NULL)
        (*lis_pstty_read_frame)(minor_ptr->ttydev.index, buf,len);
    else
	printk("convert_to_buf: !!!!uninitialized lis_pstty_read_frame.\n");
	
    /* As Pstty and its line discipline don't free the buffer,
	we need to free the buf here.*/
    kfree(buf);
    return(0);
}

/************************************************************************
*                           convert_to_skbuf                          	*
*************************************************************************
*									*
* Convert from a streams msg buffer to a Linux skbuf			*
*									*
************************************************************************/
int convert_to_skbuf( ip_to_streams_minor_t *minor_ptr, mblk_t *mp)
{
    struct sk_buff *skb;
    int		    len;
    mblk_t 	   *tmp = mp;
    char	   *ctmp ;

    if ( ( mp->b_datap->db_type == M_PROTO)
        || ( mp->b_datap->db_type == M_PCPROTO)
       )
	tmp = mp->b_cont;

    len = msgdsize(tmp);

    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	printk("convert_to_skbuf: minor_ptr 0x%p, mp 0x%p, len %d\n", 
		minor_ptr, mp, len);

    skb = dev_alloc_skb(len+4);

    if ( skb == NULL )
	{
	if ( ip_to_streams_debug_mask & DBG_SQUAWK )
		printk("convert_to_skbuf: could not allocate %d bytes\n", len);
	/* make sure we don't lose any buffers */
	freemsg(mp);

	return(-1);
	}

    /* set up the skb data structure */
    skb_reserve(skb,2);
    skb->protocol = htons(ETH_P_IP);
    skb->dev = minor_ptr->mydev;

    /*
     * skip over proto stuff, and copy the data over.
     */

    while(tmp)
    {
	len = (tmp->b_wptr - tmp->b_rptr);
	ctmp = skb_put(skb, len);
        memcpy(ctmp, tmp->b_rptr, len);
#ifdef GCOM
	if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	    Rsys_hex_print("convert_to_skbuf:", ctmp, len);
#endif
        tmp = tmp->b_cont;
    }

    /* make sure we don't lose any buffers */
    freemsg(mp);

    /* inform system that an IP packet is ready to go */
#if 0
    netif_receive_skb(skb);
#endif
    /* FIXME: I have no idea why netif_receive_skb() and netif_rx don't work 
     * (they cause the machine to lock up on SLES11 SP1 with "softirq pending")
     * it seems to work fine from the HDC driver... There's some
     * comments on the internet that we have to use netif_rx_ni from
     * process context, so we'll just use that.. It seems to work
     * which is good enough for me at this point. */
    netif_rx_ni(skb);

    return(0);
}

/*
 * The higher levels take care of making this non-reentrant (it's
 * called with bh's disabled).
 */
/************************************************************************
*                           ip_strm_xmit                          	*
*************************************************************************
*									*
* This routine will convert from the Linux native buffer types to 	*
* a streams msgblk and then send down the buffer.			*
*									*
* We are not returning erros, here for a reason.  This is the same as if*
* IP 'lost' data.  When the bind completes, or a call completes, we 	*
* will then send the data out.						*
*									*
************************************************************************/

#if defined(KERNEL_2_1)
#define SKB_FREE(skb)    dev_kfree_skb_any(skb)
#else
#define SKB_FREE(skb)    dev_kfree_skb(skb, FREE_WRITE)
#endif

static int ip_strm_xmit(struct sk_buff *skb, struct ism_dev *dev)
{
/* ### TEMP */
#if defined(KERNEL_2_5)
    ip_to_streams_minor_t  *ipptr = (ip_to_streams_minor_t *)netdev_priv(dev);
#else
    ip_to_streams_minor_t  *ipptr    = (ip_to_streams_minor_t *)dev->priv;
#endif

    dl_unitdata_req_t	   *dl;
    mblk_t *mp, *mpt;
    struct rtable *rt;   /* route to be used */
    unsigned int nh;   /* the ip address for the next hop */
#if 0
    unsigned int atomic = 0;
    unsigned int intr = 0;
#endif
  
    if ( ip_to_streams_debug_mask & DBG_PUT )
	cmn_err(CE_CONT, "\nip_strm_xmit: skb 0x%lx dev 0x%lx\n", skb, dev) ;

    /* make sure everything is ok */
    if (skb == NULL || dev == NULL) 
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: skb NULL or dev NULL\n") ;
	return(0);
    }

    /* is someone trying to confuse us? */
    if ( ipptr == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: ipptr NULL\n") ;
	ipptr->stats.tx_errors++;
	SKB_FREE(skb);				/* don't want a rexmit */
	return(0);
    }

    if (ipptr->dl_magic != DL_MAGIC)		/* not already open */
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: ipptr->dl_magic\n") ;
	ipptr->stats.tx_errors++;
	SKB_FREE(skb);				/* don't want a rexmit */
        return(0);
    }

    if ( ipptr->dlstate != DL_DATAXFER )	/* BIND not complete? */
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: state != DL_DATAXFER\n") ;
	ipptr->stats.tx_errors++;
	SKB_FREE(skb);				/* don't want a rexmit */
        return(0);
    }

    /* convert to a streams message */
    /* put a proto message in front of the data */
    /* apply for the space enough to the dl_unitdata_req head plus IP address */
    mpt = allocb(sizeof(dl_unitdata_req_t) + sizeof(unsigned int), BPRI_MED);

    if ( mpt == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: mpt NULL\n") ;
	ipptr->stats.tx_errors++;
	SKB_FREE(skb);				/* don't want a rexmit */
	return(0);
    }

    mpt->b_datap->db_type = M_PROTO;

    dl = (dl_unitdata_req_t *)mpt->b_rptr;

    dl->dl_primitive        = DL_UNITDATA_REQ;
    dl->dl_dest_addr_length = sizeof(unsigned int);
    dl->dl_dest_addr_offset = sizeof(dl_unitdata_req_t);

    /* do the IP forwarding */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
    rt = (struct rtable *)skb->dst;
#else
    rt = (struct rtable *)skb_dst(skb);
#endif
    if (!rt)
	{
	/* This is a programmer error, but keep us from panicking */
	printk("ip_strm_xmit: ERROR! skb=0x%p rt=NULL!\n", skb);
	SKB_FREE(skb);
	freemsg(mpt);
#ifdef NETDEV_TX_OK
	return (NETDEV_TX_OK);
#else
	return (0);
#endif
	}
    nh = ntohl(rt->rt_gateway);  /* IP address in machine order */
    memcpy(mpt->b_rptr + sizeof(dl_unitdata_req_t), (void *)&nh,
		sizeof(unsigned int));

    mpt->b_wptr = mpt->b_rptr + sizeof(dl_unitdata_req_t) + 
			sizeof(unsigned int);

    mp = allocb(skb->len, BPRI_MED);

    if ( mp == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\nip_strm_xmit: mp NULL\n") ;
	ipptr->stats.tx_errors++;
	SKB_FREE(skb);				/* don't want a rexmit */
	freemsg(mpt);
	return(0);
    }

    mpt->b_cont = mp;	/* hook the message off the proto */

    /* copy data over to the streams buffer */
    memcpy(mp->b_rptr, skb->data, skb->len);

#ifdef GCOM
    if ( ip_to_streams_debug_mask & DBG_SQUAWK )
	Rsys_hex_print("ip_strm_xmit:", mp->b_rptr, skb->len);
#endif

    /* set write pointer to correct place */
    mp->b_wptr += skb->len;

    /* free the skb buffer */
    SKB_FREE(skb);

    /* We have to queue all packets for the service routine
     * to handle because we always come into here atomic,
     * which means that we cannot call any potentially sleeping
     * functions like a semaphore in 2.6 kernels, 
     * which is exactly what putnext() does.... */
    putqf(ipptr->dl_wrq, mpt);
    ipptr->stats.tx_packets++;

#if 0
    /* send it down to the streams modules */
    if ((atomic = in_atomic()) || (intr = in_interrupt()))
	{
	printk(
"ip_strm_xmit: in_atomic=%d in_interrupt=%d.  Can't call a semaphore here! queuing\n", 
		atomic, intr);
	}

    if ( !atomic && !intr && canputnext(ipptr->dl_wrq) )
    {
        putnext(ipptr->dl_wrq, mpt);
        ipptr->stats.tx_packets++;
    }
    else
    {
	netif_stop_queue(dev) ;
        putqf(ipptr->dl_wrq, mpt);
    }
#endif

    if ( ip_to_streams_debug_mask & DBG_PUT )
	cmn_err(CE_CONT, "\nip_strm_xmit: putnext complete\n") ;

#ifdef NETDEV_TX_OK
	return(NETDEV_TX_OK);
#else
	return(0);
#endif
}
#undef SKB_FREE

/************************************************************************
*                           ip_strm_ioctl                               *
*************************************************************************
*									*
*									*
************************************************************************/
int ip_strm_ioctl(struct ism_dev *dev, struct ifreq *ifr, int cmd)
{
    return(-EINVAL) ;

} /* ip_strm_ioctl */


/* The routine "pstty_xmit" is used to pass  data pkt 
to bound pstty device and sent out. */

int pstty_xmit(struct pstty_dev *pstty, const unsigned char *buf,int count)
{
    ip_to_streams_minor_t  *ipptr  = (ip_to_streams_minor_t *)pstty->priv ;
    dl_unitdata_req_t	   *dl;
    mblk_t *mp, *mpt;

    if ( ip_to_streams_debug_mask & DBG_PUT )
	cmn_err(CE_CONT, "\npstty_xmit: buf 0x%lx pstty 0x%lx\n", buf, pstty);

    /* make sure everything is ok */
    if (buf == NULL || pstty == NULL) 
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    cmn_err(CE_CONT, "\npstty_xmit: pstty NULL or buf NULL\n") ;
	return(0);
    }

    /* is someone trying to confuse us? */
    if ( ipptr == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	  printk("\npstty_xmit: ipptr NULL\n") ;
	ipptr->stats.tx_errors++;
	return(0);
    }

    /* convert to a streams message */
    /* put a proto message in front of the data */
    /* apply for the space enough to the dl_unitdata_req head plus IP address */
    mpt = allocb((sizeof(dl_unitdata_req_t)+sizeof(unsigned int)), BPRI_MED);

    if ( mpt == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    printk("\npstty_xmit: mpt NULL\n") ;
	ipptr->stats.tx_errors++;
	return(0);
    }

    mpt->b_datap->db_type = M_PROTO;

    mp = allocb(count, BPRI_MED);

    if ( mp == NULL )
    {
	if ( ip_to_streams_debug_mask & DBG_PUT )
	    printk("\npstty_xmit: mp NULL\n") ;
	ipptr->stats.tx_errors++;
	freemsg(mpt);
	return(0);
    }

    mpt->b_cont = mp;	/* hook the message off the proto */

    mpt->b_wptr += sizeof(dl_unitdata_req_t)+sizeof(unsigned int);

    dl = (dl_unitdata_req_t *)mpt->b_rptr;

    dl->dl_primitive        = DL_UNITDATA_REQ;
    dl->dl_dest_addr_length =sizeof(unsigned int); /* 0; */
    dl->dl_dest_addr_offset = 0;

    /* copy data over to the streams buffer */
    memcpy(mp->b_rptr, buf, count);

    /* set write pointer to correct place */
    mp->b_wptr += count;

    /* Pass it down to the lower module or driver */

    /* We have to queue all packets for the service routine
     * to handle because we always come into here atomic,
     * which means that we cannot call any potentially sleeping
     * functions like a semaphore in 2.6 kernels,
     * which is exactly what putnext() does.... */
    putqf(ipptr->dl_wrq, mpt);
    ipptr->stats.tx_packets++;

    if ( ip_to_streams_debug_mask & DBG_PUT )
	printk("\npstty_xmit: pass data packets to lower drivers. \n");
    return(0);
}

int pstty_ioctl(struct pstty_dev *pstty, unsigned int cmd, unsigned long arg)
{
    printk("pstty_ioctl: pstty=0x%p, cmd=0x%x.\n",pstty,cmd);
    return 0;
}


/************************************************************************
*                           enet_statistics                          	*
*************************************************************************
*									*
* return a pointer to the stats area					*
*									*
************************************************************************/
static struct enet_statistics *ip_strm_get_stats(struct ism_dev *dev)
{
/* ### TEMP */
#if defined(KERNEL_2_5)
    ip_to_streams_minor_t  *ipptr = (ip_to_streams_minor_t *)netdev_priv(dev);
#else
    ip_to_streams_minor_t  *ipptr    = (ip_to_streams_minor_t *)dev->priv;
#endif
    
    if (ipptr == NULL)
	return(NULL) ;

    return (&ipptr->stats);
}

/************************************************************************
*                           ip_strm_open                          	*
*************************************************************************
*									*
* This routine gets called when the ifconfig command is used to set up	*
* the IP address and device.						*
*									*
* We have to provide linkage between the streams and linux calls and	*
* data structure.  The streams side has already been done, now the linux*
* side has to finish up the job.					*
*									*
* The minor structure allocated by the streams open routine contains	*
* the dev structure used by ip.  The "priv" pointer in the dev struct	*
* points back to the minor structure.					*
*									*
************************************************************************/

int ip_strm_open(struct ism_dev *dev)
{
/* ### TEMP */
#if defined(KERNEL_2_5)
    ip_to_streams_minor_t  *ipptr = (ip_to_streams_minor_t *)netdev_priv(dev);
#else
    ip_to_streams_minor_t  *ipptr    = (ip_to_streams_minor_t *)dev->priv;
#endif
    
    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	cmn_err(CE_CONT, "ip_strm_open: dev 0x%x\n", dev);

    /* if the stream has not been opened, fail */
    if (ipptr == NULL)
	return(EIO);

    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	printk("ip_strm_open: %s: ipptr = 0x%px\n", ipptr->myname, ipptr);

    dev->flags |= IFF_POINTOPOINT ;

    if ( ip_to_streams_debug_mask & (DBG_OPEN) )
    {
	printk("ip_strm_open: ipptr: 0x%lx, dl_wrq: 0x%lx\n",
			 (unsigned long)ipptr,
			 (unsigned long)ipptr->dl_wrq);
    }

    if ( ipptr->dl_wrq == NULL)
    {
	if ( ip_to_streams_debug_mask & (DBG_OPEN) )
	{
	    printk("ip_strm_open: dl_wrq not setup correctly\n");
	}
	return(EIO);
    }

    /* this flag is used to control where we send the reads from the
     * streams modules.  If it is IP, then we will convert to a LINUX
     * sk_buf and send it up to ip, otherwise we send it to the process
     * that has opened the stream.
     */
    ipptr->contype = IP;
    ipptr->ip_open = 1 ;

    return 0;
}

/************************************************************************
*                          ip_strm_close                                *
*************************************************************************
*									*
* Called when IP wants to close our device.				*
*									*
* The STREAMS side is still set up, so we just stop steering incoming	*
* packets to IP.							*
*									*
* Loadable modules treat this as the last thing before module unload.	*
*									*
* In our case the stream is still open and a new ifconfig can re-	*
* instate this IP interface.						*
*									*
************************************************************************/
int ip_strm_close(struct ism_dev *dev)
{
/* ### TEMP */
#if defined(KERNEL_2_5)
    ip_to_streams_minor_t  *ipptr = (ip_to_streams_minor_t *)netdev_priv(dev);
#else
    ip_to_streams_minor_t  *ipptr = (ip_to_streams_minor_t *)dev->priv;
#endif
    
    netif_stop_queue(dev) ;
    ipptr->contype = 0 ;
    ipptr->ip_open = 0 ;
    return(0) ;

} /* ip_strm_close */

/************************************************************************
*                           ip_strm_init                          	*
*************************************************************************
*									*
* Initialize the dlpi -> sk_buf device. 				*
*									*
************************************************************************/
void ip_strm_init(struct ism_dev *dev)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
	dev->open = ip_strm_open;
	dev->stop = ip_strm_close;
	dev->hard_start_xmit = ip_strm_xmit;
	dev->get_stats = ip_strm_get_stats;
	dev->do_ioctl = ip_strm_ioctl;
	dev->mtu = IP_STRM_MTU;
	dev->tx_queue_len = 100;
	dev->type = ARPHRD_DLCI;	/* 0x0001		*/
	dev->flags = IFF_NOARP ;
	dev->addr_len = 0;
	dev->hard_header_len = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
	/* ZZZ This was removed in SuSE 11.0 */
	/* Added in 3b04ddde02cf1b6f14f2697da5c20eca5715017f v2.6.24-rc1 */
	dev->hard_header = 0;
	dev->hard_header_cache = 0;
	dev->header_cache_update = 0;
	dev->hard_header_parse = 0;
	dev->rebuild_header = 0;
#else
	dev->header_ops = NULL;
#endif
	memset(dev->broadcast, 0, MAX_ADDR_LEN);
	/* We have to give a non-zero dev_addr in later kernels (unknown exactly
	 * when but it fails in SuSE 11.1 2.6.27.7) */
	dev->dev_addr[0] = 1;
	dev->dev_addr[1] = 2;
	dev->dev_addr[2] = 3;
	dev->dev_addr[3] = 4;
	dev->dev_addr[4] = 5;
	dev->dev_addr[5] = 6;
#else
static const struct net_device_ops ip_strm_netops = {
	.ndo_open		= ip_strm_open,
	.ndo_stop		= ip_strm_close,
	.ndo_start_xmit		= ip_strm_xmit,
	.ndo_get_stats		= ip_strm_get_stats,
	.ndo_do_ioctl		= ip_strm_ioctl,
};
	dev->netdev_ops = &ip_strm_netops;
	dev->mtu = IP_STRM_MTU;
	dev->tx_queue_len = 100;
	dev->type = ARPHRD_DLCI;	/* 0x0001		*/
	dev->flags |= IFF_NOARP;
#ifdef IFF_XMIT_DST_RELEASE
	/* This was added in 93f154b59 (v2.6.30-rc6-670-g93f154b),
	 * we have to have it cleared so that we get the rtable in 
	 * ip_strm_xmit */
	dev->priv_flags &= ~IFF_XMIT_DST_RELEASE;
#endif
	random_ether_addr(dev->dev_addr);
#endif

    return;
}
