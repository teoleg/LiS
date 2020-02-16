#include <sys/stropts.h>		/* must be LiS version */
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <pthread.h>

#pragma weak pthread_testcancel

#ifdef BLD32OVER64
typedef struct strbuf6 {
    int       maxlen;
    int       len;
    unsigned long long buf;
} strbuf6_t;

typedef struct getpmsg_args6
{
    int                  fd ;
    int			 pad;
    unsigned long long            ctl ;
    unsigned long long            dat ;
    unsigned long long            bandp ;
    unsigned long long            flagsp ;

} getpmsg_args6_t ;

int	getpmsg(int fd, strbuf_t *ctlptr, strbuf_t *dataptr,
		int *bandp, int *flagsp)
{
    strbuf6_t ctl6;
    strbuf6_t * ptrc6 = 0;
    strbuf6_t dat6;
    strbuf6_t * ptrd6 = 0;
    getpmsg_args6_t  args ;
    int rc;

    if (ctlptr)
    {
      ctl6.maxlen = ctlptr->maxlen;
      ctl6.len    = ctlptr->len;
      ctl6.buf    = (unsigned long long)(unsigned int)ctlptr->buf;
      ptrc6 = &ctl6;
    }

    if (dataptr)
    {
      dat6.maxlen = dataptr->maxlen;
      dat6.len    = dataptr->len;
      dat6.buf    = (unsigned long long)(unsigned int)dataptr->buf;
      ptrd6 = &dat6;
    }

    memset((void*)&args,0,sizeof(getpmsg_args6_t));
    args.fd	= fd ;

    args.ctl	= (unsigned long long)(unsigned int)ptrc6;
    args.dat	= (unsigned long long)(unsigned int)ptrd6;
    args.bandp	= (unsigned long long)(unsigned int)bandp;
    args.flagsp	= (unsigned long long)(unsigned int)flagsp;

    rc = read(fd, &args, LIS_GETMSG_PUTMSG_ULEN);

    if (ctlptr)
    {
      ctlptr->len  = ctl6.len;
    }
    if (dataptr)
    {
      dataptr->len = dat6.len;
    }

    return(rc);
}
#else
int getpmsg(int fd, strbuf_t *ctlptr, strbuf_t *dataptr,
	    int *bandp, int *flagsp)
{
getpmsg_args_t	args ;
#ifdef LIS_GETMSG_PUTMSG_AS_IOCTL
int ret;
#endif

	args.fd	= fd ;
	args.ctl = ctlptr ;
	args.dat = dataptr ;
	args.bandp = bandp ;
	args.flagsp = flagsp ;

#ifdef LIS_GETMSG_PUTMSG_AS_IOCTL
	if (pthread_testcancel)
		pthread_testcancel();
	ret = ioctl(fd, I_LIS_GETPMSG, &args);
	if (ret == -1)
		{
		/* If ioctl was interrupted by a signal we check
		 * for thread cancelation.  This is because cancelation
		 * will cause system calls to return with EINTR.
		 */
		if (pthread_testcancel)
			pthread_testcancel();
		}
	return(ret);
#else
	return(read(fd, &args, LIS_GETMSG_PUTMSG_ULEN)) ;
#endif
}
#endif
