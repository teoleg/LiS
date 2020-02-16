#ifndef	_STROPTS_H
#define	_STROPTS_H
#define _LIS_STROPTS_H

#ident "@(#) LiS stropts.h 1.2 6/19/00 19:35:40 "

#ifdef LIS_USE_IOCTL32
#include "sys/stropts_ioctl32.h"
#else
#include "sys/stropts.h"
#endif

#endif	/* _STROPTS_H */
