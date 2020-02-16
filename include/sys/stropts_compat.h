/*                               -*- Mode: C -*- 
 * <stropts_compat> --- STREAMS ops for 32bit apps on 64 bit 
 * 			pre 2.6.11 kernels.
 * Author          : Dan Gora
 * Created On      : 
 * RCS Id          ;
 * Last Modified By: Dan Gora
 *                 :
 *    Copyright (C) 2007  Adax, Inc.
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
 * This file is NOT to be included into user apps.  It's used by
 * LiS to include support for the renumbered ioctls for 32 bit app
 * support over 64 bit pre-2.6.11 kernels
 */

#ifndef _SYS_STROPTS_COMPAT_H
#define _SYS_STROPTS_COMPAT_H
#define	_LIS_SYS_STROPTS_COMPAT_H	/* so you can tell which stropts.h you got */

#ifdef __cplusplus
extern "C" {
#endif


#define __STRID           ('S'<<24 | 'T'<<16 | 'R'<<8)	/* for GNU libc compatibility */
#define I_PUSH_COMPAT          (__STRID | 2)
#define I_POP_COMPAT           (__STRID | 3)
#define I_FLUSH_COMPAT         (__STRID | 5)
#define I_STR_COMPAT           (__STRID | 8)
#define I_PEEK_COMPAT          (__STRID |15)
#define I_LINK_COMPAT          (__STRID |12)
#define I_UNLINK_COMPAT        (__STRID |13)


#ifdef __cplusplus
}
#endif

#endif /*!_SYS_STROPTS_COMPAT_H*/

/*----------------------------------------------------------------------
# Local Variables:      ***
# change-log-default-name: "~/src/prj/streams/src/NOTES" ***
# End: ***
  ----------------------------------------------------------------------*/
