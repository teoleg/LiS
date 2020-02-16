/************************************************************************
*									*
*                   Gcom DLPI <-> Linux SKBUF Driver		        *
*									*
*************************************************************************
*									*
* The file "pstty_dev.h" defines the data structure and routines        *
*	used to bind pstty device.                                      *
*									*
*			jieyan@adax.com		Nov 22, 2006            *
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

#define MAX_PSTTY_DEVICES	128
struct pstty_dev
{
  unsigned int index;
  void *priv;
  int (*write)(struct pstty_dev *pstty,const unsigned char *buf,int count);
  int (*ioctl)(struct pstty_dev *pstty,unsigned int cmd, unsigned long arg);
};

/* Define the function pointers used to register pstty driver. */
int (*lis_pstty_register) (struct pstty_dev *pstty)=NULL;
int (*lis_pstty_unregister) (struct pstty_dev *pstty)=NULL;
void (*lis_pstty_read_frame)(unsigned int chan,const char *buf, int len)=NULL;
