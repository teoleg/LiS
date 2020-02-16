/************************************************************************
*                          LiS PCI Interface                            *
*************************************************************************
*									*
* This interface was created to insulate the Linux STREAMS programmer	*
* from the constantly changing PCI interface provided by the Linux	*
* kernel.								*
*									*
*    Copyright (C) 2000  David Grothe, Gcom, Inc <dave@gcom.com>	*
*									*
* This library is free software; you can redistribute it and/or		*
* modify it under the terms of the GNU Library General Public		*
* License as published by the Free Software Foundation; either		*
* version 2 of the License, or (at your option) any later version.	*
* 									*
* This library is distributed in the hope that it will be useful,	*
* but WITHOUT ANY WARRANTY; without even the implied warranty of	*
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU	*
* Library General Public License for more details.			*
* 									*
* You should have received a copy of the GNU Library General Public	*
* License along with this library; if not, write to the			*
* Free Software Foundation, Inc., 59 Temple Place - Suite 330,		*
* Cambridge, MA 02139, USA.						*
*									*
************************************************************************/

#ident "@(#) LiS lispci.c 1.15 09/07/04"

#include <sys/stream.h>		/* gets all the right LiS stuff included */
#include <sys/lispci.h>		/* LiS PCI header file */
#include <sys/lismem.h>		/* mainly needed for 2.2 kernels */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16) 
#include <linux/kobject.h>
#endif

# ifdef RH_71_KLUDGE			/* boogered up incls in 2.4.2 */
#  undef CONFIG_HIGHMEM			/* b_page has semi-circular reference */
# endif
#include <linux/pci.h>		/* kernel PCI header file */

#include <sys/osif.h>		/* LiS kernel interface */

//#define DANDEBUG
#ifdef DANDEBUG
#define DDEBUG(fmt, args...) printk("%s:%d:" fmt, __FILE__, __LINE__, ## args)
#else
#define DDEBUG(fmt, args...)
#endif

static lis_pci_dev_t	*lis_pci_dev_list = NULL ;
static lis_pci_dev_t 	*lis_pci_dev_list_end = NULL ;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16) 
struct lis_pci_dev_internal {
	struct lis_pci_dev lispcidev;
	struct kobject kobj;
};

#define to_lis_pci_dev_int(obj) container_of(obj, struct lis_pci_dev_internal, kobj)
#define lisdrvp_to_lis_pci_dev_int(obj) container_of(obj, struct lis_pci_dev_internal, lispcidev)

int __devinit lis_driver_probe (struct pci_dev *dev, 
				const struct pci_device_id *id);
void lis_driver_remove (struct pci_dev *dev);
int lis_driver_suspend(struct pci_dev *dev, pm_message_t state);
int lis_driver_suspend_late(struct pci_dev *dev, pm_message_t state);
int lis_driver_resume_early(struct pci_dev *dev);
int lis_driver_resume(struct pci_dev *dev);
void lis_driver_shutdown(struct pci_dev *dev);

static void lis_kobj_release(struct kobject *kobj);
#endif

/************************************************************************
*                       lis_map_pci_device                              *
*************************************************************************
*									*
* Given a pointer to one of our pci device structures and to one of the	*
* kernel's structures, map theirs into ours.				*
*									*
************************************************************************/
void	_RP lis_map_pci_device(lis_pci_dev_t *p, struct pci_dev *kp)
{
    int		i ;

    /*
     * Copy some data from kernel structure into ours
     */
    p->kern_ptr		= kp ;			/* to kernel's structure */
    p->bus		= kp->bus->number ;
    p->dev_fcn		= kp->devfn ;
    p->vendor		= kp->vendor ;
    p->device		= kp->device ;
    p->class		= kp->class ;
    p->hdr_type		= kp->hdr_type ;
    p->irq		= kp->irq ;


    {
	int	nn = LIS_PCI_MEM_CNT ;		/* LiS size */

	if (nn > DEVICE_COUNT_RESOURCE)		/* kernel size */
	    nn = DEVICE_COUNT_RESOURCE ;	/* reduce loop count */

	for (i = 0; i < nn; i++)
	{
	    p->mem_addrs[i] = kp->resource[i].start ;
	}
    }

} /* lis_map_pci_device */

/************************************************************************
*                          lis_pci_find_device                          *
*************************************************************************
*									*
* Find a device of the given vendor and device id and return a pointer	*
* to a lis_pci_dev_t structure for it.					*
*									*
************************************************************************/
lis_pci_dev_t   * _RP lis_pci_find_device(unsigned vendor, unsigned device,
                                     lis_pci_dev_t *previous_struct)
{
    lis_pci_dev_t	*p ;
    struct pci_dev	*kp ;
    struct pci_dev	*prev_kp ;
    lis_pci_dev_t	*tmpp ;

    if(previous_struct == NULL)
    {
	prev_kp = NULL;
	p = lis_pci_dev_list;
    }
    else
    {
	prev_kp = previous_struct->kern_ptr ;
	p = previous_struct->next;
    }

    kp = pci_find_device(vendor, device, prev_kp) ;
    if (kp == NULL) return(NULL) ;

    /* Ensure that we don't already have a lis_pci_dev_t
     * struct with this same kern_ptr.  Can happen with
     * hot swap!!! */
    tmpp = lis_pci_dev_list;
    while (tmpp != NULL)
	    {
	    if (tmpp->kern_ptr == kp)
		    return(tmpp);
	    tmpp = tmpp->next;
	    }
		    
    /*
     * Allocate our own structure that will correspond to the kernel
     * structure.
     */
    p = (lis_pci_dev_t *) ALLOCF(sizeof(*p),"lis_pci_dev_t ");
    if (p == NULL) return(NULL) ;		/* allocation failed */

    memset(p, 0, sizeof(*p)) ;
    if(lis_pci_dev_list_end == NULL) /* first one */
    {
	lis_pci_dev_list = p;
	lis_pci_dev_list_end = p;
    }
    else
    {
	lis_pci_dev_list_end->next = p;		/* remember it */
    }
    p->next = NULL;
    lis_pci_dev_list_end = p ;

    lis_map_pci_device(p, kp) ;			/* copy info */
    return(p) ;

} /* lis_pci_find_device */

/************************************************************************
*                          lis_pci_find_class                           *
*************************************************************************
*									*
* Find a class in a manner similar to finding devices.			*
*									*
************************************************************************/
lis_pci_dev_t   * _RP lis_pci_find_class(unsigned class,
                                     lis_pci_dev_t *previous_struct)
{
    lis_pci_dev_t	*p ;
    struct pci_dev	*kp ;
    struct pci_dev	*prev_kp ;

    for (p = lis_pci_dev_list; p != NULL; p = p->next)
    {
	if (p->class == class) return(p) ;
    }

    prev_kp = previous_struct == NULL ? NULL : previous_struct->kern_ptr ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10) /* name change in 2.6.10 */
    kp = pci_get_class(class, prev_kp) ;
#else
    kp = pci_find_class(class, prev_kp) ;
#endif
    if (kp == NULL) return(NULL) ;

    /*
     * Allocate our own structure that will correspond to the kernel
     * structure.
     */
    p = (lis_pci_dev_t *) ALLOCF(sizeof(*p),"lis_pci_dev_t ");
    if (p == NULL) return(NULL) ;		/* allocation failed */

    memset(p, 0, sizeof(*p)) ;
    p->next = lis_pci_dev_list ;		/* remember it */
    lis_pci_dev_list = p ;

    lis_map_pci_device(p, kp) ;			/* copy info */
    return(p) ;

} /* lis_pci_find_class */

/************************************************************************
*                          lis_pci_find_slot                            *
*************************************************************************
*									*
* Find the structure by slot number.					*
*									*
************************************************************************/
lis_pci_dev_t   * _RP lis_pci_find_slot(unsigned bus, unsigned dev_fcn)
{
    lis_pci_dev_t	*p;
    struct pci_dev	*kp = NULL;

    for (p = lis_pci_dev_list; p != NULL; p = p->next)
    {
	if (p->bus == bus && p->dev_fcn == dev_fcn) return(p) ;
    }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30) 
/* removed pci_find_slot in commit 3b073eda */
    kp = pci_find_slot(bus, dev_fcn) ;
#endif
    if (kp == NULL) return(NULL) ;

    /*
     * Allocate our own structure that will correspond to the kernel
     * structure.
     */
    p = (lis_pci_dev_t *) ALLOCF(sizeof(*p),"lis_pci_dev_t ");
    if (p == NULL) return(NULL) ;		/* allocation failed */

    memset(p, 0, sizeof(*p)) ;
    p->next = lis_pci_dev_list ;		/* remember it */
    lis_pci_dev_list = p ;

    lis_map_pci_device(p, kp) ;			/* copy info */
    return(p) ;

} /* lis_pci_find_slot */


/************************************************************************
*                        lis_pci_read_config_byte                       *
*************************************************************************
*									*
* Read a byte from config space for the device.				*
*									*
************************************************************************/
int               _RP lis_pci_read_config_byte(lis_pci_dev_t *dev,
                                          unsigned       index,
                                          unsigned char *rtn_val)
{
    if (dev == NULL || dev->kern_ptr == NULL) return(-EINVAL) ;
    return(pci_read_config_byte(dev->kern_ptr, index, rtn_val)) ;

} /* lis_pci_read_config_byte */

/************************************************************************
*                        lis_pci_read_config_word                       *
*************************************************************************
*									*
* Read a word from config space for the device.				*
*									*
************************************************************************/
int               _RP lis_pci_read_config_word(lis_pci_dev_t  *dev,
                                          unsigned        index,
                                          unsigned short *rtn_val)
{
    if (dev == NULL || dev->kern_ptr == NULL) return(-EINVAL) ;
    return(pci_read_config_word(dev->kern_ptr, index, rtn_val)) ;

} /* lis_pci_read_config_word */

/************************************************************************
*                        lis_pci_read_config_dword                      *
*************************************************************************
*									*
* Read a dword from config space for the device.			*
*									*
************************************************************************/
int               _RP lis_pci_read_config_dword(lis_pci_dev_t *dev,
                                          unsigned       index,
                                          unsigned long *rtn_val)
{
	int ret;
	unsigned int val;
	if (dev == NULL || dev->kern_ptr == NULL) 
		return(-EINVAL) ;
	ret = pci_read_config_dword(dev->kern_ptr, index, &val) ;
	*rtn_val = (unsigned long) val;
	return(ret);

} /* lis_pci_read_config_dword */

/************************************************************************
*                       lis_pci_write_config_byte                       *
*************************************************************************
*									*
* Write a byte from config space for the device.			*
*									*
************************************************************************/
int               _RP lis_pci_write_config_byte(lis_pci_dev_t *dev,
                                          unsigned       index,
                                          unsigned char  val)
{
    if (dev == NULL || dev->kern_ptr == NULL) return(-EINVAL) ;
    return(pci_write_config_byte(dev->kern_ptr, index, val)) ;

} /* lis_pci_write_config_byte */

/************************************************************************
*                        lis_pci_write_config_word                      *
*************************************************************************
*									*
* Write a word from config space for the device.			*
*									*
************************************************************************/
int               _RP lis_pci_write_config_word(lis_pci_dev_t  *dev,
                                          unsigned        index,
                                          unsigned short  val)
{
    if (dev == NULL || dev->kern_ptr == NULL) return(-EINVAL) ;
    return(pci_write_config_word(dev->kern_ptr, index, val)) ;

} /* lis_pci_write_config_word */

/************************************************************************
*                        lis_pci_write_config_dword                     *
*************************************************************************
*									*
* Write a dword from config space for the device.			*
*									*
************************************************************************/
int               _RP lis_pci_write_config_dword(lis_pci_dev_t *dev,
                                          unsigned        index,
                                          unsigned long    val)
{
	unsigned int data;

	if (dev == NULL || dev->kern_ptr == NULL) 
		return(-EINVAL) ;

	data = (unsigned int) val;
	return(pci_write_config_dword(dev->kern_ptr, index, data)) ;

} /* lis_pci_write_config_dword */

/************************************************************************
*                          lis_pci_enable_device                        *
*************************************************************************
*									*
* Enable the device.  Sometimes this is needed to resolve IRQ routing	*
* differences between different machines.				*
*									*
************************************************************************/
int  _RP lis_pci_enable_device (lis_pci_dev_t *dev)
{
    return (pci_enable_device (dev->kern_ptr));
}

/************************************************************************
*                         lis_pci_disable_device                        *
*************************************************************************
*									*
* Disable the device.  Undoes what lis_pci_enable_device does.		*
*									*
************************************************************************/
void  _RP lis_pci_disable_device (lis_pci_dev_t *dev)
{
    pci_disable_device (dev->kern_ptr);
}

/************************************************************************
*                         lis_pci_set_master                            *
*************************************************************************
*									*
* Set bus master capability for the device.				*
*									*
************************************************************************/
void              _RP lis_pci_set_master(lis_pci_dev_t *dev)
{
    if (dev == NULL || dev->kern_ptr == NULL) return ;
    pci_set_master(dev->kern_ptr) ;

} /* lis_pci_set_master */

/************************************************************************
*                             lis_pci_cleanup                           *
*************************************************************************
*									*
* Called at LiS module exit to deallocate structures.			*
*									*
************************************************************************/
void	 lis_pci_cleanup(void)
{
    lis_pci_dev_t	*p ;
    lis_pci_dev_t	*np ;

    for (p = lis_pci_dev_list; p != NULL; p = np)
    {
	np = p->next ;
	FREE(p) ;
    }

    lis_pci_dev_list = NULL ;

} /* lis_pci_cleanup */

/************************************************************************
*                         DMA Memory Allocation				*
************************************************************************/

void    * _RP lis_pci_alloc_consistent(lis_pci_dev_t  *dev,
				  size_t          size,
				  lis_dma_addr_t *dma_handle)
{
    dma_addr_t	*dp = (dma_addr_t *) dma_handle->opaque ;
    void	*vaddr ;
    
    dma_handle->size = size ;
    dma_handle->dev  = dev ;
    vaddr = pci_alloc_consistent(dev->kern_ptr, size, dp) ;
    dma_handle->vaddr = vaddr ;

    return(vaddr) ;
}

void    * _RP lis_pci_free_consistent(lis_dma_addr_t *dma_handle)
{
    if (dma_handle->vaddr != NULL)
	pci_free_consistent(dma_handle->dev->kern_ptr,
			    dma_handle->size,
			    dma_handle->vaddr,
			    *((dma_addr_t *) dma_handle->opaque)) ;
    dma_handle->vaddr = NULL ;
    return(NULL) ;
}

u32      _RP lis_pci_dma_handle_to_32(lis_dma_addr_t *dma_handle)
{
	dma_addr_t *p = (dma_addr_t *) dma_handle->opaque ;
	unsigned int val;

	val = (unsigned int) *p;
	return(val) ;
}

u64      _RP lis_pci_dma_handle_to_64(lis_dma_addr_t *dma_handle)
{
	dma_addr_t *p = (dma_addr_t *) dma_handle->opaque ;
	unsigned long long val;

	val = (unsigned long long) *p;
	return(val) ;
}

void     _RP lis_pci_map_single(lis_pci_dev_t *dev,
			   void         *ptr,
			   size_t        size,
			   lis_dma_addr_t *dma_handle,
			   int           direction)
{
    dma_addr_t	*dp = (dma_addr_t *) dma_handle->opaque ;

    switch (direction)
    {
    case LIS_SYNC_FOR_CPU:	direction = PCI_DMA_FROMDEVICE ; break ;
    case LIS_SYNC_FOR_DMA:	direction = PCI_DMA_TODEVICE ; break ;
    case LIS_SYNC_FOR_BOTH:	direction = PCI_DMA_BIDIRECTIONAL ; break ;
    default:			return ;
    }

    dma_handle->size      = size ;
    dma_handle->dev       = dev ;
    dma_handle->vaddr     = ptr ;
    dma_handle->direction = direction ;
    *dp = pci_map_single(dev->kern_ptr, ptr, size, direction);
}

void * _RP lis_pci_unmap_single(lis_dma_addr_t *dma_handle)
{
    if (dma_handle->vaddr != NULL)
	pci_unmap_single(dma_handle->dev->kern_ptr,
			 *((dma_addr_t *) dma_handle->opaque),
			 dma_handle->size,
			 dma_handle->direction);

    dma_handle->vaddr = NULL ;
    return(NULL) ;
}

/************************************************************************
*                      lis_pci_dma_sync_single                          *
*************************************************************************
*									*
* Synchronize memory with DMA.						*
*									*
************************************************************************/
void  _RP lis_pci_dma_sync_single(lis_dma_addr_t     *dma_handle,
			     size_t              size,
			     int                 direction)
{
    switch (direction)
    {
    case LIS_SYNC_FOR_CPU:	direction = PCI_DMA_FROMDEVICE ; break ;
    case LIS_SYNC_FOR_DMA:	direction = PCI_DMA_TODEVICE ; break ;
    case LIS_SYNC_FOR_BOTH:	direction = PCI_DMA_BIDIRECTIONAL ; break ;
    default:			return ;
    }

    if (size > dma_handle->size)
	size = dma_handle->size ;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10) /* name change in 2.6.10 */
    /* Why the kernel guys couldn't do this in one function 
     * is a total mystery... */
    if (direction != PCI_DMA_TODEVICE)
	    pci_dma_sync_single_for_cpu(dma_handle->dev->kern_ptr,
				*((dma_addr_t *) dma_handle->opaque),
				size,
				direction);
    if (direction != PCI_DMA_FROMDEVICE)
	    pci_dma_sync_single_for_device(dma_handle->dev->kern_ptr,
				*((dma_addr_t *) dma_handle->opaque),
				size,
				direction);
#else
    pci_dma_sync_single(dma_handle->dev->kern_ptr,
			*((dma_addr_t *) dma_handle->opaque),
		        size,
			direction);
#endif
}

/************************************************************************
*                            Miscellaneous                              *
************************************************************************/

int      _RP lis_pci_dma_supported(lis_pci_dev_t *dev, u64 mask)
{
	return(pci_dma_supported(dev->kern_ptr, mask)) ;
}

int      _RP lis_pci_set_dma_mask(lis_pci_dev_t *dev, u64 mask)
{
	return(pci_set_dma_mask(dev->kern_ptr, mask)) ;
}

int      _RP lis_pci_set_consistent_dma_mask(lis_pci_dev_t *dev, u64 mask)
{
	/* Introduced in 2.5.47... ugh. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,47)
	return(pci_set_dma_mask(dev->kern_ptr, mask)) ;
#else
	return(pci_set_consistent_dma_mask(dev->kern_ptr, mask)) ;
#endif 
}

unsigned long long _RP lis_pci_resource_start(lis_pci_dev_t *dev, unsigned int bar)
{
	unsigned long long ret;
	ret = (unsigned long long) pci_resource_start(
				(struct pci_dev*)(dev->kern_ptr), bar);
	return(ret);
}

unsigned long long _RP lis_pci_resource_end(lis_pci_dev_t *dev, unsigned int bar)
{
	unsigned long long ret;
	ret = (unsigned long long) pci_resource_end(
				(struct pci_dev*)(dev->kern_ptr), bar);
	return(ret);
}

unsigned long long _RP lis_pci_resource_len(lis_pci_dev_t *dev, unsigned int bar)
{
	unsigned long long ret;
	ret = (unsigned long long) pci_resource_len(
				(struct pci_dev*)(dev->kern_ptr), bar);
	return(ret);
}

unsigned long _RP lis_pci_resource_flags(lis_pci_dev_t *dev, unsigned int bar)
{
	unsigned long ret;
	ret = (unsigned long) pci_resource_flags(
				(struct pci_dev*)(dev->kern_ptr), bar);
	return(ret);
}

/************************************************************************
*                            Memory Barrier                             *
*************************************************************************
*									*
* Our own routine.  Kernel versioning takes care of differences.	*
*									*
************************************************************************/
void  _RP lis_membar(void)
{
    barrier();
    mb();
}

int _RP lis_pci_select_bars(lis_pci_dev_t *dev, unsigned long flags)
{
	unsigned long kflags = 0;

	if (dev == NULL || dev->kern_ptr == NULL) return (-EINVAL) ;

	if (flags & LIS_IORESOURCE_MEM)
		kflags |= IORESOURCE_MEM;
	if (flags & LIS_IORESOURCE_IO)
		kflags |= IORESOURCE_IO;
	if (flags & LIS_IORESOURCE_PREFETCH)
		kflags |= IORESOURCE_PREFETCH;
	/* ZZZ others? */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21))
	return(0);
#else
	/* Introduced in c87deff776 (2.6.20-237) */
	return(pci_select_bars((struct pci_dev*)(dev->kern_ptr), kflags));
#endif
}

int _RP lis_pci_request_selected_regions(lis_pci_dev_t *dev, int bars, 
					 char *res_name)
{
	if (dev == NULL || dev->kern_ptr == NULL) return (-EINVAL) ;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21))
	return(pci_request_regions((struct pci_dev*)(dev->kern_ptr), res_name));
#else
	/* Introduced in c87deff776 (2.6.20-237) */
	return(pci_request_selected_regions((struct pci_dev*)(dev->kern_ptr), 
						bars, res_name));
#endif
}

void _RP lis_pci_release_selected_regions(lis_pci_dev_t *dev, int bars)
{
	if (dev == NULL || dev->kern_ptr == NULL) return ;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21))
	return(pci_release_regions((struct pci_dev*)(dev->kern_ptr)));
#else
	/* Introduced in c87deff776 (2.6.20-237) */
	return(pci_release_selected_regions((struct pci_dev*)(dev->kern_ptr), 
						bars));
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16))
/* Pointer to a list of lis_pci_driver structures passed in by the users */
static struct lis_pci_driver *lis_pci_driver_head;

/* Below is the API to "new-style" PCI drivers.  This API uses a call back
 * scheme to support things like driver probing, hot swap, etc.. */
__devinit int _RP lis_pci_register_driver(struct lis_pci_driver *lis_driver)
{
	struct pci_driver *linux_drvp;
	struct lis_pci_device_id *idp;
	unsigned int entry;
	struct pci_device_id *linux_idt;
	int i;
	int ret;

	linux_drvp = (struct pci_driver*) ALLOC(sizeof(struct pci_driver));
	if (linux_drvp == NULL)
		return (-ENOMEM);
	bzero(linux_drvp, sizeof(struct pci_driver));
	linux_drvp->name = lis_driver->name;
	idp = lis_driver->id_table;
	entry = 0;
	while (idp && idp->vendor != 0)
		{
		entry++;
		idp++;
		}
	if (entry == 0)
		{
		printk("lis_pci_register_driver: ERROR: No ID table entries in "
			"lis driver 0x%p\n",
			lis_driver);
		return (-EINVAL);
		}

	linux_idt = (struct pci_device_id *)
			ALLOC(entry * sizeof(struct pci_device_id));
	if (linux_idt == NULL)
		return (-ENOMEM);
	bzero(linux_idt, entry * sizeof(struct pci_device_id));
	for (i = 0, idp = lis_driver->id_table; i < entry; i++, idp++)
		{
		linux_idt[i].vendor = idp->vendor;
		linux_idt[i].device = idp->device;
		linux_idt[i].subvendor = idp->subvendor;
		linux_idt[i].subdevice = idp->subdevice;
		DDEBUG("lis_pci_register_driver: entry %d ven=0x%x dev=0x%x "
			"subven=0x%x subdev=0x%x\n",
			i, 
			linux_idt[i].vendor, linux_idt[i].device,
			linux_idt[i].subvendor, linux_idt[i].subdevice);
		}

	linux_drvp->id_table = linux_idt;
	linux_drvp->probe = lis_driver_probe;
	linux_drvp->remove = lis_driver_remove;
	linux_drvp->suspend = lis_driver_suspend;
	linux_drvp->resume = lis_driver_resume;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	linux_drvp->suspend_late = lis_driver_suspend_late;
	linux_drvp->resume_early = lis_driver_resume_early;
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12))
	/* Added in c895817722 */
	linux_drvp->shutdown = lis_driver_shutdown;
#endif

	lis_driver->linux_pci_driver = linux_drvp;
	lis_driver->linux_pci_id_table = linux_idt;

	lis_driver->next = lis_pci_driver_head;
	lis_pci_driver_head = lis_driver;

	ret = pci_register_driver(linux_drvp);
	if (ret < 0)
		{
		FREE(linux_drvp);
		FREE(linux_idt);
		lis_pci_driver_head = lis_driver->next;
		lis_driver->linux_pci_driver = NULL;
		lis_driver->linux_pci_id_table = NULL;
		lis_driver->next = NULL;
		DDEBUG("lis_pci_register_driver: pci_register_driver failed "
			"ret=%d\n", ret);
		return (ret);
		}

	DDEBUG("lis_pci_register_driver: lis_driver=0x%p head=0x%p "
		"linux_driver=0x%p id_table=0x%p\n",
		lis_driver, lis_pci_driver_head, linux_drvp, linux_idt);

	return (ret);
}

void _RP lis_pci_unregister_driver(struct lis_pci_driver *lis_driver)
{
	struct pci_driver *linux_drvp;
	struct lis_pci_driver *lisdrvp;

	linux_drvp = lis_driver->linux_pci_driver;
	if (!linux_drvp)
		{
		printk(
	"lis_pci_unregister_driver: ERROR: lis_driver=0x%p NULL linux_drvp", 
			lis_driver);
		return;
		}
	if (!linux_drvp->id_table)
		{
		printk(
	"lis_pci_unregister_driver: ERROR: lis_driver=0x%p linux_drvp=0x%p "
	"NULL id_table", 
			lis_driver, linux_drvp);
		return;
		}

	DDEBUG("lis_pci_unregister_driver: before unregister. lis_driver=0x%p "
		"head=0x%p linux_driver=0x%p id_table=0x%p\n",
		lis_driver, lis_pci_driver_head, linux_drvp, 
		linux_drvp->id_table);

	pci_unregister_driver(linux_drvp);

	/* Find which lis_pci_driver belongs to this pci_driver */
	lisdrvp = lis_pci_driver_head;
	if (lisdrvp == lis_driver)
		lis_pci_driver_head = lis_driver->next;
	else
		{
		while (lisdrvp)
			{
			if (lisdrvp->next == lis_driver)
				{
				lisdrvp->next = lis_driver->next;
				break;
				}
			lisdrvp = lisdrvp->next;
			}
		}

	DDEBUG("lis_pci_unregister_driver: after unregister. lis_driver=0x%p "
		"head=0x%p linux_driver=0x%p id_table=0x%p\n",
		lis_driver, lis_pci_driver_head, linux_drvp, 
		linux_drvp->id_table);

	FREE((void*) linux_drvp->id_table);
	FREE(linux_drvp);
}

static struct lis_pci_driver *lis_find_driver(struct pci_dev *dev)
{
	struct pci_driver *driver;
	struct lis_pci_driver *lisdrvp;

	/* Find which pci_driver allocated this pci_dev */
	if (!dev)
		return (NULL);

	driver = dev->driver;
	if (!driver)
		return (NULL);

	/* Find which lis_pci_driver belongs to this pci_driver */
	lisdrvp = lis_pci_driver_head;
	while (lisdrvp)
		{
		if (lisdrvp->linux_pci_driver == driver)
			break;
		lisdrvp = lisdrvp->next;
		}
	return (lisdrvp);
}

extern struct kset *lis_kset;

static void lis_kset_release(struct kobject *kobj)
{
	struct kset *kset = container_of(kobj, struct kset, kobj);
	DDEBUG("lis_kset_release: freeing kset 0x%p kobj=0x%p (%s)\n",
		kset, kobj, kobject_name(kobj));
	FREE(kset);
}

static struct kobj_type lis_kset_ktype = {
	.release = lis_kset_release,
};

struct kset *lis_kset_create_and_add(const char *name,
				     const struct kset_uevent_ops *uevent_ops,
				     struct kobject *parent_kobj)
{
	struct kset *kset;
	int ret;

	kset = ALLOC(sizeof(*kset));
	if (!kset)
		return (NULL);
	bzero((void*) kset, sizeof (*kset));

	ret = kobject_set_name(&kset->kobj, name);
	if (ret)
		{
		FREE(kset);
		return (NULL);
		}
	kset->uevent_ops = uevent_ops;
	kset->kobj.parent = parent_kobj;
	kset->kobj.ktype = &lis_kset_ktype;
	kset->kobj.kset = NULL;

	DDEBUG("lis_kset_create_and_add: created kset=0x%p kobj=0x%p\n", 
		kset, &kset->kobj);
	ret = kset_register(kset);
	DDEBUG("lis_kset_create_and_add: created kset=0x%p kobj=0x%p "
		"refcount=%d\n", 
		kset, &kset->kobj, atomic_read(&kset->kobj.kref.refcount));
	if (ret)
		{
		FREE(kset);
		return (NULL);
		}
	return (kset);
}

static struct kobj_type lis_ktype = {
	.release = lis_kobj_release,
};

static void lis_kobj_release(struct kobject *kobj)
{
	struct lis_pci_dev_internal *lisdev_interal = to_lis_pci_dev_int(kobj);
	DDEBUG("lis_kobj_release: freeing lisdev_internal=0x%p\n", 
		lisdev_interal);
	FREE(lisdev_interal);
}

/* Call the driver's probe routine.
 * Implementation note:  We call the lis_probe routine for all drivers
 * bound to lis with lis_pci_register_driver, not just ones which match
 * the ven/devids.  The driver writer must ensure to check the suppled
 * ven/devid and return -ENODEV or someother negative value if it doesn't
 * match the driver's expected values.
 * On success the driver should return the device instance number as it
 * will be used to create a kobject with the name 'lis-<driver name>-<inst>
 */
int __devinit lis_driver_probe (struct pci_dev *dev, 
				const struct pci_device_id *id)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_dev_internal *lisdev_internal;
	struct lis_pci_driver *lisdrvp;
	struct lis_pci_device_id ent;
	int inst;
	int nn;
	int i;
	int ret;
	char name[64];

	/* Allocate a lis_pci_dev_internal for this device instance */
	lisdev_internal = (struct lis_pci_dev_internal *) 
            ALLOC(sizeof(*lisdev_internal));

        if (lisdev_internal == NULL) 
		return(-ENOMEM);
	bzero(lisdev_internal, sizeof(*lisdev_internal));
	lisdev = &lisdev_internal->lispcidev;

	/* Associate the kernel's pci_dev with our lis_pci_dev_t */
	lisdev->kern_ptr = dev;
	pci_set_drvdata(dev, lisdev);

	/* Initialize the lis_pci_dev_t with info from the kernel's pci_dev */
	lisdev->bus = 0;
	lisdev->dev_fcn = dev->devfn;
	lisdev->vendor = dev->vendor;
	lisdev->device = dev->device;
	lisdev->class = dev->class;
	lisdev->hdr_type = dev->hdr_type;

	nn = min(DEVICE_COUNT_RESOURCE, LIS_PCI_MEM_CNT);
	for (i = 0; i < nn; i++)
		lisdev->mem_addrs[i] = dev->resource[i].start ;

	ent.vendor = id->vendor;
	ent.device = id->device;
	ent.subvendor = id->subvendor;
	ent.subdevice = id->subdevice;

       	lisdrvp = lis_pci_driver_head;
	inst = -ENODEV;
	while (lisdrvp)
		{
		DDEBUG("lis_driver_probe: lisdrvp=0x%p linux dev=0x%p "
			"linux driver=0x%p lisdev_internal=0x%p "
			"lisdev=0x%p lisdev->kern_ptr=0x%p probe=0x%p\n",
			lisdrvp, dev, dev->driver, lisdev_internal, 
			lisdev, lisdev->kern_ptr, lisdrvp->lis_probe);

		/* Call the driver's probe() routine, which expects a 
		 * lis_pci_dev_t. The driver probe returns an instance number 
		 * or -errno */
		if (lisdrvp->lis_probe)
                {
			inst = lisdrvp->lis_probe(lisdev, &ent);
                        
			if (inst >= 0)
				break;
                }
		lisdrvp = lisdrvp->next;
		}

	if (inst < 0)
        {
                printk("LiS: %s: PROBE id->vendor %xH, id->device %xH, inst = %xH\n",
                       __func__, id->vendor, id->device, inst);
                
		FREE(lisdev_internal);
                return(inst);
        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25))
	lisdev_internal->kobj.kset = lis_kset;
	kobject_init(&lisdev_internal->kobj, &lis_ktype);

	sprintf(name, "lis-%s-%d", lisdrvp->name, inst);
	DDEBUG("lis_driver_probe: calling kobject_add for kobj=0x%p:%s\n",
		&lisdev_internal->kobj, name);

	ret = kobject_add(&lisdev_internal->kobj, NULL, name);
	if (ret < 0)
		printk("lis_driver_probe: kobject_add failed errno=%d\n", ret);
	DDEBUG("lis_driver_probe: calling KOBJ_ADD for kobj=0x%p refcount=%d\n",
		&lisdev_internal->kobj, 
		atomic_read(&lisdev_internal->kobj.kref.refcount));
	ret = kobject_uevent(&lisdev_internal->kobj, KOBJ_ADD);
	if (ret < 0)
		printk("lis_driver_probe: kobject_uevent failed errno=%d\n", ret);
	DDEBUG("lis_driver_probe: kobject_uevent returned=%d\n", 
		ret);
#else
	(void) ret;
	kobject_init(&lisdev_internal->kobj);
	DDEBUG("lis_driver_probe: initialized kobject kobj=0x%p (%s) "
		"parent=0x%p kset=0x%p\n", 
		&lisdev_internal->kobj, kobject_name(&lisdev_internal->kobj), 
		lisdev_internal->kobj.parent,
		lisdev_internal->kobj.kset);
	lisdev_internal->kobj.ktype = &lis_ktype;
	lisdev_internal->kobj.kset = lis_kset;
	sprintf(name, "lis-%s-%d", lisdrvp->name, inst);
	kobject_set_name(&lisdev_internal->kobj, name);
	DDEBUG("lis_driver_probe: calling kobject_add for kobj=0x%p (%s) "
		"parent=0x%p kset=0x%p\n", 
		&lisdev_internal->kobj, kobject_name(&lisdev_internal->kobj), 
		lisdev_internal->kobj.parent,
		lisdev_internal->kobj.kset);
	kobject_add(&lisdev_internal->kobj);
	DDEBUG("lis_driver_probe: calling KOBJ_ADD for kobj=0x%p refcount=%d "
		"parent=0x%p kset=0x%p refcount=%d\n",
		&lisdev_internal->kobj, 
		atomic_read(&lisdev_internal->kobj.kref.refcount),
		lisdev_internal->kobj.parent,
		lis_kset, atomic_read(&lis_kset->kobj.kref.refcount));
	kobject_uevent(&lisdev_internal->kobj, KOBJ_ADD);
#endif

	if (inst < 0)
		return (inst);
	else
		return (0);
}

void lis_driver_remove (struct pci_dev *dev)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_dev_internal *lisdev_internal;
	struct lis_pci_driver *lisdrvp;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return;

	lisdev = pci_get_drvdata(dev);
	lisdev_internal = lisdrvp_to_lis_pci_dev_int(lisdev);

	DDEBUG("lis_driver_remove: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p remove=0x%p "
		"lisdev_internal=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_remove, 
		lisdev_internal);

	if (lisdrvp->lis_remove)
		lisdrvp->lis_remove(lisdev);
	
	kobject_uevent(&lisdev_internal->kobj, KOBJ_REMOVE);

	/* this will also free the lisdev memory */
	DDEBUG("lis_driver_remove: calling kobject_del kobj=0x%p refcount=%d "
		"kset=0x%p refcount=%d\n",
		&lisdev_internal->kobj, 
		atomic_read(&lisdev_internal->kobj.kref.refcount),
		lis_kset, atomic_read(&lis_kset->kobj.kref.refcount));
	kobject_del(&lisdev_internal->kobj);
	DDEBUG("lis_driver_remove: calling kobject_put kobj=0x%p refcount=%d "
		"parent=0x%p kset=0x%p refcount=%d\n",
		&lisdev_internal->kobj, 
		atomic_read(&lisdev_internal->kobj.kref.refcount),
		lisdev_internal->kobj.parent,
		lis_kset, atomic_read(&lis_kset->kobj.kref.refcount));
	lisdev_internal->kobj.parent = NULL;
	/* This is necessary because pre 2.6.25 kernels set the parent
	 * in kobject_add, but don't remove in it kobject_del.  In our
	 * case both parent points to the lis_kset->kobj, so this kobject_put
	 * will call kobject_put _twice_ once for the 'parent' and once
	 * for the kset here, which is not what we want. */
	kobject_put(&lisdev_internal->kobj);
	DDEBUG("lis_driver_remove: After kobject_put kset=0x%p refcount=%d\n",
		lis_kset, atomic_read(&lis_kset->kobj.kref.refcount));
}

int lis_driver_suspend(struct pci_dev *dev, pm_message_t state)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_driver *lisdrvp;
	int ret = -ENODEV;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return (-ENXIO);

	lisdev = pci_get_drvdata(dev);

	DDEBUG("lis_driver_suspend: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p suspend=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_suspend);

	if (lisdrvp->lis_suspend)
		ret = lisdrvp->lis_suspend(lisdev, state.event);
	return (ret);
}

int lis_driver_suspend_late(struct pci_dev *dev, pm_message_t state)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_driver *lisdrvp;
	int ret = -ENODEV;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return (-ENXIO);

	lisdev = pci_get_drvdata(dev);

	DDEBUG("lis_driver_suspend_late: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p suspend_late=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_suspend_late);

	if (lisdrvp->lis_suspend_late)
		ret = lisdrvp->lis_suspend_late(lisdev, state.event);
	return (ret);
}

int lis_driver_resume(struct pci_dev *dev)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_driver *lisdrvp;
	int ret = -ENODEV;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return (-ENXIO);

	lisdev = pci_get_drvdata(dev);

	DDEBUG("lis_driver_resume: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p resume=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_resume);

	if (lisdrvp->lis_resume)
		ret = lisdrvp->lis_resume(lisdev);
	return (ret);
}

int lis_driver_resume_early(struct pci_dev *dev)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_driver *lisdrvp;
	int ret = -ENODEV;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return (-ENXIO);

	lisdev = pci_get_drvdata(dev);

	DDEBUG("lis_driver_resume_early: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p resume_early=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_resume_early);

	if (lisdrvp->lis_resume_early)
		ret = lisdrvp->lis_resume_early(lisdev);
	return (ret);
}

void lis_driver_shutdown(struct pci_dev *dev)
{
	lis_pci_dev_t *lisdev;
	struct lis_pci_driver *lisdrvp;

	lisdrvp = lis_find_driver(dev);
	if (!lisdrvp)
		return;

	lisdev = pci_get_drvdata(dev);

	DDEBUG("lis_driver_shutdown: lisdrvp=0x%p linux dev=0x%p "
		"linux driver=0x%p lisdev=0x%p shutdown=0x%p\n",
		lisdrvp, dev, dev->driver, lisdev, lisdrvp->lis_shutdown);

	if (lisdrvp->lis_shutdown)
		lisdrvp->lis_shutdown(lisdev);
	return;
}
#endif
