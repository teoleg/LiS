/************************************************************************
*                       STREAMS Exported Symbols			*
*************************************************************************
*									*
* Author:	David Grothe	<dave@gcom.com>				*
*									*
* Copyright (C) 2002  David Grothe, Gcom, Inc <dave@gcom.com>		*
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

#ident "@(#) LiS exports.c 1.34 09/16/04"

/* MarkS@Adax - 21 Dec 2004 - moved the next two defines above include of
   sys/LiS/linux-mdep.h to correct compilation error on MontaVista 3.1 CGE 
   kernel that was failing with 
   "error this_object_must_be_defined_as_export_objs_in_the_Makefile" found
   in kernel include module.h */
#define CONFIG_MODULES  1
#define EXPORT_SYMTAB   1

#include <sys/LiS/linux-mdep.h>		/* redefine dev_t b4 any kernel incls */

#ifdef MODVERSIONS
# ifdef LISMODVERS
#  include <sys/modversions.h>      /* /usr/src/LiS/include/sys */
# else
#  include <config/modversions.h>
# endif
#endif
#define __NO_VERSION__	1	/* 2.2 kernel needs this */
#include <linux/module.h>

#ifndef EXPORT_SYMBOL_NOVERS
/* This was finally ripped out of the Linux Kernel in 2.6.11 */
#define EXPORT_SYMBOL_NOVERS(sym) EXPORT_SYMBOL(sym)
#endif

#include <sys/stream.h>
#include <sys/poll.h>
#include <sys/lislocks.h>
#include <sys/lismem.h>
#include <sys/lispci.h>
#include <sys/cmn_err.h>
#include <sys/osif.h>
#include <sys/LiS/modcnt.h>
#include <sys/lisnet.h>

/************************************************************************
*                           Prototypes                                  *
*************************************************************************
*									*
* For things that aren't in the LiS header files.			*
*									*
************************************************************************/


extern int		lis_major;
extern char		lis_kernel_version[];
extern char		lis_version[] ;
extern char		lis_date[] ;
extern char             lis_release[] ;
extern char		*lis_stropts_file ;
extern char		*lis_poll_file;
extern int              lis_num_cpus ;
extern lis_atomic_t     lis_in_syscall ;
extern lis_atomic_t     lis_queues_running ; 
extern volatile unsigned long lis_runq_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_runq_srv_rpt_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_queuerun_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_setqsched_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_setqsched_isr_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_setqsched_usr_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_setqsched_defer_cnts[LIS_NR_CPUS] ;
extern volatile unsigned long lis_setqsched_poke_cnts[LIS_NR_CPUS] ;
extern lis_atomic_t	lis_strcount ;
extern long		lis_max_mem ;
extern int        	lis_seq_cntr ;

/* MarkS@Adax - 12 Nov 2007 - in support of moving below 
   EXPORT_SYMBOL_NOVERS for pstty related items from 
   drivers/str/linux/pstty_dev.h to this file to fix a 
   problem compiling LiS on MontaVista 3.1. */
extern int lis_pstty_register;
extern int lis_pstty_unregister;
extern void lis_pstty_read_frame;




/************************************************************************
*                             Exports                                   *
*************************************************************************
*									*
* Symbols exported by LiS.  These are in alphabetical order.		*
*									*
************************************************************************/

EXPORT_SYMBOL_NOVERS(lis_adjmsg);
EXPORT_SYMBOL_NOVERS(lis_allocb);
EXPORT_SYMBOL_NOVERS(lis_allocb_physreq);
EXPORT_SYMBOL_NOVERS(lis_alloc_atomic_fcn);
EXPORT_SYMBOL_NOVERS(lis_alloc_dma_fcn);
EXPORT_SYMBOL_NOVERS(lis_alloc_etherdev);
EXPORT_SYMBOL_NOVERS(lis_alloc_kernel_fcn);
EXPORT_SYMBOL_NOVERS(lis_appq);
EXPORT_SYMBOL_NOVERS(lis_apush_get);		/* for sad */
EXPORT_SYMBOL_NOVERS(lis_apush_set);		/* for sad */
EXPORT_SYMBOL_NOVERS(lis_assert_fail);
EXPORT_SYMBOL_NOVERS(lis_atomic_add);
EXPORT_SYMBOL_NOVERS(lis_atomic_dec);
EXPORT_SYMBOL_NOVERS(lis_atomic_dec_and_test);
EXPORT_SYMBOL_NOVERS(lis_atomic_inc);
EXPORT_SYMBOL_NOVERS(lis_atomic_read);
EXPORT_SYMBOL_NOVERS(lis_atomic_set);
EXPORT_SYMBOL_NOVERS(lis_atomic_sub);
EXPORT_SYMBOL_NOVERS(lis_backq);
EXPORT_SYMBOL_NOVERS(lis_backq_fcn);
EXPORT_SYMBOL_NOVERS(lis_bcanput);
EXPORT_SYMBOL_NOVERS(lis_bcanputnext);
EXPORT_SYMBOL_NOVERS(lis_bcanputnext_anyband);
EXPORT_SYMBOL_NOVERS(lis_bufcall);
EXPORT_SYMBOL_NOVERS(lis_clone_major);
EXPORT_SYMBOL_NOVERS(lis_cmn_err);
EXPORT_SYMBOL_NOVERS(lis_copyb);
EXPORT_SYMBOL_NOVERS(lis_copymsg);
EXPORT_SYMBOL_NOVERS(lis_date);
EXPORT_SYMBOL_NOVERS(lis_debug_mask);
EXPORT_SYMBOL_NOVERS(lis_debug_mask2);
EXPORT_SYMBOL_NOVERS(lis_dev_alloc_skb);
#if 0 && defined(CONFIG_DEV)
EXPORT_SYMBOL_NOVERS(lis_dec_mod_cnt_fcn);
#endif
EXPORT_SYMBOL_NOVERS(lis_disable_irq);
EXPORT_SYMBOL_NOVERS(lis_down_fcn);
EXPORT_SYMBOL_NOVERS(lis_down_nosig_fcn);
EXPORT_SYMBOL_NOVERS(lis_dsecs);
EXPORT_SYMBOL_NOVERS(lis_dupb);
EXPORT_SYMBOL_NOVERS(lis_dupmsg);
EXPORT_SYMBOL_NOVERS(lis_enable_irq);
EXPORT_SYMBOL_NOVERS(lis_esballoc);
EXPORT_SYMBOL_NOVERS(lis_esbbcall);
EXPORT_SYMBOL_NOVERS(lis_flushband);
EXPORT_SYMBOL_NOVERS(lis_flushq);
EXPORT_SYMBOL_NOVERS(lis_free);
EXPORT_SYMBOL_NOVERS(lis_freeb);
EXPORT_SYMBOL_NOVERS(lis_free_netdev);
EXPORT_SYMBOL_NOVERS(lis_free_irq);
EXPORT_SYMBOL_NOVERS(lis_free_mem_fcn);
EXPORT_SYMBOL_NOVERS(lis_freemsg);
EXPORT_SYMBOL_NOVERS(lis_free_pages_fcn);
EXPORT_SYMBOL_NOVERS(lis_freezestr);
EXPORT_SYMBOL_NOVERS(lis_get_pci_dev_irq);
EXPORT_SYMBOL_NOVERS(lis_get_free_pages_atomic_fcn);
EXPORT_SYMBOL_NOVERS(lis_get_free_pages_fcn);
EXPORT_SYMBOL_NOVERS(lis_get_free_pages_kernel_fcn);
EXPORT_SYMBOL_NOVERS(lis_getint);
EXPORT_SYMBOL_NOVERS(lis_getq);
EXPORT_SYMBOL_NOVERS(lis_gettimeofday);
EXPORT_SYMBOL_NOVERS(lis_hitime);
#if 0 && defined(CONFIG_DEV)
EXPORT_SYMBOL_NOVERS(lis_inc_mod_cnt_fcn);
#endif
EXPORT_SYMBOL_NOVERS(lis_insq);
EXPORT_SYMBOL_NOVERS(lis_in_interrupt);
EXPORT_SYMBOL_NOVERS(lis_interruptible_sleep_on);
EXPORT_SYMBOL_NOVERS(lis_interruptible_sleep_on_timeout);
EXPORT_SYMBOL_NOVERS(lis_ioremap);
EXPORT_SYMBOL_NOVERS(lis_ioremap_nocache);
EXPORT_SYMBOL_NOVERS(lis_iounmap);
EXPORT_SYMBOL_NOVERS(lis_irqreturn_handled);
EXPORT_SYMBOL_NOVERS(lis_irqreturn_not_handled);
EXPORT_SYMBOL_NOVERS(lis_jiffies);
EXPORT_SYMBOL_NOVERS(lis_kernel_down);
EXPORT_SYMBOL_NOVERS(lis_kernel_up);
EXPORT_SYMBOL_NOVERS(lis_kernel_version);
EXPORT_SYMBOL_NOVERS(lis_kfree);
EXPORT_SYMBOL_NOVERS(lis_kmalloc);
EXPORT_SYMBOL_NOVERS(lis_linkb);
EXPORT_SYMBOL_NOVERS(lis_getmajor);
EXPORT_SYMBOL_NOVERS(lis_getminor);
EXPORT_SYMBOL_NOVERS(lis_makedevice);
EXPORT_SYMBOL_NOVERS(lis_major);
EXPORT_SYMBOL_NOVERS(lis_malloc);
EXPORT_SYMBOL_NOVERS(lis_mark_mem_fcn);
#if (!defined(_S390_LIS_) && !defined(_S390X_LIS_))
EXPORT_SYMBOL_NOVERS(lis_membar);
#endif          /* S390 or S390X */
EXPORT_SYMBOL_NOVERS(lis_milli_to_ticks);
EXPORT_SYMBOL_NOVERS(lis_mknod);
#if 0 && defined(CONFIG_DEV)
EXPORT_SYMBOL_NOVERS(lis_mod_cnt_sync_fcn);
#endif
EXPORT_SYMBOL_NOVERS(lis_modget);
EXPORT_SYMBOL_NOVERS(lis_modput);
EXPORT_SYMBOL_NOVERS(lis_msecs);
EXPORT_SYMBOL_NOVERS(lis_msgdsize);
EXPORT_SYMBOL_NOVERS(lis_msgpullup);
EXPORT_SYMBOL_NOVERS(lis_msgsize);
EXPORT_SYMBOL_NOVERS(lis_msg_type_name);
EXPORT_SYMBOL_NOVERS(lis_netif_carrier_off);
EXPORT_SYMBOL_NOVERS(lis_netif_carrier_ok);
EXPORT_SYMBOL_NOVERS(lis_netif_carrier_on);
EXPORT_SYMBOL_NOVERS(lis_netif_stop_queue);
EXPORT_SYMBOL_NOVERS(lis_netif_wake_queue);
EXPORT_SYMBOL_NOVERS(lis_num_cpus);
EXPORT_SYMBOL_NOVERS(lis_osif_cli);
EXPORT_SYMBOL_NOVERS(lis_osif_do_gettimeofday);
EXPORT_SYMBOL_NOVERS(lis_osif_do_settimeofday);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_disable_device);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_enable_device);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_find_class);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_find_device);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_find_slot);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_module_init);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_read_config_byte);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_read_config_dword);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_read_config_word);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_set_master);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_unregister_driver);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_write_config_byte);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_write_config_dword);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_write_config_word);
EXPORT_SYMBOL_NOVERS(lis_osif_sti);
EXPORT_SYMBOL_NOVERS(lis_own_spl);
#if (!defined(_S390_LIS_) && !defined(_S390X_LIS_))
EXPORT_SYMBOL_NOVERS(lis_pcibios_find_class);
EXPORT_SYMBOL_NOVERS(lis_pcibios_find_device);
EXPORT_SYMBOL_NOVERS(lis_pcibios_init);
EXPORT_SYMBOL_NOVERS(lis_pcibios_present);
EXPORT_SYMBOL_NOVERS(lis_pcibios_read_config_byte);
EXPORT_SYMBOL_NOVERS(lis_pcibios_read_config_dword);
EXPORT_SYMBOL_NOVERS(lis_pcibios_read_config_word);
EXPORT_SYMBOL_NOVERS(lis_pcibios_strerror);
EXPORT_SYMBOL_NOVERS(lis_pcibios_write_config_byte);
EXPORT_SYMBOL_NOVERS(lis_pcibios_write_config_dword);
EXPORT_SYMBOL_NOVERS(lis_pcibios_write_config_word);
EXPORT_SYMBOL_NOVERS(lis_pci_alloc_consistent);
EXPORT_SYMBOL_NOVERS(lis_pci_disable_device);
EXPORT_SYMBOL_NOVERS(lis_pci_dma_handle_to_32);
EXPORT_SYMBOL_NOVERS(lis_pci_dma_handle_to_64);
EXPORT_SYMBOL_NOVERS(lis_pci_dma_supported);
EXPORT_SYMBOL_NOVERS(lis_pci_dma_sync_single);
EXPORT_SYMBOL_NOVERS(lis_pci_enable_device);
EXPORT_SYMBOL_NOVERS(lis_pci_find_class);
EXPORT_SYMBOL_NOVERS(lis_pci_find_device);
EXPORT_SYMBOL_NOVERS(lis_pci_find_slot);
EXPORT_SYMBOL_NOVERS(lis_pci_free_consistent);
EXPORT_SYMBOL_NOVERS(lis_pci_map_single);
EXPORT_SYMBOL_NOVERS(lis_pci_read_config_byte);
EXPORT_SYMBOL_NOVERS(lis_pci_read_config_dword);
EXPORT_SYMBOL_NOVERS(lis_pci_read_config_word);
EXPORT_SYMBOL_NOVERS(lis_pci_resource_end);
EXPORT_SYMBOL_NOVERS(lis_pci_resource_flags);
EXPORT_SYMBOL_NOVERS(lis_pci_resource_len);
EXPORT_SYMBOL_NOVERS(lis_pci_resource_start);
EXPORT_SYMBOL_NOVERS(lis_pci_set_dma_mask);
EXPORT_SYMBOL_NOVERS(lis_pci_set_consistent_dma_mask);
EXPORT_SYMBOL_NOVERS(lis_pci_set_master);
EXPORT_SYMBOL_NOVERS(lis_pci_unmap_single);
EXPORT_SYMBOL_NOVERS(lis_pci_write_config_byte);
EXPORT_SYMBOL_NOVERS(lis_pci_write_config_dword);
EXPORT_SYMBOL_NOVERS(lis_pci_write_config_word);
EXPORT_SYMBOL_NOVERS(lis_pci_select_bars);
EXPORT_SYMBOL_NOVERS(lis_pci_request_selected_regions);
EXPORT_SYMBOL_NOVERS(lis_pci_release_selected_regions);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
EXPORT_SYMBOL_NOVERS(lis_pci_register_driver);
EXPORT_SYMBOL_NOVERS(lis_pci_unregister_driver);
#endif
#endif          /* S390 or S390X */
EXPORT_SYMBOL_NOVERS(lis_phys_to_virt);
EXPORT_SYMBOL_NOVERS(lis_print_block);
EXPORT_SYMBOL_NOVERS(lis_print_data);
EXPORT_SYMBOL_NOVERS(lis_printk);
EXPORT_SYMBOL_NOVERS(lis_print_mem);
EXPORT_SYMBOL_NOVERS(lis_print_msg);
EXPORT_SYMBOL_NOVERS(lis_print_queue);
/* MarkS@Adax - 12 Nov 2007 - moved the following pstty related
   EXPORT_SYMBOL_NOVERS from drivers/str/linux/pstty_dev.h to
   this file to fix a problem compiling LiS on MontaVista 3.1. */
EXPORT_SYMBOL_NOVERS(lis_pstty_register);
EXPORT_SYMBOL_NOVERS(lis_pstty_unregister);
EXPORT_SYMBOL_NOVERS(lis_pstty_read_frame);
EXPORT_SYMBOL_NOVERS(lis_pullupmsg);
EXPORT_SYMBOL_NOVERS(lis_putbq);
EXPORT_SYMBOL_NOVERS(lis_putbqf);
EXPORT_SYMBOL_NOVERS(lis_putbyte);
EXPORT_SYMBOL_NOVERS(lis_putctl);
EXPORT_SYMBOL_NOVERS(lis_putctl1);
EXPORT_SYMBOL_NOVERS(lis_putnextctl);
EXPORT_SYMBOL_NOVERS(lis_putnextctl1);
EXPORT_SYMBOL_NOVERS(lis_putq);
EXPORT_SYMBOL_NOVERS(lis_putqf);
EXPORT_SYMBOL_NOVERS(lis_qcountstrm);
EXPORT_SYMBOL_NOVERS(lis_qenable);
EXPORT_SYMBOL_NOVERS(lis_qprocsoff);
EXPORT_SYMBOL_NOVERS(lis_qprocson);
EXPORT_SYMBOL_NOVERS(lis_qsize);
EXPORT_SYMBOL_NOVERS(lis_queue_name);
EXPORT_SYMBOL_NOVERS(lis_register_netdev);
EXPORT_SYMBOL_NOVERS(lis_register_strdev);
EXPORT_SYMBOL_NOVERS(lis_register_strmod);
EXPORT_SYMBOL_NOVERS(lis_register_driver_qlock_option);
EXPORT_SYMBOL_NOVERS(lis_register_module_qlock_option);
EXPORT_SYMBOL_NOVERS(lis_release_region);
EXPORT_SYMBOL_NOVERS(lis_request_irq);
EXPORT_SYMBOL_NOVERS(lis_request_region);
EXPORT_SYMBOL_NOVERS(lis_rmvb);
EXPORT_SYMBOL_NOVERS(lis_rmvq);
EXPORT_SYMBOL_NOVERS(lis_rw_lock_alloc_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_lock_free_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_lock_init_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_lock_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_lock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_lock_irqsave_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_unlock_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_unlock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_read_unlock_irqrestore_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_lock_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_lock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_lock_irqsave_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_unlock_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_unlock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_rw_write_unlock_irqrestore_fcn);
EXPORT_SYMBOL_NOVERS(lis_rx_netbuff);
EXPORT_SYMBOL_NOVERS(lis_safe_canenable);
EXPORT_SYMBOL_NOVERS(lis_safe_enableok);
EXPORT_SYMBOL_NOVERS(lis_safe_noenable);
EXPORT_SYMBOL_NOVERS(lis_safe_OTHERQ);
EXPORT_SYMBOL_NOVERS(lis_safe_putmsg);
EXPORT_SYMBOL_NOVERS(lis_safe_putnext);
EXPORT_SYMBOL_NOVERS(lis_safe_qreply);
EXPORT_SYMBOL_NOVERS(lis_safe_RD);
EXPORT_SYMBOL_NOVERS(lis_safe_SAMESTR);
EXPORT_SYMBOL_NOVERS(lis_safe_WR);
EXPORT_SYMBOL_NOVERS(lis_secs);
EXPORT_SYMBOL_NOVERS(lis_sem_alloc);
EXPORT_SYMBOL_NOVERS(lis_sem_destroy);
EXPORT_SYMBOL_NOVERS(lis_sem_init);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
EXPORT_SYMBOL_NOVERS(lis_sleep_on);
#endif
EXPORT_SYMBOL_NOVERS(lis_sleep_on_timeout);
EXPORT_SYMBOL_NOVERS(lis_wait_event);
EXPORT_SYMBOL_NOVERS(lis_wait_event_interruptible);
EXPORT_SYMBOL_NOVERS(lis_spin_is_locked_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_alloc_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_free_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_init_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_lock_irqsave_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_trylock_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_unlock_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_unlock_irq_fcn);
EXPORT_SYMBOL_NOVERS(lis_spin_unlock_irqrestore_fcn);
EXPORT_SYMBOL_NOVERS(lis_spl0_fcn);
EXPORT_SYMBOL_NOVERS(lis_splstr_fcn);
EXPORT_SYMBOL_NOVERS(lis_splx_fcn);
EXPORT_SYMBOL_NOVERS(lis_sprintf);
EXPORT_SYMBOL_NOVERS(lis_strm_name);
EXPORT_SYMBOL_NOVERS(lis_strm_name_from_queue);
EXPORT_SYMBOL_NOVERS(lis_stropts_file);
EXPORT_SYMBOL_NOVERS(lis_strqget);
EXPORT_SYMBOL_NOVERS(lis_strqset);
EXPORT_SYMBOL_NOVERS(lis_testb);
EXPORT_SYMBOL_NOVERS(lis_thread_start);
EXPORT_SYMBOL_NOVERS(lis_thread_stop);
EXPORT_SYMBOL_NOVERS(lis_timeout_fcn);
EXPORT_SYMBOL_NOVERS(lis_touch_softlockup_watchdog);
EXPORT_SYMBOL_NOVERS(lis_udelay);
EXPORT_SYMBOL_NOVERS(lis_unbufcall);
EXPORT_SYMBOL_NOVERS(lis_unfreezestr);
EXPORT_SYMBOL_NOVERS(lis_unlink);
EXPORT_SYMBOL_NOVERS(lis_unlinkb);
EXPORT_SYMBOL_NOVERS(lis_unregister_netdev);
EXPORT_SYMBOL_NOVERS(lis_unregister_strdev);
EXPORT_SYMBOL_NOVERS(lis_unregister_strmod);
EXPORT_SYMBOL_NOVERS(lis_untimeout);
EXPORT_SYMBOL_NOVERS(lis_up_fcn);
EXPORT_SYMBOL_NOVERS(lis_usecs);
EXPORT_SYMBOL_NOVERS(lis_usectohz);
EXPORT_SYMBOL_NOVERS(lis_version);
EXPORT_SYMBOL_NOVERS(lis_vfree);
EXPORT_SYMBOL_NOVERS(lis_virt_to_phys);
EXPORT_SYMBOL_NOVERS(lis_vmalloc);
EXPORT_SYMBOL_NOVERS(lis_vremap);
EXPORT_SYMBOL_NOVERS(lis_vsprintf);
EXPORT_SYMBOL_NOVERS(lis_wake_up);
EXPORT_SYMBOL_NOVERS(lis_wake_up_interruptible);
EXPORT_SYMBOL_NOVERS(lis_xmsgsize);
EXPORT_SYMBOL_NOVERS(lis_zmalloc);

#if (!defined(_S390_LIS_) && !defined(_S390X_LIS_))

EXPORT_SYMBOL_NOVERS(lis_osif_pci_alloc_consistent);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_dma_supported);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_dma_sync_sg);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_dma_sync_single);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_free_consistent);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_map_sg);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_map_single);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_set_dma_mask);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_set_consistent_dma_mask);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_unmap_sg);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_unmap_single);
EXPORT_SYMBOL_NOVERS(lis_osif_sg_dma_address);
EXPORT_SYMBOL_NOVERS(lis_osif_sg_dma_len);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,13)      /* 2.4.13 or later */
EXPORT_SYMBOL_NOVERS(lis_osif_pci_unmap_page);
EXPORT_SYMBOL_NOVERS(lis_osif_pci_map_page);
#endif                                  /* 2.4.13 */

/*
 * Wrapper functions
 */
EXPORT_SYMBOL_NOVERS(__wrap_strcpy);
EXPORT_SYMBOL_NOVERS(__wrap_strncpy);
EXPORT_SYMBOL_NOVERS(__wrap_strcat);
EXPORT_SYMBOL_NOVERS(__wrap_strncat);
EXPORT_SYMBOL_NOVERS(__wrap_strcmp);
EXPORT_SYMBOL_NOVERS(__wrap_strncmp);
EXPORT_SYMBOL_NOVERS(__wrap_strnicmp);
EXPORT_SYMBOL_NOVERS(__wrap_strchr);
EXPORT_SYMBOL_NOVERS(__wrap_strrchr);
EXPORT_SYMBOL_NOVERS(__wrap_strstr);
EXPORT_SYMBOL_NOVERS(__wrap_strlen);
EXPORT_SYMBOL_NOVERS(__wrap_memset);
EXPORT_SYMBOL_NOVERS(__wrap_memcpy);
EXPORT_SYMBOL_NOVERS(__wrap_memcmp);
EXPORT_SYMBOL_NOVERS(__wrap_readb);
EXPORT_SYMBOL_NOVERS(__wrap_readw);
EXPORT_SYMBOL_NOVERS(__wrap_readl);
EXPORT_SYMBOL_NOVERS(__wrap_writeb);
EXPORT_SYMBOL_NOVERS(__wrap_writew);
EXPORT_SYMBOL_NOVERS(__wrap_writel);


#endif          /* S390 or S390X */
