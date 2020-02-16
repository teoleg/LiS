/*************************************************************************
 *
 *  Copyright (C)  2007 Adax, Inc
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 * 
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 * 
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the
 *  Free Software Foundation, Inc., 59 Temple Place - Suite 330, Cambridge,
 *  MA 02139, USA.
 * 
 *************************************************************************/

#include <sys/LiS/module.h>	/* must be VERY first include */

#include <sys/stream.h>

#include <asm/param.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/sched.h> 
#include <linux/kernel.h> /* printk() */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/interrupt.h> /* mark_bh */
#include <linux/netdevice.h>   /* struct net_device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/ip.h>          /* struct iphdr */
/* pr_debug was broken in SuSE 11.1.  Trying to use dynamic_pr_debug.. sigh.*/
#undef pr_debug
#define pr_debug(fmt,arg...) printk(KERN_DEBUG fmt,##arg)
#include <linux/tcp.h>         /* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <net/arp.h>

#include <sys/dlpi.h>
#include <sys/lismem.h>
#include <sys/lislocks.h>
#include <sys/lisnet.h>

//#define DANDEBUG
#ifdef DANDEBUG
#define DDEBUG(fmt, args...) printk("%s:%d:" fmt, __FILE__, __LINE__, ## args)
#else
#define DDEBUG(fmt, args...)
#endif

#ifndef WARN_ON
#define WARN_ON(a)
#endif

extern struct qinit strmhd_rdinit;
extern struct qinit strmhd_wrinit;
extern int lis_qattach( stdata_t *head, struct streamtab *info, dev_t *devp,
			int flags, cred_t *credp );
extern void lis_qdetach(queue_t *q, int do_close, int flag, cred_t *creds);
extern void lis_dismantle(stdata_t *hd, cred_t *creds);
extern void mark_closing(queue_t *sd_wq);

struct lis_net_status {
	int myminor;       /* store getminor result after open_callback */
	int instance;	   /* PPA (ie eth2 is ppa 2) */
	queue_t *lowerq;   /* lower write queue corresponding to minor */
	struct lis_net_config config;
	struct net_device_stats net_stats;
	struct stdata *head;
};

int lis_net_open(struct net_device *dev) 
{

/* ### TEMP */
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
	struct lis_net_config *config = &priv->config;
	struct stdata *head;
	unsigned int devx;
	unsigned int minor;
	unsigned int major;
	int err;
	queue_t *q;

	/* Allocate a new stream and open the driver */
	head = lis_head_get(NULL);

	head->sd_strtab = config->sd_strtab;
	head->sd_rdopt = RPROTNORM | RNORM ;
	head->sd_wropt = 0;
	head->sd_maxpsz = LIS_MAXPSZ;
	head->sd_minpsz	= LIS_MINPSZ;
	head->sd_closetime = LIS_CLTIME;
	head->sd_dev = 0 ;
	head->sd_open_flags = 0;
	priv->head = head;

	SET_SD_FLAG(head, STWOPEN);
	lis_setq(head->sd_rq, &strmhd_rdinit, &strmhd_wrinit );
	/* This will allocate new queues for the driver and call it's
	 * open routine */
	major = config->major;
	devx = makedevice(major, 0);

	DDEBUG("lis_net_open: calling lis_qattach major=0x%x devx=0x%x\n",
		major, devx);
	lis_qattach(head, head->sd_strtab, &devx, CLONEOPEN, NULL);

	major = getmajor(devx);
	minor = getminor(devx);

	if ((err = lis_set_q_sync(head->sd_wq, LIS_QLOCK_NONE)))
		{
		printk(
	"lis_net_open: ERROR: failed to set qsync for HEAD err=%d\n", 
			err);
		return(-ENXIO);
		}
	if ((err = lis_set_q_sync(head->sd_wq->q_next,
			     LIS_DEVST(major).f_qlock_option)))
		{
		printk(
 "lis_net_open: ERROR: failed to set qsync for major=%d minor=%d err=%d\n", 
			major, minor, err);
		return(-ENXIO);
		}

	CLR_SD_FLAG(head, STWOPEN) ;

	check_for_wantenable(head) ;	/* deferred queue enables */

	q = head->sd_wq->q_next;
	priv->lowerq = q;
	DDEBUG("lis_net_open: dev=0x%p q=0x%p major=%d minor=%d\n", 
		dev, q, major, minor);
	/* Tell the driver what ethernet instance is being opened */
	if (config->open_callback)
		config->open_callback(priv->lowerq, minor, priv->instance);

	priv->myminor = minor;
	
	WARN_ON(irqs_disabled());
	return 0;
}

int lis_net_release(struct net_device *dev)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config = &priv->config;
	struct stdata *head;

	head = priv->head;

	DDEBUG("lis_net_release: dev=0x%p\n", dev);
	if (config->stop_callback)
		config->stop_callback(priv->lowerq, priv->myminor, 
					priv->instance);
	
	SET_SD_FLAG(head, STRCLOSE);
	mark_closing(head->sd_wq);
	lis_qdetach(LIS_RD(head->sd_wq->q_next), 1, 0, NULL);
	lis_dismantle(head, NULL);
	lis_head_put(head);
	
	priv->head = NULL;
	priv->lowerq = NULL;
	WARN_ON(irqs_disabled());
	return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 * Not that we actually do anything with them.
 */
int lis_net_config(struct net_device *dev, struct ifmap *map)
{
	DDEBUG("lis_net_config: dev=0x%p\n", dev);
	return 0;
}

void _RP lis_dev_kfree_skb(char *arg)
{
	struct sk_buff *skb;
	skb = (struct sk_buff*) arg;
	DDEBUG("lis_dev_free_skb: freeing skb 0x%p\n", skb);
	dev_kfree_skb(skb);
}

/*
 * Transmit a packet (called by the kernel)
 */
int lis_net_tx(struct sk_buff *skb, struct net_device *dev)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *privp = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *privp = (struct lis_net_status *)dev->priv;
#endif
       
	struct ethhdr *eth;
	frtn_t freeskb;
	queue_t *q;
	mblk_t *mp, *nmp;
	dl_unitdata_req_t * req;
	int len;

	DDEBUG("lis_net_tx: skb=0x%p dev=0x%p\n", skb, dev);
	if (unlikely(skb == NULL))
		return(0);

	len = skb->len;
	eth = (struct ethhdr *)skb->data;

	/* sanity check */
	if (unlikely(len < sizeof(struct ethhdr) + sizeof(struct iphdr)))
		{
		printk(
	"lis_net_tx: Error... packet too short (%i octets). Discarding...\n",
		       len);
		dev_kfree_skb(skb);
		return(0);
		}

	dev->trans_start = jiffies; /* save the timestamp */
	q = privp->lowerq;

	if ((mp = allocb(sizeof(struct ethhdr)+DL_UNITDATA_REQ_SIZE, BPRI_LO)) 
		== NULL)
		{
		printk("lis_net_tx: failed: allocb failed");
		dev_kfree_skb(skb);
		return(0);
		}
	req = (dl_unitdata_req_t *) mp->b_rptr;
	req->dl_primitive = DL_UNITDATA_REQ;
	req->dl_dest_addr_length = sizeof(struct ethhdr);
	req->dl_dest_addr_offset = DL_UNITDATA_REQ_SIZE;
	req->dl_priority.dl_min = skb->priority;
	req->dl_priority.dl_max = skb->priority;

	mp->b_datap->db_type = M_PROTO;  
	mp->b_wptr += DL_UNITDATA_REQ_SIZE;
	/* ZZZ do this or just leave the ethhdr attached at the front? */
	bcopy(eth, mp->b_wptr, sizeof(struct ethhdr));
	mp->b_wptr += sizeof(struct ethhdr);

	freeskb.free_func = lis_dev_kfree_skb;
	freeskb.free_arg = (char*) skb;

	if ((nmp = esballoc(skb->data, len, BPRI_LO, &freeskb)) == NULL)
		{
		printk("lis_net_tx: failed: esballoc failed");
		freemsg(mp);
		dev_kfree_skb(skb);
		return(0);
		}
	nmp->b_wptr = nmp->b_rptr + len;
	linkb(mp, nmp);
	DDEBUG("lis_net_tx: skb=0x%p mp=0x%p cont=0x%p len=%d\n",
		skb, mp, mp->b_cont, mp->b_cont->b_wptr - mp->b_cont->b_rptr);
	/* We have to rely on the driver to provide tx side flow control */
	lis_putqf(q, mp);

#ifdef NETDEV_TX_OK
	return NETDEV_TX_OK;
#else
	return 0;
#endif
}

/*
 * Ioctl commands 
 */
int lis_net_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config = &priv->config;
	
	DDEBUG("lis_net_do_ioctl: dev=0x%p rq=0x%p cmd=0x%x\n", dev, rq, cmd);
	if (config->do_ioctl_callback)
		/* ZZZZ how to do this one? */
		return(config->do_ioctl_callback(priv->lowerq, NULL, cmd));
	else
		return(-EINVAL);
}

/*
 * Return statistics to the caller
 */
struct net_device_stats * lis_net_stats(struct net_device *dev)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config;
	struct lis_net_stats stats;

	DDEBUG("lis_net_stats: dev=0x%p priv=0x%p\n", dev, priv);
 	config = &priv->config;
	DDEBUG("lis_net_stats: dev=0x%p priv=0x%p config=0x%p\n", 
		dev, priv, config);
	if (config->get_stats_callback)
		config->get_stats_callback(priv->lowerq, &stats);
	
	priv->net_stats.rx_packets = stats.rx_packets;
	priv->net_stats.tx_packets = stats.tx_packets;
	priv->net_stats.rx_bytes = stats.rx_bytes;
	priv->net_stats.tx_bytes = stats.tx_bytes;
	priv->net_stats.rx_errors = stats.rx_errors;
	priv->net_stats.tx_errors = stats.tx_errors;
	priv->net_stats.rx_dropped = stats.rx_dropped;
	priv->net_stats.tx_dropped = stats.tx_dropped;
	priv->net_stats.multicast = stats.multicast;
	priv->net_stats.collisions = stats.collisions;

	priv->net_stats.rx_length_errors = stats.rx_length_errors;
	priv->net_stats.rx_over_errors = stats.rx_over_errors;
	priv->net_stats.rx_crc_errors = stats.rx_crc_errors;
	priv->net_stats.rx_frame_errors = stats.rx_frame_errors;
	priv->net_stats.rx_fifo_errors = stats.rx_fifo_errors;
	priv->net_stats.rx_missed_errors = stats.rx_missed_errors;

	priv->net_stats.tx_aborted_errors = stats.tx_aborted_errors;
	priv->net_stats.tx_carrier_errors = stats.tx_carrier_errors;
	priv->net_stats.tx_fifo_errors = stats.tx_fifo_errors;
	priv->net_stats.tx_heartbeat_errors = stats.tx_heartbeat_errors;
	priv->net_stats.tx_window_errors = stats.tx_window_errors;

	priv->net_stats.rx_compressed = stats.rx_compressed;
	priv->net_stats.tx_compressed = stats.tx_compressed;
	/* ZZZ etc... */
	WARN_ON(irqs_disabled());
	return &priv->net_stats;
}

/*
 * The "change_mtu" method is usually not needed.
 * If you need it, it must be like this.
 *
 * The change in MTU (Maximum Transfer Unit)
 * must be communicated to xinet
 */
int lis_net_change_mtu(struct net_device *dev, int new_mtu)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config = &priv->config;

	DDEBUG("lis_net_change_mtu: dev=0x%p mtu=%d\n", dev, new_mtu);
	if (config->change_mtu_callback)
		return(config->change_mtu_callback(priv->lowerq, new_mtu));
	else
		return(-EINVAL);
}

void lis_net_set_multicast_list(struct net_device *dev)
{
/* ### TEMP */
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config = &priv->config;
	struct dev_mc_list *list;
	struct lis_net_mc_list lislist[64];
	int count, i;
	unsigned int flags;

	list = dev->mc_list;
	count = min(dev->mc_count, 64);

	for (i = 0; i < count && list; i++, list = list->next)
		bcopy(list->dmi_addr, lislist[i].addr, 6);

	flags = 0;
	if (dev->flags & IFF_ALLMULTI)
		flags |= LIS_IFF_ALLMULTI;
	if (dev->flags & IFF_PROMISC)
		flags |= LIS_IFF_PROMISC;
	
	DDEBUG("lis_net_set_multicast_list: dev=0x%p flags=0x%x count=%d\n", 
		dev, flags, count);

	if (config->set_multicast_list_callback)
		config->set_multicast_list_callback(priv->lowerq, 
						flags, lislist, count);
	
	WARN_ON(irqs_disabled());
}

int lis_net_set_mac_address(struct net_device *dev, void *p)
{
#if defined(KERNEL_2_5)
        struct lis_net_status *priv = (struct lis_net_status *)netdev_priv(dev);
#else
	struct lis_net_status *priv = (struct lis_net_status *)dev->priv;
#endif
        
	struct lis_net_config *config = &priv->config;
	struct sockaddr *addr = p;

	DDEBUG("lis_net_set_mac_address: dev=0x%p p=0x%p callback=0x%p\n", 
		dev, p, config->set_mac_address_callback);
	if (!is_valid_ether_addr(addr->sa_data))
		{
		DDEBUG("lis_net_set_mac_addr: bad address! %x:%x:%x:%x:%x:%x\n",
			addr->sa_data[0],
			addr->sa_data[1],
			addr->sa_data[2],
			addr->sa_data[3],
			addr->sa_data[4],
			addr->sa_data[5]);
		return(-EADDRNOTAVAIL);
		}

	if (config->set_mac_address_callback)
		return(config->set_mac_address_callback(priv->lowerq, 
						addr->sa_data));
	else
		return(-EINVAL);
}
	
/* name: name string for interface */
/* len: length of name string */
/* inst: instance number of this interface */
/* q: pointer to driver's write queue */
/* ZZZ we need a way to pass in if the dgrams should be checksumed or not */
void* _RP lis_alloc_etherdev(char *name, int len, unsigned int inst, 
			     struct lis_net_config *configp)
{
	struct net_device *dev;
	struct lis_net_status *privp;
	struct lis_net_config *cfgp;
	struct pci_dev *pci_dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
	static const struct net_device_ops lisnet_netops = {
	.ndo_open		= lis_net_open,
	.ndo_stop		= lis_net_release,
	.ndo_start_xmit		= lis_net_tx,
	.ndo_get_stats		= lis_net_stats,
	.ndo_set_rx_mode	= lis_net_set_multicast_list,
	.ndo_set_mac_address	= lis_net_set_mac_address,
	.ndo_change_mtu		= lis_net_change_mtu,
	.ndo_do_ioctl		= lis_net_do_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
};
#endif

	/* Make sure that the name is not too long */
	if (len > 15 && inst < 10)
		return(NULL);
	if (len > 14 && inst >= 10)
		return(NULL);

	dev = alloc_etherdev(sizeof(struct lis_net_status));
	if (!dev)
		return(NULL);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30))
	dev->open		= lis_net_open;
	dev->stop		= lis_net_release;
	dev->hard_start_xmit	= lis_net_tx;
	dev->get_stats		= lis_net_stats;
	dev->set_multicast_list	= lis_net_set_multicast_list;  
	dev->set_mac_address	= lis_net_set_mac_address;  
	dev->change_mtu		= lis_net_change_mtu;  
	dev->do_ioctl		= lis_net_do_ioctl;

#else
	dev->netdev_ops = &lisnet_netops;
#endif

	dev->addr_len		= configp->addr_len;
	dev->tx_queue_len	= configp->tx_queue_len;

	if (name)
		memcpy(dev->name, name, len);
	if (inst < 10)
		dev->name[len] = '0' + inst;
	else
		{
		dev->name[len++] = '0' + inst/10;
		dev->name[len] = '0' + inst%10;
		}
	DDEBUG("LiS: lis_alloc_etherdev: inst %d name=%s\n", inst, name);
	/* The priv field is allocated by alloc_etherdev() */

#if defined(KERNEL_2_5)
	privp = netdev_priv(dev);
#else
	privp = dev->priv;
#endif
	cfgp = &privp->config;
	DDEBUG("LiS: lis_alloc_etherdev: dev=0x%p privp=0x%p cfgp=0x%p\n", 
		dev, privp, cfgp);
	if (configp->open_callback)
		cfgp->open_callback = configp->open_callback;
	if (configp->stop_callback)
		cfgp->stop_callback = configp->stop_callback;
	if (configp->get_stats_callback)
		cfgp->get_stats_callback = configp->get_stats_callback;
	if (configp->set_multicast_list_callback)
		cfgp->set_multicast_list_callback = 
			configp->set_multicast_list_callback;
	if (configp->set_mac_address_callback)
		cfgp->set_mac_address_callback = 
			configp->set_mac_address_callback;
	if (configp->change_mtu_callback)
		cfgp->change_mtu_callback = configp->change_mtu_callback;
	if (configp->do_ioctl_callback)
		cfgp->do_ioctl_callback = configp->do_ioctl_callback;
	cfgp->sd_strtab = configp->sd_strtab;
	cfgp->major = configp->major;
	cfgp->lis_pci_dev = configp->lis_pci_dev;
	pci_dev = (struct pci_dev *) cfgp->lis_pci_dev->kern_ptr;
	if (!is_valid_ether_addr(configp->dev_addr))
		{
		DDEBUG(
 "lis_alloc_etherdev: Invalid MAC address %02X:%02X:%02X:%02X:%02X:%02X. Returning NULL.\n", 
			configp->dev_addr[0], configp->dev_addr[1],
			configp->dev_addr[2], configp->dev_addr[3],
			configp->dev_addr[4], configp->dev_addr[5]);
		free_netdev(dev);
		return(NULL);
		}
	memcpy(&cfgp->dev_addr, configp->dev_addr, 6);
	privp->instance = inst;

#if !defined(KERNEL_2_5)
	/* ZZZZZ */
	dev_init_buffers(dev);
#endif

	SET_NETDEV_DEV(dev, &pci_dev->dev);
	memcpy(dev->dev_addr, cfgp->dev_addr, 6);

	WARN_ON(irqs_disabled());
	return ((void*) dev);
}

int _RP lis_register_netdev(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	return (register_netdev(dev));
}

void _RP lis_unregister_netdev(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;

	DDEBUG("lis_unregister_netdev: unregistering dev=0x%p\n", dev);
	unregister_netdev(dev);
	free_netdev(dev);
}

void _RP lis_free_netdev(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	free_netdev(dev);
}

void * _RP lis_dev_alloc_skb(void *devx, int len, unsigned char **data)
{
	struct sk_buff *skb;
	struct net_device *dev = (struct net_device*) devx;
	skb = dev_alloc_skb(len);
	if (skb)
		skb->dev = dev;
//	/* This is to align the IP header to a word boundry */
//	skb_reserve(skb, 2);
	/* ZZZ we rely on the driver to do this for us... */
	*data = skb->data;
	return((void*)skb);
}

void _RP lis_rx_netbuff(void *devx, void *skbptr, unsigned int len)
{
	struct sk_buff *skb;
	struct net_device *dev = (struct net_device*) devx; 

	skb = (struct sk_buff*) skbptr;
	skb_put(skb, len);
	skb->protocol = eth_type_trans(skb, dev);
	netif_receive_skb(skb);
}

void _RP lis_netif_carrier_off(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	netif_carrier_off(dev);
}

void _RP lis_netif_carrier_on(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	netif_carrier_on(dev);
}

int _RP lis_netif_carrier_ok(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	return(netif_carrier_ok(dev));
}

void _RP lis_netif_wake_queue(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	netif_wake_queue(dev);
}

void _RP lis_netif_stop_queue(void *devx)
{
	struct net_device *dev = (struct net_device*) devx;
	netif_stop_queue(dev);
}
