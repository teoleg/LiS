/* Copyright Adax, Inc (c) 2008 */
/* net.h
 * Utilities for STREAMS drivers to directly interface to the Linux IP 
 * subsystem */

#ifndef _LIS_NET_H
#define _LIS_NET_H

#include <sys/lispci.h>

#define LIS_IFF_ALLMULTI 0x200	/* Match IFF_ALLMULTI */
#define LIS_IFF_PROMISC	 0x100	/* Match IFF_PROMISC */

struct lis_net_stats {
	unsigned long	rx_packets;	/* total packets received	*/
	unsigned long	tx_packets;	/* total packets transmitted	*/
	unsigned long	rx_bytes;	/* total bytes received 	*/
	unsigned long	tx_bytes;	/* total bytes transmitted	*/
	unsigned long	rx_errors;	/* bad packets received		*/
	unsigned long	tx_errors;	/* packet transmit problems	*/
	unsigned long	rx_dropped;	/* no space in linux buffers	*/
	unsigned long	tx_dropped;	/* no space available in linux	*/
	unsigned long	multicast;	/* multicast packets received	*/
	unsigned long	collisions;

/* detailed rx_errors: */
	unsigned long	rx_length_errors;
	unsigned long	rx_over_errors;	/* receiver ring buff overflow	*/
	unsigned long	rx_crc_errors;	/* recved pkt with crc error	*/
	unsigned long	rx_frame_errors;/* recv'd frame alignment error */
	unsigned long	rx_fifo_errors;	/* recv'r fifo overrun		*/
	unsigned long	rx_missed_errors;/* receiver missed packet	*/

	/* detailed tx_errors */
	unsigned long	tx_aborted_errors;
	unsigned long	tx_carrier_errors;
	unsigned long	tx_fifo_errors;
	unsigned long	tx_heartbeat_errors;
	unsigned long	tx_window_errors;
	
	/* for cslip etc */
	unsigned long	rx_compressed;
	unsigned long	tx_compressed;

};

struct lis_net_mc_list {
	unsigned char addr[6];
};

struct lis_net_config {
	struct streamtab *sd_strtab;
	unsigned short flags;	/* pass to the net_device structure field */
	unsigned short type;	/* pass to the net_device structure field */
	unsigned int major;
	unsigned int addr_len;	/* pass to the net_device structure field */
	unsigned long tx_queue_len;/* pass to the net_device structure field */
	lis_pci_dev_t *lis_pci_dev;
	char dev_addr[6];	/* default MAC address */
	void _RP (*open_callback)(queue_t*, int, int);
	void _RP (*stop_callback)(queue_t*, int, int);
	void _RP (*get_stats_callback)(queue_t*, struct lis_net_stats*);
	void _RP (*set_multicast_list_callback)(queue_t*, unsigned int, 
					struct lis_net_mc_list *, unsigned int);
	int _RP (*set_mac_address_callback)(queue_t*, void*);
	int _RP (*change_mtu_callback)(queue_t*, int);
	int _RP (*do_ioctl_callback)(queue_t*, void*, int);
};

extern void * lis_dev_alloc_skb(void *devx, int len, unsigned char **data) _RP __attribute__((weak));
extern void lis_rx_netbuff(void *devx, void *skbptr, unsigned int len) _RP __attribute__((weak));
extern void lis_unregister_netdev(void *devx) _RP __attribute__((weak));
extern int lis_register_netdev(void *devx) _RP __attribute__((weak));
extern void * lis_alloc_etherdev(char *name, int len, unsigned int inst, 
				struct lis_net_config *configp) _RP __attribute__((weak));
extern void lis_free_netdev(void *devx) _RP __attribute__((weak));
extern void lis_netif_carrier_off(void *devx) _RP __attribute__((weak));
extern void lis_netif_carrier_on(void *devx) _RP __attribute__((weak));
extern int lis_netif_carrier_ok(void *devx) _RP __attribute__((weak));
extern void lis_netif_wake_queue(void *devx) _RP __attribute__((weak));
extern void lis_netif_stop_queue(void *devx) _RP __attribute__((weak));
#endif
