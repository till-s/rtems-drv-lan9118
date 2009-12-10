/* $Id$ */

#ifndef LAN_IP_BASIC_H
#define LAN_IP_BASIC_H

/* Basic IP/UDP socket */

/* IMPORTANT NOTES REGARDING THREAD-SAFETY 
 *
 * The implementation serializes
 *   - creation/destruction of sockets, i.e., two
 *     threads may e.g., simultaneously create two sockets
 *     but note that there is NO PROTECTION against 
 *     one thread deleting a socket while a send or
 *     receive operation on the same socket (by a 
 *     different thread)  is in progress.
 *     Such a scenario is is considered a programming
 *     error.
 *     Deletion of a socket is only permissible if it
 *     is 'not in use'. This restriction exists for
 *     efficiency-reasons.
 *
 *   - sending on a single socket by multiple threads
 *
 *   - receiving and sending by different threads
 *     as well as receiving by multiple threads (but
 *     that rarely makes sense). However, there exists
 *     a race condition if different threads 
 *     (simultaneously) use the udpSockRecv() and
 *     updSockNRead() routines (see below).
 *
 * Note also that it is not safe nor permissible to
 * destroy an interface that is still used by any
 * socket. All sockets using an interface must have
 * been destroyed before the interface can be deleted.
 */ 

#include <lanIpProto.h>
#include <rtems.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Create a socket listening on a UDP port
 * RETURNS: descriptor (>=0) on success, < 0 on error
 *
 * NOTES: - port is in *host* byte order.
 *        - submitting port == 0 lets the library
 *          assign an available port #
 */
int
udpSockCreate(int port);

/* Destroy socket;
 * RETURNS: 0 on success, nonzero on error.
 *
 * NOTE: destroying a socket somebody is blocking on is BAD
 */
int
udpSockDestroy(int sd);

/* Query number of bytes queued on a socket (UDP payload only)
 *
 * Note that there is a race condition if 
 * udpSockRecv() and udpSockNRead() are used
 * from different thread contexts.
 * One thread could have dequeued a packet
 * and be put to sleep before it has
 * a chance to decrease 'nbytes' so that
 * a second thread calling udpSockNRead
 * would believe that the 'nbytes' are still
 * in the queue.
 * However, in such an environment the user
 * must implement locking anyways since
 * a sequence of
 *   udpSockNRead()
 *   udpSockRecv()
 * would not be atomic.
 *
 * Several threads just using udpSockRecv() can
 * safely share a socket (but the usefulness of
 * this is of course questionable).
 *
 * RETURNS: Total number of bytes available on socket
 *          or a value < 0 if there was an error.
 *
 */
int
udpSockNRead(int sd);

/* Alloc and free buffers */
LanIpPacketRec *
udpSockGetBuf();

void
udpSockFreeBuf(LanIpPacketRec *ppacket);

/* Read packet from a socket
 *
 * 'timeout_ticks': how long to block
 *                  <0 forever
 *                   0 not at all
 *                  >0 # ticks to wait
 * RETURNS: packet or NULL on error
 *
 * NOTE:    packet must be released with udpSockFreeBuf()
 */
LanIpPacketRec *
udpSockRecv(int sd, int timeout_ticks);

/* 'Connect' to a peer, i.e., fill in a preallocated header
 * structure that is re-used for every 'Send' operation.
 * Also, datagrams are only accepted from the connected peer
 * (IP and source port; if the peer IP is a broadcast address
 * then only the source port is filtered).
 *
 * 'dipaddr': peer's IP address (*network* byte order)
 * 'dport'  : peer's UDP destination port (*host* byte order)
 *
 * RETURNS: 0 on success -errno on error.
 *
 * NOTES: - 'dipaddr' == 0 and 'dport' == 0 may be passed
 *          to 'disconnect' the socket.
 *
 *        - call fails if ARP lookup of 'dipaddr' fails
 *          (this is unlike BSD 'connect' for UDP sockets 
 *          IIRC).
 */
int
udpSockConnect(int sd, uint32_t dipaddr, int dport);

/* Send data over a 'connected' socket.
 *
 * RETURNS: number of bytes sent or -errno.
 *
 * NOTE:    payload buffer is *not* taken over by the
 *          stack but copied.
 */
int
udpSockSend(int sd, void *payload, int payload_len);

/* Send data to arbitrary destination.
 *
 * RETURNS: number of bytes sent or -errno.
 *
 * NOTES: - payload buffer is *not* taken over by the
 *          stack but copied.
 *        - use udpSendTo() when using a 'connected'
 *          socket (except if 'dipaddr'/'dport' match
 *          the peer).
 */
int
udpSockSendTo(int sd, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport);

int
_udpSockSendTo_internal(int sd, LanIpPacket buf_p, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport);

static __inline__ void*
udpSockUdpBufPayload(LanIpPacket p)
{
	return lpkt_udp_hdrs(p).pld;
}

/* Send data w/o copying the payload.
 * These entry points are just like
 * udpSockSend() and udpSockSendTo() but the
 * payload is passed in a buffer that previously
 * had been obtained from udpSockGetBuf().
 * 
 * Note that the payload has to be stored
 * into the buffer's payload area, i.e.,
 * use udpSockUdpBufPayload() to obtain a pointer.
 */
int
udpSockSendBuf(int sd, LanIpPacket b, int payload_len);

int
udpSockSendBufTo(int sd, LanIpPacket b, int payload_len, uint32_t ipaddr, uint16_t dport);


extern uint32_t udpSockMcastIfAddr;

/*
 * Choose interface for outgoing multicast traffic
 */
int
udpSockSetIfMcast(int sd, uint32_t ifipaddr);

/*
 * Read and set the value of the 'multicast-loopback' flag.
 * This flag defines whether locally sent multicast packets
 * are looped-back to local sockets.
 *
 * RETURNS: Previous value of the flag (>=0) or a negative
 *          error status.
 *
 * NOTE:    If 'val' < 0 then the value is not actually set.
 *          This can be used to read the current state of the
 *          flag.
 */
int
udpSockSetMcastLoopback(int sd, int val);

/*
 * Join and leave a multicast group.
 */
int
udpSockJoinMcast(int sd, uint32_t mcaddr);

int
udpSockLeaveMcast(int sd, uint32_t mcaddr);

/* Create private data (pass as rx callback closure pointer to drvLan9118Start)
 * 
 * This can be thought of as (and should better be called) an 'interface handle'.
 * RETURNS handle on success, NULL on failure.
 */

typedef struct IpBscIfRec_ *IpBscIf;
typedef void               *LanIpBscDrv;

/* Create and Setup private data structure (AKA 'interface handle') */
IpBscIf
lanIpBscIfCreate(LanIpBscDrv drv_p, char *ipaddr, char *netmask);

/* Retrieve the driver handle   */
LanIpBscDrv
lanIpBscIfGetDrv(IpBscIf ipbif_p);

/* Retrieve interface handle of a socket */
IpBscIf
udpSockGetIf(int sd);

/* Retrieve interface where a packet was received; 
 * calling this on a new buffer yields NULL.
 */
IpBscIf
udpSockGetBufIf(LanIpPacket buf_p);

/* Tear down interface handle and release all resources associated
 * with it (but *not* the driver). The interface and driver handles
 * are separate objects ('drv_p' passed to lanIpBscIfCreate() is only
 * a reference to the associated driver stored in the interface data
 * structure for convenience).
 * Usually, the driver must be shut-down prior to destroying the
 * interface handle.
 */
int
lanIpBscIfDestroy(IpBscIf);

/* Operations on packet headers: */

/* Setup Ethernet, IP and UDP headers in a packet.
 * An ARP lookup for 'dipaddr' is done if necessary.
 *
 * NOTES: 'dipaddr' (destination IP address) is in *network* byte order.
 *        'dport', 'sport' and 'ip_id' are in *host* byte order.
 *
 *        It is legal to provide an all zero destination IP to avoid
 *        ARP lookup. You must fill in the IP and checksum later.
 *
 * RETURNS: 0 on success, -errno on error (-ENOTCONN == ARP lookup failure)
 */
int
udpSockHdrsInit(int sd, LanUdpPkt p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id);

/* Similar to udpSockHdrsInit() but w/o requirement for a 'socket' so that
 * this can be used from low-level code.
 */
int
udpSockHdrsInitFromIf(IpBscIf intrf, LanUdpPkt p, uint32_t dipaddr, uint16_t dport, uint16_t sport, uint16_t ip_id);

/* Set length and IP checksum
 * Note: other fields must have been initialized already
 */
void
udpSockHdrsSetlen(LanUdpPkt p, int payload_len);

/* Flip source -> dest and fill-in local source addresses
 * (at ethernet, IP and UDP level) and IP checksum.
 */
void
udpSockHdrsReflect(LanUdpPkt p);

/* Send a buffer; EVERYTHING (all headers + payload must have been filled in)
 * len: total length (including all headers and initial 2-byte padding).
 * The buffer is taken over by the stack and released eventually.
 *
 * RETURNS: len
 */
int
lanIpBscSendBufRaw(IpBscIf intrf, LanIpPacket buf_p, int len);

/* Like SendBufRaw but assumes the packet is a UDP packet so that
 * the length can be extracted from the IP header.
 */
int
lanIpBscSendBufRawIp(IpBscIf intrf, LanIpPacket buf_p);

/* The ARP interface.
 * 
 * NOTES: >>> ARP replies (to other hosts) are always generated and sent by
 *        the lanIp daemon [as a result of requests from other hosts].
 *        If you don't want the transmitter to send ARP replies then
 *        you should make sure noone issues ARP requests.
 *
 *        >>> ARP lookups are ONLY performed under two circumstances:
 *        a) when filling a 'destination IP' (non-null, non-bcst) address
 *        into a header (udpSockHdrsInit())
 *
 *        b) when connecting a socket
 *        c) when sending from a connected socket
 *
 *        >>> ARP cache entries are refreshed when data is received on
 *        a socket or when an ICMP ('ping') echo-request is received.
 *        It is possible to disable this feature by setting the
 *        global variable 'lanIpBasicAutoRefreshARP' to zero.
 */

extern int lanIpBscAutoRefreshARP;

/* 'Manual' maintenance of the ARP cache */

/* Perform an ARP lookup for 'ipaddr' (network byte order) first
 * in the cache and then on the network.
 * Note that this routine doesn't synchronize with getting a reply
 * but simply delays the executing task for one OS system 'tick'
 * before re-trying the cache [which should then hit if a reply was
 * received].
 * Hence, this routine may be quite SLOW. This routine is called
 * by udpSockHdrsInit, udpSockConnect, udpSockSend.
 * By manually managing cache entries it is possible to avoid
 * network lookups.
 *
 *    'enaddr': pointer to uint8_t [6] array where MAC address is stored.
 * 'cacheonly': only consult the cache - no network lookup is done.
 *
 * RETURNS: 0 on success,  -errno on error.
 *
 * NOTE: 'enaddr' may be NULL. In this case an ARP request for
 *       'ipaddr' is broadcast (w/o waiting for a reply).
 *       This feature can be used to asynchronously update a
 *       cache entry: E.g, if we want to stream data to a peer
 *       then we can every couple of minutes do a
 *
 *         arpLookup(if, peer, 0, 0)
 *
 *       to make sure every once in a while a new lookup occurs
 *       w/o having to flush the cache. If the peer HW address
 *       changes (HW swap) then the cache entry is updated.
 *       ('cacheonly' is meaningless if 'enaddr' == NULL)
 */

int
arpLookup(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int cacheonly);

/*
 * Create an ARP cache entry for 'ipaddr' (network byte order) / 'enaddr'.
 * If 'perm' is > 0 a permanent (static) entry is created.
 *
 * RETURNS: 0 on success, -errno on error [trying to create too many
 *          permanent entries -- would exhaust the hash table].
 */
int
arpPutEntry(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int perm);

/*
 * Remove entry from the cache
 */
void
arpDelEntry(IpBscIf pd, uint32_t ipaddr);

/*
 * Swipe the arp cache and evict all entries older than 'maxage' seconds
 * except for 'permanent' entires. Then go to sleep for 'period' seconds
 * and repeat 'nloops' times. Setting nloops < 0 lets the scavenger run
 * forever.
 */

void
arpScavenger(IpBscIf pd, rtems_interval maxage, rtems_interval period, int nloops);

/* Flush the entire arp cache (except for permanent/static entires
 * if 'perm_also' is zero).
 */
void
arpFlushCache(IpBscIf pd, int perm_also);

/* Print ARP cache contents to a file (stdout if NULL) */
void
arpDumpCache(IpBscIf pd, FILE *f);

/* The following entry points need to be implemented by the driver */

/* The intended use of these routines is as follows 
 *
 * setup and start stack and driver:
 *
 *    // initialize stack core
 *    lanIpBscInit();
 *    // create driver handle
 *    drv = lanIpBscDrvCreate(unit, &enaddr);
 *    // marry them and set interface address;
 *    // driver is started by lanIpBscIfCreate().
 *    ifc = lanIpBscIfCreate(drv, "192.168.2.2", "255.255.255.0");
 *
 * take down driver and stack:
 *
 *    // destroy interface and shutdown stack;
 *    // driver is shut down from lanIpBscIfDestroy()
 *    lanIpBscIfDestroy(ifc);
 *    lanIpBscShutdown();
 */

/* Create and setup driver instance. Note that this routine must not
 * call any services from lanIpBasic yet.
 * ARGUMENTS:
 * 'instance': identifies device instance if there are more than one.
 *             A value < 0 picks the first instance available.
 *   'enaddr': ethernet address (driver may either ignore this or
 *             use it to override hardware settings).
 * RETURNS: driver handle.
 */
LanIpBscDrv
lanIpBscDrvCreate(int instance, uint8_t *enaddr_p);

/* 
 * Set/read configuration parameters of the
 * lanIpBasic stack.
 *
 * Only parameters that have their 'mask bit'
 * set will be modified.
 *
 * The previous values are returned in *p_oldcfg
 * with 'mask' having all currently supported
 * fields set.
 *
 * RETURNS: Zero on success, nonzero on error (e.g.,
 *          attempt to change parameters when the
 *          stack is running already).
 *
 * NOTES:   Either pointer may be NULL (if you are
 *          not interested in the previous values
 *          or don't want to change anything).
 *
 *          It is not possible to change parameters
 *          once the stack is initialized (lanIpBscInit()).
 *
 */


/* Pass ORed mask of settings you want to change
 * (other fields in LanIpBscConfig struct are ignored)
 */
#define LANIPCFG_RX_RING	(1<<0)
#define LANIPCFG_TX_RING	(1<<1)
#define LANIPCFG_N_RBUFS	(1<<2)
#define LANIPCFG_SQDEPTH	(1<<3)

typedef struct LanIpBscConfigRec_ {
	unsigned mask;
	unsigned rx_ring_size;
	unsigned tx_ring_size;
	unsigned num_rbufs;
	unsigned rx_queue_depth;
} LanIpBscConfigRec, *LanIpBscConfig;

int
lanIpBscConfig(LanIpBscConfig p_cfg, LanIpBscConfig p_oldcfg);

/* Initialize and shut-down the stack
 */
int
lanIpBscInit();

int
lanIpBscShutdown();

/*
 * Diagnostics routines.
 * 
 * NOTES: Most diagnostics information is not thread-safe
 *        with minor impact (e.g., it is possible that
 *        the 'used' + 'free' buffer count doesn't add up
 *        to the 'total' because the three counters are
 *        not read atomically).
 *
 *        However, it may be catastrophic to e.g., tear-down
 *        an interface while dumping information about it.
 *
 *        Most counters are 32-bit and may occasionally roll-over.
 */

/*
 * Dump stack configuration parameters and actual
 * run-time values (RBUFs, sockets).
 *
 * FILE may be NULL in which case 'stdout' is used.
 */

void
lanIpBscDumpConfig(FILE *f);

/* Dump info about all MC groups subscribed on interface to file 'f'.
 *
 * RETURNS: Zero on success, nonzero on error.
 *
 * NOTES:   Not fully thread-safe; must not destroy interface while using this.
 *   
 *          'f' may be passed as NULL in which case 'stdout' is used.
 *    
 *          The identical dump may also be obtained via lanIpBscDumpIfStats()
 *          (see below).
 */
int
lanIpBscDumpMcGroups(IpBscIf ipbif_p, FILE *f);

/*
 * Dump interface statistics to 'FILE'.
 *
 * NOTES:
 *  - 'f' may be NULL in which case 'stdout' will be used.
 *  - An ORed bitset must be passed in 'info'. Only counters
 *    related to the selected flags in 'info' are dumped.
 *    If 'info' == IPBSC_IFSTAT_INFO_ALL then all available
 *    info is printed.
 *  - IPBSC_IFSTAT_INFO_MCGRP calls lanIpBscDumpMcGroups().
 *  - IPBSC_IFSTAT_INFO_DRV depends on driver-specific
 *    implementation.
 */
void
lanIpBscDumpIfStats(IpBscIf intrf, unsigned info, FILE *f);

/*
 * Flags to be passed as 'info' to lanIpBscDumpIfStats()
 * (may be ORed).
 */

/* Ethernet/MAC-level counters          */
#define IPBSC_IFSTAT_INFO_MAC       (1<<0)
/* IP related statistics                */
#define IPBSC_IFSTAT_INFO_IP        (1<<1)
/* UDP related statistics               */
#define IPBSC_IFSTAT_INFO_UDP       (1<<2)
/* ARP related statistics               */
#define IPBSC_IFSTAT_INFO_ARP       (1<<3)
/* ICMP related statistics              */
#define IPBSC_IFSTAT_INFO_ICMP      (1<<4)
/* IGMP related statistics              */
#define IPBSC_IFSTAT_INFO_IGMP      (1<<5)
/*
 * Dump IP multicast memberships etc.;
 * identical to lanIpBscDumpMcGroups()
 */
#define IPBSC_IFSTAT_INFO_MCGRP     (1<<6)
/*
 * Call driver's statistics routine
 * (if available).
 */
#define IPBSC_IFSTAT_INFO_DRV       (1<<7)

/*
 * Dump all available info.
 */
#define IPBSC_IFSTAT_INFO_ALL       ((unsigned)(-1))

/*
 * 'Machine-readable' summary statistics:
 */

/* Interface Info */
typedef struct LanIpBscIfSumStatsRec_ {
	/* Linked list (in case there is ever support for multiple IFs) */
	struct      LanIpBscIfSumStatsRec_ *p_next;

	const char *drv_name;  /* name of driver attached to this IF                  */

	uint8_t  en_addr[6];   /* MAC address                                         */
	uint32_t ip_addr;      /* IP address (NETWORK byte order)                     */
	uint32_t ip_nmask;     /* IP netmask (NETWORK byte order)                     */

	uint32_t mc_ngroups;   /* # of MC groups IF is a member of                    */

	uint32_t eth_rx_frms;  /* Ethernet frames accepted and passed up              */
	uint32_t eth_rx_drop;  /* Ethernet frames not handled by upper layers         */

	uint32_t arp_rx_reps;  /* ARP replies received                                */
	uint32_t arp_rx_reqs;  /* ARP requests received (for this IP or others)       */
	uint32_t arp_rx_drop;  /* Frames dropped by ARP layer                         */

	uint32_t arp_tx_reqs;  /* ARP requests sent                                   */
	uint32_t arp_tx_reps;  /* ARP replies sent                                    */

	uint32_t ip_rx_ufrm;   /* IP unicast frames accepted (and passed up)          */
	uint32_t ip_rx_mfrm;   /* IP multicast frames accepted (and passed up)        */
	uint32_t ip_rx_bfrm;   /* IP broadcast frames accepted (and passed up)        */
	uint32_t ip_rx_drop;   /* Frames dropped by IP layer                          */

	uint32_t icmp_rx_ereq; /* ICMP Echo Requests Received ('ping')                */
	uint32_t icmp_rx_drop; /* Frames dropped by ICMP handler                      */

	uint32_t igmp_rx_reps; /* IGMP Reports received                               */
	uint32_t igmp_rx_qrys; /* IGMP Queries received                               */
	uint32_t igmp_rx_drop; /* Frames dropped by IGMP handler                      */
	uint32_t igmp_tx_reps; /* IGMP Reports sent                                   */
	uint32_t igmp_tx_leav; /* IGMP Leave msgs. sent                               */

	uint32_t udp_rx_frms;  /* UDP frames receifed                                 */
	uint32_t udp_rx_drop;  /* Frames dropped by UDP layer                         */
	uint32_t udp_tx_frms;  /* UDP frames sent (from sockets)                      */
} LanIpBscIfSumStatsRec, *LanIpBscIfSumStats;

typedef struct LanIpBscSumStatsRec_ {
	uint32_t           nsocks_max;
	uint32_t           nsocks_used;
	uint32_t           sock_qdepth;
	uint32_t           rbufs_max;
	uint32_t           rbufs_used;

	uint32_t           if_max;
	LanIpBscIfSumStats if_stats; /* linked list of IF stats */
} LanIpBscSumStatsRec, *LanIpBscSumStats;

/*
 * Obtain summary statistics; when done, the returned object
 * must be released by lanIpBscFreeStats().
 */
LanIpBscSumStats
lanIpBscGetStats();

/*
 * Release all resources held by 'stats'
 */
void
lanIpBscFreeStats(LanIpBscSumStats stats);

#ifdef __cplusplus
}
#endif

#endif
