/* $Id$ */

#ifndef LAN_IP_BASIC_H
#define LAN_IP_BASIC_H

/* Basic IP/UDP socket */

#include <lanIpProto.h>
#include <drvLan9118.h>

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
 * safely share a socket.
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

/* Send a buffer; EVERYTHING (all headers + payload must have been filled in)
 * len: total length (including all headers and initial 2-byte padding).
 * The buffer is taken over by the stack and released eventually.
 *
 * RETURNS: len
 */
int
udpSockSendBufRaw(LanIpPacket buf_p, int len);

/* Like SendBufRaw but assumes the packet is a UDP packet so that
 * the length can be extracted from the IP header.
 */
int
udpSockSendBufRawIp(LanIpPacket buf_p);

/*
 * RX callback for drvLan9118
 */
int
drvLan9118IpRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *arg);

/* Operations on packet headers: */

/* Setup Ethernet, IP and UDP headers in a packet.
 * An ARP lookup for 'dipaddr' is done if necessary.
 *
 * NOTES: 'dipaddr' (destination IP address) is in *network* byte order.
 *        'dport' and 'ip_id' are in *host* byte order.
 *
 *        It is legal to provide an all zero destination IP to avoid
 *        ARP lookup. You must fill in the IP and checksum later.
 *
 *        The socket descriptor is only needed to determine the source
 *        UDP port. It is legal to submit a sd < 0 and fill in the source
 *        port 'manually'.
 *
 * RETURNS: 0 on success, -errno on error (-ENOTCONN == ARP lookup failure)
 */
int
udpSockHdrsInit(int sd, LanUdpHeader p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id);

/* Set length and IP checksum
 * Note: other fields must have been initialized already
 */
void
udpSockHdrsSetlen(LanUdpHeader p, int payload_len);

/* Flip source -> dest and fill-in local source addresses
 * (at ethernet, IP and UDP level)
 */
void
udpSockHdrsReflect(LanUdpHeader p);

/* Create private data (pass as rx callback closure pointer to drvLan9118Start)
 * 
 * This can be thought of as (and should better be called) an 'interface handle'.
 * RETURNS handle on success, NULL on failure.
 */

typedef struct IpBscIfRec_ *IpBscIf;

/* Create private data */
IpBscIf
lanIpBscIfCreate(void);

/* Setup private data structure */
void
lanIpBscIfInit(IpBscIf ipbif_p, void *drv_p, char *ipaddr, char *netmask);

/* Retrieve the driver handle   */
void *
lanIpBscIfGetDrv(IpBscIf ipbif_p);

void
lanIpBscIfDestroy(IpBscIf);

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

#ifdef __cplusplus
}
#endif

#endif
