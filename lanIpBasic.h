/* $Id$ */

#ifndef LAN_IP_BASIC_H
#define LAN_IP_BASIC_H

/* Basic IP/UDP socket */

#include <lanIpProto.h>
#include <drvLan9118.h>

/* Create a socket listening on a UDP port
 * RETURNS: descriptor (>=0) on success, < 0 on error
 *
 * NOTE: port is in *host* byte order.
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
 * NOTE:    'dipaddr' == 0 and 'dport' == 0 may be passed
 *          to 'disconnect' the socket.
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
 * NOTE: 'dipaddr' (destination IP address) is in *network* byte order.
 *       'dport' and 'ip_id' are in *host* byte order.
 *
 * RETURNS: 0 on success, -errno on error (-ENOTCONN == ARP lookup failure)
 */
int
udpSockHdrsInit(int sd, LanIpPacket p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id);

/* Set length and IP checksum
 * Note: other fields must have been initialized already
 */
void
udpSockHdrsSetlen(LanIpPacket p, int payload_len);

/* Flip source -> dest and fill-in local source addresses
 * (at ethernet, IP and UDP level)
 */
void
udpSockHdrsReflect(LanIpPacket p);

/* Create private data (pass as rx callback closure pointer to drvLan9118Start)
 *
 * RETURNS handle on success, NULL on failure.
 */

typedef struct IpCbDataRec_ *IpCbData;

/* Create private data */
IpCbData
lanIpCbDataCreate(void);

/* Setup private data structure */
void
lanIpCbDataInit(IpCbData cbd_p, void *drv_p, char *ipaddr, char *netmask);

/* Retrieve the driver handle   */
void *
lanIpCbDataGetDrv(IpCbData cbd_p);

void
lanIpCbDataDestroy(IpCbData);

#endif
