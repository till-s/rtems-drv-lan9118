/* $Id$ */

#ifndef LAN_IP_BASIC_H
#define LAN_IP_BASIC_H

/* Basic IP/UDP socket */

#include <lanIpProto.h>
#include <drvLan9118.h>

/* Create a socket listening on a UDP port
 * RETURNS: descriptor (>=0) on success, < 0 on error
 */
int
udpSockCreate(uint16_t port);

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

/* Send a buffer; EVERYTHING (all headers + payload must have been filled in)
 * len: total length (including all headers and initial 2-byte padding).
 * The buffer is taken over by the stack and released eventually.
 *
 * RETURNS: len
 */
int
udpSockSendBufRaw(LanIpPacket buf_p, int len);

/*
 * RX callback for drvLan9118
 */
int
drvLan9118IpRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *arg);

/* Operations on packet headers: */

/* Setup Ethernet, IP and UDP headers in a packet.
 * An ARP lookup for 'dipaddr' is done if necessary.
 */
void
udpSockInitHdrs(int sd, LanIpPacket p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id);

/* Set length and IP checksum
 * Note: other fields must have been initialized already
 */
void
lanIpUdpHdrSetlen(LanIpPacket p, int payload_len);

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
