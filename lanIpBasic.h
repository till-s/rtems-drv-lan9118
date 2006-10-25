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

void
udpSockFreeBuf(LanIpPacketRec *ppacket);

/*
 * RX callback for drvLan9118
 */
int
drvLan9118IpRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *arg);

/* Create private data (pass as rx callback closure pointer to drvLan9118Start)
 *
 * RETURNS handle on success, NULL on failure.
 */

typedef struct IpCbDataRec_ *IpCbData;

IpCbData
lanIpCbDataCreate(DrvLan9118_tps plan_ps, char *ipaddr, char *netmask);

void
lanIpCbDataDestroy(IpCbData);

#endif

