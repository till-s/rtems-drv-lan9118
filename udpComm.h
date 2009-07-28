/* $Id$ */
#ifndef UDPCOMM_LAYER_H
#define UDPCOMM_LAYER_H

/* Glue for simple UDP communication over
 * either BSD sockets or simple 'udpSock'ets and
 * the lanIpBasic 'stack'.
 */

#ifdef BSDSOCKET
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdlib.h>
#define STATICINLINE
#else
#include <lanIpBasic.h>
#define STATICINLINE static __inline__
#endif
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void * UdpCommPkt;

/* Create */
STATICINLINE int
udpCommSocket(int port);

/* Close  */
static __inline__ int
udpCommClose(int sd);

/* Connect socket to a peer
 *
 * NOTE: 'dipaddr' is the peer's IP address in *network* byte order
 *       'port'    is the peer's port number in *host*   byte order
 */
STATICINLINE int
udpCommConnect(int sd, uint32_t diaddr, int port);


/* Receive a packet */
STATICINLINE UdpCommPkt
udpCommRecv(int sd, int timeout_ms);

/* Receive a packet and sender information
 *
 * NOTE: port is in host, IP address in network byte order.
 */
STATICINLINE UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport);

/* Allocate a packet (for sending with udpCommSendPktTo) */
STATICINLINE UdpCommPkt
udpCommAllocPacket();

/* Release packet (obtained from Recv) when done */
STATICINLINE void
udpCommFreePacket(UdpCommPkt p);

#ifdef BSDSOCKET
/* Payload size = 3*512 (<MTU) - eth, ip and udp header sizes) */
#define UDPCOMM_PKTSZ (3*512 - 16 - 20 - 8)
#else
#define UDPCOMM_PKTSZ UDPPAYLOADSIZE
#endif

/* Obtain pointer to data area in buffer (UDP payload) */
static __inline__ void *
udpCommBufPtr(UdpCommPkt p);

/* Send packet to connected peer; 
 * The data in 'buf' has to be copied
 * into the 'lanIpBasic' stack (no-op
 * when using BSD sockets).
 */
static __inline__ int
udpCommSend(int sd, void *buf, int len);

/* Send packet w/o extra copy step.
 * Packet must be pre-allocated using
 * udpCommAllocPacket() and filled with
 * data (into the user area).
 *
 * NOTE: Ownership of the packet is
 *       transferred to the stack by
 *       this call (regardless of the
 *       return value).
 */
STATICINLINE int
udpCommSendPkt(int sd, UdpCommPkt pkt, int len);

/*
 * As above but send to specified peer
 * (unconnected socket only).
 *
 * NOTE: 'dipaddr' is the peer's IP address in *network* byte order
 *       'port'    is the peer's port number in *host*   byte order
 */
STATICINLINE int
udpCommSendPktTo(int sd, UdpCommPkt pkt, int len, uint32_t dipaddr, int port);

/* Return packet to sender (similar to 'send'; 
 * this interface exists for efficiency reasons
 * [coldfire/lan9118]).
 */
STATICINLINE void
udpCommReturnPacket(UdpCommPkt p, int len);

/* Join and leave a MC group. This actually affects the interface
 * not just the socket 'sd'.
 * NOTE: calls do not nest; you cannot call this twice on the
 *       same socket.
 *
 * RETURNS: zero on success, -errno on failure.
 */

STATICINLINE int
udpCommJoinMcast(int sd, uint32_t mcaddr);

STATICINLINE int
udpCommLeaveMcast(int sd, uint32_t ifipaddr);

/*
 * Set the outgoing interface for sending multicast
 * traffic from 'sd'. The interface with IP address
 * 'ifaddr' is used for sending all MC traffic from
 * socket 'sd'. 
 *
 * Note that the same or similar functionality 
 * can be achieved by setting up the system routing
 * tables which is more transparent, flexible and
 * IMHO preferable.
 * However, in some cases -- especially for testing --
 * setting this on a 'per-socket' basis with this
 * call is useful; particularly because (on linux
 * and other general-purpose, protected OSes) no
 * special privileges are required.
 *
 * RETURNS: zero on success, nonzero (-errno) on
 *          error.
 *
 * NOTES:   use a 'ifaddr' == 0 to remove the
 *          association of a outgoing mcast IF
 *          with a socket.
 *
 *          The 'ifipaddr' is as usual given in
 *          network-byte order.
 */
STATICINLINE int
udpCommSetIfMcast(int sd, uint32_t ifipaddr);

/* This variable can be set to IP address of
 * receiving interface (if host has multiple NICs)
 * in network byte order. Defaults to INADDR_ANY,
 * i.e., system picks a suitable IF.
 */
extern uint32_t udpCommMcastIfAddr;


/* Inline implementations for both BSD and udpSocks */

static __inline__ int
udpCommClose(int sd)
{
#ifdef BSDSOCKET
	return close(sd);
#else
	return udpSockDestroy(sd);
#endif
}

static __inline__ void *
udpCommBufPtr(UdpCommPkt p)
{
#ifdef BSDSOCKET
	return (void*)p;
#else
	return (void*)lpkt_udphdr((LanIpPacket)p).pld;
#endif
}

#ifndef BSDSOCKET
static __inline__ UdpCommPkt
udpCommAllocPacket()
{
	return udpSockGetBuf();
}
#endif


#ifndef BSDSOCKET
static __inline__ void
udpCommFreePacket(UdpCommPkt p)
{
	udpSockFreeBuf(p);
}
#endif

static __inline__ int
udpCommSend(int sd, void *buf, int len)
{
#ifdef BSDSOCKET
	return send(sd, buf, len, 0);
#else
	return udpSockSend(sd, buf, len);
#endif
}

#ifndef BSDSOCKET

static __inline__ int ms2ticks(int ms)
{
	if ( ms > 0 ) {
		rtems_interval rate;
		rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &rate);
		ms *= rate;
		if ( 0 == (ms /= 1000) )
			ms = 1;
	}
	return ms;
}
/* Inline implementation for udpSocks */
static __inline__ int
udpCommSocket(int port)
{
	return udpSockCreate(port);
}

static __inline__ UdpCommPkt
udpCommRecv(int sd, int timeout_ms)
{
	return udpSockRecv(sd, ms2ticks(timeout_ms));
}

STATICINLINE UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport)
{
UdpCommPkt rval;
	rval = udpSockRecv(sd, ms2ticks(timeout_ms));
	if ( rval ) {
		if ( ppeerip )
			*ppeerip = lpkt_ip((LanIpPacket)rval).src;
		if ( ppeerport )
			*ppeerport = ntohs(lpkt_udp((LanIpPacket)rval).sport);
	}
	return rval;
}

static __inline__ int
udpCommConnect(int sd, uint32_t diaddr, int port)
{
	return udpSockConnect(sd, diaddr, port);
}

static __inline__ void
udpCommReturnPacket(UdpCommPkt p, int len)
{
	udpSockHdrsReflect(p);	/* point headers back to sender */
	udpSockSendBufRawIp(p); /* send off                     */
}


#endif

#ifdef __cplusplus
}
#endif

#endif
