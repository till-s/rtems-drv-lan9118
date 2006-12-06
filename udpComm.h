/* $Id$ */
#ifndef UDPCOMM_LAYER_H
#define UDPCOMM_LAYER_H

/* Glue for simple UDP communication over
 * either BSD sockets or simple 'udpSock'ets and
 * the lanIpBasic 'stack'.
 */

#ifdef BSDSOCKET
#include <sys/socket.h>
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

/* Connect socket to a peer */
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

/* Obtain pointer to data area in buffer (UDP payload) */
static __inline__ void *
udpCommBufPtr(UdpCommPkt p);

/* Release packet (obtained from Recv) when done */
static __inline__ void
udpCommFreePacket(UdpCommPkt p);

/* Send packet to connected peer */
static __inline__ int
udpCommSend(int sd, void *buf, int len);

/* Return packet to sender (similar to 'send'; 
 * this interface exists for efficiency reasons
 * [coldfire/lan9118]).
 */
static __inline__ void
udpCommReturnPacket(int sd, UdpCommPkt p, int len);

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

static __inline__ void
udpCommFreePacket(UdpCommPkt p)
{
#ifdef BSDSOCKET
	free(p);
#else
	udpSockFreeBuf(p);
#endif
}

static __inline__ int
udpCommSend(int sd, void *buf, int len)
{
#ifdef BSDSOCKET
	return send(sd, buf, len, 0);
#else
	return udpSockSend(sd, buf, len);
#endif
}

static __inline__ void
udpCommReturnPacket(int sd, UdpCommPkt p, int len)
{
#ifdef BSDSOCKET
	/* Subtle differences exist between sending to the peer
     * and just swapping the headers around (as done for
     * udpSocks) -- guaranteed no ARP is necessary in the
     * latter case. For BSDSOCKET we don't care about ARP.
     */
	send(sd, p, len, 0);
#else
	udpSockHdrsReflect(p);	/* point headers back to sender */
	udpSockSendBufRawIp(p); /* send off                     */
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

#endif

#ifdef __cplusplus
}
#endif

#endif
