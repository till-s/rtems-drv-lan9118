/* $Id$ */

#ifndef BSDSOCKET
#define BSDSOCKET
#endif

/* Glue layer to send padProto over ordinary UDP sockets */

#include <udpComm.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* to get alignment only */
#include <lanIpProto.h>


#define DO_ALIGN(x,a) (((uintptr_t)(x) + ((a)-1)) & ~((a)-1))
#define BUFALIGN(x)   DO_ALIGN(x,LAN_IP_BASIC_PACKET_ALIGNMENT)

/* maintain same alignment of data-area 
 * which is seen when using lanIpBasic.
 * Pad # of bytes consumed by ethernet, IP
 * and UDP headers.
 */
#define PADSZ         (16+20+8)

typedef struct {
	char   data[1500];
	struct sockaddr sender;
	int             rx_sd;
	void            *raw_mem;
} __attribute__((may_alias)) UdpCommBSDPkt;

int
udpCommSocket(int port)
{
int                sd = socket(AF_INET, SOCK_DGRAM, 0);
struct sockaddr_in me;

	if ( sd < 0 )
		return sd;

	memset(&me, 0, sizeof(me));
	me.sin_family = AF_INET;
	me.sin_port   = htons(port);

	if ( bind(sd, (void*)&me, sizeof(me)) ) {
		close(sd);
		return -errno;
	}

	return sd;
}

UdpCommPkt
udpCommRecvFrom(int sd, int timeout_ms, uint32_t *ppeerip, uint16_t *ppeerport)
{
struct timeval     tv;
fd_set             fds;
UdpCommBSDPkt      *p;
void               *p_raw;
struct sockaddr_in __attribute__((may_alias)) *sa;
socklen_t          len;

	tv.tv_sec  = timeout_ms/1000;
	tv.tv_usec = 1000*(timeout_ms % 1000);
	FD_ZERO(&fds);
	FD_SET(sd, &fds);

	if ( select(sd+1, &fds, 0, 0, &tv) <= 0 )
		return 0;
	p_raw      = malloc(sizeof(*p) + PADSZ + LAN_IP_BASIC_PACKET_ALIGNMENT-1);
	p          = (UdpCommBSDPkt*)(BUFALIGN(p_raw) + PADSZ);
	p->raw_mem = p_raw;
	sa         = (struct sockaddr_in __attribute__((may_alias)) *)&p->sender;
	len        = sizeof( p->sender );
	if ( recvfrom(sd, p->data, sizeof(p->data), 0, &p->sender, &len) < 0 ) {
		free(p_raw);
		return 0;
	} else {
		if ( ppeerip )
			*ppeerip = sa->sin_addr.s_addr;
		if ( ppeerport )
			*ppeerport = ntohs(sa->sin_addr.s_addr);
	}
	p->rx_sd = sd;
	return p;
}

void
udpCommFreePacket(UdpCommPkt p)
{
	if ( p )
		free(((UdpCommBSDPkt*)p)->raw_mem);
}

UdpCommPkt
udpCommRecv(int sd, int timeout_ms)
{
	return udpCommRecvFrom(sd, timeout_ms, 0, 0);
}

void
udpCommReturnPacket(UdpCommPkt p0, int len)
{
UdpCommBSDPkt *p = (UdpCommBSDPkt *)p0;
	sendto( p->rx_sd, p->data, len, 0, &p->sender, sizeof(p->sender));
}

int
udpCommConnect(int sd, uint32_t diaddr, int port)
{
struct sockaddr_in they;
	
	memset(&they, 0, sizeof(they));
	they.sin_family = AF_INET;
	they.sin_addr.s_addr   = (uint32_t)diaddr;
	they.sin_port   = htons(port);
	return connect(sd, (struct sockaddr*)&they, sizeof(they));
}
