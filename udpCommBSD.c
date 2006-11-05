/* $Id$ */

#ifndef BSDSOCKET
#define BSDSOCKET
#endif

/* Glue layer to send padProto over ordinary UDP sockets */

#include <udpComm.h>
#include <errno.h>
#include <stdlib.h>

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
udpCommRecv(int sd, int timeout_ms)
{
struct timeval tv;
fd_set         fds;
UdpCommPkt     p;

	tv.tv_sec  = timeout_ms/1000;
	tv.tv_usec = 1000*(timeout_ms % 1000);
	FD_ZERO(&fds);
	FD_SET(sd, &fds);

	if ( select(sd+1, &fds, 0, 0, &tv) <= 0 )
		return 0;
	p = malloc(1500);
	if ( recv(sd, p, 1500, 0) < 0 ) {
		free(p);
		return 0;
	}
	return p;
}

int
udpCommConnect(int sd, uint32_t diaddr, int port)
{
struct sockaddr_in they;
	
	memset(&they, 0, sizeof(they));
	they.sin_family = AF_INET;
	they.sin_addr.s_addr   = (in_addr_t)diaddr;
	they.sin_port   = htons(port);
	return connect(sd, (struct sockaddr*)&they, sizeof(they));
}
