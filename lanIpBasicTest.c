/* $Id$ */

/* Wrapper code to initialize the simple IP/UDP stack */

#include <rtems.h>
#include <rtems/error.h>
#include <lanIpBasic.h>
#include <lanIpBasicSetup.h>
#include <stdio.h>

/* Note: the name of this file is historic and unfortunate... */

#include <string.h>

#include <netinet/in.h>

#define PAYLDLEN 1024

#include "hwtmr.h"

void      *lanIpDrv  =  0;
IpBscIf   lanIpIf    =  0;
int		  lanIpUdpsd = -1;

int
lanIpTakedown()
{
	if ( lanIpUdpsd >= 0 ) {
		udpSockDestroy(lanIpUdpsd);
		lanIpUdpsd = -1;
	}

	if ( lanIpIf ) {
		if ( lanIpBscIfDestroy( lanIpIf ) ) {
			fprintf(stderr,"Cannot shutdown\n");
			return -1;
		}
		lanIpIf  = 0;
		lanIpDrv = 0;
	}


	if ( lanIpBscShutdown() ) {
		fprintf(stderr,"Cannot shutdown; some resources still in use\n");		
		return -1;
	}

	return 0;
}

int
lanIpSetup(char *ip, char *nmsk, int port, uint8_t *enaddr)
{
	if ( !ip || !nmsk ) {
		fprintf(stderr,"Usage: lanIpSetup(char *ip, char *netmask, int port, enaddr)\n");
		return -1;
	}

	if ( lanIpDrv ) {
		fprintf(stderr,"Can call setup only once\n");
		return -1;
	}

	if ( lanIpBscInit() ) {
		fprintf(stderr,"lanIpBscInit() failed\n");
		return -1;
	}

	lanIpDrv = lanIpBscDrvCreate(-1, enaddr);

	if ( !lanIpDrv )
		goto egress;

	if ( ! (lanIpIf = lanIpBscIfCreate(lanIpDrv, ip, nmsk)) ) {
		fprintf(stderr,"Unable to create interface data struct\n");
		goto egress;
	}

	if ( port > 0 && (lanIpUdpsd = udpSockCreate(port)) < 0 ) {
		fprintf(stderr,"Unable to create UDPSOCK: %s\n", strerror(-lanIpUdpsd));
		goto egress;
	}

	return 0;

egress:
	lanIpTakedown();
	return -1;
}

uint32_t lanIpTst_mintrip         = -1;
uint32_t lanIpTst_maxtrip         =  0;
uint32_t lanIpTst_maxsend         =  0;
uint32_t lanIpTst_avgtrip128      =  0;
uint32_t lanIpTst_pktlost         =  0;
uint32_t lanIpTst_pktsent         =  0;
uint32_t lanIpTst_pktrcvd         =  0;
volatile int lanIpTst_keeprunning =  1;

/* bounce a UDP packet back to the caller */

typedef uint32_t echodata[2];

int
udpSocketEcho(int sd, int raw, int idx, int timeout)
{
LanIpPacket p;
uint16_t    tmp;
int         len = -1, rval;
uint32_t	now, then, post;

static LanIpPacketRec dummy = {{{{{0}}}}};

	if ( idx < 0 || idx >= sizeof(echodata)/sizeof((*(echodata*)0)[0]) ) {
		return -1;
	}

	if ( !lpkt_ip(&dummy).src ) {
		if ( (rval = udpSockHdrsInit(sd, &lpkt_udp_hdrs(&dummy), 0, 0, 0)) ) {
			fprintf(stderr,"udpSocketEcho - Unable to initialize headers: %s\n", strerror(-rval));
			return rval;
		}
	}

	p = udpSockRecv(sd, timeout);

	if ( p ) {

		if ( raw ) {
			if ( 0 ) {
			/* user manages headers */
			memcpy(lpkt_eth(p).dst, lpkt_eth(p).src, sizeof(lpkt_eth(p).dst));
			lpkt_ip(p).dst    = lpkt_ip(p).src;	
			{
				/* fill source ll address, IP address and UDP port */
				memcpy(lpkt_eth(p).src, lpkt_eth(&dummy).src, sizeof(lpkt_eth(&dummy).src));
				lpkt_ip(p).src    = lpkt_ip(&dummy).src;

				tmp               = lpkt_udp(p).dport;
				lpkt_udp(p).dport = lpkt_udp(p).sport;
				lpkt_udp(p).sport = tmp;

				lpkt_ip(p).csum   = 0;
				/*
				lpkt_ip(p).csum   = htons(in_cksum_hdr((void*)&lpkt_ip(p)));
				 */

				lpkt_udp(p).csum = 0;

			}
			} else {
				udpSockHdrsReflect(&lpkt_udp_hdrs(p));
			}
			now  = Read_hwtimer();
			then = lpkt_udp_pld(p,echodata)[idx];
			lpkt_udp_pld(p,echodata)[idx] = now;
			len = lanIpBscSendBufRawIp(lanIpIf, p);
			post = Read_hwtimer();
		} else {
			now  = Read_hwtimer();
			then = lpkt_udp_pld(p,echodata)[idx];
			lpkt_udp_pld(p,echodata)[idx] = now;
			len = ntohs(lpkt_udp(p).len) - sizeof(UdpHeaderRec);
			udpSockSend(sd, &lpkt_udp_pld(p,echodata), len);
			post = Read_hwtimer();
			udpSockFreeBuf(p);
		}

		post-=now;
		if ( post > lanIpTst_maxsend )
			lanIpTst_maxsend = post;

		lanIpTst_pktsent++;
		lanIpTst_pktrcvd++;

		/* If this was not the first packet then measure roundtrip */
		if ( then != 0 ) {
			now-=then;	
			if ( now < lanIpTst_mintrip )
				lanIpTst_mintrip = now;
			if ( now > lanIpTst_maxtrip )
				lanIpTst_maxtrip = now;
			/* avgtrip = (127*avgtrip + now)/128;
             * 128 * a = 127 * a + n = 127/128 * (128*a) + n
             */
            lanIpTst_avgtrip128 += (now - (lanIpTst_avgtrip128>>7));
			/* When reading avgtrip it must be divided by 128 */
		}
	}
	return len;
}

int
udpBouncer(int master, int raw, uint32_t dipaddr, uint16_t dport)
{
LanIpPacket p;
int         st;
int         err = -1;
int         cnt = 4;

	if ( !raw ) {
		if ( (err=udpSockConnect(lanIpUdpsd, dipaddr, dport, UDPSOCK_MCPASS)) ) {
			fprintf(stderr,"bouncer: Unable to connect socket: %s\n", strerror(-err));
			return err;
		}
	}

	if ( master ) {
		do {
			/* create a packet */
			if ( !(p=udpSockGetBuf()) ) {
				fprintf(stderr,"bouncer: Unable to allocate buffer\n");
				goto egress;		
			}
			if ( raw ) {
				/* fillin headers */
				udpSockHdrsInit(lanIpUdpsd, &lpkt_udp_hdrs(p), dipaddr, dport, 0);
				udpSockHdrsSetlen(&lpkt_udp_hdrs(p), PAYLDLEN);

				/* initialize timestamps */
				lpkt_udp_pld(p,echodata)[0] = 0;
				lpkt_udp_pld(p,echodata)[1] = Read_hwtimer();
				lanIpBscSendBufRawIp(lanIpIf, p);
			} else {
				/* initialize timestamps */
				lpkt_udp_pld(p,echodata)[0] = 0;
				lpkt_udp_pld(p,echodata)[1] = Read_hwtimer();
				udpSockSend(lanIpUdpsd, p, PAYLDLEN);
				udpSockFreeBuf(p);
			}

			lanIpTst_pktsent++;

			while ( (st = udpSocketEcho(lanIpUdpsd, raw, 1, 50)) > 0 ) {
				if ( !lanIpTst_keeprunning ) {
					lanIpTst_pktlost--;
					break;
				}
			}

			lanIpTst_pktlost++;

		} while ( lanIpTst_keeprunning && cnt-- > 0 );
			fprintf(stderr,"Master terminated.\n");
			/* in case they want to start again */
			lanIpTst_keeprunning=1;
	} else {
			/* make sure socket is flushed */
			while ( (p = udpSockRecv(lanIpUdpsd,0)) )		
				udpSockFreeBuf(p);
			while ( (st = udpSocketEcho(lanIpUdpsd, raw, 0, 100)) > 0)
					;
			fprintf(stderr,"Slave timed out; terminating (status %i)\n", st);
	}
	err = 0;

egress:
	if ( !raw ) {
		if ( (err=udpSockConnect(lanIpUdpsd, 0, 0, 0)) ) {
			fprintf(stderr,"bouncer: Unable to disconnect socket: %s\n", strerror(-err));
		}
	}
	return(err);
}


int
_cexpModuleFinalize(void* unused)
{
	if ( lanIpDrv || lanIpUdpsd>=0 || lanIpIf ) {
		fprintf(stderr,"Module still in use, use 'lanIpTakedown()'\n");
		return -1;
	}
	return 0;
}
