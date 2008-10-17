/* $Id$ */

/* Rudimentary IP protocol:
 *
 *  - trivial ARP
 *  - trivial IP (only 20byte header)
 *  - ICMP echo request (ping) handling
 *
 *  - LAN only (no IP routing)
 *
 */

/* T. Straumann <strauman@slac.stanford.edu> */
#define __RTEMS_VIOLATE_KERNEL_VISIBILITY__
#include <rtems.h>
#include <rtems/error.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <machine/in_cksum.h>
#include <sys/socket.h>

#include <lanIpProto.h>
#include <lanIpBasic.h>

/* Alias helper types */
typedef uint32_t	uint32_a_t __attribute__((__may_alias__));
typedef  int32_t	 int32_a_t __attribute__((__may_alias__));
typedef uint16_t	uint16_a_t __attribute__((__may_alias__));
typedef   int8_t	  int8_a_t __attribute__((__may_alias__));
typedef  uint8_t	 uint8_a_t __attribute__((__may_alias__));
typedef  int16_t	 int16_a_t __attribute__((__may_alias__));

/* include netdriver AFTER defining VIOLATE_KERNEL_VISIBILITY (in case it uses rtems.h already) */
#include NETDRV_INCLUDE

#define DEBUG_ARP	1
#define DEBUG_IP	2
#define DEBUG_ICMP	4
#define DEBUG_UDP	8
#define DEBUG_TASK	16

#define DEBUG		0

#ifdef DEBUG
int	lanIpDebug = DEBUG;
#endif

#ifndef CACHE_OVERLAP
#define CACHE_OVERLAP 10
#endif

/* How many times to retry a network lookup */
#ifndef ARP_SEND_RETRY
#define ARP_SEND_RETRY	3
#endif

/* Trivial RX buffers */
#if defined(RBUF_ALIGNMENT) && ((RBUF_ALIGNMENT-1) & LANPKTMAX)
#error "LANPKTMAX has insufficient alignment for this chip driver"
#endif

#ifndef NRBUFS
#define NRBUFS		50	/* Total number of RX buffers */
#endif
#ifndef NSOCKS
#define NSOCKS		5
#endif
#ifndef QDEPTH
#define QDEPTH		20	/* RX socket queue depth      */
#endif

#ifndef DEFLT_PORT
#define DEFLT_PORT  31110
#endif

typedef LanIpPacketRec rbuf_t;

static rbuf_t		rbufs[NRBUFS]
#ifdef RBUF_ALIGNMENT
__attribute__ ((aligned(RBUF_ALIGNMENT)))
#endif
                    = {{{{{{0}}}}}};

/* lazy init of tbuf facility */
static int    ravail = NRBUFS;
volatile int  lanIpBufAvail = NRBUFS;

static rbuf_t *frb = 0; /* free list */

static uint16_t
ipcsum(uint8_t *d, int n)
{
register uint32_t s = 0;
register uint16_a_t *p;
int				  swapped;
union {
	uint8_t	b[2];
	uint16_t s;
} s_u;

	/* unaligned start */
	if ( (swapped = ((uint32_t)d & 1)) ) {
		s_u.b[0] = 0;
		s_u.b[1] = *d++;
		s       += s_u.s;
		n--;
	}

	p = (uint16_a_t*)d;

	while ( n >= 16 ) {
		s += *p++; s += *p++; s += *p++; s += *p++;
		s += *p++; s += *p++; s += *p++; s += *p++;
		n -= 16;
	}
	while ( n >= 8 ) {
		s += *p++; s += *p++; s += *p++; s += *p++;
		n -= 8;
	}
	while ( n>1 ) {
		s += *p++;
		n-=2;
	}
	if ( n ) {
		s_u.b[0] = *(uint8_t*)p;
		s_u.b[1] = 0;
		s       += s_u.s;
	}

	s  = (s & 0xffff) + (s >> 16);
	s += s>>16;

	if ( swapped ) {
		s = (s<<8) | (s>>8);
	}

	return ~s & 0xffff;
}

static rbuf_t *getrbuf()
{
rbuf_t                *rval;
rtems_interrupt_level key;

	rtems_interrupt_disable(key);
	if ( (rval = frb) ) {
		frb = *(rbuf_t **)rval;
	} else {
		/* get from pool */
		if ( ravail ) {
			rval = &rbufs[--ravail];
		}
	}
	if ( lanIpBufAvail )
		lanIpBufAvail--;
	rtems_interrupt_enable(key);
	return rval;
}

static void relrbuf(rbuf_t *b)
{
rtems_interrupt_level key;

	if ( b ) {
		rtems_interrupt_disable(key);
			*(rbuf_t**)b = frb;
			frb = b;
			lanIpBufAvail++;
		rtems_interrupt_enable(key);
	}
}

#define ISBCST(ip,nm)  (((ip) & ~(nm)) == ~(nm))

typedef struct IpBscIfRec_ {
	void			*drv_p;
	rtems_id		mutx;
	uint32_t		ipaddr;
	uint32_t		nmask;
	LanArpRec		arpreq;
	LanArpRec		arprep;
} IpBscIfRec;

#define FLG_ISCONN	1

typedef struct UdpSockRec_ {
	IpBscIf			  intrf;
	volatile int	  port;	/* host byte order */
	rtems_id		  msgq;
	rtems_id		  mutx;
	unsigned          flags;
	LanUdpHeaderRec   hdr;    /* a header for 'sendto' */
	volatile unsigned nbytes; /* count bytes queued for supporting FIONREAD */
} UdpSockRec, *UdpSock;

typedef struct UdpSockMsgRec_ {
	LanIpPacketRec        *pkt;
	int                   len;
} UdpSockMsgRec, *UdpSockMsg;

#define SOCKLOCK(sck)		assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain((sck)->mutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )
#define SOCKUNLOCK(sck) 	assert( RTEMS_SUCCESSFUL == rtems_semaphore_release((sck)->mutx) )

static UdpSockRec	socks[NSOCKS] = {{0}};
static int         nsocks         = 0;
static IpBscIf		intrf         = 0;


/* A helper type for copying 'spa' and 'tpa' fields in an ARP
 * header. Note that (provided the ARP header is 32-bit aligned)
 * that 'spa/tpa' which are 4-byte IP addresses are NOT 4-byte
 * aligned. This means that we cannot portably copy them as
 * 32-bit word entities.
 * Use this helper type (which was tuned to produce fast code
 * with gcc).
 */
union arpip_2_aligned {
	uint8_t	 ipbytes[4];
	uint16_t aligner[2]; /* dummy to provide 2-byte alignment */
}; /* spa/tpa are 2-byte aligned */

static inline uint32_t get_spa(IpArpRec *arphdr)
{
register union { uint32_t ip; union arpip_2_aligned bytes; } rval;
	rval.bytes = *(union arpip_2_aligned*)arphdr->spa;
	return rval.ip;
}

static inline uint32_t get_tpa(IpArpRec *arphdr)
{
register union { uint32_t ip; union arpip_2_aligned bytes; } rval;
	rval.bytes = *(union arpip_2_aligned*)arphdr->tpa;
	return rval.ip;
}

static inline void     set_tpa(IpArpRec *arphdr, uint32_t ipaddr)
{
register union { uint32_t ip; union arpip_2_aligned bytes; } val;
	val.ip = ipaddr;
	*(union arpip_2_aligned*)arphdr->tpa = val.bytes;
}

/* Permanent / 'static' flag */
#define ARP_PERM	((rtems_interval)(-1))

typedef struct ArpEntryRec_ {
	uint32_t		ipaddr;
	uint8_t			hwaddr[6];
	rtems_interval	ctime;
} ArpEntryRec, *ArpEntry;

/* NOTES: On class C networks (or equivalent A/B subnets)
 *        there will never be collisions in the cache.
 *
 *        Dont change size (256) w/o adapting hashing
 *        algorithm and w/o changing the 'modulo 256'
 *        operation implicit into declaring the 'h'
 *        loopcounter uint8_t.
 *
 */
static ArpEntry arpcache[256] = {0};

/* Keep an available entry around */
static ArpEntry	arpScratch    = 0; /* FIXME: should be free()ed when unloading module */

int lanIpBscAutoRefreshARP  = 1;

/* don't bother about MSBs; assume we're on a LAN anyways */
#define ARPHASH(h,ipaddr)			\
	do {							\
		uint32_t ipaddr_h_;			\
		ipaddr_h_ = ntohl(ipaddr);	\
		h  = ipaddr_h_;				\
		h += ipaddr_h_>>8;			\
	} while (0)

#define ARPLOCK(pd)		assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain((pd)->mutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )
#define ARPUNLOCK(pd) 	assert( RTEMS_SUCCESSFUL == rtems_semaphore_release((pd)->mutx) )

#ifdef DEBUG
/* Debug helpers */

/* Print an ethernet address to 'file' */
static void
prether(FILE *f, const unsigned char *ea)
{
int i;
	for (i=0; i<5; i++)
		fprintf(f,"%02X:",*ea++);
	fprintf(f,"%02X",*ea);
}
#endif

/* Dump an ARP cache entry */
static void
prarp(FILE *f, ArpEntry e)
{
char           ipbuf[INET_ADDRSTRLEN+1];
struct in_addr ina;
rtems_interval now;

	if ( !f )
		f = stdout;

	ina.s_addr =  e->ipaddr;
	if (!inet_ntop(AF_INET, &ina, ipbuf, sizeof(ipbuf))) {
		perror("\nUnable to print ARP entry (inet_ntop failed)");
		return;
	}
	rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &now);
	fprintf(f,"%s: ", ipbuf);
	prether(f, e->hwaddr);
	fprintf(f," (age: ");
	if ( ARP_PERM == e->ctime )
		fprintf(f,"STATIC)\n");
	else
		fprintf(f,"%lus)\n", now-e->ctime);
}

int
arpLookup(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int cacheonly)
{
ArpEntry rval;
int      i,n;
uint8_t  hh;
uint8_t  h;

		if ( ISBCST(ipaddr, pd->nmask) ) {
			if ( enaddr )
				memset(enaddr, 0xff, 6);
			return 0;
		}

		if ( !enaddr ) {
			/* They just want to send a lookup. If we (eventually)
			 * receive an answer then a cache entry will be created
			 * or updated (asynchronously).
			 */
			NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr);
			return 0;
		}


		ARPHASH(h, ipaddr);

		for ( n = 0; n < ARP_SEND_RETRY; n++ ) {

			/* Abuse the TX lock */
			ARPLOCK(pd);
			for ( i = 0, hh=h; i<CACHE_OVERLAP; i++, hh++ ) {

				if ( !(rval = arpcache[hh]) )
					continue;

				if ( ipaddr == rval->ipaddr ) {
					memcpy(enaddr, rval->hwaddr, 6);
					ARPUNLOCK(pd);
					return 0;
				}
			}
			ARPUNLOCK(pd);

			if ( cacheonly )
				break;

			/* must do a new lookup */
			NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr);

			/* should synchronize but it's easier to just delay and try again */
			rtems_task_wake_after(1);
		}
		
		return -ENOTCONN;
}

int
arpPutEntry(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int perm)
{
ArpEntry rval;
ArpEntry newe;
int      i;
int      empty;
uint8_t  oh;
uint8_t  h;
int      err = -ENOSPC;

		ARPHASH(h, ipaddr);

		/* Abuse the TX lock */
		ARPLOCK(pd);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pd);
			newe = malloc(sizeof(*newe));
			ARPLOCK(pd);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}
		for ( i=0, oh=h, empty=-1; i<CACHE_OVERLAP; i++, h++ ) {
			if ( !(rval = arpcache[h]) ) {
				if ( empty < 0 )
					empty=h;
				continue;
			}
			if ( ipaddr == rval->ipaddr ) {
				memcpy(rval->hwaddr, enaddr, 6);
				if ( perm > 0 ) {
					rval->ctime = ARP_PERM;
				} else {
					/* arg 'perm' == -1 revokes 'permanent' flag */
					if ( -1 == perm || ARP_PERM != rval->ctime )
						rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
				}
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("arpPutEntry cache hit, refreshing entry #%i: ", h);
					prarp(0, rval);
				}
#endif
				/* Done. release pre-allocated entry and leave */
				err = 0;
				goto egress;
			}
			/* permanent entries are always 'newest' (ctime in the future)
			 * hence the first non-permanent entry will be found
			 * by this algorithm.
			 */
			if ( (uint32_t)rval->ctime < (uint32_t)arpcache[oh]->ctime )
				oh = h;
		}

		if ( empty<0 ) {
			/* all slots full; must evict oldest entry */
			if ( arpcache[oh]->ctime == ARP_PERM ) {
				fprintf(stderr,"arpPutEntry: too many permanent entries, unable to allocate slot\n");
				err = -ENOSPC;
				goto egress;
			}
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_ARP ) {
				printf("arpPutEntry no more slots, evicting #%i: ", oh);
				prarp(0, arpcache[oh]);
			}
#endif
			h = oh;
		} else {
			h = empty;
			/* use new entry */
			arpcache[h] = newe;
			newe        = 0;
		}
		rval = arpcache[h];
		rval->ipaddr = ipaddr;
		memcpy(rval->hwaddr, enaddr, 6);
		if ( perm )
			rval->ctime = ARP_PERM;
		else
			rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);

#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("arpPutEntry writing entry #%i: ", h);
			prarp(0, rval);
		}
#endif

		err = 0;

egress:
		if ( newe && !arpScratch ) {
			arpScratch = newe;
			newe       = 0;
		}
		ARPUNLOCK(pd);

		free(newe);

		return err;
}

void
arpDelEntry(IpBscIf pd, uint32_t ipaddr)
{
ArpEntry rval, found = 0;
int      i;
uint8_t  h;

		ARPHASH(h, ipaddr);

		/* Abuse the TX lock */
		ARPLOCK(pd);
		for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
				if ( !(rval = arpcache[h]) )
					continue;
				if ( ipaddr == rval->ipaddr ) {
					arpcache[h] = 0;
					found = rval;
					break;
				}
		}
		if ( found && !arpScratch ) {
			arpScratch = found;
			found      = 0;
		}
		ARPUNLOCK(pd);
		free(found);
}

void
arpFlushCache(IpBscIf pd, int perm_also)
{
int i;

	/* cleanup everything */

	ARPLOCK(pd);

	for ( i=0; i<sizeof(arpcache)/sizeof(arpcache[0]); i++ ) {
		if ( perm_also || (arpcache[i] && ARP_PERM != arpcache[i]->ctime) ) {
			free(arpcache[i]);
			arpcache[i] = 0;
		}
	}

	free(arpScratch);
	arpScratch = 0;

	ARPUNLOCK(pd);
}

void
arpDumpCache(IpBscIf pd, FILE *f)
{
ArpEntryRec           abuf;
rtems_interrupt_level key;
int                   i;

	if ( !f )
		f = stdout;

	for ( i=0; i<sizeof(arpcache)/sizeof(arpcache[0]); i++ ) {
		if ( (volatile ArpEntry) arpcache[i] ) {

			rtems_interrupt_disable(key);

				/* it might have gone... */
				if ( (volatile ArpEntry) arpcache[i] ) {
					abuf = *arpcache[i];

					rtems_interrupt_enable(key);

					fprintf(f,"ARP cache entry #%i: ",i);
					prarp(f, &abuf);
				} else {

					rtems_interrupt_enable(key);
				}
		}
	}
}

void
arpScavenger(IpBscIf pd, rtems_interval maxage, rtems_interval period, int nloops)
{
rtems_interval        ancient,sec,now;
int                   i;
rtems_interrupt_level key;
ArpEntry              e;

	while ( 1 ) {
		/* calculate oldest acceptable 'ctime' */
		rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &now);
		ancient = now - maxage;

		for ( i=0; i<sizeof(arpcache)/sizeof(arpcache[0]); i++ ) {
			rtems_interrupt_disable(key);
			if ( !(e = arpcache[i]) ) {
				rtems_interrupt_enable(key);
				continue;
			}
			if ( (uint32_t)e->ctime < (uint32_t)ancient ) {
				/* evict */
				arpcache[i] = 0;
				if ( !arpScratch ) {
					arpScratch = e;
					e          = 0;
				}
				rtems_interrupt_enable(key);
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					ARPLOCK(pd);
					printf("Evicting entry #%i from ARP cache: ",i);
					/* Note the race condition, arpScratch could have
					 * changed but for debugging that's good enough...
					 */
					if ( e )
						prarp(0, e);
					else if ( arpScratch )		/* e has gone to scratch   */
						prarp(0, arpScratch);	/* probably hasn't changed */
					else
						printf("\n");
					ARPUNLOCK(pd);
				}
#endif
				free(e);
			} else {
				rtems_interrupt_enable(key);

#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					ARPLOCK(pd);
					printf("ARP cache entry #%i: ",i);
					if (arpcache[i])
						prarp(0, arpcache[i]);
					else
						printf("\n");
					ARPUNLOCK(pd);
				}
#endif
			}
		}

		/* let nloops<0 loop forever */
		if ( nloops > 0 && ! --nloops )
			break; /* don't sleep after last iteration */

		/* sleep for 'period' seconds */
		rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &sec);
		rtems_task_wake_after(sec*period);
	}
}

/* fillin our source addresses (MAC, IP, SPORT) and checksums
 * (UDP csum unused)
 */

static inline void
fillinSrcCsumIp(IpBscIf ifc, LanIp buf_p)
{
	memcpy(buf_p->ll.src, &ifc->arpreq.ll.src, sizeof(buf_p->ll.src));
	buf_p->ip.src              = ifc->ipaddr;

	buf_p->ip.csum             = 0;
	buf_p->ip.csum             = htons(in_cksum_hdr((void*)&buf_p->ip));
}

static inline void
fillinSrcCsumUdp(IpBscIf ifc, LanUdpHeader buf_p, int port)
{
	buf_p->udp.sport = htons(port);
	buf_p->udp.csum = 0;
	fillinSrcCsumIp(ifc, &buf_p->hdr);
}

/* Copy IP and MAC src->dest */
static inline void
src2dstIp(LanIp p)
{
	memcpy(p->ll.dst, p->ll.src, sizeof(p->ll.dst));
	p->ip.dst = p->ip.src;
}

static inline void
src2dstUdp(LanUdpHeader p)
{
	src2dstIp(&p->hdr);
	p->udp.dport = p->udp.sport;
}

static int
handleArp(rbuf_t **ppbuf, IpBscIf pd)
{
int			isreq = 0;
rbuf_t		*p    = *ppbuf;
IpArpRec	*pipa = &lpkt_arp(p);
uint32_t    xx;


	 /* 0x0001 == Ethernet, 0x0800 == IP */
	NETDRV_READ_INCREMENTAL(pd, pipa, 8);
	if ( htonl(0x00010800) != *(uint32_a_t*)pipa )
		return 8;

	xx = * ( (uint32_a_t *) pipa + 1 );

	/* 0x06 hw addr len, 0x04 proto len, 0x0001 ARP REQUEST */
	if        ( htonl(0x06040001) == xx ) {
		isreq = 1;
	/* 0x06 hw addr len, 0x04 proto len, 0x0002 ARP REPLY   */
	} else if ( htonl(0x06040002) != xx ) {
		return 8;
	}

	/* Fill rest of ARP packet            */
	NETDRV_READ_INCREMENTAL(pd, pipa->sha, 5*4);

	if ( isreq ) {
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP )
			printf("got ARP request for %d.%d.%d.%d\n",pipa->tpa[0],pipa->tpa[1],pipa->tpa[2],pipa->tpa[3]); 
#endif
		if ( get_tpa(pipa) != pd->ipaddr )
			return sizeof(*pipa);

		/* they mean us; send reply */
		memcpy( pd->arprep.ll.dst,  pipa->sha, 6);
		memcpy( pd->arprep.arp.tha, pipa->sha, 10);

#if defined(DEBUG)
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("MATCH -> sending\n");
#if 0
			{
			extern void md(void*,int);
			md(&pd->arprep, sizeof(pd->arprep));
			}
#endif
		}
#endif

		NETDRV_SND_PACKET(pd, 0, 0, &pd->arprep, sizeof(pd->arprep));
	} else {
		/* a reply to our request */
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("got ARP reply from "); prether(stdout, pipa->sha);
			printf("\n");
		}
#endif
		arpPutEntry(pd, get_spa(pipa), pipa->sha, 0);
	}

	return sizeof(*pipa);
}

static int
handleIP(rbuf_t **ppbuf, IpBscIf pd)
{
int          rval = 0, l, nbytes, i;
rbuf_t		 *p = *ppbuf;
IpHeaderRec  *pip = &lpkt_ip(p);
uint16_t	 dport;
int			 isbcst = 0;
LanUdpHeader hdr;

	NETDRV_READ_INCREMENTAL(pd, pip, sizeof(*pip));
	rval += sizeof(*pip);

	/* accept IP unicast and broadcast */
	if ( pip->dst != pd->ipaddr && ! (isbcst = ISBCST(pip->dst, pd->nmask)) )
		return rval;

#ifdef DEBUG
	if ( (lanIpDebug & DEBUG_IP) && pip->dst == pd->ipaddr ) {
		printf("accepting IP unicast, proto %i\n", pip->prot);
	}
#endif

	/* reject non-trivial headers (version != 4, header length > 5, fragmented,
     * i.e., MF (more fragments) or the offset are set
	 */
	if ( pip->vhl != 0x45 || ntohs(pip->off) & 0x9fff ) {
#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) )
			printf("dropping IP packet, vhl %i, off %i\n", pip->vhl, ntohs(pip->off));
#endif
		return rval;
	}

	nbytes = ntohs(pip->len);
	l = ((nbytes - sizeof(*pip)) + 3) & ~3;
	switch ( pip->prot ) {
		case 1 /* ICMP */:
		{
		IcmpHeaderRec *picmp = &lpkt_icmp(p);

			if ( sizeof(*p)-sizeof(*pip) - sizeof(EthHeaderRec) >= l ) {
				NETDRV_READ_INCREMENTAL(pd, picmp, l);
				rval += l;
				if ( picmp->type == 8 /* ICMP REQUEST */ && picmp->code == 0 ) {
#ifdef DEBUG
					if ( lanIpDebug & DEBUG_ICMP )
						printf("handling ICMP ECHO request\n");
#endif
					picmp->type = 0; /* ICMP REPLY */
					picmp->csum = 0;
					picmp->csum = ipcsum((uint8_t*)picmp, nbytes-sizeof(*pip));
					lpkt_eth(p).type = htons(0x800); /* IP */

					/* refresh peer's ARP entry */
					if ( lanIpBscAutoRefreshARP ) {
						arpPutEntry(pd, lpkt_ip(p).src, lpkt_eth(p).src, 0);
					}

					src2dstIp(&lpkt_iphdr(p));
					fillinSrcCsumIp(pd, &lpkt_iphdr(p));

					NETDRV_SND_PACKET(pd, 0, 0, p, sizeof(EthHeaderRec) + nbytes);
				}
			}
		}
		break;

		case 17 /* UDP */:
		{
		LanUdpHeader pudp = &lpkt_udphdr(p);

			/* UDP header is word aligned -> OK */
			NETDRV_READ_INCREMENTAL(pd, &pudp->udp, sizeof(pudp->udp));
			rval += sizeof(pudp->udp);
			l    -= sizeof(pudp->udp);
			dport = ntohs(pudp->udp.dport);
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_UDP ) {
				char buf[INET_ADDRSTRLEN];
				printf("handling UDP packet (dport %i%s)\n", dport, isbcst ? ", BCST":"");
				inet_ntop(AF_INET, &pudp->hdr.ip.src, buf, INET_ADDRSTRLEN);
				buf[INET_ADDRSTRLEN-1]=0;
				printf("from %s:%i ...", buf, ntohs(pudp->udp.sport));
			}
#endif
			_Thread_Disable_dispatch();
			for ( i=0; i<NSOCKS; i++ ) {
				if ( socks[i].port == dport ) {
					if ( FLG_ISCONN & socks[i].flags ) {
						hdr = &socks[i].hdr;
						/* filter source IP and port */
						if (    hdr->udp.dport  != pudp->udp.sport
							|| (hdr->hdr.ip.dst != pudp->hdr.ip.src && ! ISBCST(hdr->hdr.ip.dst, socks[i].intrf->nmask)) ) {
							_Thread_Enable_dispatch();
#ifdef DEBUG
							if ( lanIpDebug & DEBUG_UDP ) {
								printf("DROPPED [peer != connected peer]\n");
							}
#endif
							return rval;
						}
					}
					_Thread_Enable_dispatch();
					/* slurp data */
					NETDRV_READ_INCREMENTAL(pd, pudp->pld, l);
					rval += l;

					/* Refresh peer's ARP entry */
					if ( lanIpBscAutoRefreshARP ) {
						arpPutEntry(pd, pudp->hdr.ip.src, pudp->hdr.ll.src, 0);
					}

					_Thread_Disable_dispatch();
					/* see if socket is still alive */
					if ( socks[i].port == dport ) {
						UdpSockMsgRec msg;
						msg.pkt = p;
						msg.len = nbytes - sizeof(IpHeaderRec) - sizeof(UdpHeaderRec);
						if ( RTEMS_SUCCESSFUL == rtems_message_queue_send(socks[i].msgq, &msg, sizeof(msg)) ) {
							socks[i].nbytes += msg.len;
							/* they now own the buffer */
							*ppbuf = 0;
						}
					}
					break;
				}
			}
			_Thread_Enable_dispatch();
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_UDP )
				printf("%s\n", *ppbuf ? "DROPPED" : "ACCEPTED [passed to socket queue]");
#endif			
		}
		break;

		default:
		break;
	}

	return rval;
}

/* Handle ARP and ICMP echo (ping) requests
 * Dispatch UDP packets to trivial 'sockets'
 *
 * Need to pass a pointer to the buffer pointer;
 * (*pprb) is set to NULL if the buffer was handed
 * on to a 'socket'.
 * 
 * RETURNS: number of remaining elements.
 */

int
lanIpProcessBuffer(IpBscIf pd, rbuf_t **pprb, int len)
{
rbuf_t         *prb = *pprb;
EthHeaderRec   *pll = &lpkt_eth(prb);
uint16_t        tt;

	NETDRV_READ_INCREMENTAL(pd, prb, sizeof(*pll));
	len -= sizeof(*pll);

	tt = pll->type;
	if ( htons(0x806) == tt ) {
		/* ARP */
		len -= handleArp(pprb, pd);
	} else if ( htons(0x800) == tt ) {
		/* IP  */
		len -= handleIP(pprb, pd);
	} else {
#ifdef DEBUG
			if (lanIpDebug & DEBUG_IP) {
				int i;
				printf("Ethernet: dropping 0x%04x\n", ntohs(pll->type));
				for (i=0; i<20; i++)
				printf("%02x ", *(((char*)prb)+i));
				printf("\n");
			}
#endif
	}

	return len;
}


IpBscIf
lanIpBscIfAlloc()
{
IpBscIf          		rval = malloc(sizeof(*rval));
rtems_status_code		sc;
rtems_interrupt_level	key;

	if ( !rval )
		return 0;

	rtems_interrupt_disable(key);
	if ( intrf ) {
		rtems_interrupt_enable(key);
		fprintf(stderr,"Only support one interface for now\n");
		free(rval);
		return 0;
	}

	intrf = rval;
	rtems_interrupt_enable(key);

	memset( rval, 0, sizeof(*rval) );

	sc = rtems_semaphore_create(
			rtems_build_name('i','p','m','x'), 
			1,
			RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&rval->mutx);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "lanIpCb: unable to create mutex\n");

		intrf = 0;

		free(rval);
		return 0;
	}
	return rval;
}

IpBscIf
lanIpBscIfCreate(void *drv_p, char *ipaddr, char *netmask)
{
uint8_t		      (*enaddr)[6];
IpBscIf           ipbif_p;

	if ( ! (ipbif_p = lanIpBscIfAlloc()) )
		return 0;

	ipbif_p->drv_p  = drv_p;

	ipbif_p->ipaddr = inet_addr(ipaddr);
	ipbif_p->nmask  = inet_addr(netmask);


	/* convenience variable */
	enaddr = &ipbif_p->arpreq.ll.src;
	
	/* Setup ARP templates for request and reply */

	/* LL HEADERS FIRST */

	/* REQUEST */
		/* DST: bcast address            */
		memset(&ipbif_p->arpreq.ll.dst, 0xff, sizeof(*enaddr));
		/* SRC: drv_p's ethernet address  */
		NETDRV_READ_ENADDR(ipbif_p, (uint8_t*)enaddr);
		/* TYPE/LEN is ARP (0x806)       */
		ipbif_p->arpreq.ll.type = htons(0x806);
	/* REPLY   */
		/* DST: ??? filled by daemon     */

		/* SRC: drv_p's ethernet address  */
		memcpy(ipbif_p->arprep.ll.src, enaddr, sizeof(*enaddr));
		/* TYPE/LEN is ARP (0x806)       */
		ipbif_p->arprep.ll.type = htons(0x806);

	/* ARP PORTION */
	/* HW and PROTO type for both */
	ipbif_p->arprep.arp.htype = ipbif_p->arpreq.arp.htype = htons(1);     /* Ethernet */
	ipbif_p->arprep.arp.ptype = ipbif_p->arpreq.arp.ptype = htons(0x800); /* IP       */
	ipbif_p->arprep.arp.hlen  = ipbif_p->arpreq.arp.hlen  = 6; 
	ipbif_p->arprep.arp.plen  = ipbif_p->arpreq.arp.plen  = 4; 

	ipbif_p->arprep.arp.oper  = htons(2); /* ARP REPLY   */
	ipbif_p->arpreq.arp.oper  = htons(1); /* ARP REQUEST */

	/* REQUEST */
		/* TARGET HW ADDR: bcst                       */ 
		memset(ipbif_p->arpreq.arp.tha, 0xff,   sizeof(*enaddr));
		/* TARGET IP ADDR: ??? (filled by requestor)  */

		/* SOURCE HW ADDR: drv_p's ethernet address    */
		memcpy(&ipbif_p->arpreq.arp.sha, enaddr, sizeof(*enaddr));
		/* SOURCE IP ADDR: our IP                     */
		memcpy(&ipbif_p->arpreq.arp.spa, &ipbif_p->ipaddr, 4);

	/* REPLY */
		/* TARGET HW ADDR: ??? (filled by daemon)     */ 

		/* TARGET IP ADDR: ??? (filled by daemon)     */

		/* SOURCE HW ADDR: drv_p's ethernet address    */
		memcpy(ipbif_p->arprep.arp.sha, enaddr, sizeof(*enaddr));
		/* SOURCE IP ADDR: our IP                     */
		memcpy(ipbif_p->arprep.arp.spa, &ipbif_p->ipaddr, 4);
	return ipbif_p;
}

void *
lanIpBscIfGetDrv(IpBscIf ipbif_p)
{
	return ipbif_p->drv_p;
}

void
lanIpBscIfDestroy(IpBscIf pd)
{
	if ( pd ) {
		assert( nsocks == 0 );
		assert( intrf == pd );
		intrf = 0;
		arpFlushCache(pd,1);
		rtems_semaphore_delete(pd->mutx);
		free(pd);
	}
}

#ifdef DEBUG
const uint8_t dstenaddr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }; /* { 0x00,0x30,0x65,0xC9,0x9D,0xF8 }; */
const uint8_t srcenaddr[6] = { 0x08,0x00,0x56,0x00,0x01,0x00 };


#endif

void
udpSockHdrsSetlen(LanUdpHeader p, int payload_len)
{
	p->hdr.ip.len   = htons(payload_len + sizeof(UdpHeaderRec) + sizeof(IpHeaderRec));
	p->hdr.ip.csum  = 0;
	
	p->hdr.ip.csum  = in_cksum_hdr((void*)&p->hdr.ip);

	p->udp.len  = htons(payload_len + sizeof(UdpHeaderRec));
}

int
udpSockHdrsInit(int sd, LanUdpHeader p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id)
{
int rval = 0;

	if ( dipaddr ) 
		rval = arpLookup(socks[sd].intrf, dipaddr, p->hdr.ll.dst, 0);
	else /* they want to leave it blank */
		memset(p->hdr.ll.dst,0,6);

	p->hdr.ll.type  = htons(0x0800);	/* IP */

	p->hdr.ip.vhl   = 0x45;	/* version 4, 5words length */
	p->hdr.ip.tos   = 0x30; 	/* priority, minimize delay */
	p->hdr.ip.len   = 0;
	p->hdr.ip.id    = htons(ip_id);/* ? */
	p->hdr.ip.off   = htons(0);
	p->hdr.ip.ttl   = 4;
	p->hdr.ip.prot  = 17;	/* UDP */
	p->hdr.ip.dst   = dipaddr;

	p->udp.dport = htons(dport);
	p->udp.len   = 0;

	fillinSrcCsumUdp(socks[sd].intrf, p, sd >= 0 ? socks[sd].port : 0);

	/* reset checksum; length is not correct yet */
	p->hdr.ip.csum  = 0;

	return rval;
}

void
udpSockHdrsReflect(LanUdpHeader p)
{
uint16_t  port = p->udp.dport;
	src2dstUdp(p);
	fillinSrcCsumUdp(intrf, p, port);
}

/* socks array is protected by disabling thread dispatching */

/* Find socket for 'port' (host byte order)
 * and return (dispatching disabled on success)
 */
static int
sockget(int port)
{
int i;
	_Thread_Disable_dispatch();
	for (i=0; i<NSOCKS; i++) {
		if ( socks[i].port == port )
			return i;
	}
	_Thread_Enable_dispatch();
	return -ENFILE;
}

int
udpSockCreate(int port)
{
int       rval = -1, i, scan_for_port;
rtems_id  q = 0;
rtems_id  m = 0;

	if ( port < 0 || port >= 1<<16 )
		return -EINVAL;

	if ( RTEMS_SUCCESSFUL != rtems_message_queue_create(
			rtems_build_name('u','d','p','q'),
			QDEPTH,
			sizeof(UdpSockMsgRec),
			RTEMS_FIFO | RTEMS_LOCAL,
			&q) ) {
		q    =  0;
		rval = -ENOSPC;
		goto egress;
	}

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
			rtems_build_name('u','d','p','l'),
			1,
			RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&m) ) {
		m    =  0;
		rval = -ENOSPC;
		goto egress;
	}


	if ( (rval = sockget(0)) < 0 ) {
		/* no free slot */
		goto egress;
	}

	if (  (scan_for_port = (0 == port)) ) {
		/* assign an unused port number */
		port = DEFLT_PORT;
	}

again:
	for ( i=0; i<NSOCKS; i++ ) {
		if ( socks[i].port == port ) {
			if ( scan_for_port ) {
				port++;
				goto again;
			} else {
				/* they want to use a fixed port number
				 * but it is already used
				 */
				_Thread_Enable_dispatch();
				rval = -EADDRINUSE;
				goto egress;
			}
		}
	}

	/* everything OK */
	socks[rval].port   = port;

	nsocks++;

	_Thread_Enable_dispatch();

	socks[rval].intrf  = intrf;
	socks[rval].msgq   = q;
	socks[rval].mutx   = m;
	socks[rval].flags  = 0;
	socks[rval].nbytes = 0;

	udpSockHdrsInit(rval, &socks[rval].hdr, 0, 0, 0);

	q             = 0;
	m             = 0;

egress:
	if ( q )
		rtems_message_queue_delete(q);
	if ( m )
		rtems_semaphore_delete(m);
	return rval;
}

/* destroying a sock somebody is blocking on is BAD */
int
udpSockDestroy(int sd)
{
rtems_id         q = 0;
rtems_id         m = 0;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	_Thread_Disable_dispatch();
		if (socks[sd].port) {
			socks[sd].intrf = 0;
			socks[sd].port = 0;
			q = socks[sd].msgq;
			socks[sd].msgq = 0;
			m = socks[sd].mutx;
			socks[sd].mutx = 0;
			
			nsocks--;
		}
	_Thread_Enable_dispatch();

	if (q) {
		/* drain queue */
		UdpSockMsgRec msg;
		size_t sz = sizeof(msg);
		while ( RTEMS_SUCCESSFUL == rtems_message_queue_receive(
										q,
										&msg,
										&sz,
										RTEMS_NO_WAIT,
										0) ) {
			relrbuf(msg.pkt);
		}
		rtems_message_queue_delete(q);
		return 0;
	}

	if (m) {
		rtems_semaphore_delete(m);
	}
	return -EBADFD;
}

int
udpSockConnect(int sd, uint32_t dipaddr, int dport)
{
int rval      = -1;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	SOCKLOCK( & socks[sd] );

	if ( 0 == dipaddr && 0 == dport ) {
		/* disconnect */
		if ( ! (FLG_ISCONN & socks[sd].flags) ) {
			rval = -ENOTCONN;
			goto egress;
		}

		socks[sd].flags &= ~FLG_ISCONN;

	} else {

		if ( dport <= 0 || dport >= 1<<16 ) {
			rval = -EINVAL;
			goto egress;
		}

#if 0 /* BSD sockets can be re-associated on the fly; follow these semantics */
		if ( (FLG_ISCONN & socks[sd].flags) ) {
			rval = -EISCONN;
			goto egress;
		}
#endif

		if ( udpSockHdrsInit(sd, &socks[sd].hdr, dipaddr, dport, 0) ) {
			/* ARP lookup failure; BSD sockets probably would not
			 * fail here...
			 */
			rval = -ENOTCONN;
			goto egress;
		}

		socks[sd].flags |= FLG_ISCONN;
	}

	rval = 0;

egress:
	SOCKUNLOCK( &socks[sd] );
	return rval;
}

LanIpPacketRec *
udpSockRecv(int sd, int timeout_ticks)
{
UdpSockMsgRec     msg;
size_t		      sz = sizeof(msg);
rtems_status_code sc;
	if ( sd < 0 || sd >= NSOCKS ) {
		return 0;
	}
	if ( RTEMS_SUCCESSFUL != (sc=rtems_message_queue_receive(
								socks[sd].msgq,
								&msg,
								&sz,
								timeout_ticks ? RTEMS_WAIT : RTEMS_NO_WAIT,
								timeout_ticks < 0 ? RTEMS_NO_TIMEOUT : timeout_ticks)) ) {
		return 0;
	}
	_Thread_Disable_dispatch();
	socks[sd].nbytes -= msg.len;
	_Thread_Enable_dispatch();
	return msg.pkt;
}


/* 
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
 */
int
udpSockNRead(int sd)
{
int rval;
	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	_Thread_Disable_dispatch();
	rval = ( 0 == socks[sd].port ) ? -EBADF : socks[sd].nbytes;
	_Thread_Enable_dispatch();

	return rval;
}


int
udpSockSend(int sd, void *payload, int payload_len)
{
int          rval;
LanUdpHeader h;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	SOCKLOCK( &socks[sd] );

	if ( ! (FLG_ISCONN & socks[sd].flags) ) {
		SOCKUNLOCK( &socks[sd] );
		return -ENOTCONN;
	}

	h = &socks[sd].hdr;

	if ( arpLookup(socks[sd].intrf, h->hdr.ip.dst, h->hdr.ll.dst, 0) ) {
		SOCKUNLOCK( &socks[sd] );
		return -ENOTCONN;
	}

	udpSockHdrsSetlen(h, payload_len);

	rval = NETDRV_SND_PACKET( socks[sd].intrf, h, sizeof(*h), payload, payload_len );

	SOCKUNLOCK( &socks[sd] );

	return rval;
}

int
udpSockSendTo(int sd, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport)
{
int          rval;
LanUdpHeader h;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	SOCKLOCK( &socks[sd] );

	h = &socks[sd].hdr;

	/* if the socket is already connected only allow sending to peer */
	if ( (FLG_ISCONN & socks[sd].flags) ) {
		if (   h->hdr.ip.dst == ipaddr
			&& (unsigned short)ntohs( h->udp.dport ) == dport )
			rval = udpSockSend(sd, payload, payload_len);
		else
			rval = -EISCONN;
		SOCKUNLOCK( &socks[sd] );
		return rval;
	}

	h->hdr.ip.dst = ipaddr;
	h->udp.dport  = htons((unsigned short)dport);

	if ( arpLookup(socks[sd].intrf, h->hdr.ip.dst, h->hdr.ll.dst, 0) ) {
		SOCKUNLOCK( &socks[sd] );
		return -ENOTCONN;
	}

	udpSockHdrsSetlen(h, payload_len);

	rval = NETDRV_SND_PACKET( socks[sd].intrf, h, sizeof(*h), payload, payload_len );

	SOCKUNLOCK( &socks[sd] );

	return rval;
}


void
udpSockFreeBuf(LanIpPacketRec *b)
{
	relrbuf(b);
}

LanIpPacketRec *
udpSockGetBuf()
{
	return getrbuf();
}

int
udpSockSendBufRaw(LanIpPacket buf_p, int len)
{
	NETDRV_ENQ_BUFFER(intrf, buf_p,  len);
	return len;
}

int
udpSockSendBufRawIp(LanIpPacket buf_p)
{
int len = ntohs(lpkt_ip(buf_p).len) + sizeof(EthHeaderRec);
	NETDRV_ENQ_BUFFER(intrf, buf_p,  len);
	return len;
}

int
udpSockSendBuf(int sd, LanIpPacket buf_p)
{
uint16_t port;

	if ( sd < 0 || sd >= NSOCKS ) {
		return -1;
	}

	if ( !(port = socks[sd].port) )
		return -1;
	
	fillinSrcCsumUdp(intrf, &lpkt_udphdr(buf_p), port);

	return udpSockSendBufRawIp(buf_p);
}


