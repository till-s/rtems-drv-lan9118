/* $Id$ */

/* Rudimentary IP protocol:
 *
 *  - trivial ARP
 *  - trivial IP (only 20byte header)
 *  - ICMP echo request (ping) handling
 *
 *  - LAN only (no IP routing)
 *
 * NOTE: daemon using the 'drvLan9118IpRxCb()' must have enough stack
 *       allocated. The Callback uses ~2k itself.
 */

/* T. Straumann <strauman@slac.stanford.edu> */
#include <rtems.h>
#include <rtems/error.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <machine/in_cksum.h>

#include <lanIpProto.h>
#include <drvLan9118.h>

#define DEBUG_IP	1
#define DEBUG		0

#ifdef DEBUG
int	lanIpDebug = DEBUG;
#endif

#define CACHE_OVERLAP 10

typedef struct ArpEntryRec_ {
	uint32_t			ipaddr;
	uint8_t				hwaddr[6];
	Watchdog_Interval	ctime;
} ArpEntryRec, *ArpEntry;

static ArpEntry arpcache[256] = {0};

typedef struct IpCbDataRec_ {
	DrvLan9118		plan;
	rtems_id		mutx;
	uint32_t		ipaddr;
	struct {
		EtherHeaderRec  ll;
		IpArpRec		arp;
	} arpreq;
	struct {
		EtherHeaderRec  ll;
		IpArpRec		arp;
	} arprep;
} IpCbDataRec, *IpCbData;

#define ARPLOCK(pd)		rtems_semaphore_obtain((pd)->mutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT)
#define ARPUNLOCK(pd) 	rtems_semaphore_release((pd)->mutx)

int arpLookup(IpCbData pd, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry rval;
int      i,n;
uint8_t  hh;
uint8_t  h  = ipaddr;
		 h += ipaddr>>8;
		 h += ipaddr>>16;

		for ( n = 0; n < CACHE_OVERLAP; n++ ) {

			/* don't bother about MSB; assume we're on a LAN anyways */
			
			/* Abuse the TX lock */
			ARPLOCK(pd);
			for ( i = 0, hh=h; i<CACHE_OVERLAP && (rval = arpcache[hh]); i++, hh++ ) {
				if ( ipaddr == rval->ipaddr ) {
					memcpy(enaddr, rval->hwaddr, 6);
					ARPUNLOCK(pd);
					return 0;
				}
			}
			ARPUNLOCK(pd);

			/* must do a new lookup */
			drvLan9118TxPacket(pd->plan, 0, sizeof(pd->arpreq), 0);

			/* arpreq is locked and may be modified */
			*(uint32_t*)pd->arpreq.arp.tpa = ipaddr;

			/* send request */
			drvLan9118FifoWr(pd->plan, &pd->arpreq, sizeof(pd->arpreq));

			drvLan9118TxUnlock(pd->plan);

			/* should synchronize but it's easier to just delay and try again */
			rtems_task_wake_after(1);
		}
		
		return -1;
}

void
arpPutEntry(IpCbData pd, uint8_t *enaddr, uint32_t ipaddr)
{
ArpEntry rval;
ArpEntry newe = malloc(sizeof(*newe));	/* pre-allocate */
int      i;
uint8_t  oh;
uint8_t  h  = ipaddr;
		 h += ipaddr>>8;
		 h += ipaddr>>16;

		/* Abuse the TX lock */
		ARPLOCK(pd);
		for ( i = 0, oh=h; i<CACHE_OVERLAP && (rval = arpcache[h]); i++, h++ ) {
			if ( ipaddr == rval->ipaddr ) {
				memcpy(rval->hwaddr, enaddr, 6);
				rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
				ARPUNLOCK(pd);
				return;
			}
			if ( rval->ctime < arpcache[oh]->ctime )
				oh = h;
				
		}

		if ( rval ) {
			/* all slots full; must evict oldest entry */
			h = oh;
		} else {
			/* use new entry */
			arpcache[h] = newe;
			newe        = 0;
		}
		rval = arpcache[h];
		rval->ipaddr = ipaddr;
		memcpy(rval->hwaddr, enaddr, 6);
		rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
		ARPUNLOCK(pd);

		free(newe);
}

void
arpDelEntry(IpCbData pd, uint32_t ipaddr)
{
ArpEntry rval, found = 0;
int      i;
uint8_t  h  = ipaddr;
		 h += ipaddr>>8;
		 h += ipaddr>>16;


		/* Abuse the TX lock */
		ARPLOCK(pd);
		for ( i = 0; i<CACHE_OVERLAP && (rval = arpcache[h]); i++, h++ ) {
				if ( ipaddr == rval->ipaddr ) {
					arpcache[h] = 0;
					found = rval;
					break;
				}
		}
		ARPUNLOCK(pd);
		free(found);
}

#ifdef DEBUG
static void
prether(FILE *f, const unsigned char *ea)
{
int i;
	for (i=0; i<5; i++)
		fprintf(f,"%02X:",*ea++);
	fprintf(f,"%02X",*ea);
}
#endif

static int
handleArp(IpCbData pd)
{
IpArpRec ipa;
uint32_t *p    = (uint32_t*)&ipa.sha[0];
int      isreq = 0;

	 /* 0x0001 == Ethernet, 0x0800 == IP */
	drvLan9118FifoRd(pd->plan, &ipa, 8);
	if ( ntohl(0x00010800) != *(uint32_t*)&ipa )
		return 8;


	switch ( *(((uint32_t*)&ipa) + 1) ) {
		default:
			return 8;

		/* 0x06 hw addr len, 0x04 proto len, 0x0001 ARP REQUEST */
		case ntohl(0x06040001):
			isreq = 1;
		/* 0x06 hw addr len, 0x04 proto len, 0x0002 ARP REPLY   */
		case ntohl(0x06040002):
			break;
	}

	drvLan9118FifoRd(pd->plan, p, 5*4);

	if ( isreq ) {
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_IP )
			printf("got ARP request for %d.%d.%d.%d\n",ipa.tpa[0],ipa.tpa[1],ipa.tpa[2],ipa.tpa[3]); 
#endif
		if ( *(uint32_t*)ipa.tpa != pd->ipaddr )
			return sizeof(ipa);

		/* they mean us; send reply */
		memcpy( pd->arprep.ll.dst,  ipa.sha, 6);
		memcpy( pd->arprep.arp.tha, ipa.sha, 10);

#ifdef DEBUG
		if ( lanIpDebug & DEBUG_IP ) {
			extern void md(void*,int);
			printf("MATCH -> sending\n");
			md(&pd->arprep, sizeof(pd->arprep));
		}
#endif

		drvLan9118TxPacket(pd->plan, &pd->arprep, sizeof(pd->arprep), 0);
	} else {
		/* a reply to our request */
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_IP ) {
			printf("got ARP reply from "); prether(stdout, ipa.sha);
			printf("\n");
		}
#endif
		arpPutEntry(pd, ipa.sha, *(uint32_t*)ipa.spa);
	}

	return sizeof(ipa);
}

static int
handleIP(EtherHeaderRec *peh, IpCbData pd)
{
int         rval = 0, l, nbytes;
struct {
	EtherHeaderRec eh;
	IpHeaderRec    ih;
	IcmpHeaderRec  icmph;
	uint8_t        data[1500];
}           p;



	drvLan9118FifoRd(pd->plan, &p.ih, sizeof(p.ih));
	rval += sizeof(p.ih);

	if ( p.ih.dst != pd->ipaddr )
		return rval;

	/* reject non-trivial headers (version != 4, header length > 5, fragmented,
     * i.e., MF (more fragments) or the offset are set
	 */
	if ( p.ih.vhl != 0x45 || ntohs(p.ih.off) & 0x9fff )
		return rval;

	nbytes = ntohs(p.ih.len);
	l = ((nbytes - sizeof(p.ih)) + 3) & ~3;
	switch ( p.ih.prot ) {
		case 1 /* ICMP */:
		if ( sizeof(p)-sizeof(p.ih) - sizeof(p.eh) >= l ) {
			drvLan9118FifoRd(pd->plan, &p.icmph, l);
			rval += l;
			if ( p.icmph.type == 8 /* ICMP REQUEST */ && p.icmph.code == 0 ) {
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_IP )
					printf("handling ICMP ECHO request\n");
#endif
				p.icmph.type = 0; /* ICMP REPLY */
				p.icmph.csum = 0;
				memcpy( p.eh.dst, peh->src, 6 );
				memcpy( p.eh.src, pd->arprep.ll.src, 6);
				p.eh.type = htons(0x800); /* IP */
				p.ih.dst  = p.ih.src;
				p.ih.src  = pd->ipaddr; 
				p.ih.csum = 0;
				p.ih.csum = htons(in_cksum_hdr((void*)&p.ih));
				drvLan9118TxPacket(pd->plan, &p, sizeof(EtherHeaderRec) + nbytes, 0);
			}
		}
		break;
	}

	return rval;
}

/* Handle ARP and ICMP echo (ping) requests */
int
drvLan9118IpRxCb(DrvLan9118 plan, uint32_t len, void *arg)
{
IpCbData        pd = arg;
EtherHeaderRec	eh;

	drvLan9118FifoRd(plan, &eh, sizeof(eh));
	len -= sizeof(eh);

	switch ( eh.type ) {
		case htons(0x806) /* ARP */:
			len -= handleArp(pd);
			break;

		case htons(0x800) /* IP  */:
			len -= handleIP(&eh, pd);
			break;

		default:
			break;
	}

	return len;
}


IpCbData
lanIpCbDataCreate(DrvLan9118 plan, char *ipaddr)
{
IpCbData          rval = malloc(sizeof(*rval));
uint8_t		      (*enaddr)[6];
rtems_status_code sc;

	if ( !rval )
		return 0;

	rval->plan   = plan;

	rval->ipaddr = inet_addr(ipaddr);

	sc = rtems_semaphore_create(
			rtems_build_name('i','p','m','x'), 
			1,
			RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&rval->mutx);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "lanIpCb: unable to create mutex\n");
		free(rval);
		return 0;
	}


	/* convenience variable */
	enaddr = &rval->arpreq.ll.src;
	
	/* Setup ARP templates for request and reply */

	/* LL HEADERS FIRST */

	/* REQUEST */
		/* DST: bcast address            */
		memset(&rval->arpreq.ll.dst, 0xff, sizeof(*enaddr));
		/* SRC: plan's ethernet address  */
		drvLan9118ReadEnaddr(plan, (uint8_t*)enaddr);
		/* TYPE/LEN is ARP (0x806)       */
		rval->arpreq.ll.type = htons(0x806);
	/* REPLY   */
		/* DST: ??? filled by daemon     */

		/* SRC: plan's ethernet address  */
		memcpy(rval->arprep.ll.src, enaddr, sizeof(*enaddr));
		/* TYPE/LEN is ARP (0x806)       */
		rval->arprep.ll.type = htons(0x806);

	/* ARP PORTION */
	/* HW and PROTO type for both */
	rval->arprep.arp.htype = rval->arpreq.arp.htype = htons(1);     /* Ethernet */
	rval->arprep.arp.ptype = rval->arpreq.arp.ptype = htons(0x800); /* IP       */
	rval->arprep.arp.hlen  = rval->arpreq.arp.hlen  = 6; 
	rval->arprep.arp.plen  = rval->arpreq.arp.plen  = 4; 

	rval->arprep.arp.oper  = htons(2); /* ARP REPLY   */
	rval->arpreq.arp.oper  = htons(1); /* ARP REQUEST */

	/* REQUEST */
		/* TARGET HW ADDR: bcst                       */ 
		memset(rval->arpreq.arp.tha, 0xff,   sizeof(*enaddr));
		/* TARGET IP ADDR: ??? (filled by requestor)  */

		/* SOURCE HW ADDR: plan's ethernet address    */
		memcpy(&rval->arpreq.arp.sha, enaddr, sizeof(*enaddr));
		/* SOURCE IP ADDR: our IP                     */
		memcpy(&rval->arpreq.arp.spa, &rval->ipaddr, 4);

	/* REPLY */
		/* TARGET HW ADDR: ??? (filled by daemon)     */ 

		/* TARGET IP ADDR: ??? (filled by daemon)     */

		/* SOURCE HW ADDR: plan's ethernet address    */
		memcpy(rval->arprep.arp.sha, enaddr, sizeof(*enaddr));
		/* SOURCE IP ADDR: our IP                     */
		memcpy(rval->arprep.arp.spa, &rval->ipaddr, 4);
	return rval;
}

void
lanIpCbDataDestroy(IpCbData pd)
{
	if ( pd ) {
		rtems_semaphore_delete(pd->mutx);
		free(pd);
	}
}

#ifdef DEBUG
const uint8_t dstenaddr[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }; /* { 0x00,0x30,0x65,0xC9,0x9D,0xF8 }; */
const uint8_t srcenaddr[6] = { 0x08,0x00,0x56,0x00,0x01,0x00 };

uint16_t csum(uint16_t *d, int n)
{
uint32_t s = 0;
	while (n--)
		s+=*d++;
	while ( s > 0xffff )
		s = (s & 0xffff) + (s >> 16);
	return ~s & 0xffff;
}

void
drvLan9118InitUdpPacket(UdpPacketRec *p, int payload_len)
{
	memcpy(p->eh.dst, dstenaddr, 6);
	memcpy(p->eh.src, srcenaddr, 6);
	p->eh.type  = htons(0x0800);	/* IP */

	p->ih.vhl   = 0x45;	/* version 4, 5words length */
	p->ih.tos   = 0x30; 	/* priority, minimize delay */
	p->ih.len   = htons(payload_len + sizeof(UdpHeaderRec) + sizeof(IpHeaderRec));
	p->ih.id    = htons(0);	/* ? */
	p->ih.off   = htons(0);
	p->ih.ttl   = 4;
	p->ih.prot  = 17;	/* UDP */
	p->ih.src   = inet_addr("134.79.219.35"); 
	p->ih.dst   = inet_addr("134.79.216.68"); 
	p->ih.csum  = 0;
	
	p->ih.csum  = htons(in_cksum_hdr((void*)&p->ih));

	p->uh.sport = 0xabcd; 
	p->uh.dport = 0xabcd; 
	p->uh.len   = htons(payload_len + sizeof(UdpHeaderRec));
	p->uh.csum  = 0; /* csum disabled */
}
#endif
