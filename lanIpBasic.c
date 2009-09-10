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
#include <rtems/endian.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

/* Minimal stuff from BSD to get
 * 'htons()', 'inet_addr()' and 'in_cksum_hdr()'
 * and similar functionality.
 */
#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <machine/in_cksum.h>

#include <lanIpProto.h>
#include <lanIpBasic.h>

#include <lhtbl.h>

#ifdef ENABLE_PROFILE
#include "hwtmr.h"
#endif

/* Alias helper types */
typedef uint32_t	uint32_a_t __attribute__((__may_alias__));
typedef  int32_t	 int32_a_t __attribute__((__may_alias__));
typedef uint16_t	uint16_a_t __attribute__((__may_alias__));
typedef   int8_t	  int8_a_t __attribute__((__may_alias__));
typedef  uint8_t	 uint8_a_t __attribute__((__may_alias__));
typedef  int16_t	 int16_a_t __attribute__((__may_alias__));

typedef struct LanIpCalloutRec_ *LanIpCallout;

typedef struct LanIpLstNodeRec_ *LanIpLstNode;

typedef struct IpBscIfStatsRec_ {
	uint32_t	arp_nosem;
} IpBscIfStatsRec, *IpBscIfStats;


typedef struct LanIpLstNodeRec_ {
	LanIpLstNode	 l_next;
	LanIpLstNode	*l_pprev;
} LanIpLstNodeRec;

static inline void
c_enq(LanIpLstNode *where, LanIpLstNode n);

static inline void
c_deq(LanIpLstNode n);

typedef struct LanIpCalloutRec_ {
	LanIpLstNodeRec c_node;
	void          (*c_func)(void*,void*);
	void           *c_arg0;
	void           *c_arg1;
	uint32_t        c_time; 
	uint16_t        c_flags;
	uint16_t        c_slot;
} LanIpCalloutRec;

typedef union LanIpCalloutRef_ {
	LanIpLstNode    r_node;
	LanIpCallout    r_callout;
} LanIpCalloutRef;

static __inline__ LanIpCallout
lanIpCalloutNext(LanIpCallout c)
{
	return (LanIpCallout)c->c_node.l_next;
}

#define c_next  c_node.l_next
#define c_pprev c_node.l_pprev

#define CALLOUT_PENDING 	(1<<0)
#define CALLOUT_ACTIVE  	(1<<1)
#define CALLOUT_FAILEDSTOP	(1<<2)

/* Byteswap constants -- these should be optimized
 * away by the compiler.
 * Note that gcc cannot optimize normal (e.g., 'htons')
 * away since byte-swapping may be done in assembly...
 */

static inline uint16_t swapc16(uint16_t v)
__attribute__(( always_inline, error("swapc16 not inlined -- compile with at least -O1") ))
;

static inline uint32_t swapc32(uint32_t v)
__attribute__(( always_inline, error("swapc32 not inlined -- compile with at least -O1") ))
;

extern void fail_not_const()
__attribute__(( error("swapc argument not constant") ));

extern int __should_be_optimized_away();

#if BIG_ENDIAN == LITTLE_ENDIAN
#error "BIG_ENDIAN == LITTLE_ENDIAN ??"
#endif

#if BYTE_ORDER != BIG_ENDIAN && BYTE_ORDER != LITTLE_ENDIAN
#error "undefined or weird byte-ordering"
#define BYTE_ORDER_UNKOWN
#endif

/* If host byte order differs from net byte order swap a 16-bit quantity */
static inline uint16_t swapc16(uint16_t v)
{
#ifdef BYTE_ORDER_UNKNOWN
const union {
	uint8_t  yes;
	uint16_t ini;
} swp = { ini : 1 };

	/* gcc *does* optimize the endian-test away but
	 * does not recognize the result as a compile-time
	 * constant.
	 * If the test is not optimized away then we
	 * get a linkage error.
	 */
	switch ( swp.yes ) {
		case 1:
#endif
#if BYTE_ORDER != BIG_ENDIAN
			v = (v << 8) | (v >> 8) ;
			if ( ! __builtin_constant_p(v) )
				fail_not_const();
#endif
#ifdef BYTE_ORDER_UNKNOWN
		case 0:
		break;

		default: /* should never get here */
		__should_be_optimized_away();	
		break;
	}
#endif
	return v;
}

/* If host byte order differs from net byte order swap a 32-bit quantity */
static inline uint32_t swapc32(uint32_t v)
{
#ifdef BYTE_ORDER_UNKOWN
const union {
	uint8_t  yes;
	uint16_t ini;
} swp = { ini : 1 };

	/* gcc *does* optimize the endian-test away but
	 * does not recognize the result as a compile-time
	 * constant. Hence we cannot use __builtin_constant_p()
	 * to test if this optimization was done.
	 * If the test is not optimized away then we
	 * trigger a linkage error.
	 */
	switch ( swp.yes ) {
		case 1:
#endif
#if BYTE_ORDER != BIG_ENDIAN
		    v = ((v & 0x00ff00ff) << 8 ) | ((v & 0xff00ff00) >> 8 );
            v = ( v               >> 16) | ( v               << 16);

			if ( ! __builtin_constant_p(v) )
				fail_not_const();
#endif
#ifdef BYTE_ORDER_UNKNOWN
		case 0:
		break;

		default: /* should never get here */
		__should_be_optimized_away();	
		break;
	}
#endif
	return v;
}

static inline int ismcst(uint32_t v)
__attribute__(( error("ismcst not inlined -- compile with at least -O1") ));

static inline int ismcst(uint32_t v)
{
const union {
	uint8_t  yes;
	uint16_t ini;
} swp = { ini : 1 };

	switch ( swp.yes ) {
		case 1:
			return (v & 0xe0) == 0xe0;
		case 0:
			return (v & 0xe0000000) == 0xe0000000;
		default: /* should never get here */
		break;
	}
	return	__should_be_optimized_away();	
}

#define htonsc(v) swapc16(v)
#define ntohsc(v) swapc16(v)

#define htonlc(v) swapc32(v)
#define ntohlc(v) swapc32(v)

static inline int
lanIpCallout_active(LanIpCallout p_c)
{
int rval;
rtems_interrupt_level l;
	rtems_interrupt_disable(l);
		rval = p_c->c_flags & CALLOUT_ACTIVE;
	rtems_interrupt_enable(l);
	return rval;	
}

static inline int
lanIpCallout_pending(LanIpCallout p_c)
{
int rval;
rtems_interrupt_level l;
	rtems_interrupt_disable(l);
		rval = p_c->c_flags & CALLOUT_PENDING;
	rtems_interrupt_enable(l);
	return rval;	
}

static inline void
lanIpCallout_deactivate(LanIpCallout p_c)
{
rtems_interrupt_level l;
	rtems_interrupt_disable(l);
		p_c->c_flags &= ~CALLOUT_ACTIVE;
	rtems_interrupt_enable(l);
}

static inline int
lanIpCallout_failedstop(LanIpCallout p_c)
{
int rval;
rtems_interrupt_level l;
	rtems_interrupt_disable(l);
		rval = p_c->c_flags & CALLOUT_FAILEDSTOP;
	rtems_interrupt_enable(l);
	return rval;	
}
/* We cannot stop a callout that's in progress */

static int
lanIpCallout_stop(LanIpCallout c);

static int
lanIpCallout_reset(LanIpCallout c, uint32_t ticks, void (*fn)(void*,void*), void *arg0,void *arg1);

static void
lanIpCallout_init(LanIpCallout c);

/* Initialize callout facility [networking must have been initialized already] */
static rtems_id
lanIpCallout_initialize();


/* include netdriver AFTER defining VIOLATE_KERNEL_VISIBILITY (in case it uses rtems.h already) */
#include NETDRV_INCLUDE

#define DEBUG_ARP	1
#define DEBUG_IP	2
#define DEBUG_ICMP	4
#define DEBUG_UDP	8
#define DEBUG_TASK	16
#define DEBUG_IGMP  32

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

#ifndef ARP_TIMEOUT_TICKS
#define ARP_TIMEOUT_TICKS    ((rtems_interval)1) /* Ticks */
#endif

#if     RBUF_ALIGNMENT < LAN_IP_BASIC_PACKET_ALIGNMENT
#undef  RBUF_ALIGNMENT
#define RBUF_ALIGNMENT LAN_IP_BASIC_PACKET_ALIGNMENT
#endif

#define RBUF_ALGN(x)   (((x) + (RBUF_ALIGNMENT-1)) & ~(RBUF_ALIGNMENT-1))
/* Trivial RX buffers (upalign to multiple of 128 to 
 * provide safe alignment of payload for vector engines and
 * caches).
 * 
 * Cache note: If we ever use SW cache invalidation then
 *             we must be careful with the 'next' and
 *             'refcnt' fields.
 *             Possibly, the last cache line must be
 *             flushed before being invalidated so that
 *             these fields make it out to memory.
 *
 */
typedef union rbuf_ {
	LanIpPacketRec pkt;
	struct {
		uint8_t      mem[sizeof(LanIpPacketRec)];
		IpBscIf      intrf;
		union rbuf_  *next;
		uint8_t      refcnt;
	}              buf;
	uint8_t             raw[RBUF_ALGN(sizeof(LanIpPacketRec) + sizeof(union rbuf_*) + sizeof(uint8_t))];
} rbuf_t;

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

static rbuf_t		rbufs[NRBUFS]
#ifdef RBUF_ALIGNMENT
__attribute__ ((aligned(RBUF_ALIGNMENT)))
#endif
                            = {{{{{{{0}}}}}}};

/* lazy init of tbuf facility */
static int    ravail        = NRBUFS;
volatile int  lanIpBufAvail = NRBUFS;
int           lanIpBufTotal = NRBUFS;

/* FIXME: only used if we implement some sort of 'bind' operation */
uint32_t udpSockMcastIfAddr = 0;

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
	if ( (swapped = ((uintptr_t)d & 1)) ) {
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
		frb = rval->buf.next;
	} else {
		/* get from pool */
		if ( ravail ) {
			rval = &rbufs[--ravail];
		}
	}
	rval->buf.refcnt++;
	rval->buf.next = 0;
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
			if ( 0 == --b->buf.refcnt ) {
				b->buf.next  = frb;
				frb = b;
				lanIpBufAvail++;
			}
		rtems_interrupt_enable(key);
	}
}

static void refrbuf(rbuf_t *b)
{
rtems_interrupt_level key;

	rtems_interrupt_disable(key);
		b->buf.refcnt++;
	rtems_interrupt_enable(key);
}

static void *rbuf_mem = 0;

int
lanIpBscAddBufs(unsigned n)
{
rtems_interrupt_level key;
void   *mem;
rbuf_t *bufs;
int     i,j;

	if ( 0 == n ) {
		n = NRBUFS;
	}

	if ( ! (mem = malloc(sizeof(rbuf_t)*(n+1)+sizeof(void*))) )
		return -1;

	bufs = (rbuf_t*)RBUF_ALGN((uintptr_t)(mem + sizeof(void*)));

	for ( i = 0; i < n - 1; i = j) {
		j = i + 1;
		bufs[i].buf.next   = &bufs[j];
		bufs[i].buf.refcnt = 0;
	}

	bufs[i].buf.refcnt = 0;

	rtems_interrupt_disable(key);
		bufs[i].buf.next = frb;
		frb              = bufs;
		lanIpBufAvail   += n;
		lanIpBufTotal   += n;
		/* chain into list of malloced chunks */
		*(void**)mem     = rbuf_mem;
		rbuf_mem         = mem;
	rtems_interrupt_enable(key);

	return 0;	
}

static void
freeBufMem()
{
void *p;
	while ( (p = rbuf_mem) ) {
		rbuf_mem = *(void**)rbuf_mem;
		free(p);
	}
}

#define ISBCST(ip,nm)  (((ip) & ~(nm)) == ~(nm))
#if 1
/* There must be some compiler bug -- this test sometimes wouldn't inline
 * so I defined a special inline routine dedicated to the multicast-test
 * above :-(
 */
#define ISMCST(ip)     (htonlc(0xe0000000) == (htonlc(0xf0000000) & (ip)))
#else
#define ISMCST(ip)     ismcst(ip)
#endif

#define IP_GRP_ALL_SYSTEMS  0xe0000001
#define IP_GRP_ALL_ROUTERS  0xe0000002
	
#define ISMCST_ALLSYS(ip) (htonlc(IP_GRP_ALL_SYSTEMS) == (ip))

/* Convert IP multicast address into ethernet multicast address */
static inline void
ipmc2ethermc(uint32_t ipaddr, uint8_t *enaddr)
{
uint32_t tmp;

	enaddr[0] = 0x01;
	enaddr[1] = 0x00;

	tmp = (ipaddr & htonlc(0x007fffff)) | htonlc(0x5e000000);

	memcpy( enaddr + 2, &tmp , 4);
}

#define MC_FLG_IGMP_LEAVE	(1<<0)

typedef struct IpBscMcAddrRec_ {
	LanIpLstNodeRec mc_node;
	LanIpCalloutRec mc_igmp;
	uint32_t        mc_addr;
	uint16_t		mc_sobs;
	uint16_t        mc_flags;
} IpBscMcAddrRec, *IpBscMcAddr;

/* 'Special' sd to prevent 224.0.0.1 to be ever deleted */
#define MC_ALLSYS_SD	15

#if (NSOCKS) > (MC_ALLSYS_SD)
#error "MC_ALLSYS_SD clashes with normal sockets"
#endif

typedef union IpBscMcRef_ {
	LanIpLstNode	r_node;
	IpBscMcAddr     r_mcaddr;
} IpBscMcRef;;

typedef struct IpBscIfRec_ {
	void			*drv_p;
	rtems_id		mutx;
	uint32_t		ipaddr;
	uint32_t		nmask;
	LanArpRec		arpreq;
	rbuf_t          *arpbuf;
	LHTbl           mctable;
	IpBscMcRef      mclist;
	IpBscMcAddr     mcallsys;
	LanIpCalloutRec mcIgmpV1RtrSeen;
	unsigned        mcnum;	/* number of MC groups we joined */
	IpBscIfStatsRec	stats;
} IpBscIfRec;

#define arprep      arpbuf->pkt.p_u.arp_S

/* Check if 'ipaddr' is a multicast address to which
 * the interface is subscribed.
 */
inline int
mcListener(IpBscIf intrf, uint32_t ipaddr)
{
	return ISMCST( ipaddr ) && lhtblFind( intrf->mctable, ipaddr );
}

static int
addmca(IpBscIf intrf, int sd, uint32_t mcaddr);

static IpBscMcAddr
delmca(IpBscIf intrf, IpBscMcAddr mca, int sd);

static void
destroymca(IpBscMcAddr mca);

static __inline__ IpBscMcAddr
nxtmca (IpBscMcAddr mca)
{
	return (IpBscMcAddr)mca->mc_node.l_next;
}

static void
igmp_v1timeo(void *arg0, void *arg1);

static void
processIcmp(IpBscIf intrf, rbuf_t *buf_p, int len);

static void
processIgmp(IpBscIf intrf, rbuf_t *buf_p, int len);

static void
igmp_timeo(void *arg0, void *arg1);

#define IGMP_TYPE_QUERY     0x11
#define IGMP_TYPE_REPORT_V1 0x12
#define IGMP_TYPE_REPORT_V2 0x16
#define IGMP_TYPE_LEAVE     0x17

static IpBscMcAddr
igmp_send_msg(IpBscIf intrf, uint32_t gaddr, uint8_t type);

static rtems_id
task_spawn(char *name, int pri, int stacksz, void (*fn)(void*), void *arg)
;

#define TASK_JOIN_NPAD	RTEMS_NOTEPAD_0

static void
task_leave();

static void
task_init(rtems_id);

static void
arpTearDown(int perm_also);

#define KILL_BY_EVENT	1
#define KILL_BY_SEMA	2

typedef union task_killer_ {
	rtems_event_set	kill_event;	
	rtems_id		kill_resource;	
} task_killer;

static void
task_pseudojoin(rtems_id tid, int method, task_killer killer);

static rbuf_t * volatile workHead = 0;
static rbuf_t * volatile workTail = 0;
static rtems_id          workSema = 0;
static rtems_id          workTask = 0;

static void       lpWorker(void *arg);

static void
scheduleLpWork(IpBscIf intrf, rbuf_t *buf_p)
{
rtems_interrupt_level l;

	buf_p->buf.intrf = intrf;
	buf_p->buf.next  = 0;

	rtems_interrupt_disable(l);
		if ( workTail ) {
			workTail->buf.next = buf_p;
		} else {
			workHead           = buf_p;
		}
		workTail = buf_p;
	rtems_interrupt_enable(l);

	rtems_semaphore_release( workSema );
}

static rbuf_t *
dequeueLpWork()
{
rtems_status_code     sc;
rtems_interrupt_level l;
rbuf_t                *rval;

	sc = rtems_semaphore_obtain( workSema, RTEMS_WAIT, RTEMS_NO_TIMEOUT );

	if ( RTEMS_SUCCESSFUL != sc )
		return 0;

	rtems_interrupt_disable(l);
		rval = workHead;
		/* We don't have to check 'workHead' for null-ness.
		 * The counting-semaphore ensures that there is
		 * something ...
		 */
		if ( ! (workHead = workHead->buf.next) )
			workTail = 0;
	rtems_interrupt_enable(l);

	/* paranoia */
	rval->buf.next = 0;

	return rval;
}

/* a bad but simple random number generator; should be good
 * enough for IGMP (Posix.1-2001)
 */
static uint32_t
badrand16()
{
static uint32_t       v = 1234567;
uint32_t              n;
rtems_interrupt_level l;

	rtems_interrupt_disable(l);
		n = v * 1103515245 + 12345;
		v = n;
	rtems_interrupt_enable(l);

	return n>>16;
}

#define SEM_SYNC	1
#define SEM_MUTX	2
#define SEM_SMTX	3
#define SEM_CNTG    4

static rtems_id
sem_create(char *name, int type, int count)
;

static rtems_id
bsem_create(char *name, int type)
;


#warning 'debugging code still present'
void *
getlhtbl(IpBscIf intrf)
{
	return intrf ? intrf->mctable : 0;
}

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
	rbuf_t               *pkt;
	int                   len;
} UdpSockMsgRec, *UdpSockMsg;

static __inline__ void
mutex_lock(rtems_id m)
{
int rtems_semaphore_obtain_SUCCESSFUL =
	RTEMS_SUCCESSFUL == rtems_semaphore_obtain(m, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	assert( rtems_semaphore_obtain_SUCCESSFUL );
}

static __inline__ void
mutex_unlk(rtems_id m)
{
int rtems_semaphore_release_SUCCESSFUL =
	RTEMS_SUCCESSFUL == rtems_semaphore_release(m);
	assert( rtems_semaphore_release_SUCCESSFUL );
}
#define SOCKLOCK(sck)		mutex_lock((sck)->mutx)
#define SOCKUNLOCK(sck) 	mutex_unlk((sck)->mutx)

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
	/* union holds tick-time of last request sent
	 * (if reply still outstanding)
	 * or hwaddress if we have a reply.
	 * The 'sync_resp' ID can be used as a discriminator
	 * for the union (hwaddr if sync_resp == 0)
	 */
	union {
		uint8_t			hwaddr[6];
		rtems_interval  stime;
	}               data;
	rtems_interval	ctime;
	rtems_id        sync_resp;
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
static ArpEntry arpcache_raw[257] = {0};
static ArpEntry *arpcache         = arpcache_raw + 1;

#define ARP_CACHESZ  256
#define ARP_SENTINEL (-1)

/* Keep an available entry around */
static ArpEntry	arpScratch    = 0;

int lanIpBscAutoRefreshARP  = 1;

/* don't bother about MSBs; assume we're on a LAN anyways */
#define ARPHASH(h,ipaddr)			\
	do {							\
		uint32_t ipaddr_h_;			\
		ipaddr_h_ = ntohl(ipaddr);	\
		h  = ipaddr_h_;				\
		h += ipaddr_h_>>8;			\
	} while (0)

#define ARPLOCK(pd)		mutex_lock((pd)->mutx)
#define ARPUNLOCK(pd) 	mutex_unlk((pd)->mutx)

#define MCLOCK(pd)		mutex_lock((pd)->mutx)
#define MCUNLOCK(pd) 	mutex_unlk((pd)->mutx)

#define COLOCK()		mutex_lock(callout_mtx)
#define COUNLOCK() 		mutex_unlk(callout_mtx)

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

static void
prip(FILE *f, const uint32_t ipaddr)
{
union {
	uint8_t  c[4];
	uint32_t l;
} buf;
	buf.l = ipaddr;
	fprintf(f, "%u.%u.%u.%u", buf.c[0], buf.c[1], buf.c[2], buf.c[3]);
}

char *
lanIpBscNtop(uint32_t ipaddr, char *abuf, int len)
{
int put;
union {
	uint32_t u;
	uint8_t  b[4];
}   ipa;

	ipa.u = ipaddr;

	put = snprintf(abuf, len, "%u.%u.%u.%u", ipa.b[0], ipa.b[1], ipa.b[2], ipa.b[3]);

	return ( put >= len ) ? 0 : abuf;
}

/* Dump an ARP cache entry */
static void
prarp(FILE *f, ArpEntry e)
{
char           ipbuf[4*4];
rtems_interval now,difft;

	if ( !f )
		f = stdout;

	if (!lanIpBscNtop( e->ipaddr, ipbuf, sizeof(ipbuf))) {
		perror("\nUnable to print ARP entry (lanIpBscNtop failed)");
		return;
	}
	rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &now);
	difft = rtems_clock_get_ticks_since_boot();

	fprintf(f,"%s: ", ipbuf);
	if ( ! e->sync_resp ) {
		prether(f, e->data.hwaddr);
	} else  {
		difft -= e->data.stime;
		fprintf(f,"<request outstanding: %lu ticks>",difft);
	}
		
	fprintf(f," (age: ");
	if ( ARP_PERM == e->ctime )
		fprintf(f,"STATIC)\n");
	else
		fprintf(f,"%lus)\n", now-e->ctime);
}

static ArpEntry
arp_createentry()
{
	return calloc(1, sizeof(ArpEntryRec));
}

static void
arp_destroyentry(ArpEntry a)
{
	/* this is mostly called after executing arp_putscratch (from a 
	 * protected area) so there would be no need to try the semaphore
	 * again -- however, e.g., when flushing the cache this routine
	 * is also called and then deleting the semaphore might be
	 * necessary.
	 */
	if ( a ) {
		if ( a->sync_resp )
			rtems_semaphore_delete(a->sync_resp);
		free( a );
	}
}

static ArpEntry
arp_putscratch(ArpEntry e)
{
	if ( e ) {
		if ( e->sync_resp ) {
			rtems_semaphore_delete(e->sync_resp);
			e->sync_resp = 0;
		}
		if ( !arpScratch ) {
			/* mark as unused */
			e->ipaddr  = 0;
			arpScratch = e;
			e          = 0;
		}
	}
	return e;
}

static ArpEntry
arp_find_or_add(uint8_t h, uint32_t ipaddr, ArpEntry *p_newe)
{
int         i, oh, empty;
ArpEntry    found;

	for ( i=0, oh = ARP_SENTINEL, empty=-1; i<CACHE_OVERLAP; i++, h++ ) {
		if ( ! (found = arpcache[h]) ) {
			if ( empty < 0 )
				empty=h;
			continue;
		}
		if ( ipaddr == found->ipaddr ) {
			return found;
		}
		/* permanent entries are always 'newest' (ctime in the future)
		 * hence the first non-permanent entry will be found
		 * by this algorithm.
		 */
		if ( (uint32_t)found->ctime < (uint32_t)arpcache[oh]->ctime ) {
			oh     = h;
		}
	}

	if ( empty<0 ) {
		found = arpcache[oh];
		/* all slots full; must evict oldest entry;
		 * we also bail if 'oh' still points to ARP_SENTINEL.
		 */
		if ( found->ctime == ARP_PERM ) {
			/* ERROR CONDITION */
			fprintf(stderr,"arpPutEntry/arpLookup: too many permanent entries, unable to allocate slot\n");
			return found;
		}
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("arpPutEntry/arpLookup no more slots, evicting #%i: ", oh);
			prarp(0, arpcache[oh]);
		}
#endif
		if ( found->sync_resp ) {
			rtems_semaphore_delete( found->sync_resp );
			found->sync_resp = 0;
		}
		found->ipaddr = 0;
		h = oh;
	} else {
		h           = empty;
		found       = arpcache[h] = *p_newe;
		*p_newe     = 0;
	}
	return found;
}

static int
arp_find_or_del(IpBscIf pd, uint8_t h, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry found = 0;
ArpEntry rval;
int      i;

	ARPLOCK(pd);
	for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
		if ( !(rval = arpcache[h]) )
			continue;
		if ( ipaddr == rval->ipaddr ) {
			if ( ! rval->sync_resp ) {
#ifdef DEBUG
				if ( (lanIpDebug & DEBUG_ARP) ) {
					printf("arpLookup(): last-chance success for entry #%i\n",h);
					prarp(0, rval);
				}
#endif
				/* Wow - we have something */
				memcpy(enaddr, rval->data.hwaddr, 6);
				ARPUNLOCK(pd);
				return 0;
			}
#ifdef DEBUG
			if ( (lanIpDebug & DEBUG_ARP) ) {
				printf("arpLookup(): deleting placeholder entry #%i\n",h);
				prarp(0, rval);
			}
#endif
			/* Still no reply */
			arpcache[h] = 0;
			found = rval;
			break;
		}
	}
	found = arp_putscratch( found );
	ARPUNLOCK(pd);
	arp_destroyentry(found);
	return -1;
}


static rtems_status_code
ARPUNLOCK_block(IpBscIf pd, rtems_id sync_sem)
{
rtems_status_code sc;
rtems_status_code rtems_task_mode_disable_preemption;
rtems_status_code rtems_task_mode_restore_preemption;
rtems_mode        prev_mode;
const char       *dbgstr;

	/* We must atomically release the ARPLOCK and then acquire
	 * the sync_resp remaphore.
	 * This can e.g., be achieved by making this task (temporarily)
	 * non-preemptible.
	 */
	rtems_task_mode_disable_preemption =
		rtems_task_mode( RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode );
	assert ( RTEMS_SUCCESSFUL == rtems_task_mode_disable_preemption );

	ARPUNLOCK(pd);
	/* Clock rate is some 10s of Hz; if answer is not back in several milli-
	 * seconds then we can timeout (i.e., 1 clock tick is enough)
	 */
	sc = rtems_semaphore_obtain( sync_sem, RTEMS_WAIT, ARP_TIMEOUT_TICKS );

	/* Restore old preemption mode */
	rtems_task_mode_restore_preemption =
		rtems_task_mode( prev_mode, RTEMS_PREEMPT_MASK, &prev_mode );
	assert ( RTEMS_SUCCESSFUL == rtems_task_mode_restore_preemption );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_ARP ) {
		switch ( sc ) {
			default:
				rtems_error(sc, "ARPUNLOCK_block(): unexpected return code\n");
				dbgstr = "<UNKNOWN>";
				break;
			case RTEMS_TIMEOUT:
				dbgstr = "TIMEOUT";
				break;
			case RTEMS_UNSATISFIED:
				dbgstr = "REPLY RECEIVED";
				break;
			case RTEMS_OBJECT_WAS_DELETED:
				dbgstr = "ENTRY WAS REMOVED";
				break;
		}
		printf("ARPUNLOCK_block() - finished waiting (%s)\n", dbgstr);
	}
#endif
	return sc;
}

#define ARPSEM_UNNEEDED	0
#define ARPSEM_FRESH    1
#define ARPSEM_OLD      2  

static int
arpCreateSyncsem(IpBscIf pd, uint8_t h, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry          rval;
ArpEntry          newe;
int               i, err;
rtems_interval    now;

		/* This routine is called from 'arpLookup' where the
		 * validity of 'ipaddr' has been established already.
		 */
#if 0
		/* Silently ignore broadcast + multicast addresses */
		if ( ISBCST( ipaddr, pd->nmask ) || ISMCST( ipaddr ) )
			return SOME_ERROR_STATUS;
#endif

		ARPHASH(h, ipaddr);

		ARPLOCK(pd);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pd);
			newe = arp_createentry();
			ARPLOCK(pd);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(h, ipaddr, &newe);

		if ( ipaddr == rval->ipaddr ) {
			if ( rval->sync_resp ) {
				/* Have it already */
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("arpCreateSyncsem(): sem already exists for entry #%i\n",h);
					prarp(0, rval);
				}
#endif
				now = rtems_clock_get_ticks_since_boot();
				if ( now - rval->data.stime < ARP_TIMEOUT_TICKS) {
					/* another thread has recently set 'ctime' and probably
					 * sent a retry-request.
					 */
					err = ARPSEM_FRESH;
				} else {
					/* too much time elapsed since the last time we
					 * returned ARPSEM_OLD (and thus caused a ARP request
					 * to be sent).
					 */
					err = ARPSEM_OLD;
					rval->data.stime = now;
				}
			} else {
				/* Already have an answer */
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("arpCreateSyncsem(): already got an answer for entry #%i\n", h);
					prarp(0, rval);
				}
#endif
				memcpy(enaddr, rval->data.hwaddr, 6);
				err = ARPSEM_UNNEEDED;
			}
		} else {
			/* If there is an address-mismatch then 'rval' must
			 * point to a new, virgin entry. If 'rval->ipaddr'
			 * is not NULL then this must be due to the error
			 * condition in 'arp_find_or_add()'.
			 */
			if ( rval->ipaddr ) {
				fprintf(stderr,"arpCreateSyncsem(): too many permanent entries, unable to allocate slot\n");
				err = -ENOSPC;
				goto egress;
			}
			rval->sync_resp = bsem_create("arps", SEM_SYNC);
			if ( !rval->sync_resp ) {
				fprintf(stderr,"arpCreateSyncsem(): unable to create semaphore; delaying\n");
				/* must find hash and remove from table */
				assert( newe == 0 );
				for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
					if ( !(rval = arpcache[h]) )
						continue;
					if ( ipaddr == rval->ipaddr ) {
						newe        = arpcache[h];
						arpcache[h] = 0;
					}
				}
				assert( newe != 0 );
				err = -ENOMEM;
				goto egress;
			}
			rval->ipaddr     = ipaddr;
			rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
			rval->data.stime = rtems_clock_get_ticks_since_boot();

#ifdef DEBUG
			if ( lanIpDebug & DEBUG_ARP ) {
				printf("arpCreateSyncsem(): created new sem for entry #%i: ", h);
				prarp(0, rval);
			}
#endif
			err = ARPSEM_OLD;
		}

egress:
		newe = arp_putscratch(newe);
		ARPUNLOCK(pd);
		arp_destroyentry(newe);

		return err;
}

static rtems_status_code
arpAwaitReply(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry          rval;
ArpEntry          newe;
uint8_t           h;
int               i;
#ifdef DEBUG
const char       *dbgstr = 0;
#endif
rtems_status_code sc;

		/* This routine is called from 'arpLookup' where the
		 * validity of 'ipaddr' has been established already.
		 */
#if 0
		/* Silently ignore broadcast + multicast addresses */
		if ( ISBCST( ipaddr, pd->nmask ) || ISMCST( ipaddr ) )
			return SOME_ERROR_STATUS;
#endif

		ARPHASH(h, ipaddr);

		ARPLOCK(pd);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pd);
			newe = arp_createentry();
			ARPLOCK(pd);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(h, ipaddr, &newe);

		if ( ipaddr == rval->ipaddr ) {
			if ( rval->sync_resp ) {
				/* Block for reply        */
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					dbgstr = "cache hit for";
				}
#endif
			} else {
				/* Already have an answer */
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("arpAwaitReply(): already got an answer for entry #%i\n", h);
					prarp(0, rval);
				}
#endif
				memcpy(enaddr, rval->data.hwaddr, 6);
				sc = RTEMS_SUCCESSFUL;
				goto egress;
			}
		} else {
			/* If there is an address-mismatch then 'rval' must
			 * point to a new, virgin entry. If 'rval->ipaddr'
			 * is not NULL then this must be due to the error
			 * condition in 'arp_find_or_add()'.
			 */
			if ( rval->ipaddr ) {
				fprintf(stderr,"arpAwaitReply: too many permanent entries, unable to allocate slot\n");
				sc = RTEMS_NO_MEMORY;
				goto egress;
			}
			rval->sync_resp = bsem_create("arps", SEM_SYNC);
			if ( !rval->sync_resp ) {
				fprintf(stderr,"arpAwaitReply: unable to create semaphore; not waiting\n");
				/* must find hash and remove from table */
				assert( newe == 0 );
				for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
					if ( !(rval = arpcache[h]) )
						continue;
					if ( ipaddr == rval->ipaddr ) {
						newe        = arpcache[h];
						arpcache[h] = 0;
					}
				}
				assert( newe != 0 );
				sc = RTEMS_TOO_MANY;
				goto egress;
			}
			rval->ipaddr    = ipaddr;
			rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
		}

#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			if ( ! dbgstr )
				dbgstr = "created new";
			printf("arpAwaitReply %s entry #%i: ", dbgstr, h);
			prarp(0, rval);
			printf("Blocking for reply...\n");
		}
#endif

		/* cleanup if necessary */
		newe = arp_putscratch(newe);

		ARPUNLOCK_block(pd, rval->sync_resp);

		arp_destroyentry(newe);

		/* MUST be timedout, unsatisfied (due to flush operation) or
		 * object_was_deleted.
		 */
		assert ( RTEMS_SUCCESSFUL != sc );

		return sc;

egress:
		newe = arp_putscratch(newe);
		ARPUNLOCK(pd);
		arp_destroyentry(newe);

		return sc;
}

int
arpLookup(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int cacheonly)
{
ArpEntry          rval;
int               i,st;
uint8_t           hh;
uint8_t           h;
rtems_status_code sc;
int               attempts;
int               err = -ENOTCONN;

		if ( ISBCST(ipaddr, pd->nmask) ) {
			if ( enaddr )
				memset(enaddr, 0xff, 6);
			return 0;
		}

		if ( ISMCST(ipaddr) ) {
			if ( enaddr ) {
				ipmc2ethermc(ipaddr, enaddr);
			}
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

		attempts = 0;

		/* We may probe the cache one more time than we
		 * send requests so that we may find a reply
		 * to the last send attempt.
		 */
		while ( attempts < ARP_SEND_RETRY + 1 ) {

			ARPLOCK(pd);
			for ( i = 0, hh=h; i<CACHE_OVERLAP; i++, hh++ ) {

				if ( !(rval = arpcache[hh]) )
					continue;

				if ( ipaddr == rval->ipaddr ) {
					if ( ! rval->sync_resp ) {
						/* Done, found valid cache entry */
						memcpy(enaddr, rval->data.hwaddr, 6);
						ARPUNLOCK(pd);
						return 0;
					} else {
						if ( cacheonly ) {
							ARPUNLOCK(pd);
							return -ENOTCONN;
						}

						/* block for reply */
						sc = ARPUNLOCK_block(pd, rval->sync_resp);

						/* 'normal' answer if we have received a reply */
						if ( RTEMS_UNSATISFIED == sc ) {
							/* 'continue' outer for loop */
							goto try_again;
						}

						/* are already unlocked; 'break' and skip unlock operation */
						goto proceed_unlocked;
					}
				}
			}
			ARPUNLOCK(pd);

			if ( cacheonly )
				return -ENOTCONN;

proceed_unlocked:

			if ( attempts >= ARP_SEND_RETRY )
				break;

			st = arpCreateSyncsem(pd, h, ipaddr, enaddr);

			if        ( ARPSEM_OLD == st ) {
				/* must do a new lookup */
				NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr);
			} else if ( ARPSEM_FRESH == st ) {
				/* just try again and block */
			} else if ( ARPSEM_UNNEEDED == st ) {
				/* already got an answer */
				return 0;
			} else if ( -ENOMEM == st ) {
				/* could not create semaphore; work-around by just
				 * delaying this task
				 */
				rtems_task_wake_after(ARP_TIMEOUT_TICKS);
				pd->stats.arp_nosem++;
			} else {
				assert( st < 0 );
				err = st;
				break;
			}

try_again:
			attempts++; /* statement here to silence compiler warning */
		}

		/* We only get here if we have tried to use the sync-semaphore.
		 *
		 * Check the cache one more time and remove the dummy entry
		 * that is only used for synchronization.
		 * 
		 * We could succeed here if the last send/block operation
		 * resulted in the cache being updated.
		 */
		if ( 0 == arp_find_or_del(pd, h, ipaddr, enaddr) )
			return 0;
		
		return err;
}

int
arpPutEntry(IpBscIf pd, uint32_t ipaddr, uint8_t *enaddr, int perm)
{
ArpEntry rval;
ArpEntry newe;
uint8_t  h;
int      err = -ENOSPC;
#ifdef DEBUG
const char *dbgstr = 0;
#endif

		/* Silently ignore broadcast + multicast addresses */
		if ( ISBCST( ipaddr, pd->nmask ) || ISMCST( ipaddr ) )
			return 0;

		ARPHASH(h, ipaddr);

		ARPLOCK(pd);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pd);
			newe = arp_createentry();
			ARPLOCK(pd);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(h, ipaddr, &newe);

		if ( ipaddr == rval->ipaddr ) {
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_ARP ) {
				dbgstr = "cache hit, refreshing";
			}
#endif

			/* Notify waiting tasks */
			if ( rval->sync_resp ) {
				rtems_semaphore_flush( rval->sync_resp );
				rtems_semaphore_delete( rval->sync_resp );
				rval->sync_resp = 0;
			}

			/* Done. Refresh entry and leave. */
		} else {
			/* If there is an address-mismatch then 'rval' must
			 * point to a new, virgin entry. If 'rval->ipaddr'
			 * is not NULL then this must be due to the error
			 * condition in 'arp_find_or_add()'.
			 */
			if ( rval->ipaddr ) {
				fprintf(stderr,"arpPutEntry: too many permanent entries, unable to allocate slot\n");
				err = -ENOSPC;
				goto egress;
			}
			rval->ipaddr = ipaddr;

			assert( !rval->sync_resp );
		}

		memcpy(rval->data.hwaddr, enaddr, 6);

		if ( perm > 0 ) {
			rval->ctime = ARP_PERM;
		} else {
			/* arg 'perm' == -1 revokes 'permanent' flag */
			if ( -1 == perm || ARP_PERM != rval->ctime )
				rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &rval->ctime);
		}

#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			if ( ! dbgstr )
				dbgstr = "writing";
			printf("arpPutEntry %s entry #%i: ", dbgstr, h);
			prarp(0, rval);
		}
#endif

		err = 0;

egress:
		newe = arp_putscratch( newe );
		ARPUNLOCK(pd);
		arp_destroyentry(newe);

		return err;
}

void
arpDelEntry(IpBscIf pd, uint32_t ipaddr)
{
ArpEntry rval, found = 0;
int      i;
uint8_t  h;

		ARPHASH(h, ipaddr);

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

		found = arp_putscratch( found );
		ARPUNLOCK(pd);
		arp_destroyentry(found);
}

/* Unprotected version to be used when cleaning up after
 * the interface has been destroyed
 */
static void
arpTearDown(int perm_also)
{
int i;

	for ( i=0; i<ARP_CACHESZ; i++ ) {
		if ( perm_also || (arpcache[i] && ARP_PERM != arpcache[i]->ctime) ) {
			arp_destroyentry(arpcache[i]);
			arpcache[i] = 0;
		}
	}

	arp_destroyentry(arpScratch);
	arpScratch = 0;
}

void
arpFlushCache(IpBscIf pd, int perm_also)
{
	/* cleanup everything */

	ARPLOCK(pd);

	arpTearDown(perm_also);

	ARPUNLOCK(pd);
}

void
arpDumpCache(IpBscIf pd, FILE *f)
{
ArpEntryRec           abuf;
int                   i;

	if ( !f )
		f = stdout;

	for ( i=0; i<ARP_CACHESZ; i++ ) {
		if ( (volatile ArpEntry) arpcache[i] ) {

			ARPLOCK( pd );

				/* it might have gone... */
				if ( (volatile ArpEntry) arpcache[i] ) {
					abuf = *arpcache[i];

					ARPUNLOCK( pd );

					fprintf(f,"ARP cache entry #%i: ",i);
					prarp(f, &abuf);
				} else {
					ARPUNLOCK( pd );
				}
		}
	}
}

void
arpScavenger(IpBscIf pd, rtems_interval maxage, rtems_interval period, int nloops)
{
rtems_interval        ancient,sec,now;
int                   i;
ArpEntry              e;

	while ( 1 ) {
		/* calculate oldest acceptable 'ctime' */
		rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &now);
		ancient = now - maxage;

		for ( i=0; i<ARP_CACHESZ; i++ ) {
			if ( ! (volatile ArpEntry) arpcache[i] ) 
				continue;

			ARPLOCK( pd );
			/* have to check again from within protected section */
			if ( !(e = (volatile ArpEntry)arpcache[i]) ) {
				ARPUNLOCK( pd );
				continue;
			}
			if ( (uint32_t)e->ctime < (uint32_t)ancient ) {
				/* evict */
				arpcache[i] = 0;
				e = arp_putscratch(e);

#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
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
				}
#endif
				ARPUNLOCK(pd);

				arp_destroyentry(e);

			} else {
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("ARP cache entry #%i: ",i);
					if (arpcache[i])
						prarp(0, arpcache[i]);
					else
						printf("\n");
				}
#endif
				ARPUNLOCK(pd);
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
	buf_p->ip.csum             = in_cksum_hdr((void*)&buf_p->ip);
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
IpArpRec	*pipa = &lpkt_arp(&p->pkt);
uint32_t    xx;


	 /* 0x0001 == Ethernet, 0x0800 == IP */
	NETDRV_READ_INCREMENTAL(pd, pipa, 8);
	if ( htonlc(0x00010800) != *(uint32_a_t*)pipa )
		return 8;

	xx = * ( (uint32_a_t *) pipa + 1 );

	/* 0x06 hw addr len, 0x04 proto len, 0x0001 ARP REQUEST */
	if        ( htonlc(0x06040001) == xx ) {
		isreq = 1;
	/* 0x06 hw addr len, 0x04 proto len, 0x0002 ARP REPLY   */
	} else if ( htonlc(0x06040002) != xx ) {
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

		refrbuf( pd->arpbuf );
		NETDRV_ENQ_BUFFER(pd, pd->arpbuf, sizeof(pd->arprep));
	} else {
		/* a reply to our request */
		scheduleLpWork(pd, p);
		*ppbuf = 0; 
	}

	return sizeof(*pipa);
}

static int
check_vhl(uint8_t vhl, uint8_t expected)
{
	if ( vhl != expected ) {
#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) )
			printf("dropping IP packet, vhl: 0x%02x (!= 0x%02x)\n", vhl, expected);
#endif
		return -1;
	}
	return 0;
}

#define IP_PROT_ICMP		1
#define IP_PROT_IGMP		2
#define IP_PROT_UDP			17

#define IP_OPT_ROUTER_ALERT	0x94040000

static int
handleIP(rbuf_t **ppbuf, IpBscIf intrf, int loopback)
{
int          rval = 0, l, nbytes, i;
rbuf_t		 *p = *ppbuf;
IpHeaderRec  *pip = &lpkt_ip(&p->pkt);
uint16_t	 dport;
int			 isbcst = 0;
int          ismcst = 0;
LanUdpHeader hdr;

	if ( ! loopback )
		NETDRV_READ_INCREMENTAL(intrf, pip, sizeof(*pip));
	rval += sizeof(*pip);

	/* accept IP unicast and broadcast */
	if (   ! (pip->dst == intrf->ipaddr)
		&& ! (ismcst = mcListener(intrf, pip->dst))
        && ! (isbcst = ISBCST(pip->dst, intrf->nmask))
	   )
		return rval;

#ifdef DEBUG
	if ( (lanIpDebug & DEBUG_IP) ) {
		if ( pip->dst == intrf->ipaddr ) {
			printf("accepting IP unicast, proto %i\n", pip->prot);
		} else if ( ismcst ) {
			printf("accepting IP multicast (");
			prip(stdout, pip->dst);
			printf(") , proto %i\n", pip->prot);
		}
	}
#endif

	/* reject fragmented packets, i.e., headers with MF
     * (more fragments) or an offset.
	 */
	if ( ntohs(pip->off) & 0x9fff ) {
#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) )
			printf("dropping IP packet, vhl: 0x%02x flgs/off 0x%04x\n",
                   pip->vhl, ntohs(pip->off));
#endif
		return rval;
	}

	nbytes = ntohs(pip->len);

	/* Check length and reject packets that wouldn't fit
	 * in and 'rbuf_t'
	 */
	if ( nbytes > sizeof( p->pkt ) - sizeof(EthHeaderRec) ) {
#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) )
			printf("dropping IP packet, len: 0x%02x too big for rbuf_t\n",
                   nbytes);
#endif
		return rval;
	}

	l = ((nbytes - sizeof(*pip)) + 3) & ~3;

	switch ( pip->prot ) {
		case IP_PROT_ICMP :
		{
		IcmpHeaderRec *picmp = &lpkt_icmp(&p->pkt);

			/* reject non-V4 headers or headers with length > 5 */
			if ( check_vhl(pip->vhl, 0x45) )
				return rval;

			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(intrf, picmp, l);
			rval += l;
			scheduleLpWork(intrf, p);
			/* worker thread took over the buffer */
			*ppbuf = 0;
		}
		break;

		case IP_PROT_IGMP :
		{

			/* Hmm - the RFC says that all IGMP messages must have
			 * the IP Router-Alert Option set but my linksys SLM2008
			 * does no such thing :-(
			 * Let's therefore be more tolerant and accept standard
			 * headers, too.
			 */

			/* reject non-V4 headers or headers with length > 6 */
			if ( pip->vhl != 0x46 && pip->vhl != 0x45 ) {
#ifdef DEBUG
				if ( (lanIpDebug & DEBUG_IP) )
					printf("dropping IP packet, vhl: 0x%02x (not 0x45 nor 0x46)\n", pip->vhl);
#endif
				return rval;
			}

			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(intrf, &lpkt_igmpv2hdr( &p->pkt ).igmp_u, l);
			rval += l;

			scheduleLpWork(intrf, p);
			/* worker thread took over the buffer */
			*ppbuf = 0;

		}
		break;

		case IP_PROT_UDP  :
		{
		LanUdpHeader pudp = &lpkt_udphdr(&p->pkt);

			/* reject non-V4 headers or headers with length > 5 */
			if ( check_vhl(pip->vhl, 0x45) )
				return rval;

			/* UDP header is word aligned -> OK */
			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(intrf, &pudp->udp, sizeof(pudp->udp));
			rval += sizeof(pudp->udp);
			l    -= sizeof(pudp->udp);
			dport = ntohs(pudp->udp.dport);
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_UDP ) {
				char buf[4*4];
				printf("handling UDP packet (dport %i%s)\n", dport, isbcst ? ", BCST":"");
				lanIpBscNtop(pudp->hdr.ip.src, buf, sizeof(buf));
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
					if ( ! loopback )
						NETDRV_READ_INCREMENTAL(intrf, pudp->pld, l);
					rval += l;

					/* Refresh peer's ARP entry */
					if ( lanIpBscAutoRefreshARP ) {
						arpPutEntry(intrf, pudp->hdr.ip.src, pudp->hdr.ll.src, 0);
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
EthHeaderRec   *pll = &lpkt_eth(&prb->pkt);
uint16_t        tt;

	NETDRV_READ_INCREMENTAL(pd, prb, sizeof(*pll));
	len -= sizeof(*pll);

	tt = pll->type;
	if ( htonsc(0x806) == tt ) {
		/* ARP */
		len -= handleArp(pprb, pd);
	} else if ( htonsc(0x800) == tt ) {
		/* IP  */
		len -= handleIP(pprb, pd, 0 /* not loopback */);
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

	if ( ! (rval->arpbuf = getrbuf()) )
		goto bail;


	if ( ! (rval->mutx = bsem_create("ipmx", SEM_SMTX)) ) {
		fprintf(stderr, "lanIpCb: unable to create mutex\n");
		goto bail;
	}

	if ( ! ( rval->mctable = lhtblCreate(
								1000,
								(unsigned long) (&((IpBscMcAddr)0)->mc_addr)
								 - (unsigned long)(uint32_t*)0,
								0) ) ) {
		fprintf(stderr,"Unable to create multicast hash table\n");
		goto bail;
	}

	return rval;

bail:
	intrf = 0;

	if ( rval->mctable )
		lhtblDestroy(rval->mctable, 0, 0);

	if ( rval->arpbuf )
		relrbuf(rval->arpbuf);

	if ( rval->mutx )
		rtems_semaphore_delete( rval->mutx );

	free(rval);

	return 0;
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
		ipbif_p->arpreq.ll.type = htonsc(0x806);
	/* REPLY   */
		/* DST: ??? filled by daemon     */

		/* SRC: drv_p's ethernet address  */
		memcpy(ipbif_p->arprep.ll.src, enaddr, sizeof(*enaddr));
		/* TYPE/LEN is ARP (0x806)       */
		ipbif_p->arprep.ll.type = htonsc(0x806);

	/* ARP PORTION */
	/* HW and PROTO type for both */
	ipbif_p->arprep.arp.htype = ipbif_p->arpreq.arp.htype = htonsc(1);     /* Ethernet */
	ipbif_p->arprep.arp.ptype = ipbif_p->arpreq.arp.ptype = htonsc(0x800); /* IP       */
	ipbif_p->arprep.arp.hlen  = ipbif_p->arpreq.arp.hlen  = 6; 
	ipbif_p->arprep.arp.plen  = ipbif_p->arpreq.arp.plen  = 4; 

	ipbif_p->arprep.arp.oper  = htonsc(2); /* ARP REPLY   */
	ipbif_p->arpreq.arp.oper  = htonsc(1); /* ARP REQUEST */

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

	ipbif_p->mclist.r_node = 0;

	lanIpCallout_init( &ipbif_p->mcIgmpV1RtrSeen );

	if ( lanIpBscDrvStart(ipbif_p, 0) ) {
		fprintf(stderr,"Unable to start driver\n");
		lanIpBscIfDestroy(ipbif_p);
		return 0;
	}

	if ( addmca(ipbif_p, MC_ALLSYS_SD, htonlc(IP_GRP_ALL_SYSTEMS)) ) {
		fprintf(stderr,"Unable to add 224.0.0.1 MC group\n");
		lanIpBscIfDestroy(ipbif_p);
		return 0;
	}

	/* The entry we just added always remains at the end of the list
	 * and serves as a 'tail' marker...
	 */
	ipbif_p->mcallsys = ipbif_p->mclist.r_mcaddr;

	return ipbif_p;
}

void *
lanIpBscIfGetDrv(IpBscIf ipbif_p)
{
	return ipbif_p->drv_p;
}

int
lanIpBscIfDestroy(IpBscIf pd)
{
	if ( nsocks ) {
		fprintf(stderr,"Cannot take down interface -- some sockets still in use\n");
		return -1;
	}

	if ( pd ) {
		assert( nsocks == 0 );
		assert( intrf == pd );

		if ( pd->mclist.r_mcaddr ) {
			/* This address doesn't participate in IGMP - hence
			 * there is no possibility for the lanIpCallout_trystop()
			 * operation to fail (return -1). Thus we don't have
			 * to synchronize with the callout task.
			 */
			delmca( pd, pd->mclist.r_mcaddr, MC_ALLSYS_SD );
		}

		assert( pd->mclist.r_node == 0 );

		/* We don't hold MLOCK at this point so it is
		 * safe to use the synchronous lanIpCalout_stop()
		 * here.
		 */
		lanIpCallout_stop( &pd->mcIgmpV1RtrSeen );

		if ( pd->drv_p ) {
			if ( lanIpBscDrvShutdown(pd->drv_p) ) {
				fprintf(stderr,"lanIpBscIfDestroy(): Unable to shutdown driver\n");
				return -1;
			}
		}

		if ( pd->arpbuf )
			relrbuf( pd->arpbuf );

		if ( pd->mctable )
			lhtblDestroy( pd->mctable, 0, 0 );

		intrf = 0;
		arpFlushCache(pd,1);
		rtems_semaphore_delete(pd->mutx);
		free(pd);
	}
	return 0;
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

	p->udp.len      = htons(payload_len + sizeof(UdpHeaderRec));
}

int
udpSockHdrsInit(int sd, LanUdpHeader p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id)
{
int rval = 0;

	if ( dipaddr ) 
		rval = arpLookup(socks[sd].intrf, dipaddr, p->hdr.ll.dst, 0);
	else /* they want to leave it blank */
		memset(p->hdr.ll.dst,0,6);

	p->hdr.ll.type  = htonsc(0x0800);	/* IP */

	p->hdr.ip.vhl   = 0x45;	/* version 4, 5words length */
	p->hdr.ip.tos   = 0x30; 	/* priority, minimize delay */
	p->hdr.ip.len   = 0;
	p->hdr.ip.id    = htons(ip_id);/* ? */
	p->hdr.ip.off   = htonsc(0);
	p->hdr.ip.ttl   = 4;
	p->hdr.ip.prot  = IP_PROT_UDP;	/* UDP */
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
uint16_t  port = ntohs(p->udp.dport);
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

	if ( ! (m = bsem_create( "udpl", SEM_SMTX )) ) {
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
IpBscMcAddr    mca, mcan;
IpBscMcRef     junk;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( ! socks[sd].port )
		return -EBADF;

	/* unsubscribe from all MC addresses; unfortunately, we
	 * have to scan the interface's list...
	 */
	junk.r_node = 0;

	if ( socks[sd].intrf ) {
		MCLOCK( socks[sd].intrf );
			for ( mca = socks[sd].intrf->mclist.r_mcaddr;
				  mca;
                  mca = mcan ) {

				mcan = nxtmca(mca);

				/* If this is the last socket subscribed then
				 * unsubscribe, remove from hash table etc...
				 */
				if ( (mca->mc_sobs & (1<<sd)) && (mca = delmca(socks[sd].intrf, mca, sd)) ) {
					/* enqueue to junk yard.     */
					c_enq(&junk.r_node, &mca->mc_node);
				}
			}
		MCUNLOCK( socks[sd].intrf );
	}

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

	/* Clean up junkyard from unprotected section */

	for ( mca = junk.r_mcaddr; mca; mca = mcan ) {
		mcan = nxtmca(mca);
		c_deq(&mca->mc_node);
		destroymca(mca);
	}

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
	return &msg.pkt->pkt;
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

#ifdef ENABLE_PROFILE
uint32_t maxes[20]={0};

#define PRFDECL uint32_t now,then
#define dodiff(i) do { uint32_t diff__ ; \
	now = Read_hwtimer();  \
	diff__ = now - then;   \
	if ( diff__ > maxes[i] ) {  \
		maxes[i]=diff__;     \
    }                       \
    then = now;            \
	} while (0)
#define setbase() do { then = Read_hwtimer(); } while (0)

#else

#define PRFDECL
#define dodiff(i)  do { } while(0)
#define setbase(i) do { } while(0)

#endif

static int
udpSockSendTo_internal(int sd, LanIpPacket buf_p, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport)
{
int          rval;
LanUdpHeader h;
int          do_mc_loopback = 0;
PRFDECL;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	setbase();

	SOCKLOCK( &socks[sd] );

	dodiff(1);

	/* If they supply a buffer then we must copy the socket's header
	 * there.
	 */
	if ( buf_p ) {
		h = & lpkt_udphdr( buf_p );
		memcpy( h, &socks[sd].hdr, sizeof(*h) );
	} else {
		h = &socks[sd].hdr;
	}

	dodiff(2);

	if ( ! ipaddr ) {
		/* If they didn't supply a destination address the socket must be connected */
		if ( ! (FLG_ISCONN & socks[sd].flags) ) {
			SOCKUNLOCK( &socks[sd] );
			return -ENOTCONN;
		}
		ipaddr = h->hdr.ip.dst;
	} else {
		/* if the socket is already connected only allow sending to peer */
		if ( (FLG_ISCONN & socks[sd].flags) ) {
			if (   h->hdr.ip.dst != ipaddr
				|| (unsigned short)ntohs( h->udp.dport ) != dport ) {

				rval = -EISCONN;

				SOCKUNLOCK( &socks[sd] );

				return rval;
			}
		} else {
			h->hdr.ip.dst = ipaddr;
			h->udp.dport  = htons((unsigned short)dport);
		}
	}

	dodiff(3);

	if ( ! (FLG_ISCONN & socks[sd].flags) ) {
		uint8_t dummy[6];
		int     cacheonly;

		/* Doing a ARP lookup here prevents another task
		 * sending on the same socket to a different destination
		 * (if we keep the socket locked until the
		 * ARP reply arrives).
		 * For a connected socket that probably doesn't 
		 * matter because nobody can send until we have
		 * an ARP reply anyways.
		 *
		 * Check the cache; if not found then unlock the socket,
		 * do a slow lookup and start over
		 */
		if ( arpLookup(socks[sd].intrf, h->hdr.ip.dst, h->hdr.ll.dst, 1) ) {
			SOCKUNLOCK( &socks[sd] );

			if ( arpLookup(socks[sd].intrf, ipaddr, dummy, 0) ) {
				return -ENOTCONN;
			}

			/* Here we should start over again since things in the socket
			 * could have changed...
			 * However, the only relevant change I can think of is that
			 * someone connected the socked while it was unlocked. In that
			 * case we pick up the new destination address and should
			 * be fine.
			 */

			SOCKLOCK( &socks[sd] );

			cacheonly = ! (FLG_ISCONN & socks[sd].flags);

			if ( arpLookup(socks[sd].intrf, h->hdr.ip.dst, h->hdr.ll.dst, cacheonly) ) {
				SOCKUNLOCK( &socks[sd] );
				return -ENOTCONN;
			}
		}
	} else {
		if ( arpLookup(socks[sd].intrf, h->hdr.ip.dst, h->hdr.ll.dst, 0) ) {
			SOCKUNLOCK( &socks[sd] );
			return -ENOTCONN;
		}
	}

	dodiff(4);

	udpSockHdrsSetlen(h, payload_len);

	dodiff(5);

	do_mc_loopback = mcListener( socks[sd].intrf, h->hdr.ip.dst );

	if ( ! buf_p ) {
#ifdef NETDRV_SND_PACKET
		rval = NETDRV_SND_PACKET( socks[sd].intrf, h, sizeof(*h), payload, payload_len );

		
		if ( do_mc_loopback )
#endif
		{
			if ( ! (buf_p = (LanIpPacket)getrbuf()) ) {
				SOCKUNLOCK( &socks[sd] );
				return -ENOBUFS;
			}
			memcpy( &lpkt_udphdr( buf_p ), h, sizeof(*h) );
			memcpy(  lpkt_udphdr( buf_p ).pld,  payload, payload_len );
		}
	}
#ifdef NETDRV_SND_PACKET
	else {
		rval = payload_len;
	}
#endif

	dodiff(6);

#ifndef NETDRV_SND_PACKET
	if ( do_mc_loopback )
		refrbuf( (rbuf_t*) buf_p );
	rval = payload_len;
	NETDRV_ENQ_BUFFER( socks[sd].intrf, (rbuf_t*)buf_p, payload_len + sizeof(*h) );
#endif

	dodiff(7);

	/* loop back locally subscribed multicast ? */
	if ( do_mc_loopback ) {
		handleIP( (rbuf_t**)&buf_p, intrf, 1 /* loopback */ );
		if ( buf_p )
			relrbuf( (rbuf_t *)buf_p );
	}

	dodiff(8);

	SOCKUNLOCK( &socks[sd] );
	
	dodiff(9);

	return rval;
}

int
udpSockSend(int sd, void *payload, int payload_len)
{
	return udpSockSendTo_internal(sd, 0, payload, payload_len, 0, 0);
}


int
udpSockSendTo(int sd, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport)
{
	return udpSockSendTo_internal(sd, 0, payload, payload_len, ipaddr, dport);
}

int
udpSockSendBuf(int sd, LanIpPacket b, int payload_len)
{
	return udpSockSendTo_internal(sd, b, 0, payload_len, 0, 0);
}

int
udpSockSendBufTo(int sd, LanIpPacket b, int payload_len, uint32_t ipaddr, uint16_t dport)
{
	return udpSockSendTo_internal(sd, b, 0, payload_len, ipaddr, dport);
}


void
udpSockFreeBuf(LanIpPacketRec *b)
{
	relrbuf((rbuf_t*)b);
}

LanIpPacketRec *
udpSockGetBuf()
{
	return &getrbuf()->pkt;
}

IpBscIf
udpSockGetIf(int sd)
{
	if ( sd < 0 || sd >= NSOCKS )
		return 0;

	if ( 0 == socks[sd].port )
		return 0;

	return socks[sd].intrf;
}

int
lanIpBscSendBufRaw(IpBscIf intrf, LanIpPacket buf_p, int len)
{
	/* TODO ???: handle MC loopback */
	NETDRV_ENQ_BUFFER(intrf, (rbuf_t*)buf_p,  len);
	return len;
}

int
lanIpBscSendBufRawIp(IpBscIf intrf, LanIpPacket buf_p)
{
int	len ;
int do_mc_loopback;

	do_mc_loopback = mcListener( intrf, lpkt_ip(buf_p).dst );

	if ( do_mc_loopback )
		refrbuf( (rbuf_t *)buf_p );

	len = ntohs(lpkt_ip(buf_p).len) + sizeof(EthHeaderRec);
	NETDRV_ENQ_BUFFER(intrf, (rbuf_t*)buf_p,  len);

	/* loop back locally subscribed multicast */
	if ( do_mc_loopback ) {
		handleIP( (rbuf_t**)&buf_p, intrf, 1 /* loopback */ );
		if ( buf_p )
			relrbuf( (rbuf_t *)buf_p );
	}
	return len;
}

/* Implementation modelled after
 *
 * "Redesign the BSD Callout and Timer Facilities"
 * Adam M. Costello, George Varghese
 * Dpt. of Computer Science, Washington University, 1995.
 */

#define WHEELBITS 5

#define WHEELMASK ((1<<(WHEELBITS))-1)

#define CALLOUT_EVENT		RTEMS_EVENT_1
#define CALLOUT_KILL_EVENT	RTEMS_EVENT_2

typedef uint32_t               callout_time_t;

static volatile callout_time_t hard_ticks = 0;
static volatile callout_time_t soft_ticks = 0;

static LanIpCalloutRef         c_wheel[1<<WHEELBITS] = {{0}};

unsigned                       lanIpCalloutsScheduled = 0;

static rtems_id                callout_tid = 0;
static rtems_id                callout_mtx = 0;


static inline void
c_enq(LanIpLstNode *where, LanIpLstNode n)
{
	assert( n->l_pprev == 0 && n->l_next == 0 );
	if ( (n->l_next = *where) )
		(*where)->l_pprev = &n->l_next;
	n->l_pprev = where;
	*where     = n;
}

static inline void
c_deq(LanIpLstNode n)
{
LanIpLstNode nn;
	assert( n->l_pprev );
	if ( (nn = *n->l_pprev = n->l_next) )
		nn->l_pprev = n->l_pprev;
	n->l_next = 0;
	n->l_pprev = 0;
}

static inline void
softclock()
{
LanIpCallout           c, n;
rtems_interrupt_level  k1;
callout_time_t         st,ht;
LanIpCalloutRef        jobs = {0};

	/* I believe this is free of a race condition (softclock
	 * and hardclock both update volatile 'soft_ticks' variable):
	 *  a) 'hardclock' runs at IRQ level and is atomic
	 *  b) inside while loop 'soft_ticks' is != 'hard_ticks'
	 *  c) hardclock only modifies soft_ticks if 'soft_ticks'=='hard_ticks'
	 *     hence this could only happen just after the update of 'soft_ticks'
	 *     at the end of the while loop completes.
	 */

	while ( 1 ) {
	/* Must atomically read 'soft_ticks' and 'hard_ticks' -- otherwise,
	 * hardclock might update both but we get one old and one new value
	 */
	rtems_interrupt_disable(k1);
		st = soft_ticks;
		ht = hard_ticks;
	rtems_interrupt_enable(k1);
		if ( st == ht )
			break; /* caught up */

		/* at this point, we know that st != ht and therefore,
		 * hardclock will only increment hard_ticks but leave
		 * soft_ticks alone.
		 */

		st++;

		COLOCK();

		rtems_interrupt_disable(k1);
		for ( c = c_wheel[ st & WHEELMASK ].r_callout; c; c=n ) {
			n = lanIpCalloutNext(c);
			if ( 0 == c->c_time ) {
				/* this one expired */
					c->c_flags &= ~ CALLOUT_PENDING;
				c_deq(&c->c_node);
				c_enq(&jobs.r_node, &c->c_node);
			} else {
				c->c_time--;
			}
		}
		rtems_interrupt_enable(k1);

		soft_ticks = st;
		/* here, soft_ticks could have caught up and
		 * a hardclock occurring here could also
		 * update soft_ticks.
		 */
		for ( c = jobs.r_callout; c; c = n ) {
			n = lanIpCalloutNext(c);
			c_deq(&c->c_node);
			if ( c->c_func )
				c->c_func(c->c_arg0, c->c_arg1);
			rtems_interrupt_disable(k1);
				lanIpCalloutsScheduled--;
			rtems_interrupt_enable(k1);
		}

		COUNLOCK();
	}
}

static inline void
hardclock(rtems_id tid)
{
	if ( hard_ticks++ == soft_ticks && !c_wheel[hard_ticks & WHEELMASK].r_callout ) {
		/* nothing to do */
		soft_ticks++;
	} else {
		rtems_event_send(tid, CALLOUT_EVENT);
	}
}

static void
calloutTick(rtems_id myself, void *arg)
{
rtems_id tid = (rtems_id)arg;

	hardclock(tid);

	rtems_timer_fire_after(myself, 1, calloutTick, arg);
}

static void
calloutTask(void *arg)
{
rtems_event_set   ev;
rtems_status_code sc;
rtems_id          ticker = 0;
rtems_id          me;

	sc = rtems_timer_create(rtems_build_name('i','p','b','c'), &ticker);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "LanIpBasic: Creation of callout timer failed\n");
		goto bail;
	}
	rtems_task_ident(RTEMS_SELF, RTEMS_LOCAL, &me);

	rtems_timer_fire_after(ticker, 1, calloutTick, (void*)me);

	while ( 1 ) {
		sc = rtems_event_receive (CALLOUT_EVENT | CALLOUT_KILL_EVENT, RTEMS_EVENT_ANY | RTEMS_WAIT, RTEMS_NO_TIMEOUT, &ev);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc, "LanIpBasic: calloutTask unable to receive event; terminating\n");
			break;
		}
		if ( ev & CALLOUT_KILL_EVENT ) {
			break;
		}
		softclock();
	}

	assert( lanIpCalloutsScheduled == 0 );
bail:
	callout_tid = 0;	
	rtems_timer_delete(ticker);
	task_leave();
}


static int
lanIpCallout_trystop(LanIpCallout c)
{
rtems_interrupt_level l;
int                   active;

	rtems_interrupt_disable(l);
		if ( ! (c->c_flags & CALLOUT_PENDING) ) {
			if ( (active = (c->c_flags & CALLOUT_ACTIVE)) )
				c->c_flags |= CALLOUT_FAILEDSTOP;

			rtems_interrupt_enable(l);
			/* not pending anymore -- cannot remove */
			return active ? -1 : 0;
		}
		/* remove from list */
		c_deq(&c->c_node);
		c->c_flags &= ~(CALLOUT_ACTIVE | CALLOUT_PENDING | CALLOUT_FAILEDSTOP);
		lanIpCalloutsScheduled --;
	rtems_interrupt_enable(l);

	return 1;
}

static int
lanIpCallout_stop(LanIpCallout c)
{
int rval;
	if ( (rval = lanIpCallout_trystop(c)) < 0 ) {
		/* must wait for the callout task to process */
		COLOCK();
			rval = lanIpCallout_trystop(c);
		COUNLOCK();
	}
	assert ( rval >= 0 );
	return rval;
}

static int
lanIpCallout_reset(LanIpCallout c, uint32_t ticks, void (*fn)(void*,void*), void *arg0, void *arg1)
{
rtems_interrupt_level l;
int                   i, rval;

	if ( 0 == ticks )
		ticks = 1;

	rval = lanIpCallout_trystop(c);

	rtems_interrupt_disable(l);
	if ( (c->c_flags & CALLOUT_ACTIVE) ) {
		/* still active -- or already reset from other task; give up */
		rtems_interrupt_enable(l);
		return -1;
	} else {
		/* If 'stop' failed because callout was still active
		 * but it is not active anymore now then adjust...
		 */
		if ( -1 == rval ) {
			rval = 0;
			c->c_flags &= ~CALLOUT_FAILEDSTOP;
		}
	}
	c->c_func = fn;
	c->c_arg0 = arg0;
	c->c_arg1 = arg1;

	c->c_slot = i = (hard_ticks + ticks) & WHEELMASK;
	c->c_time = ticks >> WHEELBITS;

	/* enqueue */
	c_enq(&c_wheel[i].r_node, &c->c_node);

	c->c_flags |= (CALLOUT_ACTIVE | CALLOUT_PENDING);

	lanIpCalloutsScheduled++;
	rtems_interrupt_enable(l);

	return rval;
}

/* Find out how much time is remaining; if the callout is not pending
 * then return 0xffffffff
 */

static uint32_t
lanIpCallout_remaining(LanIpCallout c)
{
rtems_interrupt_level l;
uint32_t              rval;
	rtems_interrupt_disable(l);
		if ( (c->c_flags & CALLOUT_PENDING) ) {
			rval = (c->c_time << WHEELBITS) + ((c->c_slot - hard_ticks) & WHEELMASK); 
		} else {
			rval = 0xffffffff;
		}
	rtems_interrupt_enable(l);
	return rval;
}

static void
lanIpCallout_init(LanIpCallout c)
{
	/* non thread-safe lazy init in case nobody cared to do it ... */
	memset(c,0,sizeof(*c));	
}

static rtems_id
lanIpCallout_initialize()
{
	/* non thread-safe lazy init */
	if ( !callout_tid ) {
		callout_tid = task_spawn("ipbc", 25, 10000, calloutTask, 0);
	}
	if ( !callout_mtx ) {
		callout_mtx = bsem_create("colk", SEM_MUTX);
	}

	return callout_tid;
}

static void
lanIpCallout_finalize()
{
task_killer killer;

	killer.kill_event = CALLOUT_KILL_EVENT;

	if ( callout_tid ) {
		task_pseudojoin( callout_tid, KILL_BY_EVENT, killer );
	}
	if ( callout_mtx ) {
		rtems_semaphore_delete( callout_mtx );
		callout_mtx = 0;
	}
}

int
udpSockSetIfMcast(int sd, uint32_t ifipaddr)
{
int rval      = -1;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	SOCKLOCK( & socks[sd] );

	/* this is in fact unimplemented; we only support a single IF */
	rval = ifipaddr && ifipaddr != socks[sd].intrf->ipaddr ? - EADDRNOTAVAIL : 0;

	SOCKUNLOCK( & socks[sd] );

	return rval;
}

static int
addmca(IpBscIf intrf, int sd, uint32_t mcaddr)
{
IpBscMcAddr    mca, mcan;
int            rval = 0;
rtems_interval ticks_per_s;
uint32_t       report_dly_ticks;

	mcan = calloc(1, sizeof(*mcan));

	lanIpCallout_init( &mcan->mc_igmp );
	mcan->mc_addr = mcaddr;

	MCLOCK( intrf );

		if ( (mca = lhtblFind( intrf->mctable, mcaddr )) ) {
			if ( (mca->mc_sobs & (1<<sd)) ) {
				rval =  -EADDRINUSE;
				goto bail;
			}
			mca->mc_sobs |= (1<<sd);
		} else {
			uint8_t enaddr[6];

			mcan->mc_sobs = (1<<sd);

			ipmc2ethermc(mcaddr, enaddr);

			if ( lhtblAdd( intrf->mctable, mcan ) ) {
				rval = -ENOMEM;
				goto bail;
			} 

			NETDRV_MC_FILTER_ADD( intrf, enaddr );

			c_enq( &intrf->mclist.r_node, &mcan->mc_node );

			if ( ! ISMCST_ALLSYS(mcan->mc_addr) ) {

				rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &ticks_per_s);

				report_dly_ticks = 10 /* sec, 'unsolicited report interval' (RFC2236) */ * ticks_per_s;

				/*
				 * start IGMP (igmp_send_msg falls back to V1
				 * if needed).
				 */
				igmp_send_msg(intrf, mcaddr, IGMP_TYPE_REPORT_V2);

				mca->mc_flags |= MC_FLG_IGMP_LEAVE;

				/*
				 * If random delay is 0 then lanIpCallout_reset()
				 * adjusts this to 1.
				 */
				lanIpCallout_reset(
					& mcan->mc_igmp,
					badrand16() % report_dly_ticks,
					igmp_timeo,
					(void*)mcan->mc_addr,
					intrf
				);
			}

			intrf->mcnum++;
			mcan = 0;
		}
		
bail:

	MCUNLOCK( intrf );

	free(mcan);

	return rval;
}

int
udpSockJoinMcast(int sd, uint32_t mcaddr)
{
	if ( ! ISMCST( mcaddr ) )
		return -EINVAL;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	return addmca( socks[sd].intrf, sd, mcaddr );
}

/* Delay until it is safe to release memory occupied
 * by a IpBscMcAddrRec struct and then 'free()'.
 *
 * The reason is that 'delmca()' might fail to stop
 * the callout because the callout task has already
 * begun executing it.
 * 
 * 'delmca()' must not use the synchronous lanIpCallout_stop()
 * operation because of the potential deadlock situation
 * (see below).
 *
 * This routine MUST be called w/o holding MLOCK()
 * which is not necessary because the 'mca' object
 * has already been removed from the hash table
 * and the interface's mc address list.
 *
 * If we find that stopping the callout had failed
 * then we acquire the COLOCK() thus making sure
 * the callout task has finished processing this
 * callout.
 */
static void
destroymca(IpBscMcAddr mca)
{
	if ( lanIpCallout_failedstop( &mca->mc_igmp ) ) {
		/* Callout couldn't be stopped by 'delmca()'; synchronize
		 * with the callout task to make sure this callout
		 * has finished executing.
		 */
		COLOCK();
		COUNLOCK();
	}

	assert( ! lanIpCallout_active( &mca->mc_igmp ) );

	free(mca);
}

static IpBscMcAddr
delmca(IpBscIf intrf, IpBscMcAddr mca, int sd)
{
uint8_t enaddr[6];
int     lhtblDelFailedFatally;

	if ( ! (mca->mc_sobs & (1<<sd)) )
		return 0;

	if ( 0 == (mca->mc_sobs &= ~ (1<<sd)) ) {

		intrf->mcnum--;

		if ( ! ISMCST_ALLSYS(mca->mc_addr) ) {
			/* stop IGMP */
			if ( (mca->mc_flags & MC_FLG_IGMP_LEAVE) ) {
				igmp_send_msg(intrf, mca->mc_addr, IGMP_TYPE_LEAVE);
			}
		}

		/* Cannot use lanIpCallout_stop here because there is
		 * a potential for deadlock.
		 * At this point we hold the MLOCK and would try to
		 * acquire COLOCK.
		 * However, the callout task first acquires COLOCK and
		 * then executes the callbacks which might lock MLOCK
		 * (note inverse order). Hence there is a potential
		 * deadlock situation.
		 */
		lanIpCallout_trystop( &mca->mc_igmp );

		assert( ! lanIpCallout_active( &mca->mc_igmp ) );

		c_deq( &mca->mc_node );

		ipmc2ethermc( mca->mc_addr, enaddr );

		NETDRV_MC_FILTER_DEL( intrf, enaddr );

		lhtblDelFailedFatally = lhtblDel( intrf->mctable, mca );

		assert( !lhtblDelFailedFatally );

		return mca;
	}
	return 0;	
}

int
udpSockLeaveMcast(int sd, uint32_t mcaddr)
{
IpBscMcAddr mca = 0;
int         rval = 0;

	if ( ! ISMCST( mcaddr ) )
		return -EINVAL;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	MCLOCK( socks[sd].intrf );

		if ( ! (mca = lhtblFind( socks[sd].intrf->mctable, mcaddr )) ) {

			rval = -EADDRNOTAVAIL;

		} else {
			if ( ! (mca->mc_sobs & (1<<sd)) ) {
				rval = -EADDRNOTAVAIL;
			} else {
				mca = delmca(socks[sd].intrf, mca, sd);
			}
		}
		
	MCUNLOCK( socks[sd].intrf );

	destroymca(mca);

	return rval;
}

int
lanIpBscInit()
{

	if ( ! (workSema = sem_create("ipbw", SEM_CNTG, 0)) ) {
		return -1;
	}

	if ( ! ( workTask = task_spawn("ipbw", 180, 4096, lpWorker, 0) ) )
		goto bail;
	
	if ( ! lanIpCallout_initialize() )
		goto bail;

	if ( ! (arpcache[ARP_SENTINEL] = arp_createentry()) ) {
		goto bail;
	}

	arpcache[ARP_SENTINEL]->ctime = ARP_PERM;

	return 0;

bail:
	if ( workSema ) {
		/* This lets the worker task terminate if it was spawned
		 * successfully
		 */
		rtems_semaphore_delete(workSema);
		workSema = 0;
	}
	return 1;
}

int
lanIpBscShutdown()
{
unsigned              missing;
task_killer           killer;

	if ( nsocks )
		return -1;

	arpTearDown(1);
	arp_destroyentry(arpcache[ARP_SENTINEL]);
	arpcache[ARP_SENTINEL] = 0;

	killer.kill_resource = workSema;

	task_pseudojoin( workTask, KILL_BY_SEMA, killer );

	workTask = 0;

	/* Interface and driver should be down by now. Hence,
	 * no more low-priority work can be scheduled and it
	 * should be OK to kill the lpWorker task.
	 */

	if ( (missing = lanIpBufTotal - lanIpBufAvail) ) {
		fprintf(stderr,"lanIpBscShutdown() failed: %u rbufs still in use\n", missing);
		return -1;
	}

	lanIpCallout_finalize();

	freeBufMem();

	return 0;
}


/* Not fully thread-safe; must not destroy
 * interface while using this.
 */
int
lanIpBscDumpMcGroups(IpBscIf ipbif_p, FILE *f)
{
unsigned     naddrs,naddrs1,n;
uint32_t    *mc_addrs = 0;
IpBscMcAddr  mca;

	if ( ! ipbif_p && ! (ipbif_p = intrf) ) {
		fprintf(f, "No interface\n");
		return -1;
	}

	if ( !f )
		f = stdout;

	if ( ipbif_p->mctable ) {
		fprintf(f, "Hash table statistics:\n");
		lhtblDumpStats(ipbif_p->mctable, f);
	}

	MCLOCK(ipbif_p);
		naddrs = ipbif_p->mcnum;
	MCUNLOCK(ipbif_p);

	fprintf(f,"MC Groups interface is currently a member of:\n");

	/* Allocate a few more slots - just in case they
	 * add more subscriptions while we malloc space
	 */
	naddrs += 2;

	/* Allocate space for a cache of currently
	 * subscribed addresses.
	 */
	if ( ! (mc_addrs = malloc(sizeof(*mc_addrs)*naddrs)) ) {
		fprintf(f,"No memory for cached address list, sorry\n");
		return -1;
	}

	MCLOCK(ipbif_p);
		for ( n=0, mca=ipbif_p->mclist.r_mcaddr; mca && n < naddrs; mca = nxtmca(mca), n++ )
			mc_addrs[n] = mca->mc_addr;
		naddrs  = n;
		naddrs1 = ipbif_p->mcnum;
	MCUNLOCK(ipbif_p);

	/* If new count is larger than old one (more than 2 groups were joined
	 * during our malloc() operation then warn and print what we have.
	 */
	if ( naddrs1 > naddrs ) {
		fprintf(f,"WARNING: some groups will not be dumped -- (added during dump)\n");
	}

	for ( n=0; n<naddrs; n++ ) {
		fputc(' ',f);
		prip(f, mc_addrs[n]);
		fputc('\n',f);		
	}

	free(mc_addrs);
	return 0;
}

static inline int
igmp_state_delaying(IpBscMcAddr mca)
{
	return mca && lanIpCallout_active( &mca->mc_igmp );
}

static inline int
igmp_state_idle(IpBscMcAddr mca)
{
	return mca && ! lanIpCallout_active( &mca->mc_igmp );
}

static inline int
igmp_query_is_v1(IgmpV2HeaderRec *pigmph)
{
	return IGMP_TYPE_QUERY == pigmph->type && 0 == pigmph->max_rtime;
}

static int
igmp_v1_rtr_seen(IpBscIf intrf)
{
	return lanIpCallout_active( &intrf->mcIgmpV1RtrSeen);
}

static void
igmp_prepare_msg(IpBscIf intrf, rbuf_t *buf_p, uint32_t gaddr, uint8_t type)
{
LanIgmpV2 pigmp = &lpkt_igmpv2hdr( &buf_p->pkt );
uint32_t  opt;
uint16_t  s1, s2;
uint32_t  daddr;

	if( IGMP_TYPE_LEAVE == type ) {
		daddr = htonlc(IP_GRP_ALL_ROUTERS);
	} else {
		daddr = gaddr;
		type  = igmp_v1_rtr_seen( intrf ) ? IGMP_TYPE_REPORT_V1 : IGMP_TYPE_REPORT_V2;
	}


	/* Prepare Ethernet Header -- src address is filled-in later */
	ipmc2ethermc(daddr, pigmp->ll.dst);
	pigmp->ll.type = htonsc(0x800); /* IP */

	pigmp->ip.vhl     = 0x46;          /* v4, header + rtr-alert option */
	pigmp->ip.tos     = 0x00;
	pigmp->ip.len     = htonsc(sizeof(IpHeaderRec) + 4 + sizeof(IgmpV2HeaderRec));
	pigmp->ip.id      = htonsc(0);
	pigmp->ip.off     = htonsc(0);
	pigmp->ip.ttl     = 1;
	pigmp->ip.prot    = IP_PROT_IGMP;
	/* pigmp->ip.csum   filled in later */
	/* pigmp->ip.src    filled in later */
	pigmp->ip.dst     = daddr;
	pigmp->ip.opts[0] = opt = htonlc(IP_OPT_ROUTER_ALERT);

	/* Set source IP and ethernet addresses and compute
	 * header checksum (w/o router-alert option)
	 */
	fillinSrcCsumIp( intrf, &lpkt_iphdr( &buf_p->pkt ) );

	/* Adjust the checksum for the router alert option */
	s1                = (opt>>16);
	s2                =  opt;
	if ( (s1 += s2) < s1 )
		s1++;
	
	s2                = ~pigmp->ip.csum;

	if ( (s1 += s2) < s1 )
		s1++;

	pigmp->ip.csum    = ~s1;

	pigmp->igmp_u.ip46.igmp.type  = type;
	pigmp->igmp_u.ip46.igmp.max_rtime = 0;
	pigmp->igmp_u.ip46.igmp.gaddr = gaddr;

	pigmp->igmp_u.ip46.igmp.csum  = 0;
	pigmp->igmp_u.ip46.igmp.csum  = ipcsum((uint8_t*) &pigmp->igmp_u.ip46.igmp, sizeof(pigmp->igmp_u.ip46.igmp));
}

/* NOTE: if the return value of igmp_send_msg() is dereferenced
 *       then the caller must acquire MCLOCK():
 *
 *          MCLOCK();
 *             mca = igmp_send_msg();
 *             if ( mca )
 *                 use_mca(mca);
 *          MCUNLOCK();
 *       
 *       It is not necessary to MCLOCK() if the return value
 *       is discarded or only used to find out if a message
 *       was actually sent.
 */
#warning 'static removed'
IpBscMcAddr
igmp_send_msg(IpBscIf intrf, uint32_t gaddr, uint8_t type)
{
rbuf_t      *buf_p;
IpBscMcAddr  mca = 0;

	buf_p = getrbuf();

#ifdef DEBUG
	if ( (lanIpDebug & (DEBUG_IGMP)) ) {
		char *tstr;
		switch ( type ) {
			case IGMP_TYPE_REPORT_V1:
				tstr = "REPORT (V1)";
			break;
			case IGMP_TYPE_REPORT_V2:
				tstr = "REPORT (V2)";
			break;
			case IGMP_TYPE_LEAVE:
				tstr = "LEAVE (V2)";
			break;
			default:
				tstr = "<UNKNOWN>";
			break;
		}
		printf("Sending IGMP %s to ", tstr); prip(stdout, gaddr); fputc('\n', stdout);
		if( !buf_p ) {
			printf("OOPS: there was no buffer; couldn't send...\n");
		}
	}
#endif

	if ( buf_p ) {
		/* look 'mca' up -- entry could be stale if 'Leave' was
		 * executed during the window where a callout calling
		 * this routine couldn't be cancelled because it was already
		 * half-ways executing.
		 */
		if ( (mca = lhtblFind( intrf->mctable, gaddr )) ) {
			igmp_prepare_msg(intrf, buf_p, gaddr, type);
			/* For the moment this routine does not do local loopback
			 * but we need not see our own IGMP messages...
			 */
			lanIpBscSendBufRaw(intrf, &buf_p->pkt, sizeof(lpkt_igmpv2hdr(&buf_p->pkt)));
			buf_p = 0;
		}
	}

	relrbuf(buf_p);

	return mca;
}

static void
processIcmp(IpBscIf intrf, rbuf_t *p, int len)
{
IcmpHeaderRec *picmp = &lpkt_icmp(&p->pkt);

	if ( picmp->type == 8 /* ICMP REQUEST */ && picmp->code == 0 ) {
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ICMP )
			printf("handling ICMP ECHO request\n");
#endif
		picmp->type = 0; /* ICMP REPLY */
		picmp->csum = 0;
		picmp->csum = ipcsum((uint8_t*)picmp, len - sizeof(lpkt_ip( &p->pkt )));
		lpkt_eth(&p->pkt).type = htonsc(0x800); /* IP */

		/* refresh peer's ARP entry */
		if ( lanIpBscAutoRefreshARP ) {
			arpPutEntry(intrf, lpkt_ip(&p->pkt).src, lpkt_eth(&p->pkt).src, 0);
		}

		src2dstIp(&lpkt_iphdr(&p->pkt));
		fillinSrcCsumIp(intrf, &lpkt_iphdr(&p->pkt));

		refrbuf(p);
		NETDRV_ENQ_BUFFER(intrf, p, sizeof(EthHeaderRec) + len);
	}
}

static void
processIgmp(IpBscIf intrf, rbuf_t *buf_p, int len)
{
IpBscMcAddr      mca,mcal;
IgmpV2HeaderRec *pigmp;
rtems_interval   ticks_per_s;
uint32_t         report_dly_ticks;
int              pldlen;
LanIgmpV2        pkt;

	pkt = &lpkt_igmpv2hdr( &buf_p->pkt );

	assert( IP_PROT_IGMP == pkt->ip.prot );

	/* This IP header has one option word */
	pldlen = len - sizeof(IpHeaderRec);

	if ( 0x46 == pkt->ip.vhl ) {
		if ( htonlc(IP_OPT_ROUTER_ALERT) != pkt->ip.opts[0] ) {
#ifdef DEBUG
			if ( lanIpDebug & (DEBUG_IP | DEBUG_IGMP) ) {
				printf("Dropping IGMP packet w/o router alert (option[0]: 0x%08"PRIx32")\n",
						ntohl(pkt->ip.opts[0]));
			}
#endif
			return;

		}
		/* This IP header has one option word */
		pigmp   = & lpkt_igmpv2_ra( &buf_p->pkt ); 
		pldlen -= sizeof(uint32_t);
	} else {
		pigmp   = & lpkt_igmpv2_nora( &buf_p->pkt );
	}

	if ( pldlen < 8 ) {
#ifdef DEBUG
		if ( lanIpDebug & (DEBUG_IP | DEBUG_IGMP) ) {
			printf("Dropping IGMP packet with length (ip payload %u < 8)\n", pldlen);
		}
#endif
		return;
	}

	if ( ipcsum((uint8_t*) pigmp, pldlen) ) {
#ifdef DEBUG
		if ( lanIpDebug & (DEBUG_IP | DEBUG_IGMP) ) {
			printf("Dropping IGMP packet with bad checksum\n");
		}
#endif
		return;
	}

#ifdef DEBUG
	if ( lanIpDebug & (DEBUG_IGMP) ) {
		printf("IGMP packet Received: type 0x%02x, resp. time 0x%02x, group: ",
			pigmp->type, pigmp->max_rtime);
		prip(stdout, pigmp->gaddr); fputc('\n', stdout);

		printf("  SRC: "); prip(stdout,pkt->ip.src); fputc('\n', stdout);
		printf("  DST: "); prip(stdout,pkt->ip.dst); fputc('\n', stdout);
	}
#endif
	
	MCLOCK( intrf );

	switch( pigmp->type ) {
		case IGMP_TYPE_REPORT_V1:
		case IGMP_TYPE_REPORT_V2:
			mca = lhtblFind( intrf->mctable, pigmp->gaddr);
			if ( igmp_state_delaying( mca ) ) {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("Report to ");   prip(stdout, pigmp->gaddr);
					printf(" seen (src: "); prip(stdout, pkt->ip.src);
					printf(") SUPPRESSING my reports\n");
				}
#endif
				mca->mc_flags &= ~ MC_FLG_IGMP_LEAVE;
				lanIpCallout_trystop( &mca->mc_igmp );
			}
		break;

		case IGMP_TYPE_QUERY:

			rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &ticks_per_s);

			if ( igmp_query_is_v1( pigmp ) ) {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("IGMP V1 Query seen; switching to V2, starting timer...\n");
				}
#endif
				/* NOTE: we ignore failure of lanIpCallout_reset
				 *       (could happen if we try to reschedule
				 *       just while the callback is executing).
				 *       This has the same effect as losing this
				 *       packet...
				 */
				lanIpCallout_reset(
					&intrf->mcIgmpV1RtrSeen,
					400 /* sec, 'version 1 router present timeout' as per RFC2236 */ * ticks_per_s,
					igmp_v1timeo,
					0,
					intrf
				);
				report_dly_ticks = 100;
			} else {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("IGMP V2 Query seen -- ");
				}
#endif
				report_dly_ticks = pigmp->max_rtime;
			}

			/* IGMP time is measured in 100ms; convert to ticks... */
			report_dly_ticks *= ticks_per_s;
			report_dly_ticks /= 10;

			if (   htonlc(0) == pigmp->gaddr
				&& ISMCST_ALLSYS(pkt->ip.dst)   ) {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("GENERAL\n");
				}
#endif
				/* General query */
				mca  = intrf->mclist.r_mcaddr;
				mcal = intrf->mcallsys;
			} else {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("GROUP: "); prip(stdout, pigmp->gaddr); fputc('\n', stdout);
				}
#endif
				/* make sure we ignore a query for 224.0.0.1 */
				if ( ISMCST_ALLSYS(pigmp->gaddr) ) {
#ifdef DEBUG
					if ( lanIpDebug & (DEBUG_IGMP) ) {
						printf("IGNORING BAD QUERY (for 224.0.0.1)\n");
					}
#endif
					break;
				}

				if ( ! (mca = lhtblFind(intrf->mctable, pigmp->gaddr)) ) {
#ifdef DEBUG
					if ( lanIpDebug & (DEBUG_IGMP) ) {
						printf("IGNORING (I'm not a member)\n");
					}
#endif
					break;
				}

				mcal = nxtmca(mca);
			}

			while ( mca != mcal ) {
				/* start/reset all timers */
				if ( lanIpCallout_remaining( &mca->mc_igmp ) > report_dly_ticks ) {
					/* If the callout is not currently pending then
					 * lanIpCallout_remaining returns 0xffffffff
					 * If random delay is 0 then lanIpCallout_reset()
					 * adjusts this to 1.
					 *
					 * NOTE: We ignore failure of 'reset' which could
					 *       happen if the callback is executing while
					 *       we're trying to reschedule.
					 *       This has the same effect as losing this
					 *       packet...
					 */
					lanIpCallout_reset(
							& mca->mc_igmp,
							badrand16() % report_dly_ticks,
							igmp_timeo,
							(void*)mca->mc_addr,
							intrf
							);
				}
				mca = nxtmca(mca);
			}
		break;


		default:
		break;
	}

	MCUNLOCK( intrf );
}

/* IGMP callbacks; executed from the
 * context of the 'callout' task...
 */
static void
igmp_timeo(void *arg0, void *arg1)
{
uint32_t    mcaddr = (uint32_t)arg0;
IpBscIf     intrf  = arg1;
IpBscMcAddr mca;

#ifdef DEBUG
	if ( lanIpDebug & (DEBUG_IGMP) ) {
		printf("IGMP Timeout; sending report (V%c) for ",
			igmp_v1_rtr_seen(intrf) ? '1' : '2');
		prip(stdout, mcaddr);
		fputc('\n',stdout);
	}
#endif

	MCLOCK( intrf );
		/* igmp_send_msg() looks 'mca' up -- if we passed it (e.g., as
		 * 'arg0' the entry could now be stale -- if 'Leave' was
		 * executed during the window where this callout couldn't
		 * be cancelled because it was already half-ways executing
		 */

		/* msg type is corrected if the interface has recently seen a V1 query */
		if ( (mca = igmp_send_msg(intrf, mcaddr, IGMP_TYPE_REPORT_V2)) ) {
			mca->mc_flags |= MC_FLG_IGMP_LEAVE;
		}
		lanIpCallout_deactivate( &mca->mc_igmp );
	MCUNLOCK( intrf );
}

static void
igmp_v1timeo(void *arg0, void *arg1)
{
IpBscIf     intrf = arg1;
#ifdef DEBUG
	if ( lanIpDebug & (DEBUG_IGMP) ) {
		printf("No IGMP V1 Query seen in a while (time expired); switching to V2\n");
	}
#endif
	MCLOCK(intrf);
		lanIpCallout_deactivate( &intrf->mcIgmpV1RtrSeen );
	MCUNLOCK(intrf);
}


static rtems_name
str2name(const char *str)
{
int  i;
char nm[4];

	i = 0;

	if ( str ) {
		while ( i < sizeof(nm) && (nm[i]=str[i]) )
			i++;
	}

	while ( i < sizeof(nm) ) {
		nm[i++] = ' ';
	}

	return rtems_build_name(nm[0],nm[1],nm[2],nm[3]);
}

/* Utility wrappers */

static const char *
nullchk(const char *s)
{
static const char *const nobody = "<nobody>";

	return  s ? s : nobody;
}

static void
task_init( rtems_id tid )
{
	rtems_task_set_note( tid, TASK_JOIN_NPAD, 0 );
}

static rtems_id
task_spawn(char *name, int pri, int stacksz, void (*fn)(void*), void *arg)
{
rtems_status_code sc;
rtems_id          tid;

	if ( pri <= 0 )
		pri = 20;

	if ( 0 == stacksz )
		stacksz = 10000;

	sc = rtems_task_create(
				str2name(name),
				pri,	/* can be changed later */
				stacksz,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "creating task %s\n", nullchk(name));
		return 0;
	}

	task_init( tid );

	sc = rtems_task_start( tid, (rtems_task_entry)fn, (rtems_task_argument) arg );
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "starting drvAmdIpBasicTask\n");
		rtems_task_delete( tid );
		return 0;
	}
	return tid;
}

static void
task_leave()
{
rtems_id sync_sem;

	rtems_task_get_note( RTEMS_SELF, TASK_JOIN_NPAD, (uint32_t*) & sync_sem );
	rtems_semaphore_release( sync_sem );
	rtems_task_suspend(RTEMS_SELF);
}

/* Unfortunately, the RTEMS classic API doesn't provide a way to
 * synchronize with a terminating task.
 *
 * This routine provides a kludge which works as long as the 'to-be killed'
 * task doesn't block. More specifically, we assume that the target
 * task only blocks either on a set of events or on a resource such
 * as a semaphore which we may delete in order to 'kill' the target task.
 *
 * Our kludge implements the following algorithm:
 *
 *   lower executing task's priority below the target task's
 *   'kill' target task
 *   restore executing task's original priority.
 *
 * This sequence ensures that when we get to execute
 * the third step the target task has terminated.
 *
 * NOTE: this routine is intended mainly for development&debugging purposes.
 *       Normally, none of the tasks in lanIpBasic should terminate
 *       but it can be convenient (during debugging/development) to
 *       unload the lanIpBasic stack.
 */

static rtems_id
task_pseudojoin_prepare(rtems_id tid)
{
rtems_status_code sc;
rtems_id          sync_sem;

	/* Create synchronization semaphore */
	sync_sem = bsem_create("sync", SEM_SYNC);

	assert ( 0 != sync_sem );

	/* And attach to target task's context */
	sc = rtems_task_set_note( tid, TASK_JOIN_NPAD, (uint32_t)sync_sem );
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "lanIpBasic (task_pseudojoin(0x%08x)): FATAL ERROR; setting notepad failed\n", tid);
		abort();
	}

	return sync_sem;
}

static void
task_pseudojoin_wrapup(rtems_id tid, rtems_id sync_sem)
{
rtems_status_code sc;

	/* Wait for task to terminate */
	sc = rtems_semaphore_obtain( sync_sem, RTEMS_WAIT, 400 );

	if ( RTEMS_SUCCESSFUL == sc ) {
		rtems_semaphore_delete( sync_sem );
		rtems_task_delete( tid );
	} else {
		rtems_error(sc, "WARNING (task_pseudojoin_wrapup): synchronizing with termination of  Task 0x%08x\n"
                        "                                  failed; may leak resources or crash",
                    tid);
	}
}

static void
task_pseudojoin(rtems_id tid, int method, task_killer killer)
{
rtems_status_code sc;
rtems_id          sync_sem;

	sync_sem = task_pseudojoin_prepare( tid );

	/* Kill */
	switch ( method ) {
		case KILL_BY_EVENT:
			sc = rtems_event_send(tid, killer.kill_event);
			if ( RTEMS_SUCCESSFUL != sc ) {
				rtems_error(sc, "lanIpBasic (task_pseudojoin(0x%08x)): FATAL ERROR; sending KILL event failed\n", tid);
				abort();
			}
		break;

		case KILL_BY_SEMA:
			sc = rtems_semaphore_delete(killer.kill_resource);
			if ( RTEMS_SUCCESSFUL != sc ) {
				rtems_error(sc, "lanIpBasic (task_pseudojoin(0x%08x)): FATAL ERROR; deleting KILL semaphore failed\n", tid);
				abort();
			}
		break;

		default:
			fprintf(stderr,"task_pseudojoin: unimplemented KILL method used\n");
			abort();
		break; /* never get here */
	}

	task_pseudojoin_wrapup( tid, sync_sem );
}

static rtems_id
sem_create(char *name, int type, int count)
{
rtems_status_code sc;
rtems_id          rval;
rtems_attribute   atts = 0;

	switch ( type ) {
		default:
			return 0;
		
		case SEM_MUTX:
			atts = RTEMS_BINARY_SEMAPHORE;
		break;

		case SEM_SMTX:
		case SEM_SYNC:
			atts = RTEMS_SIMPLE_BINARY_SEMAPHORE;
		break;

		case SEM_CNTG:
			atts = RTEMS_COUNTING_SEMAPHORE;
		break;
	}

	if ( SEM_SYNC != type && SEM_CNTG != type ) {
		atts |= RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY;
	}

	sc = rtems_semaphore_create(
			str2name(name),
			count,
			atts,
			0,
			&rval);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "creating binary semaphore %s failed\n", nullchk(name));
		rval = 0;
	}
	return rval;
}

static rtems_id
bsem_create(char *name, int type)
{
	return sem_create(name, type, SEM_SYNC == type ? 0 : 1);
}

static void
lpWorker(void *arg)
{
rbuf_t     *buf_p;
IpBscIf    intrf;
LanIpPacket   pkt;
IpArpRec  *pipa;
int        len;

	while ( ( buf_p = dequeueLpWork() ) ) {

		intrf =   buf_p->buf.intrf;
		pkt   = & buf_p->pkt;

		if ( htonsc(0x806) == lpkt_eth(pkt).type ) {
			/* ARP; must be a reply we received and that
			 * they wanted us to store in the cache...
			 */

			pipa = &lpkt_arp( pkt );

#ifdef DEBUG
			if ( lanIpDebug & DEBUG_ARP ) {
				printf("got ARP reply from "); prether(stdout, pipa->sha);
				printf("\n");
			}
#endif

			arpPutEntry(intrf, get_spa(pipa), pipa->sha, 0);

		} else if ( htonsc(0x800) == lpkt_eth(pkt).type ) {

			len = ntohs( lpkt_ip( pkt ).len );
			switch ( lpkt_ip( pkt ).prot ) {

				case IP_PROT_IGMP:
					processIgmp(intrf, buf_p, len);
				break;

				case IP_PROT_ICMP:
					processIcmp(intrf, buf_p, len);
				break;

				default:
					fprintf(stderr,"LanIpBasic: (lpWorker) BAD IP PROTO -- we should never get here\n");
				break;
			}
		} else {
			fprintf(stderr,"LanIpBasic: (lpWorker) BAD LL PROTO -- we should never get here\n");
		}

		relrbuf(buf_p);
	}

	/* clean up remaining buffers */
	while ( (buf_p = workHead) ) {
		workHead = buf_p->buf.next;
		relrbuf( workHead );
	}

	workTail = 0;

	workSema = 0;

	task_leave();
}
