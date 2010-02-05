/* $Id$ */

/* Rudimentary IP/UDP protocol:
 *
 *  - trivial ARP
 *  - ICMP echo request (ping) handling
 *  - IGMP (v2)
 *  - trivial IP (only 20byte header)
 *  - basic UDP (no checksum)
 *
 *  - LAN only (no IP routing; we always assume IP
 *    and link-layer addresses match up)
 *
 */

/* T. Straumann <strauman@slac.stanford.edu> */

/* For _Thread_Disable_dispatch()/_Thread_Enable_dispatch() */
#define __RTEMS_VIOLATE_KERNEL_VISIBILITY__

#include <rtems.h>
#include <rtems/error.h>
#include <rtems/endian.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

/* Minimal stuff from BSD to get 'htons()', 'inet_addr()', 'in_cksum_hdr()' and
 * similar functionality.
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

/* include netdriver AFTER defining VIOLATE_KERNEL_VISIBILITY (in case it uses
 * rtems.h already)
 */
#include NETDRV_INCLUDE

/**** GENERIC CONSTANTS *******************************************************/

/* Protocol numbers                                                           */
#define IP_PROT_ICMP		1
#define IP_PROT_IGMP		2
#define IP_PROT_UDP			17

/* IGMPv2 message types                                                       */
#define IGMP_TYPE_QUERY     0x11
#define IGMP_TYPE_REPORT_V1 0x12
#define IGMP_TYPE_REPORT_V2 0x16
#define IGMP_TYPE_LEAVE     0x17

/* IPv4 options                                                               */
#define IP_OPT_ROUTER_ALERT	0x94040000

/* Debugging flags                                                            */
#define DEBUG_ARP	        1
#define DEBUG_IP	        2
#define DEBUG_ICMP	        4
#define DEBUG_UDP	        8
#define DEBUG_TASK	        16
#define DEBUG_IGMP          32

/**** CONFIGURATION CONSTANTS *************************************************/

/* If DEBUG is undefined then debugging code is removed; if it is defined
 * then the initial debug flag is defined by it's value. I.e., 
 *
 * #define DEBUG 0
 *
 * Compiles debugging code but leaves it disabled (can be enabled at run-time
 * by setting the lanIpDebug variable.
 */
#define DEBUG		0

/* How many times to retry a ARP lookup                                       */
#ifndef ARP_SEND_RETRY
#define ARP_SEND_RETRY	3
#endif

/* How long to wait for an ARP reply; 1 tick is many ms which is enough...    */
#ifndef ARP_TIMEOUT_TICKS
#define ARP_TIMEOUT_TICKS    ((rtems_interval)1) /* Ticks                     */
#endif

/* Minimal alignment of RBUFs; fall back on packet alignment if undefined     */
#if     RBUF_ALIGNMENT < LAN_IP_BASIC_PACKET_ALIGNMENT
#undef  RBUF_ALIGNMENT
#define RBUF_ALIGNMENT LAN_IP_BASIC_PACKET_ALIGNMENT
#endif

/* How many overflow buckets should the ARP hash algorithm use                */
/* FIXME: should we use a shtbl lhtbl for this?                               */
#ifndef CACHE_OVERLAP
#define CACHE_OVERLAP 10
#endif

/* How many RBUFs to configure initially - more can be added at run-time      */
#ifndef NRBUFS
#define NRBUFS		50	/* Initial total number of RX buffers                 */
#endif

/* Max. number of 'sockets' we support.                                       */
/* THE ALGORITHMS RELY ON THIS BEING A SMALL NUMBER                           */
#ifndef NSOCKS
#define NSOCKS		5
#endif

/* RX socket queue depth (initial/default value).                             */
#ifndef QDEPTH
#define QDEPTH		20
#endif

/* Port # where we start to assign when the user tells us to pick a free port */
#ifndef DEFLT_PORT
#define DEFLT_PORT  31110
#endif

/**** UTILITY MACROS **********************************************************/

/* Align to rbuf alignment                                                    */
#define RBUF_ALGN(x)   (((x) + (RBUF_ALIGNMENT-1)) & ~(RBUF_ALIGNMENT-1))

/* Special IPv4 multicast addresses                                           */
#define IP_GRP_ALL_SYSTEMS  0xe0000001
#define IP_GRP_ALL_ROUTERS  0xe0000002
	
/* Check if a IP address 'ip' is a broadcast address (netmask 'nm')           */
#define ISBCST(ip,nm)  (((ip) & ~(nm)) == ~(nm))

/* Check if a IP address 'ip' is a multicast address                          */
#define ISMCST(ip)     (htonlc(0xe0000000) == (htonlc(0xf0000000) & (ip)))

/* Check if a IP address 'ip' is the special 'all-systems' multicast address  */
#define ISMCST_ALLSYS(ip) (htonlc(IP_GRP_ALL_SYSTEMS) == (ip))

#if BIG_ENDIAN == LITTLE_ENDIAN
#error "BIG_ENDIAN == LITTLE_ENDIAN ??"
#endif

#if BYTE_ORDER != BIG_ENDIAN && BYTE_ORDER != LITTLE_ENDIAN
#error "undefined or weird byte-ordering"
#define BYTE_ORDER_UNKOWN
#endif

/**** TYPE DECLARATIONS *******************************************************/

/* Alias helper types                                                         */
typedef uint32_t	uint32_a_t __attribute__((__may_alias__));
typedef  int32_t	 int32_a_t __attribute__((__may_alias__));
typedef uint16_t	uint16_a_t __attribute__((__may_alias__));
typedef   int8_t	  int8_a_t __attribute__((__may_alias__));
typedef  uint8_t	 uint8_a_t __attribute__((__may_alias__));
typedef  int16_t	 int16_a_t __attribute__((__may_alias__));

/* Forward Declaration of structs                                             */
typedef struct LanIpCalloutRec_ *LanIpCallout;
typedef struct LanIpLstNodeRec_ *LanIpLstNode;

/* An elementary doubly-linked list node                                      */
typedef struct LanIpLstNodeRec_ {
	LanIpLstNode	 l_next;
	LanIpLstNode	*l_pprev;
} LanIpLstNodeRec;

/* Callout struct; used to schedule work in the future                        */
typedef struct LanIpCalloutRec_ {
	LanIpLstNodeRec c_node;
	void          (*c_func)(void*,void*);
	void           *c_arg0;
	void           *c_arg1;
	uint32_t        c_time; 
	uint16_t        c_flags;
	uint16_t        c_slot;
} LanIpCalloutRec;

/* Macros to abbreviate access to the list node embedded in a callout         */
#define c_next  c_node.l_next
#define c_pprev c_node.l_pprev

/* Valid settings of c_flags                                                  */
#define CALLOUT_PENDING 	(1<<0)  /* callout scheduled                      */
#define CALLOUT_ACTIVE  	(1<<1)  /* scheduled or not finished executing    */
#define CALLOUT_FAILEDSTOP	(1<<2)  /* a failed attempt to stop was detected  */

/* Callout work is subject to a race condition:
 * When the callout is scheduled then both, the PENDING and ACTIVE flags are
 * set. Once the callout task starts executing the callback the PENDING flag
 * but not the ACTIVE flag is cleared. This state (ACTIVE but NOT PENDING
 * indicates that the callout cannot be safely stopped (or rescheduled) since
 * the user-callback may be executing. If lanIpCallout_trystop() detects this
 * condition then it sets FAILEDSTOP and returns -1. The user may then
 * take action at a later time.
 *
 * The user-callback is responsible for resetting ACTIVE to indicate that
 * it has finished executing. Once ACTIVE is clear the callback MUST NO LONGER
 * access the callout struct.
 */ 

/* Reference a callout either as a callout or a opaque list node              */
typedef union LanIpCalloutRef_ {
	LanIpLstNode    r_node;
	LanIpCallout    r_callout;
} LanIpCalloutRef;

/* Trivial RX buffers aka 'rbuf's (upalign to multiple of 128 to 
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

struct _rbuf_ {
	uint8_t           mem[sizeof(LanIpPacketRec)];
	struct timespec   tstmp;
	IpBscIf           intrf;
	union rbuf_       *next;
	uint8_t          refcnt;
};

typedef union rbuf_ {
	LanIpPacketRec pkt;
	struct _rbuf_  buf;
	uint8_t        raw[ RBUF_ALGN(  sizeof(struct _rbuf_) ) ];
} rbuf_t;

/* Struct describing an ARP table entry                                       */

typedef struct ArpEntryRec_ {
	uint32_t		ipaddr;           /* IPv4 address                         */

	union {                           /* union holds tick-time of last ARPREQ */
		rtems_interval stime;         /* sent (if reply still outstanding) or */
		uint8_t        hwaddr[6];     /* hwaddress if we have a reply. The    */
	}               data;             /* 'sync_resp' ID can be used as a dis- */
                                      /* criminator for the union (hwaddr is  */
                                      /* valid if sync_resp == 0)             */

	rtems_interval	ctime;            /* 'age' of this record (ARPREP seen)   */       
	rtems_id        sync_resp;        /* sem to block for ARPREP arrival      */
} ArpEntryRec, *ArpEntry;

/* Special value for 'ctime' indicating a 'permanent' or 'static' entry       */
#define ARP_PERM	((rtems_interval)(-1))


/* Struct describing a multicast address (MCA)                                */

typedef struct IpBscMcAddrRec_ {
	LanIpLstNodeRec mc_node;   /* list of all MCAs set on an interface        */
	LanIpCalloutRec mc_igmp;   /* callout for scheduling IGMP work            */
	uint32_t        mc_addr;   /* IPv4 multicast address                      */
	uint16_t		mc_sobs;   /* Bitset of sockets having joined this MCA    */
	uint16_t        mc_flags;  /* Flags                                       */
} IpBscMcAddrRec, *IpBscMcAddr;

/* Multiple sockets may subscribe to the same MCA (but each socket only once).
 * The 'mc_sobs' field keeps track of all the sockets that have subscribed to
 * this MCA. Each socket sets (1<<sd) in this bitfield when joining and clears
 * its bit when leaving. The MCA is only really removed when the last socket
 * leaves.
 */

/* State flag indicating whether we should send a IGMP leave message          */
#define MC_FLG_IGMP_LEAVE	(1<<0)

/* 'Special' sd to prevent 224.0.0.1 to be ever deleted                       */
#define MC_ALLSYS_SD        15

#if (NSOCKS) > (MC_ALLSYS_SD)
#error "MC_ALLSYS_SD clashes with normal sockets"
#endif

/* Reference a MCA either as a MCA or a opaque list node                      */
typedef union IpBscMcRef_ {
	LanIpLstNode	r_node;
	IpBscMcAddr     r_mcaddr;
} IpBscMcRef;;

/* Interface statistics counters                                              */
typedef struct IpBscIfStatsRec_ {
	uint32_t    eth_rxfrm;
	uint32_t    eth_protdropped;
	uint32_t    eth_rxdropped;
	uint32_t    eth_txrawfrm;
	uint32_t	arp_nosem;            /* failed to create ARP sync semaphore  */
	uint32_t    arp_gotrep;
	uint32_t    arp_reqme;
	uint32_t    arp_reqother;
	uint32_t    arp_lenopdropped;
	uint32_t    arp_protdropped;
	uint32_t    arp_txreq;
	uint32_t    arp_txrep;
	uint32_t    ip_dstdropped;
	uint32_t    ip_mcdstdropped;
	uint32_t    ip_rxufrm;
	uint32_t    ip_rxmfrm;
	uint32_t    ip_rxbfrm;
	uint32_t    ip_frgdropped;
	uint32_t    ip_lendropped;
	uint32_t    ip_protdropped;
	uint32_t    ip_txmcloopback;
	uint32_t    ip_txrawfrm;
	uint32_t    icmp_hdrdropped;
	uint32_t    icmp_opdropped;
	uint32_t    icmp_rxechoreq;
	uint32_t    icmp_txechorep;
	uint32_t    igmp_hdrdropped;
	uint32_t    igmp_lendropped;
	uint32_t    igmp_csumdropped;
	uint32_t    igmp_rxreportv1;
	uint32_t    igmp_rxreportv2;
	uint32_t    igmp_rxqueryv1;
	uint32_t    igmp_rxqueryv2;
	uint32_t    igmp_txreportv1;
	uint32_t    igmp_txreportv2;
	uint32_t    igmp_txleave;
	uint32_t    udp_hdrdropped;
	uint32_t    udp_sadropped;
	uint32_t	udp_nospcdropped;
	uint32_t    udp_rxfrm;
	uint32_t    udp_rxbytes;
	uint32_t    udp_txfrm;
	uint32_t    udp_txdropped;
	uint32_t    udp_txbytes;
} IpBscIfStatsRec, *IpBscIfStats;

/* NOTES: On class C networks (or equivalent A/B subnets) there will never be
 *        collisions in the cache.
 *
 *        Dont change size (256) w/o adapting hashing algorithm and w/o changing
 *        the 'modulo 256' operation implicit into declaring the 'h' loopcounter
 *        as uint8_t.
 *
 *        When scanning for a free slot the algorithm (arp_find_or_add()) keeps
 *        track of the oldest entry which -- if such an entry is found and no
 *        free slots are available -- is then evicted.
 *
 *        The algorithm needs a special 'null'-element to catch the case where
 *        no oldest entry can be found (because there are only permanent slots
 *        in the neighborhood). The 'ARP_SENTINEL' -- which must be a permanent
 *        entry -- serves this purpose.
 *        The 'sentinel' must have a IP-address which never matches and must be
 *        flagged 'permanent'. The all-zero IP address should be OK.
 */
#define ARP_CACHESZ   256

/* index of sentinel; NOTE: unreachable by ordinary uint8_t hash index        */
#define ARP_SENTINEL (256)            

typedef ArpEntry ArpCache[ARP_CACHESZ + 1];
typedef uint8_t  ArpHash;

/* Don't bother about MSBs; assume we're on a LAN anyways                     */
static inline ArpHash ARPHASH(uint32_t ipaddr)
{
ArpHash h;
#ifdef BYTE_ORDER_UNKOWN
	ipaddr = htohl(ipaddr);
#elif  BYTE_ORDER == LITTLE_ENDIAN
	ipaddr >>= 16;
#endif
	h  = ipaddr;
	h += ipaddr >> 8;
	return h;
}

/* Interface struct                                                           */
typedef struct IpBscIfRec_ {
	void			*drv_p;           /* Opaque handle for the driver         */
	rtems_id		mutx;             /* Mutex protecting mc and arp objects  */
	uint32_t		ipaddr;           /* IPv4 address of this IF              */
	uint32_t		nmask;            /* IPv4 netmask of this IF              */
	LanArpPktRec	arpreq;           /* Prepared header to send ARP requests */
	rbuf_t          *arpbuf;          /* Prepared header to send ARP replies  */
	LHTbl           mctable;          /* MCA hash table (find by IP address)  */
	IpBscMcRef      mclist;           /* Linked list of all MCAs set on IF    */
	IpBscMcAddr     mcallsys;         /* Special 'all-systems' MCA            */
	LanIpCalloutRec mcIgmpV1RtrSeen;  /* Callout for IGMPv1-router-seen state */
	unsigned        mcnum;	          /* Number of MC groups we joined        */
	IpBscIfStatsRec	stats;            /* IF statistics                        */
	ArpCache        arphtbl;          /* Arp hash-table/cache                 */
} IpBscIfRec;

/* Macro for easy access of arp-table (historic reasons)                      */
#define arpcache(pif)   ((pif)->arphtbl)

/* Lock/unlock the IF's ARP cache                                             */
#define ARPLOCK(pif)	mutex_lock((pif)->mutx)
#define ARPUNLOCK(pif) 	mutex_unlk((pif)->mutx)

/* Lock/unlock the IF's MCA hash table, list and other related data structs.  */
#define MCLOCK(pif)		mutex_lock((pif)->mutx)
#define MCUNLOCK(pif) 	mutex_unlk((pif)->mutx)

/* Macro for easy access to arp packet layout                                 */
#define arprep          arpbuf->pkt.p_u.arp_S

/* UDP socket struct                                                          */
typedef struct UdpSockRec_ {
	IpBscIf			  intrf;          /* IF this socket is using              */ 
	volatile int	  port;	          /* My port # (host byte order)          */
	rtems_id		  msgq;           /* RX queue; user dequeues from here    */
	rtems_id		  mutx;           /* Mutex for socket access              */
	unsigned          flags;          /* Flags                                */
	LanUdpPktRec      hdr;            /* A packet header for 'sendto'         */
	volatile unsigned nbytes;         /* # bytes queued (FIONREAD support)    */
	int               mclpbk;         /* Loop-back MC packets sent from here  */
} UdpSockRec, *UdpSock;

/* Flag to indicate that a socket is 'connected' (has a fixed peer)           */
#define FLG_ISCONN	(1<<0)
#define FLG_MCPASS  (1<<1)

/* Macros to lock/unlock a socket's mutex                                     */
#define SOCKLOCK(sck)		mutex_lock((sck)->mutx)
#define SOCKUNLOCK(sck) 	mutex_unlk((sck)->mutx)

/* Message struct; these messages are passed through the socket's msgq        */
typedef struct UdpSockMsgRec_ {
	rbuf_t               *pkt;        /* packet buffer                        */
	int                   len;        /* UDP payload length                   */
} UdpSockMsgRec, *UdpSockMsg;

/**** GLOBAL VARIABLE DEFINITIONS *********************************************/

#ifdef DEBUG
int	lanIpDebug = DEBUG;
#endif

/* Initial buffer pool                                                        */
static rbuf_t		rbufs[NRBUFS]
#ifdef RBUF_ALIGNMENT
__attribute__ ((aligned(RBUF_ALIGNMENT)))
#endif
                            = {{{{{{{0}}}}}}};

/* Static init of rbuf facility                                               */
static int    ravail        = NRBUFS;

/* Stack configuration                                                        */
#ifndef RX_RING_SIZE
#define RX_RING_SIZE 0
#endif
#ifndef TX_RING_SIZE
#define TX_RING_SIZE 0
#endif
static LanIpBscConfigRec lanIpBscCfg = {
	mask:              LANIPCFG_RX_RING | LANIPCFG_TX_RING |
                       LANIPCFG_N_RBUFS | LANIPCFG_SQDEPTH,
	rx_ring_size:      RX_RING_SIZE,
	tx_ring_size:      TX_RING_SIZE,	
	num_rbufs:         NRBUFS,
	rx_queue_depth:    QDEPTH,	
};

/* Counters for available and total number of rbufs                           */
volatile int  lanIpBufAvail = NRBUFS;
int           lanIpBufTotal = NRBUFS;
/* Counter for number of times 'getrbuf()' failed due to lack of rbufs        */
volatile int  lanIpBufGFail = 0;

/* FIXME: only used if we implement some sort of 'bind' operation             */
uint32_t udpSockMcastIfAddr = 0;

/* Free list of malloced rbufs. Note that the buffer pool initially consists of
 * NRBUFS buffers from a static area. At run-time, more buffers can be added
 * using lanIpBscAddBufs() and these are appended here.
 */
static rbuf_t          *frb = 0;

/* Linked list of chunks of malloced rbufs                                    */
static void       *rbuf_mem = 0;

/* Small array of socket structurs                                            */
static UdpSockRec	socks[NSOCKS] = {{0}};
static int         nsocks         = 0;

/* Handler for 'the one and only' interface (ATM only 1 IF supported)         */
static IpBscIf		intrf         = 0;

/* Keep an available entry around                                             */
static ArpEntry	arpScratch        = 0;

/* If this flag is set then for every accepted packet we save or refresh the
 * sender's address in the ARP cache.
 */
int lanIpBscAutoRefreshARP        = 1;

/**** FUNCTION FORWARD DECLARATIONS *******************************************/

static inline void
c_enq(LanIpLstNode *where, LanIpLstNode n);

static inline void
c_deq(LanIpLstNode n);

static int
lanIpCallout_stop(LanIpCallout c);

static int
lanIpCallout_reset(LanIpCallout, uint32_t, void (*)(void*,void*), void *, void *);

static void
lanIpCallout_init(LanIpCallout c);

static rtems_id
lanIpCallout_initialize();

static int
addmca(IpBscIf pif, int sd, uint32_t mcaddr);

static IpBscMcAddr
delmca(IpBscIf pif, IpBscMcAddr mca, int sd);

static void
destroymca(IpBscMcAddr mca);

static void
igmp_v1timeo(void *arg0, void *arg1);

static void
processArp(IpBscIf pif, rbuf_t *buf_p);

static void
processIcmp(IpBscIf pif, rbuf_t *buf_p, int len);

static void
processIgmp(IpBscIf pif, rbuf_t *buf_p, int len);

static void
igmp_timeo(void *arg0, void *arg1);

static IpBscMcAddr
igmp_send_msg(IpBscIf pif, uint32_t gaddr, uint8_t type);

static void
arpTearDown(IpBscIf pif, int perm_also);

static void
lpWorker(void *arg);

/**** GENERAL INLINE FUNCTIONS ************************************************/

/* Specially crafted inlines for byteswapping constants -- these should be
 * optimized away by the compiler which should byte-swap the arguments at
 * compile-time. Note that gcc cannot optimize normal swapping-operations (e.g.,
 * 'htons') away since the real work may be done by inline-assembly.
 */

/* Declare routines with 'error' attribute. The goal here is to force gcc to
 * fail if it ever emits code to actually call these routines, i.e., if they
 * are not inlined.
 */
static inline uint16_t swapc16(uint16_t v)
__attribute__(( always_inline, error("swapc16 not inlined -- compile with at least -O1") ))
;

static inline uint32_t swapc32(uint32_t v)
__attribute__(( always_inline, error("swapc32 not inlined -- compile with at least -O1") ))
;

/* Declare another helper which triggers failure if gcc cannot optimize the
 * helper away.
 */
extern void fail_not_const()
__attribute__(( error("swapc argument not constant") ));

/* Yet another trick. We declare a routine that is nowhere defined so that
 * we get a linker error if a call is not optimized away.
 * 
 * E.g., we can make sure that gcc resolves the conditions in a statement
 *
 *    if ( constant == A )
 *      __do_something();
 *    else if ( constant == B )
 *      __do_something_else();
 *    else
 *      __should_be_optimized_away();
 *
 * at compile-time. If gcc can determine that 'constant==B' then the whole
 * 'if'-construct is replaced by a call to __do_something_else() and no
 * attempt to link __should_be_optimized_away() is made.
 * If OTOH gcc is unable to resolve the conditions then it must emit a call
 * to __should_be_optimized_away() which will then trigger a linker-error.
 */
extern int __should_be_optimized_away();

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

/* Macros (in analogy to 'htons/htonl' etc.) but these assume constant args.  */
#define htonsc(v) swapc16(v)
#define ntohsc(v) swapc16(v)

#define htonlc(v) swapc32(v)
#define ntohlc(v) swapc32(v)


/* Convert IP multicast address into ethernet multicast address               */
static inline void
ipmc2ethermc(uint32_t ipaddr, uint8_t *enaddr)
{
uint32_t tmp;

	enaddr[0] = 0x01;
	enaddr[1] = 0x00;

	tmp = (ipaddr & htonlc(0x007fffff)) | htonlc(0x5e000000);

	memcpy( enaddr + 2, &tmp , 4);
}

/* Check if 'ipaddr' is a multicast address to which 'pif' is subscribed      */
static inline int
mcListener(IpBscIf pif, uint32_t ipaddr)
{
	return ISMCST( ipaddr ) && lhtblFind( pif->mctable, ipaddr );
}

static inline IpBscMcAddr
nxtmca (IpBscMcAddr mca)
{
	return (IpBscMcAddr)mca->mc_node.l_next;
}


/**** SEMAPHORE CONVENIENCE WRAPPERS ******************************************/

static const char *
nullchk(const char *s)
{
static const char *const nobody = "<nobody>";

	return  s ? s : nobody;
}

/* Convert string into RTEMS name                                             */
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

#define SEM_SYNC 1  /* Binary semaphore for synchronization (initially emtpy) */
#define SEM_MUTX 2  /* Nesting mutex (initially full)                         */
#define SEM_CNTG 3  /* General-purpose, counting mutex                        */

static const char *semtype[4]=
{
	"",
	"SYNC",
	"MUTEX",
	"COUNTING"
};

/* Create any type of semaphore                                               */

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

		case SEM_SYNC:
			atts = RTEMS_SIMPLE_BINARY_SEMAPHORE;
		break;

		case SEM_CNTG:
			atts = RTEMS_COUNTING_SEMAPHORE;
		break;
	}

	if ( SEM_MUTX == type ) {
		atts |= RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY;
	}

	sc = rtems_semaphore_create(
			str2name(name),
			count,
			atts,
			0,
			&rval);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "creating %s semaphore %s failed\n", semtype[type], nullchk(name));
		rval = 0;
	}
	return rval;
}

/* Create binary semaphore or mutex                                           */

static rtems_id
bsem_create(char *name, int type)
{
	return sem_create(name, type, SEM_SYNC == type ? 0 : 1);
}

/* Lock and unlock mutex asserting result                                     */

static inline void
mutex_lock(rtems_id m)
{
int rtems_semaphore_obtain_SUCCESSFUL =
	RTEMS_SUCCESSFUL == rtems_semaphore_obtain(m, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
	assert( rtems_semaphore_obtain_SUCCESSFUL );
}

static inline void
mutex_unlk(rtems_id m)
{
int rtems_semaphore_release_SUCCESSFUL =
	RTEMS_SUCCESSFUL == rtems_semaphore_release(m);
	assert( rtems_semaphore_release_SUCCESSFUL );
}

/**** TASKING CONVENIENCE WRAPPERS ********************************************/

#define TASK_JOIN_NPAD	RTEMS_NOTEPAD_0

#define KILL_BY_EVENT	1
#define KILL_BY_SEMA	2

typedef union task_killer_ {
	rtems_event_set	kill_event;	
	rtems_id		kill_resource;	
} task_killer;

static void
task_init( rtems_id tid );

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

/* Unfortunately, the RTEMS classic API doesn't provide a way to
 * synchronize with a terminating task.
 *
 * These routines provide a workaround:
 *   1) at task creation or start 'task_init(target_tid)' is called
 *      to initialize a 'target-task' notepad to zero.
 *
 *   2) 'killer-task' calls 'task_pseudojoin_prepare(target_tid)'
 *      which creates a synchronization semaphore and installs
 *      it to the 'target-task's notepad. 
 *
 *   3) 'killer-task' signals the 'target-task' that it should
 *      terminate.
 *
 *   4/5) 'target-task' cleans up and eventually calls 'task_leave()'
 *      which releases the synchronization semaphore and suspends
 *      the 'target_task'.
 *
 *   5/4) 'killer-task' calls 'task_pseudojoin_wrapup(target_tid, sync_sem)'
 *      waiting for the synchronization semaphore and then deleting the
 *      'target-task' and semaphore.
 *
 *   NOTES: The order of steps 4/5 and 5/4 is subject to race conditions
 *      which is why we synchronize in the first place.
 *
 *      Target task suspends instead of deleting itself which would
 *      require it to run and this is not guaranteed. Module could
 *      theoretially be unloaded before this happens. Therefore deleting
 *      the 'target-task' is left to the 'killer-task'.
 */

static void
task_init( rtems_id tid )
{
	rtems_task_set_note( tid, TASK_JOIN_NPAD, 0 );
}

static void
task_leave()
{
rtems_id sync_sem;

	rtems_task_get_note( RTEMS_SELF, TASK_JOIN_NPAD, (uint32_t*) & sync_sem );
	rtems_semaphore_release( sync_sem );
	rtems_task_suspend(RTEMS_SELF);
}

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

/**** LIST OPERATION PRIMITIVES ***********************************************/

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

/**** CALLOUT FACILITY ********************************************************/

/* The callout facility implements a way to schedule execution of a user
 * callback at some defined time in the future by a low-priority task.
 */

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

/* The callout mutex is held by the callout task during execution of the user
 * callbacks.
 * It provides a means for synchronizing callout cancellation. The user may
 * either call lanIpCallout_stop() which acquires the callout-mutex before
 * cancelling a callout or if this bears the risk of deadlock then the user
 * may call lanIpCallout_trystop(), release any locks that could be part of
 * a deadlock cycle and then -- if the status returned by the 'trystop'
 * operation indicates that the cancellation failed -- acquire and release
 * the callout mutex which synchronizes with the callout task.
 */
#define COLOCK()		mutex_lock(callout_mtx)
#define COUNLOCK() 		mutex_unlk(callout_mtx)

/* Helpers to extract info from callouts                                      */

static inline LanIpCallout
lanIpCalloutNext(LanIpCallout c)
{
	return (LanIpCallout)c->c_node.l_next;
}

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


/* We cannot stop a callout that's in progress                                */
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
	memset(c,0,sizeof(*c));	
}

/* Initialize callout facility [networking must have been initialized already] */
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

/* Cleanup and release resources                                              */
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

/**** GENERIC HELPER FUNCTIONS ************************************************/

/* Compute IP checksum (not optimized for speed; we only need this for non-
 * critical applications, e.g., when doing IGMP or ICMP echo-replies
 */
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

/* A bad but simple random number generator; should be good enough for IGMP
 * (Posix.1-2001). Returns a pseudo-random number in the range of 0..65535.
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

/* Print an ethernet address to 'file'                                        */
static void
prether(FILE *f, const unsigned char *ea)
{
int i;
	for (i=0; i<5; i++)
		fprintf(f,"%02X:",*ea++);
	fprintf(f,"%02X",*ea);
}

/* Print an IPv4 address to 'file' ('dot'-notation)                           */
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

/* 'Print' an IPv4 address to string buffer                                   */
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

/* Fill-in our source address MAC and IP address, compute IP header checksum  */

static inline void
fillinSrcCsumIp(IpBscIf ifc, LanIpPart buf_p)
{
	memcpy(buf_p->ll.src, &ifc->arpreq.ll.src, sizeof(buf_p->ll.src));
	buf_p->ip.src  = ifc->ipaddr;

#if 0
	/* alias-rule is quite nasty. Even though the 'IpHeaderRec' is
	 * given the 'may_alias' attribute I still found that gcc
	 * optimizes initialization of the checksum away.
	 */
    buf_p->ip.csum = 0;
#else
	memset( &buf_p->ip.csum, 0, sizeof(buf_p->ip.csum) );
#endif
	buf_p->ip.csum = in_cksum_hdr((void*)&buf_p->ip);
}

/* Fill-in our source addresses (MAC, IP, UDP SPORT), IP header checksum      */
static inline void
fillinSrcCsumUdp(IpBscIf ifc, LanUdpPkt buf_p, int port)
{
	buf_p->udp.sport = htons(port);
	buf_p->udp.csum = 0;
	fillinSrcCsumIp(ifc, &buf_p->ip_part);
}

/* Copy IP and MAC source addresses to destination members                    */
static inline void
src2dstIp(LanIpPart p)
{
	memcpy(p->ll.dst, p->ll.src, sizeof(p->ll.dst));
	p->ip.dst = p->ip.src;
}

/* Copy IP, MAC and UDP source addresses and port to destination members      */
static inline void
src2dstUdp(LanUdpPkt p)
{
	src2dstIp(&p->ip_part);
	p->udp.dport = p->udp.sport;
}

/* Check IPV4 version/header-length byte agains 'expected' value              */
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

static inline int ms2ticks(int ms)
{
	if ( ms > 0 ) {
		rtems_interval rate;
		rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &rate);
		if ( ms > 50000 ) {
			ms /= 1000;
			ms *= rate;
		} else {
			ms *= rate;
			ms /= 1000;
		}
		if ( 0 == ms ) {
			ms = 1;
		}
	}
	return ms;
}

#warning 'debugging code still present'
void *
getlhtbl(IpBscIf pif)
{
	return pif ? pif->mctable : 0;
}

/**** RBUF MANAGEMENT *********************************************************/

/* Obtain a new buffer from pool                                              */

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
		} else {
			lanIpBufGFail++;
			rtems_interrupt_enable(key);
			return 0;
		}
	}
	rval->buf.refcnt++;
	rval->buf.next = 0;
	if ( lanIpBufAvail )
		lanIpBufAvail--;
	rtems_interrupt_enable(key);

	rval->buf.intrf = 0;

	return rval;
}

/* Decrement reference count and when it drops to 0 release buffer to pool    */
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

/* Increment reference count of a buffer                                      */
static void refrbuf(rbuf_t *b)
{
rtems_interrupt_level key;

	rtems_interrupt_disable(key);
		b->buf.refcnt++;
	rtems_interrupt_enable(key);
}

/* Allocate more buffers and add to pool                                      */
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
		lanIpBscCfg.num_rbufs = lanIpBufTotal;
		/* chain into list of malloced chunks */
		*(void**)mem     = rbuf_mem;
		rbuf_mem         = mem;
	rtems_interrupt_enable(key);

	return 0;	
}

/* Release all malloced rbuf memory back to malloc-heap                       */
static void
freeBufMem()
{
void *p;
	while ( (p = rbuf_mem) ) {
		rbuf_mem = *(void**)rbuf_mem;
		free(p);
	}
}

/* Inofficial / non-public helpers for profiling                              */

void
lanIpBscGetBufTstmp(rbuf_t *p_buf, struct timespec *pts)
{
	*pts = p_buf->buf.tstmp;	
}

void
lanIpBscSetBufTstmp(rbuf_t *p_buf, struct timespec *pts)
{
	if ( pts ) {
		p_buf->buf.tstmp = *pts;
	} else {
		rtems_clock_get_uptime( &p_buf->buf.tstmp );
	}
}

/**** LOW-PRIORITY JOB MANAGEMENT *********************************************/

static rbuf_t * volatile workHead = 0;
static rbuf_t * volatile workTail = 0;
static rtems_id          workSema = 0;
static rtems_id          workTask = 0;

/* Enqueue an RBUF to the list of low-priority jobs                           */
static void
scheduleLpWork(IpBscIf pif, rbuf_t *buf_p)
{
rtems_interrupt_level l;

	buf_p->buf.intrf = pif;
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

/* Block for work and dequeue an RBUF from the list of low-priority jobs      */
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

/**** ARP CACHE ACCESS AND OTHER ARP RELATED CODE *****************************/

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
} __attribute__ ((may_alias)); /* spa/tpa are 2-byte aligned */

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

/* Dump an ARP cache entry                                                    */
static void
prarp(int ind, FILE *f, ArpEntry e)
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

	fprintf(f,"%*s%s: ", ind, "", ipbuf);
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


/* Allocate and free an arp cache entry                                       */
static ArpEntry
arp_allocentry()
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

/* Release and arp cache entry but keep it around instead of freeing          */
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

/* Lookup 'ipaddr' in the ARP cache and return ArpEntry if successful. The hash
 * has already been computed by the caller and is passed in 'h'. If 'ipaddr'
 * is not found in the cache then the entry '*p_newe' is added to its place
 * (but is not necessarily valid/looked-up yet). If necessary and possible
 * (if there are too many permanent/static entries present then addition of the
 * new entry may not be possible) then the oldest entry is evicted and replaced
 * by '*p_newe'.
 *
 * RETURNS: Valid entry if found, evicted entry (with ipaddr member set to 0)
 *          if replacing an old entry was necessary or entry that could not be
 *          evicted (with nonzero ipaddr member) if eviction was necessary but
 *          not possible. Finally, if the new entry was added w/o evicting an
 *          old one then the new entry is returned.
 *          I.e., unless the operation failed the entry for 'ipaddr' is returned
 *          which may either be valid (existing) or invalid (not looked-up yet).
 *
 *          If the new entry '*p_newe' was added the '*p_newe' is set to NULL.
 */

static ArpEntry
arp_find_or_add(IpBscIf pif, ArpHash h, uint32_t ipaddr, ArpEntry *p_newe)
{
int         i, oh, empty;
ArpEntry    found;

	for ( i=0, oh = ARP_SENTINEL, empty=-1; i<CACHE_OVERLAP; i++, h++ ) {
		if ( ! (found = arpcache(pif)[h]) ) {
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
		if ( (uint32_t)found->ctime < (uint32_t)arpcache(pif)[oh]->ctime ) {
			oh     = h;
		}
	}

	if ( empty<0 ) {
		found = arpcache(pif)[oh];
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
			prarp(0, 0, arpcache(pif)[oh]);
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
		found       = arpcache(pif)[h] = *p_newe;
		*p_newe     = 0;
	}
	return found;
}

/* Lookup 'ipaddr' in the ARP cache and copy associated MAC address to '*enaddr'
 * buffer if the cache entry is valid (holds a looked-up MAC address). If
 * the entry is invalid (holds no MAC address) then the entry is removed.
 *
 * RETURNS: 0 on success (valid entry found), nonzero if either nothing
 *          was found or an invalid entry was removed.
 */

static int
arp_find_or_del(IpBscIf pif, ArpHash h, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry found = 0;
ArpEntry rval;
int      i;

	ARPLOCK(pif);
	for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
		if ( !(rval = arpcache(pif)[h]) )
			continue;
		if ( ipaddr == rval->ipaddr ) {
			if ( ! rval->sync_resp ) {
#ifdef DEBUG
				if ( (lanIpDebug & DEBUG_ARP) ) {
					printf("arpLookup(): last-chance success for entry #%i\n",h);
					prarp(0, 0, rval);
				}
#endif
				/* Wow - we have something */
				memcpy(enaddr, rval->data.hwaddr, 6);
				ARPUNLOCK(pif);
				return 0;
			}
#ifdef DEBUG
			if ( (lanIpDebug & DEBUG_ARP) ) {
				printf("arpLookup(): deleting placeholder entry #%i\n",h);
				prarp(0, 0, rval);
			}
#endif
			/* Still no reply */
			arpcache(pif)[h] = 0;
			found = rval;
			break;
		}
	}
	found = arp_putscratch( found );
	ARPUNLOCK(pif);
	arp_destroyentry(found);
	return -1;
}

/* Atomically unlock the ARP mutex and acquire 'sync_sem'.
 * 
 * RETURNS: RTEMS status code of 'rtems_semaphore_obtain(sync_sem)'
 *          operation.
 */
static rtems_status_code
ARPUNLOCK_block(IpBscIf pif, rtems_id sync_sem)
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

	ARPUNLOCK(pif);
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

/* Lookup 'ipaddr' in the arp cache (using pre-computed hash 'h') and take the
 * following action:
 *
 * a) if a valid entry is found then copy associated MAC address to '*enaddr'
 *    and RETURN ARPSEM_UNNEEDED
 * b) if no entry is found then try to create a new entry but return -ENOSPC
 *    on failure.
 *    Create a new synchronization semaphore and attach to new entry but return
 *    -ENOMEM if this fails. On success RETURN ARPSEM_OLD and update the
 *    'request sent timestamp' ('entry->data.stime').
 * c) if an invalid entry is found (no MAC address available yet) then look
 *    at the time when the last ARP request was sent (entry->data.stime). If
 *    this was less than ARP_TIMEOUT_TICKS ago then RETURN ARPSEM_FRESH otherwise
 *    update 'entry->data.stime' and RETURN ARPSEM_OLD.
 *
 * RETURNS: Return codes < 0 indicate an error condition (-ENOSPC if no entry 
 *          could be added because there are too many permanent entries or 
 *          -ENOMEM if semaphore creation failed.
 *
 *          ARPSEM_UNNEEDED: no semaphore was created because a valid ARP cache
 *                           exists; associated MAC addr. was copied to *enaddr
 *
 *          ARPSEM_OLD:      semaphore exists or was created; need to send a
 *                           ARP request and block on semaphore for reply to
 *                           arrive.
 *
 *          ARPSEM_FRESH:    semaphore exists and another thread has recently
 *                           sent a request; just block on semaphore for reply
 *                           to arrive.
 */

static int
arpCreateSyncsem(IpBscIf pif, ArpHash h, uint32_t ipaddr, uint8_t *enaddr)
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
		if ( ISBCST( ipaddr, pif->nmask ) || ISMCST( ipaddr ) )
			return SOME_ERROR_STATUS;
#endif

		h = ARPHASH(ipaddr);

		ARPLOCK(pif);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pif);
			newe = arp_allocentry();
			ARPLOCK(pif);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(pif, h, ipaddr, &newe);

		if ( ipaddr == rval->ipaddr ) {
			if ( rval->sync_resp ) {
				/* Have it already */
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("arpCreateSyncsem(): sem already exists for entry #%i\n",h);
					prarp(0, 0, rval);
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
					prarp(0, 0, rval);
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
					if ( !(rval = arpcache(pif)[h]) )
						continue;
					if ( ipaddr == rval->ipaddr ) {
						newe            = arpcache(pif)[h];
						arpcache(pif)[h] = 0;
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
				prarp(0, 0, rval);
			}
#endif
			err = ARPSEM_OLD;
		}

egress:
		newe = arp_putscratch(newe);
		ARPUNLOCK(pif);
		arp_destroyentry(newe);

		return err;
}

#if 0
static rtems_status_code
arpAwaitReply(IpBscIf pif, uint32_t ipaddr, uint8_t *enaddr)
{
ArpEntry          rval;
ArpEntry          newe;
ArpHash           h;
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
		if ( ISBCST( ipaddr, pif->nmask ) || ISMCST( ipaddr ) )
			return SOME_ERROR_STATUS;
#endif

		h = ARPHASH(ipaddr);

		ARPLOCK(pif);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pif);
			newe = arp_allocentry();
			ARPLOCK(pif);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(pif, h, ipaddr, &newe);

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
					prarp(0, 0, rval);
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
					if ( !(rval = arpcache(pif)[h]) )
						continue;
					if ( ipaddr == rval->ipaddr ) {
						newe            = arpcache(pif)[h];
						arpcache(pif)[h] = 0;
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
			prarp(0, 0, rval);
			printf("Blocking for reply...\n");
		}
#endif

		/* cleanup if necessary */
		newe = arp_putscratch(newe);

		ARPUNLOCK_block(pif, rval->sync_resp);

		arp_destroyentry(newe);

		/* MUST be timedout, unsatisfied (due to flush operation) or
		 * object_was_deleted.
		 */
		assert ( RTEMS_SUCCESSFUL != sc );

		return sc;

egress:
		newe = arp_putscratch(newe);
		ARPUNLOCK(pif);
		arp_destroyentry(newe);

		return sc;
}
#endif

/* Lookup 'ipaddr' in the ARP cache and store associated MAC addr. in '*enaddr'.
 *
 * If the 'cacheonly' argument is nonzero then the routine fails if no valid
 * entry in the ARP cache is found.
 *
 * If the 'cacheonly' argument is zero then the routine performs an ARP lookup
 * on the network, if necessary, which possibly is a slow operation which may
 * time-out unsuccessfully.
 *
 * The user may pass a NULL 'enaddr' pointer to instruct the routine to un-
 * conditionally send an ARP request for 'ipaddr' (provided that 'ipaddr' is 
 * not a broadcast or multicast address). If 'enaddr' == NULL then the routine
 * never blocks for a reply.
 *
 * RETURNS: zero on success, '- errno' on failure.
 *
 * NOTE: Routine can also be called if 'ipaddr' is an IPv4 multicast address.
 *       In this case, the corresponding ethernet MC address is copied to
 *       '*enaddr'.
 *       If 'ipaddr' is a broadcast address then the ethernet broadcast address
 *       is copied to '*enaddr'.
 *       In both cases an ARP lookup over the network is never required.
 */

int
arpLookup(IpBscIf pif, uint32_t ipaddr, uint8_t *enaddr, int cacheonly)
{
ArpEntry          rval;
int               i,st;
ArpHash           hh;
ArpHash           h;
rtems_status_code sc;
int               attempts;
int               err = -ENOTCONN;

		if ( ISBCST(ipaddr, pif->nmask) ) {
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

		if ( (ipaddr & pif->nmask) != (pif->ipaddr & pif->nmask) ) {
			return -ENETUNREACH;
		}

		if ( !enaddr ) {
			/* They just want to send a lookup. If we (eventually)
			 * receive an answer then a cache entry will be created
			 * or updated (asynchronously).
			 */
			pif->stats.arp_txreq++;
			NETDRV_ATOMIC_SEND_ARPREQ(pif, ipaddr);
			return 0;
		}


		h = ARPHASH(ipaddr);

		attempts = 0;

		/* We may probe the cache one more time than we
		 * send requests so that we may find a reply
		 * to the last send attempt.
		 */
		while ( attempts < ARP_SEND_RETRY + 1 ) {

			ARPLOCK(pif);
			for ( i = 0, hh=h; i<CACHE_OVERLAP; i++, hh++ ) {

				if ( !(rval = arpcache(pif)[hh]) )
					continue;

				if ( ipaddr == rval->ipaddr ) {
					if ( ! rval->sync_resp ) {
						/* Done, found valid cache entry */
						memcpy(enaddr, rval->data.hwaddr, 6);
						ARPUNLOCK(pif);
						return 0;
					} else {
						if ( cacheonly ) {
							ARPUNLOCK(pif);
							return -ENOTCONN;
						}

						/* block for reply */
						sc = ARPUNLOCK_block(pif, rval->sync_resp);

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
			ARPUNLOCK(pif);

			if ( cacheonly )
				return -ENOTCONN;

proceed_unlocked:

			if ( attempts >= ARP_SEND_RETRY )
				break;

			st = arpCreateSyncsem(pif, h, ipaddr, enaddr);

			if        ( ARPSEM_OLD == st ) {
				/* must do a new lookup */
				pif->stats.arp_txreq++;
				NETDRV_ATOMIC_SEND_ARPREQ(pif, ipaddr);
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
				pif->stats.arp_nosem++;
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
		if ( 0 == arp_find_or_del(pif, h, ipaddr, enaddr) )
			return 0;
		
		return err;
}

/* Store an IPv4 / MAC address pair in the ARP cache. If the 'perm' argument
 * is nonzero then the entry is marked 'permanent' or 'static' which means
 * that it is never evicted from the cache.
 * Cache entries may be evicted (if they are not 'permanent') if space is needed
 * for a new entry or (if the 'arpScavenger' is run in the background) if an
 * entry becomes too old (no lookup or auto-update has been done for some
 * 'max-age' time).
 *
 * RETURNS: zero on success or '- errno' on failure.
 */
int
arpPutEntry(IpBscIf pif, uint32_t ipaddr, uint8_t *enaddr, int perm)
{
ArpEntry rval;
ArpEntry newe;
ArpHash  h;
int      err = -ENOSPC;
#ifdef DEBUG
const char *dbgstr = 0;
#endif

		/* Silently ignore broadcast + multicast addresses */
		if ( ISBCST( ipaddr, pif->nmask ) || ISMCST( ipaddr ) )
			return 0;

		h = ARPHASH(ipaddr);

		ARPLOCK(pif);
		if ( !(newe = arpScratch) ) {
			ARPUNLOCK(pif);
			newe = arp_allocentry();
			ARPLOCK(pif);
		} else {
			/* we took over the scratch entry */
			arpScratch = 0;
		}

		rval = arp_find_or_add(pif, h, ipaddr, &newe);

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
			prarp(0, 0, rval);
		}
#endif

		err = 0;

egress:
		newe = arp_putscratch( newe );
		ARPUNLOCK(pif);
		arp_destroyentry(newe);

		return err;
}

/* Remove 'ipaddr' from ARP cache.                                            */
void
arpDelEntry(IpBscIf pif, uint32_t ipaddr)
{
ArpEntry rval, found = 0;
int      i;
ArpHash  h;

		h = ARPHASH(ipaddr);

		ARPLOCK(pif);
		for ( i = 0; i<CACHE_OVERLAP; i++, h++ ) {
				if ( !(rval = arpcache(pif)[h]) )
					continue;
				if ( ipaddr == rval->ipaddr ) {
					arpcache(pif)[h] = 0;
					found           = rval;
					break;
				}
		}

		found = arp_putscratch( found );
		ARPUNLOCK(pif);
		arp_destroyentry(found);
}

/* Unprotected version of 'arpFlushCache()' to be used when cleaning up after
 * interface object has been destroyed.
 */
static void
arpTearDown(IpBscIf pif, int perm_also)
{
int i;

	for ( i=0; i<ARP_CACHESZ; i++ ) {
		if ( perm_also || (arpcache(pif)[i] && ARP_PERM != arpcache(pif)[i]->ctime) ) {
			arp_destroyentry(arpcache(pif)[i]);
			arpcache(pif)[i] = 0;
		}
	}

	arp_destroyentry(arpScratch);
	arpScratch = 0;
}

/* Remove all entries from the ARP cache. Optionally ('perm_also' == 0) only
 * non-permanent entries are flushed/removed.
 */
void
arpFlushCache(IpBscIf pif, int perm_also)
{
	/* cleanup everything */

	ARPLOCK(pif);

	arpTearDown(pif, perm_also);

	ARPUNLOCK(pif);
}

/* Print info for all ARP cache entries to file 'f' (stdout is used if NULL)  */

static void
arpDumpCacheIndented(int ind, IpBscIf pif, FILE *f)
{
ArpEntryRec           abuf;
int                   i;

	if ( !f )
		f = stdout;

	for ( i=0; i<ARP_CACHESZ; i++ ) {
		if ( (volatile ArpEntry) arpcache(pif)[i] ) {

			ARPLOCK( pif );

				/* it might have gone... */
				if ( (volatile ArpEntry) arpcache(pif)[i] ) {
					abuf = *arpcache(pif)[i];

					ARPUNLOCK( pif );

					fprintf(f,"%*sARP cache entry #%i: ",ind,"",i);
					prarp(ind, f, &abuf);
				} else {
					ARPUNLOCK( pif );
				}
		}
	}
}

void
arpDumpCache(IpBscIf pif, FILE *f)
{
	arpDumpCacheIndented(0, pif, f);
}

/* Scan the ARP cache for 'nloops' times every 'period' number of OS ticks for
 * non-permanent entries which are older than 'maxage' seconds and evict such
 * entries from the cache.
 * 
 * NOTES: The user is responsible for spawning a task executing the scavenger.
 *        It is NOT executed by default.
 *
 *        If 'nloops' is less than zero then the algorithm is executed for an
 *        indefinite number of times.
 */

void
arpScavenger(IpBscIf pif, rtems_interval maxage, rtems_interval period, int nloops)
{
rtems_interval        ancient,sec,now;
int                   i;
ArpEntry              e;

	while ( 1 ) {
		/* calculate oldest acceptable 'ctime' */
		rtems_clock_get(RTEMS_CLOCK_GET_SECONDS_SINCE_EPOCH, &now);
		ancient = now - maxage;

		for ( i=0; i<ARP_CACHESZ; i++ ) {
			if ( ! (volatile ArpEntry) arpcache(pif)[i] ) 
				continue;

			ARPLOCK( pif );
			/* have to check again from within protected section */
			if ( !(e = (volatile ArpEntry)arpcache(pif)[i]) ) {
				ARPUNLOCK( pif );
				continue;
			}
			if ( (uint32_t)e->ctime < (uint32_t)ancient ) {
				/* evict */
				arpcache(pif)[i] = 0;
				e = arp_putscratch(e);

#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("Evicting entry #%i from ARP cache: ",i);
					/* Note the race condition, arpScratch could have
					 * changed but for debugging that's good enough...
					 */
					if ( e )
						prarp(0, 0, e);
					else if ( arpScratch )			/* e has gone to scratch   */
						prarp(0, 0, arpScratch);	/* probably hasn't changed */
					else
						printf("\n");
				}
#endif
				ARPUNLOCK(pif);

				arp_destroyentry(e);

			} else {
#ifdef DEBUG
				if ( lanIpDebug & DEBUG_ARP ) {
					printf("ARP cache entry #%i: ",i);
					if (arpcache(pif)[i])
						prarp(0, 0, arpcache(pif)[i]);
					else
						printf("\n");
				}
#endif
				ARPUNLOCK(pif);
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

/**** IGMP V2 PROTOCOL IMPLEMENTATION *****************************************/

/* Inline helpers to determine the state of a MCA                             */

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
LanIgmpV2Pkt pigmp = &lpkt_igmpv2hdr( &buf_p->pkt );
uint32_t     opt;
uint16_t     s1, s2;
uint32_t     daddr;
LanIpPart    ipp   = &pigmp->ip_part;

	if( IGMP_TYPE_LEAVE == type ) {
		daddr = htonlc(IP_GRP_ALL_ROUTERS);
	} else {
		daddr = gaddr;
		type  = igmp_v1_rtr_seen( intrf ) ? IGMP_TYPE_REPORT_V1 : IGMP_TYPE_REPORT_V2;
	}


	/* Prepare Ethernet Header -- src address is filled-in later */
	ipmc2ethermc(daddr, ipp->ll.dst);
	ipp->ll.type = htonsc(0x800); /* IP */

	ipp->ip.vhl     = 0x46;          /* v4, header + rtr-alert option */
	ipp->ip.tos     = 0x00;
	ipp->ip.len     = htonsc(sizeof(IpHeaderRec) + 4 + sizeof(IgmpV2HeaderRec));
	ipp->ip.id      = htonsc(0);
	ipp->ip.off     = htonsc(0x4000); /* set DF flag */
	ipp->ip.ttl     = 1;
	ipp->ip.prot    = IP_PROT_IGMP;
	/* ipp->ip.csum   filled in later */
	/* ipp->ip.src    filled in later */
	ipp->ip.dst     = daddr;
	ipp->ip.opts[0] = opt = htonlc(IP_OPT_ROUTER_ALERT);

	/* Set source IP and ethernet addresses and compute
	 * header checksum (w/o router-alert option)
	 */
	fillinSrcCsumIp( intrf, ipp );

	/* Adjust the checksum for the router alert option */
	s1                = (opt>>16);
	s2                =  opt;
	if ( (s1 += s2) < s2 )
		s1++;
	
	s2                = ~ipp->ip.csum;

	if ( (s1 += s2) < s2 )
		s1++;

	ipp->ip.csum    = ~s1;

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
static IpBscMcAddr
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

	switch ( type ) {
		case IGMP_TYPE_REPORT_V1:
			intrf->stats.igmp_txreportv1++;
		break;
		case IGMP_TYPE_REPORT_V2:
			intrf->stats.igmp_txreportv2++;
		break;
		case IGMP_TYPE_LEAVE:
			intrf->stats.igmp_txleave++;
		break;
		default:
		break;
	}

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
			/* adjust raw packet counter */
			intrf->stats.eth_txrawfrm--;
			lanIpBscSendBufRaw(intrf, &buf_p->pkt, sizeof(lpkt_igmpv2hdr(&buf_p->pkt)));
			buf_p = 0;
		}
	}

	relrbuf(buf_p);

	return mca;
}


/* IGMP timer callbacks; executed from the context of the 'callout' task...   */

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

/**** HIGH PRIORITY PROTOCOL HANDLING *****************************************/

/* These routines execute in the context of the high-priority driver task when
 * new packets arrive.
 * They are responsible for
 *   - defer low-priority work (ARP, ICMP, IGMP) to low-priority worker task
 *   - dispatching UDP packets to user sockets
 */

/* Handle ARP requests and reply packets we receive                           */

static int
handleArp(rbuf_t **ppbuf, IpBscIf pif)
{
rbuf_t		*p    = *ppbuf;
IpArpRec	*pipa = &lpkt_arp(&p->pkt);
uint32_t    xx;


	 /* 0x0001 == Ethernet, 0x0800 == IP */
	NETDRV_READ_INCREMENTAL(pif, pipa, 8);
	if ( htonlc(0x00010800) != *(uint32_a_t*)pipa ) {
		pif->stats.arp_protdropped++;
		return 8;
	}

	xx = * ( (uint32_a_t *) pipa + 1 );

	/* 0x06 hw addr len, 0x04 proto len, 0x0001 ARP REQUEST */
	/* 0x06 hw addr len, 0x04 proto len, 0x0002 ARP REPLY   */
	if ( htonlc(0x06040001) != xx && htonlc(0x06040002) != xx ) {
		pif->stats.arp_lenopdropped++;
		return 8;
	}

	/* Fill rest of ARP packet            */
	NETDRV_READ_INCREMENTAL(pif, pipa->sha, 5*4);

	scheduleLpWork(pif, p);
	*ppbuf = 0; 

	return sizeof(*pipa);
}

/* Refresh or create a new ARP entry. We mock up a fake UDP packet and schedule
 * the real work (arpPutEntry()) for the low-priority worker task.
 */

static void
scheduleRefreshArp(IpBscIf pif, LanUdpPkt pudp)
{
rbuf_t       *nbuf;
	if ( (nbuf = getrbuf()) ) { 
		/* fake up a new buffer; copy just enough info for the
		 * low-priority worker...
		 *
		 * Unfortunately, we can't just refrbuf() and pass the
		 * same buffer on since the user might receive and
		 * modify it (user 'owns' received buffer).
		 */

		/* It's IPv4 */
		lpkt_eth( &nbuf->pkt ).type = htonsc(0x800);

		/* It's UDP */
		lpkt_ip( &nbuf->pkt ).prot  = IP_PROT_UDP;

		/* Just because the length is read by lpWorker... */
		lpkt_ip( &nbuf->pkt ).len   = pudp->ip_part.ip.len;

		/* Copy the relevant addresses */
		lpkt_ip( &nbuf->pkt ).src   = pudp->ip_part.ip.src;
		memcpy(
			lpkt_eth( &nbuf->pkt ).src,
			pudp->ip_part.ll.src,
			sizeof(lpkt_eth( &nbuf->pkt ).src)
		);

		/* There it goes */
		scheduleLpWork(pif, nbuf);
	}
}

/* Handle IPv4 protocol (ICMP, IGMP, UDP)                                     */

static int
handleIP(rbuf_t **ppbuf, IpBscIf pif, int loopback)
{
int          rval = 0, l, nbytes, i;
rbuf_t		 *p = *ppbuf;
IpHeaderRec  *pip = &lpkt_ip(&p->pkt);
uint16_t	 dport;
int			 isbcst = 0;
int          ismcst = 0;
LanUdpPkt    hdr;

	if ( ! loopback )
		NETDRV_READ_INCREMENTAL(pif, pip, sizeof(*pip));
	rval += sizeof(*pip);

	/* accept IP unicast and broadcast */
	if ( (pip->dst == pif->ipaddr) ) {
		pif->stats.ip_rxufrm++;
	} else if ( (ismcst = mcListener(pif, pip->dst)) ) {
		pif->stats.ip_rxmfrm++;
	} else if ( (isbcst = ISBCST(pip->dst, pif->nmask)) ) {
		pif->stats.ip_rxbfrm++;
	} else {

		if ( ISMCST( pip->dst ) )
			pif->stats.ip_mcdstdropped++;
		else
			pif->stats.ip_dstdropped++;

#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) ) {
			printf("dropping IP to ");
			prip(stdout, pip->dst);
			printf(" (proto %i)\n", pip->prot);
			{
				l = ntohs(pip->len);
				for ( i=0; i<l; ) {
					printf("%02x ",*((char*)pip + i));	
					i++;
					if ( 0 == (i & 0xf) )
						printf("\n");
				}
				if ( (i & 0xf) )
					printf("\n");
			}
		}
#endif
		return rval;
	}

#ifdef DEBUG
	if ( (lanIpDebug & DEBUG_IP) ) {
		printf("accepting IP ");
		if ( pip->dst == pif->ipaddr ) {
			printf("unicast, ");
		} else if ( ismcst ) {
			printf("multicast (");
			prip(stdout, pip->dst);
			printf("), ");
		} else {
			printf("broadcast (");
			prip(stdout, pip->dst);
			printf("), ");
		}
		printf("proto %i\n", pip->prot);
	}
#endif

	/* reject fragmented packets, i.e., headers with MF
     * (more fragments) or an offset.
	 */
	if ( ntohs(pip->off) & 0x9fff ) {
		pif->stats.ip_frgdropped++;
#ifdef DEBUG
		if ( (lanIpDebug & DEBUG_IP) )
			printf("dropping IP packet, vhl: 0x%02x flgs/off 0x%04x\n",
                   pip->vhl, ntohs(pip->off));
#endif
		return rval;
	}

	nbytes = ntohs(pip->len);

	/* Check length and reject packets that wouldn't fit
	 * in an 'rbuf_t'
	 */
	if ( nbytes > sizeof( p->pkt ) - sizeof(EthHeaderRec) ) {
		pif->stats.ip_lendropped++;
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
			/* reject non-V4 headers or headers with length > 5 */
			if ( check_vhl(pip->vhl, 0x45) ) {
				pif->stats.icmp_hdrdropped++;
				return rval;
			}

			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(pif, &lpkt_icmp(&p->pkt), l);
			rval += l;
			scheduleLpWork(pif, p);
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
				pif->stats.igmp_hdrdropped++;
#ifdef DEBUG
				if ( (lanIpDebug & DEBUG_IP) )
					printf("dropping IP packet, vhl: 0x%02x (not 0x45 nor 0x46)\n", pip->vhl);
#endif
				return rval;
			}

			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(pif, &lpkt_igmpv2hdr( &p->pkt ).igmp_u, l);
			rval += l;

			scheduleLpWork(pif, p);
			/* worker thread took over the buffer */
			*ppbuf = 0;

		}
		break;

		case IP_PROT_UDP  :
		{
		LanUdpPkt pudp = &lpkt_udp_hdrs(&p->pkt);

			/* reject non-V4 headers or headers with length > 5 */
			if ( check_vhl(pip->vhl, 0x45) ) {
				pif->stats.udp_hdrdropped++;
				return rval;
			}

			/* UDP header is word aligned -> OK */
			if ( ! loopback )
				NETDRV_READ_INCREMENTAL(pif, &pudp->udp, sizeof(pudp->udp));
			rval += sizeof(pudp->udp);
			l    -= sizeof(pudp->udp);
			dport = ntohs(pudp->udp.dport);
#ifdef DEBUG
			if ( lanIpDebug & DEBUG_UDP ) {
				char buf[4*4];
				printf("handling UDP packet (dport %i%s)\n", dport, isbcst ? ", BCST":"");
				lanIpBscNtop(pudp->ip_part.ip.src, buf, sizeof(buf));
				printf("from %s:%i ...", buf, ntohs(pudp->udp.sport));
			}
#endif

			/* Scan the table of sockets for one that matches the destination
			 * port of this packet.
			 * If we find one, then we do some filtering (connected sockets
			 * only accept data from the peer), read the full payload (drivers
			 * of the FIFO type which define NETDRV_READ_INCREMENTAL) and
			 * dispatch the packet to the socket's message queue.
			 */
			
			_Thread_Disable_dispatch();
			for ( i=0; i<NSOCKS; i++ ) {
				if ( socks[i].port == dport ) {
					/* Skip source filtering if socket is not connected or
					 * FLG_MCPASS is set.
					 */
					if ( FLG_ISCONN == ((FLG_ISCONN | FLG_MCPASS) & socks[i].flags) ) {
						hdr = &socks[i].hdr;
						/* filter source IP and port */
						if (    hdr->udp.dport  != pudp->udp.sport
							|| (hdr->ip_part.ip.dst != pudp->ip_part.ip.src && ! ISBCST(hdr->ip_part.ip.dst, socks[i].intrf->nmask)) ) {
							_Thread_Enable_dispatch();
#ifdef DEBUG
							if ( lanIpDebug & DEBUG_UDP ) {
								printf("DROPPED [peer != connected peer]\n");
							}
#endif
							pif->stats.udp_sadropped++;
							return rval;
						}
					}
					_Thread_Enable_dispatch();

					/* slurp data */
					if ( ! loopback )
						NETDRV_READ_INCREMENTAL(pif, pudp->pld, l);
					rval += l;

					/* Refresh peer's ARP entry */
					if ( lanIpBscAutoRefreshARP && ! loopback ) {
						scheduleRefreshArp(pif, pudp);
					}

					_Thread_Disable_dispatch();
					/* see if socket is still alive */
					if ( socks[i].port == dport ) {
						UdpSockMsgRec msg;
						msg.pkt = p;
						msg.len = nbytes - sizeof(IpHeaderRec) - sizeof(UdpHeaderRec);

						/* post to user */
						if ( RTEMS_SUCCESSFUL == rtems_message_queue_send(socks[i].msgq, &msg, sizeof(msg)) ) {
							socks[i].nbytes += msg.len;
							/* they now own the buffer */
							*ppbuf = 0;
							pif->stats.udp_rxfrm++;
							pif->stats.udp_rxbytes+=msg.len;
						} else {
							pif->stats.udp_nospcdropped++;
						}
					} else {
						pif->stats.udp_sadropped++;
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
			pif->stats.ip_protdropped++;
		break;
	}

	return rval;
}

/* Handle ARP, IGMP and ICMP echo (ping) requests
 * Dispatch UDP packets to trivial 'sockets'
 *
 * Need to pass a pointer to the buffer pointer;
 * (*pprb) is set to NULL if the buffer was handed
 * on to a 'socket'.
 * 
 * RETURNS: number of remaining elements, i.e., number
 *          of unread bytes. This is relevant for FIFO-type
 *          chip drivers which may have to skip ahead to 
 *          find the next packet in the RX FIFO.
 *          E.g., if we receive a packet for a unknown protocol
 *          then we only have read the ethernet and IPv4 headers
 *          but not the full IPv4 payload. The routine would then
 *          return the length (from the IPv4 header) minus the size
 *          of the IPv4 header.
 */

int
lanIpProcessBuffer(IpBscIf pif, rbuf_t **pprb, int len)
{
rbuf_t         *prb = *pprb;
EthHeaderRec   *pll = &lpkt_eth(&prb->pkt);
uint16_t        tt;
int             i;

	prb->buf.intrf = pif;
#ifdef ENABLE_PROFILE
	rtems_clock_get_uptime( &prb->buf.tstmp );
#endif

	i    = len;
	len -= sizeof(*pll);
	if ( len < 0 ) {
		return i;
	}

	NETDRV_READ_INCREMENTAL(pif, prb, sizeof(*pll));

	pif->stats.eth_rxfrm++;

	if (lanIpDebug & DEBUG_IP) {
		int i;
		printf("Ethernet: got 0x%04x\n", ntohs(pll->type));
		for (i=0; i<sizeof(*pll); i++)
			printf("%02x ", *(((char*)pll)+i));
			printf("\n");
	}

	tt = pll->type;
	if ( htonsc(0x806) == tt ) {
		/* ARP */
		len -= handleArp(pprb, pif);
	} else if ( htonsc(0x800) == tt ) {
		/* IP  */
		len -= handleIP(pprb, pif, 0 /* this is not a looped-back buffer */);
	} else {
		pif->stats.eth_protdropped++;
		/* Don't count these with eth_rxdropped */
		pif->stats.eth_rxdropped--;
#ifdef DEBUG
		if (lanIpDebug & DEBUG_IP) {
			int i;
			printf("Ethernet: dropping 0x%04x\n", ntohs(pll->type));
			for (i=0; i<sizeof(*pll); i++)
				printf("%02x ", *(((char*)pll)+i));
				printf("\n");
		}
#endif
	}
	if ( *pprb )
		pif->stats.eth_rxdropped++;

	return len;
}

/**** LOW PRIORITY PROTOCOL HANDLING ******************************************/

/* Handle ARP request and reply packets we receive                            */

static void
processArp(IpBscIf intrf, rbuf_t *p)
{
IpArpRec	*pipa = &lpkt_arp(&p->pkt);
uint32_t    xx;

	xx = * ( (uint32_a_t *) pipa + 1 );

	if        ( htonsc(0x0001) == pipa->oper ) {
		/* REQUEST */
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP )
			printf("got ARP request for %d.%d.%d.%d\n",pipa->tpa[0],pipa->tpa[1],pipa->tpa[2],pipa->tpa[3]); 
#endif
		if ( get_tpa(pipa) != intrf->ipaddr ) {
			intrf->stats.arp_reqother++;
			return;
		}

		intrf->stats.arp_reqme++;

		/* they mean us; send reply */
		memcpy( intrf->arprep.ll.dst,  pipa->sha, 6);
		memcpy( intrf->arprep.arp.tha, pipa->sha, 10);

#if defined(DEBUG)
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("MATCH -> sending\n");
		}
#endif

		/* must copy 'arprep' into rbuf which we re-use for sending;
		 * we cannot enqueue arprep itself since we might need it 
		 * again before the buffer has made it out of the transmitter.
		 */
		memcpy( &lpkt_arp_pkt(&p->pkt), &intrf->arprep, sizeof(intrf->arprep) );

		refrbuf( p );
		intrf->stats.arp_txrep++;
		NETDRV_ENQ_BUFFER(intrf, p, sizeof(intrf->arprep));
	} else if ( htonsc(0x0002) == pipa->oper ) {
		/* REPLY   */

#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("got ARP reply from "); prether(stdout, pipa->sha);
			printf("\n");
		}
#endif
		intrf->stats.arp_gotrep++;

		arpPutEntry(intrf, get_spa(pipa), pipa->sha, 0);
	} else {
		/* should never get here; handleArp already checks that op is 0001 or 0002 */
		intrf->stats.arp_lenopdropped++;
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ARP ) {
			printf("dropping unknown ARP (oper 0x%04x)\n", ntohs(pipa->oper));
		}
#endif
	}
}

/* Handle ICMP ECHO requests we receive                                       */

static void
processIcmp(IpBscIf intrf, rbuf_t *p, int len)
{
IcmpHeaderRec *picmp = &lpkt_icmp(&p->pkt);

	if ( picmp->type == 8 /* ICMP REQUEST */ && picmp->code == 0 ) {
		intrf->stats.icmp_rxechoreq++;
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

		src2dstIp(&lpkt_ip_hdrs(&p->pkt));
		fillinSrcCsumIp(intrf, &lpkt_ip_hdrs(&p->pkt));

		refrbuf(p);
		intrf->stats.icmp_txechorep++;
		NETDRV_ENQ_BUFFER(intrf, p, sizeof(EthHeaderRec) + len);
	} else {
		intrf->stats.icmp_opdropped++;
#ifdef DEBUG
		if ( lanIpDebug & DEBUG_ICMP )
			printf("dropping ICMP (type %u, code %u)\n", picmp->type, picmp->code);
#endif
	}
}

/* Handle IGMP v2 reports and queries we receive                              */

static void
processIgmp(IpBscIf intrf, rbuf_t *buf_p, int len)
{
IpBscMcAddr      mca,mcal;
IgmpV2HeaderRec *pigmp;
rtems_interval   ticks_per_s;
uint32_t         report_dly_ticks;
int              pldlen;
LanIgmpV2Pkt     pkt;
LanIpPart        ipp;

	pkt = &lpkt_igmpv2hdr( &buf_p->pkt );
	ipp = &pkt->ip_part;

	assert( IP_PROT_IGMP == ipp->ip.prot );

	/* This IP header has one option word */
	pldlen = len - sizeof(IpHeaderRec);

	if ( 0x46 == ipp->ip.vhl ) {
		if ( htonlc(IP_OPT_ROUTER_ALERT) != ipp->ip.opts[0] ) {
			intrf->stats.igmp_hdrdropped++;
#ifdef DEBUG
			if ( lanIpDebug & (DEBUG_IP | DEBUG_IGMP) ) {
				printf("Dropping IGMP packet w/o router alert (option[0]: 0x%08"PRIx32")\n",
						ntohl(ipp->ip.opts[0]));
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
		intrf->stats.igmp_lendropped++;
#ifdef DEBUG
		if ( lanIpDebug & (DEBUG_IP | DEBUG_IGMP) ) {
			printf("Dropping IGMP packet with length (ip payload %u < 8)\n", pldlen);
		}
#endif
		return;
	}

	if ( ipcsum((uint8_t*) pigmp, pldlen) ) {
		intrf->stats.igmp_csumdropped++;
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

		printf("  SRC: "); prip(stdout,ipp->ip.src); fputc('\n', stdout);
		printf("  DST: "); prip(stdout,ipp->ip.dst); fputc('\n', stdout);
	}
#endif
	
	MCLOCK( intrf );

	switch( pigmp->type ) {
		case IGMP_TYPE_REPORT_V1:
			intrf->stats.igmp_rxreportv1++;
			intrf->stats.igmp_rxreportv2--;
		case IGMP_TYPE_REPORT_V2:
			intrf->stats.igmp_rxreportv2++;
			mca = lhtblFind( intrf->mctable, pigmp->gaddr);
			if ( igmp_state_delaying( mca ) ) {
#ifdef DEBUG
				if ( lanIpDebug & (DEBUG_IGMP) ) {
					printf("Report to ");   prip(stdout, pigmp->gaddr);
					printf(" seen (src: "); prip(stdout, ipp->ip.src);
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
				intrf->stats.igmp_rxqueryv1++;
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
				intrf->stats.igmp_rxqueryv2++;
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
				&& ISMCST_ALLSYS(ipp->ip.dst)   ) {
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

/* Low-priority worker task loop which blocks for work, inspects buffers and
 * executes the 'process<PROTO>' routines.
 */
static void
lpWorker(void *arg)
{
rbuf_t     *buf_p;
IpBscIf    intrf;
LanIpPacket   pkt;
int        len;

	while ( ( buf_p = dequeueLpWork() ) ) {

		intrf =   buf_p->buf.intrf;
		pkt   = & buf_p->pkt;

		if ( htonsc(0x806) == lpkt_eth(pkt).type ) {

			processArp( intrf, buf_p );

		} else if ( htonsc(0x800) == lpkt_eth(pkt).type ) {

			len = ntohs( lpkt_ip( pkt ).len );
			switch ( lpkt_ip( pkt ).prot ) {

				case IP_PROT_IGMP:
					processIgmp(intrf, buf_p, len);
				break;

				case IP_PROT_ICMP:
					processIcmp(intrf, buf_p, len);
				break;

				case IP_PROT_UDP:
					/* WARNING: we never get a full UDP packet here; just a 
					 *          dummied-up one when they want us to store
					 *          some peer's info in the arp cache.
					 */
					arpPutEntry(intrf, lpkt_ip(pkt).src, lpkt_eth(pkt).src, 0);
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

/**** INTERFACE RELATED ROUTINES **********************************************/

/* Allocate all resources necessary for an IF; cleanup and return NULL if not
 * all required resources were available.
 */
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

	if ( ! (arpcache(rval)[ARP_SENTINEL] = arp_allocentry()) ) {
		goto bail;
	}

	arpcache(rval)[ARP_SENTINEL]->ctime = ARP_PERM;

	if ( ! (rval->arpbuf = getrbuf()) )
		goto bail;


	if ( ! (rval->mutx = bsem_create("ipmx", SEM_MUTX)) ) {
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

	arp_destroyentry(arpcache(rval)[ARP_SENTINEL]);

	if ( rval->mctable )
		lhtblDestroy(rval->mctable, 0, 0);

	if ( rval->arpbuf )
		relrbuf(rval->arpbuf);

	if ( rval->mutx )
		rtems_semaphore_delete( rval->mutx );

	free(rval);

	return 0;
}

/* Allocate IF resources and initialize; attach driver handle and call driver's
 * NETDRV_START() entry point.
 * If any step fails undo all previous steps and return NULL.
 *
 * RETURNS: interface handle or NULL (on failure).
 */

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

	if ( NETDRV_START(ipbif_p, 0) ) {
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

/* Retrieve driver handle from interface handle                               */
void *
lanIpBscIfGetDrv(IpBscIf ipbif_p)
{
	return ipbif_p->drv_p;
}

/* Release all resources associated with an interface.
 *
 * RETURNS: zero on success, nonzero on failure.
 *
 * NOTE:    all sockets must be closed before IF can be destroyed
 */
int
lanIpBscIfDestroy(IpBscIf pif)
{
	if ( nsocks ) {
		fprintf(stderr,"Cannot take down interface -- some sockets still in use\n");
		return -1;
	}

	if ( pif ) {
		assert( nsocks == 0 );
		assert( intrf == pif );

		if ( pif->mclist.r_mcaddr ) {
			/* This address doesn't participate in IGMP - hence
			 * there is no possibility for the lanIpCallout_trystop()
			 * operation to fail (return -1). Thus we don't have
			 * to synchronize with the callout task.
			 */
			delmca( pif, pif->mclist.r_mcaddr, MC_ALLSYS_SD );
		}

		assert( pif->mclist.r_node == 0 );

		/* We don't hold MLOCK at this point so it is
		 * safe to use the synchronous lanIpCalout_stop()
		 * here.
		 */
		lanIpCallout_stop( &pif->mcIgmpV1RtrSeen );

		if ( pif->drv_p ) {
			if ( NETDRV_SHUTDOWN(pif->drv_p) ) {
				fprintf(stderr,"lanIpBscIfDestroy(): Unable to shutdown driver\n");
				return -1;
			}
		}

		if ( pif->arpbuf )
			relrbuf( pif->arpbuf );

		if ( pif->mctable )
			lhtblDestroy( pif->mctable, 0, 0 );

		intrf = 0;

		arpFlushCache(pif,1);
		arp_destroyentry(arpcache(pif)[ARP_SENTINEL]);
		arpcache(pif)[ARP_SENTINEL] = 0;

		rtems_semaphore_delete(pif->mutx);
		free(pif);
	}
	return 0;
}

/* Send a raw packet (user must set all headers) from an interface.
 *
 * NOTE:    No IP multicast loopback is performed (see lanIpBscSendBufRawIp()).
 *
 * RETURNS: 'len' argument.
 */
int
lanIpBscSendBufRaw(IpBscIf pif, LanIpPacket buf_p, int len)
{
	/* TODO ???: handle MC loopback */
	pif->stats.eth_txrawfrm++;
	NETDRV_ENQ_BUFFER(pif, (rbuf_t*)buf_p,  len);
	return len;
}

/* Send a raw packet (user must set all headers) from an interface assuming
 * it to be an IPv4 packet so that the length can be determined reading the
 * IPv4 header.
 *
 * NOTE:    If the destination of the packet is an IP multicast group to 
 *          which the sending interface is subscribed then a copy of the
 *          packet is looped-back into the stack.
 *
 * RETURNS: Complete packet length (including ethernet header with 2 bytes
 *          of initial alignment).
 */
int
lanIpBscSendBufRawIp(IpBscIf pif, LanIpPacket buf_p)
{
int	len ;
int do_mc_loopback;

	/* We are outside of the scope of sockets -- hence we
     * can't check a socket's multicast-loopback flag
	 * Just do it...
	 */
	do_mc_loopback = mcListener( pif, lpkt_ip(buf_p).dst );

	if ( do_mc_loopback )
		refrbuf( (rbuf_t *)buf_p );

	len = ntohs(lpkt_ip(buf_p).len) + sizeof(EthHeaderRec);
	NETDRV_ENQ_BUFFER(pif, (rbuf_t*)buf_p,  len);
	pif->stats.ip_txrawfrm++;

	/* loop back locally subscribed multicast */
	if ( do_mc_loopback ) {
		pif->stats.ip_txmcloopback++;
		/* all received bufs have the IF handle set... */
		((rbuf_t*)buf_p)->buf.intrf = pif;
		handleIP( (rbuf_t**)&buf_p, pif, 1 /* loopback */ );
		if ( buf_p )
			relrbuf( (rbuf_t *)buf_p );
	}
	return len;
}

/* Set/join multicast group on an interface (starting IGMP) and mark as used by
 * socket 'sd' (other than this marking the 'sd' has no meaning).
 */

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

				mcan->mc_flags |= MC_FLG_IGMP_LEAVE;

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

/* Unmark 'sd' in 'mca' and remove a multicast object 'mca' from an interface
 * (stopping IGMP) if 'mca' is completely unused. Other than for unmarking, 'sd'
 * has no meaning to this routine.
 *
 * NOTE:    This routine is asymmetric to 'addmca()'. Unlike the latter function
 *          which protects the relevant data structures by acquiring MCLOCK()
 *          'delmca()' cannot do the complete work but the caller needs to
 *          perform 'delmca()' as a step in a more complicated process (see
 *          e.g., udpSockLeaveMcast()).
 *
 * RETURNS: 'mca' pointer (which after return is detached from the interface and
 *          [almost] ready to be destroyed) on success or NULL on failure or if
 *          'sd' was not the last 'sd' using 'mca'.
 */
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

/* Dump info about all MC groups subscribed on interface to file 'f'.
 *
 * RETURNS: Zero on success, nonzero on error.
 *
 * NOTES:   Not fully thread-safe; must not destroy interface while using this.
 *   
 *          'f' may be passed as NULL in which case 'stdout' is used.
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

/**** UDP SOCKETS *************************************************************/

/* Compute and set IP payload length, UPD length; compute and set IP header
 * checksum.
 */
void
udpSockHdrsSetlen(LanUdpPkt p, int payload_len)
{
	p->ip_part.ip.len   = htons(payload_len + sizeof(UdpHeaderRec) + sizeof(IpHeaderRec));

#if 0
	/* alias-rule is quite nasty. Even though the 'IpHeaderRec' is
	 * given the 'may_alias' attribute I still found that gcc
	 * optimizes initialization of the checksum away.
	 */
    p->ip_part.ip.csum = 0;
#else
	memset( & p->ip_part.ip.csum, 0, sizeof(p->ip_part.ip.csum) );
#endif

	p->ip_part.ip.csum  = in_cksum_hdr((void*)&p->ip_part.ip);

	p->udp.len      = htons(payload_len + sizeof(UdpHeaderRec));
	p->udp.csum     = htonsc(0);
}

int
udpSockHdrsInitFromIf(IpBscIf intrf, LanUdpPkt p, uint32_t dipaddr, uint16_t dport, uint16_t sport, uint16_t ip_id)
{
int rval = 0;

	if ( dipaddr ) 
		rval = arpLookup(intrf, dipaddr, p->ip_part.ll.dst, 0);
	else /* they want to leave it blank */
		memset(p->ip_part.ll.dst,0,6);

	p->ip_part.ll.type  = htonsc(0x0800);	/* IP */

	p->ip_part.ip.vhl   = 0x45;	/* version 4, 5words length */
	p->ip_part.ip.tos   = 0x30; /* priority, minimize delay */
	p->ip_part.ip.len   = 0;
	p->ip_part.ip.id    = htons(ip_id);   /* ? */
	p->ip_part.ip.off   = htonsc(0x4000); /* set DF flag */
	p->ip_part.ip.ttl   = 4;
	p->ip_part.ip.prot  = IP_PROT_UDP;	  /* UDP */
	p->ip_part.ip.dst   = dipaddr;

	p->udp.dport = htons(dport);
	p->udp.len   = 0;

	fillinSrcCsumUdp(intrf, p, sport);

	/* reset checksum; length is not correct yet */
	p->ip_part.ip.csum  = 0;

	return rval;
}

int
udpSockHdrsInit(int sd, LanUdpPkt p, uint32_t dipaddr, uint16_t dport, uint16_t ip_id)
{
	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	return udpSockHdrsInitFromIf(socks[sd].intrf, p, dipaddr, dport, socks[sd].port, ip_id);
}

void
udpSockHdrsReflect(LanUdpPkt p)
{
uint16_t  port = ntohs(p->udp.dport);
	src2dstUdp(p);
	fillinSrcCsumUdp(udpSockGetBufIf((LanIpPacket)p), p, port);
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
			lanIpBscCfg.rx_queue_depth,
			sizeof(UdpSockMsgRec),
			RTEMS_FIFO | RTEMS_LOCAL,
			&q) ) {
		q    =  0;
		rval = -ENOSPC;
		goto egress;
	}

	if ( ! (m = bsem_create( "udpl", SEM_MUTX )) ) {
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
	socks[rval].mclpbk = 1;

	udpSockHdrsInitFromIf(intrf, &socks[rval].hdr, 0, 0, port, 0);

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
udpSockConnect(int sd, uint32_t dipaddr, int dport, int flags)
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

		socks[sd].flags &= ~(FLG_ISCONN | FLG_MCPASS);

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

		if ( (rval = udpSockHdrsInitFromIf(socks[sd].intrf, &socks[sd].hdr, dipaddr, dport, socks[sd].port, 0)) ) {
			/* ARP lookup failure; BSD sockets probably would not
			 * fail here...
			 */
			goto egress;
		}

		socks[sd].flags |= FLG_ISCONN;
		if ( ISMCST(dipaddr) && (UDPSOCK_MCPASS & flags) ) {
			socks[sd].flags |= FLG_MCPASS;
		} else {
			socks[sd].flags &= ~ FLG_MCPASS;
		}
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

int
_udpSockSendTo_internal(int sd, LanIpPacket buf_p, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport)
{
int          rval;
LanUdpPkt    h;
LanIpPart    ipp;
int          do_mc_loopback = 0;
IpBscIf      pif;
PRFDECL;

	if ( payload_len > UDPPAYLOADSIZE ) {
		rval = -EMSGSIZE;
		goto bail;
	}

	if ( sd < 0 || sd >= NSOCKS ) {
		rval = -EBADF;
		goto bail;
	}

	if ( 0 == socks[sd].port ) {
		rval = -EBADF;
		goto bail;
	}

	setbase();

try_again:

	SOCKLOCK( &socks[sd] );

	pif = socks[sd].intrf;

	dodiff(1);

#ifndef NETDRV_SND_PACKET
	/* If they supply a buffer then we must copy the socket's header
	 * there.
	 */
	if ( buf_p ) {
		h = & lpkt_udp_hdrs( buf_p );
		memcpy( h, &socks[sd].hdr, sizeof(*h) );
	} else
#endif
	{
		h = &socks[sd].hdr;
	}

	ipp = & h->ip_part;

	dodiff(2);

	if ( ! ipaddr ) {
		/* If they didn't supply a destination address the socket must be connected */
		if ( ! (FLG_ISCONN & socks[sd].flags) ) {
			SOCKUNLOCK( &socks[sd] );
			rval = -ENOTCONN;
			goto bail;
		}
		ipaddr = ipp->ip.dst;
	} else {
		/* if the socket is already connected only allow sending to peer */

		/* FIXME: BSD allows 'sendto' to override the 'connected' peer address 
		 *        BUT (linux) both, IP address AND port must be specified.
         */
		if ( (FLG_ISCONN & socks[sd].flags) ) {
			if (   ipp->ip.dst != ipaddr
				|| (unsigned short)ntohs( h->udp.dport ) != dport ) {

				rval = -EISCONN;

				SOCKUNLOCK( &socks[sd] );

				goto bail;
			}
		} else {
			ipp->ip.dst  = ipaddr;
			h->udp.dport = htons((unsigned short)dport);
		}
	}

	dodiff(3);

	if ( ! (FLG_ISCONN & socks[sd].flags) ) {
		uint8_t dummy[6];

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
		if ( (rval = arpLookup(pif, ipp->ip.dst, ipp->ll.dst, 1)) ) {
			SOCKUNLOCK( &socks[sd] );

			if ( -ENOTCONN != rval ) {
				/* Don't bother to try another lookup */
				goto bail;
			}

			if ( (rval = arpLookup(pif, ipaddr, dummy, 0)) ) {
				goto bail; /* most likely -ENOTCONN */
			}

			/* Here we should start over again since things in the socket
			 * could have changed...
			 */
			goto try_again;
		}
	} else {
		if ( (rval = arpLookup(pif, ipp->ip.dst, ipp->ll.dst, 0)) ) {
			SOCKUNLOCK( &socks[sd] );
			goto bail;
		}
	}

	dodiff(4);

	udpSockHdrsSetlen(h, payload_len);

	dodiff(5);

	do_mc_loopback = socks[sd].mclpbk && mcListener( pif, ipp->ip.dst );

#ifdef NETDRV_SND_PACKET
	if ( buf_p )
		payload = lpkt_udp_hdrs( buf_p ).pld;
	rval = NETDRV_SND_PACKET( pif, h, sizeof(*h), payload, payload_len );
#endif

	if ( ! buf_p ) {
#ifdef NETDRV_SND_PACKET
		if ( do_mc_loopback )
#endif
		{
			if ( ! (buf_p = (LanIpPacket)getrbuf()) ) {
				SOCKUNLOCK( &socks[sd] );
				rval = -ENOBUFS;
				goto bail;
			}
			memcpy( &lpkt_udp_hdrs( buf_p ), h, sizeof(*h) );
			memcpy(  lpkt_udp_hdrs( buf_p ).pld,  payload, payload_len );
		}
	}
#ifdef NETDRV_SND_PACKET
	else {
		if ( ! do_mc_loopback )
			relrbuf( (rbuf_t*) buf_p );
	}
#endif

	dodiff(6);

#ifndef NETDRV_SND_PACKET
	if ( do_mc_loopback )
		refrbuf( (rbuf_t*) buf_p );
	rval = payload_len;
	NETDRV_ENQ_BUFFER( pif, (rbuf_t*)buf_p, payload_len + sizeof(*h) );
#endif

	if ( rval > 0 ) {
		pif->stats.udp_txfrm++;
		pif->stats.udp_txbytes += rval;
	} else {
		pif->stats.udp_txdropped++;
	}

	dodiff(7);

	/* loop back locally subscribed multicast ? */
	if ( do_mc_loopback ) {
		pif->stats.ip_txmcloopback++;
		/* all received bufs have the IF handle set... */
		((rbuf_t*)buf_p)->buf.intrf = pif;
		handleIP( (rbuf_t**)&buf_p, pif, 1 /* loopback */ );
		if ( buf_p )
			relrbuf( (rbuf_t *)buf_p );
	}

	dodiff(8);

	SOCKUNLOCK( &socks[sd] );
	
	dodiff(9);

	return rval;

bail:
	relrbuf((rbuf_t*)buf_p);
	return rval;
}

int
udpSockSend(int sd, void *payload, int payload_len)
{
	return _udpSockSendTo_internal(sd, 0, payload, payload_len, 0, 0);
}


int
udpSockSendTo(int sd, void *payload, int payload_len, uint32_t ipaddr, uint16_t dport)
{
	return _udpSockSendTo_internal(sd, 0, payload, payload_len, ipaddr, dport);
}

int
udpSockSendBuf(int sd, LanIpPacket b, int payload_len)
{
	return _udpSockSendTo_internal(sd, b, 0, payload_len, 0, 0);
}

int
udpSockSendBufTo(int sd, LanIpPacket b, int payload_len, uint32_t ipaddr, uint16_t dport)
{
	return _udpSockSendTo_internal(sd, b, 0, payload_len, ipaddr, dport);
}


void
udpSockFreeBuf(LanIpPacketRec *b)
{
	relrbuf((rbuf_t*)b);
}

LanIpPacket
udpSockGetBuf()
{
rbuf_t *buf = getrbuf();

	return buf ? &buf->pkt : 0;
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

IpBscIf
udpSockGetBufIf(LanIpPacket buf_p)
{
	return ((rbuf_t*)buf_p)->buf.intrf;
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

int
udpSockSetMcastLoopback(int sd, int val)
{
int rval;

	if ( sd < 0 || sd >= NSOCKS )
		return -EBADF;

	if ( 0 == socks[sd].port )
		return -EBADF;

	SOCKLOCK( & socks[sd] );

	rval = socks[sd].mclpbk;
	if ( val >= 0 ) {
		/* if val < 0 they want to just read the current state */
		socks[sd].mclpbk = val;	
	}
	SOCKUNLOCK( & socks[sd] );

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

/**** STACK INITIALIZATION AND CLEANUP ****************************************/

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

	killer.kill_resource = workSema;

	if ( workTask ) {
		task_pseudojoin( workTask, KILL_BY_SEMA, killer );

		workTask = 0;
	}

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

int
lanIpBscConfig(LanIpBscConfig p_cfg, LanIpBscConfig p_oldcfg)
{
	/* Cannot change anything once stack is up */
	if ( workSema && p_cfg && p_cfg->mask )
		return -EINVAL;

	if ( p_oldcfg ) {
		*p_oldcfg           = lanIpBscCfg;
	}

	if ( p_cfg ) {
		if ( (LANIPCFG_RX_RING & p_cfg->mask) ) {
			lanIpBscCfg.rx_ring_size = p_cfg->rx_ring_size;
		}
		if ( (LANIPCFG_TX_RING & p_cfg->mask) ) {
			lanIpBscCfg.tx_ring_size = p_cfg->tx_ring_size;
		}
		if ( (LANIPCFG_N_RBUFS & p_cfg->mask) ) {
			int morebufs = p_cfg->num_rbufs - lanIpBufTotal;
			/* Don't reduce below what we already have */
			if ( morebufs > 0 ) {
				if ( lanIpBscAddBufs(morebufs) ) {
					return -ENOMEM;
				}
			}
			/* lanIpBscCfg.num_rbufs is updated by lanIpBscAddBufs() */
		}
		
		if ( (LANIPCFG_SQDEPTH & p_cfg->mask) ) {
			lanIpBscCfg.rx_queue_depth = p_cfg->rx_queue_depth;
		}
	}

	return 0;
}

void
lanIpBscDumpConfig(FILE *f)
{
	if ( !f )
		f = stdout;
	fprintf(f,"LanIpBasic Configuration Parameters:\n");
	fprintf(f,"RBUFs: Free %6u, Used %6u, Total %6u\n",
		lanIpBufAvail,
		lanIpBufTotal - lanIpBufAvail,
		lanIpBufTotal);
	fprintf(f,"Socks: Free %6u, Used %6u, Total %6u\n",
		NSOCKS - nsocks,
		nsocks,
		NSOCKS);
	fprintf(f,"Rings:              RX   %6u,  TX   %6u\n",
		lanIpBscCfg.rx_ring_size,
		lanIpBscCfg.tx_ring_size);
	fprintf(f,"Socket RX queue depth:                 %6u\n",
		lanIpBscCfg.rx_queue_depth);
}

void
lanIpBscDumpIfStats(IpBscIf intrf, unsigned info, FILE *f)
{
uint32_t tmp;

	if ( ! intrf ) {
		fprintf(stderr,"usage: void lanIpBscDumpIfStats(IpBscIf intrf, unsigned info_amount, FILE *f)\n");
		fprintf(stderr,"       missing 'intrf' handle!\n");
		return;
	}

	if ( !f )
		f = stdout;
	fprintf(f,"LanIpBasic Interface Statistics:\n");
	tmp = ntohl(intrf->ipaddr);
	tmp = ntohl(intrf->nmask);
	fprintf(f,"Driver: %s\n", NETDRV_NAME(intrf));
	fprintf(f,"Addresses:\n");
	fprintf(f," IP ");
		prip(f,intrf->ipaddr);
		fputc('/',f);
		prip(f,intrf->nmask);
		fprintf(f,"; MAC ");
		prether(f, intrf->arpreq.ll.src);
		fputc('\n',f);
	if ( (IPBSC_IFSTAT_INFO_MAC & info ) ) {
		fprintf(f,"Ethernet statistics:\n");
		fprintf(f," # Frames Received:          %9"PRIu32"\n", intrf->stats.eth_rxfrm);
		fprintf(f," # RX Frames dropped\n");
		fprintf(f,"    Unsupported Protocol:    %9"PRIu32"\n", intrf->stats.eth_protdropped);
		fprintf(f,"    Rejected by Higher Prot: %9"PRIu32"\n", intrf->stats.eth_rxdropped);
		fprintf(f," # Raw Frames Sent:          %9"PRIu32"\n", intrf->stats.eth_txrawfrm);
	}
	if ( (IPBSC_IFSTAT_INFO_IP & info ) ) {
		fprintf(f,"IP statistics:\n");
		fprintf(f," # Frames Accepted (Unicast):%9"PRIu32"\n", intrf->stats.ip_rxufrm);
		fprintf(f," # Frames Accepted (MCAST):  %9"PRIu32"\n", intrf->stats.ip_rxmfrm);
		fprintf(f," # Frames Accepted (BCAST):  %9"PRIu32"\n", intrf->stats.ip_rxbfrm);
		fprintf(f," # Dropped Frames:\n");
		fprintf(f,"    Address Mismatch:        %9"PRIu32"\n", intrf->stats.ip_dstdropped);
		fprintf(f,"    Soft Multicast Filter:   %9"PRIu32"\n", intrf->stats.ip_mcdstdropped);
		fprintf(f,"    Fragmented:              %9"PRIu32"\n", intrf->stats.ip_frgdropped);
		fprintf(f,"    Too Big:                 %9"PRIu32"\n", intrf->stats.ip_lendropped);
		fprintf(f,"    Unsupported Protocol:    %9"PRIu32"\n", intrf->stats.ip_protdropped);
		fprintf(f," # Multicast Loopback:       %9"PRIu32"\n", intrf->stats.ip_txmcloopback);
		fprintf(f," # Raw IP buffers sent:      %9"PRIu32"\n", intrf->stats.ip_txrawfrm);
	}
	if ( (IPBSC_IFSTAT_INFO_UDP & info ) ) {
		fprintf(f,"UDP statistics:\n");
		fprintf(f," # Frames Accepted:          %9"PRIu32"\n", intrf->stats.udp_rxfrm);
		fprintf(f," # Bytes Accepted:           %9"PRIu32"\n", intrf->stats.udp_rxbytes);
		fprintf(f," # Dropped Frames:\n");
		fprintf(f,"    Unsup. IP Header:        %9"PRIu32"\n", intrf->stats.udp_hdrdropped);
		fprintf(f,"    Addr./Port Mismatch:     %9"PRIu32"\n", intrf->stats.udp_sadropped);
		fprintf(f,"    No Space In Socket Queue:%9"PRIu32"\n", intrf->stats.udp_nospcdropped);
		fprintf(f," # Frames Sent:              %9"PRIu32"\n", intrf->stats.udp_txfrm);
		fprintf(f," # Bytes Sent:               %9"PRIu32"\n", intrf->stats.udp_txbytes);
		fprintf(f," # Frames Dropped (TX):      %9"PRIu32"\n", intrf->stats.udp_txdropped);
	}
	if ( (IPBSC_IFSTAT_INFO_ARP & info ) ) {
		fprintf(f,"ARP statistics:\n");
		fprintf(f," # Replies Received:         %9"PRIu32"\n", intrf->stats.arp_gotrep);
		fprintf(f," # Requests Received (Me):   %9"PRIu32"\n", intrf->stats.arp_reqme);
		fprintf(f," # Requests Received (Other):%9"PRIu32"\n", intrf->stats.arp_reqother);
		fprintf(f," # Requests Dropped:\n");
		fprintf(f,"    Unsup. Len. or Operation:%9"PRIu32"\n", intrf->stats.arp_lenopdropped);
		fprintf(f,"    Unsup. Protocol:         %9"PRIu32"\n", intrf->stats.arp_protdropped);
		fprintf(f," Failures to Create Sema:    %9"PRIu32"\n", intrf->stats.arp_nosem);
		fprintf(f," # Requests Sent:            %9"PRIu32"\n", intrf->stats.arp_txreq);
		fprintf(f," # Replies Sent:             %9"PRIu32"\n", intrf->stats.arp_txrep);
		fprintf(f," ARP Cache Dump:\n");
		arpDumpCacheIndented( 2, intrf, f );
	}
	if ( (IPBSC_IFSTAT_INFO_ICMP & info ) ) {
		fprintf(f,"ICMP statistics:\n");
		fprintf(f," # Echo Requests Received:   %9"PRIu32"\n", intrf->stats.icmp_rxechoreq);
		fprintf(f," # Echo Replies Sent:        %9"PRIu32"\n", intrf->stats.icmp_txechorep);
		fprintf(f," # Frames Dropped:\n");
		fprintf(f,"    Unsup. IP Header:        %9"PRIu32"\n", intrf->stats.icmp_hdrdropped);
		fprintf(f,"    Unsupported Operation:   %9"PRIu32"\n", intrf->stats.icmp_opdropped);
	}
	if ( (IPBSC_IFSTAT_INFO_IGMP & info ) ) {
		fprintf(f,"IGMP statistics:\n");
		fprintf(f," # V1 Reports Received:      %9"PRIu32"\n", intrf->stats.igmp_rxreportv1);
		fprintf(f," # V2 Reports Received:      %9"PRIu32"\n", intrf->stats.igmp_rxreportv2);
		fprintf(f," # V1 Queries Received:      %9"PRIu32"\n", intrf->stats.igmp_rxqueryv1);
		fprintf(f," # V2 Queries Received:      %9"PRIu32"\n", intrf->stats.igmp_rxqueryv2);
		fprintf(f," # Frames Dropped:\n");
		fprintf(f,"    Unsup. IP Header:        %9"PRIu32"\n", intrf->stats.igmp_hdrdropped);
		fprintf(f,"    Unsup. Length:           %9"PRIu32"\n", intrf->stats.igmp_lendropped);
		fprintf(f,"    Bad Checksum:            %9"PRIu32"\n", intrf->stats.igmp_csumdropped);
		fprintf(f," # V1 Reports Sent:          %9"PRIu32"\n", intrf->stats.igmp_txreportv1);
		fprintf(f," # V2 Reports Sent:          %9"PRIu32"\n", intrf->stats.igmp_txreportv2);
		fprintf(f," # Leave Group Msgs. Sent:   %9"PRIu32"\n", intrf->stats.igmp_txleave);
	}
	if ( (IPBSC_IFSTAT_INFO_MCGRP & info ) ) {
		fprintf(f,"Multicast Group Membership Info:\n");
		lanIpBscDumpMcGroups(intrf, f);
	}
	if ( (IPBSC_IFSTAT_INFO_DRV & info ) ) {
		fprintf(f,"Driver (%s) Statistics/Info:\n", NETDRV_NAME(intrf));
#ifdef NETDRV_DUMPSTATS
		NETDRV_DUMPSTATS(intrf, f);
#else
		fprintf(f,"UNSUPPORTED BY DRIVER\n");
#endif
	}
}


LanIpBscSumStats
lanIpBscGetStats()
{
IpBscIf            pif;
LanIpBscSumStats   rval    = 0;
LanIpBscIfSumStats psums   = 0;
LanIpBscIfSumStats *ppsums = 0;

	if ( ! (rval  = calloc(sizeof(*rval),1)) ) {
		return 0;
	}
	ppsums            = &rval->if_stats;
	rval->if_max      = 0;

	rval->nsocks_max  = NSOCKS;
	rval->nsocks_used = nsocks;
	rval->sock_qdepth = lanIpBscCfg.rx_queue_depth;
	rval->rbufs_max   = lanIpBufTotal;
	rval->rbufs_used  = lanIpBufTotal - lanIpBufAvail;

	/* for all IFs DO */
	{

	pif            = intrf;

	if ( ! (psums = calloc(sizeof(*psums),1)) ) {
		free(rval);
		rval = 0;
		return 0;
	}
	rval->if_max++;

	psums->p_next  = 0;
	*ppsums        = psums;
	ppsums         = &psums->p_next;

	psums->drv_name      = NETDRV_NAME(pif);
	memcpy( psums->en_addr, pif->arpreq.ll.src, sizeof(psums->en_addr) );
	psums->ip_addr       = pif->ipaddr;
	psums->ip_nmask      = pif->nmask;
	psums->mc_ngroups    = pif->mcnum;

	psums->eth_rx_frms   = pif->stats.eth_rxfrm;
	psums->eth_rx_drop   = pif->stats.eth_protdropped  + pif->stats.eth_rxdropped;

	psums->arp_rx_reps   = pif->stats.arp_gotrep;
	psums->arp_rx_reqs   = pif->stats.arp_reqme        + pif->stats.arp_reqother;
	psums->arp_rx_drop   = pif->stats.arp_lenopdropped + pif->stats.arp_protdropped;
	psums->arp_tx_reqs   = pif->stats.arp_txreq;
	psums->arp_tx_reps   = pif->stats.arp_txrep;

	psums->ip_rx_ufrm    = pif->stats.ip_rxufrm;
	psums->ip_rx_mfrm    = pif->stats.ip_rxmfrm;
	psums->ip_rx_bfrm    = pif->stats.ip_rxbfrm;

	psums->ip_rx_drop    = pif->stats.ip_mcdstdropped + pif->stats.ip_frgdropped;
	psums->ip_rx_drop   += pif->stats.ip_lendropped   + pif->stats.ip_protdropped;
	psums->ip_rx_drop   += pif->stats.ip_dstdropped;

	psums->icmp_rx_ereq  = pif->stats.icmp_rxechoreq;
	psums->icmp_rx_drop  = pif->stats.icmp_hdrdropped + pif->stats.icmp_opdropped;

	psums->igmp_rx_reps  = pif->stats.igmp_rxreportv1 + pif->stats.igmp_rxreportv2;
	psums->igmp_rx_qrys  = pif->stats.igmp_rxqueryv1  + pif->stats.igmp_rxqueryv2;
	psums->igmp_rx_drop  = pif->stats.igmp_lendropped + pif->stats.igmp_csumdropped;
	psums->igmp_rx_drop += pif->stats.igmp_hdrdropped;
	psums->igmp_tx_reps  = pif->stats.igmp_txreportv1 + pif->stats.igmp_txreportv2;
	psums->igmp_tx_leav  = pif->stats.igmp_txleave;


	psums->udp_rx_frms   = pif->stats.udp_rxfrm;
	psums->udp_rx_drop   = pif->stats.udp_sadropped   + pif->stats.udp_nospcdropped;
	psums->udp_rx_drop  += pif->stats.udp_hdrdropped;
	psums->udp_tx_frms   = pif->stats.udp_txfrm;

	}

	return rval;
}

void
lanIpBscFreeStats(LanIpBscSumStats stats)
{
LanIpBscIfSumStats ifs, ifsn;
	if ( stats ) {
		for ( ifs = stats->if_stats; ifs; ifs = ifsn ) {
			ifsn = ifs->p_next;
			free( ifs );
		}
		free( stats );
	}
}
