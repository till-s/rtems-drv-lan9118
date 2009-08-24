#include <stdint.h>
/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */

typedef struct amdeth_drv_s_ *amdeth_drv;

#define ETHERPADSZ sizeof(((EthHeaderRec*)0)->pad)

/* fwd decl of interface struct */
struct IpBscIfRec_;


#define NETDRV_ATOMIC_SEND_ARPREQ(pif, ipaddr)								\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			amdeth_drv mdrv = (amdeth_drv)(pif)->drv_p;						\
			int l_ = sizeof((pif)->arpreq) - ETHERPADSZ;					\
			memcpy(b, &(pd)->arpreq, sizeof((pif)->arpreq));				\
			set_tpa((IpArpRec*)(b+sizeof(EthHeaderRec)), ipaddr);           \
			memcpy(((LanArp)b)->arp.tpa, &ipaddr, sizeof(ipaddr));			\
			if ( amd_send_buf_locked(mdrv, b, b+ETHERPADSZ, l_) )			\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pif, ptr, nbytes)							\
	do {} while (0)

static inline int
amd_send_buf_locked(amdeth_drv mdrv, void *pbuf, void *data, int len);

#define NETDRV_ENQ_BUFFER(pif, pbuf, nbytes)								\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
		amdeth_drv mdrv = (amdeth_drv)(pif)->drv_p;							\
																			\
		if ( amd_send_buf_locked(mdrv, pbuf, b_, l_) )						\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

static inline void
NETDRV_READ_ENADDR(struct IpBscIfRec_ *ipbif_p, uint8_t *buf);

static inline void
NETDRV_MC_FILTER_ADD(struct IpBscIfRec_ *ipbif_p, uint8_t *macaddr);

static inline void
NETDRV_MC_FILTER_DEL(struct IpBscIfRec_ *ipbif_p, uint8_t *macaddr);

#define NETDRV_INCLUDE	<amdeth.h>

/* No alignment req. on RX buffers, AFAIK.
 * However: SW cache flushing probably needs 32-bytes
 */
#define RBUF_ALIGNMENT	32

/* Configure enough resources for a large application
 *
 * RX_RING_SIZE > number of 'simultaneously' arriving packets.
 *                Set to the number of satellite devices we
 *                receive data from plus some overhead.
 * NRBUFS         - the ring needs bufs
 *                - an application might keep some RX data around.
 *                  Configure enough bufs so that 20 packets per
 *                  satellite device could be kept.
 *
 * Configure for 20 satellites...
 */

#define N_SATELLITES (20)
#define RX_RING_SIZE (N_SATELLITES+(N_SATELLITES)/4 + 1)


#define TX_RING_SIZE (N_SATELLITES/4 + 1)

#undef RX_RING_SIZE
#undef TX_RING_SIZE
#define RX_RING_SIZE	8
#define TX_RING_SIZE	4

#define NRBUFS (RX_RING_SIZE + RX_RING_SIZE*20 + 20)

#include "lanIpBasic.c"

typedef struct amdeth_drv_s_ {
	AmdEthDev  mp;
	rtems_id   mutex;
	rbuf_t     *spare;
} amdeth_drv_s;

static inline void mutex_lock(rtems_id);
static inline void mutex_unlk(rtems_id);

#define DRVLOCK(drv)   mutex_lock((drv)->mutex)
#define DRVUNLOCK(drv) mutex_unlk((drv)->mutex)


static inline int
amd_send_buf_locked(amdeth_drv mdrv, void *pbuf, void *data, int len)
{
int rval;
EtherHeader dummyh;

	assert( (uintptr_t)data - (uintptr_t)pbuf == ETHERPADSZ );
	DRVLOCK(mdrv);
		/* The chip doesn't seem to like zero-length descriptors so
		 * we pass the header in separately...
		 */
		dummyh = (EtherHeader)data;
		data  += 14;
		rval   = amdEthSendPacketSwp( mdrv->mp, dummyh, &data, len - 14 );
		if ( !rval && data ) {
			relrbuf((rbuf_t*)((char*)data - ETHERPADSZ));
		}
	DRVUNLOCK(mdrv);
	return rval;
}

static inline void NETDRV_READ_ENADDR(struct IpBscIfRec_ *ipbif_p, uint8_t *buf)
{
amdeth_drv drvhdl = (amdeth_drv)ipbif_p->drv_p;
EtherHeaderRec h;
	amdEthHeaderInit(&h, 0, drvhdl->mp);
	memcpy(buf, h.src, sizeof(h.src));
}

static inline void
NETDRV_MC_FILTER_ADD(struct IpBscIfRec_ *ipbif_p, uint8_t *macaddr)
{
amdeth_drv drvhdl = (amdeth_drv)ipbif_p->drv_p;
	DRVLOCK( drvhdl );
		amdEthMcFilterAdd( drvhdl->mp, macaddr );
	DRVUNLOCK( drvhdl );
}

static inline void
NETDRV_MC_FILTER_DEL(struct IpBscIfRec_ *ipbif_p, uint8_t *macaddr)
{
amdeth_drv drvhdl = (amdeth_drv)ipbif_p->drv_p;
	DRVLOCK( drvhdl );
		amdEthMcFilterDel( drvhdl->mp, macaddr );
	DRVUNLOCK( drvhdl );
}

#define KILL_EVENT RTEMS_EVENT_4
#define ALL_EVENTS (RTEMS_EVENT_0 | RTEMS_EVENT_1 | KILL_EVENT)

static inline int rx_adjusted(AmdEthDev d, rbuf_t **ppbuf)
{
int st;
union {
	rbuf_t *b;
	char   *raw;
} buf;

	buf.b = *ppbuf;
	buf.raw += ETHERPADSZ;
	st = amdEthReceivePacket( d, &buf.raw, LANPKTMAX - ETHERPADSZ );
	if ( buf.raw ) {
		buf.raw -= ETHERPADSZ;
		*ppbuf   = buf.b;
	}
	return st;
}

void
drvAmdIpBasicTask(rtems_task_argument arg)
{
IpBscIf					ipbif  = (IpBscIf)arg;
amdeth_drv				mdrv   = lanIpBscIfGetDrv(ipbif);
AmdEthDev				mp     = mdrv->mp;
rtems_task_priority     p;
int                     st;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task starting up\n");
	}
#endif

	rtems_task_set_priority(RTEMS_SELF, RTEMS_CURRENT_PRIORITY, &p);

	amdEthStart(p,1);

	do { 
		rbuf_t *buf = getrbuf();
		if ( buf ) {
			st = rx_adjusted( mp, &buf );
			if ( st >= 0 ) {
				lanIpProcessBuffer( ipbif, &buf, st );
			}
			relrbuf(buf);
		} else {
			/* no free buffer; use the spare but don't give it away! */
			st = rx_adjusted( mp, &mdrv->spare );
		}
	} while ( st >= 0 );

	fprintf(stderr,"drvAmdEthIpBasic: RX error %i; terminating\n", st);

	relrbuf(mdrv->spare);
	rtems_semaphore_delete(mdrv->mutex);

	/*
	 * leave driver struct alone so that they still can
	 * shutdown if the task exited on error...
	free(mdrv);
	*/

	rtems_task_delete(RTEMS_SELF);
}

int
lanIpBscDrvStart(IpBscIf ipbif_p, int pri)
{
rtems_status_code sc;
rtems_id          tid;

	if ( pri <= 0 )
		pri = 20;

	sc = rtems_task_create(
				rtems_build_name('i','p','b','d'),
				pri,	/* can be changed later */
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "creating drvAmdIpBasicTask\n");
		return -1;
	}

	sc = rtems_task_start( tid, drvAmdIpBasicTask, (rtems_task_argument)ipbif_p );
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "starting drvAmdIpBasicTask\n");
		rtems_task_delete( tid );
		return -1;
	}
	return 0;
}

LanIpBscDrv
lanIpBscDrvCreate(int unit, uint8_t *enaddr)
{
int                   i, minu, maxu;
amdeth_drv            mdrv = 0;

	if ( enaddr ) {
		/* can only use ethernet address from PROM for now */
		fprintf(stderr,"drvAmdEthIpBasic: setting MAC address not supported\n");
		return 0;
	}

	if ( ! (mdrv = malloc(sizeof(*mdrv))) ) {
		return 0;
	}

	memset(mdrv, 0, sizeof(*mdrv));

	if ( ! (mdrv->spare = getrbuf()) ) {
		goto egress;
	}

	mdrv->mutex = 0;

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
								rtems_build_name('a','m','d','L'),
								1,
								RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
								0,
								&mdrv->mutex) ) {
			goto egress;
	}

	if ( unit < 0 ) {
		minu = 0;
		maxu = 3;
	} else {
		minu = unit;
		maxu = unit+1;
	}

	for ( unit=minu; unit < maxu; unit++ ) {
		if ( 0 == amdEthInit( &mdrv->mp, unit, 
				  AMDETH_FLG_TX_MODE_LAZY
				| AMDETH_FLG_RX_MODE_SYNC
				| AMDETH_FLG_DO_RETRY
				| AMDETH_FLG_HDR_ETHERNET,
				RX_RING_SIZE, TX_RING_SIZE ) ) {
			/* fill ring */
			for ( i=0; i<RX_RING_SIZE; i++ ) {
				rbuf_t *buf;
				if ( ! ( buf = getrbuf() ) ) {
					fprintf(stderr,"drvAmdIpBasicSetup: enough buffers\n");
					/* NOTE: a leak here; we can't reclaim buffers we
					 * gave to the driver already.
					 */
					goto egress;
				}
				rx_adjusted(mdrv->mp, &buf);
			}
			return mdrv;
		}
	}

egress:
	if ( mdrv ) {
		relrbuf( mdrv->spare );
		if ( mdrv->mp )
			amdEthCloseDev(mdrv->mp);
		if ( mdrv->mutex )
			rtems_semaphore_delete( mdrv->mutex );
		free(mdrv);
	}
	return 0;
}

int
lanIpBscDrvShutdown(LanIpBscDrv drv_p)
{
amdeth_drv mdrv = (amdeth_drv)drv_p;

	if ( !drv_p )
		return 0;

	amdEthCloseDev(mdrv->mp);
	/* hack: just wait instead of synchronizing */
	rtems_task_wake_after(200);

	free(mdrv);
	return 0;
}
