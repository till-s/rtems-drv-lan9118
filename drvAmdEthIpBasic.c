#include <stdint.h>
/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */

typedef struct amdeth_drv_s_ *amdeth_drv;

#define ETHERPADSZ sizeof(((EthHeaderRec*)0)->pad)


#define NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr)								\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			amdeth_drv mdrv = (amdeth_drv)(pd)->drv_p;						\
			int l_ = sizeof((pd)->arpreq) - ETHERPADSZ;						\
			memcpy(b, &(pd)->arpreq, sizeof((pd)->arpreq));					\
			memcpy(((LanArp)b)->arp.tpa, &ipaddr, sizeof(ipaddr));			\
			if ( amd_send_buf_locked(mdrv, b, b+ETHERPADSZ, l_) )			\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)							\
	do {} while (0)

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz);

static inline int
amd_send_buf_locked(amdeth_drv mdrv, void *pbuf, void *data, int len);

#define NETDRV_ENQ_BUFFER(pd, pbuf, nbytes)									\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
		amdeth_drv mdrv = (amdeth_drv)(pd)->drv_p;							\
																			\
		if ( amd_send_buf_locked(mdrv, pbuf, b_, l_) )						\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

static inline void
NETDRV_READ_ENADDR(amdeth_drv drvhdl, uint8_t *buf);

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

#define DRVLOCK(drv) assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain( (drv)->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )
#define DRVUNLOCK(drv) assert( RTEMS_SUCCESSFUL == rtems_semaphore_release( (drv)->mutex) )


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
		rval = amdEthSendPacketSwp( mdrv->mp, dummyh, &data, len - 14 );
		if ( !rval && data ) {
			relrbuf((rbuf_t*)((char*)data - ETHERPADSZ));
		}
	DRVUNLOCK(mdrv);
	return rval;
}

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz)
{
amdeth_drv            mdrv = pdrv;
char                 *b_ = (char*)getrbuf();
char                 *p;

		if ( (p=b_) ) {
			int l_ = hdrsz + dtasz - ETHERPADSZ;
			if ( phdr ) {
				memcpy(p, phdr, hdrsz);
				p += hdrsz;
			}
			memcpy(p, data, dtasz);
			p = b_ + ETHERPADSZ;


			if ( amd_send_buf_locked(mdrv, b_, p, l_) ) {
				relrbuf((rbuf_t*)b_);
				return -ENOSPC;
			}
			return dtasz;
		}
		return -ENOMEM;
}

static inline void NETDRV_READ_ENADDR(amdeth_drv drvhdl, uint8_t *buf)
{
EtherHeaderRec h;
	amdEthHeaderInit(&h, 0, drvhdl->mp);
	memcpy(buf, h.src, sizeof(h.src));
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
	st = amdEthReceivePacket( d, &buf.raw, LANPKTMAX );
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

	relrbuf(mdrv->spare);
	rtems_semaphore_delete(mdrv->mutex);
	free(mdrv);

	rtems_task_delete(RTEMS_SELF);
}

void *
drvAmdIpBasicSetup(IpBscIf ipbif)
{
int                   unit, i;
amdeth_drv            mdrv = 0;

	if ( ! (mdrv = malloc(sizeof(*mdrv))) ) {
		return 0;
	}

	memset(mdrv, 0, sizeof(*mdrv));

	if ( ! (mdrv->spare = getrbuf()) ) {
		goto egress;
	}

	mdrv->mutex = 0;

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
								rtems_build_name('m','v','e','L'),
								1,
								RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
								0,
								&mdrv->mutex) ) {
			goto egress;
	}

	for ( unit=0; unit < 3; unit++ ) {
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

void
drvAmdIpBasicShutdown(void *mdrv)
{
	amdEthCloseDev(((amdeth_drv)mdrv)->mp);
	/* hack: just wait instead of synchronizing */
	rtems_task_wake_after(200);
}
