#include <stdint.h>

#include <rtems/rtems_mii_ioctl.h>

typedef struct mveth_drv_s_ *mveth_drv;

#define ETHERPADSZ sizeof(((EthHeaderRec*)0)->pad)
#define ETHERHDRSZ (sizeof(EthHeaderRec) - ETHERPADSZ) 

/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */

#define NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr)								\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			mveth_drv mdrv = (mveth_drv)(pd)->drv_p;						\
			int l_ = sizeof((pd)->arpreq) - ETHERPADSZ;						\
			memcpy(b, &(pd)->arpreq.ll.dst, l_);							\
			*(uint32_t*)((IpArpRec*)(b + ETHERHDRSZ))->tpa = ipaddr;		\
			if ( mve_send_buf_locked(mdrv, b, b, l_) <= 0 )					\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)							\
	do {} while (0)

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz);

static inline int
mve_send_buf_locked(mveth_drv mdrv, void *pbuf, void *data, int len);

#define NETDRV_ENQ_BUFFER(pd, pbuf, nbytes)									\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
		mveth_drv mdrv = (mveth_drv)(pd)->drv_p;							\
																			\
		if ( mve_send_buf_locked(mdrv, pbuf, b_, l_) <= 0 )					\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

static inline void
NETDRV_READ_ENADDR(mveth_drv drvhdl, uint8_t *buf);

#define NETDRV_INCLUDE	<bsp/if_mve_pub.h>

/* RX buffers must be 64-bit aligned (8 byte)
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

#define NRBUFS (RX_RING_SIZE + RX_RING_SIZE*20 + 20)

#define TX_RING_SIZE (N_SATELLITES/4 + 1)


#include "lanIpBasic.c"

#define IF_FLG_STOPPED	(1<<0)

typedef struct mveth_drv_s_ {
	struct mveth_private *mp;
	rtems_id             mutex;
	int                  unit; /* zero based */
	unsigned             flags;
} mveth_drv_s;

#define DRVLOCK(drv) assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain( (drv)->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )
#define DRVUNLOCK(drv) assert( RTEMS_SUCCESSFUL == rtems_semaphore_release( (drv)->mutex) )


static inline int
mve_send_buf_locked(mveth_drv mdrv, void *pbuf, void *data, int len)
{
int rval;
	DRVLOCK(mdrv);
		if ( (mdrv->flags & IF_FLG_STOPPED) ) {
			/* drop */
			rval = 0;
		} else
		{
			rval = BSP_mve_send_buf(mdrv->mp, pbuf, data, len);
		}
	DRVUNLOCK(mdrv);
	return rval;
}

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz)
{
mveth_drv            mdrv = pdrv;
char                 *b_ = (char*)getrbuf();
char                 *p;
int                  st;

		if ( (p=b_) ) {
			int l_ = hdrsz + dtasz - ETHERPADSZ;
			if ( phdr ) {
				memcpy(p, phdr, hdrsz);
				p += hdrsz;
			}
			memcpy(p, data, dtasz);
			p = b_ + ETHERPADSZ;


			if ( (st = mve_send_buf_locked(mdrv, b_, p, l_)) <= 0 ) {
				/* If nothing was sent (packet dropped) don't report
				 * an error but release the buffer.
				 */
				relrbuf((rbuf_t*)b_);
				return st < 0 ? -ENOSPC : 0;
			}
			return dtasz;
		}
		return -ENOMEM;
}

static inline void NETDRV_READ_ENADDR(mveth_drv drvhdl, uint8_t *buf)
{
	BSP_mve_read_eaddr(drvhdl->mp, buf);
}

static void
cleanup_txbuf(void *buf, void *closure, int error_on_tx_occurred)
{
	/* tx starts at offset 2 -- align back to get buffer address */
	relrbuf((void*)((uint32_t)buf & ~3));
}

static void *
alloc_rxbuf(int *p_size, unsigned long *p_data_addr)
{
	*p_size = (*p_data_addr = (unsigned long)getrbuf()) ? LANPKTMAX : 0;
	return (void*) *p_data_addr;
}

int drvMveIpBasicRxErrs = 0;
int drvMveIpBasicRxDrop = 0;

static void
consume_rxbuf(void *buf, void *closure, int len)
{
rbuf_t   *b = buf;
IpBscIf pd = closure;

	if ( !buf ) {
		drvMveIpBasicRxDrop++;
		if ( len < 0 )
			drvMveIpBasicRxErrs++;
		return;
	}

	lanIpProcessBuffer(pd, &b, len);

	relrbuf(b);
}

#define KILL_EVENT RTEMS_EVENT_4

#ifndef SIOCGIFMEDIA
/* BSP_mve_media_ioctl() aliases '0' to SIOCGIFMEDIA :-) */
#define SIOCGIFMEDIA 0
#endif


void
drvMveIpBasicTask(rtems_task_argument arg)
{
IpBscIf               ipbif   = (IpBscIf)arg;
mveth_drv             mdrv    = lanIpBscIfGetDrv(ipbif);
struct mveth_private  *mp     = mdrv->mp; 
rtems_event_set       ev_mask = (1<<mdrv->unit) | KILL_EVENT;
rtems_event_set       evs;
uint32_t              irqs;
int                   media;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task starting up\n");
	}
#endif

	BSP_mve_init_hw( mp, 0, 0 );

	if ( 0 == BSP_mve_media_ioctl(mp, SIOCGIFMEDIA, &media) ) {
		if ( (IFM_LINK_OK & media) ) {
			mdrv->flags &= ~IF_FLG_STOPPED;
		}
	} else {
		fprintf(stderr,"WARNING: unable to determine link state; may be unable to send\n");
	}

	do {
		rtems_event_receive( ev_mask, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs );
		irqs = BSP_mve_ack_irqs(mp);
		if ( (irqs & BSP_MVE_IRQ_TX) ) {
		DRVLOCK(mdrv);
			BSP_mve_swipe_tx(mp); /* cleanup_txbuf */
		DRVUNLOCK(mdrv);
		}
		if ( (irqs & BSP_MVE_IRQ_RX) ) {
			BSP_mve_swipe_rx(mp); /* alloc_rxbuf, consume_rxbuf */
		}
		if ( (irqs & BSP_MVE_IRQ_LINK) ) {
		DRVLOCK(mdrv);
			BSP_mve_ack_link_chg(mp, &media); /* propagate link change to serial port */
			if ( (IFM_LINK_OK & media) ) {
				mdrv->flags &= ~IF_FLG_STOPPED;
			} else {
				mdrv->flags |=  IF_FLG_STOPPED;
			}

		DRVUNLOCK(mdrv);
		}
		BSP_mve_enable_irqs(mp);
	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task received shutting down\n");
	}
#endif
	BSP_mve_detach( mp );

	rtems_semaphore_delete(mdrv->mutex);
	free(mdrv);

	rtems_task_delete(RTEMS_SELF);
}

rtems_id
drvMveIpBasicSetup(IpBscIf ipbif)
{
int                   unit;
mveth_drv             mdrv = 0;
rtems_id              tid  = 0;

	if ( ! (mdrv = malloc(sizeof(*mdrv))) ) {
		return 0;
	}

	mdrv->mutex = 0;
	mdrv->flags = IF_FLG_STOPPED;

	if ( RTEMS_SUCCESSFUL != rtems_semaphore_create(
								rtems_build_name('m','v','e','L'),
								1,
								RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
								0,
								&mdrv->mutex) ) {
			goto egress;
	}

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('i','p','b','d'),
								20,	/* can be changed later */
								10000,
								RTEMS_DEFAULT_MODES,
								RTEMS_FLOATING_POINT | RTEMS_LOCAL,
								&tid) )
			goto egress;

	for ( unit=1; unit < 3; unit++ ) {
		if ( (mdrv->mp = BSP_mve_setup( unit,
								tid,
								cleanup_txbuf, ipbif,
								alloc_rxbuf,
								consume_rxbuf, ipbif,
								RX_RING_SIZE, TX_RING_SIZE,
								BSP_MVE_IRQ_TX | BSP_MVE_IRQ_RX | BSP_MVE_IRQ_LINK)) ) {
			rtems_task_set_note( tid, RTEMS_NOTEPAD_0, (uint32_t)mdrv );
			mdrv->unit = unit - 1;
			return tid;
		}
	}

egress:
	if ( mdrv ) {
		if ( mdrv->mutex )
			rtems_semaphore_delete( mdrv->mutex );
		free(mdrv);
	}
	if ( tid )
		rtems_task_delete(tid);
	return 0;
}

void *
drvMveIpBasicGetDrv(rtems_id tid)
{
uint32_t rval;
	if ( RTEMS_SUCCESSFUL == rtems_task_get_note( tid, RTEMS_NOTEPAD_0, &rval) )
		return (void*)rval;
	return 0;
}

void
drvMveIpBasicShutdown(rtems_id tid)
{
	rtems_event_send(tid, KILL_EVENT);
	/* hack: just wait instead of synchronizing */
	rtems_task_wake_after(200);
}
