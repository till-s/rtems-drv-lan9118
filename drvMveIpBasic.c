#include <stdint.h>
#include <rtems/rtems_mii_ioctl.h>

typedef struct mveth_drv_s_ *mveth_drv;

#define ETHERPADSZ sizeof(((EthHeaderRec*)0)->pad)
#define ETHERHDRSZ (sizeof(EthHeaderRec) - ETHERPADSZ) 

/* fwd decl of interface struct */
struct IpBscIfRec_;

/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */
#define NETDRV_ATOMIC_SEND_ARPREQ(pif, ipaddr)								\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			mveth_drv mdrv = (mveth_drv)(pif)->drv_p;						\
			int l_ = sizeof((pif)->arpreq) - ETHERPADSZ;					\
			memcpy(b, &(pif)->arpreq.ll.dst, l_);							\
			set_tpa((IpArpRec*)(b + ETHERHDRSZ), ipaddr);					\
			if ( mve_send_buf_locked(mdrv, b, b, l_) <= 0 )					\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pif, ptr, nbytes)							\
	do {} while (0)

int 
NETDRV_SND_PACKET(struct IpBscIfRec_ *pif, void *phdr, int hdrsz, void *data, int dtasz);

static inline int
mve_send_buf_locked(mveth_drv mdrv, void *pbuf, void *data, int len);

#define NETDRV_ENQ_BUFFER(pif, pbuf, nbytes)								\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
		mveth_drv mdrv = (mveth_drv)(pif)->drv_p;							\
																			\
		if ( mve_send_buf_locked(mdrv, pbuf, b_, l_) <= 0 )					\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

static inline void
NETDRV_READ_ENADDR(struct IpBscIfRec_ *pif, uint8_t *buf);

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
	struct mveth_private *mve_p;
	IpBscIf              ipbif_p;
	rtems_id             mutex;
	int                  unit; /* zero based */
	unsigned             flags;
	rtems_id             tid;
	int8_t               hasenaddr;
	uint8_t              enaddr[6];
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
			rval = BSP_mve_send_buf(mdrv->mve_p, pbuf, data, len);
		}
	DRVUNLOCK(mdrv);
	return rval;
}

int 
NETDRV_SND_PACKET(IpBscIf pif, void *phdr, int hdrsz, void *data, int dtasz)
{
mveth_drv            mdrv = pif->drv_p;
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

static inline void NETDRV_READ_ENADDR(IpBscIf pif, uint8_t *buf)
{
mveth_drv drvhdl = (mveth_drv)pif->drv_p;
	BSP_mve_read_eaddr(drvhdl->mve_p, buf);
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
rbuf_t      *b = buf;
mveth_drv mdrv = closure;

	if ( !buf ) {
		drvMveIpBasicRxDrop++;
		if ( len < 0 )
			drvMveIpBasicRxErrs++;
		return;
	}

	lanIpProcessBuffer(mdrv->ipbif_p, &b, len);

	relrbuf(b);
}

static void
mdrv_cleanup(mveth_drv mdrv)
{
	if ( mdrv->mve_p ) {
		BSP_mve_detach(mdrv->mve_p);
		mdrv->mve_p = 0;
	}
	if ( mdrv->mutex ) {
		rtems_semaphore_delete(mdrv->mutex);
		mdrv->mutex = 0;
	}
	if ( mdrv->tid ) {
		rtems_task_delete(mdrv->tid);
		mdrv->tid = 0;
	}
}

LanIpBscDrv
lanIpBscDrvCreate(int unit, uint8_t *enaddr)
{
mveth_drv             mdrv = 0;
rtems_status_code     sc;
int                   minu, maxu;

	if ( ! (mdrv = calloc(1, sizeof(*mdrv))) ) {
		return 0;
	}

	mdrv->flags |= IF_FLG_STOPPED;

	/* Save ethernet addr. if given; the mve low-level driver
	 * is not yet ready to use it at this point
	 */
	if ( enaddr ) {
		mdrv->hasenaddr = 1;
		memcpy(mdrv->enaddr, enaddr, 6);
	}

	/* Create driver mutex */
	sc = rtems_semaphore_create(
				rtems_build_name('m','v','e','L'),
				1,
				RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY
				| RTEMS_NO_PRIORITY_CEILING | RTEMS_LOCAL,
				0,
				&mdrv->mutex);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvMveIpBasic: unable to create driver mutex");
		goto egress;
	}

	/* Create driver task but don't start yet */
	sc = rtems_task_create(
				rtems_build_name('i','p','b','d'),
				20,
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&mdrv->tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvMveIpBasic: unable to create driver task");
		goto egress;
	}

	/* Initialize low-level driver; it needs a task ID already -- that's
	 * why we had to create the task upfront.
	 */
	if ( unit < 0 ) {
		minu = 1;
		maxu = 3;
	} else {
		unit++;	/* mve units are 1-based, lanIpBasic units are 0 based */
		minu = unit;
		maxu = minu+1;
	}

	for ( unit=minu; unit < maxu; unit++ ) {
		if ( (mdrv->mve_p = BSP_mve_setup( unit,
								mdrv->tid,
								cleanup_txbuf, mdrv,
								alloc_rxbuf,
								consume_rxbuf, mdrv,
								RX_RING_SIZE, TX_RING_SIZE,
								BSP_MVE_IRQ_TX | BSP_MVE_IRQ_RX /* | BSP_MVE_IRQ_LINK */ )) ) {
			/* SUCCESSFUL EXIT (unit found and initialized) */
			fprintf(stderr,"drvMveIpBasic: using mve instance %u\n",unit);
			mdrv->unit = unit - 1;
			return mdrv;
		}
	}
	/* if we get here the requested unit was already busy */

	sc = RTEMS_TOO_MANY;
	rtems_error(sc, "drvMveIpBasic: no free low-level driver slot found");
	/* fall through */

egress:

	mdrv_cleanup(mdrv);
	free(mdrv);
	return 0;
}

#define KILL_EVENT RTEMS_EVENT_4

#ifndef SIOCGIFMEDIA
/* BSP_mve_media_ioctl() aliases '0' to SIOCGIFMEDIA :-) */
#define SIOCGIFMEDIA 0
#endif

void
drvMveIpBasicTask(rtems_task_argument arg)
{
IpBscIf               ipbif_p = (IpBscIf)arg;
mveth_drv             mdrv    = lanIpBscIfGetDrv(ipbif_p);
struct mveth_private  *mve_p  = mdrv->mve_p; 
rtems_event_set       ev_mask = (1<<mdrv->unit) | KILL_EVENT;
rtems_event_set       evs;
uint32_t              irqs;
int                   media;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task starting up\n");
	}
#endif

	BSP_mve_init_hw( mve_p, 0, mdrv->hasenaddr ? mdrv->enaddr : 0 );

	if ( 0 == BSP_mve_media_ioctl(mp, SIOCGIFMEDIA, &media) ) {
		if ( (IFM_LINK_OK & media) ) {
			mdrv->flags &= ~IF_FLG_STOPPED;
		}
	} else {
		fprintf(stderr,"WARNING: unable to determine link state; may be unable to send\n");
	}

	do {
		rtems_event_receive( ev_mask, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs);

		irqs = BSP_mve_ack_irqs(mve_p);

		if ( (irqs & BSP_MVE_IRQ_TX) ) {
		DRVLOCK(mdrv);
			/* cleanup_txbuf */
			BSP_mve_swipe_tx(mve_p);
		DRVUNLOCK(mdrv);
		}
		if ( (irqs & BSP_MVE_IRQ_RX) ) {
			/* alloc_rxbuf, consume_rxbuf */
			BSP_mve_swipe_rx(mve_p);
		}
		if ( (irqs & BSP_MVE_IRQ_LINK) ) {
			/* propagate link change to serial port */
		DRVLOCK(mdrv);
			BSP_mve_ack_link_chg(mp, &media); /* propagate link change to serial port */
			if ( (IFM_LINK_OK & media) ) {
				mdrv->flags &= ~IF_FLG_STOPPED;
			} else {
				mdrv->flags |=  IF_FLG_STOPPED;
			}
		DRVUNLOCK(mdrv);
		}
		BSP_mve_enable_irqs(mve_p);
	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task received shutting down\n");
	}
#endif

	/* prevent this task from being deleted in cleanup routine */
	mdrv->tid = 0;

	mdrv_cleanup(mdrv);

	free(mdrv);

	rtems_task_delete(RTEMS_SELF);
}

int
lanIpBscDrvStart(IpBscIf ipbif_p, int pri)
{
mveth_drv           mdrv = lanIpBscIfGetDrv(ipbif_p);
rtems_status_code   sc;
rtems_task_priority op;

	/* sanity check: has driver been attached to interface yet ? */
	if ( !mdrv ) {
		sc = RTEMS_NOT_DEFINED;
		rtems_error(sc, "drvMveIpBasic: driver not attached to interface yet?");
		return sc;
	}

	/* sanity check: has driver already been started? */
	if ( mdrv->ipbif_p ) {
		sc = RTEMS_RESOURCE_IN_USE;
		rtems_error(sc, "drvMveIpBasic: driver already started\n");
		return sc;
	}

	if ( pri > 0 ) {
		sc = rtems_task_set_priority(mdrv->tid, pri, &op);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc,"drvMveIpBasic: unable to change driver task priority");
			return sc;
		}
	}

	/* Link to IF; non-NULL -ness of the IF pointer in the mveth_drv struct
	 * also server as a flag that the task has been started.
	 */
	mdrv->ipbif_p = ipbif_p;

	sc = rtems_task_start( mdrv->tid, drvMveIpBasicTask, (rtems_task_argument)ipbif_p);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvMveIpBasic: unable to start driver task");
		mdrv->ipbif_p = 0; /* mark as not started */
	}

	/* don't clean the driver struct; leave everything as it was on entry */
	return (int)sc;
}

int
lanIpBscDrvShutdown(LanIpBscDrv drv_p)
{
mveth_drv mdrv = drv_p;

	if ( !mdrv )
		return 0;

	if ( mdrv->ipbif_p ) {
		/* Has already been started; driver task must cleanup */
		rtems_event_send(mdrv->tid, KILL_EVENT);
		/* hack: just wait instead of synchronizing */
		rtems_task_wake_after(200);
	} else {
		/* Not started yet */
		mdrv_cleanup(mdrv);
		free(mdrv);
	}
	return 0;
}
