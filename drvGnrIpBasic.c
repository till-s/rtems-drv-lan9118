#include <stdint.h>

typedef struct gnreth_drv_s_ *gnreth_drv;

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
			gnreth_drv gdrv = (gnreth_drv)(pif)->drv_p;						\
			int l_ = sizeof((pif)->arpreq) - ETHERPADSZ;					\
			memcpy(b, &(pif)->arpreq.ll.dst, l_);							\
			set_tpa((IpArpRec*)(b + ETHERHDRSZ), ipaddr);					\
			if ( gnr_send_buf_locked(gdrv, b, b, l_) <= 0 )					\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pif, ptr, nbytes)							\
	do {} while (0)

static inline int
gnr_send_buf_locked(gnreth_drv gdrv, void *pbuf, void *data, int len);

#define NETDRV_ENQ_BUFFER(pif, pbuf, nbytes)								\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
		gnreth_drv gdrv = (gnreth_drv)(pif)->drv_p;							\
																			\
		if ( gnr_send_buf_locked(gdrv, pbuf, b_, l_) <= 0 )					\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

static inline void
NETDRV_READ_ENADDR(struct IpBscIfRec_ *pif, uint8_t *buf);

static inline void
NETDRV_MC_FILTER_ADD(struct IpBscIfRec_ *pif, uint8_t *enaddr);

static inline void
NETDRV_MC_FILTER_DEL(struct IpBscIfRec_ *pif, uint8_t *enaddr);

static inline int
NETDRV_START(struct IpBscIfRec_ *pif, int pri);

static inline int
NETDRV_SHUTDOWN(void *drv_p);

#define NETDRV_INCLUDE "gnreth_lldrv.h"

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

#include <rtems/rtems_mii_ioctl.h>

#define IF_FLG_STOPPED	(1<<0)

#define KILL_EVENT RTEMS_EVENT_7
#define IRQ_EVENT  RTEMS_EVENT_0

#ifndef SIOCGIFMEDIA
/* media_ioctl() aliases '0' to SIOCGIFMEDIA :-) */
#define SIOCGIFMEDIA 0
#endif


typedef struct gnreth_drv_s_ {
	IpBscIf              ipbif_p;
	rtems_id             mutex;
	int                  unit; /* zero based */
	unsigned             flags;
	rtems_id             rx_tid;
	rtems_id             tx_tid;
	int8_t               hasenaddr;
	uint8_t              enaddr[6];
	struct IpBscLLDrv_   lldrv;
} gnreth_drv_s;

#define DRVLOCK(drv)   mutex_lock( (drv)->mutex )
#define DRVUNLOCK(drv) mutex_unlk( (drv)->mutex )

static inline int
gnr_send_buf_locked(gnreth_drv gdrv, void *pbuf, void *data, int len)
{
int rval;
	DRVLOCK(gdrv);
		/* For now - never drop.  Sometimes it is useful to switch
		 * the PHY to 'loopback' mode in which case the link goes
		 * 'away' but we still want to send.
		 */
		if ( 0 && (gdrv->flags & IF_FLG_STOPPED) ) {
			/* drop */
			rval = 0;
		} else
		{
			rval = gdrv->lldrv.send_buf(gdrv->lldrv.dev, pbuf, data, len);
		}
	DRVUNLOCK(gdrv);
	return rval;
}

static inline void NETDRV_READ_ENADDR(IpBscIf pif, uint8_t *buf)
{
gnreth_drv drvhdl = (gnreth_drv)pif->drv_p;
	drvhdl->lldrv.read_eaddr(drvhdl->lldrv.dev, buf);
}

static inline void
NETDRV_MC_FILTER_ADD(struct IpBscIfRec_ *pif, uint8_t *enaddr)
{
gnreth_drv drvhdl = (gnreth_drv)pif->drv_p;
	DRVLOCK( drvhdl );
		drvhdl->lldrv.mc_filter_add(drvhdl->lldrv.dev, enaddr);
	DRVUNLOCK( drvhdl );
}

static inline void
NETDRV_MC_FILTER_DEL(struct IpBscIfRec_ *pif, uint8_t *enaddr)
{
gnreth_drv drvhdl = (gnreth_drv)pif->drv_p;
	DRVLOCK( drvhdl );
		drvhdl->lldrv.mc_filter_del(drvhdl->lldrv.dev, enaddr);
	DRVUNLOCK( drvhdl );
}

static void
cleanup_txbuf(void *buf, void *closure, int error_on_tx_occurred)
{
	/* tx starts at offset 2 -- align back to get buffer address */
	relrbuf((void*)((uint32_t)buf & ~3));
}

static void *
alloc_rxbuf(int *p_size, uintptr_t *p_data_addr)
{
	*p_size = (*p_data_addr = (uintptr_t)getrbuf()) ? LANPKTMAX : 0;
	return (void*) *p_data_addr;
}

int drvGnrethIpBasicRxErrs  = 0;
int drvGnrethIpBasicRxDrop  = 0;

static void
gdrv_isr(void *arg)
{
gnreth_drv gdrv = arg;
uint32_t   pending;

	/* Findout what interrupts are pending w/o clearing them */
	pending = gdrv->lldrv.ack_irqs(gdrv->lldrv.dev, 0);

	/* Disable pending interrupts                            */
	pending &= gdrv->lldrv.dis_irqs(gdrv->lldrv.dev, pending);

	/* Post IRQ event to tasks. Note that this could lead to
	 * an event being sent multiple times but that is not 
	 * considered harmful since the driver task polls for the
	 * interrupt cause:
	 *    TX irq raised, detected pending, event posted.
	 * If at this point an RX interrupt happens (before the
	 * TX task has cleared the TX interrupt) then an RX event
	 * but also a second TX event is posted (since the TX irq is
	 * still pending).
	 */
	if ( (pending & (gdrv->lldrv.tx_irq_msk | gdrv->lldrv.ln_irq_msk)) )
		rtems_event_send(gdrv->tx_tid, IRQ_EVENT);
	if ( (pending & gdrv->lldrv.rx_irq_msk) )
		rtems_event_send(gdrv->rx_tid, IRQ_EVENT);
}

static void
consume_rxbuf(void *buf, void *closure, int len)
{
rbuf_t      *b = buf;
gnreth_drv gdrv = closure;

	if ( !buf ) {
		drvGnrethIpBasicRxDrop++;
		if ( len < 0 )
			drvGnrethIpBasicRxErrs++;
		return;
	}

	lanIpProcessBuffer(gdrv->ipbif_p, &b, len);

	relrbuf(b);
}

static void
gdrv_cleanup(gnreth_drv gdrv)
{
	if ( gdrv->lldrv.dev ) {
		gdrv->lldrv.detach(gdrv->lldrv.dev);
		gdrv->lldrv.dev = 0;
	}
	if ( gdrv->mutex ) {
		rtems_semaphore_delete(gdrv->mutex);
		gdrv->mutex = 0;
	}
	if ( gdrv->rx_tid ) {
		rtems_task_delete(gdrv->rx_tid);
		gdrv->rx_tid = 0;
	}
	if ( gdrv->tx_tid ) {
		rtems_task_delete(gdrv->tx_tid);
		gdrv->tx_tid = 0;
	}
	free(gdrv);
}

LanIpBscDrv
lanIpBscDrvCreate(int unit, uint8_t *enaddr)
{
gnreth_drv             gdrv = 0;
rtems_status_code     sc;
int                   minu, maxu;
uint32_t              irq_msk;

	if ( 0 == &drvGnrethIpBasicLLDrv || 0 == drvGnrethIpBasicLLDrv ) {
		fprintf(stderr,"drvGnrethIpBasic: No low-level driver hooked\n");
		return 0;
	}

	if ( ! (gdrv = calloc(1, sizeof(*gdrv))) ) {
		return 0;
	}

	gdrv->flags |= IF_FLG_STOPPED;

	gdrv->lldrv = *drvGnrethIpBasicLLDrv;

	/* Save ethernet addr. if given; the low-level driver
	 * is not yet ready to use it at this point
	 */
	if ( enaddr ) {
		gdrv->hasenaddr = 1;
		memcpy(gdrv->enaddr, enaddr, 6);
	}

	/* Create driver mutex */
	if ( ! (gdrv->mutex = bsem_create("ipbd", SEM_MUTX)) ) {
		fprintf(stderr, "drvGnrethIpBasic: unable to create driver mutex");
		goto egress;
	}

	/* Create driver tasks but don't start yet */
	sc = rtems_task_create(
				rtems_build_name('i','p','b','R'),
				20,
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&gdrv->rx_tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to create RX driver task");
		goto egress;
	}

	sc = rtems_task_create(
				rtems_build_name('i','p','b','T'),
				80,
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&gdrv->tx_tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to create TX driver task");
		goto egress;
	}

	/* Initialize low-level driver; it may already generate interrupts
	 * -- that's why we had to create the tasks upfront.
	 */
	if ( unit < 0 ) {
		minu = 1;
		maxu = 3;
	} else {
		unit++;	/* low-level driver units are 1-based, lanIpBasic units are 0 based */
		minu = unit;
		maxu = minu+1;
	}

	irq_msk  = 0;
	irq_msk |= gdrv->lldrv.tx_irq_msk;
	irq_msk |= gdrv->lldrv.rx_irq_msk;
	irq_msk |= gdrv->lldrv.ln_irq_msk;

	for ( unit=minu; unit < maxu; unit++ ) {
		if ( (gdrv->lldrv.dev = gdrv->lldrv.setup(unit,
								gdrv_isr, gdrv,
								cleanup_txbuf, gdrv,
								alloc_rxbuf,
								consume_rxbuf, gdrv,
								RX_RING_SIZE, TX_RING_SIZE,
								irq_msk)) ) {
			/* SUCCESSFUL EXIT (unit found and initialized) */
			fprintf(stderr,"drvGnrethIpBasic: using device instance %u\n",unit);
			gdrv->unit = unit - 1;

			gdrv->lldrv.init_hw( gdrv->lldrv.dev, 0, gdrv->hasenaddr ? gdrv->enaddr : 0 );

			task_init( gdrv->rx_tid );
			task_init( gdrv->tx_tid );

			return gdrv;
		}
	}
	/* if we get here the requested unit was already busy */

	sc = RTEMS_TOO_MANY;
	rtems_error(sc, "drvGnrethIpBasic: no free low-level driver slot found");
	/* fall through */

egress:

	gdrv_cleanup(gdrv);
	return 0;
}

LLDev
drvGnrethLLDev(gnreth_drv gdrv)
{
	return gdrv->lldrv.dev;
}

void
drvGnrethIpBasicRXTask(rtems_task_argument arg)
{
IpBscIf               ipbif_p = (IpBscIf)arg;
gnreth_drv            gdrv    = lanIpBscIfGetDrv(ipbif_p);
LLDev                 lldev   = gdrv->lldrv.dev;
LLDrv                 lldrv   = &gdrv->lldrv;
rtems_event_set       ev_mask = IRQ_EVENT | KILL_EVENT;
rtems_event_set       evs;
uint32_t              irqs;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP RX task starting up\n");
	}
#endif

	do {
		rtems_event_receive( ev_mask, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs);

		irqs = lldrv->ack_irqs(lldev, lldrv->rx_irq_msk);

		if ( (irqs & lldrv->rx_irq_msk) ) {
			/* alloc_rxbuf, consume_rxbuf */
			lldrv->swipe_rx(lldev);
		}
		lldrv->enb_irqs(lldev, lldrv->rx_irq_msk);

	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP RX task received shutting down\n");
	}
#endif

	/* prevent this task from being deleted in cleanup routine */
	gdrv->rx_tid = 0;

	task_leave();
}

/* The TX task does lower-priority work */
void
drvGnrethIpBasicTXTask(rtems_task_argument arg)
{
IpBscIf               ipbif_p = (IpBscIf)arg;
gnreth_drv            gdrv    = lanIpBscIfGetDrv(ipbif_p);
LLDev                 lldev   = gdrv->lldrv.dev;
LLDrv                 lldrv   = &gdrv->lldrv;
rtems_event_set       ev_mask = IRQ_EVENT | KILL_EVENT;
rtems_event_set       evs;
uint32_t              irqs, my_irqs;
int                   media;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP TX task starting up\n");
	}
#endif

	if ( 0 == lldrv->med_ioctl( lldev, SIOCGIFMEDIA, &media ) ) {
		if ( (IFM_LINK_OK & media) ) {
			gdrv->flags &= ~IF_FLG_STOPPED;
		}
	} else {
		fprintf(stderr,"WARNING: unable to determine link state; may be unable to send\n");
	}
	
	my_irqs = lldrv->ln_irq_msk | lldrv->tx_irq_msk;

	do {
		rtems_event_receive( ev_mask, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs);

		irqs = lldrv->ack_irqs(lldev, my_irqs);

		if ( (irqs & lldrv->tx_irq_msk) ) {
		DRVLOCK(gdrv);
			/* cleanup_txbuf */
			lldrv->swipe_tx(lldev);
		DRVUNLOCK(gdrv);
		}
		if ( (irqs & lldrv->ln_irq_msk) ) {
			/* propagate link change to serial port */
		DRVLOCK(gdrv);
			lldrv->ack_ln_chg(lldev, &media); /* propagate link change to serial port */
			if ( (IFM_LINK_OK & media) ) {
				gdrv->flags &= ~IF_FLG_STOPPED;
			} else {
				gdrv->flags |=  IF_FLG_STOPPED;
			}
		DRVUNLOCK(gdrv);
		}
		lldrv->enb_irqs(lldev, my_irqs);
	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP TX task received shutting down\n");
	}
#endif

	/* prevent this task from being deleted in cleanup routine */
	gdrv->tx_tid = 0;

	task_leave();
}

static inline int
NETDRV_START(IpBscIf ipbif_p, int pri)
{
gnreth_drv           gdrv = lanIpBscIfGetDrv(ipbif_p);
rtems_status_code   sc;
rtems_task_priority op;

	/* sanity check: has driver been attached to interface yet ? */
	if ( !gdrv ) {
		sc = RTEMS_NOT_DEFINED;
		rtems_error(sc, "drvGnrethIpBasic: driver not attached to interface yet?");
		return sc;
	}

	/* sanity check: has driver already been started? */
	if ( gdrv->ipbif_p ) {
		sc = RTEMS_RESOURCE_IN_USE;
		rtems_error(sc, "drvGnrethIpBasic: driver already started\n");
		return sc;
	}

	if ( pri > 0 ) {
		sc = rtems_task_set_priority(gdrv->rx_tid, pri, &op);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc,"drvGnrethIpBasic: unable to change driver RX task priority");
			return sc;
		}
	}

	/* Link to IF; non-NULL -ness of the IF pointer in the gnreth_drv struct
	 * also server as a flag that the task has been started.
	 */
	gdrv->ipbif_p = ipbif_p;

	sc = rtems_task_start( gdrv->rx_tid, drvGnrethIpBasicRXTask, (rtems_task_argument)ipbif_p);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to start driver RX task");
		gdrv->ipbif_p = 0; /* mark as not started */
		return sc;
	}

	sc = rtems_task_start( gdrv->tx_tid, drvGnrethIpBasicTXTask, (rtems_task_argument)ipbif_p);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to start driver TX task");
		gdrv->ipbif_p = 0; /* mark as not started */
		rtems_panic("drvGnrethIpBasic: cannot recover\n");
		/* NEVER GET HERE */
		return sc;
	}

	/* don't clean the driver struct; leave everything as it was on entry */
	return (int)sc;
}

static inline int
NETDRV_SHUTDOWN(LanIpBscDrv drv_p)
{
gnreth_drv gdrv = drv_p;

	if ( !gdrv )
		return 0;

	if ( gdrv->ipbif_p ) {
		task_killer killer;
		killer.kill_event = KILL_EVENT;

		/* Has already been started */
		task_pseudojoin( gdrv->rx_tid, KILL_BY_EVENT, killer );
		task_pseudojoin( gdrv->tx_tid, KILL_BY_EVENT, killer );

	} else {
		/* Not started yet */
	}
	gdrv_cleanup(gdrv);
	return 0;
}
