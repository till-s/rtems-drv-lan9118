#include <stdint.h>
#include <rtems/rtems_mii_ioctl.h>

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

/*#define NETDRV_INCLUDE	<bsp/if_mve_pub.h>*/
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

#define IF_FLG_STOPPED	(1<<0)

#if 0
typedef struct IpBscLLDrv_ *LLDrv;
typedef void               *LLDev;

struct IpBscLLDrv_ {
	LLDev        dev;
	uint32_t     tx_irq_msk;
	uint32_t     rx_irq_msk;
	uint32_t     ln_irq_msk;
	void    	*(*setup)(
					int      unit,
					rtems_id tid,
               		void     (*cleanup_txbuf)(void*, void*, int),
					void     *cleanup_txbuf_arg,
					void     *(*alloc_rxbuf)(int*, unsigned long*),
					void     (*consume_rxbuf)(void*, void*, int),
					void     *consume_rxbuf_arg,
					int      rx_ring_size,
					int      tx_ring_size,
					uint32_t irq_mask);
	int     	(*detach)(void*);
	void    	(*read_eaddr)(void *, unsigned char *);
	void    	(*init_hw)(void*, int, unsigned char *);	
	uint32_t    (*ack_irqs)(void*);
	void        (*enb_irqs)(void*);
	int         (*ack_ln_chg)(void*, int*);
	int         (*swipe_tx)(void*);
	int         (*swipe_rx)(void*);
	int         (*send_buf)(void*, void*, void*, int);
	int         (*med_ioctl)(void*, int, int*);
} ipBscLLDrvMve = {
	tx_irq_msk:  BSP_MVE_IRQ_TX,
	rx_irq_msk:  BSP_MVE_IRQ_RX,
	ln_irq_msk:  /* BSP_MVE_IRQ_LINK */ 0,
	setup     :  (void*) BSP_mve_setup,
	detach    :  (void*) BSP_mve_detach,
	read_eaddr:  (void*) BSP_mve_read_eaddr,
	init_hw   :  (void*) BSP_mve_init_hw,
	ack_irqs  :  (void*) BSP_mve_ack_irqs,
	enb_irqs  :  (void*) BSP_mve_enable_irqs,
	ack_ln_chg:  (void*) BSP_mve_ack_link_chg,
	swipe_tx  :  (void*) BSP_mve_swipe_tx,
	swipe_rx  :  (void*) BSP_mve_swipe_rx,
	send_buf  :  (void*) BSP_mve_send_buf,
	med_ioctl :  (void*) BSP_mve_media_ioctl,
};
#endif

typedef struct gnreth_drv_s_ {
	IpBscIf              ipbif_p;
	rtems_id             mutex;
	int                  unit; /* zero based */
	unsigned             flags;
	rtems_id             tid;
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
		if ( (gdrv->flags & IF_FLG_STOPPED) ) {
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
alloc_rxbuf(int *p_size, unsigned long *p_data_addr)
{
	*p_size = (*p_data_addr = (unsigned long)getrbuf()) ? LANPKTMAX : 0;
	return (void*) *p_data_addr;
}

int drvGnrethIpBasicRxErrs  = 0;
int drvGnrethIpBasicRxDrop  = 0;

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
	if ( gdrv->tid ) {
		rtems_task_delete(gdrv->tid);
		gdrv->tid = 0;
	}
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
	if ( ! (gdrv->mutex = bsem_create("ipbd", SEM_SMTX)) ) {
		fprintf(stderr, "drvGnrethIpBasic: unable to create driver mutex");
		goto egress;
	}

	/* Create driver task but don't start yet */
	sc = rtems_task_create(
				rtems_build_name('i','p','b','d'),
				20,
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&gdrv->tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to create driver task");
		goto egress;
	}

	/* Initialize low-level driver; it needs a task ID already -- that's
	 * why we had to create the task upfront.
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
								gdrv->tid,
								cleanup_txbuf, gdrv,
								alloc_rxbuf,
								consume_rxbuf, gdrv,
								RX_RING_SIZE, TX_RING_SIZE,
								irq_msk)) ) {
			/* SUCCESSFUL EXIT (unit found and initialized) */
			fprintf(stderr,"drvGnrethIpBasic: using device instance %u\n",unit);
			gdrv->unit = unit - 1;

			task_init( gdrv->tid );

			return gdrv;
		}
	}
	/* if we get here the requested unit was already busy */

	sc = RTEMS_TOO_MANY;
	rtems_error(sc, "drvGnrethIpBasic: no free low-level driver slot found");
	/* fall through */

egress:

	gdrv_cleanup(gdrv);
	free(gdrv);
	return 0;
}

#define KILL_EVENT RTEMS_EVENT_7

#ifndef SIOCGIFMEDIA
/* media_ioctl() aliases '0' to SIOCGIFMEDIA :-) */
#define SIOCGIFMEDIA 0
#endif

LLDev
drvGnrethLLDev(gnreth_drv gdrv)
{
	return gdrv->lldrv.dev;
}

void
drvGnrethIpBasicTask(rtems_task_argument arg)
{
IpBscIf               ipbif_p = (IpBscIf)arg;
gnreth_drv            gdrv    = lanIpBscIfGetDrv(ipbif_p);
LLDev                 lldev   = gdrv->lldrv.dev;
LLDrv                 lldrv   = &gdrv->lldrv;
rtems_event_set       ev_mask = (1<<gdrv->unit) | KILL_EVENT;
rtems_event_set       evs;
uint32_t              irqs;
int                   media;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task starting up\n");
	}
#endif

	lldrv->init_hw( lldev, 0, gdrv->hasenaddr ? gdrv->enaddr : 0 );

	if ( 0 == lldrv->med_ioctl( lldev, SIOCGIFMEDIA, &media ) ) {
		if ( (IFM_LINK_OK & media) ) {
			gdrv->flags &= ~IF_FLG_STOPPED;
		}
	} else {
		fprintf(stderr,"WARNING: unable to determine link state; may be unable to send\n");
	}

	do {
		rtems_event_receive( ev_mask, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs);

		irqs = lldrv->ack_irqs(lldev);

		if ( (irqs & lldrv->tx_irq_msk) ) {
		DRVLOCK(gdrv);
			/* cleanup_txbuf */
			lldrv->swipe_tx(lldev);
		DRVUNLOCK(gdrv);
		}
		if ( (irqs & lldrv->rx_irq_msk) ) {
			/* alloc_rxbuf, consume_rxbuf */
			lldrv->swipe_rx(lldev);
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
		lldrv->enb_irqs(lldev);
	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task received shutting down\n");
	}
#endif

	/* prevent this task from being deleted in cleanup routine */
	gdrv->tid = 0;

	gdrv_cleanup(gdrv);

	free(gdrv);

	task_leave();
}

int
lanIpBscDrvStart(IpBscIf ipbif_p, int pri)
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
		sc = rtems_task_set_priority(gdrv->tid, pri, &op);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc,"drvGnrethIpBasic: unable to change driver task priority");
			return sc;
		}
	}

	/* Link to IF; non-NULL -ness of the IF pointer in the gnreth_drv struct
	 * also server as a flag that the task has been started.
	 */
	gdrv->ipbif_p = ipbif_p;

	sc = rtems_task_start( gdrv->tid, drvGnrethIpBasicTask, (rtems_task_argument)ipbif_p);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "drvGnrethIpBasic: unable to start driver task");
		gdrv->ipbif_p = 0; /* mark as not started */
	}

	/* don't clean the driver struct; leave everything as it was on entry */
	return (int)sc;
}

int
lanIpBscDrvShutdown(LanIpBscDrv drv_p)
{
gnreth_drv gdrv = drv_p;

	if ( !gdrv )
		return 0;

	if ( gdrv->ipbif_p ) {
		task_killer killer;
		killer.kill_event = KILL_EVENT;

		/* Has already been started; driver task must cleanup */
		task_pseudojoin( gdrv->tid, KILL_BY_EVENT, killer );

	} else {
		/* Not started yet */
		gdrv_cleanup(gdrv);
		free(gdrv);
	}
	return 0;
}
