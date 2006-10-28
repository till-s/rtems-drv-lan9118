
/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */

#define ETHERPADSZ sizeof(((EtherHeaderRec*)0)->pad)
#define ETHERHDRSZ (sizeof(EtherHeaderRec) - ETHERPADSZ) 

#define NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr)								\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			int l_ = sizeof((pd)->arpreq) - ETHERPADSZ;						\
			memcpy(b, &(pd)->arpreq.ll.dst, l_);							\
			*(uint32_t*)((IpArpRec*)(b + ETHERHDRSZ))->tpa = ipaddr;		\
			if ( BSP_mve_send_buf((struct mveth_private *)pd->drv_p, b, b, l_) <= 0 )	\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)							\
	do {} while (0)

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz);


#define NETDRV_ENQ_BUFFER(pd, pbuf, nbytes)									\
	do {																	\
		char *b_ = (char*)pbuf + ETHERPADSZ;								\
		int   l_ = (nbytes) - ETHERPADSZ;									\
																			\
		if ( BSP_mve_send_buf((struct mveth_private *)pd->drv_p, pbuf, b_, l_) <= 0 )	\
			relrbuf((rbuf_t*)pbuf);											\
	} while (0)

#define NETDRV_READ_ENADDR(drvhdl, buf)										\
	BSP_mve_read_eaddr((struct mveth_private*)(drvhdl),(buf))

#define NETDRV_INCLUDE	<bsp/if_mve_pub.h>

/* RX buffers must be 64-bit aligned (8 byte)
 * However: SW cache flushing probably needs 32-bytes
 */
#define RBUF_ALIGNMENT	32

#include "lanIpBasic.c"

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz)
{
struct mveth_private *mp = pdrv;
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
			if ( BSP_mve_send_buf(mp, b_, p, l_) <= 0 ) {
				relrbuf((rbuf_t*)b_);
				return -ENOSPC;
			}
			return hdrsz + dtasz;
		}
		return -ENOMEM;
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
IpCbData pd = closure;

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
#define ALL_EVENTS (RTEMS_EVENT_0 | RTEMS_EVENT_1 | KILL_EVENT)
#define RX_RING_SIZE	20
#define TX_RING_SIZE	4

void
drvMveIpBasicTask(rtems_task_argument arg)
{
IpCbData				cbd = (IpCbData)arg;
struct mveth_private	*mp = lanIpCbDataGetDrv(cbd);
rtems_event_set			evs;
uint32_t				irqs;

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task starting up\n");
	}
#endif

	BSP_mve_init_hw( mp, 0, 0 );

	do {
		rtems_event_receive( ALL_EVENTS, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &evs);
		irqs = BSP_mve_ack_irqs(mp);
		if ( irqs & BSP_MVE_IRQ_TX ) {
			BSP_mve_swipe_tx(mp); /* cleanup_txbuf */
		}
		if ( irqs & BSP_MVE_IRQ_RX ) {
			BSP_mve_swipe_rx(mp); /* alloc_rxbuf, consume_rxbuf */
		}
		BSP_mve_enable_irqs(mp);
	} while ( ! (evs & KILL_EVENT) );

#ifdef DEBUG
	if ( lanIpDebug & DEBUG_TASK ) {
		printf("IP task received shutting down\n");
	}
#endif
	BSP_mve_detach( mp );

	rtems_task_delete(RTEMS_SELF);
}

rtems_id
drvMveIpBasicSetup(IpCbData cbd)
{
int                   unit;
struct mveth_private *mp;
rtems_id              tid;

	if ( RTEMS_SUCCESSFUL != rtems_task_create(
								rtems_build_name('i','p','b','d'),
								20,	/* can be changed later */
								10000,
								RTEMS_DEFAULT_MODES,
								RTEMS_FLOATING_POINT | RTEMS_LOCAL,
								&tid) )
			return 0;

	for ( unit=1; unit < 3; unit++ ) {
		if ( (mp = BSP_mve_setup( unit,
								tid,
								cleanup_txbuf, cbd,
								alloc_rxbuf,
								consume_rxbuf, cbd,
								RX_RING_SIZE, TX_RING_SIZE,
								BSP_MVE_IRQ_TX | BSP_MVE_IRQ_RX /* | BSP_MVE_IRQ_LINK */ )) ) {
			rtems_task_set_note( tid, RTEMS_NOTEPAD_0, (uint32_t)mp );
			return tid;
		}
	}
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
