
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
			if ( BSP_mve_send_buf((struct mveth_private *)pd->drv_p, b, l_) <= 0 )	\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)							\
	do {} while (0)

#define NETDRV_ENQ_PACKET(pd, pbuf, nbytes)									\
	do {																	\
		char *b = (char*)getrbuf();											\
		if ( b ) {															\
			int l_ = (nbytes) - ETHERPADSZ;									\
			memcpy(b, ((char*)(pbuf)) + ETHERPADSZ, l_);					\
			if ( BSP_mve_send_buf((struct mveth_private *)pd->drv_p, b, l_) <= 0 )	\
				relrbuf((rbuf_t*)b);										\
		}																	\
	} while (0)

#define NETDRV_READ_ENADDR(drvhdl, buf)										\
	BSP_mve_read_eaddr((struct mveth_private*)(drvhdl),(buf))

#define NETDRV_INCLUDE	<bsp/if_mve_pub.h>

#include "lanIpBasic.c"

static void
cleanup_txbuf(void *buf, void *closure, int error_on_tx_occurred)
{
	relrbuf(buf);
}

static void *
alloc_rxbuf(int *p_size, unsigned long *p_data_addr)
{
	*p_size = (*p_data_addr = (unsigned long)getrbuf()) ? RBUFSZ : 0;
	return (void*) *p_data_addr;
}

static void
consume_rxbuf(void *buf, void *closure, int len)
{
rbuf_t   *b = buf;
IpCbData pd = closure;

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
	} while ( ! (evs & 8) );

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
