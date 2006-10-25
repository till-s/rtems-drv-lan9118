
/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */
#define NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr)							\
	do {																\
		DrvLan9118_tps	plan = (pd)->drv_p;								\
		drvLan9118TxPacket(plan, 0, sizeof((pd)->arpreq), 0);			\
																		\
		/* arpreq is locked and may be modified */						\
		 *(uint32_t*)(pd)->arpreq.arp.tpa = ipaddr;						\
																		\
		/* send request */												\
		drvLan9118FifoWr(plan, &(pd)->arpreq, sizeof((pd)->arpreq));	\
																		\
		drvLan9118TxUnlock(plan);										\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)						\
	drvLan9118FifoRd((DrvLan9118_tps)(pd)->drv_p, (ptr), (nbytes))

#define NETDRV_ENQ_PACKET(pd, pbuf, nbytes)								\
	drvLan9118TxPacket((DrvLan9118_tps)(pd)->drv_p, (pbuf), (nbytes), 0)

#define NETDRV_READ_ENADDR(drvhdl, buf)									\
	drvLan9118ReadEnaddr((DrvLan9118_tps)(drvhdl), (buf))

#define NETDRV_INCLUDE	<drvLan9118.h>

#include "lanIpBasic.c"

int
drvLan9118IpRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *arg)
{
IpCbData        pd = arg;
rbuf_t			*prb;

	if ( ! (prb = getrbuf()) ) {
		return len;
	}

	len -= lanIpProcessBuffer(pd, &prb, len);

	relrbuf(prb);

	return (len);
}
