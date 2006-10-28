
/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 */
#define NETDRV_ATOMIC_SEND_ARPREQ(pd, ipaddr)								\
	do {																	\
		DrvLan9118_tps	plan = (pd)->drv_p;									\
																			\
		if ( 0 == drvLan9118TxPacket(plan, 0, sizeof((pd)->arpreq), 0) ) {	\
																			\
			/* arpreq is locked and may be modified */						\
			 *(uint32_t*)(pd)->arpreq.arp.tpa = ipaddr;						\
																			\
			/* send request */												\
			drvLan9118FifoWr(plan, &(pd)->arpreq, sizeof((pd)->arpreq));	\
																			\
			drvLan9118TxUnlock(plan);										\
		}																	\
	} while (0)

#define NETDRV_READ_INCREMENTAL(pd, ptr, nbytes)						\
	drvLan9118FifoRd((DrvLan9118_tps)(pd)->drv_p, (ptr), (nbytes))

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz);

#define NETDRV_ENQ_BUFFER(pd, pbuf, nbytes)								\
	do {																\
		NETDRV_SND_PACKET(pd, 0, 0, pbuf, nbytes);						\
		relrbuf(pbuf);													\
	} while (0)

#define NETDRV_READ_ENADDR(drvhdl, buf)									\
	drvLan9118ReadEnaddr((DrvLan9118_tps)(drvhdl), (buf))

#define NETDRV_INCLUDE	<drvLan9118.h>

#include "lanIpBasic.c"

static inline int 
NETDRV_SND_PACKET(void *pdrv, void *phdr, int hdrsz, void *data, int dtasz)
{
DrvLan9118_tps plan = (DrvLan9118_tps)pdrv;

	assert( 0 == (hdrsz  & 3) && 0 == ((uint32_t)phdr & 3) && 0 == ((uint32_t)data & 3) );

	if ( phdr ) {
		if ( drvLan9118TxPacket(plan, 0, hdrsz+dtasz, 0) ) {
			return -ENOSPC;
		}
		drvLan9118FifoWr(plan, phdr, hdrsz);
		drvLan9118FifoWr(plan, data, dtasz);
		drvLan9118TxUnlock(plan);
		return hdrsz + dtasz;
	}
	return drvLan9118TxPacket(plan, data, dtasz, 0);
}

int
drvLan9118IpRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *arg)
{
IpCbData        pd = arg;
rbuf_t			*prb;

	if ( ! (prb = getrbuf()) ) {
		return len;
	}

	len = lanIpProcessBuffer(pd, &prb, len);

	relrbuf(prb);

	return (len);
}
