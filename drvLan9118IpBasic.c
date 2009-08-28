
/* fwd decl of interface struct */
struct IpBscIfRec_;

/* MACROS USED BY lanIpBasic.c;
 * These macros must be implemented by the driver so that the
 * stack can read/send data.
 */

/* How to send the ARP request structure:
 * Either lock, fillin-IP addr, send, unlock
 * or copy, fillin send copy...
 * The (ipbif_p)->arpreq struct is only modified by this macro so
 * any lock used here is adequate.
 */
#define NETDRV_ATOMIC_SEND_ARPREQ(ipbif_p, ipaddr)							\
	do {																	\
		DrvLan9118_tps	drv_p = (ipbif_p)->drv_p;							\
		size_t             sz = sizeof((ipbif_p)->arpreq);					\
																			\
		if ( 0 == drvLan9118TxPacket(drv_p, 0, sz, 0) ) {					\
																			\
			/* arpreq is locked and may be modified */						\
			set_tpa(&(ipbif_p)->arpreq.arp, ipaddr);						\
																			\
			/* send request - size of arpreq is multiple of words */		\
			drvLan9118FifoWr(drv_p, &(ipbif_p)->arpreq, sz);				\
																			\
			drvLan9118TxUnlock(drv_p);										\
		}																	\
	} while (0)

/* Read 'nbytes' from device into buffer at 'ptr'.
 * This macro helps supporting 'FIFO'-type devices with a need
 * for PIO.
 * More powerful controllers which DMA packet data into memory
 * don't need this if the buffer handed over to lanIpProcessBuffer()
 * already contains packet data.
 */
#define NETDRV_READ_INCREMENTAL(ipbif_p, ptr, nbytes)						\
	drvLan9118FifoRd((DrvLan9118_tps)(ipbif_p)->drv_p, (ptr), (nbytes))

/* Send a packet header (size 'hdrsz') and payload data (size 'dtasz').
 * The header and payload areas are not necessarily contiguous.
 * 'phdr' may be NULL if 'data' already contains the header.
 * There is nothing magic about the header -- this is just 'poor-man's gathering'
 * i.e., support for sending data from two non-contiguous areas.
 */
static inline int 
snd_packet_locked(struct IpBscIfRec_ *ipbif_p, void *phdr, int hdrsz, void *data, int dtasz);

#define NETDRV_SND_PACKET snd_packet_locked
/* Enqueue and send a buffer; in contrast to NETDRV_SND_PACKET() which
 * sends data out of arbitrary memory (and therefore always involves some
 * kind of copying) the buffer handed to NETDRV_ENQ_BUFFER() is a buffer
 * managed by lanIpBasic (via getrbuf()/relrbuf()).
 * The driver for a DMA-capable controller may simply pass the device
 * the buffer pointer and the descriptor-sweeper may then release the
 * buffer (relrbuf()) after transmission.
 * The driver for a DMA-less controller may just call NETDRV_SND_PACKET()
 * (copy+send buffer) and release the buffer.
 */
#define NETDRV_ENQ_BUFFER(ipbif_p, pbuf, nbytes)						\
	do {																\
		NETDRV_SND_PACKET((ipbif_p), 0, 0, pbuf, nbytes);				\
		relrbuf(pbuf);													\
	} while (0)

/* Read MAC address from device/driver into a buffer */
#define NETDRV_READ_ENADDR(ipbif_p, buf)								\
	drvLan9118ReadEnaddr((DrvLan9118_tps)(ipbif_p->drv_p), (buf))

#define NETDRV_MC_FILTER_ADD(ipbif_p, macaddr)							\
	drvLan9118McFilterAdd((DrvLan9118_tps)((ipbif_p)->drv_p), (macaddr))

#define NETDRV_MC_FILTER_DEL(ipbif_p, macaddr)							\
	drvLan9118McFilterDel((DrvLan9118_tps)((ipbif_p)->drv_p), (macaddr))

/* Driver header name with angle brackets e.g., <drvXXX.h> */
#define NETDRV_INCLUDE	<drvLan9118.h>

/* Include main C-file here */
#include "lanIpBasic.c"

/* Implement fwd-declared bits */
static inline int 
snd_packet_locked(IpBscIf ipbif_p, void *phdr, int hdrsz, void *data, int dtasz)
{
DrvLan9118_tps drv_p = (DrvLan9118_tps)ipbif_p->drv_p;
int ltot;

	/* header must be word-aligned; payload may be padded;
	 * (implicit TXCMD_A_END_ALIGN_4)
	 */
	ltot = (dtasz+3) & ~3;

	assert( 0 == (hdrsz  & 3) && 0 == ((uint32_t)phdr & 3) && 0 == ((uint32_t)data & 3) );

	if ( phdr ) {
		if ( drvLan9118TxPacket(drv_p, 0, hdrsz+dtasz, 0) ) {
			return -ENOSPC;
		}
		drvLan9118FifoWr(drv_p, phdr, hdrsz);
		drvLan9118FifoWr(drv_p, data, ltot );
		drvLan9118TxUnlock(drv_p);
		return dtasz;
	}
	return drvLan9118TxPacket(drv_p, data, dtasz, 0) ? -ENOSPC : dtasz;
}

/* Implement high-level routines (how to create, start, shutdown driver);
 *
 * lanIpBscDrvCreate(): allocates a driver slot and do first initialization.
 *
 * lanIpBscDrvStart() : create and start driver task(s) and other resources.
 *                      
 *                      Transmission: largely handled by SND_PACKET and 
 *                      ENQ_BUFFER macros above. If the driver and device
 *                      support DMA then the descriptor ring scavenger
 *                      must release sent buffers (relrbuf()).
 *
 *                      Reception:
 *                        DMA-capable / descriptor ring type device:
 *                           preallocate buffers (getrbuf()) and fill
 *                           RX ring. As packets are received driver
 *                           task calls lanIpProcessBuffer() and
 *                           subsequently either calls relrbuf() or
 *                           leaves the buffer in the ring. In the
 *                           former case, the RX-buffer-allocator
 *                           must obtain new buffers calling getrbuf().
 *
 *                        FIFO-type device:
 *                           RX task calls 
 *                              getrbuf() // allocate buffer
 *                              fill_buffer_from_device()
 *                              lanIpProcessBuffer() // munch
 *                              relrbuf() // release buffer
 *                           Note that the 'fill_buffer_from_device()'
 *                           step can be omitted if the READ_INCREMENTAL
 *                           macro is implemented. lanIpProcessBuffer will
 *                           then pull data into the buffer on the fly
 *                           (and only as much as needed).
 *
 * lanIpBscDrvShutdown(): shutdown driver task and release all
 *                        resources owned by the driver.
 *                        Routine must be able to clean up a partially
 *                        initialized driver (e.g., created but not
 *                        started). In particular, it must handle
 *                        the case of a NULL driver handle (doing
 *                        nothing).
 */

int
drvLan9118IpRxCb(DrvLan9118_tps drv_p, uint32_t len, void *arg)
{
IpBscIf        ipbif_p = arg;
rbuf_t			*prb;

	if ( ! (prb = getrbuf()) ) {
		return len;
	}

	len = lanIpProcessBuffer(ipbif_p, &prb, len);

	relrbuf(prb);

	return (len);
}

LanIpBscDrv
lanIpBscDrvCreate(int instance, uint8_t *enaddr_p)
{
	return drvLan9118Setup(enaddr_p,0);
}

int
lanIpBscDrvStart(IpBscIf ipbif_p, int pri)
{
DrvLan9118_tps drv_p = lanIpBscIfGetDrv(ipbif_p);

  if ( !drv_p ) {
	rtems_error(RTEMS_NOT_DEFINED,"drvLan9118IpBasic: driver not attached to interface yet?");  		
	return RTEMS_NOT_DEFINED;
  }
  return drvLan9118Start(drv_p, pri, 0,
                drvLan9118IpRxCb, ipbif_p,
                0, 0,
                0, 0,
                0, 0);
}

int
lanIpBscDrvShutdown(LanIpBscDrv drv_p)
{
	if ( drv_p )
        drvLan9118Shutdown((DrvLan9118_tps)drv_p);
	return 0;
}
