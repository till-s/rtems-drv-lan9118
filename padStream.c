/* $Id$ */

#include <udpComm.h>
#include <padProto.h>
#include <lanIpBasic.h>
#include <drvLan9118.h>
#include <rtems.h>
#include <errno.h>
#include <assert.h>

/* Data stream implementation. This could all be done over the
 * udpSock abstraction but less efficiently since we would have
 * to copy the PAD fifo to memory first instead of copying the
 * PAD fifo to the lan9118 chip directly...
 */


/* The padProtoHandler and the thread sending the data
 * both access the reply data structure;
 */
static rtems_id mutex = 0;

#define LOCK() \
	assert( RTEMS_SUCCESSFUL == rtems_semaphore_obtain(mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT) )

#define UNLOCK() \
	assert( RTEMS_SUCCESSFUL == rtems_semaphore_release(mutex) )

static LanIpPacketRec replyPacket;

static IpCbData intrf;

static volatile int isup = 0;

int
padStreamInitialize(IpCbData if_p)
{
rtems_status_code sc;
	
	sc = rtems_semaphore_create(
			rtems_build_name('p','a','d','S'),
			1,
            RTEMS_SIMPLE_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&mutex);

	if ( RTEMS_SUCCESSFUL != sc ) {
		mutex = 0;
		return sc;
	}
	intrf = if_p;
	return 0;
}

int
padStreamCleanup()
{
	rtems_semaphore_delete(mutex);
	mutex = 0;
	intrf = 0;
	return 0;
}

/* refresh timestamp and transaction id */
static void
dopet(PadRequest req, PadReply rply)
{
	rply->timestampHi = req->timestampHi;
	rply->timestampLo = req->timestampLo;
	rply->xid         = req->xid;
}

int
padStreamStart(PadRequest req, PadStartCommand scmd, int me, uint32_t hostip)
{
PadReply        rply = (PadReply)replyPacket.p_u.udp_s.pld;
int             nsamples;

	
	if ( (nsamples = ntohl(scmd->nsamples)) > 720 ) {
		/* doesn't fit in one packet */
		return -EINVAL;
	}

	LOCK();

		/* Avoid ARP lookup, don't provide destination IP yet */
		udpSockHdrsInit(-1, &replyPacket, 0, ntohs(scmd->port), 0); 	

		/* Add missing bits: destination IP , source port */
		replyPacket.ip.dst               = hostip;
		replyPacket.p_u.udp_s.hdr.sport  = scmd->port;

		/* Fill in length and IP header checksum etc */
		udpSockHdrsSetlen(&replyPacket, nsamples*sizeof(int16_t) + sizeof(*rply));

		/* Setup Reply */
		rply->version         = req->version;
		rply->type            = PADCMD_STRM | PADCMD_RPLY;
		rply->chnl            = me;
		rply->nBytes          = htons(nsamples*2 + sizeof(*rply));
		rply->status          = 0;
		rply->start_cmd_flags = scmd->flags;
		rply->start_cmd_idx   = 0;
		dopet(req, rply);
		
		isup                  = 1;
	UNLOCK();

	return 0;
}

int
padStreamPet(PadRequest req)
{
PadReply  rply = (PadReply)replyPacket.p_u.udp_s.pld;
	LOCK();
		dopet(req, rply);
	UNLOCK();
	return 0;
}

int
padStreamSend()
{
int            rval = 0;
PadReply       rply = (PadReply)replyPacket.p_u.udp_s.pld;
DrvLan9118_tps plan = lanIpCbDataGetDrv(intrf);
int            len;

	LOCK();

	if ( !isup ) {
		UNLOCK();
		return -EAGAIN;
	}
	/* just look in the cache - we rely on the RX daemon refreshing it */
	if ( (rval = arpLookup(intrf, replyPacket.ip.dst, replyPacket.ll.dst, 1)) ) {
		UNLOCK();
		return rval;
	}
	len = UDPPKTSZ(ntohs(rply->nBytes));
	if ( drvLan9118TxPacket(plan, 0, len, 0) ) {
		UNLOCK();
		return -ENOSPC;
	}
	drvLan9118FifoWr(plan, &replyPacket, UDPPKTSZ(sizeof(PadReplyRec)));

	/* TODO Submit payload */

	drvLan9118TxUnlock(plan);

	UNLOCK();

	return 0;
}

int
padStreamStop(void)
{
	LOCK();
	isup = 0;
	UNLOCK();
	return 0;
}
