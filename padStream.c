/* $Id$ */

#include <udpComm.h>
#include <padProto.h>
#include <lanIpBasic.h>
#include <drvLan9118.h>
#include <rtems.h>
#include <errno.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#define NCHNS	4	/* channels on a PAD */

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

static LanIpPacketRec replyPacket = {{{0}}};
static int            nsamples; /* keep value around (lazyness) */

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
padStreamStart(PadRequest req, PadStrmCommand scmd, int me, uint32_t hostip)
{
PadReply        rply = (PadReply)replyPacket.p_u.udp_s.pld;
int             len;

	
	if ( (nsamples = ntohl(scmd->nsamples)) > 720/4  ) {
		/* doesn't fit in one packet */
		return -EINVAL;
	}

	LOCK();

		/* Avoid ARP lookup, don't provide destination IP yet */
		udpSockHdrsInit(-1, &replyPacket, 0, ntohs(scmd->port), 0); 	

		/* Add missing bits: destination IP , source port */
		replyPacket.ip.dst               = hostip;
		replyPacket.p_u.udp_s.hdr.sport  = scmd->port;

		len = nsamples*sizeof(int16_t)*NCHNS + sizeof(*rply);
		/* Fill in length and IP header checksum etc */
		udpSockHdrsSetlen(&replyPacket, len);
		/* Setup Reply */
		rply->version         = req->version;
		rply->type            = PADCMD_STRM | PADCMD_RPLY;
		rply->chnl            = me;
		rply->nBytes          = htons(len);
		rply->status          = 0;
		rply->strm_cmd_flags = scmd->flags;
		rply->strm_cmd_idx   = 0;
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



int bigEndian()
{
union {
	uint8_t  x[2];
	uint16_t tst;
} endian = { {0xBE, 0} };
	return(endian.tst == 0xBE00);
}

static inline int16_t
swap(int16_t x)
{
	return x<<8 | ((uint16_t)x)>>8;
}

static void *
streamTest(void *packetBuffer,
			int idx,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t *buf = packetBuffer;
int i;
	if ( !little_endian != bigEndian() ) {
		if ( column_major ) {
			for (i=0; i<nsamples; i++) {
				buf[i*4+0] = swap(i+0x00 + (idx<<8));
				buf[i*4+1] = swap(i+0x10 + (idx<<8));
				buf[i*4+2] = swap(i+0x20 + (idx<<8));
				buf[i*4+3] = swap(i+0x30 + (idx<<8));
			}
		} else {
			for (i=0; i<nsamples; i++) {
				buf[i+0*nsamples] = swap(i+0x00 + (idx<<8));
				buf[i+1*nsamples] = swap(i+0x10 + (idx<<8));
				buf[i+2*nsamples] = swap(i+0x20 + (idx<<8));
				buf[i+3*nsamples] = swap(i+0x30 + (idx<<8));
			}
		}
	} else {
		#define swap(x)	(x)
		if ( column_major ) {
			for (i=0; i<nsamples*NCHNS; i+=4) {
				buf[i+0] = swap(i+0x00 + (idx<<8));
				buf[i+1] = swap(i+0x10 + (idx<<8));
				buf[i+2] = swap(i+0x20 + (idx<<8));
				buf[i+3] = swap(i+0x30 + (idx<<8));
			}
		} else {
			for (i=0; i<nsamples; i++) {
				buf[i+0*nsamples] = swap(i+0x00 + (idx<<8));
				buf[i+1*nsamples] = swap(i+0x10 + (idx<<8));
				buf[i+2*nsamples] = swap(i+0x20 + (idx<<8));
				buf[i+3*nsamples] = swap(i+0x30 + (idx<<8));
			}
		}
		#undef swap
	}
	return buf;
}

typedef struct StripSimValRec_ {
	int32_t	a,b,c,d;
} StripSimValRec, *StripSimVal;

extern unsigned
iir2_bpmsim(
	int16_t *pf,
	int nloops,
	int ini,
	unsigned long *pn,
	int swp,
	int stride);

static void *
streamSim(void *packetBuffer,
			int idx,
			int nsamples,
			int little_endian,
			int column_major,
			void *uarg)
{
int16_t		*buf = packetBuffer;
StripSimVal ini  = uarg;
int         swp;
static unsigned long noise = 1;

	swp    = ( bigEndian() != !little_endian );

	if ( column_major ) {
		iir2_bpmsim(buf++, nsamples, ini->a,  &noise, swp, nsamples);
		iir2_bpmsim(buf++, nsamples, ini->b,  &noise, swp, nsamples);
		iir2_bpmsim(buf++, nsamples, ini->c,  &noise, swp, nsamples);
		iir2_bpmsim(buf++, nsamples, ini->d,  &noise, swp, nsamples);
	} else {
		iir2_bpmsim(buf, nsamples, ini->a,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->b,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->c,  &noise, swp, 1);
		buf += nsamples;
		iir2_bpmsim(buf, nsamples, ini->d,  &noise, swp, 1);
	}

	return packetBuffer;
}

int
padStreamSend(void * (*getdata)(void *packBuffer, int idx, int nsamples, int endianLittle, int colMajor, void *uarg), int idx, void *uarg)
{
int            rval = 0;
PadReply       rply = (PadReply)replyPacket.p_u.udp_s.pld;
DrvLan9118_tps plan = lanIpCbDataGetDrv(intrf);
int            len;
void          *data_p;

	if ( idx != 0 )
		return -ENOTSUP;	/* not supported yet */

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

	rply->strm_cmd_idx   = idx;

	len = UDPPKTSZ(ntohs(rply->nBytes));

	if ( drvLan9118TxPacket(plan, 0, len, 0) ) {
		UNLOCK();
		return -ENOSPC;
	}

	drvLan9118FifoWr(plan, &replyPacket, UDPPKTSZ(sizeof(PadReplyRec)));

	if ( (data_p=getdata(
					&rply->data,
					idx,
					nsamples, 
					rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE,
					rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM,
					uarg)) ) {
		drvLan9118FifoWr(plan, data_p, nsamples*NCHNS*sizeof(int16_t));
	}
	/* else ['getdata' returned NULL] the getdata method already
	 * wrote to the TX FIFO
	 */

	drvLan9118TxUnlock(plan);

	UNLOCK();

	return 0;
}

int
padStreamTest()
{
	return padStreamSend(streamTest,0,0);
}

int
padStreamSim(PadSimCommand scmd)
{
StripSimValRec strips = { ntohl(scmd->a), ntohl(scmd->b), ntohl(scmd->c), ntohl(scmd->d) };

	return padStreamSend(streamSim, 0, &strips);
}

int
padStreamStop(void)
{
	LOCK();
	isup = 0;
	UNLOCK();
	return 0;
}
