#include <udpComm.h>
#include <padProto.h>
#include <errno.h>

#include <netinet/in.h>

#define NCHNS 4

extern unsigned
iir2_bpmsim(
	int16_t *pf,
	int nloops,
	int ini,
	unsigned long *pn,
	int swp,
	int stride);


static int bigEndian()
{
union {
	uint8_t  x[2];
	uint16_t tst;
} endian = { x : {0xBE, 0} };
	return(endian.tst == 0xBE00);
}

typedef struct StripSimValRec_ {
	int32_t	a,b,c,d;
} StripSimValRec, *StripSimVal;

static void *
streamSim(void *packetBuffer,
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
		iir2_bpmsim(buf++, nsamples, ini->a,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->b,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->c,  &noise, swp, NCHNS);
		iir2_bpmsim(buf++, nsamples, ini->d,  &noise, swp, NCHNS);
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

static int
strmReplySetup(PadReply rply, uint32_t xid, int le, int col_maj, int nsamples, int chnl)
{
int len = nsamples*sizeof(int16_t)*NCHNS + sizeof(*rply);

	/* Setup Reply */
	rply->version         = PADPROTO_VERSION1;
	rply->type            = PADCMD_STRM | PADCMD_RPLY;
	rply->chnl            = chnl;
	rply->nBytes          = htons(len);
	rply->timestampHi     = htons(0);
	rply->timestampLo     = htonl(0);
	rply->xid             = xid;
	rply->status          = 0;
	rply->strm_cmd_flags  = PADRPLY_STRM_FLAG_TYPE_SET(0);
	if ( le )
		rply->strm_cmd_flags |= PADCMD_STRM_FLAG_LE;
	if ( col_maj )
		rply->strm_cmd_flags |= PADCMD_STRM_FLAG_CM;
	rply->strm_cmd_idx    = 0;

	return len;
}

static int sd   = -1;
PadReply   rply =  0 ;

int
padStreamStart(PadRequest req, PadStrmCommand cmd, int me, uint32_t hostip)
{
uint32_t nsamples = ntohl(cmd->nsamples);
int      le       = cmd->flags & PADCMD_STRM_FLAG_LE;
int      col_maj  = cmd->flags & PADCMD_STRM_FLAG_CM;

	if ( sd >= 0 )
		udpCommClose(sd);

	if ( (sd = udpCommSocket(6668)) < 0 )
		goto bail;

	if ( udpCommConnect(sd, hostip, ntohs(cmd->port)) ) {
		udpCommClose(sd);
		sd = -1;
		goto bail;
	}

	if ( !rply )
		rply = malloc(2048);

	strmReplySetup(rply, req ? req->xid : 0, le, col_maj, nsamples, me);

	return 0;

bail:
	perror("padStreamStart failed");
	return errno ? -errno : 0;
}


int
padStream(int32_t a, int32_t b, int32_t c, int32_t d)
{
StripSimValRec      v = {a,b,c,d};
int len, nsamples;

	if ( sd < 0 )
		return -ENODEV;

	len      = ntohs(rply->nBytes);
	nsamples = (len - sizeof(*rply))/NCHNS/sizeof(int16_t);

	streamSim(
		rply->data,
		nsamples, 
		rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE,
		rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM,
		&v);
	udpCommSend(sd, rply, len);

	return 0;
}

int
padStreamStop()
{
	if ( sd < 0 ) 
		return -ENODEV;
	udpCommClose(sd);
	sd = -1;
	return 0;
}


#ifdef MAIN
int main()
{
PadStrmCommandRec cmd;
int               chan = 0;

	cmd.type     = PADCMD_STRM;
	cmd.flags    = PADCMD_STRM_FLAG_LE | PADCMD_STRM_FLAG_CM; 
	cmd.nsamples = htonl(128);
	cmd.port     = htons(PADPROTO_STRM_PORT);

	padStreamStart(0, &cmd, chan, inet_addr("127.0.0.1"));

	padStream(1000,2000,3000,4000);
}
#endif
