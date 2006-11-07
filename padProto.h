/* $Id$ */

#ifndef PADPROTO_DEF_H
#define PADPROTO_DEF_H

/* Communication Protocol with PAD over dedicated network */

#include <stdint.h>
#include <udpComm.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PADCMD_RPLY  ((int8_t)(1<<7)) 	/* reply ORs request with msb      */
#define PADCMD_QUIET ((int8_t)(1<<6))	/* requestor doesn't want a reply  */

#define	PADCMD_NOP  ((int8_t) 0)
#define	PADCMD_ECHO ((int8_t) 1)
#define	PADCMD_STRM ((int8_t) 2)
#define	PADCMD_STOP ((int8_t) 3)
#define PADCMD_KILL ((int8_t)15)

typedef struct PadCommandRec_ {
	int8_t		type;			/* PADCMD_XX                           */
	int8_t		sdata[3];		/* commands with small data needs      */
	uint32_t	ldata[];		/* word sized commands                 */
} PadCommandRec, *PadCommand;

#define PADCMD_START_FLAG_LE	1	/* They want little-endian data    */
#define PADCMD_START_FLAG_CM	2	/* They want column-major  data    */

typedef struct PadStartCommandRec_ {
	int8_t		type;			/* PADCMD_XX                           */
	uint8_t		flags;
	uint16_t	port;			/* port where to send data             */
	uint32_t	nsamples;
} PadStartCommandRec, *PadStartCommand;

/* Reply is sent to the alternate port specified in the request        */
typedef struct PadStartCommandReplyRec_ {
	int8_t		type;
	uint8_t		flags;			/* confirm requested options           */
	uint16_t	error;			/* 0 on success an ERRNO on error      */
} PadStartCommandReplyRec, *PadStartCommandReply;


#define PADPROTO_VERSION1		0x31	/* some magic number           */

#define PADREQ_BCST	(-128) /* Address all channels with a single command */

/* Note: cmdSize must be a multiple of 4 (dword-aligned)               */
typedef struct PadRequestRec_ {
	uint8_t		version;		/* Protocol version                    */
	int8_t		nCmds;			/* number of channels,-128: broadcast  */
	int8_t		cmdSize;		/* size of command structures          */
	int8_t		r1;				/* reserved                            */
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;            /* transaction 'id'                    */
	uint32_t	r2;				/* align to 16-bytes (incl. IP hdrs)   */
	uint8_t		data[];
} PadRequestRec, *PadRequest;
/* appended here are |nCmds| PadCommandRecs */

typedef struct PadReplyRec_ {
	uint8_t		version;		/* Protocol version                    */
	int8_t		type;			/* reply to command of 'type'          */
	int8_t		nBytes;			/* size of reply; command dependent    */
	int8_t		chnl;			/* channel sending the reply           */
	uint32_t	timestampHi;
	uint32_t	timestampLo;
	uint32_t	xid;            /* transaction 'id'                    */
	uint32_t	r2;				/* align to 16-bytes (incl. IP hdrs)   */
	uint8_t		data[];
} PadReplyRec, *PadReply;

/* Handle Protocol Request
 *    'req_p': The request. This may be modified by this routine to form a reply.
 *       'me': Channel number to look for ('our' channel/slot # in the packet).
 * 'killed_p': Pointer to a variable that is set if a 'KILL' command was received.
 *
 * RETURNS: < 0 on error (-errno), 0 or 1 on success. If 1 is returned then the
 *          request was transformed into a reply and should be returned to the requestor.
 */
int
padProtoHandler(PadRequest req_p, int me, int *killed_p);

/* Handle padProto requests on 'port' for channel/slot 'chnl' until an error
 * occurs or a KILL command is received.
 *
 * RETURNS: 0 on success -errno on error
 */
int
padUdpHandler(int port, int chnl);

/* Send a request to 'connected' peer
 *        'sd': socket descriptor
 *      'chnl': target channel/slot (may be broadcast)
 *      'type': command
 *   'cmdData': parameter to command
 * 'wantReply': if non-NULL then q reply from the peer is requested and
 *              returned in *wantReply.
 *
 * RETURNS: 0 on success, -errno on error.
 *
 * NOTES: This routine does not provide the full functionality
 *        the protocol allows. It is not possible to send different
 *        commands to different channels with this routine.
 */
int
padRequest(int sd, int chnl, int type, void *cmdData, UdpCommPkt *wantReply);

#ifdef __cplusplus
}
#endif

#endif
