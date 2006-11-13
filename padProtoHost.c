/* $Id$ */


/* Wrapper program to send padProto requests */

#ifndef BSDSOCKET
#define  BSDSOCKET
#endif

#include <padProto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int verbose  = 0;

void
pdump(UdpCommPkt p)
{
int i,sz;
	if (p) {
		printf("Packet Dump:");
		sz = ntohs(((PadReply)p)->nBytes);
		for (i=0; i<sz; i++) {
			if ( i%16 == 0 )
				printf("\n");
			printf("%02X ",((uint8_t*)p)[i]);
		}
		printf("\n");
	}
}

void
dumpReply(PadReply p)
{
int16_t tmp;
	printf("Got Reply (proto version 0x%02x):\n", p->version);
	printf("     Type: 0x%02x\n",           (uint8_t)p->type);
	printf("  Channel: %i\n",               p->chnl);
	tmp = ntohs(p->nBytes);
	printf("   nBytes: %i [payload %i]\n",  tmp, tmp-sizeof(*p));
	printf("      XID: 0x%08x\n",           ntohl(p->xid)); 
	tmp = ntohs(p->status);
	printf("   status: %i (%s)\n",          tmp, strerror(-tmp));
	printf("     spec: 0x%02x, 0x%02x\n",   p->spec[0], p->spec[1]);
}

static int
isbe()
{
union {
	uint8_t	 xx[2];
	uint16_t tst;
} endian = { {0xbe, 00} };
	return (0xbe00 == endian.tst);
}

void
usage(char *nm)
{
	fprintf(stderr,"Usage: %s [-hv] [-l port] [-n nsamples] ip:port <msg_type_int>\n",nm);
}

static void
client(int sd, char *ip, int port, PadStrmCommand scmd)
{
int               err;
UdpCommPkt        p = 0;

	if ( (err = udpCommConnect(sd, inet_addr(ip), port)) ) {
		fprintf(stderr,"udpCommConnect: %s",strerror(-err));
		exit(1);
	}


	if ( (err=padRequest(sd, 3, scmd->type, scmd, &p, 100)) ) {
		fprintf(stderr,">>> Sending request failed: %s <<<\n",strerror(-err));
	}

	if ( p )
		dumpReply((PadReply)p);

	udpCommFreePacket(p);
}

static void
streamdump(int sd)
{
UdpCommPkt        p;
PadReply          rply;
int16_t           err;
int               i;
int               sz;
int16_t           *buf;
	while ( (p = udpCommRecv(sd, 1000000000)) ) {
		rply = (PadReply)p;
		if ( verbose ) 
			dumpReply(rply);
		if ( rply->type != (PADCMD_STRM | PADCMD_RPLY) ) {
			fprintf(stderr,"Listener: received invalid reply 0x%02x\n", (uint8_t)rply->type);
			break;
		}
		if ( (err=ntohs(rply->status)) ) {
			fprintf(stderr,"Listener: received bad reply (%i -> %s)\n", err, strerror(-err));
			break;
		}
		/* Check matching endianness */
		if ( isbe() != !(rply->strm_cmd_flags & PADCMD_STRM_FLAG_LE) ) {
			fprintf(stderr,"Listener: endianness mismatch\n");
			break;
		}

		sz = ntohs(rply->nBytes) - sizeof(*rply);

		sz/=sizeof(int16_t)*4; /* four channels */

		/* Dump packet */
		buf = (int16_t*)rply->data;
		/* always write out in fortran (scilab) format with
		 * the samples going down the columns
		 */
		if ( rply->strm_cmd_flags & PADCMD_STRM_FLAG_CM ) {
			for (i=0; i<sz*4; i+=4) {
				printf("%5i %5i %5i %5i\n",
					buf[i+0], buf[i+1], buf[i+2], buf[i+3]);
			}
		} else {
			for (i=0; i<sz; i++) {
				printf("%5i %5i %5i %5i\n",
					buf[i+0*sz], buf[i+1*sz], buf[i+2*sz], buf[i+3*sz]);
			}
		}
		printf("\n\n");
		fflush(stdout);
		udpCommFreePacket(p);
	}

	if ( p ) {
		dumpReply((PadReply)p);
		udpCommFreePacket(p);
	}
}

int
main(int argc, char **argv)
{
int               sd;
int               type;
int               ch;
PadStrmCommandRec scmd;
int               port      = 0;
int               listener  = 0;
char               *col     = 0;
int               nsamples  = 8;
int               badEndian = 0;
int               colMajor  = 0;

	while ( (ch = getopt(argc, argv, "vcehl:n:")) > 0 ) {
		switch (ch) {
			default:
				fprintf(stderr,"Unknown option '%c'\n",ch);
				usage(argv[0]);
				exit(1);

			case 'h':
				usage(argv[0]);
				exit(0);

			case 'l':
				if ( 1!= sscanf(optarg,"%i",&port) || port <= 0 || port > 65535 ) {
					fprintf(stderr,"invalid port number: '%s'\n", optarg);
					usage(argv[0]);
					exit(1);
				}
				listener = 1;	
				break;

			case 'v': verbose   = 1; break;
			case 'c': colMajor  = 1; break;
			case 'e': badEndian = 1; break;

			case 'n':
				if ( 1 != sscanf(optarg,"%i",&nsamples) ) {
					fprintf(stderr,"invalid number of sampes: '%s'\n",optarg);
					usage(argv[0]);
					exit(1);
				}
		}
	}

	if ( !listener ) {
		if ( argc - optind < 2 || !(col=strchr(argv[optind],':')) || (*col++=0, INADDR_NONE == inet_addr(argv[optind])) || 1 != sscanf(col,"%i",&port) || 1 != sscanf(argv[optind+1],"%i",&type) ) {
			usage(argv[0]);
			exit(1);
		}
	}

	
	sd = udpCommSocket(port);

	if ( sd < 0 ) {
		fprintf(stderr,"udpCommSocket: %s",strerror(-sd));
		exit(1);
	}

	if ( listener ) {
		streamdump(sd);
	} else {
		if ( PADCMD_STRM == (scmd.type = type) ) {
			scmd.flags    = (!isbe() ^ (badEndian == 1)) ? PADCMD_STRM_FLAG_LE : 0;
			scmd.port     = htons(port+1);
			scmd.nsamples = htonl(nsamples);

			if ( colMajor )
				scmd.flags |= PADCMD_STRM_FLAG_CM;
		}
		client(sd, argv[optind], port, &scmd);
	}

	udpCommClose(sd);
}
