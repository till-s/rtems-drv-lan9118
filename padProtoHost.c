/* $Id$ */


/* Wrapper program to send padProto requests */

#ifndef BSDSOCKET
#define  BSDSOCKET
#endif

#include <padProto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int
main(int argc, char **argv)
{
	int sd;
	int err;
	int type;
	int port=0;
	char *col = 0;
	struct sockaddr_in x;

	UdpCommPkt p = 0;

	if ( argc < 3 || !(col=strchr(argv[1],':')) || (*col++=0, INADDR_NONE == inet_addr(argv[1])) || 1 != sscanf(col,"%i",&port) || 1 != sscanf(argv[2],"%i",&type) ) {
		fprintf(stderr,"Usage: %s ip:port <msg_type_int>\n",argv[0]);
		exit(1);
	}

	
	sd = udpCommSocket(4444);

	if ( sd < 0 ) {
		fprintf(stderr,"udpCommSocket: %s",strerror(-sd));
		exit(1);
	}

	if ( (err = udpCommConnect(sd, inet_addr(argv[1]), port)) ) {
		fprintf(stderr,"udpCommConnect: %s",strerror(-err));
		exit(1);
	}

	if ( (err=padRequest(sd, 3, type, 0, &p)) ) {
		fprintf(stderr,">>> Sending request failed: %s <<<\n",strerror(-err));
	}
	if (p) {
		printf("Packet Dump:");
		for (err=0; err<33; err++) {
			if ( err%16 == 0 )
				printf("\n");
			printf("%02X ",((uint8_t*)p)[err]);
		}
		printf("\n");
	}

	udpCommFreePacket(p);
	udpCommClose(sd);
}
