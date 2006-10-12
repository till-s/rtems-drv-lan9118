#ifndef DRV_LAN_PROTO_DEFS_H
#define DRV_LAN_PROTO_DEFS_H

/* $Id$ */

/* Raw packet driver; Protocol Header Definitions             */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2006 */

#include <stdint.h>

#define EH_PAD_BYTES 2

/* uint32_t aligned ethernet header */
typedef struct EtherHeaderRec_ {
	uint8_t		pad[EH_PAD_BYTES];
	uint8_t		dst[6];
	uint8_t		src[6];
	uint16_t	type;
} EtherHeaderRec;

typedef struct IpHeaderRec_ {
	uint8_t		vhl;
	uint8_t		tos;
	uint16_t	len;
	uint16_t	id;
	uint16_t	off;
	uint8_t		ttl;
	uint8_t		prot;
	uint16_t	csum;
	uint32_t	src;
	uint32_t	dst;
} IpHeaderRec;

typedef struct UdpHeaderRec_ {
	uint16_t	sport;
	uint16_t	dport;
	uint16_t	len;
	uint16_t	csum;
} UdpHeaderRec;

/* sizeof(UdpPacketRec) is 44 */
typedef struct UdpPacketRec_ {
	EtherHeaderRec	eh;
	IpHeaderRec		ih;
	UdpHeaderRec 	uh;
} UdpPacketRec;

typedef struct IpArpRec_ {
	uint16_t	htype;
	uint16_t	ptype;
	uint8_t		hlen;
	uint8_t		plen;
	uint16_t	oper;
	uint8_t		sha[6];
	uint8_t		spa[4];
	uint8_t		tha[6];
	uint8_t		tpa[4];
} IpArpRec;

typedef struct IcmpHeaderRec_ {
	uint8_t		type;
	uint8_t		code;
	uint16_t	csum;
	uint16_t	ident;
	uint16_t	seq;
} IcmpHeaderRec;

#endif
