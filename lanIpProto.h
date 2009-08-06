#ifndef DRV_LAN_PROTO_DEFS_H
#define DRV_LAN_PROTO_DEFS_H

/* $Id$ */

/* Raw packet driver; Protocol Header Definitions             */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2006 */

#include <stdint.h>

#define EH_PAD_BYTES 2

/* Guarantee packet alignment to 32 bytes;
 * (particular chip driver may have stricter
 * requirements and the buffer memory will
 * then be aligned accordingly)
 */
#define LAN_IP_BASIC_PACKET_ALIGNMENT 32

/* uint32_t aligned ethernet header */
typedef struct EthHeaderRec_ {
	uint8_t		pad[EH_PAD_BYTES];
	uint8_t		dst[6];
	uint8_t		src[6];
	uint16_t	type;
} EthHeaderRec;

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

/*
 * Max. packet size incl. header, FCS-space and 2-byte padding (which is
 * never transmitted on the wire).
 */
#define LANPKTMAX			(1500+14+2+4)

#define IPPAYLOADSIZE		(LANPKTMAX - sizeof(EthHeaderRec) - sizeof(IpHeaderRec) - 4)
#define UDPPAYLOADSIZE		(IPPAYLOADSIZE - sizeof(UdpHeaderRec))
#define ICMPPAYLOADSIZE		(IPPAYLOADSIZE - sizeof(IcmpHeaderRec))

typedef struct LanEtherRec_ {
	EthHeaderRec    ll;
	uint8_t			pld[];
} LanEtherRec, *LanEther;

typedef struct LanArpRec_ {
	EthHeaderRec    ll;
	IpArpRec		arp;
} LanArpRec, *LanArp;

typedef struct LanIpRec_ {
	EthHeaderRec    ll;
	IpHeaderRec     ip;
} LanIpRec, *LanIp;

typedef struct LanIpHeaderRec_ {
	LanIpRec        hdr;
	uint8_t         pld[];
} LanIpHeaderRec, *LanIpHeader;

typedef struct LanIcmpHeaderRec_ {
	LanIpRec        hdr;
	IcmpHeaderRec	icmp;
	uint8_t			pld[];
} LanIcmpHeaderRec, *LanIcmpHeader;

/* sizeof(LanUdpHeaderRec) is 44 */
typedef struct LanUdpHeaderRec_ {
	LanIpRec		hdr;
	UdpHeaderRec	udp;
	uint8_t			pld[];
} LanUdpHeaderRec, *LanUdpHeader;

typedef union LanIpPacketHeaderRec_ {
	LanEtherRec			eth_S;
	LanArpRec			arp_S;
	LanIpHeaderRec		ip_S;
	LanIcmpHeaderRec	icmp_S;
	LanUdpHeaderRec		udp_S;
} LanIpPacketHeaderRec, *LanIpPacketHeader;

typedef union LanIpPacketRec_ {
	LanIpPacketHeaderRec	p_u;
	char					raw[LANPKTMAX];
} LanIpPacketRec, *LanIpPacket;

#define lpkt_eth(p)		(p)->p_u.eth_S.ll
#define lpkt_arp(p)	    (p)->p_u.arp_S.arp
#define lpkt_ip(p)		(p)->p_u.ip_S.hdr.ip
#define lpkt_udp(p)		(p)->p_u.udp_S.udp
#define lpkt_icmp(p)	(p)->p_u.icmp_S.icmp

#define lpkt_iphdr(p)	(p)->p_u.ip_S.hdr
#define lpkt_udphdr(p)	(p)->p_u.udp_S

#define lpkt_eth_pld(p,type)	(((union { char c[sizeof(type)]; type x; } *)(p)->p_u.eth_S.pld)->x)
#define lpkt_ip_pld(p,type)	    (((union { char c[sizeof(type)]; type x; } *)(p)->p_u.ip_S.pld)->x)
#define lpkt_udp_pld(p,type)	(((union { char c[sizeof(type)]; type x; } *)(p)->p_u.udp_S.pld)->x)

#define ETHPKTSZ(eth_payload_sz)	((eth_payload_sz) + sizeof(EthHeaderRec))
#define IPPKTSZ(ip_payload_sz)      ETHPKTSZ((ip_payload_sz) + sizeof(IpHeaderRec))
#define	UDPPKTSZ(udp_payload_sz)	IPPKTSZ((udp_payload_sz) + sizeof(UdpHeaderRec))

#endif
