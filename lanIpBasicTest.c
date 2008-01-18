#include <rtems.h>
#include <rtems/error.h>
#include <lanIpBasic.h>
#include <lanIpBasicTest.h>

#include <string.h>

#include <netinet/in.h>

#define PAYLDLEN 1024

#ifdef DRVLAN9118

#include <drvLan9118.h>

static uint8_t eeprom_shadow[100];
static int     shadow_len = 0;

#define SHADOW    eeprom_shadow
#define SHADOWLEN shadow_len
#define THE_SHADOWLEN (sizeof(eeprom_shadow)/sizeof(eeprom_shadow[0]))

static inline void
shutdown(void *drv)
{
	drvLan9118Shutdown((DrvLan9118_tps)drv);
}

static void *
setup(IpBscIf ipbif, uint8_t *enaddr)
{
void *rval = drvLan9118Setup(enaddr,0);
int   err;

	if ( rval ) {
		/* Cannot read eeprom once the driver is up -- therefore
		 * we read it's contents into memory...
		 */
		if ( (err = drvLan9118E2PRead(rval, eeprom_shadow, 0, THE_SHADOWLEN)) ) {
			fprintf(stderr,"Unable to read 9118 EEPROM: %s\n", strerror(-err));
		} else {
			SHADOWLEN = THE_SHADOWLEN;
		}
	}
	return rval;
}

static int
start(void *drv, IpBscIf ipbif, int pri)
{
	return drvLan9118Start(drv, pri, 0,
				drvLan9118IpRxCb, ipbif,
				0, 0,
				0, 0,
				0, 0);
}

/* Must be initialized! */
extern uint32_t Read_timer();

#define READ_TIMER()	Read_timer()

#elif defined(DRVMVE)

#include <bsp/if_mve_pub.h>

#define SHADOW    0
#define SHADOWLEN 0

static rtems_id mve_tid = 0;

extern void
drvMveIpBasicShutdown(rtems_id tid);

extern void
drvMveIpBasicTask(rtems_task_argument arg);

extern rtems_id
drvMveIpBasicSetup(IpBscIf ipbif);

extern void *
drvMveIpBasicGetDrv(rtems_id tid);

static void *
setup(IpBscIf ipbif, uint8_t *enaddr)
{
	mve_tid = drvMveIpBasicSetup(ipbif);
	if ( mve_tid ) {
		return drvMveIpBasicGetDrv(mve_tid);
	}
	return 0;
}

static int
start(void *drv, IpBscIf ipbif, int pri)
{
rtems_task_priority op;

	if ( pri > 0 )
		rtems_task_set_priority(mve_tid, (rtems_task_priority)pri, &op);
	return RTEMS_SUCCESSFUL == rtems_task_start(mve_tid, drvMveIpBasicTask, (rtems_task_argument)ipbif) ? 0 : -1;
}

static inline void
shutdown(void *drv)
{
	drvMveIpBasicShutdown(mve_tid);
	mve_tid = 0;
}

static inline uint32_t READ_TIMER()
{
uint32_t t;
	asm volatile("mftb %0":"=r"(t));
	return t;
}

#elif defined(DRVAMD)

#define SHADOW    0
#define SHADOWLEN 0

extern void
drvAmdIpBasicShutdown(void *drv);

extern void
drvAmdIpBasicTask(rtems_task_argument arg);

extern void *
drvAmdIpBasicSetup(IpBscIf ipbif);

static void *
setup(IpBscIf ipbif, uint8_t *enaddr)
{
	return drvAmdIpBasicSetup(ipbif);
}

static int
start(void *drv, IpBscIf ipbif, int pri)
{
rtems_status_code sc;
rtems_id          tid;

	if ( pri <= 0 )
		pri = 20;

	sc = rtems_task_create(
				rtems_build_name('i','p','b','d'),
				pri,	/* can be changed later */
				10000,
				RTEMS_DEFAULT_MODES,
				RTEMS_FLOATING_POINT | RTEMS_LOCAL,
				&tid);

	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "creating drvAmdIpBasicTask\n");
		return -1;
	}

	sc = rtems_task_start( tid, drvAmdIpBasicTask, (rtems_task_argument)ipbif );
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc, "starting drvAmdIpBasicTask\n");
		rtems_task_delete( tid );
		return -1;
	}
	return 0;
}

static inline void
shutdown(void *drv)
{
	drvAmdIpBasicShutdown(drv);
}

#define READ_TIMER()	(0xdeadbeef)

#else
#error "unsupported driver"
#endif

void      *lanIpDrv  =  0;
IpBscIf   lanIpIf    =  0;
int		  lanIpUdpsd = -1;

const uint8_t * const lanIpEEPROMShadow = SHADOW;

/* Length of shadow area */

int      
lanIpEEPROMShadowLength()
{
	return SHADOWLEN;
}

void
lanIpTakedown()
{
	if ( lanIpUdpsd >= 0 ) {
		udpSockDestroy(lanIpUdpsd);
		lanIpUdpsd = -1;
	}
	if ( lanIpDrv ) {
		shutdown(lanIpDrv);
		lanIpDrv = 0;
	}
	if ( lanIpIf ) {
		lanIpBscIfDestroy( lanIpIf );
		lanIpIf = 0;
	}
}

int
lanIpSetup(char *ip, char *nmsk, int port, uint8_t *enaddr)
{
	if ( !ip || !nmsk ) {
		fprintf(stderr,"Usage: lanIpSetup(char *ip, char *netmask, int port, enaddr)\n");
		return -1;
	}
	if ( lanIpDrv ) {
		fprintf(stderr,"Can call setup only once\n");
		return -1;
	}

	if ( ! (lanIpIf = lanIpBscIfCreate()) ) {
		fprintf(stderr,"Unable to create callback data\n");
		goto egress;
	}

	lanIpDrv = setup(lanIpIf, enaddr);

	if ( !lanIpDrv )
		goto egress;

	lanIpBscIfInit(lanIpIf, lanIpDrv, ip, nmsk);

	/* Start driver */
	if ( start(lanIpDrv, lanIpIf, 0) ) {
		fprintf(stderr,"Unable to start driver\n");
		goto egress;
	}

	if ( port > 0 &&(lanIpUdpsd = udpSockCreate(port)) < 0 ) {
		fprintf(stderr,"Unable to create UDPSOCK: %s\n", strerror(-lanIpUdpsd));
		goto egress;
	}

	return 0;

egress:
	lanIpTakedown();
	return -1;
}

uint32_t lanIpTst_mintrip         = -1;
uint32_t lanIpTst_maxtrip         =  0;
uint32_t lanIpTst_avgtrip128      =  0;
uint32_t lanIpTst_pktlost         =  0;
uint32_t lanIpTst_pktsent         =  0;
volatile int lanIpTst_keeprunning =  1;

/* bounce a UDP packet back to the caller */

typedef uint32_t echodata[2];

int
udpSocketEcho(int sd, int raw, int idx, int timeout)
{
LanIpPacket p;
uint16_t    tmp;
int         len = -1, rval;
uint32_t	now, then;

static LanIpPacketRec dummy = {{{{{0}}}}};

	if ( idx < 0 || idx >= sizeof(echodata)/sizeof((*(echodata*)0)[0]) )
		return -1;

	if ( !lpkt_ip(&dummy).src ) {
		if ( (rval = udpSockHdrsInit(sd, &lpkt_udphdr(&dummy), 0, 0, 0)) ) {
			fprintf(stderr,"udpSocketEcho - Unable to initialize headers: %s\n", strerror(-rval));
			return rval;
		}
	}

	p = udpSockRecv(sd, timeout);

	if ( p ) {

		if ( raw ) {
			/* user manages headers */
			memcpy(lpkt_eth(p).dst, lpkt_eth(p).src, sizeof(lpkt_eth(p).dst));
			lpkt_ip(p).dst    = lpkt_ip(p).src;	
			lpkt_udp(p).dport = lpkt_udp(p).sport;
			{
				/* fill source ll address, IP address and UDP port */
				memcpy(lpkt_eth(p).src, lpkt_eth(&dummy).src, sizeof(lpkt_eth(&dummy).src));
				lpkt_ip(p).src    = lpkt_ip(&dummy).src;

				tmp               = lpkt_udp(p).dport;
				lpkt_udp(p).dport = lpkt_udp(p).sport;
				lpkt_udp(p).sport = tmp;

				lpkt_ip(p).csum   = 0;
				/*
				lpkt_ip(p).csum   = htons(in_cksum_hdr((void*)&lpkt_ip(p)));
				 */

				lpkt_udp(p).csum = 0;

			}
			now  = READ_TIMER();
			then = lpkt_udp_pld(p,echodata)[idx];
			lpkt_udp_pld(p,echodata)[idx] = now;
			len = udpSockSendBufRawIp(p);
		} else {
			now  = READ_TIMER();
			then = lpkt_udp_pld(p,echodata)[idx];
			lpkt_udp_pld(p,echodata)[idx] = now;
			len = ntohs(lpkt_udp(p).len) - sizeof(UdpHeaderRec);
			udpSockSend(sd, &lpkt_udp_pld(p,echodata), len);
			udpSockFreeBuf(p);
		}

		lanIpTst_pktsent++;

		/* If this was not the first packet then measure roundtrip */
		if ( then != 0 ) {
			now-=then;	
			if ( now < lanIpTst_mintrip )
				lanIpTst_mintrip = now;
			if ( now > lanIpTst_maxtrip )
				lanIpTst_maxtrip = now;
			/* avgtrip = (127*avgtrip + now)/128;
             * 128 * a = 127 * a + n = 127/128 * (128*a) + n
             */
            lanIpTst_avgtrip128 += (now - (lanIpTst_avgtrip128>>7));
			/* When reading avgtrip it must be divided by 128 */
		}
	}
	return len;
}

int
udpBouncer(int master, int raw, uint32_t dipaddr, uint16_t dport)
{
LanIpPacket p;
int         err = -1;
	if ( !raw ) {
		if ( (err=udpSockConnect(lanIpUdpsd, dipaddr, dport)) ) {
			fprintf(stderr,"bouncer: Unable to connect socket: %s\n", strerror(-err));
			return err;
		}
	}

	if ( master ) {
		do {
			/* create a packet */
			if ( !(p=udpSockGetBuf()) ) {
				fprintf(stderr,"bouncer: Unable to allocate buffer\n");
				goto egress;		
			}
			if ( raw ) {
				/* fillin headers */
				udpSockHdrsInit(lanIpUdpsd, &lpkt_udphdr(p), dipaddr, dport, 0);
				udpSockHdrsSetlen(&lpkt_udphdr(p), PAYLDLEN);

				/* initialize timestamps */
				lpkt_udp_pld(p,echodata)[0] = 0;
				lpkt_udp_pld(p,echodata)[1] = READ_TIMER();
				udpSockSendBufRawIp(p);
			} else {
				/* initialize timestamps */
				lpkt_udp_pld(p,echodata)[0] = 0;
				lpkt_udp_pld(p,echodata)[1] = READ_TIMER();
				udpSockSend(lanIpUdpsd, p, PAYLDLEN);
				udpSockFreeBuf(p);
			}

			while ( udpSocketEcho(lanIpUdpsd, raw, 1, 20) > 0 ) {
				if ( !lanIpTst_keeprunning ) {
					lanIpTst_pktlost--;
					break;
				}
			}

			lanIpTst_pktlost++;

		} while ( lanIpTst_keeprunning );
			fprintf(stderr,"Master terminated.\n");
			/* in case they want to start again */
			lanIpTst_keeprunning=1;
	} else {
			/* make sure socket is flushed */
			while ( (p = udpSockRecv(lanIpUdpsd,0)) )		
				udpSockFreeBuf(p);
			while (udpSocketEcho(lanIpUdpsd, raw, 0, 100) > 0)
					;
			fprintf(stderr,"Slave timed out; terminating\n");
	}
	err = 0;

egress:
	if ( !raw ) {
		if ( (err=udpSockConnect(lanIpUdpsd, 0, 0)) ) {
			fprintf(stderr,"bouncer: Unable to disconnect socket: %s\n", strerror(-err));
		}
	}
	return(err);
}


int
_cexpModuleFinalize(void* unused)
{
	if ( lanIpDrv || lanIpUdpsd>=0 || lanIpIf ) {
		fprintf(stderr,"Module still in use, use 'lanIpTakedown()'\n");
		return -1;
	}
	return 0;
}
