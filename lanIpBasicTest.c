#include <rtems.h>
#include <lanIpBasic.h>

#define DRVMVE
#ifdef DRVLAN9118

#include <drvLan9118.h>

static inline void
shutdown(void *drv)
{
	drvLan9118Shutdown((DrvLan9118_tps)drv);
}

static void *
setup(IpCbData cbd)
{
	return drvLan9118Setup(0,0);
}

static int
start(void *drv, IpCbData cbd, int pri)
{
	return drvLan9118Start(drv, pri, 0,
				drvLan9118IpRxCb, cbd,
				0, 0,
				0, 0,
				0, 0);
}
#elif defined(DRVMVE)

#include <bsp/if_mve_pub.h>

static rtems_id mve_tid = 0;

extern void
drvMveIpBasicShutdown(rtems_id tid);

extern void
drvMveIpBasicTask(rtems_task_argument arg);

extern rtems_id
drvMveIpBasicSetup(IpCbData cbd);

extern void *
drvMveIpBasicGetDrv(rtems_id tid);

static void *
setup(IpCbData cbd)
{
	mve_tid = drvMveIpBasicSetup(cbd);
	if ( mve_tid ) {
		return drvMveIpBasicGetDrv(mve_tid);
	}
	return 0;
}

static int
start(void *drv, IpCbData cbd, int pri)
{
rtems_task_priority op;

	if ( pri > 0 )
		rtems_task_set_priority(mve_tid, (rtems_task_priority)pri, &op);
	return RTEMS_SUCCESSFUL == rtems_task_start(mve_tid, drvMveIpBasicTask, (rtems_task_argument)cbd) ? 0 : -1;
}

static inline void
shutdown(void *drv)
{
	drvMveIpBasicShutdown(mve_tid);
	mve_tid = 0;
}

#else
#error "unsupported driver"
#endif

void           *plan = 0;
IpCbData       cbdta = 0;
int			   udpsd = -1;

void
lanIpTakedown()
{
	if ( udpsd >= 0 ) {
		udpSockDestroy(udpsd);
		udpsd = -1;
	}
	if ( cbdta ) {
		lanIpCbDataDestroy( cbdta );
		cbdta = 0;
	}
	if ( plan ) {
		shutdown(plan);
		plan = 0;
	}
}

int
lanIpSetup(char *ip, char *nmsk, int port)
{
	if ( !ip || !nmsk || !port ) {
		fprintf(stderr,"Usage: lanIpSetup(char *ip, char *netmask, int port)\n");
		return -1;
	}
	if ( plan ) {
		fprintf(stderr,"Can call setup only once\n");
		return -1;
	}

	if ( ! (cbdta = lanIpCbDataCreate()) ) {
		fprintf(stderr,"Unable to create callback data\n");
		goto egress;
	}

	plan = setup(cbdta);

	if ( !plan )
		goto egress;

	lanIpCbDataInit(cbdta, plan, ip, nmsk);

	/* Start driver */
	if ( start(plan, cbdta, 0) ) {
		fprintf(stderr,"Unable to start driver\n");
		goto egress;
	}

	if ( (udpsd = udpSockCreate(port)) < 0 ) {
		fprintf(stderr,"Unable to create UDPSOCK\n");
		goto egress;
	}

	return (udpsd);

egress:
	lanIpTakedown();
	return -1;
}

int
_cexpModuleFinalize(void* unused)
{
	if ( plan || udpsd>=0 || cbdta ) {
		fprintf(stderr,"Module still in use, use 'lanIpTakedown()'\n");
		return -1;
	}
	return 0;
}
