#include <rtems.h>

#include <mcf5282/mcf5282.h>
#include <rtems/rtems/cache.h>

#include <sys/socket.h>
#include <sys/sockio.h>
#include <net/if.h>

#include <dev/mii/mii.h>

#define __KERNEL__
#include <rtems/rtems_mii_ioctl.h>
#undef __KERNEL__

#include <stdio.h>

#define __GET_BITS(x,s,m)	(((x)>>s)&m)
#define __SET_BITS(x,s,m)	(((x)&m)<<s)

typedef enum {
	RX_DATA_FIFO	= 0x00,
	/* aliased up to  0x1c */
	TX_DATA_FIFO	= 0x20,
	/* aliased up to  0x3c */
	RX_STS_FIFO	= 0x40,
	RX_STS_PEEK	= 0x44,
	TX_STS_FIFO	= 0x48,
	TX_STS_PEEK	= 0x4c,
	ID_REV		= 0x50,
#define ID_REV_ID_GET(x)  ((x)>>16)
#define ID_REV_REV_GET(x) ((x)&0xffff)
	IRQ_CFG		= 0x54,
#define IRQ_CFG_INT_DEAS_GET(x) __GET_BITS(x,24,0xff)
#define IRQ_CFG_INT_DEAS_SET(x) __SET_BITS(x,24,0xff)
#define IRQ_CFG_INT_DEAS_CLR	(1<<14)
#define IRQ_CFG_INT_DEAS_STS	(1<<13)
#define IRQ_CFG_IRQ_INT		(1<<12)
#define IRQ_CFG_IRQ_EN		(1<< 8)
#define IRQ_CFG_IRQ_POL		(1<< 4)
#define IRQ_CFG_IRQ_PUSHPULL	(1<< 0)
	INT_STS		= 0x58,
#define SW_INT			(1<<31)
#define TXSTOP_INT		(1<<25)
#define RXSTOP_INT		(1<<24)
#define RXDFH_INT		(1<<23)
#define TIOC_INT		(1<<21)
#define RXD_INT			(1<<20)
#define GPT_INT			(1<<19)
#define PHY_INT			(1<<18)
#define PME_INT			(1<<17)
#define TXSO_INT		(1<<16)
#define RWT_INT			(1<<15)
#define RXE_INT			(1<<14)
#define TXE_INT			(1<<13)
#define TDFU_INT		(1<<11)
#define TDFO_INT		(1<<10)
#define TDFA_INT		(1<< 9)
#define TSFF_INT		(1<< 8)
#define TSFL_INT		(1<< 7)
#define RXDF_INT		(1<< 6)
#define RDFL_INT		(1<< 5)
#define RSFF_INT		(1<< 4)
#define RSFL_INT		(1<< 3)
#define GPIO2_INT		(1<< 2)
#define GPIO1_INT		(1<< 1)
#define GPIO0_INT		(1<< 0)
	INT_EN		= 0x5c,
	BYTE_TEST	= 0x64,
	FIFO_INT	= 0x68,
#define FIFO_INT_TDFA_LVL_GET(x)	__GET_BITS(x,24,0xff)
#define FIFO_INT_TDFA_LVL_SET(x)	__SET_BITS(x,24,0xff)
#define FIFO_INT_TSFL_LVL_GET(x)	__GET_BITS(x,16,0xff)
#define FIFO_INT_TSFL_LVL_SET(x)	__SET_BITS(x,16,0xff)
#define FIFO_INT_RDFL_LVL_GET(x)	__GET_BITS(x, 8,0xff)
#define FIFO_INT_RDFL_LVL_SET(x)	__SET_BITS(x, 8,0xff)
#define FIFO_INT_RSFL_LVL_GET(x)	__GET_BITS(x, 0,0xff)
#define FIFO_INT_RSFL_LVL_SET(x)	__SET_BITS(x, 0,0xff)
	RX_CFG		= 0x6c,
#define RX_CFG_END_ALIGN_MSK	(3<<30)
#define RX_CFG_END_ALIGN_32	(2<<30)
#define RX_CFG_END_ALIGN_16	(1<<30)
#define RX_CFG_END_ALIGN_4	(0<<30)
#define RX_CFG_DMA_CNT_GET(x)		__GET_BITS(x,16,0xfff)
#define RX_CFG_DMA_CNT_SET(x)		__SET_BITS(x,16,0xfff)
#define RX_CFG_RX_DUMP		(1<<15)
#define RX_CFG_RXDOFF_GET(x)		__GET_BITS(x, 8,0x1f)
#define RX_CFG_RXDOFF_SET(x)		__SET_BITS(x, 8,0x1f)
	TX_CFG		= 0x70,
#define TX_CFG_TXS_DUMP		(1<<15)
#define TX_CFG_TXD_DUMP		(1<<14)
#define TX_CFG_TXSAO		(1<< 2)
#define TX_CFG_TX_ON		(1<< 1)
#define TX_CFG_STOP_TX		(1<< 0)
	HW_CFG		= 0x74,
#define HW_CFG_TTM		(1<<21)
#define HW_CFG_SF		(1<<20)
#define HW_CFG_TX_FIF_SZ_GET(x)		__GET_BITS(x,16,0xf)
#define HW_CFG_TX_FIF_SZ_SET(x)		__SET_BITS(x,16,0xf)
#define HW_CFG_TR_LO		(0<<12)
#define HW_CFG_TR_MED_LO	(1<<12)
#define HW_CFG_TR_MED_HI	(2<<12)
#define HW_CFG_TR_HI		(3<<12)
#define HW_CFG_TR_MSK		(3<<12)
#define HW_CFG_PORT_32		(1<< 2)
#define HW_CFG_SRST_TO		(1<< 1)
#define HW_CFG_SRST		(1<< 0)
	RX_DP_CTL	= 0x78,
#define RX_DP_CTL_FFWD		(1<<31)
	RX_FIFO_INF	= 0x7c,
#define RX_FIFO_INF_RXSUSED_GET(x)	__GET_BITS(x,16,0xff)
#define RX_FIFO_INF_RXDUSED_GET(x)	__GET_BITS(x, 0,0xffff)
	TX_FIFO_INF	= 0x80,
#define TX_FIFO_INF_TXSUSED_GET(x)	__GET_BITS(x,16,0xff)
#define TX_FIFO_INF_TXDFREE_GET(x)	__GET_BITS(x, 0,0xffff)
	PMT_CTL		= 0x84,
/* unused by this driver */
	GPIO_CFG	= 0x88,
#define GPIO_CFG_LED3_EN	(1<<30)
#define GPIO_CFG_LED2_EN	(1<<29)
#define GPIO_CFG_LED1_EN	(1<<28)
#define GPIO_CFG_GPIO2_INT_POL	(1<<26)
#define GPIO_CFG_GPIO1_INT_POL	(1<<25)
#define GPIO_CFG_GPIO0_INT_POL	(1<<24)
#define GPIO_CFG_EEPR_EN_GET(x)		__GET_BITS(x,20,7)
#define GPIO_CFG_EEPR_EN_SET(x)		__SET_BITS(x,20,7)
#define GPIO_CFG_GPIOBUF2	(1<<18)
#define GPIO_CFG_GPIOBUF1	(1<<17)
#define GPIO_CFG_GPIOBUF0	(1<<16)
#define GPIO_CFG_GPIODIR2	(1<<10)
#define GPIO_CFG_GPIODIR1	(1<< 9)
#define GPIO_CFG_GPIODIR0	(1<< 8)
#define GPIO_CFG_GPOD4		(1<< 4)
#define GPIO_CFG_GPOD3		(1<< 3)
#define GPIO_CFG_GPIOD2		(1<< 2)
#define GPIO_CFG_GPIOD1		(1<< 1)
#define GPIO_CFG_GPIOD0		(1<< 0)
	GPT_CFG		= 0x8c,
#define GPT_CFG_TIMER_EN	(1<<29)
#define GPT_CFG_LOAD_GET(x)		__GET_BITS(x,0,0xffff)
#define GPT_CFG_LOAD_SET(x)		__SET_BITS(x,0,0xffff)
	GPT_CNT		= 0x90,
#define GPT_CNT_GET(x)			__GET_BITS(x,0,0xffff)
	ENDIAN		= 0x98,
	FREE_RUN	= 0x9c,
	RX_DROP		= 0xa0,
	MAC_CSR_CMD	= 0xa4,
#define MAC_CSR_CMD_BUSY	(1<<31)
#define MAC_CSR_CMD_RnW		(1<<30)
#define MAC_CSR_CMD_ADDR_GET(x)		__GET_BITS(x,0,0xff)
#define MAC_CSR_CMD_ADDR_SET(x)		__SET_BITS(x,0,0xff)
	MAC_CSR_DATA	= 0xa8,
	AFC_CFG		= 0xac,
#define AFC_CFG_AFC_HI_GET(x)		__GET_BITS(x,16,0xff)
#define AFC_CFG_AFC_HI_SET(x)		__SET_BITS(x,16,0xff)
#define AFC_CFG_AFC_LO_GET(x)		__GET_BITS(x, 8,0xff)
#define AFC_CFG_AFC_LO_SET(x)		__SET_BITS(x, 8,0xff)
#define AFC_CFG_BACK_DUR_GET(x)		__GET_BITS(x, 4,0x0f)
#define AFC_CFG_BACK_DUR_SET(x)		__SET_BITS(x, 4,0x0f)
#define AFC_CFG_FCMULT		(1<<3)
#define AFC_CFG_FCBRD		(1<<2)
#define AFC_CFG_FCADD		(1<<1)
#define AFC_CFG_FCANY		(1<<0)
	E2P_CMD		= 0xb0,
#define E2P_CMD_BUSY		(1<<31)
#define E2P_CMD_EPC_CMD_GET(x)		_GET_BITS(x,28,7)
#define E2P_CMD_EPC_CMD_SET(x)		_SET_BITS(x,28,7)
#define E2P_CMD_EPC_TIMEOUT	(1<<9)
#define E2P_CMD_MAC_LOADED	(1<<8)
#define E2P_CMD_EPC_ADDR_GET(x)		_GET_BITS(x, 0,0xff)
#define E2P_CMD_EPC_ADDR_SET(x)		_SET_BITS(x, 0,0xff)
	E2P_DATA	= 0xb4,
} DrvLan9118RegOff;

typedef enum {
	MAC_CR		= 0x01,
#define MAC_CR_RXALL		(1<<31)
#define MAC_CR_RCVOWN		(1<<23)
#define MAC_CR_LOOPBK		(1<<21)
#define MAC_CR_FDPX		(1<<20)
#define MAC_CR_MCPAS		(1<<19)
#define MAC_CR_PRMS		(1<<18)
#define MAC_CR_INVFILT		(1<<17)
#define MAC_CR_PASSPAD		(1<<16)
#define MAC_CR_HO		(1<<15)
#define MAC_CR_HPFILT		(1<<13)
#define MAC_CR_LCOLL		(1<<12)
#define MAC_CR_BCAST		(1<<11)
#define MAC_CR_DISRTY		(1<<10)
#define MAC_CR_PADSTR		(1<< 8)
#define MAC_CR_BOLMT_10BITS	(0<<6)
#define MAC_CR_BOLMT_8BITS	(1<<6)
#define MAC_CR_BOLMT_4BITS	(2<<6)
#define MAC_CR_BOLMT_1BITS	(3<<6)
#define MAC_CR_BOLMT_MSK	(3<<6)
#define MAC_CR_DFCK		(1<<5)
#define MAC_CR_TXEN		(1<<3)
#define MAC_CR_RXEN		(1<<2)
	ADDRH		= 0x02,
	ADDRL		= 0x03,
	HASHH		= 0x04,
	HASHL		= 0x05,
	MII_ACC		= 0x06,
#define MII_ACC_PHY_GET(x)		__GET_BITS(x,11,0x1f)
#define MII_ACC_PHY_SET(x)		__SET_BITS(x,11,0x1f)
#define MII_ACC_MIIRIND_GET(x)		__GET_BITS(x, 6,0x1f)
#define MII_ACC_MIIRIND_SET(x)		__SET_BITS(x, 6,0x1f)
#define MII_ACC_MIIWnR		(1<<1)
#define MII_ACC_MIIBSY		(1<<0)
	MII_DATA	= 0x07,
	FLOW		= 0x08,
#define FLOW_FCPT_GET(x)		__GET_BITS(x,16,0xffff)
#define FLOW_FCPT_SET(x)		__SET_BITS(x,16,0xffff)
#define FLOW_FCPASS		(1<<2)
#define FLOW_FCEN		(1<<1)
#define FLOW_FCBSY		(1<<0)
	VLAN1		= 0x09,
	VLAN2		= 0x0a,
	WUFF		= 0x0b,
	WUCSR		= 0x0c,
} DrvLan9118MacCsrRegOff;

typedef struct DrvLan9118Rec_ {
	uint32_t	base;
} DrvLan9118Rec, *DrvLan9118;

/* TX buffer command words */

#define TXCMD_A_IRQ		(1<<31)
#define TXCMD_A_END_ALIGN_4	(0<<24)
#define TXCMD_A_END_ALIGN_16	(1<<24)
#define TXCMD_A_END_ALIGN_32	(2<<24)
#define TXCMD_A_END_ALIGN_MSK	(3<<24)
#define TXCMD_A_START_ALIGN_GET(x)	__GET_BITS(x,16,0x1f)
#define TXCMD_A_START_ALIGN_SET(x)	__SET_BITS(x,16,0x1f)
#define TXCMD_A_FIRST		(1<<13)
#define TXCMD_A_LAST		(1<<12)
#define TXCMD_A_BUFSIZ_GET(x)		__GET_BITS(x,0,0x7ff)
#define TXCMD_A_BUFSIZ_SET(x)		__SET_BITS(x,0,0x7ff)

#define TXCMD_B_TAG_GET(x)		__GET_BITS(x,16,0xffff)
#define TXCMD_B_TAG_SET(x)		__SET_BITS(x,16,0xffff)
#define TXCMD_B_FCSGEN_DISABLE	(1<<13)
#define TXCMD_B_PAD_DISABLE	(1<<12)
#define TXCMD_B_PKTLEN_GET(x)		__GET_BITS(x,0,0x7ff)
#define TXCMD_B_PKTLEN_SET(x)		__SET_BITS(x,0,0x7ff)

/* TX status word          */
#define TXSTS_TAG_GET(x)		__GET_BITS(x,16,0xffff)
#define TXSTS_TAG_SET(x)		__SET_BITS(x,16,0xffff)
#define TXSTS_ERROR		(1<<15)
#define TXSTS_CARRIER_LOSS	(1<<11)
#define TXSTS_NO_CARRIER	(1<<10)
#define TXSTS_LATE_COLL		(1<< 9)
#define TXSTS_EXCESS_COLL	(1<< 8)
#define TXSTS_COLL_CNT_GET(x)		__GET_BITS(x,3,0xf)
#define TXSTS_COLL_CNT_SET(x)		__SET_BITS(x,3,0xf)
#define TXSTS_EXCESS_DEFER	(1<< 2)
#define TXSTS_FIFO_UNDERRUN	(1<< 1)
#define TXSTS_DEFERRED		(1<< 0)

DrvLan9118Rec theLan9118;

extern void Timer_initialize();
extern uint32_t Read_timer();


/* Hmm __IPSBAR is defined in the linker script :-( */

#ifndef __IPSBAR
#define __IPSBAR ((volatile uint8_t *)0x40000000)
#endif
          
/* Seems to be missing */
#ifndef MCF5282_DMA_DCR_DSIZE_LINE
#define MCF5282_DMA_DCR_DSIZE_LINE                      (0x00030000)
#endif

#include <stdint.h>

#define FIFO_ADDR ((volatile uint16_t *)0x30000000)
#define LAN_9118_BASE  (0x31000000)

/* uC5282-specific delay */
#define DELAY45ns()  do { __asm__ __volatile__("nop"); } while (0)
#define DELAY90ns()  do { __asm__ __volatile__("nop; nop"); } while (0)
#define DELAY135ns() do { __asm__ __volatile__("nop; nop; nop"); } while (0)
#define DELAY180ns() do { __asm__ __volatile__("nop; nop; nop; nop"); } while (0)

/* Register access */


/* WARNING: CONSULT SECTIONS 6.1.1 / 6.1.2; READING ALMOST ANY REGISTER
 *          AFTER A WRITE OPERATION AND READING SOME REGISTERS AFTER CERTAIN
 *          READ OPERATIONS REQUIRES APPROPRIATE DELAYS
 */
static inline uint32_t	rd9118Reg(DrvLan9118 plan, DrvLan9118RegOff off)
{
	return *(volatile uint32_t *)(plan->base + off);
}

static inline uint32_t	rd9118RegSlow(DrvLan9118 plan, DrvLan9118RegOff off)
{
	/* always do a worst case delay for now; the read cycle itself takes another ~160ns */
	DELAY180ns();
	return rd9118Reg(plan, off);
}

static inline void	wr9118Reg(DrvLan9118 plan, DrvLan9118RegOff off, uint32_t val)
{
	*(volatile uint32_t *)(plan->base + off) = val;
}

static void macCsrAccess(DrvLan9118 plan, uint32_t rNw, int addr)
{
uint32_t v = MAC_CSR_CMD_BUSY | MAC_CSR_CMD_ADDR_SET(addr) | rNw;
	wr9118Reg(plan, MAC_CSR_CMD, v);
	DELAY45ns();
	while ( MAC_CSR_CMD_BUSY & rd9118Reg(plan, MAC_CSR_CMD) )
		/* poll */;
}

uint32_t
macCsrRead(DrvLan9118 plan, uint32_t reg)
{
	macCsrAccess(plan, MAC_CSR_CMD_RnW, reg);
	return rd9118Reg(plan, MAC_CSR_DATA);
}

void
macCsrWrite(DrvLan9118 plan, uint32_t reg, uint32_t v)
{
	wr9118Reg(plan, MAC_CSR_DATA, v);
	macCsrAccess(plan, 0, reg);
}

static void miiAccess(DrvLan9118 plan, uint32_t wNr, uint32_t addr)
{
uint32_t v = wNr | MII_ACC_PHY_SET(0x01) | MII_ACC_MIIRIND_SET(addr) | MII_ACC_MIIBSY;
	macCsrWrite(plan, MII_ACC, v);
	while ( MII_ACC_MIIBSY & macCsrRead(plan, MII_ACC) )
		/* poll */;
}

int
lan9118_mdio_w(int phy, void *uarg, unsigned reg, uint32_t val)
{
	if ( phy > 1 )
		return -1;
	macCsrWrite(uarg, MII_DATA, val);
	miiAccess(uarg, MII_ACC_MIIWnR, reg);
	return 0;
}

int
lan9118_mdio_r(int phy, void *uarg, unsigned reg, uint32_t *pval)
{
	if ( phy > 1 )
		return -1;
	miiAccess(uarg, 0, reg);
	*pval = macCsrRead(uarg, MII_DATA);
	return 0;
}

static struct rtems_mdio_info lan9118_mdio = {
	mdio_r: lan9118_mdio_r,
	mdio_w: lan9118_mdio_w,
	has_gmii: 0,
};

/* I'm not a fan of those macros...
 * Note that e.g., MCF5282_EPDR_EPD(bit) doesn't protect 'bit' in the
 * expansion -- this tells me that whoever wrote those was maybe a novice...
 */

DrvLan9118
drvLan9118Setup(unsigned char *enaddr)
{
DrvLan9118 plan;
uint32_t   tmp,i;

	if ( !enaddr ) {
		fprintf(stderr,"Need ethernet address (6 bytes) argument\n");
		return 0;
	}
	/* CHIP SELECT setup */

	/* No need to change CSAR, we use the BSP setup */

	/* open 1M window, allow supervisory and user data read access */
	MCF5282_CS2_CSMR = MCF5282_CS_CSMR_BAM_1M
                           /* | MCF5282_CS_CSMR_WP */
                           /* | MCF5282_CS_CSMR_AM */  /* Allow DMA access */
                           | MCF5282_CS_CSMR_CI
                           | MCF5282_CS_CSMR_SC
                           /* | MCF5282_CS_CSMR_SD */
                           | MCF5282_CS_CSMR_UC
                           /* | MCF5282_CS_CSMR_UD */
       	                   ; 


	/* use 2 wait states, internal termination; bus protocol implementation doesn't allow burst reads :-( */
	MCF5282_CS2_CSCR = MCF5282_CS_CSCR_WS(2) | MCF5282_CS_CSCR_AA | MCF5282_CS_CSCR_PS_16;

	MCF5282_CS2_CSMR |= MCF5282_CS_CSMR_V;

	/* EPORT & GPIO setup */

	/* make sure interrupts are masked */
	MCF5282_EPORT_EPIER &= ~(MCF5282_EPORT_EPIER_EPIE4);

	/* make pin 4 active low, level triggered */
	MCF5282_EPORT_EPPAR &= ~MCF5282_EPORT_EPPAR_EPPA4_BOTHEDGE;
	MCF5282_EPORT_EPPAR |= ~MCF5282_EPORT_EPPAR_EPPA4_LEVEL;

	MCF5282_EPORT_EPDDR &= ~MCF5282_EPORT_EPDDR_EPDD4;

#if 0
	/* clear outputs 1&3 */
	MCF5282_EPORT_EPDR  &= ~(MCF5282_EPORT_EPDR_EPD1 | MCF5282_EPORT_EPDR_EPD1);

	/* make pin 1+3 outputs, pin 2 an input */
	MCF5282_EPORT_EPDDR |= MCF5282_EPORT_EPDDR_EPDD3 | MCF5282_EPORT_EPDDR_EPDD1;
	MCF5282_EPORT_EPDDR &= ~MCF5282_EPORT_EPDDR_EPDD2;
#endif

	/* Initialize the 9118 */
	theLan9118.base = LAN_9118_BASE;
	plan = &theLan9118;

	/* First, we must perform a read access to the BYTE TEST register */
	tmp = rd9118Reg(&theLan9118,BYTE_TEST);
	
	/* Setup for big endian mode */
	if ( 0x87654321 != tmp )
		wr9118Reg(plan, ENDIAN, 0xffffffff);

	/* soft reset; first the PHY, then the chip */
	lan9118_mdio_w(0, plan, MII_BMCR, BMCR_RESET);
	do {
		lan9118_mdio_r(0, plan, MII_BMCR, &tmp);
	} while ( BMCR_RESET & tmp );

	/* chip soft reset; endianness is unaffected */
	wr9118Reg(plan, HW_CFG, HW_CFG_SRST);
	DELAY45ns();
	do { 
		tmp = rd9118Reg(plan, HW_CFG);
		if ( HW_CFG_SRST_TO & tmp ) {
			fprintf(stderr,"ERROR: Soft Reset Timeout\n");
			return 0;
		}
	} while (HW_CFG_SRST & tmp);

	/* Enable LEDs */
	wr9118Reg(plan, GPIO_CFG, GPIO_CFG_LED3_EN | GPIO_CFG_LED2_EN | GPIO_CFG_LED1_EN);

	/* start transmitter and configure to allow status overruns */	
	wr9118Reg(plan, TX_CFG, TX_CFG_TXSAO | TX_CFG_TX_ON);

	/* Setup the MAC but don't start the receiver */
	macCsrWrite(plan, MAC_CR, MAC_CR_FDPX | MAC_CR_TXEN);
	tmp = (enaddr[0]<<8) | enaddr[1];
	macCsrWrite(plan, ADDRH, tmp);
	for (i=0,tmp=0; i<4; i++) {
		tmp = (tmp<<8) | enaddr[i+2];
	}
	macCsrWrite(plan, ADDRL, tmp);

	/* Setup the PHY */

	/* advertise 100baseTx including full duplex */
	lan9118_mdio_w(0, plan, MII_ANAR, ANAR_TX_FD | ANAR_TX | ANAR_CSMA);

	/* enable & start autonegotiation (also select 100 FD as a hint in case autoneg is switched off) */
	lan9118_mdio_w(0, plan, MII_BMCR, BMCR_SPEED0 | BMCR_AUTOEN | BMCR_FDX | BMCR_STARTNEG);

	return plan;
}

/* just the SIOCSIFMEDIA/SIOCGIFMEDIA ioctls */
int
drvLan9118ioctl(DrvLan9118 plan, int cmd, int *p_media)
{
	/* accept simple aliases for easy shell access */
	switch ( cmd ) {
		case 0 : cmd = SIOCGIFMEDIA; break;
		case 1 : cmd = SIOCSIFMEDIA; break;
		default: break;
	}
	return rtems_mii_ioctl(&lan9118_mdio, plan, cmd, p_media);
}

int
epdrTog(int bit)
{
uint32_t flags;
	if ( bit != 1 && bit != 3 )
		return -1;
	rtems_interrupt_disable(flags);
		MCF5282_EPORT_EPDR ^= MCF5282_EPORT_EPDR_EPD((bit));
	rtems_interrupt_enable(flags);
	return 0;
}

int 
tstLong()
{
	return *(volatile long*)0x31000064;
}

int
tstShort()
{
register unsigned a,b;
	a = *(volatile short*)0x31000064;
	b = *(volatile short*)0x31000066;
	return (a<<16) | b;
}
