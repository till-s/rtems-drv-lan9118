/*=============================================================================
 
		$Id$
  Name: drvLan9118.c

  Abs:  Raw packet driver for the lan9118 10/100 ethernet chip

  Auth: 12-oct-2006, Till Straumann (TSS)
  Rev:  

-----------------------------------------------------------------------------*/

/* 
 * Authorship
 * ----------
 * This software was created by
 *     Till Straumann <strauman@slac.stanford.edu>, 2006, 2007
 * 	   Stanford Linear Accelerator Center, Stanford University.
 * 
 * Acknowledgement of sponsorship
 * ------------------------------
 * This software was produced by
 *     the Stanford Linear Accelerator Center, Stanford University,
 * 	   under Contract DE-AC03-76SFO0515 with the Department of Energy.
 * 
 * Government disclaimer of liability
 * ----------------------------------
 * Neither the United States nor the United States Department of Energy,
 * nor any of their employees, makes any warranty, express or implied, or
 * assumes any legal liability or responsibility for the accuracy,
 * completeness, or usefulness of any data, apparatus, product, or process
 * disclosed, or represents that its use would not infringe privately owned
 * rights.
 * 
 * Stanford disclaimer of liability
 * --------------------------------
 * Stanford University makes no representations or warranties, express or
 * implied, nor assumes any liability for the use of this software.
 * 
 * Stanford disclaimer of copyright
 * --------------------------------
 * Stanford University, owner of the copyright, hereby disclaims its
 * copyright and all other rights in this software.  Hence, anyone may
 * freely use it for any purpose without restriction.  
 * 
 * Maintenance of notices
 * ----------------------
 * In the interest of clarity regarding the origin and status of this
 * SLAC software, this and all the preceding Stanford University notices
 * are to remain affixed to any copy or derivative of this software made
 * or distributed by the recipient and are to be affixed to any copy of
 * software made or distributed by the recipient that contains a copy or
 * derivative of this software.
 * 
 * ------------------ SLAC Software Notices, Set 4 OTT.002a, 2004 FEB 03
 */ 
 
/*-----------------------------------------------------------------------------
 
  Mod:  (newest to oldest)  
		$Log$
		Revision 1.35  2009/11/10 20:11:52  strauman
		 - added parenthesis to improve readability.
		
		Revision 1.34  2009/10/18 02:17:42  strauman
		
		2009/10/17 (TS):
		 - drvLan9118.c: Replaced SEM_SMTX by SEM_MUTX (see above).
		
		Revision 1.33  2009/09/11 01:03:03  strauman
		2009/09/10 (TS):
		 - drvLan9118.c, drvUdpSock.c, lanIpBasic.c, lanIpBasic.h,
		   lanIpBasicTest.c, lanIpProto.h, padStream.c: renamed and
		   restructured some of the protocol header structs (but I'm
		   still not quite happy).
		
		Revision 1.32  2009/08/30 01:53:13  strauman
		2009/08/29 (TS):
		 - drvLan9118.c: create a semaphore and block until driver task exits
		   when shutting-down the driver.
		
		Revision 1.31  2009/08/28 03:36:24  strauman
		2009/08/27 (TS):
		 - drvLan9118.h, drvLan9118.c, drvLan9118IpBasic.c: Added support for
		   multicast.
		
		Revision 1.30  2008/10/17 22:38:17  strauman
		 - cleaned-up and documented the 'setup' api, i.e., routines the drivers
		   must provide so that they can be attached/detached to/from the stack,
		   started and shutdown etc.
		 - removed EEPROM access from lanIpBasicTest API; this is only relevant
		   for the lan9118 where we couldn't read EEPROM after initializing the
		   device. We now read the 9118's EEPROM into a ram-shadow buffer
		   when we start the device and transparently provide cached data
		   to the user.
		 - do not access IP addresses inside of ARP packets as 32-bit entities
		   because they are not properly aligned. Provided helper inline routines
		   to do this (lanIpBasic.c).
		 - merged lanIpBscIfCreate + lanIpBscIfInit -> new API lanIpBscIfCreate().
		
		Revision 1.29  2008/10/16 23:55:51  strauman
		 - drvLan9118Shutdown(): ignore NULL argument and do nothing.
		
		Revision 1.28  2008/10/07 03:50:49  strauman
		 - rtems 4.9.0 renamed Timer_initialize/Read_timer -> benchmark_timer_initialize/
		   benchmark_timer_read :-(. All timer-related inlines were moved into
		   a new 'hwtmr.h' header which provides primitive implementations for
		   uC5282 and PPC (timebase).
		 - BUGFIX: byterev constraint must be a 'd' register, not any 'r'.
		
		Revision 1.27  2008/01/18 06:44:27  till
		 - renamed EtherHeader -> EthHeader; there was a name conflict with
		   SPEAR software / the AMD pcnet32 driver for which I wanted to add
		   support.
		 - added support for AMD 79c97x chips (PMC card which is handy; the
		   ultimate goal is using udpComm for a GDB connection.
		 - FIXED: endian-ness bug: IP checksums must not be converted to network
		   byte-order.
		
		Revision 1.26  2007-04-20 22:58:57  till
		 - added 'NOSEND' option  to simulator
		 - fixed typos with drvLan9118FifoAddr() routine

		Revision 1.25  2007-04-20 22:03:37  till
		 - Makefile: beautification
		 - added public interface to lan9118 FIFO (hack)

		Revision 1.24  2007-04-12 02:16:15  strauman
		 - print MAC address from DumpStats routine.

		Revision 1.23  2007-03-09 01:19:25  till
		 - silenced all debugging
		
		Revision 1.22  2007-03-09 00:32:01  till
		 - need to reload MAC address after reburning eeprom.

		Revision 1.21  2007-03-07 03:18:16  till
		 - added 'sanity check' to verify that all bits can be read and written.
		   This should catch hardware/wiring/soldering problems.

		Revision 1.20  2007/02/01 04:39:31  guest
		 - added slac copyright waiver
		
		Revision 1.19  2006/12/28 20:00:56  till
		 - use '__may_alias__' attribute instead of union (if __GNUC__)
		
		Revision 1.18  2006-12-15 04:13:33  till
		 - replaced 'rtems_interrupt_catch' by BSP_installVME_isr() API. Using the
		   latter we get BSP magic necessary for setting up the interrupt controller
		   [in addition to enabling the IRQ there a 'level' and 'priority' must also
		   be configured].

		Revision 1.17  2006/12/02 03:03:40  strauman
		 - redefined declarations of networking headers
		 - (hopefully) the new declarations and macros to access payloads
		   make this 'alias-safe' to meet ISOC99 alias rule.
		
		Revision 1.16  2006/11/07 08:34:57  strauman
		 - added ARP routines to header and added descriptions
		 - added ARP cache scavenger, dump and flush routines
		 - added optional refreshing of cache entries whenever
		   a packet to a UDP socket (or an ICMP echo req.) arrives.
		 - added more ARP debugging and did some testing
		
		Revision 1.15  2006-11-05 02:12:40  till
		 - added missing headers

		Revision 1.14  2006-11-05 01:06:52  strauman
		 - provide configuration (compile-time switch) for boards with and w/o
		   byte-swapped byte lanes [network-only devel. board is not byte swapped].
		   To the user who access the fifo only via the FifoWr / FifoRd routines
		   the configuration is transparent.

		Revision 1.13  2006/11/04 22:57:02  strauman
		 - swap bytes in buffer if defined(BYTES_NOT_SWAPPED)
		
		Revision 1.12  2006/10/27 17:46:25  strauman
		 - enable 'store-and-forward' mode to avoid TX FIFO underflow under all
		   conditions.
		 - TX routine now checks FIFO fill status and drops packet if necessary
		
		Revision 1.11  2006/10/25 18:43:56  strauman
		 - don't include DMA header for now
		
		Revision 1.10  2006/10/12 03:14:19  strauman
		 - adopted some of the coding standards.
		
 
=============================================================================*/


/* This driver was written for the uC5282 BSP and BSP-specifica
 * have not been separated out [yet].
 */

/* Look for compile-time configurable parameters after the 
 * include file section...
 */

/* #define HW_BYTES_NOT_SWAPPED */

/* ENDIANNESS NOTES
 * ----------------
 *
 * The lan9118 is internally (always, i.e., regardless of the ENDIAN register)
 * **little-endian**. There is a 'ENDIAN' register but it does not really swap
 * endianness. All it does is helping adjust the A1 line if a 16-bit data port
 * is used.
 * Note that all transfers are always 32-bit.
 * a) If a 32-bit data port exists and if the byte lanes between a big-endian
 *    CPU and the 9118 are wired straight through:
 *      lan9118
 *      (MSB)  B3 B2 B1 B0 (LSB)
 *      BE CPU
 *    then both chips will interpret a number 0x10000000 written to the 32-bit
 *    port as being 256M.
 * b) If a 16-bit data port exists then a 32-bit write has to be broken into
 *    two 16-bit writes. Here's where the difference in endianness comes into
 *    play: The BE associates the most-significant half-word with the lower
 *    address (A1 == 0), the 9118 with the higher (A1==1) one. Hence if a
 *    BE CPU writes 0x10000000 this would be broken into 
 *        1st write cycle  (A1 = 0)  0x1000
 *        2nd write cycle  (A1 = 1)  0x0000
 *    and the 9118 reading from the 16-bit port reassembles the 32-bit word to
 *        1st read cycle   (A1 = 0) goes into least significant half-word
 *        2nd read cycle   (A1 = 1) goes into most significant half-word
 *    ==> 0x00001000 is what the 9118 gets.
 *    
 *    All the 'ENDIAN' register does (when set to 0xffffffff) is inverting
 *    the A1 line (unused in 32-bit data port mode) so that the 1st read cycle
 *    of the example is associated with /A1 == 1, i.e., the most-significant
 *    half-word and vice versa, i.e., the 9118 correctly reads 0x10000000 as
 *    it would if a 32-bit port was used.
 *
 * This would indicate that the BE CPU and the 9118 work together seamlessly
 * regardless of endianness and data port width (provided that a BE CPU using
 * a 16-bit data port sets ENDIAN=0xffffffff).
 *
 * CAVEAT CAVEAT CAVEAT:
 * Even though register values now look fine, the 9118 IS STILL A LE CHIP.
 * In particular, the first byte it sends on the wire during transmission
 * is the one a LE system associates with the lowest address, i.e., the
 * LSB of the first 32-bit word in the FIFO.
 *
 * However, a BE CPU writing a block of memory to the FIFO sticks the byte
 * at the lowest address of the TX buffer into the MSB of the first word
 * which is the 4th byte send by the 9118!
 * 
 * ===> On a BE system THE ENTIRE TX/RX BUFFERS NEED TO BE BYTE SWAPPED
 *      (in 32-bit chunks).
 *      Since this is a potentially expensive operation, it seems better
 *      to swap byte lanes in hardware (interconnection of the lan9118 with
 *      the bus). This means that the register contents are now swapped also
 *      and need to be swapped again (in software) when the driver accesses
 *      registers.
 *
 * THIS DRIVER ASSUMES THAT BYTE LANES CONNECTING THE 9118 TO A BIG-ENDIAN
 * SYSTEM ARE SWAPPED IN HARDWARE.
 *
 * The 'ENDIAN' register is unused in this scenario and the port-width doesn't
 * matter.
 */

#include <rtems.h>

/* I'm not a fan of those macros...
 * Note that e.g., MCF5282_EPDR_EPD(bit) doesn't protect 'bit' in the
 * expansion -- this tells me that whoever wrote those was maybe a novice...
 */
#include <mcf5282/mcf5282.h>

#include <bsp.h>

#include <rtems/rtems/cache.h>
#include <rtems/bspIo.h>
#include <rtems/error.h>

#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sockio.h>
#include <net/if.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include <errno.h>
#include <time.h>

#include <dev/mii/mii.h>
#include <machine/in_cksum.h>

#define __KERNEL__
#include <rtems/rtems_mii_ioctl.h>
#undef __KERNEL__

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <machine/endian.h>

#if 0
#include "drv5282DMA.h"
#endif

#include "drvLan9118.h"

#include <lanIpProto.h>

#include "hwtmr.h"

/* Get environment var from the flash; implemented by GeSys/uC5282 */
extern const char *getbenv(const char *);

/****** COMPILE-TIME CONFIGURATION PARAMETERS ****************/

/****** BOARD-SPECIFIC CONFIGURATION *************************/

/* To which EPORT pin is the lan9118's interrupt wire hooked */
#define LAN9118_PIN	(4)

/* To which GPIO port TD pin (NOTE: this is arcturus port PTC!!)
 * is the reset line wired. Leave undefined if no connection exists.
 */
#define LAN9118_RESET_TD_PIN	(0)

/* Which vector is the ISR connected to                      */
#define LAN9118_VECTOR	(64+LAN9118_PIN)

/* Enable debugging messages; this also makes more symbols globally visible */
#define DEBUG_IP	(1<<0)
#define DEBUG_IRQ	(1<<1)
#define DEBUG_TXSTS (1<<2)

/* If DEBUG is undefined all code is removed.
 *
 * If DEBUG is defined code is compiled in
 * and the drvLan9118Debug variable is initialized
 * to DEBUG.
 *
 * ==> in order to have debugging but let the
 *     driver be quiet define DEBUG to 0.
 *     Debug messages can then be enabled at
 *     run-time.
 */
#define DEBUG		DEBUG_TXSTS


/* Which RTEMS events to use to signal the driver task that service is needed */ 
#define KILL_EVENT	RTEMS_EVENT_0	/* terminate task */
#define IRQ_EVENT	RTEMS_EVENT_1	/* interrupt happened, needs service */

	/* Even though we can switch endianness for register access
	 * this doesn't change the way the chip puts data out on the
	 * wire. The first byte that makes it out is always going to
	 * be the LSB in little endian, as seen by lan9118, i.e., the
	 * byte lane that ships bits 0..8.
	 */ 

/* How deep should the TX request queue be (maximum) */
#define TXQ_DEPTH	4

/* Default priority and stack of the driver task (can be overridden by run-time
 * args to drvLan9118Start()
 */
#define DEFLT_PRIO	30
#define DEFLT_STACK	4000


#define __GET_BITS(x,s,m)	(((x)>>s)&m)
#define __SET_BITS(x,s,m)	(((x)&m)<<s)

#ifdef DEBUG
#define STATIC
int drvLan9118Debug = DEBUG;
#else
#define STATIC static
#endif

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
#define E2P_CMD_EPC_CMD_GET(x)		__GET_BITS(x,28,7)
#define E2P_CMD_EPC_CMD_SET(x)		__SET_BITS(x,28,7)
/* Read Selected Location         */
#define E2P_CMD_READ				E2P_CMD_EPC_CMD_SET(0)
/* Erase/Write Disable            */
#define E2P_CMD_EWDS				E2P_CMD_EPC_CMD_SET(1)
/* Erase/Write Enable             */
#define E2P_CMD_EWEN				E2P_CMD_EPC_CMD_SET(2)
/* Write Selected Location        */
#define E2P_CMD_WRITE				E2P_CMD_EPC_CMD_SET(3)
/* Write Data to All Locations    */
#define E2P_CMD_WRAL				E2P_CMD_EPC_CMD_SET(4)
/* Erase Selected Location        */
#define E2P_CMD_ERASE				E2P_CMD_EPC_CMD_SET(5)
/* Bulk Erase (Erase ALL)         */
#define E2P_CMD_ERAL				E2P_CMD_EPC_CMD_SET(6)
/* Reload MAC address from EEPROM */
#define E2P_CMD_RELOAD				E2P_CMD_EPC_CMD_SET(7)
#define E2P_CMD_EPC_TIMEOUT	(1<<9)
#define E2P_CMD_MAC_LOADED	(1<<8)
#define E2P_CMD_EPC_ADDR_GET(x)		__GET_BITS(x, 0,0xff)
#define E2P_CMD_EPC_ADDR_SET(x)		__SET_BITS(x, 0,0xff)
	E2P_DATA	= 0xb4,
	FIFO_ALIAS	= 0x0800
} DrvLan9118RegOff_t;

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
} DrvLan9118MacCsrRegOff_t;

typedef enum {
	MII_INT_SRC = 29,
	MII_INT_EN  = 30,
#define PHY_ENERGYON_INT	(1<<7)
#define PHY_ANEGCOMP_INT	(1<<6)
#define PHY_REMFAULT_INT	(1<<5)
#define PHY_LINKDOWN_INT	(1<<4)
#define PHY_ANEGLACK_INT	(1<<3)
#define PHY_PARDTFLT_INT	(1<<2)
#define PHY_ANEGPGRX_INT	(1<<1)
} DrvLan9118PhyRegOff_t;

struct DrvLan9118_ts_;

#define REGLOCK(plan_ps)	assert( !rtems_semaphore_obtain((plan_ps)->mutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT))
#define REGUNLOCK(plan_ps) 	assert( !rtems_semaphore_release((plan_ps)->mutx))

#define TXLOCK(plan_ps)		assert( !rtems_semaphore_obtain((plan_ps)->tmutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT))
#define TXUNLOCK(plan_ps)	assert( !rtems_semaphore_release((plan_ps)->tmutx) )

#define NUM_MC_HASHES		64

typedef struct DrvLan9118_ts_ {
	uint32_t			base;
	uint32_t			int_msk;
	uint32_t			phy_int_msk;
	rtems_id			mutx;
	rtems_id			tmutx;
	rtems_id			tid;
	rtems_id			txq;
	rtems_id            sync;
	DrvLan9118CB_tpf	rx_cb_pf;
	void				*rx_cb_arg_p;
	DrvLan9118CB_tpf	tx_cb_pf;
	void				*tx_cb_arg_p;
	DrvLan9118CB_tpf	err_cb_pf;
	void				*err_cb_arg_p;
	DrvLan9118CB_tpf	phy_cb_pf;
	void				*phy_cb_arg_p;
	EthHeaderRec		ebcst_s;
	struct {
		uint32_t	rxu;
		uint32_t	rxm;
		uint32_t	rxb;
		uint32_t	txp;
		uint32_t	rwt;
		uint32_t	rxe;
		uint32_t	txe;
		uint32_t	tdfu;
		uint32_t	tdfo;
		uint32_t	rxdf;
		uint32_t	rsff;
		uint32_t	tsff;
		uint32_t	filf;
		uint32_t	lerr;
		uint32_t	mii;
		uint32_t	runt;
		uint32_t	tool;
		uint32_t	lcol;
		uint32_t	csum;
	} 				stats_s;
	uint16_t			mc_refcnt[NUM_MC_HASHES];
} DrvLan9118_ts;

/* Forward and extern decl. */
void drvLan9118Daemon(rtems_task_argument);


/* TX buffer command words */

#define TXCMD_A_IRQ				(1<<31)
#define TXCMD_A_END_ALIGN_4		(0<<24)
#define TXCMD_A_END_ALIGN_16	(1<<24)
#define TXCMD_A_END_ALIGN_32	(2<<24)
#define TXCMD_A_END_ALIGN_MSK	(3<<24)
#define TXCMD_A_START_ALIGN_GET(x)	__GET_BITS(x,16,0x1f)
#define TXCMD_A_START_ALIGN_SET(x)	__SET_BITS(x,16,0x1f)
#define TXCMD_A_FIRST			(1<<13)
#define TXCMD_A_LAST			(1<<12)
#define TXCMD_A_BUFSIZ_GET(x)	__GET_BITS(x,0,0x7ff)
#define TXCMD_A_BUFSIZ_SET(x)	__SET_BITS(x,0,0x7ff)

#define TXCMD_B_TAG_GET(x)		__GET_BITS(x,16,0xffff)
#define TXCMD_B_TAG_SET(x)		__SET_BITS(x,16,0xffff)
#define TXCMD_B_FCSGEN_DISABLE	(1<<13)
#define TXCMD_B_PAD_DISABLE		(1<<12)
#define TXCMD_B_PKTLEN_GET(x)	__GET_BITS(x,0,0x7ff)
#define TXCMD_B_PKTLEN_SET(x)	__SET_BITS(x,0,0x7ff)


/* IRQ configuration bits we set */
#define IRQ_CFG_BITS IRQ_CFG_IRQ_PUSHPULL

DrvLan9118_ts	theLan9118_s;

static inline uint32_t byterev(uint32_t x)
{
	asm volatile("byterev %0":"+d"(x));
	return x;
}

#define EEPROM_SIZE 256

static uint8_t eeprom_shadow[EEPROM_SIZE];


#ifndef BYTE_ORDER
#error "unknown CPU endianness"
#else
#if BYTE_ORDER == BIG_ENDIAN

#ifdef __GNUC__

typedef uint32_t CopyItem_u __attribute__((__may_alias__));
#define UINTOF(item) (item)

#else
/* Introduce union containing a char for sake of ISOC99 aliasing rule */
typedef union {
	char		c;
	uint32_t	u;
} CopyItem_u;
#define UINTOF(item) (item).u
#endif

#ifdef HW_BYTES_NOT_SWAPPED
/* Bigendian CPU with wires going straight to the chip (which is little endian).
 * We can access registers w/o byte swapping (using the 'endian' register to
 * fix the 2 16-bit accesses).
 * HOWEVER: when sending data the chip (being) LE assumes the *LSB* (which is
 *          associated with the 1st byte address) should go out on the wire first.
 *          => we must byte-swap all words that we read/write from/to the data
 *          fifos.
 */
#define BYTEREV_REG(x) (x)
#define BYTEREV_BUF(x) (CopyItem_u)byterev(UINTOF(x))
#else
/* We must byte-swap register accesses but buffers are now correct */
#define BYTEREV_REG(x) byterev(x)
#define BYTEREV_BUF(x) (x)
#endif

#else  /* if BYTE_ORDER == BIG_ENDIAN */
/* we are on a little-endian machine */

#ifdef HW_BYTES_NOT_SWAPPED
#define BYTEREV_REG(x)	(x)
#define BYTEREV_BUF(x)	(x)
#else
#define BYTEREV_REG(x)	(x)
#define BYTEREV_BUF(x)	(x)
#endif
#endif /* if BYTE_ORDER == BIG_ENDIAN */
#endif /* ifdef BYTE_ORDER */

/* Hmm __IPSBAR is defined in the linker script :-( */

#ifndef __IPSBAR
#define __IPSBAR ((volatile uint8_t *)0x40000000)
#endif
          
/* Seems to be missing */
#ifndef MCF5282_DMA_DCR_DSIZE_LINE
#define MCF5282_DMA_DCR_DSIZE_LINE                      (0x00030000)
#endif

#include <stdint.h>

/* Configuration Parameters */

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
#define rd9118Reg(base, off)		BYTEREV_REG(*(volatile uint32_t *)((uint32_t)(base) + off))
#define wr9118Reg(base, off, val)	do { *(volatile uint32_t*)((uint32_t)(base) + off) = BYTEREV_REG(val); } while (0)

static inline uint32_t	rd9118RegSlow(uint32_t base, DrvLan9118RegOff_t off)
{
	/* always do a worst case delay for now; the read cycle itself takes another ~160ns */
	DELAY180ns();
	return rd9118Reg(base, off);
}

static void macCsrAccess(uint32_t base, uint32_t rNw, int addr)
{
uint32_t v = MAC_CSR_CMD_BUSY | MAC_CSR_CMD_ADDR_SET(addr) | rNw;
	wr9118Reg(base, MAC_CSR_CMD, v);
	DELAY45ns();
	while ( MAC_CSR_CMD_BUSY & rd9118Reg(base, MAC_CSR_CMD) )
		/* poll */;
}

STATIC uint32_t
macCsrRead(DrvLan9118_tps plan_ps, uint32_t reg)
{
uint32_t base = plan_ps->base;
	macCsrAccess(base, MAC_CSR_CMD_RnW, reg);
	return rd9118Reg(base, MAC_CSR_DATA);
}

STATIC void
macCsrWrite(DrvLan9118_tps plan_ps, uint32_t reg, uint32_t v)
{
uint32_t base = plan_ps->base;
	wr9118Reg(base, MAC_CSR_DATA, v);
	macCsrAccess(base, 0, reg);
}

static void miiAccess(DrvLan9118_tps plan_ps, uint32_t wNr, uint32_t addr)
{
uint32_t v = wNr | MII_ACC_PHY_SET(0x01) | MII_ACC_MIIRIND_SET(addr) | MII_ACC_MIIBSY;
	macCsrWrite(plan_ps, MII_ACC, v);
	while ( MII_ACC_MIIBSY & macCsrRead(plan_ps, MII_ACC) )
		/* poll */;
}

void
drvLan9118FifoRd(DrvLan9118_tps plan_ps, void *buf_pa, int n_bytes)
{
register          CopyItem_u *ibuf_p = buf_pa;
register volatile CopyItem_u *src_p  = (void*)(plan_ps->base + FIFO_ALIAS);

	assert( ((uint32_t)buf_pa & 3) == 0 );
	assert( (n_bytes & 3)          == 0 );

	n_bytes >>= 2; /* convert into words */
	while (n_bytes >= 8) {
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		*ibuf_p++ = BYTEREV_BUF(*src_p);
		n_bytes -= 8;
	}
	while (n_bytes-- > 0) {
		*ibuf_p++ = BYTEREV_BUF(*src_p);
	}
}

void
drvLan9118FifoWr(DrvLan9118_tps plan_ps, const void *buf_pa, int n_bytes)
{
register const CopyItem_u    *ibuf_p = buf_pa;
register volatile CopyItem_u *dst_p  = (void*)(plan_ps->base + FIFO_ALIAS);

	assert( ((uint32_t)buf_pa & 3) == 0 );
	assert( (n_bytes & 3)       == 0 );

#if 1
	n_bytes >>= 2; /* convert into words */
	while (n_bytes >= 8) {
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		*dst_p = BYTEREV_BUF(*ibuf_p++);
		n_bytes -= 8;
	}
	while (n_bytes-- > 0) {
		*dst_p = BYTEREV_BUF(*ibuf_p++);
	}
#else
#if defined(HW_BYTES_NOT_SWAPPED) && BYTE_ORDER == BIG_ENDIAN
	drvLan9118BufRev(ibuf_p, n_bytes>>2);
#endif
	memcpy( (void*)(plan_ps->base + FIFO_ALIAS), buf, n_bytes);
#endif
}

volatile uint32_t *
drvLan9118FifoAddr(DrvLan9118_tps plan_ps, int *endianBig)
{
/* FIXME: I'm not sure the *endianBig value is computed correctly */
	if ( endianBig )
		*endianBig = 
#if defined(HW_BYTES_NOT_SWAPPED) && BYTE_ORDER == BIG_ENDIAN
		0
#else
		1
#endif
		;

	return (volatile uint32_t*)(plan_ps->base + FIFO_ALIAS);
}

/* busy wait for the EEPROM controller to be ready */
static inline uint32_t e2p_busywait(DrvLan9118_tps plan_ps)
{
uint32_t cmd;
	while ( E2P_CMD_BUSY & (cmd = rd9118Reg(plan_ps->base, E2P_CMD)) )
		/* busy wait -- this can take up to 30ms */ ;
	return cmd;
}

void
drvLan9118Lock(DrvLan9118_tps plan_ps)
{
	REGLOCK(plan_ps);
}

void
drvLan9118Unlock(DrvLan9118_tps plan_ps)
{
	REGUNLOCK(plan_ps);
}

/* MII access functions to support media ioctl */
STATIC int
drvLan9118_mdio_w(int phy, void *uarg_p, unsigned reg, uint32_t val)
{
DrvLan9118_ts *plan_ps = uarg_p;
	if ( phy > 1 )
		return -1;
	macCsrWrite(plan_ps, MII_DATA, val);
	miiAccess(plan_ps, MII_ACC_MIIWnR, reg);
	return 0;
}

STATIC int
drvLan9118_mdio_r(int phy, void *uarg_p, unsigned reg, uint32_t *val_p)
{
DrvLan9118_tps plan_ps = uarg_p;
	if ( phy > 1 )
		return -1;
	miiAccess(plan_ps, 0, reg);
	*val_p = macCsrRead(plan_ps, MII_DATA);
	return 0;
}

static struct rtems_mdio_info drvLan9118_mdio_s = {
	mdio_r: drvLan9118_mdio_r,
	mdio_w: drvLan9118_mdio_w,
	has_gmii: 0,
};

inline void
drvLan9118IrqEnable(void)
{
int level;
	rtems_interrupt_disable(level);	
	MCF5282_EPORT_EPIER |= (MCF5282_EPORT_EPIER_EPIE(LAN9118_PIN));
	rtems_interrupt_enable(level);	
}

inline void
drvLan9118IrqDisable(void)
{
int level;
	rtems_interrupt_disable(level);	
	MCF5282_EPORT_EPIER &= ~(MCF5282_EPORT_EPIER_EPIE(LAN9118_PIN));
	rtems_interrupt_enable(level);	
}

/* Hard reset via a dedicated wire from the arcturus board
 *
 * !!!!!!!
 * NOTE: clever arcturus designers *renamed* pins in a confusing
 *       way: what they call PTC is what the coldfire manual calls
 *       port TD !!
 * !!!!!!!
 *
 * Writing a low level (0) holds the device in reset.
 */
int
drvLan9118HardReset(int level)
{
#ifdef LAN9118_RESET_TD_PIN
static int lazy_init = 0;

	if ( level )
		MCF5282_GPIO_SETTD =  MCF5282_GPIO_SETx(LAN9118_RESET_TD_PIN);
	else
		MCF5282_GPIO_CLRTD = ~MCF5282_GPIO_CLRx(LAN9118_RESET_TD_PIN);

	/* Setup DDR and pin assignment after writing the desired
	 * output value in an attempt to avoid glitches...
	 */
	if ( !lazy_init ) {
		lazy_init = 1;
		/* Data direction: output */
		MCF5282_GPIO_DDRTD   = MCF5282_GPIO_DDRx(LAN9118_RESET_TD_PIN);
		/* Pin assignment: I/O    */
		MCF5282_GPIO_PTDPAR &= ~(3<<(2*LAN9118_RESET_TD_PIN));
	}

	/* return current value */
	return MCF5282_GPIO_PORTTDP & MCF5282_GPIO_PORTx(LAN9118_RESET_TD_PIN);
#else
	return -1;
#endif	
}

/* Verify that all byte lanes work */
int
drvLan9118SanityCheck(DrvLan9118_tps plan_ps)
{
int      rval = 0;
uint32_t z,o;
uint32_t oendian;

	/* verify that we can read signature from BYTE_TEST */
	o = rd9118Reg(plan_ps->base, BYTE_TEST);
	if ( 0x87654321 != o ) {
		fprintf(stderr,"drvLan9118: probing BYTE_TEST failed -- is there a chip? (read 0x%08lx -- expected 0x87654321)\n", o);
		rval = -1;
	}

	/* verify that we can write all zeros and all-ones to ENDIAN */

	oendian = rd9118Reg(plan_ps->base, ENDIAN);

	if ( oendian ) {
		/* chip would to byte-reversal for us (if register really
		 * contains all ones
		 */
		wr9118Reg(plan_ps->base, ENDIAN, 0);
		z = rd9118Reg(plan_ps->base, ENDIAN);
		if ( z != 0xffffffff ) {
			/* setting register to switch lanes failed */
			z = byterev(z);		
		}
		wr9118Reg(plan_ps->base, ENDIAN, 0xffffffff);
		o = rd9118Reg(plan_ps->base, ENDIAN);
		if ( o != 0xffffffff ) {
			/* setting register to switch lanes failed */
			o = byterev(o);
		}
	} else {
		/* chip does not do byte reversal */
		wr9118Reg(plan_ps->base, ENDIAN, 0xffffffff);
		o = rd9118Reg(plan_ps->base, ENDIAN);
		wr9118Reg(plan_ps->base, ENDIAN, 0);
		z = rd9118Reg(plan_ps->base, ENDIAN);
	}
	if ( z != 0 ) {
		fprintf(stderr,"drvLan9118: SANITY CHECK FAILURE -- writing 0x00000000 to  register failed; read back 0x%08lx\n",z);
		rval = -1;
	}
	if ( o != 0xffffffff ) {
		fprintf(stderr,"drvLan9118: SANITY CHECK FAILURE -- writing 0xffffffff to  register failed; read back 0x%08lx\n",o);
		rval = -1;
	}

	return rval;
}


STATIC int
drvLan9118ResetChip(DrvLan9118_tps plan_ps)
{
uint32_t tmp;
uint32_t base = plan_ps->base;

	/* make sure interrupts are masked */
	drvLan9118IrqDisable();

	/* soft reset; first the PHY, then the chip */
	drvLan9118_mdio_w(0, plan_ps, MII_BMCR, BMCR_RESET);
	do {
		drvLan9118_mdio_r(0, plan_ps, MII_BMCR, &tmp);
	} while ( BMCR_RESET & tmp );

	/* chip soft reset; endianness is unaffected */
	wr9118Reg(base, HW_CFG, HW_CFG_SRST);
	DELAY45ns();
	do { 
		tmp = rd9118Reg(base, HW_CFG);
		if ( HW_CFG_SRST_TO & tmp ) {
			fprintf(stderr,"ERROR: Soft Reset Timeout\n");
			return -1;
		}
	} while (HW_CFG_SRST & tmp);

	/* wait until done loading mac-address from EEPROM */
	e2p_busywait(plan_ps);

	return 0;
}

static void
lan9118isr(void *uarg, rtems_vector_number v )
{
	drvLan9118IrqDisable();
	rtems_event_send(theLan9118_s.tid, IRQ_EVENT);
#ifdef DEBUG
	if ( drvLan9118Debug & DEBUG_IRQ )
		printk("LAN ISR\n");
#endif
}

#define LAN9118_FLAG_BCDIS	1

void
drvLan9118ReadEnaddr(DrvLan9118_tps plan_ps, uint8_t *buf_pa)
{
int      i;
uint32_t tmp;

	REGLOCK(plan_ps);
	tmp = macCsrRead(plan_ps, ADDRL);
	for ( i=0; i<4; i++, tmp>>=8 )
		*buf_pa++ = tmp & 0xff;
	tmp = macCsrRead(plan_ps, ADDRH);
	REGUNLOCK(plan_ps);
	*buf_pa++ = tmp & 0xff;
	*buf_pa++ = (tmp>>8) & 0xff;
}

STATIC void
drvLan9118_setup_uc5282(void)
{
unsigned long key;

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


	/* use 2 wait states, internal termination; burst reads end up requiring even more wait states
	 * [note that write-cycles would only need one!] :-(
	 */
	MCF5282_CS2_CSCR = MCF5282_CS_CSCR_WS(2) | MCF5282_CS_CSCR_AA | MCF5282_CS_CSCR_PS_16;

	MCF5282_CS2_CSMR |= MCF5282_CS_CSMR_V;

	/* EPORT & GPIO setup */

	/* make pin LAN9118_PIN active low, level triggered */
	MCF5282_EPORT_EPPAR &= ~(MCF5282_EPORT_EPPAR_EPPA1_BOTHEDGE<<(2*(LAN9118_PIN-1)));
	MCF5282_EPORT_EPPAR |=  (MCF5282_EPORT_EPPAR_EPPA1_LEVEL<<(2*(LAN9118_PIN-1)));

	MCF5282_EPORT_EPDDR &= ~MCF5282_EPORT_EPDDR_EPDD(LAN9118_PIN);

	/* !!! DISABLE BUFFERED WRITES !!!
	 *
	 * I found that (depending on the exact sequence of assembly instructions) the
	 * external bus interface repeats write-cycles. This doesn't seem to occur if BWE is off.
	 * ==> We change the BSP's default setting (affects all accesses not matched in an ACR).
	 * since all external accesses are affected this is what we want anyways (DRAM access is
	 * through ACR0 as per the BSP settings).
	 */
	{
	/* CACR is write-only [broken design]; BSP caches it though */
	extern uint32_t mcf5282_cacr_mode;
	rtems_interrupt_disable(key);
		mcf5282_cacr_mode &= ~ MCF5XXX_CACR_DBWE;
		asm volatile("	movec %0, %%cacr"::"d"(mcf5282_cacr_mode));
	rtems_interrupt_enable(key);
	}
}

DrvLan9118_tps
drvLan9118Setup(const uint8_t *enaddr_pa, uint32_t flags)
{
DrvLan9118_tps    plan_ps;
uint32_t          tmp;
int               i;
unsigned char     buf_a[6];
unsigned short    sbuf_a[6];
rtems_status_code sc;

	theLan9118_s.base = LAN_9118_BASE;
	plan_ps           = &theLan9118_s;

	/* make sure interrupts are masked */
	drvLan9118IrqDisable();

	/* setup BSP specific glue stuff   */
	drvLan9118_setup_uc5282();

	/* Initialize the 9118 */
	memset(&plan_ps->stats_s, 0, sizeof(plan_ps->stats_s));

	/* First, we must perform a read access to the BYTE TEST register */
	tmp = rd9118Reg(plan_ps->base, BYTE_TEST);

#ifdef HW_BYTES_NOT_SWAPPED
	if ( 0x21436587 == tmp ) {
		fprintf(stderr,"ERROR: Seems this board has swapped byte lanes but the driver was compiled otherwise\n");
		return 0;
	}
#else
	if ( 0x65872143 == tmp ) {
		fprintf(stderr,"ERROR: Seems this board has NON-swapped byte lanes but the driver was compiled otherwise\n");
		return 0;
	}
#endif
	
#if BYTE_ORDER == BIG_ENDIAN
#ifdef HW_BYTES_NOT_SWAPPED
	/* Setup for big endian mode */
	if ( 0x87654321 != tmp )
		wr9118Reg(plan_ps->base, ENDIAN, 0xffffffff);
#endif
#else
#ifndef HW_BYTES_NOT_SWAPPED
#error "Setup of ENDIAN register not implemented for byte-swapped connection to a little-endian CPU"
#endif
#endif

	if ( drvLan9118SanityCheck(plan_ps) )
		return 0;

	if ( drvLan9118ResetChip(plan_ps) )
		return 0;

	if ( !enaddr_pa ) {
		const char *p = getbenv("HWADDR1");
		if ( !p || 6 != sscanf(p,"%2hx:%2hx:%2hx:%2hx:%2hx:%2hx",sbuf_a,sbuf_a+1,sbuf_a+2,sbuf_a+3,sbuf_a+4,sbuf_a+5) ) {
			if ( ! (rd9118Reg(plan_ps->base, E2P_CMD) & E2P_CMD_MAC_LOADED) ) {
				fprintf(stderr,"Need ethernet address (6 bytes) argument\n");
				return 0;
			}
		} else {
			for ( i=0; i<6; i++ )
				buf_a[i] = sbuf_a[i];
			enaddr_pa  = buf_a;
		}
	}

	if ( !plan_ps->mutx ) {
#define ERR_INTS ( RWT_INT | RXE_INT | TXE_INT | TDFU_INT | TDFO_INT | RXDF_INT | RSFF_INT)

		plan_ps->int_msk     = ERR_INTS | TSFL_INT | RSFL_INT | PHY_INT;
		plan_ps->phy_int_msk = 0;

		sc = rtems_semaphore_create(
			rtems_build_name('l','a','n','d'), 
			1,
			RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&plan_ps->mutx);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc, "drvLan9118: unable to create mutex\n");
			return 0;
		}

		sc = rtems_semaphore_create(
			rtems_build_name('l','a','n','t'), 
			1,
			RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY,
			0,
			&plan_ps->tmutx);
		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_semaphore_delete( plan_ps->mutx );
			plan_ps->mutx = 0;
			rtems_error(sc, "drvLan9118: unable to create TX mutex\n");
			return 0;
		}

		plan_ps->rx_cb_pf		= 0;
		plan_ps->rx_cb_arg_p	= 0;
		plan_ps->tx_cb_pf		= 0;
		plan_ps->tx_cb_arg_p	= 0;
		plan_ps->err_cb_pf		= 0;
		plan_ps->err_cb_arg_p	= 0;
		plan_ps->phy_cb_pf		= 0;
		plan_ps->phy_cb_arg_p	= 0;

		BSP_installVME_isr( LAN9118_VECTOR, lan9118isr, 0 );

		/* configure IRQ output as push-pull, enable interrupts */
		wr9118Reg(plan_ps->base, IRQ_CFG, IRQ_CFG_BITS | IRQ_CFG_IRQ_EN);
		wr9118Reg(plan_ps->base, INT_EN,  plan_ps->int_msk);

		drvLan9118IrqEnable();
	}


	/* Setup the convenience header   */
	if ( !enaddr_pa ) {
		drvLan9118ReadEnaddr(plan_ps, plan_ps->ebcst_s.src);
	} else {
		memcpy( plan_ps->ebcst_s.src, enaddr_pa, 6);
	}
	memset( plan_ps->ebcst_s.dst, 0xff,   6);


	/* Enable LEDs */
	wr9118Reg(plan_ps->base, GPIO_CFG, GPIO_CFG_LED3_EN | GPIO_CFG_LED2_EN | GPIO_CFG_LED1_EN);

	/* start transmitter and configure to allow status overruns */	
	wr9118Reg(plan_ps->base, TX_CFG, TX_CFG_TXSAO | TX_CFG_TX_ON);

	/* set 2-byte offset in RX so that ethernet headers are word aligned */
	wr9118Reg(plan_ps->base, RX_CFG, rd9118Reg(plan_ps->base, RX_CFG) | RX_CFG_RXDOFF_SET(2));

	/* for now, we are conservative and use 'store and forward' mode
	 * to avoid TX underflow.
	 */
	wr9118Reg(plan_ps->base, HW_CFG, rd9118Reg(plan_ps->base, HW_CFG) | HW_CFG_SF);

	/* Setup the MAC but don't start the receiver */
	macCsrWrite(plan_ps, MAC_CR, MAC_CR_FDPX | MAC_CR_TXEN | MAC_CR_HPFILT | ((flags & LAN9118_FLAG_BCDIS) ? MAC_CR_BCAST : 0));

	if ( enaddr_pa ) {
		tmp = (enaddr_pa[5]<<8) | enaddr_pa[4];
		macCsrWrite(plan_ps, ADDRH, tmp);
		for (i=3,tmp=0; i>=0; i--) {
			tmp = (tmp<<8) | enaddr_pa[i];
		}
		macCsrWrite(plan_ps, ADDRL, tmp);
	}
	drvLan9118McFilterClear(plan_ps);

	/* Setup the PHY */

	/* advertise 100baseTx including full duplex */
	drvLan9118_mdio_w(0, plan_ps, MII_ANAR, ANAR_TX_FD | ANAR_TX | ANAR_CSMA);

	/* enable & start autonegotiation (also select 100 FD as a hint in case autoneg is switched off) */
	drvLan9118_mdio_w(0, plan_ps, MII_BMCR, BMCR_SPEED0 | BMCR_AUTOEN | BMCR_FDX | BMCR_STARTNEG);

	return plan_ps;
}


int
drvLan9118Start(DrvLan9118_tps plan_ps,
				uint32_t prio, uint32_t stack,
				DrvLan9118CB_tpf rx_cb_pf, 	void *rx_cb_arg_p,
				DrvLan9118CB_tpf tx_cb_pf, 	void *tx_cb_arg_p,
				DrvLan9118CB_tpf err_cb_pf,	void *err_cb_arg_p,
				DrvLan9118CB_tpf phy_cb_pf,	void *phy_cb_arg_p)
{
rtems_id			tid;
rtems_status_code 	sc;

	sc = rtems_task_create(
					rtems_build_name('9','1','1','8'),
					prio  ? prio  : DEFLT_PRIO,
					stack && stack > DEFLT_STACK ? stack : DEFLT_STACK,
					RTEMS_FLOATING_POINT | RTEMS_DEFAULT_ATTRIBUTES,
					RTEMS_DEFAULT_MODES,
					&tid);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc,"drvLan9118; unable to create task\n");
		return -1;
	}

	plan_ps->rx_cb_pf		= rx_cb_pf;
	plan_ps->rx_cb_arg_p	= rx_cb_arg_p;
	plan_ps->tx_cb_pf		= tx_cb_pf;
	plan_ps->tx_cb_arg_p	= tx_cb_arg_p;
	plan_ps->err_cb_pf		= err_cb_pf;
	plan_ps->err_cb_arg_p	= err_cb_arg_p;
	plan_ps->phy_cb_pf		= phy_cb_pf;
	plan_ps->phy_cb_arg_p	= phy_cb_arg_p;

	if ( !tx_cb_pf ) {
		sc = rtems_message_queue_create(
								rtems_build_name('9','1','1','8'), 
								TXQ_DEPTH,
								sizeof(uint32_t),
								RTEMS_DEFAULT_ATTRIBUTES,
								&plan_ps->txq);
		if ( RTEMS_SUCCESSFUL != sc ) {
				rtems_error(sc, "drvLan9118: unable to create message queue\n");
				return -1;
		}
	}

	/* Copy EEPROM contents into shadow buffer   */
	drvLan9118E2PRead(plan_ps, eeprom_shadow, 0, EEPROM_SIZE);

	/* lock 'tid' against EEPROM access routines */
	REGLOCK(plan_ps);
	plan_ps->tid = tid;
	REGUNLOCK(plan_ps);
	
	sc = rtems_task_start(tid, drvLan9118Daemon, (rtems_task_argument)plan_ps);
	if ( RTEMS_SUCCESSFUL != sc ) {
		rtems_error(sc,"drvLan9118: unable to start task\n");
		rtems_task_delete(tid);
		plan_ps->tid = 0;
		return -1;
	}
	return 0;
}
		


/* single-threaded access to CSRs once the thread is killed */
void
drvLan9118Shutdown(DrvLan9118_tps plan_ps)
{
rtems_status_code sc;

	if ( !plan_ps )
		return;

	drvLan9118IrqDisable();

	if ( plan_ps->tid ) {

		sc = rtems_semaphore_create(
				rtems_build_name('l', 'a', 'n', 's'),
				0,
				RTEMS_SIMPLE_BINARY_SEMAPHORE,
				0,
				& plan_ps->sync
				);

		if ( RTEMS_SUCCESSFUL != sc ) {
			rtems_error(sc, "drvLan9118: unable to create sync semaphore; not synchronizing termination\n");
		}

		rtems_event_send(plan_ps->tid, KILL_EVENT);

		if ( plan_ps->sync ) {

			sc = rtems_semaphore_obtain(plan_ps->sync, RTEMS_WAIT, 20);

			if ( RTEMS_SUCCESSFUL == sc ) {
				rtems_task_delete(plan_ps->tid);
			} else {
				rtems_error(sc, "drvLan9118: unable to synchronize with driver task; may leak resources!\n");
			}
		} else {
			rtems_task_wake_after(20);
		}

		plan_ps->tid            = 0;

	}

	plan_ps->rx_cb_pf		= 0;
	plan_ps->rx_cb_arg_p	= 0;
	plan_ps->tx_cb_pf		= 0;
	plan_ps->tx_cb_arg_p	= 0;
	plan_ps->err_cb_pf 		= 0;
	plan_ps->err_cb_arg_p	= 0;
	plan_ps->phy_cb_pf 		= 0;
	plan_ps->phy_cb_arg_p	= 0;

	drvLan9118ResetChip(plan_ps);

	rtems_semaphore_delete(plan_ps->tmutx);
	rtems_semaphore_delete(plan_ps->mutx);
	plan_ps->mutx = 0;
	if ( plan_ps->txq )
		rtems_message_queue_delete(plan_ps->txq);
	plan_ps->txq  = 0;
	BSP_removeVME_isr(LAN9118_VECTOR, lan9118isr, 0);
}

int
drvLan9118DumpStats(DrvLan9118_tps plan_ps, FILE *f_p)
{
uint8_t ena[6];
	if ( !f_p )
		f_p = stdout;
	drvLan9118ReadEnaddr(plan_ps, ena);
	fprintf(f_p,"DrvLan9118_tps interface [%02X:%02X:%02X:%02X:%02X:%02X] statistics:\n",
		ena[0], ena[1], ena[2], ena[3], ena[4], ena[5]);
	fprintf(f_p,"  Unicast   recvd.: %lu\n", plan_ps->stats_s.rxu);
	fprintf(f_p,"  Broadcast recvd.: %lu\n", plan_ps->stats_s.rxb);
	fprintf(f_p,"  Multicast recvd.: %lu\n", plan_ps->stats_s.rxm);
	fprintf(f_p,"  Packets sent    : %lu\n", plan_ps->stats_s.txp);
	fprintf(f_p,"  Oversized packets received (RX watchdog timout): %lu\n", plan_ps->stats_s.rwt);
	fprintf(f_p,"  Receiver errors : %lu\n", plan_ps->stats_s.rxe);
	fprintf(f_p,"  RX dropped frms : %lu\n", plan_ps->stats_s.rxdf);
	fprintf(f_p,"  RX stat fifo overflow : %lu\n", plan_ps->stats_s.rsff);
	fprintf(f_p,"  RX filter failed: %lu\n", plan_ps->stats_s.filf);
	fprintf(f_p,"  RX length errors: %lu\n", plan_ps->stats_s.lerr);
	fprintf(f_p,"  RX runt frames  : %lu\n", plan_ps->stats_s.runt);
	fprintf(f_p,"  RX too long frms: %lu\n", plan_ps->stats_s.tool);
	fprintf(f_p,"  RX chksum errors: %lu\n", plan_ps->stats_s.csum);
	fprintf(f_p,"  late collisions : %lu\n", plan_ps->stats_s.lcol);
	fprintf(f_p,"  Transmit errors : %lu\n", plan_ps->stats_s.txe);
	fprintf(f_p,"  TX data fifo underflow: %lu\n", plan_ps->stats_s.tdfu);
	fprintf(f_p,"  TX data fifo overflow : %lu\n", plan_ps->stats_s.tdfo);
	fprintf(f_p,"  TX stat fifo overflow : %lu\n", plan_ps->stats_s.tsff);
	fprintf(f_p,"  MII errors      : %lu\n", plan_ps->stats_s.mii);
	return 0;
}

/* just the SIOCSIFMEDIA/SIOCGIFMEDIA ioctls */
int
drvLan9118ioctl(DrvLan9118_tps plan_ps, int cmd, int *media_p)
{
int rval;

	/* accept simple aliases for easy shell access */
	switch ( cmd ) {
		case 0 : cmd = SIOCGIFMEDIA; break;
		case 1 : cmd = SIOCSIFMEDIA; break;
		default: break;
	}
	REGLOCK(plan_ps);
	rval = rtems_mii_ioctl(&drvLan9118_mdio_s, plan_ps, cmd, media_p);
	REGUNLOCK(plan_ps);
	return rval;
}

#ifdef DEBUG
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
#endif

void
drvLan9118BufRev(uint32_t *data, int nwords)
{
register int i;
CopyItem_u *buf_pa = (CopyItem_u *)data;

	for (i=0; i<nwords; i++)
		buf_pa[i] = (CopyItem_u)byterev(UINTOF(buf_pa[i]));
}
 
/* To be called from TX thread ONLY */

/* NOTE: The head of the packet must be padded with 2 bytes, i.e.,
 *       the destination ethernet address starts at buf[2]
 *       The byte count includes the pad bytes but they are not
 *       sent out on the wire.
 *       Also, data are read out of the buffer in 32-bit words,
 *       i.e., even if nbytes is not a multiple of four the
 *       bytes up to the next word boundary are still read
 *       (but not sent).
 *       For sake of performance it is recommended to align
 *       buffers on a word boundary.
 *
 *       It is legal to provide a NULL buffer pointer. In this case,
 *       the routine merely puts the command words into the fifo
 *       and it is up to the user to write the actual data (again:
 *       2 pad-bytes at the beginning and the necessary padding
 *       at the end of the packet must be written to the chip!)
 *
 *       If a NULL buffer is used then the routine leaves the
 *       TX mutex locked and it must be unlocked by the user.
 */
uint32_t
drvLan9118TxPacket(DrvLan9118_tps plan_ps, const void *buf_pa, int nbytes, unsigned short tag)
{
uint32_t base = plan_ps->base;
uint32_t ltot;

	/* need 4 byte alignment (implicit TXCMD_A_END_ALIGN_4) */
	ltot = (nbytes+3) & ~3;

	TXLOCK(plan_ps);

	/* Verify space in the fifo */
	if ( TX_FIFO_INF_TXDFREE_GET(rd9118Reg(base, TX_FIFO_INF)) < ltot ) {
		/* not enough space */
		TXUNLOCK(plan_ps);
		return -1;
	}
	
	/* push the command words */
	wr9118Reg(base, TX_DATA_FIFO, TXCMD_A_FIRST | TXCMD_A_LAST | TXCMD_A_BUFSIZ_SET(nbytes - EH_PAD_BYTES) | TXCMD_A_START_ALIGN_SET(EH_PAD_BYTES));
	wr9118Reg(base, 32, TXCMD_B_TAG_SET(tag) | TXCMD_B_PKTLEN_SET(nbytes - EH_PAD_BYTES));

	if ( buf_pa ) {
#if 0
		/* DMA the packet */
		drv5282ioDMA((void*)(plan_ps->base + TX_DATA_FIFO), buf, ltot, 0, 0, 0);
#else
		/* cache flushing is slow; memcpy is faster */
		drvLan9118FifoWr(plan_ps, buf_pa, ltot);
#endif
		DELAY180ns();
		TXUNLOCK(plan_ps);
	}

	return 0;
}

void
drvLan9118TxUnlock(DrvLan9118_tps plan_ps)
{
	DELAY180ns();
	TXUNLOCK(plan_ps);
}

rtems_status_code
drvLan9118TxStatus(DrvLan9118_tps plan_ps, uint32_t *val_p, uint32_t timeout)
{
uint32_t rval;
uint32_t sz;

	if ( !val_p )
		val_p = &rval;

	if ( ! plan_ps->tid || plan_ps->tx_cb_pf )
		return -1;

	return rtems_message_queue_receive(plan_ps->txq,
					   val_p, &sz,
					   (timeout ? RTEMS_WAIT : RTEMS_NO_WAIT),
					   (uint32_t)-1 == timeout ? RTEMS_NO_TIMEOUT : timeout );
}

/* Flush TX status and DATA fifos   */
/* To be called from TX thread ONLY */
#define STATUS_FIFO	(1<<15)
#define DATA_FIFO	(1<<14)
uint32_t
drvLan9118TxFlush(DrvLan9118_tps plan_ps, int which)
{
uint32_t then;
uint32_t base = plan_ps->base;

	which &= TX_CFG_TXS_DUMP | TX_CFG_TXD_DUMP;
	if ( !which )
		return -1;
	then = Read_hwtimer();
	wr9118Reg(base, TX_CFG, which | rd9118Reg(base, TX_CFG));
	DELAY45ns();
	while ( which & rd9118Reg(base, TX_CFG) )
		/* poll */;
	return Read_hwtimer() - then;	
}

uint32_t
drvLan9118RxFlush(DrvLan9118_tps plan_ps)
{
uint32_t then = Read_hwtimer();
uint32_t base = plan_ps->base;

	wr9118Reg(base, RX_CFG, rd9118Reg(base, RX_CFG) | RX_CFG_RX_DUMP);
	DELAY45ns();
	while ( RX_CFG_RX_DUMP & rd9118Reg(base, RX_CFG) )
		/* poll */;
	return Read_hwtimer() - then;	
}

static uint32_t
skipPacket(DrvLan9118_tps plan_ps)
{
uint32_t then;
uint32_t base = plan_ps->base;

	then = Read_hwtimer();
	wr9118Reg(base, RX_DP_CTL, RX_DP_CTL_FFWD);
	DELAY45ns();
	while ( RX_DP_CTL_FFWD & rd9118Reg(base, RX_DP_CTL) )
		/* wait */;
	return Read_hwtimer() - then;
}

#ifdef DEBUG
volatile uint32_t maxFFDDelay = 0;
#endif
volatile uint32_t drvLan9118RxIntBase = 0;

void
drvLan9118Daemon(rtems_task_argument arg)
{
DrvLan9118_tps	plan_ps = (DrvLan9118_tps)arg;
uint32_t	    base    = plan_ps->base;
uint32_t	    int_sts, rx_sts, tx_sts, phy_sts;

	if ( plan_ps->rx_cb_pf || plan_ps->rx_cb_arg_p ) {
		REGLOCK(plan_ps);
		macCsrWrite(plan_ps, MAC_CR, macCsrRead(plan_ps, MAC_CR) | MAC_CR_RXEN);
		REGUNLOCK(plan_ps);
	}

	while (1) {
		rtems_event_set evs;
		rtems_event_receive(IRQ_EVENT | KILL_EVENT, RTEMS_EVENT_ANY | RTEMS_WAIT, RTEMS_NO_TIMEOUT, &evs);
		if ( KILL_EVENT & evs )
			break;
		int_sts = rd9118Reg(base, INT_STS) & plan_ps->int_msk;

		if ( RSFL_INT & int_sts ) {
		/* skip */
		drvLan9118RxIntBase = Read_hwtimer();
		while ( RX_FIFO_INF_RXSUSED_GET(rd9118Reg(base, RX_FIFO_INF)) > 0 ) {
			int left;

			rx_sts = rd9118Reg(base, RX_STS_FIFO);
			left   = RXSTS_PKTLEN_GET(rx_sts) + EH_PAD_BYTES;

			if ( RXSTS_ERR_ANY & rx_sts ) {

					if ( plan_ps->err_cb_pf )
						plan_ps->err_cb_pf(plan_ps, rx_sts | (1<<31),plan_ps->err_cb_arg_p);

					if ( RXSTS_FILT_FAIL   & rx_sts )
							plan_ps->stats_s.filf++;
					if ( RXSTS_LEN_ERR     & rx_sts )
							plan_ps->stats_s.lerr++;
					/* already handled by RWT_INT ? 
					   if ( RXSTS_RX_WDOG_TO  & rx_sts )
					   plan_ps->stats_s.rwt++;
					 */

					if ( RXSTS_MII_ERROR   & rx_sts )
							plan_ps->stats_s.mii++;
					if ( RXSTS_RUNT_FRAME  & rx_sts )
							plan_ps->stats_s.runt++;
					if ( RXSTS_FRAME_TOO_LONG  & rx_sts )
							plan_ps->stats_s.tool++;
					if ( RXSTS_LATE_COLL   & rx_sts )
							plan_ps->stats_s.lcol++;
					if ( RXSTS_FCS_ERROR   & rx_sts )
							plan_ps->stats_s.csum++;
			} else {
				if ( !plan_ps->rx_cb_pf || (left = plan_ps->rx_cb_pf(plan_ps, left, plan_ps->rx_cb_arg_p)) ) {
					uint32_t dly;
					if ( left < 16 && left > 0 ) {
						/* must do it manually */
						dly = Read_hwtimer();
						while ( left > 0 ) {
							rd9118Reg(base, RX_DATA_FIFO);
							left-=4;
						}
						dly = Read_hwtimer() - dly;
					} else {
						dly = skipPacket(plan_ps);
					}
#ifdef DEBUG
					if ( dly > maxFFDDelay )
						maxFFDDelay = dly;
#endif
				}
				if ( ( RXSTS_MCST_FRAME & rx_sts ) ) {
					plan_ps->stats_s.rxm++;
				} else if ( (RXSTS_BCST & rx_sts) ) {
					plan_ps->stats_s.rxb++;
				} else {
					plan_ps->stats_s.rxu++;
				}
			}
			/* at least 135ns delay after reading STS_FIFO - this is probably
			 * burnt by what happened up to here but we have to make sure...
			 */
#if 0
			DELAY135ns();
#else 
			DELAY90ns();
#endif
		}
		}
		if ( ERR_INTS & int_sts ) {
			if ( plan_ps->err_cb_pf )
				plan_ps->err_cb_pf(plan_ps, int_sts & ~(1<<31), plan_ps->err_cb_arg_p);
			if ( RWT_INT & int_sts )
				plan_ps->stats_s.rwt++;
			if ( RXE_INT & int_sts )
				plan_ps->stats_s.rxe++;
			if ( TXE_INT & int_sts )
				plan_ps->stats_s.txe++;
			if ( TDFU_INT & int_sts )
				plan_ps->stats_s.tdfu++;
			if ( TDFO_INT & int_sts )
				plan_ps->stats_s.tdfo++;
			if ( RXDF_INT & int_sts )
				plan_ps->stats_s.rxdf+=rd9118Reg(base, RX_DROP);
			if ( RSFF_INT & int_sts )
				plan_ps->stats_s.rsff++;
			if ( TSFF_INT & int_sts )
				plan_ps->stats_s.tsff++;
		}
		if ( TSFL_INT & int_sts ) {
			while ( TX_FIFO_INF_TXSUSED_GET(rd9118Reg(base, TX_FIFO_INF)) > 0 ) {
				tx_sts = rd9118Reg(base, TX_STS_FIFO);
				if ( plan_ps->tx_cb_pf )
					plan_ps->tx_cb_pf(plan_ps, tx_sts, plan_ps->tx_cb_arg_p);
				else {
					/* status with tag 0 is dropped */
					if ( TXSTS_TAG_GET(tx_sts) )
						rtems_message_queue_send(plan_ps->txq, &tx_sts, sizeof(tx_sts));
#ifdef DEBUG
					else
						if ( (drvLan9118Debug & DEBUG_TXSTS) && (tx_sts & 0xffff) )
							printf("Discarding TX status 0x%08lx\n", tx_sts);
#endif
				}

				/* at least 135ns delay after reading STS_FIFO - this is probably
				 * burnt by what happened up to here...
				 */
#if 1
				DELAY135ns();
#endif
				plan_ps->stats_s.txp++;
			}
		}
		if ( PHY_INT & int_sts ) {
			REGLOCK(plan_ps);
			drvLan9118_mdio_r(0, plan_ps, MII_INT_SRC, &phy_sts);
			phy_sts &= plan_ps->phy_int_msk;
			if ( plan_ps->phy_cb_pf )
				plan_ps->phy_cb_pf(plan_ps, phy_sts, plan_ps->phy_cb_arg_p);
			/* apparently, we must read the BMSR once to clear 'link lost' condition */
			drvLan9118_mdio_r(0, plan_ps, MII_BMSR, &phy_sts);
			REGUNLOCK(plan_ps);
		}
		/* clear and re-enable IRQ */
		wr9118Reg(base, INT_STS, int_sts);
		drvLan9118IrqEnable();
	}
	if ( plan_ps->sync ) {
		rtems_semaphore_release(plan_ps->sync);
		rtems_task_suspend(RTEMS_SELF);
	} else {
		rtems_task_delete(RTEMS_SELF);
	}
}

void
drvLan9118DumpRxSts(uint32_t rx_sts, FILE *f_p)
{
	if ( !f_p )
		f_p = stdout;
	fprintf(f_p,"RX status word errors:\n");
	if ( RXSTS_FILT_FAIL   & rx_sts )
		fprintf(f_p,"  Filter failure\n");
	if ( RXSTS_LEN_ERR     & rx_sts )
		fprintf(f_p,"  Length error\n");
	if ( RXSTS_RX_WDOG_TO  & rx_sts )
		fprintf(f_p,"  RX watchdog timeout (packet too long)\n");
	if ( RXSTS_MII_ERROR   & rx_sts )
		fprintf(f_p,"  MII-error\n");
	if ( RXSTS_RUNT_FRAME  & rx_sts )
		fprintf(f_p,"  RUNT frame\n");
	if ( RXSTS_FRAME_TOO_LONG  & rx_sts )
		fprintf(f_p,"  frame too long\n");
	if ( RXSTS_LATE_COLL   & rx_sts )
		fprintf(f_p,"  late collision\n");
	if ( RXSTS_FCS_ERROR   & rx_sts )
		fprintf(f_p,"  checksum error\n");
	fprintf(f_p,"---\n");
}

static void
prether(FILE *f_p, const unsigned char *ea_p)
{
int i;

	for (i=0; i<5; i++)
		fprintf(f_p,"%02X:",*ea_p++);
	fprintf(f_p,"%02X",*ea_p);
}


static int
e2p_cmd_exec(DrvLan9118_tps plan_ps, uint32_t cmd)
{

	/* timeout bit is clear-on-write; make sure we reset any old
	 * timeout condition...
	 */
	wr9118Reg(plan_ps->base, E2P_CMD, cmd | E2P_CMD_BUSY | E2P_CMD_EPC_TIMEOUT);

	cmd = e2p_busywait(plan_ps);

	if ( E2P_CMD_EPC_TIMEOUT & cmd ) {
		return -ETIMEDOUT;
	}
	return 0;
}

#define ERASE_LOC(off)	(E2P_CMD_ERASE | E2P_CMD_EPC_ADDR_SET(off))

static int
doit(DrvLan9118_tps plan_ps, uint32_t cmd, uint8_t *d_pa, const uint8_t *s_pa, unsigned off, unsigned len)
{
int rval = -1;

	if ( !plan_ps || !len || off + len > EEPROM_SIZE )
		return -EINVAL;

	REGLOCK(plan_ps);
	/* Reject access while driver is running */
	if ( plan_ps->tid ) {
		rval = -EACCES;
		goto bail;
	}

	if ( E2P_CMD_BUSY & rd9118Reg(plan_ps->base, E2P_CMD) ) {
		rval = -EBUSY;
		goto bail;
	}
	
	while ( len-- ) {

		if ( s_pa ) {
			/* WRITE operation */

			/* erase target location first */
			if ( (rval = e2p_cmd_exec(plan_ps, ERASE_LOC(off))) )
				goto bail;

			/* write data */
			wr9118Reg(plan_ps->base, E2P_DATA, *s_pa);
			s_pa++;
		}

		if ( (rval = e2p_cmd_exec(plan_ps, cmd | E2P_CMD_EPC_ADDR_SET(off) )) )
			goto bail;

		if ( d_pa ) {
			/* READ operation */
			*d_pa = rd9118Reg(plan_ps->base, E2P_DATA);
			d_pa++;
		}

		off++;
	}

	rval = 0;

bail:
	REGUNLOCK(plan_ps);
	return rval;
}

/* NOTE: All EEPROM access routines lock the registers and busy wait.
 *       They are intended to be used during initialization or maintenance
 *       and may impact daemon operation (latencies).
 *       Therefore, executing any EEPROM access routine is rejected if
 *       the driver is already running (after the start routine is executed).
 */

/* Read from EEPROM; return 0 (success) or (-ERRNO) on error */
int
drvLan9118E2PRead(DrvLan9118_tps plan_ps, void *dst_pa, unsigned src, unsigned len)
{
	if ( !plan_ps || !len || src + len > EEPROM_SIZE )
		return -EINVAL;

	if ( plan_ps->tid ) {
		memcpy(dst_pa, &eeprom_shadow[src], len);
		return 0;
	}

	return doit(plan_ps, E2P_CMD_READ, dst_pa, 0, src, len);
}

/* Write to EEPROM (erasing target locations first);
 * return 0 (success) or (-ERRNO) on error
 */
int
drvLan9118E2PWrite(DrvLan9118_tps plan_ps, const void *src_pa, unsigned dst, unsigned len)
{
	/* prevent inadvertent overwrite of MAC address */
	if ( dst < 7 )
		return -EINVAL;
	return doit(plan_ps, E2P_CMD_WRITE, 0, src_pa, dst, len);
}

int
drvLan9118E2PWriteEnaddr(DrvLan9118_tps plan_ps, const uint8_t *enaddr_pa)
{
uint8_t header = 0xa5;
int     rval   = doit(plan_ps, E2P_CMD_WRITE, 0, &header, 0, 1);

	if ( rval )
		return rval;
	rval = doit(plan_ps, E2P_CMD_WRITE, 0, enaddr_pa, 1, 6);

	if ( rval )
		return rval;

	return e2p_cmd_exec(plan_ps, E2P_CMD_RELOAD);
}


int
drvLan9118E2PWriteEnable(DrvLan9118_tps plan_ps)
{
	return doit(plan_ps, E2P_CMD_EWEN, 0, 0, 0, 1);
}

int
drvLan9118E2PWriteDisable(DrvLan9118_tps plan_ps)
{
	return doit(plan_ps, E2P_CMD_EWDS, 0, 0, 0, 1);
}

int
drvLan9118E2PErase(DrvLan9118_tps plan_ps)
{
	return doit(plan_ps, E2P_CMD_ERAL, 0, 0, 0, 1);
}

int
drvLan9118DumpHeaderRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *closure_p)
{
LanUdpPktRec	pkt;
LanIpPart       ipp = &pkt.ip_part;

FILE			*f_p = (FILE*)closure_p;
int				tmp;
char			ipbuf_a[20];
struct in_addr	sa;

	if ( !f_p )
		f_p = stdout;

	if ( len < sizeof(ipp->ll) ) {
		fprintf(f_p,"Packet shorter than an ethernet header ???\n");
		goto bail;
	}
	memcpy(&ipp->ll, (void*)(plan_ps->base + FIFO_ALIAS), sizeof(ipp->ll));
#if defined(HW_BYTES_NOT_SWAPPED) && BYTE_ORDER == BIG_ENDIAN
	drvLan9118BufRev((void*)&ipp->ll, sizeof(ipp->ll)/4);
#endif
	len -= sizeof(ipp->ll);
	prether(f_p,ipp->ll.src); fprintf(f_p," -> "); prether(f_p,ipp->ll.dst);
	if ( 0x800 != (tmp = (unsigned short)ntohs(ipp->ll.type)) ) {
		fprintf(f_p," Ethernet type/lenght %#x\n", tmp);
	} else {
		fprintf(f_p," (IP)\n");
		if ( len < sizeof(ipp->ip) ) {
			fprintf(f_p,"  Packet shorter than an IP header ???\n");
			goto bail;
		}
		memcpy(&ipp->ip, (void*)(plan_ps->base + FIFO_ALIAS), sizeof(ipp->ip));
#if defined(HW_BYTES_NOT_SWAPPED) && BYTE_ORDER == BIG_ENDIAN
		drvLan9118BufRev((void*)&ipp->ip, sizeof(ipp->ip)/4);
#endif
		len -= sizeof(ipp->ip);
		if ( ipp->ip.vhl >> 4 != 4 ) {
			fprintf(f_p,"  IP header not version 4 ???\n");
			goto bail;
		}
		sa.s_addr = ipp->ip.src;
		ipbuf_a[0]=0;
		inet_ntop(AF_INET, &sa, ipbuf_a, sizeof(ipbuf_a));
		fprintf(f_p,"  IP -- src %s -> ",ipbuf_a);
		sa.s_addr = ipp->ip.dst;
		ipbuf_a[0]=0;
		inet_ntop(AF_INET, &sa, ipbuf_a, sizeof(ipbuf_a));
		fprintf(f_p,"%s PROTO ",ipbuf_a);
		if ( 17 == ipp->ip.prot ) {
			fprintf(f_p,"UDP\n");
			if ( len < sizeof(pkt.udp) ) {
				fprintf(f_p,"    Packet shorter than UDP header ???\n");
				goto bail;
			}
			memcpy(&pkt.udp, (void*)(plan_ps->base + FIFO_ALIAS), sizeof(pkt.udp));
#if defined(HW_BYTES_NOT_SWAPPED) && BYTE_ORDER == BIG_ENDIAN
			drvLan9118BufRev((void*)&pkt.udp, sizeof(pkt.udp)/4);
#endif
			len -= sizeof(pkt.udp);
			fprintf(f_p,"    UDP -- SPORT: %u -> DPORT: %u; payload %lu bytes\n",
				ntohs(pkt.udp.sport), ntohs(pkt.udp.dport), ntohs(pkt.udp.len) - sizeof(pkt.udp));
		} else {
			fprintf(f_p,"%u LEN %u\n", ipp->ip.prot, ntohs(ipp->ip.len));
		}
	}
bail:
		return len;
}

void
drvLan9118McFilterClear(DrvLan9118_tps plan_ps)
{
int i;
	REGLOCK(plan_ps);
		macCsrWrite(plan_ps, HASHH, 0);
		macCsrWrite(plan_ps, HASHL, 0);
		for ( i=0; i<NUM_MC_HASHES; i++ ) {
			plan_ps->mc_refcnt[i] = 0;
		}
	REGUNLOCK(plan_ps);
}

static int
mc_addr_valid(uint8_t *enaddr)
{
static const char bcst[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return ( (0x01 & enaddr[0]) && memcmp(enaddr, bcst, sizeof(bcst)) );
}

static uint32_t
mc_hash(uint8_t *enaddr)
{
uint32_t crc = 0xffffffff;
int      i,j;
#define CRCPOLY_LE 0xedb88320
	for ( j=0; j<6; j++) {
		crc ^= enaddr[j];
		for ( i=0; i<8; i++ ) {
			crc = (crc>>1) ^ ( (crc & 1) ? CRCPOLY_LE : 0 );
		}
	}
	/* bit-reverse; this is not exactly documented
	 * but other chips do the same thing...
	 * 
	 * We only want the upper-most bits of the end-result.
	 * Hence, it is enough to bit-reverse the least-significant
	 * byte (which will be the MSB after bit-reversal).
	 */
	crc = ((crc&0x0f) << 4) | ((crc&0xf0) >> 4);
	crc = ((crc&0x33) << 2) | ((crc&0xcc) >> 2);
	crc = ((crc&0x55) << 1) | ((crc&0xaa) >> 1);
	/* now strip the 2 lower-bits */
	return (crc >> 2);
}

void
drvLan9118McFilterAdd(DrvLan9118_tps plan_ps, uint8_t *mac_addr)
{
uint32_t hash;
uint32_t off;
uint32_t val;
	if ( ! mc_addr_valid(mac_addr) )
		return;
	hash = mc_hash(mac_addr);
	REGLOCK(plan_ps);
	if ( 0 == plan_ps->mc_refcnt[hash]++ ) {
		off  = hash > 31 ? HASHH : HASHL;
		val  = macCsrRead(plan_ps, off);
		val |= (1<<(hash & 31));
		macCsrWrite(plan_ps, off, val);
	}
	REGUNLOCK(plan_ps);
}

void
drvLan9118McFilterDel(DrvLan9118_tps plan_ps, uint8_t *mac_addr)
{
uint32_t hash;
uint32_t off;
uint32_t val;
	if ( ! mc_addr_valid(mac_addr) )
		return;
	hash = mc_hash(mac_addr);
	REGLOCK(plan_ps);
	if ( plan_ps->mc_refcnt[hash] > 0 && 0 == --plan_ps->mc_refcnt[hash] ) {
		off  = hash > 31 ? HASHH : HASHL;
		val  = macCsrRead(plan_ps, off);
		val &= ~(1<<(hash & 31));
		macCsrWrite(plan_ps, off, val);
	}
	REGUNLOCK(plan_ps);
}

int
drvLan9118BcFilterSet(DrvLan9118_tps plan_ps, int val)
{
uint32_t v,vo;

	REGLOCK(plan_ps);
	vo = macCsrRead(plan_ps, MAC_CR);
	if ( val >= 0 ) {
		if ( val ) {
			v = vo |  MAC_CR_BCAST;
		} else {
			v = vo & ~MAC_CR_BCAST;
		}
		macCsrWrite(plan_ps, MAC_CR, v);
	}
	REGUNLOCK(plan_ps);

	return !!(vo & MAC_CR_BCAST);
}
