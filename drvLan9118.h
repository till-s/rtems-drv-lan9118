#ifndef DRV_LAN9118_H
#define DRV_LAN9118_H
/* $Id$ */

/* Raw packet driver for the lan9118 10/100 ethernet chip */

/* This driver was written for the uC5282 BSP and BSP-specifica
 * have not been separated out [yet].
 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 2006 */

/* Look for compile-time configurable parameters after the 
 * include file section in the '.c' file...
 */

/* #define BYTES_NOT_SWAPPED */

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

#include <rtems/rtems/cache.h>
#include <rtems/bspIo.h>
#include <rtems/error.h>

#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sockio.h>
#include <net/if.h>

#include <netinet/in.h>
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

#include "drv5282DMA.h"

#include <drvLanProto.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DrvLan9118Rec_;
typedef struct DrvLan9118Rec_ *DrvLan9118;


#define REGLOCK(plan)		assert( !rtems_semaphore_obtain((plan)->mutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT))
#define REGUNLOCK(plan) 	assert( !rtems_semaphore_release((plan)->mutx))

#define TXLOCK(plan)		assert( !rtems_semaphore_obtain((plan)->tmutx, RTEMS_WAIT, RTEMS_NO_TIMEOUT))
#define TXUNLOCK(plan) 		assert( !rtems_semaphore_release((plan)->tmutx) )

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

/* RX status word           */
#define RXSTS_FILT_FAIL		(1<<30)
#define RXSTS_PKTLEN_GET(x)		__GET_BITS(x,16,0x3fff)
#define RXSTS_PKTLEN_SET(x)		__SET_BITS(x,16,0x3fff)
#define RXSTS_ERROR		(1<<15) /* or of 11,7,6,1 */
#define RXSTS_BCST		(1<<13)
#define RXSTS_LEN_ERR		(1<<12)
#define RXSTS_RUNT_FRAME	(1<<11)
#define RXSTS_MCST_FRAME	(1<<10)
#define RXSTS_FRAME_TOO_LONG	(1<< 7)
#define RXSTS_LATE_COLL		(1<< 6)
#define RXSTS_FRAME_TYPE	(1<< 5)
#define RXSTS_RX_WDOG_TO	(1<< 4)
#define RXSTS_MII_ERROR		(1<< 3)
#define RXSTS_DRIBBLING		(1<< 2)
#define RXSTS_FCS_ERROR		(1<< 1)

#define RXSTS_ERR_ANY	(RXSTS_FILT_FAIL | RXSTS_ERROR | RXSTS_LEN_ERR | RXSTS_RX_WDOG_TO | RXSTS_MII_ERROR)

#include <stdint.h>


/* enable/disable interrupt at the lan9118 device (actually at the arcturus EPORT module
 * which is more efficient)
 */
inline void
drvLan9118IrqEnable();

inline void
drvLan9118IrqDisable();

/* Hard reset via a dedicated wire from the arcturus board
 *
 * !!!!!!!
 * NOTE: clever arcturus designers *renamed* pins in a confusing
 *       way: what they call PTC is what the coldfire manual calls
 *       port TD !!
 * !!!!!!!
 *
 * Writing a low level (0) holds the device in reset.
 * Proper timing requirements have to be met (lan9118 data sheet).
 * before releasing the reset line.
 * This routine is 5282 specific.
 * 
 * RETURNS: current level or -1 if not implemented.
 */
int
drvLan9118HardReset(int level);

/* Setup the driver (not completely functional yet, it must be started):
 *
 * 'enaddr': 6-byte HW address. If NULL the 'HWADDR1' environment ('benv') variable
 *           is used. If 'HWADDR1' is not set either then the hardware address is
 *           read from an e2prom (if present and programmed).
 *
 *         
 * RETURNS: driver handle on success, 0 on error.
 */

#define LAN9118_FLAG_BCDIS	1	/* disable reception of broadcast packets */
DrvLan9118
drvLan9118Setup(	unsigned char *enaddr,
					uint32_t flags
			   );

/* Read HW address; buf must provide space for 6 bytes */
void
drvLan9118ReadEnaddr(DrvLan9118 plan, uint8_t *buf);

typedef int (*DrvLan9118CB)(struct DrvLan9118Rec_ *plan, uint32_t sts, void *closure);

/* Start driver (prio/stacksz may be 0 to select a default)
 *
 * The RX, TX, Error and PHY callbacks are executed in the context of
 * the driver task (if non-zero).
 *
 * Any callback may be zero and the respective event is ignored.
 * The RX is not started if no RX callback is present.
 *
 * If no TX callback is registered then the status of a terminated
 * transmission is posted to a message queue from where (another thread)
 * can pick it up via drvLan9118TxStatus().
 * NOTE: packets sent with a zero tag are not reported via the message
 *       queue.
 *
 */
int
drvLan9118Start(DrvLan9118 plan,
				uint32_t prio, uint32_t stacksz,
				DrvLan9118CB rx_cb, 	void *rx_cb_arg,
				DrvLan9118CB tx_cb, 	void *tx_cb_arg,
				DrvLan9118CB err_cb, 	void *err_cb_arg,
				DrvLan9118CB phy_cb,	void *phy_cb_arg);
		
/* Shutdown driver and release all resources.
 * NOTE: It is illegal to call any other driver entry point the shutdown.
 *       To start the driver again, you must call 'Setup' followed by 'Start'.
 */
void
drvLan9118Shutdown(DrvLan9118 plan);

/* read from the RX fifo (to be executed from RX callback) */
void
drvLan9118FifoRd(DrvLan9118 plan, void *buf, int n_bytes);

/* Dump driver statistics to a file (stdout if NULL) */
int
drvLan9118DumpStats(DrvLan9118 plan, FILE *f);

/* just the SIOCSIFMEDIA/SIOCGIFMEDIA ioctls */
int
drvLan9118ioctl(DrvLan9118 plan, int cmd, int *p_media);

/* Byte-reverse a buffer. This is only needed on a big-endian board
 * where the byte-lanes have *not* been swapped in hardware.
 * [old test-board design].
 */
void
drvLan9118BufRev(uint32_t *buf, int nwords);
 
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
 *       TX mutex locked and it must be unlocked by the user (see below).
 *
 * The 'tag' value is passed back to the callback or tx status message queue.
 * A 0 tag is special, however: it's status is never reported to a message queue
 * but discarded if no tx callback is registered.
 */
uint32_t
drvLan9118TxPacket(DrvLan9118 plan, void *buf, int nbytes, unsigned short tag);

/* Write n_bytes to TX fifo;
 * NOTE: TX lock must be held (call TxPacket routine first)
 */
void
drvLan9118FifoWr(DrvLan9118 plan, void *buf, int n_bytes);

/* Unlock the Transmitter; Intended use:
 * 
 *  1) prepare packet with drvLan9118TxPacket() passing a NULL buffer (length must still be known)
 *  2) write payload with (repeated) calls to drvLan9118FifoWr()
 *  3) unlock transmitter calling drvLan9118TxUnlock()
 */
void
drvLan9118TxUnlock(DrvLan9118 plan);

/* Wait for the status of a pending transmission to be reported
 * 
 * RETURNS: 0 on success RTEMS_TIMEOUT on timeout or other error status
 *          -1: no daemon started or a TX callback was registered [which
 *          is then responsible for handling the status].
 */
rtems_status_code
drvLan9118TxStatus(DrvLan9118 plan, uint32_t *pval, uint32_t timeout);

/* Flush TX status and DATA fifos 
 * To be called from TX thread ONLY
 * RETURNS: time it took (5282 timer ticks), see source code; the
 *          return value is for evaluation/debugging purposes only.
 */
#define STATUS_FIFO	(1<<15)
#define DATA_FIFO	(1<<14)

uint32_t
drvLan9118TxFlush(DrvLan9118 plan, int which);

/* Flush RX fifo; 
 * RETURNS: time it took (5282 timer ticks), see source code; the
 *          return value is for evaluation/debugging purposes only.
 */
uint32_t
drvLan9118RxFlush(DrvLan9118 plan);

/* Dump RX buffer statistics to a file (stdout if NULL)
 * This routine is intended to be called from a RX callback
 */
void drvLan9118DumpRxSts(uint32_t rx_sts, FILE *f);

/* EEPROM ACCESS ROUTINES */

/* NOTES: - All EEPROM access routines lock the registers and busy wait.
 *          They are intended to be used during initialization or maintenance
 *          and may impact daemon operation (latencies).
 *          Therefore, executing any EEPROM access routine is rejected if
 *          the driver is already running (after the start routine is executed).
 *
 *        - All routines return 0 on success or (-ERRNO) on error.
 */

/* Read from EEPROM; return 0 (success) or (-ERRNO) on error */
int
drvLan9118E2PRead(DrvLan9118 plan, void *dst, unsigned src, unsigned len);

/* Write to EEPROM (erasing target locations first);
 * Returns 0 (success) or (-ERRNO) on error.
 * NOTES:  - for writing the hardware address you must use the 'WriteEnaddr' entry
 *           point below; using the generic Write routine yields -EINVAL.
 *         - you must call WriteEnable prior to writing.
 */
int
drvLan9118E2PWrite(DrvLan9118 plan, void *src, unsigned dst, unsigned len);

/* Write the first 7 bytes of the EEPROM (a '0xa5' prefix is added by this routine)
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteEnaddr(DrvLan9118 plan, uint8_t *enaddr);

/* Enable write operations
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteEnable(DrvLan9118 plan);

/* Disable write operations
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteDisable(DrvLan9118 plan);

/* Erase entire device
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PErase(DrvLan9118 plan);

/* RX callback for debugging; info about received frames is printed to 
 * (FILE*)closure [stdout if NULL].
 */
int
drvLan9118DumpHeaderRxCb(DrvLan9118 plan, uint32_t len, void *closure);

#ifdef __cplusplus
}
#endif

#endif
