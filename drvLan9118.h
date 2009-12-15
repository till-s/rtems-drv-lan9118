
/*=============================================================================

	$Id$
  Name: drvlan9118.h

  Abs:  Public Interface of Driver for the SMSC LAN9118
        Ethernet Adapter. This driver is specially designed
        for low-level packet handling. It is not a general-
        purpose rtems/bsd networking driver.

        Some dependency on the uc5282 RTEMS BSP exists.

  Auth: 12-oct-2006, Till Straumann (tss)
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
        [ consult CVS log ]
 
=============================================================================*/

#ifndef DRV_LAN9118_H
#define DRV_LAN9118_H

/* Look for compile-time configurable parameters after the 
 * include file section in the '.c' file...
 */

#include <rtems.h>
#include <stdio.h>
#include <stdint.h>

/* TX status word          */
#define TXSTS_TAG_GET(x)	(((x)>>16)&0xffff)
#define TXSTS_ERROR		(1<<15)
#define TXSTS_CARRIER_LOSS	(1<<11)
#define TXSTS_NO_CARRIER	(1<<10)
#define TXSTS_LATE_COLL		(1<< 9)
#define TXSTS_EXCESS_COLL	(1<< 8)
#define TXSTS_COLL_CNT_GET(x)	(((x)>>3)&0xf)
#define TXSTS_EXCESS_DEFER	(1<< 2)
#define TXSTS_FIFO_UNDERRUN	(1<< 1)
#define TXSTS_DEFERRED		(1<< 0)

/* RX status word           */
#define RXSTS_FILT_FAIL		(1<<30)
#define RXSTS_PKTLEN_GET(x)	(((x)>>16)&0x3fff)
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

#define RXSTS_ERR_ANY		(RXSTS_FILT_FAIL | RXSTS_ERROR | RXSTS_LEN_ERR | RXSTS_RX_WDOG_TO | RXSTS_MII_ERROR)

/* enable/disable interrupt at the lan9118 device
 * (actually at the arcturus EPORT module which is more efficient)
 */
inline void
drvLan9118IrqEnable(void);

inline void
drvLan9118IrqDisable(void);

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

/* The above routines don't take a driver handle because they
 * all operate on the MCF5282 (and are thus BSP specific).
 */

/* Type of a driver handle */
typedef struct DrvLan9118_ts_ *DrvLan9118_tps;

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
DrvLan9118_tps
drvLan9118Setup(const uint8_t *enaddr_pa, uint32_t flags);

/* Read HW address; buf must provide space for 6 bytes */
void
drvLan9118ReadEnaddr(DrvLan9118_tps plan_ps, uint8_t *buf_pa);

/* Control broadcast filter.
 *
 *  rej > 0 : Disable reception of broadcast frames (filter rejects).
 *  rej = 0 : Enable  reception of broadcast frames (filter accepts).
 *  rej < 0 : Don't modify current value (pass -1 to read current setting).
 *
 * RETURNS: previous setting (nonzero if bcast rejected, zero if accepted).
 */
int
drvLan9118BcFilterSet(DrvLan9118_tps plan_ps, int rej);

/* Clear Multicast Address Filter
 */
void
drvLan9118McFilterClear(DrvLan9118_tps plan_ps);

/* Add a MAC address to (imperfect) multicast filter.
 * A reference-count is kept so that the same address
 * can be added multiple times.
 */
void
drvLan9118McFilterAdd(DrvLan9118_tps plan_ps, uint8_t *mac_addr);

/* Remove a MAC address from (imperfect) multicast filter.
 * The address is really only removed if the reference count 
 * drops to zero.
 */ 
void
drvLan9118McFilterDel(DrvLan9118_tps plan_ps, uint8_t *mac_addr);


/* The 'sts' word passed to and the return value of the various callbacks
 * have the following meaning:
 *
 *     tx_cb: The TX status word as read from the TX status fifo.
 *            RETURN VALUE: The return value of the tx_cb is ignored.
 *
 *     rx_cb: The number of characters still left in the data fifo.
 *            RETURN VALUE: the remaining bytes in the data fifo
 *                          ('sts' minus the number of characters read by
 *                          the callback). This may be negative since
 *                          if the last byte is not aligned to a uint32_t
 *                          boundary since the fifo must always be read in
 *                          multiples of 4 bytes.
 *            NOTES: The rx_cb is only executed if no RX errors occurred.
 *
 *     err_cb: The 'err_cb' is executed under two circumstances:
 *              1) packet received but has nonzero error status:
 *                 --> 'sts' is the RX status word as read from the RX status
 *                     fifo.
 *              2) One of the 'error' interrupt bits is set (no status word
 *                 or data in fifos).
 *                 --> 'sts' is the int_sts word read from the device.
 *             To distinguish the two cases (1<<31) is set (1) or cleared (2),
 *             respectively.
 *
 *             RETURN VALUE: The return value of the err_cb is ignored.
 *
 *     phy_cb: Executed to report a 'phy interrupt/event'. 'sts' reflects
 *             the contents of the MII_INT_SRC register (current mask applied).
 *             Can be used e.g., to track link status change events.
 *
 *             RETURN VALUE: The return value of the phy_cb is ignored.
 */


typedef int (*DrvLan9118CB_tpf)(DrvLan9118_tps plan_ps, uint32_t sts, void *closure);

/* Demo RX callback for debugging; info about received frames is printed to 
 * (FILE*) closure [stdout if NULL].
 */
int
drvLan9118DumpHeaderRxCb(DrvLan9118_tps plan_ps, uint32_t len, void *closure);


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
drvLan9118Start(DrvLan9118_tps plan_ps,
				uint32_t prio, uint32_t stacksz,
				DrvLan9118CB_tpf rx_cb_pf, 	void *rx_cb_arg_p,
				DrvLan9118CB_tpf tx_cb_pf, 	void *tx_cb_arg_p,
				DrvLan9118CB_tpf err_cb_pf,	void *err_cb_arg_p,
				DrvLan9118CB_tpf phy_cb_pf,	void *phy_cb_arg_p);
		
/* Shutdown driver and release all resources.
 * NOTE: It is illegal to call any other driver entry point the shutdown.
 *       To start the driver again, you must call 'Setup' followed by 'Start'.
 */
void
drvLan9118Shutdown(DrvLan9118_tps plan_ps);

/* read from the RX fifo (to be executed from RX callback) */
void
drvLan9118FifoRd(DrvLan9118_tps plan_ps, void *buf_p, int n_bytes);

/* Dump driver statistics to a file (stdout if NULL) */
int
drvLan9118DumpStats(DrvLan9118_tps plan_ps, FILE *f_ps);

/* just the SIOCSIFMEDIA/SIOCGIFMEDIA ioctls */
int
drvLan9118ioctl(DrvLan9118_tps plan_ps, int cmd, int *media_p);

/* Byte-reverse a buffer. This is only needed on a big-endian board
 * where the byte-lanes have *not* been swapped in hardware.
 * [old test-board design].
 */
void
drvLan9118BufRev(uint32_t *buf_p, int nwords);
 
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
 *
 * RETURNS: 0 on success, nonzero if there was not enough space in the FIFO
 *          if this call fails then the TX is left unlocked.
 */
uint32_t
drvLan9118TxPacket(DrvLan9118_tps plan_ps, const void *buf_p, int nbytes, unsigned short tag);

/* Write n_bytes to TX fifo;
 * NOTE: TX lock must be held (call TxPacket routine first)
 */
void
drvLan9118FifoWr(DrvLan9118_tps plan_ps, const void *buf_p, int n_bytes);

/* Obtain address of the FIFO (drivers who might want to do
 * perform their own low-level accesses
 *
 * NOTES: * Driver must be locked/unlocked around FIFO access
 *          (TX: see drvLan9118TxUnlock(); RX: call from RX callback
 *          context)
 *
 *        * Access MUST be in by 32-bit words.
 *        * DONT use this feature of the driver. Use drvLan9118FifoWr().
 *          unless you know exactly what you are doing.
 *
 *        * 'endianBig' indicates that the MSB of a 32-bit word
 *          goes on the wire first.
 */
volatile uint32_t *
drvLan9118FifoAddr(DrvLan9118_tps plan_ps, int *endianBig);

/* Unlock the Transmitter; Intended use:
 * 
 *  1) prepare packet with drvLan9118TxPacket() passing a NULL buffer (length must still be known)
 *  2) *CHECK RETURN CODE* abort here if nonzero
 *  3) write payload with (repeated) calls to drvLan9118FifoWr()
 *  4) unlock transmitter calling drvLan9118TxUnlock()
 */
void
drvLan9118TxUnlock(DrvLan9118_tps plan_ps);

/* Wait for the status of a pending transmission to be reported
 * 
 * RETURNS: 0 on success RTEMS_TIMEOUT on timeout or other error status
 *          -1: no daemon started or a TX callback was registered [which
 *          is then responsible for handling the status].
 */
rtems_status_code
drvLan9118TxStatus(DrvLan9118_tps plan_ps, uint32_t *pval_p, uint32_t timeout);

/* Flush TX status and DATA fifos 
 * To be called from TX thread ONLY
 * RETURNS: time it took (5282 timer ticks), see source code; the
 *          return value is for evaluation/debugging purposes only.
 */
#define STATUS_FIFO	(1<<15)
#define DATA_FIFO	(1<<14)

uint32_t
drvLan9118TxFlush(DrvLan9118_tps plan_ps, int which);

/* Flush RX fifo; 
 * RETURNS: time it took (5282 timer ticks), see source code; the
 *          return value is for evaluation/debugging purposes only.
 */
uint32_t
drvLan9118RxFlush(DrvLan9118_tps plan_ps);

/* Dump RX buffer statistics to a file (stdout if NULL)
 * This routine is intended to be called from a RX callback
 */
void drvLan9118DumpRxSts(uint32_t rx_sts, FILE *f_ps);

/* EEPROM ACCESS ROUTINES */

/* NOTES: - All EEPROM access routines lock the registers and busy wait.
 *          They are intended to be used during initialization or maintenance
 *          and may impact daemon operation (latencies).
 *          Therefore, executing any EEPROM access routine is rejected if
 *          the driver is already running (after the start routine is executed).
 *
 *        - All routines return 0 on success or (-ERRNO) on error.
 */

/* Read from EEPROM; return 0 (success) or (-ERRNO) on error;
 *
 * NOTE: If called after starting the driver this routine returns the contents
 *       of a shadow buffer in RAM.
 */
int
drvLan9118E2PRead(DrvLan9118_tps plan_ps, void *dst_pa, unsigned src, unsigned len);

/* Write to EEPROM (erasing target locations first);
 * Returns 0 (success) or (-ERRNO) on error.
 * NOTES:  - for writing the hardware address you must use the 'WriteEnaddr' entry
 *           point below; using the generic Write routine yields -EINVAL.
 *         - you must call WriteEnable prior to writing.
 */
int
drvLan9118E2PWrite(DrvLan9118_tps plan_ps, const void *src_pa, unsigned dst, unsigned len);

/* Write the first 7 bytes of the EEPROM (a '0xa5' prefix is added by this routine)
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteEnaddr(DrvLan9118_tps plan_ps, const uint8_t *enaddr_pa);

/* Enable write operations
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteEnable(DrvLan9118_tps plan_ps);

/* Disable write operations
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PWriteDisable(DrvLan9118_tps plan_ps);

/* Erase entire device
 * Returns 0 (success) or (-ERRNO) on error.
 */
int
drvLan9118E2PErase(DrvLan9118_tps plan_ps);

#ifdef __cplusplus
}
#endif

#endif
