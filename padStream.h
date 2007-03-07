#ifndef PAD_STREAM_H
#define PAD_STREAM_H
/* $Id$ */

#include <udpComm.h>
#include <padProto.h>
#include <lanIpBasic.h>
#include <drvLan9118.h>
#include <rtems.h>
#include <errno.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Data stream implementation. This could all be done over the
 * udpSock abstraction but less efficiently since we would have
 * to copy the PAD fifo to memory first instead of copying the
 * PAD fifo to the lan9118 chip directly...
 */

/* Setup everything for stream processing.
 * The 'cb' callback is executed (padUdpHandler task context)
 * with a nonzero 'start' argument when the stream is
 * started and again with a nonzero argument when the stream
 * is stopped (enable/disable data source).
 * The callback should return 0 on success, nonzero if the
 * stream cannot be started or stopped.
 */

int
padStreamInitialize(IpCbData if_p, int (*cb)(int start, void *uarg), void *uarg);

/* cleanup (for module unloading) */
int
padStreamCleanup();

/* refresh timestamp and transaction id */
int
padStreamPet(PadRequest req);

/* Function that actually starts streaming transfer
 * to a host.
 *        'me': our channel #
 *    'hostip': IP address in network byte order.
 *  'hostport': UDP port on host (*host byte order*).
 */
int
padStreamStart(PadRequest req, PadStrmCommand scmd, int me, uint32_t hostip);

int
padStreamSend(void * (*getdata)(void *packBuffer, int idx, int nsamples, int endianLittle, int colMajor, void *uarg), int type, int idx, void *uarg);

/* execute 'padStreamSend' with test data */
int
padStreamTest();

/* execute 'padStreamSend' with simulated/generated data */
int
padStreamSim(PadSimCommand scmd);

/* Function that actually stops streaming transfer */
int
padStreamStop(void);

#ifdef __cplusplus
}
#endif

#endif
