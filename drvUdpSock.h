#ifndef UDP_SOCK_FS_DRIVER_H
#define UDP_SOCK_FS_DRIVER_H
/* $Id$ */

/* Simple driver providing access to 'udpSock' via the file
 * system (read/write). A few IOCTL codes are also supported
 * (mainly the ones needed by rtems-gdb-stub).
 *
 * Connection management:
 *
 *   - device minor number of filesystem node gives local
 *     port number the UDP 'socket' is bound to.
 *   - open creates a local UDP 'socket'
 *   - read/write 
 *   - ioctl can be used to connect 'socket' to a peer.
 *     If, on the first read operation the 'socket' is
 *     unconnected then the 'socket' is connected to the
 *     peer (read data source).
 *
 * Usage:
 *
 *  1) Initialize lanIpBasic IP/UDP mini-stack:
 *
 *      lanIpSetup("<ip_adr>", "<net_msk", 0, 0);
 *
 *     The IP address must be a valid IP address
 *     that is otherwise unused on your LAN. Provide
 *     a string in 'dot' notation, e.g., "10.0.0.23".
 *
 *  2) Register the drvUdpSock driver:
 *
 *       rtems_io_register_driver(0, &drvUdpSock_ops, &drvUdpSocks_major);
 *
 *     Note that your RTEMS system/application must have enough
 *     driver slots configured.
 *
 *  3) Create device nodes; one for each port you want to
 *     use sockets on (udpSocks are only created when the
 *     device file is opened). The device minor number
 *     holds the UDP port number (of your box).
 *     E.g., create a device to be used by the GDB stub
 *     (listening on the official GDB port 2159):
 *
 *       drvUdpSockCreateDev("/dev/udp-gdb", 2159)
 *
 *     Note that the device name is completely arbitrary.
 *
 *  4) You now can transparently create sockets and read/write
 *     from them:
 *
 *       int sd = open("/dev/udp-xxx", O_RDWR);
 *       // may read w/o 'connecting'; the peer address/port is remembered
 *       // when we receive the first data
 *       read(sd, buf, NBYTES);
 *
 *     or setting a particular peer:
 *
 *       int sd = open("/dev/udp-xxx", O_RDWR);
 *       DrvUdpSockPeer peer;
 *
 *         peer.ipaddr = inet_addr("10.0.2.2");
 *         peer.port   = 1234;
 *         ioctl(sd, UDPSIOC_SETPEER, &peer);
 *
 *     you may probably want to use buffered I/O
 *
 *     FILE *f = fdopen(sd,"rw");
 *
 */

#include <rtems.h>
#include <rtems/io.h>

#include <sys/ioctl.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t	ipaddr; /* network   byte order */
	uint16_t	port;   /* **host**  byte order */
} DrvUdpSockPeer;

#define UDPSIOC_SETPEER	_IOW( 'u', 0, DrvUdpSockPeer )
#define UDPSIOC_GETPEER	_IOR( 'u', 0, DrvUdpSockPeer )

/*
 * Placeholder for major device number; to be registered with
 * rtems_io_register_driver()
 */
extern uint32_t drvUdpSock_major;

/*
 * Helper routine for creating device nodes
 *
 * NOTE: - lanIp stack must be initialized
 *       - driver must be registered
 *       - port # is in host byte order
 *
 */
int
drvUdpSockCreateDev(char *name, uint32_t port);

extern rtems_driver_address_table drvUdpSock_ops;

#ifdef __cplusplus
};
#endif

#endif
