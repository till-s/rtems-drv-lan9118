#ifndef LAN_IP_BASIC_TEST_H
#define LAN_IP_BASIC_TEST_H

#include <rtems.h>
#include <lanIpBasic.h>
#include <stdint.h>

/* Note: the name of this file is historic and unfortunate... */

#ifdef __cplusplus
extern "C" {
#endif

/* Driver handle     */
extern void  *lanIpDrv;
/* Interface handle  */
IpBscIf       lanIpIf;
/* Udp Socket handle */
extern int	  lanIpUdpsd;

/* Take down the network stack */
void
lanIpTakedown();

/* Initialize and start the network stack
 * assign global driver and interface handles.
 *
 * 'ip'/'nmsk': IP address and netmask;
 * 'port'     : (optional); a UDP socket is 
 *              created (and assigned to lanIpUdpsd).
 *              Pass zero to skip socket creation.
 * 'enaddr'   : Ethernet address to use; may be
 *              NULL (driver should try to read
 *              from eeprom).
 *
 * RETURNS: 0 on success, nonzero on error.
 */

int
lanIpSetup(char *ip, char *nmsk, int port, uint8_t *enaddr);

/* bounce a UDP packet back to the caller
 *
 * (for testing only -- consult source for more information)
 */

int
udpSocketEcho(int sd, int raw, int idx, int timeout);

int
udpBouncer(int master, int raw, uint32_t dipaddr, uint16_t dport);

#ifdef __cplusplus
}
#endif

#endif
