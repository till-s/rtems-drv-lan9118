/* $Id$ */

/*
 * Simple driver providing access to 'udpSock' via the file
 * system (read/write).
 */

#include <rtems.h>
#include <rtems/io.h>
#include <rtems/libio.h>

#include <sys/errno.h>
#include <sys/ioctl.h>
#include <lanIpBasic.h>
#include <lanIpBasicSetup.h>
#include <string.h>
#include <stdlib.h>

#include <netinet/in.h>

#include <drvUdpSock.h>

typedef struct {
	DrvUdpSockPeer	peer;
	unsigned        flags;
	LanIpPacketRec  *pkt;
	unsigned short  avl;
	uint8_t         *off;
} DrvUdpSockRec, *DrvUdpSock;

/* use 'flags' field to hold:
 *  - socket descriptor.
 *  - 'is_connected' flag.
 */

#define SOCK_SD(x)	((x)&0xff)
#define SOCK_ISCONN (1<<28)
#define SOCK_NBLK   (1<<29) /* FIXME: support not implemented yet */

/* 
 * vvvvvvvvvvvvvvvvvv FOR REFERENCE vvvvvvvvvvvvvvvvvv
 *
 * The driver 'ops' are called through a double layer of
 * functions:
 *
 *   file-system  ops    (for IMFS these are in libfs/src/imfs/deviceio.c)
 *
 *   rtems_io_XXX calls  (in sapi/src/ioXXX.c)
 *
 * If driver node is on IMFS:
 *
 * Return values of driver ops (except _init, which is ignored)
 * are mapped into 'errno' values and the operation returns
 *
 * FAILURE:
 *   -1                            (with errno set)
 * SUCCESS:
 *   0                             (open, close)
 *   positive byte count           (read/write)
 *   value passed up by the driver (ioctl)
 *
rtems_assoc_t errno_assoc[] = {
    { "OK",                 RTEMS_SUCCESSFUL,                0 },
    { "BUSY",               RTEMS_RESOURCE_IN_USE,           EBUSY },
    { "INVALID NAME",       RTEMS_INVALID_NAME,              EINVAL },
    { "NOT IMPLEMENTED",    RTEMS_NOT_IMPLEMENTED,           ENOSYS },
    { "TIMEOUT",            RTEMS_TIMEOUT,                   ETIMEDOUT },
    { "NO MEMORY",          RTEMS_NO_MEMORY,                 ENOMEM },
    { "NO DEVICE",          RTEMS_UNSATISFIED,               ENODEV },
    { "INVALID NUMBER",     RTEMS_INVALID_NUMBER,            EBADF},
    { "NOT RESOURCE OWNER", RTEMS_NOT_OWNER_OF_RESOURCE,     EPERM},
    { "IO ERROR",           RTEMS_IO_ERROR,                  EIO},
    { 0, 0, 0 },
};

 * ^^^^^^^^^^^^^^^^^^ FOR REFERENCE ^^^^^^^^^^^^^^^^^^
 */

static rtems_status_code
drvUdpSock_open(
	rtems_device_major_number maj,
	rtems_device_minor_number min,
	void *v_a)
{
rtems_status_code              sc    = RTEMS_SUCCESSFUL;
rtems_libio_open_close_args_t  *argp = v_a;
int                            sd;
DrvUdpSock                     d;

/* libcsupport/include/rtems/libio.h:
typedef struct {
	rtems_libio_t          *iop;
	uint32_t                flags;
	uint32_t                mode;
} rtems_libio_open_close_args_t;
 */

	if ( (sd = udpSockCreate(min)) < 0 ) {
		switch ( sd ) {
			case -EADDRINUSE: return RTEMS_RESOURCE_IN_USE;
			case -ENOSPC:     return RTEMS_NO_MEMORY;
			case -EINVAL:     return RTEMS_INVALID_NUMBER;
			default: break;
		}
		/* There is no good code for 'unknown error' */
		sc = RTEMS_INVALID_NAME;
	} else if ( sd > SOCK_SD(-1) || !(d = calloc(1, sizeof(*d))) ) {
		udpSockDestroy(sd);
		sc = RTEMS_INTERNAL_ERROR;
	} else {
		d->flags         = sd; /* implies !'ISCONN' */
		argp->iop->data0 = (uint32_t)d;
	}

	return sc;
}

static rtems_status_code
drvUdpSock_close(
	rtems_device_major_number maj,
	rtems_device_minor_number min,
	void *v_a)
{
rtems_status_code              sc    = RTEMS_SUCCESSFUL;
rtems_libio_open_close_args_t  *argp = v_a;
DrvUdpSock                     d     = (DrvUdpSock)argp->iop->data0;

/* libcsupport/include/rtems/libio.h:
typedef struct {
	rtems_libio_t          *iop;
	uint32_t                flags;
	uint32_t                mode;
} rtems_libio_open_close_args_t;
 */

/* uint32_t argp->flags, argp->mode are 0 */

	switch ( udpSockDestroy( SOCK_SD(d->flags) ) ) {
		case -EBADF: return RTEMS_INVALID_NUMBER;
		default:     return RTEMS_INVALID_NAME;
		case 0:      break;
	}

	udpSockFreeBuf( d->pkt );
	free( d );
	argp->iop->data0 = 0;


	return sc;
}

static rtems_status_code
drvUdpSock_read(
	rtems_device_major_number maj,
	rtems_device_minor_number min,
	void *v_a)
{
rtems_status_code      sc    = RTEMS_SUCCESSFUL;
rtems_libio_rw_args_t  *argp = v_a;
DrvUdpSock             d     = (DrvUdpSock)argp->iop->data0;
uint32_t               n;

/* libcsupport/include/rtems/libio.h:
typedef struct {
	rtems_libio_t          *iop;
	off_t                   offset;
	char                   *buffer;
	uint32_t                count;
	uint32_t                flags;
	uint32_t                bytes_moved;
} rtems_libio_rw_args_t;
 */

/*
 * Here's what imfs/src/deviceio.c does:
 *
	args.iop         = iop;
	args.offset      = iop->offset;
	args.buffer      = buffer;
	args.count       = count;
	args.flags       = iop->flags;
	args.bytes_moved = 0; // to be set by driver
 */

	/* no leftovers from an old buffer ? */
	if ( ! d->avl ) {
     
		d->pkt = udpSockRecv( SOCK_SD(d->flags), (SOCK_NBLK & d->flags) ? 0 : -1 );

		if ( ! d->pkt ) {
			return (SOCK_NBLK & d->flags) ? RTEMS_TIMEOUT : RTEMS_INVALID_NAME ;
		}

		d->off = lpkt_udp_hdrs(d->pkt).pld;
		d->avl = ((unsigned short)ntohs(lpkt_udp(d->pkt).len)) - sizeof(UdpHeaderRec);

		/* Remember the sender if we are not connected. */
		if ( ! (SOCK_ISCONN & d->flags) ) {
			d->peer.ipaddr = lpkt_ip(d->pkt).src;
			d->peer.port   = (unsigned short)ntohs( lpkt_udp(d->pkt).sport );
		}
	}

	if ( (n = argp->count) > d->avl )
		n = d->avl;

	memcpy( argp->buffer, d->off, n );

	if ( (d->avl -= n) > 0 ) {
		d->off += n;
	} else {
		udpSockFreeBuf( d->pkt );
		d->pkt = 0;
		d->off = 0;
		d->avl = 0;
	}

	argp->bytes_moved = n;

	return sc;
}

static rtems_status_code
drvUdpSock_write(
	rtems_device_major_number maj,
	rtems_device_minor_number min,
	void *v_a)
{
rtems_status_code      sc    = RTEMS_SUCCESSFUL;
rtems_libio_rw_args_t  *argp = v_a;
DrvUdpSock             d     = (DrvUdpSock)argp->iop->data0;
int                    n;
/* libcsupport/include/rtems/libio.h:
typedef struct {
	rtems_libio_t          *iop;
	off_t                   offset;
	char                   *buffer;
	uint32_t                count;
	uint32_t                flags;
	uint32_t                bytes_moved;
} rtems_libio_rw_args_t;
 */

/*
 * Here's what imfs/src/deviceio.c does:
	args.iop         = iop;
	args.offset      = iop->offset;
	args.buffer      = (void *) buffer;
	args.count       = count;
	args.flags       = iop->flags;
	args.bytes_moved = 0; // to be set by driver
 */

	if ( !d->peer.ipaddr || !d->peer.port )
		return RTEMS_NOT_OWNER_OF_RESOURCE;

	n = argp->count > UDPPAYLOADSIZE ? UDPPAYLOADSIZE : argp->count;

	if ( SOCK_ISCONN & d->flags ) {
		n =	udpSockSend( SOCK_SD(d->flags), argp->buffer, n );
	} else {
		n = udpSockSendTo( SOCK_SD(d->flags), argp->buffer, n, d->peer.ipaddr, d->peer.port );
	}

	if ( n < 0 ) {
		switch (n) {
			case -ENOTCONN: return RTEMS_UNSATISFIED;
			default:        break;
		}
		sc = RTEMS_IO_ERROR;
	} else {
		argp->bytes_moved = n;
	}

	return sc;
}

static rtems_status_code
drvUdpSock_ioctl(
	rtems_device_major_number maj,
	rtems_device_minor_number min,
	void *v_a)
{
rtems_status_code         sc    = RTEMS_SUCCESSFUL;
rtems_libio_ioctl_args_t  *argp = v_a;
DrvUdpSock                d     = (DrvUdpSock)argp->iop->data0;
DrvUdpSockPeer            *peer;
int                       nbytes;
/* libcsupport/include/rtems/libio.h:
typedef struct {
	rtems_libio_t          *iop;
	uint32_t                command;
	void                   *buffer;
	uint32_t                ioctl_return;
} rtems_libio_ioctl_args_t;
 */

/*
 * Here's what imfs/src/deviceio.c does:
	args.iop     = iop;
	args.command = command;
	args.buffer  = buffer;
 */

	argp->ioctl_return = 0;

	switch ( argp->command ) {
		case UDPSIOC_SETPEER:
			if ( (SOCK_ISCONN & d->flags) ) {
				switch ( udpSockConnect( SOCK_SD(d->flags), 0, 0, 0 ) ) {
					case -ENOTCONN:
					case 0:
					break;

					default:
					return RTEMS_INVALID_NAME;
				}
				d->flags &= ~SOCK_ISCONN;
			}

			peer    = argp->buffer;

			/* allow NULL buffer; semantics are to disconnect */
			if ( peer ) {
				d->peer = *peer;

				if ( peer->ipaddr && peer->port ) {
					if ( 0 == udpSockConnect( SOCK_SD(d->flags), peer->ipaddr, peer->port, UDPSOCK_MCPASS ) ) {
						d->flags |= SOCK_ISCONN;
					} else {
						sc = RTEMS_IO_ERROR;
					}
				}
			} else {
				d->peer.ipaddr = (uint32_t)htonl(0);
				d->peer.port   = 0;
			}
		break;

		case UDPSIOC_GETPEER:
			peer = argp->buffer;
			if ( (SOCK_ISCONN & d->flags) ) {
				*peer = d->peer;
			} else {
				peer->ipaddr = 0;
				peer->port   = 0;
			}
		break;

		case FIONREAD:
			if ( (nbytes = udpSockNRead( SOCK_SD(d->flags) )) < 0 )
				sc = RTEMS_INVALID_NAME;
			*(int*)argp->buffer = nbytes + d->avl;
		break;

		default:	sc = RTEMS_INVALID_NAME;
		break;
	}

	/* On success, imfs/src/deviceio returns args.ioctl_return; */
	return sc;
}

/*
 * Placeholder for major device number; to be registered with
 * rtems_io_register_driver()
 */
uint32_t drvUdpSock_major = 0;

/*
 * Helper routine for creating device nodes
 *
 * NOTE: - lanIp stack must be initialized
 *       - driver must be registered
 *       - port number is in host byte order
 */
int
drvUdpSockCreateDev(char *name, uint32_t port)
{
	if ( !name || !port ) {
		return -EINVAL;
	}
	if ( !lanIpIf ) {
		/* IP stack must be initialized */
		return -ENODEV;
	}
	if ( !drvUdpSock_major ) {
		/* Driver not registered        */
		return -ENODEV;
	}
	if ( mknod(
			name,
			0666 | S_IFCHR,
			rtems_filesystem_make_dev_t( drvUdpSock_major, port ) ) )
		return errno ? -errno : -1;

	return 0;
}

/* Any of the entry points may be NULL (== 'RTEMS_SUCCESSFUL') */
rtems_driver_address_table drvUdpSock_ops = {
	initialization_entry:	0,
	open_entry:				drvUdpSock_open,
	close_entry:			drvUdpSock_close,
	read_entry:				drvUdpSock_read,
	write_entry:			drvUdpSock_write,
	control_entry:			drvUdpSock_ioctl,
};
