/* $Id$ */

/* Parts of this code are derived from 'if_em.c' (c) Intel Corp. */

/* Author: Till Straumann <strauman@slac.stanford.edu>           */

/**************************************************************************

Copyright (c) 2001-2007, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#include <rtems.h>
#include <bsp.h>
#include <rtems/irq.h>
#include <rtems/pci.h>
#include <rtems/rtems_mii_ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <libcpu/byteorder.h>

#define  lldrv_pvt e1k_private
#include <gnreth_lldrv.h>

/* 
 * If the 'em' driver library was built w/o 82542 support
 * then there are no access methods for the 82542 and
 * we'll get a link error if not defining NO_82542_SUPPORT.
 *
 * If the library does have 82542 support then defining
 * this is still OK (but you have to undefine to get
 * this driver to work on a 82542).
 */
#define NO_82542_SUPPORT

#include "e1000_api.h"

#define errpr(str...) fprintf(stderr,str)
#define DBGPR(str...) do { if ( drv_e1k_debug ) fprintf(stderr,str); } while (0)

#define DEBUG_LL

#ifndef AUTO_ALL_MODES
#define AUTO_ALL_MODES 0
#endif

#ifndef SPEED_MODE_BIT
#define SPEED_MODE_BIT (1<<21)
#endif

#ifndef PCI_MULTI_FUNC
#define PCI_MULTI_FUNC 0x80
#endif

#ifdef BSP_SHARED_HANDLER_SUPPORT
#define IRQINST(x) BSP_install_rtems_shared_irq_handler(x)
#define IRQREMV(x) BSP_remove_rtems_irq_handler(x)
#else
#if 1
#include <bsp/bspExt.h>
#define IRQINST(x) (!bspExtInstallSharedISR((x)->name, (x)->hdl, (x)->handle, 0))
#define IRQREMV(x) (!bspExtRemoveSharedISR((x)->name, (x)->hdl, (x)->handle))
#else
#define IRQINST(x) BSP_install_rtems_irq_handler(x)
#define IRQREMV(x) BSP_remove_rtems_irq_handler(x)
#endif
#endif

/* Align ring to max. cache line */
#define RINGALIGNMENT 128
#define RINGALIGN(x)  ((((uintptr_t)(x)) + (RINGALIGNMENT-1)) & ~(RINGALIGNMENT-1))

#ifndef BSP_PCI2LOCAL_ADDR
#define BSP_PCI2LOCAL_ADDR(x) ((uint32_t)(x))
#endif

#ifndef BSP_LOCAL2PCI_ADDR
#define BSP_LOCAL2PCI_ADDR(x) ((uint32_t)(x))
#endif

#define BUF_OFF  2
#define BUF_ALGN 4


/* A legacy descriptor */
struct e1000_leg_desc {
	uint32_t	buf_lo;
	uint32_t    buf_hi;
	uint32_t    len;
	uint32_t    sta;
};

#define E1000_RXD_ERR_ANY 0xff00

struct ring {
	void                  *mem;
	struct e1000_leg_desc *dsc;
	int                    sz;
	int                    hd, tl;
	int                    av;
};

struct e1k_private {
	struct e1000_hw          hw;
	struct ring              tx_ring;
	struct ring              rx_ring;
	uint32_t                 txd_cmd;
	int                      num_rx_ring;
	int                      num_tx_ring;
	int                      unit;
	void                     (*isr)(void *isr_arg);
	void                     *isr_arg;
	void                     (*cleanup_txbuf)(void *usr_p, void *cleanup_txbuf_arg, int error_on_tx_occurred);
	void                    *cleanup_txbuf_arg;
	void                    *(*alloc_rxbuf)(int *p_size, uintptr_t *p_data_addr);
	void                     (*consume_rxbuf)(void *usr_buf, void *consume_rxbuf_arg, int len);
	void                    *consume_rxbuf_arg;
	uint32_t                 pending;
	uint32_t                 irq_msk;
	struct {
		uint32_t             irqs;
		uint32_t             irqs_spr;
		uint32_t             txpkt;
		uint32_t             rxpkt;
	}                        stats;
	rtems_irq_connect_data   irq;
	int                      link_active;
	uint16_t                 link_speed, link_duplex;
	uint16_t                 *mc_refcnt;
	uint32_t                 mc_num_hashes;
};

int      drv_e1k_debug = 1;

/*
 * Value is calculated as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns)
 */

#define MAX_INTS_PER_SEC	8000
#define DEFAULT_ITR	     1000000000/(MAX_INTS_PER_SEC * 256)

/* Due to the way our driver works:
 *  1) ISR disables interrupts, posts signal to task.
 *  2) Task does work as long as there is work to do
 *  3) Task re-enables interrupts
 * there is 'automatic' moderation of interrupts.
 */

/* not exactly microseconds but who cares ... */
uint32_t drv_e1k_tx_int_delay_us     = 10;
uint32_t drv_e1k_tx_abs_int_delay_us = 40;
uint32_t drv_e1k_rx_int_delay_us     =  0;
uint32_t drv_e1k_rx_abs_int_delay_us =  0;
uint32_t drv_e1k_min_int_period_us   =  2;

static void
cring(struct ring *r, int n)
{
	/* ring size must always be multiple of 8 */
	n = (n + 7) & ~7;
	r->sz = n;
	n *= sizeof(*r->dsc);
	if ( n ) {
		r->mem = malloc(RINGALIGNMENT + n);
		r->dsc = (struct e1000_leg_desc*)RINGALIGN(r->mem);
		memset(r->dsc, 0, n);
	} else {
		r->mem = 0;
		r->dsc = 0;
	}
	r->hd = r->tl = 0;
}


static void
hwctrl(struct e1000_hw *hw, int acquire)
{
uint32_t ctrl_ext, swsm;
	switch ( hw->mac.type ) {
		case e1000_82573:
			swsm = E1000_READ_REG(hw, E1000_SWSM);
			if ( acquire )
				swsm |=   E1000_SWSM_DRV_LOAD;
			else
				swsm &= ~ E1000_SWSM_DRV_LOAD;
			E1000_WRITE_REG(hw, E1000_SWSM, swsm);
		break;

		case e1000_82571:
		case e1000_82572:
		case e1000_80003es2lan:
		case e1000_ich8lan:
		case e1000_ich9lan:
			ctrl_ext = E1000_READ_REG(hw, E1000_CTRL_EXT);
			if ( acquire )
				ctrl_ext |=   E1000_CTRL_EXT_DRV_LOAD;
			else
				ctrl_ext &= ~ E1000_CTRL_EXT_DRV_LOAD;
			E1000_WRITE_REG(hw, E1000_CTRL_EXT, ctrl_ext);
		break;

		default:
		break;
	}
}

static int
hwinit_ll(struct e1000_hw *hw)
{
uint16_t phy_tmp = 0;

	e1000_reset_hw(hw);

	switch ( hw->mac.type ) {
		case e1000_82573:
		case e1000_ich8lan:
		case e1000_ich9lan:
			if ( e1000_check_mng_mode(hw) ) {
				hwctrl(hw, 1);
			}
				
		default:
		break;
	}

	switch ( hw->mac.type ) {
		case e1000_82571:
		case e1000_82572:
			e1000_read_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, &phy_tmp);
			phy_tmp &= ~IGP02E1000_PM_SPD;
			e1000_write_phy_reg(hw, IGP02E1000_PHY_POWER_MGMT, phy_tmp);
		break;

		default:
		break;
	}

	hw->mac.fc = e1000_fc_none;

	if ( e1000_init_hw(hw) < 0 ) {
		errpr("e1000_init_hw() failed\n");
		return -1;
	}

	e1000_check_for_link(hw);

	return 0;
}

static void
update_link_status(struct e1k_private *ad)
{
struct e1000_hw *hw = &ad->hw;
int      tarc0;

	/* Don't bother to do any unnecessary work */
	if ( !drv_e1k_debug && e1000_82571 != hw->mac.type && e1000_82572 != hw->mac.type )
		return;

	if ( (E1000_STATUS_LU & E1000_READ_REG(hw, E1000_STATUS)) ) {
		if ( ! ad->link_active ) {
			e1000_get_speed_and_duplex(hw, &ad->link_speed, &ad->link_duplex);
			if ( SPEED_1000 != ad->link_speed && 
		     	(e1000_82571 == hw->mac.type && e1000_82572 == hw->mac.type) ) {
				tarc0  = E1000_READ_REG(hw, E1000_TARC0);	
				tarc0 &= ~SPEED_MODE_BIT;
				E1000_WRITE_REG(hw, E1000_TARC0, tarc0);	
			}
			ad->link_active = 1;
		}
		DBGPR("Link is up %u Mbps %s\n",
		      ad->link_speed, 
              FULL_DUPLEX == ad->link_duplex ? "FULL" : "HALF");
	} else {
		if ( ad->link_active ) {
			ad->link_active = 0;
			ad->link_duplex = 0;
			ad->link_speed  = 0;
		}
		DBGPR("Link Down\n");
	}
}

static void
put_rxb(struct e1k_private *ad, uintptr_t buf)
{
struct e1000_leg_desc *d;

	d = &ad->rx_ring.dsc[ad->rx_ring.tl];
	st_le32(&d->buf_lo, buf);
	/* make sure 'DD' (desc.-done) is clear */
	st_le32(&d->sta, 0);
	if ( ++ad->rx_ring.tl == ad->rx_ring.sz )
		ad->rx_ring.tl = 0;
	/* write tail to HW */
	E1000_WRITE_REG(&ad->hw, E1000_RDT, ad->rx_ring.tl);
}

void
dtxring(struct e1k_private *ad)
{
int                    i;
struct e1000_leg_desc *d;

	printf("TX Ring head: %i, tail :%i, size: %i\n", ad->tx_ring.hd, ad->tx_ring.tl, ad->tx_ring.sz);
	printf("TX ring area: %p, avail :%i\n", ad->tx_ring.dsc, ad->tx_ring.av);

	for ( i = 0, d = ad->tx_ring.dsc; i<ad->tx_ring.sz; i++, d++ ) {
		uint32_t buf = ld_le32(&d->buf_lo);
		uint32_t cmd = ld_le32(&d->len);
		uint32_t sta = ld_le32(&d->sta);
		printf("#%i: Buf: 0x%08"PRIx32" Len: 0x%04"PRIx16" Cmd: 0x%02"PRIx8" Sta: 0x%02"PRIx8"\n",
				i, buf, (uint16_t)cmd, (uint8_t)(cmd>>24), (uint8_t)(sta & 0xf) );
	}
}

void
drxring(struct e1k_private *ad)
{
int                    i;
struct e1000_leg_desc *d;

	printf("RX Ring head: %i, tail :%i, size: %i\n", ad->rx_ring.hd, ad->rx_ring.tl, ad->rx_ring.sz);
	printf("RX ring area: %p, avail :%i\n", ad->rx_ring.dsc, ad->rx_ring.av);

	for ( i = 0, d = ad->rx_ring.dsc; i<ad->rx_ring.sz; i++, d++ ) {
		uint32_t buf = ld_le32(&d->buf_lo);
		uint32_t len = ld_le32(&d->len);
		uint32_t sta = ld_le32(&d->sta);
		printf("#%i: Buf: 0x%08"PRIx32" Len: 0x%04"PRIx16" Csum: 0x%04"PRIx16" Sta: 0x%02"PRIx8" Errs: 0x%02"PRIx8"\n",
				i, buf, (uint16_t)len, (uint16_t)(len>>16), (uint8_t)(sta), (uint8_t)(sta>>8) );
	}
}

void
drv_e1k_dump_stats(struct e1k_private *ad, FILE *f)
{
	if ( !f )
		f = stdout;

	fprintf(f,
		"E1K # IRQS: %9"PRIu32" (spurious %9"PRIu32")\n    RX-FRM: %9"PRIu32",  TX-FRM: %9"PRIu32"\n",
		ad->stats.irqs,
		ad->stats.irqs_spr,
		ad->stats.rxpkt,
		ad->stats.txpkt
	);
}

uint32_t
drv_e1k_disable_irqs(struct e1k_private *ad, uint32_t mask)
{
uint32_t rval = E1000_READ_REG(&ad->hw, E1000_IMS);
	E1000_WRITE_REG(&ad->hw, E1000_IMC, mask);
	return rval;
}

void
drv_e1k_enable_irqs(struct e1k_private *ad, uint32_t mask)
{
	E1000_WRITE_REG(&ad->hw, E1000_IMS, ad->irq_msk & mask);
}

static void noop(const rtems_irq_connect_data *unused)  {           }
static int  noop1(const rtems_irq_connect_data *unused) { return -1;}

static struct IpBscLLDrv_ lldrv_e1k;

static void e1k_isr(void *arg)
{
struct e1k_private  *ad = arg;
uint32_t        icr;

	icr = E1000_READ_REG(&ad->hw, E1000_ICR);

	if (   0 == icr
		|| (ad->hw.mac.type >= e1000_82571 &&
		    ! (E1000_ICR_INT_ASSERTED & icr) ) ) {
		ad->stats.irqs_spr++;
		return;
	}
		
	ad->stats.irqs++;
	ad->pending |= icr & ad->irq_msk;

#ifdef DEBUG_LL
	if ( drv_e1k_debug > 10 )
		printk("IRQ (0x%08x)\n",icr);
#endif

	ad->isr(ad->isr_arg);
}

/* e1000_mta_clr() is modelled after e1000_mta_set() which is implemented
 * by the Intel driver (which unfortunately lacks the functionality to
 * clear individual bits in the multicast filter).
 * As of the Intel FreeBSD driver release present in FreeBSD as of 20070724
 * only e1000_mta_set_82543() did anything special. All other chips used
 * e1000_mta_set_generic().
 *
 * This routine checks the chip type and does what e1000_mta_clr_82543() or
 * e1000_mta_clr_generic() would do, respectively if they existed.
 *
 * For future (or more recent) versions of the Intel driver this routine
 * might require to include support for more chips...
 */
static void
e1000_mta_clr(struct e1000_hw *hw, u32 hash_value)
{
	u32 hash_bit, hash_reg, mta, temp;

	hash_reg = (hash_value >> 5);

	/* If we are on an 82544 and we are trying to write an odd offset
	 * in the MTA, save off the previous entry before writing and
	 * restore the old value after writing.
	 */
	if ((hw->mac.type == e1000_82544) && (hash_reg & 1)) {
		hash_reg &= (hw->mac.mta_reg_count - 1);
		hash_bit = hash_value & 0x1F;
		mta = E1000_READ_REG_ARRAY(hw, E1000_MTA, hash_reg);
		mta &= ~(1 << hash_bit);
		temp = E1000_READ_REG_ARRAY(hw, E1000_MTA, hash_reg - 1);

		E1000_WRITE_REG_ARRAY(hw, E1000_MTA, hash_reg, mta);
		E1000_WRITE_FLUSH(hw);
		E1000_WRITE_REG_ARRAY(hw, E1000_MTA, hash_reg - 1, temp);
		E1000_WRITE_FLUSH(hw);
	} else {
		/* The MTA is a register array of 32-bit registers. It is
		 * treated like an array of (32*mta_reg_count) bits.  We want to
		 * set bit BitArray[hash_value]. So we figure out what register
		 * the bit is in, read it, OR in the new bit, then write
		 * back the new value.  The (hw->mac.mta_reg_count - 1) serves as a
		 * mask to bits 31:5 of the hash value which gives us the
		 * register we're modifying.  The hash bit within that register
		 * is determined by the lower 5 bits of the hash value.
		 */
		hash_reg = (hash_value >> 5) & (hw->mac.mta_reg_count - 1);
		hash_bit = hash_value & 0x1F;

		mta = E1000_READ_REG_ARRAY(hw, E1000_MTA, hash_reg);

		mta &= ~(1 << hash_bit);

		E1000_WRITE_REG_ARRAY(hw, E1000_MTA, hash_reg, mta);
		E1000_WRITE_FLUSH(hw);
	}
}

void
drv_e1k_mcast_filter_clear(struct e1k_private *ad)
{
struct e1000_hw *hw = &ad->hw;
int              i;

	for ( i = 0; i < hw->mac.mta_reg_count; i++ ) {
		E1000_WRITE_REG_ARRAY(hw, E1000_MTA, i, 0);
		E1000_WRITE_FLUSH(hw);
	}

	for ( i = 0; i < ad->mc_num_hashes; i++ ) {
		ad->mc_refcnt[i] = 0;
	}
}

static int
mc_addr_valid(uint8_t *enaddr)
{
static const char bcst[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	return ( (0x01 & enaddr[0]) && memcmp(enaddr, bcst, sizeof(bcst)) );
}

void
drv_e1k_mcast_filter_accept_add(struct e1k_private *ad, uint8_t *enaddr)
{
struct e1000_hw *hw = &ad->hw;
uint32_t         hash;

	if ( ! mc_addr_valid(enaddr) )
		return;

	hash = e1000_hash_mc_addr(hw, enaddr);

	if ( 0 == ad->mc_refcnt[hash]++ ) {
		e1000_mta_set(hw, hash);
	}
}

void
drv_e1k_mcast_filter_accept_del(struct e1k_private *ad, uint8_t *enaddr)
{
struct e1000_hw *hw = &ad->hw;
uint32_t         hash;

	if ( ! mc_addr_valid(enaddr) )
		return;

	hash = e1000_hash_mc_addr(hw, enaddr);

	if ( ad->mc_refcnt[hash] > 0 && 0 == --ad->mc_refcnt[hash] ) {
		e1000_mta_clr(hw, hash);
	}
}



static int
hwmatch(struct e1000_hw *hw, int b, int s, int f)
{
uint16_t val;
	pci_read_config_word(b, s, f, PCI_VENDOR_ID, &val);
	if ( PCI_VENDOR_ID_INTEL != val )
		return -1;
	pci_read_config_word(b, s, f, PCI_DEVICE_ID, &hw->device_id);
	return e1000_set_mac_type(hw);
}

static int
pci_scan(struct e1000_hw *hw, int *p_b, int *p_s, int *p_f, int unit)
{
int      u,b,s,f,fmax;
uint32_t val,cmp;

	/* convert 1-based unit into 0-based; */

	if ( unit > 256 )
		unit &= 255;
	else
		unit--;

	/* i386 'pci_bus_count()' is incorrect, returns 'max bus' instead of
     * number of busses. Just add one...
	 */
	for ( u = b = 0; b < pci_bus_count() + 1; b++ ) {
		for ( s = 0; s < PCI_MAX_DEVICES; s++ ) {
			f = fmax = 0;
			do {
				pci_read_config_dword(b, s, f, PCI_VENDOR_ID, &val);
				if ( 0xffffffff != val ) {
					if ( 0 == hwmatch( hw, b, s, f ) ) {
						if ( unit > 256 ) {
							/* they gave us a PCI triple (0:0:0 is usually the host bridge) */
							cmp = ((b<<8) | (s<<3) | f) << 8;
						} else {
							cmp = u;
						}

						if ( unit == cmp ) {
							*p_b = b;
							*p_s = s;
							*p_f = f;
							return u;
						}
						u++;
					}
					if ( 0 == f ) {
						uint8_t hh;
						pci_read_config_byte(b, s, f, PCI_HEADER_TYPE, &hh);
						if (PCI_MULTI_FUNC & hh) {
							fmax = PCI_MAX_FUNCTIONS;
						}
					}
				}
			} while ( ++f < fmax );
		}
	}
	return -1;
}

static unsigned long
find_io_bar(int b, int s, int f)
{
uint32_t val;
unsigned rid;

	/* Find our IO BAR */
	for ( rid = PCI_BASE_ADDRESS_0 ; rid < PCI_CARDBUS_CIS; ) {
		pci_read_config_dword(b, s, f, rid, &val);
		if ( PCI_BASE_ADDRESS_SPACE_IO & val ) {
			return val;
		}
		rid += 4;
		/* check for 64bit BAR */
		if ( PCI_BASE_ADDRESS_MEM_TYPE_64 == (PCI_BASE_ADDRESS_MEM_TYPE_MASK & val) )
			rid += 4;
	}
	return 0;
}

struct e1k_private *
drv_e1k_setup(
	int      unit,
	void     (*isr)(void *isr_arg),
	void     *isr_arg,
	void     (*cleanup_txbuf)(void *, void *, int),
	void    *cleanup_txbuf_arg,
	void    *(*alloc_rxbuf)(int *, uintptr_t *),
	void     (*consume_rxbuf)(void *, void*, int),
	void    *consume_rxbuf_arg,
	int     rx_ring_size,
	int     tx_ring_size,
	int     irq_mask
)
{
struct e1k_private        *ad;
uint32_t               val;
struct e1000_hw       *hw;
int                    b,s,f;
uint8_t                line;

	if ( ! isr && irq_mask ) {
		errpr("Need an ISR if irq_mask != 0\n");
		return 0;
	}

	ad = calloc(1, sizeof(*ad));
	hw = &ad->hw;

	if ( (ad->unit = pci_scan(hw, &b, &s, &f, unit)) ) {
		errpr("No compatible device found\n");
		free(ad);
		return 0;	
	}

	ad->isr               = isr;
	ad->isr_arg           = isr_arg;

	ad->cleanup_txbuf     = cleanup_txbuf;
	ad->cleanup_txbuf_arg = cleanup_txbuf_arg;
	ad->alloc_rxbuf       = alloc_rxbuf;
	ad->consume_rxbuf     = consume_rxbuf;
	ad->consume_rxbuf_arg = consume_rxbuf_arg;

	pci_read_config_word(b, s, f, PCI_COMMAND, &hw->bus.pci_cmd_word);

	if ( (hw->bus.pci_cmd_word & (PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY)) 
        != (PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY) ) {
		errpr("PCI Memory Access + Bus Master not both enabled; doing so now\n");
		hw->bus.pci_cmd_word |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
		pci_write_config_word(b, s, f, PCI_COMMAND, hw->bus.pci_cmd_word);
	}
	pci_read_config_word(b, s, f, PCI_VENDOR_ID, &hw->vendor_id);
	pci_read_config_word(b, s, f, PCI_DEVICE_ID, &hw->device_id);
	pci_read_config_byte(b, s, f, PCI_REVISION_ID, &hw->revision_id);
	pci_read_config_word(b, s, f, PCI_SUBSYSTEM_VENDOR_ID, &hw->subsystem_vendor_id);
	pci_read_config_word(b, s, f, PCI_SUBSYSTEM_ID, &hw->subsystem_device_id);
	if ( e1000_set_mac_type(hw) ) {
		errpr("e1000_hw_set_mac_type() failure\n");
		goto bail;
	}

	pci_read_config_dword(b, s, f, PCI_BASE_ADDRESS_0, &val);
	if ( (PCI_BASE_ADDRESS_SPACE_IO == (val & PCI_BASE_ADDRESS_SPACE)) ) {
		errpr("IO space not supported (should be easy to add)");
		goto bail;
	}
	val &= PCI_BASE_ADDRESS_MEM_MASK;

	if ( 0 == val ) {
		errpr("Memory base address (from PCI config space) == 0; something's fishy!\n");
		goto bail;
	}

	val = BSP_PCI2LOCAL_ADDR(val);

	hw->hw_addr = (uint8_t*)val;

	if ( hw->mac.type >= e1000_82543 && hw->mac.type < e1000_82571 ) {
			if ( (hw->io_base = find_io_bar(b,s,f)) ) {
				hw->io_base &= PCI_BASE_ADDRESS_IO_MASK;
			} else {
				errpr("Unable to locate IO BAR\n");
				goto bail;
			}
	} else {
		hw->io_base = 0;
	}

	if ( e1000_ich8lan == hw->mac.type || e1000_ich9lan == hw->mac.type ) {
		pci_read_config_dword(b, s, f, PCI_BASE_ADDRESS_1, &val);
		val &= PCI_BASE_ADDRESS_MEM_MASK;
		val  = BSP_PCI2LOCAL_ADDR(val);
		hw->flash_address = (uint8_t*)val;
	} else {
		hw->flash_address = 0;
	}

	if ( e1000_setup_init_funcs(hw, TRUE) ) {
		errpr("e1000_setup_init_funcs() failed\n");
		goto bail;
	}

	/* mta_reg_count is set by e1000_setup_init_funcs */
	ad->mc_num_hashes = hw->mac.mta_reg_count * 32;
	ad->mc_refcnt = calloc(sizeof(*ad->mc_refcnt), ad->mc_num_hashes);

	if ( 0 == ad->mc_refcnt ) {
		errpr("Unable to allocate memory for MC ref. counters\n");
		goto bail;
	}
	

	e1000_get_bus_info(hw);

	hw->mac.autoneg            = TRUE;
	hw->phy.wait_for_link      = FALSE;
	hw->phy.autoneg_advertised = ADVERTISE_10_HALF |
	                             ADVERTISE_10_FULL |
	                             ADVERTISE_100_HALF |
	                             ADVERTISE_100_FULL |
	                             ADVERTISE_1000_FULL;

	e1000_init_script_state_82541(hw, TRUE);
	e1000_set_tbi_compatibility_82543(hw, TRUE);

	if ( e1000_media_type_copper == hw->media_type ) {
		hw->phy.mdix    = AUTO_ALL_MODES;
		hw->phy.disable_polarity_correction = FALSE;
		hw->phy.ms_type = e1000_ms_hw_default;
	}

	hw->mac.max_frame_size = 1520;
	hw->mac.min_frame_size = 64;

	hw->mac.report_tx_early = 1;

	cring(&ad->tx_ring, (ad->num_tx_ring = tx_ring_size));
	/* available slots == empty slots */
	ad->tx_ring.av = ad->tx_ring.sz;

	cring(&ad->rx_ring, (ad->num_rx_ring = rx_ring_size));
	/* available slots == filled slots */
	ad->rx_ring.av = 0;

	if ( e1000_validate_nvm_checksum(hw) < 0 ) {
		if ( e1000_validate_nvm_checksum(hw) < 0 ) {
			errpr("EEPROM Checksum invalid\n");
			goto bail;
		}
	}

	if ( e1000_read_part_num(hw, &val) < 0 ) {
		errpr("e1000_read_part_num() failed; EEPROM read error\n");
		goto bail;
	}

	DBGPR("Part num: 0x%08"PRIx32"\n", val);

	if ( hwinit_ll(hw) ) {
		errpr("Unable to initialize hardware\n");
		goto bail;
	}

	if ( e1000_read_mac_addr(hw) < 0 ) {
		errpr("e1000_read_mac_addr() failed (EEPROM read error)\n");
		goto bail;	
	}

	hw->mac.get_link_status = 1;
	update_link_status(ad);

	if ( e1000_check_reset_block(hw) )
		errpr("PHY reset is blocked due to SOL/IDER session\n");

	if ( e1000_enable_mng_pass_thru(hw) ) {
		errpr("We have to control mng hardware but this is not implemented\n");
		goto bail;
	}

	if ( irq_mask ) {
		pci_read_config_byte(b, s, f, PCI_INTERRUPT_LINE, &line);
		DBGPR("Trying to hook IRQ #0x%x\n", line);

		ad->irq.name   = line;
		ad->irq.hdl    = e1k_isr;
		ad->irq.handle = ad; 
		ad->irq.on     = noop;
		ad->irq.off    = noop;
		ad->irq.isOn   = noop1;
		if ( ! IRQINST( &ad->irq ) ) {
			errpr("Hooking IRQ #0x%x failed\n", line);
			ad->irq.hdl = 0;
			goto bail;
		}
#undef IRQINST
	}

	ad->irq_msk = irq_mask;

	return ad;

bail:
	hwctrl(hw, 0);
	e1000_remove_device(hw);

	free(ad->tx_ring.mem);
	free(ad->rx_ring.mem);
	free(ad->mc_refcnt);

	if ( ad->irq.hdl ) {
		(void)IRQREMV( &ad->irq );
	}

	free(ad);
	
	return 0;
}

#define GIFMEDIA 0

int
drv_e1k_media_ioctl(struct e1k_private *ad, int cmd, int *pmedia)
{

	if ( GIFMEDIA != cmd ) {
		errpr("drv_e1k_media_ioctl(): SIFMEDIA not supported\n");
		return -1;
	}

	e1000_check_for_link(&ad->hw);
	update_link_status(ad);

	*pmedia = IFM_AVALID | IFM_ETHER;

	if ( ad->link_active ) {
		*pmedia |= IFM_LINK_OK;
	}

	if ((ad->hw.media_type == e1000_media_type_fiber) ||
	    (ad->hw.media_type == e1000_media_type_internal_serdes)) {
		if (ad->hw.mac.type == e1000_82545) {
			*pmedia |= IFM_1000_LX;
		} else {
			*pmedia |= IFM_1000_SX;
		}
		*pmedia |= IFM_FDX;
	} else {
		switch (ad->link_speed) {
		case SPEED_10:
			*pmedia |= IFM_10_T;
			break;
		case SPEED_100:
			*pmedia |= IFM_100_TX;
			break;
		case SPEED_1000:
			*pmedia |= IFM_1000_T;
			break;
		}
		*pmedia |= (FULL_DUPLEX == ad->link_duplex ? IFM_FDX : IFM_HDX);
	}
	return 0;
}

int
drv_e1k_ack_link_chg(struct e1k_private *ad, int *pmedia)
{
	return drv_e1k_media_ioctl(ad, GIFMEDIA, pmedia);
}

static void
txrm(struct e1k_private *ad, uint32_t sta)
{
uint32_t buf;

	buf = ld_le32(&ad->tx_ring.dsc[ad->tx_ring.hd].buf_lo);
	ad->cleanup_txbuf(
		(void*)(BSP_PCI2LOCAL_ADDR(buf) & ~(BUF_ALGN-1)),
		ad->cleanup_txbuf_arg,
		(sta & (E1000_TXD_STAT_LC | E1000_TXD_STAT_EC)) ? 1 : 0
	);
	if ( ++ad->tx_ring.hd == ad->tx_ring.sz )
		ad->tx_ring.hd = 0;
	ad->tx_ring.av++;
}

static int
txrpl(struct e1k_private *ad)
{
uint32_t sta;

	sta = ld_le32(&ad->tx_ring.dsc[ad->tx_ring.hd].sta);
	if ( ! (E1000_TXD_STAT_DD & sta) ) {
		/* still busy */
		return -1;
	}
	ad->stats.txpkt++;

	txrm(ad, sta);

	return 0;
}

static void
free_tx_structs(struct e1k_private *ad)
{
	while ( ad->tx_ring.av < ad->tx_ring.sz ) {
		txrm(ad, -1);
	}
}

static void
free_rx_structs(struct e1k_private *ad)
{
int             i;
	for ( i = ad->rx_ring.hd; ad->rx_ring.av; ad->rx_ring.av-- ) {
		void *b = (void*)(ld_le32(&ad->rx_ring.dsc[i].buf_lo) - BUF_OFF);
		ad->consume_rxbuf(b, ad->consume_rxbuf_arg, 0);
		if ( ++i == ad->rx_ring.sz )
			i = 0;
	}
}

int
drv_e1k_swipe_tx(struct e1k_private *ad)
{
int      org = ad->tx_ring.av;

	while ( ad->tx_ring.av < ad->tx_ring.sz 
			&& 0 == txrpl(ad) )
		/* do nothing else */;

	return ad->tx_ring.av - org;
}

int
drv_e1k_swipe_rx(struct e1k_private *ad)
{
int                    rval = 0;
struct e1000_leg_desc  *d;
uint32_t               sta;
void                   *nbuf;
uintptr_t              baddr, obaddr;
int                    sz;
int                    len;
int                    err;

	while ( 1 ) {

		d = &ad->rx_ring.dsc[ad->rx_ring.hd];

		if ( ! ( (sta = ld_le32( &d->sta )) & E1000_RXD_STAT_DD ) ) {
			break;
		}

		/* Here we have a bit of a problem. The chip assumes always a
		 * buffer size of 2k (which is what alloc_rxbuf hopefully
         * gets us) but we want to start storing at an 
		 * offset of 2 bytes so that everything is aligned nicely.
		 * This means that the chip theoretically could write
		 * beyond the buffer :-(.
		 * We just hope that we're protected by RCTL.LPE == 0 which
		 * should reject packets longer than 1522 bytes.
		 */

		obaddr = ld_le32( &d->buf_lo );

		err    = E1000_RXD_ERR_ANY & sta;
			
		if ( err || ! (nbuf = ad->alloc_rxbuf(&sz, &baddr)) ) {
			/* drop */
			baddr  = obaddr;
			obaddr = 0;
			len    = err ? -1 : 0;
		} else {
#if 0 /* Do paranoia test when initially filling the ring */
			unsigned diff = (uintptr_t)baddr  - (uintptr_t)nbuf;
			if ( 2048 != sz || diff != BUF_OFF )
				errpr("FATAL Error: got size %u, offset %u\n", sz, diff);
#endif
			baddr  += BUF_OFF;
			obaddr -= BUF_OFF;
			/* strip CRC from len */
			len     = (ld_le32( &d->len ) & 0xffff) - 4;
		}

		if ( !err )
			ad->stats.rxpkt++;

		/* advance head pointer */
		if ( ++ad->rx_ring.hd == ad->rx_ring.sz )
			ad->rx_ring.hd = 0;

		put_rxb(ad, baddr);

#ifdef DEBUG_LL
		if ( drv_e1k_debug > 10 ) {
			int j;
			printf("Received:\n");
			for ( j=0; j<len+2; ) {
				printf("%02X ", ((unsigned char*)obaddr)[j]);
				if ( ++j % 16 == 0 )
					printf("\n");
			}
			printf("\n");
		}
#endif

		ad->consume_rxbuf((void*)obaddr, ad->consume_rxbuf_arg, len);

		rval++;
	}
	return rval;
}

int
drv_e1k_send_buf(struct e1k_private *ad, void *p_usr, void *buf, int len)
{
struct   e1000_leg_desc *d;

	/* We don't want to maintain a shadow ring of user-buffer pointers.
     * This is a specialized driver and we assume that the user buffer
     * and the packet header are always 2 bytes apart...
	 */
	if ( (uintptr_t)buf - (uintptr_t)p_usr >= BUF_ALGN ) {
		errpr("FATAL: user ptr and buffer > %u-bytes apart!\n", BUF_ALGN-1);
		return -1;
	}

	/* Seems (at least on qemu's 82540) head must not equal tail if
     * ring is full...
     */
	if ( ad->tx_ring.av < 2 && txrpl(ad) )
		return -1;

	d = ad->tx_ring.dsc + ad->tx_ring.tl;

	st_le32(&d->buf_lo,  BSP_LOCAL2PCI_ADDR(buf));
	st_le32(&d->len, ad->txd_cmd | len );

	ad->tx_ring.av--;

	if ( ++ad->tx_ring.tl == ad->tx_ring.sz )
		ad->tx_ring.tl = 0;

	E1000_WRITE_REG(&ad->hw, E1000_TDT, ad->tx_ring.tl);

	return len;
}

void
drv_e1k_read_eaddr(struct e1k_private *ad, unsigned char *eaddr)
{
	memcpy(eaddr, ad->hw.mac.addr, 6);
}

static void
txinit(struct e1000_hw *hw)
{
struct e1k_private *ad = (struct e1k_private *)hw;
uint32_t       tipg, tarc, tctl;

	/* setup TX ring */
	E1000_WRITE_REG(hw, E1000_TDLEN, ad->tx_ring.sz * sizeof(struct e1000_leg_desc));
	E1000_WRITE_REG(hw, E1000_TDBAH, 0);
	E1000_WRITE_REG(hw, E1000_TDBAL, BSP_LOCAL2PCI_ADDR(ad->tx_ring.dsc));

	/* head + tail */
	E1000_WRITE_REG(hw, E1000_TDT, 0);
	E1000_WRITE_REG(hw, E1000_TDH, 0);
	ad->tx_ring.hd = ad->tx_ring.tl = 0;

	/* Set the default values for the Tx Inter Packet Gap timer */
	switch (hw->mac.type) {
#if !defined(NO_82542_SUPPORT) /* __rtems__ */
	case e1000_82542:
		tipg = DEFAULT_82542_TIPG_IPGT;
		tipg |= DEFAULT_82542_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82542_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
		break;
#endif
	case e1000_80003es2lan:
		tipg = DEFAULT_82543_TIPG_IPGR1;
		tipg |= DEFAULT_80003ES2LAN_TIPG_IPGR2 <<
		    E1000_TIPG_IPGR2_SHIFT;
		break;
	default:
		if ((hw->media_type == e1000_media_type_fiber) ||
		    (hw->media_type ==
		    e1000_media_type_internal_serdes))
			tipg = DEFAULT_82543_TIPG_IPGT_FIBER;
		else
			tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
		tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
		tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	}

	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(hw, E1000_TIDV, drv_e1k_tx_int_delay_us);
	if(hw->mac.type >= e1000_82540)
		E1000_WRITE_REG(hw, E1000_TADV, drv_e1k_tx_abs_int_delay_us);

	if ((hw->mac.type == e1000_82571) ||
	    (hw->mac.type == e1000_82572)) {
		tarc = E1000_READ_REG(hw, E1000_TARC0);
		tarc |= SPEED_MODE_BIT;
		E1000_WRITE_REG(hw, E1000_TARC0, tarc);
	} else if (hw->mac.type == e1000_80003es2lan) {
		tarc = E1000_READ_REG(hw, E1000_TARC0);
		tarc |= 1;
		E1000_WRITE_REG(hw, E1000_TARC0, tarc);
		tarc = E1000_READ_REG(hw, E1000_TARC1);
		tarc |= 1;
		E1000_WRITE_REG(hw, E1000_TARC1, tarc);
	}

	/* Program the Transmit Control Register */
	tctl  = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		   (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	if (hw->mac.type >= e1000_82571)
		tctl |= E1000_TCTL_MULR;

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);

	ad->txd_cmd =   E1000_TXD_CMD_EOP
	              | E1000_TXD_CMD_IFCS
	              | E1000_TXD_CMD_RS;

	if ( drv_e1k_tx_int_delay_us > 0 && e1000_82575 != hw->mac.type )
		ad->txd_cmd |= E1000_TXD_CMD_IDE;
}

static void
rxinit(struct e1000_hw *hw)
{
struct e1k_private *ad = (struct e1k_private *)hw;
uint32_t        rctl;

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring
	 */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	if(hw->mac.type >= e1000_82540) {
		E1000_WRITE_REG(hw, E1000_RADV, drv_e1k_rx_abs_int_delay_us);
		/*
		 * Set the interrupt throttling rate. This is in 256ns units
		 * which we round to 250ns...
		 */
		E1000_WRITE_REG(hw, E1000_ITR, drv_e1k_min_int_period_us * 4);
	}

	/* Setup the Base and Length of the Rx Descriptor Ring */
	E1000_WRITE_REG(hw, E1000_RDLEN, ad->rx_ring.sz * sizeof(struct e1000_leg_desc));
	E1000_WRITE_REG(hw, E1000_RDBAH, (uint32_t)0);
	E1000_WRITE_REG(hw, E1000_RDBAL, BSP_LOCAL2PCI_ADDR(ad->rx_ring.dsc));

	/* Setup the Receive Control Register */
	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		   E1000_RCTL_RDMTS_HALF |
		   E1000_RCTL_MO_0;

	if (e1000_tbi_sbp_enabled_82543(hw))
		rctl |= E1000_RCTL_SBP;
	else
		rctl &= ~E1000_RCTL_SBP;

	rctl |= E1000_RCTL_SZ_2048;

	rctl &= ~E1000_RCTL_LPE;

#if 0
	/*
	** XXX TEMPORARY WORKAROUND: on some systems with 82573
	** long latencies are observed, like Lenovo X60. This
	** change eliminates the problem, but since having positive
	** values in RDTR is a known source of problems on other
	** platforms another solution is being sought.
	*/
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);
#else
	E1000_WRITE_REG(hw, E1000_RDTR, drv_e1k_rx_int_delay_us);
#endif

	/*
	 * Setup the HW Rx Head and
	 * Tail Descriptor Pointers
	 */
	E1000_WRITE_REG(hw, E1000_RDH, ad->rx_ring.hd);
	E1000_WRITE_REG(hw, E1000_RDT, ad->rx_ring.tl);

	/* Enable Receives */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

}


static int
setup_tx_structs(struct e1k_private *ad)
{
	return 0;
}


static int
setup_rx_structs(struct e1k_private *ad)
{
int                    i,sz;
void                  *nbuf;
uintptr_t              baddr;
unsigned               diff;
struct e1000_leg_desc *d;

	if ( ad->rx_ring.sz < 1 )
		return 0;

	for ( i=0; i<ad->rx_ring.sz - 1; i++ ) {
		if ( ! (nbuf = ad->alloc_rxbuf(&sz, &baddr)) ) {
			errpr("FATAL Error: unable to allocate RX buffer\n");
			return -1;
		}
		/* A few paranoia checks:
		 *  - buffer size must be 2k
		 *  - 'baddr' must point at 2-byte offset
		 * Note that the resulting area (2-byte offset) is not
		 * really 2k anymore (which is what the chip expects)
		 * but we rely on it rejecting packets > 1522 bytes
		 * so that we should be safe.
		 */
		diff = baddr  - (uintptr_t)nbuf;
#warning "XXX disabled test"
		if ( /*2048 != sz ||*/ diff >= BUF_ALGN ) {
			errpr("FATAL Error: got size %u, offset %u\n", sz, diff);
			return -1;
		}
		put_rxb(ad, baddr + BUF_OFF);
		ad->rx_ring.av++;
	}

	/* last descriptor is empty */
	d = &ad->rx_ring.dsc[ad->rx_ring.sz - 1];
	st_le32(&d->buf_lo, 0);
	/* make sure 'DD' (desc.-done) is clear */
	st_le32(&d->sta, 0);
	
	return 0;
}

static void
hwstop(struct e1k_private *ad)
{
struct e1000_hw *hw = &ad->hw;

	drv_e1k_disable_irqs(ad, -1);
	e1000_reset_hw(hw);
	free_tx_structs(ad);
	free_rx_structs(ad);
	if ( hw->mac.type >= e1000_82544 )
		E1000_WRITE_REG(hw, E1000_WUC, 0);
}

void
drv_e1k_init_hw(struct e1k_private *ad,  int prom, unsigned char *enaddr)
{
struct e1000_hw *hw = &ad->hw;
uint32_t        ctrl, pba;

	hwstop(ad);

	switch (hw->mac.type) {
		case e1000_82547:
		case e1000_82547_rev_2: /* 82547: Total Packet Buffer is 40K */
			if (hw->mac.max_frame_size > 8192)
				pba = E1000_PBA_22K; /* 22K for Rx, 18K for Tx */
			else
				pba = E1000_PBA_30K; /* 30K for Rx, 10K for Tx */
			break;
			/* Total Packet Buffer on these is 48K */
		case e1000_82571:
		case e1000_82572:
		case e1000_82575:
		case e1000_80003es2lan:
			pba = E1000_PBA_32K; /* 32K for Rx, 16K for Tx */
			break;
		case e1000_82573: /* 82573: Total Packet Buffer is 32K */
			pba = E1000_PBA_12K; /* 12K for Rx, 20K for Tx */
			break;
		case e1000_ich9lan:
#define E1000_PBA_10K	0x000A
			pba = E1000_PBA_10K;
			break;
		case e1000_ich8lan:
			pba = E1000_PBA_8K;
			break;
		default:
			/* Devices before 82547 had a Packet Buffer of 64K.   */
			if (hw->mac.max_frame_size > 8192)
				pba = E1000_PBA_40K; /* 40K for Rx, 24K for Tx */
			else
				pba = E1000_PBA_48K; /* 48K for Rx, 16K for Tx */
	}

	E1000_WRITE_REG(hw, E1000_PBA, pba);

	if ( enaddr )
		memcpy(hw->mac.addr, enaddr, 6);

	e1000_rar_set(hw, hw->mac.addr, 0);

	drv_e1k_mcast_filter_clear(ad);

	if ( e1000_82571 == hw->mac.type ) {
		e1000_set_laa_state_82571(hw, TRUE);
	}

	if ( hwinit_ll(hw) ) {
		errpr("Unable to initialize hardware\n");
		return;
	}
	
	update_link_status(ad);

	if ( setup_tx_structs(ad) ) {
		errpr("Unable to setup TX structs\n");
		hwstop(ad);
		return;
	}

	if ( ad->tx_ring.sz > 0 )
		txinit(hw);

	if ( setup_rx_structs(ad) ) {
		errpr("Unable to setup RX structs\n");
		hwstop(ad);
		return;
	}

	if ( ad->rx_ring.sz > 0 )
		rxinit(hw);

	ctrl = E1000_READ_REG(hw, E1000_RCTL);

	if ( prom ) {
		ctrl |= E1000_RCTL_UPE | E1000_RCTL_MPE;
	} else {
		ctrl &= ~ (E1000_RCTL_UPE | E1000_RCTL_MPE);
	}
	E1000_WRITE_REG(hw, E1000_RCTL, ctrl);
	
	e1000_clear_hw_cntrs_base_generic(hw);

	hw->phy.reset_disable = TRUE;

	drv_e1k_enable_irqs(ad, -1);
}

uint32_t
drv_e1k_ack_irqs(struct e1k_private *ad, uint32_t mask)
{
uint32_t rval, key;
	rtems_interrupt_disable(key);
		rval = ad->pending;
		ad->pending &= ~mask;
	rtems_interrupt_enable(key);
	return rval;
}

int
drv_e1k_detach(struct e1k_private *ad)
{
struct e1000_hw *hw = &ad->hw;

	drv_e1k_disable_irqs(ad, -1);

	if ( ad->irq.hdl ) {
		if ( ! IRQREMV( &ad->irq ) ) {
			errpr("Unable to remove IRQ handler\n");
			return -1;
		}
	}

	hwstop(ad);
	e1000_phy_hw_reset(hw);

	switch ( hw->mac.type ) {
		case e1000_82573:
		case e1000_ich8lan:
		case e1000_ich9lan:
			if ( e1000_check_mng_mode(hw) ) {
				hwctrl(hw, 0);
			}
		break;

		default:
		break;
	}

	e1000_remove_device(hw);

	free( ad->tx_ring.mem );
	free( ad->rx_ring.mem );

	free( ad );

	return 0;
}

#ifdef TESTING

uint8_t packraw[1000] = {
	0,0,
	0,3,2,5,4,6,
	0,4,3,5,4,6,
	0x11,0x10,
};

uint8_t *pack = packraw;

void *getb(int *p_sz, unsigned long *p_data)
{
void *rval = malloc(2048);
	*p_sz = 2048;
	*p_data = (unsigned long)rval + BUF_OFF;
	return rval;
}

void freb(void *usr_buf, void *closure, int len)
{
	free(usr_buf);
}

void nulc(void *usr_buf, void *closure, int err)
{
}

struct e1k_private *
hwsetup(int unit)
{
	if ( 0 == unit )
		unit = 1;
	return drv_e1k_setup(unit, 0, nulc, 0, getb, freb, 0, 5, 5, IMS_ENABLE_MASK);
}
#endif

static struct IpBscLLDrv_ lldrv_e1k = {
	tx_irq_msk    :  E1000_IMS_TXDW,
	rx_irq_msk    :  E1000_IMS_RXT0 | E1000_IMS_RXDMT0 | E1000_IMS_RXSEQ,
	ln_irq_msk    :  E1000_IMS_LSC,
	setup         :  drv_e1k_setup,
	detach        :  drv_e1k_detach,
	read_eaddr    :  drv_e1k_read_eaddr,
	init_hw       :  drv_e1k_init_hw,
	ack_irqs      :  drv_e1k_ack_irqs,
	dis_irqs      :  drv_e1k_disable_irqs,
	enb_irqs      :  drv_e1k_enable_irqs,
	ack_ln_chg    :  drv_e1k_ack_link_chg,
	swipe_tx      :  drv_e1k_swipe_tx,
	swipe_rx      :  drv_e1k_swipe_rx,
	send_buf      :  drv_e1k_send_buf,
	med_ioctl     :  drv_e1k_media_ioctl,
	mc_filter_add :  drv_e1k_mcast_filter_accept_add,
	mc_filter_del :  drv_e1k_mcast_filter_accept_del,
	dump_stats    :  drv_e1k_dump_stats,
	drv_name      :  "e1k",
};

LLDrv drvGnrethIpBasicLLDrv = &lldrv_e1k;
