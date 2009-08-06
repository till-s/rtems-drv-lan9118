#include <rtems.h>
#include <bsp.h>
#include <stdio.h>
#include <bsp/if_mve_pub.h>

#include <gnreth_lldrv.h>

static struct IpBscLLDrv_ lldrv_mve = {
	tx_irq_msk:  BSP_MVE_IRQ_TX,
	rx_irq_msk:  BSP_MVE_IRQ_RX,
	ln_irq_msk:  /* BSP_MVE_IRQ_LINK */ 0,
	setup     :  (void*) BSP_mve_setup,
	detach    :  (void*) BSP_mve_detach,
	read_eaddr:  (void*) BSP_mve_read_eaddr,
	init_hw   :  (void*) BSP_mve_init_hw,
	ack_irqs  :  (void*) BSP_mve_ack_irqs,
	enb_irqs  :  (void*) BSP_mve_enable_irqs,
	ack_ln_chg:  (void*) BSP_mve_ack_link_chg,
	swipe_tx  :  (void*) BSP_mve_swipe_tx,
	swipe_rx  :  (void*) BSP_mve_swipe_rx,
	send_buf  :  (void*) BSP_mve_send_buf,
	med_ioctl :  (void*) BSP_mve_media_ioctl,
};

LLDrv drvGnrethIpBasicLLDrv = &lldrv_mve;
