#include <rtems.h>
#include <bsp.h>
#include <stdio.h>
#include <bsp/if_mve_pub.h>

#define  lldrv_pvt mveth_private

#include <gnreth_lldrv.h>

static struct IpBscLLDrv_ lldrv_mve = {
	tx_irq_msk    :  BSP_MVE_IRQ_TX,
	rx_irq_msk    :  BSP_MVE_IRQ_RX,
	ln_irq_msk    :  /* BSP_MVE_IRQ_LINK */ 0,
	setup         :  BSP_mve_setup_1,
	detach        :  BSP_mve_detach,
	read_eaddr    :  BSP_mve_read_eaddr,
	init_hw       :  BSP_mve_init_hw,
	ack_irqs      :  BSP_mve_ack_irq_mask,
	dis_irqs      :  BSP_mve_disable_irq_mask,
	enb_irqs      :  BSP_mve_enable_irq_mask,
	ack_ln_chg    :  BSP_mve_ack_link_chg,
	swipe_tx      :  BSP_mve_swipe_tx,
	swipe_rx      :  BSP_mve_swipe_rx,
	send_buf      :  BSP_mve_send_buf,
	med_ioctl     :  BSP_mve_media_ioctl,
	mc_filter_add :  BSP_mve_mcast_filter_accept_add,
	mc_filter_del :  BSP_mve_mcast_filter_accept_del,
	dump_stats    :  BSP_mve_dump_stats,
	drv_name      :  "mve",
};

LLDrv drvGnrethIpBasicLLDrv = &lldrv_mve;
