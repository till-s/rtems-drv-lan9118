#include <rtems.h>
#include <bsp.h>
#include <stdio.h>
#include <bsp/if_tsec_pub.h>

#define  lldrv_pvt tsec_private

#include <gnreth_lldrv.h>

static int
ack_ln_chg(LLDev lldev, int *p_med)
{
	return BSP_tsec_media_ioctl( lldev, 0 /* == SIOCGIFMEDIA */ , p_med );
}

static struct IpBscLLDrv_ lldrv_tsec = {
	tx_irq_msk    :  TSEC_TXIRQ,
	rx_irq_msk    :  TSEC_RXIRQ,
	ln_irq_msk    :  TSEC_LKIRQ,
	setup         :  BSP_tsec_setup_1,
	detach        :  BSP_tsec_detach,
	read_eaddr    :  BSP_tsec_read_eaddr,
	init_hw       :  BSP_tsec_init_hw,
	ack_irqs      :  BSP_tsec_ack_irq_mask,
	dis_irqs      :  BSP_tsec_disable_irq_mask,
	enb_irqs      :  BSP_tsec_enable_irq_mask,
	ack_ln_chg    :  ack_ln_chg,
	swipe_tx      :  BSP_tsec_swipe_tx,
	swipe_rx      :  BSP_tsec_swipe_rx,
	send_buf      :  BSP_tsec_send_buf,
	med_ioctl     :  BSP_tsec_media_ioctl,
	mc_filter_add :  BSP_tsec_mcast_filter_accept_add,
	mc_filter_del :  BSP_tsec_mcast_filter_accept_del,
	dump_stats    :  BSP_tsec_dump_stats,
	drv_name      :  "tsec",
};

LLDrv drvGnrethIpBasicLLDrv = &lldrv_tsec;
