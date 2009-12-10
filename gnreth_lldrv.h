/* $Id$ */

/* Low-level driver API. 'drvGnrIpBasic.c' glues low-level
 * drivers implementing this API into 'lanIpBasic'.
 */
#ifndef GNRETH_LLDRV_H
#define GNRETH_LLDRV_H

#if 0
struct mveth_private;
struct tsec_private;
struct e1k_private;

typedef union __attribute__((transparent_union)) {
	void                 *vp;
	struct mveth_private *mp;
	struct tsec_private  *tp;
	struct e1k_private   *ep;
} LLDev;
#else
typedef struct lldrv_pvt   *LLDev;
#endif

typedef struct IpBscLLDrv_ *LLDrv;

/* Linking desired low-level driver overrides this weak alias */
extern LLDrv drvGnrethIpBasicLLDrv __attribute__((weak));

struct IpBscLLDrv_ {
	LLDev        dev;                 /* instance pointer (closure for ll-driver) */
	uint32_t     tx_irq_msk;          /* bitmasks for TX, RX and LINK irqs. These */
	uint32_t     rx_irq_msk;          /* masks are passed to the 'setup' function */
	uint32_t     ln_irq_msk;          /* and checked against 'ack_irqs()' rval.   */
	LLDev   	 (*setup)(            /* setup driver; install callbacks          */
					int      unit,
					void     (*isr)(void *isr_arg),
					void     *isr_arg,
               		void     (*cleanup_txbuf)(void*, void*, int),
					void     *cleanup_txbuf_arg,
					void     *(*alloc_rxbuf)(int*, uintptr_t*),
					void     (*consume_rxbuf)(void*, void*, int),
					void     *consume_rxbuf_arg,
					int      rx_ring_size,
					int      tx_ring_size,
					int      irq_mask);
	int     	(*detach)(LLDev);     /* cleanup resources allocated by 'setup'  */
	void    	(*read_eaddr)(LLDev, unsigned char *);   /* read MAC addr       */
	void    	(*init_hw)(LLDev, int, unsigned char *);  /* init+start driver   */
	uint32_t    (*ack_irqs)(LLDev, uint32_t);   /* ack+disable irqs, returns irq bitset    */
	uint32_t    (*dis_irqs)(LLDev, uint32_t);   /* disable interrupts (above bitset)        */
	void        (*enb_irqs)(LLDev, uint32_t);   /* enable interrupts (above bitset)        */
	int         (*ack_ln_chg)(LLDev, int*); /* ack. link change                  */
	int         (*swipe_tx)(LLDev);   /* cleanup/free TX buffers swiping ring    */
	int         (*swipe_rx)(LLDev);   /* swipe RX ring, call 'consume_rxbuf'     */
	int         (*send_buf)(LLDev, void*, void*, int); /* enqueue buf for TX     */
	int         (*med_ioctl)(LLDev, int, int*); /* obtain media state            */
	void        (*mc_filter_add)(LLDev, uint8_t*); /* add addr to mcast filter   */
	void        (*mc_filter_del)(LLDev, uint8_t*); /* del addr from mcast filter */
	void        (*dump_stats)(LLDev, FILE *); /* dump statistics + info (OPTIONAL) */
	const char   *drv_name;  /* ID/name for this driver */
};

#endif
