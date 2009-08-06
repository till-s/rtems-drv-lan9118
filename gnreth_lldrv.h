/* $Id$ */

/* Low-level driver API. 'drvGnrIpBasic.c' glues low-level
 * drivers implementing this API into 'lanIpBasic'.
 */
#ifndef GNRETH_LLDRV_H
#define GNRETH_LLDRV_H

typedef struct IpBscLLDrv_ *LLDrv;
typedef void               *LLDev;

/* Linking desired low-level driver overrides this weak alias */
extern LLDrv drvGnrethIpBasicLLDrv __attribute__((weak));

struct IpBscLLDrv_ {
	LLDev        dev;                 /* instance pointer (closure for ll-driver) */
	uint32_t     tx_irq_msk;          /* bitmasks for TX, RX and LINK irqs. These */
	uint32_t     rx_irq_msk;          /* masks are passed to the 'setup' function */
	uint32_t     ln_irq_msk;          /* and checked against 'ack_irqs()' rval.   */
	void    	*(*setup)(            /* setup driver; install callbacks          */
					int      unit,
					rtems_id tid,
               		void     (*cleanup_txbuf)(void*, void*, int),
					void     *cleanup_txbuf_arg,
					void     *(*alloc_rxbuf)(int*, unsigned long*),
					void     (*consume_rxbuf)(void*, void*, int),
					void     *consume_rxbuf_arg,
					int      rx_ring_size,
					int      tx_ring_size,
					uint32_t irq_mask);
	int     	(*detach)(void*);     /* cleanup resources allocated by 'setup'  */
	void    	(*read_eaddr)(void *, unsigned char *);   /* read MAC addr       */
	void    	(*init_hw)(void*, int, unsigned char *);  /* init+start driver   */
	uint32_t    (*ack_irqs)(void*);   /* ack+disable irqs, returns irq bitset    */
	void        (*enb_irqs)(void*);   /* enable interrupts (above bitset)        */
	int         (*ack_ln_chg)(void*, int*); /* ack. link change                  */
	int         (*swipe_tx)(void*);   /* cleanup/free TX buffers swiping ring    */
	int         (*swipe_rx)(void*);   /* swipe RX ring, call 'consume_rxbuf'     */
	int         (*send_buf)(void*, void*, void*, int); /* enqueue buf for TX     */
	int         (*med_ioctl)(void*, int, int*); /* obtain media state            */
};

#endif
