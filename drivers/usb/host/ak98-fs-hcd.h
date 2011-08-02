/*
 * AK98HS register declarations and HCD data structures
 */

#ifndef __AK98_FS_HCD_H

#include <mach/map.h>
#include <mach/gpio.h>

//#define AK98FS_DEBUG
#define AK98FS_VERBOSE_DEBUG

/*
 * Enable DYNAMIC_EPFIFO when you want dynamic endpoint fifo to be allocated to USB device.
 * For example, if a USB Flash disk has EP 1 + Bulk In / Bulk Out, using DYNAMIC_EPFIFO will
 * cause EPFIFO 1 <--> Flash Disk EP 1/Bulk In, EPFIFO 2 <--> Flash Disk EP 1/Bulk Out, else
 * only EPFIFO 1 <--> Flash Disk EP 1 + Bulk In / Bulk Out (Tx/Rx mode will be set on
 * in_packet() and out_packet())
 */
#define DYNAMIC_EPFIFO

#define USBFS_BASE_ADDR (AK98_VA_USB + 0x0800)
	 
	 /*Control registers.*/
#define UR_FUNADDR	0x00		/*Function address register. ---8bit*/
#define UR_PWM 		0x01 		/* Usb power control register .---8bit*/
#define UR_INTTX 	0x02		/*Usb TX ep interrupt control register.---16bit */
#define UR_INTRX 	0x04		/*Usb RX ep interrupt control register.--16bit */
#define UR_INTETX 	0x06		/*To enable the TX ep interrupt register.--16bit */
#define UR_INTERX 	0x08		/*To enable the RX ep interrupt register.--16bit */
#define UR_INTCOM 	0x0a		/*Common usb interrupt register. ---8bit*/
#define UR_INTECOM	0x0b		/*To enable the common usb interrupt register. --8bit*/
#define UR_FRMNUM	0x0c		/*Frame number register. ---16bit*/
#define UR_INDEX	0x0e		/*Index the selected ep control/status register.--8bit */
#define UR_TESTM	0x0f		/*Test mode enbale for usb2.0 register. --8bit*/
	 
	 /*Endpoint Contrl/Status Registers.*/
#define MaxPackSz_TX	0x10	/*Max packet size for TX EPx.  --16bit*/
#define CTLSTS_EP_TXL	0x12	/*Contrl & status for TX EPx.--8bit*/
#define CTLSTS_EP_TXH	0x13	/*Contrl & status for TX EPx.--8bit*/
#define MaxPackSz_RX	0x14	/*Max packet size for RX EPx.---16bit*/
#define CTLSTS_EP_RXL	0x16	/*Contrl & status for RX EPx.  --8bit*/
#define CTLSTS_EP_RXH	0x17	/*Contrl & status for RX EPx.  --8bit*/

#define EP_COUNT		0x18	/*Number of bytes in endpoint X FIFO.--16bit*/
#define TX_EP_TYPE		0x1a	/*endpoint type for TX endpoint.--8bit*/
#define NAK_TO_EP0		0x1b	/*NAK response timeout on EP0.--8bit*/
#define RX_EP_TYPE		0x1c	/*endpoint type for RX endpoint.---8bit*/
#define RX_INTVAL		0x1d	/*polling interal for RX endpoint.---8bit*/
#define CFG_FIFO_SZ		0x1f	/*core configuration & FIFO size.---8bit*/
	 
	 /*FIFO 32bit registers.Can read write 8bit, 16bit, 32bit.*/
#define FIFO_EP0		0x20	/*FIFO for EP0.*/
#define FIFO_EP1		0x24	/*FIFO for EP1.*/
#define FIFO_EP2		0x28	/*FIFO for EP2.*/
#define FIFO_EP3		0x2c	/*FIFO for EP3.*/
#define FIFO_EP4		0x30	/*FIFO for EP4.*/
#define FIFO_EP5		0x34	/*FIFO for EP5.*/
#define FIFO_EP6		0x38	/*FIFO for EP6.*/

	 
	 /*DMA controler registers.*/
#define USB_DMA_ADDR (AK88_VA_USB + 0x0A00)
#define DMA_INTR_STAT	0x0		/*DMA interrupt status.*/
#define DMA_CTRL_REG1	0x04	/*DMA channel 1 control.*/
#define DMA_CTRL_REG2	0x14	/*DMA channel 2 control.*/
#define DMA_CTRL_REG3	0x24	/*DMA channel 3 control.*/
#define DMA_CTRL_REG4	0x34	/*DMA channel 4 control.*/
#define DMA_CTRL_REG5	0x44	/*DMA channel 5 control.*/
#define DMA_CTRL_REG6	0x54	/*DMA channel 6 control.*/
	 
#define DMA_ADDR_REG1	0x08	/*DMA channel 1 AHB memory address.*/
#define DMA_ADDR_REG2	0x18	/*DMA channel 2 AHB memory address.*/
#define DMA_ADDR_REG3	0x28	/*DMA channel 3 AHB memory address.*/
#define DMA_ADDR_REG4	0x38	/*DMA channel 4 AHB memory address.*/
#define DMA_ADDR_REG5	0x48	/*DMA channel 5 AHB memory address.*/
#define DMA_ADDR_REG6	0x58	/*DMA channel 6 AHB memory address.*/
	 
#define DMA_CUNT_REG1	0x0c	/*DMA channel 1 byte count.*/
#define DMA_CUNT_REG2	0x1c	/*DMA channel 2 byte count.*/
#define DMA_CUNT_REG3	0x2c	/*DMA channel 3 byte count.*/
#define DMA_CUNT_REG4	0x3c	/*DMA channel 4 byte count.*/
#define DMA_CUNT_REG5	0x4c	/*DMA channel 5 byte count.*/
#define DMA_CUNT_REG6	0x5c	/*DMA channel 6 byte count.*/
	 
#define DEV_CTL_REG		0x60	/*Device control register.*/
	 
	 /*Interrupt types.*/
#define INTR_CONNECTED	0x10	/*Connected interrupt .*/
#define INTR_DISCONNECTED	0x20	/*Connected interrupt .*/
#define INTR_SESION_RQ		0x40    /*Session interrupt.*/
#define INTR_RESUME		0x02	/* Resume interrupt */
#define INTR_SOF		0x08    /*Session interrupt.*/

/*====================================*/
#define H_MAXPACKET	64		/* bytes in fifo */

#define MAX_EP_NUM	(6)

/*-------------------------------------------------------------------------*/

#define	LOG2_PERIODIC_SIZE	5	/* arbitrary; this matches OHCI */
#define	PERIODIC_SIZE		(1 << LOG2_PERIODIC_SIZE)

struct ak98fsh {
	spinlock_t		lock;
	void __iomem		*addr_reg;
	void __iomem		*data_reg;
	struct ak98_platform_data	*board;
	struct proc_dir_entry	*pde;

	unsigned long		stat_insrmv;
	unsigned long		stat_wake;
	unsigned long		stat_sof;
	unsigned long		stat_ep0;
	unsigned long		stat_epx[MAX_EP_NUM];
	unsigned long		stat_lost;
	unsigned long		stat_overrun;

	/* sw model */
	struct timer_list	timer;
	struct ak98fsh_ep	*next_periodic;
	struct ak98fsh_ep	*next_async_ep0;
	struct ak98fsh_ep	*next_async_epx[MAX_EP_NUM];

	struct ak98fsh_ep	*active_ep0;
	struct ak98fsh_ep	*active_epx[MAX_EP_NUM];
	unsigned long		jiffies_ep0;
	unsigned long		jiffies_epx[MAX_EP_NUM];

	u32			port_status;
	u16			frame;

	/* async schedule: control, bulk */
	struct list_head	async_ep0;
	struct list_head	async_epx[MAX_EP_NUM];

	/* periodic schedule: interrupt, iso */
	u16			load[PERIODIC_SIZE];
	struct ak98fsh_ep	*periodic[PERIODIC_SIZE];
	unsigned		periodic_count;
};

static inline struct ak98fsh *hcd_to_ak98fsh(struct usb_hcd *hcd)
{
	return (struct ak98fsh *) (hcd->hcd_priv);
}

static inline struct usb_hcd *ak98fsh_to_hcd(struct ak98fsh *ak98fsh)
{
	return container_of((void *) ak98fsh, struct usb_hcd, hcd_priv);
}

struct ak98fsh_ep {
	struct usb_host_endpoint *hep;
	struct usb_device	*udev;

	u8			maxpacket;
	u8			epnum;
	u8			nextpid;

	u16			error_count;
	u16			nak_count;
	u16			length;		/* of current packet */

	/* periodic schedule */
	u16			period;
	u16			branch;
	u16			load;
	struct ak98fsh_ep	*next;

	/* async schedule */
	struct list_head	schedule;
};

/*-------------------------------------------------------------------------*/
/*
 * AK98 Full Speed Register Access Routines:
 * Part I: Common Registers Access(Do NOT use INDEX register)
 * Part II: Index Registers Access
 */

/*
 * Part I: Common Registers Access - Just use USBFS_BASE_ADDR as base address plus offset
 */
#define fsh_readb(reg)		__raw_readb(USBFS_BASE_ADDR + (reg))
#define fsh_writeb(val, reg)	__raw_writeb(val, USBFS_BASE_ADDR + (reg))

#define fsh_readw(reg)		__raw_readw(USBFS_BASE_ADDR + (reg))
#define fsh_writew(val, reg)	__raw_writew(val, USBFS_BASE_ADDR + (reg))

#define fsh_readl(reg)		__raw_readl(USBFS_BASE_ADDR + (reg))
#define fsh_writel(val, reg)	__raw_writel(val, USBFS_BASE_ADDR + (reg))

/*
 * Part II: Index Registers Access - Spinlock+IRQ protection
 */
static DEFINE_SPINLOCK(fsh_reg_lock);
static inline unsigned char fsh_index_readb(int epindex, int reg)
{
	unsigned long flags;
	unsigned char val;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	val = fsh_readb(reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);

	return val;
}

static inline void fsh_index_writeb(int epindex, unsigned char val, int reg)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	fsh_writeb(val, reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline unsigned short fsh_index_readw(int epindex, int reg)
{
	unsigned long flags;
	unsigned short val;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	val = fsh_readw(reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);

	return val;

}

static inline void fsh_index_writew(int epindex, unsigned short val, int reg)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	fsh_writew(val, reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline unsigned long fsh_index_readl(int epindex, int reg)
{
	unsigned long flags;
	unsigned long val;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	val = fsh_readl(reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);

	return val;
}

static inline void fsh_index_writel(int epindex, unsigned long val, int reg)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(epindex, UR_INDEX);
	fsh_writel(val, reg);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);

}


#ifdef AK98FS_DEBUG
#define HDBG(stuff...) printk("USBFS: " stuff)
#else
#define HDBG(fmt, args...) do{}while(0)
#endif 

#ifdef AK98FS_VERBOSE_DEBUG
#define VDBG	HDBG
#else
#define VDBG(fmt, args...) do{}while(0)
#endif 

#define ERR(stuff...) printk(KERN_ERR "USBFS: " stuff)

#ifdef AK98FS_VERBOSE_DEBUG
static inline void dump_registers(void)
{
	int i;

	printk("USBFS: FADDR=%d, IntrTXE=%x, IntrRXE=%x, IntrUSBE=%x, DevCtl=%x\n",
		fsh_readb(UR_FUNADDR), fsh_readw(UR_INTETX), fsh_readw(UR_INTERX),
		fsh_readb(UR_INTECOM), fsh_readw(DEV_CTL_REG));
	for (i = 0; i < MAX_EP_NUM + 1; i++) {
		printk("  EP%d:",i );
		printk(" NAK_TO_EP=%x, RX_TYPE=%x, TX_TYPE=%x, TXMAXP=%x, " \
			"RXMAXP=%x, RXCSR=%x, TXCSR=%x, RXINTERVAL=%x\n",
			fsh_index_readb(i, NAK_TO_EP0), fsh_index_readb(i, RX_EP_TYPE), fsh_index_readb(i, TX_EP_TYPE),
			fsh_index_readw(i, MaxPackSz_TX), fsh_index_readw(i, MaxPackSz_RX),
			fsh_index_readw(i, CTLSTS_EP_RXL), fsh_index_readw(i, CTLSTS_EP_TXL),
			fsh_index_readw(i, RX_INTVAL));
	}
}
#else
static inline void dump_registers(void)
{
}
#endif

static inline void flush_ep0_fifo(void)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(0, UR_INDEX);
	if (fsh_readb(CTLSTS_EP_TXL) & ((1 << 0) | (1 << 1)))
		fsh_writeb(1 << 0, CTLSTS_EP_TXH);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline void flush_epx_tx_fifo(int i)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(i, UR_INDEX);
	if (fsh_readb(CTLSTS_EP_TXL) & (1 << 0))
		fsh_writeb(1 << 3, CTLSTS_EP_TXL);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);

}

static inline void flush_epx_rx_fifo(int i)
{
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(i, UR_INDEX);
	if (fsh_readb(CTLSTS_EP_RXL) & (1 << 0))
		fsh_writeb(1 << 4, CTLSTS_EP_RXL);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline void flush_epx_fifo(int i)
{
	flush_epx_tx_fifo(i);
	flush_epx_rx_fifo(i);
}

static inline void flush_ep_fifo(int i)
{
	if (i == 0)
		flush_ep0_fifo();
	else
		flush_epx_fifo(i);
}

static inline void set_epx_rx_mode(int i)
{
	u8 regval;
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(i, UR_INDEX);
	regval = fsh_readb(CTLSTS_EP_TXH);
	regval &= ~(1 << 5);
	fsh_writeb(regval, CTLSTS_EP_TXH);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline void set_epx_tx_mode(int i)
{
	u8 regval;
	unsigned long flags;

	spin_lock_irqsave(&fsh_reg_lock, flags);

	fsh_writeb(i, UR_INDEX);
	regval = fsh_readb(CTLSTS_EP_TXH);
	regval |= (1 << 5);
	fsh_writeb(regval, CTLSTS_EP_TXH);

	spin_unlock_irqrestore(&fsh_reg_lock, flags);
}

static inline void clear_epx_tx_data_toggle(int i)
{
	fsh_index_writeb(i, 1 << 6, CTLSTS_EP_TXL);
}

static inline void clear_epx_rx_data_toggle(int i)
{
	fsh_index_writeb(i, 1 << 7, CTLSTS_EP_RXL);
}

/*
 * Valid types:
 *   1 - Isochronous
 *   2 - Bulk
 *   3 - Interrupt
 * Invalid type:
 *   0 - Illegal
 */
static inline void set_epx_tx_type(int i, int epnum, int type)
{
	BUG_ON(i < 0 || i > MAX_EP_NUM);
	BUG_ON(type < 0 || type > 3);

	fsh_index_writeb(i, type << 4 | epnum, TX_EP_TYPE);
}

static inline void set_epx_rx_type(int i, int epnum, int type)
{
	BUG_ON(i < 0 || i > MAX_EP_NUM);
	BUG_ON(type < 0 || type > 3);

	fsh_index_writeb(i, type << 4 | epnum, RX_EP_TYPE);
}

static inline void enable_ep0_interrupt(void)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTETX);
	regval |= (1 << 0);
	fsh_writeb(regval, UR_INTETX);

	local_irq_restore(flags);
}

static inline void enable_epx_tx_interrupt(int i)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTETX);
	regval |= (1 << i);
	fsh_writeb(regval, UR_INTETX);

	local_irq_restore(flags);
}

static inline void enable_epx_rx_interrupt(int i)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTERX);
	regval |= (1 << i);
	fsh_writeb(regval, UR_INTERX);

	local_irq_restore(flags);
}

static inline void disable_ep0_interrupt(void)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTETX);
	regval &= ~(1 << 0);
	fsh_writeb(regval, UR_INTETX);

	local_irq_restore(flags);
}

static inline void disable_epx_tx_interrupt(int i)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTETX);
	regval &= ~(1 << i);
	fsh_writeb(regval, UR_INTETX);

	local_irq_restore(flags);
}

static inline void disable_epx_rx_interrupt(int i)
{
	u8 regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = fsh_readb(UR_INTERX);
	regval &= ~(1 << i);
	fsh_writeb(regval, UR_INTERX);

	local_irq_restore(flags);
}

static inline void disable_epx_interrupt(int i)
{
	disable_epx_tx_interrupt(i);
	disable_epx_rx_interrupt(i);
}

static inline void disable_ep_interrupt(int i)
{
	BUG_ON(i < 0 || i > MAX_EP_NUM);

	if (i == 0) {
		disable_ep0_interrupt();
	} else {
		disable_epx_interrupt(i);
	}
}

static inline void clear_all_interrupts(void)
{
	fsh_readb(UR_INTCOM);
	fsh_readw(UR_INTTX);
	fsh_readw(UR_INTRX);
}

static inline void reset_endpoint(int i)
{
	BUG_ON(i < 0 || i > MAX_EP_NUM);

	disable_ep_interrupt(i);
	if (i == 0) {
		flush_ep0_fifo();
	} else {
		flush_epx_fifo(i);
		set_epx_rx_type(i, 0, 0);
		set_epx_tx_type(i, 0, 0);
	}
}

static inline void reset_endpoints(void)
{
	int i;

	for (i = 0; i < MAX_EP_NUM + 1; i++) {
		reset_endpoint(i);
	}
}




struct epfifo_mapping {
	int	epfifo;		/* AK98 FSH HC EP FIFO number: 1 ~ 6 */
	int	used;		/* 0 - Unused, 1 - used */
	int	epnum;		/* USB Device endpoint number: 1 ~ 16 */
	int	direction;	/* 0 - In, 1 - Out */
};

struct ak98fsh_epfifo_mapping {
	spinlock_t lock;
	struct epfifo_mapping mapping[MAX_EP_NUM];
};

static inline void dump_epfifo_mapping(struct ak98fsh_epfifo_mapping *ak98_mapping)
{
	int i;
	struct epfifo_mapping *mapping;

	for (i = 0; i < MAX_EP_NUM; i++) {
		mapping = &ak98_mapping->mapping[i];
		printk("EPFIFO %d: %s, Device EP NO. %d, Direction %s\n",
			mapping->epfifo, mapping->used ? "Allocated" : "Free",
			mapping->epnum, mapping->direction ? "OUT" : "IN");
	}

}

static inline void init_epfifo_mapping(struct ak98fsh_epfifo_mapping *ak98_mapping)
{
	int i;
	struct epfifo_mapping *mapping;

	spin_lock_init(&ak98_mapping->lock);
	
	for (i = 0; i < MAX_EP_NUM; i++) {
		mapping = &ak98_mapping->mapping[i];
		mapping->epfifo = i + 1;	/* EPFIFO 1~6 is used by AK98 FS HCD */
		mapping->used = 0;
		mapping->epnum = 0;
		mapping->direction = 0;
	}
}

static inline bool __is_epnum_mapped(struct ak98fsh_epfifo_mapping *ak98_mapping,
	int epnum, int direction)
{
	int i;
	struct epfifo_mapping *mapping;

	if(epnum == 0)
		return true;

	for (i = 0; i < MAX_EP_NUM; i++) {
		mapping = &ak98_mapping->mapping[i];
		if (mapping->used && (mapping->epnum == epnum) && (mapping->direction == direction)) {
			return true;
		}
	}

	return false;
}

static inline bool is_epnum_mapped(struct ak98fsh_epfifo_mapping *ak98_mapping,
	int epnum, int direction)
{
	bool ret;
	unsigned long flags;

	BUG_ON(ak98_mapping == NULL);

	if(epnum == 0)
		return true;

	spin_lock_irqsave(&ak98_mapping->lock, flags);

	ret = __is_epnum_mapped(ak98_mapping, epnum, direction);

	spin_unlock_irqrestore(&ak98_mapping->lock, flags);

	return ret;

}

static inline bool __map_epnum_to_epfifo(struct ak98fsh_epfifo_mapping *ak98_mapping,
	int epnum, int direction, int *epfifo)
{
	int i;
	struct epfifo_mapping *mapping;

	if (__is_epnum_mapped(ak98_mapping, epnum, direction))
		return false;

	for (i = 0; i < MAX_EP_NUM; i++) {
		mapping = &ak98_mapping->mapping[i];
		if (!mapping->used) {
			mapping->used = 1;
			mapping->epnum = epnum;
			mapping->direction = direction;
			*epfifo = mapping->epfifo;
			return true;
		}
	}

	return false;
}

static inline bool map_epnum_to_epfifo(struct ak98fsh_epfifo_mapping *ak98_mapping,
	int epnum, int direction, int *epfifo)
{
	bool ret;
	unsigned long flags;

	BUG_ON(ak98_mapping == NULL);

	if (is_epnum_mapped(ak98_mapping, epnum, direction))
		return false;

	spin_lock_irqsave(&ak98_mapping->lock, flags);
	
	ret = __map_epnum_to_epfifo(ak98_mapping, epnum, direction, epfifo);

	spin_unlock_irqrestore(&ak98_mapping->lock, flags);

	return ret;
}

#ifdef DYNAMIC_EPFIFO
static inline bool epfifo_to_epnum(struct ak98fsh_epfifo_mapping *ak98_mapping, int epfifo, int *epnum, int *direction)
{
	int ret;
	unsigned long flags;
	struct epfifo_mapping *mapping;

	ret = false;

	spin_lock_irqsave(&ak98_mapping->lock, flags);

	mapping = &ak98_mapping->mapping[epfifo];
	if (mapping->used) {
		*epnum = mapping->epnum;
		*direction = mapping->direction;
		ret = true;
	} else {
		*epnum = 0;
		*direction = 0;
		ret = false;
	}

	spin_unlock_irqrestore(&ak98_mapping->lock, flags);

	return ret;
}

static inline bool epnum_to_epfifo(struct ak98fsh_epfifo_mapping *ak98_mapping, int epnum, int direction, int *epfifo)
{
	int i;
	unsigned long flags;
	struct epfifo_mapping *mapping;

	if (epnum == 0) {
		*epfifo = 0;
		return true;
	}

	spin_lock_irqsave(&ak98_mapping->lock, flags);

	for (i = 0; i < MAX_EP_NUM; i++) {
		mapping = &ak98_mapping->mapping[i];
		if (mapping->used && (mapping->epnum == epnum) && (mapping->direction == direction)) {
			*epfifo = mapping->epfifo;
			spin_unlock_irqrestore(&ak98_mapping->lock, flags);
			return true;
		}
	}

	spin_unlock_irqrestore(&ak98_mapping->lock, flags);

	return false;
}
#else
static inline bool epfifo_to_epnum(struct ak98fsh_epfifo_mapping *ak98_mapping, int epfifo, int *epnum, int *direction)
{
	*epnum = epfifo;
	*direction  = 0;	/* Useless, since 1 EPFIFO could be mapped to 1 device ep/in + out */
	return true;
}

static inline bool epnum_to_epfifo(struct ak98fsh_epfifo_mapping *ak98_mapping, int epnum, int direction, int *epfifo)
{
	*epfifo = epnum;
	return true;
}
#endif

#endif /* __AK98_FS_HCD_H */
