/*
 * ak880x_udc -- driver for ak7801 USB peripheral controller
 * Features
 * The USB 2.0 HS OTG has following features:
 *      •   compliant with USB Specification Version 2.0 (HS) and On-The-Go supplement to
 *	  the USB 2.0 specification
 *      •   operating as the host in point-to-point communications with another USB function
 *	  or as a function controller for a USB peripheral
 *      •   supporting UTMI+ Level 2 Transceiver Interface
 *      •   4 Transmit/Receive endpoints in addition to Endpoint 0
 *      •   3 DMA channels
 *
 * 09-11-14 10:45:03
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/dma-mapping.h>

#include "ak88_udc.h"

#if 0
#define dbg(fmt, arg...) printk("%s(%d): " fmt "\n", __func__, __LINE__, ##arg)
#else
#define dbg(fmt, arg...) {}
#endif

static const char ep0name[] = "ep0";
static const char driver_name[] = "ak880x_udc";

#define udc_readb(reg)	__raw_readb(udc->baseaddr + (reg))
#define udc_readw(reg)	__raw_readw(udc->baseaddr + (reg))
#define udc_readl(reg)	__raw_readl(udc->baseaddr + (reg))
#define udc_writeb(reg, val) __raw_writeb(val, udc->baseaddr + (reg))
#define udc_writew(reg, val) __raw_writew(val, udc->baseaddr + (reg))
#define udc_writel(reg, val) __raw_writel(val, udc->baseaddr + (reg))

extern struct mutex nand_lock;
static struct ak880x_udc controller;
/* static struct ak880x_udc *udc = &controller; */
struct workqueue_struct *ep_wqueue;

#include "ak88_l2.c"

volatile int usb_detect;
volatile int usb_exist;
EXPORT_SYMBOL(usb_detect);
EXPORT_SYMBOL(usb_exist);

#if 1
static void ep_irq_enable(struct ak880x_udc *udc)
{
	if (udc->ep[0].irq_enable) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) | (0x1<<0));

	if (udc->ep[1].irq_enable)
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) | (0x1<<1));

	if (udc->ep[2].irq_enable) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) | (0x1<<2));
	
	if (udc->ep[3].irq_enable)
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) | (0x1<<3));
}

static void ep_irq_disable(struct ak880x_udc *udc)
{
	udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) & ~(0x1<<0));
	udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) & ~(0x1<<1));
	udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) & ~(0x1<<2));
	udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) & ~(0x1<<3));
}
#else
#define	ep_irq_enable(udc)	local_irq_enable()
#define	ep_irq_disable(udc)	local_irq_disable()
#endif

static inline void reset_usbcontroller()
{
	rCLK_CON |= (0x1<<21);
	rCLK_CON &= ~(0x1<<21);
}

static inline void *l2_memcpy(void *dest, const void *src, size_t count)
{
	volatile unsigned int *tmp = dest;
	const unsigned int *s = src;
	
#if 0
	count = EP0_FIFO_SIZE;
#else
	count += 3;
#endif
	count = count / sizeof(int);
	while (count--) 
		*tmp++ = *s++;

	return dest;
}

#define USB_ENABLE_DMA              (1)
#define USB_DIRECTION_RX            (0<<1)
#define USB_DIRECTION_TX            (1<<1)
#define USB_DMA_MODE1               (1<<2)
#define USB_DMA_MODE0               (0<<2)
#define USB_DMA_INT_ENABLE          (1<<3)
#define USB_DMA_INT_DISABLE         (0<<3)
#define USB_DMA_BUS_ERROR           (1<<8)
#define USB_DMA_BUS_MODE0           (0<<9)
#define USB_DMA_BUS_MODE1           (1<<9)
#define USB_DMA_BUS_MODE2           (2<<9)
#define USB_DMA_BUS_MODE3           (3<<9)
#define DMA_CHANNEL1_INT          	(1)
#define USB_EP0_INDEX             	(0)
#define USB_EP1_INDEX             	(1 << 0)
#define USB_EP2_INDEX             	(1 << 1)
#define USB_EP3_INDEX             	((1 << 1)|(1 << 0))
#define USB_EP4_INDEX             	(1 << 2)
#define USB_EP5_INDEX             	((1 << 2)|(1 << 0))
#define USB_EP6_INDEX             	((1 << 2)|(1 << 1))
#define USB_EP7_INDEX             	((1 << 2)|(1 << 1)|(1 << 0))
#include "./ak88_usbudc.h"
static void usb_dma_send_mode0(struct ak880x_udc *udc, T_U8 EP_index, T_U32 addr, T_U32 count)
{
	int timeout;
    T_U32 usb_dma_int;
    
    REG8(USB_REG_INDEX) = EP_index;
    
    REG8(USB_REG_TXCSR2) =  USB_TXCSR_MODE1;
    
    REG32(USB_DMA_ADDR1) = addr;
    REG32(USB_DMA_COUNT1) = count; 
    //akprintf(C3, M_DRVSYS, "USB_DMA_COUNT_1 = %x\n", inl(USB_DMA_COUNT_1));
    REG32(USB_DMA_CNTL_1) = (USB_ENABLE_DMA | USB_DIRECTION_TX
					       | USB_DMA_INT_ENABLE | (USB_EP2_INDEX<<4) |
						USB_DMA_BUS_MODE3);
						
    //akprintf(C3, M_DRVSYS, "USB_DMA_CNTL_1 = %x\n", inl(USB_DMA_CNTL_1));                                                                          
    timeout = 5000;
    while(1)
    {
		usb_dma_int = REG32(USB_DMA_INTR);
		if(((usb_dma_int & DMA_CHANNEL1_INT)  == DMA_CHANNEL1_INT) && timeout)
		{
			timeout--;
			break;
		}
	} 
}

static void done(struct ak880x_ep *ep, struct ak880x_request *req, int status)
{
	unsigned	 stopped = ep->stopped;
	struct ak880x_udc *udc = ep->udc;

	list_del_init(&req->queue);
        /*
	 * if (req->req.status == -EINPROGRESS)
	 *         req->req.status = status;
	 * else
	 *         status = req->req.status;
         */
	req->req.status = status;
	if (status && status != -ESHUTDOWN) {
		dbg("%s done req(0x%p), status %d\n", ep->ep.name, req, status);
	}

	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	/* ep->stopped = stopped; */
	ep->stopped = 1;
	dbg("%s done, req.status(%d)", ep->ep.name, req->req.status);
	/* printk("I am done, req.status(%d)\n", req->req.status); */
}

static int write_ep0_fifo(struct ak880x_ep *ep, struct ak880x_request *req)
{
	int i;
	unsigned total, count, is_last;
	struct ak880x_udc *udc = ep->udc;

	total = req->req.length - req->req.actual;

	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count <= ep->ep.maxpacket) || !req->req.zero;
	}

	dbg("is_last(%d), count(%d), total(%d), actual(%d), length(%d)", 
			is_last, count, total, req->req.actual, req->req.length);

	udc_writeb(USB_EP_INDEX, 0);
	if (count == 0) {
		dbg(" count == 0 ");
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<1);
		done(ep, req, 0);
		return 1;
	}

	udc_writel(USB_BUFFER_FORBIDDEN, udc_readl(USB_BUFFER_FORBIDDEN) | 0x1);
	for (i = 0; i < count; i++) {
		udc_writeb(USB_EP0_FIFO, 0);
	}

	l2_memcpy((void *)EP0_L2_ADDR, req->req.buf + req->req.actual, count);
	udc_writel(USB_EP0_NUM, count&0x7f); /* 7bits */ 
	udc_writel(USB_PREREAD_START, udc_readl(USB_PREREAD_START) | 0x1);
	if (is_last) {
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<1);
	} else {
		udc_writeb(USB_CTRL_1, 0x1<<1);
	}
	udc_writel(USB_BUFFER_FORBIDDEN, udc_readl(USB_BUFFER_FORBIDDEN) & ~0x1);

	req->req.actual += count;
	if (is_last) 
		done(ep, req, 0);

	ep->done = is_last;
	return is_last;
	/* return 1; */
}

#if 0
static int read_ep0_fifo(struct ak880x_ep *ep, struct ak880x_request *req)
{
	struct ak880x_udc *udc = ep->udc;
	unsigned char *buf;
	unsigned int csr;
	unsigned int count, bufferspace, is_done;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	dbg("req.length(%d), req.actual(%d)", req->req.length, req->req.actual);

	udc_writeb(USB_EP_INDEX, 0);
	csr = udc_readb(USB_CTRL_1);
	if ((csr & 0x1) == 0) {
		dbg("waiting ep0-out data");
		return 0;
	}
	count = udc_readw(USB_EP_COUNT);
	if (count == 0) {
		dbg("rxcount(0) end");
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
		return 0;
	}
	if (count > bufferspace) {
		dbg("%s count(%d), bufferspace(%d), buffer overflow\n", 
				 ep->ep.name, count, bufferspace);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}
	memcpy(buf, (void *)EP0_L2_ADDR, count);

	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;
	if (is_done) {
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6); /* clear rx flag */ 
		done(ep, req, 0);
	}
	else
		udc_writeb(USB_CTRL_1, 0x1<<6); /* clear rx flag */ 

	return is_done;
}
#endif

static int write_ep1_fifo(struct ak880x_ep *ep, struct ak880x_request *req)
{
	return 1;
}

static int write_ep2_fifo(struct ak880x_ep *ep, struct ak880x_request *req) /* ep2 */ 
{
	struct ak880x_udc *udc = ep->udc;
	unsigned total, count, is_last;
	unsigned char *buf;
	dma_addr_t phys;

	total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count <= ep->ep.maxpacket) || !req->req.zero;
	}
	dbg("total(%d), count(%d), is_last(%d)", total, count, is_last);

	if (count == 0) { /* not support command */ 
		dbg("count == 0\n");
		ep->done = 1;
		udc_writeb(USB_EP_INDEX, 2);
		udc_writeb(USB_CTRL_1, 0x1);
		return 0;
	}
	udc_writeb(USB_EP_INDEX, 2);
	udc_writel(USB_BUFFER_FORBIDDEN, udc_readl(USB_BUFFER_FORBIDDEN) | (0x1<<1));

	while (udc_readw(USB_CTRL_1) & 0x1)
		;
	buf = req->req.buf + req->req.actual;
	phys = ep->bufphys;
	memcpy_toio((volatile void *)(ep->bufaddr), buf, count);
#ifdef UDC_L2_ENABLE
	ak88_l2_clr_status(udc->ep[2].l2_buf_id);
	ak88_l2_combuf_dma(phys, udc->ep[2].l2_buf_id, count, MEM2BUF, false);
	ak88_l2_combuf_wait_dma_finish(udc->ep[2].l2_buf_id);

	ak88_l2_set_status(udc->ep[2].l2_buf_id, 8);
#else
	/* l2 xfer */
	usb_l2_ep2(L2_USB_EP2, (unsigned int)ep->bufaddr, phys, count, ep->is_in);
#endif

	usb_dma_send_mode0(udc, 2, (unsigned int)buf, count);
	
	udc_writel(USB_EP2_NUM, count);
	udc_writel(USB_PREREAD_START, udc_readl(USB_PREREAD_START) | (0x1<<1));
	udc_writeb(USB_CTRL_1, 0x1);

	udc_writel(USB_BUFFER_FORBIDDEN, udc_readl(USB_BUFFER_FORBIDDEN) & ~0x1);

	req->req.actual += count;
	/* wait a tx complete int */
        /*
	 * if (is_last)
	 *         done(ep, req, 0);
         */

	/* return is_last; */
	ep->done = is_last;

#if 0
	while (udc_readw(USB_CTRL_1) & 0x1)
		;
#endif
	//printk("%x\n", udc_readw(USB_CTRL_1));
	//udelay(300);

	return 0;
}

static int read_ep3_fifo(struct ak880x_ep *ep, struct ak880x_request *req) /* ep3 */ 
{
	struct ak880x_udc *udc = ep->udc;
	unsigned char *buf;
	unsigned int csr;
	unsigned int count, bufferspace, is_done;
	dma_addr_t phys;

	bufferspace = req->req.length - req->req.actual;

	udc_writeb(USB_EP_INDEX, 2);
	while (udc_readw(USB_CTRL_1) & 0x1)
		;
	udc_writeb(USB_EP_INDEX, 3);
	csr = udc_readb(USB_CTRL_2);
	if ((csr & 0x1) == 0) {
		dbg("waiting bulkout data");
		return 0;
	}

	count = udc_readw(USB_EP_COUNT);
	if (count == 0) {
		dbg("USB_CTRL_2(0x%x), USB_EP_COUNT(%d), bufferspace(%d)",
				csr, count, bufferspace);
		dbg("what happened??");
		goto stall;
	} else if (count > ep->ep.maxpacket)
		count = ep->ep.maxpacket;

	if (count > bufferspace) {
		dbg("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}
	dbg("USB_CTRL_2(0x%x), USB_EP_COUNT(%d), bufferspace(%d)", csr, count, bufferspace);

	buf = req->req.buf + req->req.actual;
	phys = ep->bufphys;

#ifdef UDC_L2_ENABLE
	ak88_l2_set_status(udc->ep[3].l2_buf_id, 8);

	ak88_l2_combuf_dma(phys, udc->ep[3].l2_buf_id, count, BUF2MEM, false);
	ak88_l2_combuf_wait_dma_finish(udc->ep[3].l2_buf_id);
#else
	/* l2 xfer */
	usb_l2_ep3(L2_USB_EP3, buf, phys, count, 0);
#endif

	memcpy_fromio(buf, (volatile void *)(ep->bufaddr), count);
	
stall:
	udc_writeb(USB_CTRL_2, csr & ~0x1);
	/* udc_writeb(USB_EP_INDEX, 3); */

	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;

	ep->done = is_done;
	if (is_done) {
		done(ep, req, 0);
	}


	return is_done;
}

static int ak880x_get_frame(struct usb_gadget *gadget)
{
	dbg("");
	return 0;
}
static int ak880x_wakeup(struct usb_gadget *gadget)
{
	dbg("");
	return 0;
}

static int ak880x_pullup(struct usb_gadget *gadget, int is_on)
{
	dbg("is_on(%d)", is_on);

	return 0;
}
static int ak880x_vbus_session(struct usb_gadget *gadget, int is_active)
{
	dbg("");
	return 0;
}
static int ak880x_set_selfpowered(struct usb_gadget *gadget, int is_on)
{
	dbg("");
	return 0;
}
static const struct usb_gadget_ops ak880x_udc_ops = {
	.get_frame		= ak880x_get_frame,
	.wakeup			= ak880x_wakeup,
	.set_selfpowered	= ak880x_set_selfpowered,
	.vbus_session		= ak880x_vbus_session,
	.pullup			= ak880x_pullup,
};

static void ep2_work(struct work_struct *work)
{
	struct ak880x_request *req = NULL;
	struct ak880x_udc *udc = &controller;
	struct ak880x_ep *ep = &udc->ep[2];

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak880x_request, queue);

	mutex_lock(&nand_lock);
	if (req)
		req->status = write_ep2_fifo(ep, req);
	else
		dbg("something happend");
	mutex_unlock(&nand_lock);
}

static void ep3_work(struct work_struct *work)
{
	struct ak880x_request *req = NULL;
	struct ak880x_udc *udc = &controller;
	struct ak880x_ep *ep = &udc->ep[3];

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak880x_request, queue);

	mutex_lock(&nand_lock);
	if (req)
		req->status = read_ep3_fifo(ep, req);
	else
		dbg("something happend");
	mutex_unlock(&nand_lock);
}

static int ak880x_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct ak880x_ep *ep = container_of(_ep, struct ak880x_ep, ep);
	struct ak880x_udc *udc = ep->udc;
	int tmp, maxpacket;

	if (!_ep || !ep
			|| !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| (maxpacket = le16_to_cpu(desc->wMaxPacketSize)) == 0
			|| maxpacket > ep->maxpacket) {
		dbg("bad ep or descriptor");
		dbg("%p, %p, %p, %p", _ep, ep, desc, ep->desc);
		return -EINVAL;
	}

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		dbg("bogus udcice state\n");
		return -ESHUTDOWN;
	}

	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		dbg("only one control endpoint\n");
		return -EINVAL;
	case USB_ENDPOINT_XFER_INT:
		if (maxpacket > EP1_FIFO_SIZE)
			dbg("maxpacket too large");
		break;
	case USB_ENDPOINT_XFER_BULK:
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
		case 64:
		case 512: /* for usb20 */ 
			break;
		default:
			dbg("bogus maxpacket %d\n", maxpacket);
			return -EINVAL;
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		dbg("USB_ENDPOINT_XFER_ISOC");
		break;
	}
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	/* ep->stopped = 0; */
	ep->stopped = 1;
	ep->desc = (struct usb_endpoint_descriptor *)desc;

	dbg("%s, maxpacket(%d), desc->bEndpointAddress (0x%x) is_in(%d)",
		       	_ep->name, maxpacket, desc->bEndpointAddress, ep->is_in);
	
	ep_irq_disable(udc);
	if (!strcmp(_ep->name, udc->ep[1].ep.name)) { /* ep1 -- int */ 
		dbg("");
		udc_writeb(USB_EP_INDEX, 1);
		udc_writeb(USB_CTRL_1+1, 0);
		udc_writew(USB_RX_MAX, 64);
	} else if (!strcmp(_ep->name, udc->ep[2].ep.name)) { /* ep2 -- tx */ 
		dbg("");
		INIT_WORK(&ep->work, ep2_work);
		usb_l2_init(L2_USB_EP2, ep->is_in);
		udc_writeb(USB_EP_INDEX, 2);
		udc_writeb(USB_CTRL_1+1, 1<<5);
		udc_writew(USB_TX_MAX, 512);
	} else if (!strcmp(_ep->name, udc->ep[3].ep.name)){ /* ep3 -- rx */ 
		dbg("");
		INIT_WORK(&ep->work, ep3_work);
		usb_l2_init(L2_USB_EP3, ep->is_in);
		udc_writeb(USB_EP_INDEX, 3);
		udc_writeb(USB_CTRL_1+1, 0);
		udc_writew(USB_RX_MAX, 512);
		udc_writeb(USB_CTRL_2, udc_readb(USB_CTRL_2) & ~(0x1));
	} else {
		printk("Invalid ep");
		return -EINVAL;
	}

	ep->irq_enable = 1;

	ep_irq_enable(udc);

	return 0;
}

static int ak880x_ep_disable (struct usb_ep * _ep)
{
	struct ak880x_ep *ep = container_of(_ep, struct ak880x_ep, ep);
	struct ak880x_udc *udc = ep->udc;
	struct ak880x_request *req;

	if (ep == &ep->udc->ep[0]) /* ep0 */ 
		return -EINVAL;

	dbg("%s", _ep->name);
	ep_irq_disable(udc);

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct ak880x_request, queue);
		done(ep, req, -ESHUTDOWN);
	}

	ep->irq_enable = 0;
	ep->stopped = 1;
	ep->desc = NULL;
	ep_irq_enable(udc);
	return 0;
}

static struct usb_request *
	ak880x_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct ak880x_request *req;

	dbg("%s", _ep->name);
	req = kzalloc(sizeof (struct ak880x_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void ak880x_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ak880x_request *req;

	dbg("%s", _ep->name);
	req = container_of(_req, struct ak880x_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static int ak880x_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ak880x_request	*req;
	struct ak880x_ep	*ep;
	struct ak880x_udc	*udc;
	int			status;

	req = container_of(_req, struct ak880x_request, req);
	ep = container_of(_ep, struct ak880x_ep, ep);

	if (!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue)) {
		/* dbg("%s invalid request", _ep->name); */
		printk("%s invalid request\n", _ep->name);
		return -EINVAL;
	}

        /*
	 * if (!_ep || (!ep->desc)) {
	 *         dbg("invalid ep\n");
	 *         return -EINVAL;
	 * }
         */

	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		/* dbg("invalid device\n"); */
		printk("invalid device\n");
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	ep_irq_disable(udc);

	ep->stopped = 0;
	dbg("%s queue is_in(%d)", ep->ep.name, ep->is_in);
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->ep.name == ep0name) {
			if (ep->is_in) {
				dbg("ep0 req(0x%p)", req);
				list_add_tail(&req->queue, &ep->queue);
				status = write_ep0_fifo(ep, req);
			} else {
#if 1
				dbg("why?");
                                /*
				 * udc_writeb(USB_EP_INDEX, 0);
				 * udc_writeb(USB_CTRL_1, 0x1<<6); [> To clear the RxPktRdy Bit <] 
				 * udc_writeb(USB_CTRL_1, 0x1<<3); [> data end <]
                                 */
				ep->stopped = 1;
				status = 1;
#else
				status = read_ep0_fifo(ep, req); /* ??? */
#endif
			}
		} else {
			list_add_tail(&req->queue, &ep->queue);
			if (!strcmp(_ep->name, udc->ep[1].ep.name)) { /* ep1 */ 
				dbg("ep1");
				status = write_ep1_fifo(ep, req);
			} else if (!strcmp(_ep->name, udc->ep[2].ep.name)) { /* ep2 */ 
				queue_work(ep_wqueue, &ep->work);
			} else if (!strcmp(_ep->name, udc->ep[3].ep.name)){ /* ep3 */ 
				queue_work(ep_wqueue, &ep->work);
			} else {
				printk("Invalid ep");
				status = -EINVAL;
				goto stall;
			}
		}
	} else {
		status = 0;
		list_add_tail(&req->queue, &ep->queue);
		dbg("waiting for %s int", ep->ep.name);
	}

stall:

	ep_irq_enable(udc);
	/* return (status < 0) ? status : 0; */
	return 0;
}


static int ak880x_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ak880x_ep	*ep;
	struct ak880x_request	*req;

	dbg("dequeue");

	ep = container_of(_ep, struct ak880x_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req)
		return -EINVAL;

	done(ep, req, -ECONNRESET);
	return 0;
}

static int ak880x_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct ak880x_ep	*ep = container_of(_ep, struct ak880x_ep, ep);
	struct ak880x_udc	*udc = ep->udc;
	u32 __iomem	*creg;
	u32		csr;
	int		status = 0;

	dbg("");
	/* something have to do */
	/* ep->stopped = value ? 1 : 0; */

	return 0;
}

static const struct usb_ep_ops ak880x_ep_ops = {
	.enable		= ak880x_ep_enable,
	.disable	= ak880x_ep_disable,
	.alloc_request	= ak880x_ep_alloc_request,
	.free_request	= ak880x_ep_free_request,
	.queue		= ak880x_ep_queue,
	.dequeue	= ak880x_ep_dequeue,
	.set_halt	= ak880x_ep_set_halt,
	// there's only imprecise fifo status reporting
};

static struct ak880x_udc controller = {
	.gadget = {
		.ops	= &ak880x_udc_ops,
		.ep0	= &controller.ep[0].ep,
		.name	= driver_name,
		.dev	= {
		       .init_name = "gadget",
		}
	},
	.ep[0] = {
		.ep = {
			.name		= ep0name,//ep_name[0],
			.ops		= &ak880x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP0_FIFO_SIZE,
	},
	.ep[1] = {
		.ep = {
			.name	= "ep1-int",//ep_name[1],
			.ops	= &ak880x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP1_FIFO_SIZE,
	},
	.ep[2] = {
		.ep = {
			.name	= "ep2in-bulk",//ep_name[2],
			.ops	= &ak880x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP2_FIFO_SIZE,
	},
	.ep[3] = {
		.ep = {
			/* could actually do bulk too */
			.name	= "ep3out-bulk",//ep_name[3],
			.ops	= &ak880x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP3_FIFO_SIZE,
	},
	.ep[4] = {
		.ep = {
			.name	= "ep4-iso",//ep_name[4],
			.ops	= &ak880x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP4_FIFO_SIZE,
	},
};

static void handle_setup(struct ak880x_udc *udc, struct ak880x_ep *ep)
{
	unsigned rxcount;
	struct usb_ctrlrequest creq;
	int status = 0;
	int timeout;

#if 0
	int regval, csr;
	struct ak880x_ep *ep0 = &udc->ep[0];
	struct ak880x_request *req;

	if (!list_empty(&ep0->queue)) {
		req = list_entry(ep0->queue.next, struct ak880x_request, queue);
		dbg("ep0 req waiting");
	} else {
		req = NULL;
	}
#endif

	rxcount = udc_readw(USB_EP_COUNT);
	if (rxcount == 0) {
		dbg("rxcount(0) end");
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
		return;
	}
	memcpy(&creq, (void *)EP0_L2_ADDR, sizeof(creq));
	udc_writeb(USB_CTRL_1, 0x1<<6); /* clear rx flag */ 
	dbg("rxcount(%d), brequesttype(0x%x), brequest(0x%x)",
			rxcount, creq.bRequestType, creq.bRequest);

	(creq.bRequestType & USB_DIR_IN) ? (ep->is_in = 1) : (ep->is_in = 0);

	switch (creq.bRequest) {
		case USB_REQ_SET_ADDRESS:
			/* udc->addr = 0; */
			dbg("USB_REQ_SET_ADDRESS(%d)", creq.wValue);
			/* udc_writeb(USB_CTRL_1, 0x1<<6 | 0x1<<3); */
			udc_writeb(USB_CTRL_1, 0x1<<3);
			timeout = 5000;
			 /* waiting for next interrupt */ 
			while (!(udc_readb(USB_INTERRUPT_1) & 0x1) && timeout) {timeout--;}
			udc_writeb(USB_FUNCTION_ADDR, creq.wValue);
			udc->addr = creq.wValue;
			break;
		default:
			if (udc->driver)  
				status = udc->driver->setup(&udc->gadget, &creq);
			 else 
				status = -ENODEV;
	}
	if (status < 0) {
		dbg("stall, status(%d)", status);
		if (status = -EOPNOTSUPP) {
			dbg("What will happened");
			/* udc_writeb(USB_CTRL_1,  0x1<<3); */
		}
                /*
		 * udc_writew(USB_CTRL_1,  0x1<<8); [> flush fifo <]
		 * udc_writeb(USB_CTRL_1,  0x1<<5); [> stall <]
                 */
	} else if (status == (999 + 256)) { /* delayed status ???*/
		dbg("delayed, status(%d)", status);
                /*
		 * udc_writeb(USB_CTRL_1,  0x1<<8); [> flush fifo <]
		 * udc_writeb(USB_CTRL_1,  0x1<<5); [> stall <]
                 */

		/* udc_writeb(USB_CTRL_1, 0x1<<6 | 0x1<<3); */
	}

}

static void handle_ep0(struct ak880x_udc *udc)
{
	int csr;
	struct ak880x_ep *ep0 = &udc->ep[0];
	struct ak880x_request *req = NULL;
	if (!list_empty(&ep0->queue)) 
		req = list_entry(ep0->queue.next, struct ak880x_request, queue);
#if 0
	struct ak880x_request *req;

	if (!list_empty(&ep0->queue)) {
		req = list_entry(ep0->queue.next, struct ak880x_request, queue);
		dbg("ep0 req waiting");
	} else {
		req = NULL;
	}
#endif

	udc_writeb(USB_EP_INDEX, 0);
	csr = udc_readb(USB_CTRL_1);
	dbg("csr(0x%x)", csr);
	if (csr == 0) { /* tx int */ 
		dbg(" This is a tx interrupt");
		if (!ep0->done && req) {
			dbg("linux ep0 overrun??");
			dbg("ep0 req(0x%p)", req);
			write_ep0_fifo(ep0, req); /* usb20 for linux */ 
		}
		return;
	}

	if (csr & 0x1<<3) {
		dbg("data end");
	} 
	if (csr & 0x1<<4) {
		dbg("A control transaction ends before the DataEnd bit has been set");
		udc_writeb(USB_CTRL_1, 0x1<<7);
		/* do something else? */
	}
	if (csr & 0x1<<0) {
		dbg("A data packet has been received");
                /*
		 * if (req) 
		 *         read_ep0_fifo(ep0, req);
		 * else
                 */
		handle_setup(udc, ep0);
	} else if (~csr & 0x1<<1) {
		dbg("The CPU has not loaded a data packet into the FIFO");
	} else {
		dbg("what is it?");
	}

        /*
	 * if (csr & 0x1<<2) {
	 *         dbg("A STALL handshake has been transmitted");
	 *         udc_writeb(USB_CTRL_1, 0);
	 * }
         */
}

/* not ep0 */
static void handle_ep(struct ak880x_ep *ep)
{
	struct ak880x_request *req;
	struct ak880x_udc *udc = ep->udc;
	unsigned int csr;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next,
			struct ak880x_request, queue);
	else {
		dbg("%s: no req waiting", ep->ep.name);
		req = NULL;
	}
	
	if (!strcmp(ep->ep.name, udc->ep[1].ep.name)) { /* ep1 */ 
		dbg("ep1");
	} else if (ep->is_in) { /* ep2 */ 

		udc_writeb(USB_EP_INDEX, 2);
		csr = udc_readb(USB_CTRL_1);
		dbg("ep2 csr(0x%x), req(0x%p)", csr, req);

                /*
		 * if ((csr & 0x1) && req) {
		 *         write_ep2_fifo(ep, req);
		 * }
                 */
		/* udc_writeb(USB_CTRL_1, 0x1); */
		if (csr & 0x1<<2) {
			udc_writeb(USB_CTRL_1, csr & ~(0x1<<2));
		}
		if (csr & 0x1<<5) {
			udc_writeb(USB_CTRL_1, csr & ~(0x1<<5));
		}

		if (req) {
			if (ep->done) {
				done(ep, req, 0);
				if (!list_empty(&ep->queue)) {
					dbg("do next queue");
					req = list_entry(ep->queue.next, struct ak880x_request, queue);
					queue_work(ep_wqueue, &ep->work);
				}
			} else {
				queue_work(ep_wqueue, &ep->work);
			}

		}
	} else { /* ep3 */ 
		udc_writeb(USB_EP_INDEX, 3);
		csr = udc_readb(USB_CTRL_2);
		dbg("ep3 csr(0x%x)", csr);

		udc_writeb(USB_EP_INDEX, 3);
		if (req && (csr & 0x1<<1))
		{
			//printk("sb1 %d\n", udc_readw(USB_EP_COUNT));
			queue_work(ep_wqueue, &ep->work);
		}
		//else if ((csr & 0x2) == 0)
			//printk("sb2 %d\n", udc_readw(USB_EP_COUNT));
	}
}

static void udc_disconnect(struct ak880x_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak880x_ep *ep = &udc->ep[i];
		struct ak880x_request *req;

		ep->stopped = 1;

		// terminer chaque requete dans la queue
		if (list_empty(&ep->queue))
			continue;

		while (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct ak880x_request, queue);
			done(ep, req, -ESHUTDOWN);
		}
	}

	if (driver)
		driver->disconnect(&udc->gadget);

	udc_reinit(udc);
}

static void udc_reset(struct ak880x_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

#ifdef USB_11
	udc_writel(USB_MODE_STATUS, 1);
	udc_writew(USB_POWER_CTRL, 0);
#else
	udc_writel(USB_MODE_STATUS, 0);
	udc_writew(USB_POWER_CTRL, 0x1<<5);
#endif

	udc_writeb(USB_FUNCTION_ADDR, 0);
	udc_writeb(USB_INTERRUPT_USB, ~(0x1<<3));
	udc_writeb(USB_INTERRUPT_TX, 0x1<<0); /* ep0 */ 
	udc->ep[0].irq_enable = 1;
	
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak880x_ep *ep = &udc->ep[i];
		struct ak880x_request *req;

		ep->stopped = 1;

		if (list_empty(&ep->queue))
			continue;

		while (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct ak880x_request, queue);
			done(ep, req, -ECONNRESET);
		}
	}

}

static irqreturn_t udc_irqhandler(int irq, void *_udc)
{
	struct ak880x_udc *udc = _udc;
	short status_1, status_2;
	char status_int;

	status_int = udc_readb(USB_INTERRUPT_COMM);
	if (status_int & 0x1<<2) {
		/* dbg("status_int(0x%x), reset", status_int);	 */
		printk("\n\nstatus_int(0x%x), reset\n\n", status_int);	
		if (usb_detect) {
			usb_exist = 1;
			usb_detect = 0;
			if (!udc->driver) {
				udc_enable(udc, 0);
				return IRQ_HANDLED;
			}
		}
#ifdef USB_11
		udc->gadget.speed = USB_SPEED_FULL;
#else
		udc->gadget.speed = USB_SPEED_HIGH;
#endif
		udc_reset(udc);
		goto done;
	} else if(status_int & 0x1<<1) { /* resume */ 
		dbg("status_int(0x%x)", status_int);	
		dbg("resume");
		goto done;
	} else if(status_int & 0x1<<0) { /* suspend */ 
		dbg("status_int(0x%x)", status_int);	
		dbg("suspend");
		udc_disconnect(udc);
		goto done;
	}

	status_1 = udc_readb(USB_INTERRUPT_1);
	status_2 = udc_readb(USB_INTERRUPT_2);
	dbg("status_int(0x%x), status_1(0x%x), status_2(0x%x)", status_int, status_1, status_2);
	if (status_1 & 0x1<<0) {
		handle_ep0(udc);
	}
	if (status_1 & 0x1<<2) {
		dbg("endpoint2");
		handle_ep(&udc->ep[2]);
	}
	if (status_1 & 0x1<<4) {
		dbg("endpoint4");
		handle_ep(&udc->ep[4]);
	}

	if (status_2 & 0x1<<1) {
		dbg("endpoint 1");
		handle_ep(&udc->ep[1]);
	}
	if (status_2 & 0x1<<3) {
		dbg("endpoint3");
		handle_ep(&udc->ep[3]);
	}
done:
	return IRQ_HANDLED;
}

void udc_enable(struct ak880x_udc *udc, int enable)
{
	if (enable) {
		reset_usbcontroller();
		clk_enable(udc->clk);
		rMULFUN_CON &= ~0x7;
		rMULFUN_CON |= 6;
#ifdef USB_11
		udc_writel(USB_MODE_STATUS, 1);
		udc_writew(USB_POWER_CTRL, 0);
#else
		udc_writel(USB_MODE_STATUS, 0);
		udc_writew(USB_POWER_CTRL, 0x1<<5);
#endif
	} else {
		rMULFUN_CON &= ~0x7;
		clk_disable(udc->clk);
		reset_usbcontroller();
	}
}

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct ak880x_udc	*udc = &controller;
	int		retval;

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->setup) {
		dbg("bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		dbg("UDC already has a gadget driver\n");
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	//udc->gadget.dev.driver_data = &driver->driver;
	udc->enabled = 1;

	retval = driver->bind(&udc->gadget);
	if (retval) {
		dbg("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		//udc->gadget.dev.driver_data = NULL;
		udc->enabled = 0;
		return retval;
	}

	local_irq_disable();
	udc_enable(udc, 1);
	local_irq_enable();

	dbg("bound to %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct ak880x_udc *udc = &controller;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	local_irq_disable();
	udc_enable(udc, 0);
	local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->gadget.dev.driver = NULL;
	//udc->gadget.dev.driver_data = NULL;
	udc->driver = NULL;


	dbg("unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);


static void udc_reinit(struct ak880x_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak880x_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		/* ep->stopped = 0; */
		ep->stopped = 1;
		ep->ep.maxpacket = ep->maxpacket;
		// initialiser une queue par endpoint
		INIT_LIST_HEAD(&ep->queue);
	}
}

static int __init ak880x_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ak880x_udc *udc = &controller;
	struct resource	*res;
	int retval;

	if (pdev->num_resources < 2) {
		dbg("invalid num_resources\n");
		return -ENODEV;
	}
	if ((pdev->resource[0].flags != IORESOURCE_MEM)
			|| (pdev->resource[1].flags != IORESOURCE_IRQ)) {
		dbg("invalid resource type\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start,
			res->end - res->start + 1,
			driver_name)) {
		dbg("someone's using UDC memory\n");
		return -EBUSY;
	}

	udc->baseaddr = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!udc->baseaddr) {
		retval = -ENOMEM;
		goto fail0a;
	}
	/* init software state */
	udc->gadget.dev.parent = dev;
	udc->enabled = 0;

	udc_reinit(udc);

	/* get interface and function clocks */
	udc->clk = clk_get(dev, "udc_clk");
	if (IS_ERR(udc->clk)) {
		dbg("clocks missing\n");
		retval = -ENODEV;
		/* NOTE: we "know" here that refcounts on these are NOPs */
		goto fail0b;
	}

	retval = device_register(&udc->gadget.dev);
	if (retval < 0)
		goto fail0b;

	ep_wqueue = create_workqueue("ak880x_udc");
	/* request UDC and maybe VBUS irqs */
	udc->mcu_irq = platform_get_irq(pdev, 0);

	reset_usbcontroller();
	retval = request_irq(udc->mcu_irq, udc_irqhandler,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dbg("request irq %d failed\n", udc->mcu_irq);
		goto fail1;
	}
	platform_set_drvdata(pdev, udc);

	dbg("Build at %s %s", __DATE__, __TIME__);

	/* for g_serial.ko */
	udc->ep[2].bufaddr = dma_alloc_coherent(NULL, 512, &udc->ep[2].bufphys, DMA_TO_DEVICE);
	udc->ep[3].bufaddr = dma_alloc_coherent(NULL, 512, &udc->ep[3].bufphys, DMA_FROM_DEVICE);

#ifdef UDC_L2_ENABLE
		/* USB slave L2 buffer initialization */
		udc->ep[2].l2_buf_id = ak88_l2_alloc(ADDR_USB_BULKOUT);
		udc->ep[3].l2_buf_id = ak88_l2_alloc(ADDR_USB_BULKIN);
#endif

	return 0;

	free_irq(udc->mcu_irq, udc);
fail1:
	device_unregister(&udc->gadget.dev);
fail0b:
	iounmap(udc->baseaddr);
fail0a:
	release_mem_region(res->start, res->end - res->start + 1);
	dbg("%s probe failed, %d\n", driver_name, retval);
	return retval;
}

static int __exit ak880x_udc_remove(struct platform_device *pdev)
{
	struct ak880x_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	if (udc->driver)
		return -EBUSY;

	free_irq(udc->mcu_irq, udc);
	device_unregister(&udc->gadget.dev);

	destroy_workqueue(ep_wqueue);

	iounmap(udc->baseaddr);

#ifdef UDC_L2_ENABLE
		/* USB slave L2 buffer initialization */
	ak88_l2_free(ADDR_USB_BULKIN);
	ak88_l2_free(ADDR_USB_BULKOUT);
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

static void ak880x_udc_shutdown(struct platform_device *dev)
{
}

#ifdef CONFIG_PM
static int ak880x_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct ak880x_udc *udc = platform_get_drvdata(pdev);

	return 0;
}

static int ak880x_udc_resume(struct platform_device *pdev)
{
	struct ak880x_udc *udc = platform_get_drvdata(pdev);

	/* something to do */

	return 0;
}
#else
#define	ak880x_udc_suspend	NULL
#define	ak880x_udc_resume	NULL
#endif

static struct platform_driver ak880x_udc_driver = {
	.remove		= __exit_p(ak880x_udc_remove),
	.shutdown	= ak880x_udc_shutdown,
	.suspend	= ak880x_udc_suspend,
	.resume		= ak880x_udc_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __init udc_init_module(void)
{
	printk("AK880X UDC Driver, (c) 2010 ANYKA\n");

	return platform_driver_probe(&ak880x_udc_driver, ak880x_udc_probe);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	platform_driver_unregister(&ak880x_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("AK880X udc driver");
MODULE_AUTHOR("Anyka");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak880x_udc");
