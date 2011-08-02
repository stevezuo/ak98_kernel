/*
 * ak980x_udc -- driver for ak980x USB peripheral controller
 * Features
 * The USB 2.0 HS OTG has following features:
 *      •   compliant with USB Specification Version 2.0 (HS) and On-The-Go supplement to
 *	  the USB 2.0 specification
 *      •   operating as the host in point-to-point communications with another USB function
 *	  or as a function controller for a USB peripheral
 *      •   supporting UTMI+ Level 2 Transceiver Interface
 *      •   4 Transmit/Receive endpoints in addition to Endpoint 0
 *      •   3 DMA channels
 * AUTHOR ANYKA Zhang Jingyuan
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
#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
#include <linux/ak98_freq_policy.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/dma-mapping.h>

#include <mach/regs-comm.h>
#include <mach/l2.h>

#include "ak98_udc_full.h"

#define AK98_UDC_EPS 5


#if 0
#define dbg(fmt, arg...) printk("%s(%d): " fmt "\n", __func__, __LINE__, ##arg)
#else
#define dbg(fmt, arg...)
#endif

static const char ep0name[] = "ep0";
static const char driver_name[] = "ak98_udc";

#define udc_readb(reg)	__raw_readb(udc->baseaddr + (reg))
#define udc_readw(reg)	__raw_readw(udc->baseaddr + (reg))
#define udc_readl(reg)	__raw_readl(udc->baseaddr + (reg))
#define udc_writeb(reg, val) __raw_writeb(val, udc->baseaddr + (reg))
#define udc_writew(reg, val) __raw_writew(val, udc->baseaddr + (reg))
#define udc_writel(reg, val) __raw_writel(val, udc->baseaddr + (reg))

static struct ak980x_udc controller;
/* static struct ak980x_udc *udc = &controller; */
struct workqueue_struct *ep_wqueue;

volatile int usb_detect;
volatile int usb_exist;
EXPORT_SYMBOL(usb_detect);
EXPORT_SYMBOL(usb_exist);

unsigned int dma_rx;
unsigned int dma_tx;
dma_addr_t phys_rx;
dma_addr_t phys_tx;

static volatile char flag = 0;
static u32 start = 0;

static void ep_irq_enable(struct usb_ep *_ep)
{
	static struct ak980x_udc *udc = &controller;
	
	if (strcmp(_ep->name, udc->ep[1].ep.name))
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) | (0x1<<1));

	if (strcmp(_ep->name, udc->ep[2].ep.name)) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) | (0x1<<2));
	
	if (strcmp(_ep->name, udc->ep[3].ep.name)) 
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) | (0x1<<3));

	if (strcmp(_ep->name, udc->ep[4].ep.name)) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) | (0x1<<4));
	
	if (strcmp(_ep->name, udc->ep[5].ep.name)) 
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) | (0x1<<5));
}

static void ep_irq_disable(struct usb_ep *_ep)
{
	static struct ak980x_udc *udc = &controller;
	
	if (strcmp(_ep->name, udc->ep[1].ep.name))
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) & ~(0x1<<1));

	if (strcmp(_ep->name, udc->ep[2].ep.name)) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) & ~(0x1<<2));
	
	if (strcmp(_ep->name, udc->ep[3].ep.name)) 
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) & ~(0x1<<3));

	if (strcmp(_ep->name, udc->ep[4].ep.name)) 
		udc_writew(USB_INTERRUPT_TX, udc_readw(USB_INTERRUPT_TX) & ~(0x1<<4));
	
	if (strcmp(_ep->name, udc->ep[5].ep.name)) 
		udc_writew(USB_INTERRUPT_RX, udc_readw(USB_INTERRUPT_RX) & ~(0x1<<5));
}

static inline void reset_usbcontroller(void)
{
	rCLK_CON1 |= (0x1<<31);
	rCLK_CON1 &= ~(0x1<<31);
}

static void done(struct ak980x_ep *ep, struct ak980x_request *req, int status)
{
	unsigned	 stopped = ep->stopped;
	//struct ak980x_udc *udc = ep->udc;

	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;

	dbg("%s done, req.status(%d)", ep->ep.name, req->req.status);
}

static int write_ep0_fifo(struct ak980x_ep *ep, struct ak980x_request *req)
{
	int i;
	unsigned total, count, is_last;
	struct ak980x_udc *udc = ep->udc;
	unsigned char *buf;

	total = req->req.length - req->req.actual;

	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}

	dbg("is_last(%d), count(%d), total(%d), actual(%d), length(%d)", 
			is_last, count, total, req->req.actual, req->req.length);

	udc_writeb(USB_EP_INDEX, 0);
	if (count == 0) {
		dbg(" count == 0 ");
		udc_writel(USB_EP0_NUM, count&0x7f);
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<1);
		udc->ep0state = EP0_IDLE;
		done(ep, req, 0);
		return 1;
	}

	buf = req->req.buf + req->req.actual;
	for (i = 0; i < count; i++)
		udc_writeb(USB_EP0_FIFO, buf[i]);

	udc_writel(USB_EP0_NUM, count&0x7f); /* 7bits */ 
	if (is_last) {
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<1);
	} else {
		udc_writeb(USB_CTRL_1, 0x1<<1);
	}

	req->req.actual += count;
	if (is_last) {
		udc->ep0state = EP0_IDLE;
		done(ep, req, 0);
	}

	ep->done = is_last;
	return is_last;
	/* return 1; */
}

static int read_ep0_fifo(struct ak980x_ep *ep, struct ak980x_request *req)
{
	int i;
	unsigned total, count, is_last;
	struct ak980x_udc *udc = ep->udc;
	unsigned char *buf;

	total = req->req.length - req->req.actual;

	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}

	dbg("is_last(%d), count(%d), total(%d), actual(%d), length(%d)", 
			is_last, count, total, req->req.actual, req->req.length);

	udc_writeb(USB_EP_INDEX, 0);
	if (count == 0) {
		dbg(" count == 0 ");
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
		udc->ep0state = EP0_IDLE;
		done(ep, req, 0);
		return 1;
	}

	buf = req->req.buf + req->req.actual;
	for (i = 0; i < count; i++)
		buf[i] = udc_readb(USB_EP0_FIFO);
 
	if (is_last) {
		udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
	} else {
		udc_writeb(USB_CTRL_1, 0x1<<6);
	}

	req->req.actual += count;
	if (is_last) {
		udc->ep0state = EP0_IDLE;
		done(ep, req, 0);
	}

	ep->done = is_last;
	return is_last;
	/* return 1; */
}


static int write_ep1_fifo(struct ak980x_ep *ep, struct ak980x_request *req)
{
	return 1;
}

static int write_ep2_fifo(struct ak980x_ep *ep, struct ak980x_request *req) /* ep2 */ 
{
	struct ak980x_udc *udc = ep->udc;
	unsigned total, count, is_last;
	unsigned char *buf;
	//dma_addr_t phys;
	int dma = 0, i;

	total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket <= total) {
		count = ep->ep.maxpacket;
		is_last = (total == ep->ep.maxpacket) && !req->req.zero;
#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
		dma = 1;
#endif
#endif
	} else {
		count = total;
		is_last = count < ep->ep.maxpacket;
		dma = 0;
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

	buf = req->req.buf + req->req.actual;
	//phys = ep->bufphys;

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	if (dma == 1) {
		dma_tx = total - (total % ep->ep.maxpacket);
		phys_tx = dma_map_single(NULL, buf, dma_tx, DMA_TO_DEVICE);
		if (phys_tx == 0) {
			printk("tx dma_map_single error!\n");
			goto cpu;
		}

		udc_writeb(USB_CTRL_1_2, (1<<2) | (1<<4) | (1<<5) | (1<<7));
		
		//send data to l2
		ak98_l2_clr_status(ep->l2_buf_id);
		ak98_l2_combuf_dma(phys_tx, ep->l2_buf_id, dma_tx, MEM2BUF, false);
		udc_writel(USB_DMA_ADDR1, 0x70000000);
		udc_writel(USB_DMA_COUNT1, dma_tx);
		udc_writel(USB_DMA_CTRL1, (USB_ENABLE_DMA | USB_DIRECTION_TX | USB_DMA_MODE1 | USB_DMA_INT_ENABLE| (USB_EP2_INDEX<<4) | USB_DMA_BUS_MODE3));
		
		ep->done = 0;
		return 0;
	}
#endif
#endif

cpu:
	for (i = 0; i < count; i++)
		udc_writeb(USB_EP2_FIFO, buf[i]);

	udc_writeb(USB_CTRL_1, 0x1);

	req->req.actual += count;
	/* wait a tx complete int */
        /*
	 * if (is_last)
	 *         done(ep, req, 0);
         */

	/* return is_last; */
	ep->done = is_last;

	return 0;
}

static int read_ep3_fifo(struct ak980x_ep *ep, struct ak980x_request *req) /* ep3 */ 
{
	struct ak980x_udc *udc = ep->udc;
	unsigned char *buf;
	unsigned int csr, i;
	unsigned int count, bufferspace, is_done;
	//dma_addr_t phys;

	if (flag == 1)
		return 0;

	bufferspace = req->req.length - req->req.actual;

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
	//phys = ep->bufphys;

	for (i = 0; i < count; i++)
		buf[i] = udc_readb(USB_EP3_FIFO);

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	dma_rx = bufferspace - count;
	if (dma_rx >= ep->ep.maxpacket) {
		dma_rx -= (dma_rx % ep->ep.maxpacket);
		
		buf = req->req.buf + req->req.actual + count;
		phys_rx = dma_map_single(NULL, buf, dma_rx, DMA_FROM_DEVICE);
		if (phys_rx == 0) {
			printk("rx dma_map_single error!\n");
			goto stall;
		}

		req->req.actual += count;
		flag = 1;
		udc_writeb(USB_CTRL_2_2, udc_readb(USB_CTRL_2_2) | (1<<3) | (1<<5) | (1<<7));
		ak98_l2_combuf_dma(phys_rx, ep->l2_buf_id, dma_rx, BUF2MEM, false);
		udc_writel(USB_DMA_ADDR2, 0x71000000);
		udc_writel(USB_DMA_COUNT2, dma_rx);
		udc_writel(USB_DMA_CTRL2, (USB_ENABLE_DMA|USB_DIRECTION_RX|USB_DMA_MODE1|USB_DMA_INT_ENABLE|(USB_EP3_INDEX<<4)|USB_DMA_BUS_MODE3));
		udc_writeb(USB_EP_INDEX, 3);
		udc_writeb(USB_CTRL_2, csr & ~0x1);
		ep->done = 0;

		return 0;
	}
#endif
#endif
	
stall:
	udc_writeb(USB_CTRL_2, csr & ~0x1);

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

static int write_ep4_fifo(struct ak980x_ep *ep, struct ak980x_request *req) /* ep4 */ 
{
	struct ak980x_udc *udc = ep->udc;
	unsigned total, count, is_last;
	unsigned char *buf;
	//dma_addr_t phys;
	int dma = 0, i;

	total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket <= total) {
		count = ep->ep.maxpacket;
		is_last = (total == ep->ep.maxpacket) && !req->req.zero;
#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
		dma = 1;
#endif
#endif
	} else {
		count = total;
		is_last = count < ep->ep.maxpacket;
		dma = 0;
	}
	dbg("total(%d), count(%d), is_last(%d)", total, count, is_last);

	if (count == 0) { /* not support command */ 
		dbg("count == 0\n");
		ep->done = 1;
		udc_writeb(USB_EP_INDEX, 4);
		udc_writeb(USB_CTRL_1, 0x1);
		return 0;
	}
	udc_writeb(USB_EP_INDEX, 4);

	buf = req->req.buf + req->req.actual;
	//phys = ep->bufphys;

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	if (dma == 1) {
		dma_tx = total - (total % ep->ep.maxpacket);
		phys_tx = dma_map_single(NULL, buf, dma_tx, DMA_TO_DEVICE);
		if (phys_tx == 0) {
			printk("tx dma_map_single error!\n");
			goto cpu;
		}

		udc_writeb(USB_CTRL_1_2, (1<<2) | (1<<4) | (1<<5) | (1<<7));
		
		//send data to l2
		ak98_l2_clr_status(ep->l2_buf_id);
		ak98_l2_combuf_dma(phys_tx, ep->l2_buf_id, dma_tx, MEM2BUF, false);
		udc_writel(USB_DMA_ADDR3, 0x72000000);
		udc_writel(USB_DMA_COUNT3, dma_tx);
		udc_writel(USB_DMA_CTRL3, (USB_ENABLE_DMA | USB_DIRECTION_TX | USB_DMA_MODE1 | USB_DMA_INT_ENABLE| (USB_EP4_INDEX<<4) | USB_DMA_BUS_MODE3));
		
		ep->done = 0;
		return 0;
	}
#endif
#endif

cpu:
	for (i = 0; i < count; i++)
		udc_writeb(USB_EP4_FIFO, buf[i]);

	udc_writeb(USB_CTRL_1, 0x1);

	req->req.actual += count;
	/* wait a tx complete int */
        /*
	 * if (is_last)
	 *         done(ep, req, 0);
         */

	/* return is_last; */
	ep->done = is_last;

	return 0;
}

static int read_ep5_fifo(struct ak980x_ep *ep, struct ak980x_request *req) /* ep5 */ 
{
	struct ak980x_udc *udc = ep->udc;
	unsigned char *buf;
	unsigned int csr, i;
	unsigned int count, bufferspace, is_done;
	//dma_addr_t phys;

	if (flag == 1)
		return 0;

	bufferspace = req->req.length - req->req.actual;

	udc_writeb(USB_EP_INDEX, 5);
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
	//phys = ep->bufphys;

	for (i = 0; i < count; i++)
		buf[i] = udc_readb(USB_EP5_FIFO);

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	dma_rx = bufferspace - count;
	if (dma_rx >= ep->ep.maxpacket) {
		dma_rx -= (dma_rx % ep->ep.maxpacket);
		
		buf = req->req.buf + req->req.actual + count;
		phys_rx = dma_map_single(NULL, buf, dma_rx, DMA_FROM_DEVICE);
		if (phys_rx == 0) {
			printk("rx dma_map_single error!\n");
			goto stall;
		}

		req->req.actual += count;
		flag = 1;
		udc_writeb(USB_CTRL_2_2, udc_readb(USB_CTRL_2_2) | (1<<3) | (1<<5) | (1<<7));
		ak98_l2_combuf_dma(phys_rx, ep->l2_buf_id, dma_rx, BUF2MEM, false);
		udc_writel(USB_DMA_ADDR4, 0x73000000);
		udc_writel(USB_DMA_COUNT4, dma_rx);
		udc_writel(USB_DMA_CTRL4, (USB_ENABLE_DMA|USB_DIRECTION_RX|USB_DMA_MODE1|USB_DMA_INT_ENABLE|(USB_EP5_INDEX<<4)|USB_DMA_BUS_MODE3));
		udc_writeb(USB_EP_INDEX, 5);
		udc_writeb(USB_CTRL_2, csr & ~0x1);
		ep->done = 0;

		return 0;
	}
#endif
#endif
	
stall:
	udc_writeb(USB_CTRL_2, csr & ~0x1);

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

static int ak980x_get_frame(struct usb_gadget *gadget)
{
	dbg("");
	return 0;
}
static int ak980x_wakeup(struct usb_gadget *gadget)
{
	dbg("");
	return 0;
}

static int ak980x_pullup(struct usb_gadget *gadget, int is_on)
{
	struct ak980x_udc *udc = &controller;
	
	dbg("is_on(%d)", is_on);

	if (is_on) {
		clk_enable(udc->clk);
		
		rMULFUN_CON1 &= ~0x7;
		rMULFUN_CON1 |= 0x6;
	}
	else {
		rMULFUN_CON1 &= ~0x7;
		
		clk_disable(udc->clk);

		reset_usbcontroller();
	}

	return 0;
}

static int ak980x_vbus_session(struct usb_gadget *gadget, int is_active)
{
	dbg("");
	return 0;
}

static int ak980x_set_selfpowered(struct usb_gadget *gadget, int value)
{
	struct ak980x_udc *udc = &controller;

	dbg("%d", value);
	if (value)
		udc->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		udc->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);
	
	return 0;
}
static const struct usb_gadget_ops ak980x_udc_ops = {
	.get_frame		= ak980x_get_frame,
	.wakeup			= ak980x_wakeup,
	.set_selfpowered	= ak980x_set_selfpowered,
	.vbus_session		= ak980x_vbus_session,
	.pullup			= ak980x_pullup,
};

static void ep2_work(struct work_struct *work)
{
	struct ak980x_request *req = NULL;
	struct ak980x_udc *udc = &controller;
	struct ak980x_ep *ep = &udc->ep[2];
	unsigned long flags;

	local_irq_save(flags);
	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak980x_request, queue);

	if (req)
		req->status = write_ep2_fifo(ep, req);
	else
		dbg("something happend");
	local_irq_restore(flags);
}

static void ep3_work(struct work_struct *work)
{
	struct ak980x_request *req = NULL;
	struct ak980x_udc *udc = &controller;
	struct ak980x_ep *ep = &udc->ep[3];
	unsigned long flags;

	local_irq_save(flags);
	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak980x_request, queue);

	if (req)
		req->status = read_ep3_fifo(ep, req);
	else
		dbg("something happend");
	local_irq_restore(flags);
}

static void ep4_work(struct work_struct *work)
{
	struct ak980x_request *req = NULL;
	struct ak980x_udc *udc = &controller;
	struct ak980x_ep *ep = &udc->ep[4];
	unsigned long flags;
	
	local_irq_save(flags);
	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak980x_request, queue);

	if (req)
		req->status = write_ep4_fifo(ep, req);
	else
		dbg("something happend");
	local_irq_restore(flags);
}

static void ep5_work(struct work_struct *work)
{
	struct ak980x_request *req = NULL;
	struct ak980x_udc *udc = &controller;
	struct ak980x_ep *ep = &udc->ep[5];
	unsigned long flags;
	
	local_irq_save(flags);
	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct ak980x_request, queue);

	if (req)
		req->status = read_ep5_fifo(ep, req);
	else
		dbg("something happend");
	local_irq_restore(flags);
}

static int ak980x_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct ak980x_ep *ep = container_of(_ep, struct ak980x_ep, ep);
	struct ak980x_udc *udc = ep->udc;
	int tmp, maxpacket;
	unsigned long flags;

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
	
	local_irq_save (flags);

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
			_ep->maxpacket = maxpacket & 0x7ff;
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

	ep->stopped = 0;
	ep->desc = (struct usb_endpoint_descriptor *)desc;

	dbg("%s, maxpacket(%d), desc->bEndpointAddress (0x%x) is_in(%d)",
		       	_ep->name, maxpacket, desc->bEndpointAddress, ep->is_in);
	
	if (!strcmp(_ep->name, udc->ep[1].ep.name)) { /* ep1 -- int */ 
		dbg("");
		udc_writeb(USB_EP_INDEX, 1);
		udc_writeb(USB_CTRL_1_2, 0);
		udc_writew(USB_RX_MAX, 64);
	} else if (!strcmp(_ep->name, udc->ep[2].ep.name)) { /* ep2 -- tx */ 
		dbg("");
		//INIT_WORK(&ep->work, ep2_work);
		//usb_l2_init(L2_USB_EP2, ep->is_in);
		udc_writeb(USB_EP_INDEX, 2);
		udc_writeb(USB_CTRL_1, (1<<3) | (1<<6));
		udc_writeb(USB_CTRL_1_2, 1<<5);
		udc_writew(USB_TX_MAX, 512);
	} else if (!strcmp(_ep->name, udc->ep[3].ep.name)){ /* ep3 -- rx */ 
		dbg("");
		//INIT_WORK(&ep->work, ep3_work);
		//usb_l2_init(L2_USB_EP3, ep->is_in);
		udc_writeb(USB_EP_INDEX, 3);
		udc_writeb(USB_CTRL_1, 1<<6);
		udc_writeb(USB_CTRL_1_2, 0);
		udc_writew(USB_RX_MAX, 512);
		udc_writeb(USB_CTRL_2, udc_readb(USB_CTRL_2) & ~(0x1));
		udc_writeb(USB_CTRL_2, (1<<4) | (1<<7));
		udc_writeb(USB_CTRL_2_2, 0);
	} else if (!strcmp(_ep->name, udc->ep[4].ep.name)) { /* ep4 -- tx */ 
		dbg("");
		//INIT_WORK(&ep->work, ep4_work);
		//usb_l2_init(L2_USB_EP2, ep->is_in);
		udc_writeb(USB_EP_INDEX, 4);
		udc_writeb(USB_CTRL_1, (1<<3) | (1<<6));
		udc_writeb(USB_CTRL_1_2, 1<<5);
		udc_writew(USB_TX_MAX, 512);
	} else if (!strcmp(_ep->name, udc->ep[5].ep.name)){ /* ep5 -- rx */ 
		dbg("");
		//INIT_WORK(&ep->work, ep5_work);
		//usb_l2_init(L2_USB_EP3, ep->is_in);
		udc_writeb(USB_EP_INDEX, 5);
		udc_writeb(USB_CTRL_1, 1<<6);
		udc_writeb(USB_CTRL_1_2, 0);
		udc_writew(USB_RX_MAX, 512);
		udc_writeb(USB_CTRL_2, udc_readb(USB_CTRL_2) & ~(0x1));
		udc_writeb(USB_CTRL_2, (1<<4) | (1<<7));
		udc_writeb(USB_CTRL_2_2, 0);
	} else {
		printk("Invalid ep");
		return -EINVAL;
	}

	ep_irq_enable(_ep);
	local_irq_restore (flags);

	return 0;
}

static int ak980x_ep_disable (struct usb_ep * _ep)
{
	struct ak980x_ep *ep = container_of(_ep, struct ak980x_ep, ep);
	struct ak980x_request *req;
	unsigned long flags;

	if (!_ep || !ep->desc) {
		dbg("%s not enabled\n",
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	
	local_irq_save(flags);
	dbg("%s", _ep->name);
	ep->desc = NULL;
	ep->stopped = 1;
	
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct ak980x_request, queue);
		done(ep, req, -ESHUTDOWN);
	}
	ep_irq_disable(_ep);
	local_irq_restore(flags);
	
	return 0;
}

static struct usb_request *
	ak980x_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct ak980x_request *req;

	dbg("%s", _ep->name);
	req = kzalloc(sizeof (struct ak980x_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void ak980x_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ak980x_request *req;

	dbg("%s", _ep->name);
	req = container_of(_req, struct ak980x_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

static int ak980x_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ak980x_request	*req;
	struct ak980x_ep	*ep;
	struct ak980x_udc	*udc;
	int			status = 0;
	unsigned long flags;

	req = container_of(_req, struct ak980x_request, req);
	ep = container_of(_ep, struct ak980x_ep, ep);
	
	if (!_ep || (!ep->desc && ep->ep.name != ep0name)) {
		dbg("invalid ep\n");
		return -EINVAL;
	}

	udc = ep->udc;
	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		/* dbg("invalid device\n"); */
		printk("invalid device\n");
		return -EINVAL;
	}

	local_irq_save(flags);

	if (!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue)) {
		/* dbg("%s invalid request", _ep->name); */
		printk("%s invalid request\n", _ep->name);
		local_irq_restore(flags);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	dbg("%s queue is_in(%d)", ep->ep.name, ep->is_in);
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->ep.name == ep0name) {
			udc_writeb(USB_EP_INDEX, 0);
			
			switch (udc->ep0state) {
			case EP0_IN_DATA_PHASE:
				if ((udc_readb(USB_CTRL_1) & (1 << 1)) == 0
						&& write_ep0_fifo(ep, req)) {
					udc->ep0state = EP0_IDLE;
					req = NULL;
				}
				break;

			case EP0_OUT_DATA_PHASE:
				if ((!_req->length) ||
					((udc_readb(USB_CTRL_1) & (1 << 0))
					&& read_ep0_fifo(ep, req))) {
					udc->ep0state = EP0_IDLE;
					req = NULL;
				}
				break;
				
			default:
				local_irq_restore(flags);
				return -EL2HLT;
			}
			if (req)
				list_add_tail(&req->queue, &ep->queue);
		} else {
			list_add_tail(&req->queue, &ep->queue);
			if (!strcmp(_ep->name, udc->ep[1].ep.name)) { /* ep1 */ 
				dbg("ep1");
				status = write_ep1_fifo(ep, req);
			} else if (!strcmp(_ep->name, udc->ep[2].ep.name)) { /* ep2 */ 			
				udc_writeb(USB_EP_INDEX, 2);
				if ((udc_readb(USB_CTRL_1) & 1) == 0)
					queue_work(ep_wqueue, &ep->work);
			} else if (!strcmp(_ep->name, udc->ep[3].ep.name)){ /* ep3 */ 
				udc_writeb(USB_EP_INDEX, 3);
				if (udc_readb(USB_CTRL_2) & 1)
					queue_work(ep_wqueue, &ep->work);
			} else if (!strcmp(_ep->name, udc->ep[4].ep.name)) { /* ep4 */ 
				udc_writeb(USB_EP_INDEX, 4);
				if ((udc_readb(USB_CTRL_1) & 1) == 0)
				queue_work(ep_wqueue, &ep->work);
			} else if (!strcmp(_ep->name, udc->ep[5].ep.name)){ /* ep5 */ 
				udc_writeb(USB_EP_INDEX, 5);
				if (udc_readb(USB_CTRL_2) & 1)
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
	local_irq_restore(flags);
	/* return (status < 0) ? status : 0; */
	return status;
}


static int ak980x_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ak980x_ep	*ep;
	struct ak980x_request	*req;
	unsigned long flags;
	struct ak980x_udc *udc = &controller;
	
	dbg("dequeue");

	if (!udc->driver)
		return -ESHUTDOWN;

	ep = container_of(_ep, struct ak980x_ep, ep);
	if (!_ep || !_req)
		return -EINVAL;
	
	local_irq_save(flags);
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		local_irq_restore(flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);
	local_irq_restore(flags);
	
	return 0;
}

static int ak980x_ep_set_halt(struct usb_ep *_ep, int value)
{	
	struct ak980x_ep *ep = container_of(_ep, struct ak980x_ep, ep);
	unsigned int csr = 0;
	unsigned long		flags;
	struct ak980x_udc *udc = &controller;

	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		dbg("inval 2");
		return -EINVAL;
	}

	local_irq_save (flags);

	if ((ep->ep.name == ep0name) && value) {
		udc_writeb(USB_EP_INDEX, 0);
		udc_writeb(USB_CTRL_1, 1<<5);
		udc_writeb(USB_CTRL_1, 1<<6 | 1<<3);
	} else if (ep->is_in) { /* ep2 or ep4*/ 
		if (!strcmp(ep->ep.name, udc->ep[2].ep.name))
			udc_writeb(USB_EP_INDEX, 2);
		else
			udc_writeb(USB_EP_INDEX, 4);

		csr = udc_readw(USB_CTRL_1);
		if (value)
			udc_writew(USB_CTRL_1, csr | 1<<4);
		else {
			csr &= ~(1<<4 | 1<<5);
			udc_writew(USB_CTRL_1, csr);
			csr |= 1<<6;
			udc_writew(USB_CTRL_1, csr);
		}
	} else { /* ep3 or ep5*/
		if (!strcmp(ep->ep.name, udc->ep[3].ep.name))
			udc_writeb(USB_EP_INDEX, 3);
		else
			udc_writeb(USB_EP_INDEX, 5);
		csr = udc_readw(USB_CTRL_2);
		if (value)
			udc_writew(USB_CTRL_2, csr | 1<<5);
		else {
			csr &= ~(1<<5 | 1<<6);
			udc_writew(USB_CTRL_2, csr);
			csr |= 1<<7;
			udc_writew(USB_CTRL_2, csr);
		}
	}

	ep->stopped= value ? 1 : 0;
	local_irq_restore (flags);

	return 0;
}

static const struct usb_ep_ops ak980x_ep_ops = {
	.enable		= ak980x_ep_enable,
	.disable	= ak980x_ep_disable,
	.alloc_request	= ak980x_ep_alloc_request,
	.free_request	= ak980x_ep_free_request,
	.queue		= ak980x_ep_queue,
	.dequeue	= ak980x_ep_dequeue,
	.set_halt	= ak980x_ep_set_halt,
	// there's only imprecise fifo status reporting
};

static void nop_release(struct device *dev)
{
			/* nothing to free */
}

static struct ak980x_udc controller = {
	.gadget = {
		.ops	= &ak980x_udc_ops,
		.ep0	= &controller.ep[0].ep,
		.name	= driver_name,
		.dev	= {
		       .init_name = "gadget",
			   .release = nop_release,
		}
	},
	.ep[0] = {
		.ep = {
			.name		= ep0name,//ep_name[0],
			.ops		= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP0_FIFO_SIZE,
	},
	.ep[1] = {
		.ep = {
			.name	= "ep1-int",//ep_name[1],
			.ops	= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP1_FIFO_SIZE,
	},
	.ep[2] = {
		.ep = {
			.name	= "ep2in-bulk",//ep_name[2],
			.ops	= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP2_FIFO_SIZE,
	},
	.ep[3] = {
		.ep = {
			/* could actually do bulk too */
			.name	= "ep3out-bulk",//ep_name[3],
			.ops	= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP3_FIFO_SIZE,
	},
	.ep[4] = {
		.ep = {
			.name	= "ep4in-bulk",//ep_name[4],
			.ops	= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP4_FIFO_SIZE,
	},
	.ep[5] = {
		.ep = {
			.name	= "ep5out-bulk",//ep_name[5],
			.ops	= &ak980x_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= EP5_FIFO_SIZE,
	},
};

/**
 * ak98_udc_process_req_status - process request GET_STATUS
 * @udc: The device state
 * @ctrl: USB control request
 */
static int ak980x_udc_get_status(struct ak980x_udc *udc,
					struct usb_ctrlrequest *ctrl)
{
	u16 status = 0;
	u8 ep_num = ctrl->wIndex & 0x7F;
	u8 is_in = ctrl->wIndex & USB_DIR_IN;

	switch (ctrl->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		status = udc->devstatus;
		break;

	case USB_RECIP_INTERFACE:
		/* currently, the data result should be zero */
		break;

	case USB_RECIP_ENDPOINT:
		if (ep_num > 5 || ctrl->wLength > 2)
			return 1;

		if (ep_num == 0) {
			udc_writeb(USB_EP_INDEX, 0);
			status = udc_readb(USB_CTRL_1);
			status = status & (1<<5);
		} else {
			udc_writeb(USB_EP_INDEX, ep_num);
			if (is_in) {
				status = udc_readb(USB_CTRL_1);
				status = status & (1<<4);
			} else {
				status = udc_readb(USB_CTRL_2);
				status = status & (1<<5);
			}
		}

		status = status ? 1 : 0;
		break;
	default:
		return 1;
	}

	udc_writeb(USB_EP0_FIFO, status & 0xff);
	udc_writeb(USB_EP0_FIFO, status >> 8);

	udc_writel(USB_EP0_NUM, 2); /* 7bits */ 
	udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<1);

	udc->ep0state = EP0_END_XFER;

	return 0;
}

static void ak980x_udc_handle_ep0_idle(struct ak980x_udc *udc,
					struct ak980x_ep *ep, u32 ep0csr)
{
	struct usb_ctrlrequest crq;
	int i, len, ret, tmp, timeout;
	unsigned char *buf;

	/* start control request? */
	if (!(ep0csr & 1))
		return;

	//s3c2410_udc_nuke(dev, ep, -EPROTO);

	len = udc_readw(USB_EP_COUNT);
	buf = (unsigned char *)&crq;
	if (len > sizeof(struct usb_ctrlrequest))
		len = sizeof(struct usb_ctrlrequest);
	for (i = 0; i < len; i++)
		buf[i] = udc_readb(USB_EP0_FIFO);
	if (len != sizeof(crq)) {
		dbg("setup begin: fifo READ ERROR"
			" wanted %d bytes got %d. Stalling out...",
			sizeof(crq), len);
		udc_writeb(USB_CTRL_1, 1<<5);
		return;
	}

	dbg("bRequest = %d bRequestType %d wLength = %d",
		crq.bRequest, crq.bRequestType, crq.wLength);

	/* cope with automagic for some standard requests. */
	udc->req_std = (crq.bRequestType & USB_TYPE_MASK)
		== USB_TYPE_STANDARD;
	udc->req_config = 0;
	udc->req_pending = 1;

	switch (crq.bRequest) {
	case USB_REQ_SET_CONFIGURATION:
		dbg("USB_REQ_SET_CONFIGURATION ... ");

		if (crq.bRequestType == USB_RECIP_DEVICE) {
			udc->req_config = 1;
			udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
		}
		break;

	case USB_REQ_SET_INTERFACE:
		dbg("USB_REQ_SET_INTERFACE ... ");

		if (crq.bRequestType == USB_RECIP_INTERFACE) {
			udc->req_config = 1;
			udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
		}
		break;

	case USB_REQ_SET_ADDRESS:
		dbg("USB_REQ_SET_ADDRESS ... ");
		if (crq.bRequestType == USB_RECIP_DEVICE) {
			tmp = crq.wValue & 0x7F;
			udc_writeb(USB_CTRL_1, 0x1<<3 | 0x1<<6);
			timeout = 20000;
			/* waiting for next interrupt */ 
			while (!(udc_readb(USB_INTERRUPT_1) & 0x1) && timeout) {timeout--;}
			udc_writeb(USB_FUNCTION_ADDR, tmp);
			udc->addr = tmp;
			return;
		}
		break;

	case USB_REQ_GET_STATUS:
		dbg("USB_REQ_GET_STATUS ...");
		udc_writeb(USB_CTRL_1, 0x1<<6);

		if (udc->req_std) {
			if (ak980x_udc_get_status(udc, &crq)) {
				return;
			}
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		udc_writeb(USB_CTRL_1, 0x1<<6);

		if (crq.bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq.wValue != USB_ENDPOINT_HALT || crq.wLength != 0)
			break;

		ak980x_ep_set_halt(&udc->ep[crq.wIndex & 0x7f].ep, 0);
		udc_writeb(USB_CTRL_1, 0x1<<6 | 0x1<<3);
		
		return;

	case USB_REQ_SET_FEATURE:
		udc_writeb(USB_CTRL_1, 0x1<<6);

		if (crq.bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq.wValue != USB_ENDPOINT_HALT || crq.wLength != 0)
			break;

		ak980x_ep_set_halt(&udc->ep[crq.wIndex & 0x7f].ep, 1);
		udc_writeb(USB_CTRL_1, 0x1<<6 | 0x1<<3);
		return;

	default:
		udc_writeb(USB_CTRL_1, 0x1<<6);
		break;
	}

	if (crq.bRequestType & USB_DIR_IN)
		udc->ep0state = EP0_IN_DATA_PHASE;
	else
		udc->ep0state = EP0_OUT_DATA_PHASE;
	
	ret = udc->driver->setup(&udc->gadget, &crq);
	if (ret < 0) {
		if (udc->req_config) {
			dbg("config change %02x fail %d?",
				crq.bRequest, ret);
			return;
		}

		if (ret == -EOPNOTSUPP)
			dbg("Operation not supported");
		else
			dbg("udc->driver->setup failed. (%d)", ret);
		
		udc_writeb(USB_CTRL_1, 1<<5);
		udc_writeb(USB_CTRL_1, 1<<3 | 1<<6);
		udc->ep0state = EP0_IDLE;
		/* deferred i/o == no response yet */
	} else if (udc->req_pending) {
		dbg("dev->req_pending... what now?");
		udc->req_pending=0;
	}

	dbg("ep0state %s", ep0states[udc->ep0state]);
}

static void handle_ep0(struct ak980x_udc *udc)
{
	int csr, error = 0;
	struct ak980x_ep *ep0 = &udc->ep[0];
	struct ak980x_request *req = NULL;

	if (!list_empty(&ep0->queue)) 
		req = list_entry(ep0->queue.next, struct ak980x_request, queue);
	
	udc_writeb(USB_EP_INDEX, 0);
	csr = udc_readb(USB_CTRL_1);
	dbg("csr(0x%x)", csr);

	if (csr & 0x1<<3) {
		dbg("data end");
	} 
	if (csr & 0x1<<4) {
		dbg("A control transaction ends before the DataEnd bit has been set");
		udc_writeb(USB_CTRL_1, 0x1<<7);
		/* do something else? */
		udc->ep0state = EP0_IDLE;
		error = 1;
	}
	if (csr & 0x1<<2) {
		udc_writeb(USB_CTRL_1, udc_readb(USB_CTRL_1) & ~(1 << 2));
		udc->ep0state = EP0_IDLE;
		error = 1;
	}
	switch (udc->ep0state) {
	case EP0_IDLE:
		ak980x_udc_handle_ep0_idle(udc, ep0, csr);
		break;

	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		dbg("EP0_IN_DATA_PHASE ... what now?");
		if (!(csr & (1<<1)) && req && error == 0)
			write_ep0_fifo(ep0, req);
		break;

	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		dbg("EP0_OUT_DATA_PHASE ... what now?");
		if ((csr & (1<<0)) && req)
			read_ep0_fifo(ep0, req);
		break;

	case EP0_END_XFER:
		dbg("EP0_END_XFER ... what now?");
		udc->ep0state = EP0_IDLE;
		break;

	case EP0_STALL:
		dbg("EP0_STALL ... what now?");
		udc->ep0state = EP0_IDLE;
		break;
	}
}

/* not ep0 */
static void handle_ep(struct ak980x_ep *ep)
{
	struct ak980x_request *req;
	struct ak980x_udc *udc = ep->udc;
	unsigned int csr;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next,
			struct ak980x_request, queue);
	else {
		dbg("%s: no req waiting", ep->ep.name);
		req = NULL;
	}
	
	if (!strcmp(ep->ep.name, udc->ep[1].ep.name)) { /* ep1 */ 
		dbg("ep1");
	} else if (ep->is_in) { /* ep2 or ep4*/ 

		if (!strcmp(ep->ep.name, udc->ep[2].ep.name))
			udc_writeb(USB_EP_INDEX, 2);
		else
			udc_writeb(USB_EP_INDEX, 4);

		csr = udc_readb(USB_CTRL_1);
		dbg("ep2 or ep4 csr(0x%x), req(0x%p)", csr, req);

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
			return;
		}

		if (req) {
			if (ep->done) {
				done(ep, req, 0);
				if (!list_empty(&ep->queue)) {
					dbg("do next queue");
					req = list_entry(ep->queue.next, struct ak980x_request, queue);
					if ((csr & 1) == 0)
						queue_work(ep_wqueue, &ep->work);
				}
			} else {
				if ((csr & 1) == 0)
					queue_work(ep_wqueue, &ep->work);
			}

		}
	} else { /* ep3 or ep 5*/ 
		if (!strcmp(ep->ep.name, udc->ep[3].ep.name))
			udc_writeb(USB_EP_INDEX, 3);
		else
			udc_writeb(USB_EP_INDEX, 5);

		csr = udc_readb(USB_CTRL_2);
		dbg("ep3 or ep5 csr(0x%x)", csr);

		if (csr & 0x1<<6) {
			udc_writeb(USB_CTRL_2, csr & ~(0x1<<6));
		}
		if (req && (csr & 0x1<<0))
			queue_work(ep_wqueue, &ep->work);
	}
}

static void udc_disconnect(struct ak980x_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak980x_ep *ep = &udc->ep[i];
		struct ak980x_request *req;

		ep->stopped = 1;

		// terminer chaque requete dans la queue
		if (list_empty(&ep->queue))
			continue;

		while (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct ak980x_request, queue);
			done(ep, req, -ESHUTDOWN);
		}
	}

	if (driver)
		driver->disconnect(&udc->gadget);

	udc_reinit(udc);
}

#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
static void ak98_udc_fun(void *data)
{
	struct ak980x_udc *udc = data;
	
	msleep(500);

	clk_enable(udc->clk);
	rMULFUN_CON1 &= ~0x7;
	rMULFUN_CON1 |= 0x6;
}
#endif
	
static void udc_reset(struct ak980x_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i ,temp;

	//printk("%x\n", udc_readb(USB_INTERRUPT_USB)); /* ep0 */ 
#ifdef USB_11
	udc_writeb(USB_POWER_CTRL, 0);
	udc_writel(USB_MODE_STATUS, udc_readl(USB_MODE_STATUS) | 0x1);
#else
	udc_writel(USB_MODE_STATUS, udc_readl(USB_MODE_STATUS) & (~0x1));
	udc_writeb(USB_POWER_CTRL, 0x1<<5);
#endif

	udc_writeb(USB_FUNCTION_ADDR, 0);
	udc_writeb(USB_INTERRUPT_USB, ~(0x1<<3));
	udc_writeb(USB_INTERRUPT_TX, 0x1<<0); /* ep0 */ 
	
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	temp = udc_readw(USB_INTERRUPT_COMM);
	temp = udc_readw(USB_INTERRUPT_1);
	temp = udc_readw(USB_INTERRUPT_2);

	udc->ep0state = EP0_IDLE;

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak980x_ep *ep = &udc->ep[i];
		struct ak980x_request *req;

		if (list_empty(&ep->queue))
			continue;

		while (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next, struct ak980x_request, queue);
			done(ep, req, -ECONNRESET);
		}
	}
}

static irqreturn_t udc_irqhandler(int irq, void *_udc)
{
	struct ak980x_udc *udc = _udc;
	short status_1, status_2;
	char status_int;

#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
	if (start == 0) {
		start = 1;
		rMULFUN_CON1 &= ~0x7;
		clk_disable(udc->clk);
		reset_usbcontroller();
		printk("ak98_request_hold_mode NORMAL_MODE_CLOCK_5\n");
		ak98_request_hold_mode(NORMAL_MODE_CLOCK_5, ak98_udc_fun, udc);
		return IRQ_HANDLED;
	}
#endif

	status_int = udc_readb(USB_INTERRUPT_COMM);
	if (status_int & 0x1<<2) {
		/* dbg("status_int(0x%x), reset", status_int);	 */
		printk("\n\nstatus_int(0x%x), reset\n\n", status_int);
		if (usb_detect) {
			usb_exist = 1;
			usb_detect = 0;
			if (!udc->driver) {
				panic("If you see this, come to find Zhang Jingyuan\n");
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
#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
		if (start == 1) {
			start = 0;
			printk("ak98_release_hold_mode NORMAL_MODE_CLOCK_5\n");
			ak98_release_hold_mode(NORMAL_MODE_CLOCK_5);
		}
#endif
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
	if (status_2 & 0x1<<5) {
		dbg("endpoint5");
		handle_ep(&udc->ep[5]);
	}

done:
	return IRQ_HANDLED;
}

static irqreturn_t udc_dmahandler(int irq, void *_udc)
{
	struct ak980x_request *req = NULL;
	struct ak980x_udc *udc = _udc;
	struct ak980x_ep *ep;
	unsigned int is_done = 0;

	u32 usb_dma_int = udc_readl(USB_DMA_INTR);
	
	if ((usb_dma_int & DMA_CHANNEL1_INT) == DMA_CHANNEL1_INT) {
		ep = &udc->ep[2];
		udc_writeb(USB_EP_INDEX, USB_EP2_INDEX);
		udc_writeb(USB_CTRL_1_2, USB_TXCSR_MODE1);
        udc_writel(USB_DMA_CTRL1, 0);

        ak98_l2_combuf_wait_dma_finish(ep->l2_buf_id);

		if (!list_empty(&ep->queue))
			req = list_entry(ep->queue.next, struct ak980x_request, queue);

		if (req) {
			if (dma_tx == req->req.length - req->req.actual && !req->req.zero) 
				is_done = 1;
			req->req.actual += dma_tx;
			ep->done = is_done;
			if (is_done) {
				done(ep, req, 0);
				if (!list_empty(&ep->queue)) {
					dbg("do next queue");
					queue_work(ep_wqueue, &ep->work);
				}
			} else {
				queue_work(ep_wqueue, &ep->work);
			}
			dma_unmap_single(NULL, phys_tx, dma_tx, DMA_TO_DEVICE);
			dma_tx = 0;

			req->status = is_done;
		}
		else
			dbg("something happend");
	}
	
	if ((usb_dma_int & DMA_CHANNEL2_INT) == DMA_CHANNEL2_INT) {
		ep = &udc->ep[3];

		udc_writeb(USB_EP_INDEX, USB_EP3_INDEX);
		udc_writeb(USB_CTRL_2_2, 0);
		udc_writel(USB_DMA_CTRL2, 0);
		
		ak98_l2_combuf_wait_dma_finish(ep->l2_buf_id);

		req = NULL;
		is_done = 0;
		if (!list_empty(&ep->queue))
			req = list_entry(ep->queue.next, struct ak980x_request, queue);

		if (req) {
			if (dma_rx == req->req.length - req->req.actual)
				is_done = 1;
			req->req.actual += dma_rx;
			ep->done = is_done;
			if (is_done)
				done(ep, req, 0);
			dma_unmap_single(NULL, phys_rx, dma_rx, DMA_FROM_DEVICE);
			req->status = is_done;
			dma_rx = 0;

			flag = 0;
		}
		else
			dbg("something happend");
	}

	if ((usb_dma_int & DMA_CHANNEL3_INT) == DMA_CHANNEL3_INT) {
		ep = &udc->ep[4];
		udc_writeb(USB_EP_INDEX, USB_EP4_INDEX);
		udc_writeb(USB_CTRL_1_2, USB_TXCSR_MODE1);
        udc_writel(USB_DMA_CTRL1, 0);

        ak98_l2_combuf_wait_dma_finish(ep->l2_buf_id);

		if (!list_empty(&ep->queue))
			req = list_entry(ep->queue.next, struct ak980x_request, queue);

		if (req) {
			if (dma_tx == req->req.length - req->req.actual && !req->req.zero) 
				is_done = 1;
			req->req.actual += dma_tx;
			ep->done = is_done;
			if (is_done) {
				done(ep, req, 0);
				if (!list_empty(&ep->queue)) {
					dbg("do next queue");
					queue_work(ep_wqueue, &ep->work);
				}
			} else {
				queue_work(ep_wqueue, &ep->work);
			}
			dma_unmap_single(NULL, phys_tx, dma_tx, DMA_TO_DEVICE);
			dma_tx = 0;

			req->status = is_done;
		}
		else
			dbg("something happend");
	}
	
	if ((usb_dma_int & DMA_CHANNEL4_INT) == DMA_CHANNEL4_INT) {
		ep = &udc->ep[5];

		udc_writeb(USB_EP_INDEX, USB_EP5_INDEX);
		udc_writeb(USB_CTRL_2_2, 0);
		udc_writel(USB_DMA_CTRL2, 0);
		
		ak98_l2_combuf_wait_dma_finish(ep->l2_buf_id);

		req = NULL;
		is_done = 0;
		if (!list_empty(&ep->queue))
			req = list_entry(ep->queue.next, struct ak980x_request, queue);

		if (req) {
			if (dma_rx == req->req.length - req->req.actual)
				is_done = 1;
			req->req.actual += dma_rx;
			ep->done = is_done;
			if (is_done)
				done(ep, req, 0);
			dma_unmap_single(NULL, phys_rx, dma_rx, DMA_FROM_DEVICE);
			req->status = is_done;
			dma_rx = 0;

			flag = 0;
		}
		else
			dbg("something happend");
	}

	return IRQ_HANDLED;
}

void udc_enable(struct ak980x_udc *udc, int enable)
{
	if (enable) {
		reset_usbcontroller();
		clk_enable(udc->clk);
		rMULFUN_CON1 &= ~0x7;
		rMULFUN_CON1 |= 6;
#ifdef USB_11
		udc_writeb(USB_POWER_CTRL, 0);
		udc_writel(USB_MODE_STATUS, udc_readl(USB_MODE_STATUS) | 0x1);
#else
		udc_writel(USB_MODE_STATUS, udc_readl(USB_MODE_STATUS) & (~0x1));
		udc_writeb(USB_POWER_CTRL, 0x1<<5);
#endif

	} else {
		rMULFUN_CON1 &= ~0x7;
		clk_disable(udc->clk);
		reset_usbcontroller();
	}
}

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct ak980x_udc	*udc = &controller;
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

	retval = driver->bind(&udc->gadget);
	if (retval) {
		dbg("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;

		return retval;
	}

	//local_irq_disable();
	disable_irq(udc->mcu_irq);
	udc_enable(udc, 1);
	//local_irq_enable();
	enable_irq(udc->mcu_irq);

	dbg("bound to %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct ak980x_udc *udc = &controller;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;
	
	driver->unbind(&udc->gadget);
	udc->gadget.dev.driver = NULL;
	//udc->gadget.dev.driver_data = NULL;
	udc->driver = NULL;
	
	//local_irq_disable();
	disable_irq(udc->mcu_irq);
	
	cancel_work_sync(&udc->ep[2].work);
	cancel_work_sync(&udc->ep[3].work);
	cancel_work_sync(&udc->ep[4].work);
	cancel_work_sync(&udc->ep[5].work);

	flush_workqueue(ep_wqueue);
	
	udc_enable(udc, 0);
	//local_irq_enable();
	enable_irq(udc->mcu_irq);

	dbg("unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

static void udc_reinit(struct ak980x_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	udc->ep0state = EP0_IDLE;

	for (i = 0; i < ENDPOINTS_NUM; i++) {
		struct ak980x_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		/* ep->stopped = 0; */
		ep->stopped = 0;
		ep->ep.maxpacket = ep->maxpacket;
		// initialiser une queue par endpoint
		INIT_LIST_HEAD(&ep->queue);
	}
}

static int __init ak980x_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ak980x_udc *udc = &controller;
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

	ep_wqueue = create_workqueue("ak980x_udc");
	/* request UDC and maybe VBUS irqs */
	udc->mcu_irq = platform_get_irq(pdev, 0);

	local_irq_disable();
	rIRQ_MASK &= ~(1UL << 25);
	rCLK_CON1 |= (1UL << 15);
	rMULFUN_CON1 &= ~0x7;
	udc_writeb(USB_POWER_CTRL, 0);
	reset_usbcontroller();
	local_irq_enable();
	retval = request_irq(udc->mcu_irq, udc_irqhandler,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dbg("request irq %d failed\n", udc->mcu_irq);
		goto fail1;
	}

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	/* request DMA irqs */
	udc->dma_irq = platform_get_irq(pdev, 1);
	retval = request_irq(udc->dma_irq, udc_dmahandler,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dbg("request irq %d failed\n", udc->dma_irq);
		goto fail1;
	}
#endif
#endif
	platform_set_drvdata(pdev, udc);

	dbg("Build at %s %s", __DATE__, __TIME__);

	/* for g_serial.ko */
	//udc->ep[2].bufaddr = dma_alloc_coherent(NULL, 16384, &udc->ep[2].bufphys, DMA_TO_DEVICE);
	//udc->ep[3].bufaddr = dma_alloc_coherent(NULL, 16384, &udc->ep[3].bufphys, DMA_FROM_DEVICE);

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	/* USB slave L2 buffer initialization */
	udc->ep[2].l2_buf_id = ak98_l2_alloc(ADDR_USB_EP2);
	udc->ep[3].l2_buf_id = ak98_l2_alloc(ADDR_USB_EP3);
	udc->ep[4].l2_buf_id = ak98_l2_alloc(ADDR_USB_EP4);
	udc->ep[5].l2_buf_id = ak98_l2_alloc(ADDR_USB_EP5);
#endif
#endif

	INIT_WORK(&udc->ep[2].work, ep2_work);
	INIT_WORK(&udc->ep[3].work, ep3_work);
	INIT_WORK(&udc->ep[4].work, ep4_work);
	INIT_WORK(&udc->ep[5].work, ep5_work);

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

static int __exit ak980x_udc_remove(struct platform_device *pdev)
{
	struct ak980x_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;

	if (udc->driver)
		return -EBUSY;

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	free_irq(udc->dma_irq, udc);
#endif
#endif
	free_irq(udc->mcu_irq, udc);
	device_unregister(&udc->gadget.dev);

	destroy_workqueue(ep_wqueue);

	iounmap(udc->baseaddr);

#ifndef USB_11
#ifdef CONFIG_USB_GADGET_AK98_PRODUCER
	/* USB slave L2 buffer initialization */
	ak98_l2_free(ADDR_USB_EP2);
	ak98_l2_free(ADDR_USB_EP3);
	ak98_l2_free(ADDR_USB_EP4);
	ak98_l2_free(ADDR_USB_EP5);
#endif
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

static void ak980x_udc_shutdown(struct platform_device *dev)
{
}

#ifdef CONFIG_PM
static int ak980x_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct ak980x_udc *udc = platform_get_drvdata(pdev);

	cancel_work_sync(&udc->ep[2].work);
	cancel_work_sync(&udc->ep[3].work);
	cancel_work_sync(&udc->ep[4].work);
	cancel_work_sync(&udc->ep[5].work);

	flush_workqueue(ep_wqueue);

	rMULFUN_CON1 &= ~0x7;
		
	clk_disable(udc->clk);

#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
		if (start == 1) {
			start = 0;
			printk("ak98_release_hold_mode2 NORMAL_MODE_CLOCK_5\n");
			ak98_release_hold_mode(NORMAL_MODE_CLOCK_5);
		}
#endif
	
	reset_usbcontroller();
	
	udc_disconnect(udc);
	
	return 0;
}

static int ak980x_udc_resume(struct platform_device *pdev)
{
	struct ak980x_udc *udc = platform_get_drvdata(pdev);

	/* something to do */
	clk_enable(udc->clk);

	rMULFUN_CON1 &= ~0x7;
	rMULFUN_CON1 |= 0x6;
	
	return 0;
}
#else
#define	ak980x_udc_suspend	NULL
#define	ak980x_udc_resume	NULL
#endif

static struct platform_driver ak980x_udc_driver = {
	.remove		= __exit_p(ak980x_udc_remove),
	.shutdown	= ak980x_udc_shutdown,
	.suspend	= ak980x_udc_suspend,
	.resume		= ak980x_udc_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __init udc_init_module(void)
{
	printk("AK980X UDC Driver, (c) 2010 ANYKA\n");

	return platform_driver_probe(&ak980x_udc_driver, ak980x_udc_probe);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	platform_driver_unregister(&ak980x_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("AK980X udc driver");
MODULE_AUTHOR("Anyka");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak980x_udc");
