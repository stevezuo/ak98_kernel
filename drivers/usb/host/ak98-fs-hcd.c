/*
 * AK98xx FS HCD (Full-Speed Host Controller Driver) for USB.
 *
 * Derived from the SL811 HCD, rewritten for AK98FS HCD.
 * Copyright (C) 2010 ANYKA LTD.
 *
 * Periodic scheduling is based on Roman's OHCI code
 * 	Copyright (C) 1999 Roman Weissgaerber
 *
 * The AK98FS Host controller handles host side USB. For Documentation,
 * refer to chapter 22 USB Controllers of AK98xx Mobile Multimedia Application
 * Processor Programmer's Guide.
 *
 */

/*
 * Status:  Enumeration of USB Optical Mouse, USB Keyboard, USB Flash Disk, Ralink 2070/3070 USB WiFi OK.
 *          Pass basic test with USB Optical Mouse/USB Keyboard/USB Flash Disk.
 *          Ralink 2070/3070 USB WiFI Scanning/WEP basic test OK. Full Functions TBD.
 *
 * TODO:
 * - Use up to 6 active queues of FS HC(for now only 2 queues: EP0 & EPX(1-6))
 * - USB Suspend/Resume support
 * - Use urb->iso_frame_desc[] with ISO transfers
 * - Optimize USB FIFO data transfer/receive(4B->2B->1B)
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/ak98fsh.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

#include "../core/hcd.h"
#include "ak98-fs-hcd.h"

#include <mach/ak880x_addr.h>
#include <mach/regs-comm.h>
#include <mach/clock.h>
#include <mach/gpio.h>

#ifdef AK98FS_DEBUG
const char *trans_desc[4] = { "ISO", "INT", "CNTL", "BULK"};
const char *xfer_name[4] = { "CNTL", "ISO", "BULK", "INT" };
#endif

MODULE_DESCRIPTION("AK98HS USB Full Speed Host Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak98-fs-hcd");

#define DRIVER_VERSION	"16 Dec 2010"

static const char hcd_name[] = "ak98-fs-hcd";

static u8 ep_fifos[] = { FIFO_EP0, FIFO_EP1, FIFO_EP2, FIFO_EP3, FIFO_EP4, FIFO_EP5, FIFO_EP6 };
struct ak98fsh_epfifo_mapping ak98fsh_epfifo_mapping;
static int period_epfifo;

static void usbhost_reset(void);


static void port_power(struct ak98fsh *ak98fsh, int is_on)
{
	/*ak98 usb host is self power currently.*/
	struct usb_hcd	*hcd = ak98fsh_to_hcd(ak98fsh);

	/* hub is inactive unless the port is powered */
	if (is_on) {
		if (ak98fsh->port_status & (1 << USB_PORT_FEAT_POWER))
			return;

		ak98fsh->port_status = (1 << USB_PORT_FEAT_POWER);
	} else {
		ak98fsh->port_status = 0;
		hcd->state = HC_STATE_HALT;
	}

	if (ak98fsh->board && ak98fsh->board->port_power) {
		/* switch VBUS, at 500mA unless hub power budget gets set */
		HDBG("power %s\n", is_on ? "on" : "off");
		ak98fsh->board->port_power(hcd->self.controller, is_on);
	}

	/* reset as thoroughly as we can */
	if (ak98fsh->board && ak98fsh->board->reset)
		ak98fsh->board->reset(hcd->self.controller);
	else {
		/*softreset the usb host controller.*/
		usbhost_reset();
	}

	// if !is_on, put into lowpower mode now

}

/*-------------------------------------------------------------------------*/

/* This is a PIO-only HCD.  Queueing appends URBs to the endpoint's queue,
 * and may start I/O.  Endpoint queues are scanned during completion irq
 * handlers (one per packet: ACK, NAK, faults, etc) and urb cancellation.
 *
 * Using an external DMA engine to copy a packet at a time could work,
 * though setup/teardown costs may be too big to make it worthwhile.
 */

/* SETUP starts a new control request.  Devices are not allowed to
 * STALL or NAK these; they must cancel any pending control requests.
 */
static void setup_packet(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb
)
{
	int	i;
	unsigned int fifo_val;
	u8	len;
	unsigned char *buf = urb->setup_packet;

	HDBG("Packet: Setup Packet (EP %d)\n", ep->epnum);

	len = sizeof(struct usb_ctrlrequest);

	fsh_index_writeb(0, 0, CTLSTS_EP_TXL);
	fsh_writeb(usb_pipedevice(urb->pipe), UR_FUNADDR);
	for (i = 0; i < len; i += 4) {
		fifo_val = (*(buf+i) | ( *(buf+i+1)<<8 )
				   | ( *(buf+i+2)<<16 ) | ( *(buf+i+3)<<24 ));
		fsh_writel(fifo_val, FIFO_EP0);
	}
	fsh_index_writeb(0, 0x08 | 0x02, CTLSTS_EP_TXL);

	ep->length = 0;
}

/* STATUS finishes control requests, often after IN or OUT data packets */
static void status_packet(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb
)
{
	int			do_out;
	int			epfifo;

	HDBG("Packet: Status Packet (EP %d)\n", ep->epnum);

	do_out = urb->transfer_buffer_length && usb_pipein(urb->pipe);
	if (!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, !usb_pipein(urb->pipe), &epfifo))
		BUG();	/* Impossible, USB Device Endpoint *MUST* have been mapped to EPFIFO */
	
	fsh_writeb(usb_pipedevice(urb->pipe), UR_FUNADDR);

	if (do_out) {
		fsh_index_writeb(epfifo, 0x42, CTLSTS_EP_TXL);
	} else {
		fsh_index_writeb(epfifo, 0x60, CTLSTS_EP_TXL);
	}
		
	ep->length = 0;
}

/* IN packets can be used with any type of endpoint. here we just
 * start the transfer, data from the peripheral may arrive later.
 * urb->iso_frame_desc is currently ignored here...
 */
static void in_packet(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb
)
{
	int			epfifo;
	u8			len;
	unsigned int		pipe = urb->pipe;

	HDBG("Packet: IN Packet (EP %d), Length=%d,ep->maxpacket=%d\n",
		ep->epnum, urb->transfer_buffer_length - urb->actual_length, ep->maxpacket);

	/* avoid losing data on overflow */
	len = ep->maxpacket;

	fsh_writeb(usb_pipedevice(urb->pipe), UR_FUNADDR);
	if (ep->epnum == 0) {
		fsh_index_writeb(0, 1 << 5, CTLSTS_EP_TXL);
	} else {
		int eptype = 0;
		if (!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, 0, &epfifo))
			BUG();	/* Impossible, USB Device Endpoint *MUST* have been mapped to EPFIFO */
		if (usb_pipeisoc(pipe)) {
			eptype = 1;
		} else if (usb_pipeint(pipe)) {
			eptype = 3;
		} else if (usb_pipebulk(pipe)) {
			eptype = 2;
		} else BUG();
#ifndef DYNAMIC_EPFIFO
		set_epx_rx_type(epfifo, ep->epnum, eptype);
		set_epx_rx_mode(epfifo);
#endif
		fsh_index_writeb(epfifo, 1 << 5, CTLSTS_EP_RXL);
	}

	ep->length = min_t(u32, len,
			urb->transfer_buffer_length - urb->actual_length);
}

/* OUT packets can be used with any type of endpoint.
 * urb->iso_frame_desc is currently ignored here...
 */
static void out_packet(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb
)
{
	int i;
	int			epfifo;
	unsigned char		*buf;
	u8			len;
	unsigned int		pipe = urb->pipe;

	HDBG("Packet: OUT Packet (EP %d), Length=%d\n", ep->epnum, urb->transfer_buffer_length - urb->actual_length);

	buf = (unsigned char *)(urb->transfer_buffer + urb->actual_length);
	prefetch(buf);

	len = min_t(u32, ep->maxpacket,
			urb->transfer_buffer_length - urb->actual_length);

	if (!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, 1, &epfifo))
		BUG();	/* Impossible, USB Device Endpoint *MUST* have been mapped to EPFIFO */
	fsh_index_writeb(epfifo, 0, CTLSTS_EP_TXL);
	fsh_writeb(usb_pipedevice(pipe), UR_FUNADDR);
	for (i = 0; i < len; i ++) {
		fsh_writeb(buf[i], ep_fifos[epfifo]);
	}
	if (ep->epnum == 0) {
		fsh_index_writeb(0, 0x02, CTLSTS_EP_TXL);
	} else {
#ifndef DYNAMIC_EPFIFO
		int eptype = 0;

		if (usb_pipeisoc(pipe)) {
			eptype = 1;
		} else if (usb_pipeint(pipe)) {
			eptype = 3;
		} else if (usb_pipebulk(pipe)) {
			eptype = 2;
		} else BUG();
		set_epx_tx_type(epfifo, ep->epnum, eptype);
		set_epx_tx_mode(epfifo);
#endif
		fsh_index_writeb(epfifo, 1 << 0, CTLSTS_EP_TXL);
	}

	ep->length = len;
}

/*-------------------------------------------------------------------------*/

/* caller updates on-chip enables later */
static inline void sofirq_on(struct ak98fsh *ak98fsh)
{
    unsigned int regval;

    regval = fsh_readb(UR_INTECOM);
    regval |= 1<<3;
    fsh_writeb(regval, UR_INTECOM);
}

static inline void sofirq_off(struct ak98fsh *ak98fsh)
{
    unsigned int regval;

    regval = fsh_readb(UR_INTECOM);
    regval &= ~(1<<3);
    fsh_writeb(regval, UR_INTECOM);
}

/*-------------------------------------------------------------------------*/

static struct ak98fsh_ep *start_ep0(struct ak98fsh *ak98fsh)
{
	struct ak98fsh_ep	*ep;
	struct urb		*urb;
	
	/* use endpoint at schedule head */
	if (ak98fsh->next_async_ep0)
		ep = ak98fsh->next_async_ep0;
	else if (!list_empty(&ak98fsh->async_ep0)) {
		ep = container_of(ak98fsh->async_ep0.next,
				struct ak98fsh_ep, schedule);
	} else {
		/* could set up the first fullspeed periodic
		 * transfer for the next frame ...
		 */
		return NULL;
	}

	if (ep->schedule.next == &ak98fsh->async_ep0)
		ak98fsh->next_async_ep0 = NULL;
	else
		ak98fsh->next_async_ep0 = container_of(ep->schedule.next,
			struct ak98fsh_ep, schedule);

	if (unlikely(list_empty(&ep->hep->urb_list))) {
		HDBG("empty %p queue?\n", ep);
		return NULL;
	}

	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);

	switch (ep->nextpid) {
	case USB_PID_IN:
		in_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_OUT:
		out_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_SETUP:
		setup_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_ACK:		/* for control status */
		status_packet(ak98fsh, ep, urb);
		break;
	default:
		HDBG("bad ep%p pid %02x\n", ep, ep->nextpid);
		ep = NULL;
	}
	return ep;
}


/* pick the next endpoint for a transaction, and issue it.
 * frames start with periodic transfers (after whatever is pending
 * from the previous frame), and the rest of the time is async
 * transfers, scheduled round-robin.
 */
static struct ak98fsh_ep *start_epx(struct ak98fsh *ak98fsh, int epfifo)
{
	struct ak98fsh_ep	*ep;
	struct urb		*urb;

	
	/* use endpoint at schedule head */
	if (ak98fsh->next_periodic) {
		ep = ak98fsh->next_periodic;
		ak98fsh->next_periodic = ep->next;
	} else {
		if (ak98fsh->next_async_epx[epfifo-1])
			ep = ak98fsh->next_async_epx[epfifo-1];
		else if (!list_empty(&ak98fsh->async_epx[epfifo-1])) {
			ep = container_of(ak98fsh->async_epx[epfifo-1].next,
					struct ak98fsh_ep, schedule);

		} else {
			/* could set up the first fullspeed periodic
			 * transfer for the next frame ...
			 */
			return NULL;
		}

		if (ep->schedule.next == &ak98fsh->async_epx[epfifo-1])
			ak98fsh->next_async_epx[epfifo-1] = NULL;
		else {
			ak98fsh->next_async_epx[epfifo-1] = container_of(ep->schedule.next,
					struct ak98fsh_ep, schedule);
		}
	}

	if (unlikely(list_empty(&ep->hep->urb_list))) {
		HDBG("empty %p queue?\n", ep);
		return NULL;
	}
	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
	switch (ep->nextpid) {
	case USB_PID_IN:
		in_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_OUT:
		out_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_SETUP:
		setup_packet(ak98fsh, ep, urb);
		break;
	case USB_PID_ACK:		
		status_packet(ak98fsh, ep, urb);
		break;
	default:
		HDBG("bad ep%p pid %02x\n", ep, ep->nextpid);
		ep = NULL;
	}
	return ep;
}

#define MIN_JIFFIES	((msecs_to_jiffies(2) > 1) ? msecs_to_jiffies(2) : 2)

static inline void start_transfer_ep0(struct ak98fsh *ak98fsh)
{
	if (ak98fsh->port_status & (1 << USB_PORT_FEAT_SUSPEND))
		return;
	if (ak98fsh->active_ep0 == NULL) {
		ak98fsh->active_ep0 = start_ep0(ak98fsh);
		if (ak98fsh->active_ep0 != NULL)
			ak98fsh->jiffies_ep0 = jiffies + MIN_JIFFIES;
	}
}

static inline void start_transfer_epx(struct ak98fsh *ak98fsh, int epfifo)
{
	if (ak98fsh->port_status & (1 << USB_PORT_FEAT_SUSPEND))
		return;
	if (ak98fsh->active_epx[epfifo-1] == NULL) {
		ak98fsh->active_epx[epfifo-1] = start_epx(ak98fsh, epfifo);
		if (ak98fsh->active_epx[epfifo-1] != NULL)
			ak98fsh->jiffies_epx[epfifo-1] = jiffies + MIN_JIFFIES;
	}
}

static inline void start_transfer(struct ak98fsh *ak98fsh, int epfifo)
{
	if(epfifo == 0)
		start_transfer_ep0(ak98fsh);
	else
		start_transfer_epx(ak98fsh, epfifo);
}

static void finish_request_ep0(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb,
	int			status
) __releases(ak98fsh->lock) __acquires(ak98fsh->lock)
{
	VDBG("Finishing EP0 URB Request...\n");

	if (usb_pipecontrol(urb->pipe))
		ep->nextpid = USB_PID_SETUP;

	usb_hcd_unlink_urb_from_ep(ak98fsh_to_hcd(ak98fsh), urb);
	spin_unlock(&ak98fsh->lock);
	usb_hcd_giveback_urb(ak98fsh_to_hcd(ak98fsh), urb, status);
	spin_lock(&ak98fsh->lock);

	/* leave active endpoints in the schedule */
	if (!list_empty(&ep->hep->urb_list))
		return;

	/* async deschedule? */
	if (!list_empty(&ep->schedule)) {
		list_del_init(&ep->schedule);
		if (ep == ak98fsh->next_async_ep0)
			ak98fsh->next_async_ep0 = NULL;
		return;
	}
}


static void finish_request_epx(
	struct ak98fsh		*ak98fsh,
	struct ak98fsh_ep	*ep,
	struct urb		*urb,
	int			status
) __releases(ak98fsh->lock) __acquires(ak98fsh->lock)
{
	unsigned		i;
	int		epfifo;
	int is_out;
	unsigned int		pipe = urb->pipe;

	is_out = !usb_pipein(pipe);

	VDBG("Finishing EPx URB Request...\n");

	if (usb_pipecontrol(urb->pipe))
		ep->nextpid = USB_PID_SETUP;

	usb_hcd_unlink_urb_from_ep(ak98fsh_to_hcd(ak98fsh), urb);
	spin_unlock(&ak98fsh->lock);
	usb_hcd_giveback_urb(ak98fsh_to_hcd(ak98fsh), urb, status);
	spin_lock(&ak98fsh->lock);

	/* leave active endpoints in the schedule */
	if (!list_empty(&ep->hep->urb_list))
		return;

	/* async deschedule? */
	if(epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, is_out, &epfifo) && !list_empty(&ep->schedule)) {
		list_del_init(&ep->schedule);
		if (ep == ak98fsh->next_async_epx[epfifo-1]) {
			ak98fsh->next_async_epx[epfifo-1] = NULL;
			}
		return;
	}

	/* periodic deschedule */
	VDBG("deschedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
	for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
		struct ak98fsh_ep	*temp;
		struct ak98fsh_ep	**prev = &ak98fsh->periodic[i];

		while (*prev && ((temp = *prev) != ep))
			prev = &temp->next;
		if (*prev)
			*prev = ep->next;
		ak98fsh->load[i] -= ep->load;
	}
	ep->branch = PERIODIC_SIZE;
	ak98fsh->periodic_count--;
	ak98fsh_to_hcd(ak98fsh)->self.bandwidth_allocated
		-= ep->load / ep->period;
	if (ep == ak98fsh->next_periodic)
		ak98fsh->next_periodic = ep->next;

	/* we might turn SOFs back on again for the async schedule */
	if (ak98fsh->periodic_count == 0)
		sofirq_off(ak98fsh);
}

static void
done(struct ak98fsh *ak98fsh, struct ak98fsh_ep *ep)
{
	int 			i;
	int			err_occurred = 0;
	int			epfifo;
	u8			status;
	struct urb		*urb;
	unsigned int		pipe;
	int			is_out;
	int			urbstat = -EINPROGRESS;

	if (unlikely(!ep)) {
		return;
	}

	urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
	pipe = urb->pipe;
	is_out = !usb_pipein(pipe);
	if (!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, is_out, &epfifo))
		BUG();	/* Impossible, USB Device Endpoint *MUST* have been mapped to EPFIFO */

	status = fsh_index_readw(epfifo, CTLSTS_EP_TXL);

	if (status & 0x80) {
		if (!ep->period)
			ep->nak_count++;
		ep->error_count = 0;
		err_occurred = 1;
	}

	if (((epfifo == 0) && (status & 0x04)) || ((epfifo != 0) && (status & 0x10))) {
		ep->nak_count = ep->error_count = 0;
		urbstat = -EPIPE;
		err_occurred = 1;
	}

	if (((epfifo == 0) && (status & 0x10)) ||
		((epfifo != 0) && (status & 0x04) && (usb_pipeint(pipe) || usb_pipebulk(pipe)))) {
		urbstat = -EPROTO;
		ep->error_count = 0;
		err_occurred = 1;
	}

	if (err_occurred) {
		if (epfifo == 0)
			fsh_index_writew(0, 0, CTLSTS_EP_TXL);
		else if (is_out)
			fsh_index_writew(epfifo, 0, CTLSTS_EP_TXL);
		else
			fsh_index_writew(epfifo, 0, CTLSTS_EP_RXL);
	} else {
		struct usb_device	*udev = urb->dev;
		int			len;
		unsigned char		*buf;

		/* urb->iso_frame_desc is currently ignored here... */

		ep->nak_count = ep->error_count = 0;
		switch (ep->nextpid) {
		case USB_PID_OUT:
			urb->actual_length += ep->length;
			usb_dotoggle(udev, ep->epnum, 1);
			if (urb->actual_length
					== urb->transfer_buffer_length) {
				if (usb_pipecontrol(urb->pipe)) {
					VDBG("NEXT Packet: Status Packet.\n");
					ep->nextpid = USB_PID_ACK;
				}

				/* some bulk protocols terminate OUT transfers
				 * by a short packet, using ZLPs not padding.
				 */
				else if (ep->length < ep->maxpacket
						|| !(urb->transfer_flags
							& URB_ZERO_PACKET))
					urbstat = 0;
			}
			break;
		case USB_PID_IN:
			buf = urb->transfer_buffer + urb->actual_length;
			prefetchw(buf);
			len = fsh_index_readw(epfifo, EP_COUNT);
			if (len > ep->length) {
				printk("   USB_PID_IN(OverFlow): len=%d, ep->length=%d\n",
					len, ep->length);
				len = ep->length;
				urbstat = -EOVERFLOW;
			}
			urb->actual_length += len;
			for (i = 0; i < len; i++)
				*buf++ = fsh_readb(ep_fifos[epfifo]);
			if (ep->epnum == 0) {
				u8 regval = fsh_index_readb(epfifo, CTLSTS_EP_TXL);
				regval &= ~(1 << 0);
				fsh_index_writeb(epfifo, regval, CTLSTS_EP_TXL);
			} else {
				u8 regval = fsh_index_readb(epfifo, CTLSTS_EP_RXL);
				regval &= ~(1 << 0);
				fsh_index_writeb(epfifo, regval, CTLSTS_EP_RXL);
			}
				
			usb_dotoggle(udev, ep->epnum, 0);
			if (urbstat == -EINPROGRESS &&
					(len < ep->maxpacket ||
						urb->actual_length ==
						urb->transfer_buffer_length)) {
				if (usb_pipecontrol(urb->pipe)) {
					VDBG("NEXT Packet: Status Packet.\n");
					ep->nextpid = USB_PID_ACK;
				}
				else
					urbstat = 0;
			}
			break;
		case USB_PID_SETUP:
			if (urb->transfer_buffer_length == urb->actual_length) {
				VDBG("NEXT Packet: Status Packet.\n");
				ep->nextpid = USB_PID_ACK;
			}
			else if (usb_pipeout(urb->pipe)) {
				VDBG("NEXT Packet: OUT Packet.\n");
				usb_settoggle(udev, 0, 1, 1);
				ep->nextpid = USB_PID_OUT;
			} else {
				VDBG("NEXT Packet: IN Packet.\n");
				usb_settoggle(udev, 0, 0, 1);
				ep->nextpid = USB_PID_IN;
			}
			break;
		case USB_PID_ACK:
			if (!is_out) {
				u8 regval = fsh_index_readb(epfifo, CTLSTS_EP_RXL);
				regval &= ~(1 << 0);
				fsh_index_writeb(epfifo, regval, CTLSTS_EP_RXL);
			}
			urbstat = 0;
			break;
		}

	}

	if (urbstat != -EINPROGRESS || urb->unlinked) {
		if (ep->epnum == 0)
			finish_request_ep0(ak98fsh, ep, urb, urbstat);
		else
			finish_request_epx(ak98fsh, ep, urb, urbstat);
	}
}

static irqreturn_t ak98fsh_irq(struct usb_hcd *hcd)
{
	struct ak98fsh	*ak98fsh = hcd_to_ak98fsh(hcd);
	irqreturn_t	ret = IRQ_NONE;
	struct urb *urb;
	unsigned int pipe;
	int is_out;
	struct ak98fsh_ep *ep;
	int epnum[MAX_EP_NUM + 1] = { 0 };
	int epfifo = 0;
	int i;
	unsigned index = 0;
	
	char rINTCOM;
	unsigned short rINTTX, rINTRX;
	
	spin_lock(&ak98fsh->lock);
	
	/*Read & Clear all interrupt status.*/
	rINTCOM = fsh_readb(UR_INTCOM);
	rINTTX = fsh_readw(UR_INTTX);
	rINTRX = fsh_readw(UR_INTRX);
	
	if (rINTTX & 0x1) {
		epnum[0] = 1;
		done(ak98fsh, ak98fsh->active_ep0);
		ak98fsh->active_ep0 = NULL;
		ak98fsh->stat_ep0++;
	}
	
	for(i=0; i<MAX_EP_NUM; i++)
		if((rINTTX & (1<<(i+1))) || (rINTRX & (1<<(i+1)))) {
			epnum[i + 1] = 1;
			done(ak98fsh, ak98fsh->active_epx[i]);
			ak98fsh->active_epx[i] = NULL;
			ak98fsh->stat_epx[i]++;
	}

	if (rINTCOM & INTR_SOF) { 
		index = ak98fsh->frame++ & (PERIODIC_SIZE - 1);
		ak98fsh->stat_sof++;

		/* be graceful about almost-inevitable periodic schedule
		 * overruns:  continue the previous frame's transfers iff
		 * this one has nothing scheduled.
		 */
		if (ak98fsh->next_periodic) {
			ak98fsh->stat_overrun++;
		}
		if (ak98fsh->periodic[index]) {
			ak98fsh->next_periodic = ak98fsh->periodic[index];
			}
	}

	/* manages debouncing and wakeup */
	if(rINTCOM & (INTR_CONNECTED | INTR_DISCONNECTED)) {
		if (rINTCOM & INTR_CONNECTED) {
			//printk("USB: Device Connected!!\n");
			ak98fsh->stat_insrmv = 1;
		} else {
			//printk("USB: Device Disconnected!!\n");
			period_epfifo = 0;
			ak98fsh->stat_insrmv = 0;
		}

		/* most stats are reset for each VBUS session */
		ak98fsh->stat_wake = 0;
		ak98fsh->stat_sof = 0;
		ak98fsh->stat_ep0 = 0;
		for(i=0; i<MAX_EP_NUM; i++)
			ak98fsh->stat_epx[i] = 0;
		ak98fsh->stat_lost = 0;

		/* usbcore nukes other pending transactions on disconnect */
		if (ak98fsh->active_ep0) {
			VDBG("Finishing EP0 Active URBs...\n");
			ep = ak98fsh->active_ep0;
			urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
			pipe = urb->pipe;
			is_out = !usb_pipein(pipe);
			epfifo = 0;
			fsh_index_writeb(epfifo, 0, CTLSTS_EP_TXL);
			fsh_index_writeb(epfifo, 1 << 0, CTLSTS_EP_TXH);

			finish_request_ep0(ak98fsh, ak98fsh->active_ep0,
				container_of(ak98fsh->active_ep0
						->hep->urb_list.next,
					struct urb, urb_list),
				-ESHUTDOWN);
			ak98fsh->active_ep0 = NULL;
		}

		for(i=0; i<MAX_EP_NUM; i++) 
			if (ak98fsh->active_epx[i]) {
				VDBG("Finishing EPx Active URBs...\n");
				ep = ak98fsh->active_epx[i];
				urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
				pipe = urb->pipe;
				is_out = !usb_pipein(pipe);
				//epnum = ep->epnum;
				if (!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, is_out, &epfifo))
					BUG();
				if (is_out) {
					fsh_index_writeb(epfifo, 1 << 3, CTLSTS_EP_TXL);
				} else {
					fsh_index_writeb(epfifo, 1 << 4, CTLSTS_EP_RXL);
				}

				finish_request_epx(ak98fsh, ak98fsh->active_epx[i],
					container_of(ak98fsh->active_epx[i]
							->hep->urb_list.next,
						struct urb, urb_list),
					-ESHUTDOWN);
				ak98fsh->active_epx[i] = NULL;
			}

		/* port status seems weird until after reset, so
		 * force the reset and make khubd clean up later.
		 */
		if (rINTCOM & INTR_CONNECTED) {
			ak98fsh->port_status |= 1 << USB_PORT_FEAT_CONNECTION;
			ak98fsh->port_status |= 1 << USB_PORT_FEAT_C_CONNECTION;
		} else {
			ak98fsh->port_status &= ~(1 << USB_PORT_FEAT_CONNECTION);
			ak98fsh->port_status |= 1 << USB_PORT_FEAT_C_CONNECTION;
			init_epfifo_mapping(&ak98fsh_epfifo_mapping);
			reset_endpoints();
		}
	}

	if (rINTCOM & INTR_RESUME) {
		if (ak98fsh->port_status & (1 << USB_PORT_FEAT_SUSPEND)) {
			HDBG("wakeup\n");
			ak98fsh->port_status |= 1 << USB_PORT_FEAT_C_SUSPEND;
			ak98fsh->stat_wake++;
		}
		rINTCOM &= ~(INTR_RESUME);
	}

	if (ak98fsh->port_status & (1 << USB_PORT_FEAT_ENABLE)) {
		if (epnum[0]) {
			start_transfer(ak98fsh, 0);
			ret = IRQ_HANDLED;
		}
		if((rINTCOM & INTR_SOF) && ak98fsh->periodic[index] && period_epfifo){
			start_transfer(ak98fsh, period_epfifo);
			ret = IRQ_HANDLED;
		}
		for(i = 0; i < MAX_EP_NUM; i++) {
			if(epnum[i + 1]) {
				start_transfer(ak98fsh, i + 1);
				ret = IRQ_HANDLED;
			}
		}
	}

	if(ak98fsh->periodic_count == 0 && list_empty(&ak98fsh->async_ep0)) {
			for(i = 0; i < MAX_EP_NUM; i++) {
					if(!list_empty(&ak98fsh->async_epx[i]))
							break;
			}
			if(i == MAX_EP_NUM)
					sofirq_off(ak98fsh);
	}

	spin_unlock(&ak98fsh->lock);
	
	return ret;
}

/*-------------------------------------------------------------------------*/

/* usb 1.1 says max 90% of a frame is available for periodic transfers.
 * this driver doesn't promise that much since it's got to handle an
 * IRQ per packet; irq handling latencies also use up that time.
 *
 * NOTE:  the periodic schedule is a sparse tree, with the load for
 * each branch minimized.  see fig 3.5 in the OHCI spec for example.
 */
#define	MAX_PERIODIC_LOAD	500	/* out of 1000 usec */

static int balance(struct ak98fsh *ak98fsh, u16 period, u16 load)
{
	int	i, branch = -ENOSPC;

	/* search for the least loaded schedule branch of that period
	 * which has enough bandwidth left unreserved.
	 */
	for (i = 0; i < period ; i++) {
		if (branch < 0 || ak98fsh->load[branch] > ak98fsh->load[i]) {
			int	j;

			for (j = i; j < PERIODIC_SIZE; j += period) {
				if ((ak98fsh->load[j] + load)
						> MAX_PERIODIC_LOAD)
					break;
			}
			if (j < PERIODIC_SIZE)
				continue;
			branch = i;
		}
	}
	return branch;
}

/*-------------------------------------------------------------------------*/

static int ak98fsh_urb_enqueue(
	struct usb_hcd		*hcd,
	struct urb		*urb,
	gfp_t			mem_flags
) {
	struct ak98fsh		*ak98fsh = hcd_to_ak98fsh(hcd);
	struct usb_device	*udev = urb->dev;
	unsigned int		pipe = urb->pipe;
	int			is_out = !usb_pipein(pipe);
	int			type = usb_pipetype(pipe);
	int			epnum = usb_pipeendpoint(pipe);
#ifdef DYNAMIC_EPFIFO
	int			epfifo = 0;
#endif
	struct ak98fsh_ep	*ep = NULL;
	unsigned long		flags;
	int			i;
	int			retval;
	struct usb_host_endpoint	*hep = urb->ep;

	VDBG("Enqueue: Direction=%s, Type=%s, EP Num=%d, urb=%p\n",
		is_out ? "OUT" : "IN", trans_desc[type], epnum, urb);

	if (type == PIPE_ISOCHRONOUS)
		return -ENOSPC;

#ifdef DYNAMIC_EPFIFO
	spin_lock_irqsave(&ak98fsh_epfifo_mapping.lock,flags);
	if (!__is_epnum_mapped(&ak98fsh_epfifo_mapping, epnum, is_out)) {
		if (!__map_epnum_to_epfifo(&ak98fsh_epfifo_mapping, epnum, is_out, &epfifo)) {
			spin_unlock_irqrestore(&ak98fsh_epfifo_mapping.lock, flags);
			return -ENOSPC;
		}
		if (epnum != 0) {
			int eptype = 0;

			if (usb_pipeisoc(pipe)) {
				eptype = 1;
			} else if (usb_pipeint(pipe)) {
				eptype = 3;
			} else if (usb_pipebulk(pipe)) {
				eptype = 2;
			} else BUG();

			if (is_out) {
				disable_epx_tx_interrupt(epfifo);
				set_epx_tx_type(epfifo, epnum, eptype);
				fsh_index_writew(epfifo, hep->desc.wMaxPacketSize, MaxPackSz_TX);
				fsh_index_writeb(epfifo, 0, NAK_TO_EP0);	/* Tx Interval */
				clear_epx_tx_data_toggle(epfifo);
				set_epx_tx_mode(epfifo);
				flush_epx_tx_fifo(epfifo);
				enable_epx_tx_interrupt(epfifo);
			} else {
				disable_epx_rx_interrupt(epfifo);
				set_epx_rx_type(epfifo, epnum, eptype);
				fsh_index_writew(epfifo, hep->desc.wMaxPacketSize, MaxPackSz_RX);
				fsh_index_writeb(epfifo, 0, NAK_TO_EP0);	/* Tx Interval */
				if (usb_endpoint_xfer_isoc(&hep->desc) || usb_endpoint_xfer_int(&hep->desc))
					fsh_index_writeb(epfifo, hep->desc.bInterval, RX_INTVAL);
				else
					fsh_index_writeb(epfifo, 0, RX_INTVAL);
				set_epx_rx_mode(epfifo);
				clear_epx_rx_data_toggle(epfifo);
				flush_epx_rx_fifo(epfifo);
				enable_epx_rx_interrupt(epfifo);
			}
		}
	} else {
		if(!epnum_to_epfifo(&ak98fsh_epfifo_mapping, epnum, is_out, &epfifo))
			BUG();
	}

	spin_unlock_irqrestore(&ak98fsh_epfifo_mapping.lock, flags);
#endif

	/* avoid all allocations within spinlocks */
	if (!hep->hcpriv)
		ep = kzalloc(sizeof *ep, mem_flags);

	spin_lock_irqsave(&ak98fsh->lock, flags);

	/* don't submit to a dead or disabled port */
	if (!(ak98fsh->port_status & (1 << USB_PORT_FEAT_ENABLE))
			|| !HC_IS_RUNNING(hcd->state)) {
		retval = -ENODEV;
		kfree(ep);
		goto fail_not_linked;
	}

	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if (retval) {
		kfree(ep);
		goto fail_not_linked;
	}

	if (hep->hcpriv) {
		kfree(ep);
		ep = hep->hcpriv;
	} else if (!ep) {
		retval = -ENOMEM;
		goto fail;
	} else {
		INIT_LIST_HEAD(&ep->schedule);
		ep->udev = udev;
		ep->epnum = epnum;
		ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
		usb_settoggle(udev, epnum, is_out, 0);

		if (type == PIPE_CONTROL)
			ep->nextpid = USB_PID_SETUP;
		else if (is_out)
			ep->nextpid = USB_PID_OUT;
		else
			ep->nextpid = USB_PID_IN;

		if (ep->maxpacket > H_MAXPACKET) {
			/* iso packets up to 240 bytes could work... */
			HDBG("dev %d ep%d maxpacket %d\n",
				udev->devnum, epnum, ep->maxpacket);
			retval = -EINVAL;
			goto fail;
		}

		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			if (urb->interval > PERIODIC_SIZE)
				urb->interval = PERIODIC_SIZE;
			ep->period = urb->interval;
			ep->branch = PERIODIC_SIZE;
			ep->load = usb_calc_bus_time(udev->speed, !is_out,
				(type == PIPE_ISOCHRONOUS),
				usb_maxpacket(udev, pipe, is_out))
					/ 1000;
			period_epfifo = epfifo;
			break;
		}

		ep->hep = hep;
		hep->hcpriv = ep;
	}

	/* maybe put endpoint into schedule */
	switch (type) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		if (list_empty(&ep->schedule)) {
			if (epnum == 0) 
				list_add_tail(&ep->schedule, &ak98fsh->async_ep0);
			else 
				list_add_tail(&ep->schedule, &ak98fsh->async_epx[epfifo-1]);
		}
		break;
	case PIPE_ISOCHRONOUS:
	case PIPE_INTERRUPT:
		urb->interval = ep->period;
		if (ep->branch < PERIODIC_SIZE) {
			/* NOTE:  the phase is correct here, but the value
			 * needs offsetting by the transfer queue depth.
			 * All current drivers ignore start_frame, so this
			 * is unlikely to ever matter...
			 */
			urb->start_frame = (ak98fsh->frame & (PERIODIC_SIZE - 1))
						+ ep->branch;
			break;
		}

		retval = balance(ak98fsh, ep->period, ep->load);
		if (retval < 0)
			goto fail;
		ep->branch = retval;
		retval = 0;
		urb->start_frame = (ak98fsh->frame & (PERIODIC_SIZE - 1))
					+ ep->branch;

		/* sort each schedule branch by period (slow before fast)
		 * to share the faster parts of the tree without needing
		 * dummy/placeholder nodes
		 */
		VDBG("schedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
		for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period) {
			struct ak98fsh_ep	**prev = &ak98fsh->periodic[i];
			struct ak98fsh_ep	*here = *prev;

			while (here && ep != here) {
				if (ep->period > here->period)
					break;
				prev = &here->next;
				here = *prev;
			}
			if (ep != here) {
				ep->next = here;
				*prev = ep;
			}
			ak98fsh->load[i] += ep->load;
		}
		ak98fsh->periodic_count++;
		hcd->self.bandwidth_allocated += ep->load / ep->period;
		sofirq_on(ak98fsh);
	}


	urb->hcpriv = hep;
	start_transfer(ak98fsh, epfifo);

fail:
	if (retval)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
fail_not_linked:
	spin_unlock_irqrestore(&ak98fsh->lock, flags);
	return retval;
}

static int ak98fsh_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct ak98fsh		*ak98fsh = hcd_to_ak98fsh(hcd);
	struct usb_host_endpoint *hep;
	unsigned long		flags;
	struct ak98fsh_ep	*ep;
	int			retval, i;
	unsigned int		pipe = urb->pipe;
	int			is_out = !usb_pipein(pipe);
	int			epfifo = 0;
	
	VDBG("Dequeue: Direction=%s, Type=%s, EP Num=%d, urb=%p\n",
		is_out ? "OUT" : "IN", trans_desc[usb_pipetype(pipe)], usb_pipeendpoint(pipe), urb);

	spin_lock_irqsave(&ak98fsh->lock, flags);

	retval = usb_hcd_check_unlink_urb(hcd, urb, status);

	if (retval) {
		printk("Dequeue: check and unlink urb failed!\n");
		goto fail;
	}

	hep = urb->hcpriv;
	ep = hep->hcpriv;
	if (ep) {
		/* finish right away if this urb can't be active ...
		 * note that some drivers wrongly expect delays
		 */
		if (ep->hep->urb_list.next != &urb->urb_list) {
			/* not front of queue?  never active */

		/* for active transfers, we expect an IRQ */
		} else if (ak98fsh->active_ep0 == ep) {
			if (time_before_eq(ak98fsh->jiffies_ep0, jiffies)) {
				epfifo = 0;
				fsh_index_writeb(epfifo, 0, CTLSTS_EP_TXL);
				fsh_index_writeb(epfifo, 1 << 0, CTLSTS_EP_TXH);
				ak98fsh->active_ep0 = NULL;
			} else
				urb = NULL;
		} else {
			for(i=0; i<MAX_EP_NUM; i++) {
				if(ak98fsh->active_epx[i] == ep) {
					if(time_before_eq(ak98fsh->jiffies_epx[i], jiffies)) {
						if(!epnum_to_epfifo(&ak98fsh_epfifo_mapping, ep->epnum, is_out, &epfifo))
							BUG();
						if (is_out) {
							fsh_index_writeb(epfifo, 1 << 3, CTLSTS_EP_TXL);
						} else {
							fsh_index_writeb(epfifo, 1 << 4, CTLSTS_EP_RXL);
						}
						ak98fsh->active_epx[i] = NULL;
					} else
						urb = NULL;
				}
			}
			/* front of queue for inactive endpoint */
		}

		if (urb) {
			if (ak98fsh->active_ep0 == ep)
				finish_request_ep0(ak98fsh, ep, urb, 0);
			else
				finish_request_epx(ak98fsh, ep, urb, 0);
		} else {
			HDBG("dequeue, urb %p active ; wait4irq\n", urb);
		}
	} else
		retval = -EINVAL;
 fail:
	spin_unlock_irqrestore(&ak98fsh->lock, flags);
	return retval;
}

static void
ak98fsh_endpoint_reset(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct ak98fsh		*ak98fsh = hcd_to_ak98fsh(hcd);
	int			epnum = usb_endpoint_num(&hep->desc);
	unsigned long		flags;

	spin_lock_irqsave(&ak98fsh->lock, flags);

	HDBG("Resetting EP %d, Type=%s, Dir=%s\n",
		epnum, xfer_name[usb_endpoint_type(&hep->desc)], usb_endpoint_dir_out(&hep->desc)? "OUT" : "IN");

	if (epnum == 0) {
		fsh_index_writeb(0, 0, NAK_TO_EP0);
		fsh_index_writew(0, 0, CTLSTS_EP_TXL);
		flush_ep0_fifo();
		enable_ep0_interrupt();
	} else {
#ifndef DYNAMIC_EPFIFO
		int eptype = usb_endpoint_type(&hep->desc);
		int is_out = usb_endpoint_dir_out(&hep->desc);
		if (is_out) {
			disable_epx_tx_interrupt(epnum);
			set_epx_tx_type(epnum, epnum, eptype);
			fsh_index_writew(epnum, hep->desc.wMaxPacketSize, MaxPackSz_TX);
			fsh_index_writeb(epnum, 0, NAK_TO_EP0);	/* Tx Interval */
			clear_epx_tx_data_toggle(epnum);
			set_epx_tx_mode(epnum);
			flush_epx_tx_fifo(epnum);
			enable_epx_tx_interrupt(epnum);
		} else {
			disable_epx_rx_interrupt(epnum);
			set_epx_rx_type(epnum, epnum, eptype);
			fsh_index_writew(epnum, hep->desc.wMaxPacketSize, MaxPackSz_RX);
			fsh_index_writeb(epnum, 0, NAK_TO_EP0);	/* Tx Interval */
			if (usb_endpoint_xfer_isoc(&hep->desc) || usb_endpoint_xfer_int(&hep->desc))
				fsh_index_writeb(epnum, hep->desc.bInterval, RX_INTVAL);
			else
				fsh_index_writeb(epnum, 0, RX_INTVAL);
			set_epx_rx_mode(epnum);
			clear_epx_rx_data_toggle(epnum);
			flush_epx_rx_fifo(epnum);
			enable_epx_rx_interrupt(epnum);
		}
#endif
	}

	spin_unlock_irqrestore(&ak98fsh->lock, flags);
}

static void
ak98fsh_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct ak98fsh_ep	*ep = hep->hcpriv;
#ifdef DYNAMIC_EPFIFO
	int			epnum = usb_endpoint_num(&hep->desc);
	int			is_out = usb_endpoint_dir_out(&hep->desc);
#endif
	int			epfifo = 0;


	if (!ep) {
		return;
	}
#ifdef DYNAMIC_EPFIFO
	if (is_epnum_mapped(&ak98fsh_epfifo_mapping, epnum, is_out)) {
#else
	if (1) {
#endif
		disable_ep_interrupt(epfifo);
		flush_ep_fifo(epfifo);
	}
	/* assume we'd just wait for the irq */
	if (!list_empty(&hep->urb_list))
		msleep(3);
	if (!list_empty(&hep->urb_list))
		HDBG("ep %p not empty?\n", ep);

	kfree(ep);
	hep->hcpriv = NULL;

}

static int
ak98fsh_get_frame(struct usb_hcd *hcd)
{
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);

	/* wrong except while periodic transfers are scheduled;
	 * never matches the on-the-wire frame;
	 * subject to overruns.
	 */
	return ak98fsh->frame;
}


/*-------------------------------------------------------------------------*/

/* the virtual root hub timer IRQ checks for hub status */
static int
ak98fsh_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);
	unsigned long flags;

	/* non-SMP HACK: use root hub timer as i/o watchdog
	 * this seems essential when SOF IRQs aren't in use...
	 */
	local_irq_save(flags);
	if (!timer_pending(&ak98fsh->timer)) {
		if (ak98fsh_irq( /* ~0, */ hcd) != IRQ_NONE)
			ak98fsh->stat_lost++;
	}
	local_irq_restore(flags);

	if (!(ak98fsh->port_status & (0xffff << 16))) {
		return 0;
	}

	/* tell khubd port 1 changed */
	*buf = (1 << 1);

	return 1;
}

static void
ak98fsh_hub_descriptor (
	struct ak98fsh *ak98fsh,
	struct usb_hub_descriptor	*desc
) {
	u16		temp = 0;

	desc->bDescriptorType = 0x29;
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = 1;
	desc->bDescLength = 9;

	/* per-port power switching (gang of one!), or none */
	desc->bPwrOn2PwrGood = 0;

	/* no overcurrent errors detection/handling */
	temp |= 0x0010; 

	desc->wHubCharacteristics = cpu_to_le16(temp);

	/* two bitmaps:  ports removable, and legacy PortPwrCtrlMask */
	desc->bitmap[0] = 0 << 1; 
	desc->bitmap[1] = ~0;
}

static void
ak98fsh_timer(unsigned long _ak98fsh)
{
	struct ak98fsh *ak98fsh = (void *) _ak98fsh;
	unsigned long	flags;
	const u32	mask = (1 << USB_PORT_FEAT_CONNECTION)
				| (1 << USB_PORT_FEAT_ENABLE);

	spin_lock_irqsave(&ak98fsh->lock, flags);

	if (ak98fsh->port_status & USB_PORT_STAT_RESET) {
		ak98fsh->port_status = (1 << USB_PORT_FEAT_C_RESET)
				| (1 << USB_PORT_FEAT_POWER);
		ak98fsh->port_status |= mask;
		
		if (ak98fsh->port_status & (1 << USB_PORT_FEAT_CONNECTION)) {
			if (fsh_readb(DEV_CTL_REG) & (1 << 5))
				ak98fsh->port_status |= (1 << USB_PORT_FEAT_LOWSPEED);
			else if ((fsh_readb(DEV_CTL_REG) & (1 << 6))) {
			} else {
				/* Plug-in & plug-out quickly could lead to this... */
				ak98fsh->port_status &= ~mask;
			}
		}
	} else {
		/* NOT IMPLEMENTED YET */
		BUG();
	}
	
	spin_unlock_irqrestore(&ak98fsh->lock, flags);

}

static int
ak98fsh_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);
	int		retval = 0;
	unsigned long	flags;
	char reg8val;

	spin_lock_irqsave(&ak98fsh->lock, flags);
	
	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;
		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			HDBG("ClearPortFeature: USB_PORT_FEAT_ENABLE\n");
			ak98fsh->port_status &= (1 << USB_PORT_FEAT_POWER);
			break;
		case USB_PORT_FEAT_SUSPEND:
			HDBG("ClearPortFeature: USB_PORT_FEAT_SUSPEND\n");
			if (!(ak98fsh->port_status & (1 << USB_PORT_FEAT_SUSPEND)))
				break;

			/* 20 msec of resume/K signaling, other irqs blocked */
			HDBG("    start resume...\n");
			fsh_writeb(0x0, UR_INTECOM); 
			reg8val = fsh_readb(UR_PWM); 
			reg8val |= 1<<2;
			fsh_writeb(reg8val, UR_PWM);
			
			mod_timer(&ak98fsh->timer, jiffies
					+ msecs_to_jiffies(20));
			break;
		case USB_PORT_FEAT_POWER:
			port_power(ak98fsh, 0);
			break;
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_CONNECTION:
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			break;
		default:
			goto error;
		}
		ak98fsh->port_status &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		ak98fsh_hub_descriptor(ak98fsh, (struct usb_hub_descriptor *) buf);
		break;
	case GetHubStatus:
		put_unaligned_le32(0, buf);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;
		put_unaligned_le32(ak98fsh->port_status, buf);
		break;
	case SetPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			HDBG("    USB_PORT_FEAT_SUSPEND\n");
			if (ak98fsh->port_status & (1 << USB_PORT_FEAT_RESET))
				goto error;
			if (!(ak98fsh->port_status & (1 << USB_PORT_FEAT_ENABLE)))
				goto error;
			/*to suspend the usb host controller.*/
			reg8val = fsh_readb(UR_PWM);
			reg8val |= 1<<1;
			fsh_writeb(reg8val, UR_PWM);
			break;
		case USB_PORT_FEAT_POWER: 
			HDBG("    USB_PORT_FEAT_POWER\n");
			port_power(ak98fsh, 1);
			break;
		case USB_PORT_FEAT_RESET:
			HDBG("    USB_PORT_FEAT_RESET, Port Status=0x%08X\n", ak98fsh->port_status);
			if (ak98fsh->port_status & (1 << USB_PORT_FEAT_SUSPEND)) {
				HDBG("    USB_PORT_FEAT_SUSPEND ....\n");
				goto error;
			}
			if (!(ak98fsh->port_status & (1 << USB_PORT_FEAT_POWER))) {
				HDBG("    USB_PORT_FEAT_POWER NOT SET.\n");
				break;
			}
			clear_all_interrupts();

			/* 50 msec of reset/SE0 signaling, irqs blocked */
			/*reset device.*/
			reg8val = fsh_readb(UR_PWM);
			reg8val |= 0x08;
			fsh_writeb(reg8val, UR_PWM);
			mdelay(30);
			reg8val &= ~(0x08);
			fsh_writeb(reg8val, UR_PWM);

			fsh_writeb(0xF7, UR_INTECOM);

			ak98fsh->port_status |= (1 << USB_PORT_FEAT_RESET);
			mod_timer(&ak98fsh->timer, jiffies + msecs_to_jiffies(50));
			break;
		default:
			goto error;
		}

		ak98fsh->port_status |= 1 << wValue;
		break;
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}

	spin_unlock_irqrestore(&ak98fsh->lock, flags);
	return retval;
}

#ifdef	CONFIG_PM

static int
ak98fsh_bus_suspend(struct usb_hcd *hcd)
{
	u8 reg;
	unsigned long flags;

	msleep(10);
	local_irq_save(flags);
	reg = fsh_readb(UR_PWM);
	reg |= (1 << 1);
	fsh_writeb(reg, UR_PWM);
	local_irq_restore(flags);
	msleep(20);
	return 0;
}

static int
ak98fsh_bus_resume(struct usb_hcd *hcd)
{
	u8 reg;
	unsigned long flags;

	local_irq_save(flags);
	reg = fsh_readb(UR_PWM);
	reg |= (1 << 2);
	fsh_writeb(reg, UR_PWM);
	local_irq_restore(flags);
	msleep(20);
	local_irq_save(flags);
	reg = fsh_readb(UR_PWM);
	reg &= ~(1 << 2);
	fsh_writeb(reg, UR_PWM);
	local_irq_restore(flags);
	msleep(100);
	return 0;
}

#else

#define	ak98fsh_bus_suspend	NULL
#define	ak98fsh_bus_resume	NULL

#endif

/*-------------------------------------------------------------------------*/
static void disable_clock(void)
{
	/*disenable working clock of usb2.0 FS host controller.*/
	rCLK_CON2 |= 1<<1; 
	/*close 60M pll1 for usb2.0 fs host.*/
	rMULFUN_CON1 |= 1<<7; 
}

static void
ak98fsh_stop(struct usb_hcd *hcd)
{
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);
	unsigned long	flags;

	del_timer_sync(&hcd->rh_timer);
	disable_clock();
	
	spin_lock_irqsave(&ak98fsh->lock, flags);
	port_power(ak98fsh, 0);
	spin_unlock_irqrestore(&ak98fsh->lock, flags);
}

static void enable_clock(void)
{
	/*enable the related working clocks.*/
	rCLK_CON2 &= ~(1<<1); 

	/*Then open usb module 60M PLL1.*/
	rMULFUN_CON1 &= (~((0x1<<3)|(3<<6))); 
}

static void usbhost_reset(void)
{
	/*softreset the usb host controller.*/
	rCLK_CON2 |= 1<<17;
	mdelay(20);
	rCLK_CON2 &= ~(1<<17);

	/*To reset the 60M PLL of usb module.*/
	rMULFUN_CON1 |= 1<<6; 
	mdelay(20);
	rMULFUN_CON1 &= ~(1<<6); 
}


static int
ak98fsh_start(struct usb_hcd *hcd)
{
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);

	/*after reset usbhc, set stat to running.*/
	hcd->state = HC_STATE_RUNNING;
	
	/* chip has been reset, VBUS power is off */
	port_power(ak98fsh, 1);
	enable_clock();
	
	fsh_writeb(0x0, UR_INTECOM);
	fsh_writew(0x0, UR_INTETX);
	fsh_writew(0x0, UR_INTERX);

	fsh_writeb(0x0, UR_INDEX);

	/*enable the bus.*/
	rMULFUN_CON1 &= ~(1<<3); 
	msleep(1);
		
	/* start fs host session*/ 
	fsh_writeb(0x1, DEV_CTL_REG);

	clear_all_interrupts();
	fsh_writeb(0x0, UR_FUNADDR);

	reset_endpoints();
	fsh_writeb(0xF7, UR_INTECOM);
	
	return 0;
}

static struct hc_driver ak98fsh_hc_driver = {
	.description		= hcd_name,
	.product_desc 		= "Anyka AK98 USB Full Speed Host Controller",
	.hcd_priv_size 		= sizeof(struct ak98fsh),

	/*
	 * generic hardware linkage
	 */
	.irq			= ak98fsh_irq,
	.flags			= HCD_USB11 | HCD_MEMORY,

	/* Basic lifecycle operations */
	.start			= ak98fsh_start,
	.stop			= ak98fsh_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ak98fsh_urb_enqueue,
	.urb_dequeue		= ak98fsh_urb_dequeue,
	.endpoint_reset		= ak98fsh_endpoint_reset,
	.endpoint_disable	= ak98fsh_endpoint_disable,

	/*
	 * periodic schedule support
	 */
	.get_frame_number	= ak98fsh_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ak98fsh_hub_status_data,
	.hub_control		= ak98fsh_hub_control,
	.bus_suspend		= ak98fsh_bus_suspend,
	.bus_resume		= ak98fsh_bus_resume,
};

/*-------------------------------------------------------------------------*/

static int __devexit
ak98fsh_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct ak98fsh *ak98fsh = hcd_to_ak98fsh(hcd);
	struct resource		*res;

	usb_remove_hcd(hcd);

	/* some platforms may use IORESOURCE_IO */
	res = platform_get_resource(dev, IORESOURCE_MEM, 1);
	if (res)
		iounmap(ak98fsh->data_reg);

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res)
		iounmap(ak98fsh->addr_reg);

	usb_put_hcd(hcd);
	return 0;
}

static int __devinit
ak98fsh_probe(struct platform_device *dev)
{
	struct usb_hcd *hcd;
	struct ak98fsh *ak98fsh;
	struct resource		*addr;
	int			irq;
	void __iomem		*addr_reg;
	int			retval;
	u8			ioaddr = 0;
	unsigned long		irqflags;
	int i, j;

	/* basic sanity checks first.  board-specific init logic should
	 * have initialized these three resources and probably board
	 * specific platform_data.  we don't probe for IRQs, and do only
	 * minimal sanity checking.
	 */
	irq = platform_get_irq(dev, 0);
	if (irq <= 0) {
			dev_err(&dev->dev,
				"Found HC with no IRQ. Check %s setup!\n",
				dev_name(&dev->dev));
			retval = -ENODEV;
			goto err_nodev;
	}

	addr = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if(!addr) { 
		addr = platform_get_resource(dev, IORESOURCE_IO, 0);
		if(!addr)
			return -ENODEV;
		ioaddr = 1;
		addr_reg = (void __iomem *) (unsigned long) addr->start;
	} else {
		addr_reg = ioremap(addr->start, 1);
		if (addr_reg == NULL) {
			retval = -ENOMEM;
			goto err_nomem;
		}
	}
	
	/* allocate and initialize hcd */
	hcd = usb_create_hcd(&ak98fsh_hc_driver, &dev->dev, dev_name(&dev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err_nomem;
	}
	
	hcd->rsrc_start = addr->start;
	ak98fsh = hcd_to_ak98fsh(hcd);

	spin_lock_init(&ak98fsh->lock);
	INIT_LIST_HEAD(&ak98fsh->async_ep0);
	for(i=0; i<MAX_EP_NUM; i++)
		INIT_LIST_HEAD(&ak98fsh->async_epx[i]);
	ak98fsh->board = dev->dev.platform_data;
	init_timer(&ak98fsh->timer);
	ak98fsh->timer.function = ak98fsh_timer;
	ak98fsh->timer.data = (unsigned long) ak98fsh;
	ak98fsh->addr_reg = 0;
	ak98fsh->data_reg = 0;
	ak98fsh->active_ep0 = NULL;
	for(j=0; j<MAX_EP_NUM; j++)
		ak98fsh->active_epx[j] = NULL;
	spin_lock_irq(&ak98fsh->lock);
	port_power(ak98fsh, 0);
	spin_unlock_irq(&ak98fsh->lock);
	msleep(200);

	/* The chip's IRQ is level triggered, active high.  A requirement
	 * for platform device setup is to cope with things like signal
	 * inverters (e.g. CF is active low) or working only with edge
	 * triggers (e.g. most ARM CPUs).  Initial driver stress testing
	 * was on a system with single edge triggering, so most sorts of
	 * triggering arrangement should work.
	 *
	 * Use resource IRQ flags if set by platform device setup.
	 */
	irqflags = IRQF_SHARED;
	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | irqflags);
	if (retval != 0)
		goto err_addhcd;

	init_epfifo_mapping(&ak98fsh_epfifo_mapping);

	printk("AK98 USB FS Controller Driver Initialized\n");

	return retval;

 err_addhcd:
	usb_put_hcd(hcd);
 err_nomem:
 err_nodev:
	printk("Failed to initialize AK98 USB FS Controller Driver\n");

	return retval;
}

#ifdef	CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub, except that the root hub only gets direct PM calls
 * when CONFIG_USB_SUSPEND is enabled.
 */

static int
ak98fsh_suspend(struct platform_device *dev, pm_message_t state)
{
	int		retval = 0;

	switch (state.event) {
	case PM_EVENT_FREEZE:
		break;
	case PM_EVENT_SUSPEND:
	case PM_EVENT_HIBERNATE:
	case PM_EVENT_PRETHAW:		/* explicitly discard hw state */
		break;
	}
	return retval;
}

static int
ak98fsh_resume(struct platform_device *dev)
{
	return 0;
}

#else

#define	ak98fsh_suspend	NULL
#define	ak98fsh_resume	NULL

#endif


struct platform_driver ak98fsh_driver = {
	.probe =	ak98fsh_probe,
	.remove =	__devexit_p(ak98fsh_remove),

	.suspend =	ak98fsh_suspend,
	.resume =	ak98fsh_resume,
	.driver = {
		.name =	(char *) hcd_name,
		.owner = THIS_MODULE,
	},
};
EXPORT_SYMBOL(ak98fsh_driver);

/*-------------------------------------------------------------------------*/

static int __init ak98fsh_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	return platform_driver_register(&ak98fsh_driver);
}
module_init(ak98fsh_init);

static void __exit ak98fsh_cleanup(void)
{
	platform_driver_unregister(&ak98fsh_driver);
}
module_exit(ak98fsh_cleanup);
