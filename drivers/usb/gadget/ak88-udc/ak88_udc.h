#ifndef _AK88_UDC_H
#define _AK88_UDC_H

/* ak880x usb register */
#define USB_FUNCTION_ADDR	0x0
#define USB_POWER_CTRL		0x1
#define USB_INTERRUPT_1		0x2
#define USB_INTERRUPT_2		0x4
#define USB_INTERRUPT_TX	0x6
#define USB_INTERRUPT_RX	0x8
#define USB_INTERRUPT_COMM	0xA
#define USB_INTERRUPT_USB	0xB
#define USB_FRAME_NUM		0xC
#define USB_EP_INDEX		0xE
#define USB_TEST_MODE		0xF
#define USB_TX_MAX		0x10
#define USB_CTRL_1		0x12
#define USB_RX_MAX		0x14
#define USB_CTRL_2		0x16
#define USB_EP_COUNT		0x18
#define USB_CFG_INFO		0x1F
#define USB_EP0_FIFO		0x20
#define USB_EP1_FIFO		0x24
#define USB_EP2_FIFO		0x28
#define USB_EP3_FIFO		0x2C
#define USB_EP4_FIFO		0x30
#define USB_DEVICE_CTRL		0x60
#define USB_DMA_INTR		0x200
#define USB_DMA_CTRL1		0x204
#define USB_DMA_CTRL2		0x214
#define USB_DMA_CTRL3		0x224
#define USB_DMA_ADDR1		0x208
#define USB_DMA_ADDR2		0x218
#define USB_DMA_ADDR3		0x228
#define USB_DMA_COUNT1		0x20C
#define USB_DMA_COUNT2		0x21C
#define USB_DMA_COUNT3		0x22C
#define USB_EP0_NUM		0x330
#define USB_EP2_NUM		0x334
#define USB_BUFFER_FORBIDDEN	0x338
#define USB_PREREAD_START	0x33C
#define USB_ADDR_CHANGE		0x340
#define USB_MODE_STATUS		0x344


// #define USB_11 [> usb1.1 <]   

#define EP0_FIFO_SIZE		64  /* control, not 64byte? */ 
#define EP1_FIFO_SIZE		64 /* interrupt */ 
#define EP4_FIFO_SIZE		512 /* iso */ 
#ifdef USB_11 /* USB_SPEED_FULL */ 
#define EP2_FIFO_SIZE		64 /* ep2 in bulk */ 
#define EP3_FIFO_SIZE		64 /* ep2 out bulk */ 
#else /* USB_SPEED_HIGH */ 
#define EP2_FIFO_SIZE		512 /* ep2 in bulk */ 
#define EP3_FIFO_SIZE		512 /* ep2 out bulk */ 
#endif


#define EP0_L2_ADDR		(AK88_VA_L2MEM + 0x1500) /* L2 buffer17 for otg control transfer */

struct ak880x_request;

struct ak880x_ep {
	struct usb_ep		ep;
	struct usb_gadget	*gadget;
	struct usb_endpoint_descriptor *desc;

	struct list_head	queue;

	struct work_struct	work;
	struct ak880x_request	*req; /* req be about to handle */ 

	struct ak880x_udc	*udc;
	int			maxpacket;
	volatile int		done;
	unsigned int		bufaddr;
	dma_addr_t		bufphys;

	unsigned		irq_enable;
	volatile unsigned	stopped;
	// unsigned		stopped:1;
	unsigned		is_in:1;
	unsigned		is_iso:1;
	// unsigned		fifo_bank:1;
	u8			l2_buf_id;
};

struct ak880x_request {
	struct list_head	queue;		/* ep's requests */
	struct usb_request	req;
	int			status;
};

enum ep0_status {
        EP0_IDLE,
        EP0_IN_DATA_PHASE,
        EP0_OUT_DATA_PHASE,
        EP0_END_XFER,
        EP0_STALL,
};

struct usb_l2 {
	void *buf;
	dma_addr_t phys;
};

static const char * const ep_name[] = {
	"ep0", "ep1-int", "ep2in-bulk", "ep3out-bulk", "ep4-iso",
};

#define ENDPOINTS_NUM ARRAY_SIZE(ep_name)

struct ak880x_udc {
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;

	struct ak880x_ep	ep[ENDPOINTS_NUM];
	enum ep0_status		ep0_status;

	void __iomem		*baseaddr;

	unsigned int	mcu_irq;
	unsigned int	dma_irq;
	struct clk	*clk;
	char		addr;   /* assigned device address */ 
	int		enabled;
};

static void udc_reinit(struct ak880x_udc *udc);
static void udc_enable(struct ak880x_udc *udc, int enable);
static void done(struct ak880x_ep *ep, struct ak880x_request *req, int status);
#endif
