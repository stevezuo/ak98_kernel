#if defined(CONFIG_SERIAL_AK88_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clock.h>


#define AK88_UART0_TXBUF_BASE	(AK88_VA_L2MEM + 0x1000)	/* offset 0x0[tx], 0x80[rx] */
#define AK88_UART0_RXBUF_BASE	(AK88_VA_L2MEM + 0x1080)	/* offset 0x0[tx], 0x80[rx] */
#define AK88_UART1_TXBUF_BASE	(AK88_VA_L2MEM + 0x1100)
#define AK88_UART1_RXBUF_BASE	(AK88_VA_L2MEM + 0x1180)
#define AK88_UART2_TXBUF_BASE	(AK88_VA_L2MEM + 0x1200)
#define AK88_UART2_RXBUF_BASE	(AK88_VA_L2MEM + 0x1280)
#define AK88_UART3_TXBUF_BASE	(AK88_VA_L2MEM + 0x1300)
#define AK88_UART3_RXBUF_BASE	(AK88_VA_L2MEM + 0x1380)

#define AK88_UART_BASE(x)	(((AK88_VA_UART) + ((x) * 0x1000)))

#define AK88_UART0_BASE	(AK88_VA_UART + 0x0000)
#define AK88_UART1_BASE	(AK88_VA_UART + 0x1000)
#define AK88_UART2_BASE	(AK88_VA_UART + 0x2000)
#define AK88_UART3_BASE	(AK88_VA_UART + 0x3000)

#define AK88_UART0_PA_BASE	(AK88_PA_UART + 0x0000)
#define AK88_UART1_PA_BASE	(AK88_PA_UART + 0x1000)
#define AK88_UART2_PA_BASE	(AK88_PA_UART + 0x2000)
#define AK88_UART3_PA_BASE	(AK88_PA_UART + 0x3000)

#define UART_CONF1		0x00
#define	UART_CONF2		0x04
#define	DATA_CONF		0x08
#define	BUF_THRESHOLD		0x0C
#define	UART_RXBUF		0x10
#define	RXBUF_THRESHOLD_EXT	0x14

#define	AKUART_INT_MASK		0x3FE00000

#define TX_THR_INT_ENABLE	(29)
#define RX_THR_INT_ENABLE	(28)
#define TX_END_INT_ENABLE	(27)
#if	defined(CONFIG_ARCH_AK7801)
#define	RXBUF_FULL_INT_ENABLE	(26)
#elif	defined(CONFIG_ARCH_AK88)
#define	RXBUF_FULL_INT_ENABLE	(25)
#endif
#define TXBUF_EMP_INT_ENABLE	(24)
#define RECVDATA_ERR_INT_ENABLE	(23)
#define RX_TIMEOUT_INT_ENABLE	(22)
#define	MEM_RDY_INT_ENABLE	(21)

#define TX_THR_INT		(31)
#define RX_THR_INT		(30)
#define	TX_END_INT		(19)
#define	RX_OV			(18)
#define	MEM_RDY_INT		(17)
#define TX_BYT_CNT_VLD		(16)
#define RECVDATA_ERR_INT	(3)
#define	RX_TIMEOUT		(2)
#define RXBUF_FULL		(1)
#define TXFIFO_EMPTY		(0)


#define UART_RX_FIFO_SIZE	32

extern void printch(char);
extern void printascii(const char *);


#if 0
#define dbg(x...)	printk(x)
#else
#define dbg(x...)	do {} while(0)
#endif

#if 0
#define dbg_irq(x...)	printk(x)
#else
#define dbg_irq(x...)	do {} while(0)
#endif

/* UART name and device definitions */
#define NR_PORTS		4

#define AK88_SERIAL_NAME	"ttySAK"
#define AK88_SERIAL_MAJOR	204
#define AK88_SERIAL_MINOR	64


#ifdef CONFIG_SERIAL_AK88_CONSOLE

static struct console ak880x_serial_console;

#define AK88_SERIAL_CONSOLE &ak880x_serial_console
#else
#define AK88_SERIAL_CONSOLE NULL
#endif

struct ak880x_uart_port {
	char			*name;
	struct uart_port	port;

	unsigned char __iomem   *rxfifo_base;
	unsigned char __iomem   *txfifo_base;

	unsigned int		rxfifo_offset;
#if	defined(CONFIG_ARCH_AK88)
	unsigned int		rxbuf_offset;
#endif
	unsigned int		nbr_to_read;
	unsigned int		timeout_cnt;


	unsigned char		claimed;
	struct clk		*clk;
};

/* macros to change one thing to another */

#define tx_enabled(port)	((port)->unused[0])
#define rx_enabled(port)	((port)->unused[1])

static int clk_asic_getrate(void)
{
	union ak880x_clk_div1_reg clk_div1_reg;
	unsigned int clkrate,asicrate;
	unsigned int asic_div;

	clk_div1_reg.regval = __raw_readl(AK88_VA_SYSCTRL+0x04);

 	clkrate = 4 * (clk_div1_reg.clk_div.m + 45) / (clk_div1_reg.clk_div.n + 1);  //default 

#ifdef CONFIG_ARCH_AK7801
	if (clk_div1_reg.clk_div.m > 0x20)
		clkrate = 4 * 62 /(clk_div1_reg.clk_div.n + 1);
	else
		clkrate = 4 * (clk_div1_reg.clk_div.m + 62) / (clk_div1_reg.clk_div.n + 1);
#endif

#ifdef CONFIG_ARCH_AK88
	clkrate = 4 * (clk_div1_reg.clk_div.m + 45) / (clk_div1_reg.clk_div.n + 1);
#endif

	asic_div = clk_div1_reg.clk_div.asic_clk ;
        asicrate = clkrate ;

	if(asic_div==0){
 		#ifdef CONFIG_BOARD_AK8801EPC
		        asicrate = clkrate /2 ;
		#else
 		        asicrate = clkrate ;
 		#endif
 	}
	else{
	   	if(asic_div==1)
			asicrate = clkrate /2;
		else
			asicrate = clkrate >> asic_div;
 	} 

	return asicrate;
}


static int uart_intevent_decode(unsigned long status,
		unsigned int maskbit, unsigned int statusbit)
{
	if ((status & 1<<maskbit) && (status & 1<<statusbit))
		return 1;
	else
		return 0;
}

static inline void uart_subint_disable(struct ak880x_uart_port *ourport, unsigned long mask)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
	uart_reg &= ~mask;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
}

static inline void uart_subint_enable(struct ak880x_uart_port *ourport, unsigned long unmask)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
	uart_reg |= unmask;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
}

static void uart_subint_clear(struct ak880x_uart_port *ourport, unsigned int subint)
{
	unsigned long uart_reg;

	switch (subint) {
	case TX_THR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RX_THR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RECVDATA_ERR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RXBUF_FULL:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RX_TIMEOUT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
#if	defined(CONFIG_ARCH_AK88)
		uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
		uart_reg |= (1<<23);
		__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);
#endif
		break;

	default:
		printk(KERN_ERR "ak880x-uart: unkown subint type: %d\n", subint);
		break;
	}

	return;
}

static inline void uart_clr_rx_status(struct ak880x_uart_port *ourport)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(ourport->port.membase + UART_CONF1);

#define RX_STATUS_CLR	(1<<29)
	uart_reg |= RX_STATUS_CLR;

	__raw_writel(uart_reg, ourport->port.membase + UART_CONF1);
}

static inline void uart_clr_tx_status(struct ak880x_uart_port *ourport)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(ourport->port.membase + UART_CONF1);

#define TX_STATUS_CLR	(1<<28)
	uart_reg |= TX_STATUS_CLR;

	__raw_writel(uart_reg, ourport->port.membase + UART_CONF1);
}

static inline struct ak880x_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct ak880x_uart_port, port);
}

/*
 * forcibly clear a uart buffer status
 */
static inline void clear_uart_txbuf_status(struct ak880x_uart_port *ourport)
{
	unsigned long regval;
	local_irq_disable();
	regval = __raw_readl(AK88_VA_L2CTRL + 0x8C);

	regval |= (0x1 << (16 + ourport->port.line * 2));

	__raw_writel(regval,  AK88_VA_L2CTRL + 0x8C);
	local_irq_enable();
}

static inline void clear_uart_rxbuf_status(struct ak880x_uart_port *ourport)
{
	unsigned long regval;

	/* regval = __raw_readl(AK88_VA_L2CTRL + 0x8C);

	regval |= (0x1 << (17 + ourport->port.line * 2));

	__raw_writel(regval,  AK88_VA_L2CTRL + 0x8C); */
}

/* static inline void ak880x_uart_putchar(struct ak880x_uart_port *ourport, int ch) */
static inline void ak880x_uart_putchar(struct ak880x_uart_port *ourport, unsigned char ch)
{
	unsigned long regval;

	clear_uart_txbuf_status(ourport);

	/* __raw_writel((unsigned long)ch, ourport->txfifo_base); */
	__raw_writel(ch, ourport->txfifo_base);
	__raw_writel(0x0, ourport->txfifo_base + 0x3C);	/*to inform the buf is full*/

	regval = __raw_readl(ourport->port.membase + UART_CONF1);
	__raw_writel(regval | 0x1<<28, ourport->port.membase + UART_CONF1); /* clear the tx count reg */

	regval = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	__raw_writel(regval | 0x1<<11, ourport->port.membase + BUF_THRESHOLD); /*clear the tx count reg*/

	regval = __raw_readl(ourport->port.membase + UART_CONF2);
	__raw_writel(regval | (0x1<<4 | 0x1<<16), ourport->port.membase + UART_CONF2);

	while ( (__raw_readl(ourport->port.membase + DATA_CONF) & 0x1fff) != 0)
	{
//		if (ourport->port.line == 3)
//			/* mdelay(1); */
//			dbg("port %d waiting: 0x%x\n", ourport->port.line, (__raw_readl(ourport->port.membase + DATA_CONF) & 0x1fff));
	}
}

static inline int uart_hwport_init(struct ak880x_uart_port *ourport)
{
	unsigned int regval = 0;

	/* set share pin to UARTn, and disable pull-up */
	switch (ourport->port.line) {
	case 0:
		/* set share pin */
		*(volatile unsigned long*)(AK88_SHAREPIN_CON1) |= (1<<9);

		/* enable ppu function */
		*(volatile unsigned long*)(AK88_PPU_PPD1) &= ~(0x3<<14);
		rL2_CONBUF8_15 |= 3<<16;
		rL2_FRACDMAADDR |= 1<<29;
		break;

	case 1:
		AK88_GPIO_UART1(1);
		AK88_GPIO_UART1_FLOW(1);
		ak880x_gpio_pullup(AK88_GPIO_16, 1);
		ak880x_gpio_pullup(AK88_GPIO_17, 1);
		ak880x_gpio_pullup(AK88_GPIO_18, 1);
		ak880x_gpio_pullup(AK88_GPIO_19, 1);

		regval = __raw_readl(AK88_VA_SYSCTRL+0xD4);
		regval &= ~((0xF<<14) | (0xF<<2));
		regval |= (0x5<<2 | 0xF<<14);
		__raw_writel(regval, AK88_VA_SYSCTRL+0xD4);
		rL2_CONBUF8_15 |= 3<<18;
		rL2_FRACDMAADDR |= 1<<29;
		break;

	case 2:
		AK88_GPIO_UART2(1);
		AK88_GPIO_UART2_FLOW(0);
		ak880x_gpio_pullup(AK88_GPIO_20, 1);
		ak880x_gpio_pullup(AK88_GPIO_21, 1);
		ak880x_gpio_pullup(AK88_GPIO_22, 1);
		ak880x_gpio_pullup(AK88_GPIO_23, 1);

		regval = __raw_readl(AK88_VA_SYSCTRL+0xD4);
		regval &= ~((0xF<<18) | (0xF<<6));
		regval |= (0x5<<6 | 0xF<<18);
		__raw_writel(regval, AK88_VA_SYSCTRL+0xD4);
		rL2_CONBUF8_15 |= 3<<20;
		rL2_FRACDMAADDR |= 1<<29;
		break;

	case 3:
		AK88_GPIO_UART3(1);
		AK88_GPIO_UART3_FLOW(0);
		AK88_UART3_ENABLE();

		ak880x_gpio_pullup(AK88_GPIO_24, 1);
		ak880x_gpio_pullup(AK88_GPIO_25, 1);
		ak880x_gpio_pullup(AK88_GPIO_26, 1);
		ak880x_gpio_pullup(AK88_GPIO_27, 1);

		regval = __raw_readl(AK88_VA_SYSCTRL+0xD4);
		regval &= ~((0xF<<22) | (0xF<<10));
		regval |= (0x5<<10 | 0xF<<22);
		__raw_writel(regval, AK88_VA_SYSCTRL+0xD4);
		rL2_CONBUF8_15 |= 3<<22;
		rL2_FRACDMAADDR |= 1<<29;
		break;

	default:
		printk(KERN_ERR "unknown uart port\n");
		return -1;
		break;
	}

	return 0;
}

static int uart_enable_clock(struct ak880x_uart_port *ourport, int enable)
{
	unsigned long regval;

	regval = __raw_readl(AK88_VA_SYSCTRL + 0x0C);

	switch (ourport->port.line) {
	case 0:
		if (enable)
			regval &= ~(1<<15);
		/* else
			regval |= (1<<15); */
		break;

	case 1:
		if (enable)
			regval &= ~(1<<2);
		else
			/* shared with SPI SD clock */
			;
			/* regval |= (1<<2); */
		break;

	case 2:
	case 3:
		if (enable)
			regval &= ~(1<<8);
		else
			regval |= (1<<8);
		break;

	default:
		printk(KERN_ERR "unknown uart port\n");
		return -1;
		break;
	}

	__raw_writel(regval, AK88_VA_SYSCTRL + 0x0C);

	return 0;
}

/* power power management control */
static void ak880x_serial_pm(struct uart_port *port, unsigned int level,
		unsigned int old)
{
	switch (level) {
	case 3:	/* disable */
	//	dbg("%s: enterring pm level: %d\n", __FUNCTION__, level);
		break;

	case 0:	/* enable */
	//	dbg("%s: enterring pm level: %d\n", __FUNCTION__, level);
		break;

	default:
		printk(KERN_ERR "ak880x_serial: unknown pm %d\n", level);
		return ;
	}

}

static unsigned int ak880x_serial_tx_empty(struct uart_port *port)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(port->membase + UART_CONF2);

	if (uart_reg & 1<<TXFIFO_EMPTY)
		return 1;

	return 0;
}

/* no modem control lines */
static unsigned int ak880x_serial_get_mctrl(struct uart_port *port)
{
	/* FIXME */
	dbg("%s\n", __FUNCTION__);
	return 0;
}

static void ak880x_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* todo - possibly remove AFC and do manual CTS */
	dbg("%s\n", __FUNCTION__);
}

static void ak880x_serial_stop_tx(struct uart_port *port)
{
	struct ak880x_uart_port *ourport = to_ourport(port);

	/* dbg("%s\n", __FUNCTION__); */

	if (tx_enabled(port)) {
		uart_subint_disable(ourport, 1<<TX_END_INT_ENABLE);
		tx_enabled(port) = 0;
	}
}

static void ak880x_serial_start_tx(struct uart_port *port)
{
	struct ak880x_uart_port *ourport = to_ourport(port);

	/* dbg("%s\n", __FUNCTION__); */

	if (!tx_enabled(port)) {
		uart_subint_enable(ourport, 1<<TX_END_INT_ENABLE);
		tx_enabled(port) = 1;
	}
}

static void ak880x_serial_stop_rx(struct uart_port *port)
{
	dbg("%s\n", __FUNCTION__);
}

static void ak880x_serial_enable_ms(struct uart_port *port)
{
	dbg("%s\n", __FUNCTION__);
}

static void ak880x_serial_break_ctl(struct uart_port *port, int break_state)
{
	dbg("%s\n", __FUNCTION__);
}

static irqreturn_t ak880x_uart_irqhandler(int irq, void *dev_id)
{
	struct ak880x_uart_port	*ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;	//&port->info->xmit;
	struct tty_struct *tty = port->state->port.tty;	//port->info->tty;
	unsigned int flag= TTY_NORMAL;

	unsigned char __iomem   *pbuf;
	unsigned char *pxmitbuf;
	unsigned long uart_status;
	unsigned int rxcount = 0;
	unsigned char ch = 0;
	unsigned int i;
	int txcount , tx_tail;
	unsigned int l2_offset = 0;
	unsigned long regval;

	uart_status = __raw_readl(ourport->port.membase + UART_CONF2);

	/* clear error */
	if ( uart_intevent_decode(uart_status, RECVDATA_ERR_INT_ENABLE, RECVDATA_ERR_INT) )
	{
		dbg_irq("ak880x-uart: error occurs in received data\n");
		uart_subint_clear(ourport, RECVDATA_ERR_INT);
//		return IRQ_HANDLED;
	}

	if ( uart_intevent_decode(uart_status, TX_END_INT_ENABLE, TX_END_INT) )
	{
		/* if there isnt anything more to transmit, or the uart is now
		 * stopped, disable the uart and exit
		 */
		if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		{
			ak880x_serial_stop_tx(port);
			goto ak880x_rx_irq;
		}

		txcount = uart_circ_chars_pending(xmit);

		if(txcount > 32)
			txcount = 32;
		pbuf = ourport->txfifo_base;
		pxmitbuf = xmit->buf;
//		clear_uart_txbuf_status(ourport);

		regval = __raw_readl(ourport->port.membase + UART_CONF1);
		__raw_writel(regval | 0x1<<28, ourport->port.membase + UART_CONF1); /* clear the tx count reg */

		__raw_writel(0x0, ourport->txfifo_base + 0x3C);	/*to inform the buf is full*/
		l2_offset = 0;
		tx_tail = xmit->tail;
		regval = 0;
		for(i = 0; i < txcount; i++)
		{
			regval |= pxmitbuf[tx_tail]<<((i & 3) * 8 );
			if((i & 3) == 3)
			{
				__raw_writel(regval, pbuf + l2_offset);
				l2_offset = l2_offset + 4;
				regval = 0;
			}
			tx_tail = (tx_tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx += 1;
		}
		if(i & 3)
		{
			__raw_writel(regval, pbuf + l2_offset);
		}

		regval = (__raw_readl(ourport->port.membase + UART_CONF2) & 0xFFFE000F) | (txcount<< 4)  | (0x1<<16);
		__raw_writel(regval, ourport->port.membase + UART_CONF2);
		xmit->tail = tx_tail;

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);

		if (uart_circ_empty(xmit))
			ak880x_serial_stop_tx(port);
	}
ak880x_rx_irq:


	if ( uart_intevent_decode(uart_status, RXBUF_FULL_INT_ENABLE, RXBUF_FULL))
	{
		printk("%s: overflow\n", __func__);

		/* FIXME: 1st, Read Rx Buf */

		/* 2nd, set buf flag [access 15th byte] ?
		 *
		 */
		clear_uart_rxbuf_status(ourport);

		/* 3nd, clear irq */
		uart_subint_clear(ourport, RXBUF_FULL);

		__raw_readb(ourport->rxfifo_base + 0x3C);
		__raw_readb(ourport->rxfifo_base + 0x7C);
	}

	/* rx threshold interrupt */
	if ( uart_intevent_decode(uart_status, RX_THR_INT_ENABLE, RX_THR_INT) ||
		uart_intevent_decode(uart_status, RX_TIMEOUT_INT_ENABLE, RX_TIMEOUT))
	{
		if ( uart_intevent_decode(uart_status, RX_THR_INT_ENABLE, RX_THR_INT))
			uart_subint_clear(ourport, RX_THR_INT);
		else
			uart_subint_clear(ourport, RX_TIMEOUT);

		ourport->nbr_to_read = (__raw_readl(ourport->port.membase + DATA_CONF)>>13) & 0x1F;
		ourport->timeout_cnt = (__raw_readl(ourport->port.membase + DATA_CONF)>>23) & 0x3;


		if (ourport->nbr_to_read  != ourport->rxfifo_offset)
		{
			l2_offset = (ourport->rxfifo_offset<<2) + ourport->rxbuf_offset;
			pbuf = ourport->rxfifo_base + l2_offset;
			if(l2_offset <= (ourport->nbr_to_read<<2))
			{
				rxcount = (ourport->nbr_to_read<<2) - l2_offset;
				for (i=0; i<rxcount; i++)
				{
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}
			}
			else
			{
				rxcount = 128 - l2_offset;

				for (i=0; i<rxcount; i++)
				{
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}

				rxcount = ourport->nbr_to_read<<2;

				pbuf = ourport->rxfifo_base;
				for (i=0; i<rxcount; i++)
				{
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}

			}
			ourport->rxbuf_offset = 0;
			ourport->rxfifo_offset = ourport->nbr_to_read;
		}


		if(ourport->timeout_cnt != ourport->rxbuf_offset)
		{
			for (i = ourport->rxbuf_offset; i<ourport->timeout_cnt; i++)
			{
				ch = (__raw_readl(ourport->port.membase + UART_RXBUF) >> i*8) & 0xFF;
				uart_insert_char(port, 0, 0, ch, flag);
			}
			ourport->rxbuf_offset = (ourport->timeout_cnt);
		}

	        tty_flip_buffer_push(tty);
	}

	return IRQ_HANDLED;

}

static void ak880x_serial_shutdown(struct uart_port *port)
{
	struct ak880x_uart_port *ourport = to_ourport(port);
	unsigned int uart_reg = 0;

	/*
	 * 1st, free irq.
	 * 2nd, disable/mask hw uart setting.
	 * 3rd, close uart clock.
	 */

	uart_reg = __raw_readl(ourport->port.membase + 0xc);
	uart_reg  &= (~(1<<5));
	__raw_writel(uart_reg, ourport->port.membase + 0xc);

	uart_reg = __raw_readl(ourport->port.membase + 0x0);
	uart_reg  &= (~(1<<29));
	__raw_writel(uart_reg, ourport->port.membase + 0x0);
	uart_reg  &= (~1<<21);
	__raw_writel(uart_reg, ourport->port.membase + 0x0);

	rL2_CONBUF8_15 |= 1<<23;

	free_irq(port->irq, ourport);
	uart_enable_clock(ourport, 0);
}

/*
 * 1, setup gpio.
 * 2, enable clock.
 * 3, request irq and setting up uart control.
 * 4, enable subirq.
 */
static int ak880x_serial_startup(struct uart_port *port)
{
	struct ak880x_uart_port *ourport = to_ourport(port);
	unsigned long uart_reg;
	int ret;

	if ( rx_enabled(port) && tx_enabled(port))
		return 0;

	uart_enable_clock(ourport, 1);
	uart_hwport_init(ourport);

	uart_reg = __raw_readl(ourport->port.membase + UART_CONF1);
#if	defined(CONFIG_ARCH_AK7801)
	uart_reg |= ((1<<21) | (1<<23) | (1<<28) | (1<<29));
#elif	defined(CONFIG_ARCH_AK88)
	uart_reg |= ((1<<21) | (1<<23) | (1<<24) | (1<<28) | (1<<29));
#endif
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF1);

	__raw_writel(0, ourport->port.membase + UART_CONF2);

	/* set threshold to 4bytes */
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	uart_reg &= ~0x1F;
	/* uart_reg |= (1<<5|1<<11); */
	uart_reg |= (1<<11);

#if	defined(CONFIG_ARCH_AK88)
	__raw_writel(0, ourport->port.membase + RXBUF_THRESHOLD_EXT);
//	uart_reg |= 0x3; /* 4 Bytes */
//	__raw_writel(2, ourport->port.membase + RXBUF_THRESHOLD_EXT); //2 * 32byte
	uart_reg |= 0x1F; /* 16 Bytes */
#endif
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);

	/* ourport->rxfifo_offset = 0; */
	uart_clr_rx_status(ourport);
	clear_uart_rxbuf_status(ourport);
	ourport->rxfifo_offset = 0;
#if	defined(CONFIG_ARCH_AK88)
	ourport->rxbuf_offset = (__raw_readl(ourport->port.membase + DATA_CONF)>>23) & 0x3;
#endif

	/* clear count */
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	uart_reg |= (1<<5);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);
	udelay(10);
	uart_reg &= ~(1<<5);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);

	ret = request_irq(port->irq, ak880x_uart_irqhandler,
			IRQF_DISABLED, ourport->name, ourport);
	if (ret) {
		printk(KERN_ERR "can't request irq %d for %s\n", port->irq, ourport->name);
		goto startup_err;
	}

	uart_reg = 0;
	uart_reg |= 1<<RX_THR_INT_ENABLE|1<<RECVDATA_ERR_INT_ENABLE|1<<RXBUF_FULL_INT_ENABLE|1<<RX_TIMEOUT_INT_ENABLE| 1<<TX_END_INT_ENABLE ;
//	uart_reg |= 1<<RX_THR_INT_ENABLE|1<<RECVDATA_ERR_INT_ENABLE|1<<RXBUF_FULL_INT_ENABLE|1<<RX_TIMEOUT_INT_ENABLE;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);

	rx_enabled(port) = 1;
	tx_enabled(port) = 1;
//	tx_enabled(port) = 0;

	ourport->rxfifo_offset =0;

	return 0;

startup_err:
	ak880x_serial_shutdown(port);
	return ret;
}

static void ak880x_serial_set_termios(struct uart_port *port,
		struct ktermios *termios,
		struct ktermios *old)
{
	struct ak880x_uart_port *ourport = to_ourport(port);
	unsigned int baud;
	unsigned long flags;
	unsigned long regval;
	unsigned long asic_clk;

	asic_clk = 1000 * 1000 * clk_asic_getrate();
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 * min: 2.4bps, max: 3Mbps
	 */
	baud = uart_get_baud_rate(port, termios, old, 2400, 115200*26);
#ifdef CONFIG_BOARD_AK8802EBOOK
	if (port->line == 0)
		baud = 12800;
#endif

	spin_lock_irqsave(&port->lock, flags);

	/* baudrate setting */
	regval = ((asic_clk / baud - 1) & 0xFFFF);
	if (asic_clk % baud)
		regval |= (1<<22);

#if	defined(CONFIG_ARCH_AK7801)
	regval |= ((1<<21) | (1<<23) | (1<<28) | (1<<29));
#elif	defined(CONFIG_ARCH_AK88)
	regval |= ((1<<21) | (1<<23) | (1<<24) | (1<<28) | (1<<29));
#endif

	ourport->rxfifo_offset = 0;

	/* flow control setting */
	if(port->line != 0)
	{
		if((termios->c_cflag & CRTSCTS))
		{
			switch (port->line) {
			case 1:
				AK88_GPIO_UART1_FLOW(1);
				break;
			case 2:
				AK88_GPIO_UART2_FLOW(1);
				break;
			case 3:
				AK88_GPIO_UART3_FLOW(1);
				break;
			}
			regval &= ~(1<<18|1<<19);
		}
		else
		{
			switch(port->line) {
			case 1:
				AK88_GPIO_UART1_FLOW(0);
				break;
			case 2:
				AK88_GPIO_UART2_FLOW(0);
				break;
			case 3:
				AK88_GPIO_UART3_FLOW(0);
				break;
			}
			regval |= (1<<18|1<<19);
		}
	}

	/* parity setting */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			regval |= (0x2<<25);
		else
			regval |= (0x3<<25);
	}
#ifdef CONFIG_BOARD_AK8802EBOOK
	if (port->line == 0)
		regval |= (0x2<<25); /* ps/2 protol employ odd parity */
#endif


	__raw_writel(regval, port->membase + UART_CONF1);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ak880x_serial_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_AK88:
		return "AK88";
	default:
		return NULL;
	}
}

static void ak880x_serial_release_port(struct uart_port *port)
{
	dbg("%s\n", __FUNCTION__);
}

static int ak880x_serial_request_port(struct uart_port *port)
{
	dbg("%s\n", __FUNCTION__);
	return 0;
}

static void ak880x_serial_config_port(struct uart_port *port, int flags)
{
	struct ak880x_uart_port *ourport = to_ourport(port);

	port->type = PORT_AK88;
	ourport->rxfifo_offset = 0;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
ak880x_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	dbg("%s\n", __FUNCTION__);

	return 0;
}


static struct uart_ops ak880x_serial_ops = {
	.pm             = ak880x_serial_pm,
	.tx_empty       = ak880x_serial_tx_empty,
	.get_mctrl      = ak880x_serial_get_mctrl,
	.set_mctrl      = ak880x_serial_set_mctrl,
	.stop_tx        = ak880x_serial_stop_tx,
	.start_tx       = ak880x_serial_start_tx,
	.stop_rx        = ak880x_serial_stop_rx,
	.enable_ms      = ak880x_serial_enable_ms,
	.break_ctl      = ak880x_serial_break_ctl,
	.startup        = ak880x_serial_startup,
	.shutdown       = ak880x_serial_shutdown,
	.set_termios    = ak880x_serial_set_termios,
	.type           = ak880x_serial_type,
	.release_port   = ak880x_serial_release_port,
	.request_port   = ak880x_serial_request_port,
	.config_port    = ak880x_serial_config_port,
	.verify_port    = ak880x_serial_verify_port,
};


static struct ak880x_uart_port ak880x_serial_ports[NR_PORTS] = {
	[0] = {
		.name = "uart0",
		.rxfifo_base	= AK88_UART0_RXBUF_BASE,
		.txfifo_base	= AK88_UART0_TXBUF_BASE,
		.port = {
			.lock		= __SPIN_LOCK_UNLOCKED(ak880x_serial_ports[0].port.lock),
			.iotype		= UPIO_MEM,
			.mapbase	= AK88_UART0_PA_BASE,
			.membase	= AK88_UART0_BASE,
			.irq		= IRQ_UART0,
			.uartclk        = 0,
			.fifosize       = 64,
			.ops            = &ak880x_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line           = 0,
		},
	},
	[1] = {
		.name = "uart1",
		.rxfifo_base	= AK88_UART1_RXBUF_BASE,
		.txfifo_base	= AK88_UART1_TXBUF_BASE,
		.port = {
			.lock		= __SPIN_LOCK_UNLOCKED(ak880x_serial_ports[1].port.lock),
			.iotype		= UPIO_MEM,
			.mapbase	= AK88_UART1_PA_BASE,
			.membase	= AK88_UART1_BASE,
			.irq		= IRQ_UART1,
			.uartclk	= 0,
			.fifosize	= 64,
			.ops		= &ak880x_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 1,
		},
	},
	[2] = {
		.name = "uart2",
		.rxfifo_base	= AK88_UART2_RXBUF_BASE,
		.txfifo_base	= AK88_UART2_TXBUF_BASE,
		.port = {
			.lock		= __SPIN_LOCK_UNLOCKED(ak880x_serial_ports[2].port.lock),
			.iotype		= UPIO_MEM,
			.irq		= IRQ_UART2,
			.mapbase	= AK88_UART2_PA_BASE,
			.membase	= AK88_UART2_BASE,
			.uartclk	= 0,
			.fifosize	= 64,
			.ops		= &ak880x_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 2,
		},
	},
	[3] = {
		.name = "uart3",
		.rxfifo_base	= AK88_UART3_RXBUF_BASE,
		.txfifo_base	= AK88_UART3_TXBUF_BASE,
		.port = {
			.lock		= __SPIN_LOCK_UNLOCKED(ak880x_serial_ports[3].port.lock),
			.iotype		= UPIO_MEM,
			.irq		= IRQ_UART3,
			.mapbase	= AK88_UART3_PA_BASE,
			.membase	= AK88_UART3_BASE,
			.uartclk	= 0,
			.fifosize	= 64,
			.ops		= &ak880x_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 3,
		},
	}
};



static struct uart_driver ak880x_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= AK88_SERIAL_NAME,
	.nr		= NR_PORTS,
	.cons		= AK88_SERIAL_CONSOLE,
	.driver_name	= AK88_SERIAL_NAME,
	.major		= AK88_SERIAL_MAJOR,
	.minor		= AK88_SERIAL_MINOR,
};

/* ak880x_serial_init_port
 *
 * initialise a single serial port from the platform device given
 */
static int ak880x_serial_init_port(struct ak880x_uart_port *ourport,
		struct platform_device *platdev)
{
	struct uart_port *port = &ourport->port;

	if (platdev == NULL)
		return -ENODEV;

	/* setup info for port */
	port->dev = &platdev->dev;

	ourport->clk = clk_get(port->dev, "asic_clk");

	return 0;
}



static int ak880x_serial_probe(struct platform_device *dev)
{
	struct ak880x_uart_port *ourport;
	int ret = 0;

	ourport = &ak880x_serial_ports[dev->id];

	dbg(KERN_INFO "%s: initialising port %s...\n", __FUNCTION__, ourport->name);

	ret = ak880x_serial_init_port(ourport, dev);
	if (ret < 0)
		goto probe_err;

	dbg("%s: adding port\n", __FUNCTION__);
	uart_add_one_port(&ak880x_uart_drv, &ourport->port);
	platform_set_drvdata(dev, &ourport->port);

	return 0;

probe_err:
	return ret;
}

static int ak880x_serial_remove(struct platform_device *dev)
{
	return 0;
}


static struct platform_driver ak880x_serial_drv = {
	.probe          = ak880x_serial_probe,
	.remove         = ak880x_serial_remove,
//	.suspend        = ak880x_serial_suspend,
//	.resume         = ak880x_serial_resume,
	.driver         = {
		.name   = "ak880x-uart",
		.owner  = THIS_MODULE,
	},
};



/* module initialisation code */

static int __init ak880x_serial_modinit(void)
{
	int ret;

	printk("AK88 UART Driver, (c) 2010 ANYKA\n");

	dbg("Enterring %s\n", __FUNCTION__);

	ret = uart_register_driver(&ak880x_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	platform_driver_register(&ak880x_serial_drv);

	return 0;
}

static void __exit ak880x_serial_modexit(void)
{
	platform_driver_unregister(&ak880x_serial_drv);
	uart_unregister_driver(&ak880x_uart_drv);
}

module_init(ak880x_serial_modinit);
module_exit(ak880x_serial_modexit);

#ifdef CONFIG_SERIAL_AK88_CONSOLE

/* Console code */

static struct uart_port *cons_uart;

/*
static int
ak880x_serial_console_txrdy(struct uart_port *port)
{
	//TODO:
}
*/

static void
ak880x_serial_console_putchar(struct uart_port *port, int ch)
{
	struct ak880x_uart_port *ourport = to_ourport(port);

	/*
	 * printch(ch);
	 */
	ak880x_uart_putchar(ourport, ch);
}

static void
ak880x_serial_console_write(struct console *co, const char *s,
			     unsigned int count)
{
	uart_console_write(cons_uart, s, count, ak880x_serial_console_putchar);
}

static void __init
ak880x_serial_get_options(struct uart_port *port, int *baud,
			   int *parity, int *bits)
{
#if 0
	unsigned long regval;
	struct clk *clk;

	*bits	= 8;

	regval = __raw_readl(port->membase + UART_CONF1);

	if (regval & 0x1<<26) {
		if (regval & 0x1<<25)
			*parity = 'e';
		else
			*parity = 'o';
	}
	else
		*parity = 'n';

	clk = clk_get(port->dev, "asic_clk");
	if (!IS_ERR(clk) && clk != NULL)
		*baud = clk_get_rate(clk) / ((regval & 0xFFFF) + 1);

	printk("calculated baudrate: %d\n", *baud);
#endif
}

static int __init
ak880x_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	dbg("ak880x_serial_console_setup: co=%p (%d), %s\n", co, co->index, options);

	port = &ak880x_serial_ports[co->index].port;

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	dbg("ak880x_serial_console_setup: port=%p (%d)\n", port, co->index);

	cons_uart = port;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		ak880x_serial_get_options(port, &baud, &parity, &bits);

	dbg("ak880x_serial_console_setup: baud %d\n", baud);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console ak880x_serial_console =
{
	.name		= AK88_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= ak880x_serial_console_write,
	.setup		= ak880x_serial_console_setup
};


/* ak880x_serial_initconsole
 *
 * initialise the console from one of the uart drivers
*/
static int ak880x_serial_initconsole(void)
{
	dbg("ak880x_serial_initconsole\n");

	ak880x_serial_console.data = &ak880x_uart_drv;

	register_console(&ak880x_serial_console);

	return 0;
}

console_initcall(ak880x_serial_initconsole);

#endif /* CONFIG_SERIAL_AK88_CONSOLE */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("anyka");
MODULE_DESCRIPTION("Anyka 880x Serial port driver");
