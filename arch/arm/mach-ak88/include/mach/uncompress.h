/*
 * linux/arch/arm/mach-ak8801/include/mach/uncompress.h
 *
 */

#define CONFIG_AK88_LL_DEBUG_UART3
#define BAUD_RATE  115200

#define REG32(addr)             (*(volatile unsigned long*)(addr))

/* L2 buffer register */
#define L2BUF_CONF2_REG         REG32(0x2002c08c)

#define UART0_TXBUF_CLR_BIT     16
#define UART0_RXBUF_CLR_BIT     17
#define UART1_TXBUF_CLR_BIT     18
#define UART1_RXBUF_CLR_BIT     19
#define UART2_TXBUF_CLR_BIT     20
#define UART2_RXBUF_CLR_BIT     21
#define UART3_TXBUF_CLR_BIT     22
#define UART3_RXBUF_CLR_BIT     23

/* L2 buffer address */
#define L2BUF(addr)             (*(volatile unsigned long*)(addr))

#define UART0_TXBUF_ADDR        0x48001000
#define UART0_RXBUF_ADDR        0x48001080
#define UART1_TXBUF_ADDR        0x48001100
#define UART1_RXBUF_ADDR        0x48001180
#define UART2_TXBUF_ADDR        0x48001200
#define UART2_RXBUF_ADDR        0x48001280
#define UART3_TXBUF_ADDR        0x48001300
#define UART3_RXBUF_ADDR        0x48001380

#define ENDDING_OFFSET1         60
#define ENDDING_OFFSET2         124

/* Shared pin control reigsters */
#define SRDPIN_CTRL1_REG        REG32(0x08000078)
#define SRDPIN_CTRL2_REG        REG32(0x08000074)

#define SRDPIN_UART0_RXTX_BIT   9
#define SRDPIN_UART1_RXTX_BIT   10
#define SRDPIN_UART1_RTSCTS_BIT 11
#define SRDPIN_UART2_RXTX_BIT   12
#define SRDPIN_UART2_RTSCTS_BIT 13
#define SRDPIN_UART3_RXTX_BIT   14
#define SRDPIN_UART3_RTSCTS_BIT 15

#define SRDPIN_UART3_SDIO_BIT   1

/* Pin configure registers */
#define PPU_PPD1_REG            REG32(0x0800009c)
/* pullup control: 0 - pulldown; 1 - pullup */
#define CTS3_PU_BIT             27
#define RTS3_PU_BIT             26
#define URD3_PU_BIT             25
#define UTD3_PU_BIT             24
#define CTS2_PU_BIT             23
#define RTS2_PU_BIT             22
#define URD2_PU_BIT             21
#define UTD2_PU_BIT             20
#define CTS1_PU_BIT             19
#define RTS1_PU_BIT             18
#define URD1_PU_BIT             17
#define UTD1_PU_BIT             16
#define URD0_PU_BIT             15
#define UTD0_PU_BIT             14

#define GPIO_CTRL1_REG          REG32(0x080000d4)
/* pullup/pulldown enable */
#define CTS3_PE_BIT             24
#define RTS3_PE_BIT             25
#define URD3_PE_BIT             22
#define UTD3_PE_BIT             23
#define CTS2_PE_BIT             20
#define RTS2_PE_BIT             21
#define URD2_PE_BIT             18
#define UTD2_PE_BIT             19
#define CTS1_PE_BIT             16
#define RTS1_PE_BIT             17
#define URD1_PE_BIT             14
#define UTD1_PE_BIT             15
/* input enable */
#define CTS3_IE_BIT             12
#define RTS3_IE_BIT             13
#define URD3_IE_BIT             10
#define UTD3_IE_BIT             11
#define CTS2_IE_BIT             8
#define RTS2_IE_BIT             9
#define URD2_IE_BIT             6
#define UTD2_IE_BIT             7
#define CTS1_IE_BIT             4
#define RTS1_IE_BIT             5
#define URD1_IE_BIT             2
#define UTD1_IE_BIT             3

/* Clock control register */
#define CLK_CTRL_REG            REG32(0x0800000c)

#define UART0_ENABLE_BIT        15
#define UART1_ENABLE_BIT        2
#define UART2_ENABLE_BIT        8
#define UART3_ENABLE_BIT        8

/* Clock divider register */
#define CLK_DIV_REG             REG32(0x08000004)

/* UART registers */
#define UART0_BASE_ADDR_PHYS	0x20026000
#define UART1_BASE_ADDR_PHYS	0x20027000
#define UART2_BASE_ADDR_PHYS	0x20028000
#define UART3_BASE_ADDR_PHYS	0x20029000

#define UART0_CONF1_REG		REG32(UART0_BASE_ADDR_PHYS + 0x0)
#define UART0_CONF2_REG		REG32(UART0_BASE_ADDR_PHYS + 0x4)
#define UART0_DATA_CONF_REG	REG32(UART0_BASE_ADDR_PHYS + 0x8)
#define UART0_BUF_THRE_REG	REG32(UART0_BASE_ADDR_PHYS + 0xc)

#define UART1_CONF1_REG		REG32(UART1_BASE_ADDR_PHYS + 0x0)
#define UART1_CONF2_REG		REG32(UART1_BASE_ADDR_PHYS + 0x4)
#define UART1_DATA_CONF_REG	REG32(UART1_BASE_ADDR_PHYS + 0x8)
#define UART1_BUF_THRE_REG	REG32(UART1_BASE_ADDR_PHYS + 0xc)

#define UART2_CONF1_REG		REG32(UART2_BASE_ADDR_PHYS + 0x0)
#define UART2_CONF2_REG		REG32(UART2_BASE_ADDR_PHYS + 0x4)
#define UART2_DATA_CONF_REG	REG32(UART2_BASE_ADDR_PHYS + 0x8)
#define UART2_BUF_THRE_REG	REG32(UART2_BASE_ADDR_PHYS + 0xc)

#define UART3_CONF1_REG		REG32(UART3_BASE_ADDR_PHYS + 0x0)
#define UART3_CONF2_REG		REG32(UART3_BASE_ADDR_PHYS + 0x4)
#define UART3_DATA_CONF_REG	REG32(UART3_BASE_ADDR_PHYS + 0x8)
#define UART3_BUF_THRE_REG	REG32(UART3_BASE_ADDR_PHYS + 0xc)

/* bit define of UARTx_CONF1_REG */
#define BAUD_RATE_DIV_BIT       0
#define CTS_SEL_BIT             18
#define RTS_SEL_BIT             19
#define PORT_ENABLE_BIT         21
#define TX_STATUS_CLR_BIT       28
#define RX_STATUS_CLR_BIT       29

/* bit define of UARTx_CONF2_REG */
#define TX_COUNT_BIT            4
#define TX_COUNT_VALID_BIT      16
#define TX_END_BIT              19
#define TX_END_MASK             (1 << TX_END_BIT)


#if defined CONFIG_AK88_LL_DEBUG_UART3
#define UART_TXBUF_CLR_BIT      UART3_TXBUF_CLR_BIT
#define SRDPIN_UART_RXTX_BIT    SRDPIN_UART3_RXTX_BIT
#define URD_PU_BIT              URD3_PU_BIT
#define UTD_PU_BIT              UTD3_PU_BIT
#define URD_PE_BIT              URD3_PE_BIT
#define UTD_PE_BIT              UTD3_PE_BIT
#define URD_IE_BIT              URD3_IE_BIT
#define UTD_IE_BIT              UTD3_IE_BIT
#define UART_ENABLE_BIT         UART3_ENABLE_BIT
#define UART_TXBUF_ADDR         UART3_TXBUF_ADDR
#define UART_CONF1_REG          UART3_CONF1_REG
#define UART_CONF2_REG          UART3_CONF2_REG
#define UART_DATA_CONF_REG      UART3_DATA_CONF_REG
#elif defined CONFIG_AK88_LL_DEBUG_UART2
#define UART_TXBUF_CLR_BIT      UART2_TXBUF_CLR_BIT
#define SRDPIN_UART_RXTX_BIT    SRDPIN_UART2_RXTX_BIT
#define URD_PU_BIT              URD2_PU_BIT
#define UTD_PU_BIT              UTD2_PU_BIT
#define URD_PE_BIT              URD2_PE_BIT
#define UTD_PE_BIT              UTD2_PE_BIT
#define URD_IE_BIT              URD2_IE_BIT
#define UTD_IE_BIT              UTD2_IE_BIT
#define UART_ENABLE_BIT         UART2_ENABLE_BIT
#define UART_TXBUF_ADDR         UART2_TXBUF_ADDR
#define UART_CONF1_REG          UART2_CONF1_REG
#define UART_CONF2_REG          UART2_CONF2_REG
#define UART_DATA_CONF_REG      UART2_DATA_CONF_REG
#elif defined CONFIG_AK88_LL_DEBUG_UART1
#define UART_TXBUF_CLR_BIT      UART1_TXBUF_CLR_BIT
#define SRDPIN_UART_RXTX_BIT    SRDPIN_UART1_RXTX_BIT
#define URD_PU_BIT              URD1_PU_BIT
#define UTD_PU_BIT              UTD1_PU_BIT
#define URD_PE_BIT              URD1_PE_BIT
#define UTD_PE_BIT              UTD1_PE_BIT
#define URD_IE_BIT              URD1_IE_BIT
#define UTD_IE_BIT              UTD1_IE_BIT
#define UART_ENABLE_BIT         UART1_ENABLE_BIT
#define UART_TXBUF_ADDR         UART1_TXBUF_ADDR
#define UART_CONF1_REG          UART1_CONF1_REG
#define UART_CONF2_REG          UART1_CONF2_REG
#define UART_DATA_CONF_REG      UART1_DATA_CONF_REG
#else  /* default use UART0 */
#define UART_TXBUF_CLR_BIT      UART0_TXBUF_CLR_BIT
#define SRDPIN_UART_RXTX_BIT    SRDPIN_UART0_RXTX_BIT
#define URD_PU_BIT              URD0_PU_BIT
#define UTD_PU_BIT              UTD0_PU_BIT
#define UART_ENABLE_BIT         UART0_ENABLE_BIT
#define UART_TXBUF_ADDR         UART0_TXBUF_ADDR
#define UART_CONF1_REG          UART0_CONF1_REG
#define UART_CONF2_REG          UART0_CONF2_REG
#define UART_DATA_CONF_REG      UART0_DATA_CONF_REG
#endif

static inline void flush(void)
{
}

static unsigned int __uidiv__(unsigned int num, unsigned int den)
{
	unsigned int i;

	if (den == 1)
		return num;

	i = 1;
	while (den * i < num)
		i++;

	return i-1;
}

static void uart_init(void)
{
	int pll1_clk, asic_clk, clk_div;

	/* Enable UART port */
	CLK_CTRL_REG &= ~(0x1 << UART_ENABLE_BIT);

	/* Set shared pins to UART */
	SRDPIN_CTRL1_REG  |= (0x1 << SRDPIN_UART_RXTX_BIT);
#ifdef CONFIG_AK88_LL_DEBUG_UART3
	SRDPIN_CTRL2_REG  |= (0x1 << SRDPIN_UART3_SDIO_BIT);
#endif

	/* Set pin config */
	PPU_PPD1_REG |= (0x1 << URD_PU_BIT) | (0x1 << UTD_PU_BIT);
#ifndef CONFIG_AK88_LL_DEBUG_UART0
	GPIO_CTRL1_REG |= (0x1 << URD_PE_BIT) | (0x1 << UTD_PE_BIT);
	GPIO_CTRL1_REG &= ~(0x1 << UTD_IE_BIT);
	GPIO_CTRL1_REG |= (0x1 << URD_IE_BIT);
#endif

	/* Set baud rate */
	pll1_clk = __uidiv__(4 * ((CLK_DIV_REG&0x3f) + 45), (((CLK_DIV_REG>>17)&0xf) + 1));
	asic_clk = (pll1_clk >> ((CLK_DIV_REG>>6)&0x7)) * 1000 * 1000;
	clk_div = __uidiv__(asic_clk, BAUD_RATE) - 1;
	UART_CONF1_REG = (0x1 << TX_STATUS_CLR_BIT)
		| (0x1 << RX_STATUS_CLR_BIT)
		| clk_div;

#ifndef CONFIG_AK88_LL_DEBUG_UART0
	/* Disable flow control */
	UART_CONF1_REG |= (0x1 << CTS_SEL_BIT) | (0x1 << RTS_SEL_BIT);
#endif

	/* go */
	UART_CONF1_REG |= (0x1 << PORT_ENABLE_BIT);
}

/* print a char to uart */
static void putc(char c)
{
	/* Clear uart tx buffer */
	L2BUF_CONF2_REG   |= (0x1 << UART_TXBUF_CLR_BIT);
	/* write char to uart buffer */
	L2BUF(UART_TXBUF_ADDR) = (unsigned long)c;
	L2BUF(UART_TXBUF_ADDR+ENDDING_OFFSET1) = (unsigned long)'\0';
	/* Clear uart tx count register */
	UART_CONF1_REG |= (0x1 << TX_STATUS_CLR_BIT);
	/* Send buffer */
	UART_CONF2_REG |= (1 << TX_COUNT_BIT) | (0x1 << TX_COUNT_VALID_BIT);

	/* Wait for finish */
	while((UART_CONF2_REG & TX_END_MASK) == 0) {
	}
}

static inline void arch_decomp_setup(void)
{
	uart_init();
}

/*
 * nothing to do
 */
#define arch_decomp_wdog()
