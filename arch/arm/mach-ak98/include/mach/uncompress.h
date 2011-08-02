/*
 * linux/arch/arm/mach-ak98/include/mach/uncompress.h
 *
 */
#ifndef __UNCOMPRESS_H_
#define __UNCOMPRESS_H_

#include <asm/sizes.h>

#include <mach/map.h>
#include <mach/regs-uart.h>

#define L2_CACHE_CFG_REG	REG_PA_VAL(AK98_PA_L2CACH, 0x00)
#define L2_CACHE_SECTION0_START	REG_PA_VAL(AK98_PA_L2CACH, 0x10)
#define L2_CACHE_SECTION0_END	REG_PA_VAL(AK98_PA_L2CACH, 0x14)

#define CONFIG_ANYKA_LL_DEBUG_UART0
#define BAUD_RATE  115200

/* L2 buffer register */
#define L2BUF_CONF2_REG         REG_PA_VAL(AK98_PA_L2CTRL, 0x8C) //0x2002c08c

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

#define UART0_TXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1000) //0x48001000
#define UART0_RXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1080)
#define UART1_TXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1100)
#define UART1_RXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1180)
#define UART2_TXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1200)
#define UART2_RXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1280)
#define UART3_TXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1300)
#define UART3_RXBUF_ADDR        REG_PA_ADDR(AK98_PA_L2MEM, 0x1380)

#define ENDDING_OFFSET1         60
#define ENDDING_OFFSET2         124

/* Clock divider register */
#define CLK_DIV_REG             REG_PA_VAL(AK98_PA_SYSCTRL, 0x0004) //0x08000004

/* Pin configure registers */
#define PPU_PPD1_REG           	REG_PA_VAL(AK98_PA_SYSCTRL, 0x009C) //0x0800009C
#define GPIO_CTRL1_REG         	REG_PA_VAL(AK98_PA_SYSCTRL, 0x00D4) //0x080000D4

/* pullup control: 0 - pulldown; 1 - pullup */
#define CTS3_PU_BIT             27  //0x0800009c
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
/* pullup/pulldown enable, Active High*/
#define RTS3_PE_BIT             25  //0x080000d4
#define CTS3_PE_BIT             24
#define UTD3_PE_BIT             23
#define URD3_PE_BIT             22
#define RTS2_PE_BIT             21
#define CTS2_PE_BIT             20
#define UTD2_PE_BIT             19
#define URD2_PE_BIT             18
#define RTS1_PE_BIT             17
#define CTS1_PE_BIT             16
#define UTD1_PE_BIT             15
#define URD1_PE_BIT             14
/* input enable, Active Low */
#define RTS3_IE_BIT             13  //0x080000d4
#define CTS3_IE_BIT             12
#define UTD3_IE_BIT             11
#define URD3_IE_BIT             10
#define RTS2_IE_BIT             9
#define CTS2_IE_BIT             8
#define UTD2_IE_BIT             7
#define URD2_IE_BIT             6
#define RTS1_IE_BIT             5
#define CTS1_IE_BIT             4
#define UTD1_IE_BIT             3
#define URD1_IE_BIT             2

/* Clock control register */
#define CLK_CTRL_REG1			REG_PA_VAL(AK98_PA_SYSCTRL, 0x000C) //0x0800000C
#define CLK_CTRL_REG2		 	REG_PA_VAL(AK98_PA_SYSCTRL, 0x0010) //0x08000010

#define UART0_ENABLE_BIT        12  //0x0800000C
#define UART1_ENABLE_BIT        4 	//0x08000010
#define UART2_ENABLE_BIT        5 	//0x08000010
#define UART3_ENABLE_BIT        6  	//0x08000010

/* Shared pin control reigsters */
#define SRDPIN_CTRL1_REG     	REG_PA_VAL(AK98_PA_SYSCTRL, 0x0078) //0x08000078
#define SRDPIN_CTRL2_REG     	REG_PA_VAL(AK98_PA_SYSCTRL, 0x0074) //0x08000074

#define SRDPIN_UART0_RXTX_BIT   9   //0x08000078

#define SRDPIN_UART1_RXTX_BIT   10  //0x08000078
#define SRDPIN_UART1_RTSCTS_BIT 11  //0x08000078
#define SRDPIN_UART2_RXTX_BIT   12  //0x08000078
#define SRDPIN_UART2_RTSCTS_BIT 13  //0x08000078
#define SRDPIN_UART3_RXTX_BIT   14  //0x08000078
#define SRDPIN_UART3_RTSCTS_BIT 15  //0x08000078

#define SRDPIN_UART3_SDIO_BIT   1   //0x08000074, 01: used for uart3

/** ************ UART registers *****************************/
#define UART0_CONF1_REG			REG_PA_VAL(AK98_PA_UART_REG(0), 0x00) //0x20026000
#define UART0_CONF2_REG			REG_PA_VAL(AK98_PA_UART_REG(0), 0x04)
#define UART0_DATA_CONF_REG		REG_PA_VAL(AK98_PA_UART_REG(0), 0x08)
#define UART0_BUF_THRE_REG		REG_PA_VAL(AK98_PA_UART_REG(0), 0x0C)
#define UART0_BUF_STOPBIT_REG		REG_PA_VAL(AK98_PA_UART_REG(0), 0x18)

#define UART1_CONF1_REG			REG_PA_VAL(AK98_PA_UART_REG(1), 0x00) //0x20027000
#define UART1_CONF2_REG			REG_PA_VAL(AK98_PA_UART_REG(1), 0x04)
#define UART1_DATA_CONF_REG		REG_PA_VAL(AK98_PA_UART_REG(1), 0x08)
#define UART1_BUF_THRE_REG		REG_PA_VAL(AK98_PA_UART_REG(1), 0x0C)
#define UART1_BUF_STOPBIT_REG		REG_PA_VAL(AK98_PA_UART_REG(1), 0x18)


#define UART2_CONF1_REG			REG_PA_VAL(AK98_PA_UART_REG(2), 0x00) //0x20028000
#define UART2_CONF2_REG			REG_PA_VAL(AK98_PA_UART_REG(2), 0x04)
#define UART2_DATA_CONF_REG		REG_PA_VAL(AK98_PA_UART_REG(2), 0x08)
#define UART2_BUF_THRE_REG		REG_PA_VAL(AK98_PA_UART_REG(2), 0x0C)
#define UART2_BUF_STOPBIT_REG		REG_PA_VAL(AK98_PA_UART_REG(2), 0x18)


#define UART3_CONF1_REG			REG_PA_VAL(AK98_PA_UART_REG(3), 0x00) //0x20029000
#define UART3_CONF2_REG			REG_PA_VAL(AK98_PA_UART_REG(3), 0x04)
#define UART3_DATA_CONF_REG		REG_PA_VAL(AK98_PA_UART_REG(3), 0x08)
#define UART3_BUF_THRE_REG		REG_PA_VAL(AK98_PA_UART_REG(3), 0x0C)
#define UART3_BUF_STOPBIT_REG		REG_PA_VAL(AK98_PA_UART_REG(3), 0x18)


/* bit define of UARTx_CONF1_REG */
#define BAUD_RATE_DIV_BIT       0   //baudrate value
#define CTS_SEL_BIT             18
#define RTS_SEL_BIT             19
#define PORT_ENABLE_BIT         21  //0: disable, 1:enable
#define TX_STATUS_CLR_BIT       28
#define RX_STATUS_CLR_BIT       29

/* bit define of UARTx_CONF2_REG */
#define TX_COUNT_BIT            4
#define TX_COUNT_VALID_BIT      16
#define TX_END_BIT              19
#define TX_END_MASK             (1 << TX_END_BIT)

#if defined CONFIG_ANYKA_LL_DEBUG_UART3
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
#define UART_BUF_STOPBIT_REG	UART3_BUF_STOPBIT_REG

#elif defined CONFIG_ANYKA_LL_DEBUG_UART2
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
#define UART_BUF_STOPBIT_REG	UART2_BUF_STOPBIT_REG
#elif defined CONFIG_ANYKA_LL_DEBUG_UART1
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
#define UART_BUF_STOPBIT_REG	UART1_BUF_STOPBIT_REG
#elif  defined CONFIG_ANYKA_LL_DEBUG_UART0
#define UART_TXBUF_CLR_BIT      UART0_TXBUF_CLR_BIT
#define SRDPIN_UART_RXTX_BIT    SRDPIN_UART0_RXTX_BIT
#define URD_PU_BIT              URD0_PU_BIT
#define UTD_PU_BIT              UTD0_PU_BIT
#define UART_ENABLE_BIT         UART0_ENABLE_BIT
#define UART_TXBUF_ADDR         UART0_TXBUF_ADDR
#define UART_CONF1_REG          UART0_CONF1_REG
#define UART_CONF2_REG          UART0_CONF2_REG
#define UART_DATA_CONF_REG      UART0_DATA_CONF_REG
#define UART_BUF_STOPBIT_REG	UART0_BUF_STOPBIT_REG

#else
#error One of UART0 ~ UART4 Must be defined
#endif

static inline void flush(void)
{
}

static unsigned int uidiv(unsigned int num, unsigned int den)
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
	unsigned int pll_clk, asic_clk, clk_div;
	unsigned int asic_div;

	/* enable uart clock control */
#ifdef CONFIG_ANYKA_LL_DEBUG_UART0
	CLK_CTRL_REG1 &= ~(0x1 << UART_ENABLE_BIT);
#else
	CLK_CTRL_REG2 &= ~(0x1 << UART_ENABLE_BIT);
#endif

	/* configuration shared pins to UART */
	SRDPIN_CTRL1_REG |= (0x1 << SRDPIN_UART_RXTX_BIT);
#ifdef CONFIG_ANYKA_LL_DEBUG_UART3
	SRDPIN_CTRL2_REG &= ~(0x3 << SRDPIN_UART3_SDIO_BIT);
	SRDPIN_CTRL2_REG |= (0x1 << SRDPIN_UART3_SDIO_BIT);
#endif

	/* configuration uart pin */
	PPU_PPD1_REG |= (0x1 << URD_PU_BIT) | (0x1 << UTD_PU_BIT);
#ifndef CONFIG_ANYKA_LL_DEBUG_UART0
	GPIO_CTRL1_REG |= (0x1 << URD_PE_BIT) | (0x1 << UTD_PE_BIT);
	GPIO_CTRL1_REG &= ~(0x1 << UTD_IE_BIT);
	GPIO_CTRL1_REG |= (0x1 << URD_IE_BIT);
#endif

	/* Set baud rate, PLL CLK = 240M, CLK168M = 120M, ASIC CLK = 60M */
	pll_clk = uidiv(4 * (CLK_DIV_REG & 0x3f) + 180, (((CLK_DIV_REG >> 17 )& 0xf ) + 1));
	asic_div = (CLK_DIV_REG >> 6) & 0x7;
	if (asic_div == 0)
		asic_div = 1;
	asic_clk = (pll_clk >> asic_div) * 1000 * 1000;
	clk_div = uidiv(asic_clk, BAUD_RATE) - 1;
	UART_CONF1_REG &= ~((0x1 << TX_STATUS_CLR_BIT) | (0x1 << RX_STATUS_CLR_BIT) | 0xFF);
	UART_CONF1_REG |= (0x1 << TX_STATUS_CLR_BIT) | (0x1 << RX_STATUS_CLR_BIT) | clk_div; 
	
#ifndef CONFIG_ANYKA_LL_DEBUG_UART0
	/* Disable flow control */
	UART_CONF1_REG |= (0x1 << CTS_SEL_BIT) | (0x1 << RTS_SEL_BIT);
#endif
	UART_BUF_STOPBIT_REG = (0x1F << 16) | (0x1 << 0);

	/* enable uart port */
	UART_CONF1_REG |= (0x1 << PORT_ENABLE_BIT);
}


/* print a char to uart */
static void putc(char c)
{
	/* Clear uart tx buffer */
	L2BUF_CONF2_REG   |= (0x1 << UART_TXBUF_CLR_BIT);

	/* write char to uart buffer */
	L2BUF(UART_TXBUF_ADDR) = (unsigned long)c;
	L2BUF(UART_TXBUF_ADDR + ENDDING_OFFSET1) = (unsigned long)'\0';
	
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

/* nothing to do */
#define arch_decomp_wdog()

#endif   /* __UNCOMPRESS_H_ */
