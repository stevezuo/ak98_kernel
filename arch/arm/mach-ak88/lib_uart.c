/**
 * @file uart.c
 * @brief UART driver, define UARTs APIs.
 * This file provides UART APIs: UART initialization, write data to UART, read data from
 * UART, register callback function to handle data from UART, and interrupt handler.
 * Copyright (C) 2004 Anyka (GuangZhou) Software Technology Co., Ltd.
 * @author ZouMai
 * @date 2004-09-16
 * @version 1.0
 * @ref AK8802 technical manual.
 */

#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <mach/map.h>
#include <mach/lib_uart.h>
#include <mach/lib_l2.h>

#define UART_RX_FIFO_SIZE       (32)	//1//128 bytes

typedef void (*t_fuart_hisr_handler) (void);

typedef struct {
	unsigned int baudrate;
	unsigned char b_interrupt;
	unsigned char b_open;

	unsigned char b_use_dma;
	t_fuart_callback callback_func;
	unsigned char *dma_tx_buffer;
	unsigned int dma_tx_buffer_length;
	unsigned char *dma_rx_buffer[2];
	unsigned char dma_rx_buffer_shift;
	unsigned char *p_dma_tx_buffer;
	unsigned int n_trans_count;
	unsigned int n_trans_complete_count;
	unsigned char *p_receive_pool;
	unsigned int n_receive_pool_length;
	unsigned int n_receive_pool_head;
	unsigned int n_receive_pool_tail;
	unsigned int rxfifo_offset;	//4bytes in unit, it refer to the buffer in l2 memory,for example 0x48001000~0x4800180

	unsigned int timeout_delay_us;

} T_UART;

static volatile T_UART m_uart[MAX_UART_NUM] = { {0} };

#define	TX_STATUS		1
#define	RX_STATUS		0

/*
 * @brief: Get Uart's base register address
 */
//#define  uart_id2register(uart_id) 
//    (UART0_BASE_ADDR+(T_U32)(uart_id)*0x1000)

#define uart_id2register(uart_id/*0~3*/)  (unsigned int) (AK88_UART_BASE+(unsigned int)(uart_id/*0~3*/)*0x1000)	//0x20026000

//static void uart2_flowcontrol_init(void);
unsigned char uart0_interrupt_handler(void);
unsigned char uart1_interrupt_handler(void);
//static unsigned char uart2_interrupt_handler(void);
//static unsigned char uart3_interrupt_handler(void);
static void uart_handler(unsigned char uart_id);
static void uart_storedata(unsigned char uart_id, unsigned char *data,
			   unsigned int datalen);
static unsigned int uart_read_fifo(unsigned char uart_id, unsigned char *chr,
				   unsigned int count);

void uart_clock_ctl(unsigned char uart_id, unsigned char enable)
{
	volatile unsigned int value;

	value = __raw_readl(AK88_POWER_CLOCK);   //0x0800,000c

	switch (uart_id) {
	default:
	case uiUART0:
		if (enable) 	//VME_SetSleepModeConfig(CLOCK_UART0_ENABLE);    //bit15
			value &=~(1UL<<15);		
		else		//VME_SetSleepModeConfig(~CLOCK_UART0_ENABLE);
			value |=(1UL<<15);
		break;
#ifdef CHIP_780X
	case uiUART1:
		if (enable) ;	//VME_SetSleepModeConfig(CLOCK_UART1_ENABLE);
		else;		//VME_SetSleepModeConfig(~CLOCK_UART1_ENABLE);
		break;
	case uiUART2:
		if (enable) ;	//VME_SetSleepModeConfig(CLOCK_UART2_ENABLE);
		else;		//VME_SetSleepModeConfig(~CLOCK_UART2_ENABLE);
		break;
#endif
	case uiUART3:
		if (enable) 	//VME_SetSleepModeConfig(CLOCK_UART3_ENABLE);   //bit8
			value &=~(1UL<<8);
		else		//VME_SetSleepModeConfig(~CLOCK_UART3_ENABLE);
			value |=(1UL<<8);
		break;
 	}

	__raw_writel(value, AK88_POWER_CLOCK);	//share pin

}

void uart_pin_ctl(unsigned char uart_id)
{

	volatile unsigned int value,value1;

	value = __raw_readl(AK88_VA_SYS + 0x78);

	value1 = __raw_readl(AK88_VA_SYS + 0x74);

	/* set share pin to UARTn, and disable pull-up */
	switch (uart_id) {
	case uiUART0:
		//gpio_pin_group_cfg(ePIN_AS_UART1);
		value |= (1UL << 9);
		break;
#ifdef CHIP_780X
	case uiUART1:
		//gpio_pin_group_cfg(ePIN_AS_UART2);
		value |= (1UL << 10);
		break;
	case uiUART2:
		//gpio_pin_group_cfg(ePIN_AS_UART3);
		value |= (1UL << 12);
		break;
#endif
	case uiUART3:
		//gpio_pin_group_cfg(ePIN_AS_UART4);
		value |= (1UL << 14);
		value |= (1UL << 15);
		value1 &=~(3UL<<1);
		value1 |=(1UL<<1);
		__raw_writel(value1, (AK88_VA_SYS + 0x74));	//share pin
		break;
	default:
		break;
		value |= (1UL << 9);
		break;
		//printk("uart_pin_ctl():unknown uart id %d!!\n", uart_id);
	}

	__raw_writel(value, (AK88_VA_SYS + 0x78));	//share pin

	value = __raw_readl(AK88_VA_DEV + 0xc084);
	value |= (3UL << 28);
	__raw_writel(value, (AK88_VA_DEV + 0xc084));	//dma pin

}

/**
* @BRIEF Clear transmit or receive status of uart controller
* @AUTHOR Pumbaa
* @DATE 2007-08-04
* @PARAM UART_Select uart_id : uart id
* @PARAM T_U8 txrx_status : TX_STATUS: clear transmit status; RX_STATUS: clear receive status
* @RETURN T_VOID: 
* @NOTE: ...
*/
void uart_clear_tx_status(unsigned char uart_id)
{
	unsigned int base_addr;
	volatile unsigned int reg_value;

	base_addr = (unsigned int)uart_id2register(uart_id);
	reg_value = __raw_readl(base_addr + 0x00);

	reg_value |= (1UL << 28);
	__raw_writel(reg_value, (base_addr + 0x00));
}

void uart_clear_rx_status(unsigned char uart_id)
{
	unsigned int base_addr;
	volatile unsigned int reg_value;

	base_addr = (unsigned int)uart_id2register(uart_id);
	reg_value = __raw_readl(base_addr + 0x00);
	reg_value |= (1UL << 29);
	__raw_writel(reg_value, (base_addr + 0x00));
}

/**
* @BRIEF clear time out status
* @AUTHOR Pumbaa
* @DATE 2007-08-04
* @PARAM UART_Select uart_id : uart id
* @RETURN T_VOID: 
* @NOTE: ...
*/
//static T_VOID ClearTimeout(T_UART_ID uart_id)
unsigned int uart_clear_rx_timeout(unsigned char uart_id)
{
	volatile unsigned int reg_value;
	unsigned int base_addr;

	base_addr = uart_id2register(uart_id);	//0x20026004
	reg_value = __raw_readl(base_addr + 0x04);
	reg_value |= (1UL << 2);	//clear timeout int status
	//reg_value &= ~(1<<3); ?
	reg_value |= (1UL << 3);	//clear R_err int status
	//outl(reg_value, (base_addr+0x04));
	__raw_writel(reg_value, (base_addr + 0x04));

	reg_value = ((__raw_readl(base_addr + 0x08)) >> 23) & 0x03;

	return reg_value;

}

void uart_clear_rx_err(unsigned char uart_id)
{
	volatile unsigned int reg_value;
	unsigned int base_addr;

	base_addr = uart_id2register(uart_id);	//0x20026004
	reg_value = __raw_readl(base_addr + 0x04);
	reg_value |= (1UL << 3);	//clear R_err int status
	__raw_writel(reg_value, (base_addr + 0x04));

	return;

}

void uart_clear_rx_buffull(unsigned char uart_id)
{
	volatile unsigned int reg_value;
	unsigned int base_addr;

	base_addr = uart_id2register(uart_id);	//0x20026000
	//bit[1]:RX_buf_full, 1=The receiving buffer is full.
	//                Notes:This bit is cleared by being written with 1.
	reg_value = __raw_readl(base_addr + 0x04);	//0x20026004
	reg_value |= (1UL << 1);
	//reg_value &= ~(1<<3);?
	reg_value |= (1UL << 3);
	//outl(reg_value, (base_addr+0x04));
	__raw_writel(reg_value, (base_addr + 0x04));

}

void uart_clear_rx_th(unsigned char uart_id)
{
	volatile unsigned int reg;
	unsigned int base_addr;
	//0x2002600c,TX/RX Threshold Register
	//bit[5]:Rx_th_clr,1=To clear Rx_th interrupt,and prevent the threshold counting.

	//0x20026004,UARTn Configuration Reg2 
	//bit[30]:Rx_th_int_sta,1=A RX_th interrupt is generated
	//Note:
	//  2.The receive buffer threshold value is set by bits[4:0] of 0x2002600C.
	//  3.This bit is cleared by being written with 1 or when Rx_th_clr,bit[5]
	//    of 0x2002600C is '1'

	base_addr = uart_id2register(uart_id);	//0x20026000

	reg = __raw_readl(base_addr + 0x0c);
	reg |= (1UL << 5);
	__raw_writel(reg, base_addr + 0x0c);
	//reg &= ~(1<<5);
	//__raw_writel(reg,base_addr+0x0c);

	reg = __raw_readl(base_addr + 0x04);
	reg |= (1UL << 30);
	__raw_writel(reg, base_addr + 0x04);

	//reg=__raw_readl(base_addr+0x0c);
	//reg &= ~(1<<5);
	//__raw_writel(reg,base_addr+0x0c);

	return;

}

void uart_reen_rx_th(unsigned char uart_id)
{
	volatile unsigned int reg;
	unsigned int base_addr;

	base_addr = uart_id2register(uart_id);	//0x20026000

	reg = __raw_readl(base_addr + 0x0c);
	reg &= ~(1UL << 5);
	__raw_writel(reg, base_addr + 0x0c);
}

#define	UART_TX_END_STA				    (1UL<<19)
#define	UART_RX_TIMEOUT				    (1UL<<2)

unsigned char uart_wait_tx_finish(unsigned char uart_id)
{
	volatile unsigned int baseAddress, status;

	baseAddress = uart_id2register(uart_id);

	while (1) {
		status = __raw_readl(baseAddress + 0x04);
		//0x20026004
		//bit[19]:TX_end: 1=all the data in TX buffer has been sent
		if (status & UART_TX_END_STA)
			break;
	}

	return 1;
}

unsigned char uart_get_tx_empty(unsigned char uart_id)
{
/* if tx send empty,return 1   
  
  yes:return 1;
  no :return 0;

*/
	volatile unsigned int base_address, status;

	base_address = uart_id2register(uart_id);
	status = __raw_readl(base_address + 0x04);

	if (status & UART_TX_END_STA)
		return 1;	//yes ,tx empty,send finish
	else
		return 0;

}

//#define ak880x_uart_get_tx_rdy(uart_id)    ((__raw_readl(uart_id2register(uart_id)+0x04)) & (1<<19))? 1:0

unsigned char uart_wait_rx_timeout(unsigned char uart_id)
{
	volatile unsigned int base_address, status;

	base_address = uart_id2register(uart_id);

	while (1) {
		status = __raw_readl(base_address + 0x04);
		//0x20026004
		//bit[2]:timeout,1=the receiving timeout occurs.
		if (status & UART_RX_TIMEOUT)
			break;
	}

	return 1;
}

unsigned char uart_get_rx_timeout(unsigned char uart_id)
/*yes:return 1;
  no :return 0;
*/
{
	volatile unsigned int base_address, status;

	base_address = uart_id2register(uart_id);
	status = __raw_readl(base_address + 0x04);

	if (status & (1UL << 2))
		return 1;	//yes,timeout occur
	else
		return 0;

	//while(!(__raw_readl(baseAddress+0x04)&(1<<2)))            //0x20026004        
	//{} 
	//bit2(0x20026004):timeout,1=the receiving timeout occurs.
	//Note:
	//   1.Please refer to bit[23] of 0x20026000 for the definition of timeout function
	//   2.When this bit is set to 1,programmers should begin to process the data in received buffer
	//   3.This bit is cleared by being written with 1.
	//
	//UART module can only process 4-byte data in a data transfer operation.
	//if no more data is received when the waiting time(32 bit cycles) is out ,an interrupt is generated(if enabled)
	//and the data left is transmitted to L2 memory.
	//
}

unsigned char uart_get_rx_buffull(unsigned char uart_id)
/*yes:return 1;
  no :return 0;
*/
{
	volatile unsigned int base_address, status;

	base_address = uart_id2register(uart_id);
	status = __raw_readl(base_address + 0x04);

	if (status & (1UL << 1))
		return 1;	//yes,the receive buffer is full.
	else
		return 0;
}

//#define ak880x_uart_get_rx_rdy(uart_id)    ((__raw_readl(uart_id2register(uart_id)+0x04))&(1<<2))? 1:0

unsigned char uart_init(unsigned char uart_id /*0~3 */ , unsigned int baud_rate,
			unsigned int sys_clk)
{
	unsigned int base_address;
	unsigned int reg_baud;

	if (uart_id > MAX_UART_NUM)
		return AK_FALSE;

	base_address = uart_id2register(uart_id);

	//set sharepin
	if (uart_id == 0)
		AKSET_BITS(1UL << 9, AK88_SHAREPIN_CTRL);
	else if (uart_id == 3) {
		AKSET_BITS(1UL << 15, AK88_SHAREPIN_CTRL);
		AKSET_BITS(1UL << 14, AK88_SHAREPIN_CTRL);
		AKCLR_BITS(3UL << 1, AK88_SHAREPIN_CTRL2);
		AKSET_BITS(1UL << 1, AK88_SHAREPIN_CTRL2);
	}
	//else return AK_FALSE;

	printk("uart_init(%d)/AK88_POWER_CLOCK=0x%x\n",uart_id,AK88_POWER_CLOCK);

	//open power clock
	if (uart_id == 0)
		AKCLR_BITS(1UL << 15, AK88_POWER_CLOCK);
	else if (uart_id == 3) 		//AKCLR_BITS(1UL<<24,AK88_POWER_CLOCK); // ?
	{
		AKCLR_BITS(1UL << 8, AK88_POWER_CLOCK);
		AKCLR_BITS(1UL << 24, AK88_POWER_CLOCK);  //to reset UART4
 	}
	else ;

	printk("uart_init(%d)/AK88_POWER_CLOCK=0x%x\n",uart_id,AK88_POWER_CLOCK);
 
	//set dma flag
	AKSET_BITS(0x3UL << 28, AK88_VA_DEV + 0xc084);
	//(*(volatile u32*)0x2002c084) |=  0x3UL<<28 ;
	//0x2002c084, ldma_flag_en|Ahb_flag_en

	//open flowcontrol
	/* should reverse CTS */
	//if (uart_id != 0)  //open flowcontrol
	//{
		//reg_value |= (1UL << 18) | (1UL << 19);
		//AKSET_BITS(1UL<<18,AK88_UART_CFG_REG1(uart_id) );
		//AKSET_BITS(1UL<<19,AK88_UART_CFG_REG1(uart_id) );
	//}

	//mask all interrupts
	__raw_writel(0, AK88_UART_CFG_REG2(uart_id));	//mask all interrupt

	reg_baud = sys_clk/baud_rate - 1;

	//printk("sys_clk=%d,baud_rate=%d,reg_baud=0x%x\n",sys_clk,baud_rate,reg_baud);

	reg_baud |=(1UL<<29)|(1UL<<28)|(1UL<<21); //clear_tx_stat|clear_tx_stat|uart_enable
 
	printk("uart_id=%d,sys_clk=%d,baud_rate=%d,reg_baud=0x%x\n",uart_id,sys_clk,baud_rate,reg_baud);

	//__raw_writel(reg_baud, AK88_UART_CFG_REG1(uart_id));

	
	//open flowcontrol
	/* should reverse CTS */
	if (uart_id != 0)  //open flowcontrol
	{
		//reg_value |= (1UL << 18) | (1UL << 19);
		AKSET_BITS(1UL<<18,AK88_UART_CFG_REG1(uart_id) );
		AKSET_BITS(1UL<<19,AK88_UART_CFG_REG1(uart_id) );
	}
 
	//set baud_rate
	//baud=0x30200433; //ak880x,default freq=124000000
	//baud=0x30200219; //ak780x,default freq=62000000

	return AK_TRUE;
}

 
#if 0
unsigned char uart_enable_int(unsigned char uart_id /*0~3 */ )
{
	unsigned int br_value;
	unsigned int reg_value;
	unsigned int base_address;

	base_address = uart_id2register(uart_id);

	reg_value = __raw_readl(base_address + 0x04);	//20026004

	//reg_value |=1<<29;  //TX_th_int
	reg_value |= 1 << 27;	//TX_end_int
	//reg_value |=1<<25;  //RX_buff_full_int
	//reg_value |=1<<23;  //R_err_int
	reg_value |= 1 << 22;	//timeout_int

	__raw_writel(reg_value, base_address + 0x04);

	return 0;
}
#endif

//T_BOOL uart_init(T_UART_ID uart_id, T_U32 baud_rate, T_U32 sys_clk)
unsigned char uart_init_old(unsigned char uart_id /*0~3 */ ,
			    unsigned int baud_rate, unsigned int sys_clk)
{
	volatile unsigned int br_value;
	volatile unsigned int reg_value;
	unsigned int baseAddress;

	if (uart_id > MAX_UART_NUM)
		return AK_FALSE;

	baseAddress = uart_id2register(uart_id);

#if 1				//temprary close
	/* clock control */
	uart_clock_ctl(uart_id, AK_TRUE);

	/* share pin and pull-up control */
	uart_pin_ctl(uart_id);

	/* set baudrate */
	br_value = sys_clk / baud_rate - 1;
	reg_value = br_value & 0xffff;

	if (sys_clk % baud_rate)
		reg_value |= (1 << 22);
	/*bit22:(0x20026000)DIV_ADJ,1=to enable baud rate adjustment function */
#endif

#if 1
	/* bit21: enable uartn interface 
	   bit23: enable timeout
	   bit28,29: clear status
	 */
	reg_value |= (1 << 20) | (1 << 21) | (1 << 28) | (1 << 29);
	//if (uart_id == uiUART0)
	reg_value |= (1 << 23);	//enable timeout

	//disable rx threshold
	//reg_value &= ~(1<<28);

	/* should reverse CTS */
	if (uart_id != uiUART0)
		reg_value |= (1 << 18) | (1 << 19);

	//HAL_WRITE_UINT32(baseAddress+UART_CFG_REG1, reg_value);
	//HAL_WRITE_UINT32(baseAddress+UART_CFG_REG2, 0);//mask all interrupt
	__raw_writel(reg_value, AK88_UART_CFG_REG1(uart_id));
	__raw_writel(0, AK88_UART_CFG_REG2(uart_id));	//mask all interrupt

#endif
	if (m_uart[uart_id].b_open == AK_FALSE) {
		m_uart[uart_id].callback_func = AK_NULL;
	}
	m_uart[uart_id].baudrate = baud_rate;
	m_uart[uart_id].b_open = AK_TRUE;

	m_uart[uart_id].b_use_dma = AK_FALSE;
	m_uart[uart_id].p_dma_tx_buffer = AK_NULL;
	m_uart[uart_id].dma_rx_buffer[0] = AK_NULL;
	m_uart[uart_id].dma_rx_buffer[1] = AK_NULL;
	m_uart[uart_id].dma_rx_buffer_shift = 0;
	m_uart[uart_id].dma_tx_buffer = AK_NULL;
	m_uart[uart_id].dma_tx_buffer_length = 0;

	m_uart[uart_id].n_trans_count = 0;
	m_uart[uart_id].n_trans_complete_count = 0;

	m_uart[uart_id].p_receive_pool = AK_NULL;
	m_uart[uart_id].n_receive_pool_length = 0;
	m_uart[uart_id].n_receive_pool_head = 0;
	m_uart[uart_id].n_receive_pool_tail = 0;
	m_uart[uart_id].rxfifo_offset = 0;

	baud_rate = (0 == baud_rate) ? 115200 : baud_rate;
#ifdef CHIP_780X

	if (uiUART2 == uart_id) {
		// count each cycle used us number
		m_uart[uart_id].timeout_delay_us = 1000000 * 64 / baud_rate;
	} else {
		// count each cycle used us number
		m_uart[uart_id].timeout_delay_us = 560;
	}
#endif
	return AK_TRUE;
}

//T_BOOL uart_set_DMA_mode(T_UART_ID uart_id, T_U8 *DMATXBuffer, T_U32 DMATXBufferLength, T_U8 *DMARXBuffer, T_U32 DMARXBufferLength)
unsigned char uart_set_dma_mode(unsigned char uart_id /*0~3 */ ,
				unsigned char *dma_tx_buffer,
				unsigned int dma_tx_buffer_length,
				unsigned char *dma_rx_buffer,
				unsigned int dma_rx_buffer_length)
{

	if (uart_id > MAX_UART_NUM)
		return AK_FALSE;
/*
    if(dma_tx_buffer == AK_NULL || dma_rx_buffer == AK_NULL)
        return AK_FALSE;

    baseAddress = uart_id2register( uart_id );

    m_uart[uart_id].b_use_dma = AK_TRUE;

    m_uart[uart_id].dma_tx_buffer = dma_tx_buffer;
    m_uart[uart_id].dma_tx_buffer_length = dma_tx_buffer_length;

    m_uart[uart_id].dma_rx_buffer[0] = dma_rx_buffer;
    m_uart[uart_id].dma_rx_buffer[1] = dma_rx_buffer + dma_rx_buffer_length / 2;

    return AK_TRUE;
*/
	//to be implemented
	return AK_FALSE;
}

//T_VOID uart_on_change( T_U32 sys_clk )
void uart_on_change(unsigned int sys_clk)
{

	unsigned char i;
	volatile unsigned int baseAddress, br_value, reg_value;

	for (i = uiUART0; i < MAX_UART_NUM; i++) {
		if (m_uart[i].b_open) {
			baseAddress = uart_id2register(i);

			br_value = sys_clk / m_uart[i].baudrate - 1;

			reg_value = __raw_readl(baseAddress + 0x00);

			reg_value &= ~(0xffff);
			reg_value &= ~(1UL << 22);
			//bit22(0x20026000):DIV_ADJ,1=to enable baud rate adjustment function
			//                 Notes:
			//                  1.When baud rate adjustment function is enabled,the bit cycle of data frame will be extended one more
			//                    ASIC CLK cycle than that configured.
			//                  2.This function is used to reduce the tolerance caused by the inaccurate ratio of frequency dividing.
			reg_value |= (br_value & 0xffff);
			if (sys_clk % m_uart[i].baudrate)
				reg_value |= (1UL << 22);

			__raw_writel(reg_value, baseAddress + 0x00);
		}
	}
}

extern unsigned char b_test;

//#define       UART_TX_END_STA                             (1<<19)
/* this function for byte_nbr<64 ,64 is the dma transfer unit */
unsigned char uart_write_buf(unsigned char uart_id /*0~3 */ ,
			     unsigned char *chr,
			     unsigned int byte_nbr
			     /*<60,last 4 bytes(0x3c/0x7c) write 0 */ )
{
	unsigned int baseAddress;
	//unsigned int status;
	volatile unsigned int reg_value;
	unsigned char buf_id = 8 + (uart_id << 1);
	unsigned int buf_addr;
	//unsigned int buf_addr, i,cnt=0;
	//unsigned int value1,value2;
	//unsigned int count = 0;

	//if(m_uart[ uart_id ].b_open == AK_FALSE)   return AK_FALSE;

	if (byte_nbr >= 64)
		return AK_FALSE;

	baseAddress = uart_id2register(uart_id);
	//buf_addr = L2_BUF_MEM_BASE_ADDR + 4096/*8 * 512*/ + ((buf_id-8)<<7);  //0x48000000+0x1000
	buf_addr = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//<<7 = *128

	l2_clr_uartbuf_status(buf_id /*8~15 */ );

	uart_clear_tx_status(uart_id);

	l2_tran_data_cpu((unsigned int)chr, buf_id, 0, byte_nbr,
			 1 /*write(send) */ );

	//value1 = ReadBuf(buf_addr+SET_FLAG_OFFSET0); //forbiden to read
	write_buf(0, buf_addr + 0x3c);

	reg_value = __raw_readl(baseAddress + 0x04);	//0x20026004
	reg_value &= 0x3fe00000;
	reg_value |= (byte_nbr << 4) | (1UL << 16);
	__raw_writel(reg_value, baseAddress + 0x04);

	uart_wait_tx_finish(uart_id);

	return AK_TRUE;
}

#define	UART_RXFIFO_FULL_STA			(1<<1)
#define	REG_04_MSK_BIT				(0x3fe00000)
/*this function for byte_nbr >= 64 bytes, 64 is the dma transfer unit*/
unsigned int uart_write_cpu(unsigned char uart_id /*0~3 */ ,
			    const unsigned char *chr,
			    unsigned int byte_nbr /*>=60 */ )
{
	volatile unsigned int reg_value;
	//unsigned int status;
	unsigned int base_addr;
	unsigned int set_flag_addr;
	unsigned int tran_64_nbr;
	unsigned int frac_nbr;
	unsigned int buf_addr;
	//unsigned int i,j;
	unsigned int i;
	unsigned char buf_id;
	unsigned int ram_addr = (unsigned int)chr;

	base_addr = uart_id2register(uart_id);
	buf_id = 8 + uart_id * 2;

	l2_clr_uartbuf_status(buf_id);
	uart_clear_tx_status(uart_id);

	tran_64_nbr = byte_nbr >> 6;
	frac_nbr = byte_nbr % 64;

	buf_addr = (unsigned int)AK88_VA_L2BUF + 0x1000 + (uart_id << 8);	// <<8 = *256

	if (tran_64_nbr % 2 == 0) {
		set_flag_addr = buf_addr + 0x3c;
	} else {
		set_flag_addr = buf_addr + 0x7c;
	}

	if (tran_64_nbr || frac_nbr) {
		reg_value = __raw_readl(base_addr + 0x04);
		if (reg_value & UART_RXFIFO_FULL_STA) {
			//printk("F4!\n");
			while (1) ;
		}
		reg_value &= REG_04_MSK_BIT;

		reg_value |= (((tran_64_nbr << 6) + frac_nbr) << 4) | (1 << 16);	//data length

		__raw_writel(reg_value, base_addr + 0x04);
	}
	for (i = 0; i < tran_64_nbr; i++) {
		while (l2_uartbuf_status(buf_id) == 2) {	/*if return is 2, it mean buffer is full, 
								   if return 1, it mean buffer is half full,
								   otherwise mean buffer is empty */
		}
		if (i % 2 == 0)	//even 64'sbuf
			l2_tran_data_cpu(ram_addr + (i << 6), buf_id, 0, 64,
					 1 /*write(send) */ );
		else		//odd 64'sbuf
			l2_tran_data_cpu(ram_addr + (i << 6), buf_id, 64, 64,
					 1 /*write(send) */ );
	}
	if (frac_nbr) {
		while (l2_uartbuf_status(buf_id) == 2) {	/*if return is 2, it mean buffer is full */
		}
		if (tran_64_nbr % 2 == 0)	//even
			l2_tran_data_cpu(ram_addr + (tran_64_nbr << 6), buf_id,
					 0, frac_nbr, 1);
		else		//odd
			l2_tran_data_cpu(ram_addr + (tran_64_nbr << 6), buf_id,
					 64, frac_nbr, 1);

		if (frac_nbr <= 60)
			write_buf(0, set_flag_addr);
	}
	//putch('Q');

	//while (1 && bTest)

	uart_wait_tx_finish(uart_id);

	//putch('P');

	return byte_nbr;
}

unsigned int uart_write_dma(unsigned char uart_id, const unsigned char *chr,
			    unsigned int byte_nbr)
{
	volatile unsigned int reg_value;
	//unsigned int status;
	unsigned int base_addr;
	unsigned int set_flag_addr;
	unsigned int tran_2k_nbr;
	unsigned int tran_64_nbr;
	unsigned int frac_nbr;
	unsigned int i;
	unsigned char buf_id;
	unsigned char buf_offset;
	unsigned int ram_addr = (unsigned int)chr;
	//MMU_InvalidateDCache();

	base_addr = uart_id2register(uart_id);
	buf_id = 8 + uart_id * 2;
	tran_2k_nbr = byte_nbr >> 11;
	tran_64_nbr = (byte_nbr - (tran_2k_nbr << 11)) >> 6;	//tran_64_nbr: remainder of 2048
	frac_nbr = byte_nbr % 64;	//frac_nbr: remainder of 64
	if (tran_64_nbr % 2 == 0) {
		set_flag_addr = (unsigned int)AK88_VA_L2BUF + 0x1000 + (uart_id << 8) + 0x3c;	//0x48000000
		buf_offset = 0;
	} else {
		set_flag_addr =
		    (unsigned int)AK88_VA_L2BUF + 0x1000 + (uart_id << 8) +
		    0x7c;
		buf_offset = 1;
	}

	l2_clr_uartbuf_status(buf_id);
	uart_clear_tx_status(uart_id);

	for (i = 0; i < tran_2k_nbr; i++) {
		reg_value = __raw_readl(base_addr + 0x04);
		reg_value &= REG_04_MSK_BIT;
		reg_value |= (2048 << 4) | (1 << 16);
		__raw_writel(reg_value, base_addr + 0x04);

		l2_uartbuf_dma(ram_addr + (i << 11), uart_id, 2048, 1);
		l2_wait_uartbuf_dmafinish(uart_id, 1);
	}
	if (tran_64_nbr || frac_nbr) {
		reg_value = __raw_readl(base_addr + 0x04);
		reg_value &= REG_04_MSK_BIT;
		reg_value |= (((tran_64_nbr << 6) + frac_nbr) << 4) | (1 << 16);
		__raw_writel(reg_value, base_addr + 0x04);
	}
	if (tran_64_nbr) {
		l2_uartbuf_dma(ram_addr + (tran_2k_nbr << 11), uart_id,
			       (tran_64_nbr << 6), 1);
		l2_wait_uartbuf_dmafinish(uart_id, 1);
	}
	if (frac_nbr) {
		while (l2_uartbuf_status(buf_id) == 2) {	/*if return is 2, it mean buffer is full */
		}
		l2_uartbuf_fracdma(ram_addr + (tran_2k_nbr << 11) +
				   (tran_64_nbr << 6), uart_id, buf_offset,
				   frac_nbr, 1);
		l2_wait_frac_dmafinish();

		if (frac_nbr <= 60)
			write_buf(0, set_flag_addr);
	}

	uart_wait_tx_finish(uart_id);

	return byte_nbr;
}

/**
 * @brief Write one character to UART base on UART ID
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param T_UART_ID uart_id: UART ID
 * @param T_U8 chr: The character which will be written to UART
 * @return T_BOOL: Write character OK or not
 * @retval AK_TRUE: Successfully written character to UART.
 * @retval AK_FALSE: Writing character to UART failed.
 */
unsigned char uart_write_chr(unsigned char uart_id, unsigned char chr)
{
	return uart_write_buf(uart_id, &chr, 1);
}

/**
 * @brief Write string to UART base on UART ID
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param T_UART_ID uart_id: UART ID
 * @param T_U8 *str: The string which will be written to UART
 * @return T_U32: Length of the data which have been written to UART
 * @retval
 */
unsigned int uart_write_str(unsigned char uart_id, unsigned char *str)
{
	unsigned int written_num = 0;

	//if(m_uart[ uart_id ].b_open == AK_FALSE)    return 0;

	/*if(m_uart[uart_id].bUseDMA)
	   {
	   akprintf(C3, M_DRVSYS, "Use DMA Mode\n");
	   return AK_FALSE;
	   } */

	while (*str != '\0') {
		uart_write_chr(uart_id, *str);
		written_num++;
		str++;
	}

	return written_num;
}

/**
 * @brief Write string data to UART
 * Write data to UART base on UART ID and data length
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-16
 * @param T_UART_ID uart_id: UART ID
 * @param const T_pDATA data: Constant data to be written to UART, this data needn't be end with '\0'
 * @param T_U32 data_len: Data length
 * @return T_U32: Length of the data which have been written to UART
 * @retval
 */
unsigned int uart_write(unsigned char uart_id, const unsigned char *data,
			unsigned int data_len)
{
	unsigned int written_num = 0;

	if (m_uart[uart_id].b_open == AK_FALSE)
		return 0;

#ifdef CHIP_780X

	if (uiUART2 != uart_id)	//for UART3
	{
		while (data_len > 0) {
			if (data_len < write_cnt)
				write_cnt = data_len;

			//uart_write_buf(uart_id, (T_U8*)data, write_cnt);
			uart_write_buf(uart_id, (unsigned char *)data,
				       write_cnt);

			data_len -= write_cnt;
			data += write_cnt;
			written_num += write_cnt;
		}
	} else			//dma write
#endif
	{
		if (data_len > 0) {
			written_num = uart_write_dma(uart_id, data, data_len);
			//written_num = uart_write_cpu(uart_id, data, data_len);
		}
	}

	return written_num;
}

unsigned int uart_read(unsigned char uart_id, unsigned char *data,
		       unsigned int datalen)
{
	unsigned int i = 0;

	if (m_uart[uart_id].b_interrupt == AK_TRUE) {
		if (m_uart[uart_id].n_receive_pool_length != 0)	//has datapool
		{
			for (i = 0; i < datalen; i++) {
				if (m_uart[uart_id].n_receive_pool_tail !=
				    m_uart[uart_id].n_receive_pool_head) {
					data[i] =
					    m_uart[uart_id].
					    p_receive_pool[m_uart[uart_id].
							   n_receive_pool_head++];
					m_uart[uart_id].n_receive_pool_head &=
					    (m_uart[uart_id].
					     n_receive_pool_length - 1);
				} else
					break;
			}
		}
	} else {
		unsigned int len1_3 = datalen & 3;	// len1_3 = 1~3
		unsigned int len4;	//len4 is 4's multiple

		len4 = datalen - len1_3;
		if (len4)	//read 4byte align
		{
			i += uart_read_fifo(uart_id, data, len4);
			data += i;
		}
		if (len1_3)	//read less than 4byte
		{
			while (i < datalen) {
				uart_read_chr(uart_id, data);
				i++;
				data++;
			}
		}
	}

	return i;
}

/**
 * @brief Read a character from UART
 * This function will not return until get a character from UART
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param T_UART_ID uart_id: UART ID
 * @param T_U8 *chr: character for return
 * @return T_BOOL: Got character or not
 * @retval
 */
unsigned char uart_read_chr(unsigned char uart_id, unsigned char *chr)
{
/* only read one char
 * 
 */
	unsigned char buf_id = 9 + (uart_id << 1);
	volatile unsigned int reg_value;
	unsigned int baseAddress, bufferAddress;
	volatile unsigned int nbr_to_read, timeout_count;
	unsigned char *p_buffer;

	if (uart_id > MAX_UART_NUM)
		return AK_FALSE;

	baseAddress = uart_id2register(uart_id);

	uart_wait_rx_timeout(uart_id);

	reg_value = __raw_readl(baseAddress + 0x04);
	reg_value |= (1UL << 2);	//clear the timeout bit
	__raw_writel(reg_value, baseAddress + 0x04);

	nbr_to_read = (__raw_readl(baseAddress + 0x08) >> 13) & 0x1f;	//0x20026008
	//bit[17:13]:RX_adr 
	timeout_count = (__raw_readl(baseAddress + 0x08) >> 23) & 0x3;
	//bit[24:23]:byt_left

	bufferAddress = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001000
	//p_buffer = (T_U8*)(bufferAddress + (m_uart[uart_id].rxfifo_offset<<2));
	p_buffer =
	    (unsigned char *)(bufferAddress +
			      (m_uart[uart_id].rxfifo_offset << 2));
	*chr = *p_buffer;

	//m_uart[uart_id].rxfifo_offset = (m_uart[uart_id].rxfifo_offset+1) % UART_RX_FIFO_SIZE;

	//return AK_TRUE;
	return *chr;
}

/**
 * @brief Read a character from UART asynchronously.
 * If no data, this function will also return directly.
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param T_UART_ID uart_id: UART ID
 * @param T_U8 *chr: character for return
 * @return T_BOOL: Got character or not
 * @retval
 */
static unsigned int uart_read_fifo(unsigned char uart_id, unsigned char *chr,
				   unsigned int count)
{
	unsigned char buf_id = 9 + (uart_id << 1);
	//unsigned int reg_value;
	unsigned int baseAddress;
	volatile unsigned int nbr_to_read = 0, timeout_count = 0, read_cnt =
	    0, tmp_cnt = 0;
	unsigned int bufferAddress, rxfifo_offset = 0;
	unsigned char *p_buffer;

	/* the read count must be multiple of 4 */
	if (count & 0x3) {
		printk("the count must be multiple of 4 !!\n");
		return 0;
	}

	/*
	   read_cnt is byte(8bit) unit
	   rxfifo_offset is lword(32bit) unit
	   nbr_to_read is lword(32bit) unit
	 */

	baseAddress = uart_id2register(uart_id);

	bufferAddress = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001080

	nbr_to_read = (__raw_readl(baseAddress + 0x08) >> 13) & 0x1f;
	//bit[17:13]:Rx_adr
	//nbr_to_read is lword(32bit) unit

	/* check fifo empty or not */
	if (nbr_to_read != m_uart[uart_id].rxfifo_offset) {
		/* we suppose to get timeout data at the end */

		rxfifo_offset = m_uart[uart_id].rxfifo_offset;

		/* check timeout status. if timeout, get timeout count */

		//to see if reach the last lwond in fifo
		if ((__raw_readl(baseAddress + 0x04) & (1UL << 2)) &&	//0x20026004 
		    ((rxfifo_offset + 1) % UART_RX_FIFO_SIZE == nbr_to_read))
			/*
			   'if ( (rxfifo_offset+1)%UART_RX_FIFO_SIZE == nbr_to_read ) '
			   means it is the last lword left in fifo,need clear timeout flag
			 */
		{
			uart_clear_rx_timeout(uart_id);

			timeout_count = (__raw_readl(baseAddress + 0x08) >> 23) & 0x3;	//0x20026008
			//bit[24:23](0x20026008):byt_left:The number of data left when receiving timeout occurs
			//timeout_count = 0 ~ 3 ;
			//

			read_cnt += timeout_count;
			rxfifo_offset = (rxfifo_offset + 1) % UART_RX_FIFO_SIZE;

		}
		/* data is still left in fifo, count it */
		if (nbr_to_read != rxfifo_offset) {
			// read_cnt*=4; read_cnt is byte(8bit) unit
			// rxfifo_offset is lword(32bit) unit
			// nbr_to_read is lword(32bit) unit
			if (nbr_to_read > rxfifo_offset) {
				read_cnt += (nbr_to_read - rxfifo_offset) << 2;	//read_cnt*=4
			} else {
				read_cnt +=
				    (UART_RX_FIFO_SIZE - rxfifo_offset) << 2;
				read_cnt += nbr_to_read << 2;
			}
		}
	}

	if (read_cnt > 0) {
		if (read_cnt > count)
			read_cnt = count;

		p_buffer =
		    (unsigned char *)(bufferAddress +
				      (m_uart[uart_id].rxfifo_offset << 2));

		//UART_RX_FIFO_SIZE<<2 = 128
		if ((read_cnt + (m_uart[uart_id].rxfifo_offset << 2)) <=
		    (UART_RX_FIFO_SIZE << 2)) {
			memcpy(chr, p_buffer, read_cnt);
		} else {
			tmp_cnt =
			    (UART_RX_FIFO_SIZE -
			     m_uart[uart_id].rxfifo_offset) << 2;
			memcpy(chr, p_buffer, tmp_cnt);
			p_buffer = (unsigned char *)bufferAddress;
			memcpy(chr + tmp_cnt, p_buffer, read_cnt - tmp_cnt);
		}

		//L2_TranDataCPU((T_U32)chr, buf_id, m_uart[uart_id].rxfifo_offset<<2, 
		//                          read_cnt, BUF2MEM);

		if (read_cnt & 0x3) {
			m_uart[uart_id].rxfifo_offset =
			    (m_uart[uart_id].rxfifo_offset + read_cnt / 4 +
			     1) % UART_RX_FIFO_SIZE;
		} else {
			m_uart[uart_id].rxfifo_offset =
			    (m_uart[uart_id].rxfifo_offset +
			     read_cnt / 4) % UART_RX_FIFO_SIZE;
		}
	}

	//if (read_cnt || nbr_to_read || rxfifo_offset|| timeout_count)
	//akprintf(C3, M_DRVSYS, "[%d,%d,%d,%d]", read_cnt, nbr_to_read, rxfifo_offset, timeout_count);
	return read_cnt;
}

unsigned int uart_read_timeout(unsigned char uart_id, unsigned char *chr)
/* when timeout occur ,read all the data in fifo from 0(0x48001080),return the real read number
*/
{
	unsigned char buf_id = 9 + (uart_id << 1);
	unsigned int base_address;
	volatile unsigned int nbr_to_read = 0, timeout_count = 0, read_cnt = 0;
	unsigned int buffer_address;
	unsigned char *p_buffer;
	volatile unsigned char timeout_flag = 0;

	/*
	   read_cnt is byte(8bit) unit
	   rxfifo_offset is lword(32bit) unit
	   nbr_to_read is lword(32bit) unit
	 */

	base_address = uart_id2register(uart_id);

	buffer_address = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001080

	timeout_flag = uart_get_rx_timeout(uart_id);
	if (!timeout_flag)
		return 0;

	nbr_to_read = (__raw_readl(base_address + 0x08) >> 13) & 0x1f;
	//bit[17:13]:Rx_adr
	//nbr_to_read is lword(32bit) unit

	timeout_count = (__raw_readl(base_address + 0x08) >> 23) & 0x3;	//0x20026008
	//bit[24:23](0x20026008):byt_left:The number of data left when receiving timeout occurs
	//timeout_count = 0 ~ 3 ;
	//

	if (nbr_to_read != 0)
		read_cnt += (nbr_to_read - 0) << 2;	//read_cnt*=4

	read_cnt += timeout_count;

	if (read_cnt > 0) {

		//pbuffer = (T_U8*)(bufferAddress + (m_uart[uart_id].rxfifo_offset<<2));
		p_buffer = (unsigned char *)(buffer_address + 0);

		memcpy(chr, p_buffer, read_cnt);

	}

	uart_clear_rx_timeout(uart_id);

	return read_cnt;
}

unsigned int uart_read_buffull(unsigned char uart_id, unsigned char *chr)
/* when RX_buf_full occur ,read all the data in fifo from 0(0x48001080),return the real read number
*/
{
	unsigned char buf_id = 9 + (uart_id << 1);
	unsigned int base_address;
	//unsigned int nbr_to_read=0,timeout_count=0, read_cnt=0;
	volatile unsigned int nbr_to_read = 0, read_cnt = 0;
	unsigned int buffer_address;
	unsigned char *p_buffer;
	volatile unsigned char buffull_flag = 0;

	/*
	   read_cnt is byte(8bit) unit
	   rxfifo_offset is lword(32bit) unit
	   nbr_to_read is lword(32bit) unit
	 */

	base_address = uart_id2register(uart_id);

	buffer_address = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001080

	buffull_flag = uart_get_rx_buffull(uart_id);
	if (!buffull_flag)
		return 0;

	nbr_to_read = (__raw_readl(base_address + 0x08) >> 13) & 0x1f;
	//bit[17:13]:Rx_adr
	//nbr_to_read is lword(32bit) unit

	if (nbr_to_read != 0)
		read_cnt += (nbr_to_read - 0) << 2;	//read_cnt*=4

	if (read_cnt > 0) {

		//pbuffer = (T_U8*)(bufferAddress + (m_uart[uart_id].rxfifo_offset<<2));
		p_buffer = (unsigned char *)(buffer_address + 0);

		memcpy(chr, p_buffer, read_cnt);

	}

	uart_clear_rx_buffull(uart_id);

	return read_cnt;
}

void uart_set_datapool(unsigned char uart_id, unsigned char *pool,
		       unsigned int poollength)
{
	unsigned int i = 0;

	if (pool == AK_NULL || poollength == 0) {
		//printk("pool can't be null\n");
		return;
	}
	//m_uart[uart_id].pReceivePool = pool;
	//m_uart[uart_id].nReceivePoolLength = poollength;
	m_uart[uart_id].p_receive_pool = pool;
	m_uart[uart_id].n_receive_pool_length = poollength;

	while (1) {
		if ((m_uart[uart_id].
		     n_receive_pool_length & (0x80000000 >> i)) != 0) {
			m_uart[uart_id].n_receive_pool_length =
			    m_uart[uart_id].
			    n_receive_pool_length & (0x80000000 >> i);
			break;
		}

		i++;
	}
}

/**
 * @brief Register a callback function to process UART received data.
 * This function words only in the UART interrupt mode.
 * Caution: The macro definition "__ENABLE_UARTxx_INT__" must be defined. 
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param T_UART_ID uart_id: UART ID
 * @param T_fUART_CALLBACK callback_func: Callback function
 * @return T_VOID
 * @retval
 */
void uart_set_callback(unsigned char uart_id, t_fuart_callback callback_func)
{
	unsigned int baseAddress;
	volatile unsigned int value;

	//akprintf(C3, M_DRVSYS, "+++++++++++uart_set_callback %d++++++++++++++++++\n", uart_id);

	baseAddress = uart_id2register(uart_id);	//0x20026000

	/* disable uart interrupt callback */
	if (callback_func == AK_NULL) {
		m_uart[uart_id].b_interrupt = AK_FALSE;
		m_uart[uart_id].rxfifo_offset = 0;
		__raw_writel(0, baseAddress + 0x04);	//mask all interrupt //0x20026004
		uart_clear_rx_status(uart_id);
		l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);
		return;
	}

	m_uart[uart_id].callback_func = callback_func;
	m_uart[uart_id].b_interrupt = AK_TRUE;

	/* set threshold to 4bytes */
	value = __raw_readl(baseAddress + 0x0c);	//0x2002600c

	value &= ~0x1f;
#ifdef CHIP_780X
	if (uiUART2 == uart_id)
		value |= 0xf;	// (cfg+1)*4
	else
#endif
		value |= 0x0;
	__raw_writel(value, baseAddress + 0x0c);

	uart_clear_rx_status(uart_id);
	l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);

	/* clear count */
	value = __raw_readl(baseAddress + 0x0c);	//0x2002600c
	//bit5(0x2002600c):Rx_th_clr:1=to clear Rx_th interrupt,and prevent the threshold counting
	//                 Note:Please refer to bit[28] of UART_CFG_REG2(n)(0x20026004) for the definition of RX_th interrupt
	value |= (1UL << 5);
	__raw_writel(value, baseAddress + 0x0c);
	value &= ~(1UL << 5);
	__raw_writel(value, baseAddress + 0x0c);

	/* enable timeout, r_err, rx buffer full and rx_th interrupt */
	value = __raw_readl(baseAddress + 0x04);
	value |= (1UL << 22) | (1UL << 23) | (1UL << 26) | (1UL << 28);
	//0x20026004
	//bit[22]:timeout_int:1=to enable timeout interrupt
	//        Note:Please refer to bit[23] of UART_CFG_REG1(n)(0x20026000) for the definition of timeout function
	//bit[23]:R_err_int:1=to enable R_err interrupt
	//        Note:When an error occurs in the received data,an interrupt is generated ,
	//        this kind of interrupt is called R_err interrupt               
	//bit[26]:RX_buf_full_int:1=to enable RX_buf_full interrupt
	//bit[28]:RX_th_int:1=to enable RX_th interrupt
	//        Note:1.Programmers may set a threshold of the receive buffer.When the number of received data
	//        is equal to the threshold value,an interrupt is generated.This kind of interrupt is called 
	//        RX_th interrupt
	//        2.The receive buffer threshold value is set by bits[4:0] of UART_THREHOLD(n)(0x2002600c)

	__raw_writel(value, baseAddress + 0x04);

	return;
}

void uart_close_interrupt(unsigned char uart_id)
{
	unsigned int base_address;
	//volatile unsigned int  value;

	base_address = uart_id2register(uart_id);	//0x20026000

	__raw_writel(0, base_address + 0x04);	//mask all interrupt //0x20026004
	uart_clear_rx_status(uart_id);
	uart_clear_rx_th(uart_id);
	l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);

	uart_clear_tx_status(uart_id);
	uart_clear_rx_status(uart_id);
	uart_clear_rx_timeout(uart_id);
	uart_clear_rx_buffull(uart_id);
	__raw_writel(0, base_address + 0x04);	//mask all interrupt //0x20026004

	printk("dbg:end:uart_close_interrupt(%d)\n", uart_id);
	return;
}

void uart_open_interrupt(unsigned char uart_id)
{
	unsigned int base_address;
	volatile unsigned int value;

	printk("uart_open_interrupt(),uart_id=%d\n",uart_id);

	base_address = uart_id2register(uart_id);	//0x20026000

	value = __raw_readl(AK88_VA_SYS + 0x78);
	if (uart_id == 0)
		value |= (1UL << 9);
	else if (uart_id == 3)
		value |= (1UL << 14);
	__raw_writel(value, (AK88_VA_SYS + 0x78));	//share pin

	value = __raw_readl(AK88_VA_DEV + 0xc084);
	value |= (3UL << 28);
	__raw_writel(value, (AK88_VA_DEV + 0xc084));	//dma pin

	__raw_writel(0, base_address + 0x04);	//mask all interrupt //0x20026004

	uart_clear_rx_status(uart_id);
	uart_clear_rx_th(uart_id);
	l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);

	__raw_writel(0, base_address + 0x04);	//mask all interrupt //0x20026004

#if 0
	/* disable uart interrupt callback */
	if (callback_func == AK_NULL) {
		m_uart[uart_id].b_interrupt = AK_FALSE;
		m_uart[uart_id].rxfifo_offset = 0;
		__raw_writel(0, baseAddress + 0x04);	//mask all interrupt //0x20026004
		uart_clear_rx_status(uart_id);
		l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);
		return;
	}
#endif

	//m_uart[uart_id].callback_func = callback_func;
	//m_uart[uart_id].b_interrupt  = AK_TRUE;

	/* set threshold to 4bytes */
	value = __raw_readl(base_address + 0x0c);	//0x2002600c

	value &= ~0x1f;
#ifdef CHIP_780X
	if (uiUART2 == uart_id)
		value |= 0xf;	// (cfg+1)*4
	else
#endif
		value |= 0x0;
	__raw_writel(value, base_address + 0x0c);

	value = __raw_readl(AK88_UART_CFG_REG1(uart_id));
	//value |= ((1UL<<23)|(1UL<<21));   //timeout_en,EN
	value |= 1UL << 21;	//uart interface EN

	__raw_writel(0, (AK88_UART_THREHOLD(uart_id)));

	//uart_clear_rx_status(uart_id);
	//uart_clear_rx_timeout(uart_id);
	uart_clear_rx_buffull(uart_id);
	l2_clr_uartbuf_status((unsigned char)uart_id * 2 + 9);

	uart_clear_rx_th(uart_id);
#if 0
	///* clear interrupt count */
	value = __raw_readl(base_address + 0x0c);	//0x2002600c
	//bit5(0x2002600c):Rx_th_clr:1=to clear Rx_th interrupt,and prevent the threshold counting
	//                 Note:Please refer to bit[28] of UART_CFG_REG2(n)(0x20026004) for the definition of RX_th interrupt
	value |= (1 << 5);
	__raw_writel(value, base_address + 0x0c);
	value &= ~(1 << 5);
	__raw_writel(value, base_address + 0x0c);
#endif

	__raw_writel(0, (AK88_UART_THREHOLD(uart_id)));	//threshold count=1;

	/* enable timeout, r_err, rx buffer full and rx_th interrupt */
	value = __raw_readl(base_address + 0x04);
	//value |= (1UL<<22)|(1UL<<23)|(1UL<<26)|(1UL<<28);  //timeout_int|R_err_int|RX_buf_full_int|RX_th_int
	value |= 1UL << 28;	//RX_th_int 
	//value |=1UL<<22;
	//value |= 1UL<<27;
	//0x20026004
	//bit[22]:timeout_int:1=to enable timeout interrupt
	//        Note:Please refer to bit[23] of UART_CFG_REG1(n)(0x20026000) for the definition of timeout function
	//bit[23]:R_err_int:1=to enable R_err interrupt
	//        Note:When an error occurs in the received data,an interrupt is generated ,
	//        this kind of interrupt is called R_err interrupt               
	//bit[26]:RX_buf_full_int:1=to enable RX_buf_full interrupt
	//bit[28]:RX_th_int:1=to enable RX_th interrupt
	//        Note:1.Programmers may set a threshold of the receive buffer.When the number of received data
	//        is equal to the threshold value,an interrupt is generated.This kind of interrupt is called 
	//        RX_th interrupt
	//        2.The receive buffer threshold value is set by bits[4:0] of UART_THREHOLD(n)(0x2002600c)

	__raw_writel(value, base_address + 0x04);

	uart_reen_rx_th(uart_id);

	printk("dbg:end:uart_open_interrupt(%d)\n", uart_id);
	return;
}

static void uart_storedata(unsigned char uart_id, unsigned char *data,
			   unsigned int datalen)
{
	if (m_uart[uart_id].p_receive_pool != AK_NULL) {
		if (m_uart[uart_id].n_receive_pool_tail + datalen >
		    m_uart[uart_id].n_receive_pool_length) {
			unsigned int cpylen =
			    m_uart[uart_id].n_receive_pool_length -
			    m_uart[uart_id].n_receive_pool_tail;

			memcpy(&m_uart[uart_id].
			       p_receive_pool[m_uart[uart_id].
					      n_receive_pool_tail], data,
			       cpylen);
			memcpy(&m_uart[uart_id].p_receive_pool[0],
			       data + cpylen, datalen - cpylen);
			m_uart[uart_id].n_receive_pool_tail = datalen - cpylen;
		} else {
			memcpy(&m_uart[uart_id].
			       p_receive_pool[m_uart[uart_id].
					      n_receive_pool_tail], data,
			       datalen);
		}

		m_uart[uart_id].n_receive_pool_tail &=
		    (m_uart[uart_id].n_receive_pool_length - 1);
	}
}

void uart_timeout_handler(unsigned char uart_id)
{

	unsigned char buf_id = 9 + (uart_id << 1);
	volatile unsigned int rxcount = 0, nbr_to_read, timeout_count = 0;
	unsigned int baseAddress, status;
	unsigned int bufferAddress;
	unsigned char *p_buffer;
	//unsigned int i,value;
	volatile unsigned int value;
	//unsigned int delay_clk;

	if (m_uart[uart_id].b_open == AK_FALSE)
		return;

	if (m_uart[uart_id].b_interrupt == AK_FALSE)
		return;

	bufferAddress = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001000
	baseAddress = uart_id2register(uart_id);

	value = __raw_readl(baseAddress + 0x00);	//0x20026000,enable timeout function
	value |= (1UL << 23);
	__raw_writel(value, baseAddress + 0x00);
	//us_delay(m_uart[ uart_id ].timeout_delay_us);
	value = __raw_readl(baseAddress + 0x00);	//0x20026000, disable timeout function
	value &= ~(1UL << 23);
	__raw_writel(value, baseAddress + 0x00);

	status = __raw_readl(baseAddress + 0x04);
	if (status & (1UL << 2)) {
		uart_clear_rx_timeout(uart_id);
		value = __raw_readl(baseAddress + 0x08);	//0x20026008
		//timeout_count = (value>>23)&0x03;
		//0x200260i08
		//bit[24:23]:byt_left: the number of data left when receiving timeout occurs
		//      Note:Please refer to bit[23] of UART_CFG_REG(n)(0x20026000) for the 
		//           definition of timeout function
		timeout_count = (value >> 23) & 0x03;
		//if (uart_id == uiUART2)     
		//  printk("<%d>", timeout_count);
	} else
		return;

	nbr_to_read = (__raw_readl(baseAddress + 0x08) >> 13) & 0x1f;	//0x20026008
	//bit[17:13](0x20026008):RX_adr

	//printk("<%d>", nbr_to_read);   

	p_buffer =
	    (unsigned char *)(bufferAddress +
			      (m_uart[uart_id].rxfifo_offset << 2));

	//REG32(L2_FRAC_ADDR) |= (1<<29); //enable ahb effect
	if (nbr_to_read > m_uart[uart_id].rxfifo_offset) {
		rxcount = (nbr_to_read - m_uart[uart_id].rxfifo_offset) << 2;
		if (timeout_count)
			rxcount = rxcount - 4 + timeout_count;
		//uart_storedata(uart_id, pbuffer, rxcount);
		uart_storedata(uart_id, p_buffer, rxcount);
	} else {
		rxcount =
		    (UART_RX_FIFO_SIZE - m_uart[uart_id].rxfifo_offset) << 2;
		if (timeout_count && nbr_to_read == 0)
			rxcount = rxcount - 4 + timeout_count;
		uart_storedata(uart_id, p_buffer, rxcount);
		if (nbr_to_read > 0) {
			p_buffer = (unsigned char *)bufferAddress;
			rxcount = (nbr_to_read) << 2;
			if (timeout_count)
				rxcount = rxcount - 4 + timeout_count;
			uart_storedata(uart_id, p_buffer, rxcount);
		}
	}
	//REG32(L2_FRAC_ADDR) &= ~(1<<29); //disable ahb effect

	m_uart[uart_id].rxfifo_offset = nbr_to_read % UART_RX_FIFO_SIZE;

	if (m_uart[uart_id].b_interrupt == AK_TRUE) {
		if (rxcount != 0) {
			if (m_uart[uart_id].callback_func != AK_NULL)
				m_uart[uart_id].callback_func();
		}
	}
}

unsigned int uart_get_int_status(unsigned char uart_id, unsigned test_status)
{
	volatile unsigned int base_address, status;

	base_address = uart_id2register(uart_id);	//0x20026000

	status = __raw_readl(base_address + 0x04);
	switch (test_status) {
	case UART_RX_buf_full:
		return (status & (1UL << 1));
		break;
	case UART_INT_timeout:
		return (status & (1UL << 2));
		break;
	case UART_R_err:
		return (status & (1UL << 3));
		break;
	case UART_RX_ov:
		return (status & (1UL << 18));
		break;
	case UART_TX_end:
		return (status & (1UL << 19));
		break;
	case UART_Rx_th_int_sta:
		return (status & (1UL << 30));
		break;
	case UART_Tx_th_int_sta:
		return (status & (1UL << 31));
		break;
	default:
		break;
	}

	return status;
}

/**
 * @brief UART interrupt handler
 * If chip detect that UART0 received data, this function will be called.
 * This function will get UART data from UART Receive Data Hold Register, and call
 * UART callback function to process the data if it is available.
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-16
 * @param T_VOID
 * @return T_VOID
 * @retval
 */
//static T_VOID uart_handler(T_UART_ID uart_id)
static void uart_handler(unsigned char uart_id)
{
	unsigned char buf_id = 9 + (uart_id << 1);
	volatile unsigned int rxcount = 0, nbr_to_read, timeout_count = 0;
	volatile unsigned int baseAddress;
	volatile unsigned int bufferAddress, value, i;
	unsigned char *p_buffer;

	bufferAddress = (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  + ((buf_id - 8) << 7);	//0x48001000

	baseAddress = uart_id2register(uart_id);	//0x20026000

#if 0
	status = __raw_readl(baseAddress + 0x04);
	if (status & (1 << 3)) {
		//putch('#');
		reg = __raw_readl(baseAddress + 0x04);
		reg &= 0x3fe00000;
		reg |= (1 << 3);	//clear R_err
		__raw_writel(reg, baseAddress + 0x04);
	}
#endif
	uart_clear_rx_err(uart_id);

#if 0
	//rx threshold interrupt
	if (status & (1 << 30)) {

		//clear rx th status
		value = __raw_readl(baseAddress + 0x04);
		value |= (1 << 30);
		__raw_writel(value, baseAddress + 0x04);
		//uart_clear_rx_int();
	}
#endif
	uart_clear_rx_th(uart_id);

	nbr_to_read = (__raw_readl(baseAddress + 0x08) >> 13) & 0x1f;

	//timeout interrupt
#if 0
	if (status & (1 << 2)) {
		timeout_count = uart_clear_rx_timeout(uart_id);

		timeout_count = (__raw_readl(baseAddress + 0x08) >> 23) & 0x3;
		//printk("<%d>", timeout_count);
	}
#endif
	timeout_count = uart_clear_rx_timeout(uart_id);

	p_buffer =
	    (unsigned char *)(bufferAddress +
			      (m_uart[uart_id].rxfifo_offset << 2));

	if (nbr_to_read > m_uart[uart_id].rxfifo_offset) {
		rxcount = (nbr_to_read - m_uart[uart_id].rxfifo_offset) << 2;
		if (timeout_count)
			rxcount = rxcount - 4 + timeout_count;
		uart_storedata(uart_id, p_buffer, rxcount);
	} else {
		rxcount =
		    (UART_RX_FIFO_SIZE - m_uart[uart_id].rxfifo_offset) << 2;
		if (timeout_count && nbr_to_read == 0)
			rxcount = rxcount - 4 + timeout_count;
		for (i = 0; i < (rxcount >> 2); i++) {
			value = *(unsigned int *)(p_buffer + (i << 2));
			uart_storedata(uart_id, (unsigned char *)&value, 4);
		}
		if (nbr_to_read > 0) {
			p_buffer = (unsigned char *)bufferAddress;
			rxcount = (nbr_to_read) << 2;
			if (timeout_count)
				rxcount = rxcount - 4 + timeout_count;
			uart_storedata(uart_id, p_buffer, rxcount);
			/*for (i=0; i<(rxcount>>2); i++)
			   {
			   value = *(T_U32*)(pbuffer + (i<<2));
			   uart_storedata(uart_id, (T_U8*)&value, 4);    
			   } */
		}
	}

	//if (uiUART0 != uart_id)
	//akprintf(C3, M_DRVSYS, "[%d, %d, %d, %d]", m_uart[uart_id].rxfifo_offset, nbr_to_read, timeout_count, rxcount);

	m_uart[uart_id].rxfifo_offset = nbr_to_read % UART_RX_FIFO_SIZE;

	if (rxcount != 0) {
		if (m_uart[uart_id].callback_func != AK_NULL)
			m_uart[uart_id].callback_func();
	}

	return;
}

unsigned char uart0_interrupt_handler(void)
{
	uart_handler(uiUART0);
	return AK_TRUE;
}

#ifdef CHIP_780X
unsigned char uart1_interrupt_handler(void)
{
	uart_handler(uiUART1);
	return AK_TRUE;
}

static unsigned char uart2_interrupt_handler(void)
{
	uart_handler(uiUART2);
	return AK_TRUE;
}
#endif

#if 0
static unsigned char uart3_interrupt_handler(void)
{
	uart_handler(uiUART3);
	return AK_TRUE;
}
#endif

/**
 * @brief Close UART
 * Function uart_init() must be called before call this function
 * @author Junhua Zhao
 * @date 2005-05-18
 * @param T_UART_ID uart_id: UART ID
 * @return T_VOID
 * @retval
 */
void uart_free(unsigned char uart_id)
{
	unsigned int baseAddress;

	baseAddress = uart_id2register(uart_id);

	/* disable uart interrupt */
	__raw_writel(0, baseAddress + 0x04);	//mask all interrupt

	m_uart[uart_id].b_open = AK_FALSE;

	uart_clock_ctl(uart_id, AK_FALSE);

}

unsigned char uart_isrxfifoempty(unsigned char uart_id)
{
	return AK_TRUE;
}

unsigned char uart_istxfifoempty(unsigned char uart_id)
{
	return AK_TRUE;
}

unsigned char uart_setflowcontrol(unsigned char uart_id, unsigned char enable)
{
	unsigned int baseAddress = 0;
	volatile unsigned int value;

	if (uart_id == uiUART0)
		return AK_FALSE;

	baseAddress = uart_id2register(uart_id);

	if (AK_TRUE == enable) {
		value = __raw_readl(baseAddress + 0x00);
		//0x20026000
		//bit[19]:RTS_sel:( RTS:Request to Send (DCE -> DTE)
		//                0=to output the RTS signal directly
		//                1=to output the RTS signal inversely.
		//
		//bit[18]:CTS_sel:(CTS:Clear to Send (DCE -> DTE)
		//
		//bit[17]:URD_sel:(URD: URDx pin, UART Receive pin,UART receive serial data via the URDx pin)
		//bit[16]:UTD_sel:(UTD: UTDx pin, UART Transmit pin,UART transmit serial data via the UTDx pin)
		//
		//
		value |= (1UL << 19) | (1UL << 18);
		__raw_writel(value, baseAddress + 0x00);
	} else {
		//REG32(baseAddress+UART_CFG_REG1) &= ~(1<<19);
		//REG32(baseAddress+UART_CFG_REG1) &= ~(1<<18);
		value = __raw_readl(baseAddress + 0x00);
		value &= ~(1UL << 19);
		value &= ~(1UL << 18);
		__raw_writel(value, baseAddress + 0x00);
	}

	return AK_TRUE;
}

#if 0
static int uart_timer_id = -1;

static void timer_uart_cb(int timer_id, unsigned int delay)
{
	if (uart_timer_id == timer_id) {
		//uart_timeout_handler(uiUART0);
#ifdef CHIP_780X
		uart_timeout_handler(uiUART1);
		uart_timeout_handler(uiUART2);
#endif
		uart_timeout_handler(uiUART3);
	}
}
#endif

void uart_set_baudrate(unsigned char uart_id, unsigned int baud_rate,
		       unsigned int sys_clk)
{
	volatile unsigned int br_value;
	volatile unsigned int reg_value;
	unsigned int baseAddress;

	if (uart_id > MAX_UART_NUM)
		return;

	baseAddress = uart_id2register(uart_id);

	reg_value = __raw_readl(baseAddress + 0x00);
	reg_value &= 0xffff0000;

	/* set baudrate */
	br_value = sys_clk / baud_rate - 1;
	reg_value |= (br_value & 0xffff);

	if (sys_clk % baud_rate)
		reg_value |= (1UL << 22);

	__raw_writel(reg_value, baseAddress + 0x00);

}
