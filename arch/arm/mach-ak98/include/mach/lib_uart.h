/**
 * @file uart.h
 * @brief UART driver header file
 *
 * This file provides UART APIs: UART initialization, write data to UART, read data from
 * UART, register callback function to handle data from UART, and interrupt handler.
 * Copyright (C) 2005 Anyka (GuangZhou) Software Technology Co., Ltd.
 * @author ZouMai
 * @date 2005-07-14
 * @version 1.0
 */

#ifndef __ARCH_ARM_MACH_AK88_LIB_UART_H__
#define __ARCH_ARM_MACH_AK88_LIB_UART_H__

#include <asm/io.h>
#include <mach/map.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <mach/regs-uart.h>


//#include "common-regs.h"
/**
 * @brief UART port define
 *	define port name with port number
 */
typedef enum {
	uiUART0 = 0,
#ifdef CHIP_780X
	uiUART1,
	uiUART2,
#endif
	uiUART3,

	MAX_UART_NUM		/* UART number */
} T_UART_ID;

typedef enum {
	UART_RX_buf_full = 1,
	UART_INT_timeout,
	UART_R_err,
	UART_RX_ov,
	UART_TX_end,
	UART_Rx_th_int_sta,
	UART_Tx_th_int_sta,
} UART_STATU_REG04;

#define MODULE_UART uiUART0

//#define uart_id2register(uart_id/*0~3*/)  (unsigned int) (AK98_UART_BASE+(unsigned int)(uart_id/*0~3*/)*0x1000)  //0x20026000

static inline char ak880x_uart_get_tx_rdy(unsigned char uart_id)
{
	unsigned int status;
	//status=__raw_readl(uart_id2register(uart_id)+0x04);

	status = __raw_readl(AK98_UART_CFG_REG2(uart_id));

	if (status & (1 << 19))
		return 1;
	else
		return 0;
}

static inline char ak880x_uart_get_rx_rdy(unsigned char uart_id)
{
	unsigned int status;
	//status=__raw_readl(uart_id2register(uart_id)+0x04);
	status = __raw_readl(AK98_UART_CFG_REG2(uart_id));

	if (status & (1 << 2))
		return 1;
	else if (status & (1 << 1))
		return 1;
	else
		return 0;

	//0x20026004
	//bit[2]:timeout:1=the receiving timeout occurs.
	//bit[1]:RX_buf_full:1=the receive buffer is full.
}

void uart_clock_ctl(unsigned char uart_id, unsigned char enable);
void uart_pin_ctl(unsigned char uart_id);

unsigned int uart_get_int_status(unsigned char uart_id, unsigned test_status);

void uart_clear_tx_status(unsigned char uart_id);
void uart_clear_rx_status(unsigned char uart_id);
unsigned int uart_clear_rx_timeout(unsigned char uart_id);	//return timeout_count
void uart_clear_rx_buffull(unsigned char uart_id);
void uart_clear_rx_err(unsigned char uart_id);

void uart_clear_rx_th(unsigned char uart_id);
void uart_reen_rx_th(unsigned char uart_id);

unsigned char uart_wait_tx_finish(unsigned char uart_id);
unsigned char uart_get_tx_empty(unsigned char uart_id);
/*yes:return 1;  no :return 0;*/
unsigned char uart_wait_rx_timeout(unsigned char uart_id);
unsigned char uart_get_rx_timeout(unsigned char uart_id);
/*yes:return 1;  no :return 0;*/
unsigned char uart_get_rx_buffull(unsigned char uart_id);
/*yes:return 1;  no :return 0;*/

unsigned int uart_read_timeout(unsigned char uart_id, unsigned char *chr);
/*when timeout is occur,read all the rxfifo data,return real read number*/

unsigned int uart_read_buffull(unsigned char uart_id, unsigned char *chr);
/*when RX_buf_full is occur,read all the rxfifo data,return real read number*/

/**
 * @brief UART callback define
 *	define UART callback type
 */
//typedef T_VOID (*T_fUART_CALLBACK)(T_VOID);
typedef void (*t_fuart_callback) (void);

/**
 * @brief Initialize UART
 *
 * Initialize UART base on UART ID, baudrate and system clock. If user want to change
 * baudrate or system clock is changed, user should call this function to initialize
 * UART again.
 * Function uart_init() must be called before call any other UART functions
 * @author ZouMai
 * @date 2005-07-13
 * @param[in] unsigned char uart_id: UART ID
 * @param[in] unsigned int baud_rate: Baud rate, use UART_BAUD_9600, UART_BAUD_19200 ...
 * @return unsigned char: Init UART OK or not
 * @retval AK_TRUE: Successfully initialized UART. AK_FALSE: Initializing UART failed.
 */

unsigned char uart_init(unsigned char uart_id, unsigned int baud_rate,
			unsigned int sys_clk);

//unsigned char uart_enable_int(unsigned char uart_id/*0~3*/);

extern void uart_close_interrupt(unsigned char uart_id /*0~3 */ );
extern void uart_open_interrupt(unsigned char uart_id /*0~3 */ );

void uart_free(unsigned char uart_id);

/**
 * @brief Write one character to UART base on UART ID
 *
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param[in] unsigned char uart_id: UART ID
 * @param[in] unsigned char chr: The character which will be written to UART
 * @return unsigned char: Write character OK or not
 * @retval AK_TRUE: Successfully written character to UART. AK_FALSE: Writing character to UART failed.
 */

unsigned char uart_write_chr(unsigned char uart_id, unsigned char chr);

/**
 * @brief Write string to UART base on UART ID
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param unsigned char uart_id: UART ID
 * @param unsigned char *str: The string which will be written to UART
 * @return unsigned int: Length of the data which have been written to UART
 * @retval
 */

unsigned int uart_write_str(unsigned char uart_id, unsigned char *str);

unsigned char uart_write_buf(unsigned char uart_id /*0~3 */ ,
			     unsigned char *chr,
			     unsigned int byte_nbr
			     /*<60,last 4 bytes(0x3c/0x7c) write 0 */ );

/**
 * @brief Write string data to UART
 *
 * Write data to UART base on UART ID and data length
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-16
 * @param[in] unsigned char uart_id: UART ID
 * @param[in] const unsigned char* data: Constant data to be written to UART, this data needn't be end with '\0'
 * @param[in] unsigned int data_len: Data length
 * @return unsigned int
 * @retval Length of the data which have been written to UART
 */
unsigned int uart_write(unsigned char uart_id, const unsigned char *data,
			unsigned int data_len);

unsigned int uart_write_dma(unsigned char uart_id, const unsigned char *chr,
			    unsigned int byte_nbr);

/**
 * @brief Read a character from UART
 *
 * This function will not return until get a character from UART
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param[in] unsigned char uart_id: UART ID
 * @param[out] unsigned char *chr: character for return
 * @return unsigned char: Got character or not
 * @retval return AK_TRUE
 */
unsigned char uart_read_chr(unsigned char uart_id, unsigned char *chr);

/**
 * @brief Register a callback function to process UART received data.
 *
 * This function words only in the UART interrupt mode.
 * Caution: The macro definition "__ENABLE_UARTxx_INT__" must be defined. 
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param[in] unsigned char uart_id: UART ID
 * @param[in] t_fuart_callback callback_func: Callback function
 * @return void
 * @retval
 */

void uart_set_callback(unsigned char uart_id, t_fuart_callback callback_func);

/**
 * @brief Register a callback function to process UART received data.
 *
 * This function words only in the UART interrupt mode.
 * Caution: The macro definition "__ENABLE_UARTxx_INT__" must be defined. 
 * Function uart_init() must be called before call this function
 * @author ZouMai
 * @date 2004-09-17
 * @param[in] unsigned char uart_id: uart_id
 * @param[in] t_fuart_callback callback_func: Callback function
 * @return void
 * @retval
 */
unsigned char uart_read_chr_asy(unsigned char uart_id, unsigned char *chr);

/*@}*/

#endif
