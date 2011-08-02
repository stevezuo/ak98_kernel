/*
 * arch/arm/mach-ak98/include/mach/uart.h
 */
#ifndef __UART_H_
#define __UART_H_

#include <mach/map.h>


#define AK98_VA_UART_REG(n)	     	(AK98_VA_DEV + 0x6000 + 0x1000 * (n)) //n = 0 ~ 3
#define AK98_PA_UART_REG(n) 	    (AK98_PA_DEV + 0x6000 + 0x1000 * (n)) //n = 0 ~ 3

/********************* uart1~4 ****************************/
//n = 0 ~ 3
/*
#define rUART_CON1(n)          REG_VA_VAL(AK98_VA_UART_REG(n), 0x00)
#define rUART_CON2(n)          REG_VA_VAL(AK98_VA_UART_REG(n), 0x04)
#define rUART_DATACON(n)	    REG_VA_VAL(AK98_VA_UART_REG(n), 0x08)
#define rUART_TXRXBUF(n)      	REG_VA_VAL(AK98_VA_UART_REG(n), 0x0C)
#define rUART_RXBUF(n)         REG_VA_VAL(AK98_VA_UART_REG(n), 0x10)
#define rUART_BCK_RXBUF(n)    	REG_VA_VAL(AK98_VA_UART_REG(n), 0x14)
#define rUART_SBIT_TIMEOUT(n) 	REG_VA_VAL(AK98_VA_UART_REG(n), 0x18)
*/

/******************** uart1 *******************************/
#define rUART1_CON1             REG_VA_VAL(AK98_VA_UART_REG(0), 0x00)
#define rUART1_CON2             REG_VA_VAL(AK98_VA_UART_REG(0), 0x04)
#define rUART1_DATACON	        REG_VA_VAL(AK98_VA_UART_REG(0), 0x08)
#define rUART1_TXRXBUF         	REG_VA_VAL(AK98_VA_UART_REG(0), 0x0C)
#define rUART1_RXBUF            REG_VA_VAL(AK98_VA_UART_REG(0), 0x10)
#define rUART1_BCK_RXBUF       	REG_VA_VAL(AK98_VA_UART_REG(0), 0x14)
#define rUART1_SBIT_TIMEOUT 	REG_VA_VAL(AK98_VA_UART_REG(0), 0x18)

/******************** uart2 *******************************/
#define rUART2_CON1             REG_VA_VAL(AK98_VA_UART_REG(1), 0x00)
#define rUART2_CON2             REG_VA_VAL(AK98_VA_UART_REG(1), 0x04)
#define rUART2_DATACON        	REG_VA_VAL(AK98_VA_UART_REG(1), 0x08)
#define rUART2_TXRXBUF        	REG_VA_VAL(AK98_VA_UART_REG(1), 0x0C)
#define rUART2_RXBUF            REG_VA_VAL(AK98_VA_UART_REG(1), 0x10)
#define rUART2_BCK_RXBUF      	REG_VA_VAL(AK98_VA_UART_REG(1), 0x14)
#define rUART2_SBIT_TIMEOUT 	REG_VA_VAL(AK98_VA_UART_REG(1), 0x18)

/******************** uart3 *******************************/
#define rUART3_CON1             REG_VA_VAL(AK98_VA_UART_REG(2), 0x00)
#define rUART3_CON2             REG_VA_VAL(AK98_VA_UART_REG(2), 0x04)
#define rUART3_DATACON        	REG_VA_VAL(AK98_VA_UART_REG(2), 0x08)
#define rUART3_TXRXBUF         	REG_VA_VAL(AK98_VA_UART_REG(2), 0x0C)
#define rUART3_RXBUF            REG_VA_VAL(AK98_VA_UART_REG(2), 0x10)
#define rUART3_BCK_RXBUF       	REG_VA_VAL(AK98_VA_UART_REG(2), 0x14)
#define rUART3_SBIT_TIMEOUT		REG_VA_VAL(AK98_VA_UART_REG(2), 0x18)

/******************** uart4 *******************************/
#define rUART4_CON1             REG_VA_VAL(AK98_VA_UART_REG(3), 0x00)
#define rUART4_CON2             REG_VA_VAL(AK98_VA_UART_REG(3), 0x04)
#define rUART4_DATACON       	REG_VA_VAL(AK98_VA_UART_REG(3), 0x08)
#define rUART4_TXRX_BUF         REG_VA_VAL(AK98_VA_UART_REG(3), 0x0C)
#define rUART4_RXBUF            REG_VA_VAL(AK98_VA_UART_REG(3), 0x10)
#define rUART4_BCK_RXBUF       	REG_VA_VAL(AK98_VA_UART_REG(3), 0x14)
#define rUART4_SBIT_TIMEOUT  	REG_VA_VAL(AK98_VA_UART_REG(3), 0x18)


/************************** UART *************************************/
#define AK98_UART_CFG_REG1(n)       (AK98_VA_DEV+0x6000+(n)*0x1000)   //n=0 to 3   // 0x20026000,0x20027000,0x20028000,0x20029000
#define AK98_UART_CFG_REG2(n)       (AK98_VA_DEV+0x6004+(n)*0x1000)   //n=0 to 3   // 0x20026004,0x20027004,0x20028004,0x20029004
#define AK98_UART_DATA_CFG(n)       (AK98_VA_DEV+0x6008+(n)*0x1000)   //n=0 to 3   // 0x20026008,0x20027008,0x20028008,0x20029008
#define AK98_UART_THREHOLD(n)       (AK98_VA_DEV+0x600C+(n)*0x1000)   //n=0 to 3   // 0x2002600C,0x2002700C,0x2002800C,0x2002900C
#define AK98_UART_RX_DATA(n)        (AK98_VA_DEV+0x6010+(n)*0x1000)   //n=0 to 3   // 0x20026010,0x20027010,0x20028010,0x20029010


#define UART_BOOT_SET

#endif /* __UART_H_ */




