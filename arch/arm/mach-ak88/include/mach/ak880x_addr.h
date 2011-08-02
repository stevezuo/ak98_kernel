/*
*   Filename:	linux/arch/arm/mach-ak880x/include/mach/ak880x_addr.h
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*/

#ifndef _AK88_ADDR_H_
#define _AK88_ADDR_H_

#include <mach/map.h>

#define AK88_SYSCTRL_REG(x)		(AK88_VA_SYSCTRL + (x))
#define AK88_DISPLAY_REG(x)		(AK88_VA_DISPLAY + (x))
#define AK88_MMC_REG(x)		(AK88_VA_MMC + (x))
#define AK88_SDIO_REG(x)		(AK88_VA_SDIO + (x))
#define AK88_SPI1_REG(x)		(AK88_VA_SPI0 + (x))
#define AK88_SPI2_REG(x)		(AK88_VA_SPI0 + 0x1000 + (x))
#define AK88_UART1_REG(x)		(AK88_VA_UART	+ (x))
#define AK88_UART2_REG(x)		(AK88_VA_UART	+ 0x1000 + (x))
#define AK88_UART3_REG(x)		(AK88_VA_UART	+ 0x2000 + (x))
#define AK88_UART4_REG(x)		(AK88_VA_UART	+ 0x3000 + (x))
#define AK88_NFCTRL_REG(x)		(AK88_VA_NFCTRL + (x))
#define AK88_ECC_REG(x)		(AK88_VA_NFCTRL + 0x1000 + (x))
#define AK88_L2CTRL_REG(x)		(AK88_VA_L2CTRL + (x))
#define AK88_RAMCTRL_REG(x)		(AK88_VA_RAMCTRL + (x))
#define AK88_DACCTRL_REG(x)		(AK88_VA_DACCTRL + (x))
#define AK88_CAMIF_REG(x)		(AK88_VA_CAMIF + (x))

/* Chip ID register */
#define rCHIP_ID	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x00))	/*Chip ID register */

/* System control registers */
#define rCLK_DIV1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x04))	/*Clock divider register 1 */
#define rCLK_DIV2 	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x08))	/*Clock divider register 2 */
#define rCLK_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x0C))	/*Clock control and soft rest control register */
#define rN_CON		(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xDC))	/*N configuration register */
#define rMULFUN_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x58))	/*Multiple function control register */

#define rIRQ_MASK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x34))	/*Interrupt mask register for IRQ */
#define rFIQ_MASK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x38))	/*Interrupt mask register for FIQ */
#define rINT_STAT	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xCC))	/*Interrupt status register */
#define rINT_STATEN	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x4C))	/*Interrupt Enable/status register of system control module */

#define rWKUPGPIO_POL	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x3C))	/*Wake-up GPIO polarity selection */
#define rWKUPGPIO_CLR	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x40))	/*Clear wake-up GPIO status */
#define rWKUPGPIO_EN	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x44))	/*Enabling the wake-up function of corresponding wake-up GPIOS */
#define rWKUPGPIO_STAT	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x48))	/*Wake-up GPIO status register, displaying current wake-up GPIO status */

#define rRTCUSB_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x50))	/*RTC configuratio and USB control register */
#define rRTC_BOOTMOD	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x54))	/*RTC read back and bootup mode register */

#define rSHAREPIN_CON1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x78))	/*Shared-pin control register 1 */
#define rSHAREPIN_CON2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x74))	/*Shared-pin control register 2 */
#define rPPU_PPD1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x9C))	/*Programmable Pull-ups/Pull-downs register 1 */
#define rPPU_PPD2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xA0))	/*Programmable Pull-ups/Pull-downs register 2 */
#define rPPU_PPD3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xA4))	/*Programmable Pull-ups/Pull-downs register 3 */
#define rPPU_PPD4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xA8))	/*Programmable Pull-ups/Pull-downs register 4 */

 /*CRC*/
#define rCRC_POLYLEN	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xAC))	/*Polynomial's length and start CRC */
#define rCRC_COEFCON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xB0))	/*Coefficient configuration */
#define rCRC_RESULT	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xD0))	/*CRC result */
#define rIO_CON1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xD4))	/*IO control register 1 */
#define rIO_CON2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xD8))	/*IO control register 2 */
/* Analog control */
#define rANALOG_CON1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x5C))
#define rADC1_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x60))
#define rADC2_CON	(*(volatile unsigned long *)(AK88_VA_SUBCTRL + 0x72000))
#define rADC2_DAT	(*(volatile unsigned long *)(AK88_VA_SUBCTRL + 0x72004))
#define rANALOG_CON2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x64))
#define rTS_XVAL	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x68))
#define rTS_YVAL	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x6C))
#define rADC1_STAT	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x70))
/* PWM */
#define rPWM1_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x2C))
#define rPWM2_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x30))
#define	rPWM3_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xB4))
#define rPWM4_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xB8))
/* Timers */
#define rTIMER1_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x18))
#define rTIMER2_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x1C))
#define rTIMER3_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x20))
#define rTIMER4_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x24))
#define rTIMER5_CON	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x28))
#define rTIMER1_RDBACK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x100))
#define rTIMER2_RDBACK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x104))
#define rTIMER3_RDBACK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x108))
#define rTIMER4_RDBACK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x10C))
#define rTIMER5_RDBACK	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x110))
/* GPIOS */
#define rGPIO_DIR1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x7C))
#define rGPIO_DIR2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x84))
#define rGPIO_DIR3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x8C))
#define rGPIO_DIR4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x94))
#define rGPIO_OUT1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x80))
#define rGPIO_OUT2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x88))
#define rGPIO_OUT3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x90))
#define rGPIO_OUT4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0x98))
#define rGPIO_IN1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xBC))
#define rGPIO_IN2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xC0))
#define rGPIO_IN3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xC4))
#define rGPIO_IN4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xC8))
#define rGPIO_INT1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xE0))
#define rGPIO_INT2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xE4))
#define rGPIO_INT3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xE8))
#define rGPIO_INT4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xEC))
#define rGPIO_INTPOL1	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xF0))
#define rGPIO_INTPOL2	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xF4))
#define rGPIO_INTPOL3	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xF8))
#define rGPIO_INTPOL4	(*(volatile unsigned long *)AK88_SYSCTRL_REG(0xFC))
/* L2 controller */
#define rL2_ADDRBUF0	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x00))	/*DMA address information buffer0~buffer15 */
#define	rL2_ADDRBUF1	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x04))
#define	rL2_ADDRBUF2	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x08))
#define	rL2_ADDRBUF3	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x0C))
#define	rL2_ADDRBUF4	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x10))
#define	rL2_ADDRBUF5	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x14))
#define	rL2_ADDRBUF6	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x18))
#define	rL2_ADDRBUF7	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x1C))
#define	rL2_ADDRBUF8	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x20))
#define	rL2_ADDRBUF9	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x24))
#define	rL2_ADDRBUF10	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x28))
#define	rL2_ADDRBUF11	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x2C))
#define	rL2_ADDRBUF12	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x30))
#define	rL2_ADDRBUF13	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x34))
#define	rL2_ADDRBUF14	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x38))
#define	rL2_ADDRBUF15	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x3C))
#define rL2_CONBUF0	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x40))	/*DMA opeeration times buufer0~buffer15 */
#define rL2_CONBUF1	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x44))
#define rL2_CONBUF2	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x48))
#define rL2_CONBUF3	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x4C))
#define rL2_CONBUF4	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x50))
#define rL2_CONBUF5	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x54))
#define rL2_CONBUF6	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x58))
#define rL2_CONBUF7	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x5C))
#define rL2_CONBUF8	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x60))
#define rL2_CONBUF9	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x64))
#define rL2_CONBUF10	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x68))
#define rL2_CONBUF11	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x6C))
#define rL2_CONBUF12	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x70))
#define rL2_CONBUF13	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x74))
#define rL2_CONBUF14	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x78))
#define rL2_CONBUF15	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x7C))
#define rL2_DMAREQ	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x80))	/*DMA request configuration */
#define rL2_FRACDMAADDR	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x84))	/*Fraction DMA address information */
#define rL2_CONBUF0_7	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x88))	/*Buffer0~buffer7 configuration */
#define rL2_CONBUF8_15	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x8C))	/*CPU-controlled buffer and buffer8~buffer15 configuration */
#define rL2_BUFASSIGN1	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x90))	/*Buffer assignment */
#define rL2_BUFASSIGN2	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x94))	/*Buffer assignment */
#define rL2_LDMACON	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x98))	/*Configuration of LDMA */
#define rL2_BUFINTEN	(*(volatile unsigned long *)AK88_L2CTRL_REG(0x9C))	/*This register enables/disables the buffer interrupts */
#define rL2_BUFSTAT1	(*(volatile unsigned long *)AK88_L2CTRL_REG(0xA0))	/*Buffer status register 1 */
#define rL2_BUFSTAT2	(*(volatile unsigned long *)AK88_L2CTRL_REG(0xA8))	/*Buffer status register 2 */
#define rCRC_CON	(*(volatile unsigned long *)AK88_L2CTRL_REG(0xA4))	/*CRC configuration register */
/* RAM controller */
#define rMEM_CON1	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x00))	/*RAM controller configuration register 1 */
#define rMEM_CON2	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x04))	/*RAM controller configuration register 2 */
#define rMEM_CON3	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x08))	/*RAM controller configuration register 3 */
#define rDMA_PRI1	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x0C))	/*DMA priority configuration register 1 */
#define rDMA_PRI2	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x10))	/*DMA priority configuration register 2 */
#define rAHB_PRI	(*(volatile unsigned long *)AK88_RAMCTRL_REG(0x14))	/*AHB priority configuration register */
/* Nand flash controller */
#define rNFC_COMM1	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x100))	/*Nand flash command register 1 */
#define rNFC_COMM2	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x104))	/*Nand flash command register 2 */
#define rNFC_COMM3	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x108))	/*Nand flash command register 3 */
#define rNFC_COMM4	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x10C))	/*Nand flash command register 4 */
#define rNFC_COMM5	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x110))	/*Nand flash command register 5 */
#define rNFC_COMM6	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x114))	/*Nand flash command register 6 */
#define rNFC_COMM7	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x118))	/*Nand flash command register 7 */
#define rNFC_COMM8	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x11C))	/*Nand flash command register 8 */
#define rNFC_COMM9	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x120))	/*Nand flash command register 9 */
#define rNFC_COMM10	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x124))	/*Nand flash command register 10 */
#define rNFC_COMM11	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x128))	/*Nand flash command register 11 */
#define rNFC_COMM12	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x12C))	/*Nand flash command register 12 */
#define rNFC_COMM13	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x130))	/*Nand flash command register 13 */
#define rNFC_COMM14	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x134))	/*Nand flash command register 14 */
#define rNFC_COMM15	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x138))	/*Nand flash command register 15 */
#define rNFC_COMM16	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x13C))	/*Nand flash command register 16 */
#define rNFC_COMM17	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x140))	/*Nand flash command register 17 */
#define rNFC_COMM18	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x144))	/*Nand flash command register 18 */
#define rNFC_COMM19	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x148))	/*Nand flash command register 19 */
#define rNFC_COMM20	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x14C))	/*Nand flash command register 20 */
#define rNFC_STAT1	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x150))	/*Nand flash status register 1 */
#define rNFC_STAT2	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x154))	/*Nand flash status register 2 */
#define rNFC_CONSTAT	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x158))	/*Nand flash control/status register */
#define rNFC_COMMLEN	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x15C))	/*Nand flash command length */
#define rNFC_DATALEN	(*(volatile unsigned long *)AK88_NFCTRL_REG(0x160))	/*Nand flash data length */
/* ECC sub-module controller */
#define rECC_CON	(*(volatile unsigned long *)AK88_ECC_REG(0x00))	/*ECC control register */
#define rECC_ERRPOS1	(*(volatile unsigned long *)AK88_ECC_REG(0x04))	/*Error posision register 1 */
#define rECC_ERRPOS2	(*(volatile unsigned long *)AK88_ECC_REG(0x08))	/*Error posision register 2 */
#define rECC_ERRPOS3	(*(volatile unsigned long *)AK88_ECC_REG(0x0C))	/*Error posision register 3 */
#define rECC_ERRPOS4	(*(volatile unsigned long *)AK88_ECC_REG(0x10))	/*Error posision register 4 */
#define rECC_ERRPOS5	(*(volatile unsigned long *)AK88_ECC_REG(0x14))	/*Error posision register 5 */
#define rECC_ERRPOS6	(*(volatile unsigned long *)AK88_ECC_REG(0x18))	/*Error posision register 6 */
#define rECC_ERRPOS7	(*(volatile unsigned long *)AK88_ECC_REG(0x1C))	/*Error posision register 7 */
#define rECC_ERRPOS8	(*(volatile unsigned long *)AK88_ECC_REG(0x20))	/*Error posision register 8 */
/* DAC controller */
#define rDAC_CON	(*(volatile unsigned long *)AK88_DACCTRL_REG(0x00))	/*DAC configuration register */
#define rIIS_CON	(*(volatile unsigned long *)AK88_DACCTRL_REG(0x04))	/*IIS configuration register */
#define rDAC_CPUDATA	(*(volatile unsigned long *)AK88_DACCTRL_REG(0x08))	/*Data from CPU */
/* Camera controller */
#define rCAM_SENSORCOMM		(*(volatile unsigned long *)AK88_CAMIF_REG(0x00))	/*Image capturing command */
#define rCAM_IMAGEINF1		(*(volatile unsigned long *)AK88_CAMIF_REG(0x04))	/*Source/destination image horizontal length */
#define rCAM_IMAGEINF2		(*(volatile unsigned long *)AK88_CAMIF_REG(0x08))	/*Horizontal scalling information */
#define rCAM_IMAGEINF3		(*(volatile unsigned long *)AK88_CAMIF_REG(0x0C))	/*Source/Destination image vertical length */
#define rCAM_IMAGEINF4		(*(volatile unsigned long *)AK88_CAMIF_REG(0x10))	/*Horizontal scalling information */
#define rCAM_ODDYADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x18))	/*DMA starting address of external RAM for Y component of odd frame */
#define rCAM_ODDCbADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x1C))	/*DMA starting address of external RAM for Cb component of odd frame */
#define rCAM_ODDCrADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x20))	/*DMA starting address of external RAM for Cr component of odd frame */
#define rCAM_ODDRGBADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x24))	/*DMA starting address of external RAM for RGB/JPGE data of odd frame */
#define rCAM_EVENYADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x28))
#define rCAM_EVENCbADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x2C))
#define rCAM_EVENCrADDR		(*(volatile unsigned long *)AK88_CAMIF_REG(0x30))
#define rCAM_EVENRGBADDR	(*(volatile unsigned long *)AK88_CAMIF_REG(0x34))
#define rCAM_SENSORCON		(*(volatile unsigned long *)AK88_CAMIF_REG(0x40))	/*Image sensor configuration */
#define rCAM_FRAMESTAT		(*(volatile unsigned long *)AK88_CAMIF_REG(0x60))	/*Status of the current frame */
#define rCAM_FRAMELINE		(*(volatile unsigned long *)AK88_CAMIF_REG(0x80))	/*The line number of a frame when the input data is in JPEG-compressed format */
/* Display controller */
#define rLCD_COMM1		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x00))	/*LCD controller command register 1 */
#define rLCD_MPUIFCON		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x04))	/*MPU interface control */
#define rLCD_RSTSIG		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x08))
#define rLCD_MPURDBACK		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x0C))
#define rLCD_RGBIFCON1		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x10))
#define rLCD_RGBIFCON2		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x14))
#define rLCD_RGBPGSIZE		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x18))
#define rLCD_RGBPGOFFSET	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x1C))
#define rLCD_OSDADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x20))
#define rLCD_OSDOFFSET		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x24))
#define rLCD_OSDCOLOR1		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x28))
#define rLCD_OSDCOLOR2		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x2C))
#define rLCD_OSDCOLOR3		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x30))
#define rLCD_OSDCOLOR4		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x34))
#define rLCD_OSDCOLOR5		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xD0))
#define rLCD_OSDCOLOR6		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xD4))
#define rLCD_OSDCOLOR7		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xD8))
#define rLCD_OSDCOLOR8		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xDC))
#define rLCD_OSDSIZE		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x38))
#define rLCD_BACKCOLOR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x3C))
#define rLCD_RGBIFCON3		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x40))
#define rLCD_RGBIFCON4		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x44))
#define rLCD_RGBIFCON5		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x48))
#define rLCD_RGBIFCON6		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x4C))
#define rLCD_RGBIFCON7		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x50))
#define rLCD_RGBIFCON8		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x54))
#define rLCD_RGBIFCON9		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x58))
#define rLCD_Y1_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x5C))
#define rLCD_Cb1_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x60))
#define rLCD_Cr1_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x64))
#define rLCD_YCbCr1_HINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x68))
#define rLCD_YCbCr1_VINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x6C))
#define rLCD_YCbCr1_SCAL	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x70))
#define rLCD_YCbCr1_DISPINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x74))
#define rLCD_YCbCr1_PGSIZE	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x78))
#define rLCD_YCbCr1_PGOFFSET	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x7C))
#define rLCD_Y2_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x80))
#define rLCD_Cb2_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x84))
#define rLCD_Cr2_ADDR		(*(volatile unsigned long *)AK88_DISPLAY_REG(0x88))
#define rLCD_YCbCr2_HINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x8C))
#define rLCD_YCbCr2_VINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x90))
#define rLCD_YCbCr2_SCAL	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x94))
#define rLCD_YCbCr2_DISPINFO	(*(volatile unsigned long *)AK88_DISPLAY_REG(0x98))
#define rLCD_RGBOFFSET		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xA8))
#define rLCD_RGBSIZE		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xAC))
#define rLCD_DISPSIZE		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xB0))
#define rLCD_COMM2		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xB4))
#define rLCD_OP			(*(volatile unsigned long *)AK88_DISPLAY_REG(0xB8))
#define rLCD_STAT		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xBC))
#define rLCD_INTEN		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xC0))
#define rLCD_SOFTCON		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xC8))
#define rTV_IFCON		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xCC))
#define rLCD_CLKCON		(*(volatile unsigned long *)AK88_DISPLAY_REG(0xE8))
/* UART1~UART4 controllers */
#define rUART1_CON1		(*(volatile unsigned long *)AK88_UART1_REG(0x00))
#define rUART1_CON2		(*(volatile unsigned long *)AK88_UART1_REG(0x04))
#define rUART1_DATACON		(*(volatile unsigned long *)AK88_UART1_REG(0x08))
#define rUART1_TXRXBUF		(*(volatile unsigned long *)AK88_UART1_REG(0x0C))
#define rUART2_CON1		(*(volatile unsigned long *)AK88_UART2_REG(0x00))
#define rUART2_CON2		(*(volatile unsigned long *)AK88_UART2_REG(0x04))
#define rUART2_DATACON		(*(volatile unsigned long *)AK88_UART2_REG(0x08))
#define rUART2_TXRXBUF		(*(volatile unsigned long *)AK88_UART2_REG(0x0C))
#define rUART3_CON1		(*(volatile unsigned long *)AK88_UART3_REG(0x00))
#define rUART3_CON2		(*(volatile unsigned long *)AK88_UART3_REG(0x04))
#define rUART3_DATACON		(*(volatile unsigned long *)AK88_UART3_REG(0x08))
#define rUART3_TXRXBUF		(*(volatile unsigned long *)AK88_UART3_REG(0x0C))
#define rUART4_CON1		(*(volatile unsigned long *)AK88_UART4_REG(0x00))
#define rUART4_CON2		(*(volatile unsigned long *)AK88_UART4_REG(0x04))
#define rUART4_DATACON		(*(volatile unsigned long *)AK88_UART4_REG(0x08))
#define rUART4_TXRXBUF		(*(volatile unsigned long *)AK88_UART4_REG(0x0C))
/* SPI1 SPI2 Controllers */
#define rSPI1_CON		(*(volatile unsigned long *)AK88_SPI1_REG(0x00))
#define rSPI1_STAT		(*(volatile unsigned long *)AK88_SPI1_REG(0x04))
#define rSPI1_INTEN		(*(volatile unsigned long *)AK88_SPI1_REG(0x08))
#define rSPI1_COUNT		(*(volatile unsigned long *)AK88_SPI1_REG(0x0C))
#define rSPI1_TXBUF		(*(volatile unsigned long *)AK88_SPI1_REG(0x10))
#define rSPI1_RXBUF		(*(volatile unsigned long *)AK88_SPI1_REG(0x14))
#define rSPI1_OUTDATA		(*(volatile unsigned long *)AK88_SPI1_REG(0x18))
#define rSPI1_INDATA		(*(volatile unsigned long *)AK88_SPI1_REG(0x1C))
#define rSPI1_TIMEOUT		(*(volatile unsigned long *)AK88_SPI1_REG(0x20))
#define rSPI2_CON		(*(volatile unsigned long *)AK88_SPI2_REG(0x00))
#define rSPI2_STAT		(*(volatile unsigned long *)AK88_SPI2_REG(0x04))
#define rSPI2_INTEN		(*(volatile unsigned long *)AK88_SPI2_REG(0x08))
#define rSPI2_COUNT		(*(volatile unsigned long *)AK88_SPI2_REG(0x0C))
#define rSPI2_TXBUF		(*(volatile unsigned long *)AK88_SPI2_REG(0x10))
#define rSPI2_RXBUF		(*(volatile unsigned long *)AK88_SPI2_REG(0x14))
#define rSPI2_OUTDATA		(*(volatile unsigned long *)AK88_SPI2_REG(0x18))
#define rSPI2_INDATA		(*(volatile unsigned long *)AK88_SPI2_REG(0x1C))
#define rSPI2_TIMEOUT		(*(volatile unsigned long *)AK88_SPI2_REG(0x20))
/* MMC/SD controller */
#define rSD_CLKCON		(*(volatile unsigned long *)AK88_MMC_REG(0x04))
#define rSD_COMMARG		(*(volatile unsigned long *)AK88_MMC_REG(0x08))
#define rSD_COMM		(*(volatile unsigned long *)AK88_MMC_REG(0x0C))
#define rSD_COMMRESP		(*(volatile unsigned long *)AK88_MMC_REG(0x10))
#define rSD_RESP1		(*(volatile unsigned long *)AK88_MMC_REG(0x14))
#define rSD_RESP2		(*(volatile unsigned long *)AK88_MMC_REG(0x18))
#define rSD_RESP3		(*(volatile unsigned long *)AK88_MMC_REG(0x1C))
#define rSD_RESP4		(*(volatile unsigned long *)AK88_MMC_REG(0x20))
#define rSD_DATATIMER		(*(volatile unsigned long *)AK88_MMC_REG(0x24))
#define rSD_DATALEN		(*(volatile unsigned long *)AK88_MMC_REG(0x28))
#define rSD_DATACON		(*(volatile unsigned long *)AK88_MMC_REG(0x2C))
#define rSD_DATACOUNT		(*(volatile unsigned long *)AK88_MMC_REG(0x30))
#define rSD_STAT		(*(volatile unsigned long *)AK88_MMC_REG(0x34))
#define rSD_INTEN		(*(volatile unsigned long *)AK88_MMC_REG(0x38))
#define rSD_DMAMOD		(*(volatile unsigned long *)AK88_MMC_REG(0x3C))
#define rSD_CPUMOD		(*(volatile unsigned long *)AK88_MMC_REG(0x40))
/* SDIO controller */
#define rSDIO_CLKCON		(*(volatile unsigned long *)AK88_SDIO_REG(0x04))
#define rSDIO_COMMARG		(*(volatile unsigned long *)AK88_SDIO_REG(0x08))
#define rSDIO_COMM		(*(volatile unsigned long *)AK88_SDIO_REG(0x0C))
#define rSDIO_COMMRESP		(*(volatile unsigned long *)AK88_SDIO_REG(0x10))
#define rSDIO_RESP1		(*(volatile unsigned long *)AK88_SDIO_REG(0x14))
#define rSDIO_RESP2		(*(volatile unsigned long *)AK88_SDIO_REG(0x18))
#define rSDIO_RESP3		(*(volatile unsigned long *)AK88_SDIO_REG(0x1C))
#define rSDIO_RESP4		(*(volatile unsigned long *)AK88_SDIO_REG(0x20))
#define rSDIO_DATATIMER		(*(volatile unsigned long *)AK88_SDIO_REG(0x24))
#define rSDIO_DATALEN		(*(volatile unsigned long *)AK88_SDIO_REG(0x28))
#define rSDIO_DATACON		(*(volatile unsigned long *)AK88_SDIO_REG(0x2C))
#define rSDIO_DATACOUNT		(*(volatile unsigned long *)AK88_SDIO_REG(0x30))
#define rSDIO_STAT		(*(volatile unsigned long *)AK88_SDIO_REG(0x34))
#define rSDIO_INTEN		(*(volatile unsigned long *)AK88_SDIO_REG(0x38))
#define rSDIO_DMAMOD		(*(volatile unsigned long *)AK88_SDIO_REG(0x3C))
#define rSDIO_CPUMOD		(*(volatile unsigned long *)AK88_SDIO_REG(0x40))
#endif/*_AK88_ADDR_H_*/
