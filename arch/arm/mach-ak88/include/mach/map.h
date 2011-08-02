/* linux/include/asm-arm/arch-s3c2410/map.h
 *
 * AK88 - Memory map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MAP_H
#define __ASM_ARCH_MAP_H

/* we have a bit of a tight squeeze to fit all our registers from
 * 0xF00000000 upwards, since we use all of the nGCS space in some
 * capacity, and also need to fit the S3C2410 registers in as well...
 * 
 * we try to ensure stuff like the IRQ registers are available for
 * an single MOVS instruction (ie, only 8 bits of set data)
 *
 * Note, we are trying to remove some of these from the implementation
 * as they are only useful to certain drivers...
 */
#ifndef __ASSEMBLY__
#define AK88_ADDR(x)		((void __iomem *)0xF0000000 + (x))
#else
#define AK88_ADDR(x)		(0xF0000000 + (x))
#endif

#define AK88_VA_OCROM		AK88_ADDR(0x00000000)
#define AK88_PA_OCROM		(0x00000000)
#define AK88_SZ_OCROM		2*SZ_16K	/* 32KB */

#define AK88_VA_SYSCTRL	AK88_ADDR(0x00100000)
#define AK88_PA_SYSCTRL	(0x08000000)
#define AK88_SZ_SYSCTRL	SZ_64K	/* 64KB */
#define AK88_VA_SYS           AK88_ADDR(0x00100000)
#define AK88_PA_SYS	  	(0x08000000)
 
#define AK88_VA_SUBCTRL	AK88_ADDR(0x00200000)
#define AK88_PA_SUBCTRL	(0x20000000)
#define AK88_SZ_SUBCTRL	SZ_1M

#define AK88_VA_L2MEM		AK88_ADDR(0x00300000)
#define AK88_PA_L2MEM		(0x48000000)
#define AK88_SZ_L2MEM		SZ_1M
#define AK88_VA_L2BUF         AK88_ADDR(0x00300000)
#define	AK88_PA_L2BUF	        (0x48000000)

#define AK88_VA_USB		AK88_ADDR(0x00400000)
#define AK88_PA_USB		(0x70000000)
#define AK88_SZ_USB		SZ_1M
#define AK88_VA_USB	   	AK88_ADDR(0x00400000)
#define AK88_PA_USB	 	(0x70000000)
 
#define AK88_VA_DISPLAY	(AK88_VA_SUBCTRL + 0x10000)
#define AK88_PA_DISPLAY	(AK88_PA_SUBCTRL + 0x10000)
#define AK88_VA_DISP     	(AK88_VA_SUBCTRL + 0x10000)
#define AK88_PA_DISP	  	(0x20010000)

#define AK88_VA_MMC		(AK88_VA_SUBCTRL + 0x20000)
#define AK88_PA_MMC		(AK88_PA_SUBCTRL + 0x20000)
#define AK88_VA_DEV 	        (AK88_VA_SUBCTRL + 0x20000)
#define AK88_PA_DEV		(0x20020000)
 
#define AK88_VA_SDIO		(AK88_VA_SUBCTRL + 0x21000)
#define AK88_PA_SDIO		(AK88_PA_SUBCTRL + 0x21000)

#define AK88_VA_2DACC		(AK88_VA_SUBCTRL + 0x22000)
#define AK88_PA_2DACC		(AK88_PA_SUBCTRL + 0x22000)

#define AK88_VA_SPI0		(AK88_VA_SUBCTRL + 0x24000)
#define AK88_PA_SPI0		(AK88_PA_SUBCTRL + 0x24000)

#define AK88_VA_SPI1		(AK88_VA_SUBCTRL + 0x25000)
#define AK88_PA_SPI1		(AK88_PA_SUBCTRL + 0x25000)

#define AK88_VA_UART		(AK88_VA_SUBCTRL + 0x26000)
#define AK88_PA_UART		(AK88_PA_SUBCTRL + 0x26000)

#define AK88_VA_NFCTRL	(AK88_VA_SUBCTRL + 0x2A000)
#define AK88_PA_NFCTRL	(AK88_PA_SUBCTRL + 0x2A000)

#define AK88_VA_ECCCTRL	(AK88_VA_SUBCTRL + 0x2B000)
#define AK88_PA_ECCCTRL	(AK88_PA_SUBCTRL + 0x2B000)

#define AK88_VA_L2CTRL	(AK88_VA_SUBCTRL + 0x2C000)
#define AK88_PA_L2CTRL	(AK88_PA_SUBCTRL + 0x2C000)

#define AK88_VA_RAMCTRL	(AK88_VA_SUBCTRL + 0x2D000)
#define AK88_PA_RAMCTRL	(AK88_PA_SUBCTRL + 0x2D000)

#define AK88_VA_DACCTRL	(AK88_VA_SUBCTRL + 0x2E000)
#define AK88_PA_DACCTRL	(AK88_PA_SUBCTRL + 0x2E000)

#define AK88_VA_CAMIF		(AK88_VA_SUBCTRL + 0x30000)
#define AK88_PA_CAMIF		(AK88_PA_SUBCTRL + 0x30000)
#define AK88_VA_CAMER	   	(AK88_VA_SUBCTRL + 0x30000)
#define AK88_PA_CAMER	  	(0x20030000)
 
#define AK88_VA_HWMM		(AK88_VA_SUBCTRL + 0x40000)
#define AK88_PA_HWMM		(AK88_PA_SUBCTRL + 0x40000)
#define AK88_VA_IMAGE	        (AK88_VA_SUBCTRL + 0x40000)
#define AK88_PA_IMAGE	  	(0x20040000)
 
/* physical addresses of all the chip-select areas */

#define AK88_SDRAM_PA		(0x30000000)

#endif				/* __ASM_ARCH_MAP_H */

///========================================================================////

#ifndef __ARCH_ARM_MACH_AK88_MAP_H
#define __ARCH_ARM_MACH_AK88_MAP_H

//#include <mach/map-base.h>

#if 0
#define AK88_VA_SYS	AK88_ADDR(0x00000000)	/* System control */
#define AK88_VA_MEM	AK88_ADDR(0x00100000)	/* L2 memory control */
#define AK88_VA_DEV	AK88_ADDR(0x00200000)	/* device controller(s) */
#define AK88_VA_LCD	AK88_ADDR(0x00300000)	/* system control */
#define AK88_VA_CAMERA	AK88_ADDR(0x00400000)	/* camera control */
#define AK88_VA_IMAGE	AK88_ADDR(0x00500000)	/* image control */
#define AK88_VA_VIDEO	AK88_ADDR(0x00600000)	/* video control */
#define AK88_VA_AUDIO	AK88_ADDR(0x00700000)	/* audio control */
#endif

/* This is used for the CPU specific mappings that may be needed, so that
 * they do not need to directly used AK88_ADDR() and thus make it easier to
 * modify the space for mapping.
 */
//#define AK88_ADDR_CPU(x)    AK88_ADDR(0x00700000 + (x))

#define		AK_FALSE			0
#define		AK_TRUE				1
#define 	AK_NULL				((void*)(0))
#define		AK_EMPTY

#define write_ramb(v,p)	             (*(volatile unsigned char *)(p) = (v))
#define write_ramw(v,p)	             (*(volatile unsigned short *)(p) = (v))
#define write_raml(v,p)	             (*(volatile unsigned long  *)(p) = (v))

#define read_ramb(p)	             (*(volatile unsigned char *)(p))
#define read_ramw(p)	             (*(volatile unsigned short *)(p))
#define read_raml(p)	             (*(volatile unsigned long *)(p))

#define write_buf(v,p)		     (*(volatile unsigned long  *)(p) = (v))
#define read_buf(p)		     (*(volatile unsigned long *)(p))


/************************** IRQ ************************************/

#define AK88_IRQ_IMR                (AK88_VA_SYS+0x0034)
#define AK88_IRQ_FMR                (AK88_VA_SYS+0x0038)
#define AK88_IRQ_STA                (AK88_VA_SYS+0x00CC)
#define AK88_IRQ_SYS_STA_ENA        (AK88_VA_SYS+0x004C)

/***************************** TIMER ********************************/

#define AK88_TIMER_CON1		   (AK88_VA_SYS+0x0018)
#define AK88_TIMER_CON2		   (AK88_VA_SYS+0x001C)
#define AK88_TIMER_CON3		   (AK88_VA_SYS+0x0020)
#define AK88_TIMER_CON4		   (AK88_VA_SYS+0x0024)
#define AK88_TIMER_CON5		   (AK88_VA_SYS+0x0028)

#define AK88_TIMER_RBR1		   (AK88_VA_SYS+0x0100)
#define AK88_TIMER_RBR2		   (AK88_VA_SYS+0x0104)
#define AK88_TIMER_RBR3		   (AK88_VA_SYS+0x0108)
#define AK88_TIMER_RBR4		   (AK88_VA_SYS+0x010C)
#define AK88_TIMER_RBR5		   (AK88_VA_SYS+0x0110)

#define AK88_TIMER_BIT_EN	   (1<<26)
#define AK88_TIMER_BIT_LD	   (1<<27)	//to load new count value
#define AK88_TIMER_BIT_CLEAR	   (1<<28)
#define AK88_TIMER_BIT_STA	   (1<<29)

/************************** CLOCK/POWER ***********************************/
#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)	//0xF000000C
//1=disable corresponding pin power clock
//0=enable  corresponding pin power clock

//bit[15],0 = to enable L2 controller/UART1 working clock

//bit[3], 0 = to enable display controller working clock

/**************************** SHARE PIN CTRL**************************/

#define AK88_SHAREPIN_CTRL    (AK88_VA_SYS+0x0078)	//0xF0000078
#define AK88_PA_SHAREPIN_CTRL    (AK88_PA_SYS+0x0078)	//0xF0000078
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
//all bits default is 0, = Corresponding pin is used as GPIO
//bit[9] : 1=corresponding are used as {URD1,UTD1}  // gpio[15:14]
//bit[14]: 1=corresponding are used as {URD4,UTD4} or {SDIO_data[1:0]} // gpio[25:24]

//bit[25]: 1=corresponding are used as {LCD_DATA[15:9]}  //gpio[68:62]
//bit[26]: 1=corresponding are used as {LCD_DATA[8]}     //gpio[61]
//bit[27]: 1=corresponding are used as {LCD_DATA[17:16]} //gpio[70:69]

#define AK88_SHAREPIN_CTRL2    (AK88_VA_SYS+0x0074)	//0xF0000074
#define AK88_PA_SHAREPIN_CTRL2    (AK88_PA_SYS+0x0074)	//0xF0000074
//bit[2:1] 00:reserved
//         01 = Corresponding pins are used as those of UART4
//         10 = Corresponding pins are used as those of UART4
//         11:reserved

/************************** GPIO ***********************************/
#define AK88_GPIO_BASE         (AK88_VA_SYS)

#define AK88_GPIO_DIR(n)       (AK88_VA_SYS+0x007C+n*8)	//n=0~3
#define AK88_PA_GPIO_DIR(n)       (AK88_PA_SYS+0x007C+n*8)	//n=0~3

//0x7C for GPIO[0] ~ GPIO[31]
//0x84 for GPIO[32] ~ GPIO[63]
//0x8C for GPIO[64] ~ GPIO[79], bit20:DGPIO[19],bit29:DGPIO[28]
//0x94 for bit[9:6]: DGPIO[0]~ DGPIO[3]

#define AK88_GPIO_OUT(n)         (AK88_VA_SYS+0x0080+n*8)	//n=0~3
#define AK88_PA_GPIO_OUT(n)         (AK88_PA_SYS+0x0080+n*8)	//n=0~3

//0x80
//0x88
//0x90
//0x98

#define AK88_GPIO_IN(n)          (AK88_VA_SYS+0x00BC+n*4)	//n=0~3
#define AK88_PA_GPIO_IN(n)          (AK88_PA_SYS+0x00BC+n*4)	//n=0~3

//0xBC
//0xC0
//0xC4
//0xC8

#define AK88_GPIO_INT_EN1         (AK88_VA_SYS+0x00E0+n*4)	//n=0~3
#define AK88_PA_GPIO_INT_EN1         (AK88_VA_SYS+0x00E0+n*4)	//n=0~3
//0xE0
//0xE4
//0xE8
//0xEC

#define AK88_GPIO_INT_POLA1       (AK88_VA_SYS+0x00F0+n*4)	//n=0~3
#define AK88_PA_GPIO_INT_POLA1       (AK88_VA_SYS+0x00F0+n*4)	//n=0~3
//0xF0
//0xF4
//0xF8
//0xFC

/*************************** RAM CONTROLER ***************************/
#define AK88_AHB_PRIORITY                 (AK88_VA_DEV +0xD014)

/************************** L2 CONTROLER **************************/

#define	AK88_L2CTR_DMAADDR	             (AK88_VA_DEV+0xC000+0x00)
//0x2002c000 -> 0x2002c03c for buffer 0 to 15 (32-bit for each)

#define	AK88_L2CTR_DMACNT                    (AK88_VA_DEV+0xC000+0x40)
//0x2002c040 -> 0x2002c07c for buffer 0 to 15 (32-bit for each)
//need operating DMA times,
//DMA controller can only transfer 64 byes at each time

#define	AK88_L2CTR_DMAREQ		       (AK88_VA_DEV+0xC000+0x80)	//0x2002c080
#define	AK88_L2CTR_DMAFRAC	               (AK88_VA_DEV+0xC000+0x84)	//0x2002c084
#define	AK88_L2CTR_COMBUF_CFG	               (AK88_VA_DEV+0xC000+0x88)	//0x2002c088
#define	AK88_L2CTR_UARTBUF_CFG	       (AK88_VA_DEV+0xC000+0x8C)	//0x2002c08c
#define AK88_L2CTR_ASSIGN_REG1               (AK88_VA_DEV+0xC000+0x90)	//0x2002c090
#define AK88_L2CTR_ASSIGN_REG2               (AK88_VA_DEV+0xC000+0x94)	//0x2002c094
#define AK88_L2CTR_LDMA_CFG                  (AK88_VA_DEV+0xC000+0x98)	//0x2002c098
#define AK88_L2CTR_STAT_REG1                 (AK88_VA_DEV+0xC000+0xA0)	//0x2002c0A0
#define AK88_L2CTR_STAT_REG2                 (AK88_VA_DEV+0xC000+0xA8)	//0x2002c0A8

/************************** UART *************************************/

#define AK88_UART_BASE              (AK88_VA_DEV+0x6000)
#define AK88_UART_CFG_REG1(n)       (AK88_VA_DEV+0x6000+n*0x1000)	//n=0 to 3   // 0x20026000,0x20027000,0x20028000,0x20029000
#define AK88_UART_CFG_REG2(n)       (AK88_VA_DEV+0x6004+n*0x1000)	//n=0 to 3   // 0x20026004,0x20027004,0x20028004,0x20029004
#define AK88_UART_DATA_CFG(n)       (AK88_VA_DEV+0x6008+n*0x1000)	//n=0 to 3   // 0x20026008,0x20027008,0x20028008,0x20029008
#define AK88_UART_THREHOLD(n)       (AK88_VA_DEV+0x600C+n*0x1000)	//n=0 to 3   // 0x2002600C,0x2002700C,0x2002800C,0x2002900C
#define AK88_UART_RX_DATA(n)        (AK88_VA_DEV+0x6010+n*0x1000)	//n=0 to 3   // 0x20026010,0x20027010,0x20028010,0x20029010

#define UART_BOOT_SET 


//#include <mach/ak880x_freq.h>

#endif
