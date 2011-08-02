/*************************************************************************
*   Filename: include/asm-arm/arch-ak880x/gpio.h
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
**************************************************************************/

#ifndef _AK88_GPIO_H_
#define _AK88_GPIO_H_ __FILE__

#include <mach/hardware.h>

#include "ak880x_addr.h"
#include "io-ctl.h"

#define AK88_SHARE_GPIO		0
#define AK88_SHARE_FUNC		1
#define AK88_GPIO_OUT_0		0
#define AK88_GPIO_IN_0		1
#define AK88_GPIO_PUPD_ENABLE		0
#define AK88_GPIO_PUPD_DISABLE	1
#define AK88_GPIO_INTDISABLE		0
#define AK88_GPIO_INTENABLE 		1

#define AK88_IO_CON1                  AK88_SYSCTRL_REG(0xD4)
#define AK88_IO_CON2                  AK88_SYSCTRL_REG(0xD8)

#define AK88_SHAREPIN_CON1		AK88_SYSCTRL_REG(0x78)
#define AK88_SHAREPIN_CON2		AK88_SYSCTRL_REG(0x74)

#define AK88_GPIO_DIR1                AK88_SYSCTRL_REG(0x7C)
#define AK88_GPIO_DIR2                AK88_SYSCTRL_REG(0x84)
#define AK88_GPIO_DIR3                AK88_SYSCTRL_REG(0x8C)
#define AK88_GPIO_DIR4                AK88_SYSCTRL_REG(0x94)

#define AK88_GPIO_OUT1                AK88_SYSCTRL_REG(0x80)
#define AK88_GPIO_OUT2                AK88_SYSCTRL_REG(0x88)
#define AK88_GPIO_OUT3                AK88_SYSCTRL_REG(0x90)
#define AK88_GPIO_OUT4                AK88_SYSCTRL_REG(0x98)

#define AK88_GPIO_IN1                 AK88_SYSCTRL_REG(0xBC)
#define AK88_GPIO_IN2                 AK88_SYSCTRL_REG(0xC0)
#define AK88_GPIO_IN3                 AK88_SYSCTRL_REG(0xC4)
#define AK88_GPIO_IN4                 AK88_SYSCTRL_REG(0xC8)

#define AK88_PPU_PPD1                 AK88_SYSCTRL_REG(0x9C)
#define AK88_PPU_PPD2                 AK88_SYSCTRL_REG(0xA0)
#define AK88_PPU_PPD3                 AK88_SYSCTRL_REG(0xA4)
#define AK88_PPU_PPD4                 AK88_SYSCTRL_REG(0xA8)

#define AK88_GPIO_INTEN1              AK88_SYSCTRL_REG(0xE0)
#define AK88_GPIO_INTEN2              AK88_SYSCTRL_REG(0xE4)
#define AK88_GPIO_INTEN3              AK88_SYSCTRL_REG(0xE8)
#define AK88_GPIO_INTEN4              AK88_SYSCTRL_REG(0xEC)

#define AK88_GPIO_INTPOL1             AK88_SYSCTRL_REG(0xF0)
#define AK88_GPIO_INTPOL2             AK88_SYSCTRL_REG(0xF4)
#define AK88_GPIO_INTPOL3             AK88_SYSCTRL_REG(0xF8)
#define AK88_GPIO_INTPOL4             AK88_SYSCTRL_REG(0xFC)

#define AK88_GPIO_DIR_BASE(pin)	( ((pin)>>5)*8 + AK88_GPIO_DIR1 )
#define AK88_GPIO_OUT_BASE(pin)	( ((pin)>>5)*8 + AK88_GPIO_OUT1 )
#define AK88_GPIO_IN_BASE(pin)	( ((pin)>>5)*4 + AK88_GPIO_IN1 )
#define AK88_PPU_PPD_BASE(pin)	( ((pin)>>5)*4 + AK88_PPU_PPD1 )
#define AK88_GPIO_INTEN_BASE(pin)	( ((pin)>>5)*4 + AK88_GPIO_INTEN1 )
#define AK88_GPIO_INTPOL_BASE(pin)	( ((pin)>>5)*4 + AK88_GPIO_INTPOL1 )

/*
 * Share-pin con1 bits
 * x:0 the pins are used as gpio(default except JTAG).
 * x:1	the pins are used as function pins.
 */
#define AK88_GPIO_PCM_JTAG(x) 	ak880x_sharepin_cfg1(x,0)
#define AK88_GPIO_JTAG_RTCK(x)	ak880x_sharepin_cfg1(x,1)
#define AK88_GPIO_IIS(x)		ak880x_sharepin_cfg1(x,2)
#define AK88_GPIO_OTG_BUS(x)		ak880x_sharepin_cfg1(x,3)
#define AK88_GPIO_PWM1(x)		ak880x_sharepin_cfg1(x,4)
#define AK88_GPIO_PWM2(x)		ak880x_sharepin_cfg1(x,5)
#define AK88_GPIO_PWM3(x)		ak880x_sharepin_cfg1(x,6)
#define AK88_GPIO_PWM4(x)		ak880x_sharepin_cfg1(x,7)
#define AK88_GPIO_IIS_MCLK(x)		ak880x_sharepin_cfg1(x,8)
#define AK88_GPIO_UART0(x)		ak880x_sharepin_cfg1(x,9)
#define AK88_GPIO_UART1(x)		ak880x_sharepin_cfg1(x,10)
#define AK88_GPIO_UART1_FLOW(x)	ak880x_sharepin_cfg1(x,11)
#define AK88_GPIO_UART2(x)		ak880x_sharepin_cfg1(x,12)
#define AK88_GPIO_UART2_FLOW(x)	ak880x_sharepin_cfg1(x,13)
#define AK88_GPIO_UART3(x)		ak880x_sharepin_cfg1(x,14)
#define AK88_GPIO_UART3_FLOW(x)	ak880x_sharepin_cfg1(x,15)
#define AK88_GPIO_NFC_DATA0(x)	ak880x_sharepin_cfg1(x,16)
#define AK88_GPIO_NFC_DATA1_3(x)	ak880x_sharepin_cfg1(x,17)
#define AK88_GPIO_NFC_DATA4_7(x)	ak880x_sharepin_cfg1(x,18)
#define AK88_GPIO_NFC_CE(x)		ak880x_sharepin_cfg1(x,19)
#define AK88_GPIO_NFC_READY(x)	ak880x_sharepin_cfg1(x,22)
#define AK88_GPIO_CAMARA(x)		ak880x_sharepin_cfg1(x,24)
#define AK88_GPIO_LCD_DATA8(x)	ak880x_sharepin_cfg1(x,26)
#define AK88_GPIO_LCD_DATA9_15(x)	ak880x_sharepin_cfg1(x,25)
#define AK88_GPIO_LCD_DATA16_17(x)	ak880x_sharepin_cfg1(x,27)
#define AK88_GPIO_MPU_RST(x)		ak880x_sharepin_cfg1(x,28)
#define AK88_GPIO_MDAT2(x)		ak880x_sharepin_cfg1(x,29)
#define AK88_GPIO_SPI1(x)		ak880x_sharepin_cfg1(x,30)
#define AK88_GPIO_SPI2(x)		ak880x_sharepin_cfg1(x,31)

/* sharepin con2 */
#define AK88_MCI_ENABLE()		ak880x_sharepin_cfg2(2,5)
#define AK88_MCI_DISABLE()		ak880x_sharepin_cfg2(0,5)

#define AK88_NFC_ENABLE()		ak880x_sharepin_cfg2(1,3)
#define AK88_SD_ENABLE()		ak880x_sharepin_cfg2(2,3)
#define AK88_NFC_DISABLE()		ak880x_sharepin_cfg2(0,3)
#define AK88_SD_DISABLE()		ak880x_sharepin_cfg2(0,3)

#define AK88_UART3_ENABLE()		ak880x_sharepin_cfg2(1,1)
#define AK88_SDIO_ENABLE()		ak880x_sharepin_cfg2(2,1)
#define AK88_UART3_DISABLE()		ak880x_sharepin_cfg2(0,1)
#define AK88_SDIO_DISABLE()		ak880x_sharepin_cfg2(0,1)

#define AK88_JTAG_ENABLE()		ak880x_sharepin_cfg2(1,0)
#define AK88_PCM_ENABLE()		ak880x_sharepin_cfg2(0,0)

/* gpio offsets */
#define AK88_GPIO_GROUP1		(32*0)
#define AK88_GPIO_GROUP2		(32*1)
#define AK88_GPIO_GROUP3		(32*2)
#define AK88_GPIO_GROUP4		(32*3)

#define AK88_GPIO_GROUP1_NO(offset)	( AK88_GPIO_GROUP1 + (offset))
#define AK88_GPIO_GROUP2_NO(offset)	( AK88_GPIO_GROUP2 + (offset))
#define AK88_GPIO_GROUP3_NO(offset)	( AK88_GPIO_GROUP3 + (offset))
#define AK88_GPIO_GROUP4_NO(offset)	( AK88_GPIO_GROUP4 + (offset))

#define AK88_GPIO_0		AK88_GPIO_GROUP1_NO(0)
#define AK88_GPIO_1		AK88_GPIO_GROUP1_NO(1)
#define AK88_GPIO_2		AK88_GPIO_GROUP1_NO(2)
#define AK88_GPIO_3		AK88_GPIO_GROUP1_NO(3)
#define AK88_GPIO_4		AK88_GPIO_GROUP1_NO(4)
#define AK88_GPIO_5		AK88_GPIO_GROUP1_NO(5)
#define AK88_GPIO_6		AK88_GPIO_GROUP1_NO(6)
#define AK88_GPIO_7		AK88_GPIO_GROUP1_NO(7)
#define AK88_GPIO_8		AK88_GPIO_GROUP1_NO(8)
#define AK88_GPIO_9		AK88_GPIO_GROUP1_NO(9)
#define AK88_GPIO_10		AK88_GPIO_GROUP1_NO(10)
#define AK88_GPIO_11		AK88_GPIO_GROUP1_NO(11)
#define AK88_GPIO_12		AK88_GPIO_GROUP1_NO(12)
#define AK88_GPIO_13		AK88_GPIO_GROUP1_NO(13)
#define AK88_GPIO_14		AK88_GPIO_GROUP1_NO(14)
#define AK88_GPIO_15		AK88_GPIO_GROUP1_NO(15)
#define AK88_GPIO_16		AK88_GPIO_GROUP1_NO(16)
#define AK88_GPIO_17		AK88_GPIO_GROUP1_NO(17)
#define AK88_GPIO_18		AK88_GPIO_GROUP1_NO(18)
#define AK88_GPIO_19		AK88_GPIO_GROUP1_NO(19)
#define AK88_GPIO_20		AK88_GPIO_GROUP1_NO(20)
#define AK88_GPIO_21		AK88_GPIO_GROUP1_NO(21)
#define AK88_GPIO_22		AK88_GPIO_GROUP1_NO(22)
#define AK88_GPIO_23		AK88_GPIO_GROUP1_NO(23)
#define AK88_GPIO_24		AK88_GPIO_GROUP1_NO(24)
#define AK88_GPIO_25		AK88_GPIO_GROUP1_NO(25)
#define AK88_GPIO_26		AK88_GPIO_GROUP1_NO(26)
#define AK88_GPIO_27		AK88_GPIO_GROUP1_NO(27)
#define AK88_GPIO_28		AK88_GPIO_GROUP1_NO(28)
#define AK88_GPIO_29		AK88_GPIO_GROUP1_NO(29)
#define AK88_GPIO_30		AK88_GPIO_GROUP1_NO(30)
#define AK88_GPIO_31		AK88_GPIO_GROUP1_NO(31)

#define AK88_GPIO_32		AK88_GPIO_GROUP2_NO(0)
#define AK88_GPIO_33		AK88_GPIO_GROUP2_NO(1)
#define AK88_GPIO_34		AK88_GPIO_GROUP2_NO(2)
#define AK88_GPIO_35		AK88_GPIO_GROUP2_NO(3)
#define AK88_GPIO_36		AK88_GPIO_GROUP2_NO(4)
#define AK88_GPIO_37		AK88_GPIO_GROUP2_NO(5)
#define AK88_GPIO_38		AK88_GPIO_GROUP2_NO(6)
#define AK88_GPIO_39		AK88_GPIO_GROUP2_NO(7)
#define AK88_GPIO_40		AK88_GPIO_GROUP2_NO(8)
#define AK88_GPIO_41		AK88_GPIO_GROUP2_NO(9)
#define AK88_GPIO_42		AK88_GPIO_GROUP2_NO(10)
#define AK88_GPIO_43		AK88_GPIO_GROUP2_NO(11)
#define AK88_GPIO_44		AK88_GPIO_GROUP2_NO(12)
#define AK88_GPIO_45		AK88_GPIO_GROUP2_NO(13)
#define AK88_GPIO_46		AK88_GPIO_GROUP2_NO(14)
#define AK88_GPIO_47		AK88_GPIO_GROUP2_NO(15)
#define AK88_GPIO_48		AK88_GPIO_GROUP2_NO(16)
#define AK88_GPIO_49		AK88_GPIO_GROUP2_NO(17)
#define AK88_GPIO_50		AK88_GPIO_GROUP2_NO(18)
#define AK88_GPIO_51		AK88_GPIO_GROUP2_NO(19)
#define AK88_GPIO_52		AK88_GPIO_GROUP2_NO(20)
#define AK88_GPIO_53		AK88_GPIO_GROUP2_NO(21)
#define AK88_GPIO_54		AK88_GPIO_GROUP2_NO(22)
#define AK88_GPIO_55		AK88_GPIO_GROUP2_NO(23)
#define AK88_GPIO_56		AK88_GPIO_GROUP2_NO(24)
#define AK88_GPIO_57		AK88_GPIO_GROUP2_NO(25)
#define AK88_GPIO_58		AK88_GPIO_GROUP2_NO(26)
#define AK88_GPIO_59_RESERVED AK88_GPIO_GROUP2_NO(27)	/* Reserved */
#define AK88_GPIO_60_RESERVED	AK88_GPIO_GROUP2_NO(28)	/* Reserved */
#define AK88_GPIO_61		AK88_GPIO_GROUP2_NO(29)
#define AK88_GPIO_62		AK88_GPIO_GROUP2_NO(30)
#define AK88_GPIO_63		AK88_GPIO_GROUP2_NO(31)

#define AK88_GPIO_64		AK88_GPIO_GROUP3_NO(0)
#define AK88_GPIO_65		AK88_GPIO_GROUP3_NO(1)
#define AK88_GPIO_66		AK88_GPIO_GROUP3_NO(2)
#define AK88_GPIO_67		AK88_GPIO_GROUP3_NO(3)
#define AK88_GPIO_68		AK88_GPIO_GROUP3_NO(4)
#define AK88_GPIO_69		AK88_GPIO_GROUP3_NO(5)
#define AK88_GPIO_70		AK88_GPIO_GROUP3_NO(6)
#define AK88_GPIO_71		AK88_GPIO_GROUP3_NO(7)
#define AK88_GPIO_72		AK88_GPIO_GROUP3_NO(8)
#define AK88_GPIO_73		AK88_GPIO_GROUP3_NO(9)
#define AK88_GPIO_74		AK88_GPIO_GROUP3_NO(10)
#define AK88_GPIO_75		AK88_GPIO_GROUP3_NO(11)
#define AK88_GPIO_76		AK88_GPIO_GROUP3_NO(12)
#define AK88_GPIO_77		AK88_GPIO_GROUP3_NO(13)
#define AK88_GPIO_78		AK88_GPIO_GROUP3_NO(14)
#define AK88_GPIO_79		AK88_GPIO_GROUP3_NO(15)
#define AK88_GPIO_80		AK88_GPIO_GROUP3_NO(16)
#define AK88_GPIO_81		AK88_GPIO_GROUP3_NO(17)
#define AK88_GPIO_82		AK88_GPIO_GROUP3_NO(18)
#define AK88_GPIO_83		AK88_GPIO_GROUP3_NO(19)
#define AK88_DGPIO_19		AK88_GPIO_GROUP3_NO(20)
#define AK88_DGPIO_20		AK88_GPIO_GROUP3_NO(21)
#define AK88_DGPIO_21		AK88_GPIO_GROUP3_NO(22)
#define AK88_DGPIO_22		AK88_GPIO_GROUP3_NO(23)
#define AK88_DGPIO_23		AK88_GPIO_GROUP3_NO(24)
#define AK88_DGPIO_24		AK88_GPIO_GROUP3_NO(25)
#define AK88_DGPIO_25		AK88_GPIO_GROUP3_NO(26)
#define AK88_DGPIO_26		AK88_GPIO_GROUP3_NO(27)
#define AK88_DGPIO_27		AK88_GPIO_GROUP3_NO(28)
#define AK88_DGPIO_28		AK88_GPIO_GROUP3_NO(29)
#define AK88_DGPIO_29		AK88_GPIO_GROUP3_NO(30)
#define AK88_DGPIO_30		AK88_GPIO_GROUP3_NO(31)

#define AK88_DGPIO_31		AK88_GPIO_GROUP4_NO(0)
#define AK88_DGPIO_32		AK88_GPIO_GROUP4_NO(1)
#define AK88_DGPIO_33		AK88_GPIO_GROUP4_NO(2)
#define AK88_DGPIO_34		AK88_GPIO_GROUP4_NO(3)
#define AK88_DGPIO_35		AK88_GPIO_GROUP4_NO(4)
#define AK88_DGPIO_36		AK88_GPIO_GROUP4_NO(5)
#define AK88_DGPIO_0		AK88_GPIO_GROUP4_NO(6)
#define AK88_DGPIO_1		AK88_GPIO_GROUP4_NO(7)
#define AK88_DGPIO_2		AK88_GPIO_GROUP4_NO(8)
#define AK88_DGPIO_3		AK88_GPIO_GROUP4_NO(9)
#define AK88_DGPIO_4		AK88_GPIO_GROUP4_NO(10)
#define AK88_DGPIO_5		AK88_GPIO_GROUP4_NO(11)
#define AK88_DGPIO_6_RESERVED	AK88_GPIO_GROUP4_NO(12)	/* Reserved */
#define AK88_DGPIO_7		AK88_GPIO_GROUP4_NO(13)
#define AK88_DGPIO_8		AK88_GPIO_GROUP4_NO(14)
#define AK88_DGPIO_9		AK88_GPIO_GROUP4_NO(15)
#define AK88_DGPIO_10		AK88_GPIO_GROUP4_NO(16)
#define AK88_DGPIO_11		AK88_GPIO_GROUP4_NO(17)
#define AK88_DGPIO_12		AK88_GPIO_GROUP4_NO(18)
#define AK88_DGPIO_13		AK88_GPIO_GROUP4_NO(19)
#define AK88_DGPIO_14		AK88_GPIO_GROUP4_NO(20)

void ak880x_sharepin_cfg1(unsigned char to, unsigned char offset);
void ak880x_sharepin_cfg2(unsigned char to, unsigned char offset);
void ak880x_gpio_cfgpin(unsigned int pin, unsigned int function);
unsigned int ak880x_gpio_getcfg(unsigned int pin);
void ak880x_gpio_setpin(unsigned int pin, unsigned int to);
unsigned int ak880x_gpio_getpin(unsigned int pin);
void ak880x_gpio_pullup(unsigned int pin, unsigned int to);
void ak880x_gpio_inten(unsigned int pin, unsigned int to);
void ak880x_gpio_intpol(unsigned int pin, unsigned int to);
void ak880x_ioctl(unsigned int offset, const unsigned int to);

unsigned int ak880x_gpio_to_irq(unsigned int pin);
unsigned int ak880x_irq_to_gpio(unsigned int irq);

#endif/*_AK88_GPIO_H_*/
