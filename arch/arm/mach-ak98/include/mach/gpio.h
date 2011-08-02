/*************************************************************************
*   Filename: arch/arm/mach-ak98/include/mach/gpio.h
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
**************************************************************************/

#ifndef _AK98_GPIO_H_
#define _AK98_GPIO_H_ 

#include "map.h"
#include <linux/gpio.h>


#define	AK98_SHARE_GPIO			0
#define	AK98_SHARE_FUNC			1

#define AK98_WAKEUP_ENABLE		1
#define AK98_WAKEUP_DISABLE		0
#define AK98_FALLING_TRIGGERED   1
#define AK98_RISING_TRIGGERED	 0

#define	AK98_GPIO_DIR_OUTPUT	0
#define	AK98_GPIO_DIR_INPUT		1

#define	AK98_PULLUP_DISABLE		0
#define	AK98_PULLUP_ENABLE		1
#define	AK98_PULLDOWN_DISABLE	0
#define	AK98_PULLDOWN_ENABLE	1


#define	AK98_GPIO_INT_DISABLE	0
#define	AK98_GPIO_INT_ENABLE	1

#define	AK98_GPIO_LOW			0
#define	AK98_GPIO_HIGH			1
#define	AK98_GPIO_OUT_LOW			0
#define	AK98_GPIO_OUT_HIGH			1


#define	AK98_GPIO_INT_LOWLEVEL	0
#define	AK98_GPIO_INT_HIGHLEVEL	1

#define AK_FALSE		0
#define AK_TRUE			1
#undef AK_NULL
#define AK_NULL			((void *)(0))
#define AK_EMPTY



#define AK98_GPIO_UART1_FLOW(x)		ak98_sharepin_cfg1(x,11,SHARE_CONFG1)
#define AK98_GPIO_UART2_FLOW(x)		ak98_sharepin_cfg1(x,13,SHARE_CONFG1)
#define AK98_GPIO_UART3_FLOW(x)		ak98_sharepin_cfg1(x,15,SHARE_CONFG1)


/**************** gpio offsets ************************/
#define AK98_GPIO_GROUP1		(32*0)
#define AK98_GPIO_GROUP2		(32*1)
#define AK98_GPIO_GROUP3		(32*2)
#define AK98_GPIO_GROUP4		(32*3)
#define AK98_GPIO_GROUP5		(32*4)
#define AK98_GPIO_GROUP6		(32*5)


#define AK98_GPIO_GROUP1_NO(offset)		( AK98_GPIO_GROUP1 + (offset))
#define AK98_GPIO_GROUP2_NO(offset)		( AK98_GPIO_GROUP2 + (offset))
#define AK98_GPIO_GROUP3_NO(offset)		( AK98_GPIO_GROUP3 + (offset))
#define AK98_GPIO_GROUP4_NO(offset)		( AK98_GPIO_GROUP4 + (offset))
#define AK98_GPIO_GROUP5_NO(offset)		( AK98_GPIO_GROUP5 + (offset))
#define AK98_GPIO_GROUP6_NO(offset)		( AK98_GPIO_GROUP6 + (offset))


		
#define AK98_GPIO_0			AK98_GPIO_GROUP1_NO(0)
#define AK98_GPIO_MIN		AK98_GPIO_0

#define AK98_GPIO_1			AK98_GPIO_GROUP1_NO(1)
#define AK98_GPIO_2			AK98_GPIO_GROUP1_NO(2)
#define AK98_GPIO_3			AK98_GPIO_GROUP1_NO(3)
#define AK98_GPIO_4			AK98_GPIO_GROUP1_NO(4)
#define AK98_GPIO_5			AK98_GPIO_GROUP1_NO(5)
#define AK98_GPIO_6			AK98_GPIO_GROUP1_NO(6)
#define AK98_GPIO_7			AK98_GPIO_GROUP1_NO(7)
#define AK98_GPIO_8			AK98_GPIO_GROUP1_NO(8)
#define AK98_GPIO_9			AK98_GPIO_GROUP1_NO(9)
#define AK98_GPIO_10		AK98_GPIO_GROUP1_NO(10)
#define AK98_GPIO_11		AK98_GPIO_GROUP1_NO(11)
#define AK98_GPIO_12		AK98_GPIO_GROUP1_NO(12)
#define AK98_GPIO_13		AK98_GPIO_GROUP1_NO(13)
#define AK98_GPIO_14		AK98_GPIO_GROUP1_NO(14)
#define AK98_GPIO_15		AK98_GPIO_GROUP1_NO(15)
#define AK98_GPIO_16		AK98_GPIO_GROUP1_NO(16)
#define AK98_GPIO_17		AK98_GPIO_GROUP1_NO(17)
#define AK98_GPIO_18		AK98_GPIO_GROUP1_NO(18)
#define AK98_GPIO_19		AK98_GPIO_GROUP1_NO(19)
#define AK98_GPIO_20		AK98_GPIO_GROUP1_NO(20)
#define AK98_GPIO_21		AK98_GPIO_GROUP1_NO(21)
#define AK98_GPIO_22		AK98_GPIO_GROUP1_NO(22)
#define AK98_GPIO_23		AK98_GPIO_GROUP1_NO(23)
#define AK98_GPIO_24		AK98_GPIO_GROUP1_NO(24)
#define AK98_GPIO_25		AK98_GPIO_GROUP1_NO(25)
#define AK98_GPIO_26		AK98_GPIO_GROUP1_NO(26)
#define AK98_GPIO_27		AK98_GPIO_GROUP1_NO(27)
#define AK98_GPIO_28		AK98_GPIO_GROUP1_NO(28)
#define AK98_GPIO_29		AK98_GPIO_GROUP1_NO(29)
#define AK98_GPIO_30		AK98_GPIO_GROUP1_NO(30)
#define AK98_GPIO_31		AK98_GPIO_GROUP1_NO(31)

#define AK98_GPIO_32		AK98_GPIO_GROUP2_NO(0)
#define AK98_GPIO_33		AK98_GPIO_GROUP2_NO(1)
#define AK98_GPIO_34		AK98_GPIO_GROUP2_NO(2)
#define AK98_GPIO_35		AK98_GPIO_GROUP2_NO(3)
#define AK98_GPIO_36		AK98_GPIO_GROUP2_NO(4)
#define AK98_GPIO_37		AK98_GPIO_GROUP2_NO(5)
#define AK98_GPIO_38		AK98_GPIO_GROUP2_NO(6)
#define AK98_GPIO_39		AK98_GPIO_GROUP2_NO(7)
#define AK98_GPIO_40		AK98_GPIO_GROUP2_NO(8)
#define AK98_GPIO_41		AK98_GPIO_GROUP2_NO(9)
#define AK98_GPIO_42		AK98_GPIO_GROUP2_NO(10)
#define AK98_GPIO_43		AK98_GPIO_GROUP2_NO(11)
#define AK98_GPIO_44		AK98_GPIO_GROUP2_NO(12)
#define AK98_GPIO_45		AK98_GPIO_GROUP2_NO(13)
#define AK98_GPIO_46		AK98_GPIO_GROUP2_NO(14)
#define AK98_GPIO_47		AK98_GPIO_GROUP2_NO(15)
#define AK98_GPIO_48		AK98_GPIO_GROUP2_NO(16)
#define AK98_GPIO_49		AK98_GPIO_GROUP2_NO(17)
#define AK98_GPIO_50		AK98_GPIO_GROUP2_NO(18)
#define AK98_GPIO_51		AK98_GPIO_GROUP2_NO(19)
#define AK98_GPIO_52		AK98_GPIO_GROUP2_NO(20)
#define AK98_GPIO_53		AK98_GPIO_GROUP2_NO(21)
#define AK98_GPIO_54		AK98_GPIO_GROUP2_NO(22)
#define AK98_GPIO_55		AK98_GPIO_GROUP2_NO(23)
#define AK98_GPIO_56		AK98_GPIO_GROUP2_NO(24)
#define AK98_GPIO_57		AK98_GPIO_GROUP2_NO(25)
#define AK98_GPIO_58		AK98_GPIO_GROUP2_NO(26)
#define AK98_GPIO_59 		AK98_GPIO_GROUP2_NO(27)	/* Reserved */
#define AK98_GPIO_60		AK98_GPIO_GROUP2_NO(28)	/* Reserved */
#define AK98_GPIO_61		AK98_GPIO_GROUP2_NO(29)
#define AK98_GPIO_62		AK98_GPIO_GROUP2_NO(30)
#define AK98_GPIO_63		AK98_GPIO_GROUP2_NO(31)

#define AK98_GPIO_64		AK98_GPIO_GROUP3_NO(0)
#define AK98_GPIO_65		AK98_GPIO_GROUP3_NO(1)
#define AK98_GPIO_66		AK98_GPIO_GROUP3_NO(2)
#define AK98_GPIO_67		AK98_GPIO_GROUP3_NO(3)
#define AK98_GPIO_68		AK98_GPIO_GROUP3_NO(4)
#define AK98_GPIO_69		AK98_GPIO_GROUP3_NO(5)
#define AK98_GPIO_70		AK98_GPIO_GROUP3_NO(6)
#define AK98_GPIO_71		AK98_GPIO_GROUP3_NO(7)
#define AK98_GPIO_72		AK98_GPIO_GROUP3_NO(8)
#define AK98_GPIO_73		AK98_GPIO_GROUP3_NO(9)
#define AK98_GPIO_74		AK98_GPIO_GROUP3_NO(10)
#define AK98_GPIO_75		AK98_GPIO_GROUP3_NO(11)
#define AK98_GPIO_76		AK98_GPIO_GROUP3_NO(12)
#define AK98_GPIO_77		AK98_GPIO_GROUP3_NO(13)
#define AK98_GPIO_78		AK98_GPIO_GROUP3_NO(14)
#define AK98_GPIO_79		AK98_GPIO_GROUP3_NO(15)
#define AK98_GPIO_80		AK98_GPIO_GROUP3_NO(16)
#define AK98_GPIO_81		AK98_GPIO_GROUP3_NO(17)
#define AK98_GPIO_82		AK98_GPIO_GROUP3_NO(18)
#define AK98_GPIO_83		AK98_GPIO_GROUP3_NO(19)
#define AK98_GPIO_84		AK98_GPIO_GROUP3_NO(20)
#define AK98_GPIO_85		AK98_GPIO_GROUP3_NO(21)
#define AK98_GPIO_86		AK98_GPIO_GROUP3_NO(22)
#define AK98_GPIO_87		AK98_GPIO_GROUP3_NO(23)
#define AK98_GPIO_88		AK98_GPIO_GROUP3_NO(24)
#define AK98_GPIO_89		AK98_GPIO_GROUP3_NO(25)
#define AK98_GPIO_90		AK98_GPIO_GROUP3_NO(26)
#define AK98_GPIO_91		AK98_GPIO_GROUP3_NO(27)
#define AK98_GPIO_92		AK98_GPIO_GROUP3_NO(28)
#define AK98_GPIO_93		AK98_GPIO_GROUP3_NO(29)
#define AK98_GPIO_94		AK98_GPIO_GROUP3_NO(30) /* Reserved */
#define AK98_GPIO_95		AK98_GPIO_GROUP3_NO(31) /* Reserved */


#define AK98_GPIO_96		AK98_GPIO_GROUP4_NO(0) /* Reserved */
#define AK98_GPIO_97		AK98_GPIO_GROUP4_NO(1)
#define AK98_GPIO_98		AK98_GPIO_GROUP4_NO(2)
#define AK98_GPIO_99		AK98_GPIO_GROUP4_NO(3)
#define AK98_GPIO_100		AK98_GPIO_GROUP4_NO(4)
#define AK98_GPIO_101		AK98_GPIO_GROUP4_NO(5)
#define AK98_GPIO_102		AK98_GPIO_GROUP4_NO(6)
#define AK98_GPIO_103		AK98_GPIO_GROUP4_NO(7)
#define AK98_GPIO_104		AK98_GPIO_GROUP4_NO(8)
#define AK98_GPIO_105		AK98_GPIO_GROUP4_NO(9)
#define AK98_GPIO_106		AK98_GPIO_GROUP4_NO(10)
#define AK98_GPIO_107		AK98_GPIO_GROUP4_NO(11)
#define AK98_GPIO_108		AK98_GPIO_GROUP4_NO(12)	/* Reserved */
#define AK98_GPIO_109		AK98_GPIO_GROUP4_NO(13)
#define AK98_GPIO_110		AK98_GPIO_GROUP4_NO(14)
#define AK98_GPIO_111		AK98_GPIO_GROUP4_NO(15)
#define AK98_GPIO_112		AK98_GPIO_GROUP4_NO(16)
#define AK98_GPIO_113		AK98_GPIO_GROUP4_NO(17)
#define AK98_GPIO_114		AK98_GPIO_GROUP4_NO(18)
#define AK98_GPIO_115		AK98_GPIO_GROUP4_NO(19)
#define AK98_GPIO_116		AK98_GPIO_GROUP4_NO(20)
#define AK98_MCGPIO_0		AK98_GPIO_GROUP5_NO(0)
#define AK98_MCGPIO_1		AK98_GPIO_GROUP5_NO(1)
#define AK98_MCGPIO_2		AK98_GPIO_GROUP5_NO(2)
#define AK98_MCGPIO_3		AK98_GPIO_GROUP5_NO(3)
#define AK98_MCGPIO_4		AK98_GPIO_GROUP5_NO(4)
#define AK98_MCGPIO_5		AK98_GPIO_GROUP5_NO(5)
#define AK98_MCGPIO_6		AK98_GPIO_GROUP5_NO(6)
#define AK98_MCGPIO_7		AK98_GPIO_GROUP5_NO(7)
#define AK98_MCGPIO_8		AK98_GPIO_GROUP5_NO(8)
#define AK98_MCGPIO_9		AK98_GPIO_GROUP5_NO(9)
#define AK98_MCGPIO_10		AK98_GPIO_GROUP5_NO(10)
#define AK98_MCGPIO_11		AK98_GPIO_GROUP5_NO(11)
#define AK98_MCGPIO_12		AK98_GPIO_GROUP5_NO(12)
#define AK98_MCGPIO_13		AK98_GPIO_GROUP5_NO(13)
#define AK98_MCGPIO_14		AK98_GPIO_GROUP5_NO(14)
#define AK98_MCGPIO_15		AK98_GPIO_GROUP5_NO(15)
#define AK98_MCGPIO_16		AK98_GPIO_GROUP5_NO(16)
#define AK98_MCGPIO_17		AK98_GPIO_GROUP5_NO(17)
#define AK98_MCGPIO_18		AK98_GPIO_GROUP5_NO(18)
#define AK98_MCGPIO_19		AK98_GPIO_GROUP5_NO(19)

/* AW9523 expander GPIO pins */
#define AW9523_GPIO_P00		AK98_GPIO_GROUP6_NO(0)
#define AW9523_GPIO_P01		AK98_GPIO_GROUP6_NO(1)
#define AW9523_GPIO_P02		AK98_GPIO_GROUP6_NO(2)
#define AW9523_GPIO_P03		AK98_GPIO_GROUP6_NO(3)
#define AW9523_GPIO_P04		AK98_GPIO_GROUP6_NO(4)
#define AW9523_GPIO_P05		AK98_GPIO_GROUP6_NO(5)
#define AW9523_GPIO_P06		AK98_GPIO_GROUP6_NO(6)
#define AW9523_GPIO_P07		AK98_GPIO_GROUP6_NO(7)
#define AW9523_GPIO_P10		AK98_GPIO_GROUP6_NO(8)
#define AW9523_GPIO_P11		AK98_GPIO_GROUP6_NO(9)
#define AW9523_GPIO_P12		AK98_GPIO_GROUP6_NO(10)
#define AW9523_GPIO_P13		AK98_GPIO_GROUP6_NO(11)
#define AW9523_GPIO_P14		AK98_GPIO_GROUP6_NO(12)
#define AW9523_GPIO_P15		AK98_GPIO_GROUP6_NO(13)
#define AW9523_GPIO_P16		AK98_GPIO_GROUP6_NO(14)
#define AW9523_GPIO_P17		AK98_GPIO_GROUP6_NO(15)

#define AK98_GPIO_MAX		AW9523_GPIO_P17

#undef REG32
#define REG32(_reg)	(*(volatile unsigned long *)(_reg))

#undef REG16 
#define REG16(_reg)	(*(volatile unsigned short *)(_reg))


typedef enum {
	 ePIN_AS_GPIO = 0,			 // All pin as gpio
	 ePIN_AS_PCM,				 // share pin as PCM
	 ePIN_AS_JTAG,				 // share pin as JTAG
	 ePIN_AS_RTCK, 		   	  	 // share pin as watch dog
	 ePIN_AS_I2S,				 // share pin as I2S
	 ePIN_AS_USB_OTG,			 // share pin as USB OTG
	 ePIN_AS_PWM1,				 // share pin as PWM1
	 ePIN_AS_PWM2,				 // share pin as PWM2
	 ePIN_AS_PWM3,				 // share pin as PWM3
	 ePIN_AS_PWM4,				 // share pin as PWM4
	 ePIN_AS_UART1, 			 // share pin as UART1
	 ePIN_AS_UART2, 			 // share pin as UART2
	 ePIN_AS_UART3, 			 // share pin as UART3
	 ePIN_AS_UART4, 			 // share pin as UART4
	 ePIN_AS_NFC, 		 		 // share pin as NFC
	 ePIN_AS_NFC_EXT, 	 		 // share pin as NFC extend data line
	 ePIN_AS_CAMERA,			 // share pin as CAMERA
	 ePIN_AS_LCD,				 // share pin as LCD
	 ePIN_AS_LCD_EXT,			 // share pin as LCD Extend dataline
	 ePIN_AS_SDMMC1,			 // share pin as MDAT1, 8 lines
	 ePIN_AS_SDMMC2,			 // share pin as MDAT2, 4lines
	 ePIN_AS_SDIO,				 // share pin as SDIO
	 ePIN_AS_SPI1,				 // share pin as SPI1
	 ePIN_AS_SPI2,				 // share pin as SPI2
	 ePIN_AS_MAC,				 // share pin as Ethernet MAC
	 ePIN_AS_I2C,				 // share pin as I2C
	 ePIN_AS_RAM,				 // share pin as RAM Controller
	 ePIN_AS_HOSTUSB, 			 // share pin as USB Host 
	 ePIN_AS_CLK12MO,
	 ePIN_AS_CLK32KO,
 
	 ePIN_AS_DUMMY
 } T_GPIO_SHAREPIN_CFG ;

typedef enum {
	SHARE_CONFG1 = 0,
	SHARE_CONFG2
} T_SHARE_CONFG;

typedef enum  {
	SHARE_CFG1 = 0,   ///share config1
	SHARE_CFG2,       //share config2
	SHARE_CFGBOTH,    //share config1 and share config2 as used
	EXIT_CFG		  
}T_SHARE_CFG;

struct gpio_callbacks_info 
{
	unsigned int pin_start;
	unsigned int pin_end;

	int (*setpin_as_gpio) (unsigned int pin);
	int (*gpio_pullup)(unsigned int pin, unsigned char enable);
	int (*gpio_pulldown)(unsigned int pin, unsigned char enable);

	int (*gpio_dircfg)(unsigned int pin, unsigned int to);	
	
	int (*gpio_intcfg)(unsigned int pin, unsigned int enable);
	int (*gpio_set_intpol)(unsigned int pin, unsigned int level);

	int (*gpio_setpin)(unsigned int pin, unsigned int to);
	int (*gpio_getpin)(unsigned int pin);

	int (*gpio_to_irq)(unsigned int pin);
	int (*irq_to_gpio)(unsigned int irq);

};

struct gpio_info
{
	int pin;
	char pulldown;
	char pullup;
	char value;
	char dir;
	char int_pol;
	
};

void ak98_gpio_set(const struct gpio_info *info);

/* set gpio's wake up polarity*/

void ak98_gpio_wakeup_pol(unsigned int pin, unsigned char pol);
/* enable/disable gpio wake up function*/
int ak98_gpio_wakeup(unsigned int pin, unsigned char enable);

void ak98_sharepin_cfg1(unsigned char to, unsigned char offset, T_SHARE_CONFG conf);
void ak98_sharepin_cfg2(unsigned long mask, unsigned long value, unsigned char offset);


int ak98_gpio_pullup(unsigned int pin, unsigned char enable);
int ak98_gpio_pulldown(unsigned int pin, unsigned char enable);
int ak98_setpin_as_gpio (unsigned int pin);
void ak98_group_config(T_GPIO_SHAREPIN_CFG mod_name);




/* new version of ak98_gpio_cfgpin */
int ak98_gpio_dircfg(unsigned int pin, unsigned int to);

int ak98_gpio_setpin(unsigned int pin, unsigned int to);
int ak98_gpio_getpin(unsigned int pin);

/* new version of  ak98_gpio_inten*/
int ak98_gpio_intcfg(unsigned int pin, unsigned int enable);
/* new version of  ak98_gpio_intpol*/
int ak98_gpio_set_intpol(unsigned int pin, unsigned int level);


int ak98_gpio_to_irq(unsigned int pin);
int ak98_irq_to_gpio(unsigned int irq);



/*  to support backward compatibility*/
int ak98_gpio_cfgpin(unsigned int pin, unsigned int to);
int ak98_gpio_inten(unsigned int pin, unsigned int enable);
int ak98_gpio_intpol(unsigned int pin, unsigned int level);

extern int ak98_gpio_request(unsigned long gpio, const char *label);
extern void ak98_gpio_free(unsigned long gpio);


#endif /*_AK98_GPIO_H_*/

