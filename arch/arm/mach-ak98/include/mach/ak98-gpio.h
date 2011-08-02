/*************************************************************************
*   Filename: arch/arm/mach-ak98/include/mach/gpio.h
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
**************************************************************************/

#ifndef _ATHENA_GPIO_H_
#define _ATHENA_GPIO_H_ 

#include "map.h"
#include <linux/gpio.h>
#include <mach/gpio.h>

#ifdef CONFIG_AK9801_ATHENA
	#define CONFIG_AK9801
#endif

#ifdef CONFIG_AK9805_TV908
	#define CONFIG_AK9805
#endif

#ifdef CONFIG_AK9805_MP5
	#define CONFIG_AK9805
#endif



#define GPIO_UPLIMIT			116
#define MCGPIO_UPLIMIT			19



#define	GPIO_PIN_MODE_GPIO		0
#define	GPIO_PIN_MODE_INT		1


#define	ATTR_FIXED_1			1
#define	ATTR_FIXED_0			0
#define	PIN_ATTE_LINE			6

#define GPIO_ATTR_UNSUPPORTED   0xffff
#define END_FLAG                0xff
#define INVALID_GPIO			0xfe



/******************  access gpio register addr **********************/
#define AK98_GPIO_DIR1			(AK98_VA_SYSCTRL + 0x007C)
#define AK98_GPIO_DIR2			(AK98_VA_SYSCTRL + 0x0084)
#define AK98_GPIO_DIR3			(AK98_VA_SYSCTRL + 0x008C)
#define AK98_GPIO_DIR4			(AK98_VA_SYSCTRL + 0x0094)
#define AK98_GPIO_DIR5			(AK98_VA_SYSCTRL + 0x011C)

#define AK98_GPIO_OUT1			(AK98_VA_SYSCTRL + 0x0080)  
#define AK98_GPIO_OUT2			(AK98_VA_SYSCTRL + 0x0088) 
#define AK98_GPIO_OUT3			(AK98_VA_SYSCTRL + 0x0090)  
#define AK98_GPIO_OUT4			(AK98_VA_SYSCTRL + 0x0098)
#define AK98_GPIO_OUT5			(AK98_VA_SYSCTRL + 0x0118)

#define AK98_GPIO_IN1         	(AK98_VA_SYSCTRL + 0x00BC) 
#define AK98_GPIO_IN2         	(AK98_VA_SYSCTRL + 0x00C0) 
#define AK98_GPIO_IN3         	(AK98_VA_SYSCTRL + 0x00C4)  
#define AK98_GPIO_IN4        	(AK98_VA_SYSCTRL + 0x00C8)
#define AK98_GPIO_IN5       	(AK98_VA_SYSCTRL + 0x0124)

#define AK98_GPIO_INTEN1        (AK98_VA_SYSCTRL + 0x00E0) 
#define AK98_GPIO_INTEN2        (AK98_VA_SYSCTRL + 0x00E4) 
#define AK98_GPIO_INTEN3        (AK98_VA_SYSCTRL + 0x00E8) 
#define AK98_GPIO_INTEN4        (AK98_VA_SYSCTRL + 0x00EC) 

#define AK98_GPIO_INTPOL1      	(AK98_VA_SYSCTRL + 0x00F0)
#define AK98_GPIO_INTPOL2      	(AK98_VA_SYSCTRL + 0x00F4) 
#define AK98_GPIO_INTPOL3      	(AK98_VA_SYSCTRL + 0x00F8) 
#define AK98_GPIO_INTPOL4      	(AK98_VA_SYSCTRL + 0x00FC) 

#define AK98_PPU_PPD1           (AK98_VA_SYSCTRL + 0x009C)
#define AK98_PPU_PPD2           (AK98_VA_SYSCTRL + 0x00A0)
#define AK98_PPU_PPD3           (AK98_VA_SYSCTRL + 0x00A4)
#define AK98_PPU_PPD4           (AK98_VA_SYSCTRL + 0x00A8)
#define AK98_MCGPIO_PPU_PPD		(AK98_VA_SYSCTRL + 0x0120)

#define AK98_IO_CON1            (AK98_VA_SYSCTRL + 0x00D4)
#define AK98_IO_CON2            (AK98_VA_SYSCTRL + 0x00D8)

#define AK98_SHAREPIN_CON1		(AK98_VA_SYSCTRL + 0x0078)
#define AK98_SHAREPIN_CON2		(AK98_VA_SYSCTRL + 0x0074)

#define AK98_WGPIO_POLARITY		(AK98_VA_SYSCTRL + 0x3C)
#define AK98_WGPIO_CLEAR		(AK98_VA_SYSCTRL + 0x40)
#define AK98_WGPIO_ENABLE		(AK98_VA_SYSCTRL + 0x44)
#define AK98_WGPIO_STATUS		(AK98_VA_SYSCTRL + 0x48)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AK98_GPIO_DIR_BASE(pin)		(((pin)>>5)*8 + AK98_GPIO_DIR1 )
#define AK98_GPIO_OUT_BASE(pin)		(((pin)>>5)*8 + AK98_GPIO_OUT1 )
#define AK98_GPIO_IN_BASE(pin)		(((pin)>>5)*4 + AK98_GPIO_IN1 )
#define AK98_GPIO_INTEN_BASE(pin)	(((pin)>>5)*4 + AK98_GPIO_INTEN1 )
#define AK98_GPIO_INTPOL_BASE(pin)	(((pin)>>5)*4 + AK98_GPIO_INTPOL1 )
#define AK98_PPU_PPD_BASE(pin)		(((pin)>>5)*4 + AK98_PPU_PPD1 )
/* MCGPIO group */
#define AK98_MCGPIO_DIR_BASE(pin)	(((pin)>>5)*4 + AK98_GPIO_DIR5)
#define AK98_MCGPIO_OUT_BASE(pin)	(((pin)>>5)*4 + AK98_GPIO_OUT5)
#define AK98_MCGPIO_IN_BASE(pin)	(((pin)>>5)*4 + AK98_GPIO_IN5)
#define AK98_MCGPIO_PPU_PPD_BASE(pin)	(((pin)>>5)*4 + AK98_MCGPIO_PPU_PPD)




struct gpio_sharepin_cfg {
    T_GPIO_SHAREPIN_CFG func_module;
	T_SHARE_CFG share_config;
    unsigned long reg1_bit_mask;
    unsigned long reg1_bit_value;
    unsigned long reg2_bit_mask;
    unsigned long reg2_bit_value;
};

typedef enum {
	 PULLUP = 0,
	 PULLDOWN,
	 PULLUPDOWN,
	 UNDEFINED
 } T_GPIO_TYPE ;

typedef enum {
    GPIO_ATTR_IE = 1,   ///<input enable
    GPIO_ATTR_PE,       ///<pullup/pulldown enable
    GPIO_ATTR_SL,       ///<slew rate
    GPIO_ATTR_DS,       ///<drive strength
    GPIO_ATTR_PS        ///<pullup/pulldown selection
}T_GPIO_PIN_ATTR;

struct sharepin_group {
    unsigned char gpio_start;
    unsigned char gpio_end;
    unsigned char bit;
};

typedef struct {
    unsigned char gpio_start;
    unsigned char gpio_end;
    unsigned char bit;
}
T_SHARE_CFG_GPIO;

struct t_gpio_wakeup_cfg {
	unsigned char gpio_start;
	unsigned char gpio_end;
	unsigned char start_bit;
};


int g_ak98_setpin_as_gpio(unsigned int pin);
void g_ak98_setpin_attribute(unsigned int pin, 
	T_GPIO_PIN_ATTR attr, unsigned char enable);
int g_ak98_gpio_pullup(unsigned int pin, unsigned char enable);
int g_ak98_gpio_pulldown(unsigned int pin, unsigned char enable);

void g_ak98_setgroup_attribute(T_GPIO_SHAREPIN_CFG mod_name);
int g_ak98_gpio_cfgpin(unsigned int pin, unsigned int to);
int g_ak98_gpio_setpin(unsigned int pin, unsigned int to);
 int g_ak98_gpio_getpin(unsigned int pin);
int g_ak98_gpio_inten(unsigned int pin, unsigned int enable);

int g_ak98_gpio_intpol(unsigned int pin, unsigned int level);
  int g_ak98_gpio_to_irq(unsigned int pin);
  int g_ak98_irq_to_gpio(unsigned int irq);


 int mc_ak98_setpin_as_mcgpio(unsigned int pin);
 int mc_ak98_mcgpio_cfgpin(unsigned int pin, unsigned int to);
 int mc_ak98_mcgpio_setpin(unsigned int pin, unsigned int to);
  int mc_ak98_mcgpio_getpin(unsigned int pin);
 int mc_ak98_mcgpio_pulldown(unsigned int pin, unsigned char enable);


#endif /*_AK98_GPIO_H_*/

