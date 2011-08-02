/*
 *  arch/arm/mach-ak98/gpio.c
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/ak98-gpio.h>
#include <linux/i2c/aw9523.h>


static struct gpio_callbacks_info ak98_gpio_callbacks[] =
{
		{
			.pin_start 		= AK98_GPIO_0,
			.pin_end 		= AK98_GPIO_116,
			.setpin_as_gpio	= g_ak98_setpin_as_gpio,
			.gpio_pullup	= g_ak98_gpio_pullup,
			.gpio_pulldown	= g_ak98_gpio_pulldown,
			.gpio_dircfg	= g_ak98_gpio_cfgpin,
			.gpio_intcfg	= g_ak98_gpio_inten,
			.gpio_set_intpol= g_ak98_gpio_intpol,
			.gpio_setpin	= g_ak98_gpio_setpin,
			.gpio_getpin	= g_ak98_gpio_getpin,
			.gpio_to_irq	= g_ak98_gpio_to_irq,
			.irq_to_gpio	= g_ak98_irq_to_gpio,			
		},
		{
			.pin_start 		= AK98_MCGPIO_0,
			.pin_end 		= AK98_MCGPIO_19,
			.setpin_as_gpio	= mc_ak98_setpin_as_mcgpio,
			.gpio_pullup	= NULL,			
			.gpio_pulldown	= mc_ak98_mcgpio_pulldown,
			.gpio_dircfg	= mc_ak98_mcgpio_cfgpin,
			.gpio_intcfg	= NULL,
			.gpio_set_intpol= NULL,
			.gpio_setpin	= mc_ak98_mcgpio_setpin,
			.gpio_getpin	= mc_ak98_mcgpio_getpin,
			.gpio_to_irq	= NULL,
			.irq_to_gpio	= NULL,			
		},
	#ifdef CONFIG_I2C_AW9523_GPIO
		{
			.pin_start 		= AW9523_GPIO_P00,
			.pin_end 		= AW9523_GPIO_P17,
			.setpin_as_gpio	= NULL,
			.gpio_pullup	= NULL,
			.gpio_pulldown	= NULL,
			.gpio_dircfg	= aw9523_gpio_dircfg,
			.gpio_intcfg	= aw9523_gpio_intcfg,
			.gpio_set_intpol= NULL,
			.gpio_setpin	= aw9523_gpio_setpin,
			.gpio_getpin	= aw9523_gpio_getpin,
			.gpio_to_irq	= aw9523_gpio_to_irq,
			.irq_to_gpio	= aw9523_irq_to_gpio,			
		},
	#endif
};

#define CALLBACKS_TBL ak98_gpio_callbacks
#define GET_GPIO_CALLBACKS(pfun, pin, name)\
	do{\
	int _i_name;\
	pfun=NULL;\
	for (_i_name=0; _i_name<ARRAY_SIZE(CALLBACKS_TBL); _i_name++)\
	if (pin>= CALLBACKS_TBL[_i_name].pin_start && pin <= CALLBACKS_TBL[_i_name].pin_end)\
	{ pfun = CALLBACKS_TBL[_i_name].name; break;}\
	}while(0);
										


int ak98_setpin_as_gpio (unsigned int pin)
{
	int (*pfunc)(unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, setpin_as_gpio);
	if (pfunc)
		return pfunc(pin);
	return 0;
}
EXPORT_SYMBOL(ak98_setpin_as_gpio);


int ak98_gpio_pullup(unsigned int pin, unsigned char enable)
{
	int (*pfunc)(unsigned int, unsigned char);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_pullup);
	if (pfunc)
		return pfunc(pin, enable);	

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_pullup);

int ak98_gpio_pulldown(unsigned int pin, unsigned char enable)
{
	int (*pfunc)(unsigned int, unsigned char);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_pulldown);
	if (pfunc)
		return pfunc(pin, enable);	

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_pulldown);

/*  direction configuration  */
int ak98_gpio_dircfg(unsigned int pin, unsigned int direction)
{
	int (*pfunc)(unsigned int, unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_dircfg);
	if (pfunc)
		return pfunc(pin, direction);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_dircfg);
int ak98_gpio_cfgpin(unsigned int pin, unsigned int to)
{
		return ak98_gpio_dircfg(pin, to);
}
EXPORT_SYMBOL(ak98_gpio_cfgpin);


/* interrupt configuration */
int ak98_gpio_intcfg(unsigned int pin, unsigned int enable)
{
	int (*pfunc)(unsigned int, unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_intcfg);
	if (pfunc)
		return pfunc(pin, enable);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_intcfg);
int ak98_gpio_inten(unsigned int pin, unsigned int enable)
{
	return ak98_gpio_intcfg(pin, enable);
}
EXPORT_SYMBOL(ak98_gpio_inten);



/*  interrupt polarity configuration */
int ak98_gpio_set_intpol(unsigned int pin, unsigned int level)
{
	int (*pfunc)(unsigned int, unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_set_intpol);
	if (pfunc)
		return pfunc(pin, level);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_set_intpol);
int ak98_gpio_intpol(unsigned int pin, unsigned int level)
{
	return ak98_gpio_set_intpol(pin, level);
}
EXPORT_SYMBOL(ak98_gpio_intpol);	


int ak98_gpio_setpin(unsigned int pin, unsigned int to)
{
	int (*pfunc)(unsigned int, unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_setpin);
	if (pfunc)
		return pfunc(pin, to);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_setpin);

 int ak98_gpio_getpin(unsigned int pin)
{	
	 int (*pfunc)(unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_getpin);
	if (pfunc)
		return pfunc(pin);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_getpin);



 int ak98_gpio_to_irq(unsigned int pin)
{
	int (*pfunc)(unsigned int);
	GET_GPIO_CALLBACKS(pfunc, pin, gpio_to_irq);
	if (pfunc)
		return pfunc(pin);

	return 0;
}
EXPORT_SYMBOL(ak98_gpio_to_irq);

 int ak98_irq_to_gpio(unsigned int irq)
{
	if ( irq >= IRQ_GPIO_0 && irq<= IRQ_GPIO_116)
		return AK98_GPIO_0 + (irq-IRQ_GPIO_0);
	else if (irq >= IRQ_AW9523_P00 && irq <= IRQ_AW9523_P17)
		return AW9523_GPIO_P00 + (irq-IRQ_AW9523_P00);
	else
		panic("wrong irq number %u passed to ak98_irq_to_gpio.\n", irq);

	return -1;
}
EXPORT_SYMBOL(ak98_irq_to_gpio);


void ak98_gpio_set(const struct gpio_info *info)
{
	if ( ! (info->pin >= AK98_GPIO_MIN && info->pin <= AK98_GPIO_MAX ))
		return ;
	
	ak98_setpin_as_gpio(info->pin);
	if (info->dir == AK98_GPIO_DIR_OUTPUT || info->dir == AK98_GPIO_DIR_INPUT)
		ak98_gpio_dircfg(info->pin, info->dir);
	if (info->pullup == AK98_PULLUP_ENABLE || info->pullup == AK98_PULLUP_DISABLE)
		ak98_gpio_pullup(info->pin, info->pullup);
	if (info->pulldown == AK98_PULLDOWN_ENABLE || info->pulldown == AK98_PULLDOWN_DISABLE)
		ak98_gpio_pulldown(info->pin, info->pulldown);
	if (info->value == AK98_GPIO_HIGH || info->value == AK98_GPIO_LOW)
		ak98_gpio_setpin(info->pin, info->value);
	if (info->int_pol == AK98_GPIO_INT_LOWLEVEL || info->int_pol == AK98_GPIO_INT_HIGHLEVEL)
		ak98_gpio_set_intpol(info->pin, info->int_pol);
}
EXPORT_SYMBOL(ak98_gpio_set);

