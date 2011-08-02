/*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>

void ak880x_sharepin_cfg1(unsigned char to, unsigned char offset)
{
	void __iomem *base = AK88_SHAREPIN_CON1;
	unsigned long val = 0;
	unsigned long flags;
	if (offset > 31) {
		/*print */
		return;
	}

	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == to)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_sharepin_cfg1);

void ak880x_sharepin_cfg2(unsigned char to, unsigned char offset)
{
	void __iomem *base = AK88_SHAREPIN_CON2;
	unsigned long val = 0;
	unsigned long flags;

	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == offset) {
		if (0 == to)
			val &= ~1;
		else
			val |= 1;
	} else {
		val &= ~(3 << offset);
		val |= (to << offset);
	}
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_sharepin_cfg2);

void ak880x_gpio_cfgpin(unsigned int pin, unsigned int function)
{

	void __iomem *base = AK88_GPIO_DIR_BASE(pin);
	unsigned long val = 0;
	unsigned long flags;
	unsigned int offset = ((pin) & 31);
	if (pin >= (96 + 21)) {
		/*printk */
		return;
	}
	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == function)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_gpio_cfgpin);

unsigned int ak880x_gpio_getcfg(unsigned int pin)
{
	void __iomem *base = AK88_GPIO_DIR_BASE(pin);
	unsigned int offset = ((pin) & 31);
	return __raw_readl(base) & (1 << offset);
}

EXPORT_SYMBOL(ak880x_gpio_getcfg);

void ak880x_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK88_GPIO_OUT_BASE(pin);
	unsigned long val = 0;
	unsigned long flags;
	unsigned int offset = ((pin) & 31);
	if (pin >= (96 + 21)) {
		/*printk */
		return;
	}
	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == to)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_gpio_setpin);

unsigned int ak880x_gpio_getpin(unsigned int pin)
{
	void __iomem *base = NULL;
	unsigned int offset = 0;

#if	defined(CONFIG_CPU_AK7801)
	/* only for ak7801, ak880x fixed */
	if (pin > AK88_DGPIO_36)
		pin -= 3;
	else if ((pin >= AK88_DGPIO_34) && (pin <= AK88_DGPIO_36))
		return 0;
#endif

	base = AK88_GPIO_IN_BASE(pin);
	offset = ((pin) & 31);

	return __raw_readl(base) & (1 << offset);
}

EXPORT_SYMBOL(ak880x_gpio_getpin);

void ak880x_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK88_PPU_PPD_BASE(pin);
	unsigned long val = 0;
	unsigned long flags;
	unsigned int offset = ((pin) & 31);
	if (pin > 128) {
		/*printk */
		return;
	}
	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == to)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_gpio_pullup);

void ak880x_gpio_inten(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK88_GPIO_INTEN_BASE(pin);
	unsigned long val = 0;
	unsigned long flags;
	unsigned int offset = ((pin) & 31);
	if (pin > 128) {
		/*printk */
		return;
	}
	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == to)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_gpio_inten);

void ak880x_gpio_intpol(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK88_GPIO_INTPOL_BASE(pin);
	unsigned long val = 0;
	unsigned long flags;
	unsigned int offset = ((pin) & 31);
	if (pin >= (96 + 21)) {
		/*printk */
		return;
	}
	local_irq_save(flags);
	val = __raw_readl(base);
	if (0 == to)
		val &= ~(1 << offset);
	else
		val |= (1 << offset);
	__raw_writel(val, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_gpio_intpol);

void ak880x_ioctl(unsigned int offset, const unsigned int to)
{
	unsigned long val = 0;
	unsigned long flags;

	local_irq_save(flags);

	if (offset < 32) {
		val = __raw_readl(AK88_IO_CON1);
		if (0 == to)
			val &= ~(1 << offset);
		else
			val |= (1 << offset);
		__raw_writel(val, AK88_IO_CON1);
	} else {
		offset -= 32;
		val = __raw_readl(AK88_IO_CON2);
		if (0 == to)
			val &= ~(1 << offset);
		else
			val |= (1 << offset);
		__raw_writel(val, AK88_IO_CON2);
	}

	local_irq_restore(flags);
}

EXPORT_SYMBOL(ak880x_ioctl);

unsigned int ak880x_gpio_to_irq(unsigned int pin)
{
	return (IRQ_GPIO_0 + (pin - AK88_GPIO_0));
}

EXPORT_SYMBOL(ak880x_gpio_to_irq);

unsigned int ak880x_irq_to_gpio(unsigned int irq)
{
	return (AK88_GPIO_0 + (irq - IRQ_GPIO_0));
}

EXPORT_SYMBOL(ak880x_irq_to_gpio);
