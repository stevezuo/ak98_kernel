#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/ak98-gpio.h>


#include "cpu.h"
#include "irq.h"

#define AK98_CLKDIV1		(AK98_VA_SYSCTRL + 0x04)
#define AK98_IRQ_MASK		(AK98_VA_SYSCTRL + 0x34)
#define AK98_FIQ_MASK		(AK98_VA_SYSCTRL + 0x38)
#define AK98_INT_STATUS		(AK98_VA_SYSCTRL + 0xCC)
#define	AK98_SYSCTRL_INT_CTRL	(AK98_VA_SYSCTRL + 0x4C)

#define AK98_GPIO_INT_CTRL0	(AK98_VA_SYSCTRL + 0xE0)
#define AK98_GPIO_INT_CTRL1	(AK98_VA_SYSCTRL + 0xE4)
#define AK98_GPIO_INT_CTRL2	(AK98_VA_SYSCTRL + 0xE8)
#define AK98_GPIO_INT_CTRL3	(AK98_VA_SYSCTRL + 0xEC)

#define AK98_GPIO_INPUT0	(AK98_VA_SYSCTRL + 0xBC)
#define AK98_GPIO_INPUT1	(AK98_VA_SYSCTRL + 0xC0)
#define AK98_GPIO_INPUT2	(AK98_VA_SYSCTRL + 0xC4)
#define AK98_GPIO_INPUT3	(AK98_VA_SYSCTRL + 0xC8)

#define AK98_GPIO_INTP0		(AK98_VA_SYSCTRL + 0xF0)
#define AK98_GPIO_INTP1		(AK98_VA_SYSCTRL + 0xF4)
#define AK98_GPIO_INTP2		(AK98_VA_SYSCTRL + 0xF8)
#define AK98_GPIO_INTP3		(AK98_VA_SYSCTRL + 0xFC)

#define AK98_L2MEM_IRQ_ENABLE	(AK98_VA_L2CTRL + 0x9C)

/*
 * Disable interrupt number "irq"
 */
static void ak98_mask_irq(unsigned int irq)
{
	unsigned long mask;
	unsigned long bitval = (1UL << irq);

	mask = __raw_readl(AK98_IRQ_MASK);

	__raw_writel(mask & ~bitval, AK98_IRQ_MASK);
}

/*
 * Enable interrupt number "irq"
 */
static void ak98_unmask_irq(unsigned int irq)
{
	unsigned long mask;
	unsigned long bitval = (1UL << irq);

	mask = __raw_readl(AK98_IRQ_MASK);

	__raw_writel(mask | bitval, AK98_IRQ_MASK);
}

static struct irq_chip ak98_irq_chip = {
	.name = "ak98xx",
	.mask_ack = ak98_mask_irq,
	.mask = ak98_mask_irq,
	.unmask = ak98_unmask_irq,
};

static void sysctrl_mask_irq(unsigned int irq)
{
	unsigned long sysctrl;
	unsigned long bitval = ~(1 << (irq - IRQ_TOUCHPANEL));

	sysctrl = __raw_readl(AK98_SYSCTRL_INT_CTRL);

	__raw_writel(sysctrl & bitval, AK98_SYSCTRL_INT_CTRL);
}

static void sysctrl_unmask_irq(unsigned int irq)
{
	unsigned long sysctrl;
	unsigned long bitval = (1 << (irq - IRQ_TOUCHPANEL));

	sysctrl = __raw_readl(AK98_SYSCTRL_INT_CTRL);

	__raw_writel(sysctrl | bitval, AK98_SYSCTRL_INT_CTRL);
}

static int sysctrl_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long clkdiv1;

	if (irq == IRQ_RTC_ALARM) {

		clkdiv1 = __raw_readl(AK98_CLKDIV1);

		if (on == 1)
			clkdiv1 |= (1 << 16);
		else
			clkdiv1 &= ~(1 << 16);

		__raw_writel(clkdiv1, AK98_CLKDIV1);

		return 0;
	}

	return 0;
}

static struct irq_chip ak98_sysctrl_chip = {
	.name = "ak-sysctrl",
	.mask_ack = sysctrl_mask_irq,
	.mask = sysctrl_mask_irq,
	.unmask = sysctrl_unmask_irq,
	.set_wake = sysctrl_set_wake,
};

static void ak98_sysctrl_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long regval;
	unsigned long intpnd;
	unsigned int offset = 0;

	regval = __raw_readl(AK98_SYSCTRL_INT_CTRL);

	intpnd = (regval & 0x7FF) & ((regval & 0x7FF0000) >> 16);

	offset = 0;
	for (offset = 0; intpnd && offset < 32; offset++) {

		if (intpnd & (1 << offset))
			intpnd &= ~(1 << offset);
		else
			continue;

		irq = AK98_SYSCTRL_IRQ(offset);

		generic_handle_irq(irq);
	}
}

static void ak98_gpioirq_mask(unsigned int irq)
{
	void __iomem *gpio_ctrl = AK98_GPIO_INT_CTRL0;
	unsigned long regval;

	irq -= IRQ_GPIO_0;
	gpio_ctrl += (irq / 32) * 4;

	regval = __raw_readl(gpio_ctrl);
	regval &= ~(1 << (irq % 32));

	__raw_writel(regval, gpio_ctrl);
}

static int ak98_gpioirq_set_type(unsigned int irq, unsigned int type)
{
	void __iomem *gpio_ctrl = AK98_GPIO_INTP0;
	unsigned long regval;

	irq -= IRQ_GPIO_0;
	gpio_ctrl += (irq / 32) * 4;

	regval = __raw_readl(gpio_ctrl);

	if (type == IRQ_TYPE_LEVEL_HIGH)
		regval &= ~(1 << (irq % 32));
	else if (type == IRQ_TYPE_LEVEL_LOW)
		regval |= (1 << (irq % 32));
	else {
		printk("Not support irq type\n");
		return -1;
	}

	/* printk("0x%x, %d\n", gpio_ctrl, irq%32); */

	__raw_writel(regval, gpio_ctrl);

	return 0;
}

static void ak98_gpioirq_unmask(unsigned int irq)
{
	void __iomem *gpio_ctrl = AK98_GPIO_INT_CTRL0;
	unsigned long regval;

	irq -= IRQ_GPIO_0;
	gpio_ctrl += (irq / 32) * 4;

	regval = __raw_readl(gpio_ctrl);
	regval |= (1 << (irq % 32));

	/* printk("0x%x, %d\n", gpio_ctrl, irq%32); */

	__raw_writel(regval, gpio_ctrl);
}

static int ak98_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long wgpio_enable;

	wgpio_enable = __raw_readl(AK98_WGPIO_ENABLE);

	if (irq >= IRQ_GPIO_5 && irq <= IRQ_GPIO_7)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_5));

	else if (irq >= IRQ_GPIO_10 && irq <= IRQ_GPIO_13)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_10 + 3));

	else if (irq >= IRQ_GPIO_16 && irq <= IRQ_GPIO_19)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_16 + 7));

	else if (irq >= IRQ_GPIO_24 && irq <= IRQ_GPIO_27)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_24 + 11));

	else if (irq >= IRQ_GPIO_72 && irq <= IRQ_GPIO_75)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_72 + 15));

	else if (irq >= IRQ_GPIO_104 && irq <= IRQ_GPIO_107)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_104 + 19));
	
	else if (irq >= IRQ_GPIO_109 && irq <= IRQ_GPIO_113)
		wgpio_enable |= (1 << (irq - IRQ_GPIO_109 + 24));
	
	else if (irq == IRQ_GPIO_84)
		wgpio_enable |= (1 << 23);
	
	else {
		printk("Not WGPIO IRQ: %d\n", irq);
		return -1;
	}

	__raw_writel(wgpio_enable, AK98_WGPIO_ENABLE);

	return 0;
}

static struct irq_chip ak98_gpioirq_chip = {
	.name = "gpio_irq",
	.mask_ack = ak98_gpioirq_mask,
	.mask = ak98_gpioirq_mask,
	.set_type = ak98_gpioirq_set_type,
	.unmask = ak98_gpioirq_unmask,
	.set_wake = ak98_gpio_irq_set_wake,
};

static void ak98_gpio_irqhandler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long enabled_irq;
	unsigned int i;
	unsigned int off;

	for (i = 0; i < 4; i++) {
		enabled_irq = __raw_readl(AK98_GPIO_INT_CTRL0 + i * 4);

		while (enabled_irq) {
			off = __ffs(enabled_irq);
			enabled_irq &= ~(1 << off);
			if (test_bit(off, AK98_GPIO_INTP0 + i * 4) !=
			    test_bit(off, AK98_GPIO_INPUT0 + i * 4)) {
				irq = IRQ_GPIO_0 + i * 32 + off;
				generic_handle_irq(irq);
			}
		}
	}
}

/* ak98_init_irq
 *
 * Initialise AK7801 IRQ system
 */

void __init ak98_init_irq(void)
{
	int i;

	/* 1st, clear all interrupts */
	__raw_readl(AK98_INT_STATUS);
	__raw_readl(AK98_SYSCTRL_INT_CTRL);

	/* 2nd, mask all interrutps */
	__raw_writel(0x0, AK98_IRQ_MASK);
	__raw_writel(0x0, AK98_FIQ_MASK);
	__raw_writel(0x0, AK98_SYSCTRL_INT_CTRL);

	/* mask all gpio interrupts */
	__raw_writel(0x0, AK98_GPIO_INT_CTRL0);
	__raw_writel(0x0, AK98_GPIO_INT_CTRL1);
	__raw_writel(0x0, AK98_GPIO_INT_CTRL2);
	__raw_writel(0x0, AK98_GPIO_INT_CTRL3);

	/* mask all l2 interrupts */
	__raw_writel(0x0, AK98_L2MEM_IRQ_ENABLE);

	for (i = 0; i <= IRQ_PCM; i++) {
		set_irq_chip(i, &ak98_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(IRQ_SYSCTRL, ak98_sysctrl_handler);

	for (i = IRQ_TOUCHPANEL; i <= IRQ_PEN_DOWN_FILTER; i++) {
		set_irq_chip(i, &ak98_sysctrl_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(IRQ_GPIO, ak98_gpio_irqhandler);

	for (i = IRQ_GPIO_0; i <= IRQ_GPIO_116; i++) {
		set_irq_chip(i, &ak98_gpioirq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
}
