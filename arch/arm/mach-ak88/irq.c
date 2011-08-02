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

#include "cpu.h"
#include "irq.h"

#define AK88_CLKDIV1		(AK88_VA_SYSCTRL + 0x04)
#define AK88_IRQ_MASK		(AK88_VA_SYSCTRL + 0x34)
#define AK88_FIQ_MASK		(AK88_VA_SYSCTRL + 0x38)
#define AK88_INT_STATUS	(AK88_VA_SYSCTRL + 0xCC)
#define	AK88_SYSCTRL_INT_CTRL	(AK88_VA_SYSCTRL + 0x4C)

#define AK88_GPIO_INT_CTRL0	(AK88_VA_SYSCTRL + 0xE0)
#define AK88_GPIO_INT_CTRL1	(AK88_VA_SYSCTRL + 0xE4)
#define AK88_GPIO_INT_CTRL2	(AK88_VA_SYSCTRL + 0xE8)
#define AK88_GPIO_INT_CTRL3	(AK88_VA_SYSCTRL + 0xEC)

#define AK88_GPIO_INPUT0	(AK88_VA_SYSCTRL + 0xBC)
#define AK88_GPIO_INPUT1	(AK88_VA_SYSCTRL + 0xC0)
#define AK88_GPIO_INPUT2	(AK88_VA_SYSCTRL + 0xC4)
#define AK88_GPIO_INPUT3	(AK88_VA_SYSCTRL + 0xC8)

#define AK88_GPIO_INTP0	(AK88_VA_SYSCTRL + 0xF0)
#define AK88_GPIO_INTP1	(AK88_VA_SYSCTRL + 0xF4)
#define AK88_GPIO_INTP2	(AK88_VA_SYSCTRL + 0xF8)
#define AK88_GPIO_INTP3	(AK88_VA_SYSCTRL + 0xFC)

#define AK88_WGPIO_POLARITY	(AK88_VA_SYSCTRL + 0x3C)
#define AK88_WGPIO_CLEAR	(AK88_VA_SYSCTRL + 0x40)
#define AK88_WGPIO_ENABLE	(AK88_VA_SYSCTRL + 0x44)
#define AK88_WGPIO_STATUS	(AK88_VA_SYSCTRL + 0x48)

#define AK88_L2MEM_IRQ_ENABLE	(AK88_VA_L2CTRL + 0x9C)

/*
 * Disable interrupt number "irq"
 */
static void ak880x_mask_irq(unsigned int irq)
{
	unsigned long mask;
	unsigned long bitval = (1UL << irq);

	mask = __raw_readl(AK88_IRQ_MASK);

	__raw_writel(mask & ~bitval, AK88_IRQ_MASK);
}

/*
 * Enable interrupt number "irq"
 */
static void ak880x_unmask_irq(unsigned int irq)
{
	unsigned long mask;
	unsigned long bitval = (1UL << irq);

	mask = __raw_readl(AK88_IRQ_MASK);

	__raw_writel(mask | bitval, AK88_IRQ_MASK);
	#if 0
	if(irq==IRQ_UART0)
		printk("ak880x_unmask_irq,irq=%d\n",irq);
	if(irq==IRQ_UART3)
		printk("ak880x_unmask_irq,irq=%d\n",irq);
	#endif	

}

static struct irq_chip ak880x_irq_chip = {
	.name = "ak7801",
	.mask_ack = ak880x_mask_irq,
	.mask = ak880x_mask_irq,
	.unmask = ak880x_unmask_irq,
};

static void sysctrl_mask_irq(unsigned int irq)
{
	unsigned long sysctrl;
	unsigned long bitval = ~(1 << (irq - IRQ_TOUCHPANEL));

	sysctrl = __raw_readl(AK88_SYSCTRL_INT_CTRL);

	__raw_writel(sysctrl & bitval, AK88_SYSCTRL_INT_CTRL);
}

static void sysctrl_unmask_irq(unsigned int irq)
{
	unsigned long sysctrl;
	unsigned long bitval = (1 << (irq - IRQ_TOUCHPANEL));

	sysctrl = __raw_readl(AK88_SYSCTRL_INT_CTRL);

	__raw_writel(sysctrl | bitval, AK88_SYSCTRL_INT_CTRL);
}

static int sysctrl_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long clkdiv1;

	if (irq == IRQ_RTC_ALARM) {

		clkdiv1 = __raw_readl(AK88_CLKDIV1);

		if (on == 1)
			clkdiv1 |= (1 << 16);
		else
			clkdiv1 &= ~(1 << 16);

		__raw_writel(clkdiv1, AK88_CLKDIV1);

		return 0;
	}

	return 0;
}

static struct irq_chip ak880x_sysctrl_chip = {
	.name = "ak-sysctrl",
	.mask_ack = sysctrl_mask_irq,
	.mask = sysctrl_mask_irq,
	.unmask = sysctrl_unmask_irq,
	.set_wake = sysctrl_set_wake,
};

static void ak880x_sysctrl_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long regval;
	unsigned long intpnd;
	unsigned int offset = 0;

	regval = __raw_readl(AK88_SYSCTRL_INT_CTRL);

	intpnd = (regval & 0x7FF) & ((regval & 0x7FF0000) >> 16);

	offset = 0;
	for (offset = 0; intpnd && offset < 32; offset++) {

		if (intpnd & (1 << offset))
			intpnd &= ~(1 << offset);
		else
			continue;

		irq = AK88_SYSCTRL_IRQ(offset);

		desc_handle_irq(irq, irq_desc + irq);
	}
}

static void ak880x_gpioirq_mask(unsigned int irq)
{
	void __iomem *gpio_ctrl = AK88_GPIO_INT_CTRL0;
	unsigned long regval;

	irq -= IRQ_GPIO_0;
	gpio_ctrl += (irq / 32) * 4;

	regval = __raw_readl(gpio_ctrl);
	regval &= ~(1 << (irq % 32));

	__raw_writel(regval, gpio_ctrl);
}

static int ak880x_gpioirq_set_type(unsigned int irq, unsigned int type)
{
	void __iomem *gpio_ctrl = AK88_GPIO_INTP0;
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

static void ak880x_gpioirq_unmask(unsigned int irq)
{
	void __iomem *gpio_ctrl = AK88_GPIO_INT_CTRL0;
	unsigned long regval;

	irq -= IRQ_GPIO_0;
	gpio_ctrl += (irq / 32) * 4;

	regval = __raw_readl(gpio_ctrl);
	regval |= (1 << (irq % 32));

	/* printk("0x%x, %d\n", gpio_ctrl, irq%32); */

	__raw_writel(regval, gpio_ctrl);
}

static int ak880x_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long wgpio_enable;

	wgpio_enable = __raw_readl(AK88_WGPIO_ENABLE);

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

	else if (irq >= IRQ_DGPIO_2 && irq <= IRQ_DGPIO_5)
		wgpio_enable |= (1 << (irq - IRQ_DGPIO_2 + 19));

	else if (irq == IRQ_DGPIO_19)
		wgpio_enable |= (1 << 23);

	else if (irq >= IRQ_DGPIO_7 && irq <= IRQ_DGPIO_14)
		wgpio_enable |= (1 << (irq - IRQ_DGPIO_7 + 24));
	else {
		printk("Not WGPIO IRQ: %d\n", irq);
		return -1;
	}

	__raw_writel(wgpio_enable, AK88_WGPIO_ENABLE);

	return 0;
}

static struct irq_chip ak880x_gpioirq_chip = {
	.name = "gpio_irq",
	.mask_ack = ak880x_gpioirq_mask,
	.mask = ak880x_gpioirq_mask,
	.set_type = ak880x_gpioirq_set_type,
	.unmask = ak880x_gpioirq_unmask,
	.set_wake = ak880x_gpio_irq_set_wake,
};

static void ak880x_gpio_irqhandler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long enabled_irq;
	unsigned int i;
	unsigned int off;

	/* printk("Enterring %s\n", __FUNCTION__); */

#if	defined(CONFIG_ARCH_AK7801)

	for (i = 0; i < 3; i++) {
		enabled_irq = __raw_readl(AK88_GPIO_INT_CTRL0 + i * 4);

		while (enabled_irq) {
			off = __ffs(enabled_irq);
			enabled_irq &= ~(1 << off);
			if (test_bit(off, AK88_GPIO_INTP0 + i * 4) !=
			    test_bit(off, AK88_GPIO_INPUT0 + i * 4)) {
				irq = IRQ_GPIO_0 + i * 32 + off;
				desc_handle_irq(irq, irq_desc + irq);
				/* printk("irq: grp %d, %d\n", i, off); */
			}
		}
	}

	/* Group 3 */
	{
		enabled_irq = __raw_readl(AK88_GPIO_INT_CTRL3);

		while (enabled_irq) {

			off = __ffs(enabled_irq);
			enabled_irq &= ~(1 << off);

			if (off < 3) {

				if (test_bit(off, AK88_GPIO_INTP3) !=
				    test_bit(off, AK88_GPIO_INPUT3)) {
					irq = IRQ_DGPIO_31 + off;
					desc_handle_irq(irq, irq_desc + irq);
				}

			} else if (off < 6) {

				printk("Can't handle irq: %d\n",
				       IRQ_DGPIO_31 + off);

			} else {

				if (test_bit(off, AK88_GPIO_INTP3) !=
				    test_bit(off - 3, AK88_GPIO_INPUT3)) {
					irq = IRQ_DGPIO_31 + off;
					desc_handle_irq(irq, irq_desc + irq);
				}
			}
		}
	}

#elif	defined(CONFIG_ARCH_AK88)
	for (i = 0; i < 4; i++) {
		enabled_irq = __raw_readl(AK88_GPIO_INT_CTRL0 + i * 4);

		while (enabled_irq) {
			off = __ffs(enabled_irq);
			enabled_irq &= ~(1 << off);
			if (test_bit(off, AK88_GPIO_INTP0 + i * 4) !=
			    test_bit(off, AK88_GPIO_INPUT0 + i * 4)) {
				irq = IRQ_GPIO_0 + i * 32 + off;
				desc_handle_irq(irq, irq_desc + irq);
				/* printk("irq: grp %d, %d\n", i, off); */
			}
		}
	}
#else
#error "Unsupported CPU. please check."
#endif

	/* printk("Leaving %s\n", __FUNCTION__); */
}

static void ak880x_l2mem_irq_mask(unsigned int irq)
{
	unsigned long regval;

	regval = __raw_readl(AK88_L2MEM_IRQ_ENABLE);
	regval &= ~(1 << (irq - IRQ_L2_FRAC_DMA));
	__raw_writel(regval, AK88_L2MEM_IRQ_ENABLE);
}

static void ak880x_l2mem_irq_unmask(unsigned int irq)
{
	unsigned long regval;

	regval = __raw_readl(AK88_L2MEM_IRQ_ENABLE);
	regval |= (1 << (irq - IRQ_L2_FRAC_DMA));
	__raw_writel(regval, AK88_L2MEM_IRQ_ENABLE);
}

static struct irq_chip ak880x_l2mem_irq_chip = {
	.name = "l2mem_irq",
	.mask_ack = ak880x_l2mem_irq_mask,
	.mask = ak880x_l2mem_irq_mask,
	.unmask = ak880x_l2mem_irq_unmask,
};

static void ak880x_l2mem_irqhandler(unsigned int irq, struct irq_desc *desc)
{
	unsigned long enabled_irq;
	unsigned int off;

	/* printk("Enterring %s: irq = %d\n", __FUNCTION__, irq); */

	enabled_irq = __raw_readl(AK88_L2MEM_IRQ_ENABLE) & 0x7FFFF;

	while (enabled_irq) {
		off = __ffs(enabled_irq);
		enabled_irq &= ~(1 << off);
		irq = IRQ_L2_FRAC_DMA + off;
		desc_handle_irq(irq, irq_desc + irq);
	}
}

/* ak880x_init_irq
 *
 * Initialise AK7801 IRQ system
 */

void __init ak880x_init_irq(void)
{
	int i;

	/* 1st, clear all interrupts */
	__raw_readl(AK88_INT_STATUS);
	__raw_readl(AK88_SYSCTRL_INT_CTRL);

	/* 2nd, mask all interrutps */
	__raw_writel(0x0, AK88_IRQ_MASK);
	__raw_writel(0x0, AK88_FIQ_MASK);
	__raw_writel(0x0, AK88_SYSCTRL_INT_CTRL);

	/* mask all gpio interrupts */
	__raw_writel(0x0, AK88_GPIO_INT_CTRL0);
	__raw_writel(0x0, AK88_GPIO_INT_CTRL1);
	__raw_writel(0x0, AK88_GPIO_INT_CTRL2);
	__raw_writel(0x0, AK88_GPIO_INT_CTRL3);

	/* mask all l2 interrupts */
	__raw_writel(0x0, AK88_L2MEM_IRQ_ENABLE);

	for (i = 0; i <= IRQ_SYSCTRL; i++) {
		set_irq_chip(i, &ak880x_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(IRQ_SYSCTRL, ak880x_sysctrl_handler);

	for (i = IRQ_TOUCHPANEL; i <= IRQ_GPIO; i++) {
		set_irq_chip(i, &ak880x_sysctrl_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	set_irq_chained_handler(IRQ_GPIO, ak880x_gpio_irqhandler);

	for (i = IRQ_GPIO_0; i <= IRQ_DGPIO_14; i++) {
		set_irq_chip(i, &ak880x_gpioirq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
#if 0
	set_irq_chained_handler(IRQ_L2MEM, ak880x_l2mem_irqhandler);

	for (i = IRQ_L2_FRAC_DMA; i <= IRQ_L2_CRC_VLD; i++) {
		set_irq_chip(i, &ak880x_l2mem_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
#endif
}
