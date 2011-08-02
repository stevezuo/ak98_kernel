/*
   linux/arch/arm/mach-ak7801/clock.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/ak880x_addr.h>
#include <mach/clock.h>

/* clock information */

static LIST_HEAD(clocks);

DEFINE_MUTEX(clocks_mutex);

/* enable and disable calls for use with the clk struct */

static int clk_null_enable(struct clk *clk, int enable)
{
	return 0;
}

/* Clock API calls */

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p;
	struct clk *clk = ERR_PTR(-ENOENT);
	int idno;

	if (dev == NULL || dev->bus != &platform_bus_type)
		idno = -1;
	else
		idno = to_platform_device(dev)->id;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(p, &clocks, list) {
		if (p->id == idno &&
		    strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}

	/* check for the case where a device was supplied, but the
	 * clock that was being searched for is not device specific */

	if (IS_ERR(clk)) {
		list_for_each_entry(p, &clocks, list) {
			if (p->id == -1 && strcmp(id, p->name) == 0 &&
			    try_module_get(p->owner)) {
				clk = p;
				break;
			}
		}
	}

	mutex_unlock(&clocks_mutex);
	return clk;
}

void clk_put(struct clk *clk)
{
	module_put(clk->owner);
}

int clk_enable(struct clk *clk)
{

	if (IS_ERR(clk) || clk == NULL)
		return -EINVAL;

 	clk_enable(clk->parent);

	mutex_lock(&clocks_mutex);
	if ((clk->usage++) == 0)
		(clk->enable) (clk, 1);
 
	mutex_unlock(&clocks_mutex);

	return 0;
}

void clk_disable(struct clk *clk)
{
	if (IS_ERR(clk) || clk == NULL)
		return;

	mutex_lock(&clocks_mutex);

	if ((--clk->usage) == 0)
		(clk->enable) (clk, 0);

	mutex_unlock(&clocks_mutex);
	clk_disable(clk->parent);
}

unsigned long clk_get_rate(struct clk *clk)
{
	if (IS_ERR(clk))
		return 0;

	if (clk->rate != 0)
		return clk->rate;

	if (clk->get_rate != NULL)
		return (clk->get_rate) (clk);

	if (clk->parent != NULL)
		return clk_get_rate(clk->parent);

	return clk->rate;
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;

	if (IS_ERR(clk))
		return -EINVAL;

	/* We do not default just do a clk->rate = rate as
	 * the clock may have been made this way by choice.
	 */

	WARN_ON(clk->set_rate == NULL);

	if (clk->set_rate == NULL)
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	ret = (clk->set_rate) (clk, rate);
	mutex_unlock(&clocks_mutex);

	return ret;
}

struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = 0;

	if (IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);

	if (clk->set_parent)
		ret = (clk->set_parent) (clk, parent);

	mutex_unlock(&clocks_mutex);

	return ret;
}

EXPORT_SYMBOL(clk_get);
EXPORT_SYMBOL(clk_put);
EXPORT_SYMBOL(clk_enable);
EXPORT_SYMBOL(clk_disable);
EXPORT_SYMBOL(clk_get_rate);
EXPORT_SYMBOL(clk_set_rate);
EXPORT_SYMBOL(clk_get_parent);
EXPORT_SYMBOL(clk_set_parent);

/* base clocks */

int clk_default_setrate(struct clk *clk, unsigned long rate)
{
	clk->rate = rate;
	return 0;
}

struct clk clk_xtal_12M = {
	.name = "xtal_12M",
	.id = -1,
	.usage = 0,
	.rate = 12 * 1000 * 1000,
	.parent = NULL,
};

struct clk clk_xtal_27M = {
	.name = "xtal_27M",
	.id = -1,
	.usage = 0,
	.rate = 27 * 1000 * 1000,
	.parent = NULL,
	//.enable               = xtal_27M_enable,
};

struct clk clk_xtal_32K = {
	.name = "xtal_32K",
	.id = -1,
	.usage = 0,
	.rate = 32768,
	.parent = NULL,
};

struct clk clk_pll1 = {
	.name = "pll1",
	.id = -1,
	.usage = 0,
	.parent = NULL,
	//.set_rate     = clk_pll1_setrate,
	//.get_rate     = clk_pll1_getrate,
};

struct clk clk_pll2 = {
	.name = "pll2",
	.id = -1,
	.usage = 0,
	//.set_rate     = clk_pll2_setrate,
	//.get_rate     = clk_pll2_getrate,
};

struct clk clk_asic = {
	.name = "asic_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll1,
};

struct clk clk_cpu = {
	.name = "cpu_clk",
	.id = -1,
	.usage = 0,
	//.set_rate     = clk_cpu_setrate,
	//.get_rate     = clk_cpu_getrate,
};

struct clk ap_clk = {
	.name = "ap_clk",
	.id = -1,
	.usage = 0,
	.parent = NULL,
};

struct clk adc1_clk = {
	.name = "adc1_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll1,
};

struct clk adc2_clk = {
	.name = "adc2_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_xtal_12M,
};

struct clk dac_clk = {
	.name = "dac_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll1,
};


/* cis clock */
struct clk camif_clk = {
	.name = "camif_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll2,
	//.set_rate     = clk_camif_setrate,
	//.get_rate     = clk_camif_getrate,
};

/* spi clock */
static int spi_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON &= ~(1UL << 2);
	else
		rCLK_CON |= (1UL << 2);

	return 0;
}

struct clk spi_clk = {
	.name = "spi_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = spi_clk_enable,
};

/* MMC/SD clock */
#define AK88_MMC_CLK_CTRL     (AK88_VA_MMC + 0x04)
static int mci_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON &= ~((1UL << 2) | (1UL << 18));
	/*
	 * else
	 *         rCLK_CON |= (1<<2); [> shared with SPI1/SPI2/UART2 <] 
	 */
	return 0;
}

#if 0
static int mci_set_rate(struct clk *c, unsigned long rate)
{
	int divider, regval;
	int src_rate = clk_get_rate(c->parent) * 1000 * 1000;

	if (rate == 0)
		return -EINVAL;

	regval = __raw_readl(AK88_MMC_CLK_CTRL) & ~0xffff;
	divider = (src_rate / rate - 2) / 2 & 0xff;
	regval |= divider << 8 | divider;
	__raw_writel(regval, AK88_MMC_CLK_CTRL);

	return 0;
}
#endif

static unsigned long mci_get_rate(struct clk *c)
{
#if 0
	int divider, regval;
	int src_rate = clk_get_rate(c->parent) * 1000 * 1000;

	regval = __raw_readl(AK88_MMC_CLK_CTRL) & 0xffff;
	divider = (regval & 0xff) + ((regval >> 8) & 0xff) + 2;

	return src_rate / divider;
#else
	/* return asic clock */
	return clk_asic.rate;
#endif
}

struct clk mci_clk = {
	.name = "mci_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = mci_clk_enable,
#if 0
	.set_rate = mci_set_rate,
#endif
	.get_rate = mci_get_rate,
};

/* udc */
static int udc_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON &= ~(1UL << 5);
	else
		rCLK_CON |= (1UL << 5);
	return 0;
}

struct clk udc_clk = {
	.name = "udc_clk",
	.id = -1,
	.usage = 0,
 	.parent = &clk_pll1,
	.enable = udc_clk_enable,
};

/* uart4 clk*/
static int uart4_clk_enable(struct clk *clk, int enable)
{
	if (enable){
		rCLK_CON &= ~(1UL << 8);
 	}
	else
		rCLK_CON |= (1UL << 8);

	return 0;
}

struct clk uart4_clk = {
	.name = "uart4_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll1,
	.enable = uart4_clk_enable,
};

/* lcd clk */
static int lcd_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON &= ~(1UL << 3);
	else
		rCLK_CON |= (1UL << 3);
	return 0;
}

struct clk lcd_clk = {
	.name = "lcd_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = lcd_clk_enable,
	//.set_rate     = pclk_setrate,
	//.get_rate     = pclk_getrate,
};

/* initialise the clock system */

int ak880x_register_clock(struct clk *clk)
{
	clk->owner = THIS_MODULE;

	if (clk->enable == NULL)
		clk->enable = clk_null_enable;

	/* add to the list of available clocks */
	
	mutex_lock(&clocks_mutex);
	list_add(&clk->list, &clocks);
	mutex_unlock(&clocks_mutex);
	//printk("ak880x_register_clock(),clk->name=%s\n",clk->name);

	return 0;
}

int ak880x_register_clocks(struct clk **clks, int nr_clks)
{
	int fails = 0;

	for (; nr_clks > 0; nr_clks--, clks++) {
		if (ak880x_register_clock(*clks) < 0)
			fails++;
	}

	return fails;
}

/* initalise all the clocks */

/*
 *               4  x  M
 *  PLL1_CLK =  -----------, M = (62 + m), N = (1 + n)
 *                  N
 *
 *               PLL1_CLK
 *  ASIC_CLK =  -----------
 *               ASIC_DIV
 *
 *
 *  CPU_CLK  =  PLL1_CLK or ASIC_CLK
 *
 */
static int __init ak880x_init_clocks(void)
{
	union ak880x_clk_div1_reg clk_div1_reg;

	printk(KERN_INFO "ANYKA AK88 Clocks, (c) 2010 ANYKA \n");

	clk_div1_reg.regval = __raw_readl(AK88_VA_SYSCTRL + 0x04);

	/* work out what clocks we've got */
	/* printk("regval = 0x%lx\n", clk_div1_reg.regval); */

#if	defined(CONFIG_BOARD_AK8801EPC)

	#if 0
	if (clk_div1_reg.clk_div.m > 0x20)
		clk_pll1.rate = 4 * 62 / (clk_div1_reg.clk_div.n + 1);
	else
		clk_pll1.rate =
		    4 * (clk_div1_reg.clk_div.m +
			 62) / (clk_div1_reg.clk_div.n + 1);
	#endif

	clk_pll1.rate =
	    4 * (clk_div1_reg.clk_div.m + 45) / (clk_div1_reg.clk_div.n + 1);
	clk_asic.rate =
	    clk_pll1.rate >> (clk_div1_reg.clk_div.asic_clk ? clk_div1_reg.
			      clk_div.asic_clk : 1);
 	
	clk_cpu.rate =
	    clk_div1_reg.clk_div.cpu_clk ? clk_pll1.rate : clk_asic.rate;

	printk("AK88: PLL1 %ld MHz, ASIC %ld MHz, CPU Core %ld MHz\n",
	       clk_pll1.rate, clk_asic.rate, clk_cpu.rate);

	clk_pll1.rate =clk_pll1.rate*1000*1000;
	clk_asic.rate =clk_asic.rate*1000*1000;
	clk_cpu.rate =clk_cpu.rate*1000*1000;

#elif	defined(CONFIG_BOARD_AK8802EBOOK)

	clk_pll1.rate =
	    4 * (clk_div1_reg.clk_div.m + 45) / (clk_div1_reg.clk_div.n + 1);
	clk_asic.rate =
	    clk_pll1.rate >> (clk_div1_reg.clk_div.asic_clk ? clk_div1_reg.
			      clk_div.asic_clk : 1);
	clk_cpu.rate =
	    clk_div1_reg.clk_div.cpu_clk ? clk_pll1.rate : clk_asic.rate;

	printk("AK8802: PLL1 %ld MHz, ASIC %ld MHz, CPU Core %ld MHz\n",
	       clk_pll1.rate, clk_asic.rate, clk_cpu.rate);

	clk_pll1.rate =clk_pll1.rate*1000*1000;
	clk_asic.rate =clk_asic.rate*1000*1000;
	clk_cpu.rate =clk_cpu.rate*1000*1000;

#else
#error "Only support AK780X & AK88."
#endif

	/* register our clocks */

	if (ak880x_register_clock(&clk_xtal_12M) < 0)
		printk(KERN_ERR "failed to register master xtal\n");

	if (ak880x_register_clock(&clk_pll1) < 0)
		printk(KERN_ERR "failed to register master xtal\n");

	if (ak880x_register_clock(&clk_asic) < 0)
		printk(KERN_ERR "failed to register asic clk\n");

	if (ak880x_register_clock(&clk_cpu) < 0)
		printk(KERN_ERR "failed to register cpu clk\n");

	if (ak880x_register_clock(&spi_clk) < 0)
		printk(KERN_ERR "failed to register spi clk\n");

	if (ak880x_register_clock(&mci_clk) < 0)
		printk(KERN_ERR "failed to register mci clk\n");

	if (ak880x_register_clock(&udc_clk) < 0)
		printk(KERN_ERR "failed to register udc clk\n");

	if (ak880x_register_clock(&uart4_clk) < 0)
		printk(KERN_ERR "failed to register uart4 clk\n");

	if (ak880x_register_clock(&lcd_clk) < 0)
		printk(KERN_ERR "failed to register lcd clk\n");

	/* FIXME:
	 * here we disable unused clock, only leave SDRAM/DDR clock enabled.
	 * please manual enable clock as need.
	 */
	__raw_writel(0x7BF7, AK88_VA_SYSCTRL + 0x0C);

	//uart4_clk_enable(&uart4_clk,1);
	clk_enable(&uart4_clk);

	printk("clk control: 0x%08X\n", __raw_readl(AK88_VA_SYSCTRL + 0x0C));

	return 0;
}

arch_initcall(ak880x_init_clocks);
