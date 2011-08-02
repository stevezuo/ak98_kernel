/*
 * linux/arch/arm/mach-ak98/clock.c
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <mach/regs-comm.h>


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
	.rate = 12 * MHz,
	.parent = NULL,
};

struct clk clk_xtal_25M = {
	.name = "xtal_25M",
	.id = -1,
	.usage = 0,
	.rate = 25 * MHz,
	.parent = NULL,
};

struct clk clk_xtal_27M = {
	.name = "xtal_27M",
	.id = -1,
	.usage = 0,
	.rate = 27 * MHz,
	.parent = NULL,
};

struct clk clk_xtal_32K = {
	.name = "xtal_32K",
	.id = -1,
	.usage = 0,
	.rate = 32768,
	.parent = NULL,
};

struct clk clk_pll = {
	.name = "pll",
	.id = -1,
	.usage = 0,
	.parent = NULL,
	//.set_rate     = clk_pll_setrate,
	//.get_rate     = clk_pll_getrate,
};

static int spll_clk_enable(struct clk *clk, int enable)
{
	if (enable) {
		rMULFUN_CON1 &= ~(1UL << 30);
		rMULFUN_CON1 &= ~(1UL << 31);
		printk(KERN_DEBUG "CAMIF: pll2_clk_enable!\n");
		
	} else {
		rMULFUN_CON1 |= (1UL << 30);
		rMULFUN_CON1 |= (1UL << 31);
		printk(KERN_DEBUG "CAMIF: pll2_clk_disable!\n");
	}
	
	return 0;
}

struct clk clk_spll = {
	.name = "spll",
	.id = -1,
	.usage = 0,
	.parent = NULL,
	.enable = spll_clk_enable,
	//.set_rate     = clk_pll_setrate,
	//.get_rate     = clk_pll_getrate,
};


struct clk clk_asic = {
	.name = "asic_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll,
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
	.parent = &clk_pll,
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
	.parent = &clk_pll,
};

/* camif clock */
static int camif_clk_enable(struct clk *clk, int enable)
{
	if (enable) {
		rCLK_CON1 &= ~(1UL << 8);
		printk(KERN_DEBUG "CAMIF: camif_clk_enable!\n");
	} else {
		rCLK_CON1 |= (1UL << 8);
		printk(KERN_DEBUG "CAMIF: camif_clk_disable!\n");
	}	
	
	return 0;
}

/* cis clock */
struct clk camif_clk = {
	.name = "camif_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_spll,
	.enable = camif_clk_enable,
	//.set_rate     = clk_camif_setrate,
	//.get_rate     = clk_camif_getrate,
};


/* i2c clock */
static int i2c_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON2 &= ~(1UL << 5);
	else
		rCLK_CON2 |= (1UL << 5);
	return 0;
}

struct clk i2c_clk = {
	.name	= "i2c_clk",
	.id		= -1,
	.usage	= 0,
	.parent = &clk_asic,
	.enable = i2c_clk_enable,
};

/* mac clock */
static int mac_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON2 &= ~(1UL << 14);
	else
		rCLK_CON2 |= (1UL << 14);

	return 0;
}

struct clk mac_clk = {
	.name	= "mac_clk",
	.id		= -1,
	.usage	= 0,
	.parent = &clk_asic,
	.enable = mac_clk_enable,
};

/* spi1 clock */
static int spi1_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON1 &= ~(1 << 13);
	else
		rCLK_CON1 |= (1 << 13);

	return 0;
}

struct clk spi1_clk = {
	.name = "spi1_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = spi1_clk_enable,
};

/* spi2 clock */
static int spi2_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON2 &= ~(1 << 7);
	else
		rCLK_CON2 |= (1 << 7);

	return 0;
}

struct clk spi2_clk = {
	.name = "spi2_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = spi2_clk_enable,
};


/* MMC/SD clock */
static int mci_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON2 &= ~(1UL << 2);
	else 
		rCLK_CON2 |= (1UL << 2);
	
	return 0;
}

static unsigned long mci_get_rate(struct clk *c)
{
	/* return asic clock */
	return clk_asic.rate;
}

struct clk mci_clk = {
	.name = "mci_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = mci_clk_enable,
	.get_rate = mci_get_rate,
};

/* MMC/SD clock */
static int sdio_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON2 &= ~(1UL << 3);
	else 
		rCLK_CON2 |= (1UL << 3);
	
	return 0;
}

static unsigned long sdio_get_rate(struct clk *c)
{
	/* return asic clock */
	return clk_asic.rate;
}

struct clk sdio_clk = {
	.name = "sdio_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_asic,
	.enable = sdio_clk_enable,
	.get_rate = sdio_get_rate,
};


/* udc */
static int udc_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON1 &= ~(1UL << 15);
	else
		rCLK_CON1 |= (1UL << 15);
	return 0;
}

struct clk udc_clk = {
	.name = "udc_clk",
	.id = -1,
	.usage = 0,
 	.parent = &clk_pll,
	.enable = udc_clk_enable,
};

/* uart0 clk*/
static int uart0_clk_enable(struct clk *clk, int enable)
{
	if (enable){
		rCLK_CON1 &= ~(1UL << 12);
 	}
	else
		rCLK_CON1 |= (1UL << 12);

	return 0;
}

struct clk uart0_clk = {
	.name = "uart0_clk",
	.id = -1,
	.usage = 0,
	.parent = &clk_pll,
	.enable = uart0_clk_enable,
};

/* lcd clk */
static int lcd_clk_enable(struct clk *clk, int enable)
{
	if (enable)
		rCLK_CON1 &= ~(1UL << 11);
	else
		rCLK_CON1 |= (1UL << 11);
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

int ak98_register_clock(struct clk *clk)
{
	clk->owner = THIS_MODULE;

	if (clk->enable == NULL)
		clk->enable = clk_null_enable;

	/* add to the list of available clocks */
	
	mutex_lock(&clocks_mutex);
	list_add(&clk->list, &clocks);
	mutex_unlock(&clocks_mutex);
	//printk("ak98_register_clock(),clk->name=%s\n",clk->name);

	return 0;
}

int ak98_register_clocks(struct clk **clks, int nr_clks)
{
	int fails = 0;

	for (; nr_clks > 0; nr_clks--, clks++) {
		if (ak98_register_clock(*clks) < 0)
			fails++;
	}

	return fails;
}

bool ak98_cpu_is_3x_mode(void)
{
	return ak98_get_cpu_mode() == CPU_MODE_CPU3X;
}
EXPORT_SYMBOL(ak98_cpu_is_3x_mode);

bool ak98_cpu_is_2x_mode(void)
{
	if (ak98_cpu_is_3x_mode())
		return false;

	return __raw_readl(AK98_VA_SYSCTRL + 0x0004) & AK98_CLK_DIV_1_CPU2X;
}
EXPORT_SYMBOL(ak98_cpu_is_2x_mode);


bool ak98_cpu_is_special_mode(void)
{
	if (ak98_cpu_is_3x_mode())
		return false;

	return __raw_readl(AK98_VA_SYSCTRL + 0x0004) & AK98_CLK_DIV_1_SPECIAL;
}
EXPORT_SYMBOL(ak98_cpu_is_special_mode);

bool ak98_cpu_is_low_clock_mode(void)
{
	if (ak98_cpu_is_3x_mode())
		return false;

	return __raw_readl(AK98_VA_SYSCTRL + 0x0004) & AK98_CLK_DIV_1_LOW;
}
EXPORT_SYMBOL(ak98_cpu_is_low_clock_mode);

bool ak98_cpu_is_normal_mode(void)
{
	return ak98_get_cpu_mode() == CPU_MODE_NORMAL;
}
EXPORT_SYMBOL(ak98_cpu_is_normal_mode);

ak98_cpu_mode_t ak98_get_cpu_mode(void)
{
	unsigned long clk_div_1 = __raw_readl(AK98_VA_SYSCTRL + 0x0004);

	if (clk_div_1 & AK98_CLK_DIV_1_CPU3X)
		return CPU_MODE_CPU3X;
	else if (clk_div_1 & AK98_CLK_DIV_1_LOW)
		return CPU_MODE_LOW;
	else if (clk_div_1 & AK98_CLK_DIV_1_SPECIAL)
		return CPU_MODE_SPECIAL;

	return CPU_MODE_NORMAL;
}
EXPORT_SYMBOL(ak98_get_cpu_mode);

unsigned long ak98_get_pll_clk(void)
{
	unsigned long clk = 0UL;
	unsigned long clk_div_1 = __raw_readl(AK98_VA_SYSCTRL + 0x0004);
	unsigned long pll_sel = clk_div_1 & 0x3F;

	clk = AK98_MIN_PLL_CLK + pll_sel * 4 * MHz;
	
	return clk;
}
EXPORT_SYMBOL(ak98_get_pll_clk);


unsigned long ak98_get_clk168m_clk(void)
{
	unsigned long clk = 0UL;
	unsigned long clk_div_1 = __raw_readl(AK98_VA_SYSCTRL + 0x0004);
	unsigned long clk168_div = (clk_div_1 >> 17) & 0xF;

	clk = ak98_get_pll_clk() / (clk168_div + 1);
	
	return clk;
}
EXPORT_SYMBOL(ak98_get_clk168m_clk);


unsigned long ak98_get_asic_clk(void)
{
	unsigned long clk = 0UL;
	unsigned long clk_div_1 = __raw_readl(AK98_VA_SYSCTRL + 0x0004);
	unsigned long asic_div = (clk_div_1 >> 6) & 0x7;

	if (asic_div == 0)
		asic_div = 1;

	if (ak98_cpu_is_3x_mode())
		clk = ak98_get_clk168m_clk() / 3;
	else if (ak98_cpu_is_special_mode() && !ak98_cpu_is_low_clock_mode())
		clk = ak98_get_clk168m_clk() * 2 / 5;
	else
		clk = ak98_get_clk168m_clk() >> asic_div;
	
	return clk;
}
EXPORT_SYMBOL(ak98_get_asic_clk);

unsigned long ak98_get_mem_clk(void)
{
	unsigned long clk = 0UL;
	unsigned long clk_div_1 = __raw_readl(AK98_VA_SYSCTRL + 0x0004);
	unsigned long mem_div = (clk_div_1 >> 9) & 0x7;

	if (mem_div == 0)
		mem_div = 1;
	
	if (ak98_cpu_is_3x_mode())
		clk = ak98_get_clk168m_clk() / 3;
	else
		clk = ak98_get_clk168m_clk() >> mem_div;

	
	return clk;
}
EXPORT_SYMBOL(ak98_get_mem_clk);


unsigned long ak98_get_cpu_clk(void)
{
	if (ak98_cpu_is_3x_mode() || ak98_cpu_is_2x_mode())
		return ak98_get_clk168m_clk();
	else if (ak98_cpu_is_low_clock_mode())
		return ak98_get_asic_clk();
	else return ak98_get_mem_clk();
}
EXPORT_SYMBOL(ak98_get_cpu_clk);


/* initalise all the clocks */

static int __init ak98_init_clocks(void)
{
	printk(KERN_INFO "ANYKA AK98 Clocks, (c) 2010 ANYKA \n");

#if defined(CONFIG_AK9801_ATHENA) || defined(CONFIG_AK9805_TV908)

	/*
	* We assume that AK98xx EBOOK and AK98xx EPC board use the same clock settings.
	* May need to be changed in the future.
	*/
 	clk_default_setrate(&clk_pll, ak98_get_pll_clk());
	clk_default_setrate(&clk_asic, ak98_get_asic_clk());
	clk_default_setrate(&clk_cpu, ak98_get_cpu_clk());

	printk("AK98: PLL %ld MHz, ASIC %ld MHz, CPU Core %ld MHz,"
		"MEM %ld MHz\n", clk_pll.rate / MHz, clk_asic.rate / MHz, 
			clk_cpu.rate / MHz, ak98_get_mem_clk() / MHz);
#endif

	/* register our clocks */

	if (ak98_register_clock(&clk_xtal_12M) < 0)
		printk(KERN_ERR "failed to register 12M xtal\n");

	if (ak98_register_clock(&clk_xtal_25M) < 0)
		printk(KERN_ERR "failed to register 25M xtal\n");

	if (ak98_register_clock(&clk_xtal_27M) < 0)
		printk(KERN_ERR "failed to register 27M xtal\n");

	if (ak98_register_clock(&clk_xtal_32K) < 0)
		printk(KERN_ERR "failed to register 32K xtal\n");

	if (ak98_register_clock(&clk_pll) < 0)
		printk(KERN_ERR "failed to register pll\n");

	if (ak98_register_clock(&clk_asic) < 0)
		printk(KERN_ERR "failed to register asic clk\n");

	if (ak98_register_clock(&clk_cpu) < 0)
		printk(KERN_ERR "failed to register cpu clk\n");

	if (ak98_register_clock(&spi1_clk) < 0)
		printk(KERN_ERR "failed to register spi clk\n");

	if (ak98_register_clock(&spi2_clk) < 0)
		printk(KERN_ERR "failed to register spi clk\n");

	if (ak98_register_clock(&mci_clk) < 0)
		printk(KERN_ERR "failed to register mci clk\n");

	if (ak98_register_clock(&sdio_clk) < 0)
		printk(KERN_ERR "failed to register sdio clk\n");

	if (ak98_register_clock(&udc_clk) < 0)
		printk(KERN_ERR "failed to register udc clk\n");

	if (ak98_register_clock(&uart0_clk) < 0)
		printk(KERN_ERR "failed to register uart0 clk\n");

	if (ak98_register_clock(&lcd_clk) < 0)
		printk(KERN_ERR "failed to register lcd clk\n");

	if (ak98_register_clock(&i2c_clk) < 0)
		printk(KERN_ERR "failed to register i2c clk\n");

	if (ak98_register_clock(&mac_clk) < 0)
		printk(KERN_ERR "failed to register mac clk\n");

	if (ak98_register_clock(&camif_clk) < 0)
		printk(KERN_ERR "failed to register camif clk\n");


	/*
	 * Enable L2/RAM clock by default, Disable all other clocks.
	 */
	__raw_writel(0xFBF7, AK98_VA_SYSCTRL + 0x0C);

	clk_enable(&uart0_clk);

	printk("clk control: 0x%08X\n", __raw_readl(AK98_VA_SYSCTRL + 0x0C));

	return 0;
}

arch_initcall(ak98_init_clocks);

