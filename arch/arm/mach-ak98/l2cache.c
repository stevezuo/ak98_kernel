/*
 * linux/arch/arm/mach-ak98/l2cache.c
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/stddef.h>
#include <linux/irq.h>
 
#include <asm/dma.h>
#include <asm/sizes.h>
#include <asm/setup.h>

#include <mach/l2cache.h>


#ifdef AK98_L2CACHE_ENABLE

static DEFINE_SPINLOCK(l2cache_lock);


#define L2_CACHE_CFG_REG	REG_VA_VAL(AK98_VA_L2CACH, 0x00)
#define L2_CACHE_SECTION0_START	REG_VA_VAL(AK98_VA_L2CACH, 0x10)
#define L2_CACHE_SECTION0_END	REG_VA_VAL(AK98_VA_L2CACH, 0x14)
#define L2_CACHE_SECTION7_END	REG_VA_VAL(AK98_VA_L2CACH, 0x84)

void ak98_l2cache_init(void)
{
	volatile int i;
	unsigned long start_addr;
	unsigned long end_addr;

	if (meminfo.nr_banks == 0) {
		printk("Fatal error: NO memory banks defined!\n");
		BUG();
	}

	if (meminfo.nr_banks > 1)
		printk("Warning: memory banks != 1, mapping only first bank for L2 cache\n");

	start_addr = meminfo.bank[0].start;
	end_addr = start_addr + meminfo.bank[0].size;
	
	/* Enable L2 Cache Invalidation */
	L2_CACHE_CFG_REG |= (1 << 26) | (1 << 27);

	/* Wait for L2 Cache for initialization */
	while (L2_CACHE_CFG_REG & (1 << 25))
		;

	/* Invalidation finished */
	L2_CACHE_CFG_REG &= ~((1 << 26) | (1 << 27));

	/* Wait for enough clock cycles */
	for (i = 0; i < 3000; i++)
		;

	/* Disable all 8 L2 cache sections */
	L2_CACHE_CFG_REG &= ~(0xFF);

	/*
	 * Configure the first section to map whole RAM
	 * Start address: Low 14bits must be 0
	 * End address: Low 14bits must be 1
	 */
	printk("Mapping RAM region 0x%lx - 0x%lx to L2 cache section 0\n",
		start_addr, end_addr);
	L2_CACHE_SECTION0_START = start_addr & 0xFFFFC000;
	L2_CACHE_SECTION0_END = (end_addr - 1) | 0x3FFF;

	/* Enable L2 cache section 0 */
	L2_CACHE_CFG_REG |= (1 << 0);

	/*
	 * AK9805 V3 & up:
	 *   Force L2 Cache rready3 == 0 to resolve data abort error
	 * NOTE: Do *NOT* affect AK9801 & other version CPU
	 */
	L2_CACHE_SECTION7_END |= 1;
}
EXPORT_SYMBOL(ak98_l2cache_init);

void ak98_l2cache_clean_finish(void)
{
	int count = 0;
	unsigned long regval;
	volatile int i;
	unsigned long flags;
	
	spin_lock_irqsave(&l2cache_lock, flags);

	/*
	 * Wait for AHB1-4 channel to be free
	 */
	do {
		regval = L2_CACHE_CFG_REG;
	} while ((((regval >> 16) & 0xF) || ((regval >> 24) & 0x1)) & (count++ < 3000));
	
	if (count >= 3000) {
		printk("Failed to clean and finish L2 cache!!\n");
		BUG();
	}

	for (i = 0; i < 100; i++)
		;
	
	spin_unlock_irqrestore(&l2cache_lock, flags);

}
EXPORT_SYMBOL(ak98_l2cache_clean_finish);

void ak98_l2cache_invalidate(void)
{
	/*
	 * Assume that we only use Section 0 of L2 Cache to map whole RAM, modify the following code
	 * when the assumption is broken.
	 */

	volatile int i;
	unsigned long flags;

	spin_lock_irqsave(&l2cache_lock, flags);

	/*
	 * Disable L2 Cache Section 0
	 */
	L2_CACHE_CFG_REG &= ~(1 << 0);

	/*
	 * Clear all L2 entries & the read line buffer between CPU and external RAM for non-L2 section.
	 * That is, invalidate L2 Cache.
	 */
	L2_CACHE_CFG_REG |= (1 << 26) | (1 << 27);

	/*
	 * Wait until L2 Cache initialization finished.
	 */
	while (L2_CACHE_CFG_REG & (1 << 25))
		;
	/*
	 * L2 invalidate finished.
	 */
	L2_CACHE_CFG_REG &= ~((1 << 26) | (1 << 27));

	/*
	 * Delay for a relative long time (> 128 clocks) for all AHB Channel of RAM Controller
	 * free and +128 clocks
	 */
	for (i = 0; i < 30; i++)
		;

	/*
	 * Enable L2 Cache Section 0
	 */
	L2_CACHE_CFG_REG |= (1 << 0);

	spin_unlock_irqrestore(&l2cache_lock, flags);
	
	return ;
}
EXPORT_SYMBOL(ak98_l2cache_invalidate);

#else	/*  !AK98_L2CACHE_ENABLE */

void ak98_l2cache_init(void)
{
}
EXPORT_SYMBOL(ak98_l2cache_init);

void ak98_l2cache_clean_finish(void)
{
}
EXPORT_SYMBOL(ak98_l2cache_clean_finish);

void ak98_l2cache_invalidate(void)
{
}
EXPORT_SYMBOL(ak98_l2cache_invalidate);

#endif	/*  AK98_L2CACHE_ENABLE */
