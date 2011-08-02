/*
 * linux/include/asm-arm/arch-s3c2410/io.h
 *  from linux/include/asm-arm/arch-rpc/io.h
 *
 * Copyright (C) 1997 Russell King
 *	     (C) 2003 Simtec Electronics
*/

//#ifndef __ASM_ARM_ARCH_IO_H
//#define __ASM_ARM_ARCH_IO_H
#ifndef __ARCH_ARM_MACH_AK98_IO_H
#define __ARCH_ARM_MACH_AK98_IO_H

//#include <mach/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

/*
 * We use two different types of addressing - PC style addresses, and ARM
 * addresses.  PC style accesses the PC hardware with the normal PC IO
 * addresses, eg 0x3f8 for serial#1.  ARM addresses are above A28
 * and are translated to the start of IO.  Note that all addresses are
 * not shifted left!
 */

/*
 * 1:1 mapping for ioremapped regions.
 */
#define __mem_pci(x)	(x)

#define __io(a)		((void __iomem *)(a))

#define AK98_DUMP_REG(x) { printk("reg 0x%x=0x%x\n",(u32)AK98_##x,(u32)__raw_readl(AK98_##x)); }

static inline unsigned long AKSET_BITS(unsigned long bits_result, void *reg)
{
	volatile unsigned long val;
	val = __raw_readl(reg);

	val |= bits_result;

	__raw_writel(val, reg);

	return 0;
}

static inline unsigned long AKCLR_BITS(unsigned long bits_result, void *reg)
{
	volatile unsigned long val;
	val = __raw_readl(reg);
	val &= ~bits_result;

	__raw_writel(val, reg);

	return 0;
}

static inline bool AKGET_BIT(unsigned long bit_result, void *reg)
{
	unsigned long val;

	val = __raw_readl(reg) & bit_result;

	if (val == bit_result)
		return true;
	else
		return false;

}

#endif
