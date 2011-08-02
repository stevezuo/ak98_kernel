/*
 * linux/arch/arm/mach-ak98/include/l2cache.h
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_ARCH_L2CACHE_H
#define __ASM_ARCH_L2CACHE_H

/*
 * AK98xx L2Cache ON/OFF Flag
 * AK98_L2CACHE_ENABLE=1        : L2 Cache ON
 * Undefine AK98_L2CACHE_ENABLE : L2 Cache OFF
 */
#define AK98_L2CACHE_ENABLE	1

#include <mach/ak880x_addr.h>
#include <asm/io.h>

#define AK98_VA_L2CACH_CFG	(AK98_VA_L2CACH + 0x0)

void ak98_l2cache_init(void);
void ak98_l2cache_clean_finish(void);
void ak98_l2cache_invalidate(void);

#endif	/* __ASM_ARCH_L2CACHE_H */
