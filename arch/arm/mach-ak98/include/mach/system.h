/* linux/include/asm-arm/arch-ak98/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/proc-fns.h>
#include <mach/reset.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static void arch_reset(char mode, const char *cmd)
{
	switch (mode) {
	case 's':	/*  Software reset, using default */
		cpu_reset(0);
		break;
	case 'h':	/*  Hardware reset, platform specific */
		if (ak98_arch_reset)
			ak98_arch_reset();
		break;
	default:
		/*  Not Implemented Yet - Not needed */
		break;
	}
}

#endif	/*  __ASM_ARCH_SYSTEM_H */
