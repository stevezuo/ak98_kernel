/* arch/arm/mach-ak880x/include/mach/system-reset.h
 * from arch/arm/mach-s3c2410/include/mach/system-reset.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * AK98 - System define for arch_reset() function
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

//#include <mach/hardware.h>
//#include <mach/watchdog-reset.h>

extern void (*ak880x_reset_hook) (void);

static inline void arch_wdt_reset(void)
{
	//struct clk *wdtclk;

	printk("arch_reset: attempting watchdog reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);
}

static void arch_reset(char mode, const char *cmd)
{
	if (mode == 's') {
		cpu_reset(0);
	}

	if (ak880x_reset_hook)
		ak880x_reset_hook();

	arch_wdt_reset();

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}
