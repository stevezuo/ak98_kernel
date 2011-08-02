/* linux/include/asm-arm/arch-ak880x/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#include <mach/hardware.h>
//#include <mach/io.h>
//#include <linux/err.h>

#include <asm/io.h>

void (*ak880x_shutdown_machine) (void);

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static void arch_reset(char mode, const char *cmd)
{
	/* TODO:
	 * 1, check power key.
	 * 2. check charger
	 * --------------------
	 *    set mode to 's', and reboot system.
	 *
	 * 3. check rtc wakeup
	 * --------------------
	 *   clear wakeup status and shutdown system.
	 */

	if (mode == 'h') {	/* Power off system */
		printk("%s: poweroff system\n", __FUNCTION__);

		if (ak880x_shutdown_machine)
			ak880x_shutdown_machine();

		mdelay(1000);

	} else {		/* Reset system use watchdog timer */

		/* TODO: Add watchdog timer setup to reboot */

		/* wait for reset to assert... */
		mdelay(5000);
		printk(KERN_ERR "Watchdog reset failed to assert reset\n");

		/* we'll take a jump through zero as a poor second */
		cpu_reset(0);
	}
}

//#include <mach/system-reset.h>
