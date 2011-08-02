/* linux/arch/arm/mach-ak7801/pm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>

#include <asm/cacheflush.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>
#include <asm/arch/gpio.h>
#include <asm/arch/ak7801_addr.h>
#include <asm/arch/pm.h>

const unsigned char suspend_code[] = {
	0xb4, 0x10, 0x9f, 0xe5, 0xb4, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0,
	    0xa0, 0xe1,
	0, 0, 0xa0, 0xe1, 0xa8, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0, 0xa0,
	    0xe1,
	0, 0, 0xa0, 0xe1, 0x94, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0, 0xa0,
	    0xe1,
	0, 0, 0xa0, 0xe1, 0x8c, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0, 0xa0,
	    0xe1,
	0, 0, 0xa0, 0xe1, 0, 0, 0xa0, 0xe1, 0, 0, 0xa0, 0xe1, 0x78, 0, 0x9f,
	    0xe5,
	0, 0x10, 0x90, 0xe5, 0x40, 0x1b, 0x81, 0xe3, 0x80, 0x1d, 0x81, 0xe3, 0,
	    0x10, 0x80, 0xe5,
	0, 0, 0xa0, 0xe3, 0x1, 0, 0x80, 0xe2, 0x80, 0x7, 0x50, 0xe3, 0xfc, 0xff,
	    0xff, 0x1a,
	0x44, 0x10, 0x9f, 0xe5, 0x44, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0,
	    0xa0, 0xe1,
	0, 0, 0xa0, 0xe1, 0x44, 0, 0x9f, 0xe5, 0, 0, 0x81, 0xe5, 0, 0, 0xa0,
	    0xe1,
	0, 0, 0xa0, 0xe1, 0, 0, 0xa0, 0xe1, 0, 0, 0xa0, 0xe1, 0, 0x10, 0xa0,
	    0xe3,
	0x17, 0x1f, 0x8, 0xee, 0x17, 0x1f, 0x7, 0xee, 0x48, 0x14, 0xa0, 0xe3,
	    0xf0, 0x20, 0x91, 0xe5,
	0x2, 0xf0, 0xa0, 0xe1, 0, 0, 0xa0, 0xe1, 0xfe, 0xff, 0xff, 0xea, 0,
	    0xd0, 0x2, 0x20,
	0, 0, 0x17, 0xc0, 0, 0x4, 0x12, 0xc0, 0, 0, 0x11, 0x80, 0x4, 0, 0, 0x8,
	0, 0, 0x11, 0xe0, 0xe0,
};

extern void ak7801_cpu_sleep(void);
extern unsigned long cdma_sleep;

#if 0
static ak7801_pm_debug_init(void)
{
}
#else
#define ak7801_pm_debug_init() do { } while(0)
#endif

static void turn_kpd_led(int on)
{
	ak7801_gpio_cfgpin(AK7801_DGPIO_36, AK7801_GPIO_OUT);
	ak7801_gpio_pullup(AK7801_DGPIO_36, 1);

	if (on == 1)
		ak7801_gpio_setpin(AK7801_DGPIO_36, 1);
	else
		ak7801_gpio_setpin(AK7801_DGPIO_36, 0);
}

static void ak7801_set_wgpio(unsigned int wgpio_mask)
{
	unsigned long wgpio;

	wgpio = __raw_readl(AK7801_VA_SYSCTRL + 0x40);
	__raw_writel(wgpio | wgpio_mask, AK7801_VA_SYSCTRL + 0x40);

	wgpio = __raw_readl(AK7801_VA_SYSCTRL + 0x40);
	__raw_writel(wgpio & ~wgpio_mask, AK7801_VA_SYSCTRL + 0x40);

	wgpio = __raw_readl(AK7801_VA_SYSCTRL + 0x44);
	__raw_writel(wgpio | wgpio_mask, AK7801_VA_SYSCTRL + 0x44);
}

static int ak7801_pm_enter(suspend_state_t state)
{
	unsigned long regs_save[16];
	unsigned long wgpio_status;
	unsigned long sharepin_cfg1;
	unsigned long tmp;

	int offset = 0;
	int i = 0;

	/* turn off backlight */
	turn_kpd_led(0);

	/* store the physical address of the register recovery block */
	ak7801_sleep_save_phys = virt_to_phys(regs_save);

	/* ensure the debug is initialised (if enabled) */
	ak7801_pm_debug_init();

	flush_cache_all();

	AK7801_GPIO_UART1_FLOW(0);
	ak7801_gpio_pullup(AK7801_GPIO_18, 1);
	ak7801_gpio_setpin(AK7801_GPIO_18, 1);
	ak7801_gpio_cfgpin(AK7801_GPIO_18, AK7801_GPIO_OUT);
	rIO_CON1 &= ~(1 << 17);

	AK7801_GPIO_UART3_FLOW(0);
	ak7801_gpio_pullup(AK7801_GPIO_26, 1);
	ak7801_gpio_setpin(AK7801_GPIO_26, 1);
	ak7801_gpio_cfgpin(AK7801_GPIO_26, AK7801_GPIO_OUT);

	if (cdma_sleep == 0) {
		ak7801_gpio_cfgpin(AK7801_GPIO_28, AK7801_GPIO_OUT);
		ak7801_gpio_setpin(AK7801_GPIO_28, 0);
		ak7801_gpio_pullup(AK7801_GPIO_28, 0);
		ak7801_gpio_cfgpin(AK7801_GPIO_29, AK7801_GPIO_OUT);
		ak7801_gpio_setpin(AK7801_GPIO_29, 0);
		ak7801_gpio_pullup(AK7801_GPIO_29, 0);
		ak7801_gpio_cfgpin(AK7801_GPIO_11, AK7801_GPIO_IN);
		ak7801_gpio_pullup(AK7801_GPIO_11, 0);
		ak7801_gpio_cfgpin(AK7801_GPIO_27, AK7801_GPIO_IN);
		ak7801_gpio_pullup(AK7801_GPIO_27, 0);
		ak7801_gpio_cfgpin(AK7801_GPIO_26, AK7801_GPIO_IN);
		ak7801_gpio_pullup(AK7801_GPIO_26, 0);
		rIO_CON1 |= (0x1 << 25 | 0x1 << 24);
	} else
		rIO_CON1 &= ~(1 << 25 | 1 << 24);

	ak7801_set_wgpio(1 << 1 | 1 << 5 | 1 << 4 | 1 << 21);

	tmp = __raw_readl(AK7801_VA_SYSCTRL + 0x3C);
	tmp |= 1 << 1;
	if (ak7801_gpio_getpin(AK7801_GPIO_11))
		tmp |= 1 << 4;
	else
		tmp &= ~(1 << 4);
	if (ak7801_gpio_getpin(AK7801_GPIO_12))
		tmp |= 1 << 5;
	else
		tmp &= ~(1 << 5);
	__raw_writel(tmp, AK7801_VA_SYSCTRL + 0x3C);

	tmp = __raw_readl(AK7801_VA_SYSCTRL + 0xF0);
	__raw_writel(tmp | 1 << 6, AK7801_VA_SYSCTRL + 0xF0);
	tmp = __raw_readl(AK7801_VA_SYSCTRL + 0xE0);
	__raw_writel(tmp | 1 << 6, AK7801_VA_SYSCTRL + 0xE0);

	/* configure gpio for power save */
	sharepin_cfg1 = __raw_readl(AK7801_SHAREPIN_CON1);
	__raw_writel(1 << 30, AK7801_SHAREPIN_CON1);

	rL2_FRACDMAADDR &= ~(1 << 29);

	for (i = 0; i < sizeof(suspend_code) / sizeof(char); i += 4) {
		__raw_writel(suspend_code[i] | (suspend_code[i + 1] << 8) |
			     (suspend_code[i + 2] << 16) | (suspend_code[i + 3]
							    << 24),
			     AK7801_VA_L2MEM + i);
	}

	__raw_writel(virt_to_phys(ak7801_cpu_resume), AK7801_VA_L2MEM + 0xF0);

	if (ak7801_cpu_save(regs_save) == 0) {
		flush_cache_all();
		ak7801_cpu_sleep();
	}

	cpu_init();

	rL2_FRACDMAADDR |= (1 << 29);

	ak7801_pm_debug_init();

	__raw_writel(sharepin_cfg1, AK7801_SHAREPIN_CON1);

	/* check which irq wakeup system */
	/* 1. read wakeup GPIO status register, and go to wakeup sub-program */
	wgpio_status = __raw_readl(AK7801_VA_SYSCTRL + 0x48);

	while (wgpio_status) {
		offset = __ffs(wgpio_status);
		wgpio_status &= ~(1 << offset);
	}

	return 0;
}

static struct platform_suspend_ops ak7801_pm_ops = {
	.valid = suspend_valid_only_mem,
	.begin = NULL,
	.prepare = NULL,
	.enter = ak7801_pm_enter,
	.finish = NULL,
	.end = NULL,
};

/* ak7801_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init ak7801_pm_init(void)
{
	printk("AK88 Power Management, (c) 2010 ANYKA\n");
	suspend_set_ops(&ak7801_pm_ops);


	return 0;
}
