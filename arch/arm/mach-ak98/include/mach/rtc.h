/*
 * linux/arch/arm/mach-ak98/include/mach/rtc.h
 *
 * AK98 RTC related routines
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_ARCH_RTC_H
#define __ASM_ARCH_RTC_H

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/hardware.h>

#define EPOCH_START_YEAR	(1900)
#define RTC_START_YEAR		(1970)
#define RTC_YEAR_COUNT		(127)

#define AK98_RTC_CONF		(AK98_VA_SYSCTRL + 0x50)
#define AK98_RTC_DATA		(AK98_VA_SYSCTRL + 0x54)
#define SYSCTRL_INT_CTRL        (AK98_VA_SYSCTRL + 0x4C)
#define AK98_RTC_CDR		(AK98_VA_SYSCTRL + 0x04)
#define RTC_RDY_EN		(1 << 24)

#define RTC_WAKEUP_EN			(1 << 16)
#define RTC_CONF_RTC_WR_EN	(1 << 25)
#define RTC_CONF_RTC_EN		(1 << 24)
#define RTC_CONF_RTC_READ	((1 << 21) | (2 << 18) | (1 << 17))
#define RTC_CONF_RTC_WRITE	((1 << 21) | (2 << 18) | (0 << 17))

#define AK98_RTC_REAL_TIME1	(0x0)
#define AK98_RTC_REAL_TIME2	(0x1)
#define AK98_RTC_REAL_TIME3	(0x2)
#define AK98_RTC_ALARM_TIME1	(0x3)
#define AK98_RTC_ALARM_TIME2	(0x4)
#define AK98_RTC_ALARM_TIME3	(0x5)
#define AK98_WDT_RTC_TIMER_CONF	(0x6)
#define AK98_RTC_SETTING	(0x7)
#define AK98_RTC_REG_MAX	AK98_RTC_SETTING

#define RTC_ON 1
#define RTC_OFF 0
#define RTC_SETTING_REAL_TIME_RE	(1 << 4)
#define RTC_SETTING_REAL_TIME_WR	(1 << 3)

/*
 * When the RTC module begins to receive/send data, bit [24] of Interrupt Enable/Status
 * Register of System Control Module (Add: 0x0800, 004C) is set to 0; and then this
 * bit is set to 1 automatically to indicate that the data has been well received/sent
 */
static void inline ak98_rtc_wait_ready(void)
{
	while (!(__raw_readl(SYSCTRL_INT_CTRL) & RTC_RDY_EN))
		;
}

static void inline rtc_ready_irq_enable(void)
{
	unsigned long regval;

	/*
	 * Mask RTC Ready Interrupt
	 */
	regval = __raw_readl(SYSCTRL_INT_CTRL);
	__raw_writel(regval | (1<<8), SYSCTRL_INT_CTRL);

	/*
	 * Wait for RTC Ready Interrupt to be cleared
	 */
	ak98_rtc_wait_ready();

	/*
	 * Enable RTC Register Read/Write
	 */
	regval = __raw_readl(AK98_RTC_CONF);
	regval |= RTC_CONF_RTC_WR_EN;
	__raw_writel(regval, AK98_RTC_CONF);
}

static void inline rtc_ready_irq_disable(void)
{
	unsigned long regval;
	/*
	 * Disable RTC Register Read/Write
	 */
	regval = __raw_readl(AK98_RTC_CONF);
	regval &= ~RTC_CONF_RTC_WR_EN;
	__raw_writel(regval, AK98_RTC_CONF);
	
	/*
	 * Unmask RTC Ready Interrupt
	 */
	regval = __raw_readl(SYSCTRL_INT_CTRL);
	__raw_writel(regval & ~(1<<8), SYSCTRL_INT_CTRL);
}

void ak98_rtc_power(int op);

unsigned int ak98_rtc_read(unsigned int addr);
unsigned int ak98_rtc_write(unsigned int addr, unsigned int value);
unsigned int ak98_rtc_set_wpin(bool level);
void ak98_reboot_sys_by_wtd(void);
void ak98_reboot_sys_by_wakeup(void);

#endif /*  __ASM_ARCH_RTC_H */
