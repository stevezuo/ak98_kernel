/*
 * drivers/char/watchdog/ak98_wdt.c
 *
 * Watchdog driver for ANYKA ak98 processors
 *
 * Author: Wenyong Zhou
 *
 * Adapted from the IXP2000 watchdog driver by Lennert Buytenhek.
 * The original version carries these notices:
 *
 * Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * Copyright 2004 (c) MontaVista, Software, Inc.
 * Based on sa1100 driver, Copyright (C) 2000 Oleg Drokin <green@crimea.edu>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>

#include <mach/rtc.h>


static int nowayout = WATCHDOG_NOWAYOUT;
static unsigned int heartbeat = (0x1FFF - 1);	/* (secs) Default is 8091ms */
static unsigned long wdt_status;
static spinlock_t wdt_lock;

#define	WDT_IN_USE		0
#define	WDT_OK_TO_CLOSE		1

//static unsigned long wdt_tick_rate;
#undef REG32
#define REG32(_reg_)  (*(volatile unsigned long *)(_reg_))

#define SELECT_WTC 1
#define SELECT_RTC 0

//#define WDT_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef WDT_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
# define PDEBUG(fmt, args...) printk( KERN_INFO fmt,## args)
# else
/* This one for user space */
# define PDEBUG(fmt, args...) fprintf(stderr, "%s %d: "fmt,__FILE__, __LINE__, ## args)
# endif
#else
# define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif


static int select_wdt(int which)
{
	unsigned long val;
	int ret;

	val = ak98_rtc_read(AK98_RTC_SETTING);
	ret = val | (1 << 10);	
	if (which == 0)
		val &= ~(1 << 10);		
	else
		val |= (which << 10);
	
	
	ak98_rtc_write(AK98_RTC_SETTING, val);
	return ret;
}
static void wdt_enable(void)
{
	unsigned long val;
	int pre;
	PDEBUG("%s......\n", __FUNCTION__);
	spin_lock(&wdt_lock);	
	pre = select_wdt(SELECT_WTC);

	//enable watchdog timer
	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val |= (1<<13);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);

	//set timer
	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val &= (1<<13);
	val |= (heartbeat & 0x1FFF);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);

	//open watchdog and watchdog output
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val |= ((1<<5) | (1<<2));
	ak98_rtc_write(AK98_RTC_SETTING, val);	
	select_wdt(pre);
	spin_unlock(&wdt_lock);
}

static void wdt_disable(void)
{
	unsigned long val;
	int pre;
	
	spin_lock(&wdt_lock);
	pre = select_wdt(SELECT_WTC);
	//clear watchdog timer
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val |= (1<<6);
	ak98_rtc_write(AK98_RTC_SETTING, val);

	//disable watchdog timer
	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val &= ~(1<<13);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);

	//close watchdog and watchdog output
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val &= ~((1<<2) | (1<<5));
	ak98_rtc_write(AK98_RTC_SETTING, val);
	
	select_wdt(pre);	
	spin_unlock(&wdt_lock);
}

static void wdt_keepalive(void)
{
	unsigned long val;
	int pre;
	PDEBUG("%s......\n", __FUNCTION__);
	spin_lock(&wdt_lock);

	pre = select_wdt(SELECT_WTC);

	//clear watchdog timer
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val |= (1<<6);
	ak98_rtc_write(AK98_RTC_SETTING, val);


	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val &= (1<<13);
	val |= (heartbeat & 0x1FFF);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);

	select_wdt(pre);
	spin_unlock(&wdt_lock);
}

static int ak98_wdt_open(struct inode *inode, struct file *file)
{
	PDEBUG("%s......\n", __FUNCTION__);
	if (test_and_set_bit(WDT_IN_USE, &wdt_status))
		return -EBUSY;

	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

	wdt_enable();

	return nonseekable_open(inode, file);
}

static ssize_t ak98_wdt_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	PDEBUG("wdt write: %s  %d\n", data, len);
	if (len) {
		if (!nowayout) {
			size_t i;

			clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					set_bit(WDT_OK_TO_CLOSE, &wdt_status);
			}
		}
		wdt_keepalive();
	}

	return len;
}


static struct watchdog_info ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT |
				WDIOF_KEEPALIVEPING,
	.identity	= "ANYKA ak98 Watchdog",
};

static long ak98_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = -ENOTTY;
	int time;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_KEEPALIVE:
		wdt_enable();
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *)arg);
		if (ret)
			break;

		if (time <= 0 || time > 8) {
			ret = -EINVAL;
			break;
		}
		if (time == 8)
			heartbeat = 0x1FFF - 1;
		else
			heartbeat = time * 1024;
		wdt_keepalive();
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat/1024, (int *)arg);
		break;
	}

	return ret;
}

static int ak98_wdt_release(struct inode *inode, struct file *file)
{
	PDEBUG("%s......\n", __FUNCTION__);

	if (test_bit(WDT_OK_TO_CLOSE, &wdt_status))
		wdt_disable();
	else
		printk(KERN_CRIT "WATCHDOG: Device closed unexpectedly - "
					"timer will not stop\n");
	clear_bit(WDT_IN_USE, &wdt_status);
	clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
	
	return 0;
}


static const struct file_operations ak98_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= ak98_wdt_write,
	.unlocked_ioctl	= ak98_wdt_ioctl,
	.open		= ak98_wdt_open,
	.release	= ak98_wdt_release,
};

static struct miscdevice ak98_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &ak98_wdt_fops,
};

static int __init ak98_wdt_init(void)
{		
	spin_lock_init(&wdt_lock);
	PDEBUG("%s......\n", __FUNCTION__);
	wdt_status = 0;
	
	spin_lock(&wdt_lock);
	ak98_rtc_power(RTC_ON);
	spin_unlock(&wdt_lock);	
	
	return misc_register(&ak98_wdt_miscdev);
}

static void __exit ak98_wdt_exit(void)
{
	spin_lock(&wdt_lock);

	ak98_rtc_power(RTC_OFF);
	spin_unlock(&wdt_lock);	
	misc_deregister(&ak98_wdt_miscdev);
}

module_init(ak98_wdt_init);
module_exit(ak98_wdt_exit);


MODULE_DESCRIPTION("ANYKA ak98 Watchdog");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds (default 60s)");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

