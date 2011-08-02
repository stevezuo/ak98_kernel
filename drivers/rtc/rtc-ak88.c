/*
 * drivers/rtc/rtc-ak880x.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/mach/time.h>

#include <mach/hardware.h>

#define AK88_RTC_CONF		(AK88_VA_SYSCTRL+0x50)
#define AK88_RTC_DATA		(AK88_VA_SYSCTRL+0x54)

#define SYSCTRL_INT_CTRL        (AK88_VA_SYSCTRL + 0x4C)
#define AK88_RTC_READY	(1<<24)

#define AK88_RTC_READ		(0x3<<17)
#define AK88_RTC_WRITE	(0x2<<17)

#define AK88_RTC_SECOND	(0x0)
#define AK88_RTC_DAYHOUR	(0x1)
#define AK88_ALARM_TIME1	(0x2)
#define AK88_ALARM_TIME2	(0x3)
#define AK88_WDT_TIMER1	(0x4)
#define AK88_WDT_TIMER2	(0x5)
#define AK88_RTC_REG_MAX	AK88_WDT_TIMER2

#define RTC_WAKEUP_SIGNAL_ENABLE	(0x1<<1)
#define RTC_WAKEUP_SIGNAL_CLEAR		(0x1<<0)
#define RTC_WAKEUP_SIGNAL_LOWACTIVE	(0x1<<2)

extern int rtc_year_days(unsigned int mday, unsigned int month, unsigned int year);
static unsigned long base[] = { 4,5,6,0,2,3,4,5,0,1,2,3,5,6,0,1,3,4,5,6,1};
static const unsigned short rtc_ydays[2][13] = {
	/* Normal years */
	{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
	/* Leap years */
	{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 }
};

static int ak880x_rtc_wakeup_enable(int en);
static int alarm_year = 2009;
static int alarm_day = 0;
static int leap_year(int y)
{
	if( (y%4==0&&y%100!=0) || (y%400==0) )
		return 1;
	else
		return 0;
}


/*
 * When the RTC module begins to receive/send data, bit [24] of Interrupt Enable/Status
 * Register of System Control Module (Add: 0x0800, 004C) is set to 0; and then this
 * bit is set to 1 automatically to indicate that the data has been well received/sent
 */
static int inline ak880x_rtc_ready(void)
{
	return __raw_readl(SYSCTRL_INT_CTRL) & AK88_RTC_READY;
}

static void inline rtc_ready_irq_enable(int enable)
{
	unsigned long regval = __raw_readl(SYSCTRL_INT_CTRL);

	if (enable)
		__raw_writel(regval | (1<<8), SYSCTRL_INT_CTRL);
	else
		__raw_writel(regval & ~(1<<8), SYSCTRL_INT_CTRL);
}

#define RTC_MAX_TIMEOUT		20000	/* that's enough */

unsigned int ak880x_rtc_read(unsigned int addr)
{
	unsigned long regval = 0;
	unsigned int timeout = 0;

	if (addr > AK88_RTC_REG_MAX)
		return -1;

	local_irq_disable();

	rtc_ready_irq_enable(1);

	regval  = __raw_readl(AK88_RTC_CONF);
	regval  &= (~0x7ffff);
	regval  |= (AK88_RTC_READ | (addr<<14));

	__raw_writel(regval, AK88_RTC_CONF);

	while (1) {
		if (timeout > RTC_MAX_TIMEOUT) {
			printk("%s: timeout, please check rtc clock\n", __FUNCTION__);
			rtc_ready_irq_enable(0);
			break;
		}
		timeout ++;

		if (ak880x_rtc_ready()) {
			rtc_ready_irq_enable(0);
			break;
		}
	}

	local_irq_enable();

	udelay(1000/32+1);
	regval = __raw_readl(AK88_RTC_DATA);
	regval = (regval & 0x3fff);

	return regval;
}
EXPORT_SYMBOL(ak880x_rtc_read);

static unsigned int ak880x_rtc_write(unsigned int addr, unsigned int value)
{
	unsigned long regval = 0;
	unsigned int timeout = 0;

	if (addr > AK88_RTC_REG_MAX)
		return -1;

	local_irq_disable();

	rtc_ready_irq_enable(1);

	value &= 0x3fff;
	regval  = __raw_readl(AK88_RTC_CONF);
	regval  &= (~0x7ffff);
	regval  |= (AK88_RTC_WRITE | (addr<<14) | value);

	__raw_writel(regval, AK88_RTC_CONF);

	while (1) {
		if (timeout > RTC_MAX_TIMEOUT) {
			printk("%s: timeout, please check rtc clock\n", __FUNCTION__);
			rtc_ready_irq_enable(0);
			return 1;
		}
		timeout ++;

		if (ak880x_rtc_ready()) {
			rtc_ready_irq_enable(0);
			break;
		}
	}
	local_irq_enable();

	/* printk("write timeout: %d\n", timeout); */

	return 0;
}

static inline void ak880x_rtc_int_clear(void) 
{
	unsigned int regval;

	regval = ak880x_rtc_read(AK88_WDT_TIMER1);
	regval &= ~0x1;
	regval |= RTC_WAKEUP_SIGNAL_CLEAR;

	ak880x_rtc_write(AK88_WDT_TIMER1, regval);
}

static inline void ak880x_alarmint_enable(void)
{
	unsigned int regval = 0;

	regval = ak880x_rtc_read(AK88_WDT_TIMER1);

	regval |= (RTC_WAKEUP_SIGNAL_ENABLE | RTC_WAKEUP_SIGNAL_CLEAR);

	ak880x_rtc_write(AK88_WDT_TIMER1, regval);
}

static int ak880x_rtc_wakeup_enable(int en)
{
	unsigned int val;

	printk("%s: %s\n", __FUNCTION__, en ? "enable" : "disable");

	/* read rtc wakeup setting and clear status */
	val = ak880x_rtc_read(AK88_WDT_TIMER1);
	val &= ~(0x7);
	val |= RTC_WAKEUP_SIGNAL_CLEAR;
	ak880x_rtc_write(AK88_WDT_TIMER1, val);
	val &= ~RTC_WAKEUP_SIGNAL_CLEAR;

	if (en)
	{
		/* enable wakeup signal */
		val |= (RTC_WAKEUP_SIGNAL_ENABLE) ;//| RTC_WAKEUP_SIGNAL_LOWACTIVE);
		ak880x_rtc_write(AK88_WDT_TIMER1, val);
	} 
	else 
	{
		/* disable wakeup signal */
		/* val |= RTC_WAKEUP_SIGNAL_CLEAR; */
		/* ak880x_rtc_write(AK88_WDT_TIMER1, val); */
	}
	return 0;
}


static int ak880x_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	unsigned int second = 0;
	unsigned int dayhour = 0;
	unsigned int year, month, day, wday, mday, weeks;
	unsigned int alarm_hour, alarm_day, alarm_wday, alarm_weeks, alarm_time1, alarm_time2;
	int leap;
	int year_delta = 0;
	int i=0, tmp=0;
	unsigned long day_of_year;

	year_delta = ak880x_rtc_read(AK88_ALARM_TIME1) % 60;
	year = 2009 + year_delta;
	leap = leap_year(year);
	day_of_year = ((leap)?365:364);

	dayhour = ak880x_rtc_read(AK88_RTC_DAYHOUR);
	weeks = (dayhour&0x3f00)>>8;
	wday =  (dayhour&0xe0)>>5;
	day = weeks*7 + wday;

	if(day > day_of_year)
	{
		day = day - day_of_year -1;
		year_delta ++;
		year ++;
		dayhour = dayhour & ~0xffe0;
		dayhour |= (day/7)<<8;
		dayhour |= (day%7)<<5;
		wday =  (dayhour&0xe0)>>5;
		ak880x_rtc_write(AK88_RTC_DAYHOUR, dayhour);

		alarm_time1 = ak880x_rtc_read(AK88_ALARM_TIME1);
		ak880x_rtc_write(AK88_ALARM_TIME1, alarm_time1 - alarm_time1%60 + year_delta);

		leap = leap_year(year);

		alarm_time2 = ak880x_rtc_read(AK88_ALARM_TIME2);
		alarm_hour = alarm_time2 & 0x1f;
		alarm_day = alarm_time2 & (~0x1f);
		alarm_weeks = (alarm_day/7)<<8;
		alarm_wday = (alarm_day%7)<<5;
		alarm_day = alarm_weeks*7 + alarm_wday;
		if(alarm_day > day_of_year)
			alarm_day = alarm_day - day_of_year - 1;
		ak880x_rtc_write(AK88_ALARM_TIME2, (alarm_day/7)<<8 | (alarm_day%7)<<5 | alarm_hour);
	}

	while(1)
	{
		if( rtc_ydays[leap][i] >= (day+1))
		{
			i--;
			break;
		}
		i++;
	}
	month = i;
	mday = day + 1 - rtc_ydays[leap][i];
	second = ak880x_rtc_read(AK88_RTC_SECOND);

	tm->tm_year = year - 1900;
	tm->tm_hour = dayhour & 0x1f;
	tm->tm_min = second / 60;
	tm->tm_sec = second % 60;
	tm->tm_mon = month;
	tm->tm_mday = mday; 
	tm->tm_wday = (wday + base[year_delta])%7; 
	tm->tm_yday = day;
	tm->tm_isdst = -1;

	return 0;
}


static int ak880x_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	unsigned int second = 0;
	unsigned int dayhour = 0;
	unsigned int year_delta = 0;
	unsigned int rtc_time1 = 0;
	unsigned int rtc_set_year = 0;
	unsigned int alarm_set_year = 0;
	int days;

	year_delta = (tm->tm_year + 1900 - 2009);
	rtc_time1 = ak880x_rtc_read(AK88_ALARM_TIME1);
	rtc_time1 = rtc_time1 - rtc_time1 % 60 + year_delta;
	ak880x_rtc_write(AK88_ALARM_TIME1, rtc_time1);

	if(tm->tm_hour>=0 && tm->tm_min>=0 && tm->tm_sec>=0)
	{
		second = tm->tm_min * 60 + tm->tm_sec;
		ak880x_rtc_write(AK88_RTC_SECOND, second);
		dayhour = ak880x_rtc_read(AK88_RTC_DAYHOUR) & (~0x1f);
		dayhour |= tm->tm_hour;
		ak880x_rtc_write(AK88_RTC_DAYHOUR, dayhour);
	}

	if(tm->tm_mon>=0 && tm->tm_mday>=0)
	{
		days = tm->tm_yday;
		dayhour = ak880x_rtc_read(AK88_RTC_DAYHOUR) & ~0xffe0;
		dayhour |= (days/7)<<8;
		dayhour |= (days%7)<<5;
		ak880x_rtc_write(AK88_RTC_DAYHOUR, dayhour);
	}
	return 0;
}

static int ak880x_rtc_getalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	/* unsigned int second = 0;
	unsigned int dayhour = 0;
	int ret;

	second = ak880x_rtc_read(AK88_ALARM_TIME1);
	dayhour = ak880x_rtc_read(AK88_ALARM_TIME2);

	alarm_day = ((dayhour&0x3f00)>>8)*7 + ((dayhour&0xe0)>>5)+1;
	wkalrm->time.tm_sec = (second+1) % 60;
	wkalrm->time.tm_min = (second+1) / 60;
	wkalrm->time.tm_hour = dayhour & 0x1f;
	wkalrm->time.tm_yday = alarm_day; 
	wkalrm->time.tm_mon = 0; 
	wkalrm->time.tm_mday = 0; 
	wkalrm->time.tm_wday = 0; 
	wkalrm->time.tm_year = 0; 
	wkalrm->time.tm_isdst = -1; */

	return 0;
}

static int ak880x_rtc_setalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	unsigned long second = 0;
	unsigned long alarm_dayhour = 0;
	unsigned long rtc_dayhour = 0;
	unsigned long year, month, day, wday, mday, weeks;
	int leap;
	int year_delta = 0;
	int i=0, tmp=0;
	unsigned long day_of_year;

	alarm_year = wkalrm->time.tm_year + 1900; 
	alarm_day = rtc_year_days(wkalrm->time.tm_mday, wkalrm->time.tm_mon, alarm_year);
	year_delta = ak880x_rtc_read(AK88_ALARM_TIME1) % 60;
	year = 2009 + year_delta;
	leap = leap_year(year);

	day_of_year = ((leap)?365:364);
	rtc_dayhour = ak880x_rtc_read(AK88_RTC_DAYHOUR);
	weeks = (rtc_dayhour&0x3f00)>>8;
	wday =  (rtc_dayhour&0xe0)>>5;
	day = weeks*7 + wday;
	if(day > day_of_year)
	{
		day = day - day_of_year - 1;
		year_delta ++;
		year ++;
		rtc_dayhour = rtc_dayhour & 0x1f;
		rtc_dayhour |= (day/7)<<8;
		rtc_dayhour |= (day%7)<<5;
		/* wday =  (rtc_dayhour&0xe0)>>5; */
		ak880x_rtc_write(AK88_RTC_DAYHOUR, rtc_dayhour);
		leap = leap_year(year);
	}

	if((alarm_year - year) < 0)
	{
		ak880x_rtc_wakeup_enable(0);
		printk("alarm set error\n");
		return -1;
	}

	if((alarm_year - year) > 1)
	{
		ak880x_rtc_wakeup_enable(0);
		printk("alarm time: over one year\n");
		return -1;
	}

	if(wkalrm->time.tm_hour>=0 && wkalrm->time.tm_min>=0 && wkalrm->time.tm_sec>=0)
	{
		/* second = wkalrm->time.tm_min * 60 + wkalrm->time.tm_sec; */
		second = wkalrm->time.tm_min * 60;
		second += year_delta;
		ak880x_rtc_write(AK88_ALARM_TIME1, second);

		alarm_dayhour = ak880x_rtc_read(AK88_ALARM_TIME2) & ~0x1f;
		alarm_dayhour |= wkalrm->time.tm_hour;
		ak880x_rtc_write(AK88_ALARM_TIME2, alarm_dayhour);
	}

	if((alarm_year - year) == 0)
	{
		alarm_dayhour = ak880x_rtc_read(AK88_ALARM_TIME2) & 0x1f;
		alarm_dayhour |= (alarm_day/7)<<8;
		alarm_dayhour |= (alarm_day%7)<<5;
		ak880x_rtc_write(AK88_ALARM_TIME2, alarm_dayhour);
	}

	if((alarm_year - year) == 1)
	{
		if(alarm_day > day)
		{
			ak880x_rtc_wakeup_enable(0);
			printk("alarm time: over one year\n");
			return -1;
		}
		else
		{
			day_of_year = leap_year(year);
			alarm_dayhour = ak880x_rtc_read(AK88_ALARM_TIME2) & 0x1f;
			alarm_dayhour |= ((day_of_year +1 + alarm_day)/7)<<8;
			alarm_dayhour |= ((day_of_year +1 + alarm_day)%7)<<5;
			ak880x_rtc_write(AK88_ALARM_TIME2, alarm_dayhour);
		}
	}

	if (wkalrm->enabled)
		ak880x_rtc_wakeup_enable(1);
	else
		ak880x_rtc_wakeup_enable(0);

	return 0;
}

static int ak880x_rtc_ioctl(struct device *dev,
		unsigned int cmd, unsigned long arg)
{
	unsigned int ret = -ENOIOCTLCMD;

	switch (cmd) {
		case RTC_AIE_OFF:
			ak880x_rtc_wakeup_enable(0);
			ret = 0;
			break;
		case RTC_AIE_ON:
			ak880x_rtc_wakeup_enable(1);
			ret = 0;
			break;

		case RTC_UIE_ON:
		case RTC_UIE_OFF:
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int ak880x_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}

static irqreturn_t ak880x_rtc_alarmirq(int irq, void *id)
{
	struct rtc_device *rdev = id;
	ak880x_rtc_int_clear();
	rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);

	return IRQ_HANDLED;
}

static int ak880x_rtc_open(struct device *dev)
{
	return 0;
}

static void ak880x_rtc_release(struct device *dev)
{
}

static const struct rtc_class_ops ak880x_rtc_ops = {
	.open		= ak880x_rtc_open,
	.release	= ak880x_rtc_release,
	.ioctl		= ak880x_rtc_ioctl,
	.read_time	= ak880x_rtc_gettime,
	.set_time	= ak880x_rtc_settime,
	.read_alarm	= ak880x_rtc_getalarm,
	.set_alarm	= ak880x_rtc_setalarm,
	.proc	        = ak880x_rtc_proc,
};

static int ak880x_rtc_remove(struct platform_device *dev)
{
	struct rtc_device *rtc = platform_get_drvdata(dev);

	free_irq(IRQ_RTC_ALARM, rtc);

	platform_set_drvdata(dev, NULL);
	rtc_device_unregister(rtc);

	return 0;
}

static int ak880x_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	/* struct resource *res; */
	int ret;

	rtc = rtc_device_register("ak880x-rtc", &pdev->dev, &ak880x_rtc_ops,
			THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);

		return -1;
	}

	rtc->max_user_freq = 128;

	platform_set_drvdata(pdev, rtc);

	ak880x_rtc_wakeup_enable(0);

	ret = request_irq(IRQ_RTC_ALARM, ak880x_rtc_alarmirq,
			IRQF_DISABLED,  "ak880x-rtc alarm", rtc);
	if (ret) {
		printk("IRQ %d error %d\n", IRQ_RTC_ALARM, ret);
		return ret;
	}

	return 0;

}

#ifdef CONFIG_PM

static void ak880x_rtc_enable(struct platform_device *pdev, int en)
{
}
static struct timespec ak880x_rtc_delta;
/* RTC Power management control */
static int ak880x_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;

	/* calculate time delta for suspend */

	ak880x_rtc_gettime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&ak880x_rtc_delta, &time);

	return 0;
}

static int ak880x_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;
	ak880x_rtc_gettime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&ak880x_rtc_delta, &time);
	return 0;
}
#else
#define ak880x_rtc_suspend NULL
#define ak880x_rtc_resume  NULL
#endif

static struct platform_driver ak880x_rtcdrv = {
	.probe		= ak880x_rtc_probe,
	.remove		= ak880x_rtc_remove,
	.suspend	= ak880x_rtc_suspend,
	.resume		= ak880x_rtc_resume,
	.driver		= {
		.name	= "ak880x-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init ak880x_rtc_init(void)
{
	printk("AK88 RTC, (c) 2010 ANYKA \n");
	return platform_driver_register(&ak880x_rtcdrv);
}

static void __exit ak880x_rtc_exit(void)
{
	platform_driver_unregister(&ak880x_rtcdrv);
}

module_init(ak880x_rtc_init);
module_exit(ak880x_rtc_exit);

MODULE_DESCRIPTION("ANYKA AK88 RTC Driver");
MODULE_AUTHOR("anyka");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak880x-rtc");
