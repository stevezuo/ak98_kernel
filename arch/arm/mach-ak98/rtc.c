/*
 * linux/arch/arm/mach-ak98/rtc.c
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
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
#include <mach/rtc.h>


static int rtc_cnt = 0;

#undef REG32
#define REG32(_reg_)  (*(volatile unsigned long *)(_reg_))

//reboot system by watchdog
void ak98_reboot_sys_by_wtd(void)
{
	unsigned long val;	
	unsigned long flags;
	//static spinlock_t loc_lock;

	//spin_lock_init(&loc_lock);
	//spin_lock(&loc_lock);
	local_irq_save(flags);
	ak98_rtc_power(RTC_ON);

	//select wdt
	val = ak98_rtc_read(AK98_RTC_SETTING);	
	val |= (1 << 10);	
	ak98_rtc_write(AK98_RTC_SETTING, val);	
	
	//clear timer
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val |= (1<<6);
	ak98_rtc_write(AK98_RTC_SETTING, val);
	

	//enable watchdog timer
	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val |= (1<<13);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);
		
	//set timer
	val = ak98_rtc_read(AK98_WDT_RTC_TIMER_CONF);
	val &= (1<<13);
	val |= (5 & 0x1FFF);
	ak98_rtc_write(AK98_WDT_RTC_TIMER_CONF, val);

	
	//open watchdog and watchdog output
	val = ak98_rtc_read(AK98_RTC_SETTING);
	val |= ((1<<5) | (1<<2));
	ak98_rtc_write(AK98_RTC_SETTING, val);

	local_irq_restore(flags);
//	spin_unlock(&loc_lock);
}
EXPORT_SYMBOL(ak98_reboot_sys_by_wtd);

void ak98_reboot_sys_by_wakeup(void)
{
	//static spinlock_t loc_lock;
	struct rtc_time ptm;
	struct rtc_time *tm = &ptm;
	//unsigned char mdays[13] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
	unsigned long flags;
	
	unsigned long rtcset;
	unsigned long rtc_time1;
	unsigned long rtc_time2;
	unsigned long rtc_time3;	
	
	unsigned long rtc_alarm1;
	unsigned long rtc_alarm2;
	unsigned long rtc_alarm3;	
	unsigned int val_1, val_2;
	unsigned long time;
	
	//spin_lock_init(&loc_lock);
	//spin_lock(&loc_lock);
	local_irq_save(flags);
	ak98_rtc_power(RTC_ON);
	
	rtcset = ak98_rtc_read(AK98_RTC_SETTING);
	rtcset |= RTC_SETTING_REAL_TIME_RE;
	ak98_rtc_write(AK98_RTC_SETTING, rtcset);
	
	rtc_time1 = ak98_rtc_read(AK98_RTC_REAL_TIME1);
	rtc_time2 = ak98_rtc_read(AK98_RTC_REAL_TIME2);
	rtc_time3 = ak98_rtc_read(AK98_RTC_REAL_TIME3);
	
	tm->tm_year  = ((rtc_time3 >> 4) & 0x7F) - EPOCH_START_YEAR + RTC_START_YEAR;
	tm->tm_mon   = (rtc_time3 & 0xF) - 1;
	tm->tm_mday  = (rtc_time2 >> 5) & 0x1F;
	tm->tm_hour  = rtc_time2 & 0x1F;
	tm->tm_min   = (rtc_time1 >> 6) & 0x3F;
	tm->tm_sec   = rtc_time1 & 0x3F;
	tm->tm_wday  = (rtc_time2 >> 10) & 0x7;
	tm->tm_isdst = -1;	
	
	rtc_alarm1 = ak98_rtc_read(AK98_RTC_ALARM_TIME1);
	rtc_alarm2 = ak98_rtc_read(AK98_RTC_ALARM_TIME2);
	rtc_alarm3 = ak98_rtc_read(AK98_RTC_ALARM_TIME3);
	
	
	rtc_alarm1 &= ~(0xFFF);
	rtc_alarm2 &= ~(0x3FF);
	rtc_alarm3 &= ~(0x7FF);
	
	#define DELAY_TIME 5
	
	rtc_tm_to_time(tm, &time);

	time += DELAY_TIME;

	rtc_time_to_tm(time, tm);
	/*tm->tm_sec += DELAY_TIME;
	
	if (tm->tm_sec >= 60) 
	{
		tm->tm_sec -= 60;
		tm->tm_min++;
		if (tm->tm_min >= 60)
		{
			tm->tm_min = 0;
			tm->tm_hour++;						
			if (tm->tm_hour >= 24)
			{
				tm->tm_hour = 0;
				tm->tm_mday++;
				if ((tm->tm_year%400==0) || ((tm->tm_year%100!=0) && (tm->tm_year%4==0))) 
					mdays[2] = 29;
				if (tm->tm_mday > mdays[tm->tm_mon])
				{
					tm->tm_mday = 1;
					tm->tm_mon++;
					if (tm->tm_mon > 12)
					{
						tm->tm_mon = 1;
						tm->tm_year++;
					}
				}
			}
		}
			
	}*/
	rtc_alarm1 |= ((tm->tm_min << 6) + tm->tm_sec);
	rtc_alarm2 |= ((tm->tm_mday << 5) + tm->tm_hour);
	rtc_alarm3 |= (((tm->tm_year + EPOCH_START_YEAR - RTC_START_YEAR) << 4) + (tm->tm_mon + 1));
	
	ak98_rtc_write(AK98_RTC_ALARM_TIME1, rtc_alarm1);
	ak98_rtc_write(AK98_RTC_ALARM_TIME2, rtc_alarm2);
	ak98_rtc_write(AK98_RTC_ALARM_TIME3, rtc_alarm3);
	
	val_1 = REG32(AK98_RTC_CDR);
	
	
	val_2 = ak98_rtc_read(AK98_RTC_SETTING);
	
	if (1)
	{
		/* enable wakeup signal */
		val_1 |= RTC_WAKEUP_EN;// | RTC_WAKEUP_SIGNAL_LOWACTIVE);
		REG32(AK98_RTC_CDR) =  val_1;
		
		val_2 |= (1<<2);
		ak98_rtc_write(AK98_RTC_SETTING, val_2);
	} 
	
	rtc_alarm1 = ak98_rtc_read(AK98_RTC_ALARM_TIME1);
	rtc_alarm2 = ak98_rtc_read(AK98_RTC_ALARM_TIME2);
	rtc_alarm3 = ak98_rtc_read(AK98_RTC_ALARM_TIME3);
	
	rtc_alarm1 |= (1<<13);
	rtc_alarm2 |= (1<<13);
	rtc_alarm3 |= (1<<13);
	
	ak98_rtc_write(AK98_RTC_ALARM_TIME1, rtc_alarm1);
	ak98_rtc_write(AK98_RTC_ALARM_TIME2, rtc_alarm2);
	ak98_rtc_write(AK98_RTC_ALARM_TIME3, rtc_alarm3);	

	local_irq_restore(flags);
//	spin_unlock(&loc_lock);
	ak98_rtc_set_wpin(0);
}

EXPORT_SYMBOL(ak98_reboot_sys_by_wakeup);

void ak98_rtc_power(int op)
{
	unsigned long rtcconf;

	switch(op)
	{
	case RTC_ON:
		if (++rtc_cnt == 1)
		{
			rtcconf = __raw_readl(AK98_RTC_CONF);
			rtcconf |= RTC_CONF_RTC_EN;
			__raw_writel(rtcconf, AK98_RTC_CONF);
		}
		break;
		/*
		When RTC is powered off, this bit(AK98_RTC_CONF [24] ) should be set to 0
	*/
	case RTC_OFF:
		if (!(--rtc_cnt))
		{
			rtcconf = __raw_readl(AK98_RTC_CONF);
			rtcconf &= ~RTC_CONF_RTC_EN;
			__raw_writel(rtcconf, AK98_RTC_CONF);
		}
		break;
	default:
		printk("Error RTC power operation.\n");
		break;
	}
}

EXPORT_SYMBOL(ak98_rtc_power);


unsigned int ak98_rtc_read(unsigned int addr)
{
	unsigned long regval = 0;
	unsigned long flags;

	if (addr > AK98_RTC_REG_MAX) {
		printk("%s(): Invalid RTC Register, address=%d\n",
			__func__, addr);
		return -1;
	}

	local_irq_save(flags);

	rtc_ready_irq_enable();

	regval  = __raw_readl(AK98_RTC_CONF);
	regval  &= ~(0x3FFFFF) ;
	regval  |= (RTC_CONF_RTC_READ | (addr << 14));
	__raw_writel(regval, AK98_RTC_CONF);

	ak98_rtc_wait_ready();

	regval = __raw_readl(AK98_RTC_DATA);
	regval &= 0x3FFF;
	
	rtc_ready_irq_disable();

	local_irq_restore(flags);
	
	return regval;
}
EXPORT_SYMBOL(ak98_rtc_read);

unsigned int ak98_rtc_write(unsigned int addr, unsigned int value)
{
	unsigned long regval = 0;
	unsigned long flags;

	if (addr > AK98_RTC_REG_MAX) {
		printk("%s(): Invalid RTC Register, address=%d\n",
			__func__, addr);
		return -1;
	}

	local_irq_save(flags);

	rtc_ready_irq_enable();

	regval  = __raw_readl(AK98_RTC_CONF);
	regval  &= ~0x3FFFFF;
	regval  |= (RTC_CONF_RTC_WRITE | (addr << 14) | value);
	__raw_writel(regval, AK98_RTC_CONF);

	ak98_rtc_wait_ready();
	
	rtc_ready_irq_disable();

	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(ak98_rtc_write);

unsigned int ak98_rtc_set_wpin(bool level)
{
	unsigned long regval;
	unsigned int bit;

	ak98_rtc_power(RTC_ON);

	bit = level ? 8 : 7;

	regval = ak98_rtc_read(AK98_RTC_SETTING);
	regval |= (1 << bit);
	ak98_rtc_write(AK98_RTC_SETTING, regval);

	while (ak98_rtc_read(AK98_RTC_SETTING) & (1 << bit))
		;

	return 0;
}
EXPORT_SYMBOL(ak98_rtc_set_wpin);
