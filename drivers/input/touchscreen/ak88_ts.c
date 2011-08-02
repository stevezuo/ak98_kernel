/*
 * drivers/input/touchscreen/ak880x_ts.c
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/regs-adc.h>
#include <mach/gpio.h>
#include <mach/ts.h>

/* For ts.dev.id.version */
#define AK88TSVERSION	0x0101

/*
 * Definitions & global arrays.
 */

static char *ak880xts_name = "AK88 TouchScreen";

/*
 * Per-touchscreen data.
 */

struct ak880xts {
	struct input_dev *dev;
	unsigned int state_pin;
	unsigned int irq;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
};

static struct ak880xts ts;

static void touch_timer_handler(unsigned long data);
static struct timer_list touch_timer = TIMER_INITIALIZER(touch_timer_handler, 0, 0);

#define ADC_MAIN_CLK	(12 * 1000000)	/* 12MHz */
#define ADC_CLK		( 4 * 1000000)	/*  4MHz */

#define TS_POWER_ON_ADC		*(volatile unsigned long*)(AK88_ANALOG_CTRL1) &= ~(1<<26)
#define TS_POWER_OFF_ADC	*(volatile unsigned long*)(AK88_ANALOG_CTRL1) |= (1<<26)
// Read ADC power  status ,   1-- off    (for AK3223)
#define TS_ADC_POWER_STA	(*(volatile unsigned long*)(AK88_ANALOG_CTRL1) & (1<<26))


static void init_ts_hw(unsigned int SampleRate, unsigned int WaitTime)
{
#if 1
	unsigned long ClkDiv = 0;
	unsigned long bitcycle = 0;
	unsigned long temp;
	unsigned long HoldTime = 0;

	//reset ADC1
	*(volatile unsigned long *)AK88_CLK_DIV2 &= ~(1 << 22);
	mdelay(1);
	*(volatile unsigned long *)AK88_CLK_DIV2 |= (1 << 22);

	ClkDiv = (ADC_MAIN_CLK / ADC_CLK) - 1;

	temp = *(volatile unsigned long *)(AK88_CLK_DIV2);
	temp &= ~(1 << 29);
	temp |= (1 << 22) | (1 << 3) | (ClkDiv << 0);
	*(volatile unsigned long *)AK88_CLK_DIV2 = temp;

	/* because ADC1 is 5 channel multiplex*/
	SampleRate	 = SampleRate * 5;
	bitcycle	= (unsigned long)ADC_CLK / SampleRate;
	HoldTime	= bitcycle - 1;

	*(volatile unsigned long*)AK88_ADC1_CTRL = (HoldTime << 16) | bitcycle;

	//power on touch screen interface, select 5 channel mode
	*(volatile unsigned long*)AK88_ANALOG_CTRL1 |= (1 << 29);
	*(volatile unsigned long*)AK88_ANALOG_CTRL1 &= ~((1 << 26) | (1 << 27)  | (1 << 28));

	//Enable touch screen, set TS_THRESHOLD and TS_WaitTime
	*(volatile unsigned long*)AK88_ANALOG_CTRL2 |= ((0x3FF << 17) | (WaitTime << 0) | (1 << 10));

	//Enable ADC1
	*(volatile unsigned long*)AK88_ANALOG_CTRL2 |= (1 << 8);

	//Enable Battery monitor
	*(volatile unsigned long*)AK88_ANALOG_CTRL1 |= (1 << 28);   //eable battery volatage divider

#else
	/* unsigned long regval; */

	*(volatile unsigned long*)AK88_ANALOG_CTRL1 &= ~(PD_TS | RM_DIR);

	*(volatile unsigned long*)AK88_ANALOG_CTRL1 |= AD5_sel1;

	*(volatile unsigned long*)AK88_CLK_DIV2 &= ~ADC1_pd;
	*(volatile unsigned long*)AK88_CLK_DIV2 |= (ADC1_rst | ADC1_CLK_en | ADC1_DIV2);

	*(volatile unsigned long*)AK88_ADC1_CTRL = 0x04000401;

	*(volatile unsigned long*)AK88_ANALOG_CTRL2 |= ( 0x08<<17 | TS_ctrl255 | TS_en);

	/* 0x08000064 Enable ADC1 */
	*(volatile unsigned long*)AK88_ANALOG_CTRL2 |= ADC1_en;
#endif
}

static void touch_timer_handler(unsigned long data)
{
	volatile unsigned long xdata[4] = { 0, 0, 0, 0 };
	volatile unsigned long ydata[4] = { 0, 0, 0, 0 };
	unsigned int i = 0;

	if (ak880x_gpio_getpin(ts.state_pin)) {
		
		input_report_key(ts.dev, BTN_TOUCH, 0);
		input_report_abs(ts.dev, ABS_PRESSURE, 0);
		input_sync(ts.dev);

		enable_irq(ts.irq);

	} else {

		while (TS_ADC_POWER_STA != 0) {
			TS_POWER_ON_ADC;
			udelay(10);
		}

		while (i++ < 3) {

			mb();

			xdata[i] = __raw_readl(AK88_X_COORDINATE) & 0x3FF;
			ydata[i] = __raw_readl(AK88_Y_COORDINATE) & 0x3FF;

			mdelay(1);

#ifdef	CONFIG_TOUCHSCREEN_AK88_DEBUG
			/* FIXME:
			 * åŽæ“—…Ž‰æ“—ˆ™é‡Œçš„æ‰“åå”èŸ¡‰¸‘æè”¼ŒTG3æ’…æ’Ÿ•æ âˆª‡†æ“—ˆ‡éŠéˆ­†åš—Œéœ€é–¬ééžˆ£ çžéžŽŠè…¦ */
			printk(KERN_DEBUG "XP: %ld\n", xdata[i]);
			printk(KERN_DEBUG "YP: %ld\n", ydata[i]);
#endif
		}

		while (TS_ADC_POWER_STA == 0) {
			TS_POWER_OFF_ADC;
			udelay(10);
		}

		input_report_abs(ts.dev, ABS_X, xdata[2]);
		input_report_abs(ts.dev, ABS_Y, ydata[2]);

		input_report_key(ts.dev, BTN_TOUCH, 1);
		input_report_abs(ts.dev, ABS_PRESSURE, 1);

		input_sync(ts.dev);

		/* modify timer */
		mod_timer(&touch_timer, jiffies+HZ/100);
	}
}

/* 
 * touch screen interrupt handler.
 */
static irqreturn_t ak7801ts_irqhandler(int irq, void *dev)
{
	disable_irq(irq);

	/* printk("%s: irq no %d\n", __FUNCTION__, irq); */
	
	/* touch_timer_handler(0); */
	mod_timer(&touch_timer, jiffies+HZ/100);

	return IRQ_HANDLED;
}

/*
 * The functions for inserting/removing us as a module.
 */

static int __init ak880x_ts_probe(struct platform_device *dev)
{
	struct ak880x_ts_mach_info *info;

	info = (struct ak880x_ts_mach_info *)dev->dev.platform_data;
	if (!info) {
		printk(KERN_ERR "no platform data for ts\n");
		return -EINVAL;
	}

	/* initialize hardware */
	init_ts_hw(info->sample_rate, info->wait_time);
	TS_POWER_OFF_ADC;

	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct ak880xts));
	ts.dev = input_allocate_device();
	if (!ts.dev)
		return -ENOMEM;

	ts.dev->evbit[0] = BIT(EV_SYN) | BIT(EV_KEY) | BIT(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_WORD(BTN_TOUCH);
	input_set_abs_params(ts.dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, 1, 0, 0);

	sprintf(ts.phys, "ts0");

	//ts.dev->private = &ts;
	ts.dev->name = ak880xts_name;
	ts.dev->phys = ts.phys;
	ts.dev->id.bustype = BUS_RS232;
	ts.dev->id.vendor = 0xDEAD;
	ts.dev->id.product = 0xBEEF;
	ts.dev->id.version = AK88TSVERSION;

	ts.state_pin	= info->irqpin;
	ts.irq		= info->irq;

	ak880x_gpio_cfgpin(info->irqpin, AK88_GPIO_IN_0); 
	ak880x_gpio_pullup(info->irqpin, 1);
	
	set_irq_type(info->irq, IRQ_TYPE_LEVEL_LOW);
#if 1
	if (request_irq(info->irq, ak7801ts_irqhandler, 0, "ak880x_ts", ts.dev)) {
		printk(KERN_ERR "Could not allocate IRQ %d\n", info->irq);
		return -EIO;
	}
#endif

	printk(KERN_INFO "%s successfully loaded\n", ak880xts_name);

	/* All went ok, so register to the input system */
	return input_register_device(ts.dev);
}

static int ak880x_ts_remove(struct platform_device *dev)
{
	struct ak880x_ts_mach_info *info;

	info = (struct ak880x_ts_mach_info *)dev->dev.platform_data;
	if (!info) {
		printk(KERN_ERR "no platform data for ts\n");
		return -EINVAL;
	}

	disable_irq(info->irq);
	free_irq(info->irq, ts.dev);

#if 0
	if (adc_clock) {
		clk_disable(adc_clock);
		clk_put(adc_clock);
		adc_clock = NULL;
	}
#endif

	input_unregister_device(ts.dev);

	return 0;
}

#ifdef CONFIG_PM

static int ak880x_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int ak880x_ts_resume(struct platform_device *dev)
{
	return 0;
}
#endif


static struct platform_driver ak880x_ts_driver = {
	.driver		= {
		.name	= "ak7801-ts",
		.owner	= THIS_MODULE,
	},
	.probe		= ak880x_ts_probe,
	.remove		= ak880x_ts_remove,
#ifdef	CONFIG_PM
	.suspend	= ak880x_ts_suspend,
	.resume		= ak880x_ts_resume,
#endif
};

static int __init ak880x_ts_init(void)
{
	printk("AK88 Touchscreen Driver, (c) 2010 ANYKA\n");

	return platform_driver_register(&ak880x_ts_driver);
}

static void __exit ak880x_ts_exit(void)
{
	platform_driver_unregister(&ak880x_ts_driver);
}

module_init(ak880x_ts_init);
module_exit(ak880x_ts_exit);

MODULE_AUTHOR("ANYKA");
MODULE_DESCRIPTION("ak880x touchscreen driver");
MODULE_LICENSE("GPL");


