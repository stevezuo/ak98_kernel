/*
 * drivers/input/touchscreen/ak98_adc_ts.c
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
#include <mach/adc1.h>

/* For ts.dev.id.version */
#define AK98TSVERSION	0x0101
#define MAX_10BIT		(0x3FF)
#define AK98_GPIO_OUT_0		0
#define AK98_GPIO_IN_0		1



#define VARIANCE
//#define ADC_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef ADC_DEBUG
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

/*
 * Definitions & global arrays.
 */

static char *ak98ts_name = "AK98 ADC TouchScreen";


struct TS_SAMPLE {
	s32		x;
	s32		y;
};

/*
 * Per-touchscreen data.
 */

struct ak98ts {
	struct input_dev *dev;
	unsigned int state_pin;
	unsigned int irq;
	bool pendown;
	bool first;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
};

static struct ak98ts ts;

/***********************************************************/

#define PEROID_DELAY		3
#define POLL_DELAY			10
#define MAX_DELAY			50

#define SLOW_DELTA			20


/***********************************************************/

#ifdef VARIANCE

#define DEJITTER
#define DEJITTER_2

#define X_DELTA 		100  
#define X_DELTA_MAX 	220  //max offset of X 
#define Y_DELTA 		125  
#define Y_DELTA_MAX 	300  //max offset of Y
#define XY_DELTA 		(X_DELTA + Y_DELTA)
#define TS_HZ 			15
#define FAST_DELTA		(X_DELTA + Y_DELTA)

#define MAX_SAMPLE 5
#define SAMPLE_CNT 4

enum 
{
	IX_XX=0,
	IX_YY,
	IX_LEN
}TS_INDEX;

//static void t_swap(void **a, void **b);
//static s32 variance(struct input_dev *dev, s16 x, s16 y,  bool penup);
static void variance(	struct TS_SAMPLE *samp, int cnt);



#define NR_SAMPHISTLEN 4

static const unsigned char weight [NR_SAMPHISTLEN - 1][NR_SAMPHISTLEN + 1] =
{
	/* The last element is pow2(SUM(0..3)) */
	{ 5, 3, 0, 0, 3 },	/* When we have 2 samples ... */
	{ 8, 5, 3, 0, 4 },	/* When we have 3 samples ... */
	{ 6, 4, 3, 3, 4 },	/* When we have 4 samples ... */
};

struct dejitter2_info {	
	int delta;
	int x;
	int y;
	int nr;
	int head;
	struct TS_SAMPLE samp[NR_SAMPHISTLEN];
};

#define FAST_THRESHOLD 100
static struct dejitter2_info djt2;
static void average(struct dejitter2_info *djt, struct TS_SAMPLE *samp);
static int dejitter(struct input_dev *dev, struct TS_SAMPLE *_samp, bool penup);


#endif

static int g_poll_delay = POLL_DELAY;
#define ADC_MAIN_CLK	12 	/* 12MHz */
#define ADC_CLK		( 4 * 1000000)	/*  4MHz */

#define IS_PEN_DOWN(pin) (ak98_gpio_getpin(pin) ? 0:1)

#define GET_XP() ( (REG32(AK98_X_COORDINATE) & 0xffc00) >> 10 )
#define GET_XN() ( REG32(AK98_X_COORDINATE) & 0x3ff )
#define GET_YP() ((REG32(AK98_Y_COORDINATE) & 0xffc00) >> 10 )
#define GET_YN() ( REG32(AK98_Y_COORDINATE) & 0x3ff )

static void touch_timer_handler(unsigned long data);
static struct timer_list touch_timer = TIMER_INITIALIZER(touch_timer_handler, 0, 0);



static void report_value(struct input_dev *dev, struct TS_SAMPLE *samp)
{
	PDEBUG("==%4d %4d \n", samp->x, samp->y);
	
	input_report_abs(dev, ABS_X, samp->x);
	input_report_abs(dev, ABS_Y, samp->y);
	input_report_abs(dev, ABS_PRESSURE, 1);
	input_sync(dev);
}


static void set_pin(u32 pin)
{
	//as gpio
	ak98_setpin_as_gpio(pin);
	//input mode
	ak98_gpio_cfgpin(pin, AK98_GPIO_DIR_INPUT);
	//disable pulldown
	ak98_gpio_pulldown(pin, AK98_PULLDOWN_DISABLE);
	//active low
	ak98_gpio_intpol(pin, AK98_GPIO_INT_LOWLEVEL);
	
}

static void ak98_ts_sent_up_event(void )
{
	struct input_dev *input = ts.dev;		
	
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}


static void touch_timer_handler(unsigned long data)
{
	int i;
	struct TS_SAMPLE samp[4];
	
	if (unlikely(!IS_PEN_DOWN(ts.state_pin)))
	{
		PDEBUG("pen up...\n");	
		
		dejitter(ts.dev, &(samp[0]), true);
		ak98_ts_sent_up_event();
		ts.pendown = false;
		enable_irq(ts.irq);
	}
	else 
	{
		if (ts.pendown == false)
		{
			input_report_key(ts.dev, BTN_TOUCH, 1);	
			//drop the first sample
			ak98_power_ADC1(POWER_ON);		
			ak98_power_ts(POWER_ON);
			samp[0].x = GET_XP();
			samp[0].y = GET_YP();
			mdelay(2);
			ak98_power_ts(POWER_OFF);
			ak98_power_ADC1(POWER_OFF);	
			mdelay(2);
			ts.pendown = true;
		}		
		
		for (i=0; i<SAMPLE_CNT; i++)
		{
			if (!IS_PEN_DOWN(ts.state_pin))
				break;
			ak98_power_ADC1(POWER_ON);		
			ak98_power_ts(POWER_ON);
			mdelay(2);
			
			samp[i].x = GET_XP();
			samp[i].y = GET_YP();
		
		

			PDEBUG("x=%d\ty=%d\n", xp,yp);
			//printk("x=%d\ty=%d\t%d\n", xp,yp, g_poll_delay);
			mdelay(2);
			ak98_power_ts(POWER_OFF);
			ak98_power_ADC1(POWER_OFF);	
		}
		if (i == SAMPLE_CNT || (i && ts.first == true))
		{
			if (ts.first == true)
				ts.first = false;
			variance(samp, i);	
			samp[0].y = MAX_10BIT - samp[0].y;
			PDEBUG("x = %4d\t y = %4d\n", samp[0].x, samp[0].y);
			dejitter(ts.dev, &(samp[0]), false);			
		}
		
		mod_timer(&touch_timer, jiffies+ msecs_to_jiffies(g_poll_delay));
	}
	
}

/* 
 * touch screen interrupt handler.
 */
static irqreturn_t ak98ts_irqhandler(int irq, void *handle)
{	
	//PDEBUG("%s(): Entering..., irq=%d \n", __FUNCTION__, irq);		
	if ( likely( IS_PEN_DOWN(ts.state_pin) ) ) 
	{		
		PDEBUG("pen down\n");
				
		g_poll_delay = POLL_DELAY;
		disable_irq_nosync(ts.irq);	
		ts.first = true;
		//power_ts(POWER_ON);		
		mod_timer(&(touch_timer), jiffies + msecs_to_jiffies(PEROID_DELAY));
		
	}

	return IRQ_HANDLED;
}

/*
 * The functions for inserting/removing us as a module.
 */

static int __init ak98_ts_probe(struct platform_device *dev)
{

	struct ak98_ts_mach_info *info;

	info = (struct ak98_ts_mach_info *)dev->dev.platform_data;
	if (!info) {
		printk(KERN_ERR "no platform data for ts\n");
		return -EINVAL;
	}

	/* initialize hardware */
	ak98_init_ADC1(info->sample_rate, info->wait_time);


	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct ak98ts));
	ts.dev = input_allocate_device();
	if (!ts.dev)
		return -ENOMEM;

	ts.dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(ts.dev, ABS_X, 0, MAX_10BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, MAX_10BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, MAX_10BIT, 0, 0);
	
	sprintf(ts.phys, "ts0");

	//ts.dev->private = &ts;
	ts.dev->name = ak98ts_name;
	ts.dev->phys = ts.phys;
	/*ts.dev->id.bustype = BUS_VIRTUAL;
	ts.dev->id.vendor = 0xDEAD; 
	ts.dev->id.product = 0xBEEF;
	ts.dev->id.version = AK98TSVERSION;
*/

	ts.state_pin	= info->irqpin;//info->irqpin; //AK98_GPIO_90
	ts.irq			= info->irq; //IRQ_GPIO_90
	ts.pendown		= false;


	//configure GPIO pin
	set_pin(ts.state_pin);

	PDEBUG("TS IRQ Number=%d, IRQ PIN=%d\n", info->irq, info->irqpin);


	if (request_irq(info->irq, ak98ts_irqhandler, 0, "ak98adc_ts", 0)) {
		printk(KERN_ERR "Could not allocate IRQ %d\n", info->irq);
		return -EIO;
	}

#ifdef DEJITTER_2
	djt2.head = 0;
	djt2.delta = FAST_THRESHOLD;
	djt2.nr = 0;
#endif

	printk(KERN_INFO "%s successfully loaded\n", ak98ts_name);

	/* All went ok, so register to the input system */
	return input_register_device(ts.dev);
}

static int ak98_ts_remove(struct platform_device *dev)
{
	struct ak98_ts_mach_info *info;
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	info = (struct ak98_ts_mach_info *)dev->dev.platform_data;
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

static int ak98_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	return 0;
}

static int ak98_ts_resume(struct platform_device *dev)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);

	return 0;
}
#endif


static struct platform_driver ak98_ts_driver = {
	.driver		= {
		.name	= "ak98adc-ts",
		.owner	= THIS_MODULE,
	},
	.probe		= ak98_ts_probe,
	.remove		= ak98_ts_remove,
#ifdef	CONFIG_PM
	.suspend	= ak98_ts_suspend,
	.resume		= ak98_ts_resume,
#endif
};

static int __init ak98_ts_init(void)
{
	printk("AK98 ADC Touchscreen Driver, (c) 2011 ANYKA\n");

	return platform_driver_register(&ak98_ts_driver);
}

static void __exit ak98_ts_exit(void)
{
	platform_driver_unregister(&ak98_ts_driver);
}


static void variance(	struct TS_SAMPLE *samp, int cnt)
{
	int i, w[SAMPLE_CNT] = {0}, aver_x = 0, aver_y = 0, kmax;
	int sum_x = 0, sum_y = 0;

		
	for (i = 0; i < cnt; i++)
	{
		sum_x += samp[i].x;
		sum_y += samp[i].y;
	}
	aver_x = sum_x / cnt;
	aver_y = sum_y / cnt;

	if (cnt < 3)
	{
		samp[0].x = aver_x;
		samp[0].y = aver_y;
		return;
	}
	
	w[0] = abs(samp[0].x - aver_x) + abs(samp[0].y - aver_y);	
	kmax = 0;
	
	for (i = 1; i < cnt; i++)
	{
		w[i] = abs(samp[i].x - aver_x) + abs(samp[i].y - aver_y);
		if (w[i] > w[kmax])
		{
			kmax = i;
		}
	}
	samp[0].x = (sum_x - samp[kmax].x ) / (cnt - 1);
	samp[0].y = (sum_y - samp[kmax].y ) / (cnt - 1);
	
}

static void average(struct dejitter2_info *djt, struct TS_SAMPLE *samp)
{
	const unsigned char *w;
	int sn = djt->head;
	int i, x = 0, y = 0;
//	unsigned int p = 0;

        w = weight [djt->nr - 2];

	for (i = 0; i < djt->nr; i++) 
	{
		sn = (sn - 1) & (NR_SAMPHISTLEN - 1);		
		x += djt->samp [sn].x * w [i];
		y += djt->samp [sn].y * w [i];
	//	p += djt->samp [sn].p * w [i];
		
	}

	//printk("%4d\t%4d\t%d\n", x, y, djt->nr);
	samp->x = x >> w [NR_SAMPHISTLEN];
	samp->y = y >> w [NR_SAMPHISTLEN];
	//samp->pressure = p >> w [NR_SAMPHISTLEN];

	PDEBUG("DEJITTER2----------------> %d %d %d\n",samp->x, samp->y, samp->pressure);
}

static int dejitter(struct input_dev *dev, struct TS_SAMPLE *_samp, bool penup)
{	
	static struct TS_SAMPLE temp;	

	if (penup == true)
	{				
		djt2.head = 0;
		djt2.nr = 0;;
		return 1;
	}

	/* If the pen moves too fast, reset the backlog. */
	if (djt2.nr)
	{
		int prev = (djt2.head - 1) & (NR_SAMPHISTLEN - 1);
		if (abs(_samp->x - djt2.samp[prev].x) + abs(_samp->y - djt2.samp[prev].y) > djt2.delta)
			djt2.nr = 0;
	}

	djt2.samp[djt2.head] = *_samp;
	djt2.head = (djt2.head + 1) & (NR_SAMPHISTLEN - 1);
	
	if (djt2.nr < NR_SAMPHISTLEN)
		++djt2.nr;
	
	if (djt2.nr == 1)
	{
		temp = *_samp;
	}
	else
	{
		average(&djt2, &temp);
	}		

	report_value(dev, &temp);
	return 1;
}



module_init(ak98_ts_init);
module_exit(ak98_ts_exit);

MODULE_AUTHOR("ANYKA");
MODULE_DESCRIPTION("ak98 ADC touchscreen driver");
MODULE_LICENSE("GPL");


