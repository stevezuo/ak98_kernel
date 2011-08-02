/*
* drivers/input/touchscreen/ar7643_ts.c
* Copyright (C) 2011 ANYKA
* Author: Zhou Wenyong
*
* AR7643 TouchScreen Driver for ak98
*/


#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/delay.h>


//#define AR7643_DEBUG
#define DEJITTER

#define AR7642TSVERSION 0x01


/* definitons of pin  */
#define PIN_DOUT		AK98_GPIO_0 	
#define PIN_CLK			AK98_GPIO_1
#define PIN_CS			AK98_GPIO_2
#define PIN_DIN			AK98_GPIO_3
#define PIN_INT			AK98_GPIO_19
#define AR7643_IRQ		IRQ_GPIO_19

#define POLL_DELAY			15
#define PEROID_DELAY		2

#define	MAX_12BIT			(0xFFF)
#define SAMPLE_CNT		4

/* definitions of control byte */
#define START_BIT		(1<<7)
#define ADDR_X			(1<<4)
#define ADDR_Y			(5<<4)

#define GET_X           0xD0
#define GET_Y			0x90


enum
{
	DOUT_BIT = 0,
	CLK_BIT = 1,
	CS_BIT = 2,
	DIN_BIT = 3,
	INT_BIT = 18
};


#define SET_PIN_HIGH(pin)	ak98_gpio_setpin(pin, AK98_GPIO_HIGH)
#define SET_PIN_LOW(pin)	ak98_gpio_setpin(pin, AK98_GPIO_LOW)

#define READ_PIN(pin)		ak98_gpio_getpin(pin)
#define IS_PEN_DOWN(pin) (ak98_gpio_getpin(pin) ? 0:1)


#undef PDEBUG           /* undef it, just in case */
#ifdef AR7643_DEBUG
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

static char *ar7643ts_name = "AR7643 ADC TouchScreen for AK98";
//static spinlock_t ar7643_lock;


struct AR7643TS_SAMPLE {
	s32		x;
	s32		y;
};

/*
 * Per-touchscreen data.
 */

struct ar7643ts {
	struct input_dev *dev;
	bool pendown;
	bool first;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
};
static void touch_timer_handler(unsigned long data);
static void get_xy(struct AR7643TS_SAMPLE *samp);

static struct timer_list touch_timer = TIMER_INITIALIZER(touch_timer_handler, 0, 0);


static struct ar7643ts ts;



#ifdef DEJITTER

#define NR_SAMPHISTLEN 4

static const unsigned char weight [NR_SAMPHISTLEN - 1][NR_SAMPHISTLEN + 1] =
{
	/* The last element is pow2(SUM(0..3)) */
	{ 5, 3, 0, 0, 3 },	/* When we have 2 samples ... */
	{ 8, 5, 3, 0, 4 },	/* When we have 3 samples ... */
	{ 6, 4, 3, 3, 4 },	/* When we have 4 samples ... */
};

struct dejitter_info {	
	int delta;
	int x;
	int y;
	int nr;
	int head;
	struct AR7643TS_SAMPLE samp[NR_SAMPHISTLEN];
};

#define FAST_THRESHOLD 100
static struct dejitter_info djt;
static void average(struct dejitter_info *djt, struct AR7643TS_SAMPLE *samp);
static int dejitter(struct input_dev *dev, struct AR7643TS_SAMPLE *_samp, bool penup);


#endif


static void report_value(struct input_dev *dev, struct AR7643TS_SAMPLE *samp)
{
	PDEBUG("==%4d %4d \n", samp->x, samp->y);
	
	input_report_abs(dev, ABS_X, samp->x);
	input_report_abs(dev, ABS_Y, samp->y);
	input_report_abs(dev, ABS_PRESSURE, 1);
	input_sync(dev);
}

static void pin_init(void)
{

	ak98_setpin_as_gpio(PIN_DOUT);
	ak98_gpio_pulldown(PIN_DOUT, AK98_PULLDOWN_DISABLE);
	ak98_gpio_cfgpin(PIN_DOUT, AK98_GPIO_DIR_OUTPUT);
	
	ak98_setpin_as_gpio(PIN_CLK);
	ak98_gpio_pullup(PIN_CLK, AK98_PULLUP_DISABLE);
	ak98_gpio_cfgpin(PIN_CLK, AK98_GPIO_DIR_OUTPUT);
	
	ak98_setpin_as_gpio(PIN_CS);
	ak98_gpio_pullup(PIN_CS, AK98_PULLUP_DISABLE);
	ak98_gpio_cfgpin(PIN_CS, AK98_GPIO_DIR_OUTPUT);

		
	ak98_setpin_as_gpio(PIN_DIN);
	ak98_gpio_pullup(PIN_DIN, AK98_PULLUP_DISABLE);
	ak98_gpio_cfgpin(PIN_DIN, AK98_GPIO_DIR_INPUT);

		
	ak98_setpin_as_gpio(PIN_INT);
	ak98_gpio_pullup(PIN_INT, AK98_PULLUP_ENABLE);
	ak98_gpio_pulldown(PIN_INT, AK98_PULLDOWN_DISABLE);
	ak98_gpio_cfgpin(PIN_INT, AK98_GPIO_DIR_INPUT);
	ak98_gpio_intpol(PIN_INT, AK98_GPIO_INT_LOWLEVEL);

}

static void start(void)
{
	SET_PIN_LOW(PIN_CLK);
	SET_PIN_HIGH(PIN_CS);
	SET_PIN_HIGH(PIN_DOUT);
	SET_PIN_HIGH(PIN_CLK);
	SET_PIN_LOW(PIN_CS);
}

//write one byte
static void send_cmd_to_ar7643(unsigned char num) 
{
    unsigned char count=0;

	SET_PIN_LOW(PIN_DOUT);
	SET_PIN_LOW(PIN_CLK);

	//先把数据准备好，即设好电平，再拉高CLK线，通知从备过来取数。
	for(count=0;count<8;count++)
	{
		if((num&0x80)==0x80)   
			SET_PIN_HIGH(PIN_DOUT);
		else 
			SET_PIN_LOW(PIN_DOUT);

		num<<=1;
		SET_PIN_LOW(PIN_CLK);
		SET_PIN_HIGH(PIN_CLK);
	}
    mdelay(1);
}

static int get_pos(int cmd)
{
	int x = 0, count;

	send_cmd_to_ar7643(cmd);
	SET_PIN_HIGH(PIN_CLK);
	SET_PIN_LOW(PIN_CLK);

	// 先产生下降沿，让从设备准备好数据，再读取电平。
    for(count=0;count<12;count++)
    {
        x<<=1;
		SET_PIN_HIGH(PIN_CLK);
        SET_PIN_LOW(PIN_CLK);
        if(READ_PIN(PIN_DIN)) x++;
    }
	
    return (x);
}


static void get_xy(struct AR7643TS_SAMPLE *samp)
{
	//assert(samp);

	start();
	if (ts.first == true);
		mdelay(1);
	samp->x = get_pos(GET_X);
	samp->y = get_pos(GET_Y);

	SET_PIN_HIGH(PIN_CS);	
}



static void ar7643ts_sent_up_event(void )
{
	struct input_dev *input = ts.dev;		
	
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}

/* 
 * touch screen interrupt handler.
 */
static irqreturn_t ar7643ts_irqhandler(int irq, void *handle)
{		
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	if ( likely( IS_PEN_DOWN(PIN_INT) ) ) 
	{		
		PDEBUG("pen down\n");		
		disable_irq_nosync(AR7643_IRQ);		
		ts.first = true;
		mod_timer(&(touch_timer), jiffies + msecs_to_jiffies(PEROID_DELAY));		
	}

	return IRQ_HANDLED;
}

static void variance(	struct AR7643TS_SAMPLE *samp, int cnt)
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

static void touch_timer_handler(unsigned long data)
{		
	struct AR7643TS_SAMPLE samp[5];
	int  i;
	
	if (unlikely(!IS_PEN_DOWN(PIN_INT)))
	{
		PDEBUG("pen up...\n");	
		dejitter(ts.dev, &(samp[0]), true);
		ar7643ts_sent_up_event();
		ts.pendown = false;
		enable_irq(AR7643_IRQ);
	}
	else 
	{
		if (ts.pendown == false)
		{
			input_report_key(ts.dev, BTN_TOUCH, 1);	
			
			ts.pendown = true;
		}		
		
	
		for(i=0;i<SAMPLE_CNT;i++)
		{
			if (!IS_PEN_DOWN(PIN_INT))
				break;
			get_xy(&(samp[i]));				
		}
			
		if (i == SAMPLE_CNT || (i && ts.first == true))
		{
			if (ts.first == true)
				ts.first = false;
			variance(samp, i);	
			PDEBUG("x = %4d\t y = %4d\n", samp[0].x, samp[0].y);
			dejitter(ts.dev, &(samp[0]), false);			
		}


		mod_timer(&touch_timer, jiffies+ msecs_to_jiffies(POLL_DELAY));
	}
	
}

/*
 * The functions for inserting/removing us as a module.
 */

static int __init ar7643_ts_probe(struct platform_device *dev)
{

	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	/* initialize hardware */
	pin_init();

	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct ar7643ts));
	ts.dev = input_allocate_device();
	if (!ts.dev)
		return -ENOMEM;
	
	ts.dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(ts.dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);
	
	sprintf(ts.phys, "ar7643ts0");
	
	ts.dev->name = ar7643ts_name;
	ts.dev->phys = ts.phys;
	
	PDEBUG("0x%x\n", ts.dev->dev.class);
	PDEBUG("%s\n", ts.dev->dev.class->name);

	
	ts.pendown	= false;

	if (request_irq(AR7643_IRQ, ar7643ts_irqhandler, 0, "ar7643adc_ts", 0)) {
		printk(KERN_ERR "Could not allocate IRQ %d\n", AR7643_IRQ);
		return -EIO;
	}

	#ifdef DEJITTER
	djt.head = 0;
	djt.delta = FAST_THRESHOLD;
	djt.nr = 0;
	#endif

	printk(KERN_INFO "%s successfully loaded\n", ar7643ts_name);

	/* All went ok, so register to the input system */
	return input_register_device(ts.dev);
}

static int ar7643_ts_remove(struct platform_device *dev)
{
	
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	

	disable_irq(AR7643_IRQ);
	free_irq(AR7643_IRQ, ts.dev);


	input_unregister_device(ts.dev);

	return 0;
}

#ifdef DEJITTER
static void average(struct dejitter_info *djt, struct AR7643TS_SAMPLE *samp)
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

	//PDEBUG("DEJITTER----------------> %d %d %d\n",samp->x, samp->y, 0);
}

static int dejitter(struct input_dev *dev, struct AR7643TS_SAMPLE *_samp, bool penup)
{	
	static struct AR7643TS_SAMPLE temp;	

	if (penup == true)
	{				
		djt.head = 0;
		djt.nr = 0;;
		return 1;
	}

	/* If the pen moves too fast, reset the backlog. */
	if (djt.nr)
	{
		int prev = (djt.head - 1) & (NR_SAMPHISTLEN - 1);
		if (abs(_samp->x - djt.samp[prev].x) + abs(_samp->y - djt.samp[prev].y) > djt.delta)
			djt.nr = 0;
	}

	djt.samp[djt.head] = *_samp;
	djt.head = (djt.head + 1) & (NR_SAMPHISTLEN - 1);
	
	if (djt.nr < NR_SAMPHISTLEN)
		++djt.nr;
	
	if (djt.nr == 1)
	{
		temp = *_samp;
	}
	else
	{
		average(&djt, &temp);
	}		

	report_value(dev, &temp);
	return 1;
}

#endif

#ifdef CONFIG_PM

static int ar7643_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	return 0;
}

static int ar7643_ts_resume(struct platform_device *dev)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);

	return 0;
}
#endif

static struct platform_driver ar7643_ts_driver = {
	.driver		= {
		.name	= "ar7643-ts",
		.owner	= THIS_MODULE,
	},
	.probe		= ar7643_ts_probe,
	.remove		= ar7643_ts_remove,
#ifdef	CONFIG_PM
	.suspend	= ar7643_ts_suspend,
	.resume		= ar7643_ts_resume,
#endif
};

static int __init ar7643_ts_init(void)
{
	printk("AR7643 ADC Touchscreen Driver for AK98, (c) 2011 ANYKA\n");

	return platform_driver_register(&ar7643_ts_driver);
}

static void __exit ar7643_ts_exit(void)
{
	platform_driver_unregister(&ar7643_ts_driver);
}


module_init(ar7643_ts_init);
module_exit(ar7643_ts_exit);

MODULE_AUTHOR("Anyka,Ltd");
MODULE_DESCRIPTION("AR7643 TouchScreen Driver");
MODULE_LICENSE("GPL");














