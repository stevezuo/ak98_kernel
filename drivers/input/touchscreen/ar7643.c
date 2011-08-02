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
#include <linux/spi/spi.h>


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

#define POLL_DELAY			5
#define PEROID_DELAY		1

#define	MAX_12BIT			(0xFFF)
#define SAMPLE_CNT		4

/* definitions of control byte */
#define START_BIT		(1<<7)
#define ADDR_X			(1<<4)
#define ADDR_Y			(5<<4)

#define GET_X           0xD0
#define GET_Y			0x90
#define CS_KEEP_LOW     1

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
static struct workqueue_struct *ar7643_wq;


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

static char *ar7643_name = "AR7643 SPI TouchScreen for AK98";
//static spinlock_t ar7643_lock;
static int ar7643_send_byte(u8 command);


struct AR7643_SAMPLE {
	s32		x;
	s32		y;
};

/*
 * Per-touchscreen data.
 */

struct ar7643 {
	struct input_dev *dev;
	bool pendown;
	bool first;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
	u32 speed_hz;
	struct spi_device *spi;
	struct delayed_work	work;
};
static void ar7643_work(struct work_struct *work);//unsigned long data);
static void get_xy(struct AR7643_SAMPLE *samp);



static struct ar7643 ts;



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
	struct AR7643_SAMPLE samp[NR_SAMPHISTLEN];
};

#define FAST_THRESHOLD 100
static struct dejitter_info djt;
static void average(struct dejitter_info *djt, struct AR7643_SAMPLE *samp);
static int dejitter(struct input_dev *dev, struct AR7643_SAMPLE *_samp, bool penup);


#endif


static void report_value(struct input_dev *dev, struct AR7643_SAMPLE *samp)
{
	PDEBUG("==%4d %4d \n", samp->x, samp->y);
	
	input_report_abs(dev, ABS_X, samp->x);
	input_report_abs(dev, ABS_Y, samp->y);
	input_report_abs(dev, ABS_PRESSURE, 1);
	input_sync(dev);
}

static void pin_init(void)
{
	
	ak98_setpin_as_gpio(PIN_INT);
	ak98_gpio_pullup(PIN_INT, AK98_PULLUP_ENABLE);
	ak98_gpio_pulldown(PIN_INT, AK98_PULLDOWN_DISABLE);
	ak98_gpio_cfgpin(PIN_INT, AK98_GPIO_DIR_INPUT);
	ak98_gpio_intpol(PIN_INT, AK98_GPIO_INT_LOWLEVEL);

}

static int ar7643_read_12bit(u8 comand, int *v)
{
	struct spi_message m;
	struct spi_transfer	xfer;
	int status;
	int a=0;
	
	u8 cmd[3] = {0};
	u8 buff[3] = {0};
	cmd[0] = comand;
	
	spi_message_init(&m);	

	xfer.tx_buf = cmd;
	xfer.rx_buf = buff;
	xfer.len = sizeof(cmd);
	xfer.bits_per_word = 8;
	xfer.speed_hz = ts.speed_hz;
	xfer.cs_change = CS_KEEP_LOW ? 0:1; // 1: the #CS signal is LOW until all the data transmission has been finished
	                                    // 0: the #CS signal is HIGH after a transmission of 8 bits data
	                         
	spi_message_add_tail(&xfer, &m);

	status = spi_sync(ts.spi, &m);

	a = buff[1]&0x7f;
	a = ((a<<5) | ((buff[2]>>3)&0x1f));
	
	*v = a;	
	return status;
}

static int get_pos(int *x, int *y)
{
	//int  count;

	ar7643_read_12bit(GET_X, x);
	ar7643_read_12bit(GET_Y, y);

	//printk("x: %d  y: %d\n", *x, *y);
	
    return 0;
}


static void get_xy(struct AR7643_SAMPLE *samp)
{
	if (ts.first == true);
	{
		udelay(100);
		get_pos(&(samp->x), &(samp->y));
		ts.first = false;
	}
		
	get_pos(&(samp->x), &(samp->y));
	
}



static void ar7643_sent_up_event(void )
{
	struct input_dev *input = ts.dev;		
	
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}

/* 
 * touch screen interrupt handler.
 */
static irqreturn_t ar7643_irqhandler(int irq, void *handle)
{		
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	if ( likely( IS_PEN_DOWN(PIN_INT) ) ) 
	{		
		PDEBUG("pen down\n");		
		disable_irq_nosync(AR7643_IRQ);		
		ts.first = true;
		queue_delayed_work(ar7643_wq, &(ts.work),
				      msecs_to_jiffies(PEROID_DELAY));
		//mod_timer(&(touch_timer), jiffies + msecs_to_jiffies(PEROID_DELAY));		
	}

	return IRQ_HANDLED;
}

static void variance(	struct AR7643_SAMPLE *samp, int cnt)
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

static void ar7643_work(struct work_struct *work)//unsigned long data)
{		
	struct AR7643_SAMPLE samp[5];
	int  i;
	//static int flg = 0;
	
	if (unlikely(!IS_PEN_DOWN(PIN_INT)))
	{
		PDEBUG("pen up...\n");	
		dejitter(ts.dev, &(samp[0]), true);
		ar7643_sent_up_event();
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

		queue_delayed_work(ar7643_wq, &(ts.work),
				      msecs_to_jiffies(POLL_DELAY));
		//mod_timer(&touch_timer, jiffies+ msecs_to_jiffies(POLL_DELAY));
	}
	
}


static int ar7643_send_byte(u8 command)
{
	struct spi_message m;
	struct spi_transfer	xfer;
	int status;

	u8 buff = command;

	
	spi_message_init(&m);

	xfer.tx_buf = &buff;
	xfer.rx_buf = NULL;
	xfer.len = sizeof(buff);
	xfer.bits_per_word = 8;
	xfer.speed_hz = ts.speed_hz;

	spi_message_add_tail(&xfer, &m);

	status = spi_sync(ts.spi, &m);

	
	return status;
}

/*
 * The functions for inserting/removing us as a module.
 */

static int __init ar7643_probe(struct spi_device *spi)
{	
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	/* initialize hardware */
	pin_init();

	/* Initialise input stuff */
	memset(&ts, 0, sizeof(struct ar7643));
	ts.dev = input_allocate_device();
	if (!ts.dev)
		return -ENOMEM;
	
	ts.dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts.dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(ts.dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(ts.dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);
	
	sprintf(ts.phys, "ar7643-spi");
	
	ts.dev->name = ar7643_name;
	ts.dev->phys = ts.phys;
	ts.speed_hz = spi->max_speed_hz;	
	ts.pendown	= false;
	ts.spi = spi;

	INIT_DELAYED_WORK(&(ts.work), ar7643_work);	

	if (request_irq(AR7643_IRQ, ar7643_irqhandler, 0, "ar7643_spi", 0)) {
		printk(KERN_ERR "Could not allocate IRQ %d\n", AR7643_IRQ);
		return -EIO;
	}

	#ifdef DEJITTER
	djt.head = 0;
	djt.delta = FAST_THRESHOLD;
	djt.nr = 0;
	#endif

	//power down and enable PENIRQ	
	ar7643_send_byte(0x80);
	
	printk(KERN_INFO "%s successfully loaded\n", ar7643_name);

	/* All went ok, so register to the input system */
	return input_register_device(ts.dev);
}

static int ar7643_remove(struct spi_device *spi)
{
	
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	

	disable_irq(AR7643_IRQ);
	free_irq(AR7643_IRQ, ts.dev);


	input_unregister_device(ts.dev);

	return 0;
}

#ifdef DEJITTER
static void average(struct dejitter_info *djt, struct AR7643_SAMPLE *samp)
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

static int dejitter(struct input_dev *dev, struct AR7643_SAMPLE *_samp, bool penup)
{	
	static struct AR7643_SAMPLE temp;	

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

static int ar7643_suspend(struct spi_device *spi, pm_message_t state)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	return 0;
}

static int ar7643_resume(struct spi_device *spi)
{
	PDEBUG("%s(): Entering...\n", __FUNCTION__);

	return 0;
}
#endif

static struct spi_driver ar7643_driver = {
	.driver		= {
		.name	= "ar7643-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ar7643_probe,
	.remove		= ar7643_remove,
#ifdef	CONFIG_PM
	.suspend	= ar7643_suspend,
	.resume		= ar7643_resume,
#endif
};

static int __init ar7643_init(void)
{
	printk("AR7643 SPI Touchscreen Driver for AK98, (c) 2011 ANYKA\n");
	ar7643_wq = create_singlethread_workqueue("ar7643_wq");
	if (!ar7643_wq)
		return -ENOMEM;
	return spi_register_driver(&ar7643_driver);
}

static void __exit ar7643_exit(void)
{	
	if (ar7643_wq)
			destroy_workqueue(ar7643_wq);

	spi_unregister_driver(&ar7643_driver);
}


module_init(ar7643_init);
module_exit(ar7643_exit);

MODULE_AUTHOR("Anyka,Ltd");
MODULE_DESCRIPTION("AR7643 SPI TouchScreen Driver");
MODULE_LICENSE("GPL");














