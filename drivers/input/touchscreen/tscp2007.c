/*
 * drivers/input/touchscreen/cp2007_ts.c
 *
 * Copyright (c) 2010 Anyka, Ltd.
 *
 * Using code from:
 *  -tsc2007.c
 *   Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/cp2007.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/mutex.h>


#define TS_POLL_DELAY			1 /* ms delay between samples */
#define TS_POLL_PERIOD			10 /* ms delay between samples */
#define MIN_PRESSURE 100
#define MAX_PRESSURE 700

#define DEF_PRESSURE 1

#define VARIANCE
static struct workqueue_struct *tscp2007_wq;

struct TS_SAMPLE {
	s32		x;
	s32		y;
	u32	pressure;
};

static void report_value(struct input_dev *dev, struct TS_SAMPLE *samp);

#ifdef VARIANCE

#define P_DELTA			249
/* 2-23 added by wenyong */
#define X_DELTA 		234  
#define X_DELTA_MAX 	500  //max offset of X 
#define Y_DELTA 		330 
#define Y_DELTA_MAX 	650  //max offset of Y
#define XY_DELTA 		(X_DELTA + Y_DELTA)
#define TS_HZ 			15
#define FAST_DELTA		(X_DELTA + Y_DELTA)


//static int X_WEIGHT[5] = {0, 22, 44, 66, 88};
//static int Y_WEIGHT[5] = {0, 26, 52, 78, 104};
#define NEXT_INDEX(i, size) (i==size - 1 ? 0:i+1)
#define MAX_SAMPLE 5
enum 
{
	IX_XX=0,
	IX_YY,
	IX_PP,
	IX_LEN
};

static void t_swap(void **a, void **b);
static s32 variance(struct input_dev *dev, s16 x, s16 y, u32 pressure, bool penup);

static int dejitter(struct input_dev *dev, struct TS_SAMPLE *_samp, bool penup);


#endif
//#define TS_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef TS_DEBUG
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


#define CP2007_MEASURE_TEMP0	(0x0 << 4)
#define CP2007_MEASURE_AUX		(0x2 << 4)
#define CP2007_MEASURE_TEMP1	(0x4 << 4)
#define CP2007_ACTIVATE_XN		(0x8 << 4)
#define CP2007_ACTIVATE_YN		(0x9 << 4)
#define CP2007_ACTIVATE_YP_XN	(0xa << 4)
#define CP2007_SETUP			(0xb << 4)
#define CP2007_MEASURE_X		(0xc << 4)
#define CP2007_MEASURE_Y		(0xd << 4)
#define CP2007_MEASURE_Z1		(0xe << 4)
#define CP2007_MEASURE_Z2		(0xf << 4)


#define CP2007_POWER_OFF_IRQ_EN	(0x0 << 2)
#define CP2007_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define CP2007_ADC_OFF_IRQ_EN		(0x2 << 2)
#define CP2007_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define CP2007_12BIT		(0x0 << 1)
#define CP2007_8BIT			(0x1 << 1)

#define	MAX_12BIT			((1 << 12) - 1)

#define ADC_ON_12BIT	(CP2007_12BIT | CP2007_ADC_ON_IRQ_DIS0)


#define ADC_ON_AND_ACTIVE (ADC_ON_12BIT | CP2007_ACTIVATE_YN | CP2007_ACTIVATE_XN | CP2007_ACTIVATE_YP_XN )
#define READ_Y		(CP2007_12BIT | CP2007_MEASURE_Y )
#define READ_Z1		(CP2007_12BIT | CP2007_MEASURE_Z1)
#define READ_Z2		(CP2007_12BIT | CP2007_MEASURE_Z2)
#define READ_X		(CP2007_12BIT | CP2007_MEASURE_X )
#define PWRDOWN		(CP2007_12BIT | CP2007_POWER_OFF_IRQ_EN)
#define ADCOFF 		(CP2007_12BIT | CP2007_ADC_OFF_IRQ_EN)


struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct cp2007_ts {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;
	struct timer_list ts_timer;

	struct i2c_client	*client;

	u16			x_plate_ohms;

	bool		pendown;
	int			irq;
	unsigned int intpin;
	char		origin_pos;

	int			(*is_pen_down)(unsigned int pin);
	void		(*clear_penirq)(void);
};

struct mutex g_cp2007_mutex;

static inline int cp2007_ts_xfer(struct cp2007_ts *tsc, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(tsc->client, cmd);
	if (data < 0) {
		dev_err(&tsc->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data) >> 4;

	dev_dbg(&tsc->client->dev, "data: 0x%x, val: 0x%x\n", data, val);

	return val;
}

static void cp2007_ts_read_values(struct cp2007_ts *tsc, struct ts_event *tc)
{
	mutex_lock(&g_cp2007_mutex);
	cp2007_ts_xfer(tsc, ADC_ON_AND_ACTIVE);
	/* y- still on; turn on only y+ (and ADC) */
	tc->y = cp2007_ts_xfer(tsc, READ_Y);

	/* turn y- off, x+ on, then leave in lowpower */
	tc->x = cp2007_ts_xfer(tsc, READ_X);

	/* turn y+ off, x- on; we'll use formula #1 */
	tc->z1 = cp2007_ts_xfer(tsc, READ_Z1);
	tc->z2 = cp2007_ts_xfer(tsc, READ_Z2);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	cp2007_ts_xfer(tsc, PWRDOWN);
	mutex_unlock(&g_cp2007_mutex);
	
}

static u32 cp2007_ts_calculate_pressure(struct cp2007_ts *tsc, struct ts_event *tc)
{
	u32 rt = 0;

	/* range filtering */
	if (tc->x == MAX_12BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= tsc->x_plate_ohms;
		rt /= tc->z1;
		//rt = (rt + 2047) >> 12;
		rt = rt >> 12;
	}

	return rt;
}

static void cp2007_ts_send_up_event(struct cp2007_ts *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->client->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}

static void cp2007_ts_work(struct work_struct *work)
{
	struct cp2007_ts *ts = container_of(to_delayed_work(work), struct cp2007_ts, work);
	struct ts_event tc;
	u32 rt;

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */

	if (ts->is_pen_down) {	
		if (unlikely(!ts->is_pen_down(ts->intpin))) {
			#ifdef VARIANCE
			variance(ts->input,0, 0,0, true);
			#endif
			PDEBUG("up event\n");
			cp2007_ts_send_up_event(ts);
			ts->pendown = false;		
			goto out;
		}

		dev_dbg(&ts->client->dev, "pen is still down\n");
	}

		
	cp2007_ts_read_values(ts, &tc);

	rt = cp2007_ts_calculate_pressure(ts, &tc);
	if (ts->origin_pos == ORIGIN_TOPLEFT)
		tc.y = MAX_12BIT - tc.y;
	
	PDEBUG("%4d %4d %4d\n", tc.x, tc.y, rt);
	if ( (rt < MIN_PRESSURE) || (rt > MAX_PRESSURE)) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */		
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		PDEBUG("Ignored pressure %d\n", rt);
		goto out;
	}

	if (rt) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			dev_dbg(&ts->client->dev, "DOWN\n");

			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
		}

		#ifdef VARIANCE
		variance(input, tc.x, tc.y, rt, false);
		#else
		struct TS_SAMPLE samp = {tc.x,tc.y,rt};
		report_value(input, &samp);
/*		input_report_abs(input, ABS_X, tc.x);
		input_report_abs(input, ABS_Y, tc.y);
		input_report_abs(input, ABS_PRESSURE, rt);

		input_sync(input);
		*/
		#endif

		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			tc.x, tc.y, rt);

	} else if (!ts->is_pen_down && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		#ifdef VARIANCE
			variance(ts->input,tc.x, tc.y,0, true);
		#endif
		PDEBUG("  up event\n");
		cp2007_ts_send_up_event(ts);
		ts->pendown = false;
	}

 out:
	if (ts->pendown) {		
		queue_delayed_work(tscp2007_wq, &ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	}
	else
	{
		enable_irq(ts->irq);
	}
}

static irqreturn_t cp2007_ts_irq(int irq, void *handle)
{
	struct cp2007_ts *ts = handle;

	// if ts->is_pen_down is NULL, likely(ts->is_pen_down() will never be excuted
	if (!ts->is_pen_down || likely(ts->is_pen_down(ts->intpin))) {		
		disable_irq_nosync(ts->irq);
		PDEBUG("pen down\n");
		
			queue_delayed_work(tscp2007_wq, &ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void cp2007_ts_free_irq(struct cp2007_ts *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

//2-23 added by wenyong

static void report_value(struct input_dev *dev, struct TS_SAMPLE *samp)
{
	PDEBUG("==%4d %4d %4d\n", samp->x, samp->y, samp->pressure);
	input_report_abs(dev, ABS_X, samp->x);
	input_report_abs(dev, ABS_Y, samp->y);
	input_report_abs(dev, ABS_PRESSURE, DEF_PRESSURE);//samp->pressure);
	input_sync(dev);
}


#ifdef VARIANCE
static void t_swap(void **a, void **b)
{
	void *c;
	c = *a;
	*a = *b;
	*b = c;
}

static s32 variance(struct input_dev *dev, s16 x, s16 y, u32 pressure, bool penup)
{
	static s32 count=0, pre_p;
	static u8 k1, k2, k3;
	static struct TS_SAMPLE samp[2];
	static struct TS_SAMPLE *sampA = &(samp[0]), *sampB = &(samp[1]);
	static s32 time_delta, pre_jiffies;
	PDEBUG("----: %d %d %d %d\n", count, x, y, pressure);

	time_delta = (jiffies - pre_jiffies) * TS_HZ;
	pre_jiffies = jiffies;
	if (penup)
	{			
		goto AcceptSample;
	}
	
	if (count && abs(pressure - pre_p) > P_DELTA)
	{
		//drop the first
		if (count == 1)
		{
			sampA->x = x; sampA->y = y;
			pre_p = pressure;
		}
		//else drop the current
		
		return 0;
	}
	
	pre_p = pressure;

	switch (count)
	{
	case 0:
		sampA->x = x; sampA->y = y; //sampA->pressure = pressure;	
		count = 1;
		return 0;
	case 1:
		if ( (abs(x - sampA->x) + abs(y - sampA->y)) < (XY_DELTA + time_delta))
		{
			sampB->x = x; sampB->y = y; //sampB->pressure = pressure;
			goto AcceptSample;
		}
		else
		{
			sampB->x = x; sampB->y = y; //sampB->pressure = pressure;
			count++;
			//PDEBUG("dur: %d\n", time_delta);
			return 0;
		}
	case 2:
		if ( (abs(x - sampB->x) + abs(y - sampB->y)) < (XY_DELTA + time_delta))
		{
			sampA->x = x; sampA->y = y; //sampA->pressure = pressure;
			t_swap((void *)&sampA, (void *)&sampB);
			count = 1;
			PDEBUG("Drop A... %d\n", time_delta);
			//goto AcceptSample;
		}
		else if ( (abs(x - sampA->x) + abs(y - sampA->y)) < (XY_DELTA + time_delta))
		{
			sampB->x = x; sampB->y = y; //sampB->pressure = pressure;
			count = 1;
			PDEBUG("Drop B... %d\n", time_delta);
			//goto AcceptSample ;			
		}
		
		else
		{
			k1 = abs(sampB->x - sampA->x) > X_DELTA_MAX || abs(sampB->y - sampA->y) > Y_DELTA_MAX;
			k2 = abs(x - sampB->x) > X_DELTA_MAX || abs(y - sampB->y) > Y_DELTA_MAX;			
			k3 = abs(x - sampA->x) > X_DELTA_MAX || abs(y - sampA->y) > Y_DELTA_MAX;
			if ( (k1 == 0) && (k2 == 0) && (k3 == 0) )
			{
			//move fast
				goto AcceptSample;
			}
			else if ( (k1 == 1) && (k2 == 1) && (k3 == 1) )
			{
				count = 1;
				sampA->x = x; sampA->y = y; //sampA->pressure = pressure;
				PDEBUG("  Drop A,B ...\n");
				return 0;
			}			
			else if ( k2 == 0)
			{
				sampA->x = x; sampA->y = y; //sampA->pressure = pressure;
				t_swap((void *)&sampA, (void *)&sampB);
				PDEBUG("  Drop A...\n");
				return 0;
			}
			else if ( k3 == 0)
			{
				sampB->x = x; sampB->y = y; //sampB->pressure = pressure;
				PDEBUG("  Drop B...\n");
				return 0;
			}
			else if (k1 == 0)
			{
				PDEBUG ("  Drop C...\n");
				return 0;
			}
			//else move fast
		}
		goto AcceptSample;
		
	}
AcceptSample:
	if (count)
	{	

		dejitter(dev, sampA, false);
		if (count == 2)
		{
			dejitter(dev, sampB, false);
			sampA->x = x; sampA->y = y; //sampA->pressure = pressure;
		}
		else
		{		
			t_swap((void *)&sampA, (void *)&sampB);
		}
		count=1;		
	}
	if (penup)
	{
		count = 0;
		dejitter(dev, sampA, true);
	}
	
	return 1;
}

static void average(struct TS_SAMPLE *samp[5], int *sum, int count, struct TS_SAMPLE *temp)
{
	int w[MAX_SAMPLE], kmax, i, aver;
	
	temp->x = sum[IX_XX]/count;
	temp->y = sum[IX_YY]/count;
	//temp->pressure = sum[IX_PP]/count;	
	
	if (count > 2)
	{		
		aver = temp->x;
		w[0] = abs(samp[0]->x - aver);
		kmax = 0;
		for (i=1; i<count; i++)
		{
			w[i] = abs(samp[i]->x - aver);
			if (w[i] > w[kmax])
				kmax = i;
		}
		
		temp->x = (sum[IX_XX] - samp[kmax]->x) / (count - 1);
		//temp->pressure = (sum[IX_PP] - samp[kmax]->pressure) / (count - 1);		
	}
	if (count > 2)
	{
		
		aver = temp->y;
		w[0] = abs(samp[0]->y - aver);
		kmax = 0;
		for (i=1; i<count; i++)
		{
			w[i] = abs(samp[i]->y - aver);
			if (w[i] > w[kmax])
				kmax = i;
		}
		
		temp->y = (sum[IX_YY] - samp[kmax]->y) / (count - 1);
		//temp->pressure = (sum[IX_PP] - samp[kmax]->pressure) / (count - 1);		
	}
}


static int dejitter(struct input_dev *dev, struct TS_SAMPLE *_samp, bool penup)
{
	static int count=0, i, sum[IX_LEN];	
	static bool flush = false, first = true;
	static struct TS_SAMPLE temp, *tp;
	static struct TS_SAMPLE sampP[MAX_SAMPLE];
	static struct TS_SAMPLE *samp[5] = {&(sampP[0]), &(sampP[1]), &(sampP[2]), &(sampP[3]), &(sampP[4])};
	
	PDEBUG("de: %d %d %d %d %d\n", count, _samp->x, _samp->y, _samp->pressure, penup);
	if ( penup == true)
	{		

		if (count > 3)
		{
			average(samp, sum, count, &temp);
			
			report_value(dev, &temp);				
			count=0;		
			first = true;
			return 1;			
		}	
		
		count = 0;
		first = true;
		return 0;
	}

	//noise often happen when pendown, so the first report sample is given special care
	if ( count>3 && (abs(_samp->x - samp[count-1]->x) + abs(_samp->y - samp[count-1]->y)) > FAST_DELTA)
	{
		PDEBUG("flush\n");
		flush = true;
		goto Report;
	}
	*(samp[count]) = *_samp;
	count++;
	
	switch (count)
	{
	case 1:
		sum[IX_XX] = _samp->x;
		sum[IX_YY] = _samp->y;
		//sum[IX_PP] = _samp->pressure;
		return 0;
	case 2:	
		sum[IX_XX] += _samp->x;
		sum[IX_YY] += _samp->y;
		//sum[IX_PP] += _samp->pressure;
		return 0;
	case 3:
		sum[IX_XX] += _samp->x;
		sum[IX_YY] += _samp->y;
	//	sum[IX_PP] += _samp->pressure;
		return 0;

	case 4:
	case 5:		
		sum[IX_XX] += _samp->x;
		sum[IX_YY] += _samp->y;
	//	sum[IX_PP] += _samp->pressure;		


Report:
		temp.x = sum[IX_XX]/count;
		temp.y = sum[IX_YY]/count;
	//	temp.pressure = sum[IX_PP]/count;

		if (count > 2)
		{
			average(samp, sum, count, &temp);
		}
		//move fast, flush the history
		if (flush == true)
		{
			flush = false;			
			*(samp[0]) = *_samp;
			sum[IX_XX] = _samp->x;
			sum[IX_YY] = _samp->y;
		//	sum[IX_PP] =  _samp->pressure;
			count = 1;
		}
		if (count == 5)
		{
			tp = samp[0];
			sum[IX_XX] -= samp[0]->x;
			sum[IX_YY] -= samp[0]->y;
		//	sum[IX_PP] -= samp[0]->pressure;
			for (i=0; i<4; i++)
				samp[i] = samp[i+1];
			samp[4] = tp;
			count = 4;			
		}		
	}
	
	report_value(dev, &temp);	
	return 1;		
}

#endif

static int __devinit cp2007_ts_probe(struct i2c_client *client, 
								const struct i2c_device_id *id)
{
	struct cp2007_ts *ts;
	struct cp2007_ts_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct cp2007_ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	
	INIT_DELAYED_WORK(&ts->work, cp2007_ts_work);	

	ts->origin_pos		= pdata->origin_pos;
	ts->x_plate_ohms	= pdata->x_plate_ohms;
	ts->is_pen_down		= pdata->is_pen_down;
	ts->clear_penirq	= pdata->clear_penirq;
	ts->intpin			= pdata->intpin_info.pin;

	ts->pendown = false;

	mutex_init(&g_cp2007_mutex);
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));

	input_dev->name = client->name;// "CP2007 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	if (pdata->init_ts_hw)
		pdata->init_ts_hw(&(pdata->intpin_info));

	err = request_irq(ts->irq, cp2007_ts_irq, 0, client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings - power down ADC and enable PENIRQ */
	err = cp2007_ts_xfer(ts, PWRDOWN);
	if (err < 0)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	return 0;

 err_free_irq:
	cp2007_ts_free_irq(ts);
	if (pdata->exit_ts_hw)
		pdata->exit_ts_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit cp2007_ts_remove(struct i2c_client *client)
{
	struct cp2007_ts	*ts = i2c_get_clientdata(client);
	struct cp2007_ts_platform_data *pdata = client->dev.platform_data;

	
	cp2007_ts_free_irq(ts);
	del_timer_sync(&(ts->ts_timer));
	if (pdata->exit_ts_hw)
		pdata->exit_ts_hw();

	input_unregister_device(ts->input);
	kfree(ts);
	mutex_destroy(&g_cp2007_mutex);
	return 0;
}

static struct i2c_device_id cp2007_ts_idtable[] = {
	{ "cp2007_ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cp2007_ts_idtable);


#ifdef CONFIG_PM
int cp2007_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cp2007_ts	*ts = i2c_get_clientdata(client);
	mutex_lock(&g_cp2007_mutex);

	//low consumption, disable PENIRQ
	cp2007_ts_xfer(ts, (CP2007_12BIT | CP2007_ADC_ON_IRQ_DIS0));
	
	return 0;
}
int cp2007_ts_resume(struct i2c_client *client)
{
	struct cp2007_ts	*ts = i2c_get_clientdata(client);
	//enable PENIRQ
	cp2007_ts_xfer(ts, PWRDOWN);
	mutex_unlock(&g_cp2007_mutex);
	return 0;
}
#endif

static struct i2c_driver cp2007_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "cp2007_ts"
	},
	.id_table	= cp2007_ts_idtable,
	.probe		= cp2007_ts_probe,
	.remove		= __devexit_p(cp2007_ts_remove),
	#ifdef CONFIG_PM
	.suspend 	= cp2007_ts_suspend,
	.resume		= cp2007_ts_resume,
	#else
	.suspend	= NULL,
	.resume		= NULL,
	#endif
};

static int __init cp2007_ts_init(void)
{
	tscp2007_wq = create_singlethread_workqueue("tscp2007_wq");
	if (!tscp2007_wq)
		return -ENOMEM;
	return i2c_add_driver(&cp2007_ts_driver);
}

static void __exit cp2007_ts_exit(void)
{
	if (tscp2007_wq)
			destroy_workqueue(tscp2007_wq);

	i2c_del_driver(&cp2007_ts_driver);
}

module_init(cp2007_ts_init);
module_exit(cp2007_ts_exit);

MODULE_AUTHOR("Anyka,Ltd");
MODULE_DESCRIPTION("CP2007 TouchScreen Driver");
MODULE_LICENSE("GPL");
