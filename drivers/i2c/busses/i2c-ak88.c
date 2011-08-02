/* 
 * linux/drivers/i2c/busses/i2c-ak880x.c
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
*/

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/i2c.h>
#include <mach/gpio.h>

#define I2CSCLPIN AK88_DGPIO_0
#define I2CSDAPIN AK88_DGPIO_1

static unsigned char IICDS = 0;

/* i2c controller state */

enum ak880x_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct ak880x_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	unsigned int		tx_setup;

	enum ak880x_i2c_state	state;

	void __iomem		*regs;
	struct clk		*clk;
	struct device		*dev;
	struct resource		*irq;
	struct resource		*ioarea;
	struct i2c_adapter	adap;
};

/* default platform data to use if not supplied in the platform_device
*/

static struct ak880x_platform_i2c ak880x_i2c_default_platform = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.bus_freq	= 100*1000,
	.max_freq	= 400*1000,
//	.sda_delay	= S3C2410_IICLC_SDA_DELAY5 | S3C2410_IICLC_FILTER_ON,
};

/* ak880x_i2c_get_platformdata
 *
 * get the platform data associated with the given device, or return
 * the default if there is none
 */

static inline struct ak880x_platform_i2c *ak880x_i2c_get_platformdata(struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct ak880x_platform_i2c *)dev->platform_data;

	return &ak880x_i2c_default_platform;
}

static inline void set_scl(int high)
{
	ak880x_gpio_setpin(I2CSCLPIN, high);
}
static inline void set_sda(int high)
{
	ak880x_gpio_setpin(I2CSDAPIN, high);
}
static inline void scl_in(void)
{
	ak880x_gpio_cfgpin(I2CSCLPIN, AK88_GPIO_IN_0);
}
static inline void scl_out(void)
{
	ak880x_gpio_cfgpin(I2CSCLPIN, AK88_GPIO_OUT_0);
}
static inline void sda_in(void)
{
	ak880x_gpio_cfgpin(I2CSDAPIN, AK88_GPIO_IN_0);
}
static inline void sda_out(void)
{
	ak880x_gpio_cfgpin(I2CSDAPIN, AK88_GPIO_OUT_0);
}
static inline int get_sda(void)
{
	int ret;
	ret = ak880x_gpio_getpin(I2CSDAPIN);
	if(ret)
		return 1;
	else 
		return 0;
}

static void i2c_message_start(struct ak880x_i2c *i2c, struct i2c_msg *msg)
{
	
}

/* #define T 800 */
#define T 8
/*
 * return 1 if get an ack, else return 0
 */
static inline int wait_ack(void)
{
	int ret;
	set_scl(0);
	sda_in();
	udelay(T/2);
	set_scl(1);
	udelay(T/4);
	ret = get_sda();
	udelay(T/4);

	set_scl(0);
	//udelay(1300);
	return !ret;
}

/* #define T0 200 */
#define T0 10
static inline void start_condition(void)
{
	sda_out();

	set_sda(1);
	set_scl(1);
	udelay(T0);

	set_sda(0);
	udelay(T0);

	set_scl(0);
	udelay(T0);
}
static inline void stop_condition(void)
{
	sda_out();

	set_sda(0);
	udelay(T0);

	set_scl(1);
	udelay(T0);

	set_sda(1);
	udelay(T0);
}

#if 0
// anyka version
static int write_byte(unsigned char data)
{
    unsigned char i,ret;
    unsigned char mask_bit[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
	sda_out();

    for (i=0; i<8; i++)
    {
        if (data & mask_bit[i])
        {
            set_sda(1);
        } else
        {
            set_sda(0);
        }
        udelay(5);
        set_scl(1);
        udelay(10);
        set_scl(0);
        udelay(5);
    }
    //read the bit of answer
	set_sda(1);
    sda_in();
    udelay(5);
    set_scl(1);
    udelay(5);
    if (get_sda() == 0)
        ret = 0;
    else ret = -1;
    udelay(5);
    set_scl(0);
    udelay(5);
    sda_out();
    return ret;
}

static unsigned char read_byte(void)
{
    unsigned char i, ret = 0;
	sda_in();
    for (i=0; i<8; i++)
    {
        udelay(5);
        set_scl(1);

        ret <<= 1;
        if (get_sda() == 1)
        {
            ret += 1;
        }

        udelay(10);
        set_scl(0);
        udelay(5);
    }

    //write the bit of answer
    sda_out();
    //set_sccb_pin(gpio.sio_d);
	set_sda(1);
    udelay(5);
    //set_sccb_pin(gpio.sio_c);
	set_scl(1);
    udelay(5);
    //clr_sccb_pin(gpio.sio_c);
	set_scl(0);
    udelay(5);

    return ret;
}
#endif




#if 1
// samsung version
static int write_byte(unsigned char data)
{
	int j;
	sda_out();
	for(j=7; j>=0; --j)
	{
		set_scl(0);
		udelay(T/4);
		set_sda((data>>j) & 0x1);
		udelay(T/4);
		set_scl(1);
		udelay(T/2);
	}
	set_scl(0);
	#if 0
	wait_ack();
	sda_out();
	set_sda(1);
	return 0;
	#endif
	if(wait_ack())
	{
		sda_out();
		set_sda(1);
	}
	else
	{
		//stop_condition();
		return -1;
	}
	return 0;	
}
static unsigned char read_byte(void)
{
	int j;
	unsigned char data = 0;
	sda_in();
	for(j=7; j>=0; --j)
	{
		set_scl(0);
		udelay(T/2);
		set_scl(1);
		udelay(T/4);
		data |= (get_sda()<<j);
		udelay(T/4);
	}
	set_scl(0);
	/* send ACK */
	sda_out();
	set_scl(0);
	udelay(T/4);
	set_sda(0);
	udelay(T/4);
	set_scl(1);
	udelay(T/2);

	set_scl(0);
	set_sda(1);
	return data&0xff;
}
#endif
/* ak880x_i2c_doxfer
 *
 * this starts an i2c transfer
 */
static int ak880x_i2c_doxfer(struct ak880x_i2c *i2c, struct i2c_msg *msgs, int num)
{
	int i,k; 
	int rw_flag;	// 0 for write, 1 for read 
	struct i2c_msg *p;

	if(i2c)
	{
		i2c->msg = msgs;
		i2c->msg_num = num;
		i2c->msg_ptr = 0;
		i2c->msg_idx = 0;
	}

	/* printk("num=%d\n", num); */

	start_condition();
	for(i=0; i<num; ++i)
	{
		p = msgs + i;
		IICDS = (p->addr << 1) & ~1;
		if(p->flags & I2C_M_RD)
			rw_flag = 1;
		else
			rw_flag = 0;
		if(p->flags & I2C_M_REV_DIR_ADDR)
			rw_flag ^= 1;
		IICDS |= rw_flag;
	
		/* start_condition(); */
		/* send the address byte */
		if(write_byte(IICDS) < 0)
		{
			stop_condition();
			return -EAGAIN;
		}
		if(rw_flag == 0)
		{
			/* write */
			for(k=0; k<p->len; ++k)
			{
				IICDS = p->buf[k];
				if(write_byte(IICDS) < 0)
				{
					stop_condition();
					return -EAGAIN;
				}
			}
		}
		else
		{
			/* read */
			for(k=0; k<p->len; ++k)
			{
				IICDS = read_byte();
				p->buf[k] = IICDS;
			}
		}
		/* stop_condition(); */
	}
	stop_condition(); /* for ta801 rda5802 */ 
	msleep(1);

	return num;
}

/* ak880x_i2c_xfer
 *
 * first port of call from the i2c bus code when an message needs
 * transferring across the i2c bus.
 */

static int ak880x_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct ak880x_i2c *i2c = (struct ak880x_i2c *)adap->algo_data;
	int retry;
	int ret;
	
	for(retry=0; retry<adap->retries; retry++)
	{
		ret = ak880x_i2c_doxfer(i2c, msgs, num);
		if(ret != -EAGAIN)
			return ret;
		udelay(100);
	}
	return -EREMOTEIO;
}

/* declare our i2c functionality */
static u32 ak880x_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}

/* i2c bus registration info */

static const struct i2c_algorithm ak880x_i2c_algorithm = {
	.master_xfer		= ak880x_i2c_xfer,
	.functionality		= ak880x_i2c_func,
};

static struct ak880x_i2c ak880x_i2c = {
	.lock		= __SPIN_LOCK_UNLOCKED(ak880x_i2c.lock),
	.wait		= __WAIT_QUEUE_HEAD_INITIALIZER(ak880x_i2c.wait),
	.tx_setup	= 50,
	.adap		= {
		.name			= "ak880x-i2c",
		.owner			= THIS_MODULE,
		.algo			= &ak880x_i2c_algorithm,
		.retries		= 2,
		.class			= I2C_CLASS_HWMON,
	},
};


/* ak880x_i2c_init
 *
 * initialise the hardware, set the IO lines and frequency 
 */

static int ak880x_i2c_init(struct ak880x_i2c *i2c)
{
	/* Setup GPIO for Software I2C bus */

	/* I2CSCL */
	scl_out();
	set_scl(1);
	/*
	ak880x_gpio_cfgpin(I2CSCLPIN, AK88_GPIO_OUT_0);
	ak880x_gpio_setpin(I2CSCLPIN, 1);
	*/
	/* I2CSDA */
	sda_out();
	set_sda(1);
	/*
	ak880x_gpio_cfgpin(I2CSDAPIN, AK88_GPIO_OUT_0);
	ak880x_gpio_setpin(I2CSDAPIN, 1);
	*/

	return 0;
}

/* ak880x_i2c_probe
 *
 * called by the bus driver when a suitable device is found
 */

static void test(void)
{
	unsigned char c = 0x5f;
	int val;
	struct i2c_msg  msg = {0x30, 0, 1, &c};
	val = ak880x_i2c_doxfer(NULL, &msg, 1);
	printk("return %d\n", val);
}
static int ak880x_i2c_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ak880x_i2c *i2c = &ak880x_i2c;

	/*printk("Enterring %s\n", __FUNCTION__);*/

	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;
	ret = ak880x_i2c_init(i2c);

	ret = i2c_add_adapter(&i2c->adap);
	if(ret < 0)
		goto out;
	platform_set_drvdata(pdev, i2c);
	#if 0
	while(1)
	{
		test();
	}
	#endif

out:
	return ret;
}

static int ak880x_i2c_remove(struct platform_device *pdev)
{
	struct ak880x_i2c *i2c;

	/*printk("Enterring %s\n", __FUNCTION__);*/

	i2c = platform_get_drvdata(pdev);
	i2c_del_adapter(&i2c->adap);
	return 0;
}

#ifdef CONFIG_PM
static int ak880x_i2c_resume(struct platform_device *dev)
{
	/*printk("Enterring %s\n", __FUNCTION__);*/

	return 0;
}

#else
#define ak880x_i2c_resume NULL
#endif

/* device driver for platform bus bits */

static struct platform_driver ak880x_i2c_driver = {
	.probe		= ak880x_i2c_probe,
	.remove		= ak880x_i2c_remove,
	.resume		= ak880x_i2c_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ak880x-i2c",
	},
};

static int __init i2c_adap_ak880x_init(void)
{
	printk("AK88 I2C (base gpio), (c) 2010 AK88");

	return platform_driver_register(&ak880x_i2c_driver);
}

static void __exit i2c_adap_ak880x_exit(void)
{
	platform_driver_unregister(&ak880x_i2c_driver);
}

module_init(i2c_adap_ak880x_init);
module_exit(i2c_adap_ak880x_exit);

MODULE_DESCRIPTION("AK88 I2C Bus driver");
MODULE_AUTHOR("anyka");
MODULE_LICENSE("GPL");
