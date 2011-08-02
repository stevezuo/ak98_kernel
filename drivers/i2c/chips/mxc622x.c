/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>

#include <linux/mxc622x.h>

#define DEBUG			0
#define MAX_FAILURE_COUNT	3

#define MXC622X_DELAY_PWRON	300	/* ms, >= 300 ms */
#define MXC622X_DELAY_PWRDN	1	/* ms */
#define MXC622X_DELAY_SETDETECTION	MXC622X_DELAY_PWRON

#define MXC622X_RETRY_COUNT	3

struct mxc622x_accl_data {
	const struct mxc622x_platform_data *pdata;
	struct i2c_client *client;
};

static struct mxc622x_accl_data *mxc622x_data;

static int mxc622x_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= mxc622x_data->client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= mxc622x_data->client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MXC622X_RETRY_COUNT; i++) {
		if (i2c_transfer(mxc622x_data->client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MXC622X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MXC622X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mxc622x_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= mxc622x_data->client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < MXC622X_RETRY_COUNT; i++) {
		if (i2c_transfer(mxc622x_data->client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MXC622X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MXC622X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mxc622x_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int mxc622x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxc622x_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	void __user *pa = (void __user *)arg;
	unsigned char data[16] = {0};
	int vec[3] = {0};

	switch (cmd) {
	case MXC622X_IOC_PWRON:
		data[0] = MXC622X_REG_CTRL;
		data[1] = MXC622X_CTRL_PWRON;
		if (mxc622x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait PWRON done */
		msleep(MXC622X_DELAY_PWRON);
		break;
	case MXC622X_IOC_PWRDN:
		data[0] = MXC622X_REG_CTRL;
		data[1] = MXC622X_CTRL_PWRDN;
		if (mxc622x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait PWRDN done */
		msleep(MXC622X_DELAY_PWRDN);
		break;
	case MXC622X_IOC_READXYZ:
		data[0] = MXC622X_REG_DATA;
		if (mxc622x_i2c_rx_data(data, 2) < 0) {
			return -EFAULT;
		}
		vec[0] = (int)data[0];
		vec[1] = (int)data[1];
		vec[2] = (int)data[2];
	#if DEBUG
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}
		break;
	case MXC622X_IOC_READSTATUS:
		data[0] = MXC622X_REG_DATA;
		if (mxc622x_i2c_rx_data(data, 3) < 0) {
			return -EFAULT;
		}
		vec[0] = (int)data[0];
		vec[1] = (int)data[1];
		vec[2] = (unsigned int)data[2];
	#if DEBUG
		printk("[X - %04x] [Y - %04x] [STATUS - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
			return -EFAULT;
		}
		break;
	case MXC622X_IOC_SETDETECTION:
		data[0] = MXC622X_REG_CTRL;
		if (copy_from_user(&(data[1]), pa, sizeof(unsigned char))) {
			return -EFAULT;
		}
		if (mxc622x_i2c_tx_data(data, 2) < 0) {
			return -EFAULT;
		}
		/* wait SETDETECTION done */
		msleep(MXC622X_DELAY_SETDETECTION);
		break;
	default:
		break;
	}

	return 0;
}

static struct file_operations mxc622x_fops = {
       .owner          = THIS_MODULE,
       .open           = mxc622x_open,
       .release        = mxc622x_release,
       .ioctl          = mxc622x_ioctl,
};

static struct miscdevice mxc622x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxc622x",
	.fops = &mxc622x_fops,
};

int mxc622x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	const struct mxc622x_platform_data *pdata = client->dev.platform_data;
	
	if(!pdata->exist)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
	}

	mxc622x_data = kzalloc(sizeof(struct mxc622x_accl_data), GFP_KERNEL);
	if (!mxc622x_data) {
		pr_err("%s: mxc622x_accl_data alloc failed\n", __FUNCTION__);
		res = -ENOMEM;
		goto out;
	}
	
	mxc622x_data->pdata = pdata;
	mxc622x_data->client = client;
	i2c_set_clientdata(client, mxc622x_data);

	res = misc_register(&mxc622x_device);
	if (res) {
		pr_err("%s: mxc622x_device register failed\n", __FUNCTION__);
		goto out_deregister;
	}

	return 0;

out_deregister:
	kfree(mxc622x_data);
out:
	return res;
}

static int mxc622x_remove(struct i2c_client *client)
{
	misc_deregister(&mxc622x_device);
	kfree(mxc622x_data);

	return 0;
}

static const struct i2c_device_id mxc622x_id[] = {
	{ MXC622X_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver mxc622x_driver = {
	.probe 		= mxc622x_probe,
	.remove 	= mxc622x_remove,
	.id_table	= mxc622x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name = MXC622X_I2C_NAME,
	},
};

static int __init mxc622x_init(void)
{
	pr_info("mxc622x driver: init\n");
	return i2c_add_driver(&mxc622x_driver);
}

static void __exit mxc622x_exit(void)
{
	pr_info("mxc622x driver: exit\n");
	i2c_del_driver(&mxc622x_driver);
}

module_init(mxc622x_init);
module_exit(mxc622x_exit);

MODULE_AUTHOR("Robbie Cao<hjcao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MXC622X (DTOS) Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");

