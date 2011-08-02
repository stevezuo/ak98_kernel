/*
 *  ak98 aw9523.c - expand 16 bit I/O ports
 *
 *  Copyright (C) 2011 Anyka electronic Ltd
 *
 *  Derived from drivers/i2c/busses/aw9523.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; 
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c/aw9523.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>


//#define AW9523_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef AW9523_DEBUG
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


static struct aw9523_chip *chip;
static int g_input_update=0;


static void		aw9523_gpioirq_enable(unsigned int irq)
{
	PDEBUG("Entering %s %u\n", __FUNCTION__, irq); 
	ak98_gpio_intcfg(ak98_irq_to_gpio(irq), AK98_GPIO_INT_ENABLE);
}
static void		aw9523_gpioirq_disable(unsigned int irq)
{
	PDEBUG("Entering %s %u\n", __FUNCTION__, irq);
	ak98_gpio_intcfg(ak98_irq_to_gpio(irq), AK98_GPIO_INT_DISABLE);
}

static void aw9523_gpioirq_mask(unsigned int irq)
{
	unsigned pino = irq - chip->pdata->irq_base + chip->pdata->gpio_base;
	int index, off;

	PDEBUG("Entering %s %u\n", __FUNCTION__, irq);
	pino -= AW9523_GPIO_P00;
	
	index = pino / 8;
	off = pino % 8;

	if (index)
	{
		chip->irq_mask1 |= (1u << off);
	}
	else
	{
		chip->irq_mask0 |= (1u << off);
	}
}

static void aw9523_gpioirq_unmask(unsigned int irq)
{
	unsigned pino = irq - chip->pdata->irq_base + chip->pdata->gpio_base;
	int index, off;

	PDEBUG("Entering %s %u\n", __FUNCTION__, irq);
	pino -= AW9523_GPIO_P00;
	
	index = pino / 8;
	off = pino % 8;

	if (index)
	{
		chip->irq_mask1 &= ~(1u << off);
	}
	else
	{
		chip->irq_mask0 &= ~(1u << off);
	}
}

static struct irq_chip aw9523_gpio_chip = {
	.name = "aw9523_irq",
	.mask_ack = aw9523_gpioirq_mask,
	.mask = aw9523_gpioirq_mask,
	.unmask = aw9523_gpioirq_unmask,
	.enable = aw9523_gpioirq_unmask,//,aw9523_gpioirq_enable,
	.disable = aw9523_gpioirq_mask,
};

static int aw9523_write_reg(struct i2c_client *client, int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading register\n");
		return ret;
	}
	return 0;
}


static int aw9523_read_reg(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;
	
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing register\n");
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}



//unsigned int irq, struct irq_desc *desc
static void aw9523_irq_dispatch(int irqno, void *dev_id)
{
	uint8_t input0, input1;
	int i, off;

	PDEBUG("Entering %s ......  %d\n", __FUNCTION__, irqno);
	aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT0, &input0);	
	PDEBUG("input0: 0x%x\n", input0);
	aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT1, &input1);
	PDEBUG("input1: 0x%x\n", input1);
	
	if (input0 != chip->reg_input0)
	{
		PDEBUG("old input0: 0x%x\n", chip->reg_input0);
		for (i = AW9523_GPIO_P00; i<=AW9523_GPIO_P07; i++)
		{
			off = i-AW9523_GPIO_P00;
			if ( ((input0 & (1u << off)) != (chip->reg_input0 & (1u << off))) &&
				((chip->irq_mask0 & (1u << off)) == 0) )
			{
				//pin i had intrrupt		
				PDEBUG("dispatch:  %d\n", i - chip->pdata->gpio_base + chip->pdata->irq_base);
				generic_handle_irq(i - chip->pdata->gpio_base + chip->pdata->irq_base);
			}
		}
		chip->reg_input0 = input0;
	}

	if (input1 != chip->reg_input1)
	{
		for (i = AW9523_GPIO_P10; i<=AW9523_GPIO_P17; i++)
		{
			off = i-AW9523_GPIO_P10;
			if ( ((input1 & (1u << off)) != (chip->reg_input1 & (1u << off))) &&
				((chip->irq_mask1 & (1u << off)) == 0) )
			{
				//pin i had intrrupt
				generic_handle_irq(i - chip->pdata->gpio_base + chip->pdata->irq_base);
			}
		}
		chip->reg_input1 = input1;
	}

	enable_irq(chip->pdata->parent_irq);

}

static void aw9523_irq_setup( void )
{
	int i;
	
	chip->irq_mask0 = chip->irq_mask1 = 0;
	
	for (i=IRQ_AW9523_P00; i<=IRQ_AW9523_P17; i++)
	{
		set_irq_chip(i, &aw9523_gpio_chip);
		set_irq_handler(i, handle_level_irq);		
		set_irq_flags(i, IRQF_VALID);
	}
}


int aw9523_gpio_to_irq(unsigned int pino)
{
	return chip->pdata->irq_base + (pino - chip->pdata->gpio_base);
}
EXPORT_SYMBOL(aw9523_gpio_to_irq);

int aw9523_irq_to_gpio(unsigned int irq)
{
	return chip->pdata->gpio_base + (irq - chip->pdata->irq_base);
}


int aw9523_gpio_intcfg(unsigned int pino, unsigned int enable)
{
	int ret, index, off;
	uint8_t reg_val;

	pino -= AW9523_GPIO_P00;

	PDEBUG("%s  %u  %u\n", __FUNCTION__, pino, enable);
	if(pino > PMAX) 
	{
		panic("expand GPIO not existing\n");
		return -1;
	}
	
	index = pino / 8;
	off = pino % 8;

	if (index)
	{
		/* 0: enable   1: disable*/
		if (enable == AK98_GPIO_INT_ENABLE)
		{
			reg_val = chip->reg_int1 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_INTN_PORT1, reg_val);
		}
		else if (enable == AK98_GPIO_INT_DISABLE)
		{
			reg_val = chip->reg_int1 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_INTN_PORT1, reg_val);
		}
		else
			panic("Bad parameter passed to aw9523_int_cfg");
		chip->reg_int1 = reg_val;
	}
	else
	{
		if (enable == AK98_GPIO_INT_ENABLE)
		{
			reg_val = chip->reg_int0 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_INTN_PORT0, reg_val);
		}
		else if (enable == AK98_GPIO_INT_DISABLE)
		{
			reg_val = chip->reg_int0 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_INTN_PORT0, reg_val);
		}
		else
			panic("Bad parameter passed to aw9523_int_cfg");
		chip->reg_int0 = reg_val;
	}

	return 0;
}

EXPORT_SYMBOL(aw9523_gpio_intcfg);


int aw9523_gpio_dircfg(unsigned int pino, unsigned int direction)
{
	int ret, index, off;
	uint8_t reg_val;

	pino -= AW9523_GPIO_P00;

	PDEBUG("%s  %u  %u\n", __FUNCTION__, pino, direction);
	if(pino > PMAX) 
	{
		panic("expand GPIO not existing\n");
		return -1;
	}
	
	index = pino / 8;
	off = pino % 8;

	if(index) 
	{
		/* 0: OUTPUT  1: INPUT*/
		if(direction == AK98_GPIO_DIR_INPUT)
		{
			reg_val = chip->reg_direction1 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_CFG_PORT1, reg_val);
		}
		else if(direction == AK98_GPIO_DIR_OUTPUT){
			reg_val = chip->reg_direction1 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_CFG_PORT1, reg_val);
		} 
		else 
			panic("AW9523 GPIO direction error");
		if(ret)
		{
			PDEBUG("%s Error happend\n", __FUNCTION__);
			return ret;
		}
		chip->reg_direction1 = reg_val;
	}
	else 
	{
		if(direction == AK98_GPIO_DIR_INPUT){
			reg_val = chip->reg_direction0 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_CFG_PORT0, reg_val);
		}
		else if(direction == AK98_GPIO_DIR_OUTPUT){
			reg_val = chip->reg_direction0 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_CFG_PORT0, reg_val);
		} else panic("AW9523 GPIO direction error");
		if(ret)
			return ret;
		chip->reg_direction0 = reg_val;
	}
	g_input_update = 1;	
	return 0;	
}
EXPORT_SYMBOL(aw9523_gpio_dircfg);


int aw9523_gpio_setpin(unsigned int pino, unsigned int level)
{
	int ret, index, off;
	uint8_t reg_val;

	pino -= AW9523_GPIO_P00;

	PDEBUG("%s  %d  %d\n", __FUNCTION__, pino, level);
	if(pino > PMAX) {
		panic("expand GPIO not existing\n");
		return -1;
	}
	
	index = pino / 8;
	off = pino % 8;

	/* 1: output HIGH   0: output LOW*/
	if(index) 
	{
		if(level == AK98_GPIO_OUT_HIGH)
		{
			reg_val = chip->reg_output1 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_OUTPUT_PORT1, reg_val);
		}
		else if(level == AK98_GPIO_OUT_LOW)
		{
			reg_val = chip->reg_output1 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_OUTPUT_PORT1, reg_val);
		} 
		else 
			panic("AW9523 GPIO level error");
		if(ret)
			return -1;
		chip->reg_output1 = reg_val;
		
	}
	else 
	{
		if(level == AK98_GPIO_OUT_HIGH) {
			reg_val = chip->reg_output0 | (1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_OUTPUT_PORT0, reg_val);
		}
		else if(level == AK98_GPIO_OUT_LOW){
			reg_val = chip->reg_output0 & ~(1u << off);
			ret = aw9523_write_reg(chip->client, AW9523_REG_OUTPUT_PORT0, reg_val);
		} 
		else panic("AW9523 GPIO level error");
		if(ret)
			return -1;
		chip->reg_output0 = reg_val;
	}	
	/* input state may change*/	
	g_input_update = 1;
	return 0;
}
EXPORT_SYMBOL(aw9523_gpio_setpin);

int aw9523_gpio_getpin(unsigned int pino)
{
	int ret=0, index, off;
	uint8_t reg_val;
	
	pino -= AW9523_GPIO_P00;

	if(pino > PMAX) {
		panic("expand GPIO not existing\n");
		return -1;
	}

	//PDEBUG("%s  %d  %d\n", __FUNCTION__, pino, g_input_update);
	index = pino / 8;
	off = pino % 8;
	
	if(index) 
	{
		if (g_input_update)
		{
			ret = aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT1, &reg_val);
			chip->reg_input1 = reg_val;
			g_input_update = 0;
		}
		else
			reg_val = chip->reg_input1;
	}
	else
	{
		if (g_input_update)
		{
			ret = aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT0, &reg_val);
			chip->reg_input0 = reg_val;
			g_input_update = 0;
		}
		else
			reg_val = chip->reg_input0;
	}
	
	if(ret < 0)
		return 0;

	//printk("--input0: 0x%x\n", reg_val);
	return (reg_val & (1u << off)) ? 1 : 0;
}
EXPORT_SYMBOL(aw9523_gpio_getpin);

/* function: set group port0 working module
 * param module:
 *	1: push-pull  0: open-drain
 * param pino: pin number
 */
static int aw9523_gpio_setmod(int mode)
{
	uint8_t reg_val = 0;
	int ret;
	
	if (mode == AW9523_MOD_PUSHPULL)
		reg_val = (1u << 4);
	else if (mode == AW9523_MOD_OPENDRAIN)
		;
	else 
		panic("AW9523 GPIO mode error");
	
	ret = aw9523_write_reg(chip->client, AW9523_REG_GPOMD, reg_val);
	if(ret)
		return ret;
	chip->reg_setmod = reg_val;
	
	return 0;
}

static int aw9523_init_gpio(struct aw9523_chip *chip)
{
	int ret, i;

	
	/* default mode of P00 - P07 is OPEN-DRAIN*/
	/* setup gpio push-pull */
	ret = aw9523_gpio_setmod(AW9523_MOD_PUSHPULL);

	if (ret)
		return -1;
	
	for (i=AW9523_GPIO_P00; i<=AW9523_GPIO_P17; i++)
	{
		ak98_gpio_dircfg(i, AK98_GPIO_DIR_INPUT);
		ak98_gpio_intcfg(i, AK98_GPIO_INT_DISABLE);		
	}
		
	return 0;
}

#if 0
static void aw9523_print_regs(void)
{
	PDEBUG("output0: 0x%x\n", chip->reg_output0);	
	PDEBUG("output1: 0x%x\n", chip->reg_output1);	
	PDEBUG("dir 0: 0x%x\n", chip->reg_direction0);	
	PDEBUG("dir 1: 0x%x\n", chip->reg_direction1);	
	PDEBUG("mode: 0x%x\n", chip->reg_setmod);	
	PDEBUG("int 0: 0x%x\n", chip->reg_int0);	
	PDEBUG("int 1: 0x%x\n", chip->reg_int1);	
	PDEBUG("input 0: 0x%x\n", chip->reg_input0);
	PDEBUG("input 1: 0x%x\n", chip->reg_input1);
}
#endif

static void aw9523_get_regs_value(struct aw9523_chip *chip)
{

	aw9523_read_reg(chip->client, AW9523_REG_OUTPUT_PORT0, &chip->reg_output0);
	aw9523_read_reg(chip->client, AW9523_REG_OUTPUT_PORT1, &chip->reg_output1);
	aw9523_read_reg(chip->client, AW9523_REG_CFG_PORT0, &chip->reg_direction0);
	aw9523_read_reg(chip->client, AW9523_REG_CFG_PORT1, &chip->reg_direction1);
	aw9523_read_reg(chip->client, AW9523_REG_GPOMD, &chip->reg_setmod);
	aw9523_read_reg(chip->client, AW9523_REG_INTN_PORT0, &chip->reg_int0);
	aw9523_read_reg(chip->client, AW9523_REG_INTN_PORT1, &chip->reg_int1);
	aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT0, &chip->reg_input0);
	aw9523_read_reg(chip->client, AW9523_REG_INPUT_PORT1, &chip->reg_input1);
}


static irqreturn_t aw9523_gpio_irq(int irqno, void *dev_id)
{

	PDEBUG("Entering %s %u\n", __FUNCTION__, irqno);

	disable_irq_nosync(chip->pdata->parent_irq);	
	aw9523_irq_dispatch(irqno, dev_id);	

	return IRQ_HANDLED;
}

static int __devinit aw9523_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct aw9523_platform_data *pdata = client->dev.platform_data;
	int ret, i;

	if (!pdata) 
	{
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	chip = kzalloc(sizeof(struct aw9523_chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->reg_setmod = pdata->p0xmod;
	chip->pdata = pdata;
		
	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	

	i2c_set_clientdata(client, chip);
	mutex_init(&(chip->aw9523_lock));

	ret = aw9523_init_gpio(chip);
	if (ret != 0)
		goto out_failed;
	
	aw9523_get_regs_value(chip);
	aw9523_irq_setup();

	for (i=0; i<pdata->npins; i++)
		ak98_gpio_set(&(pdata->gpios[i]));
	
	ret = request_threaded_irq(chip->pdata->parent_irq, NULL, aw9523_gpio_irq, IRQF_ONESHOT, "aw9523_irq", chip);
	if (ret != 0) 
	{
		dev_err(&(client->dev), "cannot claim IRQ %d\n", chip->pdata->parent_irq);
		goto out_failed;
	}

	PDEBUG("Load AW9523 successfully.\n");
	return 0;

out_failed:
	kfree(chip);
	return ret;
}

static int aw9523_remove(struct i2c_client *client)
{
	mutex_destroy(&chip->aw9523_lock);
	free_irq(chip->pdata->parent_irq, chip);
	kfree(chip);

	return 0;
}
static const struct i2c_device_id aw9523_id[] = {
	{ "aw9523_ext", 16},
	{ },
};

static struct i2c_driver aw9523_driver = {
	.driver = {
		.name	= "aw9523_ext",
		.owner	= THIS_MODULE,
	},
	.probe		= aw9523_probe,
	.remove		= aw9523_remove,
	.id_table	= aw9523_id,
};

static int __init aw9523_init(void)
{
	
	return i2c_add_driver(&aw9523_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(aw9523_init);

static void __exit aw9523_exit(void)
{
	i2c_del_driver(&aw9523_driver);
}
module_exit(aw9523_exit);
MODULE_AUTHOR("Anyka zhou_wenyong@anyka.oa");
MODULE_DESCRIPTION("GPIO expander driver for AW9523");
MODULE_LICENSE("GPL");
