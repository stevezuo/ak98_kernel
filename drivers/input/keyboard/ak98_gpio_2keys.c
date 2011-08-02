/* 
 * drivers/input/keyboard/ak98_gpio_2keys.c
 * 
 * Driver for keys on GPIO lines capable of generating interrupts.
 * 
 *   Copyright 2010 Anyka
 *
 * - gpio-keys.c
 *	Copyright (c) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <mach/gpio_keys.h>

struct gpio_button_data {
	struct ak98_gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct gpio_button_data data[0];
};

static int gpio_get_value(unsigned long pin)
{
	return ak98_gpio_getpin(pin);
}

static int gpio_init(struct ak98_gpio_keys_button *button_data)
{
	ak98_setpin_as_gpio(button_data->gpio);
	ak98_gpio_cfgpin(button_data->gpio, button_data->dir);
	if (button_data->pullup == AK98_PULLUP_ENABLE || button_data->pullup == AK98_PULLUP_DISABLE)
		ak98_gpio_pullup(button_data->gpio, button_data->pullup);
	if (button_data->pulldown == AK98_PULLDOWN_ENABLE || button_data->pulldown == AK98_PULLDOWN_DISABLE)
		ak98_gpio_pulldown(button_data->gpio, button_data->pulldown);
	ak98_gpio_intpol(button_data->gpio, button_data->int_pol);
	
	return 0;
}

#if 0
static int gpio_direction_input(unsigned long pin)
{
	ak98_setpin_as_gpio(pin);
	ak98_gpio_cfgpin(pin, AK98_GPIO_DIR_INPUT);
	if(pin == AK98_GPIO_115) {
		ak98_gpio_pullup(pin, AK98_PULLUP_ENABLE);
		ak98_gpio_intpol(pin, AK98_GPIO_INT_LOWLEVEL);
	}
	else {
		ak98_gpio_pulldown(pin, AK98_PULLDOWN_ENABLE);
		ak98_gpio_intpol(pin, AK98_GPIO_INT_HIGHLEVEL);
	}
	
	return 0;
}
#endif

static void disable_button_irq(unsigned int irq)
{
	unsigned int pin;

	pin = ak98_irq_to_gpio(irq);
	ak98_gpio_intpol(pin, AK98_GPIO_INT_LOWLEVEL);
}
static void enable_button_irq(unsigned int irq)
{
	unsigned int pin;

	pin = ak98_irq_to_gpio(irq);	
	ak98_gpio_intpol(pin, AK98_GPIO_INT_HIGHLEVEL);
}

static void gpio_keys_report_event(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
	struct ak98_gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
	
	input_event(input, type, button->code, !!state);
	input_sync(input);
}

static void gpio_keys_timer(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;
	
	schedule_work(&data->work);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct ak98_gpio_keys_button *button = bdata->button;

	BUG_ON(irq != ak98_gpio_to_irq(button->gpio));

	//disalbe gpio_irq when the button down
	if(gpio_get_value(button->gpio))
		disable_button_irq(irq);

	//enable gpiot_irq when the button up
	if (!gpio_get_value(button->gpio))
		enable_button_irq(irq);

	if (button->debounce_interval)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(button->debounce_interval));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}



static int __devinit gpio_keys_probe(struct platform_device *pdev)
{	
	struct ak98_gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);
	
	//__set_bit(SW_LID, input->swbit); don't not need now

	ddata->input = input;


	for (i = 0; i < pdata->nbuttons; i++) 
	{
		struct ak98_gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		int irq;
		unsigned int type = button->type ?: EV_KEY;
		
		bdata->input = input;
		bdata->button = button;
		
		setup_timer(&bdata->timer,
		    gpio_keys_timer, (unsigned long)bdata);
		
		INIT_WORK(&bdata->work, gpio_keys_report_event);
		
	
		error = gpio_init(button);
		if (error < 0) {
			pr_err("gpio-keys: failed to configure input"
				" direction for GPIO %d, error %d\n",
				button->gpio, error);
			ak98_gpio_free(button->gpio);
			goto fail2;
		}

		irq = ak98_gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("gpio-keys: Unable to get irq number"
				" for GPIO %d, error %d\n",
				button->gpio, error);
			ak98_gpio_free(button->gpio);
			goto fail2;
		}

		error = request_irq(irq, gpio_keys_isr, 
			        (button->active_low)?(IRQF_TRIGGER_LOW):(IRQF_TRIGGER_HIGH),
				    button->desc ? button->desc : "gpio_keys", bdata);
	
		if (error) {
			pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				irq, error);
			ak98_gpio_free(button->gpio);
			goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);

#ifdef CONFIG_PM
		ak98_gpio_wakeup_pol(button->gpio, button->active_low?(AK98_FALLING_TRIGGERED):(AK98_RISING_TRIGGERED));
#endif
	}

	
	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(ak98_gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		ak98_gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{	
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = ak98_gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		ak98_gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}

#if 0
static void print()
{
	printk("-------------------------\n");
	printk("Po:\t%x\n", REG32(AK98_WGPIO_POLARITY));
	printk("Cle:\t%x\n", REG32(AK98_WGPIO_CLEAR));
	printk("Ena:\t%x\n", REG32(AK98_WGPIO_ENABLE));
	printk("Sta:\t%x\n", REG32(AK98_WGPIO_STATUS));
	printk("-------------------------\n");
}
#endif

#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct ak98_gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct ak98_gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) 
			{
				ak98_gpio_wakeup(button->gpio, AK98_WAKEUP_ENABLE);				
			}
		}
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ak98_gpio_keys_platform_data *pdata = pdev->dev.platform_data;
		
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct ak98_gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) 
			{
				ak98_gpio_wakeup(button->gpio, AK98_WAKEUP_DISABLE);				
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio_keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anyka <xx@anyka.oa");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
