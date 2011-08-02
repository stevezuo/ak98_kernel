/* 
 * AK8802 gpio track ball mouse driver
 *
 * Copyright (C) 2010 Anyka Ltd.
 * 2010-06-09: Jacky Lau <liu_zhuyuan@anyka.com>
 *             initial version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <mach/gpio.h>

struct input_dev *ak88_trkball_input_dev;

#define TRKBALL_GPIO_COUNT 4

struct ak880x_trkball {
	unsigned int irq_type[TRKBALL_GPIO_COUNT];
};

struct ak880x_trkball_irq_settings {
	const char *name;
	irq_handler_t handler;
};

static irqreturn_t ak880x_trkball_up_interrupt(int irq, void *dev_id);
static irqreturn_t ak880x_trkball_down_interrupt(int irq, void *dev_id);
static irqreturn_t ak880x_trkball_left_interrupt(int irq, void *dev_id);
static irqreturn_t ak880x_trkball_right_interrupt(int irq, void *dev_id);

static struct ak880x_trkball_irq_settings ak880x_trkball_gpio_irq_settings[TRKBALL_GPIO_COUNT] = {
	{
		.name    = "UP GPIO IRQ",
		.handler = ak880x_trkball_up_interrupt,
	}, {
		.name    = "DOWN GPIO IRQ",
		.handler = ak880x_trkball_down_interrupt,
	}, {
		.name    = "LEFT GPIO IRQ",
		.handler = ak880x_trkball_left_interrupt,
	}, {
		.name    = "RIGHT GPIO IRQ",
		.handler = ak880x_trkball_right_interrupt,
	},
};

static irqreturn_t ak880x_trkball_up_interrupt(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;
	struct ak880x_trkball *trkball = input_get_drvdata(input);

	input_report_rel(input, REL_Y, -1);
	input_sync(input);

	if (trkball->irq_type[0] == IRQ_TYPE_LEVEL_LOW) {
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		trkball->irq_type[0] = IRQ_TYPE_LEVEL_HIGH;
	} else {
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		trkball->irq_type[0] = IRQ_TYPE_LEVEL_LOW;
	}

	return IRQ_HANDLED;
}

static irqreturn_t ak880x_trkball_down_interrupt(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;
	struct ak880x_trkball *trkball = input_get_drvdata(input);

	input_report_rel(input, REL_Y, 1);
	input_sync(input);

	if (trkball->irq_type[1] == IRQ_TYPE_LEVEL_LOW) {
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		trkball->irq_type[1] = IRQ_TYPE_LEVEL_HIGH;
	} else {
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		trkball->irq_type[1] = IRQ_TYPE_LEVEL_LOW;
	}

	return IRQ_HANDLED;
}

static irqreturn_t ak880x_trkball_left_interrupt(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;
	struct ak880x_trkball *trkball = input_get_drvdata(input);

	input_report_rel(input, REL_X, -1);
	input_sync(input);

	if (trkball->irq_type[2] == IRQ_TYPE_LEVEL_LOW) {
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		trkball->irq_type[2] = IRQ_TYPE_LEVEL_HIGH;
	} else {
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		trkball->irq_type[2] = IRQ_TYPE_LEVEL_LOW;
	}

	return IRQ_HANDLED;
}

static irqreturn_t ak880x_trkball_right_interrupt(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;
	struct ak880x_trkball *trkball = input_get_drvdata(input);

	input_report_rel(input, REL_X, 1);
	input_sync(input);

	if (trkball->irq_type[3] == IRQ_TYPE_LEVEL_LOW) {
		set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
		trkball->irq_type[3] = IRQ_TYPE_LEVEL_HIGH;
	} else {
		set_irq_type(irq, IRQ_TYPE_LEVEL_LOW);
		trkball->irq_type[3] = IRQ_TYPE_LEVEL_LOW;
	}

	return IRQ_HANDLED;
}

static atomic_t trkball_ref = ATOMIC_INIT(0);

static int ak880x_trkball_open(struct input_dev *dev)
{
	int ref;

	ref = atomic_inc_return(&trkball_ref);
	if (ref == 1) {
		/* for track ball power */
		AK88_GPIO_PCM_JTAG(AK88_SHARE_GPIO);
		ak880x_gpio_pullup(AK88_GPIO_3, AK88_GPIO_PUPD_ENABLE);
		ak880x_gpio_cfgpin(AK88_GPIO_3, AK88_GPIO_OUT_0);
		/* power on */
		ak880x_gpio_setpin(AK88_GPIO_3, 1);
	}

	return 0;
}

static void ak880x_trkball_close(struct input_dev *dev)
{
	int ref;

	ref = atomic_dec_return(&trkball_ref);
	if (ref == 0) {
		/* track ball power off */
		ak880x_gpio_setpin(AK88_GPIO_3, 0);
	}

	return;
}

static int __devinit ak880x_trkball_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ak880x_trkball *trkball;
	struct input_dev *input;
	int irq[TRKBALL_GPIO_COUNT], i, error;
#if 0
	int gpio;
#endif

	for (i = 0; i < TRKBALL_GPIO_COUNT; i++) {
		irq[i] = -1;
	}

	trkball = kzalloc(sizeof(struct ak880x_trkball), GFP_KERNEL);
	if (!trkball)
		return -ENOMEM;

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed;
	}

	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->open = ak880x_trkball_open;
	input->close = ak880x_trkball_close;
	input->dev.parent = &pdev->dev;
	input_set_drvdata(input, trkball);

	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);
	input_set_capability(input, EV_KEY, BTN_LEFT);

	error = input_register_device(input);
	if (error) {
		dev_err(&pdev->dev, "unable to register input device\n");
		goto failed_free_input;
	}

	platform_set_drvdata(pdev, input);

#if 0	/* track ball gpio setting, UNNECESSARY */
	AK88_GPIO_SPI1(AK88_SHARE_GPIO);
#endif

	/* track ball irq setting */
	for (i = 0; i < TRKBALL_GPIO_COUNT; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, ak880x_trkball_gpio_irq_settings[i].name);
		if (res == NULL) {
			dev_err(&pdev->dev, "failed to get trkball irq\n");
			error = -ENXIO;
			goto failed_free_irq;
		}

#if 0			/* track ball gpio setting, UNNECESSARY */
		gpio = ak880x_irq_to_gpio(res->start);
		ak880x_gpio_pullup(gpio, AK88_GPIO_PUPD_DISABLE);
		ak880x_gpio_cfgpin(gpio, AK88_GPIO_IN_0);
#endif

		irq[i] = res->start;
		trkball->irq_type[i] = res->flags & IORESOURCE_BITS;
		set_irq_type(irq[i], trkball->irq_type[i]);
		error = request_irq(irq[i], ak880x_trkball_gpio_irq_settings[i].handler,
				    IRQF_DISABLED, pdev->name, input);
		if (error) {
			dev_err(&pdev->dev, "failed to request irq: %d\n", error);
			goto failed_free_irq;
		}
	}

	ak88_trkball_input_dev = input;

	return 0;

failed_free_irq:
	for (i = 0; i < TRKBALL_GPIO_COUNT; i++) {
		if (irq[i] >= 0)
			free_irq(irq[i], input);
	}
failed_free_input:
	input_free_device(input);
failed:
	kfree(trkball);
	return error;
}

static int __devexit ak880x_trkball_remove(struct platform_device *pdev)
{
	struct input_dev *input = platform_get_drvdata(pdev);
	struct ak880x_trkball *trkball = input_get_drvdata(input);
	int irq, i;

	ak88_trkball_input_dev = NULL;

	for (i = 0; i < TRKBALL_GPIO_COUNT; i++) {
		irq = platform_get_irq(pdev, i);
		free_irq(irq, input);
	}

	input_unregister_device(input);
	kfree(trkball);

	return 0;
}

static struct platform_driver ak880x_trkball_driver = {
	.driver		= {
		.name	= "ak880x-trkball",
	},
	.probe		= ak880x_trkball_probe,
	.remove		= __devexit_p(ak880x_trkball_remove),
};

static int __init ak880x_trkball_init(void)
{
	return platform_driver_register(&ak880x_trkball_driver);
}

static void __exit ak880x_trkball_exit(void)
{
	platform_driver_unregister(&ak880x_trkball_driver);
}

module_init(ak880x_trkball_init);
module_exit(ak880x_trkball_exit);

MODULE_AUTHOR("Jacky Lau <liu_zhuyuan@anyka.com>");
MODULE_DESCRIPTION("Ak8802 ebook Trackball Mouse Driver");
MODULE_LICENSE("GPL");
