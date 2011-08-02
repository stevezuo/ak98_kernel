/*
 * drivers/leds/leds-ak880x.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/ak880x_addr.h>

static void dump_regs(void)
{
	int i=0;

	for (i=0; i<4; i++) {
		printk("UART%d\n", i);
		printk("UART_CONF1:	0x%x\n", __raw_readl(AK88_VA_UART + i*0x1000 + 0x0));
		printk("UART_CONF2:	0x%x\n", __raw_readl(AK88_VA_UART + i*0x1000 + 0x4));
		printk("DATA_CONF:	0x%x\n", __raw_readl(AK88_VA_UART + i*0x1000 + 0x8));
		printk("BUF_THRE:	0x%x\n", __raw_readl(AK88_VA_UART + i*0x1000 + 0xC));
	}

	printk("Clock Control: 0x%x\n", __raw_readl(AK88_VA_SYSCTRL + 0xC));
}

static void lcd_backlight (int on)
{
	unsigned long regval;

	regval = __raw_readl(0xf0100094);
	regval &= ~(1<<13);
	__raw_writel(regval, 0xf0100094);

	if (on) {
		regval = __raw_readl(0xf0100098);
		regval |= (1<<13);
		__raw_writel(regval, 0xf0100098);
	} else {
		regval = __raw_readl(0xf0100098);
		regval &= ~(1<<13);
		__raw_writel(regval, 0xf0100098);
	}
}

static void backlight_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	if (value)
		lcd_backlight(1);
	else
		lcd_backlight(0);
}

static void keypad_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	unsigned long regval;

	regval = __raw_readl(0xf0100094);
	regval &= ~(1<<17);
	__raw_writel(regval, 0xf0100094);

	if (value) {
		regval = __raw_readl(0xf0100098);
		regval |= (1<<17);
		__raw_writel(regval, 0xf0100098);
	} else {
		regval = __raw_readl(0xf0100098);
		regval &= ~(1<<17);
		__raw_writel(regval, 0xf0100098);
	}

	/* dump_regs(); */
	dump_regs();
}

static struct led_classdev backlight_led = {
	.name			= "backlight_led",
	.brightness_set		= backlight_led_set,
};

static struct led_classdev keypad_led = {
	.name			= "keypad_led",
	.brightness_set		= keypad_led_set,
};

#ifdef CONFIG_PM
static int ak880xled_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&ak880x_led);
	return 0;
}

static int ak880xled_resume(struct platform_device *dev)
{
	led_classdev_resume(&ak880x_led);
	return 0;
}
#endif

static int ak880xled_probe(struct platform_device *pdev)
{
	int ret;
	unsigned long regval;

	/* printk("Enterring %s\n", __FUNCTION__); */

	regval = __raw_readl(0xf0100094);
	regval &= ~(1<<17);
	__raw_writel(regval, 0xf0100094);

	regval = __raw_readl(0xf0100098);
	regval |= (1<<17);
	__raw_writel(regval, 0xf0100098);
	
	ret = led_classdev_register(&pdev->dev, &backlight_led);
	if (ret)
		return ret;

	ret = led_classdev_register(&pdev->dev, &keypad_led);
	if (ret)
		return ret;

	return 0;
}

static int ak880xled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&keypad_led);
	led_classdev_unregister(&backlight_led);

	return 0;
}

static struct platform_driver ak880xled_driver = {
	.probe		= ak880xled_probe,
	.remove		= ak880xled_remove,
#ifdef CONFIG_PM
	.suspend	= ak880xled_suspend,
	.resume		= ak880xled_resume,
#endif
	.driver		= {
		.name		= "ak880x-led",
	},
};

static int __init ak880xled_init(void)
{
	return platform_driver_register(&ak880xled_driver);
}

static void __exit ak880xled_exit(void)
{
	platform_driver_unregister(&ak880xled_driver);
}

module_init(ak880xled_init);
module_exit(ak880xled_exit);

MODULE_AUTHOR("ANYKA>");
MODULE_DESCRIPTION("AK88 EVB LED driver");
MODULE_LICENSE("GPL");
