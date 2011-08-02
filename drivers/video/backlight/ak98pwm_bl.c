
/*
 * linux/drivers/video/backlight/ak98pwm_bl.c
 * 2011-4-13
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/pwm.h>


//#define PWM_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef PWM_DEBUG
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



static struct ak98_pwm *pwm_dev;

static int pwm_backlight_update_status(struct backlight_device *bl)
{	
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	unsigned short bt = brightness * pwm_dev->pwm_clk / max;

	PDEBUG("Entering %s\n", __FUNCTION__);
	PDEBUG("brightness: %d\n", brightness);
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	ak98_pwm_config(pwm_dev, bt ? bt:1, pwm_dev->pwm_clk - bt);
	ak98_pwm_enable(pwm_dev);
	
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	PDEBUG("Entering %s\n", __FUNCTION__);
	return bl->props.brightness;
}

static struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	
	struct ak98_platform_pwm_bl_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	//struct pwm_bl_data *pb;
	int ret;

	pwm_dev = ak98_pwm_request(data->pwm_id);
	if (pwm_dev == NULL)
	{
		dev_err(&pdev->dev, "failed to request pwm device\n");
		return -EINVAL;
	}
		
	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(pwm_dev);
		if (ret < 0)
			return ret;
	}
	
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev,
			NULL, &pwm_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
	pwm_dev->pwm_clk = data->pwm_clk;
	backlight_update_status(bl);
	
	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	if (data->exit)
		data->exit(pwm_dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct ak98_platform_pwm_bl_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_device_unregister(bl);
	
	if (data->exit)
		data->exit(pwm_dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	//struct backlight_device *bl = platform_get_drvdata(pdev);
	
	ak98_pwm_config(pwm_dev, 0, 0);
	ak98_pwm_disable(pwm_dev);
	
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver ak98pwm_backlight_driver = {
	.driver		= {
		.name	= "ak98pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	PDEBUG("AK98 PWM Backlight Driver!\n");
	
	return platform_driver_register(&ak98pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&ak98pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("AK98 PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak98pwm-backlight");






