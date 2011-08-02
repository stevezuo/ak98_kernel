/* 
 * AK98 PWM controler driver 
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/pwm.h>

struct ak98_pwm ak98_pwm1 = {
	.id = 1,
	.gpio = AK98_GPIO_9,
	.pwm_ctrl = AK98_PWM1_CTRL,
};

struct ak98_pwm ak98_pwm2 = {
	.id = 2,
	.gpio = AK98_GPIO_10,
	.pwm_ctrl = AK98_PWM2_CTRL,
};

struct ak98_pwm ak98_pwm3 = {
	.id = 3,
	.gpio = AK98_GPIO_11,
	.pwm_ctrl = AK98_PWM3_CTRL,
};

struct ak98_pwm ak98_pwm4 = {
	.id = 4,
	.gpio = AK98_GPIO_12,
	.pwm_ctrl = AK98_PWM4_CTRL,
};

int ak98_pwm_enable(struct ak98_pwm *pwm)
{
	switch(pwm->id) {
		case 1:
			ak98_group_config(ePIN_AS_PWM1);			
			break;
		case 2:
			ak98_group_config(ePIN_AS_PWM2);
			break;
		case 3:
			ak98_group_config(ePIN_AS_PWM3);
			break;
		case 4:
			ak98_group_config(ePIN_AS_PWM4);
			break;
		default:
			printk("init pwm error");
			return -1;
	}
	return 0;
}

EXPORT_SYMBOL(ak98_pwm_enable);

static int ak98_pwm_set_duty_cycle(struct ak98_pwm *pwm, unsigned short high, unsigned short low)
{	
	__raw_writel(high << 16 | low, pwm->pwm_ctrl);
	return 0;
}


static int ak98_pwm_get_duty_cycle(struct ak98_pwm *pwm, unsigned short *high, unsigned short *low)
{
	unsigned long regval;

	regval = __raw_readl(pwm->pwm_ctrl);

	*high = regval >> 16;
	*low = regval & 0xFFFF;

	return 0;
}



void ak98_pwm_disable(struct ak98_pwm *pwm)
{
	switch(pwm->id) {
		case 1:
			ak98_setpin_as_gpio(pwm->gpio);
			break;
		case 2:
			ak98_setpin_as_gpio(pwm->gpio);
			break;
		case 3:
			ak98_setpin_as_gpio(pwm->gpio);
			break;
		case 4:
			ak98_setpin_as_gpio(pwm->gpio);
			break;
		default:
			printk("init pwm error");
	}
}

EXPORT_SYMBOL(ak98_pwm_disable);


struct ak98_pwm *ak98_pwm_request(int pwm_id)
{
	switch(pwm_id) {
		case 1:
			return &ak98_pwm1;
		case 2:
			return &ak98_pwm2;
		case 3:
			return &ak98_pwm3;
		case 4:
			return &ak98_pwm4;
		default:
			printk("wrong pwm_id! should be between 1 and 4");
	}
	return NULL;
}

EXPORT_SYMBOL(ak98_pwm_request);

int ak98_pwm_config(struct ak98_pwm *pwm, unsigned short high, unsigned short low)
{
	return ak98_pwm_set_duty_cycle(pwm, high, low);
}
EXPORT_SYMBOL(ak98_pwm_config);

