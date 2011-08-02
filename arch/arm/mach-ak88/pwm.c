/* 
 * AK88 PWM controler driver 
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

struct ak880x_pwm ak880x_pwm0 = {
	.id = 0,
	.gpio = AK88_GPIO_9,
	.pwm_ctrl = AK88_PWM0_CTRL,
};

struct ak880x_pwm ak880x_pwm1 = {
	.id = 1,
	.gpio = AK88_GPIO_10,
	.pwm_ctrl = AK88_PWM1_CTRL,
};

struct ak880x_pwm ak880x_pwm2 = {
	.id = 2,
	.gpio = AK88_GPIO_11,
	.pwm_ctrl = AK88_PWM2_CTRL,
};

struct ak880x_pwm ak880x_pwm3 = {
	.id = 3,
	.gpio = AK88_GPIO_12,
	.pwm_ctrl = AK88_PWM3_CTRL,
};

unsigned int ak880x_pwm_init(struct ak880x_pwm *pwm)
{
	__raw_writel(0xFFFF0000, pwm->pwm_ctrl);

	/* init gpio for pwm */
	ak880x_sharepin_cfg1(1, pwm->id + 4);

	/* disable pullup */
	ak880x_gpio_pullup(pwm->gpio, 1);

	return 0;
}

EXPORT_SYMBOL(ak880x_pwm_init);

unsigned int ak880x_pwm_set_duty_cycle(struct ak880x_pwm *pwm)
{
	__raw_writel(pwm->high << 16 | pwm->low, pwm->pwm_ctrl);
	return 0;
}

EXPORT_SYMBOL(ak880x_pwm_set_duty_cycle);

unsigned int ak880x_pwm_get_duty_cycle(struct ak880x_pwm *pwm)
{
	unsigned long regval;

	regval = __raw_readl(pwm->pwm_ctrl);

	pwm->high = regval >> 16;
	pwm->low = regval & 0xFFFF;

	return 0;
}

EXPORT_SYMBOL(ak880x_pwm_get_duty_cycle);

unsigned int ak880x_pwm_deinit(struct ak880x_pwm *pwm)
{
	/* deinit gpio for pwm */
	ak880x_sharepin_cfg1(0, pwm->id + 4);

	/* FIXME: enable pullup */
	ak880x_gpio_pullup(pwm->gpio, 0);

	return 0;
}

EXPORT_SYMBOL(ak880x_pwm_deinit);
