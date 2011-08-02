/*
 */
#ifndef _AK88_PWM_H_
#define _AK88_PWM_H_ __FILE__

#define AK88_PWM0_CTRL	(AK88_VA_SYSCTRL+0x2C)
#define AK88_PWM1_CTRL	(AK88_VA_SYSCTRL+0x30)
#define AK88_PWM2_CTRL	(AK88_VA_SYSCTRL+0xB4)
#define AK88_PWM3_CTRL	(AK88_VA_SYSCTRL+0xB8)

struct ak880x_pwm {
	unsigned int id;
	unsigned int gpio;
	unsigned char __iomem *pwm_ctrl;
	unsigned long high;
	unsigned long low;
};

extern struct ak880x_pwm ak880x_pwm0;
extern struct ak880x_pwm ak880x_pwm1;
extern struct ak880x_pwm ak880x_pwm2;
extern struct ak880x_pwm ak880x_pwm3;

unsigned int ak880x_pwm_init(struct ak880x_pwm *pwm);
unsigned int ak880x_pwm_set_duty_cycle(struct ak880x_pwm *pwm);
unsigned int ak880x_pwm_get_duty_cycle(struct ak880x_pwm *pwm);
unsigned int ak880x_pwm_deinit(struct ak880x_pwm *pwm);

#endif				/* _AK88_PWM_H_ */
