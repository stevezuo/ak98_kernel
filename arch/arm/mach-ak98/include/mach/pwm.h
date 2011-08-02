/*
 */
#ifndef _AK98_PWM_H_
#define _AK98_PWM_H_ __FILE__

#define AK98_PWM1_CTRL	(AK98_VA_SYSCTRL+0x2C)
#define AK98_PWM2_CTRL	(AK98_VA_SYSCTRL+0x30)
#define AK98_PWM3_CTRL	(AK98_VA_SYSCTRL+0xB4)
#define AK98_PWM4_CTRL	(AK98_VA_SYSCTRL+0xB8)


struct ak98_pwm {
	unsigned int id;
	unsigned int gpio;
	unsigned char __iomem *pwm_ctrl;
	unsigned short high;
	unsigned short low;
	unsigned short pwm_clk;
};

struct ak98_platform_pwm_bl_data {
	int pwm_id;
	unsigned int max_brightness;
	unsigned int dft_brightness;
	unsigned int pwm_clk;
	int (*init)(struct ak98_pwm *dev);
	int (*notify)(int brightness);
	void (*exit)(struct ak98_pwm *dev);
};


extern struct ak98_pwm ak98_pwm1;
extern struct ak98_pwm ak98_pwm2;
extern struct ak98_pwm ak98_pwm3;
extern struct ak98_pwm ak98_pwm4;

int ak98_pwm_enable(struct ak98_pwm *pwm);
int ak98_pwm_config(struct ak98_pwm *pwm, unsigned short high, unsigned short low);
void ak98_pwm_disable(struct ak98_pwm *pwm);
struct ak98_pwm *ak98_pwm_request(int pwm_id);


#endif				/* _AK98_PWM_H_ */
