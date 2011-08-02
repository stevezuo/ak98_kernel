#ifndef __AK98_BAT_H_
#define __AK98_BAT_H_ __FILE__

#include <mach/gpio.h>
#include <mach/adc1.h>

struct bat_gpio{

	int active;					/* active value */
	int irq;						/* use for ac in or out irq */
	struct gpio_info pindata;		/* battery gpios info */

};

struct bat_info{
	int charge_full_pin;	/* use for juge if battery charge full */ 									  
	int voltage_sample;		/* read how many voltge sample */	
	int power_down_level;	/* if (capacity<=power_down_level), power down */
	int max_voltage;		/* max battery voltage  */
    int min_voltage;		/* min battery voltage  */
    int full_capacity;
    int charge_max_time;
};

struct bat_ad4{
	unsigned int sample_rate;		/* the same as touch screen */
	unsigned int wait_time;			/* the same as touch screen */
	int up_resistance;				/* read ad4 voltage resistance */
	int dw_resistance;
	int voltage_correct;			/* correct voltage from adc1-4*/
	int adc_avdd;					/* adc avdd */
	int adc_poweroff;				/* poweroff adc value */
};

struct bat_charge_bight{
	int *capacity;
	int *time;
	int	length;
	int points;
};

struct bat_discharge_bight{
	int *capacity;
	int *voltage;
	int length;
	int points;
};

struct bat_charge_cv_bight{
	int *capacity;
	int *voltage;
	int length;
	int points;
};

struct ak98_bat_mach_info {

	void (* gpio_init) (const struct gpio_info *);
	struct bat_gpio usb_gpio;
	struct bat_gpio ac_gpio;
	struct bat_gpio state_gpio;
	struct bat_info bat_mach_info;
	struct bat_ad4  bat_adc;
	struct bat_charge_bight charge;
	struct bat_charge_cv_bight charge_cv;
	struct bat_discharge_bight discharge;
};

#endif				/* __AK98_BAT_H_ */


