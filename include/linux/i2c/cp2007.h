#ifndef __LINUX_I2C_CP2007_H
#define __LINUX_I2C_CP2007_H

#include <linux/swab.h>

enum {
	ORIGIN_TOPLEFT = 1,
	ORIGIN_BOTTOMLEFT,
	ORIGIN_TOPRIGHT,
	ORIGIN_BOTTOMRIGHT,
};

struct cp2007_intpin_info {
	unsigned int pin;
	char pulldown;  //pulldown function flag
	char pullup;    //pullup function flag
	char dir;       //direction  input/output
	char int_pol;   //interrupt polarity
};

struct cp2007_ts_platform_data {
	u16	x_plate_ohms;
	char origin_pos;
	struct cp2007_intpin_info intpin_info;
	int	(*is_pen_down)(unsigned int pin);
	void (*clear_penirq)(void);		/* If needed, clear 2nd level interrupt source */
	void (*init_ts_hw)(const struct cp2007_intpin_info *intpin_info);
	void (*exit_ts_hw)(void);
};


#endif /* __LINUX_I2C_CP2007_H */
