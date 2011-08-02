/*
 * wifi.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _WIFI_H

#include <mach/gpio.h>

struct wifi_control_data {
	struct gpio_info gpio_on;
	struct gpio_info gpio_off;
	int power_on_delay;
	int power_off_delay;
};

struct ak98_wifi_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};


#endif /* _WIFI_H */

