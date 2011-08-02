/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __AK88_IIC_H
#define __AK88_IIC_H __FILE__

/* Notes:
 *	1) All frequencies are expressed in Hz
 *	2) A value of zero is `do not care`
*/

struct ak880x_platform_i2c {
	unsigned int flags;
	unsigned int slave_addr;	/* slave address for controller */
	unsigned long bus_freq;	/* standard bus frequency */
	unsigned long max_freq;	/* max frequency for the bus */
	unsigned long min_freq;	/* min frequency for the bus */
	unsigned int sda_delay;	/* pclks (s3c2440 only) */
};

#endif				/* __AK88_IIC_H */
