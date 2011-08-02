
#include <asm/io.h>
#include <mach/map.h>

#ifndef __ARCH_ARM_MACH_AK88_DEVICE_H__
#define __ARCH_ARM_MACH_AK88_DEVICE_H__

//include <linux/device.h>
//include <linux/mod_devicetable.h>
//include <linux/platform_device.h>


struct ak88_mci_platform_data {
	int gpio_cd;		/* card detect pin */
	int gpio_wp;		/* write protect pin */
};

#define AK88_MAX_UART   4

extern struct platform_device *ak880x_default_console_device;

void __init ak880x_register_uart(unsigned char id, unsigned char portnr);

void __init ak880x_set_serial_console(unsigned char portnr);

void __init ak880x_add_device_serial(void);

void __init ak880x_add_device_lcdc(struct anyka_lcdfb_info *data);

#endif
