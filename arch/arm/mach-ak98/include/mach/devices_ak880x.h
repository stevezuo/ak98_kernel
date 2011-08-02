
#include <asm/io.h>
#include <mach/map.h>

#ifndef __ARCH_ARM_MACH_AK88_DEVICE_H__
#define __ARCH_ARM_MACH_AK88_DEVICE_H__

//include <linux/device.h>
//include <linux/mod_devicetable.h>
//include <linux/platform_device.h>

#define AK98_MAX_UART   4

extern struct platform_device *ak98_default_console_device;

void __init ak98_register_uart(unsigned char id, unsigned char portnr);

void __init ak98_set_serial_console(unsigned char portnr);

void __init ak98_add_device_serial(void);

void __init ak98_add_device_lcdc(struct anyka_lcdfb_info *data);

#endif
