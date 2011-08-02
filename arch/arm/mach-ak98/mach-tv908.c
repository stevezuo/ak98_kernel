/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c/aw9523.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio_keys.h>
#include <linux/input/matrix_keypad.h>
#include <linux/input.h>
#include <linux/spi/spi.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/nand.h>
#include <mach/ts.h>
#include <mach/spi.h>
#include <mach/gpio.h>
#include <mach/reset.h>
#include <mach/devices.h>
#include <mach/ak98_mci.h>
#include <mach/regs-comm.h>
#include <mach/map.h>
#include <mach/lib_lcd.h>
#include <mach/rtc.h>

#include <mach/l2.h>

#include "cpu.h"
#include "irq.h"


void (*ak98_arch_reset)(void);

/* NAND parititon */
/* for each block=512K */
static struct mtd_partition tv908_evt_nand_part[] = {
[0] = {
		.name = "Bootloader",
		.size = SZ_512K,                 // 0 to 0
		.offset = 0,
		.mask_flags = MTD_WRITEABLE,
       },
[1] = {
		.name = "SYS",
		.size = SZ_256M , 			
 		.offset = MTDPART_OFS_APPEND,
 		},
[2] = {
		.name = "DATA",
 		.size = SZ_256M  , 		
   		.offset = MTDPART_OFS_APPEND,
 		},
[3] = {
		.name = "TEST",
 		.size = SZ_256M ,  		
  		.offset = MTDPART_OFS_APPEND,
		},
[4] = {
		.name = "GUI",              
		.size = MTDPART_SIZ_FULL,        
		.offset = MTDPART_OFS_APPEND,
		}
};

static struct ak98_nand_set tv908_evt_nand_sets[] = {
	[0] = {
	       .name = "NAND",
	       .nr_chips = 1,
	       .nr_partitions = ARRAY_SIZE(tv908_evt_nand_part),
	       .partitions = tv908_evt_nand_part,
	      // .cmd_len = 0xF5BD1,	//0x93791
	       .cmd_len = 0xF5AD1,
	       .data_len = 0xF5C5C,	//0x91517
	      // .data_len = 0x91717,
	       }
};

static struct ak98_platform_nand tv908_evt_nand_info = {
	.nr_sets = ARRAY_SIZE(tv908_evt_nand_sets),
	.sets = tv908_evt_nand_sets,
};

static struct i2c_board_info aw9523_ext_gpio_info[] = {
    {
        I2C_BOARD_INFO("aw9523", 0x58),
    },
};

struct platform_device ak98_ar7643ts_device = {
	.name = "ar7643-ts",
	.id = -1,
};

/*
 * GPIO Buttons
 */
static struct gpio_keys_button tv908_buttons[] = {
	{			
		.code		= KEY_BACK,
		.gpio		= AK98_GPIO_104,
		.active_low	= 0,
		.desc		= "btn_Back",
		.debounce_interval = 30,
		.wakeup		= 0,
	},
	{	
		.code		= KEY_MENU,
		.gpio		= AK98_GPIO_105,
		.active_low	= 0,
		.desc		= "btn_Menu",
		.debounce_interval = 30,
		.wakeup		= 1,
	},
	{	
		.code		= KEY_POWER,
		.gpio		= AK98_GPIO_13,
		.active_low	= 1,
		.desc		= "btn_powerdown",
		.debounce_interval = 30,
		.wakeup		= 0,
	}
};

static struct gpio_keys_platform_data tv908_button_data = {
	.buttons	= tv908_buttons,
	.nbuttons	= ARRAY_SIZE(tv908_buttons),
};

static struct platform_device tv908_button_device = {
	.name		= "gpio_keys",
	.id			= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &tv908_button_data,
	}
};

/* MMC/SD */

struct ak98_mci_platform_data tv908_mci_plat_data = {
	.gpio_cd  = AK98_GPIO_5,
	.gpio_wp  = -ENOSYS,
};

static struct resource tv908_mmc_resource[] = {
	[0] = {
		.start = 0x20020000,
		.end = 0x20020000 + 0x43,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MMC_SD,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tv908_mmc_device = {
	.name = "ak98_mci",
	.id = -1,
	.num_resources = ARRAY_SIZE(tv908_mmc_resource),
	.resource = tv908_mmc_resource,
	.dev = {
		.platform_data = &tv908_mci_plat_data,
	},
};

/* SDIO */
struct ak98_mci_platform_data tv908_sdio_plat_data = {
	.gpio_cd  = -ENOSYS,
	.gpio_wp  = -ENOSYS,
};

static struct resource tv908_sdio_resource[] = {
	[0] = {
		.start = 0x20021000,
		.end = 0x20021000 + 0x43,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device tv908_sdio_device = {
	.name = "ak98_sdio",
	.id = -1,
	.num_resources = ARRAY_SIZE(tv908_sdio_resource),
	.resource = tv908_sdio_resource,
	.dev = {
		.platform_data = &tv908_sdio_plat_data,
	},
};


/* platform devices */
static struct platform_device *tv908_evt_platform_devices[] __initdata = {
	//&ak98_led_device,
	&ak98_rtc_device,
	&ak98_uart0_device,
	&ak98_uart1_device,
	&ak98_uart3_device,
	&ak98_nand_device,
	&ak98_lcd_device,
	&ak98_mmx_device,
	&ak98_mmx_pmem,
	&ak98_i2c_device,
	&ak98_battery_power,
	&tv908_button_device,
	&tv908_mmc_device,
	&tv908_sdio_device,
	&ak98_usb_device,
	&ak98_usb_fs_hcd_device,
	&ak98pcm_device,
	&ak98_android_usb,
	&ak98_android_usb_mass_storage,
	&ak98_ar7643ts_device,
};

static void tv908_power_off(void)
{
	/*
	 * Do a real power off by polling WAKEUP pin to low
	 */
	ak98_rtc_set_wpin(0);
}

static void tv908_reset(void)
{
	ak98_reboot_sys();
}

static void __init tv908_evt_machine_init(void)
{
	pm_power_off = tv908_power_off;
	ak98_arch_reset = tv908_reset;
	
	ak98_nand_device.dev.platform_data = &tv908_evt_nand_info;
	i2c_register_board_info(0, aw9523_ext_gpio_info,
                    ARRAY_SIZE(aw9523_ext_gpio_info));
	
	/* register platform devices */
	platform_add_devices(tv908_evt_platform_devices,
		ARRAY_SIZE(tv908_evt_platform_devices));

	/* Initialize L2 buffer */
	ak98_l2_init();

}

//MACHINE_START(AK9805_TV908, "BOARD_AK9805TV908")
MACHINE_START(AK9805_TV908, "BOARD_AK9801ATHENA")
/* Maintainer: */
	.phys_io = 0x20000000,
	.io_pg_offst = ((0xF0200000) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.init_irq = ak98_init_irq,
	.map_io = ak98_map_io,
	.init_machine = tv908_evt_machine_init,
	.timer = &ak98_timer, 
MACHINE_END
