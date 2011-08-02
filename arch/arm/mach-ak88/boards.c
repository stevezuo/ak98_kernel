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

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

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
#include <mach/pm.h>

#include <linux/fb.h>
#include <video/anyka_lcdc.h>

#include <mach/map.h>
#include <mach/devices_ak880x.h>
#include <mach/lib_lcd.h>
#include <mach/ak880x_gpio.h>
 
#include <linux/delay.h>

#include <mach/l2.h>

#include "cpu.h"
#include "irq.h"
#include "devices.h"

  	#define DBG_UART_ID  0
	#define LCD_TFT      0

#ifdef CONFIG_BOARD_AK8801EPC 
 	#undef DBG_UART_ID  
	#undef LCD_TFT      

 	#define DBG_UART_ID  0
	#define LCD_TFT      0
#endif

#ifdef CONFIG_BOARD_AK8802EBOOK 
 	#undef DBG_UART_ID  
	#undef LCD_TFT      

	#define DBG_UART_ID  3
 	#define LCD_TFT      1
#endif

/*
 * LCD Controller
 */
//#if defined(CONFIG_FB_ANYKA) || defined(CONFIG_FB_ANYKA_MODULE)
#if defined(CONFIG_FB_AK88) || defined(CONFIG_FB_AK88_DEBUG)
static struct fb_videomode ak880x_tft_vga_modes[] = {
	{
 	 .name = "HSD070IDW1-A10",
	 .refresh = 60,
	 .xres = 800,.yres = 480,
	 .pixclock = 30000000,

	 .left_margin = 40,.right_margin = 40,
	 .upper_margin = 29,.lower_margin = 13,
	 .hsync_len = 48,.vsync_len = 3,

	 .sync = 0,
	 .vmode = FB_VMODE_NONINTERLACED,
	 },

	{
	 .name = "Q07021-701",
	 .refresh = 60,
	 .xres = 800,.yres = 600,
 	 .pixclock = 40000000,

	 .left_margin = 88,.right_margin = 112,
	 .upper_margin = 39,.lower_margin = 21,
	 .hsync_len = 48,.vsync_len = 3,

	 .sync = 0,
	 .vmode = FB_VMODE_NONINTERLACED,
	 },

};

static struct fb_monspecs ak880xfb_default_monspecs = {
	.manufacturer = "HIT",
	.monitor = "TX09D70VM1CCA",

	.modedb = &ak880x_tft_vga_modes[LCD_TFT],
	.modedb_len = ARRAY_SIZE(ak880x_tft_vga_modes),
	.hfmin = 15000,
	.hfmax = 64000,
	.vfmin = 50,
	.vfmax = 150,
};

static void ak880x_lcdc_power_control(int on)
{

#if 1
	if (on)
		//open LCD controller clock
		AKCLR_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C
	else
		//close LCD controller clock
		AKSET_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

	//panel power on
	bsplcd_set_panel_power(on);	//pullup TFT_VGH_L and TFT_AVDD

	//panel backlight
	baselcd_set_panel_backlight(on);	//will clear memory after memory setting

	printk("ak880x_lcdc_power_control(%d)\n", on);

#endif

}

/* Driver datas */
static struct anyka_lcdfb_info __initdata ak880x_lcdc_data = {
	.lcdcon_is_backlight = true,
	.default_bpp = 16,
	//.default_dmacon                       = ANYKA_LCDC_DMAEN,
	//.default_lcdcon2              = AK88_DEFAULT_LCDCON2,
	.default_monspecs = &ak880xfb_default_monspecs,
	.anyka_lcdfb_power_control = ak880x_lcdc_power_control,
	.guard_time = 1,
	//if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_RGB) {
	.lcd_wiring_mode = ANYKA_LCDC_WIRING_RGB,	//RGB:565 mode
};

#else
static struct anyka_lcdfb_info __initdata ak880x_lcdc_data;
#endif				//for defined(CONFIG_FB_ANYKA) || defined(CONFIG_FB_ANYKA_MODULE)

//#define SIZEMTD(x) (SZ_xM)

/* NAND parititon */
/* for each block=512K */
static struct mtd_partition ta801_nand_part[] = {
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

static struct ak880x_nand_set ta801_nand_sets[] = {
	[0] = {
	       .name = "NAND",
	       .nr_chips = 1,
	       .nr_partitions = ARRAY_SIZE(ta801_nand_part),
	       .partitions = ta801_nand_part,
	       //.cmd_len = 0xF5BD1,	//0x93791
	       .cmd_len = 0xF5AD1,
	       //.data_len = 0xF5C5C,	//0x91517
	       .data_len = 0x91717,
	       }
};

static struct ak880x_platform_nand ta801_nand_info = {
	.nr_sets = ARRAY_SIZE(ta801_nand_sets),
	.sets = ta801_nand_sets,
};

static struct ak880x_ts_mach_info ta801_ts_info = {
	.irq = IRQ_DGPIO_33,
	.irqpin = AK88_DGPIO_33,
	.sample_rate = 8000,
	.wait_time = 5,
};

static struct ak880x_spi_info ta801_spi_info;

/* platform devices */
static struct platform_device *ta801_platform_devices[] __initdata = {
	&ak880x_led_device,
	&ak880x_rtc_device,
	&ak880x_uart0_device,
	//&ak880x_uart1_device,
	//&ak880x_uart2_device,
	&ak880x_uart3_device,
	&ak880x_ts_device,
	&ak880x_gpio_trkball_device,
	&ak880x_kpd_device,
	&ak880x_nand_device,
	&ak880x_lcd_device,
	&ak880x_snd_device,
	&ak880x_spi0_device,
	&ak880x_i2c_device,
	&ak880x_mmc_device,
	&ak880x_usb_device,
};

static void ta801_shutdown_machine(void)
{
	ak880x_gpio_cfgpin(AK88_DGPIO_21, AK88_GPIO_OUT_0);
	ak880x_gpio_setpin(AK88_DGPIO_21, 0);
}

static void __init ta801_machine_init(void)
{

#ifdef CONFIG_BOARD_AK8802EBOOK

	unsigned int pin;

	//POWER_OFF
	//pin name VI_DATA2/GPIO53    //pull high
	pin = AK8802_GPIO_53;
	AKCLR_BITS(1UL << 24, AK88_SHAREPIN_CTRL);	//set pin as gpio[58:47]
	gpio_set_output(pin, 1);	//open,set to output and pull high
			
#endif
	
	ak880x_shutdown_machine = ta801_shutdown_machine;

	ak880x_nand_device.dev.platform_data = &ta801_nand_info;
	ak880x_ts_device.dev.platform_data = &ta801_ts_info;
	ak880x_spi0_device.dev.platform_data = &ta801_spi_info;

	printk("ta801_machine_init(),DBG_UART_ID=%d,LCD_TFT=%d\n",(int)DBG_UART_ID,(int)LCD_TFT);

	#if 0
	//* UART */
 	ak880x_register_uart(3,3);  
  	ak880x_set_serial_console(DBG_UART_ID);
	ak880x_register_uart(0,0);

	ak880x_add_device_serial();
	#endif

	/* LCD Controller */
	ak880x_add_device_lcdc(&ak880x_lcdc_data);

	/* register platform devices */
	platform_add_devices(ta801_platform_devices,
			     ARRAY_SIZE(ta801_platform_devices));

	ak880x_pm_init();

	/* Initialize L2 buffer */
	ak88_l2_init();
}

#ifdef CONFIG_BOARD_AK8801EPC
MACHINE_START(AK88, "BOARD_AK8801EPC")
#else
#ifdef CONFIG_BOARD_AK8802EBOOK
MACHINE_START(AK88, "BOARD_AK8802EBOOK")
#endif
#endif
/* Maintainer: */
	.phys_io = 0x30000000,
	.io_pg_offst = ((0xc0000000) >> 18) & 0xfffc,
	.boot_params = 0x30000100,
	.init_irq = ak880x_init_irq,
	.map_io = ak880x_map_io,
	.init_machine = ta801_machine_init,
	.timer = &ak880x_timer, 
MACHINE_END
