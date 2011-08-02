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
#include <linux/input/ak98matrix_keypad.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/mxc622x.h>
#include <linux/mmc328x.h>

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
#include <mach/bat.h>
#include <mach/ak_sensor.h>

#include <linux/fb.h>
#include <video/anyka_lcdc.h>

#include <mach/map.h>
#include <mach/lib_lcd.h>
#include <mach/rtc.h>
#include <linux/delay.h>

#include <linux/i2c/cp2007.h>
#include <linux/dma-mapping.h>

#include <mach/ak98_hal.h>
#include <mach/l2.h>
#include <mach/regs-comm.h>
#include <mach/pwm.h>
#include <mach/wifi.h>
#include <mach/clock.h>
#include <mach/gpio_keys.h>
#include <linux/i2c/aw9523.h>
#include <media/soc_camera.h>
#include <media/hi253.h>
#include <media/ov772x.h>



#include "cpu.h"
#include "irq.h"


  	#define DBG_UART_ID  0
	#define LCD_TFT      0

void (*ak98_arch_reset)(void);

/*
 * LCD Controller
 */
//#if defined(CONFIG_FB_ANYKA) || defined(CONFIG_FB_ANYKA_MODULE)
#if defined(CONFIG_FB_AK98) || defined(CONFIG_FB_AK98_DEBUG)
static struct fb_videomode ak98_tft_vga_modes[] = {
	{
 	 .name = "HSD070IDW1-A10",
	 .refresh = 60,
	 .xres = 800,.yres = 480,
	 .pixclock = 30000000,

	 .left_margin = 40,.right_margin = 170,
	 .upper_margin = 29,.lower_margin = 40,
	 .hsync_len = 48,.vsync_len = 4,

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

static struct fb_monspecs ak98fb_default_monspecs = {
	.manufacturer = "HIT",
	.monitor = "TX09D70VM1CCA",

	.modedb = &ak98_tft_vga_modes[LCD_TFT],
	.modedb_len = ARRAY_SIZE(ak98_tft_vga_modes),
	.hfmin = 15000,
	.hfmax = 64000,
	.vfmin = 50,
	.vfmax = 150,
};

static void ak98_lcdc_power_control(int on)
{

#if 1
	if (on)
		//open LCD controller clock
		AKCLR_BITS(1UL << 3, rCLK_CON1);	//0x0800000C
	else
		//close LCD controller clock
		AKSET_BITS(1UL << 3, rCLK_CON1);	//0x0800000C

	//panel power on
	bsplcd_set_panel_power(on);	//pullup TFT_VGH_L and TFT_AVDD

	//panel backlight
	baselcd_set_panel_backlight(on);	//will clear memory after memory setting

	printk("ak98_lcdc_power_control(%d)\n", on);

#endif

}

/* Driver datas */
static struct anyka_lcdfb_info __initdata ak98_lcdc_data = {
	.lcdcon_is_backlight = true,
	.default_bpp = 16,
	//.default_dmacon                       = ANYKA_LCDC_DMAEN,
	//.default_lcdcon2              = AK88_DEFAULT_LCDCON2,
	.default_monspecs = &ak98fb_default_monspecs,
	.anyka_lcdfb_power_control = ak98_lcdc_power_control,
	.guard_time = 1,
	//if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_RGB) {
	.lcd_wiring_mode = ANYKA_LCDC_WIRING_RGB,	//RGB:565 mode
};

#else
static struct anyka_lcdfb_info __initdata ak98_lcdc_data;
#endif				//for defined(CONFIG_FB_ANYKA) || defined(CONFIG_FB_ANYKA_MODULE)


/* NAND parititon */
/* for each block=512K */
static struct mtd_partition athena_evt_nand_part[] = {
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

static struct ak98_nand_set athena_evt_nand_sets[] = {
	[0] = {
	       .name = "NAND",
	       .nr_chips = 1,
	       .nr_partitions = ARRAY_SIZE(athena_evt_nand_part),
	       .partitions = athena_evt_nand_part,
	      // .cmd_len = 0xF5BD1,	//0x93791
	       .cmd_len = 0xF5AD1,
	       .data_len = 0xF5C5C,	//0x91517
	      // .data_len = 0x91717,
	       }
};

static struct ak98_platform_nand athena_evt_nand_info = {
	.nr_sets = ARRAY_SIZE(athena_evt_nand_sets),
	.sets = athena_evt_nand_sets,
};

/*
 * AK98 matrix Keyboard Device
 */
#define KEY_CENTER	KEY_REPLY
static const uint32_t athena_keypad_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_MENU),
	KEY(1, 0, KEY_UP),
	KEY(1, 1, KEY_CENTER),
	KEY(1, 2, KEY_DOWN),
	KEY(2, 0, KEY_VOLUMEDOWN),
	KEY(2, 1, KEY_LEFT),
	KEY(2, 2, KEY_HOME),
};

static struct matrix_keymap_data athena_keypad_keymap_data = {
	.keymap		= athena_keypad_keymap,
	.keymap_size	= ARRAY_SIZE(athena_keypad_keymap),
};

/*
{ AK98_GPIO_106, AK98_GPIO_107, AK98_GPIO_109 };
{ AK98_GPIO_2, AK98_GPIO_3, AK98_GPIO_4 };

*/
static const int athena_keypad_row_gpios[] = 
	//{ AW9523_GPIO_P00, AW9523_GPIO_P01, AW9523_GPIO_P02 };
	{ AK98_GPIO_106, AK98_GPIO_107, AK98_GPIO_109 };
static const int athena_keypad_col_gpios[] = 
	//{ AW9523_GPIO_P11, AW9523_GPIO_P12, AW9523_GPIO_P13 };
	{ AK98_GPIO_2, AK98_GPIO_3, AK98_GPIO_4 };

static struct matrix_keypad_platform_data athena_keypad_pdata = {
	.keymap_data	= &athena_keypad_keymap_data,
	.row_gpios		= athena_keypad_row_gpios,
	.col_gpios		= athena_keypad_col_gpios,
	.num_row_gpios		= ARRAY_SIZE(athena_keypad_row_gpios),
	.num_col_gpios		= ARRAY_SIZE(athena_keypad_col_gpios),
	.col_scan_delay_us	= 10,
	.debounce_ms		= 30,
	.active_low		= 1,
	.wakeup			= 0,
	.row_gpios_cfginfo = {
				.pin 		= -1,
				.pulldown	= -1,
				.pullup		= -1,
				.dir		= AK98_GPIO_DIR_INPUT,
				.value		= -1,
				.int_pol	= AK98_GPIO_INT_LOWLEVEL,
		},
	.col_gpios_cfginfo = {
				.pin		= -1,
				.pulldown	= -1,
				.pullup		= -1,
				.dir 		= AK98_GPIO_DIR_OUTPUT,
				.value		= AK98_GPIO_OUT_LOW, /* default state of output gpios*/
				.int_pol	= -1,
		},
	.grounding		= false, /* grounding line should be the end line*/
};

static struct platform_device athena_keypad_device = {
	.name		= "matrix-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &athena_keypad_pdata,
	},
};




/*
 * GPIO Buttons
 */
static struct ak98_gpio_keys_button athena_buttons[] = {
	{			
		.code		= KEY_ESC,
		.gpio		= AK98_GPIO_73,
		.active_low	= 0,
		.desc		= "btn_ESC",
		.debounce_interval = 30,
		.wakeup		= 0,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	{			
		.code		= KEY_PAGEUP,
		.gpio		= AK98_GPIO_72,
		.active_low	= 0,
		.desc		= "btn_PAGEUP",
		.debounce_interval = 30,
		.wakeup		= 0,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	{			
		.code		= KEY_PAGEDOWN,
		.gpio		= AK98_GPIO_74,
		.active_low	= 0,
		.desc		= "btn_PAGEDOWN",
		.debounce_interval = 30,
		.wakeup		= 0,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	{	
		.code		= KEY_MENU,
		.gpio		= AK98_GPIO_104,
		.active_low	= 0,
		.desc		= "btn_Menu",
		.debounce_interval = 30,
		.wakeup		= 0,
		.pullup		= -1,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	{	
		.code		= KEY_POWER,
		.gpio		= AK98_GPIO_7,
		.active_low	= 1,
		.desc		= "btn_powerdown",
		.debounce_interval = 30,
		.wakeup		= 1,
		.pulldown	= AK98_PULLDOWN_DISABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_LOWLEVEL,
	},
	
};

static struct ak98_gpio_keys_platform_data athena_button_data = {
	.buttons	= athena_buttons,
	.nbuttons	= ARRAY_SIZE(athena_buttons),
};

	
static struct platform_device athena_button_device = {
	.name		= "gpio_keys",
	.id			= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &athena_button_data,
	}
};

/* MMC/SD */

#ifdef CONFIG_FOUR_DATA_LINE
struct ak98_mci_platform_data mci_plat_data = {
	.gpio_cd  = AK98_GPIO_75,
	.gpio_wp  = -ENOSYS,
};

static struct resource ak98_mmc_resource[] = {
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

#elif defined CONFIG_EIGHT_DATA_LINE
struct ak98_mci_platform_data mci_plat_data = {
	.gpio_cd  = AK98_GPIO_75,
	.gpio_wp  = -ENOSYS,
};

static struct resource ak98_mmc_resource[] = {
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
#else
struct ak98_mci_platform_data mci_plat_data = {

};

static struct resource ak98_mmc_resource[] = {

};
#endif

struct platform_device ak98_mmc_device = {
	.name = "ak98_mci",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_mmc_resource),
	.resource = ak98_mmc_resource,
	.dev = {
		.platform_data = &mci_plat_data,
	},
};


/* SDIO */
struct ak98_mci_platform_data sdio_plat_data = {
	.gpio_cd  = AK98_GPIO_18,
	.gpio_wp  = -ENOSYS,
};

static struct resource ak98_sdio_resource[] = {
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

struct platform_device ak98_sdio_device = {
	.name = "ak98_sdio",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_sdio_resource),
	.resource = ak98_sdio_resource,
	.dev = {
		.platform_data = &sdio_plat_data,
	},
};

static int bl_init(struct ak98_pwm *dev)
{	
	int ret = ak98_pwm_enable(dev);
	if (ret == 0)
		ak98_gpio_pulldown(dev->gpio, AK98_PULLDOWN_DISABLE);
	
	return ret;
}

static void bl_exit(struct ak98_pwm *dev)
{
	ak98_pwm_disable(dev);
}

struct ak98_platform_pwm_bl_data ak98pwm_backlight_data = {
	.pwm_id = 1,
		.max_brightness = 255,
		.dft_brightness = 200,
#if defined CONFIG_LCD_PANEL_QD043003C0_40
		.pwm_clk = 4000,
#elif defined CONFIG_LCD_PANEL_LW700AT9009
		.pwm_clk = 6000,
#else
		.pwm_clk = 1024,
#endif

		.init = bl_init,
		.notify = NULL,
		.exit = bl_exit,
};

/* ts */
static void ts_init_hwinit(const struct cp2007_intpin_info *intpin_info)
{
	//as gpio
	ak98_setpin_as_gpio(intpin_info->pin);
	//input mode
	ak98_gpio_cfgpin(intpin_info->pin, intpin_info->dir);
	// pulldown/pullup setting
	if (intpin_info->pulldown == AK98_PULLDOWN_DISABLE || intpin_info->pulldown == AK98_PULLDOWN_ENABLE)
		ak98_gpio_pulldown(intpin_info->pin, intpin_info->pulldown);
	if (intpin_info->pullup == AK98_PULLUP_DISABLE || intpin_info->pullup == AK98_PULLUP_ENABLE)
		ak98_gpio_pullup(intpin_info->pin, intpin_info->pullup);
	//active low
	ak98_gpio_intpol(intpin_info->pin, intpin_info->int_pol);
}

static int ts_is_pen_down(unsigned int pin)
{
	unsigned int ret;	

	ret = ak98_gpio_getpin(pin);

	return ret ? 0 : 1;
}


struct cp2007_ts_platform_data cp2007_ts_info = {
	.origin_pos 	= ORIGIN_TOPLEFT,/* determined by the way the touch screen chip is connected*/
	.x_plate_ohms	= 252,
	.is_pen_down	= ts_is_pen_down,
	.init_ts_hw		= ts_init_hwinit,
	.intpin_info	= {
			.pin 		= AK98_GPIO_6,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.pullup		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= AK98_GPIO_INT_LOWLEVEL,
		},
};

static struct i2c_board_info ak98_ts_devices[] __initdata = {
	{
		I2C_BOARD_INFO("cp2007_ts", 0x48),
		.type		= "cp2007_ts",
		.platform_data	= &cp2007_ts_info,
		.irq		= IRQ_GPIO_6,
	},
};



static struct ak98_ts_mach_info ak98adc_ts_info = {
	.irq = IRQ_GPIO_107,
	.irqpin = AK98_GPIO_107,
	.sample_rate = 800,
	.wait_time = 5,
};

struct wifi_control_data athena_wifi_control_data = {
	.gpio_on = {
		.pin		= AK98_GPIO_3,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.pullup		= -1,
		.value		= AK98_GPIO_HIGH,
		.dir		= AK98_GPIO_DIR_OUTPUT,
		.int_pol	= -1,
	},
	.gpio_off = {
		.pin		= AK98_GPIO_3,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.pullup		= -1,
		.value		= AK98_GPIO_LOW,
		.dir		= AK98_GPIO_DIR_OUTPUT,
		.int_pol	= -1,
	},
	.power_on_delay   = 2000,
	.power_off_delay  = 0,
};

static int athena_rtl8188_wifi_power_on(void)
{
	struct wifi_control_data *p = &athena_wifi_control_data;

	if (p->gpio_on.pin > 0) {
			ak98_gpio_set(&p->gpio_on);
	}

	msleep(p->power_on_delay);
	printk("RTL8188: Power ON.\n");

	return 0;
}

static int athena_rtl8188_wifi_power_off(void)
{
	struct wifi_control_data *p = &athena_wifi_control_data;

	if (p->gpio_off.pin > 0) {
			ak98_gpio_set(&p->gpio_off);
	}

	msleep(p->power_off_delay);
	printk("RTL8188: Power OFF.\n");

	return 0;
}

static struct ak98_wifi_platform_data athena_rtl8188_wifi_info = {
	.power_on	= athena_rtl8188_wifi_power_on,
	.power_off	= athena_rtl8188_wifi_power_off,
};

struct platform_device ak98_wifi_rtl8188_device = {
	.name = "rtl8188",
	.id = -1,
	.dev = {
		.platform_data = &athena_rtl8188_wifi_info,
	},
};

/*ak98 battery mach info*/

// sample of battery charge capacity and time,(capacity,time)
static int charge_capacity[] 	= {0,70,90,96, 100};
static int charge_times[]	 	= {0,45,70,105,180};

// sample of battery charge capacity and voltage,(capacity,voltage)
static int charge_capacity_voltage[] 	= {0,     2,	  8,    95,    100};
static int charge_voltage[]		 		= {3300*2,3400*2,3650*2,4000*2,4100*2};


// sample of battery discharge capacity and voltage,(capacity,voltage)
static int discharge_capacity[]	 = {0,     2,	  8,     95,    100};
static int discharge_voltage[]	 = {3300*2,3400*2,3650*2,4000*2,4100*2};

static struct ak98_bat_mach_info ak98_bat_info = {
	.gpio_init	= ak98_gpio_set,
	
	.usb_gpio	= {
		.active 	= AK98_GPIO_HIGH,
		.irq		= IRQ_GPIO_110, 
		.pindata	={
			.pin		= AK98_GPIO_110,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.pullup 	= -1,
			.value		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= -1,
		},	
	},
	
	.ac_gpio	= { 
		.active 	= AK98_GPIO_HIGH,
		.irq		= IRQ_GPIO_5, 
		.pindata	={
			.pin		= AK98_GPIO_5,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.pullup 	= -1,
			.value		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= -1,
		},	
	},

	.state_gpio = {
		.active 	= AK98_GPIO_HIGH,
		.irq		= -ENOSYS, 
		.pindata	={
			.pin		= AK98_GPIO_71,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.pullup 	= -1,
			.value		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= -1,
		},	
	},
	
	.bat_mach_info	= {
		.voltage_sample 	= 6,			// the sample of read voltage
		.power_down_level	= 5,			// power down if(capacity <= power_down_level) 
		.max_voltage		= 4200 * 2,		// max battery voltage	
		.min_voltage		= 3300 * 2,		// min battery voltage	
		.full_capacity		= 100,
		.charge_max_time	= 180,
	},
	
	.bat_adc	= {
		.sample_rate		= 800,			// the same as touch screen
		.wait_time			= 5,			// the same as touch screen
		.up_resistance		= 47,
		.dw_resistance		= 20,
		.voltage_correct	= 65,			// battery correct factor
		.adc_avdd			= 3300,
	},

	.charge		={
		.capacity	= charge_capacity,
		.time		= charge_times,
		.length		= sizeof(charge_capacity),
		.points		= sizeof(charge_capacity) / sizeof(charge_capacity[0]),
	},

	.charge_cv	={
		.capacity	= charge_capacity_voltage,
		.voltage	= charge_voltage,
		.length 	= sizeof(charge_capacity_voltage),
		.points 	= sizeof(charge_capacity_voltage) / sizeof(charge_capacity_voltage[0]),
	},

	.discharge 	={
		.capacity	= discharge_capacity,
		.voltage	= discharge_voltage,
		.length		= sizeof(discharge_capacity),
		.points		= sizeof(discharge_capacity) / sizeof(discharge_capacity[0]),
	},
};

static void check_poweroff_init(void)
{
	//NULL
}

void check_poweroff(void)
{
	//NULL
}

/*end ak98 battery mach info*/

void spi_pin_setup(struct ak98_spi_info *spi, int enable)
{
	if (enable)
	{
		ak98_group_config(ePIN_AS_SPI1);
	}
	else
	{
		ak98_setpin_as_gpio(AK98_GPIO_76);
		ak98_setpin_as_gpio(AK98_GPIO_77);
		ak98_setpin_as_gpio(AK98_GPIO_78);
		ak98_setpin_as_gpio(AK98_GPIO_79);
	}
}

struct ak98_spi_info ak98_spi1_info = {
	.num_cs = 1,
	.bus_num = 1,
	.gpio_setup = spi_pin_setup,
	.clk_name = "spi1_clk",
	.mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH,
	.mode = 0,
};

struct spi_board_info spidev_info = {
	.modalias = "spidev",
	.bus_num = 1,
	.chip_select = 0,
	.mode = SPI_MODE_0,	
};

struct spi_board_info ar7643_info = {
	.modalias = "ar7643-spi",
	.bus_num = 1,
	.chip_select = 0,
	.mode = SPI_MODE_0,	
	.max_speed_hz = 1000000,
};

/* sensor */
struct platform_device ak_sensor_device = {
	.name = "ak_sensor",
	.id = -1,
};

#undef	GSENSOR_EXIST
#undef	MSENSOR_EXIST
#undef	OSENSOR_EXIST
#define GSENSOR_EXIST	(1)
#define MSENSOR_EXIST	(1)
#if (GSENSOR_EXIST && MSENSOR_EXIST)
#define OSENSOR_EXIST	(1)
#else
#define OSENSOR_EXIST	(0)
#endif

static struct sensor_t ak98_sensors[] = {
#ifdef CONFIG_SENSORS_MXC622X
	{
		.name		= "MXC622X",
		.vendor		= "MEMSIC",
		.type		= SENSOR_ACCELEROMETER,	
		.maxRange	= "20",
		.resolution	= "1531",
		.power		= "5",
		.dir		= "2",
		.exist		= GSENSOR_EXIST,
	},
#endif
#ifdef CONFIG_SENSORS_MMC328X
	{
		.name		= "MMC328X",
		.vendor		= "MEMSIC",
		.type		= SENSOR_MAGNETIC_FIELD,	
		.maxRange	= "40",
		.resolution	= "1953",
		.power		= "3",
		.dir		= "7",
		.exist		= MSENSOR_EXIST,
	},
#endif
	{
		.name		= "Mag & Acc Combo Orientation Sensor",
		.vendor		= "MEMSIC",
		.type		= SENSOR_ORIENTATION,
		.maxRange	= "20",
		.resolution	= "10000",
		.power		= "0",
		.exist		= OSENSOR_EXIST,
	},
	{
	},
};

static struct sensor_platform_data sensor_info = {
	.sensors	= ak98_sensors,
};
 
#ifdef CONFIG_SENSORS_MXC622X
static struct mxc622x_platform_data mxc622x_pdata = {
	.hw_init	= ak98_gpio_set,
	.irq		= IRQ_GPIO_109,
	.gpio		= {
		.pin		= AK98_GPIO_109,
		.pulldown	= AK98_PULLDOWN_DISABLE,
		.pullup		= AK98_PULLUP_ENABLE,
		.value		= -1,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= -1,
	}, 
	.exist = GSENSOR_EXIST,
};
#endif

static struct i2c_board_info ak98_sensor_devices[] __initdata = {
#ifdef CONFIG_SENSORS_MXC622X
	{
		I2C_BOARD_INFO(MXC622X_I2C_NAME, MXC622X_I2C_ADDR),
		.platform_data = &mxc622x_pdata,
	},
#endif
#ifdef CONFIG_SENSORS_MMC328X
	{
		I2C_BOARD_INFO(MMC328X_I2C_NAME, MMC328X_I2C_ADDR),
	},
#endif
	{
	},
};


/* aw9523 GPIO expander */
struct gpio_info aw9523_gpios[] = {
	{
		.pin 		= AK98_GPIO_106,
		.pulldown 	= AK98_PULLDOWN_DISABLE,
		.pullup 	= -1,
		.dir		= AK98_GPIO_DIR_INPUT,
		.value 		= -1,
		.int_pol	= AK98_GPIO_INT_LOWLEVEL,
	},		
};

struct aw9523_platform_data aw9523_info = {
	.gpio_base 	= AW9523_GPIO_P00,
	.irq_base	= IRQ_AW9523_P00,
	.parent_irq	= IRQ_GPIO_106,
	.p0xmod		= AW9523_MOD_PUSHPULL,
	.gpios 		= aw9523_gpios,
	.npins		= ARRAY_SIZE(aw9523_gpios),
};

static struct i2c_board_info ak98_aw9523_ext[] __initdata = {
	{
		I2C_BOARD_INFO("aw9523_ext", 0x58),
		.type	= "aw9523_ext",
		.irq	= IRQ_GPIO_106,
		.platform_data = &aw9523_info,
	},
};

struct ak98pcm_platform_data ak98pcm_plat_data =
{
	.hpdet_gpio =
	{
		.pin        = AK98_GPIO_115,
		.dir		= AK98_GPIO_DIR_INPUT,
		.pullup		= AK98_PULLUP_ENABLE,
		.pulldown	= AK98_PULLDOWN_DISABLE,
		.value      = -1,
		.int_pol	= -1,
	},
	.spk_down_gpio =
	{
		.pin        = AK98_GPIO_105,
		.dir		= AK98_GPIO_DIR_OUTPUT,
		.pullup		= -1,
		.pulldown	= AK98_PULLDOWN_ENABLE,
		.value      = AK98_GPIO_LOW,
		.int_pol	= -1,
	},
	.hpmute_gpio =
	{
		.pin        = AK98_GPIO_1,
		.dir		= AK98_GPIO_DIR_OUTPUT,
		.pullup		= AK98_PULLUP_DISABLE,
		.pulldown	= -1,
		.value      = AK98_GPIO_LOW,
		.int_pol	= -1,
	},

	.hp_on_value          = AK98_GPIO_LOW,
	.hpdet_irq            = IRQ_GPIO_115,
	.bIsHPmuteUsed        = 0,
	.hp_mute_enable_value = AK98_GPIO_HIGH,
	.bIsMetalfixed        = 0,
};
struct resource akpcm_resources[] = {
	[0] = {
			.start = 0x08000000,
			.end = 0x0800FFFF,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak98pcm_AnalogCtrlRegs",
		  },
	[1] = {
			.start = 0x2002E000,
			.end = 0x2002E00F,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak98pcm_I2SCtrlRegs",
		  },
	[2] = {
			.start = 0x2002D000,
			.end = 0x2002D00F,
			.flags = (int)IORESOURCE_MEM,
			.name = "ak98pcm_ADC2ModeCfgRegs",
		  },
};
static u64 snd_dma_mask = DMA_BIT_MASK(32);

struct platform_device ak98pcm_device = {
	.name = "snd_ak98pcm",
	.id = 0,
	
	.dev = {
			 .dma_mask	   = &snd_dma_mask,
			 .coherent_dma_mask = DMA_BIT_MASK(32),
			 .platform_data = &ak98pcm_plat_data,
		   },	
	.resource = akpcm_resources,
	.num_resources = ARRAY_SIZE(akpcm_resources),
};

/* for sensor power control in Athena*/ 
static int hi253_camera_power(struct device *dev, int on)
{
	if (on) {
		printk(KERN_DEBUG "ov772x power on:\n");
		ak98_gpio_cfgpin(AK98_GPIO_104, AK98_GPIO_DIR_OUTPUT);
		ak98_gpio_setpin(AK98_GPIO_104, AK98_GPIO_HIGH);			
		mdelay(10);		
		ak98_gpio_setpin(AK98_GPIO_104, AK98_GPIO_LOW);	

		/*reset the camera */
		ak98_gpio_cfgpin(AK98_GPIO_102, AK98_GPIO_DIR_OUTPUT);
//		ak98_gpio_setpin(AK98_GPIO_102, AK98_GPIO_HIGH);	
	
		ak98_gpio_setpin(AK98_GPIO_102, AK98_GPIO_LOW);	
		mdelay(100);
		ak98_gpio_setpin(AK98_GPIO_102, AK98_GPIO_HIGH);
	} else {
		printk(KERN_DEBUG "ov772x power down(not off):\n");
	
		ak98_gpio_setpin(AK98_GPIO_104, AK98_GPIO_HIGH); 
	}
	
	return 0;
}

struct i2c_board_info ak98_camara_devices[] = {
	{
		I2C_BOARD_INFO("hi253", 0x20),
	},
};

#if 0
struct i2c_board_info ak98_camara_devices[] = {
	{
		I2C_BOARD_INFO("ov772x", 0x21), 		
	},
};
#endif

static struct hi253_camera_info  ak98_hi253_camera_info = {
	.buswidth = SOCAM_DATAWIDTH_8,
	.link = {
		.bus_id = 98,
		.power = hi253_camera_power,
		.board_info = &ak98_camara_devices[0],
		.i2c_adapter_id = 0,		
	}	
};

#if 0
static struct ov772x_camera_info  ak98_ov772x_camera_info = {
	.buswidth = SOCAM_DATAWIDTH_8,
	.link = {
		.bus_id = 98,
		.power = hi253_camera_power,
		.board_info = &ak98_camara_devices[0],
		.i2c_adapter_id = 0,		
	}	
};
#endif

/* fake device for soc_camera subsystem */
static struct platform_device soc_camera_interface = {
	.name = "soc-camera-pdrv",
	.id   = -1,
	.dev = {
		.platform_data = &ak98_hi253_camera_info.link,
	}
};


/* platform devices */
static struct platform_device *athena_evt_platform_devices[] __initdata = {
	//&ak98_led_device,
	&ak98_rtc_device,
	&ak98_uart0_device,
	&ak98_uart1_device,
	&ak98_uart2_device,
	&ak98_uart3_device,
	//&ak98_gpio_trkball_device,
	//&ak98_kpd_device,
	&ak98_nand_device,
	&ak98_lcd_device,
	&ak98_mmx_device,
	&ak98_mmx_pmem,
	//&ak98_snd_device,
	&ak98_spi1_device,
	&ak98_i2c_device,
	&ak98_battery_power,
	&athena_button_device,
	&athena_keypad_device,
	&ak98_mmc_device,
	&ak98_sdio_device,
	&ak98_usb_device,
	&ak98_usb_fs_hcd_device,
	&ak98_mac_device,
	&ak98pcm_device,
	&ak98_android_usb,
	&ak98_android_usb_mass_storage,
	&ak98adc_ts_device,
	&ak98pwm_backlight_device,
	&ak98_wifi_rtl8188_device,
	&ak98_freq_policy_device,
	&ak_sensor_device,
	&soc_camera_interface,	
	&ak98_camera_interface,	
};

static void athena_power_off(void)
{
	/*
	 * Do a real power off by polling WAKEUP pin to low
	 */
	ak98_rtc_set_wpin(0);
	ak98_gpio_cfgpin(AK98_GPIO_10, AK98_GPIO_DIR_INPUT);
	ak98_gpio_pullup(AK98_GPIO_10, AK98_PULLUP_DISABLE);
}

static void athena_reset(void)
{
	ak98_reboot_sys_by_wtd();
}

static void __init athena_evt_machine_init(void)
{
	pm_power_off = athena_power_off;
	ak98_arch_reset = athena_reset;

	ak98_nand_device.dev.platform_data = &athena_evt_nand_info;
	i2c_register_board_info(0, ak98_ts_devices, ARRAY_SIZE(ak98_ts_devices));
	i2c_register_board_info(0, ak98_sensor_devices, ARRAY_SIZE(ak98_sensor_devices));
	i2c_register_board_info(0, ak98_aw9523_ext, ARRAY_SIZE(ak98_aw9523_ext));
	ak_sensor_device.dev.platform_data = &sensor_info;
   
	ak98adc_ts_device.dev.platform_data = &ak98adc_ts_info; 
  	// set battery platform_data
	ak98_battery_power.dev.platform_data = &ak98_bat_info;
	ak98pwm_backlight_device.dev.platform_data = &ak98pwm_backlight_data;
  
	spidev_info.max_speed_hz = ak98_get_asic_clk()/2;	   
	//spi_register_board_info(&spidev_info, 1);
	spi_register_board_info(&ar7643_info, 1);
	ak98_spi1_device.dev.platform_data = &ak98_spi1_info;
	/* register platform devices */
	platform_add_devices(athena_evt_platform_devices,
		ARRAY_SIZE(athena_evt_platform_devices));

	/* Initialize L2 buffer */
	ak98_l2_init();

}

MACHINE_START(AK9801_ATHENA, "BOARD_AK9801ATHENA")

/* Maintainer: */
	.phys_io = 0x20000000,
	.io_pg_offst = ((0xF0200000) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.init_irq = ak98_init_irq,
	.map_io = ak98_map_io,
	.init_machine = athena_evt_machine_init,
	.timer = &ak98_timer, 
MACHINE_END
