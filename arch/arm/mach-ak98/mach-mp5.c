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
#include <mach/regs-comm.h>
#include <mach/bat.h>
#include <mach/ak_sensor.h>
#include <mach/map.h>
#include <mach/lib_lcd.h>
#include <mach/rtc.h>

#include <linux/i2c/cp2007.h>
#include <linux/dma-mapping.h>
#include <mach/ak98_hal.h>
#include <mach/l2.h>
#include <mach/pwm.h>
#include <mach/wifi.h>
#include <mach/gpio_keys.h>
#include <media/soc_camera.h>
#include <media/hi253.h>
#include <media/ov772x.h>


#include "cpu.h"
#include "irq.h"



void (*ak98_arch_reset)(void);
static void mp5_power_off(void);


/* NAND parititon */
/* for each block=512K */
static struct mtd_partition mp5_nand_part[] = {
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

static struct ak98_nand_set mp5_nand_sets[] = {
	[0] = {
	       .name = "NAND",
	       .nr_chips = 1,
	       .nr_partitions = ARRAY_SIZE(mp5_nand_part),
	       .partitions = mp5_nand_part,
	      // .cmd_len = 0xF5BD1,	//0x93791
	       .cmd_len = 0xF5AD1,
	       .data_len = 0xF5C5C,	//0x91517
	      // .data_len = 0x91717,
	       }
};

static struct ak98_platform_nand mp5_nand_info = {
	.nr_sets = ARRAY_SIZE(mp5_nand_sets),
	.sets = mp5_nand_sets,
};


/* aw9523 GPIO expander */
struct gpio_info aw9523_gpios[] = {
	{
		.pin 		= AK98_GPIO_6,
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
	.parent_irq	= IRQ_GPIO_6,
	.p0xmod		= AW9523_MOD_PUSHPULL,
	.gpios 		= aw9523_gpios,
	.npins		= ARRAY_SIZE(aw9523_gpios),
};

static struct i2c_board_info ak98_aw9523_ext[] __initdata = {
	{
		I2C_BOARD_INFO("aw9523_ext", 0x58),
		.type	= "aw9523_ext",
		.irq	= IRQ_GPIO_6,
		.platform_data = &aw9523_info,
	},
};


struct platform_device ak98_ar7643ts_device = {
	.name = "ar7643-ts",
	.id = -1,
};


/*
* AK98 matrix Keyboard Device
*/
static const uint32_t athena_keypad_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_MENU),	
	KEY(1, 0, KEY_VOLUMEDOWN),
	KEY(1, 1, KEY_BACK),	
};

static struct matrix_keymap_data athena_keypad_keymap_data = {
	.keymap		= athena_keypad_keymap,
	.keymap_size	= ARRAY_SIZE(athena_keypad_keymap),
};


/* gpios whose direction is input should be row gpios */
static const int athena_keypad_row_gpios[] = 
{  AK98_GPIO_102, AK98_GPIO_103 };
/* gpios whose direction is out should be row gpios */
static const int athena_keypad_col_gpios[] = 
{ AK98_GPIO_1 };

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
				.pullup		= AK98_PULLUP_DISABLE,
				.dir 		= AK98_GPIO_DIR_OUTPUT,
				.value		= AK98_GPIO_OUT_LOW, /* default state of output gpios*/
				.int_pol	= -1,
		},
	.grounding		= true, /* grounding line should be the end line*/
				
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
static struct ak98_gpio_keys_button mp5_buttons[] = {
	
	{	
		.code		= KEY_POWER,
		.gpio		= AK98_GPIO_13,
		.active_low	= 0,
		.desc		= "btn_powerdown",
		.debounce_interval = 30,
		.wakeup		= 1,
		.pullup		= -1,
		.pulldown	= AK98_PULLDOWN_DISABLE,
		.dir		= AK98_GPIO_DIR_INPUT,
		.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	
};

static struct ak98_gpio_keys_platform_data mp5_button_data = {
	.buttons	= mp5_buttons,
	.nbuttons	= ARRAY_SIZE(mp5_buttons),
};

	
static struct platform_device mp5_button_device = {
	.name		= "gpio_keys",
	.id			= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &mp5_button_data,
	}
};

/* MMC/SD */

struct ak98_mci_platform_data mp5_mci_plat_data = {
	.gpio_cd  = AK98_GPIO_5,
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

struct platform_device ak98_mmc_device = {
	.name = "ak98_mci",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_mmc_resource),
	.resource = ak98_mmc_resource,
	.dev = {
		.platform_data = &mp5_mci_plat_data,
	},
};

/* SDIO */
struct ak98_mci_platform_data mp5_sdio_plat_data = {
	.gpio_cd  = -ENOSYS,
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
		.platform_data = &mp5_sdio_plat_data,
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
		.pwm_clk = 240,
#elif defined CONFIG_LCD_PANEL_LW700AT9009
		.pwm_clk = 240,
#else
		.pwm_clk = 240,
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
	.origin_pos 	= ORIGIN_BOTTOMLEFT,
	.x_plate_ohms	= 252,
	.is_pen_down	= ts_is_pen_down,
	.init_ts_hw		= ts_init_hwinit,
	.intpin_info	= {
			.pin 		= AK98_GPIO_19,
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
		.irq		= IRQ_GPIO_19,
	},
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
		.dir		= "7",
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
		.dir		= "6",
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
	.irq		= IRQ_AW9523_P04,
	.gpio		= {
		.pin		= AW9523_GPIO_P04,
		.pulldown	= -1,
		.pullup		= -1,
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

struct wifi_control_data athena_wifi_control_data = {
	.gpio_on = {
		.pin		= AW9523_GPIO_P17,
		.pulldown	= -1,
		.pullup		= -1,
		.value		= AK98_GPIO_OUT_HIGH,
		.dir		= AK98_GPIO_DIR_OUTPUT,
		.int_pol	= -1,
	},
	.gpio_off = {
		.pin		= AW9523_GPIO_P17,
		.pulldown	= -1,
		.pullup		= -1,
		.value		= AK98_GPIO_OUT_LOW,
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
static int charge_capacity[]	 = {0,67,74,96, 100};
static int charge_times[]		 = {0,88,159,189,245};

static int charge_capacity_voltage[]	= {0,   4,   62,  75,  99};
static int charge_voltage[]				= {3310,3542,3948,3968,4200};

// sample of battery discharge capacity and voltage,(capacity,voltage)
static int discharge_capacity[] 	= {0,   6,   10,  41,  58,  100};
static int discharge_voltage[]		= {2874,3300,3384,3668,3729,4100};


static struct ak98_bat_mach_info ak98_bat_info = {

	.gpio_init	= ak98_gpio_set,
	
	.usb_gpio	= {
		.active 	= -1,
		.irq		= -ENOSYS, 
		.pindata	={
			.pin		= -1,
			.pulldown	= -1,
			.pullup 	= -1,
			.value		= -1,
			.dir		= -1,
			.int_pol	= -1,
		},	
	},
	
	.ac_gpio	= { 
		.active 	= AK98_GPIO_HIGH,
		.irq		= IRQ_GPIO_115, 
		.pindata	={
			.pin		= AK98_GPIO_115,
			.pulldown	= -1,
			.pullup 	= AK98_PULLUP_DISABLE,
			.value		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= -1,
		},	
	},

	.state_gpio	= {
		.active 	= AK98_GPIO_HIGH,
		.irq		= -ENOSYS, 
		.pindata	={
			.pin		= AK98_GPIO_18,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.pullup 	= AK98_PULLUP_ENABLE,
			.value		= -1,
			.dir		= AK98_GPIO_DIR_INPUT,
			.int_pol	= -1,
		},	
	},
	
	.bat_mach_info	= {
		.voltage_sample 	= 6,			// the sample of read voltage
		.power_down_level	= 10,			// power down if(capacity <= power_down_level) 
		.max_voltage		= 4200, 		// max battery voltage	
		.min_voltage		= 3300, 		// min battery voltage
		.full_capacity		= 100,			// battery full 
		.charge_max_time	= 245,			// battery charge full time
	},

	.bat_adc	= {
		.sample_rate		= 800,			// the same as touch screen
		.wait_time			= 5,			// the same as touch screen
		.up_resistance		= 12,
		.dw_resistance		= 15,
		.voltage_correct	= 32,			// battery correct factor
		.adc_avdd			= 3300,			// avdd voltage
		.adc_poweroff		= 600,			// about vbat = 3.5v
	},

	.charge 	={
		.capacity	= charge_capacity,
		.time		= charge_times,
		.length 	= sizeof(charge_capacity),
		.points 	= sizeof(charge_capacity) / sizeof(charge_capacity[0]),
	},

	.charge_cv	={
		.capacity	= charge_capacity_voltage,
		.voltage	= charge_voltage,
		.length 	= sizeof(charge_capacity_voltage),
		.points 	= sizeof(charge_capacity_voltage) / sizeof(charge_capacity_voltage[0]),
	},
	
	.discharge	={
		.capacity	= discharge_capacity,
		.voltage	= discharge_voltage,
		.length 	= sizeof(discharge_capacity),
		.points 	= sizeof(discharge_capacity) / sizeof(discharge_capacity[0]),
	},
};

// check poweroff
static void check_poweroff_init(void)
{
	struct ak98_bat_mach_info *info = &ak98_bat_info;
	info->gpio_init(&info->ac_gpio.pindata);
	ak98_init_ADC1(info->bat_adc.sample_rate, info->bat_adc.wait_time);
}

void check_poweroff(void)
{
	int pinstate;
	struct ak98_bat_mach_info *info = &ak98_bat_info;
	
	pinstate = ak98_gpio_getpin((unsigned int)info->ac_gpio.pindata.pin);

	// check if charging
	if (pinstate != info->ac_gpio.active)		
	{
		int ad4_value = 0;
		unsigned int count = 0;
		
		ak98_power_ADC1(POWER_ON);
		ak98_enable_bat_mon();	
		
		count = 5;
		while ( count-- )
		{
			mdelay(1);							//delay for get next point ad4
			ad4_value += ak98_read_voltage();	//0x3ff for 10 bit adc
		}
		
		ak98_disable_bat_mon();
		ak98_power_ADC1(POWER_OFF);  
		ad4_value = ad4_value / 5;				// average of read voltage
		
		if (ad4_value <= info->bat_adc.adc_poweroff)
		{
			printk("=========ad4=%d;power off=========\n",ad4_value);
			mp5_power_off();			
		}
		
	}

}

// check powerof end
/*end ak98 battery mach info*/

struct ak98pcm_platform_data ak98pcm_plat_data =
{
	.hpdet_gpio =
	{
			.pin        = AK98_GPIO_7,
			.dir		= AK98_GPIO_DIR_INPUT,
			.pullup		= -1,
			.pulldown	= AK98_PULLDOWN_DISABLE,
			.value      = -1,
			.int_pol	= AK98_GPIO_INT_HIGHLEVEL,
	},
	.spk_down_gpio =
	{
			.pin        = AW9523_GPIO_P10,
			.dir		= AK98_GPIO_DIR_OUTPUT,
			.pullup		= -1,
			.pulldown	= -1,
			.value      = AK98_GPIO_OUT_LOW,
			.int_pol	= -1,
	},
	.hpmute_gpio =
	{
			.pin        = AK98_GPIO_2,
			.dir		= AK98_GPIO_DIR_OUTPUT,
			.pullup		= AK98_PULLUP_DISABLE,
			.pulldown	= -1,
			.value      = AK98_GPIO_LOW,
			.int_pol	= -1,
	},

	.hp_on_value          = AK98_GPIO_HIGH,
	.hpdet_irq            = IRQ_GPIO_7,
	.bIsHPmuteUsed        = 1,
	.hp_mute_enable_value = AK98_GPIO_HIGH,
	.bIsMetalfixed        = 1,
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

/* for sensor power control */ 

static int hi253_camera_power(struct device *dev, int on)
{
	if (on) {
		printk(KERN_DEBUG "hi253 power on\n");
		ak98_gpio_cfgpin(AK98_GPIO_3, AK98_GPIO_DIR_OUTPUT);
		ak98_gpio_setpin(AK98_GPIO_3, AK98_GPIO_HIGH);			
		mdelay(10);		
		ak98_gpio_setpin(AK98_GPIO_3, AK98_GPIO_LOW);	

		/*reset the camera */
		ak98_gpio_cfgpin(AK98_GPIO_4, AK98_GPIO_DIR_OUTPUT);
//		ak98_gpio_setpin(AK98_GPIO_102, AK98_GPIO_HIGH);	
	
		ak98_gpio_setpin(AK98_GPIO_4, AK98_GPIO_LOW);	
		mdelay(100);
		ak98_gpio_setpin(AK98_GPIO_4, AK98_GPIO_HIGH);
	} else {
		printk(KERN_DEBUG "hi253 power down\n");
	
		ak98_gpio_setpin(AK98_GPIO_3, AK98_GPIO_HIGH); 
	}
	
	return 0;
}

/* hi253 image sensor's board information */
static struct i2c_board_info ak98_camara_devices[] = {
	{
		I2C_BOARD_INFO("hi253", 0x20),
	},
};

static struct hi253_camera_info  ak98_hi253_camera_info = {
	.buswidth = SOCAM_DATAWIDTH_8,
	.link = {
		.bus_id = 98,
		.power = hi253_camera_power,
		.board_info = &ak98_camara_devices[0],
		.i2c_adapter_id = 0,		
	}	
};

/* fake device for soc_camera subsystem */
static struct platform_device soc_camera_interface = {
	.name = "soc-camera-pdrv",
	.id   = -1,
	.dev = {
		.platform_data = &ak98_hi253_camera_info.link,
	}
};

/* platform devices */
static struct platform_device *mp5_platform_devices[] __initdata = {
	//&ak98_led_device,
	&ak98_rtc_device,
	&ak98_uart0_device,
	&ak98_uart1_device,
	&ak98_nand_device,
	&ak98_lcd_device,
	&ak98_mmx_device,
	&ak98_mmx_pmem,
	&ak98_i2c_device,
	&ak98_battery_power,
	&mp5_button_device,
	&ak98_mmc_device,
	&ak98_sdio_device,
	&ak98_usb_device,
	&ak98_usb_fs_hcd_device,
	&ak98pcm_device,
	&ak98_android_usb,
	&ak98_android_usb_mass_storage,
	&ak98pwm_backlight_device,
	&ak98_wifi_rtl8188_device,
	&ak98_freq_policy_device,
	&athena_keypad_device,
	&ak_sensor_device,
	&soc_camera_interface,	
	&ak98_camera_interface,	

};

static void mp5_power_off(void)
{
	/*
	 * Do a real power off by polling WAKEUP pin to low
	 */
	ak98_rtc_set_wpin(0);
}

static void mp5_reset(void)
{
	ak98_reboot_sys_by_wtd();
}

static void __init mp5_machine_init(void)
{
	check_poweroff_init();
	check_poweroff();

	pm_power_off = mp5_power_off;
	ak98_arch_reset = mp5_reset;
	
	ak98_nand_device.dev.platform_data = &mp5_nand_info;

	i2c_register_board_info(0, ak98_ts_devices, ARRAY_SIZE(ak98_ts_devices));
	i2c_register_board_info(0, ak98_sensor_devices, ARRAY_SIZE(ak98_sensor_devices));
	i2c_register_board_info(0, ak98_aw9523_ext, ARRAY_SIZE(ak98_aw9523_ext));

	ak_sensor_device.dev.platform_data = &sensor_info;

  	// set battery platform_data
	ak98_battery_power.dev.platform_data = &ak98_bat_info;
	ak98pwm_backlight_device.dev.platform_data = &ak98pwm_backlight_data;
	
	/* register platform devices */
	platform_add_devices(mp5_platform_devices,
		ARRAY_SIZE(mp5_platform_devices));

	/* Initialize L2 buffer */
	ak98_l2_init();

}



MACHINE_START(AK9805_MP5, "BOARD_AK9801ATHENA")
/* Maintainer: */
	.phys_io = 0x20000000,
	.io_pg_offst = ((0xF0200000) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.init_irq = ak98_init_irq,
	.map_io = ak98_map_io,
	.init_machine = mp5_machine_init,
	.timer = &ak98_timer, 
MACHINE_END
