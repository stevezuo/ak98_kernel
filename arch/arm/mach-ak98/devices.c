/* linux/arch/arm/mach-ak98/devices.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/ts.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <video/anyka_lcdc.h>

#include <mach/map.h>
#include <mach/irqs.h>
#include <mach/lib_l2.h>
#include <mach/lib_lcd.h>
#include <mach/devices_ak880x.h>
#include <mach/gpio.h>
#include <mach/mac.h>
#include <mach/i2c.h>

#include <linux/uio_driver.h>
#include <linux/android_pmem.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <mach/regs-comm.h>
#include <linux/usb/android_composite.h>
#include <linux/ctype.h>

struct platform_device ak98_freq_policy_device = {
	.name = "freq_policy",
		.id = -1,
};
EXPORT_SYMBOL(ak98_freq_policy_device);


struct platform_device ak98_led_device = {
	.name = "ak98-led",
	.id = -1,
};
EXPORT_SYMBOL(ak98_led_device);

struct platform_device ak98_rtc_device = {
	.name = "ak98-rtc",
	.id = -1,
};
EXPORT_SYMBOL(ak98_rtc_device);

struct platform_device ak98adc_ts_device = {
	.name = "ak98adc-ts",
	.id = -1,
};
EXPORT_SYMBOL(ak98adc_ts_device);

struct platform_device ak98pwm_backlight_device = {
	.name = "ak98pwm-backlight",
		.id = -1,
};
EXPORT_SYMBOL(ak98pwm_backlight_device);

struct platform_device ak98_pwm0_device = {
	.name = "ak98-pwm",
	.id = 0,
};
EXPORT_SYMBOL(ak98_pwm0_device);

struct platform_device ak98_pwm1_device = {
	.name = "ak98-pwm",
	.id = 1,
};
EXPORT_SYMBOL(ak98_pwm1_device);

struct platform_device ak98_pwm2_device = {
	.name = "ak98-pwm",
	.id = 2,
};
EXPORT_SYMBOL(ak98_pwm2_device);

struct platform_device ak98_pwm3_device = {
	.name = "ak98-pwm",
	.id = 3,
};
EXPORT_SYMBOL(ak98_pwm3_device);

struct platform_device ak98_uart0_device = {
	.name = "ak98-uart",
	.id = 0,
};
EXPORT_SYMBOL(ak98_uart0_device);

struct platform_device ak98_uart1_device = {
	.name = "ak98-uart",
	.id = 1,
};
EXPORT_SYMBOL(ak98_uart1_device);

struct platform_device ak98_uart2_device = {
	.name = "ak98-uart",
	.id = 2,
};
EXPORT_SYMBOL(ak98_uart2_device);

struct platform_device ak98_uart3_device = {
	.name = "ak98-uart",
	.id = 3,
};
EXPORT_SYMBOL(ak98_uart3_device);

/* battery_power supply */
struct platform_device ak98_battery_power = {
	.name = "fake_battery",
	.id   = -1,
};
EXPORT_SYMBOL(ak98_battery_power);

static struct resource gpio_trkball_resources[] = {
#if 0	/* NORMAL SCREEN */
	{		/* UP IRQ */
		.name		= "UP GPIO IRQ",
		.start		= IRQ_GPIO_79,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* DOWN IRQ */
		.name		= "DOWN GPIO IRQ",
		.start		= IRQ_GPIO_77,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* LEFT IRQ */
		.name		= "LEFT GPIO IRQ",
		.start		= IRQ_GPIO_76,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* RIGHT IRQ */
		.name		= "RIGHT GPIO IRQ",
		.start		= IRQ_GPIO_78,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
#else	/* ROTATE SCREEN */
	{		/* UP IRQ */
		.name		= "UP GPIO IRQ",
		.start		= IRQ_GPIO_76,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* DOWN IRQ */
		.name		= "DOWN GPIO IRQ",
		.start		= IRQ_GPIO_78,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* LEFT IRQ */
		.name		= "LEFT GPIO IRQ",
		.start		= IRQ_GPIO_77,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	}, {		/* RIGHT IRQ */
		.name		= "RIGHT GPIO IRQ",
		.start		= IRQ_GPIO_79,
		.flags		= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
#endif
};

struct platform_device ak98_gpio_trkball_device = {
	.name		= "ak98-trkball",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gpio_trkball_resources),
	.resource	= gpio_trkball_resources,
};
EXPORT_SYMBOL(ak98_gpio_trkball_device);

struct platform_device ak98_kpd_device = {
	.name = "ak98-kpd",
	.id = -1,
};
EXPORT_SYMBOL(ak98_kpd_device);

/* AK88 AD/DA for sound system */
struct platform_device ak98_snd_device = {
	.name = "ak98-snd",
	.id = -1,
};
EXPORT_SYMBOL(ak98_snd_device);

/* AK88 SPI */
static struct resource ak98_spi1_resource[] = {
	[0] = {
	       .start = AK98_PA_SPI1,
	       .end = AK98_PA_SPI1 + 0x24,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SPI1,
	       .end = IRQ_SPI1,
	       .flags = IORESOURCE_IRQ,
	       }
};

struct platform_device ak98_spi1_device = {
	.name = "ak98-spi",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_spi1_resource),
	.resource = ak98_spi1_resource,
};
EXPORT_SYMBOL(ak98_spi1_device);

static struct resource ak98mmx_resources[] = {
	{
		.name   = "system-ctrl",
		.start	= 0x08000000,
		.end	= 0x08000017,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "video-base",
		.start	= 0x20000000,
		.end	= 0x20000693,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "h264-decoder",
		.start	= 0x20001000,
		.end	= 0x20001cff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "h264-Q_MATRIX",
		.start	= 0x20002800,
		.end	= 0x20002bbf,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "h264-dmx",
		.start	= 0x20004000,
		.end	= 0x2000400f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "mpeg4-decoder",
		.start	= 0x20040000,
		.end	= 0x200401ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "mpeg4-decoder2",
		.start	= 0x20040200,
		.end	= 0x200402ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "mpeg4-decoder3",
		.start	= 0x20040300,
		.end	= 0x200403ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "xvid-decoder",
		.start	= 0x20040800,
		.end	= 0x200408ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "h263-encoder",
		.start	= 0x20048000,
		.end	= 0x20048023,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "rv-decoder",
		.start	= 0x20070000,
		.end	= 0x20070053,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "rv-decoder2",
		.start	= 0x20072000,
		.end	= 0x2007202f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "rv-filter",
		.start	= 0x20078000,
		.end	= 0x20078021,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "mpeg2/asp-decoder",
		.start	= 0x20071000,
		.end	= 0x20071063,
		.flags	= IORESOURCE_MEM,
	}
};

static struct uio_info akmmx_uioinfo = {
	.name    = "ak98-vcodec",
	.version = "0.1.0",
#ifdef CONFIG_UIODMA
	.use_dma = true,
#endif
	.irq     = UIO_IRQ_CUSTOM,
};

struct platform_device ak98_mmx_device = {
	.name		= "uio_ak98_vcodec",
	.id		= -1,
	.dev		= {
		.platform_data = &akmmx_uioinfo,
	},
	.num_resources	= ARRAY_SIZE(ak98mmx_resources),
	.resource	= ak98mmx_resources,
};
EXPORT_SYMBOL(ak98_mmx_device);

static struct android_pmem_platform_data akmmx_pmeminfo = {
	.name    = "ak98_mmx_pmem",
#if 0
	.start   = 0x8E600000,		/* the last 26M */
#else
	.start   = CONFIG_RAM_BASE,	/* the first 26M */
#endif
	.size    = CONFIG_VIDEO_RESERVED_MEM_SIZE,
	.no_allocator = 1,
	.cached  = 1,
	.buffered = 1,
};

struct platform_device ak98_mmx_pmem = {
	.name           = "android_pmem",
	.id             = -1,
	.dev            = {
		.platform_data = &akmmx_pmeminfo,
	}
};

EXPORT_SYMBOL(ak98_mmx_pmem);

#define IRQ_DISP IRQ_DISPLAY_CTRL
static struct resource ak98fb_resources[] = {
	[0] = {
		.start	= 0x2000d000,
		.end	= 0x2000d123,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_DISP,
		.end	= IRQ_DISP,
		.flags	= IORESOURCE_IRQ,
	},
};
static u64 fb_dma_mask = DMA_BIT_MASK(32);
struct platform_device ak98_lcd_device = {
	.name		= "ak98-lcd",
	.id		= -1,
	.dev		= {
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(ak98fb_resources),
	.resource	= ak98fb_resources,
};
EXPORT_SYMBOL(ak98_lcd_device);

struct platform_device ak98_osd_device = {
	.name = "ak98-osd",
	.id = -1,
};
EXPORT_SYMBOL(ak98_osd_device);

struct platform_device ak98_tvout_device = {
	.name = "ak98-tvout",
	.id = -1,
};
EXPORT_SYMBOL(ak98_tvout_device);

struct platform_device ak98_nand_device = {
	.name = "ak98-nand",
	.id = -1,
};
EXPORT_SYMBOL(ak98_nand_device);

/* I2C */
struct gpio_info i2c_gpios[] = {
		{
			.pin 		= AK98_GPIO_92,
			.pulldown 	= -1,
			.pullup 	= -1,
			.dir		= AK98_GPIO_DIR_OUTPUT,
			.value 		= AK98_GPIO_HIGH,
			.int_pol	= -1,
		},
		{
			.pin 		= AK98_GPIO_93,
			.pulldown 	= -1,
			.pullup 	= -1,
			.dir		= AK98_GPIO_DIR_OUTPUT,
			.value 		= AK98_GPIO_HIGH,
			.int_pol	= -1,

		},
};

static struct ak98_platform_i2c ak98_default_i2c_data = {
	.flags		= 0,
	.bus_num	= 0,
	.slave_addr	= 0x10,
	.frequency	= 100*1000,
	.sda_delay	= 100,
	.gpios		= i2c_gpios,
	.npins		= ARRAY_SIZE(i2c_gpios),
};

static struct resource ak98_i2c_resource[] = {
	[0] = {
		.start = AK98_PA_I2C,
		.end   = AK98_PA_I2C + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end   = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ak98_i2c_device = {
	.name	= "ak98-i2c",
	.id		= -1,
	.dev	= {
		.platform_data = &ak98_default_i2c_data,
	},
	.num_resources	= ARRAY_SIZE(ak98_i2c_resource),
	.resource		= ak98_i2c_resource,
};
EXPORT_SYMBOL(ak98_i2c_device);

/* USB */
static struct resource ak98_usb_resource[] = {
	[0] = {
	       .start = 0x70000000,
	       .end = 0x700007ff,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "usb mcu irq",
	       .start = IRQ_USBOTG_MCU,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .name = "usb dma irq",
	       .start = IRQ_USBOTG_DMA,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device ak98_usb_device = {
	.name = "ak98_udc",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_usb_resource),
	.resource = ak98_usb_resource,
};
EXPORT_SYMBOL(ak98_usb_device);

/* USB FS HCD .*/
static struct resource ak98_usb_fs_hcd_resource[] = {
	[0] = {
	       .start = 0x70008000,
	       .end = 0x70008fff,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "usb mcu irq",
	       .start = IRQ_USBHOST_MCU,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .name = "usb dma irq",
	       .start = IRQ_USBHOST_DMA,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device ak98_usb_fs_hcd_device = {
	.name = "ak98-fs-hcd",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_usb_fs_hcd_resource),
	.resource = ak98_usb_fs_hcd_resource,
};
EXPORT_SYMBOL(ak98_usb_fs_hcd_device);

/* MAC */
static struct resource ak98_mac_resource[] = {
	[0] = {
	       .start = 0x60000000,
	       .end = 0x60001fff,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "mac irq",
	       .start = IRQ_MAC,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct ak98_mac_data ak98_mac = {
	.dev_addr = {0x00, 0x1a, 0xa0, 0x3a, 0x6c, 0xff},
};

#ifndef CONFIG_USB_GADGET_AK98_PRODUCER
// convert a capital character to int
#define CTOI(c) (isdigit(c) ? (c - '0') : (c - 'A' + 10))
			
int __init ak98_mac_boot_setup(char *str)
{
	char mac_addr[MAC_ADDR_STRING_LEN + 1];
	int i;

	if (strlen(str) != MAC_ADDR_STRING_LEN)
		goto out;
	
	strcpy(mac_addr, str);
	for (i = 0; i < MAC_ADDR_STRING_LEN; i++) {
		if ((i % 3 != 2)) {
			mac_addr[i] = toupper(mac_addr[i]);
			if (!(isdigit(mac_addr[i]) || (mac_addr[i] <= 'F' && mac_addr[i] >= 'A')))
				goto out;
		}
		else if (mac_addr[i] != ':')
			goto out;
	}
	
	for (i  = 0; i < MAC_ADDR_LEN; i++)
		ak98_mac.dev_addr[i] = CTOI(mac_addr[i * 3]) * 16 + CTOI(mac_addr[i * 3 + 1]);

	return 0;

out:
	printk("input MAC address ERROR, use the default mac address!\n");

	return 0;
}

__setup("mac_addr=", ak98_mac_boot_setup);
#endif

struct platform_device ak98_mac_device = {
	.name = "ak98_mac",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak98_mac_resource),
	.resource = ak98_mac_resource,
	.dev        = {
		.platform_data = &ak98_mac,
	}
};
EXPORT_SYMBOL(ak98_mac_device);
/* --------------------------------------------------------------------
 *  LCD Controller
 * -------------------------------------------------------------------- */

//#if defined(CONFIG_FB_ANYKA) || defined(CONFIG_FB_ANYKA_MODULE)
#if defined(CONFIG_FB_AK88) || defined(CONFIG_FB_AK88_DEBUG)

struct anyka_lcdfb_info;

static u64 lcdc_dmamask = DMA_BIT_MASK(32);	//=1UL<<31, if(n==64) : ~0ULL
static struct anyka_lcdfb_info lcdc_data;

#define IRQ_DISP IRQ_DISPLAY_CTRL
static struct resource lcdc_resources[] = {
	[0] = {
	       .start = (int)AK98_VA_DISP,
	       .end = (int)AK98_VA_DISP + SZ_4K - 1,
	       .flags = (int)IORESOURCE_MEM,
	       },
	[1] = {
	       .start = (int)IRQ_DISP,	//IRQ_DISP=1
	       .end = (int)IRQ_DISP,
	       .flags = (int)IORESOURCE_IRQ,
	       },
};

static struct platform_device ak98_lcdc_device = {
	.name = "anyka_lcdfb",
	.id = 0,
	.dev = {
		.dma_mask = &lcdc_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &lcdc_data,
		},
	.resource = lcdc_resources,
	.num_resources = ARRAY_SIZE(lcdc_resources),
};

void __init ak98_add_device_lcdc(struct anyka_lcdfb_info *data)
{
	if (!data)
		return;
 
	//set sharepin
//#define rSHAREPIN_CON1    (ANYKA_PA_SYS+0x0078)   //0x08000078
//bit[25]: 1=corresponding are used as {LCD_DATA[15:9]}  //gpio[68:62]
//bit[26]: 1=corresponding are used as {LCD_DATA[8]}     //gpio[61]
//bit[27]: 1=corresponding are used as {LCD_DATA[17:16]} //gpio[70:69] 

	AKSET_BITS(1UL << 25, rSHAREPIN_CON1);	//set pin as LCD_DATA[15:9]
	AKSET_BITS(1UL << 26, rSHAREPIN_CON1);	//set pin as LCD_DATA[8]
	AKSET_BITS(1UL << 27, rSHAREPIN_CON1);	//set pin as LCD_DATA[17:16]

	//open power clock
//#define rCLK_CON1    (AK98_VA_SYS+0x000C)   //0xF000000C
//bit[15],0 = to enable L2 controller/UART1 working clock
//bit[3], 0 = to enable display controller working clock

	//open LCD controller clock
	AKCLR_BITS(1UL << 3, rCLK_CON1);	//0x0800000C

	set_ahb_priority();

	lcdc_data = *data;
	platform_device_register(&ak98_lcdc_device);
}
#else
void __init ak98_add_device_lcdc(struct anyka_lcdfb_info *data)
{
}
#endif

//char *funs[] = {"adb", "usb_mass_storage"};
char *funs1[] = {"usb_mass_storage"};
char *funs2[] = {"usb_mass_storage", "adb"};

struct android_usb_product ak98_android_usb_product[] = {
	[0] = {
		.product_id = 0x0C02,
		.num_functions = 1,
		.functions = funs1,
	},
	[1] = {
		.product_id = 0x0D02,
		.num_functions = 2,
		.functions = funs2,
	},
};

struct android_usb_platform_data ak98_usb_platform_data = {
	.vendor_id = 0x18D1,
	.product_id = 0x0D02,
	.version = 0x1,

	.product_name = "Android",
	.manufacturer_name = "ANYKA",
	.serial_number = "0123456789ABCDEF",

	.num_products = ARRAY_SIZE(ak98_android_usb_product),
	.products = ak98_android_usb_product,

	.num_functions = 2,
	.functions = funs2,
};

struct platform_device ak98_android_usb = {
	.name = "android_usb",
	.id = 0,
	.dev = {
			.platform_data = &ak98_usb_platform_data,
	}
};
EXPORT_SYMBOL(ak98_android_usb);

struct usb_mass_storage_platform_data ak98_usb_mass_storage = {
	.vendor = "ANYKA",
	.product = "Mass Storage",
	.release = 0x01,

	.nluns = 1,
};

struct platform_device ak98_android_usb_mass_storage = {
	.name = "usb_mass_storage",
	.id = 0,
	.dev = {
			.platform_data = &ak98_usb_mass_storage,
	}
};
EXPORT_SYMBOL(ak98_android_usb_mass_storage);

/* Camera interface resource */
static struct resource ak98_camera_resource[] = {
	[0] = {
	       .start = 0x2000c000,
	       .end = 0x2000c084,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "camera if irq",
	       .start = IRQ_CAMERA_IF,
	       .flags = IORESOURCE_IRQ,
	       },
};

/* camera interface */
struct platform_device ak98_camera_interface = {
	.name = "ak98_camera",
	.id   = 98,
	.num_resources	= ARRAY_SIZE(ak98_camera_resource),	
	.resource = ak98_camera_resource,
};
EXPORT_SYMBOL(ak98_camera_interface);


