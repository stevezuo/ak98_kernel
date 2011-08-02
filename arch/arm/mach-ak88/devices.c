/* linux/arch/arm/mach-ak880x/devices.c
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
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>

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
#include <mach/lib_uart.h>
#include <mach/lib_lcd.h>
#include <mach/devices_ak880x.h>
#include <mach/gpio.h>

struct platform_device ak880x_led_device = {
	.name = "ak880x-led",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_led_device);

struct platform_device ak880x_rtc_device = {
	.name = "ak880x-rtc",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_rtc_device);

struct platform_device ak880x_pwm0_device = {
	.name = "ak880x-pwm",
	.id = 0,
};

EXPORT_SYMBOL(ak880x_pwm0_device);

struct platform_device ak880x_pwm1_device = {
	.name = "ak880x-pwm",
	.id = 1,
};

EXPORT_SYMBOL(ak880x_pwm1_device);

struct platform_device ak880x_pwm2_device = {
	.name = "ak880x-pwm",
	.id = 2,
};

EXPORT_SYMBOL(ak880x_pwm2_device);

struct platform_device ak880x_pwm3_device = {
	.name = "ak880x-pwm",
	.id = 3,
};

EXPORT_SYMBOL(ak880x_pwm3_device);

#if 1
struct platform_device ak880x_uart0_device = {
	.name = "ak880x-uart",
	.id = 0,
};

EXPORT_SYMBOL(ak880x_uart0_device);

struct platform_device ak880x_uart1_device = {
	.name = "ak880x-uart",
	.id = 1,
};

EXPORT_SYMBOL(ak880x_uart1_device);

struct platform_device ak880x_uart2_device = {
	.name = "ak880x-uart",
	.id = 2,
};

EXPORT_SYMBOL(ak880x_uart2_device);

struct platform_device ak880x_uart3_device = {
	.name = "ak880x-uart",
	.id = 3,
};

EXPORT_SYMBOL(ak880x_uart3_device);
#endif

struct platform_device ak880x_ts_device = {
	.name = "ak880x-ts",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_ts_device);

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

struct platform_device ak880x_gpio_trkball_device = {
	.name		= "ak880x-trkball",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gpio_trkball_resources),
	.resource	= gpio_trkball_resources,
};

EXPORT_SYMBOL(ak880x_gpio_trkball_device);

struct platform_device ak880x_kpd_device = {
	.name = "ak880x-kpd",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_kpd_device);

struct platform_device ak880x_sdio_device = {
	.name = "ak880x-sdio",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_sdio_device);

/* AK88 AD/DA for sound system */
struct platform_device ak880x_snd_device = {
	.name = "ak880x-snd",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_snd_device);

/* AK88 SPI */
static struct resource ak880x_spi0_resource[] = {
	[0] = {
	       .start = AK88_PA_SPI0,
	       .end = AK88_PA_SPI0 + 0x23,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = IRQ_SPI1,
	       .end = IRQ_SPI1,
	       .flags = IORESOURCE_IRQ,
	       }
};

struct platform_device ak880x_spi0_device = {
	.name = "ak880x-spi",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak880x_spi0_resource),
	.resource = ak880x_spi0_resource,
};

EXPORT_SYMBOL(ak880x_spi0_device);

struct platform_device ak880x_lcd_device = {
	.name = "ak880x-lcd",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_lcd_device);

struct platform_device ak880x_osd_device = {
	.name = "ak880x-osd",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_osd_device);

struct platform_device ak880x_tvout_device = {
	.name = "ak880x-tvout",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_tvout_device);

struct platform_device ak880x_nand_device = {
	.name = "ak880x-nand",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_nand_device);

struct platform_device ak880x_i2c_device = {
	.name = "ak880x-i2c",
	.id = -1,
};

EXPORT_SYMBOL(ak880x_i2c_device);

/* MMC/SD */
static struct resource ak880x_mmc_resource[] = {
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

struct ak88_mci_platform_data mci_plat_data = {
#ifdef CONFIG_BOARD_AK8801EPC
	.gpio_cd  = AK88_GPIO_12,
	.gpio_wp  = AK88_GPIO_19,
#elif defined CONFIG_BOARD_AK8802EBOOK
	.gpio_cd  = AK88_GPIO_7,
	.gpio_wp  = AK88_GPIO_10,
#else
#error "board defined error"
#endif
};

struct platform_device ak880x_mmc_device = {
	.name = "ak88_mci",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak880x_mmc_resource),
	.resource = ak880x_mmc_resource,
	.dev = {
		.platform_data = &mci_plat_data,
	},
};

EXPORT_SYMBOL(ak880x_mmc_device);

/* USB */
static struct resource ak880x_usb_resource[] = {
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

struct platform_device ak880x_usb_device = {
	.name = "ak880x_udc",
	.id = -1,
	.num_resources = ARRAY_SIZE(ak880x_usb_resource),
	.resource = ak880x_usb_resource,
};

EXPORT_SYMBOL(ak880x_usb_device);

#if 0

/* --------------------------------------------------------------------
 *  UART
 * --------------------------------------------------------------------*/

#if defined(CONFIG_SERIAL_AK88)

static struct resource uart0_resources[] = {
	[0] = {
		.start	= (int)AK88_UART_CFG_REG1(0) ,
		.end	= (int)AK88_UART_CFG_REG1(0) + 0x100,
		.flags	= (int)IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART0,
		.end	= IRQ_UART0,
		.flags	= IORESOURCE_IRQ,
	},

	[2] = {
		.start	= (int)AK88_VA_L2BUF + 0x1000,         //buf8 ,sendbuf
		.end	= (int)AK88_VA_L2BUF + 0x1000+0x80-1, 
		.flags	= (int)IORESOURCE_MEM,
	},
	[3] = {
		.start	= (int)AK88_VA_L2BUF + 0x1000+0x80,    //buf9 ,recvbuf
		.end	= (int)AK88_VA_L2BUF + 0x1000+0x80+0x80-1, 
		.flags	= (int)IORESOURCE_MEM,
	},

};
 
static struct resource uart3_resources[] = {
	[0] = {
		.start	= (int)AK88_UART_CFG_REG1(3) ,
		.end	= (int)AK88_UART_CFG_REG1(3) + 0x100,
		.flags	= (int)IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_UART3,
		.end	= IRQ_UART3,
		.flags	= IORESOURCE_IRQ,
	},
 
	[2] = {
		.start	= (int)AK88_VA_L2BUF + 0x1300,         //buf14,sendbuf
		.end	= (int)AK88_VA_L2BUF + 0x1300+0x80-1,   
		.flags	= (int)IORESOURCE_MEM,
	},
	[3] = {
		.start	= (int)AK88_VA_L2BUF + 0x1300+0x80,    //buf15,recvbuf
		.end	= (int)AK88_VA_L2BUF + 0x1300+0x80+0x80-1,   
		.flags	= (int)IORESOURCE_MEM,
	},

};
 
static struct platform_device ak880x_uart0_device_lynn = {
	.name		= "ak880x_serial",
 	.id		= 0,
 	.resource	= uart0_resources,
	.num_resources	= ARRAY_SIZE(uart0_resources),
};

static struct platform_device ak880x_uart3_device_lynn = {
	.name		= "ak880x_serial",
 	.id		= 3,
 	.resource	= uart3_resources,
	.num_resources	= ARRAY_SIZE(uart3_resources),
};

static inline void configure_uart0_pins(void)
{
	//at91_set_A_periph(AT91_PIN_PC30, 0);		/* DRXD */
	//at91_set_A_periph(AT91_PIN_PC31, 1);		/* DTXD */

    //set sharepin
//#define AK88_SHAREPIN_CTRL    (AK88_VA_SYS+0x0078)   //0xF0000078
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078

//all bits default is 0, = Corresponding pin is used as GPIO
//bit[9] : 1=corresponding are used as {URD1,UTD1}  // gpio[15:14]
//bit[14]: 1=corresponding are used as {URD4,UTD4}  // gpio[25:24]

 	AKSET_BITS(1UL<<9,AK88_SHAREPIN_CTRL);

        //open power clock
//#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)   //0xF000000C
//bit[15],0 = to enable L2 controller/UART1 working clock

 	AKCLR_BITS(1UL<<15,AK88_POWER_CLOCK); 
}


static inline void configure_uart3_pins(void)
{
	//at91_set_A_periph(AT91_PIN_PC30, 0);		/* DRXD */
	//at91_set_A_periph(AT91_PIN_PC31, 1);		/* DTXD */

    //set sharepin
//#define AK88_SHAREPIN_CTRL    (AK88_VA_SYS+0x0078)   //0xF0000078
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078

//all bits default is 0, = Corresponding pin is used as GPIO
//bit[9] : 1=corresponding are used as {URD1,UTD1}  // gpio[15:14]
//bit[14]: 1=corresponding are used as {URD4,UTD4}  // gpio[25:24]
//bit[15]: 1=corresponding are used as {CTS4,RTS4}  // gpio[27:26]

//#define AK88_SHAREPIN_CTRL2    (AK88_VA_SYS+0x0074)   //0xF0000074
//bit[2:1] 00:reserved
//	   01 =	Corresponding pins are used as those of UART4
//	   10 =	Corresponding pins are used as those of SDIO interface
//	   11:reserved

	AKSET_BITS(1UL<<14,AK88_SHAREPIN_CTRL);
	AKSET_BITS(1UL<<15,AK88_SHAREPIN_CTRL);
	AKCLR_BITS(3UL<<1,AK88_SHAREPIN_CTRL2);
	AKSET_BITS(1UL<<1,AK88_SHAREPIN_CTRL2);

        //open power clock
//#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)   //0xF000000C
//bit[15],0 = to enable L2 controller/UART1 working clock
//bit[24],0 = No instruction, 1 = To reset SDIO interface/UART4

 	AKCLR_BITS(1UL<<8,AK88_POWER_CLOCK); //

}
 
/* the UARTs to use */
static struct platform_device *__initdata ak880x_uarts[AK88_MAX_UART];

struct platform_device *ak880x_default_console_device;	/* the serial console device */

void __init ak880x_register_uart(unsigned char id, unsigned char portnr)
//register single serial device
{
	struct platform_device *pdev;

        uart_clear_rx_status(portnr);
        uart_clear_rx_timeout(portnr);
        uart_clear_rx_buffull(portnr);
	l2_clr_uartbuf_status((portnr*2+9));

	switch (id) {
		case 0:	/* DBGU */
	 		pdev = &ak880x_uart0_device;
 
			configure_uart0_pins();

                        //uart_init(id,115200,132000000);  //132MHZ
                        uart_init(portnr,115200,124000000);  //124MHZ
 
 			break;

		case 3:	/* for ak8802 EBOOK DBGU */
			pdev = &ak880x_uart3_device;
 
			configure_uart3_pins();

                        //uart_init(id,115200,3);
                        uart_init(portnr,115200,132000000);   //132MHZ
		        //uart_open_interrupt(portnr);

			break;

		default:
			return;
	}
	pdev->id = portnr;		/* update to mapped ID */

	if (portnr < AK88_MAX_UART)
		ak880x_uarts[portnr] = pdev;
 
}

void __init ak880x_set_serial_console(unsigned char portnr)
{
	if (portnr < AK88_MAX_UART)
		ak880x_default_console_device = ak880x_uarts[portnr];
}

void __init ak880x_add_device_serial(void)
//add all registed serial device
{
	int i;

	for (i = 0; i < AK88_MAX_UART; i++) {
		if (ak880x_uarts[i])
			platform_device_register(ak880x_uarts[i]);
	}

	if (!ak880x_default_console_device)
		printk(KERN_INFO "AK88: No default serial console defined.\n");
}

#else
void __init ak880x_register_uart(unsigned id, unsigned portnr, unsigned pins) {}
void __init ak880x_set_serial_console(unsigned portnr) {}
void __init ak880x_add_device_serial(void) {}
#endif

#endif

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
	       .start = (int)AK88_VA_DISP,
	       .end = (int)AK88_VA_DISP + SZ_4K - 1,
	       .flags = (int)IORESOURCE_MEM,
	       },
	[1] = {
	       .start = (int)IRQ_DISP,	//IRQ_DISP=1
	       .end = (int)IRQ_DISP,
	       .flags = (int)IORESOURCE_IRQ,
	       },
};

static struct platform_device ak880x_lcdc_device = {
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

void __init ak880x_add_device_lcdc(struct anyka_lcdfb_info *data)
{
	if (!data)
		return;
 
	//set sharepin
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
//bit[25]: 1=corresponding are used as {LCD_DATA[15:9]}  //gpio[68:62]
//bit[26]: 1=corresponding are used as {LCD_DATA[8]}     //gpio[61]
//bit[27]: 1=corresponding are used as {LCD_DATA[17:16]} //gpio[70:69] 

	AKSET_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[15:9]
	AKSET_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[8]
	AKSET_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[17:16]

	//open power clock
//#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)   //0xF000000C
//bit[15],0 = to enable L2 controller/UART1 working clock
//bit[3], 0 = to enable display controller working clock

	//open LCD controller clock
	AKCLR_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

	set_ahb_priority();

	lcdc_data = *data;
	platform_device_register(&ak880x_lcdc_device);
}
#else
void __init ak880x_add_device_lcdc(struct anyka_lcdfb_info *data)
{
}
#endif
