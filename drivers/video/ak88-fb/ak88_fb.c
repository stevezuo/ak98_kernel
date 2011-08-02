/*
 * linux/drivers/video/ak880xfb.c
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/div64.h>


#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <mach/map.h>
#include <mach/ak880x_addr.h>
#include <mach/gpio.h>

#define lcd_set_bit(reg, set_mask, clr_mask)  \
	do {                                        \
		unsigned long val = 0;                  \
		val = (reg);                            \
		val &= ~(clr_mask);                     \
		val |= (set_mask);                      \
		reg = val;                              \
	} while(0)

/* Debugging stuff */
#ifdef CONFIG_FB_AK88_DEBUG
static int debug	= 1;
#else
static int debug	= 0;
#endif

#define dprintk(msg...)	if (debug) { printk(KERN_DEBUG "ak880xfb: " msg); }

#define	AUTO_REFRESH 0
#define	STOP_AUTO 1
#define	MANUAL_REFRESH 2

#define FIFO_ALARM_STAT (1<<18)
#define ALERT_VALID_STAT (1<<17)
#define MPU_DISPLAY_OK_STAT (1<<2)

#define	LCD_M			0
#define	LCD_S			1
#define	LCD_MPU_CMD		0
#define	LCD_MPU_DATA	1


#define MAIN_LCD_MPU_CMD				0x00000000	//master LCD command
#define MAIN_LCD_MPU_DATA				0x00080000	//master LCD data
#define SUB_LCD_MPU_CMD					0x00040000	//slaver LCD command
#define SUB_LCD_MPU_DATA				0x000C0000	//slaver LCD data



struct fb_info *fbinfo;
static unsigned long save[53];
static unsigned long blank;

unsigned char			lcd_auto_refresh = 0 ;
unsigned int			refresh_count = 0 ;
unsigned int			pseudo_pal[16];



void index_out(unsigned int lcd, unsigned short reg_index)
{
	rLCD_COMM2 = (MAIN_LCD_MPU_CMD|reg_index);
}


static void data_out(unsigned int lcd, unsigned short reg_data)
{
	rLCD_COMM2 = MAIN_LCD_MPU_DATA|reg_data;
}

static void lcd_write_reg( unsigned short index, unsigned short data )
{
	int lcd =0;
	index_out(lcd, index);
	udelay(100);
	data_out(lcd, data);
}



void lcd_set_mode(unsigned int IF_Sel, unsigned int Disply_Color_Sel, 
		unsigned int Bus_Sel, unsigned int W_Len1, unsigned int W_Len2)
{
	unsigned int mode = 0;
	unsigned int A0_polarity = 1;

	if (IF_Sel == 0x00)
	{
		IF_Sel = 0x2;
	}
	else if (IF_Sel == 0x01) 
	{
		IF_Sel = 0x3; 
	}
	else if (IF_Sel == 0x02)
	{
		IF_Sel = 0x1; 
	}
	else
	{
		return;
	}

	W_Len1 &= 0x7f;
	W_Len2 &= 0x1f;

	lcd_set_bit(rLCD_COMM1, 0, 1<<4);
	lcd_set_bit(rLCD_COMM1, 0, 3<<5);
	lcd_set_bit(rLCD_COMM1, IF_Sel<<5, 0);
	lcd_set_bit(rLCD_COMM1, 0, 1<<15);

	mode = 0;
	mode |=  (Bus_Sel << 14);
	mode |=  (Disply_Color_Sel << 13);
	mode |=  (W_Len1 << 6);
	mode |=  (W_Len2 << 1);
	mode |=  A0_polarity;
	rLCD_MPUIFCON = mode;
}


void start_dma(void)
{
#if defined( CONFIG_LCD_HX8352 )
	*(volatile unsigned int *)(AK88_VA_DISPLAY + 0x00B4) =  0x2c; //LCD_REG_CONFIG_REG
#else
	*(volatile unsigned int *)(AK88_VA_DISPLAY + 0x00B4) =  0x22; //LCD_REG_CONFIG_REG
#endif
}


void display_refresh(void)
{
	*(volatile unsigned int*)(AK88_VA_DISPLAY + 0x00B8) = ( 1<<3 | 0<<2 | 0 );
}


static struct timer_list lcd_refresh_timer;

static void jiffies_timer(unsigned long data)
{
	int delay = HZ/20;

	if(blank == 0)
	{
		start_dma();
		display_refresh();
		mod_timer(&lcd_refresh_timer, jiffies + delay);
	}
}



static int ak880xfb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	var->transp.msb_right    = 0;
	var->transp.offset    = 0;
	var->transp.length = 0;
	return 0;
}


static void ak880xfb_activate_var(struct fb_info *info)
{
}


static int ak880xfb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	return 0;
}


static int ak880xfb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{

	unsigned int val;
	return 0;
}



void ILI9481_CPT_Initial_Code( unsigned int lcd ) 
{ 
	index_out(lcd,0x11);
	mdelay(20);
	index_out( lcd, 0xd0 );
	data_out( lcd, 0x07 );
	data_out( lcd, 0x42 );
	data_out( lcd, 0x0c );
	mdelay(10); 
	index_out( lcd, 0xd1 );
	data_out( lcd, 0x00 );
	data_out( lcd, 0x12 );
	data_out( lcd, 0x14 );
	mdelay(10);
	index_out( lcd, 0xd2 );
	data_out( lcd, 0x01 );
	data_out( lcd, 0x00 );
	index_out( lcd, 0xc0 );
	data_out( lcd, 0x14 );
	data_out( lcd, 0x3c );
	data_out( lcd, 0x3c );
	data_out( lcd, 0x02 );
	data_out( lcd, 0x11 );
	index_out( lcd, 0xc5 );
	data_out( lcd, 0x03 );
	index_out( lcd, 0x3a );
	data_out( lcd, 0x55 );
	index_out(lcd,0xC8);
	data_out(lcd,0x00);
	data_out(lcd,0x32);
	data_out(lcd,0x25);
	data_out(lcd,0x15);
	data_out(lcd,0x08);
	data_out(lcd,0x05);
	data_out(lcd,0x25);
	data_out(lcd,0x54);
	data_out(lcd,0x77);
	data_out(lcd,0x51);
	data_out(lcd,0x07);
	data_out(lcd,0x08);
	index_out(lcd,0xF3);
	data_out(lcd,0x24); 
	data_out(lcd,0x1A); 
	index_out(lcd,0xF7);
	data_out(lcd,0xC0);
	data_out(lcd,0x01);
	index_out(lcd,0x36);
	data_out(lcd,0x08);
	index_out(lcd,0x2A);
	data_out(lcd,0x00);
	data_out(lcd,0x00);
	data_out(lcd,0x01);
	data_out(lcd,0x3F);
	index_out(lcd,0x2B);
	data_out(lcd,0x00);
	data_out(lcd,0x00);
	data_out(lcd,0x00);
	data_out(lcd,0xEF);
	mdelay(50);
	index_out(lcd,0x29);
}


void start_lcd(void)
{
	if(blank == 0)
	{
		ILI9481_CPT_Initial_Code(0);
		start_dma();
		display_refresh();
		mod_timer(&lcd_refresh_timer, jiffies + HZ/20);
	}
	else
		blank = 4;
}


static int ak880xfb_blank(int blank_mode, struct fb_info *info)
{
	blank = blank_mode;
	start_lcd();
	return 0;
}


static int ak880xfb_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", debug ? "on" : "off");
}


static int ak880xfb_debug_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	if (len < 1)
		return -EINVAL;
	return len;
}



static struct fb_ops ak880xfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= ak880xfb_check_var,
	.fb_set_par	= ak880xfb_set_par,
	.fb_blank	= ak880xfb_blank,
	.fb_setcolreg	= ak880xfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static int __init ak880xfb_map_video_memory(struct fb_info *info)
{
	info->screen_base = ioremap(0x33f00000, 240*320*2) ;
	info->screen_size = 240*320*2;

	return 0;
}

static inline void ak880xfb_unmap_video_memory(struct fb_info *info)
{
	iounmap( info->screen_base ) ;
}


static char driver_name[] = "ak880xfb";

static int __init ak880xfb_probe(struct platform_device *pdev)
{
	int ret;
	int i = 0, logo_len = 0 ;
	unsigned short * logo_p = NULL ;

	init_timer(&lcd_refresh_timer);
	lcd_refresh_timer.function = jiffies_timer ;

	fbinfo = framebuffer_alloc(0, &pdev->dev);
	if (!fbinfo)
		return -ENOMEM;

	strcpy(fbinfo->fix.id, driver_name);
	fbinfo->fbops			= &ak880xfb_ops;
	fbinfo->flags			= FBINFO_FLAG_DEFAULT;
	fbinfo->fix.type		= FB_TYPE_PACKED_PIXELS;
	fbinfo->var.activate		= FB_ACTIVATE_NOW;
	fbinfo->pseudo_palette		= &pseudo_pal;

#if defined( CONFIG_LCD_HX8352 )
	fbinfo->var.xres		= 320;
	fbinfo->var.yres		= 240;
	fbinfo->var.bits_per_pixel	= 16;

	fbinfo->var.red.offset		= 11;
	fbinfo->var.green.offset	= 5;
	fbinfo->var.blue.offset		= 0;
	fbinfo->var.transp.offset	= 0;
	fbinfo->var.red.length		= 5;
	fbinfo->var.green.length	= 6;
	fbinfo->var.blue.length		= 5;
	fbinfo->var.transp.length	= 0;
	fbinfo->fix.smem_len		= 320*240*16/8 ;
	fbinfo->fix.line_length		=  320*16/8 ;
	fbinfo->fix.visual		= FB_VISUAL_TRUECOLOR;

#else
	fbinfo->var.xres		= 240;
	fbinfo->var.yres		= 320;
	fbinfo->var.bits_per_pixel	= 16;

	fbinfo->var.red.offset		= 11;
	fbinfo->var.green.offset	= 5;
	fbinfo->var.blue.offset		= 0;
	fbinfo->var.transp.offset	= 0;
	fbinfo->var.red.length		= 5;
	fbinfo->var.green.length	= 6;
	fbinfo->var.blue.length		= 5;
	fbinfo->var.transp.length	= 0;
	fbinfo->fix.smem_len		= 240*320*16/8 ;
	fbinfo->fix.line_length		=  240*16/8 ;
	fbinfo->fix.visual		= FB_VISUAL_TRUECOLOR;
#endif

	ret = ak880xfb_map_video_memory(fbinfo);
	if (ret) {
		printk(KERN_ERR "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto dealloc_fb;
	}
	fbinfo->fix.smem_start = 0x30000000 + 0x3f00000 ;

#if 0
	logo_len = sizeof(newplus_logo) ;
	logo_p = (unsigned short *)fbinfo->screen_base ;
	for(i=0; i<logo_len/2; i++ )
		*(logo_p + i ) = 0x00;//newplus_logo[i] ;
#endif

	lcd_auto_refresh = 1 ;
	refresh_count = 0 ;
	start_dma();
	display_refresh();

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		printk(KERN_ERR "Failed to register framebuffer device: %d\n",
			ret);
		goto free_video_memory;
	}

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
		fbinfo->node, fbinfo->fix.id);

	printk("register_framebuffer\n") ;
	return 0;

free_video_memory:
	ak880xfb_unmap_video_memory(fbinfo);
dealloc_fb:
	platform_set_drvdata(pdev, NULL);
	framebuffer_release(fbinfo);
	if( timer_pending( &lcd_refresh_timer ) )
		del_timer( &lcd_refresh_timer ) ;
	return ret;
	return 0;
}


static int ak880xfb_remove(struct platform_device *pdev)
{
	if( timer_pending( &lcd_refresh_timer ) )
		del_timer( &lcd_refresh_timer ) ;
	return 0;
}


static struct platform_driver ak880xfb_driver = {
	.probe		= ak880xfb_probe,
	.remove		= ak880xfb_remove,
	.driver		= {
		.name	= "ak880x-lcd",
		.owner	= THIS_MODULE,
	},
};

int __init ak880xfb_init(void)
{
	printk("AK88 Framebuffer Driver, (c) 2010 ANYKA\n");

	return platform_driver_register(&ak880xfb_driver);
}

static void __exit ak880xfb_cleanup(void)
{
	platform_driver_unregister(&ak880xfb_driver);
}

module_init(ak880xfb_init);
module_exit(ak880xfb_cleanup);

MODULE_AUTHOR("anyka");
MODULE_DESCRIPTION("Framebuffer driver for the ak880x");
MODULE_LICENSE("GPL");
