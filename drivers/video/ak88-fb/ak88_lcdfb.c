/*
 *  Driver for AK88 LCD Controller
 *
 *  Copyright (C) 2010 Anyka Corporation
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/backlight.h>


#include <video/anyka_lcdc.h>

#include <mach/map.h>
#include <mach/devices_ak880x.h>
#include <mach/lib_lcd.h>
#include <mach/ak880x_freq.h>
#include <mach/clock.h>
#include <mach/ak880x_addr.h>


#define	ANYKA_LCDFB_FBINFO_DEFAULT	(FBINFO_DEFAULT \
					 | FBINFO_PARTIAL_PAN_OK \
					 | FBINFO_HWACCEL_YPAN)

static wait_queue_head_t wq;

 
void init_backlight(struct anyka_lcdfb_info *sinfo)
{
	dev_warn(&sinfo->pdev->dev, "backlight control is not available\n");
}

static void exit_backlight(struct anyka_lcdfb_info *sinfo)
{
}

 
static void init_contrast(struct anyka_lcdfb_info *sinfo)
{
	/* have some default contrast/backlight settings */
	printk("init_contrast()\n");

	bsplcd_set_panel_power(1);	//pullup TFT_VGH_L and TFT_AVDD

	baselcd_set_panel_backlight(1);

}

static struct fb_fix_screeninfo anyka_lcdfb_fix __initdata = {
	.type = FB_TYPE_PACKED_PIXELS,	//defined in linux/fb.h
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,		/* zero if no hardware panning  */
	.ypanstep = 1,		/* zero if no hardware panning  */
	.ywrapstep = 0,		//?? I don't know what it means /* zero if no hardware ywrap    */
	.accel = FB_ACCEL_NONE,	/* no hardware accelerator     */
};

unsigned long compute_hozval(unsigned long xres, unsigned long lcdcon2)
{
	unsigned long value;

	value = xres;
	//assume ak880x use TFT PANEL

	return value;
}

static void anyka_lcdfb_stop_nowait(struct anyka_lcdfb_info *sinfo)
{
//stop refresh
//shouldn't write any command for AK88


}


void anyka_lcdfb_stop(struct anyka_lcdfb_info *sinfo)
{
	anyka_lcdfb_stop_nowait(sinfo);

	/* Wait for DMA engine to become idle... */
	msleep(10);

}

void anyka_lcdfb_start(struct anyka_lcdfb_info *sinfo)
{
//start refresh
	LCD_IF_MODE if_mode = LCD_IF_RGB;

	lcd_rgb_set_interface(if_mode);

	lcd_rgb_start_refresh();	

	baselcd_set_panel_backlight(1);


	mdelay(1000);
	printk("anyka_lcdfb_start()\n");

}

static void anyka_lcdfb_update_dma(struct fb_info *info,
				   struct fb_var_screeninfo *var)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	unsigned long dma_addr;

	dma_addr = (fix->smem_start + var->yoffset * fix->line_length
		    + var->xoffset * var->bits_per_pixel / 8);

	dma_addr &= ~3UL;

	/* Set framebuffer DMA base address and pixel offset */

	lcd_fb_init_ram(dma_addr, var->xres, var->yres);

	lcd_controller_fastdma();

	printk("anyka_lcdfb_update_dma()\n");

}

static inline void anyka_lcdfb_free_video_memory(struct anyka_lcdfb_info *sinfo)
{
	struct fb_info *info = sinfo->info;

	dma_free_writecombine(info->device, info->fix.smem_len,
			      info->screen_base, info->fix.smem_start);

	printk("anyka_lcdfb_free_video_memory()\n");

}

/**
 *	anyka_lcdfb_alloc_video_memory - Allocate framebuffer memory
 *	@sinfo: the frame buffer to allocate memory for
 * 	
 * 	This function is called only from the anyka_lcdfb_probe()
 * 	so no locking by fb_info->mm_lock around smem_len setting is needed.
 */
static int anyka_lcdfb_alloc_video_memory(struct anyka_lcdfb_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	struct fb_var_screeninfo *var = &info->var;
	unsigned int smem_len;

	smem_len = (var->xres_virtual * var->yres_virtual
		    * ((var->bits_per_pixel + 7) / 8));
	info->fix.smem_len = max(smem_len, sinfo->smem_len);

	info->screen_base =
	    dma_alloc_writecombine(info->device, info->fix.smem_len,
				   (dma_addr_t *) & info->fix.smem_start,
				   GFP_KERNEL);

	//fix.smem_start : dma address ,the address that be wrote to register

	if (!info->screen_base) {
		return -ENOMEM;
	}

	printk("anyka_lcdfb_alloc_video_memory()\n");

	return 0;
}

static const struct fb_videomode *anyka_lcdfb_choose_mode(struct
							  fb_var_screeninfo
							  *var,
							  struct fb_info *info)
{
	struct fb_videomode varfbmode;
	const struct fb_videomode *fbmode = NULL;

	fb_var_to_videomode(&varfbmode, var);
	fbmode = fb_find_nearest_mode(&varfbmode, &info->modelist);
	if (fbmode)
		fb_videomode_to_var(var, fbmode);

	printk("anyka_lcdfb_choose_mode()\n");
	return fbmode;
}

/**
 *      anyka_lcdfb_check_var - Validates a var passed in.
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Checks to see if the hardware supports the state requested by
 *	var passed in. This function does not alter the hardware
 *	state!!!  This means the data stored in struct fb_info and
 *	struct anyka_lcdfb_info do not change. This includes the var
 *	inside of struct fb_info.  Do NOT change these. This function
 *	can be called on its own if we intent to only test a mode and
 *	not actually set it. The stuff in modedb.c is a example of
 *	this. If the var passed in is slightly off by what the
 *	hardware can support then we alter the var PASSED in to what
 *	we can do. If the hardware doesn't support mode change a
 *	-EINVAL will be returned by the upper layers. You don't need
 *	to implement this function then. If you hardware doesn't
 *	support changing the resolution then this function is not
 *	needed. In this case the driver would just provide a var that
 *	represents the static state the screen is in.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int anyka_lcdfb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct device *dev = info->device;
	struct anyka_lcdfb_info *sinfo = info->par;
	unsigned long clk_value_hz;

	//clk_value_khz = clk_get_rate(sinfo->lcdc_clk) / 1000;

 	//clk_value_hz = ak880x_asicfreq_get();	//124MHZ
	clk_value_hz = (unsigned long)clk_get_rate(clk_get(NULL,"asic_clk"));
 
	dev_dbg(dev, "%s:\n", __func__);

 
	if (!(var->pixclock && var->bits_per_pixel)) {
		/* choose a suitable mode if possible */
		if (!anyka_lcdfb_choose_mode(var, info)) {
			dev_err(dev, "needed value not specified\n");
			printk("error:needed value not specified\n");
			return -EINVAL;
		}
	}

 
	if ((var->pixclock) > clk_value_hz) {
		//dev_err(dev, "%lu KHz pixel clock is too fast\n", PICOS2KHZ(var->pixclock));
		printk("%lu Hz pixel clock is too fast\n",
		       (unsigned long)var->pixclock);
		return -EINVAL;
	}

	/* Do not allow to have real resoulution larger than virtual */
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;

	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;

	/* Force same alignment for each line */
	var->xres = (var->xres + 3) & ~3UL;
	var->xres_virtual = (var->xres_virtual + 3) & ~3UL;

	var->red.msb_right = var->green.msb_right = var->blue.msb_right = 0;
	var->transp.msb_right = 0;
	var->transp.offset = var->transp.length = 0;
	var->xoffset = var->yoffset = 0;

 
	if (info->fix.smem_len) {
		unsigned int smem_len = (var->xres_virtual * var->yres_virtual
					 * ((var->bits_per_pixel + 7) / 8));
		if (smem_len > info->fix.smem_len)
			return -EINVAL;
	}

	/* Saturate vertical and horizontal timings at maximum values */
	var->vsync_len = min_t(u32, var->vsync_len,0x3fU + 1);
	var->upper_margin = min_t(u32, var->upper_margin, 0xffU);
	var->lower_margin = min_t(u32, var->lower_margin, 0xffU);
	var->right_margin = min_t(u32, var->right_margin, 0x7ffU + 1);
	var->hsync_len = min_t(u32, var->hsync_len, 0x3fU + 1);
	var->left_margin = min_t(u32, var->left_margin, 0xffU + 1);

	/* Some parameters can't be zero */
	var->vsync_len = max_t(u32, var->vsync_len, 1);
	var->right_margin = max_t(u32, var->right_margin, 1);
	var->hsync_len = max_t(u32, var->hsync_len, 1);
	var->left_margin = max_t(u32, var->left_margin, 1);

 
	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		var->red.offset = var->green.offset = var->blue.offset = 0;
		var->red.length = var->green.length = var->blue.length
		    = var->bits_per_pixel;
		break;
	case 15:
	case 16:
		if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_RGB) {
			/* RGB:565 mode */
			var->red.offset = 11;
			var->blue.offset = 0;
			var->green.length = 6;
		} else if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_RGB555) {
			var->red.offset = 10;
			var->blue.offset = 0;
			var->green.length = 5;
		} else {
			/* BGR:555 mode */
			var->red.offset = 0;
			var->blue.offset = 10;
			var->green.length = 5;
		}
		var->green.offset = 5;
		var->red.length = var->blue.length = 5;
		break;
	case 32:
		var->transp.offset = 24;
		var->transp.length = 8;
		/* fall through */
	case 24:
		if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_RGB) {
			/* RGB:888 mode */
			var->red.offset = 16;
			var->blue.offset = 0;
		} else {
			/* BGR:888 mode */
			var->red.offset = 0;
			var->blue.offset = 16;
		}
		var->green.offset = 8;
		var->red.length = var->green.length = var->blue.length = 8;
		break;
	default:
		dev_err(dev, "color depth %d not supported\n",
			var->bits_per_pixel);
		return -EINVAL;
	}

	#if 0
	printk("check_var()/var->red.offset=%d,var->blue.offset=%d,var->green.offset=%d,var->red.length=%d,var->blue.length=%d,var->green.length=%d\n",\
	     var->red.offset, var->blue.offset, var->green.offset,\
	     var->red.length, var->blue.length, var->green.length);
	#endif
	
	return 0;
}

/*
 * LCD reset sequence
 */
static void anyka_lcdfb_reset(struct anyka_lcdfb_info *sinfo)
{
	might_sleep();

	baselcd_reset_controller();	//controler reset

	baselcd_reset_panel();	//panel reset
}

/**
 *      anyka_lcdfb_set_par - Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution
 *	of the this particular framebuffer. This function alters the
 *	par AND the fb_fix_screeninfo stored in fb_info. It doesn't
 *	not alter var in fb_info since we are using that data. This
 *	means we depend on the data in var inside fb_info to be
 *	supported by the hardware.  anyka_lcdfb_check_var is always called
 *	before anyka_lcdfb_set_par to ensure this.  Again if you can't
 *	change the resolution you don't need this function.
 *
 */
static int anyka_lcdfb_set_par(struct fb_info *info)
{
	struct anyka_lcdfb_info *sinfo = info->par;
	unsigned long value;
	unsigned long clk_value_hz;
	unsigned long bits_per_line;
	unsigned long pix_factor = 1;
	u32 asic_freq;
	LCD_IF_MODE if_mode = LCD_IF_RGB;

 	asic_freq = (unsigned long)clk_get_rate(clk_get(NULL,"asic_clk"));
 
	might_sleep();

	baselcd_reset_controller();



	//dev_dbg(info->device, "%s:\n", __func__);
	printk("%s:\n", __func__);
	//dev_dbg(info->device, "  * resolution: %ux%u (%ux%u virtual)\n",
	printk("  * resolution: %ux%u (%ux%u virtual)\n",
	       info->var.xres, info->var.yres,
	       info->var.xres_virtual, info->var.yres_virtual);

	anyka_lcdfb_stop_nowait(sinfo);

	if (info->var.bits_per_pixel == 1)
		info->fix.visual = FB_VISUAL_MONO01;
	else if (info->var.bits_per_pixel <= 8)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	bits_per_line = info->var.xres_virtual * info->var.bits_per_pixel;
	info->fix.line_length = DIV_ROUND_UP(bits_per_line, 8);

	/* Re-initialize the DMA engine... */
	dev_dbg(info->device, "  * update DMA engine\n");
	anyka_lcdfb_update_dma(info, &info->var);

	/* ...set frame size and burst length = 8 words (?) */
	value =
	    (info->var.yres * info->var.xres * info->var.bits_per_pixel) / 32;

	/* Now, the LCDC core... */

	/* Set pixel clock */

	//clk_value_khz = clk_get_rate(sinfo->lcdc_clk) / 1000;
	//clk_value_hz = ak880x_asicfreq_get();
	clk_value_hz = (unsigned long)clk_get_rate(clk_get(NULL,"asic_clk"));
 
	value = DIV_ROUND_UP(clk_value_hz / 1000, (info->var.pixclock) / 1000);

	if (value < pix_factor) {
		dev_notice(info->device, "Bypassing pixel clock divider\n");		
	} else {

		lcd_rgb_set_pclk(asic_freq /*124 MHZ */ ,
				 info->var.pixclock /*30 MHZ */ );
	}


	lcd_rgb_set_pclk(asic_freq, info->var.pixclock);


	//printk("lcd_fb_set_timing()\n");
	lcd_fb_set_timing(sinfo, asic_freq);


	lcd_rgb_set_interface(if_mode);

	lcd_rgb_start_refresh();

	baselcd_set_panel_backlight(1);


	//dev_dbg(info->device, "  * DONE\n");

	return 0;
}

static inline unsigned int chan_to_field(unsigned int chan,
					 const struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

/**
 *  	anyka_lcdfb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming
 *      @red: The red value which can be up to 16 bits wide
 *	@green: The green value which can be up to 16 bits wide
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 *
 *  	Set a single color register. The values supplied have a 16 bit
 *  	magnitude which needs to be scaled in this function for the hardware.
 *	Things to take into consideration are how many color registers, if
 *	any, are supported with the current color visual. With truecolor mode
 *	no color palettes are supported. Here a psuedo palette is created
 *	which we store the value in pseudo_palette in struct fb_info. For
 *	pseudocolor mode we have a limited color palette. To deal with this
 *	we can program what color is displayed for a particular pixel value.
 *	DirectColor is similar in that we can program each color field. If
 *	we have a static colormap we don't need to implement this function.
 *
 *	Returns negative errno on error, or zero on success. In an
 *	ideal world, this would have been the case, but as it turns
 *	out, the other drivers return 1 on failure, so that's what
 *	we're going to do.
 */
static int anyka_lcdfb_setcolreg(unsigned int regno, unsigned int red,
				 unsigned int green, unsigned int blue,
				 unsigned int transp, struct fb_info *info)
{
	return 0;
}

static int anyka_lcdfb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	dev_dbg(info->device, "%s\n", __func__);

	anyka_lcdfb_update_dma(info, var);

	return 0;
}

static struct fb_ops anyka_lcdfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = anyka_lcdfb_check_var,
	.fb_set_par = anyka_lcdfb_set_par,
	.fb_setcolreg = anyka_lcdfb_setcolreg,
	.fb_pan_display = anyka_lcdfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

irqreturn_t anyka_lcdfb_interrupt(int irq, void *dev_id)
{
	struct anyka_lcdfb_info *sfb = dev_id;

	if(lcd_controller_isrefreshok())
		wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

/*
 * LCD controller task (to reset the LCD)
 */
static void anyka_lcdfb_task(struct work_struct *work)
{
	struct anyka_lcdfb_info *sinfo =
	    container_of(work, struct anyka_lcdfb_info, task);

	anyka_lcdfb_reset(sinfo);
}

static int __init anyka_lcdfb_init_fbinfo(struct anyka_lcdfb_info *sinfo)
{
	struct fb_info *info = sinfo->info;
	int ret = 0;

	info->var.activate |= FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW;

	dev_info(info->device,
		 "%luKiB frame buffer at %08lx (mapped at %p)\n",
		 (unsigned long)info->fix.smem_len / 1024,
		 (unsigned long)info->fix.smem_start, info->screen_base);

	/* Allocate colormap */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
		dev_err(info->device, "Alloc color map failed\n");

	return ret;
}

void anyka_lcdfb_start_clock(struct anyka_lcdfb_info *sinfo)
{

	//open power clock
//#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)   //0xF000000C
//bit[3], 0 = to enable display controller working clock
//        1 = to disable display controller working clock

	//open LCD controller clock
	//AKCLR_BITS(1UL<<3, AK88_POWER_CLOCK);  //0x0800000C

	lcd_controller_start_clock();

	printk("anyka_lcdfb_start_clock()\n");
}

static void anyka_lcdfb_stop_clock(struct anyka_lcdfb_info *sinfo)
{

	lcd_controller_stop_clock();	

	printk("anyka_lcdfb_stop_clock()\n");
}

static int __init anyka_lcdfb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info;
	struct anyka_lcdfb_info *sinfo;
	struct anyka_lcdfb_info *pdata_sinfo;
	struct fb_videomode fbmode;
	struct resource *regs = NULL;
	struct resource *map = NULL;
	int ret;
	u32 asic_freq;
 
  	asic_freq = (unsigned long)clk_get_rate(clk_get(NULL,"asic_clk"));
 
	dev_dbg(dev, "%s BEGIN\n", __func__);

	ret = -ENOMEM;

	set_ahb_priority();

	baselcd_controller_init(1);

	info = framebuffer_alloc(sizeof(struct anyka_lcdfb_info), dev);

	if (!info) {
		dev_err(dev, "cannot allocate memory\n");
		goto out;
	}

	sinfo = info->par;

	if (dev->platform_data) {

		pdata_sinfo = (struct anyka_lcdfb_info *)dev->platform_data;
		sinfo->default_bpp = pdata_sinfo->default_bpp;

		sinfo->default_monspecs = pdata_sinfo->default_monspecs;
		sinfo->anyka_lcdfb_power_control =
		    pdata_sinfo->anyka_lcdfb_power_control;
		sinfo->guard_time = pdata_sinfo->guard_time;
		sinfo->smem_len = pdata_sinfo->smem_len;
		sinfo->lcdcon_is_backlight = pdata_sinfo->lcdcon_is_backlight;
		sinfo->lcd_wiring_mode = pdata_sinfo->lcd_wiring_mode;

		printk("anyka_lcdfb_probe()/lcd_panel is:%s\n",sinfo->default_monspecs->modedb->name);

	} else {
		dev_err(dev, "cannot get default configuration\n");
		goto free_info;
	}
	sinfo->info = info;
	sinfo->pdev = pdev;

	strcpy(info->fix.id, sinfo->pdev->name);
	info->flags = ANYKA_LCDFB_FBINFO_DEFAULT;
	info->pseudo_palette = sinfo->pseudo_palette;
	info->fbops = &anyka_lcdfb_ops;

	memcpy(&info->monspecs, sinfo->default_monspecs,
	       sizeof(info->monspecs));

	info->fix = anyka_lcdfb_fix;


	ret = fb_find_mode(&info->var, info, NULL, info->monspecs.modedb,
			   info->monspecs.modedb_len, info->monspecs.modedb,
			   sinfo->default_bpp);
	if (!ret) {
		dev_err(dev, "no suitable video mode found\n");
		goto stop_clk;
	}
	info->var.yres_virtual = info->var.yres * 2;
	printk("%s: %ux%u, %ux%u\n", __func__, info->var.xres, info->var.yres,
	       info->var.xres_virtual, info->var.yres_virtual);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "resources unusable\n");
		ret = -ENXIO;
		goto stop_clk;
	}

	sinfo->irq_base = platform_get_irq(pdev, 0);
	if (sinfo->irq_base < 0) {
		dev_err(dev, "unable to get irq\n");
		ret = sinfo->irq_base;
		goto stop_clk;
	}

	/* Initialize video memory */
	map = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (map) {
		/* use a pre-allocated memory buffer */
		info->fix.smem_start = map->start;
		info->fix.smem_len = map->end - map->start + 1;
		if (!request_mem_region(info->fix.smem_start,
					info->fix.smem_len, pdev->name)) {
			ret = -EBUSY;
			goto stop_clk;
		}

		info->screen_base =
		    ioremap(info->fix.smem_start, info->fix.smem_len);
		if (!info->screen_base)
			goto release_intmem;

		/*
		 * Don't clear the framebuffer -- someone may have set
		 * up a splash image.
		 */
	} else {
		/* alocate memory buffer */
		ret = anyka_lcdfb_alloc_video_memory(sinfo);
		if (ret < 0) {
			dev_err(dev, "cannot allocate framebuffer: %d\n", ret);
			goto stop_clk;
		}
	}

	/* LCDC registers */
	info->fix.mmio_start = regs->start;
	info->fix.mmio_len = regs->end - regs->start + 1;

	if (!request_mem_region(info->fix.mmio_start,
				info->fix.mmio_len, pdev->name)) {
		ret = -EBUSY;
		goto free_fb;
	}

	sinfo->mmio = ioremap(info->fix.mmio_start, info->fix.mmio_len);	//ioremap get vitual address.
	if (!sinfo->mmio) {
		dev_err(dev, "cannot map LCDC registers\n");
		goto release_mem;
	}

	/* Initialize PWM for contrast or backlight ("off") */
	init_contrast(sinfo);

	/* interrupt */

	init_waitqueue_head(&wq); 
	
	ret =
	    request_irq(sinfo->irq_base, anyka_lcdfb_interrupt, 0, pdev->name,
			info);
	if (ret) {
		dev_err(dev, "request_irq failed: %d\n", ret);
		goto unmap_mmio;
	}

	/* Some operations on the LCDC might sleep and
	 * require a preemptible task context */
	INIT_WORK(&sinfo->task, anyka_lcdfb_task);

	ret = anyka_lcdfb_init_fbinfo(sinfo);

	if (ret < 0) {
		dev_err(dev, "init fbinfo failed: %d\n", ret);
		goto unregister_irqs;
	}

	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	anyka_lcdfb_check_var(&info->var, info);

	ret = fb_set_var(info, &info->var);
	if (ret) {
		dev_warn(dev, "unable to set display parameters\n");
		printk("unable to set display parameters\n");
		goto free_cmap;
	}

	dev_set_drvdata(dev, info);

	/*
	 * Tell the world that we're ready to go
	 */
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "failed to register framebuffer device: %d\n",
			ret);
		goto reset_drvdata;
	}

	/* add selected videomode to modelist */
	fb_var_to_videomode(&fbmode, &info->var);
	fb_add_videomode(&fbmode, &info->modelist);

	/* Power up the LCDC screen */
	if (sinfo->anyka_lcdfb_power_control)
		sinfo->anyka_lcdfb_power_control(1);

	dev_info(dev,
		 "fb%d: Anyka LCDC at info->fix.mmio_start=0x%08lx (sinfo->mmio=mapped at %p), irq %lu\n",
		 info->node, info->fix.mmio_start, sinfo->mmio,
		 (unsigned long)sinfo->irq_base);

	return 0;

 reset_drvdata:
	dev_set_drvdata(dev, NULL);
 free_cmap:
	fb_dealloc_cmap(&info->cmap);
 unregister_irqs:
	cancel_work_sync(&sinfo->task);
	free_irq(sinfo->irq_base, info);
 unmap_mmio:
	exit_backlight(sinfo);
	iounmap(sinfo->mmio);
 release_mem:
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
 free_fb:
	if (map)
		iounmap(info->screen_base);
	else
		anyka_lcdfb_free_video_memory(sinfo);

 release_intmem:
	if (map)
		release_mem_region(info->fix.smem_start, info->fix.smem_len);
 stop_clk:
	anyka_lcdfb_stop_clock(sinfo);
 free_info:
	framebuffer_release(info);
 out:
	dev_dbg(dev, "%s FAILED\n", __func__);
	return ret;
}

static int __exit anyka_lcdfb_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct anyka_lcdfb_info *sinfo;

	if (!info || !info->par)
		return 0;
	sinfo = info->par;

	cancel_work_sync(&sinfo->task);
	exit_backlight(sinfo);
	if (sinfo->anyka_lcdfb_power_control)
		sinfo->anyka_lcdfb_power_control(0);
	unregister_framebuffer(info);
	anyka_lcdfb_stop_clock(sinfo);
	clk_put(sinfo->lcdc_clk);
	if (sinfo->bus_clk)
		clk_put(sinfo->bus_clk);
	fb_dealloc_cmap(&info->cmap);
	free_irq(sinfo->irq_base, info);
	iounmap(sinfo->mmio);
	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);

	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		iounmap(info->screen_base);
		release_mem_region(info->fix.smem_start, info->fix.smem_len);
	} else {
 		anyka_lcdfb_free_video_memory(sinfo);
	}

	dev_set_drvdata(dev, NULL);
	framebuffer_release(info);

	return 0;
}

#ifdef CONFIG_PM

static int anyka_lcdfb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct fb_info *info = platform_get_drvdata(pdev);
 	struct anyka_lcdfb_info *sinfo = info->par;

	/*
	 * We don't want to handle interrupts while the clock is
	 * stopped. It may take forever.
	 */

	if (sinfo->anyka_lcdfb_power_control)
		sinfo->anyka_lcdfb_power_control(0);

	anyka_lcdfb_stop(sinfo);
	anyka_lcdfb_stop_clock(sinfo);

	return 0;
}

static int anyka_lcdfb_resume(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct anyka_lcdfb_info *sinfo = info->par;

	anyka_lcdfb_start_clock(sinfo);
	anyka_lcdfb_start(sinfo);
	if (sinfo->anyka_lcdfb_power_control)
		sinfo->anyka_lcdfb_power_control(1);

	return 0;
}

#else
#define anyka_lcdfb_suspend	NULL
#define anyka_lcdfb_resume	NULL
#endif

static struct platform_driver anyka_lcdfb_driver = {
	.remove = __exit_p(anyka_lcdfb_remove),
	.suspend = anyka_lcdfb_suspend,
	.resume = anyka_lcdfb_resume,

	.driver = {
		   .name = "anyka_lcdfb",
		   .owner = THIS_MODULE,
		   },
};

static int __init anyka_lcdfb_init(void)
{
	return platform_driver_probe(&anyka_lcdfb_driver, anyka_lcdfb_probe);
}

static void __exit anyka_lcdfb_exit(void)
{
	platform_driver_unregister(&anyka_lcdfb_driver);
}

module_init(anyka_lcdfb_init);
module_exit(anyka_lcdfb_exit);

MODULE_DESCRIPTION("AK88 LCD Controller framebuffer driver");
MODULE_AUTHOR("Anyka");
MODULE_LICENSE("GPL");
