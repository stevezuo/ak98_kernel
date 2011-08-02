
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>

#include <video/anyka_lcdc.h>

#include <asm/io.h>

#include <mach/map.h>
#include <mach/lib_l2.h>
#include <mach/ak880x_gpio.h>
#include <mach/ak880x_freq.h>
#include <mach/lib_lcd.h>

#if 1
//static struct TFT_LCD_TIMING_INFO hsd070idw1_a10 = {
static struct tft_lcd_timing_info hsd070idw1_a10 = {

	.bus_width = 24,
	.interlace = 1,		//1:no interlace
	.hvg_pol = 0xe,		//h_sync(bit2),v_sync(bit1),vogate(bit0) polarity,0=positive,1=negative
	.rgb_bit = 0x0888,	//RGB or GBR(bit12, if 1:BGR),R(bits 11:8),G(bit 7:4),B(bit 3:0) bits

	.clock_hz = 30000000,	//(30M)  //bit clock, clock cycle,unit HZ

	.h_sync_hz = 15711,	//??       //h_sync cycle,unit HZ
	.v_sync_001hz = 5885,	//v_sync cycle,unit 0.01HZ

	.h_tot_clk = 1058,	//horizontal total cycle,unit clock
	.h_disp_clk = 800,	//horizontal display cycle,unit clock
	.h_front_porch_clk = 170,	//right_margin //horizontal front porch,unit clock
	.h_pulse_width_clk = 48,	//horizontal pulse width,unit clock
	.h_back_porch_clk = 40,	//left_margin //horizontal back porch,unit clock

	.v_tot_clk = 553,	//vertical   total cycle,unit h_sync
	.v_disp_clk = 480,	//vertical   display cycle,unit h_sync
	.v_front_porch_clk = 40,	//lower_margin //vertical   front porch,unit clock
	.v_pulse_width_clk = 4,	//vertical   pulse width,unit clock
	.v_back_porch_clk = 29,	//upper_margin//vertical   back porch,unit clock

	.name = "HSD070IDW1-A00",

};
#endif

static struct tft_lcd_timing_info q07021_701 = {

	.bus_width = 24,
	.interlace = 1,		//1:no interlace
	.hvg_pol = 0xe,		//h_sync(bit2),v_sync(bit1),gate(bit0) polarity
	.rgb_bit = 0x0888,	//RGB or GBR(bit12, if 1:BGR),R(bits 11:8),G(bit 7:4),B(bit 3:0) bits

	.clock_hz = 40000000,	//(30M)  //bit clock, clock cycle,unit HZ

	.h_sync_hz = 15711,	//??       //h_sync cycle,unit HZ
	.v_sync_001hz = 5885,	//v_sync cycle,unit 0.01HZ

	.h_tot_clk = 1000,	//horizontal total cycle,unit clock
	.h_disp_clk = 800,	//horizontal display cycle,unit clock
	.h_front_porch_clk = 112,	//horizontal front porch,unit clock
	.h_pulse_width_clk = 48,	//horizontal pulse width,unit clock
	.h_back_porch_clk = 88,	//horizontal back porch,unit clock

	.v_tot_clk = 660,	//vertical   total cycle,unit h_sync
	.v_disp_clk = 600,	//vertical   display cycle,unit h_sync
	.v_front_porch_clk = 21,	//vertical   front porch,unit clock
	.v_pulse_width_clk = 3,	//vertical   pulse width,unit clock
	.v_back_porch_clk = 39,	//vertical   back porch,unit clock

	.name = "Q07021-701",

};

static LCD_IF_MODE if_mode;

static u32 disp_buf_size;

static struct lcd_size lcd_size;

static struct display_ram display_ram;

static struct picture_area usr_rgb_area;

void lcd_print_reg(void)
{
	printk("LCD_TOP_CONFIGURE 0x20010000 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0000));
	printk("LCD_MPU_1 0x20010004 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0004));
	printk("LCD_RGB_CONTROL 0x20010014 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0014));
	printk("RGB_BACKGROUND 0x2001003C =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x003C));

	printk("Y1_ADDR 0x2001005C =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x005C));
	printk("U1_ADDR 0x20010060 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0060));
	printk("V1_ADDR 0x20010064 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0064));

	printk("YUV1_H_INFO 0x20010068 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0068));
	printk("YUV1_V_INFO 0x2001006C =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x006C));
	printk("YUV1_SCALE_INFO 0x20010070 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0070));
	printk("YUV1_DISPLAY_INFO 0x20010074 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0074));
	printk("YUV1_VIRTUAL_SIZE 0x20010078 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0078));
	printk("YUV1_VIRTUAL_OFFSET 0x2001007C =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x007C));

	printk("Y2_ADDR 0x20010080 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0080));
	printk("U2_ADDR 0x20010084 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0084));
	printk("V2_ADDR 0x20010088 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0088));

	printk("YUV2_H_INFO 0x2001008C =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x008C));
	printk("YUV2_V_INFO 0x20010090 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0090));
	printk("YUV2_SCALE_INFO 0x20010094 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0094));
	printk("YUV2_DISPLAY_INFO 0x20010098 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x0098));

	printk("RGB_OFFSET 0x200100A8 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x00A8));
	printk("RGB_SIZE 0x200100AC =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x00AC));
	printk("PANEL_SIZE 0x200100B0 =0x%x\n",
	       *(volatile unsigned int *)(AK88_VA_DISP + 0x00B0));

}

void set_ahb_priority(void)
{
	unsigned long reg;

	reg = __raw_readl(AK88_AHB_PRIORITY);
	reg &= ~0x7f;		//set all dma priority higher than ARM priority
	__raw_writel(reg, AK88_AHB_PRIORITY);
}

//from BSP_LCD.c
void bsplcd_set_panel_power(int en /*1=open,0=close */ )	//pullup TFT_VGH_L and TFT_AVDD
{
	unsigned int pin;

#if 0
	int i;
	for (i = 0; i < 127; i++)	//debug ,set all gpio pin to output and pull high
	{
		if(i==AK8802_GPIO_57)
		{
			if (en)
				gpio_set_output(i, 0);	//open,set to output and pull high
 			else
				gpio_set_output(i, 1);	//close,set to output and pull low
		}else
		{
		if (en)
			gpio_set_output(i, 1);	//open,set to output and pull high
			//gpio_set_output(i, 0);	//open,set to output and pull high
		else
			gpio_set_output(i, 0);	//close,set to output and pull low
			//gpio_set_output(i, 1);	//close,set to output and pull low
		}	
	}
#endif

#ifdef CONFIG_BOARD_AK8802EBOOK	

	//lcd pin name:TFT_VCC3V3 
	//output, pull LOW

	//pin=AK88_GPIO_DGPIO26; //for 7801 or 8801
	//pin = AK8802_GPIO_RESV91;	//for 7802 or 8802

	pin=	AK8802_GPIO_57;  //pull LOW

	AKCLR_BITS(1UL << 24, AK88_SHAREPIN_CTRL);	//set pin as gpio[58:47]

	if (en)
		gpio_set_output(pin, 0);	//open,set to output and pull LOW
	else
		gpio_set_output(pin, 1);	//close,set to output and pull HIGH

	//lcd pin name:TFT_AVDD
	//pin=AK88_GPIO_DGPIO25;      //for 7801 or 8801
	//pin = AK8802_GPIO_RESV90;	//for 7802 or 8802
	pin = AK8802_GPIO_01;	//

	AKCLR_BITS(1UL << 0, AK88_SHAREPIN_CTRL);	//set pin as gpio[3:0]

 	if (en)
		gpio_set_output(pin, 1);	//open,set to output and pull high
	else
		gpio_set_output(pin, 0);	//close,set to output and pull low

#else

#ifdef CONFIG_BOARD_AK8801EPC	

	//lcd pin name:TFT_VGH_L
	//pin=AK88_GPIO_DGPIO26;      //for 7801 or 8801
	pin = AK8802_GPIO_RESV91;	//for 7802 or 8802

	if (en)
		gpio_set_output(pin, 1);	//open,set to output and pull high
 	else
		gpio_set_output(pin, 0);	//close,set to output and pull low

	//lcd pin name:TFT_AVDD
	//pin=AK88_GPIO_DGPIO25;  //for 7801 or 8801
	pin = AK8802_GPIO_RESV90;   //for 7802 or 8802

 	if (en)
		gpio_set_output(pin, 1);	//open,set to output and pull high
	else
		gpio_set_output(pin, 0);	//close,set to output and pull low

#endif

#endif

}

// from base_LCDController.cpp

void baselcd_reset_panel(void)	//lcd panel reset
{
	//0x2001,0008   
	//bit[31:1]: reserved
	//bit[0]:rst:   0=To send a reset signal to LCD panel
	//              1=No instruction

	AKSET_BITS(1UL << 0, AK88_LCD_REST_SIGNAL);
	ak880x_sdelay(2);

	AKCLR_BITS(1UL << 0, AK88_LCD_REST_SIGNAL);
	ak880x_sdelay(2);

	AKSET_BITS(1UL << 0, AK88_LCD_REST_SIGNAL);
	ak880x_sdelay(2);

}

void baselcd_set_panel_backlight(int en /*en=0:close; en=1:open */ ) {
	unsigned int pin;

#if 1				//debug for ak8802
	//lcd pin name: LCD_BACK_LIGHT

//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
//bit[4]: 1=corresponding are used as {#PWM1]}     //gpio[9] 
//        0=corresponding are used as {#gpio[9]}  
	//AKSET_BITS(1UL<<4,AK88_SHAREPIN_CTRL);   //set pin as #PWM1
	AKCLR_BITS(1UL << 4, AK88_SHAREPIN_CTRL);	//set pin as gpio[9]

	pin = AK8802_GPIO_09;	//

	//gpio_set_dir(pin,0);  //set output dir

	if (en)			//open
	{
		//for(i=0;i<10;i++)
		{
			//gpio_set_level(pin,0); //pull low
			gpio_set_output(pin, 0);	//set to output and pull low
			ak880x_sdelay(1);
			//gpio_set_level(pin,1);  //pull high
			gpio_set_output(pin, 1);	//set to output and pull low
			ak880x_sdelay(1);
		}
	} else {
		//gpio_set_level(pin,0);
		gpio_set_output(pin, 0);	//set to output and pull low
	}
#endif

}

void baselcd_reset_controller(void)	//to rest power clock reg bit19
{
	//0x0800,000c
	//bit[19]:0=no instruction
	//        1=to reset display controller

	//reset lcd interface
	AKSET_BITS(1 << 19, AK88_POWER_CLOCK);	//0x0800000C
	//bit[19]: to reset display controller

	ak880x_sdelay(1);

	AKCLR_BITS(1 << 19, AK88_POWER_CLOCK);	//0x0800000C
	ak880x_sdelay(1);

	//open LCD controller clock
	AKCLR_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

}

void baselcd_controller_init(int en)
{
	//mapping 0x2001,0000 AK88_VA_DISP,has done in map_desc ak880x_iodesc[]
	//mapping 0x0800,0000 AK88_VA_SYS,has done in map_desc ak880x_iodesc[]

	baselcd_reset_controller();	//controler reset

	baselcd_reset_panel();	//panel reset

	//set share pin
	//set sharepin
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
//bit[25]: 1=corresponding are used as {LCD_DATA[15:9]}  //gpio[68:62]
//bit[26]: 1=corresponding are used as {LCD_DATA[8]}     //gpio[61]
//bit[27]: 1=corresponding are used as {LCD_DATA[17:16]} //gpio[70:69] 
//bit[28]: 1=corresponding are used as {#MPU_RST]}       //gpio[71] 
	if (en) {
		AKSET_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[15:9]
		AKSET_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[8]
		AKSET_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[17:16]
		AKSET_BITS(1UL << 28, AK88_SHAREPIN_CTRL);	//set pin as #MPU_RST
	} else {

		AKCLR_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as gpio[68:62]
		AKCLR_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as gpio[61]
		AKCLR_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as gpio[70:69]
		AKCLR_BITS(1UL << 28, AK88_SHAREPIN_CTRL);	//set pin as gpio[71]

	}

	if (en)
		//open LCD controller clock
		AKCLR_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C
	else
		//close LCD controller clock
		AKSET_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

	//panel power on
	bsplcd_set_panel_power(en);	//pullup TFT_VGH_L and TFT_AVDD

	//panel backlight
	baselcd_set_panel_backlight(en);

}

bool lcd_controller_isrefreshok(void)
{
	int status = 0;
	status = __raw_readl(AK88_LCD_STATUS_REG);
	if(1UL<<3 & status)
		return true;
	else
		return false;
}

void lcd_controller_start_clock(void)
{

	//open power clock
//#define AK88_POWER_CLOCK    (AK88_VA_SYS+0x000C)   //0xF000000C

//bit[3], 0 = to enable display controller working clock
//        1 = to disable display controller working clock

	//open LCD controller clock
	AKCLR_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

}

void lcd_controller_stop_clock(void)
{

	//close LCD controller clock
	AKSET_BITS(1UL << 3, AK88_POWER_CLOCK);	//0x0800000C

}

void lcd_controller_fastdma(void)
{
	//enable for DMA
	//0x2001,00c8
	//bit[18]:sw_cs:        When the display controller is in RGB mode,this bit can control 
	//                      the signal level of pin #RGB_CS just like a GPIO
	//
	//                      0=The signal of pin #RGB_CS is LOW
	//                      1=The signal of pin #RGB_CS is HIGH
	//                      
	//bit[17]:fast_dma:     0=fast DMA function is OFF;
	//                      1=fast DMA function is ON.
	//Note:
	//      The fast DMA operation can transmit Cb and Cr data consecutively.
	//      which will shorten the DMA operation time.If programmers want to 
	//      read back the value of this register,this bit will always be 0,even
	//      when the fast DMA function is on.
	//

	//AKCLR_BITS(0x1UL<<18,AK88_LCD_SOFT_CTRL);  //#RGB_CS is LOW
	AKSET_BITS(0x1UL << 18, AK88_LCD_SOFT_CTRL);	//#RGB_CS is HIGH

	AKSET_BITS(0x1UL << 17, AK88_LCD_SOFT_CTRL);
}

//from ak780x_lcd.c

//static void lcd_rgb_set_pclk(unsigned long asic_clk,unsigned lcd_clk)
void lcd_rgb_set_pclk(unsigned long asic_clk, unsigned lcd_clk)
/* 
*  be called in void lcd_rgb_init(..)
*  lcd_rgb_set_pclk(PLLFreq,pLcdInfo->ClockCycle_Hz);
*  //PLLFreq=124000000; //(124M)
*  //pLcdInfo->ClockCycle_Hz=30000000;//(30M);
*/
{

	//0x2001,00e8
	//bit[8]:   1=lcd_clk enable
	//bit[7:1]: lcd_clk=pll1_clk/((x+1)*2) ; so: x=(pll1_clk/(lcd_clk*2)) -1 ;
	//          asic_clk=pll1_clk/2; so x=asic_clk/lcd_clk -1;	
	//bit[0]:   1=lcd_clk cfg valid

	int div;
	div = asic_clk / lcd_clk - 1;
 
	printk("lcd_rgb_set_pclk(),asic_clk=0x%d,lcd_clk=0x%d,div=%d\n",
	       (u32) asic_clk, (u32) lcd_clk, (u32) div);

	//if asic_clk=124M, lcd_clk=30M
	// then div=(124/30*2) -1 = 2-1=1

	if (div < 1)
		div = 1;
	div &= 0x7f;

	AKSET_BITS((1 << 8) | (1 << 0) | (div << 1), AK88_LCD_CLOCK_CONF);
	div = __raw_readl(AK88_LCD_CLOCK_CONF);

	printk("AK88_LCD_CLOCK_CONF=0x%x=0x%x\n", (u32) AK88_LCD_CLOCK_CONF,
	       (u32) div);

}

static void lcd_rgb_set_disp_ram(unsigned long virpage_physical_base_addr)
{
//0x2001,0014
//bit[28]:rgb_vir_en,0=to disable virtual page function of RGB channel
//                   1=to enable virtual page function of RGB channel
//bit[27:0] CFBA, Staring buffer address of current frame.

	unsigned long val, tmp;

	val = virpage_physical_base_addr & 0xfffffff;	//fetch bit[27:0]
	tmp = __raw_readl(AK88_LCD_RGB_CTRL2);
	tmp = tmp & 0xf0000000;	//fetch bit[31:28]
	val = tmp | val;

	printk("lcd_rgb_set_disp_ram,val=0x%x\n", (u32) val);

	__raw_writel(val, AK88_LCD_RGB_CTRL2);
}

//static void lcd_osd_set_disp_ram(unsigned long osd_base_addr)
void lcd_osd_set_disp_ram(unsigned long osd_base_addr)
{
//0x2001,0020
//bit[31:28] reserved
//bit[27:0] OSD ADDR, Staring address of externam RAM from which the data 
//be sent to OSD channel

	unsigned long val;
	val = osd_base_addr & 0xfffffff;	//bit[27:0]

	__raw_writel(val, AK88_LCD_OSD_ADDR);
}

static void lcd_rgb_set_virtual_page(unsigned long virpage_en,
				     unsigned long virpage_hlen,
				     unsigned long virpage_vlen)
{
//0x2001,0014
//bit[28]:rgb_vir_en,0=to disable virtual page function of RGB channel
//                   1=to enable virtual page function of RGB channel
//bit[27:0] CFBA, Staring buffer address of current frame.

//0x2001,0018
//bit[31:16] rgb_virpage_hlen ,horizontal length of virtual page
//bit[15:0]  rgb_virpage_vlen, vertical length of virtual page

	//unsigned long val;

	if (virpage_en)
		AKSET_BITS(1 << 28, AK88_LCD_RGB_CTRL2);
	else
		AKCLR_BITS(1 << 28, AK88_LCD_RGB_CTRL2);

	__raw_writel(((virpage_hlen & 0xffff) << 16) | (virpage_vlen & 0xffff),
		     AK88_LCD_RGB_VIRPAGE_SIZE);
}

static void lcd_rgb_set_virtual_offset(unsigned long virpage_hoffset,
				       unsigned long virpage_voffset)
{
//define the real display area offset compare to virtual page

//0x2001,001C
//bit[31:16] rgb_virpage_hoffset ,horizontal offset of virtual page(unit:pixel)
//bit[15:0]  rgb_virpage_voffset, vertical offset of virtual page(unit:pixel)

	//unsigned long val;

	__raw_writel(((virpage_hoffset & 0xffff) << 16) |
		     (virpage_voffset & 0xffff), AK88_LCD_RGB_VIRPAGE_OFFSET);
}

static void lcd_rgb_set_picture(unsigned long pic_hoffset,
				unsigned long pic_voffset,
				unsigned long pic_hsize,
				unsigned long pic_vsize)
// define the display area compare to real physical page
//
{
//0x2001,00A8
//bit[31:20],reservd
//bit[19:10],H_offset, Horizontal offset value of the picture from RGB channel,(unit:pixel)
//bit[9:0],V_offset,Vertical offset value of the picture from RGB channel,(unit:pixel)

//0x2001,00AC
//bit[31:20],reservd
//bit[19:10],H_length, Horizontal length value of the picture from RGB channel,(unit:pixel)
//bit[9:0],V_length,Vertical length value of the picture from RGB channel,(unit:pixel)

	//unsigned long val;

	__raw_writel(((pic_hoffset & 0x3ff) << 10) | (pic_voffset & 0x3ff),
		     AK88_LCD_RGB_OFFSET);

	__raw_writel(((pic_hsize & 0x3ff) << 10) | (pic_vsize & 0x3ff),
		     AK88_LCD_RGB_SIZE);

}

bool lcd_init_display_ram(struct display_ram *p_disp_ram,
			  struct lcd_size *p_lcd_size)
{
//disable virtual page, set picture area = real physical display area

	lcd_rgb_set_disp_ram((u32) p_disp_ram->p_rgb_base1);

	lcd_rgb_set_virtual_page(0, p_lcd_size->w_pixel, p_lcd_size->h_pixel);
	//disable virtual page function
	//virtual page size = real page size

	lcd_rgb_set_virtual_offset(0, 0);

	lcd_rgb_set_picture(0, 0, p_lcd_size->w_pixel, p_lcd_size->h_pixel);
	//pictual area = real physical display area

	lcd_osd_set_disp_ram((u32) p_disp_ram->p_osd_base);

	lcd_controller_fastdma();

	//set bg color
	//0x2001,003c background color register
	//bit[31:24]: reserved
	//bit[23:0]:  back_color. default background color

	__raw_writel(0x5a5a5a, AK88_LCD_BKG_COLO);
	//__raw_writel(0xffffff,AK88_LCD_BKG_COLO);

	return true;
}

bool lcd_fb_init_ram(unsigned long dma_addr, unsigned long xres,
		     unsigned long yres)
{
//disable virtual page, set picture area = real physical display area

	lcd_rgb_set_disp_ram((u32) dma_addr);

	lcd_rgb_set_virtual_page(0, xres, yres);
	//disable virtual page function
	//virtual page size = real page size

	lcd_rgb_set_virtual_offset(0, 0);

	lcd_rgb_set_picture(0, 0, xres, yres);
	//pictual area = real physical display area

	//lcd_osd_set_disp_ram((u32)p_disp_ram->p_osd_base);

	lcd_controller_fastdma();

	//set bg color
	//0x2001,003c background color register
	//bit[31:24]: reserved
	//bit[23:0]:  back_color. default background color

	__raw_writel(0x5a5a5a, AK88_LCD_BKG_COLO);

	return true;
}

static void lcd_rgb_set_timing(struct tft_lcd_timing_info *p_tft)
{
	u32 temp;
	bool swap_bgr;
	int  buswidthbit = 0;
	
	//setlect rgb interface
	//0x21000000    
	//bit[6:5]: if_mode: LCD interface mode selection
	//              00:reserved
	//              01=MPU interface
	//              10=RGB interface
	//              11=TV interface

	AKCLR_BITS(0x3UL << 5, AK88_LCD_CMD_REG1);
	AKSET_BITS(0x2UL << 5, AK88_LCD_CMD_REG1);

	//0x20010010,AK88_LCD_RGB_CTRL1
	//bit[22:21]:width_sel: 00=8bits,01=16bits,10=18bits,11=reserved
	//bit[20]:RGB_panel_mode:0=reserved,1=non interlace
	//bit[2]:h_pol:0=positive,1=negative,def=0(positive)
	//bit[1]:v_pol:def=0(positive)
	//bit[0]:g_pol RGB_VOGATE''s polarity,def=0(positive)
	//AKSET_BITS((0x2UL<<21)|(p_tft->interlace<<20)|(p_tft->hvg_pol&0x07), AK88_LCD_RGB_CTRL1);
	if(8 == p_tft->bus_width)
	{
		buswidthbit = 0x0<<21;
	}
	else if(16 == p_tft->bus_width)
	{
		buswidthbit = 0x1UL<<21;
	}
	else if(18 == p_tft->bus_width)
	{
		buswidthbit = 0x10UL<<21;
	}
		
	AKSET_BITS((buswidthbit) | (p_tft->interlace << 20) |
		   (p_tft->hvg_pol & 0x07), AK88_LCD_RGB_CTRL1);
	//progress (1<<20)          0xe

	//h_pulse_width_clk and v_pulse_width_clk
	//0x20010040
	AKCLR_BITS(0xfff << 12, AK88_LCD_RGB_CTRL3);
	AKCLR_BITS(0xfff << 0, AK88_LCD_RGB_CTRL3);
	AKSET_BITS((p_tft->h_pulse_width_clk << 12) | (p_tft->
						       v_pulse_width_clk << 0),
		   AK88_LCD_RGB_CTRL3);

	//h_back_porch_clk and h_disp_clk
	//0x20010044
	AKCLR_BITS(0xfff << 12, AK88_LCD_RGB_CTRL4);
	AKCLR_BITS(0xfff << 0, AK88_LCD_RGB_CTRL4);
	AKSET_BITS((p_tft->h_back_porch_clk << 12) | (p_tft->h_disp_clk << 0),
		   AK88_LCD_RGB_CTRL4);

	//h_front_porch_clk and h_tot_clk
	//0x20010048
	AKCLR_BITS(0xfff << 13, AK88_LCD_RGB_CTRL5);
	AKCLR_BITS(0x1fff << 0, AK88_LCD_RGB_CTRL5);
	AKSET_BITS((p_tft->h_front_porch_clk << 13) | (p_tft->h_tot_clk << 0),
		   AK88_LCD_RGB_CTRL5);
	//1058
	//v_back_porch_clk
	//0x2001004C
	AKCLR_BITS(0xfff << 0, AK88_LCD_RGB_CTRL6);
	AKSET_BITS(p_tft->v_back_porch_clk << 0, AK88_LCD_RGB_CTRL6);

	//v_front_porch_clk
	//0x20010050
	AKCLR_BITS(0xfff << 0, AK88_LCD_RGB_CTRL7);
	AKSET_BITS(p_tft->v_front_porch_clk << 0, AK88_LCD_RGB_CTRL7);

	//v_disp_clk
	//0x20010054
	AKCLR_BITS(0xfff << 15, AK88_LCD_RGB_CTRL8);
	AKCLR_BITS(0xfff << 1, AK88_LCD_RGB_CTRL8);
	AKSET_BITS(p_tft->v_disp_clk << 15, AK88_LCD_RGB_CTRL8);

	//v_tot_clk
	//0X20010058
	AKCLR_BITS(0x1fff << 0, AK88_LCD_RGB_CTRL9);
	AKSET_BITS(p_tft->v_tot_clk << 0, AK88_LCD_RGB_CTRL9);

	//553
	//set rgb or bgr and alr

	//CMD1_ALR relate to what???

	temp = __raw_readl(AK88_LCD_CMD_REG1);	//0x2001,0000    
	temp &= ~((0xffUL << 24) | (0xffUL << 16));

	//clk polarity
	if (p_tft->hvg_pol & 0x8)	//0xe    
		temp |= 1UL << 4;
	else
		temp &= ~(1UL << 4);

	temp |= ((0x80 << 24) | (0xa8 << 16));

	//swap_bgr=bsp_get_lcd_rbg_swap();
	swap_bgr = true;
	if ((p_tft->rgb_bit & (1UL << 12)) == 0)	//bit12=1:BGR
		//if((p_tft->rgb_bit & (1UL<<12))==1) //bit12=1:RGB
	{			//RGB
		if (swap_bgr)
			temp &= ~(1UL << 15);
		else
			temp |= 1UL << 15;
	} else {
		if (swap_bgr)
			temp |= 1UL << 15;
		else
			temp &= ~(1UL << 15);
	}

	AKSET_BITS(temp, AK88_LCD_CMD_REG1);	//0x2001,0000
	printk("lcd width=%d,lcd height=%d\n", p_tft->h_disp_clk,
	       p_tft->v_disp_clk);

	//set diaplay area size
	__raw_writel(0, AK88_LCD_DISP_AREA);	//0x2001,00B0
	AKSET_BITS((p_tft->h_disp_clk << 10) | (p_tft->v_disp_clk << 0), AK88_LCD_DISP_AREA);	//0x2001,00B0

}

void lcd_update(void)
{
	//0x2001,00c8
	//bit[11]:sw_en:1=all the necessary infomation about next frame has been set properly
	//              note:This bit is cleared automatically after writing 1
	AKSET_BITS(1UL << 11, AK88_LCD_SOFT_CTRL);	//0x2001,00c8

	//lcd_controller_fastdma();

}

void lcd_rgb_start_refresh(void)
{
	//int i;

	lcd_controller_fastdma();

	//0x2001,00c8
	//bit[11]:sw_en:1=all the necessary infomation about next frame has been set properly
	//              note:This bit is cleared automatically after writing 1
	AKSET_BITS(1UL << 11, AK88_LCD_SOFT_CTRL);	//0x2001,00c8
	AKCLR_BITS(1UL << 0, AK88_LCD_OPER_REG);	//start LCD refresh
	AKSET_BITS(1UL << 2, AK88_LCD_OPER_REG);
}

void lcd_rgb_stop_refresh(void)
{
	AKSET_BITS(1UL << 0, AK88_LCD_OPER_REG);
	mdelay(100);
	AKSET_BITS(1UL << 0, AK88_LCD_OPER_REG);
	mdelay(100);
	printk("lcd stop refresh\n");
}

void lcd_wait_status(int mask) 
{
	int status = 0;
	printk("+lcd_wait_status\n");
	//mask = ~(1<<17);

	//lcd_dump_reg();
	
	do
	{
		status = (int)__raw_readl(AK88_LCD_STATUS_REG);
	}while(!(status & mask));
	//lcd_dump_reg();
	printk("-lcd_wait_status, status%x\n", status);
}
void lcd_interrupt_mask(unsigned bits_result, bool disable)
{
	//0x2001,00C0(LCD int enable reg) <==> 0x2001,00BC
	//bitn : 0 = to disable correspondent interrupt
	//       1 = to enable  correspondent interrupt
	//       default=0
	//

	if (disable == true)	//disable=true, disable the interrupt
		AKCLR_BITS(bits_result, AK88_LCD_INT_ENAB);
	else
		AKSET_BITS(bits_result, AK88_LCD_INT_ENAB);

}

//static irqreturn_t anyka_lcdfb_interrupt(int irq, void *dev_id)
//static void anyka_lcdfb_interrupt_service(void)
void anyka_lcdfb_interrupt_service(void)
{
	//struct fb_info *info = dev_id;
	//struct atmel_lcdfb_info *sinfo = info->par;
	unsigned long status;

#if 0
	status = lcdc_readl(sinfo, ATMEL_LCDC_ISR);
	if (status & ATMEL_LCDC_UFLWI) {
		dev_warn(info->device, "FIFO underflow %#x\n", status);
		/* reset DMA and FIFO to avoid screen shifting */
		schedule_work(&sinfo->task);
	}
	lcdc_writel(sinfo, ATMEL_LCDC_ICR, status);
#endif

	status = __raw_readl(AK88_LCD_STATUS_REG);

	lcd_interrupt_mask(STATUS_SYS_ERROR, false);	//umask(enable) the interrupt

	lcd_interrupt_mask(STATUS_RGB_REFRESH_START, false);	//umask(enable) the interrupt

	if (status & STATUS_SYS_ERROR) {
		//dev_warn(info->device, "LCD:SYS_ERROR %#x\n", status);
		printk("LCD:SYS_ERROR recover now\n");

		//stop first
		//lcd_rgb_stop_refresh();
		//refresh
		//lcd_rgb_start_refresh();

		/* reset DMA and FIFO to avoid screen shifting */
		//schedule_work(&sinfo->task);          
	}

	if (status & STATUS_RGB_REFRESH_START) {
		//dev_warn(info->device, "LCD:RGB_REFRESH_START %#x\n", status);
		printk("LCD:mask interrupt of RGB_REFRESH_START now\n");

		lcd_interrupt_mask(STATUS_RGB_REFRESH_START, true);	//mask(disable) the interrupt

		/* reset DMA and FIFO to avoid screen shifting */
		//schedule_work(&sinfo->task);          
	}
	//return IRQ_HANDLED;
}

//#define AK88_DUMP_REG(x) { printk("reg 0x%x=0x%x\n",(u32)AK88_##x,(u32)__raw_readl(AK88_##x)); }

void lcd_dump_reg(void)
{
	//u32 cnt,temp;
	printk("AK88_POWER_CLOCK 0x%x=0x%x\n",
	       (unsigned int)AK88_POWER_CLOCK,
	       (unsigned int)__raw_readl(AK88_POWER_CLOCK));

	AK88_DUMP_REG(LCD_CMD_REG1);
	AK88_DUMP_REG(LCD_REST_SIGNAL);
	AK88_DUMP_REG(LCD_RGB_CTRL1);
	AK88_DUMP_REG(LCD_RGB_CTRL2);
	AK88_DUMP_REG(LCD_RGB_VIRPAGE_SIZE);
	AK88_DUMP_REG(LCD_RGB_VIRPAGE_OFFSET);
	AK88_DUMP_REG(LCD_OSD_ADDR);
	AK88_DUMP_REG(LCD_BKG_COLO);
	AK88_DUMP_REG(LCD_RGB_CTRL3);
	AK88_DUMP_REG(LCD_RGB_CTRL4);
	AK88_DUMP_REG(LCD_RGB_CTRL5);
	AK88_DUMP_REG(LCD_RGB_CTRL6);
	AK88_DUMP_REG(LCD_RGB_CTRL7);
	AK88_DUMP_REG(LCD_RGB_CTRL8);
	AK88_DUMP_REG(LCD_RGB_CTRL9);
	AK88_DUMP_REG(LCD_RGB_OFFSET);
	AK88_DUMP_REG(LCD_RGB_SIZE);
	AK88_DUMP_REG(LCD_DISP_AREA);
	AK88_DUMP_REG(LCD_CMD_REG2);
	AK88_DUMP_REG(LCD_OPER_REG);
	AK88_DUMP_REG(LCD_STATUS_REG);
	AK88_DUMP_REG(LCD_INT_ENAB);
	AK88_DUMP_REG(LCD_SOFT_CTRL);
	AK88_DUMP_REG(LCD_CLOCK_CONF);

	//lcd_print_reg();

}

void lcd_debug_status(void)
{
	u32 temp;
	temp = __raw_readl(AK88_LCD_STATUS_REG);

	if (temp & STATUS_BUF_EMPTY_ALARM)
		printk("buffer empty\n");
	if (temp & STATUS_ALERT_VALID)
		printk("alert valid\n");
	if (temp & STATUS_TV_REFRESH_START)
		printk("tv refresh start\n");
	if (temp & STATUS_TV_REFRESH_OK)
		printk("tv refresh ok\n");
	if (temp & STATUS_RGB_REFRESH_START)
		printk("rgb refresh start\n");
	if (temp & STATUS_RGB_REFRESH_OK)
		printk("rgb refresh ok\n");
	if (temp & STATUS_MPU_REFRESH_START)
		printk("mpu refresh start\n");
	if (temp & STATUS_MPU_REFRESH_OK)
		printk("mpu refresh ok\n");

	if (temp & STATUS_SYS_ERROR)
		printk("err\n");

}

void lcd_debug_refreshing_loop(void)
{
	u32 temp;
	u32 cnt = 0;

	printk("lcd refreshing loop ...\n");

	while (1) {

		temp = __raw_readl(AK88_LCD_STATUS_REG);

		if (temp & STATUS_BUF_EMPTY_ALARM)
			printk("p");
		if (temp & STATUS_ALERT_VALID)
			printk("a");

		if (temp & STATUS_TV_REFRESH_START)
			if (temp & STATUS_TV_REFRESH_OK) {
				cnt = 0;
				printk("t");
			}

		if (temp & STATUS_RGB_REFRESH_START)
			if (temp & STATUS_RGB_REFRESH_OK) {
				cnt = 0;
				printk("r");
			}

		if (temp & STATUS_MPU_REFRESH_START)
			if (temp & STATUS_MPU_REFRESH_OK) {
				cnt = 0;
				printk("m");
			}

		if (temp & STATUS_SYS_ERROR) {
			printk("e");
			lcd_dump_reg();
			while (1) ;
		}

		if (cnt > 1000000) {
			printk("lcd refresh timeout! status =0x%x\n", temp);
			lcd_dump_reg();
			while (1) ;
		}

		cnt++;
	}
}

void lcd_rgb_init(struct tft_lcd_timing_info *p_tft, u32 asic_freq)
{
	printk("sysreg_base=0x%x,lcdreg_base=0x%x\n", (u32) AK88_VA_SYS,
	       (u32) AK88_VA_DISP);

	lcd_rgb_set_pclk(asic_freq /*124 MHZ */ , p_tft->clock_hz /*30 MHZ */ );

	//0x21000000    
	//bit[3]:rgb_ch_en,   0=disable,1=enable
	//bit[2]:YCbCr1_ch_en,0=disable,1=enable
	//bit[1]:YCbCr2_ch_en,0=disable,1=enable
	//bit[0]:OSD_ch_en,   0=disable,1=enable

	//disable all channel

	__raw_writel(0, AK88_LCD_CMD_REG1);
	AKCLR_BITS(1UL << 3 | 1UL << 2 | 1UL << 1 | 1UL << 0,
		   AK88_LCD_CMD_REG1);
	//lcd_update();

	//setlect rgb interface
	//0x21000000    
	//bit[6:5]: if_mode: LCD interface mode selection
	//              00:reserved
	//              01=MPU interface
	//              10=RGB interface
	//              11=TV interface

	AKCLR_BITS(0x3UL << 5, AK88_LCD_CMD_REG1);
	AKSET_BITS(0x2UL << 5, AK88_LCD_CMD_REG1);

	//set timing
	lcd_rgb_set_timing(p_tft);

	lcd_controller_fastdma();

	//set bg color
	//0x2001,003c background color register
	//bit[31:24]: reserved
	//bit[23:0]:  back_color. default background color

	__raw_writel(0x5a5a5a, AK88_LCD_BKG_COLO);

}

//void lcd_fb_rgb_init(struct anyka_lcdfb_info *sinfo, u32 asic_freq)
void lcd_fb_set_timing(struct anyka_lcdfb_info *sinfo, u32 asic_freq)
{

	struct tft_lcd_timing_info tft;
	u8 bit3, bit2, bit1, bit0;
#if 0
	struct tft_lcd_timing_info {

		u8 bus_width;	//24
		u32 interlace;	//PROGRESS=1<<20   //interlace(1) or progress(0)
		u8 hvg_pol;	//0xe              //h_sync(bit2),v_sync(bit1),gate(bit0) polarity
		u16 rgb_bit;	//0x0888           //RGB or GBR(bit12, if 1:BGR),R(bits 11:8),G(bit 7:4),B(bit 3:0) bits

		u32 clock_hz;	//30,000,000(30M)  //bit clock, clock cycle,unit HZ

		u32 h_sync_hz;	//15711??          //h_sync cycle,unit HZ
		u32 v_sync_001hz;	//5885             //v_sync cycle,unit 0.01HZ

		u16 h_tot_clk;	//1058             //horizontal total cycle,unit clock
		u16 h_disp_clk;	//800              //horizontal display cycle,unit clock
		u16 h_front_porch_clk;	//170              //horizontal front porch,unit clock
		u16 h_pulse_width_clk;	//48               //horizontal pulse width,unit clock
		u16 h_back_porch_clk;	//40               //horizontal back porch,unit clock

		u16 v_tot_clk;	//553              //vertical   total cycle,unit h_sync
		u16 v_disp_clk;	//480              //vertical   display cycle,unit h_sync
		u16 v_front_porch_clk;	//40               //vertical   front porch,unit clock
		u16 v_pulse_width_clk;	//4                //vertical   pulse width,unit clock
		u16 v_back_porch_clk;	//29               //vertical   back porch,unit clock

		char *name;
	};
#endif
	tft.bus_width = 16;
	//if(sinfo->mode->vmode == FB_VMODE_NONINTERLACED) in board_ak8802.c
	//tft.interlace = 1<<20 ; //no interlace 

//      .vmode          = FB_VMODE_NONINTERLACED,
	tft.interlace = 1;	//no interlace 
	if (sinfo->default_monspecs->modedb->vmode == FB_VMODE_NONINTERLACED)
		tft.interlace = 1;	//no interlace 
	else
		tft.interlace = 0;	//interlace 
	printk("lcd_fb_set_timing(),tft.interlace(1)=%d\n", tft.interlace);

	//u8   hvg_pol;         //0xe      //1=positive,0=negative;
	//pclk polarity(bit3),h_sync(bit2),v_sync(bit1),gate(bit0) polarity
	//refer to board_ak8802.c

	tft.hvg_pol = 0xe;	//default
	bit0 = 0;		//VOGATE(den) high active
	bit3 = 1;		//pclk high active
	bit2 = 1;
	bit1 = 1;

	//if(sinfo->info->mode->sync & FB_SYNC_HOR_HIGH_ACT)   //oops error
	if (sinfo->default_monspecs->modedb->sync & FB_SYNC_HOR_HIGH_ACT)	//oops error
		bit2 = 0;
	else
		bit2 = 1;

	//if(sinfo->info->mode->sync & FB_SYNC_HOR_HIGH_ACT)
	if (sinfo->default_monspecs->modedb->sync & FB_SYNC_HOR_HIGH_ACT)
		bit1 = 0;
	else
		bit1 = 1;

	tft.hvg_pol = (bit3 << 3) | (bit2 << 2) | (bit1 << 1) | (bit0);

	printk("lcd_fb_set_timing(),tft.hvg_pol(0xe)=0x%x\n", tft.hvg_pol);

	//u16  rgb_bit;  //0x0888   //RGB or GBR(bit12, if 1:BGR),R(bits 11:8),G(bit 7:4),B(bit 3:0) bits
	tft.rgb_bit = 0x0888;
	if (sinfo->lcd_wiring_mode == ANYKA_LCDC_WIRING_BGR) {
		/* BGR mode */
		tft.rgb_bit |= 1UL << 12;
	}
	//tft.clock_hz = PICOS2KHZ(sinfo->info->var.pixclock)*1000;
	tft.clock_hz = sinfo->info->var.pixclock;

	printk("sinfo->pixclock=%d,sinfo->pixclock=%d\n",
	       (u32) sinfo->info->var.pixclock,
	       (u32) sinfo->info->var.pixclock);

	tft.h_sync_hz = 0;	//no use
	tft.v_sync_001hz = 0;	//no use

	tft.h_disp_clk = sinfo->info->var.xres;
	tft.h_back_porch_clk = sinfo->info->var.left_margin;
	tft.h_front_porch_clk = sinfo->info->var.right_margin;
	tft.h_pulse_width_clk = sinfo->info->var.hsync_len;
	tft.h_tot_clk =
	    tft.h_disp_clk + tft.h_front_porch_clk + tft.h_pulse_width_clk +
	    tft.h_back_porch_clk;

//printk("tft.h_disp_clk=%d,tft.h_front_porch_clk=%d,tft.h_pulse_width_clk=%d,tft.h_back_porch_clk=%d,tft.h_tot_clk=%d\n",tft.h_disp_clk,tft.h_front_porch_clk,tft.h_pulse_width_clk,tft.h_back_porch_clk,tft.h_tot_clk);

	tft.v_disp_clk = sinfo->info->var.yres;
	tft.v_back_porch_clk = sinfo->info->var.upper_margin;
	tft.v_front_porch_clk = sinfo->info->var.lower_margin;
	tft.v_pulse_width_clk = sinfo->info->var.vsync_len;
	tft.v_tot_clk =
	    tft.v_disp_clk + tft.v_front_porch_clk + tft.v_pulse_width_clk +
	    tft.v_back_porch_clk;

	printk("tft.h_tot_clk=%d,tft.v_tot_clk=%d\n", tft.h_tot_clk,
	       tft.v_tot_clk);

	//lcd_rgb_init(&tft,asic_freq );

	//lcd_rgb_set_pclk(asic_freq/*124 MHZ*/,tft.clock_hz/*30 MHZ*/);

	lcd_rgb_set_timing(&tft);

	//if_mode = LCD_IF_RGB;
	//lcd_rgb_set_interface(if_mode);

	//lcd_rgb_start_refresh();

}

//from ak780x_LCD_RGBController.cpp

void rgbcontroller_set_interface(LCD_IF_MODE if_mode,
				 struct tft_lcd_timing_info *p_tft_info)
{
	//u16 *pd;
	//u32 i;

	if (if_mode == LCD_IF_RGB) {

		//lcd_rgb_init(&hsd070idw1_a10,ak880x_asicfreq_get());

		lcd_rgb_init(p_tft_info, ak880x_asicfreq_get());	//debug
		mdelay(100);

#if 1				//debug

		//set share pin
		//set sharepin
//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
//bit[25]: 1=corresponding are used as {LCD_DATA[15:9]}  //gpio[68:62]
//bit[26]: 1=corresponding are used as {LCD_DATA[8]}     //gpio[61]
//bit[27]: 1=corresponding are used as {LCD_DATA[17:16]} //gpio[70:69] 
//bit[28]: 1=corresponding are used as {MPU_RST]}        //gpio[71] 

		AKSET_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[15:9]
		AKSET_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[8]
		AKSET_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[17:16]
		AKSET_BITS(1UL << 28, AK88_SHAREPIN_CTRL);	//set pin as #MPU_RST
		mdelay(100);

		//reset lcm
		//baselcd_reset_panel();

		//lcd_select_channel
		//0x21000000
		//bit[3]:rgb_ch_en,   0=disable,1=enable
		//bit[2]:YCbCr1_ch_en,0=disable,1=enable
		//bit[1]:YCbCr2_ch_en,0=disable,1=enable
		//bit[0]:OSD_ch_en,   0=disable,1=enable

		//disable all channel
		AKCLR_BITS(1UL << 3 | 1UL << 2 | 1UL << 1 | 1UL << 0,
			   AK88_LCD_CMD_REG1);
		//lcd_update();
		mdelay(100);

		//select the rgb channel
		AKSET_BITS(1UL << 3, AK88_LCD_CMD_REG1);
		//lcd_update();
		mdelay(100);

		//setlect rgb interface
		//0x21000000    
		//bit[6:5]: if_mode: LCD interface mode selection
		//              00:reserved
		//              01=MPU interface
		//              10=RGB interface
		//              11=TV interface

		AKCLR_BITS(0x3UL << 5, AK88_LCD_CMD_REG1);
		AKSET_BITS(0x2UL << 5, AK88_LCD_CMD_REG1);

		mdelay(100);
		//lcd_rgb_start_refresh();

		lcd_dump_reg();

		//panel power on
		bsplcd_set_panel_power(1);	//pullup TFT_VGH_L and TFT_AVDD

		//before enable back light,at least wait one frame output
		//sleep(10);
		baselcd_set_panel_backlight(1);
#endif

	}

}

//void lcd_rgb_set_interface(LCD_IF_MODE if_mode,struct tft_lcd_timing_info *p_tft_info)
void lcd_rgb_set_interface(LCD_IF_MODE if_mode)
{
	//u16 *pd;
	//u32 i;

	if (if_mode == LCD_IF_RGB) {

		AKSET_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[15:9]
		AKSET_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[8]
		AKSET_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[17:16]
		AKSET_BITS(1UL << 28, AK88_SHAREPIN_CTRL);	//set pin as MPU_RST
		//mdelay(100);

		//lcd_select_channel
		//0x21000000
		//bit[3]:rgb_ch_en,   0=disable,1=enable
		//bit[2]:YCbCr1_ch_en,0=disable,1=enable
		//bit[1]:YCbCr2_ch_en,0=disable,1=enable
		//bit[0]:OSD_ch_en,   0=disable,1=enable

		//disable all channel
		AKCLR_BITS(1UL << 3 | 1UL << 2 | 1UL << 1 | 1UL << 0,
			   AK88_LCD_CMD_REG1);
		//mdelay(100);
		//lcd_update();

		//select the rgb channel
		AKSET_BITS(1UL << 3, AK88_LCD_CMD_REG1);
		//mdelay(100);
		//lcd_update();

		//setlect rgb interface
		//0x21000000    
		//bit[6:5]: if_mode: LCD interface mode selection
		//              00:reserved
		//              01=MPU interface
		//              10=RGB interface
		//              11=TV interface

		AKCLR_BITS(0x3UL << 5, AK88_LCD_CMD_REG1);

		AKSET_BITS(0x2UL << 5, AK88_LCD_CMD_REG1);

		lcd_controller_fastdma();

		mdelay(100);

	}

}

void rgbcontroller_copy_usrdata_to_rgb(u32 h_offset, u32 v_offset, u32 h_len,
				       u32 v_len, u8 * buf)
{
	u8 *pd, *ps;
	u32 dy;

	ps = buf;
	pd = display_ram.p_rgb_vrt_base1 + (v_offset * lcd_size.w_pixel +
					    h_offset) * 2;

	//copy one line by one line
	for (dy = 0; dy < v_len; dy++) {
		memcpy(pd, ps, h_len * 2);	//copy one line
		pd += lcd_size.w_pixel * 2;
		ps += h_len * 2;
	}

}

void rgbcontroller_start_dma(void)
{
	rgbcontroller_copy_usrdata_to_rgb(usr_rgb_area.h_offset,
					  usr_rgb_area.v_offset,
					  usr_rgb_area.h_len,
					  usr_rgb_area.v_len, usr_rgb_area.buf);
}

void display_init(void)
{
	//attach_controller()
	//new ANYKA_LCD_RGB_CONTROLLER()::LCD_CONTROLLER()
	//
	struct tft_lcd_timing_info tft_info;
	u32 asic_freq, smem_len, begin, addr;
	int i, j, split;

	asic_freq = ak880x_asicfreq_get();
	tft_info = hsd070idw1_a10;
	//tft_info=q07021_701;

	lcd_size.w_pixel = tft_info.h_disp_clk;	//800
	lcd_size.h_pixel = tft_info.v_disp_clk;	//480
	lcd_size.pixels = lcd_size.w_pixel * lcd_size.h_pixel;
	lcd_size.byte_per_pixel = 2;
	lcd_size.ram_size = lcd_size.pixels * lcd_size.byte_per_pixel;	//=800*480*2
	lcd_size.freq = tft_info.v_sync_001hz * 100;	//?? 5885*100 

	if_mode = LCD_IF_RGB;

	//baselcd_reset_controller();  //to rest power clock reg bit19
	//lcd_rgb_set_interface(if_mode);

	memset(&display_ram, 0, sizeof(struct display_ram));

	display_ram.rgb_len1 = lcd_size.ram_size;

	display_ram.yuv1_len = 640 * 480 * 3 / 2;

	display_ram.osd_len = (32 * 32) / 2;	//defined in cursor.h

	disp_buf_size =
	    display_ram.rgb_len1 + display_ram.rgb_len2 + display_ram.yuv1_len +
	    display_ram.osd_len * 2;

	//display_ram.p_rgb_base1 = (u8*) AK88_PA_DISPRAM_RGB ;
	//display_ram.p_rgb_vrt_base1 = (u8*) AK88_VA_DISPRAM_RGB ;

	smem_len = lcd_size.w_pixel * lcd_size.h_pixel * 2;

	display_ram.p_rgb_vrt_base1 = dma_alloc_writecombine(NULL, smem_len,
							     (dma_addr_t *) &
							     (display_ram.
							      p_rgb_base1),
							     GFP_KERNEL);

	//display_ram.p_rgb_vrt_base1 = dma_alloc_writecombine(NULL, smem_len,
	//                              (dma_addr_t *)&dma_addr, GFP_KERNEL);

	//display_ram.p_rgb_base1 =dma_addr;

	display_ram.p_yuv1_base =
	    display_ram.p_rgb_base1 + display_ram.rgb_len1;
	display_ram.p_yuv1_vrt_base =
	    display_ram.p_rgb_vrt_base1 + display_ram.rgb_len1;

	display_ram.p_osd_base =
	    display_ram.p_rgb_base1 + display_ram.rgb_len1 +
	    display_ram.yuv1_len;
	display_ram.p_osd_vrt_base =
	    display_ram.p_rgb_vrt_base1 + display_ram.rgb_len1 +
	    display_ram.yuv1_len;

	//lcd_init_display_ram(&display_ram,&lcd_size);

	set_ahb_priority();

	baselcd_controller_init(1);

	lcd_rgb_set_pclk(asic_freq /*124 MHZ */ ,
			 tft_info.clock_hz /*30 MHZ */ );

	lcd_rgb_set_timing(&tft_info);

	lcd_init_display_ram(&display_ram, &lcd_size);

	lcd_rgb_set_interface(if_mode);

	//baselcd_reset_panel();

	lcd_rgb_start_refresh();

	baselcd_set_panel_backlight(1);

	printk("display_init()\n");

	//mdelay(1000);
	//for(i=0;i<2;i++)
	//      lcd_dump_reg();

	memset(display_ram.p_rgb_vrt_base1, 0x00, (lcd_size.ram_size >> 1));
	//mdelay(1000); 
	memset(display_ram.p_rgb_vrt_base1 + (lcd_size.ram_size >> 1), 0xff,
	       (lcd_size.ram_size >> 1));

	addr = (u32) display_ram.p_rgb_vrt_base1;

	split = 8;
	for (j = 0; j < split; j++) {
		for (i = 0; i < smem_len / split; i++) {
			begin = j * (smem_len / split);
			if (j % 2 == 0)
				*((char *)(begin + addr + i)) = 0x00;
			else
				*((char *)(begin + addr + i)) = 0xff;
		}
	}

}

//SYMBOL_EXPORT(display_init);

//#define DEBUG_LCD_1
#undef DEBUG_LCD_1
#ifdef DEBUG_LCD_1

#define REG32(_reg_)  (*(volatile unsigned long *)(_reg_))
#define IPL_LCD_WIDTH 800
#define IPL_LCD_HEIGHT 480
#define IPL_LCD_INTERFACE_TYPE_RGB 1
#define IPL_LCD_INTERFACE_TYPE_MPU 0
typedef long DWORD;
typedef char BYTE;
typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned long UINT32;

volatile char *pSysCtrlReg = AK88_VA_SYS;
volatile char *pLcmCtrlReg = AK88_VA_DISP;

#include "AK_lcd.h"

const T_LCD_TIMING_INFO SUPPORT_LCD_TABLE_IPL[] = {
	//Bus,Interlace, HVG_Pol,RGB_BIT, Clock,      HCycle,VCycle, Thlen,  Thf,Thp,Thb,Tvlen,      Tvf,Tvp,Tvb,pName
//    {18,       AK_FALSE,  0x6,    0,      9000000,   366,       320,     22,      4,       20,       492,      480,    4,        4,      4,       "LS035B7UX01"},    
//       {24,       AK_FALSE,  0x6,    0,      6410000,    280,       240,     10,      20,      10,       326,      320,    2,        2,      2,       "PT035TN01"},

	{18, PROGRESS, 0x6, 0x1888, 9000000, 19560, 5994, 366, 320, 22, 4, 20,
	 492, 480, 4, 4, 4, "LS035B7UX01"},
	{18, PROGRESS, 0x6, 0x1888, 9000000, 19560, 5994, 254, 240, 2, 10, 2,
	 365, 400, 3, 39, 3, "R61509"},
	{18, PROGRESS, 0x6, 0x1888, 9000000, 19560, 5994, 254, 240, 2, 10, 2,
	 365, 320, 3, 39, 3, "HX8347"},
	{24, PROGRESS, 0x6, 0x1888, 9000000, 17140, 5994, 525, 480, 2, 41, 2,
	 286, 272, 2, 10, 2, "AT043TN134"},
	{24, PROGRESS, 0x6, 0x1888, 6410256, 15711, 5884, 408, 320, 20, 30, 38,
	 267, 240, 1, 3, 23, "PT035TN01"},
	{24, PROGRESS, 0xe, 0x0888, 30000000, 15711, 5884, 1058, 800, 170, 48,
	 40, 553, 480, 40, 4, 29, "HSD070IDW1-A00"},

	//{18,        PROGRESS, 0xe, 0x1888, 9000000, 17140, 5994, 525, 480, 2, 41, 2, 286, 272, 2, 10, 2, "LQ043T3DX04"},
	//{24,        PROGRESS, 0x6,    0x1888,  10000000, 19560, 5994, 280, 240, 10, 20, 10, 326, 320, 2, 2, 2, "SSD1289"},

};

void Del_1us(void)
{
	volatile unsigned int i, j;
	for (i = 0; i < 6000; i++) {
		for (j = 0; j < 2; j++) ;
	}

}

void DelayX1ms(volatile unsigned int m)
{
	volatile unsigned int i;
	for (i = 0; i < m; i++)
		Del_1us();
}

#define SYS_RST_CLK_CTL_OFFSET (0x000c)	//0x0800,000c
#define SYS_RST_LCD            (1 << 19)
#define CLOCK_DSA_DISPLAY      (1 <<3 )

void IPL_LCDCInitRGB(DWORD DispBuf, DWORD picWidth, DWORD picHeight,
		     DWORD picHStart, DWORD picVStart)
{
	DWORD temp;
	DWORD dwASICFreq;
	int width, height;
	int startX, startY;
	const T_LCD_TIMING_INFO *pLcdInfo;
	//IOCTL_SHAREPIN  pinInfo = {0};

#define IPL_DEBUG 0

	if (DispBuf == (DWORD) NULL) {
		//OALMSG( IPL_DEBUG, (TEXT("IplInit_Display_error\r\n")) );
		printk("IplInit_Display_error\r\n");
		return;
	}
	//pLcdInfo = &SUPPORT_LCD_TABLE_IPL[ ID_2_INDEX(LCD_ID) ];
	pLcdInfo = &SUPPORT_LCD_TABLE_IPL[5];	//HSD070IDW1-A00
	if (picWidth > pLcdInfo->Thd_PClk) {
		width = pLcdInfo->Thd_PClk;
	} else {
		width = picWidth;
	}

	if (picHeight > pLcdInfo->Tvd_HCLK) {
		height = pLcdInfo->Tvd_HCLK;
	} else {
		height = picHeight;
	}

	startY = (pLcdInfo->Tvd_HCLK - height) >> 1;
	startX = (pLcdInfo->Thd_PClk - width) >> 1;
	//OALMSG(IPL_DEBUG, (TEXT("width %d, height %d, startX %d, startY %d\r\n"), width, height, startX, startY));
	printk("width %d, height %d, startX %d, startY %d\r\n", width, height,
	       startX, startY);

	// get ASIC frequency
	//dwASICFreq = OALHalGetASICFreq();
	dwASICFreq = ak880x_asicfreq_get();
	//EdbgOutputDebugString("ASIC freq: %d sizeof = %d\n", dwASICFreq,IPL_LCD_HEIGHT*IPL_LCD_WIDTH*2);
	printk("ASIC freq: %d sizeof = %d\n", (int)dwASICFreq,
	       IPL_LCD_HEIGHT * IPL_LCD_WIDTH * 2);

	// --- set and en pclk divider
	//temp = dwASICFreq / pLcdInfo->ClockCycle_Hz;
	temp = 124 / 30;
	temp -= 1;
	REG32(pLcmCtrlReg + LCD_CLOCK_CONFIG_OFFSET) =
	    CLOCK_CONFIG_PCLK_EN | CLOCK_CONFIG_PCLK_CFG_VALID |
	    ((temp << CLOCK_CONFIG_PCLK_DIV_OFFSET) &
	     CLOCK_CONFIG_PCLK_DIV_MASK);

	// --- set sharePin(MPU_RST and LCD_DATA[17:8]
	//pinInfo.pinGrp = eSHARE_GPIO_DISPLAY;
	//KernelIoControl(IOCTL_HAL_LOCKSET_GPIO_SHAREPIN, &pinInfo, sizeof(IOCTL_SHAREPIN), NULL, 0, NULL);

	//#define AK88_SHAREPIN_CTRL    (ANYKA_PA_SYS+0x0078)   //0x08000078
	AKSET_BITS(1UL << 25, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[15:9]
	AKSET_BITS(1UL << 26, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[8]
	AKSET_BITS(1UL << 27, AK88_SHAREPIN_CTRL);	//set pin as LCD_DATA[17:16]
	AKSET_BITS(1UL << 28, AK88_SHAREPIN_CTRL);	//set pin as #MPU_RST

	// --- set RGB mode
	temp = REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET));
	temp &= ~CMD1_IF_MODE_MASK;
	temp |= CMD1_IF_RGB;
	REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET)) = temp;

	//--- set timing

	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL1_REG_OFFSET)) =
	    RGB_CTRL1_WIDTH_18 | (pLcdInfo->Interlace) | (pLcdInfo->
							  HVG_POL &
							  RGB_CTRL1_HVG_POL_MASK);
	//thp and tvp
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL3_REG_OFFSET)) =
	    (pLcdInfo->Thp_PClk << RGB_CTL3_THPW_PCLK_OFFSET) | (pLcdInfo->
								 Tvp_PClk <<
								 RGB_CTL3_TVPW_OFFSET);
	//thb and thd
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL4_REG_OFFSET)) =
	    (pLcdInfo->Thb_PClk << RGB_CTL4_THB_PCLK_OFFSET) | (pLcdInfo->
								Thd_PClk <<
								RGB_CTL4_THD_PCLK_OFFSET);

	//thf and thlen
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL5_REG_OFFSET)) =
	    (pLcdInfo->Thf_PClk << RGB_CTL5_THF_PCLK_OFFSET) | (pLcdInfo->
								Thlen_PClk <<
								RGB_CTL5_THLEN_PCLK_OFFSET);
	//tvb
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL6_REG_OFFSET)) =
	    (pLcdInfo->Tvb_PClk << RGB_CTL6_TVB_OFFSET);
	//tvf
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL7_REG_OFFSET)) =
	    (pLcdInfo->Tvf_PClk << RGB_CTL7_TVF_OFFSET);
	//tvd
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL8_REG_OFFSET)) =
	    (pLcdInfo->Tvd_HCLK << RGB_CTL8_TVD_OFFSET);
	//tvlen
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL9_REG_OFFSET)) =
	    (pLcdInfo->Tvlen_HCLK << RGB_CTL9_TVLEN_OFFSET);

	//set rgb or bgr and alr
	temp = REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET));
	temp &= ~(CMD1_ALR_EMPTY_MASK | CMD1_ALR_FULL_MASK);
	//Pclk polarity
	if (pLcdInfo->HVG_POL & 0x8)
		temp |= 1 << 4;
	else
		temp &= ~(1 << 4);
	temp |=
	    ((EMPTY_ALARM_BYTES << CMD1_ALR_EMPTY_OFFSET) |
	     (FULL_ALARM_BYTES << CMD1_ALR_FULL_OFFSET));
	if ((pLcdInfo->RGB_BIT & (0x1 << 12)) == 0)
		temp |= CMD1_SEQ_BGR;
	else
		temp &= ~CMD1_SEQ_BGR;

	REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET)) = temp;

	//set display area size(NOTE: width is at hight bits)
	REG32((DWORD *) (pLcmCtrlReg + LCD_GINFO_REG_OFFSET)) =
	    (pLcdInfo->Tvd_HCLK << GINFO_VER_OFFSET) | (pLcdInfo->
							Thd_PClk <<
							GINFO_HOR_OFFSET);

	// --- set starting buffer address of RGB channel
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL2_REG_OFFSET)) &=
	    ~RGB_CTRL1_CFBA_MASK;
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_CTRL2_REG_OFFSET)) =
	    (DispBuf) & RGB_CTRL1_CFBA_MASK;

	// --- RGB virtual page
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_VIRT_LEN_REG_OFFSET)) =
	    (pLcdInfo->Thd_PClk << RGB_VIRT_LEN_H_PXL_OFFSET) | (pLcdInfo->
								 Tvd_HCLK <<
								 RGB_VIRT_LEN_V_PXL_OFFSET);

	// --- RGB channel offset
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_OFFSET_REG_OFFSET)) = 0;

	// --- RGB channel width/height(NOTE: width is at hight bits)
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_SIZE_REG_OFFSET)) =
	    (pLcdInfo->Thd_PClk << RGB_SIZE_H_OFFSET) | (pLcdInfo->Tvd_HCLK);

	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_VIRT_OFFSET_REG_OFFSET)) = 0;
	REG32((DWORD *) (pLcmCtrlReg + LCD_RGB_OFFSET_REG_OFFSET)) = 0;

	// --- enable RGB channel
	temp = REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET));
	temp |= CMD1_RGB_CH_EN;
	REG32((DWORD *) (pLcmCtrlReg + LCD_CMD1_REG_OFFSET)) = temp;

	//reset
	REG32((DWORD *) (pLcmCtrlReg + LCD_RESET_SIGNAL_REG_OFFSET)) =
	    RESET_SIGNAL;
	DelayX1ms(10);
	REG32((DWORD *) (pLcmCtrlReg + LCD_RESET_SIGNAL_REG_OFFSET)) = 0;
	DelayX1ms(10);
	REG32((DWORD *) (pLcmCtrlReg + LCD_RESET_SIGNAL_REG_OFFSET)) =
	    RESET_SIGNAL;
	DelayX1ms(20);

	//Set_LCDCmd((volatile BYTE *)pSysCtrlReg, 0);   //change
	bsplcd_set_panel_power(0);	//DGPIO26,DGPIO25
	baselcd_set_panel_backlight(0);	//GPIO9
	DelayX1ms(200);
	//Set_LCDCmd((volatile BYTE *)pSysCtrlReg, 1);
	bsplcd_set_panel_power(1);
	baselcd_set_panel_backlight(1);

	DelayX1ms(200);

	// --- update software configuratioin
	REG32((DWORD *) (pLcmCtrlReg + LCD_SOFTWARE_CTRL_REG_OFFSET)) |=
	    SOFTWARE_CTRL_SW_EN;

	// --- start refresh
	REG32((DWORD *) (pLcmCtrlReg + LCD_OPERATE_REG_OFFSET)) &=
	    ~OPERATE_SYS_STOP;
	REG32((DWORD *) (pLcmCtrlReg + LCD_OPERATE_REG_OFFSET)) |=
	    OPERATE_RGB_GO;
}

void IPL_LCDCInit(DWORD DispBuf, int picWidth, int picHeight, int picHStart,
		  int picVStart)
{
	// --- reset lcd interface
	REG32((DWORD *) (pSysCtrlReg + SYS_RST_CLK_CTL_OFFSET)) |= SYS_RST_LCD;
	REG32((DWORD *) (pSysCtrlReg + SYS_RST_CLK_CTL_OFFSET)) &= ~SYS_RST_LCD;
	// --- open LCD controller clock
	REG32(pSysCtrlReg + SYS_RST_CLK_CTL_OFFSET) &= ~CLOCK_DSA_DISPLAY;

	// REG32(AK88_AHB_PRIORITY) &= ~0x7f;    //set dma priority higher than ARM priority

#if IPL_LCD_INTERFACE_TYPE_MPU
	IPL_LCDCInitMPU(DispBuf, picWidth, picHeight, picHStart, picVStart);
#elif IPL_LCD_INTERFACE_TYPE_RGB
	IPL_LCDCInitRGB(DispBuf, picWidth, picHeight, picHStart, picVStart);
#endif

}

bool OEMIPLInit(void)
{

	DWORD DisplayPhysicalBuffer = AK88_PA_DISPRAM_RGB;
	DWORD DisplayVirtualBuffer = AK88_VA_DISPRAM_RGB;

	unsigned int i, j, split, begin;
	DWORD addr, vir_addr, smem_len;

	//REG32(AK88_AHB_PRIORITY) &=~0x7f;    //set dma priority higher than ARM priority
	set_ahb_priority();

	smem_len = IPL_LCD_WIDTH * IPL_LCD_HEIGHT * 2;

	DisplayVirtualBuffer = dma_alloc_writecombine(NULL, smem_len,
						      (dma_addr_t *) &
						      DisplayPhysicalBuffer,
						      GFP_KERNEL);

	printk
	    ("DisplayPhysicalBuffer=0x%x,DisplayVirtualBuffer=0x%x,smem_len=%d\n",
	     (u32) DisplayPhysicalBuffer, (u32) DisplayVirtualBuffer,
	     (int)smem_len);

	printk("Bat is larger than 3.45V!\r\n");

	//REG32(0xf020d014) &=~0x7f;    //AHB priority

	IPL_LCDCInit(DisplayPhysicalBuffer, IPL_LCD_WIDTH, IPL_LCD_HEIGHT, 0,
		     0);
	//LocalBacklightInit();

	baselcd_set_panel_backlight(1);

	//addr=DisplayPhysicalBuffer;
	addr = DisplayVirtualBuffer;

	memset((void *)(addr), 0x00, (smem_len >> 1));
	memset((void *)(addr + (smem_len >> 1)), 0xff, (smem_len >> 1));

	split = 8;
	for (j = 0; j < split; j++) {
		for (i = 0; i < smem_len / split; i++) {
			begin = j * (smem_len / split);
			if (j % 2 == 0)
				*((char *)(begin + addr + i)) = 0x00;
			else
				*((char *)(begin + addr + i)) = 0xff;
		}
	}

#if 0
	for (i = 0; i < smem_len / 4; i++)
		*((char *)(addr + i)) = 0x00;
	for (i = 0; i < smem_len / 4; i++)
		*((char *)(addr + (smem_len / 4) + i)) = 0xff;
	for (i = 0; i < smem_len / 4; i++)
		*((char *)(addr + (smem_len / 2) + i)) = 0x00;
	for (i = 0; i < smem_len / 4; i++)
		*((char *)(addr + (smem_len / 2) + (smem_len / 4) + i)) = 0xff;
#endif

	DelayX1ms(1000);

	lcd_dump_reg();
	mdelay(100);
	lcd_dump_reg();

	printk("----OEMIPLInit\r\n");
	return true;

}

#endif
