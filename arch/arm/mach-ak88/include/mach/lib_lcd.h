#ifndef __ARCH_ARM_MACH_AK88_LIB_LCD_H
#define __ARCH_ARM_MACH_AK88_LIB_LCD_H

#include <mach/map.h>
#include <linux/fb.h>
#include <video/anyka_lcdc.h>

struct tft_lcd_timing_info {

	u8 bus_width;		//24
	u32 interlace;		//PROGRESS=1<<20   //interlace(1) or progress(0)
	u8 hvg_pol;		//0xe              //h_sync(bit2),v_sync(bit1),gate(bit0) polarity
	u16 rgb_bit;		//0x0888           //RGB or GBR(bit12, if 1:BGR),R(bits 11:8),G(bit 7:4),B(bit 3:0) bits

	u32 clock_hz;		//30,000,000(30M)  //bit clock, clock cycle,unit HZ

	u32 h_sync_hz;		//15711??          //h_sync cycle,unit HZ
	u32 v_sync_001hz;	//5885             //v_sync cycle,unit 0.01HZ

	u16 h_tot_clk;		//1058             //horizontal total cycle,unit clock
	u16 h_disp_clk;		//800              //horizontal display cycle,unit clock
	u16 h_front_porch_clk;	//170              //horizontal front porch,unit clock
	u16 h_pulse_width_clk;	//48               //horizontal pulse width,unit clock
	u16 h_back_porch_clk;	//40               //horizontal back porch,unit clock

	u16 v_tot_clk;		//553              //vertical   total cycle,unit h_sync
	u16 v_disp_clk;		//480              //vertical   display cycle,unit h_sync
	u16 v_front_porch_clk;	//40               //vertical   front porch,unit clock
	u16 v_pulse_width_clk;	//4                //vertical   pulse width,unit clock
	u16 v_back_porch_clk;	//29               //vertical   back porch,unit clock

	char *name;
};

//lcd panel info
struct lcd_size {

	u16 w_pixel;
	u16 h_pixel;
	u32 pixels;
	u8 byte_per_pixel;
	u32 ram_size;
	u32 freq;
};

//display ram info
struct display_ram {

	u8 *p_rgb_base1;	//physical frame base
	u8 *p_rgb_vrt_base1;
	u32 rgb_len1;
	u8 *p_rgb_base2;
	u8 *p_rgb_vrt_base2;
	u32 rgb_len2;

	u8 *p_yuv1_base;
	u8 *p_yuv1_vrt_base;
	u32 yuv1_len;
	u8 *p_yuv2_base;
	u8 *p_yuv2_vrt_base;
	u32 yuv2_len;

	u8 *p_osd_base;
	u8 *p_osd_vrt_base;
	u32 osd_len;
};

//need refresh picture area
struct picture_area {
	u32 h_offset;
	u32 v_offset;
	u32 h_len;
	u32 v_len;
	u8 *buf;
};

typedef enum {
	LCD_IF_MPU = 0,
	LCD_IF_RGB,
	LCD_IF_TVOUT
} LCD_IF_MODE;

#define AK88_LCD_CMD_REG1	      (AK88_VA_DISP + 0x00)
#define AK88_LCD_REST_SIGNAL        (AK88_VA_DISP + 0x08)	//to send a reset signal
#define AK88_LCD_RGB_CTRL1          (AK88_VA_DISP + 0x10)	//signal of RGB interface conf
#define AK88_LCD_RGB_CTRL2          (AK88_VA_DISP + 0x14)	//buffer address setting and to enable virtual page func of data from RGB channel
#define AK88_LCD_RGB_VIRPAGE_SIZE   (AK88_VA_DISP + 0x18)	//virtual page size of the data input from RGB channel
#define AK88_LCD_RGB_VIRPAGE_OFFSET (AK88_VA_DISP + 0x1C)	//virtual page offset reg
#define AK88_LCD_OSD_ADDR           (AK88_VA_DISP + 0x20)	//osd address
#define AK88_LCD_BKG_COLO           (AK88_VA_DISP + 0x3C)	//background color
#define AK88_LCD_RGB_CTRL3          (AK88_VA_DISP + 0x40)	//Horizontal/Vertical sync pulse width
#define AK88_LCD_RGB_CTRL4          (AK88_VA_DISP + 0x44)	//Horizontal back porch width and display area width
#define AK88_LCD_RGB_CTRL5          (AK88_VA_DISP + 0x48)	//Horizontal front porch width
#define AK88_LCD_RGB_CTRL6          (AK88_VA_DISP + 0x4C)	//Vertical back porch width
#define AK88_LCD_RGB_CTRL7          (AK88_VA_DISP + 0x50)	//Vertical front porch width
#define AK88_LCD_RGB_CTRL8          (AK88_VA_DISP + 0x54)	//Vertical display area
#define AK88_LCD_RGB_CTRL9          (AK88_VA_DISP + 0x58)	//Length of VOVSYNC signal
#define AK88_LCD_RGB_OFFSET         (AK88_VA_DISP + 0xA8)	//Offset values of the data input from RGB channel
#define AK88_LCD_RGB_SIZE           (AK88_VA_DISP + 0xAC)	//the size of the data input from RGB channel
#define AK88_LCD_DISP_AREA          (AK88_VA_DISP + 0xB0)	//Display area size
#define AK88_LCD_CMD_REG2           (AK88_VA_DISP + 0xB4)	//LCD command
#define AK88_LCD_OPER_REG           (AK88_VA_DISP + 0xB8)	//to start the reflash func
#define AK88_LCD_STATUS_REG         (AK88_VA_DISP + 0xBC)
#define AK88_LCD_INT_ENAB           (AK88_VA_DISP + 0xC0)	//to enable interrupt
//status_reg bits corespondents to int_enab_reg
//0x2001,00bc bits corespondents to 0x2001,00c0

#define AK88_LCD_SOFT_CTRL          (AK88_VA_DISP + 0xC8)	//software control

#define AK88_LCD_CLOCK_CONF         (AK88_VA_DISP + 0xE8)	//LCD clock configuration

#define STATUS_BUF_EMPTY_ALARM		(0x1UL << 18)
#define STATUS_ALERT_VALID		(0x1UL << 17)
#define STATUS_TV_REFRESH_START		(0x1UL << 10)
#define STATUS_TV_REFRESH_OK		(0x1UL << 9)
#define STATUS_RGB_EVEN_START		(0x1UL << 8)
#define STATUS_RGB_EVEN_DONE		(0x1UL << 7)
#define STATUS_RGB_ODD_START		(0x1UL << 6)
#define STATUS_RGB_ODD_DONE		(0x1UL << 5)
#define STATUS_RGB_REFRESH_START	(0x1UL << 4)
#define STATUS_RGB_REFRESH_OK		(0x1UL << 3)
#define STATUS_MPU_REFRESH_OK		(0x1UL << 2)
#define STATUS_MPU_REFRESH_START	(0x1UL << 1)
#define STATUS_SYS_ERROR		(0x1UL << 0)
#define SYS_RST_CLK_CTL_OFFSET (0x000c)	//0x0800,000c
#define SYS_RST_LCD            (1 << 19)
#define CLOCK_DSA_DISPLAY      (1 <<3 )

void set_ahb_priority(void);

//extern void rgbcontroller_set_interface(LCD_IF_MODE if_mode);
void bsplcd_set_panel_power(int en /*1=open,0=close */ );	//pullup TFT_VGH_L and TFT_AVDD
void baselcd_set_panel_backlight(int en /*en=0:close; en=1:open */ );
void baselcd_reset_controller(void);	//to rest power clock reg bit19
void baselcd_reset_panel(void);	//lcd panel reset

bool lcd_controller_isrefreshok(void);

void lcd_controller_start_clock(void);
void lcd_controller_stop_clock(void);
void lcd_controller_fastdma(void);

void lcd_update(void);
void lcd_dump_reg(void);

void baselcd_controller_init(int en);
void lcd_rgb_set_interface(LCD_IF_MODE if_mode);
bool lcd_fb_init_ram(unsigned long dma_addr, unsigned long xres,
		     unsigned long yres);
void lcd_rgb_set_pclk(unsigned long pll1_clk, unsigned lcd_clk);
void lcd_fb_set_timing(struct anyka_lcdfb_info *sinfo, u32 pll1_freq);

void lcd_rgb_start_refresh(void);
void lcd_rgb_stop_refresh(void);
void lcd_interrupt_mask(unsigned bits_result, bool disable);
void lcd_wait_status(int mask);

void display_init(void);
bool OEMIPLInit(void);

#endif
