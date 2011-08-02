#define DEBUG

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/console.h>
#include <linux/anyka_cpufreq.h>
#include <linux/cpufreq.h>

#include <linux/fb.h>

#include <mach/gpio.h>

#include "ak98fb.h"
#include <linux/akfb.h>

#define MPU_PCLK (11*1000*1000)	/* 11M pclk for mpu panel */

static struct aklcd_rgb_panel bsp_rgb_panel =
#if defined CONFIG_LCD_PANEL_QD043003C0_40
{"QD043003C0-40", PANEL_PROGRESS, RGB24BITS, SEQ_RGB, \
 1, 43-1, 480, 8, 10, 12-10, 272, 4, TV_UNIT_TH, 9*1000*1000,     \
 POL_POSITIVE, POL_NEGATIVE, POL_NEGATIVE, POL_POSITIVE};
#elif defined CONFIG_LCD_PANEL_AT043TN24
/* only support DE mode */
{"AT043TN24", PANEL_PROGRESS, RGB24BITS, SEQ_RGB, \
 45, 0, 480, 0, 8, 0, 272, 0, TV_UNIT_TH, 9*1000*1000,     \
 POL_POSITIVE, POL_NEGATIVE, POL_NEGATIVE, POL_POSITIVE};
#elif defined CONFIG_LCD_PANEL_A050VW01_V5
/* only support DE mode */
{"A050VW01-V5", PANEL_PROGRESS, RGB24BITS, SEQ_RGB, \
 128, 0, 800, 0, 45, 0, 480, 0, TV_UNIT_TH, 30*1000*1000,     \
 POL_POSITIVE, POL_NEGATIVE, POL_NEGATIVE, POL_POSITIVE};
#elif defined CONFIG_LCD_PANEL_LW700AT9009
{"LW700AT9009", PANEL_PROGRESS, RGB24BITS, SEQ_RGB, \
 128, 216-128, 800, 1056-216-800, 2, 35-2, 480, 525-35-480, TV_UNIT_TH, 33260*1000, \
 POL_POSITIVE, POL_NEGATIVE, POL_NEGATIVE, POL_POSITIVE};
#elif defined CONFIG_LCD_PANEL_AT070TN92
{"AT070TN92", PANEL_PROGRESS, RGB24BITS, SEQ_RGB, \
 1, 46-1, 800, 210, 1, 23-1, 480, 22, TV_UNIT_TH, 33300*1000, \
 POL_POSITIVE, POL_NEGATIVE, POL_NEGATIVE, POL_POSITIVE};
#else
#error "Must specified a LCD panel"
#endif


struct reg {
	void* addr;		/* virtual address of register */
	const char *name;
};

static struct reg aklcd_reg_list[] = {
	{AK98_CLKRST_CTRL1,     "clock & reset control register 1"},
	{AK98_SRDPIN_CTRL1,     "shared pin control register 1"},
	{AK98_SRDPIN_CTRL2,     "shared pin control register 2"},
	{AK98_PUPD2,            "pullup/pulldown register 2"},
	{AK98_PUPD3,            "pullup/pulldown register 3"},
	{AK98_PUPD4,            "pullup/pulldown register 4"},

	{AK98_AHB_PRIORITY1,    "AHB Priority register 1"},
	{AK98_AHB_PRIORITY2,    "AHB Priority register 2"},

	{AK98_LCD_CMD1,         "LCD command register 1"},
	{AK98_LCD_RESET,        "LCD reset register"},
	{AK98_RGBIF_CTRL1,      "LCD RGB interface control 1"},
	{AK98_RGBIF_CTRL2,      "LCD RGB interface control 2"},
	{AK98_RGB_VPAGE_SIZE,   "LCD virtual page size"},
	{AK98_RGB_VPAGE_OFFSET, "LCD virtual page offset"},
	{AK98_BG_COLOR,         "LCD background color"},
	{AK98_RGBIF_CTRL3,      "LCD RGB interface control 3"},
	{AK98_RGBIF_CTRL4,      "LCD RGB interface control 4"},
	{AK98_RGBIF_CTRL5,      "LCD RGB interface control 5"},
	{AK98_RGBIF_CTRL6,      "LCD RGB interface control 6"},
	{AK98_RGBIF_CTRL7,      "LCD RGB interface control 7"},
	{AK98_RGBIF_CTRL8,      "LCD RGB interface control 8"},
	{AK98_RGBIF_CTRL9,      "LCD RGB interface control 9"},
	{AK98_RGB_OFFSET,       "LCD RGB image offset"},
	{AK98_RGB_SIZE,         "LCD RGB image size"},
	{AK98_DISP_SIZE,        "LCD display size"},
	{AK98_LCD_CMD2,         "LCD command register 2"},
	{AK98_LCD_OPER,         "LCD operation register"},
	{AK98_LCD_STATUS,       "LCD status register"},
	{AK98_LCD_INT_ENAB,     "LCD interrupt enable register"},
	{AK98_LCD_SOFT_CTRL,    "LCD software control register"},
	{AK98_LCD_CLKCONF,      "LCD clock config register"},

	{AK98_OV1_YADDR,        "LCD overlay 1 y address"},
	{AK98_OV1_UADDR,        "LCD overlay 1 u address"},
	{AK98_OV1_VADDR,        "LCD overlay 1 v address"},
	{AK98_OV1_HORI_CONF,    "LCD overlay 1 horizontal config"},
	{AK98_OV1_VERT_CONF,    "LCD overlay 1 vertical config"},
	{AK98_OV1_SCALER,       "LCD overlay 1 scaler register"},
	{AK98_OV1_DISP_CONF,    "LCD overlay 1 display config"},
	{AK98_OV1_VPAGE_SIZE,   "LCD overlay 1 virtual page size"},
	{AK98_OV1_VPAGE_OFFSET, "LCD overlay 1 virtual page offset"},

	{AK98_OV2_YADDR,        "LCD overlay 2 y address"},
	{AK98_OV2_UADDR,        "LCD overlay 2 u address"},
	{AK98_OV2_VADDR,        "LCD overlay 2 v address"},
	{AK98_OV2_HORI_CONF,    "LCD overlay 2 horizontal config"},
	{AK98_OV2_VERT_CONF,    "LCD overlay 2 vertical config"},
	{AK98_OV2_SCALER,       "LCD overlay 2 scaler register"},
	{AK98_OV2_DISP_CONF,    "LCD overlay 2 display config"},
	
	{TVOUT_CHROMA_FREQ_REG,   "TVOUT_CHROMA_FREQ_REG"},
	{TVOUT_CTRL_REG1,         "TVOUT_CTRL_REG1"},
	{TVOUT_PARA_CONFIG_REG1,  "TVOUT_PARA_CONFIG_REG1"},
	{TVOUT_PARA_CONFIG_REG2,  "TVOUT_PARA_CONFIG_REG2"},
	{TVOUT_PARA_CONFIG_REG3,  "TVOUT_PARA_CONFIG_REG3"},
	{TVOUT_PARA_CONFIG_REG4,  "TVOUT_PARA_CONFIG_REG4"},
	{TVOUT_PARA_CONFIG_REG5,  "TVOUT_PARA_CONFIG_REG5"},
	{TVOUT_PARA_CONFIG_REG6,  "TVOUT_PARA_CONFIG_REG6"},
	{TVOUT_CTRL_REG2,         "TVOUT_CTRL_REG2"},
};

static void ak_dump_regs(struct reg *reg_list, int n, const struct device *dev)
{
	int i;

	dev_dbg(dev, "dump registers\n");
	for (i = 0; i < n; i++) {
		dev_dbg(dev, "0x%p - 0x%08x - %s\n", reg_list[i].addr,
			__raw_readl(reg_list[i].addr), reg_list[i].name);
	}
}

/********************
 * hardware setting
 ********************/
static inline void aklcd_reset(void)
{
	lcd_set_reg(1, AK98_CLKRST_CTRL1, 27, 27);
	mdelay(1);
	lcd_set_reg(0, AK98_CLKRST_CTRL1, 27, 27);
}

static inline void aklcd_enable(void)
{
	lcd_set_reg(0, AK98_CLKRST_CTRL1, 11, 11);
	mdelay(1);
	aklcd_reset();
}

static inline void aklcd_disable(void)
{
	lcd_set_reg(1, AK98_CLKRST_CTRL1, 11, 11);
}

static void aklcd_set_power(bool enable)
{
	#if 1
	/* power control */
	ak98_gpio_pullup(AK98_GPIO_102, AK98_PULLUP_ENABLE);
	ak98_gpio_cfgpin(AK98_GPIO_102, AK98_GPIO_DIR_OUTPUT);
	ak98_gpio_pullup(AK98_GPIO_103, AK98_PULLUP_ENABLE);
	ak98_gpio_cfgpin(AK98_GPIO_103, AK98_GPIO_DIR_OUTPUT);
	if (enable) {
			ak98_gpio_setpin(AK98_GPIO_103, 0);
			ak98_gpio_setpin(AK98_GPIO_102, 0);
			mdelay(2);
		ak98_gpio_setpin(AK98_GPIO_102, 1);
			mdelay(40);
		ak98_gpio_setpin(AK98_GPIO_103, 1);
		/* reset panel
		lcd_set_reg(0, AK98_LCD_RESET, 1, 1);
		mdelay(1);
		lcd_set_reg(1, AK98_LCD_RESET, 1, 1);*/
	} else {
			ak98_gpio_setpin(AK98_GPIO_103, 0);
		ak98_gpio_setpin(AK98_GPIO_102, 0);
	}
	#endif
}

static void aklcd_start_tvout_clk(void)
{
	/* enable DAC */
	lcd_set_reg(0, AK98_MULTIFUNC_CTRL_REG1, 21, 21);
	/* enable 27MCLK */
	lcd_set_reg(1, AK98_MULTIFUNC_CTRL_REG1, 28, 28);
}

static void aklcd_stop_tvout_clk(void)
{
	/* disable DAC */
	lcd_set_reg(1, AK98_MULTIFUNC_CTRL_REG1, 21, 21);
	/* disable 27MHz clk */
	lcd_set_reg(0, AK98_MULTIFUNC_CTRL_REG1, 28, 28);
}

static void aklcd_start_pclk(int typical_pclk,
			     const struct device *dev)
{
	unsigned int pll_clk;
	unsigned int main_clk;
	unsigned int pclk;	/* panel clock */
	int pclk_div;

	/* Panel clock (pclk):
	   pclk = main_clk / ((pclk_div+1)*2);   (main_clk is the CLK168M in  datasheet)
	*/
	pll_clk = (lcd_get_reg(AK98_CLK_DIV1, 5, 0) * 4 + 180) * 1000 * 1000;
	dev_dbg(dev, "pll_clk = %i\n", pll_clk);
	main_clk = pll_clk / (lcd_get_reg(AK98_CLK_DIV1, 20, 17) + 1);
	dev_dbg(dev, "main_clk = %i\n", main_clk);

	dev_dbg(dev, "typical_pclk = %i\n", typical_pclk);
	/* find the first pclk which closest match the typical panel clock */
	for (pclk_div = ((1<<7)-1); pclk_div >= 0; pclk_div--) {
		pclk = main_clk / ((pclk_div+1)*2);
		if (pclk >= typical_pclk) break;
	}
	/* if the pclk override the typical panel clock 10%, reduce pclk */
	if ((pclk - typical_pclk) > typical_pclk/10)
		pclk_div++;

	dev_dbg(dev, "pclk = %i (pclk_div = %i)\n", main_clk / ((pclk_div+1)*2), pclk_div);

#if 1
	__raw_writel((1 << 8 | (pclk_div << 1) | 1), AK98_LCD_CLKCONF);
#else
	lcd_set_reg(pclk_div, AK98_LCD_CLKCONF, 7, 1);
	lcd_set_reg(1, AK98_LCD_CLKCONF, 8, 8);
	lcd_set_reg(1, AK98_LCD_CLKCONF, 0, 0);
	dev_dbg(dev, "AK98_LCD_CLKCONF: 0x%08x\n", __raw_readl(AK98_LCD_CLKCONF));
#endif
}

static void aklcd_stop_pclk(void)
{
	__raw_writel(1, AK98_LCD_CLKCONF);
}

static void aklcd_start_refresh(enum aklcd_if if_type,
				int pclk_freq,
				const struct device *dev)
{
	/* start refresh */
	switch(if_type) {
	case DISP_IF_MPU:
		aklcd_start_pclk (MPU_PCLK, dev);
		lcd_set_reg(1, AK98_LCD_OPER, 3, 3);
		break;
	case DISP_IF_RGB:
		aklcd_start_pclk (pclk_freq, dev);
		lcd_set_reg(1, AK98_LCD_OPER, 2, 2);
		break;
	case DISP_IF_TVOUT:
		aklcd_start_tvout_clk ();
		lcd_set_reg(1, AK98_LCD_OPER, 1, 1);
		break;
	}
}

static void aklcd_stop_refresh(enum aklcd_if if_type,
			       const struct device *dev)
{
	switch(if_type) {
	case DISP_IF_MPU:
	case DISP_IF_RGB:
		/* stop sync signal */
		lcd_set_reg(1, AK98_LCD_OPER, 0, 0);
		/* stop pclk signal */
		aklcd_stop_pclk();
		break;
	case DISP_IF_TVOUT:
		lcd_set_reg(1, AK98_LCD_OPER, 0, 0);
		aklcd_stop_tvout_clk ();
		break;
	}
}

static void aklcd_set_rgb_panel(const struct aklcd_rgb_panel *panel,
				const struct device *dev)
{
	int h_total_period;
	int v_total_period;

	dev_info(dev, "set rgb panel: %s\n", panel->name);

	/* set display interface */
	lcd_set_reg(DISP_IF_RGB, AK98_LCD_CMD1, 6, 5);

	/* RGB panel type */
	lcd_set_reg(panel->panel_mode, AK98_RGBIF_CTRL1, 20, 20);
	lcd_set_reg(panel->data_width, AK98_RGBIF_CTRL1, 22, 21);
	lcd_set_reg(panel->data_seq, AK98_LCD_CMD1, 12, 12);

#if 0
	__raw_writel(0, AK98_RGBIF_CTRL3);
	__raw_writel(0, AK98_RGBIF_CTRL4);
	__raw_writel(0, AK98_RGBIF_CTRL5);
	__raw_writel(0, AK98_RGBIF_CTRL6);
	__raw_writel(0, AK98_RGBIF_CTRL7);
	__raw_writel(0, AK98_RGBIF_CTRL8);
	__raw_writel(0, AK98_RGBIF_CTRL9);
#endif

	/* Horizontal Timing */
	lcd_set_reg(panel->thpw, AK98_RGBIF_CTRL3, 23, 12);
	lcd_set_reg(panel->thbp, AK98_RGBIF_CTRL4, 23, 12);
	lcd_set_reg(panel->thd,  AK98_RGBIF_CTRL4, 11, 0);
	lcd_set_reg(panel->thfp, AK98_RGBIF_CTRL5, 24, 13);
	h_total_period = panel->thpw + panel->thbp + panel->thd + panel->thfp;
	lcd_set_reg(h_total_period, AK98_RGBIF_CTRL5, 12, 0);

	/* Vertical Timing */
	lcd_set_reg(panel->tvpw, AK98_RGBIF_CTRL3, 11, 0);
	lcd_set_reg(panel->tvbp, AK98_RGBIF_CTRL6, 11, 0);
	lcd_set_reg(panel->tvd,  AK98_RGBIF_CTRL8, 26, 15);
	lcd_set_reg(panel->tvfp, AK98_RGBIF_CTRL7, 23, 12);
	v_total_period = panel->tvpw + panel->tvbp + panel->tvd + panel->tvfp;
	lcd_set_reg(v_total_period, AK98_RGBIF_CTRL9, 12, 0);
	lcd_set_reg(panel->tv_unit, AK98_RGBIF_CTRL8, 0, 0);

	dev_dbg(dev, "H: %i, %i, %i, %i, %i  V:%i, %i, %i, %i, %i\n",
		panel->thpw, panel->thbp, panel->thd, panel->thfp, h_total_period,
		panel->tvpw, panel->tvbp, panel->tvd, panel->tvfp, v_total_period);


	/* Set signals' polarity */
	if (panel->pclk_pol == POL_POSITIVE)
		lcd_set_reg(1, AK98_LCD_CMD1, 4, 4);
	else
		lcd_set_reg(0, AK98_LCD_CMD1, 4, 4);
	if (panel->hsync_pol == POL_POSITIVE)
		lcd_set_reg(0, AK98_RGBIF_CTRL1, 2, 2);
	else
		lcd_set_reg(1, AK98_RGBIF_CTRL1, 2, 2);
	if (panel->vsync_pol == POL_POSITIVE)
		lcd_set_reg(0, AK98_RGBIF_CTRL1, 1, 1);
	else
		lcd_set_reg(1, AK98_RGBIF_CTRL1, 1, 1);
	if (panel->vogate_pol == POL_POSITIVE)
		lcd_set_reg(0, AK98_RGBIF_CTRL1, 0, 0);
	else
		lcd_set_reg(1, AK98_RGBIF_CTRL1, 0, 0);

	/* Configure pins: set shared-pins, and disable pullup function(for energy saving) */
	//ak98_gpio_pullup(AK98_LCD_PINS, AK98_PULLUP_DISABLE);

	if (panel->data_width == RGB24BITS) {
		lcd_set_reg(1, AK98_SRDPIN_CTRL1, 26, 26); /* 8 */
		ak98_gpio_pullup(AK98_GPIO_61, AK98_PULLUP_DISABLE);
		lcd_set_reg(1, AK98_SRDPIN_CTRL1, 25, 25); /* 9-15 */
		ak98_gpio_pullup(AK98_GPIO_62, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_63, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_64, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_65, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_66, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_67, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_68, AK98_PULLUP_DISABLE);

		lcd_set_reg(1, AK98_SRDPIN_CTRL1, 27, 27); /* 16-17 */
		ak98_gpio_pullup(AK98_GPIO_69, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_70, AK98_PULLUP_DISABLE);

		lcd_set_reg(1, AK98_SRDPIN_CTRL2, 8, 8); /* 18-23 */
		ak98_gpio_pullup(AK98_GPIO_84, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_85, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_86, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_87, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_88, AK98_PULLUP_DISABLE);
		ak98_gpio_pullup(AK98_GPIO_89, AK98_PULLUP_DISABLE);
	}
}

#if 0
static void aklcd_set_mpu_panel(const struct aklcd_mpu_panel *panel)
{
}
#endif

static void aklcd_get_tvout_dst_params(struct aklcd_overlay_channel *ov1_channel, enum ak_tvout_mode tvmode)
{
    unsigned int max_width, max_height, disp_left, disp_top;
    unsigned int dst_width, dst_height, dst_left, dst_top;
    unsigned int src_width, src_height;
    
	if (PAL == tvmode) {
		max_width  = TV_PAL_DISP_WIDTH;
		max_height = TV_PAL_DISP_HEIGHT*2;
		disp_left = TV_PAL_DISP_LEFT;
		disp_top = TV_PAL_DISP_TOP;
	} else {
		max_width  = TV_NTSC_DISP_WIDTH;
		max_height = TV_NTSC_DISP_HEIGHT*2;
		disp_left = TV_NTSC_DISP_LEFT;
		disp_top = TV_NTSC_DISP_TOP;
	}
    src_width  = ov1_channel->src_width;
    src_height = ov1_channel->src_height;

	dst_width = max_width;
	dst_height = (dst_width * src_height) / src_width;

	if (dst_height > max_height) {
		dst_height = max_height;
		dst_width = (dst_height * src_width) / src_height;
	}

	if (dst_width == max_width) {
		dst_left = disp_left;
		dst_top = disp_top*2 + (max_height - dst_height)/2;
	} else {		/* dst_height == max_height */
		dst_top = disp_top*2;
		dst_left = disp_left + (max_width - dst_width)/2;
	}

	dst_height /= 2;
	dst_top /= 2;
	
	//2 multipler
	dst_width &= ~1;
	dst_height &= ~1;
	dst_left &= ~1;
	dst_top &= ~1;
	
	ov1_channel->disp_left = dst_left;
	ov1_channel->disp_top  = dst_top;
	ov1_channel->dst_width = dst_width;
	ov1_channel->dst_height = dst_height;
	ov1_channel->use_vpage = false;
}

static void aklcd_set_tvout_panel(enum ak_tvout_mode tvout_mode)
{
	//set output interface
	lcd_set_reg(DISP_IF_TVOUT, AK98_LCD_CMD1, 6, 5);

	switch(tvout_mode)
	{
    	case PAL:
    	    //init the tvout registers in PAL mode.
    		lcd_set_reg(89, TVOUT_PARA_CONFIG_REG6, 7, 0);		
    		lcd_set_reg(625, TVOUT_PARA_CONFIG_REG3, 27, 18);
    		lcd_set_reg(0x2A098ACB, TVOUT_CHROMA_FREQ_REG, 31, 0);	
    		lcd_set_reg(138, TVOUT_PARA_CONFIG_REG1, 7, 0);//????	
    		lcd_set_reg(1440, TVOUT_PARA_CONFIG_REG5, 10, 0);
    		lcd_set_reg(24, TVOUT_PARA_CONFIG_REG5, 31, 24);
    		lcd_set_reg(21, TVOUT_PARA_CONFIG_REG5, 23, 16); //????
    		lcd_set_reg(282, TVOUT_PARA_CONFIG_REG2, 27, 18);
    		lcd_set_reg(280, TVOUT_PARA_CONFIG_REG2, 17, 8);
    		lcd_set_reg(44, TVOUT_PARA_CONFIG_REG2, 7, 0);
    		lcd_set_reg(44, TVOUT_PARA_CONFIG_REG3, 7, 0);
    		lcd_set_reg(137, TVOUT_PARA_CONFIG_REG4, 15, 8);
    		lcd_set_reg(137, TVOUT_PARA_CONFIG_REG4, 7, 0);

    		lcd_set_reg(0, TVOUT_CTRL_REG2, 8, 8); 
    		break;

    	case NTSC:
    	default:
    	    //restore tvout  registers with default values 
    	    //when from PAL to NTSC
            lcd_set_reg(0x21f07c1f, TVOUT_CHROMA_FREQ_REG, 31, 0);
    	    lcd_set_reg(0x7e, TVOUT_PARA_CONFIG_REG1, 23, 16);
    	    lcd_set_reg(0x4476, TVOUT_PARA_CONFIG_REG1, 14, 0);
    	    lcd_set_reg(0x468f03b, TVOUT_PARA_CONFIG_REG2, 27, 0);   
    	    lcd_set_reg(0x8372000, TVOUT_PARA_CONFIG_REG3, 27, 0);
    	    lcd_set_reg(0x16008989, TVOUT_PARA_CONFIG_REG4, 28, 0);
    	    lcd_set_reg(0x2016, TVOUT_PARA_CONFIG_REG5, 31, 16);
    	    lcd_set_reg(0x5a0, TVOUT_PARA_CONFIG_REG5, 10, 0);
    	    lcd_set_reg(0xc8a34802, TVOUT_PARA_CONFIG_REG6, 31, 0);

    	    //Using the same interpolation scale mothed.
    	    lcd_set_reg(1, TVOUT_CTRL_REG1, 28, 28);
    	    //for NTSC mode, need to set soft_rst 
    		lcd_set_reg(0, TVOUT_CTRL_REG2, 8, 8);  		
    		break;
	}
}


static unsigned long aklcd_get_irq_status(void)
{
	return lcd_get_reg(AK98_LCD_STATUS, 31, 0);
}

static void aklcd_set_rgb_irq(void)
{
	lcd_set_reg(1, AK98_LCD_INT_ENAB, 0, 0);
}

static void aklcd_set_alert_line(unsigned i)
{
#if 1
	__raw_writel((1 << 11) | i, AK98_LCD_SOFT_CTRL);
#else
	lcd_set_reg(i, AK98_LCD_SOFT_CTRL, 10, 0);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 11, 11);
#endif
}

/* enable alert irq or not */
static void aklcd_enable_alert(bool enable)
{
	if (enable) {
		/* read AK98_LCD_STATUS register to clean former alert_valid status bit,
		   otherwise alert_valid interrupt will occur immediately */
		aklcd_get_irq_status();
		lcd_set_reg(1, AK98_LCD_INT_ENAB, 17, 17);
	} else {
		lcd_set_reg(0, AK98_LCD_INT_ENAB, 17, 17);
	}
}

static void aklcd_set_screen(struct aklcd_screen *screen)
{
	/* set screen geometry */
	lcd_set_reg(screen->width, AK98_DISP_SIZE, 21, 11);
	lcd_set_reg(screen->height, AK98_DISP_SIZE, 10, 0);

	/* set background color */
	lcd_set_reg(screen->bg_color, AK98_BG_COLOR, 23, 0);

	/* alert setting, for set_par (act as vsync) */
	aklcd_set_alert_line(screen->height);
}

static void aklcd_set_rgb_channel(struct aklcd_rgb_channel *rgb_channel)
{
	/* source: */
	lcd_set_reg(rgb_channel->format, AK98_LCD_CMD1, 14, 13);
	lcd_set_reg(rgb_channel->data_addr, AK98_RGBIF_CTRL2, 28, 0);
	if (rgb_channel->use_vpage) {
		lcd_set_reg(1, AK98_RGBIF_CTRL2, 29, 29);
		lcd_set_reg(rgb_channel->vpage_width, AK98_RGB_VPAGE_SIZE, 31, 16);
		lcd_set_reg(rgb_channel->vpage_height, AK98_RGB_VPAGE_SIZE, 15, 0);
		lcd_set_reg(rgb_channel->virt_left, AK98_RGB_VPAGE_OFFSET, 31, 16);
		lcd_set_reg(rgb_channel->virt_top, AK98_RGB_VPAGE_OFFSET, 15, 0);
	} else {
		lcd_set_reg(0, AK98_RGBIF_CTRL2, 29, 29);
	}

	/* destination: (dst size is screen size, and already set in aklcd_set_screen() ) */
	lcd_set_reg(rgb_channel->disp_left, AK98_RGB_OFFSET, 21, 11);
	lcd_set_reg(rgb_channel->disp_top, AK98_RGB_OFFSET, 10, 0);
	lcd_set_reg(rgb_channel->width,  AK98_RGB_SIZE, 21, 11);
	lcd_set_reg(rgb_channel->height, AK98_RGB_SIZE, 10, 0);
}

static void aklcd_set_ov1_channel(struct aklcd_overlay_channel *ov1_channel)
{
	unsigned int scaler;

	/* src: */
	lcd_set_reg(ov1_channel->src_range, AK98_OV1_DISP_CONF, 27, 27);
	lcd_set_reg(ov1_channel->y_addr, AK98_OV1_YADDR, 28, 0);
	lcd_set_reg(ov1_channel->u_addr, AK98_OV1_UADDR, 28, 0);
	lcd_set_reg(ov1_channel->v_addr, AK98_OV1_VADDR, 28, 0);
	lcd_set_reg(ov1_channel->src_width, AK98_OV1_HORI_CONF, 10, 0);
	lcd_set_reg(ov1_channel->src_height, AK98_OV1_VERT_CONF, 10, 0);

	/* virtual page */
	if (ov1_channel->use_vpage) {
		lcd_set_reg(1, AK98_OV1_DISP_CONF, 24, 24);
		lcd_set_reg(ov1_channel->vpage_width, AK98_OV1_VPAGE_SIZE, 31, 16);
		lcd_set_reg(ov1_channel->vpage_height, AK98_OV1_VPAGE_SIZE, 15, 0);
		lcd_set_reg(ov1_channel->virt_left, AK98_OV1_VPAGE_OFFSET, 31, 16);
		lcd_set_reg(ov1_channel->virt_top, AK98_OV1_VPAGE_OFFSET, 15, 0);
	} else {
		lcd_set_reg(0, AK98_OV1_DISP_CONF, 24, 24);
	}

	/* dst: */
	lcd_set_reg(ov1_channel->disp_left, AK98_OV1_DISP_CONF, 21, 11);
	lcd_set_reg(ov1_channel->disp_top, AK98_OV1_DISP_CONF, 10, 0);
	lcd_set_reg(ov1_channel->dst_width, AK98_OV1_HORI_CONF, 21, 11);
	lcd_set_reg(ov1_channel->dst_height, AK98_OV1_VERT_CONF, 21, 11);
	if (ov1_channel->dst_width != ov1_channel->src_width) {
		lcd_set_reg(1, AK98_OV1_DISP_CONF, 23, 23);
		scaler = AKLCD_HSCALER(ov1_channel->dst_width);
		if (scaler == 0) scaler = 0xfff;
		lcd_set_reg(scaler, AK98_OV1_SCALER, 11, 0);
	} else {
		lcd_set_reg(0, AK98_OV1_DISP_CONF, 23, 23);
	}
	if (ov1_channel->dst_height != ov1_channel->src_height) {
		lcd_set_reg(1, AK98_OV1_DISP_CONF, 22, 22);
		scaler = AKLCD_VSCALER(ov1_channel->dst_height);
		if (scaler == 0) scaler = 0xfff;
		lcd_set_reg(scaler, AK98_OV1_SCALER, 23, 12);
		/* for reduce DMA translation */
		if (ov1_channel->dst_height < ov1_channel->src_height) {
			lcd_set_reg(1, AK98_OV1_DISP_CONF, 25, 25);
		} else {
			lcd_set_reg(0, AK98_OV1_DISP_CONF, 25, 25);
		}
	} else {
		lcd_set_reg(0, AK98_OV1_DISP_CONF, 22, 22);
	}
}

static void aklcd_set_ov2_channel(struct aklcd_overlay_channel *ov2_channel)
{
	unsigned int scaler;
	unsigned int ov1_left, ov1_right, ov1_top, ov1_bottom;

	/* src: */
	lcd_set_reg(ov2_channel->y_addr, AK98_OV2_YADDR, 28, 0);
	lcd_set_reg(ov2_channel->u_addr, AK98_OV2_UADDR, 28, 0);
	lcd_set_reg(ov2_channel->v_addr, AK98_OV2_VADDR, 28, 0);
	lcd_set_reg(ov2_channel->src_width, AK98_OV2_HORI_CONF, 10, 0);
	lcd_set_reg(ov2_channel->src_height, AK98_OV2_VERT_CONF, 10, 0);

	/* dst: */
	lcd_set_reg(ov2_channel->disp_left, AK98_OV2_DISP_CONF, 21, 11);
	lcd_set_reg(ov2_channel->disp_top, AK98_OV2_DISP_CONF, 10, 0);
	lcd_set_reg(ov2_channel->dst_width, AK98_OV2_HORI_CONF, 21, 11);
	lcd_set_reg(ov2_channel->dst_height, AK98_OV2_VERT_CONF, 21, 11);
	if (ov2_channel->dst_width != ov2_channel->src_width) {
		lcd_set_reg(1, AK98_OV2_DISP_CONF, 23, 23);
		scaler = AKLCD_HSCALER(ov2_channel->dst_width);
		if (scaler == 0) scaler = 0xfff;
		lcd_set_reg(scaler, AK98_OV2_SCALER, 11, 0);
	} else {
		lcd_set_reg(0, AK98_OV2_DISP_CONF, 23, 23);
	}
	if (ov2_channel->dst_height != ov2_channel->src_height) {
		lcd_set_reg(1, AK98_OV2_DISP_CONF, 22, 22);
		scaler = AKLCD_VSCALER(ov2_channel->dst_height);
		if (scaler == 0) scaler = 0xfff;
		lcd_set_reg(scaler, AK98_OV2_SCALER, 23, 12);
		/* for reduce DMA translation */
		if (ov2_channel->dst_height < ov2_channel->src_height) {
			lcd_set_reg(1, AK98_OV2_DISP_CONF, 25, 25);
		} else {
			lcd_set_reg(0, AK98_OV2_DISP_CONF, 25, 25);
		}
	} else {
		lcd_set_reg(0, AK98_OV2_DISP_CONF, 22, 22);
	}

	/* 0: ov2 over ov1; 1: ov2 ex ov1; */
	if (lcd_get_reg(AK98_LCD_CMD1, 2, 2)) {
		ov1_left = lcd_get_reg(AK98_OV1_DISP_CONF, 21, 11);
		ov1_top = lcd_get_reg(AK98_OV1_DISP_CONF, 10, 0);
		ov1_right = ov1_left + lcd_get_reg(AK98_OV1_HORI_CONF, 21, 11);
		ov1_bottom = ov1_top + lcd_get_reg(AK98_OV1_VERT_CONF, 21, 11);
	} else {
		ov1_left = ov1_top = ov1_right = ov1_bottom = 0;
	}
	if (ov2_channel->disp_left >= ov1_left && (ov2_channel->disp_left + ov2_channel->dst_width) <= ov1_right
	    && ov2_channel->disp_top >= ov1_top && (ov2_channel->disp_top + ov2_channel->dst_height) <= ov1_bottom) {
		lcd_set_reg(0, AK98_OV2_DISP_CONF, 24, 24);
	} else if ((ov2_channel->disp_left + ov2_channel->dst_width) < ov1_left || ov2_channel->disp_left >= ov1_right
		   || (ov2_channel->disp_top + ov2_channel->dst_height) < ov1_top || ov2_channel->disp_top >= ov1_bottom) {
		lcd_set_reg(1, AK98_OV2_DISP_CONF, 24, 24);
	} else {
		return;
	}

	lcd_set_reg(ov2_channel->alpha, AK98_OV2_DISP_CONF, 29, 26);
}

static void aklcd_set_osd_channel(struct aklcd_osd_channel *osd_channel)
{
	/* src: */
	lcd_set_reg(osd_channel->data_addr, AK98_OSD_ADDR, 28, 0);

	/* Set color palette */
	lcd_set_reg(osd_channel->palette[1], AK98_OSD_COLOR1, 31, 16);
	lcd_set_reg(osd_channel->palette[2], AK98_OSD_COLOR1, 15, 0);
	lcd_set_reg(osd_channel->palette[3], AK98_OSD_COLOR2, 31, 16);
	lcd_set_reg(osd_channel->palette[4], AK98_OSD_COLOR2, 15, 0);
	lcd_set_reg(osd_channel->palette[5], AK98_OSD_COLOR3, 31, 16);
	lcd_set_reg(osd_channel->palette[6], AK98_OSD_COLOR3, 15, 0);
	lcd_set_reg(osd_channel->palette[7], AK98_OSD_COLOR4, 31, 16);
	lcd_set_reg(osd_channel->palette[8], AK98_OSD_COLOR4, 15, 0);
	lcd_set_reg(osd_channel->palette[9], AK98_OSD_COLOR5, 31, 16);
	lcd_set_reg(osd_channel->palette[10], AK98_OSD_COLOR5, 15, 0);
	lcd_set_reg(osd_channel->palette[11], AK98_OSD_COLOR6, 31, 16);
	lcd_set_reg(osd_channel->palette[12], AK98_OSD_COLOR6, 15, 0);
	lcd_set_reg(osd_channel->palette[13], AK98_OSD_COLOR7, 31, 16);
	lcd_set_reg(osd_channel->palette[14], AK98_OSD_COLOR7, 15, 0);
	lcd_set_reg(osd_channel->palette[15], AK98_OSD_COLOR8, 15, 0);

	/* dst: */
	lcd_set_reg(osd_channel->disp_left, AK98_OSD_OFFSET, 21, 11);
	lcd_set_reg(osd_channel->disp_top, AK98_OSD_OFFSET, 10, 0);
	lcd_set_reg(osd_channel->width, AK98_OSD_SIZE_ALPHA, 10, 0);
	lcd_set_reg(osd_channel->height, AK98_OSD_SIZE_ALPHA, 21, 11);

	lcd_set_reg(osd_channel->alpha, AK98_OSD_SIZE_ALPHA, 25, 22);
}

static void aklcd_change_rgb_vpage_offset(unsigned xoffset, unsigned yoffset)
{
	lcd_set_reg(xoffset, AK98_RGB_VPAGE_OFFSET, 31, 16);
	lcd_set_reg(yoffset, AK98_RGB_VPAGE_OFFSET, 15, 0);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_change_rgb_channel(struct aklcd_rgb_channel *rgb_channel)
{
	aklcd_set_rgb_channel(rgb_channel);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_change_ov1_channel(struct aklcd_overlay_channel *ov1_channel)
{
	aklcd_set_ov1_channel(ov1_channel);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_change_ov2_channel(struct aklcd_overlay_channel *ov2_channel)
{
	aklcd_set_ov2_channel(ov2_channel);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_change_osd_channel(struct aklcd_osd_channel *osd_channel)
{
	aklcd_set_osd_channel(osd_channel);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_open_rgb_channel(void)
{
	/* open rgb data channel */
	lcd_set_reg(1, AK98_LCD_CMD1, 3, 3);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_open_ov1_channel(void)
{
	/* open yuv1 data channel */
	lcd_set_reg(1, AK98_LCD_CMD1, 2, 2);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_open_ov2_channel(void)
{
	/* open yuv2 data channel */
	lcd_set_reg(1, AK98_LCD_CMD1, 1, 1);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_open_osd_channel(void)
{
	/* open osd data channel */
	lcd_set_reg(1, AK98_LCD_CMD1, 0, 0);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_close_rgb_channel(void)
{
	lcd_set_reg(0, AK98_LCD_CMD1, 3, 3);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_close_ov1_channel(void)
{
	lcd_set_reg(0, AK98_LCD_CMD1, 2, 2);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_close_ov2_channel(void)
{
	lcd_set_reg(0, AK98_LCD_CMD1, 1, 1);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static void aklcd_close_osd_channel(void)
{
	lcd_set_reg(0, AK98_LCD_CMD1, 0, 0);
	lcd_set_reg(1, AK98_LCD_SOFT_CTRL, 12, 12);
}

static bool aklcd_get_rgb_enable(void)
{
	return (lcd_get_reg(AK98_LCD_CMD1, 3, 3) == 1);
}

static bool aklcd_get_ov1_enable(void)
{
	return (lcd_get_reg(AK98_LCD_CMD1, 2, 2) == 1);
}

static bool aklcd_get_ov2_enable(void)
{
	return (lcd_get_reg(AK98_LCD_CMD1, 1, 1) == 1);
}

static bool aklcd_get_osd_enable(void)
{
	return (lcd_get_reg(AK98_LCD_CMD1, 0, 0) == 1);
}


/********************
 * irq handle
 ********************/
static inline void handle_alert_irq(struct akfb *akfb, struct device *dev)
{
	aklcd_enable_alert(false);

	/* after read the final line to inner FIFO,
	   LCD controller should use new parameter now */
	if (test_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits))) {
		complete(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
	}

	if (test_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits))) {
		/* update ov1ch information */
		memcpy(&(akfb->ov1ch), &(akfb->ov1ch_new),sizeof(struct aklcd_overlay_channel));
	}

	if (test_bit(AKLCD_CHANNEL_OV2, &(akfb->par_change_bits))) {
		memcpy(&(akfb->ov2ch), &(akfb->ov2ch_new),sizeof(struct aklcd_overlay_channel));
	}

	if (test_bit(AKLCD_CHANNEL_OSD, &(akfb->par_change_bits))) {
		memcpy(&(akfb->osdch), &(akfb->osdch_new),sizeof(struct aklcd_osd_channel));
	}
}

static irqreturn_t akfb_irq_handler(int irq, void *dev_id)
{
	struct akfb   *akfb = dev_id;
	struct device *dev = &(akfb->pdev->dev);
	u32 irq_status;

	irq_status = aklcd_get_irq_status();

	if (irq_status & 0x01) {
		/* lcd controller error */
		dev_err(dev, "LCD controller system error\n");
		/* restart sync signal */
		aklcd_stop_refresh(akfb->if_type, dev);
		aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
		akfb->panel_refreshing = true;

		return IRQ_HANDLED;
	}

	if (irq_status & (1<<17)) {
		handle_alert_irq(akfb, dev);
	}

	return IRQ_HANDLED;
}

/********************
 * fb_ops
 ********************/
static const struct fb_bitfield rgb565_r = {.offset=11, .length=5, .msb_right=0};
static const struct fb_bitfield rgb565_g = {.offset=5,  .length=6, .msb_right=0};
static const struct fb_bitfield rgb565_b = {.offset=0,  .length=5, .msb_right=0};
static const struct fb_bitfield bgr565_b = {.offset=11, .length=5, .msb_right=0};
static const struct fb_bitfield bgr565_g = {.offset=5,  .length=6, .msb_right=0};
static const struct fb_bitfield bgr565_r = {.offset=0,  .length=5, .msb_right=0};
static const struct fb_bitfield rgb888_r = {.offset=16, .length=8, .msb_right=0};
static const struct fb_bitfield rgb888_g = {.offset=8,  .length=8, .msb_right=0};
static const struct fb_bitfield rgb888_b = {.offset=0,  .length=8, .msb_right=0};
static const struct fb_bitfield bgr888_b = {.offset=16, .length=8, .msb_right=0};
static const struct fb_bitfield bgr888_g = {.offset=8,  .length=8, .msb_right=0};
static const struct fb_bitfield bgr888_r = {.offset=0,  .length=8, .msb_right=0};

static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

static int akfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			  unsigned blue, unsigned transp,
			  struct fb_info *fb)
{
	/* anyka lcd controller only support true color */
	if (regno >= 16)
		return -EINVAL;

	((u32*)(fb->pseudo_palette))[regno] = convert_bitfield(transp, &(fb->var.transp)) |
		convert_bitfield(blue, &(fb->var.blue)) |
		convert_bitfield(green, &(fb->var.green)) |
		convert_bitfield(red, &(fb->var.red));

	return 0;
}

static int __akfb_get_pixel_format(const struct fb_var_screeninfo *var)
{
	if (var->bits_per_pixel == 16) {
		if (memcmp(&(var->red), &rgb565_r, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->green), &rgb565_g, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->blue), &rgb565_b, sizeof(struct fb_bitfield)) == 0)
			return AKLCD_RGB565;

		if (memcmp(&(var->red), &bgr565_r, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->green), &bgr565_g, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->blue), &bgr565_b, sizeof(struct fb_bitfield)) == 0)
			return AKLCD_BGR565;
	}

	if (var->bits_per_pixel == 24) {
		if (memcmp(&(var->red), &rgb888_r, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->green), &rgb888_g, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->blue), &rgb888_b, sizeof(struct fb_bitfield)) == 0)
			return AKLCD_RGB888;

		if (memcmp(&(var->red), &bgr888_r, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->green), &bgr888_g, sizeof(struct fb_bitfield)) == 0
		    && memcmp(&(var->blue), &bgr888_b, sizeof(struct fb_bitfield)) == 0)
			return AKLCD_BGR888;
	}

	return -EINVAL;
}

static int akfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct akfb *akfb;
	struct device *dev;

	akfb = fb->par;
	dev = &(akfb->pdev->dev);

	if (var->xres_virtual < var->xres || var->yres_virtual < var->yres) {
		return -EINVAL;
	}

	if (var->xoffset > (var->xres_virtual - var->xres)
	    || var->yoffset > (var->yres_virtual - var->yres)) {
		return -EINVAL;
	}

	if (var->xres > akfb->screen.width || var->yres > akfb->screen.height
	    || var->xres_virtual > AKLCD_VPAGE_WIDTH_MAX || var->yres_virtual > AKLCD_VPAGE_HEIGHT_MAX) {
		return -EINVAL;
	}

	if (__akfb_get_pixel_format(var) < 0) {
		return -EINVAL;
	}

	return 0;
}

static int akfb_set_par(struct fb_info *fb)
{
	struct akfb                  *akfb;
	struct aklcd_rgb_channel     new_rgbch;
	struct aklcd_channel_meminfo new_rgbch_meminfo;
	struct device                *dev;
	int                          new_rgbch_bpp;

	akfb = fb->par;
	dev = &(akfb->pdev->dev);

	mutex_lock_interruptible(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	init_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));

	new_rgbch.format       = __akfb_get_pixel_format(&fb->var);
	new_rgbch.use_vpage    = true; /* alway enable virtual page */
	new_rgbch.vpage_width  = fb->var.xres_virtual;
	new_rgbch.vpage_height = fb->var.yres_virtual;
	new_rgbch.virt_left    = fb->var.xoffset;
	new_rgbch.virt_top     = fb->var.yoffset;
	new_rgbch.width        = fb->var.xres;
	new_rgbch.height       = fb->var.yres;
	new_rgbch.disp_left    = akfb->screen.width / 2 - new_rgbch.width / 2;
	new_rgbch.disp_top     = akfb->screen.height / 2 - new_rgbch.height / 2;

	if(new_rgbch.format == AKLCD_RGB565 || new_rgbch.format == AKLCD_BGR565)
		new_rgbch_bpp   = 2;
	else
		new_rgbch_bpp   = 3;

	if (new_rgbch.use_vpage)
		new_rgbch_meminfo.size  = new_rgbch.vpage_width * new_rgbch.vpage_height * new_rgbch_bpp;
	else
		new_rgbch_meminfo.size  = new_rgbch.width * new_rgbch.height * new_rgbch_bpp;
	dev_info(dev, "new fb memory size: %u\n", new_rgbch_meminfo.size);

	if (new_rgbch_meminfo.size <= akfb->rgbch_meminfo.size
	    && new_rgbch_meminfo.size > akfb->rgbch_meminfo.size/2) {
		dev_info(dev, "reuse current fb memory\n");
		new_rgbch_meminfo.paddr = akfb->rgbch_meminfo.paddr;
		new_rgbch_meminfo.vaddr = akfb->rgbch_meminfo.vaddr;
		new_rgbch_meminfo.size  = akfb->rgbch_meminfo.size;
	} else {
		new_rgbch_meminfo.vaddr =
			dma_alloc_writecombine(dev, new_rgbch_meminfo.size,
					       &new_rgbch_meminfo.paddr, GFP_KERNEL);
		if (new_rgbch_meminfo.vaddr == NULL) {
			dev_err(dev, "alloc fb memory error\n");
			if (new_rgbch_meminfo.size <= akfb->rgbch_meminfo.size) {
				dev_info(dev, "reuse current fb memory\n");
				new_rgbch_meminfo.paddr = akfb->rgbch_meminfo.paddr;
				new_rgbch_meminfo.vaddr = akfb->rgbch_meminfo.vaddr;
				new_rgbch_meminfo.size  = akfb->rgbch_meminfo.size;
			} else {
				mutex_unlock(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));
				return -ENOMEM;
			}
		}
	}

	new_rgbch.data_addr   = new_rgbch_meminfo.paddr;
	dev_info(dev, "framebuffer address: 0x%08x\n", new_rgbch_meminfo.paddr);

	aklcd_change_rgb_channel(&new_rgbch);

	if (akfb->panel_refreshing) {
		set_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits));

		aklcd_enable_alert(true);

		wait_for_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
	}

	if (new_rgbch_meminfo.paddr != akfb->rgbch_meminfo.paddr) {
		dma_free_writecombine(dev, akfb->rgbch_meminfo.size,
				      akfb->rgbch_meminfo.vaddr, akfb->rgbch_meminfo.paddr);
		memcpy(&(akfb->rgbch_meminfo), &new_rgbch_meminfo, sizeof(struct aklcd_channel_meminfo));
	}
	memcpy(&(akfb->rgbch), &new_rgbch, sizeof(struct aklcd_rgb_channel));

	/* update fix info */
	fb->fix.smem_start = new_rgbch_meminfo.paddr;
	if (new_rgbch.use_vpage) {
		fb->fix.line_length = new_rgbch.vpage_width * new_rgbch_bpp;
		/* in reuse fb memory situation, new_rgbch_meminfo.size maybe not equal to 
		   actual fb memory size, so we need to calculate it */
		fb->fix.smem_len   = fb->fix.line_length * new_rgbch.vpage_height;
	} else {
		fb->fix.line_length = new_rgbch.width * new_rgbch_bpp;
		fb->fix.smem_len   = fb->fix.line_length * new_rgbch.height;
	}

	fb->screen_base = new_rgbch_meminfo.vaddr;
	fb->screen_size = fb->fix.smem_len;

	mutex_unlock(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	return 0;
}

static int akfb_pan_display(struct fb_var_screeninfo *var,
			    struct fb_info *fb)
{
	struct akfb                  *akfb;
	struct device                *dev;

	akfb = fb->par;
	dev = &(akfb->pdev->dev);

	if (var->xoffset == fb->var.xoffset && var->yoffset == fb->var.yoffset) {
		dev_info(dev, "xoffset & yoffset not change\n");
		return 0;
	}

	mutex_lock_interruptible(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	init_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));

	aklcd_change_rgb_vpage_offset(var->xoffset, var->yoffset);

	if (akfb->panel_refreshing) {
		set_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits));

		aklcd_enable_alert(true);

		wait_for_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
	}

	akfb->rgbch.virt_left = var->xoffset;
	akfb->rgbch.virt_top = var->yoffset;

	mutex_unlock(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	return 0;
}

static int akfb_blank(int blank, struct fb_info *fb)
{
	struct akfb    *akfb;
	struct device  *dev;

	akfb = fb->par;
	dev = &(akfb->pdev->dev);

	mutex_lock_interruptible(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (akfb->panel_refreshing == true) {
			aklcd_stop_refresh(akfb->if_type, dev);
			aklcd_set_power(false);
			akfb->panel_refreshing = false;
		}
		break;
	case FB_BLANK_UNBLANK:
		if (akfb->panel_refreshing == false) {
			aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
			aklcd_set_power(true);
			akfb->panel_refreshing = true;
		}
		break;
	}

	mutex_unlock(&(akfb->aklcd_par_lock[AKLCD_CHANNEL_RGB]));

	return 0;
}

static int akfb_ioctl(struct fb_info *fb, unsigned int cmd, unsigned long arg)
{
	struct akfb               *akfb;
	struct device             *dev;
	struct aklcd_overlay_info ovi;
	struct aklcd_osd_info     osdi;
	int                       ret;
	enum ak_tvout_mode        tvout_mode = TVOUT_OFF;

	akfb = fb->par;
	dev = &(akfb->pdev->dev);
	ret = 0;

	switch(cmd) {
	case FBIOGET_AKOVINFO:
		if (copy_from_user(&ovi, (void __user *)arg, sizeof(struct aklcd_overlay_info)))
			return -EFAULT;

		switch(ovi.overlay_id) {
		case 0:
			ovi.enable = aklcd_get_ov1_enable();
			if (ovi.enable) {
				memcpy(&(ovi.overlay_setting), &(akfb->ov1ch),
				       sizeof(struct aklcd_overlay_channel));
			}
			ret = copy_to_user((void __user *)arg, &ovi,
					   sizeof(struct aklcd_overlay_info)) ? -EFAULT : 0;
			break;
		case 1:
			ovi.enable = aklcd_get_ov2_enable();
			if (ovi.enable) {
				memcpy(&(ovi.overlay_setting), &(akfb->ov2ch),
				       sizeof(struct aklcd_overlay_channel));
			}
			ret = copy_to_user((void __user *)arg, &ovi,
					   sizeof(struct aklcd_overlay_info)) ? -EFAULT : 0;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case FBIOPUT_AKOVINFO:
		if (copy_from_user(&ovi, (void __user *)arg, sizeof(struct aklcd_overlay_info)))
			return -EFAULT;

		switch(ovi.overlay_id) {
		case 0:
			if (ovi.enable == false) {
				if (aklcd_get_ov1_enable() == true)
					aklcd_close_ov1_channel();
			} else {
				if (aklcd_get_ov1_enable() == false) {
					aklcd_set_ov1_channel(&(ovi.overlay_setting));
					aklcd_open_ov1_channel();
				} else {
					aklcd_change_ov1_channel(&(ovi.overlay_setting));
				}
				memcpy(&(akfb->ov1ch_new), &(ovi.overlay_setting),
				       sizeof(struct aklcd_overlay_channel));
				set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
			}
			break;
		case 1:
			if (ovi.enable == false) {
				if (aklcd_get_ov2_enable() == true)
					aklcd_close_ov2_channel();
			} else {
				if (aklcd_get_ov2_enable() == false) {
					aklcd_set_ov2_channel(&(ovi.overlay_setting));
					aklcd_open_ov2_channel();
				} else {
					aklcd_change_ov2_channel(&(ovi.overlay_setting));
				}
				memcpy(&(akfb->ov2ch_new), &(ovi.overlay_setting),
				       sizeof(struct aklcd_overlay_channel));

				set_bit(AKLCD_CHANNEL_OV2, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
			}
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case FBIOGET_AKOSDINFO:
		memset(&osdi, 0, sizeof(osdi)); /* avoid kernel data leak */
		osdi.enable = aklcd_get_osd_enable();
		if (osdi.enable) {
			memcpy(&(osdi.osd_setting), &(akfb->osdch),
			       sizeof(struct aklcd_osd_channel));
		}

		ret = copy_to_user((void __user *)arg, &osdi,
				   sizeof(struct aklcd_osd_info)) ? -EFAULT : 0;
		break;

	case FBIOPUT_AKOSDINFO:
		if (copy_from_user(&osdi, (void __user *)arg, sizeof(struct aklcd_osd_info)))
			return -EFAULT;

		if (osdi.enable == false) {
			if (aklcd_get_osd_enable() == true)
				aklcd_close_osd_channel();
		} else {
			if (aklcd_get_osd_enable() == false) {
				aklcd_set_osd_channel(&(osdi.osd_setting));
				aklcd_open_osd_channel();
			} else {
				aklcd_change_osd_channel(&(osdi.osd_setting));
			}
			memcpy(&(akfb->osdch_new), &(osdi.osd_setting),
			       sizeof(struct aklcd_osd_channel));
			set_bit(AKLCD_CHANNEL_OSD, &(akfb->par_change_bits));
			aklcd_enable_alert(true);
		}	
		break;

	case FBIOPUT_AKOVPOSI:
		if (copy_from_user(&ovi, (void __user *)arg, sizeof(struct aklcd_overlay_info)))
			return -EFAULT;

		switch(ovi.overlay_id) {
		case 0:
			if (ovi.enable == false) {
				if (aklcd_get_ov1_enable() == true)
					aklcd_close_ov1_channel();
			} else {
				if (akfb->tvout_mode == TVOUT_OFF) {
					akfb->ov1ch_new.src_range = ovi.overlay_setting.src_range;
					akfb->ov1ch_new.src_width = ovi.overlay_setting.src_width;
					akfb->ov1ch_new.src_height = ovi.overlay_setting.src_height;
					akfb->ov1ch_new.dst_width = ovi.overlay_setting.dst_width;
					akfb->ov1ch_new.dst_height = ovi.overlay_setting.dst_height;
					akfb->ov1ch_new.disp_left = ovi.overlay_setting.disp_left;
					akfb->ov1ch_new.disp_top = ovi.overlay_setting.disp_top;
					akfb->ov1ch_new.use_vpage = false;
					set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
					aklcd_enable_alert(true);
				} else {
					akfb->ov1ch_saved.src_range = ovi.overlay_setting.src_range;
					akfb->ov1ch_saved.src_width = ovi.overlay_setting.src_width;
					akfb->ov1ch_saved.src_height = ovi.overlay_setting.src_height;
					akfb->ov1ch_saved.dst_width = ovi.overlay_setting.dst_width;
					akfb->ov1ch_saved.dst_height = ovi.overlay_setting.dst_height;
					akfb->ov1ch_saved.disp_left = ovi.overlay_setting.disp_left;
					akfb->ov1ch_saved.disp_top = ovi.overlay_setting.disp_top;
					akfb->ov1ch_saved.use_vpage = false;

					if (akfb->ov1ch_new.src_width != ovi.overlay_setting.src_width
					    || akfb->ov1ch_new.src_height != ovi.overlay_setting.src_height) {

                        akfb->ov1ch_new.src_range = ovi.overlay_setting.src_range;
                        akfb->ov1ch_new.src_width = ovi.overlay_setting.src_width;
                        akfb->ov1ch_new.src_height = ovi.overlay_setting.src_height;
                        akfb->ov1ch_new.dst_width = ovi.overlay_setting.dst_width;
                        akfb->ov1ch_new.dst_height = ovi.overlay_setting.dst_height;
                        akfb->ov1ch_new.disp_left = ovi.overlay_setting.disp_left;
                        akfb->ov1ch_new.disp_top = ovi.overlay_setting.disp_top;

						aklcd_get_tvout_dst_params(&akfb->ov1ch_new,akfb->tvout_mode);
						aklcd_set_ov1_channel(&akfb->ov1ch_new);					
						set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
						aklcd_enable_alert(true);
					}
				}
			}
			break;
		case 1:
			if (ovi.enable == false) {
				if (aklcd_get_ov2_enable() == true)
					aklcd_close_ov2_channel();
			} else {
				akfb->ov2ch_new.alpha = ovi.overlay_setting.alpha;
				akfb->ov2ch_new.src_width = ovi.overlay_setting.src_width;
				akfb->ov2ch_new.src_height = ovi.overlay_setting.src_height;
				akfb->ov2ch_new.dst_width = ovi.overlay_setting.dst_width;
				akfb->ov2ch_new.dst_height = ovi.overlay_setting.dst_height;
				akfb->ov2ch_new.disp_left = ovi.overlay_setting.disp_left;
				akfb->ov2ch_new.disp_top = ovi.overlay_setting.disp_top;
				set_bit(AKLCD_CHANNEL_OV2, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
			}	
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case FBIOPUT_AKOVDATA:
		if (copy_from_user(&ovi, (void __user *)arg, sizeof(struct aklcd_overlay_info)))
			return -EFAULT;

		switch(ovi.overlay_id) {
		case 0:
			if (ovi.enable == false) {
				if (aklcd_get_ov1_enable() == true)
					aklcd_close_ov1_channel();
			} else {
				akfb->ov1ch_new.y_addr = ovi.overlay_setting.y_addr;
				akfb->ov1ch_new.u_addr = ovi.overlay_setting.u_addr;
				akfb->ov1ch_new.v_addr = ovi.overlay_setting.v_addr;
				if (aklcd_get_ov1_enable() == false) {
					aklcd_set_ov1_channel(&(akfb->ov1ch_new));
					aklcd_open_ov1_channel();
				} else {
					aklcd_change_ov1_channel(&(akfb->ov1ch_new));
				}
				set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
			}	
			break;
		case 1:
			if (ovi.enable == false) {
				if (aklcd_get_ov2_enable() == true)
					aklcd_close_ov2_channel();
			} else {
				akfb->ov2ch_new.y_addr = ovi.overlay_setting.y_addr;
				akfb->ov2ch_new.u_addr = ovi.overlay_setting.u_addr;
				akfb->ov2ch_new.v_addr = ovi.overlay_setting.v_addr;

				if (aklcd_get_ov2_enable() == false) {
					aklcd_set_ov2_channel(&(akfb->ov2ch_new));
					aklcd_open_ov2_channel();
				} else {
					aklcd_change_ov2_channel(&(akfb->ov2ch_new));
				}
		
				set_bit(AKLCD_CHANNEL_OV2, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
			}	
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case FBIOPUT_AKOVSHOWING:
		if (copy_from_user(&ovi, (void __user *)arg, sizeof(struct aklcd_overlay_info)))
			return -EFAULT;

		switch(ovi.overlay_id) {
		case 0:
			if (ovi.enable == false) {
				if (aklcd_get_ov1_enable() == true)
					aklcd_close_ov1_channel();
			} else {
				if (aklcd_get_ov1_enable() == false) {
					aklcd_set_ov1_channel(&(akfb->ov1ch));
					aklcd_open_ov1_channel();
				}
			}
			break;
		case 1:
			if (ovi.enable == false) {
				if (aklcd_get_ov2_enable() == true)
					aklcd_close_ov2_channel();
			} else {
				if (aklcd_get_ov2_enable() == false) {
					aklcd_set_ov2_channel(&(akfb->ov2ch));
					aklcd_open_ov2_channel();
				}
			}
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case FBIOPUT_AKTVOUT:
		if (copy_from_user(&tvout_mode, (void __user *)arg, sizeof(int)))
			return -EFAULT;

		printk("++FBIOPUT_AKTVOUT tvout_mode=%d,akfb->tvout_mode=%d\n",tvout_mode,akfb->tvout_mode);

		switch (tvout_mode) {
		case TVOUT_OFF:         //turn off tvout
			if (akfb->tvout_mode == TVOUT_OFF)
				break;

			aklcd_stop_refresh(akfb->if_type, dev);
			akfb->tvout_mode = TVOUT_OFF;
			akfb->if_type = DISP_IF_RGB;

            aklcd_close_osd_channel();
            aklcd_close_ov1_channel();
            aklcd_close_ov2_channel();
            aklcd_close_rgb_channel();

			/* set screen */
			akfb->screen.width    = bsp_rgb_panel.thd;
			akfb->screen.height   = bsp_rgb_panel.tvd;
			akfb->screen.bg_color = AKRGB(0, 0, 0);
			aklcd_set_screen(&akfb->screen);

			aklcd_set_rgb_channel(&akfb->rgbch);
			aklcd_open_rgb_channel();

			//restore akfb->ov1ch_new
			akfb->ov1ch_new.src_range  = akfb->ov1ch_saved.src_range;
			akfb->ov1ch_new.src_width  = akfb->ov1ch_saved.src_width;
			akfb->ov1ch_new.src_height = akfb->ov1ch_saved.src_height;
			akfb->ov1ch_new.dst_width  = akfb->ov1ch_saved.dst_width;
			akfb->ov1ch_new.dst_height = akfb->ov1ch_saved.dst_height;
			akfb->ov1ch_new.disp_left  = akfb->ov1ch_saved.disp_left;
			akfb->ov1ch_new.disp_top   = akfb->ov1ch_saved.disp_top;
			akfb->ov1ch_new.use_vpage  = akfb->ov1ch_saved.use_vpage;
			aklcd_set_ov1_channel(&akfb->ov1ch_new);
			aklcd_open_ov1_channel();
			set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
			aklcd_enable_alert(true);

			aklcd_set_rgb_panel(akfb->rgb_panel, &(akfb->pdev->dev));
			aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
			
			akfb->panel_refreshing = true;
            
			break;

        case PAL:
		case NTSC:	//turn on tv out.
			if(tvout_mode == akfb->tvout_mode)
				break;

			akfb->tvout_mode = tvout_mode;
			akfb->if_type = DISP_IF_TVOUT;
			akfb->panel_refreshing = false;

			//tvout: stop refresh.
			aklcd_stop_refresh(akfb->if_type, dev);

            aklcd_close_osd_channel();
            aklcd_close_ov1_channel();
            aklcd_close_ov2_channel();
            aklcd_close_rgb_channel();	  
            
			//tvout: change screen.
			akfb->screen.width = (tvout_mode==PAL?TV_PAL_WIDTH:TV_NTSC_WIDTH);
			akfb->screen.height = (tvout_mode==PAL?TV_PAL_HEIGHT:TV_NTSC_HEIGHT);
			akfb->screen.bg_color = AKYUV(16,128,128);//black
			aklcd_set_screen (&(akfb->screen));

			//tvout: change overlay1
			aklcd_get_tvout_dst_params(&akfb->ov1ch_new,akfb->tvout_mode);
		    aklcd_set_ov1_channel(&akfb->ov1ch_new);
		    aklcd_open_ov1_channel();
			set_bit(AKLCD_CHANNEL_OV1, &(akfb->par_change_bits));
			aklcd_enable_alert(true);

            //tvout: set tvout interface.
			aklcd_set_tvout_panel(akfb->tvout_mode);
			aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
			break;

		default:
			ret = -EINVAL;
			break;
		} /* switch (tvout_mode) */
		printk("--FBIOPUT_AKTVOUT\n");
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct fb_ops akfb_ops = {
	.owner		= THIS_MODULE,
	/* needed by fb_set_cmap() */
	.fb_setcolreg	= akfb_setcolreg,
	/* needed by FBIOPUT_VSCREENINFO */
	.fb_check_var	= akfb_check_var,
	.fb_set_par	= akfb_set_par,
	.fb_pan_display	= akfb_pan_display,
	/* need by FBIOBLANK */
	.fb_blank	= akfb_blank,
	/* needed by fbcon */
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	/* extended ioctl */
	.fb_ioctl	= akfb_ioctl,
};

#define freq_to_akfb(_n) container_of(_n, struct akfb, freq_transition)

#ifdef CONFIG_CPU_FREQ
static void get_cpufreq_need_info(struct cpufreq_freqs *freqs,
	int *prechange, int *postchange)
{
	int close_lcd_display = 0;
	int open_lcd_display = 0;
	
	// new mode is low mode, else new mode is nomal mode
	if (freqs->new_cpufreq.low_clock) {
		if (!freqs->old_cpufreq.low_clock) {
			close_lcd_display = 1;
			open_lcd_display = 0;
		}else {
			close_lcd_display = 0;
			open_lcd_display = 0;
		}
	} else if (!freqs->new_cpufreq.low_clock) {
		if (!freqs->old_cpufreq.low_clock) {
			if (freqs->old_cpufreq.pll_sel == freqs->new_cpufreq.pll_sel) {
				if (freqs->old_cpufreq.asic_clk != freqs->new_cpufreq.asic_clk) {
					close_lcd_display = 1;
					open_lcd_display = 1;
				} else {
					close_lcd_display = 0;
					open_lcd_display = 0;
				}
			} else {
				close_lcd_display = 1;
            	open_lcd_display = 1;
			}
		} else if (freqs->old_cpufreq.low_clock){
			close_lcd_display = 0;
			open_lcd_display = 1;
		}
	}

	*prechange = close_lcd_display;
	*postchange = open_lcd_display;
}
static int ak98fb_cpufreq_transition(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct akfb *akfb = freq_to_akfb(nb);
	struct cpufreq_freqs *freqs = data;	
	struct device *dev = &(akfb->pdev->dev);
	int lcd_close_display = 0;
	int lcd_open_display = 0;

	get_cpufreq_need_info(freqs, &lcd_close_display, &lcd_open_display);
		
	if (val == CPUFREQ_PRECHANGE)
	{
		if(lcd_close_display) {
			struct fb_event event;
			printk("AK98FB: CPUFREQ PRE-Change\n");
			
			acquire_console_sem();
			event.info = akfb->fb;
			fb_notifier_call_chain(FB_EVENT_PRE_CPUFREQ, &event);
			
			akfb->bOv1ChnlOpened = aklcd_get_ov1_enable();
			akfb->bOv2ChnlOpened = aklcd_get_ov2_enable();
			akfb->bOsdChnlOpened = aklcd_get_osd_enable();
			
			aklcd_close_rgb_channel();
			aklcd_close_ov1_channel();
			aklcd_close_ov2_channel();
			aklcd_close_osd_channel();

			if (akfb->panel_refreshing) {
				init_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
				set_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
				wait_for_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
			}
				
			if (freqs->new_cpufreq.low_clock) {
				akfb->panel_refreshing = false;
			}
			release_console_sem();
		}
	}
	else if (val == CPUFREQ_POSTCHANGE)
	{
		if (lcd_open_display) {
			struct fb_event event;
			printk("AK98FB: CPUFREQ POST-Change\n");

			acquire_console_sem();
			event.info = akfb->fb;
			fb_notifier_call_chain(FB_EVENT_POST_CPUFREQ, &event);

			// from previous low mode to current's normal mode
			if (freqs->old_cpufreq.low_clock) {
				aklcd_enable();
				aklcd_set_screen(&(akfb->screen));
				aklcd_set_rgb_irq();
				aklcd_set_rgb_panel(akfb->rgb_panel, dev);
				aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
				aklcd_set_power(true);
				akfb->panel_refreshing = true;
			}
			aklcd_set_rgb_channel(&(akfb->rgbch));
			aklcd_open_rgb_channel();

			if(akfb->bOv1ChnlOpened == true)
			{
				aklcd_set_ov1_channel(&(akfb->ov1ch));
				aklcd_open_ov1_channel();
			}
			if(akfb->bOv2ChnlOpened == true)
			{
				aklcd_set_ov2_channel(&(akfb->ov2ch));
				aklcd_open_ov2_channel();
			}
			if(akfb->bOsdChnlOpened == true)
			{
				aklcd_set_osd_channel(&(akfb->osdch));
				aklcd_open_osd_channel();
			}

			if (akfb->panel_refreshing) {
				init_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
				set_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits));
				aklcd_enable_alert(true);
				wait_for_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
			}

			release_console_sem();
		}
	}
	
	return 0;

}

static inline int ak98fb_cpufreq_register(struct akfb *info)
{
	info->freq_transition.notifier_call = ak98fb_cpufreq_transition;

	return cpufreq_register_notifier(&info->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void ak98fb_cpufreq_deregister(struct akfb *info)
{
	cpufreq_unregister_notifier(&info->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else
static inline int ak98fb_cpufreq_register(struct akfb *info)
{
	return 0;
}

static inline void ak98fb_cpufreq_deregister(struct akfb *info)
{
}
#endif


/********************
 * driver entry
 ********************/
extern const unsigned int ak_logo_width;
extern const unsigned int ak_logo_height;
extern const unsigned short ak_logo[]; /* rgb565 480x272 */

static int __devinit ak98fb_probe(struct platform_device *pdev)
{
	struct akfb                  *akfb;
	struct fb_info               *fb;
	struct aklcd_rgb_panel       *rgb_panel;
	struct aklcd_screen          *screen;
	struct aklcd_rgb_channel     *rgbch;
	struct aklcd_channel_meminfo *rgbch_meminfo;
	struct aklcd_overlay_channel *ov1ch;
	struct aklcd_channel_meminfo *ov1ch_meminfo;
	struct aklcd_overlay_channel *ov2ch;
	struct aklcd_channel_meminfo *ov2ch_meminfo;
	struct aklcd_osd_channel     *osdch;
	struct aklcd_channel_meminfo *osdch_meminfo;

	unsigned int     width, height, rgbch_bpp;
	struct device    *dev = &pdev->dev;
	int              i, irq, ret;

	fb = framebuffer_alloc(sizeof(struct akfb), dev);
	if (fb == NULL) {
		dev_err(dev, "alloc framebuffer error\n");
		return -ENOMEM;
	}

	akfb = fb->par;
	akfb->fb = fb;

	platform_set_drvdata(pdev, akfb);
	akfb->pdev = pdev;

	for (i = 0; i < AKLCD_CHANNEL_MAX; i++) {
		mutex_init(&(akfb->aklcd_par_lock[i]));
		clear_bit(i, &(akfb->par_change_bits));
	}

	akfb->if_type   = DISP_IF_RGB;
	akfb->rgb_panel = &bsp_rgb_panel;
	rgb_panel       = akfb->rgb_panel;
	screen          = &(akfb->screen);
	rgbch           = &(akfb->rgbch);
	ov1ch           = &(akfb->ov1ch);
	ov2ch           = &(akfb->ov2ch);
	osdch           = &(akfb->osdch);
	rgbch_meminfo   = &(akfb->rgbch_meminfo);
	ov1ch_meminfo   = &(akfb->ov1ch_meminfo);
	ov2ch_meminfo   = &(akfb->ov2ch_meminfo);
	osdch_meminfo   = &(akfb->osdch_meminfo);

	aklcd_enable();

	/* set screen */
	screen->width    = rgb_panel->thd;
	screen->height   = rgb_panel->tvd;
	screen->bg_color = AKRGB(0, 0, 0);

	aklcd_set_screen(screen);

	/* open rgb channel */
	rgbch->format       = AKLCD_RGB565;
	rgbch->use_vpage    = true; /* alway enable virtual page */
	rgbch->vpage_width  = screen->width;
	rgbch->vpage_height = screen->height * 2;
	rgbch->virt_left    = 0;
	rgbch->virt_top     = 0;
	rgbch->disp_left    = 0;
	rgbch->disp_top     = 0;
	rgbch->width        = screen->width;
	rgbch->height       = screen->height;

	if (rgbch->format == AKLCD_RGB565 || rgbch->format == AKLCD_BGR565)
		rgbch_bpp   = 2;
	else
		rgbch_bpp   = 3;

	if (rgbch->use_vpage) {
		width = rgbch->vpage_width;
		height = rgbch->vpage_height;
	} else {
		width = rgbch->width;
		height = rgbch->height;
	}

	rgbch_meminfo->size = width * height * rgbch_bpp;
	dev_info(dev, "alloc framebuffer %i bytes\n", rgbch_meminfo->size);

	rgbch_meminfo->vaddr = dma_alloc_writecombine(dev, rgbch_meminfo->size, &rgbch_meminfo->paddr, GFP_KERNEL);
	if (rgbch_meminfo->vaddr == NULL) {
		dev_err(dev, "alloc fb memory error\n");
		ret = -ENOMEM;
		goto _probe_err1;
	}
	rgbch->data_addr   = rgbch_meminfo->paddr;
	dev_info(dev, "framebuffer address: 0x%08x\n", rgbch_meminfo->paddr);

	{
		int l, left, top;
		top = (screen->height - ak_logo_height)/2;
		left = (screen->width - ak_logo_width)/2;
		printk ("width =%i, height = %i, width =%i, height =%i, top=%i,left=%i\n",
			screen->width, screen->height, ak_logo_width, ak_logo_height, top, left);
		for (l = 0; l < ak_logo_height; l++)
			memcpy (rgbch_meminfo->vaddr+rgbch_bpp*(screen->width*(top+l) + left),
				(void *)ak_logo+rgbch_bpp*(ak_logo_width*l), ak_logo_width*rgbch_bpp);
	}

	aklcd_set_rgb_channel(rgbch);
	aklcd_open_rgb_channel();

	/* set default irq mask */
	aklcd_set_rgb_irq();

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no IRQ defined\n");
		ret = -ENODEV;
		goto _probe_err1;
	}
	ret = request_irq(irq, akfb_irq_handler, IRQF_DISABLED, "AK LCDC", akfb);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		ret = -EBUSY;
		goto _probe_err1;
	}


	/* framebuffer */
	/* set fix info */
	strncpy(fb->fix.id, "ak-lcd", sizeof(fb->fix.id));
	fb->fix.type      = FB_TYPE_PACKED_PIXELS;
	fb->fix.type_aux  = 0; /* ? */
	fb->fix.visual    = FB_VISUAL_TRUECOLOR;
	fb->fix.xpanstep  = 1;
	fb->fix.ypanstep  = 1;
	fb->fix.ywrapstep = 0;
	fb->fix.accel     = FB_ACCEL_NONE;

	fb->fix.smem_start = rgbch_meminfo->paddr;
	fb->fix.smem_len   = rgbch_meminfo->size;
	if (rgbch->use_vpage)
		fb->fix.line_length = rgbch->vpage_width * rgbch_bpp;
	else
		fb->fix.line_length = rgbch->width * rgbch_bpp;
	fb->fix.mmio_start = 0;
	fb->fix.mmio_len   = 0;

	/* set var info */
	fb->var.xres         = rgbch->width;
	fb->var.yres         = rgbch->height;
	fb->var.xres_virtual = rgbch->vpage_width;
	fb->var.yres_virtual = rgbch->vpage_height;
	fb->var.xoffset      = 0;
	fb->var.yoffset      = 0;
	fb->var.grayscale    = 0;
	memset(&(fb->var.transp), 0, sizeof(struct fb_bitfield));
	switch (rgbch->format) {
	case AKLCD_RGB565:
		fb->var.bits_per_pixel = 16;
		memcpy(&(fb->var.red), &rgb565_r, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.green), &rgb565_g, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.blue), &rgb565_b, sizeof(struct fb_bitfield));
		break;
	case AKLCD_BGR565:
		fb->var.bits_per_pixel = 16;
		memcpy(&(fb->var.red), &bgr565_r, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.green), &bgr565_g, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.blue), &bgr565_b, sizeof(struct fb_bitfield));
		break;
	case AKLCD_RGB888:
		fb->var.bits_per_pixel = 24;
		memcpy(&(fb->var.red), &rgb888_r, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.green), &rgb888_g, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.blue), &rgb888_b, sizeof(struct fb_bitfield));
		break;
	case AKLCD_BGR888:
		fb->var.bits_per_pixel = 24;
		memcpy(&(fb->var.red), &bgr888_r, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.green), &bgr888_g, sizeof(struct fb_bitfield));
		memcpy(&(fb->var.blue), &bgr888_b, sizeof(struct fb_bitfield));
		break;
	}
	fb->var.nonstd = 0;
	fb->var.activate = FB_ACTIVATE_NOW;
	fb->var.height = -1;
	fb->var.width = -1;
	fb->var.pixclock = rgb_panel->pclk_freq;
	fb->var.left_margin = rgb_panel->thbp;
	fb->var.right_margin = rgb_panel->thfp;
	fb->var.upper_margin = rgb_panel->tvbp;
	fb->var.lower_margin = rgb_panel->tvfp;
	fb->var.hsync_len = rgb_panel->thpw;
	fb->var.vsync_len = rgb_panel->tvpw;
	fb->var.sync = 0;	/* ? */
	fb->var.vmode = FB_VMODE_NONINTERLACED;
	fb->var.rotate = 0;

	fb->node = -1;
	fb->fbops = &akfb_ops;
	fb->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK | FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	fb->screen_base = rgbch_meminfo->vaddr;
	fb->screen_size = rgbch_meminfo->size;

	/* used by .fb_fillrect and .fb_imgblt */
	fb->pseudo_palette = akfb->pseudo_palette;

	/* needed by FBIOGETCMAP & FBIOPUTCMAP */
	ret = fb_alloc_cmap(&fb->cmap, 256, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to alloc cmap: %d\n", ret);
		goto _probe_err2;
	}

	dev_info(dev, "register frame buffer device: %s\n", fb->fix.id);
	ret = register_framebuffer(fb);
	if (ret < 0) {
		dev_err(dev, "Failed to register framebuffer device: %d\n", ret);
		goto _probe_err3;
	}
	dev_info(dev, "fb%d: %s frame buffer device\n", fb->node, fb->fix.id);

	/* set display interface */
	aklcd_set_rgb_panel(rgb_panel, dev);

	aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);
	aklcd_set_power(true);
	akfb->panel_refreshing = true;
	akfb->bOv1ChnlOpened = false;
	akfb->bOv2ChnlOpened = false;
	akfb->bOsdChnlOpened = false;
	akfb->tvout_mode = TVOUT_OFF;

	ret = ak98fb_cpufreq_register(akfb);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register cpufreq\n");
	}

	return 0;

_probe_err3:
	if (fb->cmap.len)
		fb_dealloc_cmap(&fb->cmap);

_probe_err2:
	free_irq(irq, akfb);

_probe_err1:
	aklcd_disable();

	if (rgbch_meminfo->vaddr)
		dma_free_writecombine(dev, rgbch_meminfo->size, rgbch_meminfo->vaddr, rgbch_meminfo->paddr);
	if (ov1ch_meminfo->vaddr)
		dma_free_writecombine(dev, ov1ch_meminfo->size, ov1ch_meminfo->vaddr, ov1ch_meminfo->paddr);
	if (ov2ch_meminfo->vaddr)
		dma_free_writecombine(dev, ov2ch_meminfo->size, ov2ch_meminfo->vaddr, ov2ch_meminfo->paddr);
	if (osdch_meminfo->vaddr)
		dma_free_writecombine(dev, osdch_meminfo->size, osdch_meminfo->vaddr, osdch_meminfo->paddr);

	framebuffer_release(fb);

	return ret;
}

static int __devexit ak98fb_remove(struct platform_device *pdev)
{
	struct akfb    *akfb;
	struct fb_info *fb;
	struct device  *dev = &pdev->dev;
	struct aklcd_channel_meminfo *rgbch_meminfo;
	struct aklcd_channel_meminfo *ov1ch_meminfo;
	struct aklcd_channel_meminfo *ov2ch_meminfo;
	struct aklcd_channel_meminfo *osdch_meminfo;
	int    irq;

	akfb = platform_get_drvdata(pdev);
	fb = akfb->fb;

	rgbch_meminfo = &(akfb->rgbch_meminfo);
	ov1ch_meminfo = &(akfb->ov1ch_meminfo);
	ov2ch_meminfo = &(akfb->ov2ch_meminfo);
	osdch_meminfo = &(akfb->osdch_meminfo);

	unregister_framebuffer(fb);
	if (fb->cmap.len)
		fb_dealloc_cmap(&fb->cmap);

	ak98fb_cpufreq_deregister(akfb);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, akfb);

	aklcd_set_power(false);
	aklcd_disable();

	if (rgbch_meminfo->vaddr)
		dma_free_writecombine(dev, rgbch_meminfo->size, rgbch_meminfo->vaddr, rgbch_meminfo->paddr);
	if (ov1ch_meminfo->vaddr)
		dma_free_writecombine(dev, ov1ch_meminfo->size, ov1ch_meminfo->vaddr, ov1ch_meminfo->paddr);
	if (ov2ch_meminfo->vaddr)
		dma_free_writecombine(dev, ov2ch_meminfo->size, ov2ch_meminfo->vaddr, ov2ch_meminfo->paddr);
	if (osdch_meminfo->vaddr)
		dma_free_writecombine(dev, osdch_meminfo->size, osdch_meminfo->vaddr, osdch_meminfo->paddr);

	framebuffer_release(fb);

	return 0;
}

#ifdef CONFIG_PM
static int ak98fb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct akfb  *akfb = platform_get_drvdata(pdev);
	struct device   *dev = &(akfb->pdev->dev);

	if (current_mode_is_low_mode()) 
		return 0;
	
	acquire_console_sem();
	akfb->bOv1ChnlOpened = aklcd_get_ov1_enable();
	akfb->bOv2ChnlOpened = aklcd_get_ov2_enable();
	akfb->bOsdChnlOpened = aklcd_get_osd_enable();
	aklcd_close_rgb_channel();
	aklcd_close_ov1_channel();
	aklcd_close_ov2_channel();
	aklcd_close_osd_channel();
	aklcd_stop_refresh(akfb->if_type, dev);
	aklcd_disable();
	aklcd_set_power(false);
	akfb->panel_refreshing = false;
	release_console_sem();
	
	return 0;
}

static int ak98fb_resume(struct platform_device *pdev)
{
	struct akfb   *akfb = platform_get_drvdata(pdev);	
	struct device   *dev = &(akfb->pdev->dev);

	if (current_mode_is_low_mode())
		return 0;
	
	acquire_console_sem();
	aklcd_enable();
	aklcd_set_screen(&(akfb->screen));
	aklcd_set_rgb_channel(&(akfb->rgbch));
	aklcd_open_rgb_channel();
	aklcd_set_rgb_irq();
	aklcd_set_rgb_panel(akfb->rgb_panel, dev);
	aklcd_start_refresh(akfb->if_type, akfb->rgb_panel->pclk_freq, dev);

	aklcd_set_power(true);
	akfb->panel_refreshing = true;

	if(akfb->bOv1ChnlOpened == true)
	{
		aklcd_set_ov1_channel(&(akfb->ov1ch));
		aklcd_open_ov1_channel();
	}
	if(akfb->bOv2ChnlOpened == true)
	{
		aklcd_set_ov2_channel(&(akfb->ov2ch));
		aklcd_open_ov2_channel();
	}
	if(akfb->bOsdChnlOpened == true)
	{
		aklcd_set_osd_channel(&(akfb->osdch));
		aklcd_open_osd_channel();
	}

	if (akfb->panel_refreshing) {
		init_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
		set_bit(AKLCD_CHANNEL_RGB, &(akfb->par_change_bits));
		aklcd_enable_alert(true);
		wait_for_completion(&(akfb->aklcd_setpar_comp[AKLCD_CHANNEL_RGB]));
	}
	release_console_sem();
	
	return 0;
}
#else
#define ak98fb_suspend	NULL
#define ak98fb_resume	NULL
#endif

static struct platform_driver ak98fb_driver = {
	.probe		= ak98fb_probe,
	.remove 	= __devexit_p(ak98fb_remove),
	.suspend	= ak98fb_suspend,
	.resume		= ak98fb_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ak98-lcd",
	},
};

static int __init ak98fb_init(void)
{
	pr_debug("+%s\n", __func__);
	return platform_driver_register(&ak98fb_driver);
}

static void __exit ak98fb_exit(void)
{
	pr_debug("+%s\n", __func__);
	platform_driver_unregister(&ak98fb_driver);
}

module_init(ak98fb_init);
module_exit(ak98fb_exit);

MODULE_DESCRIPTION("loadable framebuffer driver for AK98");
MODULE_LICENSE("GPL");
