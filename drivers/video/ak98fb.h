#ifndef __AK98FB_H__
#define __AK98FB_H__

#include <linux/fb.h>

#include <asm/io.h>

#include <mach/regs-lcd.h>

/* each macro defined below can be used, I prefer the latter */
#if 0

#define clear_low_bits(val, len)        (((len)==sizeof(val)*8)?0:((val) >> (len) << (len)))
#define mask(msb, lsb)                  clear_low_bits(~clear_low_bits(~0UL, (msb+1)), (lsb))

#else

#define clear_low_bits(val, len)        ((val) >> (len) << (len))
#define clear_high_bits(val, len)       ((val) << (len) >> (len))
#define mask(msb, lsb)                  clear_high_bits(clear_low_bits(~0UL, (lsb)), (sizeof(~0UL)*8 - 1 - (msb)))

#endif

#if 0

#define lcd_set_reg(val, reg, msb, lsb) __raw_writel((__raw_readl(reg) & ~mask((msb), (lsb))) | (((val) & mask((msb)-(lsb), 0)) << (lsb)), \
						     (reg))

#else

#define lcd_set_reg(val, reg, msb, lsb) __raw_writel((__raw_readl(reg) & ~mask((msb), (lsb))) | (((val) << (lsb)) & mask((msb), (lsb))), \
						     (reg))

#endif

#define lcd_get_reg(reg, msb, lsb) ((__raw_readl(reg) & mask(msb, lsb)) >> lsb)

/* tvout parameter */
#define TV_XPAD 8
#define TV_YPAD 4

#define TV_NTSC_WIDTH 720
#define TV_NTSC_HEIGHT 240
#define TV_NTSC_DISP_WIDTH (TV_NTSC_WIDTH - TV_XPAD*2)
#define TV_NTSC_DISP_HEIGHT (TV_NTSC_HEIGHT - TV_YPAD*2)
#define TV_NTSC_DISP_LEFT  TV_XPAD
#define TV_NTSC_DISP_TOP   TV_YPAD

#define TV_PAL_WIDTH 720
#define TV_PAL_HEIGHT 288
#define TV_PAL_DISP_WIDTH (TV_PAL_WIDTH - TV_XPAD*2)
#define TV_PAL_DISP_HEIGHT (TV_PAL_HEIGHT - TV_YPAD*2)
#define TV_PAL_DISP_LEFT  TV_XPAD
#define TV_PAL_DISP_TOP   TV_YPAD

enum aklcd_if {
	DISP_IF_MPU   = 0b01,
	DISP_IF_RGB   = 0b10,
	DISP_IF_TVOUT = 0b11
};

enum aklcd_panel_mode {
	PANEL_INTERLEAVED = 0b0,
	PANEL_PROGRESS    = 0b1
};

enum aklcd_data_width {
	RGB8BITS  = 0b00,
	RGB24BITS = 0b11
};

enum aklcd_data_seq {
	SEQ_RGB = 0b0,
	SEQ_BGR = 0b1
};

enum aklcd_tvunit {
	TV_UNIT_TH   = 0b0,
	TV_UNIT_PCLK = 0b1,
};

enum aklcd_signal_pol {
	POL_NEGATIVE,
	POL_POSITIVE
};

enum aklcd_cfmt {
	AKLCD_RGB888 = 0b11,
	AKLCD_BGR888 = 0b10,
	AKLCD_RGB565 = 0b01,
	AKLCD_BGR565 = 0b00
};

enum aklcd_yuv_range {
	YUV_SHORT_RANGE = 0b0,
	YUV_FULL_RANGE  = 0b1
};

enum aklcd_ov_alpha {
	OV_TRANS_100 = 0x0,
	OV_TRANS_87  = 0x1,
	OV_TRANS_75  = 0x2,
	OV_TRANS_62  = 0x3,
	OV_TRANS_50  = 0x4,
	OV_TRANS_37  = 0x5,
	OV_TRANS_25  = 0x6,
	OV_TRANS_12  = 0x7,
	OV_TRANS_0   = 0xf
};

enum aklcd_osd_alpha {
	OSD_TRANS_100 = 0x0,
	OSD_TRANS_87  = 0x1,
	OSD_TRANS_75  = 0x2,
	OSD_TRANS_62  = 0x3,
	OSD_TRANS_50  = 0x4,
	OSD_TRANS_37  = 0x5,
	OSD_TRANS_25  = 0x6,
	OSD_TRANS_12  = 0x7,
	OSD_TRANS_0   = 0x8
};

enum ak_tvout_mode{
	TVOUT_OFF,
	PAL,
	NTSC
};

#define AKRGB(r, g, b) ((((r)&0xff) << 16) | (((g)&0xff) << 8)| ((b)&0xff))
#define AKYUV(y, u, v) ((((y)&0xff) << 16) | (((u)&0xff) << 8)| ((v)&0xff))

typedef int ak_color;

#define AKLCD_HSCALER(w) (65536/((w)-1))
#define AKLCD_VSCALER(h) (65536/(2*((h)-1)))

struct aklcd_rgb_panel {
	const char *name;

	enum aklcd_panel_mode panel_mode;
	enum aklcd_data_width data_width;
	enum aklcd_data_seq   data_seq; /* valid when width is 8bits */

	int thpw;
	int thbp;
	int thd;
	int thfp;

	int tvpw;
	int tvbp;
	int tvd;
	int tvfp;
	enum aklcd_tvunit tv_unit;

	int pclk_freq;

	enum aklcd_signal_pol pclk_pol;
	enum aklcd_signal_pol hsync_pol;
	enum aklcd_signal_pol vsync_pol;
	enum aklcd_signal_pol vogate_pol;
};

struct aklcd_screen {
	unsigned int width;
	unsigned int height;
	ak_color     bg_color;
};

struct aklcd_rgb_channel {
	enum aklcd_cfmt format;
	dma_addr_t      data_addr; /* physical address of RGB data */

	bool            use_vpage;
	unsigned int    vpage_width;
	unsigned int    vpage_height;
	unsigned int    virt_left;
	unsigned int    virt_top;

	unsigned int    disp_left;
	unsigned int    disp_top;
	unsigned int    width;
	unsigned int    height;
};

struct aklcd_overlay_channel {
	enum aklcd_yuv_range src_range; /* only apply to overlay1 */
	dma_addr_t           y_addr;
	dma_addr_t           u_addr;
	dma_addr_t           v_addr;
	unsigned int         src_width;
	unsigned int         src_height;

	bool                 use_vpage; /* only apply to overlay1 */
	unsigned int         vpage_width;
	unsigned int         vpage_height;
	unsigned int         virt_left;
	unsigned int         virt_top;

	unsigned int         disp_left;
	unsigned int         disp_top;
	unsigned int         dst_width;
	unsigned int         dst_height;

	enum aklcd_ov_alpha  alpha; /* 0...7, 16, only apply to overlay2 */
};

typedef u16 aklcd_osd_color;	/* rgb565 */

struct aklcd_osd_channel {
	dma_addr_t           data_addr;

	aklcd_osd_color      palette[16]; /* palette[0] is for transparency */

	unsigned int         disp_left;
	unsigned int         disp_top;
	unsigned int         width;
	unsigned int         height;

	enum aklcd_osd_alpha alpha; /* 0...8 */
};

/* for framebuffer */
enum {
	AKLCD_CHANNEL_OSD,
	AKLCD_CHANNEL_OV1,
	AKLCD_CHANNEL_OV2,
	AKLCD_CHANNEL_RGB,
	AKLCD_CHANNEL_MAX
};

struct aklcd_channel_meminfo {
	int        size;
	dma_addr_t paddr;
	void *     vaddr;
};

struct akfb {
	struct platform_device       *pdev;
	struct fb_info               *fb;

	enum aklcd_if                if_type;
	struct aklcd_rgb_panel       *rgb_panel;
	enum ak_tvout_mode           tvout_mode;
	struct aklcd_screen          screen;

	struct aklcd_rgb_channel     rgbch;
	struct aklcd_channel_meminfo rgbch_meminfo;

	struct aklcd_overlay_channel ov1ch;  //params that hardware is working on
	struct aklcd_overlay_channel ov1ch_new;  //params that passed down by ioctl
	struct aklcd_overlay_channel ov1ch_saved; //params that passed down by ioctl when in tvout mode
	struct aklcd_channel_meminfo ov1ch_meminfo;
	bool   bOv1ChnlOpened;  //ov1 channel state when suspended

	struct aklcd_overlay_channel ov2ch;  //params that hardware is working on
	struct aklcd_overlay_channel ov2ch_new; //params that passed down by ioctl
	struct aklcd_channel_meminfo ov2ch_meminfo;
	bool   bOv2ChnlOpened;   //ov2 channel state when suspended

	struct aklcd_osd_channel     osdch;  //params that hardware is working on
	struct aklcd_osd_channel     osdch_new;  //params that passed down by ioctl
	struct aklcd_channel_meminfo osdch_meminfo;
	bool   bOsdChnlOpened;   //osd channel state when suspended

	struct mutex                 aklcd_par_lock[AKLCD_CHANNEL_MAX];
	unsigned long                par_change_bits;
	struct completion            aklcd_setpar_comp[AKLCD_CHANNEL_MAX];

	u32                          pseudo_palette[16];

	bool                         panel_refreshing;

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif
};

#endif /* __AK98FB_H__ */
