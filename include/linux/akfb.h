#ifndef __LINUX_AKFB_H__
#define __LINUX_AKFB_H__

#ifndef __KERNEL__
#include <linux/ioctl.h>
typedef unsigned long dma_addr_t;
typedef unsigned char bool;
#define true  1
#define false 0

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

struct aklcd_overlay_channel {
	enum aklcd_yuv_range src_range; /* only apply to overlay1 */
	dma_addr_t           y_addr;
	dma_addr_t           u_addr;
	dma_addr_t           v_addr;
	unsigned int         src_width;
	unsigned int         src_height;

	bool                 use_vpage;	  /* only apply to overlay1 */
	unsigned int         vpage_width; /* width of ov1 rectangle */
	unsigned int         vpage_height; /* height of ov1 rectangle */
	unsigned int         virt_left;	/* src rect's left on ov1 */
	unsigned int         virt_top; /* src rect's top on ov1 */

	unsigned int         disp_left;
	unsigned int         disp_top;
	unsigned int         dst_width;
	unsigned int         dst_height;

	enum aklcd_ov_alpha  alpha; /* 0...7, 16, only apply to overlay2 */
};

typedef unsigned short int aklcd_osd_color;	/* rgb565 */

struct aklcd_osd_channel {
	dma_addr_t           data_addr;

	aklcd_osd_color      palette[16]; /* palette[0] is for transparency */

	unsigned int         disp_left;
	unsigned int         disp_top;
	unsigned int         width;
	unsigned int         height;

	enum aklcd_osd_alpha alpha; /* 0...8 */
};
#endif

struct aklcd_overlay_info {
	unsigned int         overlay_id; /* 0 for overlay1, 1 for overlay2 */
	bool                 enable;
	struct aklcd_overlay_channel overlay_setting;
};

struct aklcd_osd_info {
	bool                 enable;
	struct aklcd_osd_channel osd_setting;
};

#define FBIOPUT_AKOVINFO          _IOW('F', 0x80, struct aklcd_overlay_info)
#define FBIOGET_AKOVINFO          _IOWR('F', 0x81, struct aklcd_overlay_info)
#define FBIOPUT_AKOSDINFO         _IOW('F', 0x82, struct aklcd_osd_info)
#define FBIOGET_AKOSDINFO         _IOR('F', 0x83, struct aklcd_osd_info)
/* for android overlay hal control interface. enable is valid,
   every member in aklcd_overlay_channel except [yuv]_addr and vpage is valid */
#define FBIOPUT_AKOVPOSI          _IOW('F', 0x84, struct aklcd_overlay_info)
/* for android overlay hal data interface. enable is valid,
   [yuv]_addr in aklcd_overlay_channel is valid */
#define FBIOPUT_AKOVDATA          _IOW('F', 0x85, struct aklcd_overlay_info)
/* for anyka's android overlay hal extension.
   enable == 0 means hide overlay, enable == 1 means show overlay again */
#define FBIOPUT_AKOVSHOWING       _IOW('F', 0x86, struct aklcd_overlay_info)

#define FBIOPUT_AKTVOUT       _IOW('F', 0x87, enum ak_tvout_mode)

#endif
