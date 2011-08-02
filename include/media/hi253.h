/* hi253 Camera
 *
 * Copyright (C) 2008 Renesas Solutions Corp.
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HI253_H__
#define __HI253_H__

#include <media/soc_camera.h>

/* for flags */
#define HI253_FLAG_VFLIP     0x00000001 /* Vertical flip image */
#define HI253_FLAG_HFLIP     0x00000002 /* Horizontal flip image */

/*
 * for Edge ctrl
 *
 * strength also control Auto or Manual Edge Control Mode
 * see also HI253_MANUAL_EDGE_CTRL
 */
struct hi253_edge_ctrl {
	unsigned char strength;
	unsigned char threshold;
	unsigned char upper;
	unsigned char lower;
};

/*
 * hi253 camera info
 */
struct hi253_camera_info {
	unsigned long		   buswidth;
	unsigned long		   flags;
	struct soc_camera_link link;
	struct hi253_edge_ctrl edgectrl;
};


#define HI253_MANUAL_EDGE_CTRL	0x80 /* un-used bit of strength */
#define EDGE_STRENGTH_MASK	0x1F
#define EDGE_THRESHOLD_MASK	0x0F
#define EDGE_UPPER_MASK		0xFF
#define EDGE_LOWER_MASK		0xFF

#define HI253_AUTO_EDGECTRL(u, l)	\
{					\
	.upper = (u & EDGE_UPPER_MASK),	\
	.lower = (l & EDGE_LOWER_MASK),	\
}

#define HI253_MANUAL_EDGECTRL(s, t)					\
{									\
	.strength  = (s & EDGE_STRENGTH_MASK) | hi253_MANUAL_EDGE_CTRL,\
	.threshold = (t & EDGE_THRESHOLD_MASK),				\
}

/*
** initialize parameter 
*/

#define DELAY_FLAG        0xFE   // first parameter is 0xfe, then 2nd parameter is delay time count
#define END_FLAG          0xFF   // first parameter is 0xff, then parameter table is over 

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

extern u32 HI253_pv_HI253_exposure_lines;

extern u32 HI253_cp_HI253_exposure_lines;

extern const struct regval_list hi253_sensor_yuvinit[];

/**---------------------------------------------------------------------------*
 ** 						Local Variables 								 *
 **---------------------------------------------------------------------------*/

/*lint -save -e533 */
extern const struct regval_list hi253_sensor_yuv640X480[];

extern const struct regval_list hi253_sensor_yuv352X288[];
	
extern const struct regval_list hi253_sensor_yuv800X600[];

extern const struct regval_list hi253_sensor_yuv1600X1200[];


/******************************************************************************/
// Description: set brightness 
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/
extern const struct regval_list HI253_brightness_tab[][7];
extern const struct regval_list HI253_YUV_640X480[];

#endif /* END __HI253_H__ */

