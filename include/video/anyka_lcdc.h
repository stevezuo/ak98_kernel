/*
 *  Header file for AK88 LCD Controller
 *
 *  Data structure and register user interface
 *
 *  Copyright (C) 2010 Anyka Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ANYKA_LCDC_H__
#define __ANYKA_LCDC_H__

#include <linux/workqueue.h>

/* Way LCD wires are connected to the chip:
 * Some Anyka chips use BGR color mode (instead of standard RGB)
 * A swapped wiring onboard can bring to RGB mode.
 */

#define ANYKA_LCDC_WIRING_BGR		0
#define ANYKA_LCDC_WIRING_RGB		1
#define ANYKA_LCDC_WIRING_RGB555	2

 /* LCD Controller info data structure, stored in device platform_data */
struct anyka_lcdfb_info {
	spinlock_t lock;
	struct fb_info *info;
	void __iomem *mmio;
	int irq_base;
	struct work_struct task;

	unsigned int guard_time;
	unsigned int smem_len;
	struct platform_device *pdev;
	struct clk *bus_clk;
	struct clk *lcdc_clk;

#ifdef CONFIG_BACKLIGHT_ANYKA_LCDC
	struct backlight_device *backlight;
	u8 bl_power;
#endif
	bool lcdcon_is_backlight;
	u8 saved_lcdcon;

	u8 default_bpp;
	u8 lcd_wiring_mode;
	unsigned int default_lcdcon2;
	unsigned int default_dmacon;
	void (*anyka_lcdfb_power_control) (int on);
	struct fb_monspecs *default_monspecs;
	u32 pseudo_palette[16];
};

#endif				/* __ANYKA_LCDC_H__ */
