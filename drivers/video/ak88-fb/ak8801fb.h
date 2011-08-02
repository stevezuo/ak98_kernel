/*
 */

#ifndef __AK88FB_H_
#define __AK88FB_H_

struct ak880xfb_info {
	struct device		*dev;
	struct clk		*clk;
	
	struct resource		*mem;
	void __iomem		*io;
	void __iomem		*irq_base;
	
	//struct ak7801fb_hw	regs;

	unsigned int		palette_ready;
	
	/* keep these registers in case we need to re-write palette */
	u32			palette_buffer[256];
	u32			pseudo_pal[16];
};

#endif
