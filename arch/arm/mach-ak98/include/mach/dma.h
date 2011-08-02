/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __AK88_DMA_H
#define __AK88_DMA_H __FILE__

//#include <linux/sysdev.h>
#include <mach/hardware.h>

#define AK98_DMA_CHANNELS  11

/*
 * This is the maximum DMA address(physical address) that can be DMAd to.
 *
 */
//#define MAX_DMA_ADDRESS               0x40000000
//#define MAX_DMA_TRANSFER_SIZE   0x100000 /* Data Unit is half word  */

struct ak880x_dma_channel {
	const char *name;
	void (*irq_handler) (int, void *);
	void *data;
	unsigned char dma_id;
	unsigned char irq_bit;
};

/*
 * 0: camera: 2
 * 1: display: 1
 * 2: audio processor:5
 * 4: motion Estimation /H.264 decoder: 3&6
 * 5: image processor/ MPEG4/H.263 codec: 4
 * 6: l2:10
 * 7: 2d graphics accelerator: 20
 */

#endif				/* __AK88_DMA_H */
