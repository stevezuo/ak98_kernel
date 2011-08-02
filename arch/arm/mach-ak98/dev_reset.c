/*
 * dev_reset.c - device reset routines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/dev_reset.h>
#include <mach/map.h>

static DEFINE_SPINLOCK(sys_reset_ctrl_lock);

/* The device reset control register */
static unsigned long reset_reg[] = {
	/* RESET_CTRL_1 */ 		(unsigned long)AK98_VA_SYSCTRL + 0x000c,	/* ->0x0800000c */ 
	/* RESET_CTRL_2 */ 		(unsigned long)AK98_VA_SYSCTRL + 0x0010,	/* ->0x08000010 */
};

static struct device_reset_ctrl reset_ctrl[] = {
	/* Device */				/* Register index */	/* bit */	/* delay */
	/* DEV_USB_OTG			*/		{RESET_CTRL_1,			31, 		0},
	/* DEV_NAND 			*/		{RESET_CTRL_1,			30, 		0},
	/* DEV_SPI1 			*/		{RESET_CTRL_1,			29, 		0},
	/* DEV_UART1			*/		{RESET_CTRL_1,			28, 		0},
	/* DEV_LCD,				*/		{RESET_CTRL_1,			27, 		0},
	/* DEV_RAM,				*/		{RESET_CTRL_1,			26, 		0},
	/* DEV_CAMERA,			*/		{RESET_CTRL_1,			24, 		0},
	/* DEV_H264,			*/		{RESET_CTRL_1,			23, 		0},
	/* DEV_ROTATION,		*/		{RESET_CTRL_1,			21, 		0},
	/* DEV_L2FIFO,			*/		{RESET_CTRL_1,			19, 		0},
	/* DEV_PCM,				*/		{RESET_CTRL_1,			18, 		0},
	/* DEV_DAC,				*/		{RESET_CTRL_1,			17, 		0},
	/* DEV_ADC2,			*/		{RESET_CTRL_1,			16, 		0},
	/* DEV_VIDEO,			*/		{RESET_CTRL_2,			31, 		0},
	/* DEV_MAC,				*/		{RESET_CTRL_2,			30, 		0},
	/* DEV_RMVB,			*/		{RESET_CTRL_2,			29, 		0},
	/* DEV_HUFFMAN,			*/		{RESET_CTRL_2,			28, 		0},
	/* DEV_MOTION_ESTIMATION*/		{RESET_CTRL_2,			27, 		0},
	/* DEV_H263_MPEG4,		*/		{RESET_CTRL_2,			26, 		0},
	/* DEV_JPEG,			*/		{RESET_CTRL_2,			25, 		0},
	/* DEV_MPEG2_ASP,		*/		{RESET_CTRL_2,			24, 		0},
	/* DEV_SPI2,			*/		{RESET_CTRL_2,			23, 		0},
	/* DEV_UART4,			*/		{RESET_CTRL_2,			22, 		0},
	/* DEV_UART3_I2C,		*/		{RESET_CTRL_2,			21, 		0},
	/* DEV_UART2,			*/		{RESET_CTRL_2,			20, 		0},
	/* DEV_SDIO,			*/		{RESET_CTRL_2,			19, 		0},
	/* DEV_MMC_SD,			*/		{RESET_CTRL_2,			18, 		0},
	/* DEV_USB_FS,			*/		{RESET_CTRL_2,			17, 		0},
	/* DEV_2D,				*/		{RESET_CTRL_2,			16, 		0},
};


/*
 * @brief:	AK98XX device controller reset routine
 * @author:	Zhongjunchao
 * @date:	2011-07-13
 * @param:	[in]dev_no the device selected to be reset, refrence dev_reset.h
 * @note:	This routine is created in order to solve the problem of concurrency 
 * 			and crace onditions of device driver operate Clock Control and 
 * 			Soft Reset Control Register
 * @note:	This routine depends on that corresponding registers are mapped, 
 * 			currently this is done in ak98_map_io(), so be careful about 
 * 			ak98_map_io() changes.
 * @sample:	device_controller_reset(DEV_NAND)
 *
 */
void device_controller_reset(int dev_no)
{
	unsigned long flags;
	unsigned long val;
	unsigned long reg_addr;
	struct device_reset_ctrl *reset_device;

	BUG_ON(dev_no < 0 || dev_no >= DEV_MAX);

	spin_lock_irqsave(&sys_reset_ctrl_lock, flags);

	reset_device = reset_ctrl + dev_no;
	reg_addr = reset_reg[reset_device->regdix];

	val = __raw_readl(reg_addr);
	val |= (1 << reset_device->ctrlbit);
	__raw_writel(val, reg_addr);
	udelay(reset_device->delay);
	val &= ~(1 << reset_device->ctrlbit);
	__raw_writel(val, reg_addr);

	spin_unlock_irqrestore(&sys_reset_ctrl_lock, flags);
}
EXPORT_SYMBOL(device_controller_reset);
