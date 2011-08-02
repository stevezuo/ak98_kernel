/*
 * dev_reset.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __DEV_RESET_H
#define __DEV_RESET_H

/**
 * This is the register index of array reset_reg make sure dev_reset.c 
 * reset_ctrl struct mach to it
 */
#define RESET_CTRL_1	0
#define RESET_CTRL_2	1

/**
 * struct device_reset_ctrl - define the reset control info of device
 * @dev_no:		the number of device to be reset
 * @regdix:		the index of reset control register
 * @ctrlbit:	the control bit of the corresponding register	
 * @delay:		the delay time after reset device(us)
 */
struct device_reset_ctrl {
	unsigned long regdix;
	unsigned long ctrlbit;
	unsigned long delay;
};

/**
 * This is the struct device_reset_ctrl`s regdix, make sure correct to dev_reset.c
 * define of reset_ctrl
 */
#define	DEV_USB_OTG				0
#define	DEV_NAND				1
#define	DEV_SPI1				2
#define	DEV_UART1				3
#define DEV_LCD					4
#define DEV_RAM					5
#define DEV_CAMERA				6
#define DEV_H264				7
#define DEV_ROTATION			8
#define DEV_L2FIFO				9
#define DEV_PCM					10
#define DEV_DAC					11
#define DEV_ADC2				12
#define DEV_VIDEO				13
#define DEV_MAC					14
#define DEV_RMVB				15
#define DEV_HUFFMAN				16
#define DEV_MOTION_ESTIMATION	17
#define DEV_H263_MPEG4			18
#define DEV_JPEG				19
#define DEV_MPEG2_ASP			20
#define DEV_SPI2				21
#define DEV_UART4				22
#define DEV_UART3_I2C			23
#define DEV_UART2				24
#define DEV_SDIO				25
#define DEV_MMC_SD				26
#define DEV_USB_FS				27
#define DEV_2D					28
#define DEV_MAX					29

void device_controller_reset(int dev_no);

#endif	/* __DEV_RESET_H */
