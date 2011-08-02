/* 
 * linux/include/asm-arm/arch-ak880x/clock.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _ANYKA_CLK_H_
#define _ANYKA_CLK_H_

/* reset & clock control */
#define AK_HWMMD_CODEC		0
#define AK_CAMIF		1
#define AK_SPI_MMC_UART1	2
#define AK_DISPLAY_CTRL		3
#define AK_AUDIO_PROCSESSOR	4
#define AK_USBOTG_CTRL		5
#define AK_H264_DECODER		6
#define AK_USBFS_HOST_CTRL	7
#define AK_SDIO_UART2_UART3	8
#define AK_CLK_RESERVED0	9
#define AK_MEMORY_CTRL		10
#define AK_MOTION_ESTIMATION	11
#define AK_2DGRAPHICS_ACC	12
#define AK_NANDFLASH_CTRL	13
#define AK_CLK_RESERVED1	14
#define AK_L2CTRL_UART0		15

union ak880x_clk_div1_reg {
	struct {
		unsigned long m:6;	/* 5:0  */
		unsigned long asic_clk:3;	/* 8:6  */
		unsigned long ap_clk:3;	/* 11:9 */
		unsigned long pll1_en:1;	/* 12   */
		unsigned long st:1;	/* 13   */
		unsigned long asic_ap_en:1;	/* 14   */
		unsigned long cpu_clk:1;	/* 15   */
		unsigned long rtc_wakeup:1;	/* 16   */
		unsigned long n:4;	/* 20:17 */
		unsigned long reserved:11;	/* 31:21 */
	} clk_div;
	unsigned long regval;
};

struct clk {
	struct list_head list;
	struct module *owner;
	struct clk *parent;
	const char *name;
	int id;
	int usage;
	unsigned long rate;
	unsigned long ctrlbit;

	int (*enable) (struct clk *, int enable);
	int (*set_rate) (struct clk * c, unsigned long rate);
	unsigned long (*get_rate) (struct clk * c);
	int (*set_parent) (struct clk * c, struct clk * parent);
};

/* core clock support */
extern struct clk clk_xtal_12M;
extern struct clk clk_xtal_27M;
extern struct clk clk_xtal_32K;
extern struct clk clk_pll1;
extern struct clk clk_pll2;
extern struct clk clk_asic;
extern struct clk clk_cpu;

/* other clocks which may be registered by board support */
extern struct clk ap_clk;
extern struct clk adc1_clk;
extern struct clk adc2_clk;
extern struct clk dac_clk;
extern struct clk camif_clk;
extern struct clk lcd_clk;
extern struct clk spi_clk;
extern struct clk mci_clk;
extern struct clk udc_clk;
extern struct clk uart4_clk;
 
#endif
