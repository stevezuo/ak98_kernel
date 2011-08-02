/* 
 * linux/include/asm-arm/arch-ak98/clock.h
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
extern struct clk clk_xtal_25M;
extern struct clk clk_xtal_27M;
extern struct clk clk_xtal_32K;
extern struct clk clk_pll;
extern struct clk clk_spll;
extern struct clk clk_asic;
extern struct clk clk_mem;
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
extern struct clk i2c_clk;

typedef enum {
	CPU_MODE_NORMAL,
	CPU_MODE_CPU3X,
	CPU_MODE_SPECIAL,
	CPU_MODE_LOW,
} ak98_cpu_mode_t;

#define AK98_CLK_DIV_1_CPU3X	(1 << 28)
#define AK98_CLK_DIV_1_LOW	(1 << 22)
#define AK98_CLK_DIV_1_SPECIAL	(1 << 21)
#define AK98_CLK_DIV_1_CPU2X	(1 << 15)


#define MHz	1000000UL
#define AK98_MIN_PLL_CLK	(180 * MHz)

bool ak98_cpu_is_3x_mode(void);
bool ak98_cpu_is_2x_mode(void);
bool ak98_cpu_is_special_mode(void);
bool ak98_cpu_is_low_clock_mode(void);
bool ak98_cpu_is_normal_mode(void);
ak98_cpu_mode_t ak98_get_cpu_mode(void);
unsigned long ak98_get_asic_clk(void);
unsigned long ak98_get_mem_clk(void);
unsigned long ak98_get_cpu_clk(void);
ak98_cpu_mode_t ak98_get_cpu_mode(void);
unsigned long ak98_get_pll_clk(void);
unsigned long ak98_get_clk168m_clk(void);

#endif
