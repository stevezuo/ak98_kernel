/* arch/arm/arch-ak98/include/mach/map.h
 *
 * AK98 - Memory map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MAP_H
#define __ASM_ARCH_MAP_H

#ifndef __ASSEMBLY__
#define AK98_ADDR(x)		((void __iomem *)0xF0000000 + (x))
#else
#define AK98_ADDR(x)		(0xF0000000 + (x))
#endif

#define AK98_VA_OCROM		AK98_ADDR(0x00000000)
#define AK98_PA_OCROM		0x00000000
#define AK98_SZ_OCROM		3 * SZ_16K	/* 48KB */

#define AK98_VA_SYSCTRL		AK98_ADDR(0x00100000)
#define AK98_PA_SYSCTRL		(0x08000000)
#define AK98_SZ_SYSCTRL		SZ_64K		/* 64KB */

/* some sub system control register */
#define AK98_VA_SUBCTRL		AK98_ADDR(0x00200000)
#define AK98_PA_SUBCTRL		(0x20000000)
#define AK98_SZ_SUBCTRL		SZ_1M 	 	/* 1MB */

#define AK98_VA_H264MEM		(AK98_VA_SUBCTRL + 0x0)
#define AK98_PA_H264MEM		(AK98_PA_SUBCTRL + 0x0)
#define AK98_SZ_H264MEM		2 * SZ_16K 	/* 32KB */

#define AK98_VA_CAMER		(AK98_VA_SUBCTRL + 0xC000)
#define AK98_PA_CAMER		(AK98_PA_SUBCTRL + 0xC000)

#define AK98_VA_DISP		(AK98_VA_SUBCTRL + 0xD000)
#define AK98_PA_DISP		(AK98_PA_SUBCTRL + 0xD000)

#define AK98_VA_REGRAM		(AK98_VA_SUBCTRL + 0xE000)
#define AK98_PA_REGRAM		(AK98_PA_SUBCTRL + 0xE000)

#define AK98_VA_L2CACH		(AK98_VA_SUBCTRL + 0xF000)
#define AK98_PA_L2CACH		(AK98_PA_SUBCTRL + 0xF000)

/* reference base addr for register control */
#define AK98_VA_DEV		(AK98_VA_SUBCTRL + 0x20000)
#define AK98_PA_DEV		(AK98_PA_SUBCTRL + 0x20000)

#define AK98_VA_MMC			(AK98_VA_DEV + 0x0000)
#define AK98_PA_MMC			(AK98_PA_DEV + 0x0000)

#define AK98_VA_SDIO		(AK98_VA_DEV + 0x1000)
#define AK98_PA_SDIO		(AK98_PA_DEV + 0x1000)

#define AK98_VA_2DACC		(AK98_VA_DEV + 0x2000)
#define AK98_PA_2DACC		(AK98_PA_DEV + 0x2000)

#define AK98_VA_I2C			(AK98_VA_DEV + 0x3000)
#define AK98_PA_I2C			(AK98_PA_DEV + 0x3000)

#define AK98_VA_SPI1		(AK98_VA_DEV + 0x4000)
#define AK98_PA_SPI1		(AK98_PA_DEV + 0x4000)

#define AK98_VA_SPI2		(AK98_VA_DEV + 0x5000)
#define AK98_PA_SPI2		(AK98_PA_DEV + 0x5000)

#define AK98_VA_UART		(AK98_VA_DEV + 0x6000)
#define AK98_PA_UART		(AK98_PA_DEV + 0x6000)

#define AK98_VA_NFCTRL		(AK98_VA_SUBCTRL + 0x2A000)
#define AK98_PA_NFCTRL		(AK98_PA_SUBCTRL + 0x2A000)

#define AK98_VA_ECCCTRL	    (AK98_VA_SUBCTRL + 0x2B000)
#define AK98_PA_ECCCTRL	    (AK98_PA_SUBCTRL + 0x2B000)


#define AK98_VA_ECC			(AK98_VA_DEV + 0xB000)
#define AK98_PA_ECC			(AK98_PA_DEV + 0xB000)

#define AK98_VA_L2CTRL		(AK98_VA_DEV + 0xC000)
#define AK98_PA_L2CTRL		(AK98_PA_DEV + 0xC000)

#define AK98_VA_ADC			(AK98_VA_DEV + 0xD000)
#define AK98_PA_ADC			(AK98_PA_DEV + 0xD000)

#define AK98_VA_DAC			(AK98_VA_DEV + 0xE000)
#define AK98_PA_DAC			(AK98_PA_DEV + 0xE000)

#define AK98_VA_ROTCTRL		(AK98_VA_DEV + 0xF000)
#define AK98_PA_ROTCTRL		(AK98_PA_DEV + 0xF000)

#define AK98_VA_PCM			(AK98_VA_SUBCTRL + 0x30000)
#define AK98_PA_PCM			(AK98_PA_SUBCTRL + 0x30000)

#define AK98_VA_IMAGE		(AK98_VA_SUBCTRL + 0x40000)
#define AK98_PA_IMAGE		(AK98_PA_SUBCTRL + 0x40000)

#define AK98_VA_MOTCTRL		(AK98_VA_SUBCTRL + 0x41000)
#define AK98_PA_MOTCTRL		(AK98_PA_SUBCTRL + 0x41000)

#define AK98_VA_RMVB		(AK98_VA_SUBCTRL + 0x70000)
#define AK98_PA_RMVB		(AK98_PA_SUBCTRL + 0x70000)

#define AK98_VA_MPEG2		(AK98_VA_SUBCTRL + 0x71000)
#define AK98_PA_MPEG2		(AK98_PA_SUBCTRL + 0x71000)

#define AK98_VA_RHUFF		(AK98_VA_SUBCTRL + 0x72000)
#define AK98_PA_RHUFF		(AK98_PA_SUBCTRL + 0x72000)

#define AK98_VA_L2MEM		AK98_ADDR(0x00300000)
#define AK98_PA_L2MEM		(0x48000000)
#define AK98_SZ_L2MEM		SZ_1M /* 2 * 64KB */

#define AK98_VA_MAC			AK98_ADDR(0x00400000)
#define AK98_PA_MAC			(0x60000000)
#define AK98_SZ_MAC			2 * SZ_64K  /* 2 * 64KB */

#define AK98_VA_USB			AK98_ADDR(0x00500000)
#define AK98_PA_USB			(0x70000000)
#define AK98_SZ_USB			SZ_1M

/*
 * This is used for the CPU specific mappings that may be needed, so that
 * they do not need to directly used AK98_ADDR() and thus make it easier to
 * modify the space for mapping.
 */
#define	write_ramb(v, p)		(*(volatile unsigned char *)(p) = (v))
#define write_ramw(v, p)		(*(volatile unsigned short *)(p) = (v))
#define write_raml(v, p)		(*(volatile unsigned long *)(p) = (v))

#define read_ramb(p)			(*(volatile unsigned char *)(p))
#define read_ramw(p)			(*(volatile unsigned short *)(p))
#define read_raml(p)			(*(volatile unsigned long *)(p))

#define write_buf(v, p)			(*(volatile unsigned long *)(p) = (v))
#define read_buf(p)				(*(volatile unsigned long *)(p))

/*
 * This is used mode:
 * #define rTIME1_CON      REG_VA_VAL(AK98_VA_SYS, 0x0018)
 * #define rTIME2_CON      REG_VA_VAL(AK98_VA_SYS, 0x001C)
 */
#define REG_VA_VAL(base_addr, offset)	(*(volatile unsigned long *)((base_addr) + (offset)))
#define REG_VA_ADDR(base_addr, offset)	((base_addr) + (offset))

#define REG_PA_VAL(base_addr, offset)	(*(volatile unsigned long *)((base_addr) + (offset)))
#define REG_PA_ADDR(base_addr, offset)	((base_addr) + (offset))


#endif  /* __ASM_ARCH_MAP_H */

