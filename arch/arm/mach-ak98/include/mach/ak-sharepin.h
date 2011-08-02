#ifndef _AK88_SHRPIN_H_
#define _AK88_SHRPIN_H_

#define AK_SHRPIN_CTRL1		(AK98_VA_SYSCTRL+0x78)
#define	AK_SHRPIN_CTRL2		(AK98_VA_SYSCTRL+0x74)

/* macro for setting sharepin control register 1 */

#define AK_SHRPIN_GRP0			(0)
#define AK_SHRPIN_GRP0_GPIO		(0<<0)
#define AK_SHRPIN_GRP0_JTAG_PCM		(1<<0)

#define AK_SHRPIN_GRP1			(1)
#define AK_SHRPIN_GRP1_GPIO		(0<<1)
#define AK_SHRPIN_GRP1_RTCK		(1<<1)

#define AK_SHRPIN_GRP2			(2)
#define AK_SHRPIN_GRP2_GPIO		(0<<2)
#define AK_SHRPIN_GRP2_I2S		(1<<2)

#define AK_SHRPIN_GRP3			(3)
#define AK_SHRPIN_GRP3_GPIO		(0<<3)
#define AK_SHRPIN_GRP3_OTG		(1<<3)

#define AK_SHRPIN_GRP4			(4)
#define AK_SHRPIN_GRP4_GPIO		(0<<4)
#define AK_SHRPIN_GRP4_PWM1		(1<<4)

#define AK_SHRPIN_GRP5			(5)
#define AK_SHRPIN_GRP5_GPIO		(0<<5)
#define AK_SHRPIN_GRP5_PWM2		(1<<5)

#define AK_SHRPIN_GRP6			(6)
#define AK_SHRPIN_GRP6_GPIO		(0<<6)
#define AK_SHRPIN_GRP6_PWM3		(1<<6)

#define AK_SHRPIN_GRP7			(7)
#define AK_SHRPIN_GRP7_GPIO		(0<<7)
#define AK_SHRPIN_GRP7_PWM4		(1<<7)

#define AK_SHRPIN_GRP8			(8)
#define AK_SHRPIN_GRP8_GPIO		(0<<8)
#define AK_SHRPIN_GRP8_I2S_MCLK		(1<<8)

#define AK_SHRPIN_GRP9			(9)
#define AK_SHRPIN_GRP9_GPIO		(0<<9)
#define AK_SHRPIN_GRP9_UART1		(1<<9)

#define AK_SHRPIN_GRP10			(10)
#define AK_SHRPIN_GRP10_GPIO		(0<<10)
#define AK_SHRPIN_GRP10_UART2		(1<<10)

#define AK_SHRPIN_GRP11			(11)
#define AK_SHRPIN_GRP11_GPIO		(0<<11)
#define AK_SHRPIN_GRP11_UART2		(1<<11)

#define AK_SHRPIN_GRP12			(12)
#define AK_SHRPIN_GRP12_GPIO		(0<<12)
#define AK_SHRPIN_GRP12_UART3		(1<<12)

#define AK_SHRPIN_GRP13			(13)
#define AK_SHRPIN_GRP13_GPIO		(0<<13)
#define AK_SHRPIN_GRP13_UART3		(1<<13)

#define AK_SHRPIN_GRP14			(14)
#define AK_SHRPIN_GRP14_GPIO		(0<<14)
#define AK_SHRPIN_GRP14_UART4_SDIO	(1<<14)

#define AK_SHRPIN_GRP15			(15)
#define AK_SHRPIN_GRP15_GPIO		(0<<15)
#define AK_SHRPIN_GRP15_UART4_SDIO	(1<<15)

#define AK_SHRPIN_GRP16			(16)
#define AK_SHRPIN_GRP16_GPIO		(0<<16)
#define AK_SHRPIN_GRP16_NFC_MDAT	(1<<16)

#define AK_SHRPIN_GRP17			(17)
#define AK_SHRPIN_GRP17_GPIO		(0<<17)
#define AK_SHRPIN_GRP17_NFC_MDAT	(1<<17)

#define AK_SHRPIN_GRP18			(18)
#define AK_SHRPIN_GRP18_GPIO		(0<<18)
#define AK_SHRPIN_GRP18_NFC_MDAT	(1<<18)

#define AK_SHRPIN_GRP19			(19)
#define AK_SHRPIN_GRP19_GPIO		(0<<19)
#define AK_SHRPIN_GRP19_NFC_CE		(1<<19)

#define AK_SHRPIN_GRP22			(22)
#define AK_SHRPIN_GRP22_GPIO		(0<<22)
#define AK_SHRPIN_GRP22_NFC_RDY		(1<<22)

#define AK_SHRPIN_GRP24			(24)
#define AK_SHRPIN_GRP24_GPIO		(0<<24)
#define AK_SHRPIN_GRP24_VI		(1<<24)

#define AK_SHRPIN_GRP25			(25)
#define AK_SHRPIN_GRP25_GPIO		(0<<25)
#define AK_SHRPIN_GRP25_LCD_DATA	(1<<25)

#define AK_SHRPIN_GRP26			(26)
#define AK_SHRPIN_GRP26_GPIO		(0<<26)
#define AK_SHRPIN_GRP26_LCD_DATA	(1<<26)

#define AK_SHRPIN_GRP27			(27)
#define AK_SHRPIN_GRP27_GPIO		(0<<27)
#define AK_SHRPIN_GRP27_LCD_DATA	(1<<27)

#define AK_SHRPIN_GRP28			(28)
#define AK_SHRPIN_GRP28_GPIO		(0<<28)
#define AK_SHRPIN_GRP28_MPU_RST		(1<<28)

#define AK_SHRPIN_GRP29			(29)
#define AK_SHRPIN_GRP29_GPIO		(0<<29)
#define AK_SHRPIN_GRP29_MDAT		(1<<29)

#define AK_SHRPIN_GRP30			(30)
#define AK_SHRPIN_GRP30_GPIO		(0<<30)
#define AK_SHRPIN_GRP30_SPI1		(1<<30)

#define AK_SHRPIN_GRP31			(31)
#define AK_SHRPIN_GRP31_GPIO		(0<<31)
#define AK_SHRPIN_GRP31_SPI2		(1<<31)

#define AK_SHRPIN_RESERVED1		(20)
#define	AK_SHPRIN_RESERVED2		(21)
#define AK_SHRPIN_RESERVED3		(23)

/* macro for setting sharepin control register 2 */
#define AK_SHRPIN2_GRP1_MASK		(0x1<<0)
#define AK_SHRPIN2_GRP1_PCM		(0x0<<0)
#define AK_SHRPIN2_GRP1_JTAG		(0x1<<0)

#define AK_SHRPIN2_GRP2_MASK		(0x3<<1)
#define AK_SHRPIN2_GRP2_UART4		(0x1<<1)
#define AK_SHRPIN2_GRP2_SDIO		(0x2<<1)

#define AK_SHRPIN2_GRP3_MASK		(0x3<<3)
#define AK_SHRPIN2_GRP3_NFC		(0x1<<3)
#define AK_SHRPIN2_GRP3_MMC		(0x2<<3)

#define AK_SHRPIN2_GRP4_MASK		(0x3<<5)
#define	AK_SHRPIN2_GRP4_MDAT0		(0x1<<5)

#endif				/* _AK88_SHRPIN_H_ */
