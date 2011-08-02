/*
 * arch/arm/mach-ak98/include/mach/ak98_gpio.h
 */
#ifndef __REGS_GPIO_H_
#define __REGS_GPIO_H_

#include <mach/map.h>

#define rGPIO_DIR1      REG_VA_VAL(AK98_VA_SYSCTRL, 0x007C)
#define rGPIO_DIR2      REG_VA_VAL(AK98_VA_SYSCTRL, 0x0084)
#define rGPIO_DIR3      REG_VA_VAL(AK98_VA_SYSCTRL, 0x008C)
#define rGPIO_DIR4      REG_VA_VAL(AK98_VA_SYSCTRL, 0x0094)
#define rGPIO_DIR5      REG_VA_VAL(AK98_VA_SYSCTRL, 0x011C)

#define rGPIO_OUT1   REG_VA_VAL(AK98_VA_SYSCTRL, 0x0080)
#define rGPIO_OUT2   REG_VA_VAL(AK98_VA_SYSCTRL, 0x0088)
#define rGPIO_OUT3   REG_VA_VAL(AK98_VA_SYSCTRL, 0x0090)
#define rGPIO_OUT4   REG_VA_VAL(AK98_VA_SYSCTRL, 0x0098)
#define rGPIO_OUT5   REG_VA_VAL(AK98_VA_SYSCTRL, 0x0118)

#define rGPIO_IN1    REG_VA_VAL(AK98_VA_SYSCTRL, 0x00BC)
#define rGPIO_IN2    REG_VA_VAL(AK98_VA_SYSCTRL, 0x00C0)
#define rGPIO_IN3    REG_VA_VAL(AK98_VA_SYSCTRL, 0x00C4)
#define rGPIO_IN4    REG_VA_VAL(AK98_VA_SYSCTRL, 0x00C8)
#define rGPIO_IN5    REG_VA_VAL(AK98_VA_SYSCTRL, 0x0124)

#define rGPIO_INT1   REG_VA_VAL(AK98_VA_SYSCTRL, 0x00E0)
#define rGPIO_INT2   REG_VA_VAL(AK98_VA_SYSCTRL, 0x00E4)
#define rGPIO_INT3   REG_VA_VAL(AK98_VA_SYSCTRL, 0x00E8)
#define rGPIO_INT4   REG_VA_VAL(AK98_VA_SYSCTRL, 0x00EC)

#define rGPIO_INTPOL1  REG_VA_VAL(AK98_VA_SYSCTRL, 0x00F0)
#define rGPIO_INTPOL2  REG_VA_VAL(AK98_VA_SYSCTRL, 0x00F4)
#define rGPIO_INTPOL3  REG_VA_VAL(AK98_VA_SYSCTRL, 0x00F8)
#define rGPIO_INTPOL4  REG_VA_VAL(AK98_VA_SYSCTRL, 0x00FC)

/*
 * The other mode defined
 *
 * #define AK98_GPIO_DIR(n)			(AK98_VA_SYSCTRL + 0x007C + (n) * 8) //n = 0 ~ 3
 * #define AK98_GPIO_OUT(n)			(AK98_VA_SYSCTRL + 0x0080 + (n) * 8) //n = 0 ~ 3
 * #define AK98_GPIO_IN(n)  		(AK98_VA_SYSCTRL + 0x00BC + (n) * 4) //n = 0 ~ 3
 * #define AK98_GPIO_INT(n)  		(AK98_VA_SYSCTRL + 0x00E0 + (n) * 4) //n = 0 ~ 3
 * #define AK98_GPIO_INT_POLA(n)  	(AK98_VA_SYSCTRL + 0x00F0 + (n) * 4) //n = 0 ~ 3
 *
 */

//#define rGPIO_DIR1      REG_VA_VAL(AK98_GPIO_DIR(0), 0x00)

/************************** GPIO ***********************************/
#define AK98_GPIO_DIR(n)       (AK98_VA_SYSCTRL+0x007C+n*8) //n=0~3
#define AK98_PA_GPIO_DIR(n)       (AK98_PA_SYS+0x007C+n*8)  //n=0~3

//0x7C for GPIO[0] ~ GPIO[31]
//0x84 for GPIO[32] ~ GPIO[63]
//0x8C for GPIO[64] ~ GPIO[79], bit20:DGPIO[19],bit29:DGPIO[28]
//0x94 for bit[9:6]: DGPIO[0]~ DGPIO[3]

#define AK98_GPIO_OUT(n)         (AK98_VA_SYSCTRL+0x0080+n*8)   //n=0~3
#define AK98_PA_GPIO_OUT(n)         (AK98_PA_SYS+0x0080+n*8)    //n=0~3

//0x80
//0x88
//0x90
//0x98

#define AK98_GPIO_IN(n)          (AK98_VA_SYSCTRL+0x00BC+n*4)   //n=0~3
#define AK98_PA_GPIO_IN(n)          (AK98_PA_SYS+0x00BC+n*4)    //n=0~3

//0xBC
//0xC0
//0xC4
//0xC8

#define AK98_GPIO_INT_EN1         (AK98_VA_SYSCTRL+0x00E0+n*4)  //n=0~3
#define AK98_PA_GPIO_INT_EN1         (AK98_VA_SYSCTRL+0x00E0+n*4)   //n=0~3
//0xE0
//0xE4
//0xE8
//0xEC

#define AK98_GPIO_INT_POLA1       (AK98_VA_SYSCTRL+0x00F0+n*4)  //n=0~3
#define AK98_PA_GPIO_INT_POLA1       (AK98_VA_SYSCTRL+0x00F0+n*4)   //n=0~3
//0xF0
//0xF4
//0xF8
//0xFC

#endif /* __REGS_GPIO_H_ */

