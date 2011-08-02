#ifndef _AK98_LCD_REGS_H
#define _AK98_LCD_REGS_H

#include <mach/map.h>

/* system control registers */
#define AK98_CLK_DIV1               (AK98_VA_SYSCTRL + 0x04)
#define AK98_CLKRST_CTRL1           (AK98_VA_SYSCTRL + 0x0C)

#define AK98_MULTIFUNC_CTRL_REG1    (AK98_VA_SYSCTRL + 0x58)

#define AK98_SRDPIN_CTRL1           (AK98_VA_SYSCTRL + 0x78)
#define AK98_SRDPIN_CTRL2           (AK98_VA_SYSCTRL + 0x74)

#define AK98_PUPD2                  (AK98_VA_SYSCTRL + 0xA0)
#define AK98_PUPD3                  (AK98_VA_SYSCTRL + 0xA4)
#define AK98_PUPD4                  (AK98_VA_SYSCTRL + 0xA8)

#define AK98_AHB_PRIORITY1          (AK98_VA_REGRAM + 0x18)
#define AK98_AHB_PRIORITY2          (AK98_VA_REGRAM + 0x1C)

/* LCD controller registers */
#define AK98_LCD_CMD1               (AK98_VA_DISP + 0x00)
#define AK98_LCD_RESET              (AK98_VA_DISP + 0x08)	//to send a reset signal
#define AK98_RGBIF_CTRL1            (AK98_VA_DISP + 0x10)	//signal of RGB interface conf
#define AK98_RGBIF_CTRL2            (AK98_VA_DISP + 0x14)	//buffer address setting and to enable virtual page func of data from RGB channel
#define AK98_RGB_VPAGE_SIZE         (AK98_VA_DISP + 0x18)	//virtual page size of the data input from RGB channel
#define AK98_RGB_VPAGE_OFFSET       (AK98_VA_DISP + 0x1C)	//virtual page offset reg
#define AK98_BG_COLOR               (AK98_VA_DISP + 0x3C)	//background color
#define AK98_RGBIF_CTRL3            (AK98_VA_DISP + 0x40)	//Horizontal/Vertical sync pulse width
#define AK98_RGBIF_CTRL4            (AK98_VA_DISP + 0x44)	//Horizontal back porch width and display area width
#define AK98_RGBIF_CTRL5            (AK98_VA_DISP + 0x48)	//Horizontal front porch width
#define AK98_RGBIF_CTRL6            (AK98_VA_DISP + 0x4C)	//Vertical back porch width
#define AK98_RGBIF_CTRL7            (AK98_VA_DISP + 0x50)	//Vertical front porch width
#define AK98_RGBIF_CTRL8            (AK98_VA_DISP + 0x54)	//Vertical display area
#define AK98_RGBIF_CTRL9            (AK98_VA_DISP + 0x58)	//Length of VOVSYNC signal
#define AK98_RGB_OFFSET             (AK98_VA_DISP + 0xA8)	//Offset values of the data input from RGB channel
#define AK98_RGB_SIZE               (AK98_VA_DISP + 0xAC)	//the size of the data input from RGB channel
#define AK98_DISP_SIZE              (AK98_VA_DISP + 0xB0)	//Display area size
#define AK98_LCD_CMD2               (AK98_VA_DISP + 0xB4)	//LCD command
#define AK98_LCD_OPER               (AK98_VA_DISP + 0xB8)	//to start the reflash func
#define AK98_LCD_STATUS             (AK98_VA_DISP + 0xBC)
#define AK98_LCD_INT_ENAB           (AK98_VA_DISP + 0xC0)	//to enable interrupt
//status_reg bits corespondents to int_enab_reg
//0x2001,00bc bits corespondents to 0x2001,00c0
#define AK98_LCD_SOFT_CTRL          (AK98_VA_DISP + 0xC8)	//software control
#define AK98_LCD_CLKCONF            (AK98_VA_DISP + 0xE8)	//LCD clock configuration

/* registers for overlay1 */
#define AK98_OV1_YADDR                (AK98_VA_DISP + 0x5C)
#define AK98_OV1_UADDR                (AK98_VA_DISP + 0x60)
#define AK98_OV1_VADDR                (AK98_VA_DISP + 0x64)
#define AK98_OV1_HORI_CONF            (AK98_VA_DISP + 0x68)
#define AK98_OV1_VERT_CONF            (AK98_VA_DISP + 0x6C)
#define AK98_OV1_SCALER               (AK98_VA_DISP + 0x70)
#define AK98_OV1_DISP_CONF            (AK98_VA_DISP + 0x74)
#define AK98_OV1_VPAGE_SIZE           (AK98_VA_DISP + 0x78)
#define AK98_OV1_VPAGE_OFFSET         (AK98_VA_DISP + 0x7C)

/* registers for overlay2 */
#define AK98_OV2_YADDR                (AK98_VA_DISP + 0x80)
#define AK98_OV2_UADDR                (AK98_VA_DISP + 0x84)
#define AK98_OV2_VADDR                (AK98_VA_DISP + 0x88)
#define AK98_OV2_HORI_CONF            (AK98_VA_DISP + 0x8C)
#define AK98_OV2_VERT_CONF            (AK98_VA_DISP + 0x90)
#define AK98_OV2_SCALER               (AK98_VA_DISP + 0x94)
#define AK98_OV2_DISP_CONF            (AK98_VA_DISP + 0x98)

/* registers for osd */
#define AK98_OSD_ADDR                 (AK98_VA_DISP + 0x20)
#define AK98_OSD_COLOR1               (AK98_VA_DISP + 0x28)
#define AK98_OSD_COLOR2               (AK98_VA_DISP + 0x2C)
#define AK98_OSD_COLOR3               (AK98_VA_DISP + 0x30)
#define AK98_OSD_COLOR4               (AK98_VA_DISP + 0x34)
#define AK98_OSD_COLOR5               (AK98_VA_DISP + 0xD0)
#define AK98_OSD_COLOR6               (AK98_VA_DISP + 0xD4)
#define AK98_OSD_COLOR7               (AK98_VA_DISP + 0xD8)
#define AK98_OSD_COLOR8               (AK98_VA_DISP + 0xDC)
#define AK98_OSD_OFFSET               (AK98_VA_DISP + 0x24)
#define AK98_OSD_SIZE_ALPHA           (AK98_VA_DISP + 0x38)

#define TVOUT_CHROMA_FREQ_REG         (AK98_VA_DISP + 0x0100)
#define TVOUT_CTRL_REG1               (AK98_VA_DISP + 0x0104)
#define TVOUT_PARA_CONFIG_REG1        (AK98_VA_DISP + 0x0108)
#define TVOUT_PARA_CONFIG_REG2        (AK98_VA_DISP + 0x010c)
#define TVOUT_PARA_CONFIG_REG3        (AK98_VA_DISP + 0x0110)
#define TVOUT_PARA_CONFIG_REG4        (AK98_VA_DISP + 0x0114)
#define TVOUT_PARA_CONFIG_REG5        (AK98_VA_DISP + 0x0118)
#define TVOUT_PARA_CONFIG_REG6        (AK98_VA_DISP + 0x011c)
#define TVOUT_CTRL_REG2               (AK98_VA_DISP + 0x0120)

/*
#define BITFIELD(high, low) (high-low+1), low
#define THPW_BITS BITFIELD(23,11)
*/

#define AKLCD_VPAGE_WIDTH_MAX                1280
#define AKLCD_VPAGE_HEIGHT_MAX               1024

#endif	/* _AK98_LCD_REGS_H */
