/**
* @FILENAME sysctl.h
*
* Copyright (C) 2006 Anyka (Guangzhou) Software Technology Co., LTD
* @DATE  2006-04-19
* @VERSION 1.0
* @REF 
*/

#ifndef __SYSCTL_H__
#define __SYSCTL_H__

/*以下定义的clock 模块必须独立，不能一个模块包含多个可控制clock的子模块*/
#define CLOCK_DEFAULT_ENABLE                        (0)
#define CLOCK_CAMERA_ENABLE                         (1<<0)
#define CLOCK_LCD_ENABLE                            (1<<1)
#define CLOCK_USB_ENABLE                            (1<<2)
#define CLOCK_MMCSD_ENABLE                          (1<<3)
#define CLOCK_UART0_ENABLE                          (1<<4)
#define CLOCK_UART1_ENABLE                          (1<<5)
#define CLOCK_UART2_ENABLE                          (1<<6)
#define CLOCK_UART3_ENABLE                          (1<<7)
#define CLOCK_SDIO_ENABLE                           (1<<8)
#define CLOCK_SPI_ENABLE                            (1<<9)
#define CLOCK_NAND_ENABLE                           (1<<10)
#define CLOCK_NBITS                                 (11)
#define CLOCK_ENABLE_MAX                            (1<<CLOCK_NBITS)

/** CLOCK control register bit map*/
#define CLOCK_CTRL_IMAGE_H263_MPEG4                 (0x1)
#define CLOCK_CTRL_CAMERA                           (1 << 1)
#define CLOCK_CTRL_SPI12_MMC_UART2                  (1 << 2)
#define CLOCK_CTRL_LCD                              (1 << 3)
#define CLOCK_CTRL_AUDIO                            (1 << 4)
#define CLOCK_CTRL_USBOTG                           (1 << 5)
#define CLOCK_CTRL_H264                             (1 << 6)
#define CLOCK_CTRL_USBFS                            (1 << 7)
#define CLOCK_CTRL_SDIO_UART34                      (1 << 8)
#define CLOCK_CTRL_SDRAM_DDR                        (1 << 10)
#define CLOCK_CTRL_MOTION_MODULE                    (1 << 11)
#define CLOCK_CTRL_2D_ACCELERATOR                   (1 << 12)
#define CLOCK_CTRL_NANDFLASH                        (1 << 13)
#define CLOCK_CTRL_L2_UART1                         (1 << 15)

#define RESET_IMAGE_PROCESS                         (1<<16)
#define RESET_CAMERA                                (1<<17)
#define RESET_SPI                                   (1<<18)
#define RESET_SDMMC                                 (1<<18)
#define RESET_PCM                                   (1<<18)
#define RESET_UART2                                 (1<<18)
#define RESET_LCD                                   (1<<19)
#define RESET_GPS                                   (1<<20)
#define RESET_USB_OTG                               (1<<21)
#define RESET_H264_DECODER                          (1<<22)
#define RESET_USB_FS                                (1<<23)
#define RESET_SDIO                                  (1<<24)
#define RESET_UART3                                 (1<<24)
#define RESET_UART4                                 (1<<24)
#define RESET_RAM                                   (1<<26)
#define RESET_MOTION_ESTIMATE                       (1<<27)
#define RESET_GRAPHICS                              (1<<28)
#define RESET_NANDFLASH                             (1<<29)
#define RESET_L2                                    (1<<31)
#define RESET_UART1                                 (1<<31)


/**
 * @BRIEF Set SleepMode
 * @AUTHOR guoshaofeng
 * @DATE 2007-04-23
 * @PARAM[in] T_U32 module
 * @RETURN T_VOID
 * @RETVAL
 * attention: if you close some parts such as LCD 
            you must init it again when you reopen 
            it 
            some settings may cause serious result
            better not to use it if not familar
 */
T_VOID sysctl_clock(T_U32 module);

/**
 * @brief reset module 
 * @author guoshaofeng
 * @date 2010-07-20
 * @param module [in]: module to be reset
 * @return T_VOID
 */
T_VOID sysctl_reset(T_U32 module);

#endif //__SYSCTL_H__