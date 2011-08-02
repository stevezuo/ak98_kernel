
#ifndef __AK98_CAMERA_H
#define __AK98_CAMERA_H

#define AK98_CAM_DRV_NAME 		"ak98_camera"
#define MAX_VIDEO_MEM			16

#define CSI_BUS_FLAGS	(SOCAM_MASTER | SOCAM_HSYNC_ACTIVE_HIGH | \
			SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW | \
			SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING | \
			SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_LOW | \
			SOCAM_DATAWIDTH_8)

#define AK98_CAMERA_MASTER			1
#define AK98_CAMERA_DATAWIDTH_4		2
#define AK98_CAMERA_DATAWIDTH_5		4
#define AK98_CAMERA_DATAWIDTH_8		8
#define AK98_CAMERA_DATAWIDTH_9		0x10
#define AK98_CAMERA_DATAWIDTH_10	0x20

/* Image Sensor Command Register */
#define CICR_DATA_FMT	(0x3 << 0)
#define CICR_HSCAL_EN	(1 << 2)
#define CICR_VSCAL_EN	(1 << 3)
#define CICR_MODE		(1 << 4)
#define CICR_FULL_RANGE_YUV		(1 << 5)
#define CICR_DATA_FMT_VAL(x)	(((x) << 0) & CICR_DATA_FMT)


/* Personal */


/** @{@name IMAGE sensor module register and bit map define
 */
#define IMAGE_MODULE_BASE_ADDR                          0x2000C000      // image sensor
/* image capturing command */
#define IMG_CMD                                0x0000
/* Source/Destination image horizontal length */
#define IMG_HINFO1                           0x0004
/* Horizontal scalling information */
#define IMG_HINFO2					0x0008
/* Source/Destination image vertical length */
#define IMG_VINFO1                           0x000C
/* Horizontal scalling information */
#define IMG_VINFO2                         	0x0010

/* DMA starting address of external RAM for Y component of odd frame */                                                        
#define IMG_YODD                          	0x0018
#define IMG_UODD                          	0x001c
#define IMG_VODD                           	0x0020
#define IMG_RGBODD                       	0x0024
#define IMG_YEVE                            	0x0028
#define IMG_UEVE                            	0x002c
#define IMG_VEVE                            	0x0030
#define IMG_RGBEVE                        	0x0034
/* Image sensor configuration */
#define IMG_CONFIG                        	0x0040
/* Status of the current frame */
#define IMG_STATUS                        	0x0060
/* The line number of a frame when JPEG-compressed format */
#define IMG_NUM                             	0x0080
/* Multiple function control register */ 
//#define MUL_FUN_CTL_REG                                 (CHIP_CONF_BASE_ADDR | 0x0058)
/** @} */




/** @{@name IMAGE sensor module register and bit map define
 */
#define IMAGE_MODULE_BASE_ADDR                          0x2000C000      // image sensor
/* image capturing command */
#define IMG_CMD_ADDR                                    (IMAGE_MODULE_BASE_ADDR | 0x0000)
/* Source/Destination image horizontal length */
#define IMG_HINFO1_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x0004)
/* Horizontal scalling information */
#define IMG_HINFO2_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x0008)
/* Source/Destination image vertical length */
#define IMG_VINFO1_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x000C)
/* Horizontal scalling information */
#define IMG_VINFO2_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x0010)

/* DMA starting address of external RAM for Y component of odd frame */                                                        
#define IMG_YADDR_ODD                                   (IMAGE_MODULE_BASE_ADDR | 0x0018)
#define IMG_UADDR_ODD                                   (IMAGE_MODULE_BASE_ADDR | 0x001c)
#define IMG_VADDR_ODD                                   (IMAGE_MODULE_BASE_ADDR | 0x0020)
#define IMG_RGBADDR_ODD                                 (IMAGE_MODULE_BASE_ADDR | 0x0024)
#define IMG_YADDR_EVE                                   (IMAGE_MODULE_BASE_ADDR | 0x0028)
#define IMG_UADDR_EVE                                   (IMAGE_MODULE_BASE_ADDR | 0x002c)
#define IMG_VADDR_EVE                                   (IMAGE_MODULE_BASE_ADDR | 0x0030)
#define IMG_RGBADDR_EVE                                 (IMAGE_MODULE_BASE_ADDR | 0x0034)
/* Image sensor configuration */
#define IMG_CONFIG_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x0040)
/* Status of the current frame */
#define IMG_STATUS_ADDR                                 (IMAGE_MODULE_BASE_ADDR | 0x0060)
/* The line number of a frame when JPEG-compressed format */
#define IMG_NUM_ADDR                                    (IMAGE_MODULE_BASE_ADDR | 0x0080)
/* Multiple function control register */ 
#define MUL_FUN_CTL_REG                                 (CHIP_CONF_BASE_ADDR | 0x0058)
/** @} */

struct captureSync{
	unsigned long long adcCapture_bytes;
	struct timeval tv;
};


#endif
