#ifndef _COMMUNICATE_H_
#define _COMMUNICATE_H_

//#include "AK3220_types.h"
#ifndef _AK3220_TYPES_H_
#define _AK3220_TYPES_H_

/* preliminary type definition for global area */
typedef unsigned char T_U8;	/* unsigned 8 bit integer */
typedef unsigned short T_U16;	/* unsigned 16 bit integer */
typedef unsigned long T_U32;	/* unsigned 32 bit integer */
typedef signed char T_S8;	/* signed 8 bit integer */
typedef signed short T_S16;	/* signed 16 bit integer */
typedef signed long T_S32;	/* signed 32 bit integer */
typedef void T_VOID;		/* void */

#define HAL_READ_UINT32(reg, val)      ((val) = *((volatile unsigned long *)(reg)))
#define HAL_WRITE_UINT32(reg, val)     (*((volatile  unsigned long *)(reg)) = (val))

#define AK_FALSE                                0
#define AK_TRUE                                 1
//#define AK_NULL                                 ((T_VOID*)0)
#endif

typedef enum {
	GLOBE_BUF0 = 0,
	GLOBE_BUF1,
	GLOBE_BUF2,
	GLOBE_BUF3,
	GLOBE_BUF4,
	GLOBE_BUF5,
	GLOBE_BUF6,
	GLOBE_BUF7
} GLOBE_BUF_ID;

typedef enum {
	USB_BULK_SEND,
	USB_BULK_RECE,
	USB_ISO,
	NAND_FLASH,
	MMC_SD1,
	MMC_SD2,
	MMC_SD3,
	SPI1_RECE,
	SPI1_SEND,
	DAC,
	SPI2_RECE,
	SPI2_SEND,
	GPS
} PERIPHERAL_TYPE;

//static T_U32 get_buf_id(PERIPHERAL_TYPE dev_type);
#define INVALID_BUF_ID 0xFFFFFFFF

T_U8 communicate_conf(PERIPHERAL_TYPE dev_type, GLOBE_BUF_ID buf_id);
T_U8 prepare_dat_send_cpu(const T_U8 * buf, T_U32 len,
			  PERIPHERAL_TYPE dev_type);
T_U8 rece_dat_cpu(T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type);
T_U8 prepare_dat_send_dma(const T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type);
T_U8 rece_data_dma(T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type);
void set_buf_stat_empty(GLOBE_BUF_ID buf_id);
void set_buf_empty(PERIPHERAL_TYPE dev_type);

#endif
