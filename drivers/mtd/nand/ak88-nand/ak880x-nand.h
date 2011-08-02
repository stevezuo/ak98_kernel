/*----------------------------------------------------------------
 *Copyright (c) 2004,Anyka(GuangZhou) Software Technology Co.,Ltd.                            
 *All rights reserved.                            
 *                             
 *File name:                            
 *        ak3221m-nand.h
 *Brief:                            
 *        nandflash driver.
 *author:                            
 *        liang_dongcai
 *version:                            
 *        0.0.1
 *history:    
 -----------------------------------------------------------------*/

#ifndef _AK7801_NAND_H
#define _AK7801_NAND_H

//#include <asm/sizes.h>

#define NANDDRV_SPEED_LOW		0
#define NANDDRV_SPEED_MIDDLE		1
#define NANDDRV_SPEED_HIGH		2
#define NANDDRV_SPEED_SPEC		3

#define COPYBACK_NONE			0
#define COPYBACK_SUPPORT		1
//*******************************************
//this is define of our chip nandflash driver information
// AK3223 NFC's FIFO size is 512 bytes
//this is 3221 chip controller
#define CHIP_NFC_7801
//#define CHIP_NFC_3229
//#define CHIP_NFC_3310

//**************************************************************
//all  of this define is for 3221 nandflash controller
//**************************************************************
#ifdef CHIP_NFC_7801

#define  PHY_ERR_FLAG           0
//basic save information
#define NFC_FIFO_SIZE           512
#define NFC_LOG_PAGE_SIZE       512
#define NFC_LOG_SPARE_SIZE      16
//#define NFC_FS_INFO_OFFSET    522
#define NFC_FS_INFO_OFFSET      520	//must be 4 multiple
#define NFC_FS_INFO_SIZE        6
#define NFC_SUPPORT_CHIPNUM     4

#define BASIC_DLYCNT               0x30

//#######enddefine of ecc encode and decode ################

#endif				//CHIP_NFC_3221

//********************************************************************************//
//nandflash information of one chip
//********************************************************************************//
//physic information
#define NFLASH_PAGE_SIZE            (Nand_SysInfo.phy_page_size)
#define NFLASH_BLOCK_NUM            (Nand_SysInfo.total_blk_num)

//log information
#define NFLASH_LOGPAGE_NUM          (Nand_SysInfo.total_logpage_num)

#ifdef POP_NF_HARDWARE

#define NFLASH_LOGPAGE_SIZE          512
#define NFLASH_LOGSPARE_SIZE         16
#define NFLASH_PHYPAGES_PER_BLOCK    64
#define NFLASH_LOG_PER_PHY           4
#else

#define NFLASH_LOGPAGE_SIZE          (Nand_SysInfo.log_page_size)
#define NFLASH_LOGSPARE_SIZE         (Nand_SysInfo.log_spare_size)
#define NFLASH_PHYPAGES_PER_BLOCK    chipInfo.phy_page_per_blk
#define NFLASH_LOG_PER_PHY           Nand_SysInfo.Nand_LogPerPhy
#endif				/* POP_NF_HARDWARE */

// CRC begin at second bit
#define CRC_TYPE_START_BIT    2

// ECC flag bit
#define RS1_NO_ERR_BIT                (1 << 11)
#define RS2_NO_ERR_BIT                (1 << 12)
#define BCH_NO_ERR_BIT                (1 << 13)

#define RS1_CANT_CORRECT_BIT          (1 << 5)
#define RS2_CANT_CORRECT_BIT          (1 << 7)
#define BCH_CANT_CORRECT_BIT          (1 << 9)

#define RS1_CORRECT_END_BIT           (1 << 6)
#define RS2_CORRECT_END_BIT           (1 << 8)
#define BCH_CORRECT_END_BIT           (1 << 10)

// ECC error code( reservd 16 error code)
/*
 * function do_ECC_coerr() setting has two valve: 16 or 2
 *    when do_ECC_coerr() return value >= ECC_CANT_CORRECT, 
 *        mean error can't correct, don't need execute CRC;
 *    when do_ECC_coerr() 
 *        return value < ECC_CANT_CORRECT and >= ECC_NEED_CRC_CHECK,
 *        mean error has too much, maybe correct, then execute verify.
 *    when do_ECC_coerr return value == 0, 
 *        mean no error or had correct already.
 */
#define ECC_NEED_CRC_CHECK          (2)
#define ECC_CANT_CORRECT            (1 << 4)
#define RS1_CANT_CORRECT            (1 << 4)
#define RS2_CANT_CORRECT            (1 << 5)
#define BCH_CANT_CORRECT            (1 << 6)

#define RS1_COERR                    1
#define RS2_COERR                    2

// AK3220 support CRC type list
typedef enum {
	CRC_TYPE_CRC8 = 0,
	CRC_TYPE_CRC12,
	CRC_TYPE_CRC16,
	CRC_TYPE_CRC24,
	CRC_TYPE_CRC32
} T_AK3220_CRC_TYPE;

#define ERROR_CHIP_ID			0x00000000
#define NFLASH_READ_ID			0x90

//custom nandflash bit information
//this byte is just for expand out driver
//bit 0,if nandflash no cache, set this bit,driver would avoid cache control
//bit 1,if nandflash command set is other,set this bit dirver would try to find this chips command set
//bit 2,if nandflash use 16 bit connect,set this bit
//bit 3,if pin R/B not connect,set this bit
//bit 4,bit 5,bit 6,bit 7,reserved for other status

#define  UNMAP_BLK_PER_GROUP		16

//status register bit
#define NFLASH_PROGRAM_SUCCESS		0x01	//bit 0
#define NFLASH_HANDLE_READY		0x40	//bit 6

/** @{@name Error Code of nandflash
 *  define the error code of nandflash
 */
#define NF_TIMEOUT			(16)
#define NF_SUCCESS			(1)
#define NF_FAIL				(0)
#define NF_ERR_ECC			(0)
#define NF_ERR_PROGRAM			(-2)
#define NF_ERR_ERASE			(-3)
#define NF_ERR_INIT			(-4)
#define NF_ERR_DEV			(-5)
#define NF_ERR_LAST			(-6)
#define NFLASH_RESET			0xff

#define NFLASH_FRAME_PROGRAM0		0x80
#define NFLASH_FRAME_PROGRAM1		0x10

#define NAND_SELCHIP_NONE		0x0
#define NAND_SELCHIP_NEED		0x1
#define NAND_SELCHIP_RESCALE		0x2

#define NAND_READBBT_TIMEOUT_MAX 	3

#define ERROR_NAND_ID			0

#endif
