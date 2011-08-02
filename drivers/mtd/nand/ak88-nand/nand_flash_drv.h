/*
 *	driver for nand flash
 */

#ifndef _NAND_FLASH_DRV_H_
#define _NAND_FLASH_DRV_H_

//#include "AK3220_types.h"

#define ECC_MODE1		// ECC mode 0 if not defined
//#define L2_DMA_MODE   // CPU mode if not defined

#define TEST_FAULT_NUM		8	// # of error bits to simulate
#define NAND_PAGE_PER_PHYPAGE	4

#define NAND_MAX_PAGE_SIZE	640

//  |               page               |
//  | data | fs(file system) | parity  |
//  | data |          spare            |
#ifdef  ECC_MODE1
#define ECC_CTL_MODE		ECC_CTL_MODE1
#define NAND_PARITY_SIZE	13
#define MAX_ERROR_CNT		8
#define NAND_PAGE_SIZE		528
#else
#define ECC_CTL_MODE		ECC_CTL_MODE0
#define NAND_PARITY_SIZE	7
#define MAX_ERROR_CNT		4
#define NAND_PAGE_SIZE		528
#endif

#define NAND_DATA_SIZE		512
#define NAND_SPARE_SIZE		(NAND_PAGE_SIZE-NAND_DATA_SIZE)
#define NAND_FS_SIZE		(NAND_SPARE_SIZE-NAND_PARITY_SIZE)
#define NAND_EFFECTIVE_SIZE	(NAND_DATA_SIZE+NAND_FS_SIZE)

#define FAULT_NUM		(TEST_FAULT_NUM>MAX_ERROR_CNT? MAX_ERROR_CNT:TEST_FAULT_NUM)

////// flash macro define ////////////////////////////////////

#define COMMAND_SEQUENCE_TOTAL	8
#define LARGE_PAGE_LEN		2048	// 2KB
#define SMALL_PAGE_LEN		512	// 512B
#define NFC_FIFO_LEN		512	// AK3223 nand flash controller's FIFO length
#define NAND_FLASH_MAX_DELAY	0xFFF	// 50 mS #84MHz

#define NAND_FLASH_DEFAULT_DELAY	0xa	// change this parameter all spec is descript 5us for reset

#define MAX_NAND_SECOND_BOOT_SIZE	0x40001A88	// 6792 bytes LIMIT
////// end flash macro define ////////////////////////////////////

#define NFLASH_READ_ID		0x90
#define NFLASH_READ_STATUS	0x70
#define NFLASH_RESET		0xff

#define NFLASH_BLOCK_ERASE0	0x60
#define NFLASH_BLOCK_ERASE1	0xd0

#define NFLASH_PAGE_PROGRAM0	0x80
#define NFLASH_PAGE_PROGRAM1	0x10

#define NFLASH_CACHE_PROGRAM0	0x80
#define NFLASH_CACHE_PROGRAM1	0x15

#define NFLASH_CB_PROGRAM0	0x85
#define NFLASH_CB_PROGRAM1	0x10

#define NFLASH_RAN_PROGRAM	0x85

#define NFLASH_READ0		0x00
#define NFLASH_READ0_HALF	0x01
#define NFLASH_READ1		0x30
#define NFLASH_READ12		0x50

#define NFLASH_CB_READ0		0x00
#define NFLASH_CB_READ1		0x35

#define NFLASH_RAN_READ0	0x05
#define NFLASH_RAN_READ1	0xe0
#define DEFAULT_GO		(1 << 30)

#define AK_NAND_STATUS_READY 	(1 << 6)
#define AK_NAND_STATUS_ERROR 	(1 << 0)

#define NCHIP_SELECT(x)		((0x01 << (x)) << 10)


#define FLASH_ECC_POSITION_REG0     (AK88_VA_ECCCTRL + 0X04)

/**  very important!!! **/
#define DATA_SIZE 4096      /**/
#define OOB_SIZE  128

//#define PAGE_SIZE 4224


extern unsigned long ak880x_nand_get_chipid(unsigned char chip);
extern void ak880x_nand_settiming(unsigned long cmd_timing,
				  unsigned long data_timing);
extern unsigned char ak880x_nand_get_status(unsigned char chip);
extern void ak880x_nand_eraseblock(unsigned char chip, unsigned int phypage);
extern void ak880x_nand_reset(unsigned char chip, unsigned short wait_time);
extern void ak880x_nand_inithw(void);
extern void ak880x_nand_read_page(unsigned char chip, unsigned int row,
				  unsigned int col, unsigned char *buf,
				  unsigned int len, unsigned char large_page);
extern void ak880x_nand_write_page(unsigned char chip, unsigned int row,
				   unsigned int col, const unsigned char *buf,
				   unsigned int len, unsigned char large_page);

void nand_write_page_ecc( int page, const unsigned char *buf,const unsigned char *oobbuf);
void nand_read_page_ecc( int page, unsigned char * buf,unsigned char *oobbuf);
void nand_read_page_data( int page, unsigned char * data_buf);
void nand_read_page_oob( int page, unsigned char * oob_buf);

void nand_read_oob_ecc( int page, unsigned char *oobbuf);
void nand_write_page_ecc_part( int page, const unsigned char *buf,int len );
void nand_read_page_ecc_part( int page, unsigned char * buf,int len );

#endif				// _NAND_FLASH_DRV_H_
