/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#include <mach/nand.h>
#include <mach/map.h>

//#include "ak880x-nand.h"
#include "nand_flash_drv.h"
#include "communicate.h"

/* ------------------------------------------------------------------------- */

#define WRITE_BUF	GLOBE_BUF5
#define READ_BUF	GLOBE_BUF4

/* NF controller */
#define FLASH_CTRL_REG0     (AK88_VA_NFCTRL+0x100)

#define FLASH_CTRL_REG20    (FLASH_CTRL_REG0 + 0x50)
#define FLASH_CTRL_REG21    (FLASH_CTRL_REG0 + 0x54)
#define FLASH_CTRL_REG22    (FLASH_CTRL_REG0 + 0x58)
#define FLASH_CTRL_REG23    (FLASH_CTRL_REG0 + 0x5c)
#define FLASH_CTRL_REG24    (FLASH_CTRL_REG0 + 0x60)

/* ECC sub-module */
#define FLASH_ECC_REG0      (AK88_VA_ECCCTRL)

#define ECC_CTL_DEC_RDY_EN      (1<<30)
#define ECC_CTL_ENC_RDY_EN      (1<<29)
#define ECC_CTL_END_EN          (1<<28)
#define ECC_CTL_RESULT_NO_OK    (1<<27)
#define ECC_CTL_NO_ERR          (1<<26)
#define ECC_CTL_DEC_RDY         (1<<24)
#define ECC_CTL_ENC_RDY         (1<<23)
#define ECC_CTL_MODE0           (0<<22)
#define ECC_CTL_MODE1           (1<<22)
#define ECC_CTL_NFC_EN          (1<<20)
#define ECC_CTL_BYTE_CFG(m)     ((m)<<7)	// bit [19:7]
#define ECC_CTL_END             (1<<6)
#define ECC_CTL_LITTLE_ENDIAN       (0<<5)
#define ECC_CTL_BIG_ENDIAN      (1<<5)
#define ECC_CTL_ADDR_CLR        (1<<4)
#define ECC_CTL_START           (1<<3)
#define ECC_CTL_DIR_READ        (0<<2)
#define ECC_CTL_DIR_WRITE       (1<<2)
#define ECC_CTL_DEC_EN          (1<<1)
#define ECC_CTL_ENC_EN          (1<<0)

/* *************** command sequece configuration define ************* */

/* command cycle's configure:
 *       CMD_END=X, ALE=0, CLE=1, CNT_EN=0,  (BIT[0:3])
 *       REN=0, WEN=1, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define COMMAND_CYCLES_CONF     0x64

/* address cycle's:
 *       CMD_END=X, ALE=1, CLE=0, CNT_EN=0,  (BIT[0:3])
 *       REN=0, WEN=1, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define ADDRESS_CYCLES_CONF     0x62

/*  read data cycle's:
 *       CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *       REN=1, WEN=0, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=1, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define READ_DATA_CONF          0x118

/*  write data cycle's:
 *       CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *       REN=0, WEN=1, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=1, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define WRITE_DATA_CONF         0x128

/* read command status's:(example: read ID, status, éš™æšºæ…æ“‚æŽ€éš¤•æ’…æ›‡…”èç’‡¢‡¬?)
 *       CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *       REN=1, WEN=0, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define READ_CMD_CONF           0x58
#define READ_INFO_CONF          0x58

/* wait the rising edge of R/B line:
 *       CMD_END=X, ALE=0, CLE=0, CNT_EN=0,  (BIT[0:3])
 *       REN=0, WEN=0, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=0, RBN_EN=1, CMD_WAIT=0,     (BIT[8:10])
 */
#define WAIT_JUMP_CONF          (1 << 9)

/* wait time (1024 ASIC cycles):
 *       CMD_END=X, ALE=0, CLE=0, CNT_EN=0,  (BIT[0:3])
 *       REN=0, WEN=0, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *       DAT_EN=0, RBN_EN=1, CMD_WAIT=0,     (BIT[8:10])
 */
#define DELAY_CNT_CONF          (1 << 10)

/* last command's bit0 set to 1 */
#define LAST_CMD_FLAG           (1 << 0)

/* ************** end command sequece configuration define ************/

/* ææ’–FLASH1~FLASH4(è–æ†­”éŠµ„çå…èœˆ‡æ’œˆ1):
 *       0: CE1 active;
 *       1: CE2 active;
 *       2: CE3 active;
 *       3: CE4 active;
 */
#define FLASH_CE_VALID      0	// (æ“–œèå–ç•¥…ã„è–‰éŠçš”€¥æ’–èŸâˆæ‹‡æ’“‚åœ„LASH)
#define SELECT_FLASH_CE     (1 << FLASH_CE_VALID)

/* ææ’–FLASH_WP1~FLASH_WP4(éŠçš”€Žœ•å—èâ…¡ˆË‰‡æ’œˆ1):
 *   BIT0~BIT3:
 *       BIT0: WP1 active;
 *       BIT1: WP2 active;
 *       BIT2: WP3 active;
 *       BIT3: WP4 active;
 */
#define SELECT_FLASH_WP     0x0

#define comb_ctrl_reg(ps, staff, ce_keep, ce, wp, go) (ps | (staff << 1) | (ce_keep << 9) \
		| (ce << 10) | (wp << 15) | (go << 30))

#define ECC_CHECK_NO_ERROR              (0x1<<26)
#define ECC_ERROR_REPAIR_CAN_NOT_TRUST  (0x1<<27)

#define DATA_ECC_CHECK_OK                           1
#define DATA_ECC_CHECK_ERROR                        0
#define DATA_ECC_ERROR_REPAIR_CAN_TRUST             2
#define DATA_ECC_ERROR_REPAIR_CAN_NOT_TRUST         3

#define REG32(_reg)	(*(volatile unsigned long *)(_reg))

/* ------------------------------------------------------------------------- */

static unsigned char akecc[128*2];
static unsigned char tmp_fs_buf[16*16];
static unsigned char tmp_data_buf[DATA_SIZE*2];
 
static void MMU_FlashDCache(void)  //flash dcache
{
	asm("AK_FlashDCache:\n" "mrc  p15,0,r15,c7,c10,3\n" "bne AK_FlashDCache");
}

static void MMU_InvalidateDCache(void)  //Invalidate dcache
{
	asm("AK_InvalidateDCache:\n" "mrc  p15,0,r15,c7,c14,3\n" "bne AK_InvalidateDCache");
}

static void ak880x_nand_lock_sharepin(void)
{
	/* set the share pin for nandflash */
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x74) &= (~(3 << 3));
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x74) |= (1 << 3);	//set 01 to enable the NFC

	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x78) |=
	    ((1 << 22) | (0xf << 16));

#if	defined(CONFIG_MACH_AK7801EVB) || defined(CONFIG_MACH_TA8)
	/* FIXME: use configurable nWP pin */
	/* set DGPIO 36 high, disable flash write protection */
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x94) &= (~(1 << 5));
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x98) |= (1 << 5);
#endif

}

/* NFC & HW port setup
 *
 * setting NFC, AC
 *
 * NFC share pin set
 *	Enable nandflash function pin		: 0x08000074 [4:3] = 0x01
 *	Share pin use nandflash :
 *	CE0,RE,WE,CLE,ALE,data0 -- data7	: 0x08000078 [16]=1 [17]=1 [18]=1
 *	R/B					: 0x08000078 [22]=1
 *	pull up and pull down			: use the defalut value
 */
void ak880x_nand_inithw(void)
{
	/* open the nand flash clk */
    printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_inithw() line 218\n");
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x0c) &= (~(1 << 13));
	ak880x_nand_lock_sharepin();

	/* FIXME: use configurable timing parameter */
	*(volatile unsigned int *)(AK88_VA_NFCTRL + 0x15C) = 0xF5BD1;
}

/* check cmd_done bit */
static int check_cmd_done(void)
{
	volatile unsigned long status;
    
	status = *(volatile unsigned long *)(FLASH_CTRL_REG22);

	if (status & 0x80000000)
		return 1;
	else
		return 0;
}

//col =1(for 512) or 2(for >512)
//row_cycle =3

static unsigned long *nf_send_addr(unsigned long *reg, unsigned int col,
				   unsigned int row, unsigned char col_cycle,
				   unsigned char row_cycle)
{
	unsigned char cycle, value;
    printk(KERN_INFO "zz ak880x-nfc.c nf_send_addr() line 246\n");
	ak880x_nand_lock_sharepin();

	// send column address 
	for (cycle = 0; cycle < col_cycle; cycle++) {
		value = (col >> (8 * cycle)) & 0xFF;
		REG32(reg++) = (value << 11) | ADDRESS_CYCLES_CONF;
	}

	// send row address 
	for (cycle = 0; cycle < row_cycle; cycle++) {
		value = (row >> (8 * cycle)) & 0xFF;
		REG32(reg++) = (value << 11) | ADDRESS_CYCLES_CONF;
	}

	return reg;
}

/* get nandflash chip id */
unsigned long ak880x_nand_get_chipid(unsigned char chip)
{
	unsigned long id = 0;
    printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_get_chipid() line 269\n");
	ak880x_nand_lock_sharepin();

	*(volatile unsigned long *)(FLASH_CTRL_REG22) = 0x00;

	/* send cmd */
	*(volatile unsigned long *)(FLASH_CTRL_REG0) =
	    (NFLASH_READ_ID << 11) | COMMAND_CYCLES_CONF;
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x04) =
	    (0x00 << 11) | ADDRESS_CYCLES_CONF;
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x08) = DELAY_CNT_CONF;	//wait > 10 ns < 30????(84M, 1 clock =12ns)
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x0C) =
	    (0x04 << 11) | READ_INFO_CONF | LAST_CMD_FLAG;

	/* excute operation, , enable power saving, CE# keep LOW wait R/B */
	*(volatile unsigned long *)(FLASH_CTRL_REG22) =
	    NCHIP_SELECT(chip) | DEFAULT_GO;

	while (!check_cmd_done()) ;

	/* read status */
	id = *(volatile unsigned long *)(FLASH_CTRL_REG20);

	printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_get_chipid line 292 nandid = 0x%lx\n", id); 

	return id;
}

/* software reset flash */
void ak880x_nand_reset(unsigned char chip, unsigned short wait_time)
{
	unsigned long *reg_addr = (unsigned long *)FLASH_CTRL_REG0;
    //printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_reset() line 297\n");
	ak880x_nand_lock_sharepin();

	// sta_clr = 0
	*(volatile unsigned long *)(FLASH_CTRL_REG22) = 0x00;

	*(volatile unsigned long *)(reg_addr++) =
	    ((0xFF << 11) | COMMAND_CYCLES_CONF);

	/* NOTE:
	 * when command is NOT right, the R/B rising never generate
	 * so can't wait R/B rising to judge data is ready
	 *
	 */
	wait_time &= 0xFFF;	// only low 12 bit is valid

	if (wait_time) {
		*(volatile unsigned long *)(reg_addr) =
		    ((wait_time << 11) | NCHIP_SELECT(chip) | LAST_CMD_FLAG);
	} else {
		*(volatile unsigned long *)(reg_addr) =
		    ((NAND_FLASH_DEFAULT_DELAY << 11) | NCHIP_SELECT(chip) |
		     LAST_CMD_FLAG);
	}

	/* excute operation, CE1, disable power saving, CE# keep LOW wait R/B */
	*(volatile unsigned long *)(FLASH_CTRL_REG22) =
	    (comb_ctrl_reg(0, 0, 1, SELECT_FLASH_CE, SELECT_FLASH_WP, 1));

	while (!check_cmd_done()) ;
}

/* set timing */
void ak880x_nand_settiming(unsigned long cmd_timing, unsigned long data_timing)
{
    //printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_settiming() line 335\n");
	ak880x_nand_lock_sharepin();
	if (cmd_timing)
		*(volatile unsigned long *)(FLASH_CTRL_REG23) = cmd_timing;

	if (data_timing)
		*(volatile unsigned long *)(FLASH_CTRL_REG24) = data_timing;
}

/* get nandflash chip status */
unsigned char ak880x_nand_get_status(unsigned char chip)
{
	unsigned long nand_status;
	unsigned long *reg_addr;
    //printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_get_status() line 349\n");
	ak880x_nand_lock_sharepin();

	do {
		nand_status = 0;
		reg_addr = (unsigned long *)FLASH_CTRL_REG0;

		*(volatile unsigned long *)(FLASH_CTRL_REG22) = 0x00;

		/* send cmd */
		*(volatile unsigned long *)(reg_addr++) =
		    ((NFLASH_READ_STATUS << 11) | COMMAND_CYCLES_CONF);
		*(volatile unsigned long *)(reg_addr++) = DELAY_CNT_CONF;
		*(volatile unsigned long *)(reg_addr++) = DELAY_CNT_CONF;
		*(volatile unsigned long *)(reg_addr) =
		    ((0x1 << 11) | READ_INFO_CONF | LAST_CMD_FLAG);

		/* excute operation, , enable power saving, CE# keep LOW wait R/B */
		*(volatile unsigned long *)(FLASH_CTRL_REG22) =
		    NCHIP_SELECT(chip) | DEFAULT_GO;

		while (!check_cmd_done()) ;

		/* read status */
		nand_status = *(volatile unsigned long *)(FLASH_CTRL_REG20);
		if ((nand_status & AK_NAND_STATUS_READY) !=
		    AK_NAND_STATUS_READY)
			mdelay(2);
		else
			break;
	} while (1);

	if ((nand_status & AK_NAND_STATUS_ERROR) == AK_NAND_STATUS_ERROR)
		printk("status error!\n");

	return nand_status & 0xFF;
}

/* erase nandflash block */
void ak880x_nand_eraseblock(unsigned char chip, unsigned int phypage)
{
    //printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_eraseblock() line 390\n");
	ak880x_nand_lock_sharepin();

	*(volatile unsigned long *)(FLASH_CTRL_REG22) = 0x00;

	/* send cmd */
	*(volatile unsigned long *)(FLASH_CTRL_REG0) =
	    (NFLASH_BLOCK_ERASE0 << 11) | COMMAND_CYCLES_CONF;

	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x04) =
	    ((phypage & 0xff) << 11) | ADDRESS_CYCLES_CONF;
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x08) =
	    (((phypage & 0xff00) >> 8) << 11) | ADDRESS_CYCLES_CONF;
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x0c) =
	    (((phypage & 0xff0000) >> 16) << 11) | ADDRESS_CYCLES_CONF;
	*(volatile unsigned long *)(FLASH_CTRL_REG0 + 0x10) =
	    (NFLASH_BLOCK_ERASE1 << 11) | COMMAND_CYCLES_CONF | LAST_CMD_FLAG;

	/* excute operation, , enable power saving, CE# keep LOW wait R/B */
	*(volatile unsigned long *)(FLASH_CTRL_REG22) =
	    NCHIP_SELECT(chip) | DEFAULT_GO;

	while (!check_cmd_done()) ;
}

void ak880x_nand_read_page(unsigned char chip, unsigned int rowAddr,
			   unsigned int columnAddr, unsigned char *buf,
			   unsigned int len, unsigned char large_page)
{
	unsigned long *reg_addr;
	unsigned long tmp;
	int read_times = len / 512 ? len / 512 : 1;
	int step;
	int i;

    printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_read_page() line 425\n");

	if (len >= 512)
		step = 512;
	else
		step = len;

	ak880x_nand_lock_sharepin();

	/*************** set program command *******************************/
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);	//clear the go stat reg
	reg_addr = (T_U32 *) FLASH_CTRL_REG0;

	//nandflash send command
	if (large_page) {
		REG32(reg_addr++) = (NFLASH_READ0 << 11) | COMMAND_CYCLES_CONF;
	} else {
		if (columnAddr < 256) {
			REG32(reg_addr++) =
			    (NFLASH_READ0 << 11) | COMMAND_CYCLES_CONF;
		} else if (columnAddr >= 512) {
			REG32(reg_addr++) =
			    (NFLASH_READ12 << 11) | COMMAND_CYCLES_CONF;
		} else {
			columnAddr -= 256;
			REG32(reg_addr++) =
			    (NFLASH_READ0_HALF << 11) | COMMAND_CYCLES_CONF;
		}
	}

	if (large_page)
		reg_addr = nf_send_addr(reg_addr, columnAddr, rowAddr, 2, 3);
	else {
		reg_addr = nf_send_addr(reg_addr, columnAddr, rowAddr, 1, 3);
	}

	if (large_page) {
		HAL_WRITE_UINT32((T_U32) (reg_addr++),
				 (NFLASH_READ1 << 11) | COMMAND_CYCLES_CONF);
	}

	HAL_WRITE_UINT32((T_U32) (reg_addr++), WAIT_JUMP_CONF | LAST_CMD_FLAG);	// wait R/B rising edge
	//HAL_WRITE_UINT32((T_U32)(reg_addr++), (0x1<<11) | DELAY_CNT_CONF | LAST_CMD_FLAG);  //DELAY
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, NCHIP_SELECT(chip) | DEFAULT_GO);
	while (!check_cmd_done()) ;
	

	// *******************************************************************
#if 0
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	MMU_InvalidateDCache();
#else  // in here
	MMU_CleanDCache();
	//printk("nand_read_page()/MMU_CleanDCache();\n");
#endif
 
//invalidate_dcache_range((unsigned int)buf,(unsigned int)(buf+len));
	MMU_InvalidateDCache();
#endif 

	for (i = 0; i < read_times; i++) {

		communicate_conf(NAND_FLASH, READ_BUF);

		//config ECC
		tmp = (0 | ECC_CTL_DIR_READ | ECC_CTL_ADDR_CLR
		       | ECC_CTL_BYTE_CFG(step) | ECC_CTL_NFC_EN |
		       ECC_CTL_MODE);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp | ECC_CTL_START);

		// config NAND interface
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);	//clear the go stat reg
		reg_addr = (T_U32 *) FLASH_CTRL_REG0;
		HAL_WRITE_UINT32((T_U32) reg_addr,
				 ((step -
				   1) << 11) | READ_DATA_CONF | LAST_CMD_FLAG);
		HAL_WRITE_UINT32(FLASH_CTRL_REG22,
				 NCHIP_SELECT(chip) | DEFAULT_GO);

		while (!check_cmd_done()) ;	// wait till all data is received

		// copy page data from L2 buffer
		rece_dat_cpu(buf + i * step, step, NAND_FLASH);

		// wait for ECC completion
		do {
			HAL_READ_UINT32(FLASH_ECC_REG0, tmp);
		} while (((tmp & ECC_CTL_END) == 0)
			 || !(tmp & ECC_CTL_DEC_RDY));
		HAL_WRITE_UINT32(FLASH_ECC_REG0, (tmp | ECC_CTL_END));
	}
}

void ak880x_nand_write_page(unsigned char chip, unsigned int rowAddr,
			    unsigned int columnAddr, const unsigned char *buf,
			    unsigned int len, unsigned char large_page)
{
	unsigned long *reg_addr;
	unsigned long tmp;
	int write_times = len / 512 ? len / 512 : 1;
	int step;
	int i;
    printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_write_page() line 528\n");
	if (len >= 512)
		step = 512;
	else
		step = len;

	ak880x_nand_lock_sharepin();

	/*set program command */
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);	//clear the go stat reg
	reg_addr = (T_U32 *) FLASH_CTRL_REG0;

	if (!large_page) {
		if (columnAddr < 256) {
			REG32(reg_addr++) =
			    (NFLASH_READ0 << 11) | COMMAND_CYCLES_CONF;
		} else if (columnAddr >= 512) {
			REG32(reg_addr++) =
			    (NFLASH_READ12 << 11) | COMMAND_CYCLES_CONF;
		} else {
			columnAddr -= 256;
			REG32(reg_addr++) =
			    (NFLASH_READ0_HALF << 11) | COMMAND_CYCLES_CONF;
		}
	}

	HAL_WRITE_UINT32((unsigned long)(reg_addr++),
			 (NFLASH_PAGE_PROGRAM0 << 11) | COMMAND_CYCLES_CONF);

	if (large_page) {
#if 1
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 ((columnAddr & 0xff) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 (((columnAddr & 0xff00) >> 8) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 ((rowAddr & 0xff) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 (((rowAddr & 0xff00) >> 8) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 (((rowAddr & 0xff0000) >> 16) << 11) |
				 ADDRESS_CYCLES_CONF | LAST_CMD_FLAG);
#else
		reg_addr = nf_send_addr(reg_addr, columnAddr, rowAddr, 2, 2);
#endif
	} else {
#if 1
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 ((columnAddr & 0xff) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 ((rowAddr & 0xff) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 (((rowAddr & 0xff00) >> 8) << 11) |
				 ADDRESS_CYCLES_CONF);
		HAL_WRITE_UINT32((unsigned long)(reg_addr++),
				 (((rowAddr & 0xff0000) >> 16) << 11) |
				 ADDRESS_CYCLES_CONF | LAST_CMD_FLAG);
#else
		reg_addr = nf_send_addr(reg_addr, columnAddr, rowAddr, 1, 3);
#endif
	}
	HAL_WRITE_UINT32(FLASH_CTRL_REG22,
			 (SELECT_FLASH_CE << 10) | DEFAULT_GO);

	while (!check_cmd_done()) ;

#if 0
	//flash_dcache_range((unsigned int)buf,(unsigned int)(buf+len));
	MMU_FlashDCache();
#endif

	//printk("write_page()/MMU_FlashDCache();\n");

	for (i = 0; i < write_times; i++) {

		// init L2 buffer
		communicate_conf(NAND_FLASH, WRITE_BUF);

		// config ECC module
		tmp = (0 | ECC_CTL_DIR_WRITE | ECC_CTL_ADDR_CLR
		       | ECC_CTL_BYTE_CFG(step) | ECC_CTL_NFC_EN |
		       ECC_CTL_MODE);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp);
		/* start */
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp | ECC_CTL_START);

		// config NAND interface
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);	//clear the go stat reg
		HAL_WRITE_UINT32(FLASH_CTRL_REG0,
				 ((step -
				   1) << 11) | WRITE_DATA_CONF | LAST_CMD_FLAG);
		HAL_WRITE_UINT32(FLASH_CTRL_REG22,
				 NCHIP_SELECT(chip) | DEFAULT_GO);

		// copy page data to L2 buffer
#ifdef CONFIG_MTD_NAND_DMA_MODE
		prepare_dat_send_dma((T_U8 *) buf, step, NAND_FLASH);
#else
		prepare_dat_send_cpu(buf + i * step, step, NAND_FLASH);
#endif
		while (!check_cmd_done()) ;

		// wait for ECC complete        
		do {
			HAL_READ_UINT32(FLASH_ECC_REG0, tmp);
		} while (((tmp & ECC_CTL_END) == 0)
			 || !(tmp & ECC_CTL_ENC_RDY));
		HAL_WRITE_UINT32(FLASH_ECC_REG0, (tmp | ECC_CTL_END));

	}

	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);	//clear the go stat reg
	HAL_WRITE_UINT32(FLASH_CTRL_REG0,
			 (NFLASH_PAGE_PROGRAM1 << 11) | COMMAND_CYCLES_CONF);
	HAL_WRITE_UINT32(FLASH_CTRL_REG0 + 0x4, WAIT_JUMP_CONF | LAST_CMD_FLAG);	// wait R/B rising edge
	//HAL_WRITE_UINT32(FLASH_CTRL_REG0+0x4,  (0x100<< 11) | DELAY_CNT_CONF | LAST_CMD_FLAG);
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, NCHIP_SELECT(chip) | DEFAULT_GO);

	//printk("wait cmd done...\n");
	while (!check_cmd_done()) ;
}

unsigned char ak880x_nand_ecc_stat(unsigned long stat)
{
    printk(KERN_INFO "zz ak880x-nfc.c ak880x_nand_ecc_stat() line 658\n");
	ak880x_nand_lock_sharepin();

	if ((stat & ECC_CHECK_NO_ERROR) == ECC_CHECK_NO_ERROR) {

		*(volatile unsigned long *)(FLASH_ECC_REG0) =
		    (stat | ECC_CHECK_NO_ERROR);
		stat = *(volatile unsigned long *)(FLASH_ECC_REG0);

		return DATA_ECC_CHECK_OK;

	} else if ((stat & ECC_ERROR_REPAIR_CAN_NOT_TRUST) ==
		   ECC_ERROR_REPAIR_CAN_NOT_TRUST) {

		*(volatile unsigned long *)(FLASH_ECC_REG0) =
		    stat | ECC_ERROR_REPAIR_CAN_NOT_TRUST;
		stat = *(volatile unsigned long *)(FLASH_ECC_REG0);

		return DATA_ECC_ERROR_REPAIR_CAN_NOT_TRUST;
	} else
		return DATA_ECC_ERROR_REPAIR_CAN_TRUST;
}


/******************************************************************************
	nand read/write with ecc    
	add at 08/05/2010 
	by Lynn Liu
******************************************************************************/

//mach/map.h
//#define AK880X_VA_SUBCTRL       AK880X_ADDR(0x00200000)
//#define AK880X_PA_SUBCTRL       (0x20000000)
//#define AK880X_VA_ECCCTRL       (AK880X_VA_SUBCTRL + 0x2B000)

//#define FLASH_ECC_POSITION_REG0     (0X2002B004)
//#define FLASH_ECC_POSITION_REG1     (0X2002B004+4)
//#define FLASH_ECC_POSITION_REG2     (0X2002B004+8)
//#define FLASH_ECC_POSITION_REG3     (0X2002B004+12)
//#define FLASH_ECC_POSITION_REG4     (0X2002B004+16)
//#define FLASH_ECC_POSITION_REG5     (0X2002B004+20)
//#define FLASH_ECC_POSITION_REG6     (0X2002B004+24)
//#define FLASH_ECC_POSITION_REG7     (0X2002B004+28)
//#define FLASH_ECC_POSITION_REG8     (0X2002B004+32)

//#define DATA_SIZE 4096

//#define FLASH_ECC_POSITION_REG0     (AK880X_VA_ECCCTRL + 0X04)

/*********************************************************************
Function:       NAND_Read_ChipStatus
Description:    read the Nand flash chip's status.
Input:
Output:
Return:         chip status
Author:       Zou Tianxiang
Date:         2007.7.6
**********************************************************************/

extern int print_32(unsigned char *buf);

unsigned char NAND_Read_ChipStatus(void)
{
	unsigned int nand_status;
	unsigned int *reg_addr;
    printk(KERN_INFO "zz ak880x-nfc.c NAND_Read_ChipStatus() line 723\n");
	nand_status = 0;
	reg_addr = (unsigned int *) FLASH_CTRL_REG0;

	ak880x_nand_lock_sharepin();

	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0x00);

	/* send cmd */
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (NFLASH_READ_STATUS << 11) | COMMAND_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), DELAY_CNT_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), DELAY_CNT_CONF);
	HAL_WRITE_UINT32((unsigned int)reg_addr, (0x1 << 11) | READ_INFO_CONF | LAST_CMD_FLAG);

	// excute operation, , enable power saving, CE# keep LOW wait R/B
	HAL_WRITE_UINT32( FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO );
	// wait end & read data (data at where)
	while(!check_cmd_done());

	// read status
	HAL_READ_UINT32(FLASH_CTRL_REG20, nand_status);
	return (unsigned char)(nand_status & 0xff);

}

/*********************************************************************
Function:       TNAND_ECC_POS_HIGHT
Description:    NAND ECC DATA REPAIR
Input:
Output:
Return:         1: Repair ok   0: Repair failed
Author:             Zou Tianxiang
Date:               2007.7.6
 **********************************************************************/
static unsigned int TNAND_ECC_POS_HIGHT(unsigned int position_info, unsigned char *buf, unsigned char *fs_buf)
{
	unsigned int position = (position_info>>9) & 0x3ff;
	unsigned int offset;
	//unsigned int bit_index;
	unsigned int byte_index;
	unsigned char  correct;
	unsigned int shift;
    printk(KERN_INFO "zz ak880x-nfc.c TNAND_ECC_POS_HIGHT() line 765\n");
	ak880x_nand_lock_sharepin();

	if (position)
	{// error occurs
		//        printk("ECC error position reg = 0x%x\n",  position_info);

		// There are chances that two errors fall into contiguous bits. Then position will have two bits set.
		// For each bit of position
		for (shift=0; shift<10; shift++)
		{
			if (0 == (position & (1<<shift)))
			{
				// no error in this block
				continue;
			}

			switch (position & (1<<shift))
			{
				case (1<<0):
				case (1<<1):
					// block1
					byte_index = NAND_SPARE_SIZE - 128;
					break;

				case (1<<2):
				case (1<<3):
					// block2
					byte_index = NAND_SPARE_SIZE;
					break;

				case (1<<4):
				case (1<<5):
					// block3
					byte_index = NAND_SPARE_SIZE + 128;
					break;

				case (1<<6):
				case (1<<7):
					// block4
					byte_index = NAND_SPARE_SIZE + 128*2;
					break;

				case (1<<8):
				case (1<<9):
					// block5
					byte_index = NAND_SPARE_SIZE + 128*3;
					break;
				default:
					printk("wrong position = 0x%x\n", position);
					while(1);
					break;
			}

			offset = 2 * (position_info & 0x1ff) + (((1<<shift)&0x2aa)? 1:0);
#ifndef ECC_MODE1
			offset -= 4;
			if (offset & 0x8000)
			{
				byte_index -= 1;
				offset += 8;
			}
#endif

			byte_index += offset>>3;
			correct = 1 << (7 - (offset&0x7));
			//print("byte index = %d, correct factor = 0x%x\n", byte_index, correct);

			if (byte_index < NAND_DATA_SIZE)
			{// error occurs in data
				buf[byte_index] ^= correct;
			}
			else if (byte_index < NAND_EFFECTIVE_SIZE)
			{// error occurs in file system info
				fs_buf[byte_index-NAND_DATA_SIZE] ^= correct;
			}
			else
			{// error occurs in parity data
				// nothing to do            
			}
		}
	}

	return 1;
}


/*********************************************************************
Function:       NAND_ECC_Repair
Description:    NAND ECC REPAIR
Input:        PERIPHERAL_TYPE dev_type
Output:
Return:         ecc check:  data  ok            return      1
ecc check:  data  error can't repair            return      0
ecc check,  data will repair, can trust         return      2
ecc check,  data will repair, can't trust       return      3

Author:       Zou Tianxiang
Date:         2007.7.6
**********************************************************************/

static unsigned char NAND_ECC_Data_Check(unsigned int stat)
{
	//unsigned int position_star;
	//unsigned int position_end;
    printk(KERN_INFO "zz ak880x-nfc.c NAND_ECC_Data_Check() line 869\n");
	ak880x_nand_lock_sharepin();

	if( (stat & ECC_CHECK_NO_ERROR) == ECC_CHECK_NO_ERROR )
	{
		stat |= ECC_CHECK_NO_ERROR;
		HAL_WRITE_UINT32(FLASH_ECC_REG0 ,stat);
		HAL_READ_UINT32(FLASH_ECC_REG0 ,stat);
		return DATA_ECC_CHECK_OK;
	}
	else if( (stat & ECC_ERROR_REPAIR_CAN_NOT_TRUST) == ECC_ERROR_REPAIR_CAN_NOT_TRUST)
	{
		stat |= ECC_ERROR_REPAIR_CAN_NOT_TRUST;
		HAL_WRITE_UINT32(FLASH_ECC_REG0 ,stat);
		HAL_READ_UINT32(FLASH_ECC_REG0 ,stat);
		return DATA_ECC_ERROR_REPAIR_CAN_NOT_TRUST;
	}
	else
	{
		return DATA_ECC_ERROR_REPAIR_CAN_TRUST;
	}
}


/*********************************************************************
Function:       NAND_ECC_Data_Repair
Description:    NAND ECC DATA REPAIR
Input:
Output:
Return:         1: Repair ok   0: Repair failed
Author:       Zou Tianxiang
Date:         2007.7.6
 **********************************************************************/
void NAND_ECC_Data_Repair(unsigned char *buf, unsigned char *fs_buf)
{
	unsigned int position_info;
	//unsigned int wrong_info;
	unsigned int error_count;
    printk(KERN_INFO "zz ak880x-nfc.c NAND_ECC_Data_Repair() line 907\n");
	ak880x_nand_lock_sharepin();

	for (error_count=0; error_count<MAX_ERROR_CNT; error_count++)
	{
		HAL_READ_UINT32(FLASH_ECC_POSITION_REG0 + error_count*4 ,position_info);

		TNAND_ECC_POS_HIGHT(position_info, buf, fs_buf);
	}
}

void Nand_WritePhyPage_ECC_2K(unsigned int PhyPage,const unsigned char *buf, const unsigned char *fs_buf)
{
	unsigned int *reg_addr;
	unsigned int status;
	unsigned int tmp;
	unsigned int i;
	unsigned int max_read_count = 512 ;
	//unsigned int read_loop = nand_flash_info_tab[cur_flash_info].data_size/max_read_count ;
	//unsigned int read_loop = DATA_SIZE/max_read_count ;
	unsigned int read_loop = NAND_MAX_PAGESIZE/max_read_count ;  //4096/512
    printk(KERN_INFO "zz ak880x-nfc.c Nand_WritePhyPage_ECC_2K() line 929\n");
//MMU_Clean_All_DCache();
MMU_FlashDCache();

	ak880x_nand_lock_sharepin();

	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
	reg_addr = (unsigned int *) FLASH_CTRL_REG0;
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (NFLASH_PAGE_PROGRAM0 << 11) | COMMAND_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), ((0 & 0xff) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((0 & 0xff00)>>8) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), ((PhyPage & 0xff) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((PhyPage & 0xff00)>>8) << 11) | ADDRESS_CYCLES_CONF);

	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((PhyPage & 0xff0000)>>16) << 11) | ADDRESS_CYCLES_CONF|LAST_CMD_FLAG);
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);
	while(!check_cmd_done());

	// write 4*page bytes
	for( i = 0; i < read_loop; i++ )
	{
		// init l2 buffer
		communicate_conf(NAND_FLASH, WRITE_BUF);

 		//==========================
		// config ECC module
		tmp = ( ECC_CTL_ENC_EN | ECC_CTL_DIR_WRITE | ECC_CTL_ADDR_CLR
				| ECC_CTL_BYTE_CFG(NAND_EFFECTIVE_SIZE) | ECC_CTL_NFC_EN | ECC_CTL_MODE);
		HAL_WRITE_UINT32(FLASH_ECC_REG0,  tmp);
		HAL_WRITE_UINT32(FLASH_ECC_REG0,  tmp|ECC_CTL_START); 
		
 		//=============================
		// config NAND interface
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
		HAL_WRITE_UINT32(FLASH_CTRL_REG0, ((NAND_PAGE_SIZE-1) << 11) | WRITE_DATA_CONF | LAST_CMD_FLAG);
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);

		//==========================
		// copy page data to L2 buffer
#ifdef L2_DMA_MODE
		prepare_dat_send_dma(buf+i*NAND_DATA_SIZE, NAND_DATA_SIZE, NAND_FLASH);
		prepare_dat_send_dma(fs_buf+i*NAND_FS_SIZE, NAND_FS_SIZE, NAND_FLASH);
#else
		prepare_dat_send_cpu(buf+i*NAND_DATA_SIZE, NAND_DATA_SIZE, NAND_FLASH);
		prepare_dat_send_cpu(fs_buf+i*NAND_FS_SIZE, NAND_FS_SIZE, NAND_FLASH);
#endif
		//=============================

 		// wait for ECC complete
		do
		{
			HAL_READ_UINT32(FLASH_ECC_REG0,tmp);
		} while((tmp  & ECC_CTL_END) != ECC_CTL_END);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, (tmp | ECC_CTL_END));

 		//while(!check_cmd_done());
	}

	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
	HAL_WRITE_UINT32(FLASH_CTRL_REG0, (NFLASH_PAGE_PROGRAM1 << 11) | COMMAND_CYCLES_CONF);
	//HAL_WRITE_UINT32(FLASH_CTRL_REG0+0x4,  (0x1<< 11) | DELAY_CNT_CONF | LAST_CMD_FLAG);
	//by ljh, wait rb ready
	HAL_WRITE_UINT32(FLASH_CTRL_REG0+0x4,  WAIT_JUMP_CONF|LAST_CMD_FLAG);


	HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);
	while(!check_cmd_done());

	do
	{
		status = NAND_Read_ChipStatus();
	}   while((status & NAND_STATUS_READY) != NAND_STATUS_READY);

	if( (status & AK_NAND_STATUS_ERROR) == AK_NAND_STATUS_ERROR)
	{
		printk("Program 2K ecc error!\n");
	}
}


unsigned char Nand_ReadPhyPage_ECC_2K(unsigned int PhyPage, unsigned char *buf, unsigned char *fs_buf)
{

	unsigned int *reg_addr;
	unsigned int tmp;
	unsigned char  ecc_stat;
	unsigned int i;
	unsigned int max_read_count = 512 ;
	//unsigned int read_loop = nand_flash_info_tab[cur_flash_info].data_size/max_read_count ;
 	unsigned int read_loop = NAND_MAX_PAGESIZE/max_read_count ;  //4096/512
	int ecc_all_ff=1;
	int j=0,j0=0;
    printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() line 1017\n");
MMU_InvalidateDCache();
	
	ak880x_nand_lock_sharepin();

	memset(akecc,0x00,sizeof(akecc));
  
	//set program command 
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
	reg_addr = (unsigned int *) FLASH_CTRL_REG0;
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (NFLASH_READ0 << 11) | COMMAND_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), ((0 & 0xff) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((0 & 0xff00)>>8) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), ((PhyPage & 0xff) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((PhyPage & 0xff00)>>8) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (((PhyPage & 0xff0000)>>16) << 11) | ADDRESS_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), (NFLASH_READ1 << 11) | COMMAND_CYCLES_CONF);
	HAL_WRITE_UINT32((unsigned int)(reg_addr++), WAIT_JUMP_CONF | LAST_CMD_FLAG);  //DELAY
	HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);
	while(!check_cmd_done()) ;

	for ( i = 0; i < read_loop; i++ )
	{
 
		// init L2 buffer
		communicate_conf(NAND_FLASH, READ_BUF);
 
		//==========================
		//config ECC
		tmp = ECC_CTL_DEC_EN | ECC_CTL_DIR_READ | ECC_CTL_ADDR_CLR
			| ECC_CTL_BYTE_CFG(NAND_EFFECTIVE_SIZE) | ECC_CTL_NFC_EN | ECC_CTL_MODE
			| ECC_CTL_NO_ERR | ECC_CTL_RESULT_NO_OK;    // reset ecc result status bits
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp|ECC_CTL_START);  
 
		//==========================
		// config NAND interface
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
		reg_addr = (unsigned int *) FLASH_CTRL_REG0;
		HAL_WRITE_UINT32((unsigned int)reg_addr, ((NAND_PAGE_SIZE-1) << 11) | READ_DATA_CONF | LAST_CMD_FLAG);
		HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);
 
		//==========================
		// copy page data from L2 buffer
#ifdef L2_DMA_MODE
        //printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() 1061 L2_DMA_MODE\n");
		rece_data_dma(buf+i*NAND_DATA_SIZE, NAND_DATA_SIZE, NAND_FLASH);
		while(!check_cmd_done()); // wait till all data is received
		rece_data_dma(fs_buf+i*NAND_FS_SIZE, NAND_FS_SIZE, NAND_FLASH);
		rece_dat_dma(akecc+i*NAND_PARITY_SIZE, NAND_PARITY_SIZE, NAND_FLASH);
#else
        printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() 1067 L2_CPU_MODE\n");
		rece_dat_cpu(buf+i*NAND_DATA_SIZE, NAND_DATA_SIZE, NAND_FLASH);
		while(!check_cmd_done()); // wait till all data is received
		/*
	    for (j=0; j<32; j++)
        {
            printk(KERN_INFO "data[%d] = 0x%x", j, buf[i*NAND_DATA_SIZE+j]);
        }
        */
		rece_dat_cpu(fs_buf+i*NAND_FS_SIZE, NAND_FS_SIZE, NAND_FLASH);
        /*
        printk(KERN_INFO "oob[] 0x%x 0x%x 0x%x 0x%x\n", 
               fs_buf[i*NAND_FS_SIZE],
               fs_buf[i*NAND_FS_SIZE+1],
               fs_buf[i*NAND_FS_SIZE+2],
               fs_buf[i*NAND_FS_SIZE+3]);
               */
        printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() 1086 NAND_PARITY_SIZE=%d\n",
               NAND_PARITY_SIZE);
		rece_dat_cpu(akecc+i*NAND_PARITY_SIZE, NAND_PARITY_SIZE, NAND_FLASH);

#endif

		//if read ecc result is all 0xff,it isn't a valid ecc,
		//it only means this page has beed erased, no written any words,or not written with ecc
	
		ecc_all_ff=1;
		j0=i*NAND_PARITY_SIZE;
		for(j=0;j<NAND_PARITY_SIZE;j++)	{
 			if(akecc[j0+j]!=0xff) {
 				ecc_all_ff=0;	//isn't all 0xff, it is a valid ecc
				break;
                        }
		}
		
		//invalid ecc, means this page is empty,it is an erased page, or not written with ecc
		if(ecc_all_ff==1) {	
			//printk("erased page,akecc=");
			//print_32(akecc);
			continue;
		}
		else{
			//printk("valid ecc,akecc=");
			//print_32(akecc);
		}

	
		//==========================

		// wait for ECC complete
		do
		{
			HAL_READ_UINT32(FLASH_ECC_REG0,tmp);
		}   while((tmp  & ECC_CTL_END) == 0);
		HAL_WRITE_UINT32(FLASH_ECC_REG0, (tmp | ECC_CTL_END));
        
        //buf[i*NAND_DATA_SIZE] = (buf[i*NAND_DATA_SIZE])^0b00001111;

 		ecc_stat = NAND_ECC_Data_Check(tmp);
 
		switch(ecc_stat)
		{
			case DATA_ECC_CHECK_OK:
				break;

			case DATA_ECC_ERROR_REPAIR_CAN_TRUST:
				printk("page %d: ecc error occurs and can be corrected\n", PhyPage);
				NAND_ECC_Data_Repair(buf+i*NAND_DATA_SIZE, (fs_buf+i*NAND_FS_SIZE));
				break;

			case DATA_ECC_ERROR_REPAIR_CAN_NOT_TRUST:
 				printk("page %d: ecc error occurs and cannot be corrected\n", PhyPage);
				printk("akecc=");
				print_32(akecc);

				return 0;
 
			default :
				printk("No ECC State!\n");
				break;
		}
	}
  
	return 1;
}

/*

    static T_VOID cmd_go(T_U32 Chip)
    {
        volatile T_U32 status;
    
        status = REG32(FLASH_CTRL_REG22);
        status &= ~( 1<<31 | 1<<10 | 1<<11); //remove CE flag
        status |= NCHIP_SELECT(Chip) | ((1 << 30)|(1<<9));
        REG32(FLASH_CTRL_REG22) = status;
    }

unsigned char Nand_ReadPhyPage_ECC_2K(unsigned int PhyPage, unsigned char *buf, unsigned char *fs_buf)
{
    
        unsigned int *reg_addr;
        unsigned int tmp;
        unsigned int i;
        unsigned int max_read_count = 512 ;
        //unsigned int read_loop = nand_flash_info_tab[cur_flash_info].data_size/max_read_count ;
        unsigned int read_loop = NAND_MAX_PAGESIZE/max_read_count ;  //4096/512
        printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() line 1152\n");
        MMU_InvalidateDCache();
        
        ak880x_nand_lock_sharepin();
        
        memset(akecc,0x00,sizeof(akecc));
      
        REG32(FLASH_CTRL_REG22) &= ~(1<<14);
        reg_addr = (unsigned int *) FLASH_CTRL_REG0;
         //nandflash send command
        REG32(reg_addr++) = (0x00 << 11) | COMMAND_CYCLES_CONF;
        reg_addr = nf_send_addr(reg_addr, 0, PhyPage, 2, 3);
        //nandflash read2 command,this is a wait command
        if (0 != 2)
        {
            REG32(reg_addr++) = (0x30 << 11) | COMMAND_CYCLES_CONF |(1 << 10);
        }
    
        REG32(reg_addr) = WAIT_JUMP_CONF | LAST_CMD_FLAG;// wait R/B rising edge

        cmd_go(0);
        while (!check_cmd_done());

        for ( i = 0; i < read_loop; i++ )
        {
     
            // init L2 buffer
            communicate_conf(NAND_FLASH, READ_BUF);
     
            //==========================
            //config ECC
            tmp = (ECC_CTL_DIR_READ | ECC_CTL_ADDR_CLR  \
                      | ECC_CTL_BYTE_CFG(516) | ECC_CTL_NFC_EN | ECC_CTL_MODE);
            HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp);
            HAL_WRITE_UINT32(FLASH_ECC_REG0, tmp|ECC_CTL_START);
            REG32(FLASH_CTRL_REG0) = ((516 - 1) << 11) | READ_DATA_CONF | LAST_CMD_FLAG;
            cmd_go(0);
    
            //==========================
            // config NAND interface
            HAL_WRITE_UINT32(FLASH_CTRL_REG22, 0);      //clear the go stat reg
            reg_addr = (unsigned int *) FLASH_CTRL_REG0;
            HAL_WRITE_UINT32((unsigned int)reg_addr, ((NAND_PAGE_SIZE-1) << 11) | READ_DATA_CONF | LAST_CMD_FLAG);
            HAL_WRITE_UINT32(FLASH_CTRL_REG22, (SELECT_FLASH_CE<<10)|DEFAULT_GO);
     
            //==========================
            // copy page data from L2 buffer
            printk(KERN_INFO "zz ak880x-nfc.c Nand_ReadPhyPage_ECC_2K() 1232 L2_CPU_MODE\n");
            rece_dat_cpu(buf+i*NAND_DATA_SIZE, NAND_DATA_SIZE, NAND_FLASH);
            while(!check_cmd_done()); // wait till all data is received
         
            rece_dat_cpu(fs_buf+i*NAND_FS_SIZE, NAND_FS_SIZE, NAND_FLASH);
            rece_dat_cpu(akecc+i*NAND_PARITY_SIZE, NAND_PARITY_SIZE, NAND_FLASH);
        }
        return 1;
}
*/
#define AK880X_OOB_OFFSET      (2)
//#define AK880X_OOB_OFFSET      (0)
#define AK880X_FS_LEN          (3*8)

static unsigned char data_buf[DATA_SIZE+OOB_SIZE];
static unsigned char fs_buf[AK880X_FS_LEN] ;

void nand_write_page_ecc( int page, const unsigned char *buf,const unsigned char *oobbuf)
{
/**
	yaffs tags len = { 28, //with tags ECC
			   16, //no tags ECC	
			 }
**/

    printk(KERN_INFO "zz ak880x-nfc.c nand_write_page_ecc() line 1290\n");
	if(oobbuf==NULL)
	{
		memset(tmp_fs_buf,0xff,sizeof(tmp_fs_buf));
		oobbuf=&tmp_fs_buf[0];
	}

	Nand_WritePhyPage_ECC_2K( page, buf, oobbuf+AK880X_OOB_OFFSET);

	return;
}

 
void nand_read_page_ecc( int page, unsigned char * buf,unsigned char *oobbuf)
{
    printk(KERN_INFO "zz ak880x-nfc.c nand_read_page_ecc() line 1305\n");
	if(oobbuf==NULL)
	{
		memset(tmp_fs_buf,0xff,sizeof(tmp_fs_buf));
		oobbuf=&tmp_fs_buf[0];
	}

	Nand_ReadPhyPage_ECC_2K( page, buf, oobbuf+AK880X_OOB_OFFSET);
 
	return;
}

//read data, does'n transfer oob
void nand_read_page_data( int page, unsigned char * data_buf)
{
    printk(KERN_INFO "zz ak880x-nfc.c nand_read_page_data() line 1320\n");
 	memset(tmp_fs_buf,0xff,sizeof(tmp_fs_buf));
  
	Nand_ReadPhyPage_ECC_2K( page, data_buf, tmp_fs_buf+AK880X_OOB_OFFSET);

	return;
}

//read oob,doesn't transfer data
void nand_read_page_oob( int page, unsigned char * oob_buf)
{
    printk(KERN_INFO "zz ak880x-nfc.c nand_read_page_oob() line 1331\n");
 	memset(tmp_data_buf,0xff,sizeof(tmp_data_buf));
  
	Nand_ReadPhyPage_ECC_2K( page, tmp_data_buf, oob_buf+AK880X_OOB_OFFSET);

	return;
}


void nand_read_oob_ecc( int page, unsigned char *oobbuf)
{  	
    printk(KERN_INFO "zz ak880x-nfc.c nand_read_oob_ecc() line 1342\n");
 	memset(data_buf,0xff,sizeof(data_buf));
 	Nand_ReadPhyPage_ECC_2K( page, data_buf, oobbuf + AK880X_OOB_OFFSET);
    return;
}


 
//only write some bytes
void nand_write_page_ecc_part( int page,const unsigned char *buf,int len )
{
    printk(KERN_INFO "zz ak880x-nfc.c nand_write_page_ecc_part() line 1353\n");
	memset(data_buf,0xff,sizeof(data_buf));
	memset(fs_buf,0xff,sizeof(fs_buf));
  
	len = len<DATA_SIZE? len:DATA_SIZE;

	memcpy(data_buf,buf,len);

	Nand_WritePhyPage_ECC_2K( page,data_buf, fs_buf) ;
	
	return ;
}
 

//only read some bytes
void nand_read_page_ecc_part( int page, unsigned char * buf,int len )
{ 
    printk(KERN_INFO "zz ak880x-nfc.c nand_read_page_ecc_part() line 1370\n");
	memset(data_buf,0xff,sizeof(data_buf));
	memset(fs_buf,0xff,sizeof(fs_buf));
 
	Nand_ReadPhyPage_ECC_2K( page, data_buf, fs_buf) ;

	len = len<DATA_SIZE? len:DATA_SIZE;

	memcpy(buf,data_buf,len);
  
	return ;
}

