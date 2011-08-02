/**
 * @filename wrap_nand.h
 * @brief AK880x nandflash driver
 *
 * This file wrap the nand flash driver in nand_control.c file.
 * Copyright (C) 2010 Anyka (GuangZhou) Software Technology Co., Ltd.
 * @author zhangzheng
 * @modify 
 * @date 2010-10-10
 * @version 1.0
 * @ref
 */


#ifndef _AK_NAND_WRAP_H_
#define _AK_NAND_WRAP_H_

#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <mach-anyka/anyka_types.h>
#include <mach-anyka/nand_list.h>
#include <mach-anyka/fha.h>
#include <mach-anyka/fha_asa.h>
#include "arch_nand.h"

int init_fha_lib(void);
#if 0
void release_fha_lib(void);
#endif

void ak_dump_datactrl_sparectrl(T_PNAND_ECC_STRU data_ctrl, T_PNAND_ECC_STRU spare_ctrl);	//lgh
void Nand_Config_Data_Spare(T_PNAND_ECC_STRU data_ctrl, T_U8* data, T_U32 data_len, ECC_TYPE ecc_type,
							    T_PNAND_ECC_STRU spare_ctrl, T_U8* spare, T_U32 spare_len);	//lgh

void ak_nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len);


void ak_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len);


void ak_nand_select_chip(struct mtd_info *mtd, int chip);


u_char ak_nand_read_byte(struct mtd_info *mtd);

int ak_nand_scan_bbt(struct mtd_info *mtd);

int ak_nand_init_size(struct mtd_info *mtd, struct nand_chip *chip, u8 *id_data);

int ak_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				                   uint8_t *buf, int page);


void ak_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				                     const uint8_t *buf);


int ak_nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			                    uint8_t *buf, int pag);


void ak_nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				                  const uint8_t *buf);


int ak_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			                   int page, int sndcmd);


int ak_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			                   int page);

T_U32 nand_erase_callback(T_U32 chip_num, T_U32 startpage);

T_U32 nand_read_callback(T_U32 chip_num, T_U32 page_num, T_U8 *data, 
           T_U32 data_len, T_U8 *oob, T_U32 oob_len, T_U32 eDataType);

T_U32 nand_write_callback(T_U32 chip_num, T_U32 page_num, const T_U8 *data, 
           T_U32 data_len, T_U8 *oob, T_U32 oob_len, T_U32 eDataType);

T_U32 nand_read_bytes_callback(T_U32 nChip, T_U32 rowAddr, T_U32 columnAddr, T_U8 *pData, T_U32 nDataLen);

void *ram_alloc(T_U32 size);

void *ram_free(void *point);

T_S32 print_callback(T_pCSTR s, ...);

T_U32 init_globe_para(void);

void free_globe_para(void);

#ifdef CONFIG_MTD_NAND_TEST   
void nand_balance_test_init(void);

void nand_balance_test_exit(void);
#endif

T_U32 ak_mount_partitions(void);

int ak_nand_block_isbad(struct mtd_info *mtd, loff_t offs, int getchip);

int ak_nand_block_markbad(struct mtd_info *mtd, loff_t offs);

T_U32 try_nand_para(T_U32 chip_num, T_U32 page_num, T_U8 *data, 
           T_U32 data_len, T_U8 *oob, T_U32 oob_len, T_Nandflash_Add *p_add);


#if 1
void zz_test_nand(void);
void erase_all_flash(void);
#endif

#endif
