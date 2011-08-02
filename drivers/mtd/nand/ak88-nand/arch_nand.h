/**@file arch_nand.h
 * @brief AK880x nand controller
 *
 * This file describe how to control the AK880x nandflash driver.
 * Copyright (C) 2006 Anyka (GuangZhou) Software Technology Co., Ltd.
 * @author yiruoxiang, jiangdihui
 * @date 2007-1-10
 * @version 1.0
 */
#ifndef __ARCH_NAND_H__
#define __ARCH_NAND_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup NandFlash Nandflash group
 *  @ingroup Drv_Lib
 */
/*@{*/
#define NFC_FIFO_SIZE			512
#define NFC_LOG_SPARE_SIZE		16

#define NAND_512B_PAGE      0   // 512 bytes per page
#define NAND_2K_PAGE        1
#define NAND_4K_PAGE        2
#define NAND_8K_PAGE        3

#define NFC_SUPPORT_CHIPNUM    (4)
#define MTD_PART_NAME_LEN      (4)

struct partitions
{
	char name[MTD_PART_NAME_LEN]; 		   
	unsigned long long size;			           
	unsigned long long offset;         
	unsigned int mask_flags;		       
}__attribute__((packed));

typedef enum
{
    ECC_4BIT_P512B  = 0,    /*4 bit ecc requirement per 512 bytes*/
    ECC_8BIT_P512B  = 1,    //8 bit ecc requirement per 512 bytes
    ECC_12BIT_P512B = 2,    //12 bit ecc requirement per 512 bytes
    ECC_16BIT_P512B = 3,    //16 bit ecc requirement per 512 bytes
    ECC_24BIT_P1KB  = 4,    //24 bit ecc requirement per 1024 bytes
    ECC_32BIT_P1KB  = 5     //32 bit ecc requirement per 1024 bytes
}ECC_TYPE;


typedef struct SNandEccStru
{
    T_U8 *buf;                         //data buffer, e.g. common data buffer or spare buffer
    T_U32 buf_len;                     //data total length, e.g. 4096 or 8192
    T_U32 ecc_section_len;             //ecc section length, e.g. 512, 512+4, or 1024, 1024+4
    ECC_TYPE ecc_type;                 //ecc type, e.g. ECC_4BIT or ECC_8BIT
}T_NAND_ECC_STRU, *T_PNAND_ECC_STRU;


struct SNandflash_Add
{
    T_U8    ChipPos[NFC_SUPPORT_CHIPNUM];
    T_U8    RowCycle;
    T_U8    ColCycle;
    T_U8    ChipType;
    T_U8    EccType;
    T_U32   PageSize;
    T_U32   PagesPerBlock;
};

typedef struct SNandflash_Add* T_PNandflash_Add;
typedef struct SNandflash_Add T_Nandflash_Add;
//**********************************************************************

/**
 * @brief initialization of nandflash hardware.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @return  T_VOID
 */
T_VOID nand_HWinit(T_VOID);

/**
 * @brief config nand command and data cycle
 *
 * @author xuchang
 * @date 2007-12-27
 * @param[in] CmdCycle the command cycle to config
 * @param[in] DataCycle the data cycle to config
 * @return T_VOID
 */
T_VOID nand_config_timeseq(T_U32 cmd_cycle, T_U32 data_cycle);

/**
 * @brief calculate each nand's timing under 62MHz & 124MHz
 *
 * @author yiruoxiang
 * @date 2007-12-27
 * @param[in] DefDataLen default data lenght
 * @return  T_VOID
 */
T_VOID nand_calctiming(T_U32 DefDataLen);

/**
 * @brief change nand timing when Freq has changed
 *
 * @author yiruoxiang
 * @date 2007-12-27
 * @param[in] Freq frequency
 * @return  T_VOID
 */
T_VOID nand_changetiming(T_U32 Freq);

/**
 * @brief read nand flash chip ID.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be read.
 * @return  T_U32
 * @retval  current nandflash ID
 */
T_U32 nand_read_chipID(T_U32 Chip);

/**
 * @brief reset nand flash.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be reset.
 * @return  T_VOID
 */
T_VOID nand_reset(T_U32 chip);

/**
 * @brief read data from nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[in/out] pDataCtrl control reading data section: buffer, data lenght, ECC.
 * @param[in/out] pSpareCtrl control reading spare section: buffer, data lenght, ECC.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_readpage_ecc(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_PNAND_ECC_STRU  pDataCtrl,  T_PNAND_ECC_STRU pSpareCtrl);

/**
 * @brief read one page(page size>=2048) of data from nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[out] Data buffer for read data, should be large than or equal to 2048 bytes.
 * @param[out] Spare buffer for file system info, should be 4 bytes.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_readsector_large(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 Data[], T_U32 *Spare);

/**
 * @brief read one page(page size=512) of data from nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[out] Data buffer for read data, should be 512 bytes.
 * @param[out] Spare buffer for file system info, should be 4 bytes.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_readsector(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 Data[], T_U8 *Spare);

/**
 * @brief read file system info.
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[out] Spare buffer for file system info, should be 4 bytes.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_readspare(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 *Spare);

/**
 * @brief read data from nandflash without ECC.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[out] Data buffer for read data.
 * @param[in] Len how many bytes read from nandflash
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_readbytes(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 Data[], T_U32 Len);

/**
 * @brief write data to nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[in] pDataCtrl control writting data section: buffer, data lenght, ECC.
 * @param[in] pSpareCtrl control writting spare section: buffer, data lenght, ECC.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_writepage_ecc(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_PNAND_ECC_STRU  pDataCtrl,  T_PNAND_ECC_STRU pSpareCtrl);

/**
 * @brief write one page(page size>=2048) of data to nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[in] Data buffer for write data, should be large than or equal to 2048 bytes.
 * @param[in] Spare file system info.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_writesector_large(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 Data[], T_U32 Spare);

/**
 * @brief write one page(page size=512) of data to nand flash with ECC .
 *
 * @author jiangdihui
 * @date 2010-07-23
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[in] Data buffer for write data, should be 512 bytes.
 * @param[in] Spare file system info.
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_writesector(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, T_U8 Data[], T_U8 *Spare);

/**
 * @brief write data to nandflash without ECC.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be read.
 * @param[in] RowAddr the row address of nandflash.
 * @param[in] ColumnAddr the column address of nandflash.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @param[in] Data buffer for write data.
 * @param[in] Len how many bytes write to nandflash
 * @return  T_U32
 * @retval  0 success
 */
T_U32 nand_writebytes(T_U32 Chip, T_U32 RowAddr, T_U32 ColumnAddr, T_PNandflash_Add pNF_Add, const T_U8 Data[], T_U32 Len);

/**
 * @brief erase one block of nandflash.
 *
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be operated.
 * @param[in] BlkStartPage first page of the block.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @return T_U32
 * @retval 0 means write successfully
 * @retval 1 means write unsuccessfully
 * @retval 2 means time out
 */
T_U32 nand_eraseblock(T_U32 Chip, T_U32 BlkStartPage, T_PNandflash_Add pNF_Add);

/**
 * @brief copy one physical page to another one.
 *
 * hardware copyback mode, there should be caches in nandflash, source and destation page should be in the same plane
 * @author yiruoxiang
 * @date 2006-11-02
 * @param[in] Chip which chip will be operated.
 * @param[in] SouPhyPage the source page to read.
 * @param[in] DesPhyPage the destination page to write.
 * @param[in] pNF_Add information of the nandflash characteristic.
 * @return  T_U32
 * @retval  0 means write successfully
 * @retval  1 means write unsuccessfully
 * @retval  2 means time out
 */
T_U32 nand_copyback(T_U32 Chip, T_U32 SrcPhyPage, T_U32 DestPhyPage, T_PNandflash_Add pNF_Add);

/*@}*/
#ifdef __cplusplus
}
#endif

#endif //__ARCH_NAND_H__
