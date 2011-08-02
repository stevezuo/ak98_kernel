/**
 * @filename nand_control.h
 * @brief AK880x nandflash driver
 *
 * This file describe how to control the AK880x nandflash driver.
 * Copyright (C) 2008 Anyka (GuangZhou) Software Technology Co., Ltd.
 * @author yiruoxiang
 * @modify jiangdihui
 * @date 2007-1-10
 * @version 1.0
 * @ref
 */
#ifndef __NAND_CONTORL_H__
#define __NAND_CONTORL_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup NandFlash Architecture NandFlash Interface
 *  @ingroup Architecture
 */
/*@{*/

/* contorl/status register */
#define NF_REG_CTRL_STA_CMD_DONE  (1UL << 31)
#define NF_REG_CTRL_STA_CE0_SEL   (1 << 10)
#define NF_REG_CTRL_STA_CE1_SEL   (1 << 11)
#define NF_REG_CTRL_STA_STA_CLR   (1 << 14)

/* dma cpu control register */
#define BIT_DMA_CMD_DONE          0x80000000

/* ECC control bit */
#define ECC_CTL_DIR_READ        (0<<2)
#define ECC_CTL_ADDR_CLR        (1<<4)
#define ECC_CTL_NFC_EN          (1<<20)
#define ECC_CTL_BYTE_CFG(m)     ((m)<<7)  
#define ECC_CTL_MODE(n)         ((n)<<22)
#define ECC_CTL_START           (1<<3)
#define ECC_CTL_DEC_EN          (1<<1)
#define ECC_CTL_RESULT_NO_OK    (1<<27)
#define ECC_CTL_END             (1<<6)
#define ECC_CTL_NO_ERR          (1<<26)
#define ECC_CTL_DIR_WRITE       (1<<2)
#define ECC_CTL_ENC_EN          (1<<0)
#define ECC_CTL_DEC_RDY         (1<<24)
#define ECC_CTL_ENC_RDY         (1<<23)

#define ECC_CHECK_NO_ERROR                              (0x1<<26)
#define ECC_ERROR_REPAIR_CAN_NOT_TRUST                  (0x1<<27)

#define DATA_ECC_CHECK_OK                               1
#define DATA_ECC_CHECK_ERROR                            0
#define DATA_ECC_ERROR_REPAIR_CAN_TRUST                 2
#define DATA_ECC_ERROR_REPAIR_CAN_NOT_TRUST             3

//  |               page               |
//  | data | fs(file system) | parity  |
//  | data |          spare            |
#define NAND_DATA_SIZE            512  
#define NAND_FS_SIZE              4
#define NAND_PARITY_SIZE_MODE1    13
#define NAND_PARITY_SIZE_MODE0    7

/// @cond NANDFLASH_DRV
//******************************************************************************************//
//通过CTRL reg0进行命令压栈的说明如下。（共7个confiuration，1个flag）
//*************reg0~reg19命令的类型如下***********
/*
1,nandflash本身数据总线上传输数据的类型
a,COMMAND_CYCLES_CONF:表示数据总线上写入的是Nandflash命令；
b,ADDRESS_CYCLES_CONF:表示数据总线上写入的是Nandflash地址信息
c,WRITE_DATA_CONF:表示数据总线上写入的FIFO中的数据信息，
由于读出数据可能放在不同位置，所以这里有两种读数据命令
d，READ_DATA_CONF：表示数据中总线读数据，读出数据放在FIFO中
e，READ_CMD_CONF这命名好像不太好，我把它改为READ_INFO_CONF
  表示数据总线上读的是状态或是ID信息指令,(也可用于读取小于
  8 byte的数据两）读出数据放在reg20

  另外，如果带上ECC的读，芯片会自动把data信息和文件系统信息放到相
  应的FIFO和寄存器。

2,与nandflash无关，为了引入延时和确定读写预处理等待完成时引入的命令
f,DELAY_CNT_CONF 由于nandflash不会马上相应命令，大约有3us到50us的延时，
  数据才会准备好，所以引入该命令。延时时间在bit11~bit22中写入
g.WAIT_JUMP_CONF 在Nandflash操作中，除了可以读status 软件查询状态外，
还有一个R/B引脚用来辅助快速确定读写操作是否完成。该命令等待上跳延触发。

上面命令每次任选且只能选一条，不断通过reg0地址压到命令栈中，即可实现对
nandflash的所有操作
如果是最后一条指令，必须打上LAST_CMD_FLAG标志
*/

//////// command sequece configuration define ////////////

/* command cycle's configure:
 *      CMD_END=X, ALE=0, CLE=1, CNT_EN=0,  (BIT[0:3])
 *      REN=0, WEN=1, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *      DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define COMMAND_CYCLES_CONF     0x64
/* address cycle's:
 *      CMD_END=X, ALE=1, CLE=0, CNT_EN=0,  (BIT[0:3])
 *      REN=0, WEN=1, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *      DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define ADDRESS_CYCLES_CONF     0x62

/*  read data cycle's:
 *      CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *      REN=1, WEN=0, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *      DAT_EN=1, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define READ_DATA_CONF          0x118

/*  write data cycle's:
 *      CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *      REN=0, WEN=1, CMD_EN=0, STAFF_EN=0, (BIT[4:7])
 *      DAT_EN=1, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define WRITE_DATA_CONF         0x128

/* read command status's:(example: read ID(pass), status(pass), 回读数据比较少的操作?
//告诉芯片现在读取status ID信息，芯片把该信息搬到相应寄存器(8 byte)，而不是FIFO。方便查询Nandflash状态。
 *      CMD_END=X, ALE=0, CLE=0, CNT_EN=1,  (BIT[0:3])
 *      REN=1, WEN=0, CMD_EN=1, STAFF_EN=0, (BIT[4:7])
 *      DAT_EN=0, RBN_EN=0, CMD_WAIT=0,     (BIT[8:10])
 */
#define READ_INFO_CONF          0x58

//wait delay time enable bit
#define DELAY_CNT_CONF          (1<<10)

//wait R/b enable bit
#define WAIT_JUMP_CONF          (1<<9)

// last command's bit0 set to 1
#define LAST_CMD_FLAG           (1<<0)


//#######excute comamnd,ctrl reg22 command configuration define ################
//所有的配置定义针对发送命令的方式。
//必须进行片选或片保护、操作模式、以及是否使能的省电模式flag
//片选使用宏NCHIP_SELECT(x)
#define NCHIP_SELECT(x)         ((0x01 << (x)) << 10)

/*
bit0(save mode) = 0;
bit1-bit8(staff_cont)=0
bit9(watit save mode jump) = 0
bit10-bit13(chip select)=0
bit14(sta_clr)=0
bit15-bit18(write protect)=0
bit19-bit29 reserved =0;
bit30(start control) = 1;
bit31(check statu) = 0;
*/
#define DEFAULT_GO                  ((1 << 30)|(1<<9))


/** @{@name Command List and Status define
 *  Define command set and status of nandflash
 */
#define NFLASH_READ1                0x00
#define NFLASH_READ2                0x30
#define NFLASH_READ1_HALF           0x01
#define NFLASH_READ22               0x50

#define NFLASH_COPY_BACK_READ       0x35
#define NFLASH_COPY_BACK_WRITE      0x85
#define NFLASH_COPY_BACK_WRITE1     0x8A
#define NFLASH_COPY_BACK_CONFIRM    0x10
#define NFLASH_RESET                0xff

#define NFLASH_FRAME_PROGRAM0       0x80
#define NFLASH_FRAME_PROGRAM1       0x10

#define NFLASH_BLOCK_ERASE0         0x60
#define NFLASH_BLOCK_ERASE1         0xD0
#define NFLASH_STATUS_READ          0x70
#define NFLASH_READ_ID              0x90

//status register bit
#define NFLASH_PROGRAM_SUCCESS      0x01 //bit 0
#define NFLASH_HANDLE_READY         0x40 //bit 6
#define NFLASH_WRITE_PROTECT        0x80 //bit 7

/*@}*/
#ifdef __cplusplus
}
#endif

#endif //__NAND_CONTORL_H__
