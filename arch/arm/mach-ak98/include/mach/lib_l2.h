/**
* @FILENAME: l2.h
* @BRIEF l2 buffer driver head file
* Copyright (C) 2007 Anyka (Guang zhou) Software Technology Co., LTD
* @AUTHOR Pumbaa
* @DATA 2007-09-11
* @VERSION 1.8
* @REF please refer to...
*/

/******************************************
The following is an example to use mmc and sd driver APIs

******************************************/
#ifndef _ARCH_ARM_MACH_AK88_LIB_L2_H
#define _ARCH_ARM_MACH_AK88_LIB_L2_H

//#include "anyka_types.h"

#define    BUF2MEM                0
#define    MEM2BUF                1
#define    L2_SINGLE_BUF_SIZE     512
#define    L2_INVALIDE_BUF_ID     0xff

typedef enum {
	ADDR_USB_BULKOUT = 0,
	ADDR_USB_BULKIN,	// 1
	ADDR_USB_ISO,		// 2
	ADDR_NFC,		// 3
	ADDR_MMC_SD,		// 4
	ADDR_SDIO,		// 5
	ADDR_RESERVED,		// 6
	ADDR_SPI1_RX,		// 7
	ADDR_SPI1_TX,		// 8    
	ADDR_DAC,		// 9
	ADDR_SPI2_RX,		// 10            
	ADDR_SPI2_TX		// 11
} DEVICE_SELECT;

/**
* @BRIEF initial l2 buffer
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_VOID:
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_Initial(T_VOID);
void l2_init(void);

/**
* @BRIEF allocate a buffer which doesn't used by other device
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 *buf_id : the pointer of the variable which storage return buffer id
* @RETURN T_BOOL: if allocate a buffer successful, return AK_TRUE, otherwise, return AK_FALSE
* @NOTE: to make sure buffer allocate by this function is avalid, user must check the return value
*/
//T_BOOL L2_AllocBuf(T_U8 *buf_id);
unsigned char l2_alloc_buf(unsigned char *buf_id);

/**
* @BRIEF set device information
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM DEVICE_SELECT dev_slct: the device to be select for change it's information
* @PARAM T_U8 buf_id0: a buffer of device used
* @PARAM T_U8 buf_id0: anther buffer of device used
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_SetDevInfo(DEVICE_SELECT dev_slct, T_U8 buf_id, T_U8 buf_id1);
void l2_set_devinfo(DEVICE_SELECT dev_slct, unsigned char buf_id,
		    unsigned char buf_id1);

/**
* @BRIEF free a or two buffer(s) which used by a device
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM DEVICE_SELECT dev_slct: the device which will not use buffer
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_FreeBuf(DEVICE_SELECT dev_slct);
void l2_free_buf(DEVICE_SELECT dev_slct);

/**
* @BRIEF set a buffer as device buffer
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM DEVICE_SELECT dev_slct: the device which will not use buffer
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_SlctBuf(DEVICE_SELECT dev_sel, T_U8 buf_id);
void l2_slct_buf(DEVICE_SELECT dev_sel, unsigned char buf_id);

/**
* @BRIEF transfer data between memory and l2 common buffer with dma mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: transfer size must be a or multi 64 bytes, and less than 4096 bytes
*/
//T_VOID L2_ComBufDMA(T_U32 ram_addr, T_U8 buf_id, T_U32 tran_byte, T_U8 tran_dir);
void l2_combuf_dma(unsigned int ram_addr, unsigned char buf_id,
		   unsigned int tran_byte, unsigned char tran_dir);

/**
* @BRIEF transfer fraction data between memory and l2 common buffer with dma mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U8 buf_offset: the offset between buffer start address and transfer start address
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: transfer size can be 1~64 byte(s), buffer offset can be 0~7, 1 mean 64 bytes data
*/
//T_VOID L2_ComBufFracDMA(T_U32 ram_addr, T_U8 buf_id, T_U8 buf_offset, T_U32 tran_byte, T_U8 tran_dir);
void l2_combuf_fracdma(unsigned int ram_addr, unsigned char buf_id,
		       unsigned char buf_offset, unsigned int tran_byte,
		       unsigned char tran_dir);

/**
* @BRIEF wait DMA transfer data between memory and common buffer finish 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_WaitComBufDMAFinish(T_U8 buf_id);
void l2_wait_combuf_dmafinish(unsigned char buf_id);

/**
* @BRIEF transfer data between memory and l2 common buffer with dma and fraction dma
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: parameter tran_byte must be less than 512
*/
//T_VOID L2_ComBufTranData(T_U32 ram_addr, T_U8 buf_id, T_U32 tran_byte, T_U8 tran_dir);
void l2_combuf_tran_data(unsigned int ram_addr, unsigned char buf_id,
			 unsigned int tran_byte, unsigned char tran_dir);

/**
* @BRIEF transfer data between memory and l2 common buffer with cpu
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: 1. from RAM to buffer 0. from buffer to RAM
* @RETURN T_VOID: 
* @NOTE: parameter tran_byte must be less than 512
*/
//T_VOID L2_ComBufTranData_CPU(T_U32 ram_addr, T_U8 buf_id, T_U32 tran_byte, T_U8 tran_dir);
void l2_combuf_tran_data_cpu(unsigned int ram_addr, unsigned char buf_id,
			     unsigned int tran_byte, unsigned char tran_dir);

/**
* @BRIEF transfer data between memory and l2 uart buffer with dma mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: transfer size must be a or multi 64 bytes, and less than 4096 bytes
*/
//T_VOID L2_UartBufDMA(T_U32 ram_addr, T_U8 uart_id, T_U32 tran_byte, T_U8 tran_dir);
void l2_uartbuf_dma(unsigned int ram_addr, unsigned char uart_id,
		    unsigned int tran_byte, unsigned char tran_dir);

/**
* @BRIEF transfer fraction data between memory and l2 uart buffer with dma mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U8 buf_offset: the offset between buffer start address and transfer start address
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: transfer size can be 1~64 byte(s), buffer offset can be 0~7, 1 mean 64 bytes data
*/
//T_VOID L2_UartBufFracDMA(T_U32 ram_addr, T_U8 uart_id, T_U8 buf_offset, T_U32 tran_byte, T_U8 tran_dir);
void l2_uartbuf_fracdma(unsigned int ram_addr, unsigned char uart_id,
			unsigned char buf_offset, unsigned int tran_byte,
			unsigned char tran_dir);

/**
* @BRIEF wait DMA transfer data between memory and uart buffer finish 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U8 tran_dir: the transfer direction
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_WaitUartBufDMAFinish(T_U8 uart_id, T_U8 tran_dir);
void l2_wait_uartbuf_dmafinish(unsigned char uart_id, unsigned char tran_dir);

/**
* @BRIEF transfer data between memory and l2 uart buffer with dma and fraction dma mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: parameter tran_byte must be less than 128
*/
//T_VOID L2_UartBufTxData(T_U32 ram_addr, T_U8 uart_id, T_U32 tran_byte);
void l2_uartbuf_tx_data(unsigned int ram_addr, unsigned char uart_id,
			unsigned int tran_byte);

/**
* @BRIEF wait fraction DMA transfer data between memory and buffer finish 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @RETURN T_VOID: 
* @NOTE: 
*/
//T_VOID L2_WaitFracDMAFinish(T_VOID);
void l2_wait_frac_dmafinish(void);

/**
* @BRIEF transfer data between memory and l2 buffer with CPU mode
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U32 buf_addr: the buffer address
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: transfer data from memory to buffer or from buffer to memory
* @RETURN T_VOID: 
* @NOTE: if buffer is common buffer, the max value of tran_byte should be 512, if buffer is uart buffer, the
        max value of tran_byte should be 128.
*/
//T_VOID L2_TranDataCPU(T_U32 ram_addr, T_U8 buf_id, T_U32 buf_offset, T_U32 tran_byte, T_U8 tran_dir);
void l2_tran_data_cpu(unsigned int ram_addr, unsigned char buf_id,
		      unsigned int buf_offset, unsigned int tran_byte,
		      unsigned char tran_dir);

/**
* @BRIEF transfer data from a buffer to another buffer
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 dst_addr: the destination buffer address
* @PARAM T_U32 src_addr: the source buffer address
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @RETURN T_VOID: 
* @NOTE: all the parameter must be multiple of 4 bytes
*/
//T_VOID L2_LocalDMA(T_U32 dst_addr, T_U32 src_addr, T_U32 tran_byte);
void l2_localdma(unsigned int dst_addr, unsigned int src_addr,
		 unsigned int tran_byte);

/**
* @BRIEF return a common buffer current flag 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_BOOL: if return flag is 1, it mean buffer is full, otherwise mean buffer is empty 
* @NOTE: buffer id should be 0~7
*/
//T_U8 L2_ComBufFlag(T_U8 buf_id);
unsigned char l2_combuf_flag(unsigned char buf_id);

/**
* @BRIEF forcibly set a common buffer flag
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: buffer id should be 0~7
*/
//T_VOID L2_SetComBufFlag(T_U8 buf_id);
void l2_set_combuf_flag(unsigned char buf_id);

/**
* @BRIEF forcibly clear a common buffer flag
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: buffer id should be 0~7
*/
//T_VOID L2_ClrComBufFlag(T_U8 buf_id);
void l2_clr_combuf_flag(unsigned char buf_id);

//T_VOID L2_ChangeSetFlag(T_U8 buf_id, T_U8 set_status_flag);
void l2_change_set_flag(unsigned char buf_id, unsigned char set_status_flag);

/**
* @BRIEF forcibly clear a common buffer flag
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: buffer id should be 0~7
*/
//T_VOID L2_ClrComBufStatus(T_U8 buf_id);
void l2_clr_combuf_status(unsigned char buf_id);

/**
* @BRIEF forcibly clear a uart buffer status to 0
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: buffer id should be 8~15
*/
//T_VOID L2_ClrUartBufStatus(T_U8 buf_id);
void l2_clr_uartbuf_status(unsigned char buf_id);

/**
* @BRIEF return a common buffer current flag 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_BOOL: if return flag is 1, it mean buffer is full, oterwise mean buffer is empty 
* @NOTE: buffer id should be 0~7
*/
//T_U8 L2_ComBufStatus(T_U8 buf_id);
unsigned char l2_combuf_status(unsigned char buf_id);

/**
* @BRIEF return a uart buffer current status 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_U8: if return status is 2, it mean buffer is full, if return 1, it mean buffer is half full, oterwise 
                mean buffer is empty 
* @NOTE: buffer id should be 8~15
*/
//T_U8 L2_UartBufStatus(T_U8 buf_id);
unsigned char l2_uartbuf_status(unsigned char buf_id);

//unsigned long AKSET_BITS(unsigned long bits_result,/* void __iomem* */void* reg);
//unsigned long AKCLR_BITS(unsigned long bits_result,/* void __iomem* */void* reg);
//bool AKGET_BIT(unsigned long bit_result,/* void __iomem* */void* reg);

#endif
