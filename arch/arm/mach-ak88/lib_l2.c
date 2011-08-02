/**
* @FILENAME: l2.c
* @BRIEF l2 buffer driver file
* Copyright (C) 2007 Anyka (Guang zhou) Software Technology Co., LTD
* @AUTHOR Pumbaa
* @DATA 2007-09-11
* @VERSION 1.8
* @REF please refer to...
*/

#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <mach/map.h>
#include <mach/lib_l2.h>

#define IDLE_STATE            1
#define USED_STATE            0

#define BUF_NULL             0xff

typedef struct {
	unsigned char buf_id;
	unsigned char usable;
	unsigned int used_time;
} L2_INFO;

typedef struct {
	DEVICE_SELECT device;
	unsigned char buf_id;
} DEVICE_INFO;

L2_INFO L2_INFO_TABLE[] = {
	{0, IDLE_STATE, 0},
	{1, IDLE_STATE, 0},
	{2, IDLE_STATE, 0},
	{3, IDLE_STATE, 0},
	{4, IDLE_STATE, 0},
	{5, IDLE_STATE, 0},
	{6, IDLE_STATE, 0},
	{7, IDLE_STATE, 0}
};

DEVICE_INFO DEVICE_INFO_TABLE[] = {
	{ADDR_USB_BULKOUT, BUF_NULL},
	{ADDR_USB_BULKIN, BUF_NULL},
	{ADDR_USB_ISO, BUF_NULL},
	{ADDR_NFC, BUF_NULL},
	{ADDR_MMC_SD, BUF_NULL},
	{ADDR_SDIO, BUF_NULL},
	{ADDR_RESERVED, BUF_NULL},
	{ADDR_SPI1_RX, BUF_NULL},
	{ADDR_SPI1_TX, BUF_NULL},
	{ADDR_DAC, BUF_NULL},
	{ADDR_SPI2_RX, BUF_NULL},
	{ADDR_SPI2_TX, BUF_NULL}
};

/**
* @BRIEF initial l2 buffer
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_VOID:
* @RETURN T_VOID: 
* @NOTE: 
*/
//define Fraction DMA Address Information Register 's bit map
#define AHB_FLAG_EN          29
#define LDMA_FLAG_EN         28

//define DMA Request Register 's bit map
#define DMA_EN               0

void akl2_init(void)
{
	__raw_writel(1 << DMA_EN, AK88_L2CTR_DMAREQ);
	//0x2002c080

	//yi puzzle: use auto cpu-controlling of buffer status?
	__raw_writel((1 << LDMA_FLAG_EN) | (1 << AHB_FLAG_EN),
		     AK88_L2CTR_DMAFRAC);
	//0x2002c084

	__raw_writel(0xffff00ff, AK88_L2CTR_COMBUF_CFG);
	//0x2002c088

	__raw_writel(0xf0ff0000, AK88_L2CTR_UARTBUF_CFG);
	//0x2002c08c
}

/**
* @BRIEF allocate a buffer which doesn't used by other device
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 *buf_id : the pointer of the variable which storage return buffer id
* @RETURN T_BOOL: if allocate a buffer successful, return AK_TRUE, otherwise, return AK_FALSE
* @NOTE: to make sure buffer allocate by this function is avalid, user must check the return value
*/
unsigned char l2_alloc_buf(unsigned char *buf_id)
{
	unsigned int buffer_nbr = sizeof(L2_INFO_TABLE) / sizeof(L2_INFO);
	unsigned int used_least_count = 0;
	unsigned int i;
	unsigned char first_usable_id = BUF_NULL;
	unsigned char slct_id = 0;

	for (i = 0; i < buffer_nbr; i++) {
		if (L2_INFO_TABLE[i].usable == IDLE_STATE)	//find a buffer which hasn't been used
		{
			if (first_usable_id == BUF_NULL)	//the first usable buf id
			{
				first_usable_id = L2_INFO_TABLE[i].buf_id;
				used_least_count = L2_INFO_TABLE[i].used_time;
				slct_id = first_usable_id;
			}
			if (L2_INFO_TABLE[i].used_time < used_least_count) {
				used_least_count = L2_INFO_TABLE[i].used_time;
				slct_id = L2_INFO_TABLE[i].buf_id;
			}
		}
	}

	if (first_usable_id == BUF_NULL) {
		return AK_FALSE;	//can't find any buffer which hasn't been used
		//return 0;
	} else			//malloc a buffer successfully
	{
		L2_INFO_TABLE[slct_id].usable = USED_STATE;
		L2_INFO_TABLE[slct_id].used_time++;
		if (L2_INFO_TABLE[slct_id].used_time == 0) {
			for (i = 0; i < buffer_nbr; i++)
				L2_INFO_TABLE[i].used_time = 0;	//clear all buffer's used_time
		}
		*buf_id = slct_id;
		return AK_TRUE;
		//return 1;
	}
}

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
void l2_set_devinfo(DEVICE_SELECT dev_slct, unsigned char buf_id,
		    unsigned char buf_id1)
{
	DEVICE_INFO_TABLE[(unsigned char)dev_slct].buf_id = buf_id;
}

/**
* @BRIEF free a or two buffer(s) which used by a device
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM DEVICE_SELECT dev_slct: the device which will not use buffer
* @RETURN T_VOID: 
* @NOTE: 
*/
void l2_free_buf(DEVICE_SELECT dev_slct)
{
	unsigned int device_nbr =
	    sizeof(DEVICE_INFO_TABLE) / sizeof(DEVICE_INFO);
	unsigned int i;
	unsigned char free_id;

	free_id = DEVICE_INFO_TABLE[(unsigned char)dev_slct].buf_id;
	if (free_id != BUF_NULL) {
		DEVICE_INFO_TABLE[(unsigned char)dev_slct].buf_id = BUF_NULL;
		for (i = 0; i < device_nbr; i++) {
			if (DEVICE_INFO_TABLE[i].buf_id == free_id)
				break;
		}

		if (i == device_nbr)	//there isnot device use this buffer
		{
			L2_INFO_TABLE[free_id].usable = IDLE_STATE;
		}
	}
}

/**
* @BRIEF set a buffer as device buffer
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM DEVICE_SELECT dev_slct: the device which will not use buffer
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: 
*/
void l2_slct_buf(DEVICE_SELECT dev_sel, unsigned char buf_id)
{
	volatile unsigned int reg_id;
	volatile unsigned int base_bit;
	volatile unsigned int reg_value;

	if ((unsigned char)dev_sel > 9) {
		reg_id = (unsigned int)AK88_L2CTR_ASSIGN_REG2;	//0x2002c094
		base_bit = ((unsigned char)dev_sel - 10) * 3;
	} else {
		reg_id = (unsigned int)AK88_L2CTR_ASSIGN_REG1;	//0x2002c090
		base_bit = (unsigned char)dev_sel *3;
	}

	reg_value = __raw_readl(reg_id);
	reg_value &= ~(0x7 << base_bit);	//assume device select buffer 0
	reg_value |= ((buf_id & 0x7) << base_bit);	//according buf_id to adjust selcted buf no
	__raw_writel(reg_value, reg_id);
}

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
void l2_combuf_dma(unsigned int ram_addr, unsigned char buf_id,
		   unsigned int tran_byte,
		   unsigned char tran_dir /*1:write(send),0:read(recv) */ )
{
	unsigned int tran_nbr;	//transfer times
	volatile unsigned int reg_value;
	volatile unsigned int reg_id;

	tran_nbr = tran_byte >> 6;

	//1).set ram_addr to ANYKA_L2_DMAADDR
	reg_value = ram_addr & 0xfffffff;
	reg_id = (unsigned int)AK88_L2CTR_DMAADDR + (buf_id << 2);	//0x2002c000 -> 0x2002c03c
	//0x2002c000 -> 0x2002c03c for buffer 0 to 15 (32-bit for each)

	__raw_writel(reg_value, reg_id);

	//2).set dma times to ANYKA_L2_DMACNT
	reg_value = tran_nbr & 0xff;
	reg_id = (unsigned int)AK88_L2CTR_DMACNT + (buf_id << 2);	//0x2002c040 -> 0x2002c07c
	//0x2002c040 -> 0x2002c07c for buffer 0 to 15 (32-bit for each)
	//need operating DMA times,

	__raw_writew((volatile unsigned short)reg_value, reg_id);

	//3).sele common DMA buf number(0 to 8),and sele write/read by ANYKA_L2_COMBUF_CFG
	reg_value = __raw_readl(AK88_L2CTR_COMBUF_CFG);	//0x2002c088
	//0x2002c088

	if (tran_dir)		//write(send)
		reg_value |= (1 << (8 + buf_id));
	else
		reg_value &= ~(1 << (8 + buf_id));

	__raw_writel(reg_value, AK88_L2CTR_COMBUF_CFG);	//0x2002c088

	//4).send request dma commond by ANYKA_L2_DMAREQ
	reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
	//0x2002c080

	reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req
	reg_value |= (1 << (24 + buf_id));

	__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

}

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
void l2_combuf_fracdma(unsigned int ram_addr, unsigned char buf_id,
		       unsigned char buf_offset, unsigned int tran_byte,
		       unsigned char tran_dir)
{
	unsigned int buf_addr;
	volatile unsigned int reg_value;

	//1).set ram_addr to ANYKA_L2_DMAFRAC
	reg_value = __raw_readl(AK88_L2CTR_DMAFRAC);	//0x2002c084
	reg_value &= ~0xfffffff;
	reg_value |= (ram_addr & 0xfffffff);
	__raw_writel(reg_value, AK88_L2CTR_DMAFRAC);	//0x2002c084

	//2).set frac trans len, l2 buf_id,buf_offset in l2 buf, in 0x2002c080

	buf_addr = ((buf_id & 0x7) << 3) | (buf_offset & 0x7);
	//reg_value = inl(L2_DMA_REQ); 
	//bit[7:1]:Frac_DMA_L2_addr, DMA buf contain 8's 512buf,and each 512buf contain 8's
	//64subbuf. Frac_DMA_L2_addr point the buf_id of 8's of 512buf,add the sub buf_id 
	//if 8's of 64subbuf in 512buf align
	//bit[15:10]:Frac_DMA_len,the real remaider in 64subbuf

	reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080

	reg_value &= ~((0x7f << 1) | (0x3f << 10));
	reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req        
	if (tran_dir)
		reg_value |=
		    (1 << 9) | (1 << 8) | (buf_addr << 1) | ((tran_byte - 1) <<
							     10);
	else {
		reg_value &= ~(1 << 8);
		reg_value |=
		    (1 << 9) | (buf_addr << 1) | ((tran_byte - 1) << 10);
	}

	__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

}

/**
* @BRIEF wait DMA transfer data between memory and common buffer finish 
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: 
*/
void l2_wait_combuf_dmafinish(unsigned char buf_id)
{

	while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << (24 + buf_id))) ;	//0x2002c080

	//0x2002c080 bit[31:24]: when this bit is set by CPU,the buf(buf_id) DMA operation
	//                       enabled.And this bit is cleared automatically when the 
	//                       requested DMA operation has been finished successfully.

}

/**
* @BRIEF wait Fraction DMA transfer data between memory and common buffer finish 
* @AUTHOR YiRuoxiang
* @DATE 2008-02-19
* @PARAM T_U8 buf_id: the buffer id
* @RETURN T_VOID: 
* @NOTE: 
*/
void l2_wait_combuf_frac_dmafinish(unsigned char buf_id)
{
	volatile unsigned int dma_conf;


	do {
		dma_conf = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
	} while ((dma_conf & (1 << 9)));

	//0x2002c080,bit9:Frac_DMA_rea:Fraction DMA request signal.
	//                when this bit is set by CPU,the fraction DMA operation is enabled.
	//                and this bit is cleared automatically when the requested DMA 
	//                operation has been finished successfully.
}

/**
* @BRIEF transfer data between memory and l2 common buffer with dma and fraction dma
* @AUTHOR Pumbaa
* @DATE 2007-07-19
* @PARAM T_U32 ram_addr: the memory address
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U32 tran_byte: the size of data which will be transfered
* @PARAM T_U8 tran_dir: 1. from RAM to buffer 0. from buffer to RAM
* @RETURN T_VOID: 
* @NOTE: parameter tran_byte must be less than 512
*/

extern void mmu_clean_invalidate_dcache(void);

void l2_combuf_tran_data(unsigned int ram_addr, unsigned char buf_id,
			 unsigned int tran_byte, unsigned char tran_dir)
{

	unsigned int tran_nbr = tran_byte >> 6;
	unsigned int fraction_nbr = tran_byte % 64;
	unsigned int buf_addr;
	volatile unsigned int reg_value;
	volatile unsigned int reg_id;

	mmu_clean_invalidate_dcache();

#if 0
	__asm__ __volatile__("mmu_clean_invalidate_dcache: " "\n\t"
			     "mrc p15,0,r15,c7,c14,3 "
			     "bne mmu_clean_invalidate_dcache "
			     "mov pc,lr ":::"cc");
#endif

	if (tran_nbr) {
		reg_value = ram_addr & 0xfffffff;
		reg_id = (unsigned int)AK88_L2CTR_DMAADDR + (buf_id << 2);	//0x2002c000 -> 0x2002c03c  
		__raw_writel(reg_value, reg_id);

		reg_value = tran_nbr & 0xff;
		reg_id = (unsigned int)AK88_L2CTR_DMACNT + (buf_id << 2);	//0x2002c040 -> 0x2002c07c  
		__raw_writew((unsigned short)reg_value, reg_id);

		reg_value = __raw_readl(AK88_L2CTR_COMBUF_CFG);	//0x2002c088
		if (tran_dir)	//write(send)
			reg_value |= (1 << (8 + buf_id));
		else
			reg_value &= ~(1 << (8 + buf_id));
		__raw_writel(reg_value, AK88_L2CTR_COMBUF_CFG);	//0x2002c088

		reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
		reg_value &= ~((1 << 9) | (((unsigned int)0xffff) << 16));	//clear other buf req
		reg_value |= (1 << (24 + buf_id));
		__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

		//wait combuf dma finish
		while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << (24 + buf_id)))	//0x2002c080
		{
		}
	}

	if (fraction_nbr) {
		reg_value = __raw_readl(AK88_L2CTR_DMAFRAC);	//0x2002c084
		reg_value &= ~0xfffffff;
		reg_value |= ((ram_addr + (tran_nbr << 6)) & 0xfffffff);	//frac begin addr
		__raw_writel(reg_value, AK88_L2CTR_DMAFRAC);	//0x2002c084 

		buf_addr = ((buf_id & 0x7) << 3) | ((tran_nbr % 8) & 0x7);
		//bit[7:1]:Frac_DMA_L2_addr, DMA buf contain 8's 512buf,and each 512buf contain 8's
		//64subbuf. Frac_DMA_L2_addr point the buf_id of 8's of 512buf,add the sub buf_id 
		//if 8's of 64subbuf in 512buf align
		//bit[15:10]:Frac_DMA_len,the real remaider in 64subbuf

		reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080

		reg_value &= ~((0x7f << 1) | (0x3f << 10));
		reg_value &= ~((1 << 9) | (((unsigned int)0xffff) << 16));	//clear other buf req    

		if (tran_dir)
			reg_value |=
			    (1 << 9) | (1 << 8) | (buf_addr << 1) |
			    ((fraction_nbr - 1) << 10);
		else {
			reg_value &= ~(1 << 8);
			reg_value |=
			    (1 << 9) | (buf_addr << 1) | ((fraction_nbr - 1) <<
							  10);
		}

		__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

		while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << 9))	//0x2002c080 //wait dma finish
		{
		}
	}

	if ((tran_dir == 1) && (tran_byte % 512 != 0)
	    && (tran_byte % 512 <= 512 - 4)) {
		*(volatile unsigned int *)(AK88_VA_L2BUF + buf_id * 512 +
					   0x1fc) = 0;
	}			//0x48000000
}

void l2_combuf_tran_data_cpu(unsigned int ram_addr, unsigned char buf_id,
			     unsigned int tran_byte, unsigned char tran_dir)
{
	volatile unsigned int align_4_len, len_remian_4, buf_addr, i, tmp,
	    buf_status;
	unsigned int *p_buf_int;
	unsigned char *buf = (unsigned char *)ram_addr;

	//buf_addr = L2_BUF_MEM_BASE_ADDR + (buf_id << 9);  //0x48000000 + buf_id*512
	buf_addr = (unsigned int)AK88_VA_L2BUF + (buf_id << 9);	// 512 = (1<<9) //0x48000000 + buf_id*512

	align_4_len = tran_byte & (~0x3);
	len_remian_4 = tran_byte % 4;

	if (tran_dir == MEM2BUF)	//from SDRAM to L2, write(send)
	{
		do {
			buf_status = __raw_readl(AK88_L2CTR_STAT_REG1);	//0x2002c0a0
		} while ((buf_status & (0xf << (4 * buf_id))) != 0);

		if ((unsigned int)buf % 4 == 0)	// if the buf is the even then it can 4 bytes write.
		{
			p_buf_int = (unsigned int *)buf;
			for (i = 0; i < align_4_len; i = i + 4) {
				*(volatile unsigned int *)(buf_addr + i) = *(p_buf_int++);	//SDRAM to L2
			}

			if (len_remian_4 != 0) {
				tmp = 0;
				for (i = 0; i < len_remian_4; i++)
					tmp =
					    tmp +
					    (buf[align_4_len + i] << (i * 8));

				*(volatile unsigned int *)(buf_addr + align_4_len) = tmp;	//SDRAM to L2
			}
		} else		// if the buf is the odd then it can't 4 bytes write.
		{
			for (i = 0; i < align_4_len; i = i + 4) {
				tmp = (buf[i] | (buf[i + 1] << 8) | (buf[i + 2] << 16) | (buf[i + 3] << 24));	//little endian
				*(volatile unsigned char *)(buf_addr + i) = tmp;	//SDRAM to L2
			}
			tmp = 0;
			if (len_remian_4 != 0) {
				for (i = 0; i < len_remian_4; i++) {
					tmp =
					    tmp +
					    (buf[align_4_len + i] << (i * 8));
				}
				*(volatile unsigned char *)(buf_addr + align_4_len) = tmp;	//SDRAM to L2
			}
		}

		if (tran_byte <= 512 - 4)
			*(volatile unsigned int *)(buf_addr + 0x1fc) = 0;	//end flag

	} else			//from L2 to SDRAM , read/(receive)
	{
		if (tran_byte == 512) {
			do {
				buf_status = __raw_readl(AK88_L2CTR_STAT_REG1);	//0x2002c0a0
			} while ((buf_status & (0xf << (4 * buf_id))) !=
				 (8 << (4 * buf_id)));
		}
		if ((unsigned int)buf % 4 == 0)	// if the buf is the even then it can 4 bytes write.
		{
			p_buf_int = (unsigned int *)buf;
			for (i = 0; i < align_4_len; i = i + 4)
				*(p_buf_int++) = *(volatile unsigned int *)(buf_addr + i);	//L2 to SDRAM

			if (len_remian_4 != 0) {
				tmp =
				    *(volatile unsigned int *)(buf_addr +
							       align_4_len);
				for (i = 0; i < len_remian_4; i++)
					buf[align_4_len + i] = (tmp >> (8 * i)) & 0xff;	//L2 to SDRAM
			}
		} else		// if the buf is the odd then it can't 4 bytes write. 
		{
			for (i = 0; i < align_4_len; i = i + 4) {
				tmp = *(volatile unsigned int *)(buf_addr + i);	//L2 to SDRAM 

				buf[i] = (tmp & 0xff);
				buf[i + 1] = ((tmp >> 8) & 0xff);
				buf[i + 2] = ((tmp >> 16) & 0xff);
				buf[i + 3] = ((tmp >> 24) & 0xff);
			}

			if (len_remian_4 != 0) {
				tmp = *(volatile unsigned int *)(buf_addr + align_4_len);	//L2 to SDRAM  
				for (i = 0; i < len_remian_4; i++)
					buf[align_4_len + i] =
					    (tmp >> (8 * i)) & 0xff;
			}
		}

		//yi puzzle: manually set buffer status, should change to 64B in ak7801
		if (tran_byte <= 512 - 4)	//clear flag
			l2_clr_combuf_flag(buf_id);
	}
}

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
void l2_uartbuf_dma(unsigned int ram_addr, unsigned char uart_id,
		    unsigned int tran_byte, unsigned char tran_dir)
{
	volatile unsigned int tran_nbr;
	volatile unsigned int reg_value;
	volatile unsigned int buf_id;
	volatile unsigned int reg_id;

	tran_nbr = tran_byte >> 6;

	if (tran_dir)
		buf_id = 8 + uart_id * 2;
	else
		buf_id = 8 + uart_id * 2 + 1;

	reg_value = ram_addr & 0xfffffff;
	reg_id = (unsigned int)AK88_L2CTR_DMAADDR + (buf_id << 2);	//0x2002c000 to 0x2002c03c
	__raw_writel(reg_value, reg_id);

	reg_value = tran_nbr & 0xff;
	reg_id = (unsigned int)AK88_L2CTR_DMACNT + (buf_id << 2);	//0x2002c040 to 0x2002c07c
	__raw_writew((volatile unsigned short)reg_value, reg_id);

	reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
	reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req
	reg_value |= (1 << (8 + buf_id));
	__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080
}

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
void l2_uartbuf_fracdma(unsigned int ram_addr, unsigned char uart_id,
			unsigned char buf_offset, unsigned int tran_byte,
			unsigned char tran_dir)
{
	unsigned int buf_addr;
	volatile unsigned int reg_value;
	unsigned int buf_id;

	if (tran_dir)
		buf_id = 8 + uart_id * 2;
	else
		buf_id = 8 + uart_id * 2 + 1;

	reg_value = __raw_readl(AK88_L2CTR_DMAFRAC);	//0x2002c084
	reg_value &= ~(0x3ffffff);
	reg_value |= (ram_addr & 0x3ffffff);
	__raw_writel(reg_value, AK88_L2CTR_DMAFRAC);	//0x2002c084

	buf_addr = (0x40 + ((buf_id - 8) << 1)) | buf_offset;	//0x40=64
	reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
	reg_value &= ~((0x7f << 1) | (0x3f << 10));
	reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req        
	if (tran_dir)
		reg_value |=
		    (1 << 9) | (1 << 8) | (buf_addr << 1) | ((tran_byte - 1) <<
							     10);
	//bit[7:1]:Frac_DMA_L2_add
	//bit[15:0](0x2002c080):Frac_DMD_len
	else {
		reg_value &= ~(1 << 8);
		reg_value |=
		    (1 << 9) | (buf_addr << 1) | ((tran_byte - 1) << 10);
	}
	__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080
}

/**
* @BRIEF wait DMA transfer data between memory and uart buffer finish 
* @PARAM T_U8 buf_id: the buffer id
* @PARAM T_U8 tran_dir: the transfer direction
*/
void l2_wait_uartbuf_dmafinish(unsigned char uart_id, unsigned char tran_dir)
{
	//T_U8 buf_id;
	unsigned char buf_id;

	if (tran_dir)
		buf_id = 8 + uart_id * 2;
	else
		buf_id = 8 + uart_id * 2 + 1;

	//while(inl(L2_DMA_REQ)&(1<<(8+buf_id)))
	while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << (8 + buf_id)))	//0x2002c080
	{
	}
}

/**
* @BRIEF wait fraction DMA transfer data between memory and buffer finish 
*/
void l2_wait_frac_dmafinish(void)
{
	while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << 9))	//0x2002c080
	{
	}
}

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
void l2_uartbuf_tx_data(unsigned int ram_addr, unsigned char uart_id,
			unsigned int tran_byte)
{

	unsigned int tran_nbr = tran_byte >> 6;
	unsigned int fraction_nbr = tran_byte % 64;
	unsigned int buf_addr;
	volatile unsigned int reg_value;
	volatile unsigned int reg_id;
	unsigned int buf_id;

	buf_id = 8 + uart_id * 2;

	if (tran_nbr) {
		reg_value = ram_addr & 0xfffffff;
		//reg_id = L2_DMA_ADDR+ (buf_id<<2);    //0x2002c000 => 0x2002c03c
		reg_id = (unsigned int)AK88_L2CTR_DMAADDR + (buf_id << 2);	//0x2002c000 => 0x2002c03c
		__raw_writel(reg_value, reg_id);

		reg_value = tran_nbr & 0xff;
		reg_id = (unsigned int)AK88_L2CTR_DMACNT + (buf_id << 2);	//0x2002c040 => 0x2002c07c
		__raw_writel((volatile unsigned short)reg_value, reg_id);

		reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
		reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req
		reg_value |= (1 << (8 + buf_id));
		__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

		//wait uart dma finish
		//while(inl(L2_DMA_REQ)&(1<<(8+buf_id)))    //wait dma finish
		while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << (8 + buf_id)))	//0x2002c080   //wait dma finish
		{
		}
	}
	if (fraction_nbr) {
		reg_value = __raw_readl(AK88_L2CTR_DMAFRAC);	//0x2002c084
		reg_value &= ~(0x3ffffff);
		reg_value |= ((ram_addr + (tran_nbr << 6)) & 0x3ffffff);
		__raw_writel(reg_value, AK88_L2CTR_DMAFRAC);	//0x2002c084

		buf_addr =
		    (0x40 + ((buf_id - 8) << 1)) | ((tran_nbr % 2) & 0x1);
		reg_value = __raw_readl(AK88_L2CTR_DMAREQ);	//0x2002c080
		reg_value &= ~((0x7f << 1) | (0x3f << 10));
		reg_value &= ~((1 << 9) | (0xffffUL << 16));	//clear other buf req        
		reg_value |=
		    (1 << 9) | (1 << 8) | (buf_addr << 1) | ((fraction_nbr - 1)
							     << 10);
		__raw_writel(reg_value, AK88_L2CTR_DMAREQ);	//0x2002c080

		//wait frac dma finish
		//while(inl(L2_DMA_REQ)&(1<<9))        //wait dma finish
		while (__raw_readl(AK88_L2CTR_DMAREQ) & (1 << 9))	//0x2002c080   //wait dma finish
		{
		}
	}
}

/**
* @BRIEF transfer data between memory and l2 buffer with CPU mode
* @NOTE: if buffer is common buffer, the max value of tran_byte should be 512, if buffer is uart buffer, the
        max value of tran_byte should be 128.
*/
//T_VOID L2_TranDataCPU(T_U32 ram_addr, T_U8 buf_id, T_U32 buf_offset, T_U32 tran_byte, T_U8 tran_dir)
void l2_tran_data_cpu(unsigned int ram_addr, unsigned char buf_id,
		      unsigned int buf_offset, unsigned int tran_byte,
		      unsigned char tran_dir /*1=write(send),0=read(recv) */ )
{
 	unsigned int tran_nbr;
	unsigned int frac_nbr;
	volatile unsigned int temp_ram = 0, temp_buf = 0;
	unsigned int i, j;
	unsigned int buf_addr;

	if (buf_id < 8)		//common buffer
		buf_addr = (unsigned int)AK88_VA_L2BUF + (buf_id << 9);	//0x48000000 + (buf_id<<9);
	else if (buf_id < 16)	//uart buffer
		buf_addr =
		    (unsigned int)AK88_VA_L2BUF + 4096 /*8 * 512 */  +
		    ((buf_id - 8) << 7);
	else if (buf_id == 16)	//usb host buffer
		buf_addr = (unsigned int)AK88_VA_L2BUF + 6144 /*12 * 512 */ ;
	else if (buf_id <= 18)	//usb buffer
		buf_addr =
		    (unsigned int)AK88_VA_L2BUF + 6400 /*12 * 512 + 256 */  +
		    ((buf_id - 17) << 6);
	else {
		printk("invalid buf id %d\n", buf_id);
		return;
	}

	buf_addr += buf_offset;
	tran_nbr = tran_byte >> 2;
	frac_nbr = tran_byte % 4;	//frac_nbr = 1 ~ 3

	if (tran_dir)		//write(send)
	{			//memory to buffer ,write
		if (ram_addr % 4)	//ram_addr isn't 4 bytes align
		{
			for (i = 0; i < tran_nbr; i++) {
				temp_ram = 0;
				for (j = 0; j < 4; j++)
					temp_ram |=
					    ((read_ramb(ram_addr + i * 4 + j))
					     << (j * 8));
				write_buf(temp_ram, (buf_addr + i * 4));
			}
			if (frac_nbr) {
				temp_ram = 0;
				for (j = 0; j < frac_nbr; j++)
					temp_ram |=
					    ((read_ramb
					      (ram_addr + tran_nbr * 4 +
					       j)) << (j * 8));
				write_buf(temp_ram, (buf_addr + tran_nbr * 4));
			}
		} else		//4 bytes align
		{
			for (i = 0; i < tran_nbr; i++) {
				write_buf(read_raml(ram_addr + i * 4),
					  (buf_addr + i * 4));
			}
			if (frac_nbr) {
				write_buf(read_raml(ram_addr + tran_nbr * 4),
					  (buf_addr + tran_nbr * 4));
			}
		}
	} else			//read(recv)
	{
		if (ram_addr % 4)	//not 4 byte align
		{
			for (i = 0; i < tran_nbr; i++) {
				temp_buf = read_buf(buf_addr + i * 4);
				for (j = 0; j < 4; j++) {
					write_ramb((volatile unsigned
						    char)((temp_buf >> j *
							   8) & 0xff),
						   (ram_addr + i * 4 + j));
				}
			}
			if (frac_nbr) {
				temp_buf = read_buf(buf_addr + tran_nbr * 4);
				for (j = 0; j < frac_nbr; j++) {
					write_ramb((unsigned
						    char)((temp_buf >> j *
							   8) & 0xff),
						   (ram_addr + tran_nbr * 4 +
						    j));
				}
			}
		} else		//4 byte align
		{
			for (i = 0; i < tran_nbr; i++) {
				write_raml(read_buf(buf_addr + i * 4),
					   (ram_addr + i * 4));
			}
			if (frac_nbr)	//frac_nbr = 1 ~ 3
			{
				temp_buf = read_buf(buf_addr + tran_nbr * 4);
				temp_ram = read_raml(ram_addr + tran_nbr * 4);
				temp_buf &= ((1 << (frac_nbr * 8 + 1)) - 1);	//get frac_nbr valid bits(low bits)
				temp_ram &= ~((1 << (frac_nbr * 8 + 1)) - 1);	//clear frac_nbr bits(low bits),keep original ram high bits
				temp_ram |= temp_buf;	//merge original bits and frac_nbr bits
				write_raml(temp_ram, (ram_addr + tran_nbr * 4));
			}
		}
	}
}

/**
* @BRIEF transfer data from a buffer to another buffer
* @NOTE: all the parameter must be multiple of 4 bytes
*/
//T_VOID L2_LocalDMA(T_U32 dst_addr, T_U32 src_addr, T_U32 tran_byte)
void l2_localdma(unsigned int dst_addr, unsigned int src_addr,
		 unsigned int tran_byte)
{
	//T_U32 reg_value;
	volatile unsigned int reg_value;

	reg_value =
	    ((src_addr & 0x1ffc) >> 2) | (((dst_addr & 0x1ffc) >> 2) << 11) |
	    (((tran_byte >> 2) & 0xff) << 22) | (1 << 30);
 	__raw_writel(reg_value, AK88_L2CTR_LDMA_CFG);	// 0x2002c098

 	while (__raw_readl(AK88_L2CTR_LDMA_CFG) & (1 << 30))	// 0x2002c098
	{
	}
}

/**
* @BRIEF forcibly set a common buffer flag
* @NOTE: buffer id should be 0~7
*/

void l2_set_combuf_flag(unsigned char buf_id)
{
 	volatile unsigned int reg_value;

	//select the buf and set the buf full
 	reg_value = __raw_readl(AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
	reg_value &= (~0xff);
	reg_value |= (buf_id | (1 << 3) | (8 << 4));
	//outl(reg_value, L2_UARTBUF_CFG);
	__raw_writel(reg_value, AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
 
	//deselect the buf
 	reg_value = __raw_readl(AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
	reg_value &= (~0xff);
 	__raw_writel(reg_value, AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c

	//printk("L2_BUF_STATE2 = %x\n", *(volatile unsigned int*)(L2_BUF_STATE) );
}

void l2_change_set_flag(unsigned char buf_id, unsigned char set_status_flag)
{
 	volatile unsigned int temp;

	if (set_status_flag) {
 		temp = __raw_readl(AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
		temp |= buf_id | (1 << 3) | (set_status_flag << 4);
 		__raw_writel(temp, AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
	}
}

/**
* @BRIEF forcibly clear a common buffer flag
* @NOTE: buffer id should be 0~7
*/
void l2_clr_combuf_flag(unsigned char buf_id)	//buf_id = 0 ~ 15
{
	volatile unsigned int reg_value;


	if (buf_id <= 7) {
		reg_value = __raw_readl(AK88_L2CTR_COMBUF_CFG);	//0x2002c088
		reg_value |= 1 << (buf_id + 24);
		//bit[31:24]:bufn_flag_clr
		//bit24:buf0_flag_clr,1=To clear buf0 status flag
		//outl(reg_value, L2_COMBUF_CFG);
		__raw_writel(reg_value, AK88_L2CTR_COMBUF_CFG);	//0x2002c088
	} else {
		reg_value = __raw_readl(AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
		//reg_value |= (1<<(buf_id + 16)); ?
		reg_value |= (1 << (buf_id + 8));
		//bit[23:16] 
		//bit16:U1_Tx_clr,1=To clear UART1 TX Buf status flag(buf_id=8)
		//bit17:U1_Rx_clr,1=To clear UART1 RX Buf status flag(buf_id=9)

		__raw_writel(reg_value, AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
	}
}

/**
* @BRIEF forcibly clear a uart buffer status to 0
* @NOTE: buffer id should be 8~15
*/
void l2_clr_uartbuf_status(unsigned char buf_id /*8~15 */ )
{
	volatile unsigned int reg_value;

	reg_value = __raw_readl(AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c
	reg_value |= (1 << (8 + buf_id));	//buf_id = 8 ~ 15
	//bit[23:16] 
	//bit16:U1_Tx_clr,1=To clear UART1 TX Buf status flag(buf_id=8)
	//bit17:U1_Rx_clr,1=To clear UART1 RX Buf status flag(buf_id=9)
	//outl(reg_value, L2_UARTBUF_CFG);
	__raw_writel(reg_value, AK88_L2CTR_UARTBUF_CFG);	//0x2002c08c 
}

/**
* @BRIEF return a common buffer current flag 
* @RETURN T_BOOL: if return flag is 1, it mean buffer is full, otherwise mean buffer is empty 
* @NOTE: buffer id should be 0~7
*/
unsigned char l2_combuf_flag(unsigned char buf_id)	//buf_id = 0 ~ 7
{
	//return (T_U8)((inl(L2_STAT_REG1)>>(buf_id<<2))&0xf);
	return (volatile unsigned char)((__raw_readl(AK88_L2CTR_STAT_REG1) >> (buf_id << 2)) & 0xf);	//0x2002c0a0
}

/**
* @BRIEF return a uart buffer current status 
* @NOTE: buffer id should be 8~15
*/
unsigned char l2_uartbuf_status(unsigned char buf_id)	//buf_id = 8 ~ 15
{
	//return (T_U8)((inl(L2_STAT_REG2)>>((buf_id-8)<<1))&0x3);  //0x2002c0a8
	return (volatile unsigned char)((__raw_readl(AK88_L2CTR_STAT_REG2) >> ((buf_id - 8) << 1)) & 0x3);	//0x2002c0A8
}

//#endif

