/*
 * support l2 dma
 * 09-09-10 16:46:47
 */

#include <mach/ak880x_addr.h>

#define UDC_L2_ENABLE
//#undef UDC_L2_ENABLE

#include <mach/l2.h>

/* #define L2_BASE_ADDR	0x2002c000	//L2 PA*/
#define L2_BASE_ADDR	AK88_VA_L2CTRL	//L2 VA
/* #define L2_BUF_MEM_BASE_ADDR	    	0x48000000  //L2 Buffer start address */
#define	L2_DMA_ADDR	AK88_L2CTRL_REG(0x00)
#define	L2_DMA_CNT	AK88_L2CTRL_REG(0x40)
#define	L2_DMA_REQ	AK88_L2CTRL_REG(0x80)
#define	L2_FRAC_ADDR	AK88_L2CTRL_REG(0x84)
#define	L2_COMBUF_CFG	AK88_L2CTRL_REG(0x88)
#define	L2_UARTBUF_CFG	AK88_L2CTRL_REG(0x8c)
#define	L2_ASSIGN_REG1	AK88_L2CTRL_REG(0x90)
#define	L2_ASSIGN_REG2	AK88_L2CTRL_REG(0x94)
#define	L2_LDMA_CFG	AK88_L2CTRL_REG(0x98)
#define	L2_INT_ENA	AK88_L2CTRL_REG(0x9c)
#define	L2_STAT_REG1	AK88_L2CTRL_REG(0xa0)
#define	L2_STAT_REG2	AK88_L2CTRL_REG(0xa8)

#define L2_SD_BUFX	2 /* assign l2 buf2 to sd */ 
#define L2_USB_EP2	6 /* 4&&5 for nand */ 
#define L2_USB_EP3	7

#define l2_write(reg, val) __raw_writel((val), (reg))
#define l2_read(reg)	__raw_readl(reg)

#include "ak88_udc.h"
/* anyka */
#define T_VOID void
#define T_U8	unsigned char
#define T_U16	unsigned short
#define T_U32	unsigned int
#define ak_inl(reg)		__raw_readl(reg)
#define ak_outl(reg_value, reg)	__raw_writel((reg_value), (reg))
#define BUF2MEM                0
#define MEM2BUF                1
#define REG8(reg)	(*(volatile unsigned char *)(udc->baseaddr + reg))
#define REG32(reg)	(*(volatile unsigned int *)(udc->baseaddr + reg))

#ifdef UDC_L2_ENABLE
static void usb_l2_init(int buf_id, int is_in)
{
}
#else
T_VOID L2_ClrComBufFlag(T_U8 buf_id)
{
    T_U32 reg_value;

    if (buf_id<=7)
    {
        reg_value = ak_inl(L2_COMBUF_CFG);
        reg_value |= 1<<(buf_id + 24);
        ak_outl(reg_value, L2_COMBUF_CFG);
    }
    else
    {
        reg_value = ak_inl(L2_UARTBUF_CFG);
        reg_value |= (1<<(buf_id + 16));
        ak_outl(reg_value, L2_UARTBUF_CFG);
    }
}

T_VOID L2_SetComBufFlag(T_U8 buf_id)
{       
    T_U32 reg_value;        

    //select the buf and set the buf full
    reg_value = ak_inl(L2_UARTBUF_CFG);
    reg_value &= (~0xff);
    reg_value |= ( buf_id | (1<<3) | (8<<4) );
    ak_outl(reg_value, L2_UARTBUF_CFG);
    
    //printf("L2_BUF_STATE1 = %x\n", *(volatile unsigned int*)(L2_BUF_STATE) );
    
    //deselect the buf
    reg_value = ak_inl(L2_UARTBUF_CFG);
    reg_value &= (~0xff);
    ak_outl(reg_value, L2_UARTBUF_CFG);

    //printf("L2_BUF_STATE2 = %x\n", *(volatile unsigned int*)(L2_BUF_STATE) );
}

T_U8 L2_ComBufFlag(T_U8 buf_id)
{
    return (T_U8)((ak_inl(L2_STAT_REG1)>>(buf_id<<2))&0xf);
}

T_VOID L2_ComBufTranData(T_U32 ram_addr, T_U8 buf_id, T_U32 tran_byte, T_U8 tran_dir)
{
    T_U32 tran_nbr = tran_byte>>6;
    T_U32 fraction_nbr = tran_byte%64;
    T_U32 buf_addr;
    T_U32 reg_value;
    T_U32 reg_id;     
    int timeout;


    if (tran_dir == MEM2BUF)    //from SDRAM to L2
    {
		timeout = 5000;
        while((L2_ComBufFlag(buf_id)==8) && timeout)
		{ timeout--; }
    }
    
    if (tran_nbr)
    {
		reg_value = ram_addr & 0xfffffff;    
        reg_id = (T_U32)L2_DMA_ADDR + (buf_id<<2);    
        ak_outl(reg_value, reg_id);

        reg_value = tran_nbr & 0xff;    
        reg_id = (T_U32)L2_DMA_CNT + (buf_id<<2);    
        outw((T_U16)reg_value, reg_id);

        reg_value = ak_inl(L2_COMBUF_CFG);
		if (tran_dir)
            reg_value |= (1<<(8+buf_id));
		else
	    	reg_value &= ~(1 << (8 + buf_id));
        ak_outl(reg_value, L2_COMBUF_CFG);

        reg_value = ak_inl(L2_DMA_REQ);
		reg_value &= ~((1<<9)|(((T_U32)0xffff)<<16));        //clear other buf req
        reg_value |= (1<< (24 + buf_id));
        ak_outl(reg_value, L2_DMA_REQ);

		timeout = 5000;
		while(ak_inl(L2_DMA_REQ)&(1<<(24+buf_id)) && timeout)    //wait dma finish
		{ timeout--; }
    }
    
    if (fraction_nbr)
    {
        reg_value = ak_inl(L2_FRAC_ADDR);
        reg_value &= ~0xfffffff;
        reg_value |= ((ram_addr+(tran_nbr<<6))&0xfffffff);
        ak_outl(reg_value, L2_FRAC_ADDR);
        
        buf_addr = ((buf_id&0x7 )<<3) | (tran_nbr&0x7);
        reg_value = ak_inl(L2_DMA_REQ);
        reg_value &= ~((0x7f<<1)|(0x3f<<10));
        reg_value &= ~((1<<9)|(((T_U32)0xffff)<<16));        //clear other buf req    
     
        if (tran_dir)
        {
			if (fraction_nbr & 0x1)
				reg_value |= (1<<9)|(1<<8)|(buf_addr<<1)|(fraction_nbr << 10);
			else
				reg_value |= (1<<9)|(1<<8)|(buf_addr<<1)|((fraction_nbr - 1)<< 10);
        }
		else {
			reg_value &= ~(1 << 8);
			reg_value |= (1<<9)|(buf_addr<<1)|((fraction_nbr-1)<< 10);    
		}
                    
        ak_outl(reg_value, L2_DMA_REQ);

		timeout = 5000;
		while(ak_inl(L2_DMA_REQ)&(1<<9) && timeout)        //wait dma finish
		{ timeout--; }
    }
}

/* is_in:1 ==> ep2   
 * is_in:0 ==> ep3
 */
static void usb_l2_init(int buf_id, int is_in)
{
	unsigned int regval;

	regval = ak_inl(L2_DMA_REQ);
	regval |= 0x1;
	ak_outl(regval, L2_DMA_REQ);
	regval = l2_read(L2_COMBUF_CFG);
	regval |= ((1<<buf_id) | (1<<(16+buf_id)));
	l2_write(L2_COMBUF_CFG, regval);

	regval = l2_read(L2_ASSIGN_REG1);
	if (is_in) {
		regval &= ~(0x7<<0); /* ep2 */ 
		l2_write(L2_ASSIGN_REG1, regval | (buf_id));
	} else { 
		regval &= ~(0x7<<3); /* ep3 */ 
		l2_write(L2_ASSIGN_REG1, regval | (buf_id<<3));
	}

	regval = l2_read(L2_COMBUF_CFG);
	l2_write(L2_COMBUF_CFG, regval | 0x1<<(buf_id+24));
}

static void usb_l2_ep2(int buf_id, unsigned int buf, dma_addr_t phys, unsigned int len, int is_in)
{
	L2_ClrComBufFlag(buf_id);

	L2_ComBufTranData(phys, buf_id, len, is_in);

	L2_SetComBufFlag(buf_id);
}

static void usb_l2_ep3(int buf_id, unsigned char *buf, dma_addr_t phys, unsigned int len, int is_in)
{
	L2_SetComBufFlag(buf_id);
	
	L2_ComBufTranData(phys, buf_id, len, is_in);
}
#endif
