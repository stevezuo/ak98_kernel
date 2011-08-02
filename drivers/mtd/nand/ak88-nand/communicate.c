/*******************************************************************************
 * #BRIEF communicate for AK3226 use the L2 Inter Globe Fifo.
 * Copyright (C) 2007 Anyka (Guangzhou) Software Technology Co., LTD
 * #AUTHOR Zou TianXiang
 * #DATE 2007-7-1
 ********************************************************************************/
#include <linux/kernel.h>
#include <mach/map.h>

#include "communicate.h"

#define L2_BASE_ADDR		AK88_VA_L2CTRL
#define L2_DMA_BLOCK_ADDR_CONF	AK88_VA_L2CTRL

#define L2_DMA_BLOCK_COUNT_CONF	AK88_VA_L2CTRL+0x40

#define L2_DMA_CRTL		(L2_BASE_ADDR+0x80)
#define L2_FRACTIOIN_DMA_CONF	(L2_BASE_ADDR+0x84)
#define L2_BLOCK_DMA_CONF	(L2_BASE_ADDR+0x88)
#define L2_FLAG_CONF		(L2_BASE_ADDR+0x8C)
#define L2_BUF_ID_CONF		(L2_BASE_ADDR+0x90)
#define L2_BUF2_ID_CONF		(L2_BASE_ADDR+0x94)
#define L2_LDMA_CONF		(L2_BASE_ADDR+0x98)
#define L2_INTR_EN		(L2_BASE_ADDR+0x9C)
#define L2_STATUS1		(L2_BASE_ADDR+0xA0)
#define L2_STATUS2		(L2_BASE_ADDR+0xA8)

#define FRAC_DMA_DIRECT_WRITE	(1<<8)
#define FRAC_DMA_DIRECT_READ	(0<<8)
#define FRAC_DMA_START_REQ	(1<<9)
#define	L2_DMA_ENABLE		(1)

#define L2_BUF_MEM_BASE_ADDR	AK88_VA_L2MEM

static T_U32 get_buf_id(PERIPHERAL_TYPE dev_type);

/*********************************************************************
Function:       communate_conf
Description:    Set the BUF ID Correspond to the peripheral
Input:    	  PERIPHERAL_TYPE dev_type
GLOBE_BUF_ID buf_id
Return:         0 : failed  1:  ok
 **********************************************************************/
T_U8 communicate_conf(PERIPHERAL_TYPE dev_type, GLOBE_BUF_ID buf_id)
{
	T_U32 buf_ena_val;
	T_U32 buf_id_val;
	//    T_U32 buf_status;
	T_U32 dam_ctrl_val;
	T_U32 current_buf_id;
    //printk(KERN_INFO "zz communicate.c communicate_conf() line 51\n");
	// Enable the flag effect.
	HAL_READ_UINT32(L2_FRACTIOIN_DMA_CONF, dam_ctrl_val);
	dam_ctrl_val |= (0x3 << 28);
	HAL_WRITE_UINT32(L2_FRACTIOIN_DMA_CONF, dam_ctrl_val);

	// Disable the current buf (if exist) and enable the new buf 
	HAL_READ_UINT32(L2_BLOCK_DMA_CONF, buf_ena_val);
	current_buf_id = get_buf_id(dev_type);
    //printk(KERN_INFO "zz communicate.c communicate_conf() 60 current_buf_id=0x%08x\n",
    //         current_buf_id);
	if (INVALID_BUF_ID != current_buf_id) {
		// current buf shall be empty
		/*
		   HAL_READ_UINT32(L2_STATUS1, buf_status);
		   if ((buf_status & (0xf<<(4*current_buf_id))) != 0)
		   printk("warning: L2 buf%d not empty when switch to another\n", current_buf_id);
		 */
		//printk(KERN_INFO "zz communicate.c communicate_conf() 69\n");
		set_buf_stat_empty(current_buf_id);
		buf_ena_val &= ~(1 << (16 + current_buf_id));
	}
	buf_ena_val |= (1 << (16 + buf_id));
	HAL_WRITE_UINT32(L2_BLOCK_DMA_CONF, buf_ena_val);
    //printk(KERN_INFO "zz communicate.c communicate_conf() 75 L2_BLOCK_DMA_CONF:0x%8x=0x%08x\n",
     //              L2_BLOCK_DMA_CONF, buf_ena_val);

	if (dev_type <= DAC) {
		// set the Buf Id for the peripheral dev
		HAL_READ_UINT32(L2_BUF_ID_CONF, buf_id_val);
		buf_id_val &= (~(0x7 << (dev_type * 3)));
		buf_id_val |= (buf_id << (dev_type * 3));
		HAL_WRITE_UINT32(L2_BUF_ID_CONF, buf_id_val);
        //printk(KERN_INFO "zz communicate.c communicate_conf() 84 L2_BUF_ID_CONF:0x%8x=0x%08x\n",
        //         L2_BUF_ID_CONF, buf_id_val);
	} else {
		// set the BUF ID for SPI2 or gps
		HAL_READ_UINT32(L2_BUF2_ID_CONF, buf_id_val);
		buf_id_val &= (~(0x7 << ((dev_type - SPI2_RECE) * 3)));
		buf_id_val |= (buf_id << ((dev_type - SPI2_RECE) * 3));
		HAL_WRITE_UINT32(L2_BUF2_ID_CONF, buf_id_val);
	}

	// clear the buf
	set_buf_stat_empty(buf_id);

	return AK_TRUE;
}

/*********************************************************************
Function:       prepare_dat_send_cpu
Description:    use CPU MODE to prepare the data to be sent
Input:    	  buf :			data buffer
len :			len < 512
dev_type :	the peripheral type
Return:         0 : failed  1:  ok
 **********************************************************************/
T_U8 prepare_dat_send_cpu(const T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id;
	T_U32 buf_status;
	T_U32 align_4_len, len_remian_4;
	T_U32 buf_addr;
	T_U32 *p_buf_int;
	T_U32 i;
	T_U32 tmp;
    //printk(KERN_INFO "zz communicate.c prepare_dat_send_cpu() 117\n");
	if (len > 512) {
		return 0;
	}

	align_4_len = (len & (~0x3));
	len_remian_4 = (len % 4);

	buf_id = get_buf_id(dev_type);
    //printk(KERN_INFO "zz communicate.c prepare_dat_send_cpu() 126 buf_id=%d\n", buf_id);;
	buf_addr = (T_U32) L2_BUF_MEM_BASE_ADDR + buf_id * 512;
	//set_buf_stat_empty((GLOBE_BUF_ID)buf_id);

	// wait till buffer empty
	do {
		HAL_READ_UINT32(L2_STATUS1, buf_status);
        //printk(KERN_INFO "zz communicate.c prepare_dat_send_cpu() 133 L2_STATUS1:0x%08x=0x%08x\n",
         //      L2_STATUS1, buf_status);
	} while ((buf_status & (0xf << (4 * buf_id))) != 0);

	if (((T_U32) buf % 4) == 0)	// if the buf is the even then it can 4 bytes write.
	{
		p_buf_int = (T_U32 *) buf;
		for (i = 0; i < align_4_len; i = i + 4) {
			//tmp = *(p_buf_int++);
			//WriteBuf(buf_addr+i, tmp);
			*(volatile T_U32 *)(buf_addr + i) = *(p_buf_int++);
		}
		if (len_remian_4 != 0) {
			tmp = 0;
			for (i = 0; i < len_remian_4; i++) {
				tmp = tmp + (buf[align_4_len + i] << (i * 8));
			}
			//WriteBuf(buf_addr+align_4_len, tmp);
			*(volatile T_U32 *)(buf_addr + align_4_len) = tmp;
		}
	}

	else			// if the buf is the odd then it can't 4 bytes write.
	{
		for (i = 0; i < align_4_len; i = i + 4) {
			tmp =
			    (buf[i] | (buf[i + 1] << 8) | (buf[i + 2] << 16) |
			     (buf[i + 3] << 24));
			//WriteBuf(buf_addr+i, tmp);
			*(volatile T_U32 *)(buf_addr + i) = tmp;
		}
		tmp = 0;
		if (len_remian_4 != 0) {
			for (i = 0; i < len_remian_4; i++) {
				tmp = tmp + (buf[align_4_len + i] << (i * 8));
			}
			//WriteBuf(buf_addr+align_4_len, tmp);
			*(volatile T_U32 *)(buf_addr + align_4_len) = tmp;
		}
	}

	//increase buf status
	if ((len % 64) && ((len % 64) <= 60)) {
		*(volatile T_U32 *)(buf_addr + (len & (~0x3f)) + 0x3c) = 0;
	}

	return 1;
}

/*********************************************************************
Function:       rece_dat_cpu
Description:    use CPU MODE to receive the data from L2 buf
Input:    	  buf :			data buffer
len :			len < 512
dev_type :	the peripheral type
Return:         0 : failed  1:  ok
 **********************************************************************/
T_U8 rece_dat_cpu(T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id;
	T_U32 buf_status;	//, buf_config;
	T_U32 align_4_len, len_remian_4;
	T_U32 buf_addr;
	T_U32 *p_buf_int;
	T_U32 i;
	T_U32 tmp;
    //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 190\n");
	if (len > 512) {
		//printk("error: L2 read too long\n");
		return 0;
	}

	buf_id = get_buf_id(dev_type);
    //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 197 buf_id=%d\n", buf_id);
	buf_addr = (T_U32)(L2_BUF_MEM_BASE_ADDR + buf_id * 512);
    //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 205 buf_addr=0x%08x\n", buf_addr);
	align_4_len = (len & (~0x3));
	len_remian_4 = (len % 4);

	// wait till 64-byte-aligned data have all been received
	do {
		HAL_READ_UINT32(L2_STATUS1, buf_status);
        //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 206 L2_STATUS1=0x%8x buf_status=0x%8x\n",
         //     L2_STATUS1, buf_status);
	} while (((buf_status >> (4 * buf_id)) & 0xf) < (len >> 6));
    //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 209\n");
	if (((T_U32) buf % 4) == 0)	// if the buf is the even then it can 4 bytes write.
	{
		p_buf_int = (T_U32 *) buf;
		for (i = 0; i < align_4_len; i = i + 4) {
			//*(p_buf_int++)  = ReadBuf(buf_addr+i);
			*(p_buf_int++) = *(volatile T_U32 *)(buf_addr + i);
		}
		if (len_remian_4 != 0) {
			//tmp = ReadBuf(buf_addr+align_4_len);
			tmp = *(volatile T_U32 *)(buf_addr + align_4_len);
			for (i = 0; i < len_remian_4; i++) {
				buf[align_4_len + i] = (tmp >> (8 * i)) & 0xff;
			}
		}
	}

	else			// if the buf is the odd then it can't 4 bytes write.
	{
		for (i = 0; i < align_4_len; i = i + 4) {
			//tmp = ReadBuf(buf_addr+i);
			tmp = *(volatile T_U32 *)(buf_addr + i);

			buf[i] = (tmp & 0xff);
			buf[i + 1] = ((tmp >> 8) & 0xff);
			buf[i + 2] = ((tmp >> 16) & 0xff);
			buf[i + 3] = ((tmp >> 24) & 0xff);
		}

		if (len_remian_4 != 0) {
			//tmp = ReadBuf(buf_addr + align_4_len);
			tmp = *(volatile T_U32 *)(buf_addr + align_4_len);
			for (i = 0; i < len_remian_4; i++) {
				buf[align_4_len + i] = (tmp >> (8 * i)) & 0xff;
			}
		}
	}
    //printk(KERN_INFO "zz communicate.c rece_dat_cpu() line 243\n");
	return 1;
}

/*********************************************************************
Function:       prepare_dat_send_dma
Description:    use DMA MODE to prepare the data to be sent
Input:    	  buf :			data buffer
len :			len < 512
dev_type :	the peripheral type
Return:         0 : failed  1:  ok
 **********************************************************************/
T_U8 prepare_dat_send_dma(const T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id;
	T_U32 dma_conf, frac_dma_conf, buf_status;
	T_U32 align_64_len, len_remian_64;
	T_U32 buf_addr;
    //printk(KERN_INFO "zz communicate.c prepare_dat_send_dma() line 261\n");
	if ((len > 2048) || (len == 0)) {
		return 0;
	}

	buf_id = get_buf_id(dev_type);
	if (INVALID_BUF_ID == buf_id) {
		//printk("ERROR: L2 buf not assigned, dev_type = %d\n", dev_type);
		return 0;
	}
	buf_addr = (T_U32) L2_BUF_MEM_BASE_ADDR + buf_id * 512;

	//set_buf_stat_empty((GLOBE_BUF_ID)buf_id);

	align_64_len = (len >> 6);	//        len / 64
	len_remian_64 = (len % 64);	//                len % 64

	// enable buf, enable buf dma function, set DMA direction
	HAL_READ_UINT32(L2_BLOCK_DMA_CONF, dma_conf);
	dma_conf |=
	    ((1 << buf_id) | (1 << (16 + buf_id)) | (1 << (8 + buf_id)));
	HAL_WRITE_UINT32(L2_BLOCK_DMA_CONF, dma_conf);

	//Config the L2 DMA to transfer the data of 64 align
	if (align_64_len) {
		// config the dma addr and count
		HAL_WRITE_UINT32(L2_DMA_BLOCK_ADDR_CONF + buf_id * 4,
				 ((T_U32) buf & 0xfffffff));
		HAL_WRITE_UINT32(L2_DMA_BLOCK_COUNT_CONF + buf_id * 4,
				 align_64_len);

		//start the dma
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf |= (L2_DMA_ENABLE | (1 << (24 + buf_id)));
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);
		do {
			HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		} while (dma_conf & (1 << (24 + buf_id)));
	}
	// Config the L2 DMA to transfer the data of fraction data
	if (len_remian_64) {
		const T_U8 *fdma_buf = buf + align_64_len * 64;	// RAM address
		T_U32 fdma_len = len_remian_64;	// data length

		// chip bug
		if (fdma_len & 1)	// len is odd
		{
			fdma_len++;
		}
		// Config the direction, fraction data base addr , fdma len
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf &= (~0xffff);
		dma_conf |=
		    (((buf_id * 512 / 64 +
		       (align_64_len %
			(512 /
			 64))) << 1) | FRAC_DMA_DIRECT_WRITE | ((fdma_len -
								 1) << 10));
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);

		// Config the fraction dma addr
		HAL_READ_UINT32(L2_FRACTIOIN_DMA_CONF, frac_dma_conf);
		frac_dma_conf &= ~0xfffffff;
		frac_dma_conf |= ((T_U32) fdma_buf) & 0xfffffff;
		HAL_WRITE_UINT32(L2_FRACTIOIN_DMA_CONF, frac_dma_conf);

		// wait till buffer is not full
		do {
			HAL_READ_UINT32(L2_STATUS1, buf_status);
		} while (((buf_status >> (4 * buf_id)) & 0xf) == 8);

		//start the fdma
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf |= (L2_DMA_ENABLE | FRAC_DMA_START_REQ);
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);

		do {
			HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		} while ((dma_conf & FRAC_DMA_START_REQ) == FRAC_DMA_START_REQ);

		//increase buf status
		if (len_remian_64 <= 64 - 4) {
			*(volatile T_U32 *)(buf_addr +
					    ((align_64_len * 64) % 512) +
					    0x3c) = 0;
		}
	}

	return 1;
}

T_U8 rece_data_dma(T_U8 * buf, T_U32 len, PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id;
	T_U32 align_64_len, len_remian_64;
	T_U32 dma_conf, frac_dma_conf;	//, buf_status;
	T_U32 buf_addr;
    //printk(KERN_INFO "zz communicate.c rece_data_dma() line 358\n");
	if ((len > 2048) || (len == 0)) {
		return 0;
	}

	buf_id = get_buf_id(dev_type);
	if (INVALID_BUF_ID == buf_id) {
		//printk("ERROR: L2 buf not assigned, dev_type = %d\n", dev_type);
		return 0;
	}
	buf_addr = (T_U32) L2_BUF_MEM_BASE_ADDR + buf_id * 512;

	align_64_len = (len >> 6);	//        len / 64
	len_remian_64 = (len % 64);	//                len % 64

	// enable buf, enable buf dma function, set DMA direction
	HAL_READ_UINT32(L2_BLOCK_DMA_CONF, dma_conf);
	dma_conf &= (~(1 << (8 + buf_id)));
	dma_conf |= ((1 << buf_id) | (1 << (16 + buf_id)));
	HAL_WRITE_UINT32(L2_BLOCK_DMA_CONF, dma_conf);

	//Config the L2 DMA to transfer the data of 64 align
	if (align_64_len) {
		// config the dma addr and count
		HAL_WRITE_UINT32(L2_DMA_BLOCK_ADDR_CONF + buf_id * 4,
				 ((T_U32) buf & 0xfffffff));
		HAL_WRITE_UINT32(L2_DMA_BLOCK_COUNT_CONF + buf_id * 4,
				 align_64_len);

		//start the dma
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf |= (L2_DMA_ENABLE | (1 << (24 + buf_id)));
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);
		do {
			HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		} while ((dma_conf & (1 << (24 + buf_id))) ==
			 (1 << (24 + buf_id)));
	}
	// *******************         NOTE:              *****************************
	// **** L2 alone do NOT know whether the rest of data have all been received from peripheral.
	// **** It is the peripheral driver that should make sure.

	// Config the L2 Fraction DMA to transfer the rest of data
	if (len_remian_64) {
		// Config the direct and fraction data base addr
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf &= (~0xffff);
		dma_conf |=
		    (((buf_id * 512 / 64 +
		       (align_64_len %
			(512 /
			 64))) << 1) | FRAC_DMA_DIRECT_READ | ((len_remian_64 -
								1) << 10));
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);

		// Config the fraction dma addr and count
		HAL_READ_UINT32(L2_FRACTIOIN_DMA_CONF, frac_dma_conf);
		frac_dma_conf &= ~0xfffffff;
		frac_dma_conf |=
		    ((T_U32) (buf + align_64_len * 64) & 0xfffffff);
		HAL_WRITE_UINT32(L2_FRACTIOIN_DMA_CONF, frac_dma_conf);

		//start the dma
		HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		dma_conf |= (L2_DMA_ENABLE | FRAC_DMA_START_REQ);
		HAL_WRITE_UINT32(L2_DMA_CRTL, dma_conf);

		do {
			HAL_READ_UINT32(L2_DMA_CRTL, dma_conf);
		} while ((dma_conf & FRAC_DMA_START_REQ) == FRAC_DMA_START_REQ);
	}

	return 1;
}

/*********************************************************************
Function:       get_buf_id
Description:    get the perpheral's correspond buf ID.
Input:    	  dev_type :	the peripheral type
Return:         BUF ID
**********************************************************************/
static T_U32 get_buf_id(PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id;
	T_U32 buf_en;
    //printk(KERN_INFO "zz communicate.c get_buf_id() line 443\n");
	//get the buf id
	if (dev_type <= DAC) {
		HAL_READ_UINT32(L2_BUF_ID_CONF, buf_id);
		buf_id = ((buf_id >> (dev_type * 3)) & 0x7);
		// to see if the buf is enabled
		HAL_READ_UINT32(L2_BLOCK_DMA_CONF, buf_en);
        //printk(KERN_INFO "zz communicate.c get_buf_id() 460 L2_BLOCK_DMA_CONF:0x%08x=0x%08x\n", 
         //      L2_BLOCK_DMA_CONF,buf_en);
		if ((buf_en & (1 << (buf_id + 16))) == 0) {
			buf_id = INVALID_BUF_ID;
		}
	} else {
		HAL_READ_UINT32(L2_BUF2_ID_CONF, buf_id);
		buf_id = ((buf_id >> ((dev_type - SPI2_RECE) * 3)) & 0x7);
	}
    //printk(KERN_INFO "zz communicate.c get_buf_id() 468 return buf_id=%d\n", buf_id);
	return buf_id;
}

/*********************************************************************
Function:       set_buf_stat_empty
Description:    set the buf to empty stat
Input:    	  dev_id :	the peripheral type
 *********************************************************************/
void set_buf_stat_empty(GLOBE_BUF_ID buf_id)
{
	T_U32 flag_conf;
    //printk(KERN_INFO "zz communicate.c set_buf_stat_empty() line 469\n");
	HAL_READ_UINT32(L2_BLOCK_DMA_CONF, flag_conf);
	flag_conf |= (1 << (buf_id + 24));
	HAL_WRITE_UINT32(L2_BLOCK_DMA_CONF, flag_conf);
}

/*********************************************************************
Function:       set_buf_empty
Description:    set the buf to empty stat
Input:    	  PERIPHERAL_TYPE dev_type
 **********************************************************************/
void set_buf_empty(PERIPHERAL_TYPE dev_type)
{
	T_U32 buf_id = get_buf_id(dev_type);
    //printk(KERN_INFO "zz communicate.c set_buf_empty() line 483\n");
	set_buf_stat_empty((GLOBE_BUF_ID) buf_id);
}

