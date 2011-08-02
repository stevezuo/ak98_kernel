/*
 * linux/arch/arm/mach-ak88/l2.c
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/stddef.h>
#include <linux/irq.h>
 
#include <asm/dma.h>
#include <asm/sizes.h>
 
#include <mach/l2.h>

static ak88_l2_buffer_info_t ak88_l2_buffer_info[AK88_L2_COMMON_BUFFER_NUM];
static ak88_l2_dma_info_t ak88_l2_dma_info[AK88_L2_COMMON_BUFFER_NUM + AK88_L2_UART_BUFFER_NUM];
static bool ak88_l2_frac_started = false;	/* L2 fraction DMA start flag */
static ak88_l2_device_info_t ak88_l2_device_info[] = {
	{ ADDR_USB_BULKOUT,	BUF_NULL },
	{ ADDR_USB_BULKIN,	BUF_NULL },
	{ ADDR_USB_ISO,		BUF_NULL },
	{ ADDR_NFC,		BUF_NULL },
	{ ADDR_MMC_SD,		BUF_NULL },
	{ ADDR_SDIO,		BUF_NULL },
	{ ADDR_RESERVED,	BUF_NULL },
	{ ADDR_SPI1_RX,		BUF_NULL },
	{ ADDR_SPI1_TX,		BUF_NULL },
	{ ADDR_DAC,		BUF_NULL },
	{ ADDR_SPI2_RX,		BUF_NULL },
	{ ADDR_SPI2_TX,		BUF_NULL },
	{ ADDR_PCM_RX,		BUF_NULL },
	{ ADDR_PCM_TX,		BUF_NULL },
	{ ADDR_ADC,		BUF_NULL },
};

static void ak88_l2_combuf_ctrl(u8 id, bool enable);
static void ak88_l2_select_combuf(ak88_l2_device_t device, u8 id);
static void ak88_l2_assert_combuf_id(u8 id);
static void ak88_l2_assert_buf_id(u8 id);
static void ak88_l2_clear_dma(u8 id);
static void ak88_l2_frac_dma(unsigned long ram_addr, u8 id, u8 frac_offset,
	unsigned int bytes, ak88_l2_dma_transfer_direction_t direction, bool intr_enable);
static u32 ak88_l2_get_addr(u8 id);
static void ak88_l2_dma(unsigned long ram_addr, u8 id, unsigned int bytes,
		ak88_l2_dma_transfer_direction_t direction, bool intr_enable);
static bool ak88_l2_wait_dma_finish(u8 id);
static void ak88_l2_cpu(unsigned long ram_addr, u8 id,
	unsigned long buf_offset, unsigned int bytes, ak88_l2_dma_transfer_direction_t direction);
static irqreturn_t ak88_l2_interrupt_handler(int irq, void *dev_id);

/**
 * ak88_l2_assert_buf_id - Assert a L2 buffer ID is valid
 *  @id:		L2 buffer ID
 *
 *  NOTE: Assert only L2 common buffer and UART buffer, USB buffer is not checked.
 *            Since this function is called internally by other L2 API, invalid id will cause
 *            linux kernel to oops for bug tracking.
 */
static void ak88_l2_assert_buf_id(u8 id)
{
	if (id >= AK88_L2_COMMON_BUFFER_NUM + AK88_L2_UART_BUFFER_NUM)
		BUG();
}

/**
 * ak88_l2_assert_combuf_id - Assert a L2 common buffer ID is valid
 *  @id:		L2 buffer ID
 *
 *  NOTE: Assert only L2 common buffer, UART & USB buffer is not checked.
 *            Since this function is called internally by other L2 API, invalid id will cause
 *            linux kernel to oops for bug tracking.
 */
static void ak88_l2_assert_combuf_id(u8 id)
{
	if (id >= AK88_L2_COMMON_BUFFER_NUM)
		BUG();
}

/**
 * ak88_l2_combuf_ctrl - L2 buffer enable/disable
 *  @id:		L2 buffer ID
 *  @enable:	true to enable L2 buffer, false to disable L2 buffer 
 */
static void ak88_l2_combuf_ctrl(u8 id, bool enable)
{
	unsigned long regval;
	unsigned long flags;

	ak88_l2_assert_buf_id(id);

	local_irq_save(flags);
	
	regval = __raw_readl(AK88_VA_L2_COMMON_BUF_CFG);
	if (enable) {
		/* Enable L2 buffer & L2 Buffer DMA */
		regval |= (1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_DMA_VLD_START)) |
			(1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_VLD_START));
	} else {
		/* Disable L2 buffer & L2 Buffer DMA */
		regval &= ~((1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_DMA_VLD_START)) |
			(1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_VLD_START)));
	}
	__raw_writel(regval, AK88_VA_L2_COMMON_BUF_CFG);

	local_irq_restore(flags);

}

/**
 * ak88_l2_select_combuf - Select a L2 buffer for given device
 *  @device:	Device which need to assign a L2 buffer
 *  @id:		L2 buffer ID
 */
static void ak88_l2_select_combuf(ak88_l2_device_t device, u8 id)
{
	unsigned long regid;
	unsigned long regval;
	unsigned long bits_offset;
	
	ak88_l2_assert_combuf_id(id);

	if ((u8)device < 10) {
		/*
		 * USB Bulkout ~ DAC (Device 0 ~ 9) is controlled by Buffer Assignment Register 1
		 */
		regid = (unsigned long)AK88_VA_L2_BUF_ASSIGN1;
		bits_offset = (u8)device * 3;
	} else {
		/*
		 * SPI2 Rx ~ ADC (Device 10 ~ 14) is controlled by Buffer Assignment Register 2
		 */
		regid = (unsigned long)AK88_VA_L2_BUF_ASSIGN2;
		bits_offset = ((u8)device - 10) * 3;
	}

	regval = __raw_readl(regid);
	regval &= ~(0x7 << bits_offset);
	regval |= ((id & 0x7) << bits_offset);
	__raw_writel(regval, regid);

}

/**
 * ak88_l2_clear_dma - Clear L2 buffer DMA status
 *  @id:		L2 buffer ID which need to clear DMA status
 */
static void ak88_l2_clear_dma(u8 id)
{
	bool dmapending;
	u8 status;
	
	dmapending = __raw_readl(AK88_VA_L2_DMA_REQ) & (1 << (id + AK88_L2_DMA_REQ_BUF_START));
	status = ak88_l2_get_status(id);

	if (status == 0) {
		return ;	/* NO DMA request, so do nothing */
	}

	/*
	 * Wait until DMA request of this L2 buffer is finished.
	 */
	while (dmapending) {
		printk("ak88-l2: unfinished DMA in buf[%d].\n", id);
		ak88_l2_clr_status(id);
		dmapending = __raw_readl(AK88_VA_L2_DMA_REQ) & (1 << (id + AK88_L2_DMA_REQ_BUF_START));
	}

}

/**
 * ak88_l2_frac_dma - Start data tranferring between memory and l2 common buffer in fraction DMA mode
 *  @ram_addr:		External RAM address(Physical)
 *  @id:		L2 buffer ID involved in DMA transfer
 *  @frac_offset:	The region offset between buffer start address and transfer start address
 *  @bytes:		Data transfer size
 *  @direction:		Data transfer direction between L2 memory and external RAM 
 *  @intr_enable:	Open interrupt for this L2 buffer or not
 *
 *  NOTE: Data transfer size should be 1~64Bytes, frac_offset should be 0~7 (*64Bytes)
 */
static void ak88_l2_frac_dma(unsigned long ram_addr, u8 id, u8 frac_offset,
	unsigned int bytes, ak88_l2_dma_transfer_direction_t direction,	bool intr_enable)
{
	u32 bufaddr;
	unsigned long regval;
	unsigned long flags;

#if 0
	printk("%s(): ram_addr=0x%08X, l2 buffer id=%d, frac_offset=%d, bytes=%d, direction=%s, intr_enable=%d.\n",
		__func__, (unsigned int)ram_addr, id, frac_offset, bytes, (direction == BUF2MEM)?"BUF2MEM":"MEM2BUF", intr_enable);
#endif

	if (bytes == 0) {
		printk("ak88-l2: no need to start fraction dma transfer: bytes=0.\n");
		return ;
	}

	local_irq_save(flags);

	/*
	 * Set fraction external RAM address.
	 */
	regval = __raw_readl(AK88_VA_L2_FRAC_DMA);
	regval &= ~AK88_L2_FRAC_DMA_ADDR_MASK;
	regval |= (ram_addr & AK88_L2_FRAC_DMA_ADDR_MASK);
	__raw_writel(regval, AK88_VA_L2_FRAC_DMA);

	/* Set fraction DMA address */
	bufaddr = (id < AK88_L2_COMMON_BUFFER_NUM) ? ((id & 0x7) << 3) | (frac_offset & 0x7) :
		(0x40 + ((id - AK88_L2_COMMON_BUFFER_NUM) << 1)) | (frac_offset & 0x1);

	/* Clear other fraction DMA request and info */
	regval = __raw_readl(AK88_VA_L2_DMA_REQ);
	regval &= ~(AK88_L2_DMA_REQ_FRAC_DMA_LEN_MASK | AK88_L2_DMA_REQ_FRAC_DMA_L2_ADDR_MASK |
		AK88_L2_DMA_REQ_FRAC_DMA_REQ | AK88_L2_DMA_REQ_BUF_REQ_MASK);

	switch (direction) {
	case MEM2BUF:
		if (bytes & 0x1)
			bytes = bytes + 1;	/* Round to even number when read data from external ram */
		regval |= AK88_L2_DMA_REQ_FRAC_DMA_REQ | AK88_L2_DMA_REQ_FRAC_DMA_DIR_WR |
			(bufaddr << AK88_L2_DMA_REQ_FRAC_DMA_L2_ADDR_START) |
			((bytes - 1) << AK88_L2_DMA_REQ_FRAC_DMA_LEN_START);
		__raw_writel(regval, AK88_VA_L2_DMA_REQ);
		break;
	case BUF2MEM:
		regval &= ~(AK88_L2_DMA_REQ_FRAC_DMA_DIR_WR);
		regval |= AK88_L2_DMA_REQ_FRAC_DMA_REQ |
			(bufaddr << AK88_L2_DMA_REQ_FRAC_DMA_L2_ADDR_START) |
			((bytes - 1) << AK88_L2_DMA_REQ_FRAC_DMA_LEN_START);
		__raw_writel(regval, AK88_VA_L2_DMA_REQ);
		break;
	default:
		BUG();
	}

	if (intr_enable) {
		regval = __raw_readl(AK88_VA_L2_INTR_ENABLE);
		regval |= AK88_L2_DMA_INTR_ENABLE_FRAC_INTR_EN;
		__raw_writel(regval, AK88_VA_L2_INTR_ENABLE);
	}

	local_irq_restore(flags);

	if (intr_enable)
		wait_event(ak88_l2_dma_info[id].wq, ak88_l2_dma_info[id].dma_irq_done);
}

/**
 * ak88_l2_get_addr - Get L2 memory start address for given L2 buffer
 *  @id:		L2 buffer ID
 *  Return L2 memory start address(Logical/Virtual) (NOT physical address)
 */
static u32 ak88_l2_get_addr(u8 id)
{
	u32 bufaddr = 0;

	if (id < AK88_L2_UART_BUFFER_INDEX) {	/* L2 common buffer */
		bufaddr = (u32)AK88_VA_L2MEM + AK88_L2_COMMON_BUFFER_OFFSET +
			id * AK88_L2_COMMON_BUFFER_LEN;
	} else if (id < AK88_L2_USB_HOST_BUFFER_INDEX) {	/* UART L2 buffer */
		bufaddr = (u32)AK88_VA_L2MEM + AK88_L2_UART_BUFFER_OFFSET + 
			(id - AK88_L2_COMMON_BUFFER_NUM) * AK88_L2_UART_BUFFER_LEN;
	} else if (id == AK88_L2_USB_HOST_BUFFER_INDEX) {	/* USB Host L2 buffer */
		bufaddr = (u32)AK88_VA_L2MEM + AK88_L2_USB_HOST_BUFFER_OFFSET;
	} else if (id < AK88_L2_USB_BUFFER_OFFSET) {	/* USB L2 buffer */
		bufaddr = (u32)AK88_VA_L2MEM + AK88_L2_USB_BUFFER_OFFSET +
			(id - AK88_L2_USB_BUFFER_INDEX) * AK88_L2_USB_BUFFER_LEN;
	} else {
		printk("ak88-l2: invalid buffer id %d.\n", (int)id);
	}

	return bufaddr;
}

/**
 * ak88_l2_dma - Start data tranferring between memory and l2 buffer in DMA mode
 *  @ram_addr:		External RAM address(Physical)
 *  @id:		L2 buffer ID involved in DMA transfer
 *  @bytes:		Data transfer size
 *  @direction:		Data transfer direction between L2 memory and external RAM 
 *  @intr_enable:	Open interrupt for this L2 buffer or not
 */
static void ak88_l2_dma(unsigned long ram_addr, u8 id, unsigned int bytes,
	ak88_l2_dma_transfer_direction_t direction, bool intr_enable)
{
	unsigned long regid;
	unsigned long regval;
	unsigned long flags;

#if 0
	printk("%s(): ram_addr=0x%0X, id=%d, bytes=%d, direction=%d, intr_enable=%d.\n",
		__func__, (unsigned int)ram_addr, id, bytes, direction, intr_enable);
#endif
	if (bytes == 0) {
		printk("ak88-l2: no need to start fraction dma transfer: bytes=0.\n");
		return ;
	}
	
	if (ak88_l2_dma_info[id].dma_start || ak88_l2_dma_info[id].dma_frac_start) {
		printk("ak88-l2: unable to start dma, dma NOT finished, buf id=%d.\n", (int)id);
		return ;
	}

	ak88_l2_dma_info[id].dma_op_times = bytes / AK88_DMA_ONE_SHOT_LEN;
	ak88_l2_dma_info[id].dma_frac_data_len = bytes % AK88_DMA_ONE_SHOT_LEN;
	ak88_l2_dma_info[id].dma_addr = (void *)ram_addr;
	ak88_l2_dma_info[id].direction = direction;
	ak88_l2_dma_info[id].intr_enable = intr_enable;
	ak88_l2_dma_info[id].need_frac = false;
	ak88_l2_dma_info[id].dma_irq_done = 0;

	if (ak88_l2_dma_info[id].dma_frac_data_len > 0) {
		ak88_l2_dma_info[id].need_frac = true;
		ak88_l2_dma_info[id].dma_frac_addr = (void *)(u8 *)ak88_l2_dma_info[id].dma_addr +
			ak88_l2_dma_info[id].dma_op_times* AK88_DMA_ONE_SHOT_LEN;
		ak88_l2_dma_info[id].dma_frac_offset = ak88_l2_dma_info[id].dma_op_times;
	}

	if (ak88_l2_dma_info[id].dma_op_times== 0) {
		/*
		 * If DMA transfer size < 64, we start fraction DMA immediately.
		 */
		 
		ak88_l2_dma_info[id].dma_start = false;
		ak88_l2_dma_info[id].dma_frac_start = true;

		ak88_l2_frac_dma((unsigned long)ak88_l2_dma_info[id].dma_frac_addr, id,
			ak88_l2_dma_info[id].dma_frac_offset, ak88_l2_dma_info[id].dma_frac_data_len,
			ak88_l2_dma_info[id].direction, intr_enable);
		return ;
	}
	ak88_l2_dma_info[id].dma_start = true;

	local_irq_save(flags);

	/*
	 * Set address of external RAM
	 */
	regval = (unsigned long)ak88_l2_dma_info[id].dma_addr & AK88_L2_DMA_ADDR_MASK;
	regid = (unsigned long)AK88_VA_L2_DMA_ADDR + id * 4;
	__raw_writel(regval, regid);

	/*
	 * Set DMA operation times
	 */
	regid = (unsigned long)AK88_VA_L2_DMA_OP_TIMES + id * 4;
	regval = ak88_l2_dma_info[id].dma_op_times& 0xFF;
	__raw_writew((u16)regval, regid);

	/*
	 * Set DMA direction for L2 common buffer
	 */
	if (id < AK88_L2_COMMON_BUFFER_NUM) {
		regval = __raw_readl(AK88_VA_L2_COMMON_BUF_CFG);
		if (ak88_l2_dma_info[id].direction == MEM2BUF) {
			regval |= (1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_DIR_START));;
		} else {
			regval &= ~(1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_DIR_START)); 
		}
		__raw_writel(regval, AK88_VA_L2_COMMON_BUF_CFG);
	}

	
	/*
	 * Start buffer DMA request
	 */
	regval = __raw_readl(AK88_VA_L2_DMA_REQ);
	regval &= ~(AK88_L2_DMA_REQ_FRAC_DMA_REQ | AK88_L2_DMA_REQ_BUF_REQ_MASK);
	if (id < AK88_L2_COMMON_BUFFER_NUM) {
		regval |= (1 << (id + AK88_L2_DMA_REQ_BUF_START));
	} else {
		regval |= (1 << ((id - AK88_L2_UART_BUF_START_ID + AK88_L2_UART_BUF_CFG_BUF_START)));
	}
	__raw_writel(regval, AK88_VA_L2_DMA_REQ);

	
	/*
	 * Enable DMA interrupt now
	 */
	if (intr_enable) {
		regval = __raw_readl(AK88_VA_L2_INTR_ENABLE);
		if (id < AK88_L2_COMMON_BUFFER_NUM) {
			regval |= 1 << (id + AK88_L2_DMA_INTR_ENABLE_BUF_START);
		} else {
			regval |= 1 << (id - AK88_L2_COMMON_BUFFER_NUM + AK88_L2_DMA_INTR_ENABLE_UART_BUF_START);
		}
		__raw_writel(regval, AK88_VA_L2_INTR_ENABLE);
	}

	local_irq_restore(flags);

	if (intr_enable)
		wait_event(ak88_l2_dma_info[id].wq, ak88_l2_dma_info[id].dma_irq_done);
}

/**
 * ak88_l2_wait_dma_finish - Wait for L2 DMA to finish
 *  @id:	L2 buffer ID involved in DMA transfer
 *  Return true: DMA transfer finished successfully.
 *            false: DMA transfer failed.
 *  NOTE: DMA transfer is started by ak88_l2_dma.
 */
static bool ak88_l2_wait_dma_finish(u8 id)
{
	unsigned int timeout;
	unsigned long dmareq;
	unsigned long dma_bit;
	const unsigned int max_wait_time = AK88_L2_MAX_DMA_WAIT_TIME;

	timeout = 0;
	if (ak88_l2_dma_info[id].dma_start) {
		dma_bit = (id < AK88_L2_COMMON_BUFFER_NUM) ? (1 << (id + AK88_L2_DMA_REQ_BUF_START)) :
			(1 << (id - AK88_L2_COMMON_BUFFER_NUM + AK88_L2_DMA_REQ_UART_BUF_REQ_START));
		do {
			dmareq = __raw_readl(AK88_VA_L2_DMA_REQ);
		} while((dmareq & dma_bit) && timeout++ < max_wait_time);

		ak88_l2_dma_info[id].dma_start = false;

		if (timeout >= max_wait_time) {
			printk("ak88-l2: wait dma timeout, buf id=%d, status=%d.\n", id, ak88_l2_get_status(id));
			ak88_l2_clear_dma(id);
			__raw_writel(0x0, AK88_VA_L2_DMA_OP_TIMES + id * 4);
			return false;
		}

		/*
		 * If fraction DMA  is NOT need, then everything is done.
		 */
		if (!ak88_l2_dma_info[id].need_frac) {
			return true;
		}	


		/*
		 * Start fraction DMA here for remain bytes transfer (<64Bytes).
		 */
		ak88_l2_dma_info[id].dma_frac_start = true;
		ak88_l2_frac_dma((unsigned long)ak88_l2_dma_info[id].dma_frac_addr, id,
			ak88_l2_dma_info[id].dma_frac_offset, ak88_l2_dma_info[id].dma_frac_data_len,
			ak88_l2_dma_info[id].direction, false);

	}

	/*
	 * Fraction DMA handling starts here.
	 */
	if (ak88_l2_dma_info[id].dma_frac_start) {
		timeout = 0;
		do {
			dmareq = __raw_readl(AK88_VA_L2_DMA_REQ);
		} while((dmareq & AK88_L2_DMA_REQ_FRAC_DMA_REQ) && (timeout++ < max_wait_time));

		ak88_l2_dma_info[id].dma_frac_start = false;

		if (timeout >= max_wait_time) {
			printk("ak88-l2:wait frac dma timeout, buf id=%d, status=%d.\n", id, ak88_l2_get_status(id));
			return false;
		}
	}

	return true;
}

/**
 * ak88_l2_interrupt_handler - L2 memory interrupt handler
 *  @irq:	IRQ number for L2 memory (Must be IRQ_L2MEM)
 *  @dev_id:	Device specific information used by interrupt handler
 *
 *  NOTE: Only shared IRQ need to check @irq & @dev_id.
 *            No need to check them here since L2 memory IRQ is NOT shared IRQ.
 */
static irqreturn_t ak88_l2_interrupt_handler(int irq, void *dev_id)
{
	unsigned long regval;
	int i = 0;

	regval = __raw_readl(AK88_VA_L2_DMA_REQ);

	for (i = 0; i < AK88_L2_COMMON_BUFFER_NUM; i++) {
		unsigned long dmapending = regval & (1 << ( i + AK88_L2_DMA_REQ_BUF_START));

		if (ak88_l2_dma_info[i].dma_start && !dmapending) {
			if (!ak88_l2_frac_started && ak88_l2_dma_info[i].need_frac) {
				ak88_l2_dma_info[i].dma_frac_start = true;
				ak88_l2_dma_info[i].dma_start = false;
				ak88_l2_dma_info[i].dma_irq_done = 0;

				ak88_l2_frac_dma((unsigned long)ak88_l2_dma_info[i].dma_frac_addr, i,
					ak88_l2_dma_info[i].dma_frac_offset, ak88_l2_dma_info[i].dma_frac_data_len,
					ak88_l2_dma_info[i].direction, true);

				ak88_l2_frac_started = true;
			} else {
				/* DMA has finished */
				unsigned long regval;

				regval = __raw_readl(AK88_VA_L2_INTR_ENABLE);
				regval &= ~(1 << (i + AK88_L2_DMA_INTR_ENABLE_BUF_START));
				__raw_writel(regval, AK88_VA_L2_INTR_ENABLE);

				ak88_l2_dma_info[i].dma_start = false;

				if (ak88_l2_dma_info[i].callback_func != NULL)
					ak88_l2_dma_info[i].callback_func();
				ak88_l2_dma_info[i].dma_irq_done = 1;
				wake_up(&ak88_l2_dma_info[i].wq);
				
			}
		}

		if (ak88_l2_dma_info[i].dma_frac_start) {
			unsigned long frac_dmapending = regval & AK88_L2_DMA_REQ_FRAC_DMA_REQ;
			if (ak88_l2_frac_started && !frac_dmapending) {
				ak88_l2_frac_started = false;

				switch (ak88_l2_dma_info[i].direction) {
				case MEM2BUF:
					if (ak88_l2_dma_info[i].dma_frac_data_len <= 60)
						__raw_writel(0x0, AK88_VA_L2MEM + i * 512 + 0x1FC);
					break;
				case BUF2MEM:
					if (ak88_l2_dma_info[i].dma_frac_data_len <= 512 - 4)
						ak88_l2_clear_dma(i);
					break;
				default:
					BUG();
				}
				ak88_l2_dma_info[i].dma_frac_start = false;

				if (ak88_l2_dma_info[i].callback_func != NULL)
					ak88_l2_dma_info[i].callback_func();
				
				ak88_l2_dma_info[i].dma_irq_done = 1;
				wake_up(&ak88_l2_dma_info[i].wq);
			}
		}
		
	}

	return IRQ_HANDLED;
}

/**
 * ak88_l2_cpu - Transfer data between memory and l2 buffer in CPU mode
 *  @ram_addr:		External RAM address(Physical)
 *  @id:		L2 buffer ID
 *  @buf_offset:	The buffer offset
 *  @bytes:		Data transfer size
 *  @direction:		Data transfer direction between L2 memory and external RAM 
 */
static void ak88_l2_cpu(unsigned long ram_addr, u8 id,
	unsigned long buf_offset, unsigned int bytes, ak88_l2_dma_transfer_direction_t direction)
{
	int i;
	int j;
	unsigned long trans_no;
	unsigned long frac_no;
	unsigned long buf_count;
	unsigned long buf_remain;
	unsigned long temp_ram;
	unsigned long temp_buf;
	unsigned long bufaddr;
	
	/*
	 * L2 buffer caller MUST guarantee L2 buffer offset is 4-byte aligned
	 */
	if (unlikely(buf_offset % 4))
		BUG();

	bufaddr = ak88_l2_get_addr(id);

	if (bufaddr == 0) {
		return ;
	}

	bufaddr += buf_offset;
	trans_no = bytes / 4;
	frac_no = bytes % 4;

	buf_count = (buf_offset + bytes) / AK88_L2_BUF_STATUS_MULTIPLY_RATIO;
	buf_remain = (buf_offset + bytes) % AK88_L2_BUF_STATUS_MULTIPLY_RATIO;

	switch (direction) {
	case MEM2BUF:
		if (ram_addr % 4) {
			for (i = 0; i < trans_no; i++) {
				temp_ram = 0;
				for (j = 0; j < 4; j++)
					temp_ram |= ((read_ramb(ram_addr + i*4 + j))<<(j*8));
				write_buf(temp_ram, (bufaddr + i * 4));
			}
			if (frac_no) {
				temp_ram = 0;
				for (i = 0; i < frac_no; i++)
					temp_ram |= ((read_ramb(ram_addr + trans_no*4 + j))<<(j*8));
				write_buf(temp_ram, (bufaddr + trans_no * 4));
			}
		} else {
			for (i = 0; i < trans_no; i++)
				write_buf(read_raml(ram_addr + i*4), (bufaddr + i*4));
			if (frac_no)
				write_buf(read_raml(ram_addr + trans_no*4), (bufaddr + trans_no*4));
		}
		
		/*
		 * If we do NOT write data to L2 in multiple of 64Bytes, we must write something to the 4Bytes in 64Bytes-
		 * boundary so that CPU knows writing ends..
		 */
		if ((buf_remain > 0) && (buf_remain <= AK88_L2_BUF_STATUS_MULTIPLY_RATIO - 4))
			write_buf(0, (bufaddr - buf_offset + buf_count*AK88_L2_BUF_STATUS_MULTIPLY_RATIO + AK88_L2_BUF_STATUS_MULTIPLY_RATIO - 4));
		break;
	case BUF2MEM:
		if (ram_addr % 4) {
			for (i = 0; i < trans_no; i++) {
				temp_buf = read_buf(bufaddr + i * 4);
				for (j = 0; j < 4; j++)
					write_ramb((u8)((temp_buf>>j*8) & 0xFF), (ram_addr + i*4 + j));
			}
			if (frac_no) {
				temp_buf = read_buf(bufaddr+trans_no*4);
				for (j = 0; j < frac_no; j++)
					write_ramb((u8)((temp_buf>>j*8) & 0xFF), (ram_addr + trans_no*4 + j));
			}
		} else {
			for (i = 0; i < trans_no; i++)
				write_raml(read_buf(bufaddr+i*4), (ram_addr+i*4));
			if (frac_no) {
				temp_buf = read_buf(bufaddr+trans_no*4);
				temp_ram = read_raml(ram_addr+trans_no*4);
				temp_buf &= ((1<<(frac_no*8+1))-1);
				temp_ram &= ~((1<<(frac_no*8+1))-1);
				temp_ram |= temp_buf;
				write_raml(temp_ram, (ram_addr+trans_no*4));
			}
		}
		
		/*
		 * If we do NOT read data from L2 in multiple of 64Bytes, we must read the 4Bytes in 64Bytes-
		 * boundary so that CPU knows reading ends..
		 */
		if ((buf_remain > 0) && (buf_remain <= AK88_L2_BUF_STATUS_MULTIPLY_RATIO - 4))
			temp_buf = read_buf(bufaddr-buf_offset+buf_count*AK88_L2_BUF_STATUS_MULTIPLY_RATIO+AK88_L2_BUF_STATUS_MULTIPLY_RATIO - 4);
		break;
	default:
		BUG();
	}

}



/**
 * ak88_l2_init - Initialize linux kernel L2 memory support
 */
void __init ak88_l2_init(void)
{
	int i;
	int retval;

	/*
	 * Initialize all L2 common buffer status to IDLE(could be allocated)
	 */
	for (i = 0; i < AK88_L2_COMMON_BUFFER_NUM; i++) {
		ak88_l2_buffer_info[i].id = (u8)i;
		ak88_l2_buffer_info[i].usable = L2_STAT_IDLE;
		ak88_l2_buffer_info[i].used_time = 0;
	}

	/* L2 Memory Register initializations */
	__raw_writel(AK88_L2_DMA_REQ_EN, AK88_VA_L2_DMA_REQ);
	__raw_writel(AK88_L2_FRAC_DMA_AHB_FLAG_EN | AK88_L2_FRAC_DMA_LDMA_FLAG_EN, AK88_VA_L2_FRAC_DMA);
	__raw_writel(0x0, AK88_VA_L2_COMMON_BUF_CFG);
	__raw_writel(AK88_L2_UART_BUF_CFG_UART_EN_MASK | AK88_L2_UART_BUF_CFG_UART_CLR_MASK, AK88_VA_L2_UART_BUF_CFG);
	__raw_writel(0x0, AK88_VA_L2_INTR_ENABLE);
	__raw_writel(0x0, AK88_VA_L2_BUF_ASSIGN1);
	__raw_writel(0x0, AK88_VA_L2_BUF_ASSIGN2);


	/* L2 Memory Interrupt handler registered */
	if ((retval = request_irq(IRQ_L2MEM, &ak88_l2_interrupt_handler, IRQF_DISABLED, "ak88-l2", NULL)) < 0)
		printk(KERN_ERR "ak88-l2: failed to request_irq, irq number: %d, retval=%d.\n", IRQ_L2MEM, retval);

	/* Initialize L2 DMA information status */
	memset(ak88_l2_dma_info, 0, ARRAY_SIZE(ak88_l2_dma_info));

	/* Initialize L2 DMA wait queue */
	for (i = 0; i < ARRAY_SIZE(ak88_l2_dma_info); i++)
		init_waitqueue_head(&ak88_l2_dma_info[i].wq);

	/* Initialize global L2 fraction DMA start flag */
	ak88_l2_frac_started = false;

	printk("AK88xx: L2 memory support initialized\n");
}

/**
 * ak88_l2_alloc - Allocate a common L2 buffer for given device
 *  @device:	Device ID which need common L2 buffer
 *  Return L2 buffer ID (0 ~ 7)
 *
 *  Only common L2 buffers(ID 0 ~ 7) could be allocated by ak88_l2_alloc.
 *  Other L2 buffers (UART/USB used) is handled by corresponding devices directly.
 */
u8 ak88_l2_alloc(ak88_l2_device_t device)
{
	int i;
	u16 used_times = MAX_L2_BUFFER_USED_TIMES;
	u8 id = BUF_NULL;
	u8 first_id = BUF_NULL;
	unsigned long flags;

	if (unlikely(device == ADDR_RESERVED)) {
		printk("ak88_l2: unable to allocate l2 buffer for reserved device.\n");
		
		return BUF_NULL;
	}

	if (unlikely(ak88_l2_device_info[(u8)device].id != BUF_NULL)) {
		printk("ak88_l2: device %d already have a l2 buffer %d\n",
			(int)(u8)device, (int)(u8)ak88_l2_device_info[(u8)device].id);
		
		return ak88_l2_device_info[(u8)device].id;
	}

	local_irq_save(flags);

	for (i = 0; i < AK88_L2_COMMON_BUFFER_NUM; i++) {
		if (ak88_l2_buffer_info[i].usable == L2_STAT_IDLE) {
			if (first_id == BUF_NULL) {
				first_id = ak88_l2_buffer_info[i].id;
				used_times = ak88_l2_buffer_info[i].used_time;
				id = first_id;
			}
			if (ak88_l2_buffer_info[i].used_time < used_times) {
				used_times = ak88_l2_buffer_info[i].used_time;
				id = ak88_l2_buffer_info[i].id;
			}
		}
	}

	if (unlikely(first_id == BUF_NULL)) {
		local_irq_restore(flags);
		printk(KERN_ERR "ak88-l2: fatal error: no more l2 buffer to allocate!\n");
		BUG();
		return BUF_NULL;
	}

	/*
	 * Got a L2 buffer successfully...
	 */
	ak88_l2_buffer_info[id].usable = L2_STAT_USED;
	ak88_l2_buffer_info[id].used_time++;
	if (ak88_l2_buffer_info[id].used_time == 0) {
		/*
		 * In case when the new allocated L2 buffer has been used MAX_L2_BUFFER_USED_TIMES,
		 * we just clear all L2 buffer used times as a simpfied method of balancing 8 L2 buffer usage.
		 */
		for (i = 0; i < AK88_L2_COMMON_BUFFER_NUM; i++)
			ak88_l2_buffer_info[i].used_time = 0;
	}

	/* Enable L2 buffer */
	ak88_l2_combuf_ctrl(id, true);

	/* Change device info */
	ak88_l2_device_info[device].id = id;

	/* Select L2 common buffer for device */
	ak88_l2_select_combuf(device, id);

	local_irq_restore(flags);

	/* Clear L2 buffer status */
	ak88_l2_clr_status(id);

	return id;
}
EXPORT_SYMBOL(ak88_l2_alloc);

/**
 * ak88_l2_free - Free L2 common buffer for given device
 *  @device:	Device ID which need common L2 buffer
 *  Return L2 buffer ID (0 ~ 7)
 *
 *  Only common L2 buffers(ID 0 ~ 7) could be allocated by ak88_l2_alloc.
 *  Other L2 buffers (UART/USB used) is handled by corresponding devices directly.
 *  NOTE: Return the previous L2 buffer ID if a L2 buffer has been allocated to the device.
 *            This means one device could get only one L2 buffer maximum.
 */
void ak88_l2_free(ak88_l2_device_t device)
{
	u8 id;
	unsigned long regval;
	unsigned long flags;

	id = ak88_l2_device_info[(u8)device].id;
	if (unlikely(id == BUF_NULL)) {
		printk("ak88-l2: trying to free invalid buffer id %d\n", (int)id);
		return ;
	}

	ak88_l2_clear_dma(id);

	local_irq_save(flags);

	/*
	 * Disable DMA interrupt of this L2 buffer.
	 */
	regval = __raw_readl(AK88_VA_L2_INTR_ENABLE);
	regval &= ~(1 << (id + AK88_L2_DMA_INTR_ENABLE_BUF_START));
	__raw_writel(regval, AK88_VA_L2_INTR_ENABLE);

	/* Set DMA count to 0 */
	__raw_writel(0x0, AK88_VA_L2_DMA_OP_TIMES + id * 4);

	/* Disable this L2 buffer */
	ak88_l2_combuf_ctrl(id, false);

	/* Clear DMA & DMA fraction flags */
	if (ak88_l2_dma_info[id].dma_start || ak88_l2_dma_info[id].dma_frac_start) {
		ak88_l2_dma_info[id].dma_start = false;
		ak88_l2_dma_info[id].dma_frac_start = false;
	}

	ak88_l2_device_info[(u8)device].id = BUF_NULL;
	ak88_l2_buffer_info[id].usable = L2_STAT_IDLE;

	local_irq_restore(flags);
}
EXPORT_SYMBOL(ak88_l2_free);

/**
 * ak88_l2_set_dma_callback - Set callback function when L2 DMA/fraction DMA interrupt handler is done
 *  @id:	L2 buffer ID
 *  @func:	Callback function
 *  Return true(Always)
 *  
 *  NOTE: Caller MUST guarantee that L2 buffer ID is valid. And since the callback function is called
 *  in interrupt handler, it MUST NOT call any functions which may sleep.
 */
bool ak88_l2_set_dma_callback(u8 id, ak88_l2_callback_func_t func)
{
	if (unlikely(id >= AK88_L2_COMMON_BUFFER_NUM)) {
		printk(KERN_ERR "ak88-l2: Set dma callback, invalid buf id[%d].\n", id);
		return false;
	}

	if (unlikely(ak88_l2_dma_info[id].dma_start || ak88_l2_dma_info[id].dma_frac_start)) {
		printk(KERN_ERR "ak88-l2: Set dma callback, dma not finished.\n");
		return false;
	}

	ak88_l2_dma_info[id].callback_func = func;

	return true;
}
EXPORT_SYMBOL(ak88_l2_set_dma_callback);

/**
 * ak88_l2_combuf_dma - Start data tranferring between memory and l2 common buffer in DMA mode
 *  @ram_addr:		External RAM address(Physical)
 *  @id:		L2 buffer ID involved in DMA transfer
 *  @bytes:		Data transfer size
 *  @direction:		Data transfer direction between L2 memory and external RAM 
 *  @intr_enable:	Open interrupt for this L2 buffer or not
 */
void ak88_l2_combuf_dma(unsigned long ram_addr, u8 id, unsigned int bytes, ak88_l2_dma_transfer_direction_t direction, bool intr_enable)
{
	if (unlikely(id >= AK88_L2_COMMON_BUFFER_NUM)) {
		printk("ak88-l2: begin common buffer dma, error buf id=[%d].\n", id);
		return ;
	}

	ak88_l2_dma(ram_addr, id, bytes, direction, intr_enable);
}
EXPORT_SYMBOL(ak88_l2_combuf_dma);

/**
 * ak88_l2_combuf_wait_dma_finish - Wait for L2 DMA to finish
 *  @id:	L2 buffer ID involved in DMA transfer
 *  Return true: DMA transfer finished successfully.
 *            false: DMA transfer failed.
 *  NOTE: DMA transfer is started by ak88_l2_combuf_dma.
 */
bool ak88_l2_combuf_wait_dma_finish(u8 id)
{
	if (unlikely(id >= AK88_L2_COMMON_BUFFER_NUM)) {
		printk("ak88-l2: begin common buffer dma, error buf id=[%d].\n", id);
		return false;
	}
	return ak88_l2_wait_dma_finish(id);
}
EXPORT_SYMBOL(ak88_l2_combuf_wait_dma_finish);

/**
 * ak88_l2_combuf_cpu - Transfer data between memory and l2 common buffer in CPU mode
 *  @ram_addr:	External RAM address(Physical)
 *  @id:	L2 buffer ID
 *  @bytes:	Data transfer size
 *  @direction:	Data transfer direction between L2 memory and external RAM
 *
 *  NOTE: According to XuChang, if one transfer data from Peripheral --> L2 Buffer --> RAM, 
 *            special care need to be taken when data size is NOT multiple of 64Bytes.
 *            Pheripheral driver must check hardware signals to confirm data has been transfer from
 *            peripheral to L2 buffer since L2 do NOT provide some mechanism to confirm data has
 *            been in L2 Buffer. Driver can and only can call ak88_l2_combuf_cpu() to copy data from L2
 *            Buffer --> RAM after checking hardware signals.
 *            As to 64Bytes * n size data, L2 could check Buffer Status Status Counter to confirm that
 *            Data has been transfer from peripheral to L2 buffer, so no hardware signals checking needed.
 */
void ak88_l2_combuf_cpu(unsigned long ram_addr, u8 id,
	unsigned int bytes, ak88_l2_dma_transfer_direction_t direction)
{
	int i;
	int loop;
	int remain;

	if (bytes > AK88_L2_BUFFER_SIZE) {
		printk("ak88-l2: l2_combuf_cpu buffer exceed L2 buffer size(512Bytes).\n");
		return ;
	}

	loop = bytes / AK88_L2_BUF_STATUS_MULTIPLY_RATIO;
	remain = bytes % AK88_L2_BUF_STATUS_MULTIPLY_RATIO;

	switch (direction) {
	case MEM2BUF:
		for (i = 0; i < loop; i++) {
			
			while (ak88_l2_get_status(id) == (AK88_L2_BUFFER_SIZE / AK88_L2_BUF_STATUS_MULTIPLY_RATIO))
				;	/* Waiting for L2 buffer to NOT full(means writable) */
			
			ak88_l2_cpu(ram_addr + i * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, id,
				(i % 8) * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, AK88_L2_BUF_STATUS_MULTIPLY_RATIO, direction);
		}
		if (remain > 0) {
			while (ak88_l2_get_status(id) > 0)
				;	/* Waiting for L2 buffer to empty */

			ak88_l2_cpu(ram_addr + loop * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, id,
				(loop % 8) * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, remain, direction);
		}
		break;
	case BUF2MEM:
		for (i = 0; i < loop; i++) {
			while (ak88_l2_get_status(id) == 0)
				;	/* Waiting for L2 buffer to be not empty (means readable) */
			
			ak88_l2_cpu(ram_addr + i * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, id,
				(i % 8) * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, AK88_L2_BUF_STATUS_MULTIPLY_RATIO, direction);
			
		}
		if (remain > 0) {
			ak88_l2_cpu(ram_addr + loop * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, id,
				(loop % 8) * AK88_L2_BUF_STATUS_MULTIPLY_RATIO, remain, direction);
		}
		break;
	default:
		BUG();
	}
}
EXPORT_SYMBOL(ak88_l2_combuf_cpu);

/**
 * ak88_l2_get_status - Get L2 buffer status
 *  @id:	L2 buffer ID
 */
u8 ak88_l2_get_status(u8 id)
{
	ak88_l2_assert_buf_id(id);

	return (id < AK88_L2_COMMON_BUFFER_NUM) ? (__raw_readl(AK88_VA_L2_BUF_STAT1) >> (id * 4)) & 0xF :
		(__raw_readl(AK88_VA_L2_BUF_STAT2) >> ((id - AK88_L2_UART_BUF_START_ID) << 1)) & 0x3;
}
EXPORT_SYMBOL(ak88_l2_get_status);

/**
 * ak88_l2_clr_status - Clear L2 buffer status
 *  @id:	L2 buffer ID
 */
void ak88_l2_clr_status(u8 id)
{
	unsigned long regval;
	unsigned long flags;

	ak88_l2_assert_buf_id(id);
	
	local_irq_save(flags);

	if (id < AK88_L2_COMMON_BUFFER_NUM) {
		regval = __raw_readl(AK88_VA_L2_COMMON_BUF_CFG);
		regval |= 1 << (id + AK88_L2_COMMON_BUF_CFG_BUF_CLR_START);
		__raw_writel(regval, AK88_VA_L2_COMMON_BUF_CFG);
	} else {
		regval = __raw_readl(AK88_VA_L2_UART_BUF_CFG);
		regval |= (1 << (id - AK88_L2_UART_BUF_START_ID + AK88_L2_UART_BUF_CFG_BUF_START));
		__raw_writel(regval, AK88_VA_L2_UART_BUF_CFG);
	}

	local_irq_restore(flags);

}
EXPORT_SYMBOL(ak88_l2_clr_status);

/**
 * ak88_l2_set_status - Clear L2 buffer status
 *  @id:	L2 buffer ID
 *  @status:	Status to be set (0 ~ 8)
 */
void ak88_l2_set_status(u8 id, u8 status)
{
	unsigned long regval;
	unsigned long flags;
	
	ak88_l2_assert_buf_id(id);

	if ((id >= AK88_L2_COMMON_BUFFER_NUM) || status > MAX_L2_DMA_STATUS_VALUE)
		BUG();

	local_irq_save(flags);

	/*
	 * Enable CPU-controlled buffer function and set L2 buffer `id' status
	 * status = current number of data in the CPU controlled buffer.
	 */
	regval = __raw_readl(AK88_VA_L2_UART_BUF_CFG);
	regval &= ~(AK88_l2_UART_BUF_CFG_CPU_BUF_NUM_MASK | AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_EN |
		AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_MASK);
	regval |= (id << AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_START) | AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_EN |
		(status << AK88_L2_UART_BUF_CFG_CPU_BUF_NUM_START);
	__raw_writel(regval, AK88_VA_L2_UART_BUF_CFG);

	/*
	* Disable CPU-controlled buffer function
	*/
	regval = __raw_readl(AK88_VA_L2_UART_BUF_CFG);
	regval &= ~(AK88_l2_UART_BUF_CFG_CPU_BUF_NUM_MASK | AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_EN |
		AK88_L2_UART_BUF_CFG_CPU_BUF_SEL_MASK);
	__raw_writel(regval, AK88_VA_L2_UART_BUF_CFG);

	local_irq_restore(flags);

}
EXPORT_SYMBOL(ak88_l2_set_status);
