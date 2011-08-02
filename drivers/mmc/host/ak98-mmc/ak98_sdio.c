/*
 *  linux/drivers/mmc/host/ak98_sdio.c - ak98 MMC/SD/SDIO driver
 *
 *  Copyright (C) 2010 Anyka, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/scatterlist.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/l2.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/regs-comm.h>
#include <mach/ak98_sdio.h>

#define DRIVER_NAME "ak98_sdio"

//#define AKMCI_INNERFIFO_PIO /* only 4bytes inner fifo */
#define AKMCI_L2FIFO_PIO
//#define AKMCI_L2FIFO_DMA

#define hydbg(fmt,args...) //printk("%s(%d):" fmt, __func__,__LINE__,##args)

#define DBG(host,fmt,args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

#define PK1(fmt...) 	//printk(fmt)
#define PK(fmt...) 	 	//printk(fmt)
#define PKCLK(fmt...) 	//printk(fmt)

static u8    l2_sdio_bufid = BUF_NULL ;

#if defined AKMCI_L2FIFO_PIO || defined AKMCI_L2FIFO_DMA
static unsigned int fmax = (20*1000*1000);
#elif defined AKMCI_INNERFIFO_PIO
static unsigned int fmax = (4*1000*1000);
#else
#error "Please select one FIFO translation mode!"
#endif

#if 0
static void ak98_sdio_dump_regs(void *base)
{
	int i;

	for (i = 0; i <= 0x40; i+=4) {
		PK("%02x - %08x\n", i, ioread32(base+i));
	}
}

static void dump_data(struct mmc_data *data)
{
	struct scatterlist *sg;
	u8 *sg_dat, *sg_end;
	unsigned int blks, blkdat;

	printk("%s\n", __func__);

	sg = data->sg;
	sg_dat = sg_virt(sg);
	sg_end = sg_dat + sg->length;

	for (blks = 0; blks < data->blocks; blks++) {
		for (blkdat = 0; blkdat < data->blksz; blkdat++) {
			printk("%02X ", *sg_dat);
			if ((blkdat % 16) == 15)
				printk("\n");
			sg_dat++;
			if (sg_dat >= sg_end) {
				sg = sg_next(sg);
				if (sg == NULL)
					break;
				sg_dat = sg_virt(sg);
				sg_end = sg_dat + sg->length;
			}
		}
		printk("\n");
	}
}
#endif
#ifdef AKMCI_L2FIFO_PIO
/**
 * @brief transmitting data.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of data transmitted, including data buf pointer, data len .
 * @return void.
 */

static void mci_xfer(struct ak98_mci_host *host)
{
	int  sg_remain;
	u32 *tempbuf,xferlen;
	u8 dir;

	PK("%s\n", __func__);

	if (host->data->flags & MMC_DATA_WRITE) {		
		dir = MEM2BUF;
	} else {
	    //ak98_l2_clr_status(l2_sdio_bufid);
		dir = BUF2MEM;
	}	

    tempbuf = sg_virt(host->sg_ptr) + host->sg_off;
    sg_remain = host->sg_ptr->length - host->sg_off;
    
    if (sg_remain <= 0)
    {        
        host->sg_ptr = sg_next(host->sg_ptr);        
        if (host->sg_ptr == NULL)
				return;
				
		host->sg_off = 0;
		tempbuf = sg_virt(host->sg_ptr) + host->sg_off;
        sg_remain = host->sg_ptr->length - host->sg_off;
    }
  
    xferlen = (sg_remain > host->data->blksz) ? host->data->blksz : sg_remain;    
    ak98_l2_combuf_cpu((unsigned long)tempbuf, l2_sdio_bufid, xferlen, dir); 
    host->sg_off += xferlen;
    host->data_xfered += xferlen;    
		
}
#endif

/**
 * @brief stop data, close interrupt.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host get the base address of resgister.
 * @return void.
 */

static void ak98_sdio_stop_data(struct ak98_mci_host *host)
{
	u32 masks;

	PK1("%s\n", __func__);

	writel(0, host->base + AK98MCIDMACTRL);
	writel(0, host->base + AK98MCIDATACTRL);
	masks = readl(host->base + AK98MCIMASK);
	masks &= ~(MCI_DATAIRQMASKS|MCI_FIFOFULLMASK|MCI_FIFOEMPTYMASK);
	writel(masks, host->base + AK98MCIMASK);
	PK("DISABLE DATA IRQ\n"); 
     
#ifdef MCI_USE_L2FIFO_DMA
	if (host->data->flags & MMC_DATA_WRITE) {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc), host->data->sg, host->data->sg_len, DMA_TO_DEVICE);
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->data->sg_len, DMA_TO_DEVICE);
	} else {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc), host->data->sg, host->data->sg_len, DMA_FROM_DEVICE);
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->data->sg_len, DMA_FROM_DEVICE);
	}
#endif

	host->data = NULL; 
	
}

/**
 * @brief  finish a request,release resource.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of sd controller.
 * @param [in] *mrq information of request.
 * @return void.
 */

static void ak98_sdio_request_end(struct ak98_mci_host *host, struct mmc_request *mrq)
{
    int not_retry = 0;

    PK1("%s\n", __func__);

	writel(0, host->base + AK98MCICOMMAND); 
	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

    if(l2_sdio_bufid != BUF_NULL)
    {
        ak98_l2_free(ADDR_SDIO);
        l2_sdio_bufid = BUF_NULL;
    }
    
	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;
	
	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);    

    not_retry = (!mrq->cmd->error) || ((mrq->cmd->error && (mrq->cmd->retries == 0)));
	
    mmc_request_done(host->mmc, mrq);

#ifdef CONFIG_CPU_FREQ
	 /*if request fail,then mmc_request_done send request again, 
	   * ak98_mci_send_request not down freq_lock in interrupt,so not to unlock freq_lock.
	   */

	 if (not_retry) 
	 {					   
		 up(&host->freq_lock);
	 }	  
#endif

    
	spin_lock(&host->lock);
}

/**
 * @brief  config sd controller, start transmitting data.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of sd controller.
 * @param [in] *data information of data transmitted.
 * @return void.
 */

static void ak98_sdio_start_data(struct ak98_mci_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout;
	unsigned long long clks;
	void __iomem *base;
    
	PK("%s: blksz %04x blks %04x flags %08x\n",
	       __func__, data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz * data->blocks;
	host->data_xfered = 0;

	ak98_mci_init_sg(host, data);  
	
	clks = (unsigned long long)data->timeout_ns * host->bus_clkrate;
	do_div(clks, 1000000000UL);
	timeout = data->timeout_clks + (unsigned int)clks;
	
	PK("timeout: %uns / %uclks, clks=%d\n", data->timeout_ns, data->timeout_clks,clks);

	base = host->base;
	writel(timeout, base + AK98MCIDATATIMER);
	writel(host->size, base + AK98MCIDATALENGTH);

	/* set l2 fifo info */
	writel (MCI_DMA_BUFEN | MCI_DMA_SIZE(MCI_L2FIFO_SIZE/4),
		base + AK98MCIDMACTRL);

#ifdef AKMCI_L2FIFO_DMA
    u32    regval;	

	/* get l2 fifo */
	regval = readl(host->l2base + L2FIFO_ASSIGN1);
	regval = (regval & (~(3<<12))) | (MCI_L2FIFO_NUM << 12);
	writel(regval, host->l2base + L2FIFO_ASSIGN1);

	regval = readl(host->l2base + L2FIFO_CONF1);
	regval |= (1 << (0 + MCI_L2FIFO_NUM))
		| (1 << (16 + MCI_L2FIFO_NUM))
		| (1 << (24 + MCI_L2FIFO_NUM));
	if (data->flags & MMC_DATA_WRITE)
		regval |= (1 << (8 + MCI_L2FIFO_NUM));
	else
		regval &= ~(1 << (8 + MCI_L2FIFO_NUM));
	writel(regval, host->l2base + L2FIFO_CONF1);

	/* set dma addr */
	if (data->flags & MMC_DATA_WRITE)
		dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, DMA_TO_DEVICE);
	else
		dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, DMA_FROM_DEVICE);
	writel(sg_dma_address(data->sg), host->l2base + MCI_L2FIFO_NUM);

	/* set dma size */
	if (host->size > L2DMA_MAX_SIZE)
		dma_size = L2DMA_MAX_SIZE;
	dma_times = dma_size/64;
	writel(dma_times, host->l2base + 0x40 + MCI_L2FIFO_NUM);

	if (host->size > L2DMA_MAX_SIZE) {
		/* need to handle dma int */
		regval = readl(host->l2base + L2FIFO_INTEN);
		regval |= (1 << (9 + MCI_L2FIFO_NUM));
		writel(regval, host->l2base + L2FIFO_INTEN);

		request_irq(AK88_L2MEM_IRQ(x)(9+MCI_L2FIFO_NUM), ak98_mcil2_irq,
			    IRQF_DISABLED, DRIVER_NAME "(dma)", host);
	}

	/* when to start dma? */
	regval = readl(host->l2base + L2FIFO_DMACONF);
	regval |= (1 | (1 << (24 + MCI_L2FIFO_NUM)));
	writel(regval, host->l2base + L2FIFO_DMACONF);

	if (dma_size % 64) {
		/* fraction DMA */
		(8 * MCI_L2FIFO_NUM)
	}

	/* set l2 fifo info */
	writel (MCI_DMA_BUFEN | MCI_DMA_EN | MCI_DMA_SIZE(MCI_L2FIFO_SIZE/4),
		base + AK98MCIDMACTRL);
#endif

	datactrl = MCI_DPSM_ENABLE;

	switch (host->bus_width) {
	case MMC_BUS_WIDTH_8:
		datactrl |= MCI_DPSM_BUSMODE(2);
		break;
	case MMC_BUS_WIDTH_4:
		datactrl |= MCI_DPSM_BUSMODE(1);
		break;
	case MMC_BUS_WIDTH_1:
	default:
		datactrl |= MCI_DPSM_BUSMODE(0);
		break;
	}

	if (data->flags & MMC_DATA_STREAM) {
		DBG(host, "%s", "STREAM Data\n");
		datactrl |= MCI_DPSM_STREAM;
	} else {
		DBG(host, "BLOCK Data: %u x %u\n", data->blksz, data->blocks);
		datactrl |= MCI_DPSM_BLOCKSIZE(data->blksz);
	}

	if (data->flags & MMC_DATA_READ) {
		datactrl |= MCI_DPSM_DIRECTION;
	}

	writel(readl(base + AK98MCIMASK) | MCI_DATAIRQMASKS, base + AK98MCIMASK);
	writel(datactrl, base + AK98MCIDATACTRL);

	PK("ENABLE DATA IRQ, datactrl: 0x%08x, timeout: 0x%08x, len: %u\n",
	       datactrl, readl(base+AK98MCIDATATIMER), host->size);

#ifdef AKMCI_L2FIFO_PIO
	if (data->flags & MMC_DATA_WRITE)
		mci_xfer(host);
#endif

#ifdef AKMCI_INNERFIFO_PIO
    unsigned int irqmask;
    
	irqmask = readl(base + AK98MCIMASK);
	if (data->flags & MMC_DATA_READ) {
		if (host->size > MCI_FIFOSIZE)
			irqmask |= MCI_FIFOFULLMASK;
		else
			;	/* wait for DATAEND int */
	} else {
		irqmask |= MCI_FIFOEMPTYMASK;
	}

	writel(irqmask, base + AK98MCIMASK);
#endif
}

/**
 * @brief  config sd controller, start sending command.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of sd controller.
 * @param [in] *cmd information of cmd sended.
 * @return void.
 */

static void ak98_sdio_start_command(struct ak98_mci_host *host, struct mmc_command *cmd)
{
	unsigned int c;
	void __iomem *base = host->base;

	PK1("%s: op %i arg 0x%08x flags 0x%08x\n",
	       __func__, cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + AK98MCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + AK98MCICOMMAND);
		udelay(1);
	}

	c = MCI_CPSM_CMD(cmd->opcode) | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		c |= MCI_CPSM_RESPONSE;
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
	}

	if (cmd->data)
		c |= MCI_CPSM_WITHDATA;

	host->cmd = cmd;

	writel(cmd->arg, base + AK98MCIARGUMENT);
	writel(readl(base + AK98MCIMASK) | MCI_CMDIRQMASKS, base + AK98MCIMASK);
	PK("ENABLE CMD IRQ\n");
	PK("irqmask: 0x%08x\n", readl(base+AK98MCIMASK));
	writel(c, base + AK98MCICOMMAND);
}


#ifdef AKMCI_INNERFIFO_PIO
static void ak98_sdio_pio_irq(struct ak98_mci_host *host, unsigned int status)
{
	u32 *p;

	if (host->sg_ptr == NULL) {
		printk("%s ERROR\n", __func__);
		return;
	}

	p = sg_virt(host->sg_ptr) + host->sg_off;

	if ((status & MCI_FIFOFULL) && (status & MCI_RXACTIVE)) {
		*p = readl(host->base + AK88MCIFIFO);
		PK("read: 0x%08x\n", *p);
	} else if ((status & MCI_FIFOEMPTY) && (status & MCI_TXACTIVE)) {
		writel(*p, host->base + AK88MCIFIFO);
		PK("write: 0x%08x\n", *p);
	} else {
		return;
	}

	host->data_xfered += 4;
	host->size -= 4;

	host->sg_off += 4;
	if (host->sg_off >= host->sg_ptr->length) {
		ak98_mci_next_sg(host);
	}
}
#endif

/**
 * @brief  enable or disable sdio interrupt.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *mmc information of sd controller.
 * @param [in] enable  1: enable; 0: disable.
 * @return void.
 */

void ak98_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
    struct ak98_mci_host *host = mmc_priv(mmc);	
    unsigned reg1,reg2;
    unsigned long flags;
    
    spin_lock_irqsave(&host->lock, flags); 

    reg1 = readl(host->base + AK98MCIMASK);
    reg2 = readl(host->base + AK98SDIOINTRCTR);
    
    if (enable)
    {
        reg1 |= SDIO_INTR_ENABLE; 
        reg2 |= SDIO_INTR_CTR_ENABLE;
    }
    else
    {
        reg1 &= ~SDIO_INTR_ENABLE;
        reg2 &= ~SDIO_INTR_CTR_ENABLE;
    }
    
    writel(reg2, host->base + AK98SDIOINTRCTR);    
    writel(reg1, host->base + AK98MCIMASK);
    spin_unlock_irqrestore(&host->lock, flags); 
}

/**
 * @brief  data handle in sdio interrupt.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of sd controller.
 * @param [in] *data information of data transmitting.
 * @return void.
 */

static void ak98_sdio_data_irq(struct ak98_mci_host *host, struct mmc_data *data,
		  unsigned int status)
{
	if (status & MCI_DATABLOCKEND) {
		PK("BLOCKEND\n");
#ifdef AKMCI_L2FIFO_PIO
        if (data->flags & MMC_DATA_WRITE)
        {
            ak98_l2_clr_status(l2_sdio_bufid);
        }
		if (host->size > 0)
			mci_xfer(host);
		
#endif
	}
	if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT)) {
		PK1("DATA ERROR: 0x%08x\n", status);

		if (status & MCI_DATACRCFAIL )
			data->error = -EILSEQ;
		else if (status & MCI_DATATIMEOUT)
			data->error = -ETIMEDOUT;
		status |= MCI_DATAEND;
		/*
		 * We hit an error condition.  Ensure that any data
		 * partially written to a page is properly coherent.
		 */
		if (host->sg_len && data->flags & MMC_DATA_READ)
			flush_dcache_page(sg_page(host->sg_ptr));
	}
	if (status & MCI_DATAEND) {
		ak98_sdio_stop_data(host);

		//dump_data(data);

		if (!data->stop) {
			ak98_sdio_request_end(host, data->mrq);
		} else {
			ak98_sdio_start_command(host, data->stop);
		}
	}
}

/**
 * @brief  cmd handle in sd interrupt.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *host information of sd controller.
 * @param [in] *cmd information of cmd sended.
  *@param [in] *status the status of sd controller.
 * @return void.
 */

static void ak98_sdio_cmd_irq(struct ak98_mci_host *host, struct mmc_command *cmd,
		 unsigned int status)
{
	void __iomem *base = host->base;

	PK("+%s\n", __func__);
	host->cmd = NULL;

	cmd->resp[0] = readl(base + AK98MCIRESPONSE0);
	cmd->resp[1] = readl(base + AK98MCIRESPONSE1);
	cmd->resp[2] = readl(base + AK98MCIRESPONSE2);
	cmd->resp[3] = readl(base + AK98MCIRESPONSE3);
    PK("base=0x%x,resp[0]=0x%x, [1]=0x%x,resp[2]=0x%x, [3]=0x%x",base,cmd->resp[0],
                    cmd->resp[1],cmd->resp[2],cmd->resp[3]);
	if (status & MCI_RESPTIMEOUT) {
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_RESPCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
	}

	writel(readl(base + AK98MCIMASK) & ~MCI_CMDIRQMASKS, base + AK98MCIMASK);
	PK("DISABLE CMD IRQ\n");

	if (!cmd->data || cmd->error) {
		if (host->data)
			ak98_sdio_stop_data(host);
		ak98_sdio_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ)) {
		ak98_sdio_start_data(host, cmd->data);
	}
	PK("-%s\n", __func__);
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t ak98_sdio_irq(int irq, void *dev_id)
{
	struct ak98_mci_host *host = dev_id;
	u32 status;
	int ret = 0;

	PK("+%s ", __func__);

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		status = readl(host->base + AK98MCISTATUS);

		PK(" status= 0x%08x\n", status);

#ifdef AKMCI_INNERFIFO_PIO
		if (host->data)
			ak98_sdio_pio_irq(host, status);
#endif

		cmd = host->cmd;
		if (status & (MCI_RESPCRCFAIL|MCI_RESPTIMEOUT|MCI_CMDSENT|MCI_RESPEND)
		    && cmd)
			ak98_sdio_cmd_irq(host, cmd, status);

		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_DATAEND|MCI_DATABLOCKEND|MCI_STARTBIT_ERR)
		    && data)
			ak98_sdio_data_irq(host, data, status);

		if (status & MCI_SDIOINT) {
		    /*must disable sdio irq ,than read status to clear the sdio status,
                        else sdio irq will come again.
		       */
		    ak98_enable_sdio_irq(host->mmc,0);
		    readl(host->base + AK98MCISTATUS);		    
			mmc_signal_sdio_irq(host->mmc);
		}

		ret = 1;
	} while (0);

	spin_unlock(&host->lock);

	PK("-%s, irqmask: 0x%08x\n", __func__, readl(host->base + AK98MCIMASK));

	return IRQ_RETVAL(ret);
}

/**
 * @brief  detect sdio card's level type .
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] data  getting the information of sd host.
 * @return void.
 */

static void ak98_sdio_detect_change(unsigned long data)
{
	struct ak98_mci_host *host = (struct ak98_mci_host *)data;

	PK("%s\n", __func__);

	mmc_detect_change(host->mmc, 0);

	if (host->irq_cd_type == IRQ_TYPE_LEVEL_LOW) {
		host->irq_cd_type = IRQ_TYPE_LEVEL_HIGH;
	} else {
		host->irq_cd_type = IRQ_TYPE_LEVEL_LOW;
	}
	set_irq_type(host->irq_cd, host->irq_cd_type);
	enable_irq(host->irq_cd);
}

static irqreturn_t ak98_sdio_card_detect_irq(int irq, void *dev)
{
	struct ak98_mci_host *host = dev;

	PK("%s##################\n", __func__);

	disable_irq_nosync(irq);
	mod_timer(&host->detect_timer, jiffies + msecs_to_jiffies(400));

	return IRQ_HANDLED;
}

/**
 * @brief   detect the sdio card whether or not is in.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *mmc information of host ,getting the sdio detect gpio.
 * @return int.
 * @retal 1 sdio card is in ;0 sdio card is not in
 */

static int ak98_sdio_get_cd(struct mmc_host *mmc)
{
	struct ak98_mci_host *host = mmc_priv(mmc);	

	if (host->gpio_cd == -ENOSYS)
		return -ENOSYS;	
		
	//printk("%s: %i\n", __func__, ak98_gpio_getpin(host->gpio_cd) == 0);
	return (ak98_gpio_getpin(host->gpio_cd) == 0);
}

/**
 * @brief   detect the sdio card writing protection.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *mmc information of host ,getting the sdio detect gpio.
 * @return int.
 * @retal 1 sdio card writing protected ;0 sdio card writing is not protected
 */

static int ak98_sdio_get_ro(struct mmc_host *mmc) 
{
	struct ak98_mci_host *host = mmc_priv(mmc);

	if (host->gpio_wp == -ENOSYS)
		return -ENOSYS;

	//printk("%s: %i\n", __func__, ak98_gpio_getpin(host->gpio_wp));
	return (ak98_gpio_getpin(host->gpio_wp) == 0);
}

static void ak98sdio_set_clk(struct ak98_mci_host *host, struct mmc_ios *ios)
{
	unsigned int regval;
	int div;

	PKCLK("%s\n",__func__); 
	
	if (ios->clock == 0) 
	{
		regval = readl(host->base + AK98MCICLOCK);
		regval &= ~MCI_CLK_ENABLE;
		host->bus_clkrate = 0;
		writel(regval, host->base + AK98MCICLOCK);		
	} 
	else 
	{
		regval = readl(host->base + AK98MCICLOCK);
		regval |= MCI_CLK_ENABLE;
		regval &= ~0xffff; /* clear clk div */
		div = host->asic_clkrate/ios->clock - 2;
		PKCLK("host->asic_clkrate = %ld\n",host->asic_clkrate); 
		PKCLK("ios->clock = %d\n",ios->clock); 
		printk("clk_div: %d\n", div);
		
		regval |= MMC_CLK_DIVL(div/2) | MMC_CLK_DIVH(div/2);			
		host->bus_clkrate = host->asic_clkrate / (div + 2);
		writel(regval, host->base + AK98MCICLOCK);		
	}
}

/**
 * @brief   set sdio bus mode,bus width,clock.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *mmc information of host .
 * @param [in] *ios information of sd interface .
 * @return void.
 */

static void ak98_sdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ak98_mci_host *host = mmc_priv(mmc);

	hydbg("bus_mode=%d,bus_width=%d\n", ios->bus_mode, ios->bus_width);
	
	/* ak98 (and SD spec) don't support MMC_BUSMODE_OPENDRAIN */
	host->bus_mode = ios->bus_mode;
	host->bus_width = ios->bus_width;
	//printk("%ubits(%u); ", (unsigned)1 >> (host->bus_width), host->bus_width);

	/* we can't control external power supply unit */
	switch (ios->power_mode) {
	case MMC_POWER_UP:
		PK("MMC_POWER_UP; ");
		break;
	case MMC_POWER_ON:
		PK("MMC_POWER_ON; ");
		break;
	case MMC_POWER_OFF:
		PK("MMC_POWER_OFF; ");
		break;
	}
	
    ak98_group_config(ePIN_AS_SDIO); 
    
	if (ios->clock != host->bus_clkrate) 
	{
	#ifdef CONFIG_CPU_FREQ
		down(&host->freq_lock);
	#endif
		
		ak98sdio_set_clk(host, &mmc->ios);

	#ifdef CONFIG_CPU_FREQ
		 up(&host->freq_lock);
	#endif
		
	}
	
	/* no matter high-speed mode or not, ak88 mci use the same timing */   

	//printk("ios->clock(%dkhz), host->bus_clkrate(%lukhz), host->asic_clkrate(%lumhz), MMC_CLK_CTRL(0x%08x)\n",
	     //  ios->clock/1000, host->bus_clkrate/1000, host->asic_clkrate/1000000, readl(host->base + AK98MCICLOCK));
}

/**
 * @brief   reset sdio card clock.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] void .
 * @return void.
 */

static void ak98_sdio_reset(void)
{
    rCLK_CON2 |= (0x1 << 19);
    rCLK_CON2 &= ~(0x1 << 19);
}

/**
 * @brief   send a request, starting data or command.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *mmc information of host .
 * @return void.
 */

static void ak98_sdio_send_request(struct mmc_host *mmc)
{
    struct ak98_mci_host *host = mmc_priv(mmc); 
    struct mmc_request *mrq = host->mrq;	
    unsigned long flags;    

#ifdef CONFIG_CPU_FREQ
	 /*
		* need not to acquire the freq_lock in interrupt.
	*/
	 if (!in_interrupt()) 
	 {					   
		 down(&host->freq_lock);
		 
	 }	  
#endif
    

    if(mrq->data || mrq->cmd->data)
    {
        l2_sdio_bufid = ak98_l2_alloc(ADDR_SDIO);
        if (BUF_NULL == l2_sdio_bufid)
        {
            printk("L2 buffer malloc fail!\n");
            BUG();
        }
        ak98_l2_clr_status(l2_sdio_bufid);
    }
    
    spin_lock_irqsave(&host->lock, flags);  
    
	if (mrq->data && (mrq->data->flags & MMC_DATA_READ))
		ak98_sdio_start_data(host, mrq->data);

	ak98_sdio_start_command(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);    

}

static void ak98_sdio_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ak98_mci_host *host = mmc_priv(mmc);	

	PK1("%s: CMD%i\n", __func__, mrq->cmd->opcode);
	
	host->mrq = mrq;	

    if (ak98_sdio_get_cd(mmc) == 0)
    {
        printk("%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);        
    }
    else
    {
        ak98_sdio_send_request(mmc); 
    }	
	
}

/**
 * register the function of sd driver.
 * 
 */

static const struct mmc_host_ops ak98_mci_ops = {
	.request	= ak98_sdio_request,
	.set_ios	= ak98_sdio_set_ios,
	.get_ro		= ak98_sdio_get_ro,
	.get_cd		= ak98_sdio_get_cd,
	.enable_sdio_irq = ak98_enable_sdio_irq,

};

// difine change cpu freq
#ifdef CONFIG_CPU_FREQ

static int ak98sdio_cpufreq_transition(struct notifier_block *nb,
				     unsigned long val, void *data)
{
	struct ak98_mci_host *host;
	struct mmc_host *mmc;
	unsigned long newclk;
	unsigned long flags;
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *)data;
	host = container_of(nb, struct ak98_mci_host, freq_transition);
	
	PKCLK("%s\n",__func__);	
	PKCLK("%s(): in_interrupt()=%ld\n", __func__, in_interrupt());
	PKCLK("ak98_get_asic_clk = %ld\n",ak98_get_asic_clk());	
	PKCLK("freqs->new_cpufreq.asic_clk = %d\n",
		  freqs->new_cpufreq.asic_clk);	
		  
	mmc = host->mmc;
	newclk = freqs->new_cpufreq.asic_clk;
	if ((val == CPUFREQ_PRECHANGE && newclk > host->asic_clkrate) 
		|| (val == CPUFREQ_POSTCHANGE && newclk < host->asic_clkrate)) 
	{

		if (mmc->ios.power_mode != MMC_POWER_OFF &&
			mmc->ios.clock != 0)
		{		
			PKCLK("%s(): preempt_count()=%d\n",
				__func__, preempt_count());
				
			down(&host->freq_lock);
			
			spin_lock_irqsave(&mmc->lock, flags);
		
			host->asic_clkrate = newclk;
			PKCLK("AK98MCICLOCK1 = %d\n",readl(host->base + AK98MCICLOCK));	
			ak98sdio_set_clk(host, &mmc->ios);
			PKCLK("AK98MCICLOCK2 = %d\n",readl(host->base + AK98MCICLOCK));	
			
			spin_unlock_irqrestore(&mmc->lock, flags);

			up(&host->freq_lock);
		}		

	}

	return NOTIFY_DONE;
}

static inline int ak98sdio_cpufreq_register(struct ak98_mci_host *host)
{
	// use for requst and cpufreq
	init_MUTEX(&host->freq_lock);
	
	host->freq_transition.notifier_call = ak98sdio_cpufreq_transition;

	return cpufreq_register_notifier(&host->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void ak98sdio_cpufreq_deregister(struct ak98_mci_host *host)
{
	cpufreq_unregister_notifier(&host->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else
static inline int ak98sdio_cpufreq_register(struct ak98_mci_host *host)
{
	return 0;
}

static inline void ak98sdio_cpufreq_deregister(struct ak98_mci_host *host)
{
}
#endif

// difine change cpu freq end



#if 0
static void ak98_mci_check_status(unsigned long data)
{
	struct ak98_mci_host *host = (struct ak98_mci_host *)data;
	unsigned int status = ak98_mci_get_cd(host->mmc);

	if (status ^ host->oldstat)
		mmc_detect_change(host->mmc, 0);

	host->oldstat = status;
	mod_timer(&host->timer, jiffies + HZ);
}
#endif

/**
 * @brief   sdio driver probe and init.
 * 
 * @author Hanyang
 * @date 2011-05-10
 * @param [in] *pdev information of platform device ,getting the sd driver resource .
 * @return int.
 * @retval -EINVAL no platform data , fail;
 * @retval -EBUSY  requset mem  fail;
 * @retval -ENOMEM  alloc mem fail;
 */

static int __devinit ak98_sdio_probe(struct platform_device *pdev)
{
	struct ak98_mci_platform_data *plat = pdev->dev.platform_data;
	struct ak98_mci_host *host;
	struct mmc_host *mmc;
	struct resource *res;
	int irq;
	int ret;

	/* must have platform data */
	if (!plat) {
		ret = -EINVAL;
		goto out;
	}

	PK("%s\n", __func__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);

	PK("res: %x, %u", res->start, resource_size(res));
	res = request_mem_region(res->start, resource_size(res), DRIVER_NAME);
	if (!res) {
		ret = -EBUSY;
		goto out;
	}
	PK("res: %x, %u\n", res->start, resource_size(res));

	mmc = mmc_alloc_host(sizeof(struct ak98_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->gpio_wp = -ENOSYS;
	host->gpio_cd = -ENOSYS;

    ak98_sdio_reset();
    host->clk = clk_get(&pdev->dev, "sdio_clk");
    if (IS_ERR(host->clk)) {
        ret = PTR_ERR(host->clk);
        host->clk = NULL;
        goto host_free;
    }      

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_free;

	host->plat = plat;
	host->asic_clkrate = ak98_get_asic_clk();
	

	host->base = ioremap(res->start, resource_size(res));
	if (!host->base) {
		ret = -ENOMEM;
		goto clk_disable;
	} 
    PK("asic_clkrate: %luhz,host->base=0x%x\n", host->asic_clkrate,host->base);
	mmc->ops = &ak98_mci_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;
#if 0
	mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;
#endif 
	mmc->caps |= MMC_CAP_SDIO_IRQ;

//	mmc->caps |= MMC_CAP_NEEDS_POLL;
	mmc->f_min = host->asic_clkrate / (255+1 + 255+1);
	mmc->f_max = host->asic_clkrate / (0+1 + 0+1);
	mmc->f_max = mmc->f_max < fmax ? mmc->f_max : fmax;

	/*
	 * We can do SGIO
	 */
	mmc->max_hw_segs = 16;
	mmc->max_phys_segs = NR_SG;

	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 */
	mmc->max_req_size = 65535;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	mmc->max_seg_size = mmc->max_req_size;

#if 0
	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 2048;
#else
	/* as l2 fifo limit to 512 bytes */
	mmc->max_blk_size = 512;
#endif

	/*
	 * No limit on the number of blocks transferred.
	 */
	mmc->max_blk_count = mmc->max_req_size;

	spin_lock_init(&host->lock);
	
    ak98_group_config(ePIN_AS_SDIO); 
	writel(MCI_ENABLE|MCI_FAIL_TRIGGER, host->base + AK98MCICLOCK);	
    PK("%s: MCICLOCK: 0x%08x\n", __func__, readl(host->base + AK98MCICLOCK));
    
	writel(0, host->base + AK98MCIMASK);
	
	PK("request irq %i\n", irq);
	ret = request_irq(irq, ak98_sdio_irq, IRQF_DISABLED, DRIVER_NAME " (cmd)", host);
	if (ret)
		goto unmap;
		
	host->irq_mci = irq;
	if(plat->gpio_cd >= 0)
	{
    	host->gpio_cd = plat->gpio_cd;	
    	ak98_gpio_cfgpin(host->gpio_cd, AK98_GPIO_DIR_INPUT);
    	ak98_gpio_pulldown(host->gpio_cd,AK98_PULLDOWN_DISABLE);
        ak98_gpio_pullup(host->gpio_cd, AK98_PULLUP_ENABLE);  
        
        setup_timer(&host->detect_timer, ak98_sdio_detect_change,
                    (unsigned long)host);
                    
        irq = ak98_gpio_to_irq(host->gpio_cd);    
        ret = request_irq(irq, ak98_sdio_card_detect_irq,
                  IRQF_DISABLED,
                  DRIVER_NAME " cd", host);
        printk("request gpio irq ret = %d, irq=%d", ret, irq);
        if (ret)
            goto irq_free;
        host->irq_cd = irq;
        host->irq_cd_type = IRQ_TYPE_LEVEL_LOW; 
    }
    
    if(plat->gpio_wp >= 0)
    {
        host->gpio_wp = plat->gpio_wp;	
    	ak98_gpio_cfgpin(host->gpio_wp, AK98_GPIO_DIR_INPUT);
    	ak98_gpio_pullup(host->gpio_wp, AK98_PULLUP_ENABLE);
    	ak98_gpio_pulldown(host->gpio_wp,AK98_PULLDOWN_DISABLE);
    }
	
	platform_set_drvdata(pdev, mmc);

	ret = ak98sdio_cpufreq_register(host);
	if (ret) {
		goto irq_free;
	}

	ret = mmc_add_host(mmc);
	if (ret) {
		goto cpufreq_free;
	}

	PK(KERN_INFO "%s: ak98MCI at 0x%016llx irq %d\n",
		mmc_hostname(mmc), (unsigned long long)res->start,
		host->irq_mci);

	return 0;

 cpufreq_free:
	 PK("ERR cpufreq_free\n");
	 ak98sdio_cpufreq_deregister(host);

 irq_free:
	PK("ERR irq_free\n");
	free_irq(host->irq_mci, host);
 unmap:
	PK("ERR unmap\n");
	iounmap(host->base);
 clk_disable:
	PK("ERR clk_disable\n");
	clk_disable(host->clk);
 clk_free:
	PK("ERR clk_free\n");
	clk_put(host->clk);
 host_free:
	PK("ERR host_free\n");
	mmc_free_host(mmc);
 out:
	PK("ERR out\n");
	return ret;
}

static int __devexit ak98_sdio_remove(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
  
	platform_set_drvdata(dev, NULL);

	if (mmc) {
		struct ak98_mci_host *host = mmc_priv(mmc);

#if 0
		del_timer_sync(&host->timer);
#endif

		ak98sdio_cpufreq_deregister(host);

		mmc_remove_host(mmc);

		writel(0, host->base + AK98MCIMASK);

		writel(0, host->base + AK98MCICOMMAND);
		writel(0, host->base + AK98MCIDATACTRL);

#if 0
		if (gpio_is_valid(host->gpio_cd))
			free_irq(host->irq_cd, host);
		free_irq(host->irq_mci, host);

		if (host->gpio_wp != -ENOSYS)
			gpio_free(host->gpio_wp);
		if (host->gpio_cd != -ENOSYS)
			gpio_free(host->gpio_cd);
#else
        if (host->irq_cd > 0)
        {
		    free_irq(host->irq_cd, host);
		}
		free_irq(host->irq_mci, host);
#endif

		iounmap(host->base);
		clk_disable(host->clk);
		clk_put(host->clk);

		mmc_free_host(mmc);
	}

	return 0;
}

//#ifdef CONFIG_PM
#if 0
static int ak98_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct ak98_mci_host *host = mmc_priv(mmc);

		ret = mmc_suspend_host(mmc, state);
		if (ret == 0)
			writel(0, host->base + AK88MCIMASK0);
	}

	return ret;
}

static int ak98_sdio_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct ak98_mci_host *host = mmc_priv(mmc);

		writel(MCI_IRQENABLE, host->base + AK88MCIMASK0);

		ret = mmc_resume_host(mmc);
	}

	return ret;
}
#else
#define ak98_sdio_suspend	NULL
#define ak98_sdio_resume	NULL
#endif

static struct platform_driver ak98_sdio_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
	},
	.probe		= ak98_sdio_probe,
	.remove		= __devexit_p(ak98_sdio_remove),
	.suspend	= ak98_sdio_suspend,
	.resume		= ak98_sdio_resume,
};

static int __init ak98_sdio_init(void)
{
	printk("%s\n", __func__);
	return platform_driver_register(&ak98_sdio_driver);
}

static void __exit ak98_sdio_exit(void)
{
	platform_driver_unregister(&ak98_sdio_driver);
}

module_init(ak98_sdio_init);
module_exit(ak98_sdio_exit);

MODULE_DESCRIPTION("Anyka AK98 MMC/SD/SDIO Interface driver");
MODULE_LICENSE("GPL");
