/*
 *  linux/drivers/mmc/host/ak88_mci.c - AK88 MMC/SD/SDIO driver
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
#include <linux/scatterlist.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>

#include <mach/devices_ak880x.h>
#include <mach/gpio.h>
#include "ak88_mci.h"

#define DRIVER_NAME "ak88_mci"

//#define AKMCI_INNERFIFO_PIO /* only 4bytes inner fifo */
#define AKMCI_L2FIFO_PIO
//#define AKMCI_L2FIFO_DMA

#define DBG(host,fmt,args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

#define PK1(fmt...) //printk(fmt)
#define PK(fmt...) //printk(fmt)

#define SRDPIN_USE_MUTEX

#ifdef SRDPIN_USE_MUTEX
extern struct mutex nand_lock;
#else
extern struct semaphore nand_lock;
#endif

#if defined AKMCI_L2FIFO_PIO || defined AKMCI_L2FIFO_DMA
static unsigned int fmax = (20*1000*1000);
#elif defined AKMCI_INNERFIFO_PIO
static unsigned int fmax = (4*1000*1000);
#else
#error "Please select one FIFO translation mode!"
#endif

static void ak88_mci_dump_regs(void *base)
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

#ifdef AKMCI_L2FIFO_PIO
static void
mci_xfer(struct ak88_mci_host *host)
{
	int i, sg_remain;
	u32 *src, *dst;

	PK("%s\n", __func__);

	if (host->data->flags & MMC_DATA_WRITE) {
		src = sg_virt(host->sg_ptr) + host->sg_off;
		dst = host->l2fifo;
	} else {
		src = host->l2fifo;
		dst = sg_virt(host->sg_ptr) + host->sg_off;
	}

	/* 
	   limit: blksz(512), host_remain, sg

	   xfer_len = min(host->size, host->data->blksz);
	   if (xfer_len <= 0)
	   return 0;

	   sg_remain = ;
	   while (sg_remain <= 0) {
	   next_sg();
	   sg_remain = ;
	   }

	   if (sg_remain >= xfer_len)
	   memcpy(dst, src, xfer_len);

	   do {
	   sg_remain = host->sg_ptr->length - host->sg_off;
	   x_len = xfer_len;
	   if (sg_remain < x_len)
	   x_len = sg_remain;
	   memcpy(dst, src, x_len);
	   xfer_len -= x_len;
	   sg_remain -= x_len;
	   }while (xfer_len > 0 || sg == NULL);

	   while (host->sg_ptr && offset < xfer_len) {
	   sg_remain = host->sg_ptr->length - host->sg_off;
	   x_len = min(sg_remain, xfer_len);

	   memcpy(dst, src, x_len);
	   if (read)
	   memcpy(sg_virt(host->sg_ptr) + host->sg_off, buffer + offset, x_len);
	   else
	   memcpy(buffer + offset, sg_virt(host->sg_ptr)+host->sg_off, x_len);

	   offset += x_len;
	   }

	   xfer_len -= x_len;
	   if (xfer_len <= 0) {
	   sg_remain -= x_len;
	   break; // done
	   }

	   host->sg_ptr = sg_next();
	   host->sg_off = 0;
	   if (host->sg_ptr == NULL)
	   break;

	   if (read) {
	   dst = sg_virt(host->sg_ptr);
	   src += x_len;
	   } else {
	   src = sg_virt(host->sg_ptr);
	   dst += x_len;
	   }
	   }
	   host->sg_off = sg->length
	*/

	sg_remain = host->sg_ptr->length - host->sg_off;
	for (i = 0; i < host->data->blksz; i+=4) {
		*dst = *src;
		src++;
		dst++;

		host->data_xfered += 4;
		host->size -= 4;

		sg_remain -= 4;
		if (sg_remain <= 0) {
			host->sg_ptr = sg_next(host->sg_ptr);
			if (host->sg_ptr == NULL)
				return;
			host->sg_off = 0;
			if (host->data->flags & MMC_DATA_WRITE)
				src = sg_virt(host->sg_ptr) + host->sg_off;
			else
				dst = sg_virt(host->sg_ptr) + host->sg_off;
			sg_remain = host->sg_ptr->length - host->sg_off;
		}
	}
	PK("\n");

	host->sg_off = host->sg_ptr->length - sg_remain;
}
#endif

static void ak88_mci_stop_data(struct ak88_mci_host *host)
{
	u32 masks;

	PK1("%s\n", __func__);

	writel(0, host->base + AK88MCIDMACTRL);
	writel(0, host->base + AK88MCIDATACTRL);
	masks = readl(host->base + AK88MCIMASK);
	masks &= ~(MCI_DATAIRQMASKS|MCI_FIFOFULLMASK|MCI_FIFOEMPTYMASK);
	writel(masks, host->base + AK88MCIMASK);
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


static void
ak88_mci_request_end(struct ak88_mci_host *host, struct mmc_request *mrq)
{
	PK1("%s\n", __func__);

	writel(0, host->base + AK88MCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;

	/* release shared data pins */
#ifdef SRDPIN_USE_MUTEX
	mutex_unlock(&nand_lock);
#else
	up(&nand_lock);
#endif

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void ak88_mci_start_data(struct ak88_mci_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout, irqmask;
	unsigned long long clks;
	void __iomem *base;
	u32 regval;

	PK1("%s: blksz %04x blks %04x flags %08x\n",
	       __func__, data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz * data->blocks;
	host->data_xfered = 0;

	ak88_mci_init_sg(host, data);

	if (data->timeout_clks) {
		timeout = data->timeout_clks;
	} else {
		clks = (unsigned long long)data->timeout_ns * host->bus_clkrate;
		do_div(clks, 1000000000UL);
		timeout = (unsigned int)clks;
	}
	PK("timeout: %uns / %uclks\n", data->timeout_ns, data->timeout_clks);

	base = host->base;
	writel(timeout, base + AK88MCIDATATIMER);
	writel(host->size, base + AK88MCIDATALENGTH);

#ifdef AKMCI_L2FIFO_PIO

	/* get l2 fifo */
	regval = readl(host->l2base + L2FIFO_ASSIGN1);
	regval = (regval & (~(7<<12))) | (MCI_L2FIFO_NUM << 12);
	writel(regval, host->l2base + L2FIFO_ASSIGN1);

	regval = readl(host->l2base + L2FIFO_CONF1);
	regval |= (1 << (16 + MCI_L2FIFO_NUM)) | (1 << (24 + MCI_L2FIFO_NUM));
	writel(regval, host->l2base + L2FIFO_CONF1);

	PK1("L2ASSIGN: 0x%08x, L2CONF: 0x%08x\n",
	       readl(host->l2base + L2FIFO_ASSIGN1),
	       readl(host->l2base + L2FIFO_CONF1));

	/* set l2 fifo info */
	writel (MCI_DMA_BUFEN | MCI_DMA_SIZE(MCI_L2FIFO_SIZE/4),
		base + AK88MCIDMACTRL);

#elif defined AKMCI_L2FIFO_DMA

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

		request_irq(AK88_L2MEM_IRQ(9+MCI_L2FIFO_NUM), ak88_mcil2_irq,
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
		base + AK88MCIDMACTRL);
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

	writel(readl(base + AK88MCIMASK) | MCI_DATAIRQMASKS, base + AK88MCIMASK);
	writel(datactrl, base + AK88MCIDATACTRL);

	PK("ENABLE DATA IRQ, datactrl: 0x%08x, timeout: 0x%08x, len: %u\n",
	       datactrl, readl(base+AK88MCIDATATIMER), host->size);

#ifdef AKMCI_L2FIFO_PIO
	if (data->flags & MMC_DATA_WRITE)
		mci_xfer(host);
#endif

#ifdef AKMCI_INNERFIFO_PIO
	irqmask = readl(base + AK88MCIMASK);
	if (data->flags & MMC_DATA_READ) {
		if (host->size > MCI_FIFOSIZE)
			irqmask |= MCI_FIFOFULLMASK;
		else
			;	/* wait for DATAEND int */
	} else {
		irqmask |= MCI_FIFOEMPTYMASK;
	}

	writel(irqmask, base + AK88MCIMASK);
#endif
}

static void
ak88_mci_start_command(struct ak88_mci_host *host, struct mmc_command *cmd)
{
	unsigned int c;
	void __iomem *base = host->base;

	PK1("%s: op %02x arg %08x flags %08x\n",
	       __func__, cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + AK88MCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + AK88MCICOMMAND);
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

	writel(cmd->arg, base + AK88MCIARGUMENT);
	writel(readl(base + AK88MCIMASK) | MCI_CMDIRQMASKS, base + AK88MCIMASK);
	PK("ENABLE CMD IRQ\n");
	PK("irqmask: 0x%08x\n", readl(base+AK88MCIMASK));
	writel(c, base + AK88MCICOMMAND);
#if 0
	PK("%s: cmd:0x%08x; irq:0x%08x\n",
	       __func__, c, readl(base+AK88MCIMASK));
	ak88_mci_dump_regs(host->base);
#endif
}


#ifdef AKMCI_INNERFIFO_PIO
static void
ak88_mci_pio_irq(struct ak88_mci_host *host, unsigned int status)
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
		ak88_mci_next_sg(host);
	}
}
#endif

static void
ak88_mci_data_irq(struct ak88_mci_host *host, struct mmc_data *data,
		  unsigned int status)
{
	if (status & MCI_DATABLOCKEND) {
		PK("BLOCKEND\n");
#ifdef AKMCI_L2FIFO_PIO
		if (host->size > 0)
			mci_xfer(host);
#endif
	}
	if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_STARTBIT_ERR)) {
		PK1("DATA ERROR: 0x%08x\n", status);

		if (status & MCI_DATACRCFAIL || status & MCI_STARTBIT_ERR)
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
		ak88_mci_stop_data(host);

		//dump_data(data);

		if (!data->stop) {
			ak88_mci_request_end(host, data->mrq);
		} else {
			ak88_mci_start_command(host, data->stop);
		}
	}
}

static void
ak88_mci_cmd_irq(struct ak88_mci_host *host, struct mmc_command *cmd,
		 unsigned int status)
{
	void __iomem *base = host->base;

	PK("+%s\n", __func__);
	host->cmd = NULL;

	cmd->resp[0] = readl(base + AK88MCIRESPONSE0);
	cmd->resp[1] = readl(base + AK88MCIRESPONSE1);
	cmd->resp[2] = readl(base + AK88MCIRESPONSE2);
	cmd->resp[3] = readl(base + AK88MCIRESPONSE3);

	if (status & MCI_RESPTIMEOUT) {
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_RESPCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
	}

	writel(readl(base + AK88MCIMASK) & ~MCI_CMDIRQMASKS, base + AK88MCIMASK);
	PK("DISABLE CMD IRQ\n");

	if (!cmd->data || cmd->error) {
		if (host->data)
			ak88_mci_stop_data(host);
		ak88_mci_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ)) {
		ak88_mci_start_data(host, cmd->data);
	}
	PK("-%s\n", __func__);
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t ak88_mci_irq(int irq, void *dev_id)
{
	struct ak88_mci_host *host = dev_id;
	u32 status;
	int ret = 0;

	PK("+%s ", __func__);

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		status = readl(host->base + AK88MCISTATUS);

		PK("irq0 %08x\n", status);
/*
		PK("irq: a: 0x%08x, b: 0x%08x\n", status,
		       readl(host->base + AK88MCISTATUS));
*/

#ifdef AKMCI_INNERFIFO_PIO
		if (host->data)
			ak88_mci_pio_irq(host, status);
#endif

		cmd = host->cmd;
		if (status & (MCI_RESPCRCFAIL|MCI_RESPTIMEOUT|MCI_CMDSENT|MCI_RESPEND)
		    && cmd)
			ak88_mci_cmd_irq(host, cmd, status);

		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_DATAEND|MCI_DATABLOCKEND|MCI_STARTBIT_ERR)
		    && data)
			ak88_mci_data_irq(host, data, status);

#ifdef SDIO
		if (status & MCI_SDIOINT) {
			mmc_signal_sdio_irq(host->mmc);
		}
#endif

		ret = 1;
	} while (0);

	spin_unlock(&host->lock);

	PK("-%s, irqmask: 0x%08x\n", __func__, readl(host->base + AK88MCIMASK));

	return IRQ_RETVAL(ret);
}

static void ak88_mci_detect_change(unsigned long data)
{
	struct ak88_mci_host *host = (struct ak88_mci_host *)data;

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

static irqreturn_t ak88_mci_card_detect_irq(int irq, void *dev)
{
	struct ak88_mci_host *host = dev;

	PK("%s\n", __func__);

	disable_irq_nosync(irq);
	mod_timer(&host->detect_timer, jiffies + msecs_to_jiffies(400));

	return IRQ_HANDLED;
}

static void ak88_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ak88_mci_host *host = mmc_priv(mmc);
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	PK1("%s: CMD%i\n", __func__, mrq->cmd->opcode);

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		printk(KERN_ERR "%s: Unsupported block size (%d bytes)\n",
		       mmc_hostname(mmc), mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	/* grab shared pins */
	PK("set shared pins\n");
#if defined CONFIG_BOARD_AK8801EPC

	/* soc pin mux bug */
#ifdef SRDPIN_USE_MUTEX
	mutex_lock_interruptible(&nand_lock);
#else
	down_interruptible(&nand_lock);
#endif
/*	ak880x_gpio_pullup(AK88_GPIO_39, AK88_GPIO_PUPD_DISABLE);
	ak880x_gpio_pullup(AK88_GPIO_40, AK88_GPIO_PUPD_DISABLE);*/
	AK88_GPIO_MDAT2(AK88_SHARE_FUNC);
	AK88_MCI_ENABLE();

#elif defined CONFIG_BOARD_AK8802EBOOK

#ifdef SRDPIN_USE_MUTEX
	mutex_lock_interruptible(&nand_lock);
#else
	down_interruptible(&nand_lock);
#endif
	AK88_GPIO_NFC_DATA4_7(AK88_SHARE_FUNC);
	AK88_GPIO_NFC_DATA1_3(AK88_SHARE_FUNC);
	AK88_GPIO_NFC_DATA0(AK88_SHARE_FUNC);
	AK88_SD_ENABLE();

#endif

	if (mrq->data && (mrq->data->flags & MMC_DATA_READ))
		ak88_mci_start_data(host, mrq->data);

	ak88_mci_start_command(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void ak88_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ak88_mci_host *host = mmc_priv(mmc);
	unsigned int regval;
	int div;

	printk("%s ", __func__);

	/* ak88 (and SD spec) don't support MMC_BUSMODE_OPENDRAIN */
	host->bus_mode = ios->bus_mode;
	host->bus_width = ios->bus_width;
	printk("%ubits(%u); ", (unsigned)1 >> (host->bus_width), host->bus_width);

	/* we can't control external power supply unit */
	switch (ios->power_mode) {
	case MMC_POWER_UP:
		PK("MMC_POWER_UP; ");
#if 0
		regval = readl(host->base + AK88MCICLOCK);
		regval |= MCI_ENABLE;
		writel(regval, host->base + AK88MCICLOCK);
#endif
		break;
	case MMC_POWER_ON:
		PK("MMC_POWER_ON; ");
		break;
	case MMC_POWER_OFF:
		PK("MMC_POWER_OFF; ");
#if 0
		regval = readl(host->base + AK88MCICLOCK);
		regval &= ~MCI_ENABLE;
		writel(regval, host->base + AK88MCICLOCK);
#endif
		break;
	}
	
	if (ios->clock != host->bus_clkrate) {
		regval = readl(host->base + AK88MCICLOCK);
		if (ios->clock == 0) {
			regval &= ~MCI_CLK_ENABLE;
			host->bus_clkrate = 0;
		} else {
			regval |= MCI_CLK_ENABLE;
			regval &= ~0xffff; /* clear clk div */
			div = (host->asic_clkrate + ios->clock - 1) / ios->clock - 2;
			printk("clk_div: %d\n", div);
			if (div < 256) {
				regval |= MMC_CLK_DIVL(div);
			} else {
				regval |= MMC_CLK_DIVL(255) | MMC_CLK_DIVH(div-255);
			}
			host->bus_clkrate = host->asic_clkrate / (div + 2);
		}
		writel(regval, host->base + AK88MCICLOCK);
	}

	/* no matter high-speed mode or not, ak88 mci use the same timing */

	printk("ios->clock(%dkhz), host->bus_clkrate(%lukhz), host->asic_clkrate(%lumhz), MMC_CLK_CTRL(0x%08x)\n",
	       ios->clock/1000, host->bus_clkrate/1000, host->asic_clkrate/1000000, readl(host->base + AK88MCICLOCK));
}

static int ak88_mci_get_ro(struct mmc_host *mmc)
{
	struct ak88_mci_host *host = mmc_priv(mmc);

	if (host->gpio_wp == -ENOSYS)
		return -ENOSYS;

	printk("%s: %i\n", __func__, ak880x_gpio_getpin(host->gpio_wp));
	return (ak880x_gpio_getpin(host->gpio_wp) != 0);
}

#if 0
static int ak88_mci_get_cd(struct mmc_host *mmc)
{
	struct ak88_mci_host *host = mmc_priv(mmc);
	unsigned int status;

	if (host->gpio_cd == -ENOSYS)
		status = host->plat->status(mmc_dev(host->mmc));
	else
		status = gpio_get_value(host->gpio_cd);

	return !status;
}
#endif

static const struct mmc_host_ops ak88_mci_ops = {
	.request	= ak88_mci_request,
	.set_ios	= ak88_mci_set_ios,
	.get_ro		= ak88_mci_get_ro,
#if 0
	.get_cd		= ak88_mci_get_cd,
#endif
#ifdef SDIO
	.enable_sdio_irq = ak88_mci_enable_sdio_irq,
#endif
};

#if 0
static void ak88_mci_check_status(unsigned long data)
{
	struct ak88_mci_host *host = (struct ak88_mci_host *)data;
	unsigned int status = ak88_mci_get_cd(host->mmc);

	if (status ^ host->oldstat)
		mmc_detect_change(host->mmc, 0);

	host->oldstat = status;
	mod_timer(&host->timer, jiffies + HZ);
}
#endif

static int __devinit ak88_mci_probe(struct platform_device *pdev)
{
	struct ak88_mci_platform_data *plat = pdev->dev.platform_data;
	struct ak88_mci_host *host;
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

#if 0				/* can do in akmci_request() */
	/* set shared data pins */
	PK("set shared pins\n");
#if defined CONFIG_BOARD_AK8801EPC
	ak880x_gpio_pullup(AK88_GPIO_39, AK88_GPIO_PUPD_DISABLE);
	ak880x_gpio_pullup(AK88_GPIO_40, AK88_GPIO_PUPD_DISABLE);
	AK88_GPIO_MDAT2(AK88_SHARE_FUNC);
	AK88_MCI_ENABLE();
#elif defined CONFIG_BOARD_AK8802EBOOK
	AK88_GPIO_NFC_DATA4_7(AK88_SHARE_FUNC);
	AK88_GPIO_NFC_DATA1_3(AK88_SHARE_FUNC);
	AK88_GPIO_NFC_DATA0(AK88_SHARE_FUNC);
	AK88_SD_ENABLE();
#endif
#endif

	PK("res: %x, %u", res->start, resource_size(res));
	res = request_mem_region(res->start, resource_size(res), DRIVER_NAME);
	if (!res) {
		ret = -EBUSY;
		goto out;
	}
	PK("res: %x, %u", res->start, resource_size(res));

	mmc = mmc_alloc_host(sizeof(struct ak88_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->gpio_wp = -ENOSYS;
	host->gpio_cd = -ENOSYS;

	host->clk = clk_get(&pdev->dev, "mci_clk");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto host_free;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_free;

	host->plat = plat;
	host->asic_clkrate = clk_get_rate(host->clk);
	PK("asic_clkrate: %uhz", host->asic_clkrate);

	host->base = ioremap(res->start, resource_size(res));
	if (!host->base) {
		ret = -ENOMEM;
		goto clk_disable;
	}

	mmc->ops = &ak88_mci_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;
#if 0
	mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;
#endif
#ifdef SDIO
	mmc->caps |= MMC_CAP_SDIO_IRQ;
#endif
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

	writel(0, host->base + AK88MCICLOCK);
	udelay(1000);

	writel(MCI_ENABLE|MCI_FAIL_TRIGGER, host->base + AK88MCICLOCK);

	writel(0, host->base + AK88MCIMASK);
	PK("%s: MCICLOCK: 0x%08x\n", __func__, readl(host->base + AK88MCICLOCK));

	PK("request irq %i\n", irq);
	ret = request_irq(irq, ak88_mci_irq, IRQF_DISABLED, DRIVER_NAME " (cmd)", host);
	if (ret)
		goto unmap;
	host->irq_mci = irq;

#if 0
	if (gpio_is_valid(plat->gpio_cd)) {
		ret = gpio_request(plat->gpio_cd, DRIVER_NAME " (cd)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_cd);
		if (ret == 0)
			host->gpio_cd = plat->gpio_cd;
		else if (ret != -ENOSYS)
			goto err_gpio_cd;
	}
	if (gpio_is_valid(plat->gpio_wp)) {
		ret = gpio_request(plat->gpio_wp, DRIVER_NAME " (wp)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_wp);
		if (ret == 0)
			host->gpio_wp = plat->gpio_wp;
		else if (ret != -ENOSYS)
			goto err_gpio_wp;
	}

	if (gpio_is_valid(plat->gpio_cd)) {
		irq = ak880x_gpio_to_irq(host->gpio_cd);
		PK("request card detect irq: %u - %u\n", AK88_DGPIO_12, irq);
		ret = request_irq(irq, ak88_mci_card_detect_irq,
				  IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING,
				  DRIVER_NAME " cd", host);
		if (ret)
			goto irq_free;
		host->irq_cd = irq;
	}
#else
	host->gpio_cd = plat->gpio_cd;
	host->gpio_wp = plat->gpio_wp;
	ak880x_gpio_cfgpin(host->gpio_cd, AK88_GPIO_IN_0);
	ak880x_gpio_pullup(host->gpio_wp, AK88_GPIO_PUPD_DISABLE);
	ak880x_gpio_cfgpin(host->gpio_wp, AK88_GPIO_IN_0);

	setup_timer(&host->detect_timer, ak88_mci_detect_change,
		    (unsigned long)host);
	irq = ak880x_gpio_to_irq(host->gpio_cd);
	PK("request card detect irq: %u - %u\n", AK88_DGPIO_12, irq);
	if (ak880x_gpio_getpin(host->gpio_cd))
		set_irq_type (irq, IRQ_TYPE_LEVEL_LOW);
	else
		set_irq_type (irq, IRQ_TYPE_LEVEL_HIGH);
	ret = request_irq(irq, ak88_mci_card_detect_irq,
			  IRQF_DISABLED,
			  DRIVER_NAME " cd", host);
	if (ret)
		goto irq_free;
	host->irq_cd = irq;
	host->irq_cd_type = IRQ_TYPE_LEVEL_LOW;
#endif

	platform_set_drvdata(pdev, mmc);

	mmc_add_host(mmc);

	PK(KERN_INFO "%s: AK88MCI at 0x%016llx irq %d\n",
		mmc_hostname(mmc), (unsigned long long)res->start,
		host->irq_mci);

	/* ak88_mci_dump_regs(host->base); */
	PK("srdpin conf1: 0x%08x, srdpin conf2: 0x%08x\n",
	   readl(AK88_SHAREPIN_CON1), readl(AK88_SHAREPIN_CON2));

#ifdef AKMCI_L2FIFO_PIO
	res = request_mem_region(L2BASE, SZ_256, DRIVER_NAME);
	if (!res) {
		ret = -EBUSY;
		goto irq_free;
	}
	host->l2base = ioremap(res->start, resource_size(res));
	if (!host->l2base) {
		ret = -ENOMEM;
		goto irq_free;
	}

	res = request_mem_region(L2ADDR(MCI_L2FIFO_NUM), MCI_L2FIFO_SIZE, DRIVER_NAME);
	if (!res) {
		ret = -EBUSY;
		goto irq_free;
	}
	host->l2fifo = ioremap(res->start, resource_size(res));
	if (!host->l2fifo) {
		ret = -ENOMEM;
		goto irq_free;
	}
#endif

	return 0;

 irq_free:
	PK("ERR irq_free\n");
	free_irq(host->irq_mci, host);
 unmap:
	PK("ERR unmap\n");
#if 0
	if (host->gpio_wp != -ENOSYS)
		gpio_free(host->gpio_wp);
#endif
 err_gpio_wp:
	PK("ERR gpio_wp\n");
#if 0
	if (host->gpio_cd != -ENOSYS)
		gpio_free(host->gpio_cd);
#endif
 err_gpio_cd:
	PK("ERR gpio_cd\n");
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

static int __devexit ak88_mci_remove(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (mmc) {
		struct ak88_mci_host *host = mmc_priv(mmc);

#if 0
		del_timer_sync(&host->timer);
#endif

		mmc_remove_host(mmc);

		writel(0, host->base + AK88MCIMASK);

		writel(0, host->base + AK88MCICOMMAND);
		writel(0, host->base + AK88MCIDATACTRL);

#if 0
		if (gpio_is_valid(host->gpio_cd))
			free_irq(host->irq_cd, host);
		free_irq(host->irq_mci, host);

		if (host->gpio_wp != -ENOSYS)
			gpio_free(host->gpio_wp);
		if (host->gpio_cd != -ENOSYS)
			gpio_free(host->gpio_cd);
#else
		free_irq(host->irq_cd, host);
		free_irq(host->irq_mci, host);
#endif

		iounmap(host->base);
		clk_disable(host->clk);
		clk_put(host->clk);

		mmc_free_host(mmc);
	}

	return 0;
}

#ifdef CONFIG_PM
static int ak88_mci_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct ak88_mci_host *host = mmc_priv(mmc);

		ret = mmc_suspend_host(mmc, state);
		if (ret == 0)
			writel(0, host->base + AK88MCIMASK0);
	}

	return ret;
}

static int ak88_mci_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		struct ak88_mci_host *host = mmc_priv(mmc);

		writel(MCI_IRQENABLE, host->base + AK88MCIMASK0);

		ret = mmc_resume_host(mmc);
	}

	return ret;
}
#else
#define ak88_mci_suspend	NULL
#define ak88_mci_resume	NULL
#endif

static struct platform_driver ak88_mci_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
	},
	.probe		= ak88_mci_probe,
	.remove		= __devexit_p(ak88_mci_remove),
	.suspend	= ak88_mci_suspend,
	.resume		= ak88_mci_resume,
};

static int __init ak88_mci_init(void)
{
	PK("%s\n", __func__);
	return platform_driver_register(&ak88_mci_driver);
}

static void __exit ak88_mci_exit(void)
{
	platform_driver_unregister(&ak88_mci_driver);
}

module_init(ak88_mci_init);
module_exit(ak88_mci_exit);

MODULE_DESCRIPTION("Anyka AK88 MMC/SD/SDIO Interface driver");
MODULE_LICENSE("GPL");
