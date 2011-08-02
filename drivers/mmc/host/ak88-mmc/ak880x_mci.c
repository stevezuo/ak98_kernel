/*
 *  linux/drivers/mmc/host/ak880x_mci.c
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
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#include <asm/dma.h>
#include <asm/io.h>
#include <asm/sizes.h>

#include <mach/gpio.h>

#include "ak880x_mci.h"

extern struct mutex nand_lock;

#define DRIVER_NAME	"ak880x-mci"

#if 0
#define dbg(fmt, arg...) printk("%s(%d): " fmt "\n", __func__, __LINE__, ##arg)
#else
#define dbg(fmt, arg...)
#endif

#include "ak880x_l2.c"

#define mci_read(host, reg)		__raw_readl((host)->baseaddr + (reg))
#define mci_write(host, reg, val) 	__raw_writel((val), (host)->baseaddr + (reg))

#define SD_PWRON	AK88_DGPIO_24

struct ak880xmci_host {
	struct mmc_host	*mmc;
	struct clk	*clk;
	unsigned int	irq;
	unsigned long	clkrate;

	void __iomem	*baseaddr;
	int		bus_mode;
	int		bus_width;
	int		flags;
	struct completion	completion;
	struct mmc_request	*request;
	struct mmc_command	*cmd;

	/* for dma */
	unsigned int	*buffer;
	dma_addr_t	phy_addr;
	unsigned int	total_len;
	volatile unsigned int	xfer_index;
	volatile unsigned int	blocks;
};

static void rcv_resp(struct ak880xmci_host *host, struct mmc_command *cmd)
{
	cmd->resp[0] = mci_read(host, MMC_RESP1);
	cmd->resp[1] = mci_read(host, MMC_RESP2);
	cmd->resp[2] = mci_read(host, MMC_RESP3);
	cmd->resp[3] = mci_read(host, MMC_RESP4);
	if (mmc_resp_type(cmd) & MMC_RSP_OPCODE) {
		int rsp_opcode = mci_read(host, MMC_CMD_RESP);
		/* dbg("rsp_opcode(%d)", rsp_opcode); */
	}
}

static void mci_xfer(struct ak880xmci_host *host)
{
	int i = 0;
	struct mmc_data *data = host->cmd->data;
	unsigned int *sgbuffer;

	if (!data)
		return;

	if (host->xfer_index<<2 >= host->total_len) {
		dbg("xfer overflow");
		return;
	}
	if (host->blocks >= data->blocks) {
		dbg("xfer overflow");
		return;
	}

	sgbuffer = sg_virt(data->sg);

	if (data->flags & MMC_DATA_READ) {
		unsigned long regval;
#ifdef L2_USING
		if (host->total_len >= 64) {
			dma_addr_t phy_addr = sg_phys(data->sg) + host->blocks*data->blksz;
			l2_dma(L2_SD_BUFX, phy_addr, data->blksz, 0);

			host->blocks++;
		} else  
#endif
		{
			regval = mci_read(host, MMC_CPU_MODE);
			if (regval) {
				/* dbg("[%d] r,MMC_CPU_MODE(0x%x)", host->xfer_index, regval); */
			}
			sgbuffer[host->xfer_index++] = regval;
		}
	} else if (data->flags & MMC_DATA_WRITE) {
#ifdef L2_USING
		dma_addr_t phy_addr;

		host->blocks++;
		if (host->blocks < data->blocks) {
			phy_addr = sg_phys(data->sg) + host->blocks*data->blksz;
			l2_dma(L2_SD_BUFX, phy_addr, data->blksz, 1);
		}
#else
		unsigned long val;
		val = sgbuffer[host->xfer_index++];
		mci_write(host, MMC_CPU_MODE, val);
		if (val)
			dbg("[%d], w,MMC_CPU_MODE(0x%x)", host->xfer_index, val);
#endif /* L2_USING */
	}
}

static void ak880xmci_setup_data(struct ak880xmci_host *host, struct mmc_data *data)
{
	unsigned int regval, timeout, length;
	struct mmc_command *cmd = host->cmd;

	if (data->blksz & 0x3) {
		dbg("Unsopported block size");
		cmd->error = -EINVAL;
		mmc_request_done(host->mmc, host->request);
		return;
	}

	length = data->blksz * data->blocks;
	timeout = (data->timeout_ns / 1000000) * (host->clkrate / 1000); /* mci clocks */ 
#ifdef L2_USING
	if (length >= 64) {
		regval = 0x1<<0 | 0x1<<16 | (data->blksz>>2)<<17;
		/* regval = 0x1<<0 | (data->blksz>>2)<<17; */
		mci_write(host, MMC_DMA_MODE, regval);
		dbg("dmareg(0x%x)", regval);
	} else
#endif
	{
		regval = mci_read(host, MMC_INT_CTRL);
		mci_write(host, MMC_INT_CTRL, regval | (0x1<<12));
		mci_write(host, MMC_DMA_MODE, 0);
	}

	if (data->flags & MMC_DATA_READ) {
#ifdef L2_USING
		if (length >= 64)
			l2_pre(L2_SD_BUFX, 0, 0, 0);
	} else if (data->flags & MMC_DATA_WRITE) {
		dma_addr_t phy_addr;
		phy_addr = sg_phys(data->sg);
		l2_pre(L2_SD_BUFX, 0, 0, 0);
		l2_dma(L2_SD_BUFX, phy_addr, data->blksz, 1);
#endif
	}

	host->total_len = length;
	host->xfer_index = 0;
	host->blocks = 0;

	mci_write(host, MMC_DATA_TIMER, timeout);
	mci_write(host, MMC_DATA_LENGTH, length);

	regval = (data->blksz & 0xfff) << 16;
	if (host->bus_width == MMC_BUS_WIDTH_4)
		regval |= 0x1<<3;
	if (data->flags & MMC_DATA_STREAM)
		regval |= 0x1<<2;
	if (data->flags & MMC_DATA_READ) 
		regval |= 0x1<<1;
	regval |= 0x1<<0;
	mci_write(host, MMC_DATA_CONTROL, regval);
}

/*
 * #undef dbg(fmt, arg...)
 * #define dbg(fmt, arg...)
 */

#define STATUS_ERROR (0x1<<0 | 0x1<<1 | 0x1<<2 | 0x1<<3 | 0x1<<8)
static irqreturn_t ak880xmci_irq(int irq, void *dev)
{
	int status;
	int completed;
	struct ak880xmci_host *host = (struct ak880xmci_host*)dev;
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = cmd->data;

	status = mci_read(host, MMC_STATUS);
	if (status != 0x1800)
		dbg("status(0x%x)", status);

	if (status & STATUS_ERROR) {
		completed = 2;
		if (status & 0x1<<0) {
			cmd->error |= -EILSEQ;
			printk("Cmd crc check FAILED\n");
		}
		if (status & 0x1<<1) {
			data->error |= -EILSEQ;
			printk("Data crc check FAILED");
		}
		if (status & 0x1<<2) {
			cmd->error |= -ETIMEDOUT;
			dbg("Cmd response is TIMEOUT\n");
		}
		if (status & 0x1<<3) {
			data->error |= -ETIMEDOUT;
			printk("Data is TIMEOUT\n");
		}
		if (status & 0x1<<8) {	
			data->error |= -EILSEQ;
			printk("Start bit ERR\n");
		}
	} else {
		if (status & 0x1<<4) {
			completed = 1;
			rcv_resp(host, cmd);
			dbg("Command response has been received");
		}
		if (status & 0x1<<5) {
			completed = 1;
			dbg("Cmd send successfully, no response");
		}
		if (status & 0x1<<7) { /* data block end */ 
#ifdef L2_USING
			if ((host->total_len >= 64) && data) {
				mci_xfer(host);
				completed = 0;
			}
#endif
			dbg("Data block has been sent/received");
		}
		if (status & 0x1<<9) {
			completed = 0;
			dbg("Transferring command");
		}
		if (status & 0x1<<10) {
			completed = 0;
			/* dbg("Transmitting data"); */
		}
		if (status & 0x1<<11) {
			completed = 0;
		}
		if (status & 0x1<<12) { /* Data buffer full */ 
			if (!mci_read(host, MMC_DMA_MODE)) /* cpu mode */
				if (data && (data->flags & MMC_DATA_READ)) {
					mci_xfer(host);
					completed = 0;
				}
			/* dbg("Data buffer full"); */
		}
		if (status & 0x1<<13) { /* Data buffer empty */ 
#ifndef L2_USING
			if (data && (data->flags & MMC_DATA_WRITE)) {
				if (!(status & 0x1<<4))
					mci_xfer(host);
				completed = 0;
			}
#endif
		}
		if (status & 0x1<<15) { /* half empty */ 
			if (data)
				completed = 0;
		}
		if (status & 0x1<<6) {
			completed = 2;
			dbg("Data transfer ends");
		}
	}

	switch (completed) {
		case 1:
			if (data) {
				ak880xmci_setup_data(host, data);
				break;
			}
		case 2:
			complete(&host->completion);
			break;
		default:
			/* do nothing */
			break;
	}

	return IRQ_HANDLED;
}

static void ak880xmci_setup_cmd(struct ak880xmci_host *host,struct mmc_command *cmd)
{
	unsigned int regval;

	regval = 0x1ff;
	mci_write(host, MMC_INT_CTRL, regval); 

	/* if (cmd->arg) */
	mci_write(host, MMC_CMD_ARG, cmd->arg); /* Needed each time */ 

	regval = 0;
	if (mmc_resp_type(cmd) & MMC_RSP_CRC)
		regval &= ~(0x1<<10);
	else
		regval |= 0x1<<10;
	if (mmc_resp_type(cmd) & MMC_RSP_136)
		regval |= 0x1<<8;
	if (mmc_cmd_type(cmd) !=  MMC_CMD_BC) /* ac, bcr */ 
		regval |= 0x1<<7;
	regval |= (cmd->opcode&0x3f)<<1; /* 6 bits */ 
	regval |= 0x1<<0;
	mci_write(host, MMC_CMD_REG, regval);

	wait_for_completion_interruptible(&host->completion);
}


static void ak880xmci_request_done(struct ak880xmci_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (data) {
		if (!data->error)
			data->bytes_xfered = data->blocks * data->blksz;
		else
			data->bytes_xfered = 0;
	}

	mci_write(host, MMC_INT_CTRL, 0);  /* clear irq */ 
	mci_write(host, MMC_DMA_MODE, 0); /* need */ 
#ifdef L2_USING
	l2_post(L2_SD_BUFX);
#endif

	mmc_request_done(host->mmc, mrq);
	/* printk("         ********\n");  */
}

static void ak880xmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ak880xmci_host *host = mmc_priv(mmc);

	host->request = mrq;
	host->cmd = host->request->cmd;

	if (mutex_lock_interruptible(&nand_lock))
		printk("lock error");

	ak880xmci_setup_cmd(host, mrq->cmd);

	mutex_unlock(&nand_lock);

	if (mrq->stop) {
		host->cmd = host->request->stop;
		ak880xmci_setup_cmd(host, mrq->stop);
	}

	ak880xmci_request_done(host, mrq);
}

static void ak880xmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ak880xmci_host *host = mmc_priv(mmc);
	int regval;

	host->bus_mode = ios->bus_mode;
	host->bus_width = ios->bus_width;
		
	switch (ios->power_mode) {
		case MMC_POWER_UP:
			break;
		case MMC_POWER_ON:
		{	
			/* pull up enable */
			ak880x_gpio_pullup(AK88_GPIO_72, 1);
			ak880x_gpio_pullup(AK88_GPIO_73, 1);
			ak880x_gpio_pullup(AK88_GPIO_74, 1);
			ak880x_gpio_pullup(AK88_GPIO_75, 1);
			ak880x_gpio_pullup(AK88_GPIO_39, 1); /* MMC_MCD */ 
			ak880x_gpio_pullup(AK88_GPIO_40, 1); /* MMC_MCK */ 
			ak880x_gpio_cfgpin(SD_PWRON, AK88_GPIO_OUT_0);
			ak880x_gpio_pullup(SD_PWRON, 1);
			ak880x_gpio_setpin(SD_PWRON, 0);
			AK88_GPIO_MDAT2(1);
			AK88_MCI_ENABLE();

			break;
		}
		case MMC_POWER_OFF:
		default:
		{
			AK88_MCI_DISABLE();
			AK88_GPIO_MDAT2(0);
			ak880x_gpio_cfgpin(SD_PWRON, AK88_GPIO_OUT_0);
			ak880x_gpio_pullup(SD_PWRON, 1);
			ak880x_gpio_setpin(SD_PWRON, 1);
			ak880x_gpio_pullup(AK88_GPIO_72, 1);
			ak880x_gpio_pullup(AK88_GPIO_73, 1);
			ak880x_gpio_pullup(AK88_GPIO_74, 1);
			ak880x_gpio_pullup(AK88_GPIO_75, 1);
			ak880x_gpio_pullup(AK88_GPIO_39, 1); /* MMC_MCD */ 
			ak880x_gpio_pullup(AK88_GPIO_40, 1); /* MMC_MCK */ 
			break;

		}
	}
	
	if (ios->clock == 0) {
		/* Disable SD clock */
                /*
		 * regval = mci_read(host, MMC_CLK_CTRL);
		 * regval &= ~(0x1<<20 | 0x1<<16);
                 */
		regval = 0;
	} else {
		/* Enable & setup SD clock */
		clk_enable(host->clk);
		clk_set_rate(host->clk, ios->clock);
		regval = mci_read(host, MMC_CLK_CTRL);
		regval |= 0x1<<20 | 0x1<<19 | 0x1<<17 | 0x1<<16;
	}
	mci_write(host, MMC_CLK_CTRL, regval);
	host->clkrate = clk_get_rate(host->clk);
	
	/* printk("ios->clock(%d), host->clkrate(%d), MMC_CLK_CTRL(0x%x)\n", ios->clock, host->clkrate, mci_read(host, MMC_CLK_CTRL)); */
}

static int ak880xmci_get_ro(struct mmc_host *host)
{
	dbg("");
	return 0;
}

static struct mmc_host_ops ak880xmci_ops = {
	.request	= ak880xmci_request,
  	.set_ios	= ak880xmci_set_ios,
	.get_ro		= ak880xmci_get_ro,
};

static int ak880xmci_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct ak880xmci_host *host;
	struct resource *res;
	int ret;

#ifdef L2_USING
	/* rCLK_CON &= (0x1<<15); [> lTo enable L2 controller/UART1 working clock <] */
	/* l2_init(); */
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dbg("");
		return -ENXIO;
	}
	if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME)) {
		dbg("");
		return -EBUSY;
	}

	mmc = mmc_alloc_host(sizeof(struct ak880xmci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		dbg("");
		goto err1;
	}

	mmc->ops	= &ak880xmci_ops;
	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_31_32;
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 8;
	mmc->max_req_size = 512*8;
	mmc->f_min	= 300*1000; /* <400MHz */ 
	mmc->f_max	= 20*1000*1000;

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->irq	= IRQ_MMC_SD;
	host->clk	= clk_get(&pdev->dev, "mci_clk");
	if (IS_ERR(host->clk)) {
		ret = -ENODEV;
		goto err2;
	}

	host->baseaddr = ioremap(res->start, res->end - res->start + 1);
	if (!host->baseaddr) {
		ret = -ENOMEM;
		goto err3;
	}

	platform_set_drvdata(pdev, mmc);

	init_completion(&host->completion);

	if (request_irq(host->irq, ak880xmci_irq, 0, DRIVER_NAME, host)) {
		ret = -EBUSY;
		goto err4;
	}
	
	if (mmc_add_host(mmc)) {
		goto err5;
	}

	return 0;

err5:
	free_irq(host->irq, host);
err4:
	iounmap(host->baseaddr);
err3:
	clk_put(host->clk);
err2:
	mmc_free_host(mmc);
err1:
	release_mem_region(res->start, res->end - res->start + 1);

	return ret;
}

static int ak880xmci_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct ak880xmci_host *host = mmc_priv(mmc);
	struct resource *res;

	mmc_remove_host(mmc);
	free_irq(host->irq, host);
	platform_set_drvdata(pdev, NULL);
	iounmap(host->baseaddr);
	clk_put(host->clk);
	mmc_free_host(mmc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int ak880xmci_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

        /*
	 * if (mmc)
	 *         ret = mmc_suspend_host(mmc, state);
         */

	return ret;
}

static int ak880xmci_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

        /*
	 * if (mmc)
	 *         ret = mmc_resume_host(mmc);
         */

	return ret;
}
#else
#define ak880xmci_suspend	NULL
#define ak880xmci_resume	NULL
#endif

static struct platform_driver ak880xmci_driver = {
	.probe		= ak880xmci_probe,
	.remove		= ak880xmci_remove,
	.suspend	= ak880xmci_suspend,
	.resume		= ak880xmci_resume,
	.driver		= {
		.name	= "ak880x-mci",
	},
};

static int __init ak880xmci_init(void)
{
	dbg("Build at %s %s", __DATE__, __TIME__);
	return platform_driver_register(&ak880xmci_driver);
}

static void __exit ak880xmci_exit(void)
{
	platform_driver_unregister(&ak880xmci_driver);
}

module_init(ak880xmci_init);
module_exit(ak880xmci_exit);

MODULE_DESCRIPTION("ANYKA AK88 MMC/SD Interface Driver");
MODULE_LICENSE("GPL");
