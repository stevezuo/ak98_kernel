/*
 * drivers/spi/spi_ak880x.c
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/io.h>

#include <mach/spi.h>
#include <mach/gpio.h>

/* #define DEBUG */
#ifdef DEBUG
#define pk_debug(fmt, arg...)	printk(fmt,##arg)
#else
#define pk_debug(fmt, arg...)
#endif


struct ak880x_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;

	void			(*set_cs)(struct ak880x_spi_info *spi, int cs, int pol);

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct ak880x_spi_info	*pdata;
};

#define SPICON_DEFAULT	(AK88_SPICON_MS | AK88_SPICON_CLKDIV)
#define SPIINT_DEFAULT	(AK88_SPIINT_RXHFULL | AK88_SPIINT_TXHFULL)

static inline struct ak880x_spi *to_hw(struct spi_device *sdev)
{
		return spi_master_get_devdata(sdev->master);
}

static void ak880x_spi_chipsel(struct spi_device *spi, int value)
{
	struct ak880x_spi *hw = to_hw(spi);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned int spicon;

	pk_debug("***%s******\n", __FUNCTION__);

	switch(value) {
		case BITBANG_CS_INACTIVE:
			hw->set_cs(hw->pdata, spi->chip_select, cspol^1);
			break;
		case BITBANG_CS_ACTIVE:
			spicon = ioread32(hw->regs + AK88_SPICON);
			if (spi->mode & SPI_CPHA)
				spicon |= AK88_SPICON_CPHA;
			else
				spicon &= ~AK88_SPICON_CPHA;

			if (spi->mode & SPI_CPOL)
				spicon |= AK88_SPICON_CPOL;
			else
				spicon &= ~AK88_SPICON_CPOL;
			
			iowrite32(spicon, hw->regs + AK88_SPICON);
			hw->set_cs(hw->pdata, spi->chip_select, cspol);
			break;
	}

}

static int ak880x_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak880x_spi *hw = to_hw(spi);
	unsigned int bpw;
	unsigned int hz;
	unsigned int div;
	unsigned int spicon;

	pk_debug("***%s******\n", __FUNCTION__);

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	div = clk_get_rate(hw->clk)*1000*1000 / hz;
	div = div/2 - 1;

	if (div > 255)
		div = 255;

	spicon = ioread32(hw->regs + AK88_SPICON);
	iowrite32((div << 8) | (spicon & ~(0xff<<8)), hw->regs + AK88_SPICON);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_ACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)
static int ak880x_spi_setup(struct spi_device *spi)
{
	int ret;

	pk_debug("***%s******\n", __FUNCTION__);

	if (!spi->bits_per_word)
		spi->bits_per_word = 32;

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	ret = ak880x_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__FUNCTION__, spi->mode, spi->bits_per_word,
		spi->max_speed_hz);

	return 0;
}

#if 0
static inline unsigned int hw_txbyte(struct ak880x_spi *hw, int count)
{
		//return hw->tx ? hw->tx[count] : 0;
		return hw->tx ? *(int*)(hw->tx+count) : 0;
}
#endif

static inline void ak880x_spi_setirq(int type, int sw)
{
	if (sw) {
		rSPI1_INTEN |= type;
	} else {
		rSPI1_INTEN &= (~type);
	}
}
	

#if 0
static void read_id(void)
{
	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x30;
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);

	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x38;
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);
	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x38;
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);
	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x38;
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);
	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x38;
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);
}
#endif

static int ak880x_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak880x_spi *hw = to_hw(spi);
	u32 spicon = ioread32(hw->regs + AK88_SPICON);

	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

	if (!hw->tx) { /* read only */
		/* spicon = ioread32(hw->regs + AK88_SPICON); */
	} else if (!hw->rx) {/* write only */
		/* spicon = ioread32(hw->regs + AK88_SPICON); */
		iowrite32(spicon, hw->regs + AK88_SPICON);
		iowrite32(spicon | AK88_SPICON_ARRM | AK88_SPICON_CS, hw->regs + AK88_SPICON);
		iowrite32(hw->len, hw->regs + AK88_SPICNT);
		iowrite32(*(int*)hw->tx, hw->regs + AK88_SPIOUT);
		while (ioread32(hw->regs + AK88_SPISTA) & AK88_SPISTA_MPROC);
		mdelay(1);
		iowrite32(spicon, hw->regs + AK88_SPICON);
	} else {
		/* spicon = ioread32(hw->regs + AK88_SPICON); */
		iowrite32(spicon & ~(AK88_SPICON_TGDM | AK88_SPICON_ARRM), hw->regs + AK88_SPICON);
	}

#if 0
	mdelay(1);
	rSPI1_CON |= (1<<5);
	mdelay(1);
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0x08;
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	mdelay(1);
	/* printk("in: 0x%x\n", rSPI1_INDATA); */
	rSPI1_COUNT = 1;
	rSPI1_OUTDATA = 0xff;
	mdelay(1);
	printk("in: 0x%x\n", rSPI1_INDATA);
	mdelay(1);
	rSPI1_CON &= ~(1<<5);
#endif

	pk_debug("rSPI1_CON = 0x%x,\trSPI1_COUNT = 0x%x,\trSPI1_INTEN = 0x%x,\t rCLK_CON = 0x%x,\t extx: %x,\t exrx: %x,\t rSPI1_STAT = 0x%x\n",
		       	rSPI1_CON, rSPI1_COUNT, rSPI1_INTEN, rCLK_CON, rSPI1_TXBUF, rSPI1_RXBUF, rSPI1_STAT);

	/* wait_for_completion(&hw->done); */

	/* iowrite32(spicon & ~(AK88_SPICON_TGDM | AK88_SPICON_ARRM | AK88_SPICON_CS), hw->regs + AK88_SPICON); */
	
	return hw->count;
}

static irqreturn_t ak880x_spi_irq(int irq, void *dev)
{
	u32 n;
	struct ak880x_spi *hw = dev;

#if 0
	n = (hw->len >= 4) ? 4 : hw->len;

	/* iowrite32(n, hw->regs + AK88_SPICNT); */

	if (hw->tx) {
		iowrite32(hw->len, hw->regs + AK88_SPICNT); /* one byte one time */
		/* iowrite32(n, hw->regs + AK88_SPICNT); */
		/* iowrite32(*(int*)(hw->tx + hw->count), hw->regs + AK88_SPIOUT); */
		iowrite32(hw->tx[0], hw->regs + AK88_SPIOUT);
		printk("n: %d, len: %d, count: %d, hw->tx[0]: %x\n", n, hw->len, hw->count,  hw->tx[0]);
	}
		

	if (hw->rx) {
		iowrite32(hw->len, hw->regs + AK88_SPICNT); /* one byte one time */
		iowrite32(0xffffffff, hw->regs + AK88_SPIOUT);
		printk("\n***rSPI1_INDATA : 0x%x***\n", rSPI1_INDATA);
		if (n == 4) {
			*(int*)(hw->rx+hw->count) = ioread32(hw->regs + AK88_SPIIN);
		} else {
			int tmp = ioread32(hw->regs + AK88_SPIIN);
			memcpy(hw->rx + hw->count, (char*)&tmp, n);
		}
	}

	hw->count += n;
	hw->len -= n;

	while (rSPI1_COUNT);

#endif
	if (!hw->len) {
		/* disable_irq(hw->irq); */
		ak880x_spi_setirq(AK88_SPIINT_TXHEMP, 0);
		ak880x_spi_setirq(AK88_SPIINT_RXHFULL, 0);
		ak880x_spi_setirq(AK88_SPIINT_TIMEOUT, 0);
		ak880x_spi_setirq(AK88_SPIINT_RXEMP, 0);
		complete(&hw->done);
	}
	pk_debug("rSPI1_COUNT : 0x%x,\t rSPI1_STAT : 0x%x\n", rSPI1_COUNT, rSPI1_STAT);

	return IRQ_HANDLED;
}

static void ak880x_spi_gpiocs(struct ak880x_spi_info *spi, int cs, int pol)
{
	/* ak880x_gpio_setpin(spi->pin_cs, pol); */
}

static int __init ak880x_spi_probe(struct platform_device *pdev)
{
	struct ak880x_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;
	unsigned int spicon = 0;
	unsigned int spiint = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ak880x_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct ak880x_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;


	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the state for the bitbang driver */


	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = ak880x_spi_setupxfer;
	hw->bitbang.chipselect     = ak880x_spi_chipsel;
	hw->bitbang.txrx_bufs      = ak880x_spi_txrx;
	hw->bitbang.master->setup  = ak880x_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */
#if 1
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);
	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	err = request_irq(hw->irq, ak880x_spi_irq, 0, pdev->name, hw);
	if (err < 0) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}
	/* disable_irq(hw->irq); */

	hw->clk = clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_no_clk;
	} 
	/* for the moment, permanently enable the clock */
	clk_enable(hw->clk);

#endif
	/* program defaults into the registers */
	spicon = AK88_SPICON_CLKDIV
		| AK88_SPICON_EN 
		| AK88_SPICON_MS 
		| AK88_SPICON_CPHA
//		| AK88_SPICON_ARRM 
//		| AK88_SPICON_TGDM
		;

	iowrite32(spicon, hw->regs + AK88_SPICON);
	
	//spiint = AK88_SPIINT_RXEMP | AK88_SPIINT_TXHEMP;
	/* spiint = AK88_SPIINT_TXHEMP; */
	spiint = 0;
	iowrite32(spiint, hw->regs + AK88_SPIINT);

#define IRQ_MASK_SPI1 (1<<18);
	//rIRQ_MASK |= IRQ_MASK_SPI1;
	
	pk_debug("rSPI1_CON = 0x%x,\trSPI1_COUNT = 0x%x,\trSPI1_INTEN = 0x%x,\t rCLK_CON = 0x%x,\t rSPI1_STAT = 0x%x\n",
		       	rSPI1_CON, rSPI1_COUNT, rSPI1_INTEN, rCLK_CON, rSPI1_STAT);

	/* setup any gpio we can */
	hw->set_cs = ak880x_spi_gpiocs;

	/* register our spi controller */

	master->num_chipselect = 1;
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	//enable_irq(hw->irq);
	return 0;

 err_register:
	clk_disable(hw->clk);
	clk_put(hw->clk);

 err_no_clk:
	free_irq(hw->irq, hw);

 err_no_irq:
	iounmap(hw->regs);

 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}

static int ak880x_spi_remove(struct platform_device *dev)
{
	struct ak880x_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM

static int ak880x_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ak880x_spi *hw = platform_get_drvdata(pdev);

	clk_disable(hw->clk);
	return 0;
}

static int ak880x_spi_resume(struct platform_device *pdev)
{
	struct ak880x_spi *hw = platform_get_drvdata(pdev);

	clk_enable(hw->clk);
	return 0;
}

#else
#define ak880x_spi_suspend NULL
#define ak880x_spi_resume  NULL
#endif

MODULE_ALIAS("platform:ak880x_spi");
static struct platform_driver ak880x_spidrv = {
	.remove		= __exit_p(ak880x_spi_remove),
	.suspend	= ak880x_spi_suspend,
	.resume		= ak880x_spi_resume,
	.driver		= {
		.name	= "ak7801-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init ak880x_spi_init(void)
{
	printk("AK88 SPI Driver, (c) 2010 ANYKA\n");

	return platform_driver_probe(&ak880x_spidrv, ak880x_spi_probe);
}

static void __exit ak880x_spi_exit(void)
{
	platform_driver_unregister(&ak880x_spidrv);
}

module_init(ak880x_spi_init);
module_exit(ak880x_spi_exit);

MODULE_AUTHOR("anyka");
MODULE_DESCRIPTION("AK88 spi support");
MODULE_LICENSE("GPL");
