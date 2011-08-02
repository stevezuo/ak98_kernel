/* linux/drivers/spi/spi_ak98.c
 *  modify based on  spi_s3c24xx.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <linux/io.h>
#include <mach/gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <mach/spi.h>
#include <mach/clock.h>


//#define SPI_DEBUG
/* #define DEBUG */
#undef PDEBUG           /* undef it, just in case */
#ifdef SPI_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
# define PDEBUG(fmt, args...) printk( KERN_INFO fmt,## args)
# else
/* This one for user space */
# define PDEBUG(fmt, args...) fprintf(stderr, "%s %d: "fmt,__FILE__, __LINE__, ## args)
# endif
#else
# define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif


#define READ_TIMEOUT 10000000
#define TRANS_TIMEOUT 10000000
#define MAX_LEN 64*1024
#define AK98SPI_DELAY 100000
/**
 * ak98_spi_devstate - per device data
 * @hz: Last frequency calculated for @sppre field.
 * @mode: Last mode setting for the @spcon field.
 * @spcon: Value to write to the SPCON register.
 * @sppre: Value to write to the SPIINT register.
 */
struct ak98_spi_devstate {
	unsigned int	hz;
	u16	mode;
	u16		spcon;
	u8		spint;
	u8		div;
};

struct ak98_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 count;

	void			(*set_cs)(struct ak98_spi_info *spi,
					  int cs, int pol);

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct ak98_spi_info *pdata;
};

#define DFT_CON ( AK98_SPICON_EN | AK98_SPICON_MS)
#define DFT_DIV		127
#define FORCE_CS   1 << 5
#define SPPIN_DEFAULT (0)

#if 0
static void spi_reg_print(struct ak98_spi *hw)
{
	PDEBUG("\n");
	PDEBUG("CON: \t0x%x\n", ioread32(hw->regs + AK98_SPICON));
	PDEBUG("STA: \t0x%x\n", ioread32(hw->regs + AK98_SPISTA));
	PDEBUG("INT: \t0x%x\n", ioread32(hw->regs + AK98_SPIINT));
	PDEBUG("CNT: \t0x%x\n", ioread32(hw->regs + AK98_SPICNT));
	PDEBUG("DOUT: \t0x%x\n", ioread32(hw->regs + AK98_SPIOUT));
	PDEBUG("DIN: \t0x%x\n", ioread32(hw->regs + AK98_SPIIN));
}
#endif

static inline struct ak98_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void ak98_spi_gpiocs(struct ak98_spi_info *spi, int cs, int pol)
{
	//gpio_set_value(spi->pin_cs, pol);
}

static void ak98_spi_chipsel(struct spi_device *spi, int value)
{
	//struct ak98_spi_devstate *cs = spi->controller_state;
	struct ak98_spi *hw = to_hw(spi);
	//unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned int spicon;

	//PDEBUG("Entering %s\n", __FUNCTION__);
	/* change the chipselect state and the state of the spi engine clock */	
	switch (value) {
	case BITBANG_CS_INACTIVE:
		PDEBUG("BITBANG_CS_INACTIVE\n");
		//hw->set_cs(hw->pdata, spi->chip_select, cspol^1);
		//writeb(cs->spcon, hw->regs + S3C2410_SPCON);
		break;

	case BITBANG_CS_ACTIVE:
		PDEBUG("BITBANG_CS_ACTIVE");
		spicon = ioread32(hw->regs + AK98_SPICON);
		if (spi->mode & SPI_CPHA)
			spicon |= AK98_SPICON_CPHA;
		else
			spicon &= ~AK98_SPICON_CPHA;
		if (spi->mode & SPI_CPOL)
			spicon |= AK98_SPICON_CPOL;
		else
			spicon &= ~AK98_SPICON_CPOL;

		iowrite32(spicon  , hw->regs + AK98_SPICON);
		//hw->set_cs(hw->pdata, spi->chip_select, cspol);
		break;
	}
}

static int ak98_spi_update_state(struct spi_device *spi,
				    struct spi_transfer *t)
{
	//struct ak98_spi *hw = to_hw(spi);
	struct ak98_spi_devstate *cs = spi->controller_state;
	unsigned int bpw;
	unsigned int hz;
	unsigned int div;
	unsigned long clk;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if (!bpw)
		bpw = 8;

	if (!hz)
		hz = spi->max_speed_hz;

	if (bpw != 8) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}
	
	if (spi->mode != cs->mode) {
		u16 spcon = DFT_CON;
	printk("set mode---------------------------.\n");
		if ( (spi->mode & SPI_CPHA) == SPI_CPHA)
			spcon |= AK98_SPICON_CPHA;

		if ( (spi->mode & SPI_CPOL) == SPI_CPOL)
			spcon |= AK98_SPICON_CPOL;

		cs->mode = spi->mode;
		cs->spcon = spcon;
	}

	PDEBUG("cs->hz: %u\n", cs->hz);
	if (cs->hz != hz) {
		clk = ak98_get_asic_clk();
		PDEBUG("hz: %u\n", hz);
		PDEBUG("clk: %lu\n", clk);
		div = clk / (hz*2) - 1;

		if (div > 255)
			div = 255;

		dev_dbg(&spi->dev, "pre-scaler=%d (wanted %d, got %ld)\n",
			div, hz, clk / (2 * (div + 1)));

		cs->hz = hz;
		cs->div = div;
		PDEBUG("cs div: %u\n", cs->div);
	}

	return 0;
}

static int ak98_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct ak98_spi_devstate *cs = spi->controller_state;
	struct ak98_spi *hw = to_hw(spi);
	int ret;
	//PDEBUG("Entering %s\n", __FUNCTION__);
	ret = ak98_spi_update_state(spi, t);
	//PDEBUG("cs->div: %u\n", cs->div);
	if (!ret)//cs->div
		iowrite32(cs->div << 8 | cs->spcon, hw->regs + AK98_SPICON);

	return ret;
}

#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int ak98_spi_setup(struct spi_device *spi)
{
	struct ak98_spi_devstate *cs = spi->controller_state;
	struct ak98_spi *hw = to_hw(spi);
	int ret;

	//PDEBUG("Entering %s %u %u %u\n", __FUNCTION__, spi->max_speed_hz, spi->mode, spi->chip_select);
	/* allocate settings on the first call */
	if (!cs) 
	{
		cs = kzalloc(sizeof(struct ak98_spi_devstate), GFP_KERNEL);
		if (!cs) 
		{
			dev_err(&spi->dev, "no memory for controller state\n");
			return -ENOMEM;
		}

		cs->spcon = DFT_CON;
		cs->hz = -1;
		cs->div = DFT_DIV;
		spi->controller_state = cs;
	}

	if (spi->mode & ~(hw->pdata->mode_bits)) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~(hw->pdata->mode_bits));
		return -EINVAL;
	}
	/* initialise the state from the device */
	ret = ak98_spi_update_state(spi, NULL);
	if (ret)
		return ret;

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static void ak98_spi_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
}

static void ak98_set_spi_irq(struct ak98_spi *hw, int enable)
{	
	u32 spiint = AK98_SPIINT_TRANSF | AK98_SPIINT_TXFULL | AK98_SPIINT_RXFULL; 
	if (enable)
		iowrite32(spiint, hw->regs + AK98_SPIINT);
	else
		iowrite32(0, hw->regs + AK98_SPIINT);	
}


static inline unsigned int hw_txdword(struct ak98_spi *hw, int count)
{
	u32 val = 0;
	int i = 0;
	while (i<4)
	{
		#if 0
		val = (val << 8) | hw->tx[count+i];
		#else
		val = val | (hw->tx[count+i] << i*8);
		#endif
		i++;
	}
	return val;
}

static int ak98_spi_readCPU(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak98_spi *hw = to_hw(spi);

	//u32 read_4_nbr = hw->len/4;
	u32 frac_nbr = hw->len%4;
	u32 status, val;
	u32 off_set = 0;
	u8 *buff = hw->rx;
	int i;
	u32 to=0;
	

	if (hw->len >= 64*1024)
	{
		printk("Too much to be read...\n");
		return -EINVAL;
	}
	PDEBUG("read CPU...\n");
	PDEBUG("%u  %u read CPU...\n", read_4_nbr, frac_nbr);
	//set read only
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(AK98_SPICON_ARRM);
	val |= AK98_SPICON_TGDM;
	if (t->cs_change == 0)
		val |= FORCE_CS;
	iowrite32(val, hw->regs + AK98_SPICON);
	udelay(10);

	//set data count, and the the master will rise clk
	iowrite32(hw->len, hw->regs + AK98_SPICNT);
	udelay(10);
	
	
	while(1)
	{
		status = ioread32(hw->regs + AK98_SPISTA);	
		
		
		if ( (status & AK98_SPISTA_TRANSF) == AK98_SPISTA_TRANSF)
		{
			if (status & AK98_SPISTA_RXFULL)
			{
				PDEBUG("RXFULL\n");
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff + off_set) = val;
				off_set += 4;
				PDEBUG("%x\n", val);
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff + off_set) = val;
				off_set += 4;
				PDEBUG("%x\n", val);
			}
			else if (status & AK98_SPISTA_RXHFULL)
			{
				PDEBUG("RXHFULL\n");
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff + off_set) = val;
				off_set += 4;
			}
			if (frac_nbr != 0)
			{
				val = ioread32(hw->regs + AK98_SPIIN);
				printk("-------------%x\n", val);
				for (i=0; i<frac_nbr; i++)
				{
					*(buff+off_set+i) = (val >> i*8) & 0xff;
				}
			}
			break;
		}
		else
		{
			if ( (status & AK98_SPISTA_RXHFULL) == AK98_SPISTA_RXHFULL)
			{
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff + off_set) = val;
				off_set += 4;
			}
			else
			{
				if (to++ > 10 * 1000000)
				{
					PDEBUG("master read timeout...\n");
					return off_set;
				}
			}
		}	
	}
	if (off_set + frac_nbr != hw->len)
		PDEBUG("read wasn't finished...\n");
	
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(FORCE_CS);
	iowrite32(val, hw->regs + AK98_SPICON);
	
	return hw->len;
	
}

static int ak98_spi_writeCPU(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak98_spi *hw = to_hw(spi);

	u32 tran_4_nbr = hw->len/4;
	u32 frac_nbr = hw->len%4;
	u32 status, val;
	u32 off_set = 0;
	const u8 *buff = hw->tx;
	int i;
	u32 to = 0;


	PDEBUG("write... %u  %u\n", tran_4_nbr, frac_nbr);
	if (hw->len >= 64*1024)
	{
		printk("Too much to be send...\n");
		return -EINVAL;
	}
	//set transfer only
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(AK98_SPICON_TGDM);
	val |= AK98_SPICON_ARRM;
	if (t->cs_change == 0)
		val |= FORCE_CS;

	iowrite32(val, hw->regs + AK98_SPICON);
	udelay(10);
	//set data count, and the the master will rise clk
	iowrite32(hw->len, hw->regs + AK98_SPICNT);
	udelay(10);
	
	for (i=0; i<tran_4_nbr; i++)
	{
		while(1)
		{
			status = ioread32(hw->regs + AK98_SPISTA);
			if ((status & AK98_SPISTA_TXHEMP) == AK98_SPISTA_TXHEMP)
				break;
		}
		iowrite32(*(volatile u32 *)(buff + off_set), hw->regs + AK98_SPIOUT);
		off_set += 4;
	}

	if (frac_nbr != 0)
	{
		val = 0;
		while(1)
		{
			status = ioread32(hw->regs + AK98_SPISTA);
			if ((status & AK98_SPISTA_TXHEMP) == AK98_SPISTA_TXHEMP)
				break;
			if (to++ > 31 * 2000000)
			{
				printk("SPI master write timeout...\n");
				to = 0;
				return off_set;
			}
		}
		
		for (i=0; i<frac_nbr; i++)
		{
			val |= (*(buff + off_set + i) << (i*8));
		}
		PDEBUG("-----%x -----\n", val);
		iowrite32(val, hw->regs + AK98_SPIOUT);		
	}

	//wait transfer finish
	while (1)
	{
		PDEBUG("wait...\n");
		status = ioread32(hw->regs + AK98_SPISTA);		
		if ((status & AK98_SPISTA_TRANSF) == AK98_SPISTA_TRANSF)
			break;
		if (to++ > 10 * 1000000)
		{
			printk("Master transfer timeout...\n");	
			return off_set;
		}
			
	}

	if (off_set + frac_nbr != hw->len)
		PDEBUG("write wasn't finished...\n");
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(FORCE_CS);
	iowrite32(val, hw->regs + AK98_SPICON);
	
	return hw->len;
}

static int ak98_spi_duplexCPU(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak98_spi *hw = to_hw(spi);
	u32 tran_4_nbr = hw->len/4;
	u32 frac_nbr = hw->len%4;
	u32 status, val;
	u32 off_set_read = 0, off_set_write = 0;
	const u8 *buff_tx = hw->tx;
	u8 *buff_rx = hw->rx;
	int i = 0, j;
	u32 to_read = 0, to_write = 0, to = 0;

	PDEBUG("duplex...\n");
	PDEBUG("tran_4_nbr:%u  frac_nbr:%u\n", tran_4_nbr, frac_nbr);
	
	if (hw->len >= MAX_LEN)
	{
		printk("Too much to be read and send...\n");
		return -EINVAL;
	}

	//close ARRM and TGDM
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(AK98_SPICON_ARRM | AK98_SPICON_TGDM);
	//whether #CS signal keep low
	if (t->cs_change == 0)
		val |= FORCE_CS;
	
	iowrite32(val, hw->regs + AK98_SPICON);
	
	udelay(10);
	//set data count, and the the master will rise clk
	iowrite32(hw->len, hw->regs + AK98_SPICNT);
	
	udelay(10);
	
	while(1)
	{
		//write 4 bytes first, and then read 4 bytes
		if (i<tran_4_nbr)
		{
			while(1)
			{
				status = ioread32(hw->regs + AK98_SPISTA);
				if ((status & AK98_SPISTA_TXHEMP) == AK98_SPISTA_TXHEMP)
				{
					PDEBUG("TX HEMP...\n");
					break;
				}
				else
				{
					if(to_write++ > TRANS_TIMEOUT)
					{
						PDEBUG("master transfer timeout...\n");
						goto SPI_TRANS_TIMEOUT;
					}
				}
			}			
			iowrite32(*(volatile u32 *)(buff_tx + off_set_write), hw->regs + AK98_SPIOUT);
			off_set_write += 4;
			i++;
		}
		//write not finished
		else if (off_set_write < hw->len)
		{			
			PDEBUG("write frac...\n");
			to_write = 0;
			val = 0;
			if (frac_nbr != 0)
			{
				while(1)
				{
					status = ioread32(hw->regs + AK98_SPISTA);
					if ((status & AK98_SPISTA_TXHEMP) == AK98_SPISTA_TXHEMP)
						break;
					if (to_write++ > TRANS_TIMEOUT)
					{
						printk("SPI master write timeout...\n");						
						goto SPI_TRANS_TIMEOUT;
					}
				}
					
				for (j=0; j<frac_nbr; j++)
				{
					PDEBUG("[%d]:%x ", off_set_write+j, *(buff_tx+off_set_write+j));
					val |= (*(buff_tx + off_set_write + j) << (j*8));
				}
				PDEBUG("\nval: %x", val);
				PDEBUG("\n\n");

				iowrite32(val, hw->regs + AK98_SPIOUT); 
				off_set_write += frac_nbr;
			}
		}

		
		//read
		status = ioread32(hw->regs + AK98_SPISTA);			
		
		if ( (status & AK98_SPISTA_TRANSF) == AK98_SPISTA_TRANSF)
		{
			if (status & AK98_SPISTA_RXFULL)
			{
				PDEBUG("RXFULL\n");
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff_rx + off_set_read) = val;
				off_set_read += 4;
				PDEBUG("%x\n", val);
				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff_rx + off_set_read) = val;
				off_set_read += 4;
				PDEBUG("%x\n", val);
			}
			else if (status & AK98_SPISTA_RXHFULL)
			{
				PDEBUG("RXHFULL\n");

				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff_rx + off_set_read) = val;
				off_set_read += 4;
			}
			if (frac_nbr != 0)
			{
				PDEBUG("read frac...\n");
				val = ioread32(hw->regs + AK98_SPIIN);
				PDEBUG("read ... %x\n", val);
				for (j=0; j<frac_nbr; j++)
				{
					*(buff_rx+off_set_read+j) = (val >> j*8) & 0xff;
					PDEBUG("%x ", *(buff_rx+off_set_read+j));
				}
			}
			break;
		}
		else
		{
			if ( (status & AK98_SPISTA_RXHFULL) == AK98_SPISTA_RXHFULL)
			{
				PDEBUG("RX HFULL...\n");

				val = ioread32(hw->regs + AK98_SPIIN);
				*(volatile u32 *)(buff_rx + off_set_read) = val;
				PDEBUG("rx hfull .. %x\n", val);
				PDEBUG("[0]%x [1]%x [2]%x [3]%x\n", buff_rx[0], buff_rx[1], buff_rx[2], buff_rx[3]);
				off_set_read += 4;
			}
			else
			{
				if (to_read++ > READ_TIMEOUT)
				{
					PDEBUG("master read timeout...\n");
					goto SPI_READ_TIMEOUT;
				}
			}
		}	
	}
	
	val = ioread32(hw->regs + AK98_SPICON);
	val &= ~(FORCE_CS);
	iowrite32(val, hw->regs + AK98_SPICON);
	
	return hw->len;
	SPI_TRANS_TIMEOUT:
	SPI_READ_TIMEOUT:
		val = ioread32(hw->regs + AK98_SPICON);
		val &= ~(FORCE_CS);		
		iowrite32(val, hw->regs + AK98_SPICON);
		
		return off_set_read > off_set_write ? off_set_read: off_set_write;		
	
}


static int ak98_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ak98_spi *hw = to_hw(spi);
	
	PDEBUG("txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);
	
	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;
	
	//spi_reg_print(hw);
	//ak98_prepare_transfer(spi, t);
	
	if (hw->tx && hw->rx)
	{
		PDEBUG("Duplex transfer\n");
		return ak98_spi_duplexCPU(spi, t);
	}
	if (hw->tx)
	{
		PDEBUG("write only...\n");
		return ak98_spi_writeCPU(spi, t);
	}
	if (hw->rx)
	{
		PDEBUG("read only...\n");
		return ak98_spi_readCPU(spi, t);
	}
	
	return 0;
}



static irqreturn_t ak98_spi_irq(int irq, void *dev)
{

	struct ak98_spi *hw = dev;
	PDEBUG("Entering %s  %u %u\n", __FUNCTION__, hw->count, hw->len);
	
 irq_done:
	return IRQ_HANDLED;
}


static void ak98_spi_initialsetup(struct ak98_spi *hw)
{
	/* for the moment, permanently enable the clock */
	PDEBUG("Entering %s\n", __FUNCTION__);
	clk_enable(hw->clk);

	/* program defaults into the registers */

	iowrite32(DFT_DIV<<8 | DFT_CON, hw->regs + AK98_SPICON);
	ak98_set_spi_irq(hw, 0);
	
	if (hw->pdata) 
	{
		if (hw->pdata->gpio_setup)
			hw->pdata->gpio_setup(hw->pdata, 1);
	}
}

static int __init ak98_spi_probe(struct platform_device *pdev)
{
	struct ak98_spi_info *pdata;
	struct ak98_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;

	PDEBUG("Entering %s\n", __FUNCTION__);
	master = spi_alloc_master(&pdev->dev, sizeof(struct ak98_spi));
	if (master == NULL) 
	{
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct ak98_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;

	if (pdata == NULL) 
	{
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the master state. */

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = hw->pdata->mode_bits;

	master->num_chipselect = hw->pdata->num_cs;
	master->bus_num = pdata->bus_num;

	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = ak98_spi_setupxfer;
	hw->bitbang.chipselect     = ak98_spi_chipsel;
	hw->bitbang.txrx_bufs      = ak98_spi_txrx;

	hw->master->setup  = ak98_spi_setup;
	hw->master->cleanup = ak98_spi_cleanup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, resource_size(res),
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->regs = ioremap(res->start, resource_size(res));
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

	err = request_irq(hw->irq, ak98_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	hw->clk = clk_get(&pdev->dev, pdata->clk_name);

	PDEBUG("%s: %lu  \n", hw->clk->name, clk_get_rate(hw->clk));
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_no_clk;
	}

	/* setup any gpio we can */

	ak98_spi_initialsetup(hw);

	/* register our spi controller */

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

 err_register:
	//if (hw->set_cs == ak98_spi_gpiocs)
	//	gpio_free(pdata->pin_cs);

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
	spi_master_put(hw->master);

 err_nomem:
	return err;
}

static int __exit ak98_spi_remove(struct platform_device *dev)
{
	struct ak98_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	//if (hw->set_cs == ak98_spi_gpiocs)
	//	gpio_free(hw->pdata->pin_cs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}


#ifdef CONFIG_PM

static int ak98_spi_suspend(struct device *dev)
{
	struct ak98_spi *hw = platform_get_drvdata(to_platform_device(dev));

	if (hw->pdata && hw->pdata->gpio_setup)
		hw->pdata->gpio_setup(hw->pdata, 0);

	clk_disable(hw->clk);
	return 0;
}

static int ak98_spi_resume(struct device *dev)
{
	struct ak98_spi *hw = platform_get_drvdata(to_platform_device(dev));

	ak98_spi_initialsetup(hw);
	return 0;
}

static struct dev_pm_ops ak98_spi_pmops = {
	.suspend	= ak98_spi_suspend,
	.resume		= ak98_spi_resume,
};

#define AK98_SPI_PMOPS &ak98_spi_pmops
#else
#define AK98_SPI_PMOPS NULL
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:ak98-spi");
static struct platform_driver ak98_spi_driver = {
	.remove		= __exit_p(ak98_spi_remove),
	.driver		= {
		.name	= "ak98-spi",
		.owner	= THIS_MODULE,
		.pm	= AK98_SPI_PMOPS,
	},
};

static int __init ak98_spi_init(void)
{
		PDEBUG("AK98 SPI Driver, (c) 2011 ANYKA\n");
        return platform_driver_probe(&ak98_spi_driver, ak98_spi_probe);
}

static void __exit ak98_spi_exit(void)
{
        platform_driver_unregister(&ak98_spi_driver);
}

module_init(ak98_spi_init);
module_exit(ak98_spi_exit);

MODULE_DESCRIPTION("AK98 SPI Driver");
MODULE_AUTHOR("ANYKA");
MODULE_LICENSE("GPL");
