/*
 *  linux/drivers/mmc/host/ak88_mci.h - AK88 MMC/SD/SDIO driver
 *
 *  Copyright (C) 2010 Anyka, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define AK88MCICLOCK		0x004
#define MMC_CLK_DIVL(x)		((x) & 0xff)
#define MMC_CLK_DIVH(x)		(((x) & 0xff) << 8)
#define MCI_CLK_ENABLE		(1 << 16)
#define MCI_CLK_PWRSAVE		(1 << 17)
#define MCI_FAIL_TRIGGER	(1 << 19)
#define MCI_ENABLE		(1 << 20)

#define AK88MCIARGUMENT		0x008
#define AK88MCICOMMAND		0x00c
#define MCI_CPSM_ENABLE		(1 << 0)
#define MCI_CPSM_CMD(x)		(((x) & 0x3f) << 1)
#define MCI_CPSM_RESPONSE	(1 << 7)
#define MCI_CPSM_LONGRSP	(1 << 8)
#define MCI_CPSM_PENDING	(1 << 9)
#define MCI_CPSM_RSPCRC_NOCHK	(1 << 10)
#define MCI_CPSM_WITHDATA	(1 << 11)

#define AK88MCIRESPCMD		0x010
#define AK88MCIRESPONSE0	0x014
#define AK88MCIRESPONSE1	0x018
#define AK88MCIRESPONSE2	0x01c
#define AK88MCIRESPONSE3	0x020
#define AK88MCIDATATIMER	0x024
#define AK88MCIDATALENGTH	0x028
#define AK88MCIDATACTRL		0x02c
#define MCI_DPSM_ENABLE		(1 << 0)
#define MCI_DPSM_DIRECTION	(1 << 1)
#define MCI_DPSM_STREAM		(1 << 2)
#define MCI_DPSM_BUSMODE(x)	(((x) & 0x3) << 3)
#define MCI_DPSM_BLOCKSIZE(x)	(((x) & 0xfff) << 16)

#define AK88MCIDATACNT		0x030
#define AK88MCISTATUS		0x034
#define MCI_RESPCRCFAIL		(1 << 0)
#define MCI_DATACRCFAIL		(1 << 1)
#define MCI_RESPTIMEOUT		(1 << 2)
#define MCI_DATATIMEOUT		(1 << 3)
#define MCI_RESPEND		(1 << 4)
#define MCI_CMDSENT		(1 << 5)
#define MCI_DATAEND		(1 << 6)
#define MCI_DATABLOCKEND	(1 << 7)
#define MCI_STARTBIT_ERR	(1 << 8)
#define MCI_CMDACTIVE		(1 << 9)
#define MCI_TXACTIVE		(1 << 10)
#define MCI_RXACTIVE		(1 << 11)
#define MCI_FIFOFULL		(1 << 12)
#define MCI_FIFOEMPTY		(1 << 13)
#define MCI_FIFOHALFFULL	(1 << 14)
#define MCI_FIFOHALFEMPTY	(1 << 15)
#define MCI_DATATRANS_FINISH	(1 << 16)
#define MCI_SDIOINT		(1 << 17)

#define AK88MCIMASK		0x038
#define MCI_RESPCRCFAILMASK	(1 << 0)
#define MCI_DATACRCFAILMASK	(1 << 1)
#define MCI_RESPTIMEOUTMASK	(1 << 2)
#define MCI_DATATIMEOUTMASK	(1 << 3)
#define MCI_RESPENDMASK		(1 << 4)
#define MCI_CMDSENTMASK		(1 << 5)
#define MCI_DATAENDMASK		(1 << 6)
#define MCI_DATABLOCKENDMASK	(1 << 7)
#define MCI_STARTBIT_ERRMASK	(1 << 8)
#define MCI_CMDACTIVEMASK	(1 << 9)
#define MCI_TXACTIVEMASK	(1 << 10)
#define MCI_RXACTIVEMASK	(1 << 11)
#define MCI_FIFOFULLMASK	(1 << 12)
#define MCI_FIFOEMPTYMASK	(1 << 13)
#define MCI_FIFOHALFFULLMASK	(1 << 14)
#define MCI_FIFOHALFEMPTYMASK	(1 << 15)
#define MCI_DATATRANS_FINISHMASK	(1 << 16)
#define MCI_SDIOINTMASK		(1 << 17)

#define AK88MCIDMACTRL		0x03c
#define MCI_DMA_BUFEN		(1 << 0)
#define MCI_DMA_ADDR(x)		(((x) & 0x7fff) << 1)
#define MCI_DMA_EN		(1 << 16)
#define MCI_DMA_SIZE(x)		(((x) & 0x7fff) << 17)

#define AK88MCIFIFO		0x040

#define MCI_CMDIRQMASKS \
	(MCI_CMDSENTMASK|MCI_RESPENDMASK|		\
	 MCI_RESPCRCFAILMASK|MCI_RESPTIMEOUTMASK)

#define MCI_DATAIRQMASKS \
	(MCI_DATAEND|MCI_DATABLOCKENDMASK|		\
	 MCI_DATACRCFAILMASK|MCI_DATATIMEOUTMASK|	\
	 MCI_STARTBIT_ERRMASK)

/*
 * The size of the FIFO in bytes.
 */
#define MCI_FIFOSIZE	4
#define MCI_FIFOHALFSIZE (MCI_FIFOSIZE / 2)

#define NR_SG		16

#define L2BASE		0x2002c000
#define L2FIFO_DMACONF	0x80
#define L2FIFO_CONF1	0x88
#define L2FIFO_ASSIGN1	0x90
#define L2FIFO_INTEN	0x9c

#define L2FIFOBASE	0x48000000
#define L2ADDR(n)	(L2FIFOBASE + 512 * (n))
#define MCI_L2FIFO_NUM	2	/* #6 l2fifo */
#define MCI_L2FIFO_SIZE	512

#define L2DMA_MAX_SIZE	(64*255)

struct clk;

struct ak88_mci_host {
	void __iomem		*base;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct clk		*clk;
	int			gpio_cd;
	int			gpio_wp;
	int			irq_mci;
	int			irq_cd;
	int			irq_cd_type;

	unsigned int		data_xfered;

	spinlock_t		lock;

#if 0
	unsigned int		mclk;
	unsigned int		cclk;
	u32			pwr;
#else
	unsigned char		bus_mode;
	unsigned char		bus_width;
	unsigned long		bus_clkrate;
	unsigned long		asic_clkrate;
	unsigned char		power_mode;
#endif
	struct ak88_mci_platform_data *plat;

#if 0
	u8			hw_designer;
	u8			hw_revision:4;
#endif

#if 0
	struct timer_list	timer;
	unsigned int		oldstat;
#endif

	unsigned int		sg_len;

	/* pio stuff */
	struct scatterlist	*sg_ptr;
	unsigned int		sg_off;
	unsigned int		size;

#if 0
	struct regulator	*vcc;
#endif

	struct timer_list	detect_timer;

	void __iomem		*l2base;
	void __iomem		*l2fifo;
};

static inline void ak88_mci_init_sg(struct ak88_mci_host *host, struct mmc_data *data)
{
	/*
	 * Ideally, we want the higher levels to pass us a scatter list.
	 */
	host->sg_len = data->sg_len;
	host->sg_ptr = data->sg;
	host->sg_off = 0;
}

static inline int ak88_mci_next_sg(struct ak88_mci_host *host)
{
	host->sg_ptr++;
	host->sg_off = 0;
	return --host->sg_len;
}

static inline char *ak88_mci_kmap_atomic(struct ak88_mci_host *host, unsigned long *flags)
{
	struct scatterlist *sg = host->sg_ptr;

	local_irq_save(*flags);
	return kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
}

static inline void ak88_mci_kunmap_atomic(struct ak88_mci_host *host, void *buffer, unsigned long *flags)
{
	kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
	local_irq_restore(*flags);
}
