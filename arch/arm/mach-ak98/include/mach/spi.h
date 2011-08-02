/*
 * include/asm-arm/arch-ak98/spi.h
 */

#ifndef __SPI_H__
#define __SPI_H__

struct ak98_spi_info {
	unsigned long pin_cs;
	unsigned long board_size;
	unsigned short bus_num;
	unsigned short num_cs;
	unsigned short mode_bits;
	char clk_name[20];
	int mode;
	struct spi_board_info *board_info;

	void (*gpio_setup)(struct ak98_spi_info *spi, int enable);
	void (*set_cs) (struct ak98_spi_info, int cs, int pol);
};

#define AK98_SPICON		(0x00)
#define AK98_SPICON_CLKDIV	(0x7F<<8)
#define AK98_SPICON_EN	(1<<6)
#define AK98_SPICON_CS	(1<<5)
#define AK98_SPICON_MS	(1<<4)
#define AK98_SPICON_CPHA	(1<<3)
#define AK98_SPICON_CPOL	(1<<2)
#define AK98_SPICON_ARRM	(1<<1)
#define AK98_SPICON_TGDM	(1<<0)

#define AK98_SPISTA		(0x04)
#define AK98_SPISTA_TIMEOUT	(1<<10)
#define AK98_SPISTA_MPROC	(1<<9)
#define AK98_SPISTA_TRANSF	(1<<8)
#define AK98_SPISTA_RXOVER	(1<<7)
#define AK98_SPISTA_RXHFULL	(1<<6)
#define AK98_SPISTA_RXFULL	(1<<5)
#define AK98_SPISTA_RXEMP	(1<<4)
#define AK98_SPISTA_TXUNDER	(1<<3)
#define AK98_SPISTA_TXHEMP	(1<<2)
#define AK98_SPISTA_TXFULL	(1<<1)
#define AK98_SPISTA_TXEMP	(1<<0)

#define AK98_SPIINT		(0x08)
#define AK98_SPIINT_TIMEOUT	(1<<10)
#define AK98_SPIINT_MPROC	(1<<9)
#define AK98_SPIINT_TRANSF	(1<<8)
#define AK98_SPIINT_RXOVER	(1<<7)
#define AK98_SPIINT_RXHFULL	(1<<6)
#define AK98_SPIINT_RXFULL	(1<<5)
#define AK98_SPIINT_RXEMP	(1<<4)
#define AK98_SPIINT_TXUNDER	(1<<3)
#define AK98_SPIINT_TXHEMP	(1<<2)
#define AK98_SPIINT_TXFULL	(1<<1)
#define AK98_SPIINT_TXEMP	(1<<0)

#define AK98_SPICNT		(0x0C)

#define AK98_SPIEXTX		(0x10)

#define AK98_SPIEXRX		(0x14)

#define AK98_SPIOUT		(0x18)

#define AK98_SPIIN		(0x1C)

#endif
