/*
 * include/asm-arm/arch-ak880x/spi.h
 */

#ifndef __SPI_H__
#define __SPI_H__

struct ak880x_spi_info {
	unsigned long pin_cs;
	unsigned long board_size;
	struct spi_board_info *board_info;

	void (*set_cs) (struct ak880x_spi_info, int cs, int pol);
};

#define AK88_SPICON		(0x00)
#define AK88_SPICON_CLKDIV	(0x7F<<8)
#define AK88_SPICON_EN	(1<<6)
#define AK88_SPICON_CS	(1<<5)
#define AK88_SPICON_MS	(1<<4)
#define AK88_SPICON_CPHA	(1<<3)
#define AK88_SPICON_CPOL	(1<<2)
#define AK88_SPICON_ARRM	(1<<1)
#define AK88_SPICON_TGDM	(1<<0)

#define AK88_SPISTA		(0x04)
#define AK88_SPISTA_TIMEOUT	(1<<10)
#define AK88_SPISTA_MPROC	(1<<9)
#define AK88_SPISTA_TRANSF	(1<<8)
#define AK88_SPISTA_RXOVER	(1<<7)
#define AK88_SPISTA_RXHFULL	(1<<6)
#define AK88_SPISTA_RXFULL	(1<<5)
#define AK88_SPISTA_RXEMP	(1<<4)
#define AK88_SPISTA_TXUNDER	(1<<3)
#define AK88_SPISTA_TXHEMP	(1<<2)
#define AK88_SPISTA_TXFULL	(1<<1)
#define AK88_SPISTA_TXEMP	(1<<0)

#define AK88_SPIINT		(0x08)
#define AK88_SPIINT_TIMEOUT	(1<<10)
#define AK88_SPIINT_MPROC	(1<<9)
#define AK88_SPIINT_TRANSF	(1<<8)
#define AK88_SPIINT_RXOVER	(1<<7)
#define AK88_SPIINT_RXHFULL	(1<<6)
#define AK88_SPIINT_RXFULL	(1<<5)
#define AK88_SPIINT_RXEMP	(1<<4)
#define AK88_SPIINT_TXUNDER	(1<<3)
#define AK88_SPIINT_TXHEMP	(1<<2)
#define AK88_SPIINT_TXFULL	(1<<1)
#define AK88_SPIINT_TXEMP	(1<<0)

#define AK88_SPICNT		(0x0C)

#define AK88_SPIEXTX		(0x10)

#define AK88_SPIEXRX		(0x14)

#define AK88_SPIOUT		(0x18)

#define AK88_SPIIN		(0x1C)

#endif
