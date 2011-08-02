#ifndef __H_AK88_IO_CONTROL__
#define __H_AK88_IO_CONTROL__

/* IO control register 1 */
#define DS_SDIO_MCMD		0
#define DS_SDIO_MCK		1

#define IE_URD1			2
#define IE_UTD1			3
#define IE_CTS1			4
#define IE_RTS1			5

#define IE_URD2			6
#define IE_UTD2			7
#define IE_CTS2			8
#define IE_RTS2			9

#define IE_URD3			10
#define IE_UTD3			11
#define IE_CTS3			12
#define IE_RTS3			13

#define PE_URD1			14
#define PE_UTD1			15
#define PE_CTS1			16
#define PE_RTS1			17

#define PE_URD2			18
#define PE_UTD2			19
#define PE_CTS2			20
#define PE_RTS2			21

#define PE_URD3			22
#define PE_UTD3			23
#define PE_CTS3			24
#define PE_RTS3			25

#define IE_SDIO_MCK		26
#define IE_SDIO_MCMD		27

#define DS_GPIO_29		0
#define DS_GPIO_28		1

#define IE_GPIO_17		2
#define IE_GPIO_16		3
#define IE_GPIO_19		4
#define IE_GPIO_18		5
#define IE_GPIO_21		6
#define IE_GPIO_20		7
#define IE_GPIO_23		8
#define IE_GPIO_22		9
#define IE_GPIO_25		10
#define IE_GPIO_24		11
#define IE_GPIO_27		12
#define IE_GPIO_26		13

#define PE_GPIO_17		14
#define PE_GPIO_16		15
#define PE_GPIO_19		16
#define PE_GPIO_18		17
#define PE_GPIO_21		18
#define PE_GPIO_20		19
#define PE_GPIO_23		20
#define PE_GPIO_22		21
#define PE_GPIO_25		22
#define PE_GPIO_24		23
#define PE_GPIO_27		24
#define PE_GPIO_26		25

#define IE_GPIO_29		26
#define IE_GPIO_28		27

/* IO control register 2 */

#define IE_DGPIO_7		(32 + 4)
#define PE_DGPIO_7		(32 + 8)
#define DS_DGPIO_7		(32 + 12)

void ak880x_ioctl(unsigned int offset, const unsigned int to);

#endif
