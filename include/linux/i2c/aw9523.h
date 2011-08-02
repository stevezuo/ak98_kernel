#ifndef __AW9523_H_
#define __AW9523_H_

#include <mach/gpio.h>
#include <linux/mutex.h>
#include <linux/i2c.h>


/*
 * AW9523 registers
 */
#define AW9523_REG_INPUT_PORT0	0x00
#define AW9523_REG_INPUT_PORT1	0x01
#define AW9523_REG_OUTPUT_PORT0	0x02
#define AW9523_REG_OUTPUT_PORT1	0x03
#define AW9523_REG_CFG_PORT0	0x04
#define AW9523_REG_CFG_PORT1	0x05
#define AW9523_REG_INTN_PORT0	0x06
#define AW9523_REG_INTN_PORT1	0x07
#define AW9523_REG_GPOMD		0x11
#define AW9523_REG_RESET		0x7F

/*
 * operation flags
 */
/*  mode */
#define AW9523_MOD_OPENDRAIN	0
#define AW9523_MOD_PUSHPULL	1



/*
* pin definitions 
*/
enum {
	PMIN=0,
	P00 = PMIN,
	P01,P02,P03,P04,P05,P06,P07,
	P10,P11,P12,P13,P14,P15,P16,P17,
	PMAX = P17,
};


/* platform data for the AW9523 16-bit I/O expander driver */

struct aw9523_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;
	unsigned	irq_base;

	unsigned int parent_irq;
	struct gpio_info *gpios;
	int npins;

	uint8_t		p0xmod;
	/* initial polarity inversion setting */
	uint16_t	invert;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	char		**names;
};

struct aw9523_chip {
	uint8_t reg_output0;	/* AW9523 Port 0 output value */
	uint8_t reg_output1;	/* AW9523 Port 1 output value */
	uint8_t reg_direction0;	/* AW9523 Port 0 direction: */
	uint8_t reg_direction1;	/* AW9523 Port 1 direction: */
	uint8_t reg_setmod;		/* AW9523 Port 0 mode: */
	uint8_t reg_int0;
	uint8_t reg_int1;
	uint8_t reg_input0;
	uint8_t reg_input1;
	
	struct i2c_client *client;

	uint8_t irq_mask0;
	uint8_t irq_mask1;

	struct mutex aw9523_lock;
	struct aw9523_platform_data *pdata;
	
};


int aw9523_gpio_to_irq(unsigned int pino);
int aw9523_irq_to_gpio(unsigned int irq);

/*function: set gpio direction 
 * param pinno: pin number
 * param direction:
 *	1: input mode,  0: output mode
 */
int aw9523_gpio_dircfg(unsigned int pino, unsigned int direction);

int aw9523_gpio_intcfg(unsigned int pino, unsigned int enable);

/* function: get pin state
 * param pino: pin number
 */
int aw9523_gpio_getpin(unsigned int pino);

/* function : setup output value
 * param pino: pin number
 * param level: 
 *	1: output high level, 0: ouput low level
 */
int aw9523_gpio_setpin(unsigned int pino, unsigned int level);

#endif  /* __AW9523_H */

