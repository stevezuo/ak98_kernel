#ifndef _AKUIO_DRIVER_H
#define _AKUIO_DRIVER_H

struct akuio_sysreg_write_t
{
	unsigned int paddr;
	unsigned int val;
	unsigned int mask;
};

/* write system register */
#define AKUIO_SYSREG_WRITE       _IOW('U', 100, struct akuio_sysreg_write_t)

/* wait for a interrupt occur */
#define AKUIO_WAIT_IRQ           _IOR('U', 101, int)

#endif
