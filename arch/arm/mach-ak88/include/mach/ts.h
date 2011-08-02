#ifndef __AK88_TS_H_
#define __AK88_TS_H_ __FILE__

struct ak880x_ts_mach_info {
	unsigned int irq;	/* use a macro convert gpio pin */
	unsigned int irqpin;
	unsigned int sample_rate;
	unsigned int wait_time;
};

#endif				/* __AK88_TS_H_ */
