#ifndef __AK98_TS_H_
#define __AK98_TS_H_ __FILE__

struct ak98_ts_mach_info {
	unsigned int irq;	/* use a macro convert gpio pin */
	unsigned int irqpin;
	unsigned int sample_rate;
	unsigned int wait_time;
};

#endif				/* __AK98_TS_H_ */


