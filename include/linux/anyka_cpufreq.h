#ifndef __ANYKA_CPUFREQ_H
#define __ANYKA_CPUFREQ_H

typedef enum OPERATION_MODE {	
	FREQ_MODE_MIN = 0,
	LOW_MODE_CLOCK_MIN = 0,
	LOW_MODE_CLOCK_0 = LOW_MODE_CLOCK_MIN,
	LOW_MODE_CLOCK_1,
	LOW_MODE_CLOCK_2,
	LOW_MODE_CLOCK_3,
	LOW_MODE_CLOCK_4,
	LOW_MODE_CLOCK_5,
	LOW_MODE_CLOCK_6,
	LOW_MODE_CLOCK_7,
	LOW_MODE_CLOCK_MAX = LOW_MODE_CLOCK_7,

	NORMAL_MODE_CLOCK_MIN,
	NORMAL_MODE_CLOCK_0 = NORMAL_MODE_CLOCK_MIN,
	NORMAL_MODE_CLOCK_1,
	NORMAL_MODE_CLOCK_2,
	NORMAL_MODE_CLOCK_3,
	NORMAL_MODE_CLOCK_4,
	NORMAL_MODE_CLOCK_5,
	NORMAL_MODE_CLOCK_6,
	NORMAL_MODE_CLOCK_7,
	NORMAL_MODE_CLOCK_MAX = NORMAL_MODE_CLOCK_7,

	VIDEO_MODE_CLOCK_MIN,
	VIDEO_MODE_CLOCK_0 = VIDEO_MODE_CLOCK_MIN,
	VIDEO_MODE_CLOCK_1,
	VIDEO_MODE_CLOCK_2,
	VIDEO_MODE_CLOCK_3,
	VIDEO_MODE_CLOCK_4,
	VIDEO_MODE_CLOCK_5,
	VIDEO_MODE_CLOCK_6,
	VIDEO_MODE_CLOCK_7,
	VIDEO_MODE_CLOCK_MAX = VIDEO_MODE_CLOCK_7,
	FREQ_MODE_MAX = VIDEO_MODE_CLOCK_MAX,
}T_OPERATION_MODE; 

struct cpufreq_mode_clkkdiv {
    T_OPERATION_MODE mode_name;
    unsigned int pll_sel;
    unsigned int clk168_div;
	unsigned int cpu_div;
	unsigned int mem_div;
	unsigned int asic_div;
	unsigned int low_clock;
	unsigned int is_3x;
};

T_OPERATION_MODE get_current_mode(void);
int request_cpufreq_enter(T_OPERATION_MODE mode);
unsigned int get_pll_sel(T_OPERATION_MODE state);
int is_low_clock_mode(void);
int previous_mode_is_low_mode(void);
int current_mode_is_low_mode(void);



#endif /*  __ANYKA_CPUFREQ_H */

