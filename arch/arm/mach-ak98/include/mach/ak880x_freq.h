#ifndef __ARCH_ARM_MACH_AK88_LIB_BASE_H__
#define __ARCH_ARM_MACH_AK88_LIB_BASE_H__

/*
 freq,clock,gpio,sharepin
*/

#include <asm/io.h>
#include <mach/map.h>

void ak880x_sdelay(int s_time);
void ak880x_msdelay(int ms_time);
void ak880x_usdelay(int us_time);

unsigned long ak880x_pll1freq_get(void);
unsigned long ak880x_cpufreq_get(void);
unsigned long ak880x_asicfreq_get(void);

unsigned long ak780x_cpufreq_get(void);

#endif
