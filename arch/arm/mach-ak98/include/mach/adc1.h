#ifndef _AK98_ADC1_H_
#define _AK98_ADC1_H_

/*
	ADC1 common module
	2011-04-12

 */
/***** Definitions of macros **********************************/
#define ADC_MAIN_CLK		12 	/* 12MHz */
#define ADC1_DIV		 	11
#define MHZ					1000000
#define THRESHOLD			100

#define PENDOWN				1
#define PENUP				0
#define PENDOWNLEN			0x1f

#define POWER_ON			1
#define POWER_OFF			0
/***********************************************************/
#undef REG32
#define REG32(_reg_)  (*(volatile unsigned long *)(_reg_))

void ak98_init_ADC1(unsigned int SampleRate, unsigned int WaitTime);
void ak98_power_ADC1(int op);
void ak98_power_ts(int op);
void ak98_enable_bat_mon(void);
void ak98_disable_bat_mon(void);
long ak98_read_voltage(void);

#endif