
#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <mach/map.h>
#include <mach/ak880x_freq.h>

//************************** clock **************************//

unsigned long ak880x_pll1freq_get_default(void)
{
	//ref to 0x08000004

	int N_def = 0b00001;	// 1
	int M_def = 0b010001;	//17
	unsigned long M = (45 + M_def) * 1000 * 1000;
	int N = N_def + 1;

	unsigned long freq = (unsigned long)((4 * M) / N);	//(4*62/2)M=124M

	//printk("ak880x_pll1freq_get_default()=%d\n",(u32)freq);

	return freq;
}

unsigned long ak880x_cpufreq_get_default(void)
{
	//ref to 0x08000004

	//bit15,   default :0:   cpu clk = asic clk
	//bit[8:6],default :000: asic clk =pll1clk

	//return ak880x_pll1freq_get_default();
	return 124000000;	//124MHZ
}

unsigned long ak880x_pll1freq_get(void)
{
	//return  ak880x_pll1freq_get_default();
	return 124000000;	//124MHZ
}

unsigned long ak880x_cpufreq_get(void)
{
	//return  ak880x_cpufreq_get_default();
	return 124000000;	//124MHZ
}

unsigned long ak880x_asicfreq_get(void)
{
	//return ak880x_pll1freq_get_default();

	return 124000000;	//124MHZ
}

unsigned long ak780x_pll1freq_get_default(void)
{
	//ref to 0x08000004

	int N_def = 0b00001;	//1
	int M_def = 0b00000;	//0
	unsigned long M = (62 + M_def) * 1000 * 1000;
	int N = N_def + 1;

	unsigned long freq = (unsigned long)((4 * M) / N);	//(4*62/2)M=124M

	return freq;
}

unsigned long ak780x_cpufreq_get_default(void)
{
	//ref to 0x08000004
	//bit15,   default :0:   cpu clk = asic clk
	//bit[8:6],default :000: asic clk =pll1clk/2

	return ak780x_pll1freq_get_default() / 2;
}

unsigned long ak780x_cpufreq_get(void)
{
	return ak780x_cpufreq_get_default();
}

void ak880x_sdelay(int s_time)
{
	unsigned long i, count;
	count = ak880x_cpufreq_get();	//for seconds
	count = count / 5;	//each loop consume 5 clock period (SUB,CMP,BNE)
	for (; s_time > 0; s_time--)
		for (i = count; i > 0; i--) ;	//SUB,CMP,BNE
}

void ak880x_msdelay(int ms_time)
{
	unsigned long i, count;
	count = ak880x_cpufreq_get() / 1000;	//for ms
	count = count / 5;	//each loop consume 5 clock period (SUB,CMP,BNE)
	for (; ms_time > 0; ms_time--)
		for (i = count; i > 0; i--) ;	//SUB,CMP,BNE
}

void ak880x_usdelay(int us_time)
{
	unsigned long i, count;
	count = 124 / 5;	//for us

	for (; us_time > 0; us_time--)
		for (i = count; i > 0; i--) ;	//SUB,CMP,BNE
}
