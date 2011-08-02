/* arch/arm/mach-ak98/cpufreq.c
 *
 * AK98 CPUfreq Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/anyka_cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/syscalls.h>

#include <mach/cpufreq.h>
#include <mach/clock.h>

#define SIZE_1K 0x00000400      /* 1K */
extern atomic_t suspend_flags;

static struct cpufreq_freqs freqs;
static T_OPERATION_MODE current_mode;
static int previous_mode_flag;
static int cpufreq_in_ddr2 = 1;
static unsigned int clkdiv = 1000000;

#if 1
struct cpufreq_mode_clkdiv cpufreq_divs[] = {
	//mode_name  pll_sel  clk168_div  cpu_div  mem_div  asic_div  low_clock is_3x  (cpu,mem,asic[MHz])
	{LOW_MODE_CLOCK_0, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_1, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_2, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_3, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_4, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_5, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_6, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */
	{LOW_MODE_CLOCK_7, 	0x37,	0,	0,	2,	4,	1,	0}, /* (100,200,100) */ 
	
	{NORMAL_MODE_CLOCK_0, 0x37,	 0,	0,	2,	4,	0,	0}, /* (200,200,100) */
	{NORMAL_MODE_CLOCK_1, 0x37,	 0,	0,	2,	4,	0,	0}, /* (200,200,100) */
	{NORMAL_MODE_CLOCK_2, 0x37,  0,	0,	2,	4, 	0,	0}, /* (200,200,100) */
	{NORMAL_MODE_CLOCK_3, 0x37,  0,	0,	2,	4,	0,	0}, /* (200,200,100) */
	{NORMAL_MODE_CLOCK_4, 0x37,  0,	0,	2,	4, 	0,	0}, /* (200,200,100) */
	{NORMAL_MODE_CLOCK_5, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{NORMAL_MODE_CLOCK_6, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{NORMAL_MODE_CLOCK_7, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */    

	{VIDEO_MODE_CLOCK_0, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_1, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_2, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_3, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_4, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_5, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_6, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
	{VIDEO_MODE_CLOCK_7, 0x37,  0,	1,	2,	4, 	0,	0}, /* (400,200,100) */
};
#else
struct cpufreq_mode_clkdiv cpufreq_divs[] = {
    //mode_name  pll_sel  clk168_div  cpu_div  mem_div  asic_div  low_clock is_3x  (cpu,mem,asic[MHz])
    {LOW_MODE_CLOCK_0,  0x14,   0,  0,  2,  4,  1,  0}, /* (65,130,65) */
    {LOW_MODE_CLOCK_1,  0x14,   0,  0,  2,  4,  1,  0}, /* (65,130,65) */
    {LOW_MODE_CLOCK_2,  0x14,   0,  0,  2,  4,  0,  0}, /* (130,130,65) */
    {LOW_MODE_CLOCK_3,  0x14,   0,	0,  2,  4,  0,  0}, /* (130,130,65) */
    {LOW_MODE_CLOCK_4,  0x14,   0,  0,  2,  2,  0,  0}, /* (130,130,130) */
    {LOW_MODE_CLOCK_5,  0x14,   0,  0,  2,  2,  0,  0}, /* (130,130,130) */
    {LOW_MODE_CLOCK_6,  0x14,   0,  1,  2,  2,  0,  0}, /* (260,130,130) */
    {LOW_MODE_CLOCK_7,  0x14,   0,  1,  2,  2,  0,  0}, /* (260,130,130) */

    {NORMAL_MODE_CLOCK_0, 0x37,  0, 0,  2,  4,  0,  0}, /* (200,200,100) */
    {NORMAL_MODE_CLOCK_1, 0x37,  0, 0,  2,  4,  0,  0}, /* (200,200,100) */
    {NORMAL_MODE_CLOCK_2, 0x37,  0, 0,  2,  4,  0,  0}, /* (200,200,100) */
    {NORMAL_MODE_CLOCK_3, 0x37,  0, 0,  2,  4,  0,  0}, /* (200,200,100) */
    {NORMAL_MODE_CLOCK_4, 0x37,  0, 0,  2,  4,  0,  0}, /* (200,200,100) */
    {NORMAL_MODE_CLOCK_5, 0x37,  0, 1,  2,  4,  0,  0}, /* (400,200,100) */
    {NORMAL_MODE_CLOCK_6, 0x37,  0, 1,  2,  4,  0,  0}, /* (400,200,100) */
    {NORMAL_MODE_CLOCK_7, 0x37,  0, 1,  2,  4,  0,  0}, /* (400,200,100) */

    {VIDEO_MODE_CLOCK_0, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_1, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_2, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_3, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_4, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_5, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_6, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
    {VIDEO_MODE_CLOCK_7, 0x36,  0,  1,  0,  0,  0,  1}, /* (396,132,132) */
};
#endif

static T_OPERATION_MODE prev_suspend_mode;
#define SUSPEND_NORMAL_MODE		NORMAL_MODE_CLOCK_2

#define CPU_FREQ_CHANGING(ratio) do {\
	/* change cpu frequence, and first close 2x mode then open 2x mode */\
    REG32(CLOCK_DIV_REG) = ratio & (~(1 << 15));\
    udelay(10);\
	REG32(CLOCK_DIV_REG) |= (ratio & (1 << 15));\
} while(0)

static int get_cpufreq_mode_clkdiv(T_OPERATION_MODE mode, struct cpufreq_mode_clkdiv *clkdiv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cpufreq_divs); i++) {
		if (cpufreq_divs[i].mode_name == mode) {
			*clkdiv = cpufreq_divs[i];
			return 0;
		}
	}
	return -1;
}

static unsigned int calcue_power(unsigned int num)
{
    unsigned int i;

    if(num < 0)
        return -1;
    if((num == 2)||(num == 0))
        return 0;

    for(i = 0; num % 2 == 0; i++){
        num /= 2;
    }
    if(num > 2)
        return -1;

    return i;
}

static void config_clock_parameter(struct cpufreq_mode_clkdiv *cpufreq,
	unsigned long *ratio)
{
	unsigned long clk_ratio = *ratio;
	unsigned int memdiv, asicdiv;

	memdiv = calcue_power(cpufreq->mem_div);
	if( memdiv < 0)
		printk("Error, calcue mem_div.");
	asicdiv = calcue_power(cpufreq->asic_div);
	if( asicdiv < 0)
		printk("Error, calcue asic_div.");
	
	clk_ratio &= ~((1 << 28)|(1 << 22)|(0xF << 17)|(1 << 15)|(0x7 << 9)|(0x7 << 6)|(0x3F << 0));

	clk_ratio |= (cpufreq->is_3x << 28);		//is 3x ?
	clk_ratio |= (cpufreq->clk168_div << 17);	//clk168 div
	clk_ratio |= (cpufreq->cpu_div << 15);		//cpu clk = mem clk or cpu clk = asic clk
	clk_ratio |= (cpufreq->low_clock << 22);	//cpu clk = asic clk
	clk_ratio |= (memdiv << 9)|(asicdiv << 6);	//mem div and asic div
	clk_ratio |= (cpufreq->pll_sel << 0);		//pll_sel
	
	*ratio = clk_ratio;
}

/*
 *  *function: enter L2 modify register parameters of clock for change sys clcok
 *   */
void L2_LINK(freqchange) L2FUNC_NAME(freqchange)(unsigned long param1,
	unsigned long param2,unsigned long param3,unsigned long param4)
{
	DISABLE_CACHE_MMU();
	DDR2_ENTER_POWERDOWN();
	// after send enter self - refresh, delay stable clock at least more than 1 tck
	PM_DELAY(0x200);

	//disable ram clock
	REG32(PHY_CLOCK_CTRL_REG) |= (1<<10);

	// other mode change to normal mode
	//cpu clock from other mode to normal
	REG32(PHY_CLOCK_DIV_REG) &= ~((1 << 28)|(1 << 22));
	PM_DELAY(0x100);

	//set clock div and check pll[12]
	REG32(PHY_CLOCK_DIV_REG) = param1 &(~(1<<15)); // close cpu2x
	while (REG32(PHY_CLOCK_DIV_REG) & PLL_CHANGE_ENA);

	REG32(PHY_CLOCK_DIV_REG) |= (param1&(1<<15)); // open cpu2x

	//enable ram clock
	REG32(PHY_CLOCK_CTRL_REG) &= ~(1<<10);
	// new clock stable at least more than 1tck,here is ignore because follow has few instruction.

	// softreset ddr2 memory controller
	REG32(0x0800000c) |= (0x1 << 26);
	REG32(0x0800000c) &= ~(0x1 << 26);

	// re-init ram controller
	REG32(0x2000e05c) = 0x00000200; // bypass DCC
	REG32(0x2000e078) = 0x40020100; // initial sstl = 00(12ma), tsel = 10(150ohm)
	REG32(0x2000e000) = 0x00004e90; // 32 bit bus width

    // exit precharge power-down mode before delay at least 1 tck
    PM_DELAY(10);
    DDR2_EXIT_POWERDOWN();

    // load mr, reset dll and delay for 200 tck, and set odt high
    REG32(PHY_RAM_CPU_CMD) = 0x02800532;
    // send nop, delay for 200 tck for dll reset,
    REG32(PHY_RAM_CPU_CMD) = 0x02f00000;
    PM_DELAY(0x100);

    // load mr, clean reset dll and remain odt low in ddr2 memory
    REG32(PHY_RAM_CPU_CMD) = 0x0a800432;

    // open auto-referesh
    // default as mclk = 120mhz for calc tck=8.3ns, trefi=7.7us
    REG32(0x2000e00c) = (0x39f<<1)|0x1;

    //enable dll and wate for dll stable in ram controller
    REG32(0x2000e020) = 0x00000003;
    while (!(REG32(0x2000e020) & (1 << 2)));

    //calibration sart and wait for finish
    REG32(0x2000e024) = ((param2>>0x7)<<10)|0x1;
    while (!(REG32(0x2000e024) & (1 << 1)));

    ENABLE_CACHE_MMU();
}

/*
 *function: change pll clock, include mem clock,asic clock and cpu clock.
 */
static void cpufreq_change_clocks(struct cpufreq_mode_clkdiv *cpufreq)
{
    unsigned long ratio;
    void *addr;
    unsigned long phy_addr;
	
    addr = kzalloc(512, GFP_KERNEL | GFP_ATOMIC);
    if (addr == NULL)
        return ;
    phy_addr = virt_to_phys(addr);
	
    ratio = REG32(CLOCK_DIV_REG);
    config_clock_parameter(cpufreq, &ratio);
    ratio |= (PLL_CHANGE_ENA);
	
    SPECIFIC_L2BUF_EXEC(freqchange, ratio, phy_addr,0,0);
	
    kfree(addr);
}

/*
 *function: according to needed new clocks and determine branch of cpufreq
 * @cpufreq: structure include mode name and clock div
 * note: the mode enter L2 cpufreq
 */
static int l2_cpu_freq_change(struct cpufreq_mode_clkdiv *cpufreq)
{
	int error;
	
	error = usermodehelper_disable();
	if (error)
		goto Finish;

	// freeze process and kernel task.
	error = cpufreq_freeze_processes();
	if (error)
		goto freeze;
	
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	cpufreq_change_clocks(cpufreq);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	
freeze:
	thaw_processes();
Finish:
	usermodehelper_enable();
	return error;
}


/*
 *function: according to needed new clocks and determine branch of cpufreq
 * @cpufreq: structure include mode name and clock div
 * note: the mode cpufreq in ddr2
 */
static void ddr2_cpu_freq_change(struct cpufreq_mode_clkdiv *cpufreq)
{
    unsigned long ratio,tmp;
	
    cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	// check if cpu clock from other mode to normal and cutover to normal
	if((freqs.old_cpufreq.low_clock == 1) && (cpufreq->low_clock == 0)) {
		REG32(CLOCK_DIV_REG) &= ~((1 << 28)|(1 << 22));
		//tmp = REG32(CPU_CHIP_ID);
		udelay(100);
	}
	
    ratio = REG32(CLOCK_DIV_REG);
    config_clock_parameter(cpufreq, &ratio);
	if (freqs.old_cpufreq.asic_clk != freqs.new_cpufreq.asic_clk)
		ratio |= CLOCK_ASIC_MEM_ENA;

	//change cpu frequence
    CPU_FREQ_CHANGING(ratio);

    cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
}

static void cpu_freq_change(struct cpufreq_mode_clkdiv *cpufreq)
{
	if (freqs.old_cpufreq.pll_sel == freqs.new_cpufreq.pll_sel) {
		ddr2_cpu_freq_change(cpufreq);
		cpufreq_in_ddr2 = 1;
	} else {
		l2_cpu_freq_change(cpufreq);
		cpufreq_in_ddr2 = 0;
	}
}

/*
 * change from low mode to normal mode for suspend (low mode --> normal mode)
 */
void cpu_freq_suspend_check(void)
{
	struct cpufreq_mode_clkdiv cpufreq;
	unsigned long ratio;
	
	if (freqs.old_cpufreq.low_clock == 1) {

		// save low mode before suspend change normal
		prev_suspend_mode = current_mode;
		
		if(!get_cpufreq_mode_clkdiv(SUSPEND_NORMAL_MODE, &cpufreq)){

			if (cpufreq_in_ddr2) {
				// first change to normal mode when other mode change to normal mode
				REG32(CLOCK_DIV_REG) &= ~((1 << 28)|(1 << 22));
				udelay(100);
				
				ratio = REG32(CLOCK_DIV_REG);
				config_clock_parameter(&cpufreq, &ratio);

				// change cpu frequence
				CPU_FREQ_CHANGING(ratio);
			} else {
				cpufreq_change_clocks(&cpufreq);
			}
		}
	}
}
EXPORT_SYMBOL(cpu_freq_suspend_check);

/*
 * restore low mode after resume (normal mode --> low mode)
 */
void cpu_freq_resume_check(void)
{
	struct cpufreq_mode_clkdiv cpufreq;
	unsigned long ratio;

	if (freqs.old_cpufreq.low_clock == 1) {
		if(!get_cpufreq_mode_clkdiv(prev_suspend_mode, &cpufreq)){
			if (cpufreq_in_ddr2) {
				ratio = REG32(CLOCK_DIV_REG);
				config_clock_parameter(&cpufreq, &ratio);

				//change cpu frequence
				CPU_FREQ_CHANGING(ratio);
			} else {
				cpufreq_change_clocks(&cpufreq);
			}
		}
	}
}
EXPORT_SYMBOL(cpu_freq_resume_check);

static void info_clock_value(void)
{
	printk("Cpufreq: system clocks(unit:MHz)[cpu,mem,asic] from (%d,%d,%d) to (%d,%d,%d)\n",
		freqs.old_cpufreq.cpu_clk/clkdiv, freqs.old_cpufreq.mem_clk/clkdiv,
		freqs.old_cpufreq.asic_clk/clkdiv, freqs.new_cpufreq.cpu_clk/clkdiv, 
		freqs.new_cpufreq.mem_clk/clkdiv, freqs.new_cpufreq.asic_clk/clkdiv);
}

static int clock_need_changing(void)
{
	if((freqs.new_cpufreq.cpu_clk == freqs.old_cpufreq.cpu_clk)&&
	  (freqs.new_cpufreq.mem_clk == freqs.old_cpufreq.mem_clk)&&
	  (freqs.new_cpufreq.asic_clk == freqs.old_cpufreq.asic_clk))
		return 0;
	else
		return 1;
}

static unsigned int get_asic_clk(struct cpufreq_mode_clkdiv *cpufreq)
{
    unsigned int asicclk;

    if(cpufreq->is_3x)
        asicclk = ((PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1))/3;
    else
        asicclk = ((PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1))/cpufreq->asic_div;

    return asicclk;
}

static unsigned int get_mem_clk(struct cpufreq_mode_clkdiv *cpufreq)
{
	unsigned int memclk;
	
    if(cpufreq->is_3x)
        memclk = ((PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1))/3;
    else
        memclk = ((PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1))/cpufreq->mem_div;

    return memclk;
}

static unsigned int get_cpu_clk(struct cpufreq_mode_clkdiv *cpufreq)
{
    unsigned int cpuclk;

    if(cpufreq->is_3x) {
        cpuclk = (PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1);
    } else {
        if(cpufreq->cpu_div) {
            cpuclk = (PLL_CLK_MIN+(cpufreq->pll_sel*4))/(cpufreq->clk168_div+1);
        } else {
            if(cpufreq->low_clock)
                cpuclk = get_asic_clk(cpufreq);
            else
                cpuclk = get_mem_clk(cpufreq);
        }
    }
	
    return cpuclk;
}

static void update_current_clock(void)
{
    freqs.old_cpufreq.cpu_clk = freqs.new_cpufreq.cpu_clk;
    freqs.old_cpufreq.mem_clk = freqs.new_cpufreq.mem_clk;
    freqs.old_cpufreq.asic_clk = freqs.new_cpufreq.asic_clk;
	freqs.old = freqs.new;
    freqs.old_cpufreq.pll_sel = freqs.new_cpufreq.pll_sel;
	freqs.old_cpufreq.low_clock = freqs.new_cpufreq.low_clock;
}

static void get_newmode_clock(struct cpufreq_mode_clkdiv *cpufreq)
{	
	freqs.new_cpufreq.cpu_clk = get_cpu_clk(cpufreq)*clkdiv;
	freqs.new_cpufreq.mem_clk = get_mem_clk(cpufreq)*clkdiv;
	freqs.new_cpufreq.asic_clk = get_asic_clk(cpufreq)*clkdiv;
	freqs.new = freqs.new_cpufreq.cpu_clk;	
	freqs.new_cpufreq.pll_sel = cpufreq->pll_sel;
	freqs.new_cpufreq.low_clock = cpufreq->low_clock;
}

void update_pre_mode(void)
{
	// assign to save old mode
	previous_mode_flag = freqs.old_cpufreq.low_clock;
}

unsigned int get_pll_sel(T_OPERATION_MODE state)
{
    int i, len;

    len = ARRAY_SIZE(cpufreq_divs);
    for (i = 0; i < len; i++) {
        if (state == cpufreq_divs[i].mode_name)
            break;
    }
    if (likely(i < len))
        return cpufreq_divs[i].pll_sel;

    return -1;
}
EXPORT_SYMBOL(get_pll_sel);

int previous_mode_is_low_mode(void)
{
	return previous_mode_flag;
}
EXPORT_SYMBOL(previous_mode_is_low_mode);

int current_mode_is_low_mode(void)
{
	return freqs.old_cpufreq.low_clock;
}
EXPORT_SYMBOL(current_mode_is_low_mode);

/*
 *function: get system boot's mode
 */
T_OPERATION_MODE get_current_mode(void)
{
	return current_mode;
}
EXPORT_SYMBOL(get_current_mode);

/* function: enter change cpufreq
 * @state: requested mode name
 * return:
 * 	-1: if system init mode is not surpport.
 * 	  0: if cpufreq change successful.
 */
int request_cpufreq_enter(T_OPERATION_MODE state)
{
	int i, len;

	// check if request suspending, prevent cpufreq when suspending
	if (atomic_read(&suspend_flags))
		return 0;

	if (state == current_mode) {
		//printk("requset new mode equal to current mode.\n");
		return 0;
	}

	len = ARRAY_SIZE(cpufreq_divs);
	for (i = 0; i < len; i++) {
		if (state == cpufreq_divs[i].mode_name)
			break;
	}
	if (likely(i < len)) {
		get_newmode_clock(&cpufreq_divs[i]);
		update_pre_mode();
		
		if (!clock_need_changing()) {
			//printk("Cpufreq: new mode clocks equal to old mode clocks, exit changing.\n");
			return 0;
		}
		cpu_freq_change(&cpufreq_divs[i]);
	} else {
		printk("requset new mode is not surpport.\n");
		return -1;
	}
	
	info_clock_value();
	update_current_clock();
	current_mode = state;

	return 0;
}
EXPORT_SYMBOL(request_cpufreq_enter);

/*
 * function: get system boot's mode
 */
static int get_init_mode(void)
{
    int i;
    unsigned int pllclk, clk168, cpuclk, memclk, asicclk;
    unsigned int pllsel, clk168div, cpudiv, memdiv, asicdiv, lowclock;

    pllclk = ak98_get_pll_clk()/clkdiv;
    clk168 = ak98_get_clk168m_clk()/clkdiv;
    cpuclk = freqs.old_cpufreq.cpu_clk/clkdiv;
    memclk = freqs.old_cpufreq.mem_clk/clkdiv;
    asicclk = freqs.old_cpufreq.asic_clk/clkdiv;
    pllsel = ((pllclk - PLL_CLK_MIN)/4) & 0x3f;

    if(pllclk == clk168)
        clk168div = 0;
    else
        clk168div = pllclk/clk168;
	
	//system is 3x mode
    if((cpuclk / memclk == 3)&&(cpuclk / asicclk == 3)&&
       (cpuclk % memclk == 0)&&(cpuclk % asicclk == 0)) {
        for(i = 0; i < ARRAY_SIZE(cpufreq_divs); i++) {
            if((cpufreq_divs[i].is_3x == 1) &&
              (cpufreq_divs[i].clk168_div == clk168div) &&
              (cpufreq_divs[i].pll_sel == pllsel)){

                current_mode = cpufreq_divs[i].mode_name;
				freqs.old_cpufreq.low_clock = cpufreq_divs[i].low_clock;
                return 0;
            }
        }
        return -1;
    }

	//system is normal mode
	if(cpuclk == clk168) {
		lowclock = 0;
		cpudiv = 1;
	} else if(cpuclk == memclk) {
		lowclock = 0;
		cpudiv = 0;
	} else if(cpuclk == asicclk) {
		lowclock = 1;
		cpudiv = 0;
	}
	memdiv = clk168/memclk;
	asicdiv = clk168/asicclk;

	for(i = 0;	i < ARRAY_SIZE(cpufreq_divs); i++){
		if((cpufreq_divs[i].cpu_div == cpudiv)	&&
		  (cpufreq_divs[i].low_clock == lowclock)&&
		  (cpufreq_divs[i].mem_div == memdiv)	&&
		  (cpufreq_divs[i].asic_div == asicdiv) &&
		  (cpufreq_divs[i].pll_sel == pllsel)	&&
		  (cpufreq_divs[i].clk168_div == clk168div)){

			current_mode = cpufreq_divs[i].mode_name;
			freqs.old_cpufreq.low_clock = cpufreq_divs[i].low_clock;
			return 0;
		}
	}
	return -1;
}

static void cpufreq_operation_init(void)
{
    int i, error;
    unsigned int tmp;

    freqs.old_cpufreq.cpu_clk = ak98_get_cpu_clk();
    freqs.old_cpufreq.mem_clk = ak98_get_mem_clk();
    freqs.old_cpufreq.asic_clk = ak98_get_asic_clk();
    freqs.old = freqs.old_cpufreq.cpu_clk;

    freqs.flags = 0;

    tmp = cpufreq_divs[0].pll_sel;
    for(i = 1; i < ARRAY_SIZE(cpufreq_divs); i++){
        if(cpufreq_divs[i].pll_sel > tmp)
            tmp = cpufreq_divs[i].pll_sel;
    }
    freqs.old_cpufreq.pll_sel = tmp;
	
	error = get_init_mode();
	if(error < 0)
		current_mode = error;

    return;
}

/* ak98_cpufreq_init
 *
 * Attach the cpu frequence  scaling functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/
int __init ak98_cpufreq_init(void)
{
	printk("AK98 cpu frequence change support, (c) 2011 ANYAK\n");
	cpufreq_operation_init();

	return 0;
}
module_init(ak98_cpufreq_init);

