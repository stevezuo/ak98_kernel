
#include <mach/adc1.h>
#include <linux/delay.h>
#include <mach/regs-adc.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>


#undef PDEBUG           /* undef it, just in case */
#ifdef ADC1_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
# define PDEBUG(fmt, args...) printk( KERN_INFO fmt,## args)
# else
/* This one for user space */
# define PDEBUG(fmt, args...) fprintf(stderr, "%s %d: "fmt,__FILE__, __LINE__, ## args)
# endif
#else
# define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif


//static struct semaphore adc1_sem;

static void set_ADC1(unsigned int sample_rate, unsigned int wait_time)
{
	unsigned int bitcycle, hold_time;
		
	unsigned long ADC1_clk = ADC_MAIN_CLK / (ADC1_DIV + 1);
	//clear first
	REG32(AK98_CLK_DIV2) &= ~(0xf << 0);
	//set clock
	REG32(AK98_CLK_DIV2) |= (ADC1_clk & 0xf);	

	//set bitcycle
	/* because ADC1 is 5 channel multiplex, so SampleRate * 5 */
	bitcycle	= (ADC1_clk * MHZ) / (sample_rate*5);	
	REG32(AK98_ADC1_CTRL) &= ~(0xffff << 0);
	REG32(AK98_ADC1_CTRL) |= (1000 & 0xffff); 

	//set hold time
	hold_time = bitcycle - 1;
	REG32(AK98_ADC1_CTRL) &= ~(0xffff << 16);
	REG32(AK98_ADC1_CTRL) |= (hold_time << 16);

	wait_time = bitcycle - 20;
	//set wait time
	REG32(AK98_WTPF_CTRL) &= ~(0xffff << 0);
	REG32(AK98_WTPF_CTRL) |= (wait_time << 0);
	
}

static void select_AD5(void)
{		
	//select AD5
	//AD0-AD4 are enable
	REG32(AK98_ANALOG_CTRL2) &= ~(1 << 11);
}

static void reset_ADC1(void)
{
	//reset ADC1
	REG32(AK98_CLK_DIV2) &= ~(1 << 22);
	mdelay(2);
	REG32(AK98_CLK_DIV2) |= (1 << 22);
}

static void enable_ADC1_clk(void)
{
	//Enable ADC1 clk
	REG32(AK98_CLK_DIV2) |= (1 << 30);
}

static void enable_ADC1(void)
{
	//Enable ADC1
	REG32(AK98_ANALOG_CTRL2) |= (1 << 8);
}
static void enable_ts(void)
{	
	//Enable Ts function
	REG32(AK98_ANALOG_CTRL1) &= ~(1 << 1);
	//Enable TS
	REG32(AK98_ANALOG_CTRL2) |= (1 << 10);
}

/*
	flg: 0  to sample the XP and YP channel only
	flg: 1 to sample YP, XN, YN Xp
*/
static void select_ts_mode(int flg)
{
	if (flg)
	{
		REG32( AK98_PENDOWN_CTRL_REG_WRITE) = (REG32(AK98_PENDOWN_CTRL) |= (1 << 4));
	}
	else
	{
		REG32( AK98_PENDOWN_CTRL_REG_WRITE) = (REG32(AK98_PENDOWN_CTRL) &= (~(1 << 4)));
	}	
}

static void 	set_pd_len(u32 penLen)
{
	//pen down filter counter length
	REG32(AK98_WTPF_CTRL) |= ((penLen & 0xffff) << 16);
}

static void set_threshold(u32 threshold)
{
	//set ts threshold
	//An interrupt will be generated when the change of XP/YP/XN/YN is large than the threshold
	REG32(AK98_ANALOG_CTRL2) &= ~(0x3ff << 17);
	REG32(AK98_ANALOG_CTRL2) |= (threshold << 17);
}

//to enable battery monitor
void ak98_enable_bat_mon(void)
{
	//Enable ADC1
	REG32(AK98_ANALOG_CTRL2) |= (1 << 9);
}

void ak98_disable_bat_mon(void)
{
	REG32(AK98_ANALOG_CTRL2) &= ~(1 << 9);
}
/*
	1 to on
	0 to off
*/
void 	ak98_power_ts(int op)
{
	//power down ts
	if (op == POWER_ON)
		REG32(AK98_ANALOG_CTRL1) &= ~(1 << 0);
	else if ( op == POWER_OFF )
		REG32(AK98_ANALOG_CTRL1) |= (1 << 0);
	else
		PDEBUG("Wrong power operation...\n");
}

void ak98_init_ADC1(unsigned int SampleRate, unsigned int WaitTime)
{
	static int init_flg = 0;
	PDEBUG("%s(): Entering...\n", __FUNCTION__);
	
	if (++init_flg == 1)
	{		
		ak98_power_ADC1(POWER_ON);
		reset_ADC1();
		
		set_ADC1(SampleRate, WaitTime);
		select_AD5();	
		set_pd_len(PENDOWNLEN);
		set_threshold(THRESHOLD);
		
		select_ts_mode(0);	
		enable_ADC1_clk();	
		
		mdelay(5);
		enable_ADC1();
		enable_ts();
		
		ak98_power_ADC1(POWER_OFF);
		ak98_power_ts(POWER_OFF);	
	}
}

void ak98_power_ADC1(int op)
{	
	//power up ADC1
	static int cnt = 0;
	
	if (op == POWER_ON)
	{		
		while (cnt !=0 );
		REG32(AK98_CLK_DIV2) &= ~(1 << 29);
		cnt = 1;
	}
	else if (op == POWER_OFF)
	{
		REG32(AK98_CLK_DIV2) |= (1 << 29);
		cnt=0;
	}

	else 
		PDEBUG("wrong power operation...\n");
}


long ak98_read_voltage(void)
{
	return ((REG32(AK98_ADC1_STATUS)>>10) & 0x3ff); 
}