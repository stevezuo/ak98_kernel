/*
 * arch/arm/mach-ak98/ddr2change.c
 */
 
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <mach/l2_exebuf.h>

/*
 * function: initialization ram register for ddr2 in L2
 */
void L2_LINK(ddr2change) L2FUNC_NAME(ddr2change)(unsigned long param1,
	unsigned long param2,unsigned long param3,unsigned long param4)
{
    DISABLE_CACHE_MMU();
    DDR2_ENTER_SELFREFRESH();
    PM_DELAY(0x300);

    //disable ram clock
    REG32(PHY_CLOCK_CTRL_REG) |= (1<<10);

    REG32(PHY_RAM_CFG_REG2) = param1; 
    REG32(PHY_RAM_CFG_REG3) = param2; 
	PM_DELAY(0x2000);

    //enable ram clock
    REG32(PHY_CLOCK_CTRL_REG) &= ~(1<<10);

    DDR2_EXIT_SELFREFRESH();
    PM_DELAY(0x2000);
	LED_PHY_ON;
    ENABLE_CACHE_MMU();
}

/*
 *function: configuration ram register
 *@mem_clk: mem clock for ddr2
 */
void sdram_on_change(unsigned int mem_clk)
{
	/* unit in ns
	* refer to sdram spec.
	*/
	#define tRRD	40	//ative to active time, min is 10ns
	#define tRAS	112  //acive to precharge time, min is 45
	#define tRCD	25	//actvie to read/write delay, min is 18
	#define tRP 	25	//precharge to next actvie/refresh time, min is 18
	#define tRFC	256 //refresh to active/refresh command time
	#define tWR 	25	//data in to precharge time
	#define tWTR	25	 //data in to read command delay
	#define tRTW	75	//read to write command delay

    unsigned long cycle, value1, value2, auto_refresh;
    unsigned char  t_ras=15, t_rcd=7, t_rp=7, t_rfc=15, t_rrd=3;   //in clk cycle
    unsigned char  t_wr=7, t_wtr=3, t_rtw=3;

    return; //not implemented now
    
    cycle = 1000/(mem_clk/1000000);
  
    t_rrd = tRAS / cycle + 1; if (t_rrd > 7) t_rrd = 7;
    t_ras = tRAS / cycle + 1; if (t_ras > 31) t_ras = 31;
    t_rcd = tRCD / cycle + 1; if (t_rcd > 15) t_rcd = 15;
    t_rp  = tRP / cycle + 1; if (t_rp > 15) t_rp = 15;
    t_rfc = tRFC / cycle + 1; if (t_rfc > 255) t_rfc = 255;

    t_wr  = tWR / cycle + 1; if (t_wr > 15) t_wr = 15;
    t_rtw = tRTW / cycle + 1; if (t_rtw > 15) t_rtw = 15;
    t_wtr = tWTR / cycle + 1; if (t_wtr > 15) t_wtr = 15;

    //update sdram AC charateristics
    value1 = REG32(RAM_CFG_REG2);

    value1 &= ~(0x7<<21); //clear rrd !!!
    value1 |= t_rrd<<21;    

    value1 &= ~(0x1f<<16); //clear ras
    value1 |= t_ras<<16;

    value1 &= ~(0xf<<12); //clear rp !!!
    value1 |= t_rp<<12;
    
    value1 &= ~(0xf<<8); //clear rcd
    value1 |= t_rcd<<8;

    value1 &= ~(0xff<<0); //clear rfc
    value1 |= t_rfc<<0;

    //config reg2
    value2 = REG32(RAM_CFG_REG3);
    value2 &= ~(0xf<<12);
    value2 |= t_rtw<<12;//tRTW
    value2 &= ~(0xf<<0);
    value2 |= t_wr<<0;//tWR

	//copy the code that set pll value to L2(0x48000000), then move pc to L2
	SPECIFIC_L2BUF_EXEC(ddr2change, value1, value2, 0, 0);


	//ddr2 reset


	//ram controller

}


