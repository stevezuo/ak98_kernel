#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/i2c/aw9523.h>
#include "mach/clock.h"

#include <mach/ak98_hal.h>

#define MAX_DACCLK 14000000
#define HP_GAIN_MAX     0x8

REG_ADDR RegAddr;
static int ADC23_State = 0;
static int DAC_State = 0;

/**
 * @brief  open a dac device 
 * @author 
 * @date   
 * @return void
 */
void AK98_DAC_Open(void)
{
	int RegValue;
	if(!DAC_State)
	{
	DAC_State = 1;
	//enable DAC controller clk, l2 controller clk
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) &= ~(DAC_CLK_CTRL_EN | L2_CLK_CTRL_EN);

	// to enable DAC CLK
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= DAC_CLK_EN;

	//soft reset DAC
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) |= DAC_SOFT_RST;
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) &= ~(DAC_SOFT_RST);

	//reset DAC
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~(DAC_RST);
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= DAC_RST;

	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) &= ~MUTE;
	//I2S config reg
	REG32(RegAddr.pAddress2002E + I2S_CONFIG_REG) &= (~I2S_CONFIG_WORDLENGTH_MASK);
	REG32(RegAddr.pAddress2002E + I2S_CONFIG_REG) |= 0XF;  //16 bit   for  memory saving mode.
	REG32(RegAddr.pAddress2002E + I2S_CONFIG_REG) |= POLARITY_SEL; //Send data when left channel data shen data lrclk is high 

	//enable internal DAC/ADC
	REG32(RegAddr.pAddress0800 + MULTIPLE_FUN_CTRL_REG1) |= IN_DAAD_EN;

	//to provide DAC CLK for DAC
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~(DAC_GATE);

	// to enable DACs
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) |= DAC_EN;

	//config I2S interface DAC
	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) |= L2_EN;
	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) |= DAC_CTRL_EN;
	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) &= ~ARM_INT;
	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) |= FORMAT;

	//disable vcm2 discharging
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(PTM_D_CHG_EN);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

	//set vcm2 to normal
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(PL_VCM2);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

	//charge vcm2 slow
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(VREF_CHG_FAST);  //not charge VREF fast
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) |= (1 << 13);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) &= ~(1 << 12);

	//power on integrator in DAC, power on DAC clk
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(PD_OP | PD_CK);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	
	//power on DAC amplifier
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) &= ~(PD_DAOUTL);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) &= ~(PD_DAOUTR);
	}
}

/**
 * @brief   Close a dac device
 * @author  
 * @date   
 * @return  void
 */
void AK98_DAC_Close(void)
{
	int RegValue;
	if(DAC_State)
	{
	DAC_State = 0;
	REG32(RegAddr.pAddress2002E + DAC_CONFIG_REG) &= (~L2_EN);

	// to power off DACs/DAC clock
    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);   
    RegValue |= (PD_OP);
    RegValue |= (PD_CK);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

	mdelay(10);

    REG32(RegAddr.pAddress0800 + MULTIPLE_FUN_CTRL_REG1) &= (~(IN_DAAD_EN));// tdisable internal DAC/ADC
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= (~DAC_CLK_EN);// disable DAC CLK
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= DAC_GATE;//inhibit DAC CLK for DAC
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) &= (~DAC_EN);// to disable DACs
	}
}

/**
 * @brief   Get OSR and DACDIV refer to appointed PLL and sample rate
 * @author  
 * @date   
 * @input   des_sr: destination sample rate
 * @output  osrindex: OSR index
 * @output  mclkdiv: mclk div
 * @return  void
 */
static void DAC_GetOsrDiv(unsigned char *osrindex, unsigned char *mclkdiv, unsigned long des_sr)
{
    const unsigned short OSR_table[8] = 
        {256, 272, 264, 248, 240, 136, 128, 120};

    unsigned long j;
    unsigned long max_div;
    long k;
    unsigned long SR_save, out_sr=0;
    long a;
    long b;
    unsigned long clk168m;

	clk168m = ak98_get_clk168m_clk(); 

    max_div = 0x100;
    SR_save = 0;
    *osrindex = 0;
    *mclkdiv = 0xff;
    for(j=0; j<8; j++) //OSR index
    {
        for(k=max_div-1; k>=0; k--) //DAC_DIV value
        {
            out_sr = clk168m/(k+1);
            if (out_sr > MAX_DACCLK)
                break;
            out_sr = out_sr/OSR_table[j];
            a = out_sr-des_sr;
            a = (a>0)? a : (-a);
            b = SR_save-des_sr;
            b = (b>0)? b : (-b);
            if (a<b)
            {
                SR_save = out_sr;
                *mclkdiv = k;
                *osrindex = j;
            }
        }
    }	
}

/**
 * @brief   Set sample rate
 * @author 
 * @date   
 * @param[in]  samplerate: desired sample rate
 * @return  void
 */
void AK98_DAC_Set_SampleRate(unsigned long samplerate)
{
	unsigned char osr, mclkdiv;
	DAC_GetOsrDiv(&osr, &mclkdiv, samplerate);

	 // NOTE: should reset DAC!!!
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= (~DAC_RST);
	mdelay(5);
    //set OSR
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) &= (~ANALOG_CTRL4_OSR_MASK);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) |= (ANALOG_CTRL4_OSR(osr));
	mdelay(2);

    //set DIV
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= (~(MASK_CLKDIV2_DAC_DIV));
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= (CLKDIV2_DAC_DIV(mclkdiv));    
	mdelay(2);

    //to reset dac
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= DAC_RST;
}

/**
 * @brief   Set DAC channels: mono,stereo
 * @author 
 * @date   
 * @param[in]  bool mono: true-mono, false-stereo
 * @return  void
 */
void AK98_DAC_Set_Channels(unsigned long chnl)
{

}

static void SetHpDisChgCur(unsigned long value)
{
	int RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(0x7 << 27);
	RegValue |= (value << 27);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
}

static void SetPmosCur(unsigned long value)
{
	int RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(0xf << 23);
	RegValue |= (value << 23);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
}

static void SetVcm2DischgCur(unsigned long value)
{
	int RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~(0xf << 6);
	RegValue |= (value << 6);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
}

static void SetVcm2Cur(unsigned long value)
{
	int RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
	RegValue &= ~(0xf << 16);
	RegValue |= (value << 16);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
}


/**
 * @brief   power on HP
 * @author 
 * @date   
 * @param[in] bool poweron or poweroff
 * @return  void
 */
void AK98_Poweron_HP(bool bOn)
{
	int RegValue;
	printk("-------------------------- AK98_Poweron_HP %d\n",bOn);
	if(bOn)
	{
		/*
		* power on codec,power off vcm2/vcm3, pull down vcm2
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PD_REF);
		RegValue |= PD_VCM2;
		RegValue |= PD_VCM3;
		RegValue |= PL_VCM2;
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		//mdelay(10);

		/*
		* Headphone discharging selection
		*/
		SetHpDisChgCur(0x0);
		//mdelay(2);
		SetHpDisChgCur(0x1);
		//mdelay(2);
		SetHpDisChgCur(0x3);
		//mdelay(2);
		SetHpDisChgCur(0x7);
		//mdelay(2);

		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PTM_D_CHG_EN);  //disable VCM2 discharing
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		
		//disable to discharge the off-chip AC coupling capacitor
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(1<<27);  //disable VCM2 discharing
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(1<<28);  //disable VCM2 discharing
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(1<<29);  //disable VCM2 discharing
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		* to enable de-pipa noise control
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= PRE_EN1;
		RegValue |= PRE_EN2;
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		* set hp gain
		*/
		//SetHpGain(0x1);

		SetPmosCur(0x0);   //set headphone PMOS

		/*
		* power on HP
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PD1_HP);
		RegValue &= ~(1<<23);
		RegValue &= ~(1<<24);
		RegValue &= ~(1<<25);
		RegValue &= ~(1<<26);
		RegValue &= ~(PD2_HP);
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		//mdelay(5);

		/*
		* charge PMOS
		*/
		SetPmosCur(0x0);
		//mdelay(5);
		SetPmosCur(0x8);
		//mdelay(5);
		SetPmosCur(0xb);
		//mdelay(5);
		SetPmosCur(0xe);
		//mdelay(5);
		SetPmosCur(0xf);
		//mdelay(5);
		//SetPmosCur(0x0);

		/*
		* to set VCM2 normal,  power on vcm2 vcm3
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PL_VCM2); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PD_VCM2); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PD_VCM3); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		*  charge VCM2
		*/
		SetVcm2Cur(0xf);
		//mdelay(20);
		SetVcm2Cur(0xe);
		//mdelay(10);
		SetVcm2Cur(0xd);
		//mdelay(10);
		SetVcm2Cur(0xc);
		//mdelay(10);
		SetVcm2Cur(0x7);
		//mdelay(10);
		SetVcm2Cur(0x8);
		//mdelay(10);
		SetVcm2Cur(0x6);
		//mdelay(10);
		SetVcm2Cur(0x3);
		//mdelay(10);
		SetVcm2Cur(0x4);
		//mdelay(10);
		SetVcm2Cur(0x2);
		//mdelay(10);
		SetVcm2Cur(0x1);
		//mdelay(600);
		SetVcm2Cur(0x0);

		/*
		*  select HP in
		*/
		// selectHPIn();
		//setHPGain();
	}
	else
	{
		/*
		* to set VCM2 normal
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PL_VCM2); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		//setHPGain();
		//setHPMute();

		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= PTM_D_CHG_EN;  //to enable VCM2 discharing
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		SetVcm2DischgCur(0x0);

		/*
		* power off vcm2,vcm3
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PD_VCM2); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PD_VCM3); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		* discharge vcm2
		*/
		SetVcm2DischgCur(0x1);
		//mdelay(10);
		SetVcm2DischgCur(0x3);
		//mdelay(10);
		SetVcm2DischgCur(0x7);
		//mdelay(10);
		SetVcm2DischgCur(0x8);
		//mdelay(50);
		SetVcm2DischgCur(0xf);
		//mdelay(500);

		/*
		* pull down vcm2
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PL_VCM2); 
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		* power off HP
		*/
		SetPmosCur(0x0);
		SetPmosCur(0xf);
		//mdelay(5);
		SetPmosCur(0xe);
		//mdelay(5);
		SetPmosCur(0xb);
		//mdelay(5);
		SetPmosCur(0x8);
		//mdelay(5);
		
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PD2_HP); //power off pmos
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		//mdelay(20);
		SetHpDisChgCur(0x1);
		//mdelay(2);
		SetHpDisChgCur(0x3);
		//mdelay(2);
		SetHpDisChgCur(0x7);
		//mdelay(50);
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PD1_HP); //power off nmos
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
		//mdelay(2);

		/*
		*  enable de-pipa
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= PRE_EN1;
		RegValue |= PRE_EN2;
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		/*
		* power off codec
		*/
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue |= (PD_REF);
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
}

/**
 * @brief  set HP gain
 * @author 
 * @date   
 * @param[in]  x: (0.x)times
 * @return  void
 */
void AK98_Set_HPGain(unsigned long gain)
{
    unsigned long reg_value;
    unsigned long gain_table[9] = {0x1ff,0xff,0x7f,0x3f,0x1f,0xf,0x7,0x3,0x1};
    if(gain > HP_GAIN_MAX)
    {
    	gain = HP_GAIN_MAX;
    }

    reg_value = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
    reg_value &= ~(0x1FF << 23);
    reg_value |= (gain_table[gain] << 23);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = reg_value;
}

/**
 * @brief  select HP in signal
 * @author 
 * @date   
 * @param[in]  (HP_In_Signal)signal: signal desired
 * @return  void
 */
void AK98_Set_HP_In(unsigned long signal)
{
	unsigned long RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
    RegValue &= ~(0x7 << 12);
    RegValue |= ((signal&0x7) << 12);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
}

/**
 * @brief  open ADC23
 * @author 
 * @date   
 * @param[in]  void
 * @return  void
 */
void AK98_ADC23_Open(void)
{
	unsigned long RegValue;
	if(!ADC23_State)
	{
	ADC23_State = 1;
	//reset ADC2 and ADC3
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~(ADC2_RST);
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= (ADC2_RST);

	// soft reset ADC2 controller
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) |= (ADC2Ctrl_SOFT_RST);
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) &= ~(ADC2Ctrl_SOFT_RST);

	SetPmosCur(0x0);  //must be set to 0 when ADC23 is working
	
	REG32(RegAddr.pAddress0800 + CLOCK_CTRL_REG1) &= ~(ADC2_CLK_CTRL_EN);//enable ADC23 Clock 
	REG32(RegAddr.pAddress0800 + MULTIPLE_FUN_CTRL_REG1) |= (1 << 23); //enable internal
	
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	RegValue &= ~PD_VCM3;                  //power on vcm3   
    RegValue &= ~PD_VCM2;                  //power on vcm2  
    RegValue &= ~PL_VCM2;                  //power on vcm2 
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) |= ADC2_CTRL_EN;
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) |= ADC2MODE_L2_EN;
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) &= ~HOST_RD_INT_EN;
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) |= CH_POLARITY_SEL;//Receive the left channel data when the lrclk is high
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) &= ~I2S_EN;        //Internal ADC MODE
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) &= ~WORD_LENGTH_MASK;
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) |= (0XF << 8);     //WORD LENGTH IS 16 BIT

	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) |= (1 << 13);   //enable verf 
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) |= (1 << 12);   //choose rerf

	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= (ADC2_CLK_EN);    //enable adc23 clock
	REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~(ADC2_GATE);     //provide adc23 clock

    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
    RegValue &= ~(1 << 5);                 //DisableVcm2Dischange
    RegValue &= ~(1 << 4);                 //don't change ver fast
    RegValue |= (0xf << 6);
    RegValue &= ~(1 << 15);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

    mdelay(1);

    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
    RegValue &= ~PD_REF;     //power on codec
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
}

/**
 * @brief      close ADC23
 * @author 
 * @date   
 * @param[in]    void 
 * @return  void
 */
void AK98_ADC23_Close(void)
{
	unsigned long RegValue;
	if(ADC23_State)
	{
	ADC23_State = 0;

    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~ADC2_CLK_EN;     //disable ADC2 clk
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= (ADC2_GATE);     //inhabit adc23 clock
    REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) &= ~ADC2_CTRL_EN; //disable ADC2 interface
	REG32(RegAddr.pAddress2002D + ADC2MODE_CFG_REG) &= ~ADC2MODE_L2_EN;
	
    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
    RegValue |= (PD_ADC2 | PD_ADC3); 
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
	}
}

/**
 * @brief      set ADC23 mode and clk_div
 * @author 
 * @date   
 * @param[in]    des_sr: desired rate to be set
 * @param[out]  mode_sel: reg(0x08000064) bit[12]
 * @param[out]  mclkdiv: reg(0x08000008) bit[11:4]
 * @return  void
 */
static void ADC2_GetOsrDiv(unsigned char *mode_sel, unsigned char *mclkdiv, unsigned long des_sr)
{
	
    unsigned short k, max_div;
    unsigned short OSR_value=256;
    unsigned long SR_save, out_sr=0;
    signed long a, b;
	unsigned long clk168m;
	clk168m = ak98_get_clk168m_clk();

    max_div = 0x100;
    SR_save = 0;
    *mode_sel = 0;
    *mclkdiv = 0;

    if (des_sr > 24000)
    {
        OSR_value = 256;
        *mode_sel = 1; //48k mode
    }
    else
    {
        OSR_value = 512;
        *mode_sel = 0; //16k mode
    }
    
    for(k=0; k<max_div; k++) //DIV
    {
        out_sr = clk168m/(k+1)/OSR_value;
        a = out_sr - des_sr;
        a = (a>0)? a : (-a);
        b = SR_save - des_sr;
        b = (b>0)? b : (-b);
        if (a<b)
        {
            SR_save = out_sr;
            *mclkdiv = k;
        }
    }
}

/**
 * @brief      set ADC23 sample rate
 * @author 
 * @date   
 * @param[in]    samplerate:  desired rate to be set
 * @return  void
 */
void AK98_ADC23_Set_SampleRate(unsigned long  samplerate)
{
	unsigned char mode_sel = 0;
    unsigned char save_div = 0;

	ADC2_GetOsrDiv(&mode_sel, &save_div, samplerate);

    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) &= ~(MASK_CLKDIV2_ADC2_DIV);
    REG32(RegAddr.pAddress0800 + CLK_DIV_REG2) |= CLKDIV2_ADC2_DIV(save_div);

    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) &= ~(1 << ADC_OSR);
    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG4) |= (mode_sel << ADC_OSR);
}

/**
 * @brief      set ADC23 source
 * @author 
 * @date   
 * @param[in]    signal:  DAC|LINEIN|MIC
 * @return  void
 */
void AK98_Set_ADC23_In(unsigned long signal)
{
	unsigned long RegValue;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
	RegValue &= ~(0x7 << 2);
	RegValue |= ((signal&0x7) << 2);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
}

/**
 * @brief      set ADC23 channel
 * @author 
 * @date   
 * @param[in]    chnl: 1-mono; 2-stereo
 * @return  void
 */
void AK98_ADC23_Set_Channels(unsigned long chnl)
{
	unsigned long RegValue = 0;	
	switch(chnl)
    {
        case 1:
			RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
			RegValue &= ~(1 << 20); 			   //power on adc2 conversion
			RegValue |= (1 << 21); 			   //power off adc3 conversion
			RegValue &= ~PD_ADC2;            //power on left channel
			RegValue |= PD_ADC3;             //power off right channel
			RegValue |= (1 << 5);				   //ADC2 limit function
			RegValue &= ~(1 << 6);				   //disable ADC3 limit function
			REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
        break;
        case 2:
        default:
	        RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
			RegValue &= ~(1 << 20); 			   //power on adc2 conversion
			RegValue &= ~(1 << 21); 			   //power on adc3 conversion
			RegValue &= ~PD_ADC2;				   //power on adc2
			RegValue &= ~PD_ADC3;				   //power on adc3
			RegValue |= (1 << 5);				   //ADC2 limit function
			RegValue |= (1 << 6);				   //ADC3 limit function
			REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
        break;
    }
}

/**
 * @brief      set mic gain
 * @author 
 * @date   
 * @param[in]    gain: 0~7
 * @return  void
 */
void AK98_Set_MicGain(unsigned long gain)
{
	unsigned long RegValue = 0;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
	RegValue &= ~(0x7<<11);
	RegValue |= ((gain&0x7)<<11);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
}

/**
 * @brief      set linein gain
 * @author 
 * @date   
 * @param[in]    gain: 0~15
 * @return  void
 */
void AK98_Set_LineinGain(unsigned long gain)
{
	unsigned long RegValue = 0;
	RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
	RegValue &= ~(0xF<<7);
	RegValue |= ((gain&0xF)<<7);
	REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
}

/**
 * @brief      set linein interface power
 * @author 
 * @date   
 * @param[in]    bTrue: 1-power on; 0-power off 
 * @return  void
 */
void AK98_Linein_PowerOn(bool bOn)
{
	unsigned long RegValue = 0;
	if(bOn)
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
	    RegValue &= ~(1 << 14); //power on left channel
	    RegValue &= ~(1 << 15); //power on reght channel
	    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
	}
	else
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_READ);
		RegValue |= (1 << 14); //power off left channel
		RegValue |= (1 << 15); //power off reght channel
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG2_WRITE) = RegValue;
	}
}

/**
 * @brief      set DAC to lineout
 * @author 
 * @date   
 * @param[in]    bTrue: 1-connet DACto lineout; 0-disconnect 
 * @return  void
 */
void AK98_Mic_PowerOn(bool bOn)
{
 	unsigned long RegValue = 0;
	if(bOn)
	{
		//power on mic interface
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
        RegValue &= ~(0x3 << 21);  //power on differential mic
        REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
	else
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
        RegValue |= (0x3 << 21);  //power off differential mic
        REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
}

/**
 * @brief      set DAC to lineout
 * @author 
 * @date   
 * @param[in]    bTrue: 1-connet DACto lineout; 0-disconnect 
 * @return  void
 */
void AK98_DAC_Bypass(bool bTrue)
{
	unsigned long RegValue = 0;
	if(bTrue)
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
        RegValue &= ~(MUXDA_L | MUXDA_R);  //enable left/right channel of DAC out
        REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
	else
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
        RegValue |= (MUXDA_L | MUXDA_R);  //disable left/right channel of DAC out
        REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
}

/**
 * @brief      set mic to lineout
 * @author 
 * @date   
 * @param[in]    bTrue: 1-connet mic to lineout; 0-disconnect 
 * @return  void
 */
void AK98_MicIn_Bypass(bool bTrue)
{
	unsigned long RegValue = 0;
	if(bTrue)
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3);
		RegValue |= (MICIN_BYPASS); //bypass mic in to lineout
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) = RegValue;
	}
	else
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3);
		RegValue &= ~(MICIN_BYPASS);  //not bypass
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) = RegValue;
	}
}

/**
 * @brief      set linein to lineout
 * @author 
 * @date   
 * @param[in]    bTrue: 1-connet linein to lineout; 0-disconnect 
 * @return  void
 */
void AK98_LineIn_Bypass(bool bTrue)
{
	unsigned long RegValue = 0;
	if(bTrue)
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3);
		RegValue |= (LINEIN_BYPASS);   //bypass linein to lineout
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) = RegValue;
	}
	else
	{
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3);
		RegValue &= ~(LINEIN_BYPASS);  //not bypass
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG3) = RegValue;
	}
}

/**
 * @brief      set src to lineout
 * @author 
 * @date   
 * @param[in]      src: DAC|LINEIN|MIC  
 * @return  void
 */
void AK98_Set_Bypass(unsigned long src)
{
	bool s_DAC = src&(0x1);
	bool s_Linein = src&(0x2);
	bool s_Mic = src&(0x4);
	AK98_DAC_Bypass(s_DAC);
	AK98_LineIn_Bypass(s_Linein);
	AK98_MicIn_Bypass(s_Mic);	
}

void AK98_Poweron_VCM_REF(bool bOn)
{
	unsigned long RegValue;
	if(bOn)
	{
		//add power control here for MIC-to-Lineout channel
		//power on REF
		RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
		RegValue &= ~(PD_REF | PL_VCM2 );
		//RegValue |= (PRE_EN1 | PRE_EN2 | TRIMP_HP | TRIMN_HP);
		REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

		//power on vcm2/vcm3
	    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	    RegValue &= ~(PD_VCM2 | PD_VCM3);
	    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
	else
	{
		//power off vcm2/vcm3
	    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	    RegValue |= (PD_VCM2);
	    RegValue |= (PD_VCM3);           
	    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;

	    //power off codec
	    RegValue = REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_READ);
	    RegValue |= (PD_REF | PL_VCM2);
	    REG32(RegAddr.pAddress0800 + ANALOG_CTRL_REG1_WRITE) = RegValue;
	}
}

/**
 * @brief      set input device power
 * @author 
 * @date   
 * @param[in]      src: DAC|LINEIN|MIC  
 *                       addr:route No.  
 *                       CurSrc:all three route's current src
 * @return  void
 */
void AK98_Set_SrcPower(int src,int addr,int *CurSrc)
{
	bool s_DAC = 0;
	bool s_Linein = 0;
	bool s_Mic = 0;
	bool s_src = 0;

	if(0==addr)
	{
		s_DAC = (src|CurSrc[1]|CurSrc[2])&(0x1);
		s_Linein = (src|CurSrc[1]|CurSrc[2])&(0x2);
		s_Mic = (src|CurSrc[1]|CurSrc[2])&(0x4);
		
	}
	else if(1==addr)
	{
		s_DAC = (src|CurSrc[0]|CurSrc[2])&(0x1);
		s_Linein = (src|CurSrc[0]|CurSrc[2])&(0x2);
		s_Mic = (src|CurSrc[0]|CurSrc[2])&(0x4);
	}
	else if(2==addr)
	{
		s_DAC = (src|CurSrc[0]|CurSrc[1])&(0x1);
		s_Linein = (src|CurSrc[0]|CurSrc[1])&(0x2);
		s_Mic = (src|CurSrc[0]|CurSrc[1])&(0x4);
	}
	s_src = s_DAC|s_Linein|s_Mic;
	AK98_Poweron_VCM_REF(s_src);
	
	//set DAC power in PCM interface function: ak98pcm_playback_prepare()
	AK98_Linein_PowerOn(s_Linein);
	AK98_Mic_PowerOn(s_Mic);
}

/**
 * @brief   cfg shutdown speaker GPIO
 * @author 
 * @date   
 * @param[in]  bOn: 1-power on; 0-power off
 * @return  void
 */
void AK98_Poweron_Speaker(unsigned int pin, bool bOn)
{
	printk("-----------------------AK98_Poweron_Speaker %d\n",bOn);
	ak98_setpin_as_gpio(pin);
	ak98_gpio_cfgpin(pin, AK98_GPIO_DIR_OUTPUT);
	ak98_gpio_setpin(pin, bOn);	
}
