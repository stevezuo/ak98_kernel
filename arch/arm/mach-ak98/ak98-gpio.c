/*
 *  arch/arm/mach-ak98/gpio.c
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/ak98-gpio.h>



#if 1
//share pin config fore module in AK980x
struct gpio_sharepin_cfg share_cfg_module[] = {
    //  func_module       share_config         reg1_bit_mask           reg1_bit_value                reg2_bit_mask       reg2_bit_value
    {ePIN_AS_PCM,      SHARE_CFGBOTH,	(1<<0),				(1<<0),                 (1<<0),         (0<<0)},
    {ePIN_AS_JTAG,     SHARE_CFGBOTH,	(1<<0),				(1<<0),                 (1<<0),			(1<<0)},
	{ePIN_AS_RTCK,	   SHARE_CFG1,		(1<<1),				(1<<1),                 0,				0},
	{ePIN_AS_I2S,      SHARE_CFG1,		((1<<2)|(1<<8)), 	((1<<2)|(1<<8)),		0,				0},
	{ePIN_AS_USB_OTG,  SHARE_CFG1,		(1<<3),				(1<<3),                 0,				0},
	{ePIN_AS_PWM1,     SHARE_CFG1,		(1<<4),				(1<<4),                 0,				0},
    {ePIN_AS_PWM2,     SHARE_CFG1,		(1<<5),				(1<<5),                 0,				0},
    {ePIN_AS_PWM3,     SHARE_CFG1,		(1<<6),				(1<<6),                 0,				0},
    {ePIN_AS_PWM4,     SHARE_CFG1,		(1<<7),				(1<<7),                 0,				0},
    {ePIN_AS_UART1,    SHARE_CFG1,		(1<<9),				(1<<9),                 0,				0},
    {ePIN_AS_UART2,    SHARE_CFG1,		(3<<10),			(3<<10),                0,				0},
    {ePIN_AS_UART3,    SHARE_CFG1,		(3<<12),			(3<<12),                0,				0},
    {ePIN_AS_UART4,    SHARE_CFGBOTH,	(3<<14),			(3<<14),                (3<<1),			(1<<1)}, 
    {ePIN_AS_NFC,      SHARE_CFGBOTH,	((0xF<<16)|(1<<22)), ((0xF<<16)|(1<<22)),	(3<<3),			(1<<3)},
    {ePIN_AS_NFC_EXT,  SHARE_CFG2,		0,					0,                      (1<<7),			(1<<7)},
    {ePIN_AS_CAMERA,   SHARE_CFG1,		(1<<24),			(1<<24),                0,				0},
    {ePIN_AS_LCD,      SHARE_CFG1,		(0xF<<25),			(0xF<<25),              0,				0},
    {ePIN_AS_LCD_EXT,  SHARE_CFG2,		0,					0,                      (1<<8),			(1<<8)},
    {ePIN_AS_SDMMC1,   SHARE_CFGBOTH,	(7<<16),			(7<<16),                (3<<3),			(2<<3)},
    {ePIN_AS_SDMMC2,   SHARE_CFGBOTH,	(1<<29),			(1<<29),                (3<<5),			(2<<5)},
    {ePIN_AS_SDIO,     SHARE_CFGBOTH,	(3<<14),			(3<<14),                (3<<1),			(2<<1)},
    {ePIN_AS_SPI1,     SHARE_CFG1,		(1<<30),			(1<<30),                0,				0},
    {ePIN_AS_SPI2,     SHARE_CFG1,		(1<<31),			(1<<31),                0,				0},
    {ePIN_AS_MAC,      SHARE_CFG2,		0,					0,                      (1<<11)|(1<<7),	(1<<11)|(0<<7)},
    {ePIN_AS_CLK12MO,  SHARE_CFG2,		0,					0,                      (1<<18),		(1<<18)},
    {ePIN_AS_CLK32KO,  SHARE_CFG2,		0,					0,                      (1<<19),		(1<<19)},
    {ePIN_AS_I2C,      SHARE_CFG2,		0,					0,                      (1<<21),		(1<<21)},
                                                                                                            
    {ePIN_AS_DUMMY,    EXIT_CFG,		0,					0,                      0,				0}
};


//this used to clr in gpio chare pin cfg1
T_SHARE_CFG_GPIO share_cfg_gpio[] = {
    //start         end             bit
    {0,             3,              0},
    {4,             4,              1},
    {5,             7,              2},
    {8,             8,              3},
    {9,             9,              4},
    {10,            10,             5},
    {11,            11,             6},
    {12,            12,             7},
    {13,            13,             8},
    {18,            19,             11},
    {22,            23,             13},
    {24,            25,             14},
    {26,            27,             15},
    {30,            30,             16},
    {31,            33,             17},
    {34,            37,             18},
    {38,            38,             19},
    {41,            41,             22},
    {47,            58,             24},
    {62,            68,             25},
    {61,            61,             26},
    {69,            70,             27},
    {71,            71,             28},
    {72,            75,             29},
    {76,            79,             30},
    {80,            83,             31},
};

//this used to clr in gpio chare pin cfg2
T_SHARE_CFG_GPIO share_cfg_gpio2[] = {
    //start       	    end                    bit
    {84,            89,             8},
    {90,            90,             9},
    {91,            91,             10},
};

//this used to set in gpio chare pin cfg2
T_SHARE_CFG_GPIO share_cfg_gpio3[] = {
    //start             end                   bit
    {14,            14,             12},
    {15,            15,             13},
    {16,            16,             14},
    {17,            17,             15},
    {20,            20,             16},
    {21,            21,             17},
};


#define INVALID_WK_BIT 0xff
struct t_gpio_wakeup_cfg gpio_wakeup_cfg[] = {
		//gpio_start        gpio_end      start_bit
		{AK98_GPIO_5,  AK98_GPIO_7,  0 },
		{AK98_GPIO_10, AK98_GPIO_13, 3 },
		{AK98_GPIO_16, AK98_GPIO_19, 7 },
		{AK98_GPIO_24, AK98_GPIO_27, 11},
		{AK98_GPIO_72, AK98_GPIO_75, 15},
		{AK98_GPIO_104,AK98_GPIO_107,19},
		{AK98_GPIO_84, AK98_GPIO_84, 23},
		{AK98_GPIO_109,AK98_GPIO_116,24},
};

/*
    this table contains all the GPIOs whose attribute can be set, the io attribute is one 
    of {IE, PE, SL, DS, PS}, one gpio may support only one or several attributes setting,  
    an element of row 2~5 indicates whether this    attibute setting is aupported by this gpio, 
    if supported, this element tells us how can we set register.

    the value is encoded as:
        bit[0:7]: register addr shift, base addr is CHIP_CONF_BASE_ADDR
        bit[8:15]: start bit

    take value 0x3d4 for example:
    the lower 8-bit is 0xd4, that is the shift, then the address of the register we will configure
    is CHIP_CONF_BASE_ADDR+0xd4

    bit[8:15] of 0x3d4 is 3, that means we shoulde configure the bit[3] of register 
    "CHIP_CONF_BASE_ADDR+0xd4"
*/
static const unsigned long gpio_io_set_table_9801[][PIN_ATTE_LINE] = { 
    //pin no.   IE              PE                  SL                      DS                      PS
    {16,	0x3d4,      0xfd4,      ATTR_FIXED_1,     ATTR_FIXED_0,     0x109c},
    {17,	0x2d4,      0xed4,      ATTR_FIXED_1,     ATTR_FIXED_0,     0x119c},
    {18,	0x5d4,      0x11d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x129c},
    {19,	0x4d4,      0x10d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x139c},
    {20,	0x7d4,      0x13d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x149c},
    {21,	0x6d4,      0x12d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x159c},
    {22,	0x9d4,      0x15d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x169c},
    {23,	0x8d4,      0x14d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x179c},               
    {24,	0xbd4,      0x17d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x189c},
    {25,	0xad4,      0x16d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x199c},
    {26,	0xdd4,      0x19d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x1a9c},
    {27,	0xcd4,      0x18d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x1b9c},
    {28,	0x1bd4,     0x1c9c,     ATTR_FIXED_1,     0x1d4,			ATTR_FIXED_1},
    {29,	0x1ad4,     0x1d9c,     ATTR_FIXED_1,     0x0d4,			ATTR_FIXED_1},
                
    {109,	0x4d8,      0x8d8,      ATTR_FIXED_1,     0xcd8,			0xda8},
    {END_FLAG,   0,		0,          0,					0,				0}
};
static const unsigned long gpio_io_set_table_9805[][PIN_ATTE_LINE] = { 
    //pin no.   IE              PE                  SL                      DS                      PS
    {16,	0x3d4,      0xfd4,      ATTR_FIXED_1,     ATTR_FIXED_0,     0x109c},
    {17,	0x2d4,      0xed4,      ATTR_FIXED_1,     ATTR_FIXED_0,     0x119c},
    {18,	0x5d4,      0x11d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x129c},
    {19,	0x4d4,      0x10d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x139c},
    {24,	0xbd4,      0x17d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x189c},
    {25,	0xad4,      0x16d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x199c},
    {26,	0xdd4,      0x19d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x1a9c},
    {27,	0xcd4,      0x18d4,     ATTR_FIXED_1,     ATTR_FIXED_0,     0x1b9c},
    {28,	0x1bd4,     0x1c9c,     ATTR_FIXED_1,     0x1d4,			ATTR_FIXED_1},
    {29,	0x1ad4,     0x1d9c,     ATTR_FIXED_1,     0x0d4,			ATTR_FIXED_1},
    {END_FLAG,   0,		0,          0,					0,				0}
};


/*
    this table contains all the GPIOs attribute: each GPIO is represented by 2-bit,
    which equals to:   
    			00-pullup
                   01-pulldown
                   10-pullup/pulldown
                   11-undefined
*/
const unsigned long gpio_pull_attribute_table_9801[][2] = {  
    {0x04455501, /*gpio[0:15]*/          0x00aaaaaa}, /*gpio[16:31]*/             //reg1
    {0x54000000, /*gpio[32:47]*/         0x57d55555}, /*gpio[48:63]*/             //reg2
    {0x55005555, /*gpio[64:79]*/         0xf5555555}, /*gpio[80:95]*/             //reg3   
    {0x4b55f557, /*gpio[96:111]*/        0xfffffc01}  /*gpio[112:116]*/           //reg4 
};
const unsigned long gpio_pull_attribute_table_9805[][2] = {  
    {0x07c55501, /*gpio[0:15]*/          0x00aaffaa}, /*gpio[16:31]*/             //reg1
    {0x54000000, /*gpio[32:47]*/         0x57d55555}, /*gpio[48:63]*/             //reg2
    {0x55005555, /*gpio[64:79]*/         0xf55555ff}, /*gpio[80:95]*/             //reg3   
    {0xfff5ffff, /*gpio[96:111]*/        0xffffff3f}  /*gpio[112:116]*/           //reg4 
};


unsigned int ak9801_invalid_gpio[] = {
	AK98_GPIO_59, AK98_GPIO_60, AK98_GPIO_94, 
	AK98_GPIO_95, AK98_GPIO_96, AK98_GPIO_108
};

unsigned int ak9805_invalid_gpio[] = {
	AK98_GPIO_11, AK98_GPIO_12, AK98_GPIO_20, AK98_GPIO_21, AK98_GPIO_22,
	AK98_GPIO_23, AK98_GPIO_59, AK98_GPIO_60, AK98_GPIO_80, AK98_GPIO_81,
	AK98_GPIO_82, AK98_GPIO_83, AK98_GPIO_90, AK98_GPIO_91, AK98_GPIO_94,
	AK98_GPIO_95, AK98_GPIO_96, AK98_GPIO_97, AK98_GPIO_98, AK98_GPIO_99,
	AK98_GPIO_100, AK98_GPIO_101, AK98_GPIO_106, AK98_GPIO_107, AK98_GPIO_108,
	AK98_GPIO_109, AK98_GPIO_110, AK98_GPIO_111, AK98_GPIO_112, AK98_GPIO_113,
	AK98_GPIO_114, AK98_GPIO_116
};


 static unsigned char gpio_assert_legal(unsigned long pin);


static inline unsigned int round_mcgpio(unsigned int pin)
{
	if (pin >= AK98_MCGPIO_0 && pin <= AK98_MCGPIO_19)
		return pin - AK98_MCGPIO_0;
	return pin;
}
static unsigned char get_bit_by_pin_wk(unsigned char pin)
{
	int i, n;
	n = ARRAY_SIZE(gpio_wakeup_cfg);

	for (i=0; i<n; i++)
	{
		if (pin >= gpio_wakeup_cfg[i].gpio_start && pin <= gpio_wakeup_cfg[i].gpio_end)
			return gpio_wakeup_cfg[i].start_bit + (pin - gpio_wakeup_cfg[i].gpio_start);
	}

	return INVALID_WK_BIT;
}


void ak98_gpio_wakeup_pol(unsigned int pin, unsigned char pol)
{
	unsigned char bit = get_bit_by_pin_wk(pin);
	unsigned int val;

	if (bit == INVALID_WK_BIT)
	{
		panic("this pin %u doesn't support wakeup function\n", pin);
		return;
	}

	
	val = REG32(AK98_WGPIO_POLARITY);
	//when the specific bit is set to 0, the wake-up GPIO is rising triggered
	//when the specific bit is set to 1, the wake-up GPIO is falling triggered
	val &= ~(1 << bit);
	val |= (pol << bit);
	
	REG32(AK98_WGPIO_POLARITY) = val;
		
}
EXPORT_SYMBOL(ak98_gpio_wakeup_pol);

int ak98_gpio_wakeup(unsigned int pin, unsigned char enable)
{
	unsigned char bit = get_bit_by_pin_wk(pin);
	unsigned int val;


	if (bit == INVALID_WK_BIT)
	{
		panic("this pin %d doesn't support wakeup function\n", pin);
		return -1;
	}
	//clear wake gpio status
	val = REG32(AK98_WGPIO_CLEAR);
	val |= (1 << bit);
	REG32(AK98_WGPIO_CLEAR) = val;
	val &= ~(1 << bit);
	REG32(AK98_WGPIO_CLEAR) = val;

	
	val = REG32(AK98_WGPIO_ENABLE);
	if (enable == AK98_WAKEUP_ENABLE)
	{
		val |= (1 << bit);
	}
	else if (enable == AK98_WAKEUP_DISABLE)
	{
		val &= ~(1 << bit);
	}
	else
		panic("wrong enable value in ak98_gpio_wakeup\n");
	REG32(AK98_WGPIO_ENABLE) = val;
	
	return 0;
}
EXPORT_SYMBOL(ak98_gpio_wakeup);

/*
 * @brief set gpio pin group as specified module used
 * @param[in] PinCfg enum data. the specified module
 */
void ak98_group_config(T_GPIO_SHAREPIN_CFG mod_name)
{
    unsigned long i, flags, val = 0;
	
    if(ePIN_AS_GPIO == mod_name)
    {
        //set all pin as gpio except uart0
        local_irq_save(flags);
		__raw_writel(0x100, AK98_SHAREPIN_CON1);		
		__raw_writel(0x1, AK98_SHAREPIN_CON2);
		local_irq_restore(flags);
        return;
    }

#ifdef CONFIG_AK9805
	//canot set the following group in ak9805
	if( ePIN_AS_PWM3 == mod_name || ePIN_AS_PWM4 == mod_name ||
		ePIN_AS_UART3 == mod_name || ePIN_AS_SPI2 == mod_name || 
		ePIN_AS_MAC == mod_name )
	{
		panic("Can't set pin configuration: %d in chip Ak9805\n", mod_name);
		return;
	}
#endif

    for(i = 0; ; i++)
    {
        if(ePIN_AS_DUMMY == share_cfg_module[i].func_module)
            break;

        if(mod_name == share_cfg_module[i].func_module)
        {	
        	//set pull attribute for module
            g_ak98_setgroup_attribute(mod_name);
			
			local_irq_save(flags);
            switch(share_cfg_module[i].share_config) {
            	case SHARE_CFG1: //set share pin cfg reg1
					val = __raw_readl(AK98_SHAREPIN_CON1);
					val &= ~(share_cfg_module[i].reg1_bit_mask);
					val |= (share_cfg_module[i].reg1_bit_value);
					__raw_writel(val, AK98_SHAREPIN_CON1);
					break;
					
				case SHARE_CFG2: //set share pin cfg reg2
					val = __raw_readl(AK98_SHAREPIN_CON2);
					val &= ~(share_cfg_module[i].reg2_bit_mask);
					val |= (share_cfg_module[i].reg2_bit_value);
					__raw_writel(val, AK98_SHAREPIN_CON2);
					break;
					
				case SHARE_CFGBOTH: 
					val = __raw_readl(AK98_SHAREPIN_CON1);
					val &= ~(share_cfg_module[i].reg1_bit_mask);
					val |= (share_cfg_module[i].reg1_bit_value);
					__raw_writel(val, AK98_SHAREPIN_CON1);
					
					val = __raw_readl(AK98_SHAREPIN_CON2);
					val &= ~(share_cfg_module[i].reg2_bit_mask);
					val |= (share_cfg_module[i].reg2_bit_value);
					__raw_writel(val, AK98_SHAREPIN_CON2);
					break;
				default:
					break;
            }
			local_irq_restore(flags);
            return ;
        }
    }
	return ;
}
EXPORT_SYMBOL(ak98_group_config);
#endif




void ak98_sharepin_cfg1(unsigned char to, unsigned char offset, T_SHARE_CONFG conf)
{
	void __iomem *base = AK_NULL;
	unsigned long flags;
	
	if (offset > 31) {
		printk("Error, configuration sharepin1 offset larger\n");
		return;
	}
	
	switch(conf) {
		case SHARE_CONFG1:
			base = AK98_SHAREPIN_CON1;
			break;		
		case SHARE_CONFG2:
			base = AK98_SHAREPIN_CON2;
			break;
		default:
			break;
	}
	
	local_irq_save(flags);
	if (0 == to)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return;
}
EXPORT_SYMBOL(ak98_sharepin_cfg1);

void ak98_sharepin_cfg2(unsigned long mask, unsigned long value, unsigned char offset)
{
	void __iomem *base = AK98_SHAREPIN_CON2;
	unsigned long flags, val = 0;

	if (offset > 21) {
		printk("Error, configuration sharepin2 offset larger\n");
		return;
	}

	local_irq_save(flags);
	val = __raw_readl(base);
	val &= ~(mask << offset);
	val |= (value << offset);
	__raw_writel(val, base);
	local_irq_restore(flags);
	return;
}
EXPORT_SYMBOL(ak98_sharepin_cfg2);


 unsigned char gpio_assert_legal(unsigned long pin)
{
	int i, len;
	unsigned int *gpio_legal;
#ifdef CONFIG_AK9801
	len = sizeof(ak9801_invalid_gpio)/sizeof(ak9801_invalid_gpio[0]);
	gpio_legal = ak9801_invalid_gpio;
#endif
#ifdef CONFIG_AK9805
	len = sizeof(ak9805_invalid_gpio)/sizeof(ak9805_invalid_gpio[0]);
	gpio_legal = ak9805_invalid_gpio;
#endif

	for(i = 0; i < len; i++) {
		if(gpio_legal[i] == pin)
			return AK_FALSE;	
	}
	return AK_TRUE;
}
 unsigned long gpio_pin_check(unsigned long pin)
{   
    unsigned long i = 0;
    const unsigned long (*gpio_io_set_table)[PIN_ATTE_LINE] = AK_NULL;

#ifdef CONFIG_AK9801
	gpio_io_set_table = gpio_io_set_table_9801;
#endif
#ifdef CONFIG_AK9805
	gpio_io_set_table = gpio_io_set_table_9805;    
#endif

    while(1)
	{
        if ((gpio_io_set_table[i][0] == pin) || (gpio_io_set_table[i][0] == END_FLAG))
            break;  
        i++;
    }

    if (gpio_io_set_table[i][0] != END_FLAG)
        return i;
    else
        return INVALID_GPIO;
}


/**
 * @brief set gpio share pin as gpio 
 * @param pin [in]  gpio pin ID
 */
int g_ak98_setpin_as_gpio(unsigned int pin)
{
    unsigned long i, bit = 0;
	unsigned long flags;

    //check param
    if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %u configuration!\n", pin);
		return -1;
	}

    if(28 == pin || 29 == pin)
		REG32(AK98_SHAREPIN_CON2) &= ~(0x3 << 1);
	
    //loop to find the correct bits to clr in share ping cfg1
    for(i = 0; i < ARRAY_SIZE(share_cfg_gpio); i++){
        if((pin >= share_cfg_gpio[i].gpio_start) && (pin <= share_cfg_gpio[i].gpio_end))
        {
        	local_irq_save(flags);
            bit = share_cfg_gpio[i].bit;
			REG32(AK98_SHAREPIN_CON1) &= ~(1 << bit);
			local_irq_restore(flags);
            return 0;
        }
    }

    //loop to find the correct bits to clr in share ping cfg2
    for(i = 0; i < ARRAY_SIZE(share_cfg_gpio2); i++){
        if((pin >= share_cfg_gpio2[i].gpio_start) && (pin <= share_cfg_gpio2[i].gpio_end))
        {
        	local_irq_save(flags);
            bit = share_cfg_gpio2[i].bit;
			REG32(AK98_SHAREPIN_CON2) &= ~(1 << bit);
			local_irq_restore(flags);
            return 0;
        }
    }
    
    //loop to find the correct bits to set in share ping cfg2
    for(i = 0; i < ARRAY_SIZE(share_cfg_gpio3); i++){
        if((pin >= share_cfg_gpio3[i].gpio_start) && (pin <= share_cfg_gpio3[i].gpio_end))
        {
       		local_irq_save(flags);
            bit = share_cfg_gpio3[i].bit;
			REG32(AK98_SHAREPIN_CON2) |= (1 << bit);
			local_irq_restore(flags);
            return 0;
        }
    }
    return 0;
}

 int mc_ak98_setpin_as_mcgpio(unsigned int pin)
{
	unsigned int val;
	unsigned long flags;

   	local_irq_save(flags);
	val= REG32(AK98_SHAREPIN_CON2);

	if (pin < AK98_MCGPIO_0 || pin > AK98_MCGPIO_19)
	{
		panic("Error! Invalid mcgpio %u configuration.\n", pin);
		return -1;
	}
	
	if (AK98_MCGPIO_18 == pin)
		val &= ~(1 << 18);
	else if (AK98_MCGPIO_19 == pin)
		val &= ~(1 << 19);
	else
	{
		val &= ~(1 << 11);	
		val &= ~(1 << 7);
	}

	REG32(AK98_SHAREPIN_CON2) = val;

	local_irq_restore(flags);	
	
    return 0;
}



/*
    attr: to set what kind of attribute of pin
    enable: to enable or disable this pin attribute,1:enable 0:disable

    the gpio pin attribure set is mainly for two kinds of pins: one is general GPIOs, the 
    other is RAM bus(data & addr), for general GPIO, param pin in this function is the 
    GPIO number, for RAM bus, param "pin" should be set as MADD_A or MDAT_A

    NOTE: the following four functions:ak98_setpin_attribute, ak98_gpio_pullup,
            ak98_gpio_pulldown, gpio_set_pin_share, you'd better not check the 
            returned value, because it doesn't matter to give a wrong param to a
            specific pin
*/
 void g_ak98_setpin_attribute(unsigned int pin, 
	T_GPIO_PIN_ATTR attr, unsigned char enable)
{
	void __iomem *base = AK98_VA_SYSCTRL;
	void __iomem *reg_addr;
    unsigned long flags, i, reg_bit;
    const unsigned long (*gpio_set_table)[PIN_ATTE_LINE] = AK_NULL;

    if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %u configuration!\n", pin);
		return;
	}

#ifdef CONFIG_AK9801
	gpio_set_table = gpio_io_set_table_9801;
#endif
#ifdef CONFIG_AK9805
	gpio_set_table = gpio_io_set_table_9805;    
#endif

    if ((i = gpio_pin_check(pin)) == INVALID_GPIO){
		/* printk; couldn't preempt */
        return;
    }

    if (gpio_set_table[i][attr] == GPIO_ATTR_UNSUPPORTED) {
		/* printk; couldn't preempt */
        return;		
    }
    else if ((gpio_set_table[i][attr] == ATTR_FIXED_1) || 
                (gpio_set_table[i][attr] == ATTR_FIXED_0))
    {
    	/* printk; couldn't preempt */
        return;
    }

    /*
        if we want to set this attribute, we should get two things:
        1. which register should be configured
        2. which bit of this register should be configured

        following, we decode corresponding items in gpio_io_set_table to 
        get the above two things
    */
    
    reg_addr = base + (unsigned char)(gpio_set_table[i][attr] & 0xff);
    reg_bit = (gpio_set_table[i][attr] >> 8) & 0xff;
	
	local_irq_save(flags);
    if (enable)
		REG32(reg_addr) |= (1 << reg_bit);
    else
		REG32(reg_addr) &= ~(1 << reg_bit);
	local_irq_restore(flags);
    return ; 
}

/*
    enable: 1:enable pullup 0:disable pullup function
	  if the pin is attached pullup and pulldown resistor, then writing 1 to enable
        pullup, 0 to enable pulldown, if you want to disable pullup/pulldown, then 
        disable the PE parameter
*/
 int g_ak98_gpio_pullup(unsigned int pin, unsigned char enable)
{
	void __iomem *base = AK98_PPU_PPD_BASE(pin);
    unsigned long index, reg_data, shift, rs;
    T_GPIO_TYPE reg_bit;
    const unsigned long (*gpio_pull_attribute_table)[2] = AK_NULL;
	unsigned long flags, offset;

   if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %u configuration!\n", pin);
		return -1;
	}
#ifdef CONFIG_AK9801
	gpio_pull_attribute_table = gpio_pull_attribute_table_9801;
#endif
#ifdef CONFIG_AK9805
	gpio_pull_attribute_table = gpio_pull_attribute_table_9805;
#endif

    index = pin / 32;
    offset = pin % 32;
    if (offset < 16)
        shift = 0;
    else
        shift = 1;  
    
    //get pin pullup/pulldown attribute
    reg_data = gpio_pull_attribute_table[index][shift];
    if (shift)
        rs = (offset - 16) * 2;
    else
        rs = offset * 2;
    reg_bit = (reg_data >> rs) & 0x3;   

    if ((reg_bit == PULLDOWN) || (reg_bit == UNDEFINED)) {
		/* printk; couldn't preempt */
        return 0;
    }
        
    //if this pin setting needs IO controller be set
    if (gpio_pin_check(pin) != INVALID_GPIO)
    {       
        g_ak98_setpin_attribute(pin, GPIO_ATTR_PE, AK_TRUE);
    }

	local_irq_save(flags);
    //enable/disable pullup
    if (enable)
    {       
        //if the pin is attached pullup/pulldown registor, else pullup register 	
        if (reg_bit == PULLUPDOWN)
			REG32(base) |= (1 << offset);
        else
			REG32(base) &= ~(1 << offset);
    }
    else 
    {
        //first disable PE attribute
        g_ak98_setpin_attribute(pin, GPIO_ATTR_PE, AK_FALSE);

        //if this pin is attached pullup resistor only, disable it
        if (reg_bit != PULLUPDOWN) 
			REG32(base) |= (1 << offset);
    }
	local_irq_restore(flags);
    return 0; 
}


//1.enable pulldown 0.disable pulldown
 int g_ak98_gpio_pulldown(unsigned int pin, unsigned char enable)
{
	void __iomem *base = AK98_PPU_PPD_BASE(pin);
    unsigned long index, reg_data, shift, rs;
    T_GPIO_TYPE reg_bit;
    const unsigned long (*gpio_pull_attribute_table)[2] = AK_NULL;	
	unsigned long flags, offset;
	
    if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %u configuration!\n", pin);
		return -1;
	}
	
#ifdef CONFIG_AK9801
	gpio_pull_attribute_table = gpio_pull_attribute_table_9801;
#endif
#ifdef CONFIG_AK9805
	gpio_pull_attribute_table = gpio_pull_attribute_table_9805;
#endif

    index = pin / 32;
    offset = pin % 32;
    if (offset < 16)
        shift = 0;
    else
        shift = 1;  
    
    //get pin attribute
    reg_data = gpio_pull_attribute_table[index][shift];
    if (shift)
        rs = (offset - 16) * 2;
    else
        rs = offset * 2;
    reg_bit = (reg_data >> rs) & 0x3;

    if ((reg_bit == PULLUP) || (reg_bit == UNDEFINED)) {
		/* printk; couldn't preempt */
        return -1;
    }

    //if this pin setting needs IO controller be set
    if (gpio_pin_check(pin) != INVALID_GPIO)
    {       
        g_ak98_setpin_attribute(pin, GPIO_ATTR_PE, AK_TRUE);
    }

    //enable/disable pulldown
    local_irq_save(flags);
    if (enable)    
		REG32(base) &= ~(1 << offset);
	else {
        //same as pullup configuration
        g_ak98_setpin_attribute(pin, GPIO_ATTR_PE, AK_FALSE);
        
        if (reg_bit != PULLUPDOWN) 
			REG32(base) |= (1 << offset);
    }
	local_irq_restore(flags);
    return 0; 
}


 void g_ak98_setgroup_attribute(T_GPIO_SHAREPIN_CFG mod_name)
{
    unsigned long pin, start_pin = 0, end_pin = 0;
    
    switch (mod_name) {
        case ePIN_AS_NFC:
        case ePIN_AS_SDMMC1:
            start_pin = 30, end_pin = 37;
            //set databus of Nand, MMCSD 
            for (pin = start_pin; pin <= end_pin; pin++)
            {         
                g_ak98_gpio_pullup(pin, AK_TRUE);
                g_ak98_setpin_attribute(pin, GPIO_ATTR_IE, AK_TRUE);
            }
            break;		
		case ePIN_AS_SPI1:
            start_pin = 76, end_pin = 79;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pullup(pin, AK_TRUE);
            }
            break;
        case ePIN_AS_SPI2:
            start_pin = 80, end_pin = 83;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pullup(pin, AK_TRUE);
            }
            break;
			
        case ePIN_AS_SDIO:
            start_pin = 24, end_pin = 29;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                //enable SDIO dataline pullup
                g_ak98_gpio_pullup(pin, AK_TRUE);   

                //enable SDIO dataline IE attribute
                g_ak98_setpin_attribute(pin, GPIO_ATTR_IE, AK_TRUE);
            }
            break;
            
        case ePIN_AS_I2S:
            //20090806 need to confirm!!!
            start_pin = 5, end_pin = 7;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pulldown(pin, AK_FALSE);   
            }
            g_ak98_gpio_pulldown(AK98_GPIO_13, AK_FALSE); //gpio13 share with I2S_MCLK
            break;
            
        case ePIN_AS_UART1: 
            start_pin = 14, end_pin = 15;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pullup(pin, AK_FALSE);
            }
            break;
            
        case ePIN_AS_UART4: 
            start_pin = 24, end_pin = 27; //end_pin = 27 ,caolianming           
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pullup(pin, AK_TRUE);    
                if ((pin == 25) || (pin == 27))         
                    g_ak98_setpin_attribute(pin, GPIO_ATTR_IE, AK_TRUE);                 
            }   
            break;
			
		case ePIN_AS_I2C:
            start_pin = 92, end_pin = 93;
            for (pin = start_pin; pin <= end_pin; pin++)
            {
                g_ak98_gpio_pulldown(pin, AK_FALSE);
            }
            break;
        default:
            break;
    }
}



/* 
 * configuration gpio pin
 * 1: corresponding port is input mode
 * 0: corresponding port is output mode
 */
int g_ak98_gpio_cfgpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK98_GPIO_DIR_BASE(pin);
	unsigned int offset = ((pin) & 31);
	unsigned long flags;

	
 	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %d configuration!\n", pin);
		return -1;
	}

	local_irq_save(flags);
	if (0 == to) {  //output mode
		REG32(base) &= ~(1 << offset);
		
		//pin90 and pin91 sepecial, cause chip driver has problem.
		if(pin == 90)
			REG32(AK98_SHAREPIN_CON2) &= ~(1 << 9);
		else if (pin == 91)
			REG32(AK98_SHAREPIN_CON2) &= ~(1 << 10);
	} else {   //input mode
		if(gpio_pin_check(pin) != INVALID_GPIO){
            g_ak98_setpin_attribute(pin, GPIO_ATTR_IE, AK_TRUE);
        }
		REG32(base) |= (1 << offset);
		
		if(pin == 90)
			REG32(AK98_SHAREPIN_CON2) |= (1 << 9);
		else if (pin == 91) 
			REG32(AK98_SHAREPIN_CON2) |= (1 << 10);	
	}
	local_irq_restore(flags);
	return 0;
}


/* 
 * configuration gpio pin
 * 1: corresponding port is input mode
 * 0: corresponding port is output mode
 */
int mc_ak98_mcgpio_cfgpin(unsigned int pin, unsigned int to)
{
	void __iomem *base;
	unsigned int offset;
	unsigned long flags;
	
	pin = round_mcgpio(pin);
	base = AK98_MCGPIO_DIR_BASE(pin);
	offset = ((pin) & 31);

	if (pin > MCGPIO_UPLIMIT) {
		panic("Error, Invalid mcgpio %d configuration!\n", pin);
		return -1;
	}
	
	local_irq_save(flags);
	if (0 == to)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return 0;
}





/* hold the real-time output value from GPIO[x] */ 
int g_ak98_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = AK98_GPIO_OUT_BASE(pin);
	unsigned int offset = ((pin) & 31);
	unsigned long flags;

		
	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, Invalid gpio %d configuration!\n", pin);
		return -1;
	}

	local_irq_save(flags);
	if (0 == to)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return 0;
}

int mc_ak98_mcgpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base;
	unsigned int offset;
	unsigned long flags;

	pin = round_mcgpio(pin);
	base = AK98_MCGPIO_OUT_BASE(pin);
	offset = ((pin) & 31);
	
	if (pin > MCGPIO_UPLIMIT) {
		panic("error, invalid mcgpio %d configuration!\n", pin);
		return -1;
	}
	
	local_irq_save(flags);
	if (0 == to)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return 0;
}


/* hold the real-time input value of GPIO[x] */ 
 int g_ak98_gpio_getpin(unsigned int pin)
{
	void __iomem *base = AK98_GPIO_IN_BASE(pin);
	unsigned int offset = ((pin) & 31);

	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, read invalid gpio %d status!\n", pin);
		return -1;
	}
	return ((__raw_readl(base) & (1 << offset)) == (1 << offset));
}

int mc_ak98_mcgpio_getpin(unsigned int pin)
{
	void __iomem *base;
	unsigned int offset;
	
	pin = round_mcgpio(pin);
	base = AK98_MCGPIO_IN_BASE(pin);
	offset = ((pin) & 31);

	
	if(pin > MCGPIO_UPLIMIT) {
		panic("Error, read invalid mcgpio %d status!\n", pin);
		return -1;
	}
	return ((__raw_readl(base) & (1 << offset)) == (1 << offset));
}


/* 0: enable pull down, 1: disable pull down */
int mc_ak98_mcgpio_pulldown(unsigned int pin, unsigned char enable)
{
	void __iomem *base;
	unsigned int offset;
	unsigned long flags;

	pin = round_mcgpio(pin);
	base = AK98_MCGPIO_PPU_PPD_BASE(pin);	
	offset = ((pin) & 31);
	
	if(pin > MCGPIO_UPLIMIT) {
		panic("Error, invalid mcgpio %d!\n", pin);
		return -1;
	}
	
	local_irq_save(flags);
	if (0 == enable)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return 0;
}


/* 
 * enalbe/disable the interrupt function of GPIO[X]
 * 1: interrupt function of corresponding port is enable
 * 0: interrupt function of corresponding port is disable
 */
int g_ak98_gpio_inten(unsigned int pin, unsigned int enable)
{
	void __iomem *base = AK98_GPIO_INTEN_BASE(pin);
	unsigned int offset = ((pin) & 31);
	unsigned long flags;

	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, invalid gpio %d!\n", pin);
		return -1;
	}
	
	local_irq_save(flags);
	if (0 == enable)
		REG32(base) &= ~(1 << offset);
	else
		REG32(base) |= (1 << offset);
	local_irq_restore(flags);
	return 0;
}


/*
 * interrupt polarity selection
 * 1: the input interrupt polarity of GPIO[X] is active high
 * 0: the input interrupt polarity of GPIO[X] is active low
 */
int g_ak98_gpio_intpol(unsigned int pin, unsigned int level)
{
	void __iomem *base = AK98_GPIO_INTPOL_BASE(pin);
	unsigned int offset = ((pin) & 31);
	unsigned long flags;

	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, invalid gpio %d!\n", pin);
		return -1;
	}
	
	local_irq_save(flags);
	if (0 == level)
		REG32(base) |= (1 << offset);
	else
		REG32(base) &= ~(1 << offset);
	local_irq_restore(flags);
	return 0;
}


 int g_ak98_gpio_to_irq(unsigned int pin)
{
	if(!gpio_assert_legal(pin) || (pin > GPIO_UPLIMIT)) {
		panic("Error, invalid gpio %d!\n", pin);
		return -1;
	}
	return (IRQ_GPIO_0 + (pin - AK98_GPIO_0));
}


 int g_ak98_irq_to_gpio(unsigned int irq)
{
	return (AK98_GPIO_0 + (irq - IRQ_GPIO_0));
}


#if 0
static const char *ak98_gpio_list[AK98_GPIO_MAX];

int ak98_gpio_request(unsigned long gpio, const char *label)
{
	if (gpio > GPIO_UPLIMIT)
		return -EINVAL;
	
	if (ak98_gpio_list[gpio])
		return -EBUSY;
	
	if (label)
		ak98_gpio_list[gpio] = label;
	else
		ak98_gpio_list[gpio] = "busy";
	
	return 0;
}
EXPORT_SYMBOL(ak98_gpio_request);

void ak98_gpio_free(unsigned long gpio)
{
	BUG_ON(!ak98_gpio_list[gpio]);
	
	ak98_gpio_list[gpio] = NULL;
}
EXPORT_SYMBOL(ak98_gpio_free);

#else

int ak98_gpio_request(unsigned long gpio, const char *label)
{
	return 0;
}
EXPORT_SYMBOL(ak98_gpio_request);

void ak98_gpio_free(unsigned long gpio)
{
}
EXPORT_SYMBOL(ak98_gpio_free);

#endif

