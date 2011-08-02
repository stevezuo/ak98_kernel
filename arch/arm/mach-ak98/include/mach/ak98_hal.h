#ifndef AK98_HAL_H
#define AK98_HAL_H

#include <mach/gpio.h>
#include <linux/delay.h>

typedef struct {
    void    *pAddress0800;   // @field Start of 0x08000000 - mapped
    void    *pAddress2002E;   // @field Start of 0x2002E000 - mapped
    void    *pAddress2002D;   //@field Start of 0x2002D000 - mapped
}REG_ADDR, *PREG_ADDR;

//pAddress0800   0x08000000-0x080000FF
#define CLK_DIV_REG2                  0x0008
#define CLOCK_CTRL_REG1               0x000C
#define MULTIPLE_FUN_CTRL_REG1        0x0058
#define ANALOG_CTRL_REG3              0x005C
#define ANALOG_CTRL_REG4              0x0064
#define ANALOG_CTRL_REG1_WRITE        0x012C
#define ANALOG_CTRL_REG2_WRITE        0x0130
#define ANALOG_CTRL_REG1_READ         0x0130
#define ANALOG_CTRL_REG2_READ         0x0134

//pAddress2002E  0x2002E000-0x2002E00F
#define DAC_CONFIG_REG                0x0000
#define I2S_CONFIG_REG                0x0004
#define CPU_DATA_REG                  0x0008

//pAddress2002D 0x2002D000-0x2002D00F
#define ADC2MODE_CFG_REG              0x0000
#define WORD_LENGTH_MASK (0XF << 8)
#define I2S_EN           (1 << 4)
#define CH_POLARITY_SEL  (1 << 3)
#define HOST_RD_INT_EN   (1 << 2)
#define ADC2MODE_L2_EN   (1 << 1)
#define ADC2_CTRL_EN     (1 <<0)

//CLK_DIV_REG2(0x08000008)
#define DAC_GATE        (1 << 26)
#define ADC2_GATE       (1 << 25)
#define DAC_RST         (1 << 24)
#define ADC2_RST        (1 << 23)
#define DAC_CLK_EN      (1 << 21)
#define MASK_CLKDIV2_DAC_DIV     (0xFF << 13)
#define CLKDIV2_DAC_DIV(val)     (((val)&0xFF) << 13)
#define ADC2_CLK_EN      (1 << 12)
#define MASK_CLKDIV2_ADC2_DIV    (0xFF << 4)
#define CLKDIV2_ADC2_DIV(val)    (((val)&0xFF) << 4)
#define ADC2_DIV         4


//CLOCK_CTRL_REG1(0x0800000C)
#define DAC_SOFT_RST      (1 << 17)
#define ADC2Ctrl_SOFT_RST (1 << 16)
#define L2_CLK_CTRL_EN   (1 << 3)
#define PCM_CLK_CTRL_EN  (1 << 2)
#define DAC_CLK_CTRL_EN  (1 << 1)
#define ADC2_CLK_CTRL_EN (1 << 0)

//MULTIPLE_FUN_CTRL_REG1(0x08000058)
#define IN_DAAD_EN      (1 << 23) //ENABLE INTERNAL DAC ADC via i2s

//ANALOG_CTRL_REG3(0x0800005C)
#define PD_DAOUTL       (1 << 11)
#define PD_DAOUTR       (1 << 10)
#define MICIN_BYPASS    (1 << 9)
#define LINEIN_BYPASS   (1 << 8)

//ANALOG_CTRL_REG4(0x08000064)
#define ANALOG_CTRL4_OSR_MASK       (0x7 << 14)
#define ANALOG_CTRL4_OSR(value)     (((value)&0x7) << 14)
#define DAC_EN                      (1 << 13)
#define ADC_OSR                     12

//ANALOG_CTRL_REG1(read(0x08000130)/write(0x0800012c))
#define MUXDA_R        (1 << 31)
#define MUXDA_L        (1 << 30)
#define PD_MICP        (1 << 22)
#define PD_MICN        (1 << 21)
#define PD2_HP         (1 << 20)
#define PD1_HP         (1 << 19)
#define PRE_EN1        (1 << 18)
#define PRE_EN2        (1 << 17)
#define TRIMP_HP       (1 << 16)
#define TRIMN_HP       (1 << 15)
#define PD_OP          (1 << 11)
#define PD_CK          (1 << 10)
#define PTM_D_CHG_EN   (1 << 5)
#define VREF_CHG_FAST  (1 << 4)
#define PD_VCM3        (1 << 3)
#define PL_VCM2        (1 << 2)  //pull down to ground.
#define PD_VCM2        (1 << 1)  // power off 
#define PD_REF         (1 << 0)

//ANALOG_CTRL_REG2(read(0x08000134)/write(0x08000130))
#define PD_ADC2          (1 << 0)
#define PD_ADC3          (1 << 1)

//DAC_CONFIG_REG(0x2002E000)
#define ARM_INT        (1 << 3)  //ARM interrupt enable
#define MUTE           (1 << 2)  // repeat to sent the Last data to DAC
#define FORMAT         (1 << 4)    //  1 is used memeory saving format.
#define L2_EN          (1 << 1)
#define DAC_CTRL_EN    (1 << 0)

//I2S_CONFIG_REG(0x2002E004)
#define LR_CLK          (1 << 6)
#define POLARITY_SEL    (1 << 5)  
#define I2S_CONFIG_WORDLENGTH_MASK              (0x1F << 0)

/////////////HP_IN  ADC23_IN
#define SOURCE_DAC           (0b001)
#define SOURCE_LINEIN        (0b010)
#define SOURCE_MIC           (0b100)
#define SIGNAL_SRC_MUTE      0
#define SIGNAL_SRC_MAX       (SOURCE_DAC|SOURCE_LINEIN|SOURCE_MIC)


#define HEADPHONE_GAIN_MIN    0
#define HEADPHONE_GAIN_MAX    8
#define LINEIN_GAIN_MIN       0
#define LINEIN_GAIN_MAX       15
#define MIC_GAIN_MIN          0
#define MIC_GAIN_MAX          7

struct ak98pcm_platform_data
{
	struct gpio_info hpdet_gpio;
	struct gpio_info spk_down_gpio;
	struct gpio_info hpmute_gpio;
	int hp_on_value;
	int hpdet_irq;
	int bIsHPmuteUsed;
	int hp_mute_enable_value;
	int bIsMetalfixed;
};

void AK98_DAC_Open(void);
void AK98_DAC_Close(void);
void AK98_DAC_Set_SampleRate(unsigned long samplerate);
void AK98_DAC_Set_Channels(unsigned long chnl);
void AK98_Poweron_HP(bool bOn);
void AK98_Set_HPGain(unsigned long gain);
void AK98_Set_HP_In(unsigned long signal);
void AK98_ADC23_Open(void);
void AK98_ADC23_Close(void);
void AK98_ADC23_Set_SampleRate(unsigned long samplerate);
void AK98_Set_MicGain(unsigned long gain);
void AK98_Set_ADC23_In(unsigned long signal);
void AK98_ADC23_Set_Channels(unsigned long chnl);
void AK98_Set_LineinGain(unsigned long gain);
void AK98_Linein_PowerOn(bool bOn);
void AK98_Mic_PowerOn(bool bOn);
void AK98_DAC_Bypass(bool bTrue);
void AK98_MicIn_Bypass(bool bTrue);
void AK98_LineIn_Bypass(bool bTrue);
void AK98_Set_Bypass(unsigned long signal);
void AK98_Poweron_VCM_REF(bool bOn);
void AK98_Set_SrcPower(int src,int addr,int *CurSrc);
void AK98_Poweron_Speaker(unsigned int pin, bool bOn);

#endif
