#ifndef	_AK88_ADC_H_
#define _AK88_ADC_H_ __FILE__

#define AK88_ANALOG_CTRL1	(AK88_VA_SYSCTRL+0x5C)
#define AD5_sel1		(1 << 29)
#define BAT_ON			(1 << 28)
#define RM_DIR			(1 << 27)
#define PD_TS			(1 << 26)

#define AK88_ADC1_CTRL	(AK88_VA_SYSCTRL+0x60)

#define AK88_ANALOG_CTRL2	(AK88_VA_SYSCTRL+0x64)
#define ADC1_en			(1 << 8)
#define bat_en			(1 << 9)
#define TS_en			(1 << 10)
#define AD5_sel			(1 << 11)
#define TS_threshold08		(0x08 << 17)
#define TS_threshold1023	(0x3ff << 17)
#define TS_ctrl255		(0xff << 0)
#define TS_ctrl05		(0x05 << 0)

#define AK88_ADC1_STATUS	(AK88_VA_SYSCTRL+0x70)
#define YN_int			(1 << 23)
#define YP_int			(1 << 22)
#define XN_int			(1 << 21)
#define XP_int			(1 << 20)

#define AK88_CLK_DIV2		(AK88_VA_SYSCTRL+0x08)
#define ADC1_pd			(1 << 29)
#define ADC1_rst		(1 << 22)
#define ADC1_CLK_en		(1 << 3)
#define ADC1_DIV0		(0 << 0)	/* ADC CLOCK=12M/(ADC1_DIV+1) */
#define ADC1_DIV1		(1 << 0)
#define ADC1_DIV2		(2 << 0)
#define ADC1_DIV3		(3 << 0)

#define AK88_X_COORDINATE	(AK88_VA_SYSCTRL+0x68)
#define AK88_Y_COORDINATE	(AK88_VA_SYSCTRL+0x6C)

#endif
