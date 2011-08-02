#ifndef _MACH_DEVICES_H
#define _MACH_DEVICES_H


extern struct platform_device ak98_led_device;

extern struct platform_device ak98_rtc_device;

extern struct platform_device ak98_pwm0_device;
extern struct platform_device ak98_pwm1_device;
extern struct platform_device ak98_pwm2_device;
extern struct platform_device ak98_pwm3_device;

extern struct platform_device ak98pwm_backlight_device;

extern struct platform_device ak98_uart0_device;
extern struct platform_device ak98_uart1_device;
extern struct platform_device ak98_uart2_device;
extern struct platform_device ak98_uart3_device;

extern struct platform_device ak98_battery_power;
extern struct platform_device ak98_freq_policy_device;

extern struct platform_device ak98_ts_device;
extern struct platform_device ak98adc_ts_device;

extern struct platform_device ak98_gpio_trkball_device;
extern struct platform_device ak98_kpd_device;

extern struct platform_device ak98_sdio_device;

extern struct platform_device ak98_snd_device;

extern struct platform_device ak98_spi1_device;
extern struct platform_device ak98_spi2_device;

extern struct platform_device ak98_lcd_device;
extern struct platform_device ak98_osd_device;
extern struct platform_device ak98_tvout_device;

extern struct platform_device ak98_mmx_device;
extern struct platform_device ak98_mmx_pmem;

extern struct platform_device ak98_nand_device;
extern struct platform_device ak98_i2c_device;

extern struct platform_device ak98_mmc_device;

extern struct platform_device ak98_usb_device;

extern struct platform_device ak98_mac_device;

extern struct platform_device ak98_usb_fs_hcd_device;

extern struct platform_device ak98_android_usb;
extern struct platform_device ak98_android_usb_mass_storage;

extern struct platform_device sensor_info_device;

extern struct platform_device ak98_camera_interface;

#endif
