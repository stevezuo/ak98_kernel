obj-y				+= cpu.o irq.o time.o clock.o \
				   l2mem.o dma.o ak98-gpio.o gpio.o \
				   devices.o reg.o dev_reset.o\
				   l2.o l2cache.o rtc.o l2_exebuf.o \
				   adc1.o 
			

obj-$(CONFIG_AK98_PWM)	+= pwm.o

# Power Management support

obj-$(CONFIG_AK98_PM)		+= pm.o 
obj-$(CONFIG_AK98_CPUFREQ)	+= cpufreq.o ddr2change.o

# Machin support
obj-$(CONFIG_AK9801_ATHENA)	+= mach-athena.o
obj-$(CONFIG_AK9805_TV908)	+= mach-tv908.o
obj-$(CONFIG_AK9805_MP5)	+= mach-mp5.o

