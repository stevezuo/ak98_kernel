/*
 * arch/arm/mach-ak98/l2_exebuf.c
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <mach/regs-l2.h>
#include <mach/l2_exebuf.h>

// #define PM_DEBUG
#define L2_BUFFER0_SIZE		512

void (*jumpto_L2)(unsigned long param1,unsigned long param2,
					unsigned long param3,unsigned long param4);

#ifdef PM_DEBUG
static void pm_printf(const char *start, int len)
{
	int i;
	unsigned char local_l2mem[L2_BUFFER0_SIZE] = {0} ;

	printk("start = %p, len = %d\n", start, len);
	for (i = 0; i < len; i += 4) {
		*(unsigned long *)(local_l2mem + i) = 
			*(volatile unsigned long *)(AK98_VA_L2MEM + i);
	}
	for (i = 0; i < len; i++) {
		printk(" 0x%02x", local_l2mem[i]);	
		if (i % 16 == 15) printk("\n");
	}
}
#endif

/*
 * copy from ddr2 to l2 memory to run, and exit standby
 */
int l2_exec_buf(const char *vaddr, int len, unsigned long param1,
unsigned long param2,unsigned long param3,unsigned long param4)
{
	unsigned long i, flags ;

    //disable ARM interrupt
    local_irq_save(flags);

	memset((void *)AK98_VA_L2MEM, 0, L2_BUFFER0_SIZE);
	
	//copy from ddr2 to l2 memory
	for (i = 0; i < len; i += 4) {
		*(volatile unsigned long *)(AK98_VA_L2MEM + i) = 
			*(unsigned long *)(vaddr + i);
	}
#ifdef PM_DEBUG
	pm_printf(vaddr, len);
#endif

	REG32(AK98_VA_L2CTRL + 0x84) &= ~(1 << 29);

	//jumpto_L2 run
	jumpto_L2 = (void *)(AK98_VA_L2MEM);
	jumpto_L2(param1,param2,param3,param4);

	REG32(AK98_VA_L2CTRL + 0x84) |= (1 << 29);
	
	//enable ARM interrupt
	local_irq_restore(flags);
	return 0;
}


