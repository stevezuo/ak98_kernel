/**
 * @file op_model_ak98.c
 * ak98 Performance Monitor Driver
 *
 * Based on op_model_v6.c
 *
 * @remark Copyright 2000-2004 Deepak Saxena <dsaxena@mvista.com>
 * @remark Copyright 2000-2004 MontaVista Software Inc
 * @remark Copyright 2004 Dave Jiang <dave.jiang@intel.com>
 * @remark Copyright 2004 Intel Corporation
 * @remark Copyright 2004 Zwane Mwaikambo <zwane@arm.linux.org.uk>
 * @remark Copyright 2004 OProfile Authors
 *
 * @remark Read the file COPYING
 *
 * @author Oxygen 
 */

/* #define DEBUG */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "op_counter.h"
#include "op_arm_model.h"

#include <asm/io.h>
#include <asm/mach/time.h>
#include <mach/map.h>
#include <mach/clock.h>

#define	AK98_TIMER2_CTRL	(AK98_VA_SYSCTRL+0x1C)

#define AK98_SYSCTRL_IRQ_CTRL	(AK98_VA_SYSCTRL+0x4C)
#define	AK98_IRQ_MASK		(AK98_VA_SYSCTRL+0x34)

#define TIMER_ENABLE		(1<<26)
#define TIMER_LOAD_NEWCNT	(1<<27)
#define TIMER_INT_CLR		(1<<28)
#define TIMER_INT_STA		(1<<29)
#define TIMER_CNT_MASK		(0x3F<<26)


/*
 * CPU counters' IRQ handler (one IRQ per CPU)
 */
static irqreturn_t ak98_timer_interrupt(int irq, void *arg)
{
	struct pt_regs *regs = get_irq_regs();
	
	oprofile_add_sample(regs, 0);//CCNT
	
	/* Clear counter flag(s) */
	*(volatile unsigned int *)(AK98_TIMER2_CTRL) |= TIMER_INT_CLR | TIMER_ENABLE;
	return IRQ_HANDLED;
}

static struct irqaction ak98_timer_irq = {
	.name = "oprofile cnt",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = ak98_timer_interrupt,
};


static void ak98_pmu_stop(void)
{
    //stop timer
    __raw_writel(TIMER_INT_CLR, AK98_TIMER2_CTRL);
	//release_irq
    remove_irq(IRQ_TIMER2, &ak98_timer_irq);    
}

static int ak98_pmu_start(void)
{	
    unsigned long tctrl;
    unsigned long timecnt = counter_config[0].count * 12 / (ak98_get_cpu_clk() / MHz);
    
    /* setup irq handler for IRQ_TIMER */
	setup_irq(IRQ_TIMER2, &ak98_timer_irq);

    __raw_writel(TIMER_INT_CLR, AK98_TIMER2_CTRL);
    __raw_writel(timecnt & ~TIMER_CNT_MASK, AK98_TIMER2_CTRL);

    tctrl = __raw_readl(AK98_TIMER2_CTRL);
    tctrl |= (TIMER_LOAD_NEWCNT | TIMER_ENABLE);
    __raw_writel(tctrl, AK98_TIMER2_CTRL);
    
	return 0;
}

static int ak98_detect_pmu(void)
{
	return 0;
}

static int ak98_setup_pmu(void)
{
    return 0;
}

struct op_arm_model_spec op_ak98_spec = {
	.init		= ak98_detect_pmu,
	.num_counters	= 1,
	.setup_ctrs	= ak98_setup_pmu,
	.start		= ak98_pmu_start,
	.stop		= ak98_pmu_stop,
	.name		= "timer",
};
