/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/io.h>
#if 0
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#endif
#include <asm/mach/time.h>

#include <mach/map.h>

#define	AK98_TIMER1_CTRL	(AK98_VA_SYSCTRL+0x18)
#define	AK98_TIMER1_READ	(AK98_VA_SYSCTRL+0x100)
#define	AK98_TIMER2_CTRL	(AK98_VA_SYSCTRL+0x1C)

#define AK98_SYSCTRL_IRQ_CTRL	(AK98_VA_SYSCTRL+0x4C)
#define	AK98_IRQ_MASK		(AK98_VA_SYSCTRL+0x34)

#define TIMER_ENABLE		(1<<26)
#define TIMER_LOAD_NEWCNT	(1<<27)
#define TIMER_INT_CLR		(1<<28)
#define TIMER_INT_STA		(1<<29)
#define TIMER_CNT		(12000000/HZ)
#define TIMER_CNT_MASK		(0x3F<<26)

static u_int64_t ghrtick = 0;
static unsigned long timer_startval;

static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	return (ticks * 1000 / 12000000);
}

/*
 * Returns microsecond  since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 * IRQs are disabled before entering here from do_gettimeofday()
 *
 * FIXME: this need be checked 
 */
static unsigned long ak98_gettimeoffset(void)
{
	unsigned long tval;
	unsigned long tdone;
	unsigned long tcnt;

	/* work out how many ticks have gone since last timer interrupt */

	tval = __raw_readl(AK98_TIMER1_CTRL);

	tcnt = tval & ~TIMER_CNT_MASK;

	tdone = timer_startval - tcnt;

	if (tval & TIMER_INT_STA) {	/* Timer1 has generated interrupt, and not clear */

		/* Reread timer counter */
		tval = __raw_readl(AK98_TIMER1_CTRL);
		tcnt = tval & ~TIMER_CNT_MASK;

		tdone = timer_startval - tcnt;

		if (tcnt != 0)
			tdone += timer_startval;
	}

	return timer_ticks_to_usec(tdone);
}

static void ak98_timer_setup(unsigned long timecnt)
{
	unsigned long tctrl;

	/* setting timer1 ctrl */
#if 0
	__raw_writel(TIMER_INT_CLR, AK98_TIMER1_CTRL);
	__raw_writel(timecnt & ~TIMER_CNT_MASK, AK98_TIMER1_CTRL);

	tctrl = __raw_readl(AK98_TIMER1_CTRL);
	tctrl |= (TIMER_LOAD_NEWCNT | TIMER_ENABLE);
	__raw_writel(tctrl, AK98_TIMER1_CTRL);
#else				/* reload */
	*(volatile unsigned int *)(AK98_TIMER1_CTRL) |=
	    TIMER_INT_CLR | TIMER_ENABLE;
#endif
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t ak98_timer_interrupt(int irq, void *dev_id)
{
	/* printk("%s\n", __FUNCTION__); */

	if (__raw_readl(AK98_TIMER1_CTRL) & TIMER_INT_STA) {

        ghrtick += TIMER_CNT;

		timer_tick();

		ak98_timer_setup(timer_startval);
	}

	return IRQ_HANDLED;
}

#if 1
u_int64_t ak98_gethrtick(void)
{
    unsigned long timecnt = 0;
    
    timecnt = __raw_readl(AK98_TIMER1_READ);
    
    timecnt &= (~TIMER_CNT_MASK);
    
    return (ghrtick + (u_int64_t)(TIMER_CNT-timecnt));
}

unsigned long ak98_gettimeofcycle(void)
{
    unsigned long timecnt = 0;

    timecnt = __raw_readl(AK98_TIMER1_READ);

    timecnt &= (~TIMER_CNT_MASK);

    return (TIMER_CNT - timecnt) / 12;
}
#endif

static struct irqaction ak98_timer_irq = {
	.name = "timer tick",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = ak98_timer_interrupt,
};

static void __init ak98_timer_init(void)
{
#if 0
	timer_startval = TIMER_CNT;

	/* setting timer1 ctrl */
	ak98_timer_setup(timer_startval);
#else
	unsigned long tctrl;
	unsigned long timecnt = TIMER_CNT;
	__raw_writel(TIMER_INT_CLR, AK98_TIMER1_CTRL);
	__raw_writel(timecnt & ~TIMER_CNT_MASK, AK98_TIMER1_CTRL);

	tctrl = __raw_readl(AK98_TIMER1_CTRL);
	tctrl |= (TIMER_LOAD_NEWCNT | TIMER_ENABLE);
	__raw_writel(tctrl, AK98_TIMER1_CTRL);
#endif

	/* setup irq handler for IRQ_TIMER */
	setup_irq(IRQ_TIMER1, &ak98_timer_irq);
    ghrtick = 0;
}

struct sys_timer ak98_timer = {
	.init = ak98_timer_init,
	.offset = ak98_gettimeoffset,
//      .resume         = ak98_timer_setup
};
