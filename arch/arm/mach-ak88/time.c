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

#define	AK88_TIMER1_CTRL	(AK88_VA_SYSCTRL+0x18)
#define	AK88_TIMER2_CTRL	(AK88_VA_SYSCTRL+0x1C)

#define AK88_SYSCTRL_IRQ_CTRL	(AK88_VA_SYSCTRL+0x4C)
#define	AK88_IRQ_MASK		(AK88_VA_SYSCTRL+0x34)

#define TIMER_ENABLE		(1<<26)
#define TIMER_LOAD_NEWCNT	(1<<27)
#define TIMER_INT_CLR		(1<<28)
#define TIMER_INT_STA		(1<<29)
#define TIMER_CNT		(12000000/HZ)
#define TIMER_CNT_MASK		(0x3F<<26)

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
static unsigned long ak880x_gettimeoffset(void)
{
	unsigned long tval;
	unsigned long tdone;
	unsigned long tcnt;

	/* work out how many ticks have gone since last timer interrupt */

	tval = __raw_readl(AK88_TIMER1_CTRL);

	tcnt = tval & ~TIMER_CNT_MASK;

	tdone = timer_startval - tcnt;

	if (tval & TIMER_INT_STA) {	/* Timer1 has generated interrupt, and not clear */

		/* Reread timer counter */
		tval = __raw_readl(AK88_TIMER1_CTRL);
		tcnt = tval & ~TIMER_CNT_MASK;

		tdone = timer_startval - tcnt;

		if (tcnt != 0)
			tdone += timer_startval;
	}

	return timer_ticks_to_usec(tdone);
}

static void ak880x_timer_setup(unsigned long timecnt)
{
	unsigned long tctrl;

	/* setting timer1 ctrl */
#if 0
	__raw_writel(TIMER_INT_CLR, AK88_TIMER1_CTRL);
	__raw_writel(timecnt & ~TIMER_CNT_MASK, AK88_TIMER1_CTRL);

	tctrl = __raw_readl(AK88_TIMER1_CTRL);
	tctrl |= (TIMER_LOAD_NEWCNT | TIMER_ENABLE);
	__raw_writel(tctrl, AK88_TIMER1_CTRL);
#else				/* reload */
	*(volatile unsigned int *)(AK88_TIMER1_CTRL) |=
	    TIMER_INT_CLR | TIMER_ENABLE;
#endif
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t ak880x_timer_interrupt(int irq, void *dev_id)
{
	/* printk("%s\n", __FUNCTION__); */

	if (__raw_readl(AK88_TIMER1_CTRL) & TIMER_INT_STA) {

		timer_tick();

		ak880x_timer_setup(timer_startval);
	}

	return IRQ_HANDLED;
}

static struct irqaction ak880x_timer_irq = {
	.name = "timer tick",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = ak880x_timer_interrupt,
};

static void __init ak880x_timer_init(void)
{
#if 0
	timer_startval = TIMER_CNT;

	/* setting timer1 ctrl */
	ak880x_timer_setup(timer_startval);
#else
	unsigned long tctrl;
	unsigned long timecnt = TIMER_CNT;
	__raw_writel(TIMER_INT_CLR, AK88_TIMER1_CTRL);
	__raw_writel(timecnt & ~TIMER_CNT_MASK, AK88_TIMER1_CTRL);

	tctrl = __raw_readl(AK88_TIMER1_CTRL);
	tctrl |= (TIMER_LOAD_NEWCNT | TIMER_ENABLE);
	__raw_writel(tctrl, AK88_TIMER1_CTRL);
#endif

	/* setup irq handler for IRQ_TIMER */
	setup_irq(IRQ_TIMER1, &ak880x_timer_irq);
}

struct sys_timer ak880x_timer = {
	.init = ak880x_timer_init,
	.offset = ak880x_gettimeoffset,
//      .resume         = ak880x_timer_setup
};
