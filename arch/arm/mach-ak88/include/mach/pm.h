
/* ak880x_pm_init
 *
 * called from board at initialisation time to setup the power
 * management
*/

#ifdef CONFIG_PM

extern __init int ak880x_pm_init(void);

#else

static inline int ak880x_pm_init(void)
{
	return 0;
}
#endif

/* configuration for the IRQ mask over sleep */

/* IRQ masks for IRQs allowed to go to sleep (see irq.c) */

/* per-cpu sleep functions */

extern void (*pm_cpu_prep) (void);
extern void (*pm_cpu_sleep) (void);

/* Flags for PM Control */

extern unsigned long ak880x_pm_flags;

/* from sleep.S */

extern int ak880x_cpu_save(unsigned long *saveblk);
extern void ak880x_cpu_suspend(void);
extern void ak880x_cpu_resume(void);

extern unsigned long ak880x_sleep_save_phys;

/* sleep save info */

struct sleep_save {
	void __iomem *reg;
	unsigned long val;
};

#define SAVE_ITEM(x) \
	{ .reg = (x) }

extern void ak880x_pm_do_save(struct sleep_save *ptr, int count);
extern void ak880x_pm_do_restore(struct sleep_save *ptr, int count);
