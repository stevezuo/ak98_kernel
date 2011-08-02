/*
 * init cpu freq, clock
 *
 * report cpu id
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/init.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/l2cache.h>
#include <asm/io.h>

#define IODESC_ENT(x) { (unsigned long)AK98_VA_##x, __phys_to_pfn(AK98_PA_##x), AK98_SZ_##x, MT_DEVICE }

static struct map_desc ak98_iodesc[] __initdata = {
	IODESC_ENT(SYSCTRL),
	IODESC_ENT(SUBCTRL),
	IODESC_ENT(L2MEM),
	IODESC_ENT(USB),
};

void __init ak98_map_io(void)
{
	unsigned long idcode = 0x0;

	/* initialise the io descriptors we need for initialisation */
	iotable_init(ak98_iodesc, ARRAY_SIZE(ak98_iodesc));

	idcode = __raw_readl(AK98_VA_SYSCTRL + 0x00);

	if (idcode == 0x20090C00) {
#if defined(CONFIG_AK9801_ATHENA)
		printk("ANYKA CPU %s (id 0x%lx)\n", "AK9801", idcode);
#elif defined(CONFIG_AK9805_TV908)
		printk("ANYKA CPU %s (id 0x%lx)\n", "AK9805", idcode);
#elif defined(CONFIG_AK9805_MP5)
		printk("ANYKA CPU %s (id 0x%lx)\n", "AK9805", idcode);
#else
#error AK98xx Board NOT supported
#endif
	}
	else
		panic("Unknown ANYKA CPU id: 0x%lx\n", idcode);

	ak98_l2cache_init();
	ak98_l2cache_invalidate();
}

