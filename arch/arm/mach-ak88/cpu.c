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
#include <asm/io.h>

#define IODESC_ENT(x) { (unsigned long)AK88_VA_##x, __phys_to_pfn(AK88_PA_##x), AK88_SZ_##x, MT_DEVICE }

static struct map_desc ak880x_iodesc[] __initdata = {
	IODESC_ENT(SYSCTRL),
	IODESC_ENT(SUBCTRL),
	IODESC_ENT(L2MEM),
};

void __init ak880x_map_io(void)
{
	unsigned long idcode = 0x0;

	/* initialise the io descriptors we need for initialisation */
	iotable_init(ak880x_iodesc, ARRAY_SIZE(ak880x_iodesc));

	idcode = __raw_readl(AK88_VA_SYSCTRL + 0x00);

	if (idcode == 0x33323236)
#if	defined(CONFIG_BOARD_AK8801EPC)
		printk("ANYKA CPU %s (id 0x%lx)\n", "AK7801", idcode);
#elif	defined(CONFIG_BOARD_AK8802EBOOK)
		printk("ANYKA CPU %s (id 0x%lx)\n", "AK88", idcode);
#endif
	else
		panic("Unknown ANYKA CPU id: 0x%lx\n", idcode);
}
