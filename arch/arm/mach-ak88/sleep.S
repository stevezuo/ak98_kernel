/*
 * linux/arch/arm/mach-ak7801/sleep.S 
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/hardware.h>
#include <asm/arch/map.h>

@@	Turn On kpd led with mmu on (TA801 DGPIO36)
@	ldr	r0, =(AK7801_VA_SYSCTRL+0x98)
@	ldr	r1, [r0]
@	orr	r1, r1, #(1<<5)
@	str	r1, [r0]
@@

@@	Turn On kpd led with mmu off
@	mov	r0, #0x08000000
@	ldr	r1, [r0, #0x98]
@	orr	r1, r1, #(1<<5)
@	str	r1, [r0, #0x98]


	/* Put AK7801 into stanby mode */

ENTRY(ak7801_cpu_save)
	stmfd   sp!, { r4 - r12, lr }

	@@ store co-processor registers

        mrc     p15, 0, r4, c13, c0, 0  @ PID
        mrc     p15, 0, r5, c3, c0, 0   @ Domain ID
        mrc     p15, 0, r6, c2, c0, 0   @ translation table base address
        mrc     p15, 0, r7, c1, c0, 0   @ control register

        stmia   r0, { r4 - r13 }

        mov     r0, #0
        ldmfd   sp, { r4 - r12, pc }

        @@ return to the caller, after having the MMU
        @@ turned on, this restores the last bits from the
        @@ stack
resume_with_mmu:
        mov     r0, #1
        ldmfd   sp!, { r4 - r12, pc }

        .ltorg


ENTRY(ak7801_cpu_sleep)

	mov     r0, #0
        mcr     p15, 0, r0, c7, c10, 4          @ drain write buffer

	@@ Turn off MMU
	mrc     p15, 0, r0, c1, c0, 0 
        bic     r0, r0, #0x0004                 @ Disable D cache
        bic     r0, r0, #0x0001                 @ Disable MMU
        bic     r0, r0, #0x0100                 @ Disable MMU S-bit
        mcr     p15, 0, r0, c1, c0, 0           @ The above three brought in effect
	nop

	@@ Jump To L2 mem

	mov	pc, #0x48000000
	
	nop

	.align 5

        .data
	
        .global ak7801_sleep_save_phys
ak7801_sleep_save_phys:
        .word   0

ENTRY(ak7801_cpu_resume)

	mov     r1, #0
        mcr     p15, 0, r1, c8, c7, 0           @@ invalidate I & D TLBs
        mcr     p15, 0, r1, c7, c7, 0           @@ invalidate I & D caches

	ldr     r0, ak7801_sleep_save_phys     @ address of restore block
        ldmia   r0, { r4 - r13 }

	mcr     p15, 0, r4, c13, c0, 0          @ PID
        mcr     p15, 0, r5, c3, c0, 0           @ Domain ID
        mcr     p15, 0, r6, c2, c0, 0           @ translation table base
	
	ldr     r2, =resume_with_mmu
        mcr     p15, 0, r7, c1, c0, 0           @ turn on MMU, etc
        nop                                     @ second-to-last before mmu
        mov     pc, r2                          @ go back to virtual address

	.ltorg
