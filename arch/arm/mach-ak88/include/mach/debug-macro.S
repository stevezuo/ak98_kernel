/* linux/include/asm-arm/arch-ak880x/debug-macro.S
 *
 * Debugging macro include header
 *
 * Copyright (C) 1994-1999 Russell King
 * Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <mach/map.h>

	.macro	addruart,	rx
	mrc	p15, 0, \rx, c1, c0

	@ init uart for output, no need init everytime
	@ share-pin settting
	@tst	\rx, #1
	@ldreq	r11, =(AK88_PA_SYSCTRL+0x78)
	@ldrne	r11, =(AK88_VA_SYSCTRL+0x78)
	@ldr	r12, [r11]
	@orr	r12, r12, #(0x1<<9)
	@str	r12, [r11]

	@ uart buf setting
	@tst     \rx, #1
	@ldreq   r11, =(AK88_PA_L2CTRL+0x40)    @ mmu disabled
	@ldrne   r11, =(AK88_VA_L2CTRL+0x40)    @ mmu enabled
	@ldr     r12, [r11]
	@orr     r12, r12, #(0x1<<11)
	@str     r12, [r11]
	
	@uart config (ie: baudrate)
	@tst     \rx, #1
	@ldreq   r11, =(AK88_PA_UART+0x00)     @0x20026000
	@ldrne   r11, =(AK88_VA_UART+0x00)
	@ldr     r12, [r11]
	@orr     r12, r12, #(0x1<<21)
	@orr     r12, r12, #(0x3<<28)
	@str     r12, [r11]
	
	@ uart tx threshold setting
	@tst     \rx, #1
	@ldreq   r11, =(AK88_PA_UART+0x0C)     @0x2002600C
	@ldrne   r11, =(AK88_VA_UART+0x0C)
	@mov     r12, #0x0
	@str     r12, [r11]
	.endm

	.macro	senduart,	rb,	rx
	tst	\rx, #1
	ldreq   r11, =AK88_PA_L2CTRL		@0x2002C08C
	ldrne	r11, =(AK88_VA_L2CTRL+0x8C)
	ldr     r12, [r11]
	orr     r12, r12, #0x10000
	str     r12, [r11]

	tst	\rx, #1
	ldreq	r11, =(AK88_PA_L2MEM+0x1000)	@0x48001000
	ldrne	r11, =(AK88_VA_L2MEM+0x1000)
	@ldr    r12, ='z'
	str     \rb, [r11]

	tst	\rx, #1
	ldreq	r11, =(AK88_PA_L2MEM+0x103C)	@0x4800103C
	ldrne	r11, =(AK88_VA_L2MEM+0x103C)
	mov     r12, #0x0
	str     r12, [r11]

	tst	\rx, #1
	ldreq	r11, =(AK88_PA_UART+0x00)	@0x20026000
	ldrne	r11, =(AK88_VA_UART+0x00)
	ldr     r12, [r11]
	orr     r12, r12, #(0x1<<28)
	str     r12, [r11]
	
	tst	\rx, #1
	ldreq	r11, =(AK88_PA_UART+0x04)	@0x20026004
	ldrne	r11, =(AK88_VA_UART+0x04)
	ldr     r12, [r11]
	orr     r12, r12, #(0x1<<4)
	orr     r12, r12, #(0x1<<16)
	str     r12, [r11]

ak_loop:
	tst	\rx, #1
	ldreq	r11, =(AK88_PA_UART+0x04)	@0x20026004
	ldrne	r11, =(AK88_VA_UART+0x04)
	ldr     r12, [r11]
	and     r12, r12, #0x80000
	cmp     r12, #0
	beq     ak_loop			@ FIXME

	ldr     r11, =0x800
ak_delay:
	subs    r11, r11, #1
	cmp     r11, #0
	bne     ak_delay

	.endm
	
	.macro	waituart,	rd,	rx
	.endm
	
	.macro	busyuart,	rd,	rx
	.endm
