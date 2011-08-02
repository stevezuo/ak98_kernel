/*
 * include/asm-arm/arch-ak98/entry-macro.S
 *
 * Low-level IRQ helper macros for AK98-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
*/

#include <mach/hardware.h>
#include <asm/irq.h>

#define	AK98_IRQ_INTMASK		(AK98_VA_SYSCTRL + 0x34)
#define	AK98_FIQ_INTMASK		(AK98_VA_SYSCTRL + 0x38)
#define	AK98_INT_STATUS			(AK98_VA_SYSCTRL + 0xCC)

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

		ldr	\base, =AK98_INT_STATUS
		ldr	\irqstat, [\base]		@ get interrupts status
		teq	\irqstat, #0x0
		beq	1002f
		
		ldr	\base, =AK98_IRQ_INTMASK	@ get interrupts mask
		ldr	\base, [\base]
		and	\irqstat, \irqstat, \base

		tst	\irqstat, #(1<<IRQ_H264_DECODER)
		bicne	\irqstat, \irqstat, #(1<<IRQ_H264_DECODER)
		movne	\irqnr, #IRQ_H264_DECODER
		bne	1001f

		tst	\irqstat, #(1<<IRQ_DISPLAY_CTRL)
		bicne	\irqstat, \irqstat, #(1<<IRQ_DISPLAY_CTRL)
		movne	\irqnr, #IRQ_DISPLAY_CTRL
		bne	1001f

		tst	\irqstat, #(1<<IRQ_CAMERA_IF)
		bicne	\irqstat, \irqstat, #(1<<IRQ_CAMERA_IF)
		movne	\irqnr, #IRQ_CAMERA_IF
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MOTIONESTIMATION)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MOTIONESTIMATION)
		movne	\irqnr, #IRQ_MOTIONESTIMATION
		bne	1001f

		tst	\irqstat, #(1<<IRQ_JPEG)
		bicne	\irqstat, \irqstat, #(1<<IRQ_JPEG)
		movne	\irqnr, #IRQ_JPEG
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MAC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MAC)
		movne	\irqnr, #IRQ_MAC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_ROTATION)
		bicne	\irqstat, \irqstat, #(1<<IRQ_ROTATION)
		movne	\irqnr, #IRQ_ROTATION
		bne	1001f

		tst	\irqstat, #(1<<IRQ_DAC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_DAC)
		movne	\irqnr, #IRQ_DAC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SIGDELTA_ADC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SIGDELTA_ADC)
		movne	\irqnr, #IRQ_SIGDELTA_ADC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_L2MEM)
		bicne	\irqstat, \irqstat, #(1<<IRQ_L2MEM)
		movne	\irqnr, #IRQ_L2MEM
		bne	1001f

		tst	\irqstat, #(1<<IRQ_NF_ECC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_NF_ECC)
		movne	\irqnr, #IRQ_NF_ECC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_NF_CTRL)
		bicne	\irqstat, \irqstat, #(1<<IRQ_NF_CTRL)
		movne	\irqnr, #IRQ_NF_CTRL
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART3)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART3)
		movne	\irqnr, #IRQ_UART3
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART2)
		movne	\irqnr, #IRQ_UART2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART1)
		movne	\irqnr, #IRQ_UART1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART0)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART0)
		movne	\irqnr, #IRQ_UART0
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SPI2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SPI2)
		movne	\irqnr, #IRQ_SPI2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SPI1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SPI1)
		movne	\irqnr, #IRQ_SPI1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_I2C)
		bicne	\irqstat, \irqstat, #(1<<IRQ_I2C)
		movne	\irqnr, #IRQ_I2C
		bne	1001f

		tst	\irqstat, #(1<<IRQ_2D_ACC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_2D_ACC)
		movne	\irqnr, #IRQ_2D_ACC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SDIO)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SDIO)
		movne	\irqnr, #IRQ_SDIO
		bne	1001f
	
		tst	\irqstat, #(1<<IRQ_MMC_SD)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MMC_SD)
		movne	\irqnr, #IRQ_MMC_SD
		bne	1001f
		
		tst	\irqstat, #(1<<IRQ_USBHOST_MCU)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBHOST_MCU)
		movne	\irqnr, #IRQ_USBHOST_MCU
		bne	1001f
	
		tst	\irqstat, #(1<<IRQ_USBHOST_DMA)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBHOST_DMA)
		movne	\irqnr, #IRQ_USBHOST_DMA
		bne	1001f
	
		tst	\irqstat, #(1<<IRQ_USBOTG_MCU)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBOTG_MCU)
		movne	\irqnr, #IRQ_USBOTG_MCU
		bne	1001f
	
		tst	\irqstat, #(1<<IRQ_USBOTG_DMA)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBOTG_DMA)
		movne	\irqnr, #IRQ_USBOTG_DMA
		bne	1001f
	
		tst	\irqstat, #(1<<IRQ_SYSCTRL)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SYSCTRL)
		movne	\irqnr, #IRQ_SYSCTRL
		bne	1001f

		tst	\irqstat, #(1<<IRQ_RMVB)
		bicne	\irqstat, \irqstat, #(1<<IRQ_RMVB)
		movne	\irqnr, #IRQ_RMVB
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MPEG2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MPEG2)
		movne	\irqnr, #IRQ_MPEG2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_HUFFMAN)
		bicne	\irqstat, \irqstat, #(1<<IRQ_HUFFMAN)
		movne	\irqnr, #IRQ_HUFFMAN
		bne	1001f

		tst	\irqstat, #(1<<IRQ_PCM)
		bicne	\irqstat, \irqstat, #(1<<IRQ_PCM)
		movne	\irqnr, #IRQ_PCM
		bne	1001f

	1001:
		@ got irqnr
	1002:
		@ exit here
	.endm
	
	/* currently don't need an disable_fiq macro */
	.macro	disable_fiq
	.endm

