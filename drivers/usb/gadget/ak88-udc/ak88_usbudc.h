#ifndef __USB_REG_H
#define __USB_REG_H
/** USB controller register*/
#define USB_BASE_ADDR	(0)

#define USB_FIFO_EP0                (USB_BASE_ADDR + 0x0020)

#define USB_REG_FADDR               (USB_BASE_ADDR + 0x0000)
#define USB_REG_POWER               (USB_BASE_ADDR + 0x0001)
#define USB_REG_INTRTX1             (USB_BASE_ADDR + 0x0002)
#define USB_REG_INTRTX2             (USB_BASE_ADDR + 0x0003)
#define USB_REG_INTRRX1             (USB_BASE_ADDR + 0x0004)
#define USB_REG_INTRRX2           	(USB_BASE_ADDR + 0x0005)
#define USB_REG_INTRTX1E          	(USB_BASE_ADDR + 0x0006)
#define USB_REG_INTRTX2E          	(USB_BASE_ADDR + 0x0007)
#define USB_REG_INTRRX1E          	(USB_BASE_ADDR + 0x0008)
#define USB_REG_INTRRX2E          	(USB_BASE_ADDR + 0x0009)
#define USB_REG_INTRUSB           	(USB_BASE_ADDR + 0x000A)
#define USB_REG_INTRUSBE          	(USB_BASE_ADDR + 0x000B)
#define USB_REG_FRAME1            	(USB_BASE_ADDR + 0x000C)
#define USB_REG_FRAME2            	(USB_BASE_ADDR + 0x000D)
#define USB_REG_INDEX             	(USB_BASE_ADDR + 0x000E)
#define USB_REG_TESEMODE          	(USB_BASE_ADDR + 0x000F)
#define USB_REG_DEVCTL            	(USB_BASE_ADDR + 0x0060)
#define USB_REG_TXMAXP0           	(USB_BASE_ADDR + 0x0010)
#define USB_REG_TXMAXP1           	(USB_BASE_ADDR + 0x0010)
#define USB_REG_CSR0              	(USB_BASE_ADDR + 0x0012)
#define USB_REG_TXCSR1            	(USB_BASE_ADDR + 0x0012)
#define USB_REG_CSR02             	(USB_BASE_ADDR + 0x0013)
#define USB_REG_TXCSR2            	(USB_BASE_ADDR + 0x0013)
#define USB_REG_RXMAXP1           	(USB_BASE_ADDR + 0x0014)
#define USB_REG_RXMAXP2           	(USB_BASE_ADDR + 0x0015)
#define USB_REG_RXCSR1            	(USB_BASE_ADDR + 0x0016)
#define USB_REG_RXCSR2            	(USB_BASE_ADDR + 0x0017)
#define USB_REG_COUNT0            	(USB_BASE_ADDR + 0x0018)
#define USB_REG_RXCOUNT1          	(USB_BASE_ADDR + 0x0018)
#define USB_REG_RXCOUNT2          	(USB_BASE_ADDR + 0x0019)
#define USB_REG_TXTYPE            	(USB_BASE_ADDR + 0x001A)
#define USB_REG_RXTYPE            	(USB_BASE_ADDR + 0x001C)
#define USB_REG_RXINTERVAL        	(USB_BASE_ADDR + 0x001D)
#define USB_REG_NAKLIMIT0         	(USB_BASE_ADDR + 0x001B)
					
#define	USB_EP0_TX_COUNT		  	(USB_BASE_ADDR + 0x0330)
#define	USB_EP2_TX_COUNT		  	(USB_BASE_ADDR + 0x0334)
					
#define	USB_FORBID_WRITE_REG	  	(USB_BASE_ADDR + 0x0338)
					
#define	USB_START_PRE_READ_REG	  	(USB_BASE_ADDR + 0x033C)
#define	USB_FS_SPEED_REG		  	(USB_BASE_ADDR + 0x0344)


/**  USB DMA */                   	
// #define USB_DMA_INTR              	(USB_BASE_ADDR + 0x0200)
#define USB_DMA_CNTL_1            	(USB_BASE_ADDR + 0x0204)
#define USB_DMA_ADDR_1            	(USB_BASE_ADDR + 0x0208)
#define USB_DMA_COUNT_1           	(USB_BASE_ADDR + 0x020c)
#define USB_DMA_CNTL_2            	(USB_BASE_ADDR + 0x0214)
#define USB_DMA_ADDR_2            	(USB_BASE_ADDR + 0x0218)
#define USB_DMA_COUNT_2           	(USB_BASE_ADDR + 0x021c)

/* usb control and status register */
#define USB_REG_RXCSR1_RXSTALL    	(1 << 6)
#define USB_REG_RXCSR1_REQPKT     	(1 << 5)

#define USB_TXCSR_AUTOSET          	(0x80)
#define USB_TXCSR_ISO              	(0x40)
#define USB_TXCSR_MODE1            	(0x20)
#define USB_TXCSR_DMAREQENABLE     	(0x10)
#define USB_TXCSR_FRCDATATOG       	(0x8)
#define USB_TXCSR_DMAREQMODE1      	(0x4)
#define USB_TXCSR_DMAREQMODE0      	(0x0)

#endif
