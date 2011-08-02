/*
 * support l2 dma
 * 09-09-10 16:46:47
 */

#include <mach/ak880x_addr.h>

/* #define L2_BASE_ADDR	0x2002c000	//L2 PA*/
#define L2_BASE_ADDR	AK88_VA_L2CTRL	//L2 VA
#define L2_BUF_MEM_BASE_ADDR	    	0x48000000  //L2 Buffer start address
#define	L2_DMA_ADDR	AK88_L2CTRL_REG(0x00)
#define	L2_DMA_CNT	AK88_L2CTRL_REG(0x40)
#define	L2_DMA_REQ	AK88_L2CTRL_REG(0x80)
#define	L2_FRAC_ADDR	AK88_L2CTRL_REG(0x84)
#define	L2_COMBUF_CFG	AK88_L2CTRL_REG(0x88)
#define	L2_UARTBUF_CFG	AK88_L2CTRL_REG(0x8c)
#define	L2_ASSIGN_REG1	AK88_L2CTRL_REG(0x90)
#define	L2_ASSIGN_REG2	AK88_L2CTRL_REG(0x94)
#define	L2_LDMA_CFG	AK88_L2CTRL_REG(0x98)
#define	L2_INT_ENA	AK88_L2CTRL_REG(0x9c)
#define	L2_STAT_REG1	AK88_L2CTRL_REG(0xa0)
#define	L2_STAT_REG2	AK88_L2CTRL_REG(0xa8)

#define L2_SD_BUFX	2 /* assign l2 buf2 to sd */ 

#define L2_USING	1

#define l2_write(reg, val) __raw_writel((val), (reg))
#define l2_read(reg)	__raw_readl(reg)

void l2_init(void)
{
	unsigned int regval;

	/* l2_write(L2_INT_ENA, 0); */

	regval = l2_read(L2_DMA_REQ);
	regval |= 0x1;
	l2_write(L2_DMA_REQ, regval);

	regval = l2_read(L2_COMBUF_CFG);
	regval |= 0x1<<(16+L2_SD_BUFX) | 0x1<<L2_SD_BUFX;
	l2_write(L2_COMBUF_CFG, regval);
}

void l2_pre(int buf_id, int addr, int size, int dir)
{
	unsigned int regval;

	l2_init();

	/* l2_write(L2_COMBUF_CFG, 0xffff00ff); */
	regval = l2_read(L2_COMBUF_CFG) | 0x1<<(24+buf_id);
	l2_write(L2_COMBUF_CFG, regval);
	/* dbg("L2_COMBUF_CFG(0x%x), regval(0x%x)", l2_read(L2_COMBUF_CFG), regval); */

        /*
	 * if (dir) [> write <]
	 *         regval = l2_read(L2_COMBUF_CFG) | 0x1<<(8+buf_id);
	 * else
	 *         regval = l2_read(L2_COMBUF_CFG) & ~(0x1<<(8+buf_id));
	 * l2_write(L2_COMBUF_CFG, regval);
         */

	regval = l2_read(L2_ASSIGN_REG1);
	regval &= ~(0x7<<12); /* sd */ 
	regval |= (buf_id & 0x7) << 12;
	l2_write(L2_ASSIGN_REG1, regval);
	/* dbg("L2_ASSIGN_REG1(0x%x), regval(0x%x)", l2_read(L2_ASSIGN_REG1), regval); */
}

#if 0
void l2_write_pre(int buf_id, int addr, int size, int dir)
{
	unsigned int regval;

	/* l2_write(L2_COMBUF_CFG, 0xffff00ff); */
	regval = l2_read(L2_COMBUF_CFG) | 0x1<<(24+buf_id);
	l2_write(L2_COMBUF_CFG, regval);
	dbg("L2_COMBUF_CFG(0x%x), regval(0x%x)", l2_read(L2_COMBUF_CFG), regval);

	regval = l2_read(L2_ASSIGN_REG1);
	regval &= ~(0x7<<12); /* sd */ 
	regval |= (buf_id & 0x7) << 12;
	l2_write(L2_ASSIGN_REG1, regval);
	dbg("L2_ASSIGN_REG1(0x%x), regval(0x%x)", l2_read(L2_ASSIGN_REG1), regval);

	regval = addr & ~(0xf<<28); /* 28 bits */ 
	l2_write(L2_DMA_ADDR + (buf_id<<2), regval);

	regval = (size>>6) & 0xff;
	l2_write(L2_DMA_CNT + (buf_id<<2), regval);

	if (dir) /* write */
		regval = l2_read(L2_COMBUF_CFG) | 0x1<<(8+buf_id);
	else
		regval = l2_read(L2_COMBUF_CFG) & ~(0x1<<(8+buf_id));
	l2_write(L2_COMBUF_CFG, regval);
	
	regval = l2_read(L2_DMA_REQ);
	regval &= ~((1<<9) | (0xffff<<16));
	regval |= 1 << (24+buf_id);
	l2_write(L2_DMA_REQ, regval);

        /*
	 * regval = l2_read(L2_ASSIGN_REG1);
	 * regval &= ~(0x7<<12); [> sd <] 
	 * regval |= (buf_id & 0x7) << 12;
	 * l2_write(L2_ASSIGN_REG1, regval);
	 * dbg("L2_ASSIGN_REG1(0x%x), regval(0x%x)", l2_read(L2_ASSIGN_REG1), regval);
         */
}

void l2_write_pre_2(int buf_id)
{
	unsigned int regval;

	while (l2_read(L2_DMA_REQ) & (1<<24+buf_id));
	
        /*
	 * regval = l2_read(L2_ASSIGN_REG1);
	 * regval &= ~(0x7<<12); [> sd <] 
	 * regval |= (buf_id & 0x7) << 12;
	 * l2_write(L2_ASSIGN_REG1, regval);
	 * dbg("L2_ASSIGN_REG1(0x%x), regval(0x%x)", l2_read(L2_ASSIGN_REG1), regval);
         */
}


void l2_write_dma(int buf_id, int addr, int size, int dir)
{
	int i = 0;
	dbg("waiting %d", i);
}
#endif

void l2_dma(int buf_id, int addr, int size, int dir)
{
	unsigned int regval;
	int timeout;

	regval = addr & ~(0xf<<28);
	l2_write(L2_DMA_ADDR + (buf_id<<2), regval);
	/* dbg("L2_DMA_ADDR(0x%x), regval(0x%x)", l2_read(L2_DMA_ADDR + (buf_id<<2)), regval); */

	regval = size>>6 & 0xff;
	l2_write(L2_DMA_CNT + (buf_id<<2), regval);
	/* dbg("L2_DMA_CNT(0x%x), regval(0x%x)", l2_read(L2_DMA_CNT + (buf_id<<2)), regval); */

	regval = l2_read(L2_COMBUF_CFG);
	regval |= 0x1<<buf_id | 0x1<<(16+buf_id);
	if (dir)
		regval |= 0x1<<(8+buf_id);
	else
		regval &= ~(0x1<<(8+buf_id));
	l2_write(L2_COMBUF_CFG, regval);
	/* dbg("L2_COMBUF_CFG(0x%x), regval(0x%x)", l2_read(L2_COMBUF_CFG), regval); */

	regval = l2_read(L2_DMA_REQ);
	regval &= ~((1<<9) | 0xffff<<16);
	regval |= 0x1<<(24+buf_id);
	l2_write(L2_DMA_REQ, regval);

	timeout = 50000;
	while ((l2_read(L2_DMA_REQ) & (0x1<<(24+buf_id))) && timeout) {  /* poll */
		timeout--;
	}
	dbg("waiting %d", timeout);
}
void l2_post(int buf_id)
{
	l2_write(L2_DMA_CNT + (buf_id<<2), 0);
}
