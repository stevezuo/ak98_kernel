#ifndef __AK88_MCI_H
#define __AK88_MCI_H

#define MMC_CLK_CTRL     (0x04)
#define MMC_CMD_ARG      (0x08)
#define MMC_CMD_REG      (0x0C)
#define MMC_CMD_RESP     (0x10)
#define MMC_RESP1        (0x14)
#define MMC_RESP2        (0x18)
#define MMC_RESP3        (0x1C)
#define MMC_RESP4        (0x20)
#define MMC_DATA_TIMER   (0x24)
#define MMC_DATA_LENGTH  (0x28)
#define MMC_DATA_CONTROL (0x2C)
#define MMC_DATA_COUNTER (0x30)
#define MMC_STATUS       (0x34)
#define MMC_INT_CTRL     (0x38)
#define MMC_DMA_MODE     (0x3C)
#define MMC_CPU_MODE     (0x40)

#endif /* __AK88_MCI_H */
