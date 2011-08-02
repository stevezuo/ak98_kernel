/*
 * AK98 MAC Fast Ethernet driver for Linux.
 * Features
 * Copyright (C) 2010 ANYKA
 * AUTHOR Tang Anyang
 * AUTHOR Zhang Jingyuan
 * 10-11-01 09:08:08
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/clk.h>

#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/map.h>
#include <mach/mac.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/l2cache.h>

#define MACNAME	"ak98_mac"
#define DRV_VERSION	"1.0"
#define TPD_RING_SIZE 0x50
#define RFD_RING_SIZE 0x50
#define RRD_RING_SIZE 0x50

#include "eth_ops.h"
#include "Ethernethw.h"
#include "phyhw.h"

#if 0
#define dbg(fmt, arg...) printk(KERN_DEBUG "%s(%d): " fmt "\n", __func__, __LINE__, ##arg)
#else
#define dbg(fmt, arg...) {}
#endif

/* rrd format */
typedef struct _RrdDescr_s {

	unsigned short  xsum;           /*  */

	unsigned short  nor     :4  ;   /* number of RFD */
	unsigned short  si      :12 ;   /* start index of rfd-ring */

	unsigned short  hash;           /* rss(MSFT) hash value */

	unsigned short  hash1;  

	unsigned short  vidh    :4  ;   /* vlan-id high part */
	unsigned short  cfi     :1  ;   /* vlan-cfi */ 
	unsigned short  pri     :3  ;   /* vlan-priority */
	unsigned short  vidl    :8  ;   /* vlan-id low part */
	unsigned char   hdr_len;        /* Header Length of Header-Data Split. unsigned short unit */
	unsigned char   hds_typ :2  ;   /* Header-Data Split Type, 
									00:no split, 
									01:split at upper layer protocol header
									10:split at upper layer payload */
	unsigned char   rss_cpu :2  ;   /* CPU number used by RSS */
	unsigned char   hash_t6 :1  ;   /* TCP(IPv6) flag for RSS hash algrithm */
	unsigned char   hash_i6 :1  ;   /* IPv6 flag for RSS hash algrithm */
	unsigned char   hash_t4 :1  ;   /* TCP(IPv4)  flag for RSS hash algrithm */
	unsigned char   hash_i4 :1  ;   /* IPv4 flag for RSS hash algrithm */

	unsigned short  frm_len :14 ;   /* frame length of the packet */        
	unsigned short  l4f     :1  ;   /* L4(TCP/UDP) checksum failed */
	unsigned short  ipf     :1  ;   /* IP checksum failed */
	unsigned short  vtag    :1  ;   /* vlan tag */
	unsigned short  pid     :3  ;   /* protocol id,
						  000: non-ip packet
						  001: ipv4(only)
						  011: tcp/ipv4
						  101: udp/ipv4
						  010: tcp/ipv6
						  100: udp/ipv6
						  110: ipv6(only) */
	unsigned short  res     :1  ;   /* received error summary */
	unsigned short  crc     :1  ;   /* crc error */
	unsigned short  fae     :1  ;   /* frame alignment error */
	unsigned short  trunc   :1  ;   /* truncated packet, larger than MTU */
	unsigned short  runt    :1  ;   /* runt packet */
	unsigned short  icmp    :1  ;   /* incomplete packet, due to insufficient rx-descriptor */
	unsigned short  bar     :1  ;   /* broadcast address received */
	unsigned short  mar     :1  ;   /* multicast address received */
	unsigned short  typ     :1  ;   /* type of packet (ethernet_ii(1) or snap(0)) */
	unsigned short  resv1   :2  ;   /* reserved, must be 0 */
	unsigned short  updt    :1  ;   /* update by hardware. after hw fulfill the buffer, this bit 
						  should be 1 */
} RrdDescr_t, *PRrdDescr_t;

unsigned char *pMacBase = NULL;
//unsigned char *pSystemBase;
unsigned char *psysbase;
unsigned long g_tpdconsumerindex = 0;
unsigned long g_rfdconsumerindex = 0;
unsigned long g_rrdconsumerindex = 0;
bool g_update = false;

//unsigned long rfdaddress = 0;
unsigned long tpdaddress = 0;
unsigned long tpdaddressVa = 0;
void *tpdbufaddressVa = NULL;
dma_addr_t tpdbufaddressPa = 0;
unsigned long rrdaddressVa = 0;

void *rfd_sequenceva = NULL;
dma_addr_t rfd_sequence;
void *RingbufVa = NULL;
dma_addr_t RingbufPa;

void *rfdbaseva = NULL;
dma_addr_t rfdbasepa;

static void ak98_mac_hash_table(struct net_device *ndev);

unsigned long _2xswitchflag = 0;
static inline void cpu_clk_2x_switch(void)
{
	if (ak98_get_cpu_clk() / MHz > 340)
	{
		printk("CPU Core > 340 MHz");
		_2xswitchflag = 1;
	}
	else {
		printk("CPU Core <= 340 MHz");
		_2xswitchflag = 0;
	}
}	

static inline void close_2x(void)
{
	if (_2xswitchflag) {
		udelay(5);
		REG32(psysbase + 0x04) &= ~(0x1 << 15);
		udelay(5);
	}
}
static inline void open_2x(void)
{
	if (_2xswitchflag) {
		udelay(5);
		REG32(psysbase + 0x04) |= (0x1 << 15);
		udelay(5);
	}
}


/* Structure/enum declaration ------------------------------- */
typedef struct mac_info {
	void __iomem	*io_addr;	/* Register I/O base address */
	u16		 irq;		/* IRQ */

	u16		tx_pkt_cnt;
	u16		queue_pkt_len;
	u16		queue_start_addr;
	u16		queue_ip_summed;
	u16		dbug_cnt;
	u8		io_mode;		/* 0:word, 2:byte */
	u8		phy_addr;
	u8		imr_all;

	unsigned int	flags;
	unsigned int	in_suspend :1;

	void (*inblk)(void __iomem *port, void *data, int length);
	void (*outblk)(void __iomem *port, void *data, int length);
	void (*dumpblk)(void __iomem *port, int length);

	struct device	*dev;	     /* parent device */

	struct resource	*addr_res;   /* resources found */
	struct resource	*addr_req;   /* resources requested */
	struct resource *irq_res;

	struct mutex	 addr_lock;	/* phy and eeprom access lock */

	struct delayed_work phy_poll;
	struct net_device  *ndev;

	spinlock_t	lock;

	u32		msg_enable;

	int		rx_csum;
	int		can_csum;
	int		ip_summed;
	struct clk	*clk;
} mac_info_t;

void MacDelay(unsigned long us)
{
	unsigned long i =0;
	for (i=0; i< 10*us*1000; i++)
		;
}

/** * @brief Read Phy Register 
* Read Phy Register from MII Interface 
* @author Tang Anyang
* @date 2010-11-16 
* @param unsigned long RegAddr: Phy Register address
* @retval unsigned long: the value of Phy Register. 
*/
unsigned long MIIRead(unsigned long RegAddr)
{
	unsigned int Val;
	unsigned short Index;    
	unsigned short phyVal;

	Val = 
		MDIO_CTRL_REG_ADDR(RegAddr) |		   
		MDIO_CTRL_START|
		MDIO_CTRL_READ;

	REG32(pMacBase + REG_MDIO_CTRL)= Val;

	for (Index=0; Index <MDIO_MAX_AC_TIMER; Index++)
	{
		Val = REG32(pMacBase + REG_MDIO_CTRL);
		if (0 == (Val&FLAG(MDIO_CTRL_BUSY_OFF)))
		{
			phyVal = (unsigned short) Val;            
			goto mr_exit;
		}
		MacDelay(10);
	}

	phyVal = 0;

mr_exit:
	return phyVal;
}

/** * @brief Wrtie Phy Register 
* Write dedicated value to  Phy Register from MII Interface 
* @author Tang Anyang
* @date 2010-11-16 
* @param unsigned long RegAddr: Phy Register address
* @param  unsigned long phyVal: dedicated value. 
*/
void MIIWrite(unsigned long RegAddr, unsigned long phyVal)
{
	unsigned int Val;
	unsigned short Index;

	REG32(pMacBase + REG_MDIO_CTRL)= 0;
	MacDelay(30);
	for(Index = 0; Index < MDIO_MAX_AC_TIMER; Index++)
	{
		Val = REG32(pMacBase + REG_MDIO_CTRL);
		if (0 == (Val & FLAG(MDIO_CTRL_BUSY_OFF)))
		{
			break;        
		}
		MacDelay(10);
	}

	Val = 
		MDIO_CTRL_DATA(phyVal) |
		MDIO_CTRL_REG_ADDR(RegAddr) |MDIO_CTRL_WRITE|		
		MDIO_CTRL_START;

	REG32(pMacBase +  REG_MDIO_CTRL)= Val;

	for (Index=0; Index <MDIO_MAX_AC_TIMER; Index++)
	{
		Val = REG32(pMacBase +  REG_MDIO_CTRL);
		if (0 == (Val&FLAG(MDIO_CTRL_BUSY_OFF)))
		{

			return;
		}
		MacDelay(10);
	}
}

unsigned int HwStopMAC(void)
{
	unsigned int Val;
	unsigned short Index;      

	Val = REG32(pMacBase + REG_RXQ_CTRL);
	BIT_CLEAR(Val, RXQ_CTRL_EN_OFF);
	BIT_CLEAR(Val, RXQ_CTRL_Q1_EN_OFF);
	BIT_CLEAR(Val, RXQ_CTRL_Q2_EN_OFF);
	BIT_CLEAR(Val, RXQ_CTRL_Q3_EN_OFF);  
	REG32(pMacBase + REG_RXQ_CTRL) = Val;    

	Val = REG32(pMacBase + REG_TXQ_CTRL);
	BIT_CLEAR(Val, TXQ_CTRL_EN_OFF);   
	REG32(pMacBase + REG_TXQ_CTRL) = Val;

	//  waiting for rxq/txq be idle 
	for (Index=0; Index<50; Index++)
	{     
		Val = REG32(pMacBase + REG_IDLE_STATUS);
		if (BIT_TEST(Val, IDLE_STATUS_RXQ_OFF) ||
			BIT_TEST(Val, IDLE_STATUS_TXQ_OFF))
		{
			MacDelay(20);
		}
		else
			break;
	}

	// stop mac tx/rx   
	Val = REG32(pMacBase + REG_MAC_CTRL);
	BIT_CLEAR(Val, MAC_CTRL_RXEN_OFF);
	BIT_CLEAR(Val, MAC_CTRL_TXEN_OFF);
	REG32(pMacBase + REG_MAC_CTRL) = Val;

	MacDelay(10);

	for (Index=0; Index<50; Index++) //
	{        
		Val = REG32(pMacBase + REG_IDLE_STATUS);
		if (0 == (unsigned char)Val)
			return true;

		MacDelay(20);
	}

	return false;
}

void MacRest(void)
{
	unsigned long Val =0;
	unsigned long Index;

	// clear to unmask the corresponding INTs
	REG32(pMacBase + REG_IMR)= 0x00; 
	// disable interrupt
	REG32(pMacBase + REG_ISR)=FLAG(ISR_DIS_OFF);

	HwStopMAC();

	// reset whole-MAC safely 
	Val = REG32(pMacBase + REG_MASTER_CTRL);
	BIT_SET(Val, MASTER_CTRL_MAC_SOFT_RST_OFF);
	REG32(pMacBase + REG_MASTER_CTRL)= Val;    

	MacDelay(50);

	for (Index=0; Index<50; Index++) // wait atmost 1ms 
	{   
		Val = REG32(pMacBase + REG_IDLE_STATUS);
		if (0 == (unsigned char)Val)
		{
			return ;
		}        
		MacDelay(20);
	}

	return;
}

bool InitEthernetMemory(void)
{
	int i;
	unsigned long *tempp;

	if (RingbufVa == NULL)
		RingbufVa = dma_alloc_coherent(NULL, TPD_RING_SIZE * 16 + RRD_RING_SIZE * 16, &RingbufPa, GFP_KERNEL);
	if(RingbufVa == NULL)
	{
		dbg("Alloc Memory for RingBuf Failed!");
		return false;
	}
	if (rfd_sequenceva == NULL)
		rfd_sequenceva = dma_alloc_coherent(NULL, RFD_RING_SIZE * 8, &rfd_sequence, GFP_KERNEL);
	if (rfd_sequenceva == NULL)
	{
		dbg("Alloc rfd sequence buffer failed!");
		return false;
	}

	if (tpdbufaddressVa == NULL)
		tpdbufaddressVa = dma_alloc_coherent(NULL, TPD_RING_SIZE * 1520, &tpdbufaddressPa, GFP_KERNEL);
	if(tpdbufaddressVa == NULL)
	{
		dbg("Alloc tpd sequence buffer failed!");
		return false;
	}
	if (rfdbaseva == NULL)
		rfdbaseva = dma_alloc_coherent(NULL, RFD_RING_SIZE * 1520, &rfdbasepa, GFP_KERNEL);
	if(rfdbaseva == NULL)
	{
		dbg("Alloc rfd  buffer failed!");
		return false; 	
	}

	tempp = rfd_sequenceva;
	for(i = 0; i < TPD_RING_SIZE; i++)
	{
		*tempp = rfdbasepa + i*1520;	
		tempp++;

		*tempp =0x00;
		tempp++;
	}

	return true;
}

bool init_hw(struct net_device *ndev)
{
	unsigned long Val = 0;
	unsigned long IntModerate = 100;//500000/5000;

	unsigned int mac_addL;
	unsigned int mac_addH;
	
	unsigned long rrdaddress;

	g_tpdconsumerindex = 0;
	g_rfdconsumerindex = 0;
	g_rrdconsumerindex = 0;

	MacRest();
	
	ak98_gpio_cfgpin(AK98_GPIO_98, AK98_GPIO_DIR_OUTPUT);
	ak98_gpio_setpin(AK98_GPIO_98, AK98_GPIO_LOW);
	mdelay(10);
	ak98_gpio_cfgpin(AK98_GPIO_98, AK98_GPIO_DIR_INPUT);
	mdelay(1);
	
#if 0
	MIIWrite(MII_BMCR,0x8000);

	mdelay(50);
	
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));
	
	while(MIIRead(MII_BMCR)&0x8000)
	{
		dbg("BMCR:0x%lx", MIIRead(MII_BMCR));
	}
#endif
	dbg("BMCR:0x%lx", MIIRead(MII_BMCR));
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));
	dbg("PHYSID1:0x%lx, PHYSID2:0x%lx", MIIRead(MII_PHYSID1), MIIRead(MII_PHYSID2));

	MIIWrite(MII_BMCR, MIIRead(MII_BMCR) | 0x1100);
	dbg("BMCR:0x%lx", MIIRead(MII_BMCR));
	dbg("GIGA_PSSR:0x%lx", MIIRead(MII_GIGA_PSSR));

	/* set mac-address */
	mac_addL = ndev->dev_addr[5] | (ndev->dev_addr[4] << 8)
			| (ndev->dev_addr[3] << 16) | (ndev->dev_addr[2] << 24);
	mac_addH = ndev->dev_addr[1] | (ndev->dev_addr[0] << 8);

	REG32(pMacBase + REG_MAC_STA_ADDR)= mac_addL;
	REG32(pMacBase + REG_MAC_STA_ADDR+4)= mac_addH;

	// clear the Multicast HASH table 
	REG32(pMacBase + REG_RX_HASH_TABLE) = 0x00;
	REG32(pMacBase + REG_RX_HASH_TABLE+4)= 0x00;

	// clear any WOL setting/status /
	Val = REG32(pMacBase + REG_WOL_CTRL);
	REG32(pMacBase + REG_WOL_CTRL)=0x00;

	tpdaddressVa = (unsigned long)RingbufVa;
	// tx/rx/smb Ring BaseMem 
	REG32(pMacBase + REG_NTPD_HDRADDR_LO) = RingbufPa;//NTPD_HDRADDR_LO
	REG32(pMacBase + REG_HTPD_HDRADDR_LO)= RingbufPa;//HTPD_HDRADDR_LO
	REG32(pMacBase + REG_TX_BASE_ADDR_HI) = 0x00;//TX_BASE_ADDR_HI
	REG32(pMacBase + REG_TPD_RING_SIZE)= TPD_RING_SIZE;//TPD_RING_SIZE

	REG32(pMacBase + REG_RX_BASE_ADDR_HI)= 0x00;//RX_BASE_ADDR_HI

	REG32(pMacBase + REG_RFD0_HDRADDR_LO) = rfd_sequence;//RFD0_HDRADDR_LO    

	REG32(pMacBase + REG_RFD1_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RFD2_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RFD3_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RFD_RING_SIZE)= RFD_RING_SIZE;//RFD_RING_SIZE
	REG32(pMacBase + REG_RFD_BUFFER_SIZE)= 0x600;//RFD_BUFFER_SIZE	

	rrdaddress = RingbufPa + TPD_RING_SIZE * 16;
	rrdaddressVa = (unsigned long)RingbufVa +  TPD_RING_SIZE * 16;

	REG32(pMacBase + REG_RRD0_HDRADDR_LO)= rrdaddress; // RRD0_HDRADDR_LO  
	REG32(pMacBase + REG_RRD1_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RRD2_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RRD3_HDRADDR_LO)= 0x00;
	REG32(pMacBase + REG_RRD_RING_SIZE)= RRD_RING_SIZE;//REG_RRD_RING_SIZE


	REG32(pMacBase + REG_TXQ_TXF_BURST_L1)= 0;// TX watermark, to enter l1 state. 
	REG32(pMacBase + REG_RXD_CTRL)= 0; // RXD threshold. 

	// load all base/mem ptr	 
	REG32(pMacBase + REG_SRAM_LOAD_PTR)= FLAG(SRAM_LOAD_PTR_OFF);

	// set Interrupt Moderator Timer (max interrupt per sec)
	// we use seperate time for rx/tx 
	REG16(pMacBase + REG_IRQ_MODRT_INIT)= IntModerate * 2;
	REG16(pMacBase + REG_IRQ_MODRT_RX_INIT)= IntModerate; 

	// set Interrupt Clear Timer
	// HW will enable self to assert interrupt event to system after
	// waiting x-time for software to notify it accept interrupt.	  

	REG32(pMacBase + REG_INT_RETRIG_TIMER)= 10000;// 20ms 

	// Enable Read-Clear Interrupt Mechanism   
	Val = FLAG(MASTER_CTRL_INT_RCLR_EN_OFF);
	BIT_SET(Val, MASTER_CTRL_SA_TIMER_EN_OFF);
	REG32(pMacBase + REG_MASTER_CTRL)= Val;

	REG32(pMacBase + REG_FC_RXF_HI)= 0x03300400;
	REG32(pMacBase + REG_TXQ_JUMBO_TSO_THRESHOLD)= 0xbf;

	// set MTU	 
	REG32(pMacBase + REG_MTU)= 1540;

	// set DMA		
	//mac_set_reg(REG_DMA_CTRL, 0x347ed1);//DMA Engine Control 
	//mac_set_reg(REG_DMA_CTRL, 0x47C20);//DMA Engine Control 
	//REG32(pMacBase + REG_DMA_CTRL)= 0x47C10;
	REG32(pMacBase + REG_DMA_CTRL)= 0x47C14;

	// set TXQ	 
	REG32(pMacBase + REG_TXQ_CTRL)= 0x01000025;

	// set RXQ	
	//mac_set_reg(0x600015a0, 0xc08f10f0); 
	REG32(pMacBase + REG_RXQ_CTRL)= 0xC0800000; 
	
	// rfd producer index	 
	REG32(pMacBase + REG_RFD0_PROD_INDEX)= RFD_RING_SIZE - 1; 

	//set MAC control
	//REG32(pMacBase + REG_MAC_CTRL)= 0x0e10dcef;//MAC control register
	REG32(pMacBase + REG_MAC_CTRL)= 0x06105cef;//MAC control register

	REG32(pMacBase + REG_IMR)= 0x1d608;//Interrupt Mask 
	MIIWrite(MII_IER, 0x0c00);

	ak98_mac_hash_table(ndev);

	return true;
}

/** * @brief Initialize Mac 
* Initialize MAC and PHY 
* @author Tang Anyang
* @date 2010-11-16 
* @param unsigned char * MacAddress: 
*/
bool MacInit(struct net_device *ndev)
{
	//struct mac_info *db;
	volatile unsigned long count;
	
	//db = netdev_priv(ndev);
	close_2x();
	
	//to enable the 25MHz oscillator
	//TODO: Need to be changed to new clock API
	REG32(psysbase + 0x54) |= (1 << 23);
	
	//unreset MAC module
	count = 10000;
	while(count--);
	REG32(psysbase + 0x10) &= ~(1 << 30);
	count = 10000;
	
	while(count--);
	
	//to enable the 25MHz oscillator
	REG32(psysbase + 0x54) |= (1 << 23);
	
	count = 10000;
	//reset MAC module
	while(count--);
	REG32(psysbase + 0x10) |= (1 << 30);
	count = 10000;
	
	while(count--);
	
	//unreset MAC module
	REG32(psysbase + 0x10) &= ~(1 << 30);
	count = 10000;
	
	while(count--);

	if (false == InitEthernetMemory())
		return false;
	
	if (false == init_hw(ndev))
		return false;

	open_2x();

	return true;
}

void Macexit(struct net_device *ndev)
{
	if (rfd_sequenceva) {
		dma_free_coherent(NULL, RFD_RING_SIZE * 4, rfd_sequenceva, rfd_sequence);
		rfd_sequenceva = NULL;
		rfd_sequence = 0;
	}

	if (tpdbufaddressVa) {
		dma_free_coherent(NULL, TPD_RING_SIZE * 1520, tpdbufaddressVa, tpdbufaddressPa);
		tpdbufaddressVa = NULL;
		tpdbufaddressPa = 0;
	}

	if (rfdbaseva) {
		dma_free_coherent(NULL, RFD_RING_SIZE * 1520, rfdbaseva, rfdbasepa);
		rfdbaseva = NULL;
		rfdbasepa = 0;
	}

	if (RingbufVa) {
		dma_free_coherent(NULL, TPD_RING_SIZE * 16 + RRD_RING_SIZE * 16, RingbufVa, RingbufPa);
		RingbufVa = NULL;
		RingbufPa = 0;
	}
}

unsigned long GetPacketCount(void)
{
	unsigned long ulNewRfdConsIdx = 0;

	ulNewRfdConsIdx = REG32(pMacBase +  REG_RFD0_CONS_INDEX);

	dbg("ulNewRfdConsIdx:0x%lx, g_rfdconsumerindex:%lx", ulNewRfdConsIdx, g_rfdconsumerindex);

	return ulNewRfdConsIdx - g_rfdconsumerindex;
}

long ReceivePacket(struct net_device *ndev)
{
	PRrdDescr_t prrd;
	short length;
	static unsigned long g_wait_count = 0;
	unsigned char *sendbuffer;
	struct sk_buff *skb;

	ak98_l2cache_invalidate();
	
	prrd = (PRrdDescr_t)(rrdaddressVa + g_rfdconsumerindex*16);
	length = prrd->frm_len;
	dbg("g_rfdconsumerindex:%lx, length:%x, updt:%d", 
		g_rfdconsumerindex, length, prrd->updt); 

	if (length == 0)
	{
		if (prrd->nor == 0)
		{
			if (g_update)
			{
				g_rrdconsumerindex = g_rfdconsumerindex;
				g_update = false;
			}
		}
		length = 0;

		g_rfdconsumerindex++;
		g_rfdconsumerindex %= RFD_RING_SIZE;
		
		return length;
	}

	if (prrd->updt == 0)

	{
		g_wait_count++;

		if (g_wait_count < 5)
		{
			length = -1;
			
			return length;
		}
	}
	g_wait_count = 0;
	prrd->updt = 0;
	prrd->frm_len = 0;

	sendbuffer = (unsigned char *)(rfdbaseva + (g_rfdconsumerindex)*1520);

	skb = dev_alloc_skb(length + 2);
	skb_reserve(skb, 2);
	skb->dev = ndev;
	memcpy(skb_put(skb, length), sendbuffer, length);
	skb->protocol = eth_type_trans(skb, ndev);

	netif_rx(skb);

	ndev->last_rx = jiffies;
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += length;
	
	REG32(pMacBase + REG_RFD0_PROD_INDEX) = g_rfdconsumerindex;

	g_rfdconsumerindex++;
	g_rfdconsumerindex %= RFD_RING_SIZE;
	
	return length;
}

void SendPacket(unsigned char *sendbuffer, unsigned long length)
{
	int tpdvalue = 0x80000000;	
	unsigned char *RingbufVa;
	unsigned long tpdbufv; 

	RingbufVa = (unsigned char *)(tpdbufaddressVa+g_tpdconsumerindex*1520);
	memcpy(RingbufVa, sendbuffer, length);

	tpdbufv = tpdaddressVa + g_tpdconsumerindex*16; 

	REG32(tpdbufv)= (unsigned long)0x3aa00000+length;
	dbg("Send 0x%lx", length);

	tpdbufv += 4;
	REG32(tpdbufv)= tpdvalue;

	tpdbufv += 4;

	close_2x();
	REG32(tpdbufv)= tpdbufaddressPa + g_tpdconsumerindex*1520;

	tpdbufv += 4;
	
	g_tpdconsumerindex++;
	
	g_tpdconsumerindex %= TPD_RING_SIZE;
	REG32(pMacBase + REG_HTPD_PROD_INDEX) = g_tpdconsumerindex;

	open_2x();
	
	return;
}

/* ak98_mac_release_board
 *
 * release a board, and any mapped resources
 */

static void
ak98_mac_release_board(struct platform_device *pdev, struct mac_info *db)
{
	/* unmap our resources */

	iounmap(db->io_addr);

	/* release the resources */

	release_resource(db->addr_req);
	kfree(db->addr_req);
}

/*
 * Set AK98 MAC address
 */
static int set_mac_address(struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;

	if (netif_running(ndev))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	/* set the Ethernet address */
	REG32(pMacBase + REG_MAC_STA_ADDR) = ndev->dev_addr[0] | (ndev->dev_addr[1] << 8)
			| (ndev->dev_addr[2] << 16) | (ndev->dev_addr[3] << 24);
	REG32(pMacBase + REG_MAC_STA_ADDR + 4) = ndev->dev_addr[4] | (ndev->dev_addr[5] << 8);

	return 0;
}

/*
 * atl1c_hash_mc_addr
 * 	purpose
 * 		set hash value for a multicast address
 * 		hash calcu processing :
 * 			1. calcu 32bit CRC for multicast address
 * 			2. reverse crc with MSB to LSB
 */
u32 ak98_mac_hash_mc_addr(u8 *mc_addr)
{
	u32 crc32;
	u32 value = 0;
	int i;

	crc32 = ether_crc_le(6, mc_addr);
	for (i = 0; i < 32; i++)
		value |= (((crc32 >> i) & 1) << (31 - i));

	return value;
}

/*
 * Sets the bit in the multicast table corresponding to the hash value.
 * hw - Struct containing variables accessed by shared code
 * hash_value - Multicast address hash value
 */
void ak98_mac_hash_set(u32 hash_value)
{
	u32 hash_bit, hash_reg;
	u32 mta;

	/*
	 * The HASH Table  is a register array of 2 32-bit registers.
	 * It is treated like an array of 64 bits.  We want to set
	 * bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The register is determined by the
	 * upper bit of the hash value and the bit within that
	 * register are determined by the lower 5 bits of the value.
	 */
	hash_reg = (hash_value >> 31) & 0x1;
	hash_bit = (hash_value >> 26) & 0x1F;

	if (hash_reg == 0)
		mta = REG32(pMacBase + REG_RX_HASH_TABLE);
	else
		mta = REG32(pMacBase + REG_RX_HASH_TABLE + 4);

	mta |= (1 << hash_bit);
	if (hash_reg == 0)
		REG32(pMacBase + REG_RX_HASH_TABLE) = mta;
	else
		REG32(pMacBase + REG_RX_HASH_TABLE + 4) = mta;
}

/*
 *  Set AK98 MAC multicast address
 */
static void
ak98_mac_hash_table(struct net_device *ndev)
{
	struct dev_mc_list *mc_ptr;
	u32 mac_ctrl_data;
	u32 hash_value;

	/* Check for Promiscuous and All Multicast modes */
	mac_ctrl_data = REG32(pMacBase + REG_MAC_CTRL);

	if (ndev->flags & IFF_PROMISC) {
		mac_ctrl_data |= FLAG(MAC_CTRL_PROM_MODE_OFF);
	} else if (ndev->flags & IFF_ALLMULTI) {
		mac_ctrl_data |= FLAG(MAC_CTRL_MUTI_ALL_OFF);
		mac_ctrl_data &= ~(FLAG(MAC_CTRL_PROM_MODE_OFF));
	} else {
		mac_ctrl_data &= ~(FLAG(MAC_CTRL_MUTI_ALL_OFF) | FLAG(MAC_CTRL_PROM_MODE_OFF));
	}

	REG32(pMacBase + REG_MAC_CTRL) = mac_ctrl_data;

	/* clear the old settings from the multicast hash table */
	REG32(pMacBase + REG_RX_HASH_TABLE) = 0;
	REG32(pMacBase + REG_RX_HASH_TABLE + 4) = 0;;

	/* comoute mc addresses' hash value ,and put it into hash table */
	for (mc_ptr = ndev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {
		hash_value = ak98_mac_hash_mc_addr(mc_ptr->dmi_addr);
		ak98_mac_hash_set(hash_value);
	}
}

/* Our watchdog timed out. Called by the networking layer */
static void ak98_mac_timeout(struct net_device *ndev)
{
	/* Initialize AK98 MAC */
	if (MacInit(ndev) == false) {
		printk("Mac reset fail\n");
		return;
	}

	netif_wake_queue(ndev);
}

/*
 *  Hardware start transmission.
 *  Send a packet to media from the upper layer.
 */
static int
ak98_mac_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	unsigned long flags;
	int len;
	char *data, shortpkt[ETH_ZLEN];
	mac_info_t *db = netdev_priv(ndev);

	/* keep the upload from being interrupted, since we
	 *     ask the chip to start transmitting before the
	 *     whole packet has been completely uploaded. */
	spin_lock_irqsave(&db->lock, flags);
	data = skb->data;
	len = skb->len;
	if (len < ETH_ZLEN) {
		memset(shortpkt, 0, ETH_ZLEN);
		memcpy(shortpkt, skb->data, skb->len);
		len = ETH_ZLEN;
		data = shortpkt;
	}
	netif_stop_queue(ndev);

	SendPacket(data, len);
	spin_unlock_irqrestore(&db->lock, flags);
	ndev->stats.tx_bytes += skb->len;
	ndev->trans_start = jiffies;
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

int receive(struct net_device *ndev)
{
	int length = 0;

	g_rfdconsumerindex = g_rrdconsumerindex;
	g_update = true;
	
	while (GetPacketCount())
	{
		length = ReceivePacket(ndev);
		if (length == 0)
			break;
		else if (length == -1)
			continue;
	}
	if(g_update)
		g_rrdconsumerindex = g_rfdconsumerindex;
	
	return length;
}

static irqreturn_t ak98_mac_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	mac_info_t *db = netdev_priv(ndev);
	unsigned long flags;
	unsigned long IntStatus = 0;
	unsigned long status;
	
	/* holders of db->lock must always block IRQs */
	spin_lock_irqsave(&db->lock, flags);

	close_2x();
	IntStatus = REG32(pMacBase + REG_ISR);

	dbg("IntStatus %lx", IntStatus);

	if(IntStatus & ISR_GPHY_OFF) {
		unsigned long dwPHYIntStatus = MIIRead(MII_ISR);
		if(dwPHYIntStatus & (ISR_LINK_DOWN | ISR_LINK_UP)) {
			status = MIIRead(MII_BMSR);
			dbg("BMSR: 0x%lx", status);
			if ((status & BMSR_LINK_STATUS) == 0) {
				netif_carrier_off(ndev);
				dbg("%s: link down", ndev->name);
			} else {
				netif_carrier_on(ndev);
				dbg("%s: link up", ndev->name);
			}
		}
		REG32(pMacBase + REG_ISR) = ISR_GPHY_OFF;
	}
	if(IntStatus & ISR_GPHY_LPW_OFF) {
		unsigned long dwPHYIntStatus = MIIRead(MII_ISR);
		if(dwPHYIntStatus & ISR_LINK_UP) {
			//SetNetLinkStatus(TRUE); 
			netif_carrier_on(ndev);
			dbg("%s: low power state link up", ndev->name);
		}
		if(dwPHYIntStatus & ISR_LINK_DOWN) {
			//SetNetLinkStatus(FALSE); 
			netif_carrier_off(ndev);
			dbg("%s: low power state link down", ndev->name);
		}
	}
	
	if(IntStatus & ISR_TX_PKT_OFF) {
		REG32(pMacBase + REG_ISR) = ISR_TX_PKT_OFF;
		dbg("send complete!%lx", g_tpdconsumerindex);
		ndev->stats.tx_packets++;
		netif_wake_queue(ndev);
	}
	if(IntStatus & ISR_RX0_PKT_OFF) {
		receive(ndev);
		REG32(pMacBase + REG_ISR) = ISR_RX0_PKT_OFF;
	}
	if(IntStatus & ISR_RXF_OV_OFF) {
		panic("Liu Hejun, come to help me!\n");
		receive(ndev);
	}
	if(IntStatus & ISR_DMAW_OFF)
	{
		printk("DMAW operation timeout");
		init_hw(ndev);
		netif_wake_queue(ndev);
	}
	if(IntStatus & ISR_DMAR_OFF)
	{
		printk("DMAR operation timeout");
		init_hw(ndev);
		netif_wake_queue(ndev);
	}
	if(IntStatus & ISR_TXQ_OFF)
	{
		printk("TXQ operation timeout");
		init_hw(ndev);
		netif_wake_queue(ndev);
	}

	open_2x();

	spin_unlock_irqrestore(&db->lock, flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 *Used by netconsole
 */
static void ak98_mac_poll_controller(struct net_device *ndev)
{
	disable_irq(ndev->irq);
	ak98_mac_interrupt(ndev->irq, ndev);
	enable_irq(ndev->irq);
}
#endif

/*
 *  Open the interface.
 *  The interface is opened whenever "ifconfig" actives it.
 */
static int
ak98_mac_open(struct net_device *ndev)
{
	mac_info_t *db = netdev_priv(ndev);
	unsigned long status;

	clk_enable(db->clk);

	/* Initialize AK98 MAC */
	if (MacInit(ndev) == false) {
		printk("mac init fail\n");
		return -ENOMEM;
	} else
		printk("mac init success!\n");

	if (request_irq(ndev->irq, &ak98_mac_interrupt, 0, ndev->name, ndev))
		return -EAGAIN;

	disable_irq(ndev->irq);
	status = MIIRead(MII_BMSR);
	dbg("BMSR: 0x%lx", status);
	if ((status & BMSR_LINK_STATUS) == 0)
		netif_carrier_off(ndev);
	else
		netif_carrier_on(ndev);
	enable_irq(ndev->irq);

	netif_start_queue(ndev);
	
	return 0;
}

/*
 * Stop the interface.
 * The interface is stopped when it is brought.
 */
static int
ak98_mac_stop(struct net_device *ndev)
{
	mac_info_t *db = netdev_priv(ndev);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	
	/* free interrupt */
	free_irq(ndev->irq, ndev);
	
	close_2x();
	MacRest();
	MIIWrite(MII_BMCR, MIIRead(MII_BMCR) | 0x800);
	open_2x();

	clk_disable(db->clk);

	return 0;
}

/* Get the current statistics.	This may be called with the card open or
  closed. */
static struct net_device_stats *
ak98_mac_get_stats(struct net_device *ndev)
{
	//struct mac_info_t *db = netdev_priv(ndev);
	//unsigned long flags;

	//spin_lock_irqsave(&lp->lock, flags);
	/* Update the statistics from the device registers. */
	//db->stats.rx_missed_errors += (readreg(dev, PP_RxMiss) >> 6);
	//db->stats.collisions += (readreg(dev, PP_TxCol) >> 6);
	//spin_unlock_irqrestore(&lp->lock, flags);

	return &ndev->stats;
}

static const struct net_device_ops ak98_mac_netdev_ops = {
	.ndo_open		= ak98_mac_open,
	.ndo_stop		= ak98_mac_stop,
	.ndo_start_xmit		= ak98_mac_start_xmit,
	.ndo_tx_timeout		= ak98_mac_timeout,
	.ndo_set_multicast_list	= ak98_mac_hash_table,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= set_mac_address,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= ak98_mac_poll_controller,
#endif
	.ndo_get_stats		= ak98_mac_get_stats,
};

/*
 * Search AK98 MAC, allocate space and register it
 */
static int __devinit
ak98_mac_probe(struct platform_device *pdev)
{
	struct ak98_mac_data *pdata = pdev->dev.platform_data;
	struct mac_info *db;	/* Point a board information structure */
	struct net_device *ndev;
	//volatile unsigned long count;

	int ret = 0;
	int iosize;
	int i;

	cpu_clk_2x_switch();

	/* Init network device */
	ndev = alloc_etherdev(sizeof(struct mac_info));
	if (!ndev) {
		dev_err(&pdev->dev, "could not allocate device.\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	dev_dbg(&pdev->dev, "ak98_mac_probe()\n");

	/* setup board info structure */
	db = netdev_priv(ndev);

	db->dev = &pdev->dev;
	db->ndev = ndev;

	db->addr_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	db->irq_res  = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (db->addr_res == NULL || db->irq_res == NULL) {
		dev_err(db->dev, "insufficient resources\n");
		ret = -ENOENT;
		goto out;
	}

	iosize = resource_size(db->addr_res);
	db->addr_req = request_mem_region(db->addr_res->start, iosize,
					  pdev->name);

	if (db->addr_req == NULL) {
		dev_err(db->dev, "cannot claim address reg area\n");
		ret = -EIO;
		goto out;
	}

	db->io_addr = ioremap(db->addr_res->start, iosize);

	if (db->io_addr == NULL) {
		dev_err(db->dev, "failed to ioremap address reg\n");
		ret = -EINVAL;
		goto out;
	}

	/* get mac clock */
	db->clk = clk_get(db->dev, "mac_clk");
	if (IS_ERR(db->clk)) {
		dbg("clocks missing");
		ret = -ENODEV;
		goto out;
	}
	spin_lock_init(&db->lock);

	/* fill in parameters for net-dev structure */
	ndev->base_addr = (unsigned long)db->io_addr;
	ndev->irq	= db->irq_res->start;

	/* driver system function */
	ether_setup(ndev);

	ndev->netdev_ops	= &ak98_mac_netdev_ops;
	ndev->watchdog_timeo	= msecs_to_jiffies(5000);

	memcpy(ndev->dev_addr, pdata->dev_addr, 6);

	if (!is_valid_ether_addr(ndev->dev_addr)) {
		/* try reading from mac */
		
		for (i = 0; i < 6; i++)
			ndev->dev_addr[i] = 'A' + i;
	}

	if (!is_valid_ether_addr(ndev->dev_addr))
		dev_warn(db->dev, "%s: Invalid ethernet MAC address. Please "
			 "set using ifconfig\n", ndev->name);

	pMacBase = db->io_addr;

	psysbase = AK98_VA_SYSCTRL; 
	if(psysbase == NULL)
	{
		dbg("sysbase alloc error!");
	}

	// initial share pin
	ak98_group_config(ePIN_AS_MAC);
	
	ak98_setpin_as_gpio(AK98_GPIO_98);
	ak98_gpio_pullup(AK98_GPIO_98, AK98_PULLDOWN_DISABLE);
	ak98_gpio_cfgpin(AK98_GPIO_98, AK98_GPIO_DIR_INPUT);
	
	platform_set_drvdata(pdev, ndev);
	ret = register_netdev(ndev);

	if (ret == 0)
		printk(KERN_INFO "%s: ak98_mac at %p IRQ %d MAC: %pM\n",
		       ndev->name, db->io_addr, ndev->irq, ndev->dev_addr);

	return 0;
out:
	dev_err(db->dev, "not found (%d).\n", ret);

	ak98_mac_release_board(pdev, db);
	free_netdev(ndev);

	return ret;
}

static int
ak98_mac_drv_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	mac_info_t *db = netdev_priv(ndev);

	if (ndev) {
		db = netdev_priv(ndev);
		db->in_suspend = 1;

		if (netif_running(ndev)) {
			netif_device_detach(ndev);

			close_2x();
			MacRest();
			MIIWrite(MII_BMCR, MIIRead(MII_BMCR) | 0x800);
			//MIIWrite(0x29, MIIRead(0x29) | 0x8000);
			open_2x();
			
			clk_disable(db->clk);
		}
	}
	return 0;
}

static int
ak98_mac_drv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	mac_info_t *db = netdev_priv(ndev);
	unsigned long status;

	if (ndev) {
		if (netif_running(ndev)) {	
			clk_enable(db->clk);

			/* Initialize AK98 MAC */
			if (MacInit(ndev) == false) {
				printk("mac init fail\n");
				return -ENOMEM;
			} else
				printk("mac init success!\n");
			
			disable_irq(ndev->irq);
			status = MIIRead(MII_BMSR);
			dbg("BMSR: 0x%lx", status);
			if ((status & BMSR_LINK_STATUS) == 0)
				netif_carrier_off(ndev);
			else
				netif_carrier_on(ndev);
			enable_irq(ndev->irq);
			
			netif_device_attach(ndev);
		}
	}
	return 0;
}

static struct dev_pm_ops ak98_mac_drv_pm_ops = {
	.suspend	= ak98_mac_drv_suspend,
	.resume		= ak98_mac_drv_resume,
};

static int __devexit
ak98_mac_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	unregister_netdev(ndev);
	ak98_mac_release_board(pdev, (mac_info_t *) netdev_priv(ndev));
	Macexit(ndev);
	free_netdev(ndev);		/* free device structure */

	dev_dbg(&pdev->dev, "released and freed device\n");
	return 0;
}

static struct platform_driver ak98_mac_driver = {
	.driver	= {
		.name    = "ak98_mac",
		.owner	 = THIS_MODULE,
		.pm	 = &ak98_mac_drv_pm_ops,
	},
	.probe   = ak98_mac_probe,
	.remove  = __devexit_p(ak98_mac_drv_remove),
};

static int __init
ak98_mac_init(void)
{
	printk(KERN_INFO "%s Ethernet Driver, V%s\n", MACNAME, DRV_VERSION);

	return platform_driver_register(&ak98_mac_driver);
}

static void __exit
ak98_mac_cleanup(void)
{
	platform_driver_unregister(&ak98_mac_driver);
}

module_init(ak98_mac_init);
module_exit(ak98_mac_cleanup);

MODULE_AUTHOR("Tang Anyang, Zhang Jingyuan (C) ANYKA");
MODULE_DESCRIPTION("AK98 MAC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ak98_mac");
