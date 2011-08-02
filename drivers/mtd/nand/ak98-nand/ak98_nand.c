/* linux/drivers/mtd/nand/ak98.c
 *
 * Anyka ak98 NAND driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifdef CONFIG_MTD_NAND_ak98_DEBUG
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <mach-anyka/anyka_types.h>

#include <asm/io.h>
#include <mach/nand.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include "arch_nand.h"
#include "wrap_nand.h"
#include "anyka_cpu.h"

#define ZZ_DEBUG                 0

//receive mtd command
static unsigned int m_nCommand = 0;
int page_shift;
static unsigned long nandid = 0xFFFFFFFF;

/* struct semaphore m_sem_lock; */
DECLARE_MUTEX(nand_lock);
EXPORT_SYMBOL(nand_lock);

unsigned char m_status = 0;
dma_addr_t dmahandle;
void *bufaddr = NULL;
extern T_NAND_PHY_INFO *g_pNand_Phy_Info;


/* controller and mtd information */

struct ak98_nand_info {
	/* mtd info */
	struct nand_hw_control controller;
	struct ak98_nand_mtd *mtds;
	struct ak98_platform_nand *platform;

	/* device info */
	struct device *device;
#ifdef CONFIG_CPU_FREQ
	struct notifier_block freq_transition;
#endif
	unsigned int cmd_len;
	unsigned int data_len;
	int mtd_count;
};

struct ak98_nand_mtd {
	struct mtd_info mtd;
	struct nand_chip chip;
	struct ak98_nand_set *set;
	struct ak98_nand_info *info;
	int scan_res;
};

extern T_PNandflash_Add g_pNF;
struct mtd_info *g_master = NULL; //zhangzheng add for nand char dev mount mtd partitions

int print_32(unsigned char *buf)
{	
	int i,j;
	unsigned char *p0;
	p0=buf;
	printk("print_32,buf=0x%x: ",(int)p0);
 
	for(i=0;i<1;i++){
		for(j=0;j<32;j++){
			printk("%02x,",*(p0+j+i*32));
		}
	printk("\n");
	}

return i*j;	
}

static struct ak98_nand_info *to_nand_info(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static struct ak98_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}

/* NFC & HW port setup
 *
 * setting NFC, AC
 *
 * NFC share pin set
 *	Enable nandflash function pin		: 0x08000074 [4:3] = 0x01
 *	Share pin use nandflash :
 *	CE0,RE,WE,CLE,ALE,data0 -- data7	: 0x08000078 [16]=1 [17]=1 [18]=1
 *	R/B					: 0x08000078 [22]=1
 *	pull up and pull down			: use the defalut value
 */
void ak98_nand_inithw(void)
{
	/* open the nand flash clk */
    printk(KERN_INFO "%s: line %d\n",__func__,__LINE__);
	*(volatile unsigned int *)(AK98_VA_SYSCTRL + 0x0c) &= (~(1 << 14));

	nand_HWinit();
   
	/* FIXME: use configurable timing parameter */
	//*(volatile unsigned int *)(AK98_VA_NFCTRL + 0x15C) = 0xF5AD1;
}

static unsigned int index = 0;
static u_char ak98_nand_read_byte(struct mtd_info *mtd)
{
	u_char ret_byte = 0;
    
    if (m_nCommand == NAND_CMD_STATUS) 
    {
		index = 0;
		return m_status;
	}

	if (m_nCommand == NAND_CMD_READID) 
    {
    	#if ZZ_DEBUG
        printk(KERN_INFO "zz ak98.c ak98_nand_read_byte() 178 index=%d nandid=0x%lx\n", 
               index, nandid);
		#endif
		ret_byte = ((nandid >> index) & 0xFF);
		if (index == 24)
			index = 0;
		else
			index += 8;
	}

	if (m_nCommand == NAND_CMD_READOOB)
    {
		//printk("%s: No implement yet for READOOB\n", __FUNCTION__);
		//printk("%s: please create BBT first\n", __FUNCTION__);
 	}
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_read_byte() 194 ret_byte=%d\n",
           ret_byte);
	#endif
	return ret_byte;
}

static void ak98_nand_command(struct mtd_info *mtd,
				unsigned command, int column, int page_addr)
{
    int block_index = 0;
	m_nCommand = command;
	page_shift = page_addr;

	//printk(KERN_INFO "zz ak98.c ak98_nand_command() line 304 page=%d\n",page_addr);
	switch (command) {
	case NAND_CMD_RESET:
		//printk(KERN_INFO "zz ak98.c ak98_nand_command() line 307 CMD:NAND_CMD_RESET\n");
		nand_reset(0);
		break;

	case NAND_CMD_STATUS:
		//printk(KERN_INFO "zz ak98.c ak98_nand_command() line 312 CMD:NAND_CMD_STATUS\n");
		m_status = nand_get_status(0);
		break;

	case NAND_CMD_READID:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 317 CMD:NAND_CMD_READID\n");
		index = 0;
		nandid = nand_read_chipID(0);	/* only support 1st flash */
 		break;
	case NAND_CMD_ERASE1:
		if(g_pNF == NULL)
		{
			printk(KERN_INFO "zz ak98.c ak98_nand_command() 227 g_pNF==NULL error!!!\n");
			break;
		}
		block_index = page_addr/128;
        nand_eraseblock(0, block_index*128, g_pNF);
		break;

	case NAND_CMD_ERASE2:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 327 CMD:NAND_CMD_ERASE2\n");
		break;
        
	case NAND_CMD_READOOB:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 331 CMD:NAND_CMD_READOOB\n");
		break;
        
	case NAND_CMD_READ0:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 335 CMD:NAND_CMD_READ0\n");
		break;
        
	case NAND_CMD_PAGEPROG:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 339 CMD:NAND_CMD_PAGEPROG\n");
		break;
        
	case NAND_CMD_SEQIN:
        //printk(KERN_INFO "zz ak98.c ak98_nand_command() line 343 CMD:NAND_CMD_SEQIN\n");
		break;

	default:
		//printk(KERN_INFO "zz ak98.c ak98_nand_command() line 347 Unkown CMD:0x%x\n", command);
		return;
	}

}
#if 0
#if CONFIG_MTD_PARTITIONS
static int ak98_nand_add_partition(struct ak98_nand_info *info,
				     struct ak98_nand_mtd *mtd,
				     struct ak98_nand_set *set)
{
	if (set == NULL)
    {   
    	#if ZZ_DEBUG
        printk(KERN_INFO "zz ak98.c ak98_nand_add_partition() 269\n");
		#endif
		return add_mtd_device(&mtd->mtd);
    }
	if (set->nr_partitions > 0 && set->partitions != NULL) 
    {
    	#if ZZ_DEBUG
        printk(KERN_INFO "zz ak98.c ak98_nand_add_partition() 276\n");
		#endif
		return add_mtd_partitions(&mtd->mtd, set->partitions,
					  set->nr_partitions);
	}
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_add_partition() 282\n");
	#endif
	return add_mtd_device(&mtd->mtd);
}
#else
static int ak98_nand_add_partition(struct ak98_nand_info *info,
				     struct ak98_nand_mtd *mtd,
				     struct ak98_nand_set *set)
{
	return add_mtd_device(&mtd->mtd);
}
#endif
#endif
static uint8_t ak98_bbt_pattern[] = { 'b', 'b', 't' };

static struct nand_bbt_descr ak98_nand_bbt_descr = {
	//.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	//    | NAND_BBT_1BIT,
	.options = NAND_BBT_ABSPAGE|NAND_BBT_VERSION | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_1BIT,
	.pages[0] = 127,
	.offs = 13,
	.len = 3,
	.maxblocks = 4,
	//.reserved_block_code = 1,
	.pattern = ak98_bbt_pattern
};

/* ak98_nand_init_chip
 *
 * init a single instance of an chip
 */

static void ak98_nand_init_chip(struct ak98_nand_info *info,
				  struct ak98_nand_mtd *nmtd,
				  struct ak98_nand_set *set)
{
	struct nand_chip *chip = &nmtd->chip;
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_init_chip() 321\n");
	#endif
	chip->write_buf = ak_nand_write_buf;     //ak98_nand_write_buf;
	chip->read_buf =  ak_nand_read_buf;      //ak98_nand_read_buf;   
	chip->select_chip = ak_nand_select_chip;//ak_nand_select_chip; 
	chip->chip_delay = 50;
	chip->priv = nmtd;
	chip->options = 0;
	chip->options = NAND_NO_PADDING | NAND_NO_AUTOINCR | NAND_USE_FLASH_BBT;	// | NAND_SKIP_BBTSCAN;
	chip->controller = &info->controller;
	chip->dev_ready = NULL;
	chip->cmdfunc = ak98_nand_command;
	chip->read_byte = ak98_nand_read_byte;
	chip->bbt_td = &ak98_nand_bbt_descr;

    chip->ecc.mode = NAND_ECC_HW;      //NAND_ECC_HW;
	chip->ecc.read_page = ak_nand_read_page_hwecc; 
	chip->ecc.write_page = ak_nand_write_page_hwecc; 
	chip->ecc.read_page_raw = ak_nand_read_page_raw;
	chip->ecc.write_page_raw = ak_nand_write_page_raw;
	chip->ecc.read_oob = ak_nand_read_oob;  
	chip->ecc.write_oob = ak_nand_write_oob;
	chip->init_size = ak_nand_init_size;
	chip->scan_bbt = ak_nand_scan_bbt;
	chip->block_bad = ak_nand_block_isbad;
	chip->block_markbad = ak_nand_block_markbad;
    
	nmtd->info = info;
	nmtd->mtd.priv = chip;
	nmtd->mtd.owner = THIS_MODULE;
	nmtd->set = set;
	#if ZZ_DEBUG
	printk("ak98_nand_init_chip/cmd_timing=0x%x,data_timing=0x%x\n",nmtd->set->cmd_len,nmtd->set->data_len);
	#endif
	#if defined(L2_DMA_MODE)
		printk("NAND driver:L2_DMA_MODE\n");
    #else
		printk("NAND driver:L2_CPU_MODE\n");
	#endif

}

#ifdef CONFIG_CPU_FREQ
static int ak98_nand_cpufreq_transition(struct notifier_block *nb, 
						unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *)data;
	struct ak98_nand_info *info = container_of(nb, struct ak98_nand_info, freq_transition);	
	unsigned int old_clk = freqs->old_cpufreq.asic_clk;
	unsigned int new_clk = freqs->new_cpufreq.asic_clk;
	unsigned long flags;

	if ((val == CPUFREQ_PRECHANGE && new_clk > old_clk) ||
		(val == CPUFREQ_POSTCHANGE && new_clk < old_clk)) {
		down(&nand_lock);
		local_irq_save(flags);

		nand_calctiming(info->data_len);
		nand_changetiming(new_clk);

		local_irq_restore(flags);
		up(&nand_lock);
	}
	return NOTIFY_DONE;
}

static int ak98_nand_cpufreq_register(struct ak98_nand_info *info)
{
	info->freq_transition.notifier_call = ak98_nand_cpufreq_transition;

	return cpufreq_register_notifier(&info->freq_transition, 
					CPUFREQ_TRANSITION_NOTIFIER); 
}

static void ak98_nand_cpufreq_unregister(struct ak98_nand_info *info)
{
	cpufreq_unregister_notifier(&info->freq_transition,
					CPUFREQ_TRANSITION_NOTIFIER);
}
#else 
static int ak98_nand_cpufreq_register(struct ak98_nand_info *info)
{
	return 0;
}

static void ak98_nand_cpufreq_unregister(struct ak98_nand_info *info)
{
}
#endif

static void ak98_nand_clock_set(struct ak98_nand_info *info)
{
	info->cmd_len = g_pNand_Phy_Info->cmd_len;
	info->data_len = g_pNand_Phy_Info->data_len;
	nand_calctiming(info->data_len);
	nand_changetiming(ak98_get_asic_clk());
}


/* device management functions */

static int ak98_nand_remove(struct platform_device *pdev)
{
	struct ak98_nand_info *info = to_nand_info(pdev);
	#if ZZ_DEBUG
	printk(KERN_INFO "zz ak98.c ak98_nand_remove() line 132\n");
	#endif
	platform_set_drvdata(pdev, NULL);

	if (info == NULL)
		return 0;

	/* first thing we need to do is release all our mtds
	 * and their partitions, then go through freeing the
	 * resources used
	 */

	ak98_nand_cpufreq_unregister(info);

	if (info->mtds != NULL) {
		struct ak98_nand_mtd *ptr = info->mtds;
		int mtdno;

		for (mtdno = 0; mtdno < info->mtd_count; mtdno++, ptr++) {
		    pr_debug("releasing mtd %d (%p)\n", mtdno, ptr);
			nand_release(&ptr->mtd);
		}

		kfree(info->mtds);
	}

	/* close flash contoller clock */
	*(volatile unsigned int *)(AK98_VA_SYSCTRL + 0x0c) |= (1 << 14);

	kfree(info);
	free_globe_para();
	
#ifdef CONFIG_MTD_NAND_TEST 
	nand_balance_test_exit();
#endif

	return 0;
}

static int __init ak98_nand_probe(struct platform_device *pdev)
{
	struct ak98_platform_nand *plat = to_nand_plat(pdev);
	struct ak98_nand_info *info;
	struct ak98_nand_mtd *nmtd;
	struct ak98_nand_set *sets;
	int err = 0;
	int size;
	int nr_sets;
	int setno;
	
	printk(KERN_INFO "zz ak98.c ak98_nand_probe() 371\n");
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		printk(KERN_INFO "zz ak98.c ak98_nand_probe() 381 no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error1;
	}
	
	platform_set_drvdata(pdev, info);
	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->device = &pdev->dev;
	info->platform = plat;
	sets = (plat != NULL) ? plat->sets : NULL;
	nr_sets = (plat != NULL) ? plat->nr_sets : 1;

	info->mtd_count = nr_sets;

	/* allocate our information */

	size = nr_sets * sizeof(*info->mtds);
	info->mtds = kzalloc(size, GFP_KERNEL);
	if (info->mtds == NULL) {
		printk("failed to allocate mtd storage\n");
		err = -ENOMEM;
		goto exit_error2;
	}

	ak98_nand_inithw();	

	#ifndef CONFIG_MTD_DOWNLOAD_MODE
	if(FHA_FAIL == init_fha_lib())
	{
		printk(KERN_INFO "zz ak98.c ak98_nand_probe() 375 init fha lib error return!!!\n");
		return -ENOMEM;
	}	
	ak98_nand_clock_set(info);
	#endif

	#ifdef CONFIG_MTD_NAND_TEST   
    nand_balance_test_init();
	#endif	

	/* initialise all possible chips */
	nmtd = info->mtds;
    g_master = &nmtd->mtd;//zhangzheng add for nand char dev mount mtd partitions
    #if ZZ_DEBUG
	printk(KERN_INFO "zz ak98.c ak98_nand_probe() 413 nr_sets=%d\n", nr_sets);
	#endif
	for (setno = 0; setno < nr_sets; setno++, nmtd++)   //nr_sets==1
	{
		pr_debug("initialising set %d (%p, info %p)\n", setno, nmtd,
			 info);

		ak98_nand_init_chip(info, nmtd, sets);

        #ifndef CONFIG_MTD_DOWNLOAD_MODE
		nmtd->scan_res = nand_scan(&nmtd->mtd, (sets) ? sets->nr_chips : 1);            
		ak_mount_partitions();
        #endif
        
		if (sets != NULL)
			sets++;
	}

    #if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_probe() 437 Initialize Successed!\n");
	#endif

	err = ak98_nand_cpufreq_register(info);
	if (err) {
		printk(KERN_INFO "ak98 nand cpu freq register fail\n");
		err = -EINVAL;
		goto exit_error3;
	}
		
	#if 0
    zz_test_nand();
   	erase_all_flash();
	#endif
	
	
	return 0;


exit_error3:
	kfree(info->mtds);
exit_error2:
    kfree(info);
exit_error1:
    printk(KERN_INFO "zz ak98.c ak98_nand_probe() 444 Error Exit!\n");
    ak98_nand_remove(pdev);
	return err;
}

/* PM Support */
#ifdef CONFIG_PM

static int ak98_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
    //struct ak98_nand_info *info = platform_get_drvdata(dev);
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_suspend() 456\n");    
	#endif	
    *(volatile unsigned int *)(AK98_VA_SYSCTRL + 0x0c) |= (1 << 14);
	return 0;
}

static int ak98_nand_resume(struct platform_device *dev)
{
	struct ak98_nand_info *info = platform_get_drvdata(dev);
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_resume() 465\n");
	#endif
	if (info) 
	{
		ak98_nand_inithw();
	}
	return 0;
}

#else
#define ak98_nand_suspend NULL
#define ak98_nand_resume NULL
#endif

/* driver device registration */

static struct platform_driver ak98_nand_driver = {
	.probe = ak98_nand_probe,
	.remove = ak98_nand_remove,
	.suspend = ak98_nand_suspend,
	.resume = ak98_nand_resume,
	.driver = {
		   .name = "ak98-nand",
		   .owner = THIS_MODULE,
		   },
};

static int __init ak98_nand_init(void)
{
	#if ZZ_DEBUG
	printk(KERN_INFO "zz ak98.c ak98_nand_init() line 495\n");
	#endif

	bufaddr = dma_alloc_coherent(NULL, NAND_DATA_SIZE_P1KB, &dmahandle, GFP_KERNEL);
	if (bufaddr == NULL) {
		printk(KERN_INFO "dma alloc coherent fail!\n");
		WARN_ON(1);
	}
	
	return platform_driver_register(&ak98_nand_driver);
}

static void __exit ak98_nand_exit(void)
{
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak98.c ak98_nand_exit() 506\n");
	#endif
	if (bufaddr != NULL)
	{
	    dma_free_coherent(NULL, NAND_DATA_SIZE_P1KB, bufaddr, dmahandle);
	}
	
	platform_driver_unregister(&ak98_nand_driver);
}
//--------------------------------print info---------------------------------
void print_struct_mtd_info(struct mtd_info *mtd)
{
    printk(KERN_INFO "---------------------zz print struct mtd_info start-------------------------------\n");
    printk(KERN_INFO "erasesize=%d\nerasesize_mask=%d\nwritesize=%d\noobsize=%d\n",           
           mtd->erasesize,
           mtd->erasesize_mask,
           mtd->writesize, 
           mtd->oobsize);
    printk(KERN_INFO "oobavail=%d\nerasesize_shift=%d\nwritesize_shift=%d\n",
           mtd->oobavail,
           mtd->erasesize_shift,
           mtd->writesize_shift);
    printk(KERN_INFO "erasesize_mask=%d\nwritesize_mask=%d\n",
           mtd->erasesize_mask,
           mtd->writesize_mask);
    printk(KERN_INFO "---------------------zz print struct mtd_info end---------------------------------\n");
}

void print_struct_nand_chip(struct nand_chip *chip)
{
    printk(KERN_INFO "---------------------zz print struct nand_chip start-------------------------------\n");
    printk(KERN_INFO "chip_delay=%d\noptions=%d\npage_shift=%d\nphys_erase_shift=%d\n",           
           chip->chip_delay,
           chip->options,
           chip->page_shift,
           chip->phys_erase_shift);
    printk(KERN_INFO "bbt_erase_shift=%d\nchip_shift=%d\nnumchips=%d\npagemask=%d\n",           
           chip->bbt_erase_shift,
           chip->chip_shift,
           chip->numchips,
           chip->pagemask);
    printk(KERN_INFO "pagebuf=%d\nsubpagesize=%d\ncellinfo=%d\nbadblockpos=%d\nstate=%d\n",           
           chip->pagebuf,
           chip->subpagesize,
           chip->cellinfo,
           chip->badblockpos,
           chip->state);
    printk(KERN_INFO "--------------------zz print struct nand_chip end---------------------------------\n");
}

//-----------------------------------zhangzheng add end-------------------------------------------------
module_init(ak98_nand_init);
module_exit(ak98_nand_exit);

MODULE_AUTHOR("anyka");
MODULE_DESCRIPTION("ak98 MTD NAND driver");
MODULE_LICENSE("GPL");

