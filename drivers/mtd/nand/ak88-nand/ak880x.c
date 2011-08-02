/* linux/drivers/mtd/nand/ak880x.c
 *
 * Anyka ak880x NAND driver
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

#ifdef CONFIG_MTD_NAND_AK880X_DEBUG
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

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <mach-anyka/anyka_types.h>

#include <asm/io.h>
#include <mach/nand.h>
#include <mach/map.h>
#include "nand_flash_drv.h"
#include "arch_nand.h"
#include "wrap_nand.h"

#define ZZ_DEBUG                 0

//receive mtd command
static unsigned int m_nCommand = 0;
int page_shift;
static unsigned long nandid = 0xFFFFFFFF;
#define SRDPIN_USE_MUTEX

/* struct semaphore m_sem_lock; */
#ifdef SRDPIN_USE_MUTEX
DEFINE_MUTEX(nand_lock);
#else
DECLARE_MUTEX(nand_lock);
#endif
EXPORT_SYMBOL(nand_lock);
unsigned char m_status = 0;

/* controller and mtd information */

struct ak880x_nand_info {
	/* mtd info */
	struct nand_hw_control controller;
	struct ak880x_nand_mtd *mtds;
	struct ak880x_platform_nand *platform;

	/* device info */
	struct device *device;
	struct clk *clk;
	int mtd_count;
};

struct ak880x_nand_mtd {
	struct mtd_info mtd;
	struct nand_chip chip;
	struct ak880x_nand_set *set;
	struct ak880x_nand_info *info;
	int scan_res;
};

extern T_PNandflash_Add g_pNF;

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

static struct ak880x_nand_info *to_nand_info(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static struct ak880x_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}
/* select chip */
static void ak880x_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct ak880x_nand_mtd *nmtd;
	struct nand_chip *this = mtd->priv;
	nmtd = this->priv;
	ak880x_nand_settiming(nmtd->set->cmd_len, nmtd->set->data_len);
}

/* device management functions */

static int ak880x_nand_remove(struct platform_device *pdev)
{
	struct ak880x_nand_info *info = to_nand_info(pdev);
	#if ZZ_DEBUG
	printk(KERN_INFO "zz ak880x.c ak880x_nand_remove() line 132\n");
	#endif
	platform_set_drvdata(pdev, NULL);

	if (info == NULL)
		return 0;

	/* first thing we need to do is release all our mtds
	 * and their partitions, then go through freeing the
	 * resources used
	 */

	if (info->mtds != NULL) {
		struct ak880x_nand_mtd *ptr = info->mtds;
		int mtdno;

		for (mtdno = 0; mtdno < info->mtd_count; mtdno++, ptr++) {
		    pr_debug("releasing mtd %d (%p)\n", mtdno, ptr);
			nand_release(&ptr->mtd);
		}

		kfree(info->mtds);
	}

	/* close flash contoller clock */
	*(volatile unsigned int *)(AK88_VA_SYSCTRL + 0x0c) |= (1 << 13);

	kfree(info);

	return 0;
}

static unsigned int index = 0;
static u_char ak880x_nand_read_byte(struct mtd_info *mtd)
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
        printk(KERN_INFO "zz ak880x.c ak880x_nand_read_byte() 178 index=%d nandid=0x%lx\n", 
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
    printk(KERN_INFO "zz ak880x.c ak880x_nand_read_byte() 194 ret_byte=%d\n",
           ret_byte);
	#endif
	return ret_byte;
}

static void ak880x_nand_command(struct mtd_info *mtd,
				unsigned command, int column, int page_addr)
{
    int block_index = 0;
	m_nCommand = command;
	page_shift = page_addr;

	//printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 304 page=%d\n",page_addr);
	switch (command) {
	case NAND_CMD_RESET:
		//printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 307 CMD:NAND_CMD_RESET\n");
		ak880x_nand_reset(0, 0);
		break;

	case NAND_CMD_STATUS:
		//printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 312 CMD:NAND_CMD_STATUS\n");
		m_status = ak880x_nand_get_status(0);
		break;

	case NAND_CMD_READID:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 317 CMD:NAND_CMD_READID\n");
		index = 0;
		nandid = ak880x_nand_get_chipid(0);	/* only support 1st flash */
 		break;
	case NAND_CMD_ERASE1:
		if(g_pNF == NULL)
		{
			printk(KERN_INFO "zz ak880x.c ak880x_nand_command() 227 g_pNF==NULL error!!!\n");
			break;
		}
		block_index = page_addr/128;
        nand_eraseblock(0, block_index*128, g_pNF);
		break;

	case NAND_CMD_ERASE2:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 327 CMD:NAND_CMD_ERASE2\n");
		break;
        
	case NAND_CMD_READOOB:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 331 CMD:NAND_CMD_READOOB\n");
		break;
        
	case NAND_CMD_READ0:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 335 CMD:NAND_CMD_READ0\n");
		break;
        
	case NAND_CMD_PAGEPROG:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 339 CMD:NAND_CMD_PAGEPROG\n");
		break;
        
	case NAND_CMD_SEQIN:
        //printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 343 CMD:NAND_CMD_SEQIN\n");
		break;

	default:
		//printk(KERN_INFO "zz ak880x.c ak880x_nand_command() line 347 Unkown CMD:0x%x\n", command);
		return;
	}

}
#if 0
#if CONFIG_MTD_PARTITIONS
static int ak880x_nand_add_partition(struct ak880x_nand_info *info,
				     struct ak880x_nand_mtd *mtd,
				     struct ak880x_nand_set *set)
{
	if (set == NULL)
    {   
    	#if ZZ_DEBUG
        printk(KERN_INFO "zz ak880x.c ak880x_nand_add_partition() 269\n");
		#endif
		return add_mtd_device(&mtd->mtd);
    }
	if (set->nr_partitions > 0 && set->partitions != NULL) 
    {
    	#if ZZ_DEBUG
        printk(KERN_INFO "zz ak880x.c ak880x_nand_add_partition() 276\n");
		#endif
		return add_mtd_partitions(&mtd->mtd, set->partitions,
					  set->nr_partitions);
	}
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_add_partition() 282\n");
	#endif
	return add_mtd_device(&mtd->mtd);
}
#else
static int ak880x_nand_add_partition(struct ak880x_nand_info *info,
				     struct ak880x_nand_mtd *mtd,
				     struct ak880x_nand_set *set)
{
	return add_mtd_device(&mtd->mtd);
}
#endif
#endif
static uint8_t ak880x_bbt_pattern[] = { 'b', 'b', 't' };

static struct nand_bbt_descr ak880x_nand_bbt_descr = {
	//.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
	//    | NAND_BBT_1BIT,
	.options = NAND_BBT_ABSPAGE|NAND_BBT_VERSION | NAND_BBT_CREATE | NAND_BBT_WRITE
	    | NAND_BBT_1BIT,
	.pages[0] = 127,
	.offs = 13,
	.len = 3,
	.maxblocks = 4,
	//.reserved_block_code = 1,
	.pattern = ak880x_bbt_pattern
};

/* ak880x_nand_init_chip
 *
 * init a single instance of an chip
*/

static void ak880x_nand_init_chip(struct ak880x_nand_info *info,
				  struct ak880x_nand_mtd *nmtd,
				  struct ak880x_nand_set *set)
{
	struct nand_chip *chip = &nmtd->chip;
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_init_chip() 321\n");
	#endif
	chip->write_buf = ak_nand_write_buf;     //ak880x_nand_write_buf;
	chip->read_buf =  ak_nand_read_buf;      //ak880x_nand_read_buf;   
	chip->select_chip = ak880x_nand_select_chip;//ak_nand_select_chip; 
	chip->chip_delay = 50;
	chip->priv = nmtd;
	chip->options = 0;
	chip->options = NAND_NO_PADDING | NAND_NO_AUTOINCR | NAND_USE_FLASH_BBT;	// | NAND_SKIP_BBTSCAN;
	chip->controller = &info->controller;
	chip->dev_ready = NULL;
	chip->cmdfunc = ak880x_nand_command;
	chip->read_byte = ak880x_nand_read_byte;
	chip->bbt_td = &ak880x_nand_bbt_descr;

    chip->ecc.mode = NAND_ECC_HW;      //NAND_ECC_HW;
	chip->ecc.read_page = ak_nand_read_page_hwecc; 
	chip->ecc.write_page = ak_nand_write_page_hwecc; 
	chip->ecc.read_page_raw = ak_nand_read_page_raw;
	chip->ecc.write_page_raw = ak_nand_write_page_raw;
	chip->ecc.read_oob = ak_nand_read_oob;  
	chip->ecc.write_oob = ak_nand_write_oob;
    
	nmtd->info = info;
	nmtd->mtd.priv = chip;
	nmtd->mtd.owner = THIS_MODULE;
	nmtd->set = set;
	#if ZZ_DEBUG
	printk("ak880x_nand_init_chip/cmd_timing=0x%x,data_timing=0x%x\n",nmtd->set->cmd_len,nmtd->set->data_len);
	#endif
	#if defined(L2_DMA_MODE)
		printk("NAND driver:L2_DMA_MODE\n");
    #else
		printk("NAND driver:L2_CPU_MODE\n");
	#endif

}

struct mtd_info *g_master = NULL; //zhangzheng add for nand char dev mount mtd partitions

static int __init ak880x_nand_probe(struct platform_device *pdev)
{
	struct ak880x_platform_nand *plat = to_nand_plat(pdev);
	struct ak880x_nand_info *info;
	struct ak880x_nand_mtd *nmtd;
	struct ak880x_nand_set *sets;
	int err = 0;
	int size;
	int nr_sets;
	int setno;
	
	printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 371\n");
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 381 no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}
	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	ak880x_nand_inithw();

	#ifndef CONFIG_MTD_DOWNLOAD_MODE
	if(FHA_FAIL == init_fha_lib())
	{
		printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 375 init fha lib error return!!!\n");
		return -ENOMEM;
	}
	
	if(AK_FALSE == init_globe_para())
	{
		printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 380 init globe para error return!!!\n");
		return -ENOMEM;
	}
	#endif

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
		goto exit_error;
	}

	/* initialise all possible chips */
	nmtd = info->mtds;
    g_master = &nmtd->mtd;//zhangzheng add for nand char dev mount mtd partitions
    #if ZZ_DEBUG
	printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 413 nr_sets=%d\n", nr_sets);
	#endif
	for (setno = 0; setno < nr_sets; setno++, nmtd++)   //nr_sets==1
	{
		pr_debug("initialising set %d (%p, info %p)\n", setno, nmtd,
			 info);

		ak880x_nand_init_chip(info, nmtd, sets);

		nmtd->scan_res = nand_scan(&nmtd->mtd, (sets) ? sets->nr_chips : 1);
        
        #ifndef CONFIG_MTD_DOWNLOAD_MODE
		ak_mount_partitions();
        #endif
        
		if (sets != NULL)
			sets++;
	}
    #if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 437 Initialize Successed!\n");
	#endif
	
	#if 0
    zz_test_nand();
   	erase_all_flash();
	#endif
	
	return 0;

 exit_error:
    printk(KERN_INFO "zz ak880x.c ak880x_nand_probe() 444 Error Exit!\n");
    ak880x_nand_remove(pdev);
	return err;
}

/* PM Support */
//#ifdef CONFIG_PM

static int ak880x_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
    //struct ak880x_nand_info *info = platform_get_drvdata(dev);
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_suspend() 456\n");
	#endif
	return 0;
}

static int ak880x_nand_resume(struct platform_device *dev)
{
	struct ak880x_nand_info *info = platform_get_drvdata(dev);
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_resume() 465\n");
	#endif
	if (info) 
	{
		ak880x_nand_inithw();
	}
	return 0;
}
/*
#else
#define ak880x_nand_suspend NULL
#define ak880x_nand_resume NULL
#endif
*/
/* driver device registration */

static struct platform_driver ak880x_nand_driver = {
	.probe = ak880x_nand_probe,
	.remove = ak880x_nand_remove,
	.suspend = ak880x_nand_suspend,
	.resume = ak880x_nand_resume,
	.driver = {
		   .name = "ak880x-nand",
		   .owner = THIS_MODULE,
		   },
};

static int __init ak880x_nand_init(void)
{
	#if ZZ_DEBUG
	printk(KERN_INFO "zz ak880x.c ak880x_nand_init() line 495\n");
	#endif
	/* init_MUTEX(&m_sem_lock); */
	/* mutex_init(&nand_lock); */
   
	return platform_driver_register(&ak880x_nand_driver);
}

static void __exit ak880x_nand_exit(void)
{
	#if ZZ_DEBUG
    printk(KERN_INFO "zz ak880x.c ak880x_nand_exit() 506\n");
	#endif
	platform_driver_unregister(&ak880x_nand_driver);
}
//--------------------------------zhangzheng add start---------------------------------
void zz_print_struct_mtd_info(struct mtd_info *mtd)
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

void zz_print_struct_nand_chip(struct nand_chip *chip)
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
module_init(ak880x_nand_init);
module_exit(ak880x_nand_exit);

MODULE_AUTHOR("anyka");
MODULE_DESCRIPTION("AK880X MTD NAND driver");
MODULE_LICENSE("GPL");

