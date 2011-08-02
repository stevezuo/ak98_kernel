/*
 * AK98 frequency conversion policy layer
 *
 * Copyright (C) 2011 ANYKA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/ak98_freq_policy.h>
#include <linux/semaphore.h>
#include <linux/delay.h>



//#define FREQ_DEBUG

#undef PDEBUG           /* undef it, just in case */
#ifdef FREQ_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
# define PDEBUG(fmt, args...) printk( KERN_INFO fmt,## args)
# else
/* This one for user space */
# define PDEBUG(fmt, args...) fprintf(stderr, "%s %d: "fmt,__FILE__, __LINE__, ## args)
# endif
#else
# define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif


struct t_app_to_mode app_to_mode[] = {
		{"browser", NORMAL_MODE_CLOCK_5},
		{"Email", NORMAL_MODE_CLOCK_5},
		{"LauncherApp", NORMAL_MODE_CLOCK_3},
		{"PartialWakeLockApp", LOW_MODE_CLOCK_1},
		{"ScreenDimWakeLockApp", NORMAL_MODE_CLOCK_3},
		{"ScreenBrightWakeLockApp", NORMAL_MODE_CLOCK_5},
		{"FullWakeLockApp", NORMAL_MODE_CLOCK_5},
		{"WifiService", NORMAL_MODE_CLOCK_5},
		{"low_test0",LOW_MODE_CLOCK_0},
		{"AudioOutLock", LOW_MODE_CLOCK_1},
		{"CameraService", NORMAL_MODE_CLOCK_5},
		{"MediaRecorder",NORMAL_MODE_CLOCK_5},
		{"WindowRotate",NORMAL_MODE_CLOCK_5},
		{"MonitorCpuLow", NORMAL_MODE_CLOCK_3},
		{"MonitorCpuHigh", NORMAL_MODE_CLOCK_5},
		{"low_test2", LOW_MODE_CLOCK_2},
		{"low_test3", LOW_MODE_CLOCK_3},
		{"low_test4", LOW_MODE_CLOCK_4},
		{"low_test5", LOW_MODE_CLOCK_5},
		{"low_test6", LOW_MODE_CLOCK_6},
		{"low_test7", LOW_MODE_CLOCK_7},
		{"normal_test0", NORMAL_MODE_CLOCK_0},
		{"normal_test1", NORMAL_MODE_CLOCK_1},
		{"normal_test2", NORMAL_MODE_CLOCK_2},
		{"normal_test3", NORMAL_MODE_CLOCK_3},
		{"normal_test4", NORMAL_MODE_CLOCK_4},
		{"normal_test5", NORMAL_MODE_CLOCK_5},
		{"normal_test6", NORMAL_MODE_CLOCK_6},
		{"normal_test7", NORMAL_MODE_CLOCK_7},
		{"video_test0", VIDEO_MODE_CLOCK_0},
		{"video_test1", VIDEO_MODE_CLOCK_1},
		{"video_test2", VIDEO_MODE_CLOCK_2},
		{"video_test3", VIDEO_MODE_CLOCK_3},
		{"video_test4", VIDEO_MODE_CLOCK_4},
		{"video_test5", VIDEO_MODE_CLOCK_5},
		{"video_test6", VIDEO_MODE_CLOCK_6},
		{"video_test7", VIDEO_MODE_CLOCK_7},		
		{"bkmusic_test", BK_MUSIC_MODE},	
};



struct mode_table_node name_to_mode[] = {
		{"LOW_MODE_CLOCK_0", LOW_MODE_CLOCK_0},
		{"LOW_MODE_CLOCK_1", LOW_MODE_CLOCK_1},
		{"LOW_MODE_CLOCK_2", LOW_MODE_CLOCK_2},
		{"LOW_MODE_CLOCK_3", LOW_MODE_CLOCK_3},
		{"LOW_MODE_CLOCK_4", LOW_MODE_CLOCK_4},
		{"LOW_MODE_CLOCK_5", LOW_MODE_CLOCK_5},
		{"LOW_MODE_CLOCK_6", LOW_MODE_CLOCK_6},
		{"LOW_MODE_CLOCK_7", LOW_MODE_CLOCK_7},
		{"NORMAL_MODE_CLOCK_0", NORMAL_MODE_CLOCK_0},
		{"NORMAL_MODE_CLOCK_1", NORMAL_MODE_CLOCK_1},
		{"NORMAL_MODE_CLOCK_2", NORMAL_MODE_CLOCK_2},
		{"NORMAL_MODE_CLOCK_3", NORMAL_MODE_CLOCK_3},
		{"NORMAL_MODE_CLOCK_4", NORMAL_MODE_CLOCK_4},
		{"NORMAL_MODE_CLOCK_5", NORMAL_MODE_CLOCK_5},
		{"NORMAL_MODE_CLOCK_6", NORMAL_MODE_CLOCK_6},
		{"NORMAL_MODE_CLOCK_7", NORMAL_MODE_CLOCK_7},
		{"VIDEO_MODE_CLOCK_0", VIDEO_MODE_CLOCK_0},
		{"VIDEO_MODE_CLOCK_1", VIDEO_MODE_CLOCK_1},
		{"VIDEO_MODE_CLOCK_2", VIDEO_MODE_CLOCK_2},
		{"VIDEO_MODE_CLOCK_3", VIDEO_MODE_CLOCK_3},
		{"VIDEO_MODE_CLOCK_4", VIDEO_MODE_CLOCK_4},
		{"VIDEO_MODE_CLOCK_5", VIDEO_MODE_CLOCK_5},
		{"VIDEO_MODE_CLOCK_6", VIDEO_MODE_CLOCK_6},
		{"VIDEO_MODE_CLOCK_7", VIDEO_MODE_CLOCK_7},
};
const char *undefined_mode = "Undefined mode";


/**************************************************************/


static void change_work(struct work_struct *work);
static const char* get_mode_name(T_MODE_TYPE mode);
static int is_higher_mode(T_MODE_TYPE mode1, T_MODE_TYPE mode2);
static int change_mode(T_MODE_TYPE mode );
T_MODE_TYPE get_target_mode(void);
static T_MODE_TYPE get_mode_by_name(char *name);

#define MAX_HOLD_MODE_PENDING 20
struct callback_params {
	AK98_RQSTMODE_CALLBACK fn;
	void *data;
};


static struct callback_params g_param_queue[MAX_HOLD_MODE_PENDING];
static int g_head, g_tail;
static struct rqst_node g_rqst_list[NUM_OF_MODE];
static int g_hold_modes[NUM_OF_MODE] = {0};
/* mode under which  system is running*/
static T_MODE_TYPE g_current_mode;
/* mode under which system should be running , it's value is calculated according to all the mode requests*/
static T_MODE_TYPE g_target_mode;
static unsigned int g_current_pll;
static unsigned int g_target_pll;

static int g_tag_cnt=0;
//static struct timer_list change_timer = TIMER_INITIALIZER(change_timer_handler, 0, 0);

static struct workqueue_struct *g_freq_policy_wq;
static struct delayed_work	g_work;
static struct delayed_work  g_hold_mode_work;
static struct semaphore g_sem;
static int pll_change = 0;


static void judge_and_change(void)
{
	g_target_mode = get_target_mode();
	g_target_pll = get_pll_sel(g_target_mode);
	PDEBUG("current mode: %s   target mode: %s\n", get_mode_name(g_current_mode), get_mode_name(g_target_mode));

    /* response immediately when higher mode request is coming */	
	if (is_higher_mode(g_target_mode, g_current_mode))
	{
		//del_timer(&change_timer);
		cancel_delayed_work(&g_work);
		if (g_target_pll != g_current_pll)
		{
			pll_change = 1;
			queue_delayed_work(g_freq_policy_wq, &g_work,
				      msecs_to_jiffies(PLL_CHANGE_DELAY));
		}
		else if (0 == change_mode(g_target_mode))
		{
			g_current_mode = g_target_mode;
			g_current_pll = g_target_pll;
		}
	}
	else if (is_higher_mode(g_current_mode, g_target_mode))
	{
		//del_timer(&change_timer);
		cancel_delayed_work(&g_work);
		queue_delayed_work(g_freq_policy_wq, &g_work,
				      msecs_to_jiffies(REQUEST_DELAY));
		//mod_timer(&change_timer, jiffies+ msecs_to_jiffies(REQUEST_DELAY));
	}
	
}

static void hold_mode_work(struct work_struct *work)
{	
	down(&g_sem);
	
	judge_and_change();	
	if (g_param_queue[g_head].fn)
		(*(g_param_queue[g_head].fn))(g_param_queue[g_head].data);
	g_head++;
	if (g_head == MAX_HOLD_MODE_PENDING)
		g_head = 0;
	
	up(&g_sem);
}


int ak98_request_hold_mode(T_MODE_TYPE mode, AK98_RQSTMODE_CALLBACK fn, 
	void *data)

{
	if (!(mode >= LOWEST_MODE && mode <= HIGHEST_MODE))
		panic("Wrong value passed to mode. --- %d\n", mode);
	
	g_hold_modes[mode]++;
	g_param_queue[g_tail].fn = fn;
	g_param_queue[g_tail].data = data;

	g_tail++;
	if (g_tail == g_head)
	{
		printk("Too many hold mode are pending.\n");
		return -1;
	}
	if (g_tail == MAX_HOLD_MODE_PENDING)
		g_tail = 0;
	queue_delayed_work(g_freq_policy_wq, &g_hold_mode_work,
				      0);	
	return 0;
}

EXPORT_SYMBOL(ak98_request_hold_mode);

int ak98_release_hold_mode(T_MODE_TYPE mode)
{
	if (!(mode >= LOWEST_MODE && mode <= HIGHEST_MODE))
		panic("Wrong value passed to mode. --- %d\n", mode);

	if (g_hold_modes[mode] <= 0)
		return -1;
	
	g_hold_modes[mode]--;
	PDEBUG("Release pid: %d  tag: %d\n", pid, tag);
	g_target_mode = get_target_mode();

	/* if not equal, g_target_mode must be lower than 
	 * g_current_mode (thats to say: g_target_mode <= g_current_mode)
	*/
	PDEBUG("target mode: %s\n", get_mode_name(g_target_mode));
	if (g_target_mode != g_current_mode)
	{
		cancel_delayed_work(&g_work);
		queue_delayed_work(g_freq_policy_wq, &g_work,
				      msecs_to_jiffies(RELEASE_DELAY));		
	}	
	return 0;
}

EXPORT_SYMBOL(ak98_release_hold_mode);

static const char* get_mode_name(T_MODE_TYPE mode)
{
	int i, n=ARRAY_SIZE(name_to_mode);

	if (name_to_mode[mode].mode == mode)
		return name_to_mode[mode].modeName;
	for (i=0; i<n; i++)
		if (name_to_mode[i].mode == mode)
			return name_to_mode[i].modeName;
	return undefined_mode;
}
static int is_higher_mode(T_MODE_TYPE mode1, T_MODE_TYPE mode2)
{
	if (HIGHEST_MODE > LOWEST_MODE)
		return mode1 > mode2;
	else
		return mode1 < mode2;
}
/*
T_OPERATION_MODE get_current_mode(void)
{
	return DEFAULT_MODE;
}
int request_cpufreq_enter(T_MODE_TYPE mode)
{
	PDEBUG("request mode %s\n", get_mode_name(mode));
	return 0;
}*/

static int change_mode(T_MODE_TYPE mode)
{

	return request_cpufreq_enter(mode);
}

static void change_work(struct work_struct *work)
{
	down(&g_sem);
	if (is_higher_mode(g_current_mode, g_target_mode))
	{	
		PDEBUG("change mode: from %s to %s\n", get_mode_name(g_current_mode), get_mode_name(g_target_mode));
		if (change_mode(g_target_mode) == 0)
		{
			g_current_mode = g_target_mode;
			g_current_pll = g_target_pll;
		}
	}
	else if (pll_change)
	{
		pll_change = 0;
		if (change_mode(g_target_mode) == 0)
		{
			g_current_mode = g_target_mode;
			g_current_pll = g_target_pll;
		}
	}
	up(&g_sem);
}

static ssize_t ak98_freq_policy_read(struct file *file, char *buf,
			     size_t count, loff_t * ppos)
{
	int i;
	struct rqst_node *cur;
	struct list_head *pos,*n;
	
	//spin_lock(&freq_policy_lock);
	down(&g_sem);
	printk("Hold mode:\n");
	for (i=HIGHEST_MODE; i>=LOWEST_MODE; --i)
	{
		if (g_hold_modes[i]>0)
			printk("%s  %d\n", get_mode_name(i), g_hold_modes[i]);
	}
	printk("---------------------\n");
	for (i=LOW_IX; i<= HIGH_IX; i++)
	{
		printk("%s(%d) --", get_mode_name(i),g_rqst_list[i].app_info.appName[0]);
		list_for_each_safe(pos, n, &(g_rqst_list[i].list))
		{
			cur = list_entry(pos, struct rqst_node, list);
			printk("%s(%d %d) ",cur->app_info.appName, cur->app_info.pid, cur->app_info.tag);			
		}
		printk("\n");
	}
	printk("current mode: %s  target mode: %s\n", get_mode_name(g_current_mode), 
		get_mode_name(g_target_mode));
	printk("bk music mode: %s\n", get_mode_name(BK_MUSIC_MODE));
	//spin_unlock(&freq_policy_lock);
	up(&g_sem);
	return 0;
}

/*
 * if video request and normal request are not empty, then target mode is max video request + 1
 * else if video request is not empty , then target mode is max video request
 * else if normal request is not empty, then target mode is max normal request
 * else target mode is max low request
 */
T_MODE_TYPE get_target_mode(void)
{
	int i;
	int low_max = -1;
	int normal_max = -1;
	int video_max = -1;
	//struct rqst_mode *cur;

	/*
		if mode requested by ak98_request_hold_mode exists, then keep the mode ignore 
	*/
	for (i=HIGHEST_MODE; i>=LOWEST_MODE; --i)
		if (g_hold_modes[i]>0)
			return i;

	for (i=LOW_MODE_CLOCK_MAX; i>=LOW_MODE_CLOCK_MIN; i--)
	{
		if (!list_empty(&(g_rqst_list[i].list)))
		{
			normal_max = i;
			break;
		}
	}	

	for (i=NORMAL_MODE_CLOCK_MAX; i>=NORMAL_MODE_CLOCK_MIN; i--)
	{
		if (!list_empty(&(g_rqst_list[i].list)))
		{
			normal_max = i;
			break;
		}
	}
	
	for (i=VIDEO_MODE_CLOCK_MAX; i>=VIDEO_MODE_CLOCK_MIN; i--)
	{
		if (!list_empty(&(g_rqst_list[i].list)))
		{
			video_max = i;
			break;
		}
	}

	if (normal_max >=NORMAL_DELTA_MIN && video_max != -1)
	{
		if (normal_max >= BK_MUSIC_MODE)
			return video_max+2 > VIDEO_MODE_CLOCK_MAX ? VIDEO_MODE_CLOCK_MAX: video_max+2;
		else
			return video_max+1 > VIDEO_MODE_CLOCK_MAX ? VIDEO_MODE_CLOCK_MAX: video_max+1;
		
	}
	else if (video_max != -1)
		return video_max;
	else if (normal_max != -1)
	{
		//have bk music mode request
		if (normal_max > BK_MUSIC_MODE && !list_empty(&(g_rqst_list[BK_MUSIC_MODE].list)))
			return normal_max+1 > NORMAL_MODE_CLOCK_MAX ? NORMAL_MODE_CLOCK_MAX:normal_max+1;
		return normal_max;
	}
	else if (low_max != -1) 
		return low_max;

	return DEFAULT_MODE;	
}


static int ak98_freq_policy_rqst_mode(char *app_name)
{
	pid_t pid = task_tgid_vnr(current);
	struct rqst_node *new_node;	
	T_MODE_TYPE t_rqst_mode;

	t_rqst_mode = get_mode_by_name(app_name);
	PDEBUG("name: %s pid: %d  request  mode: %s\n", app_name, pid, get_mode_name(t_rqst_mode));

	if ( (t_rqst_mode < LOW_IX) | (t_rqst_mode > HIGH_IX) )
		return -EINVAL;
	
	new_node = kzalloc(sizeof(struct rqst_node), GFP_KERNEL);
	if (new_node == NULL)
		return -ENOMEM;
	

	//spin_lock(&freq_policy_lock);
	down(&g_sem);
	strcpy(new_node->app_info.appName,app_name);
	new_node->app_info.pid = pid;
	//reserved,  count of requesting this mode
	g_rqst_list[t_rqst_mode].app_info.appName[0]++;
	new_node->app_info.tag = g_tag_cnt++;

	if (g_tag_cnt < 0)
		g_tag_cnt = 100;
	
	list_add(&(new_node->list), &(g_rqst_list[t_rqst_mode].list));

	judge_and_change();	
	
	//spin_unlock(&freq_policy_lock);
	up(&g_sem);
	return new_node->app_info.tag;
}


/* if tag == -1, then delete requests by pid
  * else delete the requests by pid and tag
  */
static void ak98_freq_policy_rls_mode(int tag)
{
	pid_t pid = task_tgid_vnr(current);
	int i;
	struct list_head *pos, *n;
	struct rqst_node *cur;
	int del_flg = 0;
			
	//spin_lock(&freq_policy_lock);
	
	down(&g_sem);

	/* delete mode requests of application whose pid is pid */
	for (i=LOW_IX; i<= HIGH_IX; i++)
	{
		list_for_each_safe(pos, n, &(g_rqst_list[i].list))
		{
			cur = list_entry(pos, struct rqst_node, list);
			if (cur->app_info.pid == pid && (tag == -1 || tag == cur->app_info.tag))
			{
				g_rqst_list[i].app_info.appName[0]--;
				list_del(pos);
				kfree(cur);
				cur = NULL;
				del_flg = 1;
			}			
		}
	}

	if (del_flg == 0)
	{
		//spin_unlock(&freq_policy_lock);	
		up(&g_sem);
		return;
	}

	PDEBUG("Release pid: %d  tag: %d\n", pid, tag);
	g_target_mode = get_target_mode();

	/* if not equal, g_target_mode must be lower than 
	 * g_current_mode (thats to say: g_target_mode <= g_current_mode)
	*/
	PDEBUG("target mode: %s\n", get_mode_name(g_target_mode));
	if (g_target_mode != g_current_mode)
	{
		cancel_delayed_work(&g_work);
		queue_delayed_work(g_freq_policy_wq, &g_work,
				      msecs_to_jiffies(RELEASE_DELAY));
		//del_timer(&change_timer);
		//mod_timer(&change_timer, jiffies+ msecs_to_jiffies(RELEASE_DELAY));
	}
	
	//spin_unlock(&freq_policy_lock);	
	up(&g_sem);
	
}

static T_MODE_TYPE get_mode_by_name(char *name)
{
	int i, n = ARRAY_SIZE(app_to_mode);
	for (i=0; i<n; i++)
	{
		if (strcasecmp(name, app_to_mode[i].appName)==0)
		{
			return app_to_mode[i].mode;
		}
	}
	return DEFAULT_MODE;
}

static int ak98_freq_policy_ioctl(struct inode *node, struct file *file, 
	                          unsigned int cmd, unsigned long data)
{
	int ret = 0;
	char app_name[MAX_NAME_LEN];

	switch (cmd)
	{
		case REQUEST_MODE:
			if (copy_from_user(app_name, (void __user *)data, sizeof(app_name)))
				return -EFAULT;
			
			ret = ak98_freq_policy_rqst_mode(app_name);
			break;
		case RELEASE_MODE:
			PDEBUG("release mode request whit tag %d\n", (int)data);
			ak98_freq_policy_rls_mode(data);
			break;
		default:
			return -EINVAL;
			break;
	}

	return ret;
}

static ssize_t ak98_freq_policy_write(struct file *file, const char *buf,
			      size_t count, loff_t * ppos)
{
	printk("not support write!\n");
	return -ENOTTY;	
}

static int ak98_freq_policy_open(struct inode *inode, struct file *file)
{	
	return 0;
}

static int ak98_freq_policy_release(struct inode *inode, struct file *file)
{
	
	ak98_freq_policy_rls_mode(-1);
	return 0;
}


/*
 *	The various file operations we support.
 */
static const struct file_operations freq_policy_fops = {
      .owner	= THIS_MODULE,
      .open		= ak98_freq_policy_open,
      .release	= ak98_freq_policy_release,
      .read		= ak98_freq_policy_read,
      .write	= ak98_freq_policy_write,
      .ioctl	= ak98_freq_policy_ioctl,
};

static struct miscdevice freq_policy_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "rqstMode",
	.fops	= &freq_policy_fops,
	.mode	= S_IRWXO,
};

static int ak98_freq_policy_probe(struct platform_device *pdev)
{	
	int ret, i;

	ret = misc_register(&freq_policy_dev);
	if (ret) {
		PDEBUG( "freq_policy: "
		       "Unable to register misc device.\n");		
		return ret;
	}

	for (i=LOW_IX; i<=HIGH_IX; i++)
		INIT_LIST_HEAD(&(g_rqst_list[i].list));

	g_current_mode = get_current_mode();
	g_target_mode = LOWEST_MODE;
	g_current_pll = get_pll_sel(g_current_mode);

	INIT_DELAYED_WORK(&g_work, change_work);	
	INIT_DELAYED_WORK(&g_hold_mode_work, hold_mode_work);	
	
	sema_init(&g_sem, 1);
	PDEBUG("AK98 frequency conversion policy driver\n");
	return 0;
}


static int __devexit ak98_freq_policy_remove(struct platform_device *pdev)
{
	int i;
	struct rqst_node *cur;
	struct list_head *pos, *n;
	misc_deregister(&freq_policy_dev);
	/* delete mode requests of application whose pid is pid */
	for (i=LOW_IX; i<= HIGH_IX; i++)
	{
		list_for_each_safe(pos, n, &(g_rqst_list[i].list))
		{
			cur = list_entry(pos, struct rqst_node, list);			
			list_del(pos);
			kfree(cur);
		}
	}
	return 0;
}

static struct platform_driver ak98_freq_policy_driver = {
	.remove		= __devexit_p(ak98_freq_policy_remove),
	.driver		= {
	.name		= "freq_policy",
	.owner		= THIS_MODULE,
	},
};

static int __init ak98_freq_policy_init(void)
{

	g_freq_policy_wq = create_singlethread_workqueue("freq_policy_wq");
	if (!g_freq_policy_wq)
		return -ENOMEM;
	return platform_driver_probe(&ak98_freq_policy_driver, ak98_freq_policy_probe);
}
subsys_initcall(ak98_freq_policy_init);

static void __exit ak98_freq_policy_exit(void)
{
	if (g_freq_policy_wq)
			destroy_workqueue(g_freq_policy_wq);
	platform_driver_unregister(&ak98_freq_policy_driver);
}
module_exit(ak98_freq_policy_exit);

MODULE_AUTHOR("Wenyong Zhou  ANYKA Inc");
MODULE_DESCRIPTION("Frequency conversion policy driver for AK98");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:freq_policy");
