#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>

//#include <asm/system.h>
//#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/ak8801_addr.h>
//#include <asm/io.h>

#define dmawarn(fmt...) DBG(KERN_DEBUG fmt)

#define BUF_MAGIC (0xcafebabe)

#define AK7801_DMAF_AUTOSTART    (1<<1)

int ak7801_dma_enqueue(unsigned int dma_id, void *runtime, dma_addr_t data, int size);

static int irq_count = 0;
static unsigned long tmp_pos = 0;

static void __iomem *dma_base;
static struct kmem_cache *dma_kmem;

enum ak7801_dma_op {
	AK7801_DMAOP_START,
	AK7801_DMAOP_STOP,
	AK7801_DMAOP_PAUSE,
	AK7801_DMAOP_RESUME,
	AK7801_DMAOP_FLUSH,
	AK7801_DMAOP_TIMEOUT,
	AK7801_DMAOP_STARTED,
};

enum ak7801_dma_dataresult { AK7801_RES_OK, AK7801_RES_ERR,
	AK7801_RES_ABORT
};


struct ak7801_dma_client {
	char	*name;
};


struct ak7801_dma_db {
	struct ak7801_dma_db	*next;
	int			magic;
	int			size;
	dma_addr_t		data;
	dma_addr_t		start;
	dma_addr_t		end;
	dma_addr_t		ptr;
	unsigned int		area;
	int			run_state;
	int			full_state;
	void			*runtime;
};

static struct ak7801_dma_db db0;
static struct ak7801_dma_db db1;
static struct ak7801_dma_db * l2_db[] = {
&db0, &db1
};

typedef void (*ak7801_dma_cbfn_t)(void *buf, int size, enum ak7801_dma_dataresult result);

typedef int  (*ak7801_dma_opfn_t)(enum ak7801_dma_op );


struct ak7801_dma_stats {
	unsigned long		loads;
	unsigned long		timeout_longest;
	unsigned long		timeout_shortest;
	unsigned long		timeout_avg;
	unsigned long		timeout_failed;
};

enum ak7801_dma_state {
	AK7801_DMA_IDLE, 
	AK7801_DMA_RUNNING,
	AK7801_DMA_PAUSED
};


enum ak7801_dma_loadst {
	AK7801_DMALOAD_NONE,
	AK7801_DMALOAD_1LOADED,
	AK7801_DMALOAD_1RUNNING,
	AK7801_DMALOAD_1LOADED_1RUNNING,
};


struct ak7801_dma_chan {
	unsigned char		 dma_id; 
	unsigned char		 in_use;
	unsigned char		 irq_claimed;
	unsigned char		 irq_enabled;
	unsigned char		 xfer_unit;
	enum ak7801_dma_state	 state;
	enum ak7801_dma_loadst	 load_state;
	struct ak7801_dma_client *client;
	unsigned long		 dev_addr;
	unsigned long		 load_timeout;
	unsigned int		 flags;	
	unsigned int		 hw_cfg;
	void __iomem		*regs;	
	void __iomem		*addr_reg;
	unsigned int		 irq;
	ak7801_dma_cbfn_t	 callback_fn;
	ak7801_dma_opfn_t	 op_fn;	
	struct ak7801_dma_stats *stats;
	struct ak7801_dma_stats  stats_store;
	struct ak7801_dma_db	*curr;
	struct ak7801_dma_db	*next;
	struct ak7801_dma_db	*end;
	struct sys_device	dev;
};



static struct ak7801_dma_chan l2_dma_dac;
static struct ak7801_dma_chan l2_dma_adc;
static struct ak7801_dma_chan * l2_dma_chan[] = {
	&l2_dma_dac, &l2_dma_adc,
};


int ak7801_dma_ctrl(struct ak7801_snd_runtime *or, enum ak7801_dma_op op);

static void ak7801_dma_stats_timeout(struct ak7801_dma_stats *stats, int val)
{
	if (stats == NULL) return;
	if (val > stats->timeout_longest)
		stats->timeout_longest = val;
	if (val < stats->timeout_shortest)
		stats->timeout_shortest = val;
	stats->timeout_avg += val;
}


static inline int ak7801_dma_loadbuffer(struct ak7801_dma_chan *dma_chan, struct ak7801_dma_db *db)
{
	if (db == NULL) {
		DBG("buffer is NULL\n");
		return -EINVAL;
	}

	if ((db->data & 0xf0000000) != 0x30000000)
	{
		DBG("dma load buf error\n");
	}

	l2_dma_config(db->data, db->size, dma_chan->dma_id);

	return 0;
}


static inline void ak7801_dma_freebuf(struct ak7801_dma_db *db)
{
	int magicok = (db->magic == BUF_MAGIC);

	db->magic = -1;

	if (magicok) 
	{
		kmem_cache_free(dma_kmem, db);
	}
	else
	{
		DBG("ak7801_dma_freebuf: buff %p with bad magic\n", db);
	}
}


static inline void ak7801_dma_bufdone(struct ak7801_dma_chan *dma_chan, struct ak7801_dma_db *db)
{
	if (dma_chan->callback_fn != NULL) 
	{
		(dma_chan->callback_fn)(dma_chan, db->runtime, db->size);
	}
	else
	{
		return IRQ_HANDLED;
	}
}



static irq_handler_t  l2_dac_irq(void)
{
	is_mask_l2_int();
	ak7801_dma_bufdone(&l2_dma_dac, &db0);
	set_l2buf_ram_addr(l2_buf0, db0.data);
	set_l2buf_dma_times(l2_buf0, db0.size >> 6);

	if(stop_state == 1)
	{
		return IRQ_HANDLED;
	} 
	else
	{
		l2_dma_req(0);
		is_unmask_l2_int();
		return IRQ_HANDLED;
	}
}


static irq_handler_t  l2_adc_irq(void)
{
	is_mask_l2_int();
	ak7801_dma_bufdone(&l2_dma_adc, &db1);
	set_l2buf_ram_addr(l2_buf1, db1.data);
	set_l2buf_dma_times(l2_buf1, db1.size >> 6);

	if(stop_state == 1)
	{
		return IRQ_HANDLED;
	}
	else
	{
		l2_dma_req(1);
		is_unmask_l2_int();
		return IRQ_HANDLED;
	}
}


int ak7801_dma_request(unsigned int dma_id, struct ak7801_dma_client *client, void *dev)
{
	unsigned long flags;
	int err;
	struct ak7801_dma_chan *dma_chan = l2_dma_chan[dma_id];

	dma_chan->client = client;
	dma_chan->in_use = 1;

	if (!dma_chan->irq_claimed) 
	{
		dma_chan->irq_claimed = 1;
		if(dma_id == 0)
		{
			err = request_irq(dma_chan->irq, l2_dac_irq, IRQF_DISABLED, client->name, (void *)l2_dac_irq);
			if (err) 
			{
				dma_chan->in_use = 0;
				dma_chan->irq_claimed = 0;

				DBG( "%s: cannot get IRQ %d for DMA %d\n",
						client->name, dma_chan->irq, dma_chan->dma_id);
				return err;
			}
			dma_chan->irq_enabled = 1;
		}
		else if(dma_id == 1)
		{
			err = request_irq(dma_chan->irq, l2_adc_irq, IRQF_DISABLED, client->name, (void *)l2_adc_irq);
			if (err) 
			{
				dma_chan->in_use = 0;
				dma_chan->irq_claimed = 0;

				DBG( "%s: cannot get IRQ %d for DMA %d\n",
						client->name, dma_chan->irq, dma_chan->dma_id);
				return err;
			}
			dma_chan->irq_enabled = 1;
		}
		else
		{
			printk("l2 buf id is wrong !\n");
		}
	}

	return 0;
}

EXPORT_SYMBOL(ak7801_dma_request);



static void ak7801_dma_call_op(struct ak7801_dma_chan *dma_chan, enum ak7801_dma_op op)
{
	if (dma_chan->op_fn != NULL) 
	{
		(dma_chan->op_fn)(op);
	}
	else
		DBG("%s: dma_chan->op_fn = NULL \n",__FUNCTION__);
}


int ak7801_dma_set_buffdone_fn(unsigned int dma_id, ak7801_dma_cbfn_t rtn)
{
	struct ak7801_dma_chan *dma_chan = l2_dma_chan[dma_id];
	if(rtn == NULL)
		DBG("%s:dma_chan->call_fn == NULL \n");
	dma_chan->callback_fn = rtn;
	return 0;
}
EXPORT_SYMBOL(ak7801_dma_set_buffdone_fn);


static int ak7801_dma_dostop(struct ak7801_snd_runtime *or, struct ak7801_dma_chan *dma_chan)
{
	unsigned long flags;

	is_mask_l2_int();

	if(or->dma_id == 0)
		disable_dac_config();
	else
		disable_adc_config();

	return 0;
}


static int ak7801_dma_flush(struct ak7801_snd_runtime *or,  struct ak7801_dma_chan *dma_dac)
{
	unsigned long flags;

	local_irq_save(flags);
	ak7801_dma_freebuf(&db0);
	ak7801_dma_freebuf(&db1);
	local_irq_restore(flags);

	return 0;
}


static int ak7801_dma_preload (struct ak7801_snd_runtime *or) 
{
	int i;
	int j;
	dma_addr_t pos;

	struct ak7801_dma_db *db = l2_db[or->dma_id];

	db->next  = NULL;
	db->data = or->dma_npos;
	db->start = or->dma_start;
	db->end = or->dma_end;
	db->ptr = or->dma_npos;
	db->area = or->dma_area;
	db->size  = or->dma_period;
	db->runtime  = or;
	db->magic = BUF_MAGIC;
	db->full_state = 1;

	or->dma_cpos = or->dma_npos;
	pos = or->dma_cpos + or->dma_period;
	if (pos >= or->dma_end)
		pos = or->dma_start;
	or->dma_npos = pos;

	return 0;
}


static int ak7801_dma_start(struct ak7801_snd_runtime *or,  struct ak7801_dma_chan *dma_chan)
{
	unsigned long tmp;
	unsigned long flags;
	int id = or->dma_id;
	ak7801_dma_preload(or);
	local_irq_save(flags);
	l2_dma_config(l2_db[id]->data, l2_db[id]->size, id);
	is_enable_dma(1);
	l2_dma_req(id);
	is_unmask_l2_int();
	local_irq_restore(flags);

	return 0;
}


int ak7801_dma_ctrl(struct ak7801_snd_runtime *or,  enum ak7801_dma_op op)
{
	struct ak7801_dma_chan *dma_chan = l2_dma_chan[or->dma_id];

	switch (op)
	{
		case AK7801_DMAOP_START:
			return ak7801_dma_start(or, dma_chan);

		case AK7801_DMAOP_STOP:
			return ak7801_dma_dostop(or, dma_chan);

		case AK7801_DMAOP_FLUSH:
			return ak7801_dma_flush(or, dma_chan);
#if 0
		case AK7801_DMAOP_PAUSE:
		case AK7801_DMAOP_RESUME:
			return -ENOENT;

		case AK7801_DMAOP_FLUSH:
			return ak7801_dma_flush(or, dma_dac);

		case AK7801_DMAOP_STARTED:
			return ak7801_dma_started(or, dma_dac);

		case AK7801_DMAOP_TIMEOUT:
			return 0;
#endif
	}

	return -ENOENT; 
}



static int ak7801_dma_canload(struct ak7801_dma_chan *dma_chan)
{
	return 0;
}



static void ak7801_dma_cache_ctor(struct kmem_cache *c, void *p )
{
	memset(p, 0, sizeof(struct ak7801_dma_db));
}


static int __init ak7801_init_dma(void)
{
	struct ak7801_dma_chan *dma_dac = &l2_dma_dac;
	struct ak7801_dma_chan *dma_adc = &l2_dma_adc;
	int ret;

	dma_kmem = kmem_cache_create("dma_desc", sizeof(struct ak7801_dma_db), 0,
		SLAB_HWCACHE_ALIGN, ak7801_dma_cache_ctor);
	if (dma_kmem == NULL) 
	{
		DBG( "dma failed to make kmem cache\n");
		ret = -ENOMEM;
		goto err;
	}

	memset(dma_dac, 0, sizeof(struct ak7801_dma_chan));
	memset(dma_adc, 0, sizeof(struct ak7801_dma_chan));

	dma_dac->irq = PLAY_IRQ;
	dma_dac->dma_id = l2_buf0;
	dma_dac->stats = &dma_dac->stats_store;
	dma_dac->stats_store.timeout_shortest = LONG_MAX;
	dma_dac->load_timeout = 1<<18;


	dma_adc->irq = CAPTURE_IRQ;
	dma_adc->dma_id = l2_buf1;
	dma_adc->stats = &dma_dac->stats_store;
	dma_adc->stats_store.timeout_shortest = LONG_MAX;
	dma_adc->load_timeout = 1<<18;

	audio_set_bit(rL2_FRACDMAADDR, 1<<29|1<<28, 0);
	audio_set_bit(rL2_CONBUF0_7, 0xffff00ff, 0);

	return 0;

 err:
	kmem_cache_destroy(dma_kmem);
	return ret;
}
__initcall(ak7801_init_dma);
