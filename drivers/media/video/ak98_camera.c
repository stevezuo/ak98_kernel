/*
  * @file ak980x camera.c
  * @camera host driver for ak980x
  * @Copyright (C) 2010 Anyka (Guangzhou) Microelectronics Technology Co
  * @author wu_daochao
  * @date 2011-04
  * @version 
  * @for more information , please refer to AK980x Programmer's Guide Mannul
  */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include <asm/io.h>

#include <media/soc_camera.h>
#include <media/videobuf-core.h>
#include <media/videobuf-dma-contig.h>

#include "ak98_camera.h"

//define the image sensor controller register address
#undef REG32
#define REG32(_reg_)  (*(volatile unsigned long *)(_reg_))

#define CAMIF_DEBUG
#ifdef CAMIF_DEBUG
#define CAMDBG(stuff...) printk(KERN_DEBUG"CAMIF: " stuff)
#else
#define CAMDBG(fmt, args...) do{}while(0)
#endif 

struct ak98_buffer {
	struct videobuf_buffer vb;
	const struct soc_camera_data_format	*fmt;
	int inwork;
};

struct ak98_camera_dev {
	struct soc_camera_host soc_host;
	struct soc_camera_device *icd;

	void __iomem	*base;		// mapped baseaddress for CI register(0x2000c000)
	struct resource *res;
	struct clk	*clk;		// camif clk. it's parent is spll defined in clock.c
	unsigned int	irq;

	/* members to manage the dma and buffer*/
	struct list_head capture;	
	struct ak98_buffer	*active;
	spinlock_t		lock;  /* for videobuf_queue , passed in init_videobuf */
	
	/* personal members for platform relative */
	unsigned long mclk;	//clock for CI, 24MHz perhaps	
	
};


/* for ak98_videobuf_release */
static void free_buffer(struct videobuf_queue *vq, struct ak98_buffer *buf)
{
//	struct soc_camera_device *icd = vq->priv_data;
	struct videobuf_buffer *vb = &buf->vb;
	int i;
	
	BUG_ON(in_interrupt());

	CAMDBG("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE */
	videobuf_waiton(vb, 0, 0);
	videobuf_dma_contig_free(vq, vb);

	/* these code enable changing the fmt without closing the device*/
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (vb == vq->bufs[i]) {
			kfree(vb);
			vq->bufs[i] = NULL;
		}
	}
//	vb->state = VIDEOBUF_NEEDS_INIT;
}

/* Called when application apply buffers */
static int ak98_videobuf_setup(struct videobuf_queue *vq, unsigned int *count, 
								unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;

	*size = icd->user_width * icd->user_height *
		((icd->current_fmt->depth + 7) >> 3);

	CAMDBG("%s: icd->user_widh = %d, icd->user_height = %d",
			__func__, icd->user_width, icd->user_height);
	CAMDBG("%s: icd->current_fmt->depth = %d", __func__, icd->current_fmt->depth);

	      
	if (!*count)
		*count = 32;

	while (*size * *count > MAX_VIDEO_MEM * 1024 * 1024)
		(*count)--;
	
	CAMDBG("%s: count=%d, size=%d\n", __func__, *count, *size);
	
	return 0;
}

/* platform independent */
static int ak98_videobuf_prepare(struct videobuf_queue *vq,
			struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct ak98_buffer *buf = container_of(vb, struct ak98_buffer, vb);
	int ret;

	CAMDBG("%s: baddr = 0x%08lx, bsize = %d\n", __func__, vb->baddr, vb->bsize);
	
	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));
	
	BUG_ON(NULL == icd->current_fmt);
	
	/* I think, in buf_prepare you only have to protect global data,
	 * the actual buffer is yours */
	buf->inwork = 1;
	
	if (buf->fmt	!= icd->current_fmt ||
		vb->width	!= icd->user_width ||
		vb->height	!= icd->user_height ||
		vb->field	!= field) {
		buf->fmt	= icd->current_fmt;
		vb->width	= icd->user_width;
		vb->height	= icd->user_height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}
	
	vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3);
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}
	
	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret) 
			goto fail;
		vb->state = VIDEOBUF_PREPARED;
	}
	
	buf->inwork = 0;
	return 0;
	
fail:
	free_buffer(vq, buf);
out:
	buf->inwork = 0;
	return ret;

}

static void ak98_videobuf_queue(struct videobuf_queue *vq, 
								struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct ak98_camera_dev *pcdev = ici->priv;
	struct ak98_buffer *buf = container_of(vb, struct ak98_buffer, vb);
	u32 dmaaddr;
	u32 size;

	CAMDBG("%s (vb=0x%p) baddr = 0x%08lx, bsize = %d\n", 
		__func__,  vb,  vb->baddr, vb->bsize);

	list_add_tail(&vb->queue, &pcdev->capture);

	vb->state = VIDEOBUF_ACTIVE;

	if (!pcdev->active) {
		CAMDBG("pcdev->active == NULL\n");
		
		pcdev->active = buf;

		/* convert the vadd to paddr */
		dmaaddr = videobuf_to_dma_contig(vb); 
		CAMDBG("%s: dmaaddr = 0x%08x\n", __func__, dmaaddr);	

		size = vb->width * vb->height;
		CAMDBG("size = %d, width = %d, height = %d\n", 
			size, vb->width, vb->height);
		
		/* setup the address of dma */
		REG32(pcdev->base + IMG_YODD) = dmaaddr;
		REG32(pcdev->base + IMG_UODD) = dmaaddr + size;
		REG32(pcdev->base + IMG_VODD) = dmaaddr + size /4 * 5;	
		
		REG32(pcdev->base + IMG_YEVE) = dmaaddr;
		REG32(pcdev->base + IMG_UEVE) = dmaaddr + size;
		REG32(pcdev->base + IMG_VEVE) = dmaaddr + size /4 * 5;	

		/*
		** @bit[5]:range of input yuv data for converting to RGB: 0, 16~235; 1, 0~255
		** @bit[4]:0, capture;					1, preview.
		** @bit[3]:0, disable vertical scaling; 		1, enable
	 	** @bit[2]:0, disable horizontal scaling; 	1, enable. 
		** @bit[1:0]:data format transformed by DMA: 00, rgb888 or jpeg; 01, yuv420	
		*/

		/* write command to capture and start dma transfer */
		REG32(pcdev->base + IMG_CMD) = (0 << 4) | (0 << 3) | (0 << 2) | 0x01 | (0 << 5);
	}
}

static void ak98_videobuf_release(struct videobuf_queue *vq, 
					struct videobuf_buffer *vb)
{
	struct ak98_buffer *buf = container_of(vb, struct ak98_buffer, vb);	
#ifdef DEBUG
	struct soc_camera_device *icd = vq->priv_data;
	struct device *dev = icd->dev.parent;
	
	CAMDBG("%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);


	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_dbg(dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_dbg(dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_dbg(dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s (unknown)\n", __func__);
		break;
	}
#endif	
//	dump_stack();

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops ak98_videobuf_ops = {
	.buf_setup      = ak98_videobuf_setup,
	.buf_prepare    = ak98_videobuf_prepare,
	.buf_queue      = ak98_videobuf_queue,
	.buf_release    = ak98_videobuf_release,
};

/* platform code*/
static int ak98_camera_setup_dma(struct ak98_camera_dev *pcdev)
{
	struct videobuf_buffer *vb = &pcdev->active->vb;
	struct device *dev = pcdev->icd->dev.parent;
	u32 dmaaddr;
	u32 size;

	if (unlikely(!pcdev->active)) {
		dev_err(dev, "DMA End IRQ with no active buffer\n");
		return -EFAULT;
	}
	  
	/* setup the DMA address for transferring */
	dmaaddr = videobuf_to_dma_contig(vb); 
	CAMDBG("%s: dmaaddr = 0x%08x\n", __func__, dmaaddr);	
	
	size = vb->width * vb->height;
	CAMDBG("size = %d, width = %d, height = %d\n", 
		size, vb->width, vb->height);

	/* setup the address of dma */
	REG32(pcdev->base + IMG_YODD) = dmaaddr;
	REG32(pcdev->base + IMG_UODD) = dmaaddr + size;
	REG32(pcdev->base + IMG_VODD) = dmaaddr + size /4 * 5;	
	
	REG32(pcdev->base + IMG_YEVE) = dmaaddr;
	REG32(pcdev->base + IMG_UEVE) = dmaaddr + size;
	REG32(pcdev->base + IMG_VEVE) = dmaaddr + size /4 * 5;	

	/* 
	** @bit[5]:range of input yuv data for converting to RGB: 0, 16~235; 1, 0~255
	** @bit[4]:0, capture;					1, preview.
	** @bit[3]:0, disable vertical scaling; 		1, enable
 	** @bit[2]:0, disable horizontal scaling; 	1, enable. 
	** @bit[1:0]:data format transformed by DMA: 00, rgb888 or jpeg; 01, yuv420	
	*/

	/* write command to continue capturing */
	REG32(pcdev->base + IMG_CMD) = (0 << 4) | (0 << 3) | (0 << 2) | 0x01 | (0 << 5);

	return 0;
}

void *getRecordSyncSamples(void);

//struct captureSync{
//	unsigned long long adcCapture_bytes;
//	struct timeval tv;
//};


/* platform code please fix me */
static void ak98_camera_wakeup(struct ak98_camera_dev *pcdev,
			      struct videobuf_buffer *vb,
			      struct ak98_buffer *buf)
{
	struct captureSync * adctime;
	struct timeval		cam_tv;
	unsigned long		adc_stamp;
	unsigned long		useconds;
	/* _init is used to debug races, see comment in mx1_camera_reqbufs() */
	list_del_init(&vb->queue);
	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&cam_tv);
	vb->field_count++;

	adctime = getRecordSyncSamples();
	
	/* figure out the timestamp of frame */
	adc_stamp = adctime->adcCapture_bytes  >> 5;

	if (cam_tv.tv_sec > adctime->tv.tv_sec) {
		useconds = cam_tv.tv_usec + 1000000 - adctime->tv.tv_usec;
	} else {
		useconds = cam_tv.tv_usec -adctime->tv.tv_usec;
	}	

	vb->ts.tv_sec = adc_stamp / 1000;
	vb->ts.tv_usec = (adc_stamp % 1000) * 1000 + useconds;

	wake_up(&vb->done);

	if (list_empty(&pcdev->capture)) {
		CAMDBG("list_empty(&pcdev->capture)\n");
		printk(KERN_DEBUG"list_empty(&pcdev->capture)\n");
		pcdev->active = NULL;
		return;
	}

	pcdev->active = list_entry(pcdev->capture.next,
				   struct ak98_buffer, vb.queue);
	
	ak98_camera_setup_dma(pcdev);

}

/* fix me */
static irqreturn_t ak98_camera_dma_irq(int channel, void *data)
{
	struct ak98_camera_dev *pcdev = data;
	struct device *dev = pcdev->icd->dev.parent;
	struct ak98_buffer *buf;
	struct videobuf_buffer *vb;
	unsigned long flags;
	unsigned long tmpValue;

	spin_lock_irqsave(&pcdev->lock, flags);

	tmpValue = REG32(pcdev->base + IMG_STATUS);
	
	if ((tmpValue & 0x01) == 0x01)			//capture end
	{
		if ((tmpValue & 0x02) == 0x02)		//capture error
		{
			goto out;
		}
	}

	if (unlikely(!pcdev->active)) {
		dev_err(dev, "ak98_camera_dma_irqDMA End IRQ with no active buffer\n");
		goto out;
	}
	
	vb = &pcdev->active->vb;
	buf = container_of(vb, struct ak98_buffer, vb);
	WARN_ON(buf->inwork || list_empty(&vb->queue));

	CAMDBG("%s (vb=0x%p) 0x%08lx %d\n", __func__,
	vb, vb->baddr, vb->bsize);
	
	ak98_camera_wakeup(pcdev, vb, buf);

out:
	spin_unlock_irqrestore(&pcdev->lock, flags);
	return IRQ_HANDLED;
}	

/* for ak98_camera_add_device, the function depends on platform*/
static void ak98_camera_activate(struct ak98_camera_dev *pcdev)
{
	u32 m = 0;
	u32 temp = 0;    
	u32 Plck_Div = 0;
	u32 Pll2_clk = 0;
	u32 mod = 0xff;  
	u32 mclk = pcdev->mclk;

	CAMDBG("entry %s\n", __func__);

	/* set the pin of camera module as camera function */
	ak98_group_config(ePIN_AS_CAMERA);

	/* enable the clock of camera module */
	clk_enable(pcdev->clk);
	mdelay(1);

	/*
	** set camera mclk, SPLL CLK is 120MHz~212MHz,SPLLCLK = 120+4*m
	** mclk = SPLLCLK/2/(PCD+1) = temp/(PCD+1)
	*/
	for (m = 0; m <= 23; m++)
	{
		temp= 60 + 2 * m;
		if (mod > (temp % mclk))
		{
			mod = temp % mclk;     //mod is the minimum value
			Pll2_clk = m & 0xff;
			Plck_Div = (temp / mclk - 1) & 0x07;
		}
	}
	
    	/* configure the spll clk and cisclk(mclk) */
	REG32(pcdev->base + IMG_CONFIG) &= ~((0x1f << 19) | (0x07 << 16));
	REG32(pcdev->base + IMG_CONFIG) |= (Pll2_clk << 19) | (Plck_Div << 16);

	/* setup CCIR601/CCIR656
	* * @bit[12]:0, CCIR 601; 1, CCIR 656. 
	* * @bit[8]:vivref, 0, active low; 1, active hight
	* * @bit[5]:input data format:0, jpeg; 1, yuv422	
	*/
	REG32(pcdev->base + IMG_CONFIG) &= ~(1 << 8);
	REG32(pcdev->base + IMG_CONFIG) &= ~(1 << 12);	
	REG32(pcdev->base + IMG_CONFIG) |= (1 << 5);  //should be stransferred to set_fmt
}

static int ak98_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct ak98_camera_dev *pcdev = ici->priv;

	CAMDBG("entry %s\n", __func__);

	/* The ak98 camera host driver only support one image sensor */
	if (pcdev->icd)
	{
		return -EBUSY;
	}

	/* platform code */
	ak98_camera_activate(pcdev);

	pcdev->icd = icd;
	
	CAMDBG("Leave %s\n", __func__);

	return 0;
}

static void ak98_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct ak98_camera_dev *pcdev = ici->priv;

	CAMDBG("entry %s\n", __func__);

	BUG_ON(icd != pcdev->icd);

	/* disable the clock of camera module */
	clk_disable(pcdev->clk);
	
	pcdev->active = NULL;   
	pcdev->icd = NULL;

	CAMDBG("Leave %s\n", __func__);	
}

/* platform independent finished */
static int ak98_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	CAMDBG("entry %s\n", __func__);

	strlcpy(cap->card, "ak98 soc_camera", sizeof(cap->card));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	
	CAMDBG("Leave %s\n", __func__);

	return 0;	
}

static int ak98_camera_get_crop(struct soc_camera_device *icd,
			       struct v4l2_crop *a)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

//	CAMDBG("entry %s\n", __func__);

//	return v4l2_subdev_call(sd, video, g_crop, a);
	return -1;
}


static int ak98_camera_set_crop(struct soc_camera_device *icd,
			       struct v4l2_crop *a)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct ak98_camera_dev *pcdev = ici->priv;	
	u32 IDimgH, IDimgV;
	u32 tmpValue;

	CAMDBG("entry %s\n", __func__);
	
	/* setup the crop */
	tmpValue = REG32(pcdev->base + IMG_STATUS);    //clear status
		
	IDimgH = 65536 / a->c.width;
	IDimgV = 65536 / a->c.height;
	REG32(pcdev->base + IMG_HINFO1) = a->c.width | (a->c.width << 16);
	REG32(pcdev->base + IMG_HINFO2) = IDimgH;
	REG32(pcdev->base + IMG_VINFO1) = a->c.height | (a->c.height << 16);
	REG32(pcdev->base + IMG_VINFO2) = IDimgV;

	return v4l2_subdev_call(sd, video, s_crop, a);
}


/* platform independent finished */
static int ak98_camera_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	CAMDBG("entry %s\n", __func__);

	return v4l2_subdev_call(sd, video, try_fmt, f);
}

/* platform independent finished */
static int ak98_camera_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
//	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
//	struct ak98_camera_dev *pcdev = ici->priv;		
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	CAMDBG("entry %s\n", __func__);

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->dev.parent, "Format %x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}

	/*  setup the format of pixels
	* * @bit[5]:input data format:0, jpeg; 1, yuv422
	*/	
//	REG32(pcdev->base + IMG_CONFIG) |= (1UL << 5);	

	ret = v4l2_subdev_call(sd, video, s_fmt, f);
	if (!ret) {
		icd->buswidth = xlate->buswidth;
		icd->current_fmt = xlate->host_fmt;
	}
	
	CAMDBG("Leave %s\n", __func__);
	
	return ret;
}

/* Maybe belong platform code fix me */
static int ak98_camera_set_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	unsigned long camera_flags, common_flags;
	int ret;

	CAMDBG("entry %s\n", __func__);

	camera_flags = icd->ops->query_bus_param(icd);

	/* MX1 supports only 8bit buswidth */
	common_flags = soc_camera_bus_param_compatible(camera_flags,
							       CSI_BUS_FLAGS);
	if (!common_flags) {
		return -EINVAL;
	}
	
	icd->buswidth = 8;

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		return ret;
	}
	
	CAMDBG("Leave %s\n", __func__);
	
	return 0;
}

/* platform independent finished*/
static void ak98_camera_init_videobuf(struct videobuf_queue *q,
			struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct ak98_camera_dev *pcdev = ici->priv;

	CAMDBG("entry %s\n", __func__);

	videobuf_queue_dma_contig_init(q, &ak98_videobuf_ops, icd->dev.parent,
					&pcdev->lock,
					V4L2_BUF_TYPE_VIDEO_CAPTURE,
					V4L2_FIELD_NONE,
					sizeof(struct ak98_buffer), icd);
}

/* platform independent finished*/
static int ak98_camera_reqbufs(struct soc_camera_file *icf, 
			struct v4l2_requestbuffers *p)
{
	int i;

	CAMDBG("entry %s\n", __func__);

	/* This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered */
	for (i = 0; i < p->count; i++) {
		struct ak98_buffer *buf = container_of(icf->vb_vidq.bufs[i],
						      struct ak98_buffer, vb);
		buf->inwork = 0;
		INIT_LIST_HEAD(&buf->vb.queue);
	}
	
	CAMDBG("Leave %s\n", __func__);
	
	return 0;
}

/* platform independent */
static unsigned int ak98_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct ak98_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next, struct ak98_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR) {
		return POLLIN | POLLRDNORM;
	}	
	
	return 0;
}


static struct soc_camera_host_ops ak98_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add			= ak98_camera_add_device,
	.remove		= ak98_camera_remove_device,
	.querycap	= ak98_camera_querycap,
	.get_crop		= ak98_camera_get_crop,
	.set_crop		= ak98_camera_set_crop,
	.set_fmt		= ak98_camera_set_fmt,
	.try_fmt		= ak98_camera_try_fmt,
	.set_bus_param	= ak98_camera_set_bus_param,	
	.init_videobuf	= ak98_camera_init_videobuf,
	.reqbufs		= ak98_camera_reqbufs,
	.poll			= ak98_camera_poll,
};

static int __init ak98_camera_probe(struct platform_device *pdev)
{
	struct ak98_camera_dev *pcdev;
	struct resource *res;
	struct clk *clk;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	CAMDBG("entry %s\n", __func__);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0)
	{
		err = -ENODEV;
		goto exit;
	}

	/* get camera interface clock for the whole soc_camera module */
	clk = clk_get(&pdev->dev, "camif_clk");
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		goto exit;
	}

	/* 
	** @allocate memory to struct ak98_camera, including struct soc_camera_host
	** @and struct v4l2_device 
	*/
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (NULL == pcdev)
	{
		err = -ENOMEM;
		goto exit_put_clk;
	}

	/* @initailization for struct pcdev */
	pcdev->res = res;
	pcdev->irq = irq;	
	pcdev->clk = clk;
	pcdev->mclk = 24;
	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, resource_size(res), AK98_CAM_DRV_NAME)) {
		err = -EBUSY;
		goto exit_kfree;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		err = -ENOMEM;
		goto exit_release;
	}

	pcdev->base = base;

	/* request irq */	
	err = request_irq(irq, ak98_camera_dma_irq, IRQF_DISABLED, "ak98_camera", pcdev);
	if (err) {
		err = -EBUSY;
		goto exit_iounmap;
	}

	/*
	** @register soc_camera_host
	*/
	pcdev->soc_host.drv_name	= AK98_CAM_DRV_NAME;
	pcdev->soc_host.ops		= &ak98_soc_camera_host_ops;
	pcdev->soc_host.priv		= pcdev;
	pcdev->soc_host.v4l2_dev.dev	= &pdev->dev;
	pcdev->soc_host.nr		= pdev->id;
	err = soc_camera_host_register(&pcdev->soc_host);
	if (err) {
		goto exit_freeirq;
	}

	return 0;
	
exit_freeirq:
	free_irq(irq, pcdev);
exit_iounmap:
	iounmap(base);
exit_release:
	release_mem_region(res->start, resource_size(res));
exit_kfree:
	kfree(pcdev);
exit_put_clk:
	clk_put(clk);
exit:
	return err;
}

static int __exit ak98_camera_remove(struct platform_device *pdev)
{

	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct ak98_camera_dev *pcdev = container_of(soc_host,
					struct ak98_camera_dev, soc_host);
	struct resource *res;

	CAMDBG("entry %s\n", __func__);

	/* free or disable irq */
	disable_irq(pcdev->irq);

	clk_put(pcdev->clk);

	soc_camera_host_unregister(soc_host);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	dev_info(&pdev->dev, "AK98 Camera driver unloaded\n");
	
	return 0;
}

static struct platform_driver ak98_camera_driver = {
	.probe		= ak98_camera_probe,
//	.remove		= ak98_camera_remove, 	
	.driver		= {
		.name = AK98_CAM_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ak98_camera_init(void)
{
	CAMDBG("entry %s\n", __func__);

	return platform_driver_register(&ak98_camera_driver);
}

static void __exit ak98_camera_exit(void)
{
	CAMDBG("entry %s\n", __func__);

	platform_driver_unregister(&ak98_camera_driver);
}

module_init(ak98_camera_init);
module_exit(ak98_camera_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wu_daochao <wu_daochao@anyka.oa>");
MODULE_DESCRIPTION("Driver for aks980x Camera Interface");
























