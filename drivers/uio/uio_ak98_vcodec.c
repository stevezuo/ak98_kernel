/*
 * drivers/uio/uio_ak98_vcodec.c
 *
 * Userspace I/O driver for anyka ak98 soc video hardware codec.
 * Based on uio_pdrv.c by Uwe Kleine-Koenig,
 *
 * Copyright (C) 2011 by Anyka Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>

#include <linux/akuio_driver.h>
#include <mach/reg.h>

#define DRIVER_NAME "uio_ak98_vcodec"

const unsigned VIDEO_IRQ_MASK = (1 << IRQ_H264_DECODER) | (1 << IRQ_RMVB) \
  | (1 << IRQ_MPEG2) | (1 << IRQ_JPEG) | (1 << IRQ_MOTIONESTIMATION) | (1 << IRQ_HUFFMAN);
const int MASK_BITS_NUM = sizeof(VIDEO_IRQ_MASK) * 8;

struct uio_platdata {
	struct uio_info *uioinfo;
	struct semaphore vcodec_sem;
};

static irqreturn_t ak98_vcodec_irq_handler(int irq, void *dev_id)
{
	struct uio_platdata *pdata = dev_id;
	int i;

	for (i = 0; i < MASK_BITS_NUM; i++)
	{
		if ((1 << i) & VIDEO_IRQ_MASK)
			disable_irq_nosync(i);
	}

	up (&(pdata->vcodec_sem));

	return IRQ_HANDLED;
}

static int uio_ak98_vcodec_ioctl(struct uio_info *uioinfo, unsigned int cmd, unsigned long arg)
{
	struct uio_platdata *pdata = uioinfo->priv;
	int err;

	switch (cmd) {
	case AKUIO_SYSREG_WRITE:
	{
		struct akuio_sysreg_write_t reg_write;

		if (copy_from_user(&reg_write, (void __user *)arg, sizeof(struct akuio_sysreg_write_t)))
			return -EFAULT;

		ak98_sys_ctrl_reg_set(reg_write.paddr, reg_write.mask, reg_write.val);

		err = 0;
	}
	break;

	case AKUIO_WAIT_IRQ:
	{
		int i;

		for (i = 0; i < MASK_BITS_NUM; i++)
		{
			if ((1 << i) & VIDEO_IRQ_MASK)
				enable_irq(i);
		}

		down (&pdata->vcodec_sem);
		err = 0;
	}
	break;

	default:
		err = -EINVAL;
		break;
	};

	return err;
}

static int uio_ak98_vcodec_open(struct uio_info *uioinfo, struct inode *inode)
{
	struct uio_platdata *pdata = uioinfo->priv;
	int i;

	init_MUTEX_LOCKED(&(pdata->vcodec_sem));

	for (i = 0; i < MASK_BITS_NUM; i++)
	{
		if ((1 << i) & VIDEO_IRQ_MASK) {
			request_irq(i, ak98_vcodec_irq_handler, IRQF_DISABLED, "AK98 VIDEO HW CODEC", pdata);
			disable_irq_nosync(i);
		}
	}

	return 0;
}

static int uio_ak98_vcodec_release(struct uio_info *uioinfo, struct inode *inode)
{
	struct uio_platdata *pdata = uioinfo->priv;
	int i;

	for (i = 0; i < MASK_BITS_NUM; i++)
	{
		if ((1 << i) & VIDEO_IRQ_MASK)
			free_irq(i, pdata);
	}

	return 0;
}

static int uio_ak98_vcodec_probe(struct platform_device *pdev)
{
	struct uio_info *uioinfo = pdev->dev.platform_data;
	struct uio_platdata *pdata;
	struct uio_mem *uiomem;
	int ret = -ENODEV;
	int i;

	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_dbg(&pdev->dev, "%s: err_uioinfo\n", __func__);
		goto err_uioinfo;
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "%s: err_alloc_pdata\n", __func__);
		goto err_alloc_pdata;
	}

	pdata->uioinfo = uioinfo;

	uiomem = &uioinfo->mem[0];

	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = r->end - r->start + 1;
		++uiomem;
	}

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	/* irq */
	pdata->uioinfo->irq = UIO_IRQ_CUSTOM;

	/* file handle */
	pdata->uioinfo->open = uio_ak98_vcodec_open;
	pdata->uioinfo->release = uio_ak98_vcodec_release;
	pdata->uioinfo->ioctl = uio_ak98_vcodec_ioctl;

	pdata->uioinfo->priv = pdata;

	ret = uio_register_device(&pdev->dev, pdata->uioinfo);

	if (ret) {
		kfree(pdata);
err_alloc_pdata:
err_uioinfo:
		return ret;
	}

	platform_set_drvdata(pdev, pdata);

	return 0;
}

static int uio_ak98_vcodec_remove(struct platform_device *pdev)
{
	struct uio_platdata *pdata = platform_get_drvdata(pdev);

	uio_unregister_device(pdata->uioinfo);

	kfree(pdata);

	return 0;
}

static struct platform_driver uio_ak98_vcodec = {
	.probe = uio_ak98_vcodec_probe,
	.remove = uio_ak98_vcodec_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init uio_ak98_vcodec_init(void)
{
	return platform_driver_register(&uio_ak98_vcodec);
}

static void __exit uio_ak98_vcodec_exit(void)
{
	platform_driver_unregister(&uio_ak98_vcodec);
}
module_init(uio_ak98_vcodec_init);
module_exit(uio_ak98_vcodec_exit);

MODULE_AUTHOR("Jacky Lau");
MODULE_DESCRIPTION("Userspace driver for anyka ak98 video hw codec");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
