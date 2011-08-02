
/*
 *  linux/drivers/mmc/host/ak98_mci.h - AK98 MMC/SD/SDIO driver
 *
 *  Copyright (C) 2010 Anyka, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __AK98_SDIO_H__
#define __AK98_SDIO_H__

#include "ak98_mci.h"

#define AK98SDIOINTRCTR		0x000
#define SDIO_INTR_CTR_ENABLE	(1 << 8)
#define SDIO_INTR_ENABLE	(1 << 17)

#endif
