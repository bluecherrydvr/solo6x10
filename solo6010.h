/*
 * Copyright (C) 2010 Ben Collins <bcollins@bluecherry.net>
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
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __SOLO6010_H
#define __SOLO6010_H

#include <linux/pci.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <media/v4l2-device.h>
#include <asm/io.h>

#include "solo6010-registers.h"

#define PCI_VENDOR_ID_SOFTLOGIC		0x9413
#define PCI_DEVICE_ID_SOLO6010		0x6010

#define SOLO6010_NAME			"solo6010"

/* Make sure these two match */
#define SOLO6010_VERSION		"0.1.0"
#define SOLO6010_VER_MAJOR		0
#define SOLO6010_VER_MINOR		1
#define SOLO6010_VER_SUB		0
#define SOLO6010_VER_NUM \
    KERNEL_VERSION(SOLO6010_VER_MAJOR, SOLO6010_VER_MINOR, SOLO6010_VER_SUB)

/* Stock runtime parameters */
#define SOLO_CLOCK_MHZ			104
/*
 * The SOLO6010 actually has 8 i2c channels, but we only use 2.
 * 0 - Techwell chip(s)
 * 1 - SAA7128 (only if we have a tw2865)
 */
#define SOLO_I2C_ADAPTERS		2

/* DMA Engine setup */
#define SOLO_NR_P2M			4
#define SOLO_NR_P2M_DESC		256
#define SOLO_P2M_DESC_SIZE		(SOLO_NR_P2M_DESC * 16)
/* MPEG and JPEG share the same interrupt and locks so they must be together
 * in the same dma channel. */
#define SOLO_P2M_DMA_ID_MP4E		0
#define SOLO_P2M_DMA_ID_JPEG		0
#define SOLO_P2M_DMA_ID_MP4D		1
#define SOLO_P2M_DMA_ID_G723D		1
#define SOLO_P2M_DMA_ID_DISP		2
#define SOLO_P2M_DMA_ID_OSG		2
#define SOLO_P2M_DMA_ID_G723E		3
#define SOLO_P2M_DMA_ID_VIN		3

enum SOLO_I2C_STATE {
	IIC_STATE_IDLE,
	IIC_STATE_START,
	IIC_STATE_READ,
	IIC_STATE_WRITE,
	IIC_STATE_STOP
};

struct solo_p2m_dev {
	struct semaphore	sem;
	struct completion	completion;
	int			error;
	u32			desc[SOLO_P2M_DESC_SIZE];
};

struct solo_filehandle {
	struct solo6010_dev	*solo_dev;
	enum v4l2_buf_type	type;
};

/* The SOLO6010 PCI Device */
struct solo6010_dev {
	/* General stuff */
	struct pci_dev		*pdev;
	u8 __iomem		*reg_base;
	u8			chip_id;
	int			nr_chans;
	u32			irq_mask;

	/* i2c related items */
	struct i2c_adapter	i2c_adap[SOLO_I2C_ADAPTERS];
	enum SOLO_I2C_STATE	i2c_state;
	struct semaphore	i2c_sem;
	int			i2c_id;
	wait_queue_head_t	i2c_wait;
	struct i2c_msg		*i2c_msg;
	unsigned int		i2c_msg_num;
	unsigned int		i2c_msg_ptr;

	/* P2M DMA Engine */
	struct solo_p2m_dev	p2m_dev[SOLO_NR_P2M];

	/* V4L2 items */
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;
	struct mutex		v4l2_mutex;
	struct solo_filehandle	*v4l2_reader;

	/* Current video settings */
	unsigned int video_type;
	unsigned int video_hsize;
	unsigned int video_vsize;
	unsigned int vout_hstart;
	unsigned int vout_vstart;
	int old_write;
	u8 dma_buf[4096 * 1000];
};


/* Register read and write helper functions. */
#define solo_reg_write(__solo_dev, __off, __data) \
	writel(__data, __solo_dev->reg_base + __off)

#define solo_reg_read(__solo_dev, __off) \
	readl(__solo_dev->reg_base + __off)

void solo6010_irq_on(struct solo6010_dev *solo_dev, u32 mask);
void solo6010_irq_off(struct solo6010_dev *solo_dev, u32 mask);


#endif /* __SOLO6010_H */
