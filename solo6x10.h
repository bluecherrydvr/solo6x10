/*
 * Copyright (C) 2010-2013 Bluecherry, LLC <http://www.bluecherrydvr.com>
 *
 * Original author:
 * Ben Collins <bcollins@ubuntu.com>
 *
 * Additional work by:
 * John Brooks <john.brooks@bluecherry.net>
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

#ifndef __SOLO6X10_H
#define __SOLO6X10_H

#include <linux/version.h>
#include <linux/pci.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/stringify.h>
#include <linux/io.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
#include <asm/atomic.h>
#else
#include <linux/atomic.h>
#endif
#include <linux/slab.h>

#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-core.h>

#include "solo6x10-regs.h"

#ifndef PCI_VENDOR_ID_SOFTLOGIC
#define PCI_VENDOR_ID_SOFTLOGIC		0x9413
#define PCI_DEVICE_ID_SOLO6010		0x6010
#define PCI_DEVICE_ID_SOLO6110		0x6110
#endif

#ifndef PCI_VENDOR_ID_BLUECHERRY
#define PCI_VENDOR_ID_BLUECHERRY	0x1BB3
/* Neugent Softlogic 6010 based cards */
#define PCI_DEVICE_ID_NEUSOLO_4		0x4304
#define PCI_DEVICE_ID_NEUSOLO_9		0x4309
#define PCI_DEVICE_ID_NEUSOLO_16	0x4310
/* Bluecherry Softlogic 6010 based cards */
#define PCI_DEVICE_ID_BC_SOLO_4		0x4E04
#define PCI_DEVICE_ID_BC_SOLO_9		0x4E09
#define PCI_DEVICE_ID_BC_SOLO_16	0x4E10
/* Bluecherry Softlogic 6110 based cards */
#define PCI_DEVICE_ID_BC_6110_4		0x5304
#define PCI_DEVICE_ID_BC_6110_8		0x5308
#define PCI_DEVICE_ID_BC_6110_16	0x5310
#endif /* Bluecherry */

/* Used in pci_device_id, and solo_dev->type */
#define SOLO_DEV_6010			0
#define SOLO_DEV_6110			1

#define SOLO6X10_NAME			"solo6x10-edge"

#define SOLO_MAX_CHANNELS		16

/* Make sure these two match */
#define SOLO6X10_VER_MAJOR		2
#define SOLO6X10_VER_MINOR		4
#define SOLO6X10_VER_SUB		7
#define SOLO6X10_VER_NUM \
	KERNEL_VERSION(SOLO6X10_VER_MAJOR, SOLO6X10_VER_MINOR, SOLO6X10_VER_SUB)
#define SOLO6X10_VERSION \
	__stringify(SOLO6X10_VER_MAJOR) "." \
	__stringify(SOLO6X10_VER_MINOR) "." \
	__stringify(SOLO6X10_VER_SUB)

/*
 * The SOLO6x10 actually has 8 i2c channels, but we only use 2.
 * 0 - Techwell chip(s)
 * 1 - SAA7128
 */
#define SOLO_I2C_ADAPTERS		2
#define SOLO_I2C_TW			0
#define SOLO_I2C_SAA			1

/* DMA Engine setup */
#define SOLO_NR_P2M			4
#define SOLO_NR_P2M_DESC		256
#define SOLO_P2M_DESC_SIZE		(SOLO_NR_P2M_DESC * 16)

/* Encoder standard modes */
#define SOLO_ENC_MODE_CIF		2
#define SOLO_ENC_MODE_HD1		1
#define SOLO_ENC_MODE_D1		9

#define SOLO_DEFAULT_GOP		30
#define SOLO_DEFAULT_QP			3

#ifndef V4L2_BUF_FLAG_MOTION_ON
#define V4L2_BUF_FLAG_MOTION_ON		0x0400
#define V4L2_BUF_FLAG_MOTION_DETECTED	0x0800
#endif
#ifndef V4L2_CID_MOTION_ENABLE
#define PRIVATE_CIDS
#define V4L2_CID_MOTION_ENABLE		(V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_MOTION_THRESHOLD	(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_MOTION_TRACE		(V4L2_CID_PRIVATE_BASE+2)
#endif

enum SOLO_I2C_STATE {
	IIC_STATE_IDLE,
	IIC_STATE_START,
	IIC_STATE_READ,
	IIC_STATE_WRITE,
	IIC_STATE_STOP
};

/* Defined in Table 4-16, Page 68-69 of the 6010 Datasheet */
struct solo_p2m_desc {
	u32	ctrl;
	u32	cfg;
	u32	dma_addr;
	u32	ext_addr;
};

struct solo_p2m_dev {
	struct mutex		mutex;
	struct completion	completion;
	int			desc_count;
	int			desc_idx;
	struct solo_p2m_desc	*descs;
	int			error;
};

#define OSD_TEXT_MAX		44

struct solo_enc_dev {
	struct solo_dev	*solo_dev;
	/* V4L2 Items */
	struct video_device	*vfd;
	/* General accounting */
	struct mutex		enable_lock;
	spinlock_t		motion_lock;
	atomic_t		readers;
	atomic_t		mpeg_readers;
	u8			ch;
	u8			mode, gop, qp, interlaced, interval;
	u8			bw_weight;
	u16			motion_thresh;
	u16			width;
	u16			height;

	/* OSD buffers */
	char			osd_text[OSD_TEXT_MAX + 1];
	u8			osd_buf[SOLO_EOSD_EXT_SIZE_MAX]
					__aligned(4);

	/* VOP stuff */
	unsigned char		vop[64];
	int			vop_len;
	unsigned char		jpeg_header[1024];
	int			jpeg_len;

	/* File handles that are listening for buffers */
	struct list_head	listeners;
};

/* The SOLO6x10 PCI Device */
struct solo_dev {
	/* General stuff */
	struct pci_dev		*pdev;
	int			type;
	unsigned int		time_sync;
	unsigned int		usec_lsb;
	unsigned int		clock_mhz;
	u8 __iomem		*reg_base;
	int			nr_chans;
	int			nr_ext;
	u32			irq_mask;
	u32			motion_mask;
	spinlock_t		reg_io_lock;

	/* tw28xx accounting */
	u8			tw2865, tw2864, tw2815;
	u8			tw28_cnt;

	/* i2c related items */
	struct i2c_adapter	i2c_adap[SOLO_I2C_ADAPTERS];
	enum SOLO_I2C_STATE	i2c_state;
	struct mutex		i2c_mutex;
	int			i2c_id;
	wait_queue_head_t	i2c_wait;
	struct i2c_msg		*i2c_msg;
	unsigned int		i2c_msg_num;
	unsigned int		i2c_msg_ptr;

	/* P2M DMA Engine */
	struct solo_p2m_dev	p2m_dev[SOLO_NR_P2M];
	atomic_t		p2m_count;
	int			p2m_jiffies;
	unsigned int		p2m_timeouts;

	/* V4L2 Display items */
	struct video_device	*vfd;
	unsigned int		erasing;
	unsigned int		frame_blank;
	u8			cur_disp_ch;
	wait_queue_head_t	disp_thread_wait;

	/* V4L2 Encoder items */
	struct solo_enc_dev	*v4l2_enc[SOLO_MAX_CHANNELS];
	u16			enc_bw_remain;
	/* IDX into hw mp4 encoder */
	u8			enc_idx;

	/* Current video settings */
	u32			video_type;
	u16			video_hsize, video_vsize;
	u16			vout_hstart, vout_vstart;
	u16			vin_hstart, vin_vstart;
	u8			fps;

	/* JPEG Qp setting */
	spinlock_t      jpeg_qp_lock;
	u32		jpeg_qp[2];

	/* Audio components */
	struct snd_card		*snd_card;
	struct snd_pcm		*snd_pcm;
	atomic_t		snd_users;
	int			g723_hw_idx;

	/* sysfs stuffs */
	struct device		dev;
	int			sdram_size;
	struct bin_attribute	sdram_attr;
	unsigned int		sys_config;

	/* Ring thread */
	struct task_struct	*ring_thread;
	wait_queue_head_t	ring_thread_wait;
	atomic_t		enc_users;
	atomic_t		disp_users;

	/* VOP_HEADER handling */
	void                    *vh_buf;
	dma_addr_t		vh_dma;
	int			vh_size;
};

static inline u32 solo_reg_read(struct solo_dev *solo_dev, int reg)
{
	unsigned long flags;
	u32 ret;
	u16 val;

	spin_lock_irqsave(&solo_dev->reg_io_lock, flags);

	ret = readl(solo_dev->reg_base + reg);
	rmb();
	pci_read_config_word(solo_dev->pdev, PCI_STATUS, &val);
	rmb();

	spin_unlock_irqrestore(&solo_dev->reg_io_lock, flags);

	return ret;
}

static inline void solo_reg_write(struct solo_dev *solo_dev, int reg,
				  u32 data)
{
	unsigned long flags;
	u16 val;

	spin_lock_irqsave(&solo_dev->reg_io_lock, flags);

	writel(data, solo_dev->reg_base + reg);
	wmb();
	pci_read_config_word(solo_dev->pdev, PCI_STATUS, &val);
	rmb();

	spin_unlock_irqrestore(&solo_dev->reg_io_lock, flags);
}

static inline void solo_irq_on(struct solo_dev *dev, u32 mask)
{
	dev->irq_mask |= mask;
	solo_reg_write(dev, SOLO_IRQ_MASK, dev->irq_mask);
}

static inline void solo_irq_off(struct solo_dev *dev, u32 mask)
{
	dev->irq_mask &= ~mask;
	solo_reg_write(dev, SOLO_IRQ_MASK, dev->irq_mask);
}

/* Init/exit routeines for subsystems */
int solo_disp_init(struct solo_dev *solo_dev);
void solo_disp_exit(struct solo_dev *solo_dev);

int solo_gpio_init(struct solo_dev *solo_dev);
void solo_gpio_exit(struct solo_dev *solo_dev);

int solo_i2c_init(struct solo_dev *solo_dev);
void solo_i2c_exit(struct solo_dev *solo_dev);

int solo_p2m_init(struct solo_dev *solo_dev);
void solo_p2m_exit(struct solo_dev *solo_dev);

int solo_v4l2_init(struct solo_dev *solo_dev, unsigned nr);
void solo_v4l2_exit(struct solo_dev *solo_dev);

int solo_enc_init(struct solo_dev *solo_dev);
void solo_enc_exit(struct solo_dev *solo_dev);

int solo_enc_v4l2_init(struct solo_dev *solo_dev, unsigned nr);
void solo_enc_v4l2_exit(struct solo_dev *solo_dev);

int solo_g723_init(struct solo_dev *solo_dev);
void solo_g723_exit(struct solo_dev *solo_dev);

/* ISR's */
int solo_i2c_isr(struct solo_dev *solo_dev);
void solo_p2m_isr(struct solo_dev *solo_dev, int id);
void solo_p2m_error_isr(struct solo_dev *solo_dev);
void solo_enc_v4l2_isr(struct solo_dev *solo_dev);
void solo_g723_isr(struct solo_dev *solo_dev);
void solo_motion_isr(struct solo_dev *solo_dev);
void solo_video_in_isr(struct solo_dev *solo_dev);

/* i2c read/write */
u8 solo_i2c_readbyte(struct solo_dev *solo_dev, int id, u8 addr, u8 off);
void solo_i2c_writebyte(struct solo_dev *solo_dev, int id, u8 addr, u8 off,
			u8 data);

/* P2M DMA */
int solo_p2m_dma_t(struct solo_dev *solo_dev, int wr,
		   dma_addr_t dma_addr, u32 ext_addr, u32 size,
		   int repeat, u32 ext_size);
int solo_p2m_dma(struct solo_dev *solo_dev, int wr,
		 void *sys_addr, u32 ext_addr, u32 size,
		 int repeat, u32 ext_size);
void solo_p2m_fill_desc(struct solo_p2m_desc *desc, int wr,
			dma_addr_t dma_addr, u32 ext_addr, u32 size,
			int repeat, u32 ext_size);
int solo_p2m_dma_desc(struct solo_dev *solo_dev,
		      struct solo_p2m_desc *desc, dma_addr_t desc_dma,
		      int desc_cnt);

/* Set the threshold for motion detection */
int solo_set_motion_threshold(struct solo_dev *solo_dev, u8 ch, u16 val);
int solo_set_motion_block(struct solo_dev *solo_dev, u8 ch, u16 val,
			   u16 block);
#define SOLO_DEF_MOT_THRESH		0x0300

/* Write text on OSD */
int solo_osd_print(struct solo_enc_dev *solo_enc);

/* EEPROM commands */
unsigned int solo_eeprom_ewen(struct solo_dev *solo_dev, int w_en);
unsigned short solo_eeprom_read(struct solo_dev *solo_dev, int loc);
int solo_eeprom_write(struct solo_dev *solo_dev, int loc,
		      unsigned short data);

/* JPEG Qp functions */
void solo_s_jpeg_qp(struct solo_dev *solo_dev, unsigned int ch,
		    unsigned int qp);
int solo_g_jpeg_qp(struct solo_dev *solo_dev, unsigned int ch);

#define CHK_FLAGS(v, flags) (((v) & (flags)) == (flags))

#endif /* __SOLO6X10_H */
