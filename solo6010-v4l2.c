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

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "solo6010-v4l2.h"
#include "solo6010-p2m.h"

#define H_SIZE_FDMA		2048
#define SOLO_PAGE_SIZE		4
#define SOLO_DISP_BUF_SIZE	(64 << 10)

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect (default)");

static void solo_disp_config(struct solo6010_dev *solo_dev)
{
        int i;

	solo_reg_write(solo_dev, SOLO_VO_BORDER_LINE_COLOR,
		       (0xa0 << 24) | (0x88 << 16) | (0xa0 << 8) | 0x88);
	solo_reg_write(solo_dev, SOLO_VO_BORDER_FILL_COLOR,
		       (0x10 << 24) | (0x8f << 16) | (0x10 << 8) | 0x8f);
	solo_reg_write(solo_dev, SOLO_VO_BKG_COLOR,
		       SOLO_VO_BG_YUV(16, 128, 128));

	solo_reg_write(solo_dev, SOLO_VO_FMT_ENC,
		((solo_dev->video_type == 0) ? 0 : SOLO_VO_FMT_TYPE_PAL) |
		SOLO_VO_USER_COLOR_SET_NAV | SOLO_VO_NA_COLOR_Y(0) |
		SOLO_VO_NA_COLOR_CB(0) | SOLO_VO_NA_COLOR_CR(0));

	solo_reg_write(solo_dev, SOLO_VO_ACT_H,
		SOLO_VO_H_START(solo_dev->vout_hstart) |
		SOLO_VO_H_STOP(solo_dev->vout_hstart + solo_dev->video_hsize));

	solo_reg_write(solo_dev, SOLO_VO_ACT_V,
		SOLO_VO_V_START(solo_dev->vout_vstart) |
		SOLO_VO_V_STOP(solo_dev->vout_vstart + solo_dev->video_vsize));

	solo_reg_write(solo_dev, SOLO_VO_RANGE_HV,
		       SOLO_VO_H_LEN(solo_dev->video_hsize) |
		       SOLO_VO_V_LEN(solo_dev->video_vsize));

	solo_reg_write(solo_dev, SOLO_VI_WIN_SW, 5);

	solo_reg_write(solo_dev, SOLO_VO_DISP_CTRL, SOLO_VO_DISP_ON |
		       SOLO_VO_DISP_ERASE_COUNT(8) |
		       SOLO_VO_DISP_BASE(SOLO_DISP_EXT_ADDR(solo_dev)));

	solo_reg_write(solo_dev, SOLO_VO_DISP_ERASE, SOLO_VO_DISP_ERASE_ON);

	/* Mute channel */
	for (i = solo_dev->nr_chans; i < 16; i++) {
		int val = ((~(1 << i) & 0xffff) &
			solo_reg_read(solo_dev, SOLO_VI_CH_ENA));
		solo_reg_write(solo_dev, SOLO_VI_CH_ENA, i);
		solo_reg_write(solo_dev, SOLO_VI_CH_ENA, val);
	}

	/* Disable the watchdog */
	solo_reg_write(solo_dev, SOLO_WATCHDOG, 0);
}

static int solo_vidioc_querycap(struct file *file, void  *priv,
			   struct v4l2_capability *cap)
{
	struct solo6010_dev *solo_dev = priv;

	strcpy(cap->driver, SOLO6010_NAME);
	strcpy(cap->card, SOLO6010_NAME);
	strlcpy(cap->bus_info, solo_dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = SOLO6010_VER_NUM;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_VIDEO_OUTPUT  |
			    V4L2_CAP_TUNER         |
			    V4L2_CAP_AUDIO         |
			    V4L2_CAP_STREAMING     |
			    V4L2_CAP_READWRITE;
        return 0;
}

static int solo_disp_open(struct file *file)
{
	struct solo6010_dev *solo_dev = video_drvdata(file);
	struct solo_filehandle *fh;

	if ((fh = kzalloc(sizeof(*fh), GFP_KERNEL)))
		return -ENOMEM;

	fh->solo_dev = solo_dev;
	file->private_data = fh;

	return 0;
}

/* Try to obtain and/or verify that a fh can read the display device. Only
 * one file descriptor can do this at a time and it retains exclusivity
 * until the file descriptor is closed. */
static int solo_disp_can_read(struct solo_filehandle *fh)
{
	struct solo6010_dev *solo_dev = fh->solo_dev;

	mutex_lock(&solo_dev->v4l2_mutex);
	if (solo_dev->v4l2_reader == NULL)
		solo_dev->v4l2_reader = fh;
	mutex_unlock(&solo_dev->v4l2_mutex);

	if (solo_dev->v4l2_reader != fh)
		return 0;

	return 1;
}

/* If this file handle has exclusivity on read rights, release them */
static void solo_disp_free_read(struct solo_filehandle *fh)
{
	struct solo6010_dev *solo_dev = fh->solo_dev;

	mutex_lock(&solo_dev->v4l2_mutex);
	if (solo_dev->v4l2_reader == fh)
		solo_dev->v4l2_reader = NULL;
	mutex_unlock(&solo_dev->v4l2_mutex);
}

static ssize_t solo_disp_read(struct file *file, char __user *data,
			      size_t count, loff_t *ppos)
{
	struct solo_filehandle *fh = file->private_data;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	unsigned int fdma_addr;
	int cur_page;
	int cur_write;
	int v_size;
	int fdma_addr_offset;
	int frame_size_fdma;
	int i, j, k;

	if (!solo_disp_can_read(fh))
		return -EBUSY;

	v_size = solo_dev->video_vsize * 2;
	fdma_addr_offset = H_SIZE_FDMA * v_size;
	frame_size_fdma = H_SIZE_FDMA * v_size;

	/* XXX: Is this really a good idea? */
	do {
		unsigned int status = solo_reg_read(solo_dev, SOLO_VI_STATUS0);
		cur_write = SOLO_VI_STATUS0_PAGE(status);
		if (cur_write != solo_dev->old_write)
			break;
		msleep_interruptible(1);
	} while(1);

	solo_dev->old_write = cur_write;
	cur_page = cur_write % SOLO_PAGE_SIZE;

	fdma_addr = SOLO_DISP_EXT_ADDR(solo6010) +
			(cur_page * fdma_addr_offset);

	for (i = 0; i < frame_size_fdma / SOLO_DISP_BUF_SIZE; i++) {
		if (solo_p2m_dma(solo_dev, SOLO_P2M_DMA_ID_DISP, 0,
				     solo_dev->dma_buf + (i * SOLO_DISP_BUF_SIZE),
				     fdma_addr + (i * SOLO_DISP_BUF_SIZE),
				     SOLO_DISP_BUF_SIZE) < 0) {
			return -EFAULT;
		}

		for (j = 0; j < SOLO_DISP_BUF_SIZE / H_SIZE_FDMA; j++) {
			k = 2 * solo_dev->video_hsize *
				(i * SOLO_DISP_BUF_SIZE / H_SIZE_FDMA + j);
			if (copy_to_user(data + k, solo_dev->dma_buf
					 + (i * SOLO_DISP_BUF_SIZE) +
					 (j * H_SIZE_FDMA),
					 2 * solo_dev->video_hsize))
				return -EFAULT;
		}
        }

        return frame_size_fdma;
}

static int solo_disp_release(struct file *file)
{
	struct solo_filehandle *fh = file->private_data;

	solo_disp_free_read(fh);
	kfree(fh);

	return 0;
}

static const struct v4l2_file_operations solo_disp_fops = {
	.owner		= THIS_MODULE,
	.open		= solo_disp_open,
	.release	= solo_disp_release,
	.read		= solo_disp_read,
	.ioctl		= video_ioctl2,
};

static const struct v4l2_ioctl_ops solo_disp_ioctl_ops = {
	.vidioc_querycap	= solo_vidioc_querycap,
};

static struct video_device solo_disp_template = {
	.name		= SOLO6010_NAME,
	.fops		= &solo_disp_fops,
	.ioctl_ops	= &solo_disp_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,

	.tvnorms	= V4L2_STD_NTSC | V4L2_STD_PAL,
	.current_norm	= V4L2_STD_NTSC_M,
};

static void solo_v4l2_remove(struct solo6010_dev *solo_dev)
{
	if (solo_dev->vfd) {
		video_unregister_device(solo_dev->vfd);
		solo_dev->vfd = NULL;
	}
	v4l2_device_unregister(&solo_dev->v4l2_dev);
}

int solo_v4l2_init(struct solo6010_dev *solo_dev)
{
	struct v4l2_device *v4l2_dev = &solo_dev->v4l2_dev;
	int ret;

	mutex_init(&solo_dev->v4l2_mutex);

	snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
		 "%s %s", SOLO6010_NAME, pci_name(solo_dev->pdev));
	ret = v4l2_device_register(NULL, v4l2_dev);
	if (ret)
		return ret;

	solo_dev->vfd = video_device_alloc();
	if (!solo_dev->vfd) {
		solo_v4l2_remove(solo_dev);
		return -ENOMEM;
	}

        *solo_dev->vfd = solo_disp_template;

	ret = video_register_device(solo_dev->vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		solo_v4l2_remove(solo_dev);
		return ret;
	}

	video_set_drvdata(solo_dev->vfd, solo_dev);

	snprintf(solo_dev->vfd->name, sizeof(solo_dev->vfd->name), "%s (%i)",
		 solo_disp_template.name, solo_dev->vfd->num);

	if (video_nr >= 0)
		video_nr++;

	v4l2_info(&solo_dev->v4l2_dev, "Registered as /dev/video%d\n",
		  solo_dev->vfd->num);

	solo_disp_config(solo_dev);

	return 0;
}

void solo_v4l2_exit(struct solo6010_dev *solo_dev)
{
	solo_v4l2_remove(solo_dev);
}
