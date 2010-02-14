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
#include <media/v4l2-ioctl.h>

#include "solo6010.h"

#define SOLO_H_SIZE_FDMA	2048
#define SOLO_PAGE_SIZE		4
#define SOLO_DISP_PIX_FORMAT	V4L2_PIX_FMT_UYVY
#define SOLO_DISP_PIX_FIELD	V4L2_FIELD_INTERLACED_TB
#define SOLO_DEFAULT_CHAN	0

//#define COPY_WHOLE_LINE

/* Image size is two fields, SOLO_H_SIZE_FDMA is one horizontal line */
#ifdef COPY_WHOLE_LINE
#define solo_image_size(__solo) (SOLO_H_SIZE_FDMA * __solo->video_vsize * 2)
#else
#define solo_image_size(__solo)	(__solo->video_hsize * __solo->video_vsize * 4)
#endif

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect (default)");

static void erase_on(struct solo6010_dev *solo_dev)
{
	solo_reg_write(solo_dev, SOLO_VO_DISP_ERASE, SOLO_VO_DISP_ERASE_ON);
	solo_dev->erasing = 1;
	solo_dev->frame_blank = 0;
}

static int erase_off(struct solo6010_dev *solo_dev)
{
	if (!solo_dev->erasing)
		return 0;

	/* First time around, assert erase off */
	if (!solo_dev->frame_blank)
		solo_reg_write(solo_dev, SOLO_VO_DISP_ERASE, 0);
	/* Keep the erasing flag on for 8 frames minimum */
	if (solo_dev->frame_blank++ >= 8)
		solo_dev->erasing = 0;

	return 1;
}

static int solo_disp_ch(struct solo6010_dev *solo_dev, u8 ch, int on)
{
	if (ch >= solo_dev->nr_chans)
		return -EINVAL;

	/* Here, we just keep window/channel the same */
	solo_reg_write(solo_dev, SOLO_VI_WIN_CTRL0(ch),
		       SOLO_VI_WIN_CHANNEL(ch) |
		       SOLO_VI_WIN_SX(on ? 0: solo_dev->video_hsize) |
		       SOLO_VI_WIN_EX(solo_dev->video_hsize) |
		       SOLO_VI_WIN_SCALE(on ? 1 : 0));

	solo_reg_write(solo_dev, SOLO_VI_WIN_CTRL1(ch),
		       SOLO_VI_WIN_SY(on ? 0 : solo_dev->video_vsize) |
		       SOLO_VI_WIN_EY(solo_dev->video_vsize));

	solo_reg_write(solo_dev, SOLO_VI_WIN_ON(ch), on ? 1 : 0);

	return 0;
}

static int solo_disp_set_ch(struct solo6010_dev *solo_dev, unsigned int ch)
{
	if (ch >= solo_dev->nr_chans)
		return -EINVAL;

	printk("Switching to channel %d\n", ch);

	erase_on(solo_dev);

	solo_disp_ch(solo_dev, solo_dev->cur_ch, 0);
	solo_disp_ch(solo_dev, ch, 1);

	solo_dev->cur_ch = ch;

	return 0;
}

static int solo_disp_open(struct file *file)
{
	struct solo6010_dev *solo_dev = video_drvdata(file);
	struct solo_filehandle *fh;

	if ((fh = kzalloc(sizeof(*fh), GFP_KERNEL)) == NULL)
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
	int cur_write;
	int frame_size;
	int image_size = solo_image_size(solo_dev);
	int i, j;
	static int next_ch = 1;
	static int frames = -30;

	if (!solo_disp_can_read(fh))
		return -EBUSY;
	
	if (count < image_size)
		return -EINVAL;

	/* XXX: Is this really a good idea? */
	do {
		unsigned int status = solo_reg_read(solo_dev, SOLO_VI_STATUS0);
		cur_write = SOLO_VI_STATUS0_PAGE(status);
		if (cur_write != solo_dev->old_write)
			break;
		msleep_interruptible(1);
	} while(1);

	solo_dev->old_write = cur_write;

	if (next_ch >= 0 && frames++ >= 30) {
		frames = 0;
		solo_disp_set_ch(solo_dev, next_ch++);
		if (next_ch >= solo_dev->nr_chans)
			next_ch = 0;
		else if (next_ch == 1)
			next_ch = -1;
	}

	if (erase_off(solo_dev)) {
		for (i = 0; i < image_size; i += 2) {
			u8 buf[2] = { 0x80, 0x00 };
			copy_to_user(data + i, buf, 2);
		}
		return image_size;
	}

	frame_size = SOLO_H_SIZE_FDMA * solo_dev->video_vsize * 2;
	fdma_addr = SOLO_DISP_EXT_ADDR(solo_dev) + (cur_write * frame_size);

	for (i = 0; i < frame_size / SOLO_DISP_BUF_SIZE; i++) {
		if (solo_p2m_dma(solo_dev, SOLO_P2M_DMA_ID_DISP, 0,
				 solo_dev->vout_buf,
				 fdma_addr + (i * SOLO_DISP_BUF_SIZE),
				 SOLO_DISP_BUF_SIZE) < 0)
			return -EFAULT;
#ifdef COPY_WHOLE_LINE
		if (copy_to_user(data + (i * SOLO_DISP_BUF_SIZE),
				 solo_dev->vout_buf, SOLO_DISP_BUF_SIZE))
			return -EFAULT;
#else
		for (j = 0; j < (SOLO_DISP_BUF_SIZE / SOLO_H_SIZE_FDMA); j++) {
			int off = 2 * solo_dev->video_hsize *
			      ((i * SOLO_DISP_BUF_SIZE / SOLO_H_SIZE_FDMA) + j);
			if (copy_to_user(data + off, solo_dev->vout_buf +
					 (j * SOLO_H_SIZE_FDMA),
					 2 * solo_dev->video_hsize))
				return -EFAULT;
		}
#endif
	}

	return image_size;
}

static int solo_disp_release(struct file *file)
{
	struct solo_filehandle *fh = file->private_data;

	solo_disp_free_read(fh);
	kfree(fh);

	return 0;
}

static int solo_querycap(struct file *file, void  *priv,
			 struct v4l2_capability *cap)
{
	struct solo_filehandle  *fh  = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;

	strcpy(cap->driver, SOLO6010_NAME);
	strcpy(cap->card, "Softlogic 6010");
	snprintf(cap->bus_info, sizeof(cap->bus_info), "%s %s",
		 SOLO6010_NAME, pci_name(solo_dev->pdev));
	cap->version = SOLO6010_VER_NUM;
	cap->capabilities =     V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_READWRITE;
	return 0;
}

static int solo_enum_input(struct file *file, void *priv,
			   struct v4l2_input *input)
{
	struct solo_filehandle *fh  = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;

	if (input->index >= solo_dev->nr_chans)
		return -EINVAL;

	snprintf(input->name, sizeof(input->name), "Camera %d",
		 input->index + 1);
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_525_60 | V4L2_STD_625_50;
	/* XXX Should check for signal status on this camera */
	input->status = 0;

	return 0;
}

static int solo_set_input(struct file *file, void *priv, unsigned int index)
{
	struct solo_filehandle *fh = priv;

	return solo_disp_set_ch(fh->solo_dev, index);
}

static int solo_get_input(struct file *file, void *priv, unsigned int *index)
{
	struct solo_filehandle *fh = priv;

	*index = fh->solo_dev->cur_ch;

	return 0;
}

static int solo_enum_fmt_cap(struct file *file, void *priv,
			     struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = SOLO_DISP_PIX_FORMAT;
	snprintf(f->description, sizeof(f->description),
		 "%s", "YUV 4:2:2 Packed");

	return 0;
}

static int solo_try_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct solo_filehandle *fh = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int image_size = solo_image_size(solo_dev);

	/* Check supported sizes */
	if (pix->width > solo_dev->video_hsize)
		pix->width = solo_dev->video_hsize;
	if (pix->height > solo_dev->video_vsize * 2)
		pix->width = solo_dev->video_vsize * 2;
	if (pix->sizeimage > image_size)
		pix->sizeimage = image_size;

	if (pix->width     != solo_dev->video_hsize ||
	    pix->height    != solo_dev->video_vsize * 2 ||
	    pix->sizeimage != image_size) {
		printk("Size is all wrong: %d, %d, %d\n", pix->width,
		       pix->height, pix->sizeimage);
		pix->width = solo_dev->video_hsize;
		pix->height = solo_dev->video_vsize;
		pix->sizeimage = image_size;
		//return -EINVAL;
	}

	/* Check formats */
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = SOLO_DISP_PIX_FIELD;

	if (pix->pixelformat != SOLO_DISP_PIX_FORMAT ||
	    pix->field       != SOLO_DISP_PIX_FIELD ||
	    pix->colorspace  != V4L2_COLORSPACE_SMPTE170M) {
		printk("Pix format is all wrong\n");
		pix->pixelformat = SOLO_DISP_PIX_FORMAT;
		pix->field = SOLO_DISP_PIX_FIELD;
		pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
		//return -EINVAL;
	}

	return 0;
}

static int solo_set_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct solo_filehandle *fh = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;

	/* If there is currently a reader, we do not change the format */
	if (solo_dev->v4l2_reader != NULL)
		return -EBUSY;

	/* For right now, if it doesn't match our running config,
	 * then fail */
	return solo_try_fmt_cap(file, priv, f);
}

static int solo_get_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct solo_filehandle *fh = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = solo_dev->video_hsize;
	pix->height = solo_dev->video_vsize * 2;
	pix->pixelformat = SOLO_DISP_PIX_FORMAT;
	pix->field = SOLO_DISP_PIX_FIELD;
	pix->sizeimage = solo_image_size(solo_dev);
#ifdef COPY_WHOLE_LINE
	pix->bytesperline = SOLO_H_SIZE_FDMA;
#else
	pix->bytesperline = solo_dev->video_hsize * 2;
#endif
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static const struct v4l2_file_operations solo_disp_fops = {
	.owner			= THIS_MODULE,
	.open			= solo_disp_open,
	.release		= solo_disp_release,
	.read			= solo_disp_read,
	.ioctl			= video_ioctl2,
};

static const struct v4l2_ioctl_ops solo_disp_ioctl_ops = {
	.vidioc_querycap		= solo_querycap,
	/* Input callbacks */
	.vidioc_enum_input		= solo_enum_input,
	.vidioc_s_input			= solo_set_input,
	.vidioc_g_input			= solo_get_input,
	/* Video capture format callbacks */
	.vidioc_enum_fmt_vid_cap	= solo_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap		= solo_try_fmt_cap,
	.vidioc_s_fmt_vid_cap		= solo_set_fmt_cap,
	.vidioc_g_fmt_vid_cap		= solo_get_fmt_cap,
};

static struct video_device solo_disp_template = {
	.name			= SOLO6010_NAME,
	.fops			= &solo_disp_fops,
	.ioctl_ops		= &solo_disp_ioctl_ops,
	.minor			= -1,
	.release		= video_device_release,

	.tvnorms		= V4L2_STD_525_60 | V4L2_STD_625_50,
	.current_norm		= V4L2_STD_NTSC_M,
};

int solo_v4l2_init(struct solo6010_dev *solo_dev)
{
	int ret;

	mutex_init(&solo_dev->v4l2_mutex);

	solo_dev->vfd = video_device_alloc();
	if (!solo_dev->vfd)
		return -ENOMEM;

	*solo_dev->vfd = solo_disp_template;
	solo_dev->vfd->parent = &solo_dev->pdev->dev;

	ret = video_register_device(solo_dev->vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		video_device_release(solo_dev->vfd);
		solo_dev->vfd = NULL;
		return ret;
	}

	video_set_drvdata(solo_dev->vfd, solo_dev);

	snprintf(solo_dev->vfd->name, sizeof(solo_dev->vfd->name), "%s (%i)",
		 SOLO6010_NAME, solo_dev->vfd->num);

	if (video_nr >= 0)
		video_nr++;

	dev_info(&solo_dev->pdev->dev, "Registered as /dev/video%d with "
		 "%d inputs\n", solo_dev->vfd->num, solo_dev->nr_chans);

	/* Set the default display channel */
	solo_disp_set_ch(solo_dev, SOLO_DEFAULT_CHAN);

	return 0;
}

void solo_v4l2_exit(struct solo6010_dev *solo_dev)
{
	video_unregister_device(solo_dev->vfd);
	solo_dev->vfd = NULL;
}
