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
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <media/v4l2-device.h>
#include <media/videobuf-vmalloc.h>
#include <media/v4l2-ioctl.h>

#include "solo6010.h"

#define MIN_VID_BUFFERS		8
#define FRAME_BUF_SIZE		(256 * 1024)
#define MP4_QUEUES		16

/* There is 8MB memory available for solo to buffer MPEG4 frames.
 * This gives us 512 * 16kbyte queues. */
#define NR_EXT_QUEUE		512

/* Simple file handle */
struct solo_enc_fh {
	struct solo6010_dev	*solo_dev;
	int			cur_ch;
	struct videobuf_queue	vidq;
	struct task_struct      *kthread;
	spinlock_t		slock;
	u8			vout_buf[FRAME_BUF_SIZE];
	struct list_head	vidq_active;
	wait_queue_head_t	thread_wait;
};

extern unsigned video_nr;

/* You must hold the slock around this yourself, otherwise
 * use the locking version below. */
static void __solo_enc_put_ch(struct solo_enc_fh *fh)
{
	struct solo6010_dev *solo_dev = fh->solo_dev;
	int ch = fh->cur_ch;

	/* This file handle has no encoder associated with it */
	if (ch < 0)
		return;

	fh->cur_ch = -1;

	BUG_ON(ch >= solo_dev->nr_chans);

	if (atomic_dec_return(&solo_dev->enc_used[ch]))
		return;

	/* No references left, disable it */
	solo_dev->enc_mode[ch] = 0;
	solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), 0);
}

static void solo_enc_put_ch(struct solo_enc_fh *fh)
{
	unsigned long flags;

	spin_lock_irqsave(&fh->slock, flags);

	__solo_enc_put_ch(fh);

	spin_unlock_irqrestore(&fh->slock, flags);
}

/* Returns >= 0 if channel was activated (success), 0 if the mode
 * was also set, and negative errno on errors. No error checking is
 * done to verify that the mode was set (it cannot be changed when
 * encoding is a reference is already taken). If return is 0, you
 * can be sure your mode was set, if it is > 0, you should check
 * solo_dev->enc_mode[ch] to see if you match. */
static int solo_enc_get_ch(struct solo_enc_fh *fh, u8 ch, u8 mode)
{
	struct solo6010_dev *solo_dev = fh->solo_dev;
	unsigned long flags;
	int ret = 1;

	if (ch >= solo_dev->nr_chans)
		return -EINVAL;

	spin_lock_irqsave(&fh->slock, flags);

	/* Already have the ref? */
	if (fh->cur_ch == ch) {
		spin_unlock_irqrestore(&fh->slock, flags);
		return 0;
	}

	/* Release the old ref */
	__solo_enc_put_ch(fh);

	fh->cur_ch = ch;

	/* A return of 1 means we are the first ref, and so set the mode */
	if (atomic_inc_return(&solo_dev->enc_used[ch]) == 1) {
		solo_dev->enc_mode[ch] = mode;
		switch (mode) {
		case SOLO_ENC_MODE_CIF:
			solo_dev->enc_width[ch] = 352;
			solo_dev->enc_height[ch] = 240;
			break;
		case SOLO_ENC_MODE_HALFD1H:
			solo_dev->enc_width[ch] = 704;
			solo_dev->enc_height[ch] = 240;
			break;
		case SOLO_ENC_MODE_D1:
			solo_dev->enc_width[ch] = 704;
			solo_dev->enc_height[ch] = 480;
			break;
		}
		solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), 0);
		solo_reg_write(solo_dev, SOLO_VE_CH_INTL(ch), (mode >> 3) & 1);
		solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), mode);
		ret = 0;
	}

	spin_unlock_irqrestore(&fh->slock, flags);

	return ret;
}

static void solo_enc_fillbuf(struct solo_enc_fh *fh,
			     struct videobuf_buffer *vb)
{
	//struct solo6010_dev *solo_dev = fh->solo_dev;
	u8 *vbuf;

	list_del(&vb->queue);

	if (!(vbuf = videobuf_to_vmalloc(vb)))
		return;
/* XXX: Fill it up here, or in ISR? */
//finish_buf:
	vb->field_count++;
	do_gettimeofday(&vb->ts);
	vb->state = VIDEOBUF_DONE;
	wake_up(&vb->done);

	return;
}

static void solo_enc_thread_sleep(struct solo_enc_fh *fh)
{
	struct videobuf_buffer *vb;
	unsigned long flags;

	for (;;) {
		long timeout = msecs_to_jiffies(100);
		spin_lock_irqsave(&fh->slock, flags);
		timeout =
			wait_event_interruptible_timeout(fh->thread_wait,
				   !list_empty(&fh->vidq_active),
				   timeout);
		if (timeout == -ERESTARTSYS || kthread_should_stop()) {
			spin_unlock_irqrestore(&fh->slock, flags);
			return;
		} else if (timeout)
			break;
		spin_unlock_irqrestore(&fh->slock, flags);
	}

	vb = list_first_entry(&fh->vidq_active, struct videobuf_buffer,
			      queue);

	if (waitqueue_active(&vb->done))
		solo_enc_fillbuf(fh, vb);

	spin_unlock_irqrestore(&fh->slock, flags);

	try_to_freeze();
}

static int solo_enc_thread(void *data)
{
	struct solo_enc_fh *fh = data;

	set_freezable();

	for (;;) {
		solo_enc_thread_sleep(fh);

		if (kthread_should_stop())
			break;
	}
        return 0;
}

static int solo_enc_start_thread(struct solo_enc_fh *fh)
{
	fh->kthread = kthread_run(solo_enc_thread, fh, SOLO6010_NAME "_enc");

	if (IS_ERR(fh->kthread))
		return PTR_ERR(fh->kthread);

	return 0;
}

static void solo_enc_stop_thread(struct solo_enc_fh *fh)
{
	if (fh->kthread) {
		kthread_stop(fh->kthread);
		fh->kthread = NULL;
	}
}

static int solo_enc_buf_setup(struct videobuf_queue *vq, unsigned int *count,
			      unsigned int *size)
{
        *size = FRAME_BUF_SIZE;

        if (*count < MIN_VID_BUFFERS)
		*count = MIN_VID_BUFFERS;

        return 0;
}

static int solo_enc_buf_prepare(struct videobuf_queue *vq,
				struct videobuf_buffer *vb,
				enum v4l2_field field)
{
	struct solo_enc_fh *fh  = vq->priv_data;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	int ch = fh->cur_ch;

	vb->size = FRAME_BUF_SIZE;
	if (vb->baddr != 0 && vb->bsize < vb->size)
		return -EINVAL;

	if (fh->cur_ch < 0)
		return -EINVAL;

	/* XXX: These properties only change when queue is idle */
	vb->width = solo_dev->enc_width[ch];
	vb->height = solo_dev->enc_height[ch];
	vb->field  = field;

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		int rc = videobuf_iolock(vq, vb, NULL);
		if (rc < 0) {
			videobuf_vmalloc_free(vb);
			vb->state = VIDEOBUF_NEEDS_INIT;
			return rc;
		}
	}
	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

static void solo_enc_buf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct solo_enc_fh *fh = vq->priv_data;

	vb->state = VIDEOBUF_QUEUED;
	list_add_tail(&vb->queue, &fh->vidq_active);
	wake_up(&fh->thread_wait);
}

static void solo_enc_buf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	videobuf_vmalloc_free(vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops solo_enc_video_qops = {
	.buf_setup	= solo_enc_buf_setup,
	.buf_prepare	= solo_enc_buf_prepare,
	.buf_queue	= solo_enc_buf_queue,
	.buf_release	= solo_enc_buf_release,
};

static unsigned int solo_enc_poll(struct file *file,
				  struct poll_table_struct *wait)
{
	struct solo_enc_fh *fh = file->private_data;

        return videobuf_poll_stream(file, &fh->vidq, wait);
}

static int solo_enc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct solo_enc_fh *fh = file->private_data;

	return videobuf_mmap_mapper(&fh->vidq, vma);
}

static int solo_enc_open(struct file *file)
{
	struct solo6010_dev *solo_dev = video_drvdata(file);
	struct solo_enc_fh *fh;
	int ret;

	if ((fh = kzalloc(sizeof(*fh), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	spin_lock_init(&fh->slock);
	INIT_LIST_HEAD(&fh->vidq_active);
	init_waitqueue_head(&fh->thread_wait);
	fh->solo_dev = solo_dev;
	fh->cur_ch = -1;
	file->private_data = fh;

	if ((ret = solo_enc_start_thread(fh))) {
		kfree(fh);
		return ret;
	}

	videobuf_queue_vmalloc_init(&fh->vidq, &solo_enc_video_qops,
				    NULL, &fh->slock,
				    V4L2_BUF_TYPE_VIDEO_CAPTURE,
				    V4L2_FIELD_INTERLACED,
				    sizeof(struct videobuf_buffer), fh);

	return 0;
}

static ssize_t solo_enc_read(struct file *file, char __user *data,
			     size_t count, loff_t *ppos)
{
	struct solo_enc_fh *fh = file->private_data;

	return videobuf_read_stream(&fh->vidq, data, count, ppos, 0,
				    file->f_flags & O_NONBLOCK);
}

static int solo_enc_release(struct file *file)
{
	struct solo_enc_fh *fh = file->private_data;

	videobuf_stop(&fh->vidq);
	videobuf_mmap_free(&fh->vidq);
	solo_enc_stop_thread(fh);
	solo_enc_put_ch(fh);
	kfree(fh);

	return 0;
}

static int solo_enc_querycap(struct file *file, void  *priv,
			     struct v4l2_capability *cap)
{
	struct solo_enc_fh  *fh  = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;

	strcpy(cap->driver, SOLO6010_NAME);
	strcpy(cap->card, "Softlogic 6010 Enc");
	snprintf(cap->bus_info, sizeof(cap->bus_info), "%s %s",
		 SOLO6010_NAME, pci_name(solo_dev->pdev));
	cap->version = SOLO6010_VER_NUM;
	cap->capabilities =     V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_READWRITE |
				V4L2_CAP_STREAMING;
	return 0;
}

static int solo_enc_enum_input(struct file *file, void *priv,
			       struct v4l2_input *input)
{
	struct solo_enc_fh *fh  = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;

	if (input->index >= solo_dev->nr_chans)
		return -EINVAL;

	snprintf(input->name, sizeof(input->name), "Camera %d",
		 input->index + 1);
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_NTSC_M;
	/* XXX Should check for signal status on this camera */
	input->status = 0;

	return 0;
}

static int solo_enc_set_input(struct file *file, void *priv, unsigned int index)
{
	struct solo_enc_fh *fh = priv;

	return solo_enc_get_ch(fh, index, SOLO_ENC_MODE_CIF);
}

static int solo_enc_get_input(struct file *file, void *priv,
			      unsigned int *index)
{
	struct solo_enc_fh *fh = priv;

	*index = fh->cur_ch;

	return 0;
}

static int solo_enc_enum_fmt_cap(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_MPEG;
	snprintf(f->description, sizeof(f->description),
		 "%s", "MPEG-4 AVC");

	return 0;
}

static int solo_enc_try_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ch = fh->cur_ch;
	u16 width, height;

	if (ch < 0)
		return -EINVAL;

	width = solo_dev->enc_width[ch];
	height = solo_dev->enc_height[ch];

	/* XXX Should use this to detect mode */
	if (pix->width != width)
		pix->width = width;
	if (pix->height != height)
		pix->height = height;
	if (pix->sizeimage != FRAME_BUF_SIZE)
		pix->sizeimage = FRAME_BUF_SIZE;

	/* Check formats */
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_INTERLACED;

	if (pix->pixelformat != V4L2_PIX_FMT_MPEG ||
	    pix->field       != V4L2_FIELD_INTERLACED ||
	    pix->colorspace  != V4L2_COLORSPACE_SMPTE170M)
		return -EINVAL;

	return 0;
}

static int solo_enc_set_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;

	if (videobuf_queue_is_busy(&fh->vidq))
		return -EBUSY;

	/* For right now, if it doesn't match our running config,
	 * then fail */
	return solo_enc_try_fmt_cap(file, priv, f);
}

static int solo_enc_get_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;
	struct solo6010_dev *solo_dev = fh->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ch = fh->cur_ch;

	if (ch < 0)
		return -EINVAL;

	pix->width = solo_dev->enc_width[ch];
	pix->height = solo_dev->enc_height[ch];
	pix->pixelformat = V4L2_PIX_FMT_MPEG;
	pix->field = V4L2_FIELD_INTERLACED;
	pix->sizeimage = FRAME_BUF_SIZE;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static int solo_enc_reqbufs(struct file *file, void *priv, 
			    struct v4l2_requestbuffers *req)
{
	struct solo_enc_fh *fh = priv;

	return videobuf_reqbufs(&fh->vidq, req);
}

static int solo_enc_querybuf(struct file *file, void *priv,
			     struct v4l2_buffer *buf)
{
	struct solo_enc_fh *fh = priv;

	return videobuf_querybuf(&fh->vidq, buf);
}

static int solo_enc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct solo_enc_fh *fh = priv;

	return videobuf_qbuf(&fh->vidq, buf);
}

static int solo_enc_dqbuf(struct file *file, void *priv,
			  struct v4l2_buffer *buf)
{
	struct solo_enc_fh *fh = priv;

	return videobuf_dqbuf(&fh->vidq, buf, file->f_flags & O_NONBLOCK);
}

static int solo_enc_streamon(struct file *file, void *priv,
			     enum v4l2_buf_type i)
{
	struct solo_enc_fh *fh = priv;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return videobuf_streamon(&fh->vidq);
}

static int solo_enc_streamoff(struct file *file, void *priv,
			      enum v4l2_buf_type i)
{
	struct solo_enc_fh *fh = priv;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return videobuf_streamoff(&fh->vidq);
}

static int solo_enc_s_std(struct file *file, void *priv, v4l2_std_id *i)
{
	return 0;
}

static const struct v4l2_file_operations solo_enc_fops = {
	.owner			= THIS_MODULE,
	.open			= solo_enc_open,
	.release		= solo_enc_release,
	.read			= solo_enc_read,
	.poll			= solo_enc_poll,
	.mmap			= solo_enc_mmap,
	.ioctl			= video_ioctl2,
};

static const struct v4l2_ioctl_ops solo_enc_ioctl_ops = {
	.vidioc_querycap		= solo_enc_querycap,
	.vidioc_s_std			= solo_enc_s_std,
	/* Input callbacks */
	.vidioc_enum_input		= solo_enc_enum_input,
	.vidioc_s_input			= solo_enc_set_input,
	.vidioc_g_input			= solo_enc_get_input,
	/* Video capture format callbacks */
	.vidioc_enum_fmt_vid_cap	= solo_enc_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap		= solo_enc_try_fmt_cap,
	.vidioc_s_fmt_vid_cap		= solo_enc_set_fmt_cap,
	.vidioc_g_fmt_vid_cap		= solo_enc_get_fmt_cap,
	/* Streaming I/O */
	.vidioc_reqbufs			= solo_enc_reqbufs,
	.vidioc_querybuf		= solo_enc_querybuf,
	.vidioc_qbuf			= solo_enc_qbuf,
	.vidioc_dqbuf			= solo_enc_dqbuf,
	.vidioc_streamon		= solo_enc_streamon,
        .vidioc_streamoff		= solo_enc_streamoff,
};

static struct video_device solo_enc_template = {
	.name			= SOLO6010_NAME,
	.fops			= &solo_enc_fops,
	.ioctl_ops		= &solo_enc_ioctl_ops,
	.minor			= -1,
	.release		= video_device_release,

	.tvnorms		= V4L2_STD_NTSC_M,
	.current_norm		= V4L2_STD_NTSC_M,
};

int solo_enc_v4l2_init(struct solo6010_dev *solo_dev)
{
	int ret;

	solo_dev->enc_vfd = video_device_alloc();
	if (!solo_dev->enc_vfd)
		return -ENOMEM;

	*solo_dev->enc_vfd = solo_enc_template;
	solo_dev->enc_vfd->parent = &solo_dev->pdev->dev;

	ret = video_register_device(solo_dev->enc_vfd, VFL_TYPE_GRABBER,
				    video_nr);
	if (ret < 0) {
		video_device_release(solo_dev->enc_vfd);
		solo_dev->enc_vfd = NULL;
		return ret;
	}

	video_set_drvdata(solo_dev->enc_vfd, solo_dev);

	snprintf(solo_dev->enc_vfd->name, sizeof(solo_dev->enc_vfd->name),
		 "%s-enc (%i)", SOLO6010_NAME, solo_dev->enc_vfd->num);

	if (video_nr >= 0)
		video_nr++;

	dev_info(&solo_dev->pdev->dev, "Encoder as /dev/video%d with "
		 "%d inputs\n", solo_dev->enc_vfd->num, solo_dev->nr_chans);

	return 0;
}

void solo_enc_v4l2_exit(struct solo6010_dev *solo_dev)
{
	video_unregister_device(solo_dev->enc_vfd);
	solo_dev->enc_vfd = NULL;
}
