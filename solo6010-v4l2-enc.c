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
#define MP4_QS			16

/* There is 8MB memory available for solo to buffer MPEG4 frames.
 * This gives us 512 * 16kbyte queues. */
#define NR_EXT_QUEUE		512

/* Simple file handle */
struct solo_enc_fh {
	struct solo6010_dev	*solo_dev;
	struct solo_enc_dev	*solo_enc;
	struct videobuf_queue	vidq;
	struct task_struct      *kthread;
	spinlock_t		slock;
	struct list_head	vidq_active;
	wait_queue_head_t	thread_wait;
};

struct solo_enc_buf {
	struct videobuf_buffer	vb;
	struct solo_enc_fh	*fh;
};

extern unsigned video_nr;

/* Returns 0 on success, -errno on failure */
static int solo_update_mode(struct solo_enc_dev *solo_enc, u8 mode)
{
	unsigned long flags;

	if (atomic_read(&solo_enc->ref) > 1)
		return -EBUSY;

	spin_lock_irqsave(&solo_enc->lock, flags);

	solo_enc->mode = mode;
	switch (mode) {
	case SOLO_ENC_MODE_CIF:
		solo_enc->width = 352;
		solo_enc->height = 240;
		break;
	case SOLO_ENC_MODE_HALFD1H:
		solo_enc->width = 704;
		solo_enc->height = 240;
		break;
	case SOLO_ENC_MODE_D1:
		solo_enc->width = 704;
		solo_enc->height = 480;
		break;
	default:
		WARN(1, "mode is unknown");
	}

	spin_unlock_irqrestore(&solo_enc->lock, flags);

	return 0;
}

static void solo_enc_put(struct solo_enc_dev *solo_enc)
{
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	unsigned long flags;

	spin_lock_irqsave(&solo_enc->lock, flags);

	if (atomic_dec_return(&solo_enc->ref)) {
		spin_unlock_irqrestore(&solo_enc->lock, flags);
		return;
	}

	/* No references left, disable it */
	solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(solo_enc->ch), 0);

	spin_unlock_irqrestore(&solo_enc->lock, flags);
}

static void solo_enc_get(struct solo_enc_dev *solo_enc)
{
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	u8 ch = solo_enc->ch;
	unsigned long flags;

	spin_lock_irqsave(&solo_enc->lock, flags);

	/* If we're the first, enable it. This kicks off IRQ's */
	if (atomic_inc_return(&solo_enc->ref) == 1) {
		u8 mode = solo_enc->mode;
		solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), 0);
		solo_reg_write(solo_dev, SOLO_VE_CH_INTL(ch), (mode >> 3) & 1);
		solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), mode);
	}

	spin_unlock_irqrestore(&solo_enc->lock, flags);
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

static void enc_reset_gop(struct solo6010_dev *solo_dev, u8 ch)
{
	BUG_ON(ch >= solo_dev->nr_chans);
	solo_reg_write(solo_dev, SOLO_VE_CH_GOP(ch), 1);
	solo_dev->v4l2_enc[ch]->reset_gop = 1;
}

static int enc_gop_reset(struct solo6010_dev *solo_dev, u8 ch, u8 vop)
{
	if (vop)
		return 1;
	BUG_ON(ch >= solo_dev->nr_chans);
	solo_dev->v4l2_enc[ch]->reset_gop = 0;
	solo_reg_write(solo_dev, SOLO_VE_CH_GOP(ch), SOLO_DEFAULT_GOP);
	return 0;
}

void solo_enc_v4l2_isr(struct solo6010_dev *solo_dev)
{
	struct videnc_status vstatus;
	u32 mpeg_current, mpeg_next;
	u32 reg_mpeg_size, mpeg_size;
	u8 cur_q, vop_type;
	u8 ch;

	solo_reg_write(solo_dev, SOLO_IRQ_STAT, SOLO_IRQ_ENCODER);

	vstatus.status11 = solo_reg_read(solo_dev, SOLO_VE_STATE(11));
	cur_q = (vstatus.status11_st.last_queue + 1) % MP4_QS;
	
	vstatus.status0 = solo_reg_read(solo_dev, SOLO_VE_STATE(0));
	reg_mpeg_size = (vstatus.status0_st.mp4_enc_code_size + 64 + 32) &
			(~31);

	do {
		mpeg_current = solo_reg_read(solo_dev,
					SOLO_VE_MPEG4_QUE(solo_dev->enc_idx));
		solo_dev->enc_idx = (solo_dev->enc_idx + 1) % MP4_QS;
		mpeg_next = solo_reg_read(solo_dev,
					SOLO_VE_MPEG4_QUE(solo_dev->enc_idx));

		ch = (mpeg_current >> 24) & 0x1f;
		vop_type = (mpeg_current >> 29) & 3;
		mpeg_current &= 0x00ffffff;

		mpeg_size = (SOLO_MP4E_EXT_SIZE + mpeg_next - mpeg_current) %
			    SOLO_MP4E_EXT_SIZE;

		/* XXX Shouldn't we tell userspace? */
		if (mpeg_current > mpeg_next && mpeg_size != reg_mpeg_size) {
			enc_reset_gop(solo_dev, ch);
			continue;
		}

		/* When resetting the GOP, skip frames until I-frame */
		if (enc_gop_reset(solo_dev, ch, vop_type))
			continue;
	} while(solo_dev->enc_idx != cur_q);

	return;
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
	struct solo_enc_dev *solo_enc = fh->solo_enc;

	vb->size = FRAME_BUF_SIZE;
	if (vb->baddr != 0 && vb->bsize < vb->size)
		return -EINVAL;

	/* These properties only change when queue is idle */
	vb->width = solo_enc->width;
	vb->height = solo_enc->height;
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
	struct solo_enc_dev *solo_enc = video_drvdata(file);
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct solo_enc_fh *fh;
	int ret;

	if ((fh = kzalloc(sizeof(*fh), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	spin_lock_init(&fh->slock);
	INIT_LIST_HEAD(&fh->vidq_active);
	init_waitqueue_head(&fh->thread_wait);
	fh->solo_dev = solo_dev;
	fh->solo_enc = solo_enc;
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

	solo_enc_get(solo_enc);

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
	struct solo_enc_dev *solo_enc = fh->solo_enc;

	videobuf_stop(&fh->vidq);
	videobuf_mmap_free(&fh->vidq);
	solo_enc_stop_thread(fh);
	solo_enc_put(solo_enc);
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
	struct solo_enc_dev *solo_enc = fh->solo_enc;

	if (input->index)
		return -EINVAL;

	snprintf(input->name, sizeof(input->name), "Encoder %d",
		 solo_enc->ch + 1);
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->std = V4L2_STD_NTSC_M;
	/* XXX Should check for signal status on this camera */
	input->status = 0;

	return 0;
}

static int solo_enc_set_input(struct file *file, void *priv, unsigned int index)
{
	if (index)
		return -EINVAL;

	return 0;
}

static int solo_enc_get_input(struct file *file, void *priv,
			      unsigned int *index)
{
	*index = 0;

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
	struct solo_enc_dev *solo_enc = fh->solo_enc;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u16 width, height;

	width = solo_enc->width;
	height = solo_enc->height;

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
	struct solo_enc_dev *solo_enc = fh->solo_enc;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = solo_enc->width;
	pix->height = solo_enc->height;
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

static struct solo_enc_dev *solo_enc_alloc(struct solo6010_dev *solo_dev, u8 ch)
{
	struct solo_enc_dev *solo_enc;
	int ret;

	solo_enc = kzalloc(sizeof(*solo_enc), GFP_KERNEL);
	if (!solo_enc)
		return ERR_PTR(-ENOMEM);

	solo_enc->vfd = video_device_alloc();
	if (!solo_enc->vfd) {
		kfree(solo_enc);
		return ERR_PTR(-ENOMEM);
	}

	solo_enc->solo_dev = solo_dev;
	solo_enc->ch = ch;

	*solo_enc->vfd = solo_enc_template;
	solo_enc->vfd->parent = &solo_dev->pdev->dev;
	ret = video_register_device(solo_enc->vfd, VFL_TYPE_GRABBER,
				    video_nr);
	if (ret < 0) {
		video_device_release(solo_enc->vfd);
		kfree(solo_enc);
		return ERR_PTR(ret);
	}

	video_set_drvdata(solo_enc->vfd, solo_enc);

	snprintf(solo_enc->vfd->name, sizeof(solo_enc->vfd->name),
		 "%s-enc (%i/%i)", SOLO6010_NAME, solo_dev->vfd->num,
		 solo_enc->vfd->num);

	if (video_nr >= 0)
		video_nr++;

	spin_lock_init(&solo_enc->lock);

	solo_update_mode(solo_enc, SOLO_ENC_MODE_CIF);

	return solo_enc;
}

static void solo_enc_free(struct solo_enc_dev *solo_enc)
{
	video_unregister_device(solo_enc->vfd);
	kfree(solo_enc);
}

int solo_enc_v4l2_init(struct solo6010_dev *solo_dev)
{
	int i;

	for (i = 0; i < solo_dev->nr_chans; i++) {
		solo_dev->v4l2_enc[i] = solo_enc_alloc(solo_dev, i);
		if (IS_ERR(solo_dev->v4l2_enc[i]))
			break;
	}

	if (i != solo_dev->nr_chans) {
		int ret = PTR_ERR(solo_dev->v4l2_enc[i]);
		while (i--)
			solo_enc_free(solo_dev->v4l2_enc[i]);
		return ret;
	}

	dev_info(&solo_dev->pdev->dev, "Encoders as /dev/video%d-%d\n",
		 solo_dev->v4l2_enc[0]->vfd->num,
		 solo_dev->v4l2_enc[solo_dev->nr_chans - 1]->vfd->num);

	return 0;
}

void solo_enc_v4l2_exit(struct solo6010_dev *solo_dev)
{
	int i;

	for (i = 0; i < solo_dev->nr_chans; i++)
		solo_enc_free(solo_dev->v4l2_enc[i]);
}
