/*
 * Copyright (C) 2010 Bluecherry, LLC www.bluecherrydvr.com
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
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/videobuf-dma-contig.h>

#include "solo6010.h"
#include "solo6010-tw28.h"
#include "solo6010-jpeg.h"

#define MIN_VID_BUFFERS		2
#define FRAME_BUF_SIZE		(128 * 1024)
#define MP4_QS			16
#define DMA_ALIGN		128

static int solo_enc_thread(void *data);

extern unsigned video_nr;

struct solo_enc_fh {
	struct			solo_enc_dev *enc;
	u32			fmt;
	u16			rd_idx;
	u8			enc_on;
	u8			jpeg_skip;
	enum solo_enc_types	type;
	struct videobuf_queue	vidq;
	struct list_head	vidq_active;
	struct task_struct	*kthread;
};

struct solo_videobuf {
	struct videobuf_buffer	vb;
	unsigned int		flags;
};

static unsigned char vid_vop_header[] = {
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x20,
	0x02, 0x48, 0x05, 0xc0, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x80, 0x00, 0x97, 0x53, 0x04,
	0x1f, 0x4c, 0x58, 0x10, 0x78, 0x51, 0x18, 0x3f,
};

/*
 * Things we can change around:
 *
 * byte  10,        4-bits 01111000                   aspect
 * bytes 21,22,23  16-bits 000x1111 11111111 1111x000 fps/res
 * bytes 23,24,25  15-bits 00000n11 11111111 11111x00 interval
 * bytes 25,26,27  13-bits 00000x11 11111111 111x0000 width
 * bytes 27,28,29  13-bits 000x1111 11111111 1x000000 height
 * byte  29         1-bit  0x100000                   interlace
 */

/* For aspect */
#define XVID_PAR_43_PAL		2
#define XVID_PAR_43_NTSC	3

static const u32 solo_user_ctrls[] = {
	V4L2_CID_BRIGHTNESS,
	V4L2_CID_CONTRAST,
	V4L2_CID_SATURATION,
	V4L2_CID_HUE,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
	V4L2_CID_SHARPNESS,
#endif
	0
};

static const u32 solo_mpeg_ctrls[] = {
	V4L2_CID_MPEG_VIDEO_ENCODING,
	V4L2_CID_MPEG_VIDEO_GOP_SIZE,
	0
};

static const u32 solo_private_ctrls[] = {
	V4L2_CID_MOTION_ENABLE,
	V4L2_CID_MOTION_THRESHOLD,
	0
};

static const u32 solo_fmtx_ctrls[] = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	V4L2_CID_RDS_TX_RADIO_TEXT,
#endif
	0
};

static const u32 *solo_ctrl_classes[] = {
	solo_user_ctrls,
	solo_mpeg_ctrls,
	solo_fmtx_ctrls,
	solo_private_ctrls,
	NULL
};

struct vop_header {
	/* VE_STATUS0 */
	u32 mpeg_size:20, sad_motion_flag:1, video_motion_flag:1, vop_type:2,
		channel:5, source_fl:1, interlace:1, progressive:1;

	/* VE_STATUS1 */
	u32 vsize:8, hsize:8, last_queue:4, nop0:8, scale:4;

	/* VE_STATUS2 */
	u32 mpeg_off;

	/* VE_STATUS3 */
	u32 jpeg_off;

	/* VE_STATUS4 */
	u32 jpeg_size:20, interval:10, nop1:2;

	/* VE_STATUS5/6 */
	u32 sec, usec;

	/* VE_STATUS7/8/9 */
	u32 nop2[3];

	/* VE_STATUS10 */
	u32 mpeg_size_alt:20, nop3:12;

	u32 end_nops[5];
} __attribute__((packed));

static int solo_is_motion_on(struct solo_enc_dev *solo_enc)
{
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	u8 ch = solo_enc->ch;
	unsigned long flags;
	int res;

	spin_lock_irqsave(&solo_enc->motion_lock, flags);
	res = (solo_dev->motion_mask & (1 << ch)) ? 1 : 0;
	spin_unlock_irqrestore(&solo_enc->motion_lock, flags);

	return res;
}

static void solo_motion_toggle(struct solo_enc_dev *solo_enc, int on)
{
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	u32 mask = 1 << solo_enc->ch;
	unsigned long flags;

	spin_lock_irqsave(&solo_enc->motion_lock, flags);

	if (on)
		solo_dev->motion_mask |= mask;
	else
		solo_dev->motion_mask &= ~mask;

	solo_reg_write(solo_dev, SOLO_VI_MOT_CLEAR, mask);

	solo_reg_write(solo_dev, SOLO_VI_MOT_ADR,
		       SOLO_VI_MOTION_EN(solo_dev->motion_mask) |
		       (SOLO_MOTION_EXT_ADDR(solo_dev) >> 16));

	spin_unlock_irqrestore(&solo_enc->motion_lock, flags);
}

/* Should be called with solo_enc->enable_lock held */
static void solo_update_mode(struct solo_enc_dev *solo_enc)
{
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	assert_spin_locked(&solo_enc->enable_lock);

	solo_enc->interlaced = (solo_enc->mode & 0x08) ? 1 : 0;
	solo_enc->bw_weight = max(solo_dev->fps / solo_enc->interval, 1);

	switch (solo_enc->mode) {
	case SOLO_ENC_MODE_CIF:
		solo_enc->width = solo_dev->video_hsize >> 1;
		solo_enc->height = solo_dev->video_vsize;
		break;
	case SOLO_ENC_MODE_D1:
		solo_enc->width = solo_dev->video_hsize;
		solo_enc->height = solo_dev->video_vsize << 1;
		solo_enc->bw_weight <<= 2;
		break;
	default:
		WARN(1, "mode is unknown");
	}
}

/* Should be called with solo_enc->enable_lock held */
static int __solo_enc_on(struct solo_enc_fh *fh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	u8 ch = solo_enc->ch;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	u8 interval;

	assert_spin_locked(&solo_enc->enable_lock);

	if (fh->enc_on)
		return 0;

	solo_update_mode(solo_enc);

	/* Make sure to bw check on first reader */
	if (!atomic_read(&solo_enc->readers)) {
		if (solo_enc->bw_weight > solo_dev->enc_bw_remain)
			return -EBUSY;
		else
			solo_dev->enc_bw_remain -= solo_enc->bw_weight;
	}

	fh->enc_on = 1;
	fh->rd_idx = solo_enc->enc_wr_idx;

	if (fh->type == SOLO_ENC_TYPE_EXT)
		solo_reg_write(solo_dev, SOLO_CAP_CH_COMP_ENA_E(ch), 1);

	/* Reset the encoder if we are the first mpeg reader, else only reset
	 * on the first mjpeg reader. */
	if (fh->fmt == V4L2_PIX_FMT_MPEG) {
		atomic_inc(&solo_enc->readers);
		if (atomic_inc_return(&solo_enc->mpeg_readers) > 1)
			return 0;
	} else if (atomic_inc_return(&solo_enc->readers) > 1) {
		return 0;
	}

	/* Disable all encoding for this channel */
	solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), 0);

	/* Common for both std and ext encoding */
	solo_reg_write(solo_dev, SOLO_VE_CH_INTL(ch),
		       solo_enc->interlaced ? 1 : 0);

	if (solo_enc->interlaced)
		interval = solo_enc->interval - 1;
	else
		interval = solo_enc->interval;

	/* Standard encoding only */
	solo_reg_write(solo_dev, SOLO_VE_CH_GOP(ch), solo_enc->gop);
	solo_reg_write(solo_dev, SOLO_VE_CH_QP(ch), solo_enc->qp);
	solo_reg_write(solo_dev, SOLO_CAP_CH_INTV(ch), interval);

	/* Extended encoding only */
	solo_reg_write(solo_dev, SOLO_VE_CH_GOP_E(ch), solo_enc->gop);
	solo_reg_write(solo_dev, SOLO_VE_CH_QP_E(ch), solo_enc->qp);
	solo_reg_write(solo_dev, SOLO_CAP_CH_INTV_E(ch), interval);

	/* Enables the standard encoder */
	solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(ch), solo_enc->mode);

	/* Settle down Beavis... */
//	mdelay(10);

	return 0;
}

static int solo_enc_on(struct solo_enc_fh *fh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	int ret;

	spin_lock(&solo_enc->enable_lock);
	ret = __solo_enc_on(fh);
	spin_unlock(&solo_enc->enable_lock);

	return ret;
}

static void __solo_enc_off(struct solo_enc_fh *fh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	assert_spin_locked(&solo_enc->enable_lock);

	if (!fh->enc_on)
		return;

	if (fh->kthread) {
		kthread_stop(fh->kthread);
		fh->kthread = NULL;
	}

	fh->enc_on = 0;

	if (fh->fmt == V4L2_PIX_FMT_MPEG)
		atomic_dec(&solo_enc->mpeg_readers);

	if (atomic_dec_return(&solo_enc->readers) > 0)
		return;

	solo_dev->enc_bw_remain += solo_enc->bw_weight;

	solo_reg_write(solo_dev, SOLO_CAP_CH_SCALE(solo_enc->ch), 0);
	solo_reg_write(solo_dev, SOLO_CAP_CH_COMP_ENA_E(solo_enc->ch), 0);
}

static void solo_enc_off(struct solo_enc_fh *fh)
{
	struct solo_enc_dev *solo_enc = fh->enc;

	spin_lock(&solo_enc->enable_lock);
	__solo_enc_off(fh);
	spin_unlock(&solo_enc->enable_lock);
}

static int solo_start_fh_thread(struct solo_enc_fh *fh)
{
	fh->kthread = kthread_run(solo_enc_thread, fh, SOLO6010_NAME "_enc");

	/* Oops, we had a problem */
	if (IS_ERR(fh->kthread)) {
		int err = PTR_ERR(fh->kthread);

		fh->kthread = NULL;
		solo_enc_off(fh);

		return err;
	}

	return 0;
}

static int solo_enc_start(struct solo_enc_fh *fh)
{
	int ret;

	if (fh->enc_on)
		return 0;

	ret = solo_enc_on(fh);
	if (ret)
		return ret;

	ret = solo_start_fh_thread(fh);
	if (ret)
		return ret;

	return 0;
}

static int enc_get_mpeg_dma_t(struct solo6010_dev *solo_dev, dma_addr_t buf,
			      unsigned int off, unsigned int size)
{
	int ret;

	if (off > SOLO_MP4E_EXT_SIZE(solo_dev))
		return -EINVAL;

	if (off + size <= SOLO_MP4E_EXT_SIZE(solo_dev))
		return solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_MP4E, 0, buf,
				      SOLO_MP4E_EXT_ADDR(solo_dev) + off, size,
				      0, 0);

	/* Buffer wrap */
	ret = solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_MP4E, 0, buf,
			    SOLO_MP4E_EXT_ADDR(solo_dev) + off,
			    SOLO_MP4E_EXT_SIZE(solo_dev) - off, 0, 0);

	ret |= solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_MP4E, 0,
			      buf + SOLO_MP4E_EXT_SIZE(solo_dev) - off,
			      SOLO_MP4E_EXT_ADDR(solo_dev),
			      size + off - SOLO_MP4E_EXT_SIZE(solo_dev), 0, 0);

	return ret;
}

static int enc_get_mpeg_dma(struct solo6010_dev *solo_dev, void *buf,
			    unsigned int off, unsigned int size)
{
	int ret;

	dma_addr_t dma_addr = pci_map_single(solo_dev->pdev, buf, size,
					     PCI_DMA_FROMDEVICE);
	ret = enc_get_mpeg_dma_t(solo_dev, dma_addr, off, size);
	pci_unmap_single(solo_dev->pdev, dma_addr, size, PCI_DMA_FROMDEVICE);

	return ret;
}

static int enc_get_jpeg_dma(struct solo6010_dev *solo_dev, dma_addr_t buf,
			    unsigned int off, unsigned int size)
{
	int ret;

	if (off > SOLO_JPEG_EXT_SIZE(solo_dev))
		return -EINVAL;

	if (off + size <= SOLO_JPEG_EXT_SIZE(solo_dev))
		return solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_JPEG, 0, buf,
				      SOLO_JPEG_EXT_ADDR(solo_dev) + off, size,
				      0, 0);

	/* Buffer wrap */
	ret = solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_JPEG, 0, buf,
			     SOLO_JPEG_EXT_ADDR(solo_dev) + off,
			     SOLO_JPEG_EXT_SIZE(solo_dev) - off, 0, 0);

	ret |= solo_p2m_dma_t(solo_dev, SOLO_P2M_DMA_ID_JPEG, 0,
			      buf + SOLO_JPEG_EXT_SIZE(solo_dev) - off,
			      SOLO_JPEG_EXT_ADDR(solo_dev),
			      size + off - SOLO_JPEG_EXT_SIZE(solo_dev),
			      0, 0);

	return ret;
}

static int solo_fill_jpeg(struct solo_enc_fh *fh, struct videobuf_buffer *vb,
			  dma_addr_t vbuf, struct vop_header *vh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	u8 *p = videobuf_queue_to_vmalloc(&fh->vidq, vb);
	int frame_size;

	vh->jpeg_off -= SOLO_JPEG_EXT_ADDR(solo_dev);

	if (WARN_ON_ONCE(vb->bsize < (vh->jpeg_size + sizeof(jpeg_header))))
		return -1;

	memcpy(p, jpeg_header, sizeof(jpeg_header));
	p[SOF0_START + 5] = 0xff & (solo_enc->height >> 8);
	p[SOF0_START + 6] = 0xff & solo_enc->height;
	p[SOF0_START + 7] = 0xff & (solo_enc->width >> 8);
	p[SOF0_START + 8] = 0xff & solo_enc->width;

	vb->width = solo_enc->width;
        vb->height = solo_enc->height;

	vbuf += sizeof(jpeg_header);
	vb->size = vh->jpeg_size + sizeof(jpeg_header);
	frame_size = (vh->jpeg_size + (DMA_ALIGN - 1)) & ~(DMA_ALIGN - 1);

	return enc_get_jpeg_dma(solo_dev, vbuf, vh->jpeg_off, frame_size);
}

static int solo_fill_mpeg(struct solo_enc_fh *fh, struct videobuf_buffer *vb,
			  dma_addr_t vbuf, struct vop_header *vh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	int frame_off, frame_size;

	if (WARN_ON_ONCE(vb->bsize < vh->mpeg_size))
		return -1;

	vb->width = vh->hsize << 4;
	vb->height = vh->vsize << 4;
	vb->size = vh->mpeg_size;
	vb->ts.tv_sec = vh->sec;
	vb->ts.tv_usec = vh->usec;

	/* If this is a key frame, add extra m4v header */
	if (!vh->vop_type) {
		u16 fps = solo_dev->fps * 1000;
		u16 interval = solo_enc->interval * 1000;
		u8 *p = videobuf_queue_to_vmalloc(&fh->vidq, vb);

		memcpy(p, vid_vop_header, sizeof(vid_vop_header));

		if (solo_dev->video_type == SOLO_VO_FMT_TYPE_NTSC)
			p[10] |= ((XVID_PAR_43_NTSC << 3) & 0x78);
		else
			p[10] |= ((XVID_PAR_43_PAL << 3) & 0x78);

		/* Frame rate and interval */
		p[22] = fps >> 4;
		p[23] = ((fps << 4) & 0xf0) | 0x0c | ((interval >> 13) & 0x3);
		p[24] = (interval >> 5) & 0xff;
		p[25] = ((interval << 3) & 0xf8) | 0x04;

		/* Width and height */
		p[26] = (vb->width >> 3) & 0xff;
		p[27] = ((vb->height >> 9) & 0x0f) | 0x10;
		p[28] = (vb->height >> 1) & 0xff;

		/* Interlace */
		if (vh->interlace)
			p[29] |= 0x20;

		/* Adjust the dma buffer past this header */
		vb->size += sizeof(vid_vop_header);
		vbuf += sizeof(vid_vop_header);
	}

	/* Now get the actual mpeg payload */
	frame_off = (vh->mpeg_off + sizeof(*vh)) % SOLO_MP4E_EXT_SIZE(solo_dev);
	frame_size = (vh->mpeg_size + (DMA_ALIGN - 1)) & ~(DMA_ALIGN - 1);

	if (enc_get_mpeg_dma_t(solo_dev, vbuf, frame_off, frame_size))
		return -1;

	return 0;
}

static void solo_enc_fillbuf(struct solo_enc_fh *fh,
			     struct videobuf_buffer *vb,
			     struct solo_enc_buf *enc_buf)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct solo_videobuf *svb = (struct solo_videobuf *)vb;
	struct vop_header vh;
	dma_addr_t vbuf;
	int ret = -EIO;

	if (WARN_ON_ONCE(!(vbuf = videobuf_to_dma_contig(vb))))
		goto vbuf_error;

	/* We need this for mpeg and jpeg */
	if (enc_get_mpeg_dma(solo_dev, &vh, enc_buf->off, sizeof(vh)))
		goto vbuf_error;

	vh.mpeg_off -= SOLO_MP4E_EXT_ADDR(solo_dev);
	if (WARN_ON_ONCE(vh.mpeg_off != enc_buf->off))
		goto vbuf_error;

	if (fh->fmt == V4L2_PIX_FMT_MPEG)
		ret = solo_fill_mpeg(fh, vb, vbuf, &vh);
	else
		ret = solo_fill_jpeg(fh, vb, vbuf, &vh);

vbuf_error:
	svb->flags = 0;
	if (ret) {
		vb->state = VIDEOBUF_ERROR;
	} else {
		vb->state = VIDEOBUF_DONE;
		vb->field_count++;
		vb->width = solo_enc->width;
		vb->height = solo_enc->height;

		/* Check for motion flags */
		if (solo_is_motion_on(solo_enc)) {
			svb->flags |= V4L2_BUF_FLAG_MOTION_ON;
			if (solo_reg_read(solo_dev, SOLO_VI_MOT_STATUS) &
			    (1 << solo_enc->ch)) {
				solo_reg_write(solo_dev, SOLO_VI_MOT_CLEAR,
					       (1 << solo_enc->ch));
				svb->flags |= V4L2_BUF_FLAG_MOTION_DETECTED;
			}
		}

		/* Set P/I frame */
		if (!vh.vop_type)
			svb->flags |= V4L2_BUF_FLAG_KEYFRAME;
		else
			svb->flags |= V4L2_BUF_FLAG_PFRAME;
	}

	wake_up(&vb->done);

	return;
}

static void solo_enc_thread_try(struct solo_enc_fh *fh)
{
	struct solo_enc_dev *solo_enc = fh->enc;
	unsigned long flags;

	for (;;) {
		struct videobuf_buffer *vb;
		struct solo_enc_buf *enc_buf = NULL;

		spin_lock_irqsave(&solo_enc->av_lock, flags);

		/* First check if the encoder has given us anything to use */
		for (;fh->rd_idx != solo_enc->enc_wr_idx;
		     fh->rd_idx = (fh->rd_idx + 1) % SOLO_NR_RING_BUFS) {
			struct solo_enc_buf *ebuf =
				&solo_enc->enc_buf[fh->rd_idx];

			if (fh->fmt == V4L2_PIX_FMT_MPEG &&
			    fh->type != ebuf->type)
				continue;

			enc_buf = ebuf;

			/* For mjpeg, we never skip a frame */
			if (fh->fmt == V4L2_PIX_FMT_MPEG)
				break;

			/* If we're here, then we are mjpeg. Skip if client is slow */
			if (!fh->jpeg_skip)
				break;
			fh->jpeg_skip--;
		}

		if (!enc_buf)
			break;

		if (list_empty(&fh->vidq_active))
			break;

		vb = list_first_entry(&fh->vidq_active,
				      struct videobuf_buffer, queue);

		/* If this happens, enable skipping mjpeg frames */
		if (!waitqueue_active(&vb->done)) {
			fh->jpeg_skip++;
			break;
		}

		/* From here, the video buf is in our care... */
		list_del(&vb->queue);
		fh->rd_idx = (fh->rd_idx + 1) % SOLO_NR_RING_BUFS;
		spin_unlock_irqrestore(&solo_enc->av_lock, flags);

		solo_enc_fillbuf(fh, vb, enc_buf);
	}

	assert_spin_locked(&solo_enc->av_lock);
	spin_unlock(&solo_enc->av_lock);
}

static int solo_enc_thread(void *data)
{
	struct solo_enc_fh *fh = data;
	struct solo_enc_dev *solo_enc = fh->enc;
	DECLARE_WAITQUEUE(wait, current);

	set_freezable();
	add_wait_queue(&solo_enc->thread_wait, &wait);

	for (;;) {
		long timeout = schedule_timeout_interruptible(HZ);
		if (timeout == -ERESTARTSYS || kthread_should_stop())
			break;
		solo_enc_thread_try(fh);
		try_to_freeze();
	}

	remove_wait_queue(&solo_enc->thread_wait, &wait);

        return 0;
}

void solo_enc_v4l2_isr(struct solo6010_dev *solo_dev)
{
	struct solo_enc_dev *solo_enc;
	struct solo_enc_buf *enc_buf;
	struct videnc_status vstatus;
	u32 mpeg_current;
	u8 cur_q, ch;
	enum solo_enc_types enc_type;

	vstatus.status11 = solo_reg_read(solo_dev, SOLO_VE_STATE(11));
	cur_q = (vstatus.status11_st.last_queue + 1) % MP4_QS;

	while (solo_dev->enc_idx != cur_q) {
		mpeg_current = solo_reg_read(solo_dev,
					SOLO_VE_MPEG4_QUE(solo_dev->enc_idx));
		solo_dev->enc_idx = (solo_dev->enc_idx + 1) % MP4_QS;

		ch = (mpeg_current >> 24) & 0x1f;

		if (ch >= SOLO_MAX_CHANNELS) {
			ch -= SOLO_MAX_CHANNELS;
			enc_type = SOLO_ENC_TYPE_EXT;
		} else
			enc_type = SOLO_ENC_TYPE_STD;

		solo_enc = solo_dev->v4l2_enc[ch];
		BUG_ON(solo_enc == NULL);
		enc_buf = &solo_enc->enc_buf[solo_enc->enc_wr_idx];

		enc_buf->off = mpeg_current & 0x00ffffff;
		enc_buf->type = enc_type;

		solo_enc->enc_wr_idx = (solo_enc->enc_wr_idx + 1) %
					SOLO_NR_RING_BUFS;

		wake_up_interruptible_all(&solo_enc->thread_wait);
	}

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
	vb->size = FRAME_BUF_SIZE;
	if (vb->baddr != 0 && vb->bsize < vb->size)
		return -EINVAL;

	/* This property only change when queue is idle */
	vb->field = field;

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		int rc = videobuf_iolock(vq, vb, NULL);
		if (rc < 0) {
			videobuf_dma_contig_free(vq, vb);
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
	wake_up_interruptible(&fh->enc->thread_wait);
}

static void solo_enc_buf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	videobuf_dma_contig_free(vq, vb);
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

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static int solo_enc_open(struct file *file)
#else
static int solo_enc_open(struct inode *ino, struct file *file)
#endif
{
	struct solo_enc_dev *solo_enc = video_drvdata(file);
	struct solo_enc_fh *fh;

	if ((fh = kzalloc(sizeof(*fh), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	fh->enc = solo_enc;
	file->private_data = fh;
	INIT_LIST_HEAD(&fh->vidq_active);
	fh->fmt = V4L2_PIX_FMT_MPEG;
	fh->type = SOLO_ENC_TYPE_STD;

	videobuf_queue_dma_contig_init(&fh->vidq, &solo_enc_video_qops,
				    &solo_enc->solo_dev->pdev->dev,
				    &solo_enc->av_lock,
				    V4L2_BUF_TYPE_VIDEO_CAPTURE,
				    V4L2_FIELD_INTERLACED,
				    sizeof(struct solo_videobuf), fh);

	return 0;
}

static ssize_t solo_enc_read(struct file *file, char __user *data,
			     size_t count, loff_t *ppos)
{
	struct solo_enc_fh *fh = file->private_data;
	int ret;

	/* Make sure the encoder is on */
	ret = solo_enc_start(fh);
	if (ret)
		return ret;

	return videobuf_read_stream(&fh->vidq, data, count, ppos, 0,
				    file->f_flags & O_NONBLOCK);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static int solo_enc_release(struct file *file)
#else
static int solo_enc_release(struct inode *ino, struct file *file)
#endif
{
	struct solo_enc_fh *fh = file->private_data;

	videobuf_stop(&fh->vidq);
	videobuf_mmap_free(&fh->vidq);

	solo_enc_off(fh);

	kfree(fh);

	return 0;
}

static int solo_enc_querycap(struct file *file, void  *priv,
			     struct v4l2_capability *cap)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	strcpy(cap->driver, SOLO6010_NAME);
	snprintf(cap->card, sizeof(cap->card), "Softlogic 6010 Enc %d",
		 solo_enc->ch);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI %s",
		 pci_name(solo_dev->pdev));
	cap->version = SOLO6010_VER_NUM;
	cap->capabilities =     V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_READWRITE |
				V4L2_CAP_STREAMING;
	return 0;
}

static int solo_enc_enum_input(struct file *file, void *priv,
			       struct v4l2_input *input)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	if (input->index)
		return -EINVAL;

	snprintf(input->name, sizeof(input->name), "Encoder %d",
		 solo_enc->ch + 1);
	input->type = V4L2_INPUT_TYPE_CAMERA;

	if (solo_dev->video_type == SOLO_VO_FMT_TYPE_NTSC)
		input->std = V4L2_STD_NTSC_M;
	else
		input->std = V4L2_STD_PAL_B;

	if (!tw28_get_video_status(solo_dev, solo_enc->ch))
		input->status = V4L2_IN_ST_NO_SIGNAL;

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
	switch (f->index) {
	case 0:
		f->pixelformat = V4L2_PIX_FMT_MPEG;
		strcpy(f->description, "MPEG-4 AVC");
		break;
	case 1:
		f->pixelformat = V4L2_PIX_FMT_MJPEG;
		strcpy(f->description, "MJPEG");
		break;
	default:
		return -EINVAL;
	}

	f->flags = V4L2_FMT_FLAG_COMPRESSED;

	return 0;
}

static int solo_enc_try_fmt_cap(struct file *file, void *priv,
			    struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->pixelformat != V4L2_PIX_FMT_MPEG &&
	    pix->pixelformat != V4L2_PIX_FMT_MJPEG)
		return -EINVAL;

	/* We cannot change width/height in mid mpeg */
	if (atomic_read(&solo_enc->mpeg_readers) > 0) {
		if (pix->width != solo_enc->width ||
		    pix->height != solo_enc->height)
			return -EBUSY;
	} else if (!(pix->width == solo_dev->video_hsize &&
	      pix->height == solo_dev->video_vsize << 1) &&
	    !(pix->width == solo_dev->video_hsize >> 1 &&
	      pix->height == solo_dev->video_vsize)) {
		/* Default to CIF 1/2 size */
		pix->width = solo_dev->video_hsize >> 1;
		pix->height = solo_dev->video_vsize;
	}

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_INTERLACED;
	else if (pix->field != V4L2_FIELD_INTERLACED) {
		pix->field = V4L2_FIELD_INTERLACED;
	}

	/* Just set these */
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->sizeimage = FRAME_BUF_SIZE;

	return 0;
}

static int solo_enc_set_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	spin_lock(&solo_enc->enable_lock);

	ret = solo_enc_try_fmt_cap(file, priv, f);
	if (ret) {
		spin_unlock(&solo_enc->enable_lock);
		return ret;
	}

	if (pix->width == solo_dev->video_hsize)
		solo_enc->mode = SOLO_ENC_MODE_D1;
	else
		solo_enc->mode = SOLO_ENC_MODE_CIF;

	/* This does not change the encoder at all */
	fh->fmt = pix->pixelformat;

	if (pix->priv)
		fh->type = SOLO_ENC_TYPE_EXT;

	ret = __solo_enc_on(fh);

	spin_unlock(&solo_enc->enable_lock);

	if (ret)
		return ret;

	return solo_start_fh_thread(fh);
}

static int solo_enc_get_fmt_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = solo_enc->width;
	pix->height = solo_enc->height;
	pix->pixelformat = fh->fmt;
	pix->field = solo_enc->interlaced ? V4L2_FIELD_INTERLACED :
		     V4L2_FIELD_NONE;
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
	struct solo_videobuf *svb;
	int ret;

	/* Make sure the encoder is on */
	ret = solo_enc_start(fh);
	if (ret)
		return ret;

	ret = videobuf_dqbuf(&fh->vidq, buf, file->f_flags & O_NONBLOCK);
	if (ret)
		return ret;

	/* Copy over the flags */
	svb = (struct solo_videobuf *)fh->vidq.bufs[buf->index];
	buf->flags |= svb->flags;

	return 0;
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
	int ret;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret = videobuf_streamoff(&fh->vidq);
	if (!ret)
		solo_enc_off(fh);

	return ret;
}

static int solo_enc_s_std(struct file *file, void *priv, v4l2_std_id *i)
{
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
static int solo_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	struct solo_enc_fh *fh = priv;
	struct solo6010_dev *solo_dev = fh->enc->solo_dev;

	if (fsize->pixel_format != V4L2_PIX_FMT_MPEG)
		return -EINVAL;

	switch (fsize->index) {
	case 0:
		fsize->discrete.width = solo_dev->video_hsize >> 1;
		fsize->discrete.height = solo_dev->video_vsize;
		break;
	case 1:
		fsize->discrete.width = solo_dev->video_hsize;
		fsize->discrete.height = solo_dev->video_vsize << 1;
		break;
	default:
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	return 0;
}

static int solo_enum_frameintervals(struct file *file, void *priv,
				    struct v4l2_frmivalenum *fintv)
{
	struct solo_enc_fh *fh = priv;
	struct solo6010_dev *solo_dev = fh->enc->solo_dev;

	if (fintv->pixel_format != V4L2_PIX_FMT_MPEG || fintv->index)
		return -EINVAL;

	fintv->type = V4L2_FRMIVAL_TYPE_STEPWISE;

	fintv->stepwise.min.numerator = solo_dev->fps;
	fintv->stepwise.min.denominator = 1;

	fintv->stepwise.max.numerator = solo_dev->fps;
	fintv->stepwise.max.denominator = 15;

	fintv->stepwise.step.numerator = 1;
	fintv->stepwise.step.denominator = 1;

	return 0;
}
#endif /* frame sizes/intervals */

static int solo_g_parm(struct file *file, void *priv,
		       struct v4l2_streamparm *sp)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct v4l2_captureparm *cp = &sp->parm.capture;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = solo_enc->interval;
	cp->timeperframe.denominator = solo_dev->fps;
	cp->capturemode = 0;
	/* XXX: Shouldn't we be able to get/set this from videobuf? */
	cp->readbuffers = 2;

        return 0;
}

static int solo_s_parm(struct file *file, void *priv,
		       struct v4l2_streamparm *sp)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;
	struct v4l2_captureparm *cp = &sp->parm.capture;

	spin_lock(&solo_enc->enable_lock);

	if (atomic_read(&solo_enc->mpeg_readers) > 0) {
		spin_unlock(&solo_enc->enable_lock);
		return -EBUSY;
	}

	if ((cp->timeperframe.numerator == 0) ||
	    (cp->timeperframe.denominator == 0)) {
		/* reset framerate */
		cp->timeperframe.numerator = 1;
		cp->timeperframe.denominator = solo_dev->fps;
	}

	if (cp->timeperframe.denominator != solo_dev->fps)
		cp->timeperframe.denominator = solo_dev->fps;

	if (cp->timeperframe.numerator > 15)
		cp->timeperframe.numerator = 15;

	solo_enc->interval = cp->timeperframe.numerator;

	cp->capability = V4L2_CAP_TIMEPERFRAME;

	solo_enc->gop = max(solo_dev->fps / solo_enc->interval, 1);
	solo_update_mode(solo_enc);

	spin_unlock(&solo_enc->enable_lock);

        return 0;
}

static int solo_queryctrl(struct file *file, void *priv,
			  struct v4l2_queryctrl *qc)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	qc->id = v4l2_ctrl_next(solo_ctrl_classes, qc->id);
	if (!qc->id)
		return -EINVAL;

	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, 0x00, 0xff, 1, 0x80);
	case V4L2_CID_SHARPNESS:
		return v4l2_ctrl_query_fill(qc, 0x00, 0x0f, 1, 0x00);
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		return v4l2_ctrl_query_fill(
			qc, V4L2_MPEG_VIDEO_ENCODING_MPEG_1,
			V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC, 1,
			V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC);
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		return v4l2_ctrl_query_fill(qc, 1, 255, 1, solo_dev->fps);
#ifdef PRIVATE_CIDS
	case V4L2_CID_MOTION_THRESHOLD:
		qc->flags |= V4L2_CTRL_FLAG_SLIDER;
		qc->type = V4L2_CTRL_TYPE_INTEGER;
		qc->minimum = 0;
		qc->maximum = 0xffff;
		qc->step = 1;
		qc->default_value = SOLO_DEF_MOT_THRESH;
		strlcpy(qc->name, "Motion Detection Threshold",
			sizeof(qc->name));
		return 0;
	case V4L2_CID_MOTION_ENABLE:
		qc->type = V4L2_CTRL_TYPE_BOOLEAN;
		qc->minimum = 0;
		qc->maximum = qc->step = 1;
		qc->default_value = 0;
		strlcpy(qc->name, "Motion Detection Enable", sizeof(qc->name));
		return 0;
#else
	case V4L2_CID_MOTION_THRESHOLD:
		return v4l2_ctrl_query_fill(qc, 0, 0xffff, 1,
					    SOLO_DEF_MOT_THRESH);
	case V4L2_CID_MOTION_ENABLE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	case V4L2_CID_RDS_TX_RADIO_TEXT:
		qc->type = V4L2_CTRL_TYPE_STRING;
		qc->minimum = 0;
		qc->maximum = OSD_TEXT_MAX;
		qc->step = 1;
		qc->default_value = 0;
		strlcpy(qc->name, "OSD Text", sizeof(qc->name));
		return 0;
#endif
	}

        return -EINVAL;
}

static int solo_querymenu(struct file *file, void *priv,
			  struct v4l2_querymenu *qmenu)
{
	struct v4l2_queryctrl qctrl;
	int err;

	qctrl.id = qmenu->id;
	if ((err = solo_queryctrl(file, priv, &qctrl)))
		return err;

	return v4l2_ctrl_query_menu(qmenu, &qctrl, NULL);
}

static int solo_g_ctrl(struct file *file, void *priv,
		       struct v4l2_control *ctrl)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_SHARPNESS:
		return tw28_get_ctrl_val(solo_dev, ctrl->id, solo_enc->ch,
					 &ctrl->value);
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		ctrl->value = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctrl->value = solo_enc->gop;
		break;
	case V4L2_CID_MOTION_THRESHOLD:
		ctrl->value = solo_enc->motion_thresh;
		break;
	case V4L2_CID_MOTION_ENABLE:
		ctrl->value = solo_is_motion_on(solo_enc);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int solo_s_ctrl(struct file *file, void *priv,
		       struct v4l2_control *ctrl)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	struct solo6010_dev *solo_dev = solo_enc->solo_dev;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_SHARPNESS:
		return tw28_set_ctrl_val(solo_dev, ctrl->id, solo_enc->ch,
					 ctrl->value);
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		if (ctrl->value != V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC)
			return -ERANGE;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		if (ctrl->value < 1 || ctrl->value > 255)
			return -ERANGE;
		solo_enc->gop = ctrl->value;
		solo_reg_write(solo_dev, SOLO_VE_CH_GOP(solo_enc->ch),
			       solo_enc->gop);
		solo_reg_write(solo_dev, SOLO_VE_CH_GOP_E(solo_enc->ch),
			       solo_enc->gop);
		break;
	case V4L2_CID_MOTION_THRESHOLD:
	{
		u16 block = (ctrl->value >> 16) & 0xffff;
		u16 value = ctrl->value & 0xffff;

		/* Block value > 0 means a specific block, else global */
		if (block > 4096)
			return -ERANGE;

		if (block == 0) {
			solo_enc->motion_thresh = value;
			solo_set_motion_threshold(solo_dev, solo_enc->ch, value);
		} else {
			solo_set_motion_block(solo_dev, solo_enc->ch, value,
					      block - 1);
		}
		break;
	}
	case V4L2_CID_MOTION_ENABLE:
		solo_motion_toggle(solo_enc, ctrl->value);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int solo_s_ext_ctrls(struct file *file, void *priv,
			    struct v4l2_ext_controls *ctrls)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	int i;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ctrl = (ctrls->controls + i);
		int err;

		switch (ctrl->id) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		case V4L2_CID_RDS_TX_RADIO_TEXT:
			if (ctrl->size - 1 > OSD_TEXT_MAX)
                                err = -ERANGE;
			else {
				mutex_lock(&solo_enc->osd_mutex);
                        	err = copy_from_user(solo_enc->osd_text,
						     ctrl->string,
						     OSD_TEXT_MAX);
				solo_enc->osd_text[OSD_TEXT_MAX] = '\0';
				if (!err)
					err = solo_osd_print(solo_enc);
				mutex_unlock(&solo_enc->osd_mutex);
			}
			break;
#endif
		default:
			err = -EINVAL;
		}

		if (err < 0) {
			ctrls->error_idx = i;
			return err;
		}
	}

	return 0;
}

static int solo_g_ext_ctrls(struct file *file, void *priv,
			    struct v4l2_ext_controls *ctrls)
{
	struct solo_enc_fh *fh = priv;
	struct solo_enc_dev *solo_enc = fh->enc;
	int i;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ctrl = (ctrls->controls + i);
		int err;

		switch (ctrl->id) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		case V4L2_CID_RDS_TX_RADIO_TEXT:
			if (ctrl->size < OSD_TEXT_MAX) {
				ctrl->size = OSD_TEXT_MAX;
				err = -ENOSPC;
			} else {
				mutex_lock(&solo_enc->osd_mutex);
				err = copy_to_user(ctrl->string,
						   solo_enc->osd_text,
						   OSD_TEXT_MAX);
				mutex_unlock(&solo_enc->osd_mutex);
			}
			break;
#endif
		default:
			err = -EINVAL;
		}

		if (err < 0) {
			ctrls->error_idx = i;
			return err;
		}
	}

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static const struct v4l2_file_operations solo_enc_fops = {
#else
static const struct file_operations solo_enc_fops = {
#endif
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
	/* Frame size and interval */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)
	.vidioc_enum_framesizes		= solo_enum_framesizes,
	.vidioc_enum_frameintervals	= solo_enum_frameintervals,
#endif
	/* Video capture parameters */
	.vidioc_s_parm			= solo_s_parm,
	.vidioc_g_parm			= solo_g_parm,
	/* Controls */
	.vidioc_queryctrl		= solo_queryctrl,
	.vidioc_querymenu		= solo_querymenu,
	.vidioc_g_ctrl			= solo_g_ctrl,
	.vidioc_s_ctrl			= solo_s_ctrl,
	.vidioc_g_ext_ctrls		= solo_g_ext_ctrls,
	.vidioc_s_ext_ctrls		= solo_s_ext_ctrls,
};

static struct video_device solo_enc_template = {
	.name			= SOLO6010_NAME,
	.fops			= &solo_enc_fops,
	.ioctl_ops		= &solo_enc_ioctl_ops,
	.minor			= -1,
	.release		= video_device_release,

	.tvnorms		= V4L2_STD_NTSC_M | V4L2_STD_PAL_B,
	.current_norm		= V4L2_STD_NTSC_M,
};

static struct solo_enc_dev *solo_enc_alloc(struct solo6010_dev *solo_dev, u8 ch)
{
	struct solo_enc_dev *solo_enc;
	int ret;

	solo_enc = kzalloc(sizeof(*solo_enc), GFP_KERNEL);
	if (!solo_enc)
		return ERR_PTR(-ENOMEM);

	solo_enc->osd_buf = kzalloc(SOLO_EOSD_EXT_SIZE, GFP_KERNEL);
	if (!solo_enc->osd_buf) {
		kfree(solo_enc);
		return ERR_PTR(-ENOMEM);
	}

	solo_enc->vfd = video_device_alloc();
	if (!solo_enc->vfd) {
		kfree(solo_enc->osd_buf);
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
		kfree(solo_enc->osd_buf);
		kfree(solo_enc);
		return ERR_PTR(ret);
	}

	video_set_drvdata(solo_enc->vfd, solo_enc);

	snprintf(solo_enc->vfd->name, sizeof(solo_enc->vfd->name),
		 "%s-enc (%i/%i)", SOLO6010_NAME, solo_dev->vfd->num,
		 solo_enc->vfd->num);

	if (video_nr >= 0)
		video_nr++;

	spin_lock_init(&solo_enc->enable_lock);
	spin_lock_init(&solo_enc->av_lock);
	spin_lock_init(&solo_enc->motion_lock);

	init_waitqueue_head(&solo_enc->thread_wait);
	atomic_set(&solo_enc->readers, 0);
	atomic_set(&solo_enc->mpeg_readers, 0);
	mutex_init(&solo_enc->osd_mutex);

	solo_enc->qp = SOLO_DEFAULT_QP;
        solo_enc->gop = solo_dev->fps;
	solo_enc->interval = 1;
	solo_enc->mode = SOLO_ENC_MODE_CIF;
	solo_enc->motion_thresh = SOLO_DEF_MOT_THRESH;

	spin_lock(&solo_enc->enable_lock);
	solo_update_mode(solo_enc);
	spin_unlock(&solo_enc->enable_lock);

	return solo_enc;
}

static void solo_enc_free(struct solo_enc_dev *solo_enc)
{
	if (solo_enc == NULL)
		return;

	video_unregister_device(solo_enc->vfd);
	kfree(solo_enc->osd_buf);
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

	/* D1@MAX-FPS * 4 */
	solo_dev->enc_bw_remain = solo_dev->fps * 4 * 4;

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
