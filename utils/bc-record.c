/*
 * Copyright (C) 2011 Bluecherry, LLC www.bluecherrydvr.com
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

#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

#include <linux/videodev2.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavformat/avio.h>

#define reset_vbuf(__vb) do {				\
	memset((__vb), 0, sizeof(*(__vb)));		\
	(__vb)->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	\
	(__vb)->memory = V4L2_MEMORY_MMAP;		\
} while(0)

#define err_out(__msg, args...) do {			\
	fprintf(stderr, __msg ": %m\n", ## args);	\
	exit(1);					\
} while(0)

static char outfile[256] = "test.mkv";
static struct v4l2_format vfmt;
static struct v4l2_capability vcap;
static struct v4l2_streamparm vparm;
static int vfd;

#define V_BUFFERS	8
static struct {
	void *data;
	size_t size;
} p_buf[V_BUFFERS];

static AVOutputFormat *fmt_out;
static AVStream *video_st;
static AVFormatContext *oc;

extern char *__progname;

static void usage(void)
{
	fprintf(stderr, "Usage: %s <viddev> <outfile>\n", __progname);
	exit(1);
}

static void open_video_dev(const char *dev)
{
	if ((vfd = open(dev, O_RDWR)) < 0)
		err_out("Opening video device");

	/* Verify this is the correct type */
	if (ioctl(vfd, VIDIOC_QUERYCAP, &vcap) < 0)
		err_out("Querying video capabilities");

	if (!(vcap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
	    !(vcap.capabilities & V4L2_CAP_STREAMING))
		err_out("Invalid video device type");

	/* Get the parameters */
	vparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vfd, VIDIOC_G_PARM, &vparm) < 0)
		err_out("Getting parameters for video device");

	/* Get the format */
	vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(vfd, VIDIOC_G_FMT, &vfmt) < 0)
		err_out("Getting video device format");

	/* Set FPS/GOP */
	vparm.parm.capture.timeperframe.denominator = 30;
	vparm.parm.capture.timeperframe.numerator = 1;
	ioctl(vfd, VIDIOC_S_PARM, &vparm);

	/* Set format */
	vfmt.fmt.pix.width = 704;
	vfmt.fmt.pix.height = 480;
	if (ioctl(vfd, VIDIOC_S_FMT, &vfmt) < 0)
		err_out("Setting video format");
}

static void set_osd(char *fmt, ...)
{
	char buf[256];
	va_list ap;
	struct v4l2_ext_control ctrl;
	struct v4l2_ext_controls ctrls;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	buf[sizeof(buf) - 1] = '\0';

	memset(&ctrl, 0, sizeof(ctrl));
	memset(&ctrls, 0, sizeof(ctrls));

	ctrls.count = 1;
	ctrls.ctrl_class = V4L2_CTRL_CLASS_FM_TX;
	ctrls.controls = &ctrl;
	ctrl.id = V4L2_CID_RDS_TX_RADIO_TEXT;
	ctrl.size = strlen(buf);
	ctrl.string = buf;

	ioctl(vfd, VIDIOC_S_EXT_CTRLS, &ctrls);

        return;
}

static void v4l_prepare(void)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	struct v4l2_requestbuffers req;
	int i;

	reset_vbuf(&req);
	req.count = V_BUFFERS;

	if (ioctl(vfd, VIDIOC_REQBUFS, &req) < 0)
		err_out("Requesting buffers");

	if (req.count != V_BUFFERS)
		err_out("Requested buffer count mismatch");

	for (i = 0; i < V_BUFFERS; i++) {
		struct v4l2_buffer vb;

		reset_vbuf(&vb);
		vb.index = i;

		if (ioctl(vfd, VIDIOC_QUERYBUF, &vb) < 0)
			err_out("Querying buffer");

		p_buf[i].size = vb.length;
		p_buf[i].data = mmap(NULL, vb.length,
				     PROT_WRITE | PROT_READ, MAP_SHARED,
				     vfd, vb.m.offset);
		if (p_buf[i].data == MAP_FAILED)
			err_out("Mmap of buffer");
	}

	if (ioctl(vfd, VIDIOC_STREAMON, &type) < 0)
		err_out("Starting video stream");

	/* Queue all buffers */
	for (i = 0; i < V_BUFFERS; i++) {
		struct v4l2_buffer vb;

		reset_vbuf(&vb);
		vb.index = i;

		if (ioctl(vfd, VIDIOC_QBUF, &vb) < 0)
			err_out("Queuing buffer");
	}
}

static int setup_codec(AVCodecContext *codec, __u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_MPEG4:
		codec->codec_id = CODEC_ID_MPEG4;
		break;

	case V4L2_PIX_FMT_H264:
		codec->codec_id = CODEC_ID_H264;
		/* deprecated: codec->crf = 20; */
		codec->me_range = 16;
		codec->me_subpel_quality = 7;
		codec->qmin = 10;
		codec->qmax = 51;
		codec->max_qdiff = 4;
		codec->qcompress = 0.6;
		codec->i_quant_factor = 0.71;
		codec->b_frame_strategy = 1;
		break;

	case V4L2_PIX_FMT_MJPEG:
		/* FIXME */
	default:
		/* XXX: Older versions of solo6x10 incorrectly report MPEG */
		return 1;
	}

	codec->codec_type = AVMEDIA_TYPE_VIDEO;
	codec->pix_fmt = PIX_FMT_YUV420P;
	return 0;
}

static inline void fourccstr(char s[5], unsigned int val)
{
	s[0] = val & 0xff;
	s[1] = (val >> 8) & 0xff;
	s[2] = (val >> 16) & 0xff;
	s[3] = (val >> 24) & 0xff;
	s[4] = 0;
}

static void av_prepare(void)
{
	AVCodec *codec;

	/* Get the output format */
	fmt_out = av_guess_format(NULL, outfile, NULL);
	if (!fmt_out)
		err_out("Error guessing format for %s", outfile);

	if ((oc = avformat_alloc_context()) == NULL)
		err_out("Error allocating av context");

	oc->oformat = fmt_out;
	snprintf(oc->filename, sizeof(oc->filename), "%s", outfile);

	/* Setup new video stream */
	if ((video_st = av_new_stream(oc, 0)) == NULL)
		err_out("Error creating new av stream");

	video_st->time_base.den =
		vparm.parm.capture.timeperframe.denominator;
	video_st->time_base.num =
		vparm.parm.capture.timeperframe.numerator;

	video_st->codec->width = vfmt.fmt.pix.width;
	video_st->codec->height = vfmt.fmt.pix.height;
	video_st->codec->time_base = video_st->time_base;

	int ret = setup_codec(video_st->codec, vfmt.fmt.pix.pixelformat);
	if (ret) {
		char fourcc[5];
		fourccstr(fourcc, vfmt.fmt.pix.pixelformat);
		err_out("Unknown '%s' format on %s\n", fourcc, vcap.card);
	}

	if (oc->oformat->flags & AVFMT_GLOBALHEADER)
		video_st->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

	/* Open Video output */
	codec = avcodec_find_encoder(video_st->codec->codec_id);
	if (codec == NULL)
		err_out("Error finding video encoder");

	if (avcodec_open(video_st->codec, codec) < 0)
		err_out("Error opening video encoder");

	/* Open output file */
	if (avio_open(&oc->pb, outfile, AVIO_FLAG_WRITE) < 0)
		err_out("Error opening out file");

	avformat_write_header(oc, NULL);
}

static void video_out(struct v4l2_buffer *vb)
{
	AVCodecContext *c = video_st->codec;
	AVPacket pkt;

	av_init_packet(&pkt);

	if (vb->flags & V4L2_BUF_FLAG_KEYFRAME)
		pkt.flags |= AV_PKT_FLAG_KEY;

	if (vb->bytesused < 100 || vb->bytesused > (128 * 1024))
		err_out("Invalid size: %d\n", vb->bytesused);

	pkt.data = p_buf[vb->index].data;
	pkt.size = vb->bytesused;

	if (c->coded_frame->pts != AV_NOPTS_VALUE)
		pkt.pts = av_rescale_q(c->coded_frame->pts, c->time_base,
				      video_st->time_base);
	pkt.stream_index = video_st->index;

	if (av_write_frame(oc, &pkt))
		err_out("Error writing frame to file");
}

int main(int argc, char **argv)
{
	int got_vop = 0;

        avcodec_init();
        av_register_all();

	if (argc != 2 && argc != 3)
		usage();

	if (argc == 3)
		strcpy(outfile, argv[2]);

	open_video_dev(argv[1]);

	v4l_prepare();
	av_prepare();

	/* Loop to capture video */
	for (;;) {
		struct v4l2_buffer vb;
		int ret;
		time_t tm = time(NULL);
		char *tm_buf = ctime(&tm);

		tm_buf[strlen(tm_buf) - 1] = '\0';
		set_osd("%s", tm_buf);

		reset_vbuf(&vb);
		ret = ioctl(vfd, VIDIOC_DQBUF, &vb);
		if (ret < 0)  {
			fprintf(stderr, "Failure in dqbuf\n");
			ioctl(vfd, VIDIOC_QBUF, &vb);
			continue;
		}

		/* Wait for key frame */
		if (!got_vop) {
			if (!(vb.flags & V4L2_BUF_FLAG_KEYFRAME)) {
				ioctl(vfd, VIDIOC_QBUF, &vb);
				continue;
			}
			got_vop = 1;
		}

		video_out(&vb);

		ioctl(vfd, VIDIOC_QBUF, &vb);
	}

	exit(0);
}
