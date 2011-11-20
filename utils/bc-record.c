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

#include <linux/videodev2.h>
#include <libavformat/avformat.h>

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
	struct v4l2_control vc;

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
	vparm.parm.capture.timeperframe.numerator = 2;
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

static void av_prepare(void)
{
	AVCodec *codec;

	/* Get the output format */
	fmt_out = guess_format(NULL, outfile, NULL);
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

	if (strstr(vcap.card, "Softlogic 6010")) {
		video_st->codec->codec_id = CODEC_ID_MPEG4;
	} else if (strstr(vcap.card, "Softlogic 6110")) {
		video_st->codec->codec_id = CODEC_ID_H264;
		video_st->codec->crf = 20;
		video_st->codec->me_range = 16;
		video_st->codec->me_subpel_quality = 7;
		video_st->codec->qmin = 10;
		video_st->codec->qmax = 51;
		video_st->codec->max_qdiff = 4;
		video_st->codec->qcompress = 0.6;
		video_st->codec->i_quant_factor = 0.71;
		video_st->codec->b_frame_strategy = 1;
	} else {
		err_out("Unknown card: %s\n", vcap.card);
	}

	video_st->codec->codec_type = CODEC_TYPE_VIDEO;
	video_st->codec->pix_fmt = PIX_FMT_YUV420P;
	video_st->codec->width = vfmt.fmt.pix.width;
	video_st->codec->height = vfmt.fmt.pix.height;
	video_st->codec->time_base = video_st->time_base;

	if (oc->oformat->flags & AVFMT_GLOBALHEADER)
		video_st->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

	if (av_set_parameters(oc, NULL) < 0)
		err_out("Error setting av parameters");

	/* Open Video output */
	codec = avcodec_find_encoder(video_st->codec->codec_id);
	if (codec == NULL)
		err_out("Error finding video encoder");

	if (avcodec_open(video_st->codec, codec) < 0)
		err_out("Error opening video encoder");

	/* Open output file */
	if (url_fopen(&oc->pb, outfile, URL_WRONLY) < 0)
		err_out("Error opening out file");

	av_write_header(oc);
}

static void video_out(struct v4l2_buffer *vb)
{
	AVCodecContext *c = video_st->codec;
	AVPacket pkt;

	av_init_packet(&pkt);

	if (vb->flags & V4L2_BUF_FLAG_KEYFRAME)
		pkt.flags |= PKT_FLAG_KEY;

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
	set_osd("Testing: %s", argv[1]);

	/* Loop to capture video */
	for (;;) {
		struct v4l2_buffer vb;
		int ret;

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
